// VioTracker::track (784) orchestration, S4 §3.1 (world SLAM: VPS and image targets inactive).

#include "vio_tracker/vio_tracker.h"

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstdint>
#include <map>
#include <memory>
#include <utility>

#include "c8/geometry/device-pose.h"
#include "c8/geometry/egomotion.h"
#include "common/check.h"
#include "common/stage_timer.h"
#include "config/accuracy.h"
#include "config/settings.h"
#include "map/world_map.h"
#include "reality/engine/features/detection-config.h"
#include "reality/engine/features/image-descriptor.h"
#include "slam_refact/replay_diagnostics.h"
#include "vio_tracker/vio_tracker_internal.h"

namespace slam_refact::vio_tracker {
namespace {

double MsSince(std::chrono::steady_clock::time_point start) {
  return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
}

// g+3080 DetectionConfig of 784 (D:82449-82481).
c8::DetectionConfig TrackingDetectionConfig(bool has_image_targets) {
  c8::DetectionConfig cfg;
  cfg.nKeypoints = 500;
  // Active WorldMap descriptor type (D:82452-82455): derived from two map vectors that only cloud
  // (VPS) maps fill; world SLAM maps leave both empty -> 0 (INFERRED, CloudEmpty()).
  int dt = 0;
  if (dt == 0) {
    dt = config::VioTrackerSettings::Get().tracking_descriptor_type;
  }
  if (dt == c8::DescriptorType::ORB) {
    cfg.allOrb = true;
  } else if (dt == c8::DescriptorType::GORB) {
    cfg.allGorb = true;
  } else {
    SR_CHECK(false, "VioTracker: unsupported tracking descriptor type");  // C8_THROW (D:82466)
  }
  // FULL-3: image targets loaded -> every keypoint gets ORB and GORB (D:82467-82468, g[1544] = 257;
  // SOURCE PUB:reality/engine/tracking/tracker.cc:105-109).
  if (has_image_targets || diagnostics::ReplayForceAllOrbGorb()) {  // the hook: OR-4 decomposition run
    cfg.allOrb = true;
    cfg.allGorb = true;
  }
  // increase_frame_points_ = 0 (VPS only), cloud empty: maxFeatures stays 500.
  cfg.gravityPortions[c8::DescriptorType::ORB] =
      static_cast<c8::GravityPortion>(c8::GravityPortion::SEVENTH | c8::GravityPortion::EIGHTH);
  return cfg;
}

// US-015: 994's map side (spec/11 section 9.2): the copy of MapTrackerOutput.tracked_ that 784
// passes (D:83453-83652, D:84154) and WorldMap::getPoint (634) on the active map. Id 0 is no point
// (994 G:1363).
class MapTrackedPointLookup final : public image_target::TrackedPointLookup {
 public:
  MapTrackedPointLookup(const std::map<uint32_t, map::PointId>& tracked, const map::WorldMap* wm)
      : tracked_(tracked), wm_(wm) {}

  bool Find(uint32_t frame_idx, image_target::TrackedMapPoint* out) const override {
    if (wm_ == nullptr) {
      return false;
    }
    const auto it = tracked_.find(frame_idx);
    if (it == tracked_.end() || it->second.id() == 0) {
      return false;
    }
    const map::WorldPoint* p = wm_->FindPoint(it->second);
    if (p == nullptr) {
      return false;
    }
    out->slot = static_cast<uint32_t>(it->second.index());
    out->id = p->id;
    out->position = p->position;
    out->source = static_cast<int32_t>(p->source);
    return true;
  }

 private:
  const std::map<uint32_t, map::PointId>& tracked_;
  const map::WorldMap* wm_;
};

}  // namespace

VioTracker::Impl::Impl(bool estimate_scale_in, bool disable_vio_in,
                       std::unique_ptr<TrackerModules> modules_in)
    : modules(std::move(modules_in)),
      disable_vio(disable_vio_in),
      estimate_scale(estimate_scale_in),
      mse(config::MetricScaleEstimatorSettings::Get().movement_check_period_in_sec) {
  // 666 copies estimateScale to +5 and +6376 (scale pending until the first latch).
  flags.scale_pending = estimate_scale_in;
  // hist_[1] initial value (485): pose I, v = a = 0, ts = 0, xr quaternion identity.
}

VioTracker::VioTracker(bool estimate_scale, bool disable_vio, std::unique_ptr<TrackerModules> modules)
    : impl_(std::make_unique<Impl>(estimate_scale, disable_vio, std::move(modules))) {
  SR_CHECK(impl_->modules != nullptr, "VioTracker: modules missing");
}

std::unique_ptr<VioTracker> NewVioTrackerWithModules(bool estimate_scale, bool disable_vio,
                                                     std::unique_ptr<TrackerModules> modules) {
  return std::unique_ptr<VioTracker>(new VioTracker(estimate_scale, disable_vio, std::move(modules)));
}

VioTracker::~VioTracker() = default;

c8::HMatrix VioTracker::Track(const TrackingSensorFrame& frame, const c8::Gr8Pyramid& pyr,
                              compat::RandomNumbers* rng, StatusOut* status, TrackDebug* debug) {
  static const DetectionImagePtrMap kNoTargets;
  return TrackWithTargets(frame, pyr, kNoTargets, rng, status, debug);
}

c8::HMatrix VioTracker::TrackWithTargets(const TrackingSensorFrame& frame,
                                         const c8::Gr8Pyramid& pyr,
                                         const DetectionImagePtrMap& targets,
                                         compat::RandomNumbers* rng, StatusOut* status,
                                         TrackDebug* debug) {
  Impl& s = *impl_;
  StageTimings timings{};

  // A. current frame record (D:80005-80065).
  if (s.first_frame_time_ns == 0) {
    s.first_frame_time_ns = frame.time_ns;
  }
  s.now.time_ns = frame.time_ns;
  s.now.xr_device_rotation = c8::xrRotationFromDeviceRotation(frame.device_pose);
  s.now.sensor_events = frame.sensor_events;

  // B. MotionPrediction (D:80092-80332).
  s.prediction = PredictMotion(s.hist[1], s.now);
  const AccuracyConfig& acc = config::Accuracy();
  const bool vi_on = acc.enabled && acc.vi_prediction && !s.disable_vio;
  if (vi_on) {  // ACC-5: IMU-propagated position replaces the constant-velocity one
    s.vi.Predict(s.hist[1], s.now.time_ns, s.now.xr_device_rotation, s.now.sensor_events,
                 frame.metric_scale_hint, &s.prediction);
  }

  // C. VPS blocks: inactive in world SLAM (vps_mode_ = 0, S4 §7.5); not modeled.

  // D. TrackerInput + detection (D:82324-82973).
  // The original allocates a new TrackerInput per frame. Here the previous one is reset to the
  // state of a freshly constructed one (S2 review LOW-7): clearAndClone from an empty frame copies
  // every FrameWithPoints member (points, pixels, store, map ties, intrinsic/K, extrinsic, xr pose,
  // ROI, excluded edge pixels) and keeps only vector capacity. Nothing holds the previous input
  // across frames (MapTracker/MapBuilder clone what they keep).
  if (s.input == nullptr) {
    s.input =
        std::make_unique<TrackerInput>(frame.intrinsic, s.now.xr_device_rotation, frame.time_ns);
  } else {
    s.input->frame_points.clearAndClone(c8::FrameWithPoints(frame.intrinsic));
    s.input->frame_points.setXRDevicePose(s.now.xr_device_rotation);  // tracker-input.h:14-16
    s.input->roi_points.clear();
    s.input->pose = s.now.xr_device_rotation;
    s.input->time_ns = frame.time_ns;
  }
  // The srcY CPU-detector branch (empty pyramid) is not part of world SLAM (S4 §3.1 D); the engine
  // always passes a level pyramid. ROIs come only with image targets (FULL-3).
  SR_CHECK(!pyr.levels.empty(), "VioTracker: pyramid without levels (srcY path unsupported)");
  {
    const auto t0 = std::chrono::steady_clock::now();
    StageTimer timer("features");
    const c8::DetectionConfig cfg = TrackingDetectionConfig(!targets.empty());
    s.modules->Detect(pyr, cfg, &s.input->frame_points);
    if (!pyr.rois.empty()) {  // "gr8gl-rois" (D:82535-82807): every ROI of the pyramid
      s.modules->DetectRois(pyr, cfg, s.input->frame_points, &s.input->roi_points);
    }
    timings.frontend_ms = MsSince(t0);
  }

  // E/F. tracking and map building (D:82978-84145).
  bool ok;
  if (s.disable_vio) {
    s.tam_extrinsic = c8::cameraMotion(c8::HPoint3(0.0f, 0.0f, 0.0f), s.now.xr_device_rotation);
    ok = false;
  } else {
    {
      const auto t0 = std::chrono::steady_clock::now();
      s.modules->TrackMap(s.input.get(), s.prediction, &s.mt_out, rng);
      timings.map_tracker_ms = MsSince(t0);
    }
    if (!config::VioTrackerSettings::Get().disable_map_builder) {
      const auto t0 = std::chrono::steady_clock::now();
      const int64_t frame_ms = static_cast<int64_t>(static_cast<double>(frame.time_ns) * 1e-6);
      s.modules->UpdateMap(*s.input, s.mt_out, frame_ms, &s.mb_out);
      timings.map_builder_ms = MsSince(t0);
    }
    s.tam_extrinsic = s.modules->ApplyMapBuilderOutput(s.mb_out);  // 664
    ok = s.mt_out.state == 1;
  }

  // G. image targets (938, D:84147-84938), also with disableVio (3DoF tam): skipped when no target
  // is loaded (eqz(d[2])), which keeps the previous locatedImages_ (SOURCE tracker.cc:127-131).
  // Public DetectionImageTracker::locate; the original call has two more arguments (the active
  // WorldMap b+7232 and !disableVio, D:84165) used by its closed scale/world tracking (US-015).
  // That tracker (994) runs when diagnostics::ImageWorldTrackerEnabled (FULL-12; on by default,
  // WT-DEFAULT-ON) and VIO is on; with SetImageWorldTracker(false) the vendored public behavior is unchanged.
  if (!targets.empty()) {
    StageTimer timer("image-locate");
    const bool world_tracker = diagnostics::ImageWorldTrackerEnabled() && !s.disable_vio;
    const MapTrackedPointLookup lookup(s.mt_out.tracked, s.modules->active_world_map());
    s.image_world_tracker.set_lookup(&lookup);
    s.image_tracker.setWorldTracker(world_tracker ? &s.image_world_tracker : nullptr);
    s.located = s.image_tracker.locate(s.input->frame_points, s.input->roi_points,
                                       s.tam_extrinsic, targets, rng);
    s.image_tracker.setWorldTracker(nullptr);
    s.image_world_tracker.set_lookup(nullptr);
    s.located_debug.clear();
    for (const c8::LocatedImage& l : s.located) {
      const auto& name = l.targetSpace.name;
      LocatedImageDebug d;
      for (const c8::FrameWithPoints& f : s.input->roi_points) {  // featsForTrackedImage
        const c8::ImageRoi& roi = f.roi();
        if ((roi.source == c8::ImageRoi::Source::IMAGE_TARGET ||
             roi.source == c8::ImageRoi::Source::CURVY_IMAGE_TARGET) &&
            roi.name == name && !f.points().empty()) {
          d.roi_feats = true;
          break;
        }
      }
      for (uint8_t v : s.image_tracker.inliers(name)) d.local_inliers += v != 0;
      for (uint8_t v : s.image_tracker.globalInliers(name)) d.global_inliers += v != 0;
      s.located_debug.push_back(d);
    }
  }

  // H. pose history and velocity (D:84939-85055).
  UpdateHistory(s.hist, s.tam_extrinsic, s.now,
                s.mb_out.world_transform_updated || s.mb_out.pose_correction_applied);
  if (vi_on) {
    s.vi.Update(s.hist[0], s.hist[1].pose, s.now.time_ns, frame.metric_scale_hint,
                acc.vi_velocity_gain,
                s.mb_out.map_reset || s.mb_out.world_transform_updated ||
                    s.mb_out.pose_correction_applied || s.mt_out.state != 1);
    if (++s.vi_frames % 300 == 0 && std::getenv("SLAM_REFACT_VI_LOG") != nullptr) {
      std::fprintf(stderr, "vi frame %lld init %d gravity angle p50 %.2f deg hint %.3f\n",
                   static_cast<long long>(s.vi_frames), s.vi.initialized(),
                   s.vi.gravity_angle_deg(), frame.metric_scale_hint);
    }
  }

  // I. loss state machine and status (D:85056-85119); vps_mode_ = 0.
  ok = ok || s.mt_out.frame_tracker_ok;
  if (ok && s.flags.wait_first_ok && !s.flags.got_first_ok) {
    s.flags.got_first_ok = true;
  }
  s.loss.Update(ok, frame.time_ns);
  StatusOut st = ComposeStatus(s.flags, s.loss.state());

  // J. yaw offset, once per VioTracker (D:85120-85200).
  if (!s.yaw_offset_initialized) {
    s.yaw_offset = YawOffsetFromPose(s.tam_extrinsic);
    s.yaw_offset_initialized = true;
  }

  // L. output (D:85344-86342).
  c8::HMatrix result = c8::HMatrixGen::i();
  if (!s.estimate_scale || s.disable_vio) {
    result = s.yaw_offset * s.tam_extrinsic;
  } else {
    const auto t0 = std::chrono::steady_clock::now();
    if (s.mb_out.map_reset) {  // D:85651-85667
      s.scale_latched = false;
      s.latched_scale = 0.0f;
      s.metric_transform = c8::HMatrixGen::i();
    }
    const bool ready = s.mse.PushFrame(frame, s.tam_extrinsic, frame.time_ns, s.mt_out.state,
                                       s.mb_out.map_reset);  // 944
    if (ready && !s.scale_latched) {  // Q11: latch once
      s.scale_latched = true;
      s.latched_scale = s.mse.scale();
      s.yaw_offset = YawOffsetFromPose(s.tam_extrinsic);
      s.metric_transform = MetricTransform(s.latched_scale, s.yaw_offset, s.tam_extrinsic);
    }
    s.flags.scale_ready = s.scale_latched;
    st = ComposeStatus(s.flags, s.loss.state());  // D:85912-85932
    result = s.scale_latched ? MetricOutput(s.metric_transform, s.tam_extrinsic)
                             : s.yaw_offset * s.tam_extrinsic;
    timings.metric_scale_ms = MsSince(t0);
  }

  // applyToLocatedImages (S4 §3.9): the transform of the returned pose on every LocatedImage.
  if (!targets.empty()) {
    const bool metric = s.estimate_scale && !s.disable_vio && s.scale_latched;
    s.located_out = s.located;
    for (c8::LocatedImage& l : s.located_out) {
      l.pose = metric ? MetricOutput(s.metric_transform, l.pose) : s.yaw_offset * l.pose;
    }
  }

  if (status != nullptr) {
    *status = st;
  }
  if (debug != nullptr) {
    debug->predicted_pose = s.prediction.pose;
    debug->map_tracker_state = s.mt_out.state;
    debug->frame_tracker_ok = s.mt_out.frame_tracker_ok;
    debug->ok = s.loss.last_ok();
    debug->loss_state = s.loss.state();
    debug->lost_frames = s.loss.lost_frames();
    debug->frames = s.loss.frames();
    debug->yaw_offset_initialized = s.yaw_offset_initialized;
    debug->map_reset = s.mb_out.map_reset;
    debug->keyframe_created = s.mb_out.keyframe_created;
    debug->scale_latched = s.scale_latched;
    debug->latched_scale = s.latched_scale;
    const frontend::DetectStats ds = s.modules->detect_stats();
    debug->num_keypoints = ds.num_keypoints;
    debug->num_orb = ds.num_orb;
    debug->num_gorb = ds.num_gorb;
    debug->num_keyframes = s.modules->num_keyframes();
    debug->num_points_by_source = s.modules->num_points_by_source();
    debug->ground_matches = s.modules->ground_match_count();
    debug->timings = timings;
  }
  return result;
}

void VioTracker::GetWorldPoints(std::vector<WorldPointOut>* out) const {
  const Impl& s = *impl_;
  const c8::HMatrix& t = s.estimate_scale ? s.metric_transform : s.yaw_offset;
  StageTimer timer("map-tracker-get-world-points");
  std::vector<VisibleMapPoint> vis;
  s.modules->VisibleMapPoints(&vis);
  const auto passes = [](const VisibleMapPoint& r) {
    return r.state != map::PointState::kDead && r.source == map::PointSource::kTriangulated;
  };
  int count = static_cast<int>(vis.size());
  uint32_t stride = 0;
  // A negative limit never ends the stride search, and 0 does not either while a passing point
  // has key_id 0 (S2 review LOW-2).
  SR_CHECK(config::MapTrackerSettings::Get().max_points >= 1, "MapTracker.maxPoints must be >= 1");
  while (config::MapTrackerSettings::Get().max_points < count) {
    ++stride;
    count = 0;
    for (const VisibleMapPoint& r : vis) {
      if (passes(r) && r.key_id % stride == 0) {
        ++count;
      }
    }
  }
  // Q10: sized by the unfiltered count when stride == 0; unused entries stay id 0, (0,0,0), 0.
  out->assign(static_cast<size_t>(count), WorldPointOut{0, {0.f, 0.f, 0.f}, 0.f});
  const std::array<float, 16>& m = t.data();
  size_t k = 0;
  for (const VisibleMapPoint& r : vis) {
    if (!((stride == 0 || r.key_id % stride == 0) && passes(r))) {
      continue;
    }
    const std::array<float, 4>& p = r.position.raw();
    // f631 wat (f32x4 lanes): col3*p3 + (col2*p2 + (col0*p0 + p1*col1)); Ghidra G631 prints the
    // nested sum flat.
    const float w = m[15] * p[3] + (m[11] * p[2] + (m[3] * p[0] + p[1] * m[7]));
    WorldPointOut& o = (*out)[k];
    o.id = r.point_id;
    o.position.x = (m[12] * p[3] + (m[8] * p[2] + (m[0] * p[0] + p[1] * m[4]))) / w;
    o.position.y = (m[13] * p[3] + (m[9] * p[2] + (m[1] * p[0] + p[1] * m[5]))) / w;
    o.position.z = (m[14] * p[3] + (m[10] * p[2] + (m[2] * p[0] + p[1] * m[6]))) / w;
    o.confidence = r.tracking_count == 0
                       ? 0.0f
                       : static_cast<float>(r.tracking_hits) / static_cast<float>(r.tracking_count);
    ++k;
  }
}

void VioTracker::SetDisableVio(bool on) {
  impl_->disable_vio = on;
  if (on) {  // 476: only turning it on recreates MapTracker (+ FFT RNG) and MapBuilder
    impl_->modules->RecreateMapTrackerAndBuilder();
  }
}

const c8::HMatrix& VioTracker::tam_extrinsic() const { return impl_->tam_extrinsic; }
const FrameState& VioTracker::last_prediction() const { return impl_->prediction; }
const LossStateMachine& VioTracker::loss_state() const { return impl_->loss; }
bool VioTracker::yaw_offset_initialized() const { return impl_->yaw_offset_initialized; }
bool VioTracker::scale_latched() const { return impl_->scale_latched; }
const std::vector<c8::LocatedImage>& VioTracker::located_images() const {
  return impl_->located_out;
}
const std::vector<LocatedImageDebug>& VioTracker::located_images_debug() const {
  return impl_->located_debug;
}

void VioTracker::ResetImageTracking() {
  impl_->image_tracker.setTrackingDisabled(true);  // tracker.h:45-50: also clears locatedImages_
  impl_->located.clear();
  impl_->located_out.clear();
  impl_->located_debug.clear();
  impl_->image_tracker.setTrackingDisabled(false);
}
float VioTracker::latched_scale() const { return impl_->latched_scale; }

}  // namespace slam_refact::vio_tracker

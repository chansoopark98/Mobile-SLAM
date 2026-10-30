// Engine: the tracking core (architecture.md §3.9, §4.1-§4.3, D-13, D-14).
//
// Construction = configureXr + initDeviceInfo + engineInit: device intrinsics (K rule, S1 §5.12),
// the pyramid source (engineInit size, S1 §2.2), the XREngine RandomNumbers (seed 5489) and a new
// VioTracker (485 initial values). Track = 784 (VioTracker::Track) + 631 (GetWorldPoints) on one
// staged frame whose events are already post-647 (D-14). No 647, no pipeline, no 733/704
// post-processing here: those belong to GlueRunner.

#include "slam_refact/engine.h"

#include <cctype>
#include <chrono>
#include <cstring>
#include <memory>
#include <optional>
#include <string>
#include <utility>

#include "c8/pixels/gr8-pyramid.h"
#include "common/c8_adapters.h"
#include "common/check.h"
#include "compat/random_numbers.h"
#include "config/accuracy.h"
#include "config/params.h"
#include "engine/device_orientation.h"
#include "engine/device_pose.h"
#include "engine/lighting.h"
#include "frontend/frontend.h"
#include "frontend/intrinsics.h"
#include "frontend/pyramid_source.h"
#include "frontend/sensor_intake.h"
#include "image_target/image_roi_list.h"
#include "image_target/image_target_set.h"
#include "slam_refact/replay_diagnostics.h"
#include "vio_tracker/vio_tracker.h"

namespace slam_refact {
namespace {

std::string Upper(const std::string& s) {
  std::string out = s;
  for (char& c : out) {
    c = static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
  }
  return out;
}

// R16 (lifted by FULL-8, US-023): any capture size and the rotations the original maps to a device
// orientation (0, 90, 180, 270 and their negatives, tracking-controller.cc:152-167). The tracker works
// in the portrait frame: the pyramid renderer turns the image by `rotation_deg`, a landscape capture
// (width > height) is exchanged to portrait for the K rule and the pyramid layout (engineInit).
void CheckGeometry(const EngineConfig& cfg) {
  SR_CHECK(IsSupportedRotation(cfg.rotation_deg),
           "Engine: rotation_deg must be 0, 90, 180, 270 or -90, -180, -270 (rotationToDeviceOrientation)");
  SR_CHECK(cfg.capture_width > 0 && cfg.capture_height > 0, "Engine: capture size missing");
}

bool SameBits(float a, float b) { return std::memcmp(&a, &b, sizeof(float)) == 0; }

// "Unchanged" test for the intrinsics override: same presence and bit-identical fields (so the
// exact value given at construction, NaN included, is accepted again; any other value is not).
bool SameIntrinsicsOverride(const std::optional<PinholeIntrinsics>& a,
                            const std::optional<PinholeIntrinsics>& b) {
  if (a.has_value() != b.has_value()) {
    return false;
  }
  if (!a.has_value()) {
    return true;
  }
  return a->width == b->width && a->height == b->height && SameBits(a->cx, b->cx) &&
         SameBits(a->cy, b->cy) && SameBits(a->fx, b->fx) && SameBits(a->fy, b->fy);
}

double MsSince(std::chrono::steady_clock::time_point start) {
  return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
}

}  // namespace

class Engine::Impl {
 public:
  explicit Impl(const EngineConfig& cfg)
      : cfg_(cfg),
        pyramid_(cfg.capture_width, cfg.capture_height, frontend::ShaderParams{},
                 cfg.pyramid_model, cfg.rotation_deg),
        vio_(cfg.estimate_scale, cfg.disable_vio) {
    // tracking-sensor-event.cc:84 isIos (S1 U10, INFERRED); deviceModel = getDeviceModel (frame+0).
    is_ios_ = Upper(cfg.manufacturer) == "APPLE";
    device_model_ = static_cast<int32_t>(frontend::GetDeviceModel(cfg.manufacturer, cfg.model));
    // processing = cropAndScale(capture, level-0 size) (S1 §5.12, U7 INFERRED).
    const frontend::LevelLayout& l0 = pyramid_.layout().levels[0];
    // Accuracy mode (ACC-4): a calibrated web-stream focal for known devices replaces the table.
    const float accuracy_focal = cfg.accuracy.enabled
                                     ? frontend::AccuracyDeviceFocal(cfg.manufacturer, cfg.model)
                                     : 0.0f;
    const PinholeIntrinsics k =
        cfg.intrinsics_override.has_value()
            ? *cfg.intrinsics_override
            : accuracy_focal > 0.0f
                  ? frontend::ComputeIntrinsicsSetWithFocal(accuracy_focal, cfg.capture_width,
                                                            cfg.capture_height, l0.w, l0.h)
                        .processing
                  : frontend::ComputeIntrinsicsSet(cfg.manufacturer, cfg.model, cfg.capture_width,
                                                   cfg.capture_height, l0.w, l0.h)
                        .processing;
    intrinsic_ = ToC8Intrinsics(k);
  }

  void Configure(const EngineConfig& cfg) {
    // Only the configure fields of a running XREngine are accepted; geometry, device and
    // estimateScale are fixed at construction (a change needs a new Engine, like engineInit).
    SR_CHECK(cfg.estimate_scale == cfg_.estimate_scale, "Engine::Configure: estimate_scale is fixed");
    SR_CHECK(cfg.capture_width == cfg_.capture_width && cfg.capture_height == cfg_.capture_height &&
                 cfg.rotation_deg == cfg_.rotation_deg,
             "Engine::Configure: capture geometry is fixed (engineInit)");
    SR_CHECK(cfg.manufacturer == cfg_.manufacturer && cfg.model == cfg_.model,
             "Engine::Configure: device is fixed (initDeviceInfo)");
    SR_CHECK(cfg.pyramid_model == cfg_.pyramid_model, "Engine::Configure: pyramid model is fixed");
    // The intrinsics are computed once at construction (initDeviceInfo); accepting a different
    // override here would update cfg_ without changing the K the tracker uses (S2 review LOW-1).
    SR_CHECK(SameIntrinsicsOverride(cfg.intrinsics_override, cfg_.intrinsics_override),
             "Engine::Configure: intrinsics_override is fixed (initDeviceInfo)");
    if (!cfg.param_overrides.empty()) {
      config::ApplyParamOverrides(cfg.param_overrides);
    }
    // 476: setting disableVio on recreates MapTracker + MapBuilder; turning it off does not.
    vio_.SetDisableVio(cfg.disable_vio);
    cfg_ = cfg;
  }

  TrackerStep Track(const FrameInput& in) {
    // D-14 runtime guard: post-647 kind 2 (MAGNETOMETER) and time-reversed events abort.
    frontend::ValidatePostPrepareEvents(in.imu_events);

    // 647 output already computed by SensorIntake; build TrackingSensorFrame only.
    TrackingSensorFrame frame;
    frame.device_model = device_model_;
    frame.time_ns = in.time_ns;
    frame.is_ios = is_ios_;
    frame.intrinsic = intrinsic_;
    frame.device_pose = TrackingDevicePose(in.device_pose);  // 647 (device_pose.h)
    frame.sensor_events = in.imu_events;
    frame.metric_scale_hint = in.metric_scale_hint;

    // FULL-3: clearDetectionImages resets the image tracking state of the running XREngine.
    const ImageTargetSet* targets = in.image_targets;
    if (targets != nullptr && targets->clear_generation() != image_clear_generation_) {
      vio_.ResetImageTracking();
      image_clear_generation_ = targets->clear_generation();
    }
    const bool has_targets = targets != nullptr && !targets->empty();

    // Pyramid of this frame (GPU pyramid in the original, CPU shader port or injection here),
    // with the ROI slots of this draw when image targets requested them (FULL-3).
    const auto t0 = std::chrono::steady_clock::now();
    const c8::Gr8Pyramid pyr =
        in.image_rois != nullptr && !in.image_rois->rois.empty()
            ? pyramid_.GetWithRois(in.pyramid, in.rgba, in.gray, in.image_rois->rois)
            : pyramid_.Get(in.pyramid, in.rgba, in.gray);
    const double pyramid_ms = MsSince(t0);

    TrackerStep step;
    vio_tracker::StatusOut status;
    vio_tracker::TrackDebug debug;
    const c8::HMatrix pose =
        has_targets ? vio_.TrackWithTargets(frame, pyr, targets->ptr_map(), &rng_, &status, &debug)
                    : vio_.Track(frame, pyr, &rng_, &status, &debug);  // 784
    vio_.GetWorldPoints(&step.world_points_internal);                         // 631

    step.pose = ToHMatrix4f(pose);
    step.tam_extrinsic = ToHMatrix4f(vio_.tam_extrinsic());
    step.status = status.status;
    step.reason = status.reason;
    step.predicted_pose = ToHMatrix4f(debug.predicted_pose);
    step.map_tracker_state = debug.map_tracker_state;
    step.frame_tracker_ok = debug.frame_tracker_ok;
    step.ok = debug.ok;
    step.loss_state = debug.loss_state;
    step.lost_frames = debug.lost_frames;
    step.frames = debug.frames;
    step.yaw_offset_initialized = debug.yaw_offset_initialized;
    step.map_reset = debug.map_reset;
    step.keyframe_created = debug.keyframe_created;
    step.scale_latched = debug.scale_latched;
    step.latched_scale = debug.latched_scale;
    step.num_keypoints = debug.num_keypoints;
    step.num_orb = debug.num_orb;
    step.num_gorb = debug.num_gorb;
    step.num_keyframes = debug.num_keyframes;
    for (int k = 0; k < 5; ++k) {
      step.num_points_by_source[k] = debug.num_points_by_source[static_cast<size_t>(k)];
    }
    step.ground_matches = debug.ground_matches;
    step.timings = debug.timings;
    step.intrinsics.width = intrinsic_.pixelsWidth;
    step.intrinsics.height = intrinsic_.pixelsHeight;
    step.intrinsics.cx = intrinsic_.centerPointX;
    step.intrinsics.cy = intrinsic_.centerPointY;
    step.intrinsics.fx = intrinsic_.focalLengthHorizontal;
    step.intrinsics.fy = intrinsic_.focalLengthVertical;
    // frontend = pyramid (GPU stage in the original) + detection (784 "features" scope).
    step.timings.frontend_ms += pyramid_ms;
    if (in.estimate_lighting) {
      // FULL-1: LightingRequestExecutor on the pyramid of the tracked frame (spec 06 §4).
      step.has_lighting = true;
      step.lighting_exposure = EstimateExposure(pyr);
    }
    if (has_targets) {
      FillLocatedImages(*targets, &step);
    }
    return step;
  }

 private:
  // FULL-3: Tracker::locatedImageTargets as public records, and the ROIs they request for a later
  // draw (fillDetectedImagesC8, image-detection-request-executor.cc:196-201: lastSeen < 20).
  void FillLocatedImages(const ImageTargetSet& targets, TrackerStep* step) const {
    auto rois = std::make_shared<ImageRoiList>();
    const auto& debug = vio_.located_images_debug();
    size_t index = 0;
    for (const c8::LocatedImage& l : vio_.located_images()) {
      const size_t at = index++;
      if (l.targetSpace.lastSeen < 20) {
        rois->rois.push_back(l.roi);
      }
      const c8::DetectionImage& im = targets.at(l.targetSpace.name);
      LocatedImageOut o;
      o.name = l.targetSpace.name;
      o.id = l.roi.faceId;
      o.pose = ToHMatrix4f(l.pose);
      o.width = l.width;
      o.height = l.height;
      o.type = image_target::ToDetectedImageType(im.getType());
      if (im.getType() == c8::DetectionImageType::CURVY) {
        o.curvy_geometry = image_target::CurvyGeometryOf(im);
      }
      o.status = static_cast<int32_t>(l.targetSpace.status);
      o.last_seen = l.targetSpace.lastSeen;
      o.first_seen = l.targetSpace.firstSeen;
      o.scale = l.targetSpace.scale;
      o.target_view_pose = ToHMatrix4f(l.targetSpace.pose);
      if (at < debug.size()) {
        o.roi_feats = debug[at].roi_feats;
        o.local_inliers = debug[at].local_inliers;
        o.global_inliers = debug[at].global_inliers;
      }
      step->located_images.push_back(std::move(o));
    }
    step->requested_rois = std::move(rois);
  }

  EngineConfig cfg_;
  bool is_ios_ = false;
  int32_t device_model_ = 0;
  c8_PixelPinholeCameraModel intrinsic_{0, 0, 0.f, 0.f, 0.f, 0.f};
  frontend::PyramidSource pyramid_;
  // XREngine RandomNumbers, default seed 5489 (§5.2); the OR-4 replay hook may select another stream.
  compat::RandomNumbers rng_{diagnostics::ReplayEngineRngSeed().value_or(compat::kEngineRngSeed)};
  vio_tracker::VioTracker vio_;
  uint64_t image_clear_generation_ = 0;
};

Engine::Engine(const EngineConfig& cfg) {
  CheckGeometry(cfg);
  // globalParams() is process-global (§4.6): overrides persist across engine rebuilds.
  if (!cfg.param_overrides.empty()) {
    config::ApplyParamOverrides(cfg.param_overrides);
  }
  config::SetAccuracy(cfg.accuracy);
  impl_ = std::make_unique<Impl>(cfg);
}

Engine::~Engine() = default;

void Engine::Configure(const EngineConfig& cfg) { impl_->Configure(cfg); }

TrackerStep Engine::Track(const FrameInput& in) { return impl_->Track(in); }

}  // namespace slam_refact

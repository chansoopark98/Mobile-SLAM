// GlueRunner: the JS / tracking-controller layer around the core (architecture.md §4.3-§4.4, D-3,
// D-4, D-13, D-21).
//
//   ProcessFrame = sensors.setEventQueue (SensorIntake::TakeForStage, the only func-647 caller)
//                  + c8EmAsm_stageFrame + c8EmAsm_processStagedFrame (Publish).
//   Publish      = pipeline push; when pipeline.size() >= 2 the oldest entry is tracked (1-frame
//                  lag), post-processed (733 recenterPose, 704 rewriteCoordinateSystemPostprocess,
//                  JS world point exposure) and popped; startup recenter (framesUntilRecenter = 5)
//                  overwrites the position with the configured origin and rebuilds the core after
//                  the 5th processed frame (rebuildEngine: new core, pipeline.clear(),
//                  frameTick = 0; framesUntilRecenter is never reset).
// SOURCE: PUB:reality/app/xr/js/tracking-controller.cc:134-150, 623-741;
//         PUB:reality/engine/pose/pose-request-executor.cc:278-327 (recenterPose);
//         PUB:reality/engine/geometry/orientation.cc:462-481, 570-595 (704).
// VERIFIED trace behaviour: frames 0-4 at the origin, frame 5 = frame 4 re-published (S4 §3.8).

#include "slam_refact/glue_runner.h"

#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <deque>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "c8/geometry/egomotion.h"
#include "c8/geometry/homography.h"
#include "c8/hmatrix.h"
#include "c8/hpoint.h"
#include "c8/quaternion.h"
#include "common/check.h"
#include "engine/device_orientation.h"
#include "engine/display_intrinsics.h"
#include "engine/floor_detector.h"
#include "engine/hit_test.h"
#include "engine/image_target_glue.h"
#include "engine/lighting.h"
#include "engine/metric_scale_tracker.h"
#include "engine/startup_recenter.h"
#include "frontend/frontend.h"
#include "frontend/roi_draw.h"
#include "image_target/image_roi_list.h"
#include "image_target/image_target_set.h"

namespace slam_refact {
namespace {

// One PipelineData entry. The pixel buffers are owned so the entry outlives the caller's views
// (the original keeps the camera texture alive until the deferred readback).
struct PipelineEntry {
  FrameInput in;
  int64_t stage_index = 0;
  std::vector<uint8_t> gray;
  std::vector<uint8_t> rgba;
  std::vector<uint8_t> atlas;
};

// Copies `rows` rows of `row_bytes` from a strided view into a tight buffer.
std::vector<uint8_t> CopyRows(const uint8_t* data, int row_bytes, int rows, int stride_bytes) {
  std::vector<uint8_t> out(static_cast<size_t>(row_bytes) * static_cast<size_t>(rows));
  for (int r = 0; r < rows; ++r) {
    std::memcpy(out.data() + static_cast<size_t>(r) * static_cast<size_t>(row_bytes),
                data + static_cast<size_t>(r) * static_cast<size_t>(stride_bytes),
                static_cast<size_t>(row_bytes));
  }
  return out;
}

// View validation before the copy (S2 review MEDIUM-2). The frozen views carry no byte length,
// so the caller guarantees `stride * (height - 1) + row bytes` readable bytes; what can be checked
// here is that the geometry is positive and that every row fits in its stride. Violations abort
// like every other engine input error (SR_CHECK, no recoverable error path).
void CheckStridedView(int width, int height, int stride, int bytes_per_pixel, const char* what) {
  SR_CHECK(width > 0 && height > 0, what);
  SR_CHECK(static_cast<int64_t>(stride) >= static_cast<int64_t>(width) * bytes_per_pixel, what);
}

void OwnPixels(PipelineEntry* e) {
  FrameInput& in = e->in;
  if (in.pyramid.rgba != nullptr) {
    SR_CHECK(in.pyramid.size > 0 && in.pyramid.rows > 0,
             "GlueRunner: injected pyramid size and rows must be positive");
    e->atlas.assign(in.pyramid.rgba,
                    in.pyramid.rgba + static_cast<size_t>(in.pyramid.size) * 4u *
                                          static_cast<size_t>(in.pyramid.rows));
    in.pyramid.rgba = e->atlas.data();
  }
  if (in.rgba.data != nullptr) {
    CheckStridedView(in.rgba.width, in.rgba.height, in.rgba.stride, 4,
                     "GlueRunner: rgba view needs width, height > 0 and stride >= 4 * width");
    e->rgba = CopyRows(in.rgba.data, in.rgba.width * 4, in.rgba.height, in.rgba.stride);
    in.rgba.data = e->rgba.data();
    in.rgba.stride = in.rgba.width * 4;
  }
  if (in.gray.data != nullptr) {
    CheckStridedView(in.gray.width, in.gray.height, in.gray.stride, 1,
                     "GlueRunner: gray view needs width, height > 0 and stride >= width");
    e->gray = CopyRows(in.gray.data, in.gray.width, in.gray.height, in.gray.stride);
    in.gray.data = e->gray.data();
    in.gray.stride = in.gray.width;
  }
}

// Forward matrix only (no inverse stored, noInvert): 733 reads fromHMatrix and translation only.
c8::HMatrix FromHMatrix4f(const HMatrix4f& m) { return c8::HMatrix(m.m, nullptr); }

}  // namespace

std::unique_ptr<TrackerCore> DefaultEngineFactory(const EngineConfig& cfg) {
  return std::make_unique<Engine>(cfg);
}

class GlueRunner::Impl {
 public:
  Impl(EngineConfig engine_config, GlueConfig glue_config, TrackerCoreFactory factory)
      : engine_config_(std::move(engine_config)),
        glue_config_(glue_config),
        factory_(std::move(factory)),
        intake_(Upper(engine_config_.manufacturer) == "APPLE") {
    SR_CHECK(static_cast<bool>(factory_), "GlueRunner: factory missing");
    SR_CHECK(glue_config_.frames_until_recenter >= 0,
             "GlueRunner: frames_until_recenter must be >= 0");
    CheckFiniteCoordinates(glue_config_.origin, glue_config_.facing);
    frames_until_recenter_ = glue_config_.frames_until_recenter;
    // JS@5517107 (N4): absolute scale forces origin.y = 1 and scale 1; responsive scale = origin.y.
    // The recenter point p is copied before that override (xA: `p = Object.assign({}, P)` precedes
    // `a ? (P.y = 1, d = 1) : ...`), so recenter() restores the origin the caller gave.
    const Vec3f recenter_origin = glue_config_.origin;
    if (engine_config_.estimate_scale) {
      glue_config_.origin.y = 1.0f;
      scale_ = 1.0f;
    } else {
      scale_ = glue_config_.origin.y;
    }
    core_ = factory_(engine_config_);
    SR_CHECK(core_ != nullptr, "GlueRunner: factory returned no core");
    // FULL-1 (spec 06 §3.2): the construction state equals the original after one
    // updateCameraProjectionMatrix({origin, facing}); world points on (glue default).
    js_.origin = glue_config_.origin;
    js_.facing = glue_config_.facing;
    js_.recenter_origin = recenter_origin;
    js_.recenter_facing = glue_config_.facing;
    js_.scale = scale_;
    js_.absolute = engine_config_.estimate_scale;
    js_.disable_world_tracking = engine_config_.disable_vio;
    js_.left_handed_axes = glue_config_.left_handed_axes;
    js_.mirrored_display = glue_config_.mirrored_display;
    // A zero origin.y gives scale 0, which configureXr does not send (fA: scale is truthy-checked),
    // so coords_ would keep its default (origin (0, 2, 0), right-handed). The glue keeps the
    // construction values instead (the pre-FULL-1 behaviour: user origin, facing, axes and mirror,
    // scale 1 in the 704 chain); the original would use the capnp defaults. A non-finite origin or
    // facing is rejected above (RM-3a2): the original passes NaN / Infinity to capnp unchecked and
    // would publish non-finite poses (or, for a NaN origin.y, silently keep the previous
    // coordinates); no valid pose can come out of it.
    coords_.origin = glue_config_.origin;
    coords_.facing = glue_config_.facing;
    coords_.scale = scale_;
    coords_.left_handed_axes = glue_config_.left_handed_axes;
    coords_.mirrored_display = glue_config_.mirrored_display;
    // Injected cores (tests) may be built without a frame size: no capture K then, and
    // reality.intrinsics stays all zero.
    orientation_ = RotationToDeviceOrientation(engine_config_.rotation_deg);
    if (engine_config_.capture_width > 0 && engine_config_.capture_height > 0) {
      capture_k_ = CaptureIntrinsicsFor(engine_config_);
    }
    ConfigureXr();
    const AccuracyConfig& acc = engine_config_.accuracy;
    metric_enabled_ = acc.enabled && acc.metric_output && !engine_config_.estimate_scale;
    MetricScaleParams mp;
    mp.window_s = acc.metric_window_s;
    mp.prior_scale = acc.metric_prior_scale;
    mp.low_band = acc.metric_low_band;
    mp.fixed_lag_s = acc.metric_fixed_lag_s;
    metric_ = MetricScaleTracker(mp);
    ResetSurfaceState();
  }

  void PushDeviceMotion(const DeviceMotionSample& s) {
    intake_.OnDeviceMotion(s);
    if (metric_enabled_ && s.acceleration_including_gravity.has_value()) {
      const Vec3f& a = *s.acceleration_including_gravity;
      metric_.PushImu(s.receive_time_ns, {a.x, a.y, a.z});
    }
  }
  void PushDeviceOrientation(const DeviceOrientationSample& s) { intake_.OnDeviceOrientation(s); }

  PublishedFrame ProcessFrame(int64_t time_ns, int64_t video_time_ns, const GrayImageView& img,
                              PyramidAtlasView injected) {
    // onProcessGpu: q = getQuaternion(alpha, beta, gamma); setEventQueue(); stageFrame(...).
    frontend::StagedSensors staged = intake_.TakeForStage();
    FrameInput in;
    in.time_ns = time_ns;
    in.video_time_ns = video_time_ns;
    in.gray = img;
    in.pyramid = injected;
    in.device_pose = staged.q;
    in.imu_events = std::move(staged.events);
    // ACC-5: metres per internal unit = metres per published unit * published units per internal
    // unit (responsive scale = origin.y); the metric tracker's prior until it converges.
    const AccuracyConfig& acc = engine_config_.accuracy;
    if (acc.enabled && acc.vi_prediction && !engine_config_.estimate_scale) {
      in.metric_scale_hint = metric_.scale() * static_cast<double>(scale_);
    }
    return Publish(std::move(in));
  }

  PublishedFrame Publish(FrameInput in) {
    started_ = true;
    // c8EmAsm_stageFrame: frameTick++, pipeline.push_back.
    ++frame_tick_;
    const int64_t stage_index = next_stage_index_++;
    pipeline_.emplace_back();
    PipelineEntry& entry = pipeline_.back();
    entry.in = std::move(in);
    entry.stage_index = stage_index;
    OwnPixels(&entry);
    // FULL-3: this draw consumes the ROIs registered so far (drawStage: drawnRois_ = nextRois_,
    // nextRois_.clear()).
    entry.in.image_rois = std::move(next_draw_rois_);
    next_draw_rois_.reset();

    // c8EmAsm_processStagedFrame: processPipeline() is false while pipeline.size() < 2.
    if (pipeline_.size() < 2) {
      PublishedFrame out = last_;
      out.video_frame_index = stage_index;
      out.repeated = true;
      out.step.reset();
      out.tracking_status_changed = false;
      out.image_events.clear();
      SetLightingOutput(&out);  // the repeated response is read again with the current `h`
      return out;
    }

    PipelineEntry& front = pipeline_.front();
    const int64_t front_time_ns = front.in.time_ns;
    front.in.estimate_lighting = mask_lighting_;  // XRRequestMask.lighting of this request
    front.in.image_targets = &targets_;           // XREngine::setDetectionImages (FULL-3)
    TrackerStep step = core_->Track(front.in);
    const auto t0 = std::chrono::steady_clock::now();
    PublishedFrame out = Postprocess(step, &hit_state_);
    // FULL-3 processPipelineData: ROIs of the response for the next draw, then HIRES scans in the
    // remaining slots while targets are loaded (tracking-controller.cc:321-327).
    if (step.requested_rois != nullptr || !targets_.empty()) {
      auto next_rois = std::make_shared<ImageRoiList>();
      if (step.requested_rois != nullptr) {
        for (const c8::ImageRoi& roi : step.requested_rois->rois) {
          image_target::AddNextDrawRoi(roi, next_rois.get());
        }
      }
      if (!targets_.empty()) image_target::AddNextDrawHiResScans(step.intrinsics, next_rois.get());
      if (!next_rois->rois.empty()) next_draw_rois_ = std::move(next_rois);
    }
    if (!step.located_images.empty()) {
      out.detected_images = DetectedImagesForResponse(
          step.located_images,
          c8::cameraMotion(initial_transform_->second, initial_transform_->first), coords_);
    }
    out.image_events = image_events_.Update(out.detected_images);
    // XREngine lastCall keeps the response before the application context correction (xr-engine.cc:
    // 190-226): the portrait camera and projection, and the orientation of the request (query turns
    // the hit input by it). The published projection is the corrected one (FULL-8).
    hit_state_.projection = projection_;
    hit_state_.orientation = orientation_;
    have_hit_state_ = true;  // XREngine lastCall (RebuildCore clears it)
    out.intrinsics = display_projection_;
    // The response's lighting message: the estimate when this request had the lighting mask, else
    // the capnp default (exposure 0, temperature 0).
    response_lighting_ = step.has_lighting
                             ? LightingEstimate{step.lighting_exposure, kWebLightingTemperature}
                             : LightingEstimate{};
    have_response_ = true;
    SetLightingOutput(&out);
    out.video_frame_index = stage_index;
    out.source_frame_index = front.stage_index;
    out.repeated = false;
    pipeline_.pop_front();

    if (frames_until_recenter_ > 0) {
      if (frames_until_recenter_ == 1) {
        RebuildCore();  // c8EmAsm_recenter() = rebuildEngine()
      }
      --frames_until_recenter_;
      // Position only; rotation and world points are kept (the original rewrites only the
      // camera extrinsic position with the configured origin; there is no map in this window).
      out.position = coords_.origin;
    }
    if (metric_enabled_) {
      ApplyMetric(front_time_ns, step, &out);
    }
    if (glue_config_.detect_surfaces && frames_until_recenter_ == 0) {
      if (step.map_reset || metric_reanchored_) {
        floor_.Reset();  // published frame of the stored points changed
      }
      // Tracker status of this step (the published status equals it; map points of a NORMAL step).
      if (step.status == TrackingStatus::kNormal) {
        floor_.Update(out.position, out.world_points);
        have_camera_ = true;
        camera_position_ = out.position;
        camera_rotation_ = out.rotation;
        intrinsics_ = step.intrinsics;
      }
    }
    // reality.trackingstatus: dispatched when (status, reason) differs from the last one.
    out.tracking_status_changed = out.status != event_status_ || out.reason != event_reason_;
    event_status_ = out.status;
    event_reason_ = out.reason;
    step.timings.glue_ms =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
    out.step = std::move(step);
    last_ = out;
    return out;
  }

  // reality.lighting (YA, JS@5501337 `if(h){...getLighting().getGlobal()...}`): published when the
  // JS state h (enableLighting) is on, whatever mask the response was requested with. A configure()
  // that throws after setting h (e.g. disableWorldTracking after start) therefore publishes the
  // default {0, 0} until a request with the mask has been processed, and turning h off stops the
  // output even for a repeated response.
  void SetLightingOutput(PublishedFrame* out) const {
    out->has_lighting = have_response_ && js_.enable_lighting;
    out->lighting = out->has_lighting ? response_lighting_ : LightingEstimate{};
  }

  // XrController.recenter (XA): P = p, l = f, c8EmAsm_recenter(); no configureXr.
  void Recenter() {
    js_.origin = js_.recenter_origin;
    js_.facing = js_.recenter_facing;
    RebuildCore();
  }

  // XrController.configure (nA, spec 06 §3.1): same order; a throw leaves the earlier changes.
  XrError Configure(const XrConfigureOptions& o) {
    if (o.enable_world_points.has_value()) js_.enable_world_points = *o.enable_world_points;
    if (o.absolute_scale.has_value()) {
      if (started_) return XrError{"", "[XR] Scale can only be changed before calling XR8.run()"};
      js_.absolute = *o.absolute_scale;
    }
    if (o.enable_lighting.has_value()) js_.enable_lighting = *o.enable_lighting;
    if (o.left_handed_axes.has_value()) js_.left_handed_axes = *o.left_handed_axes;
    if (o.mirrored_display.has_value()) js_.mirrored_display = *o.mirrored_display;
    if (o.disable_world_tracking.has_value()) {
      if (started_) {
        return XrError{"", "[XR] World tracking can only be changed before calling XR8.run()"};
      }
      js_.disable_world_tracking = *o.disable_world_tracking;
    }
    if (js_.absolute && js_.disable_world_tracking) {
      return XrError{
          "", "[XR] Unable to perform metric scale estimation if world tracking is disabled."};
    }
    ConfigureXr();
    return XrError{};
  }

  // XrController.updateCameraProjectionMatrix (xA, spec 06 §3.1).
  void UpdateCameraProjectionMatrix(const CameraProjectionUpdate& u) {
    // Validate before merging anything (an abort leaves no half-merged state).
    if (u.origin.has_value() || u.facing.has_value()) {
      CheckFiniteCoordinates(u.origin.value_or(js_.origin), u.facing.value_or(js_.facing));
    }
    if (u.pixel_rect_width.has_value()) js_.cam.texture_width = *u.pixel_rect_width;
    if (u.pixel_rect_height.has_value()) js_.cam.texture_height = *u.pixel_rect_height;
    if (u.near_clip_plane.has_value()) js_.cam.near_clip = *u.near_clip_plane;
    if (u.far_clip_plane.has_value()) js_.cam.far_clip = *u.far_clip_plane;
    if (u.origin.has_value()) {
      js_.origin = *u.origin;
      if (u.update_recenter_point) js_.recenter_origin = js_.origin;
    }
    if (u.facing.has_value()) {
      js_.facing = *u.facing;
      if (u.update_recenter_point) js_.recenter_facing = js_.facing;
    }
    if (js_.absolute) {
      js_.origin.y = 1.0f;
      js_.scale = 1.0f;
    } else {
      js_.scale = js_.origin.y;
    }
    ConfigureXr();
  }

  // onCameraStatusChange (ZA, spec 06 §5).
  XrError OnCameraStatusChange(const std::string& status, CameraDirection direction) {
    started_ = true;
    if (status != "requesting") return XrError{};
    if (direction != CameraDirection::kBack && !js_.disable_world_tracking) {
      return XrError{"slam-front-camera-unsupported",
                     "[XR] World tracking is only supported on the back camera"};
    }
    return XrError{};
  }

  // XrController.hitTest (zA -> c8EmAsm_query -> XREngine::query, spec 06 §2).
  std::vector<XrHitResult> XrHitTest(float x, float y,
                                     const std::vector<XrHitType>& included_types) const {
    if (!have_hit_state_) return {};  // no lastCall in this XREngine
    const uint32_t mask = HitTypeMask(included_types);
    std::vector<XrHitResult> out;
    for (const InternalHit& h :
         FeaturePointHitTest(hit_state_, x, y, mask, coords_.mirrored_display)) {
      XrHitResult r = RewriteHit(coords_, h);
      // RM-3b (M3): the same unit as the published pose and world points of this response.
      r.position = ToPublishedUnits(r.position);
      if (metric_hit_.active) r.distance = static_cast<float>(r.distance * metric_hit_.scale);
      out.push_back(r);
    }
    if (glue_config_.detect_surfaces && have_camera_) {
      // ACC-3 surfaces extension (not in the original web runtime): the same display ray in the
      // published frame against the detected floor / horizontal surfaces.
      Vec3f c_int, p_int;
      DisplayRayInternal(hit_state_, x, y, coords_.mirrored_display, &c_int, &p_int);
      const Vec3f c = ToPublishedUnits(RewritePosition(coords_, c_int));
      const Vec3f p = ToPublishedUnits(RewritePosition(coords_, p_int));
      const Vec3f d{p.x - c.x, p.y - c.y, p.z - c.z};
      uint32_t floor_mask = 0;
      if (ContainsResultType(mask, XrHitType::kDetectedSurface)) {
        floor_mask |= static_cast<uint32_t>(HitType::kDetectedSurface);
      }
      if (ContainsResultType(mask, XrHitType::kEstimatedSurface)) {
        floor_mask |= static_cast<uint32_t>(HitType::kEstimatedSurface);
      }
      const std::vector<HitResult> floor_hits =
          floor_mask == 0 ? std::vector<HitResult>{} : floor_.HitTest(c, d, floor_mask);
      for (const HitType want : {HitType::kDetectedSurface, HitType::kEstimatedSurface}) {
        for (const HitResult& h : floor_hits) {
          if (h.type != want) continue;
          XrHitResult r;
          r.type = want == HitType::kDetectedSurface ? XrHitType::kDetectedSurface
                                                     : XrHitType::kEstimatedSurface;
          r.position = h.position;
          r.rotation = QuatWxyz{coords_.left_handed_axes ? 0.0f : -0.0f, 0.0f, 0.0f,
                                coords_.left_handed_axes ? 0.0f : -0.0f};
          r.distance = h.distance;
          out.push_back(r);
        }
      }
    }
    return out;
  }

  const PublishedFrame& last_published() const { return last_; }
  int frames_until_recenter() const { return frames_until_recenter_; }

  std::string SetImageTargets(const std::vector<ImageTargetInput>& targets) {
    return targets_.Set(targets,
                        frontend::FullIntrinsicsForDevice(engine_config_.manufacturer,
                                                          engine_config_.model),
                        engine_config_.pyramid_model);
  }
  void UnloadImageTargets(const std::vector<std::string>& names) { targets_.Unload(names); }
  void ClearImageTargets() { targets_.Clear(); }
  std::vector<std::string> image_target_names() const { return targets_.names(); }
  bool image_target_geometry(const std::string& name, CurvyGeometryOut* out) const {
    const c8::DetectionImage* im = targets_.find(name);
    if (im == nullptr || im->getType() != c8::DetectionImageType::CURVY) return false;
    *out = image_target::CurvyGeometryOf(*im);
    return true;
  }
  std::vector<PyramidRoiRequest> NextDrawRois() const {
    std::vector<PyramidRoiRequest> out;
    if (next_draw_rois_ == nullptr) return out;
    for (const c8::ImageRoi& roi : next_draw_rois_->rois) {
      PyramidRoiRequest r;
      r.slot = static_cast<int32_t>(out.size());
      r.source = static_cast<PyramidRoiSource>(static_cast<int>(roi.source));
      r.face_id = roi.faceId;
      r.name = roi.name;
      const std::array<float, 16>& m = roi.warp.data();
      for (size_t i = 0; i < 16; ++i) r.warp[i] = m[i];
      if (roi.source == c8::ImageRoi::Source::CURVY_IMAGE_TARGET) {
        // FULL-9 (US-014): the curvy slot draw uniforms of gl-reality-frame.cc:397-433 (the same function the CPU draw uses).
        SR_CHECK(engine_config_.capture_width > 0 && engine_config_.capture_height > 0,
                 "NextDrawRois: a curvy ROI needs the engineInit capture size (the engine requires it)");
        const frontend::PyramidLayout layout = frontend::ComputePyramidLayout(
            engine_config_.capture_width, engine_config_.capture_height);
        const frontend::LevelLayout& slot = layout.roi_slots[static_cast<size_t>(r.slot)];
        const int rotation = engine_config_.rotation_deg;
        r.curvy = frontend::MakeRoiCurvyExport(roi, slot, layout.levels[0], slot.rotated ? rotation - 90 : rotation);
      }
      out.push_back(std::move(r));
    }
    return out;
  }

  const SurfaceState& surfaces() const { return floor_.state(); }

  std::vector<HitResult> HitTest(float u, float v, uint32_t mask) const {
    if (!glue_config_.detect_surfaces || !have_camera_ || intrinsics_.fx == 0.f) return {};
    // Pixel of the processed image -> GL camera ray (x right, y up, -z forward).
    const float px = u * static_cast<float>(intrinsics_.width);
    const float py = v * static_cast<float>(intrinsics_.height);
    const float fy = std::fabs(intrinsics_.fy) > 0.f ? std::fabs(intrinsics_.fy) : intrinsics_.fx;
    const c8::HVector3 ray((px - intrinsics_.cx) / intrinsics_.fx, -(py - intrinsics_.cy) / fy, -1.0f);
    const c8::Quaternion q(camera_rotation_.w, camera_rotation_.x, camera_rotation_.y,
                           camera_rotation_.z);
    const c8::HMatrix r = q.toRotationMat();
    const c8::HPoint3 d = r * c8::HPoint3(ray.x(), ray.y(), ray.z());
    return floor_.HitTest(camera_position_, Vec3f{d.x(), d.y(), d.z()}, mask);
  }

 private:
  // RM-3b (M3): ApplyMetric publishes the pose and the world points of a response as
  // metric_pos + scale * (x - responsive camera position). A hit of the same response lies in the
  // responsive published frame (RewriteHit), so it takes the same map; without metric output (or
  // inside the startup recenter window, where ApplyMetric scales only the position) the identity.
  Vec3f ToPublishedUnits(const Vec3f& v) const {
    if (!metric_hit_.active) return v;
    const double s = metric_hit_.scale;
    return Vec3f{static_cast<float>(metric_hit_.metric_camera[0] + s * (v.x - metric_hit_.responsive_camera[0])),
                 static_cast<float>(metric_hit_.metric_camera[1] + s * (v.y - metric_hit_.responsive_camera[1])),
                 static_cast<float>(metric_hit_.metric_camera[2] + s * (v.z - metric_hit_.responsive_camera[2]))};
  }

  static void CheckFiniteCoordinates(const Vec3f& origin, const QuatWxyz& facing) {
    SR_CHECK(std::isfinite(origin.x) && std::isfinite(origin.y) && std::isfinite(origin.z),
             "GlueRunner: origin must be finite");
    SR_CHECK(std::isfinite(facing.w) && std::isfinite(facing.x) && std::isfinite(facing.y) &&
                 std::isfinite(facing.z),
             "GlueRunner: facing must be finite");
  }

  static std::string Upper(const std::string& s) {
    std::string out = s;
    for (char& c : out) {
      if (c >= 'a' && c <= 'z') c = static_cast<char>(c - 'a' + 'A');
    }
    return out;
  }

  // rebuildEngine (tracking-controller.cc:134-150): new XREngine (core, incl. its RandomNumbers
  // and the 733 initial transform), pipeline.clear(), frameTick = 0, configuration re-applied
  // (the GlueConfig is immutable here, so origin/facing are already the configured values).
  // framesUntilRecenter is not touched (D-21). SensorIntake and the frontend state survive.
  void RebuildCore() {
    core_.reset();
    core_ = factory_(engine_config_);
    SR_CHECK(core_ != nullptr, "GlueRunner: factory returned no core");
    pipeline_.clear();
    frame_tick_ = 0;
    initial_transform_.reset();
    metric_.Reset();
    metric_origin_.reset();
    metric_was_initialized_ = false;
    metric_hit_.active = false;
    floor_.Reset();
    have_camera_ = false;
    have_hit_state_ = false;
  }

  // ACC-2: responsive published pose -> metres. Displacements are integrated with the current
  // scale, so a scale update never moves the published pose; at convergence the position is
  // re-anchored once (start + scale * displacement since start).
  void ApplyMetric(int64_t time_ns, const TrackerStep& step, PublishedFrame* out) {
    metric_reanchored_ = false;
    metric_hit_.active = false;
    if (frames_until_recenter_ > 0) {
      // Startup recenter window (the core is rebuilt after it): publish the origin with the same
      // scale the metric output starts with, so the first metric frame does not jump.
      const double s0 = metric_.scale();
      out->position = Vec3f{static_cast<float>(s0 * out->position.x),
                            static_cast<float>(s0 * out->position.y),
                            static_cast<float>(s0 * out->position.z)};
      out->metric = true;
      out->metric_scale = s0;
      out->metric_sigma_log = metric_.sigma_log();
      return;
    }
    const std::array<double, 3> p = {out->position.x, out->position.y, out->position.z};
    if (step.map_reset && metric_origin_.has_value()) {
      // New map, new responsive unit: integration continues from the current metric position,
      // and the convergence re-anchor (if it happens later) uses this frame as its origin.
      metric_.Perturb();
      metric_origin_ = p;
      metric_anchor_ = metric_pos_;
      metric_prev_ = p;
      metric_was_initialized_ = false;
    }
    if (out->status == TrackingStatus::kNormal) {
      metric_.PushFrame(time_ns, p,
                        {out->rotation.w, out->rotation.x, out->rotation.y, out->rotation.z});
    }
    const double s = metric_.scale();
    if (!metric_origin_.has_value()) {
      metric_origin_ = p;
      metric_pos_ = {s * p[0], s * p[1], s * p[2]};
      metric_anchor_ = metric_pos_;
    } else if (metric_.initialized() && !metric_was_initialized_) {
      metric_reanchored_ = true;
      for (int c = 0; c < 3; ++c) {
        metric_pos_[static_cast<size_t>(c)] =
            metric_anchor_[static_cast<size_t>(c)] +
            s * (p[static_cast<size_t>(c)] - (*metric_origin_)[static_cast<size_t>(c)]);
      }
    } else {
      for (int c = 0; c < 3; ++c) {
        metric_pos_[static_cast<size_t>(c)] +=
            s * (p[static_cast<size_t>(c)] - metric_prev_[static_cast<size_t>(c)]);
      }
    }
    metric_was_initialized_ = metric_.initialized();
    metric_prev_ = p;
    for (WorldPointOut& w : out->world_points) {
      if (w.id == 0) continue;  // 0-padding (Q10)
      w.position = Vec3f{static_cast<float>(metric_pos_[0] + s * (w.position.x - p[0])),
                         static_cast<float>(metric_pos_[1] + s * (w.position.y - p[1])),
                         static_cast<float>(metric_pos_[2] + s * (w.position.z - p[2]))};
    }
    out->position = Vec3f{static_cast<float>(metric_pos_[0]), static_cast<float>(metric_pos_[1]),
                          static_cast<float>(metric_pos_[2])};
    // The tracking status is not changed: before convergence the positions use the prior scale
    // (camera height 1.4 m at the responsive origin), like the original responsive units; whether
    // they are metric is PublishedFrame::metric_initialized.
    out->metric = true;
    out->metric_scale = s;
    out->metric_sigma_log = metric_.sigma_log();
    out->metric_initialized = metric_.initialized();
    out->metric_lag_s = metric_.lag_s();
    metric_hit_ = MetricHitFrame{true, p, metric_pos_, s};
  }

  // configureXr (fA): the mask + graphics intrinsics (always sent with cam) and the coordinate
  // configuration (sent when origin, facing and scale are truthy; scale 0 is not sent).
  void ConfigureXr() {
    mask_world_points_ = js_.enable_world_points;
    mask_lighting_ = js_.enable_lighting;
    if (capture_k_.width > 0) {  // no capture size (injected core): the projection stays zero
      GraphicsIntrinsics g = js_.cam;
      // onStart: canvas size. The canvas follows the video, so the capture size as engineInit got it
      // (landscape for a landscape capture); portrait captures give the same values as before.
      if (g.texture_width == 0) g.texture_width = engine_config_.capture_width;
      if (g.texture_height == 0) g.texture_height = engine_config_.capture_height;
      // deviceToPortraitOrientationPreprocess: landscape exchanges the graphics texture size; the
      // engine computes the portrait projection, portraitToDeviceOrientationPostprocess corrects it.
      PortraitGraphicsSize(orientation_, &g.texture_width, &g.texture_height);
      projection_ = DisplayProjection(capture_k_, g);
      display_projection_ = PortraitToDeviceProjection(orientation_, projection_);
    }
    if (!started_ && (js_.absolute != engine_config_.estimate_scale ||
                      js_.disable_world_tracking != engine_config_.disable_vio)) {
      // estimateScale / disableVio of a session that has not started: the core that will process
      // the first frame is built with them (only reachable before XR8.run in the original).
      engine_config_.estimate_scale = js_.absolute;
      engine_config_.disable_vio = js_.disable_world_tracking;
      const AccuracyConfig& acc = engine_config_.accuracy;
      metric_enabled_ = acc.enabled && acc.metric_output && !engine_config_.estimate_scale;
      RebuildCore();
    }
    if (js_.scale != 0.0f && !std::isnan(js_.scale)) {
      CoordinateSystem next;
      next.origin = js_.origin;
      next.facing = js_.facing;
      next.scale = js_.scale;
      next.left_handed_axes = js_.left_handed_axes;
      next.mirrored_display = js_.mirrored_display;
      const bool changed = !SameCoordinateSystem(coords_, next);
      coords_ = next;
      scale_ = js_.scale;
      // ACC-3 floor_ and the stored camera are in the published coordinates of the moment they
      // were stored; a changed coordinate system makes them stale, so surface detection restarts
      // (the height bins also follow the new scale).
      if (changed) ResetSurfaceState();
    }
  }

  static bool SameCoordinateSystem(const CoordinateSystem& a, const CoordinateSystem& b) {
    return a.origin.x == b.origin.x && a.origin.y == b.origin.y && a.origin.z == b.origin.z &&
           a.facing.w == b.facing.w && a.facing.x == b.facing.x && a.facing.y == b.facing.y &&
           a.facing.z == b.facing.z && a.scale == b.scale &&
           a.left_handed_axes == b.left_handed_axes && a.mirrored_display == b.mirrored_display;
  }

  // Fresh FloorDetector for the current scale / metric mode (height bin about 2 cm: metres in
  // metric output, else responsive units where origin.y = camera height, about 1.4 m) and no
  // stored camera.
  void ResetSurfaceState() {
    FloorDetectorParams fp;
    fp.bin = metric_enabled_ ? 0.02f : 0.015f * scale_;
    fp.min_below = metric_enabled_ ? 0.3f : 0.2f * scale_;
    fp.max_below = metric_enabled_ ? 4.0f : 3.0f * scale_;
    floor_ = FloorDetector(fp);
    have_camera_ = false;
  }

  // 733 (transform from the 784 return value, recenterPose) + 704 + JS exposure. Also returns the
  // response camera (733-corrected, internal) and the feature set as XREngine's lastCall holds them.
  PublishedFrame Postprocess(const TrackerStep& step, HitTestState* last_call) {
    PublishedFrame out;
    out.status = step.status;
    out.reason = step.reason;

    // 733: transform = (fromHMatrix(P), translation(P)) of the 784 return (D:71847-71857).
    const c8::HMatrix p = FromHMatrix4f(step.pose);
    c8::Quaternion q = c8::Quaternion::fromHMatrix(p);
    c8::HVector3 t = c8::translation(p);
    // recenterPose (pose-request-executor.cc:278-327): the first response of an XREngine has no
    // initial transform -> (groundPlaneRotation(current), current position); later frames are
    // expressed relative to it. With the yaw offset the first pose has yaw 0 and the first
    // translation is the MapTracker start, so this is close to the identity (S4 §3.8, INFERRED).
    if (!initial_transform_.has_value()) {
      initial_transform_ = std::make_pair(c8::groundPlaneRotation(q), t);
    }
    const c8::HMatrix current = c8::cameraMotion(t, q);
    const c8::HMatrix original =
        c8::cameraMotion(initial_transform_->second, initial_transform_->first);
    const c8::HMatrix corrected = c8::egomotion(original, current);
    t = c8::translation(corrected);
    q = c8::rotation(corrected);
    last_call->camera_position = Vec3f{t.x(), t.y(), t.z()};
    last_call->camera_rotation = QuatWxyz{q.w(), q.x(), q.y(), q.z()};
    last_call->points.clear();
    last_call->points.reserve(step.world_points_internal.size());
    for (const WorldPointOut& w : step.world_points_internal) last_call->points.push_back(w.position);

    // portraitToDeviceOrientationPostprocess (xr-engine.cc:221-226, orientation.cc:76-125): the
    // response camera is turned by the device orientation of the request, after the internal state
    // copy and before 704.
    PortraitToDevicePose(orientation_, &t, &q);

    // 704 (orientation.cc:462-481). Position32f / Quaternion32f fields.
    float px = t.x(), py = t.y(), pz = t.z();
    float qw = q.w(), qx = q.x(), qy = q.y(), qz = q.z();
    const bool right_handed = !coords_.left_handed_axes;  // X_LEFT_Y_UP_Z_FORWARD
    if (right_handed) {  // makeRightHanded
      pz = -pz;
      qw = -qw;
      qz = -qz;
    }
    if (coords_.mirrored_display) {  // mirror
      px = -px;
      qy = -1.0f * qy;
      qz = -1.0f * qz;
    }
    // recenterAndScale (orientation.cc:253-300, 581-589).
    const float s = scale_ > 0.0f ? scale_ : 1.0f;
    const c8::Quaternion oq = c8::groundPlaneRotation(
        c8::Quaternion(coords_.facing.w, coords_.facing.x, coords_.facing.y, coords_.facing.z));
    const c8::HMatrix o = c8::cameraMotion(
        c8::HVector3(coords_.origin.x, coords_.origin.y, coords_.origin.z), oq);
    const c8::HPoint3 np = o * c8::HPoint3(s * px, s * py, s * pz);
    const c8::Quaternion nq = oq.times(c8::Quaternion(qw, qx, qy, qz));
    out.position = Vec3f{np.x(), np.y(), np.z()};
    out.rotation = QuatWxyz{nq.w(), nq.x(), nq.y(), nq.z()};

    // World points: 631 list (0-padding included) through the same 704 chain; JS exposes them only
    // when reason != INITIALIZING (S4 §3.10, JS@5501536) and the featureSet mask is on (FULL-1).
    if (step.reason != TrackingReason::kInitializing && mask_world_points_) {
      out.world_points.reserve(step.world_points_internal.size());
      for (const WorldPointOut& w : step.world_points_internal) {
        float x = w.position.x, y = w.position.y, z = w.position.z;
        if (right_handed) z = -z;
        if (coords_.mirrored_display) x = -x;
        const c8::HPoint3 nw = o * c8::HPoint3(s * x, s * y, s * z);
        out.world_points.push_back(WorldPointOut{w.id, Vec3f{nw.x(), nw.y(), nw.z()}, w.confidence});
      }
    }
    return out;
  }

  EngineConfig engine_config_;
  GlueConfig glue_config_;
  float scale_ = 1.0f;
  TrackerCoreFactory factory_;
  frontend::SensorIntake intake_;
  std::unique_ptr<TrackerCore> core_;
  std::deque<PipelineEntry> pipeline_;
  int frame_tick_ = 0;  // tracking-controller.cc frameTick: reset/incremented only, never read
  int frames_until_recenter_ = kInitialFramesUntilRecenter;
  int64_t next_stage_index_ = 0;
  // 733 lastResponse.pose.initialTransform: (rotation, position); belongs to the XREngine.
  std::optional<std::pair<c8::Quaternion, c8::HVector3>> initial_transform_;
  PublishedFrame last_;
  LightingEstimate response_lighting_;  // lighting message of the last response
  bool have_response_ = false;          // the JS `reality` exists once a response has been read
  // ACC-2 metric output state.
  bool metric_enabled_ = false;
  MetricScaleTracker metric_;
  std::optional<std::array<double, 3>> metric_origin_;
  std::array<double, 3> metric_anchor_{};
  std::array<double, 3> metric_pos_{};
  std::array<double, 3> metric_prev_{};
  bool metric_was_initialized_ = false;
  bool metric_reanchored_ = false;
  // RM-3b (M3): the responsive -> metric map of the response a hit test reads.
  struct MetricHitFrame {
    bool active = false;
    std::array<double, 3> responsive_camera{};
    std::array<double, 3> metric_camera{};
    double scale = 1.0;
  };
  MetricHitFrame metric_hit_;
  // ACC-3 surfaces state.
  FloorDetector floor_;
  bool have_camera_ = false;
  Vec3f camera_position_;
  QuatWxyz camera_rotation_;
  PinholeIntrinsics intrinsics_;
  // FULL-1 XrController state (spec 06 §3): JS variables and what configureXr last sent.
  struct JsState {
    Vec3f origin;            // P
    QuatWxyz facing;         // l
    Vec3f recenter_origin;   // p
    QuatWxyz recenter_facing;  // f
    float scale = 1.0f;      // d
    bool absolute = false;   // a
    bool enable_world_points = true;  // S (glue default true, spec 06 §3.2)
    bool enable_lighting = false;     // h
    bool left_handed_axes = false;    // L
    bool mirrored_display = false;    // Y
    bool disable_world_tracking = false;  // R
    GraphicsIntrinsics cam;  // n
  };
  JsState js_;
  bool started_ = false;  // AA
  CoordinateSystem coords_;  // configuredCoordinateSystem
  bool mask_world_points_ = true;
  bool mask_lighting_ = false;
  PinholeIntrinsics capture_k_;
  // FULL-8: device orientation of the engineInit rotation; projection_ is the portrait matrix the
  // tracker's lastCall holds, display_projection_ the one published as reality.intrinsics.
  DeviceOrientation orientation_ = DeviceOrientation::kPortrait;
  std::array<float, 16> projection_{};
  std::array<float, 16> display_projection_{};
  // XREngine lastCall for hitTest.
  HitTestState hit_state_;
  bool have_hit_state_ = false;
  // reality.trackingstatus last dispatched (t, e).
  TrackingStatus event_status_ = TrackingStatus::kUnspecified;
  TrackingReason event_reason_ = TrackingReason::kUnspecified;
  // FULL-3 image targets: detectionImages_ (survives rebuildEngine), the GlRealityFrame nextRois
  // (the pyramid renderer is not rebuilt by recenter) and the JS detectedImageMap_.
  ImageTargetSet targets_;
  std::shared_ptr<const ImageRoiList> next_draw_rois_;
  ImageEventTracker image_events_;
};

GlueRunner::GlueRunner(EngineConfig engine_config, GlueConfig glue_config,
                       TrackerCoreFactory factory)
    : impl_(std::make_unique<Impl>(std::move(engine_config), glue_config, std::move(factory))) {}

GlueRunner::~GlueRunner() = default;

void GlueRunner::PushDeviceMotion(const DeviceMotionSample& s) { impl_->PushDeviceMotion(s); }

void GlueRunner::PushDeviceOrientation(const DeviceOrientationSample& s) {
  impl_->PushDeviceOrientation(s);
}

PublishedFrame GlueRunner::ProcessFrame(int64_t time_ns, int64_t video_time_ns,
                                        const GrayImageView& img, PyramidAtlasView injected) {
  return impl_->ProcessFrame(time_ns, video_time_ns, img, injected);
}

PublishedFrame GlueRunner::Publish(FrameInput in) { return impl_->Publish(std::move(in)); }

void GlueRunner::Recenter() { impl_->Recenter(); }

const PublishedFrame& GlueRunner::last_published() const { return impl_->last_published(); }

int GlueRunner::frames_until_recenter() const { return impl_->frames_until_recenter(); }

const SurfaceState& GlueRunner::surfaces() const { return impl_->surfaces(); }

std::vector<HitResult> GlueRunner::HitTest(float u, float v, uint32_t mask) const {
  return impl_->HitTest(u, v, mask);
}

XrError GlueRunner::Configure(const XrConfigureOptions& options) {
  return impl_->Configure(options);
}

void GlueRunner::UpdateCameraProjectionMatrix(const CameraProjectionUpdate& update) {
  impl_->UpdateCameraProjectionMatrix(update);
}

std::vector<XrHitResult> GlueRunner::XrHitTest(float x, float y,
                                               const std::vector<XrHitType>& included_types) const {
  return impl_->XrHitTest(x, y, included_types);
}

XrError GlueRunner::OnCameraStatusChange(const std::string& status, CameraDirection direction) {
  return impl_->OnCameraStatusChange(status, direction);
}

std::string GlueRunner::SetImageTargets(const std::vector<ImageTargetInput>& targets) {
  return impl_->SetImageTargets(targets);
}

void GlueRunner::UnloadImageTargets(const std::vector<std::string>& names) {
  impl_->UnloadImageTargets(names);
}

void GlueRunner::ClearImageTargets() { impl_->ClearImageTargets(); }

std::vector<std::string> GlueRunner::image_target_names() const {
  return impl_->image_target_names();
}

bool GlueRunner::image_target_geometry(const std::string& name, CurvyGeometryOut* out) const {
  return impl_->image_target_geometry(name, out);
}

std::vector<PyramidRoiRequest> GlueRunner::NextDrawRois() const { return impl_->NextDrawRois(); }

}  // namespace slam_refact

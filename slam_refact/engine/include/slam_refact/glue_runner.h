// slam_refact public API: JS/tracking-controller glue layer (architecture.md §4.2-§4.4, D-3, D-13).
//
// FROZEN CONTRACT (WP0b). Changes follow slam_refact/docs/design/contract-changes.md.
// Implementation: src/engine/glue_runner.cc. Not thread-safe.

#ifndef SLAM_REFACT_GLUE_RUNNER_H_
#define SLAM_REFACT_GLUE_RUNNER_H_

#include <array>
#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "slam_refact/engine.h"
#include "slam_refact/engine_config.h"
#include "slam_refact/image_targets.h"
#include "slam_refact/image_types.h"
#include "slam_refact/sensor_types.h"
#include "slam_refact/surfaces.h"
#include "slam_refact/types.h"
#include "slam_refact/xr_controller.h"

namespace slam_refact {

// One published response (1 frame delay, startup recenter, repeats).
struct PublishedFrame {
  int64_t video_frame_index = 0;
  int64_t source_frame_index = 0;
  bool repeated = false;
  // world_from_camera, right-handed y-up, GL camera (-Z forward).
  Vec3f position;
  QuatWxyz rotation;
  TrackingStatus status = TrackingStatus::kUnspecified;
  TrackingReason reason = TrackingReason::kUnspecified;
  // Exposed only when reason != INITIALIZING (S4 §3.10).
  std::vector<WorldPointOut> world_points;
  // Empty when repeated.
  std::optional<TrackerStep> step;
  // Accuracy mode ACC-2 (AccuracyConfig::metric_output): metres per responsive unit, its log
  // standard deviation, whether it has converged, and the estimated IMU lag (seconds).
  bool metric = false;
  double metric_scale = 0.0;
  double metric_sigma_log = 0.0;
  bool metric_initialized = false;
  double metric_lag_s = 0.0;
  // FULL-1 reality.intrinsics: display projection matrix (column-major 4x4) of this response,
  // from the capture intrinsics and the configured graphics intrinsics (spec 06 §1). All zero
  // before the first response.
  std::array<float, 16> intrinsics{};
  // FULL-1 reality.lighting: set only when lighting is enabled (configure enableLighting).
  bool has_lighting = false;
  LightingEstimate lighting;
  // FULL-1 reality.trackingstatus: (status, reason) differs from the previous response's.
  bool tracking_status_changed = false;
  // FULL-3 reality.detectedImages of this response (located images seen in one of the last 3
  // frames and found in more than 3 consecutive frames, sorted by name) and the image events the
  // original JS dispatches for it (imagefound / imageupdated in detected order, then imagelost).
  // A repeated response keeps the images and has no events.
  std::vector<DetectedImageOut> detected_images;
  std::vector<ImageTargetEvent> image_events;
};

// Creates a fresh core; called at construction and at every recenter.
// The EngineConfig given to GlueRunner is passed through.
using TrackerCoreFactory = std::function<std::unique_ptr<TrackerCore>(const EngineConfig&)>;

// Returns std::make_unique<Engine>(cfg).
std::unique_ptr<TrackerCore> DefaultEngineFactory(const EngineConfig& cfg);

class GlueRunner {
 public:
  GlueRunner(EngineConfig engine_config, GlueConfig glue_config,
             TrackerCoreFactory factory = DefaultEngineFactory);
  ~GlueRunner();

  GlueRunner(const GlueRunner&) = delete;
  GlueRunner& operator=(const GlueRunner&) = delete;

  // US-010 pushImu.
  void PushDeviceMotion(const DeviceMotionSample& s);
  void PushDeviceOrientation(const DeviceOrientationSample& s);

  // SensorIntake::TakeForStage (func 647 runs here only) followed by Publish.
  PublishedFrame ProcessFrame(int64_t time_ns, int64_t video_time_ns, const GrayImageView& img,
                              PyramidAtlasView injected = {});

  // §4.4. in.imu_events must already be post-647.
  PublishedFrame Publish(FrameInput in);

  // XrController.recenter (spec 06 §3): restores the JS origin/facing to the recenter point and
  // rebuilds the core (rebuildEngine: new core, pipeline cleared, the last configured coordinate
  // system re-applied).
  void Recenter();

  // US-010 getPose / getStatus / getWorldPoints.
  const PublishedFrame& last_published() const;

  // FULL-7 (RM-4b): the tracking-controller's framesUntilRecenter now (starts at
  // GlueConfig::frames_until_recenter, 5 by default; counts down per processed frame, the core is
  // rebuilt when it passes 1 -> 0). A caller that re-creates the runner for an engineInit re-run
  // passes this value as the new GlueConfig::frames_until_recenter.
  int frames_until_recenter() const;

  // US-010 reset.
  void Reset() { Recenter(); }

  // ACC-3 (GlueConfig::detect_surfaces): detected floor and horizontal surfaces, published world.
  const SurfaceState& surfaces() const;
  // Hit test of a point of the processed (portrait) image, u, v in [0, 1] from its top-left, against
  // the last published camera; mask = OR of HitType bits. Empty when surfaces are disabled or no
  // NORMAL frame has been published.
  std::vector<HitResult> HitTest(float u, float v, uint32_t mask) const;

  // FULL-1 XrController.configure (spec 06 §3). Returns the original JS error (and changes what
  // the original changed before throwing) when scale / disableWorldTracking change after start or
  // absolute scale is combined with disableWorldTracking.
  XrError Configure(const XrConfigureOptions& options);
  // FULL-1 XrController.updateCameraProjectionMatrix (spec 06 §3); never rebuilds the core.
  void UpdateCameraProjectionMatrix(const CameraProjectionUpdate& update);
  // FULL-1 XrController.hitTest (spec 06 §2): x, y in [0, 1] of the displayed camera feed (top-left
  // origin), included_types empty = all. FEATURE_POINT as the original; ESTIMATED_SURFACE and
  // DETECTED_SURFACE are empty unless GlueConfig::detect_surfaces (ACC-3 extension) is on.
  std::vector<XrHitResult> XrHitTest(float x, float y,
                                     const std::vector<XrHitType>& included_types) const;
  // FULL-1 onCameraStatusChange (spec 06 §5): marks the session as started; a world-tracking
  // request for a non-back camera returns slam-front-camera-unsupported.
  XrError OnCameraStatusChange(const std::string& status, CameraDirection direction);

  // FULL-3 image targets (docs/design/image-target.md §4). SetImageTargets = setActiveImageTargets
  // (tracking-controller.ts:1130-1180) + c8EmAsm_addNewDetectionImageLoader /
  // processNewDetectionImageTexture / unloadDetectionImages: a target whose name and imagePath are
  // already loaded is kept, the others are loaded on the CPU pyramid port, missing names are
  // unloaded. Rejects more than kMaxActiveImageTargets targets, duplicate names, malformed JSON or
  // luminance (curvy CYLINDER / CONICAL targets are supported since US-014, FULL-9); on error
  // nothing changes. Returns "" on success.
  std::string SetImageTargets(const std::vector<ImageTargetInput>& targets);
  // c8EmAsm_unloadDetectionImages.
  void UnloadImageTargets(const std::vector<std::string>& names);
  // c8EmAsm_clearDetectionImages (clearImageTargetState: also resets the image tracking state).
  void ClearImageTargets();
  // Names of the loaded targets, sorted.
  std::vector<std::string> image_target_names() const;
  // US-030 / D-1 (docs/design/request-engine-image-target-geometry.md): the CurvyGeometry the original JS reads right after the load
  // (DetectionImage::toCurvyGeometry). Returns false (and leaves *out alone) when the target is not loaded or is not curvy.
  // Same values as DetectedImage::curvy_geometry of a detection of that target.
  bool image_target_geometry(const std::string& name, CurvyGeometryOut* out) const;
  // FULL-6: the ROIs of the next draw (the frame staged by the next ProcessFrame / Publish): what a
  // GPU pyramid producer must draw into the ROI slots of PyramidLayout. The list the next staged
  // frame consumes; empty without targets. FULL-9: a curvy ROI also carries its slot draw uniforms
  // (PyramidRoiRequest::curvy).
  std::vector<PyramidRoiRequest> NextDrawRois() const;

 private:
  class Impl;  // defined by WP5
  std::unique_ptr<Impl> impl_;
};

}  // namespace slam_refact

#endif  // SLAM_REFACT_GLUE_RUNNER_H_

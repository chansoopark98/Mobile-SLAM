// slam_refact public API: tracking core (architecture.md §4.1-§4.3, D-13, D-14).
//
// FROZEN CONTRACT (WP0b). Changes follow slam_refact/docs/design/contract-changes.md.
// Implementation: src/engine/engine.cc. Engine is single-threaded and not thread-safe.

#ifndef SLAM_REFACT_ENGINE_H_
#define SLAM_REFACT_ENGINE_H_

#include <cstdint>
#include <memory>
#include <vector>

#include "slam_refact/engine_config.h"
#include "slam_refact/image_targets.h"
#include "slam_refact/image_types.h"
#include "slam_refact/sensor_types.h"
#include "slam_refact/types.h"

namespace slam_refact {

// Input of one core tracking step.
struct FrameInput {
  int64_t time_ns = 0;        // TrackingSensorFrame.timeNanos
  int64_t video_time_ns = 0;  // for output only
  // Exactly one image source is used; pyramid has priority over rgba and gray.
  GrayImageView gray{};
  RgbaImageView rgba{};
  PyramidAtlasView pyramid{};
  // getQuaternion(alpha, beta, gamma) rounded to float, before xrRotationFromDeviceRotation.
  QuatWxyz device_pose;
  // Post-647 events, sorted by time. Engine never converts or reorders them (D-14).
  std::vector<ImuEvent> imu_events;
  // ACC-5: metres per internal map unit for the visual-inertial prediction (0 = none).
  double metric_scale_hint = 0.0;
  // FULL-1: XRRequestMask.lighting of this request (reality.lighting, spec 06 §4).
  bool estimate_lighting = false;
  // FULL-3 image targets (docs/design/image-target.md §4): the loaded targets of this request
  // (XREngine::setDetectionImages; owned by the caller, null or empty = none: 938 is skipped and
  // the detection config is unchanged) and the ROIs drawn into this frame's pyramid (the
  // GlRealityFrame nextRois of the draw; null = none). GlueRunner fills both.
  const ImageTargetSet* image_targets = nullptr;
  std::shared_ptr<const ImageRoiList> image_rois;
};

// One core step (no delay). Coordinates = internal XR (y-up, left-handed, camera +z forward).
struct TrackerStep {
  HMatrix4f pose;  // 784 return value (Y*P or metric)
  HMatrix4f tam_extrinsic;
  TrackingStatus status = TrackingStatus::kUnspecified;
  TrackingReason reason = TrackingReason::kUnspecified;
  std::vector<WorldPointOut> world_points_internal;  // 631 result (before 704)

  // Parity-only debug fields, filled by WP4 (§3.7).
  HMatrix4f predicted_pose;
  int map_tracker_state = 0;
  bool frame_tracker_ok = false;
  bool ok = false;
  int loss_state = 0;
  int lost_frames = 0;
  int frames = 0;
  bool yaw_offset_initialized = false;
  bool map_reset = false;
  bool keyframe_created = false;
  bool scale_latched = false;
  float latched_scale = 0.f;
  int num_keypoints = 0;
  int num_orb = 0;
  int num_gorb = 0;
  int num_keyframes = 0;
  // Indexed by PointSource - 1: TRIANGULATED, GROUND, SKY, DEPTH, CLOUD.
  int num_points_by_source[5] = {0, 0, 0, 0, 0};
  int ground_matches = 0;
  StageTimings timings;
  // Processing (level-0) intrinsics of this step (ACC-3: hit test rays; not an original field).
  PinholeIntrinsics intrinsics;
  // FULL-1: web lighting exposure of the tracked pyramid when FrameInput::estimate_lighting.
  bool has_lighting = false;
  float lighting_exposure = 0.f;
  // FULL-3: 938 located images (Tracker::locatedImageTargets after applyToLocatedImages; empty
  // when no target is loaded) and the ROIs they request for a later draw (lastSeen < 20, in located
  // order; image-detection-request-executor.cc:198-201). Null when no target is loaded.
  std::vector<LocatedImageOut> located_images;
  std::shared_ptr<const ImageRoiList> requested_rois;
};

// Core interface (D-13). GlueRunner owns exactly one and recreates it on recenter.
class TrackerCore {
 public:
  virtual ~TrackerCore() = default;
  virtual TrackerStep Track(const FrameInput& in) = 0;
};

// Product core: configureXr + initDeviceInfo + engineInit semantics; Track = 784 + 631.
class Engine final : public TrackerCore {
 public:
  explicit Engine(const EngineConfig& cfg);
  ~Engine() override;

  Engine(const Engine&) = delete;
  Engine& operator=(const Engine&) = delete;

  // 476: recreates MapTracker and MapBuilder only when disable_vio turns on.
  void Configure(const EngineConfig& cfg);

  // Aborts (SR_CHECK) on post-647 kind 2 events and on events out of time order (D-14).
  TrackerStep Track(const FrameInput& in) override;

 private:
  class Impl;  // defined by WP5
  std::unique_ptr<Impl> impl_;
};

}  // namespace slam_refact

#endif  // SLAM_REFACT_ENGINE_H_

// VioTracker module public header (architecture.md §3.7; S4 §2-§6).
//
// FROZEN CONTRACT (WP0b): every declaration in this file. Changes follow
// slam_refact/docs/design/contract-changes.md. `private:` sections are owned by WP4.

#ifndef SLAM_REFACT_VIO_TRACKER_VIO_TRACKER_H_
#define SLAM_REFACT_VIO_TRACKER_VIO_TRACKER_H_

#include <array>
#include <cstdint>
#include <vector>

#include "c8/hmatrix.h"
#include "c8/pixels/gr8-pyramid.h"
#include "c8/quaternion.h"
#include "common/tracking_types.h"
#include "compat/random_numbers.h"
#include "reality/engine/imagedetection/detection-image-tracker.h"
#include "slam_refact/types.h"

namespace slam_refact::vio_tracker {

// XrTrackingState written by 784 (S4 §2.6, §3.3).
struct StatusOut {
  TrackingStatus status = TrackingStatus::kUnspecified;
  TrackingReason reason = TrackingReason::kUnspecified;
};

// now_ (VioTracker+7712): ts / xr device quaternion / events (S4 §2.3).
struct FrameRecord {
  int64_t time_ns = 0;
  c8::Quaternion xr_device_rotation;
  std::vector<TrackingSensorEvent> sensor_events;
};

// Parity-only debug output of one Track call; Engine copies it into TrackerStep (§4.2).
// Filled by WP4 from the module getters (FeatureFrontend::last_stats, WorldMap::NumKeyframes /
// NumPointsBySource, FrameFrameTracker::last_ground_match_count, MapTrackerOutput,
// MapBuilderOutput).
struct TrackDebug {
  c8::HMatrix predicted_pose = c8::HMatrixGen::i();
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
  std::array<int, 5> num_points_by_source{};  // indexed by PointSource - 1
  int ground_matches = 0;
  StageTimings timings;
};

// Tracking-loss state machine (S4 §3.3, §6.1).
class LossStateMachine {
 public:
  void Update(bool ok, int64_t now_ns);
  TrackingStatus status() const;

  // Debug getters for TrackDebug.
  int state() const;        // loss_state_ (+6352), 0..3
  int lost_frames() const;  // +6368
  int frames() const;       // +6372
  bool last_ok() const;     // ok argument of the last Update

 private:
  // Owned by WP4.
  friend struct LossStateMachineAccess;  // unit tests seed the state (vio_tracker_internal.h)
  int state_ = 0;                  // +6352
  int64_t state_enter_ns_ = 0;     // +6360
  int lost_frames_ = 0;            // +6368
  int frames_ = 0;                 // +6372
  bool last_ok_ = false;
};

// 784 (initial values 485, configure 476).
// FULL-6 (docs/design/image-target.md section 7.7, gate G-ROI-C): which path of the public
// ImageTrackingState::track located an image in the last 938 call. roi_feats = the local tracker
// used the IMAGE_TARGET ROI feats of this frame (ImageTrackingState::featsForTrackedImage);
// local_inliers / global_inliers = non-zero entries of the public debug API
// DetectionImageTracker::inliers / globalInliers. The inlier counts are meaningful only while
// ImageTrackingState::AGGRESSIVELY_FREE_MEMORY is false (the default true clears the scratch right
// after tracking, so both are 0).
struct LocatedImageDebug {
  bool roi_feats = false;
  int local_inliers = 0;
  int global_inliers = 0;
};

class VioTracker {
 public:
  VioTracker(bool estimate_scale, bool disable_vio);
  ~VioTracker();

  VioTracker(const VioTracker&) = delete;
  VioTracker& operator=(const VioTracker&) = delete;

  // S4 §3.1. Returns the 784 pose (Y*P or metric transform).
  c8::HMatrix Track(const TrackingSensorFrame& frame, const c8::Gr8Pyramid& pyr,
                    compat::RandomNumbers* rng, StatusOut* status, TrackDebug* debug);

  // FULL-3: 784 with the image targets of this request (D:71846 `&imageTargets`). A non-empty map
  // switches the detection config to allOrb + allGorb (D:82467-82468) and runs 938 after 926 / 936 /
  // 664 with the same RNG as 926 (D:83064, D:84165). Track() is this call with an empty map.
  c8::HMatrix TrackWithTargets(const TrackingSensorFrame& frame, const c8::Gr8Pyramid& pyr,
                               const DetectionImagePtrMap& targets, compat::RandomNumbers* rng,
                               StatusOut* status, TrackDebug* debug);

  // FULL-3: Tracker::locatedImageTargets after applyToLocatedImages (the transform of the last
  // returned pose); refreshed by every TrackWithTargets call with a non-empty map.
  const std::vector<c8::LocatedImage>& located_images() const;
  // FULL-6: per-located-image tracking-path debug, parallel to located_images() (see
  // LocatedImageDebug). Refreshed together with it.
  const std::vector<LocatedImageDebug>& located_images_debug() const;

  // FULL-3: Tracker::setImageTrackingDisabled(true) then (false) (clearImageTargetState).
  void ResetImageTracking();

  // 631 (Q10: list sized by the unfiltered count; remaining entries are 0-padding).
  void GetWorldPoints(std::vector<WorldPointOut>* out) const;

  // 476: only turning it on recreates MapTracker and MapBuilder (incl. the FFT RNG reseed).
  void SetDisableVio(bool on);

  const c8::HMatrix& tam_extrinsic() const;

  // Debug getters for TrackDebug (one per field without another source; CONTRACTS.md table).
  const FrameState& last_prediction() const;   // predicted_pose = last_prediction().pose
  const LossStateMachine& loss_state() const;   // loss_state / lost_frames / frames / ok
  bool yaw_offset_initialized() const;          // +6344
  bool scale_latched() const;                   // +2840
  float latched_scale() const;                  // +2844

 private:
  // Owned by WP4. The collaborators 784 calls (frontend, MapTracker, MapBuilder, world map) sit
  // behind the TrackerModules seam (vio_tracker/tracker_modules.h); the public constructor uses
  // the real modules, NewVioTrackerWithModules (vio_tracker_internal.h) injects test doubles.
  // All state lives in Impl (vio_tracker/vio_tracker_internal.h).
  VioTracker(bool estimate_scale, bool disable_vio, std::unique_ptr<class TrackerModules> modules);
  friend std::unique_ptr<VioTracker> NewVioTrackerWithModules(bool estimate_scale, bool disable_vio,
                                                              std::unique_ptr<TrackerModules> modules);
  friend struct VioTrackerAccess;
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

// S4 §3.2.
FrameState PredictMotion(const FrameState& prev, const FrameRecord& now);

// S4 §3.4. hist[0] = previous, hist[1] = current.
void UpdateHistory(FrameState hist[2], const c8::HMatrix& p, const FrameRecord& now,
                   bool world_transform_updated);

}  // namespace slam_refact::vio_tracker

#endif  // SLAM_REFACT_VIO_TRACKER_VIO_TRACKER_H_

// MapTracker module public header (architecture.md §3.4; S2).
//
// FROZEN CONTRACT (WP0b): every declaration in this file. Changes follow
// slam_refact/docs/design/contract-changes.md. `private:` sections are owned by WP2.
//
// Not frozen here (signature not fixed by the design; WP2 declares them in private headers):
// ComputeFramePoseUsingRobustP2P (1265), SelectSubset (§5.4). MapTrackResult (S2 §2.5) is only
// forward-declared; WP2 defines it in a private header.

#ifndef SLAM_REFACT_MAP_TRACKER_MAP_TRACKER_H_
#define SLAM_REFACT_MAP_TRACKER_MAP_TRACKER_H_

#include <memory>
#include <vector>

#include "c8/hmatrix.h"
#include "c8/quaternion.h"
#include "common/tracking_types.h"
#include "compat/random_numbers.h"
#include "frame_frame_tracker/frame_frame_tracker.h"
#include "map/world_map.h"

namespace slam_refact::map_tracker {

// trackPoints (1272) result, S2 §2.5. Defined by WP2.
struct MapTrackResult;

// 926; constructor 640; event application 664.
class MapTracker {
 public:
  // 640: state_ = 0 (LOST), frame_tracker_ok_ = 0, frame_count_ = -1 (0 on the first Track),
  // last_state_change_frame_ = 0 (VERIFIED, WP2-PRE ④). Holds the FrameFrameTracker by value,
  // with its own compat::RandomNumbers(5489).
  MapTracker();
  ~MapTracker();

  MapTracker(const MapTracker&) = delete;
  MapTracker& operator=(const MapTracker&) = delete;

  // S2 §3 B-G. Path A (depth-keyframe relocalization) is not implemented: SR_CHECK abort when
  // reloc_kf_pending_ is set (architecture.md §1.2 M2).
  void Track(map::WorldMap& map, TrackerInput* frame, const FrameState& predicted,
             MapTrackerOutput* out, compat::RandomNumbers* rng);

  // 664, S4 §3.5. Returns the current pose (tam_extrinsic).
  c8::HMatrix ApplyMapBuilderOutput(const MapBuilderOutput& o);

  const c8::HMatrix& pose() const;
  frame_frame_tracker::FrameFrameTracker& frame_tracker();

 private:
  // Owned by WP2. State (S2 §2.1 layout) lives in map_tracker/map_tracker_internal.h.
  // std::unique_ptr: <memory> included explicitly (WP2-CC-4).
  friend struct MapTrackerAccess;
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

// 1272. Calls 1146 with the public parameter struct defaults
// {suppressOutliers = 1, scale = 1.0, useImageTargetCostFunction = 0, cameraMotionWeight = 0.0,
//  logReport = 0} and an empty lastTwo deque (VERIFIED, WP2-PRE ③).
void TrackPoints(const map::WorldMap& map, const TrackerInput& frame, const c8::HMatrix& pose,
                 const std::vector<map::PointId>& ids, MapTrackResult* result);

// 1274 (cloud-empty path).
void Relocalize(RelocResult* out, const map::WorldMap& map, const TrackerInput& frame,
                const c8::HMatrix& tracking_pose, compat::RandomNumbers* rng);

// 519 (Q6).
float YawOf(c8::Quaternion q);

// 1271.
c8::HMatrix ReplaceYaw(const c8::HMatrix& pose, const c8::Quaternion& q_dev);

}  // namespace slam_refact::map_tracker

#endif  // SLAM_REFACT_MAP_TRACKER_MAP_TRACKER_H_

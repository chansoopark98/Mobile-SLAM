// MapTracker state and module-internal declarations (WP2 private header; not part of the frozen
// contract). Field meaning follows the original object (S2 §2.1, G640, F926); offsets are kept as
// comments only.

#ifndef SLAM_REFACT_MAP_TRACKER_MAP_TRACKER_INTERNAL_H_
#define SLAM_REFACT_MAP_TRACKER_MAP_TRACKER_INTERNAL_H_

#include <cfloat>
#include <cstdint>
#include <map>
#include <vector>

#include "c8/geometry/pixel-pinhole-camera-model.h"
#include "c8/hmatrix.h"
#include "c8/hpoint.h"
#include "c8/quaternion.h"
#include "common/tracking_types.h"
#include "compat/random_numbers.h"
#include "frame_frame_tracker/frame_frame_tracker.h"
#include "map/world_map.h"
#include "map_tracker/map_tracker.h"
#include "reality/engine/features/local-matcher.h"
#include "reality/engine/geometry/bundle.h"
#include "reality/engine/geometry/observed-point.h"
#include "reality/engine/geometry/pose-pnp.h"

namespace slam_refact::map_tracker {

// One frame-point <-> map-point association (12 B record {u32 frameIdx; WorldMapPointId}).
struct TrackedMatch {
  uint32_t frame_index = 0;
  map::PointId id;
};

// trackPoints (1272) result, S2 §2.5 (MapTracker+0x1320).
struct MapTrackResult {
  c8::HMatrix pose = c8::HMatrixGen::i();        // +0x00
  float residual = FLT_MAX;                      // +0x84 (§5.2), FLT_MAX on failure
  // +0x88 / +0x8c: caller-provided outputs, cleared at the start of every call (F1272:248-255).
  std::vector<map::PointId>* visible_out = nullptr;
  std::vector<uint8_t>* matched_out = nullptr;
  std::vector<map::PointId> matched_ids;         // +0x90 -> last_tracked_ids_
  std::map<uint32_t, map::PointId> dict_map;     // +0x9c dictionaryIdx -> id
  std::vector<c8::ObservedPoint> all_obs;        // +0xa8
  std::vector<TrackedMatch> all_matches;         // +0xb4
  // Per-match locals of 1272 kept for the subset step: word (projected point) index and the world
  // point projected.extrinsic() * position3() (F1272:1121-1141).
  std::vector<uint32_t> word_indices;
  std::vector<c8::HPoint3> all_world_points;
  std::vector<c8::ObservedPoint> subset_obs;     // +0xc0
  std::vector<TrackedMatch> subset_matches;      // +0xcc
  std::vector<c8::HPoint3> subset_world_points;  // +0xd8
  c8::PoseEstimationResidualOutput bundle_out;   // +0xe4
  uint32_t num_loaded = 0;                       // +0x114
  c8::MatchResultFrequency frequency;            // +0x118 GORB + ORB
  map::ProjectedPoints projected;                // +0x160 (1548 output)
};

// 1273: the tracked bimap (MapTracker +0x12cc frameIdx -> id, +0x12d8 id -> frameIdx).
struct TrackedBimap {
  std::map<uint32_t, map::PointId> by_frame;
  std::map<map::PointId, uint32_t> by_id;

  void Clear() {
    by_frame.clear();
    by_id.clear();
  }
  // 1273: an id that is already present drops its old pair first (by_frame entry of its old frame
  // index, then its by_id entry); then both sides are inserted only if their key is absent.
  void Insert(uint32_t frame_index, map::PointId id);
};

struct MapTracker::Impl {
  frame_frame_tracker::FrameFrameTracker frame_tracker;   // +0x0088
  c8_PixelPinholeCameraModel intrinsic{};                  // +0x0ff8
  c8::HMatrix pose = c8::HMatrixGen::i();                  // +0x1010
  c8::HMatrix anchor_pose = c8::HMatrixGen::i();           // +0x1094
  c8::HMatrix frame_start_pose = c8::HMatrixGen::i();      // +0x1118
  int64_t frame_count = -1;                                // +0x11a0
  int64_t last_state_change_frame = 0;                     // +0x11a8
  int64_t consecutive_frame_tracker_ok = 0;                // +0x11b0
  c8::Quaternion prev_device_rotation;                     // +0x11b8
  std::vector<c8::HPoint3> last_world_points;              // +0x11c8
  std::vector<map::PointId> last_tracked_ids;              // +0x11d4
  // G640 (MapTracker ctor) stores the u64 0xbf8000003f800000 at +0x11e0: ground_height_ = 1.0f,
  // ground_y_ = -1.0f, so the FFT scale pose.ty - ground_y_ starts at ty + 1 (S2 §2 / table at 745).
  float ground_height = 1.0f;                              // +0x11e0
  float ground_y = -1.0f;                                  // +0x11e4
  bool extreme_motion = false;                             // +0x11e8
  float last_good_residual = 0.0f;                         // +0x11ec
  float lost_residual = 0.0f;                              // +0x11f0
  int32_t state = 0;                                       // +0x11f4 0 = LOST, 1 = TRACKING
  bool frame_tracker_ok = false;                           // +0x11f8
  c8::HMatrix frame_delta = c8::HMatrixGen::i();           // +0x11fc
  std::vector<uint32_t> ground_idx;                        // +0x1280
  std::vector<uint32_t> sky_idx;                           // +0x128c
  std::vector<uint32_t> depth_idx;                         // +0x1298
  map::KeyframeId reloc_kf_id;                             // +0x12a4
  bool reloc_kf_pending = false;                           // +0x12ac
  bool map_initialized_pending = false;                    // +0x12b0
  std::vector<c8::ObservedPoint> all_obs;                  // +0x12b4 (1260)
  std::vector<TrackedMatch> all_matches;                   // +0x12c0 (1260)
  TrackedBimap tracked;                                    // +0x12cc / +0x12d8
  std::vector<c8::ObservedPoint> subset_obs;               // +0x12e4 (1260)
  std::vector<TrackedMatch> subset_matches;                // +0x12f0 (1260)
  MapTrackResult track_result;                             // +0x1320
  int64_t last_reloc_time_ns = 0;                          // +0x1848
  // 1265 RobustP2PScratchSpace: a process-wide static in the original (Q16); owned per MapTracker
  // here (see ComputeFramePoseUsingRobustP2P in relocalize.cc for why the output is the same).
  c8::RobustP2PScratchSpace p2p_scratch;
};

struct MapTrackerAccess {
  using Impl = MapTracker::Impl;
  static Impl& impl(MapTracker& t) { return *t.impl_; }
  static const Impl& impl(const MapTracker& t) { return *t.impl_; }
};

// 1265 computeFramePoseUsingRobustP2P. out->pose = tracking_pose; robustP2P (1025) with the
// WorldMaps RANSAC settings and the caller's scratch space (process-wide static in the original,
// per MapTracker here); on success the inliers are copied and out->pose = the RANSAC pose.
void ComputeFramePoseUsingRobustP2P(const TrackerInput& frame,
                                    const std::vector<c8::HPoint3>& world_points,
                                    const std::vector<map::PointId>& ids,
                                    const std::vector<c8::HPoint2>& rays,
                                    const c8::HMatrix& tracking_pose, compat::RandomNumbers* rng,
                                    c8::RobustP2PScratchSpace* scratch, RelocResult* out);

// 1274 with an explicit robustP2P scratch (MapTracker passes its own, Impl::p2p_scratch).
void Relocalize(RelocResult* out, const map::WorldMap& map, const TrackerInput& frame,
                const c8::HMatrix& tracking_pose, compat::RandomNumbers* rng,
                c8::RobustP2PScratchSpace* scratch);

// F926 small-motion gate inputs, in the f926 wasm's float operation order:
//   translation       |frame_delta translation| = sqrt(z^2 + (y^2 + (x^2 + 0)))
//   device_rotation2  |vec(d)|^2 of d = normalize(normalize(prev)^-1 * rotation), z^2 + (x^2 + y^2)
//   visual_rotation2  |vec(v)|^2 of v = Quaternion::fromHMatrix(frame_delta) (520), z^2 + (x^2 + y^2)
struct MotionGate {
  float translation = 0.0f;
  float device_rotation2 = 0.0f;
  float visual_rotation2 = 0.0f;
};
MotionGate ComputeMotionGate(const c8::HMatrix& frame_delta, const c8::Quaternion& prev_rotation,
                             const c8::Quaternion& rotation);

// §5.4 map-points-subset (F1272:1460-2007): fills result->subset_* from result->all_*.
void SelectSubset(const map::WorldMap& map, MapTrackResult* result);

// §5.6 ground-height window (F926:2440-2503): new low-passed ground height, or nothing when the
// sample counts are below S.minGroundPointsForScale. `heights` is sorted in place
// (std::stable_sort, 1257).
bool UpdateGroundHeight(std::vector<float>* heights, size_t ground_candidates, float pose_ty,
                        float* ground_height);

}  // namespace slam_refact::map_tracker

#endif  // SLAM_REFACT_MAP_TRACKER_MAP_TRACKER_INTERNAL_H_

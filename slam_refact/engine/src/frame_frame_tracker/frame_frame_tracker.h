// FrameFrameTracker module public header (architecture.md §3.5; S4 §3.7).
//
// FROZEN CONTRACT (WP0b): every declaration in this file. Changes follow
// slam_refact/docs/design/contract-changes.md. `private:` sections are owned by WP2.
//
// Not frozen here (signature not fixed by the design; WP2 declares it in a private header):
// RobustGroundHomography (1279). Its RNG use is resolved by WP2-PRE (docs/re/04-wp2-pre.md §1.3):
// 1266/1279 draw from the FFT's own compat::RandomNumbers via libc++ std::shuffle (805) and
// nextUnsignedInt() % n (895).

#ifndef SLAM_REFACT_FRAME_FRAME_TRACKER_FRAME_FRAME_TRACKER_H_
#define SLAM_REFACT_FRAME_FRAME_TRACKER_FRAME_FRAME_TRACKER_H_

#include <cstddef>
#include <cstdint>
#include <memory>
#include <vector>

#include "c8/hmatrix.h"
#include "c8/hpoint.h"
#include "c8/quaternion.h"
#include "compat/random_numbers.h"
#include "reality/engine/features/frame-point.h"

namespace slam_refact::frame_frame_tracker {

// Held by value inside MapTracker (MapTracker+0x88).
class FrameFrameTracker {
 public:
  // LocalMatcher x2, RobustHomographyScratchSpace, compat::RandomNumbers seeded 5489 (a full
  // c8::RandomNumbers object, 2552 B at +0x158, just before height_; WP2-PRE ①),
  // height_ = 1.0f (G640).
  FrameFrameTracker();
  ~FrameFrameTracker();

  // 1266. delta_out = previous camera -> current camera egomotion. On failure
  // delta = (R(normalize(q_prev^-1 * q_cur)), t = 0).
  void Track(c8::HMatrix* delta_out, const c8::FrameWithPoints& cur, const c8::Quaternion& q_cur,
             std::vector<uint32_t>* ground_idx, std::vector<uint32_t>* sky_idx,
             std::vector<uint32_t>* depth_idx, bool* success);

  // Camera height above ground (+0xb50), updated by MapTracker every frame (S2 §5.6).
  float height() const;
  void set_height(float height);

  // 913 / 1229 (map-reset event of 664).
  void ResetPartial();

  // Previous-frame map depth per frame point index (+0xf18), filled by MapTracker.
  void SetPrevDepths(const std::vector<float>& depth_by_frame_point);

  // Debug getter (TrackDebug::ground_matches): ground<->ground matches of the last Track.
  int last_ground_match_count() const;

  // The FFT's own generator (+0x158). Exposed for 1279 and for reseed checks (SetDisableVio /
  // core recreation, architecture.md §5.2, M12).
  compat::RandomNumbers* rng();

 private:
  // Owned by WP2. State (G640 / G1266 layout) lives in frame_frame_tracker_internal.h.
  // std::unique_ptr: <memory> included explicitly (WP2-CC-4).
  friend struct FrameFrameTrackerAccess;
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

// 1267: horizon class (1..5) per ray for device rotation q. Elements are int32 (4 B per element
// in the binary; WP2-PRE §1.4).
std::vector<int32_t> ClassifyHorizon(const c8::Quaternion& q, const std::vector<c8::HPoint3>& rays);

// 1269.
std::vector<uint32_t> DepthPointIndices(size_t n);

}  // namespace slam_refact::frame_frame_tracker

#endif  // SLAM_REFACT_FRAME_FRAME_TRACKER_FRAME_FRAME_TRACKER_H_

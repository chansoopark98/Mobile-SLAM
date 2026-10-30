// FrameFrameTracker state (WP2 private header). Field order and comments follow the original
// object (G640 constructor, G1266 / DCMP func1266 accesses; offsets relative to the FFT, which is
// MapTracker+0x88).

#ifndef SLAM_REFACT_FRAME_FRAME_TRACKER_FRAME_FRAME_TRACKER_INTERNAL_H_
#define SLAM_REFACT_FRAME_FRAME_TRACKER_FRAME_FRAME_TRACKER_INTERNAL_H_

#include <algorithm>
#include <cfloat>
#include <cstddef>
#include <cstdint>
#include <utility>
#include <vector>

#include "c8/geometry/homography.h"
#include "c8/hpoint.h"
#include "c8/quaternion.h"
#include "compat/random_numbers.h"
#include "frame_frame_tracker/frame_frame_tracker.h"
#include "reality/engine/features/frame-point.h"
#include "reality/engine/features/local-matcher.h"

namespace slam_refact::frame_frame_tracker {

// Model of the original's std::vector<float> at +0xf18 (libc++), including the storage past the
// logical size. 1266 reads depths[dictionaryIdx] unchecked (G1266:454-459, :920), and the list
// can be shorter than the previous-frame point list (stale list, see Track). An index in
// [size, capacity) reads the float a longer earlier list left in the vector's storage, and that
// value decides how many depth matches there are (RNG draws, fftHeight; flip6-s2 frame 326).
// Operations the original performs on +0xf18 (all verified in the Ghidra export):
//  - F926:2586-2592 (MapTracker tracked frame): end = begin (clear()); then, only if the
//    previous-frame point list is non-empty, func1276(vec, n, &-FLT_MAX) = libc++
//    vector<float>::__append(n, -FLT_MAX) on the empty vector; MapTracker then overwrites the
//    entries of matched points. n <= capacity: the first n storage slots are rewritten and the
//    tail keeps its old floats. n > capacity: new capacity = max(2 * capacity, n) (G1276
//    uVar9 = cap_bytes >> 1 vs size + n), a fresh allocation whose tail is uninitialized heap in
//    the original.
//  - G1266:738 / :1746: end = begin (clear()); storage and capacity untouched.
// Nothing else writes the list (G664 does not touch it).
// Deterministic across libstdc++ / libc++: reads never go beyond the modeled storage_.
// UNVERIFIED residual: the uninitialized tail of a fresh allocation and any index >= capacity are
// unreproducible heap reads in the original; both are modeled as "no depth" (-FLT_MAX).
class PrevDepthList {
 public:
  // 926: clear(), then resize(n, -FLT_MAX), then the per-point depths (values.size() == n).
  void Assign(const std::vector<float>& values) {
    const size_t n = values.size();
    size_ = 0;
    if (n > storage_.size()) {
      storage_.assign(std::max(2 * storage_.size(), n), -FLT_MAX);
    }
    std::copy(values.begin(), values.end(), storage_.begin());
    size_ = n;
  }
  // 1266 (G1266:738, :1746): end = begin.
  void clear() { size_ = 0; }
  bool empty() const { return size_ == 0; }
  size_t size() const { return size_; }
  size_t capacity() const { return storage_.size(); }
  // depths[idx] as the original reads it; -FLT_MAX (no depth) past the modeled capacity.
  float Depth(size_t idx) const { return idx < storage_.size() ? storage_[idx] : -FLT_MAX; }
  // Storage access for state inspection (harness diagnostics): data()/operator[] cover the whole
  // modeled capacity, operator= is Assign().
  const float* data() const { return storage_.data(); }
  float operator[](size_t idx) const { return storage_[idx]; }
  PrevDepthList& operator=(const std::vector<float>& values) {
    Assign(values);
    return *this;
  }
  // Logical contents (size() floats), for state inspection.
  std::vector<float> values() const { return {storage_.begin(), storage_.begin() + size_}; }
  // Let the logical list compare with / convert to a plain vector (existing state checks).
  operator std::vector<float>() const { return values(); }
  friend bool operator==(const PrevDepthList& a, const std::vector<float>& b) {
    return a.values() == b;
  }

 private:
  std::vector<float> storage_;  // length == modeled capacity
  size_t size_ = 0;
};

struct FrameFrameTracker::Impl {
  c8::LocalMatcher<c8::OrbFeature> orb_matcher;      // +0x00 (BinMap 15x20, r 0.1, dist 64)
  c8::LocalMatcher<c8::GorbFeature> gorb_matcher;    // +0x58
  c8::RobustHomographyScratchSpace scratch;          // +0xb0
  compat::RandomNumbers rng{compat::kEngineRngSeed};  // +0x158 (2552 B c8::RandomNumbers)
  float height = 1.0f;                               // +0xb50
  c8::FrameWithPoints prev_frame{c8_PixelPinholeCameraModel{}};  // +0xb54
  c8::Quaternion prev_q_a;    // +0xeec: ground solver q_prev, height-update rotation
  c8::Quaternion prev_q_b;    // +0xefc: relative rotation of depth egomotion and fallback
  std::vector<c8::HPoint2> prev_rays;      // +0xf0c (x, y, 1)
  PrevDepthList prev_depths;               // +0xf18 per previous-frame point, -FLT_MAX = none
  std::vector<c8::PointMatch> depth_matches;  // +0xf24
  std::vector<int32_t> prev_classes;       // +0xf30 horizon classes of the previous frame
  bool last_success = false;               // +0xf3c
  float last_translation = 0.0f;           // +0xf40 |t| of the delta before the fallback
  float last_rotation = 0.0f;              // +0xf44 2 * acos(w) of the final delta
  int32_t call_count = 0;                  // +0xf48
  std::vector<c8::PointMatch> matches;         // +0xf4c (words = current, dictionary = previous)
  std::vector<c8::PointMatch> ground_matches;  // +0xf58 class 1 in both frames
  std::vector<c8::PointMatch> sky_matches;     // +0xf64 class 5 in both frames
};

// Module-internal access (MapTracker's inertial path updates the same state, F926:1227-1251).
struct FrameFrameTrackerAccess {
  using Impl = FrameFrameTracker::Impl;
  static Impl& impl(FrameFrameTracker& t) { return *t.impl_; }
  static const Impl& impl(const FrameFrameTracker& t) { return *t.impl_; }
};

// 1267 on HPoint2 rays (x, y, w): the binary extrudes each ray to (x, y, w, 1) first.
std::vector<int32_t> ClassifyHorizonRays(const c8::Quaternion& q,
                                         const std::vector<c8::HPoint2>& rays);

// Depth-egomotion hypothesis pairs: all (i < j), i outer, shuffled with libc++ std::shuffle
// (805) on the FFT generator (WP2-PRE §1.3 (a)).
std::vector<std::pair<uint32_t, uint32_t>> ShuffledDepthPairs(uint32_t n,
                                                              compat::RandomNumbers* rng);

}  // namespace slam_refact::frame_frame_tracker

#endif  // SLAM_REFACT_FRAME_FRAME_TRACKER_FRAME_FRAME_TRACKER_INTERNAL_H_

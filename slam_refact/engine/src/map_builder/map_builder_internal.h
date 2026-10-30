// MapBuilder module-internal declarations (WP3 private header; not part of the frozen contract).
// Policy predicates of 936 split out so the S3 §8.2 boundary tests (MB-UNIT) can call them, the
// closed epipolar helpers of 1112 (1122/1123, the sort comparator of 1120/1121) for MB-EPI, and
// the lazily built per-keyframe matchers (S3 §2.3).
//
// Citations: <N>.c:L = build/slam-refact-20261015/spec/map-builder/annotated/<N>.c line L (Ghidra
// C), DCMP = build/slam-refact-20261015/re/wabt/xr-slam.dcmp, OBJ = wasm-objdump disassembly.

#ifndef SLAM_REFACT_MAP_BUILDER_MAP_BUILDER_INTERNAL_H_
#define SLAM_REFACT_MAP_BUILDER_MAP_BUILDER_INTERNAL_H_

#include <array>
#include <cstddef>
#include <cstdint>
#include <map>
#include <vector>

#include "c8/hmatrix.h"
#include "c8/hpoint.h"
#include "common/tracking_types.h"
#include "map/world_map.h"
#include "map_builder/map_builder.h"
#include "reality/engine/binning/linear-bin.h"
#include "reality/engine/features/frame-point.h"

namespace slam_refact::map_builder {

// ---- 936 [A] map reset (936.c:696-731) ----------------------------------------------------------

struct ResetInputs {
  int32_t state = 0;                    // trk.state (+0x84)
  bool frame_tracker_ok = false;        // trk.valid (+0x88)
  size_t num_candidates = 0;            // |trk.ground_idx| (+0x98)
  int64_t frame_count = 0;              // MapBuilder frame_count_ (+0xa10), already incremented
  int64_t last_state_change_frame = 0;  // +0xa18
  size_t keyframe_slots = 0;            // raw keyframe slot count incl. free slots (wm +0x0c)
  int64_t consecutive_frame_tracker_ok = 0;  // trk +0x90
};

// The tracking-statistics part of the reset decision (the map/VPS gates are checked by Update).
bool ShouldResetMap(const ResetInputs& in);

// ---- 936 [C] keyframe decision (936.c:770-958) --------------------------------------------------

// Gyroscope events (kind 3) with |w| > max_deg_s (strict; deg/s). 936.c:906-913.
bool AnyGyroAbove(const std::vector<TrackingSensorEvent>& events, float max_deg_s);

// Number of entries of `indices` that are not keys of `matches` (936.c:792-884).
int CountNotIn(const std::vector<uint32_t>& indices, const std::map<uint32_t, map::PointId>& matches);

// (frame_count - last_reset >= relocalizationDelay) && since_change < relocalizationDelay
// (936.c:895-903); since_change is the int32 difference used by the binary.
bool RelocalizationHold(int64_t frame_count, int64_t last_reset_frame,
                        int64_t last_state_change_frame);

struct KeyframeDecisionInputs {
  bool map_reset = false;               // out->map_reset of this frame
  int32_t state = 0;                    // trk.state
  const std::vector<TrackingSensorEvent>* imu = nullptr;  // trk +0x198
  int64_t frame_count = 0;
  int64_t last_state_change_frame = 0;
  int64_t last_reset_frame = 0;
  int64_t last_keyframe_frame = 0;      // wm +0x1a0
  bool need = false;                    // NeedKeyframe(tracked, n_new)
};

// true = CREATE, false = NO_KF. force_keyframe_ (VPS) and the cloud-map branch are not part of
// world SLAM and are handled by Update.
bool DecideKeyframe(const KeyframeDecisionInputs& in);

// ---- 936 [D] ground plane y = -1 (936.c:1998-2170, DCMP:105272-105277) ---------------------------

struct GroundPlane {
  // Plane through the three world points (0,-1,0), (1,-1,0), (0,-1,1) in camera coordinates:
  // normal n and the first point p0.
  float nx = 0.f, ny = 0.f, nz = 0.f;
  float p0x = 0.f, p0y = 0.f, p0z = 0.f;
};

GroundPlane GroundPlaneInCamera(const c8::HMatrix& world_from_camera);

// X_w = T_wc * (s * (x, y, 1), 1), s = n.p0 / n.r (n.r == 0 -> denominator 1). No sign check
// (quirk Q7).
c8::HPoint3 GroundPointInWorld(const GroundPlane& plane, const c8::HMatrix& world_from_camera,
                               float ray_x, float ray_y);

// Ground candidates that survive "remove-ground-points-behind-triangulated" (936.c:1152-2040).
// Returns the input order when no triangulated point qualifies, else the surviving indices in
// ascending order.
std::vector<uint32_t> PruneGroundCandidates(const map::WorldMap& wm, map::KeyframeId kf,
                                            const std::vector<uint32_t>& candidates);

// ---- 936 [F] pair queue (936.c:2678-2705) ---------------------------------------------------------

// If |queue| > 0.9 * maxPairSize (double compare): drop the oldest maxPairSize / 3 entries when
// maxPairSize + 2 > 4.
void TrimPairQueue(std::vector<KfPair>* queue, int max_pair_size);

// ---- 936 [G] prune-keyframes (936.c:2869-3286) ------------------------------------------------------

// Removal test of pass 1: redundancyPointPercentage * total < redundant (strict, float).
bool IsKeyframeRedundant(int total, int redundant, float percentage);

// ---- 1112 helpers -----------------------------------------------------------------------------

// func 1122 output (544 B in wasm32). Field offsets are the wasm32 ones; values are float32 like
// the original. `first`/`second` are the two pose arguments of 1122.
struct RayPairPrework {
  std::array<float, 16> essential{};     // +0   essentialMatrixForCameras(first, second), col-major
  std::array<float, 16> essential_t{};   // +132 its transpose (HMatrix::t of a noInvert matrix)
  std::array<float, 16> rotation_first{};  // +264 rotationMat(first), col-major
  std::array<float, 3> center_first{};   // +396 translation(first)
  std::array<float, 16> second_block{};  // +412 [R_second | 0; -(R_second^T d)^T 1], d = c2 - c1
};

// 1122 (DCMP:138687-138903), arithmetic order as decompiled.
RayPairPrework MakeRayPairPrework(const c8::HMatrix& first, const c8::HMatrix& second);

// 1123 (OBJ func[1123], DCMP:138904-138978): point on the ray of the second camera built from the
// prework and both rays (each (x, y, z), divided by z). w of the result is 1.
c8::HPoint3 RayPairPoint(const c8::HPoint3& ray_first, const c8::HPoint3& ray_second,
                         const RayPairPrework& prework);

// Comparator of the sort-and-filter-duplicates sort (1120/1121): ascending
// (descriptorDist, dictionaryScale, wordsIdx, dictionaryIdx).
bool EpipolarMatchLess(const c8::PointMatch& a, const c8::PointMatch& b);

// libc++ std::sort with EpipolarMatchLess (1120 for GORB, 1121 for ORB; same comparator).
void SortEpipolarMatches(std::vector<c8::PointMatch>* matches);

// Lazily built epipolar matcher stored on the dictionary keyframe (+0x3c4 ORB, +0x3c8 GORB,
// 0x60 B; 1112.c:313-375): BinMap 15x20 over the keyframe frame bounds excluding the edge,
// indexed with the keypoints of one descriptor type; radius copied from
// EpipolarMatcher.radius at creation; ratio test on.
struct EpipolarKeyframeMatcher {
  const c8::FrameWithPoints* dictionary = nullptr;
  bool use_ratio_test = true;
  c8::BinMap map{15, 20};
  float radius = 0.f;
  c8::Vector<size_t> bins;
};

}  // namespace slam_refact::map_builder

#endif  // SLAM_REFACT_MAP_BUILDER_MAP_BUILDER_INTERNAL_H_

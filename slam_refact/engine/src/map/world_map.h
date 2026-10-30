// World map data model public header (architecture.md §3.3; S2 §2.3, §3.3, S3 §2.2-2.5,
// §3.7-3.8, §7.2).
//
// FROZEN CONTRACT (WP0b): every declaration in this file. Changes follow
// slam_refact/docs/design/contract-changes.md. `private:` sections and the members after the
// "WP2-owned tail" markers are owned by WP2 (CONTRACTS.md).
//
// What must match the original is iteration order, not memory layout (§3.3): pools iterate in
// slot order; maps/sets of ids iterate in PointId/KeyframeId operator< order; AllPointIds() is
// pool order TRIANGULATED -> GROUND -> SKY -> DEPTH.

#ifndef SLAM_REFACT_MAP_WORLD_MAP_H_
#define SLAM_REFACT_MAP_WORLD_MAP_H_

#include <array>
#include <cstdint>
#include <map>
#include <memory>
#include <vector>

#include "c8/geometry/pixel-pinhole-camera-model.h"
#include "c8/hmatrix.h"
#include "c8/hpoint.h"
#include "reality/engine/features/frame-point.h"
#include "slam_refact/types.h"
#include "common/tracking_types.h"

namespace slam_refact::map {

// Map point (0x48 B record; S2 §2.3, S3 §2.5).
struct WorldPoint {
  uint32_t birth_kf_seq = 0;    // +0x00 keyframe sequence at creation (1562 age)
  uint32_t id = 0;              // +0x04
  c8::HPoint3 position;         // +0x08 float
  PointState state = PointState::kNew;  // +0x18
  std::map<KeyframeId, uint8_t> scale_by_kf;    // +0x1c
  std::map<KeyframeId, uint32_t> observations;  // +0x28 keyframe -> observation index
  uint32_t tracking_hits = 0;   // +0x34 found (inlier) count
  uint32_t tracking_count = 0;  // +0x38 visible (projected) count
  uint8_t min_scale = 0xff;     // +0x3c
  PointSource source = PointSource::kTriangulated;  // +0x40
  bool loaded_from_map = false; // +0x44
  // ---- WP2-owned tail: WP2 may append members below this line ----
};

// Keyframe (0x3d8 B record; S2 §2.3, S3 §2.3). The keyframe pose is frame.extrinsic().
struct Keyframe {
  uint32_t seq = 0;             // +0x000
  int64_t creation_frame = 0;   // +0x008 MapBuilder frame_count_ (sort key of 1142/1143)
  uint32_t id = 0;              // +0x010 0 = empty slot
  c8::FrameWithPoints frame{c8_PixelPinholeCameraModel{}};  // +0x014
  uint8_t prune_protect = 0;    // +0x3d0 (INFERRED)
  // wm+0x184 slot entry (654 -> 1263): agate global descriptor of this slot, written by the
  // MapBuilder only when MapBuilder.loopCloseEnabled is on (US-025, US025-CC-1; all zero
  // otherwise). It is not erased when the keyframe is removed, like the original.
  std::array<uint8_t, 32> global_descriptor{};
  // ---- WP2-owned tail: lazy matcher caches (S3 §2.3: match-new-points LocalMatcher(0.05, 32),
  // epipolar matcher) are appended below this line by WP2 ----
  // Lazily built matchers over this keyframe (S3 §2.3). Type-erased so this header does not pull
  // in matcher headers; the MapBuilder (WP3) creates and casts them. WorldMap releases all four
  // when the slot is reused (654) or the keyframe is removed (1144), like the original.
  std::shared_ptr<void> new_points_matcher_orb;   // +0x3b8 LocalMatcher<OrbFeature>(…, 0.05, 32)
  std::shared_ptr<void> new_points_matcher_gorb;  // +0x3bc LocalMatcher<GorbFeature>(…, 0.05, 32)
  std::shared_ptr<void> epipolar_matcher_orb;     // +0x3c4 epipolar matcher, BinMap 15x20
  std::shared_ptr<void> epipolar_matcher_gorb;    // +0x3c8
};

// verifyPointVisibility (1549) per-point view: keyframe and observation index in it.
struct PointView {
  KeyframeId kf;
  uint32_t point_index = 0;
};

// 1549 result, indexed like the input ids (S2 §3.3).
// status: 0 = not considered: invalid id or DEAD, or inside the frustum but with no observation
//             (zero observations, or every observing keyframe excluded by the viewing-angle gate
//             maxViewingAngleKfToCam);
//         1 = visible, 2 = outside frustum/behind,
//         3 = an observation passed the angle gate but failed the scale range, and no observation
//             has been accepted (WP2-CC-3, DCMP func1549 per-point loop).
struct VisibilityResult {
  std::vector<uint8_t> status;
  std::vector<uint8_t> scale;                  // 0xff when not visible
  std::vector<std::vector<PointView>> views;   // admissible observations per point
};

// 1548 result: a synthesized frame (intrinsics, pose) holding one world-map point per visible
// map point; ids are frame.points()[i].id() (S2 §3.3, §2.5 +0x160).
struct ProjectedPoints {
  c8::FrameWithPoints frame{c8_PixelPinholeCameraModel{}};
};

class WorldMap {
 public:
  WorldMap();
  ~WorldMap();

  WorldMap(const WorldMap&) = delete;
  WorldMap& operator=(const WorldMap&) = delete;

  KeyframeId AddKeyframe(const c8::FrameWithPoints& f, int64_t frame_count);         // 654
  void SetKeyframePose(KeyframeId kf, const c8::HMatrix& world_from_camera);         // 655
  PointId AddPoint(uint32_t kf_seq, const c8::HPoint3& p, PointSource source);       // 657
  void AddObservation(PointId point, uint32_t obs_idx, KeyframeId kf);               // 658 -> 4074

  // 634 / 1125. A stale or invalid id aborts (SR_CHECK).
  const WorldPoint& Point(PointId id) const;
  WorldPoint& MutablePoint(PointId id);

  // Keyframe access with the same abort rule (WP0b addition: needed by 1548/1549 and S3).
  const Keyframe& GetKeyframe(KeyframeId id) const;
  Keyframe& MutableKeyframe(KeyframeId id);

  // Slot-level and non-aborting access (WP2-CC-1, WP3-CC-1; kContractVersion 2).
  // 1150 setPointPosition: writes the position, recomputes the depth of every observation and
  // unlinks observations whose ray residual exceeds maxInternalSquaredResidualForTriangulatedPts.
  void SetPointPosition(PointId id, const c8::HPoint3& position);
  // Keyframe slot count including free slots (+0x0c vector size, S3 §3 [A]).
  size_t NumKeyframeSlots() const;
  // Keyframe in a slot, free slots (id 0) included; aborts only when slot >= NumKeyframeSlots().
  const Keyframe& KeyframeAtSlot(size_t slot) const;
  Keyframe& MutableKeyframeAtSlot(size_t slot);
  // id != 0, slot in range and slot id equal (1112.c:248-262).
  bool IsKeyframeValid(KeyframeId id) const;
  // 634 lookup without abort: nullptr when no pool slot holds this id (state not checked).
  const WorldPoint* FindPoint(PointId id) const;
  // Inserts into the dirty point set (+0x1b0) processed by AdjustDirtyPoints.
  void MarkPointDirty(PointId id);

  void Retire(PointId id);                         // 1130
  void MergePoint(PointId src, PointId dst);       // 1124
  void AdjustDirtyPoints();                        // 1127 / 1150
  void RemoveKeyframe(KeyframeId id);              // 1144
  std::vector<KeyframeId> AllKeyframesByCreation() const;                         // 1142
  std::vector<KeyframeId> CovisibleKeyframes(KeyframeId kf, int min_shared = 10) const;  // 1143
  std::vector<PointId> AllPointIds() const;        // 1147: TRI, GROUND, SKY, DEPTH
  void CullAndTenureAll();                         // 1141 -> 1562

  // World SLAM never loads cloud points.
  bool CloudEmpty() const { return true; }

  uint32_t keyframe_seq_counter() const;           // +0x08
  int64_t last_keyframe_frame() const;             // +0x1a0
  KeyframeId newest() const;                       // +0x1a8

  // Debug getters for TrackDebug (§3.7).
  int NumKeyframes() const;
  // Live (not DEAD) points per source, indexed by PointSource - 1.
  std::array<int, 5> NumPointsBySource() const;

 private:
  // Owned by WP2. State lives in map/world_map_internal.h (pools, keyframe slots, dirty set, id
  // generator); WorldMapAccess is the module-internal accessor used by map/ and map_tracker/.
  friend struct WorldMapAccess;
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

// 1549.
VisibilityResult VerifyPointVisibility(const WorldMap& map, const PinholeIntrinsics& intrinsics,
                                       const c8::HMatrix& pose, const std::vector<PointId>& ids);

// 1548. Empty ids = every map point (1147 order).
ProjectedPoints ProjectPoints(const WorldMap& map, std::vector<PointId> ids,
                              const PinholeIntrinsics& intrinsics, const c8::HMatrix& pose);

// Owns the active WorldMap; ResetMap() = 653; generates map ids.
class WorldMaps {
 public:
  WorldMaps();
  ~WorldMaps();

  WorldMaps(const WorldMaps&) = delete;
  WorldMaps& operator=(const WorldMaps&) = delete;

  WorldMap& active();
  const WorldMap& active() const;

  // 653: replaces the active map with a new empty one.
  void ResetMap();

 private:
  // Owned by WP2: the shared point/keyframe/map id generator and the active map.
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace slam_refact::map

#endif  // SLAM_REFACT_MAP_WORLD_MAP_H_

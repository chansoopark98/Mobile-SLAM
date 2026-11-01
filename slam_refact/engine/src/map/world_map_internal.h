// WorldMap implementation state and module-internal accessors (WP2 private header; not part of
// the frozen contract). Layout follows the original WorldMap (S3 §2.2, S2 §2.3) by meaning:
//   point pools TRIANGULATED/GROUND/SKY/DEPTH/CLOUD (+0x24/+0x48/+0x6c/+0x90/+0xb4), each with a
//   point vector, a per-slot side record (majority descriptor, 0x430 B; pools 1-4 only) and a free
//   slot list; keyframe slots (+0x0c) with free list (+0x18); keyframe sequence counter (+0x08);
//   last keyframe frame (+0x1a0); newest keyframe (+0x1a8); dirty point set (+0x1b0).
//
// Id generator (VERIFIED: G641/DCMP:50355-50400, 653.c, 654.c, 657.c): map, keyframe and point
// ids are draws of uniform_int_distribution<uint32_t>(0, 0xffffffff) (= raw mt19937 output,
// WP2-PRE §1.2 Rp = 0) from a mt19937 owned by the MapBuilder and seeded with getentropy(), i.e.
// the original ids are random per run. DECISION (WP2): the engine seeds this generator with the
// fixed kIdGeneratorSeed so runs are reproducible; XrId ordering (id, index) therefore gives a
// deterministic but original-like (random) iteration order.

#ifndef SLAM_REFACT_MAP_WORLD_MAP_INTERNAL_H_
#define SLAM_REFACT_MAP_WORLD_MAP_INTERNAL_H_

#include <array>
#include <cstddef>
#include <cstdint>
#include <random>
#include <set>
#include <utility>
#include <vector>

#include "c8/hpoint.h"
#include "map/world_map.h"

namespace slam_refact::map {

inline constexpr uint32_t kIdGeneratorSeed = 5489u;

// Point side record (0x430 B; 657.c, 658.c, 1550): one counts of the GORB descriptor bits of every
// observation, the observation count, and a lazily recomputed majority descriptor.
struct MajorityRecord {
  std::array<size_t, 256> one_counts{};  // +0x000
  uint32_t count = 0;                    // +0x400
  std::array<uint8_t, 32> majority{};    // +0x408
  bool majority_valid = true;            // +0x428 (1 on creation, 0 after every count change)
};

// Monotone counters that outlive a map (WorldMaps owns them; a standalone WorldMap owns its own):
// creation serials of points and keyframes (f2-contract-v3.3.md 6.1; NextId() is random, so an id
// can repeat, a serial cannot) and the F2 epochs. WorldMaps::ResetMap does not reset them.
struct MapSerials {
  uint64_t point = 0;
  uint64_t kf = 0;
  uint32_t tracker_epoch = 0;  // +1 when the tracker state changes or the MapBuilder is rebuilt
  uint32_t map_epoch = 0;      // +1 on every map reset
  int epoch_reason = 7;        // F2Invalidation of the pending tracker_epoch bump (7 = tracker_epoch, 8 = rebuild)
};

struct PointPool {
  std::vector<WorldPoint> points;
  std::vector<MajorityRecord> majority;  // same index as points (unused for CLOUD)
  std::vector<uint32_t> free_slots;      // LIFO
};

struct WorldMap::Impl {
  std::mt19937* id_generator = nullptr;   // +0x000 (owned by WorldMaps, or by own_generator)
  std::mt19937 own_generator{kIdGeneratorSeed};
  uint32_t map_id = 0;                    // +0x004
  uint32_t keyframe_seq = 0;              // +0x008
  std::vector<Keyframe> keyframes;        // +0x00c
  std::vector<uint32_t> free_keyframe_slots;  // +0x018
  std::array<PointPool, 5> pools;         // index = PointSource - 1
  int64_t last_keyframe_frame = 0;        // +0x1a0
  KeyframeId newest;                      // +0x1a8
  std::set<PointId> dirty;                // +0x1b0
  // ---- P2 candidate F2 (map/f2_confirm.h); untouched while F2 is off.
  MapSerials own_serials;
  MapSerials* serials = &own_serials;
  // Ground points whose F2 basis may have changed, keyed by (PointId, point_serial) so that a point
  // that reuses the random id of a retired one is told apart (id_reuse).
  std::set<std::pair<PointId, uint64_t>> f2_dirty;
  bool f2_all_dirty = false;              // every ground point must be re-evaluated
  uint32_t f2_last_tracker_epoch = 0;
  bool f2_in_merge = false;               // MergePoint reports one 'merge' event, not obs_added

  uint32_t NextId() { return (*id_generator)(); }
};

// Module-internal access (map/, map_tracker/). Free functions keep WorldMap's frozen public
// surface unchanged.
struct WorldMapAccess {
  using Impl = WorldMap::Impl;
  static Impl& impl(WorldMap& map) { return *map.impl_; }
  static const Impl& impl(const WorldMap& map) { return *map.impl_; }
  // Rebinds a freshly constructed map to a shared id generator and draws the map id from it
  // (653: map id = first draw after construction).
  // `serials` (optional) binds the monotone serials / epochs of the owning WorldMaps.
  static void BindGenerator(WorldMap& map, std::mt19937* generator, MapSerials* serials = nullptr);
};

// 1150 WorldMap::setPointPosition. Writes the position, recomputes the depth of every observation
// and unlinks observations whose ray residual exceeds
// WorldMap.maxInternalSquaredResidualForTriangulatedPts. Used by AdjustDirtyPoints and by the
// MapBuilder's iterative bundle adjustment (S3 §3.5). Requested as a public WorldMap method in
// contract-changes.md (WP2-CC-1).
void SetPointPosition(WorldMap& map, PointId id, const c8::HPoint3& position);

// Raw keyframe slot count including free slots (WorldMap +0x0c vector size). S3 §3 [A] compares
// this value, not the live keyframe count. Requested as a public method (WP2-CC-1).
size_t KeyframeSlotCount(const WorldMap& map);

// Side record of the point in its pool slot (MapTracker relocalization, 1550).
MajorityRecord& MutableMajorityRecord(WorldMap& map, PointSource source, uint32_t slot);

}  // namespace slam_refact::map

#endif  // SLAM_REFACT_MAP_WORLD_MAP_INTERNAL_H_

// WorldMap / WorldMaps unit tests (WP2-M1): add/merge/retire/remove invariants, iteration order,
// id generator, tenure boundaries, 1127/1150 formulas.

#include <cstdint>
#include <optional>
#include <random>
#include <set>
#include <vector>

#include <gtest/gtest.h>

#include "c8/hmatrix.h"
#include "config/settings.h"
#include "map/world_map.h"
#include "map/world_map_internal.h"
#include "map_test_util.h"
#include "reality/engine/features/descriptor-bit-counter.h"
#include "slam_refact/replay_diagnostics.h"

namespace slam_refact::map {
namespace {

using testing::Grid;
using testing::MakeFrame;

c8::HMatrix Translation(float x, float y, float z) {
  return c8::HMatrixGen::translation(x, y, z);
}

// mt19937(5489) draw sequence: a standalone WorldMap draws its map id first.
TEST(WorldMapIds, StandaloneMapDrawsMapIdThenKeyframeAndPointIds) {
  std::mt19937 g(kIdGeneratorSeed);
  const uint32_t map_id = g();
  const uint32_t kf_id = g();
  const uint32_t pt_id = g();
  (void)map_id;
  WorldMap map;
  const KeyframeId kf = map.AddKeyframe(MakeFrame(Grid(3)), 7);
  const PointId p = map.AddPoint(0, c8::HPoint3(0.f, 0.f, 1.f), PointSource::kGround);
  EXPECT_EQ(kf.id(), kf_id);
  EXPECT_EQ(kf.index(), 0u);
  EXPECT_EQ(p.id(), pt_id);
  EXPECT_EQ(p.index(), 0u);
}

// VioTracker ctor (DCMP:28418-28431): the first map's id, then a second draw (VioTracker +7240),
// then keyframe / point ids; ResetMap (653) draws only the new map's id.
TEST(WorldMapIds, WorldMapsSharesGeneratorAcrossReset) {
  std::mt19937 g(kIdGeneratorSeed);
  g();  // first map id
  g();  // VioTracker +7240
  const uint32_t first_kf = g();
  g();  // second map id (ResetMap)
  const uint32_t second_kf = g();
  WorldMaps maps;
  EXPECT_EQ(maps.active().AddKeyframe(MakeFrame(Grid(2)), 0).id(), first_kf);
  maps.ResetMap();
  EXPECT_EQ(maps.active().NumKeyframes(), 0);
  EXPECT_EQ(maps.active().AddKeyframe(MakeFrame(Grid(2)), 0).id(), second_kf);
}

// Exact-input replays pass the recorded run's seed (diagnostics::SetReplayIdGeneratorSeed); WorldMaps
// constructed afterwards draw from mt19937(seed); std::nullopt restores kIdGeneratorSeed.
TEST(WorldMapIds, ReplaySeedAppliesToNewWorldMaps) {
  constexpr uint32_t kSeed = 343035571u;  // the recovered seed of capture fix-s2q2
  std::mt19937 g(kSeed);
  g();
  g();
  const uint32_t kf_id = g();
  {
    // Restores the default even if an assertion aborts the scope (the seed is process-global).
    struct SeedGuard {
      explicit SeedGuard(uint32_t seed) { diagnostics::SetReplayIdGeneratorSeed(seed); }
      ~SeedGuard() { diagnostics::SetReplayIdGeneratorSeed(std::nullopt); }
    } guard(kSeed);
    WorldMaps maps;
    EXPECT_EQ(maps.active().AddKeyframe(MakeFrame(Grid(2)), 0).id(), kf_id);
  }
  std::mt19937 d(kIdGeneratorSeed);
  d();
  d();
  const uint32_t default_kf = d();
  WorldMaps maps;
  EXPECT_EQ(maps.active().AddKeyframe(MakeFrame(Grid(2)), 0).id(), default_kf);
}

TEST(WorldMap, AddKeyframeCountersAndNewest) {
  WorldMap map;
  const KeyframeId a = map.AddKeyframe(MakeFrame(Grid(4)), 10);
  const KeyframeId b = map.AddKeyframe(MakeFrame(Grid(4)), 15);
  EXPECT_EQ(map.keyframe_seq_counter(), 2u);
  EXPECT_EQ(map.GetKeyframe(a).seq, 0u);
  EXPECT_EQ(map.GetKeyframe(b).seq, 1u);
  EXPECT_EQ(map.GetKeyframe(b).creation_frame, 15);
  EXPECT_EQ(map.last_keyframe_frame(), 15);
  EXPECT_EQ(map.newest(), b);
  EXPECT_EQ(map.NumKeyframes(), 2);
  EXPECT_EQ(map.GetKeyframe(a).frame.points().size(), 4u);
}

TEST(WorldMap, AddObservationLinksBothSides) {
  WorldMap map;
  const KeyframeId kf = map.AddKeyframe(MakeFrame(Grid(4, /*scale=*/2)), 0);
  map.SetKeyframePose(kf, Translation(0.f, 0.f, 0.f));
  const PointId p = map.AddPoint(map.GetKeyframe(kf).seq, c8::HPoint3(0.1f, 0.2f, 3.f),
                                 PointSource::kTriangulated);
  map.AddObservation(p, 1, kf);
  const WorldPoint& w = map.Point(p);
  ASSERT_EQ(w.observations.size(), 1u);
  EXPECT_EQ(w.observations.at(kf), 1u);
  EXPECT_EQ(w.scale_by_kf.at(kf), 2);
  EXPECT_EQ(w.min_scale, 2);
  const c8::FrameWithPoints& f = map.GetKeyframe(kf).frame;
  EXPECT_EQ(f.points()[1].id(), p);
  EXPECT_EQ(f.worldPointIndices(), std::set<size_t>({1}));
  // depth_ = camera-frame z of the point (658 tail).
  EXPECT_EQ(f.points()[1].depth(), 3.f);
  // Majority record counts the GORB bits once.
  const MajorityRecord& rec = MutableMajorityRecord(map, PointSource::kTriangulated, 0);
  EXPECT_EQ(rec.count, 1u);
  EXPECT_FALSE(rec.majority_valid);
}

TEST(WorldMap, AddObservationRelinkWithinKeyframeUnlinksPreviousIndex) {
  WorldMap map;
  const KeyframeId kf = map.AddKeyframe(MakeFrame(Grid(4)), 0);
  const PointId p = map.AddPoint(0, c8::HPoint3(0.f, 0.f, 2.f), PointSource::kGround);
  map.AddObservation(p, 0, kf);
  map.AddObservation(p, 2, kf);
  const c8::FrameWithPoints& f = map.GetKeyframe(kf).frame;
  EXPECT_EQ(f.points()[0].id(), PointId::nullId());
  EXPECT_EQ(f.points()[2].id(), p);
  EXPECT_EQ(map.Point(p).observations.at(kf), 2u);
  EXPECT_EQ(MutableMajorityRecord(map, PointSource::kGround, 0).count, 1u);
}

TEST(WorldMap, AddObservationReplacesOldPointAndRemovesViews) {
  WorldMap map;
  const KeyframeId k0 = map.AddKeyframe(MakeFrame(Grid(4)), 0);
  const KeyframeId k1 = map.AddKeyframe(MakeFrame(Grid(4)), 5);
  map.SetKeyframePose(k1, Translation(0.5f, 0.f, 0.f));
  const PointId a = map.AddPoint(0, c8::HPoint3(0.f, 0.f, 2.f), PointSource::kTriangulated);
  const PointId b = map.AddPoint(0, c8::HPoint3(0.f, 0.f, 2.f), PointSource::kTriangulated);
  map.AddObservation(a, 0, k0);
  map.AddObservation(a, 0, k1);
  // Covisibility through the point view maps.
  EXPECT_EQ(map.GetKeyframe(k0).frame.pointsViewedByKeyframe().count(k1), 1u);
  map.AddObservation(b, 0, k1);  // replaces a in k1
  EXPECT_EQ(map.Point(a).observations.size(), 1u);
  EXPECT_EQ(map.Point(a).observations.count(k1), 0u);
  EXPECT_EQ(map.Point(a).scale_by_kf.count(k1), 0u);
  EXPECT_EQ(map.GetKeyframe(k0).frame.pointsViewedByKeyframe().count(k1), 0u);
  EXPECT_EQ(map.GetKeyframe(k1).frame.points()[0].id(), b);
}

TEST(WorldMap, RetireUnlinksAndSlotIsReusedLifo) {
  WorldMap map;
  const KeyframeId kf = map.AddKeyframe(MakeFrame(Grid(4)), 0);
  const PointId a = map.AddPoint(0, c8::HPoint3(0.f, 0.f, 1.f), PointSource::kGround);
  const PointId b = map.AddPoint(0, c8::HPoint3(0.f, 0.f, 1.f), PointSource::kGround);
  map.AddObservation(a, 0, kf);
  map.AddObservation(b, 1, kf);
  map.Retire(a);
  map.Retire(b);
  EXPECT_EQ(map.Point(a).state, PointState::kDead);
  EXPECT_TRUE(map.Point(a).observations.empty());
  EXPECT_EQ(map.GetKeyframe(kf).frame.points()[0].id(), PointId::nullId());
  EXPECT_TRUE(map.GetKeyframe(kf).frame.worldPointIndices().empty());
  EXPECT_EQ(map.NumPointsBySource()[1], 0);
  // Free list is LIFO: slot of b (1) first, then slot of a (0).
  const PointId c = map.AddPoint(0, c8::HPoint3(1.f, 0.f, 1.f), PointSource::kGround);
  const PointId d = map.AddPoint(0, c8::HPoint3(1.f, 0.f, 1.f), PointSource::kGround);
  EXPECT_EQ(c.index(), 1u);
  EXPECT_EQ(d.index(), 0u);
  EXPECT_EQ(map.Point(c).state, PointState::kNew);
  EXPECT_EQ(MutableMajorityRecord(map, PointSource::kGround, 1).count, 0u);
  EXPECT_TRUE(MutableMajorityRecord(map, PointSource::kGround, 1).majority_valid);
  // The old id now fails lookup: the slot answers to the new id.
  EXPECT_DEATH(map.Point(b), "getPoint");
}

TEST(WorldMap, MergeMovesObservationsNotAlreadyPresent) {
  WorldMap map;
  const KeyframeId k0 = map.AddKeyframe(MakeFrame(Grid(4)), 0);
  const KeyframeId k1 = map.AddKeyframe(MakeFrame(Grid(4)), 5);
  const KeyframeId k2 = map.AddKeyframe(MakeFrame(Grid(4)), 10);
  const PointId src = map.AddPoint(0, c8::HPoint3(0.f, 0.f, 1.f), PointSource::kGround);
  const PointId dst = map.AddPoint(0, c8::HPoint3(0.f, 0.f, 1.f), PointSource::kTriangulated);
  map.AddObservation(src, 0, k0);
  map.AddObservation(src, 1, k1);
  map.AddObservation(dst, 2, k1);
  map.AddObservation(dst, 3, k2);
  map.MergePoint(src, dst);
  EXPECT_EQ(map.Point(src).state, PointState::kDead);
  const WorldPoint& d = map.Point(dst);
  ASSERT_EQ(d.observations.size(), 3u);
  EXPECT_EQ(d.observations.at(k0), 0u);
  EXPECT_EQ(d.observations.at(k1), 2u);  // dst already had k1: kept
  EXPECT_EQ(d.observations.at(k2), 3u);
  EXPECT_EQ(map.GetKeyframe(k1).frame.points()[1].id(), PointId::nullId());
}

TEST(WorldMap, AdjustDirtyGroundPointIsMeanOfInitialDepthBackprojections) {
  WorldMap map;
  const KeyframeId k0 = map.AddKeyframe(MakeFrame({{0.0f, -0.4f, 1, 3}}), 0);
  const KeyframeId k1 = map.AddKeyframe(MakeFrame({{-0.08f, -0.44f, 1, 3}}), 5);
  map.SetKeyframePose(k0, Translation(0.f, 0.f, 0.f));
  map.SetKeyframePose(k1, Translation(0.2f, 0.1f, 0.f));
  const PointId p = map.AddPoint(0, c8::HPoint3(0.f, -1.f, 2.f), PointSource::kGround);
  map.AddObservation(p, 0, k0);
  map.AddObservation(p, 0, k1);
  // Expected value with the 1127 arithmetic (double sums, float storage).
  double sx = 0.0, sy = 0.0, sz = 0.0;
  for (KeyframeId kf : {k0, k1}) {
    const c8::FrameWithPoints& f = map.GetKeyframe(kf).frame;
    const c8::FramePoint& fp = f.points()[0];
    const double d = fp.initialDepth();
    const c8::HPoint3 w = f.extrinsic() * c8::HPoint3(static_cast<float>(d * fp.x()),
                                                      static_cast<float>(d * fp.y()),
                                                      static_cast<float>(d));
    sx += w.x();
    sy += w.y();
    sz += w.z();
  }
  map.AdjustDirtyPoints();
  const c8::HPoint3& pos = map.Point(p).position;
  EXPECT_EQ(pos.x(), static_cast<float>(sx * 0.5));
  EXPECT_EQ(pos.y(), static_cast<float>(sy * 0.5));
  EXPECT_EQ(pos.z(), static_cast<float>(sz * 0.5));
  // Both rays hit the ground plane y = -1 (initial depth), so the mean lies on it.
  EXPECT_NEAR(pos.y(), -1.0f, 1e-5f);
}

TEST(WorldMap, SetPointPositionUnlinksHighResidualObservationsAndRetiresEmpty) {
  WorldMap map;
  const KeyframeId k0 = map.AddKeyframe(MakeFrame({{0.0f, 0.0f, 1, 3}}), 0);
  const KeyframeId k1 = map.AddKeyframe(MakeFrame({{0.3f, 0.0f, 1, 3}}), 5);
  map.SetKeyframePose(k0, Translation(0.f, 0.f, 0.f));
  map.SetKeyframePose(k1, Translation(0.f, 0.f, 0.f));
  const PointId p = map.AddPoint(0, c8::HPoint3(0.f, 0.f, 2.f), PointSource::kTriangulated);
  map.AddObservation(p, 0, k0);
  map.AddObservation(p, 0, k1);
  // (0, 0, 2) projects to ray (0, 0): k0 residual 0, k1 residual 0.09 > 0.01 -> unlinked.
  SetPointPosition(map, p, c8::HPoint3(0.f, 0.f, 2.f));
  EXPECT_EQ(map.Point(p).observations.size(), 1u);
  EXPECT_EQ(map.Point(p).observations.count(k0), 1u);
  EXPECT_EQ(map.GetKeyframe(k1).frame.points()[0].id(), PointId::nullId());
  EXPECT_EQ(map.GetKeyframe(k0).frame.points()[0].depth(), 2.f);
}

TEST(WorldMap, AdjustDirtyRetiresPointWithoutObservations) {
  WorldMap map;
  const KeyframeId k0 = map.AddKeyframe(MakeFrame(Grid(2)), 0);
  const KeyframeId k1 = map.AddKeyframe(MakeFrame(Grid(2)), 5);
  const PointId p = map.AddPoint(0, c8::HPoint3(0.f, 0.f, 2.f), PointSource::kGround);
  const PointId q = map.AddPoint(0, c8::HPoint3(0.f, 0.f, 2.f), PointSource::kGround);
  map.AddObservation(p, 0, k0);
  map.AddObservation(p, 0, k1);
  map.AddObservation(q, 1, k0);
  map.AdjustDirtyPoints();
  // Unlinking dirties a point only while another keyframe still observes it (658): q's single
  // unlink leaves it clean, p's first unlink dirties it.
  map.AddObservation(PointId::nullId(), 1, k0);
  map.AddObservation(PointId::nullId(), 0, k0);
  map.AddObservation(PointId::nullId(), 0, k1);
  map.AdjustDirtyPoints();
  EXPECT_EQ(map.Point(p).state, PointState::kDead);
  EXPECT_EQ(map.Point(q).state, PointState::kNew);
  EXPECT_TRUE(map.Point(q).observations.empty());
}

TEST(WorldMap, RemoveKeyframeRetiresWeakPointsAndFreesSlot) {
  WorldMap map;
  const KeyframeId k0 = map.AddKeyframe(MakeFrame(Grid(3)), 0);
  const KeyframeId k1 = map.AddKeyframe(MakeFrame(Grid(3)), 5);
  const KeyframeId k2 = map.AddKeyframe(MakeFrame(Grid(3)), 10);
  const PointId shared3 = map.AddPoint(0, c8::HPoint3(0.f, 0.f, 2.f), PointSource::kGround);
  const PointId shared2 = map.AddPoint(0, c8::HPoint3(0.f, 0.f, 2.f), PointSource::kGround);
  map.AddObservation(shared3, 0, k0);
  map.AddObservation(shared3, 0, k1);
  map.AddObservation(shared3, 0, k2);
  map.AddObservation(shared2, 1, k0);
  map.AddObservation(shared2, 1, k1);
  map.RemoveKeyframe(k0);
  EXPECT_EQ(map.NumKeyframes(), 2);
  EXPECT_EQ(map.Point(shared3).state, PointState::kNew);  // 2 observations remain
  EXPECT_EQ(map.Point(shared2).state, PointState::kDead);  // 1 observation remained
  EXPECT_EQ(KeyframeSlotCount(map), 3u);
  EXPECT_DEATH(map.GetKeyframe(k0), "invalid keyframe");
  const KeyframeId k3 = map.AddKeyframe(MakeFrame(Grid(3)), 15);
  EXPECT_EQ(k3.index(), k0.index());  // freed slot reused
  EXPECT_EQ(map.GetKeyframe(k3).seq, 3u);
  EXPECT_EQ(KeyframeSlotCount(map), 3u);
}

TEST(WorldMap, KeyframeListsAreSortedByCreationFrame) {
  WorldMap map;
  const KeyframeId k0 = map.AddKeyframe(MakeFrame(Grid(12)), 30);
  const KeyframeId k1 = map.AddKeyframe(MakeFrame(Grid(12)), 10);
  const KeyframeId k2 = map.AddKeyframe(MakeFrame(Grid(12)), 20);
  EXPECT_EQ(map.AllKeyframesByCreation(), std::vector<KeyframeId>({k1, k2, k0}));
  // k0 shares 10 points with k1 and 9 with k2.
  for (uint32_t i = 0; i < 10; ++i) {
    const PointId p = map.AddPoint(0, c8::HPoint3(0.f, 0.f, 2.f), PointSource::kGround);
    map.AddObservation(p, i, k0);
    map.AddObservation(p, i, k1);
    if (i < 9) {
      map.AddObservation(p, i, k2);
    }
  }
  EXPECT_EQ(map.CovisibleKeyframes(k0), std::vector<KeyframeId>({k1}));
  EXPECT_EQ(map.CovisibleKeyframes(k0, 9), std::vector<KeyframeId>({k1, k2}));
}

TEST(WorldMap, AllPointIdsPoolOrderIncludesRetiredSlots) {
  WorldMap map;
  const PointId sky = map.AddPoint(0, c8::HPoint3(0.f, 0.f, 9.f), PointSource::kSky);
  const PointId tri = map.AddPoint(0, c8::HPoint3(0.f, 0.f, 1.f), PointSource::kTriangulated);
  const PointId depth = map.AddPoint(0, c8::HPoint3(0.f, 0.f, 1.f), PointSource::kDepth);
  const PointId ground = map.AddPoint(0, c8::HPoint3(0.f, 0.f, 1.f), PointSource::kGround);
  map.Retire(ground);
  EXPECT_EQ(map.AllPointIds(), std::vector<PointId>({tri, ground, sky, depth}));
  EXPECT_EQ(map.NumPointsBySource(), (std::array<int, 5>{1, 0, 1, 1, 0}));
}

TEST(WorldMap, TenureBoundaries) {
  const config::WorldMapsSettings& s = config::WorldMapsSettings::Get();
  ASSERT_EQ(s.tenure_keyframe_count, 5);
  ASSERT_EQ(s.required_tracking_count, 8);
  WorldMap map;
  for (int i = 0; i < 6; ++i) {
    map.AddKeyframe(MakeFrame(Grid(1)), i);
  }
  // counter = 6: age = 6 - birth - 1. birth 0 -> age 5 (eligible), birth 1 -> age 4.
  const PointId young = map.AddPoint(1, c8::HPoint3(0.f, 0.f, 1.f), PointSource::kGround);
  const PointId ok = map.AddPoint(0, c8::HPoint3(0.f, 0.f, 1.f), PointSource::kGround);
  const PointId few = map.AddPoint(0, c8::HPoint3(0.f, 0.f, 1.f), PointSource::kGround);
  const PointId low_ratio = map.AddPoint(0, c8::HPoint3(0.f, 0.f, 1.f), PointSource::kGround);
  const PointId sky = map.AddPoint(0, c8::HPoint3(0.f, 0.f, 1.f), PointSource::kSky);
  const PointId wrap = map.AddPoint(6, c8::HPoint3(0.f, 0.f, 1.f), PointSource::kGround);
  auto set_counts = [&map](PointId p, uint32_t hits, uint32_t count) {
    map.MutablePoint(p).tracking_hits = hits;
    map.MutablePoint(p).tracking_count = count;
  };
  set_counts(young, 0, 0);
  set_counts(ok, 2, 8);         // 0.25 * 8 = 2 <= 2: tenured
  set_counts(few, 7, 7);        // count 7 < 8: retired
  set_counts(low_ratio, 2, 9);  // 0.25 * 9 = 2.25 > 2: retired
  set_counts(sky, 8, 8);        // allowTenuredSkypoints = 1: tenured
  set_counts(wrap, 8, 8);       // age = 6 - 6 - 1 wraps to 2^32 - 1 (unsigned, 1562): tenured
  map.CullAndTenureAll();
  EXPECT_EQ(map.Point(young).state, PointState::kNew);
  EXPECT_EQ(map.Point(ok).state, PointState::kTenured);
  EXPECT_EQ(map.Point(few).state, PointState::kDead);
  EXPECT_EQ(map.Point(low_ratio).state, PointState::kDead);
  EXPECT_EQ(map.Point(sky).state, PointState::kTenured);
  EXPECT_EQ(map.Point(wrap).state, PointState::kTenured);
  // Tenured: retired when 0.1 * count > hits.
  set_counts(ok, 1, 10);  // 1.0 > 1 is false: stays
  map.CullAndTenureAll();
  EXPECT_EQ(map.Point(ok).state, PointState::kTenured);
  set_counts(ok, 1, 11);  // 1.1 > 1: retired
  map.CullAndTenureAll();
  EXPECT_EQ(map.Point(ok).state, PointState::kDead);
}

TEST(WorldMap, MajorityRecordMatchesBitCounts) {
  WorldMap map;
  const KeyframeId k0 = map.AddKeyframe(MakeFrame({{0.f, 0.f, 1, 0xf0}}), 0);
  const KeyframeId k1 = map.AddKeyframe(MakeFrame({{0.f, 0.f, 1, 0x3c}}), 1);
  const KeyframeId k2 = map.AddKeyframe(MakeFrame({{0.f, 0.f, 1, 0x0f}}), 2);
  const PointId p = map.AddPoint(0, c8::HPoint3(0.f, 0.f, 2.f), PointSource::kTriangulated);
  map.AddObservation(p, 0, k0);
  map.AddObservation(p, 0, k1);
  map.AddObservation(p, 0, k2);
  MajorityRecord& rec = MutableMajorityRecord(map, PointSource::kTriangulated, 0);
  ASSERT_EQ(rec.count, 3u);
  // Bits (MSB first per byte): 0xf0 = 11110000, 0x3c = 00111100, 0x0f = 00001111.
  const std::array<size_t, 8> expect = {1, 1, 2, 2, 2, 2, 1, 1};
  for (size_t b = 0; b < 8; ++b) {
    EXPECT_EQ(rec.one_counts[b], expect[b]) << b;
  }
  map.AddObservation(PointId::nullId(), 0, k1);
  EXPECT_EQ(rec.count, 2u);
  EXPECT_EQ(rec.one_counts[2], 1u);
}

TEST(WorldMapDeath, InvalidIdsAbort) {
  WorldMap map;
  const KeyframeId kf = map.AddKeyframe(MakeFrame(Grid(2)), 0);
  EXPECT_DEATH(map.Point(PointId(0, 1234)), "getPoint");
  EXPECT_DEATH(map.MutablePoint(PointId(3, 1)), "Invalid point requested");
  EXPECT_DEATH(map.AddObservation(PointId(0, 99), 0, kf), "getPoint");
  EXPECT_DEATH(map.AddObservation(PointId::nullId(), 0, KeyframeId(0, kf.id() + 1)),
               "Associating invalid keyframe id");
  EXPECT_DEATH(map.AddPoint(0, c8::HPoint3(0.f, 0.f, 1.f), PointSource::kCloud), "cloud");
}

}  // namespace
}  // namespace slam_refact::map

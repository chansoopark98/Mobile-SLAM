// trackPoints (1272) unit tests: pose recovery on a noise-free scene, match weights (§5.3), subset
// scores against hand-computed values (§5.4), dictionary dedup, matchMinimum, and the 1273 bimap.

#include <algorithm>
#include <cfloat>
#include <cstdint>
#include <cstdio>
#include <set>
#include <vector>

#include <gtest/gtest.h>

#include "c8/geometry/egomotion.h"
#include "c8/hmatrix.h"
#include "c8/hpoint.h"
#include "common/c8_adapters.h"
#include "config/params.h"
#include "map/world_map.h"
#include "map_tracker/map_tracker.h"
#include "map_tracker/map_tracker_internal.h"
#include "scene_util.h"

namespace slam_refact::map_tracker {
namespace {

using testing::BuildScene;
using testing::GridPoints;
using testing::MakeInput;
using testing::RandomDescriptors;
using testing::Scene;

TEST(TrackPoints, RecoversPoseOnNoiseFreeScene) {
  const std::vector<c8::HPoint3> world = GridPoints(10, 8);
  const auto desc = RandomDescriptors(world.size(), 11);
  Scene s;
  BuildScene(&s, world, desc, c8::HMatrixGen::i(), map::PointSource::kDepth);
  const c8::HMatrix truth = c8::HMatrixGen::translation(0.04f, -0.01f, 0.03f);
  const TrackerInput in = MakeInput(world, desc, truth, 0);
  std::vector<map::PointId> visible;
  std::vector<uint8_t> matched;
  MapTrackResult r;
  r.visible_out = &visible;
  r.matched_out = &matched;
  TrackPoints(s.map, in, c8::HMatrixGen::i(), {}, &r);
  // Every frame point matches its map point (unique random descriptors, distance 0).
  EXPECT_EQ(r.all_matches.size(), in.frame_points.points().size());
  for (const TrackedMatch& m : r.all_matches) {
    EXPECT_EQ(in.frame_points.store().get<c8::GorbFeature>(m.frame_index).data()[0],
              desc[m.id.index()].gorb[0]);
  }
  EXPECT_EQ(r.subset_obs.size(), 75u);
  EXPECT_LT(r.residual, 0.005f);
  // One 1146 call with the RotationTarget / ZeroTarget priors anchored at the input pose (S2 §3.1)
  // moves only part of the way from the input (identity, 4 cm off) toward the truth.
  const float e = testing::MaxTranslationError(r.pose, truth);
  std::printf("trackPoints single call: |dt| %.4g (input 0.04)\n", e);
  EXPECT_LT(e, testing::MaxTranslationError(c8::HMatrixGen::i(), truth));
  EXPECT_EQ(visible.size(), matched.size());
  EXPECT_FALSE(visible.empty());
}

// F1272:2339-2429 reports a projected point as visible only when 1549 accepted a keyframe view of it
// at the final pose (the view count at 1549 result+24). A point that was projected at the input pose
// but falls outside the frame at the final pose (status 2, no views) is not reported, so MapBuilder
// does not count it as tracked (936 tracking statistics). Found by memprobe: the engine counted
// these points (27 of 27 tracking_count mismatches on flip6-s3 fix-s3a before frame 133).
TEST(TrackPoints, OutOfFramePointsAtTheFinalPoseAreNotReportedVisible) {
  std::vector<c8::HPoint3> world = GridPoints(10, 8);
  // Edge points at ray x 0.55, depth 2.5 in the identity keyframe. The input pose is shifted so that
  // they sit 5e-4 inside the right frame bound; the truth is 6 cm further to -x, so the pose
  // trackPoints returns moves them outside the frame.
  constexpr float kDepth = 2.5f;
  for (int j = 0; j < 6; ++j) {
    world.emplace_back(0.55f * kDepth, (-0.2f + 0.08f * static_cast<float>(j)) * kDepth, kDepth);
  }
  c8::HPoint2 lower_left;
  c8::HPoint2 upper_right;
  c8::frameBounds(testing::SceneIntrinsics(), &lower_left, &upper_right);
  const float tx_in = -(upper_right.x() - 5e-4f - 0.55f) * kDepth;
  const auto desc = RandomDescriptors(world.size(), 21);
  Scene s;
  BuildScene(&s, world, desc, c8::HMatrixGen::i(), map::PointSource::kDepth);
  const c8::HMatrix input = c8::HMatrixGen::translation(tx_in, 0.0f, 0.0f);
  const c8::HMatrix truth = c8::HMatrixGen::translation(tx_in - 0.06f, 0.0f, 0.0f);
  const TrackerInput in = MakeInput(world, desc, truth, 0);
  std::vector<map::PointId> visible;
  std::vector<uint8_t> matched;
  MapTrackResult r;
  r.visible_out = &visible;
  r.matched_out = &matched;
  TrackPoints(s.map, in, input, {}, &r);
  ASSERT_LT(r.residual, FLT_MAX);
  std::printf("final pose tx %.5g (input %.5g, truth %.5g)\n", static_cast<double>(r.pose(0, 3)),
              static_cast<double>(tx_in), static_cast<double>(tx_in - 0.06f));
  std::vector<map::PointId> projected;
  for (const c8::FramePoint& fp : r.projected.frame.points()) {
    projected.push_back(fp.id());
  }
  const map::VisibilityResult vis = map::VerifyPointVisibility(
      s.map, ToPinholeIntrinsics(in.frame_points.intrinsic()), r.pose, projected);
  std::vector<map::PointId> with_views;
  size_t status2 = 0;
  for (size_t i = 0; i < projected.size(); ++i) {
    if (!vis.views[i].empty()) {
      with_views.push_back(projected[i]);
    }
    if (vis.status[i] == 2) {
      ++status2;
      EXPECT_TRUE(vis.views[i].empty());
      EXPECT_EQ(std::count(visible.begin(), visible.end(), projected[i]), 0);
    }
  }
  ASSERT_GT(status2, 0u) << "scene must put a projected point outside the frame at the final pose";
  EXPECT_EQ(visible, with_views);
  EXPECT_EQ(visible.size(), matched.size());
}

TEST(TrackPoints, TooFewMatchesKeepsInputPoseAndFltMax) {
  const std::vector<c8::HPoint3> world = GridPoints(10, 8);
  const auto desc = RandomDescriptors(world.size(), 12);
  Scene s;
  BuildScene(&s, world, desc, c8::HMatrixGen::i(), map::PointSource::kDepth);
  const auto other = RandomDescriptors(world.size(), 99);
  const TrackerInput in = MakeInput(world, other, c8::HMatrixGen::i(), 0);
  std::vector<map::PointId> visible = {map::PointId(3, 3)};
  std::vector<uint8_t> matched = {1};
  MapTrackResult r;
  r.visible_out = &visible;
  r.matched_out = &matched;
  const c8::HMatrix input = c8::HMatrixGen::translation(0.1f, 0.f, 0.f);
  TrackPoints(s.map, in, input, {}, &r);
  EXPECT_EQ(r.residual, FLT_MAX);
  EXPECT_EQ(r.pose.data(), input.data());
  EXPECT_TRUE(r.all_matches.empty());
  // The outputs were cleared at the start and visibility is skipped on this early return.
  EXPECT_TRUE(visible.empty());
  EXPECT_TRUE(matched.empty());
}

// S2 review LOW-2: matchMinimum = 0 would average over an empty subset (0 / 0).
TEST(TrackPointsDeathTest, MatchMinimumZeroAborts) {
  const std::vector<c8::HPoint3> world = GridPoints(10, 8);
  const auto desc = RandomDescriptors(world.size(), 13);
  Scene s;
  BuildScene(&s, world, desc, c8::HMatrixGen::i(), map::PointSource::kDepth);
  const TrackerInput in = MakeInput(world, desc, c8::HMatrixGen::i(), 0);
  MapTrackResult r;
  EXPECT_DEATH(
      {
        config::ApplyParamOverrides({{"WorldMaps.matchMinimum", ParamValue(0)}});
        TrackPoints(s.map, in, c8::HMatrixGen::i(), {}, &r);
      },
      "matchMinimum must be >= 1");
}

TEST(TrackPoints, DuplicateProjectionsNeverYieldDuplicateIds) {
  // Each map point projected twice: the two words compete for the same frame keypoint at equal
  // descriptor distance. The vendored LocalMatcher resolves that competition itself, so the
  // dictionary dedup (F1272:699-994) is not reached here; the result must still hold every id at
  // most once.
  const std::vector<c8::HPoint3> world = GridPoints(10, 8);
  const auto desc = RandomDescriptors(world.size(), 13);
  Scene s;
  BuildScene(&s, world, desc, c8::HMatrixGen::i(), map::PointSource::kDepth);
  const TrackerInput in = MakeInput(world, desc, c8::HMatrixGen::i(), 0);
  std::vector<map::PointId> ids;
  for (const map::PointId& id : s.ids) {
    ids.push_back(id);
    ids.push_back(id);
  }
  MapTrackResult r;
  TrackPoints(s.map, in, c8::HMatrixGen::i(), ids, &r);
  EXPECT_EQ(r.projected.frame.points().size(), 2 * world.size());
  std::set<map::PointId> unique(r.matched_ids.begin(), r.matched_ids.end());
  EXPECT_EQ(unique.size(), r.matched_ids.size());
}

TEST(TrackPoints, MatchWeightsBySource) {
  // 20 points on the keyframe's prior ground plane (camera y = -1, rays y in [-0.42, -0.12]):
  // their ground-plane initial depth (certainty 4e-10) equals the true depth, so all are visible.
  // Sources cycle TRI (ground-prior certainty only), TRI (known depth), SKY, DEPTH, GROUND.
  std::vector<c8::HPoint3> world;
  for (int j = 0; j < 4; ++j) {
    for (int i = 0; i < 5; ++i) {
      const float rx = -0.4f + 0.2f * static_cast<float>(i);
      const float ry = -0.42f + 0.1f * static_cast<float>(j);
      const float d = -1.0f / ry;
      world.emplace_back(rx * d, ry * d, d);
    }
  }
  const auto desc = RandomDescriptors(world.size(), 14);
  Scene s;
  BuildScene(&s, world, desc, c8::HMatrixGen::i(), map::PointSource::kGround);
  ASSERT_EQ(s.map.GetKeyframe(s.kf).frame.points().size(), world.size());
  // Re-create the points with the wanted sources on the same keyframe observations.
  map::WorldMap& m = s.map;
  std::vector<int> kind(world.size());
  for (size_t i = 0; i < world.size(); ++i) {
    kind[i] = static_cast<int>(i % 5);
    const map::PointSource src = kind[i] <= 1   ? map::PointSource::kTriangulated
                                 : kind[i] == 2 ? map::PointSource::kSky
                                 : kind[i] == 3 ? map::PointSource::kDepth
                                                : map::PointSource::kGround;
    const map::PointId pid = m.AddPoint(m.keyframe_seq_counter(), world[i], src);
    m.AddObservation(pid, static_cast<uint32_t>(i), s.kf);  // replaces the ground point
    s.ids[i] = pid;
    if (kind[i] == 1) {
      m.MutableKeyframe(s.kf).frame.setKnownDepth(i, world[i].z());
    }
  }
  const TrackerInput in = MakeInput(world, desc, c8::HMatrixGen::i(), 0);
  MapTrackResult r;
  TrackPoints(m, in, c8::HMatrixGen::i(), s.ids, &r);
  ASSERT_EQ(r.all_obs.size(), world.size());
  for (size_t k = 0; k < r.all_obs.size(); ++k) {
    const size_t i = static_cast<size_t>(
        std::find(s.ids.begin(), s.ids.end(), r.all_matches[k].id) - s.ids.begin());
    ASSERT_LT(i, world.size());
    EXPECT_EQ(r.all_obs[k].descriptorDist, 0.0f);
    const float base = (256.0f - 0.0f) * (1.0f / 256.0f);  // 1.0
    switch (kind[i]) {
      case 0:  // certainty 4e-10: log10 = -9.4 < -7 -> t < 0 -> 0 -> weight * (0 + 0 * 1)
        EXPECT_EQ(r.all_obs[k].weight, 0.0f) << i;
        break;
      case 1:  // known depth certainty (> 1e-4): t clamps to 1 -> weight * (0 + 1 * 1)
        EXPECT_EQ(r.all_obs[k].weight, base) << i;
        break;
      case 2:
        EXPECT_EQ(r.all_obs[k].weight, base * 0.01f) << i;
        break;
      default:
        EXPECT_EQ(r.all_obs[k].weight, base) << i;
        break;
    }
  }
}

// §5.4 by hand: 80 matches -> 75 kept.
//   5 DEPTH          0 + 2.0 + 0.15                      = 2.15
//   60 GROUND        first 52 in match order: 0.15 + 0.55 = 0.70, last 8: 0.15
//   5 TRI            0.1 + 0.15 = 0.25; TRI #0 tenured +0.2; TRI #1 hits 4 / count 8 -> +0.275
//   10 SKY           0, minus 0.75 * d_k^2 with observation offset d_k = 0.01 * (k + 1)
// Descending: 5 DEPTH, 52 GROUND, 5 TRI, 8 GROUND, then the 5 SKY with the smallest offsets
// (k = 0..4); the SKY k = 5..9 are dropped.
TEST(SelectSubset, HandComputedScores) {
  map::WorldMap m;
  MapTrackResult r;
  std::vector<map::PointSource> src;
  for (int i = 0; i < 5; ++i) src.push_back(map::PointSource::kDepth);
  for (int i = 0; i < 60; ++i) src.push_back(map::PointSource::kGround);
  for (int i = 0; i < 5; ++i) src.push_back(map::PointSource::kTriangulated);
  for (int i = 0; i < 10; ++i) src.push_back(map::PointSource::kSky);
  c8::FrameWithPoints proj(testing::SceneIntrinsics());
  std::vector<map::PointId> ids;
  for (size_t i = 0; i < src.size(); ++i) {
    const float x = -0.4f + 0.01f * static_cast<float>(i);
    const map::PointId pid = m.AddPoint(0, c8::HPoint3(x, 0.f, 1.f), src[i]);
    ids.push_back(pid);
    proj.addImageRayPoint(c8::HPoint2(x, 0.0f), 1, 0.f, 0.f, c8::FeatureSet{});
    c8::ObservedPoint o;
    float dy = 0.0f;
    if (i >= 70) {
      dy = 0.01f * static_cast<float>(i - 70 + 1);
    }
    o.position = c8::HPoint2(x, dy);
    o.scale = 1;
    o.descriptorDist = 0.f;
    o.weight = 1.f;
    r.all_obs.push_back(o);
    r.all_matches.push_back({static_cast<uint32_t>(i), pid});
    r.word_indices.push_back(static_cast<uint32_t>(i));
    r.all_world_points.push_back(c8::HPoint3(x, 0.f, 1.f));
  }
  m.MutablePoint(ids[65]).state = map::PointState::kTenured;
  m.MutablePoint(ids[66]).tracking_hits = 4;
  m.MutablePoint(ids[66]).tracking_count = 8;
  r.projected.frame = std::move(proj);

  SelectSubset(m, &r);
  ASSERT_EQ(r.subset_matches.size(), 75u);
  std::set<uint32_t> chosen;
  for (const TrackedMatch& sm : r.subset_matches) {
    chosen.insert(sm.frame_index);
  }
  std::set<uint32_t> expected;
  for (uint32_t i = 0; i < 75; ++i) {
    expected.insert(i);  // DEPTH 0-4, GROUND 5-64, TRI 65-69, SKY 70-74
  }
  EXPECT_EQ(chosen, expected);
  // Order: the 5 DEPTH points first (score 2.15), the 5 kept SKY points last by offset.
  for (size_t k = 0; k < 5; ++k) {
    EXPECT_LT(r.subset_matches[k].frame_index, 5u);
  }
  // After the 52 encouraged GROUND points (0.70): TRI #1 (0.525), then the tenured TRI #0 (0.45).
  EXPECT_EQ(r.subset_matches[5 + 52].frame_index, 66u);
  EXPECT_EQ(r.subset_matches[5 + 52 + 1].frame_index, 65u);
  for (size_t k = 70; k < 75; ++k) {
    EXPECT_EQ(r.subset_matches[k].frame_index, static_cast<uint32_t>(k));
  }
}

TEST(TrackedBimap, InsertFollows1273) {
  TrackedBimap b;
  const map::PointId a(1, 10), c(2, 20);
  b.Insert(5, a);
  b.Insert(6, c);
  // Same id at a new frame index: the old pair is removed first.
  b.Insert(7, a);
  EXPECT_EQ(b.by_frame.count(5), 0u);
  EXPECT_EQ(b.by_frame.at(7), a);
  EXPECT_EQ(b.by_id.at(a), 7u);
  // A frame index that is already used keeps its first id; the new id is still recorded by id.
  const map::PointId d(3, 30);
  b.Insert(6, d);
  EXPECT_EQ(b.by_frame.at(6), c);
  EXPECT_EQ(b.by_id.at(d), 6u);
  EXPECT_EQ(b.by_id.at(c), 6u);
}

}  // namespace
}  // namespace slam_refact::map_tracker

// P2 candidate F2: the finalize hook at both exits of MapBuilder::Update (f2-contract-v3.3.md 6.2, 7).
// Synthetic Update sequence as in update_test.cc: reset at frame 0, then tracking with a slowly moving
// camera; the keyframe at frame 5 is the first one after the reset keyframe.

#include <cstdint>
#include <cstring>
#include <memory>
#include <vector>

#include "common/tracking_types.h"
#include "gtest/gtest.h"
#include "map/f2_confirm.h"
#include "map/world_map.h"
#include "map_builder/map_builder.h"
#include "map_tracker/map_tracker_internal.h"
#include "p2_test_util.h"

namespace slam_refact::p2 {
namespace {

using map_builder::MapBuilder;
using map_builder::testing::AddRays;
using map_builder::testing::Pose;
using map_builder::testing::RayFeature;
using map_builder::testing::TestIntrinsics;
using map::F2Reason;
using map::F2Status;

constexpr int kGround = 24;
constexpr int kSky = 40;

std::unique_ptr<TrackerInput> MakeInput(int frame) {
  auto in = std::make_unique<TrackerInput>(TestIntrinsics(), c8::Quaternion(1.f, 0.f, 0.f, 0.f),
                                           static_cast<int64_t>(frame) * 33333333);
  std::vector<RayFeature> rays;
  for (int i = 0; i < kGround; ++i) {
    rays.push_back({-0.4f + 0.035f * static_cast<float>(i), -0.1f - 0.01f * static_cast<float>(i % 7),
                    1, static_cast<uint32_t>(frame * 1000 + i)});
  }
  for (int i = 0; i < kSky; ++i) {
    rays.push_back({-0.4f + 0.02f * static_cast<float>(i), 0.2f + 0.003f * static_cast<float>(i), 1,
                    static_cast<uint32_t>(frame * 1000 + 500 + i)});
  }
  AddRays(&in->frame_points, rays);
  return in;
}

MapTrackerOutput MakeTrk(int32_t state, int64_t consecutive, const c8::HMatrix& pose) {
  MapTrackerOutput trk;
  trk.pose = pose;
  trk.state = state;
  trk.frame_tracker_ok = true;
  trk.consecutive_frame_tracker_ok = consecutive;
  for (uint32_t i = 0; i < kGround; ++i) {
    trk.ground_idx.push_back(i);
  }
  for (uint32_t i = 0; i < kSky; ++i) {
    trk.sky_idx.push_back(kGround + i);
  }
  return trk;
}

c8::HMatrix PoseOf(int f) {
  return Pose(0.002f * static_cast<float>(f), 0.01f * static_cast<float>(f), 0.f, 0.f);
}

MapBuilderOutput Step(map::WorldMaps* maps, MapBuilder* mb, int f, int32_t state) {
  MapBuilderOutput out;
  const std::unique_ptr<TrackerInput> frm = MakeInput(f);
  mb->Update(maps, *frm, MakeTrk(state, 7 + f, PoseOf(f)), f * 33, &out);
  return out;
}

size_t ObsCount(const map::WorldMap& wm, map::PointId p) { return wm.Point(p).observations.size(); }

// Two more keyframes (4 units to either side) that see the world position of `g`, so g has three
// keyframes and exact geometry (parallax well above 3 degrees).
void AddTwoViews(map::WorldMap* wm, map::PointId g) {
  const c8::HPoint3 x = wm->Point(g).position;
  const c8::HMatrix poses[2] = {PoseAt(4.0, 0.0, 0.0), PoseAt(-4.0, 0.0, 0.0)};
  for (int k = 0; k < 2; ++k) {
    const c8::HPoint2 r = map_builder::testing::Project(poses[k], x);
    const map::KeyframeId kf = wm->AddKeyframe(
        map_builder::testing::MakeFrame({{r.x(), r.y(), 1, static_cast<uint32_t>(9000 + k)}}), 0);
    wm->SetKeyframePose(kf, poses[k]);
    wm->MutableKeyframe(kf).prune_protect = 1;  // keep the hand-made views through PruneKeyframes
    wm->AddObservation(g, 0, kf);
  }
}

// The scenario of the hook tests: after `last_nonkf` plain Updates, g (a ground seed of the reset
// keyframe) gets two more consistent views and is confirmed by the next Update. Then the initial
// depth of the creator's keypoint is made inconsistent and g is marked dirty: the
// AdjustDirtyPoints of the *same* Update then unlinks observations of g (an event that happens
// inside Update), and the state must be re-evaluated at that Update's end. `through_kf` selects
// whether that final Update creates a keyframe (frame 5) or not (frame 3).
void RunHookScenario(bool through_kf) {
  ScopedAccuracy acc(false, true);
  map::WorldMaps maps;
  MapBuilder mb;
  const MapBuilderOutput f0 = Step(&maps, &mb, 0, 0);
  ASSERT_TRUE(f0.map_reset);
  ASSERT_TRUE(f0.keyframe_created);
  map::PointId g;
  for (const map::PointId id : f0.new_points) {
    if (maps.active().Point(id).source == map::PointSource::kGround) {
      g = id;
      break;
    }
  }
  ASSERT_TRUE(g != map::PointId::nullId());
  EXPECT_FALSE(maps.active().Point(g).f2.evaluated) << "frame 0 is a held frame (tracker state 0)";
  EXPECT_EQ(Counters().f2_hold_frames, 1u);
  EXPECT_EQ(Counters().f2_path_kf, 1u);

  Step(&maps, &mb, 1, 1);  // non-keyframe: the seeds of the reset keyframe are evaluated here
  EXPECT_TRUE(maps.active().Point(g).f2.evaluated);
  EXPECT_EQ(static_cast<int>(maps.active().Point(g).f2.last_reason),
            static_cast<int>(F2Reason::kTooFewKfs));
  EXPECT_EQ(maps.active().Point(g).f2.eval_frame, 1);
  EXPECT_EQ(Counters().f2_path_nonkf, 1u);

  AddTwoViews(&maps.active(), g);
  const int last_before = through_kf ? 4 : 2;
  for (int f = 2; f <= last_before; ++f) {
    const MapBuilderOutput o = Step(&maps, &mb, f, 1);
    EXPECT_FALSE(o.keyframe_created) << "frame " << f;
  }
  ASSERT_EQ(static_cast<int>(maps.active().Point(g).f2.state), static_cast<int>(F2Status::kConfirmed))
      << "reason " << static_cast<int>(maps.active().Point(g).f2.last_reason);
  ASSERT_EQ(ObsCount(maps.active(), g), 3u);
  {
    // Birth snapshot (the position MapBuilder [D] gave the seed) against the plane depth F2 uses:
    // same creator pose revision => |z(birth_pos) - d_plane| <= 1e-5 (contract 5.2, d_plane).
    const map::WorldPoint& q = maps.active().Point(g);
    const map::Keyframe& creator = maps.active().GetKeyframe(q.f2.creator);
    ASSERT_TRUE(q.f2.birth_valid);
    EXPECT_EQ(creator.pose_rev, q.f2.birth_creator_rev);
    const c8::HPoint3 cam =
        creator.frame.extrinsic().inv() * c8::HPoint3(static_cast<float>(q.f2.birth_pos[0]),
                                                      static_cast<float>(q.f2.birth_pos[1]),
                                                      static_cast<float>(q.f2.birth_pos[2]));
    EXPECT_NEAR(static_cast<double>(cam.z()), q.f2.d_plane, 1e-5);
  }

  // Make the creator's initial depth 10x too deep and re-adjust g inside the next Update: the mean
  // position moves along the creator's ray, so the other two views exceed the residual limit and
  // are unlinked by SetPointPosition.
  c8::FramePoint& fp = (*maps.active().MutableKeyframe(
      maps.active().Point(g).f2.creator).frame.mutablePoints())[maps.active().Point(g).observations.at(
      maps.active().Point(g).f2.creator)];
  fp.updateInitialDepthDuringDeserialization(fp.initialDepth() * 10.0);
  maps.active().MarkPointDirty(g);

  const int f = last_before + 1;
  const uint64_t kf_before = Counters().f2_path_kf;
  const uint64_t nonkf_before = Counters().f2_path_nonkf;
  const MapBuilderOutput o = Step(&maps, &mb, f, 1);
  EXPECT_EQ(o.keyframe_created, through_kf) << "frame " << f;
  EXPECT_EQ(Counters().f2_path_kf - kf_before, through_kf ? 1u : 0u);
  EXPECT_EQ(Counters().f2_path_nonkf - nonkf_before, through_kf ? 0u : 1u);

  // The observation set changed inside this Update ...
  const map::WorldPoint* now = maps.active().FindPoint(g);
  ASSERT_NE(now, nullptr);
  ASSERT_NE(static_cast<int>(now->state), static_cast<int>(map::PointState::kDead));
  EXPECT_LT(ObsCount(maps.active(), g), 3u) << "scenario must unlink observations inside Update";
  // ... and the state was re-evaluated at its end, on the new basis, and published for frame f + 1.
  EXPECT_EQ(now->f2.eval_frame, f);
  EXPECT_EQ(now->f2.applied_frame, f + 1);
  EXPECT_NE(static_cast<int>(now->f2.state), static_cast<int>(F2Status::kConfirmed));
  EXPECT_EQ(static_cast<int>(now->f2.last_reason), static_cast<int>(F2Reason::kTooFewKfs));
  map::F2EvalResult fresh;
  ASSERT_TRUE(map::F2EvaluatePointNow(maps.active(), g, &fresh));
  EXPECT_EQ(static_cast<int>(fresh.status), static_cast<int>(now->f2.state));
  EXPECT_EQ(static_cast<int>(fresh.reason), static_cast<int>(now->f2.last_reason));
  EXPECT_EQ(Counters().f2_dirty_remainder, 0u);
}

TEST(P2F2Hook, Hook_nonkf_path_reevaluates_same_update) { RunHookScenario(false); }

TEST(P2F2Hook, Hook_kf_path_reevaluates_same_update) { RunHookScenario(true); }

// ---------------------------------------------------------------------------------------------
// End to end: the real Update (keyframe or not) evaluates, and the next tracking frame's actual
// subset follows. finalize is never called directly here.

map_tracker::MapTrackResult MakeResult(const std::vector<map::PointId>& ids) {
  map_tracker::MapTrackResult r;
  r.projected.frame = c8::FrameWithPoints(TestIntrinsics());
  std::vector<RayFeature> rays;
  for (size_t i = 0; i < ids.size(); ++i) {
    rays.push_back({-0.3f + 0.6f * static_cast<float>(i % 20) / 19.0f,
                    -0.3f + 0.6f * static_cast<float>(i / 20) / 6.0f, 1,
                    static_cast<uint32_t>(70000 + i)});
  }
  AddRays(&r.projected.frame, rays);
  for (size_t i = 0; i < ids.size(); ++i) {
    r.all_matches.push_back({static_cast<uint32_t>(i), ids[i]});
    r.all_obs.push_back({c8::HPoint2(rays[i].x, rays[i].y), 1, 0.0f, 1.0f});
    r.word_indices.push_back(static_cast<uint32_t>(i));
    r.all_world_points.push_back(c8::HPoint3(0.f, 0.f, 1.f));
  }
  return r;
}

bool InSubset(const map_tracker::MapTrackResult& r, map::PointId p) {
  for (const map_tracker::TrackedMatch& m : r.subset_matches) {
    if (m.id == p) {
      return true;
    }
  }
  return false;
}

void RunEndToEnd(bool through_kf) {
  ScopedAccuracy acc(false, true);
  map::WorldMaps maps;
  MapBuilder mb;
  const MapBuilderOutput f0 = Step(&maps, &mb, 0, 0);
  map::PointId g;
  for (const map::PointId id : f0.new_points) {
    if (maps.active().Point(id).source == map::PointSource::kGround) {
      g = id;
      break;
    }
  }
  ASSERT_TRUE(g != map::PointId::nullId());
  maps.active().MutableKeyframe(maps.active().Point(g).f2.creator).prune_protect = 1;
  // Competitors that survive the culling of Update: tenured ground points (0.35), tenured
  // triangulated points (0.45). g alone: 0.15 unconfirmed, 0.70 + 0.15 confirmed.
  std::vector<map::PointId> ids = {g};
  for (int i = 0; i < 25; ++i) {
    const map::PointId t = maps.active().AddPoint(maps.active().keyframe_seq_counter(),
                                                  c8::HPoint3(static_cast<float>(i), 0.f, 3.f),
                                                  map::PointSource::kGround);
    maps.active().MutablePoint(t).state = map::PointState::kTenured;
    ids.push_back(t);
  }
  for (int i = 0; i < 65; ++i) {
    const map::PointId t = maps.active().AddPoint(maps.active().keyframe_seq_counter(),
                                                  c8::HPoint3(static_cast<float>(i), 1.f, 3.f),
                                                  map::PointSource::kTriangulated);
    maps.active().MutablePoint(t).state = map::PointState::kTenured;
    ids.push_back(t);
  }
  const int target = through_kf ? 5 : 2;
  for (int f = 1; f < target; ++f) {
    EXPECT_FALSE(Step(&maps, &mb, f, 1).keyframe_created) << "frame " << f;
  }
  AddTwoViews(&maps.active(), g);  // g now has three consistent keyframes, nobody evaluated them yet

  // Tracking frame `target`: SelectSubset runs before the Update of that frame.
  map_tracker::MapTrackResult before = MakeResult(ids);
  map_tracker::SelectSubset(maps.active(), &before);
  ASSERT_EQ(before.subset_matches.size(), 75u);
  EXPECT_FALSE(InSubset(before, g)) << "no bonus before the Update evaluated g";

  const MapBuilderOutput out = Step(&maps, &mb, target, 1);
  EXPECT_EQ(out.keyframe_created, through_kf);
  const map::WorldPoint* now = maps.active().FindPoint(g);
  ASSERT_NE(now, nullptr);
  ASSERT_NE(static_cast<int>(now->state), static_cast<int>(map::PointState::kDead));
  EXPECT_EQ(static_cast<int>(now->f2.state), static_cast<int>(F2Status::kConfirmed))
      << "reason " << static_cast<int>(now->f2.last_reason);
  EXPECT_EQ(now->f2.eval_frame, target);
  EXPECT_EQ(map::F2AppliedFrame(), target + 1);

  // Tracking frame target + 1.
  const uint64_t fresh_before = Counters().sel_new_confirmed_subset;
  map_tracker::MapTrackResult next = MakeResult(ids);
  map_tracker::SelectSubset(maps.active(), &next);
  EXPECT_TRUE(InSubset(next, g)) << "confirmed in Update(" << target << "): bonus in frame " << target + 1;
  EXPECT_EQ(next.subset_matches.front().id == g, true) << "0.90 is the top score";
  EXPECT_EQ(Counters().sel_new_confirmed_subset - fresh_before, 1u);
}

TEST(P2F2Hook, Hook_end_to_end_nonkf_next_frame_subset) { RunEndToEnd(false); }

TEST(P2F2Hook, Hook_end_to_end_kf_next_frame_subset) { RunEndToEnd(true); }

// A new MapBuilder on a map that already holds ground points (engine rebuild) starts a new tracker
// epoch: the seeds go back to unconfirmed and are evaluated again in its first Update.
TEST(P2F2Hook, Hook_rebuild_bumps_tracker_epoch) {
  ScopedAccuracy acc(false, true);
  map::WorldMaps maps;
  const SceneResult sc = BuildHeightScene(&maps.active(), -1.0, 5);
  map::F2Finalize(maps.active(), Args(1));
  ASSERT_EQ(Counters().f2_eval[static_cast<size_t>(F2Reason::kConfirmed)], 10u);
  for (const map::PointId p : sc.points) {
    maps.active().MutablePoint(p).state = map::PointState::kTenured;  // survive the culling of Update
  }
  MapBuilder rebuilt;
  Step(&maps, &rebuilt, 0, 1);
  EXPECT_EQ(Counters().f2_invalidation[kInvRebuild], 10u);
  EXPECT_EQ(Counters().f2_invalidation[kInvTrackerEpoch], 0u);
  for (const map::PointId p : sc.points) {
    EXPECT_EQ(static_cast<int>(maps.active().Point(p).f2.state), static_cast<int>(F2Status::kConfirmed));
    EXPECT_EQ(maps.active().Point(p).f2.eval_frame, 0) << "evaluated again by the rebuilt MapBuilder";
  }
}

TEST(P2F2Hook, Hook_dirty_remainder_zero) {
  ScopedAccuracy acc(false, true);
  map::WorldMaps maps;
  MapBuilder mb;
  constexpr int kFrames = 80;
  int keyframes = 0;
  for (int f = 0; f < kFrames; ++f) {
    const bool lost = f == 0 || (f >= 20 && f < 23);  // held frames: reset frame and a short LOST spell
    const MapBuilderOutput o = Step(&maps, &mb, f, lost ? 0 : 1);
    keyframes += o.keyframe_created ? 1 : 0;
  }
  EXPECT_GT(keyframes, 5);
  // Every Update ends in exactly one finalize, on the exit it took.
  EXPECT_EQ(Counters().f2_path_kf, static_cast<uint64_t>(keyframes));
  EXPECT_EQ(Counters().f2_path_nonkf, static_cast<uint64_t>(kFrames - keyframes));
  EXPECT_GE(Counters().f2_hold_frames, 4u);
  EXPECT_EQ(Counters().f2_dirty_remainder, 0u);
  EXPECT_GT(Counters().f2_eval[static_cast<size_t>(F2Reason::kTooFewKfs)], 0u);
}

}  // namespace
}  // namespace slam_refact::p2

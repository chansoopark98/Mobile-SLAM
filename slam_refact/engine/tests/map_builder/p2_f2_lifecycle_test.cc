// P2 candidate F2: identifier reuse, state lifecycle (deletion, merge, creator removal, tracker epoch,
// map reset) and path independence of the finalize hook (f2-contract-v3.3.md sections 6, 7).

#include <cmath>
#include <cstdint>
#include <cstring>
#include <random>
#include <vector>

#include "gtest/gtest.h"
#include "map/f2_confirm.h"
#include "map/world_map.h"
#include "map/world_map_internal.h"
#include "p2_test_util.h"

namespace slam_refact::p2 {
namespace {

using map::F2Reason;
using map::F2Status;

const std::vector<c8::HMatrix> kPoses3 = {PoseAt(0.0, 0.0, 0.0), PoseAt(-1.2, 0.0, 0.0),
                                          PoseAt(1.0, 0.1, -0.3)};
const ScenePoint kPoint{V3(0.0, -1.0, 3.0), {0, 1, 2}};

size_t Eval(F2Reason r) { return Counters().f2_eval[static_cast<size_t>(r)]; }

// One single-keypoint keyframe that sees `x` from `pose`.
map::KeyframeId AddSingleRayKeyframe(map::WorldMap* wm, const c8::HMatrix& pose, const V3& x,
                                     uint32_t descriptor, int64_t frame) {
  const c8::HPoint2 r = map_builder::testing::Project(
      pose, c8::HPoint3(static_cast<float>(x.x()), static_cast<float>(x.y()),
                        static_cast<float>(x.z())));
  const map::KeyframeId kf =
      wm->AddKeyframe(map_builder::testing::MakeFrame({{r.x(), r.y(), 1, descriptor}}), frame);
  wm->SetKeyframePose(kf, pose);
  return kf;
}

// ---------------------------------------------------------------------------------------------
// Identifier reuse (random NextId may repeat; serials may not)

// Point id reuse: a retired point A and a new point B draw the same id from a restored generator.
void RunPointReuse(uint64_t* id_reuse_delta) {
  std::mt19937 rng(777);
  map::WorldMap wm;
  map::WorldMapAccess::BindGenerator(wm, &rng);
  const std::vector<ScenePoint> pts = {kPoint};
  const SceneKfs kfs = AddSceneKeyframes(&wm, kPoses3, pts);
  const std::mt19937 before_a = rng;  // state before A draws its id
  const map::PointId a = AddScenePoint(&wm, kfs, kPoses3, pts, 0, -1.0);
  const uint32_t a_id = wm.Point(a).id;
  const uint64_t a_serial = wm.Point(a).point_serial;
  map::F2Finalize(wm, Args(1));
  ASSERT_EQ(static_cast<int>(wm.Point(a).f2.state), static_cast<int>(F2Status::kConfirmed));

  wm.Retire(a);
  rng = before_a;  // the next draw repeats A's id
  const map::PointId b = AddScenePoint(&wm, kfs, kPoses3, pts, 0, -1.0);
  EXPECT_TRUE(a == b) << "same slot and id";
  EXPECT_EQ(wm.Point(b).id, a_id);
  EXPECT_NE(wm.Point(b).point_serial, a_serial);
  EXPECT_GT(wm.Point(b).point_serial, a_serial);
  // A's confirmed state is not inherited.
  EXPECT_EQ(static_cast<int>(wm.Point(b).f2.state), static_cast<int>(F2Status::kUnconfirmed));
  EXPECT_FALSE(wm.Point(b).f2.evaluated);
  const uint64_t before = Counters().f2_id_reuse;
  map::F2Finalize(wm, Args(2));
  *id_reuse_delta = Counters().f2_id_reuse - before;
  // B is judged on its own observations.
  EXPECT_TRUE(wm.Point(b).f2.evaluated);
  EXPECT_EQ(wm.Point(b).f2.eval_frame, 2);
  EXPECT_EQ(wm.Point(b).f2.point_serial_at_eval, wm.Point(b).point_serial);
}

// Keyframe id reuse: a removed keyframe K3 and a new keyframe K3' draw the same id and slot.
void RunKfReuse(uint64_t* id_reuse_delta) {
  std::mt19937 rng(4242);
  map::WorldMap wm;
  map::WorldMapAccess::BindGenerator(wm, &rng);
  const V3 x(0.0, -1.0, 3.0);
  const map::KeyframeId k0 = AddSingleRayKeyframe(&wm, PoseAt(0.0, 0.0, 0.0), x, 1, 0);
  const map::KeyframeId k1 = AddSingleRayKeyframe(&wm, PoseAt(-1.2, 0.0, 0.0), x, 2, 5);
  const std::mt19937 before_k3 = rng;
  const map::KeyframeId k3 = AddSingleRayKeyframe(&wm, PoseAt(1.0, 0.1, -0.3), x, 3, 10);
  const uint64_t k3_serial = wm.GetKeyframe(k3).kf_serial;
  const map::PointId p = wm.AddPoint(wm.GetKeyframe(k0).seq, c8::HPoint3(0.f, -1.f, 3.f),
                                     map::PointSource::kGround);
  wm.AddObservation(p, 0, k0);
  wm.AddObservation(p, 0, k1);
  wm.AddObservation(p, 0, k3);
  map::F2Finalize(wm, Args(1));
  ASSERT_EQ(static_cast<int>(wm.Point(p).f2.state), static_cast<int>(F2Status::kConfirmed));

  wm.RemoveKeyframe(k3);  // no finalize: the stored basis still names (K3 id, K3 serial)
  rng = before_k3;
  // K3' sees a different world point: P's observation there is inconsistent with K0 and K1.
  const map::KeyframeId k3b =
      AddSingleRayKeyframe(&wm, PoseAt(1.0, 0.1, -0.3), V3(2.0, -1.0, 3.0), 3, 12);
  EXPECT_TRUE(k3b == k3) << "same slot and id";
  EXPECT_NE(wm.GetKeyframe(k3b).kf_serial, k3_serial);
  EXPECT_GT(wm.GetKeyframe(k3b).kf_serial, k3_serial);
  wm.AddObservation(p, 0, k3b);
  const uint64_t before = Counters().f2_id_reuse;
  map::F2Finalize(wm, Args(2));
  *id_reuse_delta = Counters().f2_id_reuse - before;
  // The confirmed state of the old basis is gone; the new basis does not confirm.
  EXPECT_NE(static_cast<int>(wm.Point(p).f2.state), static_cast<int>(F2Status::kConfirmed));
  EXPECT_EQ(wm.Point(p).f2.eval_frame, 2);
  EXPECT_EQ(wm.Point(p).f2.basis.size(), 3u);
  bool names_new_serial = false;
  for (const map::F2Basis& b : wm.Point(p).f2.basis) {
    names_new_serial = names_new_serial || b.kf_serial == wm.GetKeyframe(k3b).kf_serial;
  }
  EXPECT_TRUE(names_new_serial);
}

TEST(P2F2Lifecycle, IdReuse_point_serial) {
  ScopedAccuracy acc(false, true);
  uint64_t delta = 99;
  RunPointReuse(&delta);
  EXPECT_EQ(delta, 1u);
}

TEST(P2F2Lifecycle, IdReuse_kf_serial) {
  ScopedAccuracy acc(false, true);
  uint64_t delta = 99;
  RunKfReuse(&delta);
  EXPECT_EQ(delta, 1u);
}

TEST(P2F2Lifecycle, IdReuse_serial_monotonic_across_reset) {
  ScopedAccuracy acc(false, true);
  map::WorldMaps maps;
  uint64_t last_point = 0;
  uint64_t last_kf = 0;
  uint32_t last_map_epoch = 0;
  for (int round = 0; round < 4; ++round) {
    const std::vector<ScenePoint> pts = {kPoint, kPoint};
    const SceneResult sc = BuildScene(&maps.active(), kPoses3, pts, -1.0);
    for (const map::PointId pid : sc.points) {
      EXPECT_GT(maps.active().Point(pid).point_serial, last_point) << "round " << round;
      last_point = maps.active().Point(pid).point_serial;
    }
    for (const map::KeyframeId kf : sc.kfs) {
      EXPECT_GT(maps.active().GetKeyframe(kf).kf_serial, last_kf) << "round " << round;
      last_kf = maps.active().GetKeyframe(kf).kf_serial;
    }
    maps.ResetMap();
    const uint32_t epoch = map::WorldMapAccess::impl(maps.active()).serials->map_epoch;
    EXPECT_GT(epoch, last_map_epoch);
    last_map_epoch = epoch;
  }
  EXPECT_EQ(last_point, 8u);  // 4 rounds x 2 points: never restarted at 0
  EXPECT_EQ(last_kf, 12u);    // 4 rounds x 3 keyframes
}

TEST(P2F2Lifecycle, IdReuse_counter_increments_once) {
  ScopedAccuracy acc(false, true);
  EXPECT_EQ(Counters().f2_id_reuse, 0u);
  uint64_t point_delta = 99;
  RunPointReuse(&point_delta);
  EXPECT_EQ(point_delta, 1u);
  EXPECT_EQ(Counters().f2_id_reuse, 1u);
  uint64_t kf_delta = 99;
  RunKfReuse(&kf_delta);
  EXPECT_EQ(kf_delta, 1u);
  EXPECT_EQ(Counters().f2_id_reuse, 2u) << "exactly one increment per reuse scenario";
}

// An observation moved to another keypoint of the same keyframe is one event: obs_replaced, not
// also obs_added (contract 6.2).
TEST(P2F2Lifecycle, Lifecycle_obs_move_counts_once) {
  ScopedAccuracy acc(false, true);
  map::WorldMap wm;
  const V3 x(0.0, -1.0, 3.0);
  std::vector<map::KeyframeId> kfs;
  for (const c8::HMatrix& pose : kPoses3) {
    const c8::HPoint2 r = map_builder::testing::Project(
        pose, c8::HPoint3(static_cast<float>(x.x()), static_cast<float>(x.y()), static_cast<float>(x.z())));
    kfs.push_back(wm.AddKeyframe(
        map_builder::testing::MakeFrame({{r.x(), r.y(), 1, 11}, {r.x(), r.y(), 1, 12}}), 0));
    wm.SetKeyframePose(kfs.back(), pose);
  }
  const map::PointId p = wm.AddPoint(wm.GetKeyframe(kfs[0]).seq, c8::HPoint3(0.f, -1.f, 3.f),
                                     map::PointSource::kGround);
  for (const map::KeyframeId kf : kfs) {
    wm.AddObservation(p, 0, kf);
  }
  map::F2Finalize(wm, Args(1));
  ASSERT_EQ(static_cast<int>(wm.Point(p).f2.state), static_cast<int>(F2Status::kConfirmed));
  const auto before = Counters().f2_invalidation;
  wm.AddObservation(p, 1, kfs[2]);  // same keyframe, other keypoint
  const auto after = Counters().f2_invalidation;
  EXPECT_EQ(after[kInvObsReplaced] - before[kInvObsReplaced], 1u);
  EXPECT_EQ(after[kInvObsAdded] - before[kInvObsAdded], 0u);
  EXPECT_EQ(after[kInvObsRemoved] - before[kInvObsRemoved], 0u);
  map::F2Finalize(wm, Args(2));
  EXPECT_EQ(wm.Point(p).f2.eval_frame, 2);
  for (const map::F2Basis& b : wm.Point(p).f2.basis) {
    if (b.kf_id == kfs[2].id()) {
      EXPECT_EQ(b.obs_idx, 1u);
    }
  }
}

// The creator's observation is removed first, so RemoveKeyframe(creator) does not see the point;
// the creator id is then drawn again. The creator-serial and the basis-serial mismatch are one
// reuse event of this point: id_reuse increases by exactly one.
TEST(P2F2Lifecycle, IdReuse_creator_obs_removed_first_counts_once) {
  ScopedAccuracy acc(false, true);
  std::mt19937 rng(31337);
  map::WorldMap wm;
  map::WorldMapAccess::BindGenerator(wm, &rng);
  const V3 x(0.0, -1.0, 3.0);
  auto add_kf = [&](const c8::HMatrix& pose, uint32_t d, int64_t frame) {
    const c8::HPoint2 r = map_builder::testing::Project(
        pose, c8::HPoint3(static_cast<float>(x.x()), static_cast<float>(x.y()), static_cast<float>(x.z())));
    const map::KeyframeId kf =
        wm.AddKeyframe(map_builder::testing::MakeFrame({{r.x(), r.y(), 1, d}}), frame);
    wm.SetKeyframePose(kf, pose);
    return kf;
  };
  const std::mt19937 before_k0 = rng;
  const map::KeyframeId k0 = add_kf(PoseAt(0.0, 0.0, 0.0), 1, 0);
  const map::KeyframeId k1 = add_kf(PoseAt(-1.2, 0.0, 0.0), 2, 5);
  const map::KeyframeId k2 = add_kf(PoseAt(1.0, 0.1, -0.3), 3, 10);
  const map::KeyframeId k3 = add_kf(PoseAt(0.4, 0.2, -0.6), 4, 15);
  const map::PointId p = wm.AddPoint(wm.GetKeyframe(k0).seq, c8::HPoint3(0.f, -1.f, 3.f),
                                     map::PointSource::kGround);
  for (const map::KeyframeId kf : {k0, k1, k2, k3}) {
    wm.AddObservation(p, 0, kf);
  }
  map::F2Finalize(wm, Args(1));
  ASSERT_EQ(static_cast<int>(wm.Point(p).f2.state), static_cast<int>(F2Status::kConfirmed));

  wm.AddObservation(map::PointId::nullId(), 0, k0);  // the creator's observation goes first
  wm.RemoveKeyframe(k0);                             // the point is no longer seen by the keyframe
  EXPECT_FALSE(wm.Point(p).f2.creator_lost) << "RemoveKeyframe skipped the point";
  rng = before_k0;
  const map::KeyframeId k0b = add_kf(PoseAt(0.0, 0.0, 0.0), 1, 20);
  ASSERT_TRUE(k0b == k0) << "same slot and id";
  wm.AddObservation(p, 0, k0b);
  const uint64_t before = Counters().f2_id_reuse;
  map::F2Finalize(wm, Args(2));
  EXPECT_EQ(Counters().f2_id_reuse - before, 1u) << "one reuse event, one count";
  EXPECT_EQ(static_cast<int>(wm.Point(p).f2.state), static_cast<int>(F2Status::kUnconfirmed));
  EXPECT_TRUE(wm.Point(p).f2.creator_lost);
}

// ---------------------------------------------------------------------------------------------
// Lifecycle

TEST(P2F2Lifecycle, Lifecycle_observation_deleted) {
  ScopedAccuracy acc(false, true);
  map::WorldMap wm;
  const std::vector<c8::HMatrix> poses = {PoseAt(0.0, 0.0, 0.0), PoseAt(-1.2, 0.0, 0.0),
                                          PoseAt(1.0, 0.1, -0.3), PoseAt(0.4, 0.2, -0.6)};
  const std::vector<ScenePoint> pts = {{V3(0.0, -1.0, 3.0), {0, 1, 2}},         // exactly 3 keyframes
                                       {V3(0.3, -1.0, 3.5), {0, 1, 2, 3}}};     // 4 keyframes
  const SceneResult sc = BuildScene(&wm, poses, pts, -1.0);
  map::F2Finalize(wm, Args(1));
  for (const map::PointId p : sc.points) {
    ASSERT_EQ(static_cast<int>(wm.Point(p).f2.state), static_cast<int>(F2Status::kConfirmed));
  }
  // Delete the observation in keyframe 2 (used for the confirmation) of both points.
  for (const map::PointId p : sc.points) {
    const uint32_t idx = wm.Point(p).observations.at(sc.kfs[2]);
    wm.AddObservation(map::PointId::nullId(), idx, sc.kfs[2]);
  }
  EXPECT_EQ(Counters().f2_invalidation[kInvObsRemoved], 2u);
  map::F2Finalize(wm, Args(2));
  // 3 -> 2 keyframes: too_few_kfs. 4 -> 3 keyframes: re-evaluated on the new basis, still confirmed.
  const map::F2PointState& three = wm.Point(sc.points[0]).f2;
  EXPECT_EQ(static_cast<int>(three.state), static_cast<int>(F2Status::kUnconfirmed));
  EXPECT_EQ(static_cast<int>(three.last_reason), static_cast<int>(F2Reason::kTooFewKfs));
  EXPECT_EQ(three.eval_frame, 2);
  const map::F2PointState& four = wm.Point(sc.points[1]).f2;
  EXPECT_EQ(static_cast<int>(four.state), static_cast<int>(F2Status::kConfirmed));
  EXPECT_EQ(four.eval_frame, 2) << "the basis changed: it was evaluated again in the same finalize";
  EXPECT_EQ(four.basis.size(), 3u);
  EXPECT_EQ(Counters().f2_dirty_remainder, 0u);
}

TEST(P2F2Lifecycle, Lifecycle_merge_point) {
  ScopedAccuracy acc(false, true);
  map::WorldMap wm;
  const std::vector<c8::HMatrix> poses = {PoseAt(0.0, 0.0, 0.0), PoseAt(-1.2, 0.0, 0.0),
                                          PoseAt(1.0, 0.1, -0.3), PoseAt(0.4, 0.2, -0.6),
                                          PoseAt(-0.5, 0.1, 0.2)};
  // Two seeds of the same world point: dst sees it from keyframes 0 and 1, src from 2, 3, 4.
  const std::vector<ScenePoint> pts = {{V3(0.0, -1.0, 3.0), {0, 1}}, {V3(0.0, -1.0, 3.0), {2, 3, 4}}};
  const SceneResult sc = BuildScene(&wm, poses, pts, -1.0);
  const map::PointId dst = sc.points[0];
  const map::PointId src = sc.points[1];
  map::F2Finalize(wm, Args(1));
  EXPECT_EQ(static_cast<int>(wm.Point(dst).f2.last_reason), static_cast<int>(F2Reason::kTooFewKfs));
  EXPECT_EQ(static_cast<int>(wm.Point(src).f2.state), static_cast<int>(F2Status::kConfirmed));
  const size_t merges_before = Counters().f2_invalidation[kInvMerge];

  wm.MergePoint(src, dst);
  EXPECT_EQ(Counters().f2_invalidation[kInvMerge] - merges_before, 1u);
  // src is dead and its state is gone.
  ASSERT_NE(wm.FindPoint(src), nullptr);
  EXPECT_EQ(static_cast<int>(wm.FindPoint(src)->state), static_cast<int>(map::PointState::kDead));
  EXPECT_FALSE(wm.FindPoint(src)->f2.evaluated);
  EXPECT_EQ(static_cast<int>(wm.FindPoint(src)->f2.state), static_cast<int>(F2Status::kUnconfirmed));
  EXPECT_EQ(wm.Point(dst).observations.size(), 5u);
  map::F2Finalize(wm, Args(2));
  // dst is evaluated again on its larger basis (creator: keyframe 0).
  EXPECT_EQ(static_cast<int>(wm.Point(dst).f2.state), static_cast<int>(F2Status::kConfirmed));
  EXPECT_EQ(wm.Point(dst).f2.eval_frame, 2);
  EXPECT_EQ(wm.Point(dst).f2.basis.size(), 5u);
}

TEST(P2F2Lifecycle, Lifecycle_creator_kf_slot_reuse) {
  ScopedAccuracy acc(false, true);
  map::WorldMap wm;
  const V3 x(0.0, -1.0, 3.0);
  const std::vector<c8::HMatrix> poses = {PoseAt(0.0, 0.0, 0.0), PoseAt(-1.2, 0.0, 0.0),
                                          PoseAt(1.0, 0.1, -0.3), PoseAt(0.4, 0.2, -0.6),
                                          PoseAt(-0.5, 0.1, 0.2)};
  const std::vector<ScenePoint> pts = {{x, {0, 1, 2, 3}}};
  const SceneResult sc = BuildScene(&wm, poses, pts, -1.0);
  const map::PointId p = sc.points[0];
  map::F2Finalize(wm, Args(1));
  ASSERT_EQ(static_cast<int>(wm.Point(p).f2.state), static_cast<int>(F2Status::kConfirmed));

  wm.RemoveKeyframe(sc.kfs[0]);  // the creator
  EXPECT_EQ(Counters().f2_invalidation[kInvCreatorDeleted], 1u);
  // A new keyframe reuses the slot (new random id) and sees the point again.
  const map::KeyframeId reused = AddSingleRayKeyframe(&wm, PoseAt(0.0, 0.0, 0.0), x, 77, 20);
  EXPECT_EQ(reused.index(), sc.kfs[0].index());
  EXPECT_NE(reused.id(), sc.kfs[0].id());
  wm.AddObservation(p, 0, reused);
  map::F2Finalize(wm, Args(2));
  EXPECT_EQ(wm.Point(p).observations.size(), 4u);  // four keyframes, but not the creator
  EXPECT_EQ(static_cast<int>(wm.Point(p).f2.state), static_cast<int>(F2Status::kUnconfirmed));
  EXPECT_EQ(static_cast<int>(wm.Point(p).f2.last_reason), static_cast<int>(F2Reason::kTooFewKfs));
  EXPECT_TRUE(wm.Point(p).f2.no_creator);
  EXPECT_GE(Counters().f2_no_creator, 1u);
  // More observations never bring it back: the plane depth is undefined without the creator.
  const map::KeyframeId another = AddSingleRayKeyframe(&wm, PoseAt(0.9, -0.1, 0.4), x, 78, 25);
  wm.AddObservation(p, 0, another);
  map::F2Finalize(wm, Args(3));
  EXPECT_EQ(static_cast<int>(wm.Point(p).f2.state), static_cast<int>(F2Status::kUnconfirmed));
  EXPECT_EQ(wm.Point(p).f2.eval_frame, 2) << "a point without creator is not evaluated again";
}

TEST(P2F2Lifecycle, Lifecycle_tracker_epoch_hold) {
  ScopedAccuracy acc(false, true);
  map::WorldMap wm;
  const SceneResult sc = BuildHeightScene(&wm, -1.0, 10);
  map::F2Finalize(wm, Args(5));
  const size_t evals = Eval(F2Reason::kConfirmed);
  ASSERT_EQ(evals, 20u);
  for (const map::PointId p : sc.points) {
    ASSERT_EQ(static_cast<int>(wm.Point(p).f2.state), static_cast<int>(F2Status::kConfirmed));
  }
  map::F2BumpTrackerEpoch(wm);  // LOST / relocalization
  map::F2Finalize(wm, Args(6, /*hold=*/true));
  for (const map::PointId p : sc.points) {
    EXPECT_EQ(static_cast<int>(wm.Point(p).f2.state), static_cast<int>(F2Status::kUnconfirmed));
  }
  EXPECT_EQ(Counters().f2_invalidation[kInvTrackerEpoch], 20u);
  EXPECT_EQ(Counters().f2_hold_frames, 1u);
  EXPECT_EQ(Eval(F2Reason::kConfirmed), evals) << "nothing is evaluated while held";
  map::F2Finalize(wm, Args(7, /*hold=*/true));
  EXPECT_EQ(Counters().f2_hold_frames, 2u);
  EXPECT_EQ(Eval(F2Reason::kConfirmed), evals);
  for (const map::PointId p : sc.points) {
    EXPECT_EQ(static_cast<int>(wm.Point(p).f2.state), static_cast<int>(F2Status::kUnconfirmed));
  }
  map::F2Finalize(wm, Args(8, /*hold=*/false));  // hold over: confirmed again
  for (const map::PointId p : sc.points) {
    EXPECT_EQ(static_cast<int>(wm.Point(p).f2.state), static_cast<int>(F2Status::kConfirmed));
    EXPECT_EQ(wm.Point(p).f2.eval_frame, 8);
  }
  EXPECT_EQ(Eval(F2Reason::kConfirmed), evals + 20u);
  EXPECT_EQ(Counters().f2_dirty_remainder, 0u);
}

TEST(P2F2Lifecycle, Lifecycle_map_reset) {
  ScopedAccuracy acc(false, true);
  map::WorldMaps maps;
  BuildHeightScene(&maps.active(), -1.0, 5);
  map::F2Finalize(maps.active(), Args(1));
  EXPECT_EQ(Eval(F2Reason::kConfirmed), 10u);
  maps.ResetMap();
  EXPECT_EQ(Counters().f2_invalidation[kInvMapReset], 10u);
  const std::array<int, 5> by_source = maps.active().NumPointsBySource();
  EXPECT_EQ(by_source[1], 0);  // no ground points: no F2 state at all
  EXPECT_EQ(maps.active().NumKeyframes(), 0);
  // The new map starts clean: a seed has no state until it is evaluated.
  const SceneResult sc = BuildScene(&maps.active(), kPoses3, {kPoint}, -1.0);
  EXPECT_FALSE(maps.active().Point(sc.points[0]).f2.evaluated);
  EXPECT_EQ(static_cast<int>(maps.active().Point(sc.points[0]).f2.state),
            static_cast<int>(F2Status::kUnconfirmed));
}

// Same input, either finalize exit: identical states.
TEST(P2F2Lifecycle, Hook_path_equivalence) {
  ScopedAccuracy acc(false, true);
  map::WorldMap a;
  map::WorldMap b;
  const SceneResult sa = BuildHeightScene(&a, -0.9, 12);  // a mix of outcomes
  const SceneResult sb = BuildHeightScene(&b, -0.9, 12);
  map::F2Finalize(a, Args(4, false, /*kf=*/true));
  map::F2Finalize(b, Args(4, false, /*kf=*/false));
  EXPECT_EQ(Counters().f2_path_kf, 1u);
  EXPECT_EQ(Counters().f2_path_nonkf, 1u);
  ASSERT_EQ(sa.points.size(), sb.points.size());
  for (size_t i = 0; i < sa.points.size(); ++i) {
    const map::F2PointState& x = a.Point(sa.points[i]).f2;
    const map::F2PointState& y = b.Point(sb.points[i]).f2;
    EXPECT_EQ(static_cast<int>(x.state), static_cast<int>(y.state)) << i;
    EXPECT_EQ(static_cast<int>(x.last_reason), static_cast<int>(y.last_reason)) << i;
    EXPECT_EQ(x.basis_hash, y.basis_hash) << i;
    EXPECT_EQ(x.d_mv, y.d_mv) << i;
    EXPECT_EQ(x.d_plane, y.d_plane) << i;
    EXPECT_EQ(x.ratio, y.ratio) << i;
    EXPECT_EQ(x.eval_frame, y.eval_frame) << i;
  }
}

}  // namespace
}  // namespace slam_refact::p2

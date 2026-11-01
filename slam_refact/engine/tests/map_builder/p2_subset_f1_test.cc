// P2 candidates: (F2) the ground preference of SelectSubset follows the confirmation state with
// one-frame latency and a cap that unconfirmed points do not consume; (F1) seeds of a keyframe are
// placed on the tracker's measured plane when it is valid, otherwise on y = -1 with a counted reason
// (f2-contract-v3.3.md section 7: subset ID tests and F1 tests).

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <memory>
#include <random>
#include <set>
#include <vector>

#include "Eigen/Geometry"
#include "common/tracking_types.h"
#include "config/settings.h"
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
using map::F2Status;

// ---------------------------------------------------------------------------------------------
// SelectSubset

const std::vector<c8::HMatrix>& kPosesForSubset() {
  static const std::vector<c8::HMatrix> poses = {PoseAt(0.0, 0.0, 0.0), PoseAt(-1.2, 0.0, 0.0),
                                                 PoseAt(1.0, 0.1, -0.3)};
  return poses;
}

// A tracking result over `ids` in this order (all rays equal their projections: zero residual).
map_tracker::MapTrackResult MakeResult(const std::vector<map::PointId>& ids) {
  map_tracker::MapTrackResult r;
  r.projected.frame = c8::FrameWithPoints(TestIntrinsics());
  std::vector<RayFeature> rays;
  for (size_t i = 0; i < ids.size(); ++i) {
    rays.push_back({-0.3f + 0.6f * static_cast<float>(i % 20) / 19.0f,
                    -0.3f + 0.6f * static_cast<float>(i / 20) / 6.0f, 1,
                    static_cast<uint32_t>(50000 + i)});
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

std::set<uint32_t> SubsetKeys(const map_tracker::MapTrackResult& r) {
  std::set<uint32_t> out;
  for (const map_tracker::TrackedMatch& m : r.subset_matches) {
    out.insert(m.id.id());
  }
  return out;
}

size_t CountIn(const std::set<uint32_t>& subset, const std::vector<map::PointId>& ids, size_t from,
               size_t to) {
  size_t n = 0;
  for (size_t i = from; i < to; ++i) {
    n += subset.count(ids[i].id());
  }
  return n;
}

map::PointId AddBare(map::WorldMap* wm, map::PointSource source, float x) {
  return wm->AddPoint(wm->keyframe_seq_counter(), c8::HPoint3(x, 0.f, 3.f), source);
}

TEST(P2F2Subset, SubsetId_bonus_next_frame_only) {
  ScopedAccuracy acc(false, true);
  EXPECT_EQ(config::WorldMapsSettings::Get().encouraged_n_ground_points, 52);
  EXPECT_EQ(config::WorldMapsSettings::Get().max_subset_points, 75);
  map::WorldMap wm;
  // 10 ground seeds that F2 will confirm, 25 tenured ground seeds (score 0.35 either way) and 65
  // triangulated points (score 0.25). Unconfirmed ground seeds score 0.15, confirmed ones 0.70.
  std::vector<ScenePoint> spread(10, ScenePoint{V3(0.0, -1.0, 3.0), {0, 1, 2}});
  for (size_t i = 0; i < spread.size(); ++i) {
    spread[i].world = V3(-0.9 + 0.2 * static_cast<double>(i), -1.0, 3.0 + 0.1 * static_cast<double>(i));
  }
  const SceneResult sc = BuildScene(&wm, kPosesForSubset(), spread, -1.0);
  std::vector<map::PointId> ids = sc.points;  // [0, 10): confirmable
  for (int i = 0; i < 25; ++i) {
    const map::PointId t = AddBare(&wm, map::PointSource::kGround, static_cast<float>(i));
    wm.MutablePoint(t).state = map::PointState::kTenured;
    ids.push_back(t);
  }
  for (int i = 0; i < 65; ++i) {
    ids.push_back(AddBare(&wm, map::PointSource::kTriangulated, static_cast<float>(i)));
  }
  ASSERT_EQ(ids.size(), 100u);

  // Tracking frame e: the seeds are unconfirmed (no finalize yet).
  map_tracker::MapTrackResult at_e = MakeResult(ids);
  map_tracker::SelectSubset(wm, &at_e);
  ASSERT_EQ(at_e.subset_matches.size(), 75u);
  EXPECT_EQ(CountIn(SubsetKeys(at_e), ids, 0, 10), 0u) << "no bonus before the confirmation is published";
  EXPECT_EQ(CountIn(SubsetKeys(at_e), ids, 10, 35), 25u);  // the tenured ground seeds (0.35)
  EXPECT_EQ(Counters().sel_bonus_granted, 0u);

  // Update(e) computes the confirmation ...
  const int64_t e = 7;
  map::F2Finalize(wm, Args(e));
  for (size_t i = 0; i < 10; ++i) {
    ASSERT_EQ(static_cast<int>(wm.Point(ids[i]).f2.state), static_cast<int>(F2Status::kConfirmed));
    EXPECT_EQ(wm.Point(ids[i]).f2.eval_frame, e);
    EXPECT_EQ(wm.Point(ids[i]).f2.applied_frame, e + 1);
  }
  EXPECT_EQ(map::F2AppliedFrame(), e + 1);
  // ... and tracking frame e + 1 sees it: all ten score 0.70 and are in the subset.
  map_tracker::MapTrackResult at_e1 = MakeResult(ids);
  map_tracker::SelectSubset(wm, &at_e1);
  ASSERT_EQ(at_e1.subset_matches.size(), 75u);
  EXPECT_EQ(CountIn(SubsetKeys(at_e1), ids, 0, 10), 10u);
  EXPECT_EQ(CountIn(SubsetKeys(at_e1), ids, 10, 35), 25u);
  EXPECT_EQ(Counters().sel_bonus_granted, 10u);
  EXPECT_EQ(Counters().sel_new_confirmed_subset, 10u);
  EXPECT_EQ(Counters().sel_new_confirmed_all, 10u);
  EXPECT_EQ(Counters().sel_active, 2u);
  // The first ten subset slots are the confirmed seeds (score order).
  std::set<uint32_t> top;
  for (size_t i = 0; i < 10; ++i) {
    top.insert(at_e1.subset_matches[i].id.id());
  }
  EXPECT_EQ(CountIn(top, ids, 0, 10), 10u);
}

TEST(P2F2Subset, SubsetId_cap_60_and_unconfirmed_does_not_consume) {
  std::vector<map::PointId> ids;
  std::vector<ScenePoint> spread(60, ScenePoint{V3(0.0, -1.0, 3.0), {0, 1, 2}});
  for (size_t i = 0; i < spread.size(); ++i) {
    spread[i].world = V3(-1.5 + 0.05 * static_cast<double>(i), -1.0, 3.0 + 0.02 * static_cast<double>(i));
  }
  auto build = [&](map::WorldMap* wm, std::vector<map::PointId>* out_ids) {
    for (int i = 0; i < 20; ++i) {  // 20 unconfirmed ground seeds FIRST in match order
      out_ids->push_back(AddBare(wm, map::PointSource::kGround, static_cast<float>(i)));
    }
    const SceneResult sc = BuildScene(wm, kPosesForSubset(), spread, -1.0);  // 60 confirmable
    out_ids->insert(out_ids->end(), sc.points.begin(), sc.points.end());
    for (int i = 0; i < 40; ++i) {
      out_ids->push_back(AddBare(wm, map::PointSource::kTriangulated, static_cast<float>(i)));
    }
  };
  {
    ScopedAccuracy acc(false, true);
    map::WorldMap wm;
    build(&wm, &ids);
    ASSERT_EQ(ids.size(), 120u);
    map::F2Finalize(wm, Args(3));
    size_t confirmed = 0;
    for (size_t i = 20; i < 80; ++i) {
      confirmed += wm.Point(ids[i]).f2.state == F2Status::kConfirmed ? 1 : 0;
    }
    ASSERT_EQ(confirmed, 60u);
    map_tracker::MapTrackResult r = MakeResult(ids);
    map_tracker::SelectSubset(wm, &r);
    const std::set<uint32_t> subset = SubsetKeys(r);
    ASSERT_EQ(subset.size(), 75u);
    // The first 52 confirmed seeds (match order) are encouraged; the 53rd..60th are not; the
    // unconfirmed seeds in front of them did not use up the cap.
    EXPECT_EQ(CountIn(subset, ids, 20, 72), 52u);
    EXPECT_EQ(CountIn(subset, ids, 72, 80), 0u);
    EXPECT_EQ(CountIn(subset, ids, 0, 20), 0u);
    EXPECT_EQ(CountIn(subset, ids, 80, 120), 23u);
    EXPECT_EQ(Counters().sel_bonus_granted, 52u);
    EXPECT_EQ(Counters().sel_cap_reached, 1u);
    EXPECT_EQ(Counters().sel_all_ground[0], 60u);
    EXPECT_EQ(Counters().sel_all_ground[1], 20u);
  }
  {
    // F2 off: the original rule, the first 52 ground points in match order (unconfirmed ones too).
    ScopedAccuracy acc(false, false);
    std::vector<map::PointId> ids_off;
    map::WorldMap wm;
    build(&wm, &ids_off);
    map_tracker::MapTrackResult r = MakeResult(ids_off);
    map_tracker::SelectSubset(wm, &r);
    const std::set<uint32_t> subset = SubsetKeys(r);
    ASSERT_EQ(subset.size(), 75u);
    EXPECT_EQ(CountIn(subset, ids_off, 0, 52), 52u);
    EXPECT_EQ(CountIn(subset, ids_off, 52, 80), 0u);
    EXPECT_EQ(Counters().sel_bonus_granted, 0u) << "counters stay 0 while F2 is off";
  }
}

// A keyframe that was removed and whose id is drawn again by a new keyframe, before any finalize:
// the point's confirmed state names the old keyframe object, so it must not give a bonus.
TEST(P2F2Subset, IdReuse_stale_bonus_before_finalize) {
  ScopedAccuracy acc(false, true);
  std::mt19937 rng(99);
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
  const map::KeyframeId k0 = add_kf(PoseAt(0.0, 0.0, 0.0), 1, 0);
  const map::KeyframeId k1 = add_kf(PoseAt(-1.2, 0.0, 0.0), 2, 5);
  const std::mt19937 before_k3 = rng;
  const map::KeyframeId k3 = add_kf(PoseAt(1.0, 0.1, -0.3), 3, 10);
  const map::PointId p = wm.AddPoint(wm.GetKeyframe(k0).seq, c8::HPoint3(0.f, -1.f, 3.f),
                                     map::PointSource::kGround);
  wm.AddObservation(p, 0, k0);
  wm.AddObservation(p, 0, k1);
  wm.AddObservation(p, 0, k3);
  map::F2Finalize(wm, Args(1));
  ASSERT_EQ(static_cast<int>(wm.Point(p).f2.state), static_cast<int>(F2Status::kConfirmed));

  std::vector<map::PointId> ids = {p};
  for (int i = 0; i < 25; ++i) {
    const map::PointId t = AddBare(&wm, map::PointSource::kGround, static_cast<float>(i));
    wm.MutablePoint(t).state = map::PointState::kTenured;
    ids.push_back(t);
  }
  for (int i = 0; i < 65; ++i) {
    ids.push_back(AddBare(&wm, map::PointSource::kTriangulated, static_cast<float>(i)));
  }
  map_tracker::MapTrackResult control = MakeResult(ids);
  map_tracker::SelectSubset(wm, &control);
  EXPECT_EQ(SubsetKeys(control).count(p.id()), 1u) << "confirmed: in the subset";

  wm.RemoveKeyframe(k3);
  rng = before_k3;
  const map::KeyframeId k3b = add_kf(PoseAt(1.0, 0.1, -0.3), 3, 12);  // same slot and id, new serial
  ASSERT_TRUE(k3b == k3);
  wm.AddObservation(p, 0, k3b);
  const uint64_t reuse_before = Counters().f2_id_reuse;
  map_tracker::MapTrackResult stale = MakeResult(ids);
  map_tracker::SelectSubset(wm, &stale);  // no finalize in between
  EXPECT_EQ(SubsetKeys(stale).count(p.id()), 0u) << "stale confirmed state must not give the bonus";
  for (int i = 0; i < 3; ++i) {  // queries have no side effect
    map_tracker::MapTrackResult again = MakeResult(ids);
    map_tracker::SelectSubset(wm, &again);
    EXPECT_EQ(SubsetKeys(again).count(p.id()), 0u);
  }
  EXPECT_EQ(Counters().f2_id_reuse, reuse_before);
  map::F2Finalize(wm, Args(2));
  EXPECT_EQ(Counters().f2_id_reuse - reuse_before, 1u) << "exactly one count per reuse event";
  map_tracker::MapTrackResult fresh = MakeResult(ids);
  map_tracker::SelectSubset(wm, &fresh);
  EXPECT_EQ(SubsetKeys(fresh).count(p.id()), 1u) << "re-evaluated on the new keyframe: confirmed again";
}

// ---------------------------------------------------------------------------------------------
// F1

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

struct TrkOptions {
  float ground_y = -0.6f;
  uint8_t reason = kF1Ok;
  bool candidates_are_sky = false;  // ground candidates with upward rays (lambda <= 0)
};

MapTrackerOutput MakeTrk(int32_t state, int64_t consecutive, const c8::HMatrix& pose,
                         const TrkOptions& o) {
  MapTrackerOutput trk;
  trk.pose = pose;
  trk.state = state;
  trk.frame_tracker_ok = true;
  trk.consecutive_frame_tracker_ok = consecutive;
  trk.f1_ground_y = o.ground_y;
  trk.f1_invalid_reason = o.reason;
  if (o.candidates_are_sky) {
    for (uint32_t i = 0; i < kSky; ++i) {
      trk.ground_idx.push_back(kGround + i);
    }
  } else {
    for (uint32_t i = 0; i < kGround; ++i) {
      trk.ground_idx.push_back(i);
    }
  }
  for (uint32_t i = 0; i < kSky; ++i) {
    trk.sky_idx.push_back(kGround + i);
  }
  return trk;
}

c8::HMatrix PoseOf(int f) {
  return Pose(0.002f * static_cast<float>(f), 0.01f * static_cast<float>(f), 0.f, 0.f);
}

struct KeyframeRun {
  std::vector<map::PointId> frame0_ground;  // seeds of the reset keyframe
  std::vector<map::PointId> frame5_ground;  // seeds of the keyframe at frame 5
  AccuracyCounters before5;                 // counters just before frame 5
  bool kf5 = false;
};

// Frames 0..5 (reset keyframe at 0, next keyframe at 5). `opt5` shapes the tracker output of frame 5.
KeyframeRun RunKeyframes(map::WorldMaps* maps, const TrkOptions& opt0, const TrkOptions& opt5) {
  MapBuilder mb;
  KeyframeRun run;
  for (int f = 0; f <= 5; ++f) {
    if (f == 5) {
      run.before5 = Counters();
    }
    MapBuilderOutput out;
    const std::unique_ptr<TrackerInput> frm = MakeInput(f);
    mb.Update(maps, *frm, MakeTrk(f == 0 ? 0 : 1, 7 + f, PoseOf(f), f == 5 ? opt5 : opt0), f * 33,
              &out);
    for (const map::PointId id : out.new_points) {
      if (maps->active().Point(id).source == map::PointSource::kGround) {
        (f == 0 ? run.frame0_ground : run.frame5_ground).push_back(id);
      }
    }
    if (f == 5) {
      run.kf5 = out.keyframe_created;
    }
  }
  return run;
}

TEST(P2F1, F1_valid_ground_y_applied) {
  ScopedAccuracy acc(true, false);
  map::WorldMaps maps;
  TrkOptions valid;
  valid.ground_y = -0.6f;
  const KeyframeRun run = RunKeyframes(&maps, valid, valid);
  ASSERT_TRUE(run.kf5);
  ASSERT_EQ(run.frame5_ground.size(), static_cast<size_t>(kGround));
  for (const map::PointId id : run.frame5_ground) {
    const map::WorldPoint& p = maps.active().Point(id);
    EXPECT_NEAR(p.position.y(), -0.6f, 2e-3f) << "seed on the measured plane";
    EXPECT_FLOAT_EQ(static_cast<float>(p.f2.plane_y), -0.6f) << "plane value recorded on the point";
  }
  EXPECT_EQ(Counters().f1_applied, static_cast<uint64_t>(kGround));
  EXPECT_EQ(Counters().f1_kf_applied, 1u);
}

TEST(P2F1, F1_initial_keyframe_minus_one) {
  ScopedAccuracy acc(true, false);
  map::WorldMaps maps;
  TrkOptions valid;
  valid.ground_y = -0.6f;  // valid measurement, but the reset keyframe defines the unit
  const KeyframeRun run = RunKeyframes(&maps, valid, valid);
  ASSERT_EQ(run.frame0_ground.size(), static_cast<size_t>(kGround));
  for (const map::PointId id : run.frame0_ground) {
    EXPECT_NEAR(maps.active().Point(id).position.y(), -1.0f, 1e-4f);
    EXPECT_DOUBLE_EQ(maps.active().Point(id).f2.plane_y, -1.0);
  }
  EXPECT_EQ(Counters().f1_fallback[kF1InitialKeyframe], static_cast<uint64_t>(kGround));
  EXPECT_EQ(run.before5.f1_applied, 0u);
  EXPECT_EQ(run.before5.f1_kf_fallback, 1u);
}

TEST(P2F1, F1_fallback_reasons_counted) {
  // Tracker side: the 15-frame window, samples, epoch.
  EXPECT_EQ(map_tracker::F1ReasonFor(100, 86, 8, -0.5f), kF1Ok);              // age 14: inside
  EXPECT_EQ(map_tracker::F1ReasonFor(100, 85, 8, -0.5f), kF1Stale);           // age 15: outside
  EXPECT_EQ(map_tracker::F1ReasonFor(100, 100, 8, -0.5f), kF1Ok);             // updated this frame
  EXPECT_EQ(map_tracker::F1ReasonFor(100, -1, 3, -0.5f), kF1InsufficientSamples);
  EXPECT_EQ(map_tracker::F1ReasonFor(100, -1, 0, -0.5f), kF1InsufficientSamples);
  EXPECT_EQ(map_tracker::F1ReasonFor(100, -1, -1, -0.5f), kF1EpochInvalid);   // cleared, no attempt yet
  EXPECT_EQ(map_tracker::F1ReasonFor(100, 99, 8, std::numeric_limits<float>::quiet_NaN()),
            kF1InvalidInput);

  // MapBuilder side: every reason leaves the seeds on y = -1 and is counted once per seed.
  struct Case {
    const char* name;
    TrkOptions opt;
    F1Reason reason;
    size_t seeds;
  };
  std::vector<Case> cases;
  {
    TrkOptions o;
    o.reason = kF1Stale;
    cases.push_back({"stale", o, kF1Stale, kGround});
    o.reason = kF1InsufficientSamples;
    cases.push_back({"insufficient_samples", o, kF1InsufficientSamples, kGround});
    o.reason = kF1EpochInvalid;
    cases.push_back({"epoch_invalid", o, kF1EpochInvalid, kGround});
    TrkOptions close;
    close.ground_y = -0.049f;  // camera 0.049 above the plane (< 0.05)
    cases.push_back({"camera_too_close", close, kF1CameraTooClose, kGround});
    TrkOptions nan;
    nan.ground_y = std::numeric_limits<float>::quiet_NaN();
    cases.push_back({"invalid_input", nan, kF1InvalidInput, kGround});
    TrkOptions up;
    up.candidates_are_sky = true;  // valid plane, rays that point away from it: lambda <= 0
    cases.push_back({"lambda_nonpositive", up, kF1LambdaNonpositive, kSky});
  }
  for (const Case& c : cases) {
    SCOPED_TRACE(c.name);
    ScopedAccuracy acc(true, false);
    map::WorldMaps maps;
    TrkOptions plain;
    const KeyframeRun run = RunKeyframes(&maps, plain, c.opt);
    ASSERT_TRUE(run.kf5);
    const AccuracyCounters& after = Counters();
    EXPECT_EQ(after.f1_fallback[c.reason] - run.before5.f1_fallback[c.reason], c.seeds);
    EXPECT_EQ(after.f1_applied, 0u);
    for (int r = 1; r < kF1ReasonCount; ++r) {
      if (r != c.reason && r != kF1InitialKeyframe) {
        EXPECT_EQ(after.f1_fallback[static_cast<size_t>(r)], 0u) << kF1ReasonNames[r];
      }
    }
    // Fallback seeds sit on the original plane y = -1.
    ASSERT_FALSE(run.frame5_ground.empty());
    for (const map::PointId id : run.frame5_ground) {
      if (c.reason != kF1LambdaNonpositive) {
        EXPECT_NEAR(maps.active().Point(id).position.y(), -1.0f, 1e-3f);
      }
      EXPECT_DOUBLE_EQ(maps.active().Point(id).f2.plane_y, -1.0);
    }
  }
  // Just above the camera-height limit the measured plane is used (boundary 0.05).
  {
    ScopedAccuracy acc(true, false);
    map::WorldMaps maps;
    TrkOptions plain;
    TrkOptions ok;
    ok.ground_y = -0.051f;
    const KeyframeRun run = RunKeyframes(&maps, plain, ok);
    EXPECT_EQ(Counters().f1_applied, static_cast<uint64_t>(kGround));
    EXPECT_EQ(run.frame5_ground.size(), static_cast<size_t>(kGround));
  }
}

// The measured plane of a seed survives a pose update of its creating keyframe (SetKeyframePose
// re-derives initial depths from y = -1; local BA and loop close go through the same call).
TEST(P2F1, F1_seed_plane_persists_pose_update) {
  ScopedAccuracy acc(true, false);
  map::WorldMaps maps;
  TrkOptions valid;
  valid.ground_y = -0.6f;
  const KeyframeRun run = RunKeyframes(&maps, valid, valid);
  ASSERT_TRUE(run.kf5);
  map::WorldMap& wm = maps.active();
  const map::KeyframeId nk = wm.newest();
  ASSERT_EQ(run.frame5_ground.size(), static_cast<size_t>(kGround));
  for (const map::PointId id : run.frame5_ground) {
    ASSERT_NEAR(wm.Point(id).position.y(), -0.6f, 2e-3f);
  }
  const float x_before = wm.Point(run.frame5_ground[0]).position.x();
  wm.SetKeyframePose(nk, Pose(0.01f, 0.35f, 0.f, 0.f));  // the camera moved sideways
  wm.AdjustDirtyPoints();
  for (const map::PointId id : run.frame5_ground) {
    EXPECT_NEAR(wm.Point(id).position.y(), -0.6f, 2e-3f) << "seed stays on the measured plane";
  }
  EXPECT_GT(std::fabs(wm.Point(run.frame5_ground[0]).position.x() - x_before), 0.05f)
      << "the pose update did move the seed";
}

// Same with a pose that has pitch and roll (not only yaw): the plane depth must be derived from the
// camera-to-world rotation correctly (column-major world_from_camera).
TEST(P2F1, F1_seed_plane_persists_pose_update_roll_pitch) {
  ScopedAccuracy acc(true, false);
  map::WorldMaps maps;
  TrkOptions valid;
  valid.ground_y = -0.6f;
  const KeyframeRun run = RunKeyframes(&maps, valid, valid);
  ASSERT_TRUE(run.kf5);
  map::WorldMap& wm = maps.active();
  const map::KeyframeId nk = wm.newest();
  const Eigen::Matrix3d r = (Eigen::AngleAxisd(0.2, Eigen::Vector3d::UnitX()) *
                             Eigen::AngleAxisd(0.15, Eigen::Vector3d::UnitZ()))
                                .toRotationMatrix();
  float m[4][4] = {};
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      m[i][j] = static_cast<float>(r(i, j));  // rows of the camera-to-world rotation
    }
  }
  m[3][3] = 1.0f;
  const c8::HMatrix pose = c8::HMatrixGen::translation(0.3f, 0.0f, 0.1f) *
                           c8::HMatrix(m[0], m[1], m[2], m[3]);
  wm.SetKeyframePose(nk, pose);
  wm.AdjustDirtyPoints();
  for (const map::PointId id : run.frame5_ground) {
    EXPECT_NEAR(wm.Point(id).position.y(), -0.6f, 2e-3f) << "seed stays on the measured plane";
  }
}

// A snapshot of everything the sequence leaves in the map (ids, positions, keyframe poses).
struct MapSnapshot {
  std::vector<uint32_t> ids;
  std::vector<float> positions;
  std::vector<float> poses;
  bool operator==(const MapSnapshot& o) const {
    return ids == o.ids && positions == o.positions && poses == o.poses;
  }
};

MapSnapshot Snapshot(const map::WorldMap& wm) {
  MapSnapshot s;
  for (const map::PointId id : wm.AllPointIds()) {
    const map::WorldPoint& p = wm.Point(id);
    s.ids.push_back(p.id);
    s.positions.insert(s.positions.end(), {p.position.x(), p.position.y(), p.position.z()});
  }
  for (const map::KeyframeId kf : wm.AllKeyframesByCreation()) {
    const c8::HMatrix& m = wm.GetKeyframe(kf).frame.extrinsic();
    for (int r = 0; r < 3; ++r) {
      for (int c = 0; c < 4; ++c) {
        s.poses.push_back(m(r, c));
      }
    }
  }
  return s;
}

TEST(P2F1, F1_off_bit_identical) {
  TrkOptions misleading;  // a valid measured plane that must be ignored while F1 is off
  misleading.ground_y = -0.6f;
  TrkOptions neutral;
  neutral.ground_y = -1.0f;
  neutral.reason = kF1EpochInvalid;
  auto run = [&](bool accuracy_enabled, bool f1, bool f2, const TrkOptions& opt, MapSnapshot* snap) {
    AccuracyConfig cfg;
    cfg.enabled = accuracy_enabled;
    cfg.f1_ground_plane = f1;
    cfg.f2_ground_confirmation = f2;
    const AccuracyConfig previous = config::Accuracy();
    config::SetAccuracy(cfg);
    ResetCounters();
    map::WorldMaps maps;
    MapBuilder mb;
    for (int f = 0; f < 40; ++f) {
      MapBuilderOutput out;
      const std::unique_ptr<TrackerInput> frm = MakeInput(f);
      mb.Update(&maps, *frm, MakeTrk(f == 0 ? 0 : 1, 7 + f, PoseOf(f), opt), f * 33, &out);
    }
    *snap = Snapshot(maps.active());
    config::SetAccuracy(previous);
  };
  MapSnapshot base, enabled_only, with_trk_fields, f1_on, f2_on;
  run(false, false, false, neutral, &base);
  EXPECT_FALSE(base.ids.empty());
  run(true, false, false, neutral, &enabled_only);
  run(false, false, false, misleading, &with_trk_fields);
  EXPECT_TRUE(base == enabled_only);
  EXPECT_TRUE(base == with_trk_fields) << "tracker F1 fields are ignored while F1 is off";
  // The counters of both candidates stay at zero for the off runs.
  EXPECT_EQ(Counters().f1_applied, 0u);
  EXPECT_EQ(Counters().f1_kf_fallback, 0u);
  EXPECT_EQ(Counters().sel_calls, 0u);
  EXPECT_EQ(Counters().f2_path_kf + Counters().f2_path_nonkf, 0u);
  // The test is not vacuous: with F1 on the same inputs give a different map.
  run(true, true, false, misleading, &f1_on);
  EXPECT_FALSE(base == f1_on);
  // F2 does not move points or poses (it only changes the subset selection, not exercised here).
  run(true, false, true, neutral, &f2_on);
  EXPECT_TRUE(base == f2_on);
}

}  // namespace
}  // namespace slam_refact::p2

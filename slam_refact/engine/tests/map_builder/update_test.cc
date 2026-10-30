// MapBuilder::Update and EpipolarMatcher on synthetic inputs (architecture.md §7.3 map_builder):
// reset => same-frame keyframe and ground/sky seeding (MB-UNIT row "reset => keyframe", Q7, sky
// cap 30, world_matches = worldPointIndices().size()), the G-M2/G-M3 invariants as engine-internal
// asserts (keyframe interval >= 5, live keyframes <= 64; §6.6.3 note), the dead-key row
// (4 unread config keys do not change the output), and epipolar midpoint triangulation.

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <deque>
#include <memory>
#include <vector>

#include "c8/hmatrix.h"
#include "c8/hpoint.h"
#include "c8/quaternion.h"
#include "common/tracking_types.h"
#include "config/params.h"
#include "config/settings.h"
#include "gtest/gtest.h"
#include "map/world_map.h"
#include "map_builder/map_builder.h"
#include "map_builder/map_builder_internal.h"
#ifdef SLAM_REFACT_LOOP_CLOSE
#include "map_builder/loop_close.h"
#include "reality/engine/geometry/bundle.h"
#endif
#include "map_builder_test_util.h"

namespace slam_refact::map_builder {
namespace {

using testing::AddRays;
using testing::MakeFrame;
using testing::Pose;
using testing::Project;
using testing::RayFeature;
using testing::TestIntrinsics;

constexpr int kGround = 24;  // rays 0..23 look below the horizon
constexpr int kSky = 40;     // rays 24..63 look above it

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

TEST(MapBuilderUpdate, ResetCreatesKeyframeWithGroundAndSkyPointsSameFrame) {
  map::WorldMaps maps;
  MapBuilder mb;
  const std::unique_ptr<TrackerInput> frm = MakeInput(0);
  ASSERT_EQ(frm->frame_points.worldPointIndices().size(), 0u);  // D-22: world_matches = 0
  const c8::HMatrix pose = Pose(0.2f, 0.1f, 0.3f, -0.4f);
  MapBuilderOutput out;
  mb.Update(&maps, *frm, MakeTrk(0, 7, pose), 0, &out);
  EXPECT_TRUE(out.map_reset);
  EXPECT_TRUE(out.keyframe_created);
  const map::WorldMap& wm = maps.active();
  EXPECT_EQ(wm.NumKeyframes(), 1);
  const std::array<int, 5> by_source = wm.NumPointsBySource();
  EXPECT_EQ(by_source[1], kGround);  // every candidate becomes a ground point (no matches)
  EXPECT_EQ(by_source[2], 30);       // maxNewSkypoints of the 40 sky indices
  EXPECT_EQ(out.new_points.size(), static_cast<size_t>(kGround + 30));
  for (const map::PointId id : out.new_points) {
    const map::WorldPoint& p = wm.Point(id);
    if (p.source == map::PointSource::kGround) {
      EXPECT_NEAR(p.position.y(), -1.0f, 1e-4f);  // plane y = -1
    } else {
      ASSERT_EQ(p.source, map::PointSource::kSky);
      // Created at skypointsDistance (1000) along the ray, then re-placed by 1127 at the
      // observation's initialDepth (FramePoint.skypointsDistance-clamped, 655): still on the ray.
      const c8::HPoint3 c = pose.inv() * p.position;
      size_t obs = p.observations.begin()->second;
      const c8::FramePoint& fp = wm.GetKeyframe(p.observations.begin()->first).frame.points()[obs];
      EXPECT_GT(c.z(), 0.0f);
      EXPECT_NEAR(c.x() / c.z(), fp.x(), 1e-4f);
      EXPECT_NEAR(c.y() / c.z(), fp.y(), 1e-4f);
    }
  }
}

TEST(MapBuilderUpdate, NoResetWithoutEnoughSuccesses) {
  map::WorldMaps maps;
  MapBuilder mb;
  MapBuilderOutput out;
  mb.Update(&maps, *MakeInput(0), MakeTrk(0, 6, c8::HMatrixGen::i()), 0, &out);
  EXPECT_FALSE(out.map_reset);
  EXPECT_FALSE(out.keyframe_created);
  EXPECT_EQ(maps.active().NumKeyframes(), 0);
}

struct RunRecord {
  std::vector<bool> resets;
  std::vector<bool> keyframes;
  std::vector<std::vector<map::PointId>> new_points;
  std::vector<int> live_keyframes;
};

// Reset at frame 0, then tracking (state 1) with a slowly moving camera.
RunRecord RunSequence(int frames) {
  map::WorldMaps maps;
  MapBuilder mb;
  RunRecord rec;
  for (int f = 0; f < frames; ++f) {
    const std::unique_ptr<TrackerInput> frm = MakeInput(f);
    const c8::HMatrix pose = Pose(0.002f * static_cast<float>(f), 0.01f * static_cast<float>(f),
                                  0.f, 0.f);
    MapBuilderOutput out;
    mb.Update(&maps, *frm, MakeTrk(f == 0 ? 0 : 1, 7 + f, pose), f * 33, &out);
    rec.resets.push_back(out.map_reset);
    rec.keyframes.push_back(out.keyframe_created);
    rec.new_points.push_back(out.new_points);
    rec.live_keyframes.push_back(maps.active().NumKeyframes());
  }
  return rec;
}

TEST(MapBuilderUpdate, KeyframeIntervalAndSizeCapInvariants) {
  const RunRecord rec = RunSequence(420);
  int last_kf = -1;
  int keyframes = 0;
  for (size_t f = 0; f < rec.keyframes.size(); ++f) {
    EXPECT_LE(rec.live_keyframes[f], 64) << "frame " << f;  // G-M3 invariant
    if (!rec.keyframes[f]) {
      continue;
    }
    ++keyframes;
    if (!rec.resets[f] && last_kf >= 0) {
      EXPECT_GE(static_cast<int>(f) - last_kf, 5) << "frame " << f;  // G-M2 invariant
    }
    last_kf = static_cast<int>(f);
  }
  EXPECT_TRUE(rec.resets[0]);
  EXPECT_EQ(std::count(rec.resets.begin(), rec.resets.end(), true), 1);
  // Enough keyframes to exercise the 64 cap.
  EXPECT_GT(keyframes, 70);
  EXPECT_EQ(rec.live_keyframes.back(), 64);
  // First keyframes after the reset: delay 3 frames after the state change at frame 1, cooldown
  // 5 from the reset keyframe -> frame 5, then every 5 frames; frame 30 is held by the
  // relocalization delay (30 frames after the reset, state changed 29 frames ago).
  EXPECT_TRUE(rec.keyframes[5]);
  EXPECT_FALSE(rec.keyframes[4]);
  EXPECT_TRUE(rec.keyframes[25]);
  EXPECT_FALSE(rec.keyframes[30]);
  EXPECT_TRUE(rec.keyframes[31]);
}

TEST(MapBuilderUpdate, DeadConfigKeysDoNotChangeOutput) {
  const RunRecord base = RunSequence(120);
  RunRecord changed;
  {
    const testing::ScopedParamOverrides overrides(
        {{"BundleAdjustment.cameraMotionWeight", ParamValue(55.0f)},
         {"BundleAdjustment.numPreviousKeyframesToAdjust", ParamValue(1)},
         {"BundleAdjustment.minNumKeyframesForPointToAdjust", ParamValue(9)},
         {"MapBuilder.keyframeVariabilityCriterion", ParamValue(0.01f)}});
    changed = RunSequence(120);
  }
  EXPECT_EQ(base.resets, changed.resets);
  EXPECT_EQ(base.keyframes, changed.keyframes);
  EXPECT_EQ(base.live_keyframes, changed.live_keyframes);
  ASSERT_EQ(base.new_points.size(), changed.new_points.size());
  for (size_t f = 0; f < base.new_points.size(); ++f) {
    ASSERT_EQ(base.new_points[f].size(), changed.new_points[f].size());
    for (size_t i = 0; i < base.new_points[f].size(); ++i) {
      EXPECT_TRUE(base.new_points[f][i] == changed.new_points[f][i]);
    }
  }
}

#ifdef SLAM_REFACT_LOOP_CLOSE
// US-025 LC-3 smoke tests (detailed unit tests: LC-4). Loop close on, native build only.

// No keyframe revisits another one (every frame carries fresh descriptors), so the loop close
// stage must run (descriptors are computed, the ring is armed) without changing anything the
// OFF run produces.
TEST(MapBuilderLoopClose, SequenceWithoutRevisitMatchesOffRun) {
  const RunRecord off = RunSequence(120);
  RunRecord on;
  {
    const testing::ScopedParamOverrides overrides(
        {{"MapBuilder.loopCloseEnabled", ParamValue(true)}});
    on = RunSequence(120);
  }
  EXPECT_EQ(off.resets, on.resets);
  EXPECT_EQ(off.keyframes, on.keyframes);
  EXPECT_EQ(off.live_keyframes, on.live_keyframes);
  ASSERT_EQ(off.new_points.size(), on.new_points.size());
  for (size_t f = 0; f < off.new_points.size(); ++f) {
    ASSERT_EQ(off.new_points[f].size(), on.new_points[f].size()) << "frame " << f;
    for (size_t i = 0; i < off.new_points[f].size(); ++i) {
      EXPECT_TRUE(off.new_points[f][i] == on.new_points[f][i]);
    }
  }

  // The ON run stores a descriptor in every live keyframe slot; the OFF run never computes one.
  map::WorldMaps maps_on;
  map::WorldMaps maps_off;
  MapBuilder mb_on;
  MapBuilder mb_off;
  for (int f = 0; f < 60; ++f) {
    const c8::HMatrix pose = Pose(0.002f * static_cast<float>(f), 0.01f * static_cast<float>(f),
                                  0.f, 0.f);
    MapBuilderOutput out;
    mb_off.Update(&maps_off, *MakeInput(f), MakeTrk(f == 0 ? 0 : 1, 7 + f, pose), f * 33, &out);
    const testing::ScopedParamOverrides overrides(
        {{"MapBuilder.loopCloseEnabled", ParamValue(true)}});
    mb_on.Update(&maps_on, *MakeInput(f), MakeTrk(f == 0 ? 0 : 1, 7 + f, pose), f * 33, &out);
    EXPECT_FALSE(out.world_transform_updated);
  }
  const std::array<uint8_t, 32> zero{};
  int live_on = 0;
  int described_on = 0;
  int described_off = 0;
  for (size_t slot = 0; slot < maps_on.active().NumKeyframeSlots(); ++slot) {
    const map::Keyframe& kf = maps_on.active().KeyframeAtSlot(slot);
    live_on += kf.id != 0;
    described_on += kf.id != 0 && kf.global_descriptor != zero;
  }
  for (size_t slot = 0; slot < maps_off.active().NumKeyframeSlots(); ++slot) {
    described_off += maps_off.active().KeyframeAtSlot(slot).global_descriptor != zero;
  }
  EXPECT_GE(live_on, 5);
  EXPECT_EQ(described_on, live_on);
  EXPECT_EQ(described_off, 0);
}

// Slot 0 and the newest keyframe (slot 5) see the same 80 world-point appearances (equal
// descriptors, rays of the true revisit pose); the newest keyframe's stored pose is off and it
// holds no point links. Slots 1-4 are far away in descriptor space.
TEST(MapBuilderLoopClose, RevisitReplacesNewestPoseAndLinksPoints) {
  map::WorldMaps maps;
  map::WorldMap& wm = maps.active();
  constexpr int kPoints = 80;
  const c8::HMatrix pose_first = Pose(0.0f, 0.0f, 0.0f, 0.0f);
  // The revisit pose stays near the candidate's: poseEstimationAnalytical runs with the Cauchy
  // loss scale 0.01 (Bundle.poseEstCauchyLoss), so a start far from the optimum converges only
  // partway (the pose is adopted without a check).
  const c8::HMatrix pose_true = Pose(0.01f, 0.03f, 0.005f, 0.02f);
  const c8::HMatrix pose_stored = Pose(0.013f, 0.038f, -0.004f, 0.027f);
  std::vector<c8::HPoint3> truth;
  std::vector<RayFeature> rays_first;
  std::vector<RayFeature> rays_newest;
  for (int i = 0; i < kPoints; ++i) {
    const c8::HPoint3 x(-1.2f + 0.03f * static_cast<float>(i),
                        -0.5f + 0.03f * static_cast<float>((i * 7) % 30),
                        2.0f + 0.06f * static_cast<float>((i * 11) % 30));
    truth.push_back(x);
    const c8::HPoint2 ra = Project(pose_first, x);
    const c8::HPoint2 rb = Project(pose_true, x);
    rays_first.push_back({ra.x(), ra.y(), 1, static_cast<uint32_t>(100 + i)});
    rays_newest.push_back({rb.x(), rb.y(), 1, static_cast<uint32_t>(100 + i)});
  }

  const map::KeyframeId first = wm.AddKeyframe(MakeFrame(rays_first), 0);
  wm.SetKeyframePose(first, pose_first);
  std::vector<map::PointId> points;
  for (int i = 0; i < kPoints; ++i) {
    const map::PointId p = wm.AddPoint(wm.GetKeyframe(first).seq, truth[static_cast<size_t>(i)],
                                       map::PointSource::kTriangulated);
    wm.AddObservation(p, static_cast<uint32_t>(i), first);
    points.push_back(p);
  }
  for (int k = 1; k <= 4; ++k) {
    std::vector<RayFeature> rays;
    for (int i = 0; i < 40; ++i) {
      rays.push_back({-0.5f + 0.025f * static_cast<float>(i), 0.1f * static_cast<float>(k) - 0.2f,
                      1, static_cast<uint32_t>(1000 * k + i)});
    }
    const map::KeyframeId kf = wm.AddKeyframe(MakeFrame(rays), 5 * k);
    wm.SetKeyframePose(kf, Pose(0.0f, 0.1f * static_cast<float>(k), 0.f, 0.f));
    wm.MutableKeyframe(kf).global_descriptor.fill(0xFF);  // 256 bits from the newest keyframe
  }
  const map::KeyframeId newest = wm.AddKeyframe(MakeFrame(rays_newest), 30);
  wm.SetKeyframePose(newest, pose_stored);
  ASSERT_EQ(wm.newest().id(), newest.id());
  ASSERT_EQ(wm.NumKeyframeSlots(), 6u);

  LoopCloseState state;
  MapBuilderOutput out;
  LoopCloseReport report;
  EXPECT_TRUE(LoopClose(wm, &state, &out, &report));
  EXPECT_EQ(report.outcome, LoopCloseOutcome::kApplied);
  EXPECT_EQ(report.candidate.id(), first.id());
  EXPECT_EQ(report.candidate_distance, 0.0f);
  EXPECT_EQ(report.num_matches, static_cast<size_t>(kPoints));
  EXPECT_EQ(report.num_correspondences, static_cast<size_t>(kPoints));
  EXPECT_TRUE(out.world_transform_updated);
  EXPECT_EQ(state.ring[0], first.id());
  EXPECT_EQ(state.cursor, 1u);

  // The newest keyframe now sits at the pose poseEstimationAnalytical returns from the
  // candidate's pose for these correspondences (spec S11; adopted without a check), near the
  // revisit pose but not exactly on it (the vendored solver stops early). The correction maps the
  // old pose onto it (pose_new = pose_old * C, right multiplication), and every observation is
  // linked.
  c8::Vector<c8::ObservedPoint> ref_rays;
  c8::Vector<c8::HPoint3> ref_points;
  for (int i = 0; i < kPoints; ++i) {
    ref_rays.push_back(
        c8::ObservedPoint{Project(pose_true, truth[static_cast<size_t>(i)]), 1, 0.0f, 1.0f});
    ref_points.push_back(truth[static_cast<size_t>(i)]);
  }
  c8::HMatrix expected = pose_first;
  c8::PoseEstimationAnalyticalParams params;
  const std::deque<c8::HMatrix> last_two;
  ASSERT_TRUE(c8::poseEstimationAnalytical(ref_rays, ref_points, last_two, &expected, params));
  const c8::HMatrix& moved = wm.GetKeyframe(newest).frame.extrinsic();
  const c8::HMatrix check = pose_stored * out.world_correction;
  for (int i = 0; i < 16; ++i) {
    EXPECT_EQ(moved.data()[static_cast<size_t>(i)], expected.data()[static_cast<size_t>(i)]);
    EXPECT_NEAR(moved.data()[static_cast<size_t>(i)], pose_true.data()[static_cast<size_t>(i)], 5e-3f);
    EXPECT_NEAR(check.data()[static_cast<size_t>(i)], moved.data()[static_cast<size_t>(i)], 1e-4f);
  }
  EXPECT_EQ(wm.GetKeyframe(newest).frame.worldPointIndices().size(), static_cast<size_t>(kPoints));
  for (int i = 0; i < kPoints; ++i) {
    EXPECT_TRUE(wm.GetKeyframe(newest).frame.points()[static_cast<size_t>(i)].id() ==
                points[static_cast<size_t>(i)]);
  }

  // Same newest keyframe, second attempt: the candidate is in the ring, nothing else qualifies.
  MapBuilderOutput again;
  LoopCloseReport second;
  EXPECT_FALSE(LoopClose(wm, &state, &again, &second));
  EXPECT_EQ(second.outcome, LoopCloseOutcome::kNoCandidate);
  EXPECT_FALSE(again.world_transform_updated);
}
#else
TEST(MapBuilderUpdate, LoopCloseEnabledAbortsAsUnsupported) {
  EXPECT_DEATH(
      {
        config::ApplyParamOverrides({{"MapBuilder.loopCloseEnabled", ParamValue(true)}});
        map::WorldMaps maps;
        MapBuilder mb;
        MapBuilderOutput out;
        mb.Update(&maps, *MakeInput(0), MakeTrk(0, 7, c8::HMatrixGen::i()), 0, &out);
      },
      "loop close is unsupported");
}
#endif  // SLAM_REFACT_LOOP_CLOSE

// Two keyframes observing the same 30 world points with identical descriptors: the queued pair is
// matched along epipolar segments and every match becomes a TRIANGULATED point near the truth.
TEST(MapBuilderEpipolar, TriangulatesSharedObservationsOfAKeyframePair) {
  map::WorldMaps maps;
  map::WorldMap& wm = maps.active();
  const c8::HMatrix pose_a = Pose(0.0f, 0.0f, 0.0f, 0.0f);
  const c8::HMatrix pose_b = Pose(-0.05f, 0.3f, 0.02f, 0.05f);
  std::vector<c8::HPoint3> truth;
  std::vector<RayFeature> rays_a;
  std::vector<RayFeature> rays_b;
  for (int i = 0; i < 30; ++i) {
    const c8::HPoint3 x(-0.8f + 0.055f * static_cast<float>(i),
                        -0.4f + 0.027f * static_cast<float>((i * 7) % 30),
                        2.0f + 0.07f * static_cast<float>((i * 11) % 30));
    truth.push_back(x);
    const c8::HPoint2 ra = Project(pose_a, x);
    const c8::HPoint2 rb = Project(pose_b, x);
    rays_a.push_back({ra.x(), ra.y(), 1, static_cast<uint32_t>(100 + i)});
    rays_b.push_back({rb.x(), rb.y(), 1, static_cast<uint32_t>(100 + i)});
  }
  const map::KeyframeId a = wm.AddKeyframe(MakeFrame(rays_a), 0);
  wm.SetKeyframePose(a, pose_a);
  const map::KeyframeId b = wm.AddKeyframe(MakeFrame(rays_b), 5);
  wm.SetKeyframePose(b, pose_b);
  std::vector<KfPair> queue = {{a, b}};
  EpipolarMatcher(wm, &queue, false);
  EXPECT_TRUE(queue.empty());
  // The 1123 midpoint of every pair is the true point; the stored position is then the 1127
  // refinement (certainty-weighted epipolar depth of each observation, WP2 + public FramePoint),
  // whose float depth from a short baseline differs by up to ~2% at the far points.
  const RayPairPrework ab = MakeRayPairPrework(pose_a, pose_b);
  const RayPairPrework ba = MakeRayPairPrework(pose_b, pose_a);
  int matched = 0;
  int refined_close = 0;
  for (int i = 0; i < 30; ++i) {
    const size_t k = static_cast<size_t>(i);
    const c8::HPoint3& t = truth[k];
    const c8::HPoint3 ra(rays_a[k].x, rays_a[k].y, 1.f);
    const c8::HPoint3 rb(rays_b[k].x, rays_b[k].y, 1.f);
    const c8::HPoint3 o1 = RayPairPoint(ra, rb, ab);
    const c8::HPoint3 o2 = RayPairPoint(rb, ra, ba);
    const float mx = (o1.x() + o2.x()) * 0.5f - t.x();
    const float my = (o1.y() + o2.y()) * 0.5f - t.y();
    const float mz = (o1.z() + o2.z()) * 0.5f - t.z();
    EXPECT_LT(std::sqrt(mx * mx + my * my + mz * mz), 1e-4f * t.z()) << "midpoint " << i;

    const map::PointId pa = wm.GetKeyframe(a).frame.points()[k].id();
    const map::PointId pb = wm.GetKeyframe(b).frame.points()[k].id();
    if (pa == map::PointId::nullId() || !(pa == pb)) {
      continue;
    }
    const map::WorldPoint& p = wm.Point(pa);
    EXPECT_EQ(p.source, map::PointSource::kTriangulated);
    EXPECT_NE(p.state, map::PointState::kDead);
    ++matched;
    const float dx = p.position.x() - t.x();
    const float dy = p.position.y() - t.y();
    const float dz = p.position.z() - t.z();
    const float d = std::sqrt(dx * dx + dy * dy + dz * dz);
    EXPECT_LT(d, 0.03f * t.z()) << "refined point " << i;
    refined_close += d < 0.01f * t.z() ? 1 : 0;
  }
  std::printf("[epipolar] matched %d/30, refined within 1%% of depth %d\n", matched,
              refined_close);
  EXPECT_GE(matched, 25);
}

TEST(MapBuilderEpipolar, InvalidPairsArePoppedWithoutCounting) {
  map::WorldMaps maps;
  map::WorldMap& wm = maps.active();
  std::vector<KfPair> queue(5, KfPair{map::KeyframeId(7, 99), map::KeyframeId(8, 98)});
  EpipolarMatcher(wm, &queue, false);
  EXPECT_TRUE(queue.empty());
}

}  // namespace
}  // namespace slam_refact::map_builder

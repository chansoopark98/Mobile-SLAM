// MB-UNIT (architecture.md §6.6.3): the S3 §8.2 boundary table, fixed before any engine result.
// Rows: keyframe need, delay/cooldown, relocalization hold, gyro, reset, ground depth, prune
// redundancy, size cap, pair queue. "reset => keyframe", the keyframe-interval / 64-keyframe
// invariants and the dead keys are in update_test.cc.

#include <cmath>
#include <cstdint>
#include <vector>

#include "c8/hmatrix.h"
#include "c8/hpoint.h"
#include "config/params.h"
#include "config/settings.h"
#include "gtest/gtest.h"
#include "map/world_map.h"
#include "map_builder/map_builder.h"
#include "map_builder/map_builder_internal.h"
#include "map_builder_test_util.h"
#include "reality/engine/geometry/epipolar.h"

namespace slam_refact::map_builder {
namespace {

using testing::MakeFrame;
using testing::Pose;
using testing::RayFeature;

// ---- keyframe need (S3 §5.3) ----

TEST(MapBuilderPolicy, NeedKeyframeTrackedBoundary119And120) {
  EXPECT_TRUE(NeedKeyframe(119, 0));
  EXPECT_FALSE(NeedKeyframe(120, 0));
}

TEST(MapBuilderPolicy, NeedKeyframeRatioBoundaryAt200Tracked) {
  // S3 §8.2 lists (200, 60) -> true and (200, 59) -> false, i.e. the exact-arithmetic boundary.
  // The pre-registered criterion is "bool equal to §5.3", whose comparison is in float
  // (0.3f * float(tracked) <= float(n_new), 936.c:892 VERIFIED): 0.3f * 200 = 60.0000024 rounds
  // to 60.000004f > 60, so the binary rejects 60 and the first accepted n_new is 61.
  const float lhs = 0.3f * static_cast<float>(200);
  EXPECT_GT(lhs, 60.0f);
  EXPECT_EQ(NeedKeyframe(200, 60), lhs <= 60.0f);
  EXPECT_FALSE(NeedKeyframe(200, 60));
  EXPECT_FALSE(NeedKeyframe(200, 59));
  EXPECT_TRUE(NeedKeyframe(200, 61));
  // Exact boundary where float arithmetic is exact: 0.3f * 120 rounds to 36.000004f too, so use
  // tracked = 400 (0.3f * 400 = 120.0000048 -> 120.00001f): 120 rejected, 121 accepted.
  EXPECT_FALSE(NeedKeyframe(400, 120));
  EXPECT_TRUE(NeedKeyframe(400, 121));
}

TEST(MapBuilderPolicy, NeedKeyframeMinimumNewPoints19And20) {
  // With the default ratio 0.3 the minimum (20) never binds at tracked >= 120 (0.3 * 120 = 36), so
  // the ratio is lowered to isolate the keyframeMinimumNewPoints comparison.
  {
    const testing::ScopedParamOverrides overrides(
        {{"MapBuilder.keyframeCountVariability", ParamValue(0.1f)}});
    EXPECT_FALSE(NeedKeyframe(120, 19));
    EXPECT_TRUE(NeedKeyframe(120, 20));
  }
  EXPECT_FALSE(NeedKeyframe(120, 20));
}

// ---- keyframe decision (936.c:770-958) ----

KeyframeDecisionInputs Ready(const std::vector<TrackingSensorEvent>* imu) {
  KeyframeDecisionInputs in;
  in.map_reset = false;
  in.state = 1;
  in.imu = imu;
  in.frame_count = 100;
  in.last_state_change_frame = 50;  // since_change 50
  in.last_reset_frame = 0;          // reset 100 frames ago, since_change >= 30: no hold
  in.last_keyframe_frame = 90;      // 10 frames since the last keyframe
  in.need = true;
  return in;
}

TEST(MapBuilderPolicy, KeyframeDelayFirstAllowedAt3) {
  const std::vector<TrackingSensorEvent> imu;
  KeyframeDecisionInputs in = Ready(&imu);
  in.last_reset_frame = 99;  // keep the relocalization hold out of the picture (reset < 30 ago)
  in.last_state_change_frame = in.frame_count - 2;
  EXPECT_FALSE(DecideKeyframe(in));
  in.last_state_change_frame = in.frame_count - 3;
  EXPECT_TRUE(DecideKeyframe(in));
}

TEST(MapBuilderPolicy, KeyframeCooldownFirstAllowedAt5) {
  const std::vector<TrackingSensorEvent> imu;
  KeyframeDecisionInputs in = Ready(&imu);
  in.last_keyframe_frame = in.frame_count - 4;
  EXPECT_FALSE(DecideKeyframe(in));
  in.last_keyframe_frame = in.frame_count - 5;
  EXPECT_TRUE(DecideKeyframe(in));
}

TEST(MapBuilderPolicy, KeyframeRequiresNeedAndTrackingState) {
  const std::vector<TrackingSensorEvent> imu;
  KeyframeDecisionInputs in = Ready(&imu);
  in.need = false;
  EXPECT_FALSE(DecideKeyframe(in));
  in = Ready(&imu);
  in.state = 0;
  EXPECT_FALSE(DecideKeyframe(in));
}

TEST(MapBuilderPolicy, RelocalizationHoldOnlyAt30And29) {
  const int64_t fc = 1000;
  for (int64_t since_reset : {29, 30}) {
    for (int64_t since_change : {29, 30}) {
      const bool hold = RelocalizationHold(fc, fc - since_reset, fc - since_change);
      EXPECT_EQ(hold, since_reset == 30 && since_change == 29)
          << "since_reset " << since_reset << " since_change " << since_change;
    }
  }
  // Through the decision: the hold blocks a keyframe that is otherwise due.
  const std::vector<TrackingSensorEvent> imu;
  KeyframeDecisionInputs in = Ready(&imu);
  in.frame_count = fc;
  in.last_reset_frame = fc - 30;
  in.last_state_change_frame = fc - 29;
  in.last_keyframe_frame = fc - 10;
  EXPECT_FALSE(DecideKeyframe(in));
  in.last_state_change_frame = fc - 30;
  EXPECT_TRUE(DecideKeyframe(in));
}

TrackingSensorEvent Gyro(float x, float y, float z) {
  TrackingSensorEvent e;
  e.kind = ImuEvent::Kind::kGyroscope;
  e.x = x;
  e.y = y;
  e.z = z;
  return e;
}

TEST(MapBuilderPolicy, GyroGate80PassesAnd80p0001Blocks) {
  EXPECT_FALSE(AnyGyroAbove({Gyro(80.0f, 0.f, 0.f)}, 80.0f));
  EXPECT_TRUE(AnyGyroAbove({Gyro(80.0001f, 0.f, 0.f)}, 80.0f));
  EXPECT_FALSE(AnyGyroAbove({Gyro(0.f, 0.f, -80.0f)}, 80.0f));
  EXPECT_TRUE(AnyGyroAbove({Gyro(0.f, -80.0001f, 0.f)}, 80.0f));
  // Only gyroscope events count.
  TrackingSensorEvent accel = Gyro(500.f, 0.f, 0.f);
  accel.kind = ImuEvent::Kind::kAccelerometer;
  EXPECT_FALSE(AnyGyroAbove({accel}, 80.0f));

  std::vector<TrackingSensorEvent> calm = {Gyro(80.0f, 0.f, 0.f)};
  std::vector<TrackingSensorEvent> fast = {Gyro(80.0001f, 0.f, 0.f)};
  EXPECT_TRUE(DecideKeyframe(Ready(&calm)));
  EXPECT_FALSE(DecideKeyframe(Ready(&fast)));
  // The reset frame skips the gyro gate (936.c:904).
  KeyframeDecisionInputs reset = Ready(&fast);
  reset.map_reset = true;
  reset.state = 0;
  EXPECT_TRUE(DecideKeyframe(reset));
}

// ---- reset (936.c:696-731) ----

ResetInputs LostWithCandidates() {
  ResetInputs in;
  in.state = 0;
  in.frame_tracker_ok = true;
  in.num_candidates = 20;
  in.frame_count = 100;
  in.last_state_change_frame = 90;  // since_change 10 > 5
  in.keyframe_slots = 5;
  in.consecutive_frame_tracker_ok = 7;
  return in;
}

TEST(MapBuilderPolicy, ResetSmallMapNeeds7Successes) {
  ResetInputs in = LostWithCandidates();
  in.keyframe_slots = 5;
  in.consecutive_frame_tracker_ok = 6;
  EXPECT_FALSE(ShouldResetMap(in));
  in.consecutive_frame_tracker_ok = 7;
  EXPECT_TRUE(ShouldResetMap(in));
}

TEST(MapBuilderPolicy, ResetLargeMapNeeds20Successes) {
  ResetInputs in = LostWithCandidates();
  in.keyframe_slots = 6;
  in.consecutive_frame_tracker_ok = 19;
  EXPECT_FALSE(ShouldResetMap(in));
  in.consecutive_frame_tracker_ok = 20;
  EXPECT_TRUE(ShouldResetMap(in));
}

TEST(MapBuilderPolicy, ResetNeeds20Candidates) {
  ResetInputs in = LostWithCandidates();
  in.num_candidates = 19;
  EXPECT_FALSE(ShouldResetMap(in));
  in.num_candidates = 20;
  EXPECT_TRUE(ShouldResetMap(in));
}

TEST(MapBuilderPolicy, ResetOfNonEmptyMapNeedsMoreThan5FramesSinceStateChange) {
  ResetInputs in = LostWithCandidates();
  in.last_state_change_frame = in.frame_count - 5;
  EXPECT_FALSE(ShouldResetMap(in));  // map not empty
  in.last_state_change_frame = in.frame_count - 6;
  EXPECT_TRUE(ShouldResetMap(in));
  // An empty map (no keyframe slot at all) resets right after the state change.
  in.last_state_change_frame = in.frame_count - 5;
  in.keyframe_slots = 0;
  EXPECT_TRUE(ShouldResetMap(in));
}

TEST(MapBuilderPolicy, ResetRequiresLostStateAndFrameTracking) {
  ResetInputs in = LostWithCandidates();
  in.state = 1;
  EXPECT_FALSE(ShouldResetMap(in));
  in = LostWithCandidates();
  in.frame_tracker_ok = false;
  EXPECT_FALSE(ShouldResetMap(in));
}

// ---- ground depth (S3 §5.1) vs the public groundPointTriangulationPrework ----

TEST(MapBuilderPolicy, GroundPointMatchesPublicGroundPrework) {
  int checked = 0;
  for (float yaw : {0.0f, 0.4f, -0.9f}) {
    for (float height : {0.0f, 0.5f, 1.3f}) {
      // Camera looking down 30 degrees (rotation about x) after the yaw.
      const float c = std::cos(-0.52f);
      const float s = std::sin(-0.52f);
      const c8::HMatrix pitch({1.f, 0.f, 0.f, 0.f}, {0.f, c, -s, 0.f}, {0.f, s, c, 0.f},
                              {0.f, 0.f, 0.f, 1.f}, {1.f, 0.f, 0.f, 0.f}, {0.f, c, s, 0.f},
                              {0.f, -s, c, 0.f}, {0.f, 0.f, 0.f, 1.f});
      const c8::HMatrix pose = Pose(yaw, 0.3f, height, -0.2f) * pitch;
      const GroundPlane plane = GroundPlaneInCamera(pose);
      const c8::HVector3 n = c8::groundPointTriangulationPrework(pose);
      for (float x : {-0.4f, 0.0f, 0.35f}) {
        for (float y : {-0.3f, 0.1f, 0.3f}) {
          const c8::HPoint3 ours = GroundPointInWorld(plane, pose, x, y);
          const float z = 1.0f / n.dot(c8::HVector3(x, y, 1.0f));
          const c8::HPoint3 pub = pose * c8::HPoint3(x * z, y * z, z);
          for (int i = 0; i < 3; ++i) {
            const float a = ours.raw()[i] / ours.raw()[3];
            const float b = pub.raw()[i] / pub.raw()[3];
            EXPECT_LE(std::fabs(a - b), 1e-5f * std::fmax(1.0f, std::fabs(b)))
                << "yaw " << yaw << " h " << height << " ray " << x << "," << y;
          }
          // On the plane y = -1.
          EXPECT_NEAR(ours.y(), -1.0f, 1e-4f * std::fmax(1.0f, std::fabs(ours.z())));
          ++checked;
        }
      }
    }
  }
  EXPECT_EQ(checked, 81);
}

// f936 wat association of the ground intersection (precision lane, 2026-10-16): the numerator is
// n.z*p0z + (n.x*p0x + p0y*n.y) and the denominator n.z*rz + (n.x*rx + n.y*ry); Ghidra prints both
// as flat left-to-right sums. With n.z*p0z = 1 and the two other products 2^-24 each, the flat
// order rounds to 1 and the wat order to 1 + 2^-23.
TEST(MapBuilderPolicy, GroundPointUsesWatAssociation) {
  GroundPlane g;
  g.nx = 0x1p-24f;
  g.ny = 0x1p-24f;
  g.nz = 1.0f;
  g.p0x = 1.0f;
  g.p0y = 1.0f;
  g.p0z = 1.0f;
  const c8::HPoint3 p = GroundPointInWorld(g, c8::HMatrixGen::i(), 0.0f, 0.0f);
  EXPECT_EQ(p.z(), 1.0f + 0x1p-23f);
  EXPECT_EQ(p.x(), 0.0f);
  EXPECT_EQ(p.y(), 0.0f);
}

// ---- prune redundancy and size cap (936.c:2869-3286) ----

TEST(MapBuilderPolicy, PruneRedundancyStrictAt70Percent) {
  EXPECT_FALSE(IsKeyframeRedundant(10, 7, 0.7f));  // 7 kept
  EXPECT_TRUE(IsKeyframeRedundant(10, 8, 0.7f));   // 8 removed
  EXPECT_FALSE(IsKeyframeRedundant(0, 0, 0.7f));
}

// Map of keyframes 0..3; keyframe 0 has 10 points; `redundant` of them are also observed by
// keyframes 1 and 2 (3 observers incl. itself), the others only by keyframe 3 (2 observers).
// Keyframe 3 is the newest and never a candidate.
void BuildRedundancyMap(map::WorldMap* wm, int redundant, std::vector<map::KeyframeId>* kfs) {
  std::vector<RayFeature> rays;
  for (int i = 0; i < 10; ++i) {
    rays.push_back({-0.3f + 0.06f * static_cast<float>(i), 0.05f * static_cast<float>(i % 3), 1,
                    static_cast<uint32_t>(i + 1)});
  }
  for (int k = 0; k < 4; ++k) {
    kfs->push_back(wm->AddKeyframe(MakeFrame(rays), k));
    wm->SetKeyframePose(kfs->back(), Pose(0.0f, 0.1f * static_cast<float>(k), 0.f, 0.f));
  }
  for (int i = 0; i < 10; ++i) {
    const map::PointId p = wm->AddPoint(0, c8::HPoint3(0.f, 0.f, 3.f),
                                        map::PointSource::kTriangulated);
    wm->AddObservation(p, static_cast<uint32_t>(i), (*kfs)[0]);
    if (i < redundant) {
      wm->AddObservation(p, static_cast<uint32_t>(i), (*kfs)[1]);
      wm->AddObservation(p, static_cast<uint32_t>(i), (*kfs)[2]);
    } else {
      wm->AddObservation(p, static_cast<uint32_t>(i), (*kfs)[3]);
    }
  }
}

bool Live(const map::WorldMap& wm, map::KeyframeId id) {
  for (map::KeyframeId k : wm.AllKeyframesByCreation()) {
    if (k == id) {
      return true;
    }
  }
  return false;
}

TEST(MapBuilderPolicy, PruneKeyframesKeepsAt7RedundantOf10) {
  map::WorldMaps maps;
  std::vector<map::KeyframeId> kfs;
  BuildRedundancyMap(&maps.active(), 7, &kfs);
  PruneKeyframes(maps.active(), {});
  EXPECT_TRUE(Live(maps.active(), kfs[0]));
}

TEST(MapBuilderPolicy, PruneKeyframesRemovesAt8RedundantOf10) {
  map::WorldMaps maps;
  std::vector<map::KeyframeId> kfs;
  BuildRedundancyMap(&maps.active(), 8, &kfs);
  PruneKeyframes(maps.active(), {});
  EXPECT_FALSE(Live(maps.active(), kfs[0]));
  EXPECT_TRUE(Live(maps.active(), kfs[3]));  // newest is never a candidate
}

TEST(MapBuilderPolicy, PruneKeyframesHonorsProtectedSet) {
  map::WorldMaps maps;
  std::vector<map::KeyframeId> kfs;
  BuildRedundancyMap(&maps.active(), 8, &kfs);
  PruneKeyframes(maps.active(), {kfs[0]});
  EXPECT_TRUE(Live(maps.active(), kfs[0]));
}

TEST(MapBuilderPolicy, PruneKeyframesCap65To64RemovesOldest) {
  map::WorldMaps maps;
  map::WorldMap& wm = maps.active();
  std::vector<map::KeyframeId> kfs;
  for (int k = 0; k < 65; ++k) {
    kfs.push_back(wm.AddKeyframe(MakeFrame({{0.f, 0.f, 1, 1}}), k));
  }
  EXPECT_EQ(wm.NumKeyframes(), 65);
  PruneKeyframes(wm, {});
  EXPECT_EQ(wm.NumKeyframes(), 64);
  EXPECT_FALSE(Live(wm, kfs[0]));
  for (int k = 1; k < 65; ++k) {
    EXPECT_TRUE(Live(wm, kfs[static_cast<size_t>(k)]));
  }
  PruneKeyframes(wm, {});
  EXPECT_EQ(wm.NumKeyframes(), 64);
}

TEST(MapBuilderPolicy, PruneKeyframesCapSkipsProtectedOldest) {
  map::WorldMaps maps;
  map::WorldMap& wm = maps.active();
  std::vector<map::KeyframeId> kfs;
  for (int k = 0; k < 65; ++k) {
    kfs.push_back(wm.AddKeyframe(MakeFrame({{0.f, 0.f, 1, 1}}), k));
  }
  PruneKeyframes(wm, {kfs[0]});
  EXPECT_TRUE(Live(wm, kfs[0]));
  EXPECT_FALSE(Live(wm, kfs[1]));
  EXPECT_EQ(wm.NumKeyframes(), 64);
}

// ---- pair queue (936.c:2678-2705) ----

TEST(MapBuilderPolicy, PairQueue91DropsOldest33) {
  std::vector<KfPair> q;
  for (uint32_t i = 0; i < 91; ++i) {
    q.push_back({map::KeyframeId(i, i + 1), map::KeyframeId(0, 1000)});
  }
  TrimPairQueue(&q, 100);
  ASSERT_EQ(q.size(), 58u);
  EXPECT_EQ(q.front().a.id(), 34u);  // element 33 (id 34) is now first
  EXPECT_EQ(q.back().a.id(), 91u);
}

TEST(MapBuilderPolicy, PairQueue90IsKept) {
  std::vector<KfPair> q(90);
  TrimPairQueue(&q, 100);
  EXPECT_EQ(q.size(), 90u);
}

}  // namespace
}  // namespace slam_refact::map_builder

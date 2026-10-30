// MapTracker::Track (926) / ApplyMapBuilderOutput (664) tests: map-init entry, tracking, the
// TRACKING -> LOST transition (pose = replaceYaw(anchor)), LOST -> relocalized, Q5, path A abort,
// event application, and MT-P2u: the public status state machine (VioTracker 784, S2 §6.3) fed with
// synthetic ok sequences and with MapTracker outputs, against hand-derived status timelines.

#include <cmath>
#include <cfloat>
#include <cstdint>
#include <cstring>
#include <vector>

#include <gtest/gtest.h>

#include "c8/geometry/egomotion.h"
#include "c8/hmatrix.h"
#include "c8/hpoint.h"
#include "compat/random_numbers.h"
#include "config/params.h"
#include "frame_frame_tracker/frame_frame_tracker_internal.h"
#include "map/world_map.h"
#include "map_tracker/map_tracker.h"
#include "map_tracker/map_tracker_internal.h"
#include "scene_util.h"
#include "vio_tracker/vio_tracker.h"

namespace slam_refact::map_tracker {
namespace {

using testing::BuildScene;
using testing::GridPoints;
using testing::MakeInput;
using testing::RandomDescriptors;
using testing::Scene;

constexpr int64_t kFrameNs = 50000000;  // 20 fps

bool SameBits(const c8::HMatrix& a, const c8::HMatrix& b) {
  return std::memcmp(a.data().data(), b.data().data(), sizeof(float) * 16) == 0 &&
         std::memcmp(a.inverseData().data(), b.inverseData().data(), sizeof(float) * 16) == 0;
}

class MapTrackerTest : public ::testing::Test {
 protected:
  void SetUp() override {
    world_ = GridPoints(12, 9);
    desc_ = RandomDescriptors(world_.size(), 21);
    BuildScene(&scene_, world_, desc_, c8::HMatrixGen::i(), map::PointSource::kDepth);
  }

  MapTrackerOutput TrackAt(const c8::HMatrix& truth, bool matching) {
    const auto& d = matching ? desc_ : RandomDescriptors(world_.size(), 1000 + frame_);
    TrackerInput in = MakeInput(world_, d, truth, frame_ * kFrameNs);
    FrameState pred;
    pred.pose = tracker_.pose();
    pred.time_ns = frame_ * kFrameNs;
    pred.xr_device_rotation = c8::rotation(truth);
    MapTrackerOutput out;
    tracker_.Track(scene_.map, &in, pred, &out, &rng_);
    ++frame_;
    return out;
  }

  std::vector<c8::HPoint3> world_;
  std::vector<testing::Descriptor> desc_;
  Scene scene_;
  MapTracker tracker_;
  compat::RandomNumbers rng_{compat::kEngineRngSeed};
  int64_t frame_ = 0;
};

TEST_F(MapTrackerTest, MapInitEventEntersTrackingAndTracksMotion) {
  MapBuilderOutput ev;
  ev.keyframe_created = true;
  tracker_.ApplyMapBuilderOutput(ev);
  MapTrackerOutput out = TrackAt(c8::HMatrixGen::i(), true);
  EXPECT_EQ(out.state, 1);
  EXPECT_LT(testing::MaxTranslationError(out.pose, c8::HMatrixGen::i()), 0.005f);
  EXPECT_FALSE(out.tracked.empty());
  for (int k = 1; k <= 6; ++k) {
    const c8::HMatrix truth = c8::HMatrixGen::translation(0.01f * k, 0.0f, 0.005f * k);
    out = TrackAt(truth, true);
    EXPECT_EQ(out.state, 1) << k;
    EXPECT_LT(testing::MaxTranslationError(out.pose, truth), 0.005f) << k;
    EXPECT_LT(testing::MaxRotationError(out.pose, truth), 0.002f) << k;
  }
}

TEST_F(MapTrackerTest, LostFrameKeepsAnchorPositionThenRelocalizes) {
  MapBuilderOutput ev;
  ev.keyframe_created = true;
  tracker_.ApplyMapBuilderOutput(ev);
  MapTrackerOutput out = TrackAt(c8::HMatrixGen::i(), true);
  out = TrackAt(c8::HMatrixGen::translation(0.01f, 0.f, 0.f), true);
  ASSERT_EQ(out.state, 1);
  const c8::HMatrix anchor = tracker_.pose();

  // Unmatched frame: both passes return FLT_MAX -> LOST, pose = replaceYaw(anchor, q).
  const c8::HMatrix truth_lost = c8::HMatrixGen::translation(0.02f, 0.f, 0.f);
  out = TrackAt(truth_lost, false);
  EXPECT_EQ(out.state, 0);
  EXPECT_TRUE(SameBits(out.pose, ReplaceYaw(anchor, c8::rotation(truth_lost))));
  const MapTrackerAccess::Impl& impl = MapTrackerAccess::impl(tracker_);
  EXPECT_EQ(impl.lost_residual, FLT_MAX);
  EXPECT_EQ(impl.last_state_change_frame, impl.frame_count);

  // Matching frame while LOST: relocalize (robustP2P) + trackPoints residual <= 0.05 -> TRACKING.
  const c8::HMatrix truth = c8::HMatrixGen::translation(0.06f, 0.02f, 0.04f);
  out = TrackAt(truth, true);
  EXPECT_TRUE(out.lost_reloc_attempted);
  EXPECT_EQ(out.state, 1);
  EXPECT_LT(testing::MaxTranslationError(out.pose, truth), 0.005f);
}

TEST_F(MapTrackerTest, RelocalizesFromInitialLostState) {
  // No map-init event: the first frame takes the LOST path and relocalizes against the map.
  const c8::HMatrix truth = c8::HMatrixGen::translation(0.03f, 0.f, 0.02f);
  const MapTrackerOutput out = TrackAt(truth, true);
  EXPECT_TRUE(out.lost_reloc_attempted);
  EXPECT_EQ(out.state, 1);
  EXPECT_LT(testing::MaxTranslationError(out.pose, truth), 0.005f);
}

// S2 review MEDIUM-1 at the MapTracker level: each tracker relocalizes with its own robustP2P
// scratch, so a second tracker with other intrinsics builds its confidence grid from its own frame
// bounds (with the former process-wide static it would reuse the first tracker's grid).
TEST(MapTrackerScratch, EachTrackerBuildsItsOwnP2PGrid) {
  const std::vector<c8::HPoint3> world = GridPoints(12, 9);
  const auto desc = RandomDescriptors(world.size(), 21);
  Scene scene;
  BuildScene(&scene, world, desc, c8::HMatrixGen::i(), map::PointSource::kDepth);
  c8_PixelPinholeCameraModel wide = testing::SceneIntrinsics();
  wide.focalLengthHorizontal = 130.0f;
  wide.focalLengthVertical = 130.0f;
  const c8::HMatrix truth = c8::HMatrixGen::translation(0.03f, 0.f, 0.02f);
  MapTracker first;
  MapTracker second;
  for (int k = 0; k < 2; ++k) {
    const c8_PixelPinholeCameraModel intrinsic = k == 0 ? testing::SceneIntrinsics() : wide;
    MapTracker& tracker = k == 0 ? first : second;
    TrackerInput in(intrinsic, c8::rotation(truth), 0);
    testing::AddObservations(&in.frame_points, world, desc, truth, nullptr);
    FrameState pred;
    pred.pose = tracker.pose();
    pred.xr_device_rotation = c8::rotation(truth);
    MapTrackerOutput out;
    compat::RandomNumbers rng(compat::kEngineRngSeed);
    tracker.Track(scene.map, &in, pred, &out, &rng);  // initial LOST -> relocalize (1274)
    ASSERT_TRUE(out.lost_reloc_attempted);
    const std::vector<c8::HPoint2>& grid = MapTrackerAccess::impl(tracker).p2p_scratch.confidenceGrid;
    ASSERT_FALSE(grid.empty()) << k;
    c8::HPoint2 ll;
    c8::HPoint2 ur;
    c8::frameBounds(intrinsic, &ll, &ur);
    const float step = (ur.x() - ll.x()) / 25;  // uniformSampleGrid, GRID_X_STEPS = 25
    EXPECT_EQ(grid[0].x(), ll.x() + 0.5f * step) << k;
    EXPECT_EQ(grid[0].y(), ll.y() + 0.5f * step) << k;
  }
  EXPECT_NE(MapTrackerAccess::impl(first).p2p_scratch.confidenceGrid[0].x(),
            MapTrackerAccess::impl(second).p2p_scratch.confidenceGrid[0].x());
}

TEST_F(MapTrackerTest, Q5SecondPassPoseIsApplied) {
  MapBuilderOutput ev;
  ev.keyframe_created = true;
  tracker_.ApplyMapBuilderOutput(ev);
  TrackAt(c8::HMatrixGen::i(), true);
  const MapTrackerOutput out = TrackAt(c8::HMatrixGen::translation(0.01f, 0.f, 0.f), true);
  ASSERT_EQ(out.state, 1);
  const MapTrackerAccess::Impl& impl = MapTrackerAccess::impl(tracker_);
  // 1260 copies the second-pass pose into pose_ (and moves its matched ids to last_tracked_ids_).
  EXPECT_TRUE(SameBits(impl.pose, impl.track_result.pose));
  EXPECT_TRUE(SameBits(out.pose, impl.track_result.pose));
  EXPECT_FALSE(impl.last_tracked_ids.empty());
  EXPECT_TRUE(impl.track_result.matched_ids.empty());
}

TEST_F(MapTrackerTest, PathADepthKeyframeRelocalizationAborts) {
  MapBuilderOutput ev;
  ev.reloc_kf_request = true;
  ev.reloc_kf_id = scene_.kf;
  tracker_.ApplyMapBuilderOutput(ev);
  EXPECT_DEATH(TrackAt(c8::HMatrixGen::i(), true), "depth-keyframe relocalization");
}

TEST(MapTrackerEvents, ApplyMapBuilderOutput664) {
  MapTracker t;
  MapTrackerAccess::Impl& impl = MapTrackerAccess::impl(t);
  impl.pose = c8::HMatrixGen::translation(1.f, 2.f, 3.f);
  impl.last_tracked_ids = {map::PointId(0, 7)};
  t.frame_tracker().set_height(3.0f);

  MapBuilderOutput add;
  add.new_points = {map::PointId(1, 8), map::PointId(2, 9)};
  t.ApplyMapBuilderOutput(add);
  EXPECT_EQ(impl.last_tracked_ids.size(), 3u);

  MapBuilderOutput reset;
  reset.map_reset = true;
  t.ApplyMapBuilderOutput(reset);
  EXPECT_EQ(t.frame_tracker().height(), 1.0f);

  t.frame_tracker().set_height(3.0f);
  MapBuilderOutput corr;
  corr.world_transform_updated = true;
  corr.world_correction = c8::HMatrixGen::translation(0.5f, 0.f, 0.f);
  corr.new_points = {map::PointId(3, 10)};  // ignored when the world transform is updated
  const c8::HMatrix p = t.ApplyMapBuilderOutput(corr);
  EXPECT_EQ(p(0, 3), 1.5f);
  EXPECT_TRUE(SameBits(impl.frame_start_pose, impl.pose));
  EXPECT_TRUE(impl.last_tracked_ids.empty());
  EXPECT_EQ(t.frame_tracker().height(), 1.0f);

  MapBuilderOutput kf;
  kf.keyframe_created = true;
  t.ApplyMapBuilderOutput(kf);
  EXPECT_TRUE(impl.map_initialized_pending);
}

// US-025: 664 right-multiplies the loop close correction (pose_ = pose_ * C, spec/12-loop-close.md
// §5.2). A rotated pose and a translation correction tell the two orders apart.
TEST(MapTrackerEvents, WorldCorrectionIsRightMultiplied) {
  MapTracker t;
  MapTrackerAccess::Impl& impl = MapTrackerAccess::impl(t);
  impl.pose = c8::HMatrixGen::translation(1.f, 2.f, 3.f) * c8::HMatrixGen::yDegrees(90.f);
  const c8::HMatrix corr = c8::HMatrixGen::translation(0.5f, 0.f, 0.f);
  const c8::HMatrix right = impl.pose * corr;
  const c8::HMatrix left = corr * impl.pose;
  ASSERT_GT(std::fabs(right(2, 3) - left(2, 3)), 0.1f);  // the orders differ for this input

  MapBuilderOutput o;
  o.world_transform_updated = true;
  o.world_correction = corr;
  const c8::HMatrix p = t.ApplyMapBuilderOutput(o);
  for (int r = 0; r < 4; ++r) {
    for (int c = 0; c < 4; ++c) {
      EXPECT_NEAR(p(r, c), right(r, c), 1e-6f);
    }
  }
}

TEST(MapTrackerCtor, InitialState) {
  MapTracker t;
  const MapTrackerAccess::Impl& impl = MapTrackerAccess::impl(t);
  EXPECT_EQ(impl.state, 0);
  EXPECT_EQ(impl.frame_count, -1);
  EXPECT_EQ(impl.last_state_change_frame, 0);
  EXPECT_FALSE(impl.frame_tracker_ok);
  EXPECT_EQ(t.frame_tracker().height(), 1.0f);
}

// ---- MT-P2u ----

std::vector<TrackingStatus> RunStatus(const std::vector<bool>& ok) {
  vio_tracker::LossStateMachine sm;
  std::vector<TrackingStatus> out;
  for (size_t i = 0; i < ok.size(); ++i) {
    sm.Update(ok[i], static_cast<int64_t>(i) * kFrameNs);
    out.push_back(sm.status());
  }
  return out;
}

// Expected: LIMITED exactly on [begin, end) frames.
void ExpectLimited(const std::vector<TrackingStatus>& s, size_t begin, size_t end) {
  for (size_t i = 0; i < s.size(); ++i) {
    const bool limited = i >= begin && i < end;
    EXPECT_EQ(s[i], limited ? TrackingStatus::kLimited : TrackingStatus::kNormal) << "frame " << i;
  }
}

TEST(MapTrackerStatus, MtP2uStep) {
  // ok 0-39, fail 40-119, ok 120-239 (20 fps).
  // 40: lost -> s1 (t0 = 40). 80: 2 s, 40/40 lost -> s2 (LIMITED). 120: first ok -> s3.
  // 160: 2 s, 0/40 lost -> s0 (NORMAL).
  std::vector<bool> ok(240, true);
  for (int i = 40; i < 120; ++i) ok[i] = false;
  ExpectLimited(RunStatus(ok), 80, 160);
}

TEST(MapTrackerStatus, MtP2uIntermittentTenPercent) {
  // Fail at 5, 15, 25, ...: every 2 s window has 4/40 = 0.1 < 0.2 lost -> always NORMAL.
  std::vector<bool> ok(400, true);
  for (int i = 5; i < 400; i += 10) ok[i] = false;
  ExpectLimited(RunStatus(ok), 0, 0);
}

TEST(MapTrackerStatus, MtP2uIntermittentTwentyFivePercent) {
  // Fail at 0, 4, 8, ...: 0 -> s1; 40: 10/40 = 0.25 -> s2; 41 ok -> s3; 81: 10/40 -> s2; 82 -> s3
  // ... LIMITED from frame 40 on.
  std::vector<bool> ok(400, true);
  for (int i = 0; i < 400; i += 4) ok[i] = false;
  ExpectLimited(RunStatus(ok), 40, 400);
}

TEST(MapTrackerStatus, MtP2uAllFail) {
  std::vector<bool> ok(200, false);
  ExpectLimited(RunStatus(ok), 40, 200);
}

TEST_F(MapTrackerTest, MtP2uMapTrackerDrivenStatus) {
  // ok = state == TRACKING || frameTrackerOk (S2 §6.3 with the world-SLAM mode). Frames 0-29
  // track, 30-89 see no map point, 90-219 track again. Frame 30 is still ok: it is processed on
  // the inertial path (TRACKING for > initialFrameDelay frames), which sets frameTrackerOk = 1
  // (S2 §6.2), and only then turns LOST. Frames 31-89 are LOST and take the FFT path, which fails
  // on unrelated descriptors. Expected: 31 -> s1, 71 (2 s, all lost) -> LIMITED, 90 first ok ->
  // s3, 130 (2 s, 0 lost) -> NORMAL.
  MapBuilderOutput ev;
  ev.keyframe_created = true;
  tracker_.ApplyMapBuilderOutput(ev);
  vio_tracker::LossStateMachine sm;
  std::vector<TrackingStatus> status;
  for (int i = 0; i < 220; ++i) {
    const bool matching = i < 30 || i >= 90;
    const MapTrackerOutput out = TrackAt(c8::HMatrixGen::i(), matching);
    const bool ok = out.state == 1 || out.frame_tracker_ok;
    EXPECT_EQ(ok, matching || i == 30) << "frame " << i;
    sm.Update(ok, static_cast<int64_t>(i) * kFrameNs);
    status.push_back(sm.status());
  }
  ExpectLimited(status, 71, 130);
}

// Open review question (stale prev_depths): an inertial frame that goes LOST leaves the FFT depth
// list (+0xf18) of the last tracked frame in place. G926 writes the list at one site only (926.c
// 2586-2592: clear + resize to the previous-frame point count, -FLT_MAX fill, on a tracked frame);
// the inertial bookkeeping (F926:1227-1251) and the 664 map-reset (664.c:93-99, clears +0xf94 /
// +0xfb8 only) do not touch it, and 1266 clears it after its depth / ground step. The engine keeps
// that behaviour; this test pins it.
TEST_F(MapTrackerTest, InertialLostFrameKeepsDepthListUntilNextFrameTracker) {
  MapBuilderOutput ev;
  ev.keyframe_created = true;
  tracker_.ApplyMapBuilderOutput(ev);
  using frame_frame_tracker::FrameFrameTrackerAccess;
  std::vector<float> last_tracked;
  for (int i = 0; i < 30; ++i) {
    ASSERT_EQ(TrackAt(c8::HMatrixGen::i(), true).state, 1) << i;
    last_tracked = FrameFrameTrackerAccess::impl(tracker_.frame_tracker()).prev_depths;
  }
  ASSERT_FALSE(last_tracked.empty());
  // Frame 30: inertial path (TRACKING > initialFrameDelay), no map match -> LOST.
  const MapTrackerOutput lost = TrackAt(c8::HMatrixGen::i(), false);
  ASSERT_EQ(lost.state, 0);
  ASSERT_TRUE(lost.frame_tracker_ok);  // inertial frames report frameTrackerOk = 1
  const auto& ft = FrameFrameTrackerAccess::impl(tracker_.frame_tracker());
  EXPECT_EQ(ft.prev_depths, last_tracked);  // stale list kept (original behaviour)
  // Frame 31: LOST -> FrameFrameTracker::Track runs and clears the list.
  TrackAt(c8::HMatrixGen::i(), false);
  EXPECT_TRUE(FrameFrameTrackerAccess::impl(tracker_.frame_tracker()).prev_depths.empty());
}

// S2 review LOW-2: the inertial ground / sky sampling uses i % fraction.
using MapTrackerDeathTest = MapTrackerTest;

TEST_F(MapTrackerDeathTest, InertialGroundFractionZeroAborts) {
  MapBuilderOutput ev;
  ev.keyframe_created = true;
  tracker_.ApplyMapBuilderOutput(ev);
  for (int i = 0; i < 12; ++i) {
    ASSERT_EQ(TrackAt(c8::HMatrixGen::i(), true).state, 1) << i;
  }
  EXPECT_DEATH(
      {
        config::ApplyParamOverrides(
            {{"FrameFrameTracker.inertialTrackingGroundPointsFraction", ParamValue(0)}});
        TrackAt(c8::HMatrixGen::i(), true);
      },
      "inertialTrackingGroundPointsFraction must be >= 1");
}

// F926 small-motion gate (section D). Inputs are flip6-s2 capture fix-s2p1 frames 30 and 31 (engine
// dumps; bit patterns, matrix column-major, quaternions w, x, y, z). The instrumented original
// (1146 input copy, fix lane round 2) showed that at frame 31 the original tracked from the
// UNCHANGED previous pose: |t| <= 0.001, |vec(v)|^2 <= 1e-6 and |vec(d)|^2 <= 2e-5. At frame 30 it
// composed (|vec(v)|^2 > 1e-6). The engine used to compose whenever |vec(d)|^2 < 0.001.
float Bits(uint32_t u) {
  float f;
  std::memcpy(&f, &u, 4);
  return f;
}

c8::HMatrix DeltaFromBits(const uint32_t (&w)[16]) {
  float m[16];
  for (int i = 0; i < 16; ++i) m[i] = Bits(w[i]);
  return c8::HMatrix(m, nullptr, true);
}

c8::Quaternion QuatFromBits(uint32_t w, uint32_t x, uint32_t y, uint32_t z) {
  return c8::Quaternion(Bits(w), Bits(x), Bits(y), Bits(z));
}

TEST(MapTrackerMotionGate, KeepsThePoseBelowAllThresholds) {
  const uint32_t delta31[16] = {0x3f7ffff2, 0xba091a80, 0xba958d40, 0x00000000, 0x3a08d100, 0x3f7ffff7,
                                0xba786e00, 0x00000000, 0x3a959d40, 0x3a784400, 0x3f7fffee, 0x00000000,
                                0x39009180, 0xb8561e00, 0xba536a20, 0x3f800000};
  const c8::Quaternion prev = QuatFromBits(0xbf7883ee, 0xbe73cc42, 0xbcf65ff6, 0x3bb5762e);
  const c8::Quaternion rot = QuatFromBits(0xbf788140, 0xbe73ec40, 0xbcfa0786, 0x3ba34412);
  const MotionGate g = ComputeMotionGate(DeltaFromBits(delta31), prev, rot);
  EXPECT_LE(static_cast<double>(g.translation), 0.001);
  EXPECT_LE(static_cast<double>(g.visual_rotation2), 1e-6);
  EXPECT_LE(static_cast<double>(g.device_rotation2), 2e-5);
}

TEST(MapTrackerMotionGate, ComposesOnVisualRotation) {
  const uint32_t delta30[16] = {0x3f7fffd3, 0xbb146010, 0xb9fcb600, 0x00000000, 0x3b145900, 0x3f7fffce,
                                0xba60ec00, 0x00000000, 0x39fdbc00, 0x3a60a200, 0x3f7ffff8, 0x00000000,
                                0xb8e82100, 0xb9a36080, 0xba1c69b0, 0x3f800000};
  const c8::Quaternion prev = QuatFromBits(0xbf787cca, 0xbe744240, 0xbcf53a7c, 0x3bc4ee8a);
  const c8::Quaternion rot = QuatFromBits(0xbf7883ee, 0xbe73cc42, 0xbcf65ff6, 0x3bb5762e);
  const MotionGate g = ComputeMotionGate(DeltaFromBits(delta30), prev, rot);
  EXPECT_LE(static_cast<double>(g.translation), 0.001);
  EXPECT_GT(static_cast<double>(g.visual_rotation2), 1e-6);
}

TEST(MapTrackerMotionGate, NormalizesTheDeviceDelta) {
  // A non-unit previous rotation (scaled by 2) gives the same device delta as the unit one: 926
  // normalizes the conjugate and the product before squaring.
  const c8::Quaternion prev(0.9f, 0.1f, 0.3f, 0.2f);
  const c8::Quaternion prev2(1.8f, 0.2f, 0.6f, 0.4f);
  const c8::Quaternion rot(0.8f, 0.2f, 0.4f, 0.1f);
  const c8::HMatrix id = c8::HMatrixGen::i();
  EXPECT_NEAR(ComputeMotionGate(id, prev, rot).device_rotation2,
              ComputeMotionGate(id, prev2, rot).device_rotation2, 1e-6f);
  EXPECT_LT(ComputeMotionGate(id, rot, rot).device_rotation2, 1e-12f);  // rounding only
  EXPECT_EQ(ComputeMotionGate(id, rot, rot).visual_rotation2, 0.0f);
}

}  // namespace
}  // namespace slam_refact::map_tracker

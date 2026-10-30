// US-008 noiseless PnP (MUST-PASS, D-8): robustP2P and robustPnP run to the end (including
// solveOptimalPose) on a noise-free synthetic scene with 80 points and 25 % outliers; pose error
// < 1e-6 (architecture.md §7.3). The engine path is the 1265 wrapper ComputeFramePoseUsingRobustP2P.
// Q15 (MUST-PASS): poseEstimationAnalytical (1146) started at the exact pose fails with "Initial
// residual and Jacobian evaluation failed" (returns false); its accuracy on noisy input is measured
// and reported.

#include <cmath>
#include <cstdio>
#include <cstring>
#include <random>
#include <vector>

#include <gtest/gtest.h>

#include "c8/geometry/egomotion.h"
#include "c8/hmatrix.h"
#include "c8/hpoint.h"
#include "c8/quaternion.h"
#include "compat/random_numbers.h"
#include "map_tracker/map_tracker_internal.h"
#include "reality/engine/geometry/bundle.h"
#include "reality/engine/geometry/pose-pnp.h"
#include "scene_util.h"

namespace slam_refact::map_tracker {
namespace {

constexpr float kPoseTol = 1e-6f;

struct PnpScene {
  c8::HMatrix truth = c8::HMatrixGen::i();
  std::vector<c8::HPoint3> world;
  std::vector<c8::HPoint2> rays;
  std::vector<bool> inlier;
};

// 80 points, the last 20 with rays replaced by unrelated rays (25 % outliers).
PnpScene MakePnpScene(const c8::HMatrix& truth, float depth_min, float depth_max) {
  PnpScene s;
  s.truth = truth;
  std::mt19937 gen(8008);
  std::uniform_real_distribution<float> ux(-0.5f, 0.5f);
  std::uniform_real_distribution<float> uy(-0.38f, 0.38f);
  std::uniform_real_distribution<float> ud(depth_min, depth_max);
  for (int i = 0; i < 80; ++i) {
    const float rx = ux(gen);
    const float ry = uy(gen);
    const float d = ud(gen);
    const c8::HPoint3 pc(rx * d, ry * d, d);
    s.world.push_back(truth * pc);
    if (i < 60) {
      const c8::HPoint3 back = truth.inv() * s.world.back();
      s.rays.emplace_back(back.x() / back.z(), back.y() / back.z());
      s.inlier.push_back(true);
    } else {
      s.rays.emplace_back(ux(gen), uy(gen));
      s.inlier.push_back(false);
    }
  }
  return s;
}

TEST(MapTrackerPnp, Us008RobustP2PNoiselessThroughWrapper) {
  const c8::HMatrix truth = c8::cameraMotion(c8::HVector3(0.3f, -0.2f, 0.5f),
                                             c8::Quaternion::fromPitchYawRollDegrees(5, 20, -3));
  const PnpScene s = MakePnpScene(truth, 2.0f, 6.0f);
  std::vector<map::PointId> ids;
  for (size_t i = 0; i < s.world.size(); ++i) {
    ids.emplace_back(i, static_cast<uint32_t>(i + 1));
  }
  TrackerInput frame(testing::SceneIntrinsics(), c8::rotation(truth), 0);
  compat::RandomNumbers rng(compat::kEngineRngSeed);
  // Tracking pose: the true rotation (p2pImpl uses its rotation) with a 20 cm position offset.
  const c8::HMatrix tracking =
      c8::cameraMotion(c8::HVector3(0.5f, -0.2f, 0.5f), c8::rotation(truth));
  RelocResult r;
  c8::RobustP2PScratchSpace scratch;
  ComputeFramePoseUsingRobustP2P(frame, s.world, ids, s.rays, tracking, &rng, &scratch, &r);
  ASSERT_TRUE(r.success);
  const float et = testing::MaxTranslationError(r.pose, truth);
  const float er = testing::MaxRotationError(r.pose, truth);
  std::printf("US-008 robustP2P: |dt| %.3g |dR| %.3g inliers %zu confidence %.3f\n", et, er,
              r.inlier_ids.size(), r.confidence);
  EXPECT_LT(et, kPoseTol);
  EXPECT_LT(er, kPoseTol);
  EXPECT_EQ(r.inlier_ids.size(), 60u);
  for (const map::PointId& id : r.inlier_ids) {
    EXPECT_TRUE(s.inlier[id.index()]);
  }
}

// S2 review MEDIUM-1: the robustP2P scratch is owned by the caller (MapTracker::Impl), not a
// process-wide static. Within one owner the confidence grid keeps the bounds of the first call
// (Q16, F1265:109); a second owner builds its own grid from its own first call.
TEST(MapTrackerPnp, P2PScratchKeepsFirstGridPerOwner) {
  const c8::HMatrix truth = c8::cameraMotion(c8::HVector3(0.3f, -0.2f, 0.5f),
                                             c8::Quaternion::fromPitchYawRollDegrees(5, 20, -3));
  const PnpScene s = MakePnpScene(truth, 2.0f, 6.0f);
  std::vector<map::PointId> ids;
  for (size_t i = 0; i < s.world.size(); ++i) {
    ids.emplace_back(i, static_cast<uint32_t>(i + 1));
  }
  const c8::HMatrix tracking =
      c8::cameraMotion(c8::HVector3(0.5f, -0.2f, 0.5f), c8::rotation(truth));
  c8_PixelPinholeCameraModel k_wide = testing::SceneIntrinsics();
  k_wide.focalLengthHorizontal = 130.0f;
  k_wide.focalLengthVertical = 130.0f;
  const TrackerInput frame(testing::SceneIntrinsics(), c8::rotation(truth), 0);
  const TrackerInput frame_wide(k_wide, c8::rotation(truth), 0);

  const auto first_cell = [](const c8_PixelPinholeCameraModel& k) {
    c8::HPoint2 ll;
    c8::HPoint2 ur;
    c8::frameBounds(k, &ll, &ur);
    const float step = (ur.x() - ll.x()) / 25;  // uniformSampleGrid, GRID_X_STEPS = 25
    return c8::HPoint2(ll.x() + 0.5f * step, ll.y() + 0.5f * step);
  };
  const auto run = [&](const TrackerInput& f, c8::RobustP2PScratchSpace* scratch) {
    compat::RandomNumbers rng(compat::kEngineRngSeed);
    RelocResult r;
    ComputeFramePoseUsingRobustP2P(f, s.world, ids, s.rays, tracking, &rng, scratch, &r);
    return r;
  };

  c8::RobustP2PScratchSpace a;
  const RelocResult ra = run(frame, &a);
  ASSERT_TRUE(ra.success);
  ASSERT_FALSE(a.confidenceGrid.empty());
  const c8::HPoint2 narrow = first_cell(testing::SceneIntrinsics());
  EXPECT_EQ(a.confidenceGrid[0].x(), narrow.x());
  EXPECT_EQ(a.confidenceGrid[0].y(), narrow.y());
  const std::vector<c8::HPoint2> grid_a = a.confidenceGrid;
  run(frame_wide, &a);  // same owner, other bounds: the first grid stays (Q16)
  ASSERT_EQ(a.confidenceGrid.size(), grid_a.size());
  EXPECT_EQ(a.confidenceGrid[0].x(), grid_a[0].x());

  c8::RobustP2PScratchSpace b;  // another owner (another MapTracker / Engine)
  run(frame_wide, &b);
  const c8::HPoint2 wide = first_cell(k_wide);
  EXPECT_EQ(b.confidenceGrid[0].x(), wide.x());
  EXPECT_EQ(b.confidenceGrid[0].y(), wide.y());
  EXPECT_NE(b.confidenceGrid[0].x(), grid_a[0].x());

  // A fresh owner on the first frame reproduces the first owner's result bit for bit.
  c8::RobustP2PScratchSpace c;
  const RelocResult rc = run(frame, &c);
  ASSERT_TRUE(rc.success);
  EXPECT_EQ(std::memcmp(ra.pose.data().data(), rc.pose.data().data(), sizeof(float) * 16), 0);
  EXPECT_EQ(ra.confidence, rc.confidence);
  EXPECT_EQ(ra.inlier_ids.size(), rc.inlier_ids.size());
}

TEST(MapTrackerPnp, Us008RobustPnPNoiseless) {
  // robustPnP proposes camera positions with z in [minZ, maxZ] = [-9, 0.67] facing +z within 45
  // degrees (public defaults); the scene is placed accordingly.
  const c8::HMatrix truth = c8::cameraMotion(c8::HVector3(0.2f, -0.1f, -3.0f),
                                             c8::Quaternion::fromPitchYawRollDegrees(4, -10, 2));
  const PnpScene s = MakePnpScene(truth, 2.5f, 4.0f);
  compat::RandomNumbers rng(compat::kEngineRngSeed);
  c8::RobustPoseScratchSpace scratch;
  c8::HMatrix cam = c8::HMatrixGen::i();
  std::vector<uint8_t> inliers;
  c8::RobustPnPParams params;
  ASSERT_TRUE(c8::robustPnP(s.world, s.rays, c8::HMatrixGen::i(), params, &cam, &inliers, &rng,
                            &scratch));
  const float et = testing::MaxTranslationError(cam, truth);
  const float er = testing::MaxRotationError(cam, truth);
  std::printf("US-008 robustPnP: |dt| %.3g |dR| %.3g\n", et, er);
  EXPECT_LT(et, kPoseTol);
  EXPECT_LT(er, kPoseTol);
  size_t n = 0;
  for (size_t i = 0; i < inliers.size(); ++i) {
    if (inliers[i] != 0) {
      ++n;
      EXPECT_TRUE(s.inlier[i]) << i;
    }
  }
  // fillInliers keeps residuals <= 1.1 * the best proposal's percentile residual; on noise-free
  // data that residual is at float rounding level, so only part of the true inliers is flagged.
  // The flagged set must still be outlier-free and large enough for solveOptimalPose.
  EXPECT_GE(n, params.minPoseInliers);
}

std::vector<c8::ObservedPoint> Observations(const std::vector<c8::HPoint2>& rays) {
  std::vector<c8::ObservedPoint> obs;
  for (const c8::HPoint2& r : rays) {
    c8::ObservedPoint o;
    o.position = r;
    o.scale = 0;
    o.descriptorDist = 0.0f;
    o.weight = 1.0f;
    obs.push_back(o);
  }
  return obs;
}

c8::PoseEstimationAnalyticalParams Params1272() {
  c8::PoseEstimationAnalyticalParams p;
  p.suppressOutliers = true;
  p.scale = 1.0;
  p.useImageTargetCostFunction = false;
  p.cameraMotionWeight = 0.0;
  p.logReport = false;
  return p;
}

TEST(MapTrackerPnp, Q15AnalyticFailsAtExactPose) {
  const c8::HMatrix truth = c8::cameraMotion(c8::HVector3(0.3f, -0.2f, 0.5f),
                                             c8::Quaternion::fromPitchYawRollDegrees(5, 20, -3));
  PnpScene s = MakePnpScene(truth, 2.0f, 6.0f);
  s.world.resize(60);
  s.rays.resize(60);
  // Exact rays from the float pose: every residual is exactly 0 at the initial pose.
  for (size_t i = 0; i < s.world.size(); ++i) {
    const c8::HPoint3 pc = truth.inv() * s.world[i];
    s.rays[i] = c8::HPoint2(pc.x() / pc.z(), pc.y() / pc.z());
  }
  c8::HMatrix pose = truth;
  const bool ok = c8::poseEstimationAnalytical(Observations(s.rays), s.world, {}, &pose,
                                               Params1272());
  EXPECT_FALSE(ok);
}

TEST(MapTrackerPnp, Q15AnalyticAccuracyOnNoisyInput) {
  const c8::HMatrix truth = c8::cameraMotion(c8::HVector3(0.3f, -0.2f, 0.5f),
                                             c8::Quaternion::fromPitchYawRollDegrees(5, 20, -3));
  PnpScene s = MakePnpScene(truth, 2.0f, 6.0f);
  s.world.resize(60);
  s.rays.resize(60);
  std::mt19937 gen(1146);
  std::normal_distribution<float> noise(0.0f, 1e-3f);
  for (c8::HPoint2& r : s.rays) {
    r = c8::HPoint2(r.x() + noise(gen), r.y() + noise(gen));
  }
  // Start 2 cm / 0.5 degree away from the truth.
  c8::HMatrix pose = c8::cameraMotion(c8::HVector3(0.32f, -0.2f, 0.5f),
                                      c8::Quaternion::fromPitchYawRollDegrees(5.5f, 20, -3));
  const float et0 = testing::MaxTranslationError(pose, truth);
  const bool ok = c8::poseEstimationAnalytical(Observations(s.rays), s.world, {}, &pose,
                                               Params1272());
  ASSERT_TRUE(ok);
  const float et = testing::MaxTranslationError(pose, truth);
  const float er = testing::MaxRotationError(pose, truth);
  std::printf("Q15 1146 noisy (sigma 1e-3 ray): start |dt| %.3g -> |dt| %.3g |dR| %.3g\n", et0, et,
              er);
  EXPECT_TRUE(std::isfinite(et));
  EXPECT_TRUE(std::isfinite(er));
}

}  // namespace
}  // namespace slam_refact::map_tracker

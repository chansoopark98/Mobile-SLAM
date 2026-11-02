// P3 candidate R7: keyframe pose-graph loop closure (map_builder/pose_graph.h). Compiled only with
// SLAM_REFACT_LOOP_CLOSE=ON. Synthetic scenes only (no registered fixture is run here).
//
// Pre-declared expectations (fixed before the first run): the synthetic loop's translation RMSE
// against ground truth drops below half of the open-loop value; a consistent graph and a map
// without an accepted loop stay unchanged; the loop close success path with R7 runs to the end
// and C maps the old newest pose onto its pose-graph pose; moved points keep their coordinates in
// their reference keyframe (1e-5) and points of unmoved reference keyframes keep their bits.

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <memory>
#include <tuple>
#include <utility>
#include <vector>

#include "Eigen/Geometry"
#include "c8/hmatrix.h"
#include "c8/hpoint.h"
#include "c8/quaternion.h"
#include "common/tracking_types.h"
#include "config/accuracy.h"
#include "config/params.h"
#include "gtest/gtest.h"
#include "map/world_map.h"
#include "map_builder/loop_close.h"
#include "map_builder/map_builder.h"
#include "map_builder/pose_graph.h"
#include "map_builder_test_util.h"
#include "reality/engine/features/frame-point.h"
#include "reality/engine/features/image-descriptor.h"

namespace slam_refact::map_builder {
namespace {

using testing::Pose;
using testing::Project;
using testing::TestIntrinsics;

using Desc = std::array<uint8_t, 32>;

constexpr double kPi = 3.14159265358979323846;

class ScopedAccuracy {
 public:
  explicit ScopedAccuracy(bool r7) : previous_(config::Accuracy()) {
    AccuracyConfig cfg;
    cfg.enabled = true;
    cfg.r7_pose_graph = r7;
    config::SetAccuracy(cfg);
  }
  ~ScopedAccuracy() { config::SetAccuracy(previous_); }
  ScopedAccuracy(const ScopedAccuracy&) = delete;
  ScopedAccuracy& operator=(const ScopedAccuracy&) = delete;

 private:
  AccuracyConfig previous_;
};

uint32_t BitsOf(float f) {
  uint32_t u;
  std::memcpy(&u, &f, sizeof(u));
  return u;
}

bool SameBits(const c8::HMatrix& a, const c8::HMatrix& b) {
  for (size_t i = 0; i < 16; ++i) {
    if (BitsOf(a.data()[i]) != BitsOf(b.data()[i])) return false;
  }
  return true;
}

bool SameBits(const c8::HPoint3& a, const c8::HPoint3& b) {
  for (size_t i = 0; i < 4; ++i) {
    if (BitsOf(a.raw()[i]) != BitsOf(b.raw()[i])) return false;
  }
  return true;
}

float MaxAbsDiff(const c8::HMatrix& a, const c8::HMatrix& b) {
  float m = 0.0f;
  for (size_t i = 0; i < 16; ++i) m = std::max(m, std::fabs(a.data()[i] - b.data()[i]));
  return m;
}

// Angle (radians) of the rotation between two poses: acos((trace(R_a^T R_b) - 1) / 2).
double RotationAngle(const c8::HMatrix& a, const c8::HMatrix& b) {
  double tr = 0.0;
  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 3; ++c) tr += static_cast<double>(a(r, c)) * static_cast<double>(b(r, c));
  }
  return std::acos(std::max(-1.0, std::min(1.0, (tr - 1.0) / 2.0)));
}

c8::FeatureSet OrbSet(const Desc& d) {
  c8::FeatureSet set;
  set.orbFeature = c8::OrbFeature(d);
  return set;
}

double TranslationRmse(const std::vector<Eigen::Isometry3d>& est, const std::vector<Eigen::Isometry3d>& gt) {
  double sum = 0.0;
  for (size_t i = 0; i < est.size(); ++i) sum += (est[i].translation() - gt[i].translation()).squaredNorm();
  return std::sqrt(sum / static_cast<double>(est.size()));
}

// ---------------------------------------------------------------------------------------------
// Loop scene (the loop close live-run scene of loop_close_test.cc): the candidate keyframe sees
// `n` world points, four keyframes follow, and the newest keyframe sees the same appearances from
// pose_true while its stored (drifted) pose is pose_stored. `loop` = false gives the newest
// keyframe a global descriptor 128 bits away from every other one (no candidate below 80).
// ---------------------------------------------------------------------------------------------
struct LoopScene {
  map::WorldMaps maps;
  map::KeyframeId first;
  std::vector<map::KeyframeId> middle;
  map::KeyframeId newest;
  std::vector<map::PointId> points;
  c8::HMatrix pose_first = Pose(0.0f, 0.0f, 0.0f, 0.0f);
  c8::HMatrix pose_true = Pose(0.01f, 0.03f, 0.005f, 0.02f);
  c8::HMatrix pose_stored = c8::HMatrixGen::translation(0.04f, -0.01f, 0.03f) * c8::HMatrixGen::xRadians(0.31f) *
                            c8::HMatrixGen::yRadians(-0.52f) * c8::HMatrixGen::zRadians(0.17f);

  explicit LoopScene(bool loop, int n = 80) {
    map::WorldMap& w = maps.active();
    c8::FrameWithPoints fa(TestIntrinsics());
    c8::FrameWithPoints fb(TestIntrinsics());
    std::vector<c8::HPoint3> truth;
    for (int i = 0; i < n; ++i) {
      const c8::HPoint3 x(-1.2f + 0.03f * static_cast<float>(i), -0.5f + 0.03f * static_cast<float>((i * 7) % 30),
                          2.0f + 0.06f * static_cast<float>((i * 11) % 30));
      truth.push_back(x);
      const Desc d = testing::Descriptor(static_cast<uint32_t>(100 + i));
      fa.addImageRayPoint(Project(pose_first, x), 1, 0.25f, 0.5f, OrbSet(d));
      fb.addImageRayPoint(Project(pose_true, x), 1, 0.25f, 0.5f, OrbSet(d));
    }
    first = w.AddKeyframe(fa, 0);
    w.SetKeyframePose(first, pose_first);
    for (int i = 0; i < n; ++i) {
      const map::PointId p = w.AddPoint(w.GetKeyframe(first).seq, truth[static_cast<size_t>(i)],
                                        map::PointSource::kTriangulated);
      w.AddObservation(p, static_cast<uint32_t>(i), first);
      points.push_back(p);
    }
    for (int k = 1; k <= 4; ++k) {
      c8::FrameWithPoints ff(TestIntrinsics());
      for (int i = 0; i < 40; ++i) {
        ff.addImageRayPoint(c8::HPoint2(-0.5f + 0.025f * static_cast<float>(i), 0.1f * static_cast<float>(k) - 0.2f), 1,
                            0.25f, 0.5f, OrbSet(testing::Descriptor(static_cast<uint32_t>(1000 * k + i))));
      }
      const map::KeyframeId kf = w.AddKeyframe(ff, 5 * k);
      w.SetKeyframePose(kf, Pose(0.0f, 0.1f * static_cast<float>(k), 0.f, 0.f));
      w.MutableKeyframe(kf).global_descriptor.fill(0xFF);
      middle.push_back(kf);
    }
    newest = w.AddKeyframe(fb, 30);
    w.SetKeyframePose(newest, pose_stored);
    if (!loop) {
      w.MutableKeyframe(newest).global_descriptor.fill(0x0F);
    }
  }

  map::WorldMap& wm() { return maps.active(); }
};

// 1. A drifted loop (biased odometry, exact loop edge) is pulled back towards the ground truth.
TEST(P3R7PoseGraph, SyntheticLoopDriftIsReduced) {
  constexpr int kNodes = 12;
  std::vector<Eigen::Isometry3d> gt(kNodes);
  for (int i = 0; i < kNodes; ++i) {
    const double a = 2.0 * kPi * static_cast<double>(i) / static_cast<double>(kNodes);
    gt[static_cast<size_t>(i)] = Eigen::Isometry3d::Identity();
    gt[static_cast<size_t>(i)].linear() = Eigen::AngleAxisd(a, Eigen::Vector3d::UnitY()).toRotationMatrix();
    gt[static_cast<size_t>(i)].translation() = Eigen::Vector3d(2.0 * std::sin(a), 0.1 * std::sin(3.0 * a), 2.0 * std::cos(a));
  }
  const Eigen::Matrix3d bias = (Eigen::AngleAxisd(0.03, Eigen::Vector3d::UnitY()) *
                                Eigen::AngleAxisd(0.01, Eigen::Vector3d::UnitX())).toRotationMatrix();
  std::vector<Eigen::Isometry3d> est(kNodes);
  est[0] = gt[0];
  std::vector<PoseGraphEdge> edges;
  for (size_t i = 0; i + 1 < static_cast<size_t>(kNodes); ++i) {
    Eigen::Isometry3d z = gt[i].inverse() * gt[i + 1];
    z.linear() = z.linear() * bias;
    z.translation() = 1.05 * z.translation() + Eigen::Vector3d(0.01, 0.0, 0.0);
    est[i + 1] = est[i] * z;
    edges.push_back({i, i + 1, z});
  }
  edges.push_back({0, static_cast<size_t>(kNodes - 1), gt[0].inverse() * gt[static_cast<size_t>(kNodes - 1)]});
  const double before = TranslationRmse(est, gt);
  const double end_before = (est.back().translation() - gt.back().translation()).norm();
  const PoseGraphSummary s = OptimizePoseGraph(&est, 0, edges);
  ASSERT_TRUE(s.usable);
  EXPECT_LT(s.final_cost, s.initial_cost);
  const double after = TranslationRmse(est, gt);
  const double end_after = (est.back().translation() - gt.back().translation()).norm();
  EXPECT_GT(before, 0.1);
  EXPECT_LT(after, 0.5 * before) << "rmse before " << before << " after " << after;
  EXPECT_LT(end_after, 0.5 * end_before) << "end before " << end_before << " after " << end_after;
  EXPECT_TRUE(est[0].isApprox(gt[0], 1e-12));  // the fixed node
}

// 2. Without a loop nothing moves: a graph of consistent odometry edges keeps its poses, and with
// R7 on a map whose loop close finds no candidate is left bit-identical.
TEST(P3R7PoseGraph, NoLoopGraphIsUnchanged) {
  std::vector<Eigen::Isometry3d> poses(6);
  for (size_t i = 0; i < poses.size(); ++i) {
    poses[i] = Eigen::Isometry3d::Identity();
    poses[i].linear() = Eigen::AngleAxisd(0.2 * static_cast<double>(i), Eigen::Vector3d::UnitY()).toRotationMatrix();
    poses[i].translation() = Eigen::Vector3d(0.3 * static_cast<double>(i), 0.05, -0.1 * static_cast<double>(i));
  }
  std::vector<PoseGraphEdge> edges;
  for (size_t i = 0; i + 1 < poses.size(); ++i) edges.push_back({i, i + 1, poses[i].inverse() * poses[i + 1]});
  const std::vector<Eigen::Isometry3d> before = poses;
  const PoseGraphSummary s = OptimizePoseGraph(&poses, 0, edges);
  ASSERT_TRUE(s.usable);
  EXPECT_LT(s.initial_cost, 1e-20);
  for (size_t i = 0; i < poses.size(); ++i) EXPECT_TRUE(poses[i].isApprox(before[i], 1e-9)) << i;

  const ScopedAccuracy r7(true);
  LoopScene scene(false);
  map::WorldMap& wm = scene.wm();
  std::vector<c8::HMatrix> kf_before;
  for (const map::KeyframeId kf : wm.AllKeyframesByCreation()) kf_before.push_back(wm.GetKeyframe(kf).frame.extrinsic());
  std::vector<c8::HPoint3> pt_before;
  for (const map::PointId p : scene.points) pt_before.push_back(wm.Point(p).position);
  LoopCloseState st;
  MapBuilderOutput out;
  LoopCloseReport rep;
  EXPECT_FALSE(LoopClose(wm, &st, &out, &rep));
  EXPECT_EQ(rep.outcome, LoopCloseOutcome::kNoCandidate);
  EXPECT_FALSE(rep.pose_graph.applied);
  EXPECT_FALSE(out.world_transform_updated);
  size_t i = 0;
  for (const map::KeyframeId kf : wm.AllKeyframesByCreation()) {
    EXPECT_TRUE(SameBits(wm.GetKeyframe(kf).frame.extrinsic(), kf_before[i++]));
  }
  for (size_t k = 0; k < scene.points.size(); ++k) EXPECT_TRUE(SameBits(wm.Point(scene.points[k]).position, pt_before[k]));
}

// 3. The loop close success path with R7: same acceptance, the graph is applied, the run reaches
// the end of the merge / unlink stage and the common 1127 without aborting, and C carries the old
// newest pose onto the pose-graph pose (the tracker pose equals that old pose on a keyframe frame).
TEST(P3R7PoseGraph, LoopCloseSuccessPathRunsWithThePoseGraph) {
  const ScopedAccuracy r7(true);
  LoopScene scene(true);
  map::WorldMap& wm = scene.wm();
  std::vector<c8::HMatrix> middle_before;
  for (const map::KeyframeId kf : scene.middle) middle_before.push_back(wm.GetKeyframe(kf).frame.extrinsic());
  const c8::HMatrix old_newest = wm.GetKeyframe(scene.newest).frame.extrinsic();
  LoopCloseState st;
  MapBuilderOutput out;
  LoopCloseReport rep;
  ASSERT_TRUE(LoopClose(wm, &st, &out, &rep));
  wm.AdjustDirtyPoints();  // the common 1127 of MapBuilder::Update
  EXPECT_EQ(rep.outcome, LoopCloseOutcome::kApplied);
  EXPECT_TRUE(rep.candidate == scene.first);
  EXPECT_GE(rep.num_matches, 75u);  // WorldMaps.loopClosingMatchesRequired
  ASSERT_TRUE(rep.pose_graph.applied);
  EXPECT_EQ(rep.pose_graph.nodes, 6);
  EXPECT_EQ(rep.pose_graph.edges, 6);
  EXPECT_LT(rep.pose_graph.solve.final_cost, rep.pose_graph.solve.initial_cost);
  EXPECT_TRUE(out.world_transform_updated);
  EXPECT_TRUE(SameBits(wm.GetKeyframe(scene.first).frame.extrinsic(), scene.pose_first));  // gauge
  for (size_t k = 0; k < scene.middle.size(); ++k) {
    EXPECT_GT(MaxAbsDiff(wm.GetKeyframe(scene.middle[k]).frame.extrinsic(), middle_before[k]), 1e-4f) << k;
  }
  const c8::HMatrix& now = wm.GetKeyframe(scene.newest).frame.extrinsic();
  EXPECT_LT(RotationAngle(now, scene.pose_true), RotationAngle(old_newest, scene.pose_true));
  EXPECT_LT(MaxAbsDiff(old_newest * out.world_correction, now), 1e-5f);
  // a second attempt with the same newest keyframe: the ring holds the candidate, no new loop
  MapBuilderOutput out2;
  LoopCloseReport rep2;
  EXPECT_FALSE(LoopClose(wm, &st, &out2, &rep2));
  EXPECT_FALSE(rep2.pose_graph.applied);
}

// 4. Points move rigidly with their reference keyframe (the oldest observer); points whose
// reference keyframe is not corrected keep their bits, the corrected keyframes get the given poses.
TEST(P3R7PoseGraph, MapPointsFollowTheirReferenceKeyframe) {
  map::WorldMaps maps;
  map::WorldMap& wm = maps.active();
  const std::array<c8::HMatrix, 3> pose = {Pose(0.0f, 0.0f, 0.0f, 0.0f), Pose(0.1f, 0.3f, 0.0f, 0.1f),
                                           Pose(0.2f, 0.6f, 0.02f, 0.25f)};
  const std::array<c8::HPoint3, 3> x = {c8::HPoint3(0.2f, -0.1f, 3.0f), c8::HPoint3(0.6f, 0.2f, 3.5f),
                                        c8::HPoint3(0.9f, -0.3f, 4.0f)};
  // observers: x0 by {A, B} (reference A), x1 by {B, C} (reference B), x2 by {C} (reference C)
  const std::array<std::vector<int>, 3> seen_by = {std::vector<int>{0, 1}, std::vector<int>{1, 2}, std::vector<int>{2}};
  std::array<map::KeyframeId, 3> kf;
  for (int k = 0; k < 3; ++k) {
    c8::FrameWithPoints f(TestIntrinsics());
    for (int i = 0; i < 3; ++i) {
      f.addImageRayPoint(Project(pose[static_cast<size_t>(k)], x[static_cast<size_t>(i)]), 1, 0.25f, 0.5f,
                         OrbSet(testing::Descriptor(static_cast<uint32_t>(10 * k + i))));
    }
    kf[static_cast<size_t>(k)] = wm.AddKeyframe(f, 10 * k);
    wm.SetKeyframePose(kf[static_cast<size_t>(k)], pose[static_cast<size_t>(k)]);
  }
  std::array<map::PointId, 3> pid;
  for (int i = 0; i < 3; ++i) {
    pid[static_cast<size_t>(i)] = wm.AddPoint(0, x[static_cast<size_t>(i)], map::PointSource::kTriangulated);
    for (int k : seen_by[static_cast<size_t>(i)]) {
      wm.AddObservation(pid[static_cast<size_t>(i)], static_cast<uint32_t>(i), kf[static_cast<size_t>(k)]);
    }
  }
  const c8::HMatrix new_b = c8::HMatrixGen::translation(0.05f, -0.02f, 0.03f) * pose[1] * c8::HMatrixGen::yRadians(0.04f);
  const c8::HMatrix new_c = c8::HMatrixGen::translation(-0.08f, 0.01f, 0.06f) * pose[2] * c8::HMatrixGen::xRadians(-0.03f);
  const c8::HPoint3 x0_before = wm.Point(pid[0]).position;
  const int moved = ApplyKeyframeCorrections(wm, {{kf[1], new_b}, {kf[2], new_c}});
  EXPECT_EQ(moved, 2);
  EXPECT_TRUE(SameBits(wm.Point(pid[0]).position, x0_before));
  EXPECT_TRUE(SameBits(wm.GetKeyframe(kf[0]).frame.extrinsic(), pose[0]));
  EXPECT_TRUE(SameBits(wm.GetKeyframe(kf[1]).frame.extrinsic(), new_b));
  EXPECT_TRUE(SameBits(wm.GetKeyframe(kf[2]).frame.extrinsic(), new_c));
  const std::array<c8::HMatrix, 3> now = {pose[0], new_b, new_c};
  for (int i = 1; i < 3; ++i) {
    const size_t r = static_cast<size_t>(i);  // reference keyframe of x_i
    const c8::HPoint3 cam_before = pose[r].inv() * x[r];
    const c8::HPoint3 cam_after = now[r].inv() * wm.Point(pid[r]).position;
    EXPECT_NEAR(cam_after.x(), cam_before.x(), 1e-5f) << i;
    EXPECT_NEAR(cam_after.y(), cam_before.y(), 1e-5f) << i;
    EXPECT_NEAR(cam_after.z(), cam_before.z(), 1e-5f) << i;
    EXPECT_GT(std::fabs(wm.Point(pid[r]).position.x() - x[r].x()) +
                  std::fabs(wm.Point(pid[r]).position.z() - x[r].z()),
              1e-3f) << i;  // the world position did change
  }
}

// 5. R7 alone (MapBuilder.loopCloseEnabled left at its default false) activates the loop close
// detection in MapBuilder::Update and keeps the keyframe decisions and ids of the run without it.
TEST(P3R7PoseGraph, R7EnablesTheLoopCloseDetectionWithoutTheConfigKey) {
  constexpr int kGround = 24;
  constexpr int kSky = 40;
  auto run = [&](bool r7) {
    const ScopedAccuracy acc(r7);
    map::WorldMaps maps;
    MapBuilder mb;
    std::vector<uint32_t> ids;
    std::vector<bool> created;
    for (int f = 0; f < 60; ++f) {
      TrackerInput in(TestIntrinsics(), c8::Quaternion(1.f, 0.f, 0.f, 0.f), static_cast<int64_t>(f) * 33333333);
      std::vector<testing::RayFeature> rays;
      for (int i = 0; i < kGround; ++i) {
        rays.push_back({-0.4f + 0.035f * static_cast<float>(i), -0.1f - 0.01f * static_cast<float>(i % 7), 1,
                        static_cast<uint32_t>(f * 1000 + i)});
      }
      for (int i = 0; i < kSky; ++i) {
        rays.push_back({-0.4f + 0.02f * static_cast<float>(i), 0.2f + 0.003f * static_cast<float>(i), 1,
                        static_cast<uint32_t>(f * 1000 + 500 + i)});
      }
      testing::AddRays(&in.frame_points, rays);
      MapTrackerOutput trk;
      trk.pose = Pose(0.002f * static_cast<float>(f), 0.01f * static_cast<float>(f), 0.f, 0.f);
      trk.state = f == 0 ? 0 : 1;
      trk.frame_tracker_ok = true;
      trk.consecutive_frame_tracker_ok = 7 + f;
      for (uint32_t i = 0; i < kGround; ++i) trk.ground_idx.push_back(i);
      for (uint32_t i = 0; i < kSky; ++i) trk.sky_idx.push_back(kGround + i);
      MapBuilderOutput out;
      mb.Update(&maps, in, trk, f * 33, &out);
      created.push_back(out.keyframe_created);
    }
    size_t descriptors = 0;
    const Desc zero{};
    for (size_t s = 0; s < maps.active().NumKeyframeSlots(); ++s) {
      ids.push_back(maps.active().KeyframeAtSlot(s).id);
      descriptors += maps.active().KeyframeAtSlot(s).global_descriptor != zero ? 1 : 0;
    }
    return std::make_tuple(ids, created, descriptors);
  };
  const auto off = run(false);
  const auto on = run(true);
  EXPECT_EQ(std::get<0>(off), std::get<0>(on));
  EXPECT_EQ(std::get<1>(off), std::get<1>(on));
  EXPECT_GE(std::get<0>(on).size(), 5u);
  EXPECT_EQ(std::get<2>(off), 0u);
  EXPECT_GT(std::get<2>(on), 0u);
}

}  // namespace
}  // namespace slam_refact::map_builder

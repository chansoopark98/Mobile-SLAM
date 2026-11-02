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
#include "c8/geometry/egomotion.h"
#include "c8/hmatrix.h"
#include "c8/hpoint.h"
#include "c8/quaternion.h"
#include "common/tracking_types.h"
#include "compat/random_numbers.h"
#include "config/accuracy.h"
#include "config/params.h"
#include "config/settings.h"
#include "gtest/gtest.h"
#include "map/world_map.h"
#include "map/world_map_internal.h"
#include "map_builder/loop_close.h"
#include "map_builder/map_builder.h"
#include "map_builder/pose_graph.h"
#include "map_tracker/map_tracker.h"
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

float TranslationDistance(const c8::HMatrix& a, const c8::HMatrix& b) {
  const float dx = a(0, 3) - b(0, 3);
  const float dy = a(1, 3) - b(1, 3);
  const float dz = a(2, 3) - b(2, 3);
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

std::vector<map::KeyframeId> Observers(const map::WorldMap& wm, map::PointId p) {
  std::vector<map::KeyframeId> out;
  for (const auto& [kf, idx] : wm.Point(p).observations) out.push_back(kf);
  return out;
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

  std::vector<c8::HPoint3> truth;
  std::vector<map::PointId> extra_points;  // seen by the candidate only (no newest appearance)

  explicit LoopScene(bool loop, int n = 80, int extra = 0) {
    map::WorldMap& w = maps.active();
    c8::FrameWithPoints fa(TestIntrinsics());
    c8::FrameWithPoints fb(TestIntrinsics());
    for (int i = 0; i < n; ++i) {
      const c8::HPoint3 x(-1.2f + 0.03f * static_cast<float>(i), -0.5f + 0.03f * static_cast<float>((i * 7) % 30),
                          2.0f + 0.06f * static_cast<float>((i * 11) % 30));
      truth.push_back(x);
      const Desc d = testing::Descriptor(static_cast<uint32_t>(100 + i));
      fa.addImageRayPoint(Project(pose_first, x), 1, 0.25f, 0.5f, OrbSet(d));
      fb.addImageRayPoint(Project(pose_true, x), 1, 0.25f, 0.5f, OrbSet(d));
    }
    std::vector<c8::HPoint3> extra_x;
    for (int i = 0; i < extra; ++i) {
      const c8::HPoint3 x(-0.4f + 0.08f * static_cast<float>(i), 0.35f, 2.5f + 0.1f * static_cast<float>(i));
      extra_x.push_back(x);
      fa.addImageRayPoint(Project(pose_first, x), 1, 0.25f, 0.5f,
                          OrbSet(testing::Descriptor(static_cast<uint32_t>(70000 + i))));
    }
    first = w.AddKeyframe(fa, 0);
    w.SetKeyframePose(first, pose_first);
    for (int i = 0; i < n; ++i) {
      const map::PointId p = w.AddPoint(w.GetKeyframe(first).seq, truth[static_cast<size_t>(i)],
                                        map::PointSource::kTriangulated);
      w.AddObservation(p, static_cast<uint32_t>(i), first);
      points.push_back(p);
    }
    for (int i = 0; i < extra; ++i) {
      const map::PointId p = w.AddPoint(w.GetKeyframe(first).seq, extra_x[static_cast<size_t>(i)],
                                        map::PointSource::kTriangulated);
      w.AddObservation(p, static_cast<uint32_t>(n + i), first);
      extra_points.push_back(p);
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

// 1. A drifted loop (biased odometry; the loop end held at its exact loop pose, as R7 holds the
// newest keyframe at the loop close's pose) is pulled back towards the ground truth.
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
  const double before = TranslationRmse(est, gt);
  const double end_before = (est.back().translation() - gt.back().translation()).norm();
  est.back() = est[0] * (gt[0].inverse() * gt.back());  // loop end at the loop measurement
  const PoseGraphSummary s = OptimizePoseGraph(&est, {0, static_cast<size_t>(kNodes - 1)}, edges);
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
  const PoseGraphSummary s = OptimizePoseGraph(&poses, {0}, edges);
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
  EXPECT_EQ(rep.pose_graph.edges, 5);  // odometry only; the loop end is held at the PnP pose
  EXPECT_LT(rep.pose_graph.solve.final_cost, rep.pose_graph.solve.initial_cost);
  EXPECT_TRUE(out.world_transform_updated);
  EXPECT_TRUE(SameBits(wm.GetKeyframe(scene.first).frame.extrinsic(), scene.pose_first));  // gauge
  for (size_t k = 0; k < scene.middle.size(); ++k) {
    EXPECT_GT(MaxAbsDiff(wm.GetKeyframe(scene.middle[k]).frame.extrinsic(), middle_before[k]), 1e-4f) << k;
  }
  const c8::HMatrix& now = wm.GetKeyframe(scene.newest).frame.extrinsic();
  EXPECT_LT(RotationAngle(now, scene.pose_true), RotationAngle(old_newest, scene.pose_true));
  EXPECT_LT(TranslationDistance(now, scene.pose_true), TranslationDistance(old_newest, scene.pose_true));
  EXPECT_LT(MaxAbsDiff(old_newest * out.world_correction, now), 1e-5f);
  // the candidate's points are referenced by the fixed candidate: not moved by the graph
  EXPECT_EQ(rep.pose_graph.points.moved_points, 0);
  {  // the newest keyframe and C are exactly those of the original loop close (R7 off)
    const ScopedAccuracy off(false);
    LoopScene orig(true);
    LoopCloseState st_o;
    MapBuilderOutput out_o;
    ASSERT_TRUE(LoopClose(orig.wm(), &st_o, &out_o));
    EXPECT_TRUE(SameBits(now, orig.wm().GetKeyframe(orig.newest).frame.extrinsic()));
    EXPECT_TRUE(SameBits(out.world_correction, out_o.world_correction));
  }
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
  const KeyframeCorrectionResult res = ApplyKeyframeCorrections(wm, {{kf[1], new_b}, {kf[2], new_c}});
  wm.AdjustDirtyPoints();  // 1127 must not re-place the corrected points
  EXPECT_EQ(res.checked_points, 3);
  EXPECT_EQ(res.moved_points, 2);
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

// 6. A point shared by a keyframe that is not corrected (A, like the fixed loop candidate) and a
// corrected one (B) stays where A sees it, also after 1127; B's observation is kept when its ray
// still agrees within WorldMap.maxInternalSquaredResidualForTriangulatedPts and unlinked by 1150
// when it does not. A point referenced by B moves with B and keeps its link.
TEST(P3R7PoseGraph, SharedPointStaysInTheUncorrectedReferenceFrameAfterAdjust) {
  const float max_sq = config::WorldMapSettings::Get().max_internal_squared_residual_for_triangulated_pts;
  for (const float b_new_x : {3.0f, 1.01f}) {
    map::WorldMaps maps;
    map::WorldMap& wm = maps.active();
    const c8::HMatrix pa = Pose(0.0f, 0.0f, 0.0f, 0.0f);
    const c8::HMatrix pb = Pose(0.0f, 1.0f, 0.0f, 0.0f);
    const c8::HMatrix pb_new = Pose(0.0f, b_new_x, 0.0f, 0.0f);
    const c8::HPoint3 xs(0.0f, 0.0f, 5.0f);  // seen by A and B
    const c8::HPoint3 xb(1.0f, 0.2f, 4.0f);  // seen by B only
    c8::FrameWithPoints fa(TestIntrinsics());
    fa.addImageRayPoint(Project(pa, xs), 1, 0.25f, 0.5f, OrbSet(testing::Descriptor(1)));
    c8::FrameWithPoints fb(TestIntrinsics());
    fb.addImageRayPoint(Project(pb, xs), 1, 0.25f, 0.5f, OrbSet(testing::Descriptor(2)));
    fb.addImageRayPoint(Project(pb, xb), 1, 0.25f, 0.5f, OrbSet(testing::Descriptor(3)));
    const map::KeyframeId a = wm.AddKeyframe(fa, 0);
    wm.SetKeyframePose(a, pa);
    const map::KeyframeId b = wm.AddKeyframe(fb, 10);
    wm.SetKeyframePose(b, pb);
    const map::PointId ps = wm.AddPoint(0, xs, map::PointSource::kTriangulated);
    wm.AddObservation(ps, 0, a);
    wm.AddObservation(ps, 0, b);
    const map::PointId pbo = wm.AddPoint(0, xb, map::PointSource::kTriangulated);
    wm.AddObservation(pbo, 1, b);
    wm.AdjustDirtyPoints();
    const c8::HPoint3 xs_before = wm.Point(ps).position;
    const c8::HPoint3 xb_before = wm.Point(pbo).position;

    // expected link of B on xs: its stored ray against xs seen from the corrected B
    const c8::HPoint2 ray_b = Project(pb, xs_before);
    const c8::HPoint2 seen = Project(pb_new, xs_before);
    const float dx = seen.x() - ray_b.x();
    const float dy = seen.y() - ray_b.y();
    const bool b_kept = !(dx * dx + dy * dy > max_sq);

    const KeyframeCorrectionResult res = ApplyKeyframeCorrections(wm, {{b, pb_new}});
    wm.AdjustDirtyPoints();
    EXPECT_EQ(res.checked_points, 2) << b_new_x;
    EXPECT_EQ(res.moved_points, 1) << b_new_x;
    EXPECT_EQ(res.unlinked_observations, b_kept ? 0 : 1) << b_new_x;
    EXPECT_TRUE(SameBits(wm.Point(ps).position, xs_before)) << b_new_x;  // A's frame
    const std::vector<map::KeyframeId> expect_obs = b_kept ? std::vector<map::KeyframeId>{a, b}
                                                           : std::vector<map::KeyframeId>{a};
    std::vector<map::KeyframeId> sorted_expect = expect_obs;
    std::sort(sorted_expect.begin(), sorted_expect.end());
    EXPECT_EQ(Observers(wm, ps), sorted_expect) << b_new_x;
    EXPECT_NE(wm.Point(ps).state, map::PointState::kDead) << b_new_x;
    // the point referenced by B moved with B and keeps B's observation
    const c8::HPoint3 cam_before = pb.inv() * xb_before;
    const c8::HPoint3 cam_after = pb_new.inv() * wm.Point(pbo).position;
    EXPECT_NEAR(cam_after.x(), cam_before.x(), 1e-5f) << b_new_x;
    EXPECT_NEAR(cam_after.y(), cam_before.y(), 1e-5f) << b_new_x;
    EXPECT_NEAR(cam_after.z(), cam_before.z(), 1e-5f) << b_new_x;
    EXPECT_EQ(Observers(wm, pbo), std::vector<map::KeyframeId>{b}) << b_new_x;
    if (b_new_x == 3.0f) {
      EXPECT_FALSE(b_kept);  // the scene exercises the unlink branch
    } else {
      EXPECT_TRUE(b_kept);   // and the keep branch
    }
  }
}

// 7. The tracker frame after the loop close starts from the corrected newest keyframe pose: the
// MapTracker holds the old newest pose on the keyframe frame, applies the MapBuilderOutput (664),
// and its pose (the next Track's start pose) equals the pose-graph pose of the newest keyframe.
// Then one real Track() of the next frame (camera at the true newest pose, seeing the candidate's
// points) runs from there; pre-declared: its pose is within 0.02 (translation and rotation matrix
// elements) of the true pose.
TEST(P3R7PoseGraph, NextTrackerFrameStartsFromTheCorrectedNewestPose) {
  const ScopedAccuracy r7(true);
  LoopScene scene(true);
  map::WorldMap& wm = scene.wm();
  const c8::HMatrix old_newest = wm.GetKeyframe(scene.newest).frame.extrinsic();
  map_tracker::MapTracker tracker;
  MapBuilderOutput prime;
  prime.keyframe_created = true;  // the map is initialized (tracking, not relocalization)
  prime.pose_correction_applied = true;
  prime.pose_correction = old_newest;  // identity start pose -> old newest pose
  tracker.ApplyMapBuilderOutput(prime);
  ASSERT_LT(MaxAbsDiff(tracker.pose(), old_newest), 1e-6f);
  LoopCloseState st;
  MapBuilderOutput out;
  LoopCloseReport rep;
  ASSERT_TRUE(LoopClose(wm, &st, &out, &rep));
  ASSERT_TRUE(rep.pose_graph.applied);
  wm.AdjustDirtyPoints();
  const c8::HMatrix next_start = tracker.ApplyMapBuilderOutput(out);
  const c8::HMatrix& corrected = wm.GetKeyframe(scene.newest).frame.extrinsic();
  EXPECT_LT(MaxAbsDiff(next_start, corrected), 1e-5f);
  EXPECT_LT(MaxAbsDiff(tracker.pose(), corrected), 1e-5f);
  EXPECT_GT(MaxAbsDiff(next_start, old_newest), 1e-2f);  // the correction is not hidden

  TrackerInput in(TestIntrinsics(), c8::rotation(scene.pose_true), 33333333);
  for (size_t i = 0; i < scene.truth.size(); ++i) {
    in.frame_points.addImageRayPoint(Project(scene.pose_true, scene.truth[i]), 1, 0.25f, 0.5f,
                                     OrbSet(testing::Descriptor(static_cast<uint32_t>(100 + i))));
  }
  FrameState pred;
  pred.pose = tracker.pose();
  pred.time_ns = 33333333;
  pred.xr_device_rotation = c8::rotation(scene.pose_true);
  MapTrackerOutput trk;
  compat::RandomNumbers rng(compat::kEngineRngSeed);
  tracker.Track(wm, &in, pred, &trk, &rng);
  float max_t = 0.0f;
  float max_r = 0.0f;
  for (int r = 0; r < 3; ++r) {
    max_t = std::max(max_t, std::fabs(trk.pose(r, 3) - scene.pose_true(r, 3)));
    for (int c = 0; c < 3; ++c) max_r = std::max(max_r, std::fabs(trk.pose(r, c) - scene.pose_true(r, c)));
  }
  EXPECT_LE(max_t, 0.02f) << "state " << trk.state;
  EXPECT_LE(max_r, 0.02f) << "state " << trk.state;
}

// Restated LC merge of the loop scene on a map built without R7: keyframes set directly to the
// given poses, the candidate's points settled (not dirty), then the newest keyframe's matched
// observations added in match order and the original 1127/1150 run.
void ReferenceMerge(LoopScene* ref, const std::vector<c8::HMatrix>& middle_poses, const c8::HMatrix& newest_pose) {
  map::WorldMap& w = ref->wm();
  for (size_t k = 0; k < ref->middle.size(); ++k) w.SetKeyframePose(ref->middle[k], middle_poses[k]);
  w.SetKeyframePose(ref->newest, newest_pose);
  map::WorldMapAccess::impl(w).dirty.clear();
  for (size_t i = 0; i < ref->points.size(); ++i) {
    w.AddObservation(ref->points[i], static_cast<uint32_t>(i), ref->newest);
  }
  w.AdjustDirtyPoints();
  w.AdjustDirtyPoints();
}

// 10. Full LoopClose success path with R7 and the following 1127: the graph corrects the poses
// first, then the newest keyframe's matched observations are added and the points re-placed. The
// candidate's matched points end where the original 1127/1150 puts them from the corrected poses
// (a map built without R7 and set to those poses), with the same observation links; the
// candidate's other points are bit-identical and keep only the candidate's observation.
TEST(P3R7PoseGraph, LoopCloseFullPathPlacesMatchedPointsByTheOriginalRule) {
  const ScopedAccuracy r7(true);
  LoopScene scene(true, 80, 6);
  map::WorldMap& wm = scene.wm();
  map::WorldMapAccess::impl(wm).dirty.clear();  // a settled map: positions as built
  std::vector<c8::HPoint3> extra_before;
  for (const map::PointId p : scene.extra_points) extra_before.push_back(wm.Point(p).position);
  LoopCloseState st;
  MapBuilderOutput out;
  LoopCloseReport rep;
  ASSERT_TRUE(LoopClose(wm, &st, &out, &rep));
  ASSERT_TRUE(rep.pose_graph.applied);
  ASSERT_EQ(rep.num_matches, 80u);
  ASSERT_EQ(rep.num_correspondences, 80u);
  wm.AdjustDirtyPoints();  // the common 1127 of MapBuilder::Update

  std::vector<c8::HMatrix> middle_poses;
  for (const map::KeyframeId kf : scene.middle) middle_poses.push_back(wm.GetKeyframe(kf).frame.extrinsic());
  LoopScene ref(true, 80, 6);
  ReferenceMerge(&ref, middle_poses, wm.GetKeyframe(scene.newest).frame.extrinsic());

  int both = 0;
  for (size_t i = 0; i < scene.points.size(); ++i) {
    const map::WorldPoint& a = wm.Point(scene.points[i]);
    const map::WorldPoint& b = ref.wm().Point(ref.points[i]);
    EXPECT_EQ(a.state == map::PointState::kDead, b.state == map::PointState::kDead) << i;
    EXPECT_NEAR(a.position.x(), b.position.x(), 1e-6f) << i;
    EXPECT_NEAR(a.position.y(), b.position.y(), 1e-6f) << i;
    EXPECT_NEAR(a.position.z(), b.position.z(), 1e-6f) << i;
    EXPECT_EQ(a.observations.count(scene.first), b.observations.count(ref.first)) << i;
    EXPECT_EQ(a.observations.count(scene.newest), b.observations.count(ref.newest)) << i;
    EXPECT_EQ(a.observations.size(), b.observations.size()) << i;
    both += a.observations.size() == 2 ? 1 : 0;
  }
  ::testing::Test::RecordProperty("matched_points_linked_to_both", both);
  for (size_t k = 0; k < scene.extra_points.size(); ++k) {
    EXPECT_TRUE(SameBits(wm.Point(scene.extra_points[k]).position, extra_before[k])) << k;
    EXPECT_EQ(Observers(wm, scene.extra_points[k]), std::vector<map::KeyframeId>{scene.first}) << k;
  }
}

// Pose bits of every keyframe in creation order and position bits of the given points.
struct MapBits {
  std::vector<c8::HMatrix> kf;
  std::vector<c8::HPoint3> pt;
};

MapBits Snapshot(const map::WorldMap& wm, const std::vector<map::PointId>& points) {
  MapBits b;
  for (const map::KeyframeId kf : wm.AllKeyframesByCreation()) b.kf.push_back(wm.GetKeyframe(kf).frame.extrinsic());
  for (const map::PointId p : points) b.pt.push_back(wm.Point(p).position);
  return b;
}

// 12. Candidate and newest keyframe adjacent (two nodes, no free node): nothing is solved and the
// newest keyframe gets exactly the loop pose, every other keyframe and the candidate's points keep
// their bits (the original single-keyframe update).
TEST(P3R7PoseGraph, AdjacentCandidateBehavesAsTheOriginalLoopClose) {
  LoopScene scene(true);
  map::WorldMap& wm = scene.wm();
  const MapBits before = Snapshot(wm, scene.points);
  const map::KeyframeId cand = scene.middle.back();  // created right before the newest keyframe
  LoopPoseGraphReport rep;
  ASSERT_TRUE(CloseLoopWithPoseGraph(wm, cand, scene.newest, scene.pose_true, &rep));
  EXPECT_TRUE(rep.applied);
  EXPECT_EQ(rep.nodes, 2);
  EXPECT_EQ(rep.edges, 1);
  EXPECT_TRUE(rep.solve.usable);
  EXPECT_EQ(rep.solve.iterations, 0);
  EXPECT_EQ(rep.points.checked_points, 0);
  const MapBits after = Snapshot(wm, scene.points);
  const std::vector<map::KeyframeId> order = wm.AllKeyframesByCreation();
  for (size_t i = 0; i < order.size(); ++i) {
    const bool is_newest = order[i] == scene.newest;
    EXPECT_TRUE(SameBits(after.kf[i], is_newest ? scene.pose_true : before.kf[i])) << i;
  }
  for (size_t k = 0; k < after.pt.size(); ++k) EXPECT_TRUE(SameBits(after.pt[k], before.pt[k])) << k;
}

// 13. The same adjacent pair with a non-finite loop pose: refused before anything is applied, the
// map keeps its bits (the caller then keeps the original update).
TEST(P3R7PoseGraph, AdjacentCandidateWithNonFiniteLoopPoseIsRejected) {
  LoopScene scene(true);
  map::WorldMap& wm = scene.wm();
  const MapBits before = Snapshot(wm, scene.points);
  const float nan = std::nanf("");
  LoopPoseGraphReport rep;
  EXPECT_FALSE(CloseLoopWithPoseGraph(wm, scene.middle.back(), scene.newest,
                                      c8::HMatrixGen::translation(nan, 0.0f, 0.0f), &rep));
  EXPECT_FALSE(rep.applied);
  EXPECT_FALSE(rep.solve.usable);
  EXPECT_EQ(rep.nodes, 2);
  const MapBits after = Snapshot(wm, scene.points);
  for (size_t i = 0; i < after.kf.size(); ++i) EXPECT_TRUE(SameBits(after.kf[i], before.kf[i])) << i;
  for (size_t k = 0; k < after.pt.size(); ++k) EXPECT_TRUE(SameBits(after.pt[k], before.pt[k])) << k;
}

// 11. Ground points keep the original placement rule: a lone ground point whose keyframe is
// corrected upwards by 0.5 is re-placed by 1127 on the plane y = -1 along its ray from the corrected
// camera, not carried rigidly to y = -0.5.
TEST(P3R7PoseGraph, GroundPointIsRePlacedOnThePlaneFromTheCorrectedKeyframe) {
  map::WorldMaps maps;
  map::WorldMap& wm = maps.active();
  const c8::HMatrix pose = Pose(0.0f, 0.0f, 0.0f, 0.0f);
  const c8::HPoint3 g(0.3f, -1.0f, 3.0f);
  c8::FrameWithPoints f(TestIntrinsics());
  f.addImageRayPoint(Project(pose, g), 1, 0.25f, 0.5f, OrbSet(testing::Descriptor(7)));
  const map::KeyframeId kf = wm.AddKeyframe(f, 0);
  wm.SetKeyframePose(kf, pose);
  const map::PointId pid = wm.AddPoint(0, g, map::PointSource::kGround);
  wm.AddObservation(pid, 0, kf);
  wm.AdjustDirtyPoints();
  ASSERT_NEAR(wm.Point(pid).position.y(), -1.0f, 1e-4f);  // the ground rule before the correction

  const c8::HMatrix up = c8::HMatrixGen::translation(0.0f, 0.5f, 0.0f) * pose;
  const KeyframeCorrectionResult res = ApplyKeyframeCorrections(wm, {{kf, up}});
  wm.AdjustDirtyPoints();
  EXPECT_EQ(res.checked_points, 0);  // not placed by R7
  // ray (0.1, -1/3, 1) from the camera at (0, 0.5, 0) meets y = -1 at s = 4.5
  const c8::HPoint2 ray = Project(pose, g);
  const float s = (-1.0f - 0.5f) / ray.y();
  EXPECT_NEAR(wm.Point(pid).position.x(), s * ray.x(), 1e-4f);
  EXPECT_NEAR(wm.Point(pid).position.y(), -1.0f, 1e-4f);
  EXPECT_NEAR(wm.Point(pid).position.z(), s, 1e-4f);
  EXPECT_EQ(Observers(wm, pid), std::vector<map::KeyframeId>{kf});
}

// 8. Graphs without a cycle are refused and leave the map untouched: the candidate equal to the
// newest keyframe (self candidate) and a candidate that is not older than the newest keyframe.
TEST(P3R7PoseGraph, SelfCandidateAndUnorderedNodesLeaveTheMapUntouched) {
  LoopScene scene(true);
  map::WorldMap& wm = scene.wm();
  std::vector<c8::HMatrix> kf_before;
  for (const map::KeyframeId kf : wm.AllKeyframesByCreation()) kf_before.push_back(wm.GetKeyframe(kf).frame.extrinsic());
  std::vector<c8::HPoint3> pt_before;
  for (const map::PointId p : scene.points) pt_before.push_back(wm.Point(p).position);
  LoopPoseGraphReport rep;
  EXPECT_FALSE(CloseLoopWithPoseGraph(wm, scene.newest, scene.newest, scene.pose_true, &rep));
  EXPECT_FALSE(rep.applied);
  EXPECT_EQ(rep.nodes, 0);
  EXPECT_FALSE(CloseLoopWithPoseGraph(wm, scene.newest, scene.first, scene.pose_true, &rep));  // reversed
  EXPECT_FALSE(rep.applied);
  EXPECT_EQ(rep.nodes, 0);
  size_t i = 0;
  for (const map::KeyframeId kf : wm.AllKeyframesByCreation()) {
    EXPECT_TRUE(SameBits(wm.GetKeyframe(kf).frame.extrinsic(), kf_before[i++]));
  }
  for (size_t k = 0; k < scene.points.size(); ++k) EXPECT_TRUE(SameBits(wm.Point(scene.points[k]).position, pt_before[k]));
}

// 9. An unusable solve (non-finite loop measurement) is refused and changes no pose (the loop
// close then keeps its original single-keyframe update, loop_close.cc ApplyResult).
TEST(P3R7PoseGraph, UnusableSolveChangesNothing) {
  std::vector<Eigen::Isometry3d> poses(3, Eigen::Isometry3d::Identity());
  poses[1].translation() = Eigen::Vector3d(1.0, 0.0, 0.0);
  poses[2].translation() = Eigen::Vector3d(2.0, 0.0, 0.0);
  const std::vector<Eigen::Isometry3d> before = poses;
  Eigen::Isometry3d bad = Eigen::Isometry3d::Identity();
  bad.translation() = Eigen::Vector3d(std::nan(""), 0.0, 0.0);
  const std::vector<PoseGraphEdge> edges = {{0, 1, poses[0].inverse() * poses[1]},
                                            {1, 2, poses[1].inverse() * poses[2]},
                                            {0, 2, bad}};
  const PoseGraphSummary s = OptimizePoseGraph(&poses, {0}, edges);
  EXPECT_FALSE(s.usable);
  for (size_t i = 0; i < poses.size(); ++i) EXPECT_TRUE(poses[i].isApprox(before[i], 0.0)) << i;

  LoopScene scene(true);
  map::WorldMap& wm = scene.wm();
  std::vector<c8::HMatrix> kf_before;
  for (const map::KeyframeId kf : wm.AllKeyframesByCreation()) kf_before.push_back(wm.GetKeyframe(kf).frame.extrinsic());
  const float nan = std::nanf("");
  LoopPoseGraphReport rep;
  EXPECT_FALSE(CloseLoopWithPoseGraph(wm, scene.first, scene.newest,
                                      c8::HMatrixGen::translation(nan, 0.0f, 0.0f), &rep));
  EXPECT_FALSE(rep.applied);
  EXPECT_FALSE(rep.solve.usable);
  EXPECT_EQ(rep.nodes, 6);
  size_t i = 0;
  for (const map::KeyframeId kf : wm.AllKeyframesByCreation()) {
    EXPECT_TRUE(SameBits(wm.GetKeyframe(kf).frame.extrinsic(), kf_before[i++]));
  }
}

}  // namespace
}  // namespace slam_refact::map_builder

// P3 candidate R7: keyframe pose-graph loop closure (design in pose_graph.h). Compiled only with
// SLAM_REFACT_LOOP_CLOSE=ON (map_builder/CMakeLists.txt).

#include "map_builder/pose_graph.h"

#include <array>
#include <cstddef>
#include <cstdint>
#include <map>
#include <utility>
#include <vector>

#include "Eigen/Core"
#include "Eigen/Geometry"
#include "ceres/ceres.h"
#include "c8/geometry/egomotion.h"
#include "c8/hpoint.h"
#include "c8/quaternion.h"
#include "common/check.h"
#include "common/stage_timer.h"

namespace slam_refact::map_builder {
namespace {

constexpr int kMaxIterations = 50;

Eigen::Matrix4d ToEigen(const c8::HMatrix& m) {
  const std::array<float, 16>& d = m.data();  // column-major
  Eigen::Matrix4d e;
  for (int c = 0; c < 4; ++c) {
    for (int r = 0; r < 4; ++r) {
      e(r, c) = static_cast<double>(d[static_cast<size_t>(c * 4 + r)]);
    }
  }
  return e;
}

Eigen::Isometry3d ToIsometry(const c8::HMatrix& m) {
  const Eigen::Matrix4d e = ToEigen(m);
  Eigen::Isometry3d iso = Eigen::Isometry3d::Identity();
  iso.linear() = Eigen::Quaterniond(Eigen::Matrix3d(e.block<3, 3>(0, 0))).normalized().toRotationMatrix();
  iso.translation() = e.block<3, 1>(0, 3);
  return iso;
}

c8::HMatrix ToHMatrix(const Eigen::Isometry3d& p) {
  const Eigen::Quaterniond q(p.rotation());
  const Eigen::Vector3d& t = p.translation();
  return c8::cameraMotion(
      c8::HPoint3(static_cast<float>(t.x()), static_cast<float>(t.y()), static_cast<float>(t.z())),
      c8::Quaternion(static_cast<float>(q.w()), static_cast<float>(q.x()), static_cast<float>(q.y()),
                     static_cast<float>(q.z())));
}

// Relative SE(3) error of one edge (the ceres pose_graph_3d residual). Parameter blocks:
// translation (3) and Eigen quaternion x, y, z, w (4) of a and of b.
struct RelativePoseError {
  explicit RelativePoseError(const Eigen::Isometry3d& a_from_b)
      : p_ab(a_from_b.translation()), q_ab(Eigen::Quaterniond(a_from_b.rotation()).normalized()) {}

  template <typename T>
  bool operator()(const T* pa, const T* qa, const T* pb, const T* qb, T* residual) const {
    const Eigen::Map<const Eigen::Matrix<T, 3, 1>> p_a(pa);
    const Eigen::Map<const Eigen::Quaternion<T>> q_a(qa);
    const Eigen::Map<const Eigen::Matrix<T, 3, 1>> p_b(pb);
    const Eigen::Map<const Eigen::Quaternion<T>> q_b(qb);
    const Eigen::Quaternion<T> q_a_inv = q_a.conjugate();
    const Eigen::Quaternion<T> q_ab_est = q_a_inv * q_b;
    const Eigen::Matrix<T, 3, 1> p_ab_est = q_a_inv * (p_b - p_a);
    const Eigen::Quaternion<T> delta = q_ab.template cast<T>() * q_ab_est.conjugate();
    Eigen::Map<Eigen::Matrix<T, 6, 1>> r(residual);
    r.template head<3>() = p_ab_est - p_ab.template cast<T>();
    r.template tail<3>() = T(2.0) * delta.vec();
    return true;
  }

  Eigen::Vector3d p_ab;
  Eigen::Quaterniond q_ab;
};

}  // namespace

PoseGraphSummary OptimizePoseGraph(std::vector<Eigen::Isometry3d>* poses, size_t fixed,
                                   const std::vector<PoseGraphEdge>& edges) {
  PoseGraphSummary out;
  const size_t n = poses->size();
  SR_CHECK(fixed < n, "[pose_graph] fixed node out of range");
  std::vector<std::array<double, 3>> p(n);
  std::vector<std::array<double, 4>> q(n);  // Eigen order x, y, z, w
  for (size_t i = 0; i < n; ++i) {
    const Eigen::Vector3d& t = (*poses)[i].translation();
    const Eigen::Quaterniond r = Eigen::Quaterniond((*poses)[i].rotation()).normalized();
    p[i] = {t.x(), t.y(), t.z()};
    q[i] = {r.x(), r.y(), r.z(), r.w()};
  }

  ceres::EigenQuaternionManifold quaternion_manifold;  // outlives the problem
  ceres::Problem::Options problem_options;
  problem_options.manifold_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
  ceres::Problem problem(problem_options);
  for (size_t i = 0; i < n; ++i) {
    problem.AddParameterBlock(p[i].data(), 3);
    problem.AddParameterBlock(q[i].data(), 4, &quaternion_manifold);
  }
  for (const PoseGraphEdge& e : edges) {
    SR_CHECK(e.a < n && e.b < n && e.a != e.b, "[pose_graph] edge node out of range");
    problem.AddResidualBlock(
        new ceres::AutoDiffCostFunction<RelativePoseError, 6, 3, 4, 3, 4>(
            new RelativePoseError(e.a_from_b)),
        nullptr, p[e.a].data(), q[e.a].data(), p[e.b].data(), q[e.b].data());
  }
  problem.SetParameterBlockConstant(p[fixed].data());
  problem.SetParameterBlockConstant(q[fixed].data());
  if (edges.empty()) {
    out.usable = true;  // nothing constrains the nodes: the poses stay as they are
    return out;
  }

  ceres::Solver::Options options;
  options.linear_solver_type = ceres::DENSE_NORMAL_CHOLESKY;
  options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
  options.max_num_iterations = kMaxIterations;
  options.num_threads = 1;
  options.logging_type = ceres::SILENT;
  options.minimizer_progress_to_stdout = false;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  out.usable = summary.IsSolutionUsable();
  out.iterations = static_cast<int>(summary.iterations.size());
  out.initial_cost = summary.initial_cost;
  out.final_cost = summary.final_cost;
  if (!out.usable) {
    return out;
  }
  for (size_t i = 0; i < n; ++i) {
    Eigen::Isometry3d iso = Eigen::Isometry3d::Identity();
    iso.linear() = Eigen::Quaterniond(q[i][3], q[i][0], q[i][1], q[i][2]).normalized().toRotationMatrix();
    iso.translation() = Eigen::Vector3d(p[i][0], p[i][1], p[i][2]);
    (*poses)[i] = iso;
  }
  return out;
}

int ApplyKeyframeCorrections(map::WorldMap& wm,
                             const std::vector<std::pair<map::KeyframeId, c8::HMatrix>>& corrected) {
  // world_from_world change of each corrected keyframe, from the poses as stored (float).
  std::map<map::KeyframeId, Eigen::Matrix4d> delta;
  for (const auto& [kf, pose] : corrected) {
    SR_CHECK(wm.IsKeyframeValid(kf), "[pose_graph] corrected keyframe is not valid");
    delta[kf] = ToEigen(pose) * ToEigen(wm.GetKeyframe(kf).frame.extrinsic()).inverse();
  }
  int moved = 0;
  for (const map::PointId pid : wm.AllPointIds()) {
    const map::WorldPoint* p = wm.FindPoint(pid);
    if (p == nullptr || p->state == map::PointState::kDead) {
      continue;
    }
    bool found = false;
    map::KeyframeId ref;
    int64_t ref_creation = 0;
    for (const auto& [kf, obs_idx] : p->observations) {
      if (!wm.IsKeyframeValid(kf)) {
        continue;
      }
      const int64_t creation = wm.GetKeyframe(kf).creation_frame;
      if (!found || creation < ref_creation) {
        found = true;
        ref = kf;
        ref_creation = creation;
      }
    }
    if (!found) {
      continue;
    }
    const auto it = delta.find(ref);
    if (it == delta.end()) {
      continue;
    }
    const auto& raw = p->position.raw();
    const Eigen::Vector3d x(static_cast<double>(raw[0] / raw[3]), static_cast<double>(raw[1] / raw[3]),
                            static_cast<double>(raw[2] / raw[3]));
    const Eigen::Vector3d y = it->second.block<3, 3>(0, 0) * x + it->second.block<3, 1>(0, 3);
    wm.MutablePoint(pid).position =
        c8::HPoint3(static_cast<float>(y.x()), static_cast<float>(y.y()), static_cast<float>(y.z()));
    ++moved;
  }
  for (const auto& [kf, pose] : corrected) {
    wm.SetKeyframePose(kf, pose);  // 655: depths from the moved points, dirty set
  }
  return moved;
}

bool CloseLoopWithPoseGraph(map::WorldMap& wm, map::KeyframeId cand, map::KeyframeId newest,
                            const c8::HMatrix& newest_loop_pose, LoopPoseGraphReport* report) {
  StageTimer timer("r7-pose-graph");
  *report = LoopPoseGraphReport{};
  if (cand == newest) {
    return false;  // a self-loop relates no two keyframes
  }
  const std::vector<map::KeyframeId> all = wm.AllKeyframesByCreation();
  size_t first = all.size();
  size_t last = all.size();
  for (size_t i = 0; i < all.size(); ++i) {
    if (all[i] == cand) first = i;
    if (all[i] == newest) last = i;
  }
  if (first >= all.size() || last >= all.size() || !(first < last)) {
    return false;
  }
  const std::vector<map::KeyframeId> nodes(all.begin() + static_cast<std::ptrdiff_t>(first),
                                           all.begin() + static_cast<std::ptrdiff_t>(last) + 1);
  std::vector<Eigen::Isometry3d> poses;
  poses.reserve(nodes.size());
  for (const map::KeyframeId kf : nodes) {
    poses.push_back(ToIsometry(wm.GetKeyframe(kf).frame.extrinsic()));
  }
  std::vector<PoseGraphEdge> edges;
  for (size_t i = 0; i + 1 < nodes.size(); ++i) {
    edges.push_back({i, i + 1, poses[i].inverse() * poses[i + 1]});  // odometry
  }
  edges.push_back({0, nodes.size() - 1, poses[0].inverse() * ToIsometry(newest_loop_pose)});  // loop
  report->nodes = static_cast<int>(nodes.size());
  report->edges = static_cast<int>(edges.size());
  report->solve = OptimizePoseGraph(&poses, 0, edges);
  if (!report->solve.usable) {
    return false;
  }
  std::vector<std::pair<map::KeyframeId, c8::HMatrix>> corrected;
  for (size_t i = 1; i < nodes.size(); ++i) {
    corrected.emplace_back(nodes[i], ToHMatrix(poses[i]));
  }
  report->moved_points = ApplyKeyframeCorrections(wm, corrected);
  report->applied = true;
  return true;
}

}  // namespace slam_refact::map_builder

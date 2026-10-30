// Accuracy mode (ACC-1): windowed joint bundle adjustment of keyframe poses and triangulated points.
//
// Not part of the original (936 has no joint BA on its default path; architecture.md §1.2 lists the
// public windowed BA keys as unread). The original keeps every keyframe at the tracker pose it was
// created with and places points by averaging per-view depths, so tracking errors accumulate
// open-loop (slam_refact/docs/reports/accuracy.md §2). This step refines the newest keyframes
// jointly with their points after each new keyframe:
//
//   variables  camera_from_world (unit quaternion w,x,y,z + translation) of the newest
//              AccuracyConfig::ba_window keyframes; positions of the TRIANGULATED points they see
//              and (x, z) of the GROUND points they see
//   fixed      every other keyframe that observes one of those points (gauge and scale anchor);
//              when no such keyframe exists the two oldest window keyframes are fixed
//   residuals  ray reprojection error of every valid observation, scaled to pixels by the keyframe
//              focal length, Huber(ba_huber_px); a tilt prior per variable keyframe that keeps the
//              world up direction seen from the camera at the tracker value (the tracker sets the
//              keyframe tilt from the device orientation), sigma ba_tilt_sigma_rad
//
// Ground points take part with their two free coordinates on the original's ground plane y = -1
// (936 [D]), so the keyframe heights stay tied to the floor that defines the responsive scale.
// Sky and depth points are not adjusted.
// The result is applied only when the solver reports a usable solution with a lower cost.

#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <array>
#include <cmath>
#include <map>
#include <set>
#include <utility>
#include <vector>

#include "Eigen/Core"
#include "Eigen/Geometry"
#include "ceres/ceres.h"
#include "ceres/rotation.h"
#include "c8/geometry/egomotion.h"
#include "c8/hmatrix.h"
#include "c8/hpoint.h"
#include "c8/quaternion.h"
#include "common/stage_timer.h"
#include "config/accuracy.h"
#include "map/world_map.h"
#include "map/world_map_internal.h"
#include "map_builder/map_builder.h"

namespace slam_refact::map_builder {
namespace {

struct PoseParam {
  std::array<double, 4> q{1.0, 0.0, 0.0, 0.0};  // camera_from_world rotation, w x y z
  std::array<double, 3> t{0.0, 0.0, 0.0};       // camera_from_world translation
  bool variable = false;
  std::array<double, 3> up_in_camera{0.0, 1.0, 0.0};  // tilt prior target
  float focal = 1.0f;
};

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

// world_from_camera from a camera_from_world parameter block.
c8::HMatrix WorldFromCamera(const PoseParam& p) {
  const Eigen::Quaterniond q_cw(p.q[0], p.q[1], p.q[2], p.q[3]);
  const Eigen::Matrix3d r_wc = q_cw.normalized().toRotationMatrix().transpose();
  const Eigen::Vector3d c = -r_wc * Eigen::Vector3d(p.t[0], p.t[1], p.t[2]);
  const Eigen::Quaterniond q_wc(r_wc);
  return c8::cameraMotion(c8::HPoint3(static_cast<float>(c.x()), static_cast<float>(c.y()),
                                      static_cast<float>(c.z())),
                          c8::Quaternion(static_cast<float>(q_wc.w()), static_cast<float>(q_wc.x()),
                                         static_cast<float>(q_wc.y()), static_cast<float>(q_wc.z())));
}

PoseParam ParamFromWorldFromCamera(const c8::HMatrix& world_from_camera) {
  const Eigen::Matrix4d wc = ToEigen(world_from_camera);
  const Eigen::Matrix3d r_cw = wc.block<3, 3>(0, 0).transpose();
  const Eigen::Vector3d t_cw = -r_cw * wc.block<3, 1>(0, 3);
  const Eigen::Quaterniond q(r_cw);
  PoseParam p;
  p.q = {q.w(), q.x(), q.y(), q.z()};
  p.t = {t_cw.x(), t_cw.y(), t_cw.z()};
  const Eigen::Vector3d up = r_cw * Eigen::Vector3d(0.0, 1.0, 0.0);
  p.up_in_camera = {up.x(), up.y(), up.z()};
  return p;
}

struct RayReprojection {
  RayReprojection(double rx, double ry, double focal) : rx_(rx), ry_(ry), focal_(focal) {}

  template <typename T>
  bool operator()(const T* q_cw, const T* t_cw, const T* x_w, T* r) const {
    T xc[3];
    ceres::QuaternionRotatePoint(q_cw, x_w, xc);
    xc[0] += t_cw[0];
    xc[1] += t_cw[1];
    xc[2] += t_cw[2];
    if (xc[2] <= T(1e-6)) {
      // Behind the camera: a large constant residual keeps the cost finite and the step rejected.
      r[0] = T(focal_);
      r[1] = T(focal_);
      return true;
    }
    r[0] = (xc[0] / xc[2] - T(rx_)) * T(focal_);
    r[1] = (xc[1] / xc[2] - T(ry_)) * T(focal_);
    return true;
  }

  double rx_, ry_, focal_;
};

struct RayReprojectionFixedPose {
  RayReprojectionFixedPose(double rx, double ry, double focal, const PoseParam& pose)
      : inner_(rx, ry, focal), pose_(pose) {}

  template <typename T>
  bool operator()(const T* x_w, T* r) const {
    const T q[4] = {T(pose_.q[0]), T(pose_.q[1]), T(pose_.q[2]), T(pose_.q[3])};
    const T t[3] = {T(pose_.t[0]), T(pose_.t[1]), T(pose_.t[2])};
    return inner_(q, t, x_w, r);
  }

  RayReprojection inner_;
  PoseParam pose_;
};

// Ground point on the plane y = kGroundY of the internal world (the original's ground model,
// 936 [D]): two free coordinates (x, z).
constexpr double kGroundY = -1.0;

struct GroundReprojection {
  GroundReprojection(double rx, double ry, double focal) : inner_(rx, ry, focal) {}

  template <typename T>
  bool operator()(const T* q_cw, const T* t_cw, const T* xz, T* r) const {
    const T x_w[3] = {xz[0], T(kGroundY), xz[1]};
    return inner_(q_cw, t_cw, x_w, r);
  }

  RayReprojection inner_;
};

struct GroundReprojectionFixedPose {
  GroundReprojectionFixedPose(double rx, double ry, double focal, const PoseParam& pose)
      : inner_(rx, ry, focal, pose) {}

  template <typename T>
  bool operator()(const T* xz, T* r) const {
    const T x_w[3] = {xz[0], T(kGroundY), xz[1]};
    return inner_(x_w, r);
  }

  RayReprojectionFixedPose inner_;
};

struct TiltPrior {
  TiltPrior(const std::array<double, 3>& up_in_camera, double weight)
      : up_(up_in_camera), w_(weight) {}

  template <typename T>
  bool operator()(const T* q_cw, T* r) const {
    const T up_w[3] = {T(0.0), T(1.0), T(0.0)};
    T up_c[3];
    ceres::QuaternionRotatePoint(q_cw, up_w, up_c);
    for (int k = 0; k < 3; ++k) {
      r[k] = (up_c[k] - T(up_[static_cast<size_t>(k)])) * T(w_);
    }
    return true;
  }

  std::array<double, 3> up_;
  double w_;
};

ceres::LossFunction* MakeLoss(const AccuracyConfig& acc) {
  if (acc.ba_cauchy) {
    return new ceres::CauchyLoss(acc.ba_huber_px);
  }
  return new ceres::HuberLoss(acc.ba_huber_px);
}

bool ValidKeyframe(const map::WorldMap& wm, map::KeyframeId id) {
  const auto& slots = map::WorldMapAccess::impl(wm).keyframes;
  return id.id() != 0 && id.index() < slots.size() && slots[id.index()].id == id.id();
}

}  // namespace

LocalBaResult LocalBundleAdjustment(map::WorldMap& wm, map::KeyframeId newest) {
  StageTimer timer("accuracy-local-bundle-adjustment");
  const AccuracyConfig& acc = config::Accuracy();
  LocalBaResult result;
  if (!ValidKeyframe(wm, newest)) {
    return result;
  }
  std::vector<map::KeyframeId> by_creation = wm.AllKeyframesByCreation();
  if (by_creation.size() < 3) {
    return result;
  }
  const size_t window = static_cast<size_t>(std::max(2, acc.ba_window));
  const std::vector<map::KeyframeId> win(
      by_creation.end() - static_cast<std::ptrdiff_t>(std::min(window, by_creation.size())),
      by_creation.end());
  std::map<uint32_t, map::KeyframeId> window_ids;
  for (map::KeyframeId k : win) {
    window_ids[k.id()] = k;
  }
  if (window_ids.find(newest.id()) == window_ids.end()) {
    return result;
  }

  // Points: TRIANGULATED, alive, seen by a window keyframe, with >= 2 valid observations.
  std::set<map::PointId> point_ids;
  for (map::KeyframeId k : win) {
    const map::Keyframe& kf = wm.GetKeyframe(k);
    for (size_t idx : kf.frame.worldPointIndices()) {
      const map::PointId pid = kf.frame.points()[idx].id();
      const map::WorldPoint* p = wm.FindPoint(pid);
      if (p == nullptr || p->state == map::PointState::kDead ||
          !(p->source == map::PointSource::kTriangulated ||
            (acc.ba_ground_points != 0 && p->source == map::PointSource::kGround))) {
        continue;
      }
      if (point_ids.count(pid) != 0) {
        continue;
      }
      // Parallax gate: the widest angle between two observing camera centres seen from the point.
      const auto& raw = p->position.raw();
      const Eigen::Vector3d x(raw[0] / raw[3], raw[1] / raw[3], raw[2] / raw[3]);
      std::vector<Eigen::Vector3d> dirs;
      for (const auto& [kf_id, obs_idx] : p->observations) {
        if (!ValidKeyframe(wm, kf_id)) {
          continue;
        }
        const Eigen::Matrix4d wc = ToEigen(wm.GetKeyframe(kf_id).frame.extrinsic());
        dirs.push_back((x - wc.block<3, 1>(0, 3)).normalized());
      }
      double max_angle = 0.0;
      for (size_t i = 0; i < dirs.size(); ++i) {
        for (size_t j = i + 1; j < dirs.size(); ++j) {
          max_angle = std::max(max_angle, std::acos(std::min(1.0, std::max(-1.0, dirs[i].dot(dirs[j])))));
        }
      }
      const bool plane_point = p->source == map::PointSource::kGround && acc.ba_ground_points == 1;
      if (dirs.size() >= 2 &&
          (plane_point || max_angle * 180.0 / 3.14159265358979323846 >= acc.ba_min_parallax_deg)) {
        point_ids.insert(pid);
      }
    }
  }
  if (point_ids.size() < 10) {
    return result;
  }

  // Pose parameters of every keyframe that takes part.
  std::map<uint32_t, PoseParam> poses;
  std::map<uint32_t, map::KeyframeId> pose_kf;
  const auto pose_of = [&](map::KeyframeId k) -> PoseParam& {
    auto it = poses.find(k.id());
    if (it == poses.end()) {
      const map::Keyframe& kf = wm.GetKeyframe(k);
      PoseParam p = ParamFromWorldFromCamera(kf.frame.extrinsic());
      p.variable = window_ids.count(k.id()) != 0;
      p.focal = kf.frame.intrinsic().focalLengthHorizontal;
      it = poses.emplace(k.id(), p).first;
      pose_kf[k.id()] = k;
    }
    return it->second;
  };
  for (map::KeyframeId k : win) {
    pose_of(k);
  }
  for (map::PointId pid : point_ids) {
    for (const auto& [kf_id, obs_idx] : wm.Point(pid).observations) {
      if (ValidKeyframe(wm, kf_id)) {
        pose_of(kf_id);
      }
    }
  }
  int fixed = 0;
  for (const auto& [id, p] : poses) {
    fixed += p.variable ? 0 : 1;
  }
  if (fixed == 0) {
    // Gauge (and monocular scale) anchor: hold the two oldest window keyframes.
    poses[win[0].id()].variable = false;
    poses[win[1].id()].variable = false;
    fixed = 2;
  }

  // TRIANGULATED: (x, y, z); GROUND: (x, z) on the plane, third entry unused.
  std::map<map::PointId, std::array<double, 3>> points;
  std::set<map::PointId> ground_ids;
  for (map::PointId pid : point_ids) {
    const map::WorldPoint& wp = wm.Point(pid);
    const auto& raw = wp.position.raw();
    if (wp.source == map::PointSource::kGround && acc.ba_ground_points == 1) {
      ground_ids.insert(pid);
      points[pid] = {static_cast<double>(raw[0] / raw[3]), static_cast<double>(raw[2] / raw[3]), 0.0};
    } else {
      points[pid] = {static_cast<double>(raw[0] / raw[3]), static_cast<double>(raw[1] / raw[3]),
                     static_cast<double>(raw[2] / raw[3])};
    }
  }

  ceres::QuaternionManifold quaternion_manifold;  // outlives the problem
  ceres::Problem::Options problem_options;
  problem_options.manifold_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
  ceres::Problem problem(problem_options);
  // Ceres owns loss functions (TAKE_OWNERSHIP default): every block gets its own instance.
  int residuals = 0;
  for (auto& [pid, x] : points) {
    for (const auto& [kf_id, obs_idx] : wm.Point(pid).observations) {
      if (!ValidKeyframe(wm, kf_id)) {
        continue;
      }
      PoseParam& pose = poses[kf_id.id()];
      const c8::FramePoint& fp = wm.GetKeyframe(kf_id).frame.points()[obs_idx];
      const double focal = static_cast<double>(pose.focal);
      const bool ground = ground_ids.count(pid) != 0;
      if (ground && pose.variable) {
        problem.AddResidualBlock(
            new ceres::AutoDiffCostFunction<GroundReprojection, 2, 4, 3, 2>(
                new GroundReprojection(fp.x(), fp.y(), focal)),
            MakeLoss(acc), pose.q.data(), pose.t.data(), x.data());
      } else if (ground) {
        problem.AddResidualBlock(
            new ceres::AutoDiffCostFunction<GroundReprojectionFixedPose, 2, 2>(
                new GroundReprojectionFixedPose(fp.x(), fp.y(), focal, pose)),
            MakeLoss(acc), x.data());
      } else if (pose.variable) {
        problem.AddResidualBlock(
            new ceres::AutoDiffCostFunction<RayReprojection, 2, 4, 3, 3>(
                new RayReprojection(fp.x(), fp.y(), focal)),
            MakeLoss(acc), pose.q.data(), pose.t.data(), x.data());
      } else {
        problem.AddResidualBlock(
            new ceres::AutoDiffCostFunction<RayReprojectionFixedPose, 2, 3>(
                new RayReprojectionFixedPose(fp.x(), fp.y(), focal, pose)),
            MakeLoss(acc), x.data());
      }
      ++residuals;
    }
  }
  const double tilt_weight = 1.0 / std::max(1e-4, acc.ba_tilt_sigma_rad);
  int variable_kfs = 0;
  for (auto& [id, p] : poses) {
    if (!p.variable) {
      continue;
    }
    ++variable_kfs;
    problem.SetManifold(p.q.data(), &quaternion_manifold);
    problem.AddResidualBlock(
        new ceres::AutoDiffCostFunction<TiltPrior, 3, 4>(new TiltPrior(p.up_in_camera, tilt_weight)),
        nullptr, p.q.data());
  }
  if (variable_kfs == 0) {
    return result;
  }

  ceres::Solver::Options options;
  options.linear_solver_type = ceres::DENSE_SCHUR;
  options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
  options.max_num_iterations = std::max(1, acc.ba_max_iterations);
  options.max_solver_time_in_seconds = acc.ba_max_seconds;
  options.num_threads = 1;
  options.logging_type = ceres::SILENT;
  options.minimizer_progress_to_stdout = false;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);

  result.keyframes = variable_kfs;
  result.fixed_keyframes = fixed;
  result.points = static_cast<int>(points.size());
  result.residuals = residuals;
  result.initial_cost = summary.initial_cost;
  result.final_cost = summary.final_cost;
  if (!summary.IsSolutionUsable() || !(summary.final_cost < summary.initial_cost)) {
    return result;
  }

  const c8::HMatrix newest_before = wm.GetKeyframe(newest).frame.extrinsic();
  for (const auto& [id, p] : poses) {
    if (p.variable) {
      wm.SetKeyframePose(pose_kf[id], WorldFromCamera(p));  // 655: depths + dirty set
    }
  }
  map::WorldMapAccess::Impl& impl = map::WorldMapAccess::impl(wm);
  for (const auto& [pid, x] : points) {
    if (wm.FindPoint(pid) == nullptr || wm.Point(pid).state == map::PointState::kDead) {
      continue;
    }
    const c8::HPoint3 position =
        ground_ids.count(pid) != 0
            ? c8::HPoint3(static_cast<float>(x[0]), static_cast<float>(kGroundY), static_cast<float>(x[1]))
            : c8::HPoint3(static_cast<float>(x[0]), static_cast<float>(x[1]), static_cast<float>(x[2]));
    wm.SetPointPosition(pid, position);  // 1150
    if (wm.Point(pid).observations.empty()) {
      // 1150 unlinked every observation: the point stays dirty so 1127 retires it. Unlinking the
      // last observation does not dirty a point by itself, and an earlier unlink's entry must
      // not be dropped here.
      // The retirement relies on WorldMap AdjustDirtyPoints, i.e. WorldMap.adjustPoints (default true).
      wm.MarkPointDirty(pid);
    } else {
      impl.dirty.erase(pid);  // the BA position is final for this keyframe
    }
  }
  if (acc.ba_outlier_px > 0.0) {
    // Unlink observations whose reprojection error exceeds ba_outlier_px on the new estimate.
    std::vector<std::pair<map::KeyframeId, uint32_t>> unlink;
    std::set<map::PointId> unlink_points;
    for (const auto& [pid, x] : points) {
      const map::WorldPoint* wp = wm.FindPoint(pid);
      if (wp == nullptr || wp->state == map::PointState::kDead) {
        continue;
      }
      const auto& raw = wp->position.raw();
      const Eigen::Vector3d xw(raw[0] / raw[3], raw[1] / raw[3], raw[2] / raw[3]);
      for (const auto& [kf_id, obs_idx] : wp->observations) {
        if (!ValidKeyframe(wm, kf_id)) {
          continue;
        }
        const map::Keyframe& kf = wm.GetKeyframe(kf_id);
        const Eigen::Matrix4d wc = ToEigen(kf.frame.extrinsic());
        const Eigen::Vector3d xc = wc.block<3, 3>(0, 0).transpose() * (xw - wc.block<3, 1>(0, 3));
        const c8::FramePoint& fp = kf.frame.points()[obs_idx];
        const double f = static_cast<double>(kf.frame.intrinsic().focalLengthHorizontal);
        const double ex = (xc.x() / xc.z() - fp.x()) * f;
        const double ey = (xc.y() / xc.z() - fp.y()) * f;
        if (xc.z() <= 0.0 || ex * ex + ey * ey > acc.ba_outlier_px * acc.ba_outlier_px) {
          unlink.emplace_back(kf_id, obs_idx);
          unlink_points.insert(pid);
        }
      }
    }
    for (const auto& [kf_id, obs_idx] : unlink) {
      wm.AddObservation(map::PointId::nullId(), obs_idx, kf_id);
    }
    for (const map::PointId pid : unlink_points) {
      // Unlinking the last observation does not dirty a point; 1127 must retire it.
      // The retirement relies on WorldMap AdjustDirtyPoints, i.e. WorldMap.adjustPoints (default true).
      if (wm.Point(pid).observations.empty() && wm.Point(pid).state != map::PointState::kDead) {
        wm.MarkPointDirty(pid);
      }
    }
    result.unlinked = static_cast<int>(unlink.size());
  }
  const c8::HMatrix newest_after = wm.GetKeyframe(newest).frame.extrinsic();
  result.newest_correction = newest_after * newest_before.inv();
  result.applied = true;
  if (std::getenv("SLAM_REFACT_BA_LOG") != nullptr) {
    const auto& c = result.newest_correction.data();
    std::fprintf(stderr, "ba kfs=%d fixed=%d pts=%d res=%d cost=%.3g->%.3g iters=%d dt=%.3f\n",
                 variable_kfs, fixed, result.points, residuals, summary.initial_cost,
                 summary.final_cost, static_cast<int>(summary.iterations.size()),
                 std::sqrt(c[12] * c[12] + c[13] * c[13] + c[14] * c[14]));
  }
  return result;
}

}  // namespace slam_refact::map_builder

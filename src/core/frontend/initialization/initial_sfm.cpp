// Copyright 2025 Mobile-SLAM Authors. All rights reserved.
// Licensed under the MIT License.

#include "slam/frontend/initialization/initial_sfm.h"

#include <cassert>
#include <iostream>
#include <vector>

#include <ceres/ceres.h>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/eigen.hpp>

namespace slam {
namespace frontend {

// ---------------------------------------------------------------------------
// Triangulate a single 3-D point from two projection matrices (DLT).
// ---------------------------------------------------------------------------

void InitialSFM::triangulatePoint(Eigen::Matrix<double, 3, 4>& Pose0,
                                  Eigen::Matrix<double, 3, 4>& Pose1,
                                  Eigen::Vector2d& point0,
                                  Eigen::Vector2d& point1,
                                  Eigen::Vector3d& point_3d) {
  Eigen::Matrix4d design_matrix = Eigen::Matrix4d::Zero();
  design_matrix.row(0) = point0[0] * Pose0.row(2) - Pose0.row(0);
  design_matrix.row(1) = point0[1] * Pose0.row(2) - Pose0.row(1);
  design_matrix.row(2) = point1[0] * Pose1.row(2) - Pose1.row(0);
  design_matrix.row(3) = point1[1] * Pose1.row(2) - Pose1.row(1);

  Eigen::Vector4d triangulated_point =
      design_matrix.jacobiSvd(Eigen::ComputeFullV).matrixV().rightCols<1>();

  point_3d(0) = triangulated_point(0) / triangulated_point(3);
  point_3d(1) = triangulated_point(1) / triangulated_point(3);
  point_3d(2) = triangulated_point(2) / triangulated_point(3);
}

// ---------------------------------------------------------------------------
// Solve the camera pose of frame i using PnP.
// ---------------------------------------------------------------------------

bool InitialSFM::solveFrameByPnP(Eigen::Matrix3d& R_initial,
                                  Eigen::Vector3d& P_initial,
                                  int i,
                                  std::vector<SfMFeature>& sfm_f) {
  std::vector<cv::Point2f> pts_2_vector;
  std::vector<cv::Point3f> pts_3_vector;

  for (int j = 0; j < feature_num_; j++) {
    if (!sfm_f[j].state) continue;

    for (int k = 0; k < static_cast<int>(sfm_f[j].observation.size()); k++) {
      if (sfm_f[j].observation[k].first == i) {
        Eigen::Vector2d img_pts = sfm_f[j].observation[k].second;
        pts_2_vector.emplace_back(static_cast<float>(img_pts(0)),
                                  static_cast<float>(img_pts(1)));
        pts_3_vector.emplace_back(static_cast<float>(sfm_f[j].position[0]),
                                  static_cast<float>(sfm_f[j].position[1]),
                                  static_cast<float>(sfm_f[j].position[2]));
        break;
      }
    }
  }

  if (static_cast<int>(pts_2_vector.size()) < 15) {
    std::cout << "unstable features tracking, please slowly move your device!"
              << std::endl;
    if (static_cast<int>(pts_2_vector.size()) < 10) return false;
  }

  cv::Mat r, rvec, t, D, tmp_r;
  cv::eigen2cv(R_initial, tmp_r);
  cv::Rodrigues(tmp_r, rvec);
  cv::eigen2cv(P_initial, t);
  cv::Mat K = (cv::Mat_<double>(3, 3) << 1, 0, 0, 0, 1, 0, 0, 0, 1);

  if (!cv::solvePnP(pts_3_vector, pts_2_vector, K, D, rvec, t, true)) {
    return false;
  }

  cv::Rodrigues(rvec, r);
  Eigen::MatrixXd R_pnp;
  cv::cv2eigen(r, R_pnp);
  Eigen::MatrixXd T_pnp;
  cv::cv2eigen(t, T_pnp);
  R_initial = R_pnp;
  P_initial = T_pnp;
  return true;
}

// ---------------------------------------------------------------------------
// Triangulate all untriangulated features visible in two given frames.
// ---------------------------------------------------------------------------

void InitialSFM::triangulateTwoFrames(int frame0,
                                      Eigen::Matrix<double, 3, 4>& Pose0,
                                      int frame1,
                                      Eigen::Matrix<double, 3, 4>& Pose1,
                                      std::vector<SfMFeature>& sfm_f) {
  assert(frame0 != frame1);
  for (int j = 0; j < feature_num_; j++) {
    if (sfm_f[j].state) continue;

    bool has_0 = false, has_1 = false;
    Eigen::Vector2d point0, point1;

    for (int k = 0; k < static_cast<int>(sfm_f[j].observation.size()); k++) {
      if (sfm_f[j].observation[k].first == frame0) {
        point0 = sfm_f[j].observation[k].second;
        has_0 = true;
      }
      if (sfm_f[j].observation[k].first == frame1) {
        point1 = sfm_f[j].observation[k].second;
        has_1 = true;
      }
    }

    if (has_0 && has_1) {
      Eigen::Vector3d point_3d;
      triangulatePoint(Pose0, Pose1, point0, point1, point_3d);
      sfm_f[j].state = true;
      sfm_f[j].position[0] = point_3d(0);
      sfm_f[j].position[1] = point_3d(1);
      sfm_f[j].position[2] = point_3d(2);
    }
  }
}

// ---------------------------------------------------------------------------
// Full SfM reconstruction with Ceres global bundle adjustment.
// ---------------------------------------------------------------------------

bool InitialSFM::construct(
    int frame_num,
    std::vector<Eigen::Quaterniond>& q,
    std::vector<Eigen::Vector3d>& T,
    int reference_frame_id,
    const Eigen::Matrix3d& relative_R,
    const Eigen::Vector3d& relative_T,
    std::vector<SfMFeature>& sfm_f,
    std::map<int, Eigen::Vector3d>& sfm_tracked_points) {
  const int latest_frame_id = frame_num - 1;
  feature_num_ = static_cast<int>(sfm_f.size());

  q.resize(frame_num);
  T.resize(frame_num);

  // Set reference frame to identity.
  q[reference_frame_id].w() = 1;
  q[reference_frame_id].x() = 0;
  q[reference_frame_id].y() = 0;
  q[reference_frame_id].z() = 0;
  T[reference_frame_id].setZero();

  // Set latest frame from relative pose.
  q[latest_frame_id] = q[reference_frame_id] * Eigen::Quaterniond(relative_R);
  T[latest_frame_id] = relative_T;

  // Per-frame camera poses in camera frame (inverted).
  std::vector<Eigen::Matrix3d> c_Rotation(frame_num);
  std::vector<Eigen::Vector3d> c_Translation(frame_num);
  std::vector<Eigen::Quaterniond> c_Quat(frame_num);
  std::vector<Eigen::Matrix<double, 3, 4>> Pose(frame_num);

  // Allocate arrays for Ceres (Eigen quaternion order: x, y, z, w).
  std::vector<std::array<double, 4>> c_rotation(frame_num);
  std::vector<std::array<double, 3>> c_translation(frame_num);

  auto set_camera_pose = [&](int idx) {
    c_Quat[idx] = q[idx].inverse();
    c_Rotation[idx] = c_Quat[idx].toRotationMatrix();
    c_Translation[idx] = -1.0 * (c_Rotation[idx] * T[idx]);
    Pose[idx].block<3, 3>(0, 0) = c_Rotation[idx];
    Pose[idx].block<3, 1>(0, 3) = c_Translation[idx];
  };

  set_camera_pose(reference_frame_id);
  set_camera_pose(latest_frame_id);

  // 1. Triangulate from reference to latest, solving PnP for in-between.
  int fixed_frame_id = latest_frame_id;
  for (int i = reference_frame_id; i < latest_frame_id; i++) {
    if (i > reference_frame_id) {
      Eigen::Matrix3d R_initial = c_Rotation[i - 1];
      Eigen::Vector3d P_initial = c_Translation[i - 1];
      if (!solveFrameByPnP(R_initial, P_initial, i, sfm_f)) return false;
      c_Rotation[i] = R_initial;
      c_Translation[i] = P_initial;
      c_Quat[i] = c_Rotation[i];
      Pose[i].block<3, 3>(0, 0) = c_Rotation[i];
      Pose[i].block<3, 1>(0, 3) = c_Translation[i];
    }
    triangulateTwoFrames(i, Pose[i], fixed_frame_id, Pose[fixed_frame_id],
                         sfm_f);
  }

  // 2. Triangulate in-between frames using the reference frame.
  fixed_frame_id = reference_frame_id;
  for (int i = reference_frame_id + 1; i < latest_frame_id; i++) {
    triangulateTwoFrames(fixed_frame_id, Pose[fixed_frame_id], i, Pose[i],
                         sfm_f);
  }

  // 3. Solve PnP and triangulate for frames before the reference.
  for (int i = reference_frame_id - 1; i >= 0; i--) {
    Eigen::Matrix3d R_initial = c_Rotation[i + 1];
    Eigen::Vector3d P_initial = c_Translation[i + 1];
    if (!solveFrameByPnP(R_initial, P_initial, i, sfm_f)) return false;
    c_Rotation[i] = R_initial;
    c_Translation[i] = P_initial;
    c_Quat[i] = c_Rotation[i];
    Pose[i].block<3, 3>(0, 0) = c_Rotation[i];
    Pose[i].block<3, 1>(0, 3) = c_Translation[i];
    triangulateTwoFrames(i, Pose[i], fixed_frame_id, Pose[fixed_frame_id],
                         sfm_f);
  }

  // 4. Triangulate all remaining features.
  for (int j = 0; j < feature_num_; j++) {
    if (sfm_f[j].state) continue;
    if (static_cast<int>(sfm_f[j].observation.size()) < 2) continue;

    Eigen::Vector2d point0 = sfm_f[j].observation.front().second;
    int frame_0 = sfm_f[j].observation.front().first;
    Eigen::Vector2d point1 = sfm_f[j].observation.back().second;
    int frame_1 = sfm_f[j].observation.back().first;

    Eigen::Vector3d point_3d;
    triangulatePoint(Pose[frame_0], Pose[frame_1], point0, point1, point_3d);
    sfm_f[j].state = true;
    sfm_f[j].position[0] = point_3d(0);
    sfm_f[j].position[1] = point_3d(1);
    sfm_f[j].position[2] = point_3d(2);
  }

  // -----------------------------------------------------------------------
  // Global bundle adjustment using Ceres.
  // Uses ceres::EigenQuaternionManifold for the quaternion parameterisation
  // (Eigen storage order: x, y, z, w).
  // -----------------------------------------------------------------------
  ceres::Problem problem;
  auto* quaternion_manifold = new ceres::EigenQuaternionManifold();

  for (int i = 0; i < frame_num; i++) {
    // Store in Eigen quaternion order: x, y, z, w.
    c_rotation[i][0] = c_Quat[i].x();
    c_rotation[i][1] = c_Quat[i].y();
    c_rotation[i][2] = c_Quat[i].z();
    c_rotation[i][3] = c_Quat[i].w();

    c_translation[i][0] = c_Translation[i].x();
    c_translation[i][1] = c_Translation[i].y();
    c_translation[i][2] = c_Translation[i].z();

    problem.AddParameterBlock(c_rotation[i].data(), 4, quaternion_manifold);
    problem.AddParameterBlock(c_translation[i].data(), 3);

    if (i == reference_frame_id) {
      problem.SetParameterBlockConstant(c_rotation[i].data());
    }
    if (i == reference_frame_id || i == latest_frame_id) {
      problem.SetParameterBlockConstant(c_translation[i].data());
    }
  }

  for (int i = 0; i < feature_num_; i++) {
    if (!sfm_f[i].state) continue;
    for (int j = 0; j < static_cast<int>(sfm_f[i].observation.size()); j++) {
      int l = sfm_f[i].observation[j].first;
      ceres::CostFunction* cost_function = ReprojectionError3D::Create(
          sfm_f[i].observation[j].second.x(),
          sfm_f[i].observation[j].second.y());

      problem.AddResidualBlock(cost_function, nullptr,
                               c_rotation[l].data(),
                               c_translation[l].data(),
                               sfm_f[i].position);
    }
  }

  ceres::Solver::Options options;
  options.linear_solver_type = ceres::DENSE_SCHUR;
  options.max_solver_time_in_seconds = 0.2;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);

  if (summary.termination_type != ceres::CONVERGENCE &&
      summary.final_cost >= 5e-03) {
    return false;
  }

  // Extract results: convert camera poses back to world-to-body.
  for (int i = 0; i < frame_num; i++) {
    // Eigen quaternion constructor: Quaterniond(w, x, y, z)
    q[i] = Eigen::Quaterniond(c_rotation[i][3], c_rotation[i][0],
                               c_rotation[i][1], c_rotation[i][2]);
    q[i] = q[i].inverse();
  }
  for (int i = 0; i < frame_num; i++) {
    T[i] = -1.0 * (q[i] * Eigen::Vector3d(c_translation[i][0],
                                            c_translation[i][1],
                                            c_translation[i][2]));
  }
  for (int i = 0; i < static_cast<int>(sfm_f.size()); i++) {
    if (sfm_f[i].state) {
      sfm_tracked_points[sfm_f[i].id] =
          Eigen::Vector3d(sfm_f[i].position[0], sfm_f[i].position[1],
                          sfm_f[i].position[2]);
    }
  }

  return true;
}

}  // namespace frontend
}  // namespace slam

// Copyright 2025 Mobile-SLAM Authors. All rights reserved.
// Licensed under the MIT License.

#include "slam/backend/factor/projection_factor.h"

namespace slam {
namespace backend {

// ---------------------------------------------------------------------------
// Static member definitions
// ---------------------------------------------------------------------------
Eigen::Matrix2d ProjectionFactor::sqrt_info;
double ProjectionFactor::sum_t = 0.0;

// ---------------------------------------------------------------------------
// Constructor
// ---------------------------------------------------------------------------
ProjectionFactor::ProjectionFactor(const Eigen::Vector3d& pts_i,
                                   const Eigen::Vector3d& pts_j)
    : pts_i_(pts_i), pts_j_(pts_j) {}

// ---------------------------------------------------------------------------
// SkewSymmetric -- 3x3 skew-symmetric matrix (local helper).
// ---------------------------------------------------------------------------
static Eigen::Matrix3d SkewSymmetric(const Eigen::Vector3d& v) {
  Eigen::Matrix3d m;
  m << 0.0, -v(2), v(1),
       v(2), 0.0, -v(0),
      -v(1), v(0), 0.0;
  return m;
}

// ---------------------------------------------------------------------------
// Evaluate
//
// Geometry:
//   pts_camera_i = pts_i / inv_dep_i        (3D point in host camera frame)
//   pts_imu_i    = R_ic * pts_camera_i + t_ic
//   pts_w        = R_i  * pts_imu_i    + P_i
//   pts_imu_j    = R_j^T * (pts_w - P_j)
//   pts_camera_j = R_ic^T * (pts_imu_j - t_ic)
//
//   residual = [pts_camera_j.x / pts_camera_j.z,
//               pts_camera_j.y / pts_camera_j.z] - pts_j.head<2>()
// ---------------------------------------------------------------------------
bool ProjectionFactor::Evaluate(double const* const* parameters,
                                double* residuals,
                                double** jacobians) const {
  // ------------------------------------------------------------------
  // Extract parameters
  // ------------------------------------------------------------------
  Eigen::Vector3d Pi(parameters[0][0], parameters[0][1], parameters[0][2]);
  Eigen::Quaterniond Qi(parameters[0][6], parameters[0][3],
                        parameters[0][4], parameters[0][5]);

  Eigen::Vector3d Pj(parameters[1][0], parameters[1][1], parameters[1][2]);
  Eigen::Quaterniond Qj(parameters[1][6], parameters[1][3],
                        parameters[1][4], parameters[1][5]);

  Eigen::Vector3d t_ic(parameters[2][0], parameters[2][1], parameters[2][2]);
  Eigen::Quaterniond q_ic(parameters[2][6], parameters[2][3],
                          parameters[2][4], parameters[2][5]);

  double inv_dep_i = parameters[3][0];

  // ------------------------------------------------------------------
  // Forward projection chain
  // ------------------------------------------------------------------
  Eigen::Vector3d pts_camera_i = pts_i_ / inv_dep_i;
  Eigen::Vector3d pts_imu_i = q_ic * pts_camera_i + t_ic;
  Eigen::Vector3d pts_w = Qi * pts_imu_i + Pi;
  Eigen::Vector3d pts_imu_j = Qj.inverse() * (pts_w - Pj);
  Eigen::Vector3d pts_camera_j = q_ic.inverse() * (pts_imu_j - t_ic);

  // ------------------------------------------------------------------
  // Residual: normalized reprojection error
  // ------------------------------------------------------------------
  Eigen::Map<Eigen::Vector2d> residual(residuals);
  double dep_j = pts_camera_j.z();
  residual = (pts_camera_j / dep_j).head<2>() - pts_j_.head<2>();
  residual = sqrt_info * residual;

  // ------------------------------------------------------------------
  // Analytical Jacobians
  // ------------------------------------------------------------------
  if (jacobians) {
    Eigen::Matrix3d Ri = Qi.toRotationMatrix();
    Eigen::Matrix3d Rj = Qj.toRotationMatrix();
    Eigen::Matrix3d r_ic = q_ic.toRotationMatrix();

    // Jacobian of normalized projection w.r.t. 3D point in camera_j
    Eigen::Matrix<double, 2, 3> reduce;
    reduce << 1.0 / dep_j, 0.0, -pts_camera_j(0) / (dep_j * dep_j),
              0.0, 1.0 / dep_j, -pts_camera_j(1) / (dep_j * dep_j);
    reduce = sqrt_info * reduce;

    // ----------------------------------------------------------------
    // Jacobian w.r.t. pose_i (2 x 7, row-major)
    // ----------------------------------------------------------------
    if (jacobians[0]) {
      Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>>
          jacobian_pose_i(jacobians[0]);

      Eigen::Matrix<double, 3, 6> jaco_i;
      // d(pts_camera_j) / d(P_i)
      jaco_i.leftCols<3>() = r_ic.transpose() * Rj.transpose();
      // d(pts_camera_j) / d(theta_i)
      jaco_i.rightCols<3>() =
          r_ic.transpose() * Rj.transpose() * Ri *
          -SkewSymmetric(pts_imu_i);

      jacobian_pose_i.leftCols<6>() = reduce * jaco_i;
      jacobian_pose_i.rightCols<1>().setZero();
    }

    // ----------------------------------------------------------------
    // Jacobian w.r.t. pose_j (2 x 7, row-major)
    // ----------------------------------------------------------------
    if (jacobians[1]) {
      Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>>
          jacobian_pose_j(jacobians[1]);

      Eigen::Matrix<double, 3, 6> jaco_j;
      // d(pts_camera_j) / d(P_j)
      jaco_j.leftCols<3>() = r_ic.transpose() * -Rj.transpose();
      // d(pts_camera_j) / d(theta_j)
      jaco_j.rightCols<3>() =
          r_ic.transpose() * SkewSymmetric(pts_imu_j);

      jacobian_pose_j.leftCols<6>() = reduce * jaco_j;
      jacobian_pose_j.rightCols<1>().setZero();
    }

    // ----------------------------------------------------------------
    // Jacobian w.r.t. extrinsics (2 x 7, row-major)
    // ----------------------------------------------------------------
    if (jacobians[2]) {
      Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>>
          jacobian_ex_pose(jacobians[2]);

      Eigen::Matrix<double, 3, 6> jaco_ex;
      // d(pts_camera_j) / d(t_ic)
      jaco_ex.leftCols<3>() =
          r_ic.transpose() * (Rj.transpose() * Ri - Eigen::Matrix3d::Identity());

      // d(pts_camera_j) / d(theta_ic)
      Eigen::Matrix3d tmp_r = r_ic.transpose() * Rj.transpose() * Ri * r_ic;
      jaco_ex.rightCols<3>() =
          -tmp_r * SkewSymmetric(pts_camera_i) +
          SkewSymmetric(tmp_r * pts_camera_i) +
          SkewSymmetric(r_ic.transpose() *
                        (Rj.transpose() * (Ri * t_ic + Pi - Pj) - t_ic));

      jacobian_ex_pose.leftCols<6>() = reduce * jaco_ex;
      jacobian_ex_pose.rightCols<1>().setZero();
    }

    // ----------------------------------------------------------------
    // Jacobian w.r.t. inverse depth (2 x 1)
    // ----------------------------------------------------------------
    if (jacobians[3]) {
      Eigen::Map<Eigen::Vector2d> jacobian_feature(jacobians[3]);
      jacobian_feature =
          reduce * r_ic.transpose() * Rj.transpose() * Ri * r_ic *
          pts_i_ * -1.0 / (inv_dep_i * inv_dep_i);
    }
  }

  return true;
}

}  // namespace backend
}  // namespace slam

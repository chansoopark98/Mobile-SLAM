// Copyright 2025 Mobile-SLAM Authors. All rights reserved.
// Licensed under the MIT License.

#include "slam/backend/factor/pose_local_parameterization.h"

namespace slam {
namespace backend {

// ---------------------------------------------------------------------------
// Plus -- Apply a 6-DOF local perturbation to the SE(3) state.
// ---------------------------------------------------------------------------
bool PoseLocalParameterization::Plus(const double* x,
                                     const double* delta,
                                     double* x_plus_delta) const {
  Eigen::Map<const Eigen::Vector3d> p(x);
  Eigen::Map<const Eigen::Quaterniond> q(x + 3);

  Eigen::Map<const Eigen::Vector3d> dp(delta);
  Eigen::Map<const Eigen::Vector3d> dtheta(delta + 3);

  // Small-angle quaternion from dtheta: dq = [1, dtheta/2]
  Eigen::Quaterniond dq;
  Eigen::Vector3d half_theta = dtheta * 0.5;
  dq.w() = 1.0;
  dq.x() = half_theta.x();
  dq.y() = half_theta.y();
  dq.z() = half_theta.z();

  Eigen::Map<Eigen::Vector3d> p_plus(x_plus_delta);
  Eigen::Map<Eigen::Quaterniond> q_plus(x_plus_delta + 3);

  p_plus = p + dp;
  q_plus = (q * dq).normalized();

  return true;
}

// ---------------------------------------------------------------------------
// PlusJacobian -- Jacobian of Plus(x, delta) w.r.t. delta at delta = 0.
//
// 7x6 matrix (row-major):
//   [ I_3x3   0_3x3 ]
//   [ 0_3x3   I_3x3 ]
//   [ 0_1x3   0_1x3 ]
// ---------------------------------------------------------------------------
bool PoseLocalParameterization::PlusJacobian(const double* x,
                                             double* jacobian) const {
  Eigen::Map<Eigen::Matrix<double, 7, 6, Eigen::RowMajor>> j(jacobian);
  j.topRows<6>().setIdentity();
  j.bottomRows<1>().setZero();
  return true;
}

// ---------------------------------------------------------------------------
// Minus -- Compute the tangent-space difference y_minus_x.
//
//   dp = p_y - p_x
//   dtheta = 2 * (q_x^{-1} * q_y).vec()  (with sign flip for w < 0)
// ---------------------------------------------------------------------------
bool PoseLocalParameterization::Minus(const double* y,
                                      const double* x,
                                      double* y_minus_x) const {
  Eigen::Map<const Eigen::Vector3d> p_y(y);
  Eigen::Map<const Eigen::Quaterniond> q_y(y + 3);

  Eigen::Map<const Eigen::Vector3d> p_x(x);
  Eigen::Map<const Eigen::Quaterniond> q_x(x + 3);

  Eigen::Map<Eigen::Matrix<double, 6, 1>> delta(y_minus_x);

  delta.head<3>() = p_y - p_x;

  Eigen::Quaterniond dq = q_x.inverse() * q_y;
  if (dq.w() < 0.0) {
    dq.coeffs() = -dq.coeffs();
  }
  delta.tail<3>() = 2.0 * dq.vec();

  return true;
}

// ---------------------------------------------------------------------------
// MinusJacobian -- Jacobian of Minus(y, x) w.r.t. y at y = x.
//
// 6x7 matrix (row-major):
//   [ I_3x3   0_3x3   0_3x1 ]
//   [ 0_3x3   I_3x3   0_3x1 ]
// ---------------------------------------------------------------------------
bool PoseLocalParameterization::MinusJacobian(const double* x,
                                              double* jacobian) const {
  Eigen::Map<Eigen::Matrix<double, 6, 7, Eigen::RowMajor>> j(jacobian);
  j.setZero();
  j.leftCols<6>().setIdentity();
  return true;
}

}  // namespace backend
}  // namespace slam

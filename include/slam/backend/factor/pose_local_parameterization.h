// Copyright 2025 Mobile-SLAM Authors. All rights reserved.
// Licensed under the MIT License.

#ifndef SLAM_BACKEND_FACTOR_POSE_LOCAL_PARAMETERIZATION_H_
#define SLAM_BACKEND_FACTOR_POSE_LOCAL_PARAMETERIZATION_H_

#include <ceres/ceres.h>
#include <Eigen/Dense>

namespace slam {
namespace backend {

// ---------------------------------------------------------------------------
// PoseLocalParameterization -- SE(3) manifold for Ceres 2.2+.
//
// Global representation (7-DOF):  [px, py, pz, qx, qy, qz, qw]
// Local perturbation   (6-DOF):  [dp_x, dp_y, dp_z, dtheta_x, dtheta_y, dtheta_z]
//
// Plus operation:
//   p_plus = p + dp
//   q_plus = q * deltaQ(dtheta)        (right multiplication)
//
// Minus operation:
//   dp = p_y - p_x
//   dtheta = 2 * (q_x^{-1} * q_y).vec()  (ensure positive w)
// ---------------------------------------------------------------------------
class PoseLocalParameterization : public ceres::Manifold {
 public:
  bool Plus(const double* x,
            const double* delta,
            double* x_plus_delta) const override;

  bool PlusJacobian(const double* x, double* jacobian) const override;

  bool Minus(const double* y,
             const double* x,
             double* y_minus_x) const override;

  bool MinusJacobian(const double* x, double* jacobian) const override;

  int AmbientSize() const override { return 7; }
  int TangentSize() const override { return 6; }
};

}  // namespace backend
}  // namespace slam

#endif  // SLAM_BACKEND_FACTOR_POSE_LOCAL_PARAMETERIZATION_H_

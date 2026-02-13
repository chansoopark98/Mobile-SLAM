// Copyright 2025 Mobile-SLAM Authors. All rights reserved.
// Licensed under the MIT License.

#ifndef SLAM_BACKEND_FACTOR_PROJECTION_FACTOR_H_
#define SLAM_BACKEND_FACTOR_PROJECTION_FACTOR_H_

#include <ceres/ceres.h>
#include <Eigen/Dense>

namespace slam {
namespace backend {

// ---------------------------------------------------------------------------
// ProjectionFactor -- Visual reprojection cost function for Ceres.
//
// Residual dimension: 2  (pixel reprojection error)
//
// Parameter blocks (4 total):
//   [0] pose_i       (7): [px, py, pz, qx, qy, qz, qw]  -- host frame
//   [1] pose_j       (7): [px, py, pz, qx, qy, qz, qw]  -- target frame
//   [2] extrinsics   (7): [tx, ty, tz, qx, qy, qz, qw]  -- IMU-camera
//   [3] inv_depth_i  (1): inverse depth of the feature in host camera frame
//
// The factor reprojects a 3D point (parameterized by inverse depth in the
// host frame) into the target frame and computes the 2D error against the
// observed normalized coordinates.
//
// Provides analytical Jacobians for all four parameter blocks.
// ---------------------------------------------------------------------------
class ProjectionFactor : public ceres::SizedCostFunction<2, 7, 7, 7, 1> {
 public:
  /// @brief Construct with observed normalized coordinates in host and target.
  /// @param pts_i  Observation in host frame (normalized plane, z=1 or bearing).
  /// @param pts_j  Observation in target frame (normalized plane, z=1 or bearing).
  ProjectionFactor(const Eigen::Vector3d& pts_i,
                   const Eigen::Vector3d& pts_j);

  /// @brief Evaluate residual and (optionally) analytical Jacobians.
  virtual bool Evaluate(double const* const* parameters,
                        double* residuals,
                        double** jacobians) const override;

  // Observed normalized coordinates
  Eigen::Vector3d pts_i_;
  Eigen::Vector3d pts_j_;

  // Tangent basis for residual computation (unused in this formulation,
  // kept for interface compatibility)
  Eigen::Matrix<double, 2, 3> tangent_base_;

  /// Information matrix (sqrt). Typically focal_length / 1.5 * I_2x2.
  static Eigen::Matrix2d sqrt_info;

  /// Cumulative computation time (for profiling).
  static double sum_t;
};

}  // namespace backend
}  // namespace slam

#endif  // SLAM_BACKEND_FACTOR_PROJECTION_FACTOR_H_

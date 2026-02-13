// Copyright 2025 Mobile-SLAM Authors. All rights reserved.
// Licensed under the MIT License.

#ifndef SLAM_FRONTEND_INITIALIZATION_INITIAL_ALIGNMENT_H_
#define SLAM_FRONTEND_INITIALIZATION_INITIAL_ALIGNMENT_H_

#include <Eigen/Dense>
#include <map>

#include "slam/backend/factor/integration_base.h"
#include "slam/common/image_frame.h"
#include "slam/config/slam_config.h"

namespace slam {
namespace frontend {

/// @brief Solve the gyroscope bias from visual-only rotations and IMU
///        pre-integration rotation measurements.
///
/// Constructs and solves a linear system A * delta_bg = b, then updates
/// the bias in @p Bgs and re-propagates all pre-integrations.
///
/// @param[in,out] all_image_frame  Map of all image frames with
///                                 pre-integration data.
/// @param[in,out] Bgs              Gyroscope biases for each sliding-window
///                                 frame (array of size kWindowSize + 1).
void solveGyroscopeBias(
    std::map<double, common::ImageFrame>& all_image_frame,
    Eigen::Vector3d* Bgs);

/// @brief Solve for per-frame velocities, gravity direction, and scale
///        using the linear formulation from VINS-Mono.
///
/// Reference: "Formula Derivation and Analysis of the VINS-Mono", eq (31).
///
/// @param[in]  all_image_frame  Image frames with visual poses and
///                              pre-integration.
/// @param[out] g                Estimated gravity vector (world frame).
/// @param[out] x                Solution vector [v_0..v_n, g, s].
/// @param[in]  config           SLAM configuration (for gravity magnitude,
///                              extrinsics).
/// @return true if the estimated gravity and scale are physically plausible.
bool LinearAlignment(
    std::map<double, common::ImageFrame>& all_image_frame,
    Eigen::Vector3d& g,
    Eigen::VectorXd& x,
    const config::SlamConfig& config);

/// @brief Refine the gravity direction using a 2-DOF tangent-space
///        parameterisation.
///
/// Reference: "Formula Derivation and Analysis of the VINS-Mono", eq (33).
///
/// @param[in]     all_image_frame  Image frames with visual poses.
/// @param[in,out] g                Gravity vector (refined in-place).
/// @param[out]    x                Updated solution vector.
/// @param[in]     config           SLAM configuration.
void RefineGravity(
    std::map<double, common::ImageFrame>& all_image_frame,
    Eigen::Vector3d& g,
    Eigen::VectorXd& x,
    const config::SlamConfig& config);

/// @brief Top-level visual-inertial alignment: solve gyro bias, then
///        linear alignment (velocity + gravity + scale).
///
/// @param[in,out] all_image_frame  All image frames with pre-integration.
/// @param[in,out] Bgs              Gyroscope biases (updated).
/// @param[out]    g                Estimated gravity vector.
/// @param[out]    x                Solution vector.
/// @param[in]     config           SLAM configuration.
/// @return true on success.
bool VisualIMUAlignment(
    std::map<double, common::ImageFrame>& all_image_frame,
    Eigen::Vector3d* Bgs,
    Eigen::Vector3d& g,
    Eigen::VectorXd& x,
    const config::SlamConfig& config);

}  // namespace frontend
}  // namespace slam

#endif  // SLAM_FRONTEND_INITIALIZATION_INITIAL_ALIGNMENT_H_

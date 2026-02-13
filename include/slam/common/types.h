// Copyright 2025 Mobile-SLAM Authors. All rights reserved.
// Licensed under the MIT License.

#ifndef SLAM_COMMON_TYPES_H_
#define SLAM_COMMON_TYPES_H_

#include <Eigen/Dense>
#include <map>

namespace slam {
namespace common {

// ---------------------------------------------------------------------------
// Solver state flag
// ---------------------------------------------------------------------------
enum class SolverFlag : int {
  kInitial = 0,     // System not yet initialized
  kNonLinear = 1    // System initialized, running nonlinear optimization
};

// ---------------------------------------------------------------------------
// Marginalization strategy flag
// ---------------------------------------------------------------------------
enum class MarginalizationFlag : int {
  kMarginOldKeyframe = 0,       // Marginalize the oldest keyframe
  kMarginNewGeneralFrame = 1    // Marginalize the newest non-keyframe
};

// ---------------------------------------------------------------------------
// Sliding-window and feature-tracker constants
// ---------------------------------------------------------------------------
constexpr int kWindowSize = 10;
constexpr int kNumOfFeatures = 1000;
constexpr double kMinParallax = 10.0;  // pixels

// ---------------------------------------------------------------------------
// Gravity magnitude (m/s^2)
// ---------------------------------------------------------------------------
constexpr double kGravityNorm = 9.81;

// ---------------------------------------------------------------------------
// Eigen type aliases  (kept in slam::common for convenience)
// ---------------------------------------------------------------------------
using Matrix3d = Eigen::Matrix3d;
using Matrix4d = Eigen::Matrix4d;
using Vector3d = Eigen::Vector3d;
using Vector2d = Eigen::Vector2d;
using VectorXd = Eigen::VectorXd;
using MatrixXd = Eigen::MatrixXd;
using Quaterniond = Eigen::Quaterniond;

// ---------------------------------------------------------------------------
// Per-feature observation vector (7-DOF)
//   [ray_x, ray_y, ray_z, pixel_u, pixel_v, vel_x, vel_y]
//
//   ray_{x,y,z}  : normalized bearing vector on the unit plane
//   pixel_{u,v}  : raw pixel coordinates
//   vel_{x,y}    : optical-flow velocity (pixels/s)
// ---------------------------------------------------------------------------
using FeatureObservation = Eigen::Matrix<double, 7, 1>;

// ---------------------------------------------------------------------------
// ImageData: feature_id -> observation vector
//   Stores all feature observations extracted from a single image frame.
// ---------------------------------------------------------------------------
using ImageData = std::map<int, FeatureObservation>;

}  // namespace common
}  // namespace slam

#endif  // SLAM_COMMON_TYPES_H_

// Copyright 2025 Mobile-SLAM Authors. All rights reserved.
// Licensed under the MIT License.

#ifndef SLAM_FRONTEND_INITIALIZATION_SOLVE_5PTS_H_
#define SLAM_FRONTEND_INITIALIZATION_SOLVE_5PTS_H_

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <vector>

#include "slam/frontend/feature_manager.h"

namespace slam {
namespace frontend {

/// @brief Estimates the relative rotation and translation between two frames
///        using the five-point algorithm (via fundamental matrix + cheirality
///        check).
class MotionEstimator {
 public:
  /// @brief Solve for the relative rotation R and translation T between two
  ///        views given 2D-2D feature correspondences.
  ///
  /// Internally uses cv::findFundamentalMat (RANSAC) and a custom recoverPose
  /// implementation with full cheirality check over all four (R,t) combos.
  ///
  /// @param[in]  corres  Feature correspondences (normalised bearing rays).
  /// @param[out] R       Relative rotation (frame1 -> frame0).
  /// @param[out] T       Relative translation (frame1 -> frame0).
  /// @return true if a valid pose with enough inliers (> 12) was found.
  static bool solveRelativeRT(const Correspondences& corres,
                              Eigen::Matrix3d& R,
                              Eigen::Vector3d& T);
};

}  // namespace frontend
}  // namespace slam

#endif  // SLAM_FRONTEND_INITIALIZATION_SOLVE_5PTS_H_

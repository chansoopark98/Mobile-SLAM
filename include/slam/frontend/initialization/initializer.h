// Copyright 2025 Mobile-SLAM Authors. All rights reserved.
// Licensed under the MIT License.

#ifndef SLAM_FRONTEND_INITIALIZATION_INITIALIZER_H_
#define SLAM_FRONTEND_INITIALIZATION_INITIALIZER_H_

#include <Eigen/Dense>
#include <map>

#include "slam/common/image_frame.h"
#include "slam/common/types.h"
#include "slam/config/slam_config.h"
#include "slam/frontend/feature_manager.h"
#include "slam/frontend/initialization/initial_alignment.h"
#include "slam/frontend/initialization/initial_sfm.h"
#include "slam/frontend/initialization/solve_5pts.h"

namespace slam {
namespace frontend {

/// @brief Orchestrates the full VIO initialisation pipeline:
///        1. Check IMU excitation
///        2. Solve initial SfM (relative pose, triangulation, BA)
///        3. Visual-inertial alignment (gyro bias, gravity, scale)
///
/// This class does NOT own any estimator state.  It operates on externally
/// provided data structures (sliding-window poses, image frames, features)
/// and writes back the results.
class Initializer {
 public:
  /// @param config  Reference to the global SLAM configuration.
  explicit Initializer(const config::SlamConfig& config);

  /// @brief Run the full initialisation pipeline.
  ///
  /// @param[in,out] all_image_frame  All buffered image frames.
  /// @param[in,out] Bgs              Gyroscope biases per window frame.
  /// @param[out]    g                Estimated gravity vector.
  /// @param[out]    x                Alignment solution vector.
  /// @param[in,out] Rs               Rotation matrices of the sliding window.
  /// @param[in,out] Ps               Position vectors of the sliding window.
  /// @param[in]     f_manager        Feature manager with current features.
  /// @param[in]     window_size      Current number of frames in the window.
  /// @return true if initialisation succeeds.
  bool initialize(
      std::map<double, common::ImageFrame>& all_image_frame,
      Eigen::Vector3d* Bgs,
      Eigen::Vector3d& g,
      Eigen::VectorXd& x,
      Eigen::Matrix3d* Rs,
      Eigen::Vector3d* Ps,
      FeatureManager& f_manager,
      int window_size);

 private:
  /// @brief Check whether the IMU has experienced sufficient rotational
  ///        excitation for a reliable initialisation.
  bool checkImuExcitation(
      const std::map<double, common::ImageFrame>& all_image_frame,
      double threshold) const;

  /// @brief Find the reference frame with enough parallax to the latest
  ///        frame and solve the relative pose via the five-point algorithm.
  bool relativePose(const FeatureManager& f_manager,
                    int window_size,
                    Eigen::Matrix3d& relative_R,
                    Eigen::Vector3d& relative_T,
                    int& l) const;

  /// @brief Run the initial SfM pipeline: relative pose, incremental PnP,
  ///        triangulation, and global bundle adjustment.
  bool solveInitialSfM(
      std::map<double, common::ImageFrame>& all_image_frame,
      FeatureManager& f_manager,
      int window_size,
      Eigen::Matrix3d* Rs,
      Eigen::Vector3d* Ps);

  /// @brief Perform visual-inertial alignment to recover gyroscope bias,
  ///        gravity direction, velocity, and metric scale.
  bool visualInitialAlign(
      std::map<double, common::ImageFrame>& all_image_frame,
      Eigen::Vector3d* Bgs,
      Eigen::Vector3d& g,
      Eigen::VectorXd& x);

  const config::SlamConfig& config_;
};

}  // namespace frontend
}  // namespace slam

#endif  // SLAM_FRONTEND_INITIALIZATION_INITIALIZER_H_

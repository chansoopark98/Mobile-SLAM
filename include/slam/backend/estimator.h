// Copyright 2025 Mobile-SLAM Authors. All rights reserved.
// Licensed under the MIT License.

#ifndef SLAM_BACKEND_ESTIMATOR_H_
#define SLAM_BACKEND_ESTIMATOR_H_

#include <Eigen/Dense>
#include <map>
#include <memory>
#include <mutex>
#include <vector>

#include "slam/backend/factor/integration_base.h"
#include "slam/backend/factor/marginalization_factor.h"
#include "slam/backend/optimizer.h"
#include "slam/backend/sliding_window.h"
#include "slam/common/image_frame.h"
#include "slam/common/types.h"
#include "slam/config/slam_config.h"
#include "slam/frontend/failure_detector.h"
#include "slam/frontend/feature_manager.h"
#include "slam/frontend/initialization/initializer.h"

namespace slam {
namespace backend {

// ---------------------------------------------------------------------------
// Estimator -- Core orchestrator for the Mobile-SLAM VIO pipeline.
//
// Ties together IMU pre-integration, feature management, visual-inertial
// initialization, nonlinear optimization, and sliding-window marginalization
// into a single coherent processing pipeline.
//
// Usage:
//   1. Feed IMU measurements via processIMU() at IMU rate.
//   2. Feed image features via processImage() at camera rate.
//   3. Query state via getSolverFlag(), getRotation(), getPosition().
// ---------------------------------------------------------------------------
class Estimator {
 public:
  explicit Estimator(const config::SlamConfig& config);
  ~Estimator();

  // Non-copyable
  Estimator(const Estimator&) = delete;
  Estimator& operator=(const Estimator&) = delete;

  // -----------------------------------------------------------------------
  // Main processing pipeline
  // -----------------------------------------------------------------------

  /// @brief Process a single IMU measurement.
  /// @param dt  Time delta since last IMU sample (seconds).
  /// @param linear_acceleration  Accelerometer reading (m/s^2, body frame).
  /// @param angular_velocity     Gyroscope reading (rad/s, body frame).
  void processIMU(double dt, const Eigen::Vector3d& linear_acceleration,
                  const Eigen::Vector3d& angular_velocity);

  /// @brief Process one image frame's feature observations.
  /// @param image   Feature observations (feature_id -> 7-DOF vector).
  /// @param header  Capture timestamp in seconds.
  void processImage(const common::ImageData& image, double header);

  // -----------------------------------------------------------------------
  // State access
  // -----------------------------------------------------------------------

  /// @brief Current solver state (initializing or running).
  common::SolverFlag getSolverFlag() const { return solver_flag_; }

  /// @brief Get the current body-to-world rotation.
  Eigen::Matrix3d getRotation() const;

  /// @brief Get the current body position in world frame.
  Eigen::Vector3d getPosition() const;

  /// @brief Collect 3D map points from the sliding window for visualization.
  std::vector<Eigen::Vector3d> getSlidingWindowMapPoints() const;

 private:
  /// @brief Reset all state to initial values.
  void clearState();

  /// @brief Propagate the IMU state of the current frame using mid-point
  ///        integration.
  void propagateIMU(double dt, const Eigen::Vector3d& acc,
                    const Eigen::Vector3d& gyr);

  /// @brief Run triangulation + optimization + failure check.
  void solveOdometry();

  /// @brief Manage sliding-window shift based on marginalization flag.
  void slideWindow();

  /// @brief Marginalize newest non-keyframe (merge its IMU data into the
  ///        previous keyframe's pre-integration).
  void slideWindowNew();

  /// @brief Marginalize oldest keyframe (shift all frames forward and
  ///        update feature depths).
  void slideWindowOld();

  /// @brief Clean up old image frames and their pre-integration data.
  void cleanupOldImageFrames(double timestamp);

  /// @brief Store the latest pose for failure detection.
  void storeLastPose();

  /// @brief Load extrinsic parameters from config and set projection info.
  void setExtrinsicParameters();

  // -----------------------------------------------------------------------
  // Members
  // -----------------------------------------------------------------------
  const config::SlamConfig& config_;
  common::SolverFlag solver_flag_;
  common::MarginalizationFlag marginalization_flag_;

  SlidingWindow window_;
  Optimizer optimizer_;
  frontend::FeatureManager f_manager_;
  frontend::Initializer initializer_;
  frontend::FailureDetector failure_detector_;

  /// All image frames buffered for initialization.
  std::map<double, common::ImageFrame> all_image_frame_;

  /// Temporary pre-integration for the current inter-frame interval.
  std::shared_ptr<IntegrationBase> tmp_pre_integration_;

  /// Marginalization state from the previous optimization cycle.
  MarginalizationInfo* last_marginalization_info_ = nullptr;
  std::vector<double*> last_marginalization_parameter_blocks_;

  /// Frame counter within the sliding window.
  int frame_count_ = 0;

  /// Whether the first IMU sample has been received.
  bool first_imu_ = false;

  /// Previous IMU measurements for mid-point integration.
  Eigen::Vector3d acc_0_, gyr_0_;

  /// Extrinsic parameters (IMU-to-camera).
  Eigen::Matrix3d ric_;
  Eigen::Vector3d tic_;

  /// Gravity vector (world frame).
  Eigen::Vector3d g_;

  /// Last known pose for failure detection.
  Eigen::Matrix3d last_R_;
  Eigen::Vector3d last_P_;

  /// Initial timestamp for rate-limiting initialization attempts.
  double initial_timestamp_ = 0.0;

  /// Whether the estimator is in a valid state.
  bool is_valid_ = true;

  /// Mutex for thread safety.
  mutable std::mutex estimator_mutex_;
};

}  // namespace backend
}  // namespace slam

#endif  // SLAM_BACKEND_ESTIMATOR_H_

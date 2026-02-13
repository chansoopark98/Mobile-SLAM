// Copyright 2025 Mobile-SLAM Authors. All rights reserved.
// Licensed under the MIT License.

#ifndef SLAM_COMMON_FRAME_H_
#define SLAM_COMMON_FRAME_H_

#include <Eigen/Dense>
#include <memory>
#include <vector>

namespace slam {
namespace backend {
class IntegrationBase;  // forward declaration
}  // namespace backend
}  // namespace slam

namespace slam {
namespace common {

/// @brief Sliding-window frame storing IMU-propagated state and pre-integration.
///
/// Each Frame holds the body-frame pose (R, P), velocity (V), and IMU biases
/// (Ba, Bg) at the frame's timestamp.  The raw IMU measurements between this
/// frame and the next are buffered in dt_buf / linear_acceleration_buf /
/// angular_velocity_buf so that IntegrationBase can be re-propagated when
/// biases are updated.
class Frame {
 public:
  /// @brief Default constructor -- identity pose, zero state.
  Frame();

  /// @brief Destructor (default, but declared because of unique_ptr member).
  ~Frame();

  // -----------------------------------------------------------------------
  // Move semantics (unique_ptr member prevents implicit copy)
  // -----------------------------------------------------------------------
  Frame(Frame&& other) noexcept;
  Frame& operator=(Frame&& other) noexcept;

  // Copy is explicitly deleted (unique_ptr<IntegrationBase>)
  Frame(const Frame&) = delete;
  Frame& operator=(const Frame&) = delete;

  /// @brief Reset all members to initial state.
  void Reset();

  // -----------------------------------------------------------------------
  // State variables
  // -----------------------------------------------------------------------

  /// Timestamp in seconds
  double timestamp_;

  /// Rotation from body to world frame
  Eigen::Matrix3d R_;

  /// Position in world frame (meters)
  Eigen::Vector3d P_;

  /// Velocity in world frame (m/s)
  Eigen::Vector3d V_;

  /// Accelerometer bias (m/s^2)
  Eigen::Vector3d Ba_;

  /// Gyroscope bias (rad/s)
  Eigen::Vector3d Bg_;

  /// IMU pre-integration from previous frame to this frame
  std::unique_ptr<backend::IntegrationBase> pre_integration_;

  // -----------------------------------------------------------------------
  // Raw IMU measurement buffers (between previous frame and this frame)
  // -----------------------------------------------------------------------

  /// Time deltas between consecutive IMU samples (seconds)
  std::vector<double> dt_buf_;

  /// Linear accelerations (m/s^2, body frame)
  std::vector<Eigen::Vector3d> linear_acceleration_buf_;

  /// Angular velocities (rad/s, body frame)
  std::vector<Eigen::Vector3d> angular_velocity_buf_;
};

}  // namespace common
}  // namespace slam

#endif  // SLAM_COMMON_FRAME_H_

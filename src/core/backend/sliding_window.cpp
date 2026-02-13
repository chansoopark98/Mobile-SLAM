// Copyright 2025 Mobile-SLAM Authors. All rights reserved.
// Licensed under the MIT License.

#include "slam/backend/sliding_window.h"

#include <cmath>
#include <memory>

#include "slam/utility/math_utils.h"

namespace slam {
namespace backend {

// ===========================================================================
// Constructor
// ===========================================================================
SlidingWindow::SlidingWindow() {
  clearSlidingWindow();
}

// ===========================================================================
// clearSlidingWindow
// ===========================================================================
void SlidingWindow::clearSlidingWindow() {
  for (int i = 0; i <= common::kWindowSize; ++i) {
    frames_[i].Reset();
  }
}

// ===========================================================================
// clearBuffer
// ===========================================================================
void SlidingWindow::clearBuffer(int32_t index) {
  frames_[index].dt_buf_.clear();
  frames_[index].linear_acceleration_buf_.clear();
  frames_[index].angular_velocity_buf_.clear();
}

// ===========================================================================
// copyFrame -- copies state variables only (no pre-integration, no buffers)
// ===========================================================================
void SlidingWindow::copyFrame(int32_t dst, int32_t src) {
  frames_[dst].timestamp_ = frames_[src].timestamp_;
  frames_[dst].R_ = frames_[src].R_;
  frames_[dst].P_ = frames_[src].P_;
  frames_[dst].V_ = frames_[src].V_;
  frames_[dst].Ba_ = frames_[src].Ba_;
  frames_[dst].Bg_ = frames_[src].Bg_;
}

// ===========================================================================
// swapFrame -- swaps state + pre-integration ownership
// ===========================================================================
void SlidingWindow::swapFrame(int32_t a, int32_t b) {
  std::swap(frames_[a].timestamp_, frames_[b].timestamp_);
  std::swap(frames_[a].R_, frames_[b].R_);
  std::swap(frames_[a].P_, frames_[b].P_);
  std::swap(frames_[a].V_, frames_[b].V_);
  std::swap(frames_[a].Ba_, frames_[b].Ba_);
  std::swap(frames_[a].Bg_, frames_[b].Bg_);
  std::swap(frames_[a].pre_integration_, frames_[b].pre_integration_);
}

// ===========================================================================
// swapBuffer -- swaps only IMU measurement buffers
// ===========================================================================
void SlidingWindow::swapBuffer(int32_t a, int32_t b) {
  std::swap(frames_[a].dt_buf_, frames_[b].dt_buf_);
  std::swap(frames_[a].linear_acceleration_buf_,
            frames_[b].linear_acceleration_buf_);
  std::swap(frames_[a].angular_velocity_buf_,
            frames_[b].angular_velocity_buf_);
}

// ===========================================================================
// pushBackBuffer -- append raw IMU sample to frame's buffers
// ===========================================================================
void SlidingWindow::pushBackBuffer(int32_t index, double dt,
                                   const Eigen::Vector3d& linear_acceleration,
                                   const Eigen::Vector3d& angular_velocity) {
  frames_[index].dt_buf_.push_back(dt);
  frames_[index].linear_acceleration_buf_.push_back(linear_acceleration);
  frames_[index].angular_velocity_buf_.push_back(angular_velocity);
}

// ===========================================================================
// pushBackPreintegration -- propagate one IMU measurement
// ===========================================================================
void SlidingWindow::pushBackPreintegration(
    int32_t index, double dt,
    const Eigen::Vector3d& linear_acceleration,
    const Eigen::Vector3d& angular_velocity) {
  frames_[index].pre_integration_->push_back(dt, linear_acceleration,
                                             angular_velocity);
}

// ===========================================================================
// createNewPreintegration -- fresh IntegrationBase with given biases
// ===========================================================================
void SlidingWindow::createNewPreintegration(
    int32_t index,
    const Eigen::Vector3d& linear_acceleration,
    const Eigen::Vector3d& angular_velocity,
    const Eigen::Vector3d& ba,
    const Eigen::Vector3d& bg,
    const config::SlamConfig& config) {
  frames_[index].pre_integration_ = std::make_unique<IntegrationBase>(
      linear_acceleration, angular_velocity, ba, bg, config);
}

// ===========================================================================
// stateToParameter -- Eigen state -> raw double arrays for Ceres
// ===========================================================================
void SlidingWindow::stateToParameter(const Eigen::Matrix3d& r_ic,
                                     const Eigen::Vector3d& t_ic,
                                     const Eigen::VectorXd& feature_depths,
                                     int num_features) {
  for (int i = 0; i <= common::kWindowSize; ++i) {
    // Position
    para_pose_[i][0] = frames_[i].P_.x();
    para_pose_[i][1] = frames_[i].P_.y();
    para_pose_[i][2] = frames_[i].P_.z();

    // Rotation (quaternion: [qx, qy, qz, qw])
    Eigen::Quaterniond q{frames_[i].R_};
    para_pose_[i][3] = q.x();
    para_pose_[i][4] = q.y();
    para_pose_[i][5] = q.z();
    para_pose_[i][6] = q.w();

    // Velocity
    para_speed_bias_[i][0] = frames_[i].V_.x();
    para_speed_bias_[i][1] = frames_[i].V_.y();
    para_speed_bias_[i][2] = frames_[i].V_.z();

    // Accelerometer bias
    para_speed_bias_[i][3] = frames_[i].Ba_.x();
    para_speed_bias_[i][4] = frames_[i].Ba_.y();
    para_speed_bias_[i][5] = frames_[i].Ba_.z();

    // Gyroscope bias
    para_speed_bias_[i][6] = frames_[i].Bg_.x();
    para_speed_bias_[i][7] = frames_[i].Bg_.y();
    para_speed_bias_[i][8] = frames_[i].Bg_.z();
  }

  // Camera-IMU extrinsic
  para_ex_pose_[0][0] = t_ic.x();
  para_ex_pose_[0][1] = t_ic.y();
  para_ex_pose_[0][2] = t_ic.z();

  Eigen::Quaterniond q_ic{r_ic};
  para_ex_pose_[0][3] = q_ic.x();
  para_ex_pose_[0][4] = q_ic.y();
  para_ex_pose_[0][5] = q_ic.z();
  para_ex_pose_[0][6] = q_ic.w();

  // Feature inverse depths
  for (int i = 0; i < num_features; ++i) {
    para_feature_[i][0] = feature_depths(i);
  }
}

// ===========================================================================
// parameterToState -- raw double arrays -> Eigen state with yaw correction
// ===========================================================================
void SlidingWindow::parameterToState(Eigen::Matrix3d& r_ic,
                                     Eigen::Vector3d& t_ic,
                                     Eigen::VectorXd& feature_depths,
                                     int num_features) {
  // Yaw correction: preserve the first frame's original yaw to avoid drift
  // in the unobservable global yaw direction.
  Eigen::Vector3d origin_r0 = utility::YawPitchRoll(frames_[0].R_);
  Eigen::Vector3d origin_p0 = frames_[0].P_;

  Eigen::Vector3d origin_r00 = utility::YawPitchRoll(
      Eigen::Quaterniond(para_pose_[0][6], para_pose_[0][3],
                         para_pose_[0][4], para_pose_[0][5])
          .toRotationMatrix());

  double y_diff = origin_r0.x() - origin_r00.x();
  Eigen::Matrix3d rot_diff = utility::RpyToRotation(
      Eigen::Vector3d(y_diff, 0.0, 0.0));

  // Handle gimbal-lock near pitch = +/-90 degrees
  if (std::abs(std::abs(origin_r0.y()) - 90.0) < 1.0 ||
      std::abs(std::abs(origin_r00.y()) - 90.0) < 1.0) {
    rot_diff = frames_[0].R_ *
               Eigen::Quaterniond(para_pose_[0][6], para_pose_[0][3],
                                  para_pose_[0][4], para_pose_[0][5])
                   .toRotationMatrix()
                   .transpose();
  }

  for (int i = 0; i <= common::kWindowSize; ++i) {
    frames_[i].R_ =
        rot_diff *
        Eigen::Quaterniond(para_pose_[i][6], para_pose_[i][3],
                           para_pose_[i][4], para_pose_[i][5])
            .normalized()
            .toRotationMatrix();

    frames_[i].P_ =
        rot_diff *
            Eigen::Vector3d(para_pose_[i][0] - para_pose_[0][0],
                            para_pose_[i][1] - para_pose_[0][1],
                            para_pose_[i][2] - para_pose_[0][2]) +
        origin_p0;

    frames_[i].V_ =
        rot_diff *
        Eigen::Vector3d(para_speed_bias_[i][0], para_speed_bias_[i][1],
                        para_speed_bias_[i][2]);

    frames_[i].Ba_ = Eigen::Vector3d(para_speed_bias_[i][3],
                                     para_speed_bias_[i][4],
                                     para_speed_bias_[i][5]);

    frames_[i].Bg_ = Eigen::Vector3d(para_speed_bias_[i][6],
                                     para_speed_bias_[i][7],
                                     para_speed_bias_[i][8]);
  }

  // Update extrinsic parameters
  t_ic = Eigen::Vector3d(para_ex_pose_[0][0], para_ex_pose_[0][1],
                         para_ex_pose_[0][2]);
  r_ic = Eigen::Quaterniond(para_ex_pose_[0][6], para_ex_pose_[0][3],
                            para_ex_pose_[0][4], para_ex_pose_[0][5])
             .toRotationMatrix();

  // Update feature depths
  for (int i = 0; i < num_features; ++i) {
    feature_depths(i) = para_feature_[i][0];
  }
}

}  // namespace backend
}  // namespace slam

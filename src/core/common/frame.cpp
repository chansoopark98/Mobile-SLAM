// Copyright 2025 Mobile-SLAM Authors. All rights reserved.
// Licensed under the MIT License.

#include "slam/common/frame.h"

#include "slam/backend/factor/integration_base.h"

namespace slam {
namespace common {

// -----------------------------------------------------------------------
// Constructor
// -----------------------------------------------------------------------
Frame::Frame()
    : timestamp_(0.0),
      R_(Eigen::Matrix3d::Identity()),
      P_(Eigen::Vector3d::Zero()),
      V_(Eigen::Vector3d::Zero()),
      Ba_(Eigen::Vector3d::Zero()),
      Bg_(Eigen::Vector3d::Zero()),
      pre_integration_(nullptr) {}

// -----------------------------------------------------------------------
// Destructor  (must be defined in .cpp where IntegrationBase is complete
//              for unique_ptr destruction -- currently forward-declared,
//              so defaulting here is safe once the full type is available)
// -----------------------------------------------------------------------
Frame::~Frame() = default;

// -----------------------------------------------------------------------
// Move constructor
// -----------------------------------------------------------------------
Frame::Frame(Frame&& other) noexcept
    : timestamp_(other.timestamp_),
      R_(other.R_),
      P_(other.P_),
      V_(other.V_),
      Ba_(other.Ba_),
      Bg_(other.Bg_),
      pre_integration_(std::move(other.pre_integration_)),
      dt_buf_(std::move(other.dt_buf_)),
      linear_acceleration_buf_(std::move(other.linear_acceleration_buf_)),
      angular_velocity_buf_(std::move(other.angular_velocity_buf_)) {}

// -----------------------------------------------------------------------
// Move assignment
// -----------------------------------------------------------------------
Frame& Frame::operator=(Frame&& other) noexcept {
  if (this != &other) {
    timestamp_ = other.timestamp_;
    R_ = other.R_;
    P_ = other.P_;
    V_ = other.V_;
    Ba_ = other.Ba_;
    Bg_ = other.Bg_;
    pre_integration_ = std::move(other.pre_integration_);
    dt_buf_ = std::move(other.dt_buf_);
    linear_acceleration_buf_ = std::move(other.linear_acceleration_buf_);
    angular_velocity_buf_ = std::move(other.angular_velocity_buf_);
  }
  return *this;
}

// -----------------------------------------------------------------------
// Reset
// -----------------------------------------------------------------------
void Frame::Reset() {
  timestamp_ = 0.0;
  R_ = Eigen::Matrix3d::Identity();
  P_ = Eigen::Vector3d::Zero();
  V_ = Eigen::Vector3d::Zero();
  Ba_ = Eigen::Vector3d::Zero();
  Bg_ = Eigen::Vector3d::Zero();
  pre_integration_.reset();

  dt_buf_.clear();
  linear_acceleration_buf_.clear();
  angular_velocity_buf_.clear();
}

}  // namespace common
}  // namespace slam

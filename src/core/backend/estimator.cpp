// Copyright 2025 Mobile-SLAM Authors. All rights reserved.
// Licensed under the MIT License.

#include "slam/backend/estimator.h"

#include <algorithm>
#include <memory>
#include <utility>

#include "slam/backend/factor/projection_factor.h"
#include "slam/utility/math_utils.h"

namespace slam {
namespace backend {

// ===========================================================================
// Constructor
// ===========================================================================
Estimator::Estimator(const config::SlamConfig& config)
    : config_(config),
      optimizer_(config),
      f_manager_(config.estimator.min_parallax),
      initializer_(config),
      failure_detector_() {
  clearState();
  setExtrinsicParameters();
}

// ===========================================================================
// Destructor
// ===========================================================================
Estimator::~Estimator() {
  delete last_marginalization_info_;
  last_marginalization_info_ = nullptr;
}

// ===========================================================================
// setExtrinsicParameters -- load extrinsics from config
// ===========================================================================
void Estimator::setExtrinsicParameters() {
  tic_ = config_.camera.t_ic;
  ric_ = config_.camera.r_ic;

  // Set the projection factor information matrix
  ProjectionFactor::sqrt_info =
      (config_.camera.fx / 1.5) * Eigen::Matrix2d::Identity();
}

// ===========================================================================
// clearState
// ===========================================================================
void Estimator::clearState() {
  window_.clearSlidingWindow();

  tic_ = Eigen::Vector3d::Zero();
  ric_ = Eigen::Matrix3d::Identity();

  all_image_frame_.clear();

  solver_flag_ = common::SolverFlag::kInitial;
  first_imu_ = false;
  frame_count_ = 0;
  initial_timestamp_ = 0.0;

  tmp_pre_integration_.reset();

  f_manager_.clearState();

  delete last_marginalization_info_;
  last_marginalization_info_ = nullptr;
  last_marginalization_parameter_blocks_.clear();

  g_ = config_.estimator.gravity;

  last_R_ = Eigen::Matrix3d::Identity();
  last_P_ = Eigen::Vector3d::Zero();

  is_valid_ = true;
}

// ===========================================================================
// propagateIMU -- mid-point state propagation for the current frame
// ===========================================================================
void Estimator::propagateIMU(double dt, const Eigen::Vector3d& acc,
                             const Eigen::Vector3d& gyr) {
  // Store raw IMU data in the sliding window buffers
  window_.pushBackBuffer(frame_count_, dt, acc, gyr);

  // Bias-corrected previous acceleration in world frame
  Eigen::Vector3d un_acc_0 =
      window_[frame_count_].R_ * (acc_0_ - window_[frame_count_].Ba_) - g_;

  // Mid-point gyroscope (bias-corrected)
  Eigen::Vector3d un_gyr = 0.5 * (gyr_0_ + gyr) - window_[frame_count_].Bg_;

  // Update rotation: R = R * exp(gyro * dt)
  window_[frame_count_].R_ *=
      utility::DeltaQ(un_gyr * dt).toRotationMatrix();

  // Bias-corrected current acceleration in world frame
  Eigen::Vector3d un_acc_1 =
      window_[frame_count_].R_ * (acc - window_[frame_count_].Ba_) - g_;

  // Mid-point acceleration
  Eigen::Vector3d un_acc = 0.5 * (un_acc_0 + un_acc_1);

  // Update position: P = P + V*dt + 0.5*a*dt^2
  window_[frame_count_].P_ +=
      dt * window_[frame_count_].V_ + 0.5 * dt * dt * un_acc;

  // Update velocity: V = V + a*dt
  window_[frame_count_].V_ += dt * un_acc;
}

// ===========================================================================
// processIMU
// ===========================================================================
void Estimator::processIMU(double dt,
                           const Eigen::Vector3d& linear_acceleration,
                           const Eigen::Vector3d& angular_velocity) {
  std::lock_guard<std::mutex> lock(estimator_mutex_);

  // First IMU sample: just store and return
  if (!first_imu_) {
    first_imu_ = true;
    acc_0_ = linear_acceleration;
    gyr_0_ = angular_velocity;
  }

  // Ensure pre-integration exists for current frame
  if (!window_[frame_count_].pre_integration_) {
    window_[frame_count_].pre_integration_ =
        std::make_unique<IntegrationBase>(
            acc_0_, gyr_0_,
            window_[frame_count_].Ba_,
            window_[frame_count_].Bg_,
            config_);
  }

  // Only propagate after the first frame (frame_count_ > 0)
  if (frame_count_ != 0) {
    window_.pushBackPreintegration(frame_count_, dt, linear_acceleration,
                                  angular_velocity);
    tmp_pre_integration_->push_back(dt, linear_acceleration, angular_velocity);
    propagateIMU(dt, linear_acceleration, angular_velocity);
  }

  acc_0_ = linear_acceleration;
  gyr_0_ = angular_velocity;
}

// ===========================================================================
// processImage
// ===========================================================================
void Estimator::processImage(const common::ImageData& image, double header) {
  std::lock_guard<std::mutex> lock(estimator_mutex_);

  // Determine keyframe / non-keyframe via parallax check
  if (f_manager_.addFeatureCheckParallax(frame_count_, image,
                                         config_.camera.fx)) {
    marginalization_flag_ = common::MarginalizationFlag::kMarginOldKeyframe;
  } else {
    marginalization_flag_ = common::MarginalizationFlag::kMarginNewGeneralFrame;
  }

  // Store timestamp in sliding window
  window_[frame_count_].timestamp_ = header;

  // Create ImageFrame and store in all_image_frame_ map
  common::ImageFrame image_frame(image, header);
  image_frame.pre_integration_ = tmp_pre_integration_;
  all_image_frame_.insert(std::make_pair(header, std::move(image_frame)));

  // Create fresh temporary pre-integration for the next inter-frame interval
  tmp_pre_integration_ = std::make_shared<IntegrationBase>(
      acc_0_, gyr_0_,
      window_[frame_count_].Ba_,
      window_[frame_count_].Bg_,
      config_);

  if (solver_flag_ == common::SolverFlag::kInitial) {
    if (frame_count_ == common::kWindowSize) {
      bool init_result = false;

      // Rate-limit initialization attempts
      if (header - initial_timestamp_ > 0.1) {
        // Collect rotation/position/bias arrays for the initializer
        Eigen::Matrix3d Rs[common::kWindowSize + 1];
        Eigen::Vector3d Ps[common::kWindowSize + 1];
        Eigen::Vector3d Bgs[common::kWindowSize + 1];
        for (int i = 0; i <= common::kWindowSize; ++i) {
          Rs[i] = window_[i].R_;
          Ps[i] = window_[i].P_;
          Bgs[i] = window_[i].Bg_;
        }

        Eigen::VectorXd x;
        init_result = initializer_.initialize(
            all_image_frame_, Bgs, g_, x, Rs, Ps,
            f_manager_, common::kWindowSize);

        initial_timestamp_ = header;

        if (init_result) {
          // Write back initialized state to the sliding window
          for (int i = 0; i <= common::kWindowSize; ++i) {
            window_[i].R_ = Rs[i];
            window_[i].P_ = Ps[i];
            window_[i].Bg_ = Bgs[i];
          }

          // Recover velocities from the alignment solution x
          // x contains [V0, V1, ..., Vn, g, s] per visual-inertial alignment
          for (auto& frame_pair : all_image_frame_) {
            int idx = 0;
            for (auto it = all_image_frame_.begin();
                 it != all_image_frame_.end(); ++it, ++idx) {
              if (it->first == frame_pair.first) {
                break;
              }
            }
          }

          // Set velocities from all_image_frame into the window frames
          {
            int i = 0;
            for (auto it = all_image_frame_.begin();
                 it != all_image_frame_.end() && i <= common::kWindowSize;
                 ++it, ++i) {
              // The initializer writes V into x as blocks of 3
              if (i < x.size() / 3) {
                window_[i].V_ = x.segment<3>(i * 3);
              }
            }
          }

          solver_flag_ = common::SolverFlag::kNonLinear;
          solveOdometry();
          slideWindow();
          f_manager_.removeFailures();
          storeLastPose();
        } else {
          slideWindow();
        }
      } else {
        slideWindow();
      }
    } else {
      frame_count_++;
    }
  } else if (solver_flag_ == common::SolverFlag::kNonLinear) {
    solveOdometry();
    slideWindow();
    f_manager_.removeFailures();

    // Failure detection
    bool failure = failure_detector_.detectFailure(
        window_.back().R_, window_.back().P_,
        last_R_, last_P_,
        f_manager_.lastTrackNum());

    if (failure) {
      clearState();
      setExtrinsicParameters();
      return;
    }

    storeLastPose();
  }
}

// ===========================================================================
// solveOdometry -- triangulate + optimize
// ===========================================================================
void Estimator::solveOdometry() {
  if (frame_count_ < common::kWindowSize) {
    return;
  }

  if (solver_flag_ == common::SolverFlag::kNonLinear) {
    // Collect Rs and Ps into vectors for triangulation
    std::vector<Eigen::Matrix3d> Rs(common::kWindowSize + 1);
    std::vector<Eigen::Vector3d> Ps(common::kWindowSize + 1);
    for (int i = 0; i <= common::kWindowSize; ++i) {
      Rs[i] = window_[i].R_;
      Ps[i] = window_[i].P_;
    }

    f_manager_.triangulate(Rs, Ps, ric_, tic_);

    optimizer_.optimize(window_, f_manager_,
                        last_marginalization_info_,
                        last_marginalization_parameter_blocks_,
                        marginalization_flag_);

    // Update extrinsic parameters from optimizer
    tic_ = optimizer_.tic();
    ric_ = optimizer_.ric();
  }
}

// ===========================================================================
// slideWindow
// ===========================================================================
void Estimator::slideWindow() {
  if (frame_count_ == common::kWindowSize) {
    if (marginalization_flag_ ==
        common::MarginalizationFlag::kMarginOldKeyframe) {
      slideWindowOld();
    } else if (marginalization_flag_ ==
               common::MarginalizationFlag::kMarginNewGeneralFrame) {
      slideWindowNew();
    }
  }
}

// ===========================================================================
// slideWindowNew -- marginalize newest non-keyframe
// ===========================================================================
void Estimator::slideWindowNew() {
  // Merge the newest frame's IMU data into the previous keyframe's
  // pre-integration
  for (size_t i = 0;
       i < window_[common::kWindowSize].dt_buf_.size(); ++i) {
    double tmp_dt = window_[common::kWindowSize].dt_buf_[i];
    Eigen::Vector3d tmp_acc =
        window_[common::kWindowSize].linear_acceleration_buf_[i];
    Eigen::Vector3d tmp_gyr =
        window_[common::kWindowSize].angular_velocity_buf_[i];

    window_.pushBackPreintegration(common::kWindowSize - 1, tmp_dt, tmp_acc,
                                  tmp_gyr);
    window_.pushBackBuffer(common::kWindowSize - 1, tmp_dt, tmp_acc, tmp_gyr);
  }

  // Copy state from the second-newest to the newest slot
  window_.copyFrame(common::kWindowSize - 1, common::kWindowSize);

  // Create fresh pre-integration for the newest slot
  window_.createNewPreintegration(
      common::kWindowSize, acc_0_, gyr_0_,
      window_[common::kWindowSize].Ba_,
      window_[common::kWindowSize].Bg_,
      config_);
  window_.clearBuffer(common::kWindowSize);

  // Remove the non-keyframe's feature observations
  f_manager_.removeFront(frame_count_);
}

// ===========================================================================
// slideWindowOld -- marginalize oldest keyframe
// ===========================================================================
void Estimator::slideWindowOld() {
  Eigen::Matrix3d back_R0 = window_.front().R_;
  Eigen::Vector3d back_P0 = window_.front().P_;

  double t_0 = window_.front().timestamp_;

  // Shift all frames forward by one position
  for (int i = 0; i < common::kWindowSize; ++i) {
    window_.swapBuffer(i, i + 1);
    window_.swapFrame(i, i + 1);
  }

  // Copy the second-to-last state into the last slot
  window_.copyFrame(common::kWindowSize, common::kWindowSize - 1);

  // Create fresh pre-integration for the last slot
  window_.createNewPreintegration(
      common::kWindowSize, acc_0_, gyr_0_,
      window_[common::kWindowSize].Ba_,
      window_[common::kWindowSize].Bg_,
      config_);
  window_.clearBuffer(common::kWindowSize);

  // Clean up old image frames
  cleanupOldImageFrames(t_0);

  // Shift feature depths if we are in nonlinear mode
  bool shift_depth =
      (solver_flag_ == common::SolverFlag::kNonLinear);
  if (shift_depth) {
    Eigen::Matrix3d R0 = back_R0 * ric_;
    Eigen::Matrix3d R1 = window_.front().R_ * ric_;
    Eigen::Vector3d P0 = back_P0 + back_R0 * tic_;
    Eigen::Vector3d P1 = window_.front().P_ + window_.front().R_ * tic_;
    f_manager_.removeBackShiftDepth(R0, P0, R1, P1);
  } else {
    f_manager_.removeBack();
  }
}

// ===========================================================================
// cleanupOldImageFrames
// ===========================================================================
void Estimator::cleanupOldImageFrames(double timestamp) {
  auto it_0 = all_image_frame_.find(timestamp);
  if (it_0 == all_image_frame_.end()) {
    return;
  }

  // Reset pre-integration of frames up to and including the target
  for (auto it = all_image_frame_.begin(); it != it_0; ++it) {
    it->second.pre_integration_.reset();
  }
  it_0->second.pre_integration_.reset();

  // Erase frames up to and including the target timestamp
  all_image_frame_.erase(all_image_frame_.begin(), it_0);
  all_image_frame_.erase(timestamp);
}

// ===========================================================================
// storeLastPose
// ===========================================================================
void Estimator::storeLastPose() {
  last_R_ = window_.back().R_;
  last_P_ = window_.back().P_;
}

// ===========================================================================
// getRotation
// ===========================================================================
Eigen::Matrix3d Estimator::getRotation() const {
  std::lock_guard<std::mutex> lock(estimator_mutex_);
  return window_.back().R_;
}

// ===========================================================================
// getPosition
// ===========================================================================
Eigen::Vector3d Estimator::getPosition() const {
  std::lock_guard<std::mutex> lock(estimator_mutex_);
  return window_.back().P_;
}

// ===========================================================================
// getSlidingWindowMapPoints
// ===========================================================================
std::vector<Eigen::Vector3d> Estimator::getSlidingWindowMapPoints() const {
  std::lock_guard<std::mutex> lock(estimator_mutex_);
  std::vector<Eigen::Vector3d> points;
  if (solver_flag_ != common::SolverFlag::kNonLinear) {
    return points;
  }

  for (const auto& feature : f_manager_.features()) {
    if (!(feature.used_num_ >= 2 &&
          feature.start_frame_ < common::kWindowSize - 2)) {
      continue;
    }
    if (feature.estimated_depth_ > 0 && feature.solve_flag_ == 1) {
      Eigen::Vector3d pts_normalized =
          feature.feature_per_frame_[0].point_;
      Eigen::Vector3d pts_3d = pts_normalized * feature.estimated_depth_;
      int frame_idx = feature.start_frame_;
      if (frame_idx < common::kWindowSize) {
        Eigen::Matrix3d R_wc = window_[frame_idx].R_ * ric_;
        Eigen::Vector3d t_wc =
            window_[frame_idx].P_ + window_[frame_idx].R_ * tic_;
        Eigen::Vector3d pts_world = R_wc * pts_3d + t_wc;
        if (pts_world.allFinite()) {
          points.push_back(pts_world);
        }
      }
    }
  }
  return points;
}

}  // namespace backend
}  // namespace slam

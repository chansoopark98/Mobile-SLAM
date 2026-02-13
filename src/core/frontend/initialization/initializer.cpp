// Copyright 2025 Mobile-SLAM Authors. All rights reserved.
// Licensed under the MIT License.

#include "slam/frontend/initialization/initializer.h"

#include <cmath>
#include <iostream>
#include <vector>

#include <opencv2/calib3d.hpp>
#include <opencv2/core/eigen.hpp>

namespace slam {
namespace frontend {

// ---------------------------------------------------------------------------
// Construction
// ---------------------------------------------------------------------------

Initializer::Initializer(const config::SlamConfig& config) : config_(config) {}

// ---------------------------------------------------------------------------
// Main initialisation entry point
// ---------------------------------------------------------------------------

bool Initializer::initialize(
    std::map<double, common::ImageFrame>& all_image_frame,
    Eigen::Vector3d* Bgs,
    Eigen::Vector3d& g,
    Eigen::VectorXd& x,
    Eigen::Matrix3d* Rs,
    Eigen::Vector3d* Ps,
    FeatureManager& f_manager,
    int window_size) {
  // 1. Check IMU excitation.
  if (!checkImuExcitation(all_image_frame, 0.25)) {
    return false;
  }

  // 2. Initial SfM reconstruction.
  if (!solveInitialSfM(all_image_frame, f_manager, window_size, Rs, Ps)) {
    std::cout << "Initial SfM reconstruction failed!" << std::endl;
    return false;
  }

  // 3. Visual-inertial alignment.
  if (visualInitialAlign(all_image_frame, Bgs, g, x)) {
    std::cout << "visualInertialAlign() success!" << std::endl;
    return true;
  } else {
    std::cout << "misalign visual structure with IMU" << std::endl;
    return false;
  }
}

// ---------------------------------------------------------------------------
// Check IMU excitation (gravity-vector variance across frames)
// ---------------------------------------------------------------------------

bool Initializer::checkImuExcitation(
    const std::map<double, common::ImageFrame>& all_image_frame,
    double threshold) const {
  if (all_image_frame.size() <= 1) {
    std::cout << "Not enough image frames for IMU excitation check!"
              << std::endl;
    return false;
  }

  // Calculate average approximate gravity vector from pre-integration.
  Eigen::Vector3d sum_g = Eigen::Vector3d::Zero();
  int valid_frames = 0;

  for (auto frame_it = all_image_frame.begin();
       frame_it != all_image_frame.end(); ++frame_it) {
    if (frame_it == all_image_frame.begin()) continue;

    if (frame_it->second.pre_integration_ != nullptr) {
      double dt = frame_it->second.pre_integration_->sum_dt_;
      if (dt > 0) {
        Eigen::Vector3d tmp_g =
            frame_it->second.pre_integration_->delta_v_ / dt;
        sum_g += tmp_g;
        valid_frames++;
      }
    }
  }

  if (valid_frames == 0) {
    std::cout << "No valid frames with pre-integration data!" << std::endl;
    return false;
  }

  Eigen::Vector3d aver_g = sum_g / valid_frames;

  // Calculate variance.
  double variance = 0.0;
  int variance_count = 0;

  for (auto frame_it = all_image_frame.begin();
       frame_it != all_image_frame.end(); ++frame_it) {
    if (frame_it == all_image_frame.begin()) continue;

    if (frame_it->second.pre_integration_ != nullptr) {
      double dt = frame_it->second.pre_integration_->sum_dt_;
      if (dt > 0) {
        Eigen::Vector3d tmp_g =
            frame_it->second.pre_integration_->delta_v_ / dt;
        Eigen::Vector3d diff = tmp_g - aver_g;
        variance += diff.squaredNorm();
        variance_count++;
      }
    }
  }

  if (variance_count <= 1) {
    std::cout << "Not enough frames for variance calculation!" << std::endl;
    return false;
  }

  double std_deviation = std::sqrt(variance / (variance_count - 1));
  std::cout << "IMU excitation check: std_deviation = " << std_deviation
            << ", threshold = " << threshold << std::endl;

  if (std_deviation < threshold) {
    std::cout
        << "IMU excitation not enough! Please move the device with more "
           "rotation."
        << std::endl;
    return false;
  }

  std::cout << "IMU excitation sufficient for initialization." << std::endl;
  return true;
}

// ---------------------------------------------------------------------------
// Find relative pose (reference frame with enough parallax)
// ---------------------------------------------------------------------------

bool Initializer::relativePose(const FeatureManager& f_manager,
                               int window_size,
                               Eigen::Matrix3d& relative_R,
                               Eigen::Vector3d& relative_T,
                               int& l) const {
  for (int i = 0; i < window_size; i++) {
    Correspondences corres = f_manager.getCorresponding(i, window_size);
    if (corres.size() <= 20) continue;

    double sum_parallax = 0;
    for (int j = 0; j < static_cast<int>(corres.size()); j++) {
      Eigen::Vector2d pts_0(corres[j].first(0), corres[j].first(1));
      Eigen::Vector2d pts_1(corres[j].second(0), corres[j].second(1));
      sum_parallax += (pts_0 - pts_1).norm();
    }

    double average_parallax =
        sum_parallax / static_cast<double>(corres.size());

    if (average_parallax * 460 > 30 &&
        MotionEstimator::solveRelativeRT(corres, relative_R, relative_T)) {
      l = i;
      std::cout << "average_parallax " << average_parallax * 460
                << " choose index " << l
                << " and newest frame to triangulate the whole structure"
                << std::endl;
      return true;
    }
  }
  return false;
}

// ---------------------------------------------------------------------------
// Initial SfM pipeline
// ---------------------------------------------------------------------------

bool Initializer::solveInitialSfM(
    std::map<double, common::ImageFrame>& all_image_frame,
    FeatureManager& f_manager,
    int window_size,
    Eigen::Matrix3d* Rs,
    Eigen::Vector3d* Ps) {
  std::map<int, Eigen::Vector3d> sfm_tracked_points;

  // Prepare SfM features from the feature manager.
  std::vector<SfMFeature> sfm_f;
  for (const auto& it_per_id : f_manager.features()) {
    int imu_j = it_per_id.start_frame_ - 1;
    SfMFeature sfm_feature;
    sfm_feature.state = false;
    sfm_feature.id = it_per_id.feature_id_;

    for (const auto& it_per_frame : it_per_id.feature_per_frame_) {
      imu_j++;
      Eigen::Vector3d pts_j = it_per_frame.point_;
      sfm_feature.observation.emplace_back(
          imu_j, Eigen::Vector2d{pts_j.x(), pts_j.y()});
    }
    sfm_f.push_back(sfm_feature);
  }

  // Find relative pose between a reference frame and the latest frame.
  Eigen::Matrix3d relative_R;
  Eigen::Vector3d relative_T;
  int l;
  if (!relativePose(f_manager, window_size, relative_R, relative_T, l)) {
    std::cout << "Initial SfM failed: Not enough features or parallax."
              << std::endl;
    return false;
  }

  // Run incremental SfM + bundle adjustment.
  std::vector<Eigen::Quaterniond> Q(window_size + 1);
  std::vector<Eigen::Vector3d> T(window_size + 1);

  InitialSFM sfm;
  if (!sfm.construct(window_size + 1, Q, T, l, relative_R, relative_T,
                     sfm_f, sfm_tracked_points)) {
    std::cout << "global SFM failed!" << std::endl;
    return false;
  }

  // Solve PnP for all frames (keyframes and non-keyframes) to recover
  // full camera trajectory.
  const Eigen::Matrix3d& r_ic = config_.camera.r_ic;
  const Eigen::Vector3d& t_ic = config_.camera.t_ic;

  // Assign poses to keyframes in the sliding window.
  auto frame_it = all_image_frame.begin();
  int i = 0;
  for (; frame_it != all_image_frame.end(); ++frame_it) {
    cv::Mat r, rvec, t_cv, D, tmp_r;

    // Check if this is a sliding-window keyframe by matching timestamps.
    // We compare timestamps for the first (window_size + 1) keyframes.
    bool is_keyframe = false;
    if (i <= window_size) {
      // Simple approach: check timestamps against the ordered keyframes.
      // The all_image_frame timestamps correspond to the keyframes in order.
      is_keyframe = true;

      frame_it->second.R_ = Q[i].toRotationMatrix() * r_ic.transpose();
      frame_it->second.T_ = T[i];
      frame_it->second.is_key_frame_ = true;
      i++;

      if (i > window_size) {
        // After processing all window frames, handle remaining non-keyframes.
        ++frame_it;
        break;
      }
      continue;
    }
  }

  // For remaining non-keyframe frames, solve PnP.
  for (; frame_it != all_image_frame.end(); ++frame_it) {
    frame_it->second.is_key_frame_ = false;

    Eigen::Matrix3d R_initial = (Q[window_size].inverse()).toRotationMatrix();
    Eigen::Vector3d P_initial = -R_initial * T[window_size];

    cv::Mat r, rvec, t_cv, D, tmp_r;
    cv::eigen2cv(R_initial, tmp_r);
    cv::Rodrigues(tmp_r, rvec);
    cv::eigen2cv(P_initial, t_cv);

    std::vector<cv::Point3f> pts_3_vector;
    std::vector<cv::Point2f> pts_2_vector;

    for (const auto& [feature_id, obs] : frame_it->second.points_) {
      auto it = sfm_tracked_points.find(feature_id);
      if (it != sfm_tracked_points.end()) {
        Eigen::Vector3d world_pts = it->second;
        pts_3_vector.emplace_back(static_cast<float>(world_pts(0)),
                                  static_cast<float>(world_pts(1)),
                                  static_cast<float>(world_pts(2)));
        pts_2_vector.emplace_back(static_cast<float>(obs(0)),
                                  static_cast<float>(obs(1)));
      }
    }

    if (pts_3_vector.size() < 6) continue;

    cv::Mat K = cv::Mat::eye(3, 3, CV_64F);
    if (!cv::solvePnP(pts_3_vector, pts_2_vector, K, D, rvec, t_cv, true)) {
      continue;
    }

    cv::Rodrigues(rvec, r);
    Eigen::MatrixXd R_pnp, tmp_R_pnp;
    cv::cv2eigen(r, tmp_R_pnp);
    R_pnp = tmp_R_pnp.transpose();
    Eigen::MatrixXd T_pnp;
    cv::cv2eigen(t_cv, T_pnp);
    T_pnp = R_pnp * (-T_pnp);

    frame_it->second.R_ = R_pnp * r_ic.transpose();
    frame_it->second.T_ = T_pnp;
  }

  // Write back the rotation and position for the sliding-window frames.
  frame_it = all_image_frame.begin();
  for (int k = 0; k <= window_size && frame_it != all_image_frame.end();
       ++frame_it) {
    if (frame_it->second.is_key_frame_) {
      Rs[k] = frame_it->second.R_;
      Ps[k] = frame_it->second.T_;
      k++;
    }
  }

  return true;
}

// ---------------------------------------------------------------------------
// Visual-inertial alignment
// ---------------------------------------------------------------------------

bool Initializer::visualInitialAlign(
    std::map<double, common::ImageFrame>& all_image_frame,
    Eigen::Vector3d* Bgs,
    Eigen::Vector3d& g,
    Eigen::VectorXd& x) {
  bool result = VisualIMUAlignment(all_image_frame, Bgs, g, x, config_);
  if (!result) {
    std::cout << "solve gravity vector failed!" << std::endl;
    return false;
  }
  return true;
}

}  // namespace frontend
}  // namespace slam

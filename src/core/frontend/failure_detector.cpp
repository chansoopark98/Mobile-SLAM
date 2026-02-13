// Copyright 2025 Mobile-SLAM Authors. All rights reserved.
// Licensed under the MIT License.

#include "slam/frontend/failure_detector.h"

#include <cmath>
#include <iostream>

namespace slam {
namespace frontend {

// ---------------------------------------------------------------------------
// Constructors
// ---------------------------------------------------------------------------

FailureDetector::FailureDetector()
    : max_translation_(5.0),
      max_rotation_(50.0),
      min_features_(15) {}

FailureDetector::FailureDetector(double max_translation, double max_rotation,
                                 int min_features)
    : max_translation_(max_translation),
      max_rotation_(max_rotation),
      min_features_(min_features) {}

// ---------------------------------------------------------------------------
// Main detection entry point
// ---------------------------------------------------------------------------

bool FailureDetector::detectFailure(
    const Eigen::Matrix3d& R, const Eigen::Vector3d& P,
    const Eigen::Matrix3d& last_R, const Eigen::Vector3d& last_P,
    int tracked_features) const {
  if (checkFeatureCount(tracked_features)) {
    std::cerr << "[FailureDetector] Too few tracked features: "
              << tracked_features << " (min " << min_features_ << ")\n";
    return true;
  }

  if (checkTranslation(P, last_P)) {
    std::cerr << "[FailureDetector] Large translation jump: "
              << (P - last_P).norm() << " m (max " << max_translation_
              << " m)\n";
    return true;
  }

  if (checkRotation(R, last_R)) {
    Eigen::Matrix3d delta_R = R.transpose() * last_R;
    Eigen::Quaterniond delta_q(delta_R);
    double angle = std::acos(std::min(std::abs(delta_q.w()), 1.0)) *
                   2.0 * 180.0 / M_PI;
    std::cerr << "[FailureDetector] Large rotation jump: " << angle
              << " deg (max " << max_rotation_ << " deg)\n";
    return true;
  }

  return false;
}

// ---------------------------------------------------------------------------
// Individual checks
// ---------------------------------------------------------------------------

bool FailureDetector::checkTranslation(
    const Eigen::Vector3d& P,
    const Eigen::Vector3d& last_P) const {
  return (P - last_P).norm() > max_translation_;
}

bool FailureDetector::checkRotation(
    const Eigen::Matrix3d& R,
    const Eigen::Matrix3d& last_R) const {
  Eigen::Matrix3d delta_R = R.transpose() * last_R;
  Eigen::Quaterniond delta_q(delta_R);
  // acos(|w|) * 2 gives the total rotation angle in radians
  double angle_deg =
      std::acos(std::min(std::abs(delta_q.w()), 1.0)) * 2.0 * 180.0 / M_PI;
  return angle_deg > max_rotation_;
}

bool FailureDetector::checkFeatureCount(int tracked_features) const {
  return tracked_features < min_features_;
}

}  // namespace frontend
}  // namespace slam

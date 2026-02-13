// Copyright 2025 Mobile-SLAM Authors. All rights reserved.
// Licensed under the MIT License.

#ifndef SLAM_FRONTEND_FAILURE_DETECTOR_H_
#define SLAM_FRONTEND_FAILURE_DETECTOR_H_

#include <Eigen/Dense>

namespace slam {
namespace frontend {

// ---------------------------------------------------------------------------
// FailureDetector
// ---------------------------------------------------------------------------

/// @brief Lightweight heuristic detector for catastrophic VIO failures.
///
/// After each optimisation step the estimator calls detectFailure() with the
/// current and previous poses plus the number of tracked features.  If any of
/// the following conditions is met the system should be re-initialised:
///
///   - Translation between consecutive frames exceeds max_translation_.
///   - Rotation between consecutive frames exceeds max_rotation_ (degrees).
///   - Number of tracked features drops below min_features_.
class FailureDetector {
 public:
  /// @brief Construct with default thresholds.
  FailureDetector();

  /// @brief Construct with custom thresholds.
  /// @param max_translation  Maximum acceptable translation jump (meters).
  /// @param max_rotation     Maximum acceptable rotation jump (degrees).
  /// @param min_features     Minimum number of tracked features.
  FailureDetector(double max_translation, double max_rotation,
                  int min_features);

  // -----------------------------------------------------------------------
  // Main detection interface
  // -----------------------------------------------------------------------

  /// @brief Check for a tracking failure.
  ///
  /// @param R               Current frame rotation (body-to-world).
  /// @param P               Current frame position (world).
  /// @param last_R          Previous frame rotation (body-to-world).
  /// @param last_P          Previous frame position (world).
  /// @param tracked_features Number of features currently tracked.
  /// @return true if a failure is detected and re-initialisation is needed.
  bool detectFailure(const Eigen::Matrix3d& R, const Eigen::Vector3d& P,
                     const Eigen::Matrix3d& last_R,
                     const Eigen::Vector3d& last_P,
                     int tracked_features) const;

  // -----------------------------------------------------------------------
  // Individual checks (exposed for unit testing / diagnostics)
  // -----------------------------------------------------------------------

  /// @brief Check whether the translation jump is too large.
  bool checkTranslation(const Eigen::Vector3d& P,
                        const Eigen::Vector3d& last_P) const;

  /// @brief Check whether the rotation jump is too large.
  bool checkRotation(const Eigen::Matrix3d& R,
                     const Eigen::Matrix3d& last_R) const;

  /// @brief Check whether there are too few tracked features.
  bool checkFeatureCount(int tracked_features) const;

  // -----------------------------------------------------------------------
  // Threshold setters
  // -----------------------------------------------------------------------
  void setMaxTranslation(double v) { max_translation_ = v; }
  void setMaxRotation(double v) { max_rotation_ = v; }
  void setMinFeatures(int v) { min_features_ = v; }

  // -----------------------------------------------------------------------
  // Threshold getters
  // -----------------------------------------------------------------------
  double maxTranslation() const { return max_translation_; }
  double maxRotation() const { return max_rotation_; }
  int minFeatures() const { return min_features_; }

 private:
  double max_translation_;  ///< meters
  double max_rotation_;     ///< degrees
  int min_features_;
};

}  // namespace frontend
}  // namespace slam

#endif  // SLAM_FRONTEND_FAILURE_DETECTOR_H_

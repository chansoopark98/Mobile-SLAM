// Copyright 2025 Mobile-SLAM Authors. All rights reserved.
// Licensed under the MIT License.

#ifndef SLAM_COMMON_IMAGE_FRAME_H_
#define SLAM_COMMON_IMAGE_FRAME_H_

#include <Eigen/Dense>
#include <map>
#include <memory>
#include <utility>

#include "slam/common/types.h"

namespace slam {
namespace backend {
class IntegrationBase;  // forward declaration
}  // namespace backend
}  // namespace slam

namespace slam {
namespace common {

/// @brief Lightweight image-frame container used during initialization and
///        keyframe selection.
///
/// Unlike Frame (which lives in the sliding window and owns a unique_ptr to
/// its IntegrationBase), ImageFrame uses a shared_ptr so that the
/// initialization module and the estimator can share the same pre-integration
/// object.
class ImageFrame {
 public:
  /// @brief Default constructor.
  ImageFrame()
      : timestamp_(0.0),
        R_(Eigen::Matrix3d::Identity()),
        T_(Eigen::Vector3d::Zero()),
        is_key_frame_(false) {}

  /// @brief Construct from feature observations and timestamp.
  /// @param points  Per-feature observation map (feature_id -> 7-DOF vector).
  /// @param timestamp  Capture time in seconds.
  ImageFrame(const ImageData& points, double timestamp)
      : points_(points),
        timestamp_(timestamp),
        R_(Eigen::Matrix3d::Identity()),
        T_(Eigen::Vector3d::Zero()),
        is_key_frame_(false) {}

  ~ImageFrame() = default;

  // -----------------------------------------------------------------------
  // Move semantics
  // -----------------------------------------------------------------------
  ImageFrame(ImageFrame&& other) noexcept
      : points_(std::move(other.points_)),
        timestamp_(other.timestamp_),
        R_(other.R_),
        T_(other.T_),
        pre_integration_(std::move(other.pre_integration_)),
        is_key_frame_(other.is_key_frame_) {}

  ImageFrame& operator=(ImageFrame&& other) noexcept {
    if (this != &other) {
      points_ = std::move(other.points_);
      timestamp_ = other.timestamp_;
      R_ = other.R_;
      T_ = other.T_;
      pre_integration_ = std::move(other.pre_integration_);
      is_key_frame_ = other.is_key_frame_;
    }
    return *this;
  }

  // Copy is allowed (shared_ptr is copyable)
  ImageFrame(const ImageFrame&) = default;
  ImageFrame& operator=(const ImageFrame&) = default;

  // -----------------------------------------------------------------------
  // Accessors
  // -----------------------------------------------------------------------

  /// @brief Total number of feature observations in this frame.
  size_t GetFeatureCount() const { return points_.size(); }

  /// @brief Check if the frame has at least @p min_features observations.
  bool HasEnoughFeatures(size_t min_features = 10) const {
    return GetFeatureCount() >= min_features;
  }

  /// @brief Return the 4x4 rigid-body transformation [R T; 0 1].
  Eigen::Matrix4d GetPoseMatrix() const {
    Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
    pose.block<3, 3>(0, 0) = R_;
    pose.block<3, 1>(0, 3) = T_;
    return pose;
  }

  /// @brief Set rotation and translation from a 4x4 matrix.
  void SetPoseMatrix(const Eigen::Matrix4d& pose) {
    R_ = pose.block<3, 3>(0, 0);
    T_ = pose.block<3, 1>(0, 3);
  }

  // -----------------------------------------------------------------------
  // Data members
  // -----------------------------------------------------------------------

  /// Feature observations: feature_id -> [ray_x, ray_y, ray_z, u, v, vx, vy]
  ImageData points_;

  /// Capture timestamp (seconds)
  double timestamp_;

  /// Rotation matrix (body/camera to world)
  Eigen::Matrix3d R_;

  /// Translation vector (position in world frame)
  Eigen::Vector3d T_;

  /// IMU pre-integration shared with initializer
  std::shared_ptr<backend::IntegrationBase> pre_integration_;

  /// Whether this frame has been selected as a keyframe
  bool is_key_frame_;
};

}  // namespace common
}  // namespace slam

#endif  // SLAM_COMMON_IMAGE_FRAME_H_

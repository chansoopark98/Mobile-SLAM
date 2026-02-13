// Copyright 2025 Mobile-SLAM Authors. All rights reserved.
// Licensed under the MIT License.

#include "slam/frontend/feature_manager.h"

#include <cassert>
#include <cmath>
#include <iostream>

namespace slam {
namespace frontend {

// ---------------------------------------------------------------------------
// Constructor / state reset
// ---------------------------------------------------------------------------

FeatureManager::FeatureManager(double min_parallax)
    : min_parallax_(min_parallax) {}

void FeatureManager::clearState() {
  features_.clear();
}

// ---------------------------------------------------------------------------
// Feature count (features usable by the solver)
// ---------------------------------------------------------------------------

int FeatureManager::getFeatureCount() const {
  int cnt = 0;
  for (auto& f : features_) {
    // Use a local copy so we don't mutate in a const method
    int used = static_cast<int>(f.feature_per_frame_.size());
    if (used >= 2 && f.start_frame_ < common::kWindowSize - 2) {
      ++cnt;
    }
  }
  return cnt;
}

// ---------------------------------------------------------------------------
// Add features and decide keyframe by parallax
// ---------------------------------------------------------------------------

bool FeatureManager::addFeatureCheckParallax(
    int frame_count,
    const common::ImageData& image_data,
    double focal_length) {
  double parallax_sum = 0.0;
  int parallax_num = 0;
  last_track_num_ = 0;

  // -- Insert observations into the feature bank -------------------------
  for (const auto& [feature_id, observation] : image_data) {
    FeaturePerFrame fpf(observation);

    auto it = std::find_if(
        features_.begin(), features_.end(),
        [feature_id](const FeaturePerId& f) {
          return f.feature_id_ == feature_id;
        });

    if (it == features_.end()) {
      // New feature
      features_.emplace_back(feature_id, frame_count);
      features_.back().feature_per_frame_.push_back(fpf);
    } else {
      // Existing feature -- append observation
      it->feature_per_frame_.push_back(fpf);
      ++last_track_num_;
    }
  }

  // -- Keyframe decision --------------------------------------------------
  // If we are early in the sequence or tracking very few features, always
  // declare a keyframe.
  if (frame_count < 2 || last_track_num_ < 20) {
    return true;
  }

  // Accumulate parallax of features that span at least from (frame_count-2)
  // to (frame_count-1) -- i.e. the second-to-last and third-to-last frames.
  for (auto& f : features_) {
    if (f.start_frame_ <= frame_count - 2 &&
        f.endFrame() >= frame_count - 1) {
      parallax_sum += compensatedParallax(f, frame_count);
      ++parallax_num;
    }
  }

  if (parallax_num == 0) {
    return true;  // No overlap -- treat as novel view
  }

  // Compare mean parallax (in normalised coordinates) against the threshold
  // (which is stored in pixels and divided by focal length here).
  return (parallax_sum / parallax_num) >= (min_parallax_ / focal_length);
}

// ---------------------------------------------------------------------------
// Correspondences between two frames
// ---------------------------------------------------------------------------

Correspondences FeatureManager::getCorresponding(int frame_0,
                                                 int frame_1) const {
  Correspondences corres;
  for (const auto& f : features_) {
    if (f.start_frame_ <= frame_0 && f.endFrame() >= frame_1) {
      int idx_0 = frame_0 - f.start_frame_;
      int idx_1 = frame_1 - f.start_frame_;
      corres.emplace_back(f.feature_per_frame_[idx_0].point_,
                          f.feature_per_frame_[idx_1].point_);
    }
  }
  return corres;
}

// ---------------------------------------------------------------------------
// Depth management
// ---------------------------------------------------------------------------

void FeatureManager::setDepth(const Eigen::VectorXd& depths) {
  int idx = -1;
  for (auto& f : features_) {
    f.used_num_ = static_cast<int>(f.feature_per_frame_.size());
    if (!(f.used_num_ >= 2 && f.start_frame_ < common::kWindowSize - 2)) {
      continue;
    }

    f.estimated_depth_ = 1.0 / depths(++idx);
    if (f.estimated_depth_ < 0) {
      f.solve_flag_ = 2;  // failed
    } else {
      f.solve_flag_ = 1;  // success
    }
  }
}

Eigen::VectorXd FeatureManager::getDepthVector() const {
  Eigen::VectorXd dep(getFeatureCount());
  int idx = -1;
  for (const auto& f : features_) {
    int used = static_cast<int>(f.feature_per_frame_.size());
    if (!(used >= 2 && f.start_frame_ < common::kWindowSize - 2)) {
      continue;
    }
    dep(++idx) = 1.0 / f.estimated_depth_;
  }
  return dep;
}

void FeatureManager::removeFailures() {
  for (auto it = features_.begin(); it != features_.end();) {
    if (it->solve_flag_ == 2) {
      it = features_.erase(it);
    } else {
      ++it;
    }
  }
}

void FeatureManager::clearDepth() {
  for (auto& f : features_) {
    f.estimated_depth_ = -1.0;
    f.solve_flag_ = 0;
  }
}

// ---------------------------------------------------------------------------
// Triangulation (multi-view linear SVD)
// ---------------------------------------------------------------------------

void FeatureManager::triangulate(
    const std::vector<Eigen::Matrix3d>& Rs,
    const std::vector<Eigen::Vector3d>& Ps,
    const Eigen::Matrix3d& ric,
    const Eigen::Vector3d& tic) {
  for (auto& f : features_) {
    f.used_num_ = static_cast<int>(f.feature_per_frame_.size());
    if (!(f.used_num_ >= 2 && f.start_frame_ < common::kWindowSize - 2)) {
      continue;
    }

    // Already triangulated -- skip
    if (f.estimated_depth_ > 0) {
      continue;
    }

    int imu_i = f.start_frame_;
    int imu_j = imu_i - 1;

    // Build the camera pose for the anchor frame (frame imu_i)
    Eigen::Vector3d t0 = Ps[imu_i] + Rs[imu_i] * tic;
    Eigen::Matrix3d R0 = Rs[imu_i] * ric;

    // Reference projection matrix: P0 = [I | 0]
    Eigen::Matrix<double, 3, 4> P0;
    P0.leftCols<3>() = Eigen::Matrix3d::Identity();
    P0.rightCols<1>() = Eigen::Vector3d::Zero();

    // Accumulate DLT rows
    Eigen::MatrixXd svd_A(2 * f.used_num_, 4);
    int svd_idx = 0;

    for (const auto& fpf : f.feature_per_frame_) {
      ++imu_j;

      Eigen::Vector3d t1 = Ps[imu_j] + Rs[imu_j] * tic;
      Eigen::Matrix3d R1 = Rs[imu_j] * ric;
      Eigen::Vector3d relative_t = R0.transpose() * (t1 - t0);
      Eigen::Matrix3d relative_R = R0.transpose() * R1;

      Eigen::Matrix<double, 3, 4> P;
      P.leftCols<3>() = relative_R.transpose();
      P.rightCols<1>() = -relative_R.transpose() * relative_t;

      Eigen::Vector3d ray = fpf.point_.normalized();
      svd_A.row(svd_idx++) = ray[0] * P.row(2) - ray[2] * P.row(0);
      svd_A.row(svd_idx++) = ray[1] * P.row(2) - ray[2] * P.row(1);
    }

    assert(svd_idx == svd_A.rows());

    Eigen::Vector4d svd_V =
        Eigen::JacobiSVD<Eigen::MatrixXd>(svd_A, Eigen::ComputeThinV)
            .matrixV()
            .rightCols<1>();

    double depth = svd_V[2] / svd_V[3];
    f.estimated_depth_ = depth;

    if (f.estimated_depth_ < 0.1) {
      f.estimated_depth_ = kInitDepth;
    }
  }
}

// ---------------------------------------------------------------------------
// Sliding-window maintenance
// ---------------------------------------------------------------------------

void FeatureManager::removeBack() {
  for (auto it = features_.begin(); it != features_.end();) {
    if (it->start_frame_ != 0) {
      --(it->start_frame_);
      ++it;
    } else {
      it->feature_per_frame_.erase(it->feature_per_frame_.begin());
      if (it->feature_per_frame_.empty()) {
        it = features_.erase(it);
      } else {
        ++it;
      }
    }
  }
}

void FeatureManager::removeBackShiftDepth(
    const Eigen::Matrix3d& marg_R, const Eigen::Vector3d& marg_P,
    const Eigen::Matrix3d& new_R, const Eigen::Vector3d& new_P) {
  for (auto it = features_.begin(); it != features_.end();) {
    if (it->start_frame_ != 0) {
      --(it->start_frame_);
      ++it;
    } else {
      Eigen::Vector3d uv_i = it->feature_per_frame_[0].point_;
      it->feature_per_frame_.erase(it->feature_per_frame_.begin());

      if (it->feature_per_frame_.size() < 2) {
        it = features_.erase(it);
        continue;
      }

      // Re-project the 3-D point from the old anchor into the new anchor
      Eigen::Vector3d pts_i = uv_i * it->estimated_depth_;
      Eigen::Vector3d w_pts_i = marg_R * pts_i + marg_P;
      Eigen::Vector3d pts_j = new_R.transpose() * (w_pts_i - new_P);
      double dep_j = pts_j(2);
      if (dep_j > 0) {
        it->estimated_depth_ = dep_j;
      } else {
        it->estimated_depth_ = kInitDepth;
      }
      ++it;
    }
  }
}

void FeatureManager::removeFront(int frame_count) {
  for (auto it = features_.begin(); it != features_.end();) {
    if (it->start_frame_ == frame_count) {
      --(it->start_frame_);
      ++it;
    } else {
      int j = common::kWindowSize - 1 - it->start_frame_;
      if (it->endFrame() < frame_count - 1) {
        ++it;
        continue;
      }
      it->feature_per_frame_.erase(it->feature_per_frame_.begin() + j);
      if (it->feature_per_frame_.empty()) {
        it = features_.erase(it);
      } else {
        ++it;
      }
    }
  }
}

// ---------------------------------------------------------------------------
// Compensated parallax between (frame_count-2) and (frame_count-1)
// ---------------------------------------------------------------------------

double FeatureManager::compensatedParallax(const FeaturePerId& f,
                                           int frame_count) const {
  // Observations at the second-to-last and third-to-last frames
  const FeaturePerFrame& frame_i =
      f.feature_per_frame_[frame_count - 2 - f.start_frame_];
  const FeaturePerFrame& frame_j =
      f.feature_per_frame_[frame_count - 1 - f.start_frame_];

  Eigen::Vector3d p_j = frame_j.point_;
  double u_j = p_j(0);
  double v_j = p_j(1);

  Eigen::Vector3d p_i = frame_i.point_;
  double dep_i = p_i(2);
  double u_i = p_i(0) / dep_i;
  double v_i = p_i(1) / dep_i;

  double du = u_i - u_j;
  double dv = v_i - v_j;

  return std::sqrt(du * du + dv * dv);
}

}  // namespace frontend
}  // namespace slam

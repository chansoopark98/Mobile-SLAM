// Copyright 2025 Mobile-SLAM Authors. All rights reserved.
// Licensed under the MIT License.

#ifndef SLAM_FRONTEND_INITIALIZATION_INITIAL_SFM_H_
#define SLAM_FRONTEND_INITIALIZATION_INITIAL_SFM_H_

#include <Eigen/Dense>
#include <map>
#include <utility>
#include <vector>

#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>

namespace slam {
namespace frontend {

// ---------------------------------------------------------------------------
// SfM feature observation across multiple frames
// ---------------------------------------------------------------------------

/// @brief Lightweight feature record used during the initial SfM
///        reconstruction.  Each feature stores its frame-observation pairs
///        and, once triangulated, the reconstructed 3-D position.
struct SfMFeature {
  bool state = false;
  int id = -1;
  std::vector<std::pair<int, Eigen::Vector2d>> observation;
  double position[3] = {0.0, 0.0, 0.0};
  double depth = 0.0;
};

// ---------------------------------------------------------------------------
// Ceres reprojection cost functor for global BA
// ---------------------------------------------------------------------------

/// @brief Autodiff reprojection error for the vision-only bundle adjustment
///        used during SfM initialisation.
///
/// The camera rotation is stored as a quaternion in Eigen convention
/// (x, y, z, w).  The functor uses ceres::QuaternionRotatePoint which
/// expects (w, x, y, z), so we re-order internally.
struct ReprojectionError3D {
  ReprojectionError3D(double observed_u, double observed_v)
      : observed_u_(observed_u), observed_v_(observed_v) {}

  template <typename T>
  bool operator()(const T* const camera_R,
                  const T* const camera_T,
                  const T* point,
                  T* residuals) const {
    // camera_R is stored in Eigen order [x, y, z, w].
    // ceres::QuaternionRotatePoint expects [w, x, y, z].
    T q_wxyz[4] = {camera_R[3], camera_R[0], camera_R[1], camera_R[2]};

    T p[3];
    ceres::QuaternionRotatePoint(q_wxyz, point, p);
    p[0] += camera_T[0];
    p[1] += camera_T[1];
    p[2] += camera_T[2];

    T xp = p[0] / p[2];
    T yp = p[1] / p[2];
    residuals[0] = xp - T(observed_u_);
    residuals[1] = yp - T(observed_v_);
    return true;
  }

  static ceres::CostFunction* Create(double observed_x, double observed_y) {
    return new ceres::AutoDiffCostFunction<ReprojectionError3D, 2, 4, 3, 3>(
        new ReprojectionError3D(observed_x, observed_y));
  }

  double observed_u_;
  double observed_v_;
};

// ---------------------------------------------------------------------------
// Initial SfM reconstruction
// ---------------------------------------------------------------------------

/// @brief Performs incremental Structure-from-Motion initialisation.
///
/// Given a reference frame and a latest frame with known relative pose, the
/// class triangulates features, solves PnP for intermediate frames, and runs
/// a global bundle adjustment using Ceres.
class InitialSFM {
 public:
  InitialSFM() = default;

  /// @brief Full SfM reconstruction pipeline.
  ///
  /// @param[in]     frame_num           Total number of frames.
  /// @param[in,out] q                   Output quaternions (world-to-body).
  /// @param[in,out] T                   Output translations.
  /// @param[in]     l                   Reference frame index.
  /// @param[in]     relative_R          Relative rotation  (ref -> latest).
  /// @param[in]     relative_T          Relative translation (ref -> latest).
  /// @param[in,out] sfm_f               SfM feature observations.
  /// @param[out]    sfm_tracked_points  Reconstructed 3-D points.
  /// @return true on success.
  bool construct(int frame_num,
                 std::vector<Eigen::Quaterniond>& q,
                 std::vector<Eigen::Vector3d>& T,
                 int l,
                 const Eigen::Matrix3d& relative_R,
                 const Eigen::Vector3d& relative_T,
                 std::vector<SfMFeature>& sfm_f,
                 std::map<int, Eigen::Vector3d>& sfm_tracked_points);

 private:
  /// @brief Triangulate a single 3-D point from two camera poses.
  void triangulatePoint(Eigen::Matrix<double, 3, 4>& Pose0,
                        Eigen::Matrix<double, 3, 4>& Pose1,
                        Eigen::Vector2d& point0,
                        Eigen::Vector2d& point1,
                        Eigen::Vector3d& point_3d);

  /// @brief Triangulate all untriangulated features visible in both frames.
  void triangulateTwoFrames(int frame0,
                            Eigen::Matrix<double, 3, 4>& Pose0,
                            int frame1,
                            Eigen::Matrix<double, 3, 4>& Pose1,
                            std::vector<SfMFeature>& sfm_f);

  /// @brief Solve the camera pose of frame i using PnP on already-
  ///        triangulated features.
  bool solveFrameByPnP(Eigen::Matrix3d& R_initial,
                       Eigen::Vector3d& P_initial,
                       int i,
                       std::vector<SfMFeature>& sfm_f);

  int feature_num_ = 0;
};

}  // namespace frontend
}  // namespace slam

#endif  // SLAM_FRONTEND_INITIALIZATION_INITIAL_SFM_H_

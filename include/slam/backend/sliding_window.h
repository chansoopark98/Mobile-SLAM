// Copyright 2025 Mobile-SLAM Authors. All rights reserved.
// Licensed under the MIT License.

#ifndef SLAM_BACKEND_SLIDING_WINDOW_H_
#define SLAM_BACKEND_SLIDING_WINDOW_H_

#include <Eigen/Dense>
#include <array>
#include <cstdint>

#include "slam/backend/factor/integration_base.h"
#include "slam/common/frame.h"
#include "slam/common/types.h"
#include "slam/config/slam_config.h"

namespace slam {
namespace backend {

// ---------------------------------------------------------------------------
// SlidingWindow -- Fixed-size array of Frames plus raw double parameter
//                  arrays used by the Ceres solver.
//
// The parameter arrays (para_pose_, para_speed_bias_, para_feature_,
// para_ex_pose_) are the canonical double-buffer between the Eigen state
// stored in each Frame and the raw memory that Ceres operates on.
//
// stateToParameter() copies Eigen state -> raw arrays before optimization.
// parameterToState() copies raw arrays -> Eigen state after optimization.
// ---------------------------------------------------------------------------
class SlidingWindow {
 public:
  // Parameter block sizes (matching Ceres parameter layout)
  static constexpr int kSizePose = 7;        // [px,py,pz, qx,qy,qz,qw]
  static constexpr int kSizeSpeedBias = 9;   // [vx,vy,vz, bax,bay,baz, bgx,bgy,bgz]
  static constexpr int kSizeFeature = 1;     // inverse depth

  SlidingWindow();

  /// @brief Reset all frames in the window to initial state.
  void clearSlidingWindow();

  // -----------------------------------------------------------------------
  // Element access
  // -----------------------------------------------------------------------
  common::Frame& operator[](int index) { return frames_[index]; }
  const common::Frame& operator[](int index) const { return frames_[index]; }

  const common::Frame& front() const { return frames_[0]; }
  const common::Frame& back() const { return frames_[common::kWindowSize]; }

  // -----------------------------------------------------------------------
  // Frame manipulation helpers (used during sliding-window shift)
  // -----------------------------------------------------------------------

  /// @brief Clear the IMU measurement buffers of frame[index].
  void clearBuffer(int32_t index);

  /// @brief Copy state (timestamp, R, P, V, Ba, Bg) from src to dst.
  void copyFrame(int32_t dst, int32_t src);

  /// @brief Swap full frame state including pre-integration between two indices.
  void swapFrame(int32_t a, int32_t b);

  /// @brief Swap only the IMU measurement buffers between two indices.
  void swapBuffer(int32_t a, int32_t b);

  /// @brief Append a raw IMU measurement to frame[index]'s buffers.
  void pushBackBuffer(int32_t index, double dt,
                      const Eigen::Vector3d& linear_acceleration,
                      const Eigen::Vector3d& angular_velocity);

  /// @brief Propagate a measurement through the pre-integration of frame[index].
  void pushBackPreintegration(int32_t index, double dt,
                              const Eigen::Vector3d& linear_acceleration,
                              const Eigen::Vector3d& angular_velocity);

  /// @brief Create a fresh IntegrationBase for frame[index] with given biases.
  void createNewPreintegration(int32_t index,
                               const Eigen::Vector3d& linear_acceleration,
                               const Eigen::Vector3d& angular_velocity,
                               const Eigen::Vector3d& ba,
                               const Eigen::Vector3d& bg,
                               const config::SlamConfig& config);

  // -----------------------------------------------------------------------
  // State <-> Parameter array conversion for Ceres
  // -----------------------------------------------------------------------

  /// @brief Copy Eigen state from frames_ and extrinsic/feature data into the
  ///        raw double arrays that Ceres optimizes over.
  /// @param r_ic  IMU-to-camera rotation.
  /// @param t_ic  IMU-to-camera translation.
  /// @param feature_depths  Dense vector of inverse-depth values.
  /// @param num_features    Number of valid features.
  void stateToParameter(const Eigen::Matrix3d& r_ic,
                        const Eigen::Vector3d& t_ic,
                        const Eigen::VectorXd& feature_depths,
                        int num_features);

  /// @brief Read optimized parameters back from raw arrays, applying yaw
  ///        correction to keep the first frame's yaw unchanged.
  /// @param[out] r_ic  Updated IMU-to-camera rotation.
  /// @param[out] t_ic  Updated IMU-to-camera translation.
  /// @param[out] feature_depths  Updated inverse-depth vector.
  /// @param num_features  Number of valid features.
  void parameterToState(Eigen::Matrix3d& r_ic,
                        Eigen::Vector3d& t_ic,
                        Eigen::VectorXd& feature_depths,
                        int num_features);

  // -----------------------------------------------------------------------
  // Raw parameter arrays (public for direct Ceres access)
  // -----------------------------------------------------------------------
  double para_pose_[common::kWindowSize + 1][kSizePose];
  double para_speed_bias_[common::kWindowSize + 1][kSizeSpeedBias];
  double para_feature_[common::kNumOfFeatures][kSizeFeature];
  double para_ex_pose_[1][kSizePose];

 private:
  std::array<common::Frame, common::kWindowSize + 1> frames_;
};

}  // namespace backend
}  // namespace slam

#endif  // SLAM_BACKEND_SLIDING_WINDOW_H_

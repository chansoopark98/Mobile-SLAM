// Copyright 2025 Mobile-SLAM Authors. All rights reserved.
// Licensed under the MIT License.

#ifndef SLAM_BACKEND_OPTIMIZER_H_
#define SLAM_BACKEND_OPTIMIZER_H_

#include <ceres/ceres.h>
#include <Eigen/Dense>
#include <vector>

#include "slam/backend/factor/imu_factor.h"
#include "slam/backend/factor/integration_base.h"
#include "slam/backend/factor/marginalization_factor.h"
#include "slam/backend/factor/pose_local_parameterization.h"
#include "slam/backend/factor/projection_factor.h"
#include "slam/backend/sliding_window.h"
#include "slam/common/types.h"
#include "slam/config/slam_config.h"
#include "slam/frontend/feature_manager.h"

namespace slam {
namespace backend {

// ---------------------------------------------------------------------------
// Optimizer -- Ceres-based nonlinear optimizer for the VIO sliding window.
//
// Responsibilities:
//   1. Set up the Ceres problem with pose manifold constraints.
//   2. Add IMU pre-integration factors between consecutive frames.
//   3. Add visual reprojection factors for tracked features.
//   4. Add the marginalization prior from the previous optimization.
//   5. Solve the nonlinear least-squares problem.
//   6. Write optimized parameters back to the sliding window.
//   7. Perform Schur-complement marginalization for the next iteration.
//
// Uses ceres::Manifold (Ceres 2.2+) via PoseLocalParameterization.
// ---------------------------------------------------------------------------
class Optimizer {
 public:
  /// @brief Construct with SLAM configuration (for solver params, gravity, etc.).
  explicit Optimizer(const config::SlamConfig& config);

  ~Optimizer();

  // Non-copyable
  Optimizer(const Optimizer&) = delete;
  Optimizer& operator=(const Optimizer&) = delete;

  // -----------------------------------------------------------------------
  // Main optimization interface
  // -----------------------------------------------------------------------

  /// @brief Run one full optimization + marginalization cycle.
  ///
  /// @param window         The sliding window holding frame states.
  /// @param f_manager      Feature manager with tracked features.
  /// @param last_marginalization_info
  ///   Pointer to the previous marginalization result. Ownership is managed
  ///   by the caller; this method may delete the old info and replace it.
  /// @param last_marginalization_parameter_blocks
  ///   Parameter block pointers associated with the last marginalization.
  /// @param marg_flag      Whether to marginalize the oldest keyframe or
  ///                       the newest non-keyframe.
  void optimize(SlidingWindow& window,
                frontend::FeatureManager& f_manager,
                MarginalizationInfo*& last_marginalization_info,
                std::vector<double*>& last_marginalization_parameter_blocks,
                common::MarginalizationFlag marg_flag);

  // -----------------------------------------------------------------------
  // Extrinsic parameters (IMU-to-camera)
  // -----------------------------------------------------------------------

  void setExtrinsicParameters(const Eigen::Vector3d& t_ic,
                              const Eigen::Matrix3d& r_ic);

  Eigen::Vector3d tic() const { return t_ic_; }
  Eigen::Matrix3d ric() const { return r_ic_; }

 private:
  // -----------------------------------------------------------------------
  // Optimization sub-steps
  // -----------------------------------------------------------------------

  /// @brief Add pose (with manifold) and speed-bias parameter blocks.
  void setupOptimizationProblem(ceres::Problem& problem,
                                SlidingWindow& window);

  /// @brief Add IMU pre-integration factors between consecutive frames.
  void addImuFactors(ceres::Problem& problem,
                     SlidingWindow& window);

  /// @brief Add visual reprojection factors for all valid features.
  /// @return Number of feature factors added.
  int addFeatureFactors(ceres::Problem& problem,
                        SlidingWindow& window,
                        frontend::FeatureManager& f_manager);

  /// @brief Add the marginalization prior factor.
  void addMarginalizationFactor(
      ceres::Problem& problem,
      MarginalizationInfo* last_marginalization_info,
      const std::vector<double*>& last_marginalization_parameter_blocks);

  /// @brief Configure and run the Ceres solver.
  void solveCeresProblem(ceres::Problem& problem);

  /// @brief Apply optimization results back to window state.
  void applyOptimizationResults(SlidingWindow& window,
                                frontend::FeatureManager& f_manager);

  /// @brief Perform Schur-complement marginalization.
  void marginalize(SlidingWindow& window,
                   frontend::FeatureManager& f_manager,
                   MarginalizationInfo*& last_marginalization_info,
                   std::vector<double*>& last_marginalization_parameter_blocks,
                   common::MarginalizationFlag marg_flag);

  // -----------------------------------------------------------------------
  // Marginalization helpers
  // -----------------------------------------------------------------------
  void marginalizeOldKeyframe(
      SlidingWindow& window,
      frontend::FeatureManager& f_manager,
      MarginalizationInfo*& last_marginalization_info,
      std::vector<double*>& last_marginalization_parameter_blocks);

  void marginalizeNewGeneralFrame(
      SlidingWindow& window,
      MarginalizationInfo*& last_marginalization_info,
      std::vector<double*>& last_marginalization_parameter_blocks);

  // -----------------------------------------------------------------------
  // State
  // -----------------------------------------------------------------------
  const config::SlamConfig& config_;
  Eigen::Vector3d t_ic_;
  Eigen::Matrix3d r_ic_;
};

}  // namespace backend
}  // namespace slam

#endif  // SLAM_BACKEND_OPTIMIZER_H_

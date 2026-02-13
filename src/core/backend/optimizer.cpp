// Copyright 2025 Mobile-SLAM Authors. All rights reserved.
// Licensed under the MIT License.

#include "slam/backend/optimizer.h"

#include <algorithm>
#include <cassert>
#include <memory>
#include <unordered_map>
#include <vector>

#include "slam/utility/math_utils.h"

namespace slam {
namespace backend {

// ===========================================================================
// Constructor / Destructor
// ===========================================================================
Optimizer::Optimizer(const config::SlamConfig& config)
    : config_(config),
      t_ic_(Eigen::Vector3d::Zero()),
      r_ic_(Eigen::Matrix3d::Identity()) {}

Optimizer::~Optimizer() = default;

// ===========================================================================
// setExtrinsicParameters
// ===========================================================================
void Optimizer::setExtrinsicParameters(const Eigen::Vector3d& t_ic,
                                       const Eigen::Matrix3d& r_ic) {
  t_ic_ = t_ic;
  r_ic_ = r_ic;
}

// ===========================================================================
// optimize -- Main entry point: solve + marginalize
// ===========================================================================
void Optimizer::optimize(
    SlidingWindow& window,
    frontend::FeatureManager& f_manager,
    MarginalizationInfo*& last_marginalization_info,
    std::vector<double*>& last_marginalization_parameter_blocks,
    common::MarginalizationFlag marg_flag) {
  // Step 1: Prepare parameter arrays from Eigen state
  int num_features = f_manager.getFeatureCount();
  Eigen::VectorXd dep = f_manager.getDepthVector();
  window.stateToParameter(r_ic_, t_ic_, dep, num_features);

  // Step 2: Set up the Ceres problem with parameter blocks
  ceres::Problem problem;
  setupOptimizationProblem(problem, window);

  // Step 3: Add constraint factors
  addMarginalizationFactor(problem, last_marginalization_info,
                           last_marginalization_parameter_blocks);
  addImuFactors(problem, window);
  addFeatureFactors(problem, window, f_manager);

  // Step 4: Solve
  solveCeresProblem(problem);

  // Step 5: Apply results
  applyOptimizationResults(window, f_manager);

  // Step 6: Marginalize
  marginalize(window, f_manager, last_marginalization_info,
              last_marginalization_parameter_blocks, marg_flag);
}

// ===========================================================================
// setupOptimizationProblem
// ===========================================================================
void Optimizer::setupOptimizationProblem(ceres::Problem& problem,
                                         SlidingWindow& window) {
  // Add pose and speed-bias parameter blocks for each window frame
  for (int i = 0; i < common::kWindowSize + 1; ++i) {
    auto* manifold = new PoseLocalParameterization();
    problem.AddParameterBlock(window.para_pose_[i],
                              SlidingWindow::kSizePose, manifold);
    problem.AddParameterBlock(window.para_speed_bias_[i],
                              SlidingWindow::kSizeSpeedBias);
  }

  // Add extrinsic parameter block (fixed during optimization)
  auto* ex_manifold = new PoseLocalParameterization();
  problem.AddParameterBlock(window.para_ex_pose_[0],
                            SlidingWindow::kSizePose, ex_manifold);
  problem.SetParameterBlockConstant(window.para_ex_pose_[0]);
}

// ===========================================================================
// addMarginalizationFactor
// ===========================================================================
void Optimizer::addMarginalizationFactor(
    ceres::Problem& problem,
    MarginalizationInfo* last_marginalization_info,
    const std::vector<double*>& last_marginalization_parameter_blocks) {
  if (last_marginalization_info) {
    auto* marginalization_factor =
        new MarginalizationFactor(last_marginalization_info);
    problem.AddResidualBlock(marginalization_factor, nullptr,
                             last_marginalization_parameter_blocks);
  }
}

// ===========================================================================
// addImuFactors
// ===========================================================================
void Optimizer::addImuFactors(ceres::Problem& problem,
                              SlidingWindow& window) {
  for (int i = 0; i < common::kWindowSize; ++i) {
    int j = i + 1;
    if (window[j].pre_integration_->sum_dt_ > 10.0) {
      continue;
    }
    auto* imu_factor = new ImuFactor(window[j].pre_integration_.get());
    problem.AddResidualBlock(imu_factor, nullptr,
                             window.para_pose_[i],
                             window.para_speed_bias_[i],
                             window.para_pose_[j],
                             window.para_speed_bias_[j]);
  }
}

// ===========================================================================
// addFeatureFactors
// ===========================================================================
int Optimizer::addFeatureFactors(ceres::Problem& problem,
                                 SlidingWindow& window,
                                 frontend::FeatureManager& f_manager) {
  auto* loss_function = new ceres::CauchyLoss(1.0);

  int f_m_cnt = 0;
  int feature_index = -1;

  for (auto& it_per_id : f_manager.features()) {
    it_per_id.used_num_ =
        static_cast<int>(it_per_id.feature_per_frame_.size());
    if (!(it_per_id.used_num_ >= 2 &&
          it_per_id.start_frame_ < common::kWindowSize - 2)) {
      continue;
    }

    ++feature_index;

    int imu_i = it_per_id.start_frame_;
    int imu_j = imu_i - 1;
    Eigen::Vector3d pts_i = it_per_id.feature_per_frame_[0].point_;

    for (auto& it_per_frame : it_per_id.feature_per_frame_) {
      imu_j++;
      if (imu_i == imu_j) {
        continue;
      }
      Eigen::Vector3d pts_j = it_per_frame.point_;
      auto* f = new ProjectionFactor(pts_i, pts_j);
      problem.AddResidualBlock(f, loss_function,
                               window.para_pose_[imu_i],
                               window.para_pose_[imu_j],
                               window.para_ex_pose_[0],
                               window.para_feature_[feature_index]);
      f_m_cnt++;
    }
  }
  return f_m_cnt;
}

// ===========================================================================
// solveCeresProblem
// ===========================================================================
void Optimizer::solveCeresProblem(ceres::Problem& problem) {
  ceres::Solver::Options options;
  options.linear_solver_type = ceres::DENSE_SCHUR;
  options.trust_region_strategy_type = ceres::DOGLEG;
  options.max_solver_time_in_seconds = config_.estimator.solver_time;
  options.max_num_iterations = config_.estimator.num_iterations;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
}

// ===========================================================================
// applyOptimizationResults
// ===========================================================================
void Optimizer::applyOptimizationResults(
    SlidingWindow& window,
    frontend::FeatureManager& f_manager) {
  int num_features = f_manager.getFeatureCount();
  Eigen::VectorXd dep = f_manager.getDepthVector();

  window.parameterToState(r_ic_, t_ic_, dep, num_features);

  f_manager.setDepth(dep);
}

// ===========================================================================
// marginalize -- dispatch to old-keyframe or new-general-frame
// ===========================================================================
void Optimizer::marginalize(
    SlidingWindow& window,
    frontend::FeatureManager& f_manager,
    MarginalizationInfo*& last_marginalization_info,
    std::vector<double*>& last_marginalization_parameter_blocks,
    common::MarginalizationFlag marg_flag) {
  if (marg_flag == common::MarginalizationFlag::kMarginOldKeyframe) {
    marginalizeOldKeyframe(window, f_manager, last_marginalization_info,
                           last_marginalization_parameter_blocks);
  } else {
    marginalizeNewGeneralFrame(window, last_marginalization_info,
                               last_marginalization_parameter_blocks);
  }
}

// ===========================================================================
// marginalizeOldKeyframe
// ===========================================================================
void Optimizer::marginalizeOldKeyframe(
    SlidingWindow& window,
    frontend::FeatureManager& f_manager,
    MarginalizationInfo*& last_marginalization_info,
    std::vector<double*>& last_marginalization_parameter_blocks) {
  auto* marginalization_info = new MarginalizationInfo();

  // Re-prepare parameters (may have changed after applyOptimizationResults)
  int num_features = f_manager.getFeatureCount();
  Eigen::VectorXd dep = f_manager.getDepthVector();
  window.stateToParameter(r_ic_, t_ic_, dep, num_features);

  // -----------------------------------------------------------------------
  // 1. Add prior from last marginalization (if touching frame 0)
  // -----------------------------------------------------------------------
  if (last_marginalization_info) {
    std::vector<int> drop_set;
    for (int i = 0;
         i < static_cast<int>(last_marginalization_parameter_blocks.size());
         ++i) {
      if (last_marginalization_parameter_blocks[i] == window.para_pose_[0] ||
          last_marginalization_parameter_blocks[i] ==
              window.para_speed_bias_[0]) {
        drop_set.push_back(i);
      }
    }
    auto* marg_factor = new MarginalizationFactor(last_marginalization_info);
    auto residual_block_info = std::make_unique<ResidualBlockInfo>(
        marg_factor, nullptr, last_marginalization_parameter_blocks, drop_set);
    marginalization_info->AddResidualBlockInfo(std::move(residual_block_info));
  }

  // -----------------------------------------------------------------------
  // 2. Add IMU factor between frame 0 and frame 1
  // -----------------------------------------------------------------------
  if (window[1].pre_integration_->sum_dt_ < 10.0) {
    auto* imu_factor = new ImuFactor(window[1].pre_integration_.get());
    auto residual_block_info = std::make_unique<ResidualBlockInfo>(
        imu_factor, nullptr,
        std::vector<double*>{window.para_pose_[0],
                             window.para_speed_bias_[0],
                             window.para_pose_[1],
                             window.para_speed_bias_[1]},
        std::vector<int>{0, 1});
    marginalization_info->AddResidualBlockInfo(std::move(residual_block_info));
  }

  // -----------------------------------------------------------------------
  // 3. Add feature factors anchored at frame 0
  // -----------------------------------------------------------------------
  {
    auto* loss_function = new ceres::CauchyLoss(1.0);
    int feature_index = -1;

    for (auto& it_per_id : f_manager.features()) {
      it_per_id.used_num_ =
          static_cast<int>(it_per_id.feature_per_frame_.size());
      if (!(it_per_id.used_num_ >= 2 &&
            it_per_id.start_frame_ < common::kWindowSize - 2)) {
        continue;
      }

      ++feature_index;

      int imu_i = it_per_id.start_frame_;
      int imu_j = imu_i - 1;
      if (imu_i != 0) {
        continue;
      }

      Eigen::Vector3d pts_i = it_per_id.feature_per_frame_[0].point_;

      for (auto& it_per_frame : it_per_id.feature_per_frame_) {
        imu_j++;
        if (imu_i == imu_j) {
          continue;
        }
        Eigen::Vector3d pts_j = it_per_frame.point_;
        auto* f = new ProjectionFactor(pts_i, pts_j);
        auto residual_block_info = std::make_unique<ResidualBlockInfo>(
            f, loss_function,
            std::vector<double*>{window.para_pose_[imu_i],
                                 window.para_pose_[imu_j],
                                 window.para_ex_pose_[0],
                                 window.para_feature_[feature_index]},
            std::vector<int>{0, 3});
        marginalization_info->AddResidualBlockInfo(
            std::move(residual_block_info));
      }
    }
  }

  // -----------------------------------------------------------------------
  // 4. Execute marginalization
  // -----------------------------------------------------------------------
  marginalization_info->PreMarginalize();
  marginalization_info->Marginalize();

  // -----------------------------------------------------------------------
  // 5. Shift addresses: frame i maps to frame i-1
  // -----------------------------------------------------------------------
  std::unordered_map<long, double*> addr_shift;
  for (int i = 1; i <= common::kWindowSize; ++i) {
    addr_shift[reinterpret_cast<long>(window.para_pose_[i])] =
        window.para_pose_[i - 1];
    addr_shift[reinterpret_cast<long>(window.para_speed_bias_[i])] =
        window.para_speed_bias_[i - 1];
  }
  addr_shift[reinterpret_cast<long>(window.para_ex_pose_[0])] =
      window.para_ex_pose_[0];

  std::vector<double*> parameter_blocks =
      marginalization_info->GetParameterBlocks(addr_shift);

  if (last_marginalization_info) {
    delete last_marginalization_info;
  }
  last_marginalization_info = marginalization_info;
  last_marginalization_parameter_blocks = parameter_blocks;
}

// ===========================================================================
// marginalizeNewGeneralFrame
// ===========================================================================
void Optimizer::marginalizeNewGeneralFrame(
    SlidingWindow& window,
    MarginalizationInfo*& last_marginalization_info,
    std::vector<double*>& last_marginalization_parameter_blocks) {
  // Only proceed if the last marginalization touches the second-newest frame
  bool has_second_newest = std::count(
      last_marginalization_parameter_blocks.begin(),
      last_marginalization_parameter_blocks.end(),
      window.para_pose_[common::kWindowSize - 1]);

  if (!has_second_newest) {
    return;
  }

  auto* marginalization_info = new MarginalizationInfo();

  // Re-prepare parameters
  // (feature depths not needed; we only re-marginalize the prior here)
  Eigen::VectorXd empty_depths;
  window.stateToParameter(r_ic_, t_ic_, empty_depths, 0);

  if (last_marginalization_info) {
    std::vector<int> drop_set;
    for (int i = 0;
         i < static_cast<int>(last_marginalization_parameter_blocks.size());
         ++i) {
      assert(last_marginalization_parameter_blocks[i] !=
             window.para_speed_bias_[common::kWindowSize - 1]);
      if (last_marginalization_parameter_blocks[i] ==
          window.para_pose_[common::kWindowSize - 1]) {
        drop_set.push_back(i);
      }
    }
    auto* marg_factor = new MarginalizationFactor(last_marginalization_info);
    auto residual_block_info = std::make_unique<ResidualBlockInfo>(
        marg_factor, nullptr, last_marginalization_parameter_blocks, drop_set);
    marginalization_info->AddResidualBlockInfo(std::move(residual_block_info));
  }

  marginalization_info->PreMarginalize();
  marginalization_info->Marginalize();

  // -----------------------------------------------------------------------
  // Shift addresses: frame WINDOW_SIZE maps to WINDOW_SIZE-1,
  // frame WINDOW_SIZE-1 is dropped, others stay the same.
  // -----------------------------------------------------------------------
  std::unordered_map<long, double*> addr_shift;
  for (int i = 0; i <= common::kWindowSize; ++i) {
    if (i == common::kWindowSize - 1) {
      continue;
    } else if (i == common::kWindowSize) {
      addr_shift[reinterpret_cast<long>(window.para_pose_[i])] =
          window.para_pose_[i - 1];
      addr_shift[reinterpret_cast<long>(window.para_speed_bias_[i])] =
          window.para_speed_bias_[i - 1];
    } else {
      addr_shift[reinterpret_cast<long>(window.para_pose_[i])] =
          window.para_pose_[i];
      addr_shift[reinterpret_cast<long>(window.para_speed_bias_[i])] =
          window.para_speed_bias_[i];
    }
  }
  addr_shift[reinterpret_cast<long>(window.para_ex_pose_[0])] =
      window.para_ex_pose_[0];

  std::vector<double*> parameter_blocks =
      marginalization_info->GetParameterBlocks(addr_shift);

  if (last_marginalization_info) {
    delete last_marginalization_info;
  }
  last_marginalization_info = marginalization_info;
  last_marginalization_parameter_blocks = parameter_blocks;
}

}  // namespace backend
}  // namespace slam

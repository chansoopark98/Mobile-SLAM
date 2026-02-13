// Copyright 2025 Mobile-SLAM Authors. All rights reserved.
// Licensed under the MIT License.

#ifndef SLAM_BACKEND_FACTOR_MARGINALIZATION_FACTOR_H_
#define SLAM_BACKEND_FACTOR_MARGINALIZATION_FACTOR_H_

#include <ceres/ceres.h>
#include <Eigen/Dense>
#include <memory>
#include <numeric>
#include <unordered_map>
#include <vector>

namespace slam {
namespace backend {

// ---------------------------------------------------------------------------
// ResidualBlockInfo -- Wrapper around a single Ceres cost function for
//                      marginalization.
//
// Stores:
//   - The cost function and (optional) loss function
//   - The parameter block pointers involved
//   - The "drop set" (indices of parameter blocks to be marginalized out)
//   - After Evaluate(): the residuals and Jacobians
// ---------------------------------------------------------------------------
struct ResidualBlockInfo {
  ResidualBlockInfo(ceres::CostFunction* cost_function,
                    ceres::LossFunction* loss_function,
                    std::vector<double*> parameter_blocks,
                    std::vector<int> drop_set)
      : cost_function_(cost_function),
        loss_function_(loss_function),
        parameter_blocks_(std::move(parameter_blocks)),
        drop_set_(std::move(drop_set)),
        raw_jacobians_(nullptr) {}

  ~ResidualBlockInfo() {
    // raw_jacobians_ is a dynamically allocated array of pointers
    // (the data they point to is owned by jacobians_ Eigen matrices).
    delete[] raw_jacobians_;
  }

  // Non-copyable, non-movable (pointers to internal state)
  ResidualBlockInfo(const ResidualBlockInfo&) = delete;
  ResidualBlockInfo& operator=(const ResidualBlockInfo&) = delete;

  /// @brief Evaluate the cost function, storing residuals and Jacobians.
  ///        Applies robust loss function scaling if present.
  void Evaluate();

  /// @brief Map global param size to local tangent size (7 -> 6 for SE3).
  static int LocalSize(int size) { return size == 7 ? 6 : size; }

  ceres::CostFunction* cost_function_;   // Not owned (ownership varies)
  ceres::LossFunction* loss_function_;   // Not owned (may be nullptr)
  std::vector<double*> parameter_blocks_;
  std::vector<int> drop_set_;

  double** raw_jacobians_;
  std::vector<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic,
                            Eigen::RowMajor>> jacobians_;
  Eigen::VectorXd residuals_;
};

// ---------------------------------------------------------------------------
// MarginalizationInfo -- Schur-complement marginalization.
//
// Collects all residual blocks that touch the parameter blocks to be
// marginalized.  Builds the Hessian (H * dx = b), applies the Schur
// complement to eliminate the marginalized blocks, and stores the resulting
// linearized prior as (linearized_jacobians_, linearized_residuals_).
//
// Single-threaded Hessian construction for WASM compatibility.
// Uses smart pointers for parameter block data copies.
// ---------------------------------------------------------------------------
class MarginalizationInfo {
 public:
  MarginalizationInfo() = default;
  ~MarginalizationInfo();

  // Non-copyable
  MarginalizationInfo(const MarginalizationInfo&) = delete;
  MarginalizationInfo& operator=(const MarginalizationInfo&) = delete;

  /// @brief Map global param size to local tangent size (7 -> 6).
  int LocalSize(int size) const { return size == 7 ? 6 : size; }

  /// @brief Map local tangent size to global param size (6 -> 7).
  int GlobalSize(int size) const { return size == 6 ? 7 : size; }

  /// @brief Add a residual block to the marginalization problem.
  /// @param residual_block_info  Unique pointer; ownership transferred.
  void AddResidualBlockInfo(
      std::unique_ptr<ResidualBlockInfo> residual_block_info);

  /// @brief Evaluate all factors and snapshot parameter block data.
  void PreMarginalize();

  /// @brief Build Hessian, apply Schur complement, store linearized prior.
  void Marginalize();

  /// @brief Return the kept parameter block addresses (shifted to new memory).
  std::vector<double*> GetParameterBlocks(
      std::unordered_map<long, double*>& addr_shift);

  // -----------------------------------------------------------------------
  // Internal state
  // -----------------------------------------------------------------------

  /// All residual blocks involved in the marginalization.
  std::vector<std::unique_ptr<ResidualBlockInfo>> factors_;

  /// Dimensions of the marginalized (m_) and kept (n_) blocks.
  int m_ = 0;
  int n_ = 0;

  /// Maps parameter-block address -> global size.
  std::unordered_map<long, int> parameter_block_size_;

  /// Maps parameter-block address -> column index in the Hessian (local size).
  std::unordered_map<long, int> parameter_block_idx_;

  /// Snapshots of parameter data (address -> heap-allocated copy).
  /// Using shared_ptr<double[]> for RAII.
  std::unordered_map<long, std::shared_ptr<double>> parameter_block_data_;

  int sum_block_size_ = 0;

  /// Kept block information (for MarginalizationFactor construction).
  std::vector<int> keep_block_size_;    // global size
  std::vector<int> keep_block_idx_;     // local index in Hessian
  std::vector<double*> keep_block_data_;  // raw pointer into snapshots

  /// Linearized prior from Schur complement decomposition.
  Eigen::MatrixXd linearized_jacobians_;
  Eigen::VectorXd linearized_residuals_;

  static constexpr double kEps = 1e-8;
};

// ---------------------------------------------------------------------------
// MarginalizationFactor -- Ceres cost function wrapping a linearized prior.
//
// After MarginalizationInfo::Marginalize(), this factor encodes the prior
// from marginalized states.  It evaluates:
//   residual = linearized_residuals + linearized_jacobians * dx
// where dx is computed from the change of kept parameters since
// linearization.
// ---------------------------------------------------------------------------
class MarginalizationFactor : public ceres::CostFunction {
 public:
  /// @brief Construct from marginalization result.
  /// @param marginalization_info  Not owned; must outlive this factor.
  explicit MarginalizationFactor(MarginalizationInfo* marginalization_info);

  /// @brief Evaluate the linearized prior residual and Jacobians.
  virtual bool Evaluate(double const* const* parameters,
                        double* residuals,
                        double** jacobians) const override;

 private:
  MarginalizationInfo* marginalization_info_;  // Non-owning
};

}  // namespace backend
}  // namespace slam

#endif  // SLAM_BACKEND_FACTOR_MARGINALIZATION_FACTOR_H_

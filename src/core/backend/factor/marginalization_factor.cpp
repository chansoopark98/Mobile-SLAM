// Copyright 2025 Mobile-SLAM Authors. All rights reserved.
// Licensed under the MIT License.

#include "slam/backend/factor/marginalization_factor.h"

#include <algorithm>
#include <cstring>
#include <iostream>

namespace slam {
namespace backend {

// ===========================================================================
// ResidualBlockInfo::Evaluate
// ===========================================================================
void ResidualBlockInfo::Evaluate() {
  residuals_.resize(cost_function_->num_residuals());

  std::vector<int> block_sizes = cost_function_->parameter_block_sizes();
  raw_jacobians_ = new double*[block_sizes.size()];
  jacobians_.resize(block_sizes.size());

  for (int i = 0; i < static_cast<int>(block_sizes.size()); ++i) {
    jacobians_[i].resize(cost_function_->num_residuals(), block_sizes[i]);
    raw_jacobians_[i] = jacobians_[i].data();
  }

  cost_function_->Evaluate(parameter_blocks_.data(),
                            residuals_.data(),
                            raw_jacobians_);

  // -----------------------------------------------------------------------
  // Apply robust loss function (if present) following Ceres convention.
  //
  //   rho[0] = rho(s),   rho[1] = rho'(s),   rho[2] = rho''(s)
  //   where s = ||r||^2
  //
  // The corrected residual and Jacobian follow the "Triggs correction":
  //   r_corrected = sqrt(rho') * (I - alpha * r * r^T / s) * r
  //   J_corrected = sqrt(rho') * (J - alpha * r * r^T * J / s)
  // with  alpha = 1 - sqrt(1 + 2*s*rho''/rho')  clipped.
  // -----------------------------------------------------------------------
  if (loss_function_) {
    double residual_scaling;
    double alpha_sq_norm;

    double sq_norm = residuals_.squaredNorm();
    double rho[3];
    loss_function_->Evaluate(sq_norm, rho);

    double sqrt_rho1 = std::sqrt(rho[1]);

    if ((sq_norm == 0.0) || (rho[2] <= 0.0)) {
      residual_scaling = sqrt_rho1;
      alpha_sq_norm = 0.0;
    } else {
      const double D = 1.0 + 2.0 * sq_norm * rho[2] / rho[1];
      const double alpha = 1.0 - std::sqrt(D);
      residual_scaling = sqrt_rho1 / (1.0 - alpha);
      alpha_sq_norm = alpha / sq_norm;
    }

    for (int i = 0; i < static_cast<int>(parameter_blocks_.size()); ++i) {
      jacobians_[i] =
          sqrt_rho1 * (jacobians_[i] -
                       alpha_sq_norm * residuals_ *
                           (residuals_.transpose() * jacobians_[i]));
    }

    residuals_ *= residual_scaling;
  }
}

// ===========================================================================
// MarginalizationInfo -- destructor
// ===========================================================================
MarginalizationInfo::~MarginalizationInfo() {
  // factors_ are unique_ptr -- auto-cleaned.
  // parameter_block_data_ are shared_ptr<double> with array deleter -- auto-cleaned.
}

// ===========================================================================
// AddResidualBlockInfo
// ===========================================================================
void MarginalizationInfo::AddResidualBlockInfo(
    std::unique_ptr<ResidualBlockInfo> residual_block_info) {
  // Register all parameter blocks with their sizes.
  std::vector<int> block_sizes =
      residual_block_info->cost_function_->parameter_block_sizes();

  for (int i = 0;
       i < static_cast<int>(residual_block_info->parameter_blocks_.size());
       ++i) {
    double* addr = residual_block_info->parameter_blocks_[i];
    int size = block_sizes[i];
    parameter_block_size_[reinterpret_cast<long>(addr)] = size;
  }

  // Mark parameter blocks in the drop set (to be marginalized).
  for (int i = 0;
       i < static_cast<int>(residual_block_info->drop_set_.size()); ++i) {
    double* addr =
        residual_block_info->parameter_blocks_[residual_block_info->drop_set_[i]];
    parameter_block_idx_[reinterpret_cast<long>(addr)] = 0;
  }

  factors_.push_back(std::move(residual_block_info));
}

// ===========================================================================
// PreMarginalize -- Evaluate all factors, snapshot parameter data.
// ===========================================================================
void MarginalizationInfo::PreMarginalize() {
  for (auto& factor : factors_) {
    factor->Evaluate();

    std::vector<int> block_sizes =
        factor->cost_function_->parameter_block_sizes();

    for (int i = 0; i < static_cast<int>(block_sizes.size()); ++i) {
      long addr = reinterpret_cast<long>(factor->parameter_blocks_[i]);
      int size = block_sizes[i];

      if (parameter_block_data_.find(addr) == parameter_block_data_.end()) {
        // Allocate and copy parameter data (shared_ptr with array deleter)
        auto data = std::shared_ptr<double>(
            new double[size], std::default_delete<double[]>());
        std::memcpy(data.get(), factor->parameter_blocks_[i],
                    sizeof(double) * size);
        parameter_block_data_[addr] = data;
      }
    }
  }
}

// ===========================================================================
// ConstructHessian -- Single-threaded Hessian construction (WASM-safe).
//
// Builds A (Hessian) and b (gradient) from all evaluated factors.
// Replaces the pthreads-based multi-threaded version in the reference.
// ===========================================================================
static void ConstructHessian(
    const std::vector<std::unique_ptr<ResidualBlockInfo>>& factors,
    const std::unordered_map<long, int>& parameter_block_size,
    const std::unordered_map<long, int>& parameter_block_idx,
    Eigen::MatrixXd& A,
    Eigen::VectorXd& b) {
  for (const auto& it : factors) {
    for (int i = 0;
         i < static_cast<int>(it->parameter_blocks_.size()); ++i) {
      int idx_i =
          parameter_block_idx.at(reinterpret_cast<long>(it->parameter_blocks_[i]));
      int size_i =
          parameter_block_size.at(reinterpret_cast<long>(it->parameter_blocks_[i]));
      if (size_i == 7) size_i = 6;

      Eigen::MatrixXd jacobian_i = it->jacobians_[i].leftCols(size_i);

      for (int j = i;
           j < static_cast<int>(it->parameter_blocks_.size()); ++j) {
        int idx_j =
            parameter_block_idx.at(
                reinterpret_cast<long>(it->parameter_blocks_[j]));
        int size_j =
            parameter_block_size.at(
                reinterpret_cast<long>(it->parameter_blocks_[j]));
        if (size_j == 7) size_j = 6;

        Eigen::MatrixXd jacobian_j = it->jacobians_[j].leftCols(size_j);

        if (i == j) {
          A.block(idx_i, idx_j, size_i, size_j) +=
              jacobian_i.transpose() * jacobian_j;
        } else {
          A.block(idx_i, idx_j, size_i, size_j) +=
              jacobian_i.transpose() * jacobian_j;
          A.block(idx_j, idx_i, size_j, size_i) =
              A.block(idx_i, idx_j, size_i, size_j).transpose();
        }
      }
      b.segment(idx_i, size_i) +=
          jacobian_i.transpose() * it->residuals_;
    }
  }
}

// ===========================================================================
// Marginalize -- Build Hessian, Schur complement, store linearized prior.
// ===========================================================================
void MarginalizationInfo::Marginalize() {
  // ------------------------------------------------------------------
  // 1. Assign column indices: marginalized blocks first, then kept.
  // ------------------------------------------------------------------
  int pos = 0;

  // Marginalized blocks (already registered in parameter_block_idx_)
  for (auto& it : parameter_block_idx_) {
    it.second = pos;
    pos += LocalSize(parameter_block_size_[it.first]);
  }
  m_ = pos;

  // Kept blocks (all blocks NOT in parameter_block_idx_)
  for (const auto& it : parameter_block_size_) {
    if (parameter_block_idx_.find(it.first) == parameter_block_idx_.end()) {
      parameter_block_idx_[it.first] = pos;
      pos += LocalSize(it.second);
    }
  }
  n_ = pos - m_;

  // ------------------------------------------------------------------
  // 2. Construct Hessian A and gradient b (single-threaded).
  // ------------------------------------------------------------------
  Eigen::MatrixXd A(pos, pos);
  Eigen::VectorXd b(pos);
  A.setZero();
  b.setZero();

  ConstructHessian(factors_, parameter_block_size_, parameter_block_idx_, A, b);

  // ------------------------------------------------------------------
  // 3. Schur complement.
  //
  //    A = [Amm  Amr]     b = [bmm]
  //        [Arm  Arr]         [brr]
  //
  //    A_prior = Arr - Arm * Amm^{-1} * Amr
  //    b_prior = brr - Arm * Amm^{-1} * bmm
  // ------------------------------------------------------------------
  Eigen::MatrixXd Amm =
      0.5 * (A.block(0, 0, m_, m_) + A.block(0, 0, m_, m_).transpose());
  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> saes(Amm);

  // Pseudo-inverse via eigendecomposition (numerical stability)
  Eigen::MatrixXd Amm_inv =
      saes.eigenvectors() *
      Eigen::VectorXd(
          (saes.eigenvalues().array() > kEps)
              .select(saes.eigenvalues().array().inverse(), 0))
          .asDiagonal() *
      saes.eigenvectors().transpose();

  Eigen::VectorXd bmm = b.segment(0, m_);
  Eigen::MatrixXd Amr = A.block(0, m_, m_, n_);
  Eigen::MatrixXd Arm = A.block(m_, 0, n_, m_);
  Eigen::MatrixXd Arr = A.block(m_, m_, n_, n_);
  Eigen::VectorXd brr = b.segment(m_, n_);

  A = Arr - Arm * Amm_inv * Amr;
  b = brr - Arm * Amm_inv * bmm;

  // ------------------------------------------------------------------
  // 4. Decompose the prior Hessian into linearized Jacobian & residual.
  //
  //    A = J^T * J   =>  J = S^{1/2} * V^T
  //    b = J^T * r   =>  r = S^{-1/2} * V^T * b
  //
  //    where A = V * S * V^T (eigendecomposition, S diagonal)
  // ------------------------------------------------------------------
  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> saes2(A);
  Eigen::VectorXd S = Eigen::VectorXd(
      (saes2.eigenvalues().array() > kEps)
          .select(saes2.eigenvalues().array(), 0));
  Eigen::VectorXd S_inv = Eigen::VectorXd(
      (saes2.eigenvalues().array() > kEps)
          .select(saes2.eigenvalues().array().inverse(), 0));

  Eigen::VectorXd S_sqrt = S.cwiseSqrt();
  Eigen::VectorXd S_inv_sqrt = S_inv.cwiseSqrt();

  linearized_jacobians_ =
      S_sqrt.asDiagonal() * saes2.eigenvectors().transpose();
  linearized_residuals_ =
      S_inv_sqrt.asDiagonal() * saes2.eigenvectors().transpose() * b;
}

// ===========================================================================
// GetParameterBlocks -- Return kept block addresses in the new frame.
// ===========================================================================
std::vector<double*> MarginalizationInfo::GetParameterBlocks(
    std::unordered_map<long, double*>& addr_shift) {
  std::vector<double*> keep_block_addr;
  keep_block_size_.clear();
  keep_block_idx_.clear();
  keep_block_data_.clear();

  for (const auto& it : parameter_block_idx_) {
    if (it.second >= m_) {
      keep_block_size_.push_back(parameter_block_size_[it.first]);
      keep_block_idx_.push_back(parameter_block_idx_[it.first]);
      keep_block_data_.push_back(parameter_block_data_[it.first].get());
      keep_block_addr.push_back(addr_shift[it.first]);
    }
  }

  sum_block_size_ = std::accumulate(
      keep_block_size_.begin(), keep_block_size_.end(), 0);

  return keep_block_addr;
}

// ===========================================================================
// MarginalizationFactor -- Constructor
// ===========================================================================
MarginalizationFactor::MarginalizationFactor(
    MarginalizationInfo* marginalization_info)
    : marginalization_info_(marginalization_info) {
  int cnt = 0;
  for (auto it : marginalization_info_->keep_block_size_) {
    mutable_parameter_block_sizes()->push_back(it);
    cnt += it;
  }
  set_num_residuals(marginalization_info_->n_);
}

// ===========================================================================
// MarginalizationFactor::Evaluate
//
// Computes the linearized prior residual:
//   residual = linearized_residuals + linearized_jacobians * dx
//
// where dx is the change of kept parameters since linearization.
// For SE(3) blocks (size 7), dx uses the quaternion log map.
// ===========================================================================
bool MarginalizationFactor::Evaluate(double const* const* parameters,
                                     double* residuals,
                                     double** jacobians) const {
  int n = marginalization_info_->n_;
  int m = marginalization_info_->m_;

  Eigen::VectorXd dx(n);

  for (int i = 0;
       i < static_cast<int>(marginalization_info_->keep_block_size_.size());
       ++i) {
    int size = marginalization_info_->keep_block_size_[i];
    int idx = marginalization_info_->keep_block_idx_[i] - m;

    Eigen::VectorXd x =
        Eigen::Map<const Eigen::VectorXd>(parameters[i], size);
    Eigen::VectorXd x0 =
        Eigen::Map<const Eigen::VectorXd>(
            marginalization_info_->keep_block_data_[i], size);

    if (size != 7) {
      // Generic parameter block: simple difference
      dx.segment(idx, size) = x - x0;
    } else {
      // SE(3) parameter block: position difference + quaternion log
      dx.segment<3>(idx + 0) = x.head<3>() - x0.head<3>();

      // Quaternion difference via log map
      //   dq = q0^{-1} * q
      //   dtheta = 2 * dq.vec()  (small-angle, ensure positive w)
      Eigen::Quaterniond q0(x0(6), x0(3), x0(4), x0(5));
      Eigen::Quaterniond q(x(6), x(3), x(4), x(5));
      Eigen::Quaterniond dq = q0.inverse() * q;

      // Positify: ensure w >= 0 for unique representation
      // (positify returns q as-is per reference implementation)
      if (dq.w() >= 0.0) {
        dx.segment<3>(idx + 3) = 2.0 * dq.vec();
      } else {
        dx.segment<3>(idx + 3) = -2.0 * dq.vec();
      }
    }
  }

  // Linearized prior: r = r0 + J * dx
  Eigen::Map<Eigen::VectorXd>(residuals, n) =
      marginalization_info_->linearized_residuals_ +
      marginalization_info_->linearized_jacobians_ * dx;

  // Jacobians: simply the columns of the linearized Jacobian
  if (jacobians) {
    for (int i = 0;
         i < static_cast<int>(marginalization_info_->keep_block_size_.size());
         ++i) {
      if (jacobians[i]) {
        int size = marginalization_info_->keep_block_size_[i];
        int local_size = marginalization_info_->LocalSize(size);
        int idx = marginalization_info_->keep_block_idx_[i] - m;

        Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic,
                                 Eigen::RowMajor>>
            jacobian(jacobians[i], n, size);
        jacobian.setZero();
        jacobian.leftCols(local_size) =
            marginalization_info_->linearized_jacobians_.middleCols(
                idx, local_size);
      }
    }
  }

  return true;
}

}  // namespace backend
}  // namespace slam

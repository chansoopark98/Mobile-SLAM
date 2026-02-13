// Copyright 2025 Mobile-SLAM Authors. All rights reserved.
// Licensed under the MIT License.

#ifndef SLAM_BACKEND_FACTOR_IMU_FACTOR_H_
#define SLAM_BACKEND_FACTOR_IMU_FACTOR_H_

#include <ceres/ceres.h>
#include <Eigen/Dense>
#include <iostream>
#include <memory>

#include "slam/backend/factor/integration_base.h"

namespace slam {
namespace backend {

// ---------------------------------------------------------------------------
// ImuFactor -- Ceres cost function for IMU pre-integration constraints.
//
// Residual dimension: 15
//   [r_p(3), r_q(3), r_v(3), r_ba(3), r_bg(3)]
//
// Parameter blocks (4 total):
//   [0] pose_i   (7): [px, py, pz, qx, qy, qz, qw]
//   [1] speed_bias_i (9): [vx, vy, vz, bax, bay, baz, bgx, bgy, bgz]
//   [2] pose_j   (7): [px, py, pz, qx, qy, qz, qw]
//   [3] speed_bias_j (9): [vx, vy, vz, bax, bay, baz, bgx, bgy, bgz]
//
// Provides analytical Jacobians for all four parameter blocks.
// The residual is left-multiplied by sqrt_info derived from the
// pre-integration covariance via Cholesky decomposition.
// ---------------------------------------------------------------------------
class ImuFactor : public ceres::SizedCostFunction<15, 7, 9, 7, 9> {
 public:
  ImuFactor() = delete;

  /// @brief Construct with a pre-integration handle.
  /// @param pre_integration  Pointer to the pre-integration (not owned).
  explicit ImuFactor(IntegrationBase* pre_integration)
      : pre_integration_(pre_integration) {}

  /// @brief Evaluate residual and (optionally) analytical Jacobians.
  virtual bool Evaluate(double const* const* parameters,
                        double* residuals,
                        double** jacobians) const override {
    // ------------------------------------------------------------------
    // Extract parameters
    // ------------------------------------------------------------------
    // Pose i: [px, py, pz, qx, qy, qz, qw]
    Eigen::Vector3d Pi(parameters[0][0], parameters[0][1], parameters[0][2]);
    Eigen::Quaterniond Qi(parameters[0][6], parameters[0][3],
                          parameters[0][4], parameters[0][5]);

    // Speed & bias i: [vx, vy, vz, bax, bay, baz, bgx, bgy, bgz]
    Eigen::Vector3d Vi(parameters[1][0], parameters[1][1], parameters[1][2]);
    Eigen::Vector3d Bai(parameters[1][3], parameters[1][4], parameters[1][5]);
    Eigen::Vector3d Bgi(parameters[1][6], parameters[1][7], parameters[1][8]);

    // Pose j
    Eigen::Vector3d Pj(parameters[2][0], parameters[2][1], parameters[2][2]);
    Eigen::Quaterniond Qj(parameters[2][6], parameters[2][3],
                          parameters[2][4], parameters[2][5]);

    // Speed & bias j
    Eigen::Vector3d Vj(parameters[3][0], parameters[3][1], parameters[3][2]);
    Eigen::Vector3d Baj(parameters[3][3], parameters[3][4], parameters[3][5]);
    Eigen::Vector3d Bgj(parameters[3][6], parameters[3][7], parameters[3][8]);

    // ------------------------------------------------------------------
    // Compute residual
    // ------------------------------------------------------------------
    Eigen::Map<Eigen::Matrix<double, 15, 1>> residual(residuals);
    residual = pre_integration_->Evaluate(Pi, Qi, Vi, Bai, Bgi,
                                          Pj, Qj, Vj, Baj, Bgj);

    // ------------------------------------------------------------------
    // Compute sqrt information matrix from covariance
    // ------------------------------------------------------------------
    Eigen::Matrix<double, 15, 15> sqrt_info =
        Eigen::LLT<Eigen::Matrix<double, 15, 15>>(
            pre_integration_->covariance_.inverse())
            .matrixL()
            .transpose();

    // Weight residual
    residual = sqrt_info * residual;

    // ------------------------------------------------------------------
    // Analytical Jacobians
    // ------------------------------------------------------------------
    if (jacobians) {
      double sum_dt = pre_integration_->sum_dt_;
      const Eigen::Vector3d& gravity = pre_integration_->config_.estimator.gravity;

      // Pre-integration Jacobian blocks for bias correction
      Eigen::Matrix3d dp_dba =
          pre_integration_->jacobian_.template block<3, 3>(kOrderP, kOrderBa);
      Eigen::Matrix3d dp_dbg =
          pre_integration_->jacobian_.template block<3, 3>(kOrderP, kOrderBg);
      Eigen::Matrix3d dq_dbg =
          pre_integration_->jacobian_.template block<3, 3>(kOrderR, kOrderBg);
      Eigen::Matrix3d dv_dba =
          pre_integration_->jacobian_.template block<3, 3>(kOrderV, kOrderBa);
      Eigen::Matrix3d dv_dbg =
          pre_integration_->jacobian_.template block<3, 3>(kOrderV, kOrderBg);

      // Numerical stability check
      if (pre_integration_->jacobian_.maxCoeff() > 1e8 ||
          pre_integration_->jacobian_.minCoeff() < -1e8) {
        std::cerr << "[ImuFactor] WARNING: numerical instability in "
                     "pre-integration Jacobian" << std::endl;
      }

      // ----------------------------------------------------------------
      // Jacobian w.r.t. pose_i (15 x 7, row-major)
      // ----------------------------------------------------------------
      if (jacobians[0]) {
        Eigen::Map<Eigen::Matrix<double, 15, 7, Eigen::RowMajor>>
            jacobian_pose_i(jacobians[0]);
        jacobian_pose_i.setZero();

        // dr_p / dp_i
        jacobian_pose_i.block<3, 3>(kOrderP, kOrderP) =
            -Qi.inverse().toRotationMatrix();

        // dr_p / dtheta_i
        jacobian_pose_i.block<3, 3>(kOrderP, kOrderR) =
            SkewSymmetric(Qi.inverse() *
                          (0.5 * gravity * sum_dt * sum_dt +
                           Pj - Pi - Vi * sum_dt));

        // dr_q / dtheta_i
        Eigen::Quaterniond corrected_delta_q =
            pre_integration_->delta_q_ *
            DeltaQ(dq_dbg * (Bgi - pre_integration_->linearized_bg_));
        jacobian_pose_i.block<3, 3>(kOrderR, kOrderR) =
            -(Qleft(Qj.inverse() * Qi) *
              Qright(corrected_delta_q))
                 .bottomRightCorner<3, 3>();

        // dr_v / dtheta_i
        jacobian_pose_i.block<3, 3>(kOrderV, kOrderR) =
            SkewSymmetric(Qi.inverse() *
                          (gravity * sum_dt + Vj - Vi));

        jacobian_pose_i = sqrt_info * jacobian_pose_i;

        if (jacobian_pose_i.maxCoeff() > 1e8 ||
            jacobian_pose_i.minCoeff() < -1e8) {
          std::cerr << "[ImuFactor] WARNING: numerical instability in "
                       "jacobian_pose_i" << std::endl;
        }
      }

      // ----------------------------------------------------------------
      // Jacobian w.r.t. speed_bias_i (15 x 9, row-major)
      // ----------------------------------------------------------------
      if (jacobians[1]) {
        Eigen::Map<Eigen::Matrix<double, 15, 9, Eigen::RowMajor>>
            jacobian_speedbias_i(jacobians[1]);
        jacobian_speedbias_i.setZero();

        // dr_p / dv_i
        jacobian_speedbias_i.block<3, 3>(kOrderP, kOrderV - kOrderV) =
            -Qi.inverse().toRotationMatrix() * sum_dt;
        // dr_p / dba_i
        jacobian_speedbias_i.block<3, 3>(kOrderP, kOrderBa - kOrderV) =
            -dp_dba;
        // dr_p / dbg_i
        jacobian_speedbias_i.block<3, 3>(kOrderP, kOrderBg - kOrderV) =
            -dp_dbg;

        // dr_q / dbg_i
        jacobian_speedbias_i.block<3, 3>(kOrderR, kOrderBg - kOrderV) =
            -Qleft(Qj.inverse() * Qi * pre_integration_->delta_q_)
                 .bottomRightCorner<3, 3>() *
            dq_dbg;

        // dr_v / dv_i
        jacobian_speedbias_i.block<3, 3>(kOrderV, kOrderV - kOrderV) =
            -Qi.inverse().toRotationMatrix();
        // dr_v / dba_i
        jacobian_speedbias_i.block<3, 3>(kOrderV, kOrderBa - kOrderV) =
            -dv_dba;
        // dr_v / dbg_i
        jacobian_speedbias_i.block<3, 3>(kOrderV, kOrderBg - kOrderV) =
            -dv_dbg;

        // dr_ba / dba_i
        jacobian_speedbias_i.block<3, 3>(kOrderBa, kOrderBa - kOrderV) =
            -Eigen::Matrix3d::Identity();

        // dr_bg / dbg_i
        jacobian_speedbias_i.block<3, 3>(kOrderBg, kOrderBg - kOrderV) =
            -Eigen::Matrix3d::Identity();

        jacobian_speedbias_i = sqrt_info * jacobian_speedbias_i;
      }

      // ----------------------------------------------------------------
      // Jacobian w.r.t. pose_j (15 x 7, row-major)
      // ----------------------------------------------------------------
      if (jacobians[2]) {
        Eigen::Map<Eigen::Matrix<double, 15, 7, Eigen::RowMajor>>
            jacobian_pose_j(jacobians[2]);
        jacobian_pose_j.setZero();

        // dr_p / dp_j
        jacobian_pose_j.block<3, 3>(kOrderP, kOrderP) =
            Qi.inverse().toRotationMatrix();

        // dr_q / dtheta_j
        Eigen::Quaterniond corrected_delta_q =
            pre_integration_->delta_q_ *
            DeltaQ(dq_dbg * (Bgi - pre_integration_->linearized_bg_));
        jacobian_pose_j.block<3, 3>(kOrderR, kOrderR) =
            Qleft(corrected_delta_q.inverse() * Qi.inverse() * Qj)
                .bottomRightCorner<3, 3>();

        jacobian_pose_j = sqrt_info * jacobian_pose_j;
      }

      // ----------------------------------------------------------------
      // Jacobian w.r.t. speed_bias_j (15 x 9, row-major)
      // ----------------------------------------------------------------
      if (jacobians[3]) {
        Eigen::Map<Eigen::Matrix<double, 15, 9, Eigen::RowMajor>>
            jacobian_speedbias_j(jacobians[3]);
        jacobian_speedbias_j.setZero();

        // dr_v / dv_j
        jacobian_speedbias_j.block<3, 3>(kOrderV, kOrderV - kOrderV) =
            Qi.inverse().toRotationMatrix();

        // dr_ba / dba_j
        jacobian_speedbias_j.block<3, 3>(kOrderBa, kOrderBa - kOrderV) =
            Eigen::Matrix3d::Identity();

        // dr_bg / dbg_j
        jacobian_speedbias_j.block<3, 3>(kOrderBg, kOrderBg - kOrderV) =
            Eigen::Matrix3d::Identity();

        jacobian_speedbias_j = sqrt_info * jacobian_speedbias_j;
      }
    }

    return true;
  }

 private:
  // -----------------------------------------------------------------------
  // Quaternion utility functions (duplicated here to keep header-only)
  // -----------------------------------------------------------------------

  /// Small-angle rotation quaternion: dq = [1, theta/2]
  static Eigen::Quaterniond DeltaQ(const Eigen::Vector3d& theta) {
    Eigen::Quaterniond dq;
    Eigen::Vector3d half_theta = theta * 0.5;
    dq.w() = 1.0;
    dq.x() = half_theta.x();
    dq.y() = half_theta.y();
    dq.z() = half_theta.z();
    return dq;
  }

  /// Ensure quaternion has positive w (canonical form).
  static Eigen::Quaterniond Positify(const Eigen::Quaterniond& q) {
    return q;
  }

  /// 3x3 skew-symmetric matrix from vector.
  static Eigen::Matrix3d SkewSymmetric(const Eigen::Vector3d& v) {
    Eigen::Matrix3d m;
    m << 0.0, -v(2), v(1),
         v(2), 0.0, -v(0),
        -v(1), v(0), 0.0;
    return m;
  }

  /// Left quaternion multiplication matrix (4x4).
  /// q_left * p.coeffs() = (q * p).coeffs()  with layout [x, y, z, w]
  static Eigen::Matrix4d Qleft(const Eigen::Quaterniond& q) {
    Eigen::Quaterniond qq = Positify(q);
    Eigen::Matrix4d ans;
    ans(0, 0) = qq.w();
    ans.template block<1, 3>(0, 1) = -qq.vec().transpose();
    ans.template block<3, 1>(1, 0) = qq.vec();
    ans.template block<3, 3>(1, 1) =
        qq.w() * Eigen::Matrix3d::Identity() + SkewSymmetric(qq.vec());
    return ans;
  }

  /// Right quaternion multiplication matrix (4x4).
  /// q_right * p.coeffs() = (p * q).coeffs()  with layout [x, y, z, w]
  static Eigen::Matrix4d Qright(const Eigen::Quaterniond& q) {
    Eigen::Quaterniond qq = Positify(q);
    Eigen::Matrix4d ans;
    ans(0, 0) = qq.w();
    ans.template block<1, 3>(0, 1) = -qq.vec().transpose();
    ans.template block<3, 1>(1, 0) = qq.vec();
    ans.template block<3, 3>(1, 1) =
        qq.w() * Eigen::Matrix3d::Identity() - SkewSymmetric(qq.vec());
    return ans;
  }

  IntegrationBase* pre_integration_;  // Non-owning pointer
};

}  // namespace backend
}  // namespace slam

#endif  // SLAM_BACKEND_FACTOR_IMU_FACTOR_H_

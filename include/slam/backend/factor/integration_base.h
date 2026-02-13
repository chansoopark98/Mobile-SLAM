// Copyright 2025 Mobile-SLAM Authors. All rights reserved.
// Licensed under the MIT License.

#ifndef SLAM_BACKEND_FACTOR_INTEGRATION_BASE_H_
#define SLAM_BACKEND_FACTOR_INTEGRATION_BASE_H_

#include <Eigen/Dense>
#include <vector>

#include "slam/config/slam_config.h"

namespace slam {
namespace backend {

// ---------------------------------------------------------------------------
// State-order indices for the 15-DOF pre-integration state.
// Matches the convention used across the entire SLAM pipeline.
//   [delta_p(3), delta_q(3), delta_v(3), bias_a(3), bias_g(3)]
// ---------------------------------------------------------------------------
enum StateOrder : int {
  kOrderP  = 0,
  kOrderR  = 3,
  kOrderV  = 6,
  kOrderBa = 9,
  kOrderBg = 12
};

// ---------------------------------------------------------------------------
// IntegrationBase -- Mid-point IMU pre-integration
//
// Integrates gyroscope and accelerometer measurements between two keyframes
// using the mid-point method.  Maintains:
//   - Pre-integrated delta state  (delta_p_, delta_q_, delta_v_)
//   - Jacobian of the delta state w.r.t. linearization-point biases (15x15)
//   - Covariance of the delta state (15x15)
//   - First-order bias correction via stored Jacobian blocks
//
// Reference: VINS-Mono, Forster et al. "On-Manifold Preintegration for
//            Real-Time Visual-Inertial Odometry"
// ---------------------------------------------------------------------------
class IntegrationBase {
 public:
  IntegrationBase() = delete;

  /// @brief Construct with first IMU sample and linearization-point biases.
  /// @param acc_0  First accelerometer measurement (m/s^2, body frame).
  /// @param gyr_0  First gyroscope measurement (rad/s, body frame).
  /// @param linearized_ba  Accelerometer bias linearization point.
  /// @param linearized_bg  Gyroscope bias linearization point.
  /// @param cfg  Reference to the SLAM configuration (for noise parameters).
  IntegrationBase(const Eigen::Vector3d& acc_0,
                  const Eigen::Vector3d& gyr_0,
                  const Eigen::Vector3d& linearized_ba,
                  const Eigen::Vector3d& linearized_bg,
                  const config::SlamConfig& cfg)
      : acc_0_(acc_0),
        gyr_0_(gyr_0),
        linearized_acc_(acc_0),
        linearized_gyr_(gyr_0),
        linearized_ba_(linearized_ba),
        linearized_bg_(linearized_bg),
        jacobian_(Eigen::Matrix<double, 15, 15>::Identity()),
        covariance_(Eigen::Matrix<double, 15, 15>::Zero()),
        sum_dt_(0.0),
        delta_p_(Eigen::Vector3d::Zero()),
        delta_q_(Eigen::Quaterniond::Identity()),
        delta_v_(Eigen::Vector3d::Zero()),
        config_(cfg) {
    // Build the 18x18 continuous-time noise matrix.
    // Layout: [na_0(3), ng_0(3), na_1(3), ng_1(3), nba(3), nbg(3)]
    noise_.setZero();
    const double acc_n = config_.estimator.acc_n;
    const double gyr_n = config_.estimator.gyr_n;
    const double acc_w = config_.estimator.acc_w;
    const double gyr_w = config_.estimator.gyr_w;
    noise_.block<3, 3>(0, 0)   = (acc_n * acc_n) * Eigen::Matrix3d::Identity();
    noise_.block<3, 3>(3, 3)   = (gyr_n * gyr_n) * Eigen::Matrix3d::Identity();
    noise_.block<3, 3>(6, 6)   = (acc_n * acc_n) * Eigen::Matrix3d::Identity();
    noise_.block<3, 3>(9, 9)   = (gyr_n * gyr_n) * Eigen::Matrix3d::Identity();
    noise_.block<3, 3>(12, 12) = (acc_w * acc_w) * Eigen::Matrix3d::Identity();
    noise_.block<3, 3>(15, 15) = (gyr_w * gyr_w) * Eigen::Matrix3d::Identity();
  }

  // -----------------------------------------------------------------------
  // push_back -- Buffer a new IMU measurement and propagate.
  // -----------------------------------------------------------------------
  void push_back(double dt,
                 const Eigen::Vector3d& acc,
                 const Eigen::Vector3d& gyr) {
    dt_buf_.push_back(dt);
    acc_buf_.push_back(acc);
    gyr_buf_.push_back(gyr);
    Propagate(dt, acc, gyr);
  }

  // -----------------------------------------------------------------------
  // repropagate -- Re-integrate all buffered measurements from scratch
  //                with updated bias linearization points.
  // -----------------------------------------------------------------------
  void Repropagate(const Eigen::Vector3d& linearized_ba,
                   const Eigen::Vector3d& linearized_bg) {
    sum_dt_ = 0.0;
    acc_0_ = linearized_acc_;
    gyr_0_ = linearized_gyr_;
    delta_p_.setZero();
    delta_q_.setIdentity();
    delta_v_.setZero();
    linearized_ba_ = linearized_ba;
    linearized_bg_ = linearized_bg;
    jacobian_.setIdentity();
    covariance_.setZero();
    for (int i = 0; i < static_cast<int>(dt_buf_.size()); ++i) {
      Propagate(dt_buf_[i], acc_buf_[i], gyr_buf_[i]);
    }
  }

  // -----------------------------------------------------------------------
  // Evaluate -- Compute the 15-DOF residual between two frames given
  //             their states and the pre-integrated measurements.
  //
  // The residual is ordered as:
  //   [r_p(3), r_q(3), r_v(3), r_ba(3), r_bg(3)]
  //
  // Bias correction is applied via first-order Taylor expansion using
  // the Jacobian blocks stored during propagation.
  // -----------------------------------------------------------------------
  Eigen::Matrix<double, 15, 1> Evaluate(
      const Eigen::Vector3d& Pi, const Eigen::Quaterniond& Qi,
      const Eigen::Vector3d& Vi, const Eigen::Vector3d& Bai,
      const Eigen::Vector3d& Bgi,
      const Eigen::Vector3d& Pj, const Eigen::Quaterniond& Qj,
      const Eigen::Vector3d& Vj, const Eigen::Vector3d& Baj,
      const Eigen::Vector3d& Bgj) const {
    Eigen::Matrix<double, 15, 1> residuals;

    // Extract Jacobian blocks for first-order bias correction
    Eigen::Matrix3d dp_dba = jacobian_.block<3, 3>(kOrderP, kOrderBa);
    Eigen::Matrix3d dp_dbg = jacobian_.block<3, 3>(kOrderP, kOrderBg);
    Eigen::Matrix3d dq_dbg = jacobian_.block<3, 3>(kOrderR, kOrderBg);
    Eigen::Matrix3d dv_dba = jacobian_.block<3, 3>(kOrderV, kOrderBa);
    Eigen::Matrix3d dv_dbg = jacobian_.block<3, 3>(kOrderV, kOrderBg);

    // Bias changes from linearization point
    Eigen::Vector3d dba = Bai - linearized_ba_;
    Eigen::Vector3d dbg = Bgi - linearized_bg_;

    // Apply first-order bias correction to pre-integrated measurements
    Eigen::Quaterniond corrected_delta_q =
        delta_q_ * DeltaQ(dq_dbg * dbg);
    Eigen::Vector3d corrected_delta_v =
        delta_v_ + dv_dba * dba + dv_dbg * dbg;
    Eigen::Vector3d corrected_delta_p =
        delta_p_ + dp_dba * dba + dp_dbg * dbg;

    const Eigen::Vector3d& gravity = config_.estimator.gravity;

    // Position residual
    residuals.block<3, 1>(kOrderP, 0) =
        Qi.inverse() * (0.5 * gravity * sum_dt_ * sum_dt_ +
                         Pj - Pi - Vi * sum_dt_) -
        corrected_delta_p;

    // Rotation residual (2 * quaternion error vector part)
    residuals.block<3, 1>(kOrderR, 0) =
        2.0 * (corrected_delta_q.inverse() * (Qi.inverse() * Qj)).vec();

    // Velocity residual
    residuals.block<3, 1>(kOrderV, 0) =
        Qi.inverse() * (gravity * sum_dt_ + Vj - Vi) -
        corrected_delta_v;

    // Bias residuals (random walk)
    residuals.block<3, 1>(kOrderBa, 0) = Baj - Bai;
    residuals.block<3, 1>(kOrderBg, 0) = Bgj - Bgi;

    return residuals;
  }

  // -----------------------------------------------------------------------
  // Public state -- accessible by ImuFactor for Jacobian computation
  // -----------------------------------------------------------------------
  double dt_;
  Eigen::Vector3d acc_0_, gyr_0_;
  Eigen::Vector3d acc_1_, gyr_1_;

  const Eigen::Vector3d linearized_acc_;
  const Eigen::Vector3d linearized_gyr_;
  Eigen::Vector3d linearized_ba_, linearized_bg_;

  Eigen::Matrix<double, 15, 15> jacobian_;
  Eigen::Matrix<double, 15, 15> covariance_;
  Eigen::Matrix<double, 18, 18> noise_;

  double sum_dt_;
  Eigen::Vector3d delta_p_;
  Eigen::Quaterniond delta_q_;
  Eigen::Vector3d delta_v_;

  std::vector<double> dt_buf_;
  std::vector<Eigen::Vector3d> acc_buf_;
  std::vector<Eigen::Vector3d> gyr_buf_;

  const config::SlamConfig& config_;

 private:
  // -----------------------------------------------------------------------
  // DeltaQ -- Small-angle rotation quaternion from an angle-axis vector.
  //   dq = [1, theta/2]  (first-order approximation, then normalized)
  // -----------------------------------------------------------------------
  static Eigen::Quaterniond DeltaQ(const Eigen::Vector3d& theta) {
    Eigen::Quaterniond dq;
    Eigen::Vector3d half_theta = theta * 0.5;
    dq.w() = 1.0;
    dq.x() = half_theta.x();
    dq.y() = half_theta.y();
    dq.z() = half_theta.z();
    return dq;
  }

  // -----------------------------------------------------------------------
  // SkewSymmetric -- 3x3 skew-symmetric matrix from a vector.
  // -----------------------------------------------------------------------
  static Eigen::Matrix3d SkewSymmetric(const Eigen::Vector3d& v) {
    Eigen::Matrix3d m;
    m << 0.0, -v(2), v(1),
         v(2), 0.0, -v(0),
        -v(1), v(0), 0.0;
    return m;
  }

  // -----------------------------------------------------------------------
  // MidPointIntegration -- Core mid-point integration step.
  //
  // Integrates one IMU interval [acc_0, gyr_0] -> [acc_1, gyr_1] over dt.
  // Optionally updates the Jacobian and covariance (F, V matrices).
  // -----------------------------------------------------------------------
  void MidPointIntegration(
      double dt,
      const Eigen::Vector3d& acc_0, const Eigen::Vector3d& gyr_0,
      const Eigen::Vector3d& acc_1, const Eigen::Vector3d& gyr_1,
      const Eigen::Vector3d& delta_p, const Eigen::Quaterniond& delta_q,
      const Eigen::Vector3d& delta_v,
      const Eigen::Vector3d& linearized_ba,
      const Eigen::Vector3d& linearized_bg,
      Eigen::Vector3d& result_delta_p,
      Eigen::Quaterniond& result_delta_q,
      Eigen::Vector3d& result_delta_v,
      Eigen::Vector3d& result_linearized_ba,
      Eigen::Vector3d& result_linearized_bg,
      bool update_jacobian) {
    // --- Mid-point state integration ---
    // Bias-corrected acc in pre-integrated frame at time 0
    Eigen::Vector3d un_acc_0 = delta_q * (acc_0 - linearized_ba);

    // Mid-point gyro (bias-corrected)
    Eigen::Vector3d un_gyr = 0.5 * (gyr_0 + gyr_1) - linearized_bg;

    // Incremental rotation quaternion
    result_delta_q = delta_q * Eigen::Quaterniond(
        1.0,
        un_gyr(0) * dt / 2.0,
        un_gyr(1) * dt / 2.0,
        un_gyr(2) * dt / 2.0);

    // Bias-corrected acc in pre-integrated frame at time 1
    Eigen::Vector3d un_acc_1 = result_delta_q * (acc_1 - linearized_ba);

    // Mid-point acceleration
    Eigen::Vector3d un_acc = 0.5 * (un_acc_0 + un_acc_1);

    // State update
    result_delta_p = delta_p + delta_v * dt + 0.5 * un_acc * dt * dt;
    result_delta_v = delta_v + un_acc * dt;

    // Biases unchanged during integration
    result_linearized_ba = linearized_ba;
    result_linearized_bg = linearized_bg;

    // --- Jacobian and covariance propagation ---
    if (update_jacobian) {
      // Skew-symmetric matrices for Jacobian computation
      Eigen::Vector3d w_x = 0.5 * (gyr_0 + gyr_1) - linearized_bg;
      Eigen::Vector3d a_0_x = acc_0 - linearized_ba;
      Eigen::Vector3d a_1_x = acc_1 - linearized_ba;

      Eigen::Matrix3d R_w_x = SkewSymmetric(w_x);
      Eigen::Matrix3d R_a_0_x = SkewSymmetric(a_0_x);
      Eigen::Matrix3d R_a_1_x = SkewSymmetric(a_1_x);

      // --- F matrix (15x15 state transition Jacobian) ---
      Eigen::Matrix<double, 15, 15> F = Eigen::Matrix<double, 15, 15>::Zero();

      // dp/dp = I
      F.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
      // dp/dtheta
      F.block<3, 3>(0, 3) =
          -0.25 * delta_q.toRotationMatrix() * R_a_0_x * dt * dt +
          -0.25 * result_delta_q.toRotationMatrix() * R_a_1_x *
              (Eigen::Matrix3d::Identity() - R_w_x * dt) * dt * dt;
      // dp/dv = I*dt
      F.block<3, 3>(0, 6) = Eigen::Matrix3d::Identity() * dt;
      // dp/dba
      F.block<3, 3>(0, 9) =
          -0.25 * (delta_q.toRotationMatrix() +
                   result_delta_q.toRotationMatrix()) * dt * dt;
      // dp/dbg
      F.block<3, 3>(0, 12) =
          -0.25 * result_delta_q.toRotationMatrix() * R_a_1_x *
              dt * dt * (-dt);

      // dtheta/dtheta
      F.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity() - R_w_x * dt;
      // dtheta/dbg
      F.block<3, 3>(3, 12) = -Eigen::Matrix3d::Identity() * dt;

      // dv/dtheta
      F.block<3, 3>(6, 3) =
          -0.5 * delta_q.toRotationMatrix() * R_a_0_x * dt +
          -0.5 * result_delta_q.toRotationMatrix() * R_a_1_x *
              (Eigen::Matrix3d::Identity() - R_w_x * dt) * dt;
      // dv/dv = I
      F.block<3, 3>(6, 6) = Eigen::Matrix3d::Identity();
      // dv/dba
      F.block<3, 3>(6, 9) =
          -0.5 * (delta_q.toRotationMatrix() +
                  result_delta_q.toRotationMatrix()) * dt;
      // dv/dbg
      F.block<3, 3>(6, 12) =
          -0.5 * result_delta_q.toRotationMatrix() * R_a_1_x * dt * (-dt);

      // dba/dba = I
      F.block<3, 3>(9, 9) = Eigen::Matrix3d::Identity();
      // dbg/dbg = I
      F.block<3, 3>(12, 12) = Eigen::Matrix3d::Identity();

      // --- V matrix (15x18 noise-input Jacobian) ---
      Eigen::Matrix<double, 15, 18> V = Eigen::Matrix<double, 15, 18>::Zero();

      // dp w.r.t. na_0
      V.block<3, 3>(0, 0) = 0.25 * delta_q.toRotationMatrix() * dt * dt;
      // dp w.r.t. ng_0
      V.block<3, 3>(0, 3) =
          0.25 * -result_delta_q.toRotationMatrix() * R_a_1_x *
              dt * dt * 0.5 * dt;
      // dp w.r.t. na_1
      V.block<3, 3>(0, 6) = 0.25 * result_delta_q.toRotationMatrix() * dt * dt;
      // dp w.r.t. ng_1
      V.block<3, 3>(0, 9) = V.block<3, 3>(0, 3);

      // dtheta w.r.t. ng_0
      V.block<3, 3>(3, 3) = 0.5 * Eigen::Matrix3d::Identity() * dt;
      // dtheta w.r.t. ng_1
      V.block<3, 3>(3, 9) = 0.5 * Eigen::Matrix3d::Identity() * dt;

      // dv w.r.t. na_0
      V.block<3, 3>(6, 0) = 0.5 * delta_q.toRotationMatrix() * dt;
      // dv w.r.t. ng_0
      V.block<3, 3>(6, 3) =
          0.5 * -result_delta_q.toRotationMatrix() * R_a_1_x * dt * 0.5 * dt;
      // dv w.r.t. na_1
      V.block<3, 3>(6, 6) = 0.5 * result_delta_q.toRotationMatrix() * dt;
      // dv w.r.t. ng_1
      V.block<3, 3>(6, 9) = V.block<3, 3>(6, 3);

      // dba w.r.t. nba
      V.block<3, 3>(9, 12) = Eigen::Matrix3d::Identity() * dt;
      // dbg w.r.t. nbg
      V.block<3, 3>(12, 15) = Eigen::Matrix3d::Identity() * dt;

      // Propagate Jacobian and covariance
      jacobian_ = F * jacobian_;
      covariance_ = F * covariance_ * F.transpose() +
                    V * noise_ * V.transpose();
    }
  }

  // -----------------------------------------------------------------------
  // Propagate -- Single-step propagation using mid-point integration.
  // -----------------------------------------------------------------------
  void Propagate(double dt,
                 const Eigen::Vector3d& acc_1,
                 const Eigen::Vector3d& gyr_1) {
    dt_ = dt;
    acc_1_ = acc_1;
    gyr_1_ = gyr_1;

    Eigen::Vector3d result_delta_p;
    Eigen::Quaterniond result_delta_q;
    Eigen::Vector3d result_delta_v;
    Eigen::Vector3d result_linearized_ba;
    Eigen::Vector3d result_linearized_bg;

    MidPointIntegration(dt, acc_0_, gyr_0_, acc_1, gyr_1,
                        delta_p_, delta_q_, delta_v_,
                        linearized_ba_, linearized_bg_,
                        result_delta_p, result_delta_q, result_delta_v,
                        result_linearized_ba, result_linearized_bg,
                        true);

    delta_p_ = result_delta_p;
    delta_q_ = result_delta_q;
    delta_v_ = result_delta_v;
    linearized_ba_ = result_linearized_ba;
    linearized_bg_ = result_linearized_bg;
    delta_q_.normalize();
    sum_dt_ += dt;
    acc_0_ = acc_1;
    gyr_0_ = gyr_1;
  }
};

}  // namespace backend
}  // namespace slam

#endif  // SLAM_BACKEND_FACTOR_INTEGRATION_BASE_H_

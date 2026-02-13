// Copyright 2025 Mobile-SLAM Authors. All rights reserved.
// Licensed under the MIT License.

#include "slam/frontend/initialization/initial_alignment.h"

#include <cmath>
#include <iostream>

namespace slam {
namespace frontend {

namespace {

/// @brief Compute a 3x2 tangent-space basis orthogonal to the gravity vector.
Eigen::MatrixXd TangentBasis(Eigen::Vector3d& g0) {
  Eigen::Vector3d a = g0.normalized();
  Eigen::Vector3d tmp(0, 0, 1);
  if (a == tmp) tmp << 1, 0, 0;
  Eigen::Vector3d b = (tmp - a * (a.transpose() * tmp)).normalized();
  Eigen::Vector3d c = a.cross(b);
  Eigen::MatrixXd bc(3, 2);
  bc.block<3, 1>(0, 0) = b;
  bc.block<3, 1>(0, 1) = c;
  return bc;
}

}  // anonymous namespace

// ---------------------------------------------------------------------------
// Solve gyroscope bias from visual rotation and IMU pre-integration.
// ---------------------------------------------------------------------------

void solveGyroscopeBias(
    std::map<double, common::ImageFrame>& all_image_frame,
    Eigen::Vector3d* Bgs) {
  Eigen::Matrix3d A = Eigen::Matrix3d::Zero();
  Eigen::Vector3d b = Eigen::Vector3d::Zero();

  auto frame_i = all_image_frame.begin();
  for (; std::next(frame_i) != all_image_frame.end(); ++frame_i) {
    auto frame_j = std::next(frame_i);

    Eigen::MatrixXd tmp_A(3, 3);
    tmp_A.setZero();
    Eigen::VectorXd tmp_b(3);
    tmp_b.setZero();

    Eigen::Quaterniond q_ij(
        frame_i->second.R_.transpose() * frame_j->second.R_);

    tmp_A = frame_j->second.pre_integration_->jacobian_
                .template block<3, 3>(backend::kOrderR, backend::kOrderBg);
    tmp_b =
        2.0 *
        (frame_j->second.pre_integration_->delta_q_.inverse() * q_ij).vec();

    A += tmp_A.transpose() * tmp_A;
    b += tmp_A.transpose() * tmp_b;
  }

  Eigen::Vector3d delta_bg = A.ldlt().solve(b);
  std::cout << "gyroscope bias initial calibration " << delta_bg.transpose()
            << std::endl;

  // Update gyroscope bias for all sliding-window frames.
  for (int i = 0; i <= common::kWindowSize; i++) {
    Bgs[i] += delta_bg;
  }

  // Re-propagate all pre-integrations with the updated bias.
  for (frame_i = all_image_frame.begin();
       std::next(frame_i) != all_image_frame.end(); ++frame_i) {
    auto frame_j = std::next(frame_i);
    frame_j->second.pre_integration_->Repropagate(
        Eigen::Vector3d::Zero(), Bgs[0]);
  }
}

// ---------------------------------------------------------------------------
// Refine gravity direction using 2-DOF tangent-space parameterisation.
//
// Reference: "Formula Derivation and Analysis of the VINS-Mono", eq (33).
// ---------------------------------------------------------------------------

void RefineGravity(
    std::map<double, common::ImageFrame>& all_image_frame,
    Eigen::Vector3d& g,
    Eigen::VectorXd& x,
    const config::SlamConfig& config) {
  Eigen::Vector3d g0 =
      g.normalized() * config.estimator.gravity.norm();
  int all_frame_count = static_cast<int>(all_image_frame.size());
  int n_state = all_frame_count * 3 + 2 + 1;

  Eigen::MatrixXd A(n_state, n_state);
  Eigen::VectorXd b_vec(n_state);

  for (int k = 0; k < 4; k++) {
    A.setZero();
    b_vec.setZero();

    Eigen::MatrixXd lxly = TangentBasis(g0);
    int i = 0;

    auto frame_i = all_image_frame.begin();
    for (; std::next(frame_i) != all_image_frame.end(); ++frame_i, ++i) {
      auto frame_j = std::next(frame_i);

      Eigen::MatrixXd tmp_A(6, 9);
      tmp_A.setZero();
      Eigen::VectorXd tmp_b(6);
      tmp_b.setZero();

      double dt = frame_j->second.pre_integration_->sum_dt_;

      tmp_A.block<3, 3>(0, 0) = -dt * Eigen::Matrix3d::Identity();
      tmp_A.block<3, 2>(0, 6) =
          frame_i->second.R_.transpose() * dt * dt / 2.0 *
          Eigen::Matrix3d::Identity() * lxly;
      tmp_A.block<3, 1>(0, 8) =
          frame_i->second.R_.transpose() *
          (frame_j->second.T_ - frame_i->second.T_) / 100.0;
      tmp_b.block<3, 1>(0, 0) =
          frame_j->second.pre_integration_->delta_p_ +
          frame_i->second.R_.transpose() * frame_j->second.R_ *
              config.camera.t_ic -
          config.camera.t_ic -
          frame_i->second.R_.transpose() * dt * dt / 2.0 * g0;

      tmp_A.block<3, 3>(3, 0) = -Eigen::Matrix3d::Identity();
      tmp_A.block<3, 3>(3, 3) =
          frame_i->second.R_.transpose() * frame_j->second.R_;
      tmp_A.block<3, 2>(3, 6) =
          frame_i->second.R_.transpose() * dt *
          Eigen::Matrix3d::Identity() * lxly;
      tmp_b.block<3, 1>(3, 0) =
          frame_j->second.pre_integration_->delta_v_ -
          frame_i->second.R_.transpose() * dt *
              Eigen::Matrix3d::Identity() * g0;

      Eigen::Matrix<double, 6, 6> cov_inv =
          Eigen::Matrix<double, 6, 6>::Identity();

      Eigen::MatrixXd r_A = tmp_A.transpose() * cov_inv * tmp_A;
      Eigen::VectorXd r_b = tmp_A.transpose() * cov_inv * tmp_b;

      A.block<6, 6>(i * 3, i * 3) += r_A.topLeftCorner<6, 6>();
      b_vec.segment<6>(i * 3) += r_b.head<6>();

      A.bottomRightCorner<3, 3>() += r_A.bottomRightCorner<3, 3>();
      b_vec.tail<3>() += r_b.tail<3>();

      A.block<6, 3>(i * 3, n_state - 3) += r_A.topRightCorner<6, 3>();
      A.block<3, 6>(n_state - 3, i * 3) += r_A.bottomLeftCorner<3, 6>();
    }

    A = A * 1000.0;
    b_vec = b_vec * 1000.0;
    x = A.ldlt().solve(b_vec);
    Eigen::VectorXd dg = x.segment<2>(n_state - 3);
    g0 = (g0 + lxly * dg).normalized() * config.estimator.gravity.norm();
  }
  g = g0;
}

// ---------------------------------------------------------------------------
// Linear alignment: solve for per-frame velocities, gravity, and scale.
//
// Reference: "Formula Derivation and Analysis of the VINS-Mono", eq (31).
// ---------------------------------------------------------------------------

bool LinearAlignment(
    std::map<double, common::ImageFrame>& all_image_frame,
    Eigen::Vector3d& g,
    Eigen::VectorXd& x,
    const config::SlamConfig& config) {
  int all_frame_count = static_cast<int>(all_image_frame.size());
  int n_state = all_frame_count * 3 + 3 + 1;

  Eigen::MatrixXd A(n_state, n_state);
  A.setZero();
  Eigen::VectorXd b_vec(n_state);
  b_vec.setZero();

  int i = 0;
  auto frame_i = all_image_frame.begin();
  for (; std::next(frame_i) != all_image_frame.end(); ++frame_i, ++i) {
    auto frame_j = std::next(frame_i);

    Eigen::MatrixXd tmp_A(6, 10);
    tmp_A.setZero();
    Eigen::VectorXd tmp_b(6);
    tmp_b.setZero();

    double dt = frame_j->second.pre_integration_->sum_dt_;

    tmp_A.block<3, 3>(0, 0) = -dt * Eigen::Matrix3d::Identity();
    tmp_A.block<3, 3>(0, 6) =
        frame_i->second.R_.transpose() * dt * dt / 2.0 *
        Eigen::Matrix3d::Identity();
    tmp_A.block<3, 1>(0, 9) =
        frame_i->second.R_.transpose() *
        (frame_j->second.T_ - frame_i->second.T_) / 100.0;
    tmp_b.block<3, 1>(0, 0) =
        frame_j->second.pre_integration_->delta_p_ +
        frame_i->second.R_.transpose() * frame_j->second.R_ *
            config.camera.t_ic -
        config.camera.t_ic;

    tmp_A.block<3, 3>(3, 0) = -Eigen::Matrix3d::Identity();
    tmp_A.block<3, 3>(3, 3) =
        frame_i->second.R_.transpose() * frame_j->second.R_;
    tmp_A.block<3, 3>(3, 6) =
        frame_i->second.R_.transpose() * dt * Eigen::Matrix3d::Identity();
    tmp_b.block<3, 1>(3, 0) =
        frame_j->second.pre_integration_->delta_v_;

    Eigen::Matrix<double, 6, 6> cov_inv =
        Eigen::Matrix<double, 6, 6>::Identity();

    Eigen::MatrixXd r_A = tmp_A.transpose() * cov_inv * tmp_A;
    Eigen::VectorXd r_b = tmp_A.transpose() * cov_inv * tmp_b;

    A.block<6, 6>(i * 3, i * 3) += r_A.topLeftCorner<6, 6>();
    b_vec.segment<6>(i * 3) += r_b.head<6>();

    A.bottomRightCorner<4, 4>() += r_A.bottomRightCorner<4, 4>();
    b_vec.tail<4>() += r_b.tail<4>();

    A.block<6, 4>(i * 3, n_state - 4) += r_A.topRightCorner<6, 4>();
    A.block<4, 6>(n_state - 4, i * 3) += r_A.bottomLeftCorner<4, 6>();
  }

  A = A * 1000.0;
  b_vec = b_vec * 1000.0;
  x = A.ldlt().solve(b_vec);

  double s = x(n_state - 1) / 100.0;
  std::cout << "estimated scale: " << s << std::endl;

  g = x.segment<3>(n_state - 4);
  std::cout << " result g     " << g.norm() << " " << g.transpose()
            << std::endl;

  if (std::fabs(g.norm() - config.estimator.gravity.norm()) > 1.0 || s < 0) {
    return false;
  }

  RefineGravity(all_image_frame, g, x, config);
  s = (x.tail<1>())(0) / 100.0;
  (x.tail<1>())(0) = s;
  std::cout << " refine     " << g.norm() << " " << g.transpose()
            << std::endl;

  return s >= 0.0;
}

// ---------------------------------------------------------------------------
// Top-level visual-inertial alignment.
// ---------------------------------------------------------------------------

bool VisualIMUAlignment(
    std::map<double, common::ImageFrame>& all_image_frame,
    Eigen::Vector3d* Bgs,
    Eigen::Vector3d& g,
    Eigen::VectorXd& x,
    const config::SlamConfig& config) {
  solveGyroscopeBias(all_image_frame, Bgs);
  return LinearAlignment(all_image_frame, g, x, config);
}

}  // namespace frontend
}  // namespace slam

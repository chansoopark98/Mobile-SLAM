// Copyright 2025 Mobile-SLAM Authors. All rights reserved.
// Licensed under the MIT License.

#ifndef SLAM_UTILITY_MATH_UTILS_H_
#define SLAM_UTILITY_MATH_UTILS_H_

#include <Eigen/Dense>
#include <chrono>
#include <cmath>

namespace slam {
namespace utility {

// ===========================================================================
// Angle helpers
// ===========================================================================

/// @brief Normalize an angle in degrees to the range (-180, 180].
/// @tparam T  Arithmetic type (double, float, ceres::Jet, ...).
template <typename T>
inline T NormalizeAngle(const T& angle_degrees) {
  T two_pi(360.0);
  if (angle_degrees > T(0.0)) {
    return angle_degrees -
           two_pi * std::floor((angle_degrees + T(180.0)) / two_pi);
  }
  return angle_degrees +
         two_pi * std::floor((-angle_degrees + T(180.0)) / two_pi);
}

// ===========================================================================
// Skew-symmetric (hat) operator
// ===========================================================================

/// @brief Build the 3x3 skew-symmetric matrix [v]_x from a 3-vector.
///        SkewSymmetric(v) * u  ==  v.cross(u)
template <typename Derived>
inline Eigen::Matrix<typename Derived::Scalar, 3, 3> SkewSymmetric(
    const Eigen::MatrixBase<Derived>& v) {
  Eigen::Matrix<typename Derived::Scalar, 3, 3> m;
  using S = typename Derived::Scalar;
  // clang-format off
  m << S(0),  -v(2),  v(1),
       v(2),  S(0),  -v(0),
      -v(1),  v(0),   S(0);
  // clang-format on
  return m;
}

// ===========================================================================
// Quaternion <-> Rotation conversions
// ===========================================================================

/// @brief Convert a unit quaternion to a 3x3 rotation matrix.
inline Eigen::Matrix3d Quaternion2Rotation(const Eigen::Quaterniond& q) {
  return q.normalized().toRotationMatrix();
}

/// @brief Convert a 3x3 rotation matrix to a unit quaternion.
inline Eigen::Quaterniond Rotation2Quaternion(const Eigen::Matrix3d& R) {
  return Eigen::Quaterniond(R).normalized();
}

// ===========================================================================
// Small-angle quaternion (first-order)
// ===========================================================================

/// @brief Compute a small-angle quaternion from a rotation vector.
///        q ~ [1, theta/2] for small ||theta||.
template <typename Derived>
inline Eigen::Quaternion<typename Derived::Scalar> DeltaQ(
    const Eigen::MatrixBase<Derived>& theta) {
  using S = typename Derived::Scalar;
  Eigen::Quaternion<S> dq;
  Eigen::Matrix<S, 3, 1> half_theta = theta;
  half_theta /= S(2.0);
  dq.w() = S(1.0);
  dq.x() = half_theta.x();
  dq.y() = half_theta.y();
  dq.z() = half_theta.z();
  return dq;
}

// ===========================================================================
// Euler-angle conversions  (ZYX convention, degrees)
// ===========================================================================

/// @brief Extract yaw-pitch-roll (degrees) from a rotation matrix.
///        Convention: R = Rz(yaw) * Ry(pitch) * Rx(roll).
inline Eigen::Vector3d YawPitchRoll(const Eigen::Matrix3d& R) {
  Eigen::Vector3d n = R.col(0);
  Eigen::Vector3d o = R.col(1);
  Eigen::Vector3d a = R.col(2);

  double y = std::atan2(n(1), n(0));
  double p = std::atan2(-n(2), n(0) * std::cos(y) + n(1) * std::sin(y));
  double r = std::atan2(a(0) * std::sin(y) - a(1) * std::cos(y),
                        -o(0) * std::sin(y) + o(1) * std::cos(y));

  constexpr double kRadToDeg = 180.0 / M_PI;
  return Eigen::Vector3d(y, p, r) * kRadToDeg;
}

/// @brief Build a rotation matrix from yaw-pitch-roll (degrees).
///        Convention: R = Rz(yaw) * Ry(pitch) * Rx(roll).
template <typename Derived>
inline Eigen::Matrix<typename Derived::Scalar, 3, 3> RpyToRotation(
    const Eigen::MatrixBase<Derived>& ypr) {
  using S = typename Derived::Scalar;
  constexpr double kDegToRad = M_PI / 180.0;

  S y = ypr(0) * S(kDegToRad);
  S p = ypr(1) * S(kDegToRad);
  S r = ypr(2) * S(kDegToRad);

  Eigen::Matrix<S, 3, 3> Rz;
  // clang-format off
  Rz << std::cos(y), -std::sin(y), S(0),
        std::sin(y),  std::cos(y), S(0),
        S(0),         S(0),        S(1);
  // clang-format on

  Eigen::Matrix<S, 3, 3> Ry;
  // clang-format off
  Ry << std::cos(p), S(0), std::sin(p),
        S(0),        S(1), S(0),
       -std::sin(p), S(0), std::cos(p);
  // clang-format on

  Eigen::Matrix<S, 3, 3> Rx;
  // clang-format off
  Rx << S(1), S(0),         S(0),
        S(0), std::cos(r), -std::sin(r),
        S(0), std::sin(r),  std::cos(r);
  // clang-format on

  return Rz * Ry * Rx;
}

// ===========================================================================
// Gravity alignment
// ===========================================================================

/// @brief Compute a rotation that aligns the gravity vector g with the
///        world-frame z-axis [0,0,1], with zero yaw.
///
///   1. Find the quaternion that rotates g_normalized onto [0,0,1].
///   2. Remove the residual yaw component so that the result is pure
///      roll+pitch alignment.
inline Eigen::Matrix3d G2R(const Eigen::Vector3d& g) {
  Eigen::Vector3d ng1 = g.normalized();
  Eigen::Vector3d ng2{0.0, 0.0, 1.0};
  Eigen::Matrix3d R0 =
      Eigen::Quaterniond::FromTwoVectors(ng1, ng2).toRotationMatrix();
  double yaw = YawPitchRoll(R0).x();
  R0 = RpyToRotation(Eigen::Vector3d{-yaw, 0.0, 0.0}) * R0;
  return R0;
}

// ===========================================================================
// Quaternion left/right multiplication matrices
// ===========================================================================

/// @brief Ensure quaternion has non-negative scalar part (w >= 0).
template <typename Derived>
inline Eigen::Quaternion<typename Derived::Scalar> Positify(
    const Eigen::QuaternionBase<Derived>& q) {
  // Identity pass-through -- kept for API parity with reference code.
  return q;
}

/// @brief 4x4 left-multiplication matrix: Qleft(q) * p_vec == (q * p)_vec.
///
///   Qleft(q) = [ w,  -v^T ]
///              [ v,  wI + [v]_x ]
template <typename Derived>
inline Eigen::Matrix<typename Derived::Scalar, 4, 4> Qleft(
    const Eigen::QuaternionBase<Derived>& q) {
  using S = typename Derived::Scalar;
  Eigen::Quaternion<S> qq = Positify(q);
  Eigen::Matrix<S, 4, 4> ans;
  ans(0, 0) = qq.w();
  ans.template block<1, 3>(0, 1) = -qq.vec().transpose();
  ans.template block<3, 1>(1, 0) = qq.vec();
  ans.template block<3, 3>(1, 1) =
      qq.w() * Eigen::Matrix<S, 3, 3>::Identity() + SkewSymmetric(qq.vec());
  return ans;
}

/// @brief 4x4 right-multiplication matrix: Qright(p) * q_vec == (q * p)_vec.
///
///   Qright(p) = [ w,  -v^T ]
///               [ v,  wI - [v]_x ]
template <typename Derived>
inline Eigen::Matrix<typename Derived::Scalar, 4, 4> Qright(
    const Eigen::QuaternionBase<Derived>& p) {
  using S = typename Derived::Scalar;
  Eigen::Quaternion<S> pp = Positify(p);
  Eigen::Matrix<S, 4, 4> ans;
  ans(0, 0) = pp.w();
  ans.template block<1, 3>(0, 1) = -pp.vec().transpose();
  ans.template block<3, 1>(1, 0) = pp.vec();
  ans.template block<3, 3>(1, 1) =
      pp.w() * Eigen::Matrix<S, 3, 3>::Identity() - SkewSymmetric(pp.vec());
  return ans;
}

// ===========================================================================
// Simple wall-clock timer
// ===========================================================================

/// @brief RAII-style timer using std::chrono::steady_clock.
///
/// Usage:
///   TicToc timer;
///   ... work ...
///   double ms = timer.Toc();  // elapsed since construction or last Tic()
class TicToc {
 public:
  TicToc() { Tic(); }

  /// @brief Reset the start time.
  void Tic() { start_ = std::chrono::steady_clock::now(); }

  /// @brief Return elapsed time in milliseconds since the last Tic().
  double Toc() const {
    auto end = std::chrono::steady_clock::now();
    return std::chrono::duration<double, std::milli>(end - start_).count();
  }

 private:
  std::chrono::steady_clock::time_point start_;
};

}  // namespace utility
}  // namespace slam

#endif  // SLAM_UTILITY_MATH_UTILS_H_

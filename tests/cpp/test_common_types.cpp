// Copyright 2025 Mobile-SLAM Authors. All rights reserved.
// Licensed under the MIT License.
//
// Unit tests for slam/common/types.h, frame.h, image_frame.h, and
// slam/utility/math_utils.h

#include <gtest/gtest.h>

#include <cmath>
#include <memory>

#include "slam/common/frame.h"
#include "slam/common/image_frame.h"
#include "slam/common/types.h"
#include "slam/utility/math_utils.h"

// ===========================================================================
// types.h
// ===========================================================================

TEST(TypesTest, Constants) {
  EXPECT_EQ(slam::common::kWindowSize, 10);
  EXPECT_EQ(slam::common::kNumOfFeatures, 1000);
  EXPECT_DOUBLE_EQ(slam::common::kMinParallax, 10.0);
  EXPECT_DOUBLE_EQ(slam::common::kGravityNorm, 9.81);
}

TEST(TypesTest, SolverFlagValues) {
  EXPECT_EQ(static_cast<int>(slam::common::SolverFlag::kInitial), 0);
  EXPECT_EQ(static_cast<int>(slam::common::SolverFlag::kNonLinear), 1);
}

TEST(TypesTest, MarginalizationFlagValues) {
  EXPECT_EQ(
      static_cast<int>(slam::common::MarginalizationFlag::kMarginOldKeyframe),
      0);
  EXPECT_EQ(static_cast<int>(
                slam::common::MarginalizationFlag::kMarginNewGeneralFrame),
            1);
}

TEST(TypesTest, ImageDataInsertAndRead) {
  slam::common::ImageData data;
  slam::common::FeatureObservation obs;
  obs << 0.1, 0.2, 1.0, 320.0, 240.0, 0.5, -0.3;
  data[42] = obs;

  ASSERT_EQ(data.size(), 1u);
  EXPECT_DOUBLE_EQ(data[42](0), 0.1);
  EXPECT_DOUBLE_EQ(data[42](3), 320.0);
}

// ===========================================================================
// frame.h / frame.cpp
// ===========================================================================

TEST(FrameTest, DefaultConstruction) {
  slam::common::Frame f;
  EXPECT_DOUBLE_EQ(f.timestamp_, 0.0);
  EXPECT_TRUE(f.R_.isIdentity());
  EXPECT_TRUE(f.P_.isZero());
  EXPECT_TRUE(f.V_.isZero());
  EXPECT_TRUE(f.Ba_.isZero());
  EXPECT_TRUE(f.Bg_.isZero());
  EXPECT_EQ(f.pre_integration_, nullptr);
  EXPECT_TRUE(f.dt_buf_.empty());
}

TEST(FrameTest, Reset) {
  slam::common::Frame f;
  f.timestamp_ = 1.5;
  f.P_ = Eigen::Vector3d(1, 2, 3);
  f.dt_buf_.push_back(0.01);
  f.linear_acceleration_buf_.push_back(Eigen::Vector3d::Ones());

  f.Reset();

  EXPECT_DOUBLE_EQ(f.timestamp_, 0.0);
  EXPECT_TRUE(f.P_.isZero());
  EXPECT_TRUE(f.dt_buf_.empty());
  EXPECT_TRUE(f.linear_acceleration_buf_.empty());
}

TEST(FrameTest, MoveConstruction) {
  slam::common::Frame a;
  a.timestamp_ = 3.14;
  a.P_ = Eigen::Vector3d(10, 20, 30);
  a.dt_buf_.push_back(0.005);

  slam::common::Frame b(std::move(a));
  EXPECT_DOUBLE_EQ(b.timestamp_, 3.14);
  EXPECT_EQ(b.P_, Eigen::Vector3d(10, 20, 30));
  EXPECT_EQ(b.dt_buf_.size(), 1u);
}

TEST(FrameTest, MoveAssignment) {
  slam::common::Frame a;
  a.timestamp_ = 2.71;
  a.V_ = Eigen::Vector3d(0.1, 0.2, 0.3);

  slam::common::Frame b;
  b = std::move(a);
  EXPECT_DOUBLE_EQ(b.timestamp_, 2.71);
  EXPECT_EQ(b.V_, Eigen::Vector3d(0.1, 0.2, 0.3));
}

// ===========================================================================
// image_frame.h
// ===========================================================================

TEST(ImageFrameTest, DefaultConstruction) {
  slam::common::ImageFrame f;
  EXPECT_DOUBLE_EQ(f.timestamp_, 0.0);
  EXPECT_TRUE(f.R_.isIdentity());
  EXPECT_TRUE(f.T_.isZero());
  EXPECT_FALSE(f.is_key_frame_);
  EXPECT_TRUE(f.points_.empty());
}

TEST(ImageFrameTest, ConstructWithPoints) {
  slam::common::ImageData pts;
  slam::common::FeatureObservation obs;
  obs << 0.0, 0.0, 1.0, 100.0, 200.0, 0.0, 0.0;
  pts[1] = obs;
  pts[2] = obs;

  slam::common::ImageFrame f(pts, 1.5);
  EXPECT_EQ(f.GetFeatureCount(), 2u);
  EXPECT_DOUBLE_EQ(f.timestamp_, 1.5);
  EXPECT_TRUE(f.HasEnoughFeatures(2));
  EXPECT_FALSE(f.HasEnoughFeatures(5));
}

TEST(ImageFrameTest, PoseMatrix) {
  slam::common::ImageFrame f;
  Eigen::Matrix3d R = Eigen::AngleAxisd(0.1, Eigen::Vector3d::UnitZ())
                          .toRotationMatrix();
  Eigen::Vector3d T(1.0, 2.0, 3.0);
  f.R_ = R;
  f.T_ = T;

  Eigen::Matrix4d pose = f.GetPoseMatrix();
  EXPECT_TRUE((pose.block<3, 3>(0, 0).isApprox(R)));
  EXPECT_TRUE((pose.block<3, 1>(0, 3).isApprox(T)));

  // Round-trip through SetPoseMatrix
  slam::common::ImageFrame g;
  g.SetPoseMatrix(pose);
  EXPECT_TRUE(g.R_.isApprox(R));
  EXPECT_TRUE(g.T_.isApprox(T));
}

TEST(ImageFrameTest, MoveSemantics) {
  slam::common::ImageData pts;
  slam::common::FeatureObservation obs;
  obs << 0.0, 0.0, 1.0, 100.0, 200.0, 0.0, 0.0;
  pts[1] = obs;

  slam::common::ImageFrame a(pts, 2.0);
  a.is_key_frame_ = true;

  slam::common::ImageFrame b(std::move(a));
  EXPECT_EQ(b.GetFeatureCount(), 1u);
  EXPECT_DOUBLE_EQ(b.timestamp_, 2.0);
  EXPECT_TRUE(b.is_key_frame_);
}

// ===========================================================================
// math_utils.h
// ===========================================================================

TEST(MathUtilsTest, NormalizeAngle) {
  using slam::utility::NormalizeAngle;
  EXPECT_NEAR(NormalizeAngle(0.0), 0.0, 1e-12);
  EXPECT_NEAR(std::abs(NormalizeAngle(180.0)), 180.0, 1e-12);
  EXPECT_NEAR(NormalizeAngle(360.0), 0.0, 1e-12);
  EXPECT_NEAR(std::abs(NormalizeAngle(-180.0)), 180.0, 1e-12);
  EXPECT_NEAR(NormalizeAngle(270.0), -90.0, 1e-12);
  EXPECT_NEAR(NormalizeAngle(-270.0), 90.0, 1e-12);
}

TEST(MathUtilsTest, SkewSymmetric) {
  Eigen::Vector3d v(1.0, 2.0, 3.0);
  Eigen::Matrix3d S = slam::utility::SkewSymmetric(v);

  // S should be anti-symmetric
  EXPECT_TRUE(S.isApprox(-S.transpose(), 1e-12));

  // S * u == v x u
  Eigen::Vector3d u(4.0, 5.0, 6.0);
  EXPECT_TRUE((S * u).isApprox(v.cross(u), 1e-12));
}

TEST(MathUtilsTest, Quaternion2RotationRoundTrip) {
  Eigen::Quaterniond q =
      Eigen::AngleAxisd(0.3, Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(0.1, Eigen::Vector3d::UnitX());
  q.normalize();

  Eigen::Matrix3d R = slam::utility::Quaternion2Rotation(q);
  Eigen::Quaterniond q2 = slam::utility::Rotation2Quaternion(R);

  // Quaternions may differ by sign
  if (q.w() * q2.w() < 0) {
    q2.coeffs() *= -1;
  }
  EXPECT_TRUE(q.isApprox(q2, 1e-12));
}

TEST(MathUtilsTest, YawPitchRollRoundTrip) {
  Eigen::Vector3d ypr_in(30.0, -15.0, 5.0);  // degrees
  Eigen::Matrix3d R = slam::utility::RpyToRotation(ypr_in);
  Eigen::Vector3d ypr_out = slam::utility::YawPitchRoll(R);

  EXPECT_NEAR(ypr_out(0), ypr_in(0), 1e-10);
  EXPECT_NEAR(ypr_out(1), ypr_in(1), 1e-10);
  EXPECT_NEAR(ypr_out(2), ypr_in(2), 1e-10);
}

TEST(MathUtilsTest, DeltaQ) {
  Eigen::Vector3d theta(0.01, 0.02, -0.005);
  auto dq = slam::utility::DeltaQ(theta);
  EXPECT_NEAR(dq.w(), 1.0, 1e-6);
  EXPECT_NEAR(dq.x(), 0.005, 1e-10);
  EXPECT_NEAR(dq.y(), 0.01, 1e-10);
  EXPECT_NEAR(dq.z(), -0.0025, 1e-10);
}

TEST(MathUtilsTest, G2R_AlignGravity) {
  // Standard gravity pointing down
  Eigen::Vector3d g(0.0, 0.0, -9.81);
  Eigen::Matrix3d R = slam::utility::G2R(g);

  // R * g_normalized should point along z_world direction (up or down)
  Eigen::Vector3d aligned = R * g.normalized();
  EXPECT_NEAR(std::abs(aligned(2)), 1.0, 1e-10);

  // Check that yaw is effectively zero
  Eigen::Vector3d ypr = slam::utility::YawPitchRoll(R);
  EXPECT_NEAR(ypr(0), 0.0, 1e-10);
}

TEST(MathUtilsTest, TicToc) {
  slam::utility::TicToc timer;
  // Just check that Toc returns a non-negative number
  double elapsed = timer.Toc();
  EXPECT_GE(elapsed, 0.0);
}

TEST(MathUtilsTest, QleftQright) {
  Eigen::Quaterniond q(0.5, 0.5, 0.5, 0.5);
  q.normalize();
  Eigen::Quaterniond p(0.1, 0.2, 0.3, 0.4);
  p.normalize();

  // q * p via matrices
  Eigen::Vector4d p_vec;
  p_vec << p.w(), p.x(), p.y(), p.z();
  Eigen::Vector4d q_vec;
  q_vec << q.w(), q.x(), q.y(), q.z();

  auto ql = slam::utility::Qleft(q);
  Eigen::Vector4d result_left = ql * p_vec;

  auto qr = slam::utility::Qright(p);
  Eigen::Vector4d result_right = qr * q_vec;

  // Both should give the same quaternion product
  EXPECT_TRUE(result_left.isApprox(result_right, 1e-10));

  // And match the direct Eigen product
  Eigen::Quaterniond qp = q * p;
  Eigen::Vector4d expected;
  expected << qp.w(), qp.x(), qp.y(), qp.z();
  EXPECT_TRUE(result_left.isApprox(expected, 1e-10));
}

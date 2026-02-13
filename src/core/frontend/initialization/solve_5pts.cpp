// Copyright 2025 Mobile-SLAM Authors. All rights reserved.
// Licensed under the MIT License.

#include "slam/frontend/initialization/solve_5pts.h"

#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>

namespace {

// ---------------------------------------------------------------------------
// Custom essential-matrix decomposition and pose recovery.
//
// We provide our own implementation rather than relying on cv::recoverPose
// because the OpenCV version may not handle all degenerate cases correctly
// for SLAM initialisation.
// ---------------------------------------------------------------------------

void decomposeE(cv::InputArray _E,
                cv::OutputArray _R1,
                cv::OutputArray _R2,
                cv::OutputArray _t) {
  cv::Mat E = _E.getMat().reshape(1, 3);
  CV_Assert(E.cols == 3 && E.rows == 3);

  cv::Mat D, U, Vt;
  cv::SVD::compute(E, D, U, Vt);

  if (cv::determinant(U) < 0) U *= -1.0;
  if (cv::determinant(Vt) < 0) Vt *= -1.0;

  cv::Mat W = (cv::Mat_<double>(3, 3) << 0, 1, 0, -1, 0, 0, 0, 0, 1);
  W.convertTo(W, E.type());

  cv::Mat R1 = U * W * Vt;
  cv::Mat R2 = U * W.t() * Vt;
  cv::Mat t = U.col(2) * 1.0;

  R1.copyTo(_R1);
  R2.copyTo(_R2);
  t.copyTo(_t);
}

int recoverPose(cv::InputArray E,
                cv::InputArray _points1,
                cv::InputArray _points2,
                cv::InputArray _cameraMatrix,
                cv::OutputArray _R,
                cv::OutputArray _t,
                cv::InputOutputArray _mask) {
  cv::Mat points1, points2, cameraMatrix;
  _points1.getMat().convertTo(points1, CV_64F);
  _points2.getMat().convertTo(points2, CV_64F);
  _cameraMatrix.getMat().convertTo(cameraMatrix, CV_64F);

  int npoints = points1.checkVector(2);
  CV_Assert(npoints >= 0 && points2.checkVector(2) == npoints &&
            points1.type() == points2.type());
  CV_Assert(cameraMatrix.rows == 3 && cameraMatrix.cols == 3 &&
            cameraMatrix.channels() == 1);

  if (points1.channels() > 1) {
    points1 = points1.reshape(1, npoints);
    points2 = points2.reshape(1, npoints);
  }

  double fx = cameraMatrix.at<double>(0, 0);
  double fy = cameraMatrix.at<double>(1, 1);
  double cx = cameraMatrix.at<double>(0, 2);
  double cy = cameraMatrix.at<double>(1, 2);

  points1.col(0) = (points1.col(0) - cx) / fx;
  points2.col(0) = (points2.col(0) - cx) / fx;
  points1.col(1) = (points1.col(1) - cy) / fy;
  points2.col(1) = (points2.col(1) - cy) / fy;

  points1 = points1.t();
  points2 = points2.t();

  cv::Mat R1, R2, t;
  decomposeE(E, R1, R2, t);

  cv::Mat P0 = cv::Mat::eye(3, 4, R1.type());
  cv::Mat P1(3, 4, R1.type()), P2(3, 4, R1.type());
  cv::Mat P3(3, 4, R1.type()), P4(3, 4, R1.type());

  P1(cv::Range::all(), cv::Range(0, 3)) = R1 * 1.0;
  P1.col(3) = t * 1.0;
  P2(cv::Range::all(), cv::Range(0, 3)) = R2 * 1.0;
  P2.col(3) = t * 1.0;
  P3(cv::Range::all(), cv::Range(0, 3)) = R1 * 1.0;
  P3.col(3) = -t * 1.0;
  P4(cv::Range::all(), cv::Range(0, 3)) = R2 * 1.0;
  P4.col(3) = -t * 1.0;

  // Cheirality check: filter out far-away (infinite) points.
  double dist = 50.0;
  cv::Mat Q;

  // --- Combo 1: (R1, +t) ---
  cv::triangulatePoints(P0, P1, points1, points2, Q);
  cv::Mat mask1 = Q.row(2).mul(Q.row(3)) > 0;
  Q.row(0) /= Q.row(3);
  Q.row(1) /= Q.row(3);
  Q.row(2) /= Q.row(3);
  Q.row(3) /= Q.row(3);
  mask1 = (Q.row(2) < dist) & mask1;
  Q = P1 * Q;
  mask1 = (Q.row(2) > 0) & mask1;
  mask1 = (Q.row(2) < dist) & mask1;

  // --- Combo 2: (R2, +t) ---
  cv::triangulatePoints(P0, P2, points1, points2, Q);
  cv::Mat mask2 = Q.row(2).mul(Q.row(3)) > 0;
  Q.row(0) /= Q.row(3);
  Q.row(1) /= Q.row(3);
  Q.row(2) /= Q.row(3);
  Q.row(3) /= Q.row(3);
  mask2 = (Q.row(2) < dist) & mask2;
  Q = P2 * Q;
  mask2 = (Q.row(2) > 0) & mask2;
  mask2 = (Q.row(2) < dist) & mask2;

  // --- Combo 3: (R1, -t) ---
  cv::triangulatePoints(P0, P3, points1, points2, Q);
  cv::Mat mask3 = Q.row(2).mul(Q.row(3)) > 0;
  Q.row(0) /= Q.row(3);
  Q.row(1) /= Q.row(3);
  Q.row(2) /= Q.row(3);
  Q.row(3) /= Q.row(3);
  mask3 = (Q.row(2) < dist) & mask3;
  Q = P3 * Q;
  mask3 = (Q.row(2) > 0) & mask3;
  mask3 = (Q.row(2) < dist) & mask3;

  // --- Combo 4: (R2, -t) ---
  cv::triangulatePoints(P0, P4, points1, points2, Q);
  cv::Mat mask4 = Q.row(2).mul(Q.row(3)) > 0;
  Q.row(0) /= Q.row(3);
  Q.row(1) /= Q.row(3);
  Q.row(2) /= Q.row(3);
  Q.row(3) /= Q.row(3);
  mask4 = (Q.row(2) < dist) & mask4;
  Q = P4 * Q;
  mask4 = (Q.row(2) > 0) & mask4;
  mask4 = (Q.row(2) < dist) & mask4;

  mask1 = mask1.t();
  mask2 = mask2.t();
  mask3 = mask3.t();
  mask4 = mask4.t();

  if (!_mask.empty()) {
    cv::Mat mask = _mask.getMat();
    CV_Assert(mask.size() == mask1.size());
    cv::bitwise_and(mask, mask1, mask1);
    cv::bitwise_and(mask, mask2, mask2);
    cv::bitwise_and(mask, mask3, mask3);
    cv::bitwise_and(mask, mask4, mask4);
  }
  if (_mask.empty() && _mask.needed()) {
    _mask.create(mask1.size(), CV_8U);
  }

  CV_Assert(_R.needed() && _t.needed());
  _R.create(3, 3, R1.type());
  _t.create(3, 1, t.type());

  int good1 = cv::countNonZero(mask1);
  int good2 = cv::countNonZero(mask2);
  int good3 = cv::countNonZero(mask3);
  int good4 = cv::countNonZero(mask4);

  if (good1 >= good2 && good1 >= good3 && good1 >= good4) {
    R1.copyTo(_R);
    t.copyTo(_t);
    if (_mask.needed()) mask1.copyTo(_mask);
    return good1;
  } else if (good2 >= good1 && good2 >= good3 && good2 >= good4) {
    R2.copyTo(_R);
    t.copyTo(_t);
    if (_mask.needed()) mask2.copyTo(_mask);
    return good2;
  } else if (good3 >= good1 && good3 >= good2 && good3 >= good4) {
    t = -t;
    R1.copyTo(_R);
    t.copyTo(_t);
    if (_mask.needed()) mask3.copyTo(_mask);
    return good3;
  } else {
    t = -t;
    R2.copyTo(_R);
    t.copyTo(_t);
    if (_mask.needed()) mask4.copyTo(_mask);
    return good4;
  }
}

}  // anonymous namespace

namespace slam {
namespace frontend {

bool MotionEstimator::solveRelativeRT(const Correspondences& corres,
                                      Eigen::Matrix3d& Rotation,
                                      Eigen::Vector3d& Translation) {
  if (corres.size() < 15) {
    return false;
  }

  std::vector<cv::Point2f> ll, rr;
  ll.reserve(corres.size());
  rr.reserve(corres.size());
  for (const auto& c : corres) {
    ll.emplace_back(static_cast<float>(c.first(0)),
                    static_cast<float>(c.first(1)));
    rr.emplace_back(static_cast<float>(c.second(0)),
                    static_cast<float>(c.second(1)));
  }

  cv::Mat mask;
  cv::Mat E =
      cv::findFundamentalMat(ll, rr, cv::FM_RANSAC, 0.3 / 460, 0.99, mask);
  cv::Mat camera_matrix =
      (cv::Mat_<double>(3, 3) << 1, 0, 0, 0, 1, 0, 0, 0, 1);
  cv::Mat rot, trans;

  int inlier_cnt =
      ::recoverPose(E, ll, rr, camera_matrix, rot, trans, mask);

  Eigen::Matrix3d R;
  Eigen::Vector3d T;
  for (int i = 0; i < 3; i++) {
    T(i) = trans.at<double>(i, 0);
    for (int j = 0; j < 3; j++) {
      R(i, j) = rot.at<double>(i, j);
    }
  }

  Rotation = R.transpose();
  Translation = -R.transpose() * T;
  return inlier_cnt > 12;
}

}  // namespace frontend
}  // namespace slam

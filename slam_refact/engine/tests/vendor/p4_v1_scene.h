// Scene helpers of the P4 V1 test and of its golden generator (p4_v1_golden_gen.cc): a world point
// whose camera-space position under a pose is chosen, so that the reprojection error of the
// pose-estimation residual can be set to a tiny value exactly.

#ifndef SLAM_REFACT_TESTS_VENDOR_P4_V1_SCENE_H_
#define SLAM_REFACT_TESTS_VENDOR_P4_V1_SCENE_H_

#include <cmath>

namespace p4v1 {

struct Pose {
  double r[3];  // angle-axis
  double t[3];
};

// R(r) * X + t: Rodrigues for theta^2 >= 1e-6, the first order model of the small rotation branch.
inline void Camera(const Pose& p, const double* X, double* pc) {
  const double th2 = p.r[0] * p.r[0] + p.r[1] * p.r[1] + p.r[2] * p.r[2];
  double rx[3];
  if (th2 < 1e-6) {
    rx[0] = X[0] + (p.r[1] * X[2] - p.r[2] * X[1]);
    rx[1] = X[1] + (p.r[2] * X[0] - p.r[0] * X[2]);
    rx[2] = X[2] + (p.r[0] * X[1] - p.r[1] * X[0]);
  } else {
    const double th = std::sqrt(th2);
    const double k[3] = {p.r[0] / th, p.r[1] / th, p.r[2] / th};
    const double c = std::cos(th), s = std::sin(th);
    const double kx[3] = {k[1] * X[2] - k[2] * X[1], k[2] * X[0] - k[0] * X[2], k[0] * X[1] - k[1] * X[0]};
    const double kd = k[0] * X[0] + k[1] * X[1] + k[2] * X[2];
    for (int i = 0; i < 3; ++i) rx[i] = X[i] * c + kx[i] * s + k[i] * kd * (1.0 - c);
  }
  for (int i = 0; i < 3; ++i) pc[i] = rx[i] + p.t[i];
}

// The world point whose camera-space position under `p` is `target`: X = R^-1 (target - t).
inline void WorldPoint(const Pose& p, const double* target, double* X) {
  const double d[3] = {target[0] - p.t[0], target[1] - p.t[1], target[2] - p.t[2]};
  const double th2 = p.r[0] * p.r[0] + p.r[1] * p.r[1] + p.r[2] * p.r[2];
  if (th2 < 1e-6) {  // (I + [r]x)^-1 = (I - [r]x + r r^T) / (1 + |r|^2)
    const double* r = p.r;
    const double rd = r[0] * d[0] + r[1] * d[1] + r[2] * d[2];
    const double rxd[3] = {r[1] * d[2] - r[2] * d[1], r[2] * d[0] - r[0] * d[2], r[0] * d[1] - r[1] * d[0]};
    for (int i = 0; i < 3; ++i) X[i] = (d[i] - rxd[i] + r[i] * rd) / (1.0 + th2);
  } else {
    const Pose inv{{-p.r[0], -p.r[1], -p.r[2]}, {0, 0, 0}};
    Camera(inv, d, X);
  }
}

// World point seen at ray (0.25 + su, 0.125 + sv) at camera depth `depth` (the observation is the
// float ray (0.25f, 0.125f), exact in float: the reprojection error is su resp. sv).
inline void WorldPointAtError(const Pose& p, double depth, double su, double sv, double* X) {
  const double target[3] = {(0.25 + su) * depth, (0.125 + sv) * depth, depth};
  WorldPoint(p, target, X);
}

}  // namespace p4v1

#endif  // SLAM_REFACT_TESTS_VENDOR_P4_V1_SCENE_H_

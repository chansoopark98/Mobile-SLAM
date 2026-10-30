// MT-P1 (MUST-PASS): YawOf (519), Quaternion::fromHMatrix (520) and ReplaceYaw (1271) on the 205
// probe cases of S2 §8.1. Inputs are regenerated exactly as probe-yaw-replace.mjs does (LCG seed
// 20261015, Math.fround of the double values); expected values are the original binary outputs in
// tests/data/map-tracker/probe-yaw-replace-r1/cases.json (yaw, quaternion) and the script's double
// reference (1271). Tolerances are the pre-registered spec §8.1 values.

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "../frame_frame_tracker/json_lite.h"
#include "c8/hmatrix.h"
#include "c8/quaternion.h"
#include "map_tracker/map_tracker.h"

namespace slam_refact::map_tracker {
namespace {

constexpr double kTolYaw = 1e-5;
constexpr double kTolQuat = 1e-6;
constexpr double kTolRot = 1e-5;
constexpr double kTolInvTransRel = 1e-5;

using Quat = std::array<double, 4>;  // x, y, z, w
using Mat3 = std::array<std::array<double, 3>, 3>;

struct Lcg {
  uint32_t seed = 20261015u;
  double Next() {
    seed = seed * 1664525u + 1013904223u;
    return static_cast<double>(seed) / 4294967296.0;
  }
};

Quat RandQuat(Lcg* g) {
  const double u1 = g->Next();
  const double u2 = g->Next() * 2 * M_PI;
  const double u3 = g->Next() * 2 * M_PI;
  const double a = std::sqrt(1 - u1);
  const double b = std::sqrt(u1);
  return {a * std::sin(u2), a * std::cos(u2), b * std::sin(u3), b * std::cos(u3)};
}

Quat AxisAngleX(double ang) { return {std::sin(ang / 2), 0.0, 0.0, std::cos(ang / 2)}; }

Quat QuatMul(const Quat& a, const Quat& b) {
  const double ax = a[0], ay = a[1], az = a[2], aw = a[3];
  const double bx = b[0], by = b[1], bz = b[2], bw = b[3];
  return {aw * bx + ax * bw + ay * bz - az * by, aw * by - ax * bz + ay * bw + az * bx,
          aw * bz + ax * by - ay * bx + az * bw, aw * bw - ax * bx - ay * by - az * bz};
}

Mat3 RotOf(const Quat& q) {
  const double x = q[0], y = q[1], z = q[2], w = q[3];
  return {{{1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)},
           {2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)},
           {2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)}}};
}

double YawRef(Quat q) {
  double x = q[0], y = q[1], z = q[2], w = q[3];
  for (int it = 0; it < 8; ++it) {
    const double m22 = 1 - 2 * (x * x + y * y);
    const double m02 = 2 * (w * y + x * z);
    if (std::sqrt(m22 * m22 + m02 * m02) > 1e-5) {
      return std::atan2(m02, m22);
    }
    const double r = M_SQRT1_2;
    const double nx = r * (x - w), ny = r * (y - z), nz = r * (z + y), nw = r * (w + x);
    double n = std::sqrt(nx * nx + ny * ny + nz * nz + nw * nw);
    if (n == 0) {
      n = 1;
    }
    x = nx / n;
    y = ny / n;
    z = nz / n;
    w = nw / n;
  }
  return NAN;
}

Quat FromHMatrixRef(const Mat3& R) {
  const double tr = R[0][0] + R[1][1] + R[2][2];
  if (tr >= 0) {
    const double t = std::sqrt(tr + 1), s = 0.5 / t;
    return {s * (R[2][1] - R[1][2]), s * (R[0][2] - R[2][0]), s * (R[1][0] - R[0][1]), 0.5 * t};
  }
  int i = 0;
  if (R[1][1] > R[0][0]) i = 1;
  if (R[2][2] > R[i][i]) i = 2;
  const int j = (i + 1) % 3, k = (j + 1) % 3;
  const double t = std::sqrt(R[i][i] - R[j][j] - R[k][k] + 1), s = 0.5 / t;
  double v[3] = {0, 0, 0};
  v[i] = 0.5 * t;
  v[j] = s * (R[j][i] + R[i][j]);
  v[k] = s * (R[k][i] + R[i][k]);
  return {v[0], v[1], v[2], s * (R[k][j] - R[j][k])};
}

double Wrap(double a) { return std::atan2(std::sin(a), std::cos(a)); }

struct Case {
  std::string kind;
  Quat q_pose;
  std::array<double, 3> t;
  Quat q_dev;
};

std::vector<Case> MakeCases() {
  Lcg g;
  std::vector<Case> cases;
  for (int k = 0; k < 200; ++k) {
    Case c;
    c.kind = "random";
    c.q_pose = RandQuat(&g);
    const double t0 = g.Next() * 10 - 5;
    const double t1 = g.Next() * 10 - 5;
    const double t2 = g.Next() * 10 - 5;
    c.t = {t0, t1, t2};
    c.q_dev = RandQuat(&g);
    cases.push_back(c);
  }
  cases.push_back({"identity", {0, 0, 0, 1}, {0, 2, 0}, {0, 0, 0, 1}});
  for (int s : {1, -1}) {
    const std::string sfx = s > 0 ? "+90" : "-90";
    Case dev;
    dev.kind = "dev-pitch" + sfx;
    dev.q_pose = RandQuat(&g);
    dev.t = {1, 2, 3};
    dev.q_dev = AxisAngleX(s * M_PI / 2);
    cases.push_back(dev);
    Case pose;
    pose.kind = "pose-pitch" + sfx;
    pose.q_pose = AxisAngleX(s * M_PI / 2);
    pose.t = {-1, 0.5, 4};
    pose.q_dev = RandQuat(&g);
    cases.push_back(pose);
  }
  return cases;
}

TEST(MapTrackerYaw, MtP1ProbeCases) {
  testing::JsonValue golden;
  ASSERT_TRUE(testing::ReadJson(
      std::string(SR_TEST_DATA_DIR) + "/map-tracker/probe-yaw-replace-r1/cases.json", &golden));
  const std::vector<Case> cases = MakeCases();
  ASSERT_EQ(cases.size(), 205u);
  ASSERT_EQ(golden.size(), 205u);

  double worst_yaw = 0, worst_quat = 0, worst_rot = 0, worst_inv_t = 0;
  int retries_seen = 0;
  for (size_t n = 0; n < cases.size(); ++n) {
    const Case& cs = cases[n];
    const testing::JsonValue& g = golden[n];
    ASSERT_EQ(g["kind"].str, cs.kind) << n;

    // Float inputs exactly as the probe writes them.
    const Mat3 rd = RotOf(cs.q_pose);
    Mat3 rf;
    for (int r = 0; r < 3; ++r) {
      for (int c = 0; c < 3; ++c) {
        rf[r][c] = static_cast<float>(rd[r][c]);
      }
    }
    std::array<double, 3> tf;
    for (int r = 0; r < 3; ++r) {
      tf[r] = static_cast<float>(cs.t[r]);
    }
    Quat qdf;
    for (int i = 0; i < 4; ++i) {
      qdf[i] = static_cast<float>(cs.q_dev[i]);
    }
    float fwd[16];
    float inv[16];
    for (int c = 0; c < 4; ++c) {
      for (int r = 0; r < 4; ++r) {
        fwd[4 * c + r] = static_cast<float>(c < 3 ? (r < 3 ? rf[r][c] : 0) : (r < 3 ? tf[r] : 1));
      }
    }
    double t_inv[3];
    for (int r = 0; r < 3; ++r) {
      t_inv[r] = -(rf[0][r] * tf[0] + rf[1][r] * tf[1] + rf[2][r] * tf[2]);
    }
    for (int c = 0; c < 4; ++c) {
      for (int r = 0; r < 4; ++r) {
        inv[4 * c + r] =
            static_cast<float>(c < 3 ? (r < 3 ? rf[c][r] : 0) : (r < 3 ? t_inv[r] : 1));
      }
    }
    const c8::HMatrix pose(fwd, inv);
    const c8::Quaternion q_dev(static_cast<float>(qdf[3]), static_cast<float>(qdf[0]),
                               static_cast<float>(qdf[1]), static_cast<float>(qdf[2]));

    // Input regeneration check: the probe's double reference yaw of this input.
    const double yaw_ref = YawRef(qdf);
    ASSERT_NEAR(yaw_ref, g["yaw519"]["ref"].AsDouble(), 1e-12) << "input mismatch, case " << n;
    if (g["yaw519"]["retries"].AsInt() > 0) {
      ++retries_seen;
    }

    // 519 vs the original binary output.
    const double yaw = YawOf(q_dev);
    const double e_yaw = std::fabs(Wrap(yaw - g["yaw519"]["bin"].AsDouble()));
    EXPECT_LE(e_yaw, kTolYaw) << "yaw case " << n;
    worst_yaw = std::max(worst_yaw, e_yaw);

    // 520 vs the original binary output (sign-aligned).
    const c8::Quaternion q = c8::Quaternion::fromHMatrix(pose);
    const double qv[4] = {q.x(), q.y(), q.z(), q.w()};
    double qb[4];
    for (int i = 0; i < 4; ++i) {
      qb[i] = g["quat520"]["bin"][i].AsDouble();
    }
    const double dot = qv[0] * qb[0] + qv[1] * qb[1] + qv[2] * qb[2] + qv[3] * qb[3];
    const double sgn = dot < 0 ? -1 : 1;
    double e_q = 0;
    for (int i = 0; i < 4; ++i) {
      e_q = std::max(e_q, std::fabs(qv[i] - sgn * qb[i]));
    }
    EXPECT_LE(e_q, kTolQuat) << "quat case " << n;
    worst_quat = std::max(worst_quat, e_q);

    // 1271 vs the double reference of the probe.
    const c8::HMatrix out = ReplaceYaw(pose, q_dev);
    const double d = (YawRef(FromHMatrixRef(rf)) - YawRef(qdf)) * 0.5;
    Quat qr = QuatMul({0, std::sin(d), 0, std::cos(d)}, qdf);
    const double qn = std::sqrt(qr[0] * qr[0] + qr[1] * qr[1] + qr[2] * qr[2] + qr[3] * qr[3]);
    for (double& v : qr) {
      v /= qn;
    }
    const Mat3 rr = RotOf(qr);
    double e_rot = 0;
    for (int r = 0; r < 3; ++r) {
      for (int c = 0; c < 3; ++c) {
        e_rot = std::max(e_rot, std::fabs(out.data()[4 * c + r] - rr[r][c]));
        e_rot = std::max(e_rot, std::fabs(out.inverseData()[4 * c + r] - rr[c][r]));
      }
    }
    const double t_norm = std::sqrt(tf[0] * tf[0] + tf[1] * tf[1] + tf[2] * tf[2]);
    double e_inv_t = 0;
    for (int r = 0; r < 3; ++r) {
      const double ref = -(rr[0][r] * tf[0] + rr[1][r] * tf[1] + rr[2][r] * tf[2]);
      e_inv_t = std::max(e_inv_t, std::fabs(out.inverseData()[12 + r] - ref) / (1 + t_norm));
    }
    EXPECT_LE(e_rot, kTolRot) << "rot case " << n;
    EXPECT_LE(e_inv_t, kTolInvTransRel) << "invT case " << n;
    worst_rot = std::max(worst_rot, e_rot);
    worst_inv_t = std::max(worst_inv_t, e_inv_t);
    for (int r = 0; r < 3; ++r) {
      const float expect = static_cast<float>(tf[r]) + 0.0f;
      EXPECT_EQ(std::memcmp(&out.data()[12 + r], &expect, sizeof(float)), 0)
          << "translation bits case " << n;
    }
    const auto& f = out.data();
    const auto& iv = out.inverseData();
    EXPECT_TRUE(f[3] == 0 && f[7] == 0 && f[11] == 0 && f[15] == 1 && iv[3] == 0 && iv[7] == 0 &&
                iv[11] == 0 && iv[15] == 1)
        << "bottom row case " << n;
    // Flags: inv() aborts when noInvert / invertFailed is set.
    (void)out.inv();
  }
  EXPECT_GE(retries_seen, 1);
  std::printf("MT-P1 worst: yaw %.3g quat %.3g rot %.3g invT %.3g (retry cases %d)\n", worst_yaw,
              worst_quat, worst_rot, worst_inv_t, retries_seen);
}

TEST(MapTrackerYaw, YawOfRetriesOnVerticalForwardAxis) {
  // Camera forward axis pointing straight up (+y): m02 = m22 = 0 -> one -90 degree x retry.
  const c8::Quaternion up = c8::Quaternion::xDegrees(-90.0f);
  const float yaw = YawOf(up);
  EXPECT_TRUE(std::isfinite(yaw));
}

}  // namespace
}  // namespace slam_refact::map_tracker

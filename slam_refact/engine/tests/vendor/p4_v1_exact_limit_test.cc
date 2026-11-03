// P4 candidate V1 (AccuracyConfig::v1_exact_limit, third_party/xr_source/patches/
// v1-pose-estimation-exact-limit.patch): where PoseEstimationAnalytic::Evaluate divides by zero in
// its general branch (x30 * x31 == 0, i.e. |reprojection error * weight| below ~7.1e-11), the
// Jacobian takes the exact limit of the pseudo-Huber derivative instead of +-inf / NaN.
//
// Design and test list: build/xr-drift-p4-20261103/v1-vendor-denominator-analysis.md section 2.3, 4, 5.
// Synthetic inputs only (no registered fixture is run here).
//
// Tolerances (fixed before the first run, from the design document unless noted):
//  - relative error of a Jacobian row = max_k |a_k - b_k| / max_k |b_k| ("relative" below);
//  - degenerate band and one-sided checks: Jacobian vs the derivative of the rationalized residual
//    1e-10 + sqrt(c) |s| / sqrt(q + c), q = sqrt(s^2 + c^2), c = 0.005, by automatic differentiation: 1e-12;
//  - original expression vs the closed form for |s| = 1e-3 .. 1e-6: the cancellation bound of the
//    document, 3.5e-8 * (1e-6 / |s|)^2, plus a floor of 1e-12 for the roundoff of the expanded formula;
//  - v1 on: residuals bit-identical to off for every input, Jacobian bit-identical wherever off is finite;
//  - non-degenerate Jacobian against central differences (h = 1e-7): 1e-6 * (1 + |J|); against automatic
//    differentiation of the unrationalized residual: 1e-9;
//  - the limit +-1/sqrt(2) of dr/ds through the whole Jacobian chain (about ten roundings and a
//    reference camera depth that is itself off by an ulp or two): 5e-15. The document's 1e-15 is the
//    figure for the isolated factor x65, which the evaluation does not expose.

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <random>
#include <string>
#include <vector>

#include "c8/geometry/egomotion.h"
#include "c8/hmatrix.h"
#include "c8/hpoint.h"
#include "c8/quaternion.h"
#include "ceres/jet.h"
#include "ceres/rotation.h"
#include "common/vendor_switches.h"
#include "gtest/gtest.h"
#include "p4_v1_scene.h"
#include "reality/engine/geometry/bundle-residual-analytic.h"
#include "reality/engine/geometry/bundle.h"
#include "reality/engine/geometry/observed-point.h"
#include "slam_refact/engine_config.h"

namespace {

// G1 binds its gtest XML to the binary under test (see p3_r7_pose_graph_test.cc).
class BindBinaryEnvironment : public ::testing::Environment {
 public:
  void SetUp() override {
    const char* sha = std::getenv("SR_BINARY_SHA256");
    if (sha != nullptr && *sha != '\0') {
      ::testing::Test::RecordProperty("binary_sha256", std::string(sha));
    }
  }
};
const ::testing::Environment* const kBindBinary =
    ::testing::AddGlobalTestEnvironment(new BindBinaryEnvironment);

using p4v1::Pose;
using Jet = ceres::Jet<double, 6>;

constexpr double kC = 0.0050000000000000001;  // pseudo-Huber delta of the vendored residual
constexpr double kInvSqrt2 = 0.70710678118654752440;

class ScopedV1 {
 public:
  explicit ScopedV1(bool on) : previous_(c8::poseEstimationAnalyticExactLimit()) {
    c8::setPoseEstimationAnalyticExactLimit(on);
  }
  ~ScopedV1() { c8::setPoseEstimationAnalyticExactLimit(previous_); }
  ScopedV1(const ScopedV1&) = delete;
  ScopedV1& operator=(const ScopedV1&) = delete;

 private:
  bool previous_;
};

struct Input {
  double p[6] = {0, 0, 0, 0, 0, 0};  // angle-axis, translation
  double X[3] = {0, 0, 0};
  float u = 0.25f;
  float v = 0.125f;
  float w = 1.0f;
};

struct Output {
  double res[2] = {0, 0};
  double jac[12] = {0};
};

Output Eval(const Input& in, bool v1) {
  const ScopedV1 guard(v1);
  c8::ObservedPoint pt{c8::HPoint2(in.u, in.v), 0, 0.0f, in.w};
  c8::PoseEstimationAnalytic f(in.X[0], in.X[1], in.X[2], pt, 1.0);
  const double* params[1] = {in.p};
  Output out;
  double* jacs[1] = {out.jac};
  EXPECT_TRUE(f.Evaluate(params, out.res, jacs));
  return out;
}

Input MakeInput(const Pose& pose, double depth, double su, double sv, float w = 1.0f) {
  Input in;
  for (int i = 0; i < 3; ++i) {
    in.p[i] = pose.r[i];
    in.p[3 + i] = pose.t[i];
  }
  in.w = w;
  p4v1::WorldPointAtError(pose, depth, su, sv, in.X);
  return in;
}

uint64_t Bits(double d) {
  uint64_t u;
  std::memcpy(&u, &d, sizeof(u));
  return u;
}

bool AllFinite(const double* a, int n) {
  for (int i = 0; i < n; ++i) {
    if (!std::isfinite(a[i])) return false;
  }
  return true;
}

// ---- reference derivatives by automatic differentiation -------------------------------------------

// Reprojection errors (e_u, e_v) and the camera depth as Jets in the six pose parameters.
void ErrorsJet(const Input& in, Jet* eu, Jet* ev, double* depth) {
  Jet p[6];
  for (int i = 0; i < 6; ++i) p[i] = Jet(in.p[i], i);
  Jet X[3] = {Jet(in.X[0]), Jet(in.X[1]), Jet(in.X[2])};
  Jet pc[3];
  ceres::AngleAxisRotatePoint(p, X, pc);
  for (int i = 0; i < 3; ++i) pc[i] += p[3 + i];
  *eu = pc[0] / pc[2] - Jet(static_cast<double>(in.u));
  *ev = pc[1] / pc[2] - Jet(static_cast<double>(in.v));
  *depth = pc[2].a;
}

// x23 = w^2, times 10000 behind the camera (the vendored weight).
double WeightSquared(const Input& in, double depth) {
  const double w = static_cast<double>(in.w);
  return depth < 0 ? 10000.0 * w * w : w * w;
}

// Jacobian row (6 entries) of the rationalized residual 1e-10 + sqrt(c) |s| / sqrt(q + c) with
// s = sqrt(x23) * e, by automatic differentiation of s (the sign taken from `sign_override` when
// non-zero, else from s).
void RationalizedRow(const Input& in, int comp, double sign_override, double* row) {
  Jet eu, ev;
  double depth;
  ErrorsJet(in, &eu, &ev, &depth);
  const Jet e = comp == 0 ? eu : ev;
  const Jet s = e * std::sqrt(WeightSquared(in, depth));
  const double sign = sign_override != 0.0 ? sign_override : (s.a < 0 ? -1.0 : 1.0);
  const Jet sa = s * sign;
  const Jet q = ceres::sqrt(sa * sa + Jet(kC * kC));
  const Jet r = Jet(std::sqrt(kC)) * sa / ceres::sqrt(q + Jet(kC)) + Jet(1e-10);
  for (int k = 0; k < 6; ++k) row[k] = r.v[k];
}

// Jacobian row of the original (unrationalized) residual, for inputs well away from s = 0.
void OriginalRow(const Input& in, int comp, double* row) {
  Jet eu, ev;
  double depth;
  ErrorsJet(in, &eu, &ev, &depth);
  const Jet e = comp == 0 ? eu : ev;
  const Jet s = e * std::sqrt(WeightSquared(in, depth));
  const Jet x30 = ceres::sqrt(s * s + Jet(2.5000000000000001e-5));
  const Jet r = Jet(0.070710678118654752) * ceres::sqrt(x30 - Jet(kC)) + Jet(1e-10);
  for (int k = 0; k < 6; ++k) row[k] = r.v[k];
}

double RelativeRowError(const double* a, const double* b) {
  double scale = 0.0;
  double err = 0.0;
  for (int k = 0; k < 6; ++k) {
    scale = std::max(scale, std::fabs(b[k]));
    err = std::max(err, std::fabs(a[k] - b[k]));
  }
  return scale > 0.0 ? err / scale : err;
}

// ---- degenerate inputs ----------------------------------------------------------------------------

// Sign of the reprojection error (x29 resp. x36) of the degenerate component.
enum class Kind { kPositive, kNegative, kZero };

// Searches the half-ulp neighbourhood of an exactly representable reprojection error for an input on
// which the original Jacobian entry of component `comp` is not finite and the error is positive,
// negative or exactly +0 (NaN: 0 / 0). The error sign is read off the original Jacobian itself
// (jac = x65 * x28 * ..., x28 = 1 / camera depth), not from a reference computation whose rounding
// differs at the 1e-16 level.
bool FindDegenerate(const Pose& pose, double depth, int comp, Kind kind, Input* found, double* sign = nullptr) {
  const int entry = comp == 0 ? 3 : 10;
  for (int step = 0; step < 16 * 801; ++step) {
    // a few nearby depths, in case no step of the first one lands on an exactly zero error
    const double d = depth * (1.0 + 0.001 * static_cast<double>(step / 801));
    const double sigma = static_cast<double>(step % 801 - 400) * 2.7755575615628914e-17;  // 2^-55
    const Input in = comp == 0 ? MakeInput(pose, d, sigma, 0.01) : MakeInput(pose, d, 0.01, sigma);
    const Output off = Eval(in, false);
    const double j = off.jac[entry];
    Kind got;
    if (std::isnan(j)) {
      got = Kind::kZero;
    } else if (std::isinf(j)) {
      got = ((j > 0) != (depth < 0)) ? Kind::kPositive : Kind::kNegative;
    } else {
      continue;
    }
    if (got == kind) {
      *found = in;
      if (sign != nullptr) *sign = kind == Kind::kNegative ? -1.0 : 1.0;
      return true;
    }
  }
  return false;
}

const Pose kPoseA = {{0.31, -0.42, 0.27}, {0.12, -0.07, 0.35}};
const Pose kPoseB = {{1.4, -1.1, 1.5}, {-0.2, 0.1, 0.4}};
const Pose kPoseSmall = {{3e-4, -2e-4, 1e-4}, {0.05, 0.02, 0.1}};

const int kEntriesU[5] = {0, 1, 2, 3, 5};
const int kEntriesV[5] = {6, 7, 8, 10, 11};

// 1. A zero denominator gives a finite Jacobian that is the exact limit; off it stays +-inf / NaN.
TEST(P4V1ExactLimit, BoundaryInputsGiveFiniteJacobian) {
  struct Case {
    const char* name;
    Pose pose;
    double depth;
    int comp;
    Kind kind;
  };
  const std::vector<Case> cases = {
      {"u error > 0 (A)", kPoseA, 2.5, 0, Kind::kPositive}, {"u error < 0 (A)", kPoseA, 2.5, 0, Kind::kNegative},
      {"u error == +0 (A)", kPoseA, 2.5, 0, Kind::kZero},       {"v error > 0 (A)", kPoseA, 2.5, 1, Kind::kPositive},
      {"v error < 0 (A)", kPoseA, 2.5, 1, Kind::kNegative}, {"v error == +0 (A)", kPoseA, 2.5, 1, Kind::kZero},
      {"u error > 0 (B)", kPoseB, 2.5, 0, Kind::kPositive}, {"u error < 0 (B)", kPoseB, 2.5, 0, Kind::kNegative},
      {"behind the camera, u error > 0", kPoseA, -2.5, 0, Kind::kPositive},
      {"behind the camera, v error < 0", kPoseA, -2.5, 1, Kind::kNegative},
  };
  for (const Case& c : cases) {
    Input in;
    ASSERT_TRUE(FindDegenerate(c.pose, c.depth, c.comp, c.kind, &in)) << c.name;
    const Output off = Eval(in, false);
    const Output on = Eval(in, true);
    const int* entries = c.comp == 0 ? kEntriesU : kEntriesV;
    const int* other = c.comp == 0 ? kEntriesV : kEntriesU;
    // off: the degenerate component's five entries are not finite, its residual is exactly 1e-10
    for (int i = 0; i < 5; ++i) EXPECT_FALSE(std::isfinite(off.jac[entries[i]])) << c.name << " off " << entries[i];
    EXPECT_EQ(off.res[c.comp], 1e-10) << c.name;
    // on: everything finite, residuals untouched, the other component bit for bit as off
    EXPECT_TRUE(AllFinite(on.jac, 12)) << c.name;
    EXPECT_EQ(Bits(on.res[0]), Bits(off.res[0])) << c.name;
    EXPECT_EQ(Bits(on.res[1]), Bits(off.res[1])) << c.name;
    for (int i = 0; i < 5; ++i) EXPECT_EQ(Bits(on.jac[other[i]]), Bits(off.jac[other[i]])) << c.name;
    EXPECT_EQ(on.jac[c.comp == 0 ? 4 : 9], 0.0) << c.name;
    // the limit: sign(e) * (1 / sqrt(2)) * sqrt(x23) * de/dp (NaN case: the sign of x29 == +0)
    const double sign = c.kind == Kind::kNegative ? -1.0 : 1.0;
    double ref[6];
    RationalizedRow(in, c.comp, sign, ref);
    double got[6];
    for (int k = 0; k < 6; ++k) got[k] = on.jac[(c.comp == 0 ? 0 : 6) + k];
    EXPECT_LT(RelativeRowError(got, ref), 1e-12) << c.name;
  }
  {  // both components degenerate at once
    Input in = MakeInput(kPoseA, 2.5, 3e-11, -3e-11);
    const Output off = Eval(in, false);
    const Output on = Eval(in, true);
    for (int i = 0; i < 5; ++i) {
      EXPECT_FALSE(std::isfinite(off.jac[kEntriesU[i]]));
      EXPECT_FALSE(std::isfinite(off.jac[kEntriesV[i]]));
    }
    EXPECT_TRUE(AllFinite(on.jac, 12));
    double ref_u[6], ref_v[6], got_u[6], got_v[6];
    RationalizedRow(in, 0, +1.0, ref_u);
    RationalizedRow(in, 1, -1.0, ref_v);
    for (int k = 0; k < 6; ++k) {
      got_u[k] = on.jac[k];
      got_v[k] = on.jac[6 + k];
    }
    EXPECT_LT(RelativeRowError(got_u, ref_u), 1e-12);
    EXPECT_LT(RelativeRowError(got_v, ref_v), 1e-12);
  }
}

// 2. The closed form is continuous with the original expression, and its value at the zero
// denominator is the limit +-1 / sqrt(2) of dr/ds.
TEST(P4V1ExactLimit, LimitIsContinuousWithTheGeneralFormula) {
  for (const double magnitude : {1e-3, 1e-4, 1e-5, 1e-6}) {
    for (const double sign : {1.0, -1.0}) {
      const double s_u = sign * magnitude;  // weight 1, in front of the camera: s equals the error
      const Input in = MakeInput(kPoseA, 2.5, s_u, 0.01);
      const Output off = Eval(in, false);
      double ref[6];
      RationalizedRow(in, 0, 0.0, ref);
      double got[6];
      for (int k = 0; k < 6; ++k) got[k] = off.jac[k];
      const double tol = 3.5e-8 * (1e-6 / magnitude) * (1e-6 / magnitude) + 1e-12;
      EXPECT_LT(RelativeRowError(got, ref), tol) << "s " << s_u;
      if (magnitude <= 1e-5) {  // (ii) |dr/ds| sqrt(2) - 1 = -3 s^2 / (8 c^2) + O(s^4 / c^4)
        Jet eu, ev;
        double depth;
        ErrorsJet(in, &eu, &ev, &depth);
        int k_max = 0;
        for (int k = 1; k < 6; ++k) {
          if (std::fabs(eu.v[k]) > std::fabs(eu.v[k_max])) k_max = k;
        }
        const double drds = off.jac[k_max] / eu.v[k_max];  // weight 1: ds = de
        EXPECT_NEAR(std::fabs(drds) * std::sqrt(2.0) - 1.0, -3.0 * s_u * s_u / (8.0 * kC * kC), tol) << "s " << s_u;
      }
    }
  }
  // (iii) at the zero denominator the Jacobian is +-1/sqrt(2) * sqrt(x23) * de/dp (jac[3] = d/dtx:
  // de_u/dtx = 1 / camera depth), through the whole chain.
  for (const double depth : {2.5, -2.5}) {
    Input in;
    ASSERT_TRUE(FindDegenerate(kPoseA, depth, 0, Kind::kPositive, &in));
    const Output on = Eval(in, true);
    Jet eu, ev;
    double z;
    ErrorsJet(in, &eu, &ev, &z);
    const double expected = kInvSqrt2 * std::sqrt(WeightSquared(in, z)) / z;
    EXPECT_LT(std::fabs(on.jac[3] - expected) / std::fabs(expected), 5e-15) << "depth " << depth;
  }
}

// 3. The Jacobian agrees with numeric and automatic differentiation: on non-degenerate inputs
// (where on == off bit for bit), in the degenerate band, and one-sided at the kink s == 0.
TEST(P4V1ExactLimit, MatchesNumericAndAutodiffJacobian) {
  std::mt19937 gen(20261103);
  std::uniform_real_distribution<double> ur(-1.0, 1.0);
  int checked = 0;
  for (int i = 0; i < 100; ++i) {
    Pose pose;
    do {
      for (int k = 0; k < 3; ++k) {
        pose.r[k] = 0.9 * ur(gen);
        pose.t[k] = 0.5 * ur(gen);
      }
    } while (pose.r[0] * pose.r[0] + pose.r[1] * pose.r[1] + pose.r[2] * pose.r[2] < 1e-3);  // general branch
    const double depth = 1.5 + 3.5 * (ur(gen) * 0.5 + 0.5);
    const double du = (0.02 + 0.08 * (ur(gen) * 0.5 + 0.5)) * (ur(gen) < 0 ? -1.0 : 1.0);
    const double dv = (0.02 + 0.08 * (ur(gen) * 0.5 + 0.5)) * (ur(gen) < 0 ? -1.0 : 1.0);
    const Input in = MakeInput(pose, depth, du, dv, i % 2 == 0 ? 1.0f : 0.5f);
    const Output off = Eval(in, false);
    const Output on = Eval(in, true);
    for (int k = 0; k < 12; ++k) ASSERT_EQ(Bits(on.jac[k]), Bits(off.jac[k])) << i << " " << k;
    for (int comp = 0; comp < 2; ++comp) {
      double ad[6];
      OriginalRow(in, comp, ad);
      double got[6];
      for (int k = 0; k < 6; ++k) got[k] = on.jac[comp * 6 + k];
      EXPECT_LT(RelativeRowError(got, ad), 1e-9) << i << " comp " << comp;
      for (int k = 0; k < 6; ++k) {  // central differences of the vendored residual
        constexpr double h = 1e-7;
        Input hi = in, lo = in;
        hi.p[k] += h;
        lo.p[k] -= h;
        const double fd = (Eval(hi, false).res[comp] - Eval(lo, false).res[comp]) / (2.0 * h);
        EXPECT_NEAR(fd, got[k], 1e-6 * (1.0 + std::fabs(got[k]))) << i << " comp " << comp << " k " << k;
      }
    }
    ++checked;
  }
  EXPECT_EQ(checked, 100);

  // the degenerate band 0 < |s| < 7.1e-11: automatic differentiation of the rationalized residual
  for (const Kind kind : {Kind::kPositive, Kind::kNegative}) {
    for (const int comp : {0, 1}) {
      Input in;
      double sign = 0.0;
      ASSERT_TRUE(FindDegenerate(kPoseB, 2.5, comp, kind, &in, &sign));
      const Output on = Eval(in, true);
      double ref[6];
      RationalizedRow(in, comp, sign, ref);
      double got[6];
      for (int k = 0; k < 6; ++k) got[k] = on.jac[comp * 6 + k];
      EXPECT_LT(RelativeRowError(got, ref), 1e-12);
    }
  }

  // s == 0 exactly (kink): the one-sided difference along the direction of increasing s equals the
  // chosen (+) one-sided derivative, O(h).
  Input in;
  ASSERT_TRUE(FindDegenerate(kPoseA, 2.5, 0, Kind::kZero, &in));
  const Output on = Eval(in, true);
  Jet eu, ev;
  double depth;
  ErrorsJet(in, &eu, &ev, &depth);
  double norm = 0.0;
  for (int k = 0; k < 6; ++k) norm += eu.v[k] * eu.v[k];
  norm = std::sqrt(norm);
  constexpr double h = 1e-5;
  Input moved = in;
  double jd = 0.0;
  for (int k = 0; k < 6; ++k) {
    const double d = eu.v[k] / norm;  // grad(e) . d = |grad(e)| > 0, weight 1: s increases
    moved.p[k] += h * d;
    jd += on.jac[k] * d;
  }
  const double fd = (Eval(moved, false).res[0] - 1e-10) / h;
  EXPECT_NEAR(fd, jd, 1e-3 * std::fabs(jd));
  EXPECT_NEAR(jd, kInvSqrt2 * norm, 1e-12 * norm);
}

// Golden: the original evaluation (bundle-residual-analytic.cc without the V1 patch), inputs and the
// 14 outputs as bits (tests/vendor/p4_v1_golden_gen.cc).
struct GoldenRow {
  const char* tag;
  uint64_t p[6];
  uint64_t X[3];
  uint32_t u, v, w;
  uint64_t out[14];
};
const GoldenRow kGolden[] = {
#include "p4_v1_golden.inc"
};

double FromBits(uint64_t b) {
  double d;
  std::memcpy(&d, &b, sizeof(d));
  return d;
}
float FromBits32(uint32_t b) {
  float f;
  std::memcpy(&f, &b, sizeof(f));
  return f;
}

Input FromGolden(const GoldenRow& g) {
  Input in;
  for (int i = 0; i < 6; ++i) in.p[i] = FromBits(g.p[i]);
  for (int i = 0; i < 3; ++i) in.X[i] = FromBits(g.X[i]);
  in.u = FromBits32(g.u);
  in.v = FromBits32(g.v);
  in.w = FromBits32(g.w);
  return in;
}

// 4. With the switch off the evaluation is the original one bit for bit (also where it is inf / NaN);
// with it on the residuals are the same bits and the Jacobian differs only where off is not finite.
TEST(P4V1ExactLimit, OffIsBitIdenticalToOriginal) {
  const size_t n = sizeof(kGolden) / sizeof(kGolden[0]);
  ASSERT_GE(n, 70u);
  size_t nonfinite_rows = 0;
  for (const GoldenRow& g : kGolden) {
    const Input in = FromGolden(g);
    const Output off = Eval(in, false);
    const Output on = Eval(in, true);
    for (int i = 0; i < 2; ++i) EXPECT_EQ(Bits(off.res[i]), g.out[i]) << g.tag << " res " << i;
    for (int i = 0; i < 12; ++i) EXPECT_EQ(Bits(off.jac[i]), g.out[2 + i]) << g.tag << " jac " << i;
    for (int i = 0; i < 2; ++i) EXPECT_EQ(Bits(on.res[i]), g.out[i]) << g.tag << " on res " << i;
    bool any_nonfinite = false;
    for (int i = 0; i < 12; ++i) {
      if (std::isfinite(FromBits(g.out[2 + i]))) {
        EXPECT_EQ(Bits(on.jac[i]), g.out[2 + i]) << g.tag << " on jac " << i;
      } else {
        any_nonfinite = true;
        EXPECT_TRUE(std::isfinite(on.jac[i])) << g.tag << " on jac " << i;
      }
    }
    nonfinite_rows += any_nonfinite ? 1 : 0;
  }
  ::testing::Test::RecordProperty("golden_rows", static_cast<int>(n));
  ::testing::Test::RecordProperty("golden_rows_with_nonfinite_original_jacobian", static_cast<int>(nonfinite_rows));
  EXPECT_GT(nonfinite_rows, 5u);  // the corpus really contains the zero denominators
}

// ---- solver level (Q15) ---------------------------------------------------------------------------

struct PnpScene {
  c8::HMatrix truth = c8::HMatrixGen::i();
  std::vector<c8::HPoint3> world;
  std::vector<c8::HPoint2> rays;
};

// The scene of MapTrackerPnp.Q15* (pnp_test.cc): 60 points in front of the camera, exact rays.
PnpScene MakeQ15Scene() {
  PnpScene s;
  s.truth = c8::cameraMotion(c8::HVector3(0.3f, -0.2f, 0.5f), c8::Quaternion::fromPitchYawRollDegrees(5, 20, -3));
  std::mt19937 gen(8008);
  std::uniform_real_distribution<float> ux(-0.5f, 0.5f);
  std::uniform_real_distribution<float> uy(-0.38f, 0.38f);
  std::uniform_real_distribution<float> ud(2.0f, 6.0f);
  for (int i = 0; i < 80; ++i) {
    const float rx = ux(gen);
    const float ry = uy(gen);
    const float d = ud(gen);
    const c8::HPoint3 pc(rx * d, ry * d, d);
    if (i < 60) s.world.push_back(s.truth * pc);
  }
  for (const c8::HPoint3& x : s.world) {
    const c8::HPoint3 pc = s.truth.inv() * x;
    s.rays.emplace_back(pc.x() / pc.z(), pc.y() / pc.z());
  }
  return s;
}

c8::Vector<c8::ObservedPoint> Observations(const std::vector<c8::HPoint2>& rays) {
  c8::Vector<c8::ObservedPoint> obs;
  for (const c8::HPoint2& r : rays) obs.push_back(c8::ObservedPoint{r, 0, 0.0f, 1.0f});
  return obs;
}

c8::PoseEstimationAnalyticalParams Params1272() {
  c8::PoseEstimationAnalyticalParams p;
  p.suppressOutliers = true;
  p.scale = 1.0;
  p.useImageTargetCostFunction = false;
  p.cameraMotionWeight = 0.0;
  p.logReport = false;
  return p;
}

float MaxAbsDiff(const c8::HMatrix& a, const c8::HMatrix& b) {
  float m = 0.0f;
  for (size_t i = 0; i < 16; ++i) m = std::max(m, std::fabs(a.data()[i] - b.data()[i]));
  return m;
}

// 5. Q15: started at the exact pose every residual is 0, the original Jacobian is NaN and the solve
// fails (the MUST-PASS test MapTrackerPnp.Q15AnalyticFailsAtExactPose keeps asserting that with the
// switch off); with V1 on the solve runs and stays at the exact pose. A noisy scene is unaffected.
TEST(P4V1ExactLimit, SolverLevelQ15) {
  const PnpScene s = MakeQ15Scene();
  {
    c8::HMatrix pose = s.truth;
    const ScopedV1 off(false);
    EXPECT_FALSE(c8::poseEstimationAnalytical(Observations(s.rays), s.world, {}, &pose, Params1272()));
  }
  {
    c8::HMatrix pose = s.truth;
    const ScopedV1 on(true);
    const bool ok = c8::poseEstimationAnalytical(Observations(s.rays), s.world, {}, &pose, Params1272());
    EXPECT_TRUE(ok);
    EXPECT_LT(MaxAbsDiff(pose, s.truth), 1e-6f);
  }
  {  // noisy rays, started 2 cm / 0.5 degree away: the same bits with the switch on and off
    PnpScene noisy = s;
    std::mt19937 gen(1146);
    std::normal_distribution<float> noise(0.0f, 1e-3f);
    for (c8::HPoint2& r : noisy.rays) r = c8::HPoint2(r.x() + noise(gen), r.y() + noise(gen));
    const c8::HMatrix start = c8::cameraMotion(c8::HVector3(0.32f, -0.2f, 0.5f),
                                               c8::Quaternion::fromPitchYawRollDegrees(5.5f, 20, -3));
    c8::HMatrix pose_off = start;
    c8::HMatrix pose_on = start;
    bool ok_off, ok_on;
    {
      const ScopedV1 off(false);
      ok_off = c8::poseEstimationAnalytical(Observations(noisy.rays), noisy.world, {}, &pose_off, Params1272());
    }
    {
      const ScopedV1 on(true);
      ok_on = c8::poseEstimationAnalytical(Observations(noisy.rays), noisy.world, {}, &pose_on, Params1272());
    }
    ASSERT_TRUE(ok_off);
    ASSERT_TRUE(ok_on);
    EXPECT_EQ(std::memcmp(pose_off.data().data(), pose_on.data().data(), 16 * sizeof(float)), 0);
  }
}

// 6. The small rotation branch (theta^2 < 1e-6) is not changed by V1: it keeps its own zero rows.
TEST(P4V1ExactLimit, SmallAngleBranchUnchanged) {
  int degenerate = 0;
  for (const double sigma : {0.0, 1e-13, -1e-13, 3e-11, -3e-11, 7.2e-11, 1e-9, 1e-3}) {
    for (const double sv : {0.01, 3e-11}) {
      const Input in = MakeInput(kPoseSmall, 2.5, sigma, sv);
      const Output off = Eval(in, false);
      const Output on = Eval(in, true);
      for (int k = 0; k < 14; ++k) {
        const double a = k < 2 ? off.res[k] : off.jac[k - 2];
        const double b = k < 2 ? on.res[k] : on.jac[k - 2];
        EXPECT_EQ(Bits(a), Bits(b)) << "sigma " << sigma << " sv " << sv << " k " << k;
      }
      if (off.res[0] == 1e-10 || off.res[1] == 1e-10) {  // x7 == 0: the original's explicit zero rows
        ++degenerate;
        EXPECT_TRUE(AllFinite(off.jac, 12));
      }
    }
  }
  EXPECT_GT(degenerate, 0);
}

// 7. A non-finite value that is not the zero denominator is not hidden: a point on the camera plane
// (x22 == 0), a NaN point and a NaN parameter stay non-finite, bit for bit as with the switch off.
TEST(P4V1ExactLimit, OtherNonFiniteIsNotMasked) {
  const auto expect_same_nonfinite = [](const Input& in, const char* what) {
    const Output off = Eval(in, false);
    const Output on = Eval(in, true);
    EXPECT_FALSE(AllFinite(off.jac, 12) && AllFinite(off.res, 2)) << what;
    EXPECT_EQ(std::memcmp(&off, &on, sizeof(Output)), 0) << what;
  };
  {  // camera depth exactly 0: scan the translation z in ulp steps around the value that cancels it
    Input in = MakeInput(kPoseA, 2.5, 0.0, 0.0);
    Jet eu, ev;
    double z;
    ErrorsJet(in, &eu, &ev, &z);
    const double tz0 = in.p[5] - z;
    bool found = false;
    for (int k = -64; k <= 64 && !found; ++k) {
      in.p[5] = tz0 + static_cast<double>(k) * 1.1102230246251565e-16;
      const Output off = Eval(in, false);
      if (!(AllFinite(off.jac, 12) && AllFinite(off.res, 2))) {
        found = true;
        expect_same_nonfinite(in, "x22 == 0");
      }
    }
    EXPECT_TRUE(found);
  }
  {
    Input in = MakeInput(kPoseA, 2.5, 0.01, 0.01);
    in.X[1] = std::nan("");
    expect_same_nonfinite(in, "NaN point");
  }
  {
    Input in = MakeInput(kPoseA, 2.5, 0.01, 0.01);
    in.p[4] = std::nan("");
    expect_same_nonfinite(in, "NaN parameter");
  }
}

// 8. The switch: default off; on only with `enabled` and `v1_exact_limit`; restored by the guard.
TEST(P4V1ExactLimit, FlagPlumbing) {
  const slam_refact::AccuracyConfig defaults;
  EXPECT_FALSE(defaults.v1_exact_limit);
  EXPECT_FALSE(defaults.r7p_loop_select);
  EXPECT_FALSE(c8::poseEstimationAnalyticExactLimit());
  slam_refact::AccuracyConfig cfg;
  cfg.enabled = true;
  cfg.v1_exact_limit = true;
  slam_refact::ApplyVendorAccuracy(cfg);
  EXPECT_TRUE(c8::poseEstimationAnalyticExactLimit());
  cfg.enabled = false;  // v1 without enabled
  slam_refact::ApplyVendorAccuracy(cfg);
  EXPECT_FALSE(c8::poseEstimationAnalyticExactLimit());
  cfg.enabled = true;
  cfg.v1_exact_limit = false;  // enabled without v1
  slam_refact::ApplyVendorAccuracy(cfg);
  EXPECT_FALSE(c8::poseEstimationAnalyticExactLimit());
  {
    const ScopedV1 on(true);
    EXPECT_TRUE(c8::poseEstimationAnalyticExactLimit());
  }
  EXPECT_FALSE(c8::poseEstimationAnalyticExactLimit());
}

}  // namespace

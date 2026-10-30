// MS-P1..MS-P9 (architecture.md §6.6.5, S5 §8.2): C++ port vs the original-WASM golden
// tests/data/metric-scale/probe-e/result.json. Tolerances are the pre-registered ones of
// mse-probe.mjs / S5 §8.2. Inputs that the golden stores only as a recipe (LCG seeds, scenario
// formulas) are regenerated exactly like mse-probe.mjs.

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <limits>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "c8/hmatrix.h"
#include "config/params.h"
#include "config/settings.h"
#include "metric_scale/alignment_math.h"
#include "metric_scale/fixed_products.h"
#include "metric_scale/kalman.h"
#include "metric_scale/metric_scale_estimator.h"
#include "metric_scale/metric_scale_estimator_internal.h"
#include "wp4_json.h"

namespace slam_refact::metric_scale {
namespace {

const wp4_json::Value& Results() {
  static const wp4_json::Value kRoot =
      wp4_json::Load(std::string(SR_TEST_DATA_DIR) + "/metric-scale/probe-e/result.json");
  return kRoot["results"];
}

double Ulp(double x) {
  const float a = std::fabs(static_cast<float>(x));
  return static_cast<double>(std::nextafter(a, std::numeric_limits<float>::infinity()) - a);
}

// Relative difference; a zero reference requires an exact zero (rel 0 / 0 is undefined).
double Rel(double a, double b) {
  if (b == 0) {
    return a == 0 ? 0.0 : std::numeric_limits<double>::infinity();
  }
  return std::fabs(a - b) / std::fabs(b);
}

// mse-probe.mjs lcg(seed): (Math.imul(s, 1664525) + 1013904223) >>> 0, / 2^32.
struct Lcg {
  uint32_t s;
  double Next() {
    s = s * 1664525u + 1013904223u;
    return s / 4294967296.0;
  }
};

double JsRound(double x) { return std::floor(x + 0.5); }  // Math.round

std::vector<float> RotY(double th) {  // mse-probe.mjs rotY, row-major, f32
  const double c = std::cos(th);
  const double s = std::sin(th);
  return {static_cast<float>(c), 0.f, static_cast<float>(s), 0.f, 1.f, 0.f,
          static_cast<float>(-s), 0.f, static_cast<float>(c)};
}

// mse-probe.mjs putPose: R row-major, t; inverse translation computed in double then f32.
c8::HMatrix PutPose(const std::vector<float>& r, const float t[3]) {
  float m[16] = {0};
  float inv[16] = {0};
  for (int row = 0; row < 3; ++row) {
    for (int col = 0; col < 3; ++col) {
      m[4 * col + row] = r[3 * row + col];
      inv[4 * col + row] = r[3 * col + row];
    }
  }
  m[12] = t[0];
  m[13] = t[1];
  m[14] = t[2];
  m[15] = 1.f;
  for (int row = 0; row < 3; ++row) {
    const double v = static_cast<double>(r[row]) * t[0] + static_cast<double>(r[3 + row]) * t[1] +
                     static_cast<double>(r[6 + row]) * t[2];
    inv[12 + row] = static_cast<float>(-v);
  }
  inv[15] = 1.f;
  return c8::HMatrix(m, inv);
}

// ---- MS-P1: crossCorrelate (1183), abs <= 4 ulp(max|out|) ----
TEST(MetricScaleGolden, P1CrossCorrelate) {
  const wp4_json::Value& r = Results()["xcorr_1183"];
  ASSERT_EQ(r["inputs"].size(), 3u);
  for (size_t c = 0; c < r["inputs"].size(); ++c) {
    const std::vector<float> out =
        CrossCorrelate(r["inputs"][c]["x"].floats(), r["inputs"][c]["y"].floats());
    const std::vector<float> golden = r["outputs"][c].floats();
    ASSERT_EQ(out.size(), golden.size()) << "case " << c;
    double mx = 0;
    for (float g : golden) mx = std::max(mx, static_cast<double>(std::fabs(g)));
    const double tol = 4 * Ulp(mx == 0 ? 1 : mx);
    for (size_t i = 0; i < out.size(); ++i) {
      EXPECT_LE(std::fabs(static_cast<double>(out[i]) - golden[i]), tol) << "case " << c << " i " << i;
    }
  }
}

// ---- MS-P2: methods 1/2 (1191, 1192), scale rel <= 1e-6, residual rel <= 1e-5 ----
TEST(MetricScaleGolden, P2ScaleFits) {
  const wp4_json::Value& r = Results()["scale_fit_1191_1192"];
  Lcg rnd{2};
  const int n = 200;
  std::vector<Vec3> a(n);
  std::vector<Vec3> c(n);
  for (int i = 0; i < n; ++i) {
    for (int j = 0; j < 3; ++j) a[i][j] = static_cast<float>(-2 + 4 * rnd.Next());
  }
  for (int i = 0; i < n; ++i) {
    for (int j = 0; j < 3; ++j) c[i][j] = static_cast<float>(0.37 * a[i][j] + 0.05 * (rnd.Next() - 0.5));
  }
  const std::vector<std::vector<Vec3>> as = {a, {a[0]}};
  const std::vector<std::vector<Vec3>> cs = {c, {c[0]}};
  for (size_t k = 0; k < 2; ++k) {
    const ScaleFit f1 = ScaleCameraSpace(as[k], cs[k]);
    const ScaleFit f2 = ScaleL2(as[k], cs[k]);
    const wp4_json::Value& g = r["outputs"][k];
    EXPECT_LE(Rel(f1.scale, g["scaleCameraSpace"]["scale"].d()), 1e-6) << k;
    EXPECT_LE(Rel(f1.residual, g["scaleCameraSpace"]["residual"].d()), 1e-5) << k;
    EXPECT_LE(Rel(f2.scale, g["scaleL2"]["scale"].d()), 1e-6) << k;
    EXPECT_LE(Rel(f2.residual, g["scaleL2"]["residual"].d()), 1e-5) << k;
  }
  // NaN = NaN: empty inputs give NaN scale (0x7fc00000) like the original.
  EXPECT_TRUE(std::isnan(ScaleCameraSpace({}, {}).scale));
  EXPECT_TRUE(std::isnan(ScaleL2({}, {}).scale));
}

// ---- MS-P3: interpolate (1198), abs <= 2 ulp + 1e-7, empty exact ----
TEST(MetricScaleGolden, P3Interpolate) {
  const wp4_json::Value& r = Results()["interp_1198"];
  ASSERT_EQ(r["inputs"].size(), 4u);
  for (size_t c = 0; c < r["inputs"].size(); ++c) {
    const wp4_json::Value& in = r["inputs"][c];
    const std::vector<float> out = Interpolate(in["ts"].floats(), in["xs"].floats(), in["td"].floats());
    const std::vector<float> golden = r["outputs"][c].floats();
    ASSERT_EQ(out.size(), golden.size()) << "case " << c;
    for (size_t i = 0; i < out.size(); ++i) {
      EXPECT_LE(std::fabs(static_cast<double>(out[i]) - golden[i]), 2 * Ulp(golden[i]) + 1e-7)
          << "case " << c << " i " << i;
    }
  }
}

// ---- MS-P4: meanDt rel <= 1e-6; crop / dedupe / merge bitwise ----
TEST(MetricScaleGolden, P4MeanCropMergeDedupe) {
  const wp4_json::Value& r = Results()["misc_1179_1186_1187_1188"];
  const wp4_json::Value& in = r["inputs"];
  const wp4_json::Value& out = r["outputs"];
  const std::vector<float> ts = in["ts"].floats();
  EXPECT_LE(Rel(MeanDt(ts), out["meanDt"].d()), 1e-6);
  std::vector<float> t = ts;
  std::vector<Vec3> data;
  for (size_t i = 0; i < ts.size(); ++i) {
    data.push_back({static_cast<float>(i), -static_cast<float>(i), ts[i]});
  }
  CropToRange(in["cropLo"].f(), in["cropHi"].f(), &t, &data);
  EXPECT_EQ(t, out["croppedT"].floats());
  EXPECT_EQ(Dedupe(in["dedupeIn"].floats(), 0.001f), out["dedupe"].floats());
  EXPECT_EQ(MergeTimes(in["mergeA"].floats(), in["mergeB"].floats()), out["merge"].floats());
}

// ---- MS-P5: gaussian kernel rel <= 2e-6, smoothed abs <= 1e-6 ----
TEST(MetricScaleGolden, P5Gaussian) {
  const wp4_json::Value& r = Results()["gauss_1199_1200"];
  const std::vector<float> sig = r["inputs"]["signal"].floats();
  ASSERT_EQ(r["outputs"].size(), 4u);
  for (size_t c = 0; c < r["outputs"].size(); ++c) {
    const wp4_json::Value& g = r["outputs"][c];
    const std::vector<float> k = GaussianKernel(g["sigma"].f(), static_cast<int>(g["size"].ll()));
    const std::vector<float> gk = g["kernel"].floats();
    ASSERT_EQ(k.size(), gk.size()) << c;
    for (size_t i = 0; i < k.size(); ++i) EXPECT_LE(Rel(k[i], gk[i]), 2e-6) << c << " " << i;
    const std::vector<float> s = GaussianSmooth(sig, g["sigma"].f(), static_cast<int>(g["size"].ll()));
    const std::vector<float> gs = g["smoothed"].floats();
    ASSERT_EQ(s.size(), gs.size()) << c;
    for (size_t i = 0; i < s.size(); ++i) EXPECT_LE(std::fabs(s[i] - gs[i]), 1e-6) << c << " " << i;
  }
}

// ---- MS-P6: FftCompute, frequencies bitwise, amplitudes abs <= 1e-5 max + 1e-7 ----
TEST(MetricScaleGolden, P6Fft) {
  const wp4_json::Value& r = Results()["fft_1240_1241"];
  Lcg rnd{6};
  ASSERT_EQ(r["outputs"].size(), 5u);
  for (size_t c = 0; c < r["outputs"].size(); ++c) {
    const wp4_json::Value& g = r["outputs"][c];
    const int n = static_cast<int>(g["n"].ll());
    std::vector<float> data(n);
    for (int i = 0; i < n; ++i) {
      data[i] = static_cast<float>(std::sin(2 * M_PI * 3.0 * i / 60) + 0.5 * std::cos(2 * M_PI * 7.5 * i / 60) +
                                   0.1 * (rnd.Next() - 0.5));
    }
    FftCompute fft(n);
    fft.Compute(data, 60.0f);
    EXPECT_EQ(fft.frequencies(), g["frequencies"].floats()) << "n " << n;
    const std::vector<float> ga = g["amplitudes"].floats();
    ASSERT_EQ(fft.amplitudes().size(), ga.size());
    double mx = 0;
    for (float a : ga) mx = std::max(mx, static_cast<double>(a));
    for (size_t i = 0; i < ga.size(); ++i) {
      EXPECT_LE(std::fabs(fft.amplitudes()[i] - ga[i]), 1e-5 * mx + 1e-7) << "n " << n << " bin " << i;
    }
  }
}

// ---- MS-P7: position Kalman (1169) ----
TEST(MetricScaleGolden, P7PositionKalman) {
  config::RegisterDefaults();
  const wp4_json::Value& r = Results()["kalman_position_1169"];
  const wp4_json::Value& steps = r["outputs"]["steps"];
  PositionKalman kf;
  const int64_t base = 1000000000LL;
  size_t si = 0;
  for (int k = 0; k < 40; ++k) {
    const double t = k / 30.0;
    const int64_t tns = base + static_cast<int64_t>(JsRound(t * 1e9));
    const float tr[3] = {static_cast<float>(0.5 * std::sin(2 * t)), static_cast<float>(0.1 * t * t),
                         static_cast<float>(-0.3 * t)};
    const bool ret = kf.Push(PutPose(RotY(0.2 * t), tr), base, tns, 2000, 25, 50, false);
    if (si < steps.size() && steps[si]["k"].ll() == k) {
      const wp4_json::Value& g = steps[si];
      EXPECT_EQ(ret, g["ret"].ll() != 0) << k;
      EXPECT_EQ(kf.last_t, g["lastT"].f()) << k;
      for (int i = 0; i < 9; ++i) {
        const double gx = g["x"][i].d();
        const double gp = g["xPred"][i].d();
        EXPECT_LE(std::fabs(kf.x(i) - gx), 1e-5 * (1 + std::fabs(gx))) << "k " << k << " x" << i;
        EXPECT_LE(std::fabs(kf.x_pred(i) - gp), 1e-5 * (1 + std::fabs(gp))) << "k " << k << " xPred" << i;
        if (k > 0) {
          EXPECT_LE(Rel(kf.p(i, i), g["Pdiag"][i].d()), 1e-4) << "k " << k << " P" << i;
        }
      }
      ++si;
    }
  }
  EXPECT_EQ(si, steps.size());
}

// ---- MS-P8 / MS-P9a-n: end-to-end pushFrame (944) ----
struct Accept {
  int k = -1;
  float scale = 0, residual = 0, sum = 0;
};

Accept RunScenario(const wp4_json::Value& in) {
  const wp4_json::Value& cfg = in["cfg"];
  config::RegisterDefaults();
  config::ApplyParamOverrides({
      {"MetricScaleEstimator.scaleEstimationMethod", static_cast<int>(cfg["method"].ll())},
      {"MetricScaleEstimator.imuGaussianSigma", cfg["sigma"].f()},
      {"MetricScaleEstimator.activeThresholdCheck", cfg["activeCheck"].ll() != 0},
      {"MetricScaleEstimator.sumCheck", cfg["sumCheck"].ll() != 0},
      {"MetricScaleEstimator.temporalAlignment", cfg["temporalAlignment"].ll() != 0},
  });
  const double s_true = in["sTrue"].d();
  const double off = in["imuOffsetSec"].d();
  const double start = in["imuStartSec"].d();
  const int frames = static_cast<int>(in["frames"].ll());
  auto p = [](double t) {
    return std::vector<double>{std::sin(1.3 * t) * 0.8, 0.3 * std::sin(2.1 * t + 0.4), std::cos(0.9 * t) * 0.6};
  };
  auto pdd = [](double t) {
    return std::vector<double>{-1.69 * std::sin(1.3 * t) * 0.8, -4.41 * 0.3 * std::sin(2.1 * t + 0.4),
                               -0.81 * std::cos(0.9 * t) * 0.6};
  };
  auto th = [](double t) { return 0.3 * std::sin(0.7 * t); };
  auto thd = [](double t) { return 0.21 * std::cos(0.7 * t); };
  MetricScaleEstimator mse(1.0f);
  const int64_t base = 5000000000LL;
  int64_t imu_k = static_cast<int64_t>(JsRound(start * 60));
  Accept acc;
  for (int k = 0; k < frames; ++k) {
    const double tf = k / 30.0;
    TrackingSensorFrame f;
    while (imu_k / 60.0 <= tf + 1e-9 && f.sensor_events.size() < 63) {
      const double ti = imu_k / 60.0;
      const std::vector<float> rr = RotY(th(ti));
      std::vector<double> am = pdd(ti);
      for (double& v : am) v *= s_true;
      const int64_t tns = base + static_cast<int64_t>(JsRound((ti + off) * 1e9));
      ImuEvent lin{ImuEvent::Kind::kLinearAcceleration, tns, 16666667, 0, 0, 0};
      float* dst[3] = {&lin.x, &lin.y, &lin.z};
      for (int c = 0; c < 3; ++c) {
        *dst[c] = static_cast<float>(rr[c] * am[0] + rr[3 + c] * am[1] + rr[6 + c] * am[2]);
      }
      f.sensor_events.push_back(lin);
      f.sensor_events.push_back(ImuEvent{ImuEvent::Kind::kGyroscope, tns, 16666667, 0.f,
                                         static_cast<float>(thd(ti) * 180 / M_PI), 0.f});
      ++imu_k;
    }
    const std::vector<double> pp = p(tf);
    const float tr[3] = {static_cast<float>(pp[0]), static_cast<float>(pp[1]), static_cast<float>(pp[2])};
    const int64_t tns = base + static_cast<int64_t>(JsRound(tf * 1e9));
    const bool ret = mse.PushFrame(f, PutPose(RotY(th(tf)), tr), tns, 1, false);
    if (ret && acc.k < 0) {
      acc.k = k;
      acc.scale = mse.scale();
      acc.residual = mse.residual();
      acc.sum = MetricScaleEstimatorAccess::state(mse).acc_sum;
    }
  }
  return acc;
}

class PushFrameGolden : public ::testing::TestWithParam<std::string> {};

TEST_P(PushFrameGolden, FirstAcceptMatchesOriginal) {
  const wp4_json::Value& r = Results()[GetParam()];
  const Accept mine = RunScenario(r["inputs"]);
  const wp4_json::Value& g = r["outputs"]["firstAccept"];
  if (g.is_null()) {
    EXPECT_EQ(mine.k, -1) << "original never accepts";
    return;
  }
  EXPECT_EQ(mine.k, g["k"].ll());
  EXPECT_LE(Rel(mine.scale, g["scale"].d()), 1e-3) << mine.scale << " vs " << g["scale"].d();
  if (g["residual"].d() >= 0) {
    EXPECT_LE(Rel(mine.residual, g["residual"].d()), 1e-2) << mine.residual << " vs " << g["residual"].d();
  }
  EXPECT_LE(Rel(mine.sum, g["sum"].d()), 1e-6) << mine.sum << " vs " << g["sum"].d();
}

INSTANTIATE_TEST_SUITE_P(
    P8P9, PushFrameGolden,
    ::testing::Values("push_frame_944", "push_frame_944_p9a_offset83ms_s1p7",
                      "push_frame_944_p9b_offset_minus120ms", "push_frame_944_p9c_method2_l2",
                      "push_frame_944_p9d_method3_frequency", "push_frame_944_p9e_method4_spatial",
                      "push_frame_944_p9f_method5_fixed_spatial", "push_frame_944_p9g_sigma1",
                      "push_frame_944_p9h_active_check", "push_frame_944_p9i_no_temporal_alignment_offset83ms",
                      "push_frame_944_p9j_scale_out_of_bounds_s3", "push_frame_944_p9k_preroll200ms",
                      "push_frame_944_p9l_preroll200ms_method3", "push_frame_944_p9m_active_check_s1p7",
                      "push_frame_944_p9n_preroll450ms_s1p7"),
    [](const ::testing::TestParamInfo<std::string>& info) { return info.param; });

// ---- MSE precision (WP5b MS-RT): bit-exact against the original 944 ----

uint32_t Bits(float v) {
  uint32_t u;
  std::memcpy(&u, &v, 4);
  return u;
}

float FromHex(const std::string& s, size_t at) {
  const uint32_t u = static_cast<uint32_t>(std::stoul(s.substr(at, 8), nullptr, 16));
  float v;
  std::memcpy(&v, &u, 4);
  return v;
}

std::string Hex32(uint32_t u) {
  char buf[9];
  std::snprintf(buf, sizeof(buf), "%08x", u);
  return buf;
}

// mse_precision_golden.mjs fnv: FNV-1a 64 over the little-endian bytes of n floats.
std::string Fnv(const float* p, int n) {
  uint64_t h = 0xcbf29ce484222325ULL;
  const unsigned char* b = reinterpret_cast<const unsigned char*>(p);
  for (int i = 0; i < 4 * n; ++i) {
    h ^= b[i];
    h *= 0x100000001b3ULL;
  }
  char buf[17];
  std::snprintf(buf, sizeof(buf), "%016llx", static_cast<unsigned long long>(h));
  return buf;
}

// Recorded 944 inputs (engine run flip6-s2-absolute, core 2, up to the latch + 3) replayed through
// the ORIGINAL 944 by slam_refact/tools/harness/mse_precision_golden.mjs. Every call must give the
// original's return value, scale and residual bits and the same bits of both Kalman states. This
// pins the original's float order of the 9-state filter (1169), the RTS smoother and LU inverse of
// 944, and the estimate chain behind the latched scale.
TEST(MsePrecision, OriginalPushFrameBitsOnRecordedInputs) {
  config::RegisterDefaults();
  const wp4_json::Value root = wp4_json::Load(
      std::string(SR_TEST_DATA_DIR) + "/metric-scale/mse-precision/flip6-s2-absolute-core2.json");
  const wp4_json::Value& calls = root["calls"];
  ASSERT_GT(calls.size(), 200u);
  MetricScaleEstimator mse(config::MetricScaleEstimatorSettings::Get().movement_check_period_in_sec);
  const auto& s = MetricScaleEstimatorAccess::state(mse);
  int accepted = 0;
  for (size_t c = 0; c < calls.size(); ++c) {
    const wp4_json::Value& in = calls[c]["in"];
    const wp4_json::Value& out = calls[c]["out"];
    TrackingSensorFrame f;
    for (size_t e = 0; e < in["events"].size(); ++e) {
      const wp4_json::Value& ev = in["events"][e];
      ImuEvent x;
      x.kind = static_cast<ImuEvent::Kind>(ev[0].ll());
      x.event_time_ns = std::stoll(ev[1].str);
      x.interval_ns = std::stoll(ev[2].str);
      x.x = FromHex(ev[3].str, 0);
      x.y = FromHex(ev[3].str, 8);
      x.z = FromHex(ev[3].str, 16);
      f.sensor_events.push_back(x);
    }
    float m[16];
    float inv[16];
    for (int k = 0; k < 16; ++k) {
      m[k] = FromHex(in["pose"].str, 8 * k);
      inv[k] = FromHex(in["inverse"].str, 8 * k);
    }
    const bool ret = mse.PushFrame(f, c8::HMatrix(m, inv), std::stoll(in["timeNs"].str),
                                   static_cast<int>(in["state"].ll()), in["reset"].ll() != 0);
    EXPECT_EQ(ret ? 1 : 0, out["ret"].ll()) << "call " << c;
    EXPECT_EQ(Hex32(Bits(mse.scale())), out["scale"].str) << "call " << c;
    EXPECT_EQ(Hex32(Bits(mse.residual())), out["residual"].str) << "call " << c;
    EXPECT_EQ(Fnv(s.kf.x.data(), 9) + Fnv(s.kf.p.data(), 81), out["kf"].str) << "call " << c;
    EXPECT_EQ(Fnv(s.imu.x.data(), 6) + Fnv(s.imu.p.data(), 36), out["imu"].str) << "call " << c;
    accepted += ret ? 1 : 0;
  }
  EXPECT_GT(accepted, 0);
}

// func 1236 (gebp): the four-column path sums even and odd k of the peeled depth separately
// (odd + even), the cols % 4 tail sums in k order. With p = (1, 2^24, 1, -2^24, 0...) the two
// orders give 2 and 0.
TEST(MsePrecision, GebpEvenOddOrder) {
  const float lhs[9] = {1.f, 16777216.f, 1.f, -16777216.f, 0.f, 0.f, 0.f, 0.f, 0.f};
  float res[5] = {0.f, 0.f, 0.f, 0.f, 0.f};
  fixed_products::Gebp(1, 5, 9, [&](int, int k) { return lhs[k]; }, [](int, int) { return 1.f; }, 1.f,
                       res, 1);
  for (int j = 0; j < 4; ++j) {
    EXPECT_EQ(res[j], 2.f) << j;
  }
  EXPECT_EQ(res[4], 0.f);
  // Depth < 8: every column sums in k order.
  float res2[4] = {0.f, 0.f, 0.f, 0.f};
  fixed_products::Gebp(1, 4, 4, [&](int, int k) { return lhs[k]; }, [](int, int) { return 1.f; }, 1.f,
                       res2, 1);
  EXPECT_EQ(res2[0], 0.f);
}

// Eigen's unrolled redux (1169 innovation, 1205): halves, TreeSum(4) = (f0 + f1) + (f2 + f3).
TEST(MsePrecision, TreeSumHalvingOrder) {
  const float v[4] = {1.f, 16777216.f, 1.f, -16777216.f};
  EXPECT_EQ(fixed_products::TreeSum(0, 4, [&](int k) { return v[k]; }), 1.f);  // in order: 0
  const float w[3] = {16777216.f, 1.f, 1.f};  // f0 + (f1 + f2)
  EXPECT_EQ(fixed_products::TreeSum(0, 3, [&](int k) { return w[k]; }), 16777218.f);
}

// Reset (1163) keeps the time bases, windows and min |a|; the first call always resets.
TEST(MetricScale, FirstCallResetsAndKeepsTimeBases) {
  config::RegisterDefaults();
  MetricScaleEstimator mse(1.0f);
  TrackingSensorFrame f;
  f.sensor_events.push_back(ImuEvent{ImuEvent::Kind::kLinearAcceleration, 2000, 0, 1.f, 0.f, 0.f});
  const float t[3] = {0.f, 0.f, 0.f};
  EXPECT_FALSE(mse.PushFrame(f, PutPose(RotY(0), t), 1000, 1, false));
  const auto& s = MetricScaleEstimatorAccess::state(mse);
  EXPECT_EQ(s.imu_time_base_ns, 2000);
  EXPECT_EQ(s.frame_time_base_ns, 1000);
  EXPECT_EQ(s.first_event_ever_ns, 2000);
  EXPECT_EQ(s.min_lin_acc, 1.f);
  EXPECT_EQ(s.kf.last_t, -1.f);
  EXPECT_EQ(mse.scale(), 1.f);
  // LOST resets every frame.
  EXPECT_FALSE(mse.PushFrame(f, PutPose(RotY(0), t), 2000, 0, false));
  EXPECT_EQ(s.kf.last_t, -1.f);
}

}  // namespace
}  // namespace slam_refact::metric_scale

// US-026 S10 unit golden comparisons, gate us026-mse-unit-gate.v2 (slam_refact/docs/design/us026-design.md §4.4 G-MSE-U,
// §4.5 MS-P10 / MS-P11). This file is the v2 copy of us026_method_precision_test.cc (v1, unchanged and still pinned by
// the v1 manifest). The ONLY judging difference to v1 is the scale rule of the method-4 re-estimation variant (gate file
// us026-mse-unit-gate.v2.md §3.1): per call, |a - g| <= 2 * S_c * |g| (or within 3 ulp, the U3 floor that keeps the protocol's 3x
// resolution rule) where S_c is the largest relative scale change over 400 fixed-seed element-wise 2^-24 perturbations of the
// engine's own normal equation (ata, atb) of that call. Everything
// else (U3 3 ulp, R34 1e-6 for method 3 and for method 4 without re-estimation, integers exact, MS-P10/P11, GATE and
// REPORT grades) is the v1 code. The engine's normal equation is obtained WITHOUT touching slam_refact/engine/src: the
// test binary is linked with -Wl,--wrap on ScaleSpatial (tools/gates/us026-mse-unit-v2/cmake/CMakeLists.txt), the wrapper
// calls the real function and keeps a copy of its four input arrays; ata / atb are recomputed from them with the same
// source loop and the same compile flags, and the replica is accepted only if its scale has the same bits as the engine's
// (otherwise the call is a violation, never a silent pass).
// Expected values come from the ORIGINAL xr-slam.wasm run in isolation:
//   *.methods.json  slam_refact/tools/harness/us026/mse_method_golden.mjs: func 944 per call on
//                   recorded 944 inputs, once per scaleEstimationMethod (1-5, M@D = device model D).
//   *.kf6.json      slam_refact/tools/re/spec_metric_scale/mse-probe-kf6.mjs: func 1170 alone
//                   (6-state angular KF), the 1169 iOS fps-scaling pair, func 1178 (6x6 RTS).
//
// Two executables are built from this file (tools/gates/us026-mse-unit-v2/cmake/CMakeLists.txt):
//   sr_tests_us026_method_precision_v2        label us026 (dev): judges the goldens in $SR_US026_DATA_DIR (default: the
//                                          checked-in dev goldens; the dev reruns name a goldens directory), plus the gate's own null-run /
//                                          comparator / sensitivity / release-order tests (Us026Null.*, Us026Comparator.*,
//                                          Us026Sens.*, Us026Integrity.*; synthetic files).
//   sr_tests_us026_method_precision_v2_held   label held (SR_US026_HELD): the gate us026-mse-unit-gate.v2. Fails
//                                          closed (HELD) before the first comparison unless StartRelease passes:
//                                          the shared device sr_held (tests/support/sr_held.h; kind "judge"),
//                                          the golden step's one-run marker (kind "golden", written by
//                                          tools/gates/us026-mse-unit-v2/golden.mjs) and the provenance of every
//                                          golden in $SR_US026_DATA_DIR against the manifest "registration" block.
// Both judge with the SAME code (JudgeRun and the MS-P10 / MS-P11 tests below); the tolerances are the ones the
// gate file us026-mse-unit-gate.v2.md fixes (the method-4 re-estimation rule was chosen AFTER the v1 result, gate file
// section 0). Every judged test prints one machine-readable "US026-REPORT {...}" line (max ulp distance, exact-bit
// counts, first differing call) and, for every re-estimation call whose scale bits differ, one "US026-SENS {...}" line.
//
// A methods golden may carry "cpp_param_overrides" ([key, type, value]): parameters applied to the C++ side only.
// Real goldens never have it; mutation copies use it and the test prints a US026-MUTATION line. The held gate refuses
// such a golden before its marker (StartRelease), and the runner refuses an output that contains that line.

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <filesystem>
#include <fstream>
#include <functional>
#include <limits>
#include <map>
#include <memory>
#include <set>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include <sys/stat.h>
#include <unistd.h>

#include <Eigen/Dense>
#include <gtest/gtest.h>

#include "../support/sr_held.h"
#include "c8/hmatrix.h"
#include "config/params.h"
#include "config/settings.h"
#include "metric_scale/alignment_math.h"
#include "metric_scale/kalman.h"
#include "metric_scale/metric_scale_estimator.h"
#include "metric_scale/metric_scale_estimator_internal.h"
#include "wp4_json.h"

namespace slam_refact::metric_scale {

// ---- link-time interposition of ScaleSpatial (gate file §2.4); the engine sources are not modified ----
// The test target is linked with -Wl,--wrap=<mangled ScaleSpatial>: every reference to ScaleSpatial from another object
// (EstimateScale in metric_scale_estimator.cc) resolves to WrappedScaleSpatial, which calls the real function and keeps a
// copy of its four input arrays and its result. A call that never reaches the wrapper leaves calls == 0 (the judge turns
// that into a violation).
#define SR_US026_SS_MANGLED \
  "_ZN11slam_refact12metric_scale12ScaleSpatialERKSt6vectorISt5arrayIfLm3EESaIS3_EES7_S7_S7_"
ScaleFit RealScaleSpatial(const std::vector<Vec3>& v, const std::vector<Vec3>& imu, const std::vector<Vec3>& omega,
                          const std::vector<Vec3>& omega_dot) __asm__("__real_" SR_US026_SS_MANGLED);
ScaleFit WrappedScaleSpatial(const std::vector<Vec3>& v, const std::vector<Vec3>& imu, const std::vector<Vec3>& omega,
                             const std::vector<Vec3>& omega_dot) __asm__("__wrap_" SR_US026_SS_MANGLED);

struct SpatialCapture {
  int64_t calls = 0;  // wrapper invocations since process start
  std::vector<Vec3> v, imu, omega, omega_dot;
  ScaleFit real;
};

SpatialCapture& Capture() {
  static SpatialCapture c;
  return c;
}

ScaleFit WrappedScaleSpatial(const std::vector<Vec3>& v, const std::vector<Vec3>& imu, const std::vector<Vec3>& omega,
                             const std::vector<Vec3>& omega_dot) {
  const ScaleFit r = RealScaleSpatial(v, imu, omega, omega_dot);
  SpatialCapture& c = Capture();
  ++c.calls;
  c.v = v;
  c.imu = imu;
  c.omega = omega;
  c.omega_dot = omega_dot;
  c.real = r;
  return r;
}

namespace {

namespace sh = slam_refact::sr_held;

// ---- gate tolerances (us026-mse-unit-gate.v2.md §3; the v1 values, plus the sensitivity rule below) ----
// Element-wise: |a - g| <= kUlps * ulp32(max(|a|, |g|)) (the G-BA-S form of us026-design.md §3.8).
constexpr int kUlps = 3;
// Methods 3 (FFT) and 4 (4x4 LDLT): scale and residual relative difference <= 1e-6 (design §4.2 candidate,
// chosen before the dev results; float32 relative resolution 2^-23 = 1.19e-7, ratio 8.39).
constexpr double kRelMethods34 = 1e-6;
// Exact: ret, have_estimate, first accepted call, call count, deque sizes, selected times (T_i, T_g), A_i, last_t,
// interval. Method 5 with a device model inside the lever-arm table (M@D, D != 0) is REPORT, not GATE.

std::string DataDir() {
  const char* env = std::getenv("SR_US026_DATA_DIR");
  if (env != nullptr && env[0] != '\0') {
    return env;
  }
#ifdef SR_US026_HELD
  return "";  // the gate judges only the golden directory the runner names
#else
  return std::string(SR_TEST_DATA_DIR) + "/metric-scale/us026";
#endif
}

std::vector<std::string> Files(const std::string& suffix) {
  std::vector<std::string> out;
  if (DataDir().empty()) {
    return out;
  }
  std::error_code ec;
  for (const auto& e : std::filesystem::directory_iterator(DataDir(), ec)) {
    const std::string name = e.path().filename().string();
    if (name.size() > suffix.size() && name.compare(name.size() - suffix.size(), suffix.size(), suffix) == 0) {
      out.push_back(name);
    }
  }
  std::sort(out.begin(), out.end());
  return out;
}

// Parsed goldens by full path. StartRelease replaces an entry with the parse of the bytes it hashed.
std::map<std::string, std::unique_ptr<wp4_json::Value>>& Cache() {
  static std::map<std::string, std::unique_ptr<wp4_json::Value>> cache;
  return cache;
}

const wp4_json::Value& Load(const std::string& name) {
  const std::string path = DataDir() + "/" + name;
  auto& slot = Cache()[path];
  if (!slot) {
    slot = std::make_unique<wp4_json::Value>(wp4_json::Load(path));
  }
  return *slot;
}

uint32_t Bits(float v) {
  uint32_t u;
  std::memcpy(&u, &v, 4);
  return u;
}

float FromBits(uint32_t u) {
  float v;
  std::memcpy(&v, &u, 4);
  return v;
}

float FromHex(const std::string& s, size_t at) {
  return FromBits(static_cast<uint32_t>(std::stoul(s.substr(at, 8), nullptr, 16)));
}

std::vector<float> HexFloats(const std::string& s) {
  std::vector<float> out;
  for (size_t at = 0; at + 8 <= s.size(); at += 8) {
    out.push_back(FromHex(s, at));
  }
  return out;
}

double Ulp32(double x) {
  const float a = static_cast<float>(std::fabs(x));
  return static_cast<double>(std::nextafter(a, std::numeric_limits<float>::infinity()) - a);
}

// Ordered-integer distance (reporting): 0 = same bits; NaN vs NaN with the same bits = 0.
int64_t UlpDistance(float a, float b) {
  auto ord = [](float v) {
    const int64_t i = static_cast<int32_t>(Bits(v));
    return i < 0 ? static_cast<int64_t>(INT32_MIN) - i : i;
  };
  if (Bits(a) == Bits(b)) {
    return 0;
  }
  if (std::isnan(a) || std::isnan(b)) {
    return std::numeric_limits<int64_t>::max();
  }
  return std::llabs(ord(a) - ord(b));
}

bool WithinUlps(float a, float g) {
  if (Bits(a) == Bits(g)) {
    return true;
  }
  if (std::isnan(a) || std::isnan(g)) {
    return false;
  }
  const double m = std::max(std::fabs(static_cast<double>(a)), std::fabs(static_cast<double>(g)));
  return std::fabs(static_cast<double>(a) - g) <= kUlps * Ulp32(m);
}

bool WithinRel(float a, float g) {
  if (Bits(a) == Bits(g)) {
    return true;
  }
  if (!std::isfinite(a) || !std::isfinite(g)) {
    return false;
  }
  return std::fabs(static_cast<double>(a) - g) <= kRelMethods34 * std::fabs(static_cast<double>(g));
}

// ---- method-4 re-estimation scale: per-call sensitivity bound (us026-mse-unit-gate.v2.md §3.1, tier SENS) ----
// For the call's normal equation (ata, atb) of the ENGINE (float 4x4 / 4): x0 = A^-1 b in double, s0 = |x0[3]|. Trial t
// (400 trials, fixed seed): E = uniform(-2^-24, 2^-24) 4x4, E = (E + E^T)/2; A' = A .* (1 + E); b' = b .* (1 + e),
// e uniform(-2^-24, 2^-24)^4; s_t = |(A'^-1 b')[3]|; change_t = |s_t - s0| / s0. S_c = max_t change_t; the call's
// relative tolerance is kSensFactor * S_c; the scale is accepted when it is within 3 ulp (U3) OR within that relative
// tolerance (tier SENS = max(U3, 2 S_c)). (Same recipe as the diagnosis sens.py, numpy replaced by a self-contained
// generator so the result does not depend on a library's stream.)
constexpr double kSensEps = 5.9604644775390625e-08;  // 2^-24
constexpr int kSensTrials = 400;
constexpr double kSensFactor = 2.0;
constexpr uint64_t kSensSeed = 1;

struct SplitMix64 {
  uint64_t state;
  uint64_t Next() {
    uint64_t z = (state += 0x9E3779B97F4A7C15ULL);
    z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ULL;
    z = (z ^ (z >> 27)) * 0x94D049BB133111EBULL;
    return z ^ (z >> 31);
  }
  double Uniform() { return static_cast<double>(Next() >> 11) * (1.0 / 9007199254740992.0); }  // [0, 1)
  double Signed(double eps) { return eps * (2.0 * Uniform() - 1.0); }                            // [-eps, eps)
};

struct SensResult {
  double s0 = 0.0;          // |x0[3]| of the unperturbed double solve
  double max_change = 0.0;  // S_c
  double bound = 0.0;       // kSensFactor * S_c (0 = exact bits only when degenerate)
  double cond = 0.0;        // 2-norm condition number of ata in double (report only)
  bool degenerate = false;  // s0 is zero / not finite: the bound is undefined and the call fails closed (bound 0)
};

// call_index makes the stream of one call independent of the other calls (and of the run order).
SensResult SensitivityBound(const Eigen::Matrix4f& ata, const Eigen::Vector4f& atb, int64_t call_index) {
  SensResult r;
  const Eigen::Matrix4d a = ata.cast<double>();
  const Eigen::Vector4d b = atb.cast<double>();
  const Eigen::PartialPivLU<Eigen::Matrix4d> lu(a);
  const Eigen::Vector4d x0 = lu.solve(b);
  r.s0 = std::fabs(x0(3));
  if (!(r.s0 > 0.0) || !std::isfinite(r.s0)) {
    r.degenerate = true;
    return r;
  }
  const Eigen::JacobiSVD<Eigen::Matrix4d> svd(a);
  const Eigen::Vector4d sv = svd.singularValues();
  r.cond = sv(3) > 0.0 ? sv(0) / sv(3) : std::numeric_limits<double>::infinity();
  SplitMix64 rng{kSensSeed * 0x9E3779B97F4A7C15ULL + static_cast<uint64_t>(call_index + 1) * 0xBF58476D1CE4E5B9ULL};
  for (int t = 0; t < kSensTrials; ++t) {
    Eigen::Matrix4d e;
    for (int i = 0; i < 4; ++i) {
      for (int j = 0; j < 4; ++j) e(i, j) = rng.Signed(kSensEps);
    }
    e = ((e + e.transpose()) * 0.5).eval();
    Eigen::Vector4d be;
    for (int i = 0; i < 4; ++i) be(i) = rng.Signed(kSensEps);
    const Eigen::Matrix4d ap = a.cwiseProduct(Eigen::Matrix4d::Ones() + e);
    const Eigen::Vector4d bp = b.cwiseProduct(Eigen::Vector4d::Ones() + be);
    const Eigen::Vector4d xp = Eigen::PartialPivLU<Eigen::Matrix4d>(ap).solve(bp);
    const double change = std::fabs(std::fabs(xp(3)) - r.s0) / r.s0;
    r.max_change = std::max(r.max_change, change);
  }
  r.bound = kSensFactor * r.max_change;
  return r;
}

// The engine's ScaleSpatial (alignment_math.cc 436-459) re-run on a captured call: same loop, same types, same flags.
// Returns the scale it produces; ata / atb are the float normal equation that scale came from.
Eigen::Matrix3f SkewOf(const Vec3& w) {
  Eigen::Matrix3f m;
  m << 0.f, -w[2], w[1], w[2], 0.f, -w[0], -w[1], w[0], 0.f;
  return m;
}

float ReplicaScaleSpatial(const SpatialCapture& c, Eigen::Matrix4f* ata_out, Eigen::Vector4f* atb_out) {
  Eigen::Matrix4f ata = Eigen::Matrix4f::Zero();
  Eigen::Vector4f atb = Eigen::Vector4f::Zero();
  const size_t n = c.v.size();
  for (size_t k = 0; k < n; ++k) {
    const Vec3 w = k < c.omega.size() ? c.omega[k] : Vec3{0.f, 0.f, 0.f};
    const Vec3 al = k < c.omega_dot.size() ? c.omega_dot[k] : Vec3{0.f, 0.f, 0.f};
    const Eigen::Matrix3f wx = SkewOf(w);
    const Eigen::Matrix3f m = wx * wx + SkewOf(al);
    Eigen::Matrix<float, 3, 4> row;
    row.leftCols<3>() = m;
    row.col(3) << c.v[k][0], c.v[k][1], c.v[k][2];
    const Eigen::Vector3f b(c.imu[k][0], c.imu[k][1], c.imu[k][2]);
    ata.noalias() += row.transpose() * row;
    atb.noalias() += row.transpose() * b;
  }
  const Eigen::Vector4f x = Eigen::LDLT<Eigen::Matrix4f>(ata).solve(atb);
  *ata_out = ata;
  *atb_out = atb;
  return std::fabs(x(3));
}

// Per-call relative scale tolerance of one re-estimation run (index = call). rel < 0: no sensitivity bound for this call
// (the v1 rule R34 applies); defect != "": the call cannot be judged and is a violation (gate file §3.1 rules S1-S4).
struct ScaleBounds {
  std::vector<double> rel;
  std::vector<std::string> defect;
  std::vector<double> cond;
};

bool WithinRelTol(float a, float g, double tol) {
  if (Bits(a) == Bits(g)) {
    return true;
  }
  if (!std::isfinite(a) || !std::isfinite(g)) {
    return false;
  }
  return std::fabs(static_cast<double>(a) - g) <= tol * std::fabs(static_cast<double>(g));
}

// Agreement statistics of one quantity (printed in the REPORT line).
struct Agree {
  int64_t n = 0;
  int64_t exact = 0;
  int64_t max_ulp = 0;
  double max_rel = 0.0;
  int64_t out_of_tol = 0;
  int64_t first_diff = -1;  // index of the first non-identical comparison (call / step)

  void Add(float mine, float golden, int64_t at, bool in_tol) {
    ++n;
    const int64_t d = UlpDistance(mine, golden);
    if (d == 0) {
      ++exact;
    } else if (first_diff < 0) {
      first_diff = at;
    }
    max_ulp = std::max(max_ulp, d);
    if (golden != 0 && std::isfinite(mine) && std::isfinite(golden)) {
      max_rel = std::max(max_rel, std::fabs(static_cast<double>(mine) - golden) / std::fabs(static_cast<double>(golden)));
    }
    if (!in_tol) {
      ++out_of_tol;
    }
  }
  std::string Json() const {
    char buf[200];
    std::snprintf(buf, sizeof(buf),
                  "{\"n\":%lld,\"exact\":%lld,\"max_ulp\":%lld,\"max_rel\":%.3g,\"out_of_tol\":%lld,\"first_diff\":%lld}",
                  static_cast<long long>(n), static_cast<long long>(exact), static_cast<long long>(max_ulp), max_rel,
                  static_cast<long long>(out_of_tol), static_cast<long long>(first_diff));
    return buf;
  }
};

// Compares a vector element-wise under the ulp tolerance; reports the first few failures.
void CompareUlps(const float* mine, const std::vector<float>& golden, int64_t at, const char* what, Agree* agree,
                 int* reported) {
  for (size_t i = 0; i < golden.size(); ++i) {
    const bool ok = WithinUlps(mine[i], golden[i]);
    agree->Add(mine[i], golden[i], at, ok);
    if (!ok && (*reported)++ < 12) {
      ADD_FAILURE() << what << " at " << at << " element " << i << ": " << mine[i] << " vs " << golden[i]
                    << " (ulp distance " << UlpDistance(mine[i], golden[i]) << ")";
    }
  }
}

std::vector<TrackingSensorEvent> Events(const wp4_json::Value& evs) {
  std::vector<TrackingSensorEvent> out;
  for (size_t e = 0; e < evs.size(); ++e) {
    const wp4_json::Value& ev = evs[e];
    ImuEvent x;
    x.kind = static_cast<ImuEvent::Kind>(ev[0].ll());
    x.event_time_ns = std::stoll(ev[1].str);
    x.interval_ns = std::stoll(ev[2].str);
    x.x = FromHex(ev[3].str, 0);
    x.y = FromHex(ev[3].str, 8);
    x.z = FromHex(ev[3].str, 16);
    out.push_back(x);
  }
  return out;
}

c8::HMatrix Pose(const wp4_json::Value& in) {
  float m[16];
  float inv[16];
  for (int k = 0; k < 16; ++k) {
    m[k] = FromHex(in["pose"].str, 8 * k);
    inv[k] = FromHex(in["inverse"].str, 8 * k);
  }
  return c8::HMatrix(m, inv);
}

// ======================================================================= release (protocol §12.5, §12.6)
// Start of the first real run of the gate. The release decision and the one-run markers are the shared device sr_held;
// the gate is designated by the caller (SR_US026_GATE=<gate_id>.v<N>), this source names no gate id or version. Order
// (every check that can refuse the run comes BEFORE the judge marker, so a refusal never uses up the one judge run):
//   1. sr_held::CheckRelease (kind "judge"): env value = sha256 of the registered designation file, manifest, review,
//      designation pins, held.pins (this executable and every other file the runner presents), coordinator seed policy
//   2. the golden step ran under the device: its one-run marker (kind "golden") exists and names the same designation
//   3. the golden directory holds exactly the goldens of the manifest "registration" block plus GOLDEN-SHA256SUMS, each
//      golden has the sha256 the golden step listed there, and its provenance fields (original wasm, generator script,
//      dump sha256 and core, method runs, estimate_after_have, poke kept, seed class and seed file sha256) equal the
//      registration block; a golden with cpp_param_overrides (a mutation copy) is refused
//   4. sr_held::HeldGate (kind "judge"): the same device checks again, then the O_EXCL marker; the code never removes it
// The goldens are parsed from the bytes that were hashed (no second read).

bool SplitGateName(const std::string& name, std::string* gate_id, int* version) {
  const size_t p = name.rfind(".v");
  if (p == std::string::npos || p == 0 || p + 2 >= name.size()) return false;
  const std::string digits = name.substr(p + 2);
  if (digits.size() > 6 || digits[0] == '0' ||
      !std::all_of(digits.begin(), digits.end(), [](char c) { return c >= '0' && c <= '9'; })) {
    return false;
  }
  *gate_id = name.substr(0, p);
  *version = std::atoi(digits.c_str());
  return sh::IsId(*gate_id);
}

struct ReleaseInputs {
  std::string env_value;          // SR_US026_DESIGNATION
  std::string prereg_dir;         // manifest, designation, review, gate file, seed file, markers, inputs
  std::string gate_name;          // "<gate_id>.v<N>", SR_US026_GATE
  std::vector<std::string> pins;  // this executable + SR_US026_PINS (exactly the files of manifest held.pins)
  std::string golden_dir;         // SR_US026_DATA_DIR (written by the golden step of the same run)
};

struct ReleaseResult {
  bool ok = false;
  std::string code;              // "" when ok; an sr_held refusal code, "ARGS", "REGISTRATION" or "GOLDEN"
  std::string reason;            // first failed check
  std::vector<std::string> log;  // every check performed, for the head of run.log
};

const sh::Json* Field(const sh::Json* o, const char* key, sh::Json::Type type) { return sh::internal::Member(o, key, type); }

bool WpString(const wp4_json::Value& v, const char* key, std::string* out) {
  const auto it = v.obj.find(key);
  if (it == v.obj.end() || it->second.type != wp4_json::Value::Type::kString) return false;
  *out = it->second.str;
  return true;
}

bool WpTrue(const wp4_json::Value& v, const char* key) {
  const auto it = v.obj.find(key);
  return it != v.obj.end() && it->second.type == wp4_json::Value::Type::kBool && it->second.b;
}

ReleaseResult StartRelease(const ReleaseInputs& in) {
  ReleaseResult r;
  auto fail = [&r](const std::string& code, const std::string& why) {
    r.ok = false;
    r.code = code;
    r.reason = why;
    r.log.push_back("[US026-INTEGRITY] FAIL " + code + ": " + why);
    return r;
  };
  auto note = [&r](const std::string& line) { r.log.push_back("[US026-INTEGRITY] " + line); };
  sh::Request rq;
  rq.prereg_dir = in.prereg_dir;
  rq.kind = "judge";
  rq.env_value = in.env_value;
  rq.pins = in.pins;
  rq.want_seed = true;  // manifest held.seed (the coordinator seed of the MS-P10 / MS-P11 golden)
  if (!SplitGateName(in.gate_name, &rq.gate_id, &rq.version)) {
    return fail("ARGS", "HELD: the gate is not designated as <gate_id>.v<N> (SR_US026_GATE='" + in.gate_name + "')");
  }
  if (in.golden_dir.empty() || !std::filesystem::is_directory(in.golden_dir)) {
    return fail("ARGS", "HELD: SR_US026_DATA_DIR is not a directory ('" + in.golden_dir + "')");
  }
  const sh::Outcome check = sh::CheckRelease(rq);
  for (const std::string& l : check.log) r.log.push_back(l);
  if (!check.ok) return fail(check.code, check.why);

  // The manifest registration block (strict parse; the manifest is pinned by the designation, checked above).
  std::string mtext, err;
  sh::Json m;
  if (!sh::ReadFile(rq.prereg_dir + "/" + sh::ManifestName(rq), &mtext) || !sh::ParseJson(mtext, &m, &err)) {
    return fail("REGISTRATION", "manifest unreadable after the release check: " + err);
  }
  const sh::Json* reg = Field(&m, "registration", sh::Json::Type::kObject);
  const sh::Json* wasm = Field(reg, "original_wasm_sha256", sh::Json::Type::kString);
  const sh::Json* tools = Field(reg, "tool_sha256", sh::Json::Type::kObject);
  const sh::Json* runs = Field(reg, "methods_runs", sh::Json::Type::kArray);
  const sh::Json* goldens = Field(reg, "goldens", sh::Json::Type::kArray);
  if (wasm == nullptr || tools == nullptr || runs == nullptr || goldens == nullptr || goldens->items.empty()) {
    return fail("REGISTRATION", "manifest registration block lacks original_wasm_sha256 / tool_sha256 / methods_runs / goldens");
  }
  std::set<std::string> want_runs;
  for (const sh::Json& x : runs->items) {
    if (x.type != sh::Json::Type::kString) return fail("REGISTRATION", "registration.methods_runs has a non-string entry");
    want_runs.insert(x.text);
  }

  // 2. the golden step's marker
  const std::string golden_marker = rq.prereg_dir + "/" + rq.gate_id + ".v" + std::to_string(rq.version) + ".first-run.golden.marker";
  struct stat st;
  std::string marker_text;
  sh::Json marker;
  if (::lstat(golden_marker.c_str(), &st) != 0 || !S_ISREG(st.st_mode) || !sh::ReadFile(golden_marker, &marker_text) ||
      !sh::ParseJson(marker_text, &marker, &err)) {
    return fail("GOLDEN", "the golden step did not run under the device: no readable marker " + golden_marker);
  }
  const sh::Json* mk_kind = Field(&marker, "kind", sh::Json::Type::kString);
  const sh::Json* mk_gate = Field(&marker, "gate", sh::Json::Type::kString);
  const sh::Json* mk_des = Field(&marker, "designation_sha256", sh::Json::Type::kString);
  if (mk_kind == nullptr || mk_kind->text != "golden" || mk_gate == nullptr || mk_gate->text != in.gate_name ||
      mk_des == nullptr || mk_des->text != check.designation_sha256) {
    return fail("GOLDEN", "the golden marker does not name this gate, kind golden and this designation");
  }
  note("golden marker " + golden_marker + " names designation " + mk_des->text);

  // 3. the golden directory: exactly the registered goldens + GOLDEN-SHA256SUMS
  std::set<std::string> want_files = {"GOLDEN-SHA256SUMS"};
  for (const sh::Json& g : goldens->items) {
    const sh::Json* f = Field(&g, "file", sh::Json::Type::kString);
    if (f == nullptr || !sh::IsBareName(f->text)) return fail("REGISTRATION", "registration.goldens has an entry without a bare file name");
    want_files.insert(f->text);
  }
  std::set<std::string> have_files;
  std::error_code ec;
  for (const auto& e : std::filesystem::directory_iterator(in.golden_dir, ec)) {
    if (!e.is_regular_file() || e.is_symlink()) return fail("GOLDEN", "golden directory holds a non-regular entry: " + e.path().string());
    have_files.insert(e.path().filename().string());
  }
  if (ec) return fail("GOLDEN", "golden directory unreadable: " + in.golden_dir);
  if (have_files != want_files) {
    std::string extra, missing;
    for (const std::string& f : have_files) if (!want_files.count(f)) extra += " " + f;
    for (const std::string& f : want_files) if (!have_files.count(f)) missing += " " + f;
    return fail("GOLDEN", "golden directory is not the registered set (extra:" + extra + "; missing:" + missing + ")");
  }
  std::string sums_text;
  if (!sh::ReadFile(in.golden_dir + "/GOLDEN-SHA256SUMS", &sums_text)) return fail("GOLDEN", "GOLDEN-SHA256SUMS unreadable");
  std::map<std::string, std::string> sums;
  {
    std::istringstream ss(sums_text);
    std::string line;
    while (std::getline(ss, line)) {
      if (line.size() < 67 || line.compare(64, 2, "  ") != 0 || !sh::IsHex64(line.substr(0, 64))) {
        return fail("GOLDEN", "GOLDEN-SHA256SUMS has a malformed line");
      }
      if (!sums.emplace(line.substr(66), line.substr(0, 64)).second) return fail("GOLDEN", "GOLDEN-SHA256SUMS lists a file twice");
    }
  }
  std::string seed_sha;
  if (!sh::Sha256File(rq.prereg_dir + "/" + sh::SeedFileName(rq), &seed_sha)) return fail("GOLDEN", "the coordinator seed file is unreadable");
  for (const sh::Json& g : goldens->items) {
    const std::string file = Field(&g, "file", sh::Json::Type::kString)->text;
    const sh::Json* kind = Field(&g, "kind", sh::Json::Type::kString);
    if (kind == nullptr || (kind->text != "methods" && kind->text != "kf6")) return fail("REGISTRATION", file + ": kind is not methods / kf6");
    const auto listed = sums.find(file);
    if (listed == sums.end()) return fail("GOLDEN", file + " is not listed in GOLDEN-SHA256SUMS");
    std::string bytes;
    if (!sh::ReadFile(in.golden_dir + "/" + file, &bytes)) return fail("GOLDEN", file + " unreadable");
    const std::string actual = testutil::Sha256Hex(bytes);
    note("golden " + file + " sha256 actual=" + actual + " listed=" + listed->second);
    if (actual != listed->second) return fail("GOLDEN", file + " differs from GOLDEN-SHA256SUMS");
    auto parsed = std::make_unique<wp4_json::Value>(wp4_json::Parser(bytes).Parse());
    const wp4_json::Value& v = *parsed;
    std::string s;
    if (!WpString(v, "original_sha256", &s) || s != wasm->text) return fail("GOLDEN", file + ": original_sha256 is not the registered wasm");
    if (!WpTrue(v, "code_bytes_unchanged")) return fail("GOLDEN", file + ": code_bytes_unchanged is not true");
    const sh::Json* tool = Field(tools, kind->text == "methods" ? "mse_method_golden.mjs" : "mse-probe-kf6.mjs", sh::Json::Type::kString);
    if (tool == nullptr || !WpString(v, "script_sha256", &s) || s != tool->text) return fail("GOLDEN", file + ": script_sha256 is not the registered generator");
    if (v.obj.count("cpp_param_overrides") != 0) return fail("GOLDEN", file + ": carries cpp_param_overrides (a mutation copy)");
    if (kind->text == "methods") {
      const sh::Json* dump_sha = Field(&g, "dump_sha256", sh::Json::Type::kString);
      const sh::Json* core = Field(&g, "core", sh::Json::Type::kNumber);
      const sh::Json* eah = Field(&g, "estimate_after_have", sh::Json::Type::kNumber);
      if (dump_sha == nullptr || core == nullptr || eah == nullptr) return fail("REGISTRATION", file + ": methods entry lacks dump_sha256 / core / estimate_after_have");
      if (!WpString(v, "schema", &s) || s != "slam-refact-us026-mse-method-golden-v1") return fail("GOLDEN", file + ": schema");
      if (!WpString(v, "dump_sha256", &s) || s != dump_sha->text) return fail("GOLDEN", file + ": dump_sha256 is not the registered dump");
      if (v.obj.count("core") == 0 || std::to_string(v["core"].ll()) != core->text) return fail("GOLDEN", file + ": core is not the registered core");
      if (v.obj.count("last_call") == 0 || !v["last_call"].is_null()) return fail("GOLDEN", file + ": last_call is set (a truncated golden)");
      if (v.obj.count("methods") == 0 || v.obj.count("inputs") == 0 || v.obj.count("states") == 0) return fail("GOLDEN", file + ": methods / inputs / states missing");
      std::set<std::string> have_runs;
      for (const auto& kv : v["methods"].obj) {
        have_runs.insert(kv.first);
        const wp4_json::Value& run = kv.second;
        if (run.obj.count("poke") == 0 || !WpTrue(run["poke"], "held")) return fail("GOLDEN", file + " run " + kv.first + ": the original did not keep the poked setting");
        const bool reest = run.obj.count("estimate_after_have") != 0 && run["estimate_after_have"].ll() == 1;
        if (reest != (eah->text == "1")) return fail("GOLDEN", file + " run " + kv.first + ": estimate_after_have is not the registered variant");
        if (run.obj.count("calls") == 0 || run["calls"].size() != v["inputs"].size()) return fail("GOLDEN", file + " run " + kv.first + ": call count differs from the inputs");
      }
      if (have_runs != want_runs) return fail("GOLDEN", file + ": method runs are not the registered set");
    } else {
      if (v.obj.count("seed") == 0 || !WpString(v["seed"], "class", &s) || s != "registration") return fail("GOLDEN", file + ": seed class is not registration");
      if (!WpString(v["seed"], "seed_file_sha256", &s) || s != seed_sha) return fail("GOLDEN", file + ": seed_file_sha256 is not the coordinator seed file");
      if (v.obj.count("results") == 0) return fail("GOLDEN", file + ": results missing");
    }
    Cache()[in.golden_dir + "/" + file] = std::move(parsed);  // the tests judge exactly the hashed bytes
  }
  note("golden directory = registered set (" + std::to_string(goldens->items.size()) + " goldens), provenance OK");

  // 4. the judge marker
  const sh::Outcome claimed = sh::HeldGate(rq);
  for (const std::string& l : claimed.log) {
    if (l.find("first-run marker created") != std::string::npos) r.log.push_back(l);
  }
  if (!claimed.ok) return fail(claimed.code, claimed.why);
  note("first-run marker created for " + in.gate_name + " kind judge: " + claimed.marker);
  r.ok = true;
  note("OK: release conditions hold");
  return r;
}

#ifdef SR_US026_HELD
ReleaseResult g_release;

class GateEnvironment : public ::testing::Environment {
 public:
  void SetUp() override {
    auto env = [](const char* name) {
      const char* v = std::getenv(name);
      return std::string(v == nullptr ? "" : v);
    };
    ReleaseInputs in;
    in.env_value = env("SR_US026_DESIGNATION");
    in.gate_name = env("SR_US026_GATE");
    in.prereg_dir = env("SR_US026_PREREG_DIR").empty() ? std::string(SR_US026_PREREG_DIR) : env("SR_US026_PREREG_DIR");
    in.golden_dir = env("SR_US026_DATA_DIR");
    in.pins = {sh::SelfExe()};
    std::stringstream pins(env("SR_US026_PINS"));
    std::string p;
    while (std::getline(pins, p, ':')) {
      if (!p.empty()) in.pins.push_back(p);
    }
    g_release = StartRelease(in);
    for (const std::string& line : g_release.log) std::printf("%s\n", line.c_str());
    std::fflush(stdout);
  }
};

const ::testing::Environment* const kGateEnvironment = ::testing::AddGlobalTestEnvironment(new GateEnvironment);

bool GateOpen() { return g_release.ok; }
std::string GateRefusal() { return "HELD [" + g_release.code + "]: " + g_release.reason; }

TEST(Us026Held, Release) { ASSERT_TRUE(g_release.ok) << GateRefusal(); }
#else
bool GateOpen() { return true; }
std::string GateRefusal() { return ""; }
#endif

// ======================================================================= G-MSE-U (944 per call)

struct MethodRun {
  std::string file;
  std::string run;  // "N" or "N@D"
};

std::vector<MethodRun> MethodRuns() {
  std::vector<MethodRun> out;
  for (const std::string& f : Files(".methods.json")) {
    for (const auto& kv : Load(f)["methods"].obj) {
      out.push_back({f, kv.first});
    }
  }
  return out;
}

void ApplyCppOverrides(const wp4_json::Value& root, std::vector<std::pair<std::string, ParamValue>>* out) {
  const auto it = root.obj.find("cpp_param_overrides");
  if (it == root.obj.end()) {
    return;
  }
  for (size_t k = 0; k < it->second.size(); ++k) {
    const wp4_json::Value& o = it->second[k];
    const std::string& key = o[0].str;
    const std::string& type = o[1].str;
    const std::string& value = o[2].str;
    std::printf("US026-MUTATION cpp_param_override %s %s %s\n", key.c_str(), type.c_str(), value.c_str());
    if (type == "f") {
      out->emplace_back(key, ParamValue(FromHex(value, 0)));
    } else if (type == "i") {
      out->emplace_back(key, ParamValue(std::stoi(value)));
    } else if (type == "b") {
      out->emplace_back(key, ParamValue(value == "1"));
    } else {
      ADD_FAILURE() << "unknown override type " << type;
    }
  }
}

// One 944 call as both sides record it.
struct CallOut {
  int ret = 0;
  int have = 0;
  float scale = 0;
  float residual = 0;
  float acc_sum = 0;
  std::vector<float> kfx, kfp, imux, imup;
};

std::vector<CallOut> GoldenCalls(const wp4_json::Value& root, const wp4_json::Value& run) {
  std::vector<CallOut> out;
  const wp4_json::Value& calls = run["calls"];
  const wp4_json::Value& states = root["states"];
  for (size_t c = 0; c < calls.size(); ++c) {
    const wp4_json::Value& g = calls[c];
    CallOut o;
    o.ret = static_cast<int>(g["ret"].ll());
    o.have = static_cast<int>(g["haveEstimate"].ll());
    o.scale = FromHex(g["scale"].str, 0);
    o.residual = FromHex(g["residual"].str, 0);
    o.acc_sum = FromHex(g["accSum"].str, 0);
    const wp4_json::Value& st = states[static_cast<size_t>(g["s"].ll())];
    o.kfx = HexFloats(st["kfX"].str);
    o.kfp = HexFloats(st["kfP"].str);
    o.imux = HexFloats(st["imuX"].str);
    o.imup = HexFloats(st["imuP"].str);
    out.push_back(std::move(o));
  }
  return out;
}

struct RunJudgement {
  Agree ret, have, scale, res, sum, kfx, kfp, imux, imup;
  int64_t first_accept = -1;
  int64_t golden_first_accept = -1;
  int64_t violations = 0;
  std::vector<std::string> failures;  // the first few, for the failure message
  // Sensitivity rule (method 4 with re-estimation only; sb != nullptr). v1_rule_scale_violations = how many calls would
  // fail the v1 rule (relative 1e-6) on the scale; they are REPORT, the decision is the per-call bound.
  bool sens = false;
  int64_t sens_calls = 0;        // calls judged with a per-call bound
  int64_t sens_fallback = 0;     // calls of the run judged with the v1 rule R34 (no bound for the call)
  int64_t sens_degenerate = 0;
  int64_t v1_rule_scale_violations = 0;
  int64_t sens_bit_diff_calls = 0;  // calls with a bound whose scale bits differ from the golden
  int64_t sens_rescued = 0;         // bit-different, outside 1e-6, inside the bound
  double sens_min_bound = std::numeric_limits<double>::infinity();
  double sens_max_bound = 0.0;
  double sens_max_ratio = 0.0;      // max over bound calls of observed relative difference / bound
  double sens_min_bound_ulps = std::numeric_limits<double>::infinity();  // min bound / (2^-23)
  std::vector<std::string> sens_lines;  // one US026-SENS line per bit-different bound call
};

// The G-MSE-U decision for one run (us026-mse-unit-gate.v1.md §3): exact ret / have_estimate / first accepted call /
// call count; scale and residual <= 3 ulp (methods 1, 2, 5) or relative <= 1e-6 (methods 3, 4); acc_sum and every
// 9-state and 6-state Kalman element <= 3 ulp. A violation is any of these failing; the gate needs 0.
RunJudgement JudgeRun(const std::vector<CallOut>& mine, const std::vector<CallOut>& golden, int method,
                      int64_t golden_first_accept, const ScaleBounds* sb = nullptr) {
  RunJudgement j;
  j.sens = sb != nullptr;
  j.golden_first_accept = golden_first_accept;
  auto bad = [&j](const std::string& what) {
    ++j.violations;
    if (j.failures.size() < 12) j.failures.push_back(what);
  };
  if (mine.size() != golden.size()) {
    bad("call count " + std::to_string(mine.size()) + " vs " + std::to_string(golden.size()));
  }
  const bool rel = method == 3 || method == 4;
  const size_t n = std::min(mine.size(), golden.size());
  auto vec = [&bad](const std::vector<float>& a, const std::vector<float>& g, int64_t at, const char* what, Agree* agree) {
    if (a.size() != g.size()) {
      bad(std::string(what) + " size at call " + std::to_string(at));
      return;
    }
    for (size_t i = 0; i < g.size(); ++i) {
      const bool ok = WithinUlps(a[i], g[i]);
      agree->Add(a[i], g[i], at, ok);
      if (!ok) bad(std::string(what) + " call " + std::to_string(at) + " element " + std::to_string(i) + " ulp " +
                   std::to_string(UlpDistance(a[i], g[i])));
    }
  };
  for (size_t c = 0; c < n; ++c) {
    const CallOut& a = mine[c];
    const CallOut& g = golden[c];
    const int64_t at = static_cast<int64_t>(c);
    j.ret.Add(static_cast<float>(a.ret), static_cast<float>(g.ret), at, a.ret == g.ret);
    if (a.ret != g.ret) bad("ret call " + std::to_string(c));
    if (a.ret != 0 && j.first_accept < 0) j.first_accept = at;
    j.have.Add(static_cast<float>(a.have), static_cast<float>(g.have), at, a.have == g.have);
    if (a.have != g.have) bad("have_estimate call " + std::to_string(c));
    bool scale_ok = rel ? WithinRel(a.scale, g.scale) : WithinUlps(a.scale, g.scale);
    bool defect_reported = false;
    if (sb != nullptr && method == 4 && c < sb->rel.size()) {
      // gate file §3.1 tier SENS: method 4 with re-estimation, per-call bound (rel < 0: the call has none -> R34).
      if (!WithinRel(a.scale, g.scale)) ++j.v1_rule_scale_violations;
      if (!sb->defect[c].empty()) {
        scale_ok = false;
        if (sb->defect[c] == "SENS-DEGENERATE") ++j.sens_degenerate;
        bad(sb->defect[c] + " call " + std::to_string(c));
        defect_reported = true;
      } else if (sb->rel[c] >= 0.0) {
        // tolerance = max(U3, 2 x S_c): within 3 ulp (tier U3, the protocol 3x-resolution floor) OR within the call's bound.
        scale_ok = WithinUlps(a.scale, g.scale) || WithinRelTol(a.scale, g.scale, sb->rel[c]);
        ++j.sens_calls;
        j.sens_min_bound = std::min(j.sens_min_bound, sb->rel[c]);
        j.sens_max_bound = std::max(j.sens_max_bound, sb->rel[c]);
        j.sens_min_bound_ulps = std::min(j.sens_min_bound_ulps, sb->rel[c] / 1.1920928955078125e-07);
        if (Bits(a.scale) != Bits(g.scale) && std::isfinite(a.scale) && std::isfinite(g.scale) && g.scale != 0.f) {
          const double obs = std::fabs(static_cast<double>(a.scale) - g.scale) / std::fabs(static_cast<double>(g.scale));
          ++j.sens_bit_diff_calls;
          if (obs > kRelMethods34 && scale_ok) ++j.sens_rescued;
          if (sb->rel[c] > 0.0) j.sens_max_ratio = std::max(j.sens_max_ratio, obs / sb->rel[c]);
          char buf[300];
          std::snprintf(buf, sizeof(buf),
                        "{\"call\":%lld,\"ulp\":%lld,\"rel\":%.4g,\"bound\":%.4g,\"ratio\":%.4g,\"cond\":%.4g,\"within\":%s}",
                        static_cast<long long>(c), static_cast<long long>(UlpDistance(a.scale, g.scale)), obs, sb->rel[c],
                        sb->rel[c] > 0.0 ? obs / sb->rel[c] : -1.0, sb->cond[c], scale_ok ? "true" : "false");
          j.sens_lines.push_back(buf);
        }
      } else {
        ++j.sens_fallback;
      }
    }
    j.scale.Add(a.scale, g.scale, at, scale_ok);
    if (!scale_ok && !defect_reported) {
      bad("scale call " + std::to_string(c) + " ulp " + std::to_string(UlpDistance(a.scale, g.scale)));
    }
    const bool res_ok = rel ? WithinRel(a.residual, g.residual) : WithinUlps(a.residual, g.residual);
    j.res.Add(a.residual, g.residual, at, res_ok);
    if (!res_ok) bad("residual call " + std::to_string(c));
    const bool sum_ok = WithinUlps(a.acc_sum, g.acc_sum);
    j.sum.Add(a.acc_sum, g.acc_sum, at, sum_ok);
    if (!sum_ok) bad("acc_sum call " + std::to_string(c));
    vec(a.kfx, g.kfx, at, "kf.x", &j.kfx);
    vec(a.kfp, g.kfp, at, "kf.P", &j.kfp);
    vec(a.imux, g.imux, at, "imu.x", &j.imux);
    vec(a.imup, g.imup, at, "imu.P", &j.imup);
  }
  if (j.first_accept != golden_first_accept) {
    bad("first accepted call " + std::to_string(j.first_accept) + " vs " + std::to_string(golden_first_accept));
  }
  return j;
}

std::string Joined(const std::vector<std::string>& v) {
  std::string s;
  for (const std::string& x : v) s += "\n  " + x;
  return s;
}

std::string SensJson(const RunJudgement& j) {
  if (!j.sens) return "null";
  char buf[420];
  std::snprintf(buf, sizeof(buf),
                "{\"bound_calls\":%lld,\"r34_fallback_calls\":%lld,\"degenerate\":%lld,\"bit_diff_calls\":%lld,"
                "\"v1_rule_scale_violations\":%lld,\"rescued_vs_v1\":%lld,\"min_bound\":%.4g,\"max_bound\":%.4g,"
                "\"min_bound_over_2^-23\":%.4g,\"max_ratio_obs_over_bound\":%.4g}",
                static_cast<long long>(j.sens_calls), static_cast<long long>(j.sens_fallback),
                static_cast<long long>(j.sens_degenerate), static_cast<long long>(j.sens_bit_diff_calls),
                static_cast<long long>(j.v1_rule_scale_violations), static_cast<long long>(j.sens_rescued),
                j.sens_calls > 0 ? j.sens_min_bound : 0.0, j.sens_max_bound,
                j.sens_calls > 0 ? j.sens_min_bound_ulps : 0.0, j.sens_max_ratio);
  return buf;
}

class Us026MethodGolden : public ::testing::TestWithParam<MethodRun> {};

TEST_P(Us026MethodGolden, OriginalPushFrameOnRecordedInputs) {
  if (!GateOpen()) FAIL() << GateRefusal();
  const wp4_json::Value& root = Load(GetParam().file);
  const wp4_json::Value& run = root["methods"][GetParam().run];
  const int method = static_cast<int>(run["method"].ll());
  const int32_t device_model = static_cast<int32_t>(run["device_model"].ll());
  ASSERT_TRUE(run["poke"]["held"].b) << "the original run did not keep the poked method";
  config::RegisterDefaults();
  // RegisterDefaults keeps keys that are already set, so every key a golden or a mutation copy may
  // change is reset to its binary default first (config_defaults.inc), then set.
  const bool reest = run.obj.count("estimate_after_have") != 0 && run["estimate_after_have"].ll() == 1;
  std::vector<std::pair<std::string, ParamValue>> params = {
      {"MetricScaleEstimator.imuGaussianSigma", ParamValue(0.0f)},
      {"MetricScaleEstimator.maxFreqForAlignment", ParamValue(5.0f)},
      {"MetricScaleEstimator.scaleEstimationMethod", ParamValue(method)},
      // Re-estimation variant (mse_method_golden.mjs --estimate-after-have 1): re-estimate every call.
      {"MetricScaleEstimator.estimateScaleAfterHaveEstimate", ParamValue(reest)}};
  ApplyCppOverrides(root, &params);
  config::ApplyParamOverrides(params);

  const wp4_json::Value& inputs = root["inputs"];
  ASSERT_EQ(inputs.size(), run["calls"].size());
  MetricScaleEstimator mse(config::MetricScaleEstimatorSettings::Get().movement_check_period_in_sec);
  const auto& s = MetricScaleEstimatorAccess::state(mse);
  std::vector<CallOut> mine;
  mine.reserve(inputs.size());
  // Tier SENS (gate file §3.1): method 4 with re-estimation. Every other run is judged exactly as in v1.
  const bool sens_run = method == 4 && reest;
  ScaleBounds sb;
  if (sens_run) {
    sb.rel.assign(inputs.size(), -1.0);
    sb.defect.assign(inputs.size(), std::string());
    sb.cond.assign(inputs.size(), 0.0);
  }
  for (size_t c = 0; c < inputs.size(); ++c) {
    const wp4_json::Value& in = inputs[c];
    TrackingSensorFrame f;
    f.device_model = device_model;
    f.sensor_events = Events(in["events"]);
    CallOut o;
    const int64_t captured_before = Capture().calls;
    o.ret = mse.PushFrame(f, Pose(in), std::stoll(in["timeNs"].str), static_cast<int>(in["state"].ll()),
                          in["reset"].ll() != 0)
                ? 1
                : 0;
    const bool captured = Capture().calls > captured_before;
    o.have = s.have_estimate ? 1 : 0;
    o.scale = mse.scale();
    o.residual = mse.residual();
    o.acc_sum = s.acc_sum;
    o.kfx.assign(s.kf.x.data(), s.kf.x.data() + 9);
    o.kfp.assign(s.kf.p.data(), s.kf.p.data() + 81);
    o.imux.assign(s.imu.x.data(), s.imu.x.data() + 6);
    o.imup.assign(s.imu.p.data(), s.imu.p.data() + 36);
    if (sens_run) {
      if (o.ret == 1) {
        // The call that produced mse.scale(): it must have gone through ScaleSpatial (S1), the replica of its normal
        // equation must reproduce the engine's scale bits (S2), and the sensitivity must be defined (S3).
        if (!captured) {
          sb.defect[c] = "SENS-NOCAPTURE";
        } else {
          Eigen::Matrix4f ata;
          Eigen::Vector4f atb;
          const float replica = ReplicaScaleSpatial(Capture(), &ata, &atb);
          if (Bits(replica) != Bits(Capture().real.scale) || Bits(Capture().real.scale) != Bits(o.scale)) {
            sb.defect[c] = "SENS-REPLICA";
          } else {
            const SensResult r = SensitivityBound(ata, atb, static_cast<int64_t>(c));
#ifndef SR_US026_HELD
            // Dev only (never in the held judge): dump the normal equation and the bound so that an independent
            // implementation (numpy, registration evidence) can recompute the bound from the same float bits.
            if (const char* dump = std::getenv("SR_US026_SENS_DUMP")) {
              if (std::FILE* fp = std::fopen(dump, "a")) {
                std::fprintf(fp, "%s %s %zu", GetParam().file.c_str(), GetParam().run.c_str(), c);
                for (int i = 0; i < 16; ++i) std::fprintf(fp, " %08x", Bits(ata.data()[i]));
                for (int i = 0; i < 4; ++i) std::fprintf(fp, " %08x", Bits(atb.data()[i]));
                std::fprintf(fp, " %.17g %.17g\n", r.bound, r.cond);
                std::fclose(fp);
              }
            }
#endif
            if (r.degenerate) {
              sb.defect[c] = "SENS-DEGENERATE";
            } else {
              sb.rel[c] = r.bound;
              sb.cond[c] = r.cond;
            }
          }
        }
      } else if (c > 0 && Bits(o.scale) == Bits(mine[c - 1].scale) && sb.rel[c - 1] >= 0.0 && sb.defect[c - 1].empty()) {
        // S4: no estimate in this call and the scale is the one of the previous call: it keeps that call's bound.
        sb.rel[c] = sb.rel[c - 1];
        sb.cond[c] = sb.cond[c - 1];
      }
    }
    mine.push_back(std::move(o));
  }
  const RunJudgement j = JudgeRun(mine, GoldenCalls(root, run), method, run["firstAccept"].ll(), sens_run ? &sb : nullptr);
  // Method 5 with a device model inside the lever-arm table is REPORT (gate file §3: 3-ulp boundary in dev, device
  // index <-> name INFERRED (spec 05 U7), no device-table UA scenario); its violations are printed, not judged.
  const bool gate = device_model == 0;
  if (gate) {
    EXPECT_EQ(j.violations, 0) << "G-MSE-U violations, first ones:" << Joined(j.failures);
  }
  std::printf(
      "US026-REPORT {\"test\":\"G-MSE-U\",\"grade\":\"%s\",\"file\":\"%s\",\"run\":\"%s\",\"method\":%d,"
      "\"device_model\":%d,\"estimate_after_have\":%d,\"calls\":%zu,\"first_accept\":%lld,\"golden_first_accept\":%lld,"
      "\"violations\":%lld,\"ret\":%s,\"have\":%s,\"scale\":%s,\"residual\":%s,\"acc_sum\":%s,\"kf_x\":%s,\"kf_P\":%s,"
      "\"imu_x\":%s,\"imu_P\":%s,\"scale_rule\":\"%s\",\"sens\":%s}\n",
      gate ? "GATE" : "REPORT", GetParam().file.c_str(), GetParam().run.c_str(), method, device_model, reest ? 1 : 0,
      mine.size(), static_cast<long long>(j.first_accept), static_cast<long long>(j.golden_first_accept),
      static_cast<long long>(j.violations), j.ret.Json().c_str(), j.have.Json().c_str(), j.scale.Json().c_str(),
      j.res.Json().c_str(), j.sum.Json().c_str(), j.kfx.Json().c_str(), j.kfp.Json().c_str(), j.imux.Json().c_str(),
      j.imup.Json().c_str(), sens_run ? "SENS" : (method == 3 || method == 4 ? "R34" : "U3"), SensJson(j).c_str());
  for (const std::string& line : j.sens_lines) {
    std::printf("US026-SENS {\"file\":\"%s\",\"run\":\"%s\",\"d\":%s}\n", GetParam().file.c_str(),
                GetParam().run.c_str(), line.c_str());
  }
}

std::string SafeName(std::string n) {
  for (char& ch : n) {
    if (!std::isalnum(static_cast<unsigned char>(ch))) ch = '_';
  }
  return n;
}

INSTANTIATE_TEST_SUITE_P(GMseU, Us026MethodGolden, ::testing::ValuesIn(MethodRuns()),
                         [](const ::testing::TestParamInfo<MethodRun>& info) {
                           return SafeName(info.param.file.substr(0, info.param.file.find('.')) + "_m" + info.param.run);
                         });

// ======================================================================= MS-P10 / MS-P11

struct Kf6Case {
  std::string file;
  std::string name;
};

std::vector<Kf6Case> Kf6Cases(const std::string& prefix) {
  std::vector<Kf6Case> out;
  for (const std::string& f : Files(".kf6.json")) {
    for (const auto& kv : Load(f)["results"].obj) {
      if (kv.first.compare(0, prefix.size(), prefix) == 0) {
        out.push_back({f, kv.first});
      }
    }
  }
  return out;
}

std::string Kf6Name(const ::testing::TestParamInfo<Kf6Case>& info) {
  return SafeName(info.param.file.substr(0, info.param.file.find('.')) + "_" + info.param.name);
}

// ---- MS-P10: func 1170 ImuBuffer::push (6-state angular KF) ----
class Us026Kf6Push : public ::testing::TestWithParam<Kf6Case> {};

TEST_P(Us026Kf6Push, OriginalImuPushPerCall) {
  if (!GateOpen()) FAIL() << GateRefusal();
  config::RegisterDefaults();
  const wp4_json::Value& r = Load(GetParam().file)["results"][GetParam().name];
  const int64_t base = std::stoll(r["base_ns"].str);
  const int max_frames = static_cast<int>(r["max_frames"].ll());
  const wp4_json::Value& steps = r["steps"];
  ImuBuffer buf;
  Agree a_x, a_p, a_f;
  int64_t exact_fail = 0;
  int reported = 0;
  for (size_t k = 0; k < steps.size(); ++k) {
    const wp4_json::Value& st = steps[k];
    const bool ret = buf.Push(Events(st["in"]["events"]), base, FromHex(st["in"]["tOldest"].str, 0), max_frames);
    const int64_t at = static_cast<int64_t>(k);
    EXPECT_EQ(ret ? 1 : 0, st["ret"].ll()) << "ret, call " << k;
    const long long sizes[5] = {static_cast<long long>(buf.lin_times.size()), static_cast<long long>(buf.gyro_times.size()),
                                static_cast<long long>(buf.gyro_values.size()),
                                static_cast<long long>(buf.all_event_times.size()), static_cast<long long>(buf.states.size())};
    const char* names[5] = {"nLin", "nGyroT", "nGyroV", "nAll", "nStates"};
    for (int i = 0; i < 5; ++i) {
      if (sizes[i] != st[names[i]].ll()) ++exact_fail;
      EXPECT_EQ(sizes[i], st[names[i]].ll()) << names[i] << ", call " << k;
    }
    EXPECT_EQ(Bits(buf.last_t), Bits(FromHex(st["lastT"].str, 0))) << "last_t, call " << k;
    EXPECT_EQ(buf.interval_ns, std::stoll(st["interval"].str)) << "interval, call " << k;
    CompareUlps(buf.x.data(), HexFloats(st["x"].str), at, "x", &a_x, &reported);
    CompareUlps(buf.p.data(), HexFloats(st["P"].str), at, "P", &a_p, &reported);
    if (st.obj.count("F") != 0) {
      CompareUlps(buf.f.data(), HexFloats(st["F"].str), at, "F", &a_f, &reported);
    }
  }
  const wp4_json::Value& fin = r["final"];
  CompareUlps(buf.f.data(), HexFloats(fin["F"].str), -1, "final F", &a_f, &reported);
  EXPECT_EQ(HexFloats(fin["Q"].str), std::vector<float>(buf.q.data(), buf.q.data() + 36)) << "Q";
  EXPECT_EQ(Bits(buf.rho), Bits(FromHex(fin["rho"].str, 0))) << "rho";
  ASSERT_EQ(static_cast<long long>(buf.states.size()), fin["nStates"].ll());
  Agree a_front;
  if (!buf.states.empty()) {
    for (const auto& pr : {std::make_pair(&fin["statesFront"], &buf.states.front()),
                           std::make_pair(&fin["statesBack"], &buf.states.back())}) {
      const std::vector<float> g = HexFloats(pr.first->str);
      CompareUlps(pr.second->x.data(), std::vector<float>(g.begin(), g.begin() + 6), -1, "state x", &a_front, &reported);
      CompareUlps(pr.second->p.data(), std::vector<float>(g.begin() + 6, g.begin() + 42), -1, "state P", &a_front,
                  &reported);
      CompareUlps(pr.second->f.data(), std::vector<float>(g.begin() + 42, g.begin() + 78), -1, "state F", &a_front,
                  &reported);
    }
    EXPECT_EQ(Bits(buf.gyro_times.front()), Bits(FromHex(fin["gyroTimesFront"].str, 0)));
    EXPECT_EQ(Bits(buf.gyro_times.back()), Bits(FromHex(fin["gyroTimesBack"].str, 0)));
  }
  std::printf(
      "US026-REPORT {\"test\":\"MS-P10\",\"grade\":\"GATE\",\"file\":\"%s\",\"case\":\"%s\",\"calls\":%zu,\"x\":%s,"
      "\"P\":%s,\"F\":%s,\"final_states\":%s,\"size_mismatch\":%lld}\n",
      GetParam().file.c_str(), GetParam().name.c_str(), steps.size(), a_x.Json().c_str(), a_p.Json().c_str(),
      a_f.Json().c_str(), a_front.Json().c_str(), static_cast<long long>(exact_fail));
}

INSTANTIATE_TEST_SUITE_P(MsP10, Us026Kf6Push, ::testing::ValuesIn(Kf6Cases("kf6_1170_")), Kf6Name);

// ---- MS-P10 (iOS pair): func 1169 position KF with useFpsScalingIfIos on/off ----
class Us026Kf9Ios : public ::testing::TestWithParam<Kf6Case> {};

TEST_P(Us026Kf9Ios, OriginalPositionPushPerFrame) {
  if (!GateOpen()) FAIL() << GateRefusal();
  config::RegisterDefaults();
  const wp4_json::Value& r = Load(GetParam().file)["results"][GetParam().name];
  const int64_t base = std::stoll(r["base_ns"].str);
  const bool ios = r["ios"].ll() != 0;
  const wp4_json::Value& steps = r["steps"];
  PositionKalman kf;
  Agree a_x, a_p, a_q;
  int reported = 0;
  for (size_t k = 0; k < steps.size(); ++k) {
    const wp4_json::Value& st = steps[k];
    const bool ret = kf.Push(Pose(st["in"]), base, std::stoll(st["in"]["timeNs"].str), static_cast<int>(r["max_frames"].ll()),
                             static_cast<int>(r["skip"].ll()), static_cast<int>(r["pred_n"].ll()), ios);
    const int64_t at = static_cast<int64_t>(k);
    EXPECT_EQ(ret ? 1 : 0, st["ret"].ll()) << "ret, frame " << k;
    EXPECT_EQ(Bits(kf.last_t), Bits(FromHex(st["lastT"].str, 0))) << "last_t, frame " << k;
    CompareUlps(kf.x.data(), HexFloats(st["x"].str), at, "x", &a_x, &reported);
    CompareUlps(kf.p.data(), HexFloats(st["P"].str), at, "P", &a_p, &reported);
    const float qa = kf.q(6, 6);
    CompareUlps(&qa, HexFloats(st["Qacc"].str), at, "Q(6,6)", &a_q, &reported);
  }
  std::printf("US026-REPORT {\"test\":\"MS-P10-ios-1169\",\"grade\":\"GATE\",\"file\":\"%s\",\"case\":\"%s\","
              "\"frames\":%zu,\"x\":%s,\"P\":%s,\"Qacc\":%s}\n",
              GetParam().file.c_str(), GetParam().name.c_str(), steps.size(), a_x.Json().c_str(), a_p.Json().c_str(),
              a_q.Json().c_str());
}

INSTANTIATE_TEST_SUITE_P(MsP10Ios, Us026Kf9Ios, ::testing::ValuesIn(Kf6Cases("kf9_1169_ios_")), Kf6Name);

// ---- MS-P11: func 1178 extract with the 6x6 RTS smoother ----
// "isolated": the C++ smoother runs on the original's filtered State6 deque and Q (read out of the
// original's memory), so only the smoother is compared (GATE). "chain": the C++ ImuBuffer is filled from the
// same events first (MS-P10 path) and then smoothed; REPORT only.
class Us026Rts6 : public ::testing::TestWithParam<Kf6Case> {};

struct Extracted {
  std::vector<float> ti, tg;
  std::vector<Vec3> ai, w, wdot;
};

Extracted Extract(const std::deque<float>& lin_times, const std::deque<Vec3>& lin_values,
                  const std::deque<float>& gyro_times, const std::vector<KalmanState>& sm, float t0, float t1,
                  bool want_gyro) {
  Extracted e;
  for (size_t j = 0; j < lin_times.size(); ++j) {
    if (t0 <= lin_times[j] && lin_times[j] <= t1) {
      e.ti.push_back(lin_times[j]);
      e.ai.push_back(lin_values[j]);
    }
  }
  if (want_gyro) {
    for (size_t j = 0; j < gyro_times.size() && j < sm.size(); ++j) {
      if (t0 <= gyro_times[j] && gyro_times[j] <= t1) {
        e.tg.push_back(gyro_times[j]);
        e.w.push_back({sm[j].x(0), sm[j].x(1), sm[j].x(2)});
        e.wdot.push_back({sm[j].x(3), sm[j].x(4), sm[j].x(5)});
      }
    }
  }
  return e;
}

void CompareVec4(const std::vector<Vec3>& mine, const wp4_json::Value& golden, const char* what, Agree* agree,
                 int* reported, bool judge) {
  ASSERT_EQ(mine.size(), golden.size()) << what;
  for (size_t j = 0; j < mine.size(); ++j) {
    const std::vector<float> g = HexFloats(golden[j].str);
    for (int c = 0; c < 3; ++c) {
      const bool ok = WithinUlps(mine[j][c], g[c]);
      agree->Add(mine[j][c], g[c], static_cast<int64_t>(j), ok);
      if (judge && !ok && (*reported)++ < 12) {
        ADD_FAILURE() << what << " row " << j << " c " << c << ": " << mine[j][c] << " vs " << g[c] << " (ulp "
                      << UlpDistance(mine[j][c], g[c]) << ")";
      }
    }
    EXPECT_EQ(Bits(g[3]), 0u) << what << " w lane";
  }
}

TEST_P(Us026Rts6, OriginalExtractSmoothedGyro) {
  if (!GateOpen()) FAIL() << GateRefusal();
  config::RegisterDefaults();
  const wp4_json::Value& r = Load(GetParam().file)["results"][GetParam().name];
  const wp4_json::Value& b = r["buffer"];
  const float t0 = FromHex(r["in"]["t0"].str, 0);
  const float t1 = FromHex(r["in"]["t1"].str, 0);
  // Original buffer contents.
  std::deque<float> lin_times, gyro_times;
  std::deque<Vec3> lin_values;
  for (size_t j = 0; j < b["linTimes"].size(); ++j) lin_times.push_back(FromHex(b["linTimes"][j].str, 0));
  for (size_t j = 0; j < b["gyroTimes"].size(); ++j) gyro_times.push_back(FromHex(b["gyroTimes"][j].str, 0));
  for (size_t j = 0; j < b["linValues"].size(); ++j) {
    const std::vector<float> v = HexFloats(b["linValues"][j].str);
    lin_values.push_back({v[0], v[1], v[2]});
  }
  std::deque<KalmanState> states;
  for (size_t j = 0; j < b["states"].size(); ++j) {
    const std::vector<float> v = HexFloats(b["states"][j].str);
    KalmanState s;
    s.x = Eigen::Map<const Eigen::VectorXf>(v.data(), 6);
    s.p = Eigen::Map<const Eigen::MatrixXf>(v.data() + 6, 6, 6);
    s.f = Eigen::Map<const Eigen::MatrixXf>(v.data() + 42, 6, 6);
    states.push_back(s);
  }
  const std::vector<float> qv = HexFloats(b["Q"].str);
  const Eigen::MatrixXf q = Eigen::Map<const Eigen::MatrixXf>(qv.data(), 6, 6);
  int reported = 0;
  Agree iso_w, iso_wdot, chain_w, chain_wdot, chain_states;
  for (const bool want : {true, false}) {
    const wp4_json::Value& g = r["out"][want ? "wantGyro1" : "wantGyro0"];
    const Extracted e = Extract(lin_times, lin_values, gyro_times, want ? RtsSmooth(states, q) : std::vector<KalmanState>(),
                                t0, t1, want);
    // Selection: exact.
    ASSERT_EQ(e.ti.size(), g["Ti"].size());
    for (size_t j = 0; j < e.ti.size(); ++j) EXPECT_EQ(Bits(e.ti[j]), Bits(FromHex(g["Ti"][j].str, 0))) << "Ti " << j;
    ASSERT_EQ(e.ai.size(), g["Ai"].size());
    for (size_t j = 0; j < e.ai.size(); ++j) {
      const std::vector<float> v = HexFloats(g["Ai"][j].str);
      for (int c = 0; c < 3; ++c) EXPECT_EQ(Bits(e.ai[j][c]), Bits(v[c])) << "Ai " << j;
    }
    ASSERT_EQ(e.tg.size(), g["Tg"].size());
    for (size_t j = 0; j < e.tg.size(); ++j) EXPECT_EQ(Bits(e.tg[j]), Bits(FromHex(g["Tg"][j].str, 0))) << "Tg " << j;
    CompareVec4(e.w, g["W"], "W (isolated)", &iso_w, &reported, true);
    CompareVec4(e.wdot, g["Wdot"], "Wdot (isolated)", &iso_wdot, &reported, true);
  }
  // Chain (REPORT): C++ filter on the same events, then the C++ smoother.
  {
    ImuBuffer buf;
    const int64_t base = std::stoll(r["in"]["base_ns"].str);
    const int max_frames = static_cast<int>(r["in"]["max_frames"].ll());
    for (size_t c = 0; c < r["in"]["events"].size(); ++c) {
      buf.Push(Events(r["in"]["events"][c]), base, 0.0f, max_frames);
    }
    EXPECT_EQ(buf.states.size(), states.size()) << "chain: State6 count";
    for (size_t j = 0; j < std::min(buf.states.size(), states.size()); ++j) {
      for (int i = 0; i < 6; ++i) chain_states.Add(buf.states[j].x(i), states[j].x(i), static_cast<int64_t>(j), true);
    }
    const Extracted e =
        Extract(buf.lin_times, buf.lin_values, buf.gyro_times, RtsSmooth(buf.states, buf.q), t0, t1, true);
    const wp4_json::Value& g = r["out"]["wantGyro1"];
    CompareVec4(e.w, g["W"], "W (chain)", &chain_w, &reported, false);
    CompareVec4(e.wdot, g["Wdot"], "Wdot (chain)", &chain_wdot, &reported, false);
  }
  std::printf(
      "US026-REPORT {\"test\":\"MS-P11\",\"grade\":\"GATE(isolated)+REPORT(chain)\",\"file\":\"%s\",\"case\":\"%s\","
      "\"states\":%zu,\"isolated_W\":%s,\"isolated_Wdot\":%s,\"chain_filter_x\":%s,\"chain_W\":%s,\"chain_Wdot\":%s}\n",
      GetParam().file.c_str(), GetParam().name.c_str(), states.size(), iso_w.Json().c_str(), iso_wdot.Json().c_str(),
      chain_states.Json().c_str(), chain_w.Json().c_str(), chain_wdot.Json().c_str());
}

INSTANTIATE_TEST_SUITE_P(MsP11, Us026Rts6, ::testing::ValuesIn(Kf6Cases("rts6_1178_")), Kf6Name);

// A data directory may hold only one golden kind (mutation copies); a missing kind is then not a failure of the
// suites above. The judged directory must hold both: Us026Data.GoldenFilesPresent fails otherwise (an empty
// directory must not pass).
GTEST_ALLOW_UNINSTANTIATED_PARAMETERIZED_TEST(Us026MethodGolden);
GTEST_ALLOW_UNINSTANTIATED_PARAMETERIZED_TEST(Us026Kf6Push);
GTEST_ALLOW_UNINSTANTIATED_PARAMETERIZED_TEST(Us026Kf9Ios);
GTEST_ALLOW_UNINSTANTIATED_PARAMETERIZED_TEST(Us026Rts6);

TEST(Us026Data, GoldenFilesPresent) {
  if (!GateOpen()) FAIL() << GateRefusal();
  EXPECT_FALSE(Files(".methods.json").empty()) << DataDir();
  EXPECT_FALSE(Files(".kf6.json").empty()) << DataDir();
  std::printf("US026-REPORT {\"test\":\"data\",\"dir\":\"%s\",\"methods_files\":%zu,\"kf6_files\":%zu}\n",
              DataDir().c_str(), Files(".methods.json").size(), Files(".kf6.json").size());
}

#ifndef SR_US026_HELD
// ======================================================================= the gate's own null-run and comparator power
// (us026-mse-unit-gate.v1.md §4 NR-MSE-1, §5 comparator mutations). These use the checked-in DEV golden only.

std::string DevMethodsGolden() {
  return std::string(SR_TEST_DATA_DIR) + "/metric-scale/us026/flip6-arcore-s3-absolute-core2.dev.methods.json";
}

const wp4_json::Value& DevRoot() {
  static const wp4_json::Value root = wp4_json::Load(DevMethodsGolden());
  return root;
}

// NR-MSE-1: the comparator applied to a golden against itself reports no difference (every comparison exact).
TEST(Us026Null, ComparatorSelfVsSelfMethods) {
  int64_t runs = 0;
  int64_t comparisons = 0;
  for (const std::string& f : Files(".methods.json")) {
    const wp4_json::Value& root = Load(f);
    for (const auto& kv : root["methods"].obj) {
      const std::vector<CallOut> g = GoldenCalls(root, kv.second);
      const RunJudgement j = JudgeRun(g, g, static_cast<int>(kv.second["method"].ll()), kv.second["firstAccept"].ll());
      EXPECT_EQ(j.violations, 0) << f << " " << kv.first;
      for (const Agree* a : {&j.ret, &j.have, &j.scale, &j.res, &j.sum, &j.kfx, &j.kfp, &j.imux, &j.imup}) {
        EXPECT_EQ(a->exact, a->n) << f << " " << kv.first;
        EXPECT_EQ(a->max_ulp, 0) << f << " " << kv.first;
        comparisons += a->n;
      }
      ++runs;
    }
  }
  EXPECT_GT(runs, 0);
  std::printf("US026-NULL {\"id\":\"NR-MSE-1\",\"what\":\"methods comparator self vs self\",\"runs\":%lld,"
              "\"comparisons\":%lld,\"violations\":0}\n",
              static_cast<long long>(runs), static_cast<long long>(comparisons));
}

TEST(Us026Null, ComparatorSelfVsSelfKf6) {
  int64_t n = 0;
  for (const std::string& f : Files(".kf6.json")) {
    for (const auto& kv : Load(f)["results"].obj) {
      if (kv.second.obj.count("steps") == 0) continue;
      for (size_t k = 0; k < kv.second["steps"].size(); ++k) {
        const wp4_json::Value& st = kv.second["steps"][k];
        for (const char* key : {"x", "P", "F"}) {
          if (st.obj.count(key) == 0) continue;
          const std::vector<float> g = HexFloats(st[key].str);
          Agree a;
          int reported = 0;
          CompareUlps(g.data(), g, static_cast<int64_t>(k), key, &a, &reported);
          EXPECT_EQ(a.exact, a.n);
          n += a.n;
        }
      }
    }
  }
  EXPECT_GT(n, 0);
  std::printf("US026-NULL {\"id\":\"NR-MSE-1\",\"what\":\"kf6 comparator self vs self\",\"comparisons\":%lld,"
              "\"violations\":0}\n",
              static_cast<long long>(n));
}

float AddUlps(float v, int k) {
  uint32_t u = Bits(v);
  u = static_cast<uint32_t>(static_cast<int64_t>(u) + k);  // away from zero for k > 0 (both signs)
  return FromBits(u);
}

struct CmpCase {
  const char* id;
  int method;  // golden run key
  std::function<void(std::vector<CallOut>*, int64_t* first_accept)> mutate;
  bool expect_detected;
};

// Comparator power on in-memory copies of the dev golden (the C++ estimator is not run: golden vs mutated golden).
TEST(Us026Comparator, DetectsAtTheDeclaredResolution) {
  const wp4_json::Value& root = DevRoot();
  const CmpCase cases[] = {
      // integer tier (exact-class sub-check): the smallest possible difference is one step
      {"CMP-INT-1 ret flipped at the first accepted call", 1,
       [](std::vector<CallOut>* c, int64_t* fa) { (*c)[static_cast<size_t>(*fa)].ret ^= 1; }, true},
      {"CMP-INT-2 first accepted call one later (ret of the first accept 0)", 2,
       [](std::vector<CallOut>* c, int64_t* fa) {
         (*c)[static_cast<size_t>(*fa)].ret = 0;
         ++*fa;
         (*c)[static_cast<size_t>(*fa)].ret = 1;
       },
       true},
      {"CMP-INT-3 have_estimate flipped at one call", 1, [](std::vector<CallOut>* c, int64_t*) { (*c)[800].have ^= 1; }, true},
      {"CMP-INT-4 one call missing", 1, [](std::vector<CallOut>* c, int64_t*) { c->pop_back(); }, true},
      // 3-ulp tier: 3 ulp is inside the tolerance by definition, 4 ulp is outside
      {"CMP-ULP-3 one kf.P element +3 ulp (inside tolerance, not detected by design)", 1,
       [](std::vector<CallOut>* c, int64_t*) { (*c)[800].kfp[10] = AddUlps((*c)[800].kfp[10], 3); }, false},
      {"CMP-ULP-4 one kf.P element +4 ulp", 1,
       [](std::vector<CallOut>* c, int64_t*) { (*c)[800].kfp[10] = AddUlps((*c)[800].kfp[10], 4); }, true},
      {"CMP-ULP-4s scale +4 ulp at the first accepted call (method 1)", 1,
       [](std::vector<CallOut>* c, int64_t* fa) { (*c)[static_cast<size_t>(*fa)].scale = AddUlps((*c)[static_cast<size_t>(*fa)].scale, 4); },
       true},
      {"CMP-ULP-1 imu.x element +1 ulp (inside tolerance, not detected by design)", 2,
       [](std::vector<CallOut>* c, int64_t*) { (*c)[800].imux[2] = AddUlps((*c)[800].imux[2], 1); }, false},
      // relative tier (methods 3, 4): 5e-7 inside, 2e-6 outside
      {"CMP-REL-in method 3 scale x(1+5e-7) at the first accepted call (inside, not detected by design)", 3,
       [](std::vector<CallOut>* c, int64_t* fa) {
         float& s = (*c)[static_cast<size_t>(*fa)].scale;
         s = static_cast<float>(s * (1.0 + 5e-7));
       },
       false},
      {"CMP-REL-out method 3 scale x(1+2e-6) at the first accepted call", 3,
       [](std::vector<CallOut>* c, int64_t* fa) {
         float& s = (*c)[static_cast<size_t>(*fa)].scale;
         s = static_cast<float>(s * (1.0 + 2e-6));
       },
       true},
      {"CMP-REL-out4 method 4 residual -1 -> -1.000002", 4,
       [](std::vector<CallOut>* c, int64_t*) { (*c)[800].residual = -1.000002f; }, true},
      // non-finite golden values fail closed
      {"CMP-NAN golden scale NaN at one call", 1,
       [](std::vector<CallOut>* c, int64_t*) { (*c)[800].scale = std::numeric_limits<float>::quiet_NaN(); }, true},
  };
  for (const CmpCase& cs : cases) {
    const wp4_json::Value& run = root["methods"][std::to_string(cs.method)];
    const std::vector<CallOut> golden = GoldenCalls(root, run);
    std::vector<CallOut> mutated = golden;
    int64_t fa = run["firstAccept"].ll();
    ASSERT_GE(fa, 0);
    const int64_t original_fa = fa;
    cs.mutate(&mutated, &fa);
    // the mutated copy plays the C++ side; the golden keeps its own first accepted call
    const RunJudgement j = JudgeRun(mutated, golden, cs.method, original_fa);
    const bool detected = j.violations > 0;
    EXPECT_EQ(detected, cs.expect_detected) << cs.id << Joined(j.failures);
    std::printf("US026-CMP {\"id\":\"%s\",\"method\":%d,\"expect_detected\":%s,\"detected\":%s,\"violations\":%lld}\n",
                cs.id, cs.method, cs.expect_detected ? "true" : "false", detected ? "true" : "false",
                static_cast<long long>(j.violations));
  }
}

// ======================================================================= tier SENS (gate file §3.1, §5): power of the new rule
// Synthetic captured call: n samples of V, Is, Omega, Omega-dot from a fixed generator. axes = 3: rotation about all three
// axes (well-conditioned normal equation); axes = 1: rotation about y only (the lever-arm columns degenerate, spec 05 U6).
SpatialCapture SyntheticCall(int axes, int n, float amp = 1.f) {
  SpatialCapture c;
  SplitMix64 g{12345};
  for (int k = 0; k < n; ++k) {
    const float t = 0.02f * static_cast<float>(k);
    const float w0 = axes == 3 ? amp * 1.5f * std::sin(0.7f * t + static_cast<float>(g.Uniform()) * 0.1f) : 0.f;
    const float w1 = amp * 1.2f * std::cos(0.9f * t);
    const float w2 = axes == 3 ? amp * 1.0f * std::sin(1.3f * t + 1.f) : 0.f;
    c.omega.push_back({w0, w1, w2});
    c.omega_dot.push_back({0.1f * w1, -0.2f * w0, 0.05f * w2});
    c.v.push_back({static_cast<float>(0.5 * std::sin(t) + 0.01 * g.Signed(1.0)), static_cast<float>(0.4 * std::cos(1.1 * t)),
                   static_cast<float>(0.3 * std::sin(0.5 * t + 2.0))});
    c.imu.push_back({static_cast<float>(0.5 * c.v.back()[0] * 0.45 + 0.02 * g.Signed(1.0)), 0.2f * w1, 0.1f * w2});
  }
  return c;
}

TEST(Us026Sens, BoundIsDeterministicPerCallAndFollowsTheMatrix) {
  Eigen::Matrix4f well_a, ill_a;
  Eigen::Vector4f well_b, ill_b;
  const SpatialCapture well = SyntheticCall(3, 400);
  ReplicaScaleSpatial(well, &well_a, &well_b);
  // an explicitly ill-conditioned normal equation: x3 is almost collinear with x0 (coupling 0.9999)
  ill_a << 1.f, 0.f, 0.f, 0.9999f, 0.f, 1.f, 0.f, 0.f, 0.f, 0.f, 1.f, 0.f, 0.9999f, 0.f, 0.f, 1.f;
  ill_b << 1.f, 0.1f, 0.1f, 1.f;
  const SensResult w1 = SensitivityBound(well_a, well_b, 7);
  const SensResult w2 = SensitivityBound(well_a, well_b, 7);
  const SensResult w3 = SensitivityBound(well_a, well_b, 8);
  const SensResult i1 = SensitivityBound(ill_a, ill_b, 7);
  ASSERT_FALSE(w1.degenerate);
  ASSERT_FALSE(i1.degenerate);
  EXPECT_EQ(std::memcmp(&w1.bound, &w2.bound, sizeof(double)), 0) << "same matrix, same call index: same bound";
  EXPECT_NE(std::memcmp(&w1.bound, &w3.bound, sizeof(double)), 0) << "another call index draws another stream";
  EXPECT_GT(w1.bound, 0.0);
  EXPECT_GT(i1.cond, 100.0 * w1.cond) << "the ill matrix is much worse conditioned";
  EXPECT_GT(i1.bound, 100.0 * w1.bound) << "a much worse-conditioned matrix gets a much larger bound";
  EXPECT_LT(w1.bound, 1e-5);
  // a singular normal equation has no sensitivity: degenerate (the judge then fails the call closed)
  Eigen::Matrix4f zero = Eigen::Matrix4f::Zero();
  EXPECT_TRUE(SensitivityBound(zero, well_b, 1).degenerate);
  std::printf("US026-SENSCASE {\"well_cond\":%.4g,\"well_bound\":%.4g,\"ill_cond\":%.4g,\"ill_bound\":%.4g,\"well_other_stream\":%.4g}\n",
              w1.cond, w1.bound, i1.cond, i1.bound, w3.bound);
}

TEST(Us026Sens, ReplicaReproducesTheEngineAndDetectsAChangedMatrix) {
  const SpatialCapture in = SyntheticCall(3, 300);
  const int64_t before = Capture().calls;
  const ScaleFit real = WrappedScaleSpatial(in.v, in.imu, in.omega, in.omega_dot);  // through the engine's function
  ASSERT_EQ(Capture().calls, before + 1);
  Eigen::Matrix4f a;
  Eigen::Vector4f b;
  EXPECT_EQ(Bits(ReplicaScaleSpatial(Capture(), &a, &b)), Bits(real.scale)) << "replica = engine on a synthetic call";
  SpatialCapture tampered = Capture();
  tampered.imu[10][0] *= 1.01f;  // a changed input: the replica must stop matching (mutation of the replica check)
  EXPECT_NE(Bits(ReplicaScaleSpatial(tampered, &a, &b)), Bits(real.scale));
  SpatialCapture tampered2 = Capture();
  tampered2.omega[20][1] += 0.01f;
  EXPECT_NE(Bits(ReplicaScaleSpatial(tampered2, &a, &b)), Bits(real.scale));
}

TEST(Us026Sens, JudgeBoundaryAndFailClosedCases) {
  const wp4_json::Value& root = DevRoot();
  const wp4_json::Value& run = root["methods"]["4"];
  const std::vector<CallOut> golden = GoldenCalls(root, run);
  const int64_t fa = run["firstAccept"].ll();
  ASSERT_GE(fa, 0);
  const size_t at = static_cast<size_t>(fa);
  struct Case {
    const char* id;
    double scale_factor;  // mutated C++ scale = golden * factor
    double bound;         // per-call bound; <0 none (R34)
    const char* defect;
    bool expect_detected;
  };
  constexpr double kB = 1e-5;
  const Case cases[] = {
      {"SENS-IN-PLUS   scale x(1+0.9B)", 1.0 + 0.9 * kB, kB, "", false},
      {"SENS-OUT-PLUS  scale x(1+1.1B)", 1.0 + 1.1 * kB, kB, "", true},
      {"SENS-IN-MINUS  scale x(1-0.9B)", 1.0 - 0.9 * kB, kB, "", false},
      {"SENS-OUT-MINUS scale x(1-1.1B)", 1.0 - 1.1 * kB, kB, "", true},
      {"SENS-WIDER-THAN-R34 x(1+5e-6) inside B=1e-5 (outside the v1 rule)", 1.0 + 5e-6, kB, "", false},
      {"SENS-NO-BOUND-IN   x(1+5e-7) with no bound: v1 rule R34", 1.0 + 5e-7, -1.0, "", false},
      {"SENS-NO-BOUND-OUT  x(1+2e-6) with no bound: v1 rule R34", 1.0 + 2e-6, -1.0, "", true},
      {"SENS-ZERO-BOUND    x(1+1e-6) with bound 0 (outside 3 ulp)", 1.0 + 1e-6, 0.0, "", true},
      {"SENS-U3-FLOOR      x(1+1.5e-7) (1-2 ulp) with a tiny bound 1e-9: inside via U3", 1.0 + 1.5e-7, 1e-9, "", false},
      {"SENS-U3-FLOOR-OUT  x(1+1e-6) (8+ ulp) with a tiny bound 1e-9", 1.0 + 1e-6, 1e-9, "", true},
      {"SENS-DEFECT-NOCAPTURE the call has no capture", 1.0, kB, "SENS-NOCAPTURE", true},
      {"SENS-DEFECT-REPLICA   the replica differs", 1.0, kB, "SENS-REPLICA", true},
  };
  for (const Case& cs : cases) {
    std::vector<CallOut> mutated = golden;
    mutated[at].scale = static_cast<float>(static_cast<double>(golden[at].scale) * cs.scale_factor);
    ScaleBounds sb;
    sb.rel.assign(golden.size(), -1.0);
    sb.defect.assign(golden.size(), std::string());
    sb.cond.assign(golden.size(), 1.0);
    sb.rel[at] = cs.bound;
    sb.defect[at] = cs.defect;
    const RunJudgement j = JudgeRun(mutated, golden, 4, fa, &sb);
    const bool detected = j.violations > 0;
    EXPECT_EQ(detected, cs.expect_detected) << cs.id << Joined(j.failures);
    std::printf("US026-CMP {\"id\":\"%s\",\"method\":4,\"expect_detected\":%s,\"detected\":%s,\"violations\":%lld}\n",
                cs.id, cs.expect_detected ? "true" : "false", detected ? "true" : "false", static_cast<long long>(j.violations));
  }
  // without bounds (sb = nullptr) the judge is the v1 judge: x(1+5e-6) is outside 1e-6
  std::vector<CallOut> mutated = golden;
  mutated[at].scale = static_cast<float>(static_cast<double>(golden[at].scale) * (1.0 + 5e-6));
  EXPECT_GT(JudgeRun(mutated, golden, 4, fa).violations, 0) << "no bound: the v1 rule must still reject 5e-6";
  // a run that is not method 4 never uses the bounds
  ScaleBounds sb;
  sb.rel.assign(golden.size(), 1.0);
  sb.defect.assign(golden.size(), std::string());
  sb.cond.assign(golden.size(), 1.0);
  const RunJudgement m1 = JudgeRun(mutated, golden, 3, fa, &sb);
  EXPECT_GT(m1.violations, 0) << "method 3 is judged by R34 even if bounds are supplied";
}

// ======================================================================= release order on synthetic files
// The release logic itself (every bypass of protocol §12.5) is the conformance of the shared device
// (tests/support/sr_held_conformance.cc). What this file adds: the ORDER (golden marker, golden provenance, then the
// judge marker) and that every refusal leaves no judge marker, so it does not use up the one judge run.

void PutFile(const std::string& path, const std::string& body) {
  std::filesystem::create_directories(std::filesystem::path(path).parent_path());
  std::ofstream f(path, std::ios::binary | std::ios::trunc);
  f << body;
}

bool Exists(const std::string& path) {
  struct stat st;
  return ::lstat(path.c_str(), &st) == 0;
}

std::string Sha(const std::string& s) { return testutil::Sha256Hex(s); }

struct ReleaseFixture {
  std::string root, prereg, inputs, goldens, exe, judge_marker, golden_marker;
  std::string wasm_sha = std::string(64, 'a'), mmg_sha = std::string(64, 'b'), kf6_sha = std::string(64, 'c');
  std::string dump_sha, seed_sha;
  ReleaseInputs in;
};

std::string MethodsGoldenText(const ReleaseFixture& f, bool reest, const std::string& extra = "") {
  const std::string run_tail = reest ? ",\"estimate_after_have\":1" : "";
  return "{\"schema\":\"slam-refact-us026-mse-method-golden-v1\",\"original_sha256\":\"" + f.wasm_sha +
         "\",\"code_bytes_unchanged\":true,\"script_sha256\":\"" + f.mmg_sha + "\",\"dump_sha256\":\"" + f.dump_sha +
         "\",\"core\":2,\"last_call\":null,\"inputs\":[{}],\"states\":[]," + extra + "\"methods\":{\"1\":{\"poke\":{\"held\":true}" +
         run_tail + ",\"calls\":[{}]},\"5@52\":{\"poke\":{\"held\":true}" + run_tail + ",\"calls\":[{}]}}}";
}

std::string Kf6GoldenText(const ReleaseFixture& f, const std::string& seed_class = "registration") {
  return "{\"original_sha256\":\"" + f.wasm_sha + "\",\"code_bytes_unchanged\":true,\"script_sha256\":\"" + f.kf6_sha +
         "\",\"seed\":{\"class\":\"" + seed_class + "\",\"seed_file_sha256\":\"" + f.seed_sha + "\"},\"results\":{}}";
}

void WriteGoldens(ReleaseFixture* f, const std::map<std::string, std::string>& files) {
  std::filesystem::remove_all(f->goldens);
  std::string sums;
  for (const auto& [name, body] : files) {
    PutFile(f->goldens + "/" + name, body);
    sums += Sha(body) + "  " + name + "\n";
  }
  PutFile(f->goldens + "/GOLDEN-SHA256SUMS", sums);
}

std::map<std::string, std::string> RegisteredGoldens(const ReleaseFixture& f) {
  return {{"d1-c2-default.methods.json", MethodsGoldenText(f, false)},
          {"d1-c2-reest.methods.json", MethodsGoldenText(f, true)},
          {"registration.kf6.json", Kf6GoldenText(f)}};
}

// Builds a registered, reviewed and designated synthetic gate g1.v1 and runs its golden step (kind golden).
ReleaseFixture MakeReleaseFixture(const std::string& tag, bool run_golden_step = true) {
  ReleaseFixture f;
  f.root = (std::filesystem::temp_directory_path() / ("sr_us026_release_" + tag + "_" + std::to_string(getpid()))).string();
  std::filesystem::remove_all(f.root);
  f.prereg = f.root + "/prereg";
  f.inputs = f.prereg + "/held-inputs";
  f.goldens = f.root + "/goldens";
  f.exe = f.root + "/exe.bin";
  const std::string exe_bytes = "the held binary", gate_md = "the gate file", review_md = "the review", dump = "a dump\n";
  const std::string seed = "12345\n";
  PutFile(f.exe, exe_bytes);
  PutFile(f.prereg + "/g1.v1.md", gate_md);
  PutFile(f.prereg + "/g1.v1.review.md", review_md);
  PutFile(f.prereg + "/g1.v1.coordinator-seed.txt", seed);
  PutFile(f.inputs + "/d1.dump.jsonl", dump);
  f.dump_sha = Sha(dump);
  f.seed_sha = Sha(seed);
  std::string tree, code;
  EXPECT_TRUE(sh::TreeSha(f.inputs, &tree, &code)) << code;
  auto manifest = [&](const std::string& designation_sha) {
    return "{\"gate_id\":\"g1\",\"version\":1,\"file\":\"g1.v1.md\",\"sha256\":\"" + Sha(gate_md) +
           "\",\"created_at\":\"2026-01-01T00:00:00+09:00\",\"held\":{\"kinds\":[\"golden\",\"judge\"],\"pins\":{\"exe.bin\":\"" +
           Sha(exe_bytes) + "\"},\"inputs\":{\"golden\":\"" + tree + "\"},\"seed\":true,\"dev_seeds\":[\"26001\"]}," +
           "\"registration\":{\"inputs_dir\":\"held-inputs\",\"original_wasm_sha256\":\"" + f.wasm_sha +
           "\",\"tool_sha256\":{\"mse_method_golden.mjs\":\"" + f.mmg_sha + "\",\"mse-probe-kf6.mjs\":\"" + f.kf6_sha +
           "\"},\"methods_runs\":[\"1\",\"5@52\"],\"goldens\":[" +
           "{\"file\":\"d1-c2-default.methods.json\",\"kind\":\"methods\",\"dump\":\"d1.dump.jsonl\",\"dump_sha256\":\"" + f.dump_sha +
           "\",\"core\":2,\"estimate_after_have\":0}," +
           "{\"file\":\"d1-c2-reest.methods.json\",\"kind\":\"methods\",\"dump\":\"d1.dump.jsonl\",\"dump_sha256\":\"" + f.dump_sha +
           "\",\"core\":2,\"estimate_after_have\":1}," + "{\"file\":\"registration.kf6.json\",\"kind\":\"kf6\"}]}," +
           "\"review\":{\"reviewer\":\"test\",\"at\":\"2026-01-02T00:00:00+09:00\",\"file\":\"g1.v1.review.md\",\"sha256\":\"" +
           Sha(review_md) + "\",\"designation_file\":\"g1.v1.review-designation.json\",\"designation_file_sha256\":\"" +
           designation_sha + "\"},\"first_real_run_at\":null}";
  };
  sh::Json parsed;
  std::string err, canon;
  EXPECT_TRUE(sh::ParseJson(manifest(std::string(64, '0')), &parsed, &err)) << err;
  EXPECT_TRUE(sh::CanonManifestSha(parsed, &canon));
  const std::string designation = "{\"gate\":\"g1.v1\",\"first_real_run_permitted\":true,\"pinned\":{\"gate_md_sha256\":\"" +
                                  Sha(gate_md) + "\",\"review_file_sha256\":\"" + Sha(review_md) +
                                  "\",\"manifest_before_review_sha256\":\"" + canon + "\",\"seed_file_sha256\":\"" + f.seed_sha + "\"}}";
  PutFile(f.prereg + "/g1.v1.review-designation.json", designation);
  PutFile(f.prereg + "/g1.v1.manifest.json", manifest(Sha(designation)));
  f.judge_marker = f.prereg + "/g1.v1.first-run.judge.marker";
  f.golden_marker = f.prereg + "/g1.v1.first-run.golden.marker";
  f.in.env_value = Sha(designation);
  f.in.prereg_dir = f.prereg;
  f.in.gate_name = "g1.v1";
  f.in.pins = {f.exe};
  f.in.golden_dir = f.goldens;
  WriteGoldens(&f, RegisteredGoldens(f));
  if (run_golden_step) {
    // what tools/gates/us026-mse-unit-v1/golden.mjs does through sr_held.mjs (kind golden, with the input tree)
    sh::Request rq;
    rq.prereg_dir = f.prereg;
    rq.gate_id = "g1";
    rq.version = 1;
    rq.kind = "golden";
    rq.env_value = f.in.env_value;
    rq.pins = {f.exe};
    rq.data_dir = f.inputs;
    rq.want_seed = true;
    const sh::Outcome g = sh::HeldGate(rq);
    EXPECT_TRUE(g.ok) << g.code << ": " << g.why;
  }
  return f;
}

TEST(Us026Integrity, ReleaseStartsOnceAfterTheGoldenStep) {
  const ReleaseFixture f = MakeReleaseFixture("once");
  ASSERT_TRUE(Exists(f.golden_marker));
  const ReleaseResult first = StartRelease(f.in);
  EXPECT_TRUE(first.ok) << first.code << ": " << first.reason;
  EXPECT_TRUE(Exists(f.judge_marker));
  for (const std::string& l : first.log) std::printf("[US026 release] %s\n", l.c_str());
  const ReleaseResult second = StartRelease(f.in);
  EXPECT_FALSE(second.ok);
  EXPECT_EQ(second.code, "MARKER_EXISTS");
  std::printf("[US026 release] second start: %s\n", second.code.c_str());
  // the golden step is one-shot as well
  sh::Request rq;
  rq.prereg_dir = f.prereg;
  rq.gate_id = "g1";
  rq.version = 1;
  rq.kind = "golden";
  rq.env_value = f.in.env_value;
  rq.pins = {f.exe};
  rq.data_dir = f.inputs;
  rq.want_seed = true;
  const sh::Outcome again = sh::HeldGate(rq);
  EXPECT_EQ(again.code, "MARKER_EXISTS");
  std::printf("[US026 release] second golden step: %s\n", again.code.c_str());
}

TEST(Us026Integrity, GoldenRefusalsLeaveNoJudgeMarker) {
  struct Case {
    const char* name;
    const char* code;
    std::function<void(ReleaseFixture*)> damage;
  };
  const Case cases[] = {
      {"golden step not run (no golden marker)", "GOLDEN", [](ReleaseFixture* f) { std::filesystem::remove(f->golden_marker); }},
      {"golden with cpp_param_overrides (mutation copy)", "GOLDEN",
       [](ReleaseFixture* f) {
         auto g = RegisteredGoldens(*f);
         g["d1-c2-default.methods.json"] =
             MethodsGoldenText(*f, false, "\"cpp_param_overrides\":[[\"MetricScaleEstimator.imuGaussianSigma\",\"f\",\"3f800000\"]],");
         WriteGoldens(f, g);
       }},
      {"an unregistered extra golden", "GOLDEN",
       [](ReleaseFixture* f) {
         auto g = RegisteredGoldens(*f);
         g["extra.methods.json"] = MethodsGoldenText(*f, false);
         WriteGoldens(f, g);
       }},
      {"a registered golden missing", "GOLDEN",
       [](ReleaseFixture* f) {
         auto g = RegisteredGoldens(*f);
         g.erase("d1-c2-reest.methods.json");
         WriteGoldens(f, g);
       }},
      {"golden edited after GOLDEN-SHA256SUMS", "GOLDEN",
       [](ReleaseFixture* f) { PutFile(f->goldens + "/registration.kf6.json", Kf6GoldenText(*f) + " "); }},
      {"golden from another dump", "GOLDEN",
       [](ReleaseFixture* f) {
         auto g = RegisteredGoldens(*f);
         ReleaseFixture other = *f;
         other.dump_sha = std::string(64, 'd');
         g["d1-c2-default.methods.json"] = MethodsGoldenText(other, false);
         WriteGoldens(f, g);
       }},
      {"golden from another generator script", "GOLDEN",
       [](ReleaseFixture* f) {
         auto g = RegisteredGoldens(*f);
         ReleaseFixture other = *f;
         other.mmg_sha = std::string(64, 'e');
         g["d1-c2-default.methods.json"] = MethodsGoldenText(other, false);
         WriteGoldens(f, g);
       }},
      {"default-setting golden is a re-estimation run", "GOLDEN",
       [](ReleaseFixture* f) {
         auto g = RegisteredGoldens(*f);
         g["d1-c2-default.methods.json"] = MethodsGoldenText(*f, true);
         WriteGoldens(f, g);
       }},
      {"kf6 golden from a dev seed class", "GOLDEN",
       [](ReleaseFixture* f) {
         auto g = RegisteredGoldens(*f);
         g["registration.kf6.json"] = Kf6GoldenText(*f, "dev");
         WriteGoldens(f, g);
       }},
      {"kf6 golden from another seed file", "GOLDEN",
       [](ReleaseFixture* f) {
         auto g = RegisteredGoldens(*f);
         ReleaseFixture other = *f;
         other.seed_sha = std::string(64, 'f');
         g["registration.kf6.json"] = Kf6GoldenText(other);
         WriteGoldens(f, g);
       }},
      {"golden of another original module", "GOLDEN",
       [](ReleaseFixture* f) {
         auto g = RegisteredGoldens(*f);
         ReleaseFixture other = *f;
         other.wasm_sha = std::string(64, '1');
         g["d1-c2-reest.methods.json"] = MethodsGoldenText(other, true);
         WriteGoldens(f, g);
       }},
      {"golden marker of another designation", "GOLDEN",
       [](ReleaseFixture* f) {
         std::filesystem::remove(f->golden_marker);
         PutFile(f->golden_marker, "{\"gate\":\"g1.v1\",\"kind\":\"golden\",\"designation_sha256\":\"" + std::string(64, '9') +
                                       "\",\"pid\":1}\n");
       }},
  };
  int n = 0;
  for (const Case& c : cases) {
    ReleaseFixture f = MakeReleaseFixture("golden" + std::to_string(n++));
    c.damage(&f);
    const ReleaseResult refused = StartRelease(f.in);
    EXPECT_FALSE(refused.ok) << c.name;
    EXPECT_EQ(refused.code, c.code) << c.name << ": " << refused.reason;
    EXPECT_FALSE(Exists(f.judge_marker)) << c.name << ": the refusal created the judge marker";
    std::printf("[US026 release] %-52s %s (%s)\n", c.name, refused.code.c_str(), refused.reason.c_str());
  }
}

TEST(Us026Integrity, DeviceRefusalsLeaveNoJudgeMarker) {
  struct Case {
    const char* name;
    const char* code;
    std::function<void(ReleaseFixture*)> damage;
  };
  const Case cases[] = {
      {"environment value not set", "ENV_MISSING", [](ReleaseFixture* f) { f->in.env_value = ""; }},
      {"environment value is any other string", "ENV_MISMATCH", [](ReleaseFixture* f) { f->in.env_value = "x"; }},
      {"gate not designated as <gate_id>.v<N>", "ARGS", [](ReleaseFixture* f) { f->in.gate_name = ""; }},
      {"another gate designated", "MANIFEST_UNREADABLE", [](ReleaseFixture* f) { f->in.gate_name = "g1.v99"; }},
      {"no golden directory", "ARGS", [](ReleaseFixture* f) { f->in.golden_dir = f->root + "/nowhere"; }},
      {"the held binary differs from the registered one", "PIN_MISMATCH", [](ReleaseFixture* f) { PutFile(f->exe, "rebuilt"); }},
      {"an unregistered extra pin presented", "PIN_UNREGISTERED",
       [](ReleaseFixture* f) {
         PutFile(f->root + "/other.bin", "x");
         f->in.pins.push_back(f->root + "/other.bin");
       }},
      {"a registered pin not presented", "PIN_NOT_PRESENTED", [](ReleaseFixture* f) { f->in.pins.clear(); }},
      {"coordinator seed file missing", "SEED_MISSING",
       [](ReleaseFixture* f) { std::filesystem::remove(f->prereg + "/g1.v1.coordinator-seed.txt"); }},
      {"coordinator seed replaced by the dev seed", "SEED_FORBIDDEN",
       [](ReleaseFixture* f) { PutFile(f->prereg + "/g1.v1.coordinator-seed.txt", "26001\n"); }},
  };
  int n = 0;
  for (const Case& c : cases) {
    ReleaseFixture f = MakeReleaseFixture("device" + std::to_string(n++));
    c.damage(&f);
    const ReleaseResult refused = StartRelease(f.in);
    EXPECT_FALSE(refused.ok) << c.name;
    EXPECT_EQ(refused.code, c.code) << c.name << ": " << refused.reason;
    EXPECT_FALSE(Exists(f.judge_marker)) << c.name << ": the refusal created the judge marker";
    std::printf("[US026 release] %-52s %s\n", c.name, refused.code.c_str());
  }
}

// The golden step refuses a changed input tree before its marker (kind golden): the dumps are pinned.
TEST(Us026Integrity, GoldenStepRefusesChangedDumps) {
  ReleaseFixture f = MakeReleaseFixture("dumps", false);
  PutFile(f.inputs + "/d1.dump.jsonl", "an edited dump\n");
  sh::Request rq;
  rq.prereg_dir = f.prereg;
  rq.gate_id = "g1";
  rq.version = 1;
  rq.kind = "golden";
  rq.env_value = f.in.env_value;
  rq.pins = {f.exe};
  rq.data_dir = f.inputs;
  rq.want_seed = true;
  const sh::Outcome g = sh::HeldGate(rq);
  EXPECT_FALSE(g.ok);
  EXPECT_EQ(g.code, "INPUT_MISMATCH");
  EXPECT_FALSE(Exists(f.golden_marker));
  std::printf("[US026 release] golden step with an edited dump: %s, golden marker %s\n", g.code.c_str(),
              Exists(f.golden_marker) ? "CREATED" : "absent");
}

TEST(Us026Integrity, SelfExecutableIsReadable) {
  std::string sha;
  ASSERT_TRUE(sh::Sha256File(sh::SelfExe(), &sha));
  std::printf("[US026 self executable] sha256=%s\n", sha.c_str());
}
#endif  // SR_US026_HELD

}  // namespace
}  // namespace slam_refact::metric_scale

// US-014 curvy ROI draw gates (gate file preregistration/curvy-it-gate.v<N>.md, N from SLAM_REFACT_CURVY_GATE_VERSION):
// the CPU ROI draw of a CURVY_IMAGE_TARGET (frontend::DrawRoisIntoAtlas via
// PyramidSource::GetWithRois) against the independent numpy oracle
// slam_refact/tools/image_target/curvy_reference.py.
//   G-CROI-A  slot vs the numpy shader model (gr8-stage1.frag getCurvyLiftedUv written out in numpy),
//             4 cases: cylinder, cone, landscape target (swap + vertex rotation), post rotation 90.
//   G-CROI-B  slot vs the geometric rule (the target texture where the surface faces the camera and
//             projects into the image; no shader formula) on 6 poses x {cylinder, cone}.
// Also: null-runs N1 (constant camera), N3 (+1 level), N4 / N5 (numpy only), the 3x resolution rule,
// mutation detection by input (geometry parameters) and, in the mutant executables built from
// copies of roi_draw.cc (CMake, label mutation, env SLAM_REFACT_CURVY_EXPECT_MUTANT=1), by draw bugs,
// a sensitivity REPORT, and the equality of the exported curvy uniforms with the CPU draw.
//
// DATA SEPARATION (v3, review B-1): every engine run before the independent review uses the DEV variant only
// (the exposed data); the registered variant (derived from the coordinator seed) is engine input only in the
// gate test GatesAB of the ordinary executable, behind the shared HELD device (curvy_sr_held.h over support/sr_held.h).
// Each test asserts the variant of the data it uses (variant.txt of the oracle), and with a gate selected (environment, no
// default) a dev directory that is the registered data is refused.
//
// Paths: SLAM_REFACT_PYTHON, SLAM_REFACT_CURVY_REFERENCE (curvy_reference.py), SLAM_REFACT_CURVY_DIR
// (work directory), defaulting to the CMake compile definitions. Missing paths fail the test
// (BLOCKED, not skipped). SLAM_REFACT_CURVY_VARIANT=dev runs the gate test GatesAB of the ordinary executable on the dev
// data (a control of the code, never a gate result).
// Synthetic data: not a device accuracy (CLAUDE.md section 3).

#include <gtest/gtest.h>
#include <unistd.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <functional>
#include <map>
#include <sstream>
#include <string>
#include <vector>

#include "c8/hmatrix.h"
#include "c8/pixels/image-roi.h"
#include "common/c8_adapters.h"
#include "frontend/pyramid_source.h"
#include "frontend/roi_draw.h"
#include "curvy_sr_held.h"
#include "image_target/target_files.h"

namespace slam_refact {
namespace {

using image_target::GrayImage;

// Registered tolerances: the values of the gate file of the selected version (preregistration/curvy-it-gate.v<N>.md, N =
// SLAM_REFACT_CURVY_GATE_VERSION), fixed before the first real run and never changed after a result was seen.
constexpr double kA1CoverageFraction = 0.002;
constexpr int kA2MaxAbs = 3;
constexpr double kA3MeanAbs = 0.1;
constexpr double kB1Rms = 0.15;
constexpr double kB2CoverageFraction = 0.002;
constexpr int kBPoses = 6;
constexpr int kACases = 4;

std::string Env(const char* name, const char* fallback = "") {
  const char* v = std::getenv(name);
  return v == nullptr ? std::string(fallback) : std::string(v);
}

#ifndef SR_CURVY_PYTHON
#define SR_CURVY_PYTHON ""
#define SR_CURVY_REFERENCE ""
#define SR_CURVY_DIR ""
#endif

// Mutant executables (CMake, SR_CURVY_MUTANT_BUILD) are built from a copy of roi_draw.cc with a draw bug and are
// expected to violate the gates (env SLAM_REFACT_CURVY_EXPECT_MUTANT=1 inverts the assertion). They run on the dev
// variant only. The ordinary executable ignores that variable.
bool ExpectMutant() {
#ifdef SR_CURVY_MUTANT_BUILD
  return std::getenv("SLAM_REFACT_CURVY_EXPECT_MUTANT") != nullptr;
#else
  return false;
#endif
}

// The data variant of the gate test: registered (default) for the ordinary executable, dev for the mutants (and for the
// ordinary executable when SLAM_REFACT_CURVY_VARIANT=dev).
std::string GateVariant() {
#ifdef SR_CURVY_MUTANT_BUILD
  return "dev";
#else
  const char* v = std::getenv("SLAM_REFACT_CURVY_VARIANT");
  return v != nullptr && std::string(v) == "dev" ? "dev" : "registered";
#endif
}

// "" when the engine may run on `dir` (the data of `variant`): dev data must not be sealed gate data (checked when a gate is
// selected, curvy_sr_held.h); registered data needs the full release of the shared HELD device (and claims the one-run marker
// of `kind`). The gate id, the version and the preregistration directory come from the environment (no default).
std::string DataGuard(const std::string& variant, const std::string& dir, const std::string& kind) {
  const curvy_gate_held::Selection sel = curvy_gate_held::SelectFromEnv();
  std::vector<std::string> log;
  const std::string why = variant == "dev" ? curvy_gate_held::DevGuard(sel, dir, &log)
                                           : curvy_gate_held::GateGuard(sel, kind, dir, &log);
  curvy_gate_held::Print(log);
  return why;
}

GrayImage ReadPgm(const std::string& path) {
  std::string bytes, error;
  GrayImage img;
  EXPECT_TRUE(image_target::ReadFileBytes(path, &bytes, &error)) << path << ": " << error;
  EXPECT_TRUE(image_target::ParsePgm(bytes, &img, &error)) << path << ": " << error;
  return img;
}

std::vector<double> ReadNumbers(const std::string& path, size_t line, size_t count) {
  std::ifstream f(path);
  std::string text;
  for (size_t i = 0; i <= line && std::getline(f, text); ++i) {
  }
  std::istringstream is(text);
  std::vector<double> v(count, 0.0);
  for (double& x : v) is >> x;
  return v;
}

// One curvy ROI: geometry of the (cropped) target, K, camera -> model pose, engine rotation.
struct Case {
  double radius = 0, radius_bottom = 0, rows = 0, cols = 0, left = 0, right = 0;
  int rotation_deg = 0;
  double k[4] = {0, 0, 0, 0};
  double r[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
  double t[3] = {0, 0, 0};
};

Case ReadCaseA(const std::string& path) {
  Case c;
  const std::vector<double> g = ReadNumbers(path, 0, 7);
  c.radius = g[0];
  c.radius_bottom = g[1];
  c.rows = g[2];
  c.cols = g[3];
  c.left = g[4];
  c.right = g[5];
  c.rotation_deg = static_cast<int>(g[6]);
  const std::vector<double> k = ReadNumbers(path, 1, 4);
  for (int i = 0; i < 4; ++i) c.k[i] = k[static_cast<size_t>(i)];
  const std::vector<double> p = ReadNumbers(path, 2, 12);
  for (int i = 0; i < 9; ++i) c.r[i] = p[static_cast<size_t>(i)];
  for (int i = 0; i < 3; ++i) c.t[i] = p[static_cast<size_t>(9 + i)];
  return c;
}

c8::ImageRoi MakeRoi(const Case& c) {
  c8::ImageRoi roi;
  roi.source = c8::ImageRoi::Source::CURVY_IMAGE_TARGET;
  roi.faceId = 0;
  roi.name = "curvy-roi-test";
  roi.warp = c8::HMatrixGen::i();
  roi.geom.radius = static_cast<float>(c.radius);
  roi.geom.radiusBottom = static_cast<float>(c.radius_bottom);
  roi.geom.height = 1.0f;
  roi.geom.activationRegion = {static_cast<float>(c.left), static_cast<float>(c.right), 0.f, 1.f};
  roi.geom.srcRows = static_cast<int>(c.rows);
  roi.geom.srcCols = static_cast<int>(c.cols);
  roi.geom.isCone = std::fabs(c.radius - c.radius_bottom) > 1e-6;
  roi.intrinsics = c8::HMatrixGen::intrinsic(ToC8Intrinsics(PinholeIntrinsics{
      480, 640, static_cast<float>(c.k[2]), static_cast<float>(c.k[3]), static_cast<float>(c.k[0]),
      static_cast<float>(c.k[1])}));
  const auto f = [](double v) { return static_cast<float>(v); };
  roi.globalPose = c8::HMatrix{{f(c.r[0]), f(c.r[1]), f(c.r[2]), f(c.t[0])},
                               {f(c.r[3]), f(c.r[4]), f(c.r[5]), f(c.t[1])},
                               {f(c.r[6]), f(c.r[7]), f(c.r[8]), f(c.t[2])},
                               {0.f, 0.f, 0.f, 1.f}};
  return roi;
}

struct Slots {
  int rotation_deg;
  frontend::PyramidSource source;
  explicit Slots(int rotation)
      : rotation_deg(rotation),
        source(480, 640, frontend::ShaderParams{}, ShaderNumericModel::kIeee, rotation) {}
  int w() const { return source.layout().roi_slots[0].w; }
  int h() const { return source.layout().roi_slots[0].h; }
};

// Draws `roi` into slot 0 of a fresh pyramid built from `camera` and returns the slot's R plane.
GrayImage DrawSlot(const GrayImage& camera, const c8::ImageRoi& roi, int rotation_deg) {
  Slots s(rotation_deg);
  s.source.GetWithRois(PyramidAtlasView{}, RgbaImageView{}, camera.view(), {roi});
  const frontend::RgbaAtlas& atlas = s.source.last_cpu_atlas();
  const frontend::LevelLayout& l = s.source.layout().roi_slots[0];
  GrayImage g;
  g.width = l.w;
  g.height = l.h;
  g.pixels.resize(static_cast<size_t>(l.w) * l.h);
  for (int y = 0; y < l.h; ++y) {
    for (int x = 0; x < l.w; ++x) {
      g.pixels[static_cast<size_t>(y) * l.w + x] =
          atlas.rgba[(static_cast<size_t>(l.r + y) * atlas.size + (l.c + x)) * 4];
    }
  }
  return g;
}

struct ADiff {
  double coverage_fraction = 0.0;
  int max_abs = 0;
  double mean_abs = 0.0;
  size_t both = 0;
  bool Pass() const {
    return coverage_fraction <= kA1CoverageFraction && max_abs <= kA2MaxAbs && mean_abs <= kA3MeanAbs;
  }
};

ADiff CompareA(const GrayImage& actual, const GrayImage& expected, const GrayImage& mask) {
  EXPECT_EQ(actual.width, expected.width);
  EXPECT_EQ(actual.height, expected.height);
  ADiff d;
  size_t mismatch = 0, both = 0;
  double sum = 0.0;
  for (size_t i = 0; i < actual.pixels.size(); ++i) {
    const bool a = actual.pixels[i] > 0;
    const bool e = mask.pixels[i] > 0;
    if (a != e) ++mismatch;
    if (a && e) {
      const int diff = std::abs(static_cast<int>(actual.pixels[i]) - static_cast<int>(expected.pixels[i]));
      d.max_abs = std::max(d.max_abs, diff);
      sum += diff;
      ++both;
    }
  }
  d.coverage_fraction = static_cast<double>(mismatch) / static_cast<double>(actual.pixels.size());
  d.mean_abs = both == 0 ? 0.0 : sum / static_cast<double>(both);
  d.both = both;
  return d;
}

// e = RMS(actual - expected) / std(expected) over the mask (B1).
double NormalisedRms(const GrayImage& actual, const GrayImage& expected, const GrayImage& mask) {
  double sum = 0.0, sq = 0.0, n = 0.0, diff2 = 0.0;
  for (size_t i = 0; i < actual.pixels.size(); ++i) {
    if (mask.pixels[i] == 0) continue;
    const double e = expected.pixels[i];
    const double d = static_cast<double>(actual.pixels[i]) - e;
    sum += e;
    sq += e * e;
    diff2 += d * d;
    n += 1.0;
  }
  if (n == 0.0) return 1e9;
  const double mean = sum / n;
  const double var = std::max(sq / n - mean * mean, 1e-12);
  return std::sqrt(diff2 / n) / std::sqrt(var);
}

// Coverage mismatch fraction between a drawn slot and a 255-mask (B2).
double CoverageMismatch(const GrayImage& actual, const GrayImage& cover) {
  size_t mismatch = 0;
  for (size_t i = 0; i < actual.pixels.size(); ++i) {
    if ((actual.pixels[i] > 0) != (cover.pixels[i] > 0)) ++mismatch;
  }
  return static_cast<double>(mismatch) / static_cast<double>(actual.pixels.size());
}

struct BResult {
  double e = 0.0;
  double coverage = 0.0;
  bool Pass() const { return e <= kB1Rms && coverage <= kB2CoverageFraction; }
};

using CaseMutation = std::function<void(Case*)>;

class CurvyRoiOracle : public ::testing::Test {
 protected:
  static void SetUpTestSuite() {
    python_ = Env("SLAM_REFACT_PYTHON", SR_CURVY_PYTHON);
    script_ = Env("SLAM_REFACT_CURVY_REFERENCE", SR_CURVY_REFERENCE);
    base_ = Env("SLAM_REFACT_CURVY_DIR", SR_CURVY_DIR);
  }

  // The oracle files of `kind` for `variant` (numpy only, no engine); asserts the variant recorded in variant.txt and
  // returns the directory, or "" after the DataGuard refused it (the failure is reported).
  static std::string Prepare(const std::string& kind, const std::string& variant) {
    EXPECT_FALSE(python_.empty() || script_.empty() || base_.empty())
        << "SLAM_REFACT_PYTHON / SLAM_REFACT_CURVY_REFERENCE / SLAM_REFACT_CURVY_DIR not set (BLOCKED)";
    const std::string key = kind + "/" + variant;
    const auto it = dirs_.find(key);
    if (it != dirs_.end()) return it->second;
    const std::string dir = base_ + "/" + std::to_string(::getpid()) + "-" + kind + "-" + variant;  // ctest runs in parallel
    Slots s(0);
    const std::string cmd = "\"" + python_ + "\" -I \"" + script_ + "\" prepare --out \"" + dir +
                            "\" --slot-w " + std::to_string(s.w()) + " --slot-h " +
                            std::to_string(s.h()) + " --camera-kind " + kind + " --variant " + variant + " > \"" + dir +
                            ".log\" 2>&1";
    EXPECT_EQ(std::system(("mkdir -p \"" + base_ + "\"").c_str()), 0);
    EXPECT_EQ(std::system(cmd.c_str()), 0) << cmd;
    std::string recorded;
    sr_held::ReadFile(dir + "/variant.txt", &recorded);
    EXPECT_EQ(recorded, variant + "\n") << "the oracle data of " << dir << " is not the " << variant << " variant";
    const std::string held = DataGuard(variant, dir, "roi");
    if (!held.empty() || recorded != variant + "\n") {
      ADD_FAILURE() << held;
      dirs_[key] = "";
      return "";
    }
    dirs_[key] = dir;
    return dir;
  }

  // G-CROI-A on `kind` ("textured" real gate, "constant" null-run N1).
  static std::vector<ADiff> GateA(const std::string& kind, const std::string& variant) {
    const std::string dir = Prepare(kind, variant);
    if (dir.empty()) return {};
    const GrayImage camera = ReadPgm(dir + "/a_camera.pgm");
    std::vector<ADiff> out;
    for (int i = 0; i < kACases; ++i) {
      const Case c = ReadCaseA(dir + "/a_case_" + std::to_string(i) + ".txt");
      const GrayImage actual = DrawSlot(camera, MakeRoi(c), c.rotation_deg);
      out.push_back(CompareA(actual, ReadPgm(dir + "/a_expected_" + std::to_string(i) + ".pgm"),
                             ReadPgm(dir + "/a_mask_" + std::to_string(i) + ".pgm")));
    }
    return out;
  }

  // G-CROI-B of one shape; `mutation` edits the geometry given to the draw (the oracle files stay).
  static std::vector<BResult> GateB(const std::string& shape, const CaseMutation* mutation, const std::string& variant) {
    const std::string dir = Prepare("textured", variant);
    if (dir.empty()) return {};
    const std::vector<double> g = ReadNumbers(dir + "/b_" + shape + "_geom.txt", 0, 6);
    std::vector<BResult> out;
    for (int i = 0; i < kBPoses; ++i) {
      Case c;
      c.radius = g[0];
      c.radius_bottom = g[1];
      c.left = g[2];
      c.right = g[3];
      c.cols = g[4];
      c.rows = g[5];
      const double kk[4] = {506.4104, 506.4104, 239.5, 319.5};
      for (int j = 0; j < 4; ++j) c.k[j] = kk[j];
      const std::vector<double> p = ReadNumbers(dir + "/b_" + shape + "_pose_" + std::to_string(i) + ".txt", 0, 12);
      for (int j = 0; j < 9; ++j) c.r[j] = p[static_cast<size_t>(j)];
      for (int j = 0; j < 3; ++j) c.t[j] = p[static_cast<size_t>(9 + j)];
      if (mutation != nullptr) (*mutation)(&c);
      const GrayImage actual = DrawSlot(ReadPgm(dir + "/b_" + shape + "_camera_" + std::to_string(i) + ".pgm"),
                                        MakeRoi(c), 0);
      BResult r;
      r.e = NormalisedRms(actual, ReadPgm(dir + "/b_" + shape + "_expected_" + std::to_string(i) + ".pgm"),
                          ReadPgm(dir + "/b_" + shape + "_mask_" + std::to_string(i) + ".pgm"));
      r.coverage = CoverageMismatch(actual, ReadPgm(dir + "/b_" + shape + "_cover_" + std::to_string(i) + ".pgm"));
      out.push_back(r);
    }
    return out;
  }

  static std::string python_, script_, base_;
  static std::map<std::string, std::string> dirs_;
};
std::string CurvyRoiOracle::python_, CurvyRoiOracle::script_, CurvyRoiOracle::base_;
std::map<std::string, std::string> CurvyRoiOracle::dirs_;

void PrintA(const char* tag, const std::vector<ADiff>& d) {
  for (size_t i = 0; i < d.size(); ++i) {
    std::printf("[CURVY %s] case %zu: coverage mismatch %.6f, max abs %d, mean abs %.6f (n %zu) %s\n", tag,
                i, d[i].coverage_fraction, d[i].max_abs, d[i].mean_abs, d[i].both,
                d[i].Pass() ? "PASS" : "VIOLATION");
  }
}

void PrintB(const char* tag, const std::string& shape, const std::vector<BResult>& r) {
  for (size_t i = 0; i < r.size(); ++i) {
    std::printf("[CURVY %s] %s pose %zu: e %.5f (tol %.2f), coverage mismatch %.6f (tol %.4f) %s\n", tag,
                shape.c_str(), i, r[i].e, kB1Rms, r[i].coverage, kB2CoverageFraction,
                r[i].Pass() ? "PASS" : "VIOLATION");
  }
}

// The gates G-CROI-A and G-CROI-B; in a mutant executable (draw bug in a copy of roi_draw.cc) at least
// one of them must be violated instead.
TEST_F(CurvyRoiOracle, GatesAB) {
  const std::string variant = GateVariant();  // registered (HELD device) unless dev / a mutant executable
  size_t violations = 0;
  const std::vector<ADiff> a = GateA("textured", variant);
  ASSERT_FALSE(a.empty()) << "no oracle data (the HELD device or the data variant refused the run)";
  PrintA("G-CROI-A", a);
  for (const ADiff& x : a) violations += x.Pass() ? 0 : 1;
  for (const char* shape : {"cylinder", "cone"}) {
    const std::vector<BResult> b = GateB(shape, nullptr, variant);
    PrintB("G-CROI-B", shape, b);
    for (const BResult& x : b) violations += x.Pass() ? 0 : 1;
  }
  std::printf("[CURVY GatesAB] violations %zu\n", violations);
  if (ExpectMutant()) {
    EXPECT_GT(violations, 0u) << "mutant not detected: G-CROI-A and G-CROI-B both hold";
  } else {
    EXPECT_EQ(violations, 0u);
  }
}

// N1: constant camera. Every covered pixel must be exactly the camera value and the coverage must
// match the numpy shader model (null-run: measurement noise floor).
TEST_F(CurvyRoiOracle, NullRunN1ConstantCamera) {
  const std::vector<ADiff> d = GateA("constant", "dev");  // dev data only (v3 review B-1)
  ASSERT_EQ(d.size(), static_cast<size_t>(kACases));
  PrintA("N1 constant", d);
  for (const ADiff& x : d) {
    EXPECT_EQ(x.max_abs, 0);
    EXPECT_EQ(x.mean_abs, 0.0);
    EXPECT_LE(x.coverage_fraction, kA1CoverageFraction);
  }
}

double ParseKey(const std::string& text, const std::string& key) {
  const size_t at = text.find(key + "=");
  if (at == std::string::npos) return -1.0;
  return std::atof(text.c_str() + at + key.size() + 1);
}

// N3 (+1 level in e), N4 / N5 (numpy only, curvy_reference.py selftest; run for BOTH variants when the coordinator seed file is
// given, because the tolerance must hold for the registered poses too, which needs no engine) and the 3 x resolution rule.
TEST_F(CurvyRoiOracle, NullRunsN3N4N5ResolutionRule) {
  const std::string dir = Prepare("textured", "dev");
  ASSERT_FALSE(dir.empty());
  const GrayImage expected = ReadPgm(dir + "/b_cylinder_expected_0.pgm");
  const GrayImage mask = ReadPgm(dir + "/b_cylinder_mask_0.pgm");
  GrayImage plus1 = expected;
  for (uint8_t& p : plus1.pixels) p = static_cast<uint8_t>(std::min(255, p + 1));
  const double e0 = NormalisedRms(expected, expected, mask);
  const double e1 = NormalisedRms(plus1, expected, mask);
  Slots s(0);
  double n4_max = 0, n4_mean = 0, n4_cov = 0, n5_e = 0, n5_cov = 0;
  // The registered poses derive from the coordinator seed file, which has no default (SLAM_REFACT_CURVY_SEED_FILE, read by
  // curvy_reference.py). Without it only the dev poses are evaluated; the line below says so (the registered part is then
  // UNVERIFIED in this run, not passed). Which poses N5 should use is the open question F-2 of the v3 first-run review.
  std::vector<const char*> variants = {"dev"};
  if (!Env("SLAM_REFACT_CURVY_SEED_FILE").empty()) {
    variants.push_back("registered");
  } else {
    std::printf("[CURVY N4/N5] registered poses NOT evaluated: SLAM_REFACT_CURVY_SEED_FILE is not set "
                "(UNVERIFIED for the registered variant; the resolution rule below covers the dev poses only)\n");
  }
  for (const char* variant : variants) {
    const std::string out = base_ + "/selftest-" + variant + "-" + std::to_string(::getpid());
    const std::string cmd = "\"" + python_ + "\" -I \"" + script_ + "\" selftest --variant " + variant + " --out \"" + out +
                            "\" --slot-w " + std::to_string(s.w()) + " --slot-h " + std::to_string(s.h()) +
                            " > \"" + out + ".log\" 2>&1";
    ASSERT_EQ(std::system(cmd.c_str()), 0) << cmd;
    std::string text, error;
    ASSERT_TRUE(image_target::ReadFileBytes(out + "/selftest.txt", &text, &error)) << error;
    n4_max = std::max(n4_max, ParseKey(text, "a_max"));
    n4_mean = std::max(n4_mean, ParseKey(text, "a_mean"));
    n4_cov = std::max(n4_cov, ParseKey(text, "a_cov"));
    n5_e = std::max(n5_e, ParseKey(text, "b_e"));
    n5_cov = std::max(n5_cov, ParseKey(text, "b_cov"));
  }
  double n1_max = 0, n1_mean = 0, n1_cov = 0;
  size_t covered = 1u << 30;
  for (const ADiff& x : GateA("constant", "dev")) {
    n1_max = std::max<double>(n1_max, x.max_abs);
    n1_mean = std::max(n1_mean, x.mean_abs);
    n1_cov = std::max(n1_cov, x.coverage_fraction);
    covered = std::min(covered, x.both);
  }
  const double slot_area = static_cast<double>(s.w()) * s.h();
  const double res_a2 = std::max({1.0, n1_max, n4_max});
  const double res_a3 = std::max({n1_mean, n4_mean, 1.0 / static_cast<double>(covered)});
  const double res_a1 = std::max({n1_cov, n4_cov, 1.0 / slot_area});
  const double res_b1 = std::max(e1, n5_e);
  const double res_b2 = std::max({n5_cov, n1_cov, 1.0 / slot_area});
  std::printf("[CURVY N3/N4/N5] e(self) %.6f e(+1 level) %.6f | N4 numpy f32 vs f64: A2 %.0f A3 %.6f A1 %.6f |"
              " N5 oracle vs oracle: B1 %.6f B2 %.6f | N1: A2 %.0f A3 %.6f A1 %.6f | covered min %zu\n",
              e0, e1, n4_max, n4_mean, n4_cov, n5_e, n5_cov, n1_max, n1_mean, n1_cov, covered);
  std::printf("[CURVY resolution] A1 %.6f (tol %.4f) A2 %.1f (tol %d) A3 %.6f (tol %.2f) B1 %.6f (tol %.2f) "
              "B2 %.6f (tol %.4f)\n", res_a1, kA1CoverageFraction, res_a2, kA2MaxAbs, res_a3, kA3MeanAbs,
              res_b1, kB1Rms, res_b2, kB2CoverageFraction);
  EXPECT_EQ(e0, 0.0);
  EXPECT_GE(covered, 20000u);
  EXPECT_GE(kA1CoverageFraction, 3.0 * res_a1);  // protocol section 3: tolerance >= 3 x resolution
  EXPECT_GE(static_cast<double>(kA2MaxAbs), 3.0 * res_a2);
  EXPECT_GE(kA3MeanAbs, 3.0 * res_a3);
  EXPECT_GE(kB1Rms, 3.0 * res_b1);
  EXPECT_GE(kB2CoverageFraction, 3.0 * res_b2);
}

// Input mutations of the geometry given to the draw: each must violate G-CROI-B in at least one pose of
// the shape(s) it applies to (the numpy expectation is unchanged).
TEST_F(CurvyRoiOracle, GeometryMutationsAreDetected) {
  struct Mutation {
    const char* name;
    const char* shape;
    CaseMutation apply;
  };
  const std::vector<Mutation> mutations = {
      {"MC radius x1.3", "cylinder", [](Case* c) { c->radius *= 1.3; c->radius_bottom *= 1.3; }},
      {"MC radius x1.3", "cone", [](Case* c) { c->radius *= 1.3; c->radius_bottom *= 1.3; }},
      {"MT top/bottom radius swapped", "cone", [](Case* c) { std::swap(c->radius, c->radius_bottom); }},
      {"MA arc x0.8", "cylinder", [](Case* c) { c->right = c->left + (c->right - c->left) * 0.8; }},
      {"MX mirrored arc (left/right swapped)", "cylinder", [](Case* c) { std::swap(c->left, c->right); }},
      {"MS radius sign", "cylinder", [](Case* c) { c->radius = -c->radius; c->radius_bottom = -c->radius_bottom; }},
      {"MS radius sign", "cone", [](Case* c) { c->radius = -c->radius; c->radius_bottom = -c->radius_bottom; }},
  };
  for (const Mutation& m : mutations) {
    const std::vector<BResult> r = GateB(m.shape, &m.apply, "dev");
    PrintB(m.name, m.shape, r);
    bool detected = false;
    for (const BResult& x : r) detected = detected || !x.Pass();
    EXPECT_TRUE(detected) << m.name << " (" << m.shape << "): G-CROI-B did not react";
  }
}

// REPORT (not a gate; dev data): how small a geometry error G-CROI-B sees (review L-8). The worst e and coverage mismatch
// over the 6 poses for a radius error of 2 / 5 / 10 / 20 % and an arc error of 5 / 10 %, against the tolerances.
TEST_F(CurvyRoiOracle, SensitivityReportOfTheGeometryGate) {
  for (const char* shape : {"cylinder", "cone"}) {
    for (const double f : {1.02, 1.05, 1.10, 1.20}) {
      const CaseMutation m = [f](Case* c) { c->radius *= f; c->radius_bottom *= f; };
      const std::vector<BResult> r = GateB(shape, &m, "dev");
      ASSERT_FALSE(r.empty());
      double e = 0, cov = 0;
      for (const BResult& x : r) {
        e = std::max(e, x.e);
        cov = std::max(cov, x.coverage);
      }
      double emin = 1e9;
      for (const BResult& x : r) emin = std::min(emin, x.e);
      std::printf("[CURVY sensitivity REPORT] %s radius x%.2f: e min %.4f max %.4f (tol %.2f), coverage max %.5f (tol %.4f) -> %s\n", shape,
                  f, emin, e, kB1Rms, cov, kB2CoverageFraction, (e > kB1Rms || cov > kB2CoverageFraction) ? "seen in at least one pose" : "NOT seen");
    }
    for (const double f : {0.95, 0.90}) {
      const CaseMutation m = [f](Case* c) { c->right = c->left + (c->right - c->left) * f; };
      const std::vector<BResult> r = GateB(shape, &m, "dev");
      ASSERT_FALSE(r.empty());
      double e = 0, cov = 0, emin = 1e9;
      for (const BResult& x : r) {
        e = std::max(e, x.e);
        cov = std::max(cov, x.coverage);
        emin = std::min(emin, x.e);
      }
      std::printf("[CURVY sensitivity REPORT] %s arc x%.2f: e min %.4f max %.4f (tol %.2f), coverage max %.5f (tol %.4f) -> %s\n", shape, f,
                  emin, e, kB1Rms, cov, kB2CoverageFraction, (e > kB1Rms || cov > kB2CoverageFraction) ? "seen in at least one pose" : "NOT seen");
    }
  }
}

RgbaImageView ToRgba(const GrayImage& g, std::vector<uint8_t>* store) {
  store->assign(static_cast<size_t>(g.width) * g.height * 4, 255);
  for (size_t i = 0; i < g.pixels.size(); ++i) (*store)[4 * i] = (*store)[4 * i + 1] = (*store)[4 * i + 2] = g.pixels[i];
  return RgbaImageView{store->data(), g.width, g.height, g.width * 4};
}

// Review L-4: a GPU producer draws a curvy slot from PyramidRoiRequest::curvy. The export must carry exactly the uniforms the CPU
// draw computes (bit for bit), and a slot redrawn from the exported uniforms must equal the CPU draw's slot (dev data).
TEST_F(CurvyRoiOracle, ExportedUniformsRedrawTheCpuSlotBitForBit) {
  const std::string dir = Prepare("textured", "dev");
  ASSERT_FALSE(dir.empty());
  const GrayImage camera = ReadPgm(dir + "/a_camera.pgm");
  std::vector<uint8_t> rgba_store;
  const RgbaImageView cam = ToRgba(camera, &rgba_store);
  for (int i = 0; i < kACases; ++i) {
    const Case c = ReadCaseA(dir + "/a_case_" + std::to_string(i) + ".txt");
    const c8::ImageRoi roi = MakeRoi(c);
    Slots s(c.rotation_deg);
    const frontend::PyramidLayout& layout = s.source.layout();
    const frontend::LevelLayout& slot = layout.roi_slots[0];
    const int rotation = slot.rotated ? c.rotation_deg - 90 : c.rotation_deg;
    const PyramidRoiCurvy exported = frontend::MakeRoiCurvyExport(roi, slot, layout.levels[0], rotation);
    ASSERT_TRUE(exported.is_curvy);
    const frontend::CurvyRoiUniforms direct = frontend::ComputeCurvyRoiUniforms(roi, slot, layout.levels[0], rotation);
    const frontend::CurvyRoiUniforms back = frontend::UniformsFromExport(exported);
    EXPECT_EQ(std::memcmp(&direct, &back, sizeof(direct)), 0) << "case " << i << ": export differs from the CPU uniforms";
    std::vector<uint8_t> atlas(static_cast<size_t>(layout.texture_size) * layout.texture_size * 4, 0);
    frontend::DrawCurvySlot(slot, roi.warp, back, ShaderNumericModel::kIeee, cam, layout.texture_size, atlas.data());
    const GrayImage cpu = DrawSlot(camera, roi, c.rotation_deg);
    size_t different = 0, lit = 0;
    for (int y = 0; y < slot.h; ++y) {
      for (int x = 0; x < slot.w; ++x) {
        const uint8_t a = atlas[(static_cast<size_t>(slot.r + y) * layout.texture_size + slot.c + x) * 4];
        const uint8_t b = cpu.pixels[static_cast<size_t>(y) * slot.w + x];
        different += a != b;
        lit += b != 0;
      }
    }
    std::printf("[CURVY export redraw] case %d: lit %zu, pixels differing from the CPU draw %zu\n", i, lit, different);
    EXPECT_GT(lit, 20000u);
    EXPECT_EQ(different, 0u) << "case " << i;
  }
}

// Review L-5: a curvy ROI with a non-identity warp must not abort (the original uploads mvp = roi.warp for both kinds of ROI):
// the quad moves like the planar draw's, a degenerate warp covers nothing.
TEST_F(CurvyRoiOracle, NonIdentityWarpMovesTheQuadAndDoesNotAbort) {
  const std::string dir = Prepare("textured", "dev");
  ASSERT_FALSE(dir.empty());
  const GrayImage camera = ReadPgm(dir + "/a_camera.pgm");
  std::vector<uint8_t> rgba_store;
  const RgbaImageView cam = ToRgba(camera, &rgba_store);
  const Case c = ReadCaseA(dir + "/a_case_0.txt");
  c8::ImageRoi roi = MakeRoi(c);
  Slots s(0);
  const frontend::PyramidLayout& layout = s.source.layout();
  const frontend::LevelLayout& slot = layout.roi_slots[0];
  const frontend::CurvyRoiUniforms u = frontend::ComputeCurvyRoiUniforms(roi, slot, layout.levels[0], 0);
  const auto lit_of = [&](const c8::HMatrix& warp) {
    std::vector<uint8_t> atlas(static_cast<size_t>(layout.texture_size) * layout.texture_size * 4, 0);
    frontend::DrawCurvySlot(slot, warp, u, ShaderNumericModel::kIeee, cam, layout.texture_size, atlas.data());
    size_t lit = 0;
    for (int y = 0; y < slot.h; ++y) {
      for (int x = 0; x < slot.w; ++x) lit += atlas[(static_cast<size_t>(slot.r + y) * layout.texture_size + slot.c + x) * 4] != 0;
    }
    return lit;
  };
  const size_t identity = lit_of(c8::HMatrixGen::i());
  const c8::HMatrix shifted{{1.f, 0.f, 0.f, 0.4f}, {0.f, 1.f, 0.f, 0.f}, {0.f, 0.f, 1.f, 0.f}, {0.f, 0.f, 0.f, 1.f}};
  const size_t moved = lit_of(shifted);
  const c8::HMatrix degenerate{{0.f, 0.f, 0.f, 0.f}, {0.f, 0.f, 0.f, 0.f}, {0.f, 0.f, 1.f, 0.f}, {0.f, 0.f, 0.f, 0.f}};
  std::printf("[CURVY warp] lit identity %zu, shifted by 0.4 NDC %zu, degenerate %zu\n", identity, moved, lit_of(degenerate));
  EXPECT_GT(identity, 20000u);
  EXPECT_GT(moved, 0u);
  EXPECT_LT(moved, identity);          // the quad left the slot on one side
  EXPECT_EQ(lit_of(degenerate), 0u);
}

}  // namespace
}  // namespace slam_refact

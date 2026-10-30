// RM-6 gates G-ROI-A and G-ROI-B (build/slam-refact-full-20261019/preregistration/rm6-it-roi-gate.v1.md):
// the CPU ROI draw (frontend::DrawRoisIntoAtlas via PyramidSource::GetWithRois) against the
// independent numpy oracle slam_refact/tools/image_target/roi_reference.py, the null-runs that
// measure the resolution, and mutation detection (a vertically flipped, zoomed or shifted ROI warp
// must violate the gate). Review M1 of opus-xhigh-r1-it-integration.md.
//
// Paths: SLAM_REFACT_PYTHON, SLAM_REFACT_ROI_REFERENCE (roi_reference.py), SLAM_REFACT_ROI_DIR (work
// directory), defaulting to the CMake compile definitions. Missing paths fail the test (BLOCKED, not skipped).

#include <gtest/gtest.h>
#include <unistd.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

#include "c8/hmatrix.h"
#include "c8/pixels/image-roi.h"
#include "common/c8_adapters.h"
#include "frontend/pyramid_source.h"
#include "frontend/roi_draw.h"
#include "image_target/target_files.h"
#include "roi_mutations.h"
#include "reality/engine/geometry/image-warp.h"

namespace slam_refact {
namespace {

using image_target::GrayImage;
using image_target::testing::Mutation;
using image_target::testing::Mutations;

// Registered tolerances (gate file; fixed before the first run).
constexpr double kA1CoverageFraction = 0.002;
constexpr int kA2MaxAbs = 3;
constexpr double kA3MeanAbs = 0.1;
constexpr double kB1Rms = 0.10;
constexpr int kBPoses = 6;

// Environment variable first (manual runs), else the CMake compile definition (ctest).
std::string Env(const char* name, const char* fallback = "") {
  const char* v = std::getenv(name);
  return v == nullptr ? std::string(fallback) : std::string(v);
}

#ifndef SR_RM6_PYTHON
#define SR_RM6_PYTHON ""
#define SR_RM6_ROI_REFERENCE ""
#define SR_RM6_ROI_DIR ""
#endif

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

c8::HMatrix FromRowMajor(const std::vector<double>& m) {
  const auto f = [&m](int i) { return static_cast<float>(m[static_cast<size_t>(i)]); };
  return c8::HMatrix{{f(0), f(1), f(2), f(3)},
                     {f(4), f(5), f(6), f(7)},
                     {f(8), f(9), f(10), f(11)},
                     {f(12), f(13), f(14), f(15)}};
}

c8::HMatrix FromPose(const std::vector<double>& p) {  // R row-major (9), t (3)
  const auto f = [&p](int i) { return static_cast<float>(p[static_cast<size_t>(i)]); };
  return c8::HMatrix{{f(0), f(1), f(2), f(9)},
                     {f(3), f(4), f(5), f(10)},
                     {f(6), f(7), f(8), f(11)},
                     {0.f, 0.f, 0.f, 1.f}};
}

struct Slots {
  frontend::PyramidSource source{480, 640, frontend::ShaderParams{}, ShaderNumericModel::kIeee};
  int w() const { return source.layout().roi_slots[0].w; }
  int h() const { return source.layout().roi_slots[0].h; }
};

// Draws `warps` into slots 0.. of a fresh pyramid built from `camera` and returns the R plane of
// every slot (slot w x h).
std::vector<GrayImage> DrawSlots(const GrayImage& camera, const std::vector<c8::HMatrix>& warps) {
  Slots s;
  std::vector<c8::ImageRoi> rois;
  for (size_t i = 0; i < warps.size(); ++i) {
    c8::ImageRoi r;
    r.source = c8::ImageRoi::Source::HIRES_SCAN;
    r.faceId = -static_cast<int>(i) - 1;
    r.name = "roi-test";
    r.warp = warps[i];
    rois.push_back(r);
  }
  s.source.GetWithRois(PyramidAtlasView{}, RgbaImageView{}, camera.view(), rois);
  const frontend::RgbaAtlas& atlas = s.source.last_cpu_atlas();
  std::vector<GrayImage> out;
  for (size_t i = 0; i < warps.size(); ++i) {
    const frontend::LevelLayout& l = s.source.layout().roi_slots[i];
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
    out.push_back(std::move(g));
  }
  return out;
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

// e = RMS(actual - expected) / std(expected) over the middle 80 % x 80 % (B1).
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
  const double mean = sum / n;
  const double var = std::max(sq / n - mean * mean, 1e-12);
  return std::sqrt(diff2 / n) / std::sqrt(var);
}

class RoiOracle : public ::testing::Test {
 protected:
  static void SetUpTestSuite() {
    python_ = Env("SLAM_REFACT_PYTHON", SR_RM6_PYTHON);
    script_ = Env("SLAM_REFACT_ROI_REFERENCE", SR_RM6_ROI_REFERENCE);
    base_ = Env("SLAM_REFACT_ROI_DIR", SR_RM6_ROI_DIR);
  }
  // Runs roi_reference.py prepare for a camera kind once; returns its directory.
  static std::string Prepare(const std::string& kind) {
    EXPECT_FALSE(python_.empty() || script_.empty() || base_.empty())
        << "SLAM_REFACT_PYTHON / SLAM_REFACT_ROI_REFERENCE / SLAM_REFACT_ROI_DIR not set (BLOCKED)";
    const auto it = dirs_.find(kind);
    if (it != dirs_.end()) return it->second;
    const std::string dir = base_ + "/" + std::to_string(::getpid()) + "-" + kind;  // ctest runs in parallel
    Slots s;
    const std::string cmd = "\"" + python_ + "\" -I \"" + script_ + "\" prepare --out \"" + dir +
                            "\" --slot-w " + std::to_string(s.w()) + " --slot-h " +
                            std::to_string(s.h()) + " --camera-kind " + kind + " > \"" + dir + ".log\" 2>&1";
    EXPECT_EQ(std::system(("mkdir -p \"" + base_ + "\"").c_str()), 0);
    EXPECT_EQ(std::system(cmd.c_str()), 0) << cmd;
    dirs_[kind] = dir;
    return dir;
  }

  static std::vector<c8::HMatrix> WarpsA(const std::string& dir) {
    std::vector<c8::HMatrix> w;
    for (size_t i = 0; i < 4; ++i) w.push_back(FromRowMajor(ReadNumbers(dir + "/a_warps.txt", i, 16)));
    return w;
  }

  // G-ROI-A on `kind` with the warps optionally transformed; returns the per-warp diffs.
  static std::vector<ADiff> GateA(const std::string& kind, const Mutation* mutation) {
    const std::string dir = Prepare(kind);
    std::vector<c8::HMatrix> warps = WarpsA(dir);
    if (mutation != nullptr) {
      for (c8::HMatrix& w : warps) w = mutation->apply(w);
    }
    const std::vector<GrayImage> actual = DrawSlots(ReadPgm(dir + "/a_camera.pgm"), warps);
    std::vector<ADiff> out;
    for (size_t i = 0; i < 4; ++i) {
      out.push_back(CompareA(actual[i], ReadPgm(dir + "/a_expected_" + std::to_string(i) + ".pgm"),
                             ReadPgm(dir + "/a_mask_" + std::to_string(i) + ".pgm")));
    }
    return out;
  }

  // G-ROI-B: e of the six poses (frontal, roll +8 / -8, registered trajectory k = 18, 32, 64).
  static std::vector<double> GateB(const Mutation* mutation) {
    const std::string dir = Prepare("textured");
    const c8_PixelPinholeCameraModel k =
        ToC8Intrinsics(PinholeIntrinsics{480, 640, 239.5f, 319.5f, 506.4104f, 506.4104f});
    const GrayImage expected = ReadPgm(dir + "/b_expected.pgm");
    const GrayImage mask = ReadPgm(dir + "/b_mask.pgm");
    std::vector<double> e;
    for (int i = 0; i < kBPoses; ++i) {
      const c8::HMatrix pose = FromPose(ReadNumbers(dir + "/b_pose_" + std::to_string(i) + ".txt", 0, 12));
      c8::HMatrix warp = c8::glImageTargetWarp(k, k, pose, c8::PlanarImageGeometry{});
      if (mutation != nullptr) warp = mutation->apply(warp);
      const std::vector<GrayImage> actual =
          DrawSlots(ReadPgm(dir + "/b_camera_" + std::to_string(i) + ".pgm"), {warp});
      e.push_back(NormalisedRms(actual[0], expected, mask));
    }
    return e;
  }

  static std::string python_, script_, base_;
  static std::map<std::string, std::string> dirs_;
};
std::string RoiOracle::python_, RoiOracle::script_, RoiOracle::base_;
std::map<std::string, std::string> RoiOracle::dirs_;

void Print(const char* tag, const std::vector<ADiff>& d) {
  for (size_t i = 0; i < d.size(); ++i) {
    std::printf("[RM6 %s] warp %zu: coverage mismatch %.5f, max abs %d, mean abs %.5f (n %zu) %s\n",
                tag, i, d[i].coverage_fraction, d[i].max_abs, d[i].mean_abs, d[i].both,
                d[i].Pass() ? "PASS" : "VIOLATION");
  }
}

// N1: constant camera (null-run, measurement noise floor; expected exactly 0).
TEST_F(RoiOracle, NullRunN1ConstantCamera) {
  const std::vector<ADiff> d = GateA("constant", nullptr);
  Print("N1 constant", d);
  for (const ADiff& x : d) {
    EXPECT_EQ(x.max_abs, 0);
    EXPECT_EQ(x.coverage_fraction, 0.0);
    EXPECT_EQ(x.mean_abs, 0.0);
  }
}

// I2: implementation check on a 2D ramp camera. NOT a null-run: any difference above the tolerances
// is an implementation failure, not a resolution (gate file v2 section 4).
TEST_F(RoiOracle, ImplementationCheckI2RampCamera) {
  const std::vector<ADiff> d = GateA("ramp", nullptr);
  Print("I2 ramp", d);
  for (const ADiff& x : d) EXPECT_TRUE(x.Pass());
}

double ParseKey(const std::string& text, const std::string& key) {
  const size_t at = text.find(key + "=");
  if (at == std::string::npos) return -1.0;
  return std::atof(text.c_str() + at + key.size() + 1);
}

// N3: B1 of the expected image against itself and against itself + 1 level (resolution in e);
// N4: the oracle's own float32 versus float64 noise; the 3 x resolution rule of the protocol.
TEST_F(RoiOracle, NullRunsN3N4ResolutionRule) {
  const std::string dir = Prepare("textured");
  const GrayImage expected = ReadPgm(dir + "/b_expected.pgm");
  const GrayImage mask = ReadPgm(dir + "/b_mask.pgm");
  GrayImage plus1 = expected;
  for (uint8_t& p : plus1.pixels) p = static_cast<uint8_t>(std::min(255, p + 1));  // constant +1 level
  const double e0 = NormalisedRms(expected, expected, mask);
  const double e1 = NormalisedRms(plus1, expected, mask);
  Slots s;
  const std::string cmd = "\"" + python_ + "\" -I \"" + script_ + "\" selftest --out \"" + base_ +
                          "/selftest-" + std::to_string(::getpid()) + "\" --slot-w " + std::to_string(s.w()) + " --slot-h " +
                          std::to_string(s.h()) + " > \"" + base_ + "/selftest.log\" 2>&1";
  ASSERT_EQ(std::system(cmd.c_str()), 0) << cmd;
  std::string text, error;
  ASSERT_TRUE(image_target::ReadFileBytes(base_ + "/selftest-" + std::to_string(::getpid()) + "/selftest.txt", &text, &error)) << error;
  const double n4_max = ParseKey(text, "a_max"), n4_mean = ParseKey(text, "a_mean"),
               n4_cov = ParseKey(text, "a_cov"), n4_e = ParseKey(text, "b_e");
  // N1 (constant camera) worst values.
  double n1_max = 0, n1_mean = 0, n1_cov = 0;
  size_t covered = 1u << 30;
  for (const ADiff& x : GateA("constant", nullptr)) {
    n1_max = std::max<double>(n1_max, x.max_abs);
    n1_mean = std::max(n1_mean, x.mean_abs);
    n1_cov = std::max(n1_cov, x.coverage_fraction);
    covered = std::min(covered, x.both);
  }
  const double res_a2 = std::max({1.0, n1_max, n4_max});
  const double res_a3 = std::max({n1_mean, n4_mean, 1.0 / static_cast<double>(covered)});
  const double res_a1 = std::max(n1_cov, n4_cov);
  const double res_b1 = std::max(e1, n4_e);
  std::printf("[RM6 N3/N4] e(self) %.6f e(+1 level) %.6f | N4 oracle noise: A2 %.0f A3 %.6f A1 %.6f B1 %.6f |"
              " N1: A2 %.0f A3 %.6f A1 %.6f | covered min %zu\n", e0, e1, n4_max, n4_mean, n4_cov,
              n4_e, n1_max, n1_mean, n1_cov, covered);
  std::printf("[RM6 resolution] A1 %.6f (tol %.4f) A2 %.1f (tol %d) A3 %.6f (tol %.2f) B1 %.6f (tol %.2f)\n",
              res_a1, kA1CoverageFraction, res_a2, kA2MaxAbs, res_a3, kA3MeanAbs, res_b1, kB1Rms);
  EXPECT_EQ(e0, 0.0);
  EXPECT_GE(covered, 50000u);
  EXPECT_GE(kA1CoverageFraction, 3.0 * res_a1);  // protocol section 3: tolerance >= 3 x resolution
  EXPECT_GE(static_cast<double>(kA2MaxAbs), 3.0 * res_a2);
  EXPECT_GE(kA3MeanAbs, 3.0 * res_a3);
  EXPECT_GE(kB1Rms, 3.0 * res_b1);
}

// A4: the slot draw clears its slot first: slots 2 and 3 are all 0 after a second draw with 2 ROIs.
TEST_F(RoiOracle, A4SlotsAreClearedBeforeTheDraw) {
  const std::string dir = Prepare("textured");
  const std::vector<c8::HMatrix> warps = WarpsA(dir);
  const GrayImage camera = ReadPgm(dir + "/a_camera.pgm");
  frontend::PyramidSource source(480, 640, frontend::ShaderParams{}, ShaderNumericModel::kIeee);
  const frontend::PyramidLayout& layout = source.layout();
  std::vector<uint8_t> rgba(static_cast<size_t>(480) * 640 * 4, 255);
  for (size_t i = 0; i < camera.pixels.size(); ++i) {
    rgba[4 * i] = rgba[4 * i + 1] = rgba[4 * i + 2] = camera.pixels[i];
  }
  const frontend::CpuGr8Pyramid cpu(layout, frontend::ShaderParams{}, ShaderNumericModel::kIeee);
  frontend::RgbaAtlas atlas;
  cpu.Build(RgbaImageView{rgba.data(), 480, 640, 480 * 4}, 0, &atlas);
  const auto roi = [&warps](size_t i) {
    c8::ImageRoi r;
    r.source = c8::ImageRoi::Source::HIRES_SCAN;
    r.warp = warps[i];
    return r;
  };
  const RgbaImageView cam_view{rgba.data(), 480, 640, 480 * 4};
  frontend::DrawRoisIntoAtlas(layout, cpu, ShaderNumericModel::kIeee, cam_view, 0,
                              {roi(0), roi(1), roi(2), roi(3)}, &atlas);
  const auto lit = [&](int slot) {
    const frontend::LevelLayout& l = layout.roi_slots[static_cast<size_t>(slot)];
    size_t n = 0;
    for (int y = 0; y < l.h; ++y) {
      for (int x = 0; x < l.w; ++x) {
        n += atlas.rgba[(static_cast<size_t>(l.r + y) * atlas.size + (l.c + x)) * 4] != 0;
      }
    }
    return n;
  };
  EXPECT_GT(lit(2), 0u);
  EXPECT_GT(lit(3), 0u);
  frontend::DrawRoisIntoAtlas(layout, cpu, ShaderNumericModel::kIeee, cam_view, 0, {roi(0), roi(1)},
                              &atlas);
  std::printf("[RM6 A4] lit pixels after the 2-ROI redraw: slot 2 %zu, slot 3 %zu (slots 0/1: %zu / %zu)\n",
              lit(2), lit(3), lit(0), lit(1));
  EXPECT_EQ(lit(2), 0u);
  EXPECT_EQ(lit(3), 0u);
  EXPECT_GT(lit(0), 0u);
}

TEST_F(RoiOracle, GateAArbitraryWarps) {
  const std::vector<ADiff> d = GateA("textured", nullptr);
  Print("G-ROI-A", d);
  for (const ADiff& x : d) {
    EXPECT_LE(x.coverage_fraction, kA1CoverageFraction);
    EXPECT_LE(x.max_abs, kA2MaxAbs);
    EXPECT_LE(x.mean_abs, kA3MeanAbs);
  }
}

TEST_F(RoiOracle, GateBKnownPoseGeometry) {
  const std::vector<double> e = GateB(nullptr);
  std::printf("[RM6 G-ROI-B] e frontal %.5f, roll+8 %.5f, roll-8 %.5f, k18 %.5f, k32 %.5f, k64 %.5f "
              "(tolerance %.2f)\n", e[0], e[1], e[2], e[3], e[4], e[5], kB1Rms);
  for (double x : e) EXPECT_LE(x, kB1Rms);
}

// Mutation detection: each mutated warp must violate G-ROI-A in at least one warp and G-ROI-B in at
// least one pose.
TEST_F(RoiOracle, MutationsAreDetected) {
  for (const Mutation& m : Mutations()) {
    const std::vector<ADiff> a = GateA("textured", &m);
    Print(m.name, a);
    bool a_detected = false;
    for (const ADiff& x : a) a_detected = a_detected || !x.Pass();
    EXPECT_TRUE(a_detected) << m.name << ": G-ROI-A did not react";
    const std::vector<double> e = GateB(&m);
    std::printf("[RM6 %s] G-ROI-B e: %.5f %.5f %.5f %.5f %.5f %.5f\n", m.name, e[0], e[1], e[2], e[3],
                e[4], e[5]);
    bool b_detected = false;
    for (double x : e) b_detected = b_detected || x > kB1Rms;
    EXPECT_TRUE(b_detected) << m.name << ": G-ROI-B did not react";
  }
}

}  // namespace
}  // namespace slam_refact

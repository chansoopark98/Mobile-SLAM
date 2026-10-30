// RM-7 G1 (build/slam-refact-full-20261019/preregistration/face-ear-rm7.v1.md): how often do the
// two ported public tests that drew their inputs with rand() keep their original assertion when
// the input is another random draw? It is a measurement, not a gate: the only assertions below are
// about the measuring instrument (null-run, mutation, counts), never about the violation fraction.
//
// TestRenderTexToRenderTex (PUB:reality/engine/faces/face-geometry-test.cc): EXPECT_FLOAT_EQ, i.e.
//   at most 4 ULP, on the x and y of 3 points mapped through two ROIs.
// TestMirrorInvisibleEarVertices (PUB:reality/engine/ears/ear-types-test.cc): EXPECT_NEAR 1e-5.
// The vendored code under test is byte-identical to the public source (check_vendor.py), so these
// results characterize the assertions and the platform arithmetic, not the port.
//
// SR_FACE_T1_DIST_OUT=<path> writes the histograms as JSON (the test fails if the file exists).
#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <functional>
#include <string>
#include <sys/stat.h>

#include "face/face_roi_renderer.h"
#include "face_test_util.h"
#include "reality/engine/ears/ear-types.h"
#include "reality/engine/faces/face-geometry.h"

namespace c8 {
namespace {

using slam_refact::face::testing::GlibcRand;
using slam_refact::face::testing::IPhone6;
using slam_refact::face::testing::Lcg;

constexpr int kInputsPerSet = 5000;
constexpr int kUlpLimit = 4;  // gtest EXPECT_FLOAT_EQ

// Distance in units in the last place, as gtest's FloatingPoint::AlmostEquals measures it
// (sign-and-magnitude mapped to a biased unsigned integer). NaN: max.
uint32_t UlpDistance(float a, float b) {
  if (std::isnan(a) || std::isnan(b)) {
    return UINT32_MAX;
  }
  auto biased = [](float f) {
    uint32_t bits;
    std::memcpy(&bits, &f, sizeof(bits));
    return (bits & 0x80000000u) ? ~bits + 1u : bits | 0x80000000u;
  };
  const uint32_t x = biased(a);
  const uint32_t y = biased(b);
  return x >= y ? x - y : y - x;
}

// Draw sources: each returns the next value used as `rand0to1()` of the public tests.
using Draw = std::function<float()>;

// ANSI C example rand(): next = next * 1103515245 + 12345; (next / 65536) % 32768 (32-bit state).
class AnsiRand {
 public:
  int32_t Next() {
    state_ = state_ * 1103515245u + 12345u;
    return static_cast<int32_t>((state_ / 65536u) % 32768u);
  }

 private:
  uint32_t state_ = 1;
};

// The computation of TestRenderTexToRenderTex, with the inputs drawn from `draw`. Returns the
// largest ULP distance over the 6 compared float pairs.
uint32_t RenderTexToRenderTexMaxUlp(const Draw& rand0to1) {
  ImageViewport viewport{0, 0, 256, 256};
  auto k = IPhone6();
  const float originRotRad = rand0to1() * M_PI;
  const float originCx = rand0to1();
  const float originCy = rand0to1();
  HMatrix originRot = HMatrixGen::zRadians(originRotRad);
  HMatrix originCenter = HMatrixGen::translation(-originCx, -originCy, 0.0f);
  DetectionRoi originRoi =
      slam_refact::face::MakeImageRoi(ImageRoi::Source::FACE, 0, originRot * originCenter);
  DetectedPoints detInOriginRoi{1, 0, viewport, originRoi, {}, {}, k, {}};
  for (size_t i = 0; i < 3; ++i) {
    detInOriginRoi.points.push_back({rand0to1(), rand0to1(), 1.0f});
  }
  const float rotRad = rand0to1() * M_PI;
  const float cx = rand0to1();
  const float cy = rand0to1();
  DetectionRoi newRoi = slam_refact::face::MakeImageRoi(
      ImageRoi::Source::FACE, 0,
      HMatrixGen::zRadians(rotRad) * HMatrixGen::translation(-cx, -cy, 0.0f));
  DetectedPoints detInNewRoi{1, 0, viewport, newRoi, {}, {}, k, {}};
  HMatrix m = renderTexToRenderTex(originRoi, newRoi);
  detInNewRoi.points = m * detInOriginRoi.points;
  DetectedImagePoints originImgPts = detectionToImageSpace(detInOriginRoi);
  DetectedImagePoints newImgPts = detectionToImageSpace(detInNewRoi);
  uint32_t worst = 0;
  for (size_t i = 0; i < originImgPts.points.size(); ++i) {
    worst = std::max(worst, UlpDistance(originImgPts.points[i].x(), newImgPts.points[i].x()));
    worst = std::max(worst, UlpDistance(originImgPts.points[i].y(), newImgPts.points[i].y()));
  }
  return worst;
}

// The computation of TestMirrorInvisibleEarVertices; returns the largest |lhs - rhs| of its nine
// EXPECT_NEAR comparisons.
double MirrorMaxDiff(const Draw& rand) {
  auto random0To1Point = [&rand]() -> HPoint3 {
    const float x = rand();
    const float y = rand();
    const float z = rand();
    return {x, y, z};
  };
  Ear3d ear3d;
  HPoint3 leftRefPt = random0To1Point();
  HPoint3 rightRefPt = random0To1Point();
  for (size_t i = 0; i < 3; ++i) {
    ear3d.leftVertices.push_back(random0To1Point());
    ear3d.rightVertices.push_back(random0To1Point());
    ear3d.leftVisibilities.push_back(0.6f);
    ear3d.rightVisibilities.push_back(0.2f);
  }
  mirrorInvisibleEarVertices(leftRefPt, rightRefPt, &ear3d);
  double worst = 0.0;
  for (size_t i = 0; i < ear3d.leftVertices.size(); ++i) {
    worst = std::max(worst, std::fabs(double(ear3d.leftVertices[i].x() - leftRefPt.x()) -
                                      double(rightRefPt.x() - ear3d.rightVertices[i].x())));
    worst = std::max(worst, std::fabs(double(ear3d.leftVertices[i].y() - leftRefPt.y()) -
                                      double(ear3d.rightVertices[i].y() - rightRefPt.y())));
    worst = std::max(worst, std::fabs(double(ear3d.leftVertices[i].z() - leftRefPt.z()) -
                                      double(ear3d.rightVertices[i].z() - rightRefPt.z())));
  }
  return worst;
}

struct Histogram {
  // ULP buckets {0-4, 5-8, 9-16, 17-64, 65-1024, >1024}.
  std::array<int, 6> buckets{};
  int total = 0;
  uint32_t max_ulp = 0;
  void Add(uint32_t ulp) {
    ++total;
    max_ulp = std::max(max_ulp, ulp);
    const int b = ulp <= 4 ? 0 : ulp <= 8 ? 1 : ulp <= 16 ? 2 : ulp <= 64 ? 3 : ulp <= 1024 ? 4 : 5;
    ++buckets[b];
  }
  int over4() const { return total - buckets[0]; }
};

struct MirrorHist {
  int total = 0;
  int over = 0;
  double max_diff = 0.0;
  void Add(double d) {
    ++total;
    over += d > 1e-5 ? 1 : 0;
    max_diff = std::max(max_diff, d);
  }
};

struct Sets {
  Histogram ansi;
  Histogram glibc;
  Histogram lcg;
  MirrorHist m_ansi;
  MirrorHist m_glibc;
  MirrorHist m_lcg;
  uint32_t first_glibc_ulp = 0;
};

Sets Measure() {
  Sets s;
  {
    AnsiRand ansi;
    Draw d = [&ansi]() { return static_cast<float>(ansi.Next() % 256) / 256.0f; };
    for (int i = 0; i < kInputsPerSet; ++i) {
      s.ansi.Add(RenderTexToRenderTexMaxUlp(d));
    }
    AnsiRand ansi_m;
    Draw dm = [&ansi_m]() { return static_cast<float>(ansi_m.Next() % 256) / 256.0f; };
    for (int i = 0; i < kInputsPerSet; ++i) {
      s.m_ansi.Add(MirrorMaxDiff(dm));
    }
  }
  {
    GlibcRand glibc;
    Draw d = [&glibc]() { return static_cast<float>(glibc.Next() % 256) / 256.0f; };
    for (int i = 0; i < kInputsPerSet; ++i) {
      const uint32_t u = RenderTexToRenderTexMaxUlp(d);
      if (i == 0) {
        s.first_glibc_ulp = u;
      }
      s.glibc.Add(u);
    }
    GlibcRand glibc_m;
    Draw dm = [&glibc_m]() { return static_cast<float>(glibc_m.Next() % 256) / 256.0f; };
    for (int i = 0; i < kInputsPerSet; ++i) {
      s.m_glibc.Add(MirrorMaxDiff(dm));
    }
  }
  for (int seed = 1; seed <= kInputsPerSet; ++seed) {
    Lcg lcg(static_cast<uint32_t>(seed));
    Draw d = [&lcg]() { return lcg.Next(); };
    s.lcg.Add(RenderTexToRenderTexMaxUlp(d));
    Lcg lcg_m(static_cast<uint32_t>(seed));
    Draw dm = [&lcg_m]() { return lcg_m.Next(); };
    s.m_lcg.Add(MirrorMaxDiff(dm));
  }
  return s;
}

std::string HistJson(const Histogram& h) {
  char buf[256];
  std::snprintf(buf, sizeof(buf),
                "{\"total\":%d,\"over4ulp\":%d,\"max_ulp\":%u,\"buckets_0_4_5_8_9_16_17_64_65_1024_gt1024\":"
                "[%d,%d,%d,%d,%d,%d]}",
                h.total, h.over4(), h.max_ulp, h.buckets[0], h.buckets[1], h.buckets[2],
                h.buckets[3], h.buckets[4], h.buckets[5]);
  return buf;
}

std::string MirrorJson(const MirrorHist& h) {
  char buf[160];
  std::snprintf(buf, sizeof(buf), "{\"total\":%d,\"over_1e-5\":%d,\"max_diff\":%.9g}", h.total,
                h.over, h.max_diff);
  return buf;
}

TEST(FaceT1InputDistribution, InstrumentNullRunAndMutation) {
  // Null-run: identical operands are 0 ULP apart. Resolution = 1 ULP.
  EXPECT_EQ(0u, UlpDistance(0.18931115f, 0.18931115f));
  EXPECT_EQ(1u, UlpDistance(1.0f, std::nextafter(1.0f, 2.0f)));
  EXPECT_EQ(0u, UlpDistance(0.0f, -0.0f));  // +0 and -0 are the same point on the ULP scale
  // Mutation: a comparator that cannot see a 5 ULP shift would make the limit meaningless.
  float shifted = 0.18931115f;
  for (int i = 0; i < 5; ++i) {
    shifted = std::nextafter(shifted, 1.0f);
  }
  EXPECT_EQ(5u, UlpDistance(0.18931115f, shifted));
  EXPECT_GT(UlpDistance(0.18931115f, shifted), static_cast<uint32_t>(kUlpLimit));
}

TEST(FaceT1InputDistribution, RenderTexToRenderTexAndMirrorOverRandomInputs) {
  const Sets s = Measure();
  // The test of face_geometry_test.cc passes on the first glibc input (that is the selected
  // sample); this is the only property asserted about the data. Like that test it is a platform
  // pin (it repeats the fragile 4 ULP assertion on the libm of the build host), not evidence.
  EXPECT_LE(s.first_glibc_ulp, static_cast<uint32_t>(kUlpLimit));
  for (const Histogram* h : {&s.ansi, &s.glibc, &s.lcg}) {
    EXPECT_EQ(kInputsPerSet, h->total);
    int sum = 0;
    for (int b : h->buckets) {
      sum += b;
    }
    EXPECT_EQ(kInputsPerSet, sum);
  }
  std::printf(
      "[RM-7 G1] TestRenderTexToRenderTex 4-ULP violations of %d inputs: ansi %d (%.1f%%), "
      "glibc %d (%.1f%%), lcg %d (%.1f%%); first glibc input %u ULP\n",
      kInputsPerSet, s.ansi.over4(), 100.0 * s.ansi.over4() / kInputsPerSet, s.glibc.over4(),
      100.0 * s.glibc.over4() / kInputsPerSet, s.lcg.over4(),
      100.0 * s.lcg.over4() / kInputsPerSet, s.first_glibc_ulp);
  std::printf(
      "[RM-7 G1] TestMirrorInvisibleEarVertices >1e-5 of %d inputs: ansi %d, glibc %d, lcg %d\n",
      kInputsPerSet, s.m_ansi.over, s.m_glibc.over, s.m_lcg.over);
  if (const char* out = std::getenv("SR_FACE_T1_DIST_OUT")) {
    struct stat st;
    ASSERT_NE(0, ::stat(out, &st)) << "output exists: " << out;
    std::ofstream f(out);
    f << "{\"inputs_per_set\":" << kInputsPerSet << ",\"ulp_limit\":" << kUlpLimit
      << ",\"first_glibc_input_ulp\":" << s.first_glibc_ulp
      << ",\"render_tex_to_render_tex\":{\"ansi\":" << HistJson(s.ansi)
      << ",\"glibc\":" << HistJson(s.glibc) << ",\"lcg\":" << HistJson(s.lcg)
      << "},\"mirror_invisible_ear_vertices_delta_1e-5\":{\"ansi\":" << MirrorJson(s.m_ansi)
      << ",\"glibc\":" << MirrorJson(s.m_glibc) << ",\"lcg\":" << MirrorJson(s.m_lcg) << "}}\n";
  }
}

}  // namespace
}  // namespace c8

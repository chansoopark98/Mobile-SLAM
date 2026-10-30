// WP1 unit gates on the checked-in TUM captures (tests/data/frontend, architecture.md §7.2):
// F-H3 / F-H4 (CPU shader port), F-H2 (CPU half, binary func 917+918 golden of cap-001),
// F-H7 (injected pyramid -> detection path), plus pyramid-source and detection unit checks.

#include <cstdint>
#include <string>
#include <vector>

#include "c8/geometry/device-pose.h"
#include "c8/hmatrix.h"
#include "common/c8_adapters.h"
#include "frontend/frontend.h"
#include "frontend/intrinsics.h"
#include "frontend/pyramid_source.h"
#include "gate_util.h"
#include "gtest/gtest.h"
#include "test_util.h"

namespace slam_refact::frontend {
namespace {

using testing::Json;

constexpr int kCaptureRows = 660;

std::string TumCapture(int k) {
  char name[64];
  std::snprintf(name, sizeof(name), "/frontend/capture/tum-room1-full/c2/cap-%03d.rgba.gz", k);
  return testing::TestDataDir() + name;
}

class TumCaptureTest : public ::testing::TestWithParam<int> {};

// F-H3: captured gray -> stage 2 + 3 (SwiftShader-approximate model) == captured G/B/A, BIT_EXACT
// in every level region (S1 §8.3 H3).
TEST_P(TumCaptureTest, Stage23FromCapturedGrayIsBitExact) {
  RgbaAtlas captured;
  ASSERT_TRUE(testing::LoadCaptureAtlas(TumCapture(GetParam()), kCaptureRows, &captured));
  const PyramidLayout layout = ComputePyramidLayout(240, 320);
  RgbaAtlas mine = captured;
  CpuGr8Pyramid(layout, ShaderParams{}, ShaderNumericModel::kSwiftShaderApprox)
      .RunStage2And3(&mine);
  const testing::AtlasStats st = testing::CompareLevels(mine, captured, layout);
  EXPECT_TRUE(testing::Stage23BitExact(st))
      << "equal R/G/B/A " << st.equal[0] << "/" << st.equal[1] << "/" << st.equal[2] << "/"
      << st.equal[3] << " of " << st.pixels;
}

// F-H4: captured level 0 -> blur/decimate/stage 2/3 vs capture: R, G identical >= 0.9999 and
// |d| <= 1; B, A identical >= 0.9999 (S1 §8.3 H4).
TEST_P(TumCaptureTest, LevelsRebuiltFromCapturedLevel0) {
  RgbaAtlas captured;
  ASSERT_TRUE(testing::LoadCaptureAtlas(TumCapture(GetParam()), kCaptureRows, &captured));
  const PyramidLayout layout = ComputePyramidLayout(240, 320);
  RgbaAtlas mine = captured;
  CpuGr8Pyramid(layout, ShaderParams{}, ShaderNumericModel::kSwiftShaderApprox)
      .RunFromLevel0(&mine);
  const testing::AtlasStats st = testing::CompareLevels(mine, captured, layout);
  EXPECT_TRUE(testing::RebuildPass(st))
      << "identical R " << st.identical_fraction(0) << " G " << st.identical_fraction(1) << " B "
      << st.identical_fraction(2) << " A " << st.identical_fraction(3) << " maxdiff R "
      << st.max_diff[0] << " G " << st.max_diff[1];
}

INSTANTIATE_TEST_SUITE_P(TumC2, TumCaptureTest, ::testing::Values(1, 2, 3));

// F-H2 (unit): engine CPU half on the injected capture == binary func 917 + 918 (set and order).
TEST(FrontendCpuHalf, MatchesBinaryOnTumCap001) {
  Json golden;
  ASSERT_TRUE(testing::ReadJsonFile(testing::TestDataDir() + "/frontend/binary-gr8/c2-cap001.json",
                                    &golden));
  ASSERT_EQ(golden["wasmSha256"].str(),
            "780731fe294a725acf1b7717ed5ffb07ad9f0929e408a1d74f47450c291ec422");
  RgbaAtlas captured;
  ASSERT_TRUE(testing::LoadCaptureAtlas(TumCapture(1), kCaptureRows, &captured));
  const PyramidLayout layout = ComputePyramidLayout(240, 320);
  const c8::Gr8Pyramid pyr = PyramidFromInjectedAtlas(
      PyramidAtlasView{captured.rgba.data(), captured.size, captured.rows}, layout);
  const auto lists = testing::CpuHalfPerLevel(pyr);
  std::string detail;
  EXPECT_EQ(testing::CompareCpuHalfWithBinary(lists, golden, &detail), 0) << detail;
  // The capture has a level where retainBest trims (the order-sensitive case, Q3).
  bool trimmed = false;
  for (size_t li = 0; li < 4; ++li) {
    trimmed = trimmed || golden["levels"][li]["candidates"].number() >
                             golden["levels"][li]["kept"].number();
  }
  EXPECT_TRUE(trimmed);
}

// Runs the production detection path (PyramidSource injection -> FeatureFrontend::Detect) and
// checks it against the binary per-level lists mapped to level-0 pixels.
void ExpectDetectionMatchesBinary(const c8::Gr8Pyramid& pyr, const Json& golden,
                                  const c8::FrameWithPoints& frame) {
  const std::vector<testing::ExpectedPoint> expected = testing::ExpectedBasePoints(golden, pyr);
  ASSERT_EQ(frame.pixels().size(), expected.size());
  ASSERT_EQ(frame.points().size(), expected.size());
  int mismatches = 0;
  for (size_t i = 0; i < expected.size(); ++i) {
    const bool ok = frame.pixels()[i].x() == expected[i].x &&
                    frame.pixels()[i].y() == expected[i].y &&
                    static_cast<int>(frame.points()[i].scale()) == expected[i].scale;
    mismatches += ok ? 0 : 1;
  }
  EXPECT_EQ(mismatches, 0);
}

// F-H7 (unit): injected pyramid -> detection path reproduces the binary CPU-half output.
TEST(FrontendDetect, InjectedPyramidDetectionPathMatchesBinary) {
  Json golden;
  ASSERT_TRUE(testing::ReadJsonFile(testing::TestDataDir() + "/frontend/binary-gr8/c2-cap001.json",
                                    &golden));
  RgbaAtlas captured;
  ASSERT_TRUE(testing::LoadCaptureAtlas(TumCapture(1), kCaptureRows, &captured));
  PyramidSource source(240, 320, ShaderParams{}, ShaderNumericModel::kSwiftShaderApprox);
  const c8::Gr8Pyramid pyr =
      source.Get(PyramidAtlasView{captured.rgba.data(), captured.size, captured.rows}, {}, {});
  EXPECT_EQ(pyr.data.pixels(), captured.rgba.data());  // injection does not copy or rebuild

  Json meta;
  ASSERT_TRUE(testing::ReadJsonFile(
      testing::TestDataDir() + "/frontend/capture/tum-room1-full/c2/meta-excerpt.json", &meta));
  const Json& q = meta["captures"][0]["stageArgs"]["qwxyz"];
  const c8::Quaternion device(static_cast<float>(q[0].number()), static_cast<float>(q[1].number()),
                              static_cast<float>(q[2].number()), static_cast<float>(q[3].number()));
  const IntrinsicsSet k = ComputeIntrinsicsSet("Google", "Pixel 7", 240, 320,
                                               source.layout().levels[0].w,
                                               source.layout().levels[0].h);
  c8::FrameWithPoints frame(ToC8Intrinsics(k.processing), c8::HMatrixGen::i(),
                            c8::xrRotationFromDeviceRotation(device));
  FeatureFrontend frontend;
  frontend.Detect(pyr, WorldSlamDetectionConfig(), &frame);
  ExpectDetectionMatchesBinary(pyr, golden, frame);

  int kept = 0;
  for (size_t li = 0; li < 4; ++li) {
    kept += static_cast<int>(golden["levels"][li]["kept"].number());
  }
  EXPECT_EQ(frontend.last_stats().num_keypoints, kept);
  EXPECT_EQ(frontend.last_stats().num_gorb, kept);  // allGorb
  EXPECT_GE(frontend.last_stats().num_orb, 0);
  EXPECT_LE(frontend.last_stats().num_orb, kept);
  EXPECT_EQ(frame.store().numKeypoints(), static_cast<size_t>(kept));

  // A second Detect into the same frame clears the previous points first.
  frontend.Detect(pyr, WorldSlamDetectionConfig(), &frame);
  EXPECT_EQ(frame.pixels().size(), static_cast<size_t>(kept));
}

TEST(FrontendDetect, WorldSlamDetectionConfig) {
  const c8::DetectionConfig cfg = WorldSlamDetectionConfig();
  EXPECT_EQ(cfg.nKeypoints, 500);
  EXPECT_TRUE(cfg.allGorb);
  EXPECT_FALSE(cfg.allOrb);
  EXPECT_FALSE(cfg.allLearned);
  EXPECT_EQ(static_cast<int>(cfg.gravityPortions[c8::DescriptorType::ORB]), 192);
  EXPECT_EQ(static_cast<int>(cfg.gravityPortions[c8::DescriptorType::GORB]), 0);
  EXPECT_EQ(static_cast<int>(cfg.gravityPortions[c8::DescriptorType::LEARNED]), 0);
}

// CPU path of PyramidSource: gray input is uploaded as R = G = B, A = 255, so the gray and RGBA
// entry points give the same atlas, and the build is deterministic.
TEST(FrontendPyramidSource, GrayAndRgbaPathsAgree) {
  constexpr int kW = 240, kH = 320;
  std::vector<uint8_t> gray(kW * kH);
  for (int y = 0; y < kH; ++y) {
    for (int x = 0; x < kW; ++x) {
      gray[y * kW + x] = static_cast<uint8_t>((x * 7 + y * 13 + ((x / 16 + y / 16) % 2) * 90) & 255);
    }
  }
  std::vector<uint8_t> rgba(kW * kH * 4);
  for (int i = 0; i < kW * kH; ++i) {
    rgba[4 * i] = rgba[4 * i + 1] = rgba[4 * i + 2] = gray[i];
    rgba[4 * i + 3] = 255;
  }
  for (ShaderNumericModel model : {ShaderNumericModel::kIeee, ShaderNumericModel::kSwiftShaderApprox}) {
    PyramidSource a(kW, kH, ShaderParams{}, model);
    PyramidSource b(kW, kH, ShaderParams{}, model);
    a.Get({}, {}, GrayImageView{gray.data(), kW, kH, kW});
    b.Get({}, RgbaImageView{rgba.data(), kW, kH, kW * 4}, {});
    EXPECT_EQ(a.last_cpu_atlas().rgba, b.last_cpu_atlas().rgba);
    EXPECT_EQ(a.last_cpu_atlas().size, 1024);
    EXPECT_EQ(a.last_cpu_atlas().rows, a.layout().rows);
    const RgbaAtlas first = a.last_cpu_atlas();
    a.Get({}, {}, GrayImageView{gray.data(), kW, kH, kW});
    EXPECT_EQ(a.last_cpu_atlas().rgba, first.rgba);
    // Feature candidates exist on the textured image.
    long candidates = 0;
    for (size_t i = 2; i < first.rgba.size(); i += 4) {
      candidates += first.rgba[i] != 0;
    }
    EXPECT_GT(candidates, 0);
  }
}

// A flat image: every stage keeps the gray value inside the levels (away from the level borders),
// and no FAST candidate exists.
TEST(FrontendPyramidSource, FlatImageGivesFlatLevelsAndNoCandidates) {
  constexpr int kW = 240, kH = 320;
  std::vector<uint8_t> gray(kW * kH, 128);
  PyramidSource source(kW, kH, ShaderParams{}, ShaderNumericModel::kIeee);
  source.Get({}, {}, GrayImageView{gray.data(), kW, kH, kW});
  const RgbaAtlas& atlas = source.last_cpu_atlas();
  for (const LevelLayout& l : source.layout().levels) {
    for (int y = 8; y < l.h - 8; ++y) {
      for (int x = 8; x < l.w - 8; ++x) {
        const uint8_t* p = &atlas.rgba[(static_cast<size_t>(l.r + y) * atlas.size + l.c + x) * 4];
        ASSERT_EQ(p[0], 128) << "R at level (" << x << "," << y << ")";
        ASSERT_EQ(p[2], 0) << "B at level (" << x << "," << y << ")";
        ASSERT_EQ(p[3], 0x77) << "A at level (" << x << "," << y << ")";
      }
    }
  }
}

TEST(FrontendPyramidSourceDeathTest, InjectedAtlasMustMatchLayout) {
  const PyramidLayout layout = ComputePyramidLayout(240, 320);
  std::vector<uint8_t> small(512 * 400 * 4, 0);
  EXPECT_DEATH(PyramidFromInjectedAtlas(PyramidAtlasView{small.data(), 512, 400}, layout),
               "size differs");
  std::vector<uint8_t> short_rows(1024 * 600 * 4, 0);
  EXPECT_DEATH(PyramidFromInjectedAtlas(PyramidAtlasView{short_rows.data(), 1024, 600}, layout),
               "every level");
}

TEST(FrontendCpuPyramidDeathTest, CameraOrientationMustMatchLayout) {
  const PyramidLayout layout = ComputePyramidLayout(240, 320);
  std::vector<uint8_t> rgba(320 * 240 * 4, 0);
  RgbaAtlas out;
  const CpuGr8Pyramid cpu(layout, ShaderParams{}, ShaderNumericModel::kIeee);
  EXPECT_DEATH(cpu.Build(RgbaImageView{rgba.data(), 320, 240, 320 * 4}, 0, &out), "orientation");
}

// S2 review MEDIUM-3: a CPU-path image must have the engineInit (capture) size.
TEST(FrontendPyramidSourceDeathTest, FrameMustHaveCaptureSize) {
  PyramidSource source(240, 320, ShaderParams{}, ShaderNumericModel::kIeee);
  std::vector<uint8_t> px(240 * 320 * 4, 0);
  EXPECT_DEATH(source.Get({}, {}, GrayImageView{px.data(), 232, 320, 240}),
               "differs from the capture geometry");
  EXPECT_DEATH(source.Get({}, RgbaImageView{px.data(), 240, 300, 240 * 4}, {}),
               "differs from the capture geometry");
}

// S2 review LOW-7: the kept texture buffers only save allocations; a build after another build
// (different image, or the H3/H4 entry points) equals a build on a fresh instance byte for byte.
TEST(FrontendCpuPyramid, ReusedTexturesDoNotCarryStateBetweenBuilds) {
  constexpr int kW = 240, kH = 320;
  const PyramidLayout layout = ComputePyramidLayout(kW, kH);
  std::vector<uint8_t> textured(kW * kH * 4);
  std::vector<uint8_t> flat(kW * kH * 4, 0);
  for (int i = 0; i < kW * kH; ++i) {
    const int x = i % kW, y = i / kW;
    const uint8_t v = static_cast<uint8_t>((x * 7 + y * 13 + ((x / 16 + y / 16) % 2) * 90) & 255);
    textured[4 * i] = textured[4 * i + 1] = textured[4 * i + 2] = v;
    textured[4 * i + 3] = 255;
    flat[4 * i] = flat[4 * i + 1] = flat[4 * i + 2] = 60;
    flat[4 * i + 3] = 255;
  }
  const RgbaImageView tex_view{textured.data(), kW, kH, kW * 4};
  const RgbaImageView flat_view{flat.data(), kW, kH, kW * 4};
  for (ShaderNumericModel model : {ShaderNumericModel::kIeee, ShaderNumericModel::kSwiftShaderApprox}) {
    const CpuGr8Pyramid reused(layout, ShaderParams{}, model);
    RgbaAtlas warm;
    reused.Build(tex_view, 0, &warm);
    RgbaAtlas second;
    reused.Build(flat_view, 0, &second);
    RgbaAtlas fresh_flat;
    CpuGr8Pyramid(layout, ShaderParams{}, model).Build(flat_view, 0, &fresh_flat);
    EXPECT_EQ(second.rgba, fresh_flat.rgba);
    RgbaAtlas third;
    reused.Build(tex_view, 0, &third);
    EXPECT_EQ(third.rgba, warm.rgba);

    RgbaAtlas h3_reused = warm;
    RgbaAtlas h3_fresh = warm;
    reused.RunStage2And3(&h3_reused);
    CpuGr8Pyramid(layout, ShaderParams{}, model).RunStage2And3(&h3_fresh);
    EXPECT_EQ(h3_reused.rgba, h3_fresh.rgba);
    RgbaAtlas h4_reused = warm;
    RgbaAtlas h4_fresh = warm;
    reused.RunFromLevel0(&h4_reused);
    CpuGr8Pyramid(layout, ShaderParams{}, model).RunFromLevel0(&h4_fresh);
    EXPECT_EQ(h4_reused.rgba, h4_fresh.rgba);
  }
}

}  // namespace
}  // namespace slam_refact::frontend

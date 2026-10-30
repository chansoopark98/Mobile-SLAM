// Hand image operations against OpenCV (cv2) golden data (US-021; docs/spec/10-hand.md §2, gate HAND-G1).
// The expected sha256 values come from tools/hand/gen_image_golden.py (cv2 executes the same calls the
// original calculators make); the C++ under test never links OpenCV.
#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <iterator>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include "hand/hand_geometry.h"
#include "hand/hand_image.h"
#include "hand_test_util.h"

namespace slam_refact::hand {
namespace {

using testutil::DifferingPixelFraction;
using testutil::MaxDiff;
using testutil::ReadRaw;
using testutil::TestImage;

std::vector<std::vector<std::string>> ReadTsv(const std::string& name) {
  return testutil::ReadTsv(SR_HAND_GOLDEN_DIR, name);
}

// Registered tolerances (hand-us021.v1.md HAND-G1). Resolution of the measurement: 1 level.
constexpr int kMaxLevelDiff = 3;               // resize / FIT: every byte within 3 levels of cv2
constexpr double kCropMaxDifferingPixels = 0.02;  // crop: fraction of pixels that differ at all

TEST(HandImageGolden, ResizeLinearIsWithinThreeLevelsOfCv2) {
  const auto rows = ReadTsv("resize_cases.tsv");
  ASSERT_EQ(rows.size(), 12u);
  for (size_t i = 0; i < rows.size(); ++i) {
    const auto& r = rows[i];
    const int sw = std::stoi(r[0]), sh = std::stoi(r[1]), dw = std::stoi(r[2]), dh = std::stoi(r[3]);
    const Image3 out = ResizeLinear(TestImage(sw, sh, static_cast<uint32_t>(std::stoul(r[4]))), dw, dh);
    std::vector<uint8_t> expected;
    ASSERT_TRUE(ReadRaw(SR_HAND_GOLDEN_DIR, "resize-" + std::to_string(i) + ".rgb", static_cast<size_t>(dw) * dh * 3, &expected));
    ASSERT_EQ(out.px.size(), expected.size());
    EXPECT_LE(MaxDiff(out, expected), kMaxLevelDiff) << sw << "x" << sh << " -> " << dw << "x" << dh;
  }
}

TEST(HandImageGolden, FitToSizeIsWithinThreeLevelsOfCv2WithTheSameLetterboxPadding) {
  const auto rows = ReadTsv("fit_cases.tsv");
  ASSERT_EQ(rows.size(), 8u);
  for (size_t i = 0; i < rows.size(); ++i) {
    const auto& r = rows[i];
    const int sw = std::stoi(r[0]), sh = std::stoi(r[1]), ow = std::stoi(r[2]), oh = std::stoi(r[3]);
    std::array<float, 4> pad{};
    const Image3 out = FitToSize(TestImage(sw, sh, static_cast<uint32_t>(std::stoul(r[4]))), ow, oh, &pad);
    std::vector<uint8_t> expected;
    ASSERT_TRUE(ReadRaw(SR_HAND_GOLDEN_DIR, "fit-" + std::to_string(i) + ".rgb", static_cast<size_t>(ow) * oh * 3, &expected));
    EXPECT_LE(MaxDiff(out, expected), kMaxLevelDiff) << sw << "x" << sh;
    for (int k = 0; k < 4; ++k) {
      EXPECT_EQ(static_cast<double>(pad[k]), std::stod(r[5 + k])) << "padding " << k << " of " << sw << "x" << sh;
    }
  }
}

TEST(HandImageGolden, BoxPointsMatchCv2) {
  for (const auto& r : ReadTsv("boxpoints_cases.tsv")) {
    const auto p = BoxPoints(std::stof(r[0]), std::stof(r[1]), std::stof(r[2]), std::stof(r[3]), std::stof(r[4]));
    for (int i = 0; i < 4; ++i) {
      EXPECT_NEAR(p[i][0], std::stod(r[5 + 2 * i]), 1e-4) << "case " << r[4] << " point " << i;
      EXPECT_NEAR(p[i][1], std::stod(r[6 + 2 * i]), 1e-4) << "case " << r[4] << " point " << i;
    }
  }
}

TEST(HandImageGolden, PerspectiveTransformMatchesCv2) {
  for (const auto& r : ReadTsv("perspective_cases.tsv")) {
    std::array<std::array<float, 2>, 4> src{};
    for (int i = 0; i < 4; ++i) {
      src[i][0] = std::stof(r[2 * i]);
      src[i][1] = std::stof(r[2 * i + 1]);
    }
    const float w = std::stof(r[8]), h = std::stof(r[9]);
    const std::array<std::array<float, 2>, 4> dst = {{{0.0f, h - 1}, {0.0f, 0.0f}, {w - 1, 0.0f}, {w - 1, h - 1}}};
    const auto m = GetPerspectiveTransform(src, dst);
    for (int i = 0; i < 9; ++i) {
      // cv2 returns float64 computed from float32 inputs through its own LU; compare relatively.
      const double e = std::stod(r[10 + i]);
      EXPECT_NEAR(m[i], e, 1e-4 * std::max(1.0, std::fabs(e))) << "entry " << i;
    }
  }
}

TEST(HandImageGolden, RotatedCropDiffersFromCv2OnlyAtTieRoundedPixels) {
  const auto rows = ReadTsv("crop_cases.tsv");
  ASSERT_EQ(rows.size(), 10u);
  for (size_t i = 0; i < rows.size(); ++i) {
    const auto& r = rows[i];
    const int sw = std::stoi(r[0]), sh = std::stoi(r[1]), cx = std::stoi(r[2]), cy = std::stoi(r[3]);
    const int w = std::stoi(r[4]), h = std::stoi(r[5]);
    const double rot = std::stod(r[6]);
    NormRect rect;
    rect.x_center = static_cast<float>(cx) / sw;
    rect.y_center = static_cast<float>(cy) / sh;
    rect.width = static_cast<float>(w) / sw;
    rect.height = static_cast<float>(h) / sh;
    rect.rotation = static_cast<float>(rot);
    const Image3 out = CropNormRect(TestImage(sw, sh, static_cast<uint32_t>(std::stoul(r[7]))), rect);
    ASSERT_EQ(out.width, w) << sw << "x" << sh;
    ASSERT_EQ(out.height, h);
    std::vector<uint8_t> expected;
    ASSERT_TRUE(ReadRaw(SR_HAND_GOLDEN_DIR, "crop-" + std::to_string(i) + ".rgb", static_cast<size_t>(w) * h * 3, &expected));
    const double fraction = DifferingPixelFraction(out, expected);
    EXPECT_LE(fraction, kCropMaxDifferingPixels) << "crop " << cx << "," << cy << " " << w << "x" << h << " rot " << rot;
  }
}

TEST(HandImage, TensorNormalisation) {
  Image3 img(2, 1);
  img.px = {0, 255, 128, 1, 2, 3};
  float zc[6], unit[6];
  ImageToTensor(img, true, zc);
  ImageToTensor(img, false, unit);
  EXPECT_EQ(zc[0], -1.0f);
  EXPECT_EQ(zc[1], 1.0f);
  EXPECT_EQ(zc[2], 128 / 127.5f - 1.0f);
  EXPECT_EQ(unit[1], 1.0f);
  EXPECT_EQ(unit[3], 1 / 255.0f);
}

}  // namespace
}  // namespace slam_refact::hand

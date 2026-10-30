// Calculator math of the hand graph against a second implementation (tools/hand/hand_ref.py, numpy) and
// structural properties (US-021; docs/spec/10-hand.md §3, gate HAND-G2). Same executor, different tool:
// agreement between the two is self-consistency (T3), not an independent oracle.
#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include "hand/hand_geometry.h"
#include "hand_test_util.h"

namespace slam_refact::hand {
namespace {

// Registered tolerances (hand-us021.v1.md HAND-G2): float32 resolution 6e-8 near 1 (1.2e-7 near 2,
// 2.4e-7 near 4), tolerance 1e-5 >= 3 x resolution for every quantity below (all |value| <= 64 and
// compared absolutely, relative for the rotation).
constexpr double kTol = 1e-5;

std::vector<std::vector<std::string>> ReadTsv(const std::string& name) {
  return testutil::ReadTsv(SR_HAND_GEOMETRY_GOLDEN_DIR, name);
}

using testutil::LcgValues;

TEST(HandAnchors, PalmGraphAnchorsAre2944AndMatchTheNumpyPort) {
  const auto anchors = GenerateAnchors(SsdAnchorsOptions());
  ASSERT_EQ(anchors.size(), 2944u);
  std::ifstream f(std::string(SR_HAND_GEOMETRY_GOLDEN_DIR) + "/anchors.f32", std::ios::binary);
  std::vector<float> expected(2944 * 4);
  f.read(reinterpret_cast<char*>(expected.data()), static_cast<std::streamsize>(expected.size() * 4));
  ASSERT_TRUE(f.good());
  for (size_t i = 0; i < anchors.size(); ++i) {
    ASSERT_NEAR(anchors[i].x_center, expected[4 * i], kTol) << i;
    ASSERT_NEAR(anchors[i].y_center, expected[4 * i + 1], kTol) << i;
    ASSERT_EQ(anchors[i].w, expected[4 * i + 2]) << i;
    ASSERT_EQ(anchors[i].h, expected[4 * i + 3]) << i;
  }
  // Layout: 32x32x2 at stride 8, 16x16x2 at stride 16, 8x8x6 at stride 32 (= 2048 + 512 + 384).
  EXPECT_NEAR(anchors[0].x_center, 0.5f / 32, 1e-7);
  EXPECT_NEAR(anchors[2048].x_center, 0.5f / 16, 1e-7);
  EXPECT_NEAR(anchors[2048 + 512].x_center, 0.5f / 8, 1e-7);
}

TEST(HandDetectionGolden, DecodeNmsAndLetterboxMatchTheNumpyPort) {
  const auto anchors = GenerateAnchors(SsdAnchorsOptions());
  const auto rows = ReadTsv("detect_cases.tsv");
  size_t row = 0;
  int cases = 0;
  while (row < rows.size()) {
    ASSERT_EQ(rows[row][0], "case");
    const uint32_t seed = static_cast<uint32_t>(std::stoul(rows[row][1]));
    const size_t n_decoded = std::stoul(rows[row][2]), n_final = std::stoul(rows[row][3]);
    std::array<float, 4> pad;
    for (int i = 0; i < 4; ++i) pad[i] = std::stof(rows[row][4 + i]);
    const int centers[3] = {std::stoi(rows[row][8]), std::stoi(rows[row][9]), std::stoi(rows[row][10])};
    std::vector<float> boxes, scores;
    testutil::PlantedTensors(seed, centers, &boxes, &scores);
    const auto decoded = DecodePalmDetections(boxes.data(), scores.data(), anchors, PalmDecodeOptions());
    EXPECT_EQ(decoded.size(), n_decoded) << "seed " << seed;
    auto kept = WeightedNonMaxSuppression(decoded, 0.3f, 0.5f);
    RemoveLetterbox(&kept, pad);
    ASSERT_EQ(kept.size(), n_final) << "seed " << seed;
    for (size_t d = 0; d < n_final; ++d) {
      const auto& e = rows[row + 1 + d];
      ASSERT_EQ(e[0], "det");
      const Detection& g = kept[d];
      EXPECT_NEAR(g.score, std::stod(e[1]), kTol);
      EXPECT_NEAR(g.xmin, std::stod(e[2]), kTol) << "seed " << seed << " det " << d;
      EXPECT_NEAR(g.ymin, std::stod(e[3]), kTol);
      EXPECT_NEAR(g.width, std::stod(e[4]), kTol);
      EXPECT_NEAR(g.height, std::stod(e[5]), kTol);
      ASSERT_EQ(g.keypoints.size(), 7u);
      for (int k = 0; k < 7; ++k) {
        EXPECT_NEAR(g.keypoints[k].x, std::stod(e[6 + 2 * k]), kTol);
        EXPECT_NEAR(g.keypoints[k].y, std::stod(e[7 + 2 * k]), kTol);
      }
    }
    row += 1 + n_final;
    ++cases;
  }
  EXPECT_EQ(cases, 4);
}

TEST(HandRectGolden, DetectionToRectAndTransformsMatchTheNumpyPort) {
  const auto rows = ReadTsv("rect_cases.tsv");
  ASSERT_EQ(rows.size(), 24u);
  for (const auto& r : rows) {
    const int iw = std::stoi(r[0]), ih = std::stoi(r[1]);
    Detection d;
    d.xmin = std::stof(r[2]);
    d.ymin = std::stof(r[3]);
    d.width = std::stof(r[4]);
    d.height = std::stof(r[5]);
    for (int k = 0; k < 10; ++k) d.keypoints.push_back({std::stof(r[6 + 2 * k]), std::stof(r[7 + 2 * k])});
    const size_t b = 6 + 20;
    const NormRect rect = DetectionToNormRect(d, iw, ih, 0, 2, 90.0f);
    EXPECT_NEAR(rect.x_center, std::stod(r[b]), kTol);
    EXPECT_NEAR(rect.y_center, std::stod(r[b + 1]), kTol);
    EXPECT_NEAR(rect.width, std::stod(r[b + 2]), kTol);
    EXPECT_NEAR(rect.height, std::stod(r[b + 3]), kTol);
    EXPECT_NEAR(rect.rotation, std::stod(r[b + 4]), kTol);
    RectTransformOptions palm;
    palm.scale_x = palm.scale_y = 2.6f;
    palm.shift_y = -0.5f;
    palm.square_long = true;
    const NormRect t = TransformNormRect(rect, iw, ih, palm);
    EXPECT_NEAR(t.x_center, std::stod(r[b + 5]), kTol);
    EXPECT_NEAR(t.y_center, std::stod(r[b + 6]), kTol);
    EXPECT_NEAR(t.width, std::stod(r[b + 7]), kTol);
    EXPECT_NEAR(t.height, std::stod(r[b + 8]), kTol);
    const NormRect next0 = DetectionToNormRect(d, iw, ih, 0, 9, 90.0f);
    RectTransformOptions next;
    next.scale_x = next.scale_y = 1.6f;
    next.square_long = true;
    const NormRect nt = TransformNormRect(next0, iw, ih, next);
    EXPECT_NEAR(nt.x_center, std::stod(r[b + 10]), kTol);
    EXPECT_NEAR(nt.y_center, std::stod(r[b + 11]), kTol);
    EXPECT_NEAR(nt.width, std::stod(r[b + 12]), kTol);
    EXPECT_NEAR(nt.height, std::stod(r[b + 13]), kTol);
    EXPECT_NEAR(nt.rotation, std::stod(r[b + 14]), kTol);
  }
}

TEST(HandLandmarkGolden, LetterboxProjectionAndBoundingBoxMatchTheNumpyPort) {
  const auto rows = ReadTsv("landmark_cases.tsv");
  ASSERT_EQ(rows.size(), 12u);
  for (const auto& r : rows) {
    std::array<float, 4> pad;
    for (int i = 0; i < 4; ++i) pad[i] = std::stof(r[i]);
    NormRect rect;
    rect.x_center = std::stof(r[4]);
    rect.y_center = std::stof(r[5]);
    rect.width = std::stof(r[6]);
    rect.height = std::stof(r[7]);
    rect.rotation = std::stof(r[8]);
    std::vector<Landmark> lms(21);
    for (int i = 0; i < 21; ++i) lms[i] = {std::stof(r[9 + 3 * i]), std::stof(r[10 + 3 * i]), std::stof(r[11 + 3 * i])};
    RemoveLetterbox(&lms, pad);
    const auto proj = ProjectLandmarks(lms, rect);
    const size_t b = 9 + 63;
    for (int i = 0; i < 21; ++i) {
      EXPECT_NEAR(proj[i].x, std::stod(r[b + 3 * i]), kTol) << i;
      EXPECT_NEAR(proj[i].y, std::stod(r[b + 3 * i + 1]), kTol) << i;
      EXPECT_EQ(static_cast<double>(proj[i].z), std::stod(r[b + 3 * i + 2])) << "z is kept as is";
    }
    const Detection det = LandmarksToDetection(proj);
    const size_t c = b + 63;
    EXPECT_NEAR(det.xmin, std::stod(r[c]), kTol);
    EXPECT_NEAR(det.ymin, std::stod(r[c + 1]), kTol);
    EXPECT_NEAR(det.width, std::stod(r[c + 2]), kTol);
    EXPECT_NEAR(det.height, std::stod(r[c + 3]), kTol);
  }
}

// --- structural properties of the calculators (metamorphic pairs) -------------------------------

TEST(HandGeometry, NormalizeRadiansWrapsIntoMinusPiToPi) {
  EXPECT_NEAR(NormalizeRadians(0.0f), 0.0f, 1e-7);
  EXPECT_NEAR(NormalizeRadians(3.5f), 3.5f - 2 * 3.14159265f, 1e-6);
  EXPECT_NEAR(NormalizeRadians(-3.5f), -3.5f + 2 * 3.14159265f, 1e-6);
  EXPECT_NEAR(NormalizeRadians(7.0f), 7.0f - 2 * 3.14159265f, 1e-5);
}

TEST(HandGeometry, UprightHandHasZeroRotationAndRotatingTheKeypointsRotatesTheRect) {
  // Wrist (kp 0) below the middle-finger MCP (kp 2) in image coordinates (y down): hand points up, so
  // the rect rotation is 0 (target angle 90 degrees minus atan2(-(y1 - y0), x1 - x0) = pi/2 - pi/2).
  Detection d;
  d.xmin = 0.4f;
  d.ymin = 0.4f;
  d.width = 0.2f;
  d.height = 0.2f;
  d.keypoints = {{0.5f, 0.7f}, {0.4f, 0.5f}, {0.5f, 0.5f}};
  const NormRect up = DetectionToNormRect(d, 100, 100, 0, 2, 90.0f);
  EXPECT_NEAR(up.rotation, 0.0f, 1e-6);
  // The MCP moves to the right of the wrist (the hand points right on screen, turned clockwise by 90
  // degrees; the image y axis points down): the rect rotation becomes +90 degrees, and pointing left
  // gives -90 degrees.
  d.keypoints[2] = {0.7f, 0.7f};
  const NormRect right = DetectionToNormRect(d, 100, 100, 0, 2, 90.0f);
  EXPECT_NEAR(right.rotation - up.rotation, 3.14159265f / 2, 1e-6);
  d.keypoints[2] = {0.3f, 0.7f};
  const NormRect left = DetectionToNormRect(d, 100, 100, 0, 2, 90.0f);
  EXPECT_NEAR(left.rotation - up.rotation, -3.14159265f / 2, 1e-6);
}

TEST(HandGeometry, SquareLongRectUsesTheLongSideInPixelsAndScalesIt) {
  NormRect r;
  r.x_center = 0.5f;
  r.y_center = 0.5f;
  r.width = 0.1f;   // 60 px of 600
  r.height = 0.2f;  // 160 px of 800
  RectTransformOptions o;
  o.scale_x = o.scale_y = 2.0f;
  o.square_long = true;
  const NormRect t = TransformNormRect(r, 600, 800, o);
  EXPECT_NEAR(t.width * 600, 2 * 160, 1e-3);
  EXPECT_NEAR(t.height * 800, 2 * 160, 1e-3);
}

TEST(HandGeometry, ProjectionIsIdentityForTheFullImageRect) {
  NormRect full;
  full.x_center = 0.5f;
  full.y_center = 0.5f;
  full.width = 1.0f;
  full.height = 1.0f;
  const std::vector<Landmark> in = {{0.1f, 0.2f, 3.0f}, {0.9f, 0.4f, -2.0f}};
  const auto out = ProjectLandmarks(in, full);
  EXPECT_NEAR(out[0].x, 0.1f, 1e-7);
  EXPECT_NEAR(out[1].y, 0.4f, 1e-7);
  EXPECT_EQ(out[1].z, -2.0f);
}

TEST(HandGeometry, NonMaxSuppressionKeepsDisjointBoxesAndMergesOverlappingOnes) {
  auto make = [](float score, float x, float y) {
    Detection d;
    d.score = score;
    d.xmin = x;
    d.ymin = y;
    d.width = 0.2f;
    d.height = 0.2f;
    d.keypoints = {{x, y}};
    return d;
  };
  const std::vector<Detection> dets = {make(0.9f, 0.1f, 0.1f), make(0.8f, 0.11f, 0.1f), make(0.7f, 0.6f, 0.6f)};
  const auto out = WeightedNonMaxSuppression(dets, 0.3f, 0.5f);
  ASSERT_EQ(out.size(), 2u);
  EXPECT_EQ(out[0].score, 0.9f);  // the first of the cluster keeps its score
  // Weighted box of the cluster: score-weighted mean of the two xmin values.
  EXPECT_NEAR(out[0].xmin, (0.1f * 0.9f + 0.11f * 0.8f) / 1.7f, 1e-6);
  EXPECT_NEAR(out[1].xmin, 0.6f, 1e-7);
  // Below the minimum score nothing survives.
  EXPECT_TRUE(WeightedNonMaxSuppression({make(0.4f, 0.1f, 0.1f)}, 0.3f, 0.5f).empty());
}

TEST(HandGeometry, DecodeRejectsScoresBelowThresholdAndClipsLogits) {
  const auto anchors = GenerateAnchors(SsdAnchorsOptions());
  std::vector<float> boxes(2944 * 18, 0.0f), scores(2944, -10.0f);
  scores[100] = 1000.0f;  // clipped to 100: sigmoid = 1
  scores[101] = -0.1f;    // sigmoid 0.475 < 0.5
  scores[102] = 0.0f;     // sigmoid exactly 0.5: kept (the original only rejects score < threshold)
  const auto dets = DecodePalmDetections(boxes.data(), scores.data(), anchors, PalmDecodeOptions());
  ASSERT_EQ(dets.size(), 2u);
  EXPECT_EQ(dets[0].score, 1.0f);
  EXPECT_EQ(dets[1].score, 0.5f);
  // With zero raw boxes every detection is the anchor point with zero size.
  EXPECT_NEAR(dets[0].xmin, anchors[100].x_center, 1e-7);
  EXPECT_EQ(dets[0].width, 0.0f);
}

}  // namespace
}  // namespace slam_refact::hand

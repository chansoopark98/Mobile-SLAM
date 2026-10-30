// Ear geometry against the public golden JSON files (US-019; docs/spec/08-face-ear.md §5 T2, T3).
//
// Port of PUB:reality/engine/faces/ear-tracker-test.cc TestCroppingImage / TestLiftingImage. The
// golden files are byte copies of PUB:reality/engine/faces/testdata/fidelity_testing_*.json
// (tests/face/data/PROVENANCE.md; the sha256 is checked below). The original builds the intrinsics
// from the size of reality/engine/testdata/real_test_face.jpg, a git-lfs pointer here, so every
// check runs for each 3:4 size of kSizes and must pass for all of them.
#include <gtest/gtest.h>

#include <cstdio>
#include <string>
#include <vector>

#include "c8/geometry/egomotion.h"
#include "c8/geometry/facemesh-data.h"
#include "c8/geometry/intrinsics-clip.h"
#include "c8/parameter-data.h"
#include "c8/pixels/render/object8.h"
#include "face/face_models.h"
#include "face/face_roi_renderer.h"
#include "face_test_util.h"
#include "reality/engine/ears/ear-types.h"
#include "reality/engine/faces/face-geometry.h"

namespace c8 {
namespace {

using slam_refact::face::testing::IPhone13;
using slam_refact::face::testing::Json;
using slam_refact::face::testing::ReadJsonFile;

constexpr float kTol = 0.0001f;  // checkPoint of ear-tracker-test.cc
constexpr int kSizes[][2] = {{480, 640},   {600, 800},   {720, 960},   {768, 1024},
                             {960, 1280},  {1080, 1440}, {1200, 1600}, {1536, 2048},
                             {2160, 2880}, {3024, 4032}};

const std::string kDataDir = SR_FACE_TEST_DATA_DIR;

HPoint2 ToHPoint2(const Json &j) {
  return {static_cast<float>(j[0].number), static_cast<float>(j[1].number)};
}
HPoint3 ToHPoint3(const Json &j) {
  return {static_cast<float>(j[0].number), static_cast<float>(j[1].number),
          static_cast<float>(j[2].number)};
}

Vector<HPoint3> Landmarks(const Json &root) {
  Vector<HPoint3> out;
  const Json &lm = root["input"]["mediapipe_3d_landmarks"];
  for (size_t i = 0; i < lm.size(); ++i) {
    out.push_back(ToHPoint3(lm[i]));
  }
  return out;
}

void LoadGolden(const std::string &name, const char *sha256, Json *out) {
  std::vector<uint8_t> bytes;
  std::string error;
  ASSERT_TRUE(slam_refact::face::ReadFileBytes(kDataDir + "/" + name, &bytes, &error)) << error;
  ASSERT_EQ(slam_refact::face::Sha256Hex(bytes), sha256) << name;
  ASSERT_TRUE(ReadJsonFile(kDataDir + "/" + name, out)) << name;
}

constexpr char kCroppingSha[] = "5a3bc9dc09375eedef42f1b879c6c6ded91726fc83a46408c50b54da3ce603fe";
constexpr char kLiftingSha[] = "1a57fce2f9a7ea49efbe0ef1f16f2401f2456edf1d93ad0a8dd382d5d23a3eff";

// ear-tracker-test.cc makeFaceDetectedPoints.
DetectedPoints MakeFaceDetectedPoints(const Vector<HPoint3> &landmarks, int width, int height) {
  auto scaledIntrinsic = Intrinsics::rotateCropAndScaleIntrinsics(IPhone13(), width, height);
  DetectedPoints d;
  d.points = landmarks;
  d.boundingBox = {{0.0f, 0.0f}, {1.0f, 0.0f}, {0.0f, 1.0f}, {1.0f, 1.0f}};
  d.roi = slam_refact::face::MakeImageRoi(ImageRoi::Source::FACE, 1, HMatrixGen::i());
  d.roi.intrinsics = HMatrixGen::intrinsic(scaledIntrinsic);
  d.viewport = {0, 0, static_cast<float>(width), static_cast<float>(height)};
  d.intrinsics = scaledIntrinsic;
  return d;
}

// ear-tracker-test.cc addEarDetectedPtsInfo.
void AddEarDetectedPtsInfo(DetectedPoints *d, const ImageRoi::Source &earSrc,
                           const c8_PixelPinholeCameraModel &intrinsics, int width, int height) {
  d->roi = slam_refact::face::MakeImageRoi(earSrc, 1, HMatrixGen::i());
  d->roi.intrinsics = HMatrixGen::intrinsic(intrinsics);
  d->viewport = {0, 0, static_cast<float>(width), static_cast<float>(height)};
  d->intrinsics = intrinsics;
}

// ear-tracker-test.cc makeEarDetectedPoints (including its second-point y quirk).
DetectedPoints MakeEarDetectedPoints(const HPoint3 &top, const HPoint3 &low, const HPoint2 &lobe,
                                     const HPoint2 &canal, const HPoint2 &helix, bool visible) {
  DetectedPoints earPts;
  const float visibleZ = visible ? 1.0f : 0.0f;
  earPts.points = {
    {top.x(), top.y(), 1.f},
    {low.x(), top.y(), 1.f},
    {lobe.x(), lobe.y(), visibleZ},
    {canal.x(), canal.y(), visibleZ},
    {helix.x(), helix.y(), visibleZ},
  };
  return earPts;
}

class EarTrackerTest : public ::testing::Test {
 protected:
  void SetUp() override {
    globalParams().set("EarTypes.filterEarAnchors", false);
    globalParams().set("EarTypes.earLandmarkDetectionCropAlpha", 0.6f);
  }
};

void CheckPoint(const HPoint2 &expected, const HPoint2 &actual, const std::string &what) {
  EXPECT_NEAR(expected.x(), actual.x(), kTol) << what;
  EXPECT_NEAR(expected.y(), actual.y(), kTol) << what;
}

TEST_F(EarTrackerTest, TestCroppingImage) {
  Json golden;
  LoadGolden("fidelity_testing_cropping.json", kCroppingSha, &golden);
  const auto landmarks = Landmarks(golden);
  ASSERT_EQ(478u, landmarks.size());
  EXPECT_FLOAT_EQ(0.39597558975219727, landmarks[0].x());
  EXPECT_FLOAT_EQ(0.5725057125091553, landmarks[0].y());
  EXPECT_FLOAT_EQ(-0.03312905505299568, landmarks[0].z());
  const Json &outL = golden["output"]["models_left"];
  const Json &outR = golden["output"]["models_right"];
  EXPECT_FLOAT_EQ(0.6523301601409912, ToHPoint2(outL["top_inner_corner"]).x());
  EXPECT_FLOAT_EQ(0.42510518431663513, ToHPoint2(outR["top_inner_corner"]).x());

  for (const auto &size : kSizes) {
    const std::string tag = std::to_string(size[0]) + "x" + std::to_string(size[1]);
    auto face = MakeFaceDetectedPoints(landmarks, size[0], size[1]);
    EarBoundingBoxDebug bb;
    auto earRois = earRoisByFaceMesh(face, &bb);
    EXPECT_EQ(2u, earRois.size());
    // Left: the original assertions.
    CheckPoint(ToHPoint2(outL["top_inner_corner"]), bb.left.upperLeft, tag + " left top_inner");
    CheckPoint(ToHPoint2(outL["top_outer_corner"]), bb.left.upperRight, tag + " left top_outer");
    CheckPoint(ToHPoint2(outL["bottom_outer_corner"]), bb.left.lowerRight,
               tag + " left bottom_outer");
    // Right: the corner order of the original WRITE_IMAGE debug code (debugRight).
    CheckPoint(ToHPoint2(outR["top_inner_corner"]), bb.right.upperRight, tag + " right top_inner");
    CheckPoint(ToHPoint2(outR["top_outer_corner"]), bb.right.upperLeft, tag + " right top_outer");
    CheckPoint(ToHPoint2(outR["bottom_outer_corner"]), bb.right.lowerLeft,
               tag + " right bottom_outer");
    // The ear ROIs are the left and right crops.
    EXPECT_EQ(ImageRoi::Source::EAR_LEFT, earRois[0].roi.source);
    EXPECT_EQ(ImageRoi::Source::EAR_RIGHT, earRois[1].roi.source);
  }
}

struct LiftingInput {
  Vector<HPoint3> landmarks;
  HPoint2 left[3];
  HPoint2 right[3];
  HPoint3 outLeft[3];
  HPoint3 outRight[3];
};

LiftingInput ReadLifting(const Json &golden) {
  LiftingInput in;
  in.landmarks = Landmarks(golden);
  const Json &il = golden["input"]["ear_2d_landmarks"]["models_left"];
  const Json &ir = golden["input"]["ear_2d_landmarks"]["models_right"];
  const Json &ol = golden["output"]["ear_3d_landmarks"]["models_left"];
  const Json &orr = golden["output"]["ear_3d_landmarks"]["models_right"];
  for (int i = 0; i < 3; ++i) {
    in.left[i] = ToHPoint2(il[i]);
    in.right[i] = ToHPoint2(ir[i]);
    in.outLeft[i] = ToHPoint3(ol[i]);
    in.outRight[i] = ToHPoint3(orr[i]);
  }
  return in;
}

// ear-tracker-test.cc makeFace: identity face anchor transform.
Face3d MakeFace(const Vector<HPoint3> &vertices) {
  Face3d face;
  face.vertices = vertices;
  face.status = Face3d::TrackingStatus::FOUND;
  face.transform = {{}, {}, 1.f, 1.f, 1.f, 1.f};
  return face;
}

// The mesh -> image matrix of earLiftLandmarksTo3D (ear-types.cc, texToClipYFlip :150-161).
HMatrix MeshToImage(const Face3d &face, const c8_PixelPinholeCameraModel &intrinsics) {
  const HMatrix texToClipYFlip{
    {2.0f, 0.0f, 0.0f, -1.0f}, {0.0f, -2.0f, 0.0f, 1.0f}, {0.0f, 0.0f, 1.0f, 0.0f},
    {0.0f, 0.0f, 0.0f, 1.0f},  {0.5f, 0.0f, 0.0f, 0.5f},  {0.0f, -0.5f, 0.0f, 0.5f},
    {0.0f, 0.0f, 1.0f, 0.0f},  {0.0f, 0.0f, 0.0f, 1.0f},
  };
  const HMatrix mv = trsMat(face.transform.position, face.transform.rotation, face.transform.scale);
  auto k = Intrinsics::rotateCropAndScaleIntrinsics(intrinsics, intrinsics.pixelsWidth,
                                                    intrinsics.pixelsHeight);
  auto proj =
    Intrinsics::toClipSpaceMatLeftHanded(k, CAM_PROJECTION_NEAR_CLIP, CAM_PROJECTION_FAR_CLIP);
  return texToClipYFlip.inv() * proj * mv;
}

TEST_F(EarTrackerTest, TestLiftingImage) {
  Json golden;
  LoadGolden("fidelity_testing_lifting3d.json", kLiftingSha, &golden);
  const auto in = ReadLifting(golden);
  ASSERT_EQ(478u, in.landmarks.size());
  EXPECT_FLOAT_EQ(0.39597558975219727, in.landmarks[0].x());
  EXPECT_FLOAT_EQ(0.7294090390205383, in.left[0].x());
  EXPECT_FLOAT_EQ(0.379135400056839, in.right[0].x());

  for (const auto &size : kSizes) {
    const int w = size[0];
    const int h = size[1];
    auto localFace = MakeFaceDetectedPoints(in.landmarks, w, h);
    DetectedPoints leftEar = MakeEarDetectedPoints(in.landmarks[FACEMESH_L_EAR],
      in.landmarks[FACEMESH_L_EAR_LOW], in.left[0], in.left[1], in.left[2], true);
    DetectedPoints rightEar = MakeEarDetectedPoints(in.landmarks[FACEMESH_R_EAR],
      in.landmarks[FACEMESH_R_EAR_LOW], in.right[0], in.right[1], in.right[2], false);
    auto k = Intrinsics::rotateCropAndScaleIntrinsics(IPhone13(), w, h);
    AddEarDetectedPtsInfo(&leftEar, ImageRoi::Source::EAR_LEFT, k, w, h);
    AddEarDetectedPtsInfo(&rightEar, ImageRoi::Source::EAR_RIGHT, k, w, h);
    Face3d face = MakeFace(localFace.points);
    auto ear3d = earLiftLandmarksTo3D(localFace, leftEar, rightEar, face, k);
    ASSERT_EQ(0u, ear3d.leftAttachmentPoints.size());
    ASSERT_EQ(0u, ear3d.rightAttachmentPoints.size());
    ASSERT_EQ(3u, ear3d.leftVertices.size());
    ASSERT_EQ(3u, ear3d.leftVisibilities.size());
    ASSERT_EQ(3u, ear3d.rightVertices.size());
    ASSERT_EQ(3u, ear3d.rightVisibilities.size());
    for (int i = 0; i < 3; ++i) {
      EXPECT_FLOAT_EQ(1.f, ear3d.leftVisibilities[i]);
      EXPECT_FLOAT_EQ(0.f, ear3d.rightVisibilities[i]);
    }
  }
}

// §5 T3 (b): face vertices = M^-1 L, both ears visible; M * lifted vertex must equal the golden
// 3D landmarks (x, y = the 2D input, z = the lifted depth).
TEST_F(EarTrackerTest, LiftedDepthMatchesGolden) {
  Json golden;
  LoadGolden("fidelity_testing_lifting3d.json", kLiftingSha, &golden);
  const auto in = ReadLifting(golden);
  for (const auto &size : kSizes) {
    const int w = size[0];
    const int h = size[1];
    const std::string tag = std::to_string(w) + "x" + std::to_string(h);
    auto k = Intrinsics::rotateCropAndScaleIntrinsics(IPhone13(), w, h);
    auto localFace = MakeFaceDetectedPoints(in.landmarks, w, h);
    Face3d face = MakeFace({});
    const HMatrix m = MeshToImage(face, k);
    face.vertices = m.inv() * in.landmarks;
    DetectedPoints leftEar = MakeEarDetectedPoints(in.landmarks[FACEMESH_L_EAR],
      in.landmarks[FACEMESH_L_EAR_LOW], in.left[0], in.left[1], in.left[2], true);
    DetectedPoints rightEar = MakeEarDetectedPoints(in.landmarks[FACEMESH_R_EAR],
      in.landmarks[FACEMESH_R_EAR_LOW], in.right[0], in.right[1], in.right[2], true);
    AddEarDetectedPtsInfo(&leftEar, ImageRoi::Source::EAR_LEFT, k, w, h);
    AddEarDetectedPtsInfo(&rightEar, ImageRoi::Source::EAR_RIGHT, k, w, h);
    auto ear3d = earLiftLandmarksTo3D(localFace, leftEar, rightEar, face, k);
    ASSERT_EQ(3u, ear3d.leftVertices.size());
    ASSERT_EQ(3u, ear3d.rightVertices.size());
    const auto leftImg = m * ear3d.leftVertices;
    const auto rightImg = m * ear3d.rightVertices;
    for (int i = 0; i < 3; ++i) {
      EXPECT_NEAR(in.outLeft[i].x(), leftImg[i].x(), kTol) << tag << " left " << i;
      EXPECT_NEAR(in.outLeft[i].y(), leftImg[i].y(), kTol) << tag << " left " << i;
      EXPECT_NEAR(in.outLeft[i].z(), leftImg[i].z(), kTol) << tag << " left " << i;
      EXPECT_NEAR(in.outRight[i].x(), rightImg[i].x(), kTol) << tag << " right " << i;
      EXPECT_NEAR(in.outRight[i].y(), rightImg[i].y(), kTol) << tag << " right " << i;
      EXPECT_NEAR(in.outRight[i].z(), rightImg[i].z(), kTol) << tag << " right " << i;
    }
  }
}

}  // namespace
}  // namespace c8

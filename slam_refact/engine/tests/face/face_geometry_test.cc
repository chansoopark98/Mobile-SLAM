// Port of PUB:reality/engine/faces/face-geometry-test.cc (US-018, docs/spec/08-face-ear.md §5 T1):
// same inputs, assertions and tolerances. Intrinsics::getCameraIntrinsics(APPLE_IPHONE_6) is the
// table value (face_test_util.h). TestRenderTexToRenderTex drew its inputs with rand(); it uses a
// glibc-equivalent generator here (face_test_util.h GlibcRand). That is NOT the input of the
// original run (unknown); about 17% of random inputs exceed the 4 ULP assertion
// (FaceT1InputDistribution, docs/spec/08-face-ear.md §8.6 G1), the first glibc input sits exactly
// at 4 ULP, and the result depends on the libm cosf/sinf rounding. The test is a platform pin and
// is UNVERIFIED as evidence of the port.
#include <gtest/gtest.h>

#include <array>
#include <cmath>

#include "c8/geometry/facemesh-data.h"
#include "c8/geometry/mesh.h"
#include "face/face_roi_renderer.h"
#include "face_test_util.h"
#include "reality/engine/faces/face-geometry.h"

namespace c8 {
namespace {

using slam_refact::face::testing::IPhone6;
using slam_refact::face::testing::GlibcRand;

// face-geometry-test.cc AreWithin(0.0001) applied pointwise (Pointwise matcher of the original).
template <size_t N>
::testing::AssertionResult AreWithin(const std::array<float, N> &a, const std::array<float, N> &b) {
  for (size_t i = 0; i < N; ++i) {
    if (!(std::fabs(a[i] - b[i]) < 0.0001f)) {
      return ::testing::AssertionFailure() << "element " << i << ": " << a[i] << " vs " << b[i];
    }
  }
  return ::testing::AssertionSuccess();
}

TEST(FaceGeometryTest, TestIntersectionOverUnion) {
  BoundingBox a = {{0.f, 1.f}, {1.f, 1.f}, {0.f, 0.f}, {1.f, 0.f}};
  BoundingBox b = {{-3.f, 1.f}, {-2.f, 1.f}, {-3.f, 0.f}, {-2.f, 0.f}};
  EXPECT_FLOAT_EQ(0.f, intersectionOverUnion(a, b));
  EXPECT_FLOAT_EQ(1.f, intersectionOverUnion(a, a));
  BoundingBox c = {{0.5f, 1.5f}, {1.5f, 1.5f}, {0.5f, 0.5f}, {1.5f, 0.5f}};
  EXPECT_FLOAT_EQ(0.1428571f, intersectionOverUnion(a, c));
}

TEST(FaceGeometryTest, uprightBBox) {
  BoundingBox a = {{0.f, 1.f}, {1.f, 1.f}, {0.f, 0.f}, {1.f, 0.f}};
  auto uprightA = uprightBBox(a);
  EXPECT_FLOAT_EQ(uprightA.upperLeft.x(), 0.f);
  EXPECT_FLOAT_EQ(uprightA.upperLeft.y(), 1.f);
  EXPECT_FLOAT_EQ(uprightA.upperRight.x(), 1.f);
  EXPECT_FLOAT_EQ(uprightA.upperRight.y(), 1.f);
  EXPECT_FLOAT_EQ(uprightA.lowerLeft.x(), 0.f);
  EXPECT_FLOAT_EQ(uprightA.lowerLeft.y(), 0.f);
  EXPECT_FLOAT_EQ(uprightA.lowerRight.x(), 1.f);
  EXPECT_FLOAT_EQ(uprightA.lowerRight.y(), 0.f);

  // Box rotated 45 degrees clockwise.
  BoundingBox b = {{0.5f, 1.f}, {1.f, 0.5f}, {0.f, 0.5f}, {0.5f, 0.f}};
  auto uprightB = uprightBBox(b);
  EXPECT_FLOAT_EQ(uprightB.upperLeft.x(), 0.f);
  EXPECT_FLOAT_EQ(uprightB.upperLeft.y(), 1.f);
  EXPECT_FLOAT_EQ(uprightB.upperRight.x(), 1.f);
  EXPECT_FLOAT_EQ(uprightB.upperRight.y(), 1.f);
  EXPECT_FLOAT_EQ(uprightB.lowerLeft.x(), 0.f);
  EXPECT_FLOAT_EQ(uprightB.lowerLeft.y(), 0.f);
  EXPECT_FLOAT_EQ(uprightB.lowerRight.x(), 1.f);
  EXPECT_FLOAT_EQ(uprightB.lowerRight.y(), 0.f);
}

TEST(FaceGeometryTest, TestMinBBoxDistance) {
  BoundingBox a = {{0.f, 1.f}, {1.f, 1.f}, {0.f, 0.f}, {1.f, 0.f}};
  BoundingBox b = {{-3.f, 1.f}, {-2.f, 1.f}, {-3.f, 0.f}, {-2.f, 0.f}};
  EXPECT_FLOAT_EQ(2.f, minBBoxDistance(a, b));
  EXPECT_FLOAT_EQ(0.f, minBBoxDistance(a, a));
  BoundingBox c = {{-3.f, -2.f}, {-2.f, -2.f}, {-3.f, -3.f}, {-2.f, -3.f}};
  EXPECT_FLOAT_EQ(2.8284271f, minBBoxDistance(a, c));
  BoundingBox d = {{0.5f, 1.5f}, {1.5f, 1.5f}, {0.5f, 0.5f}, {1.5f, 0.5f}};
  EXPECT_FLOAT_EQ(0.f, minBBoxDistance(a, d));
  BoundingBox e = {{1.f, 1.f}, {2.f, 1.f}, {1.f, 0.f}, {2.f, 0.f}};
  EXPECT_FLOAT_EQ(0.f, minBBoxDistance(a, e));
}

TEST(FaceGeometryTest, TestDetectionToImageSpace) {
  DetectedPoints ptsWithBb;
  ptsWithBb.intrinsics = IPhone6();
  ptsWithBb.boundingBox = {{0.2f, 0.3f}, {0.3f, 0.3f}, {0.2f, 0.5f}, {0.3f, 0.5f}};
  auto transform = detectionRoiNoPadding(ptsWithBb);

  DetectedPoints a{
    1.0f,
    0,
    {0, 0, 128, 128},
    slam_refact::face::MakeImageRoi(ImageRoi::Source::FACE, 0, HMatrixGen::i()),
    {{.25f, 0.5f}, {.75f, 0.5f}, {.25f, 1.0f}, {.75f, 1.0f}},
    {{0.5f, 0.75f, 0.0f}, {0.75f, 0.5f, -1.0f}},
    IPhone6(),
    {},
  };
  DetectedPoints b{
    0.8f,
    0,
    {32, 64, 128, 256},
    transform,
    {{0.0f, 0.0f}, {1.0f, 0.0f}, {0.0f, 1.0f}, {1.0f, 1.0f}},
    {{0.5f, 0.25f, 1.0f}, {0.75f, 1.0f, 0.0f}},
    IPhone6(),
    {},
  };
  Vector<DetectedPoints> d{a, b};

  auto ia = detectionToImageSpace(a);
  auto ib = detectionToImageSpace(b);
  auto id = detectionToImageSpace(d);
  BoundingBox bImageBbox = detectionBoundingBoxToImageSpace(b.boundingBox, b.roi);

  Vector<DetectedImagePoints> as = {ia, id[0]};
  for (const auto &p : as) {
    EXPECT_FLOAT_EQ(1, p.confidence);
    EXPECT_FLOAT_EQ(0, p.viewport.x);
    EXPECT_FLOAT_EQ(0, p.viewport.y);
    EXPECT_FLOAT_EQ(128, p.viewport.w);
    EXPECT_FLOAT_EQ(128, p.viewport.h);
    EXPECT_TRUE(AreWithin(p.roi.warp.data(), HMatrixGen::i().data()));
    EXPECT_FLOAT_EQ(.25f, p.boundingBox.upperLeft.x());
    EXPECT_FLOAT_EQ(.5f, p.boundingBox.upperLeft.y());
    EXPECT_FLOAT_EQ(.5f, p.boundingBox.width());
    EXPECT_FLOAT_EQ(.5f, p.boundingBox.height());
    EXPECT_FLOAT_EQ(0.5f, p.points[0].x());
    EXPECT_FLOAT_EQ(0.75f, p.points[0].y());
    EXPECT_FLOAT_EQ(0, p.points[0].z());
    EXPECT_FLOAT_EQ(0.75f, p.points[1].x());
    EXPECT_FLOAT_EQ(0.5f, p.points[1].y());
    EXPECT_FLOAT_EQ(-1, p.points[1].z());
  }

  Vector<DetectedImagePoints> bs = {ib, id[1]};
  for (const auto &p : bs) {
    EXPECT_FLOAT_EQ(0.8f, p.confidence);
    EXPECT_FLOAT_EQ(32, p.viewport.x);
    EXPECT_FLOAT_EQ(64, p.viewport.y);
    EXPECT_FLOAT_EQ(128, p.viewport.w);
    EXPECT_FLOAT_EQ(256, p.viewport.h);
    EXPECT_TRUE(AreWithin(p.roi.warp.data(), transform.warp.data()));
    EXPECT_FLOAT_EQ(0.2f, p.boundingBox.upperLeft.x());
    EXPECT_FLOAT_EQ(0.3f, p.boundingBox.upperLeft.y());
    EXPECT_FLOAT_EQ(0.1f, p.boundingBox.width());
    EXPECT_FLOAT_EQ(0.2f, p.boundingBox.height());
    EXPECT_FLOAT_EQ(0.25f, p.points[0].x());
    EXPECT_FLOAT_EQ(0.35f, p.points[0].y());
    EXPECT_FLOAT_EQ(1, p.points[0].z());
    EXPECT_FLOAT_EQ(0.275f, p.points[1].x());
    EXPECT_FLOAT_EQ(0.5f, p.points[1].y());
    EXPECT_FLOAT_EQ(0, p.points[1].z());
  }

  EXPECT_FLOAT_EQ(ib.boundingBox.upperLeft.x(), bImageBbox.upperLeft.x());
  EXPECT_FLOAT_EQ(ib.boundingBox.upperRight.x(), bImageBbox.upperRight.x());
  EXPECT_FLOAT_EQ(ib.boundingBox.lowerLeft.x(), bImageBbox.lowerLeft.x());
  EXPECT_FLOAT_EQ(ib.boundingBox.lowerRight.x(), bImageBbox.lowerRight.x());
}

TEST(FaceGeometryTest, TestComputeAnchorTransform) {
  DetectedPoints a{
    1.0f,
    0,
    {0, 0, 128, 128},
    slam_refact::face::MakeImageRoi(ImageRoi::Source::FACE, 0, HMatrixGen::i()),
    {{.25f, 0.5f}, {.75f, 0.5f}, {.25f, 1.0f}, {.75f, 1.0f}},
    {{0.5f, 0.75f, 0.0f}, {0.75f, 0.5f, -1.0f}},
    IPhone6(),
    {},
  };

  auto rayDetections = detectionToRaySpace(a);
  auto faceAnchor = computeAnchorTransform(rayDetections);
  EXPECT_FLOAT_EQ(faceAnchor.position.x(), 0.0f * 0.32625365f);
  EXPECT_FLOAT_EQ(faceAnchor.position.y(), -0.28418082f * 0.32625365f);
  EXPECT_FLOAT_EQ(faceAnchor.position.z(), 0.32625365f);
  EXPECT_FLOAT_EQ(faceAnchor.rotation.w(), 0.0f);
  EXPECT_FLOAT_EQ(faceAnchor.rotation.x(), 0.0f);
  EXPECT_FLOAT_EQ(faceAnchor.rotation.y(), 1.0f);
  EXPECT_FLOAT_EQ(faceAnchor.rotation.z(), 0.0f);
  EXPECT_FLOAT_EQ(faceAnchor.scale, FACE_WIDTH);
  EXPECT_FLOAT_EQ(faceAnchor.scaledWidth, 1.0f);

  DetectedPoints ptsWithBb;
  ptsWithBb.boundingBox = {{0.2f, 0.3f}, {0.3f, 0.3f}, {0.2f, 0.5f}, {0.3f, 0.5f}};
  ptsWithBb.intrinsics = IPhone6();
  auto transform = detectionRoiNoPadding(ptsWithBb);

  DetectedPoints b{
    0.8f,
    0,
    {32, 64, 128, 256},
    transform,
    {{0.0f, 0.0f}, {1.0f, 0.0f}, {0.0f, 1.0f}, {1.0f, 1.0f}},
    {{0.5f, 0.25f, 1.0f}, {0.75f, 1.0f, 0.0f}},
    IPhone6(),
    {},
  };

  rayDetections = detectionToRaySpace(b);
  faceAnchor = computeAnchorTransform(rayDetections);
  EXPECT_FLOAT_EQ(faceAnchor.position.x(), -0.21302444f * 1.6312683f);
  EXPECT_FLOAT_EQ(faceAnchor.position.y(), 0.11367232f * 1.6312683f);
  EXPECT_FLOAT_EQ(faceAnchor.position.z(), 1.6312683f);
  EXPECT_FLOAT_EQ(faceAnchor.rotation.w(), 0.0f);
  EXPECT_FLOAT_EQ(faceAnchor.rotation.x(), 0.0f);
  EXPECT_FLOAT_EQ(faceAnchor.rotation.y(), 1.0f);
  EXPECT_FLOAT_EQ(faceAnchor.rotation.z(), 0.0f);
  EXPECT_FLOAT_EQ(faceAnchor.scale, FACE_WIDTH);
  EXPECT_FLOAT_EQ(faceAnchor.scaledWidth, 1.0f);
}

TEST(FaceGeometryTest, TestDetectionRoi) {
  DetectedPoints ptsWithBb;
  ptsWithBb.boundingBox = {{0.2f, 0.3f}, {0.3f, 0.3f}, {0.2f, 0.5f}, {0.3f, 0.5f}};
  ptsWithBb.intrinsics = IPhone6();
  DetectedPoints ptsWithRoi;
  ptsWithRoi.roi = detectionRoi(ptsWithBb);
  auto roiBox = roiCornersInImage(ptsWithRoi);
  EXPECT_TRUE(AreWithin(roiBox[0].data(), HPoint2{0.175f, 0.25f}.data()));
  EXPECT_TRUE(AreWithin(roiBox[1].data(), HPoint2{0.175f, 0.55f}.data()));
  EXPECT_TRUE(AreWithin(roiBox[2].data(), HPoint2{0.325f, 0.55f}.data()));
  EXPECT_TRUE(AreWithin(roiBox[3].data(), HPoint2{0.325f, 0.25f}.data()));
}

TEST(FaceGeometryTest, TestDetectionRoiNoPadding) {
  DetectedPoints ptsWithBb;
  ptsWithBb.intrinsics = IPhone6();
  ptsWithBb.boundingBox = {{0.2f, 0.3f}, {0.3f, 0.3f}, {0.2f, 0.5f}, {0.3f, 0.5f}};
  DetectedPoints ptsWithRoi = ptsWithBb;
  ptsWithRoi.roi = detectionRoiNoPadding(ptsWithBb);
  auto roiBox = roiCornersInImage(ptsWithRoi);
  EXPECT_TRUE(AreWithin(roiBox[0].data(), HPoint2{0.2f, 0.3f}.data()));
  EXPECT_TRUE(AreWithin(roiBox[1].data(), HPoint2{0.2f, 0.5f}.data()));
  EXPECT_TRUE(AreWithin(roiBox[2].data(), HPoint2{0.3f, 0.5f}.data()));
  EXPECT_TRUE(AreWithin(roiBox[3].data(), HPoint2{0.3f, 0.3f}.data()));
}

TEST(FaceGeometryTest, TestDetectionToMeshNoFilter) {
  DetectedPoints a{
    1.0f,
    0,
    {0, 0, 128, 128},
    slam_refact::face::MakeImageRoi(ImageRoi::Source::FACE, 0, HMatrixGen::i()),
    {{.25f, 0.5f}, {.75f, 0.5f}, {.25f, 1.0f}, {.75f, 1.0f}},
    FACEMESH_SAMPLE_OUTPUT,
    IPhone6(),
    {},
  };
  // The original only runs it (no assertion); the port also checks the output sizes.
  const Face3d faceData = detectionToMeshNoFilter(a);
  EXPECT_EQ(478u, faceData.vertices.size());
  EXPECT_EQ(478u, faceData.normals.size());
  EXPECT_EQ(29u, faceData.attachmentPoints.size());
}

void checkAttachment(
  AttachmentPointMsg::AttachmentName name, int index, const Vector<HVector3> &normals) {
  auto pt = createAttachmentPoint(name, index, BLAZEFACE_SAMPLE_VERTICES, normals);
  EXPECT_TRUE(AreWithin(pt.position.data(), BLAZEFACE_SAMPLE_VERTICES[index].data()));
  HPoint3 up{0.0f, 1.0f, 0.0f};
  auto normalUp = pt.rotation.toRotationMat() * up;
  EXPECT_NEAR(normalUp.x(), normals[index].x(), 1e-6);
  EXPECT_NEAR(normalUp.y(), normals[index].y(), 1e-6);
  EXPECT_NEAR(normalUp.z(), normals[index].z(), 1e-6);
}

TEST(FaceGeometryTest, TestCreateAttachmentPoint) {
  Vector<HVector3> normals;
  computeVertexNormals(BLAZEFACE_SAMPLE_VERTICES, BLAZEFACE_INDICES, &normals);
  checkAttachment(AttachmentPointMsg::AttachmentName::LEFT_EYE, BLAZEFACE_L_EYE, normals);
  checkAttachment(AttachmentPointMsg::AttachmentName::RIGHT_EYE, BLAZEFACE_R_EYE, normals);
  checkAttachment(AttachmentPointMsg::AttachmentName::NOSE_TIP, BLAZEFACE_NOSE, normals);
  checkAttachment(AttachmentPointMsg::AttachmentName::MOUTH, BLAZEFACE_MOUTH, normals);
  checkAttachment(AttachmentPointMsg::AttachmentName::LEFT_EAR, BLAZEFACE_L_EAR, normals);
  checkAttachment(AttachmentPointMsg::AttachmentName::RIGHT_EAR, BLAZEFACE_R_EAR, normals);
}

TEST(FaceGeometryTest, GlibcRandMatchesTheGlibcSequence) {
  // rand() % 256 of glibc 2.35 without srand (gcc test program, 2026-10-19).
  GlibcRand rng;
  const int expected[12] = {103, 198, 105, 115, 81, 255, 74, 236, 41, 205, 186, 171};
  for (int e : expected) {
    EXPECT_EQ(e, rng.Next() % 256);
  }
  GlibcRand again;
  EXPECT_EQ(1804289383, again.Next());
}

// PLATFORM PIN, not evidence of the port (review RM-7 MINOR 4): the 4 ULP assertion holds for the
// first glibc rand() input only because cosf/sinf round that way on this libm; +-1 ULP in cos/sin
// breaks it (docs/spec/08-face-ear.md §8.6 G1). A libm, compiler or platform change may fail it
// without any regression of the vendored code, which is byte identical to the original.
TEST(FaceGeometryTest, TestRenderTexToRenderTex) {
  ImageViewport viewport{0, 0, 256, 256};
  auto k = IPhone6();
  GlibcRand rng;
  auto rand0to1 = [&rng]() { return static_cast<float>(rng.Next() % 256) / 256.0f; };

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
  EXPECT_EQ(originImgPts.points.size(), newImgPts.points.size());
  for (size_t i = 0; i < originImgPts.points.size(); ++i) {
    EXPECT_FLOAT_EQ(originImgPts.points[i].x(), newImgPts.points[i].x());
    EXPECT_FLOAT_EQ(originImgPts.points[i].y(), newImgPts.points[i].y());
  }
}

}  // namespace
}  // namespace c8

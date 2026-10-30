// Ports of the public face/ear unit tests that need no model or image (US-018/US-019,
// docs/spec/08-face-ear.md §5 T1):
//   PUB:reality/engine/faces/face-tracker-test.cc  TestRecordFaceBBoxInRay, TestFilterGlobalAndAssignIds*
//   PUB:reality/engine/faces/tracked-face-state-test.cc  TestLocate
//   PUB:reality/engine/ears/ear-types-test.cc  TestMirrorInvisibleEarVertices (rand() -> fixed LCG)
//   PUB:reality/engine/ears/ear-roi-renderer-test.cc  TestViewportForCrop (on the CPU port)
// TestMesh / TestFaces (face-tracker-test) and the detector tests need git-lfs images: BLOCKED.
#include <gtest/gtest.h>

#include "c8/geometry/facemesh-data.h"
#include "face/face_roi_renderer.h"
#include "face_test_util.h"
#include "reality/engine/ears/ear-types.h"
#include "reality/engine/faces/face-tracker.h"
#include "reality/engine/faces/tracked-face-state.h"

namespace c8 {
namespace {

using slam_refact::face::testing::IPhone6;
using slam_refact::face::testing::Lcg;

BoundingBox BoxA() { return {{0.5f, 1.f}, {1.f, 0.5f}, {0.f, 0.5f}, {0.5f, 0.f}}; }
BoundingBox BoxB() { return {{-0.5f, 1.f}, {0.f, 0.5f}, {-1.f, 0.5f}, {-0.5f, 0.f}}; }

TEST(FaceTrackerTest, TestRecordFaceBBoxInRay) {
  DetectedPoints globalA;
  globalA.boundingBox = BoxA();
  globalA.roi.faceId = 2;
  TreeMap<int, BoundingBox> faceToUprightBBoxInRay;
  recordFaceBBoxInRay({globalA}, &faceToUprightBBoxInRay);
  EXPECT_FLOAT_EQ(faceToUprightBBoxInRay.at(2).upperLeft.x(), -1.f);
  EXPECT_FLOAT_EQ(faceToUprightBBoxInRay.at(2).upperLeft.y(), 0.f);
  EXPECT_FLOAT_EQ(faceToUprightBBoxInRay.at(2).upperRight.x(), 0.f);
  EXPECT_FLOAT_EQ(faceToUprightBBoxInRay.at(2).upperRight.y(), 0.f);
  EXPECT_FLOAT_EQ(faceToUprightBBoxInRay.at(2).lowerLeft.x(), -1.f);
  EXPECT_FLOAT_EQ(faceToUprightBBoxInRay.at(2).lowerLeft.y(), -1.f);
  EXPECT_FLOAT_EQ(faceToUprightBBoxInRay.at(2).lowerRight.x(), 0.f);
  EXPECT_FLOAT_EQ(faceToUprightBBoxInRay.at(2).lowerRight.y(), -1.f);
}

TEST(FaceTrackerTest, TestFilterGlobalAndAssignIdsEmpty) {
  const Vector<DetectedPoints> globalDetections;
  const Vector<DetectedPoints> previousGlobalDetections;
  TreeMap<int, TrackedFaceState> facesById;
  TreeMap<int, BoundingBox> faceToUprightBBoxInRay;
  TreeMap<int, int> faceIdToFirstFrame;
  auto filteredGlobal = filterGlobalAndAssignIds(globalDetections, facesById,
    previousGlobalDetections, 1, 1, &faceToUprightBBoxInRay, &faceIdToFirstFrame);
  EXPECT_TRUE(filteredGlobal.empty());
  EXPECT_TRUE(faceToUprightBBoxInRay.empty());
}

TEST(FaceTrackerTest, TestFilterGlobalAndAssignIdsRedundantGlobal) {
  DetectedPoints globalA;
  globalA.boundingBox = BoxA();
  DetectedPoints globalB;
  globalB.boundingBox = BoxB();
  DetectedPoints localB;
  localB.boundingBox = BoxB();
  localB.roi.faceId = 1;
  const Vector<DetectedPoints> globalDetections = {globalA, globalB};
  const Vector<DetectedPoints> previousGlobalDetections;
  TreeMap<int, TrackedFaceState> facesById;
  facesById[1];
  TreeMap<int, BoundingBox> faceToUprightBBoxInRay;
  TreeMap<int, int> faceIdToFirstFrame;
  recordFaceBBoxInRay({localB}, &faceToUprightBBoxInRay);
  auto filteredGlobal = filterGlobalAndAssignIds(globalDetections, facesById,
    previousGlobalDetections, 2, 2, &faceToUprightBBoxInRay, &faceIdToFirstFrame);
  EXPECT_EQ(filteredGlobal.size(), 1u);
  EXPECT_EQ(filteredGlobal[0].roi.faceId, 2);
  EXPECT_EQ(faceToUprightBBoxInRay.size(), 2u);
}

TEST(FaceTrackerTest, TestFilterGlobalAndAssignIdsTestNewGlobal) {
  DetectedPoints globalA;
  globalA.boundingBox = BoxA();
  DetectedPoints globalB;
  globalB.boundingBox = BoxB();
  const Vector<DetectedPoints> globalDetections = {globalA, globalB};
  const Vector<DetectedPoints> previousGlobalDetections;
  TreeMap<int, TrackedFaceState> facesById;
  TreeMap<int, BoundingBox> faceToUprightBBoxInRay;
  TreeMap<int, int> faceIdToFirstFrame;
  auto filteredGlobal = filterGlobalAndAssignIds(globalDetections, facesById,
    previousGlobalDetections, 1, 1, &faceToUprightBBoxInRay, &faceIdToFirstFrame);
  EXPECT_EQ(filteredGlobal.size(), 2u);
  EXPECT_EQ(filteredGlobal[0].roi.faceId, 1);
  EXPECT_EQ(filteredGlobal[1].roi.faceId, 2);
  EXPECT_EQ(faceToUprightBBoxInRay.size(), 2u);
}

TEST(FaceTrackerTest, TestFilterGlobalAndAssignIdsTestMissedGlobalNPlus1) {
  DetectedPoints globalA;
  globalA.boundingBox = BoxA();
  DetectedPoints globalB;
  globalB.boundingBox = BoxB();
  const Vector<DetectedPoints> globalDetections = {globalA};
  Vector<DetectedPoints> previousGlobalDetections = {globalA, globalB};
  previousGlobalDetections.front().roi.faceId = 1;
  previousGlobalDetections.back().roi.faceId = 2;
  TreeMap<int, TrackedFaceState> facesById;
  TreeMap<int, BoundingBox> faceToUprightBBoxInRay;
  TreeMap<int, int> faceIdToFirstFrame = {{0, 1}, {0, 2}};
  auto filteredGlobal = filterGlobalAndAssignIds(globalDetections, facesById,
    previousGlobalDetections, 3, 1, &faceToUprightBBoxInRay, &faceIdToFirstFrame);
  EXPECT_EQ(filteredGlobal.size(), 2u);
  EXPECT_EQ(filteredGlobal[0].roi.faceId, 1);
  EXPECT_EQ(filteredGlobal[1].roi.faceId, 2);
}

TEST(FaceTrackerTest, TestFilterGlobalAndAssignIdsTestMissedGlobalNPlus2) {
  DetectedPoints globalA;
  globalA.boundingBox = BoxA();
  DetectedPoints globalB;
  globalB.boundingBox = BoxB();
  const Vector<DetectedPoints> globalDetections;
  Vector<DetectedPoints> previousGlobalDetections = {globalA, globalB};
  previousGlobalDetections.front().roi.faceId = 1;
  previousGlobalDetections.back().roi.faceId = 2;
  TreeMap<int, TrackedFaceState> facesById;
  facesById[1];
  TreeMap<int, BoundingBox> faceToUprightBBoxInRay;
  TreeMap<int, int> faceIdToFirstFrame = {{1, 0}, {2, 1}};
  auto filteredGlobal = filterGlobalAndAssignIds(globalDetections, facesById,
    previousGlobalDetections, 3, 2, &faceToUprightBBoxInRay, &faceIdToFirstFrame);
  EXPECT_EQ(filteredGlobal.size(), 1u);
  EXPECT_EQ(filteredGlobal[0].roi.faceId, 2);
}

TEST(TrackedFaceStateTest, TestLocate) {
  ImageRoi roi = slam_refact::face::MakeImageRoi(ImageRoi::Source::FACE, 2,
                                                 {{3.631096f, 0.000000f, 0.000000f, -1.445287f},
                                                  {0.000000f, 4.841462f, 0.000000f, 2.579028f},
                                                  {0.000000f, 0.000000f, 1.000000f, 0.000000f},
                                                  {0.000000f, 0.000000f, 0.000000f, 1.000000f}});
  DetectedPoints p{1.0f, 0, {0.0f, 192.0f, 192.0f, 192.0f}, roi,
                   {{.25f, 0.5f}, {.75f, 0.5f}, {.25f, 1.0f}, {1.0f, 1.0f}},
                   FACEMESH_SAMPLE_OUTPUT, IPhone6(), {}};
  TrackedFaceState s;
  auto tracked = s.locateFace(p);
  EXPECT_EQ(478u, tracked.vertices.size());
  EXPECT_EQ(478u, tracked.uvsInCameraSpace.size());
  EXPECT_EQ(2, tracked.id);
  EXPECT_EQ(Face3d::TrackingStatus::FOUND, tracked.status);
  EXPECT_EQ(Face3d::TrackingStatus::FOUND, s.status());

  s.markFrame();
  tracked = s.locateFace(p);
  EXPECT_EQ(478u, tracked.vertices.size());
  EXPECT_EQ(478u, tracked.uvsInCameraSpace.size());
  EXPECT_EQ(2, tracked.id);
  EXPECT_EQ(Face3d::TrackingStatus::UPDATED, tracked.status);
  EXPECT_EQ(Face3d::TrackingStatus::UPDATED, s.status());

  s.markFrame();
  EXPECT_EQ(Face3d::TrackingStatus::LOST, s.status());
}

TEST(EarTypesTest, TestMirrorInvisibleEarVertices) {
  Lcg lcg(7u);
  auto random0To1Point = [&lcg]() -> HPoint3 {
    const float x = lcg.Next();
    const float y = lcg.Next();
    const float z = lcg.Next();
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
  constexpr float delta = 0.00001f;
  mirrorInvisibleEarVertices(leftRefPt, rightRefPt, &ear3d);
  for (size_t i = 0; i < ear3d.leftVertices.size(); ++i) {
    EXPECT_NEAR(ear3d.leftVertices[i].x() - leftRefPt.x(),
                rightRefPt.x() - ear3d.rightVertices[i].x(), delta);
    EXPECT_NEAR(ear3d.leftVertices[i].y() - leftRefPt.y(),
                ear3d.rightVertices[i].y() - rightRefPt.y(), delta);
    EXPECT_NEAR(ear3d.leftVertices[i].z() - leftRefPt.z(),
                ear3d.rightVertices[i].z() - rightRefPt.z(), delta);
  }
}

TEST(EarRoiRendererTest, TestViewportForCrop) {
  const float w = static_cast<float>(EAR_LANDMARK_DETECTION_INPUT_WIDTH);
  const float h = static_cast<float>(EAR_LANDMARK_DETECTION_INPUT_HEIGHT);
  ImageViewport e0 = slam_refact::face::EarViewportForCrop(0);
  EXPECT_FLOAT_EQ(e0.x, 0);
  EXPECT_FLOAT_EQ(e0.y, 0);
  EXPECT_FLOAT_EQ(e0.w, w);
  EXPECT_FLOAT_EQ(e0.h, h);
  ImageViewport e1 = slam_refact::face::EarViewportForCrop(1);
  EXPECT_FLOAT_EQ(e1.x, 0);
  EXPECT_FLOAT_EQ(e1.y, h);
  EXPECT_FLOAT_EQ(e1.w, w);
  EXPECT_FLOAT_EQ(e1.h, h);
  ImageViewport e3 = slam_refact::face::EarViewportForCrop(3);
  EXPECT_FLOAT_EQ(e3.x, w);
  EXPECT_FLOAT_EQ(e3.y, h);
  EXPECT_FLOAT_EQ(e3.w, w);
  EXPECT_FLOAT_EQ(e3.h, h);
}

}  // namespace
}  // namespace c8

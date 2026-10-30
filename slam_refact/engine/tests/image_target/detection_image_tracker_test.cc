// Port of PUB:reality/engine/imagedetection/detection-image-tracker-test.cc (US-011).
//
// Differences from the public test (test bodies, motions and tolerances unchanged):
// - loadTestImage(): the GL DetectionImageLoader on testdata/flower-big.jpg (a git-lfs pointer
//   locally) is replaced by image_target::TargetLoader (CPU pyramid port, model kIeee) on a
//   SyntheticTarget() of the same size (1440x1920), planar metadata as in the public test.
// - EXPECT_THAT(m.data(), equalsMatrix(e, t)) -> EXPECT_TRUE(MatrixWithin(m, e, t)) (same
//   per-element |a - e| < t rule; gmock is not built). The default threshold stays 1e-6.
// - testK(): GOOGLE_PIXEL3 from the engine device table (frontend::CameraIntrinsicsForModel).

#include <gtest/gtest.h>

#include <utility>

#include "c8/c8-log.h"
#include "c8/geometry/egomotion.h"
#include "c8/random-numbers.h"
#include "image_target/image_target_spec.h"
#include "image_target/target_loader.h"
#include "reality/engine/imagedetection/detection-image-tracker.h"
#include "reality/engine/imagedetection/detection-image.h"
#include "test_util.h"

namespace c8 {
namespace {

using ::slam_refact::image_target::testing::MatrixWithin;
using ::slam_refact::image_target::testing::TestK;

constexpr const char *TARGET_NAME = "tracker-test-target";

// Take a virtual picture of an image target; The target's points are in src. The center axis of the
// target is at targetLoc. The points are scaled relative to the source with scale.  The photo is
// taken at camPos.
FrameWithPoints image(
  const TargetWithPoints &src, const HMatrix &targetLoc, float scale, const HMatrix &camPos) {
  auto k = TestK();
  FrameWithPoints ret{k};
  auto K = HMatrixGen::intrinsic(k);
  for (size_t i = 0; i < src.points().size(); ++i) {
    const auto &pt = src.points()[i];
    HPoint3 tpt{pt.x() * scale, pt.y() * scale, 0.0f};
    auto wpt = targetLoc * tpt;
    auto campt = camPos.inv() * wpt;
    if (campt.z() < 0) {
      continue;
    }
    auto impt = (K * camPos.inv() * wpt).flatten();
    if (impt.x() < 0 || impt.y() < 0 || impt.x() >= k.pixelsWidth || impt.y() >= k.pixelsHeight) {
      continue;
    }
    ret.addImagePixelPoint(
      impt, pt.scale(), pt.angle(), pt.gravityAngle(), src.store().getClone(i));
  }
  C8Log("[detection-image-tracker-test] Imaged %d points", static_cast<int>(ret.points().size()));
  return ret;
}

// Take a virtual picture of an image target at a location relative to an imagined camera that took
// the photo. The target's points are in src. The points are scaled relative to the source with
// scale.  The photo is taken at camPos, where the origin is an imagined camera that took the
// original photo of the image target.
FrameWithPoints image(const TargetWithPoints &src, float scale, const HMatrix &pos) {
  return image(src, HMatrixGen::translation(0.0f, 0.0f, scale), scale, pos);
}

// Loads the synthetic test image and extracts features and descriptors with the CPU loader.
DetectionImage loadTestImage() {
  static const ::slam_refact::image_target::GrayImage img =
    ::slam_refact::image_target::testing::SyntheticTarget(1440, 1920, 7);
  ::slam_refact::image_target::ImageTargetSpec spec;
  spec.name = TARGET_NAME;
  spec.type = ::slam_refact::image_target::ImageTargetType::kPlanar;
  return ::slam_refact::image_target::LoadDetectionImage(
    spec,
    img.view(),
    ::slam_refact::image_target::testing::TestPinholeK(),
    ::slam_refact::ShaderNumericModel::kIeee);
}

// Tests that an image taken from the same location as the original camera that took the image
// image target photo returns identity as the tracked possition.
TEST(DetectionImageTrackerTest, TrackIdentity) {
  // Load the test image.
  ScopeTimer rt("test");
  RandomNumbers r;
  DetectionImageMap targets;
  targets.insert(std::make_pair(TARGET_NAME, loadTestImage()));

  // Take a photo from the same location as the source camera.
  auto img = image(targets.at(TARGET_NAME).framePoints(), 1.0f, HMatrixGen::i());

  // Check that identity is returned as the tracked camera location. Tracking a first time tests
  // global mode tracking.
  DetectionImageTracker t;
  auto res = t.track(img, {}, targets, &r);
  EXPECT_TRUE(MatrixWithin(res.pose, HMatrixGen::i(), 1e-6f));

  // Check that identity is returned as the tracked camera location. Tracking a second time tests
  // local mode tracking.
  res = t.track(img, {}, targets, &r);
  EXPECT_TRUE(MatrixWithin(res.pose, HMatrixGen::i(), 1e-6f));
}

// Tests that an image taken from the same location as the original camera that took the image
// image target photo, but at a different scale, returns identity as the tracked possition.
TEST(DetectionImageTrackerTest, TrackScaledIdentity) {
  // Load the test image.
  ScopeTimer rt("test");
  RandomNumbers r;
  DetectionImageMap targets;
  targets.insert(std::make_pair(TARGET_NAME, loadTestImage()));

  // Take a photo from the same location as the source camera, scaling the world size of the image
  // by 1/5.
  auto img = image(targets.at(TARGET_NAME).framePoints(), 0.2f, HMatrixGen::i());

  // Check that identity is returned as the tracked camera location. Tracking a second time tests
  // local mode tracking.
  DetectionImageTracker t;
  auto res = t.track(img, {}, targets, &r);
  EXPECT_TRUE(MatrixWithin(res.pose, HMatrixGen::i(), 1e-6f));

  // Check that identity is returned as the tracked camera location. Tracking a second time tests
  // local mode tracking.
  res = t.track(img, {}, targets, &r);
  EXPECT_TRUE(MatrixWithin(res.pose, HMatrixGen::i(), 1e-6f));
}

// Tests that an image taken from a different location as the original camera that took the image
// image target photo, and at the same scale, recovers the relative position of the camera.
TEST(DetectionImageTrackerTest, TrackScaleMatch) {
  // Load the test image.
  ScopeTimer rt("test");
  RandomNumbers r;
  DetectionImageMap targets;
  targets.insert(std::make_pair(TARGET_NAME, loadTestImage()));

  // Construct a camera offset matrix.
  auto tmat = HMatrixGen::translation(0.01f, -0.02f, -0.03f);
  auto rmat = HMatrixGen::xDegrees(1.0f) * HMatrixGen::yDegrees(3.0f) * HMatrixGen::zDegrees(5.0f);
  auto motion = cameraMotion(tmat, rmat);
  auto camPos = motion;

  // Take a photo of the image target from the offset camera location.
  auto img = image(targets.at(TARGET_NAME).framePoints(), 1.0f, camPos);

  // Check that the offset of the camera can be correctly recovered.
  DetectionImageTracker t;
  auto res = t.track(img, {}, targets, &r);
  EXPECT_TRUE(MatrixWithin(res.pose, camPos, 1e-6f));

  // Keep moving the camera along the same offset, and check that its overall position can be
  // recovered.
  camPos = updateWorldPosition(camPos, motion);
  img = image(targets.at(TARGET_NAME).framePoints(), 1.0f, camPos);
  res = t.track(img, {}, targets, &r);
  EXPECT_TRUE(MatrixWithin(res.pose, camPos, 2e-3f));

  // Keep moving the camera along the same offset, and check that its overall position can be
  // recovered.
  camPos = updateWorldPosition(camPos, motion);
  img = image(targets.at(TARGET_NAME).framePoints(), 1.0f, camPos);
  res = t.track(img, {}, targets, &r);
  EXPECT_TRUE(MatrixWithin(res.pose, camPos, 2e-3f));

  // Keep moving the camera along the same offset, and check that its overall position can be
  // recovered.
  camPos = updateWorldPosition(camPos, motion);
  img = image(targets.at(TARGET_NAME).framePoints(), 1.0f, camPos);
  res = t.track(img, {}, targets, &r);
  EXPECT_TRUE(MatrixWithin(res.pose, camPos, 2e-3f));
}

// Tests that an image taken from a different location as the original camera that took the image
// image target photo, and a different scale, recovers the relative position of the camera.
TEST(DetectionImageTrackerTest, TrackKnownScale) {
  // Load the test image.
  ScopeTimer rt("test");
  RandomNumbers r;
  DetectionImageMap targets;
  targets.insert(std::make_pair(TARGET_NAME, loadTestImage()));

  // Construct a camera offset matrix.
  auto tmat = HMatrixGen::translation(0.01f * 0.2f, -0.02f * 0.2f, -0.03f * 0.2f);
  auto rmat = HMatrixGen::xDegrees(1.0f) * HMatrixGen::yDegrees(3.0f) * HMatrixGen::zDegrees(5.0f);
  auto motion = cameraMotion(tmat, rmat);
  auto camPos = motion;

  // Take a photo of the image target (at 1/5 scale) from the offset camera location.
  auto img = image(targets.at(TARGET_NAME).framePoints(), 0.2f, camPos);

  // Check that the offset of the camera can be correctly recovered.
  DetectionImageTracker t;
  auto res = t.track(img, {}, targets, &r);
  EXPECT_TRUE(MatrixWithin(scaleTranslation(0.2f, res.pose), camPos, 1e-6f));

  // Keep moving the camera along the same offset, and check that its overall position can be
  // recovered.
  camPos = updateWorldPosition(camPos, motion);
  img = image(targets.at(TARGET_NAME).framePoints(), 0.2f, camPos);
  res = t.track(img, {}, targets, &r);
  EXPECT_TRUE(MatrixWithin(scaleTranslation(0.2f, res.pose), camPos, 1e-3f));
}

// Tests that the location of an image target can be recovered when it is located at the origin.
TEST(DetectionImageTrackerTest, LocateTargetIdentity) {
  // Load the test image.
  ScopeTimer rt("test");
  RandomNumbers r;
  DetectionImageMap targets;
  targets.insert(std::make_pair(TARGET_NAME, loadTestImage()));

  // Set the image target location and scale.
  float scale = 1.0f;
  auto imageLoc = HMatrixGen::i();

  // Construct the camera offset that will match the virtual camera that took the original photo.
  auto camPos = HMatrixGen::translation(0.0f, 0.0f, -scale);

  // Take a photo of the image target.
  auto img = image(targets.at(TARGET_NAME).framePoints(), imageLoc, scale, camPos);

  // Recover the location of the image target, and check that is the same as the original.
  DetectionImageTracker t;
  auto res = t.track(img, {}, targets, &r);
  auto located = DetectionImageTracker::locate(res, targets, camPos, scale);
  EXPECT_TRUE(MatrixWithin(located.pose, imageLoc, 1e-6f));
}

// Tests that the location of an image target can be recovered when it is located in space (not at
// the origin), but the camera taking the photo is positioned to fill the frame exactly.
TEST(DetectionImageTrackerTest, LocateTargetFullCamera) {
  // Load the test image.
  ScopeTimer rt("test");
  RandomNumbers r;
  DetectionImageMap targets;
  targets.insert(std::make_pair(TARGET_NAME, loadTestImage()));

  // Set the image target location and scale.
  float scale = 1.0f;
  auto tmat = HMatrixGen::translation(0.01f, -0.02f, -0.03f);
  auto rmat = HMatrixGen::xDegrees(1.0f) * HMatrixGen::yDegrees(3.0f) * HMatrixGen::zDegrees(5.0f);
  auto imageLoc = cameraMotion(tmat, rmat);

  // Construct the camera offset that will match the virtual camera that took the original photo.
  auto camPos = updateWorldPosition(imageLoc, HMatrixGen::translation(0.0f, 0.0f, -scale));

  // Take a photo of the image target.
  auto img = image(targets.at(TARGET_NAME).framePoints(), imageLoc, scale, camPos);

  // Recover the location of the image target, and check that is the same as the original.
  DetectionImageTracker t;
  auto res = t.track(img, {}, targets, &r);
  auto located = DetectionImageTracker::locate(res, targets, camPos, scale);
  EXPECT_TRUE(MatrixWithin(located.pose, imageLoc, 3e-6f));
}

// Tests that the location of an image target can be recovered when it is located in space (not at
// the origin), and the camera taking the photo is further offset.
TEST(DetectionImageTrackerTest, LocateTargetPartialCamera) {
  // Load the test image.
  ScopeTimer rt("test");
  RandomNumbers r;
  DetectionImageMap targets;
  targets.insert(std::make_pair(TARGET_NAME, loadTestImage()));

  // Set the image target location and scale.
  float scale = 1.0f;
  auto tmat = HMatrixGen::translation(0.01f, -0.02f, -0.03f);
  auto rmat = HMatrixGen::xDegrees(1.0f) * HMatrixGen::yDegrees(3.0f) * HMatrixGen::zDegrees(5.0f);
  auto imageLoc = cameraMotion(tmat, rmat);

  // Construct the camera to be offset from the image target.
  auto camPos = updateWorldPosition(imageLoc, HMatrixGen::translation(0.0f, 0.0f, -scale));
  camPos = updateWorldPosition(camPos, imageLoc);

  // Take a photo of the image target.
  auto img = image(targets.at(TARGET_NAME).framePoints(), imageLoc, scale, camPos);

  // Recover the location of the image target, and check that is the same as the original.
  DetectionImageTracker t;
  auto res = t.track(img, {}, targets, &r);
  auto located = DetectionImageTracker::locate(res, targets, camPos, scale);
  EXPECT_TRUE(MatrixWithin(located.pose, imageLoc, 1e-5f));
}

// Tests that the location of an image target can be recovered when it is located in space (not at
// the origin), and when it is a different size than the original (but at a known scale factor), and
// the camera taking the photo is further offset.
TEST(DetectionImageTrackerTest, LocateTargetPartialCameraKnownScale) {
  // Load the test image.
  ScopeTimer rt("test");
  RandomNumbers r;
  DetectionImageMap targets;
  targets.insert(std::make_pair(TARGET_NAME, loadTestImage()));

  // Set the image target location and scale.
  float scale = 0.2f;
  auto tmat = HMatrixGen::translation(0.01f, -0.02f, -0.03f);
  auto rmat = HMatrixGen::xDegrees(1.0f) * HMatrixGen::yDegrees(3.0f) * HMatrixGen::zDegrees(5.0f);
  auto imageLoc = cameraMotion(tmat, rmat);

  // Construct the camera to be offset from the image target.
  auto camPos = updateWorldPosition(imageLoc, HMatrixGen::translation(0.0f, 0.0f, -scale));
  camPos = updateWorldPosition(camPos, scaleTranslation(scale, imageLoc));

  // Take a photo of the image target.
  auto img = image(targets.at(TARGET_NAME).framePoints(), imageLoc, scale, camPos);

  // Recover the location of the image target, and check that is the same as the original.
  DetectionImageTracker t;
  auto res = t.track(img, {}, targets, &r);
  auto located = DetectionImageTracker::locate(res, targets, camPos, scale);
  EXPECT_TRUE(MatrixWithin(located.pose, imageLoc, 5e-6f));

  // Move the camera again and construct a new image.
  camPos = updateWorldPosition(camPos, scaleTranslation(scale, imageLoc));
  img = image(targets.at(TARGET_NAME).framePoints(), imageLoc, scale, camPos);

  // Recover the unchanged location of the image target, and check that is still the same as the
  // original.
  res = t.track(img, {}, targets, &r);
  located = DetectionImageTracker::locate(res, targets, camPos, scale);
  EXPECT_TRUE(MatrixWithin(located.pose, imageLoc, 1e-3f));
}

// Tests that image target scale can be recovered from two detections at a baseline.
TEST(DetectionImageTrackerTest, RecoverScale) {
  // Load the test image.
  ScopeTimer rt("test");
  RandomNumbers r;
  DetectionImageMap targets;
  targets.insert(std::make_pair(TARGET_NAME, loadTestImage()));

  // Set the image target location and scale.
  float scale = 0.2f;
  auto tmat = HMatrixGen::translation(0.05f, -0.10f, -0.15f);
  auto rmat = HMatrixGen::xDegrees(1.0f) * HMatrixGen::yDegrees(3.0f) * HMatrixGen::zDegrees(5.0f);
  auto imageLoc = cameraMotion(tmat, rmat);

  // Construct the first camera to be offset from the image target.
  auto camPosA = updateWorldPosition(imageLoc, HMatrixGen::translation(0.0f, 0.0f, -scale));
  camPosA = updateWorldPosition(camPosA, scaleTranslation(scale, imageLoc));

  // Take a photo of the image target from the first camera.
  auto img = image(targets.at(TARGET_NAME).framePoints(), imageLoc, scale, camPosA);

  // Locate the first camera in the reference frame of the image target's virtual camera.
  DetectionImageTracker t;
  auto resA = t.track(img, {}, targets, &r);

  // Construct the second camera to be offset from the first camera.
  auto camPosB = updateWorldPosition(camPosA, scaleTranslation(scale, imageLoc));

  // Take a photo of the image target from the second camera.
  img = image(targets.at(TARGET_NAME).framePoints(), imageLoc, scale, camPosB);

  // Locate the second camera in the reference frame of the image target's virtual camera.
  auto resB = t.track(img, {}, targets, &r);

  // Recover the image target scale from the two cameras pairs in the two reference frames.
  auto estimatedScale = ImageTrackingState::computeScale(camPosA, camPosB, resA.pose, resB.pose);
  EXPECT_NEAR(estimatedScale, scale, 6e-3);

  // Check that image target can be correctly located from the first image using the recovered
  // scale.
  auto locatedA = DetectionImageTracker::locate(resA, targets, camPosA, estimatedScale);
  EXPECT_TRUE(MatrixWithin(locatedA.pose, imageLoc, 6e-3f));

  // Check that image target can be correctly located from the second image using the recovered
  // scale.
  auto locatedB = DetectionImageTracker::locate(resA, targets, camPosA, estimatedScale);
  EXPECT_TRUE(MatrixWithin(locatedB.pose, imageLoc, 6e-3f));

  // Check that the image target is located in the same place from both images.
  auto locatedDist = (translation(locatedA.pose) - translation(locatedB.pose)).l2Norm();
  EXPECT_EQ(locatedDist, 0.0f);
}

}  // namespace
}  // namespace c8

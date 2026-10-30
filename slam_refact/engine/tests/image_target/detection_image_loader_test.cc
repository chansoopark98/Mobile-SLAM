// Port of PUB:reality/engine/imagedetection/detection-image-loader-test.cc (US-011, US-012).
//
// Differences from the public test (assertions and tolerances unchanged):
// - The GL loader (OffscreenGlContext, Gr8FeatureShader, GlTexture upload) is replaced by
//   image_target::TargetLoader on the CPU pyramid port (model kIeee, the engine default).
// - The JPEG inputs are git-lfs pointers locally; SyntheticTarget() of the same size replaces them:
//   flower-big.jpg 1440x1920 (FLOWER_IMG_WIDTH/HEIGHT of the test), amity_up.jpg and
//   amity_rotated.jpg 480x640 (the 3:4 luminance size; their real size is not recoverable from the
//   pointer files). The geometry assertions depend only on the metadata and the level-0 size.
// - MutableRootMessage<ImageTargetMetadata> -> ImageTargetSpec (same fields).
// - equalsMatrix (gmock Pointwise, eps 1e-4) -> MatrixWithin(.., 1e-4).

#include <gtest/gtest.h>

#include <cmath>

#include "image_target/image_target_spec.h"
#include "image_target/target_loader.h"
#include "test_util.h"

namespace slam_refact::image_target {
namespace {

using testing::MatrixWithin;
using testing::SyntheticTarget;
using testing::TestPinholeK;

constexpr float kMatrixEps = 0.0001f;

c8::DetectionImage LoadTestImage(const GrayImage& img, const ImageTargetSpec& spec) {
  return LoadDetectionImage(spec, img.view(), TestPinholeK(), ShaderNumericModel::kIeee);
}

const int FLOWER_IMG_WIDTH = 1440;
const int FLOWER_IMG_HEIGHT = 1920;
const float CURVY_SIDE_LENGTH = 9.3f;
const float TARGET_CIRCUMFERENCE = CURVY_SIDE_LENGTH * FLOWER_IMG_WIDTH / FLOWER_IMG_HEIGHT;
const float CURVY_CIRCUMFERENCE = 21.0f;

const GrayImage& FlowerImage() {
  static const GrayImage img = SyntheticTarget(FLOWER_IMG_WIDTH, FLOWER_IMG_HEIGHT, 7);
  return img;
}

const GrayImage& AmityImage() {
  static const GrayImage img = SyntheticTarget(480, 640, 11);
  return img;
}

c8::DetectionImage loadPlanarTestImage() {
  ImageTargetSpec spec;
  spec.type = ImageTargetType::kPlanar;
  return LoadTestImage(FlowerImage(), spec);
}

c8::DetectionImage loadCroppedCurvyTestImage(int x, int y, int w, int h, bool isRotated = false) {
  ImageTargetSpec spec;
  spec.type = ImageTargetType::kCurvy;
  spec.curvy_circumference_top = CURVY_CIRCUMFERENCE;
  spec.curvy_circumference_bottom = CURVY_CIRCUMFERENCE;
  spec.curvy_side_length = CURVY_SIDE_LENGTH;
  spec.target_circumference_top = TARGET_CIRCUMFERENCE;
  spec.original_image_width = isRotated ? FLOWER_IMG_HEIGHT : FLOWER_IMG_WIDTH;
  spec.original_image_height = isRotated ? FLOWER_IMG_WIDTH : FLOWER_IMG_HEIGHT;
  spec.crop_original_image_x = x;
  spec.crop_original_image_y = y;
  spec.crop_original_image_width = w;
  spec.crop_original_image_height = h;
  spec.is_rotated = isRotated;
  return LoadTestImage(FlowerImage(), spec);
}

c8::DetectionImage loadFullCurvyGeometry(int w, int h) {
  ImageTargetSpec spec;
  spec.type = ImageTargetType::kCurvy;
  spec.curvy_circumference_top = CURVY_CIRCUMFERENCE;
  spec.curvy_circumference_bottom = CURVY_CIRCUMFERENCE;
  const float sideLength = CURVY_SIDE_LENGTH / FLOWER_IMG_HEIGHT * h;
  spec.curvy_side_length = sideLength;
  spec.target_circumference_top = sideLength * w / h;
  spec.original_image_width = w;
  spec.original_image_height = h;
  spec.crop_original_image_x = 0;
  spec.crop_original_image_y = 0;
  spec.crop_original_image_width = w;
  spec.crop_original_image_height = h;
  return LoadTestImage(FlowerImage(), spec);
}

// Return the full image target with no crop
c8::DetectionImage loadCurvyTestImage() {
  return loadCroppedCurvyTestImage(0, 0, FLOWER_IMG_WIDTH, FLOWER_IMG_HEIGHT);
}

TEST(DetectionImageLoaderTest, TrackLoadPlanarImage) {
  auto detectionImage = loadPlanarTestImage();
  EXPECT_GT(detectionImage.framePoints().points().size(), 1000u);
}

TEST(DetectionImageLoaderTest, TrackLoadPlanarImageAgain) {
  auto detectionImage = loadPlanarTestImage();
  EXPECT_GT(detectionImage.framePoints().points().size(), 1000u);
}

TEST(DetectionImageLoaderTest, TrackLoadCurvyImage) {
  auto detectionImage = loadCurvyTestImage();
  EXPECT_GT(detectionImage.framePoints().points().size(), 1000u);

  auto curvyGeometry = detectionImage.getGeometry();

  EXPECT_FLOAT_EQ(curvyGeometry.height, 1.f);
  EXPECT_NEAR(curvyGeometry.radius, 0.359382f, 1e-6);
  EXPECT_FLOAT_EQ(curvyGeometry.activationRegion.top, 0.f);
  EXPECT_FLOAT_EQ(curvyGeometry.activationRegion.bottom, 1.f);
  EXPECT_NEAR(curvyGeometry.activationRegion.left, 0.333928f, 1e-6);
  EXPECT_NEAR(curvyGeometry.activationRegion.right, 0.666071, 1e-6);

  EXPECT_EQ(curvyGeometry.srcCols, 480) << "geometry of source is pyramid in portrait";
  EXPECT_EQ(curvyGeometry.srcRows, 640) << "geometry of source is pyramid in portrait";
}

TEST(DetectionImageLoaderTest, TrackLoadCurvyImageCropped) {
  // Got these numbers by opening up GIMP, then select a 3x4 selection
  auto detectionImage = loadCroppedCurvyTestImage(230, 62, 1065, 1420);
  EXPECT_GT(detectionImage.framePoints().points().size(), 1000u);

  auto curvyGeometry = detectionImage.getGeometry();

  EXPECT_NEAR(curvyGeometry.height, 1.f, 1e-6) << "internal curvy geometry in the engine is 1.0";
  // Since the cropped curvy has to have height of 1.0, its radius grows as a ratio of
  // originalHeight to croppedHeight
  EXPECT_NEAR(curvyGeometry.radius, 0.485925f, 1e-6);
  EXPECT_FLOAT_EQ(curvyGeometry.activationRegion.top, 0.032291669f);
  EXPECT_FLOAT_EQ(curvyGeometry.activationRegion.bottom, 0.77187502f);
  EXPECT_FLOAT_EQ(curvyGeometry.activationRegion.left, 0.38697919f);
  EXPECT_FLOAT_EQ(curvyGeometry.activationRegion.right, 0.63262653f);

  EXPECT_EQ(curvyGeometry.srcCols, 480) << "geometry of source is pyramid in portrait";
  EXPECT_EQ(curvyGeometry.srcRows, 640) << "geometry of source is pyramid in portrait";
}

TEST(DetectionImageLoaderTest, TrackLoadCurvyImageCroppedSameAsFull) {
  // Loading an image cropped to a specific dimension should result in the same geometry
  // as loading the cropped image of that dimension if the cropped is centered in X
  int croppedHeight = 1200;
  auto detectionImage = loadCroppedCurvyTestImage(270, 62, 900, croppedHeight);
  EXPECT_GT(detectionImage.framePoints().points().size(), 1000u);
  auto croppedGeometry = detectionImage.getGeometry();
  auto fullGeometry = loadFullCurvyGeometry(900, croppedHeight).getGeometry();

  EXPECT_NEAR(croppedGeometry.height, fullGeometry.height, 1e-6)
    << "internal curvy geometry in the engine is 1.0";
  EXPECT_NEAR(croppedGeometry.radius, fullGeometry.radius, 1e-6);
  EXPECT_FLOAT_EQ(croppedGeometry.activationRegion.top, 0.032291669f);
  EXPECT_FLOAT_EQ(croppedGeometry.activationRegion.bottom, 0.65729171f);
  EXPECT_FLOAT_EQ(croppedGeometry.activationRegion.left, fullGeometry.activationRegion.left);
  EXPECT_FLOAT_EQ(croppedGeometry.activationRegion.right, fullGeometry.activationRegion.right);

  EXPECT_EQ(croppedGeometry.srcCols, 480) << "geometry of source is pyramid in portrait";
  EXPECT_EQ(croppedGeometry.srcRows, 640) << "geometry of source is pyramid in portrait";
}

TEST(DetectionImageLoaderTest, CenterCropIsNoTransformation) {
  int croppedWidth = 900;
  int croppedHeight = 1200;
  int shiftX = (FLOWER_IMG_WIDTH - croppedWidth) / 2;
  int shiftY = (FLOWER_IMG_HEIGHT - croppedHeight) / 2;
  auto detectionImage = loadCroppedCurvyTestImage(shiftX, shiftY, croppedWidth, croppedHeight);
  EXPECT_GT(detectionImage.framePoints().points().size(), 1000u);

  // Since the crop is at the center of the image target, there is no transformation between
  // the coordinate of the cropped curvy and the full curvy
  EXPECT_TRUE(MatrixWithin(
    detectionImage.getFullLabelToCroppedLabelPose(), c8::HMatrixGen::i(), kMatrixEps));
}

TEST(DetectionImageLoaderTest, CenterCropIsNoTransformationRotated) {
  int croppedWidth = 480;
  int croppedHeight = 640;
  int shiftX = (FLOWER_IMG_HEIGHT - croppedWidth) / 2;
  int shiftY = (FLOWER_IMG_WIDTH - croppedHeight) / 2;
  auto detectionImage =
    loadCroppedCurvyTestImage(shiftX, shiftY, croppedWidth, croppedHeight, true);
  EXPECT_GT(detectionImage.framePoints().points().size(), 1000u);

  // Since the crop is at the center of the image target, there is no transformation between
  // the coordinate of the cropped curvy and the full curvy
  EXPECT_TRUE(MatrixWithin(
    detectionImage.getFullLabelToCroppedLabelPose(), c8::HMatrixGen::i(), kMatrixEps));
}

TEST(DetectionImageLoaderTest, MaxRotationNoShift) {
  // we are doing a left crop of this image
  int croppedWidth = FLOWER_IMG_WIDTH / 2;
  int croppedHeight = FLOWER_IMG_HEIGHT / 2;
  int shiftX = 0;
  int shiftY = (FLOWER_IMG_HEIGHT - croppedHeight) / 2;
  auto detectionImage = loadCroppedCurvyTestImage(shiftX, shiftY, croppedWidth, croppedHeight);
  EXPECT_GT(detectionImage.framePoints().points().size(), 1000u);

  // if the label has been fully wrapped around the curvy, the left of the label
  // would be exactly -90 degree. Instead, it is linearly scaled by the target label size
  c8::HMatrix rotation =
    c8::HMatrixGen::rotationD(0, -90 * TARGET_CIRCUMFERENCE / CURVY_CIRCUMFERENCE, 0);
  EXPECT_TRUE(
    MatrixWithin(detectionImage.getFullLabelToCroppedLabelPose(), rotation, kMatrixEps));
}

TEST(DetectionImageLoaderTest, MaxRotationNoShiftRotated) {
  // we are doing a top crop of this image. When rotated back counter clock-wise, it is like a left
  // crop of the image.
  int croppedWidth = FLOWER_IMG_HEIGHT / 2;
  int croppedHeight = FLOWER_IMG_WIDTH / 2;
  int shiftX = (FLOWER_IMG_HEIGHT - croppedWidth) / 2;
  int shiftY = 0;
  auto detectionImage =
    loadCroppedCurvyTestImage(shiftX, shiftY, croppedWidth, croppedHeight, true);
  EXPECT_GT(detectionImage.framePoints().points().size(), 1000u);

  // if the label has been fully wrapped around the curvy, the left of the label
  // would be exactly -90 degree. Instead, it is linearly scaled by the target label size
  c8::HMatrix rotation =
    c8::HMatrixGen::rotationD(0, -90 * TARGET_CIRCUMFERENCE / CURVY_CIRCUMFERENCE, 0);
  EXPECT_TRUE(
    MatrixWithin(detectionImage.getFullLabelToCroppedLabelPose(), rotation, kMatrixEps));
}

TEST(DetectionImageLoaderTest, RotatedMaxRotationIsMaxShift) {
  // we are doing a left crop of this image. But since it is a rotated image, the pose diff is just
  // a translation
  int croppedWidth = FLOWER_IMG_HEIGHT / 2;
  int croppedHeight = FLOWER_IMG_WIDTH / 2;
  int shiftX = 0;
  int shiftY = (FLOWER_IMG_WIDTH - croppedHeight) / 2;
  auto detectionImage =
    loadCroppedCurvyTestImage(shiftX, shiftY, croppedWidth, croppedHeight, true);
  EXPECT_GT(detectionImage.framePoints().points().size(), 1000u);

  // Since we crop out the left, the cropped label is shifted downward by half (when rotated
  // counter-clockwise)
  c8::HMatrix translation = c8::HMatrixGen::translation(0, -0.5, 0);
  EXPECT_TRUE(
    MatrixWithin(detectionImage.getFullLabelToCroppedLabelPose(), translation, kMatrixEps));
}

TEST(DetectionImageLoaderTest, RotationAndShift) {
  // we are doing a bottom right crop of this image
  int croppedWidth = FLOWER_IMG_WIDTH / 2;
  int croppedHeight = FLOWER_IMG_HEIGHT / 2;
  int shiftX = FLOWER_IMG_WIDTH / 2;
  int shiftY = FLOWER_IMG_HEIGHT / 2;
  auto detectionImage = loadCroppedCurvyTestImage(shiftX, shiftY, croppedWidth, croppedHeight);
  EXPECT_GT(detectionImage.framePoints().points().size(), 1000u);

  // see MaxRotationNoShift unit test for explanation
  c8::HMatrix rotation =
    c8::HMatrixGen::rotationD(0, 90 * TARGET_CIRCUMFERENCE / CURVY_CIRCUMFERENCE, 0);
  // Since we crop out the bottom right, the cropped label is shifted downward by half
  c8::HMatrix translation = c8::HMatrixGen::translation(0, -0.5, 0);
  EXPECT_TRUE(MatrixWithin(
    detectionImage.getFullLabelToCroppedLabelPose(), translation * rotation, kMatrixEps));
}

TEST(DetectionImageLoaderTest, RotationAndShiftRotated) {
  // we are doing bottom right crop of the rotated image
  int croppedWidth = FLOWER_IMG_HEIGHT / 2;
  int croppedHeight = FLOWER_IMG_WIDTH / 2;
  int shiftX = FLOWER_IMG_HEIGHT / 2;
  int shiftY = FLOWER_IMG_WIDTH / 2;
  auto detectionImage =
    loadCroppedCurvyTestImage(shiftX, shiftY, croppedWidth, croppedHeight, true);
  EXPECT_GT(detectionImage.framePoints().points().size(), 1000u);

  c8::HMatrix rotation =
    c8::HMatrixGen::rotationD(0, 90 * TARGET_CIRCUMFERENCE / CURVY_CIRCUMFERENCE, 0);
  // Since we crop out the bottom right, the cropped label is shifted upward by half
  c8::HMatrix translation = c8::HMatrixGen::translation(0, 0.5, 0);
  EXPECT_TRUE(MatrixWithin(
    detectionImage.getFullLabelToCroppedLabelPose(), translation * rotation, kMatrixEps));
}

TEST(DetectionImageLoaderTest, DetectionImageIntrinsicIsRotatedForRotatedInput) {
  ImageTargetSpec spec;
  spec.type = ImageTargetType::kPlanar;
  spec.original_image_width = 1092;
  spec.original_image_height = 1278;
  spec.crop_original_image_x = 68;
  spec.crop_original_image_y = 0;
  spec.crop_original_image_width = 959;
  spec.crop_original_image_height = 1278;

  // Unrotated
  spec.is_rotated = false;
  c8::DetectionImage detectionImage = LoadTestImage(AmityImage(), spec);
  c8_PixelPinholeCameraModel intrinsic = detectionImage.framePoints().intrinsic();

  // Rotated
  spec.is_rotated = true;
  c8::DetectionImage detectionImageRotated = LoadTestImage(AmityImage(), spec);
  c8_PixelPinholeCameraModel intrinsicRotated = detectionImageRotated.framePoints().intrinsic();

  EXPECT_EQ(intrinsic.pixelsWidth, intrinsicRotated.pixelsHeight);
  EXPECT_EQ(intrinsic.pixelsHeight, intrinsicRotated.pixelsWidth);
  EXPECT_EQ(intrinsic.centerPointX, intrinsicRotated.centerPointY);
  EXPECT_EQ(intrinsic.centerPointY, intrinsicRotated.centerPointX);
}

TEST(DetectionImageLoaderTest, DetectionImageCurvyGeometryIsCorrectForRotatedInput) {
  ImageTargetSpec spec;
  spec.type = ImageTargetType::kCurvy;
  int ORIGINAL_WIDTH = 1278;
  int ORIGINAL_HEIGHT = 1092;
  int CROP_WIDTH = 819;
  int CROP_HEIGHT = ORIGINAL_HEIGHT;
  spec.original_image_width = ORIGINAL_WIDTH;
  spec.original_image_height = ORIGINAL_HEIGHT;
  spec.crop_original_image_x = 236;
  spec.crop_original_image_y = 0;
  spec.crop_original_image_width = CROP_WIDTH;
  spec.crop_original_image_height = CROP_HEIGHT;

  float SIDE_LENGTH = 12.9f;
  float TARGET_CIRCUMFERENCE_ROTATED = SIDE_LENGTH * ORIGINAL_HEIGHT / ORIGINAL_WIDTH;
  float CIRCUMFERENCE = 27.3f;
  spec.curvy_circumference_top = CIRCUMFERENCE;
  spec.curvy_circumference_bottom = CIRCUMFERENCE;
  spec.curvy_side_length = SIDE_LENGTH;
  // the target circumference is measured, and thus not in width / height aspect ratio
  // but on height / width aspect ratio
  spec.target_circumference_top = TARGET_CIRCUMFERENCE_ROTATED;
  spec.is_rotated = true;

  c8::DetectionImage detectionImageRotated = LoadTestImage(AmityImage(), spec);
  c8::CurvyImageGeometry curvyGeometry = detectionImageRotated.getGeometry();

  // Have to swap width and height because the image is rotated
  float originalWidth = static_cast<float>(spec.original_image_height);
  float originalHeight = static_cast<float>(spec.original_image_width);
  float cropWidth = static_cast<float>(spec.crop_original_image_height);
  float cropHeight = static_cast<float>(spec.crop_original_image_width);

  // The radius in the engine is the curvy's radius divided by sideLength.
  // A cropped image target corresponds to a curvy with the engine full radius scaled by the
  // cropped width and height.
  // croppedRadius = engineFullRadius * (cropHeight/originalHeight) / (cropWidth/originalWidth)
  // 0.525581 = ((27.3 / 2 / pi) / 12.9) * (cropHeight/originalHeight) / (cropWidth/originalWidth)
  float fullRadius = spec.curvy_circumference_top / (2 * M_PI);
  float engineFullRadius = fullRadius / SIDE_LENGTH;
  float fullRadiusToCroppedRadiusRatio =
    (cropHeight / originalHeight) / (cropWidth / originalWidth);
  float croppedRadius = engineFullRadius / fullRadiusToCroppedRadiusRatio;
  EXPECT_NEAR(curvyGeometry.radius, croppedRadius, 1e-6);
  EXPECT_NEAR(curvyGeometry.radius, 0.525581, 1e-6);

  EXPECT_FLOAT_EQ(curvyGeometry.height, 1.f);
  EXPECT_FLOAT_EQ(curvyGeometry.activationRegion.top, 0.17449139f);
  EXPECT_FLOAT_EQ(curvyGeometry.activationRegion.bottom, 0.81533647f);
  float left = 0.5f - TARGET_CIRCUMFERENCE_ROTATED / CIRCUMFERENCE / 2;
  float right = 0.5f + TARGET_CIRCUMFERENCE_ROTATED / CIRCUMFERENCE / 2;
  EXPECT_NEAR(curvyGeometry.activationRegion.left, left, 1e-6);
  EXPECT_NEAR(curvyGeometry.activationRegion.right, right, 1e-6);

  EXPECT_EQ(curvyGeometry.srcCols, 640) << "geometry source is set to pyramid level in landscape";
  EXPECT_EQ(curvyGeometry.srcRows, 480) << "geometry source is set to pyramid level in landscape";
}

}  // namespace
}  // namespace slam_refact::image_target

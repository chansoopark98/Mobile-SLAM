// US-012: CPU target loader on synthetic luminance targets (height 640, the CLI luminance size).

#include <gtest/gtest.h>

#include <cstring>
#include <fstream>
#include <string>

#include "frontend/frontend.h"
#include "image_target/image_target_spec.h"
#include "image_target/target_files.h"
#include "image_target/target_json.h"
#include "image_target/target_loader.h"
#include "reality/engine/features/image-descriptor.h"
#include "test_util.h"

namespace slam_refact::image_target {
namespace {

using testing::SyntheticTarget;
using testing::TestPinholeK;

ImageTargetSpec PlanarSpec() {
  ImageTargetSpec spec;
  spec.name = "synthetic";
  spec.type = ImageTargetType::kPlanar;
  spec.original_image_width = 480;
  spec.original_image_height = 640;
  spec.crop_original_image_width = 480;
  spec.crop_original_image_height = 640;
  return spec;
}

// Every point lies inside the pixel frame of the target intrinsics.
void ExpectPointsInsideIntrinsics(const c8::DetectionImage& im) {
  const c8::TargetWithPoints& pts = im.framePoints();
  const c8_PixelPinholeCameraModel k = pts.intrinsic();
  for (const c8::HPoint2& p : pts.pixels()) {
    EXPECT_GE(p.x(), -1e-3f);
    EXPECT_GE(p.y(), -1e-3f);
    EXPECT_LT(p.x(), k.pixelsWidth + 1e-3f);
    EXPECT_LT(p.y(), k.pixelsHeight + 1e-3f);
  }
}

TEST(TargetLoaderTest, PortraitLuminanceGivesOrbAndGorbDescriptors) {
  const GrayImage img = SyntheticTarget(480, 640, 3);
  const c8::DetectionImage im =
      LoadDetectionImage(PlanarSpec(), img.view(), TestPinholeK(), ShaderNumericModel::kIeee);
  const c8::TargetWithPoints& pts = im.framePoints();
  EXPECT_GT(pts.points().size(), 1000u);
  EXPECT_LE(pts.points().size(), 2500u);  // nKeypoints of extractFeatures
  // allOrb + allGorb: every keypoint carries both descriptors.
  EXPECT_EQ(pts.store().numDescriptors<c8::OrbFeature>(), pts.points().size());
  EXPECT_EQ(pts.store().numDescriptors<c8::GorbFeature>(), pts.points().size());
  EXPECT_EQ(pts.featureType(), c8::DescriptorType::ORB);
  EXPECT_EQ(im.getType(), c8::DetectionImageType::PLANAR);
  EXPECT_EQ(im.rotation(), 0);
  EXPECT_EQ(im.getName(), "synthetic");
  // 480x640 luminance: level 0 is 480x640 and K is the device K unchanged.
  const c8_PixelPinholeCameraModel k = pts.intrinsic();
  const PinholeIntrinsics dk = TestPinholeK();
  EXPECT_EQ(k.pixelsWidth, 480);
  EXPECT_EQ(k.pixelsHeight, 640);
  EXPECT_FLOAT_EQ(k.centerPointX, dk.cx);
  EXPECT_FLOAT_EQ(k.centerPointY, dk.cy);
  EXPECT_FLOAT_EQ(k.focalLengthHorizontal, dk.fx);
  ExpectPointsInsideIntrinsics(im);
}

TEST(TargetLoaderTest, LoadIsDeterministic) {
  const GrayImage img = SyntheticTarget(480, 640, 5);
  const c8::DetectionImage a =
      LoadDetectionImage(PlanarSpec(), img.view(), TestPinholeK(), ShaderNumericModel::kIeee);
  const c8::DetectionImage b =
      LoadDetectionImage(PlanarSpec(), img.view(), TestPinholeK(), ShaderNumericModel::kIeee);
  ASSERT_EQ(a.framePoints().points().size(), b.framePoints().points().size());
  for (size_t i = 0; i < a.framePoints().points().size(); ++i) {
    const c8::TargetPoint& pa = a.framePoints().points()[i];
    const c8::TargetPoint& pb = b.framePoints().points()[i];
    EXPECT_EQ(pa.x(), pb.x());
    EXPECT_EQ(pa.y(), pb.y());
    EXPECT_EQ(pa.scale(), pb.scale());
    EXPECT_EQ(pa.angle(), pb.angle());
  }
  const auto& da = a.framePoints().store().getFeatures<c8::OrbFeature>();
  const auto& db = b.framePoints().store().getFeatures<c8::OrbFeature>();
  ASSERT_EQ(da.size(), db.size());
  for (size_t i = 0; i < da.size(); ++i) {
    EXPECT_EQ(0, std::memcmp(da[i].data(), db[i].data(), c8::OrbFeature::size()));
  }
}

TEST(TargetLoaderTest, BothShaderModelsLoad) {
  const GrayImage img = SyntheticTarget(480, 640, 9);
  const c8::DetectionImage im = LoadDetectionImage(PlanarSpec(), img.view(), TestPinholeK(),
                                                   ShaderNumericModel::kSwiftShaderApprox);
  EXPECT_GT(im.framePoints().points().size(), 1000u);
}

// A landscape luminance (width > height) is drawn rotated by 90 degrees into a portrait pyramid
// (detection-image-loader.cc:82-88); ORB targets keep the pyramid orientation.
TEST(TargetLoaderTest, LandscapeLuminanceUsesRotation90) {
  const GrayImage img = SyntheticTarget(853, 640, 13);
  ImageTargetSpec spec = PlanarSpec();
  TargetLoader loader(spec, img.width, img.height, TestPinholeK());
  EXPECT_EQ(loader.rotation(), 90);
  EXPECT_FALSE(loader.rotate_features());
  EXPECT_TRUE(loader.layout().rotate_input);
  const frontend::LevelLayout& l0 = loader.layout().levels[0];
  EXPECT_LT(l0.w, l0.h);
  const c8::DetectionImage im = loader.ExtractFeatures(img.view(), ShaderNumericModel::kIeee);
  EXPECT_EQ(im.rotation(), 90);
  EXPECT_EQ(im.framePoints().intrinsic().pixelsWidth, l0.w);
  EXPECT_EQ(im.framePoints().intrinsic().pixelsHeight, l0.h);
  EXPECT_GT(im.framePoints().points().size(), 500u);
  ExpectPointsInsideIntrinsics(im);
}

// detection-image-loader.cc:60-80.
TEST(TargetLoaderTest, GorbOnlyForStaticPlanarTargetsAwayFromTheXzPlane) {
  struct Case {
    float roll;
    float pitch;
    bool is_static;
    ImageTargetType type;
    c8::DescriptorType expected;
  };
  const Case cases[] = {
      {0.0f, 0.0f, true, ImageTargetType::kPlanar, c8::DescriptorType::GORB},
      {180.0f, 0.0f, true, ImageTargetType::kPlanar, c8::DescriptorType::GORB},
      {90.0f, 0.0f, true, ImageTargetType::kPlanar, c8::DescriptorType::ORB},
      {91.5f, 0.0f, true, ImageTargetType::kPlanar, c8::DescriptorType::ORB},
      {92.5f, 0.0f, true, ImageTargetType::kPlanar, c8::DescriptorType::GORB},
      {268.5f, 0.0f, true, ImageTargetType::kPlanar, c8::DescriptorType::ORB},
      {0.0f, 1.0f, true, ImageTargetType::kPlanar, c8::DescriptorType::ORB},
      {0.0f, 0.0f, false, ImageTargetType::kPlanar, c8::DescriptorType::ORB},
      {0.0f, 0.0f, true, ImageTargetType::kUnspecified, c8::DescriptorType::ORB},
  };
  for (const Case& c : cases) {
    ImageTargetSpec spec = PlanarSpec();
    spec.type = c.type;
    spec.is_static_target = c.is_static;
    spec.roll_angle = c.roll;
    spec.pitch_angle = c.pitch;
    const TargetLoader loader(spec, 480, 640, TestPinholeK());
    EXPECT_EQ(loader.feature_type(), c.expected) << "roll " << c.roll << " pitch " << c.pitch;
    EXPECT_FALSE(loader.rotate_features());
  }
  // GORB on a landscape image also rotates the features back (rotateFeatures_ |= w > h).
  ImageTargetSpec spec = PlanarSpec();
  spec.is_static_target = true;
  const TargetLoader landscape(spec, 853, 640, TestPinholeK());
  EXPECT_EQ(landscape.feature_type(), c8::DescriptorType::GORB);
  EXPECT_TRUE(landscape.rotate_features());
}

// isRotated (xrhome / CLI landscape crop): features and intrinsics are rotated back to 4:3.
TEST(TargetLoaderTest, RotatedTargetRotatesFeaturesAndIntrinsics) {
  const GrayImage img = SyntheticTarget(480, 640, 17);
  ImageTargetSpec spec = PlanarSpec();
  spec.is_rotated = true;
  const TargetLoader loader(spec, img.width, img.height, TestPinholeK());
  EXPECT_EQ(loader.rotation(), 0);
  EXPECT_TRUE(loader.rotate_features());
  EXPECT_EQ(loader.intrinsics().width, 640);
  EXPECT_EQ(loader.intrinsics().height, 480);
  const c8::DetectionImage im = loader.ExtractFeatures(img.view(), ShaderNumericModel::kIeee);
  EXPECT_TRUE(im.getPlanarGeometry().isRotated);
  EXPECT_GT(im.framePoints().points().size(), 1000u);
  ExpectPointsInsideIntrinsics(im);
}

TEST(TargetLoaderTest, RotateCropAndScaleIntrinsics) {
  const PinholeIntrinsics k{480, 640, 239.5f, 319.5f, 500.0f, 501.0f};
  const PinholeIntrinsics same = RotateCropAndScaleIntrinsics(k, 480, 640);
  EXPECT_EQ(same.width, 480);
  EXPECT_FLOAT_EQ(same.cx, 239.5f);
  EXPECT_FLOAT_EQ(same.fy, 501.0f);
  // Landscape request: swap, then crop and scale (intrinsics.cc:168-184).
  const PinholeIntrinsics rot = RotateCropAndScaleIntrinsics(k, 640, 480);
  EXPECT_EQ(rot.width, 640);
  EXPECT_EQ(rot.height, 480);
  EXPECT_FLOAT_EQ(rot.cx, 319.5f);
  EXPECT_FLOAT_EQ(rot.cy, 239.5f);
  EXPECT_FLOAT_EQ(rot.fx, 501.0f);
  EXPECT_FLOAT_EQ(rot.fy, 500.0f);
  const PinholeIntrinsics half = RotateCropAndScaleIntrinsics(k, 240, 320);
  EXPECT_FLOAT_EQ(half.fx, 250.0f);
  EXPECT_FLOAT_EQ(half.cx, 119.5f);
}

TEST(TargetLoaderTest, CurvySpecFromMetadata) {
  ImageTargetSpec spec;
  spec.type = ImageTargetType::kCurvy;
  spec.original_image_width = 1000;
  spec.original_image_height = 1500;
  spec.crop_original_image_x = 0;
  spec.crop_original_image_y = 82;
  spec.crop_original_image_width = 1000;
  spec.crop_original_image_height = 1333;
  spec.curvy_circumference_top = 100.0f;
  spec.curvy_circumference_bottom = 40.0f;
  spec.target_circumference_top = 25.0f;
  const c8::CurvySpec s = TargetLoader::BuildCurvySpec(spec);
  EXPECT_FLOAT_EQ(s.arc, 0.25f);
  EXPECT_FLOAT_EQ(s.base, 2.5f);
  EXPECT_FALSE(s.isRotated);
  EXPECT_FLOAT_EQ(s.crop.outerWidth, 1.0f);
  EXPECT_FLOAT_EQ(s.crop.outerHeight, 1500.0f / 1333.0f);
  EXPECT_FLOAT_EQ(s.crop.cropStartX, 0.0f);
  EXPECT_FLOAT_EQ(s.crop.cropStartY, 82.0f / 1500.0f);
}

std::string WriteFile(const std::string& name, const std::string& bytes) {
  const std::string path = ::testing::TempDir() + name;
  std::ofstream out(path, std::ios::binary);
  out.write(bytes.data(), static_cast<std::streamsize>(bytes.size()));
  return path;
}

// JSON (CLI output format) + luminance PGM -> DetectionImage.
TEST(TargetLoaderTest, LoadsCliJsonAndPgmPair) {
  const GrayImage img = SyntheticTarget(480, 640, 21);
  std::string pgm = "P5\n# synthetic\n480 640\n255\n";
  pgm.append(reinterpret_cast<const char*>(img.pixels.data()), img.pixels.size());
  const std::string json = R"({
  "type": "PLANAR",
  "properties": {
    "left": 155, "top": 249, "width": 480, "height": 640,
    "isRotated": false, "originalWidth": 1442, "originalHeight": 1866
  },
  "imagePath": "image-targets/pair-target_luminance.jpg",
  "metadata": null,
  "name": "pair-target",
  "resources": {"luminanceImage": "pair-target_luminance.jpg"},
  "created": 1776988431956,
  "updated": 1776988460359
}
)";
  const std::string json_path = WriteFile("sr_it_pair-target.json", json);
  const std::string pgm_path = WriteFile("sr_it_pair-target_luminance.pgm", pgm);
  ImageTargetSpec spec;
  GrayImage luminance;
  std::string error;
  ASSERT_TRUE(LoadImageTargetFiles(json_path, pgm_path, &spec, &luminance, &error)) << error;
  EXPECT_EQ(spec.name, "pair-target");
  EXPECT_EQ(luminance.width, 480);
  EXPECT_EQ(luminance.height, 640);
  EXPECT_EQ(luminance.pixels, img.pixels);
  const c8::DetectionImage im =
      LoadDetectionImage(spec, luminance.view(), TestPinholeK(), ShaderNumericModel::kIeee);
  EXPECT_EQ(im.getName(), "pair-target");
  EXPECT_GT(im.framePoints().points().size(), 1000u);

  EXPECT_FALSE(LoadImageTargetFiles(json_path, ::testing::TempDir() + "sr_it_missing.pgm", &spec,
                                    &luminance, &error));
  EXPECT_NE(error.find("cannot open"), std::string::npos);
}

}  // namespace
}  // namespace slam_refact::image_target

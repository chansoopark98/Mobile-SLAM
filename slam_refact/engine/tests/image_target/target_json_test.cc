// US-012: target JSON reader (CLI / Studio format, tracking-controller.ts:822-873 mapping) and the
// binary PGM reader.

#include <gtest/gtest.h>

#include <string>

#include "image_target/image_target_spec.h"
#include "image_target/target_files.h"
#include "image_target/target_json.h"

namespace slam_refact::image_target {
namespace {

// PUB:../aframe-image-targets-example/image-targets/video-target.json (CLI output, planar).
constexpr char kPlanarJson[] = R"({
  "type": "PLANAR",
  "properties": {
    "top": 968,
    "left": 755,
    "width": 584,
    "height": 779,
    "isRotated": false,
    "originalWidth": 1442,
    "originalHeight": 1866
  },
  "imagePath": "image-targets/video-target_luminance.jpg",
  "metadata": null,
  "name": "video-target",
  "resources": {
    "originalImage": "video-target_original.jpg",
    "croppedImage": "video-target_cropped.jpg",
    "thumbnailImage": "video-target_thumbnail.jpg",
    "luminanceImage": "video-target_luminance.jpg"
  },
  "created": 1776988466127,
  "updated": 1776988466127
}
)";

// Values of the CLI test "selectConicalGeometry handles wide top cone with default crop"
// (PUB:apps/image-target-cli/src/interactive.test.js).
constexpr char kConicalJson[] = R"({
  "type": "CONICAL",
  "properties": {
    "top": 452, "left": 0, "width": 823, "height": 1097, "isRotated": true,
    "originalWidth": 823, "originalHeight": 2000,
    "cylinderCircumferenceTop": 100, "cylinderCircumferenceBottom": 40,
    "targetCircumferenceTop": 50, "cylinderSideLength": 20.575, "inputMode": "ADVANCED",
    "coniness": 1.3219280948873624, "arcAngle": 180, "unit": "mm",
    "topRadius": 1500, "bottomRadius": 600
  },
  "imagePath": "image-targets/cone_luminance.png",
  "metadata": null,
  "name": "cone \u00e9\ud83d\ude00"
}
)";

TEST(TargetJsonTest, PlanarCliOutput) {
  ImageTargetSpec spec;
  std::string error;
  ASSERT_TRUE(ParseImageTargetJson(kPlanarJson, &spec, &error)) << error;
  EXPECT_EQ(spec.name, "video-target");
  EXPECT_EQ(spec.type, ImageTargetType::kPlanar);
  EXPECT_EQ(spec.type_name, "PLANAR");
  EXPECT_EQ(spec.image_path, "image-targets/video-target_luminance.jpg");
  EXPECT_EQ(spec.original_image_width, 1442);
  EXPECT_EQ(spec.original_image_height, 1866);
  EXPECT_EQ(spec.crop_original_image_x, 755);
  EXPECT_EQ(spec.crop_original_image_y, 968);
  EXPECT_EQ(spec.crop_original_image_width, 584);
  EXPECT_EQ(spec.crop_original_image_height, 779);
  EXPECT_FALSE(spec.is_rotated);
  EXPECT_FALSE(spec.is_static_target);
}

TEST(TargetJsonTest, ConicalIsCurvyWithCircumferences) {
  ImageTargetSpec spec;
  std::string error;
  ASSERT_TRUE(ParseImageTargetJson(kConicalJson, &spec, &error)) << error;
  EXPECT_EQ(spec.type, ImageTargetType::kCurvy);
  EXPECT_EQ(spec.type_name, "CONICAL");
  EXPECT_TRUE(spec.is_rotated);
  EXPECT_FLOAT_EQ(spec.curvy_circumference_top, 100.0f);
  EXPECT_FLOAT_EQ(spec.curvy_circumference_bottom, 40.0f);
  EXPECT_FLOAT_EQ(spec.curvy_side_length, 20.575f);
  EXPECT_FLOAT_EQ(spec.target_circumference_top, 50.0f);
  EXPECT_EQ(spec.name, "cone \xC3\xA9\xF0\x9F\x98\x80");  // \u escapes decoded to UTF-8
}

TEST(TargetJsonTest, StringIsRotatedStaticOrientationAndTruncation) {
  const char* json = R"({"name": "s", "type": "CYLINDER", "properties": {
    "top": 1.9, "left": -2.7, "width": 480, "height": 640, "isRotated": "true",
    "originalWidth": 480, "originalHeight": 640,
    "staticOrientation": {"rollAngle": 12.5, "pitchAngle": 0},
    "cylinderCircumferenceTop": 30, "cylinderCircumferenceBottom": 30,
    "targetCircumferenceTop": 10, "cylinderSideLength": 7.52}})";
  ImageTargetSpec spec;
  std::string error;
  ASSERT_TRUE(ParseImageTargetJson(json, &spec, &error)) << error;
  EXPECT_TRUE(spec.is_rotated);
  EXPECT_EQ(spec.crop_original_image_y, 1);   // Int32 setter truncates toward zero
  EXPECT_EQ(spec.crop_original_image_x, -2);
  EXPECT_TRUE(spec.is_static_target);
  EXPECT_FLOAT_EQ(spec.roll_angle, 12.5f);
  EXPECT_FLOAT_EQ(spec.pitch_angle, 0.0f);
  EXPECT_EQ(spec.type, ImageTargetType::kCurvy);
}

TEST(TargetJsonTest, UnknownTypeIsUnspecifiedAndNullOrientationIsNotStatic) {
  const char* json = R"({"name": "u", "type": "MESH", "properties": {
    "top": 0, "left": 0, "width": 480, "height": 640, "originalWidth": 480,
    "originalHeight": 640, "staticOrientation": null}})";
  ImageTargetSpec spec;
  std::string error;
  ASSERT_TRUE(ParseImageTargetJson(json, &spec, &error)) << error;
  EXPECT_EQ(spec.type, ImageTargetType::kUnspecified);
  EXPECT_FALSE(spec.is_static_target);
  EXPECT_FALSE(spec.is_rotated);
}

TEST(TargetJsonTest, RejectsMalformedAndIncompleteInput) {
  const char* bad[] = {
      "",
      "[]",
      "{\"name\": \"x\"",                                                   // truncated
      "{\"name\": \"x\", \"type\": \"PLANAR\"} trailing",                   // trailing data
      "{\"type\": \"PLANAR\", \"properties\": {}}",                         // no name
      "{\"name\": \"x\", \"type\": \"PLANAR\", \"properties\": {\"top\": 0}}",  // missing ints
      "{\"name\": \"x\", \"type\": \"PLANAR\", \"properties\": {\"top\": 0, \"left\": 0, "
      "\"width\": \"480\", \"height\": 640, \"originalWidth\": 1, \"originalHeight\": 1}}",
      // curvy target without its circumferences
      "{\"name\": \"x\", \"type\": \"CYLINDER\", \"properties\": {\"top\": 0, \"left\": 0, "
      "\"width\": 480, \"height\": 640, \"originalWidth\": 1, \"originalHeight\": 1}}",
      "{\"name\": \"bad \\ud83d\", \"type\": \"PLANAR\"}",  // unpaired surrogate
      "{\"name\": 01}",                                      // leading zero
  };
  for (const char* json : bad) {
    ImageTargetSpec spec;
    std::string error;
    EXPECT_FALSE(ParseImageTargetJson(json, &spec, &error)) << json;
    EXPECT_FALSE(error.empty()) << json;
  }
}

TEST(PgmTest, ParsesBinaryPgmWithComments) {
  std::string bytes = "P5\n# comment line\n3 2\n# another\n255\n";
  bytes += std::string("\x00\x10\x20\x30\x40\xff", 6);
  GrayImage img;
  std::string error;
  ASSERT_TRUE(ParsePgm(bytes, &img, &error)) << error;
  EXPECT_EQ(img.width, 3);
  EXPECT_EQ(img.height, 2);
  ASSERT_EQ(img.pixels.size(), 6u);
  EXPECT_EQ(img.pixels[1], 0x10);
  EXPECT_EQ(img.pixels[5], 0xff);
  const GrayImageView v = img.view();
  EXPECT_EQ(v.stride, 3);
  EXPECT_EQ(v.data, img.pixels.data());
}

TEST(PgmTest, RejectsUnsupportedOrInconsistentFiles) {
  const std::string pixels(6, '\x01');
  const std::string bad[] = {
      "P2\n3 2\n255\n" + pixels,      // ASCII PGM
      "P5\n3 2\n65535\n" + pixels,    // 16-bit
      "P5\n3 2\n255\n" + pixels + "x",  // raster too long
      "P5\n3 2\n255\n" + pixels.substr(1),
      "P5\n0 2\n255\n",
      "P5\n3",
  };
  for (const std::string& b : bad) {
    GrayImage img;
    std::string error;
    EXPECT_FALSE(ParsePgm(b, &img, &error));
    EXPECT_FALSE(error.empty());
  }
}

}  // namespace
}  // namespace slam_refact::image_target

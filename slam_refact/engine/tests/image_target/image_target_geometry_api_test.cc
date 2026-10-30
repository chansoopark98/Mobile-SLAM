// US-030 / D-1 (docs/design/request-engine-image-target-geometry.md): GlueRunner::image_target_geometry returns the load-time
// CurvyGeometry of a curvy target (what the original JS reads right after the load), false for flat or unknown targets.
// Checked-in code only (synthetic luminance), label unit. The Dumela label is the known answer of image_target_curvy_synth_test.cc
// (CurvyKnownAnswer.DumelaLabelNormalisedRadius): 15.3 cm of a 21.1034483 cm circumference, 10 cm tall.

#include <gtest/gtest.h>

#include <cmath>
#include <cstdlib>
#include <string>
#include <vector>

#include "image_target/target_files.h"
#include "slam_refact/glue_runner.h"
#include "test_util.h"

namespace slam_refact {
namespace {

std::string DumelaJson(const std::string& name) {
  return "{\"type\":\"CYLINDER\",\"name\":\"" + name +
         "\",\"imagePath\":\"" + name + ".png\",\"properties\":{\"left\":0,\"top\":0,"
         "\"width\":1530,\"height\":1000,\"isRotated\":false,\"originalWidth\":1530,\"originalHeight\":1000,"
         "\"cylinderCircumferenceTop\":21.1034483,\"cylinderCircumferenceBottom\":21.1034483,\"cylinderSideLength\":10,"
         "\"targetCircumferenceTop\":15.3}}";
}

TEST(GlueImageTargetGeometry, CurvyReportsGeometryFlatAndUnknownDoNot) {
  const image_target::GrayImage curvy_lum = image_target::testing::SyntheticTarget(306, 200, 12345);
  const image_target::GrayImage flat_lum = image_target::testing::SyntheticTarget(120, 160, 5);
  const std::string flat_json =
      "{\"type\":\"PLANAR\",\"name\":\"flat\",\"imagePath\":\"flat.png\",\"properties\":{\"left\":0,\"top\":0,"
      "\"width\":120,\"height\":160,\"isRotated\":false,\"originalWidth\":120,\"originalHeight\":160}}";
  EngineConfig e;
  e.capture_width = 480;
  e.capture_height = 640;
  GlueRunner glue(e, GlueConfig{});
  ASSERT_EQ(glue.SetImageTargets({{DumelaJson("dumela"), curvy_lum.view()}, {flat_json, flat_lum.view()}}), "");

  CurvyGeometryOut g;
  g.height = -1.f;
  EXPECT_FALSE(glue.image_target_geometry("flat", &g));
  EXPECT_FALSE(glue.image_target_geometry("missing", &g));
  EXPECT_EQ(g.height, -1.f) << "a false answer leaves *out alone";

  ASSERT_TRUE(glue.image_target_geometry("dumela", &g));
  // Known answers: a cylinder has equal radii; the engine's normalised radius is the radius in crop heights (0.335872);
  // the label covers 15.3 / 21.1034483 of the circumference.
  EXPECT_NEAR(g.top_radius, 0.335872f, 1e-4f);
  EXPECT_EQ(g.top_radius, g.bottom_radius);
  EXPECT_NEAR(g.arc_length_radians, 2.0 * M_PI * 15.3 / 21.1034483, 1e-3);
  EXPECT_GT(g.height, 0.f);

  // Unloading removes it.
  glue.UnloadImageTargets({"dumela"});
  EXPECT_FALSE(glue.image_target_geometry("dumela", &g));
}

// The phone demo cylinder (tools/image_target/make_demo_targets.py: 220 mm circumference, 160 mm wide, 80 mm tall label) read from
// files: SLAM_REFACT_DEMO_TARGET_JSON / SLAM_REFACT_DEMO_TARGET_PGM (the CLI JSON and its luminance image converted by to_pgm.mjs).
// Skipped without them (no checked-in PGM).
TEST(GlueImageTargetGeometry, DemoCylinderFromFiles) {
  const char* json_path = std::getenv("SLAM_REFACT_DEMO_TARGET_JSON");
  const char* pgm_path = std::getenv("SLAM_REFACT_DEMO_TARGET_PGM");
  if (json_path == nullptr || pgm_path == nullptr) GTEST_SKIP() << "SLAM_REFACT_DEMO_TARGET_JSON / _PGM not set";
  std::string json, bytes, error;
  image_target::GrayImage lum;
  ASSERT_TRUE(image_target::ReadFileBytes(json_path, &json, &error)) << error;
  ASSERT_TRUE(image_target::ReadFileBytes(pgm_path, &bytes, &error)) << error;
  ASSERT_TRUE(image_target::ParsePgm(bytes, &lum, &error)) << error;
  EngineConfig e;
  e.capture_width = 480;
  e.capture_height = 640;
  GlueRunner glue(e, GlueConfig{});
  ASSERT_EQ(glue.SetImageTargets({{json, lum.view()}}), "");
  CurvyGeometryOut g;
  ASSERT_TRUE(glue.image_target_geometry("demo-cylinder", &g));
  std::printf("[demo-cylinder geometry] height %.6f topRadius %.6f bottomRadius %.6f arcLength %.6f arcStart %.6f\n", g.height,
              g.top_radius, g.bottom_radius, g.arc_length_radians, g.arc_start_radians);
  EXPECT_EQ(g.top_radius, g.bottom_radius);
  // arc: the label covers 160 / 220 of the circumference
  EXPECT_NEAR(g.arc_length_radians, 2.0 * M_PI * 160.0 / 220.0, 1e-3);
  EXPECT_GT(g.top_radius, 0.f);
}

}  // namespace
}  // namespace slam_refact

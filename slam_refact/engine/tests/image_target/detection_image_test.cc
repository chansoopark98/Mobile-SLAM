// Port of PUB:reality/engine/imagedetection/detection-image-test.cc (US-011).
// Difference: MutableRootMessage<CurvyGeometry> (capnp) -> the plain CurvyGeometry::Data of the
// reality.capnp.h shim with its Builder/Reader; the assertions are unchanged.

#include <gtest/gtest.h>

#include <cmath>
#include <utility>

#include "reality/engine/api/reality.capnp.h"
#include "reality/engine/imagedetection/detection-image.h"
#include "test_util.h"

namespace c8 {
namespace {

using ::slam_refact::image_target::testing::TestK;

TEST(DetectionImageTest, toCurvyGeometryMessage) {
  TargetWithPoints framePoints(TestK());

  CurvyImageGeometry geom;
  CurvyImageGeometry fullGeom;
  curvyForTarget(306, 640, {1.f, false, {2.f, 2.f, 0.25f, 0.25f}, 1.375}, &geom, &fullGeom);
  DetectionImage detectionImage(
    "curvy", std::move(framePoints), 0, geom, fullGeom, HMatrixGen::i());

  CurvyGeometry::Data curvyGeometryData;
  CurvyGeometry::Builder curvyGeometry(&curvyGeometryData);
  detectionImage.toCurvyGeometry(curvyGeometry);
  CurvyGeometry::Reader reader = curvyGeometry.asReader();

  EXPECT_NEAR(reader.getCurvyCircumferenceTop(), fullGeom.radius * 2 * M_PI, 1e-6);
  EXPECT_NEAR(reader.getCurvyCircumferenceBottom(), fullGeom.radiusBottom * 2 * M_PI, 1e-6);
  float sideLength =
    std::sqrt(std::pow(fullGeom.radius - fullGeom.radiusBottom, 2) + std::pow(fullGeom.height, 2));
  EXPECT_NEAR(reader.getCurvySideLength(), sideLength, 1e-6);

  EXPECT_NEAR(reader.getHeight(), fullGeom.height, 1e-6);
  EXPECT_NEAR(reader.getTopRadius(), fullGeom.radius, 1e-6);
  EXPECT_NEAR(reader.getBottomRadius(), fullGeom.radiusBottom, 1e-6);
  EXPECT_NEAR(
    reader.getArcLengthRadians(),
    (fullGeom.activationRegion.right - fullGeom.activationRegion.left) * 2 * M_PI,
    1e-6);
  EXPECT_NEAR(reader.getArcStartRadians(), fullGeom.activationRegion.left * 2 * M_PI, 1e-6);
}

}  // namespace
}  // namespace c8

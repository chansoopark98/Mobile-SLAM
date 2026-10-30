// Port of PUB:reality/engine/imagedetection/image-targets-rigid-assembly-test.cc (US-011).
// Differences: EXPECT_THAT(m.data(), equalsMatrix(e)) -> EXPECT_TRUE(MatrixWithin(m, e, 1e-6))
// (same per-element |a - e| < eps rule; gmock is not built), testK() from the engine device table,
// TI() value-initializes the two debug members explicitly (-Wmissing-field-initializers).

#include <gtest/gtest.h>

#include <algorithm>

#include "c8/hmatrix.h"
#include "reality/engine/imagedetection/image-targets-rigid-assembly.h"
#include "test_util.h"

namespace c8 {
namespace {

using ::slam_refact::image_target::testing::MatrixWithin;
using ::slam_refact::image_target::testing::TestK;

constexpr float kEps = 1e-6f;

TrackedImage TI(int index, const HMatrix &pose) {
  return TrackedImage{
    TrackedImage::Status::TRACKED, index, "", pose, HMatrixGen::i(), 1.0f, 0, 0, TestK(), TestK(),
    {}, {}};
}

TEST(ImageTagetsRigidAssemblyTest, TestInit) {
  ImageTargetsRigidAssembly ra;
  EXPECT_EQ(ra.valid(), false);
  EXPECT_EQ(ra.valid(10), false);
  EXPECT_TRUE(MatrixWithin(ra.worldPose(), HMatrixGen::i(), kEps));

  Vector<TrackedImage> targets;
  ra.observe(targets);
  EXPECT_EQ(ra.valid(), false);
  EXPECT_EQ(ra.valid(10), false);
  EXPECT_TRUE(MatrixWithin(ra.worldPose(), HMatrixGen::i(), kEps));
}

TEST(ImageTagetsRigidAssemblyTest, TestObserve) {
  ImageTargetsRigidAssembly ra;
  Vector<TrackedImage> targets;
  ra.observe(targets);
  targets.emplace_back(TI(5, HMatrixGen::translation(1.0f, 5.0f, 0.0f)));

  ra.observe(targets);
  EXPECT_EQ(ra.valid(), true);
  EXPECT_EQ(ra.valid(3), false);
  EXPECT_EQ(ra.valid(5), true);
  EXPECT_TRUE(MatrixWithin(ra.worldPose(), targets.at(0).pose, kEps));

  // Move
  targets.at(0).pose = HMatrixGen::translation(1.0f, 5.0f, 0.5f);
  ra.observe(targets);
  EXPECT_TRUE(MatrixWithin(ra.worldPose(), targets.at(0).pose, kEps));

  // Observe second
  targets.emplace(targets.begin(), TI(3, HMatrixGen::translation(2.0f, 5.0f, 0.5f)));
  ra.observe(targets);
  EXPECT_TRUE(MatrixWithin(ra.worldPose(), targets.at(1).pose, kEps));
  EXPECT_TRUE(MatrixWithin(ra.transform(5), HMatrixGen::i(), kEps));
  EXPECT_TRUE(MatrixWithin(ra.transform(3), HMatrixGen::translation(-1.0f, 0.0f, 0.0f), kEps));

  // First is no longer observable.
  // Move the second
  targets.at(1).status = TrackedImage::Status::NOT_FOUND;
  targets.at(0).pose = targets.at(0).pose.translate(0.0f, 0.0f, -0.5f);
  ra.observe(targets);
  EXPECT_EQ(ra.valid(), true);
  EXPECT_EQ(ra.valid(3), true);
  EXPECT_EQ(ra.valid(5), false);
  EXPECT_TRUE(
    MatrixWithin(ra.worldPose(), targets.at(1).pose.translate(0.0f, 0.0f, -0.5f), kEps));
  EXPECT_TRUE(MatrixWithin(ra.transform(5), HMatrixGen::i(), kEps));
  EXPECT_TRUE(MatrixWithin(ra.transform(3), HMatrixGen::translation(-1.0f, 0.0f, 0.0f), kEps));

  // Loose the second
  targets.at(0).status = TrackedImage::Status::NOT_FOUND;
  ra.observe(targets);
  EXPECT_EQ(ra.valid(), false);  // none observable
  EXPECT_EQ(ra.valid(3), false);
  EXPECT_EQ(ra.valid(5), false);

  // Observe a third
  targets.emplace(targets.begin(), TI(1, HMatrixGen::translation(3.0f, 5.0f, 0.25f)));
  ra.observe(targets);
  EXPECT_EQ(ra.valid(), true);
  EXPECT_EQ(ra.valid(1), true);
  EXPECT_TRUE(MatrixWithin(ra.worldPose(), targets.at(0).pose, kEps));

  // Regain the first two, now relative to index=1 (at the end)
  std::reverse(targets.begin(), targets.end());
  targets.at(2).pose = targets.at(0).pose.translate(0.0f, 0.0f, -0.25f);
  targets.at(1).status = TrackedImage::Status::TRACKED;
  targets.at(0).status = TrackedImage::Status::TRACKED;
  ra.observe(targets);
  EXPECT_EQ(ra.valid(), true);
  EXPECT_EQ(ra.valid(1), true);
  EXPECT_EQ(ra.valid(3), true);
  EXPECT_EQ(ra.valid(5), true);
  EXPECT_TRUE(MatrixWithin(ra.worldPose(), targets.at(2).pose, kEps));
}

TEST(ImageTagetsRigidAssemblyTest, TestScale) {
  ImageTargetsRigidAssembly ra;
  Vector<TrackedImage> targets;
  ra.observe(targets);
  targets.emplace_back(TI(3, HMatrixGen::translation(1.0f, 5.0f, 0.0f)));
  targets.emplace_back(TI(5, HMatrixGen::translation(2.0f, 5.0f, 0.0f)));

  ra.observe(targets);
  EXPECT_EQ(ra.valid(), true);
  EXPECT_EQ(ra.valid(3), true);
  EXPECT_EQ(ra.valid(5), true);
  EXPECT_TRUE(MatrixWithin(ra.worldPose(), targets.at(0).pose, kEps));
  EXPECT_TRUE(MatrixWithin(ra.transform(3), HMatrixGen::i(), kEps));
  EXPECT_TRUE(MatrixWithin(ra.transform(5), HMatrixGen::translation(-1.0f, 0.0f, 0.0f), kEps));

  targets.at(0).pose = targets.at(0).pose.translate(0.0f, 0.0f, 1.0f);
  targets.at(1).pose = targets.at(1).pose.translate(0.0f, 0.0f, 0.25f);
  ra.observe(targets);
  EXPECT_TRUE(MatrixWithin(ra.worldPose(), targets.at(0).pose, kEps));
  EXPECT_TRUE(MatrixWithin(ra.transform(3), HMatrixGen::i(), kEps));
  // The public test ends with a commented-out expectation on transform(5) (relative scale is not
  // implemented upstream); it is not ported.
}

}  // namespace
}  // namespace c8

// verifyPointVisibility (1549) / projectPoints (1548) unit tests: status codes, scale LUT range
// boundaries, keyframe viewing-angle gate, view selection.

#include <cstdint>
#include <vector>

#include <gtest/gtest.h>

#include "c8/hmatrix.h"
#include "map/world_map.h"
#include "map_test_util.h"
#include "reality/engine/features/feature-scales.h"

namespace slam_refact::map {
namespace {

using testing::MakeFrame;
using testing::TestPinhole;

class VisibilityTest : public ::testing::Test {
 protected:
  // One keyframe at the origin with one keypoint at ray (0, -0.4) (ground-plane initial depth
  // 2.5), scale 1, linked to a triangulated point; returns the point id.
  PointId SetUpSinglePoint(const c8::HMatrix& kf_pose) {
    kf_ = map_.AddKeyframe(MakeFrame({{0.0f, -0.4f, 1, 5}}), 0);
    map_.SetKeyframePose(kf_, kf_pose);
    const PointId p = map_.AddPoint(0, c8::HPoint3(0.f, 0.f, 2.f), PointSource::kTriangulated);
    map_.AddObservation(p, 0, kf_);
    return p;
  }

  // dmin of the keyframe observation (1549: depthInWords if certainty > 0, else depth_).
  float Dmin() const {
    const c8::FramePoint& fp = map_.GetKeyframe(kf_).frame.points()[0];
    const double d = fp.depthWithCertainty().certainty > 0.0 ? fp.depthWithCertainty().depthInWords
                                                             : static_cast<double>(fp.depth());
    return c8::dMinForScale(fp.scale(), static_cast<float>(d));
  }

  VisibilityResult At(PointId p, float z) {
    map_.MutablePoint(p).position = c8::HPoint3(0.f, 0.f, z);
    return VerifyPointVisibility(map_, TestPinhole(), c8::HMatrixGen::i(), {p});
  }

  WorldMap map_;
  KeyframeId kf_;
};

TEST_F(VisibilityTest, ScaleRangeBoundaries) {
  const PointId p = SetUpSinglePoint(c8::HMatrixGen::i());
  const float dmin = Dmin();
  const float dmax = c8::dMaxForDMin(dmin);
  ASSERT_GT(dmin, 0.f);
  // z == dmin is accepted (dmin <= z), z == dmax is rejected (z < dmax).
  VisibilityResult r = At(p, dmin);
  EXPECT_EQ(r.status[0], 1);
  EXPECT_EQ(r.scale[0], c8::scaleForDist(dmin, dmin));
  ASSERT_EQ(r.views[0].size(), 1u);
  EXPECT_EQ(r.views[0][0].kf, kf_);
  EXPECT_EQ(r.views[0][0].point_index, 0u);
  r = At(p, std::nextafter(dmin, 0.f));
  EXPECT_EQ(r.status[0], 3);
  EXPECT_EQ(r.scale[0], 0xff);
  EXPECT_TRUE(r.views[0].empty());
  r = At(p, dmax);
  EXPECT_EQ(r.status[0], 3);
  r = At(p, std::nextafter(dmax, 0.f));
  EXPECT_EQ(r.status[0], 1);
}

TEST_F(VisibilityTest, BehindOrOutsideIsStatus2) {
  const PointId p = SetUpSinglePoint(c8::HMatrixGen::i());
  EXPECT_EQ(At(p, -1.f).status[0], 2);
  map_.MutablePoint(p).position = c8::HPoint3(10.f, 0.f, 2.f);  // x/z = 5, outside the frame
  EXPECT_EQ(VerifyPointVisibility(map_, TestPinhole(), c8::HMatrixGen::i(), {p}).status[0], 2);
}

TEST_F(VisibilityTest, InvalidOrDeadIsStatus0) {
  const PointId p = SetUpSinglePoint(c8::HMatrixGen::i());
  const VisibilityResult r = VerifyPointVisibility(
      map_, TestPinhole(), c8::HMatrixGen::i(), {PointId::nullId(), PointId(7, 7), p});
  EXPECT_EQ(r.status, std::vector<uint8_t>({0, 0, 1}));
  map_.Retire(p);
  EXPECT_EQ(VerifyPointVisibility(map_, TestPinhole(), c8::HMatrixGen::i(), {p}).status[0], 0);
}

TEST_F(VisibilityTest, KeyframeFacingAwayLeavesStatus0) {
  // Keyframe looking along +x: forward dot = 0 < maxViewingAngleKfToCam (0.5). Binary 1549
  // skips the observation and the point keeps status 0 (the S2 pseudo-code would give 3).
  const PointId p = SetUpSinglePoint(c8::HMatrixGen::yDegrees(90.f));
  EXPECT_EQ(At(p, 2.f).status[0], 0);
}

TEST(ProjectPoints, ChoosesMostUsedKeyframeTieSmallerSlot) {
  WorldMap map;
  const KeyframeId k0 =
      map.AddKeyframe(MakeFrame({{0.f, -0.4f, 1, 1}, {0.1f, -0.4f, 1, 2}}), 0);
  const KeyframeId k1 =
      map.AddKeyframe(MakeFrame({{0.f, -0.4f, 1, 3}, {0.1f, -0.4f, 1, 4}}), 1);
  const KeyframeId k2 = map.AddKeyframe(MakeFrame({{0.f, -0.4f, 1, 5}}), 2);
  for (KeyframeId kf : {k0, k1, k2}) {
    map.SetKeyframePose(kf, c8::HMatrixGen::i());
  }
  // a: views k0, k1, k2; b: views k1 only. Uses: k0 = 1, k1 = 2, k2 = 1.
  const PointId a = map.AddPoint(0, c8::HPoint3(0.f, -1.f, 2.5f), PointSource::kGround);
  const PointId b = map.AddPoint(0, c8::HPoint3(0.25f, -1.f, 2.5f), PointSource::kGround);
  map.AddObservation(a, 0, k0);
  map.AddObservation(a, 0, k1);
  map.AddObservation(a, 0, k2);
  map.AddObservation(b, 1, k1);
  const ProjectedPoints out = ProjectPoints(map, {}, TestPinhole(), c8::HMatrixGen::i());
  ASSERT_EQ(out.frame.points().size(), 2u);
  EXPECT_EQ(out.frame.points()[0].id(), a);
  EXPECT_EQ(out.frame.points()[1].id(), b);
  // Descriptor of a comes from k1 (GORB byte 3).
  EXPECT_EQ(out.frame.store().get<c8::GorbFeature>(0).data()[0], 3);
  EXPECT_EQ(out.frame.store().get<c8::GorbFeature>(1).data()[0], 4);
  // Tie: remove b's view, now k0 = k1 = k2 = 1 -> smallest slot k0 (byte 1).
  map.AddObservation(PointId::nullId(), 1, k1);
  const ProjectedPoints tie = ProjectPoints(map, {a}, TestPinhole(), c8::HMatrixGen::i());
  ASSERT_EQ(tie.frame.points().size(), 1u);
  EXPECT_EQ(tie.frame.store().get<c8::GorbFeature>(0).data()[0], 1);
  EXPECT_EQ(tie.frame.points()[0].position().x(), 0.f);
  EXPECT_EQ(tie.frame.points()[0].depth(), 2.5f);
}

}  // namespace
}  // namespace slam_refact::map

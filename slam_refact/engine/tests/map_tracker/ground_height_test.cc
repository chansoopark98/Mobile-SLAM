// MT-P6 (MUST-PASS): ground-height window of S2 §5.6 (F926:2440-2503) against hand-computed
// expectations: window of k = minGroundPointsForScale (8) sorted heights with the smallest
// spread (first on ties), median h[k/2 + i*], low pass with alpha 0.4, reset to 1.0 outside
// [-100, 100].

#include <vector>

#include <gtest/gtest.h>

#include "config/params.h"
#include "map_tracker/map_tracker.h"
#include "map_tracker/map_tracker_internal.h"

namespace slam_refact::map_tracker {
namespace {

TEST(MapTrackerGroundHeight, MtP6WindowMedianAndLowPass) {
  // Sorted: -1.3 -1.25 -1.24 -1.23 -1.22 -1.21 -1.2 -1.19 -1.18 0.9 2.0
  // Spreads (k = 8): i=0: -1.19+1.3 = 0.11, i=1: -1.18+1.25 = 0.07, i=2: 0.9+1.24, i=3: 2.0+1.23
  // -> i* = 1, median = h[1 + 4] = -1.21.
  std::vector<float> heights = {0.9f,   -1.2f,  -1.25f, -1.3f,  -1.22f, -1.18f,
                                -1.21f, -1.19f, -1.24f, 2.0f,   -1.23f};
  float ground_height = 1.0f;
  const float ty = 0.3f;
  ASSERT_TRUE(UpdateGroundHeight(&heights, 11, ty, &ground_height));
  const float expected = 1.0f * (1.0f - 0.4f) + 0.4f * (0.3f - (-1.21f));
  EXPECT_EQ(ground_height, expected);
  EXPECT_EQ(heights.front(), -1.3f);
  EXPECT_EQ(heights.back(), 2.0f);
}

TEST(MapTrackerGroundHeight, MtP6TiesTakeFirstWindow) {
  // 0..9: every window spans 7 -> i* = 0, median = h[4] = 4.
  std::vector<float> heights = {9, 8, 7, 6, 5, 4, 3, 2, 1, 0};
  float ground_height = 2.0f;
  ASSERT_TRUE(UpdateGroundHeight(&heights, 8, 10.0f, &ground_height));
  EXPECT_EQ(ground_height, 2.0f * (1.0f - 0.4f) + 0.4f * (10.0f - 4.0f));
}

TEST(MapTrackerGroundHeight, MtP6OutOfRangeResetsToOne) {
  std::vector<float> heights = {0, 0, 0, 0, 0, 0, 0, 0};
  float ground_height = 1.0f;
  ASSERT_TRUE(UpdateGroundHeight(&heights, 8, 500.0f, &ground_height));  // v = 0.6 + 200
  EXPECT_EQ(ground_height, 1.0f);
  std::vector<float> low = {0, 0, 0, 0, 0, 0, 0, 0};
  ground_height = 1.0f;
  ASSERT_TRUE(UpdateGroundHeight(&low, 8, -500.0f, &ground_height));  // v = 0.6 - 200
  EXPECT_EQ(ground_height, 1.0f);
  // Inside the range: exactly 100 is kept.
  std::vector<float> edge = {0, 0, 0, 0, 0, 0, 0, 0};
  ground_height = 100.0f;
  ASSERT_TRUE(UpdateGroundHeight(&edge, 8, 100.0f, &ground_height));
  EXPECT_EQ(ground_height, 100.0f * (1.0f - 0.4f) + 0.4f * 100.0f);
}

TEST(MapTrackerGroundHeight, MtP6TooFewSamplesLeavesHeight) {
  std::vector<float> seven = {1, 2, 3, 4, 5, 6, 7};
  float ground_height = 3.0f;
  EXPECT_FALSE(UpdateGroundHeight(&seven, 20, 1.0f, &ground_height));
  EXPECT_EQ(ground_height, 3.0f);
  std::vector<float> eight = {1, 2, 3, 4, 5, 6, 7, 8};
  EXPECT_FALSE(UpdateGroundHeight(&eight, 7, 1.0f, &ground_height));
  EXPECT_EQ(ground_height, 3.0f);
}

// S2 review LOW-2: k = 0 would read h[-1]; the override is a configuration error (the override
// only lives in the death-test child process).
TEST(MapTrackerGroundHeightDeathTest, WindowSizeZeroAborts) {
  std::vector<float> heights = {1.0f, 2.0f, 3.0f};
  float ground_height = 1.0f;
  EXPECT_DEATH(
      {
        config::ApplyParamOverrides({{"MapTracker.minGroundPointsForScale", ParamValue(0)}});
        UpdateGroundHeight(&heights, 3, 0.0f, &ground_height);
      },
      "minGroundPointsForScale must be >= 1");
}

// G640 (MapTracker ctor) writes the u64 0xbf8000003f800000 at +0x11e0: ground_height_ = 1.0f and
// ground_y_ = -1.0f. With ground_y_ = 0 the FFT scale pose.ty - ground_y_ starts near 0 instead of
// near 1 whenever the map-initialized event does not reset it (frame-start ty >= -0.999).
TEST(MapTrackerGroundHeight, ConstructorInitialValuesMatchG640) {
  MapTracker tracker;
  const MapTrackerAccess::Impl& impl = MapTrackerAccess::impl(tracker);
  EXPECT_EQ(impl.ground_height, 1.0f);
  EXPECT_EQ(impl.ground_y, -1.0f);
}

}  // namespace
}  // namespace slam_refact::map_tracker

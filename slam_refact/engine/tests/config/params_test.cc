// Runtime behavior of the globalParams()-backed settings (architecture.md §3.1, §4.6).
// Mutates process-global state; this executable must stay separate from other tests.

#include "c8/parameter-data.h"
#include "config/params.h"
#include "gtest/gtest.h"

namespace slam_refact::config {
namespace {

TEST(ConfigParams, GetReturnsBinaryDefaultsAndRegistersThem) {
  const MapTrackerSettings& s = MapTrackerSettings::Get();
  EXPECT_EQ(s.max_points, 400);
  EXPECT_EQ(s.tracking_lost_threshold, 0.005f);
  // getOrSet registered the key with the binary default.
  EXPECT_TRUE(c8::globalParams().contains("MapTracker.maxPoints"));
  EXPECT_EQ(c8::globalParams().get<int>("MapTracker.maxPoints"), 400);
}

TEST(ConfigParams, RegisterDefaultsCoversEveryTypedKey) {
  RegisterDefaults();
  int typed = 0;
  for (const ConfigKeyInfo& info : ConfigKeyTable()) {
    if (info.default_value.has_value()) {
      ++typed;
      EXPECT_TRUE(c8::globalParams().contains(info.key)) << info.key;
    } else {
      EXPECT_FALSE(c8::globalParams().contains(info.key)) << info.key;
    }
  }
  EXPECT_EQ(typed, 208);
}

TEST(ConfigParams, OverrideIsSeenAfterVersionChange) {
  const int v0 = ParamsVersion();
  EXPECT_EQ(WorldMapsSettings::Get().max_keyframes_in_map, 64);
  ApplyParamOverrides({{"WorldMaps.maxKeyframesInMap", ParamValue(32)},
                       {"WorldMaps.matchRadiusMax", ParamValue(0.04f)},
                       {"MapBuilder.loopCloseEnabled", ParamValue(false)},
                       {"MapBuilder.pruningGroundPointsTriangulatedCertaintyThreshold",
                        ParamValue(2e-4)}});
  EXPECT_GT(ParamsVersion(), v0);
  EXPECT_EQ(WorldMapsSettings::Get().max_keyframes_in_map, 32);
  EXPECT_EQ(WorldMapsSettings::Get().match_radius_max, 0.04f);
  EXPECT_EQ(MapBuilderSettings::Get().pruning_ground_points_triangulated_certainty_threshold,
            2e-4);
  // Restore so later tests in this executable see defaults.
  ApplyParamOverrides({{"WorldMaps.maxKeyframesInMap", ParamValue(64)},
                       {"WorldMaps.matchRadiusMax", ParamValue(0.05f)},
                       {"MapBuilder.pruningGroundPointsTriangulatedCertaintyThreshold",
                        ParamValue(1e-4)}});
  EXPECT_EQ(WorldMapsSettings::Get().max_keyframes_in_map, 64);
}

TEST(ConfigParams, CachedValueIsStableWithoutVersionChange) {
  const WorldMapsSettings* a = &WorldMapsSettings::Get();
  const WorldMapsSettings* b = &WorldMapsSettings::Get();
  EXPECT_EQ(a, b);
  EXPECT_EQ(a->max_subset_points, 75);
}

using ConfigParamsDeathTest = ::testing::Test;

TEST_F(ConfigParamsDeathTest, UnknownKeyAborts) {
  EXPECT_DEATH(ApplyParamOverrides({{"WorldMaps.noSuchKey", ParamValue(1)}}), "unknown config key");
}

TEST_F(ConfigParamsDeathTest, TypeMismatchAborts) {
  // maxKeyframesInMap is an int key (getter 851); a float value is rejected.
  EXPECT_DEATH(ApplyParamOverrides({{"WorldMaps.maxKeyframesInMap", ParamValue(64.f)}}),
               "does not match");
}

TEST_F(ConfigParamsDeathTest, UnrecoveredKeyAborts) {
  EXPECT_DEATH(ApplyParamOverrides({{"VpsMeshManager.debugDracoFile", ParamValue(1)}}),
               "no recovered type");
}

}  // namespace
}  // namespace slam_refact::config

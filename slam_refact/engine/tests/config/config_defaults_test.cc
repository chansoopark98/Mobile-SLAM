// Config table invariants and spot checks against the spec tables (S2 §4.1, S3 §4.1/§4.3,
// S5 §4.1). The full comparison with config-defaults.json is sr_config_defaults_vs_binary.

#include <cstring>
#include <set>
#include <string>
#include <variant>

#include "config/config_defaults.h"
#include "config/settings.h"
#include "gtest/gtest.h"

namespace slam_refact::config {
namespace {

// architecture.md §3.1 [M9]: 244 entries, 211 unique keys.
static_assert(kNumConfigKeys == 211);

TEST(ConfigDefaults, TableHasUniqueKeysAndConsistentKinds) {
  std::set<std::string> keys;
  int unrecovered = 0;
  for (const ConfigKeyInfo& info : ConfigKeyTable()) {
    EXPECT_TRUE(keys.insert(info.key).second) << "duplicate key " << info.key;
    ASSERT_NE(std::strchr(info.key, '.'), nullptr) << info.key;
    if (info.kind == ParamKind::kUnrecovered) {
      ++unrecovered;
      EXPECT_FALSE(info.default_value.has_value()) << info.key;
      EXPECT_EQ(info.getter_func, 0) << info.key;
      continue;
    }
    ASSERT_TRUE(info.default_value.has_value()) << info.key;
    // ParamValue alternative order: bool, int, float, double == ParamKind order.
    EXPECT_EQ(info.default_value->index(), static_cast<size_t>(info.kind)) << info.key;
    switch (info.kind) {
      case ParamKind::kBool: EXPECT_EQ(info.getter_func, 881) << info.key; break;
      case ParamKind::kInt: EXPECT_EQ(info.getter_func, 851) << info.key; break;
      case ParamKind::kFloat: EXPECT_EQ(info.getter_func, 458) << info.key; break;
      case ParamKind::kDouble: EXPECT_EQ(info.getter_func, 1131) << info.key; break;
      case ParamKind::kUnrecovered: break;
    }
  }
  EXPECT_EQ(keys.size(), 211u);
  // VPS/debug keys without a typed literal in the binary.
  EXPECT_EQ(unrecovered, 3);
  for (const char* k : {"RequestManager.usePreviouslyLocalizedNode", "VpsMeshManager.debugDracoFile",
                        "VpsRequestManager.debugSpecifiedNodeId"}) {
    const ConfigKeyInfo* info = FindConfigKey(k);
    ASSERT_NE(info, nullptr) << k;
    EXPECT_EQ(info->kind, ParamKind::kUnrecovered) << k;
  }
  EXPECT_EQ(FindConfigKey("No.suchKey"), nullptr);
}

TEST(ConfigDefaults, UnreadKeysAreRegisteredOnly) {
  // US-009 revision: S3 §4.3 (4 keys) and S2 §4.2 (2 keys) are never loaded by the binary.
  const std::set<std::string> unread = {
      "BundleAdjustment.cameraMotionWeight", "BundleAdjustment.numPreviousKeyframesToAdjust",
      "BundleAdjustment.minNumKeyframesForPointToAdjust", "MapBuilder.keyframeVariabilityCriterion",
      "WorldMaps.lostMatcherRadius", "WorldMaps.enableLostLocalMatcher"};
  int n_unread = 0;
  for (const ConfigKeyInfo& info : ConfigKeyTable()) {
    const bool expect_read = unread.count(info.key) == 0;
    EXPECT_EQ(info.read_by_binary, expect_read) << info.key;
    n_unread += info.read_by_binary ? 0 : 1;
  }
  EXPECT_EQ(n_unread, 6);
}

TEST(ConfigDefaults, StructDefaultsEqualTable) {
#define SR_CONFIG_BEGIN(prefix) { const prefix##Settings s;
#define SR_CONFIG_KEY(prefix, field, key, kind, ctype, def, getter, owner, read)            \
  {                                                                                          \
    const ConfigKeyInfo* info = FindConfigKey(key);                                          \
    ASSERT_NE(info, nullptr) << key;                                                         \
    ASSERT_TRUE(std::holds_alternative<ctype>(*info->default_value)) << key;                \
    EXPECT_EQ(s.field, std::get<ctype>(*info->default_value)) << key;                       \
    EXPECT_EQ(std::string(key).rfind(std::string(prefix##Settings::kPrefix) + ".", 0), 0u)  \
        << key;                                                                              \
  }
#define SR_CONFIG_END(prefix) }
#define SR_CONFIG_UNRECOVERED(prefix, key, func)
#include "config/config_defaults.inc"
#undef SR_CONFIG_BEGIN
#undef SR_CONFIG_KEY
#undef SR_CONFIG_END
#undef SR_CONFIG_UNRECOVERED
}

TEST(ConfigDefaults, SpecSpotChecks) {
  // S2 §4.1 (getter 633).
  const MapTrackerSettings mt;
  EXPECT_TRUE(mt.inertial_tracking);
  EXPECT_EQ(mt.initial_frame_delay, 10);
  EXPECT_EQ(mt.max_points, 400);
  EXPECT_EQ(mt.min_ground_points_for_scale, 8);
  EXPECT_EQ(mt.low_pass_ground_height_alpha, 0.4f);
  EXPECT_EQ(mt.relocalization_threshold, 0.05f);
  EXPECT_EQ(mt.cloud_map_max_relocalization_time_delay_nanos, 5e8f);
  EXPECT_EQ(mt.secret_relocalization_time_delay_nanos, 2e9f);
  EXPECT_EQ(mt.tracking_lost_threshold, 0.005f);
  EXPECT_EQ(mt.max_gyro_magnitude, 70.f);
  EXPECT_EQ(mt.max_linear_acceleration_magnitude, 1.7f);
  EXPECT_EQ(mt.reloc_to_depth_kf_dist, 76);
  EXPECT_FALSE(mt.use_external_relocalization_extrinsic);
  EXPECT_EQ(mt.max_points_for_second_pass_tracking, 5000);

  // S3 §4.1 (getter 651) and §4.3 (getter 1145).
  const MapBuilderSettings mb;
  EXPECT_EQ(mb.map_good_key_frame_count, 5);
  EXPECT_EQ(mb.min_parallax_threshold, 0.02f);
  EXPECT_EQ(mb.keyframe_minimum_new_points, 20);
  EXPECT_EQ(mb.pruning_ground_points_triangulated_certainty_threshold, 1e-4);
  EXPECT_EQ(mb.max_new_skypoints, 30);
  EXPECT_EQ(mb.skypoints_distance, 1000.f);
  EXPECT_FALSE(mb.loop_close_enabled);
  EXPECT_FALSE(mb.bundle_adjustment_iterative_enabled);
  const BundleAdjustmentSettings ba;
  EXPECT_EQ(ba.camera_motion_weight, 10.f);
  EXPECT_EQ(ba.num_previous_keyframes_to_adjust, 4);
  EXPECT_EQ(ba.min_num_points_for_keyframe_to_adjust, 5);
  EXPECT_EQ(ba.min_num_keyframes_for_point_to_adjust, 2);
  EXPECT_EQ(ba.num_global_iterative_iterations, 10);
  EXPECT_FALSE(ba.use_fast_point_adjustment_in_global_iterative);

  // S2 §4.2 (getter 1113) and S3 §4.2.
  const WorldMapsSettings wm;
  EXPECT_EQ(wm.max_keyframes_in_map, 64);
  EXPECT_EQ(wm.match_radius_max, 0.05f);
  EXPECT_EQ(wm.full_map_global_matcher_distance_threshold, 64);
  EXPECT_EQ(wm.ransac_max_num_iteration, 2500);
  EXPECT_EQ(wm.ransac_min_inlier_required, 14);
  EXPECT_EQ(wm.max_viewing_angle_kf_to_cam, 0.5f);
  const EpipolarMatcherSettings em;
  EXPECT_EQ(em.radius, 0.01f);
  EXPECT_EQ(em.descriptor_distance, 32);
  const FramePointSettings fp;
  EXPECT_EQ(fp.skypoints_distance, 100.f);

  // S5 §4.1 (getter 637).
  const MetricScaleEstimatorSettings mse;
  EXPECT_EQ(mse.scale_estimation_method, 1);
  EXPECT_EQ(mse.max_scale_allowed, 2.25f);
  EXPECT_EQ(mse.min_scale_allowed, 0.05f);
  EXPECT_EQ(mse.movement_check_period_in_sec, 1.f);
  EXPECT_TRUE(mse.place_with_inteval);  // key spelling as in the binary

  // VPS debug key stored unsigned in the JSON (4294967295) is int -1.
  const VpsRequestManagerSettings vps;
  EXPECT_EQ(vps.debug_max_num_successful_localizations, -1);
}

}  // namespace
}  // namespace slam_refact::config

// Per-getter settings structs (architecture.md §3.1). One struct per key prefix, e.g.
// MapTrackerSettings (getter 633), WorldMapsSettings (1113), MapBuilderSettings (651).
// Field defaults are the recovered binary defaults (config_defaults.inc); field names are the
// key names in snake_case (MapTracker.maxPointsForSecondPassTracking ->
// MapTrackerSettings::max_points_for_second_pass_tracking).
//
// Get() returns the values from the process-global globalParams() with a version cache: the
// keys are re-read (getOrSet with the binary default) only when the parameter version changes,
// like the original getters (S2 §4). Get() is implemented in params.cc (links sr_config).

#ifndef SLAM_REFACT_CONFIG_SETTINGS_H_
#define SLAM_REFACT_CONFIG_SETTINGS_H_

namespace slam_refact::config {

#define SR_CONFIG_BEGIN(prefix)                 \
  struct prefix##Settings {                     \
    static constexpr char kPrefix[] = #prefix;  \
    static const prefix##Settings& Get();
#define SR_CONFIG_KEY(prefix, field, key, kind, ctype, def, getter, owner, read) \
  ctype field = def;
#define SR_CONFIG_END(prefix) \
  };
#define SR_CONFIG_UNRECOVERED(prefix, key, func)
#include "config/config_defaults.inc"
#undef SR_CONFIG_BEGIN
#undef SR_CONFIG_KEY
#undef SR_CONFIG_END
#undef SR_CONFIG_UNRECOVERED

}  // namespace slam_refact::config

#endif  // SLAM_REFACT_CONFIG_SETTINGS_H_

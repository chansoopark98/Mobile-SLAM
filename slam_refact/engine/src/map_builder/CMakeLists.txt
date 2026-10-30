# Module target. Created by WP0b; owned by WP3 from the contract freeze (architecture.md §8.1).
# The glob picks up the .cc files in this directory.
sr_add_module(sr_map_builder
  DEPS sr_map sr_vendor_geometry Ceres::ceres)

# US-025 LC-2b/LC-3: the agate and loop close sources are part of the glob but are compiled only when
# SLAM_REFACT_LOOP_CLOSE is ON (the option decides inclusion only, nothing else changes).
get_target_property(_sr_mb_srcs sr_map_builder SOURCES)
if(SLAM_REFACT_LOOP_CLOSE)
  target_compile_definitions(sr_map_builder PUBLIC SLAM_REFACT_LOOP_CLOSE=1)
elseif(_sr_mb_srcs)
  list(FILTER _sr_mb_srcs EXCLUDE REGEX "/(agate_descriptor|loop_close)\\.cc$")
  set_property(TARGET sr_map_builder PROPERTY SOURCES ${_sr_mb_srcs})
endif()

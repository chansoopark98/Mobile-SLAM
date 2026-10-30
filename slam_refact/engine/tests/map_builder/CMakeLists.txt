# WP3 map builder tests (architecture.md §6.6.3, §7.3 "map_builder").
# Golden data: tests/data/map-builder/ (MANIFEST.tsv; generators gen_*.mjs in this directory).

sr_add_unit_test(sr_map_builder_parity_test
  SOURCES parity_test.cc map_builder_test_util.h
  LIBS sr_map_builder sr_config
  LABELS unit)
target_compile_definitions(sr_map_builder_parity_test PRIVATE
  SR_TEST_DATA_DIR="${SLAM_REFACT_TEST_DATA_DIR}")

sr_add_unit_test(sr_map_builder_policy_test
  SOURCES policy_test.cc map_builder_test_util.h
  LIBS sr_map_builder sr_config
  LABELS unit)

sr_add_unit_test(sr_map_builder_update_test
  SOURCES update_test.cc map_builder_test_util.h
  LIBS sr_map_builder sr_config
  LABELS unit)

sr_add_unit_test(sr_map_builder_bundle_test
  SOURCES bundle_test.cc map_builder_test_util.h
  LIBS sr_map_builder sr_config
  LABELS unit)

# Accuracy-mode local BA observation bookkeeping (review E-05).
sr_add_unit_test(sr_map_builder_local_ba_test SOURCES local_ba_test.cc map_builder_test_util.h LIBS sr_map_builder sr_config LABELS unit)

# US-025 LC-2b: agate global descriptor (golden bit comparison). Only with SLAM_REFACT_LOOP_CLOSE.
if(SLAM_REFACT_LOOP_CLOSE)
  sr_add_unit_test(sr_map_builder_agate_test
    SOURCES agate_test.cc
    LIBS sr_map_builder sr_compat
    LABELS unit)
  target_compile_definitions(sr_map_builder_agate_test PRIVATE
    SR_TEST_DATA_DIR="${SLAM_REFACT_TEST_DATA_DIR}")
endif()

# US-025 LC-4: loop close unit tests (golden: lc-g original-wasm dump; mutation checks in
# docs/design/us025-lc4-tests.md). Only with SLAM_REFACT_LOOP_CLOSE.
if(SLAM_REFACT_LOOP_CLOSE)
  sr_add_unit_test(sr_map_builder_loop_close_test
    SOURCES loop_close_test.cc map_builder_test_util.h
    LIBS sr_map_builder sr_config
    LABELS unit)
  target_compile_definitions(sr_map_builder_loop_close_test PRIVATE
    SR_TEST_DATA_DIR="${SLAM_REFACT_TEST_DATA_DIR}")
endif()

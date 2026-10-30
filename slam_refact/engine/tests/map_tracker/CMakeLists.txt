# WP2 MapTracker tests (architecture.md §7.3 "map_tracker").
sr_add_unit_test(sr_map_tracker_yaw_test
  SOURCES yaw_test.cc
  LIBS sr_map_tracker)
target_compile_definitions(sr_map_tracker_yaw_test PRIVATE
  SR_TEST_DATA_DIR="${SLAM_REFACT_TEST_DATA_DIR}")

sr_add_unit_test(sr_map_tracker_ground_height_test
  SOURCES ground_height_test.cc
  LIBS sr_map_tracker)

sr_add_unit_test(sr_map_tracker_pnp_test
  SOURCES pnp_test.cc scene_util.h
  LIBS sr_map_tracker)

sr_add_unit_test(sr_map_tracker_track_points_test
  SOURCES track_points_test.cc scene_util.h
  LIBS sr_map_tracker)

# MT-P2u feeds MapTracker outputs into the public status state machine (vio_tracker.h,
# LossStateMachine, implemented by WP4 in sr_vio_tracker_core).
sr_add_unit_test(sr_map_tracker_test
  SOURCES map_tracker_test.cc scene_util.h
  LIBS sr_map_tracker sr_vio_tracker_core)

# WP2 FrameFrameTracker tests (architecture.md §7.3 "frame_frame_tracker").
sr_add_unit_test(sr_fft_ground_homography_test
  SOURCES ground_homography_test.cc json_lite.h
  LIBS sr_frame_frame_tracker)
target_compile_definitions(sr_fft_ground_homography_test PRIVATE
  SR_TEST_DATA_DIR="${SLAM_REFACT_TEST_DATA_DIR}")

sr_add_unit_test(sr_fft_frame_frame_tracker_test
  SOURCES frame_frame_tracker_test.cc json_lite.h
  LIBS sr_frame_frame_tracker)
target_compile_definitions(sr_fft_frame_frame_tracker_test PRIVATE
  SR_TEST_DATA_DIR="${SLAM_REFACT_TEST_DATA_DIR}")

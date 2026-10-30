# WP4 vio_tracker tests (architecture.md §6.6.4: V-MP, V-H1/H2/H3, V-ST, V-3DOF, V-LATCH, V-XR,
# Q10, partial reset). Links the seam-only core (test doubles for WP1-WP3 modules).
sr_add_unit_test(sr_tests_vio_tracker
  SOURCES vio_tracker_test.cc
  LIBS sr_vio_tracker_core)
target_compile_definitions(sr_tests_vio_tracker PRIVATE SR_TEST_DATA_DIR="${SLAM_REFACT_TEST_DATA_DIR}")

# Accuracy mode ACC-5: tracking-stage visual-inertial prediction.
sr_add_unit_test(sr_tests_vi_prediction
  SOURCES vi_prediction_test.cc
  LIBS sr_vio_tracker_core)

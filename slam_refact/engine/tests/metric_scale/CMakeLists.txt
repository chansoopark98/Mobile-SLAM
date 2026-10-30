# WP4 metric_scale tests (architecture.md §6.6.5 MS-P1..P9).
sr_add_unit_test(sr_tests_metric_scale
  SOURCES metric_scale_test.cc
  LIBS sr_metric_scale)
target_compile_definitions(sr_tests_metric_scale PRIVATE SR_TEST_DATA_DIR="${SLAM_REFACT_TEST_DATA_DIR}")

# US-026 S10 unit goldens (docs/design/us026-design.md §4.4 G-MSE-U, §4.5 MS-P10/P11). Separate
# executable so the pinned sr_tests_metric_scale binary is unchanged; label us026 (not unit) because
# its tolerances are candidates until the us026-mse-unit gate file fixes them.
sr_add_unit_test(sr_tests_us026_method_precision
  SOURCES us026_method_precision_test.cc
  LIBS sr_metric_scale
  LABELS us026)
target_compile_definitions(sr_tests_us026_method_precision PRIVATE SR_TEST_DATA_DIR="${SLAM_REFACT_TEST_DATA_DIR}")
# The gate us026-mse-unit-gate.v1 (label held): same source with SR_US026_HELD. Fails closed (HELD) until the
# shared device sr_held (tests/support/sr_held.h) releases kind "judge" and the golden step of the same run (kind
# "golden", tools/gates/us026-mse-unit-v1/golden.mjs) left its marker and the registered goldens.
sr_add_unit_test(sr_tests_us026_method_precision_held
  SOURCES us026_method_precision_test.cc
  LIBS sr_metric_scale
  LABELS held)
target_compile_definitions(sr_tests_us026_method_precision_held PRIVATE SR_TEST_DATA_DIR="${SLAM_REFACT_TEST_DATA_DIR}"
  SR_US026_HELD=1 SR_US026_PREREG_DIR="${SLAM_REFACT_REPO_ROOT}/build/slam-refact-full-20261019/preregistration")

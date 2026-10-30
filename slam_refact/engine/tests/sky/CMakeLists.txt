# Sky tests (US-020; docs/spec/09-sky.md §5).
#
# unit      sr_tests_sky: pure functions against golden data from the original JS (golden/), the CPU
#           cubemap renderer and the LayersController state machine with a fake worker. No model, no
#           recording: fresh clones pass.
# mlmodel   sr_tests_sky_model: the deployed semantics model from assets/references (gitignored,
#           never copied). Same policy as tests/ml and tests/face (ml_assets.cmake): configure always
#           succeeds; when the model is missing ONE always-failing test (sr_sky_models_assets_missing)
#           is registered under `mlmodel` instead, and -DSLAM_REFACT_ALLOW_BLOCKED_MLMODEL=ON only stops
#           registering it.
# skygate   the gate bodies of the US-020 preregistration (build/slam-refact-full-20261019/
#           preregistration/sky-us020.v1.md) that compare this pipeline with the recorded original run:
#           added below (sr_tests_sky_gate). Label skygate, never `unit`.
sr_add_unit_test(sr_tests_sky
  SOURCES sky_unit_test.cc
  LIBS sr_sky)
target_compile_definitions(sr_tests_sky PRIVATE
  SR_SKY_GOLDEN_DIR="${CMAKE_CURRENT_SOURCE_DIR}/golden")

include("${CMAKE_CURRENT_SOURCE_DIR}/../ml/ml_assets.cmake")
if(NOT EXISTS "${SLAM_REFACT_REPO_ROOT}/assets/references/xr_reference/engine/resources/semantics-model.tflite")
  sr_ml_report_blocked(sr_sky_models_assets_missing mlmodel
    "missing sky model: engine/resources/semantics-model.tflite.")
else()
  add_executable(sr_tests_sky_model sky_model_test.cc)
  target_link_libraries(sr_tests_sky_model PRIVATE sr_sky GTest::gtest_main)
  sr_engine_warnings(sr_tests_sky_model)
  target_compile_definitions(sr_tests_sky_model PRIVATE SR_REPO_ROOT="${SLAM_REFACT_REPO_ROOT}")
  include(GoogleTest)
  gtest_discover_tests(sr_tests_sky_model
    PROPERTIES LABELS mlmodel TIMEOUT 900
    DISCOVERY_TIMEOUT 60)

  # skygate: the preregistered gate (HELD by the shared sr_held device, fails closed, until the manifest records the
  # independent pre-review, SR_SKY_REVIEWER carries the sha256 of its designation file and the caller names the registration:
  # SR_SKY_PREREG_DIR / _GATE_ID / _GATE_VERSION / _GATE_KIND; see sky_gate_test.cc and bench/sky_held.h).
  # The SR_SKY_PREREG_DIR compile definition below is NOT a registration: it only adds a preregistration directory that
  # report mode and the replay tool scan for sealed sets (the environment variable of the same name adds another).
  # Needs the sealed recordings: SR_SKY_REF_ROOT (reference sets), SR_SKY_REPORT_DIR (new directory).
  add_executable(sr_tests_sky_gate sky_gate_test.cc)
  target_link_libraries(sr_tests_sky_gate PRIVATE sr_sky GTest::gtest_main)
  sr_engine_warnings(sr_tests_sky_gate)
  target_compile_definitions(sr_tests_sky_gate PRIVATE
    SR_REPO_ROOT="${SLAM_REFACT_REPO_ROOT}"
    SR_SKY_PREREG_DIR="${SLAM_REFACT_REPO_ROOT}/build/slam-refact-full-20261019/preregistration")
  include(GoogleTest)
  gtest_discover_tests(sr_tests_sky_gate
    PROPERTIES LABELS skygate TIMEOUT 1800
    DISCOVERY_TIMEOUT 60)
endif()

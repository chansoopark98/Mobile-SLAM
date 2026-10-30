# Face and ear tests (US-018/US-019; docs/spec/08-face-ear.md §5).

# unit: ported public tests that need no model or image, the fidelity JSON goldens (byte copies in
# data/, see data/PROVENANCE.md), the CPU ROI renderer, the coordinate rewrite and the event logic.
sr_add_unit_test(sr_tests_face
  SOURCES face_geometry_test.cc face_tracker_logic_test.cc ear_fidelity_test.cc
          face_roi_renderer_test.cc face_coordinates_test.cc face_events_test.cc
          face_t1_input_distribution_test.cc
  LIBS sr_face)
target_compile_definitions(sr_tests_face PRIVATE
  SR_FACE_TEST_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/data")

# mlmodel (RM-1 label, as in tests/ml): the deployed face models from assets/references (gitignored,
# never copied). Not `evidence`: `ctest -L evidence` is a regular expression and the old name
# `ml-evidence` was selected by it. RM-1c: the same policy as tests/ml (ml_assets.cmake): configure
# succeeds; when a face model file is missing ONE always-failing test (sr_face_models_assets_missing)
# is registered under `mlmodel` instead of the tests below, and
# -DSLAM_REFACT_ALLOW_BLOCKED_MLMODEL=ON only stops registering it. Label `mlmodel` only: the
# `mlgate` label belongs to the ML regression gate (tests/ml). Once registered, a wrong model path at
# run time (SLAM_REFACT_ML_REPO_ROOT) still FAILS the tests; they never skip.
include("${CMAKE_CURRENT_SOURCE_DIR}/../ml/ml_assets.cmake")
set(_sr_face_models
  engine/resources/face-model.tflite
  engine/resources/face-mesh-model.tflite
  engine/resources/face-ear-model.tflite)
set(_sr_face_missing "")
foreach(_m IN LISTS _sr_face_models)
  if(NOT EXISTS "${SLAM_REFACT_REPO_ROOT}/assets/references/xr_reference/${_m}")
    list(APPEND _sr_face_missing "${_m}")
  endif()
endforeach()
if(_sr_face_missing)
  list(JOIN _sr_face_missing ", " _sr_face_missing_text)
  sr_ml_report_blocked(sr_face_models_assets_missing mlmodel "missing face models: ${_sr_face_missing_text}.")
else()
  add_executable(sr_tests_face_models face_models_evidence_test.cc)
  target_link_libraries(sr_tests_face_models PRIVATE sr_face GTest::gtest_main)
  sr_engine_warnings(sr_tests_face_models)
  target_compile_definitions(sr_tests_face_models PRIVATE
    SR_REPO_ROOT="${SLAM_REFACT_REPO_ROOT}")
  include(GoogleTest)
  gtest_discover_tests(sr_tests_face_models
    PROPERTIES LABELS mlmodel TIMEOUT 900
    DISCOVERY_TIMEOUT 60)
endif()

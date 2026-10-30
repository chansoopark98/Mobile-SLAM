# Hand tests (US-021; docs/spec/10-hand.md §5).
#
# unit      sr_tests_hand: image operations against OpenCV golden data (golden/), calculator math and the
#           tracker graph logic with synthetic model outputs. No model file needed.
# mlmodel   sr_tests_hand_model: the public MediaPipe palm / landmark models from assets/references
#           (gitignored, never copied). Same policy as tests/ml, tests/face, tests/sky (ml_assets.cmake).
# handgate  the gate bodies of the US-021 preregistrations (v1 in build/slam-refact-full-20261019/
#           preregistration/; v2 and later in the lane that registers them): added below (sr_tests_hand_gate). Never `unit`.
sr_add_unit_test(sr_tests_hand
  SOURCES hand_image_test.cc hand_geometry_test.cc
  LIBS sr_hand)
target_compile_definitions(sr_tests_hand PRIVATE
  SR_HAND_GOLDEN_DIR="${CMAKE_CURRENT_SOURCE_DIR}/golden"
  SR_HAND_GEOMETRY_GOLDEN_DIR="${CMAKE_CURRENT_SOURCE_DIR}/golden-geometry")

include("${CMAKE_CURRENT_SOURCE_DIR}/../ml/ml_assets.cmake")
set(_sr_hand_models
  official/mediapipe-fork/mediapipe/models/palm_detection.tflite
  official/mediapipe-fork/mediapipe/models/hand_landmark.tflite
  official/mediapipe-fork/mediapipe/models/hand_landmark_3d.tflite)
set(_sr_hand_missing "")
foreach(_m IN LISTS _sr_hand_models)
  if(NOT EXISTS "${SLAM_REFACT_REPO_ROOT}/assets/references/xr_reference/${_m}")
    list(APPEND _sr_hand_missing "${_m}")
  endif()
endforeach()
if(_sr_hand_missing)
  list(JOIN _sr_hand_missing ", " _sr_hand_missing_text)
  sr_ml_report_blocked(sr_hand_models_assets_missing mlmodel "missing hand models: ${_sr_hand_missing_text}.")
else()
  add_executable(sr_tests_hand_model hand_model_test.cc)
  target_link_libraries(sr_tests_hand_model PRIVATE sr_hand GTest::gtest_main)
  sr_engine_warnings(sr_tests_hand_model)
  target_compile_definitions(sr_tests_hand_model PRIVATE SR_REPO_ROOT="${SLAM_REFACT_REPO_ROOT}")
  include(GoogleTest)
  gtest_discover_tests(sr_tests_hand_model
    PROPERTIES LABELS mlmodel TIMEOUT 900
    DISCOVERY_TIMEOUT 60)
endif()

# handgate: the preregistered gate (HELD by the shared sr_held device, fails closed, until the manifest records the
# independent pre-review and the caller names the registration: SR_HAND_PREREG_DIR / _GATE_ID / _GATE_VERSION / _GATE_KIND,
# SR_HAND_REVIEWER; see hand_gate_test.cc and hand_held.h). docs/spec/10-hand.md §5. Label handgate, never `unit`; needs no
# model file (the real palm tensors are sealed files under SR_HAND_GATE_ROOT).
# The SR_HAND_PREREG_DIR compile definition below is NOT a registration: it only adds a preregistration directory that report
# mode scans for sealed data (the environment variable of the same name adds another). No gate id, version or file name of
# a registration is compiled in.
add_executable(sr_tests_hand_gate hand_gate_test.cc)
target_link_libraries(sr_tests_hand_gate PRIVATE sr_hand GTest::gtest_main)
sr_engine_warnings(sr_tests_hand_gate)
target_compile_definitions(sr_tests_hand_gate PRIVATE
  SR_HAND_REPO_ROOT="${SLAM_REFACT_REPO_ROOT}"
  SR_HAND_PREREG_DIR="${SLAM_REFACT_REPO_ROOT}/build/slam-refact-full-20261019/preregistration")
include(GoogleTest)
gtest_discover_tests(sr_tests_hand_gate
  PROPERTIES LABELS handgate TIMEOUT 600
  DISCOVERY_TIMEOUT 60)

# Image target tests (full-parity PRD US-011, US-012). Checked-in data only (label unit):
# the public test images are git-lfs pointers, so the ported tests use synthetic targets
# (test_util.h).

sr_add_unit_test(sr_tests_image_target_vendor
  SOURCES detection_image_local_matcher_test.cc image_targets_rigid_assembly_test.cc
          detection_image_test.cc detection_image_loader_test.cc detection_image_tracker_test.cc
  LIBS sr_image_target sr_frontend
  LABELS unit)

sr_add_unit_test(sr_tests_image_target
  SOURCES target_loader_test.cc target_json_test.cc
  LIBS sr_image_target sr_frontend
  LABELS unit)

# US-013 integration (contract FULL-3): config switch, ROI slots, RNG sharing, limits.
sr_add_unit_test(sr_tests_image_target_vio
  SOURCES vio_image_target_test.cc image_target_geometry_api_test.cc
  LIBS sr_engine sr_image_target sr_vio_tracker sr_frontend
  LABELS unit)

# US-015 closed image world tracker (func994, contract FULL-12, opt-in): constructed geometry with known
# answers. sr_iwt_replay is the engine side of the func994 oracle comparison (tools/re/iwt_oracle), run by
# hand; it is not a test.
sr_add_unit_test(sr_tests_image_world_tracker
  SOURCES image_world_tracker_test.cc
  LIBS sr_image_target sr_vio_tracker sr_frontend
  LABELS unit)
add_executable(sr_iwt_replay iwt_replay_main.cc)
target_link_libraries(sr_iwt_replay PRIVATE sr_image_target)
sr_engine_warnings(sr_iwt_replay)
# Review F-1: default-off output dump of a detected/tracked/lost sequence, compared byte for byte with the same
# source built against the unpatched public files (tools/re/iwt_oracle/offpath_ab.py). Not a test.
add_executable(sr_iwt_offpath_dump iwt_offpath_dump_main.cc)
target_link_libraries(sr_iwt_offpath_dump PRIVATE sr_image_target sr_frontend GTest::gtest)
sr_engine_warnings(sr_iwt_offpath_dump)

# US-013 synthetic render gate (docs/design/image-target.md §7, tolerances fixed before the run).
# The fixture renders the sequence with the checked-in Python tool (numpy only, development
# dependency); the gate tests read it from SLAM_REFACT_IT_SYNTH_DIR.
find_package(Python3 COMPONENTS Interpreter)
if(NOT Python3_Interpreter_FOUND)
  # Only this gate needs Python; everything above builds and runs without it (RM-2).
  message(STATUS "slam_refact: Python3 not found: sr_image_target_synth_render and "
                 "sr_tests_image_target_synth are NOT registered (BLOCKED, not skipped)")
  return()
endif()
set(_sr_it_synth_dir "${CMAKE_CURRENT_BINARY_DIR}/image-target-synth")
add_test(NAME sr_image_target_synth_render
  COMMAND "${Python3_EXECUTABLE}" -I
          "${SLAM_REFACT_REPO_ROOT}/slam_refact/tools/image_target/render_synthetic.py"
          --out "${_sr_it_synth_dir}" --force)
set_tests_properties(sr_image_target_synth_render PROPERTIES
  FIXTURES_SETUP sr_it_synth LABELS unit)
add_executable(sr_tests_image_target_synth image_target_synth_test.cc)
target_link_libraries(sr_tests_image_target_synth PRIVATE sr_engine sr_image_target Eigen3::Eigen
                                                          GTest::gtest_main)
sr_engine_warnings(sr_tests_image_target_synth)
include(GoogleTest)
gtest_discover_tests(sr_tests_image_target_synth
  PROPERTIES LABELS unit FIXTURES_REQUIRED sr_it_synth
             ENVIRONMENT "SLAM_REFACT_IT_SYNTH_DIR=${_sr_it_synth_dir}" TIMEOUT 900
  DISCOVERY_TIMEOUT 60)

# RM-6 (review M1): ROI draw against the independent numpy oracle (gates G-ROI-A / G-ROI-B,
# preregistration/rm6-it-roi-gate.v1.md), null-runs and mutation detection. Needs python3 + numpy.
set(_sr_it_roi_dir "${CMAKE_CURRENT_BINARY_DIR}/image-target-roi")
add_executable(sr_tests_image_target_roi image_target_roi_test.cc)
target_link_libraries(sr_tests_image_target_roi PRIVATE sr_image_target sr_frontend GTest::gtest_main)
sr_engine_warnings(sr_tests_image_target_roi)
# The paths are compile definitions: a ';' separated ENVIRONMENT list does not survive
# gtest_discover_tests (it would keep only the first variable).
target_compile_definitions(sr_tests_image_target_roi PRIVATE
  "SR_RM6_PYTHON=\"${Python3_EXECUTABLE}\""
  "SR_RM6_ROI_REFERENCE=\"${SLAM_REFACT_REPO_ROOT}/slam_refact/tools/image_target/roi_reference.py\""
  "SR_RM6_ROI_DIR=\"${_sr_it_roi_dir}\"")
gtest_discover_tests(sr_tests_image_target_roi
  PROPERTIES LABELS unit TIMEOUT 300
  DISCOVERY_TIMEOUT 60)

# RM-6 synthetic-sequence gates G-TRAJ, G-K, G-ROI-C (+ C0) on the fixture sequence rendered above.
add_executable(sr_tests_image_target_rm6_synth image_target_rm6_synth_test.cc)
target_link_libraries(sr_tests_image_target_rm6_synth PRIVATE sr_engine sr_image_target Eigen3::Eigen
                                                              GTest::gtest_main)
sr_engine_warnings(sr_tests_image_target_rm6_synth)
gtest_discover_tests(sr_tests_image_target_rm6_synth
  PROPERTIES LABELS unit FIXTURES_REQUIRED sr_it_synth
             ENVIRONMENT "SLAM_REFACT_IT_SYNTH_DIR=${_sr_it_synth_dir}" TIMEOUT 900
  DISCOVERY_TIMEOUT 60)

# RM-4b (contract FULL-7): an injected atlas whose ROI slots a producer drew gives the CPU path's
# result bit for bit, and the engine reads the injected slots (cleared slots change the result).
add_executable(sr_tests_image_target_injected_roi image_target_injected_roi_test.cc)
target_link_libraries(sr_tests_image_target_injected_roi PRIVATE sr_engine sr_image_target
                                                                 GTest::gtest_main)
sr_engine_warnings(sr_tests_image_target_injected_roi)
gtest_discover_tests(sr_tests_image_target_injected_roi
  PROPERTIES LABELS unit FIXTURES_REQUIRED sr_it_synth
             ENVIRONMENT "SLAM_REFACT_IT_SYNTH_DIR=${_sr_it_synth_dir}" TIMEOUT 900
  DISCOVERY_TIMEOUT 60)

# Mutation detection of G-ROI-C: copies of roi_draw.cc with a bug in the slot draw (the shared
# source is not touched). The copy defines RoiAtlasRows / DrawRoisIntoAtlas itself, so the linker
# does not pull roi_draw.o out of libsr_frontend. The mutants run the same ratio test and must fail C1 or C2
# (env SLAM_REFACT_RM6_EXPECT_MUTANT=1 inverts C1/C2), so the gate keeps proving that it sees a flipped or mis-scaled ROI.
set(_sr_it_roi_src "${CMAKE_CURRENT_SOURCE_DIR}/../../src/frontend/roi_draw.cc")
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS "${_sr_it_roi_src}")
file(READ "${_sr_it_roi_src}" _sr_it_roi_text)
set(_sr_it_ny_old "const double ny = 2.0 * ((y + 0.5) / slot.h) - 1.0")
set(_sr_it_nx_old "const double nx = 2.0 * ((x + 0.5) / slot.w) - 1.0")
foreach(_mut MF MZ MS)
  if(_mut STREQUAL "MF")  # vertical flip of the slot rows
    string(REPLACE "${_sr_it_ny_old}" "const double ny = 1.0 - 2.0 * ((y + 0.5) / slot.h)"
           _sr_it_mutated "${_sr_it_roi_text}")
  elseif(_mut STREQUAL "MS")  # slot coordinate x shifted by +0.5 NDC
    string(REPLACE "${_sr_it_nx_old}" "const double nx = (2.0 * ((x + 0.5) / slot.w) - 1.0) + 0.5"
           _sr_it_mutated "${_sr_it_roi_text}")
  else()                  # scale error 1.15 of the slot coordinates
    string(REPLACE "${_sr_it_ny_old}" "const double ny = 1.15 * (2.0 * ((y + 0.5) / slot.h) - 1.0)"
           _sr_it_mutated "${_sr_it_roi_text}")
    string(REPLACE "${_sr_it_nx_old}" "const double nx = 1.15 * (2.0 * ((x + 0.5) / slot.w) - 1.0)"
           _sr_it_mutated "${_sr_it_mutated}")
  endif()
  if(_sr_it_mutated STREQUAL _sr_it_roi_text)
    message(FATAL_ERROR "RM-6 mutant ${_mut}: roi_draw.cc no longer contains the mutated lines")
  endif()
  file(WRITE "${CMAKE_CURRENT_BINARY_DIR}/mutants/roi_draw_${_mut}.cc" "${_sr_it_mutated}")
  add_executable(sr_tests_image_target_rm6_mutant_${_mut} image_target_rm6_synth_test.cc
                 "${CMAKE_CURRENT_BINARY_DIR}/mutants/roi_draw_${_mut}.cc")
  target_link_libraries(sr_tests_image_target_rm6_mutant_${_mut}
    PRIVATE sr_engine sr_image_target Eigen3::Eigen GTest::gtest_main)
  sr_engine_warnings(sr_tests_image_target_rm6_mutant_${_mut})
  add_test(NAME sr_rm6_mutant_${_mut}_detected
    COMMAND sr_tests_image_target_rm6_mutant_${_mut} --gtest_filter=Rm6Synth.RoiLocalSuccessRatio)
  set_tests_properties(sr_rm6_mutant_${_mut}_detected PROPERTIES
    LABELS "unit;mutation" FIXTURES_REQUIRED sr_it_synth TIMEOUT 900
    ENVIRONMENT "SLAM_REFACT_IT_SYNTH_DIR=${_sr_it_synth_dir};SLAM_REFACT_RM6_EXPECT_MUTANT=1")
endforeach()

# US-014 curved (cylinder / cone) targets: gates G-CROI-A/B (curvy ROI draw against the independent numpy
# oracle curvy_reference.py), G-CGEO, G-CTRAJ, IT-T/P/W and G-CROI-C on sequences rendered by
# render_synthetic.py --shape (ray casting), mutation detection (preregistration/curvy-it-gate.v<N>.md; N, the gate id and
# the preregistration directory are NOT compiled in: SLAM_REFACT_CURVY_GATE_ID / _GATE_VERSION / _PREREG_DIR, see
# curvy_sr_held.h).
# Needs python3 + numpy.
#
# DATA SEPARATION and HELD (since v3, after the v2 review B-1 / M-1): every test that runs before the independent review
# (null-runs, mutation, report, mutant executables) uses the DEV variant only (fixtures ..._dev_*) and asserts it. The
# gate tests (label `held`, one ctest entry per executable so that one process = one real run) use the REGISTERED variant
# (fixtures ..._registered_*, rendering only) and refuse to start (HELD) unless the coordinator's designation file, the
# review record, the pinned binary, the input tree sha256 and the one-run marker allow it (curvy_sr_held.h over the shared
# support/sr_held.h). The registered data is never an engine input on the unit path. `ctest -L unit` therefore never touches
# it. The coordinator seed of the registered variant is read by the rendering tools from SLAM_REFACT_CURVY_SEED_FILE (or
# --seed-file); there is no default seed.
function(sr_curvy_unit_discover target held_filter timeout)
  if(CMAKE_VERSION VERSION_GREATER_EQUAL 3.22)
    gtest_discover_tests(${target} TEST_FILTER "-${held_filter}"
      PROPERTIES LABELS unit FIXTURES_REQUIRED sr_it_curvy_dev TIMEOUT ${timeout}
      DISCOVERY_TIMEOUT 60)
  else()
    message(STATUS "slam_refact: CMake < 3.22: the unit tests of ${target} are not registered (no TEST_FILTER)")
  endif()
endfunction()

set(_sr_it_curvy_roi_dir "${CMAKE_CURRENT_BINARY_DIR}/image-target-curvy-roi")
foreach(_shape cylinder cone)
  add_test(NAME sr_image_target_curvy_render_dev_${_shape}
    COMMAND "${Python3_EXECUTABLE}" -I
            "${SLAM_REFACT_REPO_ROOT}/slam_refact/tools/image_target/render_synthetic.py"
            --shape ${_shape} --variant dev --out "${CMAKE_CURRENT_BINARY_DIR}/image-target-curvy-dev-${_shape}" --force)
  set_tests_properties(sr_image_target_curvy_render_dev_${_shape} PROPERTIES
    FIXTURES_SETUP sr_it_curvy_dev LABELS unit)
  # rendering only (numpy, no engine); the registered data is engine input only behind the HELD device
  add_test(NAME sr_image_target_curvy_render_registered_${_shape}
    COMMAND "${Python3_EXECUTABLE}" -I
            "${SLAM_REFACT_REPO_ROOT}/slam_refact/tools/image_target/render_synthetic.py"
            --shape ${_shape} --variant registered
            --out "${CMAKE_CURRENT_BINARY_DIR}/image-target-curvy-registered-${_shape}" --force)
  set_tests_properties(sr_image_target_curvy_render_registered_${_shape} PROPERTIES
    FIXTURES_SETUP sr_it_curvy_registered LABELS held)
endforeach()
set(_sr_curvy_defs
  "SR_CURVY_PYTHON=\"${Python3_EXECUTABLE}\""
  "SR_CURVY_REFERENCE=\"${SLAM_REFACT_REPO_ROOT}/slam_refact/tools/image_target/curvy_reference.py\""
  "SR_CURVY_HELD_TOOL=\"${SLAM_REFACT_REPO_ROOT}/slam_refact/tools/image_target/curvy_held_tool.py\""
  "SR_CURVY_DIR=\"${_sr_it_curvy_roi_dir}\""
  "SR_CURVY_CYL_DIR_DEV=\"${CMAKE_CURRENT_BINARY_DIR}/image-target-curvy-dev-cylinder\""
  "SR_CURVY_CONE_DIR_DEV=\"${CMAKE_CURRENT_BINARY_DIR}/image-target-curvy-dev-cone\""
  "SR_CURVY_CYL_DIR_REG=\"${CMAKE_CURRENT_BINARY_DIR}/image-target-curvy-registered-cylinder\""
  "SR_CURVY_CONE_DIR_REG=\"${CMAKE_CURRENT_BINARY_DIR}/image-target-curvy-registered-cone\"")
add_executable(sr_tests_image_target_curvy_roi image_target_curvy_roi_test.cc)
target_link_libraries(sr_tests_image_target_curvy_roi PRIVATE sr_image_target sr_frontend GTest::gtest_main)
sr_engine_warnings(sr_tests_image_target_curvy_roi)
target_compile_definitions(sr_tests_image_target_curvy_roi PRIVATE ${_sr_curvy_defs})
sr_curvy_unit_discover(sr_tests_image_target_curvy_roi "CurvyRoiOracle.GatesAB" 300)
add_test(NAME sr_curvy_gate_roi COMMAND sr_tests_image_target_curvy_roi --gtest_filter=CurvyRoiOracle.GatesAB)
set_tests_properties(sr_curvy_gate_roi PROPERTIES LABELS held TIMEOUT 300)
set(_sr_curvy_held_synth_filter
  "*GeometryAgainstPhysicalModel*:*TrajectoryAndKMatchRegistration*:*ImageTargetOnly*:*WorldOn*:*RoiLocalSuccessRatio*:CurvyCrop.*")
add_executable(sr_tests_image_target_curvy_synth image_target_curvy_synth_test.cc)
target_link_libraries(sr_tests_image_target_curvy_synth PRIVATE sr_engine sr_image_target Eigen3::Eigen
                                                                GTest::gtest_main)
sr_engine_warnings(sr_tests_image_target_curvy_synth)
target_compile_definitions(sr_tests_image_target_curvy_synth PRIVATE ${_sr_curvy_defs})
sr_curvy_unit_discover(sr_tests_image_target_curvy_synth "${_sr_curvy_held_synth_filter}" 900)
add_test(NAME sr_curvy_gate_synth COMMAND sr_tests_image_target_curvy_synth --gtest_filter=${_sr_curvy_held_synth_filter})
set_tests_properties(sr_curvy_gate_synth PROPERTIES LABELS held TIMEOUT 1800
                     FIXTURES_REQUIRED sr_it_curvy_registered)
# The HELD device. curvy_held.h / curvy_held_test.cc are the v3 per-gate device, kept unchanged because its_held.h (it-scale
# v1, already run) builds on that header. The curvy gate itself uses the shared device through curvy_sr_held.h: unit tests of
# that front on synthetic files (REPORT mode without a selection, every malformed or bypassing input HELD for the expected
# reason, one release per kind) and the command line front used by the WASM node test.
add_executable(sr_tests_image_target_curvy_sr_held curvy_sr_held_test.cc)
target_link_libraries(sr_tests_image_target_curvy_sr_held PRIVATE GTest::gtest_main)
sr_engine_warnings(sr_tests_image_target_curvy_sr_held)
gtest_discover_tests(sr_tests_image_target_curvy_sr_held PROPERTIES LABELS unit TIMEOUT 120 DISCOVERY_TIMEOUT 60)
add_executable(sr_tests_image_target_curvy_held curvy_held_test.cc)
target_link_libraries(sr_tests_image_target_curvy_held PRIVATE GTest::gtest_main)
sr_engine_warnings(sr_tests_image_target_curvy_held)
target_compile_definitions(sr_tests_image_target_curvy_held PRIVATE ${_sr_curvy_defs})
gtest_discover_tests(sr_tests_image_target_curvy_held PROPERTIES LABELS unit TIMEOUT 120 DISCOVERY_TIMEOUT 60)
add_executable(sr_curvy_held_check sr_curvy_held_check.cc)
target_compile_definitions(sr_curvy_held_check PRIVATE ${_sr_curvy_defs})

# Mutation detection of the curvy ROI draw: copies of roi_draw.cc with one draw bug (the shared source is
# not touched; the copy defines RoiAtlasRows / DrawRoisIntoAtlas / ComputeCurvyRoiUniforms itself, so the
# linker does not pull roi_draw.o out of libsr_frontend). With SLAM_REFACT_CURVY_EXPECT_MUTANT=1 the oracle
# test asserts that G-CROI-A or G-CROI-B is violated, the sequence test that C1 or C2 fails.
#   MV mirrored lift (sign of sin: x and the normal mirrored)   MY vertical flip of the model height
#   MN no back-face test                                        MR top / bottom radius swapped
#   MP post rotation 0: x and y of the camera pixel exchanged
foreach(_mut MV MY MN MR MP)
  file(READ "${_sr_it_roi_src}" _sr_curvy_text)
  set(_sr_curvy_mutated "${_sr_curvy_text}")
  if(_mut STREQUAL "MV")
    string(REPLACE "-radius_at_y * sin_v" "radius_at_y * sin_v" _sr_curvy_mutated "${_sr_curvy_text}")
  elseif(_mut STREQUAL "MY")
    string(REPLACE "-u.scales[1] * ppy + 0.5f" "u.scales[1] * ppy - 0.5f" _sr_curvy_mutated "${_sr_curvy_text}")
  elseif(_mut STREQUAL "MN")
    string(REPLACE "continue;  // faces away from the camera: liftedUv = (-1, -1)"
           "/* mutant: no back-face test */" _sr_curvy_mutated "${_sr_curvy_text}")
  elseif(_mut STREQUAL "MR")
    string(REPLACE "tv * u.radius_bottom + u.radius * (1.0f - tv)"
           "tv * u.radius + u.radius_bottom * (1.0f - tv)" _sr_curvy_mutated "${_sr_curvy_text}")
  else()
    string(REPLACE "lx = pix_x_pt / u.search_dims[0];" "lx = pix_y_pt / u.search_dims[1];"
           _sr_curvy_mutated "${_sr_curvy_text}")
    string(REPLACE "ly = pix_y_pt / u.search_dims[1];" "ly = pix_x_pt / u.search_dims[0];"
           _sr_curvy_mutated "${_sr_curvy_mutated}")
  endif()
  if(_sr_curvy_mutated STREQUAL _sr_curvy_text)
    message(FATAL_ERROR "curvy mutant ${_mut}: roi_draw.cc no longer contains the mutated text")
  endif()
  file(WRITE "${CMAKE_CURRENT_BINARY_DIR}/mutants/roi_draw_curvy_${_mut}.cc" "${_sr_curvy_mutated}")
  add_executable(sr_tests_image_target_curvy_roi_mutant_${_mut} image_target_curvy_roi_test.cc
                 "${CMAKE_CURRENT_BINARY_DIR}/mutants/roi_draw_curvy_${_mut}.cc")
  target_link_libraries(sr_tests_image_target_curvy_roi_mutant_${_mut}
    PRIVATE sr_image_target sr_frontend GTest::gtest_main)
  sr_engine_warnings(sr_tests_image_target_curvy_roi_mutant_${_mut})
  target_compile_definitions(sr_tests_image_target_curvy_roi_mutant_${_mut} PRIVATE ${_sr_curvy_defs}
                             SR_CURVY_MUTANT_BUILD=1)
  # dev variant only (SR_CURVY_MUTANT_BUILD): the oracle data is generated with --variant dev
  add_test(NAME sr_curvy_roi_mutant_${_mut}_detected
    COMMAND sr_tests_image_target_curvy_roi_mutant_${_mut} --gtest_filter=CurvyRoiOracle.GatesAB)
  set_tests_properties(sr_curvy_roi_mutant_${_mut}_detected PROPERTIES
    LABELS "unit;mutation" TIMEOUT 300 ENVIRONMENT "SLAM_REFACT_CURVY_EXPECT_MUTANT=1")
endforeach()
# G-CROI-C on the (dev) cylinder sequence with a mirrored (MV) and a vertically flipped (MY) lift.
foreach(_mut MV MY)
  add_executable(sr_tests_image_target_curvy_synth_mutant_${_mut} image_target_curvy_synth_test.cc
                 "${CMAKE_CURRENT_BINARY_DIR}/mutants/roi_draw_curvy_${_mut}.cc")
  target_link_libraries(sr_tests_image_target_curvy_synth_mutant_${_mut}
    PRIVATE sr_engine sr_image_target Eigen3::Eigen GTest::gtest_main)
  sr_engine_warnings(sr_tests_image_target_curvy_synth_mutant_${_mut})
  target_compile_definitions(sr_tests_image_target_curvy_synth_mutant_${_mut} PRIVATE ${_sr_curvy_defs}
                             SR_CURVY_MUTANT_BUILD=1)
  add_test(NAME sr_curvy_synth_mutant_${_mut}_detected
    COMMAND sr_tests_image_target_curvy_synth_mutant_${_mut}
            --gtest_filter=Shapes/CurvySynth.RoiLocalSuccessRatio/cylinder)
  set_tests_properties(sr_curvy_synth_mutant_${_mut}_detected PROPERTIES
    LABELS "unit;mutation" FIXTURES_REQUIRED sr_it_curvy_dev TIMEOUT 900
    ENVIRONMENT "SLAM_REFACT_CURVY_EXPECT_MUTANT=1")
endforeach()

# US-015 image-target scale gate `it-scale-gate` (preregistration/it-scale-gate.v1.md): virtual feature photos of a planar target
# from an analytic camera orbit, with a scripted VIO extrinsic, against analytic ground truth; replaces the role of IT-P2.
# DEV and REGISTERED data are separate (as in the curvy gate): ItsDev.* use the dev scenario table only and run before the
# independent review; ItsGate.* use the registered table (its_gate_data.py --variant registered --seed-file <coordinator seed>)
# and refuse to start (HELD) unless the coordinator's designation, the review record, the pinned binary, the input tree
# sha256 and the one-run marker allow it (its_held.h). `ctest -L unit` never touches the registered data.
set(_sr_its_prereg "${SLAM_REFACT_REPO_ROOT}/build/slam-refact-full-20261019/preregistration")
set(_sr_its_dev_dir "${CMAKE_CURRENT_BINARY_DIR}/image-target-its-dev")
set(_sr_its_reg_dir "${CMAKE_CURRENT_BINARY_DIR}/image-target-its-registered")
add_test(NAME sr_its_render_dev
  COMMAND "${Python3_EXECUTABLE}" -I "${SLAM_REFACT_REPO_ROOT}/slam_refact/tools/image_target/its_gate_data.py"
          render --variant dev --out "${_sr_its_dev_dir}" --force)
set_tests_properties(sr_its_render_dev PROPERTIES FIXTURES_SETUP sr_its_dev LABELS unit)
# the registered table needs the coordinator seed file; without it this test fails as BLOCKED (label held only)
add_test(NAME sr_its_render_registered
  COMMAND "${Python3_EXECUTABLE}" -I "${SLAM_REFACT_REPO_ROOT}/slam_refact/tools/image_target/its_gate_data.py"
          render --variant registered --seed-file "${_sr_its_prereg}/it-scale-gate.v1.coordinator-seed.txt"
          --out "${_sr_its_reg_dir}" --force)
set_tests_properties(sr_its_render_registered PROPERTIES FIXTURES_SETUP sr_its_registered LABELS held)
set(_sr_its_defs
  "SR_ITS_DEV_DIR=\"${_sr_its_dev_dir}\""
  "SR_ITS_REG_DIR=\"${_sr_its_reg_dir}\""
  "SR_ITS_PREREG_DIR=\"${_sr_its_prereg}\"")
add_executable(sr_tests_image_target_scale image_target_scale_test.cc)
target_link_libraries(sr_tests_image_target_scale PRIVATE sr_engine sr_image_target sr_vio_tracker sr_frontend
                                                          Eigen3::Eigen GTest::gtest_main)
sr_engine_warnings(sr_tests_image_target_scale)
target_compile_definitions(sr_tests_image_target_scale PRIVATE ${_sr_its_defs})
if(CMAKE_VERSION VERSION_GREATER_EQUAL 3.22)
  gtest_discover_tests(sr_tests_image_target_scale TEST_FILTER "-ItsGate.*"
    PROPERTIES LABELS unit FIXTURES_REQUIRED sr_its_dev TIMEOUT 1200 DISCOVERY_TIMEOUT 60)
else()
  message(STATUS "slam_refact: CMake < 3.22: the unit tests of sr_tests_image_target_scale are not registered (no TEST_FILTER)")
endif()
add_test(NAME sr_its_gate COMMAND sr_tests_image_target_scale --gtest_filter=ItsGate.*)
set_tests_properties(sr_its_gate PROPERTIES LABELS held TIMEOUT 1800 FIXTURES_REQUIRED sr_its_registered)

# The HELD device itself: unit tests on synthetic files (every bypass rejected for the expected reason).
add_executable(sr_tests_image_target_its_held its_held_test.cc)
target_link_libraries(sr_tests_image_target_its_held PRIVATE GTest::gtest_main)
sr_engine_warnings(sr_tests_image_target_its_held)
target_compile_definitions(sr_tests_image_target_its_held PRIVATE
  "SR_ITS_PYTHON=\"${Python3_EXECUTABLE}\""
  "SR_ITS_HELD_TOOL=\"${SLAM_REFACT_REPO_ROOT}/slam_refact/tools/image_target/curvy_held_tool.py\"")
gtest_discover_tests(sr_tests_image_target_its_held PROPERTIES LABELS unit TIMEOUT 120 DISCOVERY_TIMEOUT 60)

# Mutation detection of the gate by CODE mutants (dev data only): copies of one engine / vendored source with one planted bug
# (the shared sources are not touched). The copy defines every symbol of the original object, so the linker does not pull the
# original out of its static library. The test binary is compiled with SR_ITS_MUTANT_BUILD and SR_ITS_EXPECT_FAIL: it runs the
# unmutated dev control and asserts that the listed gates FAIL.
#   IS1 computeScale returns d / n instead of n / d                      expected FAIL: S1 S2
#   IS2 computeScale baseline threshold 3e-2 -> 3e-4 (updates on slow motion)   expected FAIL: S0
#   IV1 applyToLocatedImages without the yaw offset (vio_tracker.cc)     expected FAIL: S2 S5
set(_sr_its_state_src "${SLAM_REFACT_THIRD_PARTY_DIR}/xr_source/reality/engine/imagedetection/image-tracking-state.cc")
set(_sr_its_vio_src "${CMAKE_CURRENT_SOURCE_DIR}/../../src/vio_tracker/vio_tracker.cc")
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS "${_sr_its_state_src}" "${_sr_its_vio_src}")
file(READ "${_sr_its_state_src}" _sr_its_state_text)
file(READ "${_sr_its_vio_src}" _sr_its_vio_text)
foreach(_mut IS1 IS2 IV1)
  if(_mut STREQUAL "IS1")
    string(REPLACE "return n > 3e-2 && d >= 1e-4 ? n / d : -1.0f;" "return n > 3e-2 && d >= 1e-4 ? d / n : -1.0f;"
           _sr_its_mutated "${_sr_its_state_text}")
    set(_sr_its_orig "${_sr_its_state_text}")
    set(_sr_its_expect "S1,S2")
  elseif(_mut STREQUAL "IS2")
    string(REPLACE "return n > 3e-2 && d >= 1e-4 ? n / d : -1.0f;" "return n > 3e-4 && d >= 1e-4 ? n / d : -1.0f;"
           _sr_its_mutated "${_sr_its_state_text}")
    set(_sr_its_orig "${_sr_its_state_text}")
    set(_sr_its_expect "S0")
  else()
    string(REPLACE "l.pose = metric ? MetricOutput(s.metric_transform, l.pose) : s.yaw_offset * l.pose;"
           "l.pose = metric ? MetricOutput(s.metric_transform, l.pose) : l.pose;"
           _sr_its_mutated "${_sr_its_vio_text}")
    set(_sr_its_orig "${_sr_its_vio_text}")
    set(_sr_its_expect "S2,S5")
  endif()
  if(_sr_its_mutated STREQUAL _sr_its_orig)
    message(FATAL_ERROR "ITS mutant ${_mut}: the source no longer contains the mutated text")
  endif()
  if(_mut STREQUAL "IV1")
    set(_sr_its_copy "${CMAKE_CURRENT_BINARY_DIR}/mutants/its_vio_tracker_${_mut}.cc")
  else()
    set(_sr_its_copy "${CMAKE_CURRENT_BINARY_DIR}/mutants/its_image_tracking_state_${_mut}.cc")
    # same compile options as sr_vendor_imagedetection (src/image_target/CMakeLists.txt)
    set_source_files_properties("${_sr_its_copy}" PROPERTIES COMPILE_OPTIONS "-w" COMPILE_DEFINITIONS "_USE_MATH_DEFINES")
  endif()
  if(NOT _mut STREQUAL "IV1")  # the inliner rule hash is a function of the file path; 0 skips the assert (rules2.h)
    string(REPLACE "cc_end(0x2791d6b5);" "cc_end(0);" _sr_its_mutated "${_sr_its_mutated}")
  endif()
  file(WRITE "${_sr_its_copy}" "${_sr_its_mutated}")
  add_executable(sr_tests_image_target_scale_mutant_${_mut} image_target_scale_test.cc "${_sr_its_copy}")
  target_link_libraries(sr_tests_image_target_scale_mutant_${_mut}
    PRIVATE sr_engine sr_image_target sr_vio_tracker sr_frontend Eigen3::Eigen GTest::gtest_main)
  sr_engine_warnings(sr_tests_image_target_scale_mutant_${_mut})
  target_compile_definitions(sr_tests_image_target_scale_mutant_${_mut} PRIVATE ${_sr_its_defs}
                             SR_ITS_MUTANT_BUILD=1 "SR_ITS_EXPECT_FAIL=\"${_sr_its_expect}\"")
  add_test(NAME sr_its_mutant_${_mut}_detected
    COMMAND sr_tests_image_target_scale_mutant_${_mut} --gtest_filter=ItsDev.CodeMutantIsDetected)
  set_tests_properties(sr_its_mutant_${_mut}_detected PROPERTIES
    LABELS "unit;mutation" FIXTURES_REQUIRED sr_its_dev TIMEOUT 600)
endforeach()

# RM-6 (review m6): desktop cost measurement of the image target path (run by hand, not a test).
add_executable(sr_bench_image_target image_target_bench.cc)
target_link_libraries(sr_bench_image_target PRIVATE sr_engine sr_image_target sr_frontend GTest::gtest)
sr_engine_warnings(sr_bench_image_target)

# OR-4 (full-parity plan Rev B): exact-input replay with / without a loaded image target (world SLAM
# regression gate, preregistration/or4-it-world-regression.v1.md). Development harness, run by hand and by
# slam_refact/tools/image_target/or4_world_regression.py; needs the replay bundle reader of the harness.
if(SLAM_REFACT_BUILD_HARNESS)
  add_executable(sr_or4_replay or4_replay_main.cc)
  target_link_libraries(sr_or4_replay PRIVATE sr_replay_io slam_refact::engine)
  sr_engine_warnings(sr_or4_replay)
endif()

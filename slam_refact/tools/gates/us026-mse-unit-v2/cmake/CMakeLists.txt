# us026-mse-unit-gate.v2: build wrapper. Builds the three v2 test executables WITHOUT editing any existing file under
# slam_refact/engine: the engine project is added as a subdirectory (its tests are configured but only the targets named
# below are built) and the v2 targets are defined here.
#
#   cmake -S slam_refact/tools/gates/us026-mse-unit-v2/cmake -B build/slam-refact-full-20261019/us026/mse-unit-v2-build -G Ninja \
#     -DCMAKE_BUILD_TYPE=Release -DFETCHCONTENT_FULLY_DISCONNECTED=ON -DSLAM_REFACT_BUILD_TESTS=ON -DSLAM_REFACT_BUILD_HARNESS=OFF \
#     -DSLAM_REFACT_CERES_SOURCE_DIR=<repo>/build/_deps/ceres-solver-src -DSLAM_REFACT_GTEST_SOURCE_DIR=<repo>/build/_deps/googletest-src \
#     -DSLAM_REFACT_EVIDENCE_ROOT=<repo>/build/slam-refact-20261015
#   cmake --build <dir> --target sr_tests_us026_method_precision_v2 sr_tests_us026_method_precision_v2_held \
#     sr_tests_us026_method_precision_v2_nowrap -j 2
#
# Targets (all compile us026_method_precision_v2_test.cc with the engine flags of SlamRefactFlags.cmake):
#   sr_tests_us026_method_precision_v2         label us026: dev / null-run / mutation binary (not held)
#   sr_tests_us026_method_precision_v2_held    SR_US026_HELD: the gate judge (fails closed until the shared device releases it)
#   sr_tests_us026_method_precision_v2_nowrap  mutation target only (M-SENS-NOWRAP): the same source linked WITHOUT the
#                                              ScaleSpatial interposition; the judge must report SENS-NOCAPTURE
# The interposition: -Wl,--wrap=<mangled slam_refact::metric_scale::ScaleSpatial>. The engine library is linked unchanged.
cmake_minimum_required(VERSION 3.21)
project(us026_mse_unit_v2 LANGUAGES C CXX)

get_filename_component(US026V2_ENGINE_DIR "${CMAKE_CURRENT_LIST_DIR}/../../../../engine" ABSOLUTE)
get_filename_component(US026V2_REPO_ROOT "${US026V2_ENGINE_DIR}/../.." ABSOLUTE)
set(US026V2_SS "_ZN11slam_refact12metric_scale12ScaleSpatialERKSt6vectorISt5arrayIfLm3EESaIS3_EES7_S7_S7_")

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "" FORCE)
endif()
set(SLAM_REFACT_BUILD_TESTS ON CACHE BOOL "" FORCE)

add_subdirectory("${US026V2_ENGINE_DIR}" engine)

function(us026v2_target name)
  cmake_parse_arguments(ARG "HELD;NOWRAP" "" "" ${ARGN})
  add_executable(${name} "${US026V2_ENGINE_DIR}/tests/metric_scale/us026_method_precision_v2_test.cc")
  target_link_libraries(${name} PRIVATE sr_metric_scale GTest::gtest_main)
  set_target_properties(${name} PROPERTIES CXX_STANDARD 17 CXX_STANDARD_REQUIRED ON CXX_EXTENSIONS OFF)
  sr_target_engine_flags(${name})
  sr_engine_warnings(${name})
  target_compile_definitions(${name} PRIVATE SR_TEST_DATA_DIR="${US026V2_ENGINE_DIR}/tests/data")
  if(ARG_HELD)
    target_compile_definitions(${name} PRIVATE SR_US026_HELD=1
      SR_US026_PREREG_DIR="${US026V2_REPO_ROOT}/build/slam-refact-full-20261019/preregistration")
  endif()
  if(ARG_NOWRAP)
    target_link_options(${name} PRIVATE "LINKER:--defsym=__real_${US026V2_SS}=${US026V2_SS}")
  else()
    target_link_options(${name} PRIVATE "LINKER:--wrap=${US026V2_SS}")
  endif()
endfunction()

us026v2_target(sr_tests_us026_method_precision_v2)
us026v2_target(sr_tests_us026_method_precision_v2_held HELD)
us026v2_target(sr_tests_us026_method_precision_v2_nowrap NOWRAP)

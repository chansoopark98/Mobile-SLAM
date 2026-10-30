# slam_refact engine: standalone CMake project (architecture.md §2.1, §2.3). The repository
# root CMakeLists.txt is not involved (it requires OpenCV/yaml-cpp and defines
# EIGEN_DONT_VECTORIZE globally).
#
#   (from slam_refact/engine; build/test presets resolve CMakePresets.json in the cwd)
#   cmake --preset slam-refact-native
#   cmake --build --preset slam-refact-native
#   ctest --preset slam-refact-native -L unit
cmake_minimum_required(VERSION 3.21)
project(slam_refact VERSION 0.1.0 LANGUAGES C CXX)

get_filename_component(SLAM_REFACT_REPO_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../.." ABSOLUTE)
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

# ---- Options (architecture.md §2.3) -------------------------------------------------------------
if(EMSCRIPTEN)
  set(_sr_default_host_only OFF)
else()
  set(_sr_default_host_only ON)
endif()
option(SLAM_REFACT_WERROR "-Werror on engine sources" ON)
option(SLAM_REFACT_BUILD_TESTS "Build GTest unit/evidence tests" ${_sr_default_host_only})
option(SLAM_REFACT_BUILD_HARNESS "Build the replay runner" ${_sr_default_host_only})
# US-025 loop close: decides only whether the agate descriptor and the loop close sources
# are compiled and linked into sr_map_builder. Native test builds default ON, WASM OFF; with OFF
# every existing target behaves as before (no macro, no extra objects).
option(SLAM_REFACT_LOOP_CLOSE "Compile the agate descriptor / loop close sources (US-025)"
  ${_sr_default_host_only})
option(SLAM_REFACT_ALLOW_MISSING_VENDOR
  "Configure without third_party/ (WP0a): only c8-free contract checks are built" OFF)
set(SLAM_REFACT_EVIDENCE_ROOT "${SLAM_REFACT_REPO_ROOT}/build/slam-refact-20261015"
  CACHE PATH "Evidence-tier inputs; evidence tests are registered only if this exists")
# Ceres release the engine links (SlamRefactDeps.cmake). 2.2 = 2.2.0, 2.0 = 2.0.0, the release
# the original xr-slam.wasm contains (string and Solver::Options layout evidence:
# build/slam-refact-20261016/ceres/; the 2b32b321 pin is indistinguishable from 2.0.0 there:
# build/slam-refact-20261016/ceres2/).
set(SLAM_REFACT_CERES_VERSION "2.2" CACHE STRING "Ceres release of the engine build: 2.2 or 2.0")
set_property(CACHE SLAM_REFACT_CERES_VERSION PROPERTY STRINGS 2.2 2.0)
# Optional exact upstream commit of a 2.0 source (empty = the 2.0.0 release tarball). The public
# reference runtime tree pins 2b32b321242b32f6364108694e0069f87e56326c (MODULE.bazel `ceres`, 2.0.0 + 17
# commits; tools/fetch_ceres_pin.sh). The source's VERSION or SLAM_REFACT_SOURCE file must name it.
set(SLAM_REFACT_CERES_COMMIT "" CACHE STRING
  "Ceres upstream commit of a 2.0 build (empty: 2.0.0 release tarball)")
string(LENGTH "${SLAM_REFACT_CERES_COMMIT}" _sr_ceres_commit_len)
if(SLAM_REFACT_CERES_COMMIT AND (NOT SLAM_REFACT_CERES_COMMIT MATCHES "^[0-9a-f]+$"
                                 OR NOT _sr_ceres_commit_len EQUAL 40))
  message(FATAL_ERROR "slam_refact: SLAM_REFACT_CERES_COMMIT='${SLAM_REFACT_CERES_COMMIT}' is "
                      "not a full lowercase hex commit id")
endif()
if(SLAM_REFACT_CERES_VERSION STREQUAL "2.0")
  if(SLAM_REFACT_CERES_COMMIT)
    string(SUBSTRING "${SLAM_REFACT_CERES_COMMIT}" 0 8 _sr_ceres_short)
  else()
    set(_sr_ceres_short "2.0.0")
  endif()
  set(_sr_ceres_src_default "${SLAM_REFACT_REPO_ROOT}/build/_deps/ceres-${_sr_ceres_short}-src")
  set(_sr_wasm_ceres_default
    "${SLAM_REFACT_REPO_ROOT}/build/slam-refact-wasm-ceres-${_sr_ceres_short}/install/lib/cmake/Ceres")
elseif(SLAM_REFACT_CERES_VERSION STREQUAL "2.2")
  if(SLAM_REFACT_CERES_COMMIT)
    message(FATAL_ERROR "slam_refact: SLAM_REFACT_CERES_COMMIT is only supported with "
                        "SLAM_REFACT_CERES_VERSION=2.0")
  endif()
  set(_sr_ceres_src_default "${SLAM_REFACT_REPO_ROOT}/build/_deps/ceres-solver-src")
  set(_sr_wasm_ceres_default "${SLAM_REFACT_REPO_ROOT}/wasm/libs/ceres/lib/cmake/Ceres")
else()
  message(FATAL_ERROR
    "slam_refact: SLAM_REFACT_CERES_VERSION='${SLAM_REFACT_CERES_VERSION}' (expected 2.2 or 2.0)")
endif()
set(SLAM_REFACT_CERES_SOURCE_DIR "${_sr_ceres_src_default}"
  CACHE PATH "Native Ceres source of SLAM_REFACT_CERES_VERSION (offline)")
set(SLAM_REFACT_GTEST_SOURCE_DIR "${SLAM_REFACT_REPO_ROOT}/build/_deps/googletest-src"
  CACHE PATH "GoogleTest 1.14.0 source (offline)")
set(SLAM_REFACT_WASM_CERES_DIR "${_sr_wasm_ceres_default}"
  CACHE PATH "WASM Ceres package config directory of SLAM_REFACT_CERES_VERSION")
set(SLAM_REFACT_THIRD_PARTY_DIR "${CMAKE_CURRENT_SOURCE_DIR}/third_party"
  CACHE PATH "Vendored sources (WP0a)")

if(EMSCRIPTEN AND SLAM_REFACT_BUILD_TESTS)
  message(FATAL_ERROR "slam_refact: GTest targets are native only; WASM tests run with node --test")
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "" FORCE)
endif()

include(SlamRefactFlags)
include(SlamRefactWarnings)
include(SlamRefactTargets)
include(SlamRefactDeps)
if(EMSCRIPTEN)
  include(SlamRefactWasm)
endif()

# ---- Dependencies -------------------------------------------------------------------------------
# Ceres and GoogleTest keep their own compile flags (NI-9); engine flags are per target.
if(EMSCRIPTEN)
  sr_find_wasm_ceres()
else()
  sr_add_native_ceres()
endif()

if(SLAM_REFACT_BUILD_TESTS)
  sr_add_gtest()
  enable_testing()
endif()

sr_add_vendor()  # sets SLAM_REFACT_HAVE_VENDOR

# ---- Engine -------------------------------------------------------------------------------------
# Include roots: public API (#include "slam_refact/...") and src/ (#include "common/...").
add_library(sr_includes INTERFACE)
target_include_directories(sr_includes INTERFACE
  "${CMAKE_CURRENT_SOURCE_DIR}/include"
  "${CMAKE_CURRENT_SOURCE_DIR}/src")

add_subdirectory(src)

if(SLAM_REFACT_BUILD_TESTS)
  add_subdirectory(tests)
endif()

# WP5-owned build parts, added when present.
if(EMSCRIPTEN AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/wasm/CMakeLists.txt"
   AND SLAM_REFACT_HAVE_VENDOR)
  add_subdirectory(wasm)
endif()
set(_sr_harness_dir "${SLAM_REFACT_REPO_ROOT}/slam_refact/tools/harness/replay")
if(SLAM_REFACT_BUILD_HARNESS AND EXISTS "${_sr_harness_dir}/CMakeLists.txt"
   AND SLAM_REFACT_HAVE_VENDOR)
  add_subdirectory("${_sr_harness_dir}" "${CMAKE_BINARY_DIR}/harness")
endif()

# Engine flags on every sr_* / slam_refact_* target, including WP0a's vendored libraries and
# targets added by later work packages (SlamRefactFlags.cmake).
sr_apply_engine_flags_to_tree()

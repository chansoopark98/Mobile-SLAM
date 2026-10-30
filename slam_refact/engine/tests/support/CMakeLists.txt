# sr_held: the shared HELD device (preregistration-protocol.md section 12.5). The header is header-only; this directory builds
# its conformance test against the case table shared with the node twin (tools/prereg/sr_held_cases.json).
#
# Standalone (no engine, no vendored code, no GTest, no network):
#   cmake -S slam_refact/engine/tests/support -B build/slam-refact-full-c1-held -G Ninja
#   cmake --build build/slam-refact-full-c1-held --parallel 2
#   ctest --test-dir build/slam-refact-full-c1-held --output-on-failure
# Inside the engine build this file would be added with add_subdirectory(support) (patches/sr-held-cmake.patch).

if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
  cmake_minimum_required(VERSION 3.21)
  project(sr_held_support LANGUAGES CXX)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
  if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
  endif()
  enable_testing()
endif()

set(_sr_held_cases "${CMAKE_CURRENT_SOURCE_DIR}/../../../tools/prereg/sr_held_cases.json")

add_executable(sr_held_conformance sr_held_conformance.cc)
target_compile_options(sr_held_conformance PRIVATE -Wall -Wextra)

add_test(NAME sr_held_conformance COMMAND sr_held_conformance "${_sr_held_cases}")
set_tests_properties(sr_held_conformance PROPERTIES
  LABELS unit
  ENVIRONMENT "SR_HELD_TMP=${CMAKE_CURRENT_BINARY_DIR}/work"
  FAIL_REGULAR_EXPRESSION "CASE [^\n]* FAIL;FATAL")

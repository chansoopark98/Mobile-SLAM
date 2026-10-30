# Target helpers for engine modules and tests (architecture.md §2.3).
#
# Module targets glob their directory, so a work package only adds .cc files: with no .cc the
# module is an INTERFACE (header-only) target, with sources it becomes a STATIC library. Globs use
# CONFIGURE_DEPENDS, so a new .cc re-runs the configure step automatically.

include_guard(GLOBAL)

# sr_add_module(<target> DIR <dir> DEPS <targets...>)
function(sr_add_module target)
  cmake_parse_arguments(ARG "" "DIR" "DEPS" ${ARGN})
  if(NOT ARG_DIR)
    set(ARG_DIR "${CMAKE_CURRENT_SOURCE_DIR}")
  endif()
  file(GLOB _srcs CONFIGURE_DEPENDS "${ARG_DIR}/*.cc")
  list(SORT _srcs)
  if(_srcs)
    add_library(${target} STATIC ${_srcs})
    target_link_libraries(${target} PUBLIC ${ARG_DEPS})
    sr_engine_warnings(${target})
  else()
    add_library(${target} INTERFACE)
    target_link_libraries(${target} INTERFACE ${ARG_DEPS})
    message(STATUS "slam_refact: ${target}: no .cc in ${ARG_DIR} yet (header-only target)")
  endif()
endfunction()

# sr_add_unit_test(<name> SOURCES <files...> LIBS <targets...> [LABELS <labels...>])
# GTest executable registered through gtest_discover_tests. Default label: unit.
function(sr_add_unit_test name)
  cmake_parse_arguments(ARG "" "" "SOURCES;LIBS;LABELS" ${ARGN})
  if(NOT ARG_LABELS)
    set(ARG_LABELS unit)
  endif()
  add_executable(${name} ${ARG_SOURCES})
  target_link_libraries(${name} PRIVATE ${ARG_LIBS} GTest::gtest_main)
  sr_engine_warnings(${name})
  include(GoogleTest)
  gtest_discover_tests(${name}
    PROPERTIES LABELS "${ARG_LABELS}"
    DISCOVERY_TIMEOUT 60)
endfunction()

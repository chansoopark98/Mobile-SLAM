# Contract tests and header self-containment (architecture.md §8.2 WP0b).

# sr_header_selfcontained_check(<target> <headers...>): one generated TU per header, each
# including its header twice (self-containment + include guard). Built as part of `all`, so a
# header that does not compile on its own fails the build.
function(sr_header_selfcontained_check target)
  set(_tus)
  foreach(hdr IN LISTS ARGN)
    file(RELATIVE_PATH _rel "${PROJECT_SOURCE_DIR}" "${hdr}")
    string(MAKE_C_IDENTIFIER "${_rel}" _id)
    if(_rel MATCHES "^include/(.*)$")
      set(_inc "${CMAKE_MATCH_1}")
    elseif(_rel MATCHES "^src/(.*)$")
      set(_inc "${CMAKE_MATCH_1}")
    else()
      message(FATAL_ERROR "unexpected header location: ${_rel}")
    endif()
    set(_tu "${CMAKE_CURRENT_BINARY_DIR}/${target}/${_id}.cc")
    file(CONFIGURE OUTPUT "${_tu}" CONTENT
      "// Generated: self-containment check of ${_rel}\n#include \"${_inc}\"\n#include \"${_inc}\"\n")
    list(APPEND _tus "${_tu}")
  endforeach()
  add_library(${target} OBJECT ${_tus})
  sr_engine_warnings(${target})
endfunction()

# ---- c8-free public API (always) ----
file(GLOB _sr_public_headers CONFIGURE_DEPENDS "${PROJECT_SOURCE_DIR}/include/slam_refact/*.h")
list(SORT _sr_public_headers)
sr_header_selfcontained_check(sr_header_selfcontained_public ${_sr_public_headers})
target_link_libraries(sr_header_selfcontained_public PRIVATE sr_includes)

sr_add_unit_test(sr_contract_public_test
  SOURCES contract_public_test.cc
  LIBS sr_includes)

if(NOT SLAM_REFACT_HAVE_VENDOR)
  return()
endif()

# ---- Engine headers (need the vendored c8 / reality headers) ----
# Every header under src/ (frozen and module-private alike) must compile on its own.
file(GLOB_RECURSE _sr_engine_headers CONFIGURE_DEPENDS "${PROJECT_SOURCE_DIR}/src/*.h")
list(SORT _sr_engine_headers)
sr_header_selfcontained_check(sr_header_selfcontained_engine ${_sr_engine_headers})
target_link_libraries(sr_header_selfcontained_engine PRIVATE
  sr_common sr_compat sr_config sr_vendor_c8 sr_vendor_features sr_vendor_geometry
  sr_libcxx_compat Ceres::ceres Eigen3::Eigen)

sr_add_unit_test(sr_contract_internal_test
  SOURCES contract_internal_test.cc
  LIBS sr_common sr_compat sr_vendor_features sr_vendor_c8)

# Engine source tree (architecture.md §2.1, §3.0). Module order follows the dependency graph:
#   compat - common - config - frontend - map - frame_frame_tracker - map_tracker - map_builder
#   - metric_scale - vio_tracker - engine

# c8-free configuration tables (always available; used by the config defaults check).
add_library(sr_config_defaults INTERFACE)
target_link_libraries(sr_config_defaults INTERFACE sr_includes)

if(NOT SLAM_REFACT_HAVE_VENDOR)
  message(STATUS "slam_refact: vendored targets absent; engine modules are not configured")
  return()
endif()

sr_add_module(sr_common
  DIR "${CMAKE_CURRENT_SOURCE_DIR}/common"
  DEPS sr_includes sr_vendor_features sr_vendor_c8 Eigen3::Eigen)
sr_add_module(sr_compat
  DIR "${CMAKE_CURRENT_SOURCE_DIR}/compat"
  DEPS sr_common sr_libcxx_compat sr_vendor_c8)

add_subdirectory(config)
add_subdirectory(frontend)
add_subdirectory(map)
add_subdirectory(frame_frame_tracker)
add_subdirectory(map_tracker)
add_subdirectory(map_builder)
add_subdirectory(metric_scale)
add_subdirectory(vio_tracker)
add_subdirectory(engine)
add_subdirectory(image_target)
add_subdirectory(ml)
add_subdirectory(face)
add_subdirectory(sky)
add_subdirectory(hand)

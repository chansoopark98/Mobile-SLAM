# VioTracker module (architecture.md §3.7). Owned by WP4.
# sr_vio_tracker_core: 784 orchestration and its pure parts, written against the TrackerModules
# seam only (no MapBuilder link dependency; MapTracker only for the shared YawOf 519), so the
# unit tests can inject doubles.
# sr_vio_tracker: core + the product modules (default_tracker_modules.cc, public constructor).
add_library(sr_vio_tracker_core STATIC
  ${CMAKE_CURRENT_SOURCE_DIR}/motion.cc
  ${CMAKE_CURRENT_SOURCE_DIR}/vio_tracker.cc
  ${CMAKE_CURRENT_SOURCE_DIR}/vi_prediction.cc)
# sr_vendor_imagedetection: DetectionImageTracker of 938 (FULL-3, src/image_target/CMakeLists.txt).
# sr_image_target: the US-015 image world tracker (994) behind the vendored hook (FULL-12, opt-in).
target_link_libraries(sr_vio_tracker_core PUBLIC sr_frontend sr_metric_scale sr_map_tracker
                                          sr_vendor_imagedetection sr_image_target)
sr_engine_warnings(sr_vio_tracker_core)

add_library(sr_vio_tracker STATIC ${CMAKE_CURRENT_SOURCE_DIR}/default_tracker_modules.cc)
target_link_libraries(sr_vio_tracker PUBLIC sr_vio_tracker_core sr_frontend sr_map_tracker sr_map_builder)
sr_engine_warnings(sr_vio_tracker)

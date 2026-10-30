# Engine tests (architecture.md §7). Labels: unit (always; must pass on a fresh clone),
# evidence (registered only when SLAM_REFACT_EVIDENCE_ROOT exists), integration (evidence).
# tests/<module>/ directories are owned by the module's work package (§8.2); they are added here
# when they contain a CMakeLists.txt.

if(EXISTS "${SLAM_REFACT_EVIDENCE_ROOT}")
  set(SLAM_REFACT_HAVE_EVIDENCE ON)
else()
  set(SLAM_REFACT_HAVE_EVIDENCE OFF)
  message(STATUS
    "slam_refact: SLAM_REFACT_EVIDENCE_ROOT '${SLAM_REFACT_EVIDENCE_ROOT}' not found: evidence "
    "tests are NOT registered (report as BLOCKED, not as skipped)")
endif()
set(SLAM_REFACT_TEST_DATA_DIR "${CMAKE_CURRENT_SOURCE_DIR}/data")

add_subdirectory(common)
add_subdirectory(config)
# sr_held: the shared HELD device and its conformance test (docs/design/sr-held-design.md); header-only, no vendored code needed
add_subdirectory(support)

if(SLAM_REFACT_HAVE_VENDOR)
  foreach(dir vendor compat frontend map map_tracker frame_frame_tracker map_builder
              vio_tracker metric_scale engine)
    if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/${dir}/CMakeLists.txt")
      add_subdirectory(${dir})
    endif()
  endforeach()
  add_subdirectory(image_target)
  add_subdirectory(ml)
  add_subdirectory(face)
  add_subdirectory(sky)
  add_subdirectory(hand)
endif()

# WP1 frontend tests (architecture.md §6.6.1, §7.3).
#
# zlib is a test-only dependency: it reads the gzip golden captures (tests/data/frontend and the
# evidence tree). The engine libraries do not link it.
find_package(ZLIB REQUIRED)

add_library(sr_frontend_test_util STATIC test_util.cc gate_util.cc)
target_link_libraries(sr_frontend_test_util PUBLIC sr_frontend ZLIB::ZLIB)
target_include_directories(sr_frontend_test_util PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")
target_compile_definitions(sr_frontend_test_util PRIVATE
  SR_TEST_DATA_DIR="${SLAM_REFACT_TEST_DATA_DIR}"
  SR_EVIDENCE_ROOT="${SLAM_REFACT_EVIDENCE_ROOT}")
sr_engine_warnings(sr_frontend_test_util)

# Unit gates on checked-in data: F-LAY, F-FAST, F-H9 (tables), F-K, F-H8 (excerpt), F-H2/F-H3/
# F-H4/F-H7 (TUM cap-001..003), D-14 guards, sensor intake, planar ROI draw golden (US015-CC-1).
sr_add_unit_test(sr_tests_frontend
  SOURCES layout_shader_test.cc sensor_intrinsics_test.cc pyramid_detect_test.cc roi_draw_golden_test.cc
  LIBS sr_frontend_test_util sr_config
  LABELS unit)

# Evidence gates on the full captures (46 frames) and the original wasm.
if(SLAM_REFACT_HAVE_EVIDENCE)
  sr_add_unit_test(sr_tests_frontend_evidence
    SOURCES frontend_evidence_test.cc
    LIBS sr_frontend_test_util
    LABELS evidence)
endif()

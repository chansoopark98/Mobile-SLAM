# Config tests (architecture.md §7.3 config, §8.2 WP0b "config 211 key 대조").

# c8-free table / struct invariants and spec spot checks.
sr_add_unit_test(sr_config_defaults_test
  SOURCES config_defaults_test.cc
  LIBS sr_config_defaults)

# Compiled defaults vs the recovered binary defaults (config-defaults.json).
add_executable(sr_config_dump config_dump.cc)
target_link_libraries(sr_config_dump PRIVATE sr_config_defaults)
sr_engine_warnings(sr_config_dump)

find_package(Python3 3.8 COMPONENTS Interpreter)
set(_json_unit "${SLAM_REFACT_TEST_DATA_DIR}/config/config-defaults.json")
set(_json_evidence "${SLAM_REFACT_REPO_ROOT}/build/rea-analysis-20261015/xr_reference/config-defaults.json")
set(_inc "${PROJECT_SOURCE_DIR}/src/config/config_defaults.inc")
if(NOT Python3_Interpreter_FOUND)
  message(STATUS "slam_refact: python3 not found: config defaults check NOT registered (BLOCKED)")
elseif(EXISTS "${_json_unit}")
  add_test(NAME sr_config_defaults_vs_binary
    COMMAND Python3::Interpreter -I "${CMAKE_CURRENT_SOURCE_DIR}/check_config_defaults.py"
            --json "${_json_unit}" --inc "${_inc}" --dump $<TARGET_FILE:sr_config_dump>)
  set_tests_properties(sr_config_defaults_vs_binary PROPERTIES LABELS unit)
elseif(SLAM_REFACT_HAVE_EVIDENCE AND EXISTS "${_json_evidence}")
  message(STATUS
    "slam_refact: tests/data/config/config-defaults.json (WP0a) missing: config defaults check "
    "registered against the evidence copy with label 'evidence'")
  add_test(NAME sr_config_defaults_vs_binary
    COMMAND Python3::Interpreter -I "${CMAKE_CURRENT_SOURCE_DIR}/check_config_defaults.py"
            --json "${_json_evidence}" --inc "${_inc}" --dump $<TARGET_FILE:sr_config_dump>)
  set_tests_properties(sr_config_defaults_vs_binary PROPERTIES LABELS evidence)
else()
  message(STATUS "slam_refact: config-defaults.json not found: config defaults check NOT registered (BLOCKED)")
endif()

if(NOT SLAM_REFACT_HAVE_VENDOR)
  return()
endif()

# Runtime behavior of globalParams()-backed settings. Separate executable: it mutates
# process-global state (§4.6).
sr_add_unit_test(sr_config_params_test
  SOURCES params_test.cc
  LIBS sr_config)

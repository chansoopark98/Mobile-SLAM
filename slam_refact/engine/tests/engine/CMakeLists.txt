# Engine core + glue tests (architecture.md §7.3 engine/glue, §7.4 IT-3).
# Checked-in data only (tests/data), so both executables are label `unit`. The TUM capture loader
# (zlib, test-only) comes from the frontend test utility.

# GlueRunner with a fixed-output TrackerCore (the injection seam, no engine modules exercised).
sr_add_unit_test(sr_tests_glue_runner
  SOURCES glue_runner_test.cc
  LIBS sr_engine
  LABELS unit)

# Real Engine: D-14 guards, R16, determinism / RNG reseed, partial reset, US-010 API (IT-3).
# Separate executable because the engine keeps process-global state (§4.6).
sr_add_unit_test(sr_tests_engine
  SOURCES engine_test.cc
  LIBS sr_engine sr_frontend_test_util
  LABELS unit)

# Accuracy-mode modules without the tracking core (contract ACC-2 metric scale, ACC-3 surfaces).
sr_add_unit_test(sr_tests_accuracy
  SOURCES accuracy_test.cc
  LIBS sr_engine
  LABELS unit)

# XrController API (contract FULL-1, spec 06): display intrinsics, hit test, configure, lighting,
# tracking status events (scripted core; checked-in constants only).
sr_add_unit_test(sr_tests_xr_api
  SOURCES xr_api_test.cc
  LIBS sr_engine
  LABELS unit)

# XrController API against the original SDK (spec 06 §1.3, §2.3, §4.3): TSV extracts of the
# reference runs and of the api-oracle runs under build/slam-refact-full-20261019/oracle.
# When the evidence root exists but the extracts do not, a failing test is registered instead
# (RM-3a: a missing oracle must not silently remove the evidence tests from the list).
set(_sr_full_oracle "${SLAM_REFACT_REPO_ROOT}/build/slam-refact-full-20261019/oracle")
if(SLAM_REFACT_HAVE_EVIDENCE)
  if(EXISTS "${_sr_full_oracle}/tsv-api1/hits.tsv" AND EXISTS "${_sr_full_oracle}/tsv-api3/lighting.tsv")
    sr_add_unit_test(sr_tests_xr_api_evidence
      SOURCES xr_api_evidence_test.cc
      LIBS sr_engine
      LABELS evidence)
    target_compile_definitions(sr_tests_xr_api_evidence PRIVATE
      SR_FULL_ORACLE_DIR="${_sr_full_oracle}" SR_REPO_ROOT="${SLAM_REFACT_REPO_ROOT}")
  else()
    message(WARNING "slam_refact: XrController API oracle extracts missing under ${_sr_full_oracle}: "
                    "registering a FAILING test sr_tests_xr_api_evidence_oracle_missing")
    add_test(NAME sr_tests_xr_api_evidence_oracle_missing
      COMMAND ${CMAKE_COMMAND} -DORACLE_DIR=${_sr_full_oracle}
              -P ${CMAKE_CURRENT_SOURCE_DIR}/fail_missing_oracle.cmake)
    set_tests_properties(sr_tests_xr_api_evidence_oracle_missing PROPERTIES LABELS evidence)
  endif()
endif()

# XrController hitTest A/B oracle (contract FULL-7, RM-3b, review M2; preregistered in
# build/slam-refact-full-20261019/preregistration/rm3b-hit-ab-gate.v2.md, which supersedes v1). TSV extracts of the A/B
# recordings (slam_refact/tools/harness/oracle/extract_hit_ab.py) under oracle-ab:
#   sr_tests_xr_api_hit_ab        label evidence: instrument null-runs and mutation detection on a synthetic
#                                 recording (no original data), original-side properties of the recording
#   sr_tests_xr_api_hit_ab_held   label held (RM-3c: was evidence-held, which `ctest -L evidence` selects as
#                                 a regex): the gate itself (engine replay vs the original
#                                 recording). Fails closed (HELD) until the manifest records the
#                                 coordinator-designated delta review and SLAM_REFACT_RM3B_REVIEWER equals the
#                                 sha256 of the registered designation file (shared device tests/support/sr_held.h,
#                                 called by the test's StartRelease). The gate id and version are designated at run
#                                 time (SLAM_REFACT_RM3B_GATE=<gate_id>.v<N>); v3 supersedes the v2 named above.
set(_sr_full_oracle_ab "${SLAM_REFACT_REPO_ROOT}/build/slam-refact-full-20261019/oracle-ab")
if(SLAM_REFACT_HAVE_EVIDENCE)
  if(EXISTS "${_sr_full_oracle_ab}/tsv-hab1/hit_ab.tsv" AND EXISTS "${_sr_full_oracle_ab}/tsv-hab2/hit_ab.tsv"
     AND EXISTS "${_sr_full_oracle_ab}/tsv-hab3/hit_ab.tsv")
    sr_add_unit_test(sr_tests_xr_api_hit_ab
      SOURCES xr_api_hit_ab_test.cc
      LIBS sr_engine
      LABELS evidence)
    target_compile_definitions(sr_tests_xr_api_hit_ab PRIVATE SR_FULL_ORACLE_AB_DIR="${_sr_full_oracle_ab}")
    sr_add_unit_test(sr_tests_xr_api_hit_ab_held
      SOURCES xr_api_hit_ab_test.cc
      LIBS sr_engine
      LABELS held)
    target_compile_definitions(sr_tests_xr_api_hit_ab_held PRIVATE SR_FULL_ORACLE_AB_DIR="${_sr_full_oracle_ab}"
                                                                   SR_HITAB_HELD=1)
  else()
    message(WARNING "slam_refact: hit A/B oracle extracts missing under ${_sr_full_oracle_ab}: registering a "
                    "FAILING test sr_tests_xr_api_hit_ab_oracle_missing")
    add_test(NAME sr_tests_xr_api_hit_ab_oracle_missing
      COMMAND ${CMAKE_COMMAND} -DORACLE_DIR=${_sr_full_oracle_ab}
              -P ${CMAKE_CURRENT_SOURCE_DIR}/fail_missing_oracle.cmake)
    set_tests_properties(sr_tests_xr_api_hit_ab_oracle_missing PROPERTIES LABELS evidence)
  endif()
endif()

# Device orientation (contract FULL-8, US-023): landscape captures and rotation 0 / 90 / 180 / 270,
# projection reorder, pose correction, hit input turn (independent expectations, checked-in constants only).
# Separate executable: the real-Engine cases keep process-global state (§4.6).
sr_add_unit_test(sr_tests_orientation
  SOURCES orientation_test.cc
  LIBS sr_engine
  LABELS unit)

# US-023 / US-024 orientation and iOS gates against recordings of the original SDK (preregistered in
# build/slam-refact-full-20261019/preregistration/orient-us02{3,4}-gate.v<N>.md (v1 rejected by the pre-run review); tools/re/runtime/orient records them):
#   sr_tests_orient_gate        label unit: comparator null-runs and mutation detection on a synthetic recording (no
#                               original data), and the order of the held-run start checks on synthetic files;
#                               SLAM_REFACT_ORIENT_REPORT_DIR=<exploration recording> prints a REPORT
#   sr_tests_orient_gate_held   label held: the gates (engine vs the registered recordings). Fails closed (HELD) until the
#                               manifests record the coordinator-designated review and SLAM_REFACT_ORIENT_REVIEWER equals the
#                               sha256 of the registered designation file (shared device tests/support/sr_held.h, protocol
#                               §12.5). The gate id and version are designated at run time (SLAM_REFACT_ORIENT_GATE=<gate_id>.v<N>);
#                               the directories below are defaults that SLAM_REFACT_ORIENT_PREREG_DIR / _REC_DIR override.
sr_add_unit_test(sr_tests_orient_gate
  SOURCES orient_gate_test.cc
  LIBS sr_engine sr_frontend_test_util
  LABELS unit)
if(SLAM_REFACT_HAVE_EVIDENCE)
  sr_add_unit_test(sr_tests_orient_gate_held
    SOURCES orient_gate_test.cc
    LIBS sr_engine sr_frontend_test_util
    LABELS held)
  target_compile_definitions(sr_tests_orient_gate_held PRIVATE
    SR_ORIENT_HELD=1
    SR_ORIENT_PREREG_DIR="${SLAM_REFACT_REPO_ROOT}/build/slam-refact-full-20261019/preregistration"
    SR_ORIENT_REC_DIR="${SLAM_REFACT_REPO_ROOT}/build/slam-refact-full-20261019/orient/original/flip6-s2"
    SR_ORIENT_US024_SENSOR_DIRS="v2-a-ios-12promax","v2-a-ios-14pro-nz","v2-a-android-pixel7","v2-a-android-reduced-nz"
    SR_ORIENT_US024_K_DIRS="v2-k-ios-se1","v2-k-ios-8","v2-k-ios-xr","v2-k-ios-12","v2-k-ios-12promax","v2-k-ios-14pro","v2-k-ios-15plus"
    SR_ORIENT_US023_SENSOR_DIRS="v2-b-s90","v2-b-s180","v2-b-s270"
    SR_ORIENT_US023_K_DIRS="v2-b-o0","v2-b-o90","v2-b-o180","v2-b-o270","v2-b-px90","v2-b-px270"
    SR_ORIENT_US023_PYRAMID_DIRS="v2-b-o0","v2-b-o90","v2-b-o180","v2-b-o270"
    SR_ORIENT_US023_POSE_DIRS="v2-b-o0","v2-b-o90","v2-b-o180","v2-b-o270")
endif()

# Exact-input replay of a recording of the original (US-023 REPORT; not a test): tests/engine/orient_replay_main.cc.
add_executable(slam_refact_orient_replay orient_replay_main.cc)
target_link_libraries(slam_refact_orient_replay PRIVATE sr_engine sr_frontend_test_util)
sr_engine_warnings(slam_refact_orient_replay)

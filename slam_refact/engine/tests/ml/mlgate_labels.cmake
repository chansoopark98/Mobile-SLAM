# Read by ctest (an include file of CTestTestfile.cmake) after the tests of sr_tests_ml_models were
# discovered. RM-1c gave every test of that binary the label `mlgate`; RM-1d (review RM-1c MAJOR-1)
# fixes the gate to the NAME LIST data/mlgate_names.txt (the 17 tests of the regression gate v3) so
# that a missing, stale or extended binary cannot pass quietly:
#   * only the tests whose name is on the list get `mlgate` (an extra test of the binary gets
#     `mlmodel` only and is reported);
#   * binary not built (discovery did not run, ctest lists *_NOT_BUILT): ONE failing test is
#     registered under `mlmodel;mlgate`;
#   * a listed name is missing, or the binary has a test that is not listed: ONE failing test
#     (sr_ml_gate_names_mismatch) under `mlmodel;mlgate` says which.
# Variables set by the generated include file: SR_ML_GATE_NAMES_FILE, SR_ML_GATE_FAIL_SCRIPT,
# SR_ML_GATE_CMAKE (path of cmake), SR_ML_GATE_TESTS_DEFINED, SR_ML_GATE_TESTS (the discovered names).
file(STRINGS "${SR_ML_GATE_NAMES_FILE}" _sr_gate_expected)
set(_sr_gate_extra "")
set(_sr_gate_missing "")
if(NOT SR_ML_GATE_TESTS_DEFINED)
  set(_sr_gate_missing "sr_tests_ml_models was not built, so none of the gate tests were discovered")
else()
  foreach(_t IN LISTS SR_ML_GATE_TESTS)
    list(FIND _sr_gate_expected "${_t}" _i)
    if(_i LESS 0)
      list(APPEND _sr_gate_extra "${_t}")
      set_tests_properties("${_t}" PROPERTIES LABELS "mlmodel")
    else()
      set_tests_properties("${_t}" PROPERTIES LABELS "mlmodel;mlgate")
    endif()
  endforeach()
  set(_sr_gate_missing_names "")
  foreach(_e IN LISTS _sr_gate_expected)
    list(FIND SR_ML_GATE_TESTS "${_e}" _i)
    if(_i LESS 0)
      list(APPEND _sr_gate_missing_names "${_e}")
    endif()
  endforeach()
  if(_sr_gate_missing_names)
    list(LENGTH _sr_gate_missing_names _n)
    list(GET _sr_gate_missing_names 0 _first)
    set(_sr_gate_missing "${_n} registered gate test(s) are not in the binary (stale or partial build), first: ${_first}.")
  endif()
endif()
if(_sr_gate_extra)
  list(LENGTH _sr_gate_extra _n)
  list(GET _sr_gate_extra 0 _first)
  string(APPEND _sr_gate_missing " ${_n} test(s) of the binary are not registered gate tests, first: ${_first}.")
endif()
if(_sr_gate_missing)
  add_test(sr_ml_gate_names_mismatch "${SR_ML_GATE_CMAKE}" "-DWHAT=${_sr_gate_missing}" -P "${SR_ML_GATE_FAIL_SCRIPT}")
  set_tests_properties(sr_ml_gate_names_mismatch PROPERTIES LABELS "mlmodel;mlgate")
endif()

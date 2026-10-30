# Run by ctest (cmake -P) when the registered `mlgate` tests are not the 17 of the regression gate
# v3 (data/mlgate_names.txt): the test binary was not built (ctest then lists only a *_NOT_BUILT
# placeholder), is stale (fewer or other tests than the registered ones), or got an extra test.
# `ctest -L mlgate` must FAIL in each case, not pass on a subset (RM-1d, review RM-1c MAJOR-1).
# WHAT says what differs.
message(FATAL_ERROR
  "BLOCKED: the mlgate test set differs from the 17 registered gate tests "
  "(slam_refact/engine/tests/ml/data/mlgate_names.txt): ${WHAT} "
  "Rebuild sr_tests_ml_models (`cmake --build <dir> --target sr_tests_ml_models`) in this build "
  "directory, or update data/mlgate_names.txt together with a new preregistration if the gate "
  "tests changed on purpose.")

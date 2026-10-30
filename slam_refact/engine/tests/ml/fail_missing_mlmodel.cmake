# Run by ctest (cmake -P) when the reference assets of the `mlmodel` tests (model files, python3,
# numpy) are missing: the label must not pass silently, and `ctest -L mlmodel` must FAIL even
# without --no-tests=error (RM-1c, review RM-1b/RM-5 MAJOR-3). WHAT names what is missing.
message(FATAL_ERROR
  "BLOCKED: the mlmodel tests are not registered because the reference assets are missing: ${WHAT} "
  "Provide them (models under assets/references/xr_reference, a python3 with numpy), or configure with "
  "-DSLAM_REFACT_ALLOW_BLOCKED_MLMODEL=ON to stop registering this failing test (then run "
  "`ctest --no-tests=error -L mlmodel` so that an empty label cannot pass). A missing asset is a "
  "failure, not a skip.")

# Shared by tests/ml and tests/face (RM-1c). The `mlmodel` tests need reference assets that are
# gitignored and never copied into the repository (model files; python3 with numpy for tests/ml).
#
# Policy (coordinator decision, review RM-1b/RM-5 MAJOR-3 and RM-7 J1):
#   * configure always succeeds, so unit tests build on a fresh clone;
#   * when an asset is missing, ONE test that always fails is registered under the `mlmodel` label
#     (message: what is missing), so `ctest -L mlmodel` fails without --no-tests=error;
#   * -DSLAM_REFACT_ALLOW_BLOCKED_MLMODEL=ON only stops registering that failing test (an empty
#     label then passes unless ctest is run with --no-tests=error), and only when NOTHING of that
#     asset group is there (a fresh clone). RM-1d (review RM-1c MINOR-1): a group with some of its
#     assets present but not all (models present, numpy missing; one of seven models missing) is a
#     broken checkout, not a fresh clone: the failing test is registered even with ALLOW=ON
#     (sr_ml_report_blocked ... PARTIAL), because otherwise the other group's tests (face, 5/5) made
#     `ctest --no-tests=error -L mlmodel` pass while the 17 ML gate tests were gone.
#   * The regression gate is judged with `-L '^mlgate$'` and the 17 names of data/mlgate_names.txt
#     (mlgate_labels.cmake), never with the count of a label.
option(SLAM_REFACT_ALLOW_BLOCKED_MLMODEL
  "Do not register the always-failing test that reports missing mlmodel reference assets" OFF)
set(_sr_ml_assets_dir "${CMAKE_CURRENT_LIST_DIR}")

# sr_ml_report_blocked(<test name> <labels> <what is missing> [PARTIAL]): registers the failing test.
# PARTIAL: some assets of the group exist, so SLAM_REFACT_ALLOW_BLOCKED_MLMODEL does not apply.
function(sr_ml_report_blocked name labels what)
  set(_partial FALSE)
  if("${ARGV3}" STREQUAL "PARTIAL")
    set(_partial TRUE)
  endif()
  if(SLAM_REFACT_ALLOW_BLOCKED_MLMODEL AND NOT _partial)
    message(STATUS
      "slam_refact: ${name} NOT registered (BLOCKED, failing test suppressed by "
      "SLAM_REFACT_ALLOW_BLOCKED_MLMODEL): ${what}")
    return()
  endif()
  if(_partial AND SLAM_REFACT_ALLOW_BLOCKED_MLMODEL)
    message(WARNING
      "slam_refact: ${name}: the assets are only PARTLY there (${what}); "
      "SLAM_REFACT_ALLOW_BLOCKED_MLMODEL does not suppress the failing test for an incomplete set.")
  else()
    message(WARNING
      "slam_refact: mlmodel reference assets are missing (${what}). Registering the FAILING test "
      "${name}; configure with -DSLAM_REFACT_ALLOW_BLOCKED_MLMODEL=ON to suppress it.")
  endif()
  add_test(NAME ${name}
    COMMAND ${CMAKE_COMMAND} "-DWHAT=${what}" -P "${_sr_ml_assets_dir}/fail_missing_mlmodel.cmake")
  set_tests_properties(${name} PROPERTIES LABELS "${labels}")
endfunction()

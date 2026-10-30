# WP0a compat tests (architecture.md §5.3, §7.3). Added by tests/CMakeLists.txt (WP0b) with
# add_subdirectory(compat) when SLAM_REFACT_HAVE_VENDOR is ON; needs GTest::gtest_main.
# The golden file is produced by tools/gen_libcxx_golden.sh (em++ 5.0.0), not by this build.
include(GoogleTest)

add_executable(sr_tests_compat libcxx_golden_test.cc)
target_link_libraries(sr_tests_compat PRIVATE sr_libcxx_compat GTest::gtest_main)
target_compile_definitions(sr_tests_compat PRIVATE SR_TEST_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/../data")
target_compile_options(sr_tests_compat PRIVATE -fno-exceptions -fno-threadsafe-statics -ffp-contract=off -fno-fast-math)
gtest_discover_tests(sr_tests_compat PROPERTIES LABELS unit DISCOVERY_TIMEOUT 60)

# third_party/musl_libm (host builds only; em++ links the same musl code itself).
if(TARGET sr_musl_libm)
  add_executable(sr_tests_musl_libm musl_libm_test.cc)
  target_link_libraries(sr_tests_musl_libm PRIVATE sr_musl_libm GTest::gtest_main)
  target_compile_options(sr_tests_musl_libm PRIVATE -fno-builtin -ffp-contract=off -fno-fast-math)
  gtest_discover_tests(sr_tests_musl_libm PROPERTIES LABELS unit DISCOVERY_TIMEOUT 60)
endif()

# WP0a vendor tests (architecture.md §7.3). Added by tests/CMakeLists.txt (WP0b) with
# add_subdirectory(vendor) when SLAM_REFACT_HAVE_VENDOR is ON; needs GTest::gtest_main.
include(GoogleTest)

add_executable(sr_tests_vendor vendor_test.cc vendor_probe.cc)
target_link_libraries(sr_tests_vendor PRIVATE sr_vendor_geometry GTest::gtest_main)
target_compile_definitions(sr_tests_vendor PRIVATE SR_TEST_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/../data")
target_compile_options(sr_tests_vendor PRIVATE -fno-exceptions -fno-threadsafe-statics -ffp-contract=off -fno-fast-math)
gtest_discover_tests(sr_tests_vendor PROPERTIES LABELS unit DISCOVERY_TIMEOUT 60)

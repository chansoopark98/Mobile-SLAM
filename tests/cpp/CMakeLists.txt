# ── Mobile-SLAM unit tests ────────────────────────────────

enable_testing()

file(GLOB_RECURSE TEST_SOURCES
    "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp"
)

if(TEST_SOURCES)
    add_executable(slam_tests ${TEST_SOURCES})

    target_include_directories(slam_tests PRIVATE
        ${CMAKE_SOURCE_DIR}/include
        ${EIGEN3_INCLUDE_DIR}
    )

    target_link_libraries(slam_tests PRIVATE
        GTest::gtest
        GTest::gtest_main
        Eigen3::Eigen
    )

    # Link slam_core if it exists
    if(TARGET slam_core)
        target_link_libraries(slam_tests PRIVATE slam_core)
    endif()

    include(GoogleTest)
    gtest_discover_tests(slam_tests)
else()
    message(STATUS "No test sources found yet -- skipping tests")
endif()

# ── Mobile-SLAM core library ──────────────────────────────
#
# Collects all .cpp sources under src/core/ into a single STATIC library.

file(GLOB_RECURSE SLAM_CORE_SOURCES
    "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp"
)

# Guard: only create the library if we actually have source files.
if(SLAM_CORE_SOURCES)
    add_library(slam_core STATIC ${SLAM_CORE_SOURCES})

    target_include_directories(slam_core PUBLIC
        ${CMAKE_SOURCE_DIR}/include
        ${EIGEN3_INCLUDE_DIR}
    )

    target_link_libraries(slam_core PUBLIC
        Eigen3::Eigen
    )

    if(TARGET ceres)
        target_link_libraries(slam_core PUBLIC ceres)
    elseif(Ceres_FOUND)
        target_link_libraries(slam_core PUBLIC ${CERES_LIBRARIES})
        target_include_directories(slam_core PUBLIC ${CERES_INCLUDE_DIRS})
    endif()

    if(OpenCV_FOUND)
        target_link_libraries(slam_core PUBLIC ${OpenCV_LIBS})
    endif()
else()
    message(STATUS "No core sources found yet -- skipping slam_core library")
endif()

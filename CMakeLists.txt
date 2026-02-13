cmake_minimum_required(VERSION 3.16)
project(MobileSLAM VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

# ccache support
find_program(CCACHE_PROGRAM ccache)
if(CCACHE_PROGRAM)
    set_property(GLOBAL PROPERTY RULE_LAUNCH_COMPILE "${CCACHE_PROGRAM}")
    set_property(GLOBAL PROPERTY RULE_LAUNCH_LINK "${CCACHE_PROGRAM}")
endif()

# Strict IEEE 754 compliance (critical for SLAM numerical stability)
add_compile_definitions(
    EIGEN_FAST_MATH=0
    EIGEN_DONT_VECTORIZE=1
    EIGEN_DISABLE_UNALIGNED_ARRAY_ASSERT=1
)

if(CMAKE_BUILD_TYPE STREQUAL "Release")
    set(CMAKE_CXX_FLAGS_RELEASE "-O2 -DNDEBUG")
elseif(CMAKE_BUILD_TYPE STREQUAL "Debug")
    set(CMAKE_CXX_FLAGS_DEBUG "-O0 -g -DDEBUG")
endif()

# ── Dependencies ──────────────────────────────────────────

# Eigen3 (system or bundled)
find_package(Eigen3 3.4 QUIET)
if(NOT Eigen3_FOUND)
    message(STATUS "System Eigen3 not found, using bundled version")
    set(EIGEN3_INCLUDE_DIR "${CMAKE_SOURCE_DIR}/third_party/eigen")
else()
    message(STATUS "Found system Eigen3: ${Eigen3_VERSION}")
endif()

# OpenCV
find_package(OpenCV 4.0 REQUIRED COMPONENTS core imgproc video calib3d highgui imgcodecs)
message(STATUS "Found OpenCV: ${OpenCV_VERSION}")

# Ceres Solver (FetchContent if not found)
find_package(Ceres 2.0 QUIET)
if(NOT Ceres_FOUND)
    message(STATUS "Ceres not found, fetching via FetchContent...")
    include(FetchContent)
    FetchContent_Declare(
        ceres-solver
        GIT_REPOSITORY https://ceres-solver.googlesource.com/ceres-solver
        GIT_TAG        2.2.0
        GIT_SHALLOW    TRUE
    )
    set(BUILD_TESTING OFF CACHE BOOL "" FORCE)
    set(BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
    set(BUILD_BENCHMARKS OFF CACHE BOOL "" FORCE)
    set(MINIGLOG ON CACHE BOOL "" FORCE)
    set(GFLAGS OFF CACHE BOOL "" FORCE)
    set(EIGENSPARSE ON CACHE BOOL "" FORCE)
    FetchContent_MakeAvailable(ceres-solver)
else()
    message(STATUS "Found system Ceres: ${Ceres_VERSION}")
endif()

# Pangolin (optional, Phase A visualization)
option(BUILD_WITH_PANGOLIN "Build with Pangolin 3D visualization" ON)
if(BUILD_WITH_PANGOLIN)
    find_package(Pangolin QUIET)
    if(Pangolin_FOUND)
        message(STATUS "Found Pangolin")
        add_compile_definitions(USE_PANGOLIN)
    else()
        message(STATUS "Pangolin not found, visualization disabled")
        set(BUILD_WITH_PANGOLIN OFF)
    endif()
endif()

# GTest (FetchContent)
include(FetchContent)
FetchContent_Declare(
    googletest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG        v1.14.0
    GIT_SHALLOW    TRUE
)
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

# ── Include Directories ──────────────────────────────────

include_directories(
    ${CMAKE_SOURCE_DIR}/include
    ${EIGEN3_INCLUDE_DIR}
)

# ── Subdirectories ──────────────────────────────────────

add_subdirectory(src/core)
add_subdirectory(src/app)
add_subdirectory(tests/cpp)

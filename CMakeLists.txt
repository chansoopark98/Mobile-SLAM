cmake_minimum_required(VERSION 3.14)
project(tiny_vins_mono)

# Enable ccache for faster rebuilds
find_program(CCACHE_PROGRAM ccache)
if(CCACHE_PROGRAM)
    set_property(GLOBAL PROPERTY RULE_LAUNCH_COMPILE "${CCACHE_PROGRAM}")
    set_property(GLOBAL PROPERTY RULE_LAUNCH_LINK "${CCACHE_PROGRAM}")
    message(STATUS "Using ccache for faster builds")
else()
    message(STATUS "ccache not found, builds will not be cached")
endif()

set(CMAKE_CXX_STANDARD 17)

# Set build type if not specified
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

# Disable all fast-math optimizations for numerical consistency
add_compile_definitions(
    EIGEN_FAST_MATH=0              # Disable Eigen's fast math
    EIGEN_DONT_VECTORIZE=1         # Disable Eigen vectorization
    EIGEN_DISABLE_UNALIGNED_ARRAY_ASSERT=1
)

# Force strict IEEE 754 compliance
if(CMAKE_BUILD_TYPE STREQUAL "Release")
    set(CMAKE_CXX_FLAGS_RELEASE "-O2 -DNDEBUG")
elseif(CMAKE_BUILD_TYPE STREQUAL "Debug")
    set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -O0 -g -DDEBUG")
endif()

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Pangolin REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(YAML_CPP REQUIRED yaml-cpp)

# ── Ceres Solver (FetchContent) ─────────────────────────
include(FetchContent)

# Configure Ceres for lightweight build (Eigen-only, no SuiteSparse)
set(MINIGLOG ON CACHE BOOL "Use miniglog instead of glog" FORCE)
set(GFLAGS OFF CACHE BOOL "Disable gflags" FORCE)
set(SUITESPARSE OFF CACHE BOOL "Disable SuiteSparse" FORCE)
set(CXSPARSE OFF CACHE BOOL "Disable CXSparse" FORCE)
set(LAPACK OFF CACHE BOOL "Disable LAPACK" FORCE)
set(EIGENSPARSE ON CACHE BOOL "Use Eigen sparse solver" FORCE)
set(SCHUR_SPECIALIZATIONS OFF CACHE BOOL "Disable Schur specializations for faster build" FORCE)
set(BUILD_TESTING OFF CACHE BOOL "Disable Ceres tests" FORCE)
set(BUILD_EXAMPLES OFF CACHE BOOL "Disable Ceres examples" FORCE)
set(BUILD_BENCHMARKS OFF CACHE BOOL "Disable Ceres benchmarks" FORCE)
set(BUILD_SHARED_LIBS OFF CACHE BOOL "Build Ceres as static lib" FORCE)
set(PROVIDE_UNINSTALL_TARGET OFF CACHE BOOL "" FORCE)

FetchContent_Declare(
    ceres-solver
    GIT_REPOSITORY https://github.com/ceres-solver/ceres-solver.git
    GIT_TAG        2.2.0
    GIT_SHALLOW    TRUE
    GIT_PROGRESS   TRUE
)
FetchContent_MakeAvailable(ceres-solver)
message(STATUS "Ceres: 2.2.0 (FetchContent)")

# ── Google Test (FetchContent) ──────────────────────────
FetchContent_Declare(
    googletest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG        v1.14.0
    GIT_SHALLOW    TRUE
)
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)
message(STATUS "GoogleTest: 1.14.0 (FetchContent)")

include_directories(include
    ${EIGEN3_INCLUDE_DIR}
    ${OpenCV_INCLUDE_DIRS}
    ${YAML_CPP_INCLUDE_DIRS}
    ${Pangolin_INCLUDE_DIRS}
)

# Libraries 
add_library(utility src/utility/utility.cpp src/utility/config.cpp)
target_link_libraries(utility ${YAML_CPP_LIBRARIES})

add_library(pose_local_parameterization src/backend/factor/pose_local_parameterization.cpp)
target_link_libraries(pose_local_parameterization ceres)

add_library(marginalization_factor src/backend/factor/marginalization_factor.cpp)
target_link_libraries(marginalization_factor ceres)

add_library(projection_factor src/backend/factor/projection_factor.cpp)
target_link_libraries(projection_factor ceres)

add_library(optimizer src/backend/optimizer.cpp)
target_link_libraries(optimizer ceres marginalization_factor pose_local_parameterization)

add_library(failure_detector src/frontend/failure_detector.cpp)
target_link_libraries(failure_detector ceres)

add_library(initializer
    src/frontend/initialization/initializer.cpp
    src/frontend/initialization/initial_alignment.cpp
    src/frontend/initialization/initial_sfm.cpp
    src/frontend/initialization/solve_5pts.cpp
)
target_link_libraries(initializer ceres utility ${OpenCV_LIBS})

add_library(gpl 
    src/common/gpl/gpl.cc
    src/common/gpl/EigenQuaternionParameterization.cc
)
target_link_libraries(gpl ceres)

add_library(camera_models
    src/common/camera_models/CameraFactory.cc
    src/common/camera_models/CataCamera.cc
    src/common/camera_models/PinholeCamera.cc
    src/common/camera_models/Camera.cc
    src/common/camera_models/EquidistantCamera.cc
    src/common/camera_models/ScaramuzzaCamera.cc
    src/common/camera_models/CostFunctionFactory.cc
)
target_link_libraries(camera_models gpl)

add_library(feature_tracker
    src/frontend/feature_tracker.cpp
)
target_link_libraries(feature_tracker
    utility
    ceres
    camera_models
)

add_library(estimator
    src/backend/estimator.cpp
    src/common/frame.cpp
    src/frontend/feature_manager.cpp
    src/backend/sliding_window.cpp
)
target_link_libraries(estimator
    utility
    ceres
    pose_local_parameterization
    marginalization_factor
    projection_factor
    optimizer
    failure_detector
    initializer
    ${OpenCV_LIBS}
)

# VIO Engine (headless wrapper for WASM and programmatic use)
add_library(vio_engine src/vio_engine.cpp)
target_link_libraries(vio_engine
    estimator
    feature_tracker
    utility
    camera_models
    ${OpenCV_LIBS}
)

# Configuration management library
add_library(config_manager
    src/config/config_manager.cpp
)
target_link_libraries(config_manager utility)

# Trajectory evaluator
add_library(trajectory_evaluator src/utility/trajectory_evaluator.cpp)
target_link_libraries(trajectory_evaluator ${EIGEN3_LIBRARIES})
target_include_directories(trajectory_evaluator PRIVATE include)

# Executable
add_executable(tiny_vins_mono 
    src/tiny_vins_mono.cpp 
    src/vio_system.cpp
    src/utility/measurement_processor.cpp
    src/utility/visualizer.cpp
    src/utility/imu_graph_visualizer.cpp
    src/utility/test_result_logger.cpp
)
target_link_libraries(tiny_vins_mono
    ${OpenCV_LIBS}
    ${YAML_CPP_LIBRARIES}
    ${EIGEN3_LIBRARIES}
    estimator
    feature_tracker
    utility
    config_manager
    trajectory_evaluator
    ${Pangolin_LIBRARIES}
    stdc++fs
)

# ── Tests ────────────────────────────────────────────────
enable_testing()

add_executable(test_integration_base tests/test_integration_base.cpp)
target_link_libraries(test_integration_base
    gtest gtest_main
    estimator
    utility
    ceres
)
target_include_directories(test_integration_base PRIVATE include)
add_test(NAME IntegrationBaseTest COMMAND test_integration_base)

add_executable(test_sliding_window tests/test_sliding_window.cpp)
target_link_libraries(test_sliding_window
    gtest gtest_main
    estimator
    utility
    ceres
)
target_include_directories(test_sliding_window PRIVATE include)
add_test(NAME SlidingWindowTest COMMAND test_sliding_window)

add_executable(test_trajectory_evaluator tests/test_trajectory_evaluator.cpp)
target_link_libraries(test_trajectory_evaluator
    gtest gtest_main
    trajectory_evaluator
)
target_include_directories(test_trajectory_evaluator PRIVATE include)
add_test(NAME TrajectoryEvaluatorTest COMMAND test_trajectory_evaluator)

add_executable(test_config_validation tests/test_config_validation.cpp)
target_link_libraries(test_config_validation
    gtest gtest_main
    config_manager
    utility
)
target_include_directories(test_config_validation PRIVATE include)
add_test(NAME ConfigValidationTest COMMAND test_config_validation)

add_executable(test_measurement_robustness tests/test_measurement_robustness.cpp
    src/utility/measurement_processor.cpp
)
target_link_libraries(test_measurement_robustness
    gtest gtest_main
    utility
    feature_tracker
    ${OpenCV_LIBS}
)
target_include_directories(test_measurement_robustness PRIVATE include)
add_test(NAME MeasurementRobustnessTest COMMAND test_measurement_robustness)
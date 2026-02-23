cmake_minimum_required(VERSION 3.14)
project(vio_wasm)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Release)

# Project root
set(PROJECT_SRC_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/..")
set(LIBS_DIR "${CMAKE_CURRENT_SOURCE_DIR}/libs")

# Eigen (system-installed, header-only — skip find_package for Emscripten compatibility)
set(EIGEN3_INCLUDE_DIR "/usr/include/eigen3")

# Pre-built WASM libraries
set(OPENCV_DIR "${LIBS_DIR}/opencv")
set(CERES_DIR "${LIBS_DIR}/ceres")

# ── Include paths ──
include_directories(
    ${PROJECT_SRC_ROOT}/include
    ${EIGEN3_INCLUDE_DIR}
    ${OPENCV_DIR}/include/opencv4
    ${CERES_DIR}/include
)

# ── OpenCV WASM static libraries ──
file(GLOB LIB_OPENCV
    "${OPENCV_DIR}/lib/libopencv_core.a"
    "${OPENCV_DIR}/lib/libopencv_imgproc.a"
    "${OPENCV_DIR}/lib/libopencv_calib3d.a"
    "${OPENCV_DIR}/lib/libopencv_features2d.a"
    "${OPENCV_DIR}/lib/libopencv_video.a"
    "${OPENCV_DIR}/lib/libopencv_flann.a"
)

file(GLOB LIB_OPENCV_3RDPARTY
    "${OPENCV_DIR}/lib/opencv4/3rdparty/*.a"
)

# ── Ceres WASM static library ──
file(GLOB LIB_CERES
    "${CERES_DIR}/lib/libceres.a"
)

# ── VIO Engine sources (no visualization, no file I/O, no threading) ──
set(VIO_SOURCES
    # VIO Engine (headless wrapper)
    ${PROJECT_SRC_ROOT}/src/vio_engine.cpp

    # Backend
    ${PROJECT_SRC_ROOT}/src/backend/estimator.cpp
    ${PROJECT_SRC_ROOT}/src/backend/optimizer.cpp
    ${PROJECT_SRC_ROOT}/src/backend/sliding_window.cpp
    ${PROJECT_SRC_ROOT}/src/backend/factor/marginalization_factor.cpp
    ${PROJECT_SRC_ROOT}/src/backend/factor/projection_factor.cpp
    ${PROJECT_SRC_ROOT}/src/backend/factor/pose_local_parameterization.cpp

    # Frontend
    ${PROJECT_SRC_ROOT}/src/frontend/feature_tracker.cpp
    ${PROJECT_SRC_ROOT}/src/frontend/feature_manager.cpp
    ${PROJECT_SRC_ROOT}/src/frontend/failure_detector.cpp
    ${PROJECT_SRC_ROOT}/src/frontend/initialization/initializer.cpp
    ${PROJECT_SRC_ROOT}/src/frontend/initialization/initial_alignment.cpp
    ${PROJECT_SRC_ROOT}/src/frontend/initialization/initial_sfm.cpp
    ${PROJECT_SRC_ROOT}/src/frontend/initialization/solve_5pts.cpp

    # Common
    ${PROJECT_SRC_ROOT}/src/common/frame.cpp
    ${PROJECT_SRC_ROOT}/src/common/gpl/gpl.cc
    ${PROJECT_SRC_ROOT}/src/common/gpl/EigenQuaternionParameterization.cc
    ${PROJECT_SRC_ROOT}/src/common/camera_models/CameraFactory.cc
    ${PROJECT_SRC_ROOT}/src/common/camera_models/Camera.cc
    ${PROJECT_SRC_ROOT}/src/common/camera_models/PinholeCamera.cc
    ${PROJECT_SRC_ROOT}/src/common/camera_models/EquidistantCamera.cc
    ${PROJECT_SRC_ROOT}/src/common/camera_models/CataCamera.cc
    ${PROJECT_SRC_ROOT}/src/common/camera_models/ScaramuzzaCamera.cc
    ${PROJECT_SRC_ROOT}/src/common/camera_models/CostFunctionFactory.cc

    # Utility (config only, no file I/O heavy parts)
    ${PROJECT_SRC_ROOT}/src/utility/utility.cpp
    ${PROJECT_SRC_ROOT}/src/utility/config.cpp

    # Embind bindings
    ${CMAKE_CURRENT_SOURCE_DIR}/vio_bindings.cpp
)

add_compile_definitions(
    __EMSCRIPTEN__
    EIGEN_FAST_MATH=0
    EIGEN_DONT_VECTORIZE=1
    EIGEN_DISABLE_UNALIGNED_ARRAY_ASSERT=1
)

# ── Build as WASM module ──
add_executable(vio_engine ${VIO_SOURCES})

target_link_libraries(vio_engine
    ${LIB_OPENCV}
    ${LIB_OPENCV_3RDPARTY}
    ${LIB_CERES}
)

# ── Emscripten linker flags ──
# STACK_SIZE: Ceres solver + Eigen matrix ops require significant stack space.
# Default 64KB is insufficient; 8MB provides ample room for DENSE_SCHUR + marginalization.
# ASSERTIONS: Set to 1 for debug builds to catch memory errors early.
set(EM_LINK_FLAGS
    "-s EXPORT_NAME='VIOWasm'"
    "-s ENVIRONMENT=web,worker"
    "-s MODULARIZE=1"
    "-s EXPORT_ES6=1"
    "-s SINGLE_FILE=1"
    "-s INITIAL_MEMORY=256MB"
    "-s MAXIMUM_MEMORY=512MB"
    "-s ALLOW_MEMORY_GROWTH=1"
    "-s ALLOW_TABLE_GROWTH=1"
    "-s STACK_SIZE=8MB"
    "-s EXPORTED_FUNCTIONS=['_malloc','_free']"
    "-s EXPORTED_RUNTIME_METHODS=['HEAPU8','HEAPF64','HEAPF32','HEAP32','HEAPU32','wasmMemory']"
    "-s DISABLE_EXCEPTION_CATCHING=0"
    "-s ERROR_ON_UNDEFINED_SYMBOLS=0"
    "-s ASSERTIONS=0"
    "-s USE_ZLIB=1"
    "--bind"
    "--no-entry"
    "-O3"
    "-msimd128"
    "-lembind"
)

string(REPLACE ";" " " EM_LINK_FLAGS_STR "${EM_LINK_FLAGS}")
set_target_properties(vio_engine PROPERTIES
    SUFFIX ".js"
    LINK_FLAGS "${EM_LINK_FLAGS_STR}"
)

# ── Worker-compatible build target ──
add_executable(vio_engine_worker ${VIO_SOURCES})

target_link_libraries(vio_engine_worker
    ${LIB_OPENCV}
    ${LIB_OPENCV_3RDPARTY}
    ${LIB_CERES}
)

set(EM_WORKER_LINK_FLAGS
    "-s EXPORT_NAME='VIOWasm'"
    "-s ENVIRONMENT=worker"
    "-s MODULARIZE=1"
    "-s EXPORT_ES6=0"
    "-s SINGLE_FILE=1"
    "-s INITIAL_MEMORY=256MB"
    "-s MAXIMUM_MEMORY=512MB"
    "-s ALLOW_MEMORY_GROWTH=1"
    "-s ALLOW_TABLE_GROWTH=1"
    "-s STACK_SIZE=8MB"
    "-s EXPORTED_FUNCTIONS=['_malloc','_free']"
    "-s EXPORTED_RUNTIME_METHODS=['HEAPU8','HEAPF64','HEAPF32','HEAP32','HEAPU32','wasmMemory']"
    "-s DISABLE_EXCEPTION_CATCHING=0"
    "-s ERROR_ON_UNDEFINED_SYMBOLS=0"
    "-s ASSERTIONS=0"
    "-s USE_ZLIB=1"
    "--bind"
    "--no-entry"
    "-O3"
    "-msimd128"
    "-lembind"
)

string(REPLACE ";" " " EM_WORKER_LINK_FLAGS_STR "${EM_WORKER_LINK_FLAGS}")
set_target_properties(vio_engine_worker PROPERTIES
    SUFFIX ".js"
    LINK_FLAGS "${EM_WORKER_LINK_FLAGS_STR}"
)

# ── Install ──
install(TARGETS vio_engine vio_engine_worker
    RUNTIME DESTINATION "${CMAKE_CURRENT_SOURCE_DIR}/dist"
)

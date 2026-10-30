# Vendored third-party code of the slam_refact engine (WP0a, architecture.md §2.2-§2.3).
#
# Included from slam_refact/engine/CMakeLists.txt with add_subdirectory(third_party).
# Interface expected from the including project (WP0b, cmake/SlamRefactDeps.cmake):
#   - targets Eigen3::Eigen and Ceres::ceres exist before this file is processed.
# The §2.5 flags are applied here per target (target_compile_options on the sr_vendor_* targets
# only, never add_compile_options), so Ceres and GTest keep their own defaults (review NI-9).
# Optimization level comes from the build type (WASM preset: -O3).
# Targets defined here:
#   sr_libcxx_compat    INTERFACE  libc++ (emsdk 5.0.0) algorithm/distribution excerpts
#   sr_vendor_c8        STATIC     C8_SRCS (22) + parameter-data shim
#   sr_vendor_features  STATIC     FEAT_SRCS (18)
#   sr_vendor_geometry  STATIC     GEO_SRCS (12)
#   sr_musl_libm        OBJECT     musl 1.2.5 libm subset (host builds only, musl_libm/PROVENANCE.md)
# Vendored include directories are SYSTEM so their warnings never reach the engine's -Werror;
# vendored sources themselves compile with -w (outside the warning policy, US-007).

if(NOT TARGET Eigen3::Eigen)
  message(FATAL_ERROR "slam_refact third_party: target Eigen3::Eigen must be defined first")
endif()
if(NOT TARGET Ceres::ceres)
  message(FATAL_ERROR "slam_refact third_party: target Ceres::ceres must be defined first")
endif()

set(SR_XR_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/xr_source")
set(SR_XR_SHIM_DIR "${CMAKE_CURRENT_SOURCE_DIR}/xr_shim")

add_library(sr_libcxx_compat INTERFACE)
target_include_directories(sr_libcxx_compat SYSTEM INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}/libcxx_compat/include")

set(SR_VENDOR_C8_SRCS
  c8/hmatrix.cc c8/hpoint.cc c8/hvector.cc c8/quaternion.cc c8/c8-log.cc c8/exceptions.cc
  c8/string/format.cc c8/task-queue.cc c8/thread-pool.cc c8/time/now.cc
  c8/geometry/egomotion.cc c8/geometry/homography.cc c8/geometry/parameterized-geometry.cc
  c8/geometry/vectors.cc c8/geometry/device-pose.cc c8/geometry/two-d.cc c8/geometry/line.cc
  c8/geometry/interpolation.cc
  c8/pixels/gr8-pyramid.cc c8/pixels/pixel-transforms.cc c8/pixels/image-roi.cc
  reality/engine/binning/linear-bin.cc
  # US-005: LightingEstimator calls estimateExposureScore, which links pixel-transforms.o and so
  # needs the two definitions it references (also built into sr_vendor_face_pixels for face
  # consumers; objects given directly on a link line win over these archive members).
  reality/engine/lighting/lighting-estimator.cc c8/color-maps.cc c8/pixels/packed.cc)
set(SR_VENDOR_FEATURES_SRCS
  reality/engine/features/gr8gl.cc reality/engine/features/image-point.cc
  reality/engine/features/image-descriptor.cc reality/engine/features/frame-point.cc
  reality/engine/features/keypoint-queue.cc reality/engine/features/feature-scales.cc
  reality/engine/features/feature-manager.cc reality/engine/features/detection-config.cc
  reality/engine/features/descriptor-lsh-index.cc reality/engine/features/descriptor-bit-counter.cc
  reality/engine/features/global-matcher.cc reality/engine/features/local-matcher.cc
  reality/engine/features/point-descriptor.cc reality/engine/features/point-matches.cc
  reality/engine/features/gr8cpu-interface.cc reality/engine/features/gr8cpu-nop.cc
  reality/engine/tracking/ray-point-filter.cc reality/engine/tracking/rolling-lin-reg.cc)
set(SR_VENDOR_GEOMETRY_SRCS
  reality/engine/geometry/bundle.cc reality/engine/geometry/bundle-residual.cc
  reality/engine/geometry/bundle-residual-analytic.cc reality/engine/geometry/observed-point.cc
  reality/engine/geometry/epipolar.cc reality/engine/geometry/pose-pnp.cc
  reality/engine/geometry/ap3p.cc reality/engine/geometry/p2p.cc
  reality/engine/geometry/reprojection.cc reality/engine/geometry/homography-decomp.cc
  reality/engine/geometry/point-regression.cc
  reality/engine/geometry/analytical-residual/pose-estimation.cc)

list(TRANSFORM SR_VENDOR_C8_SRCS PREPEND "${SR_XR_SOURCE_DIR}/")
list(TRANSFORM SR_VENDOR_FEATURES_SRCS PREPEND "${SR_XR_SOURCE_DIR}/")
list(TRANSFORM SR_VENDOR_GEOMETRY_SRCS PREPEND "${SR_XR_SOURCE_DIR}/")

if(NOT EMSCRIPTEN)
  # c8/time/now.cc is excluded from the host build: unreferenced in the closure (nothing includes
  # c8/time/now.h or calls nowMicros()), and under JAVASCRIPT it is emscripten-only
  # (emscripten/emscripten.h + emscripten_get_now). JAVASCRIPT itself stays defined on every
  # target (single-threaded detection, gr8gl.cc:93-97, thread-pool.cc:27). See PROVENANCE.md.
  list(REMOVE_ITEM SR_VENDOR_C8_SRCS "${SR_XR_SOURCE_DIR}/c8/time/now.cc")
endif()

add_library(sr_vendor_c8 STATIC ${SR_VENDOR_C8_SRCS} "${SR_XR_SHIM_DIR}/parameter-data.cc")
add_library(sr_vendor_features STATIC ${SR_VENDOR_FEATURES_SRCS})
add_library(sr_vendor_geometry STATIC ${SR_VENDOR_GEOMETRY_SRCS})

# The shim directory precedes the vendored tree (scope-timer.h and intrinsics.h replace the
# Cap'n Proto dependent originals).
target_include_directories(sr_vendor_c8 SYSTEM PUBLIC "${SR_XR_SHIM_DIR}" "${SR_XR_SOURCE_DIR}")
# Original WASM toolchain defines (PUB:bzl/crosstool/wasm-cc-toolchain.bzl:303-307, D-15).
# PUBLIC so every translation unit that includes the vendored headers sees the same inline code.
target_compile_definitions(sr_vendor_c8 PUBLIC JAVASCRIPT C8_NO_EXCEPT EIGEN_FAST_MATH=0)
target_link_libraries(sr_vendor_c8 PUBLIC Eigen3::Eigen sr_libcxx_compat)
target_link_libraries(sr_vendor_features PUBLIC sr_vendor_c8)
target_link_libraries(sr_vendor_geometry PUBLIC sr_vendor_features Ceres::ceres)
# frame-point.cc (features) calls epipolar.cc (geometry): the two archives are mutually
# dependent upstream, so declare the back edge and let CMake repeat them on the link line.
target_link_libraries(sr_vendor_features INTERFACE sr_vendor_geometry)
set_property(TARGET sr_vendor_features sr_vendor_geometry PROPERTY LINK_INTERFACE_MULTIPLICITY 2)

# §2.5 (D-15): exception model and floating-point contract of the original WASM toolchain.
set(SR_VENDOR_COMPILE_OPTIONS -fno-exceptions -fno-threadsafe-statics -ffp-contract=off -fno-fast-math -w)
if(EMSCRIPTEN)
  list(APPEND SR_VENDOR_COMPILE_OPTIONS -msimd128)
else()
  list(APPEND SR_VENDOR_COMPILE_OPTIONS -march=x86-64)
endif()
foreach(tgt sr_vendor_c8 sr_vendor_features sr_vendor_geometry)
  target_compile_options(${tgt} PRIVATE ${SR_VENDOR_COMPILE_OPTIONS})
  set_target_properties(${tgt} PROPERTIES CXX_STANDARD 17 CXX_STANDARD_REQUIRED ON CXX_EXTENSIONS OFF)
endforeach()

# musl 1.2.5 libm subset (emsdk 5.0.0 libc sources, musl_libm/PROVENANCE.md). The original
# xr-slam.wasm links emscripten's musl libm; glibc rounds these differently in the last bit for some
# arguments, which moved native exact-input replays off the original. Host builds therefore link these
# definitions, which take precedence over the shared glibc libm for every call in the executable.
# em++ already links the same musl code.
if(NOT EMSCRIPTEN)
  set(SR_MUSL_LIBM_DIR "${CMAKE_CURRENT_SOURCE_DIR}/musl_libm")
  set(SR_MUSL_LIBM_SRCS
    sinf.c cosf.c sincosf.c __sindf.c __cosdf.c __rem_pio2f.c __rem_pio2_large.c
    atan2f.c atanf.c log10f.c
    acosf.c logf.c logf_data.c expf.c exp2f_data.c
    sin.c cos.c sincos.c __sin.c __cos.c __rem_pio2.c
    pow.c pow_data.c exp_data.c
    __math_xflowf.c __math_uflowf.c __math_oflowf.c __math_divzerof.c __math_invalidf.c
    __math_xflow.c __math_uflow.c __math_oflow.c __math_divzero.c __math_invalid.c)
  list(TRANSFORM SR_MUSL_LIBM_SRCS PREPEND "${SR_MUSL_LIBM_DIR}/src/")
  add_library(sr_musl_libm OBJECT ${SR_MUSL_LIBM_SRCS})
  target_include_directories(sr_musl_libm PRIVATE "${SR_MUSL_LIBM_DIR}/include")
  target_compile_options(sr_musl_libm PRIVATE -ffp-contract=off -fno-fast-math -fno-builtin -w)
  set_target_properties(sr_musl_libm PROPERTIES C_STANDARD 99 C_STANDARD_REQUIRED ON
                                                POSITION_INDEPENDENT_CODE ON)
  # PUBLIC: every target that links the vendored c8 code links these objects too.
  target_link_libraries(sr_vendor_c8 PUBLIC sr_musl_libm)
endif()

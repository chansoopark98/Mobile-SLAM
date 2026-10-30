# Third-party dependencies of the slam_refact engine (architecture.md §2.3, §2.4).
#   Eigen 3.4 (/usr/include/eigen3), Ceres SLAM_REFACT_CERES_VERSION (2.2 default: native offline
#   FetchContent from the cached source, WASM prebuilt wasm/libs/ceres; 2.0: the original
#   xr-slam.wasm's release, native from build/_deps/ceres-2.0.0-src, WASM from
#   build_wasm_ceres.sh; with SLAM_REFACT_CERES_COMMIT an exact upstream commit such as the reference
#   source pin 2b32b321 from fetch_ceres_pin.sh), GoogleTest 1.14 (native tests only, offline).
# Nothing here downloads: FETCHCONTENT_FULLY_DISCONNECTED is forced ON.

include_guard(GLOBAL)

# ---- Eigen ------------------------------------------------------------------------------------
set(Eigen3_DIR "/usr/share/eigen3/cmake" CACHE PATH "Eigen 3.4 package config directory")
find_package(Eigen3 3.4 REQUIRED NO_MODULE)
message(STATUS "slam_refact: Eigen ${Eigen3_VERSION} (${EIGEN3_INCLUDE_DIR})")

# ---- Ceres ------------------------------------------------------------------------------------
# Single-threaded execution is Solver::Options::num_threads = 1 (Ceres default); engine code must
# not raise it (architecture.md §4.6). Ceres 2.2 has no CERES_THREADING_MODEL option (removed
# upstream; it always builds against std::thread). Ceres 2.0 is built with NO_THREADS, as the
# original is (its binary carries parallel_for_nothreads.cc and no other parallel_for source).
# Upstream commit of a Ceres source or install: "commit <sha>" in VERSION (release tarballs) or
# SLAM_REFACT_SOURCE (fetch_ceres_pin.sh, copied into the WASM install by build_wasm_ceres.sh).
# Empty when neither exists (2.2 FetchContent source, wasm/libs/ceres).
function(sr_ceres_commit_of dir out_var)
  set(_commit "")
  foreach(f SLAM_REFACT_SOURCE VERSION)
    if(_commit STREQUAL "" AND EXISTS "${dir}/${f}")
      file(STRINGS "${dir}/${f}" _line REGEX "^commit [0-9a-f]+$" LIMIT_COUNT 1)
      if(_line)
        string(REGEX REPLACE "^commit " "" _commit "${_line}")
      endif()
    endif()
  endforeach()
  set(${out_var} "${_commit}" PARENT_SCOPE)
endfunction()

# The 2.0.0 release commit (tag 2.0.0 -> 399cda77; the release tarball's VERSION names it).
set(SR_CERES_2_0_0_COMMIT "399cda773035d99eaf1f4a129a666b3c4df9d1b1")

# Refuses a source/install whose commit is not SLAM_REFACT_CERES_COMMIT (or, when that is empty,
# a 2.0 source that is not the 2.0.0 release) and records the selection in the build dir.
function(sr_check_ceres_commit dir kind)
  sr_ceres_commit_of("${dir}" _found)
  if(SLAM_REFACT_CERES_COMMIT)
    if(NOT _found STREQUAL SLAM_REFACT_CERES_COMMIT)
      message(FATAL_ERROR
        "slam_refact: ${kind} Ceres at '${dir}' is commit '${_found}', but "
        "SLAM_REFACT_CERES_COMMIT=${SLAM_REFACT_CERES_COMMIT}")
    endif()
  elseif(SLAM_REFACT_CERES_VERSION STREQUAL "2.0" AND _found
         AND NOT _found STREQUAL SR_CERES_2_0_0_COMMIT)
    message(FATAL_ERROR
      "slam_refact: ${kind} Ceres at '${dir}' is commit ${_found}, not the 2.0.0 release; set "
      "SLAM_REFACT_CERES_COMMIT=${_found} to build it")
  endif()
  if(_found STREQUAL "")
    set(_found "unrecorded")
  endif()
  file(WRITE "${CMAKE_BINARY_DIR}/slam_refact_ceres.txt"
    "version ${SLAM_REFACT_CERES_VERSION}\ncommit ${_found}\nkind ${kind}\ndir ${dir}\n")
  set(SR_CERES_COMMIT_FOUND "${_found}" PARENT_SCOPE)
endfunction()

function(sr_add_native_ceres)
  if(NOT EXISTS "${SLAM_REFACT_CERES_SOURCE_DIR}/CMakeLists.txt")
    message(FATAL_ERROR
      "slam_refact: Ceres source not found at SLAM_REFACT_CERES_SOURCE_DIR="
      "'${SLAM_REFACT_CERES_SOURCE_DIR}'. Expected the cached Ceres "
      "${SLAM_REFACT_CERES_VERSION}.0 source; the engine build never downloads it.")
  endif()
  file(READ "${SLAM_REFACT_CERES_SOURCE_DIR}/package.xml" _pkg)
  string(REGEX MATCH "<version>([0-9.]+)</version>" _m "${_pkg}")
  set(_sr_ceres_src_version "${CMAKE_MATCH_1}")
  if(NOT _sr_ceres_src_version STREQUAL "${SLAM_REFACT_CERES_VERSION}.0")
    message(FATAL_ERROR
      "slam_refact: '${SLAM_REFACT_CERES_SOURCE_DIR}' is Ceres '${_sr_ceres_src_version}', but "
      "SLAM_REFACT_CERES_VERSION=${SLAM_REFACT_CERES_VERSION} needs ${SLAM_REFACT_CERES_VERSION}.0")
  endif()
  # Same options as the repo root build (CMakeLists.txt:48-61), plus EIGENMETIS OFF to match the
  # WASM Ceres components (CeresConfig.cmake: EigenSparse;SparseLinearAlgebraLibrary;
  # SchurSpecializations).
  set(MINIGLOG ON CACHE BOOL "" FORCE)
  set(GFLAGS OFF CACHE BOOL "" FORCE)
  set(SUITESPARSE OFF CACHE BOOL "" FORCE)
  set(CXSPARSE OFF CACHE BOOL "" FORCE)
  set(LAPACK OFF CACHE BOOL "" FORCE)
  set(EIGENSPARSE ON CACHE BOOL "" FORCE)
  set(EIGENMETIS OFF CACHE BOOL "" FORCE)
  set(USE_CUDA OFF CACHE BOOL "" FORCE)
  set(SCHUR_SPECIALIZATIONS ON CACHE BOOL "" FORCE)
  set(BUILD_TESTING OFF CACHE BOOL "" FORCE)
  set(BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
  set(BUILD_BENCHMARKS OFF CACHE BOOL "" FORCE)
  set(BUILD_DOCUMENTATION OFF CACHE BOOL "" FORCE)
  set(BUILD_SHARED_LIBS OFF CACHE BOOL "" FORCE)
  set(PROVIDE_UNINSTALL_TARGET OFF CACHE BOOL "" FORCE)
  set(EXPORT_BUILD_DIR OFF CACHE BOOL "" FORCE)
  if(SLAM_REFACT_CERES_VERSION STREQUAL "2.0")
    # 2.0-only options. CUSTOM_BLAS stays at its default (ON), as in Ceres' own Bazel build,
    # which the original used (its source paths are external/ceres/...).
    set(CERES_THREADING_MODEL "NO_THREADS" CACHE STRING "" FORCE)
    set(ACCELERATESPARSE OFF CACHE BOOL "" FORCE)
    set(CUSTOM_BLAS ON CACHE BOOL "" FORCE)
  endif()

  sr_check_ceres_commit("${SLAM_REFACT_CERES_SOURCE_DIR}" native)

  include(FetchContent)
  set(FETCHCONTENT_FULLY_DISCONNECTED ON CACHE BOOL "" FORCE)
  set(FETCHCONTENT_SOURCE_DIR_CERES_SOLVER "${SLAM_REFACT_CERES_SOURCE_DIR}" CACHE PATH "" FORCE)
  FetchContent_Declare(ceres_solver
    GIT_REPOSITORY https://github.com/ceres-solver/ceres-solver.git
    GIT_TAG ${SLAM_REFACT_CERES_VERSION}.0)
  FetchContent_MakeAvailable(ceres_solver)
  if(NOT TARGET Ceres::ceres OR NOT TARGET ceres)
    message(FATAL_ERROR "slam_refact: Ceres::ceres target missing after FetchContent")
  endif()
  # Ceres headers are SYSTEM for engine code (§2.5); the imported WASM target already is.
  get_target_property(_ceres_inc ceres INTERFACE_INCLUDE_DIRECTORIES)
  set_property(TARGET ceres APPEND PROPERTY INTERFACE_SYSTEM_INCLUDE_DIRECTORIES ${_ceres_inc})
  message(STATUS "slam_refact: Ceres ${_sr_ceres_src_version} commit ${SR_CERES_COMMIT_FOUND} "
                 "(native, offline source ${SLAM_REFACT_CERES_SOURCE_DIR})")
endfunction()

function(sr_find_wasm_ceres)
  if(NOT EXISTS "${SLAM_REFACT_WASM_CERES_DIR}/CeresConfig.cmake")
    message(FATAL_ERROR
      "slam_refact: WASM Ceres not found at SLAM_REFACT_WASM_CERES_DIR="
      "'${SLAM_REFACT_WASM_CERES_DIR}' (2.2: wasm/build_ceres.sh; 2.0: "
      "slam_refact/engine/tools/build_wasm_ceres.sh).")
  endif()
  # Explicit <Pkg>_DIR so the Emscripten CMAKE_FIND_ROOT_PATH re-rooting does not apply.
  set(Ceres_DIR "${SLAM_REFACT_WASM_CERES_DIR}" CACHE PATH "" FORCE)
  find_package(Ceres ${SLAM_REFACT_CERES_VERSION} REQUIRED CONFIG)
  if(NOT Ceres_VERSION MATCHES "^${SLAM_REFACT_CERES_VERSION}\\.")
    message(FATAL_ERROR "slam_refact: WASM Ceres ${Ceres_VERSION} at ${Ceres_DIR}, but "
                        "SLAM_REFACT_CERES_VERSION=${SLAM_REFACT_CERES_VERSION}")
  endif()
  sr_check_ceres_commit("${SLAM_REFACT_WASM_CERES_DIR}" wasm)
  message(STATUS "slam_refact: Ceres ${Ceres_VERSION} commit ${SR_CERES_COMMIT_FOUND} "
                 "(WASM prebuilt ${Ceres_DIR})")
endfunction()

# ---- GoogleTest (native only) -------------------------------------------------------------------
# gtest keeps its own compile flags (NI-9). GTEST_HAS_EXCEPTIONS=0 is a PUBLIC definition of
# gtest/gtest_main so the library and the -fno-exceptions test TUs agree on gtest's exception
# configuration (gtest builds cleanly this way; verified in the WP0b report).
function(sr_add_gtest)
  if(NOT EXISTS "${SLAM_REFACT_GTEST_SOURCE_DIR}/CMakeLists.txt")
    message(FATAL_ERROR
      "slam_refact: GoogleTest source not found at SLAM_REFACT_GTEST_SOURCE_DIR="
      "'${SLAM_REFACT_GTEST_SOURCE_DIR}' (expected cached googletest 1.14.0).")
  endif()
  set(BUILD_GMOCK OFF CACHE BOOL "" FORCE)
  set(INSTALL_GTEST OFF CACHE BOOL "" FORCE)
  set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
  include(FetchContent)
  set(FETCHCONTENT_FULLY_DISCONNECTED ON CACHE BOOL "" FORCE)
  set(FETCHCONTENT_SOURCE_DIR_GOOGLETEST "${SLAM_REFACT_GTEST_SOURCE_DIR}" CACHE PATH "" FORCE)
  FetchContent_Declare(googletest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG v1.14.0)
  FetchContent_MakeAvailable(googletest)
  foreach(t gtest gtest_main)
    target_compile_definitions(${t} PUBLIC GTEST_HAS_EXCEPTIONS=0)
  endforeach()
  message(STATUS "slam_refact: GoogleTest (offline source ${SLAM_REFACT_GTEST_SOURCE_DIR})")
endfunction()

# ---- Vendored public reference runtime sources (WP0a) ----------------------------------------------------
# WP0a owns third_party/ and its CMakeLists.txt. Contract with WP0a (CONTRACTS.md):
#   targets sr_vendor_c8, sr_vendor_features, sr_vendor_geometry (STATIC) and sr_libcxx_compat
#   (INTERFACE) with usage requirements carrying their include directories.
set(SR_VENDOR_TARGETS sr_vendor_c8 sr_vendor_features sr_vendor_geometry sr_libcxx_compat)

function(sr_add_vendor)
  set(_tp "${SLAM_REFACT_THIRD_PARTY_DIR}")
  if(NOT EXISTS "${_tp}/CMakeLists.txt")
    set(SLAM_REFACT_HAVE_VENDOR OFF PARENT_SCOPE)
    if(SLAM_REFACT_ALLOW_MISSING_VENDOR)
      message(WARNING
        "slam_refact: vendored sources missing (${_tp}/CMakeLists.txt, owned by WP0a). "
        "SLAM_REFACT_ALLOW_MISSING_VENDOR=ON: building only the c8-free contract checks "
        "(public API headers, public contract test, config defaults). Engine modules, "
        "src/common tracking types, compat, config params and their tests are NOT built.")
      return()
    endif()
    message(FATAL_ERROR
      "slam_refact: vendored public reference runtime sources are missing: "
      "'${_tp}/CMakeLists.txt' does not exist.\n"
      "This tree is produced by work package WP0a (architecture.md §2.2, §8.2). The engine "
      "cannot be configured without it.\n"
      "To check only the c8-free public API contract, configure with "
      "-DSLAM_REFACT_ALLOW_MISSING_VENDOR=ON.")
  endif()
  add_subdirectory("${_tp}" "${CMAKE_BINARY_DIR}/third_party")
  foreach(t IN LISTS SR_VENDOR_TARGETS)
    if(NOT TARGET ${t})
      message(FATAL_ERROR
        "slam_refact: ${_tp}/CMakeLists.txt did not define target '${t}'. Expected targets: "
        "${SR_VENDOR_TARGETS} (architecture.md §2.3).")
    endif()
    # Vendored headers are SYSTEM for engine code so their warnings never trip -Werror (§2.5).
    get_target_property(_inc ${t} INTERFACE_INCLUDE_DIRECTORIES)
    if(_inc)
      set_property(TARGET ${t} APPEND PROPERTY INTERFACE_SYSTEM_INCLUDE_DIRECTORIES ${_inc})
    endif()
  endforeach()
  set(SLAM_REFACT_HAVE_VENDOR ON PARENT_SCOPE)
  message(STATUS "slam_refact: vendored targets: ${SR_VENDOR_TARGETS}")
endfunction()

# Compile flags of every slam_refact target (architecture.md §2.5 "전체", D-15; review NI-1, NI-9).
#
# Applied per target (target_compile_options / target_compile_definitions), never as global
# add_compile_options: Ceres and GoogleTest keep their own defaults. Applied to every non-imported,
# non-INTERFACE target whose name starts with sr_ or slam_refact_ (engine modules, vendored
# sr_vendor_* from WP0a, tests, wasm, harness) by sr_apply_engine_flags_to_tree() at the end of
# the top-level CMakeLists.txt, so targets added later by any work package are covered.

include_guard(GLOBAL)

function(sr_target_engine_flags target)
  # Exception model of the original WASM toolchain (PUB:bzl/crosstool/wasm-cc-toolchain.bzl:
  # 303-307): C8_THROW and SR_CHECK log and abort.
  target_compile_options(${target} PRIVATE -fno-exceptions -fno-threadsafe-statics)
  # JAVASCRIPT on native and WASM: single-threaded gr8 detection path (NI-1). WP0a handles the
  # native-only exception for c8/time/now.cc; this file does not override it.
  target_compile_definitions(${target} PRIVATE C8_NO_EXCEPT JAVASCRIPT)
  # Same float semantics on native and WASM: no fast-math, no FMA contraction (S5 §8.3).
  target_compile_options(${target} PRIVATE -fno-fast-math -ffp-contract=off)
  target_compile_definitions(${target} PRIVATE EIGEN_FAST_MATH=0)
  if(EMSCRIPTEN)
    target_compile_options(${target} PRIVATE -msimd128)
  else()
    # SSE2 only (no AVX): Eigen packets of 4 floats like WASM SIMD128 [M7a].
    target_compile_options(${target} PRIVATE -march=x86-64)
  endif()
endfunction()

function(_sr_collect_targets dir out_var)
  get_property(_targets DIRECTORY "${dir}" PROPERTY BUILDSYSTEM_TARGETS)
  get_property(_subdirs DIRECTORY "${dir}" PROPERTY SUBDIRECTORIES)
  foreach(sub IN LISTS _subdirs)
    _sr_collect_targets("${sub}" _sub_targets)
    list(APPEND _targets ${_sub_targets})
  endforeach()
  set(${out_var} ${_targets} PARENT_SCOPE)
endfunction()

function(sr_apply_engine_flags_to_tree)
  _sr_collect_targets("${PROJECT_SOURCE_DIR}" _all)
  set(_applied 0)
  foreach(t IN LISTS _all)
    if(NOT t MATCHES "^(sr_|slam_refact_)")
      continue()
    endif()
    get_target_property(_type ${t} TYPE)
    get_target_property(_imported ${t} IMPORTED)
    if(_type STREQUAL "INTERFACE_LIBRARY" OR _type STREQUAL "UTILITY" OR _imported)
      continue()
    endif()
    sr_target_engine_flags(${t})
    math(EXPR _applied "${_applied} + 1")
  endforeach()
  message(STATUS "slam_refact: engine compile flags applied to ${_applied} sr_*/slam_refact_* targets")
endfunction()

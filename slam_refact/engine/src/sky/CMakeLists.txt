# Sky segmentation module (full-parity PRD US-020; docs/spec/09-sky.md).
#
# sr_vendor_sky: the public reference runtime semantics classifier vendored under third_party/xr_source
# (PROVENANCE.md "sky"): reality/engine/semantics/semantics-classifier.cc and
# reality/engine/deepnets/tflite-multiclassifier.cc, running the model through the TFLiteInterpreter
# shim (third_party/xr_shim/tflite-interpreter.cc -> src/ml). Defined here, not in
# third_party/CMakeLists.txt, so the world SLAM vendor targets stay as they were. Like the other
# vendored libraries it compiles with -w; the engine flags reach it through
# sr_apply_engine_flags_to_tree(). tflite-multiclassifier.h includes "tflite-interpreter.h" without a
# directory, so the shim's deepnets directory is on this target's include path (no patch to the
# vendored file).
# The replaced parts of the public module (the GL cubemap renderer and its shaders, the capnp
# messages, the JS glue layers-controller.cc / layers-controller.ts / semantics-worker.{cc,js}) are
# implemented by sr_sky.

set(_sr_xr_reference_dir "${SLAM_REFACT_THIRD_PARTY_DIR}/xr_source")
set(_sr_sky_vendor_srcs
  reality/engine/semantics/semantics-classifier.cc
  reality/engine/deepnets/tflite-multiclassifier.cc)
list(TRANSFORM _sr_sky_vendor_srcs PREPEND "${_sr_xr_reference_dir}/")

# The TFLiteInterpreter shim is also compiled into sr_vendor_face; it is built here separately from
# the same file so sr_sky does not depend on the face closure.
add_library(sr_vendor_sky STATIC ${_sr_sky_vendor_srcs}
  "${SLAM_REFACT_THIRD_PARTY_DIR}/xr_shim/tflite-interpreter.cc")
target_link_libraries(sr_vendor_sky PUBLIC sr_vendor_c8 sr_ml)
target_include_directories(sr_vendor_sky SYSTEM PUBLIC
  "${SLAM_REFACT_THIRD_PARTY_DIR}/xr_shim/reality/engine/deepnets")
target_compile_options(sr_vendor_sky PRIVATE -w)
set_target_properties(sr_vendor_sky PROPERTIES CXX_STANDARD 17 CXX_STANDARD_REQUIRED ON
                                               CXX_EXTENSIONS OFF)

sr_add_module(sr_sky
  DEPS sr_common sr_ml sr_vendor_sky)

# Pipeline timing bench (preprocess / inference / cubemap + post-process per stage), native and WASM
# (node). Reads the model from assets/references.
add_executable(sr_sky_bench "${CMAKE_CURRENT_SOURCE_DIR}/bench/sky_bench_main.cc")
target_link_libraries(sr_sky_bench PRIVATE sr_sky)
sr_engine_warnings(sr_sky_bench)
if(EMSCRIPTEN)
  # Built on request only (--target sr_sky_bench), so engine WASM builds of other lanes are
  # unchanged.
  set_target_properties(sr_sky_bench PROPERTIES SUFFIX ".js" EXCLUDE_FROM_ALL ON)
  target_link_options(sr_sky_bench PRIVATE
    -sENVIRONMENT=node
    -sNODERAWFS=1
    -sEXIT_RUNTIME=1
    -sINITIAL_MEMORY=256MB
    -sMAXIMUM_MEMORY=512MB
    -sALLOW_MEMORY_GROWTH=1
    -sSTACK_SIZE=8MB)
endif()

# Replay of a recorded original run (tools/re/runtime/run-sky-reference.mjs): the exact camera
# textures and device orientations through this pipeline, native only (comparison tools).
if(NOT EMSCRIPTEN)
  add_executable(sr_sky_replay "${CMAKE_CURRENT_SOURCE_DIR}/bench/sky_replay_main.cc")
  target_link_libraries(sr_sky_replay PRIVATE sr_sky)
  sr_engine_warnings(sr_sky_replay)
  # The replay tool refuses the sealed gate sets of the registration (bench/sky_held.h).
  target_compile_definitions(sr_sky_replay PRIVATE
    SR_REPO_ROOT="${SLAM_REFACT_REPO_ROOT}"
    SR_SKY_PREREG_DIR="${SLAM_REFACT_REPO_ROOT}/build/slam-refact-full-20261019/preregistration")
endif()

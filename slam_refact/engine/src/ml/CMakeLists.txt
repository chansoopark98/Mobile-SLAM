# ML inference engine (US-016/US-017; slam_refact/docs/design/ml-runtime.md). The C++ standard
# library only (own deterministic GEMM, gemm.h); no flatbuffers / TFLite / XNNPACK. Eigen3 stays
# linked only because other lanes may rely on its transitive include path; no ML code uses it.
sr_add_module(sr_ml
  DEPS sr_includes Eigen3::Eigen)

# Inference time and accuracy bench, native and WASM (node). Reads the models from
# assets/references and the numpy reference bundles (tools/ml_ref/ml_ref.py run-all).
add_executable(sr_ml_bench "${CMAKE_CURRENT_SOURCE_DIR}/bench/ml_bench_main.cc")
target_link_libraries(sr_ml_bench PRIVATE sr_ml)
sr_engine_warnings(sr_ml_bench)
if(EMSCRIPTEN)
  # Built on request only (--target sr_ml_bench), so engine WASM builds of other lanes are
  # unchanged.
  set_target_properties(sr_ml_bench PROPERTIES SUFFIX ".js" EXCLUDE_FROM_ALL ON)
  target_link_options(sr_ml_bench PRIVATE
    -sENVIRONMENT=node
    -sNODERAWFS=1
    -sEXIT_RUNTIME=1
    -sINITIAL_MEMORY=256MB
    -sMAXIMUM_MEMORY=512MB
    -sALLOW_MEMORY_GROWTH=1
    -sSTACK_SIZE=8MB)
endif()

# Hardening run (rejects, structure-aware family, byte-flip fuzz) without GoogleTest, so the same
# checks can run under node (RM-1d, review ML m3; docs/design/ml-lane-closure.md). Built on request
# only (--target sr_ml_harden): native and WASM print the same counters and digest.
add_executable(sr_ml_harden "${CMAKE_CURRENT_SOURCE_DIR}/bench/ml_harden_main.cc")
target_link_libraries(sr_ml_harden PRIVATE sr_ml)
sr_engine_warnings(sr_ml_harden)
set_target_properties(sr_ml_harden PROPERTIES EXCLUDE_FROM_ALL ON)
if(EMSCRIPTEN)
  set_target_properties(sr_ml_harden PROPERTIES SUFFIX ".js")
  target_link_options(sr_ml_harden PRIVATE
    -sENVIRONMENT=node
    -sNODERAWFS=1
    -sEXIT_RUNTIME=1
    -sINITIAL_MEMORY=256MB
    -sMAXIMUM_MEMORY=512MB
    -sALLOW_MEMORY_GROWTH=1
    -sSTACK_SIZE=8MB)
endif()

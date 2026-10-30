# Hand module (full-parity PRD US-021; docs/spec/10-hand.md): the MediaPipe 2020 palm detection + hand
# landmark graph on the slam_refact ML engine. CONFIGURED ceiling: a substitute for the removed reference
# runtime hand models, not a reproduction. Dependencies: sr_ml and Eigen only (OpenCV is the test
# oracle of the image operations in tools/hand, never linked).
sr_add_module(sr_hand
  DEPS sr_common sr_ml Eigen3::Eigen)

# Per-stage timing bench, native and WASM (node): palm / landmark preprocess, inference,
# postprocess on a synthetic camera frame (docs/spec/10-hand.md §6). Models from assets/references.
add_executable(sr_hand_bench "${CMAKE_CURRENT_SOURCE_DIR}/bench/hand_bench_main.cc")
target_link_libraries(sr_hand_bench PRIVATE sr_hand)
sr_engine_warnings(sr_hand_bench)
if(EMSCRIPTEN)
  set_target_properties(sr_hand_bench PROPERTIES SUFFIX ".js" EXCLUDE_FROM_ALL ON)
  target_link_options(sr_hand_bench PRIVATE
    -sENVIRONMENT=node
    -sNODERAWFS=1
    -sEXIT_RUNTIME=1
    -sINITIAL_MEMORY=256MB
    -sMAXIMUM_MEMORY=512MB
    -sALLOW_MEMORY_GROWTH=1
    -sSTACK_SIZE=8MB)
endif()

# Runs the tracker on a directory of RGB frames and writes per-frame JSON lines (and optionally the
# raw palm model outputs for the independent oracle). Native only.
if(NOT EMSCRIPTEN)
  add_executable(sr_hand_run "${CMAKE_CURRENT_SOURCE_DIR}/bench/hand_run_main.cc")
  target_link_libraries(sr_hand_run PRIVATE sr_hand)
  sr_engine_warnings(sr_hand_run)
endif()

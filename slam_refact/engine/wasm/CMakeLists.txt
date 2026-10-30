# WASM module of the engine (architecture.md §2.6, §7.4; WP5). Added by the top-level CMakeLists.txt
# for Emscripten builds. Output: <build>/wasm/slam_refact_engine.{js,wasm} (MODULARIZE,
# EXPORT_NAME SlamRefactEngine, stack 8 MiB; SlamRefactWasm.cmake).
add_executable(slam_refact_wasm "${CMAKE_CURRENT_SOURCE_DIR}/bindings.cc")
target_link_libraries(slam_refact_wasm PRIVATE sr_engine)
sr_engine_warnings(slam_refact_wasm)
sr_wasm_link_options(slam_refact_wasm)

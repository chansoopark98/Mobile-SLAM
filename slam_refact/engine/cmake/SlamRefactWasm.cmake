# Emscripten specifics (architecture.md §2.6). em++ 5.0.0 from ~/emsdk; configure through the
# slam-refact-wasm preset after `source ~/emsdk/emsdk_env.sh`.

include_guard(GLOBAL)

if(NOT EMSCRIPTEN)
  message(FATAL_ERROR "SlamRefactWasm.cmake included in a non-Emscripten build")
endif()

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "" FORCE)
endif()
# -O3 for Release (CMake's Emscripten default is -O3 as well; stated explicitly).
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG" CACHE STRING "" FORCE)

# Link options of the engine module (WP5 applies this to slam_refact_wasm in wasm/CMakeLists.txt).
# Output: slam_refact_engine.{js,wasm}; SINGLE_FILE is not used.
function(sr_wasm_link_options target)
  set_target_properties(${target} PROPERTIES OUTPUT_NAME slam_refact_engine SUFFIX ".js")
  target_link_options(${target} PRIVATE
    -sMODULARIZE=1
    -sEXPORT_NAME=SlamRefactEngine
    -sENVIRONMENT=node,web
    -sINITIAL_MEMORY=256MB
    -sMAXIMUM_MEMORY=512MB
    -sALLOW_MEMORY_GROWTH=1
    -sSTACK_SIZE=8MB
    -lembind)
endfunction()

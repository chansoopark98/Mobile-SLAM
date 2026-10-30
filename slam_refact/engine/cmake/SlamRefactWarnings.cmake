# Warning policy (architecture.md §2.5): -Wall -Wextra on engine sources (src/**, wasm/,
# tests/**, harness); -Werror when SLAM_REFACT_WERROR=ON. Vendored code is outside this policy
# (WP0a builds it with -w) and is consumed through SYSTEM include directories.

include_guard(GLOBAL)

function(sr_engine_warnings target)
  get_target_property(_type ${target} TYPE)
  if(_type STREQUAL "INTERFACE_LIBRARY")
    return()
  endif()
  target_compile_options(${target} PRIVATE -Wall -Wextra)
  if(SLAM_REFACT_WERROR)
    target_compile_options(${target} PRIVATE -Werror)
  endif()
endfunction()

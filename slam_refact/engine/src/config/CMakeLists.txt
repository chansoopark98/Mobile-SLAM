# Process-global parameters and per-getter settings (architecture.md §3.1).
add_library(sr_config STATIC params.cc accuracy.cc)
target_link_libraries(sr_config PUBLIC sr_config_defaults sr_common sr_vendor_c8)
sr_engine_warnings(sr_config)

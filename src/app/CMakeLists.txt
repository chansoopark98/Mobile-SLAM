# ── Mobile-SLAM application executables ────────────────────

add_executable(run_tum_vi run_tum_vi.cpp)

target_link_libraries(run_tum_vi PRIVATE slam_core)

# Install to bin/
install(TARGETS run_tum_vi DESTINATION bin)

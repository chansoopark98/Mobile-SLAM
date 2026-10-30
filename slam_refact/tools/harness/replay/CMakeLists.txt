# Replay runner (architecture.md §2.3 slam_refact_replay, §6.1; WP5). Added by the engine's top-level
# CMakeLists.txt when SLAM_REFACT_BUILD_HARNESS is ON (native only).
#
# zlib (system, development harness only) reads the gzip pyramid captures for injection (H7, V-H7);
# it is not an engine dependency (the engine links Eigen and Ceres only).
find_package(ZLIB REQUIRED)

# Bundle reader + trace writer: public API types only, no engine symbols.
add_library(sr_replay_io STATIC
  "${CMAKE_CURRENT_SOURCE_DIR}/bundle_reader.cc"
  "${CMAKE_CURRENT_SOURCE_DIR}/trace_writer.cc")
target_include_directories(sr_replay_io PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")
target_link_libraries(sr_replay_io PUBLIC sr_includes PRIVATE ZLIB::ZLIB)
sr_engine_warnings(sr_replay_io)

# The runner compiles against the frozen engine.h / glue_runner.h in every build (object library),
# so API drift breaks the default build immediately.
add_library(slam_refact_replay_obj OBJECT "${CMAKE_CURRENT_SOURCE_DIR}/replay_main.cc")
target_link_libraries(slam_refact_replay_obj PRIVATE sr_replay_io sr_includes)
sr_engine_warnings(slam_refact_replay_obj)

# The executable links Engine/GlueRunner (src/engine, WP5b) and is part of the default build.
add_executable(slam_refact_replay $<TARGET_OBJECTS:slam_refact_replay_obj>)
target_link_libraries(slam_refact_replay PRIVATE sr_replay_io slam_refact::engine)

if(SLAM_REFACT_BUILD_TESTS)
  sr_add_unit_test(sr_replay_io_test
    SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/replay_io_test.cc"
    LIBS sr_replay_io
    LABELS unit)
  target_link_libraries(sr_replay_io_test PRIVATE ZLIB::ZLIB)
endif()

# Differential MS-RT check (WP5b): records the 944 inputs at the TrackerCore boundary so the
# original 944 can be run on the engine's own inputs (tools/harness/mse_original_probe.mjs).
add_executable(slam_refact_mse_dump "${CMAKE_CURRENT_SOURCE_DIR}/mse_dump_main.cc")
target_link_libraries(slam_refact_mse_dump PRIVATE sr_replay_io slam_refact::engine)
sr_engine_warnings(slam_refact_mse_dump)

# US-026 root-cause diagnostic (REPORT tier): replay + per-frame keyframe-pose / type-1 point hashes in the
# original memprobe convention. Reads the WorldMap via linker --wrap captures; no engine/src change.
add_executable(slam_refact_replay_maphash "${CMAKE_CURRENT_SOURCE_DIR}/maphash_replay_main.cc"
  "${CMAKE_CURRENT_SOURCE_DIR}/trkprobe.cc")
target_link_libraries(slam_refact_replay_maphash PRIVATE sr_replay_io slam_refact::engine sr_includes)
target_include_directories(slam_refact_replay_maphash PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/../../../engine/src")
target_link_options(slam_refact_replay_maphash PRIVATE
  "LINKER:--wrap=_ZN11slam_refact11map_builder25IterativeBundleAdjustmentERNS_3map8WorldMapE"
  "LINKER:--wrap=_ZN11slam_refact3map8WorldMapC1Ev"
  "LINKER:--wrap=_ZN11slam_refact3map8WorldMapD1Ev"
  "LINKER:--wrap=_ZN11slam_refact3map8WorldMapD2Ev"
  # A-3 trkprobe.cc read-only captures (tracker modules, RNG draw counts)
  "LINKER:--wrap=_ZN11slam_refact11vio_tracker10VioTrackerC1Ebb"
  "LINKER:--wrap=_ZN11slam_refact11vio_tracker10VioTrackerD1Ev"
  "LINKER:--wrap=_ZN11slam_refact11vio_tracker10VioTrackerD2Ev"
  "LINKER:--wrap=_ZN11slam_refact11map_tracker10MapTracker5TrackERNS_3map8WorldMapEPNS_12TrackerInputERKNS_10FrameStateEPNS_16MapTrackerOutputEPN2c813RandomNumbersE"
  "LINKER:--wrap=_ZN11slam_refact19frame_frame_tracker17FrameFrameTracker5TrackEPN2c87HMatrixERKNS2_15FrameWithPointsERKNS2_10QuaternionEPSt6vectorIjSaIjEESE_SE_Pb"
  "LINKER:--wrap=_ZN11slam_refact19frame_frame_tracker22RobustGroundHomographyERKN2c810QuaternionERKSt6vectorINS1_6HPointILm2EEESaIS7_EES4_SB_PNS1_28RobustHomographyScratchSpaceEPNS1_13RandomNumbersEPNS1_7HMatrixESH_PS5_IhSaIhEE"
  "LINKER:--wrap=_ZN11slam_refact11map_tracker11TrackPointsERKNS_3map8WorldMapERKNS_12TrackerInputERKN2c87HMatrixERKSt6vectorINS8_15WorldMapPointIdESaISD_EEPNS0_14MapTrackResultE"
  "LINKER:--wrap=_ZN11slam_refact11map_tracker10RelocalizeEPNS_11RelocResultERKNS_3map8WorldMapERKNS_12TrackerInputERKN2c87HMatrixEPNSA_13RandomNumbersEPNSA_21RobustP2PScratchSpaceE"
  "LINKER:--wrap=_ZN11slam_refact11map_builder10MapBuilder6UpdateEPNS_3map9WorldMapsERKNS_12TrackerInputERKNS_16MapTrackerOutputElPNS_16MapBuilderOutputE")
sr_engine_warnings(slam_refact_replay_maphash)

# Face and ear module (full-parity PRD US-018, US-019; docs/spec/08-face-ear.md).
#
# sr_vendor_face: the public reference runtime face/ear closure vendored under third_party/xr_source
# (PROVENANCE.md "face and ear"), plus the TFLiteInterpreter shim implementation
# (third_party/xr_shim/tflite-interpreter.cc) that runs the models on sr_ml. Defined here, not
# in third_party/CMakeLists.txt, so the world SLAM vendor targets stay as they were. Like the other
# vendored libraries it compiles with -w; the engine flags reach it through
# sr_apply_engine_flags_to_tree().
# The replaced parts of the public module (GL face/ear ROI renderers, face-messages capnp filling,
# the JS glue facecontroller.cc) are implemented by sr_face.

set(_sr_xr_reference_dir "${SLAM_REFACT_THIRD_PARTY_DIR}/xr_source")
set(_sr_face_vendor_srcs
  c8/geometry/box2.cc
  c8/geometry/face-types.cc
  c8/geometry/face-augmentation.cc
  c8/geometry/mesh.cc
  c8/geometry/mesh-types.cc
  reality/engine/deepnets/detection-anchor-nms.cc
  reality/engine/faces/face-geometry.cc
  reality/engine/faces/tracked-face-state.cc
  reality/engine/faces/face-detector-global.cc
  reality/engine/faces/face-detector-local.cc
  reality/engine/faces/face-tracker.cc
  reality/engine/ears/ear-types.cc
  reality/engine/ears/ear-detector-local.cc)
list(TRANSFORM _sr_face_vendor_srcs PREPEND "${_sr_xr_reference_dir}/")

# Definitions referenced by c8/pixels/pixel-transforms.cc (in sr_vendor_c8), whose object file the
# face detectors are the first executables to pull in (toLetterboxRGBFloat*): GRAYSCALE_RGB_256 and
# decodeFloat. An OBJECT library whose objects go on the link line of every sr_face consumer (object
# files are always linked, so the archive order of sr_vendor_c8 does not matter).
add_library(sr_vendor_face_pixels OBJECT "${_sr_xr_reference_dir}/c8/color-maps.cc"
                                         "${_sr_xr_reference_dir}/c8/pixels/packed.cc")
target_link_libraries(sr_vendor_face_pixels PRIVATE sr_vendor_c8)
target_compile_options(sr_vendor_face_pixels PRIVATE -w)
set_target_properties(sr_vendor_face_pixels PROPERTIES CXX_STANDARD 17 CXX_STANDARD_REQUIRED ON
                                                       CXX_EXTENSIONS OFF)

add_library(sr_vendor_face STATIC ${_sr_face_vendor_srcs}
  "${SLAM_REFACT_THIRD_PARTY_DIR}/xr_shim/tflite-interpreter.cc")
target_link_libraries(sr_vendor_face PUBLIC sr_vendor_geometry sr_vendor_features sr_vendor_c8
                                            sr_ml)
target_link_libraries(sr_vendor_face INTERFACE $<TARGET_OBJECTS:sr_vendor_face_pixels>)
target_compile_options(sr_vendor_face PRIVATE -w)
set_target_properties(sr_vendor_face PROPERTIES CXX_STANDARD 17 CXX_STANDARD_REQUIRED ON
                                                CXX_EXTENSIONS OFF)

sr_add_module(sr_face
  DEPS sr_common sr_ml sr_vendor_face)

# Pipeline timing bench, native and WASM (node): synthetic frames through FaceController, per
# stage and per model (docs/spec/08-face-ear.md §7). Reads the models from assets/references.
add_executable(sr_face_bench "${CMAKE_CURRENT_SOURCE_DIR}/bench/face_bench_main.cc")
target_link_libraries(sr_face_bench PRIVATE sr_face)
sr_engine_warnings(sr_face_bench)
if(EMSCRIPTEN)
  # Built on request only (--target sr_face_bench), so engine WASM builds of other lanes are
  # unchanged.
  set_target_properties(sr_face_bench PROPERTIES SUFFIX ".js" EXCLUDE_FROM_ALL ON)
  target_link_options(sr_face_bench PRIVATE
    -sENVIRONMENT=node
    -sNODERAWFS=1
    -sEXIT_RUNTIME=1
    -sINITIAL_MEMORY=256MB
    -sMAXIMUM_MEMORY=512MB
    -sALLOW_MEMORY_GROWTH=1
    -sSTACK_SIZE=8MB)
endif()

# Replay of recorded/synthetic RGBA frames to JSON lines (comparison with the original xr-face
# runtime, docs/spec/08-face-ear.md §8.4). Native only.
if(NOT EMSCRIPTEN)
  add_executable(sr_face_replay "${CMAKE_CURRENT_SOURCE_DIR}/bench/face_replay_main.cc")
  target_link_libraries(sr_face_replay PRIVATE sr_face)
  sr_engine_warnings(sr_face_replay)
  # Open-loop mesh diagnostic on the original ROIs (§5.2 T7e).
  add_executable(sr_face_openloop "${CMAKE_CURRENT_SOURCE_DIR}/bench/face_openloop_main.cc")
  target_link_libraries(sr_face_openloop PRIVATE sr_face)
  sr_engine_warnings(sr_face_openloop)
  # Open-loop ear model diagnostic on the original ear crops (§5.5, RM-7 G3).
  add_executable(sr_face_earloop "${CMAKE_CURRENT_SOURCE_DIR}/bench/face_earloop_main.cc")
  target_link_libraries(sr_face_earloop PRIVATE sr_face)
  sr_engine_warnings(sr_face_earloop)
endif()

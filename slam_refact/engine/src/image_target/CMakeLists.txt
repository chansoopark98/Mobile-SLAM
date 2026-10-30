# Image target module (full-parity PRD US-011, US-012).
#
# sr_vendor_imagedetection: the public reference runtime image target core vendored under
# third_party/xr_source (PROVENANCE.md "image target core"). It is defined here, not in
# third_party/CMakeLists.txt, so the world SLAM vendor targets stay byte-for-byte as they were.
# The engine flags (-fno-exceptions, JAVASCRIPT, -ffp-contract=off, ...) reach it through
# sr_apply_engine_flags_to_tree() like every sr_* target; like the other vendored libraries it
# compiles with -w (outside the warning policy) and its headers are SYSTEM through sr_vendor_c8.
# The replaced parts of the public module (detection-image-loader, the GL path, and
# image-detection-request-executor, the capnp response) are not vendored: sr_image_target
# implements the loader on the CPU pyramid port.

set(_sr_xr_reference_dir "${SLAM_REFACT_THIRD_PARTY_DIR}/xr_source")
set(_sr_imagedetection_srcs
  reality/engine/imagedetection/tracked-image.cc
  reality/engine/imagedetection/target-point.cc
  reality/engine/imagedetection/detection-image-matcher.cc
  reality/engine/imagedetection/detection-image-local-matcher.cc
  reality/engine/imagedetection/detection-image.cc
  reality/engine/imagedetection/image-tracking-state.cc
  reality/engine/imagedetection/detection-image-tracker.cc
  reality/engine/imagedetection/location.cc
  reality/engine/imagedetection/local-points-filter.cc
  reality/engine/imagedetection/image-targets-rigid-assembly.cc
  reality/engine/geometry/image-warp.cc)
list(TRANSFORM _sr_imagedetection_srcs PREPEND "${_sr_xr_reference_dir}/")

add_library(sr_vendor_imagedetection STATIC ${_sr_imagedetection_srcs})
target_link_libraries(sr_vendor_imagedetection PUBLIC sr_vendor_geometry sr_vendor_features
                                                      sr_vendor_c8)
# PUB:reality/engine/imagedetection/BUILD detection-image copts.
target_compile_definitions(sr_vendor_imagedetection PRIVATE _USE_MATH_DEFINES)
target_compile_options(sr_vendor_imagedetection PRIVATE -w)
set_target_properties(sr_vendor_imagedetection PROPERTIES CXX_STANDARD 17 CXX_STANDARD_REQUIRED ON
                                                          CXX_EXTENSIONS OFF)

sr_add_module(sr_image_target
  DEPS sr_common sr_config sr_frontend sr_vendor_imagedetection)

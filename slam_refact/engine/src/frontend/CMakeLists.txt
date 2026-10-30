# Module target. Created by WP0b; owned by WP1 from the contract freeze (architecture.md §8.1).
# sr_vendor_geometry: the vendored FrameWithPoints (frame-point.cc in sr_vendor_features, used by
# FeatureFrontend::Detect) calls the epipolar depth helpers of geometry/epipolar.cc.
sr_add_module(sr_frontend
  DEPS sr_common sr_compat sr_config sr_vendor_features sr_vendor_geometry)
# sr_vendor_features -> sr_vendor_geometry is an undeclared static-library cycle in
# third_party/CMakeLists.txt (WP0a). Until it is declared there, repeat the geometry archive after
# the vendored libraries so frame-point.o resolves epipolar.o (reported in the WP1 report).
target_link_libraries(sr_frontend PUBLIC "$<LINK_ONLY:$<TARGET_FILE:sr_vendor_geometry>>")

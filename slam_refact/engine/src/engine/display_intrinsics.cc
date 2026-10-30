// Display intrinsics (spec 06 §1). Same float operations and order as the public source.

#include "engine/display_intrinsics.h"

#include <algorithm>

#include "frontend/frontend.h"
#include "frontend/intrinsics.h"

namespace slam_refact {

PinholeIntrinsics CropAndScaleIntrinsics(const PinholeIntrinsics& k, int width, int height) {
  // intrinsics.cc:247-263.
  const float scale = std::max(static_cast<float>(width) / k.width,
                               static_cast<float>(height) / k.height);
  const float center_x = (width - 1) / 2.0f + (k.cx - (k.width - 1) / 2.0f) * scale;
  const float center_y = (height - 1) / 2.0f + (k.cy - (k.height - 1) / 2.0f) * scale;
  return {width, height, center_x, center_y, k.fx * scale, k.fy * scale};
}

std::array<float, 16> ToClipSpace(const PinholeIntrinsics& k, float near_clip, float far_clip) {
  // intrinsics.cc:88-109.
  const float c00 = 2 * k.fx / k.width;
  const float c11 = 2 * k.fy / k.height;
  const float c02 = 1.0f - (2.0f * k.cx + 1.0f) / k.width;
  const float c12 = -1.0f + (2.0f * k.cy + 1.0f) / k.height;
  const float c22 = (far_clip + near_clip) / (near_clip - far_clip);
  const float c23 = 2 * far_clip * near_clip / (near_clip - far_clip);
  const float c32 = -1;
  std::array<float, 16> m{};
  m[0] = c00;
  m[5] = c11;
  m[8] = c02;
  m[9] = c12;
  m[10] = c22;
  m[11] = c32;
  m[14] = c23;
  return m;
}

PinholeIntrinsics CaptureIntrinsicsFor(const EngineConfig& cfg) {
  // Level-0 size of the engine's pyramid (the processing K is cropped to it); the capture K does
  // not depend on it, but ComputeIntrinsicsSet takes it.
  const frontend::PyramidLayout layout =
      frontend::ComputePyramidLayout(cfg.capture_width, cfg.capture_height);
  const int l0w = layout.levels[0].w;
  const int l0h = layout.levels[0].h;
  if (cfg.intrinsics_override.has_value()) {
    return CropAndScaleIntrinsics(*cfg.intrinsics_override, cfg.capture_width, cfg.capture_height);
  }
  const float accuracy_focal =
      cfg.accuracy.enabled ? frontend::AccuracyDeviceFocal(cfg.manufacturer, cfg.model) : 0.0f;
  if (accuracy_focal > 0.0f) {
    return frontend::ComputeIntrinsicsSetWithFocal(accuracy_focal, cfg.capture_width,
                                                   cfg.capture_height, l0w, l0h)
        .capture;
  }
  return frontend::ComputeIntrinsicsSet(cfg.manufacturer, cfg.model, cfg.capture_width,
                                        cfg.capture_height, l0w, l0h)
      .capture;
}

std::array<float, 16> DisplayProjection(const PinholeIntrinsics& capture,
                                        const GraphicsIntrinsics& graphics) {
  if (graphics.far_clip > 0) {
    // getDisplayIntrinsics (intrinsics.cc:285-288) + setIntrinsicsMatrix (executor :80-90).
    return ToClipSpace(
        CropAndScaleIntrinsics(capture, graphics.texture_width, graphics.texture_height),
        graphics.near_clip, graphics.far_clip);
  }
  // camera-request-executor.cc:103-106 (zeros) and :192-198 (fixed fallback).
  std::array<float, 16> m{};
  m[0] = 2.92424f;
  m[5] = 1.64488f;
  m[8] = 0.0f;
  m[9] = 0.0015625f;
  m[10] = -1.0006f;
  m[11] = -1.0f;
  m[14] = -0.60018f;
  return m;
}

}  // namespace slam_refact

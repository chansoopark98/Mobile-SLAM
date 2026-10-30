// Pyramid atlas layout: GlRealityFrame::initialize + initializeLevels (func 459;
// public reality/engine/features/gl-reality-frame.cc:115-196, 198-233; S1 §2.2).

#include <algorithm>
#include <cmath>
#include <utility>

#include "common/check.h"
#include "frontend/frontend.h"
#include "frontend/gr8_cpu_shader.h"

namespace slam_refact::frontend {
namespace {

constexpr float kRoiAspect = 0.75f;  // c8/pixels/image-roi.h ROI_ASPECT

}  // namespace

PyramidLayout ComputePyramidLayout(int camera_w, int camera_h, int pyramid_size) {
  SR_CHECK(camera_w > 0 && camera_h > 0, "camera size must be positive");
  // initializeLevels leaves the texture size unset below 128 (gl-reality-frame.cc:118-139).
  SR_CHECK(pyramid_size > 127, "GlRealityFrame.PyramidSize must be >= 128");

  PyramidLayout out;
  // gl-reality-frame.cc:203-208.
  const bool rotate = camera_w > camera_h;
  const int input_w = rotate ? camera_h : camera_w;
  const int input_h = rotate ? camera_w : camera_h;
  out.rotate_input = rotate;
  out.filtered_w = rotate ? kFilteredHeight : kFilteredWidth;
  out.filtered_h = rotate ? kFilteredWidth : kFilteredHeight;

  // initializeLevels(inputWidth_, inputHeight_, pyrSize, pyrSize): gl-reality-frame.cc:115-196.
  const int output_w = pyramid_size;
  const int output_h = pyramid_size;
  int tex = 0;
  int target_height = 640;
  if (output_h > 2047 && output_w > 2047) {
    tex = 2048;
    target_height = 1280;
  } else if (output_h > 1023 && output_w > 1023) {
    tex = 1024;
    target_height = 640;
  } else if (output_h > 511 && output_w > 511) {
    tex = 512;
    target_height = 320;
  } else if (output_h > 255 && output_w > 255) {
    tex = 256;
    target_height = 160;
  } else {
    tex = 128;
    target_height = 80;
  }
  out.texture_size = tex;

  const float src_aspect = static_cast<float>(input_w) / input_h;
  float target_width = src_aspect * target_height;
  if (target_width > target_height * 3 / 4) {
    const float scale = (target_height * 3 / 4) / target_width;
    target_width *= scale;
    target_height = static_cast<int>(target_height * scale);  // int *= float truncates
  }

  auto& levels = out.levels;
  for (int i = 0; i < 4; ++i) {
    const float scale = 1.0f / std::pow(1.44, i);
    levels[i].w = static_cast<int>(target_width * scale);
    levels[i].h = static_cast<int>(target_height * scale);
  }
  levels[0].r = kAtlasGutter;
  levels[0].c = kAtlasGutter;
  levels[1].r = kAtlasGutter;
  levels[1].c = levels[0].w + kAtlasGutter + kAtlasGutter;
  std::swap(levels[1].h, levels[1].w);
  levels[1].rotated = true;
  levels[2].r = levels[1].h + kAtlasGutter + kAtlasGutter;
  levels[2].c = levels[0].w + kAtlasGutter + kAtlasGutter;
  levels[3].r = levels[2].r;
  levels[3].c = levels[2].c + levels[2].w + kAtlasGutter;

  const int roi_max_width = (tex - 15) / 4;
  int roi_max_height = tex;
  int roi_y = 0;
  for (const LevelLayout& l : levels) {
    const int r = l.r + l.h + kAtlasGutter;
    const int h = tex - r - kAtlasGutter;
    if (h < roi_max_height) {
      roi_max_height = h;
      roi_y = r;
    }
  }
  const int roi_height = roi_max_height * kRoiAspect > roi_max_width
                             ? static_cast<int>(roi_max_width / kRoiAspect)
                             : roi_max_height;
  const int roi_width = roi_max_width / kRoiAspect > roi_max_height
                            ? static_cast<int>(roi_max_height * kRoiAspect)
                            : roi_max_width;
  for (int i = 0; i < 4; ++i) {
    out.roi_slots[i] = {roi_y, i * (kAtlasGutter + roi_width) + kAtlasGutter, roi_width,
                        roi_height, false};
  }

  // DECISION (WP1): the CPU atlas covers the level rectangles plus the gutter below the lowest
  // level. Everything below is R = 0 in the GPU texture (ROI slots are cleared, the rest is never
  // drawn), so treating unallocated rows as 0 reproduces the GPU stage-2/3 values inside the
  // levels exactly. The CPU half only reads inside level rectangles (S1 §5.6-5.10).
  int bottom = 0;
  for (const LevelLayout& l : levels) {
    bottom = std::max(bottom, l.r + l.h);
  }
  out.rows = std::min(tex, bottom + kAtlasGutter);
  return out;
}

}  // namespace slam_refact::frontend

// slam_refact public API: non-owning image views (architecture.md §4.1).
//
// FROZEN CONTRACT (WP0b). Changes follow slam_refact/docs/design/contract-changes.md.

#ifndef SLAM_REFACT_IMAGE_TYPES_H_
#define SLAM_REFACT_IMAGE_TYPES_H_

#include <cstdint>

namespace slam_refact {

// 8-bit gray image; stride in bytes. Row 0 = top of the image.
struct GrayImageView {
  const uint8_t* data = nullptr;
  int width = 0;
  int height = 0;
  int stride = 0;
};

// RGBA8 image; stride in bytes. Row 0 = top of the image.
struct RgbaImageView {
  const uint8_t* data = nullptr;
  int width = 0;
  int height = 0;
  int stride = 0;
};

// Injected pyramid atlas (H7): size x rows RGBA8, row bytes = 4 * size, row 0 = GL row 0
// (S1 §2.2, §7).
struct PyramidAtlasView {
  const uint8_t* rgba = nullptr;
  int size = 0;
  int rows = 0;
};

// Rows an injected atlas (size 1024) must have for a capture of capture_width x capture_height
// (the EngineConfig capture size), computed by the engine's own pyramid layout (RM-3c, API-2 review
// MINOR-9: the layout itself is an internal type, so a native producer had no public way to learn
// this number; the embind surface reports the same values as pyramidLayout().rows / roiAtlasRows).
//   PyramidAtlasRows:    the lowest rows of an atlas without image-target ROI slots.
//   PyramidAtlasRoiRows: the lowest rows of an atlas that carries the ROI slots (FULL-7): required
//                        while GlueRunner::NextDrawRois() is not empty, and the producer must have
//                        drawn those ROIs into the slots (the engine reads the slots as given; rows
//                        alone do not make an atlas valid).
// Both return 0 for a non-positive capture size.
int PyramidAtlasRows(int capture_width, int capture_height);
int PyramidAtlasRoiRows(int capture_width, int capture_height);

}  // namespace slam_refact

#endif  // SLAM_REFACT_IMAGE_TYPES_H_

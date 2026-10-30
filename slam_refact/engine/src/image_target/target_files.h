// Target files (US-012): binary PGM luminance images and the JSON + PGM pair written by
// slam_refact/tools/image_target/image_target_cli.py.
//
// The engine runtime has no JPEG/PNG decoder (dependency rule): the CLI's `<name>_luminance.<ext>`
// is decoded by the browser (createImageBitmap) or, offline, by the Python tool, which also writes
// `<name>_luminance.pgm` with the decoded luminance bytes. Only binary PGM (P5) with maxval 255 is
// accepted.

#ifndef SLAM_REFACT_IMAGE_TARGET_TARGET_FILES_H_
#define SLAM_REFACT_IMAGE_TARGET_TARGET_FILES_H_

#include <cstdint>
#include <string>
#include <string_view>
#include <vector>

#include "image_target/image_target_spec.h"
#include "slam_refact/image_types.h"

namespace slam_refact::image_target {

// Owning 8-bit gray image (row-major, stride = width).
struct GrayImage {
  int width = 0;
  int height = 0;
  std::vector<uint8_t> pixels;

  GrayImageView view() const { return {pixels.data(), width, height, width}; }
};

// Binary PGM (P5, maxval 255, '#' comments in the header). Returns false with *error on bad input.
bool ParsePgm(std::string_view bytes, GrayImage* out, std::string* error);

bool ReadFileBytes(const std::string& path, std::string* bytes, std::string* error);

// Reads `json_path` (ParseImageTargetJson) and the luminance PGM `pgm_path`.
bool LoadImageTargetFiles(const std::string& json_path, const std::string& pgm_path,
                          ImageTargetSpec* spec, GrayImage* luminance, std::string* error);

}  // namespace slam_refact::image_target

#endif  // SLAM_REFACT_IMAGE_TARGET_TARGET_FILES_H_

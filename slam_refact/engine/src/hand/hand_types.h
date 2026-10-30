// Types of the hand module (US-021; docs/spec/10-hand.md): the MediaPipe 2020 palm detection + hand
// landmark graph (mediapipe-fork graphs/hand_tracking) reimplemented on the slam_refact ML engine.
//
// CONFIGURED ceiling: the three hand models of the original reference runtime hand tracker (palm_lite,
// landmark_lite, hand_mesh_v2) were removed before the sources were opened, so this is a substitute
// built from the public MediaPipe models. Nothing here claims equality with the reference runtime output.
#ifndef SLAM_REFACT_HAND_HAND_TYPES_H_
#define SLAM_REFACT_HAND_HAND_TYPES_H_

#include <array>
#include <cstdint>
#include <string>
#include <vector>

namespace slam_refact::hand {

// mediapipe::NormalizedRect: centre and size normalised by the image size, rotation in radians.
struct NormRect {
  float x_center = 0.0f;
  float y_center = 0.0f;
  float width = 0.0f;
  float height = 0.0f;
  float rotation = 0.0f;
};

struct Keypoint {
  float x = 0.0f;
  float y = 0.0f;
};

// mediapipe::Detection with a RELATIVE_BOUNDING_BOX and relative keypoints (a single score).
struct Detection {
  float score = 0.0f;
  float xmin = 0.0f;
  float ymin = 0.0f;
  float width = 0.0f;
  float height = 0.0f;
  std::vector<Keypoint> keypoints;
};

// mediapipe::NormalizedLandmark: x, y normalised by the image size; z in the model's pixel-like
// units divided by normalize_z (1.0 for the hand graph).
struct Landmark {
  float x = 0.0f;
  float y = 0.0f;
  float z = 0.0f;
};

// mediapipe::Anchor (SsdAnchorsCalculator output).
struct Anchor {
  float x_center = 0.0f;
  float y_center = 0.0f;
  float w = 0.0f;
  float h = 0.0f;
};

// 8-bit RGB image, row 0 = top (the SRGB ImageFrame the CPU graph works on).
struct Image3 {
  int width = 0;
  int height = 0;
  std::vector<uint8_t> px;  // width * height * 3

  Image3() = default;
  Image3(int w, int h) : width(w), height(h), px(static_cast<size_t>(w) * h * 3, 0) {}
  uint8_t* at(int x, int y) { return &px[(static_cast<size_t>(y) * width + x) * 3]; }
  const uint8_t* at(int x, int y) const { return &px[(static_cast<size_t>(y) * width + x) * 3]; }
  bool empty() const { return width <= 0 || height <= 0; }
};

// Drops alpha from an RGBA8 frame (stride in bytes).
Image3 RgbaToRgb(const uint8_t* rgba, int width, int height, int stride);

}  // namespace slam_refact::hand

#endif  // SLAM_REFACT_HAND_HAND_TYPES_H_

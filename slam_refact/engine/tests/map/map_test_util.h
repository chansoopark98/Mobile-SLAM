// Helpers for the world map tests: synthetic keyframe frames with ORB+GORB descriptors.

#ifndef SLAM_REFACT_TESTS_MAP_MAP_TEST_UTIL_H_
#define SLAM_REFACT_TESTS_MAP_MAP_TEST_UTIL_H_

#include <array>
#include <cstdint>
#include <vector>

#include "c8/geometry/pixel-pinhole-camera-model.h"
#include "c8/hpoint.h"
#include "reality/engine/features/frame-point.h"
#include "reality/engine/features/image-descriptor.h"
#include "slam_refact/types.h"

namespace slam_refact::map::testing {

inline c8_PixelPinholeCameraModel TestIntrinsics() {
  c8_PixelPinholeCameraModel k;
  k.pixelsWidth = 320;
  k.pixelsHeight = 240;
  k.centerPointX = 159.5f;
  k.centerPointY = 119.5f;
  k.focalLengthHorizontal = 260.0f;
  k.focalLengthVertical = 260.0f;
  return k;
}

inline PinholeIntrinsics TestPinhole() { return {320, 240, 159.5f, 119.5f, 260.0f, 260.0f}; }

// Descriptor whose bytes are all `byte`.
inline std::array<uint8_t, 32> Bytes(uint8_t byte) {
  std::array<uint8_t, 32> d;
  d.fill(byte);
  return d;
}

struct RayPoint {
  float x;
  float y;
  uint8_t scale;
  uint8_t gorb_byte;
};

// A frame with one keypoint per ray; every keypoint has an ORB and a GORB descriptor.
inline c8::FrameWithPoints MakeFrame(const std::vector<RayPoint>& pts) {
  c8::FrameWithPoints f(TestIntrinsics());
  for (const RayPoint& p : pts) {
    c8::FeatureSet set;
    set.orbFeature = c8::OrbFeature(Bytes(static_cast<uint8_t>(p.gorb_byte ^ 0xff)));
    set.gorbFeature = c8::GorbFeature(Bytes(p.gorb_byte));
    f.addImageRayPoint(c8::HPoint2(p.x, p.y), p.scale, 0.25f, 0.5f, std::move(set));
  }
  return f;
}

inline std::vector<RayPoint> Grid(int n, uint8_t scale = 1) {
  std::vector<RayPoint> pts;
  for (int i = 0; i < n; ++i) {
    pts.push_back({-0.3f + 0.6f * static_cast<float>(i % 5) / 4.0f,
                   -0.3f + 0.6f * static_cast<float>(i / 5 % 5) / 4.0f, scale,
                   static_cast<uint8_t>(i * 37 + 11)});
  }
  return pts;
}

}  // namespace slam_refact::map::testing

#endif  // SLAM_REFACT_TESTS_MAP_MAP_TEST_UTIL_H_

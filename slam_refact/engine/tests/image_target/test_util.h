// Shared helpers of the image target tests (US-011, US-012).
//
// The public tests load `testdata/flower-big.jpg`, `amity_up.jpg` and `amity_rotated.jpg`, which
// are git-lfs pointers in the reference checkout (feas-B §4.2, VERIFIED with `file`). The ports
// replace them with SyntheticTarget(): a deterministic textured luminance image of the same size.

#ifndef SLAM_REFACT_TESTS_IMAGE_TARGET_TEST_UTIL_H_
#define SLAM_REFACT_TESTS_IMAGE_TARGET_TEST_UTIL_H_

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <string>

#include "c8/geometry/pixel-pinhole-camera-model.h"
#include "c8/hmatrix.h"
#include "common/c8_adapters.h"
#include "frontend/intrinsics.h"
#include "image_target/target_files.h"

namespace slam_refact::image_target::testing {

// Intrinsics::getCameraIntrinsics(DeviceInfos::GOOGLE_PIXEL3) of the public tests.
inline c8_PixelPinholeCameraModel TestK() {
  return ToC8Intrinsics(
      frontend::CameraIntrinsicsForModel(frontend::device_infos::GOOGLE_PIXEL3));
}

inline PinholeIntrinsics TestPinholeK() {
  return frontend::CameraIntrinsicsForModel(frontend::device_infos::GOOGLE_PIXEL3);
}

// xorshift32: deterministic on every platform (no std distributions).
class TestRng {
 public:
  explicit TestRng(uint32_t seed) : state_(seed == 0 ? 0x9E3779B9u : seed) {}
  uint32_t Next() {
    state_ ^= state_ << 13;
    state_ ^= state_ >> 17;
    state_ ^= state_ << 5;
    return state_;
  }
  int Uniform(int lo, int hi) {  // [lo, hi]
    return lo + static_cast<int>(Next() % static_cast<uint32_t>(hi - lo + 1));
  }

 private:
  uint32_t state_;
};

// Textured luminance image: overlapping rectangles and discs of random gray levels on a gradient,
// sized relative to the image so the content survives the pyramid downscale to 480x640.
inline GrayImage SyntheticTarget(int width, int height, uint32_t seed = 1) {
  GrayImage img;
  img.width = width;
  img.height = height;
  img.pixels.resize(static_cast<size_t>(width) * height);
  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      img.pixels[static_cast<size_t>(y) * width + x] =
          static_cast<uint8_t>(64 + (96 * x) / width + (64 * y) / height);
    }
  }
  TestRng rng(seed);
  const int unit = std::max(2, std::min(width, height) / 160);
  for (int i = 0; i < 900; ++i) {
    const int cx = rng.Uniform(0, width - 1);
    const int cy = rng.Uniform(0, height - 1);
    const int rw = rng.Uniform(2, 12) * unit;
    const int rh = rng.Uniform(2, 12) * unit;
    const uint8_t value = static_cast<uint8_t>(rng.Uniform(0, 255));
    const bool disc = (rng.Next() & 3u) == 0u;
    for (int y = std::max(0, cy - rh); y < std::min(height, cy + rh); ++y) {
      for (int x = std::max(0, cx - rw); x < std::min(width, cx + rw); ++x) {
        if (disc) {
          const float dx = static_cast<float>(x - cx) / rw;
          const float dy = static_cast<float>(y - cy) / rh;
          if (dx * dx + dy * dy > 1.0f) {
            continue;
          }
        }
        img.pixels[static_cast<size_t>(y) * width + x] = value;
      }
    }
  }
  return img;
}

// Pointwise(AreWithin(eps), expected.data()) of the public tests: |a - e| < eps per element.
inline ::testing::AssertionResult MatrixWithin(const c8::HMatrix& actual, const c8::HMatrix& expected,
                                               float eps) {
  for (int i = 0; i < 16; ++i) {
    const float a = actual.data()[i];
    const float e = expected.data()[i];
    if (!(std::fabs(a - e) < eps)) {
      return ::testing::AssertionFailure()
             << "element " << i << ": " << a << " vs " << e << " (eps " << eps << ")";
    }
  }
  return ::testing::AssertionSuccess();
}

}  // namespace slam_refact::image_target::testing

#endif  // SLAM_REFACT_TESTS_IMAGE_TARGET_TEST_UTIL_H_

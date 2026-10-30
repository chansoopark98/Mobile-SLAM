// Standalone probe for the public xr_reference Gr8Cpu path (reality/engine/features/gr8cpu.cc ->
// gr8.cc), which depends on third_party/cvlite (an OpenCV 3.2 fork: FAST, Gaussian filter, resize).
// Runs Gr8Cpu::create() defaults on a fixed synthetic 320x240 Y plane and prints deterministic
// keypoint/descriptor summaries.
#include <cinttypes>
#include <cmath>
#include <cstdio>

#include "c8/pixels/pixels.h"
#include "reality/engine/features/gr8cpu.h"

namespace {

uint64_t Fnv1a(const uint8_t *data, size_t n, uint64_t h) {
  for (size_t i = 0; i < n; ++i) {
    h ^= data[i];
    h *= 1099511628211ull;
  }
  return h;
}

}  // namespace

int main() {
  using namespace c8;  // NOLINT: probe-only translation unit.
  const int cols = 320, rows = 240;
  Vector<uint8_t> img(static_cast<size_t>(cols) * rows);
  for (int y = 0; y < rows; ++y) {
    for (int x = 0; x < cols; ++x) {
      float v = 128.0f + 60.0f * std::sin(x * 0.11f) * std::cos(y * 0.07f)
        + 40.0f * std::sin((x + 2.0f * y) * 0.031f)
        + (((x / 16) + (y / 16)) % 2 ? 20.0f : -20.0f);
      img[static_cast<size_t>(y) * cols + x] =
        static_cast<uint8_t>(std::min(255.0f, std::max(0.0f, v)));
    }
  }
  YPlanePixels y_plane(rows, cols, cols, img.data());
  Gr8Cpu gr8 = Gr8Cpu::create();
  ImagePoints pts = gr8.detectAndCompute(y_plane);
  int per_level[8] = {0};
  uint64_t h = 1469598103934665603ull;
  for (size_t i = 0; i < pts.size(); ++i) {
    per_level[pts[i].location().scale]++;
    const auto &fs = pts[i].features();
    if (fs.orbFeature.has_value()) {
      h = Fnv1a(fs.orbFeature->data(), 32, h);
    }
  }
  std::printf("gr8cpu: __cplusplus=%ld n=%zu per_level=[%d,%d,%d,%d,%d,%d,%d,%d] desc_fnv=%016" PRIx64
    "\n", __cplusplus, pts.size(), per_level[0], per_level[1], per_level[2], per_level[3],
    per_level[4], per_level[5], per_level[6], per_level[7], h);
  for (size_t i = 0; i < std::min<size_t>(5, pts.size()); ++i) {
    const auto loc = pts[i].location();
    std::printf("gr8cpu.kp[%zu] x=%.3f y=%.3f scale=%u size=%.3f angle=%.3f resp=%.6g\n", i,
      loc.pt.x, loc.pt.y, loc.scale, loc.size, loc.angle, loc.response);
  }
  return 0;
}

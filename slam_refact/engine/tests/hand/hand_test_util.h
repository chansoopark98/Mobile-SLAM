// Shared helpers of the hand tests (US-021): golden file readers and the deterministic inputs that the
// python generators (tools/hand/gen_image_golden.py, gen_geometry_golden.py) reproduce bit for bit.
#ifndef SLAM_REFACT_TESTS_HAND_HAND_TEST_UTIL_H_
#define SLAM_REFACT_TESTS_HAND_HAND_TEST_UTIL_H_

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iterator>
#include <sstream>
#include <string>
#include <vector>

#include "hand/hand_types.h"

namespace slam_refact::hand::testutil {

inline std::vector<std::vector<std::string>> ReadTsv(const std::string& dir, const std::string& name) {
  std::ifstream f(dir + "/" + name);
  EXPECT_TRUE(f.good()) << dir << "/" << name;
  std::vector<std::vector<std::string>> rows;
  std::string line;
  while (std::getline(f, line)) {
    if (line.empty() || line[0] == '#') continue;
    std::vector<std::string> cols;
    std::stringstream s(line);
    std::string c;
    while (std::getline(s, c, '\t')) cols.push_back(c);
    rows.push_back(cols);
  }
  return rows;
}

inline bool ReadRaw(const std::string& dir, const std::string& name, size_t bytes, std::vector<uint8_t>* out) {
  std::ifstream f(dir + "/" + name, std::ios::binary);
  out->assign(std::istreambuf_iterator<char>(f), std::istreambuf_iterator<char>());
  return out->size() == bytes;
}

// image(w, h, seed) of gen_image_golden.py.
inline Image3 TestImage(int w, int h, uint32_t seed) {
  std::vector<double> noise(static_cast<size_t>(w) * h * 3);
  uint32_t s = seed;
  for (auto& v : noise) {
    s = s * 1664525u + 1013904223u;
    v = static_cast<double>(s >> 24);
  }
  Image3 img(w, h);
  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w; ++x) {
      const double grad = (x * 255.0 / std::max(1, w - 1) + y * 255.0 / std::max(1, h - 1)) / 2.0;
      for (int c = 0; c < 3; ++c) {
        const double v = 0.5 * noise[(static_cast<size_t>(y) * w + x) * 3 + c] + 0.5 * grad;
        img.at(x, y)[c] = static_cast<uint8_t>(std::min(255.0, std::max(0.0, v)));
      }
    }
  }
  return img;
}

inline int MaxDiff(const Image3& a, const std::vector<uint8_t>& b) {
  int m = 0;
  for (size_t i = 0; i < b.size(); ++i) m = std::max(m, std::abs(static_cast<int>(a.px[i]) - b[i]));
  return m;
}

inline double DifferingPixelFraction(const Image3& a, const std::vector<uint8_t>& b) {
  size_t differing = 0;
  for (size_t p = 0; p < b.size(); p += 3) {
    differing += a.px[p] != b[p] || a.px[p + 1] != b[p + 1] || a.px[p + 2] != b[p + 2];
  }
  return static_cast<double>(differing) / static_cast<double>(b.size() / 3);
}

inline float LcgValue(uint32_t* s) {
  *s = *s * 1664525u + 1013904223u;
  return static_cast<float>(static_cast<int>(*s >> 8) - (1 << 23)) * 9.5367431640625e-07f;  // 2^-20
}

inline std::vector<float> LcgValues(size_t n, uint32_t seed) {
  std::vector<float> v(n);
  uint32_t s = seed;
  for (auto& x : v) x = LcgValue(&s);
  return v;
}

// planted(seed) of gen_geometry_golden.py (the cluster centres come from the case line).
inline void PlantedTensors(uint32_t seed, const int centers[3], std::vector<float>* boxes,
                           std::vector<float>* scores) {
  *boxes = LcgValues(2944 * 18, seed);
  for (auto& v : *boxes) v = v * 4.0f;
  *scores = LcgValues(2944, seed + 1000);
  for (auto& v : *scores) v = v * 0.5f - 4.0f;
  for (int k = 0; k < 3; ++k) {
    for (int j = 0; j < 8; ++j) {
      const int i = centers[k] + j;
      (*scores)[i] = static_cast<float>(2.0 + 0.3 * j);
      (*boxes)[i * 18 + 0] = (*boxes)[i * 18 + 1] = 0.0f;
      (*boxes)[i * 18 + 2] = (*boxes)[i * 18 + 3] = static_cast<float>(60.0 + 3 * j);
      const auto kp = LcgValues(14, seed + 7 * j);
      for (int q = 0; q < 14; ++q) (*boxes)[i * 18 + 4 + q] = kp[q] * 2.0f;
    }
  }
}

}  // namespace slam_refact::hand::testutil

#endif  // SLAM_REFACT_TESTS_HAND_HAND_TEST_UTIL_H_

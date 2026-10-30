// third_party/musl_libm on host builds: sinf/cosf/atan2f/log10f must give the original binary's (musl)
// results, not glibc's. Inputs are arguments where glibc 2.35 rounds the last bit differently; the
// expected bits are the original xr-slam.wasm results (461 sinf, 462 cosf, 843 atan2f, run in
// isolation, build/slam-refact-20261016/precision/) and, for log10f, the emsdk 5.0.0 musl result.
#include <cmath>
#include <cstdint>
#include <cstring>

#include "gtest/gtest.h"

namespace {

float F(uint32_t u) {
  float v;
  std::memcpy(&v, &u, 4);
  return v;
}

uint32_t B(float v) {
  uint32_t u;
  std::memcpy(&u, &v, 4);
  return u;
}

// volatile arguments keep the compiler from folding the calls at compile time.
TEST(MuslLibm, SinCosMatchTheOriginal) {
  volatile uint32_t s = 0x3eb63166u;
  volatile uint32_t c = 0x3fba0b26u;
  EXPECT_EQ(B(std::sin(F(s))), 0x3eb25f46u);  // glibc: 0x3eb25f47
  EXPECT_EQ(B(std::cos(F(c))), 0x3defbe3eu);  // glibc: 0x3defbe3f
  float sv = 0.f, cv = 0.f;
  sincosf(F(c), &sv, &cv);
  EXPECT_EQ(B(cv), 0x3defbe3eu);
}

TEST(MuslLibm, Atan2fMatchesTheOriginal) {
  volatile uint32_t y = 0xbec78280u;
  volatile uint32_t x = 0x3f68a7c4u;
  EXPECT_EQ(B(std::atan2(F(y), F(x))), 0xbecf639cu);  // glibc: 0xbecf639b
  volatile uint32_t y2 = 0x3e7a7678u;
  volatile uint32_t x2 = 0x3f1008f4u;
  EXPECT_EQ(B(std::atan2(F(y2), F(x2))), 0x3ed1f5edu);  // glibc: 0x3ed1f5ee
}

TEST(MuslLibm, Log10fMatchesMusl) {
  volatile uint32_t z = 0x40917477u;
  EXPECT_EQ(B(std::log10(F(z))), 0x3f285710u);  // glibc: 0x3f285711
}

double D(uint64_t u) {
  double v;
  std::memcpy(&v, &u, 8);
  return v;
}

uint64_t B64(double v) {
  uint64_t u;
  std::memcpy(&u, &v, 8);
  return u;
}

// The second set (acosf, logf, expf, pow, sin, cos, sincos): expected bits are the original
// xr-slam.wasm functions run in isolation (993 acosf, 354 logf, 1217 expf, 342 pow, 898 sin,
// 899 cos), which equal emsdk 5.0.0 musl on 200,000 sweep inputs each
// (build/slam-refact-20261016/fix/libm/). The arguments are ones where glibc 2.35 rounds the
// last bit differently, except logf / expf, where the sweep found no glibc difference.
TEST(MuslLibm, AcosfMatchesTheOriginal) {  // map_tracker / frame_frame_tracker last_rotation
  volatile uint32_t a = 0x3f6b1c5au;
  volatile uint32_t b = 0x3f592f36u;
  EXPECT_EQ(B(std::acos(F(a))), 0x3ed044e1u);  // glibc: 0x3ed044e2
  EXPECT_EQ(B(std::acos(F(b))), 0x3f0ed18fu);  // glibc: 0x3f0ed18e
}

TEST(MuslLibm, LogfExpfMatchTheOriginal) {
  volatile uint32_t l = 0x40917477u;
  volatile uint32_t e = 0xc0f1e3a1u;
  EXPECT_EQ(B(std::log(F(l))), 0x3fc1cf07u);  // RANSAC needed-trials (frame_frame_tracker)
  EXPECT_EQ(B(std::exp(F(e))), 0x3a08ad09u);  // metric-scale gaussian kernel
}

TEST(MuslLibm, PowMatchesTheOriginal) {  // kalman fps scaling, Ceres
  volatile uint64_t x1 = 0x4060c1e29c7442e4ull, y1 = 0xbfd917e9d0eb7e60ull;
  volatile uint64_t x2 = 0x4013693534530795ull, y2 = 0x3ffaf5aa82634d08ull;
  EXPECT_EQ(B64(std::pow(D(x1), D(y1))), 0x3fc2c16ccdd7c240ull);  // glibc: ...c241
  EXPECT_EQ(B64(std::pow(D(x2), D(y2))), 0x402ca2a72ed28be4ull);  // glibc: ...be3
}

TEST(MuslLibm, DoubleSinCosMatchTheOriginal) {  // Ceres AngleAxisToRotationMatrix, c8
  volatile uint64_t s = 0x3fed93b1739ed9a2ull;
  volatile uint64_t c = 0x3ff1747032aecff7ull;
  EXPECT_EQ(B64(std::sin(D(s))), 0x3fe98abea1dddac5ull);  // glibc: ...dac6
  EXPECT_EQ(B64(std::cos(D(c))), 0x3fdd8be2ea696e9bull);  // glibc: ...6e9a
  double sv = 0.0, cv = 0.0;
  sincos(D(s), &sv, &cv);  // what g++ turns sin(x) + cos(x) into
  EXPECT_EQ(B64(sv), 0x3fe98abea1dddac5ull);
  EXPECT_EQ(B64(cv), 0x3fe346f1ec884158ull);
}

// pyramid_layout.cc computes 1.0f / std::pow(1.44, i); the original (func 459) has these folded
// to the f32 constants 0x1.638e38p-1, 0x1.edd3cp-2, 0x1.56ef7ep-2 for i = 1, 2, 3.
TEST(MuslLibm, PyramidScalesEqualTheOriginalConstants) {
  const uint32_t expected[4] = {0x3f800000u, 0x3f31c71cu, 0x3ef6e9e0u, 0x3eab77bfu};
  for (int i = 0; i < 4; ++i) {
    volatile int k = i;
    EXPECT_EQ(B(static_cast<float>(1.0f / std::pow(1.44, k))), expected[i]) << i;
  }
}

}  // namespace

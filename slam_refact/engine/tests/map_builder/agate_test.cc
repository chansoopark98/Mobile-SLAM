// Agate global descriptor (US-025 LC-2b, spec/12-loop-close.md §2.5, design/us025-lc2b-agate-port.md).
//
// Tiers:
//   Self       properties that must hold without any original data (basis reproducibility against a
//              test-local libc++ re-statement, normalisation, determinism, empty input).
//   Golden     bit comparison with the original's func 1263 output on recorded keyframes
//              (tests/data/agate/). No tolerance exists: the output is 32 integer bytes.
//
// Compiled only with SLAM_REFACT_LOOP_CLOSE=ON.

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <iostream>
#include <random>
#include <sstream>
#include <string>
#include <vector>

#include "gtest/gtest.h"
#include "map_builder/agate_descriptor.h"

namespace slam_refact::map_builder {
namespace {

uint32_t BitsOf(float f) {
  uint32_t u;
  std::memcpy(&u, &f, sizeof(u));
  return u;
}

// libc++ (LLVM 21) generate_canonical<float, 24> for a 32-bit URNG, restated here independently of
// libcxx_compat: k = 1, R = float(2^32 - 1) + 1 = 2^32, result = float(g()) / R.
float Canonical(std::mt19937& g) {
  const float rp = static_cast<float>(std::mt19937::max() - std::mt19937::min()) + 1.0f;
  const float sp = static_cast<float>(g() - std::mt19937::min());
  return sp / rp;
}

// libc++ normal_distribution<float>::operator() (Marsaglia polar with a cached second value).
class LibcxxNormal {
 public:
  float operator()(std::mt19937& g) {
    float up;
    if (hot_) {
      hot_ = false;
      up = v_;
    } else {
      float u, v, s;
      do {
        u = 2.0f * Canonical(g) + -1.0f;  // uniform_real_distribution<float>(-1, 1)
        v = 2.0f * Canonical(g) + -1.0f;
        s = u * u + v * v;
      } while (s > 1 || s == 0);
      const float fp = std::sqrt(-2 * std::log(s) / s);
      v_ = v * fp;
      hot_ = true;
      up = u * fp;
    }
    return up * 1.0f + 0.0f;
  }

 private:
  float v_ = 0.0f;
  bool hot_ = false;
};

std::vector<AgateOrb> RandomOrbs(uint32_t seed, size_t n) {
  std::mt19937 g(seed);
  std::vector<AgateOrb> v(n);
  for (AgateOrb& d : v) {
    for (uint8_t& b : d) b = static_cast<uint8_t>(g());
  }
  return v;
}

const AgateDescriptor& Extractor() {
  static const AgateDescriptor* extractor = new AgateDescriptor();
  return *extractor;
}

TEST(AgateSelf, RandomBasisMatchesLibcxxRestatement) {
  const std::vector<float>& basis = Extractor().basis();
  ASSERT_EQ(basis.size(), kAgatePcaDims * kAgateProjectionBits);
  std::mt19937 g(0);
  LibcxxNormal dist;
  for (size_t i = 0; i < basis.size(); ++i) {
    const float want = dist(g);
    ASSERT_EQ(BitsOf(basis[i]), BitsOf(want)) << "basis element " << i;
  }
  // The stream is a standard normal: sanity of the moments over 8192 draws.
  double sum = 0, sq = 0;
  for (float f : basis) {
    sum += f;
    sq += static_cast<double>(f) * f;
  }
  EXPECT_NEAR(sum / basis.size(), 0.0, 0.05);
  EXPECT_NEAR(sq / basis.size(), 1.0, 0.05);
}

TEST(AgateSelf, FirstMt19937OutputsAreTheStandardValues) {
  // C++ standard [rand.predef]: the 10000th output of a default-seeded mt19937 is 4123659995;
  // seed 0 first output 2357136044 (public reference value).
  std::mt19937 g(0);
  EXPECT_EQ(g(), 2357136044u);
}

TEST(AgateSelf, NormalisationInvariants) {
  AgateTrace tr;
  const std::vector<AgateOrb> orb = RandomOrbs(7, 300);
  const AgateGlobal out = Extractor().Extract(orb, &tr);
  ASSERT_EQ(tr.histogram.size(), kAgateHistogramSize);
  ASSERT_EQ(tr.pca.size(), kAgatePcaDims);
  ASSERT_EQ(tr.random_projection.size(), kAgateProjectionBits);
  double h2 = 0, p2 = 0;
  for (float f : tr.histogram) {
    ASSERT_TRUE(std::isfinite(f));
    h2 += static_cast<double>(f) * f;
  }
  for (float f : tr.pca) p2 += static_cast<double>(f) * f;
  EXPECT_NEAR(std::sqrt(h2), 1.0, 1e-4);
  EXPECT_NEAR(std::sqrt(p2), 1.0, 1e-5);
  // The output bits are exactly the signs of the projection.
  for (size_t p = 0; p < kAgateOrbBytes; ++p) {
    for (size_t b = 0; b < 8; ++b) {
      EXPECT_EQ((out[p] >> b) & 1, tr.random_projection[8 * p + b] > 0.0f ? 1 : 0) << "byte " << p << " bit " << b;
    }
  }
}

TEST(AgateSelf, DeterministicAndInputDependent) {
  const std::vector<AgateOrb> a = RandomOrbs(1, 120);
  const std::vector<AgateOrb> b = RandomOrbs(2, 120);
  const AgateGlobal ra1 = Extractor().Extract(a);
  const AgateGlobal ra2 = AgateDescriptor().Extract(a);  // a fresh extractor rebuilds the same basis
  const AgateGlobal rb = Extractor().Extract(b);
  EXPECT_EQ(ra1, ra2);
  EXPECT_NE(ra1, rb);
}

TEST(AgateSelf, EmptyInputIsFiniteAndStable) {
  AgateTrace tr;
  const AgateGlobal e1 = Extractor().Extract(nullptr, 0, &tr);
  for (float f : tr.histogram) ASSERT_TRUE(std::isfinite(f));
  for (float f : tr.pca) ASSERT_TRUE(std::isfinite(f));
  EXPECT_EQ(e1, Extractor().Extract(std::vector<AgateOrb>{}));
}

// ---- Golden --------------------------------------------------------------------------------------

std::string ReadFile(const std::string& path) {
  std::ifstream f(path, std::ios::binary);
  EXPECT_TRUE(static_cast<bool>(f)) << "missing golden file " << path;
  std::ostringstream ss;
  ss << f.rdbuf();
  return ss.str();
}

std::string DataPath(const std::string& name) { return std::string(SR_TEST_DATA_DIR) + "/agate/" + name; }

std::string Hex(const AgateGlobal& g) {
  static const char kDigits[] = "0123456789abcdef";
  std::string s;
  for (uint8_t b : g) {
    s += kDigits[b >> 4];
    s += kDigits[b & 15];
  }
  return s;
}

// One golden record: ORB store slice -> the 32 bytes the original wrote.
struct Record {
  std::string label;
  size_t n;
  size_t off;
  std::string want_hex;
};

// Compares and, on a mismatch, names the first differing byte/bit and the projection values there
// (a value near zero points at a rounding difference, a large one at a wrong stage).
void ExpectBitExact(const std::string& blob, const Record& r) {
  ASSERT_LE(r.off + 32 * r.n, blob.size()) << r.label;
  AgateTrace tr;
  const AgateGlobal got = Extractor().Extract(
      r.n == 0 ? nullptr : reinterpret_cast<const uint8_t*>(blob.data()) + r.off, r.n, &tr);
  const std::string got_hex = Hex(got);
  if (got_hex == r.want_hex) return;
  std::ostringstream why;
  why << r.label << " (n=" << r.n << ") differs\n  want " << r.want_hex << "\n  got  " << got_hex;
  int bits = 0;
  for (size_t p = 0; p < kAgateOrbBytes; ++p) {
    const unsigned want = static_cast<unsigned>(std::stoul(r.want_hex.substr(2 * p, 2), nullptr, 16));
    for (size_t b = 0; b < 8; ++b) {
      if (((want ^ got[p]) >> b) & 1) {
        if (bits++ < 8) {
          why << "\n  bit " << (8 * p + b) << ": original " << ((want >> b) & 1) << ", projection "
              << tr.random_projection[8 * p + b] << " (bits 0x" << std::hex << BitsOf(tr.random_projection[8 * p + b])
              << std::dec << ")";
        }
      }
    }
  }
  ADD_FAILURE() << why.str() << "\n  differing bits: " << bits;
}

std::vector<Record> ReadTsv(const std::string& path, bool with_k) {
  std::vector<Record> out;
  std::istringstream in(ReadFile(path));
  std::string line;
  while (std::getline(in, line)) {
    if (line.empty() || line[0] == '#') continue;
    std::vector<std::string> f;
    std::istringstream ls(line);
    for (std::string c; std::getline(ls, c, '\t');) f.push_back(c);
    Record r;
    if (with_k) {  // k  orb_n  orb_off  orb_sha256  agate_hex
      EXPECT_EQ(f.size(), 5u) << line;
      r = {"keyframe " + f[0], std::stoul(f[1]), std::stoul(f[2]), f[4]};
    } else {       // name  n  off  agate_hex
      EXPECT_EQ(f.size(), 4u) << line;
      r = {f[0], std::stoul(f[1]), std::stoul(f[2]), f[3]};
    }
    out.push_back(r);
  }
  return out;
}

void RunGa(const std::string& run, size_t want_records) {
  const std::string blob = ReadFile(DataPath("ga-" + run + ".bin"));
  const std::vector<Record> recs = ReadTsv(DataPath("ga-" + run + ".tsv"), true);
  ASSERT_EQ(recs.size(), want_records);
  size_t empty = 0;
  for (const Record& r : recs) {
    SCOPED_TRACE(run);
    ExpectBitExact(blob, r);
    empty += r.n == 0;
  }
  ::testing::Test::RecordProperty("empty_orb_keyframes", static_cast<int>(empty));
}

// G-A: keyframes of real original runs (loop close OFF), ORB store -> 32 B written by func 1263.
TEST(AgateGolden, GaFlip6S2) { RunGa("flip6-s2-ga-s2", 117); }
TEST(AgateGolden, GaFlip6S3LowTex) { RunGa("flip6-s3-lowtex-ga-s3", 24); }
TEST(AgateGolden, GaTumRoom1) { RunGa("tum-room1-full-ga-room1", 46); }

// G-B: func 1263 called alone on synthetic inputs (empty, 1..1000 descriptors, all-zero, all-ones,
// duplicates, single-bit).
TEST(AgateGolden, GbSynthetic) {
  const std::string blob = ReadFile(DataPath("gb-synthetic.bin"));
  const std::vector<Record> recs = ReadTsv(DataPath("gb-synthetic.tsv"), false);
  ASSERT_EQ(recs.size(), 20u);
  for (const Record& r : recs) ExpectBitExact(blob, r);
}

// Information, not a gate: how close the recorded bits are to flipping, and whether another
// summation order of the 256x32 product would already have flipped one of them. A small margin
// means the golden pins the accumulation order of the product; a large one means it does not.
TEST(AgateGolden, ReportProjectionMargin) {
  double min_margin = 1e30;
  size_t bits = 0, reversed_flips = 0, fma_flips = 0, double_flips = 0;
  for (const char* run : {"flip6-s2-ga-s2", "flip6-s3-lowtex-ga-s3"}) {
    const std::string blob = ReadFile(DataPath(std::string("ga-") + run + ".bin"));
    for (const Record& r : ReadTsv(DataPath(std::string("ga-") + run + ".tsv"), true)) {
      if (r.n == 0) continue;
      AgateTrace tr;
      Extractor().Extract(reinterpret_cast<const uint8_t*>(blob.data()) + r.off, r.n, &tr);
      const std::vector<float>& a = Extractor().basis();
      for (size_t i = 0; i < kAgateProjectionBits; ++i) {
        float rev = 0.0f, fm = 0.0f;
        double dbl = 0.0;
        for (size_t j = 0; j < kAgatePcaDims; ++j) {
          rev = rev + a[(kAgatePcaDims - 1 - j) * kAgateProjectionBits + i] * tr.pca[kAgatePcaDims - 1 - j];
          fm = std::fma(a[j * kAgateProjectionBits + i], tr.pca[j], fm);
          dbl += static_cast<double>(a[j * kAgateProjectionBits + i]) * tr.pca[j];
        }
        const float v = tr.random_projection[i];
        min_margin = std::min(min_margin, static_cast<double>(std::fabs(v)));
        reversed_flips += (rev > 0.0f) != (v > 0.0f);
        fma_flips += (fm > 0.0f) != (v > 0.0f);
        double_flips += (dbl > 0.0) != (v > 0.0f);
        ++bits;
      }
    }
  }
  std::cout << "[agate-margin] bits=" << bits << " min|projection|=" << min_margin << " flips: reversed-order="
            << reversed_flips << " fma=" << fma_flips << " double=" << double_flips << std::endl;
  EXPECT_GT(bits, 0u);
}

}  // namespace
}  // namespace slam_refact::map_builder

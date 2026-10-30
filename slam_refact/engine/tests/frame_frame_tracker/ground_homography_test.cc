// V-FFT-ISO and V-FFT (d): RobustGroundHomography (1279) against the original-binary golden
// (WP2-PRE probe-fft-1279-r3, 18 cases, seed 5489, one generator in call order) and the RNG
// vectors of rng_model.mjs (VERIFIED bit-equal to the binary, WP2-PRE §1.2-1.3).
// Pre-registered criteria (architecture.md §6.6.4): sample selection and generator state
// identical, ret identical, H and camMotion elements |d| <= 1e-5, mask identical (all 18 cases,
// including the noise-free cases 1, 2, 13; D-23).

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <random>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "c8/geometry/homography.h"
#include "compat/libcxx_algorithms.h"
#include "compat/random_numbers.h"
#include "frame_frame_tracker/frame_frame_tracker_internal.h"
#include "frame_frame_tracker/ground_homography.h"
#include "json_lite.h"

namespace slam_refact::frame_frame_tracker {
namespace {

using slam_refact::testing::JsonValue;
using slam_refact::testing::ReadJson;

const std::string kDataDir = std::string(SR_TEST_DATA_DIR) + "/frame-frame-tracker";

c8::Quaternion Quat(const JsonValue& v) {  // [x, y, z, w]
  return c8::Quaternion(v[3].AsFloat(), v[0].AsFloat(), v[1].AsFloat(), v[2].AsFloat());
}

std::vector<c8::HPoint2> Rays(const JsonValue& v) {
  std::vector<c8::HPoint2> out;
  for (size_t i = 0; i < v.size(); ++i) {
    out.emplace_back(std::array<float, 3>{v[i][0].AsFloat(), v[i][1].AsFloat(), v[i][2].AsFloat()});
  }
  return out;
}

double MaxMatErr(const c8::HMatrix& m, const JsonValue& golden) {
  double e = 0.0;
  for (int r = 0; r < 4; ++r) {
    for (int c = 0; c < 4; ++c) {
      e = std::max(e, std::fabs(static_cast<double>(m(r, c)) - golden[r][c].AsDouble()));
    }
  }
  return e;
}

// Raw generator draws consumed: compares the next output with a reference mt19937(5489).
bool NextDrawMatches(compat::RandomNumbers* rng, uint64_t consumed) {
  std::mt19937 ref(compat::kEngineRngSeed);
  ref.discard(consumed);
  return rng->nextUnsignedInt() == ref();
}

class GroundHomographyGolden : public ::testing::Test {
 protected:
  void SetUp() override {
    ASSERT_TRUE(ReadJson(kDataDir + "/probe-fft-1279-r3/golden-1279.json", &golden_));
    ASSERT_TRUE(ReadJson(kDataDir + "/probe-fft-1279-r3/cases.json", &cases_));
    ASSERT_EQ(golden_.size(), 18u);
    ASSERT_EQ(cases_.size(), 18u);
  }

  struct Out {
    bool ret = false;
    c8::HMatrix h = c8::HMatrixGen::i();
    c8::HMatrix cam = c8::HMatrixGen::i();
    std::vector<uint8_t> mask;
  };

  Out Run(size_t k, compat::RandomNumbers* rng, c8::RobustHomographyScratchSpace* scratch) {
    const JsonValue& g = golden_[k];
    Out o;
    o.ret = RobustGroundHomography(Quat(g["qA"]), Rays(g["A"]), Quat(g["qB"]), Rays(g["B"]),
                                   scratch, rng, &o.h, &o.cam, &o.mask);
    return o;
  }

  void ExpectOutputs(size_t k, const Out& o) {
    const JsonValue& g = golden_[k];
    EXPECT_EQ(o.ret, g["ret"].AsInt() != 0) << "case " << k;
    EXPECT_LE(MaxMatErr(o.h, g["H"]), 1e-5) << "case " << k;
    EXPECT_LE(MaxMatErr(o.cam, g["cam"]), 1e-5) << "case " << k;
    ASSERT_EQ(o.mask.size(), g["mask"].size()) << "case " << k;
    size_t diff = 0;
    for (size_t i = 0; i < o.mask.size(); ++i) {
      diff += (o.mask[i] != 0) != (g["mask"][i].AsInt() != 0) ? 1 : 0;
    }
    EXPECT_EQ(diff, 0u) << "case " << k << " (" << cases_[k]["kind"].str << ")";
  }

  JsonValue golden_;
  JsonValue cases_;
};

// V-FFT-ISO: one generator, the 18 calls in golden order.
TEST_F(GroundHomographyGolden, SequentialEighteenCasesMatchOriginal) {
  compat::RandomNumbers rng(compat::kEngineRngSeed);
  c8::RobustHomographyScratchSpace scratch;
  uint64_t consumed = 0;
  for (size_t k = 0; k < golden_.size(); ++k) {
    EXPECT_EQ(consumed % 624, static_cast<uint64_t>(golden_[k]["mtIndexBefore"].AsInt()))
        << "case " << k;
    ExpectOutputs(k, Run(k, &rng, &scratch));
    consumed += static_cast<uint64_t>(cases_[k]["rawDraws"].AsInt());
  }
  EXPECT_TRUE(NextDrawMatches(&rng, consumed));
}

// Sample selection and generator consumption per case (fresh generator advanced to the case's
// position in the original call sequence).
TEST_F(GroundHomographyGolden, PerCaseSamplingAndDrawCount) {
  uint64_t before = 0;
  for (size_t k = 0; k < golden_.size(); ++k) {
    compat::RandomNumbers rng(compat::kEngineRngSeed);
    for (uint64_t d = 0; d < before; ++d) {
      rng.nextUnsignedInt();
    }
    c8::RobustHomographyScratchSpace scratch;
    ExpectOutputs(k, Run(k, &rng, &scratch));
    const JsonValue& sampling = cases_[k]["sampling"];
    const uint64_t raw = static_cast<uint64_t>(cases_[k]["rawDraws"].AsInt());
    if (golden_[k]["n"].AsInt() >= 15) {
      ASSERT_EQ(scratch.hSampleInds.size(), static_cast<size_t>(sampling["nPairs"].AsInt()))
          << "case " << k;
      const JsonValue& first = sampling["firstPairs"];
      for (size_t i = 0; i < first.size(); ++i) {
        EXPECT_EQ(scratch.hSampleInds[i].first, static_cast<size_t>(first[i][0].AsInt()));
        EXPECT_EQ(scratch.hSampleInds[i].second, static_cast<size_t>(first[i][1].AsInt()));
      }
    }
    EXPECT_TRUE(NextDrawMatches(&rng, before + raw)) << "case " << k;
    before += raw;
  }
}

// V-FFT (d): pair lists, point samples and draw counts of rng_model.mjs groundCall (fresh
// generator per n) and of the 1266 depth pairs (one generator, m = 10 then 12).
TEST(GroundHomographyRng, PairAndSampleOrderMatchesModel) {
  JsonValue v;
  ASSERT_TRUE(ReadJson(kDataDir + "/rng-vectors.json", &v));
  const JsonValue& calls = v["ground_calls"];
  ASSERT_EQ(calls.size(), 9u);
  for (size_t c = 0; c < calls.size(); ++c) {
    const JsonValue& call = calls[c];
    const size_t n = static_cast<size_t>(call["n"].AsInt());
    // Repeated rays: every pair hypothesis degenerates to identity, which does not explain the
    // shifted current rays, so the call is rejected and only the sampling is exercised.
    const std::vector<c8::HPoint2> rays(n, c8::HPoint2(0.0f, -0.5f));
    const std::vector<c8::HPoint2> shifted(n, c8::HPoint2(0.3f, -0.5f));
    compat::RandomNumbers rng(compat::kEngineRngSeed);
    c8::RobustHomographyScratchSpace scratch;
    c8::HMatrix h = c8::HMatrixGen::i();
    c8::HMatrix cam = c8::HMatrixGen::i();
    std::vector<uint8_t> mask;
    EXPECT_FALSE(RobustGroundHomography(c8::Quaternion(), rays, c8::Quaternion(), shifted,
                                        &scratch, &rng, &h, &cam, &mask));
    const JsonValue& pairs = call["pairs"];
    if (n >= kMinGroundHomographyPoints) {
      ASSERT_EQ(scratch.hSampleInds.size(), pairs.size()) << "n " << n;
      for (size_t i = 0; i < pairs.size(); ++i) {
        EXPECT_EQ(scratch.hSampleInds[i].first, static_cast<size_t>(pairs[i][0].AsInt()));
        EXPECT_EQ(scratch.hSampleInds[i].second, static_cast<size_t>(pairs[i][1].AsInt()));
      }
      ASSERT_EQ(scratch.aPtsSample.size(), static_cast<size_t>(call["n_samples"].AsInt()));
      const JsonValue& first = call["samples_first10"];
      for (size_t i = 0; i < first.size(); ++i) {
        const size_t idx = static_cast<size_t>(first[i].AsInt());
        if (n > kMaxGroundHomographySamples) {
          EXPECT_EQ(scratch.ptSampleInds[i], idx) << "n " << n;
        } else {
          EXPECT_EQ(idx, i);
        }
      }
    }
    EXPECT_TRUE(NextDrawMatches(&rng, static_cast<uint64_t>(call["raw_draws"].AsInt())))
        << "n " << n;
  }

  const JsonValue& depth = v["fft_depth_pairs"];
  compat::RandomNumbers rng(compat::kEngineRngSeed);
  uint64_t consumed = 0;
  for (size_t c = 0; c < depth.size(); ++c) {
    const uint32_t m = static_cast<uint32_t>(depth[c]["n"].AsInt());
    const auto pairs = ShuffledDepthPairs(m, &rng);
    ASSERT_EQ(pairs.size(), static_cast<size_t>(depth[c]["count"].AsInt()));
    const JsonValue& first = depth[c]["first12"];
    for (size_t i = 0; i < first.size(); ++i) {
      EXPECT_EQ(pairs[i].first, static_cast<uint32_t>(first[i][0].AsInt())) << "m " << m;
      EXPECT_EQ(pairs[i].second, static_cast<uint32_t>(first[i][1].AsInt())) << "m " << m;
    }
    consumed += static_cast<uint64_t>(depth[c]["raw_draws"].AsInt());
  }
  EXPECT_TRUE(NextDrawMatches(&rng, consumed));
}

// 1279 rejection mask handling (WP2-PRE §2 / G1279 tail).
TEST(GroundHomographyMask, RejectionResizesOrClears) {
  compat::RandomNumbers rng(compat::kEngineRngSeed);
  c8::RobustHomographyScratchSpace scratch;
  c8::HMatrix h = c8::HMatrixGen::i();
  c8::HMatrix cam = c8::HMatrixGen::i();
  // n < 15: resized to n; existing elements are kept, new ones are 0.
  std::vector<uint8_t> mask = {1, 1, 1};
  const std::vector<c8::HPoint2> few(5, c8::HPoint2(0.0f, -0.5f));
  EXPECT_FALSE(RobustGroundHomography(c8::Quaternion(), few, c8::Quaternion(), few, &scratch, &rng,
                                      &h, &cam, &mask));
  EXPECT_EQ(mask, std::vector<uint8_t>({1, 1, 1, 0, 0}));
  mask.assign(8, 1);
  EXPECT_FALSE(RobustGroundHomography(c8::Quaternion(), few, c8::Quaternion(), few, &scratch, &rng,
                                      &h, &cam, &mask));
  EXPECT_EQ(mask, std::vector<uint8_t>(5, 1));
  // n >= 15 rejected: mask cleared, H = camMotion = I.
  const std::vector<c8::HPoint2> same(20, c8::HPoint2(0.0f, -0.5f));
  const std::vector<c8::HPoint2> shifted(20, c8::HPoint2(0.3f, -0.5f));
  EXPECT_FALSE(RobustGroundHomography(c8::Quaternion(), same, c8::Quaternion(), shifted, &scratch,
                                      &rng, &h, &cam, &mask));
  EXPECT_TRUE(mask.empty());
  for (int r = 0; r < 4; ++r) {
    for (int c = 0; c < 4; ++c) {
      EXPECT_EQ(h(r, c), r == c ? 1.0f : 0.0f);
      EXPECT_EQ(cam(r, c), r == c ? 1.0f : 0.0f);
    }
  }
}

// fit-pairs (ground_homography.cc, the nth_element call) consumes only sampleResiduals[k_idx],
// and the vector is cleared before the next sample. The engine links the emsdk 5.0.0 libc++
// nth_element, the original the libc++ 14 one; the two place the elements other than k
// differently on some inputs (8 of the 56 compat golden cases). For a strict weak order the
// k-th value is the k-th order statistic whatever the algorithm, so the score is the same bits.
// The residuals are dx * dx + dy * dy: never -0.0f (the only value that compares equal to a
// different bit pattern) and NaN only for a NaN ray, which flatten() does not produce for the
// finite, z != 0 rays the tracker passes. Pins: [k] == sorted[k] bit for bit, for every input
// order, with the k_idx rule of the call site.
TEST(GroundHomographyKthResidual, OnlyTheKthOrderStatisticIsConsumed) {
  std::mt19937 gen(5489u);
  std::uniform_real_distribution<float> coord(-0.05f, 0.05f);
  std::uniform_int_distribution<int> dup(0, 3);
  for (int trial = 0; trial < 400; ++trial) {
    const size_t num_samples = 1 + static_cast<size_t>(trial % 64);
    std::vector<float> residuals;
    for (size_t s = 0; s < num_samples; ++s) {
      if (s > 0 && dup(gen) == 0) {  // exact ties, as rays shared by several samples give
        residuals.push_back(residuals[s - 1]);
        continue;
      }
      const float dx = coord(gen);
      const float dy = trial % 7 == 0 ? 0.0f : coord(gen);
      residuals.push_back(dx * dx + dy * dy);
    }
    const size_t floor_frac =
        static_cast<size_t>(static_cast<float>(num_samples) * kGroundHomographyKFraction);
    size_t k_idx = num_samples - 1;
    if (k_idx > 3) {
      k_idx = 4;
    }
    if (floor_frac > 4) {
      k_idx = floor_frac;
    }
    std::vector<float> sorted = residuals;
    std::sort(sorted.begin(), sorted.end());
    for (int perm = 0; perm < 4; ++perm) {
      std::vector<float> v = residuals;
      if (perm == 1) std::reverse(v.begin(), v.end());
      if (perm == 2) std::sort(v.begin(), v.end());
      if (perm == 3) std::shuffle(v.begin(), v.end(), gen);
      slam_refact::libcxx::nth_element(v.begin(), v.begin() + k_idx, v.end());
      uint32_t got = 0;
      uint32_t want = 0;
      std::memcpy(&got, &v[k_idx], 4);
      std::memcpy(&want, &sorted[k_idx], 4);
      ASSERT_EQ(got, want) << "trial " << trial << " perm " << perm << " n " << num_samples
                           << " k " << k_idx;
    }
  }
}

}  // namespace
}  // namespace slam_refact::frame_frame_tracker

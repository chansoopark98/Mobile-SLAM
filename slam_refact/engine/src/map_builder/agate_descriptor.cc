// Agate global keyframe descriptor (US-025 LC-2b). See agate_descriptor.h for the numeric contract.
//
// Source of each stage (public reference runtime archive, reality/engine/features/):
//   Histogram()   agate-histogram.cc  AgateHistogram<OrbFeature>::extract
//   Pca()         binary-pca.cc       BinaryPca::projectAndWhiten + agate-pca.cc (l2Normalize)
//   RandomBasis   random-basis.cc     RandomBasis<OrbFeature>(32) ctor and projectAndBinarize
//   Extract()     agate-descriptor.cc AgateDescriptor::extract
// Intermediate roundings are written out the same way as in the sources (see the comments).

#include "map_builder/agate_descriptor.h"

#include <cmath>
#include <cstring>
#include <limits>
#include <random>
#include <utility>

#include "compat/libcxx_algorithms.h"

#ifdef SLAM_REFACT_LOOP_CLOSE

// Compile-time guards for the float contract of agate_descriptor.h. The golden (207 outputs of 256
// sign bits) cannot tell fused or reassociated arithmetic from plain arithmetic, so the build must
// not allow it: no fast-math, no FMA-capable target (the compiler would contract a*b+c on its own
// when -ffp-contract is not off), and float expressions evaluated in float.
#if defined(__FAST_MATH__) || (defined(__FINITE_MATH_ONLY__) && __FINITE_MATH_ONLY__)
#error "agate_descriptor.cc must be built without -ffast-math (bit-exact float contract)"
#endif
#if defined(__FP_FAST_FMA) || defined(__FP_FAST_FMAF)
#error "agate_descriptor.cc must be built for a target without FMA (e.g. -march=x86-64); see SlamRefactFlags.cmake"
#endif
#if defined(__FLT_EVAL_METHOD__) && __FLT_EVAL_METHOD__ != 0
#error "agate_descriptor.cc needs FLT_EVAL_METHOD == 0 (float expressions evaluated in float)"
#endif

namespace slam_refact::map_builder {

namespace {

#include "map_builder/agate_dictionary64_n0.inc"
#include "map_builder/agate_pca32.inc"

static_assert(kAgateDictionaryDescriptorSize == kAgateOrbBytes, "dictionary entry size");
static_assert(kAgateDictionaryFileEntries == kAgateDictionaryEntries,
              "dictionary entry count");
static_assert(kAgatePcaBasisVectorSize == kAgateHistogramSize, "PCA input size");
static_assert(kAgatePcaWhiteningSize == kAgatePcaDims, "PCA output size");

constexpr size_t kPcaBlocks = kAgatePcaBasisVectorSize / 64;                // 256 uint64 per basis vector
static_assert(sizeof(kAgatePcaBasis) / sizeof(uint64_t) == kAgatePcaDims * kPcaBlocks, "PCA basis size");

float BitsToFloat(uint32_t bits) {
  float f;
  std::memcpy(&f, &bits, sizeof(f));
  return f;
}

uint64_t Load64(const uint8_t* p) {
  uint64_t v;
  std::memcpy(&v, p, sizeof(v));
  return v;
}

// OrbFeature::hammingDistance: popcount of the xor, four uint64 blocks.
int Hamming(const uint8_t* a, const uint8_t* b) {
  int dist = 0;
  for (size_t k = 0; k < kAgateOrbBytes; k += 8) {
    dist += __builtin_popcountll(Load64(a + k) ^ Load64(b + k));
  }
  return dist;
}

// FloatVector::l2Norm: std::accumulate, one float accumulator, element order.
float L2Norm(const float* v, size_t n) {
  float acc = 0.0f;
  for (size_t i = 0; i < n; ++i) {
    acc = acc + (v[i] * v[i]);
  }
  return std::sqrt(acc);
}

// AgateHistogram<OrbFeature>::extract.
void Histogram(const uint8_t* orb, size_t n, std::vector<float>* histogram) {
  constexpr size_t kBits = 8 * kAgateOrbBytes;  // localDescriptorBits
  // Four 16-bit counters per uint64 (bit-exact copy of the original packing, including carries).
  std::vector<uint64_t> counter(kAgateDictionaryEntries * kBits / 4, 0);
  std::vector<int> assignments(n);
  for (size_t i = 0; i < n; ++i) {
    int closest = std::numeric_limits<int>::max();
    for (size_t j = 0; j < kAgateDictionaryEntries; ++j) {
      const int d = Hamming(orb + i * kAgateOrbBytes, kAgateDictionary64N0 + j * kAgateOrbBytes);
      if (d < closest) {
        closest = d;
        assignments[i] = static_cast<int>(j);
      }
    }
  }
  std::vector<int> index_counts(kAgateDictionaryEntries, 0);
  for (size_t i = 0; i < n; ++i) {
    index_counts[assignments[i]]++;
  }
  for (size_t i = 0; i < n; ++i) {
    uint64_t* iter = counter.data() + ((static_cast<size_t>(assignments[i]) * kBits) >> 2);
    for (size_t p = 0; p < kAgateOrbBytes; ++p) {
      const uint64_t data = orb[i * kAgateOrbBytes + p];
      iter[0] += (data & 0b1ull) | ((data & 0b10ull) << 15) | ((data & 0b100ull) << 30) |
                 ((data & 0b1000ull) << 45);
      iter[1] += ((data & 0b10000ull) >> 4) | ((data & 0b100000ull) << 11) |
                 ((data & 0b1000000ull) << 26) | ((data & 0b10000000ull) << 41);
      iter += 2;
    }
  }
  histogram->assign(kAgateHistogramSize, 0.0f);
  float* h = histogram->data();
  for (size_t i = 0; i < counter.size(); ++i) {
    const uint64_t data = counter[i];
    for (size_t j = 0; j < 4; ++j) {
      h[(i << 2) + j] = static_cast<float>((data >> (j * 16)) & 0xFFFFull);
    }
  }
  for (size_t i = 0; i < kAgateDictionaryEntries; ++i) {
    float* iter = h + i * kBits;
    const uint8_t* match = kAgateDictionary64N0 + i * kAgateOrbBytes;
    // Scale the segment by the count of dictionary term matches.
    const float scale = 1.0f / (0.00001f + static_cast<float>(index_counts[i]));
    for (size_t j = 0; j < kBits; ++j) {
      iter[j] *= scale;
    }
    // Subtract the cluster centre; accumulate the squared norm in the same loop.
    float cluster_scale = 0.0f;
    for (size_t p = 0; p < kAgateOrbBytes; ++p) {
      const size_t bit_start = p << 3;
      const int data = match[p];
      for (size_t b = 0; b < 8; ++b) {
        float& item = iter[bit_start + b];
        item -= static_cast<float>((data >> b) & 0b1);
        cluster_scale += item * item;
      }
    }
    // L2 normalise the segment with regularisation.
    cluster_scale = 1.0f / (0.00001f + std::sqrt(cluster_scale));
    for (size_t j = 0; j < kBits; ++j) {
      iter[j] *= cluster_scale;
    }
  }
  // `histogram_ *= 1.0f / (0.00001 + l2Norm(histogram_))`: the sum and the quotient are double
  // (0.00001 is a double literal); the product operator takes a float, so it rounds once here.
  const double full = 1.0f / (0.00001 + static_cast<double>(L2Norm(h, kAgateHistogramSize)));
  const float full_scale = static_cast<float>(full);
  for (size_t i = 0; i < kAgateHistogramSize; ++i) {
    h[i] *= full_scale;
  }
}

// BinaryPca::projectAndWhiten (32 dimensions) followed by FloatVector::l2Normalize.
void Pca(const std::vector<float>& histogram, std::vector<float>* out) {
  uint64_t input64[kPcaBlocks];
  for (size_t b = 0; b < kPcaBlocks; ++b) {
    uint64_t block = 0;
    for (int i = 63; i >= 0; --i) {
      const size_t idx = b * 64 + static_cast<size_t>(i);
      const float shifted = histogram[idx] + BitsToFloat(kAgatePcaTranslationLutBits[kAgatePcaTranslationIdx[idx]]);
      block = (block << 1) | static_cast<uint64_t>(0.0f < shifted);
    }
    input64[b] = block;
  }
  const int acc_start = static_cast<int>(kAgatePcaBasisVectorSize >> 1);
  out->assign(kAgatePcaDims, 0.0f);
  for (size_t i = 0; i < kAgatePcaDims; ++i) {
    int acc = acc_start;
    const uint64_t* basis = kAgatePcaBasis + i * kPcaBlocks;
    for (size_t b = 0; b < kPcaBlocks; ++b) {
      acc -= __builtin_popcountll(basis[b] ^ input64[b]);
    }
    (*out)[i] = static_cast<float>(acc);
  }
  for (size_t i = 0; i < kAgatePcaDims; ++i) {
    (*out)[i] *= BitsToFloat(kAgatePcaWhiteningBits[i]);
  }
  const float scale = 1.0f / L2Norm(out->data(), out->size());
  for (float& v : *out) {
    v *= scale;
  }
}

}  // namespace

AgateDescriptor::AgateDescriptor() {
  // RandomBasis<OrbFeature>(32): seeded with 0, normal_distribution<float>(0, 1), filled in order.
  std::mt19937 rnd(0);
  slam_refact::libcxx::normal_distribution<float> distribution(0.0f, 1.0f);
  basis_.resize(kAgatePcaDims * kAgateProjectionBits);
  for (float& item : basis_) {
    item = distribution(rnd);
  }
}

AgateGlobal AgateDescriptor::Extract(const uint8_t* orb, size_t n, AgateTrace* trace) const {
  std::vector<float> histogram;
  Histogram(orb, n, &histogram);
  std::vector<float> pca;
  Pca(histogram, &pca);

  // Y = A X with A the 256 x 32 column-major basis. Eigen's column-major gemv accumulates each row
  // over the columns in order from 0 (separate multiply and add).
  std::vector<float> projection(kAgateProjectionBits, 0.0f);
  for (size_t i = 0; i < kAgateProjectionBits; ++i) {
    float c = 0.0f;
    for (size_t j = 0; j < kAgatePcaDims; ++j) {
      c = c + basis_[j * kAgateProjectionBits + i] * pca[j];
    }
    projection[i] = c;
  }
  AgateGlobal result;
  for (size_t p = 0; p < kAgateOrbBytes; ++p) {
    const size_t bit_start = p << 3;
    uint8_t byte = 0;
    for (int b = 7; b >= 0; --b) {
      byte = static_cast<uint8_t>((byte << 1) | (projection[bit_start + static_cast<size_t>(b)] > 0.0f));
    }
    result[p] = byte;
  }
  if (trace != nullptr) {
    trace->histogram = std::move(histogram);
    trace->pca = std::move(pca);
    trace->random_projection = std::move(projection);
  }
  return result;
}

}  // namespace slam_refact::map_builder

#endif  // SLAM_REFACT_LOOP_CLOSE

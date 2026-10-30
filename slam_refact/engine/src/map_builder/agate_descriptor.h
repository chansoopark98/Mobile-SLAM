// Agate global keyframe descriptor (US-025 LC-2b). Compiled only with SLAM_REFACT_LOOP_CLOSE=ON.
//
// Re-implementation of the public reference runtime AgateDescriptor128 = AgateDescriptor<OrbFeature,
// AgatePca32> (reality/engine/features/agate-{descriptor,histogram,pca}.cc, binary-pca.cc,
// random-basis.cc; MIT archive) without capnp or c8 dependencies. The original xr-slam.wasm calls
// the same computation as func 1263 once per keyframe (spec/12-loop-close.md §2.5).
//
// input  : the keyframe's ORB descriptors, 32 bytes each (any count, including 0).
// output : 32 bytes. Bit b of byte p is (random_projection[8p + b] > 0).
//
// Numeric contract (the output must be bit-identical to the original; no tolerance exists):
//   * float arithmetic everywhere, except the final histogram scale
//     `1.0f / (0.00001 + l2Norm)` which the original evaluates in double and rounds to float once;
//   * every sum is sequential in index order; no FMA, no fast-math (engine flags);
//   * the random basis is std::mt19937(0) + libc++'s normal_distribution<float> (libcxx_compat),
//     8192 draws fill the 256 x 32 column-major matrix in memory order (the original FloatVector
//     fill); logf is the musl one (sr_musl_libm), like the wasm.
//   * the 256x32 matrix-vector product accumulates per row over the columns in order
//     (Eigen's column-major gemv kernel without FMA).
//
// Dictionary (64 entries of 32 B) and BinaryPca (32 dimensions, whitening) come from generated
// .inc files (tools/re/agate/gen_agate_inc.py).

#ifndef SLAM_REFACT_MAP_BUILDER_AGATE_DESCRIPTOR_H_
#define SLAM_REFACT_MAP_BUILDER_AGATE_DESCRIPTOR_H_

#include <array>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace slam_refact::map_builder {

inline constexpr size_t kAgateOrbBytes = 32;
inline constexpr size_t kAgateDictionaryEntries = 64;
inline constexpr size_t kAgateHistogramSize = kAgateDictionaryEntries * kAgateOrbBytes * 8;  // 16384
inline constexpr size_t kAgatePcaDims = 32;
inline constexpr size_t kAgateProjectionBits = kAgateOrbBytes * 8;  // 256

using AgateOrb = std::array<uint8_t, kAgateOrbBytes>;
using AgateGlobal = std::array<uint8_t, kAgateOrbBytes>;

// Intermediate values of one Extract call, for the golden test's first-difference search.
struct AgateTrace {
  std::vector<float> histogram;           // 16384, after the final L2 normalisation
  std::vector<float> pca;                 // 32, BinaryPca result * whitening, then L2-normalised
  std::vector<float> random_projection;   // 256, before binarisation
};

class AgateDescriptor {
 public:
  // Builds the random basis (8192 draws of mt19937(0) / libc++ normal_distribution<float>).
  AgateDescriptor();

  // orb points to n * 32 bytes (may be null when n == 0). trace may be null.
  AgateGlobal Extract(const uint8_t* orb, size_t n, AgateTrace* trace = nullptr) const;
  AgateGlobal Extract(const std::vector<AgateOrb>& orb, AgateTrace* trace = nullptr) const {
    return Extract(orb.empty() ? nullptr : orb[0].data(), orb.size(), trace);
  }

  // Random basis, 32 columns of 256 floats: basis()[j * 256 + i] multiplies pca[j] into row i.
  const std::vector<float>& basis() const { return basis_; }

 private:
  std::vector<float> basis_;
};

}  // namespace slam_refact::map_builder

#endif  // SLAM_REFACT_MAP_BUILDER_AGATE_DESCRIPTOR_H_

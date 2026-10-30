// Engine RNG (architecture.md §5.2). The engine uses the vendored public c8::RandomNumbers
// unchanged in its algorithm lines; WP0a's libcxx-compat.patch routes its shuffle and
// distributions to slam_refact::libcxx (emsdk 5.0.0 libc++ semantics, D-17). Using the same type
// lets engine code pass the generator straight into vendored robustP2P / MultinomialDistribution.

#ifndef SLAM_REFACT_COMPAT_RANDOM_NUMBERS_H_
#define SLAM_REFACT_COMPAT_RANDOM_NUMBERS_H_

#include <cstdint>

#include "c8/random-numbers.h"

namespace slam_refact::compat {

using RandomNumbers = c8::RandomNumbers;

// Seed of XREngine RandomNumbers and of the FrameFrameTracker std::mt19937 (VERIFIED 485
// D:28481-28496, G640:52-65). std::mt19937 default seed is the same value.
inline constexpr uint32_t kEngineRngSeed = 5489u;

}  // namespace slam_refact::compat

#endif  // SLAM_REFACT_COMPAT_RANDOM_NUMBERS_H_

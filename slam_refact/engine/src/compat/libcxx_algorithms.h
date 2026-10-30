// Engine entry point to the libc++-equivalent algorithms and distributions
// (architecture.md §5.2-§5.3, D-17). Implementation: WP0a third_party/libcxx_compat (emsdk 5.0.0
// libc++ excerpts in namespace slam_refact::libcxx).
//
// Rule for engine code: wherever the order or value produced by sort / nth_element /
// partial_sort / shuffle / a distribution can reach the output, call slam_refact::libcxx::*
// instead of std::* (checked by tools/check_std_algos.py). std::stable_sort may stay std (its
// result is implementation independent). libcxx::shuffle pins its distribution integer type to
// int32_t (wasm32 difference_type, §5.2 [M1]).

#ifndef SLAM_REFACT_COMPAT_LIBCXX_ALGORITHMS_H_
#define SLAM_REFACT_COMPAT_LIBCXX_ALGORITHMS_H_

#include "libcxx_compat/algorithm.h"
#include "libcxx_compat/random.h"

#endif  // SLAM_REFACT_COMPAT_LIBCXX_ALGORITHMS_H_

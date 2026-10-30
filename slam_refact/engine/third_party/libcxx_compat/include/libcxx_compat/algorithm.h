// libcxx_compat umbrella: libc++ (emsdk 5.0.0, LLVM 21.1.8) algorithms in slam_refact::libcxx.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception (excerpted files)
//
// Public entry points (same signatures as the std:: ones):
//   sort, nth_element, partial_sort, make_heap, push_heap, pop_heap, sort_heap, min_element,
//   shuffle (distribution integer type pinned to int32_t, architecture.md §5.2).
#ifndef SLAM_REFACT_LIBCXX_COMPAT_ALGORITHM_H
#define SLAM_REFACT_LIBCXX_COMPAT_ALGORITHM_H

#include "libcxx_compat/__config.h"
#include "libcxx_compat/__algorithm/make_heap.h"
#include "libcxx_compat/__algorithm/min_element.h"
#include "libcxx_compat/__algorithm/nth_element.h"
#include "libcxx_compat/__algorithm/partial_sort.h"
#include "libcxx_compat/__algorithm/pop_heap.h"
#include "libcxx_compat/__algorithm/push_heap.h"
#include "libcxx_compat/__algorithm/shuffle.h"
#include "libcxx_compat/__algorithm/sort.h"
#include "libcxx_compat/__algorithm/sort_heap.h"

#endif  // SLAM_REFACT_LIBCXX_COMPAT_ALGORITHM_H

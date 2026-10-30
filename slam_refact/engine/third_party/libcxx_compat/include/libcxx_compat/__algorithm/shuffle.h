//===----------------------------------------------------------------------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

#ifndef _SR_LIBCPP___ALGORITHM_SHUFFLE_H
#define _SR_LIBCPP___ALGORITHM_SHUFFLE_H

// [libcxx_compat/__config.h] #include <__algorithm/iterator_operations.h>
#include "libcxx_compat/__config.h"
// [libcxx_compat/__config.h] #include <__cstddef/ptrdiff_t.h>
// [libcxx_compat/__config.h] #include <__iterator/iterator_traits.h>
#include "libcxx_compat/__random/uniform_int_distribution.h"
// [libcxx_compat/__config.h] #include <__utility/forward.h>
// [libcxx_compat/__config.h] #include <__utility/move.h>
// [libcxx_compat/__config.h] #include <__utility/swap.h>
#include <cstdint>

#if !defined(_SR_LIBCPP_HAS_NO_PRAGMA_SYSTEM_HEADER)
#  pragma GCC system_header
#endif

_SR_LIBCPP_PUSH_MACROS
// [libcxx_compat/__config.h] #include <__undef_macros>

_SR_LIBCXX_BEGIN_NAMESPACE

// [libcxx_compat] Removed: __libcpp_debug_randomizer (debug-mode only) and the C++14
// random_shuffle overloads (compiled out at _LIBCPP_STD_VER >= 17). __shuffle and shuffle below
// are unchanged except for the distribution's integer type (see the DECISION note there).

template <class _AlgPolicy, class _RandomAccessIterator, class _Sentinel, class _UniformRandomNumberGenerator>
_SR_LIBCPP_HIDE_FROM_ABI _RandomAccessIterator
__shuffle(_RandomAccessIterator __first, _Sentinel __last_sentinel, _UniformRandomNumberGenerator&& __g) {
  typedef typename iterator_traits<_RandomAccessIterator>::difference_type difference_type;
  // [libcxx_compat] DECISION (architecture.md §5.2): libc++ uses uniform_int_distribution<ptrdiff_t>.
  // ptrdiff_t is 32-bit on wasm32 (the original binary) and 64-bit on x86-64, which changes how
  // many engine draws each step consumes. The excerpt fixes the type to the wasm32 width.
  typedef uniform_int_distribution<__sr_wasm32_ptrdiff_t> _Dp;
  typedef typename _Dp::param_type _Pp;

  auto __original_last = _IterOps<_AlgPolicy>::next(__first, __last_sentinel);
  auto __last          = __original_last;
  difference_type __d  = __last - __first;
  if (__d > 1) {
    _Dp __uid;
    for (--__last, (void)--__d; __first < __last; ++__first, (void)--__d) {
      difference_type __i = __uid(__g, _Pp(0, __d));
      if (__i != difference_type(0))
        _IterOps<_AlgPolicy>::iter_swap(__first, __first + __i);
    }
  }

  return __original_last;
}

template <class _RandomAccessIterator, class _UniformRandomNumberGenerator>
_SR_LIBCPP_HIDE_FROM_ABI void
shuffle(_RandomAccessIterator __first, _RandomAccessIterator __last, _UniformRandomNumberGenerator&& __g) {
  (void)::slam_refact::libcxx::__shuffle<_ClassicAlgPolicy>(
      std::move(__first), std::move(__last), std::forward<_UniformRandomNumberGenerator>(__g));
}

_SR_LIBCXX_END_NAMESPACE

_SR_LIBCPP_POP_MACROS

#endif // _SR_LIBCPP___ALGORITHM_SHUFFLE_H

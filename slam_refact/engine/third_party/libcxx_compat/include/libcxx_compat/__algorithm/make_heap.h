//===----------------------------------------------------------------------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

#ifndef _SR_LIBCPP___ALGORITHM_MAKE_HEAP_H
#define _SR_LIBCPP___ALGORITHM_MAKE_HEAP_H

// [libcxx_compat/__config.h] #include <__algorithm/comp.h>
// [libcxx_compat/__config.h] #include <__algorithm/comp_ref_type.h>
// [libcxx_compat/__config.h] #include <__algorithm/iterator_operations.h>
#include "libcxx_compat/__algorithm/sift_down.h"
#include "libcxx_compat/__config.h"
// [libcxx_compat/__config.h] #include <__iterator/iterator_traits.h>
// [libcxx_compat/__config.h] #include <__utility/move.h>

#if !defined(_SR_LIBCPP_HAS_NO_PRAGMA_SYSTEM_HEADER)
#  pragma GCC system_header
#endif

_SR_LIBCPP_PUSH_MACROS
// [libcxx_compat/__config.h] #include <__undef_macros>

_SR_LIBCXX_BEGIN_NAMESPACE

template <class _AlgPolicy, class _Compare, class _RandomAccessIterator>
inline _SR_LIBCPP_HIDE_FROM_ABI _SR_LIBCPP_CONSTEXPR_SINCE_CXX14 void
__make_heap(_RandomAccessIterator __first, _RandomAccessIterator __last, _Compare&& __comp) {
  __comp_ref_type<_Compare> __comp_ref = __comp;

  using difference_type = typename iterator_traits<_RandomAccessIterator>::difference_type;
  difference_type __n   = __last - __first;
  if (__n > 1) {
    // start from the first parent, there is no need to consider children
    for (difference_type __start = (__n - 2) / 2; __start >= 0; --__start) {
      ::slam_refact::libcxx::__sift_down<_AlgPolicy>(__first, __comp_ref, __n, __first + __start);
    }
  }
}

template <class _RandomAccessIterator, class _Compare>
inline _SR_LIBCPP_HIDE_FROM_ABI _SR_LIBCPP_CONSTEXPR_SINCE_CXX20 void
make_heap(_RandomAccessIterator __first, _RandomAccessIterator __last, _Compare __comp) {
  ::slam_refact::libcxx::__make_heap<_ClassicAlgPolicy>(std::move(__first), std::move(__last), __comp);
}

template <class _RandomAccessIterator>
inline _SR_LIBCPP_HIDE_FROM_ABI _SR_LIBCPP_CONSTEXPR_SINCE_CXX20 void
make_heap(_RandomAccessIterator __first, _RandomAccessIterator __last) {
  ::slam_refact::libcxx::make_heap(std::move(__first), std::move(__last), __less<>());
}

_SR_LIBCXX_END_NAMESPACE

_SR_LIBCPP_POP_MACROS

#endif // _SR_LIBCPP___ALGORITHM_MAKE_HEAP_H

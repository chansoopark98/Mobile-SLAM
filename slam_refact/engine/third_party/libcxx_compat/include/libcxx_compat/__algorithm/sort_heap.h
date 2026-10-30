//===----------------------------------------------------------------------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

#ifndef _SR_LIBCPP___ALGORITHM_SORT_HEAP_H
#define _SR_LIBCPP___ALGORITHM_SORT_HEAP_H

// [libcxx_compat/__config.h] #include <__algorithm/comp.h>
// [libcxx_compat/__config.h] #include <__algorithm/comp_ref_type.h>
// [libcxx_compat/__config.h] #include <__algorithm/iterator_operations.h>
#include "libcxx_compat/__algorithm/pop_heap.h"
#include "libcxx_compat/__config.h"
// [libcxx_compat/__config.h] #include <__debug_utils/strict_weak_ordering_check.h>
// [libcxx_compat/__config.h] #include <__iterator/iterator_traits.h>
// [libcxx_compat/__config.h] #include <__type_traits/is_assignable.h>
// [libcxx_compat/__config.h] #include <__type_traits/is_constructible.h>
// [libcxx_compat/__config.h] #include <__utility/move.h>

#if !defined(_SR_LIBCPP_HAS_NO_PRAGMA_SYSTEM_HEADER)
#  pragma GCC system_header
#endif

_SR_LIBCPP_PUSH_MACROS
// [libcxx_compat/__config.h] #include <__undef_macros>

_SR_LIBCXX_BEGIN_NAMESPACE

template <class _AlgPolicy, class _Compare, class _RandomAccessIterator>
inline _SR_LIBCPP_HIDE_FROM_ABI _SR_LIBCPP_CONSTEXPR_SINCE_CXX14 void
__sort_heap(_RandomAccessIterator __first, _RandomAccessIterator __last, _Compare&& __comp) {
  _RandomAccessIterator __saved_last   = __last;
  __comp_ref_type<_Compare> __comp_ref = __comp;

  using difference_type = typename iterator_traits<_RandomAccessIterator>::difference_type;
  for (difference_type __n = __last - __first; __n > 1; --__last, (void)--__n)
    ::slam_refact::libcxx::__pop_heap<_AlgPolicy>(__first, __last, __comp_ref, __n);
  ::slam_refact::libcxx::__check_strict_weak_ordering_sorted(__first, __saved_last, __comp_ref);
}

template <class _RandomAccessIterator, class _Compare>
inline _SR_LIBCPP_HIDE_FROM_ABI _SR_LIBCPP_CONSTEXPR_SINCE_CXX20 void
sort_heap(_RandomAccessIterator __first, _RandomAccessIterator __last, _Compare __comp) {
  static_assert(std::is_copy_constructible<_RandomAccessIterator>::value, "Iterators must be copy constructible.");
  static_assert(std::is_copy_assignable<_RandomAccessIterator>::value, "Iterators must be copy assignable.");

  ::slam_refact::libcxx::__sort_heap<_ClassicAlgPolicy>(std::move(__first), std::move(__last), __comp);
}

template <class _RandomAccessIterator>
inline _SR_LIBCPP_HIDE_FROM_ABI _SR_LIBCPP_CONSTEXPR_SINCE_CXX20 void
sort_heap(_RandomAccessIterator __first, _RandomAccessIterator __last) {
  ::slam_refact::libcxx::sort_heap(std::move(__first), std::move(__last), __less<>());
}

_SR_LIBCXX_END_NAMESPACE

_SR_LIBCPP_POP_MACROS

#endif // _SR_LIBCPP___ALGORITHM_SORT_HEAP_H

//===----------------------------------------------------------------------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

#ifndef _SR_LIBCPP___ALGORITHM_PARTIAL_SORT_H
#define _SR_LIBCPP___ALGORITHM_PARTIAL_SORT_H

// [libcxx_compat/__config.h] #include <__algorithm/comp.h>
// [libcxx_compat/__config.h] #include <__algorithm/comp_ref_type.h>
// [libcxx_compat/__config.h] #include <__algorithm/iterator_operations.h>
#include "libcxx_compat/__algorithm/make_heap.h"
#include "libcxx_compat/__algorithm/sift_down.h"
#include "libcxx_compat/__algorithm/sort_heap.h"
#include "libcxx_compat/__config.h"
// [libcxx_compat/__config.h] #include <__debug_utils/randomize_range.h>
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

template <class _AlgPolicy, class _Compare, class _RandomAccessIterator, class _Sentinel>
_SR_LIBCPP_HIDE_FROM_ABI _SR_LIBCPP_CONSTEXPR_SINCE_CXX20 _RandomAccessIterator __partial_sort_impl(
    _RandomAccessIterator __first, _RandomAccessIterator __middle, _Sentinel __last, _Compare&& __comp) {
  if (__first == __middle) {
    return _IterOps<_AlgPolicy>::next(__middle, __last);
  }

  ::slam_refact::libcxx::__make_heap<_AlgPolicy>(__first, __middle, __comp);

  typename iterator_traits<_RandomAccessIterator>::difference_type __len = __middle - __first;
  _RandomAccessIterator __i                                              = __middle;
  for (; __i != __last; ++__i) {
    if (__comp(*__i, *__first)) {
      _IterOps<_AlgPolicy>::iter_swap(__i, __first);
      ::slam_refact::libcxx::__sift_down<_AlgPolicy>(__first, __comp, __len, __first);
    }
  }
  ::slam_refact::libcxx::__sort_heap<_AlgPolicy>(std::move(__first), std::move(__middle), __comp);

  return __i;
}

template <class _AlgPolicy, class _Compare, class _RandomAccessIterator, class _Sentinel>
_SR_LIBCPP_HIDE_FROM_ABI _SR_LIBCPP_CONSTEXPR_SINCE_CXX20 _RandomAccessIterator
__partial_sort(_RandomAccessIterator __first, _RandomAccessIterator __middle, _Sentinel __last, _Compare& __comp) {
  if (__first == __middle)
    return _IterOps<_AlgPolicy>::next(__middle, __last);

  ::slam_refact::libcxx::__debug_randomize_range<_AlgPolicy>(__first, __last);

  auto __last_iter =
      ::slam_refact::libcxx::__partial_sort_impl<_AlgPolicy>(__first, __middle, __last, static_cast<__comp_ref_type<_Compare> >(__comp));

  ::slam_refact::libcxx::__debug_randomize_range<_AlgPolicy>(__middle, __last);

  return __last_iter;
}

template <class _RandomAccessIterator, class _Compare>
inline _SR_LIBCPP_HIDE_FROM_ABI _SR_LIBCPP_CONSTEXPR_SINCE_CXX20 void partial_sort(
    _RandomAccessIterator __first, _RandomAccessIterator __middle, _RandomAccessIterator __last, _Compare __comp) {
  static_assert(std::is_copy_constructible<_RandomAccessIterator>::value, "Iterators must be copy constructible.");
  static_assert(std::is_copy_assignable<_RandomAccessIterator>::value, "Iterators must be copy assignable.");

  (void)::slam_refact::libcxx::__partial_sort<_ClassicAlgPolicy>(std::move(__first), std::move(__middle), std::move(__last), __comp);
}

template <class _RandomAccessIterator>
inline _SR_LIBCPP_HIDE_FROM_ABI _SR_LIBCPP_CONSTEXPR_SINCE_CXX20 void
partial_sort(_RandomAccessIterator __first, _RandomAccessIterator __middle, _RandomAccessIterator __last) {
  ::slam_refact::libcxx::partial_sort(__first, __middle, __last, __less<>());
}

_SR_LIBCXX_END_NAMESPACE

_SR_LIBCPP_POP_MACROS

#endif // _SR_LIBCPP___ALGORITHM_PARTIAL_SORT_H

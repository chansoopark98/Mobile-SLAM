//===----------------------------------------------------------------------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

#ifndef _SR_LIBCPP___ALGORITHM_POP_HEAP_H
#define _SR_LIBCPP___ALGORITHM_POP_HEAP_H

// [libcxx_compat/__config.h] #include <__algorithm/comp.h>
// [libcxx_compat/__config.h] #include <__algorithm/comp_ref_type.h>
// [libcxx_compat/__config.h] #include <__algorithm/iterator_operations.h>
#include "libcxx_compat/__algorithm/push_heap.h"
#include "libcxx_compat/__algorithm/sift_down.h"
// [libcxx_compat/__config.h] #include <__assert>
#include "libcxx_compat/__config.h"
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
__pop_heap(_RandomAccessIterator __first,
           _RandomAccessIterator __last,
           _Compare& __comp,
           typename iterator_traits<_RandomAccessIterator>::difference_type __len) {
  // Calling `pop_heap` on an empty range is undefined behavior, but in practice it will be a no-op.
  _SR_LIBCPP_ASSERT_PEDANTIC(__len > 0, "The heap given to pop_heap must be non-empty");

  __comp_ref_type<_Compare> __comp_ref = __comp;

  using value_type = typename iterator_traits<_RandomAccessIterator>::value_type;
  if (__len > 1) {
    value_type __top             = _IterOps<_AlgPolicy>::__iter_move(__first); // create a hole at __first
    _RandomAccessIterator __hole = ::slam_refact::libcxx::__floyd_sift_down<_AlgPolicy>(__first, __comp_ref, __len);
    --__last;

    if (__hole == __last) {
      *__hole = std::move(__top);
    } else {
      *__hole = _IterOps<_AlgPolicy>::__iter_move(__last);
      ++__hole;
      *__last = std::move(__top);
      ::slam_refact::libcxx::__sift_up<_AlgPolicy>(__first, __hole, __comp_ref, __hole - __first);
    }
  }
}

template <class _RandomAccessIterator, class _Compare>
inline _SR_LIBCPP_HIDE_FROM_ABI _SR_LIBCPP_CONSTEXPR_SINCE_CXX20 void
pop_heap(_RandomAccessIterator __first, _RandomAccessIterator __last, _Compare __comp) {
  static_assert(std::is_copy_constructible<_RandomAccessIterator>::value, "Iterators must be copy constructible.");
  static_assert(std::is_copy_assignable<_RandomAccessIterator>::value, "Iterators must be copy assignable.");

  typename iterator_traits<_RandomAccessIterator>::difference_type __len = __last - __first;
  ::slam_refact::libcxx::__pop_heap<_ClassicAlgPolicy>(std::move(__first), std::move(__last), __comp, __len);
}

template <class _RandomAccessIterator>
inline _SR_LIBCPP_HIDE_FROM_ABI _SR_LIBCPP_CONSTEXPR_SINCE_CXX20 void
pop_heap(_RandomAccessIterator __first, _RandomAccessIterator __last) {
  ::slam_refact::libcxx::pop_heap(std::move(__first), std::move(__last), __less<>());
}

_SR_LIBCXX_END_NAMESPACE

_SR_LIBCPP_POP_MACROS

#endif // _SR_LIBCPP___ALGORITHM_POP_HEAP_H

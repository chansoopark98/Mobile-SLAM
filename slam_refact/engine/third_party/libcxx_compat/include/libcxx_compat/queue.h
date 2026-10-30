//===----------------------------------------------------------------------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// libcxx_compat: excerpt of libc++ <queue> (emsdk 5.0.0, LLVM 21.1.8): priority_queue with the
// members the vendored reference runtime code uses (default/compare constructors, empty, size, top, push,
// emplace, pop). The member bodies are the libc++ ones; push_heap/pop_heap resolve to the
// slam_refact::libcxx excerpts so the heap layout matches libc++. Allocator-extended and range
// constructors, swap and comparison operators are not excerpted.
#ifndef SLAM_REFACT_LIBCXX_COMPAT_QUEUE_H
#define SLAM_REFACT_LIBCXX_COMPAT_QUEUE_H

#include <vector>

#include "libcxx_compat/__config.h"
#include "libcxx_compat/__algorithm/make_heap.h"
#include "libcxx_compat/__algorithm/pop_heap.h"
#include "libcxx_compat/__algorithm/push_heap.h"

_SR_LIBCXX_BEGIN_NAMESPACE

template <class _Tp, class _Container = std::vector<_Tp>, class _Compare = less<typename _Container::value_type> >
class priority_queue {
public:
  typedef _Container container_type;
  typedef _Compare value_compare;
  typedef typename container_type::value_type value_type;
  typedef typename container_type::reference reference;
  typedef typename container_type::const_reference const_reference;
  typedef typename container_type::size_type size_type;
  static_assert(is_same<_Tp, value_type>::value, "");

protected:
  container_type c;
  value_compare comp;

public:
  _SR_LIBCPP_HIDE_FROM_ABI priority_queue() : c(), comp() {}

  _SR_LIBCPP_HIDE_FROM_ABI explicit priority_queue(const value_compare& __comp) : c(), comp(__comp) {}

  _SR_LIBCPP_HIDE_FROM_ABI priority_queue(const value_compare& __comp, const container_type& __c)
      : c(__c), comp(__comp) {
    ::slam_refact::libcxx::make_heap(c.begin(), c.end(), comp);
  }

  [[__nodiscard__]] _SR_LIBCPP_HIDE_FROM_ABI bool empty() const { return c.empty(); }
  _SR_LIBCPP_HIDE_FROM_ABI size_type size() const { return c.size(); }
  _SR_LIBCPP_HIDE_FROM_ABI const_reference top() const { return c.front(); }

  _SR_LIBCPP_HIDE_FROM_ABI void push(const value_type& __v);
  _SR_LIBCPP_HIDE_FROM_ABI void push(value_type&& __v);
  template <class... _Args>
  _SR_LIBCPP_HIDE_FROM_ABI void emplace(_Args&&... __args);
  _SR_LIBCPP_HIDE_FROM_ABI void pop();
};

template <class _Tp, class _Container, class _Compare>
inline void priority_queue<_Tp, _Container, _Compare>::push(const value_type& __v) {
  c.push_back(__v);
  ::slam_refact::libcxx::push_heap(c.begin(), c.end(), comp);
}

template <class _Tp, class _Container, class _Compare>
inline void priority_queue<_Tp, _Container, _Compare>::push(value_type&& __v) {
  c.push_back(std::move(__v));
  ::slam_refact::libcxx::push_heap(c.begin(), c.end(), comp);
}

template <class _Tp, class _Container, class _Compare>
template <class... _Args>
inline void priority_queue<_Tp, _Container, _Compare>::emplace(_Args&&... __args) {
  c.emplace_back(std::forward<_Args>(__args)...);
  ::slam_refact::libcxx::push_heap(c.begin(), c.end(), comp);
}

template <class _Tp, class _Container, class _Compare>
inline void priority_queue<_Tp, _Container, _Compare>::pop() {
  ::slam_refact::libcxx::pop_heap(c.begin(), c.end(), comp);
  c.pop_back();
}

_SR_LIBCXX_END_NAMESPACE

#endif  // SLAM_REFACT_LIBCXX_COMPAT_QUEUE_H

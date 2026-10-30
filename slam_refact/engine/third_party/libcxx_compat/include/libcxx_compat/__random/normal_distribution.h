//===----------------------------------------------------------------------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

#ifndef _SR_LIBCPP___RANDOM_NORMAL_DISTRIBUTION_H
#define _SR_LIBCPP___RANDOM_NORMAL_DISTRIBUTION_H

#include "libcxx_compat/__config.h"
// [libcxx_compat/__config.h] #include <__random/is_valid.h>
#include "libcxx_compat/__random/uniform_real_distribution.h"
#include <cmath>
#include <iosfwd>
#include <limits>

#if !defined(_SR_LIBCPP_HAS_NO_PRAGMA_SYSTEM_HEADER)
#  pragma GCC system_header
#endif

_SR_LIBCPP_PUSH_MACROS
// [libcxx_compat/__config.h] #include <__undef_macros>

_SR_LIBCXX_BEGIN_NAMESPACE

template <class _RealType = double>
class normal_distribution {
  static_assert(__libcpp_random_is_valid_realtype<_RealType>::value,
                "RealType must be a supported floating-point type");

public:
  // types
  typedef _RealType result_type;

  class param_type {
    result_type __mean_;
    result_type __stddev_;

  public:
    typedef normal_distribution distribution_type;

    _SR_LIBCPP_HIDE_FROM_ABI explicit param_type(result_type __mean = 0, result_type __stddev = 1)
        : __mean_(__mean), __stddev_(__stddev) {}

    _SR_LIBCPP_HIDE_FROM_ABI result_type mean() const { return __mean_; }
    _SR_LIBCPP_HIDE_FROM_ABI result_type stddev() const { return __stddev_; }

    friend _SR_LIBCPP_HIDE_FROM_ABI bool operator==(const param_type& __x, const param_type& __y) {
      return __x.__mean_ == __y.__mean_ && __x.__stddev_ == __y.__stddev_;
    }
    friend _SR_LIBCPP_HIDE_FROM_ABI bool operator!=(const param_type& __x, const param_type& __y) { return !(__x == __y); }
  };

private:
  param_type __p_;
  result_type __v_;
  bool __v_hot_;

public:
  // constructors and reset functions
#ifndef _SR_LIBCPP_CXX03_LANG
  _SR_LIBCPP_HIDE_FROM_ABI normal_distribution() : normal_distribution(0) {}
  _SR_LIBCPP_HIDE_FROM_ABI explicit normal_distribution(result_type __mean, result_type __stddev = 1)
      : __p_(param_type(__mean, __stddev)), __v_hot_(false) {}
#else
  _SR_LIBCPP_HIDE_FROM_ABI explicit normal_distribution(result_type __mean = 0, result_type __stddev = 1)
      : __p_(param_type(__mean, __stddev)), __v_hot_(false) {}
#endif
  _SR_LIBCPP_HIDE_FROM_ABI explicit normal_distribution(const param_type& __p) : __p_(__p), __v_hot_(false) {}
  _SR_LIBCPP_HIDE_FROM_ABI void reset() { __v_hot_ = false; }

  // generating functions
  template <class _URNG>
  _SR_LIBCPP_HIDE_FROM_ABI result_type operator()(_URNG& __g) {
    return (*this)(__g, __p_);
  }
  template <class _URNG>
  _SR_LIBCPP_HIDE_FROM_ABI result_type operator()(_URNG& __g, const param_type& __p);

  // property functions
  _SR_LIBCPP_HIDE_FROM_ABI result_type mean() const { return __p_.mean(); }
  _SR_LIBCPP_HIDE_FROM_ABI result_type stddev() const { return __p_.stddev(); }

  _SR_LIBCPP_HIDE_FROM_ABI param_type param() const { return __p_; }
  _SR_LIBCPP_HIDE_FROM_ABI void param(const param_type& __p) { __p_ = __p; }

  _SR_LIBCPP_HIDE_FROM_ABI result_type min() const { return -numeric_limits<result_type>::infinity(); }
  _SR_LIBCPP_HIDE_FROM_ABI result_type max() const { return numeric_limits<result_type>::infinity(); }

  friend _SR_LIBCPP_HIDE_FROM_ABI bool operator==(const normal_distribution& __x, const normal_distribution& __y) {
    return __x.__p_ == __y.__p_ && __x.__v_hot_ == __y.__v_hot_ && (!__x.__v_hot_ || __x.__v_ == __y.__v_);
  }
  friend _SR_LIBCPP_HIDE_FROM_ABI bool operator!=(const normal_distribution& __x, const normal_distribution& __y) {
    return !(__x == __y);
  }

  // [libcxx_compat] Removed: iostream operator<< / operator>> (not used by the engine).
};

template <class _RealType>
template <class _URNG>
_RealType normal_distribution<_RealType>::operator()(_URNG& __g, const param_type& __p) {
  static_assert(__libcpp_random_is_valid_urng<_URNG>::value, "");
  result_type __up;
  if (__v_hot_) {
    __v_hot_ = false;
    __up     = __v_;
  } else {
    uniform_real_distribution<result_type> __uni(-1, 1);
    result_type __u;
    result_type __v;
    result_type __s;
    do {
      __u = __uni(__g);
      __v = __uni(__g);
      __s = __u * __u + __v * __v;
    } while (__s > 1 || __s == 0);
    result_type __fp = std::sqrt(-2 * std::log(__s) / __s);
    __v_             = __v * __fp;
    __v_hot_         = true;
    __up             = __u * __fp;
  }
  return __up * __p.stddev() + __p.mean();
}

// [libcxx_compat] Removed: iostream operator<< / operator>> (not used by the engine).
_SR_LIBCXX_END_NAMESPACE

_SR_LIBCPP_POP_MACROS

#endif // _SR_LIBCPP___RANDOM_NORMAL_DISTRIBUTION_H

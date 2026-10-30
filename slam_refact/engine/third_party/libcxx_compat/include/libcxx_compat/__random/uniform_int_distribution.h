//===----------------------------------------------------------------------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

#ifndef _SR_LIBCPP___RANDOM_UNIFORM_INT_DISTRIBUTION_H
#define _SR_LIBCPP___RANDOM_UNIFORM_INT_DISTRIBUTION_H

// [libcxx_compat/__config.h] #include <__bit/countl.h>
#include "libcxx_compat/__config.h"
// [libcxx_compat/__config.h] #include <__cstddef/size_t.h>
// [libcxx_compat/__config.h] #include <__random/is_valid.h>
#include "libcxx_compat/__random/log2.h"
// [libcxx_compat/__config.h] #include <__type_traits/conditional.h>
// [libcxx_compat/__config.h] #include <__type_traits/make_unsigned.h>
#include <cstdint>
#include <iosfwd>
#include <limits>

#if !defined(_SR_LIBCPP_HAS_NO_PRAGMA_SYSTEM_HEADER)
#  pragma GCC system_header
#endif

_SR_LIBCPP_PUSH_MACROS
// [libcxx_compat/__config.h] #include <__undef_macros>

_SR_LIBCXX_BEGIN_NAMESPACE

template <class _Engine, class _UIntType>
class __independent_bits_engine {
public:
  // types
  typedef _UIntType result_type;

private:
  typedef typename _Engine::result_type _Engine_result_type;
  typedef __conditional_t<sizeof(_Engine_result_type) <= sizeof(result_type), result_type, _Engine_result_type>
      _Working_result_type;

  _Engine& __e_;
  size_t __w_;
  size_t __w0_;
  size_t __n_;
  size_t __n0_;
  _Working_result_type __y0_;
  _Working_result_type __y1_;
  _Engine_result_type __mask0_;
  _Engine_result_type __mask1_;

#ifdef _SR_LIBCPP_CXX03_LANG
  static const _Working_result_type _Rp = _Engine::_Max - _Engine::_Min + _Working_result_type(1);
#else
  static _SR_LIBCPP_CONSTEXPR const _Working_result_type _Rp = _Engine::max() - _Engine::min() + _Working_result_type(1);
#endif
  static _SR_LIBCPP_CONSTEXPR const size_t __m  = __log2<_Working_result_type, _Rp>::value;
  static _SR_LIBCPP_CONSTEXPR const size_t _WDt = numeric_limits<_Working_result_type>::digits;
  static _SR_LIBCPP_CONSTEXPR const size_t _EDt = numeric_limits<_Engine_result_type>::digits;

public:
  // constructors and seeding functions
  _SR_LIBCPP_HIDE_FROM_ABI __independent_bits_engine(_Engine& __e, size_t __w);

  // generating functions
  _SR_LIBCPP_HIDE_FROM_ABI result_type operator()() { return __eval(integral_constant<bool, _Rp != 0>()); }

private:
  _SR_LIBCPP_HIDE_FROM_ABI result_type __eval(false_type);
  _SR_LIBCPP_HIDE_FROM_ABI result_type __eval(true_type);
};

template <class _Engine, class _UIntType>
__independent_bits_engine<_Engine, _UIntType>::__independent_bits_engine(_Engine& __e, size_t __w)
    : __e_(__e), __w_(__w) {
  __n_  = __w_ / __m + (__w_ % __m != 0);
  __w0_ = __w_ / __n_;
  if (_Rp == 0)
    __y0_ = _Rp;
  else if (__w0_ < _WDt)
    __y0_ = (_Rp >> __w0_) << __w0_;
  else
    __y0_ = 0;
  if (_Rp - __y0_ > __y0_ / __n_) {
    ++__n_;
    __w0_ = __w_ / __n_;
    if (__w0_ < _WDt)
      __y0_ = (_Rp >> __w0_) << __w0_;
    else
      __y0_ = 0;
  }
  __n0_ = __n_ - __w_ % __n_;
  if (__w0_ < _WDt - 1)
    __y1_ = (_Rp >> (__w0_ + 1)) << (__w0_ + 1);
  else
    __y1_ = 0;
  __mask0_ = __w0_ > 0 ? _Engine_result_type(~0) >> (_EDt - __w0_) : _Engine_result_type(0);
  __mask1_ = __w0_ < _EDt - 1 ? _Engine_result_type(~0) >> (_EDt - (__w0_ + 1)) : _Engine_result_type(~0);
}

template <class _Engine, class _UIntType>
inline _UIntType __independent_bits_engine<_Engine, _UIntType>::__eval(false_type) {
  return static_cast<result_type>(__e_() & __mask0_);
}

template <class _Engine, class _UIntType>
_UIntType __independent_bits_engine<_Engine, _UIntType>::__eval(true_type) {
  const size_t __w_rt = numeric_limits<result_type>::digits;
  result_type __sp    = 0;
  for (size_t __k = 0; __k < __n0_; ++__k) {
    _Engine_result_type __u;
    do {
      __u = __e_() - _Engine::min();
    } while (__u >= __y0_);
    if (__w0_ < __w_rt)
      __sp <<= __w0_;
    else
      __sp = 0;
    __sp += __u & __mask0_;
  }
  for (size_t __k = __n0_; __k < __n_; ++__k) {
    _Engine_result_type __u;
    do {
      __u = __e_() - _Engine::min();
    } while (__u >= __y1_);
    if (__w0_ < __w_rt - 1)
      __sp <<= __w0_ + 1;
    else
      __sp = 0;
    __sp += __u & __mask1_;
  }
  return __sp;
}

template <class _IntType = int>
class uniform_int_distribution {
  static_assert(__libcpp_random_is_valid_inttype<_IntType>::value, "IntType must be a supported integer type");

public:
  // types
  typedef _IntType result_type;

  class param_type {
    result_type __a_;
    result_type __b_;

  public:
    typedef uniform_int_distribution distribution_type;

    _SR_LIBCPP_HIDE_FROM_ABI explicit param_type(result_type __a = 0, result_type __b = numeric_limits<result_type>::max())
        : __a_(__a), __b_(__b) {}

    _SR_LIBCPP_HIDE_FROM_ABI result_type a() const { return __a_; }
    _SR_LIBCPP_HIDE_FROM_ABI result_type b() const { return __b_; }

    _SR_LIBCPP_HIDE_FROM_ABI friend bool operator==(const param_type& __x, const param_type& __y) {
      return __x.__a_ == __y.__a_ && __x.__b_ == __y.__b_;
    }
    _SR_LIBCPP_HIDE_FROM_ABI friend bool operator!=(const param_type& __x, const param_type& __y) { return !(__x == __y); }
  };

private:
  param_type __p_;

public:
  // constructors and reset functions
#ifndef _SR_LIBCPP_CXX03_LANG
  _SR_LIBCPP_HIDE_FROM_ABI uniform_int_distribution() : uniform_int_distribution(0) {}
  _SR_LIBCPP_HIDE_FROM_ABI explicit uniform_int_distribution(
      result_type __a, result_type __b = numeric_limits<result_type>::max())
      : __p_(param_type(__a, __b)) {}
#else
  explicit uniform_int_distribution(result_type __a = 0, result_type __b = numeric_limits<result_type>::max())
      : __p_(param_type(__a, __b)) {}
#endif
  _SR_LIBCPP_HIDE_FROM_ABI explicit uniform_int_distribution(const param_type& __p) : __p_(__p) {}
  _SR_LIBCPP_HIDE_FROM_ABI void reset() {}

  // generating functions
  template <class _URNG>
  _SR_LIBCPP_HIDE_FROM_ABI result_type operator()(_URNG& __g) {
    return (*this)(__g, __p_);
  }
  template <class _URNG>
  _SR_LIBCPP_HIDE_FROM_ABI result_type operator()(_URNG& __g, const param_type& __p);

  // property functions
  _SR_LIBCPP_HIDE_FROM_ABI result_type a() const { return __p_.a(); }
  _SR_LIBCPP_HIDE_FROM_ABI result_type b() const { return __p_.b(); }

  _SR_LIBCPP_HIDE_FROM_ABI param_type param() const { return __p_; }
  _SR_LIBCPP_HIDE_FROM_ABI void param(const param_type& __p) { __p_ = __p; }

  _SR_LIBCPP_HIDE_FROM_ABI result_type min() const { return a(); }
  _SR_LIBCPP_HIDE_FROM_ABI result_type max() const { return b(); }

  _SR_LIBCPP_HIDE_FROM_ABI friend bool
  operator==(const uniform_int_distribution& __x, const uniform_int_distribution& __y) {
    return __x.__p_ == __y.__p_;
  }
  _SR_LIBCPP_HIDE_FROM_ABI friend bool
  operator!=(const uniform_int_distribution& __x, const uniform_int_distribution& __y) {
    return !(__x == __y);
  }
};

template <class _IntType>
template <class _URNG>
typename uniform_int_distribution<_IntType>::result_type uniform_int_distribution<_IntType>::operator()(
    _URNG& __g, const param_type& __p) _SR_LIBCPP_DISABLE_UBSAN_UNSIGNED_INTEGER_CHECK {
  static_assert(__libcpp_random_is_valid_urng<_URNG>::value, "");
  typedef __conditional_t<sizeof(result_type) <= sizeof(uint32_t), uint32_t, __make_unsigned_t<result_type> > _UIntType;
  const _UIntType __rp = _UIntType(__p.b()) - _UIntType(__p.a()) + _UIntType(1);
  if (__rp == 1)
    return __p.a();
  const size_t __dt = numeric_limits<_UIntType>::digits;
  typedef __independent_bits_engine<_URNG, _UIntType> _Eng;
  if (__rp == 0)
    return static_cast<result_type>(_Eng(__g, __dt)());
  size_t __w = __dt - ::slam_refact::libcxx::__countl_zero(__rp) - 1;
  if ((__rp & (numeric_limits<_UIntType>::max() >> (__dt - __w))) != 0)
    ++__w;
  _Eng __e(__g, __w);
  _UIntType __u;
  do {
    __u = __e();
  } while (__u >= __rp);
  return static_cast<result_type>(__u + __p.a());
}

// [libcxx_compat] Removed: iostream operator<< / operator>> (not used by the engine).
_SR_LIBCXX_END_NAMESPACE

_SR_LIBCPP_POP_MACROS

#endif // _SR_LIBCPP___RANDOM_UNIFORM_INT_DISTRIBUTION_H

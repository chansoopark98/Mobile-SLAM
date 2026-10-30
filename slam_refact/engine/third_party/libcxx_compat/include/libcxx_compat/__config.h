//===----------------------------------------------------------------------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// libcxx_compat configuration header (slam_refact).
//
// Replaces libc++'s <__config> and the small internal helpers that the excerpted algorithm and
// distribution headers reference. The excerpts live in namespace slam_refact::libcxx so that
// the same libc++ (emsdk 5.0.0, LLVM 21.1.8) behaviour is available on any standard library.
// Every macro is renamed from _LIBCPP_* to _SR_LIBCPP_* so this header can be included next to
// the real libc++ (em++ builds) without redefining its macros.
//
// Contents, by origin (paths relative to emsdk upstream/emscripten/system/lib/libcxx/include):
//   - macro shims: values libc++ uses for C++17, hardening mode NONE, no debug randomization.
//   - __type_traits/desugars_to.h, __algorithm/comp.h (__less), __functional/operations.h
//     (desugars_to specializations of less/greater), __functional/ranges_operations.h
//     (ranges::less/greater, provided here for C++17): excerpts.
//   - __algorithm/iterator_operations.h: the _ClassicAlgPolicy members the excerpts call.
//   - __bit/{countl,countr,blsr,bit_log2}.h, __type_traits/make_unsigned.h (__to_unsigned_like):
//     excerpts. __builtin_clzg/__builtin_ctzg (clang >= 19) are expressed with the classic
//     __builtin_clz*/__builtin_ctz* builtins, same values for non-zero input and digits for 0.
//   - __libcpp_is_contiguous_iterator / __unwrap_iter: libc++ recognises its own __wrap_iter and
//     raw pointers. The excerpt recognises raw pointers and the iterators of std::vector and
//     std::basic_string of the standard library in use (libstdc++ or libc++). INFERRED to cover
//     every iterator the engine passes; other iterator types take the generic (non-branchless)
//     path, which libc++ would also take for them.
//===----------------------------------------------------------------------===//

#ifndef SLAM_REFACT_LIBCXX_COMPAT_CONFIG_H
#define SLAM_REFACT_LIBCXX_COMPAT_CONFIG_H

#include <climits>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <iterator>
#include <limits>
#include <memory>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

// ---- macro shims -------------------------------------------------------------------------------
#define _SR_LIBCXX_BEGIN_NAMESPACE \
  namespace slam_refact {          \
  namespace libcxx {
#define _SR_LIBCXX_END_NAMESPACE \
  }                              \
  }

#define _SR_LIBCPP_STD_VER 17
#define _SR_LIBCPP_HIDE_FROM_ABI
#define _SR_LIBCPP_EXPORTED_FROM_ABI
#define _SR_LIBCPP_NODEBUG
#define _SR_LIBCPP_CONSTEXPR constexpr
#define _SR_LIBCPP_CONSTEXPR_SINCE_CXX14 constexpr
#define _SR_LIBCPP_CONSTEXPR_SINCE_CXX20
#define _SR_LIBCPP_CONSTEXPR_SINCE_CXX26
#define _SR_LIBCPP_DEPRECATED_IN_CXX14
#define _SR_LIBCPP_DISABLE_UBSAN_UNSIGNED_INTEGER_CHECK
#define _SR_LIBCPP_PUSH_MACROS
#define _SR_LIBCPP_POP_MACROS
#define _SR_LIBCPP_HAS_WIDE_CHARACTERS 1
#if defined(__SIZEOF_INT128__)
#define _SR_LIBCPP_HAS_INT128 1
#else
#define _SR_LIBCPP_HAS_INT128 0
#endif
// Hardening mode NONE (emscripten __config_site: _LIBCPP_HARDENING_MODE_DEFAULT = NONE).
#define _SR_LIBCPP_ASSERT_VALID_ELEMENT_ACCESS(expr, msg) ((void)0)
#define _SR_LIBCPP_ASSERT_INTERNAL(expr, msg) ((void)0)
#define _SR_LIBCPP_ASSERT_PEDANTIC(expr, msg) ((void)0)

_SR_LIBCXX_BEGIN_NAMESPACE

// ---- names the excerpts use unqualified (libc++ is inside namespace std) ------------------------
using std::bidirectional_iterator_tag;
using std::conditional;
using std::false_type;
using std::greater;
using std::input_iterator_tag;
using std::integral_constant;
using std::intmax_t;
using std::is_arithmetic;
using std::is_reference;
using std::is_same;
using std::iterator_traits;
using std::less;
using std::numeric_limits;
using std::pair;
using std::ptrdiff_t;
using std::random_access_iterator_tag;
using std::size_t;
using std::true_type;
using std::uint32_t;
using std::uint64_t;
using std::uint_fast32_t;
using std::uint_fast64_t;
using std::uintptr_t;

// DECISION (architecture.md §5.2): ptrdiff_t of the original target (wasm32).
using __sr_wasm32_ptrdiff_t = std::int32_t;

template <bool _Bp, class _Tp = void>
using __enable_if_t = typename std::enable_if<_Bp, _Tp>::type;
template <bool _Bp, class _If, class _Then>
using __conditional_t = typename std::conditional<_Bp, _If, _Then>::type;
template <class _Tp>
using __make_unsigned_t = typename std::make_unsigned<_Tp>::type;
template <class _Tp>
using __remove_cvref_t = typename std::remove_cv<typename std::remove_reference<_Tp>::type>::type;
template <class... _Args>
using _Or = std::disjunction<_Args...>;

// ---- __type_traits/desugars_to.h ---------------------------------------------------------------
struct __equal_tag {};
struct __plus_tag {};
struct __less_tag {};
struct __greater_tag {};
struct __totally_ordered_less_tag {};

template <class _CanonicalTag, class _Operation, class... _Args>
inline const bool __desugars_to_v = false;

template <class _CanonicalTag, class _Operation, class... _Args>
inline const bool __desugars_to_v<_CanonicalTag, _Operation const, _Args...> =
    __desugars_to_v<_CanonicalTag, _Operation, _Args...>;
template <class _CanonicalTag, class _Operation, class... _Args>
inline const bool __desugars_to_v<_CanonicalTag, _Operation&, _Args...> =
    __desugars_to_v<_CanonicalTag, _Operation, _Args...>;
template <class _CanonicalTag, class _Operation, class... _Args>
inline const bool __desugars_to_v<_CanonicalTag, _Operation&&, _Args...> =
    __desugars_to_v<_CanonicalTag, _Operation, _Args...>;

// ---- __functional/operations.h (desugars_to specializations of the standard comparators) -------
template <class _Tp>
inline const bool __desugars_to_v<__less_tag, std::less<_Tp>, _Tp, _Tp> = true;
template <class _Tp, class _Up>
inline const bool __desugars_to_v<__less_tag, std::less<>, _Tp, _Up> = true;
template <class _Tp>
inline const bool __desugars_to_v<__greater_tag, std::greater<_Tp>, _Tp, _Tp> = true;
template <class _Tp, class _Up>
inline const bool __desugars_to_v<__greater_tag, std::greater<>, _Tp, _Up> = true;

// ---- __algorithm/comp.h ------------------------------------------------------------------------
// The definition is required because __less is part of the ABI, but it's empty
// because all comparisons should be transparent.
template <class _T1 = void, class _T2 = _T1>
struct __less {};

template <>
struct __less<void, void> {
  template <class _Tp, class _Up>
  _SR_LIBCPP_HIDE_FROM_ABI _SR_LIBCPP_CONSTEXPR_SINCE_CXX14 bool operator()(const _Tp& __lhs, const _Up& __rhs) const {
    return __lhs < __rhs;
  }
};

template <class _Tp>
inline const bool __desugars_to_v<__less_tag, __less<>, _Tp, _Tp> = true;

// ---- __functional/ranges_operations.h (C++20 in libc++; needed by libcxx/src/algorithm.cpp) -----
namespace ranges {
struct less {
  template <class _Tp, class _Up>
  [[nodiscard]] _SR_LIBCPP_HIDE_FROM_ABI constexpr bool operator()(_Tp&& __t, _Up&& __u) const {
    return std::forward<_Tp>(__t) < std::forward<_Up>(__u);
  }
  using is_transparent = void;
};
struct greater {
  template <class _Tp, class _Up>
  [[nodiscard]] _SR_LIBCPP_HIDE_FROM_ABI constexpr bool operator()(_Tp&& __t, _Up&& __u) const {
    return std::forward<_Up>(__u) < std::forward<_Tp>(__t);
  }
  using is_transparent = void;
};
}  // namespace ranges

template <class _Tp, class _Up>
inline const bool __desugars_to_v<__less_tag, ranges::less, _Tp, _Up> = true;
template <class _Tp, class _Up>
inline const bool __desugars_to_v<__greater_tag, ranges::greater, _Tp, _Up> = true;

// ---- __type_traits/is_trivially_copyable.h ----------------------------------------------------
template <class _Tp>
inline const bool __is_cheap_to_copy = std::is_trivially_copyable<_Tp>::value && sizeof(_Tp) <= sizeof(std::intmax_t);

// ---- __algorithm/comp_ref_type.h (hardening mode NONE) ----------------------------------------
template <class _Comp>
using __comp_ref_type = _Comp&;

// ---- __debug_utils (no-ops unless _LIBCPP_DEBUG_RANDOMIZE_UNSPECIFIED_STABILITY / debug mode) ---
template <class _AlgPolicy, class _Iterator, class _Sentinel>
_SR_LIBCPP_HIDE_FROM_ABI _SR_LIBCPP_CONSTEXPR_SINCE_CXX14 void __debug_randomize_range(_Iterator, _Sentinel) {}

template <class _RandomAccessIterator, class _Comp>
_SR_LIBCPP_HIDE_FROM_ABI _SR_LIBCPP_CONSTEXPR_SINCE_CXX14 void
__check_strict_weak_ordering_sorted(_RandomAccessIterator, _RandomAccessIterator, _Comp&) {}

// ---- __type_traits/is_constant_evaluated.h (only consulted at run time here) -------------------
_SR_LIBCPP_HIDE_FROM_ABI inline constexpr bool __libcpp_is_constant_evaluated() noexcept { return false; }

// ---- contiguous iterators and __unwrap_iter (see file comment) ---------------------------------
template <class _Iter, class = void>
struct __sr_is_std_contiguous_container_iterator : false_type {};

template <class _Iter>
struct __sr_is_std_contiguous_container_iterator<_Iter, std::void_t<typename iterator_traits<_Iter>::value_type>>
    : integral_constant<bool,
                        is_same<_Iter, typename std::vector<typename iterator_traits<_Iter>::value_type>::iterator>::value ||
                            is_same<_Iter,
                                    typename std::vector<typename iterator_traits<_Iter>::value_type>::const_iterator>::value ||
                            is_same<_Iter, std::string::iterator>::value || is_same<_Iter, std::string::const_iterator>::value> {};

template <>
struct __sr_is_std_contiguous_container_iterator<std::vector<bool>::iterator, void> : false_type {};
template <>
struct __sr_is_std_contiguous_container_iterator<std::vector<bool>::const_iterator, void> : false_type {};

template <class _Tp>
struct __libcpp_is_contiguous_iterator : __sr_is_std_contiguous_container_iterator<_Tp> {};
template <class _Up>
struct __libcpp_is_contiguous_iterator<_Up*> : true_type {};

template <class _Iter, __enable_if_t<!__sr_is_std_contiguous_container_iterator<_Iter>::value, int> = 0>
_SR_LIBCPP_HIDE_FROM_ABI constexpr _Iter __unwrap_iter(_Iter __i) noexcept {
  return __i;
}
template <class _Iter, __enable_if_t<__sr_is_std_contiguous_container_iterator<_Iter>::value, int> = 0>
_SR_LIBCPP_HIDE_FROM_ABI constexpr auto __unwrap_iter(_Iter __i) noexcept {
  return __i.base();
}

// ---- __algorithm/iterator_operations.h (members used by the excerpts) -------------------------
template <class _AlgPolicy>
struct _IterOps;

struct _ClassicAlgPolicy {};

template <>
struct _IterOps<_ClassicAlgPolicy> {
  template <class _Iter>
  using __deref_t _SR_LIBCPP_NODEBUG = decltype(*std::declval<_Iter&>());

  template <class _Iter>
  using __move_t _SR_LIBCPP_NODEBUG = decltype(std::move(*std::declval<_Iter&>()));

  template <class _Iter>
  _SR_LIBCPP_HIDE_FROM_ABI _SR_LIBCPP_CONSTEXPR_SINCE_CXX14 static void __validate_iter_reference() {
    static_assert(
        is_same<__deref_t<_Iter>, typename iterator_traits<__remove_cvref_t<_Iter> >::reference>::value,
        "It looks like your iterator's `iterator_traits<It>::reference` does not match the return type of "
        "dereferencing the iterator, i.e., calling `*it`. This is undefined behavior according to [input.iterators] "
        "and can lead to dangling reference issues at runtime, so we are flagging this.");
  }

  // iter_move
  template <class _Iter, __enable_if_t<is_reference<__deref_t<_Iter> >::value, int> = 0>
  _SR_LIBCPP_HIDE_FROM_ABI _SR_LIBCPP_CONSTEXPR_SINCE_CXX14 static
      // If the result of dereferencing `_Iter` is a reference type, deduce the result of calling `std::move` on it.
      // Note that the C++03 mode doesn't support `decltype(auto)` as the return type.
      __move_t<_Iter>
      __iter_move(_Iter&& __i) {
    __validate_iter_reference<_Iter>();

    return std::move(*std::forward<_Iter>(__i));
  }

  template <class _Iter, __enable_if_t<!is_reference<__deref_t<_Iter> >::value, int> = 0>
  _SR_LIBCPP_HIDE_FROM_ABI _SR_LIBCPP_CONSTEXPR_SINCE_CXX14 static
      // If the result of dereferencing `_Iter` is a value type, deduce the return value of this function to also be a
      // value -- otherwise, after `operator*` returns a temporary, this function would return a dangling reference to
      // that temporary. Note that the C++03 mode doesn't support `auto` as the return type.
      __deref_t<_Iter>
      __iter_move(_Iter&& __i) {
    __validate_iter_reference<_Iter>();

    return *std::forward<_Iter>(__i);
  }

  // iter_swap
  // [libcxx_compat] libc++'s std::iter_swap is `swap(*__a, *__b)` with ADL, as is the
  // standard library's std::iter_swap used here.
  template <class _Iter1, class _Iter2>
  _SR_LIBCPP_HIDE_FROM_ABI _SR_LIBCPP_CONSTEXPR_SINCE_CXX20 static void iter_swap(_Iter1&& __a, _Iter2&& __b) {
    std::iter_swap(std::forward<_Iter1>(__a), std::forward<_Iter2>(__b));
  }

  // next
  template <class _Iterator>
  _SR_LIBCPP_HIDE_FROM_ABI static _SR_LIBCPP_CONSTEXPR_SINCE_CXX14 _Iterator next(_Iterator, _Iterator __last) {
    return __last;
  }
};

// ---- __bit/countl.h, __bit/countr.h, __bit/blsr.h, __bit/bit_log2.h ----------------------------
template <class _Tp>
_SR_LIBCPP_HIDE_FROM_ABI _SR_LIBCPP_CONSTEXPR_SINCE_CXX14 int __countl_zero(_Tp __t) noexcept {
  static_assert(std::is_unsigned<_Tp>::value && !is_same<_Tp, bool>::value,
                "__countl_zero requires an unsigned integer type");
  // libc++: return __builtin_clzg(__t, numeric_limits<_Tp>::digits);
  if (__t == 0)
    return numeric_limits<_Tp>::digits;
  if (sizeof(_Tp) <= sizeof(unsigned int))
    return __builtin_clz(static_cast<unsigned int>(__t)) - (numeric_limits<unsigned int>::digits - numeric_limits<_Tp>::digits);
  if (sizeof(_Tp) <= sizeof(unsigned long))
    return __builtin_clzl(static_cast<unsigned long>(__t)) - (numeric_limits<unsigned long>::digits - numeric_limits<_Tp>::digits);
  return __builtin_clzll(static_cast<unsigned long long>(__t)) -
         (numeric_limits<unsigned long long>::digits - numeric_limits<_Tp>::digits);
}

template <class _Tp>
[[__nodiscard__]] _SR_LIBCPP_HIDE_FROM_ABI _SR_LIBCPP_CONSTEXPR int __countr_zero(_Tp __t) noexcept {
  static_assert(std::is_unsigned<_Tp>::value && !is_same<_Tp, bool>::value,
                "__countr_zero only works with unsigned types");
  // libc++: return __builtin_ctzg(__t, numeric_limits<_Tp>::digits);
  if (__t == 0)
    return numeric_limits<_Tp>::digits;
  if (sizeof(_Tp) <= sizeof(unsigned int))
    return __builtin_ctz(static_cast<unsigned int>(__t));
  if (sizeof(_Tp) <= sizeof(unsigned long))
    return __builtin_ctzl(static_cast<unsigned long>(__t));
  return __builtin_ctzll(static_cast<unsigned long long>(__t));
}

inline _SR_LIBCPP_HIDE_FROM_ABI _SR_LIBCPP_CONSTEXPR unsigned __libcpp_blsr(unsigned __x) noexcept {
  return __x ^ (__x & -__x);
}

inline _SR_LIBCPP_HIDE_FROM_ABI _SR_LIBCPP_CONSTEXPR unsigned long __libcpp_blsr(unsigned long __x) noexcept {
  return __x ^ (__x & -__x);
}

inline _SR_LIBCPP_HIDE_FROM_ABI _SR_LIBCPP_CONSTEXPR unsigned long long __libcpp_blsr(unsigned long long __x) noexcept {
  return __x ^ (__x & -__x);
}

template <class _Tp>
_SR_LIBCPP_HIDE_FROM_ABI _SR_LIBCPP_CONSTEXPR_SINCE_CXX14 _Tp __bit_log2(_Tp __t) noexcept {
  static_assert(std::is_unsigned<_Tp>::value && !is_same<_Tp, bool>::value,
                "__bit_log2 requires an unsigned integer type");
  _SR_LIBCPP_ASSERT_INTERNAL(__t != 0, "logarithm of 0 is undefined");
  return numeric_limits<_Tp>::digits - 1 - ::slam_refact::libcxx::__countl_zero(__t);
}

// ---- __type_traits/make_unsigned.h -------------------------------------------------------------
template <class _Tp>
_SR_LIBCPP_HIDE_FROM_ABI _SR_LIBCPP_CONSTEXPR __make_unsigned_t<_Tp> __to_unsigned_like(_Tp __x) noexcept {
  return static_cast<__make_unsigned_t<_Tp> >(__x);
}

// ---- __type_traits/invoke.h, __functional/identity.h, misc. static-assert traits ---------------
// [libcxx_compat] Reduced to the call forms the excerpts use (callable object, plain arguments).
template <class _Fp, class... _Args>
_SR_LIBCPP_HIDE_FROM_ABI constexpr decltype(auto) __invoke(_Fp&& __f, _Args&&... __args) {
  return std::forward<_Fp>(__f)(std::forward<_Args>(__args)...);
}

struct __identity {
  template <class _Tp>
  [[__nodiscard__]] _SR_LIBCPP_HIDE_FROM_ABI constexpr _Tp&& operator()(_Tp&& __t) const noexcept {
    return std::forward<_Tp>(__t);
  }
};

template <class _Iter>
struct __has_forward_iterator_category
    : std::is_base_of<std::forward_iterator_tag, typename iterator_traits<_Iter>::iterator_category> {};

template <class _Func, class... _Args>
struct __is_callable : std::is_invocable<_Func, _Args...> {};

// __random/is_valid.h: the excerpts only static_assert these; the engine instantiates the
// distributions with the standard integer/floating types and std::mt19937.
template <class _IntType>
struct __libcpp_random_is_valid_inttype
    : integral_constant<bool, std::is_integral<_IntType>::value && !is_same<_IntType, bool>::value> {};
template <class _RealType>
struct __libcpp_random_is_valid_realtype : std::is_floating_point<_RealType> {};
template <class _Gen>
struct __libcpp_random_is_valid_urng : true_type {};

_SR_LIBCXX_END_NAMESPACE

#endif  // SLAM_REFACT_LIBCXX_COMPAT_CONFIG_H

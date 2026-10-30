// Implementation policies for libcxx_golden_cases.h.
//   StdImpl    : the standard library in use (libc++ under em++ -> golden generator).
//   CompatImpl : slam_refact::libcxx (third_party/libcxx_compat).
#ifndef SLAM_REFACT_TESTS_COMPAT_LIBCXX_GOLDEN_IMPLS_H_
#define SLAM_REFACT_TESTS_COMPAT_LIBCXX_GOLDEN_IMPLS_H_

#include <algorithm>
#include <cstddef>
#include <queue>
#include <random>
#include <vector>

#include "libcxx_compat/algorithm.h"
#include "libcxx_compat/queue.h"
#include "libcxx_compat/random.h"

namespace slam_refact {
namespace golden {

struct StdImpl {
  static constexpr const char* kName = "std";
  template <class It>
  static void Sort(It f, It l) { std::sort(f, l); }
  template <class It, class C>
  static void Sort(It f, It l, C c) { std::sort(f, l, c); }
  template <class It>
  static void NthElement(It f, It n, It l) { std::nth_element(f, n, l); }
  template <class It, class C>
  static void NthElement(It f, It n, It l, C c) { std::nth_element(f, n, l, c); }
  template <class It>
  static void PartialSort(It f, It m, It l) { std::partial_sort(f, m, l); }
  template <class It, class C>
  static void PartialSort(It f, It m, It l, C c) { std::partial_sort(f, m, l, c); }
  template <class It>
  static void MakeHeap(It f, It l) { std::make_heap(f, l); }
  template <class It, class C>
  static void MakeHeap(It f, It l, C c) { std::make_heap(f, l, c); }
  template <class It>
  static void SortHeap(It f, It l) { std::sort_heap(f, l); }
  template <class It, class C>
  static void SortHeap(It f, It l, C c) { std::sort_heap(f, l, c); }
  template <class It>
  static void PushHeap(It f, It l) { std::push_heap(f, l); }
  template <class It, class C>
  static void PushHeap(It f, It l, C c) { std::push_heap(f, l, c); }
  template <class It>
  static void PopHeap(It f, It l) { std::pop_heap(f, l); }
  template <class It, class C>
  static void PopHeap(It f, It l, C c) { std::pop_heap(f, l, c); }
  template <class It, class G>
  static void Shuffle(It f, It l, G& g) { std::shuffle(f, l, g); }
  template <class R, size_t B, class G>
  static R Canonical(G& g) { return std::generate_canonical<R, B>(g); }
  template <class T>
  using UniformInt = std::uniform_int_distribution<T>;
  template <class T>
  using UniformReal = std::uniform_real_distribution<T>;
  template <class T>
  using Normal = std::normal_distribution<T>;
  template <class T, class C>
  using PriorityQueue = std::priority_queue<T, std::vector<T>, C>;
};

struct CompatImpl {
  static constexpr const char* kName = "compat";
  template <class It>
  static void Sort(It f, It l) { libcxx::sort(f, l); }
  template <class It, class C>
  static void Sort(It f, It l, C c) { libcxx::sort(f, l, c); }
  template <class It>
  static void NthElement(It f, It n, It l) { libcxx::nth_element(f, n, l); }
  template <class It, class C>
  static void NthElement(It f, It n, It l, C c) { libcxx::nth_element(f, n, l, c); }
  template <class It>
  static void PartialSort(It f, It m, It l) { libcxx::partial_sort(f, m, l); }
  template <class It, class C>
  static void PartialSort(It f, It m, It l, C c) { libcxx::partial_sort(f, m, l, c); }
  template <class It>
  static void MakeHeap(It f, It l) { libcxx::make_heap(f, l); }
  template <class It, class C>
  static void MakeHeap(It f, It l, C c) { libcxx::make_heap(f, l, c); }
  template <class It>
  static void SortHeap(It f, It l) { libcxx::sort_heap(f, l); }
  template <class It, class C>
  static void SortHeap(It f, It l, C c) { libcxx::sort_heap(f, l, c); }
  template <class It>
  static void PushHeap(It f, It l) { libcxx::push_heap(f, l); }
  template <class It, class C>
  static void PushHeap(It f, It l, C c) { libcxx::push_heap(f, l, c); }
  template <class It>
  static void PopHeap(It f, It l) { libcxx::pop_heap(f, l); }
  template <class It, class C>
  static void PopHeap(It f, It l, C c) { libcxx::pop_heap(f, l, c); }
  template <class It, class G>
  static void Shuffle(It f, It l, G& g) { libcxx::shuffle(f, l, g); }
  template <class R, size_t B, class G>
  static R Canonical(G& g) { return libcxx::generate_canonical<R, B>(g); }
  template <class T>
  using UniformInt = libcxx::uniform_int_distribution<T>;
  template <class T>
  using UniformReal = libcxx::uniform_real_distribution<T>;
  template <class T>
  using Normal = libcxx::normal_distribution<T>;
  template <class T, class C>
  using PriorityQueue = libcxx::priority_queue<T, std::vector<T>, C>;
};

}  // namespace golden
}  // namespace slam_refact

#endif  // SLAM_REFACT_TESTS_COMPAT_LIBCXX_GOLDEN_IMPLS_H_

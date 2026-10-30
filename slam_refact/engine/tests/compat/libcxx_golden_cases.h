// Golden cases for slam_refact::libcxx (third_party/libcxx_compat).
//
// RunGoldenCases<Impl>() runs a fixed, deterministic set of inputs through the algorithms and
// distributions of `Impl` and returns one FNV-1a digest per case. The golden file
// tests/data/compat/libcxx-golden.json is RunGoldenCases<StdImpl> built with em++ (emsdk 5.0.0
// libc++); the unit test checks RunGoldenCases<CompatImpl> against it on the host toolchain.
//
// Inputs come from a local LCG (not from <random>) so they do not depend on the library under
// test. Records carry their original index, so the digest sees the order of equivalent elements
// (tie order), and float keys include -0.0f/+0.0f so the arithmetic (branchless) paths expose
// tie order through bit patterns as well.
#ifndef SLAM_REFACT_TESTS_COMPAT_LIBCXX_GOLDEN_CASES_H_
#define SLAM_REFACT_TESTS_COMPAT_LIBCXX_GOLDEN_CASES_H_

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <functional>
#include <limits>
#include <random>
#include <string>
#include <utility>
#include <vector>

namespace slam_refact {
namespace golden {

struct Record {
  int32_t key;
  int32_t idx;
};

struct RecordKeyLess {
  bool operator()(const Record& a, const Record& b) const { return a.key < b.key; }
};
struct RecordKeyGreater {
  bool operator()(const Record& a, const Record& b) const { return a.key > b.key; }
};
// Same shape as c8::KeypointResponsePairMore (reality/engine/features/keypoint-queue.h).
struct PairFirstLessSecondMore {
  bool operator()(const std::pair<int, Record>& a, const std::pair<int, Record>& b) const {
    return a.first != b.first ? a.first < b.first : a.second.key > b.second.key;
  }
};

class Digest {
 public:
  void Bytes(const void* data, size_t n) {
    const auto* p = static_cast<const uint8_t*>(data);
    for (size_t i = 0; i < n; ++i) {
      h_ ^= p[i];
      h_ *= 1099511628211ull;
    }
  }
  template <class T>
  void Value(const T& v) {
    Bytes(&v, sizeof(T));
  }
  void Record(const golden::Record& r) {
    Value(r.key);
    Value(r.idx);
  }
  uint64_t value() const { return h_; }

 private:
  uint64_t h_ = 1469598103934665603ull;
};

struct Case {
  std::string name;
  uint64_t digest;
};

class Lcg {
 public:
  explicit Lcg(uint32_t seed) : s_(seed) {}
  uint32_t Next() {
    s_ = s_ * 1664525u + 1013904223u;
    return s_ >> 8;
  }

 private:
  uint32_t s_;
};

enum class Pattern { kRandomFewKeys, kRandomManyKeys, kAscending, kDescending, kAllEqual, kOrganPipe, kSawtooth };

inline const char* PatternName(Pattern p) {
  switch (p) {
    case Pattern::kRandomFewKeys: return "random_few";
    case Pattern::kRandomManyKeys: return "random_many";
    case Pattern::kAscending: return "ascending";
    case Pattern::kDescending: return "descending";
    case Pattern::kAllEqual: return "all_equal";
    case Pattern::kOrganPipe: return "organ_pipe";
    case Pattern::kSawtooth: return "sawtooth";
  }
  return "?";
}

inline const std::vector<Pattern>& AllPatterns() {
  static const std::vector<Pattern> kPatterns = {Pattern::kRandomFewKeys, Pattern::kRandomManyKeys,
                                                 Pattern::kAscending,     Pattern::kDescending,
                                                 Pattern::kAllEqual,      Pattern::kOrganPipe,
                                                 Pattern::kSawtooth};
  return kPatterns;
}

// Sizes cover the insertion-sort cut-offs (<= 5, < 24 for sort, <= 7 for nth_element), the
// ninther threshold (128), the 64-element bitset blocks, and larger inputs.
inline const std::vector<int>& AllSizes() {
  static const std::vector<int> kSizes = {0,  1,  2,   3,   4,   5,   6,   7,   8,   9,    13,   23,   24,  25,
                                          31, 64, 100, 127, 128, 129, 130, 200, 256, 513, 1000, 2047, 5000};
  return kSizes;
}

inline std::vector<Record> MakeRecords(Pattern p, int n, uint32_t seed) {
  Lcg lcg(seed);
  std::vector<Record> out(static_cast<size_t>(n));
  for (int i = 0; i < n; ++i) {
    int32_t key = 0;
    switch (p) {
      case Pattern::kRandomFewKeys: key = static_cast<int32_t>(lcg.Next() % 5u) - 2; break;
      case Pattern::kRandomManyKeys: key = static_cast<int32_t>(lcg.Next() % 100000u) - 50000; break;
      case Pattern::kAscending: key = i / 3; break;
      case Pattern::kDescending: key = (n - i) / 3; break;
      case Pattern::kAllEqual: key = 7; break;
      case Pattern::kOrganPipe: key = i < n / 2 ? i : n - i; break;
      case Pattern::kSawtooth: key = i % 17; break;
    }
    out[static_cast<size_t>(i)] = Record{key, i};
  }
  return out;
}

// key 0 alternates between +0.0f and -0.0f by original index; other keys scale by 0.25.
inline float KeyToFloat(const Record& r) {
  if (r.key == 0) return (r.idx & 1) ? -0.0f : 0.0f;
  return static_cast<float>(r.key) * 0.25f;
}

template <class T>
inline void DigestFloats(Digest* d, const std::vector<T>& v) {
  for (const T& x : v) {
    unsigned char bits[sizeof(T)];
    std::memcpy(bits, &x, sizeof(T));
    d->Bytes(bits, sizeof(T));
  }
}

// ---- element-type adapters ---------------------------------------------------------------------
// Each adapter builds a vector from records, runs `op(first, last, comp_or_none)` and digests.
enum class ElemKind { kRecLess, kRecGreater, kPairKeypoint, kF32StdLess, kF32Default, kI32StdGreater, kU8Default,
                      kF64Lambda };

inline const char* ElemName(ElemKind k) {
  switch (k) {
    case ElemKind::kRecLess: return "rec_less";
    case ElemKind::kRecGreater: return "rec_greater";
    case ElemKind::kPairKeypoint: return "pair_keypoint";
    case ElemKind::kF32StdLess: return "f32_std_less";
    case ElemKind::kF32Default: return "f32_default";
    case ElemKind::kI32StdGreater: return "i32_std_greater";
    case ElemKind::kU8Default: return "u8_default";
    case ElemKind::kF64Lambda: return "f64_lambda";
  }
  return "?";
}

inline const std::vector<ElemKind>& AllElemKinds() {
  static const std::vector<ElemKind> kKinds = {ElemKind::kRecLess,     ElemKind::kRecGreater,    ElemKind::kPairKeypoint,
                                               ElemKind::kF32StdLess,  ElemKind::kF32Default,    ElemKind::kI32StdGreater,
                                               ElemKind::kU8Default,   ElemKind::kF64Lambda};
  return kKinds;
}

// Calls fn(vector&, comparator-or-tag) for the element kind and digests the vector afterwards.
struct NoComp {};

template <class Fn>
void WithElems(ElemKind kind, const std::vector<Record>& recs, Digest* d, Fn fn) {
  switch (kind) {
    case ElemKind::kRecLess: {
      std::vector<Record> v = recs;
      fn(v, RecordKeyLess());
      for (const auto& r : v) d->Record(r);
      break;
    }
    case ElemKind::kRecGreater: {
      std::vector<Record> v = recs;
      fn(v, RecordKeyGreater());
      for (const auto& r : v) d->Record(r);
      break;
    }
    case ElemKind::kPairKeypoint: {
      std::vector<std::pair<int, Record>> v;
      v.reserve(recs.size());
      for (const auto& r : recs) v.emplace_back(r.idx % 13, r);
      fn(v, PairFirstLessSecondMore());
      for (const auto& p : v) {
        d->Value(p.first);
        d->Record(p.second);
      }
      break;
    }
    case ElemKind::kF32StdLess: {
      std::vector<float> v;
      for (const auto& r : recs) v.push_back(KeyToFloat(r));
      fn(v, std::less<float>());
      DigestFloats(d, v);
      break;
    }
    case ElemKind::kF32Default: {
      std::vector<float> v;
      for (const auto& r : recs) v.push_back(KeyToFloat(r));
      fn(v, NoComp());
      DigestFloats(d, v);
      break;
    }
    case ElemKind::kI32StdGreater: {
      std::vector<int32_t> v;
      for (const auto& r : recs) v.push_back(r.key);
      fn(v, std::greater<int32_t>());
      for (auto x : v) d->Value(x);
      break;
    }
    case ElemKind::kU8Default: {
      std::vector<uint8_t> v;
      for (const auto& r : recs) v.push_back(static_cast<uint8_t>(r.key & 0xff));
      fn(v, NoComp());
      for (auto x : v) d->Value(x);
      break;
    }
    case ElemKind::kF64Lambda: {
      std::vector<double> v;
      for (const auto& r : recs) v.push_back(static_cast<double>(KeyToFloat(r)));
      fn(v, [](double a, double b) { return a < b; });
      DigestFloats(d, v);
      break;
    }
  }
}

// ---- Impl dispatch helpers (comparator vs. default) -------------------------------------------
template <class Impl>
struct Ops {
  template <class It, class C>
  static void Sort(It f, It l, C c) { Impl::Sort(f, l, c); }
  template <class It>
  static void Sort(It f, It l, NoComp) { Impl::Sort(f, l); }
  template <class It, class C>
  static void Nth(It f, It n, It l, C c) { Impl::NthElement(f, n, l, c); }
  template <class It>
  static void Nth(It f, It n, It l, NoComp) { Impl::NthElement(f, n, l); }
  template <class It, class C>
  static void Partial(It f, It m, It l, C c) { Impl::PartialSort(f, m, l, c); }
  template <class It>
  static void Partial(It f, It m, It l, NoComp) { Impl::PartialSort(f, m, l); }
  template <class It, class C>
  static void MakeHeap(It f, It l, C c) { Impl::MakeHeap(f, l, c); }
  template <class It>
  static void MakeHeap(It f, It l, NoComp) { Impl::MakeHeap(f, l); }
  template <class It, class C>
  static void SortHeap(It f, It l, C c) { Impl::SortHeap(f, l, c); }
  template <class It>
  static void SortHeap(It f, It l, NoComp) { Impl::SortHeap(f, l); }
  template <class It, class C>
  static void PushHeap(It f, It l, C c) { Impl::PushHeap(f, l, c); }
  template <class It>
  static void PushHeap(It f, It l, NoComp) { Impl::PushHeap(f, l); }
  template <class It, class C>
  static void PopHeap(It f, It l, C c) { Impl::PopHeap(f, l, c); }
  template <class It>
  static void PopHeap(It f, It l, NoComp) { Impl::PopHeap(f, l); }
};

template <class Impl>
void AddAlgorithmCases(std::vector<Case>* cases) {
  using O = Ops<Impl>;
  for (ElemKind kind : AllElemKinds()) {
    for (Pattern pat : AllPatterns()) {
      const std::string suffix = std::string("/") + ElemName(kind) + "/" + PatternName(pat);
      Digest sort_d, nth_d, partial_d, heap_d, push_pop_d;
      for (int n : AllSizes()) {
        const std::vector<Record> recs = MakeRecords(pat, n, 0x5eed0000u + static_cast<uint32_t>(n));
        WithElems(kind, recs, &sort_d, [](auto& v, auto c) { O::Sort(v.begin(), v.end(), c); });
        for (int nth : {0, n / 3, n / 2, n - 1, n}) {
          if (nth < 0) continue;
          WithElems(kind, recs, &nth_d,
                    [nth](auto& v, auto c) { O::Nth(v.begin(), v.begin() + nth, v.end(), c); });
        }
        for (int mid : {0, 1, n / 4, n}) {
          if (mid > n) continue;
          WithElems(kind, recs, &partial_d,
                    [mid](auto& v, auto c) { O::Partial(v.begin(), v.begin() + mid, v.end(), c); });
        }
        // make_heap state, then sort_heap result.
        WithElems(kind, recs, &heap_d, [&heap_d](auto& v, auto c) {
          O::MakeHeap(v.begin(), v.end(), c);
          heap_d.Value(static_cast<uint64_t>(v.size()));
          O::SortHeap(v.begin(), v.end(), c);
        });
        // Incremental push_heap of every element, then pop_heap all but the last half.
        WithElems(kind, recs, &push_pop_d, [](auto& v, auto c) {
          for (size_t i = 1; i <= v.size(); ++i) O::PushHeap(v.begin(), v.begin() + static_cast<std::ptrdiff_t>(i), c);
          for (size_t len = v.size(); len > v.size() / 2; --len)
            O::PopHeap(v.begin(), v.begin() + static_cast<std::ptrdiff_t>(len), c);
        });
      }
      cases->push_back({"sort" + suffix, sort_d.value()});
      cases->push_back({"nth_element" + suffix, nth_d.value()});
      cases->push_back({"partial_sort" + suffix, partial_d.value()});
      cases->push_back({"make_sort_heap" + suffix, heap_d.value()});
      cases->push_back({"push_pop_heap" + suffix, push_pop_d.value()});
    }
  }
}

template <class Impl>
void AddPriorityQueueCases(std::vector<Case>* cases) {
  for (Pattern pat : AllPatterns()) {
    Digest d;
    for (int n : AllSizes()) {
      const std::vector<Record> recs = MakeRecords(pat, n, 0x9e3779b9u + static_cast<uint32_t>(n));
      typename Impl::template PriorityQueue<Record, RecordKeyGreater> q;
      for (const auto& r : recs) q.push(r);
      while (!q.empty()) {
        d.Record(q.top());
        q.pop();
      }
    }
    cases->push_back({std::string("priority_queue/rec_greater/") + PatternName(pat), d.value()});
  }
}

template <class Impl>
void AddShuffleCases(std::vector<Case>* cases) {
  std::mt19937 rng;  // default seed 5489, as c8::RandomNumbers and FrameFrameTracker
  std::vector<int> lengths;
  for (int n = 0; n <= 70; ++n) lengths.push_back(n);
  for (int n : {100, 127, 128, 255, 256, 257, 1000, 1023, 1024, 4096, 5000}) lengths.push_back(n);
  for (int n : lengths) {
    std::vector<int32_t> v(static_cast<size_t>(n));
    for (int i = 0; i < n; ++i) v[static_cast<size_t>(i)] = i;
    Impl::Shuffle(v.begin(), v.end(), rng);
    Digest d;
    for (auto x : v) d.Value(x);
    const uint32_t next_raw = static_cast<uint32_t>(rng());  // exposes the number of draws used
    d.Value(next_raw);
    cases->push_back({"shuffle/mt19937_5489/n" + std::to_string(n), d.value()});
  }
  // Records (non-trivial element type through the same code path), fresh engine.
  std::mt19937 rng2(12345u);
  Digest d2;
  for (int n : AllSizes()) {
    std::vector<Record> v = MakeRecords(Pattern::kRandomManyKeys, n, 77u);
    Impl::Shuffle(v.begin(), v.end(), rng2);
    for (const auto& r : v) d2.Record(r);
  }
  d2.Value(static_cast<uint32_t>(rng2()));
  cases->push_back({"shuffle/mt19937_12345/records", d2.value()});
}

template <class Impl, class T>
uint64_t DigestUniformInt(std::mt19937* rng, T a, T b, int draws) {
  typename Impl::template UniformInt<T> dist(a, b);
  Digest d;
  for (int i = 0; i < draws; ++i) d.Value(dist(*rng));
  d.Value(static_cast<uint32_t>((*rng)()));
  return d.value();
}

template <class Impl>
void AddDistributionCases(std::vector<Case>* cases) {
  std::mt19937 rng;  // 5489
  {
    typename Impl::template UniformInt<uint32_t> dist;
    Digest d;
    for (int i = 0; i < 64; ++i) d.Value(dist(rng));
    d.Value(static_cast<uint32_t>(rng()));
    cases->push_back({"uniform_int/u32/full", d.value()});
  }
  {
    typename Impl::template UniformInt<uint64_t> dist;
    Digest d;
    for (int i = 0; i < 64; ++i) d.Value(dist(rng));
    d.Value(static_cast<uint32_t>(rng()));
    cases->push_back({"uniform_int/u64/full", d.value()});
  }
  for (int32_t b : {1, 2, 3, 5, 7, 10, 100, 1000, 65535, 65536, 1 << 30, std::numeric_limits<int32_t>::max()}) {
    cases->push_back({"uniform_int/i32/0_" + std::to_string(b), DigestUniformInt<Impl, int32_t>(&rng, 0, b, 32)});
  }
  cases->push_back({"uniform_int/i32/-5_5", DigestUniformInt<Impl, int32_t>(&rng, -5, 5, 32)});
  cases->push_back({"uniform_int/i64/0_2^40", DigestUniformInt<Impl, int64_t>(&rng, 0, int64_t(1) << 40, 32)});
  cases->push_back({"uniform_int/u64/0_1e12", DigestUniformInt<Impl, uint64_t>(&rng, 0, 1000000000000ull, 32)});
  {
    Digest d;
    for (int i = 0; i < 64; ++i) d.Value(Impl::template Canonical<float, std::numeric_limits<float>::digits>(rng));
    d.Value(static_cast<uint32_t>(rng()));
    cases->push_back({"generate_canonical/f32_24", d.value()});
  }
  {
    Digest d;
    for (int i = 0; i < 64; ++i) d.Value(Impl::template Canonical<double, std::numeric_limits<double>::digits>(rng));
    d.Value(static_cast<uint32_t>(rng()));
    cases->push_back({"generate_canonical/f64_53", d.value()});
  }
  {
    typename Impl::template UniformReal<float> dist;
    Digest d;
    for (int i = 0; i < 256; ++i) d.Value(dist(rng));
    d.Value(static_cast<uint32_t>(rng()));
    cases->push_back({"uniform_real/f32/0_1", d.value()});
  }
  {
    typename Impl::template UniformReal<float> dist(-1.0f, 1.0f);
    Digest d;
    for (int i = 0; i < 64; ++i) d.Value(dist(rng));
    d.Value(static_cast<uint32_t>(rng()));
    cases->push_back({"uniform_real/f32/-1_1", d.value()});
  }
  {
    typename Impl::template UniformReal<double> dist(0.0, 10.0);
    Digest d;
    for (int i = 0; i < 64; ++i) d.Value(dist(rng));
    d.Value(static_cast<uint32_t>(rng()));
    cases->push_back({"uniform_real/f64/0_10", d.value()});
  }
  {
    // Exact values depend on libm sqrtf/logf (musl vs glibc, architecture.md §5.1); the draw
    // count and acceptance sequence do not. Both are part of this digest.
    typename Impl::template Normal<float> dist;
    Digest d;
    for (int i = 0; i < 64; ++i) d.Value(dist(rng));
    d.Value(static_cast<uint32_t>(rng()));
    cases->push_back({"normal/f32/0_1", d.value()});
  }
  {
    typename Impl::template Normal<double> dist(2.0, 3.0);
    Digest d;
    for (int i = 0; i < 32; ++i) d.Value(dist(rng));
    d.Value(static_cast<uint32_t>(rng()));
    cases->push_back({"normal/f64/2_3", d.value()});
  }
}

template <class Impl>
std::vector<Case> RunGoldenCases() {
  std::vector<Case> cases;
  AddAlgorithmCases<Impl>(&cases);
  AddPriorityQueueCases<Impl>(&cases);
  AddShuffleCases<Impl>(&cases);
  AddDistributionCases<Impl>(&cases);
  return cases;
}

}  // namespace golden
}  // namespace slam_refact

#endif  // SLAM_REFACT_TESTS_COMPAT_LIBCXX_GOLDEN_CASES_H_

#include "ml/kernels.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>

#include "ml/gemm.h"

namespace slam_refact {
namespace ml {
namespace kernels {
namespace {

constexpr int kMaxRank = 6;

inline float Clamp(float v, ActivationRange act) {
  return std::min(std::max(v, act.min), act.max);
}

std::vector<size_t> Strides(const Shape& shape) {
  std::vector<size_t> s(shape.size(), 1);
  for (int i = static_cast<int>(shape.size()) - 2; i >= 0; --i) {
    s[i] = s[i + 1] * static_cast<size_t>(shape[i + 1]);
  }
  return s;
}

// Broadcast driver: calls f(a_ptr, b_ptr, out_ptr, n, a_step, b_step) on rows of the last axis.
template <typename T, typename RowFn>
void BroadcastRows(const T* a, const Shape& a_shape, const T* b, const Shape& b_shape, T* out,
                   const Shape& out_shape, RowFn row_fn) {
  const int rank = static_cast<int>(out_shape.size());
  if (rank == 0) {
    row_fn(a, b, out, size_t{1}, size_t{0}, size_t{0});
    return;
  }
  // Right-align operand shapes to the output rank.
  size_t a_stride[kMaxRank];
  size_t b_stride[kMaxRank];
  {
    Shape ap(rank, 1);
    Shape bp(rank, 1);
    for (size_t i = 0; i < a_shape.size(); ++i) ap[rank - a_shape.size() + i] = a_shape[i];
    for (size_t i = 0; i < b_shape.size(); ++i) bp[rank - b_shape.size() + i] = b_shape[i];
    const std::vector<size_t> as = Strides(ap);
    const std::vector<size_t> bs = Strides(bp);
    for (int d = 0; d < rank; ++d) {
      a_stride[d] = ap[d] == 1 && out_shape[d] != 1 ? 0 : as[d];
      b_stride[d] = bp[d] == 1 && out_shape[d] != 1 ? 0 : bs[d];
    }
  }
  const size_t inner = static_cast<size_t>(out_shape[rank - 1]);
  const size_t total = NumElements(out_shape);
  if (inner == 0 || total == 0) return;
  const size_t outer = total / inner;
  // Fast path: one operand spans the output contiguously and the other repeats a single row (or
  // a scalar) for every row, e.g. PRELU alpha [1,1,C] or a per-channel [1,1,1,C] constant.
  bool a_full = a_stride[rank - 1] == 1 || inner == 1;
  bool b_full = b_stride[rank - 1] == 1 || inner == 1;
  bool a_row = true;
  bool b_row = true;
  for (int d = 0; d < rank - 1; ++d) {
    const size_t dense = NumElements(Shape(out_shape.begin() + d + 1, out_shape.end()));
    a_full = a_full && (a_stride[d] == dense || out_shape[d] == 1);
    b_full = b_full && (b_stride[d] == dense || out_shape[d] == 1);
    a_row = a_row && a_stride[d] == 0;
    b_row = b_row && b_stride[d] == 0;
  }
  if (a_full && b_full) {
    row_fn(a, b, out, total, size_t{1}, size_t{1});
    return;
  }
  if ((a_full && b_row) || (a_row && b_full)) {
    for (size_t o = 0; o < outer; ++o) {
      row_fn(a + (a_full ? o * inner : 0), b + (b_full ? o * inner : 0), out + o * inner, inner,
             a_stride[rank - 1], b_stride[rank - 1]);
    }
    return;
  }
  int idx[kMaxRank] = {0, 0, 0, 0, 0, 0};
  for (size_t o = 0; o < outer; ++o) {
    size_t a_off = 0;
    size_t b_off = 0;
    for (int d = 0; d < rank - 1; ++d) {
      a_off += static_cast<size_t>(idx[d]) * a_stride[d];
      b_off += static_cast<size_t>(idx[d]) * b_stride[d];
    }
    row_fn(a + a_off, b + b_off, out + o * inner, inner, a_stride[rank - 1], b_stride[rank - 1]);
    for (int d = rank - 2; d >= 0; --d) {
      if (++idx[d] < out_shape[d]) break;
      idx[d] = 0;
    }
  }
}

// Row loop specialized on operand steps so the common cases vectorize.
template <typename T, typename F>
void RowApply(const T* a, const T* b, T* out, size_t n, size_t as, size_t bs, F f) {
  if (as == 1 && bs == 1) {
    for (size_t i = 0; i < n; ++i) out[i] = f(a[i], b[i]);
  } else if (as == 1 && bs == 0) {
    const T bv = b[0];
    for (size_t i = 0; i < n; ++i) out[i] = f(a[i], bv);
  } else if (as == 0 && bs == 1) {
    const T av = a[0];
    for (size_t i = 0; i < n; ++i) out[i] = f(av, b[i]);
  } else {
    for (size_t i = 0; i < n; ++i) out[i] = f(a[i * as], b[i * bs]);
  }
}

template <typename T>
inline T FloorModInt(T x, T y) {
  // TFLite floor_mod.h FloorMod for integers: trunc mod, then shift into the divisor's sign.
  // x mod -1 is 0 for every x; handling it here avoids evaluating INT_MIN % -1 (a trap on x86 and
  // WASM). The divisor 0 is rejected by the callers.
  if (y == -1) return 0;
  const T trunc_mod = x % y;
  return (trunc_mod != 0) && ((y < 0) != (trunc_mod < 0)) ? trunc_mod + y : trunc_mod;
}

inline float FloorModFloat(float x, float y) {
  // TFLite floor_mod.h FloorMod for floats (std::fmod based).
  const float trunc_mod = std::fmod(x, y);
  return (trunc_mod != 0) && ((y < 0) != (trunc_mod < 0)) ? trunc_mod + y : trunc_mod;
}

void ComputeInterpolationValues(float value, float scale, bool half_pixel_centers, int input_size,
                                float* scaled_value, int* lower_bound, int* upper_bound) {
  // reference_ops.h ComputeInterpolationValues.
  if (half_pixel_centers) {
    *scaled_value = (value + 0.5f) * scale - 0.5f;
  } else {
    *scaled_value = value * scale;
  }
  const float scaled_value_floor = std::floor(*scaled_value);
  // The reference has no upper bound on lower_bound. In exact arithmetic floor(value * scale) <=
  // input_size - 1, but the float product can round up to input_size (reviewer reproduction: input
  // [1,1,1,1], size [1, 2^25], or an output width of 13,103,685 for any input width of 1 or 2), which
  // then reads one element past the row (RM-1b, review MAJOR-1). The clamp changes nothing while
  // the index is in range, i.e. for every result the reference computes without an out-of-bounds
  // read; there it replicates the border element (dy then is >= 1 and both rows are the same).
  *lower_bound = std::min(std::max(static_cast<int>(scaled_value_floor), 0), input_size - 1);
  *upper_bound = std::min(static_cast<int>(std::ceil(*scaled_value)), input_size - 1);
}

// Maps every input element to its reduced output element (reduced axes collapse to index 0).
template <typename F>
void ReduceGeneric(const float* in, const Shape& in_shape, const std::vector<int>& axes,
                   float* out, F f) {
  const int rank = static_cast<int>(in_shape.size());
  bool reduced[kMaxRank] = {false, false, false, false, false, false};
  for (int a : axes) reduced[a] = true;
  // Output strides over the non-reduced axes, in input axis order.
  size_t out_stride[kMaxRank];
  size_t s = 1;
  for (int d = rank - 1; d >= 0; --d) {
    out_stride[d] = reduced[d] ? 0 : s;
    if (!reduced[d]) s *= static_cast<size_t>(in_shape[d]);
  }
  const size_t total = NumElements(in_shape);
  if (rank == 0) {
    out[0] = f(out[0], in[0]);
    return;
  }
  // Rows of the innermost axis; inputs are visited in memory order, so every output element
  // accumulates its inputs in the same order as an element-by-element walk.
  const size_t inner = static_cast<size_t>(in_shape[rank - 1]);
  if (inner == 0 || total == 0) return;
  const bool inner_reduced = reduced[rank - 1];
  int idx[kMaxRank] = {0, 0, 0, 0, 0, 0};
  for (size_t row = 0; row < total / inner; ++row) {
    size_t o = 0;
    for (int d = 0; d < rank - 1; ++d) o += static_cast<size_t>(idx[d]) * out_stride[d];
    const float* src = in + row * inner;
    if (inner_reduced) {
      for (size_t i = 0; i < inner; ++i) out[o] = f(out[o], src[i]);
    } else {
      float* dst = out + o;
      for (size_t i = 0; i < inner; ++i) dst[i] = f(dst[i], src[i]);
    }
    for (int d = rank - 2; d >= 0; --d) {
      if (++idx[d] < in_shape[d]) break;
      idx[d] = 0;
    }
  }
}

}  // namespace

size_t NumElements(const Shape& shape) {
  size_t n = 1;
  for (int32_t d : shape) n *= static_cast<size_t>(d);
  return n;
}

bool CheckedNumElements(const Shape& shape, size_t* count) {
  uint64_t n = 1;
  for (int32_t d : shape) {
    if (d < 0) return false;
    n *= static_cast<uint64_t>(d);  // n <= 2^26 and d < 2^31: no uint64 overflow
    if (n > kMaxTensorElements) return false;
  }
  *count = static_cast<size_t>(n);
  return true;
}

bool CheckedMul(size_t a, size_t b, size_t limit, size_t* out) {
  const uint64_t p = static_cast<uint64_t>(a) * static_cast<uint64_t>(b);
  if (a != 0 && p / a != b) return false;  // 64-bit wrap (only if a and b are both > 2^32)
  if (p > limit) return false;
  *out = static_cast<size_t>(p);
  return true;
}

bool RangeFor(Activation activation, ActivationRange* out) {
  const float inf = std::numeric_limits<float>::infinity();
  switch (activation) {
    case Activation::kNone:
      *out = {-inf, inf};
      return true;
    case Activation::kRelu:
      *out = {0.0f, inf};
      return true;
    case Activation::kReluN1To1:
      *out = {-1.0f, 1.0f};
      return true;
    case Activation::kRelu6:
      *out = {0.0f, 6.0f};
      return true;
    case Activation::kTanh:
    case Activation::kSignBit:
      return false;
  }
  return false;
}

int ComputeOutSize(Padding padding, int in_size, int filter, int stride, int dilation) {
  // 64-bit intermediates: the callers bound stride, dilation and the effective filter extent
  // (kMaxGeometryParam), but this function must not overflow for any int arguments.
  const int64_t effective = (static_cast<int64_t>(filter) - 1) * dilation + 1;
  if (stride <= 0) return 0;
  int64_t out;
  if (padding == Padding::kSame) {
    out = (static_cast<int64_t>(in_size) + stride - 1) / stride;
  } else {
    out = (static_cast<int64_t>(in_size) + stride - effective) / stride;
  }
  return static_cast<int>(std::max<int64_t>(std::min<int64_t>(out, INT32_MAX), INT32_MIN));
}

int ComputePadBefore(Padding padding, int in_size, int out_size, int filter, int stride,
                     int dilation) {
  (void)padding;  // VALID: total padding is <= 0 for its output size, giving 0 (padding.h).
  const int64_t effective = (static_cast<int64_t>(filter) - 1) * dilation + 1;
  const int64_t total = (static_cast<int64_t>(out_size) - 1) * stride + effective - in_size;
  return total > 0 ? static_cast<int>(std::min<int64_t>(total / 2, INT32_MAX)) : 0;
}

bool Conv2DIsPointwise(const ConvGeometry& g) {
  return g.kernel_h == 1 && g.kernel_w == 1 && g.stride_h == 1 && g.stride_w == 1 &&
         g.pad_top == 0 && g.pad_left == 0 && g.out_h == g.in_h && g.out_w == g.in_w;
}

void Conv2D(const ConvGeometry& g, const float* input, const float* packed_filter,
            const float* bias, ActivationRange act, float* output, std::vector<float>* scratch) {
  const int k = g.kernel_h * g.kernel_w * g.in_c;
  const int rows = g.out_h * g.out_w;
  const bool pointwise = Conv2DIsPointwise(g);
  if (!pointwise && scratch->size() < static_cast<size_t>(rows) * static_cast<size_t>(k)) {
    scratch->resize(static_cast<size_t>(rows) * static_cast<size_t>(k));
  }
  for (int n = 0; n < g.batch; ++n) {
    const float* in_n = input + static_cast<size_t>(n) * g.in_h * g.in_w * g.in_c;
    float* out_n = output + static_cast<size_t>(n) * rows * g.out_c;
    const float* a_data = in_n;
    if (!pointwise) {
      float* col = scratch->data();
      for (int oy = 0; oy < g.out_h; ++oy) {
        for (int ox = 0; ox < g.out_w; ++ox) {
          float* row = col + (static_cast<size_t>(oy) * g.out_w + ox) * k;
          for (int ky = 0; ky < g.kernel_h; ++ky) {
            const int iy = oy * g.stride_h - g.pad_top + ky * g.dilation_h;
            for (int kx = 0; kx < g.kernel_w; ++kx) {
              const int ix = ox * g.stride_w - g.pad_left + kx * g.dilation_w;
              float* dst = row + (static_cast<size_t>(ky) * g.kernel_w + kx) * g.in_c;
              if (iy >= 0 && iy < g.in_h && ix >= 0 && ix < g.in_w) {
                std::memcpy(dst, in_n + (static_cast<size_t>(iy) * g.in_w + ix) * g.in_c,
                            sizeof(float) * g.in_c);
              } else {
                std::memset(dst, 0, sizeof(float) * g.in_c);
              }
            }
          }
        }
      }
      a_data = col;
    }
    gemm::Gemm(a_data, rows, k, static_cast<size_t>(k), packed_filter, g.out_c, out_n,
               static_cast<size_t>(g.out_c));
    for (int r = 0; r < rows; ++r) {
      float* o = out_n + static_cast<size_t>(r) * g.out_c;
      if (bias != nullptr) {
        for (int oc = 0; oc < g.out_c; ++oc) o[oc] = Clamp(o[oc] + bias[oc], act);
      } else {
        for (int oc = 0; oc < g.out_c; ++oc) o[oc] = Clamp(o[oc], act);
      }
    }
  }
}

namespace {

// Same vector type as gemm.cc (GCC / Clang vector extensions: SSE2 on x86-64, simd128 in WASM).
typedef float V4 __attribute__((vector_size(16)));
typedef float V4u __attribute__((vector_size(16), aligned(4), may_alias));

inline V4 LoadV4(const float* p) { return *reinterpret_cast<const V4u*>(p); }
inline void StoreV4(float* p, V4 v) { *reinterpret_cast<V4u*>(p) = v; }

// Taps of a window along one axis whose input coordinate base + k * dilation is inside [0, size):
// the inclusive range [*lo, *hi] of k (empty if *lo > *hi).
inline void ValidTaps(int base, int dilation, int taps, int size, int* lo, int* hi) {
  *lo = base >= 0 ? 0 : (-base + dilation - 1) / dilation;
  const int last = size - 1 - base;
  *hi = last < 0 ? -1 : std::min(taps - 1, last / dilation);
}

}  // namespace

void DepthwiseConv2D(const ConvGeometry& g, int depth_multiplier, const float* input,
                     const float* filter, const float* bias, ActivationRange act, float* output) {
  const int oc_n = g.out_c;
  for (int n = 0; n < g.batch; ++n) {
    const float* in_n = input + static_cast<size_t>(n) * g.in_h * g.in_w * g.in_c;
    for (int oy = 0; oy < g.out_h; ++oy) {
      int ky_lo;
      int ky_hi;
      ValidTaps(oy * g.stride_h - g.pad_top, g.dilation_h, g.kernel_h, g.in_h, &ky_lo, &ky_hi);
      for (int ox = 0; ox < g.out_w; ++ox) {
        float* o = output + ((static_cast<size_t>(n) * g.out_h + oy) * g.out_w + ox) * oc_n;
        int kx_lo;
        int kx_hi;
        ValidTaps(ox * g.stride_w - g.pad_left, g.dilation_w, g.kernel_w, g.in_w, &kx_lo, &kx_hi);
        // Out-of-image taps are skipped (not added as zero); the others are added in (ky, kx)
        // order to a zero start, per channel, in registers.
        if (depth_multiplier == 1) {
          int c0 = 0;
          for (; c0 + 16 <= oc_n; c0 += 16) {
            V4 a0 = {0, 0, 0, 0};
            V4 a1 = a0;
            V4 a2 = a0;
            V4 a3 = a0;
            for (int ky = ky_lo; ky <= ky_hi; ++ky) {
              const int iy = oy * g.stride_h - g.pad_top + ky * g.dilation_h;
              for (int kx = kx_lo; kx <= kx_hi; ++kx) {
                const int ix = ox * g.stride_w - g.pad_left + kx * g.dilation_w;
                const float* ip = in_n + (static_cast<size_t>(iy) * g.in_w + ix) * g.in_c + c0;
                const float* wp = filter + (static_cast<size_t>(ky) * g.kernel_w + kx) * oc_n + c0;
                a0 = a0 + LoadV4(ip) * LoadV4(wp);
                a1 = a1 + LoadV4(ip + 4) * LoadV4(wp + 4);
                a2 = a2 + LoadV4(ip + 8) * LoadV4(wp + 8);
                a3 = a3 + LoadV4(ip + 12) * LoadV4(wp + 12);
              }
            }
            StoreV4(o + c0, a0);
            StoreV4(o + c0 + 4, a1);
            StoreV4(o + c0 + 8, a2);
            StoreV4(o + c0 + 12, a3);
          }
          for (; c0 + 4 <= oc_n; c0 += 4) {
            V4 a0 = {0, 0, 0, 0};
            for (int ky = ky_lo; ky <= ky_hi; ++ky) {
              const int iy = oy * g.stride_h - g.pad_top + ky * g.dilation_h;
              for (int kx = kx_lo; kx <= kx_hi; ++kx) {
                const int ix = ox * g.stride_w - g.pad_left + kx * g.dilation_w;
                const float* ip = in_n + (static_cast<size_t>(iy) * g.in_w + ix) * g.in_c + c0;
                const float* wp = filter + (static_cast<size_t>(ky) * g.kernel_w + kx) * oc_n + c0;
                a0 = a0 + LoadV4(ip) * LoadV4(wp);
              }
            }
            StoreV4(o + c0, a0);
          }
          for (; c0 < oc_n; ++c0) {
            float a0 = 0.0f;
            for (int ky = ky_lo; ky <= ky_hi; ++ky) {
              const int iy = oy * g.stride_h - g.pad_top + ky * g.dilation_h;
              for (int kx = kx_lo; kx <= kx_hi; ++kx) {
                const int ix = ox * g.stride_w - g.pad_left + kx * g.dilation_w;
                a0 += in_n[(static_cast<size_t>(iy) * g.in_w + ix) * g.in_c + c0] *
                      filter[(static_cast<size_t>(ky) * g.kernel_w + kx) * oc_n + c0];
              }
            }
            o[c0] = a0;
          }
        } else {
          std::fill(o, o + oc_n, 0.0f);
          for (int ky = ky_lo; ky <= ky_hi; ++ky) {
            const int iy = oy * g.stride_h - g.pad_top + ky * g.dilation_h;
            for (int kx = kx_lo; kx <= kx_hi; ++kx) {
              const int ix = ox * g.stride_w - g.pad_left + kx * g.dilation_w;
              const float* ip = in_n + (static_cast<size_t>(iy) * g.in_w + ix) * g.in_c;
              const float* wp = filter + (static_cast<size_t>(ky) * g.kernel_w + kx) * oc_n;
              for (int c = 0; c < g.in_c; ++c) {
                for (int m = 0; m < depth_multiplier; ++m) {
                  o[c * depth_multiplier + m] += ip[c] * wp[c * depth_multiplier + m];
                }
              }
            }
          }
        }
        if (bias != nullptr) {
          for (int c = 0; c < oc_n; ++c) o[c] = Clamp(o[c] + bias[c], act);
        } else {
          for (int c = 0; c < oc_n; ++c) o[c] = Clamp(o[c], act);
        }
      }
    }
  }
}

void MaxPool2D(const ConvGeometry& g, ActivationRange act, const float* input, float* output) {
  // reference_ops pooling.h MaxPool (float): window clipped to the image.
  for (int n = 0; n < g.batch; ++n) {
    for (int oy = 0; oy < g.out_h; ++oy) {
      const int iy0 = oy * g.stride_h - g.pad_top;
      const int fy_start = std::max(0, -iy0);
      const int fy_end = std::min(g.kernel_h, g.in_h - iy0);
      for (int ox = 0; ox < g.out_w; ++ox) {
        const int ix0 = ox * g.stride_w - g.pad_left;
        const int fx_start = std::max(0, -ix0);
        const int fx_end = std::min(g.kernel_w, g.in_w - ix0);
        float* o = output + ((static_cast<size_t>(n) * g.out_h + oy) * g.out_w + ox) * g.out_c;
        std::fill(o, o + g.out_c, std::numeric_limits<float>::lowest());
        for (int fy = fy_start; fy < fy_end; ++fy) {
          for (int fx = fx_start; fx < fx_end; ++fx) {
            const float* ip =
                input + ((static_cast<size_t>(n) * g.in_h + iy0 + fy) * g.in_w + ix0 + fx) * g.in_c;
            for (int c = 0; c < g.out_c; ++c) o[c] = std::max(o[c], ip[c]);
          }
        }
        for (int c = 0; c < g.out_c; ++c) o[c] = Clamp(o[c], act);
      }
    }
  }
}

bool BroadcastShape(const Shape& a, const Shape& b, Shape* out) {
  const size_t rank = std::max(a.size(), b.size());
  if (rank > static_cast<size_t>(kMaxRank)) return false;
  out->assign(rank, 1);
  for (size_t i = 0; i < rank; ++i) {
    const int32_t da = i < rank - a.size() ? 1 : a[i - (rank - a.size())];
    const int32_t db = i < rank - b.size() ? 1 : b[i - (rank - b.size())];
    if (da != db && da != 1 && db != 1) return false;
    (*out)[i] = da == 1 ? db : da;
  }
  return true;
}

void BroadcastBinary(BinaryOp op, const float* a, const Shape& a_shape, const float* b,
                     const Shape& b_shape, float* out, const Shape& out_shape,
                     ActivationRange act) {
  switch (op) {
    case BinaryOp::kAdd:
      BroadcastRows(a, a_shape, b, b_shape, out, out_shape,
                    [act](const float* x, const float* y, float* o, size_t n, size_t xs,
                          size_t ys) {
                      RowApply(x, y, o, n, xs, ys,
                               [act](float u, float v) { return Clamp(u + v, act); });
                    });
      break;
    case BinaryOp::kSub:
      BroadcastRows(a, a_shape, b, b_shape, out, out_shape,
                    [act](const float* x, const float* y, float* o, size_t n, size_t xs,
                          size_t ys) {
                      RowApply(x, y, o, n, xs, ys,
                               [act](float u, float v) { return Clamp(u - v, act); });
                    });
      break;
    case BinaryOp::kMul:
      BroadcastRows(a, a_shape, b, b_shape, out, out_shape,
                    [act](const float* x, const float* y, float* o, size_t n, size_t xs,
                          size_t ys) {
                      RowApply(x, y, o, n, xs, ys,
                               [act](float u, float v) { return Clamp(u * v, act); });
                    });
      break;
    case BinaryOp::kMinimum:
      BroadcastRows(a, a_shape, b, b_shape, out, out_shape,
                    [](const float* x, const float* y, float* o, size_t n, size_t xs, size_t ys) {
                      RowApply(x, y, o, n, xs, ys, [](float u, float v) { return u < v ? u : v; });
                    });
      break;
    case BinaryOp::kPrelu:
      // reference_ops prelu.h: input >= 0 ? input : input * alpha.
      BroadcastRows(a, a_shape, b, b_shape, out, out_shape,
                    [](const float* x, const float* y, float* o, size_t n, size_t xs, size_t ys) {
                      RowApply(x, y, o, n, xs, ys,
                               [](float u, float v) { return u >= 0.0f ? u : u * v; });
                    });
      break;
  }
}

template <typename T>
bool BroadcastFloorModInt(const T* a, const Shape& a_shape, const T* b, const Shape& b_shape,
                          T* out, const Shape& out_shape) {
  bool zero_divisor = false;
  BroadcastRows(a, a_shape, b, b_shape, out, out_shape,
                [&zero_divisor](const T* x, const T* y, T* o, size_t n, size_t xs, size_t ys) {
                  RowApply(x, y, o, n, xs, ys, [&zero_divisor](T u, T v) -> T {
                    if (v == 0) {
                      zero_divisor = true;
                      return 0;
                    }
                    return FloorModInt<T>(u, v);
                  });
                });
  return !zero_divisor;
}

bool BroadcastFloorMod(const int64_t* a, const Shape& a_shape, const int64_t* b,
                       const Shape& b_shape, int64_t* out, const Shape& out_shape) {
  return BroadcastFloorModInt<int64_t>(a, a_shape, b, b_shape, out, out_shape);
}

bool BroadcastFloorMod(const int32_t* a, const Shape& a_shape, const int32_t* b,
                       const Shape& b_shape, int32_t* out, const Shape& out_shape) {
  return BroadcastFloorModInt<int32_t>(a, a_shape, b, b_shape, out, out_shape);
}

void BroadcastFloorMod(const float* a, const Shape& a_shape, const float* b, const Shape& b_shape,
                       float* out, const Shape& out_shape) {
  BroadcastRows(a, a_shape, b, b_shape, out, out_shape,
                [](const float* x, const float* y, float* o, size_t n, size_t xs, size_t ys) {
                  RowApply(x, y, o, n, xs, ys, FloorModFloat);
                });
}

void Relu(const float* in, size_t n, float* out) {
  for (size_t i = 0; i < n; ++i) out[i] = std::max(in[i], 0.0f);
}

void LeakyRelu(const float* in, size_t n, float alpha, float* out) {
  // reference_ops leaky_relu.h: val > 0 ? val : val * alpha.
  for (size_t i = 0; i < n; ++i) out[i] = in[i] > 0.0f ? in[i] : in[i] * alpha;
}

void HardSwish(const float* in, size_t n, float* out) {
  // reference_ops hard_swish.h: in * min(6, max(0, in + 3)) / 6.
  for (size_t i = 0; i < n; ++i) {
    const float v = in[i];
    out[i] = v * std::min(6.0f, std::max(0.0f, v + 3.0f)) / 6.0f;
  }
}

void Logistic(const float* in, size_t n, float* out) {
  // reference_ops logistic.h (float): 1 / (1 + exp(-x)).
  for (size_t i = 0; i < n; ++i) out[i] = 1.0f / (1.0f + std::exp(-in[i]));
}

void Pad(const float* in, const Shape& in_shape, const std::vector<int32_t>& paddings, float* out,
         const Shape& out_shape) {
  const int rank = static_cast<int>(in_shape.size());
  std::fill(out, out + NumElements(out_shape), 0.0f);
  if (rank == 0) {
    out[0] = in[0];
    return;
  }
  const std::vector<size_t> os = Strides(out_shape);
  const size_t inner = static_cast<size_t>(in_shape[rank - 1]);
  const size_t total = NumElements(in_shape);
  if (inner == 0 || total == 0) return;
  int idx[kMaxRank] = {0, 0, 0, 0, 0, 0};
  for (size_t row = 0; row < total / inner; ++row) {
    size_t o = static_cast<size_t>(paddings[2 * (rank - 1)]);
    for (int d = 0; d < rank - 1; ++d) {
      o += static_cast<size_t>(idx[d] + paddings[2 * d]) * os[d];
    }
    std::memcpy(out + o, in + row * inner, sizeof(float) * inner);
    for (int d = rank - 2; d >= 0; --d) {
      if (++idx[d] < in_shape[d]) break;
      idx[d] = 0;
    }
  }
}

void Concatenation(const std::vector<const float*>& inputs, const std::vector<Shape>& shapes,
                   int axis, float* out) {
  size_t outer = 1;
  for (int d = 0; d < axis; ++d) outer *= static_cast<size_t>(shapes[0][d]);
  std::vector<size_t> chunk(inputs.size());
  for (size_t k = 0; k < inputs.size(); ++k) {
    size_t c = 1;
    for (size_t d = static_cast<size_t>(axis); d < shapes[k].size(); ++d) {
      c *= static_cast<size_t>(shapes[k][d]);
    }
    chunk[k] = c;
  }
  float* o = out;
  for (size_t i = 0; i < outer; ++i) {
    for (size_t k = 0; k < inputs.size(); ++k) {
      // A zero-size input has no storage to copy from (its data pointer may be null).
      if (chunk[k] != 0) std::memcpy(o, inputs[k] + i * chunk[k], sizeof(float) * chunk[k]);
      o += chunk[k];
    }
  }
}

void Transpose(const float* in, const Shape& in_shape, const std::vector<int32_t>& perm,
               float* out) {
  const int rank = static_cast<int>(in_shape.size());
  const std::vector<size_t> is = Strides(in_shape);
  Shape out_shape(rank);
  size_t step[kMaxRank];
  for (int d = 0; d < rank; ++d) {
    out_shape[d] = in_shape[perm[d]];
    step[d] = is[perm[d]];
  }
  const size_t total = NumElements(in_shape);
  int idx[kMaxRank] = {0, 0, 0, 0, 0, 0};
  for (size_t i = 0; i < total; ++i) {
    size_t src = 0;
    for (int d = 0; d < rank; ++d) src += static_cast<size_t>(idx[d]) * step[d];
    out[i] = in[src];
    for (int d = rank - 1; d >= 0; --d) {
      if (++idx[d] < out_shape[d]) break;
      idx[d] = 0;
    }
  }
}

std::vector<SliceAxis> ResolveStridedSlice(const Shape& in_shape, const std::vector<int32_t>& begin,
                                           const std::vector<int32_t>& end,
                                           const std::vector<int32_t>& strides, int begin_mask,
                                           int end_mask, int shrink_axis_mask) {
  // strided_slice_logic.h StartForAxis / StopForAxis.
  std::vector<SliceAxis> axes;
  for (size_t a = 0; a < in_shape.size(); ++a) {
    const int size = in_shape[a];
    const int stride = strides[a];
    SliceAxis s{0, 0, stride};
    if (size == 0) {
      axes.push_back(s);
      continue;
    }
    int64_t start = begin[a];
    if (begin_mask & (1 << a)) {
      start = stride > 0 ? std::numeric_limits<int>::lowest() : std::numeric_limits<int>::max();
    }
    if (start < 0) start += size;
    start = stride > 0 ? std::min<int64_t>(std::max<int64_t>(start, 0), size)
                       : std::min<int64_t>(std::max<int64_t>(start, -1), size - 1);
    int64_t stop;
    if (shrink_axis_mask & (1 << a)) {
      stop = start + 1;
    } else {
      stop = end[a];
      if (end_mask & (1 << a)) {
        stop = stride > 0 ? std::numeric_limits<int>::max() : std::numeric_limits<int>::lowest();
      }
      if (stop < 0) stop += size;
      stop = stride > 0 ? std::min<int64_t>(std::max<int64_t>(stop, 0), size)
                        : std::min<int64_t>(std::max<int64_t>(stop, -1), size - 1);
    }
    s.start = static_cast<int>(start);
    s.stop = static_cast<int>(stop);
    axes.push_back(s);
  }
  return axes;
}

int SliceLength(const SliceAxis& a) {
  // 64-bit: a stride near INT_MAX / INT_MIN must not overflow the rounding term.
  const int64_t stride = a.stride;
  int64_t n = 0;
  if (stride > 0) {
    n = a.stop > a.start ? (static_cast<int64_t>(a.stop) - a.start + stride - 1) / stride : 0;
  } else if (stride < 0) {
    n = a.start > a.stop ? (static_cast<int64_t>(a.start) - a.stop - stride - 1) / (-stride) : 0;
  }
  return static_cast<int>(n);  // <= the axis size
}

void StridedSlice(const float* in, const Shape& in_shape, const std::vector<SliceAxis>& axes,
                  float* out) {
  const int rank = static_cast<int>(in_shape.size());
  const std::vector<size_t> is = Strides(in_shape);
  int len[kMaxRank];
  size_t total = 1;
  for (int d = 0; d < rank; ++d) {
    len[d] = SliceLength(axes[d]);
    total *= static_cast<size_t>(len[d]);
  }
  int idx[kMaxRank] = {0, 0, 0, 0, 0, 0};
  for (size_t i = 0; i < total; ++i) {
    size_t src = 0;
    for (int d = 0; d < rank; ++d) {
      src += static_cast<size_t>(axes[d].start + idx[d] * axes[d].stride) * is[d];
    }
    out[i] = in[src];
    for (int d = rank - 1; d >= 0; --d) {
      if (++idx[d] < len[d]) break;
      idx[d] = 0;
    }
  }
}

void ResizeBilinear(const float* in, const Shape& in_shape, int out_h, int out_w,
                    bool align_corners, bool half_pixel_centers, float* out) {
  // reference_ops.h ResizeBilinear (float).
  const int batches = in_shape[0];
  const int in_h = in_shape[1];
  const int in_w = in_shape[2];
  const int depth = in_shape[3];
  float height_scale = static_cast<float>(in_h) / out_h;
  float width_scale = static_cast<float>(in_w) / out_w;
  if (align_corners && out_h > 1) height_scale = static_cast<float>(in_h - 1) / (out_h - 1);
  if (align_corners && out_w > 1) width_scale = static_cast<float>(in_w - 1) / (out_w - 1);
  auto at = [&](int b, int y, int x) {
    return in + ((static_cast<size_t>(b) * in_h + y) * in_w + x) * depth;
  };
  for (int b = 0; b < batches; ++b) {
    for (int y = 0; y < out_h; ++y) {
      float input_y;
      int y0;
      int y1;
      ComputeInterpolationValues(static_cast<float>(y), height_scale, half_pixel_centers, in_h,
                                 &input_y, &y0, &y1);
      for (int x = 0; x < out_w; ++x) {
        float input_x;
        int x0;
        int x1;
        ComputeInterpolationValues(static_cast<float>(x), width_scale, half_pixel_centers, in_w,
                                   &input_x, &x0, &x1);
        const float dy = input_y - y0;
        const float dx = input_x - x0;
        const float* p00 = at(b, y0, x0);
        const float* p10 = at(b, y1, x0);
        const float* p01 = at(b, y0, x1);
        const float* p11 = at(b, y1, x1);
        float* o = out + ((static_cast<size_t>(b) * out_h + y) * out_w + x) * depth;
        for (int c = 0; c < depth; ++c) {
          o[c] = p00[c] * (1 - dy) * (1 - dx) + p10[c] * dy * (1 - dx) +
                 p01[c] * (1 - dy) * dx + p11[c] * dy * dx;
        }
      }
    }
  }
}

void ReduceMean(const float* in, const Shape& in_shape, const std::vector<int>& axes, float* out,
                size_t out_count) {
  std::fill(out, out + out_count, 0.0f);
  ReduceGeneric(in, in_shape, axes, out, [](float acc, float v) { return acc + v; });
  size_t reduced = 1;
  for (int a : axes) reduced *= static_cast<size_t>(in_shape[a]);
  const float count = static_cast<float>(reduced);
  for (size_t i = 0; i < out_count; ++i) out[i] = out[i] / count;
}

void ReduceMax(const float* in, const Shape& in_shape, const std::vector<int>& axes, float* out,
               size_t out_count) {
  std::fill(out, out + out_count, std::numeric_limits<float>::lowest());
  ReduceGeneric(in, in_shape, axes, out, [](float acc, float v) { return v > acc ? v : acc; });
}

namespace {

template <typename Index>
void ArgMaxImpl(const float* in, const Shape& in_shape, int axis, Index* out) {
  size_t outer = 1;
  size_t inner = 1;
  for (int d = 0; d < axis; ++d) outer *= static_cast<size_t>(in_shape[d]);
  for (size_t d = static_cast<size_t>(axis) + 1; d < in_shape.size(); ++d) {
    inner *= static_cast<size_t>(in_shape[d]);
  }
  const size_t n = static_cast<size_t>(in_shape[axis]);
  for (size_t o = 0; o < outer; ++o) {
    for (size_t i = 0; i < inner; ++i) {
      const float* p = in + o * n * inner + i;
      size_t best = 0;
      float best_v = p[0];
      for (size_t a = 1; a < n; ++a) {
        if (p[a * inner] > best_v) {
          best_v = p[a * inner];
          best = a;
        }
      }
      out[o * inner + i] = static_cast<Index>(best);
    }
  }
}

}  // namespace

void ArgMax(const float* in, const Shape& in_shape, int axis, int64_t* out) {
  ArgMaxImpl(in, in_shape, axis, out);
}

void ArgMax(const float* in, const Shape& in_shape, int axis, int32_t* out) {
  ArgMaxImpl(in, in_shape, axis, out);
}

int32_t SaturatingFloatToInt32(float v) {
  if (std::isnan(v)) return 0;
  if (v >= 2147483648.0f) return INT32_MAX;  // 2^31 is exactly representable
  if (v <= -2147483648.0f) return INT32_MIN;
  return static_cast<int32_t>(v);
}

int64_t SaturatingFloatToInt64(float v) {
  if (std::isnan(v)) return 0;
  if (v >= 9223372036854775808.0f) return INT64_MAX;  // 2^63 is exactly representable
  if (v <= -9223372036854775808.0f) return INT64_MIN;
  return static_cast<int64_t>(v);
}

float HalfToFloat(uint16_t h) {
  const uint32_t sign = static_cast<uint32_t>(h & 0x8000u) << 16;
  const uint32_t exponent = (h >> 10) & 0x1fu;
  uint32_t mantissa = h & 0x3ffu;
  uint32_t bits;
  if (exponent == 0) {
    if (mantissa == 0) {
      bits = sign;
    } else {
      int shift = 0;
      while ((mantissa & 0x400u) == 0) {
        mantissa <<= 1;
        ++shift;
      }
      mantissa &= 0x3ffu;
      bits = sign | (static_cast<uint32_t>(113 - shift) << 23) | (mantissa << 13);
    }
  } else if (exponent == 31) {
    bits = sign | 0x7f800000u | (mantissa << 13);
  } else {
    bits = sign | ((exponent + 112u) << 23) | (mantissa << 13);
  }
  float f;
  std::memcpy(&f, &bits, sizeof(f));
  return f;
}

}  // namespace kernels
}  // namespace ml
}  // namespace slam_refact

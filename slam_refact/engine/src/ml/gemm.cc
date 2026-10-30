#include "ml/gemm.h"

#include <cstdint>
#include <cstring>

// GCC / Clang vector extensions: one source for x86-64 (SSE2 baseline, no AVX) and WASM simd128.
// The multiply and the add are separate operations (the engine flags forbid FMA contraction), so
// the rounding is the same on every target. See gemm.h for the accumulation order that is promised.
namespace slam_refact {
namespace ml {
namespace gemm {
namespace {

typedef float V4 __attribute__((vector_size(16)));
// Unaligned access to float memory.
typedef float V4u __attribute__((vector_size(16), aligned(4), may_alias));

inline V4 Load(const float* p) { return *reinterpret_cast<const V4u*>(p); }
inline void Store(float* p, V4 v) { *reinterpret_cast<V4u*>(p) = v; }
inline V4 Splat(float x) { return V4{x, x, x, x}; }

// MR rows x kNr columns of c over the full K. The K axis is cut into blocks of kKBlock (the last one
// may be shorter); inside a block every output is accumulated from zero in k order, and the block
// sums are added to the running total in block order.
template <int MR>
inline void MicroKernel(const float* a, size_t lda, const float* b, int k, float* c, size_t ldc,
                        int n_valid) {
  V4 total_lo[MR];
  V4 total_hi[MR];
  for (int r = 0; r < MR; ++r) {
    total_lo[r] = Splat(0.0f);
    total_hi[r] = Splat(0.0f);
  }
  for (int k0 = 0; k0 < k; k0 += kKBlock) {
    const int k1 = k - k0 < kKBlock ? k : k0 + kKBlock;
    V4 lo[MR];
    V4 hi[MR];
    for (int r = 0; r < MR; ++r) {
      lo[r] = Splat(0.0f);
      hi[r] = Splat(0.0f);
    }
    const float* bk = b + static_cast<size_t>(k0) * kNr;
    for (int kk = k0; kk < k1; ++kk) {
      const V4 b0 = Load(bk);
      const V4 b1 = Load(bk + 4);
      bk += kNr;
      for (int r = 0; r < MR; ++r) {
        const V4 s = Splat(a[static_cast<size_t>(r) * lda + static_cast<size_t>(kk)]);
        lo[r] = lo[r] + s * b0;
        hi[r] = hi[r] + s * b1;
      }
    }
    for (int r = 0; r < MR; ++r) {
      total_lo[r] = total_lo[r] + lo[r];
      total_hi[r] = total_hi[r] + hi[r];
    }
  }
  for (int r = 0; r < MR; ++r) {
    float* row = c + static_cast<size_t>(r) * ldc;
    if (n_valid == kNr) {
      Store(row, total_lo[r]);
      Store(row + 4, total_hi[r]);
    } else {
      float tmp[kNr];
      Store(tmp, total_lo[r]);
      Store(tmp + 4, total_hi[r]);
      std::memcpy(row, tmp, sizeof(float) * static_cast<size_t>(n_valid));
    }
  }
}

}  // namespace

size_t PackedSize(int n, int k) {
  const size_t panels = (static_cast<size_t>(n) + kNr - 1) / kNr;
  return panels * kNr * static_cast<size_t>(k);
}

void PackWeights(const float* w, int n, int k, size_t w_stride, float* packed) {
  const int panels = (n + kNr - 1) / kNr;
  for (int p = 0; p < panels; ++p) {
    float* dst = packed + static_cast<size_t>(p) * kNr * static_cast<size_t>(k);
    for (int kk = 0; kk < k; ++kk) {
      for (int j = 0; j < kNr; ++j) {
        const int col = p * kNr + j;
        dst[static_cast<size_t>(kk) * kNr + j] =
            col < n ? w[static_cast<size_t>(col) * w_stride + static_cast<size_t>(kk)] : 0.0f;
      }
    }
  }
}

void Gemm(const float* a, int m, int k, size_t lda, const float* packed, int n, float* c,
          size_t ldc) {
  const int panels = (n + kNr - 1) / kNr;
  for (int m0 = 0; m0 < m; m0 += kMc) {
    const int mc = m - m0 < kMc ? m - m0 : kMc;
    for (int p = 0; p < panels; ++p) {
      const float* b = packed + static_cast<size_t>(p) * kNr * static_cast<size_t>(k);
      const int n_valid = n - p * kNr < kNr ? n - p * kNr : kNr;
      float* c_panel = c + static_cast<size_t>(p) * kNr;
      for (int i = 0; i < mc; i += kMr) {
        const float* a_tile = a + static_cast<size_t>(m0 + i) * lda;
        float* c_tile = c_panel + static_cast<size_t>(m0 + i) * ldc;
        switch (mc - i < kMr ? mc - i : kMr) {
          case 4:
            MicroKernel<4>(a_tile, lda, b, k, c_tile, ldc, n_valid);
            break;
          case 3:
            MicroKernel<3>(a_tile, lda, b, k, c_tile, ldc, n_valid);
            break;
          case 2:
            MicroKernel<2>(a_tile, lda, b, k, c_tile, ldc, n_valid);
            break;
          default:
            MicroKernel<1>(a_tile, lda, b, k, c_tile, ldc, n_valid);
            break;
        }
      }
    }
  }
}

}  // namespace gemm
}  // namespace ml
}  // namespace slam_refact

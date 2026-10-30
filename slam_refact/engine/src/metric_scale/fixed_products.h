// Float evaluation order of the original's fixed-size Eigen products in the MetricScaleEstimator
// lane (WP5b MSE precision). Module-private header.
//
// The original builds Eigen without its own SIMD (scalar packets) and lets the compiler vectorize,
// so the order of every sum differs from a host Eigen with SSE packets. Two shapes occur:
//   * Gebp: the GEMM micro kernel func 1236 (Eigen gebp, one lhs row times four rhs columns) used
//     by the 9x9 / 9x3 / 3x9 GEMMs 1173 (C += a A B) and 1174 (C += a A M^T) of 1169 and 944.
//   * TreeSum: Eigen's unrolled redux (halving split) of a fixed-size lazy product, e.g. the
//     innovation H x (1169) or S = (H P) H^T (func 1205).
// Every rounding below is pinned by tests/metric_scale/metric_scale_test.cc (MsePrecision.*).

#ifndef SLAM_REFACT_METRIC_SCALE_FIXED_PRODUCTS_H_
#define SLAM_REFACT_METRIC_SCALE_FIXED_PRODUCTS_H_

namespace slam_refact::metric_scale::fixed_products {

// func 1236 for one block (rows x cols, depth <= kc). res(i, j) = acc * alpha + res(i, j) with
//   * columns j < cols & ~3 (the four-column SIMD path): odd = sum of the products of odd k and
//     even = sum of even k over k < depth & ~7, each in increasing k from +0; acc = odd + even;
//     then the remaining k are added one by one (acc = 0 when depth < 8);
//   * the last cols % 4 columns (scalar path): acc = 0 + p0 + p1 + ... in increasing k.
// p_k = lhs(i, k) * rhs(k, j). res is column-major with leading dimension res_ld.
template <typename Lhs, typename Rhs>
void Gebp(int rows, int cols, int depth, const Lhs& lhs, const Rhs& rhs, float alpha, float* res,
          int res_ld) {
  const int peeled = depth & ~7;
  const int cols4 = cols & ~3;
  for (int i = 0; i < rows; ++i) {
    for (int j = 0; j < cols; ++j) {
      float acc = 0.0f;
      int k = 0;
      if (j < cols4) {
        if (peeled > 0) {
          float odd = 0.0f;
          float even = 0.0f;
          for (; k < peeled; k += 2) {
            even = even + lhs(i, k) * rhs(k, j);
            odd = odd + lhs(i, k + 1) * rhs(k + 1, j);
          }
          acc = odd + even;
        }
      }
      for (; k < depth; ++k) {
        acc = acc + lhs(i, k) * rhs(k, j);
      }
      float& r = res[i + j * res_ld];
      r = acc * alpha + r;
    }
  }
}

// Eigen redux_novec_unroller: sum of f(0..n-1) split at n / 2, i.e. TreeSum(3) = f0 + (f1 + f2),
// TreeSum(9) = ((f0 + f1) + (f2 + f3)) + ((f4 + f5) + (f6 + (f7 + f8))).
template <typename F>
float TreeSum(int start, int length, const F& f) {
  if (length == 1) {
    return f(start);
  }
  const int half = length / 2;
  return TreeSum(start, half, f) + TreeSum(start + half, length - half, f);
}

}  // namespace slam_refact::metric_scale::fixed_products

#endif  // SLAM_REFACT_METRIC_SCALE_FIXED_PRODUCTS_H_

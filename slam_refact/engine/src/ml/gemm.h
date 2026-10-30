// Deterministic matrix products for the ML engine (RM-5; docs/design/ml-runtime.md section 8.8).
//
// Why not Eigen: Eigen's GEMM picks its K / M / N block sizes from the cache sizes cpuid reports at
// the first call, so on a hybrid CPU (P and E cores) the summation order of a long dot product
// depends on the core the process started on, and the same model gives different float bits.
// `Eigen::setCpuCacheSizes` would fix that but is process-global (Ceres / VIO bit-identity evidence).
// The kernels here take no tunable parameter from the machine: with B = kKBlock every output element
// is
//
//     c[i][j] = (((0 + S_0) + S_1) + ... + S_last),
//     S_b     = ((0 + a[i][bB] * w[j][bB]) + a[i][bB+1] * w[j][bB+1]) + ... (up to min(K, bB+B)-1)
//
// in exactly this order (k ascending inside a block, blocks in order, separate multiply and add, no
// FMA), whatever the tile shape, the matrix size or the CPU. Not fusing the multiply and the add is
// a property of the build (the engine flags give -ffp-contract=off to every target,
// cmake/SlamRefactFlags.cmake); MlGemm.MultiplyAndAddAreRoundedSeparately fails if a build fuses
// them. The block sums keep the rounding error of a long dot product where Eigen's blocked kernel
// had it: a single running sum over all of K measured 1.84x the RMS error of this order (geometric
// mean over the 32 non-trivial outputs of the seven models, against the float64 reference).
// Native x86-64 (SSE2 only) and WASM (simd128) give bit-identical results for the same input, and
// the P-core / E-core difference disappears. The per-element order is also why the tests compare
// every tile path against a plain scalar loop bit by bit.
//
// Layout: A is row-major [m][k] with row stride lda; the weight matrix W is [n][k] (one row per
// output channel, the TFLite CONV_2D / FULLY_CONNECTED filter layout) and is packed once into
// panels of kNr columns, [panel][k][kNr], zero padded, so that the micro kernel reads kNr
// consecutive weights per k.
#ifndef SLAM_REFACT_ML_GEMM_H_
#define SLAM_REFACT_ML_GEMM_H_

#include <cstddef>

namespace slam_refact {
namespace ml {
namespace gemm {

constexpr int kNr = 8;  // output channels per packed panel (two 4-float vectors)
constexpr int kMr = 4;  // rows per micro tile
constexpr int kMc = 64;  // rows per cache block (any value gives the same bits)
constexpr int kKBlock = 32;  // K elements summed in one run before the sum joins the total

// Floats needed to pack an [n][k] matrix.
size_t PackedSize(int n, int k);
// Packs W[n][k] (row stride w_stride, in floats) into `packed` (PackedSize floats).
void PackWeights(const float* w, int n, int k, size_t w_stride, float* packed);
// c[m][n] = A[m][k] . W^T, c row stride ldc. `packed` comes from PackWeights(W, n, k, ...).
// c is overwritten (no accumulation into existing values).
void Gemm(const float* a, int m, int k, size_t lda, const float* packed, int n, float* c,
          size_t ldc);

}  // namespace gemm
}  // namespace ml
}  // namespace slam_refact

#endif  // SLAM_REFACT_ML_GEMM_H_

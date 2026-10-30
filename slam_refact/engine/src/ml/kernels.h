// float32 kernels of the builtin TFLite ops used by the seven models (US-017).
//
// Semantics follow the TFLite reference kernels (tensorflow-fork 2.6
// tensorflow/lite/kernels/internal/reference/); the summation order differs (own GEMM with a fixed
// order, gemm.h), so results are compared with tolerances (docs/design/ml-runtime.md §5).
// Layout is NHWC, row-major. Callers (Interpreter) validate shapes; kernels assume consistent
// arguments.
#ifndef SLAM_REFACT_ML_KERNELS_H_
#define SLAM_REFACT_ML_KERNELS_H_

#include <cstddef>
#include <cstdint>
#include <vector>

#include "ml/tflite_model.h"

namespace slam_refact {
namespace ml {
namespace kernels {

using Shape = std::vector<int32_t>;

size_t NumElements(const Shape& shape);

// Resource limits enforced by Interpreter::Create (RM-1, review M4). Every size that reaches a
// kernel has passed through them, so no element count, index or buffer size can overflow 32-bit
// arithmetic (WASM) or request an absurd allocation. The seven models stay far below them (largest
// tensor 1.2 M elements, largest arena 8.4 MB, strides <= 3).
constexpr size_t kMaxTensorElements = size_t{1} << 26;  // 256 MiB of float32
// Everything one Interpreter allocates together: decoded constants (also folded and shared-buffer
// copies), packed weights, scratch and activations. 128 MiB is a quarter of the WASM maximum
// memory (512 MB) and leaves the other three quarters for the model file, the host and the rest
// of the engine. The seven models need at most 26.8 MiB, hand_landmark
// (docs/design/ml-runtime.md section 8.9).
constexpr size_t kMaxTotalBytes = size_t{128} << 20;
constexpr int64_t kMaxGeometryParam = int64_t{1} << 28;  // stride, dilation, window, extent, size

// Element count of `shape`. False for a negative dimension, an overflow, or a count above
// kMaxTensorElements; `*count` is set only on success.
bool CheckedNumElements(const Shape& shape, size_t* count);
// a * b; false if the product exceeds `limit`.
bool CheckedMul(size_t a, size_t b, size_t limit, size_t* out);

struct ActivationRange {
  float min;
  float max;
};
// NONE, RELU, RELU_N1_TO_1, RELU6. Returns false for TANH / SIGN_BIT (not fused-clamp ops).
bool RangeFor(Activation activation, ActivationRange* out);

// Output size and leading padding (TFLite kernels/padding.h).
int ComputeOutSize(Padding padding, int in_size, int filter, int stride, int dilation);
int ComputePadBefore(Padding padding, int in_size, int out_size, int filter, int stride,
                     int dilation);

struct ConvGeometry {
  int batch = 0;
  int in_h = 0;
  int in_w = 0;
  int in_c = 0;
  int out_h = 0;
  int out_w = 0;
  int out_c = 0;
  int kernel_h = 0;
  int kernel_w = 0;
  int stride_h = 1;
  int stride_w = 1;
  int dilation_h = 1;
  int dilation_w = 1;
  int pad_top = 0;
  int pad_left = 0;
};

// True if the convolution is a plain matrix product of the input (1x1 kernel, stride 1, no padding,
// output size = input size): Conv2D then needs no im2col buffer.
bool Conv2DIsPointwise(const ConvGeometry& g);
// packed_filter: gemm::PackWeights of the filter [out_c][kernel_h*kernel_w*in_c]; bias: [out_c] or
// nullptr. `scratch` holds the im2col rows (the interpreter sizes it at Create). Each output is
// accumulated over (ky, kx, ic) in the blocked order of gemm.h, independent of cache sizes and
// target.
void Conv2D(const ConvGeometry& g, const float* input, const float* packed_filter,
            const float* bias, ActivationRange act, float* output, std::vector<float>* scratch);
// filter: [1][kernel_h][kernel_w][out_c], out_c = in_c * depth_multiplier.
void DepthwiseConv2D(const ConvGeometry& g, int depth_multiplier, const float* input,
                     const float* filter, const float* bias, ActivationRange act, float* output);
// out_c == in_c; kernel = pool window.
void MaxPool2D(const ConvGeometry& g, ActivationRange act, const float* input, float* output);

enum class BinaryOp { kAdd, kSub, kMul, kMinimum, kPrelu };
// numpy broadcasting (TFLite broadcast semantics), rank <= 6. Fused activation applies to
// ADD / SUB / MUL (pass an unbounded range otherwise).
void BroadcastBinary(BinaryOp op, const float* a, const Shape& a_shape, const float* b,
                     const Shape& b_shape, float* out, const Shape& out_shape,
                     ActivationRange act);
// Floor modulo (result has the sign of the divisor), integer types. Returns false if a divisor is
// 0 (TFLite reports "Division by 0"; the outputs are then unspecified). A divisor of -1 gives 0
// without evaluating INT_MIN % -1.
bool BroadcastFloorMod(const int64_t* a, const Shape& a_shape, const int64_t* b,
                       const Shape& b_shape, int64_t* out, const Shape& out_shape);
bool BroadcastFloorMod(const int32_t* a, const Shape& a_shape, const int32_t* b,
                       const Shape& b_shape, int32_t* out, const Shape& out_shape);
void BroadcastFloorMod(const float* a, const Shape& a_shape, const float* b, const Shape& b_shape,
                       float* out, const Shape& out_shape);
// Output shape of a numpy broadcast; false if incompatible or rank > 6.
bool BroadcastShape(const Shape& a, const Shape& b, Shape* out);

void Relu(const float* in, size_t n, float* out);
void LeakyRelu(const float* in, size_t n, float alpha, float* out);
void HardSwish(const float* in, size_t n, float* out);
void Logistic(const float* in, size_t n, float* out);

// paddings: rank x 2 (before, after), zero fill.
void Pad(const float* in, const Shape& in_shape, const std::vector<int32_t>& paddings, float* out,
         const Shape& out_shape);
void Concatenation(const std::vector<const float*>& inputs, const std::vector<Shape>& shapes,
                   int axis, float* out);
void Transpose(const float* in, const Shape& in_shape, const std::vector<int32_t>& perm,
               float* out);

struct SliceAxis {
  int start;
  int stop;
  int stride;
};
// TFLite strided_slice_logic.h StartForAxis / StopForAxis for each axis (no ellipsis / new axis).
std::vector<SliceAxis> ResolveStridedSlice(const Shape& in_shape, const std::vector<int32_t>& begin,
                                           const std::vector<int32_t>& end,
                                           const std::vector<int32_t>& strides, int begin_mask,
                                           int end_mask, int shrink_axis_mask);
// Number of elements taken along an axis.
int SliceLength(const SliceAxis& a);
void StridedSlice(const float* in, const Shape& in_shape, const std::vector<SliceAxis>& axes,
                  float* out);

void ResizeBilinear(const float* in, const Shape& in_shape, int out_h, int out_w,
                    bool align_corners, bool half_pixel_centers, float* out);

// axes: normalized (0..rank-1), unique. Output shape without kept dims is the caller's concern.
void ReduceMean(const float* in, const Shape& in_shape, const std::vector<int>& axes, float* out,
                size_t out_count);
void ReduceMax(const float* in, const Shape& in_shape, const std::vector<int>& axes, float* out,
               size_t out_count);
// Index of the first maximum along `axis`.
void ArgMax(const float* in, const Shape& in_shape, int axis, int64_t* out);
void ArgMax(const float* in, const Shape& in_shape, int axis, int32_t* out);

// float -> integer conversions of CAST: truncate toward zero, saturate at the type's range, NaN
// gives 0 (the C++ conversion is undefined for those inputs).
int32_t SaturatingFloatToInt32(float v);
int64_t SaturatingFloatToInt64(float v);

// IEEE binary16 -> binary32 (exact).
float HalfToFloat(uint16_t h);

}  // namespace kernels
}  // namespace ml
}  // namespace slam_refact

#endif  // SLAM_REFACT_ML_KERNELS_H_

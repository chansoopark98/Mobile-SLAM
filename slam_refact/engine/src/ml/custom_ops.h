// Custom TFLite ops of the face mesh and palm models (US-017).
//
// Ports of (Apache License 2.0; Copyright 2021 The MediaPipe Authors, modifications by Niantic
// Labs / reference runtime):
//   reality/engine/deepnets/operations/transform-landmarks.cc        (TransformLandmarks v2)
//   reality/engine/deepnets/operations/landmarks-to-transform-matrix.cc
//                                                                    (Landmarks2TransformMatrix v2)
//   reality/engine/deepnets/operations/transform-tensor-bilinear.cc  (TransformTensorBilinear v2)
//   mediapipe/util/tflite/operations/transpose_conv_bias.cc          (Convolution2DTransposeBias)
// (local copies under assets/references/xr_reference/official/). The option parsers of the first three
// live in upstream TF delegates/gpu/common/mediapipe/, which is not in the local tree; their
// FlexBuffers keys are read from the models (docs/design/ml-runtime.md §3).
#ifndef SLAM_REFACT_ML_CUSTOM_OPS_H_
#define SLAM_REFACT_ML_CUSTOM_OPS_H_

#include <cstdint>
#include <string>
#include <utility>
#include <vector>

namespace slam_refact {
namespace ml {
namespace custom {

struct LandmarksToTransformMatrixParams {
  int left_rotation_idx = 0;
  int right_rotation_idx = 0;
  float target_rotation_radians = 0.0f;
  int output_height = 0;
  int output_width = 0;
  float scale_x = 0.0f;
  float scale_y = 0.0f;
  std::vector<std::pair<int, int>> subset_idxs;
};

// Reads the FlexBuffers options and applies the original Eval checks (non-negative indices,
// positive sizes and scales). `num_landmarks` bounds the indices.
bool ParseLandmarksToTransformMatrix(const std::vector<uint8_t>& options, int num_landmarks,
                                     LandmarksToTransformMatrixParams* out, std::string* error);
// landmarks: [num_landmarks][3]; out: 4x4 row-major.
void LandmarksToTransformMatrix(const LandmarksToTransformMatrixParams& p, const float* landmarks,
                                float* out);

// landmarks: [n][dims] (dims 2 or 3); matrix: 4x4 row-major (rows 0 and 1 are used).
void TransformLandmarks(const float* landmarks, int n, int dims, const float* matrix, float* out);

// TransformTensorBilinear v2 options: output_height / output_width (FlexBuffers).
bool ParseTransformTensorBilinear(const std::vector<uint8_t>& options, int* out_h, int* out_w,
                                  std::string* error);
// input: [1][in_h][in_w][c]; out: [1][out_h][out_w][c].
void TransformTensorBilinear(const float* input, int in_h, int in_w, int channels,
                             const float* matrix, int out_h, int out_w, float* out);

// Convolution2DTransposeBias options: raw TfLiteTransposeConvParams {padding, stride_w,
// stride_h} (int32 x 3; padding 1 = SAME, 2 = VALID).
bool ParseTransposeConvBias(const std::vector<uint8_t>& options, bool* padding_same,
                            int* stride_w, int* stride_h, std::string* error);
// Output size of the op (Prepare in transpose_conv_bias.cc). False if an input size or stride is
// below 1 or an output size does not fit 1..INT_MAX.
bool TransposeConvBiasOutputSize(int in_h, int in_w, int filter_h, int filter_w, int stride_h,
                                 int stride_w, bool padding_same, int* out_h, int* out_w,
                                 int* pad_h, int* pad_w);
// input [batch][in_h][in_w][in_c], filter [out_c][fh][fw][in_c] (OHWI), bias [out_c]. `scratch`
// (grown on demand) holds the GEMM result of one tap and the packed filter of one tap: at least
// in_h*in_w*out_c + gemm::PackedSize(out_c, in_c) floats. Every output element is the bias plus the
// taps in (fy, fx) order, each tap summed over in_c in the blocked order of gemm.h.
void TransposeConvBias(const float* input, int batch, int in_h, int in_w, int in_c,
                       const float* filter, int out_c, int filter_h, int filter_w,
                       const float* bias, int stride_h, int stride_w, int pad_top, int pad_left,
                       int out_h, int out_w, float* out, std::vector<float>* scratch);

}  // namespace custom
}  // namespace ml
}  // namespace slam_refact

#endif  // SLAM_REFACT_ML_CUSTOM_OPS_H_

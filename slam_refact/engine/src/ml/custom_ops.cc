// Ports of MediaPipe-derived custom ops (Apache License 2.0; see custom_ops.h for the sources).
// Float arithmetic follows the original expressions, including their double-precision literals.
#include "ml/custom_ops.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>

#include "ml/flexbuffers.h"
#include "ml/gemm.h"

namespace slam_refact {
namespace ml {
namespace custom {
namespace {

struct Float3 {
  float x;
  float y;
  float z;
};

// landmarks-to-transform-matrix.cc Mat3 (row-major, float accumulation in k order).
struct Mat3 {
  float d[9];
  float Get(int r, int c) const { return d[r * 3 + c]; }
  Float3 operator*(const Float3& v) const {
    const float in[3] = {v.x, v.y, v.z};
    float res[3];
    for (int r = 0; r < 3; ++r) {
      float sum = 0;
      for (int k = 0; k < 3; ++k) sum += Get(r, k) * in[k];
      res[r] = sum;
    }
    return {res[0], res[1], res[2]};
  }
};

struct Mat4 {
  float d[16];
  float Get(int r, int c) const { return d[r * 4 + c]; }
  void MulAssign(const Mat4& other) {
    float res[16];
    for (int r = 0; r < 4; ++r) {
      for (int c = 0; c < 4; ++c) {
        float sum = 0;
        for (int k = 0; k < 4; ++k) sum += Get(r, k) * other.Get(k, c);
        res[r * 4 + c] = sum;
      }
    }
    std::memcpy(d, res, sizeof(d));
  }
};

Mat4 MakeMat4(float x00, float x01, float x02, float x03, float x10, float x11, float x12,
              float x13, float x20, float x21, float x22, float x23, float x30, float x31,
              float x32, float x33) {
  return Mat4{{x00, x01, x02, x03, x10, x11, x12, x13, x20, x21, x22, x23, x30, x31, x32, x33}};
}

Float3 ReadXyz(const float* data, int idx) {
  return {data[idx * 3], data[idx * 3 + 1], data[idx * 3 + 2]};
}

float Dot4(const float* l, const float* r) { return l[0] * r[0] + l[1] * r[1] + l[2] * r[2] + l[3] * r[3]; }

}  // namespace

bool ParseLandmarksToTransformMatrix(const std::vector<uint8_t>& options, int num_landmarks,
                                     LandmarksToTransformMatrixParams* out, std::string* error) {
  FlexMap map;
  if (!ParseFlexMap(options.data(), options.size(), &map, error)) return false;
  int64_t left = 0;
  int64_t right = 0;
  int64_t oh = 0;
  int64_t ow = 0;
  double sx = 0.0;
  double sy = 0.0;
  double target = 0.0;
  std::vector<int64_t> subset;
  if (!FlexGetInt(map, "left_rotation_idx", &left) ||
      !FlexGetInt(map, "right_rotation_idx", &right) ||
      !FlexGetInt(map, "output_height", &oh) || !FlexGetInt(map, "output_width", &ow) ||
      !FlexGetFloat(map, "scale_x", &sx) || !FlexGetFloat(map, "scale_y", &sy) ||
      !FlexGetFloat(map, "target_rotation_radians", &target) ||
      !FlexGetIntVector(map, "subset_idxs", &subset)) {
    *error = "Landmarks2TransformMatrix: missing or mistyped option";
    return false;
  }
  // Eval checks of landmarks-to-transform-matrix.cc, plus index bounds.
  if (left < 0 || right < 0 || left >= num_landmarks || right >= num_landmarks || oh <= 0 ||
      ow <= 0 || oh > INT32_MAX || ow > INT32_MAX || !(sx > 0) || !(sy > 0) || subset.empty()) {
    *error = "Landmarks2TransformMatrix: invalid option values";
    return false;
  }
  for (int64_t v : subset) {
    if (v < 0 || v >= num_landmarks) {
      *error = "Landmarks2TransformMatrix: subset index out of range";
      return false;
    }
  }
  out->left_rotation_idx = static_cast<int>(left);
  out->right_rotation_idx = static_cast<int>(right);
  out->output_height = static_cast<int>(oh);
  out->output_width = static_cast<int>(ow);
  out->scale_x = static_cast<float>(sx);
  out->scale_y = static_cast<float>(sy);
  out->target_rotation_radians = static_cast<float>(target);
  // Flat index list -> pairs; an odd trailing index becomes (last, last). The upstream parser is
  // not in the local tree; the rule was first reconstructed from memory and is now confirmed by
  // two independent observations of the original runtime (docs/spec/08-face-ear.md §4, OR-2):
  // the xr-face.wasm option parser (function 685) reads index amount-1 twice, and on 32 recorded
  // original mesh detections rule (last, last) is closer to the original output than dropping the
  // last index (A_CONFIRMED; synthetic input).
  out->subset_idxs.clear();
  const size_t pairs = subset.size() / 2;
  for (size_t i = 0; i < pairs; ++i) {
    out->subset_idxs.emplace_back(static_cast<int>(subset[2 * i]),
                                  static_cast<int>(subset[2 * i + 1]));
  }
  if (subset.size() % 2) {
    const int last = static_cast<int>(subset.back());
    out->subset_idxs.emplace_back(last, last);
  }
  return true;
}

void LandmarksToTransformMatrix(const LandmarksToTransformMatrixParams& p, const float* landmarks,
                                float* out) {
  // EstimateRotationRadians.
  const Float3 left = ReadXyz(landmarks, p.left_rotation_idx);
  const Float3 right = ReadXyz(landmarks, p.right_rotation_idx);
  float rotation = std::atan2(right.y - left.y, right.x - left.x);
  rotation = p.target_rotation_radians - rotation;

  // EstimateCenterAndSize.
  std::vector<Float3> pts;
  pts.reserve(p.subset_idxs.size() * 2);
  for (const auto& pair : p.subset_idxs) {
    pts.push_back(ReadXyz(landmarks, pair.first));
    pts.push_back(ReadXyz(landmarks, pair.second));
  }
  for (Float3& v : pts) v.z = 1.0;
  const float r = rotation;
  const Mat3 t_rotation{{std::cos(r), -std::sin(r), 0.0f, std::sin(r), std::cos(r), 0.0f, 0.0f,
                         0.0f, 1.0f}};
  const Mat3 t_rotation_inverse{{std::cos(-r), -std::sin(-r), 0.0f, std::sin(-r), std::cos(-r),
                                 0.0f, 0.0f, 0.0f, 1.0f}};
  for (Float3& v : pts) v = t_rotation * v;
  Float3 max = pts[0];
  Float3 min = pts[0];
  for (size_t i = 1; i < pts.size(); ++i) {
    if (max.x < pts[i].x) max.x = pts[i].x;
    if (max.y < pts[i].y) max.y = pts[i].y;
    if (min.x > pts[i].x) min.x = pts[i].x;
    if (min.y > pts[i].y) min.y = pts[i].y;
  }
  const float crop_width = max.x - min.x;
  const float crop_height = max.y - min.y;
  Float3 crop = min;
  crop.x += max.x;
  crop.y += max.y;
  crop.x /= 2;
  crop.y /= 2;
  crop = t_rotation_inverse * crop;
  const float crop_x = crop.x;
  const float crop_y = crop.y;

  // LandmarksToTransformMatrixV2.
  Mat4 t = MakeMat4(1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1);
  t.MulAssign(MakeMat4(1, 0, 0, crop_x, 0, 1, 0, crop_y, 0, 0, 1, 0, 0, 0, 0, 1));
  const float rr = -rotation;
  t.MulAssign(MakeMat4(std::cos(rr), -std::sin(rr), 0, 0, std::sin(rr), std::cos(rr), 0, 0, 0, 0,
                       1, 0, 0, 0, 0, 1));
  const float scale_x = p.scale_x * crop_width / p.output_width;
  const float scale_y = p.scale_y * crop_height / p.output_height;
  t.MulAssign(MakeMat4(scale_x, 0, 0, 0, 0, scale_y, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1));
  const float shift_x = static_cast<float>(-1.0 * (p.output_width / 2.0));
  const float shift_y = static_cast<float>(-1.0 * (p.output_height / 2.0));
  t.MulAssign(MakeMat4(1, 0, 0, shift_x, 0, 1, 0, shift_y, 0, 0, 1, 0, 0, 0, 0, 1));
  std::memcpy(out, t.d, sizeof(t.d));
}

void TransformLandmarks(const float* landmarks, int n, int dims, const float* matrix, float* out) {
  // transform-landmarks.cc TransformLandmarksV2.
  const float x_t[4] = {matrix[0], matrix[1], matrix[2], matrix[3]};
  const float y_t[4] = {matrix[4], matrix[5], matrix[6], matrix[7]};
  for (int i = 0; i < n; ++i) {
    const int offset = i * dims;
    const float lv[4] = {landmarks[offset], landmarks[offset + 1], 0.0f, 1.0f};
    out[offset] = Dot4(x_t, lv);
    out[offset + 1] = Dot4(y_t, lv);
    if (dims == 3) out[offset + 2] = landmarks[offset + 2];
  }
}

bool ParseTransformTensorBilinear(const std::vector<uint8_t>& options, int* out_h, int* out_w,
                                  std::string* error) {
  FlexMap map;
  if (!ParseFlexMap(options.data(), options.size(), &map, error)) return false;
  int64_t h = 0;
  int64_t w = 0;
  if (!FlexGetInt(map, "output_height", &h) || !FlexGetInt(map, "output_width", &w) || h <= 0 ||
      w <= 0 || h > INT32_MAX || w > INT32_MAX) {
    *error = "TransformTensorBilinear: missing or invalid output size";
    return false;
  }
  *out_h = static_cast<int>(h);
  *out_w = static_cast<int>(w);
  return true;
}

void TransformTensorBilinear(const float* input, int in_h, int in_w, int channels,
                             const float* matrix, int out_h, int out_w, float* out) {
  // transform-tensor-bilinear.cc TransformTensorBilinearV2.
  float x_t[4] = {matrix[0], matrix[1], matrix[2], matrix[3]};
  float y_t[4] = {matrix[4], matrix[5], matrix[6], matrix[7]};
  // Align corners correction (double literals as in the original).
  x_t[3] += x_t[0] * 0.5 + x_t[1] * 0.5 - 0.5;
  y_t[3] += y_t[0] * 0.5 + y_t[1] * 0.5 - 0.5;
  auto read = [&](int h, int w, int c) -> float {
    return h < 0 || w < 0 || h >= in_h || w >= in_w
               ? 0
               : input[(static_cast<size_t>(h) * in_w + w) * channels + c];
  };
  for (int oy = 0; oy < out_h; ++oy) {
    for (int ox = 0; ox < out_w; ++ox) {
      const float coord[4] = {static_cast<float>(ox), static_cast<float>(oy), 0.0f, 1.0f};
      const float tx = Dot4(x_t, coord);
      const float ty = Dot4(y_t, coord);
      // Same test as the original, written so that a NaN coordinate counts as out of bound
      // (the original then converts NaN to int, which is undefined).
      const bool out_of_bound = !(tx >= 0.0 && tx <= in_w - 1 && ty >= 0.0 && ty <= in_h - 1);
      float* o = out + (static_cast<size_t>(oy) * out_w + ox) * channels;
      if (out_of_bound) {
        std::fill(o, o + channels, 0.0f);
        continue;
      }
      const float fx = std::floor(tx);
      const float fy = std::floor(ty);
      const int ix = static_cast<int>(fx);
      const int iy = static_cast<int>(fy);
      const float right_contrib = tx - fx;
      const float lower_contrib = ty - fy;
      for (int c = 0; c < channels; ++c) {
        const float q_11 = read(iy, ix, c);
        const float q_21 = read(iy, ix + 1, c);
        const float q_12 = read(iy + 1, ix, c);
        const float q_22 = read(iy + 1, ix + 1, c);
        const float upper = (1.0 - right_contrib) * q_11 + right_contrib * q_21;
        const float lower = (1.0 - right_contrib) * q_12 + right_contrib * q_22;
        o[c] = lower_contrib * lower + (1.0 - lower_contrib) * upper;
      }
    }
  }
}

bool ParseTransposeConvBias(const std::vector<uint8_t>& options, bool* padding_same,
                            int* stride_w, int* stride_h, std::string* error) {
  if (options.size() < 12) {
    *error = "Convolution2DTransposeBias: options shorter than TfLiteTransposeConvParams";
    return false;
  }
  int32_t v[3];
  std::memcpy(v, options.data(), sizeof(v));
  // TfLitePadding: 1 = kTfLitePaddingSame, 2 = kTfLitePaddingValid.
  if ((v[0] != 1 && v[0] != 2) || v[1] <= 0 || v[2] <= 0) {
    *error = "Convolution2DTransposeBias: invalid padding or stride";
    return false;
  }
  *padding_same = v[0] == 1;
  *stride_w = v[1];
  *stride_h = v[2];
  return true;
}

bool TransposeConvBiasOutputSize(int in_h, int in_w, int filter_h, int filter_w, int stride_h,
                                 int stride_w, bool padding_same, int* out_h, int* out_w,
                                 int* pad_h, int* pad_w) {
  // Prepare() of transpose_conv_bias.cc in 64-bit arithmetic; false when a size leaves 1..INT_MAX
  // (stride * (in - 1) overflows int for large strides). Needs in_h, in_w, filters, strides >= 1.
  if (in_h < 1 || in_w < 1 || filter_h < 1 || filter_w < 1 || stride_h < 1 || stride_w < 1) {
    return false;
  }
  int64_t ph = 0;
  int64_t pw = 0;
  if (padding_same) {
    ph = std::max<int64_t>(0, static_cast<int64_t>(filter_h) - (in_h - 1) % stride_h - 1);
    pw = std::max<int64_t>(0, static_cast<int64_t>(filter_w) - (in_w - 1) % stride_w - 1);
  }
  const int64_t oh = static_cast<int64_t>(stride_h) * (in_h - 1) + filter_h - ph;
  const int64_t ow = static_cast<int64_t>(stride_w) * (in_w - 1) + filter_w - pw;
  if (oh < 1 || ow < 1 || oh > INT32_MAX || ow > INT32_MAX) return false;
  *out_h = static_cast<int>(oh);
  *out_w = static_cast<int>(ow);
  *pad_h = static_cast<int>(ph);
  *pad_w = static_cast<int>(pw);
  return true;
}

void TransposeConvBias(const float* input, int batch, int in_h, int in_w, int in_c,
                       const float* filter, int out_c, int filter_h, int filter_w,
                       const float* bias, int stride_h, int stride_w, int pad_top, int pad_left,
                       int out_h, int out_w, float* out, std::vector<float>* scratch) {
  // transpose_conv_bias.cc TransposeConvBias: output = bias, then every input pixel scatters
  // input . filter[:, fy, fx, :] to (in * stride - pad + f). The per-tap products are formed with
  // one GEMM per (fy, fx) instead of the original scalar loop (same terms; each tap is summed in
  // ascending in_c order, the taps are added in (fy, fx) order).
  const int rows = in_h * in_w;
  const size_t taps_floats = static_cast<size_t>(rows) * static_cast<size_t>(out_c);
  const size_t need = taps_floats + gemm::PackedSize(out_c, in_c);
  if (scratch->size() < need) scratch->resize(need);
  float* taps = scratch->data();
  float* packed = taps + taps_floats;
  for (int n = 0; n < batch; ++n) {
    float* out_n = out + static_cast<size_t>(n) * out_h * out_w * out_c;
    for (int i = 0; i < out_h * out_w; ++i) {
      std::memcpy(out_n + static_cast<size_t>(i) * out_c, bias, sizeof(float) * out_c);
    }
  }
  const size_t filter_stride = static_cast<size_t>(filter_h) * filter_w * in_c;
  for (int fy = 0; fy < filter_h; ++fy) {
    for (int fx = 0; fx < filter_w; ++fx) {
      gemm::PackWeights(filter + (static_cast<size_t>(fy) * filter_w + fx) * in_c, out_c, in_c,
                        filter_stride, packed);
      for (int n = 0; n < batch; ++n) {
        float* out_n = out + static_cast<size_t>(n) * out_h * out_w * out_c;
        gemm::Gemm(input + static_cast<size_t>(n) * rows * in_c, rows, in_c,
                   static_cast<size_t>(in_c), packed, out_c, taps, static_cast<size_t>(out_c));
        for (int iy = 0; iy < in_h; ++iy) {
          const int oy = iy * stride_h - pad_top + fy;
          if (oy < 0 || oy >= out_h) continue;
          for (int ix = 0; ix < in_w; ++ix) {
            const int ox = ix * stride_w - pad_left + fx;
            if (ox < 0 || ox >= out_w) continue;
            float* o = out_n + (static_cast<size_t>(oy) * out_w + ox) * out_c;
            const float* t = taps + (static_cast<size_t>(iy) * in_w + ix) * out_c;
            for (int c = 0; c < out_c; ++c) o[c] += t[c];
          }
        }
      }
    }
  }
}

}  // namespace custom
}  // namespace ml
}  // namespace slam_refact

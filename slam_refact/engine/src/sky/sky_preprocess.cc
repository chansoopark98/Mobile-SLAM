// See sky_preprocess.h.
#include "sky/sky_preprocess.h"

#include <algorithm>
#include <cmath>

namespace slam_refact::sky {

void CropDimensionByAspectRatio(int input_width, int input_height, float aspect_ratio,
                                int* crop_width, int* crop_height) {
  if (!crop_width || !crop_height || aspect_ratio < 1.0e-5) {
    return;
  }
  const float new_width = input_height * aspect_ratio;
  if (new_width < input_width) {
    *crop_width = static_cast<int>(new_width);
    *crop_height = input_height;
  } else {
    *crop_width = input_width;
    const float new_height = input_width / aspect_ratio;
    *crop_height = static_cast<int>(new_height);
  }
}

QuaternionXyzw GetQuaternion(double alpha, double beta, double gamma) {
  const double deg_to_rad = 3.14159265358979323846 / 180.0;
  auto or_zero = [](double v) { return std::isnan(v) ? 0.0 : v; };
  const double x = or_zero(beta * deg_to_rad);
  const double y = or_zero(gamma * deg_to_rad);
  const double z = or_zero(alpha * deg_to_rad);
  const double cx = std::cos(x / 2), cy = std::cos(y / 2), cz = std::cos(z / 2);
  const double sx = std::sin(x / 2), sy = std::sin(y / 2), sz = std::sin(z / 2);
  QuaternionXyzw q;
  q.w = cx * cy * cz - sx * sy * sz;
  q.x = sx * cy * cz - cx * sy * sz;
  q.y = cx * sy * cz + sx * cy * sz;
  q.z = cx * cy * sz + sx * sy * cz;
  return q;
}

double ScaleFactorSameAspectRatioCrop(double image_w, double image_h, double desired_w,
                                      double desired_h) {
  const double w_ratio = desired_w / image_w;
  const double h_ratio = desired_h / image_h;
  return (w_ratio > h_ratio) ? w_ratio : h_ratio;
}

ModelInputGeometry CalculateOutputSize(int video_width, int video_height) {
  ModelInputGeometry g;
  const double aspect_ratio = static_cast<double>(video_height) / static_cast<double>(video_width);
  if (aspect_ratio > 1) {  // portrait view
    g.model_input_height = 256;
    g.model_input_width = 144;
  } else {  // landscape view
    g.model_input_height = 144;
    g.model_input_width = 256;
  }
  const double scale = ScaleFactorSameAspectRatioCrop(video_width, video_height, g.model_input_width,
                                                      g.model_input_height);
  g.video_resize_width = scale * video_width;
  g.video_resize_height = scale * video_height;
  return g;
}

namespace {

// Writes `v` to a JS Uint8Array of `len` elements at the double index `idx`: only integer indices in
// range are stored.
inline void JsStore(uint8_t* arr, size_t len, double idx, uint8_t v) {
  if (idx < 0 || idx >= static_cast<double>(len) || idx != std::floor(idx)) {
    return;
  }
  arr[static_cast<size_t>(idx)] = v;
}

// Reads a JS Uint8Array element: out of range gives undefined, which stores as 0 (ToUint8(NaN)).
inline uint8_t JsLoad(const uint8_t* arr, size_t len, double idx) {
  if (idx < 0 || idx >= static_cast<double>(len) || idx != std::floor(idx)) {
    return 0;
  }
  return arr[static_cast<size_t>(idx)];
}

}  // namespace

void ToImageArrayLetterbox(const uint8_t* src, int src_w, int src_h, int dst_w, int dst_h,
                           uint8_t* dst) {
  const size_t src_len = static_cast<size_t>(src_w) * src_h * 4;
  const size_t dst_len = static_cast<size_t>(dst_w) * dst_h * 4;
  const double row_offset = (static_cast<double>(dst_h) - src_h) / 2;
  const double col_offset = (static_cast<double>(dst_w) - src_w) / 2;
  for (int r = 0; r < src_h; ++r) {
    double src_idx = 4.0 * r * src_w;
    double dst_idx = 4 * ((r + row_offset) * dst_w + col_offset);
    for (int c = 0; c < src_w; ++c) {
      for (int k = 0; k < 4; ++k) {
        JsStore(dst, dst_len, dst_idx + k, JsLoad(src, src_len, src_idx + k));
      }
      src_idx += 4;
      dst_idx += 4;
    }
  }
}

void ToImageArrayRotateCW90(const uint8_t* src, int src_w, int src_h, int dst_w, int dst_h,
                            uint8_t* dst) {
  const size_t src_len = static_cast<size_t>(src_w) * src_h * 4;
  const size_t dst_len = static_cast<size_t>(dst_w) * dst_h * 4;
  const double src_row_bytes = 4.0 * src_w;
  const double dst_col_offset = (static_cast<double>(dst_w) - src_h) / 2;
  const double dst_row_offset = (static_cast<double>(dst_h) - src_w) / 2;
  const double dst_row_bytes = 4.0 * dst_w;
  for (int c = 0; c < src_w; ++c) {
    double src_idx = (src_h - 1) * src_row_bytes + 4.0 * c;
    double dst_idx = dst_row_bytes * (dst_row_offset + c) + 4 * dst_col_offset;
    for (int r = src_h - 1; r >= 0; --r) {
      for (int k = 0; k < 4; ++k) {
        JsStore(dst, dst_len, dst_idx + k, JsLoad(src, src_len, src_idx + k));
      }
      src_idx -= src_row_bytes;
      dst_idx += 4;
    }
  }
}

void ToImageArrayRotateCCW90(const uint8_t* src, int src_w, int src_h, int dst_w, int dst_h,
                             uint8_t* dst) {
  const size_t src_len = static_cast<size_t>(src_w) * src_h * 4;
  const size_t dst_len = static_cast<size_t>(dst_w) * dst_h * 4;
  const double src_row_bytes = 4.0 * src_w;
  const double dst_col_offset = (static_cast<double>(dst_w) - src_h) / 2;
  const double dst_row_offset = (static_cast<double>(dst_h) - src_w) / 2;
  const double dst_row_bytes = 4.0 * dst_w;
  for (int c = 0; c < src_w; ++c) {
    double src_idx = 4.0 * (src_w - 1 - c);
    double dst_idx = dst_row_bytes * (dst_row_offset + c) + 4 * dst_col_offset;
    for (int r = src_h - 1; r >= 0; --r) {
      for (int k = 0; k < 4; ++k) {
        JsStore(dst, dst_len, dst_idx + k, JsLoad(src, src_len, src_idx + k));
      }
      src_idx += src_row_bytes;
      dst_idx += 4;
    }
  }
}

float QuantizeFilterWeight(float w) {
  // 8 fractional bits, round to nearest: the sampler precision the recorded original shows (spec §3.4).
  return std::floor(w * 256.0f + 0.5f) / 256.0f;
}

uint8_t ToUnorm8(float v) {
  if (!(v > 0.0f)) {
    return 0;
  }
  if (v >= 1.0f) {
    return 255;
  }
  return static_cast<uint8_t>(std::lround(static_cast<double>(v) * 255.0));
}

void SampleBilinearClamp(const Rgba8Image& tex, double u, double v, float out[4]) {
  const double x = u * tex.width - 0.5;
  const double y = v * tex.height - 0.5;
  const double x0f = std::floor(x);
  const double y0f = std::floor(y);
  const float fx = QuantizeFilterWeight(static_cast<float>(x - x0f));
  const float fy = QuantizeFilterWeight(static_cast<float>(y - y0f));
  auto clampi = [](double v, int hi) {
    return static_cast<int>(std::min<double>(std::max<double>(v, 0.0), hi));
  };
  const int xa = clampi(x0f, tex.width - 1);
  const int xb = clampi(x0f + 1, tex.width - 1);
  const int ya = clampi(y0f, tex.height - 1);
  const int yb = clampi(y0f + 1, tex.height - 1);
  const uint8_t* p00 = tex.at(xa, ya);
  const uint8_t* p10 = tex.at(xb, ya);
  const uint8_t* p01 = tex.at(xa, yb);
  const uint8_t* p11 = tex.at(xb, yb);
  for (int k = 0; k < 4; ++k) {
    const float top = static_cast<float>(p00[k]) * (1.0f - fx) + static_cast<float>(p10[k]) * fx;
    const float bot = static_cast<float>(p01[k]) * (1.0f - fx) + static_cast<float>(p11[k]) * fx;
    out[k] = (top * (1.0f - fy) + bot * fy) / 255.0f;
  }
}

Rgba8Image ResizeCameraToModelInput(const Rgba8Image& camera, int dest_w, int dest_h, int vp_x,
                                    int vp_y, int vp_w, int vp_h) {
  Rgba8Image out(dest_w, dest_h);
  if (camera.empty() || vp_w <= 0 || vp_h <= 0) {
    return out;
  }
  for (int y = 0; y < dest_h; ++y) {
    for (int x = 0; x < dest_w; ++x) {
      const double u = ((x + 0.5) - vp_x) / vp_w;
      const double v = ((y + 0.5) - vp_y) / vp_h;
      float c[4];
      SampleBilinearClamp(camera, u, v, c);
      uint8_t* o = out.at(x, y);
      for (int k = 0; k < 4; ++k) {
        o[k] = ToUnorm8(c[k]);
      }
    }
  }
  return out;
}

}  // namespace slam_refact::sky

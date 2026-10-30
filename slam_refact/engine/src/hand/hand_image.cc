// See hand_image.h.
#include "hand/hand_image.h"

#include <algorithm>
#include <climits>
#include <cmath>
#include <cstdlib>

#include <Eigen/Dense>

namespace slam_refact::hand {

Image3 RgbaToRgb(const uint8_t* rgba, int width, int height, int stride) {
  Image3 out(width, height);
  for (int y = 0; y < height; ++y) {
    const uint8_t* s = rgba + static_cast<size_t>(y) * stride;
    uint8_t* d = out.at(0, y);
    for (int x = 0; x < width; ++x) {
      d[3 * x] = s[4 * x];
      d[3 * x + 1] = s[4 * x + 1];
      d[3 * x + 2] = s[4 * x + 2];
    }
  }
  return out;
}

namespace {

constexpr int kCoefBits = 11;  // INTER_RESIZE_COEF_BITS
constexpr int kCoefScale = 1 << kCoefBits;

int CvFloor(float v) { return static_cast<int>(std::floor(v)); }
int CvRoundF(float v) { return static_cast<int>(std::lrint(v)); }

}  // namespace

Image3 ResizeLinear(const Image3& src, int dw, int dh) {
  if (src.empty() || dw <= 0 || dh <= 0) {
    return Image3();
  }
  if (src.width == dw && src.height == dh) {
    return src;
  }
  const int sw = src.width, sh = src.height;
  const double scale_x = static_cast<double>(sw) / dw;
  const double scale_y = static_cast<double>(sh) / dh;
  std::vector<int> xofs(dw);
  std::vector<short> alpha(static_cast<size_t>(dw) * 2);
  for (int dx = 0; dx < dw; ++dx) {
    float fx = static_cast<float>((dx + 0.5) * scale_x - 0.5);
    int sx = CvFloor(fx);
    fx -= static_cast<float>(sx);
    if (sx < 0) {
      fx = 0;
      sx = 0;
    }
    if (sx >= sw - 1) {
      fx = 0;
      sx = sw - 1;
    }
    xofs[dx] = sx;
    alpha[2 * dx] = static_cast<short>(CvRoundF((1.f - fx) * kCoefScale));
    alpha[2 * dx + 1] = static_cast<short>(CvRoundF(fx * kCoefScale));
  }
  Image3 dst(dw, dh);
  std::vector<int> rows(2 * static_cast<size_t>(dw) * 3);
  std::vector<int> row_ids(2, -1);
  auto fill_row = [&](int sy, int* out) {
    const uint8_t* s = src.at(0, sy);
    for (int dx = 0; dx < dw; ++dx) {
      const int sx = xofs[dx];
      const int sx1 = std::min(sx + 1, sw - 1);
      for (int c = 0; c < 3; ++c) {
        out[3 * dx + c] = s[3 * sx + c] * alpha[2 * dx] + s[3 * sx1 + c] * alpha[2 * dx + 1];
      }
    }
  };
  for (int dy = 0; dy < dh; ++dy) {
    float fy = static_cast<float>((dy + 0.5) * scale_y - 0.5);
    int sy = CvFloor(fy);
    fy -= static_cast<float>(sy);
    const short b0 = static_cast<short>(CvRoundF((1.f - fy) * kCoefScale));
    const short b1 = static_cast<short>(CvRoundF(fy * kCoefScale));
    const int ya = std::min(std::max(sy, 0), sh - 1);
    const int yb = std::min(std::max(sy + 1, 0), sh - 1);
    int* r0 = rows.data();
    int* r1 = rows.data() + static_cast<size_t>(dw) * 3;
    fill_row(ya, r0);
    fill_row(yb, r1);
    uint8_t* d = dst.at(0, dy);
    for (size_t i = 0; i < static_cast<size_t>(dw) * 3; ++i) {
      const int v = (static_cast<int>(b0) * r0[i] + static_cast<int>(b1) * r1[i] +
                     (1 << (2 * kCoefBits - 1))) >>
                    (2 * kCoefBits);
      d[i] = static_cast<uint8_t>(std::min(std::max(v, 0), 255));
    }
  }
  return dst;
}

Image3 FitToSize(const Image3& src, int out_w, int out_h, std::array<float, 4>* padding) {
  const int input_width = src.width, input_height = src.height;
  const float scale =
      std::min(static_cast<float>(out_w) / input_width, static_cast<float>(out_h) / input_height);
  const int target_width = static_cast<int>(std::round(input_width * scale));
  const int target_height = static_cast<int>(std::round(input_height * scale));
  const Image3 inter = ResizeLinear(src, target_width, target_height);
  const int top = (out_h - target_height) / 2;
  const int left = (out_w - target_width) / 2;
  Image3 out(out_w, out_h);  // BORDER_CONSTANT 0
  for (int y = 0; y < target_height; ++y) {
    if (top + y < 0 || top + y >= out_h) continue;
    for (int x = 0; x < target_width; ++x) {
      if (left + x < 0 || left + x >= out_w) continue;
      std::copy(inter.at(x, y), inter.at(x, y) + 3, out.at(left + x, top + y));
    }
  }
  if (padding != nullptr) {
    // ComputeOutputLetterboxPadding (scale mode FIT, rotation 0).
    padding->fill(0.0f);
    const float input_aspect_ratio = static_cast<float>(input_width) / input_height;
    const float output_aspect_ratio = static_cast<float>(out_w) / out_h;
    if (input_aspect_ratio < output_aspect_ratio) {
      (*padding)[0] = (1.f - input_aspect_ratio / output_aspect_ratio) / 2.f;
      (*padding)[2] = (*padding)[0];
    } else if (output_aspect_ratio < input_aspect_ratio) {
      (*padding)[1] = (1.f - output_aspect_ratio / input_aspect_ratio) / 2.f;
      (*padding)[3] = (*padding)[1];
    }
  }
  return out;
}

std::array<std::array<float, 2>, 4> BoxPoints(float cx, float cy, float w, float h,
                                              float angle_deg) {
  // cv::RotatedRect::points.
  const double angle = static_cast<double>(angle_deg) * 3.14159265358979323846 / 180.0;
  const float b = static_cast<float>(std::cos(angle)) * 0.5f;
  const float a = static_cast<float>(std::sin(angle)) * 0.5f;
  std::array<std::array<float, 2>, 4> pt{};
  pt[0][0] = cx - a * h - b * w;
  pt[0][1] = cy + b * h - a * w;
  pt[1][0] = cx + a * h - b * w;
  pt[1][1] = cy - b * h - a * w;
  pt[2][0] = 2 * cx - pt[0][0];
  pt[2][1] = 2 * cy - pt[0][1];
  pt[3][0] = 2 * cx - pt[1][0];
  pt[3][1] = 2 * cy - pt[1][1];
  return pt;
}

std::array<double, 9> GetPerspectiveTransform(const std::array<std::array<float, 2>, 4>& src,
                                              const std::array<std::array<float, 2>, 4>& dst) {
  // cv::getPerspectiveTransform: the 8x8 system, LU with partial pivoting, M[8] = 1.
  Eigen::Matrix<double, 8, 8> A = Eigen::Matrix<double, 8, 8>::Zero();
  Eigen::Matrix<double, 8, 1> b;
  for (int i = 0; i < 4; ++i) {
    const double sx = src[i][0], sy = src[i][1], dx = dst[i][0], dy = dst[i][1];
    A(i, 0) = sx;
    A(i, 1) = sy;
    A(i, 2) = 1;
    A(i, 6) = -sx * dx;
    A(i, 7) = -sy * dx;
    A(i + 4, 3) = sx;
    A(i + 4, 4) = sy;
    A(i + 4, 5) = 1;
    A(i + 4, 6) = -sx * dy;
    A(i + 4, 7) = -sy * dy;
    b(i) = dx;
    b(i + 4) = dy;
  }
  const Eigen::Matrix<double, 8, 1> x = A.partialPivLu().solve(b);
  std::array<double, 9> m{};
  for (int i = 0; i < 8; ++i) m[i] = x(i);
  m[8] = 1.0;
  return m;
}

Image3 CropNormRect(const Image3& src, const NormRect& rect) {
  const int src_w = src.width, src_h = src.height;
  // GetCropSpecs (NORM_RECT branch).
  int crop_w = src_w, crop_h = src_h;
  int x_center = src_w / 2, y_center = src_h / 2;
  float rotation = 0.0f;
  float norm_w = 0.0f, norm_h = 0.0f;
  if (rect.width > 0.0 && rect.height > 0.0) {
    norm_w = rect.width;
    norm_h = rect.height;
    x_center = static_cast<int>(std::round(rect.x_center * src_w));
    y_center = static_cast<int>(std::round(rect.y_center * src_h));
    rotation = rect.rotation;
  }
  if (norm_w > 0 && norm_h > 0) {
    crop_w = static_cast<int>(std::round(norm_w * src_w));
    crop_h = static_cast<int>(std::round(norm_h * src_h));
  }
  if (crop_w <= 0 || crop_h <= 0) {
    return Image3();
  }
  const float angle = static_cast<float>(rotation * 180.f / 3.14159265358979323846);
  const auto src_points = BoxPoints(static_cast<float>(x_center), static_cast<float>(y_center),
                                    static_cast<float>(crop_w), static_cast<float>(crop_h), angle);
  const std::array<std::array<float, 2>, 4> dst_points = {{{0.0f, static_cast<float>(crop_h - 1)},
                                                           {0.0f, 0.0f},
                                                           {static_cast<float>(crop_w - 1), 0.0f},
                                                           {static_cast<float>(crop_w - 1),
                                                            static_cast<float>(crop_h - 1)}}};
  const std::array<double, 9> m = GetPerspectiveTransform(src_points, dst_points);
  // warpPerspective inverts the matrix unless WARP_INVERSE_MAP is set.
  Eigen::Matrix3d M;
  M << m[0], m[1], m[2], m[3], m[4], m[5], m[6], m[7], m[8];
  const Eigen::Matrix3d Mi = M.inverse();
  Image3 dst(crop_w, crop_h);
  for (int y = 0; y < crop_h; ++y) {
    const double x0 = Mi(0, 1) * y + Mi(0, 2);
    const double y0 = Mi(1, 1) * y + Mi(1, 2);
    const double w0 = Mi(2, 1) * y + Mi(2, 2);
    for (int x = 0; x < crop_w; ++x) {
      double w = w0 + Mi(2, 0) * x;
      w = w != 0.0 ? 1. / w : 0.0;
      const double fx = std::max<double>(INT_MIN, std::min<double>(INT_MAX, (x0 + Mi(0, 0) * x) * w));
      const double fy = std::max<double>(INT_MIN, std::min<double>(INT_MAX, (y0 + Mi(1, 0) * x) * w));
      const int sx = std::min(std::max(static_cast<int>(std::lrint(fx)), 0), src_w - 1);
      const int sy = std::min(std::max(static_cast<int>(std::lrint(fy)), 0), src_h - 1);
      std::copy(src.at(sx, sy), src.at(sx, sy) + 3, dst.at(x, y));
    }
  }
  return dst;
}

void ImageToTensor(const Image3& img, bool zero_center, float* out) {
  float div, sub;
  if (zero_center) {
    div = 127.5f;
    sub = 1.0f;
  } else {
    div = 255.0f;
    sub = 0.0f;
  }
  for (size_t i = 0; i < img.px.size(); ++i) {
    out[i] = img.px[i] / div - sub;
  }
}

}  // namespace slam_refact::hand

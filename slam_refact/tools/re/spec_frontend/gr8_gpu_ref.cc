// See gr8_gpu_ref.h. Citations are to public xr_reference/xr_reference@4edfa484 paths under
// assets/references/xr_reference/official/xr_reference/.
#include "gr8_gpu_ref.h"

#include <algorithm>
#include <cmath>

namespace spec_frontend {
namespace {

constexpr int kFilteredWidth = 720;   // gl-reality-frame.cc:108
constexpr int kFilteredHeight = 960;  // gl-reality-frame.cc:109
constexpr float kRoiAspect = 0.75f;   // c8/pixels/image-roi.h:9

int RoundToPowerOfTwo(int v) {  // gl-reality-frame.cc:97-106
  v--;
  v |= v >> 1;
  v |= v >> 2;
  v |= v >> 4;
  v |= v >> 8;
  v |= v >> 16;
  v++;
  return v;
}

bool g_swiftshader_approx = false;  // set from Gr8GpuParams in Draw()

inline float ToFloat(uint8_t b) {
  return g_swiftshader_approx ? static_cast<float>(b) * (1.0f / 255.0f) : static_cast<float>(b) / 255.0f;
}

// unorm8 write: round to nearest (GLES 3.0 2.1.6.1 conversion), clamp to [0, 1].
inline uint8_t Quant(float f) {
  if (!(f > 0.0f)) return 0;  // also maps NaN to 0
  if (f >= 1.0f) return 255;
  const float y = f * 255.0f;
  return static_cast<uint8_t>(g_swiftshader_approx ? std::nearbyint(y) : std::floor(y + 0.5f));
}

inline float GlslMin(float x, float y) { return y < x ? y : x; }
inline float GlslMax(float x, float y) { return x < y ? y : x; }
inline float GlslClamp(float x, float lo, float hi) { return GlslMin(GlslMax(x, lo), hi); }
inline float GlslMix(float x, float y, float a) {
  return g_swiftshader_approx ? x + a * (y - x) : x * (1.0f - a) + y * a;
}
inline float GlslFract(float x) { return x - std::floor(x); }

struct V4 {
  float x, y, z, w;
};
inline V4 Neg(V4 a) { return {-a.x, -a.y, -a.z, -a.w}; }

// gr8-stage2.frag:17-23 (literal component order).
V4 Min9Quad(V4 da, V4 db, V4 dc) {
  const float im2x = GlslMin(db.x, db.z);
  const float im2y = GlslMin(db.y, db.w);
  const float inner_min = GlslMin(im2x, im2y);
  const float o4x = GlslMin(da.y, da.w);
  const float o4y = GlslMin(dc.y, da.w);
  const float o4z = GlslMin(da.z, dc.x);
  const float o4w = GlslMin(dc.z, dc.x);
  const float omx = GlslMin(GlslMin(o4x, o4z), inner_min);
  const float omy = GlslMin(GlslMin(o4y, o4w), inner_min);
  return {GlslMin(da.x, omx), GlslMin(da.z, omy), GlslMin(dc.y, omx), GlslMin(dc.w, omy)};
}

// gr8-stage2.frag:25-30.
float Max9Full(V4 a, V4 b, V4 c, V4 d) {
  const V4 m1 = Min9Quad(a, b, c);
  const V4 m2 = Min9Quad(b, c, d);
  const V4 m3 = Min9Quad(c, d, a);
  const V4 m4 = Min9Quad(d, a, b);
  auto vmax = [](V4 p, V4 q) {
    return V4{GlslMax(p.x, q.x), GlslMax(p.y, q.y), GlslMax(p.z, q.z), GlslMax(p.w, q.w)};
  };
  const V4 m = vmax(m1, vmax(m2, vmax(m3, m4)));
  const float m2x = GlslMax(m.x, m.z);
  const float m2y = GlslMax(m.y, m.w);
  return GlslMax(m2x, m2y);
}

float ScoreGr8(V4 d0, V4 d4, V4 d8, V4 d12) {  // gr8-stage2.frag:32-34
  return GlslMax(Max9Full(d0, d4, d8, d12), Max9Full(Neg(d0), Neg(d4), Neg(d8), Neg(d12)));
}

// GLSL dot products, evaluated left to right.
inline float Dot3(float a0, float a1, float a2, float b0, float b1, float b2) {
  return (a0 * b0 + a1 * b1) + a2 * b2;
}
inline float Dot4(float a0, float a1, float a2, float a3, float b0, float b1, float b2, float b3) {
  return ((a0 * b0 + a1 * b1) + a2 * b2) + a3 * b3;
}

constexpr float kG7_0 = 0.070159f;  // gr8-stage2.frag:65-66, gr8-stage3.frag:264-266
constexpr float kG7_1 = 0.131075f;
constexpr float kG7_2 = 0.190713f;
constexpr float kG7_3 = 0.216106f;

// gl rotate(-only).vert uv -> texUv mapping.
inline void RotateUv(int rotation, float u, float v, float *tu, float *tv) {
  if (rotation == -90 || rotation == 270) {
    *tu = 1.0f - v;
    *tv = u;
  } else if (rotation == 180 || rotation == -180) {
    *tu = 1.0f - u;
    *tv = 1.0f - v;
  } else if (rotation == -270 || rotation == 90) {
    *tu = v;
    *tv = 1.0f - u;
  } else {
    *tu = u;
    *tv = v;
  }
}

inline const uint8_t *Nearest(const Tex &t, int x, int y) {  // NEAREST + CLAMP_TO_EDGE
  x = std::min(std::max(x, 0), t.w - 1);
  y = std::min(std::max(y, 0), t.h - 1);
  return t.At(x, y);
}

}  // namespace

std::array<float, 6> Gr8GpuRef::SubpixelParams() {  // gr8-feature-shader.cc:111-138
  constexpr double p0 = 0.975316;
  constexpr double p1 = 0.006133;
  constexpr double p2 = 0.000039;
  constexpr double k0 = p0 * p1;
  constexpr double k1 = p0 * p2;
  constexpr double k2 = p1 * p2;
  constexpr double k3 = k1 + 2.0 * k2;
  constexpr double k4 = k0 + 2.0 * k1;
  constexpr double k5 = k0 + 4.0 * (k1 + k2);
  constexpr double k6 = 2.0 * p1 + 4.0 * p2;
  return {static_cast<float>(0.5 * k3 / k5), static_cast<float>(0.5 * k4 / k5),
          static_cast<float>(p1 / k6), static_cast<float>(p2 / k6), static_cast<float>(k2 / k5),
          static_cast<float>((k4 + 2.0 * k3 - 4.0 * k2) / k5)};
}

float Max9FullScore(const float d[16]) {
  return ScoreGr8({d[0], d[1], d[2], d[3]}, {d[4], d[5], d[6], d[7]}, {d[8], d[9], d[10], d[11]},
                  {d[12], d[13], d[14], d[15]});
}

float Fast9Textbook(const float d[16]) {
  float best = -1e30f;
  for (int sign = 0; sign < 2; ++sign) {
    for (int k = 0; k < 16; ++k) {
      float m = 1e30f;
      for (int j = 0; j < 9; ++j) {
        const float v = d[(k + j) % 16];
        m = std::min(m, sign ? -v : v);
      }
      best = std::max(best, m);
    }
  }
  return best;
}

void Gr8GpuRef::InitializeLevels(int src_w, int src_h, int out_w, int out_h) {
  // gl-reality-frame.cc:115-196 (types kept: targetHeight is int, targetWidth float).
  int target_height = 640;
  if (out_h > 2047 && out_w > 2047) {
    tex_w_ = tex_h_ = 2048;
    target_height = 1280;
  } else if (out_h > 1023 && out_w > 1023) {
    tex_w_ = tex_h_ = 1024;
    target_height = 640;
  } else if (out_h > 511 && out_w > 511) {
    tex_w_ = tex_h_ = 512;
    target_height = 320;
  } else if (out_h > 255 && out_w > 255) {
    tex_w_ = tex_h_ = 256;
    target_height = 160;
  } else if (out_h > 127 && out_w > 127) {
    tex_w_ = tex_h_ = 128;
    target_height = 80;
  }
  const float src_aspect = static_cast<float>(src_w) / src_h;
  float target_width = src_aspect * target_height;
  if (target_width > target_height * 3 / 4) {
    const float scale = (target_height * 3 / 4) / target_width;
    target_width *= scale;
    target_height = static_cast<int>(target_height * scale);  // int *= float truncates
  }
  levels_.assign(4, Layout{});
  for (int i = 0; i < 4; ++i) {
    const float scale = 1.0f / std::pow(1.44, i);
    levels_[i].w = static_cast<int>(target_width * scale);
    levels_[i].h = static_cast<int>(target_height * scale);
  }
  levels_[0].r = 3;
  levels_[0].c = 3;
  levels_[1].r = 3;
  levels_[1].c = levels_[0].w + 3 + 3;
  std::swap(levels_[1].h, levels_[1].w);
  levels_[1].rotated = true;
  levels_[2].r = levels_[1].h + 3 + 3;
  levels_[2].c = levels_[0].w + 3 + 3;
  levels_[3].r = levels_[2].r;
  levels_[3].c = levels_[2].c + levels_[2].w + 3;

  const int roi_max_width = (tex_w_ - 15) / 4;
  int roi_max_height = tex_h_;
  int roi_y = 0;
  for (const auto &l : levels_) {
    const int r = l.r + l.h + 3;
    const int h = tex_h_ - r - 3;
    if (h < roi_max_height) {
      roi_max_height = h;
      roi_y = r;
    }
  }
  const int roi_height = roi_max_height * kRoiAspect > roi_max_width
                           ? static_cast<int>(roi_max_width / kRoiAspect)
                           : roi_max_height;
  const int roi_width = roi_max_width / kRoiAspect > roi_max_height
                          ? static_cast<int>(roi_max_height * kRoiAspect)
                          : roi_max_width;
  roi_layouts_.assign(4, Layout{});
  for (int i = 0; i < 4; ++i) {
    roi_layouts_[i] = {roi_y, i * (3 + roi_width) + 3, roi_width, roi_height, false};
  }
}

void Gr8GpuRef::Initialize(int src_w, int src_h, const Gr8GpuParams &params) {
  params_ = params;
  const bool rotate = src_w > src_h;  // gl-reality-frame.cc:203-208
  input_w_ = rotate ? src_h : src_w;
  input_h_ = rotate ? src_w : src_h;
  filtered_w_ = rotate ? kFilteredHeight : kFilteredWidth;
  filtered_h_ = rotate ? kFilteredWidth : kFilteredHeight;
  InitializeLevels(input_w_, input_h_, params.pyramid_size, params.pyramid_size);
  camera_.Init(src_w, src_h);
  filtered_.Init(RoundToPowerOfTwo(filtered_w_), RoundToPowerOfTwo(filtered_h_));
  dest1_.Init(tex_w_, tex_h_);
  dest2_.Init(tex_w_, tex_h_);
  dest3_.Init(tex_w_, tex_h_);
}

void Gr8GpuRef::SampleLinear(const Tex &t, float u, float v, float out[4]) const {
  const float s = u * static_cast<float>(t.w) - 0.5f;
  const float q = v * static_cast<float>(t.h) - 0.5f;
  const float fs = std::floor(s);
  const float fq = std::floor(q);
  float a = s - fs;
  float b = q - fq;
  if (params_.linear_weight_bits > 0) {
    const float k = static_cast<float>(1 << params_.linear_weight_bits);
    a = std::floor(a * k + 0.5f) / k;
    b = std::floor(b * k + 0.5f) / k;
  }
  const int x0 = static_cast<int>(fs);
  const int y0 = static_cast<int>(fq);
  const uint8_t *t00 = Nearest(t, x0, y0);
  const uint8_t *t10 = Nearest(t, x0 + 1, y0);
  const uint8_t *t01 = Nearest(t, x0, y0 + 1);
  const uint8_t *t11 = Nearest(t, x0 + 1, y0 + 1);
  for (int ch = 0; ch < 4; ++ch) {
    const float top = GlslMix(ToFloat(t00[ch]), ToFloat(t10[ch]), a);
    const float bot = GlslMix(ToFloat(t01[ch]), ToFloat(t11[ch]), a);
    out[ch] = GlslMix(top, bot, b);
  }
}

void Gr8GpuRef::InputStage() {  // gl-reality-frame.cc:311-334 + gr8-input.frag
  const float sx = static_cast<float>(1.0 / input_w_);
  const float sy = static_cast<float>(1.0 / input_h_);
  constexpr float p00 = 0.268958f, p11 = 0.124827f, p22 = 0.057934f;
  const int taps[9][2] = {{-1, -1}, {0, -1}, {1, -1}, {-1, 0}, {0, 0},
                          {1, 0},   {-1, 1}, {0, 1}, {1, 1}};
  const float wts[9] = {p22, p11, p22, p11, p00, p11, p22, p11, p22};
  for (int y = 0; y < filtered_h_; ++y) {
    const float v = (static_cast<float>(y) + 0.5f) / static_cast<float>(filtered_h_);
    for (int x = 0; x < filtered_w_; ++x) {
      const float u = (static_cast<float>(x) + 0.5f) / static_cast<float>(filtered_w_);
      float acc[3] = {0.0f, 0.0f, 0.0f};
      for (int k = 0; k < 9; ++k) {
        float s[4];
        SampleLinear(camera_, sx * taps[k][0] + u, sy * taps[k][1] + v, s);
        for (int ch = 0; ch < 3; ++ch) acc[ch] = acc[ch] + wts[k] * s[ch];
      }
      uint8_t *o = filtered_.At(x, y);
      o[0] = Quant(acc[0]);
      o[1] = Quant(acc[1]);
      o[2] = Quant(acc[2]);
      o[3] = 255;
    }
  }
}

void Gr8GpuRef::Stage1() {  // gl-reality-frame.cc:336-458 + gr8-rotate.vert + gr8-stage1.frag
  const Layout &l = levels_[0];
  const int shader_rotation = l.rotated ? params_.rotation - 90 : params_.rotation;
  const float scale_x = static_cast<float>(1.0 / filtered_.w);
  const float scale_y = static_cast<float>(1.0 / filtered_.h);
  const float pix_x = static_cast<float>(filtered_w_);
  const float pix_y = static_cast<float>(filtered_h_);
  for (int y = 0; y < l.h; ++y) {
    float v = (static_cast<float>(y) + 0.5f) / static_cast<float>(l.h);
    if (params_.flip_y > 0) v = 1.0f - v;
    for (int x = 0; x < l.w; ++x) {
      const float u = (static_cast<float>(x) + 0.5f) / static_cast<float>(l.w);
      float tu, tv;
      RotateUv(shader_rotation, u, v, &tu, &tv);
      float s[4];
      SampleLinear(filtered_, scale_x * (0.0f + pix_x * tu), scale_y * (0.0f + pix_y * tv), s);
      const float gray = Dot3(0.299f, 0.587f, 0.114f, s[0], s[1], s[2]);
      uint8_t *o = dest1_.At(l.c + x, l.r + y);
      o[0] = Quant(gray);
      o[1] = 0;
      o[2] = 0;
      o[3] = 255;
    }
  }
  // No ROIs are requested in world tracking: every ROI layout is cleared (clear = 1).
  for (const auto &rl : roi_layouts_) {
    for (int y = 0; y < rl.h; ++y) {
      for (int x = 0; x < rl.w; ++x) {
        uint8_t *o = dest1_.At(rl.c + x, rl.r + y);
        o[0] = o[1] = o[2] = 0;
        o[3] = 255;
      }
    }
  }
}

void Gr8GpuRef::PyramidBlur(int level) {  // gl-reality-frame.cc:253-276 + gr8-pyramid-blur.frag
  const Layout &l = levels_[level];
  const float lw = static_cast<float>(l.w), lh = static_cast<float>(l.h);
  const float lc = static_cast<float>(l.c), lr = static_cast<float>(l.r);
  const int taps[9][2] = {{-1, -1}, {0, -1}, {1, -1}, {-1, 0}, {0, 0},
                          {1, 0},   {-1, 1}, {0, 1}, {1, 1}};
  const float wts[9] = {0.054659f, 0.124474f, 0.054659f, 0.124474f, 0.283464f,
                        0.124474f, 0.054659f, 0.124474f, 0.054659f};
  for (int y = 0; y < l.h; ++y) {
    const float v = (static_cast<float>(y) + 0.5f) / lh;
    for (int x = 0; x < l.w; ++x) {
      const float u = (static_cast<float>(x) + 0.5f) / lw;
      float t[9];
      for (int k = 0; k < 9; ++k) {
        const float px = (lw * u + lc) + static_cast<float>(taps[k][0]);
        const float py = (lh * v + lr) + static_cast<float>(taps[k][1]);
        t[k] = ToFloat(Nearest(dest1_, static_cast<int>(std::floor(px)), static_cast<int>(std::floor(py)))[0]);
      }
      float blur;
      if (params_.binary_expression_forms) {
        // xr-slam-strings.txt 8019: w(1,1)T(1,1) + (w(0,1)T(0,1) + (... + (w(-1,-1)T(-1,-1) + w(0,-1)T(0,-1))))
        blur = wts[0] * t[0] + wts[1] * t[1];
        blur = wts[2] * t[2] + blur;
        blur = wts[3] * t[3] + blur;
        blur = wts[4] * t[4] + blur;
        blur = wts[5] * t[5] + blur;
        blur = wts[6] * t[6] + blur;
        blur = wts[7] * t[7] + blur;
        blur = wts[8] * t[8] + blur;
      } else {
        blur = wts[0] * t[0];
        for (int k = 1; k < 9; ++k) blur = blur + wts[k] * t[k];
      }
      uint8_t *o = dest2_.At(l.c + x, l.r + y);
      o[0] = Quant(blur);
      o[1] = 0;
      o[2] = 0;
      o[3] = 255;
    }
  }
}

void Gr8GpuRef::PyramidDecimate(int level) {  // gl-reality-frame.cc:278-310 + decimate.frag
  const Layout &ol = levels_[level];
  const Layout &l = levels_[level + 1];
  const int o_rot = ol.rotated ? params_.rotation - 90 : params_.rotation;
  const int n_rot = l.rotated ? params_.rotation - 90 : params_.rotation;
  const int rotation_diff = n_rot - o_rot;
  const float ow = static_cast<float>(ol.w), oh = static_cast<float>(ol.h);
  const float oc = static_cast<float>(ol.c), orr = static_cast<float>(ol.r);
  for (int y = 0; y < l.h; ++y) {
    const float v = (static_cast<float>(y) + 0.5f) / static_cast<float>(l.h);
    for (int x = 0; x < l.w; ++x) {
      const float u = (static_cast<float>(x) + 0.5f) / static_cast<float>(l.w);
      float tu, tv;
      RotateUv(rotation_diff, u, v, &tu, &tv);
      const float pix_x = ow * tu + oc;
      const float pix_y = oh * tv + orr;
      const float lt_x = std::floor(pix_x);
      const float lt_y = std::floor(pix_y);
      const float fx = pix_x - lt_x;
      const float fy = pix_y - lt_y;
      const int li = static_cast<int>(lt_x), ti = static_cast<int>(lt_y);
      const float v00 = ToFloat(Nearest(dest2_, li, ti)[0]);
      const float v10 = ToFloat(Nearest(dest2_, li + 1, ti)[0]);
      const float v01 = ToFloat(Nearest(dest2_, li, ti + 1)[0]);
      const float v11 = ToFloat(Nearest(dest2_, li + 1, ti + 1)[0]);
      const float tx = GlslMix(v00, v10, fx);
      const float bx = GlslMix(v01, v11, fx);
      const float val = GlslMix(tx, bx, fy);
      uint8_t *o = dest1_.At(l.c + x, l.r + y);
      o[0] = Quant(val);
      o[1] = 0;
      o[2] = 0;
      o[3] = 255;
    }
  }
}

void Gr8GpuRef::Stage2() {  // gr8-stage2.frag (C8_USE_ALTERNATE_FEATURE_SCORE = 0)
  auto r = [&](int x, int y) { return ToFloat(Nearest(dest1_, x, y)[0]); };
  for (int y = 0; y < tex_h_; ++y) {
    for (int x = 0; x < tex_w_; ++x) {
      const float v02 = r(x - 1, y - 3), v03 = r(x, y - 3), v04 = r(x + 1, y - 3);
      const float v11 = r(x - 2, y - 2), v15 = r(x + 2, y - 2);
      const float v20 = r(x - 3, y - 1), v26 = r(x + 3, y - 1);
      const float v30 = r(x - 3, y), v31 = r(x - 2, y), v32 = r(x - 1, y);
      const float c = r(x, y);
      const float v34 = r(x + 1, y), v35 = r(x + 2, y), v36 = r(x + 3, y);
      const float v40 = r(x - 3, y + 1), v46 = r(x + 3, y + 1);
      const float v51 = r(x - 2, y + 2), v55 = r(x + 2, y + 2);
      const float v62 = r(x - 1, y + 3), v63 = r(x, y + 3), v64 = r(x + 1, y + 3);

      const float g = Dot4(kG7_0, kG7_1, kG7_2, kG7_3, v30, v31, v32, c) +
                      Dot3(kG7_2, kG7_1, kG7_0, v34, v35, v36);
      const V4 d0{v30 - c, v20 - c, v11 - c, v02 - c};
      const V4 d4{v03 - c, v04 - c, v15 - c, v26 - c};
      const V4 d8{v36 - c, v46 - c, v55 - c, v64 - c};
      const V4 d12{v63 - c, v62 - c, v51 - c, v40 - c};
      const float score = ScoreGr8(d0, d4, d8, d12);
      const float clmp = GlslClamp(score, 0.0f, 0.99999f);
      constexpr float kUnit = 1.0f / 255.0f;
      float low_bits, packed_hi;
      if (params_.binary_expression_forms) {
        // xr-slam-strings.txt 8674-8678 (no FMA contraction assumed).
        const float fl = std::floor(255.0f * clmp);
        low_bits = 255.0f * clmp + (-fl);
        packed_hi = (-kUnit) * low_bits + clmp;
      } else {
        const float scale_up = 255.0f * clmp;
        low_bits = scale_up - std::floor(scale_up);
        packed_hi = clmp - kUnit * low_bits;
      }
      uint8_t *o = dest2_.At(x, y);
      o[0] = Quant(c);
      o[1] = Quant(g);
      o[2] = Quant(packed_hi);
      o[3] = Quant(low_bits);
    }
  }
}

void Gr8GpuRef::Stage3() {  // gr8-stage3.frag
  const std::array<float, 6> q = SubpixelParams();
  constexpr float kUnit = 1.0f / 255.0f;
  constexpr float kDecode = 1.0f / 255.0f;
  const float tol = params_.nms_tolerance;
  auto px = [&](int x, int y) { return Nearest(dest2_, x, y); };
  auto unpack = [&](const uint8_t *p) { return ToFloat(p[2]) * 1.0f + ToFloat(p[3]) * kDecode; };
  for (int y = 0; y < tex_h_; ++y) {
    for (int x = 0; x < tex_w_; ++x) {
      const uint8_t *c33 = px(x, y);
      const float threshold = kUnit * params_.score_thresh;
      const float center = unpack(c33);
      float off_x = 0.0f, off_y = 0.0f;
      float out_r = ToFloat(c33[0]);
      float out_g, out_b;
      if (center > threshold) {
        // s[row][col] = unpacked score at (x + col - 3, y + row - 3); gcol[row] = G at column 0.
        float s[7][7];
        float gcol[7];
        for (int row = 0; row < 7; ++row) {
          for (int col = 0; col < 7; ++col) s[row][col] = unpack(px(x + col - 3, y + row - 3));
          gcol[row] = ToFloat(px(x, y + row - 3)[1]);
        }
        out_g = Dot4(kG7_0, kG7_1, kG7_2, kG7_3, gcol[0], gcol[1], gcol[2], ToFloat(c33[1])) +
                Dot3(kG7_2, kG7_1, kG7_0, gcol[4], gcol[5], gcol[6]);
        float keep = 1.0f;
        if (tol == 0.0f || tol == 1.0f) {
          bool pass = true;
          for (int row = 0; row < 7 && pass; ++row) {
            for (int col = 0; col < 7; ++col) {
              if (row == 3 && col == 3) continue;
              const bool later = (row > 3) || (row == 3 && col > 3);
              const bool ok = (tol == 1.0f && later) ? (center >= s[row][col]) : (center > s[row][col]);
              if (!ok) {
                pass = false;
                break;
              }
            }
          }
          keep = pass ? 1.0f : 0.0f;
        }
        const float final_score = params_.gain * (center * keep);
        if (final_score > 0.0f) {
          if (!params_.skip_subpixel && params_.binary_expression_forms) {
            // xr-slam-strings.txt 9120-9160 (w0 = w1 = w3 = w4 = w5 = w7 = 0 folded by the optimizer).
            const float c0 = center;
            const float zl = s[3][2], zr = s[3][4], zu = s[2][3], zd = s[4][3];
            const float zul = s[2][2], zur = s[2][4], zdl = s[4][2], zdr = s[4][4];
            const float neg_c = -c0;
            const float u00 = 0.5f * (zl + zr) + neg_c;          // _1138
            const float u01 = 0.5f * (zu + zd) + neg_c;          // _1148
            const float u02 = ((zul - zu) - zl) + c0;            // _1160
            const float dx = zr - zl;                            // _1166
            const float dy = zd - zu;                            // _1173
            const float s01 = u00 + u01;                         // _1188
            const float s012 = s01 - u02;                        // _1191
            const float w2 = zur - (((-0.5f) * dy + (0.5f * dx + s012)) + c0);   // _1201
            const float w6 = zdl - ((0.5f * dy + ((-0.5f) * dx + s012)) + c0);   // _1225
            const float w8 = zdr - ((0.5f * dy + (0.5f * dx + (s01 + u02))) + c0);  // _1247
            const float wsum = (w2 + w6) + w8;                   // _1274
            const float u0 = q[0] * wsum + u00;                  // _1276
            const float u1 = q[0] * wsum + u01;                  // _1315
            const float u2 = 0.25f * (((-w2) - w6) + w8) + u02;  // _1345
            const float u3 = 0.5f * dx + (q[3] * ((w2 - w6) + w8));  // _1372
            const float u4 = 0.5f * dy + (q[3] * ((w6 - w2) + w8));  // _1399
            const float curvature = u2 * u2 + (-((4.0f * u1) * u0));  // _1454
            if (u0 < 0.0f && u1 < 0.0f && curvature < 0.0f) {
              off_x = ((2.0f * u1) * u3 + (-(u2 * u4))) / curvature;
              off_y = ((2.0f * u0) * u4 + (-(u2 * u3))) / curvature;
            }
          } else if (!params_.skip_subpixel) {
            float z[9] = {s[2][2], s[2][3], s[2][4], s[3][2], center, s[3][4], s[4][2], s[4][3], s[4][4]};
            float u0[6];
            u0[0] = 0.5f * (z[3] + z[5]) - z[4];
            u0[1] = 0.5f * (z[1] + z[7]) - z[4];
            u0[2] = z[0] - z[1] - z[3] + z[4];
            u0[3] = 0.5f * (z[5] - z[3]);
            u0[4] = 0.5f * (z[7] - z[1]);
            u0[5] = z[4];
            float w[9] = {0.0f};
            w[2] = z[2] - (u0[0] + u0[1] - u0[2] + u0[3] - u0[4] + u0[5]);
            w[6] = z[6] - (u0[0] + u0[1] - u0[2] - u0[3] + u0[4] + u0[5]);
            w[8] = z[8] - (u0[0] + u0[1] + u0[2] + u0[3] + u0[4] + u0[5]);
            float u[6];
            u[0] = u0[0] + q[0] * (w[0] - 2.0f * w[1] + w[2] + w[6] - 2.0f * w[7] + w[8]) +
                   q[1] * (w[3] - 2.0f * w[4] + w[5]);
            u[1] = u0[1] + q[0] * (w[0] + w[2] - 2.0f * w[3] - 2.0f * w[5] + w[6] + w[8]) +
                   q[1] * (w[1] - 2.0f * w[4] + w[7]);
            u[2] = u0[2] + 0.25f * (w[0] - w[2] - w[6] + w[8]);
            u[3] = u0[3] + q[2] * (w[5] - w[3]) + q[3] * (w[2] - w[0] - w[6] + w[8]);
            u[4] = u0[4] + q[2] * (w[7] - w[1]) + q[3] * (w[6] - w[0] - w[2] + w[8]);
            u[5] = u0[5] +
                   q[4] * (2.0f * w[1] - w[0] - w[2] + 2.0f * w[3] + 2.0f * w[5] - w[6] +
                           2.0f * w[7] - w[8]) +
                   q[5] * w[4];
            const float curvature = u[2] * u[2] - 4.0f * u[1] * u[0];
            if (u[0] < 0.0f && u[1] < 0.0f && curvature < 0.0f) {
              off_x = (2.0f * u[1] * u[3] - u[2] * u[4]) / curvature;
              off_y = (2.0f * u[0] * u[4] - u[2] * u[3]) / curvature;
            }
          }
          out_b = final_score;
        } else {
          out_b = 0.0f;
        }
      } else {
        float gcol[7];
        for (int row = 0; row < 7; ++row) gcol[row] = ToFloat(px(x, y + row - 3)[1]);
        out_g = Dot4(kG7_0, kG7_1, kG7_2, kG7_3, gcol[0], gcol[1], gcol[2], ToFloat(c33[1])) +
                Dot3(kG7_2, kG7_1, kG7_0, gcol[4], gcol[5], gcol[6]);
        out_b = 0.0f;
      }
      float alpha;
      if (params_.binary_expression_forms) {
        // xr-slam-strings.txt 9165-9166.
        const float sx = GlslClamp((off_x + 0.5f) * 0.90909087657928466796875f, 0.0f, 0.9375f);
        const float sy = GlslClamp((off_y + 0.5f) * 0.90909087657928466796875f, 0.0f, 0.9375f);
        alpha = std::floor(sx * 16.0f) * 0.0039215688593685626983642578125f +
                std::floor(sy * 16.0f) * 0.062745101749897003173828125f;
      } else {
        const float sx = GlslClamp((off_x + 0.5f) / 1.1f, 0.0f, 0.9375f);
        const float sy = GlslClamp((off_y + 0.5f) / 1.1f, 0.0f, 0.9375f);
        alpha = std::floor(sx * 16.0f) / 255.0f + 16.0f * std::floor(sy * 16.0f) / 255.0f;
      }
      uint8_t *o = dest3_.At(x, y);
      o[0] = Quant(out_r);
      o[1] = Quant(out_g);
      o[2] = Quant(out_b);
      o[3] = Quant(alpha);
    }
  }
}

void Gr8GpuRef::Draw(const uint8_t *rgba, int stride_bytes) {
  g_swiftshader_approx = params_.swiftshader_approx;
  for (int y = 0; y < camera_.h; ++y) {
    std::copy(rgba + static_cast<size_t>(y) * stride_bytes,
              rgba + static_cast<size_t>(y) * stride_bytes + camera_.w * 4, camera_.At(0, y));
  }
  InputStage();
  Stage1();
  for (int i = 0; i < 3; ++i) {
    PyramidBlur(i);
    PyramidDecimate(i);
  }
  Stage2();
  Stage3();
}

void Gr8GpuRef::DrawFromCapturedGray(const uint8_t *readback, int rows, bool rebuild_levels) {
  g_swiftshader_approx = params_.swiftshader_approx;
  dest1_.Init(tex_w_, tex_h_);
  auto inside = [&](const Layout &l, int x, int y) {
    return x >= l.c && x < l.c + l.w && y >= l.r && y < l.r + l.h;
  };
  for (int y = 0; y < std::min(rows, tex_h_); ++y) {
    for (int x = 0; x < tex_w_; ++x) {
      bool keep = !rebuild_levels || inside(levels_[0], x, y);
      for (const auto &rl : roi_layouts_) keep = keep || inside(rl, x, y);
      uint8_t *o = dest1_.At(x, y);
      o[0] = keep ? readback[(static_cast<size_t>(y) * tex_w_ + x) * 4] : 0;
      o[3] = 255;
    }
  }
  if (rebuild_levels) {
    for (int i = 0; i < 3; ++i) {
      PyramidBlur(i);
      PyramidDecimate(i);
    }
  }
  Stage2();
  Stage3();
}

}  // namespace spec_frontend

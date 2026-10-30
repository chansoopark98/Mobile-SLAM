// CPU port of GlRealityFrame::drawTexture (func 496) and the Gr8 shaders (S1 §3.3, §5.1-5.5).
//
// The port produces the GPU readback format (RGBA8, row 0 = GL row 0): R level gray, G 7x7
// Gaussian, B NMES FAST score, A subpixel nibbles. Expression forms follow the ES3 GLSL embedded
// in xr-slam.wasm (build/rea-analysis-20261015/xr_reference/xr-slam-strings.txt 7973-9167, S1 §5.1).
// Starting point: the spec-lane reference slam_refact/tools/re/spec_frontend/gr8_gpu_ref.cc
// (validated by S1 H3/H4), rewritten without global state and restricted to the binary forms.
//
// Numeric models (S1 §5.1.2, D-9):
//   kIeee               texel = b / 255.0f, mix = x(1-a) + y a,  unorm write = floor(255 f + 0.5),
//                       texUv = ideal pixel centre (i + 0.5) / w
//   kSwiftShaderApprox  the arithmetic of the reference environment, Chrome 144 ANGLE + SwiftShader
//                       04fbb7da (S1 §5.1.3): texel = b * (1/255.f), mix = x + a(y - x), unorm write =
//                       nearbyint(255 f), LINEAR = lerp of the raw texel integers then one scale,
//                       texUv = SwiftShader plane-equation interpolation of the quad's two triangles
//                       (QuadVaryings). The enum keeps its historical name; the model is exact for every
//                       stage whose expressions are reproduced here.
//
// GPU texture state: every frame writes the same atlas regions (level 0, ROI slots, levels 1-3,
// then full-texture stage 2 and stage 3), and every read outside those regions hits texels that
// are never drawn (R = 0). A fresh zero texture per call is therefore equivalent to the GPU's
// persistent textures. Rows at or below the computed row count are R = 0 for the same reason
// (PyramidLayout::rows DECISION in pyramid_layout.cc).

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <vector>

#include "common/check.h"
#include "frontend/frontend.h"
#include "frontend/gr8_cpu_shader.h"

namespace slam_refact::frontend {
namespace {

// ---- numeric model ----

template <bool kApprox>
struct Num {
  static float ToFloat(uint8_t b) {
    return kApprox ? static_cast<float>(b) * (1.0f / 255.0f) : static_cast<float>(b) / 255.0f;
  }
  // unorm8 write: clamp to [0, 1], round to nearest (NaN -> 0).
  static uint8_t Quant(float f) {
    if (!(f > 0.0f)) {
      return 0;
    }
    if (f >= 1.0f) {
      return 255;
    }
    const float y = f * 255.0f;
    return static_cast<uint8_t>(kApprox ? std::nearbyint(y) : std::floor(y + 0.5f));
  }
  static float Mix(float x, float y, float a) {
    return kApprox ? x + a * (y - x) : x * (1.0f - a) + y * a;
  }
};

inline float GlslMin(float x, float y) { return y < x ? y : x; }
inline float GlslMax(float x, float y) { return x < y ? y : x; }
inline float GlslClamp(float x, float lo, float hi) { return GlslMin(GlslMax(x, lo), hi); }

// GLSL dot products, evaluated left to right.
inline float Dot3(float a0, float a1, float a2, float b0, float b1, float b2) {
  return (a0 * b0 + a1 * b1) + a2 * b2;
}
inline float Dot4(float a0, float a1, float a2, float a3, float b0, float b1, float b2, float b3) {
  return ((a0 * b0 + a1 * b1) + a2 * b2) + a3 * b3;
}

// 7x7 separable Gaussian, sigma 2 (gr8-stage2.frag:65-66, gr8-stage3.frag:264-266).
constexpr float kG7_0 = 0.070159f;
constexpr float kG7_1 = 0.131075f;
constexpr float kG7_2 = 0.190713f;
constexpr float kG7_3 = 0.216106f;

// ---- FAST-9 score (gr8-stage2.frag:17-34, literal component order) ----

struct V4 {
  float x, y, z, w;
};
inline V4 Neg(V4 a) { return {-a.x, -a.y, -a.z, -a.w}; }

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

float Max9Full(V4 a, V4 b, V4 c, V4 d) {
  const V4 m1 = Min9Quad(a, b, c);
  const V4 m2 = Min9Quad(b, c, d);
  const V4 m3 = Min9Quad(c, d, a);
  const V4 m4 = Min9Quad(d, a, b);
  auto vmax = [](V4 p, V4 q) {
    return V4{GlslMax(p.x, q.x), GlslMax(p.y, q.y), GlslMax(p.z, q.z), GlslMax(p.w, q.w)};
  };
  const V4 m = vmax(m1, vmax(m2, vmax(m3, m4)));
  return GlslMax(GlslMax(m.x, m.z), GlslMax(m.y, m.w));
}

float ScoreGr8(V4 d0, V4 d4, V4 d8, V4 d12) {
  return GlslMax(Max9Full(d0, d4, d8, d12), Max9Full(Neg(d0), Neg(d4), Neg(d8), Neg(d12)));
}

// ---- textures ----

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

// One-channel NEAREST + CLAMP_TO_EDGE texture of size tex x tex whose rows >= rows are 0.
struct GrayTex {
  int tex = 0;
  int rows = 0;
  std::vector<uint8_t> v;

  void Init(int texture_size, int computed_rows) {
    tex = texture_size;
    rows = computed_rows;
    v.assign(static_cast<size_t>(tex) * rows, 0);
  }
  uint8_t At(int x, int y) const {
    x = std::min(std::max(x, 0), tex - 1);
    y = std::min(std::max(y, 0), tex - 1);
    return y < rows ? v[static_cast<size_t>(y) * tex + x] : 0;
  }
  uint8_t* Mutable(int x, int y) { return &v[static_cast<size_t>(y) * tex + x]; }
};

// RGBA NEAREST + CLAMP_TO_EDGE view of stage-2 output; rows >= rows read as 0.
struct RgbaTex {
  int tex = 0;
  int rows = 0;
  std::vector<uint8_t> v;

  void Init(int texture_size, int computed_rows) {
    tex = texture_size;
    rows = computed_rows;
    v.assign(static_cast<size_t>(tex) * rows * 4, 0);
  }
  const uint8_t* At(int x, int y) const {
    static constexpr uint8_t kZero[4] = {0, 0, 0, 0};
    x = std::min(std::max(x, 0), tex - 1);
    y = std::min(std::max(y, 0), tex - 1);
    return y < rows ? &v[(static_cast<size_t>(y) * tex + x) * 4] : kZero;
  }
  uint8_t* Mutable(int x, int y) { return &v[(static_cast<size_t>(y) * tex + x) * 4]; }
};

// LINEAR filtered input texture (RGB, A is never read): pow2 size, zero outside the drawn area.
struct RgbTex {
  int w = 0;
  int h = 0;
  std::vector<uint8_t> v;

  void Init(int width, int height) {
    w = width;
    h = height;
    v.assign(static_cast<size_t>(w) * h * 3, 0);
  }
  const uint8_t* At(int x, int y) const {
    x = std::min(std::max(x, 0), w - 1);
    y = std::min(std::max(y, 0), h - 1);
    return &v[(static_cast<size_t>(y) * w + x) * 3];
  }
};

// Lends a kept buffer to a texture for one scope and takes it back afterwards (capacity reuse
// only; the texture's Init zero-fills it). A null store leaves the texture with its own buffer.
template <typename Tex>
class BorrowedTex {
 public:
  explicit BorrowedTex(std::vector<uint8_t>* store) : store_(store) {
    if (store_ != nullptr) {
      tex.v.swap(*store_);
    }
  }
  ~BorrowedTex() {
    if (store_ != nullptr) {
      tex.v.swap(*store_);
    }
  }
  BorrowedTex(const BorrowedTex&) = delete;
  BorrowedTex& operator=(const BorrowedTex&) = delete;

  Tex tex;

 private:
  std::vector<uint8_t>* store_;
};

// The kept texture buffers of one CpuGr8Pyramid (members of the class, see frontend.h).
struct TexStores {
  std::vector<uint8_t>* filtered = nullptr;
  std::vector<uint8_t>* dest1 = nullptr;
  std::vector<uint8_t>* blur = nullptr;
  std::vector<uint8_t>* dest2 = nullptr;
};

const uint8_t* CameraAt(const RgbaImageView& cam, int x, int y) {
  x = std::min(std::max(x, 0), cam.width - 1);
  y = std::min(std::max(y, 0), cam.height - 1);
  return cam.data + static_cast<size_t>(y) * cam.stride + static_cast<size_t>(x) * 4;
}

// texture() with LINEAR filtering: s = u w - 0.5, weights = fract (exact float weights).
//
// kSwiftShaderApprox follows SwiftShader's 32-bit filtering path, which Chromium forces for every
// non-NEAREST sampler (src/Vulkan/BUILD.gn:66-69 SWIFTSHADER_HIGH_PRECISION_FILTERING,
// VkSampler.cpp:38-40, SamplerCore.cpp:173-182):
//   coordinate  clamp(u, 0, 0x3F7FFFFF) * size - 0.5, floor, frac (SamplerCore.cpp:2489-2593)
//   texel       RGBA8 unorm unpacked to b << 8 (SamplerCore.cpp:1772-1787, 2228-2231)
//   filter      c00 + fu (c10 - c00), then the rows, on those integers (SamplerCore.cpp:1212-1225)
//   scale       one multiply by 1 / 0xFF00 after filtering (SamplerCore.cpp:200-209, 2755-2777)
// The integers are exact in float, so the result equals lerp(b) * float(1/255) with one rounding per
// operation; verified on all 40,960 1D/2D WebGL measurements (S1 §5.1.3).
template <bool kApprox, typename TexelAt>
void SampleLinear3(int tw, int th, const TexelAt& at, float u, float v, float out[3]) {
  using N = Num<kApprox>;
  // Largest float below 1 (SamplerCore.cpp:2489 oneBits 0x3F7FFFFF).
  constexpr float kOneBelow = 0.999999940395355224609375f;
  if (kApprox) {
    u = GlslMin(GlslMax(u, 0.0f), kOneBelow);
    v = GlslMin(GlslMax(v, 0.0f), kOneBelow);
  }
  const float s = u * static_cast<float>(tw) - 0.5f;
  const float q = v * static_cast<float>(th) - 0.5f;
  const float fs = std::floor(s);
  const float fq = std::floor(q);
  const float a = s - fs;
  const float b = q - fq;
  const int x0 = static_cast<int>(fs);
  const int y0 = static_cast<int>(fq);
  const uint8_t* t00 = at(x0, y0);
  const uint8_t* t10 = at(x0 + 1, y0);
  const uint8_t* t01 = at(x0, y0 + 1);
  const uint8_t* t11 = at(x0 + 1, y0 + 1);
  for (int ch = 0; ch < 3; ++ch) {
    if (kApprox) {
      constexpr float kUnpackScale = 1.0f / 65280.0f;  // 1 / 0xFF00
      const auto raw = [](uint8_t b8) { return static_cast<float>(b8 << 8); };
      const float c00 = raw(t00[ch]);
      const float c10 = raw(t10[ch]);
      const float c01 = raw(t01[ch]);
      const float c11 = raw(t11[ch]);
      const float top = c00 + a * (c10 - c00);
      const float bot = c01 + a * (c11 - c01);
      out[ch] = (top + b * (bot - top)) * kUnpackScale;
    } else {
      const float top = N::Mix(N::ToFloat(t00[ch]), N::ToFloat(t10[ch]), a);
      const float bot = N::Mix(N::ToFloat(t01[ch]), N::ToFloat(t11[ch]), a);
      out[ch] = N::Mix(top, bot, b);
    }
  }
}

// gr8-rotate.vert / gr8-rotate-only.vert uv -> texUv (S1 §5.1.1).
inline void RotateUv(int rotation, float u, float v, float* tu, float* tv) {
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

// ---- texUv varying of the reference runtime quad ----
//
// The quad (c8/pixels/opengl/gl-quad.cc:30-72 makeVertexArrayRect): positions (-1,1) (-1,-1) (1,1)
// (1,-1), uv (0,1) (0,0) (1,1) (1,0), triangles {0,1,2} {2,1,3}, drawn into glViewport(c, r, w, h).
// The vertex shaders only permute uv (gr8-rotate*.vert, S1 §5.1.1), so every vertex value is 0 or 1.
//
// kSwiftShaderApprox reproduces SwiftShader 04fbb7da as built into Chrome 144 (S1 §5.1.3):
//   viewport    X = round(16 (c + w/2) - 8 + x_ndc 16 w/2), 4 subpixel bits under
//               SWIFTSHADER_LEGACY_PRECISION (Renderer.cpp:296-310, VertexRoutine.cpp:603-609,
//               VkConfig.hpp:101)
//   assembly    provoking vertex last (ANGLE): triangle (i0,i1,i2) -> (i2,i0,i1) (Renderer.cpp:100-108)
//   setup       rotate so the first vertex has the smallest clip y (SetupRoutine.cpp:270-292,
//               623-662), edge spans (SetupRoutine.cpp:550-618), plane equations
//               (SetupRoutine.cpp:307-377, 514-547)
//   fragment    x' = x - x0, y' = y - y0, w = x' W.A + (W.C + y' W.B), rhw = 1 / w,
//               uv = (x' P.A + (P.C + y' P.B)) rhw, unfused (QuadRasterizer.cpp:126-162, 235-249,
//               PixelRoutine.cpp:127, 196-199, ShaderCore.cpp:713-722 mulAdd under LEGACY_PRECISION,
//               Reactor.cpp:4813-4838 exact 1/w on x86)
// The resulting texUv is not the ideal (i + 0.5) / w and differs per triangle; that difference is
// what moved stage-1 outputs across the k + 0.5 rounding boundary (S1 §9 U1).
class QuadVaryings {
 public:
  // The identity quad of every pyramid stage (mvp = identity, w = 1 for every vertex).
  QuadVaryings(int c, int r, int w, int h, int rotation, int framebuffer_size, bool swiftshader)
      : c_(c), r_(r), w_(w), h_(h), rotation_(rotation), swiftshader_(swiftshader) {
    if (!swiftshader_) {
      return;
    }
    constexpr float kIdentity[16] = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
    valid_ = Setup(kIdentity, framebuffer_size);
    SR_CHECK(valid_, "degenerate quad triangle");
  }

  // The same quad drawn with a general vertex transform (US015-CC-1): `mvp` = the 16 floats of the `mvp` uniform as
  // uniformMatrix4fv receives it (column-major, c8::HMatrix::data() order). The clip coordinates are float32 in the
  // vertex shader's order, ((m0 x + m4 y) + m8 0) + m12, the vertex X/Y are round(x0xf + x rhw wxf) with rhw = 1 / w, and
  // the setup sorts by the projected y, then by the largest clip w, and builds the plane equations from w1, w2 and 1/w0
  // of the UNCLIPPED triangle (SetupRoutine.cpp:270-377). A triangle with a vertex outside the frustum is first clipped in
  // clip space against x = -w, x = w, y = -w, y = w (a viewport smaller than the framebuffer is bounded by this clip, there
  // is no scissor) and only the spans come from the clipped polygon, a triangle fan of vertices rounded like the others.
  // Validated byte-exact against the original's GPU readback of every planar ROI slot of 67 frames of the E5 seq-a run
  // (HIRES_SCAN and IMAGE_TARGET, 4 slots each; slam_refact/tools/harness/e5/diag/README.md). `valid()` is false when the
  // quad is outside that domain (a vertex with w <= 0, non-finite or beyond the 2^24 sub-pixel range) or the triangles are
  // degenerate; nothing is drawn by Covers() then.
  QuadVaryings(int c, int r, int w, int h, const float mvp[16], int rotation, int framebuffer_size)
      : c_(c), r_(r), w_(w), h_(h), rotation_(rotation), swiftshader_(true) {
    valid_ = Setup(mvp, framebuffer_size);
  }

  bool valid() const { return valid_; }

  // texUv at the centre of framebuffer pixel (x, y) when the quad covers it; false otherwise (only the general quad has
  // uncovered pixels).
  bool Covers(int x, int y, float* u, float* v) const {
    for (const Tri& t : tris_) {
      if (y < t.y_min || y >= t.y_max || x < t.left[y] || x >= t.right[y]) {
        continue;
      }
      const float xf = static_cast<float>(x) - t.x0;
      const float yf = static_cast<float>(y) - t.y0;
      const float dw = t.w[2] + yf * t.w[1];
      const float rhw = 1.0f / (xf * t.w[0] + dw);
      const float du = t.pu[2] + yf * t.pu[1];
      const float dv = t.pv[2] + yf * t.pv[1];
      *u = (xf * t.pu[0] + du) * rhw;
      *v = (xf * t.pv[0] + dv) * rhw;
      return true;
    }
    return false;
  }

  // texUv (after the vertex-shader rotation) at the centre of framebuffer pixel (x, y) of the
  // viewport.
  void At(int x, int y, float* u, float* v) const {
    if (!swiftshader_) {
      const float iu = (static_cast<float>(x - c_) + 0.5f) / static_cast<float>(w_);
      const float iv = (static_cast<float>(y - r_) + 0.5f) / static_cast<float>(h_);
      RotateUv(rotation_, iu, iv, u, v);
      return;
    }
    if (Covers(x, y, u, v)) {
      return;
    }
    SR_CHECK(false, "viewport pixel covered by neither quad triangle");
  }

 private:
  static constexpr float kSubPixF = 16.0f;  // 1 << SUBPIXEL_PRECISION_BITS (legacy precision)
  static constexpr int kSubPixB = 4;
  static constexpr int kSubPixM = 15;
  static constexpr float kMaxSubPix = 16777216.0f;  // 2^24: keeps every setup product inside int32

  struct Tri {
    std::vector<int> left, right;  // span [left, right) per framebuffer row
    int y_min = 0, y_max = 0;
    float x0 = 0.0f, y0 = 0.0f;
    float pu[3] = {0.0f, 0.0f, 0.0f};  // A, B, C of texUv.x
    float pv[3] = {0.0f, 0.0f, 0.0f};
    float w[3] = {0.0f, 0.0f, 0.0f};
  };

  struct V {
    int X, Y;
    float y_proj;  // projected y (clip y * rhw), the setup's sort key
    float u, v;
    float w, rhw;  // clip w, 1 / w
    float cx, cy;  // clip x, y
  };

  // A clip-space position (x, y, w) of the clipper.
  struct ClipPoint {
    float x, y, w;
  };

  // Clip distance of `p` to the frustum side `plane` (0 left x >= -w, 1 right x <= w, 2 bottom y >= -w, 3 top y <= w).
  static float ClipDistance(const ClipPoint& p, int plane) {
    switch (plane) {
      case 0: return p.x + p.w;
      case 1: return p.w - p.x;
      case 2: return p.y + p.w;
      default: return p.w - p.y;
    }
  }

  // Clipper::clipEdge: the point of edge (vi, vj) on the plane, (dj vi - di vj) / (dj - di) in float32.
  static ClipPoint ClipEdge(const ClipPoint& vi, const ClipPoint& vj, float di, float dj) {
    const float d = 1.0f / (dj - di);
    return {(dj * vi.x - di * vj.x) * d, (dj * vi.y - di * vj.y) * d, (dj * vi.w - di * vj.w) * d};
  }

  // The polygon of `in` clipped to the four frustum sides in turn (Sutherland-Hodgman, the Clipper's plane loops).
  static std::vector<ClipPoint> ClipPolygon(std::vector<ClipPoint> in) {
    for (int plane = 0; plane < 4 && in.size() >= 3; ++plane) {
      std::vector<ClipPoint> out;
      const size_t n = in.size();
      for (size_t i = 0; i < n; ++i) {
        const ClipPoint& a = in[i];
        const ClipPoint& b = in[(i + 1) % n];
        const float da = ClipDistance(a, plane), db = ClipDistance(b, plane);
        if (da >= 0.0f) {
          out.push_back(a);
          if (db < 0.0f) out.push_back(ClipEdge(a, b, da, db));
        } else if (db > 0.0f) {
          out.push_back(ClipEdge(a, b, da, db));
        }
      }
      in = std::move(out);
    }
    return in;
  }

  static void Rasterize(const V& a, const V& b, const V& c, int fb, Tri* tri) {
    const int xs[4] = {a.X, b.X, c.X, a.X};
    const int ys[4] = {a.Y, b.Y, c.Y, a.Y};
    int y_min = std::min(std::min(ys[0], ys[1]), ys[2]);
    int y_max = std::max(std::max(ys[0], ys[1]), ys[2]);
    tri->y_min = std::max((y_min + kSubPixM) >> kSubPixB, 0);
    tri->y_max = std::min((y_max + kSubPixM) >> kSubPixB, fb);
    // Winding (SetupRoutine.cpp:76-105): d = 1 when the area is positive.
    const float fx0 = static_cast<float>(xs[0]), fx1 = static_cast<float>(xs[1]);
    const float fx2 = static_cast<float>(xs[2]);
    const float fy0 = static_cast<float>(ys[0]), fy1 = static_cast<float>(ys[1]);
    const float fy2 = static_cast<float>(ys[2]);
    const float area = ((fy0 - fy2) * fx1 + (fy2 - fy1) * fx0) + (fy1 - fy0) * fx2;
    const int d = area > 0.0f ? 1 : 0;
    for (int i = 0; i < 3; ++i) {
      Edge(xs[i + 1 - d], ys[i + 1 - d], xs[i + d], ys[i + d], fb, tri);
    }
  }

  // SetupRoutine::edge (SetupRoutine.cpp:550-618), scissor = the whole framebuffer.
  static void Edge(int xa, int ya, int xb, int yb, int fb, Tri* tri) {
    if (ya == yb) {
      return;
    }
    const bool swap = yb < ya;
    const int x1 = swap ? xb : xa, x2 = swap ? xa : xb;
    const int y1 = swap ? yb : ya, y2 = swap ? ya : yb;
    const int row1 = (y1 + kSubPixM) >> kSubPixB;
    const int row2 = (y2 + kSubPixM) >> kSubPixB;
    const int row_min = std::max(row1, 0);
    const int row_max = std::min(row2, fb);
    if (row_min >= row_max) {
      return;
    }
    std::vector<int>& edge = swap ? tri->right : tri->left;
    const int dx12 = x2 - x1, dy12 = y2 - y1;
    const int fdx12 = dx12 * (1 << kSubPixB), fdy12 = dy12 * (1 << kSubPixB);
    const int xx = dx12 * (row1 * (1 << kSubPixB) - y1) + (x1 & kSubPixM) * dy12;
    int x = (x1 >> kSubPixB) + xx / fdy12;
    int err = xx % fdy12;
    const int ceil = err > 0 ? -1 : 0;  // -d >> 31
    x -= ceil;
    err -= ceil & fdy12;
    int q = fdx12 / fdy12;
    int rem = fdx12 % fdy12;
    const int floor = rem < 0 ? -1 : 0;  // R >> 31
    q += floor;
    rem += floor & fdy12;
    for (int row = row1; row < row_max; ++row) {
      if (row >= row_min) {
        edge[static_cast<size_t>(row)] = std::min(std::max(x, 0), fb);
      }
      x += q;
      err += rem;
      const int overflow = err > 0 ? -1 : 0;  // -d >> 31
      err -= fdy12 & overflow;
      x -= overflow;
    }
  }

  // Vertex transform, viewport transform (Renderer.cpp:296-310) and the setup of both triangles. False when the quad is
  // not representable (see the general constructor).
  bool Setup(const float m[16], int fb) {
    constexpr float kPos[4][2] = {{-1.0f, 1.0f}, {-1.0f, -1.0f}, {1.0f, 1.0f}, {1.0f, -1.0f}};
    constexpr float kUv[4][2] = {{0.0f, 1.0f}, {0.0f, 0.0f}, {1.0f, 1.0f}, {1.0f, 0.0f}};
    constexpr int kIndices[2][3] = {{0, 1, 2}, {2, 1, 3}};
    const float half_w = 0.5f * static_cast<float>(w_);
    const float half_h = 0.5f * static_cast<float>(h_);
    const float x0xf = (static_cast<float>(c_) + half_w) * kSubPixF - kSubPixF / 2.0f;
    const float y0xf = (static_cast<float>(r_) + half_h) * kSubPixF - kSubPixF / 2.0f;
    const float wxf = half_w * kSubPixF;
    const float hxf = half_h * kSubPixF;
    V verts[4];
    for (int i = 0; i < 4; ++i) {
      const float px = kPos[i][0], py = kPos[i][1];
      // GLSL mat4 * vec4(position, 0, 1), unfused, column by column.
      const float cx = ((m[0] * px + m[4] * py) + m[8] * 0.0f) + m[12];
      const float cy = ((m[1] * px + m[5] * py) + m[9] * 0.0f) + m[13];
      const float cw = ((m[3] * px + m[7] * py) + m[11] * 0.0f) + m[15];
      if (!(cw > 0.0f) || !std::isfinite(cw)) {
        return false;
      }
      const float rhw = 1.0f / cw;
      const float fx = x0xf + cx * rhw * wxf;
      const float fy = y0xf + cy * rhw * hxf;
      if (!(std::fabs(fx) <= kMaxSubPix) || !(std::fabs(fy) <= kMaxSubPix)) {
        return false;
      }
      verts[i].X = static_cast<int>(std::nearbyint(fx));
      verts[i].Y = static_cast<int>(std::nearbyint(fy));
      verts[i].y_proj = cy * rhw;
      verts[i].w = cw;
      verts[i].rhw = rhw;
      verts[i].cx = cx;
      verts[i].cy = cy;
      RotateUv(rotation_, kUv[i][0], kUv[i][1], &verts[i].u, &verts[i].v);
    }
    for (int t = 0; t < 2; ++t) {
      // Provoking vertex last.
      const V* v0 = &verts[kIndices[t][2]];
      const V* v1 = &verts[kIndices[t][0]];
      const V* v2 = &verts[kIndices[t][1]];
      const std::vector<ClipPoint> polygon = {{v0->cx, v0->cy, v0->w}, {v1->cx, v1->cy, v1->w}, {v2->cx, v2->cy, v2->w}};
      // Sort by minimum y with the values read before the rotations (SetupRoutine.cpp:270-292), then by maximum w
      // (the identity when every w is equal).
      const float y0 = v0->y_proj, y1 = v1->y_proj, y2 = v2->y_proj;
      const float y_min = std::min(std::min(y0, y1), y2);
      if (y_min == y1) {  // conditionalRotate1
        const V* vx = v0;
        v0 = v1;
        v1 = v2;
        v2 = vx;
      }
      if (y_min == y2) {  // conditionalRotate2
        const V* vx = v2;
        v2 = v1;
        v1 = v0;
        v0 = vx;
      }
      const float w_max = std::max(std::max(v0->w, v1->w), v2->w);
      const float wa = v1->w, wb = v2->w;
      if (w_max == wa) {
        const V* vx = v0;
        v0 = v1;
        v1 = v2;
        v2 = vx;
      }
      if (w_max == wb) {
        const V* vx = v2;
        v2 = v1;
        v1 = v0;
        v0 = vx;
      }
      // The plane equations come from the unclipped triangle; the clipped polygon (a triangle fan) only bounds the spans.
      Tri planes;
      if (!SetupPlanes(*v0, *v1, *v2, &planes)) {
        return false;
      }
      const std::vector<ClipPoint> clipped = ClipPolygon(polygon);
      std::vector<V> cv(clipped.size());
      for (size_t k = 0; k < clipped.size(); ++k) {
        const float rhw = 1.0f / clipped[k].w;
        const float fx = x0xf + clipped[k].x * rhw * wxf;
        const float fy = y0xf + clipped[k].y * rhw * hxf;
        if (!(std::fabs(fx) <= kMaxSubPix) || !(std::fabs(fy) <= kMaxSubPix)) {
          return false;
        }
        cv[k].X = static_cast<int>(std::nearbyint(fx));
        cv[k].Y = static_cast<int>(std::nearbyint(fy));
      }
      for (size_t k = 1; k + 1 < cv.size(); ++k) {
        Tri tri = planes;
        tri.left.assign(static_cast<size_t>(fb) + 1, 0);
        tri.right.assign(static_cast<size_t>(fb) + 1, 0);
        Rasterize(cv[0], cv[k], cv[k + 1], fb, &tri);
        tris_.push_back(std::move(tri));
      }
    }
    return true;
  }

  // SetupRoutine.cpp:307-377 and setupGradient 514-547. False for a degenerate triangle.
  static bool SetupPlanes(const V& v0, const V& v1, const V& v2, Tri* tri) {
    constexpr float kInv = 1.0f / kSubPixF;
    tri->x0 = static_cast<float>(v0.X) * kInv;
    tri->y0 = static_cast<float>(v0.Y) * kInv;
    const float w1 = v1.w, w2 = v2.w, rhw0 = v0.rhw;
    const float x1 = w1 * kInv * static_cast<float>(v1.X - v0.X);
    const float y1 = w1 * kInv * static_cast<float>(v1.Y - v0.Y);
    const float x2 = w2 * kInv * static_cast<float>(v2.X - v0.X);
    const float y2 = w2 * kInv * static_cast<float>(v2.Y - v0.Y);
    const float a = x1 * y2 - x2 * y1;
    if (!(a != 0.0f) || !std::isfinite(a)) {
      return false;
    }
    const float inv_a = 1.0f / a;
    const float dd = inv_a * rhw0;
    const float m0[3] = {(y1 * w2 - y2 * w1) * dd, (x2 * w1 - x1 * w2) * dd, rhw0};
    const float m1[3] = {y2 * inv_a, -x2 * inv_a, 0.0f};
    const float m2[3] = {-y1 * inv_a, x1 * inv_a, 0.0f};
    for (int k = 0; k < 3; ++k) {
      tri->w[k] = (m0[k] + m1[k]) + m2[k];
      tri->pu[k] = (v0.u * m0[k] + v1.u * m1[k]) + v2.u * m2[k];
      tri->pv[k] = (v0.v * m0[k] + v1.v * m1[k]) + v2.v * m2[k];
    }
    return true;
  }

  int c_, r_, w_, h_, rotation_;
  bool swiftshader_;
  bool valid_ = false;
  std::vector<Tri> tris_;  // the fan triangles of the clipped quad triangles, in draw order
};

constexpr int kTaps[9][2] = {{-1, -1}, {0, -1}, {1, -1}, {-1, 0}, {0, 0},
                             {1, 0},   {-1, 1}, {0, 1},  {1, 1}};

// ---- stages ----

// Input stage (gl-reality-frame.cc:311-334, gr8-input.frag): 3x3 blur sigma 0.75 of the LINEAR
// camera texture into the filtered input viewport.
template <bool kApprox>
void InputStage(const PyramidLayout& layout, const RgbaImageView& cam, RgbTex* filtered) {
  using N = Num<kApprox>;
  const int input_w = layout.rotate_input ? cam.height : cam.width;
  const int input_h = layout.rotate_input ? cam.width : cam.height;
  const float sx = static_cast<float>(1.0 / input_w);
  const float sy = static_cast<float>(1.0 / input_h);
  constexpr float p00 = 0.268958f, p11 = 0.124827f, p22 = 0.057934f;
  constexpr float kWeights[9] = {p22, p11, p22, p11, p00, p11, p22, p11, p22};
  const auto at = [&cam](int x, int y) { return CameraAt(cam, x, y); };
  // gr8-noop.vert, glViewport(0, 0, filtered_w, filtered_h) into the pow2 filtered texture.
  const QuadVaryings uv(0, 0, layout.filtered_w, layout.filtered_h, 0, filtered->w, kApprox);
  for (int y = 0; y < layout.filtered_h; ++y) {
    for (int x = 0; x < layout.filtered_w; ++x) {
      float u, v;
      uv.At(x, y, &u, &v);
      float acc[3] = {0.0f, 0.0f, 0.0f};
      for (int k = 0; k < 9; ++k) {
        float s[3];
        SampleLinear3<kApprox>(cam.width, cam.height, at, sx * kTaps[k][0] + u,
                               sy * kTaps[k][1] + v, s);
        for (int ch = 0; ch < 3; ++ch) {
          acc[ch] = acc[ch] + kWeights[k] * s[ch];
        }
      }
      uint8_t* o = &filtered->v[(static_cast<size_t>(y) * filtered->w + x) * 3];
      o[0] = N::Quant(acc[0]);
      o[1] = N::Quant(acc[1]);
      o[2] = N::Quant(acc[2]);
    }
  }
}

// Stage 1 (gl-reality-frame.cc:336-458 drawInitialStage, gr8-rotate.vert, gr8-stage1.frag):
// level-0 gray; every ROI slot is cleared (world SLAM requests no ROI, S1 §2.3).
template <bool kApprox>
void Stage1(const PyramidLayout& layout, int rotation, const RgbTex& filtered, GrayTex* dest1) {
  using N = Num<kApprox>;
  const LevelLayout& l = layout.levels[0];
  const int shader_rotation = l.rotated ? rotation - 90 : rotation;
  const float scale_x = static_cast<float>(1.0 / filtered.w);
  const float scale_y = static_cast<float>(1.0 / filtered.h);
  const float pix_x = static_cast<float>(layout.filtered_w);
  const float pix_y = static_cast<float>(layout.filtered_h);
  const auto at = [&filtered](int x, int y) { return filtered.At(x, y); };
  // gr8-rotate.vert with mvp = identity and flipY = 0 (GlRealityFrame default).
  const QuadVaryings uv(l.c, l.r, l.w, l.h, shader_rotation, dest1->tex, kApprox);
  for (int y = 0; y < l.h; ++y) {
    for (int x = 0; x < l.w; ++x) {
      float tu, tv;
      uv.At(l.c + x, l.r + y, &tu, &tv);
      float s[3];
      SampleLinear3<kApprox>(filtered.w, filtered.h, at, scale_x * (0.0f + pix_x * tu),
                             scale_y * (0.0f + pix_y * tv), s);
      const float gray = Dot3(0.299f, 0.587f, 0.114f, s[0], s[1], s[2]);
      if (l.r + y < dest1->rows) {
        *dest1->Mutable(l.c + x, l.r + y) = N::Quant(gray);
      }
    }
  }
  for (const LevelLayout& rl : layout.roi_slots) {
    for (int y = 0; y < rl.h && rl.r + y < dest1->rows; ++y) {
      for (int x = 0; x < rl.w; ++x) {
        *dest1->Mutable(rl.c + x, rl.r + y) = 0;
      }
    }
  }
}

// Pyramid blur of level i (gl-reality-frame.cc:253-276, gr8-pyramid-blur.frag), binary
// reassociated sum (xr-slam-strings.txt 8019).
template <bool kApprox>
void PyramidBlur(const PyramidLayout& layout, int level, const GrayTex& dest1, GrayTex* blur) {
  using N = Num<kApprox>;
  const LevelLayout& l = layout.levels[level];
  const float lw = static_cast<float>(l.w), lh = static_cast<float>(l.h);
  const float lc = static_cast<float>(l.c), lr = static_cast<float>(l.r);
  constexpr float kW[9] = {0.054659f, 0.124474f, 0.054659f, 0.124474f, 0.283464f,
                           0.124474f, 0.054659f, 0.124474f, 0.054659f};
  const QuadVaryings uv(l.c, l.r, l.w, l.h, 0, blur->tex, kApprox);  // gr8-noop.vert
  for (int y = 0; y < l.h; ++y) {
    for (int x = 0; x < l.w; ++x) {
      float u, v;
      uv.At(l.c + x, l.r + y, &u, &v);
      float t[9];
      for (int k = 0; k < 9; ++k) {
        const float px = (lw * u + lc) + static_cast<float>(kTaps[k][0]);
        const float py = (lh * v + lr) + static_cast<float>(kTaps[k][1]);
        t[k] = N::ToFloat(
            dest1.At(static_cast<int>(std::floor(px)), static_cast<int>(std::floor(py))));
      }
      float b = kW[0] * t[0] + kW[1] * t[1];
      b = kW[2] * t[2] + b;
      b = kW[3] * t[3] + b;
      b = kW[4] * t[4] + b;
      b = kW[5] * t[5] + b;
      b = kW[6] * t[6] + b;
      b = kW[7] * t[7] + b;
      b = kW[8] * t[8] + b;
      if (l.r + y < blur->rows) {
        *blur->Mutable(l.c + x, l.r + y) = N::Quant(b);
      }
    }
  }
}

// Decimate level i -> i+1 (gl-reality-frame.cc:278-310, gr8-pyramid-decimate.frag): manual
// bilinear on integer texel corners, with the level rotation difference.
template <bool kApprox>
void PyramidDecimate(const PyramidLayout& layout, int rotation, int level, const GrayTex& blur,
                     GrayTex* dest1) {
  using N = Num<kApprox>;
  const LevelLayout& ol = layout.levels[level];
  const LevelLayout& l = layout.levels[level + 1];
  const int o_rot = ol.rotated ? rotation - 90 : rotation;
  const int n_rot = l.rotated ? rotation - 90 : rotation;
  const int rotation_diff = n_rot - o_rot;
  const float ow = static_cast<float>(ol.w), oh = static_cast<float>(ol.h);
  const float oc = static_cast<float>(ol.c), orr = static_cast<float>(ol.r);
  // gr8-rotate-only.vert with the level rotation difference.
  const QuadVaryings uv(l.c, l.r, l.w, l.h, rotation_diff, dest1->tex, kApprox);
  for (int y = 0; y < l.h; ++y) {
    for (int x = 0; x < l.w; ++x) {
      float tu, tv;
      uv.At(l.c + x, l.r + y, &tu, &tv);
      const float pix_x = ow * tu + oc;
      const float pix_y = oh * tv + orr;
      const float lt_x = std::floor(pix_x);
      const float lt_y = std::floor(pix_y);
      const float fx = pix_x - lt_x;
      const float fy = pix_y - lt_y;
      const int li = static_cast<int>(lt_x), ti = static_cast<int>(lt_y);
      const float v00 = N::ToFloat(blur.At(li, ti));
      const float v10 = N::ToFloat(blur.At(li + 1, ti));
      const float v01 = N::ToFloat(blur.At(li, ti + 1));
      const float v11 = N::ToFloat(blur.At(li + 1, ti + 1));
      const float val = N::Mix(N::Mix(v00, v10, fx), N::Mix(v01, v11, fx), fy);
      if (l.r + y < dest1->rows) {
        *dest1->Mutable(l.c + x, l.r + y) = N::Quant(val);
      }
    }
  }
}

// Stage 2 (gr8-stage2.frag, C8_USE_ALTERNATE_FEATURE_SCORE = 0): R copy, horizontal 1x7 Gaussian,
// FAST-9 score packed into 16 bits (B high, A low), binary form xr-slam-strings.txt 8674-8678.
template <bool kApprox>
void Stage2(const GrayTex& dest1, RgbaTex* dest2) {
  using N = Num<kApprox>;
  const auto r = [&dest1](int x, int y) { return N::ToFloat(dest1.At(x, y)); };
  constexpr float kUnit = 1.0f / 255.0f;
  for (int y = 0; y < dest2->rows; ++y) {
    for (int x = 0; x < dest2->tex; ++x) {
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
      const float fl = std::floor(255.0f * clmp);
      const float low_bits = 255.0f * clmp + (-fl);
      const float packed_hi = (-kUnit) * low_bits + clmp;
      uint8_t* o = dest2->Mutable(x, y);
      o[0] = N::Quant(c);
      o[1] = N::Quant(g);
      o[2] = N::Quant(packed_hi);
      o[3] = N::Quant(low_bits);
    }
  }
}

// Stage 3 (gr8-stage3.frag): vertical 7x1 Gaussian, threshold, NMES, subpixel (binary simplified
// form, xr-slam-strings.txt 9120-9166; S1 §5.3-5.4).
template <bool kApprox>
void Stage3(const ShaderParams& params, const RgbaTex& dest2, uint8_t* out, int out_rows) {
  using N = Num<kApprox>;
  const std::array<float, 6> q = SubpixelParams();
  constexpr float kUnit = 1.0f / 255.0f;
  constexpr float kDecode = 1.0f / 255.0f;  // dot(vec2(1, 0.0039215688), zw)
  const float tol = params.nms_tolerance;
  const float threshold = kUnit * params.score_thresh;
  const auto unpack = [](const uint8_t* p) {
    return N::ToFloat(p[2]) * 1.0f + N::ToFloat(p[3]) * kDecode;
  };
  const int tex = dest2.tex;
  for (int y = 0; y < out_rows; ++y) {
    for (int x = 0; x < tex; ++x) {
      const uint8_t* c33 = dest2.At(x, y);
      const float center = unpack(c33);
      float off_x = 0.0f, off_y = 0.0f;
      const float out_r = N::ToFloat(c33[0]);
      float gcol[7];
      for (int row = 0; row < 7; ++row) {
        gcol[row] = N::ToFloat(dest2.At(x, y + row - 3)[1]);
      }
      const float out_g = Dot4(kG7_0, kG7_1, kG7_2, kG7_3, gcol[0], gcol[1], gcol[2],
                               N::ToFloat(c33[1])) +
                          Dot3(kG7_2, kG7_1, kG7_0, gcol[4], gcol[5], gcol[6]);
      float out_b = 0.0f;
      if (center > threshold) {
        // s[row][col] = unpacked score at (x + col - 3, y + row - 3).
        float s[7][7];
        for (int row = 0; row < 7; ++row) {
          for (int col = 0; col < 7; ++col) {
            s[row][col] = unpack(dest2.At(x + col - 3, y + row - 3));
          }
        }
        float keep = 1.0f;
        if (tol == 0.0f || tol == 1.0f) {
          bool pass = true;
          for (int row = 0; row < 7 && pass; ++row) {
            for (int col = 0; col < 7; ++col) {
              if (row == 3 && col == 3) {
                continue;
              }
              const bool later = (row > 3) || (row == 3 && col > 3);
              const bool ok =
                  (tol == 1.0f && later) ? (center >= s[row][col]) : (center > s[row][col]);
              if (!ok) {
                pass = false;
                break;
              }
            }
          }
          keep = pass ? 1.0f : 0.0f;
        }
        const float final_score = params.gain * (center * keep);
        if (final_score > 0.0f) {
          if (!params.skip_subpixel) {
            const float c0 = center;
            const float zl = s[3][2], zr = s[3][4], zu = s[2][3], zd = s[4][3];
            const float zul = s[2][2], zur = s[2][4], zdl = s[4][2], zdr = s[4][4];
            const float neg_c = -c0;
            const float u00 = 0.5f * (zl + zr) + neg_c;
            const float u01 = 0.5f * (zu + zd) + neg_c;
            const float u02 = ((zul - zu) - zl) + c0;
            const float dx = zr - zl;
            const float dy = zd - zu;
            const float s01 = u00 + u01;
            const float s012 = s01 - u02;
            const float w2 = zur - (((-0.5f) * dy + (0.5f * dx + s012)) + c0);
            const float w6 = zdl - ((0.5f * dy + ((-0.5f) * dx + s012)) + c0);
            const float w8 = zdr - ((0.5f * dy + (0.5f * dx + (s01 + u02))) + c0);
            const float wsum = (w2 + w6) + w8;
            const float u0 = q[0] * wsum + u00;
            const float u1 = q[0] * wsum + u01;
            const float u2 = 0.25f * (((-w2) - w6) + w8) + u02;
            const float u3 = 0.5f * dx + (q[3] * ((w2 - w6) + w8));
            const float u4 = 0.5f * dy + (q[3] * ((w6 - w2) + w8));
            const float curvature = u2 * u2 + (-((4.0f * u1) * u0));
            if (u0 < 0.0f && u1 < 0.0f && curvature < 0.0f) {
              off_x = ((2.0f * u1) * u3 + (-(u2 * u4))) / curvature;
              off_y = ((2.0f * u0) * u4 + (-(u2 * u3))) / curvature;
            }
          }
          out_b = final_score;
        }
      }
      // xr-slam-strings.txt 9165-9166.
      const float ex = GlslClamp((off_x + 0.5f) * 0.90909087657928466796875f, 0.0f, 0.9375f);
      const float ey = GlslClamp((off_y + 0.5f) * 0.90909087657928466796875f, 0.0f, 0.9375f);
      const float alpha = std::floor(ex * 16.0f) * 0.0039215688593685626983642578125f +
                          std::floor(ey * 16.0f) * 0.062745101749897003173828125f;
      uint8_t* o = out + (static_cast<size_t>(y) * tex + x) * 4;
      o[0] = N::Quant(out_r);
      o[1] = N::Quant(out_g);
      o[2] = N::Quant(out_b);
      o[3] = N::Quant(alpha);
    }
  }
}

template <bool kApprox>
void Stage2And3(const ShaderParams& params, const GrayTex& dest1, const TexStores& stores,
                RgbaAtlas* out) {
  BorrowedTex<RgbaTex> borrowed(stores.dest2);
  RgbaTex& dest2 = borrowed.tex;
  dest2.Init(dest1.tex, dest1.rows);
  Stage2<kApprox>(dest1, &dest2);
  out->size = dest1.tex;
  out->rows = dest1.rows;
  out->rgba.assign(static_cast<size_t>(out->size) * out->rows * 4, 0);
  Stage3<kApprox>(params, dest2, out->rgba.data(), out->rows);
}

template <bool kApprox>
void BlurDecimateLevels(const PyramidLayout& layout, int rotation, const TexStores& stores,
                        GrayTex* dest1) {
  BorrowedTex<GrayTex> borrowed(stores.blur);
  GrayTex& blur = borrowed.tex;
  blur.Init(dest1->tex, dest1->rows);
  for (int i = 0; i < 3; ++i) {
    PyramidBlur<kApprox>(layout, i, *dest1, &blur);
    PyramidDecimate<kApprox>(layout, rotation, i, blur, dest1);
  }
}

template <bool kApprox>
void BuildImpl(const PyramidLayout& layout, const ShaderParams& params, const RgbaImageView& cam,
               int rotation, const TexStores& stores, RgbaAtlas* out) {
  BorrowedTex<RgbTex> borrowed_filtered(stores.filtered);
  RgbTex& filtered = borrowed_filtered.tex;
  filtered.Init(RoundToPowerOfTwo(layout.filtered_w), RoundToPowerOfTwo(layout.filtered_h));
  InputStage<kApprox>(layout, cam, &filtered);
  BorrowedTex<GrayTex> borrowed_dest1(stores.dest1);
  GrayTex& dest1 = borrowed_dest1.tex;
  dest1.Init(layout.texture_size, layout.rows);
  Stage1<kApprox>(layout, rotation, filtered, &dest1);
  BlurDecimateLevels<kApprox>(layout, rotation, stores, &dest1);
  Stage2And3<kApprox>(params, dest1, stores, out);
}

bool Inside(const LevelLayout& l, int x, int y) {
  return x >= l.c && x < l.c + l.w && y >= l.r && y < l.r + l.h;
}

// R channel of a readback atlas into a gray texture; keep_all = false keeps level 0 and the ROI
// slots only (the H4 input), as the spec-lane isolation tool does.
GrayTex GrayFromAtlas(const PyramidLayout& layout, const RgbaAtlas& atlas, bool keep_all) {
  GrayTex dest1;
  dest1.Init(atlas.size, atlas.rows);
  for (int y = 0; y < atlas.rows; ++y) {
    for (int x = 0; x < atlas.size; ++x) {
      bool keep = keep_all || Inside(layout.levels[0], x, y);
      for (const LevelLayout& rl : layout.roi_slots) {
        keep = keep || Inside(rl, x, y);
      }
      *dest1.Mutable(x, y) =
          keep ? atlas.rgba[(static_cast<size_t>(y) * atlas.size + x) * 4] : 0;
    }
  }
  return dest1;
}

void CheckAtlas(const PyramidLayout& layout, const RgbaAtlas& atlas) {
  SR_CHECK(atlas.size == layout.texture_size, "atlas size differs from the pyramid layout");
  SR_CHECK(atlas.rows > 0 && atlas.rows <= atlas.size, "atlas rows out of range");
  SR_CHECK(atlas.rgba.size() == static_cast<size_t>(atlas.size) * atlas.rows * 4,
           "atlas byte size differs from size x rows x 4");
}

}  // namespace

float ShaderFast9Score(const float d[16]) {
  return ScoreGr8({d[0], d[1], d[2], d[3]}, {d[4], d[5], d[6], d[7]}, {d[8], d[9], d[10], d[11]},
                  {d[12], d[13], d[14], d[15]});
}

std::array<float, 6> SubpixelParams() {
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
          static_cast<float>(p1 / k6),       static_cast<float>(p2 / k6),
          static_cast<float>(k2 / k5),       static_cast<float>((k4 + 2.0 * k3 - 4.0 * k2) / k5)};
}

std::vector<float> SwiftShaderQuadTexUv(int c, int r, int w, int h, int rotation,
                                        int framebuffer_size) {
  SR_CHECK(w > 0 && h > 0 && c >= 0 && r >= 0 && c + w <= framebuffer_size &&
               r + h <= framebuffer_size,
           "viewport outside the framebuffer");
  const QuadVaryings uv(c, r, w, h, rotation, framebuffer_size, /*swiftshader=*/true);
  std::vector<float> out(static_cast<size_t>(w) * h * 2);
  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w; ++x) {
      float* o = &out[(static_cast<size_t>(y) * w + x) * 2];
      uv.At(c + x, r + y, &o[0], &o[1]);
    }
  }
  return out;
}

bool SwiftShaderMvpQuadTexUv(int c, int r, int w, int h, const float mvp[16], int rotation, int framebuffer_size,
                             std::vector<float>* uv, std::vector<uint8_t>* covered) {
  SR_CHECK(w > 0 && h > 0 && c >= 0 && r >= 0 && c + w <= framebuffer_size && r + h <= framebuffer_size,
           "viewport outside the framebuffer");
  const QuadVaryings quad(c, r, w, h, mvp, rotation, framebuffer_size);
  if (!quad.valid()) {
    return false;
  }
  uv->assign(static_cast<size_t>(w) * h * 2, 0.0f);
  covered->assign(static_cast<size_t>(w) * h, 0);
  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w; ++x) {
      const size_t i = static_cast<size_t>(y) * w + x;
      (*covered)[i] = quad.Covers(c + x, r + y, &(*uv)[2 * i], &(*uv)[2 * i + 1]) ? 1 : 0;
    }
  }
  return true;
}

uint8_t ShaderStage1CameraGray(const RgbaImageView& camera, float u, float v,
                               ShaderNumericModel model) {
  const auto at = [&camera](int x, int y) { return CameraAt(camera, x, y); };
  float s[3];
  if (model == ShaderNumericModel::kSwiftShaderApprox) {
    SampleLinear3<true>(camera.width, camera.height, at, u, v, s);
    return Num<true>::Quant(Dot3(0.299f, 0.587f, 0.114f, s[0], s[1], s[2]));
  }
  SampleLinear3<false>(camera.width, camera.height, at, u, v, s);
  return Num<false>::Quant(Dot3(0.299f, 0.587f, 0.114f, s[0], s[1], s[2]));
}

CpuGr8Pyramid::CpuGr8Pyramid(const PyramidLayout& layout, ShaderParams params,
                             ShaderNumericModel model)
    : layout_(layout), params_(params), model_(model) {
  SR_CHECK(layout_.texture_size > 0 && layout_.rows > 0, "pyramid layout is not initialised");
}

void CpuGr8Pyramid::Build(const RgbaImageView& camera, int rotation_deg, RgbaAtlas* out) const {
  SR_CHECK(out != nullptr, "null output atlas");
  SR_CHECK(camera.data != nullptr && camera.width > 0 && camera.height > 0,
           "camera image is empty");
  SR_CHECK(camera.stride >= 4 * camera.width, "camera stride smaller than 4 * width");
  SR_CHECK((camera.width > camera.height) == layout_.rotate_input,
           "camera orientation differs from the pyramid layout");
  const TexStores stores{&filtered_store_, &dest1_store_, &blur_store_, &dest2_store_};
  if (model_ == ShaderNumericModel::kSwiftShaderApprox) {
    BuildImpl<true>(layout_, params_, camera, rotation_deg, stores, out);
  } else {
    BuildImpl<false>(layout_, params_, camera, rotation_deg, stores, out);
  }
}

void CpuGr8Pyramid::RunStage2And3(RgbaAtlas* atlas_with_r) const {
  SR_CHECK(atlas_with_r != nullptr, "null atlas");
  CheckAtlas(layout_, *atlas_with_r);
  const GrayTex dest1 = GrayFromAtlas(layout_, *atlas_with_r, /*keep_all=*/true);
  const TexStores stores{nullptr, nullptr, &blur_store_, &dest2_store_};
  if (model_ == ShaderNumericModel::kSwiftShaderApprox) {
    Stage2And3<true>(params_, dest1, stores, atlas_with_r);
  } else {
    Stage2And3<false>(params_, dest1, stores, atlas_with_r);
  }
}

void CpuGr8Pyramid::RunFromLevel0(RgbaAtlas* atlas_with_l0) const {
  SR_CHECK(atlas_with_l0 != nullptr, "null atlas");
  CheckAtlas(layout_, *atlas_with_l0);
  // The decimate rotation difference does not depend on the absolute rotation (portrait 0).
  constexpr int kRotation = 0;
  GrayTex dest1 = GrayFromAtlas(layout_, *atlas_with_l0, /*keep_all=*/false);
  const TexStores stores{nullptr, nullptr, &blur_store_, &dest2_store_};
  if (model_ == ShaderNumericModel::kSwiftShaderApprox) {
    BlurDecimateLevels<true>(layout_, kRotation, stores, &dest1);
    Stage2And3<true>(params_, dest1, stores, atlas_with_l0);
  } else {
    BlurDecimateLevels<false>(layout_, kRotation, stores, &dest1);
    Stage2And3<false>(params_, dest1, stores, atlas_with_l0);
  }
}

}  // namespace slam_refact::frontend

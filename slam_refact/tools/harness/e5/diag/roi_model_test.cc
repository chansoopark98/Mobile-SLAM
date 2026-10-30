// Harness-only experiment (pixel-level ROI diagnosis): model candidates for the GPU (SwiftShader) draw of one ROI slot, compared byte-wise with the R plane
// the ORIGINAL read back.  Not product code.  usage: roi_model_test <variant> <camera.pgm> <case.txt>
//   case.txt line: c r w h m0..m15 (the `mvp` uniform as uniformMatrix4fv received it, column-major) <raw R plane of the original slot (w*h bytes)>
//   variants: engine = the engine's current DrawPlanarRoi (double inverse homography); ss = SwiftShader model of the identity quad (QuadVaryings of
//   gr8_cpu_shader.cc) generalised to the vertices mvp * (+-1, +-1, 0, 1), plane equations from the UNCLIPPED vertices, spans cut to the slot rectangle.
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iterator>
#include <sstream>
#include <string>
#include <vector>

#include "frontend/gr8_cpu_shader.h"

using slam_refact::RgbaImageView;
using slam_refact::ShaderNumericModel;
using slam_refact::frontend::ShaderStage1CameraGray;

namespace {
int LB = 0, RB = 0, EDGEVAR = 0, VX = 0, CV = 0, RV = 0, RM = 0, PV = -1, PDX = 0, PDY = 0;
constexpr float kSub = 16.0f;
struct V { int X, Y; float u, v; float yclip; float w, rhw; };
struct Tri { std::vector<int> left, right; int ymin = 0, ymax = 0; float x0 = 0, y0 = 0, pu[3] = {}, pv[3] = {}, w[3] = {}; };

void Edge(int xa, int ya, int xb, int yb, int fb, Tri* tri) {
  if (ya == yb) return;
  const bool swap = yb < ya;
  const int bias = swap ? RB : LB;
  const int x1 = (swap ? xb : xa) + bias, x2 = (swap ? xa : xb) + bias, y1 = swap ? yb : ya, y2 = swap ? ya : yb;
  const int row1 = (y1 + 15) >> 4, row2 = (y2 + 15) >> 4;
  const int row_min = std::max(row1, 0), row_max = std::min(row2, fb);
  if (row_min >= row_max) return;
  std::vector<int>& edge = swap ? tri->right : tri->left;
  const int dx12 = x2 - x1, dy12 = y2 - y1;
  const int fdx12 = dx12 * 16, fdy12 = dy12 * 16;
  const int xx = dx12 * (row1 * 16 - y1 + (EDGEVAR == 3 ? 8 : 0)) + (x1 & 15) * dy12;
  int x = (x1 >> 4) + xx / fdy12;
  int err = xx % fdy12;
  const int ceil = EDGEVAR == 2 ? 0 : (EDGEVAR == 1 ? (err >= 0 ? -1 : 0) : (err > 0 ? -1 : 0));
  x -= ceil; err -= ceil & fdy12;
  int q = fdx12 / fdy12, rem = fdx12 % fdy12;
  const int fl = rem < 0 ? -1 : 0;
  q += fl; rem += fl & fdy12;
  for (int row = row1; row < row_max; ++row) {
    if (row >= row_min) edge[static_cast<size_t>(row)] = std::min(std::max(x, 0), fb);
    x += q; err += rem;
    const int ov = err > 0 ? -1 : 0;
    err -= fdy12 & ov; x -= ov;
  }
}

void Rasterize(const V& a, const V& b, const V& c, int fb, Tri* t) {
  const int xs[4] = {a.X, b.X, c.X, a.X}, ys[4] = {a.Y, b.Y, c.Y, a.Y};
  const int ymin = std::min(std::min(ys[0], ys[1]), ys[2]), ymax = std::max(std::max(ys[0], ys[1]), ys[2]);
  t->ymin = std::max((ymin + 15) >> 4, 0);
  t->ymax = std::min((ymax + 15) >> 4, fb);
  const float fx0 = static_cast<float>(xs[0]), fx1 = static_cast<float>(xs[1]), fx2 = static_cast<float>(xs[2]);
  const float fy0 = static_cast<float>(ys[0]), fy1 = static_cast<float>(ys[1]), fy2 = static_cast<float>(ys[2]);
  const float area = ((fy0 - fy2) * fx1 + (fy2 - fy1) * fx0) + (fy1 - fy0) * fx2;
  const int d = area > 0.0f ? 1 : 0;
  for (int i = 0; i < 3; ++i) Edge(xs[i + 1 - d], ys[i + 1 - d], xs[i + d], ys[i + d], fb, t);
}

void SetupPlanes(const V& v0, const V& v1, const V& v2, Tri* t, bool general) {
  constexpr float kInv = 1.0f / kSub;
  t->x0 = static_cast<float>(v0.X) * kInv;
  t->y0 = static_cast<float>(v0.Y) * kInv;
  const float w1 = general ? v1.w : 1.0f, w2 = general ? v2.w : 1.0f, rhw0 = general ? v0.rhw : 1.0f;
  const float x1 = w1 * kInv * static_cast<float>(v1.X - v0.X), y1 = w1 * kInv * static_cast<float>(v1.Y - v0.Y);
  const float x2 = w2 * kInv * static_cast<float>(v2.X - v0.X), y2 = w2 * kInv * static_cast<float>(v2.Y - v0.Y);
  const float a = x1 * y2 - x2 * y1;
  const float inv_a = 1.0f / a, dd = inv_a * rhw0;
  const float m0[3] = {(y1 * w2 - y2 * w1) * dd, (x2 * w1 - x1 * w2) * dd, rhw0};
  const float m1[3] = {y2 * inv_a, -x2 * inv_a, 0.0f};
  const float m2[3] = {-y1 * inv_a, x1 * inv_a, 0.0f};
  for (int k = 0; k < 3; ++k) {
    t->w[k] = (m0[k] + m1[k]) + m2[k];
    t->pu[k] = (v0.u * m0[k] + v1.u * m1[k]) + v2.u * m2[k];
    t->pv[k] = (v0.v * m0[k] + v1.v * m1[k]) + v2.v * m2[k];
  }
}
}  // namespace

int main(int argc, char** argv) {
  const std::string variant = argv[1];
  if (const char* e = std::getenv("EDGEVAR")) EDGEVAR = std::atoi(e);
  if (const char* e = std::getenv("VX")) VX = std::atoi(e);
  if (const char* e = std::getenv("CV")) CV = std::atoi(e);
  if (const char* e = std::getenv("RV")) RV = std::atoi(e);
  if (const char* e = std::getenv("LB")) LB = std::atoi(e);
  if (const char* e = std::getenv("RB")) RB = std::atoi(e);
  if (const char* e = std::getenv("RM")) RM = std::atoi(e);
  if (const char* e = std::getenv("PV")) PV = std::atoi(e);
  if (const char* e = std::getenv("PDX")) PDX = std::atoi(e);
  if (const char* e = std::getenv("PDY")) PDY = std::atoi(e);
  std::ifstream pf(argv[2], std::ios::binary);
  std::string magic; int cw, ch, mx;
  pf >> magic >> cw >> ch >> mx; pf.get();
  std::vector<uint8_t> gray(static_cast<size_t>(cw) * ch);
  pf.read(reinterpret_cast<char*>(gray.data()), static_cast<std::streamsize>(gray.size()));
  std::vector<uint8_t> rgba(gray.size() * 4);
  for (size_t i = 0; i < gray.size(); ++i) { rgba[4 * i] = rgba[4 * i + 1] = rgba[4 * i + 2] = gray[i]; rgba[4 * i + 3] = 255; }
  const RgbaImageView cam{rgba.data(), cw, ch, cw * 4};
  const float scale_x = static_cast<float>(1.0 / cw), scale_y = static_cast<float>(1.0 / ch), pix_x = static_cast<float>(cw), pix_y = static_cast<float>(ch);
  std::ifstream cf(argv[3]);
  std::string line;
  int slot = 0;
  while (std::getline(cf, line)) {
    std::istringstream is(line);
    int c, r, w, h; float m[16]; std::string raw;
    is >> c >> r >> w >> h;
    for (float& v : m) is >> v;
    is >> raw;
    std::ifstream rf(raw, std::ios::binary);
    std::vector<uint8_t> orig((std::istreambuf_iterator<char>(rf)), std::istreambuf_iterator<char>());
    std::vector<uint8_t> mine(static_cast<size_t>(w) * h, 0);
    std::vector<float> uu(mine.size(), -1), vv(mine.size(), -1);
    if (variant == "engine") {
      const double hh[3][3] = {{m[0], m[4], m[12]}, {m[1], m[5], m[13]}, {m[3], m[7], m[15]}};
      const double det = hh[0][0] * (hh[1][1] * hh[2][2] - hh[1][2] * hh[2][1]) - hh[0][1] * (hh[1][0] * hh[2][2] - hh[1][2] * hh[2][0]) + hh[0][2] * (hh[1][0] * hh[2][1] - hh[1][1] * hh[2][0]);
      const double inv[3][3] = {{(hh[1][1] * hh[2][2] - hh[1][2] * hh[2][1]) / det, (hh[0][2] * hh[2][1] - hh[0][1] * hh[2][2]) / det, (hh[0][1] * hh[1][2] - hh[0][2] * hh[1][1]) / det},
                                {(hh[1][2] * hh[2][0] - hh[1][0] * hh[2][2]) / det, (hh[0][0] * hh[2][2] - hh[0][2] * hh[2][0]) / det, (hh[0][2] * hh[1][0] - hh[0][0] * hh[1][2]) / det},
                                {(hh[1][0] * hh[2][1] - hh[1][1] * hh[2][0]) / det, (hh[0][1] * hh[2][0] - hh[0][0] * hh[2][1]) / det, (hh[0][0] * hh[1][1] - hh[0][1] * hh[1][0]) / det}};
      for (int y = 0; y < h; ++y) for (int x = 0; x < w; ++x) {
        const double nx = 2.0 * ((x + 0.5) / w) - 1.0, ny = 2.0 * ((y + 0.5) / h) - 1.0;
        const double qx = inv[0][0] * nx + inv[0][1] * ny + inv[0][2], qy = inv[1][0] * nx + inv[1][1] * ny + inv[1][2], qw = inv[2][0] * nx + inv[2][1] * ny + inv[2][2];
        const double px = qx / qw, py = qy / qw;
        if (!(px >= -1 && px <= 1 && py >= -1 && py <= 1)) continue;
        const float tu = static_cast<float>(0.5 * (px + 1.0)), tv = static_cast<float>(0.5 * (py + 1.0));
        uu[y * w + x] = tu; vv[y * w + x] = tv;
        mine[y * w + x] = ShaderStage1CameraGray(cam, scale_x * (0.0f + pix_x * tu), scale_y * (0.0f + pix_y * tv), ShaderNumericModel::kSwiftShaderApprox);
      }
    } else {  // ss, ssg (general w, no w sort), ssw (general w + w sort)
      constexpr float kPos[4][2] = {{-1, 1}, {-1, -1}, {1, 1}, {1, -1}};
      constexpr float kUv[4][2] = {{0, 1}, {0, 0}, {1, 1}, {1, 0}};
      constexpr int kIdx[2][3] = {{0, 1, 2}, {2, 1, 3}};
      const float half_w = 0.5f * static_cast<float>(w), half_h = 0.5f * static_cast<float>(h);
      const float x0xf = (static_cast<float>(c) + half_w) * kSub - kSub / 2.0f, y0xf = (static_cast<float>(r) + half_h) * kSub - kSub / 2.0f;
      const float wxf = half_w * kSub, hxf = half_h * kSub;
      V vs[4];
      for (int i = 0; i < 4; ++i) {
        // GLSL mat4 * vec4(position, 0, 1), column-major uniform, unfused
        const float px = kPos[i][0], py = kPos[i][1];
        float cx, cy, cwv;
        if (CV == 1) {  // row-wise: m12 first
          cx = m[12] + (m[0] * px + m[4] * py); cy = m[13] + (m[1] * px + m[5] * py); cwv = m[15] + (m[3] * px + m[7] * py);
        } else if (CV == 2) {  // fused
          cx = std::fma(m[4], py, m[0] * px) + m[12]; cy = std::fma(m[5], py, m[1] * px) + m[13]; cwv = std::fma(m[7], py, m[3] * px) + m[15];
        } else if (CV == 3) {  // right-to-left
          cx = m[0] * px + (m[4] * py + (m[8] * 0.0f + m[12])); cy = m[1] * px + (m[5] * py + (m[9] * 0.0f + m[13])); cwv = m[3] * px + (m[7] * py + (m[11] * 0.0f + m[15]));
        } else {
          cx = ((m[0] * px + m[4] * py) + m[8] * 0.0f) + m[12]; cy = ((m[1] * px + m[5] * py) + m[9] * 0.0f) + m[13]; cwv = ((m[3] * px + m[7] * py) + m[11] * 0.0f) + m[15];
        }
        const float rhw = 1.0f / cwv;
        float fx, fy;
        switch (VX) {
          case 1: fx = x0xf + cx * (rhw * wxf); fy = y0xf + cy * (rhw * hxf); break;
          case 2: fx = std::fma(cx * rhw, wxf, x0xf); fy = std::fma(cy * rhw, hxf, y0xf); break;
          case 3: fx = static_cast<float>(static_cast<double>(x0xf) + static_cast<double>(cx) / cwv * wxf); fy = static_cast<float>(static_cast<double>(y0xf) + static_cast<double>(cy) / cwv * hxf); break;
          case 4: fx = x0xf + (cx / cwv) * wxf; fy = y0xf + (cy / cwv) * hxf; break;
          case 5: fx = x0xf + std::fma(cx, rhw, 0.0f) * wxf; fy = y0xf + cy * rhw * hxf; break;
          default: fx = x0xf + cx * rhw * wxf; fy = y0xf + cy * rhw * hxf;
        }
        auto rnd = [](float v) { switch (RM) { case 1: return static_cast<int>(std::floor(v + 0.5f)); case 2: return static_cast<int>(v); case 3: return static_cast<int>(std::ceil(v)); case 4: return static_cast<int>(std::floor(v)); default: return static_cast<int>(std::nearbyint(v)); } };
        vs[i].X = rnd(fx) + (i == PV ? PDX : 0);
        vs[i].Y = rnd(fy) + (i == PV ? PDY : 0);
        vs[i].yclip = variant == "ss" ? cy : cy * rhw;  vs[i].w = cwv; vs[i].rhw = rhw;
        vs[i].u = kUv[i][0]; vs[i].v = kUv[i][1];
      }
      if (std::getenv("DBG")) for (int i = 0; i < 4; ++i) std::printf("   vertex %d: X %d Y %d (px %.4f %.4f) w %.9g\n", i, vs[i].X, vs[i].Y, vs[i].X / 16.0, vs[i].Y / 16.0, vs[i].w);
      Tri tr[2];
      for (int t = 0; t < 2; ++t) {
        const V* v0 = &vs[kIdx[t][2]]; const V* v1 = &vs[kIdx[t][0]]; const V* v2 = &vs[kIdx[t][1]];
        tr[t].left.assign(1025, 0); tr[t].right.assign(1025, 0);
        if (RV == 0) Rasterize(*v0, *v1, *v2, 1024, &tr[t]);
        const float y0 = v0->yclip, y1 = v1->yclip, y2 = v2->yclip, ymin = std::min(std::min(y0, y1), y2);
        if (ymin == y1) { const V* vx = v0; v0 = v1; v1 = v2; v2 = vx; }
        if (ymin == y2) { const V* vx = v2; v2 = v1; v1 = v0; v0 = vx; }
        if (variant == "ssw") {  // second sort: maximum clip w first (SetupRoutine.cpp, after the y sort), values read after the first rotations
          const float w0 = v0->w, w1 = v1->w, w2 = v2->w, wmax = std::max(std::max(w0, w1), w2);
          if (wmax == w1) { const V* vx = v0; v0 = v1; v1 = v2; v2 = vx; }
          if (wmax == w2) { const V* vx = v2; v2 = v1; v1 = v0; v0 = vx; }
        }
        if (RV == 1) Rasterize(*v0, *v1, *v2, 1024, &tr[t]);
        SetupPlanes(*v0, *v1, *v2, &tr[t], variant != "ss");
      }
      for (int y = 0; y < h; ++y) for (int x = 0; x < w; ++x) {
        const int fx = c + x, fy = r + y;
        for (const Tri& t : tr) {
          if (fy < t.ymin || fy >= t.ymax || fx < t.left[fy] || fx >= t.right[fy]) continue;
          const float xf = static_cast<float>(fx) - t.x0, yf = static_cast<float>(fy) - t.y0;
          const float dw = t.w[2] + yf * t.w[1];
          const float rhw = 1.0f / (xf * t.w[0] + dw);
          const float du = t.pu[2] + yf * t.pu[1], dv = t.pv[2] + yf * t.pv[1];
          const float tu = (xf * t.pu[0] + du) * rhw, tv = (xf * t.pv[0] + dv) * rhw;
          uu[y * w + x] = tu; vv[y * w + x] = tv;
          mine[y * w + x] = ShaderStage1CameraGray(cam, scale_x * (0.0f + pix_x * tu), scale_y * (0.0f + pix_y * tv), ShaderNumericModel::kSwiftShaderApprox);
          break;
        }
      }
    }
    long diff = 0, cov_o = 0, cov_m = 0; int maxd = 0;
    for (size_t i = 0; i < mine.size(); ++i) { diff += mine[i] != orig[i]; maxd = std::max(maxd, std::abs(int(mine[i]) - int(orig[i]))); cov_m += uu[i] >= 0; cov_o += orig[i] != 0; }
    std::printf("%s slot%d (%dx%d): differing %ld / %zu  max|d| %d  covered(model) %ld  nonzero(orig) %ld\n", variant.c_str(), slot, w, h, diff, mine.size(), maxd, cov_m, cov_o);
    if (argc > 4) {  // dump the first differing pixels: x y orig mine u v
      int shown = 0;
      for (int y = 0; y < h && shown < 12; ++y) for (int x = 0; x < w && shown < 12; ++x)
        if (mine[y * w + x] != orig[y * w + x]) { std::printf("   (%d,%d) orig %d model %d  u %.9g v %.9g\n", x, y, orig[y * w + x], mine[y * w + x], uu[y * w + x], vv[y * w + x]); ++shown; }
    }
    if (argc > 4 && std::string(argv[4]) == "cov") {
      for (int y = 0; y < h; ++y) {
        int lo = -1, lm = -1, ro = -1, rm = -1;
        for (int x = 0; x < w; ++x) { if (orig[y * w + x] && lo < 0) lo = x; if (uu[y * w + x] >= 0 && lm < 0) lm = x; }
        for (int x = w - 1; x >= 0; --x) { if (orig[y * w + x] && ro < 0) ro = x; if (uu[y * w + x] >= 0 && rm < 0) rm = x; }
        if (lo != lm || ro != rm) std::printf("   row %d: left orig %d model %d | right orig %d model %d\n", y, lo, lm, ro, rm);
      }
    }
    ++slot;
  }
  return 0;
}

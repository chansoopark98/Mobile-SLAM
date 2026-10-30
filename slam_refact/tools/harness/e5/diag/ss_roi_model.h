// Harness-only (pixel-level ROI diagnosis, REPORT tier): SwiftShader model of one planar ROI slot draw, validated byte-for-byte against the ORIGINAL's GPU
// readback (glcap) on HIRES_SCAN and IMAGE_TARGET slots.  It is the identity-quad model of gr8_cpu_shader.cc (QuadVaryings) generalised to the vertices
// mvp * (+-1, +-1, 0, 1) with perspective: clip positions in float32, rhw = 1 / w, X/Y = round(x0xf + x rhw wxf) in 4 subpixel bits, triangles
// (0,1,2) (2,1,3) with the provoking vertex last, sort by minimum projected y then by maximum clip w (SetupRoutine), edge spans, plane equations with
// the clip w of vertices 1, 2 and 1/w of vertex 0, uv = P(x', y') / W(x', y'), spans cut to the slot rectangle (no geometric clipping of the quad).
// Not modelled: SwiftShader's clipper for quads that leave the viewport (visible as a few edge pixels, see report).
#ifndef SLAM_REFACT_TOOLS_E5_DIAG_SS_ROI_MODEL_H_
#define SLAM_REFACT_TOOLS_E5_DIAG_SS_ROI_MODEL_H_
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <vector>

#include "frontend/gr8_cpu_shader.h"

namespace ssroi {
constexpr float kSub = 16.0f;
struct V { int X, Y; float u, v, yproj, w, rhw; };
struct Tri { std::vector<int> left, right; int ymin = 0, ymax = 0; float x0 = 0, y0 = 0, pu[3] = {}, pv[3] = {}, w[3] = {}; };

inline void Edge(int xa, int ya, int xb, int yb, int fb, Tri* tri) {
  if (ya == yb) return;
  const bool swap = yb < ya;
  const int x1 = swap ? xb : xa, x2 = swap ? xa : xb, y1 = swap ? yb : ya, y2 = swap ? ya : yb;
  const int row1 = (y1 + 15) >> 4, row2 = (y2 + 15) >> 4;
  const int row_min = std::max(row1, 0), row_max = std::min(row2, fb);
  if (row_min >= row_max) return;
  std::vector<int>& edge = swap ? tri->right : tri->left;
  const int dx12 = x2 - x1, dy12 = y2 - y1, fdx12 = dx12 * 16, fdy12 = dy12 * 16;
  const int xx = dx12 * (row1 * 16 - y1) + (x1 & 15) * dy12;
  int x = (x1 >> 4) + xx / fdy12, err = xx % fdy12;
  const int ceil = err > 0 ? -1 : 0;
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

inline void Rasterize(const V& a, const V& b, const V& c, int fb, Tri* t) {
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

inline void SetupPlanes(const V& v0, const V& v1, const V& v2, Tri* t) {
  constexpr float kInv = 1.0f / kSub;
  t->x0 = static_cast<float>(v0.X) * kInv;
  t->y0 = static_cast<float>(v0.Y) * kInv;
  const float w1 = v1.w, w2 = v2.w, rhw0 = v0.rhw;
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

inline void RotateUv(int rotation, float u, float v, float* tu, float* tv) {
  if (rotation == -90 || rotation == 270) { *tu = 1.0f - v; *tv = u; }
  else if (rotation == 180 || rotation == -180) { *tu = 1.0f - u; *tv = 1.0f - v; }
  else if (rotation == -270 || rotation == 90) { *tu = v; *tv = 1.0f - u; }
  else { *tu = u; *tv = v; }
}

// Draws one slot (viewport c, r, w, h) with `m` = the 16 floats of the mvp uniform (column-major, c8::HMatrix::data() order) into the R plane of an RGBA
// atlas of width `size`: covered pixels get the gray of the camera sample, uncovered pixels are left unchanged (the caller clears the slot first).
inline void DrawSlot(int c, int r, int w, int h, const float m[16], int shader_rotation, const slam_refact::RgbaImageView& cam, slam_refact::ShaderNumericModel model,
                     int size, uint8_t* rgba) {
  constexpr float kPos[4][2] = {{-1, 1}, {-1, -1}, {1, 1}, {1, -1}};
  constexpr float kUv[4][2] = {{0, 1}, {0, 0}, {1, 1}, {1, 0}};
  constexpr int kIdx[2][3] = {{0, 1, 2}, {2, 1, 3}};
  const float half_w = 0.5f * static_cast<float>(w), half_h = 0.5f * static_cast<float>(h);
  const float x0xf = (static_cast<float>(c) + half_w) * kSub - kSub / 2.0f, y0xf = (static_cast<float>(r) + half_h) * kSub - kSub / 2.0f;
  const float wxf = half_w * kSub, hxf = half_h * kSub;
  const float scale_x = static_cast<float>(1.0 / cam.width), scale_y = static_cast<float>(1.0 / cam.height);
  const float pix_x = static_cast<float>(cam.width), pix_y = static_cast<float>(cam.height);
  V vs[4];
  for (int i = 0; i < 4; ++i) {
    const float px = kPos[i][0], py = kPos[i][1];
    const float cx = ((m[0] * px + m[4] * py) + m[8] * 0.0f) + m[12];
    const float cy = ((m[1] * px + m[5] * py) + m[9] * 0.0f) + m[13];
    const float cw = ((m[3] * px + m[7] * py) + m[11] * 0.0f) + m[15];
    const float rhw = 1.0f / cw;
    vs[i].X = static_cast<int>(std::nearbyint(x0xf + cx * rhw * wxf));
    vs[i].Y = static_cast<int>(std::nearbyint(y0xf + cy * rhw * hxf));
    vs[i].yproj = cy * rhw; vs[i].w = cw; vs[i].rhw = rhw;
    RotateUv(shader_rotation, kUv[i][0], kUv[i][1], &vs[i].u, &vs[i].v);
  }
  Tri tr[2];
  for (int t = 0; t < 2; ++t) {
    const V* v0 = &vs[kIdx[t][2]]; const V* v1 = &vs[kIdx[t][0]]; const V* v2 = &vs[kIdx[t][1]];
    tr[t].left.assign(static_cast<size_t>(size) + 1, 0); tr[t].right.assign(static_cast<size_t>(size) + 1, 0);
    Rasterize(*v0, *v1, *v2, size, &tr[t]);
    const float ymin = std::min(std::min(v0->yproj, v1->yproj), v2->yproj);
    const float y1 = v1->yproj, y2 = v2->yproj;
    if (ymin == y1) { const V* vx = v0; v0 = v1; v1 = v2; v2 = vx; }
    if (ymin == y2) { const V* vx = v2; v2 = v1; v1 = v0; v0 = vx; }
    const float wmax = std::max(std::max(v0->w, v1->w), v2->w);
    const float wa = v1->w, wb = v2->w;
    if (wmax == wa) { const V* vx = v0; v0 = v1; v1 = v2; v2 = vx; }
    if (wmax == wb) { const V* vx = v2; v2 = v1; v1 = v0; v0 = vx; }
    SetupPlanes(*v0, *v1, *v2, &tr[t]);
  }
  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w; ++x) {
      const int fx = c + x, fy = r + y;
      for (const Tri& t : tr) {
        if (fy < t.ymin || fy >= t.ymax || fx < t.left[fy] || fx >= t.right[fy]) continue;
        const float xf = static_cast<float>(fx) - t.x0, yf = static_cast<float>(fy) - t.y0;
        const float dw = t.w[2] + yf * t.w[1];
        const float rhw = 1.0f / (xf * t.w[0] + dw);
        const float du = t.pu[2] + yf * t.pu[1], dv = t.pv[2] + yf * t.pv[1];
        const float tu = (xf * t.pu[0] + du) * rhw, tv = (xf * t.pv[0] + dv) * rhw;
        rgba[(static_cast<size_t>(fy) * size + fx) * 4] =
            slam_refact::frontend::ShaderStage1CameraGray(cam, scale_x * (0.0f + pix_x * tu), scale_y * (0.0f + pix_y * tv), model);
        break;
      }
    }
  }
}
}  // namespace ssroi
#endif  // SLAM_REFACT_TOOLS_E5_DIAG_SS_ROI_MODEL_H_

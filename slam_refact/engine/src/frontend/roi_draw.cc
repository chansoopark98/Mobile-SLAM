// CPU ROI draw (contract FULL-3, US-013). Line references are to
// PUB:reality/engine/features/gl-reality-frame.cc unless noted.

#include "frontend/roi_draw.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <array>
#include <utility>
#include <vector>

#include "c8/hmatrix.h"
#include "common/check.h"
#include "frontend/gr8_cpu_shader.h"

namespace slam_refact::frontend {
namespace {

// gr8-rotate.vert uv -> texUv (flipY = 0).
void RotateUv(int rotation, float u, float v, float* tu, float* tv) {
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

// Geometric draw of one planar ROI (the exact inverse homography, double): the fallback for a quad the SwiftShader model does
// not cover (DrawPlanarRoi).
void DrawPlanarRoiGeometric(const LevelLayout& slot, const c8::HMatrix& warp, int shader_rotation,
                            ShaderNumericModel model, const RgbaImageView& camera, int size,
                            uint8_t* rgba) {
  // gl_Position = mvp * vec4(position.xy, 0, 1): the homography of rows/columns {0, 1, 3}.
  const double h[3][3] = {{warp(0, 0), warp(0, 1), warp(0, 3)},
                          {warp(1, 0), warp(1, 1), warp(1, 3)},
                          {warp(3, 0), warp(3, 1), warp(3, 3)}};
  const double det = h[0][0] * (h[1][1] * h[2][2] - h[1][2] * h[2][1]) -
                     h[0][1] * (h[1][0] * h[2][2] - h[1][2] * h[2][0]) +
                     h[0][2] * (h[1][0] * h[2][1] - h[1][1] * h[2][0]);
  if (det == 0.0 || !std::isfinite(det)) {
    return;  // degenerate warp: the quad covers no pixel
  }
  const double inv[3][3] = {
      {(h[1][1] * h[2][2] - h[1][2] * h[2][1]) / det, (h[0][2] * h[2][1] - h[0][1] * h[2][2]) / det,
       (h[0][1] * h[1][2] - h[0][2] * h[1][1]) / det},
      {(h[1][2] * h[2][0] - h[1][0] * h[2][2]) / det, (h[0][0] * h[2][2] - h[0][2] * h[2][0]) / det,
       (h[0][2] * h[1][0] - h[0][0] * h[1][2]) / det},
      {(h[1][0] * h[2][1] - h[1][1] * h[2][0]) / det, (h[0][1] * h[2][0] - h[0][0] * h[2][1]) / det,
       (h[0][0] * h[1][1] - h[0][1] * h[1][0]) / det}};
  // glUniform2f(scale, 1.0 / roiSrc.width(), ...) and pixScale = inputWidth_, inputHeight_.
  const float scale_x = static_cast<float>(1.0 / camera.width);
  const float scale_y = static_cast<float>(1.0 / camera.height);
  const float pix_x = static_cast<float>(camera.width);
  const float pix_y = static_cast<float>(camera.height);
  for (int y = 0; y < slot.h; ++y) {
    for (int x = 0; x < slot.w; ++x) {
      // Pixel centre in normalized device coordinates of glViewport(c, r, w, h) (row = GL y).
      const double nx = 2.0 * ((x + 0.5) / slot.w) - 1.0;
      const double ny = 2.0 * ((y + 0.5) / slot.h) - 1.0;
      const double qx = inv[0][0] * nx + inv[0][1] * ny + inv[0][2];
      const double qy = inv[1][0] * nx + inv[1][1] * ny + inv[1][2];
      const double qw = inv[2][0] * nx + inv[2][1] * ny + inv[2][2];
      if (qw == 0.0) continue;
      const double px = qx / qw;
      const double py = qy / qw;
      if (!(px >= -1.0 && px <= 1.0 && py >= -1.0 && py <= 1.0)) continue;
      const double clip_w = h[2][0] * px + h[2][1] * py + h[2][2];
      if (!(clip_w > 0.0)) continue;
      // Quad uv = (position + 1) / 2 (gl-quad.cc makeVertexArrayRect), then the vertex rotation.
      float tu, tv;
      RotateUv(shader_rotation, static_cast<float>(0.5 * (px + 1.0)),
               static_cast<float>(0.5 * (py + 1.0)), &tu, &tv);
      const uint8_t g = ShaderStage1CameraGray(camera, scale_x * (0.0f + pix_x * tu),
                                               scale_y * (0.0f + pix_y * tv), model);
      rgba[(static_cast<size_t>(slot.r + y) * size + (slot.c + x)) * 4] = g;
    }
  }
}

// One planar ROI into its slot of the R plane (drawInitialStage :384-445 with isCurvy = 0). Under kSwiftShaderApprox the
// quad is drawn by the SwiftShader setup model with the vertices mvp * (+-1, +-1, 0, 1) (US015-CC-1: texUv of the plane
// equations, edge spans of the 1/16 sub-pixel vertices), which is what the original's GPU readback contains. kIeee, and a
// quad that model does not cover (w <= 0 vertex, degenerate), use the geometric draw.
void DrawPlanarRoi(const LevelLayout& slot, const c8::HMatrix& warp, int shader_rotation,
                   ShaderNumericModel model, const RgbaImageView& camera, int size,
                   uint8_t* rgba) {
  std::vector<float> uv;
  std::vector<uint8_t> covered;
  if (model != ShaderNumericModel::kSwiftShaderApprox ||
      !SwiftShaderMvpQuadTexUv(slot.c, slot.r, slot.w, slot.h, warp.data().data(), shader_rotation, size, &uv,
                               &covered)) {
    DrawPlanarRoiGeometric(slot, warp, shader_rotation, model, camera, size, rgba);
    return;
  }
  // glUniform2f(scale, 1.0 / roiSrc.width(), ...) and pixScale = inputWidth_, inputHeight_.
  const float scale_x = static_cast<float>(1.0 / camera.width);
  const float scale_y = static_cast<float>(1.0 / camera.height);
  const float pix_x = static_cast<float>(camera.width);
  const float pix_y = static_cast<float>(camera.height);
  for (int y = 0; y < slot.h; ++y) {
    for (int x = 0; x < slot.w; ++x) {
      const size_t i = static_cast<size_t>(y) * slot.w + x;
      if (!covered[i]) continue;
      rgba[(static_cast<size_t>(slot.r + y) * size + (slot.c + x)) * 4] =
          ShaderStage1CameraGray(camera, scale_x * (0.0f + pix_x * uv[2 * i]), scale_y * (0.0f + pix_y * uv[2 * i + 1]), model);
    }
  }
}

// One curvy ROI into its slot (drawInitialStage :395-433 + gr8-stage1.frag getCurvyLiftedUv and
// main with isCurvy = 1, clear = 0). Float32 in the shader's operation order; GLSL mat4 * vec4 is
// m[col][row] * v[col] summed over the columns.
void Mat4Mul(const float* m, const float v[4], float out[4]) {
  for (int row = 0; row < 4; ++row) {
    out[row] = m[0 * 4 + row] * v[0] + m[1 * 4 + row] * v[1] + m[2 * 4 + row] * v[2] +
               m[3 * 4 + row] * v[3];
  }
}

bool IsIdentityWarp(const c8::HMatrix& w) {
  for (int r = 0; r < 4; ++r) {
    for (int c = 0; c < 4; ++c) {
      if (w(r, c) != (r == c ? 1.0f : 0.0f)) return false;
    }
  }
  return true;
}

// h = the 3x3 homography of the warp rows/columns {0, 1, 3} (gl_Position = mvp * vec4(position.xy, 0, 1)) and its inverse.
bool InvertWarp(const c8::HMatrix& warp, double h[3][3], double inv[3][3]) {
  const double m[3][3] = {{warp(0, 0), warp(0, 1), warp(0, 3)},
                          {warp(1, 0), warp(1, 1), warp(1, 3)},
                          {warp(3, 0), warp(3, 1), warp(3, 3)}};
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) h[i][j] = m[i][j];
  }
  const double det = h[0][0] * (h[1][1] * h[2][2] - h[1][2] * h[2][1]) -
                     h[0][1] * (h[1][0] * h[2][2] - h[1][2] * h[2][0]) +
                     h[0][2] * (h[1][0] * h[2][1] - h[1][1] * h[2][0]);
  if (det == 0.0 || !std::isfinite(det)) return false;
  inv[0][0] = (h[1][1] * h[2][2] - h[1][2] * h[2][1]) / det;
  inv[0][1] = (h[0][2] * h[2][1] - h[0][1] * h[2][2]) / det;
  inv[0][2] = (h[0][1] * h[1][2] - h[0][2] * h[1][1]) / det;
  inv[1][0] = (h[1][2] * h[2][0] - h[1][0] * h[2][2]) / det;
  inv[1][1] = (h[0][0] * h[2][2] - h[0][2] * h[2][0]) / det;
  inv[1][2] = (h[0][2] * h[1][0] - h[0][0] * h[1][2]) / det;
  inv[2][0] = (h[1][0] * h[2][1] - h[1][1] * h[2][0]) / det;
  inv[2][1] = (h[0][1] * h[2][0] - h[0][0] * h[2][1]) / det;
  inv[2][2] = (h[0][0] * h[1][1] - h[0][1] * h[1][0]) / det;
  return true;
}

// `warp` is the vertex `mvp` of the slot draw. The tracker gives an identity warp for a curvy ROI
// (detection-image-tracker.cc:250-256), where the quad uv is the pixel centre / slot size and the code below is the shader
// port; any other warp moves the quad like the planar draw does (the original uploads mvp = roi.warp for both).
void DrawCurvyRoi(const LevelLayout& slot, const c8::HMatrix& warp, const CurvyRoiUniforms& u, ShaderNumericModel model,
                  const RgbaImageView& camera, int size, uint8_t* rgba) {
  constexpr float kPiF = 3.1415926535897932384626433832795f;
  const bool identity = IsIdentityWarp(warp);
  double wh[3][3], winv[3][3];
  if (!identity && !InvertWarp(warp, wh, winv)) return;  // degenerate warp: the quad covers no pixel
  // `scale` = 1 / roiSrc size, `pixScale` = input size (:372-374).
  const float scale_x = static_cast<float>(1.0 / camera.width);
  const float scale_y = static_cast<float>(1.0 / camera.height);
  const float pix_x = static_cast<float>(camera.width);
  const float pix_y = static_cast<float>(camera.height);
  for (int y = 0; y < slot.h; ++y) {
    for (int x = 0; x < slot.w; ++x) {
      // Quad uv at the pixel centre, then the vertex-shader rotation (gr8-rotate.vert).
      float qu = (static_cast<float>(x) + 0.5f) / static_cast<float>(slot.w);
      float qv = (static_cast<float>(y) + 0.5f) / static_cast<float>(slot.h);
      if (!identity) {  // the quad point under the warp, covered iff inside the quad with clip w > 0 (as DrawPlanarRoi)
        const double nx = 2.0 * ((x + 0.5) / slot.w) - 1.0;
        const double ny = 2.0 * ((y + 0.5) / slot.h) - 1.0;
        const double qx = winv[0][0] * nx + winv[0][1] * ny + winv[0][2];
        const double qy = winv[1][0] * nx + winv[1][1] * ny + winv[1][2];
        const double qw = winv[2][0] * nx + winv[2][1] * ny + winv[2][2];
        if (qw == 0.0) continue;
        const double px = qx / qw;
        const double py = qy / qw;
        if (!(px >= -1.0 && px <= 1.0 && py >= -1.0 && py <= 1.0)) continue;
        if (!(wh[2][0] * px + wh[2][1] * py + wh[2][2] > 0.0)) continue;
        qu = static_cast<float>(0.5 * (px + 1.0));
        qv = static_cast<float>(0.5 * (py + 1.0));
      }
      float tu, tv;
      RotateUv(u.rotation, qu, qv, &tu, &tv);
      // getCurvyLiftedUv.
      const float ppx = tu * u.roi_dims[0];
      const float ppy = tv * u.roi_dims[1];
      if (ppx > u.target_dims[0] || ppy > u.target_dims[1]) continue;
      const float radius_at_y = tv * u.radius_bottom + u.radius * (1.0f - tv);
      const float v_x = 2.0f * kPiF * (u.shift + (u.scales[0] * ppx));
      const float sin_v = std::sin(v_x);
      const float cos_v = std::cos(v_x);
      const float normal[4] = {-radius_at_y * sin_v, radius_at_y * (u.radius_bottom - u.radius),
                               radius_at_y * cos_v, 0.0f};
      float transformed_normal[4];
      Mat4Mul(u.global_pose_inverse, normal, transformed_normal);
      const float search_pt[4] = {-radius_at_y * sin_v, -u.scales[1] * ppy + 0.5f,
                                  radius_at_y * cos_v, 1.0f};
      float ray_pt3[4];
      Mat4Mul(u.global_pose_inverse, search_pt, ray_pt3);
      const float one_over_z = 1.0f / ray_pt3[2];
      const float ray_x = ray_pt3[0] * one_over_z;
      const float ray_y = ray_pt3[1] * one_over_z;
      float pix_pt3[4];
      Mat4Mul(u.intrinsics, ray_pt3, pix_pt3);
      const float pix_x_pt = pix_pt3[0] * one_over_z;
      const float pix_y_pt = pix_pt3[1] * one_over_z;
      if (!std::isfinite(one_over_z) || !std::isfinite(pix_x_pt) || !std::isfinite(pix_y_pt)) continue;
      if (transformed_normal[0] * ray_x + transformed_normal[1] * ray_y + transformed_normal[2] * 1.0f >=
          0.0f) {
        continue;  // faces away from the camera: liftedUv = (-1, -1)
      }
      float lx, ly;
      const int post = u.curvy_post_rotation;
      if (post == -90 || post == 270) {
        lx = 1.0f - (pix_y_pt / u.search_dims[1]);
        ly = pix_x_pt / u.search_dims[0];
      } else if (post == 180 || post == -180) {
        lx = 1.0f - (pix_x_pt / u.search_dims[0]);
        ly = 1.0f - (pix_y_pt / u.search_dims[1]);
      } else if (post == -270 || post == 90) {
        lx = pix_y_pt / u.search_dims[1];
        ly = 1.0f - (pix_x_pt / u.search_dims[0]);
      } else {
        lx = pix_x_pt / u.search_dims[0];
        ly = pix_y_pt / u.search_dims[1];
      }
      if (lx == -1.0f && ly == -1.0f) continue;
      if (lx < 0.0f || lx > 1.0f || ly < 0.0f || ly > 1.0f) continue;
      // main: offsetPixels = pixScale * liftedUv - pixScale * texUv; texGray(offsetPixels) samples
      // scale * (offsetPixels + pixScale * texUv).
      const float off_x = (pix_x * lx) - (pix_x * tu);
      const float off_y = (pix_y * ly) - (pix_y * tv);
      const uint8_t g = ShaderStage1CameraGray(camera, scale_x * (off_x + pix_x * tu),
                                               scale_y * (off_y + pix_y * tv), model);
      rgba[(static_cast<size_t>(slot.r + y) * size + (slot.c + x)) * 4] = g;
    }
  }
}

}  // namespace

CurvyRoiUniforms ComputeCurvyRoiUniforms(const c8::ImageRoi& roi, const LevelLayout& slot,
                                         const LevelLayout& level0, int shader_rotation) {
  // gl-reality-frame.cc:397-433.
  c8::CurvyImageGeometry geom = roi.geom;
  bool is_rotated = false;
  if (geom.srcCols > geom.srcRows) {  // landscape image targets (including isRotated = true)
    is_rotated = true;
    std::swap(geom.srcRows, geom.srcCols);
  }
  const float sx = (geom.activationRegion.right - geom.activationRegion.left) / (geom.srcCols - 1);
  const float sy = 1.0f / (geom.srcRows - 1);
  const float shift = geom.activationRegion.left;
  const float aspect_ratio = 1.f * slot.w / slot.h;
  const bool wide = static_cast<float>(geom.srcCols) / geom.srcRows > aspect_ratio;
  const float roi_rows = wide ? geom.srcCols / aspect_ratio : geom.srcRows;
  const float roi_cols = wide ? geom.srcCols : aspect_ratio * geom.srcRows;
  CurvyRoiUniforms u;
  u.rotation = is_rotated ? 90 : 0;
  u.curvy_post_rotation = shader_rotation;
  u.target_dims[0] = static_cast<float>(geom.srcCols);
  u.target_dims[1] = static_cast<float>(geom.srcRows);
  u.roi_dims[0] = roi_cols;
  u.roi_dims[1] = roi_rows;
  u.search_dims[0] = static_cast<float>(level0.w);
  u.search_dims[1] = static_cast<float>(level0.h);
  const std::array<float, 16>& k = roi.intrinsics.data();
  const std::array<float, 16>& gp = roi.globalPose.inv().data();
  for (size_t i = 0; i < 16; ++i) {
    u.intrinsics[i] = k[i];
    u.global_pose_inverse[i] = gp[i];
  }
  u.scales[0] = sx;
  u.scales[1] = sy;
  u.shift = shift;
  u.radius = roi.geom.radius;
  u.radius_bottom = roi.geom.radiusBottom;
  return u;
}

void DrawCurvySlot(const LevelLayout& slot, const c8::HMatrix& warp, const CurvyRoiUniforms& u, ShaderNumericModel model,
                   const RgbaImageView& camera, int atlas_size, uint8_t* rgba) {
  DrawCurvyRoi(slot, warp, u, model, camera, atlas_size, rgba);
}

PyramidRoiCurvy MakeRoiCurvyExport(const c8::ImageRoi& roi, const LevelLayout& slot, const LevelLayout& level0,
                                   int shader_rotation) {
  const CurvyRoiUniforms u = ComputeCurvyRoiUniforms(roi, slot, level0, shader_rotation);
  PyramidRoiCurvy c;
  c.is_curvy = true;
  c.rotation = u.rotation;
  c.curvy_post_rotation = u.curvy_post_rotation;
  for (int i = 0; i < 2; ++i) {
    c.target_dims[i] = u.target_dims[i];
    c.roi_dims[i] = u.roi_dims[i];
    c.search_dims[i] = u.search_dims[i];
    c.scales[i] = u.scales[i];
  }
  for (int i = 0; i < 16; ++i) {
    c.intrinsics[i] = u.intrinsics[i];
    c.global_pose_inverse[i] = u.global_pose_inverse[i];
  }
  c.shift = u.shift;
  c.radius = u.radius;
  c.radius_bottom = u.radius_bottom;
  return c;
}

CurvyRoiUniforms UniformsFromExport(const PyramidRoiCurvy& c) {
  CurvyRoiUniforms u;
  u.rotation = c.rotation;
  u.curvy_post_rotation = c.curvy_post_rotation;
  for (int i = 0; i < 2; ++i) {
    u.target_dims[i] = c.target_dims[i];
    u.roi_dims[i] = c.roi_dims[i];
    u.search_dims[i] = c.search_dims[i];
    u.scales[i] = c.scales[i];
  }
  for (int i = 0; i < 16; ++i) {
    u.intrinsics[i] = c.intrinsics[i];
    u.global_pose_inverse[i] = c.global_pose_inverse[i];
  }
  u.shift = c.shift;
  u.radius = c.radius;
  u.radius_bottom = c.radius_bottom;
  return u;
}

int RoiAtlasRows(const PyramidLayout& layout) {
  int bottom = 0;
  for (const LevelLayout& s : layout.roi_slots) bottom = std::max(bottom, s.r + s.h);
  return std::min(layout.texture_size, std::max(layout.rows, bottom + kAtlasGutter));
}

void DrawRoisIntoAtlas(const PyramidLayout& layout, const CpuGr8Pyramid& cpu,
                       ShaderNumericModel model, const RgbaImageView& camera, int rotation_deg,
                       const std::vector<c8::ImageRoi>& rois, RgbaAtlas* atlas) {
  SR_CHECK(atlas != nullptr && atlas->size == layout.texture_size, "ROI draw: atlas size");
  SR_CHECK(rois.size() <= layout.roi_slots.size(), "ROI draw: more ROIs than slots");
  SR_CHECK(camera.data != nullptr && camera.width > 0 && camera.height > 0, "ROI draw: camera");
  const int rows = RoiAtlasRows(layout);
  if (atlas->rows < rows) {
    atlas->rgba.resize(static_cast<size_t>(atlas->size) * rows * 4, 0);
    atlas->rows = rows;
  }
  // Only R feeds stages 2 and 3 (CpuGr8Pyramid::RunStage2And3 reads the R plane); clear the slots
  // (the clear = 1 draw over the full quad, :389-392).
  for (const LevelLayout& s : layout.roi_slots) {
    for (int y = 0; y < s.h && s.r + y < atlas->rows; ++y) {
      for (int x = 0; x < s.w; ++x) {
        atlas->rgba[(static_cast<size_t>(s.r + y) * atlas->size + (s.c + x)) * 4] = 0;
      }
    }
  }
  for (size_t i = 0; i < rois.size(); ++i) {
    const c8::ImageRoi& roi = rois[i];
    const LevelLayout& slot = layout.roi_slots[i];
    const int shader_rotation = slot.rotated ? rotation_deg - 90 : rotation_deg;
    if (roi.source == c8::ImageRoi::Source::CURVY_IMAGE_TARGET) {
      DrawCurvyRoi(slot, roi.warp, ComputeCurvyRoiUniforms(roi, slot, layout.levels[0], shader_rotation), model,
                   camera, atlas->size, atlas->rgba.data());
    } else {
      DrawPlanarRoi(slot, roi.warp, shader_rotation, model, camera, atlas->size, atlas->rgba.data());
    }
  }
  cpu.RunStage2And3(atlas);
}

}  // namespace slam_refact::frontend

namespace slam_refact {

// Public atlas row counts (image_types.h) of the engine's pyramid layout.
int PyramidAtlasRows(int capture_width, int capture_height) {
  if (capture_width <= 0 || capture_height <= 0) return 0;
  return frontend::ComputePyramidLayout(capture_width, capture_height).rows;
}

int PyramidAtlasRoiRows(int capture_width, int capture_height) {
  if (capture_width <= 0 || capture_height <= 0) return 0;
  return frontend::RoiAtlasRows(frontend::ComputePyramidLayout(capture_width, capture_height));
}

}  // namespace slam_refact

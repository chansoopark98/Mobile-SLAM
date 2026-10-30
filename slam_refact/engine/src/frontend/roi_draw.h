// CPU port of the ROI part of GlRealityFrame::drawInitialStage (contract FULL-3, US-013;
// PUB:reality/engine/features/gl-reality-frame.cc:372-458) for planar ROIs (IMAGE_TARGET,
// HIRES_SCAN) and, since US-014, curvy ROIs (CURVY_IMAGE_TARGET: gr8-stage1.frag getCurvyLiftedUv).

#ifndef SLAM_REFACT_FRONTEND_ROI_DRAW_H_
#define SLAM_REFACT_FRONTEND_ROI_DRAW_H_

#include <vector>

#include "c8/pixels/image-roi.h"
#include "frontend/frontend.h"
#include "slam_refact/image_targets.h"
#include "slam_refact/image_types.h"
#include "slam_refact/types.h"

namespace slam_refact::frontend {

// The uniforms gl-reality-frame.cc:397-433 uploads for a CURVY_IMAGE_TARGET slot draw (isCurvy = 1),
// derived from the ROI and the slot exactly as the host code does. Matrices are 16 floats in
// c8::HMatrix storage order (column-major, usable directly with gl.uniformMatrix4fv(loc, false, m)).
// Used by the CPU draw (DrawRoisIntoAtlas) and exported to a GPU producer (GlueRunner::NextDrawRois).
struct CurvyRoiUniforms {
  int rotation = 0;              // `rotation` of the vertex shader: 90 for a landscape target, else 0
  int curvy_post_rotation = 0;   // `curvyPostRotation` = the slot's shader rotation
  float target_dims[2] = {0, 0};  // `targetDims`: geom.srcCols, geom.srcRows (after the landscape swap)
  float roi_dims[2] = {0, 0};     // `roiDims`: the ROI size in target pixels (keeps the target aspect)
  float search_dims[2] = {0, 0};  // `searchDims`: pyramid level 0 w, h
  float intrinsics[16] = {};      // `intrinsics` = roi.intrinsics (K, c8 convention)
  float global_pose_inverse[16] = {};  // `globalPoseInverse` = roi.globalPose.inv()
  float scales[2] = {0, 0};       // `scales`: (right - left) / (srcCols - 1), 1 / (srcRows - 1)
  float shift = 0.f;              // `shift`: activation region left
  float radius = 0.f;             // `radius` (top)
  float radius_bottom = 0.f;      // `radiusBottom`
};

// `slot` = the ROI slot layout, `level0` = pyramid level 0 layout, `shader_rotation` = slot.rotated ?
// rotation_deg - 90 : rotation_deg (the `rotation` of the slot draw).
CurvyRoiUniforms ComputeCurvyRoiUniforms(const c8::ImageRoi& roi, const LevelLayout& slot,
                                         const LevelLayout& level0, int shader_rotation);

// Stage 1 of one curvy slot from explicit uniforms and the ROI warp (the vertex `mvp`; identity for the tracker's ROIs): what
// DrawRoisIntoAtlas does for a CURVY_IMAGE_TARGET ROI before stages 2 and 3. A GPU producer's draw is this function with the
// uniforms of PyramidRoiRequest::curvy (UniformsFromExport), so the two can be compared bit for bit.
void DrawCurvySlot(const LevelLayout& slot, const c8::HMatrix& warp, const CurvyRoiUniforms& u, ShaderNumericModel model,
                   const RgbaImageView& camera, int atlas_size, uint8_t* rgba);

// The public export of the same uniforms (PyramidRoiRequest::curvy; contract FULL-9) and its inverse.
PyramidRoiCurvy MakeRoiCurvyExport(const c8::ImageRoi& roi, const LevelLayout& slot, const LevelLayout& level0,
                                   int shader_rotation);
CurvyRoiUniforms UniformsFromExport(const PyramidRoiCurvy& c);

// Number of atlas rows that cover every ROI slot plus the gutter below them (the GPU texture is
// processed in full; rows below this are R = 0, like PyramidLayout::rows below the levels).
int RoiAtlasRows(const PyramidLayout& layout);

// Draws `rois` (at most 4, slot i = rois[i]) into an atlas built by CpuGr8Pyramid::Build from the
// same camera image and reruns stages 2 and 3 on the whole atlas, as the GPU does on the full
// texture. Slots without an ROI stay cleared. The atlas is extended to RoiAtlasRows rows.
//
// Per slot (drawInitialStage): the slot is cleared, then the reference runtime quad is drawn with
// mvp = roi.warp, sampler = the camera texture (roiSrc, unfiltered input), scale = 1 / camera size,
// pixScale = camera size, rotation = the stage-1 rotation of the slot. Under kSwiftShaderApprox a slot pixel takes
// the texUv of the original's GPU draw (SwiftShader: vertices mvp * (+-1, +-1, 0, 1) in float32 rounded to 1/16 pixel,
// clipped to the viewport, triangle setup and plane equations of SwiftShaderMvpQuadTexUv; contract change US015-CC-1,
// byte-exact against the original's readback of 67 frames) and gets texGray of gr8-stage1.frag. Under kIeee, and for a
// quad that model does not cover (a vertex with clip w <= 0, degenerate), the pixel takes the perspective-correct texUv
// of the exact inverse homography of mvp rows/columns {0, 1, 3} at the pixel centre when that point lies inside the quad
// with clip w > 0 (the geometric draw; UNVERIFIED against a GPU readback).
//
// A CURVY_IMAGE_TARGET slot (US-014, roi.warp must be the identity) is drawn with the curvy branch
// of gr8-stage1.frag: every slot pixel centre gets texUv = (the vertex-shader rotation of the quad
// uv), is lifted to the cylinder / cone surface (getCurvyLiftedUv, uniforms CurvyRoiUniforms),
// projected into the camera with roi.intrinsics and roi.globalPose.inv() and samples the camera
// texture there; pixels that miss the target area, face away from the camera or project outside the
// camera image stay 0. The arithmetic is float32 in the shader's operation order (mediump on a GPU
// may be coarser, UNVERIFIED against a GPU readback).
void DrawRoisIntoAtlas(const PyramidLayout& layout, const CpuGr8Pyramid& cpu,
                       ShaderNumericModel model, const RgbaImageView& camera, int rotation_deg,
                       const std::vector<c8::ImageRoi>& rois, RgbaAtlas* atlas);

}  // namespace slam_refact::frontend

#endif  // SLAM_REFACT_FRONTEND_ROI_DRAW_H_

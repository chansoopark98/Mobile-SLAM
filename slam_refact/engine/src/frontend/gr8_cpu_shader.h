// Frontend private header: constants and shader helpers of the CPU port of the Gr8 GPU pyramid
// (S1 §2.2, §5.1-5.4). CpuGr8Pyramid itself is declared in frontend.h.

#ifndef SLAM_REFACT_FRONTEND_GR8_CPU_SHADER_H_
#define SLAM_REFACT_FRONTEND_GR8_CPU_SHADER_H_

#include <array>
#include <cstdint>
#include <vector>

#include "slam_refact/image_types.h"
#include "slam_refact/types.h"

namespace slam_refact::frontend {

// GlRealityFrame.PyramidSize default (func 459 f_tfa(31309, 1024), S1 §4 VERIFIED). The key is
// not in the recovered config table (config-defaults.json), so it is not overridable here.
inline constexpr int kPyramidSizeDefault = 1024;
// Gutter between atlas regions (gl-reality-frame.cc:157-163).
inline constexpr int kAtlasGutter = 3;
// Filtered input size (gl-reality-frame.cc:108-109).
inline constexpr int kFilteredWidth = 720;
inline constexpr int kFilteredHeight = 960;

// gr8-stage2.frag:17-34 scoreGr8 evaluated literally on the 16 circle differences d0..d15
// (S1 §5.2 order). Exposed for the F-FAST gate.
float ShaderFast9Score(const float d[16]);

// gr8-feature-shader.cc:111-140 createSubpixelParams (double math, cast to float); S1 §4
// subpixelQ[6]. Exposed for the F-H9 gate.
std::array<float, 6> SubpixelParams();

// texUv of every pixel of glViewport(c, r, w, h) drawn with the reference runtime quad under the SwiftShader
// varying model of kSwiftShaderApprox (S1 §5.1.3): (u, v) interleaved, row-major from (c, r), after
// the vertex-shader rotation. Exposed for the F-U1 gates (bit equality with the WebGL readback).
std::vector<float> SwiftShaderQuadTexUv(int c, int r, int w, int h, int rotation,
                                        int framebuffer_size);

// The same draw for a quad transformed by `mvp` (16 floats, column-major as uniformMatrix4fv receives them; contract change
// US015-CC-1): texUv of every pixel of the viewport in `uv` (2 floats per pixel) and `covered` (1 byte per pixel; uncovered
// pixels have uv 0). Returns false, leaving the outputs untouched, when the quad is outside the modelled domain (a vertex with
// clip w <= 0, non-finite or beyond 2^24 sub-pixels) or degenerate. A quad that leaves the viewport is clipped in clip space
// first, as SwiftShader does (see QuadVaryings).
bool SwiftShaderMvpQuadTexUv(int c, int r, int w, int h, const float mvp[16], int rotation, int framebuffer_size,
                             std::vector<float>* uv, std::vector<uint8_t>* covered);

// gr8-stage1.frag texGray(0) on the LINEAR camera texture at texture coordinate (u, v), followed by
// the unorm8 write, under the numeric model of the port (S1 §5.1.2). Exposed for the image target
// ROI draw (roi_draw.cc, contract FULL-3).
uint8_t ShaderStage1CameraGray(const RgbaImageView& camera, float u, float v,
                               ShaderNumericModel model);

}  // namespace slam_refact::frontend

#endif  // SLAM_REFACT_FRONTEND_GR8_CPU_SHADER_H_

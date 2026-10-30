// CPU port of the sky post-processing of the original LayersController: the cubemap accumulation
// and the output rendering of PUB:reality/engine/semantics/semantics-cubemap-renderer.{h,cc} with
// its four shaders (semantics-project, semantics-gaussian-linear, semantics-sky, image/cubemap).
// US-020; docs/spec/09-sky.md §3.
//
// The original draws with OpenGL ES (WebGL2). This port evaluates the same passes per pixel on the
// CPU: every pixel of a render target is computed at its pixel centre from the same matrices, quad
// uv tables, texture filters (GL_LINEAR, CLAMP_TO_EDGE; cubemap sampling is seamless) and 8-bit
// RGBA render targets (shader outputs are rounded to unorm8 at every pass). Where a GPU rasterises
// a primitive the port uses the equivalent inverse mapping; the places where that is only
// "equivalent up to the GPU's fixed-point arithmetic" are listed in the spec §3.4.
//
// Texture memory layout: row 0 = GL row 0 (see sky_types.h).
#ifndef SLAM_REFACT_SKY_SKY_CUBEMAP_H_
#define SLAM_REFACT_SKY_SKY_CUBEMAP_H_

#include <array>
#include <vector>

#include "c8/geometry/pixel-pinhole-camera-model.h"
#include "c8/hmatrix.h"
#include "sky/sky_types.h"

namespace slam_refact::sky {

class SkyCubemapRenderer {
 public:
  // SemanticsCubemapRenderer(int texSize = 256): the cubemap faces start as the original
  // constructor leaves them (solid yellow +Y, opaque black -Y, transparent black elsewhere in the
  // cubemap texture; the two working sets additionally carry the yellow "above the horizon" half on
  // the four side faces).
  explicit SkyCubemapRenderer(int tex_size = 256);

  // setProjectionIntrinsics: intrinsics of the camera that took the semantics image (the cropped
  // video, pixelsWidth x pixelsHeight = crop size).
  void SetProjectionIntrinsics(const c8_PixelPinholeCameraModel& k);

  // updateSrcTexture: the model output as gray RGBA (rgb = value, a = 255), kModelInputWidth x
  // kModelInputHeight.
  void UpdateSrcTexture(const Rgba8Image& src_pixels);

  // renderToCubemap(cameraExtrinsics): projects the source texture onto the cubemap faces the camera
  // frustum intersects.
  void RenderToCubemap(const c8::HMatrix& camera_extrinsics);

  // initOutputSemanticsScene(k, outputWidth, outputHeight).
  void InitOutputSemanticsScene(const c8_PixelPinholeCameraModel& k, int output_width,
                                int output_height);

  void SetDeviceOrientation(int orientation) { device_orientation_ = orientation; }
  // setSkyPostProcessParameters(edgeSmoothness, haveSeenSky, flipAlphaMask).
  void SetSkyPostProcessParameters(float edge_smoothness, bool have_seen_sky, bool flip_alpha_mask);

  // drawOutputSemantics(cameraExtrinsics, texId, texWidth, texHeight): renders the cubemap for the
  // camera, downsizes and blurs it, runs the sky shader, and returns the result texture content
  // (texWidth x texHeight).
  Rgba8Image DrawOutputSemantics(const c8::HMatrix& camera_extrinsics, int tex_width,
                                 int tex_height);

  // Cubemap filtering across face edges: seamless (texels across an edge come from the neighbouring
  // face) or clamped to the face. GL ES 3.0 requires seamless filtering; whether SwiftShader does it
  // is a measured property (docs/spec/09-sky.md §3.4).
  void set_seamless(bool seamless) { seamless_ = seamless; }
  bool seamless() const { return seamless_; }

  // Test access: the six faces of the cubemap texture (PX, NX, PY, NY, PZ, NZ order of the
  // original) and the ping-pong state.
  const Rgba8Image& cubemap_face(int face) const { return cube_tex_[face]; }
  int cube_tex_sampler_index() const { return cube_tex_sampler_index_; }
  // faces rendered by the last RenderToCubemap (bit i = face i).
  int last_rendered_faces() const { return last_rendered_faces_; }
  const Rgba8Image& source_texture() const { return src_tex_; }

 private:
  struct FaceGeometry {
    std::array<std::array<float, 3>, 4> points;  // quad vertices 0..3
    std::array<std::array<float, 2>, 4> uvs;
  };

  Rgba8Image RenderCubemap(const c8::HMatrix& camera_extrinsics, int width, int height) const;
  std::array<float, 4> SampleCubemap(float dx, float dy, float dz) const;

  int tex_size_ = 256;
  bool seamless_ = true;
  // cubePlanes_, cubeFaceTriangles_, cubeFaceBoundaryLines_ of the original.
  std::array<std::array<float, 4>, 6> cube_planes_{};
  std::array<FaceGeometry, 6> faces_{};

  Rgba8Image src_tex_;
  std::array<Rgba8Image, 6> cube_tex_;                  // cubeTex_ (cubemap texture)
  std::array<std::array<Rgba8Image, 6>, 2> cube_faces_;  // cubeFaces_[set][face]
  int cube_tex_sampler_index_ = 0;
  int last_rendered_faces_ = 0;

  bool has_projection_camera_ = false;
  c8::HMatrix project_camera_projection_;
  std::array<std::array<float, 3>, 4> frustum_local_corners_{};
  bool has_output_camera_ = false;
  c8::HMatrix output_camera_projection_;

  int output_width_ = 0;
  int output_height_ = 0;
  int portrait_width_ = 0;
  int portrait_height_ = 0;
  int device_orientation_ = 0;
  int have_seen_sky_ = 0;
  int flip_alpha_mask_ = 0;
  float edge_smoothness_ = 0.5f;
  std::array<float, 13> kernel_{};
  int kernel_size_ = 0;
};

}  // namespace slam_refact::sky

#endif  // SLAM_REFACT_SKY_SKY_CUBEMAP_H_

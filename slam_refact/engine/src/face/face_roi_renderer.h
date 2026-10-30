// CPU port of the face and ear ROI renderers (US-018/US-019; docs/spec/08-face-ear.md §3).
//
// The original renders the regions of interest on the GPU (PUB:reality/engine/faces/
// face-roi-renderer.{h,cc}, PUB:reality/engine/ears/ear-roi-renderer.{h,cc}, shader
// PUB:reality/engine/faces/face-roi-shader-data.h ROTATE + STAGE_1). The slam_refact engine has no
// GL, so this file draws the same quads on the CPU:
//   - the destination is cleared to (0, 0, 0, 255) (glClearColor(0, 0, 0, 1));
//   - each draw maps the camera image quad (clip space [-1, 1]^2, uv = (clip + 1) / 2) through
//     `mvp` (identity for the full image, ImageRoi::warp for a crop) into a viewport;
//   - the two triangles of the quad are rasterized like a GL rasterizer: window coordinates
//     snapped to 4 subpixel bits, pixel-centre samples, top-left rule, uv interpolated with the
//     barycentric weights of the snapped triangle;
//   - the camera texel value at that uv is read with bilinear filtering and clamp-to-edge
//     (GL_LINEAR; the camera texture parameters are set by the XR8 framework outside the public
//     source) and rounded to 8 bits.
// The read-back row order is preserved: buffer row 0 is GL row 0, which samples texture v near 0,
// which is the first (top) camera row. The draw/read pipelining of the original
// (GpuReadPixelsOptions::DEFER_READ: draw() first publishes the previous draw, then draws) is kept
// in DrawDeferred().
//
// Compared with the original render on SwiftShader (docs/spec/08-face-ear.md §8.2 T7a, §8.6 G2):
// the full-image draw is byte identical on the recorded sequence, but no recorded crop is (1-13
// edge pixels differ per crop), so the crops are "close", not "matching". Other GPUs (subpixel
// bits, texture filtering precision) are UNVERIFIED.
#ifndef SLAM_REFACT_FACE_FACE_ROI_RENDERER_H_
#define SLAM_REFACT_FACE_FACE_ROI_RENDERER_H_

#include "c8/geometry/face-types.h"
#include "c8/pixels/pixel-buffer.h"
#include "c8/vector.h"
#include "reality/engine/ears/ear-types.h"
#include "slam_refact/image_types.h"

namespace slam_refact::face {

// ImageRoi{source, faceId, "", warp} with the remaining members defaulted (the brace form of the
// public code, written as a function so -Wmissing-field-initializers stays quiet).
c8::ImageRoi MakeImageRoi(c8::ImageRoi::Source source, int face_id, const c8::HMatrix& warp);

// face-roi-renderer.cc viewportForFullImage: the letterboxed full image in a roi_width square.
c8::ImageViewport ViewportForFullImage(int video_width, int video_height, int roi_width);
// face-roi-renderer.cc viewportForCrop: 192x192 crop slot of ROI i.
c8::ImageViewport ViewportForCrop(int i);
// ear-roi-renderer.cc earViewportForCrop: 112x160 slot of ear ROI i.
c8::ImageViewport EarViewportForCrop(int i);

// Draws `src` (RGBA, row 0 = top) into `dest` inside viewport `vp` through `mvp` (column vector
// convention of c8::HMatrix, applied to quad vertices (x, y, 0, 1)). Pixels not rasterized are left
// unchanged. Returns false and draws nothing if `mvp` is not an affine matrix of finite numbers
// (the face and ear ROIs are affine) or maps the quad outside +-kMaxWindowCoordinate window
// pixels (a non-finite or absurd warp from a bad detection must not overflow the integer edge
// functions); true otherwise.
constexpr float kMaxWindowCoordinate = 16777216.0f;  // 2^24 pixels
bool DrawQuad(const RgbaImageView& src, const c8::HMatrix& mvp, const c8::ImageViewport& vp,
              c8::RGBA8888PlanePixels dest);

// FaceRoiRenderer on the CPU (face-roi-renderer.{h,cc}).
class FaceRoiRendererCpu {
 public:
  // roiWidth of the original constructor (default 128, at most 192).
  explicit FaceRoiRendererCpu(int roi_width = 128);

  // initialize(shader, srcWidth, srcHeight): 2x192 by 2x192 output.
  void Initialize(int src_width, int src_height);

  // setDetectedFaces: ROIs (detectionRoi = padded by 1.5) for the next draw, local faces first.
  void SetDetectedFaces(const c8::Vector<c8::DetectedPoints>& global_faces,
                        const c8::Vector<c8::DetectedPoints>& local_faces);
  void SetDetectedFaces(const c8::Vector<c8::DetectedPoints>& faces);

  // draw(texture, DEFER_READ): the previous draw becomes result(), then `frame` is drawn with the
  // ROIs set since the previous draw. Returns false, with no state change, if `frame` is not an
  // RGBA image of the Initialize() size (null data, other width/height, stride < 4 * width); the
  // original GL path re-creates the renderer in updateVideoGeometry() instead of aborting.
  bool DrawDeferred(const RgbaImageView& frame);
  // draw(texture, READ_IMMEDIATELY). Same return value.
  bool DrawImmediate(const RgbaImageView& frame);
  // Number of ROI quads skipped by DrawQuad() returning false (diagnostic; 0 for finite warps).
  int skipped_quads() const { return skipped_quads_; }

  // result(): viewports and sub-images of the published render. srcTex is the draw counter.
  c8::FaceRenderResult Result() const;
  bool has_render() const { return has_render_; }
  c8::ConstRGBA8888PlanePixels pixels() const { return out_.pixels(); }

 private:
  void DrawTexture(const RgbaImageView& frame);
  void ReadPixels();

  int skipped_quads_ = 0;
  int roi_width_;
  int input_width_ = 0;
  int input_height_ = 0;
  bool has_render_ = false;
  bool needs_read_ = false;
  int draw_count_ = 0;
  int drawn_tex_ = 0;
  int tex_ = 0;
  c8::RGBA8888PlanePixelBuffer drawn_;
  c8::RGBA8888PlanePixelBuffer out_;
  c8::Vector<c8::ImageRoi> next_face_rois_;
  c8::Vector<c8::ImageRoi> drawn_face_rois_;
  c8::Vector<c8::ImageRoi> face_rois_;
};

// EarRoiRenderer on the CPU (ear-roi-renderer.{h,cc}): 3x112 by 2x160 output, no full image.
class EarRoiRendererCpu {
 public:
  void Initialize(int src_width, int src_height);
  // setDetectedEars: detectionRoiNoPadding of each ear, face id preserved.
  void SetDetectedEars(const c8::Vector<c8::DetectedPoints>& ears);
  // Same contract as FaceRoiRendererCpu::DrawDeferred / DrawImmediate.
  bool DrawDeferred(const RgbaImageView& frame);
  bool DrawImmediate(const RgbaImageView& frame);
  int skipped_quads() const { return skipped_quads_; }
  c8::EarRenderResult Result() const;
  bool has_render() const { return has_render_; }
  c8::ConstRGBA8888PlanePixels pixels() const { return out_.pixels(); }

 private:
  void DrawTexture(const RgbaImageView& frame);
  void ReadPixels();

  int skipped_quads_ = 0;
  int input_width_ = 0;
  int input_height_ = 0;
  bool has_render_ = false;
  bool needs_read_ = false;
  int draw_count_ = 0;
  int drawn_tex_ = 0;
  int tex_ = 0;
  c8::RGBA8888PlanePixelBuffer drawn_;
  c8::RGBA8888PlanePixelBuffer out_;
  c8::Vector<c8::ImageRoi> next_ear_rois_;
  c8::Vector<c8::ImageRoi> drawn_ear_rois_;
  c8::Vector<c8::ImageRoi> ear_rois_;
};

}  // namespace slam_refact::face

#endif  // SLAM_REFACT_FACE_FACE_ROI_RENDERER_H_

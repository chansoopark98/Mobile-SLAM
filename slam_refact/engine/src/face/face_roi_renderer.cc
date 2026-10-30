// CPU face and ear ROI renderers (US-018/US-019). See face_roi_renderer.h.

#include "face/face_roi_renderer.h"

#include <algorithm>
#include <cmath>
#include <cstdint>

#include "c8/hmatrix.h"
#include "c8/hpoint.h"
#include "reality/engine/faces/face-geometry.h"

namespace slam_refact::face {
namespace {

// FaceRoiRenderer::initialize / EarRoiRenderer::initialize output sizes.
constexpr int kFaceOutputSize = 2 * 192;
constexpr int kEarOutputWidth = 3 * EAR_LANDMARK_DETECTION_INPUT_WIDTH;
constexpr int kEarOutputHeight = 2 * EAR_LANDMARK_DETECTION_INPUT_HEIGHT;

// glClearColor(0, 0, 0, 1); glClear(GL_COLOR_BUFFER_BIT).
void Clear(c8::RGBA8888PlanePixels dest) {
  for (int r = 0; r < dest.rows(); ++r) {
    uint8_t* row = dest.pixels() + r * dest.rowBytes();
    for (int c = 0; c < dest.cols(); ++c) {
      row[4 * c + 0] = 0;
      row[4 * c + 1] = 0;
      row[4 * c + 2] = 0;
      row[4 * c + 3] = 255;
    }
  }
}

// GL_LINEAR sample of an RGBA8 texture at normalized (s, t), clamp-to-edge, as 8-bit unorm with
// round-to-nearest. GL texel centres are at (i + 0.5) / size.
void SampleBilinear(const RgbaImageView& src, float s, float t, uint8_t* out) {
  const float u = s * static_cast<float>(src.width) - 0.5f;
  const float v = t * static_cast<float>(src.height) - 0.5f;
  const float u0f = std::floor(u);
  const float v0f = std::floor(v);
  const float fu = u - u0f;
  const float fv = v - v0f;
  const int u0 = static_cast<int>(u0f);
  const int v0 = static_cast<int>(v0f);
  const int x0 = std::clamp(u0, 0, src.width - 1);
  const int x1 = std::clamp(u0 + 1, 0, src.width - 1);
  const int y0 = std::clamp(v0, 0, src.height - 1);
  const int y1 = std::clamp(v0 + 1, 0, src.height - 1);
  const uint8_t* r0 = src.data + y0 * src.stride;
  const uint8_t* r1 = src.data + y1 * src.stride;
  for (int ch = 0; ch < 3; ++ch) {
    const float top = static_cast<float>(r0[4 * x0 + ch]) * (1.0f - fu) +
                      static_cast<float>(r0[4 * x1 + ch]) * fu;
    const float bottom = static_cast<float>(r1[4 * x0 + ch]) * (1.0f - fu) +
                         static_cast<float>(r1[4 * x1 + ch]) * fu;
    const float value = top * (1.0f - fv) + bottom * fv;
    out[ch] = static_cast<uint8_t>(std::clamp(std::floor(value + 0.5f), 0.0f, 255.0f));
  }
  // STAGE_1 shader: color.a = 1.0.
  out[3] = 255;
}

bool FrameMatches(const RgbaImageView& frame, int width, int height) {
  return frame.data != nullptr && frame.width == width && frame.height == height &&
         width > 0 && height > 0 && frame.stride >= 4 * frame.width;
}

}  // namespace

c8::ImageRoi MakeImageRoi(c8::ImageRoi::Source source, int face_id, const c8::HMatrix& warp) {
  c8::ImageRoi roi;
  roi.source = source;
  roi.faceId = face_id;
  roi.warp = warp;
  return roi;
}

c8::ImageViewport ViewportForFullImage(int video_width, int video_height, int roi_width) {
  // face-roi-renderer.cc:78-100.
  int target_width = video_width * roi_width / video_height;
  int target_height = roi_width;
  if (target_width > roi_width) {
    target_width = roi_width;
    target_height = video_height * roi_width / video_width;
  }
  const int half_roi_width = roi_width / 2;
  return {
      static_cast<float>(half_roi_width - target_width / 2),
      static_cast<float>(half_roi_width - target_height / 2),
      static_cast<float>(target_width),
      static_cast<float>(target_height),
  };
}

c8::ImageViewport ViewportForCrop(int i) {
  // face-roi-renderer.cc:102-131. i >= 3 goes where the letterbox crop normally goes, at 0, 0.
  float offset_x = 0;
  float offset_y = 0;
  if (i == 0) {
    offset_x = 192;
  }
  if (i == 1) {
    offset_y = 192;
  }
  if (i == 2) {
    offset_x = 192;
    offset_y = 192;
  }
  return {offset_x, offset_y, 192, 192};
}

c8::ImageViewport EarViewportForCrop(int i) {
  // ear-roi-renderer.cc earViewportForCrop.
  const int face_offset = i / 2;
  const int ear_offset = i % 2;
  int offset_x = face_offset * EAR_LANDMARK_DETECTION_INPUT_WIDTH;
  int offset_y = ear_offset * EAR_LANDMARK_DETECTION_INPUT_HEIGHT;
  if (i >= 6) {
    offset_x = offset_y = 0;
  }
  return {
      static_cast<float>(offset_x),
      static_cast<float>(offset_y),
      static_cast<float>(EAR_LANDMARK_DETECTION_INPUT_WIDTH),
      static_cast<float>(EAR_LANDMARK_DETECTION_INPUT_HEIGHT),
  };
}

bool DrawQuad(const RgbaImageView& src, const c8::HMatrix& mvp, const c8::ImageViewport& vp,
              c8::RGBA8888PlanePixels dest) {
  // gl-quad.cc makeVertexArrayRect: positions (x, y, 0), uvs, triangles {0, 1, 2}, {2, 1, 3}.
  // ROTATE shader: gl_Position = mvp * vec4(position, 1.0) (float), texUv = uv (rotation 0,
  // flipY 0). The rasterization follows a GL rasterizer: window coordinates
  // (ndc * 0.5 + 0.5) * size + offset snapped to kSubpixelBits, a sample at each pixel centre,
  // edge functions with the top-left rule (y up, as GL window coordinates; buffer row r is
  // window y = r), and texUv interpolated with the barycentric weights of the snapped triangle.
  // kSubpixelBits = 4 is the minimum of the GL ES 3.0 spec and the value of Chromium's SwiftShader
  // (SUBPIXEL_PRECISION_BITS with SWIFTSHADER_LEGACY_PRECISION). It gave the fewest differing
  // pixels against the 32 recorded original GPU crops, but none of them is byte identical, so it
  // is NOT a match of the original render (docs/spec/08-face-ear.md §3.2, §8.6 G2). Phone GPUs may
  // use more bits (UNVERIFIED).
  if (!(mvp(3, 0) == 0.0f && mvp(3, 1) == 0.0f && mvp(3, 2) == 0.0f && mvp(3, 3) == 1.0f)) {
    return false;  // projective or NaN matrix
  }
  constexpr int kSubpixelBits = 4;
  constexpr int64_t kOne = int64_t{1} << kSubpixelBits;
  constexpr float kPos[4][2] = {{-1.0f, 1.0f}, {-1.0f, -1.0f}, {1.0f, 1.0f}, {1.0f, -1.0f}};
  constexpr float kUv[4][2] = {{0.0f, 1.0f}, {0.0f, 0.0f}, {1.0f, 1.0f}, {1.0f, 0.0f}};
  constexpr int kTriangles[2][3] = {{0, 1, 2}, {2, 1, 3}};
  int64_t wx[4];
  int64_t wy[4];
  for (int k = 0; k < 4; ++k) {
    const c8::HPoint3 clip = mvp * c8::HPoint3(kPos[k][0], kPos[k][1], 0.0f);
    const float xw = (clip.x() * 0.5f + 0.5f) * vp.w + vp.x;
    const float yw = (clip.y() * 0.5f + 0.5f) * vp.h + vp.y;
    // With |window| <= 2^24 pixels the 4-bit fixed-point edge products stay below 2^62.
    if (!(std::fabs(xw) <= kMaxWindowCoordinate && std::fabs(yw) <= kMaxWindowCoordinate)) {
      return false;  // NaN, infinity or an absurd warp
    }
    wx[k] = std::llround(static_cast<double>(xw) * static_cast<double>(kOne));
    wy[k] = std::llround(static_cast<double>(yw) * static_cast<double>(kOne));
  }
  // Fragments are limited to the viewport (the scissor of glViewport is the framebuffer, but the
  // quad never leaves its viewport for the identity draw, and ROI draws clip at the viewport).
  const int vx0 = std::max(0, static_cast<int>(vp.x));
  const int vy0 = std::max(0, static_cast<int>(vp.y));
  const int vx1 = std::min(dest.cols(), static_cast<int>(vp.x + vp.w));
  const int vy1 = std::min(dest.rows(), static_cast<int>(vp.y + vp.h));
  for (const auto& tri : kTriangles) {
    int idx[3] = {tri[0], tri[1], tri[2]};
    auto edge = [&](int a, int b, int64_t cx, int64_t cy) {
      return (wx[b] - wx[a]) * (cy - wy[a]) - (wy[b] - wy[a]) * (cx - wx[a]);
    };
    int64_t area = edge(idx[0], idx[1], wx[idx[2]], wy[idx[2]]);
    if (area == 0) {
      continue;
    }
    if (area < 0) {
      std::swap(idx[1], idx[2]);
      area = -area;
    }
    // Edge k is opposite vertex k: (1, 2), (2, 0), (0, 1). Top-left rule for a counter-clockwise
    // triangle in y-up coordinates: left edges go down (dy < 0), top edges are horizontal and go
    // left (dy == 0, dx < 0).
    const int ea[3] = {idx[1], idx[2], idx[0]};
    const int eb[3] = {idx[2], idx[0], idx[1]};
    bool top_left[3];
    for (int e = 0; e < 3; ++e) {
      const int64_t dx = wx[eb[e]] - wx[ea[e]];
      const int64_t dy = wy[eb[e]] - wy[ea[e]];
      top_left[e] = dy < 0 || (dy == 0 && dx < 0);
    }
    const int64_t min_x = std::min({wx[idx[0]], wx[idx[1]], wx[idx[2]]});
    const int64_t max_x = std::max({wx[idx[0]], wx[idx[1]], wx[idx[2]]});
    const int64_t min_y = std::min({wy[idx[0]], wy[idx[1]], wy[idx[2]]});
    const int64_t max_y = std::max({wy[idx[0]], wy[idx[1]], wy[idx[2]]});
    const int x_begin = std::max<int64_t>(vx0, (min_x - kOne / 2) / kOne - 1);
    const int x_end = std::min<int64_t>(vx1, max_x / kOne + 1);
    const int y_begin = std::max<int64_t>(vy0, (min_y - kOne / 2) / kOne - 1);
    const int y_end = std::min<int64_t>(vy1, max_y / kOne + 1);
    for (int y = y_begin; y < y_end; ++y) {
      uint8_t* row = dest.pixels() + y * dest.rowBytes();
      const int64_t cy = static_cast<int64_t>(y) * kOne + kOne / 2;
      for (int x = x_begin; x < x_end; ++x) {
        const int64_t cx = static_cast<int64_t>(x) * kOne + kOne / 2;
        int64_t w[3];
        bool inside = true;
        for (int e = 0; e < 3 && inside; ++e) {
          w[e] = edge(ea[e], eb[e], cx, cy);
          inside = w[e] > 0 || (w[e] == 0 && top_left[e]);
        }
        if (!inside) {
          continue;
        }
        const double inv_area = 1.0 / static_cast<double>(area);
        double s = 0.0;
        double t = 0.0;
        for (int e = 0; e < 3; ++e) {
          const double b = static_cast<double>(w[e]) * inv_area;  // weight of vertex idx[e]
          s += b * kUv[idx[e]][0];
          t += b * kUv[idx[e]][1];
        }
        SampleBilinear(src, static_cast<float>(s), static_cast<float>(t), row + 4 * x);
      }
    }
  }
  return true;
}

FaceRoiRendererCpu::FaceRoiRendererCpu(int roi_width) : roi_width_(std::min(roi_width, 192)) {}

void FaceRoiRendererCpu::Initialize(int src_width, int src_height) {
  input_width_ = src_width;
  input_height_ = src_height;
  drawn_ = c8::RGBA8888PlanePixelBuffer(kFaceOutputSize, kFaceOutputSize);
  out_ = c8::RGBA8888PlanePixelBuffer(kFaceOutputSize, kFaceOutputSize);
  Clear(out_.pixels());
  has_render_ = false;
  needs_read_ = false;
}

void FaceRoiRendererCpu::SetDetectedFaces(const c8::Vector<c8::DetectedPoints>& global_faces,
                                          const c8::Vector<c8::DetectedPoints>& local_faces) {
  auto detected_faces = local_faces;
  detected_faces.insert(detected_faces.end(), global_faces.begin(), global_faces.end());
  SetDetectedFaces(detected_faces);
}

void FaceRoiRendererCpu::SetDetectedFaces(const c8::Vector<c8::DetectedPoints>& faces) {
  next_face_rois_.clear();
  next_face_rois_.reserve(faces.size());
  for (const auto& face : faces) {
    next_face_rois_.push_back(c8::detectionRoi(face));
    next_face_rois_.back().faceId = face.roi.faceId;  // Preserve the ROI id.
  }
}

void FaceRoiRendererCpu::DrawTexture(const RgbaImageView& frame) {
  // drawStage: clear, the full image with identity mvp, then each ROI crop. A crop whose warp
  // DrawQuad rejects stays the clear colour (and is counted in skipped_quads()).
  auto dest = drawn_.pixels();
  Clear(dest);
  DrawQuad(frame, c8::HMatrixGen::i(), ViewportForFullImage(input_width_, input_height_, roi_width_),
           dest);
  for (size_t i = 0; i < next_face_rois_.size(); ++i) {
    if (!DrawQuad(frame, next_face_rois_[i].warp, ViewportForCrop(static_cast<int>(i)), dest)) {
      ++skipped_quads_;
    }
  }
  needs_read_ = true;
  drawn_face_rois_ = next_face_rois_;
  drawn_tex_ = ++draw_count_;
  next_face_rois_.clear();
}

void FaceRoiRendererCpu::ReadPixels() {
  if (!needs_read_) {
    return;
  }
  needs_read_ = false;
  std::swap(out_, drawn_);
  has_render_ = true;
  face_rois_ = drawn_face_rois_;
  tex_ = drawn_tex_;
  drawn_face_rois_.clear();
  drawn_tex_ = 0;
}

bool FaceRoiRendererCpu::DrawDeferred(const RgbaImageView& frame) {
  if (!FrameMatches(frame, input_width_, input_height_)) {
    return false;
  }
  ReadPixels();
  DrawTexture(frame);
  return true;
}

bool FaceRoiRendererCpu::DrawImmediate(const RgbaImageView& frame) {
  if (!FrameMatches(frame, input_width_, input_height_)) {
    return false;
  }
  DrawTexture(frame);
  ReadPixels();
  return true;
}

c8::FaceRenderResult FaceRoiRendererCpu::Result() const {
  // face-roi-renderer.cc result().
  const auto fvp = ViewportForFullImage(input_width_, input_height_, roi_width_);
  c8::ConstRGBA8888PlanePixels op = out_.pixels();
  c8::FaceRenderResult r{
      tex_,
      op,
      {fvp, c8::imageForViewport(op, fvp),
       MakeImageRoi(c8::ImageRoi::Source::FACE, 0, c8::HMatrixGen::i())},
      {},
  };
  for (size_t i = 0; i < face_rois_.size(); ++i) {
    const auto vp = ViewportForCrop(static_cast<int>(i));
    r.faceMeshImages.push_back({vp, c8::imageForViewport(op, vp), face_rois_[i]});
  }
  return r;
}

void EarRoiRendererCpu::Initialize(int src_width, int src_height) {
  input_width_ = src_width;
  input_height_ = src_height;
  drawn_ = c8::RGBA8888PlanePixelBuffer(kEarOutputHeight, kEarOutputWidth);
  out_ = c8::RGBA8888PlanePixelBuffer(kEarOutputHeight, kEarOutputWidth);
  Clear(out_.pixels());
  has_render_ = false;
  needs_read_ = false;
}

void EarRoiRendererCpu::SetDetectedEars(const c8::Vector<c8::DetectedPoints>& ears) {
  next_ear_rois_.clear();
  next_ear_rois_.reserve(ears.size());
  for (const auto& ear : ears) {
    next_ear_rois_.push_back(c8::detectionRoiNoPadding(ear));
    next_ear_rois_.back().faceId = ear.roi.faceId;
  }
}

void EarRoiRendererCpu::DrawTexture(const RgbaImageView& frame) {
  auto dest = drawn_.pixels();
  Clear(dest);
  for (size_t i = 0; i < next_ear_rois_.size(); ++i) {
    if (!DrawQuad(frame, next_ear_rois_[i].warp, EarViewportForCrop(static_cast<int>(i)), dest)) {
      ++skipped_quads_;
    }
  }
  needs_read_ = true;
  drawn_ear_rois_ = next_ear_rois_;
  drawn_tex_ = ++draw_count_;
  next_ear_rois_.clear();
}

void EarRoiRendererCpu::ReadPixels() {
  if (!needs_read_) {
    return;
  }
  needs_read_ = false;
  std::swap(out_, drawn_);
  has_render_ = true;
  ear_rois_ = drawn_ear_rois_;
  tex_ = drawn_tex_;
  drawn_ear_rois_.clear();
  drawn_tex_ = 0;
}

bool EarRoiRendererCpu::DrawDeferred(const RgbaImageView& frame) {
  if (!FrameMatches(frame, input_width_, input_height_)) {
    return false;
  }
  ReadPixels();
  DrawTexture(frame);
  return true;
}

bool EarRoiRendererCpu::DrawImmediate(const RgbaImageView& frame) {
  if (!FrameMatches(frame, input_width_, input_height_)) {
    return false;
  }
  DrawTexture(frame);
  ReadPixels();
  return true;
}

c8::EarRenderResult EarRoiRendererCpu::Result() const {
  // ear-roi-renderer.cc result().
  c8::ConstRGBA8888PlanePixels op = out_.pixels();
  c8::EarRenderResult r{tex_, op, {}};
  for (size_t i = 0; i < ear_rois_.size(); ++i) {
    const auto vp = EarViewportForCrop(static_cast<int>(i));
    r.earDetectImages.push_back({vp, c8::imageForViewport(op, vp), ear_rois_[i]});
  }
  return r;
}

}  // namespace slam_refact::face

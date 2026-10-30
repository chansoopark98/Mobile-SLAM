// CPU ROI renderer (US-018/US-019; docs/spec/08-face-ear.md §5 T5).
#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

#include "c8/pixels/pixel-buffer.h"
#include "face/face_controller.h"
#include "face/face_models.h"
#include "face/face_roi_renderer.h"
#include "face_test_util.h"
#include "reality/engine/faces/face-geometry.h"

namespace slam_refact::face {
namespace {

std::vector<uint8_t> Frame(int w, int h, uint32_t seed) {
  testing::Lcg lcg(seed);
  std::vector<uint8_t> px(static_cast<size_t>(4 * w * h));
  for (size_t i = 0; i < px.size(); ++i) {
    px[i] = (i % 4 == 3) ? 255 : static_cast<uint8_t>(lcg.Next() * 256.0f);
  }
  return px;
}

TEST(FaceRoiRenderer, ViewportForFullImageMatchesTheOriginalExample) {
  // face-roi-renderer.cc:89-95 comment: 720x960 -> {16, 0, 96, 128}.
  const auto v = ViewportForFullImage(720, 960, 128);
  EXPECT_EQ(16.0f, v.x);
  EXPECT_EQ(0.0f, v.y);
  EXPECT_EQ(96.0f, v.w);
  EXPECT_EQ(128.0f, v.h);
  const auto l = ViewportForFullImage(640, 480, 128);  // landscape: letterboxed vertically
  EXPECT_EQ(0.0f, l.x);
  EXPECT_EQ(16.0f, l.y);
  EXPECT_EQ(128.0f, l.w);
  EXPECT_EQ(96.0f, l.h);
}

TEST(FaceRoiRenderer, CropSlots) {
  EXPECT_EQ(192.0f, ViewportForCrop(0).x);
  EXPECT_EQ(0.0f, ViewportForCrop(0).y);
  EXPECT_EQ(0.0f, ViewportForCrop(1).x);
  EXPECT_EQ(192.0f, ViewportForCrop(1).y);
  EXPECT_EQ(192.0f, ViewportForCrop(2).x);
  EXPECT_EQ(192.0f, ViewportForCrop(2).y);
  EXPECT_EQ(0.0f, ViewportForCrop(3).x);
  EXPECT_EQ(0.0f, ViewportForCrop(3).y);
}

TEST(FaceRoiRenderer, IdentitySameSizeIsAnExactCopy) {
  const int w = 96;
  const int h = 128;
  const auto px = Frame(w, h, 1);
  const RgbaImageView src{px.data(), w, h, 4 * w};
  c8::RGBA8888PlanePixelBuffer dest(h, w);
  DrawQuad(src, c8::HMatrixGen::i(), {0.0f, 0.0f, static_cast<float>(w), static_cast<float>(h)},
           dest.pixels());
  for (int r = 0; r < h; ++r) {
    for (int c = 0; c < 4 * w; ++c) {
      ASSERT_EQ(px[static_cast<size_t>(r * 4 * w + c)], dest.pixels().pixels()[r * dest.pixels().rowBytes() + c])
          << r << "," << c;
    }
  }
}

TEST(FaceRoiRenderer, PixelsOutsideTheWarpedQuadKeepTheClearColor) {
  const int w = 480;
  const int h = 640;
  const auto px = Frame(w, h, 2);
  FaceRoiRendererCpu renderer;
  renderer.Initialize(w, h);
  renderer.DrawImmediate({px.data(), w, h, 4 * w});
  const auto out = renderer.pixels();
  // Letterbox columns 0..15 of the 128 square and the empty 192 slots stay (0, 0, 0, 255).
  for (int r = 0; r < out.rows(); ++r) {
    for (int c = 0; c < out.cols(); ++c) {
      const bool in_full = c >= 16 && c < 112 && r < 128;
      if (in_full) {
        continue;
      }
      const uint8_t* p = out.pixels() + r * out.rowBytes() + 4 * c;
      ASSERT_EQ(0, p[0]) << r << "," << c;
      ASSERT_EQ(0, p[1]);
      ASSERT_EQ(0, p[2]);
      ASSERT_EQ(255, p[3]);
    }
  }
}

TEST(FaceRoiRenderer, NoPaddingCropOfAUniformBoxIsFullyCovered) {
  const int w = 480;
  const int h = 640;
  std::vector<uint8_t> px(static_cast<size_t>(4 * w * h), 0);
  // A uniform (200, 100, 50) rectangle over tex [0.3, 0.7] x [0.2, 0.6] on black.
  for (int r = 0; r < h; ++r) {
    for (int c = 0; c < w; ++c) {
      uint8_t* p = &px[static_cast<size_t>(4 * (r * w + c))];
      const float s = (c + 0.5f) / w;
      const float t = (r + 0.5f) / h;
      const bool inside = s > 0.3f && s < 0.7f && t > 0.2f && t < 0.6f;
      p[0] = inside ? 200 : 0;
      p[1] = inside ? 100 : 0;
      p[2] = inside ? 50 : 0;
      p[3] = 255;
    }
  }
  c8::DetectedPoints d;
  d.intrinsics = testing::IPhone6();
  // A box well inside the rectangle (two texels of margin for the bilinear footprint).
  d.boundingBox = {{0.32f, 0.22f}, {0.68f, 0.22f}, {0.32f, 0.58f}, {0.68f, 0.58f}};
  d.roi.faceId = 1;
  EarRoiRendererCpu renderer;
  renderer.Initialize(w, h);
  renderer.SetDetectedEars({d});
  renderer.DrawImmediate({px.data(), w, h, 4 * w});
  const auto result = renderer.Result();
  ASSERT_EQ(1u, result.earDetectImages.size());
  EXPECT_EQ(1, result.earDetectImages[0].roi.faceId);
  const auto crop = result.earDetectImages[0].image;
  ASSERT_EQ(160, crop.rows());
  ASSERT_EQ(112, crop.cols());
  for (int r = 0; r < crop.rows(); ++r) {
    for (int c = 0; c < crop.cols(); ++c) {
      const uint8_t* p = crop.pixels() + r * crop.rowBytes() + 4 * c;
      ASSERT_EQ(200, p[0]) << r << "," << c;
      ASSERT_EQ(100, p[1]);
      ASSERT_EQ(50, p[2]);
    }
  }
}

TEST(FaceRoiRenderer, DeferredDrawPublishesThePreviousFrame) {
  const int w = 480;
  const int h = 640;
  const auto a = Frame(w, h, 3);
  const auto b = Frame(w, h, 4);
  FaceRoiRendererCpu deferred;
  deferred.Initialize(w, h);
  FaceRoiRendererCpu immediate;
  immediate.Initialize(w, h);
  deferred.DrawDeferred({a.data(), w, h, 4 * w});
  EXPECT_FALSE(deferred.has_render());
  deferred.DrawDeferred({b.data(), w, h, 4 * w});
  ASSERT_TRUE(deferred.has_render());
  immediate.DrawImmediate({a.data(), w, h, 4 * w});
  const auto x = deferred.pixels();
  const auto y = immediate.pixels();
  for (int r = 0; r < x.rows(); ++r) {
    for (int c = 0; c < 4 * x.cols(); ++c) {
      ASSERT_EQ(y.pixels()[r * y.rowBytes() + c], x.pixels()[r * x.rowBytes() + c]);
    }
  }
  EXPECT_EQ(1, deferred.Result().srcTex);
}

TEST(FaceRoiRenderer, RoisSetBeforeADrawApplyToThatDrawOnly) {
  const int w = 480;
  const int h = 640;
  const auto a = Frame(w, h, 5);
  c8::DetectedPoints d;
  d.intrinsics = testing::IPhone6();
  d.boundingBox = {{0.2f, 0.3f}, {0.3f, 0.3f}, {0.2f, 0.5f}, {0.3f, 0.5f}};
  d.roi.faceId = 2;
  FaceRoiRendererCpu renderer;
  renderer.Initialize(w, h);
  renderer.SetDetectedFaces({}, {d});
  renderer.DrawImmediate({a.data(), w, h, 4 * w});
  auto r1 = renderer.Result();
  ASSERT_EQ(1u, r1.faceMeshImages.size());
  EXPECT_EQ(2, r1.faceMeshImages[0].roi.faceId);
  EXPECT_EQ(192.0f, r1.faceMeshImages[0].viewport.x);
  renderer.DrawImmediate({a.data(), w, h, 4 * w});
  EXPECT_TRUE(renderer.Result().faceMeshImages.empty());
}

// RM-7 (review MINOR 8): a frame that does not match Initialize() is an error the caller can
// handle, not an abort (the original GL path adapts in updateVideoGeometry()).
TEST(FaceRoiRenderer, FrameSizeMismatchIsAnErrorNotAnAbort) {
  const int w = 480;
  const int h = 640;
  const auto good = Frame(w, h, 6);
  const auto other = Frame(h, w, 7);  // 640x480
  FaceRoiRendererCpu face;
  face.Initialize(w, h);
  EarRoiRendererCpu ear;
  ear.Initialize(w, h);
  EXPECT_FALSE(face.DrawImmediate({other.data(), h, w, 4 * h}));
  EXPECT_FALSE(face.DrawDeferred({other.data(), h, w, 4 * h}));
  EXPECT_FALSE(face.DrawImmediate({nullptr, w, h, 4 * w}));
  EXPECT_FALSE(face.DrawImmediate({good.data(), w, h, 4 * w - 1}));
  EXPECT_FALSE(ear.DrawImmediate({other.data(), h, w, 4 * h}));
  EXPECT_FALSE(ear.DrawDeferred({nullptr, w, h, 4 * w}));
  EXPECT_FALSE(face.has_render());  // rejected frames change no state
  EXPECT_FALSE(ear.has_render());
  EXPECT_TRUE(face.DrawImmediate({good.data(), w, h, 4 * w}));
  EXPECT_TRUE(ear.DrawImmediate({good.data(), w, h, 4 * w}));
  EXPECT_TRUE(face.has_render());
}

// RM-7 (review MINOR 8): non-finite or absurd warps neither overflow the int64 edge functions
// (undefined behaviour) nor draw; the slot keeps the clear colour.
TEST(FaceRoiRenderer, NonFiniteOrAbsurdWarpsAreRejected) {
  const int w = 96;
  const int h = 128;
  const auto px = Frame(w, h, 8);
  const RgbaImageView src{px.data(), w, h, 4 * w};
  const float nan = std::numeric_limits<float>::quiet_NaN();
  const float inf = std::numeric_limits<float>::infinity();
  const c8::ImageViewport vp{0.0f, 0.0f, static_cast<float>(w), static_cast<float>(h)};
  auto expect_clear = [&](const c8::RGBA8888PlanePixelBuffer& d) {
    for (int r = 0; r < h; ++r) {
      for (int c = 0; c < w; ++c) {
        const uint8_t* p = d.pixels().pixels() + r * d.pixels().rowBytes() + 4 * c;
        ASSERT_EQ(0, p[0] | p[1] | p[2]) << r << "," << c;
      }
    }
  };
  for (float bad : {nan, inf, -inf, 1e30f, 3.0e8f}) {
    c8::RGBA8888PlanePixelBuffer dest(h, w);
    for (int r = 0; r < h; ++r) {
      for (int c = 0; c < w; ++c) {
        uint8_t* p = dest.pixels().pixels() + r * dest.pixels().rowBytes() + 4 * c;
        p[0] = p[1] = p[2] = 0;
        p[3] = 255;
      }
    }
    const c8::HMatrix scale = c8::HMatrixGen::scale(bad, 1.0f, 1.0f);
    EXPECT_FALSE(DrawQuad(src, scale, vp, dest.pixels())) << bad;
    const c8::HMatrix shift = c8::HMatrixGen::translation(0.0f, bad, 0.0f);
    EXPECT_FALSE(DrawQuad(src, shift, vp, dest.pixels())) << bad;
    expect_clear(dest);
  }
  // The same through the renderer: a NaN bounding box gives a NaN warp; the draw is skipped and
  // counted, the full image is still drawn.
  c8::DetectedPoints d;
  d.intrinsics = testing::IPhone6();
  d.boundingBox = {{nan, 0.3f}, {0.3f, 0.3f}, {0.2f, 0.5f}, {0.3f, 0.5f}};
  d.roi.faceId = 1;
  const auto big = Frame(480, 640, 9);
  FaceRoiRendererCpu renderer;
  renderer.Initialize(480, 640);
  renderer.SetDetectedFaces({}, {d});
  ASSERT_TRUE(renderer.DrawImmediate({big.data(), 480, 640, 4 * 480}));
  EXPECT_EQ(1, renderer.skipped_quads());
  // A finite warp is not counted.
  d.boundingBox = {{0.2f, 0.3f}, {0.3f, 0.3f}, {0.2f, 0.5f}, {0.3f, 0.5f}};
  renderer.SetDetectedFaces({}, {d});
  ASSERT_TRUE(renderer.DrawImmediate({big.data(), 480, 640, 4 * 480}));
  EXPECT_EQ(1, renderer.skipped_quads());
}

// RM-1d (review RM-7 MINOR 9): the acceptance boundary of the warp guard. For this 96x128 viewport
// the largest window coordinate of a scale s is 48 s + 48 in x and 64 s + 64 in y, and the guard
// admits it up to kMaxWindowCoordinate (2^24 = 1.678e7 pixels): x scale 3.4e5 (1.632e7) must draw,
// 3.6e5 (1.728e7) must be refused; y scale 2.5e5 (1.600e7) must draw, 2.7e5 (1.728e7) refused.
// The accepted cases exercise the largest int64 edge products the guard admits (about 2^59). This
// is a logic test; an UBSan run of it was not made (UNVERIFIED).
TEST(FaceRoiRenderer, WarpGuardAcceptsJustBelowAndRefusesJustAboveTheLimit) {
  const int w = 96;
  const int h = 128;
  const auto px = Frame(w, h, 11);
  const RgbaImageView src{px.data(), w, h, 4 * w};
  const c8::ImageViewport vp{0.0f, 0.0f, static_cast<float>(w), static_cast<float>(h)};
  struct Case {
    bool vertical;
    float scale;
  };
  for (const Case c : {Case{false, 3.4e5f}, Case{false, 3.6e5f}, Case{true, 2.5e5f},
                       Case{true, 2.7e5f}}) {
    c8::RGBA8888PlanePixelBuffer dest(h, w);
    const c8::HMatrix m = c.vertical ? c8::HMatrixGen::scale(1.0f, c.scale, 1.0f)
                                     : c8::HMatrixGen::scale(c.scale, 1.0f, 1.0f);
    const double extent = c.vertical ? 64.0 : 48.0;
    // The literal 2^24, not kMaxWindowCoordinate: comparing against the constant under test could
    // not notice a changed limit (checked with a mutant of 2^25).
    const bool inside = extent * c.scale + extent <= 16777216.0;
    EXPECT_EQ(inside, DrawQuad(src, m, vp, dest.pixels()))
        << (c.vertical ? "y " : "x ") << c.scale;
  }
}

// RM-7 (review MINOR 8): FaceController reports a mismatched frame and survives a zero-size
// camera environment between StageFrame() and ProcessStagedFrame() (no models are needed: neither
// path reaches the tracker).
TEST(FaceControllerFrames, MismatchedFrameAndZeroEnvironmentAreHandled) {
  FaceController controller;
  FaceCameraEnvironment env;
  env.camera_width = 480;
  env.camera_height = 640;
  env.display_width = 480;
  env.display_height = 640;
  controller.UpdateCameraEnvironment(env);
  const auto good = Frame(480, 640, 10);
  const auto other = Frame(640, 480, 11);
  std::string error;
  EXPECT_FALSE(controller.StageFrame({other.data(), 640, 480, 4 * 640}, &error));
  EXPECT_NE(std::string::npos, error.find("differs"));
  EXPECT_FALSE(controller.StageFrame({nullptr, 480, 640, 4 * 480}));
  EXPECT_EQ(nullptr, controller.ProcessStagedFrame());  // nothing was queued
  EXPECT_TRUE(controller.StageFrame({good.data(), 480, 640, 4 * 480}));
  EXPECT_TRUE(controller.StageFrame({good.data(), 480, 640, 4 * 480}));
  env.camera_width = 0;
  env.camera_height = 0;
  controller.UpdateCameraEnvironment(env);  // drops the renderers
  EXPECT_EQ(nullptr, controller.ProcessStagedFrame());
  error.clear();
  EXPECT_FALSE(controller.StageFrame({good.data(), 480, 640, 4 * 480}, &error));
  EXPECT_NE(std::string::npos, error.find("zero size"));
}

// RM-7 (review MINOR 5): pin the 4 subpixel-bit snap of DrawQuad. Inputs recorded from the
// original runtime (SwiftShader GPU, run-face-reference.mjs sincos60-r1): the camera texture of
// staged frame 2, the render-tex-to-image-tex matrix of its first mesh detection, and the 192x192
// crop the original GPU drew for it. The expectation is the observation made when RM-7 was
// written (after the fact, a regression pin, not original-equivalence evidence): the CPU crop
// differs from the GPU crop in exactly one pixel, (row 127, col 157), by 115 levels, a quad-edge
// coverage disagreement. Measured on the same 32 recorded crops: kSubpixelBits = 4 differs in 198
// pixels in total, 5 in 246,632, 6 in 205,727, 8 in 200,133, 3 in 387,178 (RM-7 G5).
std::vector<uint8_t> ReadDataFile(const std::string& name, const char* sha256) {
  std::vector<uint8_t> bytes;
  std::string error;
  EXPECT_TRUE(ReadFileBytes(std::string(SR_FACE_TEST_DATA_DIR) + "/" + name, &bytes, &error))
      << error;
  EXPECT_EQ(std::string(sha256), Sha256Hex(bytes)) << name;
  return bytes;
}

TEST(FaceRoiRenderer, SubpixelSnapMatchesTheRecordedOriginalGpuCrop) {
  const auto texture = ReadDataFile(
      "rm7_snap_texture_0002.rgba", "2d121a15d3ed6854ccec6abda7e5f3f23b789302ce424ddb5d340d1362769789");
  const auto gpu = ReadDataFile(
      "rm7_snap_gpu_crop_0.rgba", "8f62fbcba598a589214388ae10782fa74102b7aa68e9213175d1fb26aee90515");
  const auto roi = ReadDataFile(
      "rm7_snap_roi_0.txt", "56c99f1fac828012b166e33d90fc298a2f21881d770cae292f4f016e9bffe712");
  ASSERT_EQ(480u * 640u * 4u, texture.size());
  ASSERT_EQ(192u * 192u * 4u, gpu.size());
  std::istringstream ss(std::string(roi.begin(), roi.end()));
  int id = 0;
  int tex = 0;
  float m[16];
  ss >> id >> tex;
  for (float& v : m) {
    ss >> v;
  }
  ASSERT_TRUE(static_cast<bool>(ss));
  // warp = texToClip M^-1 texToClip^-1 (the renderTexToImageTex of the original is
  // texToClip^-1 warp^-1 texToClip), rebuilt exactly as sr_face_openloop does.
  const c8::HMatrix tex_to_clip{{2.0f, 0.0f, 0.0f, -1.0f}, {0.0f, 2.0f, 0.0f, -1.0f},
                                {0.0f, 0.0f, 1.0f, 0.0f},  {0.0f, 0.0f, 0.0f, 1.0f},
                                {0.5f, 0.0f, 0.0f, 0.5f},  {0.0f, 0.5f, 0.0f, 0.5f},
                                {0.0f, 0.0f, 1.0f, 0.0f},  {0.0f, 0.0f, 0.0f, 1.0f}};
  const c8::HMatrix m_full{{m[0], m[4], m[8], m[12]},
                           {m[1], m[5], m[9], m[13]},
                           {m[2], m[6], m[10], m[14]},
                           {m[3], m[7], m[11], m[15]}};
  const c8::HMatrix w = tex_to_clip * m_full.inv() * tex_to_clip.inv();
  const c8::HMatrix warp{{w(0, 0), w(0, 1), w(0, 2), w(0, 3)},
                         {w(1, 0), w(1, 1), w(1, 2), w(1, 3)},
                         {w(2, 0), w(2, 1), w(2, 2), w(2, 3)},
                         {0.0f, 0.0f, 0.0f, 1.0f}};
  c8::RGBA8888PlanePixelBuffer crop(192, 192);
  auto cp = crop.pixels();
  for (int r = 0; r < 192; ++r) {
    for (int c = 0; c < 192; ++c) {
      uint8_t* p = cp.pixels() + r * cp.rowBytes() + 4 * c;
      p[0] = p[1] = p[2] = 0;
      p[3] = 255;
    }
  }
  ASSERT_TRUE(DrawQuad({texture.data(), 480, 640, 4 * 480}, warp, {0.0f, 0.0f, 192.0f, 192.0f}, cp));
  int differing = 0;
  int row = -1;
  int col = -1;
  int level = 0;
  for (int r = 0; r < 192; ++r) {
    for (int c = 0; c < 192; ++c) {
      int worst = 0;
      for (int ch = 0; ch < 3; ++ch) {
        worst = std::max(worst, std::abs(static_cast<int>(cp.pixels()[r * cp.rowBytes() + 4 * c + ch]) -
                                         static_cast<int>(gpu[static_cast<size_t>((r * 192 + c) * 4 + ch)])));
      }
      if (worst > 0) {
        ++differing;
        row = r;
        col = c;
        level = worst;
      }
    }
  }
  EXPECT_EQ(1, differing);
  EXPECT_EQ(127, row);
  EXPECT_EQ(157, col);
  EXPECT_EQ(115, level);
}

}  // namespace
}  // namespace slam_refact::face

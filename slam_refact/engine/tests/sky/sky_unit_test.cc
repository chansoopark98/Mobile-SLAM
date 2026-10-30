// Sky unit tests (US-020; docs/spec/09-sky.md §5). No model and no recording needed: the pure
// functions against golden data produced by executing the original JS (tools/sky/gen_js_golden.mjs),
// the CPU cubemap renderer's structure and metamorphic properties, and the LayersController state
// machine with a fake worker. Results from synthetic inputs are not accuracy evidence.
#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include "c8/geometry/device-pose.h"
#include "c8/geometry/intrinsics-clip.h"
#include "sky/sky_cubemap.h"
#include "sky/sky_layers_controller.h"
#include "sky/sky_model.h"
#include "sky/sky_pipeline.h"
#include "sky/sky_preprocess.h"

namespace slam_refact::sky {
namespace {

std::vector<std::vector<std::string>> ReadTsv(const std::string& name) {
  std::ifstream f(std::string(SR_SKY_GOLDEN_DIR) + "/" + name);
  EXPECT_TRUE(f.good()) << "cannot open golden file " << name << " in " << SR_SKY_GOLDEN_DIR;
  std::vector<std::vector<std::string>> rows;
  std::string line;
  while (std::getline(f, line)) {
    if (line.empty() || line[0] == '#') continue;
    std::vector<std::string> cols;
    std::stringstream s(line);
    std::string c;
    while (std::getline(s, c, '\t')) cols.push_back(c);
    rows.push_back(cols);
  }
  return rows;
}

std::vector<uint8_t> LcgBytes(size_t n, uint32_t seed) {
  std::vector<uint8_t> a(n);
  uint32_t s = seed;
  for (size_t i = 0; i < n; ++i) {
    s = s * 1664525u + 1013904223u;
    a[i] = static_cast<uint8_t>(s >> 24);
  }
  return a;
}

double ParseJsNumber(const std::string& t) {
  if (t == "NaN" || t == "undefined") return std::nan("");
  return std::strtod(t.c_str(), nullptr);
}

// --- SKY-G0: the pure functions against the original JS --------------------------------------

TEST(SkyJsGolden, PixelTransformsMatchTheOriginalSourceExecutedInNode) {
  const auto rows = ReadTsv("preprocess_cases.tsv");
  ASSERT_EQ(rows.size(), 36u);
  int checked = 0;
  for (const auto& r : rows) {
    const std::string fn = r[0];
    const int sw = std::stoi(r[1]), sh = std::stoi(r[2]), dw = std::stoi(r[3]), dh = std::stoi(r[4]);
    const uint32_t seed = static_cast<uint32_t>(std::stoul(r[5]));
    const auto src = LcgBytes(static_cast<size_t>(sw) * sh * 4, seed);
    std::vector<uint8_t> dst(static_cast<size_t>(dw) * dh * 4, 0);
    if (fn == "toImageArrayLetterbox") ToImageArrayLetterbox(src.data(), sw, sh, dw, dh, dst.data());
    else if (fn == "toImageArrayRotateCW90") ToImageArrayRotateCW90(src.data(), sw, sh, dw, dh, dst.data());
    else if (fn == "toImageArrayRotateCCW90") ToImageArrayRotateCCW90(src.data(), sw, sh, dw, dh, dst.data());
    else FAIL() << fn;
    EXPECT_EQ(Sha256Hex(dst), r[6]) << fn << " " << sw << "x" << sh << " -> " << dw << "x" << dh;
    ++checked;
  }
  EXPECT_EQ(checked, 36);
}

TEST(SkyJsGolden, GetQuaternionMatchesTheOriginalWithin3UlpOfDouble) {
  const auto rows = ReadTsv("quaternion_cases.tsv");
  ASSERT_EQ(rows.size(), 12u);
  for (const auto& r : rows) {
    const QuaternionXyzw q = GetQuaternion(ParseJsNumber(r[0]), ParseJsNumber(r[1]), ParseJsNumber(r[2]));
    const double ex[4] = {ParseJsNumber(r[3]), ParseJsNumber(r[4]), ParseJsNumber(r[5]), ParseJsNumber(r[6])};
    const double got[4] = {q.x, q.y, q.z, q.w};
    for (int i = 0; i < 4; ++i) {
      // 1 ulp of double near 1 is 2.2e-16; libm rounds cos/sin differently in the last bit.
      EXPECT_NEAR(got[i], ex[i], 1e-15) << r[0] << "," << r[1] << "," << r[2] << " component " << i;
    }
  }
}

TEST(SkyJsGolden, ScaleFactorIsTheSameDouble) {
  for (const auto& r : ReadTsv("scalefactor_cases.tsv")) {
    const double got = ScaleFactorSameAspectRatioCrop(std::stod(r[0]), std::stod(r[1]),
                                                     std::stod(r[2]), std::stod(r[3]));
    EXPECT_EQ(got, std::strtod(r[4].c_str(), nullptr)) << r[0] << "x" << r[1];
  }
}

TEST(SkyPreprocess, PortraitOutputSizeIsTheModelInputAndKeepsJsDoubleArithmetic) {
  const ModelInputGeometry g = CalculateOutputSize(480, 640);
  EXPECT_EQ(g.model_input_width, 144);
  EXPECT_EQ(g.model_input_height, 256);
  EXPECT_EQ(g.video_resize_width, 0.4 * 480);   // 192.00000000000003 in JS doubles
  EXPECT_EQ(g.video_resize_height, 0.4 * 640);  // 256
  const ModelInputGeometry l = CalculateOutputSize(640, 480);
  EXPECT_EQ(l.model_input_width, 256);
  EXPECT_EQ(l.model_input_height, 144);
}

TEST(SkyPreprocess, CropDimensionByAspectRatioMatchesTheOriginalCases) {
  int w = 0, h = 0;
  CropDimensionByAspectRatio(480, 640, 144.0f / 256.0f, &w, &h);  // layers-controller.cc:98
  EXPECT_EQ(w, 360);
  EXPECT_EQ(h, 640);
  // PUB:c8/pixels/pixel-transforms-test.cc:1883-1888.
  CropDimensionByAspectRatio(480, 640, 9.0 / 16.0, &w, &h);
  EXPECT_EQ(w, 360);
  EXPECT_EQ(h, 640);
  CropDimensionByAspectRatio(480, 640, 1.0, &w, &h);
  EXPECT_EQ(w, 480);
  EXPECT_EQ(h, 480);
}

TEST(SkyPreprocess, ResizerKeepsRowOrderAndCentreCrop) {
  // A horizontal ramp: output column x (144 wide) samples camera columns of the centre 360 px of a
  // 480 px wide frame (viewport 192 wide, offset -24).
  Rgba8Image cam(480, 640);
  for (int y = 0; y < 640; ++y) {
    for (int x = 0; x < 480; ++x) {
      uint8_t* p = cam.at(x, y);
      p[0] = static_cast<uint8_t>(x * 255 / 479);
      p[1] = static_cast<uint8_t>(y * 255 / 639);
      p[2] = 7;
      p[3] = 255;
    }
  }
  const Rgba8Image out = ResizeCameraToModelInput(cam, 144, 256, -24, 0, 192, 256);
  ASSERT_EQ(out.width, 144);
  ASSERT_EQ(out.height, 256);
  // Row order is preserved (row 0 shows camera row 0, no flip) and columns are the centre crop.
  EXPECT_LT(out.at(70, 0)[1], 3);
  EXPECT_GT(out.at(70, 255)[1], 252);
  EXPECT_GT(out.at(143, 100)[0], out.at(0, 100)[0]);
  EXPECT_NEAR(out.at(0, 100)[0], 255.0 * (24.5 / 192.0 * 480 - 0.5) / 479.0 + 0.0, 2.0);
  EXPECT_EQ(out.at(5, 5)[2], 7);
  EXPECT_EQ(out.at(5, 5)[3], 255);
}

TEST(SkyPreprocess, Unorm8RoundsToNearestAndClamps) {
  EXPECT_EQ(ToUnorm8(0.0f), 0);
  EXPECT_EQ(ToUnorm8(1.0f), 255);
  EXPECT_EQ(ToUnorm8(-0.3f), 0);
  EXPECT_EQ(ToUnorm8(7.0f), 255);
  EXPECT_EQ(ToUnorm8(0.1f), 26);  // 25.500000382 rounds up: the "max(0.1, b)" first-seen value
  EXPECT_EQ(ToUnorm8(128.0f / 255.0f), 128);
}

// --- the CPU cubemap renderer ---------------------------------------------------------------

// A renderer set up exactly like the controller does for a 480x640 video and default intrinsics.
struct RendererFixture {
  SkyCubemapRenderer renderer;
  c8_PixelPinholeCameraModel k;
  RendererFixture() {
    k = c8::Intrinsics::rotateCropAndScaleIntrinsics(DefaultDeviceIntrinsics(), 480, 640);
    renderer.SetDeviceOrientation(0);
    renderer.InitOutputSemanticsScene(k, 192, 256);
    int cw = 0, ch = 0;
    CropDimensionByAspectRatio(480, 640, 144.0f / 256.0f, &cw, &ch);
    renderer.SetProjectionIntrinsics(c8::Intrinsics::rotateCropAndScaleIntrinsics(k, cw, ch));
    renderer.SetSkyPostProcessParameters(0.0f, true, false);
  }
  static c8::HMatrix Extrinsics(double alpha, double beta, double gamma) {
    const QuaternionXyzw q = GetQuaternion(alpha, beta, gamma);
    return c8::xrRotationFromDeviceRotation(
               c8::Quaternion(static_cast<float>(q.w), static_cast<float>(q.x), static_cast<float>(q.y),
                              static_cast<float>(q.z)))
        .toRotationMat();
  }
  static Rgba8Image Mask(uint8_t value) {
    Rgba8Image m(kModelInputWidth, kModelInputHeight);
    for (size_t i = 0; i < m.px.size(); i += 4) {
      m.px[i] = m.px[i + 1] = m.px[i + 2] = value;
      m.px[i + 3] = 255;
    }
    return m;
  }
};

TEST(SkyCubemap, InitialFacesAreTheOriginalConstructorState) {
  SkyCubemapRenderer r;
  // +Y: solid yellow, -Y: opaque black, side faces: transparent black until rendered.
  const Rgba8Image& py = r.cubemap_face(2);
  EXPECT_EQ(py.at(0, 0)[0], 255);
  EXPECT_EQ(py.at(10, 20)[1], 255);
  EXPECT_EQ(py.at(255, 255)[2], 0);
  EXPECT_EQ(py.at(7, 7)[3], 255);
  const Rgba8Image& ny = r.cubemap_face(3);
  EXPECT_EQ(ny.at(3, 3)[0], 0);
  EXPECT_EQ(ny.at(3, 3)[3], 255);
  for (int f : {0, 1, 4, 5}) {
    for (size_t i = 0; i < r.cubemap_face(f).px.size(); i += 4097) {
      EXPECT_EQ(r.cubemap_face(f).px[i], 0) << "face " << f;
    }
  }
}

TEST(SkyCubemap, PingPongWritesTheOtherSetAndFacesFollowTheFrustum) {
  RendererFixture fx;
  fx.renderer.UpdateSrcTexture(RendererFixture::Mask(255));
  // Looking along the horizon (beta 90, portrait upright): the frustum covers one side face.
  const auto ext = RendererFixture::Extrinsics(0, 90, 0);
  EXPECT_EQ(fx.renderer.cube_tex_sampler_index(), 0);
  fx.renderer.RenderToCubemap(ext);
  EXPECT_EQ(fx.renderer.cube_tex_sampler_index(), 1);
  const int faces = fx.renderer.last_rendered_faces();
  EXPECT_NE(faces, 0);
  EXPECT_EQ(faces & (1 << 3), 0) << "negative Y is never rendered";
  fx.renderer.RenderToCubemap(ext);
  EXPECT_EQ(fx.renderer.cube_tex_sampler_index(), 0);
  EXPECT_EQ(fx.renderer.last_rendered_faces(), faces);
  // A written face now holds sky-marked pixels (b > 0) and an untouched one does not.
  int written = -1, untouched = -1;
  for (int f : {0, 1, 4, 5}) {
    ((faces >> f) & 1 ? written : untouched) = f;
  }
  ASSERT_NE(written, -1);
  bool any_marked = false;
  for (size_t i = 2; i < fx.renderer.cubemap_face(written).px.size(); i += 4) {
    any_marked = any_marked || fx.renderer.cubemap_face(written).px[i] == 255;
  }
  EXPECT_TRUE(any_marked);
  if (untouched != -1) {
    EXPECT_EQ(fx.renderer.cubemap_face(untouched).px[0], 0);
  }
}

TEST(SkyCubemap, YawByNinetyDegreesMovesTheRenderedFaceToAnAdjacentSideFace) {
  RendererFixture a, b;
  a.renderer.UpdateSrcTexture(RendererFixture::Mask(200));
  b.renderer.UpdateSrcTexture(RendererFixture::Mask(200));
  a.renderer.RenderToCubemap(RendererFixture::Extrinsics(0, 90, 0));
  b.renderer.RenderToCubemap(RendererFixture::Extrinsics(90, 90, 0));
  const int fa = a.renderer.last_rendered_faces();
  const int fb = b.renderer.last_rendered_faces();
  EXPECT_NE(fa, fb) << "metamorphic: a 90 degree yaw must select different faces";
  EXPECT_EQ(fa & (1 << 2), 0) << "horizontal view does not reach +Y from the portrait crop";
}

TEST(SkyCubemap, SkyShaderMetamorphicProperties) {
  const auto ext = RendererFixture::Extrinsics(0, 90, 0);
  auto draw = [&](float smooth, bool seen, bool flip) {
    RendererFixture fx;
    fx.renderer.SetSkyPostProcessParameters(smooth, seen, flip);
    fx.renderer.UpdateSrcTexture(RendererFixture::Mask(255));
    for (int i = 0; i < 3; ++i) fx.renderer.RenderToCubemap(ext);
    return fx.renderer.DrawOutputSemantics(ext, 192, 256);
  };
  const Rgba8Image plain = draw(0.0f, true, false);
  const Rgba8Image flipped = draw(0.0f, true, true);
  ASSERT_EQ(plain.px.size(), flipped.px.size());
  int sky_pixels = 0;
  for (size_t i = 0; i < plain.px.size(); i += 4) {
    // invertLayerMask: r, g -> 1 - r, 1 - g; b (isMarked) and a unchanged.
    EXPECT_EQ(static_cast<int>(plain.px[i]) + flipped.px[i], 255);
    EXPECT_EQ(static_cast<int>(plain.px[i + 1]) + flipped.px[i + 1], 255);
    EXPECT_EQ(plain.px[i + 2], flipped.px[i + 2]);
    EXPECT_EQ(plain.px[i + 3], 255);
    sky_pixels += plain.px[i] > 200;
  }
  EXPECT_GT(sky_pixels, 1000) << "a full sky mask must produce a sky region";
  // edgeSmoothness changes the result only through the blurred edge term.
  const Rgba8Image smooth = draw(1.0f, true, false);
  EXPECT_NE(plain.px, smooth.px);
  // haveSeenSky: unknown pixels above the horizon turn on (seen) or off (not seen).
  const Rgba8Image unseen = draw(0.0f, false, false);
  EXPECT_NE(plain.px, unseen.px);
}

TEST(SkyCubemap, OutputIsTheRequestedTextureSizeAndOpaque) {
  RendererFixture fx;
  fx.renderer.UpdateSrcTexture(RendererFixture::Mask(10));
  const auto ext = RendererFixture::Extrinsics(10, 80, 0);
  fx.renderer.RenderToCubemap(ext);
  const Rgba8Image t = fx.renderer.DrawOutputSemantics(ext, 192, 256);
  EXPECT_EQ(t.width, 192);
  EXPECT_EQ(t.height, 256);
  for (size_t i = 3; i < t.px.size(); i += 4) {
    ASSERT_EQ(t.px[i], 255);
  }
}

// --- LayersController -----------------------------------------------------------------------

struct ControllerFixture {
  SkyPipeline pipe;
  SkyEnvironment env;
  Rgba8Image cam;
  ControllerFixture() {
    env.video_width = 480;
    env.video_height = 640;
    env.canvas_width = 480;
    env.canvas_height = 640;
    cam = Rgba8Image(480, 640);
    for (size_t i = 0; i < cam.px.size(); i += 4) {
      cam.px[i] = 90;
      cam.px[i + 1] = 140;
      cam.px[i + 2] = 230;
      cam.px[i + 3] = 255;
    }
    ConfigureArgs a;
    a.sky.present = true;
    a.sky.has_smoothness = true;
    a.sky.smoothness = 0.5;
    pipe.controller().Configure(a);
    pipe.Start(env);
  }
  // A fake worker: every pixel scores `score`.
  static SkyWorkerFn Fake(float score, int* calls) {
    return [score, calls](const SkyWorkerRequest&, SkyWorkerAnswer* ans) {
      ++*calls;
      ans->data.assign(static_cast<size_t>(kModelInputWidth) * kModelInputHeight, score);
      size_t n = 0;
      for (float v : ans->data) n += static_cast<double>(v) > kModelThreshold;
      ans->percentage = static_cast<double>(n) / static_cast<double>(ans->data.size());
      return true;
    };
  }
};

std::vector<std::string> Names(const std::vector<SkyEvent>& ev) {
  std::vector<std::string> n;
  for (const auto& e : ev) n.push_back(e.name);
  return n;
}

TEST(SkyController, EventsAreLoadingScanningFoundOnceWithThePercentage) {
  ControllerFixture fx;
  int calls = 0;
  const auto worker = ControllerFixture::Fake(0.9f, &calls);
  DeviceOrientation o{0, 90, 0};
  fx.pipe.ProcessFrame(fx.cam, false, o, worker);  // worker not started: no request
  EXPECT_EQ(calls, 0);
  fx.pipe.WorkerStarted();
  for (int i = 0; i < 4; ++i) fx.pipe.ProcessFrame(fx.cam, false, o, worker);
  const auto ev = fx.pipe.controller().TakeEvents();
  ASSERT_EQ(Names(ev), (std::vector<std::string>{"layerloading", "layerscanning", "layerfound"}));
  EXPECT_EQ(ev[0].detail_json, "{}");
  EXPECT_EQ(ev[1].detail_json, "{\"name\":\"sky\"}");
  EXPECT_EQ(ev[2].detail_json, "{\"name\":\"sky\",\"percentage\":1}");
  EXPECT_TRUE(fx.pipe.controller().have_seen_sky());
  EXPECT_EQ(calls, 4);
}

TEST(SkyController, NoLayerFoundBelowFivePercent) {
  ControllerFixture fx;
  int calls = 0;
  const auto worker = ControllerFixture::Fake(0.2f, &calls);  // below the 0.4 pixel threshold
  fx.pipe.WorkerStarted();
  DeviceOrientation o{0, 90, 0};
  for (int i = 0; i < 4; ++i) fx.pipe.ProcessFrame(fx.cam, false, o, worker);
  EXPECT_EQ(Names(fx.pipe.controller().TakeEvents()),
            (std::vector<std::string>{"layerloading", "layerscanning"}));
  EXPECT_FALSE(fx.pipe.controller().have_seen_sky());
}

TEST(SkyController, WorkerIsInvokedOnlyWhenReadyAndOutputAppearsOnTheSecondStagedFrame) {
  ControllerFixture fx;
  fx.pipe.WorkerStarted();
  DeviceOrientation o{0, 90, 0};
  int requests = 0;
  // Frame 0: staged, no previous resized frame, so no invokeData and no request. Frame 1: the
  // pipeline queue has two entries now, the first response exists.
  SkyWorkerFn counting = [&](const SkyWorkerRequest&, SkyWorkerAnswer*) {
    ++requests;
    return false;  // never answers: isInvokeReady_ stays false
  };
  LayersFrameOutput f0 = fx.pipe.ProcessFrame(fx.cam, false, o, counting);
  EXPECT_FALSE(f0.valid);
  EXPECT_EQ(requests, 0);
  LayersFrameOutput f1 = fx.pipe.ProcessFrame(fx.cam, false, o, counting);
  EXPECT_TRUE(f1.valid);
  EXPECT_EQ(requests, 1);
  fx.pipe.ProcessFrame(fx.cam, false, o, counting);
  fx.pipe.ProcessFrame(fx.cam, false, o, counting);
  EXPECT_EQ(requests, 1) << "no new request until the worker answered the first one";
  // Repeat frames do nothing in onProcessGpu and do not post requests.
  const auto before = requests;
  fx.pipe.ProcessFrame(fx.cam, true, o, counting);
  EXPECT_EQ(requests, before);
}

TEST(SkyController, ResponseCameraHasTheDefaultProjectionAndYawCorrectedRotation) {
  ControllerFixture fx;
  int calls = 0;
  const auto worker = ControllerFixture::Fake(0.9f, &calls);
  fx.pipe.WorkerStarted();
  LayersFrameOutput out;
  for (int i = 0; i < 4; ++i) {
    out = fx.pipe.ProcessFrame(fx.cam, false, DeviceOrientation{37, 90, 0}, worker);
  }
  ASSERT_TRUE(out.valid);
  // Intrinsics::toClipSpace of cropAndScale(rotateCropAndScale(default K, 480, 640), 480, 640) with
  // near 0.01 / far 1000: 2 f / w = 2 * 506.4104 / 480.
  EXPECT_NEAR(out.intrinsics[0], 2.0f * 506.4104f / 480.0f, 1e-6f);
  EXPECT_NEAR(out.intrinsics[5], 2.0f * 506.4104f / 640.0f * 1.0f, 1e-6f);
  EXPECT_NEAR(out.intrinsics[10], (1000.0f + 0.01f) / (0.01f - 1000.0f), 1e-6f);
  EXPECT_EQ(out.intrinsics[11], -1.0f);
  // Origin default (0, 2, 0): the camera position is the origin; the yaw offset makes the first
  // frame face forward whatever the device's alpha was, so the rotation is a pure ground-plane
  // rotation of the origin (identity) times the pitch of the device.
  EXPECT_NEAR(out.position.x, 0.0, 1e-6);
  EXPECT_NEAR(out.position.y, 2.0, 1e-6);
  EXPECT_NEAR(out.position.z, 0.0, 1e-6);
  const double n = out.rotation.x * out.rotation.x + out.rotation.y * out.rotation.y +
                   out.rotation.z * out.rotation.z + out.rotation.w * out.rotation.w;
  EXPECT_NEAR(n, 1.0, 1e-5);
  // Same pitch, different alpha -> the same response rotation (yaw is removed at the first frame).
  ControllerFixture fx2;
  const auto worker2 = ControllerFixture::Fake(0.9f, &calls);
  fx2.pipe.WorkerStarted();
  LayersFrameOutput out2;
  for (int i = 0; i < 4; ++i) {
    out2 = fx2.pipe.ProcessFrame(fx2.cam, false, DeviceOrientation{211, 90, 0}, worker2);
  }
  ASSERT_TRUE(out2.valid);
  // q and -q are the same rotation: compare |dot| with 1.
  const double dot = out.rotation.x * out2.rotation.x + out.rotation.y * out2.rotation.y +
                     out.rotation.z * out2.rotation.z + out.rotation.w * out2.rotation.w;
  EXPECT_NEAR(std::fabs(dot), 1.0, 1e-5);
}

TEST(SkyController, ConfigureReducerFollowsNullAndUndefinedSemantics) {
  LayersController c;
  EXPECT_TRUE(c.GetLayerNames().empty());
  ConfigureArgs add;
  add.sky.present = true;
  c.Configure(add);
  EXPECT_EQ(c.GetLayerNames(), (std::vector<std::string>{"sky"}));
  ConfigureArgs rm;
  rm.sky.present = true;
  rm.sky.remove = true;
  c.Configure(rm);
  EXPECT_TRUE(c.GetLayerNames().empty());
  c.Configure(add);
  ConfigureArgs all_null;
  all_null.layers_null = true;
  c.Configure(all_null);
  EXPECT_TRUE(c.GetLayerNames().empty());
}

TEST(SkyController, InvertAndSmoothnessReachTheOutputTexture) {
  auto run = [](bool invert, double smoothness) {
    ControllerFixture fx;
    ConfigureArgs a;
    a.sky.present = true;
    a.sky.has_invert = true;
    a.sky.invert = invert;
    a.sky.has_smoothness = true;
    a.sky.smoothness = smoothness;
    fx.pipe.controller().Configure(a);
    int calls = 0;
    const auto worker = ControllerFixture::Fake(0.95f, &calls);
    fx.pipe.WorkerStarted();
    LayersFrameOutput out;
    for (int i = 0; i < 5; ++i) out = fx.pipe.ProcessFrame(fx.cam, false, DeviceOrientation{0, 90, 0}, worker);
    return out;
  };
  const LayersFrameOutput normal = run(false, 0.0);
  const LayersFrameOutput inverted = run(true, 0.0);
  ASSERT_TRUE(normal.valid && inverted.valid);
  ASSERT_EQ(normal.sky.texture.px.size(), inverted.sky.texture.px.size());
  for (size_t i = 0; i < normal.sky.texture.px.size(); i += 4) {
    ASSERT_EQ(static_cast<int>(normal.sky.texture.px[i]) + inverted.sky.texture.px[i], 255);
  }
  EXPECT_EQ(normal.sky.texture_width, 192);
  EXPECT_EQ(normal.sky.texture_height, 256);
  EXPECT_NEAR(normal.sky.percentage, 1.0, 1e-12);
}

TEST(SkyController, RecenterRecomputesTheYawOffsetFromTheNextDrawnFrame) {
  ControllerFixture fx;
  int calls = 0;
  const auto worker = ControllerFixture::Fake(0.9f, &calls);
  fx.pipe.WorkerStarted();
  // Frames f1, f2 at alpha 20 and f3 at alpha 200. The pipeline draws the frame staged one call
  // earlier: the first response (at f2) belongs to f1.
  fx.pipe.ProcessFrame(fx.cam, false, DeviceOrientation{20, 90, 0}, worker);
  const LayersFrameOutput first = fx.pipe.ProcessFrame(fx.cam, false, DeviceOrientation{20, 90, 0}, worker);
  const LayersFrameOutput third = fx.pipe.ProcessFrame(fx.cam, false, DeviceOrientation{200, 90, 0}, worker);
  ASSERT_TRUE(first.valid && third.valid);
  auto dot = [](const QuaternionXyzw& a, const QuaternionXyzw& b) {
    return std::fabs(a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w);
  };
  // f2 (alpha 20) was drawn at f3 with the offset taken from f1: still facing forward.
  EXPECT_NEAR(dot(first.rotation, third.rotation), 1.0, 1e-5);
  // Without a recenter the next draw (f3, alpha 200) is 180 degrees away from forward.
  ControllerFixture keep;
  const auto w2 = ControllerFixture::Fake(0.9f, &calls);
  keep.pipe.WorkerStarted();
  keep.pipe.ProcessFrame(keep.cam, false, DeviceOrientation{20, 90, 0}, w2);
  const LayersFrameOutput k1 = keep.pipe.ProcessFrame(keep.cam, false, DeviceOrientation{20, 90, 0}, w2);
  keep.pipe.ProcessFrame(keep.cam, false, DeviceOrientation{200, 90, 0}, w2);
  const LayersFrameOutput k4 = keep.pipe.ProcessFrame(keep.cam, false, DeviceOrientation{200, 90, 0}, w2);
  ASSERT_TRUE(k1.valid && k4.valid);
  EXPECT_LT(dot(k1.rotation, k4.rotation), 0.1);
  // With recenter() before the 4th frame, the offset is recomputed from the frame drawn then (f3,
  // alpha 200), which is therefore "forward" again.
  fx.pipe.controller().Recenter();
  const LayersFrameOutput after = fx.pipe.ProcessFrame(fx.cam, false, DeviceOrientation{200, 90, 0}, worker);
  ASSERT_TRUE(after.valid);
  EXPECT_NEAR(dot(first.rotation, after.rotation), 1.0, 1e-5);
}

TEST(SkyController, PercentageJsonUsesTheShortestRoundTripLikeJsonStringify) {
  ControllerFixture fx;
  fx.pipe.WorkerStarted();
  // 0.5177137586805556 is the percentage of a recorded original run (documented in the spec).
  const size_t n = static_cast<size_t>(kModelInputWidth) * kModelInputHeight;
  const size_t k = static_cast<size_t>(std::llround(0.5177137586805556 * static_cast<double>(n)));
  SkyWorkerFn w = [&](const SkyWorkerRequest&, SkyWorkerAnswer* a) {
    a->data.assign(n, 0.0f);
    for (size_t i = 0; i < k; ++i) a->data[i] = 1.0f;
    a->percentage = static_cast<double>(k) / static_cast<double>(n);
    return true;
  };
  for (int i = 0; i < 3; ++i) fx.pipe.ProcessFrame(fx.cam, false, DeviceOrientation{0, 90, 0}, w);
  const auto ev = fx.pipe.controller().TakeEvents();
  ASSERT_EQ(ev.size(), 3u);
  EXPECT_EQ(ev[2].detail_json, "{\"name\":\"sky\",\"percentage\":0.5177137586805556}");
}

}  // namespace
}  // namespace slam_refact::sky

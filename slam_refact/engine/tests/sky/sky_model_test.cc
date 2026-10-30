// Sky model tests (label mlmodel; needs assets/references/xr_reference/engine/resources/semantics-model.tflite,
// never copied into the repository). Synthetic inputs: properties of the pipeline, not accuracy.
#include <gtest/gtest.h>

#include <cstdint>
#include <string>
#include <vector>

#include "sky/sky_model.h"
#include "sky/sky_pipeline.h"

namespace slam_refact::sky {
namespace {

std::vector<uint8_t> LoadModel() {
  std::vector<uint8_t> bytes;
  std::string err;
  EXPECT_TRUE(ReadFileBytes(std::string(SR_REPO_ROOT) + "/" + kSemanticsModelPath, &bytes, &err)) << err;
  return bytes;
}

std::vector<uint8_t> SkyLikeImage(uint32_t seed) {
  std::vector<uint8_t> img(static_cast<size_t>(kModelInputWidth) * kModelInputHeight * 4);
  uint32_t s = seed;
  for (int y = 0; y < kModelInputHeight; ++y) {
    for (int x = 0; x < kModelInputWidth; ++x) {
      s = s * 1664525u + 1013904223u;
      const int noise = static_cast<int>(s >> 28) - 8;
      uint8_t* p = &img[(static_cast<size_t>(y) * kModelInputWidth + x) * 4];
      if (y < kModelInputHeight / 2) {
        p[0] = static_cast<uint8_t>(80 + y / 3 + noise);
        p[1] = static_cast<uint8_t>(130 + y / 3 + noise);
        p[2] = static_cast<uint8_t>(220 + noise);
      } else {
        p[0] = static_cast<uint8_t>(60 + noise);
        p[1] = static_cast<uint8_t>(105 + noise);
        p[2] = static_cast<uint8_t>(50 + noise);
      }
      p[3] = 255;
    }
  }
  return img;
}

TEST(SkyModelRun, ModelBytesAreTheDeployedModel) {
  const auto bytes = LoadModel();
  EXPECT_EQ(Sha256Hex(bytes), kSemanticsModelSha256);
  SkyModel m;
  std::string err;
  std::vector<uint8_t> wrong = bytes;
  wrong[bytes.size() / 2] ^= 1;
  EXPECT_FALSE(m.Load(wrong, true, &err));
  EXPECT_NE(err.find("sha256"), std::string::npos);
  EXPECT_FALSE(m.loaded());
  SkyInference inf;
  EXPECT_FALSE(m.Invoke(SkyLikeImage(1).data(), &inf)) << "Invoke before Load must fail";
  ASSERT_TRUE(m.Load(bytes, true, &err)) << err;
  EXPECT_TRUE(m.loaded());
}

TEST(SkyModelRun, OutputIsADeterministicScorePlaneAndPercentageFollowsTheWorkerRule) {
  SkyModel m;
  std::string err;
  ASSERT_TRUE(m.Load(LoadModel(), true, &err)) << err;
  const auto img = SkyLikeImage(3);
  SkyInference a, b;
  ASSERT_TRUE(m.Invoke(img.data(), &a));
  ASSERT_TRUE(m.Invoke(img.data(), &b));
  ASSERT_EQ(a.data.size(), static_cast<size_t>(kModelInputWidth) * kModelInputHeight);
  EXPECT_EQ(a.data, b.data) << "same input, same process: bit identical";
  size_t above = 0;
  for (float v : a.data) {
    ASSERT_GE(v, 0.0f);
    ASSERT_LE(v, 1.0f);
    above += static_cast<double>(v) > 0.4;
  }
  EXPECT_EQ(a.percentage, static_cast<double>(above) / static_cast<double>(a.data.size()));
}

TEST(SkyModelRun, PipelineEmitsTheLayerEventsWithTheBuiltInWorker) {
  SkyPipeline pipe;
  std::string err;
  ASSERT_TRUE(pipe.LoadModel(LoadModel(), true, &err)) << err;
  SkyEnvironment env;
  env.video_width = 480;
  env.video_height = 640;
  env.canvas_width = 480;
  env.canvas_height = 640;
  ConfigureArgs args;
  args.sky.present = true;
  pipe.controller().Configure(args);
  pipe.Start(env);
  pipe.WorkerStarted();
  Rgba8Image cam(480, 640);
  const auto base = SkyLikeImage(9);
  for (int y = 0; y < 640; ++y) {
    for (int x = 0; x < 480; ++x) {
      const uint8_t* s = &base[(static_cast<size_t>(y * 256 / 640) * kModelInputWidth + x * 144 / 480) * 4];
      std::copy(s, s + 4, cam.at(x, y));
    }
  }
  LayersFrameOutput out;
  for (int i = 0; i < 6; ++i) out = pipe.ProcessFrame(cam, false, DeviceOrientation{0, 90, 0});
  ASSERT_TRUE(out.valid);
  EXPECT_EQ(out.sky.texture_width, 192);
  EXPECT_EQ(out.sky.texture_height, 256);
  const auto ev = pipe.controller().TakeEvents();
  ASSERT_GE(ev.size(), 2u);
  EXPECT_EQ(ev[0].name, "layerloading");
  EXPECT_EQ(ev[1].name, "layerscanning");
  EXPECT_GE(out.sky.percentage, 0.0);
  EXPECT_LE(out.sky.percentage, 1.0);
}

}  // namespace
}  // namespace slam_refact::sky

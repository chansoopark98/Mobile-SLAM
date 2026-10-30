// Per-stage timing of the sky pipeline on synthetic camera frames (docs/spec/09-sky.md §7). Native
// and WASM (node). Desktop numbers, not phone numbers.
//
//   sr_sky_bench --repo <repo root> [--iters 100] [--warmup 30] [--json <file>]
//
// Stages (per processed frame): resize (CPU model of the GPU camera resizer, 480x640 -> 144x256),
// model (SemanticsClassifier::obtainSemantics: input normalisation + Invoke + class-0 plane),
// cubemap (RenderToCubemap of the answer), output (DrawOutputSemantics: cubemap render, blur, sky
// shader), total (LayersController + model + cubemap as SkyPipeline::ProcessFrame runs them).
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <iostream>
#include <string>
#include <utility>
#include <vector>

#include "c8/geometry/device-pose.h"
#include "c8/geometry/intrinsics-clip.h"
#include "sky/sky_pipeline.h"
#include "sky/sky_preprocess.h"

using namespace slam_refact::sky;
using Clock = std::chrono::steady_clock;

namespace {

Rgba8Image SyntheticCamera(int w, int h, uint32_t seed) {
  Rgba8Image img(w, h);
  uint32_t s = seed;
  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w; ++x) {
      s = s * 1664525u + 1013904223u;
      const int noise = static_cast<int>(s >> 28) - 8;
      uint8_t* p = img.at(x, y);
      if (y < h * 52 / 100) {
        const double t = static_cast<double>(y) / (h * 0.52);
        p[0] = static_cast<uint8_t>(std::min(255.0, std::max(0.0, 70 + 120 * t + noise)));
        p[1] = static_cast<uint8_t>(std::min(255.0, std::max(0.0, 120 + 95 * t + noise)));
        p[2] = static_cast<uint8_t>(std::min(255.0, std::max(0.0, 215 + 30 * t + noise)));
      } else {
        p[0] = static_cast<uint8_t>(std::min(255, std::max(0, 60 + noise * 3)));
        p[1] = static_cast<uint8_t>(std::min(255, std::max(0, 110 + noise * 3)));
        p[2] = static_cast<uint8_t>(std::min(255, std::max(0, 45 + noise * 3)));
      }
      p[3] = 255;
    }
  }
  return img;
}

struct Stat {
  explicit Stat(std::string n) : name(std::move(n)) {}
  std::string name;
  std::vector<double> ms;
  double Pct(double q) const {
    std::vector<double> v = ms;
    std::sort(v.begin(), v.end());
    if (v.empty()) return 0;
    // nearest-rank
    size_t idx = static_cast<size_t>(std::ceil(q * static_cast<double>(v.size()))) - 1;
    return v[std::min(idx, v.size() - 1)];
  }
};

double Ms(Clock::time_point a, Clock::time_point b) {
  return std::chrono::duration<double, std::milli>(b - a).count();
}

}  // namespace

int main(int argc, char** argv) {
  std::string repo, json_path;
  int iters = 100, warmup = 30;
  for (int i = 1; i + 1 < argc; i += 2) {
    const std::string a = argv[i];
    if (a == "--repo") repo = argv[i + 1];
    else if (a == "--iters") iters = std::stoi(argv[i + 1]);
    else if (a == "--warmup") warmup = std::stoi(argv[i + 1]);
    else if (a == "--json") json_path = argv[i + 1];
    else {
      std::cerr << "unknown argument " << a << "\n";
      return 2;
    }
  }
  if (repo.empty()) {
    std::cerr << "usage: sr_sky_bench --repo <root> [--iters N] [--warmup N] [--json file]\n";
    return 2;
  }
  std::vector<uint8_t> bytes;
  std::string err;
  if (!ReadFileBytes(repo + "/" + kSemanticsModelPath, &bytes, &err)) {
    std::cerr << err << "\n";
    return 2;
  }
  SkyPipeline pipe;
  if (!pipe.LoadModel(bytes, true, &err)) {
    std::cerr << err << "\n";
    return 2;
  }
  SkyEnvironment env;
  env.video_width = 480;
  env.video_height = 640;
  env.canvas_width = 480;
  env.canvas_height = 640;
  ConfigureArgs args;
  args.sky.present = true;
  args.sky.has_smoothness = true;
  args.sky.smoothness = 0.5;
  pipe.controller().Configure(args);
  pipe.Start(env);
  pipe.WorkerStarted();

  Stat resize("resize"), model("model"), cubemap("cubemap"), output("output"), total("total");
  const Rgba8Image cam = SyntheticCamera(480, 640, 7);
  const int geo_w = 192, geo_h = 256;
  (void)geo_w;
  (void)geo_h;
  // Stage timing on the same objects the pipeline uses.
  SkyCubemapRenderer renderer;
  {
    // Minimal renderer set-up identical to the controller's (default device intrinsics).
    const auto dev = DefaultDeviceIntrinsics();
    auto k = c8::Intrinsics::rotateCropAndScaleIntrinsics(dev, 480, 640);
    renderer.SetDeviceOrientation(0);
    renderer.InitOutputSemanticsScene(k, 192, 256);
    int cw = 0, ch = 0;
    CropDimensionByAspectRatio(480, 640, 144.0f / 256.0f, &cw, &ch);
    renderer.SetProjectionIntrinsics(c8::Intrinsics::rotateCropAndScaleIntrinsics(k, cw, ch));
    renderer.SetSkyPostProcessParameters(0.5f, true, false);
  }
  Rgba8Image small = ResizeCameraToModelInput(cam, 144, 256, -24, 0, 192, 256);
  SkyInference inf;
  for (int it = -warmup; it < iters; ++it) {
    const double beta = 70.0 + 40.0 * ((it + warmup) % 40) / 40.0;
    const auto q = GetQuaternion(0.0, beta, 0.0);
    const c8::HMatrix ext =
        c8::xrRotationFromDeviceRotation(c8::Quaternion(static_cast<float>(q.w), static_cast<float>(q.x),
                                                        static_cast<float>(q.y), static_cast<float>(q.z)))
            .toRotationMat();
    auto t0 = Clock::now();
    small = ResizeCameraToModelInput(cam, 144, 256, -24, 0, 192, 256);
    auto t1 = Clock::now();
    pipe.model().Invoke(small.px.data(), &inf);
    auto t2 = Clock::now();
    Rgba8Image mask(kModelInputWidth, kModelInputHeight);
    for (size_t i = 0; i < inf.data.size(); ++i) {
      const uint8_t c = static_cast<uint8_t>(std::min(255.0, inf.data[i] * 255.0));
      mask.px[4 * i] = mask.px[4 * i + 1] = mask.px[4 * i + 2] = c;
      mask.px[4 * i + 3] = 255;
    }
    renderer.UpdateSrcTexture(mask);
    auto t3 = Clock::now();
    renderer.RenderToCubemap(ext);
    auto t4 = Clock::now();
    const Rgba8Image tex = renderer.DrawOutputSemantics(ext, 192, 256);
    auto t5 = Clock::now();
    const DeviceOrientation o{0.0, beta, 0.0};
    const LayersFrameOutput out = pipe.ProcessFrame(cam, false, o);
    auto t6 = Clock::now();
    (void)tex;
    (void)out;
    if (it >= 0) {
      resize.ms.push_back(Ms(t0, t1));
      model.ms.push_back(Ms(t1, t2));
      cubemap.ms.push_back(Ms(t3, t4));
      output.ms.push_back(Ms(t4, t5));
      total.ms.push_back(Ms(t5, t6));
    }
  }
  std::string json = "{\n  \"iters\": " + std::to_string(iters) + ",\n  \"warmup\": " +
                     std::to_string(warmup) + ",\n  \"stages\": {\n";
  const std::vector<Stat*> stats = {&resize, &model, &cubemap, &output, &total};
  for (size_t i = 0; i < stats.size(); ++i) {
    char buf[256];
    std::snprintf(buf, sizeof(buf),
                  "    \"%s\": {\"p50\": %.4f, \"p95\": %.4f, \"p99\": %.4f, \"max\": %.4f}%s\n",
                  stats[i]->name.c_str(), stats[i]->Pct(0.5), stats[i]->Pct(0.95), stats[i]->Pct(0.99),
                  stats[i]->Pct(1.0), i + 1 < stats.size() ? "," : "");
    json += buf;
  }
  json += "  }\n}\n";
  std::cout << json;
  if (!json_path.empty()) {
    std::FILE* f = std::fopen(json_path.c_str(), "wb");
    if (f != nullptr) {
      std::fwrite(json.data(), 1, json.size(), f);
      std::fclose(f);
    }
  }
  return 0;
}

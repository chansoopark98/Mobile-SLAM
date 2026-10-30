// Per-stage timing of the hand pipeline on a synthetic camera frame (docs/spec/10-hand.md §6).
// Native and WASM (node). Desktop numbers, not phone numbers.
//
//   sr_hand_bench --repo <root> [--iters 100] [--warmup 30] [--json <file>] [--landmark 2d|3d]
//
// Stages: palm preprocess (FIT 256 + normalise), palm inference, palm postprocess (decode, NMS,
// letterbox, rects), landmark preprocess (rotated crop + FIT + normalise), landmark inference,
// landmark postprocess. The palm stages run on every iteration (RunPalmDetection), the landmark
// stages on the rect from the palm stage or, when nothing is detected in the synthetic frame, on a
// fixed central rect.
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <iostream>
#include <string>
#include <utility>
#include <vector>

#include "hand/hand_models.h"
#include "hand/hand_tracker.h"

using namespace slam_refact::hand;

namespace {

Image3 SyntheticFrame(int w, int h) {
  Image3 img(w, h);
  uint32_t s = 12345;
  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w; ++x) {
      s = s * 1664525u + 1013904223u;
      const int noise = static_cast<int>(s >> 28) - 8;
      uint8_t* p = img.at(x, y);
      const int v = static_cast<int>(120 + 70 * std::sin(0.05 * x) * std::cos(0.04 * y)) + noise;
      p[0] = static_cast<uint8_t>(std::min(255, std::max(0, v + 20)));
      p[1] = static_cast<uint8_t>(std::min(255, std::max(0, v)));
      p[2] = static_cast<uint8_t>(std::min(255, std::max(0, v - 20)));
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
    size_t idx = static_cast<size_t>(std::ceil(q * static_cast<double>(v.size()))) - 1;
    return v[std::min(idx, v.size() - 1)];
  }
};

}  // namespace

int main(int argc, char** argv) {
  std::string repo, json_path, kind = "3d";
  int iters = 100, warmup = 30;
  for (int i = 1; i + 1 < argc; i += 2) {
    const std::string a = argv[i];
    if (a == "--repo") repo = argv[i + 1];
    else if (a == "--iters") iters = std::stoi(argv[i + 1]);
    else if (a == "--warmup") warmup = std::stoi(argv[i + 1]);
    else if (a == "--json") json_path = argv[i + 1];
    else if (a == "--landmark") kind = argv[i + 1];
    else {
      std::cerr << "unknown argument " << a << "\n";
      return 2;
    }
  }
  if (repo.empty()) {
    std::cerr << "usage: sr_hand_bench --repo <root> [--iters N] [--warmup N] [--json file] [--landmark 2d|3d]\n";
    return 2;
  }
  std::vector<uint8_t> palm, landmark;
  std::string err;
  if (!ReadFileBytes(repo + "/" + kPalmDetectionPath, &palm, &err) ||
      !ReadFileBytes(repo + "/" + (kind == "2d" ? kHandLandmarkPath : kHandLandmark3dPath), &landmark, &err)) {
    std::cerr << err << "\n";
    return 2;
  }
  HandTracker tracker;
  if (!tracker.Load(palm, landmark, kind == "2d" ? LandmarkModel::k2d : LandmarkModel::k3d, true, &err)) {
    std::cerr << err << "\n";
    return 2;
  }
  const Image3 frame = SyntheticFrame(480, 640);
  Stat pp("palm_preprocess"), pi("palm_inference"), pq("palm_postprocess"), lp("landmark_preprocess"),
      li("landmark_inference"), lq("landmark_postprocess"), total("total");
  for (int it = -warmup; it < iters; ++it) {
    HandResult r;
    tracker.Reset();
    const auto t0 = std::chrono::steady_clock::now();
    if (!tracker.Process(frame, &r, &err)) {
      std::cerr << err << "\n";
      return 2;
    }
    const auto t1 = std::chrono::steady_clock::now();
    if (it >= 0) {
      pp.ms.push_back(r.times.palm_preprocess);
      pi.ms.push_back(r.times.palm_inference);
      pq.ms.push_back(r.times.palm_postprocess);
      lp.ms.push_back(r.times.landmark_preprocess);
      li.ms.push_back(r.times.landmark_inference);
      lq.ms.push_back(r.times.landmark_postprocess);
      total.ms.push_back(std::chrono::duration<double, std::milli>(t1 - t0).count());
    }
  }
  std::string json = "{\n  \"landmark_model\": \"" + kind + "\",\n  \"iters\": " + std::to_string(iters) +
                     ",\n  \"warmup\": " + std::to_string(warmup) + ",\n  \"stages\": {\n";
  const std::vector<Stat*> stats = {&pp, &pi, &pq, &lp, &li, &lq, &total};
  for (size_t i = 0; i < stats.size(); ++i) {
    char buf[256];
    std::snprintf(buf, sizeof(buf), "    \"%s\": {\"p50\": %.4f, \"p95\": %.4f, \"p99\": %.4f, \"max\": %.4f}%s\n",
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

// Runs the hand tracker on a directory of frames (docs/spec/10-hand.md §5).
//
//   sr_hand_run --repo <root> --frames <dir> --out <new file.jsonl> [--landmark 2d|3d] [--dump <new dir>] [--reset-each-frame 0|1]
//
// <dir>/meta.txt: "<width> <height> <count>", <dir>/frame-%04d.rgb: raw RGB8, row 0 = top.
// --dump writes, per frame k that ran the palm model: palm-boxes-k.f32 (2944 x 18), palm-scores-k.f32
// (2944 x 1) and palm-input-k.rgb (the 256x256 FIT input) for the independent post-processing oracle.
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>

#include "hand/hand_geometry.h"
#include "hand/hand_image.h"
#include "hand/hand_models.h"
#include "hand/hand_tracker.h"

namespace fs = std::filesystem;
using namespace slam_refact::hand;

int main(int argc, char** argv) {
  std::string repo, frames_dir, out_path, dump_dir, kind = "3d";
  bool reset_each_frame = false;
  for (int i = 1; i + 1 < argc; i += 2) {
    const std::string a = argv[i];
    if (a == "--repo") repo = argv[i + 1];
    else if (a == "--frames") frames_dir = argv[i + 1];
    else if (a == "--out") out_path = argv[i + 1];
    else if (a == "--landmark") kind = argv[i + 1];
    else if (a == "--dump") dump_dir = argv[i + 1];
    else if (a == "--reset-each-frame") reset_each_frame = std::string(argv[i + 1]) != "0";
    else {
      std::cerr << "unknown argument " << a << "\n";
      return 2;
    }
  }
  if (repo.empty() || frames_dir.empty() || out_path.empty()) {
    std::cerr << "usage: sr_hand_run --repo <root> --frames <dir> --out <new file> [--landmark 2d|3d] [--dump <new dir>] [--reset-each-frame 0|1]\n";
    return 2;
  }
  if (fs::exists(out_path) || (!dump_dir.empty() && fs::exists(dump_dir))) {
    std::cerr << "refusing to overwrite\n";
    return 2;
  }
  int w = 0, h = 0, count = 0;
  {
    std::ifstream m(frames_dir + "/meta.txt");
    if (!(m >> w >> h >> count)) {
      std::cerr << "cannot read meta.txt\n";
      return 2;
    }
  }
  std::vector<uint8_t> palm, landmark;
  std::string err;
  const std::string landmark_path = repo + "/" + (kind == "2d" ? kHandLandmarkPath : kHandLandmark3dPath);
  if (!ReadFileBytes(repo + "/" + kPalmDetectionPath, &palm, &err) ||
      !ReadFileBytes(landmark_path, &landmark, &err)) {
    std::cerr << err << "\n";
    return 2;
  }
  HandTracker tracker;
  if (!tracker.Load(palm, landmark, kind == "2d" ? LandmarkModel::k2d : LandmarkModel::k3d, true, &err)) {
    std::cerr << err << "\n";
    return 2;
  }
  if (!dump_dir.empty()) fs::create_directories(dump_dir);
  std::ofstream out(out_path);
  for (int k = 0; k < count; ++k) {
    char name[64];
    std::snprintf(name, sizeof(name), "/frame-%04d.rgb", k);
    std::ifstream f(frames_dir + name, std::ios::binary);
    Image3 img(w, h);
    f.read(reinterpret_cast<char*>(img.px.data()), static_cast<std::streamsize>(img.px.size()));
    if (!f) {
      std::cerr << "cannot read frame " << k << "\n";
      return 2;
    }
    if (reset_each_frame) tracker.Reset();  // palm model on every frame (tensor dumps for the post-processing gate)
    HandResult r;
    if (!tracker.Process(img, &r, &err)) {
      std::cerr << err << "\n";
      return 2;
    }
    if (!dump_dir.empty() && r.palm_ran) {
      auto wr = [&](const std::string& n, const void* p, size_t bytes) {
        std::ofstream o(dump_dir + "/" + n, std::ios::binary);
        o.write(static_cast<const char*>(p), static_cast<std::streamsize>(bytes));
      };
      wr("palm-boxes-" + std::to_string(k) + ".f32", tracker.last_palm_boxes().data(), tracker.last_palm_boxes().size() * 4);
      wr("palm-scores-" + std::to_string(k) + ".f32", tracker.last_palm_scores().data(), tracker.last_palm_scores().size() * 4);
      std::array<float, 4> pad{};
      const Image3 fitted = FitToSize(img, kModelInputSize, kModelInputSize, &pad);
      wr("palm-input-" + std::to_string(k) + ".rgb", fitted.px.data(), fitted.px.size());
    }
    std::ostringstream o;
    o.precision(9);
    o << "{\"k\":" << k << ",\"palm_ran\":" << (r.palm_ran ? 1 : 0) << ",\"present\":" << (r.present ? 1 : 0)
      << ",\"presence_score\":" << r.presence_score << ",\"palm\":[";
    for (size_t i = 0; i < r.palm_detections.size(); ++i) {
      const auto& d = r.palm_detections[i];
      o << (i ? "," : "") << "{\"score\":" << d.score << ",\"xmin\":" << d.xmin << ",\"ymin\":" << d.ymin
        << ",\"w\":" << d.width << ",\"h\":" << d.height << ",\"kp\":[";
      for (size_t j = 0; j < d.keypoints.size(); ++j) o << (j ? "," : "") << "[" << d.keypoints[j].x << "," << d.keypoints[j].y << "]";
      o << "]}";
    }
    auto rect = [&](const NormRect& q) {
      std::ostringstream s;
      s.precision(9);
      s << "[" << q.x_center << "," << q.y_center << "," << q.width << "," << q.height << "," << q.rotation << "]";
      return s.str();
    };
    o << "],\"rect_from_palm\":" << rect(r.rect_from_palm) << ",\"hand_rect\":" << rect(r.hand_rect)
      << ",\"rect_next\":" << rect(r.rect_for_next_frame) << ",\"landmarks\":[";
    for (size_t i = 0; i < r.landmarks.size(); ++i) {
      o << (i ? "," : "") << "[" << r.landmarks[i].x << "," << r.landmarks[i].y << "," << r.landmarks[i].z << "]";
    }
    o << "],\"ms\":{\"palm_pre\":" << r.times.palm_preprocess << ",\"palm_inf\":" << r.times.palm_inference
      << ",\"palm_post\":" << r.times.palm_postprocess << ",\"lm_pre\":" << r.times.landmark_preprocess
      << ",\"lm_inf\":" << r.times.landmark_inference << ",\"lm_post\":" << r.times.landmark_postprocess << "}}\n";
    out << o.str();
  }
  return 0;
}

// Open-loop ear model diagnostic (docs/spec/08-face-ear.md §5.5, RM-7 G3): run the vendored
// EarDetectorLocal once per left/right ear pair on the 112x160 crops the original runtime itself
// rendered on its GPU (ear_openloop.py extract), with no ROI feedback, so the output differs from
// the original ear detections only by inference.
//
// Usage: sr_face_earloop --repo <root> --pairs-dir <dir> --pairs <txt> --out <jsonl>
//                        [--swap-lr 1] [--frames <dir> --cpu-crops-dir <dir>]
// <txt>: "<pair> <texture index> mL0..mL15 mR0..mR15" (column-major renderTexToImageTex matrices of
// the original left and right ear detections). Crops: <pairs-dir>/ear-pair-<pair>-{L,R}.rgba.
// --swap-lr feeds the right crop as the left input and vice versa (mutation of the gate).
// --frames/--cpu-crops-dir additionally draw the CPU renderer crop of each ear from the recorded
// camera texture and the original ROI, to be compared with the GPU crop (crop-diff).

#include <cstdio>
#include <cstring>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include "c8/geometry/intrinsics-clip.h"
#include "c8/hmatrix.h"
#include "c8/pixels/pixel-buffer.h"
#include "face/face_controller.h"
#include "face/face_models.h"
#include "face/face_roi_renderer.h"
#include "reality/engine/ears/ear-detector-local.h"
#include "reality/engine/faces/face-geometry.h"

namespace slam_refact::face {
namespace {

constexpr int kW = EAR_LANDMARK_DETECTION_INPUT_WIDTH;
constexpr int kH = EAR_LANDMARK_DETECTION_INPUT_HEIGHT;

c8::HMatrix WarpFromRenderTexToImageTex(const float* m) {
  // texToClip of face-geometry.cc; renderTexToImageTex = texToClip^-1 warp^-1 texToClip.
  const c8::HMatrix tex_to_clip{{2.0f, 0.0f, 0.0f, -1.0f}, {0.0f, 2.0f, 0.0f, -1.0f},
                                {0.0f, 0.0f, 1.0f, 0.0f},  {0.0f, 0.0f, 0.0f, 1.0f},
                                {0.5f, 0.0f, 0.0f, 0.5f},  {0.0f, 0.5f, 0.0f, 0.5f},
                                {0.0f, 0.0f, 1.0f, 0.0f},  {0.0f, 0.0f, 0.0f, 1.0f}};
  const c8::HMatrix m_full{{m[0], m[4], m[8], m[12]},
                           {m[1], m[5], m[9], m[13]},
                           {m[2], m[6], m[10], m[14]},
                           {m[3], m[7], m[11], m[15]}};
  const c8::HMatrix w = tex_to_clip * m_full.inv() * tex_to_clip.inv();
  return c8::HMatrix{{w(0, 0), w(0, 1), w(0, 2), w(0, 3)},
                     {w(1, 0), w(1, 1), w(1, 2), w(1, 3)},
                     {w(2, 0), w(2, 1), w(2, 2), w(2, 3)},
                     {0.0f, 0.0f, 0.0f, 1.0f}};
}

bool ReadCrop(const std::string& path, c8::RGBA8888PlanePixelBuffer* crop, std::string* error) {
  std::vector<uint8_t> bytes;
  if (!ReadFileBytes(path, &bytes, error) || bytes.size() != static_cast<size_t>(4 * kW * kH)) {
    *error = "bad ear crop " + path;
    return false;
  }
  auto cp = crop->pixels();
  for (int r = 0; r < kH; ++r) {
    std::memcpy(cp.pixels() + r * cp.rowBytes(), bytes.data() + r * 4 * kW, 4 * kW);
  }
  return true;
}

std::string DetectionJson(const c8::DetectedPoints& d) {
  char buf[96];
  std::snprintf(buf, sizeof(buf), "{\"confidence\":%.9g,\"points\":[", d.confidence);
  std::string o = buf;
  for (size_t i = 0; i < d.points.size(); ++i) {
    std::snprintf(buf, sizeof(buf), "%s[%.9g,%.9g,%.9g]", i ? "," : "", d.points[i].x(),
                  d.points[i].y(), d.points[i].z());
    o += buf;
  }
  return o + "]}";
}

int Run(int argc, char** argv) {
  std::string repo = ".";
  std::string pairs_dir;
  std::string pairs;
  std::string out_path;
  std::string frames;
  std::string cpu_crops_dir;
  bool swap_lr = false;
  for (int i = 1; i + 1 < argc; i += 2) {
    const std::string k = argv[i];
    const std::string v = argv[i + 1];
    if (k == "--repo") {
      repo = v;
    } else if (k == "--pairs-dir") {
      pairs_dir = v;
    } else if (k == "--pairs") {
      pairs = v;
    } else if (k == "--out") {
      out_path = v;
    } else if (k == "--swap-lr") {
      swap_lr = v == "1";
    } else if (k == "--frames") {
      frames = v;
    } else if (k == "--cpu-crops-dir") {
      cpu_crops_dir = v;
    } else {
      std::fprintf(stderr, "unknown argument %s\n", k.c_str());
      return 2;
    }
  }
  if (pairs_dir.empty() || pairs.empty() || out_path.empty()) {
    std::fprintf(stderr, "--pairs-dir, --pairs and --out are required\n");
    return 2;
  }
  std::vector<uint8_t> ear_bytes;
  std::string error;
  if (!ReadFileBytes(repo + "/" + kFaceEarModelPath, &ear_bytes, &error)) {
    std::fprintf(stderr, "%s\n", error.c_str());
    return 1;
  }
  c8::EarDetectorLocal detector(ear_bytes);
  const auto k = DefaultDeviceIntrinsics();
  std::ifstream in(pairs);
  std::ofstream out(out_path);
  std::string line;
  while (std::getline(in, line)) {
    std::istringstream ss(line);
    int pair = 0;
    int tex = 0;
    float m[2][16];
    ss >> pair >> tex;
    for (auto& mat : m) {
      for (float& v : mat) {
        ss >> v;
      }
    }
    if (!ss) {
      continue;
    }
    char name[96];
    c8::RGBA8888PlanePixelBuffer crops[2] = {c8::RGBA8888PlanePixelBuffer(kH, kW),
                                             c8::RGBA8888PlanePixelBuffer(kH, kW)};
    for (int side = 0; side < 2; ++side) {
      std::snprintf(name, sizeof(name), "/ear-pair-%04d-%c.rgba", pair, side == 0 ? 'L' : 'R');
      if (!ReadCrop(pairs_dir + name, &crops[side], &error)) {
        std::fprintf(stderr, "%s\n", error.c_str());
        return 1;
      }
    }
    if (!frames.empty() && !cpu_crops_dir.empty()) {
      std::vector<uint8_t> bytes;
      std::snprintf(name, sizeof(name), "/frame-%04d.rgba", tex);
      if (!ReadFileBytes(frames + name, &bytes, &error) || bytes.size() != 480u * 640u * 4u) {
        std::fprintf(stderr, "bad frame %s\n", name);
        return 1;
      }
      for (int side = 0; side < 2; ++side) {
        c8::RGBA8888PlanePixelBuffer cpu(kH, kW);
        auto cp = cpu.pixels();
        for (int r = 0; r < kH; ++r) {
          for (int c = 0; c < kW; ++c) {
            uint8_t* p = cp.pixels() + r * cp.rowBytes() + 4 * c;
            p[0] = p[1] = p[2] = 0;
            p[3] = 255;
          }
        }
        DrawQuad({bytes.data(), 480, 640, 4 * 480}, WarpFromRenderTexToImageTex(m[side]),
                 {0.0f, 0.0f, static_cast<float>(kW), static_cast<float>(kH)}, cp);
        std::snprintf(name, sizeof(name), "/cpu-ear-%04d-%c.rgba", pair, side == 0 ? 'L' : 'R');
        std::ofstream f(cpu_crops_dir + name, std::ios::binary);
        for (int r = 0; r < kH; ++r) {
          f.write(reinterpret_cast<const char*>(cp.pixels() + r * cp.rowBytes()), 4 * kW);
        }
      }
    }
    const int li = swap_lr ? 1 : 0;
    const int ri = swap_lr ? 0 : 1;
    const c8::RenderedSubImage left{
        {0.0f, 0.0f, static_cast<float>(kW), static_cast<float>(kH)}, crops[li].pixels(),
        MakeImageRoi(c8::ImageRoi::Source::EAR_LEFT, 1, WarpFromRenderTexToImageTex(m[0]))};
    const c8::RenderedSubImage right{
        {0.0f, 0.0f, static_cast<float>(kW), static_cast<float>(kH)}, crops[ri].pixels(),
        MakeImageRoi(c8::ImageRoi::Source::EAR_RIGHT, 1, WarpFromRenderTexToImageTex(m[1]))};
    const auto ears = detector.analyzeEars(left, right, k);
    out << "{\"pair\":" << pair << ",\"left\":" << DetectionJson(ears[0])
        << ",\"right\":" << DetectionJson(ears[1]) << "}\n";
  }
  return 0;
}

}  // namespace
}  // namespace slam_refact::face

int main(int argc, char** argv) { return slam_refact::face::Run(argc, argv); }

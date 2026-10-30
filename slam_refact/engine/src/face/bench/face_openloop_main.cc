// Open-loop mesh diagnostic (docs/spec/08-face-ear.md §5.2 T7e): for each ROI the original used,
// draw the crop of the recorded camera texture with the CPU renderer and run the vendored
// FaceDetectorLocal once, with no ROI feedback.
//
// Usage: sr_face_openloop --repo <root> --frames <dir> --rois <txt> --out <jsonl> [--crops-dir <dir>]
//                         [--gpu-crops-dir <dir>]
// --crops-dir writes each 192x192 RGBA crop as crop-<id>.rgba (input of the OR-2 check, §5.3).
// --gpu-crops-dir (RM-7 G2): do not draw anything; read gpu-crop-<id>.rgba (the original GPU
// render cut at the detection's viewport, openloop_mesh.py extract-gpu-crops) and run the detector
// on it, so the output differs from the original only by inference (docs/spec/08-face-ear.md §5.5).
// <txt>: one line per original mesh detection: "<id> <texture index> m0 .. m15" with m the
// column-major renderTexToImageTex matrix of the original detection.

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
#include "reality/engine/faces/face-detector-local.h"

namespace slam_refact::face {
namespace {

int Run(int argc, char** argv) {
  std::string repo = ".";
  std::string frames;
  std::string rois;
  std::string out_path;
  std::string crops_dir;
  std::string gpu_crops_dir;
  for (int i = 1; i + 1 < argc; i += 2) {
    const std::string k = argv[i];
    const std::string v = argv[i + 1];
    if (k == "--repo") {
      repo = v;
    } else if (k == "--frames") {
      frames = v;
    } else if (k == "--rois") {
      rois = v;
    } else if (k == "--out") {
      out_path = v;
    } else if (k == "--crops-dir") {
      crops_dir = v;
    } else if (k == "--gpu-crops-dir") {
      gpu_crops_dir = v;
    } else {
      std::fprintf(stderr, "unknown argument %s\n", k.c_str());
      return 2;
    }
  }
  std::vector<uint8_t> mesh_bytes;
  std::string error;
  if (!ReadFileBytes(repo + "/" + kFaceMeshModelPath, &mesh_bytes, &error)) {
    std::fprintf(stderr, "%s\n", error.c_str());
    return 1;
  }
  c8::EarConfig no_ears;
  c8::FaceDetectorLocal detector(mesh_bytes, no_ears);
  const auto k = c8::Intrinsics::rotateCropAndScaleIntrinsics(DefaultDeviceIntrinsics(), 480, 640);
  // texToClip of face-geometry.cc.
  const c8::HMatrix tex_to_clip{{2.0f, 0.0f, 0.0f, -1.0f}, {0.0f, 2.0f, 0.0f, -1.0f},
                                {0.0f, 0.0f, 1.0f, 0.0f},  {0.0f, 0.0f, 0.0f, 1.0f},
                                {0.5f, 0.0f, 0.0f, 0.5f},  {0.0f, 0.5f, 0.0f, 0.5f},
                                {0.0f, 0.0f, 1.0f, 0.0f},  {0.0f, 0.0f, 0.0f, 1.0f}};
  std::ifstream in(rois);
  std::ofstream out(out_path);
  std::string line;
  std::vector<uint8_t> bytes;
  while (std::getline(in, line)) {
    std::istringstream ss(line);
    int id = 0;
    int tex = 0;
    float m[16];
    ss >> id >> tex;
    for (float& v : m) {
      ss >> v;
    }
    if (!ss) {
      continue;
    }
    char name[64];
    // renderTexToImageTex = texToClip^-1 warp^-1 texToClip, so warp = texToClip M^-1 texToClip^-1.
    const c8::HMatrix m_full{{m[0], m[4], m[8], m[12]},
                             {m[1], m[5], m[9], m[13]},
                             {m[2], m[6], m[10], m[14]},
                             {m[3], m[7], m[11], m[15]}};
    const c8::HMatrix w = tex_to_clip * m_full.inv() * tex_to_clip.inv();
    // The float round trip leaves ~1e-8 in the last row; the original warp is affine.
    const c8::HMatrix warp{{w(0, 0), w(0, 1), w(0, 2), w(0, 3)},
                           {w(1, 0), w(1, 1), w(1, 2), w(1, 3)},
                           {w(2, 0), w(2, 1), w(2, 2), w(2, 3)},
                           {0.0f, 0.0f, 0.0f, 1.0f}};
    c8::RGBA8888PlanePixelBuffer crop(192, 192);
    auto cp = crop.pixels();
    if (!gpu_crops_dir.empty()) {
      std::snprintf(name, sizeof(name), "/gpu-crop-%04d.rgba", id);
      if (!ReadFileBytes(gpu_crops_dir + name, &bytes, &error) || bytes.size() != 192u * 192u * 4u) {
        std::fprintf(stderr, "bad gpu crop %s\n", name);
        return 1;
      }
      for (int r = 0; r < 192; ++r) {
        std::memcpy(cp.pixels() + r * cp.rowBytes(), bytes.data() + r * 4 * 192, 4 * 192);
      }
    } else {
      std::snprintf(name, sizeof(name), "/frame-%04d.rgba", tex);
      if (!ReadFileBytes(frames + name, &bytes, &error) || bytes.size() != 480u * 640u * 4u) {
        std::fprintf(stderr, "bad frame %s\n", name);
        return 1;
      }
      for (int r = 0; r < 192; ++r) {
        for (int c = 0; c < 192; ++c) {
          uint8_t* p = cp.pixels() + r * cp.rowBytes() + 4 * c;
          p[0] = p[1] = p[2] = 0;
          p[3] = 255;
        }
      }
      DrawQuad({bytes.data(), 480, 640, 4 * 480}, warp, {0.0f, 0.0f, 192.0f, 192.0f}, cp);
    }
    if (!crops_dir.empty()) {
      std::snprintf(name, sizeof(name), "/crop-%04d.rgba", id);
      std::ofstream crop_file(crops_dir + name, std::ios::binary);
      for (int r = 0; r < 192; ++r) {
        crop_file.write(reinterpret_cast<const char*>(cp.pixels() + r * cp.rowBytes()), 4 * 192);
      }
    }
    const auto roi = MakeImageRoi(c8::ImageRoi::Source::FACE, 1, warp);
    const auto dets = detector.analyzeFace({{192.0f, 0.0f, 192.0f, 192.0f}, cp, roi}, k);
    std::string o = "{\"id\":" + std::to_string(id) + ",\"texture\":" + std::to_string(tex);
    if (dets.empty()) {
      o += ",\"detected\":false}";
    } else {
      char buf[64];
      std::snprintf(buf, sizeof(buf), "%.9g", dets[0].confidence);
      o += ",\"detected\":true,\"confidence\":" + std::string(buf) + ",\"points\":[";
      for (size_t i = 0; i < dets[0].points.size(); ++i) {
        std::snprintf(buf, sizeof(buf), "%s[%.9g,%.9g]", i ? "," : "", dets[0].points[i].x(),
                      dets[0].points[i].y());
        o += buf;
      }
      o += "]}";
    }
    out << o << "\n";
  }
  return 0;
}

}  // namespace
}  // namespace slam_refact::face

int main(int argc, char** argv) { return slam_refact::face::Run(argc, argv); }

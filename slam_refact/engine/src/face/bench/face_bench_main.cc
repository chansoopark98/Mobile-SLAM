// Face/ear pipeline timing bench, native and WASM/node (US-018/US-019; docs/spec/08-face-ear.md
// §7). Desktop numbers are a reference record, not mobile numbers.
//
// Usage:
//   sr_face_bench --repo <repo root> [--frames <dir>] [--iters N] [--warmup W] [--ears 0|1]
//                 [--json <out>]
// Components (always): the vendored FaceDetectorGlobal (128 letterbox), FaceDetectorLocal (192
// crop), TrackedFaceState::locateFace (3D, Ceres pose), EarDetectorLocal (2 x 112x160) and
// earLiftLandmarksTo3D, and the CPU ROI render of one StageFrame, each on fixed synthetic input.
// Pipeline (with --frames): <dir>/meta.json {"width","height","count"} and frame-%04d.rgba
// (RGBA8, row 0 = top) through FaceController (StageFrame + ProcessStagedFrame), split by
// whether the processed frame tracked a face.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <functional>
#include <sstream>
#include <string>
#include <vector>

#include "c8/geometry/facemesh-data.h"
#include "c8/pixels/pixel-buffer.h"
#include "face/face_controller.h"
#include "face/face_models.h"
#include "face/face_roi_renderer.h"
#include "reality/engine/ears/ear-detector-local.h"
#include "reality/engine/ears/ear-types.h"
#include "reality/engine/faces/face-detector-global.h"
#include "reality/engine/faces/face-detector-local.h"
#include "reality/engine/faces/face-geometry.h"
#include "reality/engine/faces/tracked-face-state.h"
#include "c8/geometry/intrinsics-clip.h"

namespace slam_refact::face {
namespace {

struct Stats {
  size_t n = 0;
  double p50 = 0.0;
  double p95 = 0.0;
  double p99 = 0.0;
  double max = 0.0;
};

Stats Summarize(std::vector<double> v) {
  Stats s;
  s.n = v.size();
  if (v.empty()) {
    return s;
  }
  std::vector<double> sorted = v;
  // Values only (no ties that matter): a plain sort of doubles gives the same order everywhere.
  std::stable_sort(sorted.begin(), sorted.end());
  auto at = [&sorted](double q) {
    return sorted[static_cast<size_t>(std::floor(q * static_cast<double>(sorted.size() - 1)))];
  };
  s.p50 = at(0.50);
  s.p95 = at(0.95);
  s.p99 = at(0.99);
  s.max = sorted.back();
  return s;
}

double TimeMs(const std::function<void()>& fn) {
  const auto t0 = std::chrono::steady_clock::now();
  fn();
  return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
}

std::vector<double> Repeat(int warmup, int iters, const std::function<void()>& fn) {
  for (int i = 0; i < warmup; ++i) {
    fn();
  }
  std::vector<double> out;
  for (int i = 0; i < iters; ++i) {
    out.push_back(TimeMs(fn));
  }
  return out;
}

// Deterministic gray pattern (sin/cos of normalized coordinates), RGBA.
c8::RGBA8888PlanePixelBuffer Pattern(int rows, int cols) {
  c8::RGBA8888PlanePixelBuffer b(rows, cols);
  auto p = b.pixels();
  for (int r = 0; r < rows; ++r) {
    for (int c = 0; c < cols; ++c) {
      const double x = static_cast<double>(c) / cols;
      const double y = static_cast<double>(r) / rows;
      const double v = 127.5 + 100.0 * std::sin(9.0 * x) * std::cos(7.0 * y);
      uint8_t* px = p.pixels() + r * p.rowBytes() + 4 * c;
      px[0] = px[1] = px[2] = static_cast<uint8_t>(std::clamp(v, 0.0, 255.0));
      px[3] = 255;
    }
  }
  return b;
}

std::string StatsJson(const Stats& s) {
  char buf[200];
  std::snprintf(buf, sizeof(buf),
                "{\"n\":%zu,\"p50_ms\":%.4f,\"p95_ms\":%.4f,\"p99_ms\":%.4f,\"max_ms\":%.4f}", s.n,
                s.p50, s.p95, s.p99, s.max);
  return buf;
}

bool ReadInt(const std::string& json, const std::string& key, int* out) {
  const auto pos = json.find("\"" + key + "\"");
  if (pos == std::string::npos) {
    return false;
  }
  const auto colon = json.find(':', pos);
  if (colon == std::string::npos) {
    return false;
  }
  *out = std::atoi(json.c_str() + colon + 1);
  return true;
}

int Run(int argc, char** argv) {
  std::string repo = ".";
  std::string frames_dir;
  std::string json_out;
  int iters = 50;
  int warmup = 5;
  bool ears = true;
  for (int i = 1; i + 1 < argc; i += 2) {
    const std::string k = argv[i];
    const std::string v = argv[i + 1];
    if (k == "--repo") {
      repo = v;
    } else if (k == "--frames") {
      frames_dir = v;
    } else if (k == "--iters") {
      iters = std::atoi(v.c_str());
    } else if (k == "--warmup") {
      warmup = std::atoi(v.c_str());
    } else if (k == "--ears") {
      ears = v != "0";
    } else if (k == "--json") {
      json_out = v;
    } else {
      std::fprintf(stderr, "unknown argument %s\n", k.c_str());
      return 2;
    }
  }
  std::vector<uint8_t> detect_bytes;
  std::vector<uint8_t> mesh_bytes;
  std::vector<uint8_t> ear_bytes;
  std::string error;
  if (!ReadFileBytes(repo + "/" + kFaceDetectModelPath, &detect_bytes, &error) ||
      !ReadFileBytes(repo + "/" + kFaceMeshModelPath, &mesh_bytes, &error) ||
      !ReadFileBytes(repo + "/" + kFaceEarModelPath, &ear_bytes, &error)) {
    std::fprintf(stderr, "model file missing: %s\n", error.c_str());
    return 1;
  }

  std::ostringstream js;
  js << "{\"iters\":" << iters << ",\"warmup\":" << warmup << ",\"components\":{";

  // FaceDetectorGlobal on the 96x128 viewport of a 480x640 frame (face-roi-renderer letterbox).
  c8::FaceDetectorGlobal global(detect_bytes);
  auto global_img = Pattern(128, 96);
  c8::RenderedSubImage global_src{{16.0f, 0.0f, 96.0f, 128.0f}, global_img.pixels(),
                                  MakeImageRoi(c8::ImageRoi::Source::FACE, 0, c8::HMatrixGen::i())};
  const auto k = c8::Intrinsics::rotateCropAndScaleIntrinsics(DefaultDeviceIntrinsics(), 480, 640);
  js << "\"global_detect\":"
     << StatsJson(Summarize(Repeat(warmup, iters, [&] { global.detectFaces(global_src, k); })));

  // FaceDetectorLocal on a 192x192 crop.
  c8::EarConfig ear_config;
  ear_config.isEnabled = ears;
  c8::FaceDetectorLocal local(mesh_bytes, ear_config);
  auto local_img = Pattern(192, 192);
  c8::RenderedSubImage local_src{{192.0f, 0.0f, 192.0f, 192.0f}, local_img.pixels(),
                                 MakeImageRoi(c8::ImageRoi::Source::FACE, 1, c8::HMatrixGen::i())};
  js << ",\"local_mesh\":"
     << StatsJson(Summarize(Repeat(warmup, iters, [&] { local.analyzeFace(local_src, k); })));

  // TrackedFaceState::locateFace on the public sample mesh output (tracked-face-state-test.cc).
  c8::DetectedPoints sample{1.0f, 0, {0.0f, 192.0f, 192.0f, 192.0f},
                            MakeImageRoi(c8::ImageRoi::Source::FACE, 2,
                                         c8::HMatrix{{3.631096f, 0.0f, 0.0f, -1.445287f},
                                                     {0.0f, 4.841462f, 0.0f, 2.579028f},
                                                     {0.0f, 0.0f, 1.0f, 0.0f},
                                                     {0.0f, 0.0f, 0.0f, 1.0f}}),
                            {{.25f, 0.5f}, {.75f, 0.5f}, {.25f, 1.0f}, {1.0f, 1.0f}},
                            FACEMESH_SAMPLE_OUTPUT, k, {}};
  c8::TrackedFaceState state;
  js << ",\"locate_face_3d\":"
     << StatsJson(Summarize(Repeat(warmup, iters, [&] { state.locateFace(sample); })));

  // EarDetectorLocal on two 112x160 crops, and the 3D lift.
  c8::EarDetectorLocal ear(ear_bytes);
  auto ear_img = Pattern(160, 112);
  c8::RenderedSubImage ear_l{{0.0f, 0.0f, 112.0f, 160.0f}, ear_img.pixels(),
                             MakeImageRoi(c8::ImageRoi::Source::EAR_LEFT, 1, c8::HMatrixGen::i())};
  c8::RenderedSubImage ear_r{{0.0f, 160.0f, 112.0f, 160.0f}, ear_img.pixels(),
                             MakeImageRoi(c8::ImageRoi::Source::EAR_RIGHT, 1, c8::HMatrixGen::i())};
  js << ",\"ear_landmarks\":"
     << StatsJson(Summarize(Repeat(warmup, iters, [&] { ear.analyzeEars(ear_l, ear_r, k); })));
  const c8::Face3d face3d = c8::detectionToMeshNoFilter(sample);
  auto ear_rois = c8::earRoisByFaceMesh(sample);
  auto ear_dets = ear.analyzeEars(ear_l, ear_r, k);
  for (const auto& pt : ear_dets[0].points) {
    ear_rois[0].points.push_back(pt);
  }
  for (const auto& pt : ear_dets[1].points) {
    ear_rois[1].points.push_back(pt);
  }
  js << ",\"ear_lift_3d\":" << StatsJson(Summarize(Repeat(warmup, iters, [&] {
       c8::earLiftLandmarksTo3D(sample, ear_rois[0], ear_rois[1], face3d, k);
     })));

  // CPU ROI render of one frame: full image + one face crop (+ two ear crops).
  FaceRoiRendererCpu face_renderer;
  face_renderer.Initialize(480, 640);
  EarRoiRendererCpu ear_renderer;
  ear_renderer.Initialize(480, 640);
  auto frame_buf = Pattern(640, 480);
  const RgbaImageView frame{frame_buf.pixels().pixels(), 480, 640, frame_buf.pixels().rowBytes()};
  js << ",\"cpu_render\":" << StatsJson(Summarize(Repeat(warmup, iters, [&] {
       face_renderer.SetDetectedFaces({sample});
       face_renderer.DrawImmediate(frame);
       if (ears) {
         ear_renderer.SetDetectedEars(ear_rois);
         ear_renderer.DrawImmediate(frame);
       }
     })));
  js << "}";

  if (!frames_dir.empty()) {
    std::ifstream meta_file(frames_dir + "/meta.json");
    std::stringstream meta;
    meta << meta_file.rdbuf();
    int width = 0;
    int height = 0;
    int count = 0;
    if (!ReadInt(meta.str(), "width", &width) || !ReadInt(meta.str(), "height", &height) ||
        !ReadInt(meta.str(), "count", &count) || width <= 0 || height <= 0 || count <= 0) {
      std::fprintf(stderr, "bad %s/meta.json\n", frames_dir.c_str());
      return 1;
    }
    FaceController controller;
    FaceOptions opts = DefaultFaceOptions();
    opts.enable_ears = ears;
    controller.UpdateFaceOptions(opts);
    FaceCameraEnvironment env;
    env.camera_width = width;
    env.camera_height = height;
    env.display_width = width;
    env.display_height = height;
    controller.UpdateCameraEnvironment(env);
    controller.GetFrameworkData();
    if (!controller.LoadFaceDetectModel(detect_bytes, &error) ||
        !controller.LoadFaceMeshModel(mesh_bytes, &error) ||
        (ears && !controller.LoadEarModel(ear_bytes, &error))) {
      std::fprintf(stderr, "%s\n", error.c_str());
      return 1;
    }
    std::vector<double> total_face;
    std::vector<double> total_noface;
    std::vector<double> render;
    std::vector<double> track_face;
    std::vector<double> track_noface;
    std::vector<double> ear_ms;
    std::vector<double> response_ms;
    int tracked_frames = 0;
    std::vector<uint8_t> bytes;
    for (int i = 0; i < count; ++i) {
      char name[64];
      std::snprintf(name, sizeof(name), "/frame-%04d.rgba", i);
      if (!ReadFileBytes(frames_dir + name, &bytes, &error) ||
          bytes.size() != static_cast<size_t>(4 * width * height)) {
        std::fprintf(stderr, "bad frame %s\n", name);
        return 1;
      }
      const RgbaImageView view{bytes.data(), width, height, 4 * width};
      const FaceResponse* response = nullptr;
      bool staged = false;
      const double total = TimeMs([&] {
        staged = controller.StageFrame(view, &error);
        response = controller.ProcessStagedFrame();
      });
      if (!staged) {
        std::fprintf(stderr, "%s\n", error.c_str());
        return 1;
      }
      const auto& t = controller.last_timings();
      if (!t.processed) {
        continue;
      }
      bool tracked = false;
      for (const auto& f : response->faces) {
        tracked = tracked || f.status == FaceTrackingStatus::kFound ||
                  f.status == FaceTrackingStatus::kUpdated;
      }
      render.push_back(t.render_ms);
      response_ms.push_back(t.response_ms);
      if (tracked) {
        ++tracked_frames;
        total_face.push_back(total);
        track_face.push_back(t.track_ms);
        if (ears) {
          ear_ms.push_back(t.ears_ms);
        }
      } else {
        total_noface.push_back(total);
        track_noface.push_back(t.track_ms);
      }
    }
    js << ",\"pipeline\":{\"frames\":" << count << ",\"tracked_frames\":" << tracked_frames
       << ",\"total_tracked\":" << StatsJson(Summarize(total_face))
       << ",\"total_untracked\":" << StatsJson(Summarize(total_noface))
       << ",\"render\":" << StatsJson(Summarize(render))
       << ",\"track_tracked\":" << StatsJson(Summarize(track_face))
       << ",\"track_untracked\":" << StatsJson(Summarize(track_noface))
       << ",\"ears_tracked\":" << StatsJson(Summarize(ear_ms))
       << ",\"response\":" << StatsJson(Summarize(response_ms)) << "}";
  }
  js << "}";
  std::printf("%s\n", js.str().c_str());
  if (!json_out.empty()) {
    std::ofstream out(json_out);
    out << js.str() << "\n";
  }
  return 0;
}

}  // namespace
}  // namespace slam_refact::face

int main(int argc, char** argv) { return slam_refact::face::Run(argc, argv); }

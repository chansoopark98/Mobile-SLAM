// Replays a recorded original sky run (tools/re/runtime/run-sky-reference.mjs) through the
// slam_refact sky pipeline and writes what the comparison tools read (docs/spec/09-sky.md §6).
//
//   sr_sky_replay --record <dir> --out <new dir> --mode e2e|inject-worker [--model <path>]
//
// modes
//   e2e            camera texture -> our resize -> our model -> our cubemap (closed loop)
//   infer          open loop of the model alone: every recorded ORIGINAL worker input image
//                  (invoke-<n>.img.rgba) through our model; writes <out>/invoke/invoke-<n>.data.f32
//   inject-worker  camera texture -> our resize -> the ORIGINAL worker answers (data, percentage in
//                  record order): isolates the cubemap / post-process / controller port from the model
//
// Output: <out>/sky/sky-<k>.rgba (our layers.sky texture after processed frame k), <out>/frames.tsv
// (k valid percentage texture w h request rotation), <out>/events.tsv, <out>/invoke/invoke-<n>.{img.rgba,
// data.f32} (our request image and answer of the n-th request), <out>/resized/resized-<k>.rgba.
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>

#include "sky/bench/sky_held.h"
#include "sky/bench/sky_record.h"
#include "sky/sky_pipeline.h"

namespace fs = std::filesystem;
using namespace slam_refact::sky;

namespace {
bool WriteBytes(const std::string& path, const void* p, size_t n) {
  std::ofstream f(path, std::ios::binary);
  f.write(static_cast<const char*>(p), static_cast<std::streamsize>(n));
  return static_cast<bool>(f);
}
}  // namespace

int main(int argc, char** argv) {
  std::string record_dir, out_dir, mode = "e2e", model_path;
  bool seamless = true;
  bool dump_cube = false;
  for (int i = 1; i + 1 < argc; i += 2) {
    const std::string a = argv[i];
    if (a == "--record") record_dir = argv[i + 1];
    else if (a == "--out") out_dir = argv[i + 1];
    else if (a == "--mode") mode = argv[i + 1];
    else if (a == "--model") model_path = argv[i + 1];
    else if (a == "--seamless") seamless = std::string(argv[i + 1]) != "0";
    else if (a == "--dump-cube") dump_cube = std::string(argv[i + 1]) != "0";
    else {
      std::cerr << "unknown argument " << a << "\n";
      return 2;
    }
  }
  if (record_dir.empty() || out_dir.empty() ||
      (mode != "e2e" && mode != "inject-worker" && mode != "infer")) {
    std::cerr << "usage: sr_sky_replay --record <dir> --out <new dir> --mode "
                 "e2e|inject-worker|infer [--model <path>]\n";
    return 2;
  }
  {
    // Sealed gate sets are measured by the gate only (sky-us020.v1, protocol 12.6): never replayed by this tool.
    std::string why;
    if (IsSealedGateSet(SR_SKY_PREREG_DIR, SR_REPO_ROOT, fs::path(record_dir).filename().string(), record_dir, &why)) {
      std::cerr << "HELD: " << why << "\n";
      return 2;
    }
  }
  SkyRecord rec;
  std::string err;
  if (!SkyRecord::Load(record_dir, &rec, &err)) {
    std::cerr << err << "\n";
    return 2;
  }
  if (fs::exists(out_dir)) {
    std::cerr << "refusing to overwrite " << out_dir << "\n";
    return 2;
  }
  fs::create_directories(out_dir + "/sky");
  fs::create_directories(out_dir + "/invoke");
  fs::create_directories(out_dir + "/resized");

  SkyPipeline pipe;
  if (mode == "e2e" || mode == "infer") {
    if (model_path.empty()) {
      std::cerr << "--model required for e2e\n";
      return 2;
    }
    std::vector<uint8_t> bytes;
    if (!ReadFileBytes(model_path, &bytes, &err) || !pipe.LoadModel(bytes, true, &err)) {
      std::cerr << err << "\n";
      return 2;
    }
  }
  if (mode == "infer") {
    std::ofstream tsv(out_dir + "/infer.tsv");
    tsv << "n\tpercentage\n";
    for (size_t n = 0; n < rec.invokes.size(); ++n) {
      Rgba8Image img;
      if (!rec.ReadImage(rec.invokes[n].img, kModelInputWidth, kModelInputHeight, &img)) {
        std::cerr << "cannot read " << rec.invokes[n].img << "\n";
        return 2;
      }
      SkyInference inf;
      if (!pipe.model().Invoke(img.px.data(), &inf)) return 2;
      WriteBytes(out_dir + "/invoke/invoke-" + std::to_string(rec.invokes[n].k) + ".data.f32",
                 inf.data.data(), inf.data.size() * sizeof(float));
      tsv << rec.invokes[n].k << "\t" << inf.percentage << "\n";
    }
    std::cout << "mode infer invokes " << rec.invokes.size() << "\n";
    return 0;
  }
  SkyEnvironment env;
  env.video_width = rec.video_w;
  env.video_height = rec.video_h;
  env.canvas_width = rec.canvas_w;
  env.canvas_height = rec.canvas_h;
  env.orientation = rec.orientation;
  ConfigureArgs args;
  args.sky.present = true;
  args.sky.has_invert = true;
  args.sky.invert = rec.invert;
  args.sky.has_smoothness = true;
  args.sky.smoothness = rec.edge_smoothness;
  if (rec.near_clip >= 0) {
    args.has_near_clip = true;
    args.near_clip = rec.near_clip;
  }
  if (rec.far_clip >= 0) {
    args.has_far_clip = true;
    args.far_clip = rec.far_clip;
  }
  args.has_coordinates = true;
  args.coordinates.mirrored_display = rec.mirrored;
  pipe.controller().Configure(args);
  pipe.controller().set_cubemap_seamless(seamless);
  pipe.controller().OnStart(env);
  // The deployed module is attached before the first frame: the recorded layerloading event is
  // dispatched asynchronously (a promise callback) and shows up one frame later, but frame 0 is
  // already staged (the first response appears at processed frame 1).
  pipe.controller().OnAttach(env);

  size_t next_invoke = 0;
  int rotation_mismatch = 0;
  int request_count = 0;
  std::ofstream frames_tsv(out_dir + "/frames.tsv");
  std::ofstream events_tsv(out_dir + "/events.tsv");
  std::ofstream pose_tsv(out_dir + "/pose.tsv");  // k, 16 intrinsics, rotation x y z w, position x y z (%.17g)
  pose_tsv.precision(17);
  frames_tsv << "k\tvalid\tpercentage\ttex_w\ttex_h\trequest\n";
  std::vector<float> last_data;
  for (const auto& fr : rec.frames) {
    for (const auto& e : rec.events) {
      if (e.staged != fr.k) continue;
      if (e.name == "layerscanning") pipe.WorkerStarted();
    }
    Rgba8Image cam;
    if (!rec.ReadImage(fr.file, fr.w, fr.h, &cam)) {
      std::cerr << "cannot read " << fr.file << "\n";
      return 2;
    }
    bool requested = false;
    SkyWorkerFn worker = nullptr;
    if (mode == "inject-worker") {
      worker = [&](const SkyWorkerRequest& req, SkyWorkerAnswer* ans) {
        requested = true;
        WriteBytes(out_dir + "/invoke/req-" + std::to_string(request_count) + ".img.rgba", req.img.px.data(),
                   req.img.px.size());
        if (next_invoke >= rec.invokes.size()) return false;
        const auto& iv = rec.invokes[next_invoke++];
        const double d = std::fabs(req.rotation.x - iv.rot.x) + std::fabs(req.rotation.y - iv.rot.y) +
                         std::fabs(req.rotation.z - iv.rot.z) + std::fabs(req.rotation.w - iv.rot.w);
        if (d > 1e-9) ++rotation_mismatch;
        if (!rec.ReadFloats(iv.data, &ans->data)) return false;
        ans->percentage = iv.percentage;
        return true;
      };
    } else {
      worker = [&](const SkyWorkerRequest& req, SkyWorkerAnswer* ans) {
        requested = true;
        SkyInference inf;
        if (!pipe.model().Invoke(req.img.px.data(), &inf)) return false;
        WriteBytes(out_dir + "/invoke/invoke-" + std::to_string(request_count) + ".img.rgba",
                   req.img.px.data(), req.img.px.size());
        WriteBytes(out_dir + "/invoke/invoke-" + std::to_string(request_count) + ".data.f32",
                   inf.data.data(), inf.data.size() * sizeof(float));
        ans->data = std::move(inf.data);
        ans->percentage = inf.percentage;
        return true;
      };
    }
    const LayersFrameOutput out = pipe.ProcessFrame(cam, false, fr.o, worker);
    if (requested) ++request_count;
    frames_tsv << fr.k << "\t" << (out.valid ? 1 : 0) << "\t" << out.sky.percentage << "\t"
               << out.sky.texture_width << "\t" << out.sky.texture_height << "\t"
               << (requested ? 1 : 0) << "\n";
    if (out.valid) {
      pose_tsv << fr.k;
      for (int i = 0; i < 16; ++i) pose_tsv << "\t" << out.intrinsics[i];
      pose_tsv << "\t" << out.rotation.x << "\t" << out.rotation.y << "\t" << out.rotation.z << "\t" << out.rotation.w
               << "\t" << out.position.x << "\t" << out.position.y << "\t" << out.position.z << "\n";
      WriteBytes(out_dir + "/sky/sky-" + std::to_string(fr.k) + ".rgba", out.sky.texture.px.data(),
                 out.sky.texture.px.size());
    }
    for (const auto& e : pipe.controller().TakeEvents()) {
      events_tsv << fr.k << "\t" << e.name << "\t" << e.detail_json << "\n";
    }
  }
  if (dump_cube && pipe.controller().renderer() != nullptr) {
    fs::create_directories(out_dir + "/cube");
    for (int f = 0; f < 6; ++f) {
      const auto& face = pipe.controller().renderer()->cubemap_face(f);
      WriteBytes(out_dir + "/cube/face-" + std::to_string(f) + ".rgba", face.px.data(), face.px.size());
    }
  }
  std::ofstream meta(out_dir + "/replay-meta.txt");
  meta << "mode " << mode << "\nframes " << rec.frames.size() << "\nrequests " << request_count
       << "\nrotation_mismatch " << rotation_mismatch << "\n";
  std::cout << "mode " << mode << " frames " << rec.frames.size() << " requests " << request_count
            << " rotation_mismatch " << rotation_mismatch << "\n";
  return 0;
}

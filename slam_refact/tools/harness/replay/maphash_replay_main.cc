// slam_refact_replay_maphash: US-026 root-cause diagnostic (REPORT tier, not a gate tool). Identical to
// slam_refact_replay (same bootstrap, sensor pushes, drop mask, id seed, trace) and additionally writes
// <out>/maphash.jsonl: after every processed frame the FNV-1a 64 hashes the original memprobe
// (tools/re/runtime/us026/config-poke.js) records as kfPoseHash and ptsHash1, so the first frame at
// which the engine's keyframe poses / type-1 point positions differ from the original's can be found.
// It reads the engine's WorldMap through a linker --wrap capture of IterativeBundleAdjustment (the
// WorldMap it is called with) and of ~WorldMap (to drop the pointer); no engine/src file is changed.
//
//   slam_refact_replay_maphash --bundle <bundle dir> --out <new run dir>
//
// Order of calls per stage (page-runtime.js emit() + the bootstrap of §4.4 / D-4):
//   1. bootstrap: push the bootstrap sensor rows, ProcessFrame(bootstrap stage), discard the output
//   2. for each input frame k: push every run sensor row with receive_time_ns <= time_ns(k)
//      (DeviceMotion then DeviceOrientation per packet); if k is processed, ProcessFrame(k) and
//      write the published response, otherwise write a non-fresh row (drop-mask mode).
// One scenario per process: the engine keeps process-global static state (§4.6).

#include <sys/stat.h>

#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <sstream>
#include <string>

#include "bundle_reader.h"
#include "slam_refact/glue_runner.h"
#include "slam_refact/replay_diagnostics.h"
#include "slam_refact/version.h"
#include "trace_writer.h"
#include "trkprobe.h"
#include "map/world_map_internal.h"

// ---- WorldMap capture (linker --wrap, see CMakeLists.txt) -------------------------------------------
namespace {
void* g_wm = nullptr;
bool g_ba_called = false;  // original baLatch (u8 @608328): the BA loop ran at least once
}  // namespace
extern "C" {
void __real__ZN11slam_refact11map_builder25IterativeBundleAdjustmentERNS_3map8WorldMapE(void*);
void __wrap__ZN11slam_refact11map_builder25IterativeBundleAdjustmentERNS_3map8WorldMapE(void* wm) {
  g_wm = wm;
  g_ba_called = true;
  __real__ZN11slam_refact11map_builder25IterativeBundleAdjustmentERNS_3map8WorldMapE(wm);
}
void __real__ZN11slam_refact3map8WorldMapC1Ev(void*);
void __wrap__ZN11slam_refact3map8WorldMapC1Ev(void* self) {  // newest constructed map; the BA call re-pins (no capture happens with BA off, see report)
  __real__ZN11slam_refact3map8WorldMapC1Ev(self);
  g_wm = self;
}
void __real__ZN11slam_refact3map8WorldMapD1Ev(void*);
void __wrap__ZN11slam_refact3map8WorldMapD1Ev(void* self) {
  if (g_wm == self) g_wm = nullptr;
  __real__ZN11slam_refact3map8WorldMapD1Ev(self);
}
void __real__ZN11slam_refact3map8WorldMapD2Ev(void*);
void __wrap__ZN11slam_refact3map8WorldMapD2Ev(void* self) {
  if (g_wm == self) g_wm = nullptr;
  __real__ZN11slam_refact3map8WorldMapD2Ev(self);
}
}

namespace slam_refact::harness {
namespace {

struct Fnv {
  uint64_t h = 0xcbf29ce484222325ull;
  void Bytes(const void* p, size_t n) {
    const unsigned char* b = static_cast<const unsigned char*>(p);
    for (size_t i = 0; i < n; ++i) h = (h ^ b[i]) * 0x100000001b3ull;
  }
  void U32(uint32_t v) {  // little-endian bytes (host is little-endian x86-64)
    Bytes(&v, 4);
  }
  std::string Hex() const {
    char buf[20];
    std::snprintf(buf, sizeof(buf), "%016llx", static_cast<unsigned long long>(h));
    return buf;
  }
};

// Same definitions as config-poke.js: kfPoseHash = FNV-1a 64 over the forward extrinsic (16 f32) of every
// live keyframe in slot order; ptsHash1 = FNV-1a 64 over [slot u32, x,y,z,w f32] of every live (state != 2)
// type-1 point in pool slot order.
std::string WriteMapHash(std::FILE* f, int frame_index) {
  if (g_wm == nullptr) {
    std::fprintf(f, "{\"i\":%d,\"wm\":false}\n", frame_index);
    return "";
  }
  const auto& im = map::WorldMapAccess::impl(*static_cast<const map::WorldMap*>(g_wm));
  Fnv kf;
  int kf_live = 0;
  for (const map::Keyframe& k : im.keyframes) {
    if (k.id == 0) continue;
    ++kf_live;
    kf.Bytes(k.frame.extrinsic().data().data(), 64);
  }
  Fnv pt;
  int pt_live = 0;
  const map::PointPool& pool = im.pools[0];
  for (size_t s = 0; s < pool.points.size(); ++s) {
    const map::WorldPoint& p = pool.points[s];
    if (static_cast<int32_t>(p.state) == 2) continue;
    ++pt_live;
    pt.U32(static_cast<uint32_t>(s));
    pt.Bytes(p.position.raw().data(), 16);
  }
  // memprobe pool(wm, k) = [slots, free list size (0 for k=5), live (state != 2), tenured (state == 1)], k = 1..5.
  std::string pools = "[";
  for (int k = 0; k < 5; ++k) {
    const map::PointPool& pl = im.pools[k];
    int live = 0, tenured = 0;
    for (const map::WorldPoint& p : pl.points) {
      const int32_t st = static_cast<int32_t>(p.state);
      if (st != 2) ++live;
      if (st == 1) ++tenured;
    }
    char buf[96];
    std::snprintf(buf, sizeof(buf), "%s[%zu,%zu,%d,%d]", k ? "," : "", pl.points.size(),
                  k == 4 ? static_cast<size_t>(0) : pl.free_slots.size(), live, tenured);
    pools += buf;
  }
  pools += "]";
  std::fprintf(f, "{\"i\":%d,\"wm\":true,\"kfLive\":%d,\"kfPoseHash\":\"%s\",\"pts1Live\":%d,\"ptsHash1\":\"%s\",\"pools\":%s,\"baLatch\":%d}\n",
               frame_index, kf_live, kf.Hex().c_str(), pt_live, pt.Hex().c_str(), pools.c_str(), g_ba_called ? 1 : 0);
  return "";
}

}  // namespace
}  // namespace slam_refact::harness

namespace slam_refact::harness {
namespace {

[[noreturn]] void Fail(const std::string& msg) {
  std::fprintf(stderr, "slam_refact_replay: %s\n", msg.c_str());
  std::exit(2);
}

std::string JsonString(const std::string& s) {
  std::string out = "\"";
  for (char c : s) {
    if (c == '"' || c == '\\') {
      out.push_back('\\');
      out.push_back(c);
    } else if (static_cast<unsigned char>(c) < 0x20) {
      char buf[8];
      std::snprintf(buf, sizeof(buf), "\\u%04x", static_cast<unsigned>(c));
      out.append(buf);
    } else {
      out.push_back(c);
    }
  }
  return out + "\"";
}

void PushSensor(const SensorRow& row, GlueRunner* glue) {
  glue->PushDeviceMotion(row.motion);
  glue->PushDeviceOrientation(row.orientation);
}

// SLAM_REFACT_ACCURACY="key=value,..." (accuracy mode, ACC-1): enabled, tri, local_ba, window,
// iterations, seconds, huber, tilt, imu, acc_noise, acc_walk. Unknown keys abort.
void ApplyAccuracyEnv(EngineConfig* cfg) {
  const char* env = std::getenv("SLAM_REFACT_ACCURACY");
  if (env == nullptr || *env == '\0') return;
  AccuracyConfig& a = cfg->accuracy;
  std::istringstream in(env);
  std::string item;
  while (std::getline(in, item, ',')) {
    const size_t eq = item.find('=');
    if (eq == std::string::npos) Fail("SLAM_REFACT_ACCURACY: expected key=value in '" + item + "'");
    const std::string k = item.substr(0, eq);
    const std::string v = item.substr(eq + 1);
    char* end = nullptr;
    const double d = std::strtod(v.c_str(), &end);
    if (v.empty() || end == nullptr || *end != '\0' || !std::isfinite(d)) {
      Fail("SLAM_REFACT_ACCURACY: bad number in '" + item + "'");
    }
    if (k == "enabled") a.enabled = d != 0.0;
    else if (k == "tri") a.multiview_triangulation = d != 0.0;
    else if (k == "local_ba") a.local_ba = d != 0.0;
    else if (k == "window") a.ba_window = static_cast<int>(d);
    else if (k == "iterations") a.ba_max_iterations = static_cast<int>(d);
    else if (k == "seconds") a.ba_max_seconds = d;
    else if (k == "huber") a.ba_huber_px = d;
    else if (k == "cauchy") a.ba_cauchy = d != 0.0;
    else if (k == "outlier") a.ba_outlier_px = d;
    else if (k == "tilt") a.ba_tilt_sigma_rad = d;
    else if (k == "ba_ground") a.ba_ground_points = static_cast<int>(d);
    else if (k == "parallax") a.ba_min_parallax_deg = d;
    else if (k == "metric") a.metric_output = d != 0.0;
    else if (k == "vi") a.vi_prediction = d != 0.0;
    else if (k == "vi_gain") a.vi_velocity_gain = d;
    else if (k == "metric_window") a.metric_window_s = d;
    else if (k == "metric_prior") a.metric_prior_scale = d;
    else if (k == "metric_low") a.metric_low_band = d != 0.0;
    else if (k == "metric_lag") {
      if (!(std::fabs(d) <= 0.3)) Fail("SLAM_REFACT_ACCURACY: metric_lag must be in [-0.3, 0.3]");
      a.metric_fixed_lag_s = d;
    }
    else Fail("SLAM_REFACT_ACCURACY: unknown key '" + k + "'");
  }
  std::fprintf(stderr, "accuracy enabled=%d tri=%d local_ba=%d window=%d metric=%d\n", a.enabled,
               a.multiview_triangulation, a.local_ba, a.ba_window, a.metric_output);
}

int Run(int argc, char** argv) {
  std::string bundle_dir, out_dir;
  for (int k = 1; k + 1 < argc; k += 2) {
    if (std::strcmp(argv[k], "--bundle") == 0) {
      bundle_dir = argv[k + 1];
    } else if (std::strcmp(argv[k], "--out") == 0) {
      out_dir = argv[k + 1];
    } else {
      Fail(std::string("unknown argument ") + argv[k]);
    }
  }
  if (bundle_dir.empty() || out_dir.empty() || argc % 2 == 0) {
    Fail("usage: slam_refact_replay --bundle <dir> --out <new dir>");
  }
  ReplayBundle bundle;
  std::string error;
  if (!LoadBundle(bundle_dir, &bundle, &error)) Fail(error);
  if (mkdir(out_dir.c_str(), 0775) != 0) {
    Fail("cannot create " + out_dir + " (a new run directory is required; runs are never overwritten)");
  }
  const int sample_every = std::atoi(bundle.config.at("world_point_sample_every").c_str());
  TraceWriter writer;
  if (!writer.Open(out_dir, sample_every, &error)) Fail(error);
  std::FILE* hash_file = std::fopen((out_dir + "/maphash.jsonl").c_str(), "wx");
  if (hash_file == nullptr) Fail("cannot create maphash.jsonl");
  // A-3: engine equivalents of the original memprobe / poke tracker fields + stage log (trkprobe.cc).
  std::FILE* trk_file = std::fopen((out_dir + "/trkprobe.jsonl").c_str(), "wx");
  if (trk_file == nullptr) Fail("cannot create trkprobe.jsonl");

  // Optional id_generator_seed: the recorded original run's id generator seed (make_variant
  // --capture-id-seed), so point / keyframe ids and id-ordered iterations are the original's.
  const auto seed_it = bundle.config.find("id_generator_seed");
  if (seed_it != bundle.config.end()) {
    char* end = nullptr;
    const unsigned long long seed = std::strtoull(seed_it->second.c_str(), &end, 10);
    if (end == seed_it->second.c_str() || *end != '\0' || seed > 0xffffffffull) {
      Fail("engine-config.tsv: bad id_generator_seed '" + seed_it->second + "'");
    }
    slam_refact::diagnostics::SetReplayIdGeneratorSeed(static_cast<uint32_t>(seed));
    std::fprintf(stderr, "id_generator_seed %llu\n", seed);
  }
  EngineConfig engine_config = bundle.MakeEngineConfig();
  ApplyAccuracyEnv(&engine_config);
  GlueConfig glue_config = bundle.MakeGlueConfig();
  const char* surf_env = std::getenv("SLAM_REFACT_SURFACES");
  const bool surfaces = surf_env != nullptr && std::string(surf_env) == "1";
  glue_config.detect_surfaces = surfaces;
  GlueRunner glue(engine_config, glue_config);
  {  // Applied accuracy / surface configuration, next to the trace (provenance).
    const AccuracyConfig& a = engine_config.accuracy;
    std::FILE* f = std::fopen((out_dir + "/accuracy-config.json").c_str(), "w");
    if (f == nullptr) Fail("cannot write accuracy-config.json");
    std::fprintf(f,
                 "{\"enabled\":%s,\"metric_output\":%s,\"metric_window_s\":%.17g,"
                 "\"metric_prior_scale\":%.17g,\"local_ba\":%s,\"multiview_triangulation\":%s,"
                 "\"ba_window\":%d,\"ba_ground_points\":%d,\"vi_prediction\":%s,"
                 "\"vi_velocity_gain\":%.17g,\"metric_low_band\":%s,\"metric_fixed_lag_s\":%s,"
                 "\"detect_surfaces\":%s}\n",
                 a.enabled ? "true" : "false", a.metric_output ? "true" : "false", a.metric_window_s,
                 a.metric_prior_scale, a.local_ba ? "true" : "false",
                 a.multiview_triangulation ? "true" : "false", a.ba_window, a.ba_ground_points,
                 a.vi_prediction ? "true" : "false", a.vi_velocity_gain,
                 a.metric_low_band ? "true" : "false",
                 std::isfinite(a.metric_fixed_lag_s) ? std::to_string(a.metric_fixed_lag_s).c_str() : "null",
                 glue_config.detect_surfaces ? "true" : "false");
    std::fclose(f);
  }
  const auto wall_start = std::chrono::steady_clock::now();

  size_t next_sensor = 0;
  LoadedImages images;
  // 1. bootstrap stage (D-4): output discarded.
  while (next_sensor < bundle.sensors.size() && bundle.sensors[next_sensor].bootstrap) {
    PushSensor(bundle.sensors[next_sensor++], &glue);
  }
  const FrameRow& boot = bundle.frames.front();
  if (!LoadImages(boot, &images, &error)) Fail(error);
  {
    const FrameInput in = MakeFrameInput(boot, images);
    glue.ProcessFrame(in.time_ns, in.video_time_ns, in.gray, in.pyramid);
  }
  trkprobe::ResetFrame();  // the bootstrap stage's tracker calls are not part of any input frame
  // 2. input frames.
  int dropped = 0;
  int injected = 0;
  for (size_t f = 1; f < bundle.frames.size(); ++f) {
    const FrameRow& frame = bundle.frames[f];
    while (next_sensor < bundle.sensors.size() &&
           bundle.sensors[next_sensor].motion.receive_time_ns <= frame.time_ns) {
      PushSensor(bundle.sensors[next_sensor++], &glue);
    }
    if (!frame.process) {
      writer.WriteDropped(frame);
      ++dropped;
      continue;
    }
    if (!LoadImages(frame, &images, &error)) Fail(error);
    injected += images.atlas.empty() ? 0 : 1;
    const FrameInput in = MakeFrameInput(frame, images);
    const PublishedFrame out = glue.ProcessFrame(in.time_ns, in.video_time_ns, in.gray, in.pyramid);
    if (surfaces) {
      writer.SetSurfaces(glue.surfaces(), glue.HitTest(0.5f, 0.5f, 7u));
    }
    writer.WriteProcessed(frame, out);
    WriteMapHash(hash_file, frame.index);
    trkprobe::EmitFrame(trk_file, frame.index);
  }
  const double wall_s =
      std::chrono::duration<double>(std::chrono::steady_clock::now() - wall_start).count();
  const int rows = writer.rows();
  const int processed = writer.processed();
  if (!writer.Close(&error)) Fail(error);
  std::fclose(hash_file);
  std::fclose(trk_file);

  std::string meta = "{\n \"schema\": \"slam-refact-engine-run-v1\",\n";
  meta += " \"bundle\": " + JsonString(bundle.dir) + ",\n";
  meta += " \"contractVersion\": " + std::to_string(kContractVersion) + ",\n";
  meta += " \"engineConfig\": {";
  bool first = true;
  for (const auto& kv : bundle.config) {
    meta += std::string(first ? "" : ", ") + JsonString(kv.first) + ": " + JsonString(kv.second);
    first = false;
  }
  meta += "},\n";
  meta += " \"bootstrap\": {\"timeNs\": " + std::to_string(boot.time_ns) +
          ", \"injectedPyramid\": " + (boot.pyramid.empty() ? "false" : "true") + "},\n";
  meta += " \"rows\": " + std::to_string(rows) + ",\n \"processedFrames\": " +
          std::to_string(processed) + ",\n \"droppedFrames\": " + std::to_string(dropped) +
          ",\n \"injectedPyramidFrames\": " + std::to_string(injected) + ",\n \"sensorRows\": " +
          std::to_string(bundle.sensors.size()) + ",\n \"sensorRowsPushed\": " +
          std::to_string(next_sensor) + ",\n \"wallSeconds\": " + JsNumber(wall_s) + ",\n";
  meta += " \"fresh\": \"input frame processed (not drop-masked); startup re-publication has srRepeated true\"\n}\n";
  std::FILE* m = std::fopen((out_dir + "/meta.json").c_str(), "wx");
  if (m == nullptr) Fail("cannot create meta.json");
  std::fputs(meta.c_str(), m);
  std::fclose(m);
  std::printf("rows=%d processed=%d dropped=%d injected=%d wall=%.1fs out=%s\n", rows, processed,
              dropped, injected, wall_s, out_dir.c_str());
  return 0;
}

}  // namespace
}  // namespace slam_refact::harness

int main(int argc, char** argv) { return slam_refact::harness::Run(argc, argv); }

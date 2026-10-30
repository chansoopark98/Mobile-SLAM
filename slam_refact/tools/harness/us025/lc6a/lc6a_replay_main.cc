// slam_refact_replay_lc6a: US-025 LC-6a dev/report driver (not a gate tool). hand-made copy of
// tools/harness/replay/replay_main.cc (same bootstrap, sensor pushes, drop mask, id seed, trace; `diff` shows
// only the hunks marked LC6A below) that additionally writes <out>/lc6a-attempts.jsonl: one line per call of
// map_builder::LoopClose (a keyframe creation with MapBuilder.loopCloseEnabled on), recorded through a linker
// --wrap of that function (see build_lc6a.sh); no engine/src file is changed and the wrapped call is made with
// the caller's arguments (a report object of our own is added when the caller passed none).
//
//   slam_refact_replay_lc6a --bundle <bundle dir> --out <new run dir>
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
#include "map/world_map.h"             // LC6A
#include "map_builder/loop_close.h"      // LC6A
#include "slam_refact/glue_runner.h"
#include "slam_refact/replay_diagnostics.h"
#include "slam_refact/version.h"
#include "trace_writer.h"

// ---- LC6A: LoopClose capture (linker --wrap, see build_lc6a.sh) --------------------------------------
namespace {
int g_frame = -1;              // input frame index of the ProcessFrame in progress (-1 = bootstrap stage)
std::FILE* g_attempts = nullptr;

std::string Hex(const void* p, size_t n) {
  static const char* d = "0123456789abcdef";
  const unsigned char* b = static_cast<const unsigned char*>(p);
  std::string s;
  s.reserve(n * 2);
  for (size_t i = 0; i < n; ++i) {
    s.push_back(d[b[i] >> 4]);
    s.push_back(d[b[i] & 15]);
  }
  return s;
}

// HMatrix as the original's 0x80 leading bytes: forward float[16] then inverse float[16] (flags not exposed, N5).
std::string HexMatrix(const c8::HMatrix& m) {
  return Hex(m.data().data(), 64) + Hex(m.inverseData().data(), 64);
}
}  // namespace

extern "C" {
bool __real__ZN11slam_refact11map_builder9LoopCloseERNS_3map8WorldMapEPNS0_14LoopCloseStateEPNS_16MapBuilderOutputEPNS0_15LoopCloseReportE(
    slam_refact::map::WorldMap*, slam_refact::map_builder::LoopCloseState*, slam_refact::MapBuilderOutput*,
    slam_refact::map_builder::LoopCloseReport*);
bool __wrap__ZN11slam_refact11map_builder9LoopCloseERNS_3map8WorldMapEPNS0_14LoopCloseStateEPNS_16MapBuilderOutputEPNS0_15LoopCloseReportE(
    slam_refact::map::WorldMap* wm, slam_refact::map_builder::LoopCloseState* st, slam_refact::MapBuilderOutput* out,
    slam_refact::map_builder::LoopCloseReport* rep) {
  using namespace slam_refact;
  map_builder::LoopCloseReport own;
  if (rep == nullptr) rep = &own;
  const size_t n_slots = wm->NumKeyframeSlots();
  const map::KeyframeId newest = wm->newest();
  std::string slots = "[";
  std::string desc = "[";
  size_t live = 0;
  for (size_t s = 0; s < n_slots; ++s) {
    const map::Keyframe& k = wm->KeyframeAtSlot(s);
    live += k.id != 0 ? 1 : 0;
    slots += (s ? "," : "") + std::to_string(k.id);
    desc += std::string(s ? ",\"" : "\"") + Hex(k.global_descriptor.data(), 32) + "\"";
  }
  slots += "]";
  desc += "]";
  char ring_pre[160];
  std::snprintf(ring_pre, sizeof(ring_pre), "[%u,%u,%u,%u,%u,%u]", static_cast<unsigned>(st->last_newest.index()),
                static_cast<unsigned>(st->last_newest.id()), st->ring[0], st->ring[1], st->ring[2], st->cursor);
  const bool newest_ok = newest.index() < n_slots && wm->IsKeyframeValid(newest);
  const std::string pose_before = newest_ok ? HexMatrix(wm->GetKeyframe(newest).frame.extrinsic()) : std::string();
  const bool ok =
      __real__ZN11slam_refact11map_builder9LoopCloseERNS_3map8WorldMapEPNS0_14LoopCloseStateEPNS_16MapBuilderOutputEPNS0_15LoopCloseReportE(
          wm, st, out, rep);
  char ring_post[160];
  std::snprintf(ring_post, sizeof(ring_post), "[%u,%u,%u,%u,%u,%u]", static_cast<unsigned>(st->last_newest.index()),
                static_cast<unsigned>(st->last_newest.id()), st->ring[0], st->ring[1], st->ring[2], st->cursor);
  const bool post_ok = newest.index() < wm->NumKeyframeSlots() && wm->IsKeyframeValid(newest);
  std::fprintf(g_attempts,
               "{\"frame\":%d,\"nSlots\":%zu,\"nLive\":%zu,\"newest\":[%u,%u],\"slots\":%s,\"desc\":%s,"
               "\"ringPre\":%s,\"ringPost\":%s,\"outcome\":%d,\"applied\":%s,\"outLcOk\":%d,"
               "\"cand\":[%u,%u],\"candDist\":%.9g,\"nMatches\":%zu,\"nCorr\":%zu,\"nUnlinked\":%zu,"
               "\"poseBefore\":\"%s\",\"poseAfter\":\"%s\",\"C\":\"%s\"}\n",
               g_frame, n_slots, live, static_cast<unsigned>(newest.index()), static_cast<unsigned>(newest.id()),
               slots.c_str(), desc.c_str(), ring_pre, ring_post, static_cast<int>(rep->outcome), ok ? "true" : "false",
               out->world_transform_updated ? 1 : 0, static_cast<unsigned>(rep->candidate.index()),
               static_cast<unsigned>(rep->candidate.id()), static_cast<double>(rep->candidate_distance),
               rep->num_matches, rep->num_correspondences, rep->num_unlinked, pose_before.c_str(),
               (post_ok ? HexMatrix(wm->GetKeyframe(newest).frame.extrinsic()) : std::string()).c_str(),
               ok ? HexMatrix(out->world_correction).c_str() : "");
  std::fflush(g_attempts);
  return ok;
}
}

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
  g_attempts = std::fopen((out_dir + "/lc6a-attempts.jsonl").c_str(), "wx");  // LC6A
  if (g_attempts == nullptr) Fail("cannot create lc6a-attempts.jsonl");       // LC6A

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
    g_frame = frame.index;  // LC6A
    const PublishedFrame out = glue.ProcessFrame(in.time_ns, in.video_time_ns, in.gray, in.pyramid);
    if (surfaces) {
      writer.SetSurfaces(glue.surfaces(), glue.HitTest(0.5f, 0.5f, 7u));
    }
    writer.WriteProcessed(frame, out);
  }
  const double wall_s =
      std::chrono::duration<double>(std::chrono::steady_clock::now() - wall_start).count();
  const int rows = writer.rows();
  const int processed = writer.processed();
  if (!writer.Close(&error)) Fail(error);
  std::fclose(g_attempts);  // LC6A

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

// slam_refact_replay: drives GlueRunner with a replay bundle and writes a trace in the schema of
// the original headless reference runtime trace (architecture.md §6.1-6.3, §4.4 bootstrap, WP5).
//
//   slam_refact_replay --bundle <bundle dir> --out <new run dir>
//   slam_refact_replay --bundle <bundle dir> --timing-csv <new file>
//
// The second form is the production timing path of the P2 candidate gate G7: the same engine calls,
// no trace and no counters, only the wall time of each processed frame (ProcessFrame, measured
// outside the engine) as `frame,frame_ms` rows; the options come from SLAM_REFACT_ACCURACY
// (enabled=1,f1=0/1,f2=0/1) so that OFF and ON runs differ only in the config field.
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
#include <utility>
#include <vector>

#include "bundle_reader.h"
#include "slam_refact/accuracy_counters.h"
#include "slam_refact/glue_runner.h"
#include "slam_refact/replay_diagnostics.h"
#include "slam_refact/version.h"
#include "trace_writer.h"

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
// iterations, seconds, huber, tilt, imu, acc_noise, acc_walk, and the P2 candidates f1, f2.
// Unknown keys abort.
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
    else if (k == "f1") a.f1_ground_plane = d != 0.0;
    else if (k == "f2") a.f2_ground_confirmation = d != 0.0;
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

// G7 production timing path: the replay loop without TraceWriter, accuracy-config.json, counters or
// any probe. Each ProcessFrame call is timed from the outside; the row is written after the loop.
int RunTiming(const std::string& bundle_dir, const std::string& csv_path) {
  ReplayBundle bundle;
  std::string error;
  if (!LoadBundle(bundle_dir, &bundle, &error)) Fail(error);
  const auto seed_it = bundle.config.find("id_generator_seed");
  if (seed_it != bundle.config.end()) {
    char* end = nullptr;
    const unsigned long long seed = std::strtoull(seed_it->second.c_str(), &end, 10);
    if (end == seed_it->second.c_str() || *end != '\0' || seed > 0xffffffffull) {
      Fail("engine-config.tsv: bad id_generator_seed '" + seed_it->second + "'");
    }
    slam_refact::diagnostics::SetReplayIdGeneratorSeed(static_cast<uint32_t>(seed));
  }
  EngineConfig engine_config = bundle.MakeEngineConfig();
  ApplyAccuracyEnv(&engine_config);
  GlueConfig glue_config = bundle.MakeGlueConfig();
  GlueRunner glue(engine_config, glue_config);

  size_t next_sensor = 0;
  LoadedImages images;
  while (next_sensor < bundle.sensors.size() && bundle.sensors[next_sensor].bootstrap) {
    PushSensor(bundle.sensors[next_sensor++], &glue);
  }
  const FrameRow& boot = bundle.frames.front();
  if (!LoadImages(boot, &images, &error)) Fail(error);
  {
    const FrameInput in = MakeFrameInput(boot, images);
    glue.ProcessFrame(in.time_ns, in.video_time_ns, in.gray, in.pyramid);
  }
  std::vector<std::pair<int, double>> rows;  // (frame index, ms)
  for (size_t f = 1; f < bundle.frames.size(); ++f) {
    const FrameRow& frame = bundle.frames[f];
    while (next_sensor < bundle.sensors.size() &&
           bundle.sensors[next_sensor].motion.receive_time_ns <= frame.time_ns) {
      PushSensor(bundle.sensors[next_sensor++], &glue);
    }
    if (!frame.process) continue;
    if (!LoadImages(frame, &images, &error)) Fail(error);
    const FrameInput in = MakeFrameInput(frame, images);
    const auto t0 = std::chrono::steady_clock::now();
    const PublishedFrame out = glue.ProcessFrame(in.time_ns, in.video_time_ns, in.gray, in.pyramid);
    const auto t1 = std::chrono::steady_clock::now();
    (void)out;
    rows.emplace_back(frame.index, std::chrono::duration<double, std::milli>(t1 - t0).count());
  }
  std::FILE* f = std::fopen(csv_path.c_str(), "wx");
  if (f == nullptr) Fail("cannot create " + csv_path + " (timing files are never overwritten)");
  std::fputs("frame,frame_ms\n", f);
  for (const auto& r : rows) std::fprintf(f, "%d,%.6f\n", r.first, r.second);
  std::fclose(f);
  std::printf("timing rows=%zu out=%s\n", rows.size(), csv_path.c_str());
  return 0;
}

// f2-counters.jsonl: cumulative AccuracyCounters after each processed frame (only written when F1
// or F2 is on; the analysis takes differences). Not part of the trace.
void WriteCounters(std::FILE* f, int frame_index) {
  const AccuracyCounters& c = Counters();
  auto arr = [](const auto& a) {
    std::string s = "[";
    for (size_t i = 0; i < a.size(); ++i) s += (i ? "," : "") + std::to_string(a[i]);
    return s + "]";
  };
  std::fprintf(f,
               "{\"f\":%d,\"f2_eval\":%s,\"f2_no_creator\":%llu,\"f2_newly_confirmed\":%llu,"
               "\"f2_hold_frames\":%llu,\"f2_hold_points\":%llu,\"f2_id_reuse\":%llu,"
               "\"f2_path_kf\":%llu,\"f2_path_nonkf\":%llu,\"f2_dirty_remainder\":%llu,"
               "\"f2_invalidation\":%s,\"sel_calls\":%llu,\"sel_active\":%llu,"
               "\"sel_cap_reached\":%llu,\"sel_bonus_granted\":%llu,\"sel_all_ground\":%s,"
               "\"sel_subset_ground\":%s,\"sel_new_confirmed_all\":%llu,"
               "\"sel_new_confirmed_subset\":%llu,\"used_state_before_epoch\":%llu,"
               "\"f1_applied\":%llu,\"f1_fallback\":%s,\"f1_kf_applied\":%llu,"
               "\"f1_kf_fallback\":%llu}\n",
               frame_index, arr(c.f2_eval).c_str(), static_cast<unsigned long long>(c.f2_no_creator),
               static_cast<unsigned long long>(c.f2_newly_confirmed),
               static_cast<unsigned long long>(c.f2_hold_frames),
               static_cast<unsigned long long>(c.f2_hold_points),
               static_cast<unsigned long long>(c.f2_id_reuse),
               static_cast<unsigned long long>(c.f2_path_kf),
               static_cast<unsigned long long>(c.f2_path_nonkf),
               static_cast<unsigned long long>(c.f2_dirty_remainder), arr(c.f2_invalidation).c_str(),
               static_cast<unsigned long long>(c.sel_calls), static_cast<unsigned long long>(c.sel_active),
               static_cast<unsigned long long>(c.sel_cap_reached),
               static_cast<unsigned long long>(c.sel_bonus_granted), arr(c.sel_all_ground).c_str(),
               arr(c.sel_subset_ground).c_str(), static_cast<unsigned long long>(c.sel_new_confirmed_all),
               static_cast<unsigned long long>(c.sel_new_confirmed_subset),
               static_cast<unsigned long long>(c.used_state_before_epoch),
               static_cast<unsigned long long>(c.f1_applied), arr(c.f1_fallback).c_str(),
               static_cast<unsigned long long>(c.f1_kf_applied),
               static_cast<unsigned long long>(c.f1_kf_fallback));
}

int Run(int argc, char** argv) {
  std::string bundle_dir, out_dir, timing_csv;
  for (int k = 1; k + 1 < argc; k += 2) {
    if (std::strcmp(argv[k], "--bundle") == 0) {
      bundle_dir = argv[k + 1];
    } else if (std::strcmp(argv[k], "--out") == 0) {
      out_dir = argv[k + 1];
    } else if (std::strcmp(argv[k], "--timing-csv") == 0) {
      timing_csv = argv[k + 1];
    } else {
      Fail(std::string("unknown argument ") + argv[k]);
    }
  }
  if (bundle_dir.empty() || (out_dir.empty() == timing_csv.empty()) || argc % 2 == 0) {
    Fail("usage: slam_refact_replay --bundle <dir> (--out <new dir> | --timing-csv <new file>)");
  }
  if (!timing_csv.empty()) {
    return RunTiming(bundle_dir, timing_csv);
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
                 "\"detect_surfaces\":%s,\"f1_ground_plane\":%s,\"f2_ground_confirmation\":%s}\n",
                 a.enabled ? "true" : "false", a.metric_output ? "true" : "false", a.metric_window_s,
                 a.metric_prior_scale, a.local_ba ? "true" : "false",
                 a.multiview_triangulation ? "true" : "false", a.ba_window, a.ba_ground_points,
                 a.vi_prediction ? "true" : "false", a.vi_velocity_gain,
                 a.metric_low_band ? "true" : "false",
                 std::isfinite(a.metric_fixed_lag_s) ? std::to_string(a.metric_fixed_lag_s).c_str() : "null",
                 glue_config.detect_surfaces ? "true" : "false",
                 a.f1_ground_plane ? "true" : "false", a.f2_ground_confirmation ? "true" : "false");
    std::fclose(f);
  }
  const auto wall_start = std::chrono::steady_clock::now();
  std::FILE* counters_file = nullptr;
  if (engine_config.accuracy.enabled &&
      (engine_config.accuracy.f1_ground_plane || engine_config.accuracy.f2_ground_confirmation)) {
    counters_file = std::fopen((out_dir + "/f2-counters.jsonl").c_str(), "wx");
    if (counters_file == nullptr) Fail("cannot create f2-counters.jsonl");
    ResetCounters();
  }

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
    const PublishedFrame out = glue.ProcessFrame(in.time_ns, in.video_time_ns, in.gray, in.pyramid);
    if (surfaces) {
      writer.SetSurfaces(glue.surfaces(), glue.HitTest(0.5f, 0.5f, 7u));
    }
    writer.WriteProcessed(frame, out);
    if (counters_file != nullptr) WriteCounters(counters_file, frame.index);
  }
  if (counters_file != nullptr) std::fclose(counters_file);
  const double wall_s =
      std::chrono::duration<double>(std::chrono::steady_clock::now() - wall_start).count();
  const int rows = writer.rows();
  const int processed = writer.processed();
  if (!writer.Close(&error)) Fail(error);

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

// sr_or4_replay (OR-4, full-parity plan Rev B; gate or4-it-world-regression.v5): the exact-input replay of tools/harness/replay/replay_main.cc
// with two additions, so that the world SLAM of one bundle can be run with and without a loaded image target:
//
//   sr_or4_replay --bundle <dir> --out <new run dir> [--targets <dir>] [--id-seed <uint32>]
//
//   --targets <dir>  every <name>.json in the directory (sorted) is loaded with <name>_luminance.pgm through
//                    GlueRunner::SetImageTargets BEFORE the bootstrap stage (the original configures
//                    imageTargetData before XR8.run). Without the option the call sequence is exactly
//                    replay_main.cc's (the gate G-H compares the two binaries' traces bit for bit).
//   --mutation <m>   gyro-bias:<deg/s> | drop-every:<n>: a registered world-regression mutation (gate section 7).
//   --rng-seed <n>   seed of the shared XREngine RandomNumbers (diagnostics::SetReplayEngineRngSeed; default 5489): the
//                    benign "other RNG stream" null of the gate.
//   --force-orb-gorb 1  forces allOrb = allGorb = true without any target (diagnostics::SetReplayForceAllOrbGorb):
//                    the descriptor-configuration part of loading a target (decomposition run, gate v5 section 8).
//   --world-tracker 0|1  diagnostics::SetImageWorldTracker before the engine is built. Without it the engine default
//                    applies (on since contract-changes.md WT-DEFAULT-ON; 0 = the public pre-US-015 behavior, the A/B arm).
//                    Not written to meta.json (G-H compares meta bit for bit with replay_main.cc); the arm is the command line.
//   --id-seed <n>    overrides the bundle's id_generator_seed (the benign run-to-run factor of the null-runs:
//                    the original draws a new seed per run; the registered seed set is in the gate file).
//
// Extra output next to trace.jsonl / worldpoints-sampled.jsonl / meta.json:
//   images.jsonl   one line per processed frame: detected images, image events, located-image debug, the
//                  ROI slots the next staged frame will draw. Meta-only; no gate reads it for pose.
// One scenario per process (the engine keeps process-global state). Development harness, not product code.

#include <sys/stat.h>

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include "bundle_reader.h"
#include "slam_refact/glue_runner.h"
#include "slam_refact/replay_diagnostics.h"
#include "slam_refact/version.h"
#include "trace_writer.h"

namespace slam_refact::harness {
namespace {

[[noreturn]] void Fail(const std::string& msg) {
  std::fprintf(stderr, "sr_or4_replay: %s\n", msg.c_str());
  std::exit(2);
}

// Registered mutations (gate or4-it-world-regression.v5 section 7; they model a world regression of unknown origin
// and are applied to a run without targets so no target data is needed to prove the gate's detection power):
//   gyro-bias:<deg/s>  adds the value to the alpha component of every DeviceMotion rotationRate (v2: no effect on most bundles;
//                      the engine only gates on |rotationRate| thresholds)
//   orient-beta:<deg>  adds the value to the beta (pitch) of every DeviceOrientation sample
//   orient-drift:<deg/s> adds value * (t - t_first) to the alpha (yaw) of every DeviceOrientation sample (slow yaw drift)
//   drop-every:<n>     treats every n-th input frame (index % n == 0) as dropped (not processed)
struct Mutation {
  double gyro_bias_dps = 0.0;
  double orient_beta_deg = 0.0;
  double orient_drift_dps = 0.0;
  int drop_every = 0;
  std::string text;
};

void PushSensor(const SensorRow& row, const Mutation& mut, GlueRunner* glue) {
  static int64_t first_ns = -1;
  if (first_ns < 0) first_ns = row.orientation.receive_time_ns;
  DeviceMotionSample motion = row.motion;
  if (mut.gyro_bias_dps != 0.0 && motion.rotation_rate_abg.has_value()) {
    motion.rotation_rate_abg->x += static_cast<float>(mut.gyro_bias_dps);
  }
  DeviceOrientationSample orientation = row.orientation;
  orientation.beta_deg += mut.orient_beta_deg;
  orientation.alpha_deg += mut.orient_drift_dps * static_cast<double>(orientation.receive_time_ns - first_ns) * 1e-9;
  glue->PushDeviceMotion(motion);
  glue->PushDeviceOrientation(orientation);
}

struct LoadedTarget {
  std::string name;
  std::string json;
  std::vector<uint8_t> pixels;
  int width = 0;
  int height = 0;
};

std::string ReadFile(const std::string& path) {
  std::ifstream in(path, std::ios::binary);
  if (!in) Fail("cannot read " + path);
  std::ostringstream s;
  s << in.rdbuf();
  return s.str();
}

std::vector<LoadedTarget> LoadTargets(const std::string& dir) {
  std::vector<std::string> jsons;
  for (const auto& e : std::filesystem::directory_iterator(dir)) {
    if (e.path().extension() == ".json") jsons.push_back(e.path().string());
  }
  std::sort(jsons.begin(), jsons.end());
  if (jsons.empty()) Fail("--targets: no *.json in " + dir);
  std::vector<LoadedTarget> out;
  for (const std::string& j : jsons) {
    LoadedTarget t;
    t.json = ReadFile(j);
    t.name = std::filesystem::path(j).stem().string();
    const std::string pgm = (std::filesystem::path(j).parent_path() / (t.name + "_luminance.pgm")).string();
    std::string error;
    if (!ReadPgm(pgm, &t.pixels, &t.width, &t.height, &error)) Fail(error);
    out.push_back(std::move(t));
  }
  return out;
}

std::string Num(double v) { return JsNumber(v); }

// With a target loaded the engine requires an injected atlas to carry the ROI slots (PyramidAtlasRoiRows,
// pyramid_source.cc GetWithRois). The recorded atlases come from original runs without targets and end below
// that row count; the missing rows are appended as zeros = slots a producer drew nothing into (no ROI
// information). Counted in meta.json (paddedAtlases). Only used with --targets.
void PadRoiRows(LoadedImages* images, int roi_rows, int* padded) {
  if (images->atlas.empty() || images->atlas_rows >= roi_rows) return;
  images->atlas.resize(static_cast<size_t>(roi_rows) * 1024 * 4, 0);
  images->atlas_rows = roi_rows;
  ++*padded;
}

std::string ImagesLine(const FrameRow& frame, const PublishedFrame& out, const GlueRunner& glue) {
  std::string s = "{\"i\":" + std::to_string(frame.index) +
                  ",\"srcFrame\":" + std::to_string(out.source_frame_index) +
                  ",\"repeated\":" + (out.repeated ? "true" : "false");
  s += ",\"detected\":[";
  for (size_t k = 0; k < out.detected_images.size(); ++k) {
    const DetectedImageOut& d = out.detected_images[k];
    s += std::string(k ? "," : "") + "{\"name\":\"" + d.name + "\",\"p\":[" + Num(d.position.x) + "," +
         Num(d.position.y) + "," + Num(d.position.z) + "],\"q\":[" + Num(d.rotation.w) + "," +
         Num(d.rotation.x) + "," + Num(d.rotation.y) + "," + Num(d.rotation.z) + "]}";
  }
  s += "],\"events\":[";
  for (size_t k = 0; k < out.image_events.size(); ++k) {
    s += std::string(k ? "," : "") + "{\"kind\":" + std::to_string(static_cast<int>(out.image_events[k].kind)) +
         ",\"name\":\"" + out.image_events[k].image.name + "\"}";
  }
  s += "],\"located\":[";
  if (out.step.has_value()) {
    const auto& li = out.step->located_images;
    for (size_t k = 0; k < li.size(); ++k) {
      s += std::string(k ? "," : "") + "{\"name\":\"" + li[k].name + "\",\"status\":" + std::to_string(li[k].status) +
           ",\"lastSeen\":" + std::to_string(li[k].last_seen) + ",\"firstSeen\":" + std::to_string(li[k].first_seen) +
           ",\"roiFeats\":" + (li[k].roi_feats ? "true" : "false") + "}";
    }
  }
  const std::vector<PyramidRoiRequest> rois = glue.NextDrawRois();
  s += "],\"nextRois\":" + std::to_string(rois.size()) + "}";
  return s;
}

int Run(int argc, char** argv) {
  std::string bundle_dir, out_dir, targets_dir;
  bool have_seed = false;
  uint32_t seed_override = 0;
  Mutation mut;
  std::string rng_seed_text;
  bool force_orb_gorb = false;
  for (int k = 1; k + 1 < argc; k += 2) {
    if (std::strcmp(argv[k], "--bundle") == 0) {
      bundle_dir = argv[k + 1];
    } else if (std::strcmp(argv[k], "--out") == 0) {
      out_dir = argv[k + 1];
    } else if (std::strcmp(argv[k], "--targets") == 0) {
      targets_dir = argv[k + 1];
    } else if (std::strcmp(argv[k], "--mutation") == 0) {
      const std::string v = argv[k + 1];
      mut.text = v;
      char* end = nullptr;
      if (v.rfind("gyro-bias:", 0) == 0) {
        mut.gyro_bias_dps = std::strtod(v.c_str() + 10, &end);
        if (*end != '\0' || mut.gyro_bias_dps == 0.0) Fail("bad --mutation " + v);
      } else if (v.rfind("orient-beta:", 0) == 0) {
        mut.orient_beta_deg = std::strtod(v.c_str() + 12, &end);
        if (*end != '\0' || mut.orient_beta_deg == 0.0) Fail("bad --mutation " + v);
      } else if (v.rfind("orient-drift:", 0) == 0) {
        mut.orient_drift_dps = std::strtod(v.c_str() + 13, &end);
        if (*end != '\0' || mut.orient_drift_dps == 0.0) Fail("bad --mutation " + v);
      } else if (v.rfind("drop-every:", 0) == 0) {
        mut.drop_every = static_cast<int>(std::strtol(v.c_str() + 11, &end, 10));
        if (*end != '\0' || mut.drop_every < 2) Fail("bad --mutation " + v);
      } else {
        Fail("unknown --mutation " + v);
      }
    } else if (std::strcmp(argv[k], "--rng-seed") == 0) {
      char* end = nullptr;
      const unsigned long long v = std::strtoull(argv[k + 1], &end, 10);
      if (end == argv[k + 1] || *end != '\0' || v > 0xffffffffull) Fail("bad --rng-seed");
      slam_refact::diagnostics::SetReplayEngineRngSeed(static_cast<uint32_t>(v));
      rng_seed_text = argv[k + 1];
    } else if (std::strcmp(argv[k], "--force-orb-gorb") == 0) {
      force_orb_gorb = std::strcmp(argv[k + 1], "1") == 0;
      if (!force_orb_gorb && std::strcmp(argv[k + 1], "0") != 0) Fail("bad --force-orb-gorb");
      slam_refact::diagnostics::SetReplayForceAllOrbGorb(force_orb_gorb);
    } else if (std::strcmp(argv[k], "--world-tracker") == 0) {
      const bool on = std::strcmp(argv[k + 1], "1") == 0;
      if (!on && std::strcmp(argv[k + 1], "0") != 0) Fail("bad --world-tracker");
      slam_refact::diagnostics::SetImageWorldTracker(on);
    } else if (std::strcmp(argv[k], "--id-seed") == 0) {
      char* end = nullptr;
      const unsigned long long v = std::strtoull(argv[k + 1], &end, 10);
      if (end == argv[k + 1] || *end != '\0' || v > 0xffffffffull) Fail("bad --id-seed");
      seed_override = static_cast<uint32_t>(v);
      have_seed = true;
    } else {
      Fail(std::string("unknown argument ") + argv[k]);
    }
  }
  if (bundle_dir.empty() || out_dir.empty() || argc % 2 == 0) {
    Fail("usage: sr_or4_replay --bundle <dir> --out <new dir> [--targets <dir>] [--id-seed <n>] [--rng-seed <n>] [--force-orb-gorb 1] [--world-tracker 0|1] [--mutation gyro-bias:<dps>|orient-beta:<deg>|orient-drift:<dps>|drop-every:<n>]");
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

  uint32_t used_seed = 0;
  bool seeded = false;
  const auto seed_it = bundle.config.find("id_generator_seed");
  if (have_seed) {
    used_seed = seed_override;
    seeded = true;
  } else if (seed_it != bundle.config.end()) {
    char* end = nullptr;
    const unsigned long long seed = std::strtoull(seed_it->second.c_str(), &end, 10);
    if (end == seed_it->second.c_str() || *end != '\0' || seed > 0xffffffffull) {
      Fail("engine-config.tsv: bad id_generator_seed '" + seed_it->second + "'");
    }
    used_seed = static_cast<uint32_t>(seed);
    seeded = true;
  }
  if (seeded) {
    slam_refact::diagnostics::SetReplayIdGeneratorSeed(used_seed);
    std::fprintf(stderr, "id_generator_seed %u\n", used_seed);
  }
  EngineConfig engine_config = bundle.MakeEngineConfig();
  GlueConfig glue_config = bundle.MakeGlueConfig();
  GlueRunner glue(engine_config, glue_config);

  std::vector<LoadedTarget> targets;
  double target_load_ms = 0.0;
  if (!targets_dir.empty()) {
    targets = LoadTargets(targets_dir);
    std::vector<ImageTargetInput> inputs;
    for (const LoadedTarget& t : targets) {
      ImageTargetInput in;
      in.json = t.json;
      in.luminance = GrayImageView{t.pixels.data(), t.width, t.height, t.width};
      inputs.push_back(in);
    }
    const auto t0 = std::chrono::steady_clock::now();
    const std::string err = glue.SetImageTargets(inputs);
    target_load_ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
    if (!err.empty()) Fail("SetImageTargets: " + err);
    std::fprintf(stderr, "image targets loaded: %zu (%.0f ms)\n", glue.image_target_names().size(), target_load_ms);
  }
  std::FILE* images = std::fopen((out_dir + "/images.jsonl").c_str(), "wx");
  if (images == nullptr) Fail("cannot create images.jsonl");

  const auto wall_start = std::chrono::steady_clock::now();
  size_t next_sensor = 0;
  LoadedImages loaded;
  while (next_sensor < bundle.sensors.size() && bundle.sensors[next_sensor].bootstrap) {
    PushSensor(bundle.sensors[next_sensor++], mut, &glue);
  }
  const int roi_rows = PyramidAtlasRoiRows(engine_config.capture_width, engine_config.capture_height);
  int padded = 0;
  const FrameRow& boot = bundle.frames.front();
  if (!LoadImages(boot, &loaded, &error)) Fail(error);
  if (!targets.empty()) PadRoiRows(&loaded, roi_rows, &padded);
  {
    const FrameInput in = MakeFrameInput(boot, loaded);
    glue.ProcessFrame(in.time_ns, in.video_time_ns, in.gray, in.pyramid);
  }
  int dropped = 0, injected = 0, detected_frames = 0, event_count = 0, located_frames = 0, roi_frames = 0;
  for (size_t f = 1; f < bundle.frames.size(); ++f) {
    const FrameRow& frame = bundle.frames[f];
    while (next_sensor < bundle.sensors.size() &&
           bundle.sensors[next_sensor].motion.receive_time_ns <= frame.time_ns) {
      PushSensor(bundle.sensors[next_sensor++], mut, &glue);
    }
    if (!frame.process || (mut.drop_every > 0 && frame.index % mut.drop_every == 0)) {
      writer.WriteDropped(frame);
      ++dropped;
      continue;
    }
    if (!LoadImages(frame, &loaded, &error)) Fail(error);
    if (!targets.empty()) PadRoiRows(&loaded, roi_rows, &padded);
    injected += loaded.atlas.empty() ? 0 : 1;
    const FrameInput in = MakeFrameInput(frame, loaded);
    const PublishedFrame out = glue.ProcessFrame(in.time_ns, in.video_time_ns, in.gray, in.pyramid);
    writer.WriteProcessed(frame, out);
    detected_frames += out.detected_images.empty() ? 0 : 1;
    event_count += static_cast<int>(out.image_events.size());
    located_frames += (out.step.has_value() && !out.step->located_images.empty()) ? 1 : 0;
    roi_frames += glue.NextDrawRois().empty() ? 0 : 1;
    const std::string line = ImagesLine(frame, out, glue) + "\n";
    std::fputs(line.c_str(), images);
  }
  const double wall_s = std::chrono::duration<double>(std::chrono::steady_clock::now() - wall_start).count();
  const int rows = writer.rows();
  const int processed = writer.processed();
  if (!writer.Close(&error)) Fail(error);
  std::fclose(images);

  std::string meta = "{\n \"schema\": \"slam-refact-or4-run-v1\",\n";
  meta += " \"bundle\": \"" + bundle.dir + "\",\n \"contractVersion\": " + std::to_string(kContractVersion) + ",\n";
  meta += " \"targetsDir\": " + std::string(targets_dir.empty() ? "null" : "\"" + targets_dir + "\"") + ",\n";
  meta += " \"targetsLoaded\": " + std::to_string(glue.image_target_names().size()) + ",\n";
  meta += " \"targetLoadMs\": " + Num(target_load_ms) + ",\n";
  meta += " \"idGeneratorSeed\": " + std::string(seeded ? std::to_string(used_seed) : "null") + ",\n";
  meta += " \"mutation\": " + std::string(mut.text.empty() ? "null" : "\"" + mut.text + "\"") + ",\n";
  meta += " \"engineRngSeed\": " + std::string(rng_seed_text.empty() ? "null" : rng_seed_text) + ",\n";
  meta += " \"forceOrbGorb\": " + std::string(force_orb_gorb ? "true" : "false") + ",\n";
  meta += " \"idSeedOverridden\": " + std::string(have_seed ? "true" : "false") + ",\n";
  meta += " \"rows\": " + std::to_string(rows) + ",\n \"processedFrames\": " + std::to_string(processed) +
          ",\n \"droppedFrames\": " + std::to_string(dropped) + ",\n \"injectedPyramidFrames\": " +
          std::to_string(injected) + ",\n \"paddedAtlases\": " + std::to_string(padded) + ",\n \"framesWithDetectedImages\": " + std::to_string(detected_frames) +
          ",\n \"imageEvents\": " + std::to_string(event_count) + ",\n \"framesWithLocatedImages\": " +
          std::to_string(located_frames) + ",\n \"framesWithNextRois\": " + std::to_string(roi_frames) +
          ",\n \"wallSeconds\": " + Num(wall_s) + "\n}\n";
  std::FILE* m = std::fopen((out_dir + "/meta.json").c_str(), "wx");
  if (m == nullptr) Fail("cannot create meta.json");
  std::fputs(meta.c_str(), m);
  std::fclose(m);
  std::printf("rows=%d processed=%d dropped=%d injected=%d targets=%zu detectedFrames=%d events=%d located=%d roiFrames=%d wall=%.1fs out=%s\n",
              rows, processed, dropped, injected, glue.image_target_names().size(), detected_frames, event_count,
              located_frames, roi_frames, wall_s, out_dir.c_str());
  return 0;
}

}  // namespace
}  // namespace slam_refact::harness

int main(int argc, char** argv) { return slam_refact::harness::Run(argc, argv); }

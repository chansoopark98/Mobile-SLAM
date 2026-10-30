// rm3b_recenter_replay (G6 / RM-3b criterion 5, 2026-10-25): the exact-input replay of tools/harness/replay/replay_main.cc plus a scripted
// sequence of XrController calls, so that the engine can be compared with the original SDK on the same input and the same call order.
//
//   rm3b_recenter_replay --bundle <bundle dir> --out <new run dir> --actions <actions.tsv>
//
// actions.tsv (written by pipeline.py from the original run's recenter-probe.json): one call per line, tab separated
//   <input frame index> <fn> <ox> <oy> <oz> <update_recenter_point>      fn = recenter | configure | update
// configure is configure({}) (no options set); update is updateCameraProjectionMatrix({origin: {ox, oy, oz}, updateRecenterPoint}) (no cam, no facing).
// The calls of a frame run, in file order, right after ProcessFrame returned for that frame (the original runs them inside the onUpdate of
// the callback that carries that frame, i.e. between two processStagedFrame calls).
// Output: rows.jsonl (one line per input frame: i, processed, repeated, status, reason, p, q_wxyz as %.9g float32 values), calls.jsonl, meta.json.
// Development harness (desktop replay of a recorded input); not product code. Built out of tree against an existing build's libraries.

#include <sys/stat.h>

#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

#include "bundle_reader.h"
#include "slam_refact/glue_runner.h"
#include "slam_refact/replay_diagnostics.h"
#include "slam_refact/version.h"

namespace slam_refact::harness {
namespace {

[[noreturn]] void Fail(const std::string& msg) {
  std::fprintf(stderr, "rm3b_recenter_replay: %s\n", msg.c_str());
  std::exit(2);
}

struct Call {
  std::string fn;
  Vec3f origin;
  bool update_recenter_point = true;
};

std::string Esc(const std::string& s) {
  std::string out;
  for (char c : s) {
    if (c == '"' || c == '\\') out.push_back('\\');
    out.push_back(c);
  }
  return out;
}

void PushSensor(const SensorRow& row, GlueRunner* glue) {
  glue->PushDeviceMotion(row.motion);
  glue->PushDeviceOrientation(row.orientation);
}

const char* StatusName(TrackingStatus s) {
  switch (s) {
    case TrackingStatus::kNotAvailable: return "NOT_AVAILABLE";
    case TrackingStatus::kLimited: return "LIMITED";
    case TrackingStatus::kNormal: return "NORMAL";
    default: return "UNSPECIFIED";
  }
}

const char* ReasonName(TrackingReason r) {
  switch (r) {
    case TrackingReason::kInitializing: return "INITIALIZING";
    case TrackingReason::kRelocalizing: return "RELOCALIZING";
    case TrackingReason::kTooMuchMotion: return "TOO_MUCH_MOTION";
    case TrackingReason::kNotEnoughTexture: return "NOT_ENOUGH_TEXTURE";
    default: return "UNSPECIFIED";
  }
}

int Run(int argc, char** argv) {
  std::string bundle_dir, out_dir, actions_path;
  for (int k = 1; k + 1 < argc; k += 2) {
    if (std::strcmp(argv[k], "--bundle") == 0) bundle_dir = argv[k + 1];
    else if (std::strcmp(argv[k], "--out") == 0) out_dir = argv[k + 1];
    else if (std::strcmp(argv[k], "--actions") == 0) actions_path = argv[k + 1];
    else Fail(std::string("unknown argument ") + argv[k]);
  }
  if (bundle_dir.empty() || out_dir.empty() || actions_path.empty() || argc % 2 == 0) {
    Fail("usage: rm3b_recenter_replay --bundle <dir> --out <new dir> --actions <actions.tsv>");
  }
  std::map<int, std::vector<Call>> actions;
  {
    std::ifstream in(actions_path);
    if (!in) Fail("cannot open " + actions_path);
    std::string line;
    while (std::getline(in, line)) {
      if (line.empty() || line[0] == '#') continue;
      std::istringstream ls(line);
      int frame = -1;
      Call c;
      std::string rcp;
      std::string ox, oy, oz;
      if (!(ls >> frame >> c.fn >> ox >> oy >> oz >> rcp)) Fail("bad actions line: " + line);
      if (c.fn != "recenter" && c.fn != "configure" && c.fn != "update") Fail("unknown fn in: " + line);
      if (c.fn == "update") {
        c.origin = Vec3f{std::strtof(ox.c_str(), nullptr), std::strtof(oy.c_str(), nullptr),
                         std::strtof(oz.c_str(), nullptr)};
        c.update_recenter_point = rcp != "0";
      }
      actions[frame].push_back(c);
    }
  }
  ReplayBundle bundle;
  std::string error;
  if (!LoadBundle(bundle_dir, &bundle, &error)) Fail(error);
  if (mkdir(out_dir.c_str(), 0775) != 0) Fail("cannot create " + out_dir + " (a new run directory is required)");
  const auto seed_it = bundle.config.find("id_generator_seed");
  if (seed_it != bundle.config.end()) {
    slam_refact::diagnostics::SetReplayIdGeneratorSeed(
        static_cast<uint32_t>(std::strtoull(seed_it->second.c_str(), nullptr, 10)));
  }
  EngineConfig engine_config = bundle.MakeEngineConfig();
  GlueConfig glue_config = bundle.MakeGlueConfig();
  GlueRunner glue(engine_config, glue_config);
  std::FILE* rows = std::fopen((out_dir + "/rows.jsonl").c_str(), "wx");
  std::FILE* calls = std::fopen((out_dir + "/calls.jsonl").c_str(), "wx");
  if (rows == nullptr || calls == nullptr) Fail("cannot create output files");

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
  int processed = 0, executed = 0;
  for (size_t f = 1; f < bundle.frames.size(); ++f) {
    const FrameRow& frame = bundle.frames[f];
    while (next_sensor < bundle.sensors.size() &&
           bundle.sensors[next_sensor].motion.receive_time_ns <= frame.time_ns) {
      PushSensor(bundle.sensors[next_sensor++], &glue);
    }
    if (!frame.process) {
      std::fprintf(rows, "{\"i\":%d,\"processed\":false}\n", frame.index);
      continue;
    }
    if (!LoadImages(frame, &images, &error)) Fail(error);
    const FrameInput in = MakeFrameInput(frame, images);
    const PublishedFrame out = glue.ProcessFrame(in.time_ns, in.video_time_ns, in.gray, in.pyramid);
    ++processed;
    std::fprintf(rows,
                 "{\"i\":%d,\"processed\":true,\"repeated\":%s,\"status\":\"%s\",\"reason\":\"%s\","
                 "\"p\":[%.9g,%.9g,%.9g],\"q_wxyz\":[%.9g,%.9g,%.9g,%.9g]}\n",
                 frame.index, out.repeated ? "true" : "false", StatusName(out.status),
                 ReasonName(out.reason), out.position.x, out.position.y, out.position.z,
                 out.rotation.w, out.rotation.x, out.rotation.y, out.rotation.z);
    const auto it = actions.find(frame.index);
    if (it == actions.end()) continue;
    for (const Call& c : it->second) {
      std::string err;
      if (c.fn == "recenter") {
        glue.Recenter();
      } else if (c.fn == "configure") {
        const XrError e = glue.Configure(XrConfigureOptions{});
        if (!e.ok()) err = e.message;
      } else {
        CameraProjectionUpdate u;
        u.origin = c.origin;
        u.update_recenter_point = c.update_recenter_point;
        glue.UpdateCameraProjectionMatrix(u);
      }
      ++executed;
      std::fprintf(calls, "{\"frame\":%d,\"fn\":\"%s\",\"ok\":%s,\"error\":\"%s\"}\n", frame.index,
                   c.fn.c_str(), err.empty() ? "true" : "false", Esc(err).c_str());
    }
  }
  std::fclose(rows);
  std::fclose(calls);
  std::FILE* m = std::fopen((out_dir + "/meta.json").c_str(), "wx");
  if (m == nullptr) Fail("cannot create meta.json");
  std::fprintf(m,
               "{\"schema\":\"rm3b-recenter-engine-run-v1\",\"bundle\":\"%s\",\"actions\":\"%s\","
               "\"contractVersion\":%d,\"processedFrames\":%d,\"executedCalls\":%d,\"sensorRows\":%zu,"
               "\"sensorRowsPushed\":%zu}\n",
               Esc(bundle.dir).c_str(), Esc(actions_path).c_str(), kContractVersion, processed, executed,
               bundle.sensors.size(), next_sensor);
  std::fclose(m);
  std::printf("processed=%d executed=%d out=%s\n", processed, executed, out_dir.c_str());
  return 0;
}

}  // namespace
}  // namespace slam_refact::harness

int main(int argc, char** argv) { return slam_refact::harness::Run(argc, argv); }

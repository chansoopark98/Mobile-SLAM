// slam_refact_mse_dump: replays a bundle exactly like slam_refact_replay and records, for every core
// Track call, the inputs of MetricScaleEstimator::PushFrame (944) that cross the TrackerCore
// boundary: post-647 events, time, tamExtrinsic (the pose 784 passes to 944), MapTracker state,
// MapBuilder map_reset, plus the engine's latch result. Used by mse_original_probe.mjs to run the
// ORIGINAL 944 on the engine's own inputs (differential MS-RT root-cause check, WP5b).
//
//   slam_refact_mse_dump --bundle <bundle dir> --out <new jsonl file> [--max-frames N]

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <string>

#include "bundle_reader.h"
#include "slam_refact/glue_runner.h"
#include "trace_writer.h"

namespace slam_refact::harness {
namespace {

[[noreturn]] void Fail(const std::string& msg) {
  std::fprintf(stderr, "slam_refact_mse_dump: %s\n", msg.c_str());
  std::exit(2);
}

// Decorator that records the PushFrame inputs of each Track call of the wrapped Engine.
class RecordingCore final : public TrackerCore {
 public:
  RecordingCore(const EngineConfig& cfg, int core_id, std::FILE* out)
      : inner_(DefaultEngineFactory(cfg)), core_id_(core_id), out_(out) {}

  TrackerStep Track(const FrameInput& in) override {
    TrackerStep s = inner_->Track(in);
    std::string line = "{\"core\":" + std::to_string(core_id_) + ",\"call\":" + std::to_string(call_++) +
                       ",\"timeNs\":" + std::to_string(in.time_ns) +
                       ",\"state\":" + std::to_string(s.map_tracker_state) +
                       ",\"reset\":" + (s.map_reset ? "1" : "0") +
                       ",\"latched\":" + (s.scale_latched ? "1" : "0") +
                       ",\"latchedScale\":" + JsNumber(s.latched_scale) + ",\"tam\":[";
    for (int k = 0; k < 16; ++k) {
      line += (k ? "," : "") + JsNumber(s.tam_extrinsic.m[k]);
    }
    line += "],\"events\":[";
    for (size_t k = 0; k < in.imu_events.size(); ++k) {
      const ImuEvent& e = in.imu_events[k];
      line += std::string(k ? "," : "") + "[" + std::to_string(static_cast<int>(e.kind)) + "," +
              std::to_string(e.event_time_ns) + "," + std::to_string(e.interval_ns) + "," +
              JsNumber(e.x) + "," + JsNumber(e.y) + "," + JsNumber(e.z) + "]";
    }
    line += "]}\n";
    std::fputs(line.c_str(), out_);
    return s;
  }

 private:
  std::unique_ptr<TrackerCore> inner_;
  int core_id_;
  int call_ = 0;
  std::FILE* out_;
};

int Run(int argc, char** argv) {
  std::string bundle_dir, out_path;
  int max_frames = -1;
  for (int k = 1; k + 1 < argc; k += 2) {
    if (std::strcmp(argv[k], "--bundle") == 0) {
      bundle_dir = argv[k + 1];
    } else if (std::strcmp(argv[k], "--out") == 0) {
      out_path = argv[k + 1];
    } else if (std::strcmp(argv[k], "--max-frames") == 0) {
      max_frames = std::atoi(argv[k + 1]);
    } else {
      Fail(std::string("unknown argument ") + argv[k]);
    }
  }
  if (bundle_dir.empty() || out_path.empty()) {
    Fail("usage: slam_refact_mse_dump --bundle <dir> --out <new jsonl> [--max-frames N]");
  }
  ReplayBundle bundle;
  std::string error;
  if (!LoadBundle(bundle_dir, &bundle, &error)) Fail(error);
  std::FILE* out = std::fopen(out_path.c_str(), "wx");
  if (out == nullptr) Fail("cannot create " + out_path + " (existing files are never overwritten)");

  int cores = 0;
  GlueRunner glue(bundle.MakeEngineConfig(), bundle.MakeGlueConfig(),
                  [&cores, out](const EngineConfig& cfg) -> std::unique_ptr<TrackerCore> {
                    return std::make_unique<RecordingCore>(cfg, ++cores, out);
                  });
  // Same call order as replay_main.cc (bootstrap, then sensors <= frame time, drop mask).
  size_t next = 0;
  while (next < bundle.sensors.size() && bundle.sensors[next].bootstrap) {
    glue.PushDeviceMotion(bundle.sensors[next].motion);
    glue.PushDeviceOrientation(bundle.sensors[next].orientation);
    ++next;
  }
  LoadedImages images;
  if (!LoadImages(bundle.frames.front(), &images, &error)) Fail(error);
  {
    const FrameInput in = MakeFrameInput(bundle.frames.front(), images);
    glue.ProcessFrame(in.time_ns, in.video_time_ns, in.gray, in.pyramid);
  }
  const size_t end = max_frames < 0 ? bundle.frames.size()
                                    : std::min(bundle.frames.size(), static_cast<size_t>(max_frames) + 1);
  for (size_t f = 1; f < end; ++f) {
    const FrameRow& frame = bundle.frames[f];
    while (next < bundle.sensors.size() && bundle.sensors[next].motion.receive_time_ns <= frame.time_ns) {
      glue.PushDeviceMotion(bundle.sensors[next].motion);
      glue.PushDeviceOrientation(bundle.sensors[next].orientation);
      ++next;
    }
    if (!frame.process) continue;
    if (!LoadImages(frame, &images, &error)) Fail(error);
    const FrameInput in = MakeFrameInput(frame, images);
    glue.ProcessFrame(in.time_ns, in.video_time_ns, in.gray, in.pyramid);
  }
  std::fclose(out);
  std::printf("cores=%d out=%s\n", cores, out_path.c_str());
  return 0;
}

}  // namespace
}  // namespace slam_refact::harness

int main(int argc, char** argv) { return slam_refact::harness::Run(argc, argv); }

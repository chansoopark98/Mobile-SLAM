// Replay of a recorded original run through SkyPipeline, shared by sr_sky_replay and the gate tests
// (docs/spec/09-sky.md §6). Header-only, bench/test use only.
#ifndef SLAM_REFACT_SKY_BENCH_SKY_REPLAY_LIB_H_
#define SLAM_REFACT_SKY_BENCH_SKY_REPLAY_LIB_H_

#include <cmath>
#include <map>
#include <string>
#include <vector>

#include "sky/bench/sky_record.h"
#include "sky/sky_pipeline.h"

namespace slam_refact::sky {

enum class ReplayMode { kE2e, kInjectWorker };

struct ReplayOptions {
  ReplayMode mode = ReplayMode::kInjectWorker;
  bool seamless = true;
  // Optional override of the recorded configuration (mutation experiments, never the gate itself).
  bool override_config = false;
  bool invert = false;
  double edge_smoothness = 0.0;
  double orientation_beta_offset_deg = 0.0;  // added to every recorded beta
};

struct ReplayFrame {
  bool valid = false;
  LayersFrameOutput out;
  bool requested = false;
};

struct ReplayResult {
  std::vector<ReplayFrame> frames;                    // per processed frame k
  std::vector<SkyWorkerRequest> requests;             // our request n
  std::vector<SkyWorkerAnswer> answers;               // our answer n
  std::vector<SkyEvent> events;
  int rotation_mismatch = 0;
};

// `model` may be null in kInjectWorker mode.
inline bool RunReplay(const SkyRecord& rec, const ReplayOptions& opt, SkyModel* model_owner_unused,
                      const std::vector<uint8_t>* model_bytes, ReplayResult* res,
                      std::string* error) {
  (void)model_owner_unused;
  SkyPipeline pipe;
  if (opt.mode == ReplayMode::kE2e) {
    if (model_bytes == nullptr || !pipe.LoadModel(*model_bytes, true, error)) {
      return false;
    }
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
  args.sky.invert = opt.override_config ? opt.invert : rec.invert;
  args.sky.has_smoothness = true;
  args.sky.smoothness = opt.override_config ? opt.edge_smoothness : rec.edge_smoothness;
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
  pipe.controller().set_cubemap_seamless(opt.seamless);
  pipe.controller().Configure(args);
  pipe.controller().OnStart(env);
  // The module is attached before the first frame (the recorded layerloading event is asynchronous).
  pipe.controller().OnAttach(env);
  size_t next_invoke = 0;
  for (const auto& fr : rec.frames) {
    for (const auto& e : rec.events) {
      if (e.staged == fr.k && e.name == "layerscanning") {
        pipe.WorkerStarted();
      }
    }
    Rgba8Image cam;
    if (!rec.ReadImage(fr.file, fr.w, fr.h, &cam)) {
      *error = "cannot read " + fr.file;
      return false;
    }
    ReplayFrame rf;
    SkyWorkerFn worker;
    if (opt.mode == ReplayMode::kInjectWorker) {
      worker = [&](const SkyWorkerRequest& req, SkyWorkerAnswer* ans) {
        rf.requested = true;
        res->requests.push_back(req);
        if (next_invoke >= rec.invokes.size()) {
          res->answers.emplace_back();
          return false;
        }
        const auto& iv = rec.invokes[next_invoke++];
        const double d = std::fabs(req.rotation.x - iv.rot.x) + std::fabs(req.rotation.y - iv.rot.y) +
                         std::fabs(req.rotation.z - iv.rot.z) + std::fabs(req.rotation.w - iv.rot.w);
        if (d > 1e-9) {
          ++res->rotation_mismatch;
        }
        if (!rec.ReadFloats(iv.data, &ans->data)) {
          res->answers.emplace_back();
          return false;
        }
        ans->percentage = iv.percentage;
        res->answers.push_back(*ans);
        return true;
      };
    } else {
      worker = [&](const SkyWorkerRequest& req, SkyWorkerAnswer* ans) {
        rf.requested = true;
        res->requests.push_back(req);
        SkyInference inf;
        if (!pipe.model().Invoke(req.img.px.data(), &inf)) {
          res->answers.emplace_back();
          return false;
        }
        ans->data = std::move(inf.data);
        ans->percentage = inf.percentage;
        res->answers.push_back(*ans);
        return true;
      };
    }
    DeviceOrientation o = fr.o;
    o.beta += opt.orientation_beta_offset_deg;
    rf.out = pipe.ProcessFrame(cam, false, o, worker);
    rf.valid = rf.out.valid;
    res->frames.push_back(std::move(rf));
    for (auto& e : pipe.controller().TakeEvents()) {
      res->events.push_back(std::move(e));
    }
  }
  return true;
}

}  // namespace slam_refact::sky

#endif  // SLAM_REFACT_SKY_BENCH_SKY_REPLAY_LIB_H_

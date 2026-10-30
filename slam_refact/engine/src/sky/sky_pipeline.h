// SkyPipeline: LayersController + semantics worker wired synchronously (US-020).
//
// The original runs the model in a Worker: the request posted in onProcessCpu(k) is answered
// asynchronously, usually before the next camera callback (recorded: every answer arrived before the
// next processed frame at 250 ms frame spacing). SkyPipeline answers it at once, inside the same
// ProcessFrame() call, which is that ordering with zero latency. A caller that wants the original's
// exact worker output (open-loop comparison) passes its own WorkerFn.
#ifndef SLAM_REFACT_SKY_SKY_PIPELINE_H_
#define SLAM_REFACT_SKY_SKY_PIPELINE_H_

#include <functional>
#include <string>
#include <vector>

#include "sky/sky_layers_controller.h"
#include "sky/sky_model.h"

namespace slam_refact::sky {

// The worker's answer to one request.
struct SkyWorkerAnswer {
  std::vector<float> data;  // kModelInputHeight x kModelInputWidth scores
  double percentage = 0.0;
};
using SkyWorkerFn = std::function<bool(const SkyWorkerRequest&, SkyWorkerAnswer*)>;

class SkyPipeline {
 public:
  SkyPipeline() = default;

  bool LoadModel(const std::vector<uint8_t>& model, bool check_sha, std::string* error) {
    return model_.Load(model, check_sha, error);
  }
  LayersController& controller() { return controller_; }
  const LayersController& controller() const { return controller_; }
  SkyModel& model() { return model_; }

  // Session start in the order the deployed module sees it (onStart, then onAttach), and the
  // worker's `started` message once the model is loaded.
  void Start(const SkyEnvironment& env) {
    controller_.OnStart(env);
    controller_.OnAttach(env);
  }
  void WorkerStarted() { controller_.WorkerStarted(); }

  // One camera callback. `worker` == nullptr runs the built-in SkyModel.
  LayersFrameOutput ProcessFrame(const Rgba8Image& camera, bool repeat_frame,
                                 const DeviceOrientation& orientation,
                                 const SkyWorkerFn& worker = nullptr) {
    const auto gpu = controller_.OnProcessGpu(camera, repeat_frame, orientation);
    SkyCpuResult cpu = controller_.OnProcessCpu(gpu, repeat_frame);
    if (cpu.request.has_value()) {
      SkyWorkerAnswer answer;
      bool ok = false;
      if (worker) {
        ok = worker(*cpu.request, &answer);
      } else if (model_.loaded()) {
        SkyInference inf;
        ok = model_.Invoke(cpu.request->img.px.data(), &inf);
        answer.data = std::move(inf.data);
        answer.percentage = inf.percentage;
      }
      if (ok) {
        controller_.OnWorkerResult(answer.data, answer.percentage, cpu.request->rotation,
                                   cpu.request->position);
      }
    }
    return std::move(cpu.output);
  }

 private:
  LayersController controller_;
  SkyModel model_;
};

}  // namespace slam_refact::sky

#endif  // SLAM_REFACT_SKY_SKY_PIPELINE_H_

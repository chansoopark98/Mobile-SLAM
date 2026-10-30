// LayersController core without GL, Cap'n Proto, a Worker or emscripten (US-020;
// docs/spec/09-sky.md §4).
//
// Port of the sky path of PUB:reality/app/xr/js/src/layers-controller.ts (the pipeline module, the
// configure() reducer, the events layerloading / layerscanning / layerfound, the 1-frame GPU readback
// delay, the isInvokeReady_ gate, haveSeenSky) together with PUB:reality/app/xr/js/layers-controller.cc
// (intrinsics, the two-entry pipeline queue, the yaw offset and the response camera) and
// PUB:reality/app/xr/js/src/semantics/options.ts. What the original does on the GPU is the CPU
// renderer (sky_cubemap.h) and the camera resize (sky_preprocess.h); the worker is the caller's job:
// OnProcessCpu() returns the request the original posts to the semantics worker and
// OnWorkerResult() is the worker's `invoke` answer (SkyPipeline wires both to SkyModel).
#ifndef SLAM_REFACT_SKY_SKY_LAYERS_CONTROLLER_H_
#define SLAM_REFACT_SKY_SKY_LAYERS_CONTROLLER_H_

#include <deque>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "c8/geometry/pixel-pinhole-camera-model.h"
#include "c8/quaternion.h"
#include "sky/sky_cubemap.h"
#include "sky/sky_types.h"

namespace slam_refact::sky {

// Intrinsics::getCameraIntrinsics default branch (intrinsics.cc:358-360, 470-472): the device table
// has no entry for desktop Chrome or a reduced Android UA ("K"): 480x640, centre (239.5, 319.5),
// focal 506.4104. A device-specific focal can be given in Environment instead.
c8_PixelPinholeCameraModel DefaultDeviceIntrinsics();

// Intrinsics::toClipSpace (intrinsics.cc:81-106), column-major: the projection matrix of the
// response camera.
std::array<float, 16> ToClipSpace(const c8_PixelPinholeCameraModel& k, float near_clip, float far_clip);

// What updateEnvironment() of layers-controller.ts receives (video = camera texture, canvas = view).
struct SkyEnvironment {
  int video_width = 0;
  int video_height = 0;
  int canvas_width = 0;
  int canvas_height = 0;
  int orientation = 0;  // 0 portrait, 90 / -90 landscape
  bool has_device_estimate = true;
  c8_PixelPinholeCameraModel device_intrinsics = DefaultDeviceIntrinsics();
};

// invokeData of onProcessGpu: the resized frame of the PREVIOUS processed frame and the device
// rotation of that frame.
struct SkyInvokeData {
  Rgba8Image pixels;  // dest_.width x dest_.height
  int rows = 0;
  int cols = 0;
  QuaternionXyzw rotation;
};

// What onProcessCpu posts to the semantics worker.
struct SkyWorkerRequest {
  Rgba8Image img;  // kModelInputWidth x kModelInputHeight
  QuaternionXyzw rotation;
  Position3 position;  // IDENTITY_POS
};

struct SkyCpuResult {
  LayersFrameOutput output;                 // valid = false when the original returns null
  std::optional<SkyWorkerRequest> request;  // set when the worker was ready and got a frame
};

class LayersController {
 public:
  LayersController();
  ~LayersController();

  // LayersController.configure(args) (optionsReducer + the options push that follows it).
  void Configure(const ConfigureArgs& args);
  // recenter(): clears the yaw offset (the next staged frame sets it again).
  void Recenter();

  // Pipeline module hooks. OnAttach dispatches `layerloading`; OnStart resets the per-session GPU
  // state; WorkerStarted() is the worker's `started` message (isInvokeReady_ = true, dispatches
  // `layerscanning`).
  void OnAttach(const SkyEnvironment& env);
  void OnStart(const SkyEnvironment& env);
  void OnCanvasSizeChange(const SkyEnvironment& env);
  void OnVideoSizeChange(const SkyEnvironment& env);
  void OnDeviceOrientationChange(const SkyEnvironment& env);
  void WorkerStarted();

  // onProcessGpu: `camera` is the frame's camera texture, `orientation` the latest
  // `deviceorientation` event. Returns the invokeData for onProcessCpu (the original returns null
  // for repeat frames and for the first frame).
  std::optional<SkyInvokeData> OnProcessGpu(const Rgba8Image& camera, bool repeat_frame,
                                            const DeviceOrientation& orientation);
  // onProcessCpu. `gpu` is onProcessGpu's return value for the same frame.
  SkyCpuResult OnProcessCpu(const std::optional<SkyInvokeData>& gpu, bool repeat_frame);
  // The worker's `invoke` answer.
  void OnWorkerResult(const std::vector<float>& data, double percentage,
                      const QuaternionXyzw& rotation, const Position3& position);

  std::vector<SkyEvent> TakeEvents();
  std::vector<std::string> GetLayerNames() const;
  bool have_seen_sky() const { return have_seen_sky_; }
  bool invoke_ready() const { return is_invoke_ready_; }
  const SkyCubemapRenderer* renderer() const { return renderer_.get(); }
  // Cubemap edge filtering of the renderer created at the next OnAttach/environment update.
  void set_cubemap_seamless(bool seamless) { cubemap_seamless_ = seamless; }

 private:
  struct StagedFrame {
    c8::Quaternion device_pose_in_xr;
    bool has_response = false;
    std::array<float, 16> intrinsics{};
    Position3 position;
    QuaternionXyzw rotation;
  };
  // createSemanticsOptionsMsg + c8EmAsm_updateSemanticsOptions.
  void PushSemanticsOptions();
  void UpdateEnvironment(const SkyEnvironment& env);
  void UpdateResizerDimensions(int video_width, int video_height);
  void UpdateIntrinsics();
  void UpdateCubemap(const std::vector<float>& data, const QuaternionXyzw& rotation);
  void ProcessStagedFrame();
  void Dispatch(const std::string& name, const std::string& detail_json);

  // --- layers-controller.ts state ---
  SkyEnvironment env_in_;                 // last environment given
  bool attached_ = false;
  int env_video_width_ = 0, env_video_height_ = 0, env_canvas_width_ = 0, env_canvas_height_ = 0;
  int env_orientation_ = 0;
  int model_input_width_ = 0, model_input_height_ = 0;
  double video_resize_width_ = 0.0, video_resize_height_ = 0.0;
  int src_video_width_ = 0, src_video_height_ = 0;
  int dest_width_ = 0, dest_height_ = 0;
  bool has_resizer_ = false;
  std::optional<Rgba8Image> camera_resized_tex_;
  std::optional<QuaternionXyzw> camera_resized_tex_rotation_;
  Rgba8Image semantics_tex_;  // semanticsTex_
  int semantics_tex_width_ = 0, semantics_tex_height_ = 0;
  bool has_semantics_tex_ = false;
  double percentage_ = 0.0;
  bool is_invoke_ready_ = false;
  bool have_seen_sky_ = false;
  bool debug_input_mask_ = false;
  Rgba8Image debug_semantics_mask_;
  DeviceOrientation last_device_rotation_;
  // options_
  bool has_near_ = false, has_far_ = false;
  double near_clip_ = 0.0, far_clip_ = 0.0;
  bool has_coordinates_ = false;
  Coordinates coordinates_;
  bool has_sky_layer_ = false;
  LayerOptions sky_options_;
  std::vector<SkyEvent> events_;

  // --- layers-controller.cc state ---
  std::unique_ptr<SkyCubemapRenderer> renderer_;
  bool cubemap_seamless_ = true;
  c8_PixelPinholeCameraModel intrinsics_;
  std::array<float, 16> projection_matrix_{};
  bool has_cam_env_ = false;
  int cam_env_camera_w_ = 0, cam_env_camera_h_ = 0, cam_env_display_w_ = 0, cam_env_display_h_ = 0;
  int cam_env_orientation_ = 0;
  bool has_cam_env_estimate_ = false;
  c8_PixelPinholeCameraModel cam_env_device_k_;
  // data()->opts
  bool has_sem_opts_ = false;
  int sem_out_w_ = 0, sem_out_h_ = 0;
  float sem_near_ = 0.0f, sem_far_ = 0.0f;
  Coordinates sem_coordinates_;
  float sem_origin_scale_ = 1.0f;
  Position3 sem_origin_pos_{0, 2, 0};
  QuaternionXyzw sem_origin_rot_{0, 0, 0, 1};
  std::deque<StagedFrame> pipeline_;
  StagedFrame active_;
  bool has_active_ = false;
  bool has_set_yaw_offset_ = false;
  c8::Quaternion yaw_offset_;
  bool response_valid_ = false;
};

}  // namespace slam_refact::sky

#endif  // SLAM_REFACT_SKY_SKY_LAYERS_CONTROLLER_H_

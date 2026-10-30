// LayersController core. See sky_layers_controller.h and docs/spec/09-sky.md §4.
//
// Source map (PUB = public reference runtime commit 4edfa484):
//   PUB:reality/app/xr/js/src/layers-controller.ts  updateEnvironment :477-592, onProcessGpu :980-1050,
//     onProcessCpu :1052-1214, worker `invoke` handler :727-785, updateCubemap :395-413, configure
//     reducer :1304-1396
//   PUB:reality/app/xr/js/layers-controller.cc       updateIntrinsics :63-96, updateSemanticsOptions
//     :108-124, processStagedLayersControllerFrame :186-243
//   PUB:reality/app/xr/js/src/semantics/options.ts   createSemanticsOptionsMsg
//   PUB:reality/engine/geometry/orientation.cc        rewriteCoordinateSystemPostprocess(SemanticsResponse)
#include "sky/sky_layers_controller.h"

#include <algorithm>
#include <charconv>
#include <cmath>
#include <cstdio>

#include "c8/geometry/device-pose.h"
#include "c8/geometry/egomotion.h"
#include "c8/geometry/homography.h"
#include "c8/geometry/intrinsics-clip.h"
#include "sky/sky_preprocess.h"

namespace slam_refact::sky {
namespace {

// JS Number -> shortest round-trip string (JSON.stringify of a finite number).
std::string JsNumber(double v) {
  char buf[64];
  const auto r = std::to_chars(buf, buf + sizeof(buf), v);
  std::string s(buf, r.ptr);
  // to_chars may print exponents as 1e-07; JS prints 1e-7.
  const auto e = s.find("e-0");
  if (e != std::string::npos) {
    s.erase(e + 2, 1);
  }
  return s;
}

// Uint8Array store of a double (ToUint8: truncate toward zero, modulo 256, NaN -> 0).
uint8_t JsToUint8(double v) {
  if (std::isnan(v) || std::isinf(v)) {
    return 0;
  }
  double t = std::trunc(v);
  t = std::fmod(t, 256.0);
  if (t < 0) {
    t += 256.0;
  }
  return static_cast<uint8_t>(t);
}

int JsToInt32(double v) {
  if (std::isnan(v) || std::isinf(v)) {
    return 0;
  }
  return static_cast<int>(static_cast<int32_t>(static_cast<int64_t>(std::trunc(v))));
}

c8::Quaternion ToC8Quaternion(const QuaternionXyzw& q) {
  return c8::Quaternion(static_cast<float>(q.w), static_cast<float>(q.x), static_cast<float>(q.y),
                        static_cast<float>(q.z));
}

}  // namespace

c8_PixelPinholeCameraModel DefaultDeviceIntrinsics() {
  return c8_PixelPinholeCameraModel{480, 640, 239.5f, 319.5f, 506.4104f, 506.4104f};
}

std::array<float, 16> ToClipSpace(const c8_PixelPinholeCameraModel& k, float kn, float kf) {
  const float c00 = 2 * k.focalLengthHorizontal / k.pixelsWidth;
  const float c11 = 2 * k.focalLengthVertical / k.pixelsHeight;
  const float c02 = 1.0f - (2.0f * k.centerPointX + 1.0f) / k.pixelsWidth;
  const float c12 = -1.0f + (2.0f * k.centerPointY + 1.0f) / k.pixelsHeight;
  const float c22 = (kf + kn) / (kn - kf);
  const float c23 = 2 * kf * kn / (kn - kf);
  const float c32 = -1;
  std::array<float, 16> m{};
  m[0] = c00;
  m[5] = c11;
  m[8] = c02;
  m[9] = c12;
  m[10] = c22;
  m[11] = c32;
  m[14] = c23;
  return m;
}

LayersController::LayersController() = default;
LayersController::~LayersController() = default;

void LayersController::Dispatch(const std::string& name, const std::string& detail_json) {
  events_.push_back({name, detail_json});
}

std::vector<SkyEvent> LayersController::TakeEvents() {
  std::vector<SkyEvent> out;
  out.swap(events_);
  return out;
}

std::vector<std::string> LayersController::GetLayerNames() const {
  return has_sky_layer_ ? std::vector<std::string>{"sky"} : std::vector<std::string>{};
}

void LayersController::Configure(const ConfigureArgs& args) {
  // optionsReducer.
  if (args.has_near_clip) {
    has_near_ = true;
    near_clip_ = args.near_clip;
  }
  if (args.has_far_clip) {
    has_far_ = true;
    far_clip_ = args.far_clip;
  }
  if (args.coordinates_null) {
    has_coordinates_ = false;
    coordinates_ = Coordinates{};
  } else if (args.has_coordinates) {
    if (!has_coordinates_) {
      has_coordinates_ = true;
      coordinates_ = Coordinates{};
    }
    const Coordinates& c = args.coordinates;
    if (c.has_origin_position) {
      coordinates_.has_origin_position = true;
      coordinates_.origin_position = c.origin_position;
    }
    if (c.has_origin_rotation) {
      coordinates_.has_origin_rotation = true;
      coordinates_.origin_rotation = c.origin_rotation;
    }
    coordinates_.left_handed = c.left_handed;
    coordinates_.mirrored_display = c.mirrored_display;
  }
  if (args.layers_null) {
    has_sky_layer_ = false;
    sky_options_ = LayerOptions{};
  } else if (args.sky.present) {
    const LayerOptionsPatch& p = args.sky;
    if (p.remove) {
      has_sky_layer_ = false;
    } else {
      if (!has_sky_layer_) {
        has_sky_layer_ = true;
        sky_options_ = LayerOptions{};
      }
      if (p.has_invert) {
        sky_options_.invert_layer_mask = p.invert_is_null ? kDefaultInvertLayerMask : p.invert;
      }
      if (p.has_smoothness) {
        sky_options_.edge_smoothness = p.smoothness_is_null ? kDefaultEdgeSmoothness : p.smoothness;
      }
    }
  }
  if (args.debug_null) {
    debug_input_mask_ = false;
  } else if (args.has_debug) {
    debug_input_mask_ = args.debug_input_mask;
  }
  // configure(): the options push once the engine module exists and the video size is known.
  if (video_resize_width_ > 0 && video_resize_height_ > 0 && attached_) {
    PushSemanticsOptions();
  }
}

void LayersController::Recenter() {
  // c8EmAsm_semanticsControllerRecenter: the next staged frame sets the yaw offset again.
  has_set_yaw_offset_ = false;
  yaw_offset_ = c8::Quaternion();
}

void LayersController::PushSemanticsOptions() {
  // createSemanticsOptionsMsg + updateSemanticsOptions (layers-controller.cc:108-124).
  const int out_w = JsToInt32(video_resize_width_);
  const int out_h = JsToInt32(video_resize_height_);
  // Defaults of getOrigin(): position (0, 2, 0), rotation identity.
  Position3 pos{0, 2, 0};
  QuaternionXyzw rot{0, 0, 0, 1};
  if (has_coordinates_ && coordinates_.has_origin_position) {
    pos = coordinates_.origin_position;
  }
  if (has_coordinates_ && coordinates_.has_origin_rotation) {
    rot = coordinates_.origin_rotation;
  }
  Coordinates sem;
  sem.mirrored_display = has_coordinates_ && coordinates_.mirrored_display;
  sem.left_handed = has_coordinates_ && coordinates_.left_handed;
  sem.has_origin_position = true;
  sem.origin_position = pos;
  sem.has_origin_rotation = true;
  sem.origin_rotation = rot;
  const float near_clip = (has_near_ && near_clip_ > 0) ? static_cast<float>(near_clip_) : 0.01f;
  const float far_clip = (has_far_ && far_clip_ > 0) ? static_cast<float>(far_clip_) : 1000.0f;
  // We are in 'responsive' mode: scale = height of the camera origin.
  const float scale = static_cast<float>(pos.y > 0 ? pos.y : 1.0);
  bool invert = kDefaultInvertLayerMask;
  double edge = kDefaultEdgeSmoothness;
  if (has_sky_layer_) {
    invert = sky_options_.invert_layer_mask;
    edge = sky_options_.edge_smoothness;
  }
  if (out_w == 0 || out_h == 0 || renderer_ == nullptr) {
    return;  // updateSemanticsOptions: no GL resources before camera environment and output size
  }
  has_sem_opts_ = true;
  sem_out_w_ = out_w;
  sem_out_h_ = out_h;
  sem_near_ = near_clip;
  sem_far_ = far_clip;
  sem_coordinates_ = sem;
  sem_origin_scale_ = scale;
  sem_origin_pos_ = pos;
  sem_origin_rot_ = rot;
  UpdateIntrinsics();
  renderer_->InitOutputSemanticsScene(intrinsics_, out_w, out_h);
  renderer_->SetSkyPostProcessParameters(static_cast<float>(edge), have_seen_sky_, invert);
}

void LayersController::UpdateIntrinsics() {
  // updateIntrinsics(newEnv, newOpts) (layers-controller.cc:63-96) with the stored environment.
  if (!has_cam_env_ || cam_env_camera_w_ == 0 || cam_env_camera_h_ == 0 || renderer_ == nullptr ||
      !has_cam_env_estimate_) {
    return;
  }
  renderer_->SetDeviceOrientation(cam_env_orientation_);
  intrinsics_ = c8::Intrinsics::rotateCropAndScaleIntrinsics(cam_env_device_k_, cam_env_camera_w_,
                                                             cam_env_camera_h_);
  const auto kd = c8::Intrinsics::cropAndScaleIntrinsics(intrinsics_, cam_env_display_w_,
                                                         cam_env_display_h_);
  projection_matrix_ = ToClipSpace(kd, has_sem_opts_ ? sem_near_ : 0.0f,
                                   has_sem_opts_ ? sem_far_ : 0.0f);
  // Lower resolution for the semantics output rendering.
  const float ratio = 0.3f;
  const int w = static_cast<int>(ratio * cam_env_camera_w_);
  const int h = static_cast<int>(ratio * cam_env_camera_h_);
  renderer_->InitOutputSemanticsScene(intrinsics_, w, h);
  int crop_w = 0, crop_h = 0;
  CropDimensionByAspectRatio(cam_env_camera_w_, cam_env_camera_h_, 144.0f / 256.0f, &crop_w, &crop_h);
  const auto proj_intrinsics = c8::Intrinsics::rotateCropAndScaleIntrinsics(intrinsics_, crop_w, crop_h);
  renderer_->SetProjectionIntrinsics(proj_intrinsics);
}

void LayersController::UpdateResizerDimensions(int video_width, int video_height) {
  if (!attached_) {
    return;
  }
  if (video_width == src_video_width_ && video_height == src_video_height_) {
    return;
  }
  src_video_height_ = video_height;
  src_video_width_ = video_width;
  const ModelInputGeometry g = CalculateOutputSize(video_width, video_height);
  model_input_width_ = g.model_input_width;
  model_input_height_ = g.model_input_height;
  video_resize_width_ = g.video_resize_width;
  video_resize_height_ = g.video_resize_height;
  if (g.model_input_width == 0 || g.model_input_height == 0) {
    return;
  }
  if (has_resizer_ && g.model_input_width == dest_width_ && g.model_input_height == dest_height_) {
    return;
  }
  dest_width_ = g.model_input_width;
  dest_height_ = g.model_input_height;
  has_resizer_ = true;
  camera_resized_tex_.reset();
}

void LayersController::UpdateEnvironment(const SkyEnvironment& env) {
  env_in_ = env;
  env_video_width_ = env.video_width;
  env_video_height_ = env.video_height;
  env_canvas_width_ = env.canvas_width;
  env_canvas_height_ = env.canvas_height;
  env_orientation_ = env.orientation;
  UpdateResizerDimensions(env.video_width, env.video_height);
  // The GL part of updateEnvironment runs once the rendering context is registered (attached).
  if (!attached_) {
    return;
  }
  if (renderer_ == nullptr) {
    renderer_ = std::make_unique<SkyCubemapRenderer>();  // c8EmAsm_initSemanticsRenderer
    renderer_->set_seamless(cubemap_seamless_);
  }
  if (!has_semantics_tex_ || semantics_tex_width_ != JsToInt32(video_resize_width_) ||
      semantics_tex_height_ != JsToInt32(video_resize_height_)) {
    semantics_tex_width_ = JsToInt32(video_resize_width_);
    semantics_tex_height_ = JsToInt32(video_resize_height_);
    semantics_tex_ = Rgba8Image(semantics_tex_width_, semantics_tex_height_);
    has_semantics_tex_ = semantics_tex_width_ > 0 && semantics_tex_height_ > 0;
  }
  if (env.has_device_estimate) {
    // updateSemanticsCameraEnvironment(env): updateIntrinsics, then store the environment.
    has_cam_env_ = true;
    cam_env_camera_w_ = env_video_width_;
    cam_env_camera_h_ = env_video_height_;
    cam_env_display_w_ = env_canvas_width_;
    cam_env_display_h_ = env_canvas_height_;
    cam_env_orientation_ = env_orientation_;
    has_cam_env_estimate_ = true;
    cam_env_device_k_ = env.device_intrinsics;
    UpdateIntrinsics();
  }
  PushSemanticsOptions();
}

void LayersController::OnAttach(const SkyEnvironment& env) {
  attached_ = true;
  Dispatch("layerloading", "{}");
  UpdateEnvironment(env);
}

void LayersController::OnStart(const SkyEnvironment& env) {
  // onStart: drop the per-session GPU state, then updateEnvironment; the module is re-attached by
  // the framework afterwards (OnAttach).
  has_resizer_ = false;
  camera_resized_tex_.reset();
  camera_resized_tex_rotation_.reset();
  attached_ = false;
  src_video_width_ = 0;
  src_video_height_ = 0;
  dest_width_ = 0;
  dest_height_ = 0;
  UpdateEnvironment(env);
}

void LayersController::OnCanvasSizeChange(const SkyEnvironment& env) { UpdateEnvironment(env); }
void LayersController::OnVideoSizeChange(const SkyEnvironment& env) { UpdateEnvironment(env); }
void LayersController::OnDeviceOrientationChange(const SkyEnvironment& env) {
  UpdateEnvironment(env);
}

void LayersController::WorkerStarted() {
  is_invoke_ready_ = true;
  Dispatch("layerscanning", "{\"name\":\"sky\"}");
}

std::optional<SkyInvokeData> LayersController::OnProcessGpu(const Rgba8Image& camera,
                                                            bool repeat_frame,
                                                            const DeviceOrientation& orientation) {
  last_device_rotation_ = orientation;
  if (camera.empty() || repeat_frame) {
    return std::nullopt;
  }
  UpdateResizerDimensions(camera.width, camera.height);
  if (!has_resizer_) {
    return std::nullopt;
  }
  std::optional<SkyInvokeData> retval;
  if (camera_resized_tex_.has_value() && camera_resized_tex_rotation_.has_value()) {
    SkyInvokeData d;
    d.pixels = *camera_resized_tex_;
    d.cols = dest_width_;
    d.rows = dest_height_;
    d.rotation = *camera_resized_tex_rotation_;
    retval = std::move(d);
  }
  const QuaternionXyzw device_rotation = GetQuaternion(
      last_device_rotation_.alpha, last_device_rotation_.beta, last_device_rotation_.gamma);
  // Render the centre part of the video texture into the viewport.
  const double off_x = (video_resize_width_ - model_input_width_) / 2;
  const double off_y = (video_resize_height_ - model_input_height_) / 2;
  camera_resized_tex_ = ResizeCameraToModelInput(
      camera, dest_width_, dest_height_, JsToInt32(-off_x), JsToInt32(-off_y),
      JsToInt32(video_resize_width_), JsToInt32(video_resize_height_));
  camera_resized_tex_rotation_ = device_rotation;
  // Stage the current frame (c8EmAsm_stageLayersControllerFrame).
  StagedFrame s;
  s.device_pose_in_xr = c8::xrRotationFromDeviceRotation(ToC8Quaternion(device_rotation));
  pipeline_.push_back(s);
  return retval;
}

void LayersController::ProcessStagedFrame() {
  // processStagedLayersControllerFrame(outputSemanticsTexId, W, H).
  if (renderer_ == nullptr) {
    return;
  }
  while (pipeline_.size() > 2) {
    pipeline_.pop_front();  // "Processing queue is too full, draining old frames."
  }
  if (pipeline_.size() == 2) {
    semantics_tex_ = renderer_->DrawOutputSemantics(pipeline_[0].device_pose_in_xr.toRotationMat(),
                                                    semantics_tex_width_, semantics_tex_height_);
    StagedFrame& p = pipeline_[0];
    p.intrinsics = projection_matrix_;
    if (!has_set_yaw_offset_) {
      yaw_offset_ = c8::groundPlaneRotation(p.device_pose_in_xr).inverse();
      has_set_yaw_offset_ = true;
    }
    const c8::Quaternion responsive_extrinsic = yaw_offset_.times(p.device_pose_in_xr);
    float rw = responsive_extrinsic.w(), rx = responsive_extrinsic.x(),
          ry = responsive_extrinsic.y(), rz = responsive_extrinsic.z();
    float px = 0.0f, py = 0.0f, pz = 0.0f;
    // rewriteCoordinateSystemPostprocess(opts.coordinates, response) for SemanticsResponse.
    if (!sem_coordinates_.left_handed) {  // RIGHT_HANDED: makeRightHanded
      rw = -rw;
      rz = -rz;
      pz = -pz;
    }
    if (sem_coordinates_.mirrored_display) {  // mirror
      px = -px;
      ry = -1.0f * ry;
      rz = -1.0f * rz;
    }
    // recenterAndScale: rotation correction does not include pitch.
    const float s = sem_origin_scale_ > 0.0f ? sem_origin_scale_ : 1.0f;
    const c8::HVector3 op{static_cast<float>(sem_origin_pos_.x), static_cast<float>(sem_origin_pos_.y),
                          static_cast<float>(sem_origin_pos_.z)};
    const c8::Quaternion oq = c8::groundPlaneRotation(ToC8Quaternion(sem_origin_rot_));
    const c8::HMatrix o = c8::cameraMotion(op, oq);
    const c8::HPoint3 np = o * c8::HPoint3(s * px, s * py, s * pz);
    const c8::Quaternion nq = oq.times(c8::Quaternion(rw, rx, ry, rz));
    p.position = {np.x(), np.y(), np.z()};
    p.rotation = {nq.x(), nq.y(), nq.z(), nq.w()};
    p.has_response = true;
    active_ = p;
    has_active_ = true;
    pipeline_.pop_front();
  }
  // The EM_ASM at the end exposes `active` (and its camera texture, truthy once a frame was drawn).
  response_valid_ = has_active_ && active_.has_response;
}

void LayersController::UpdateCubemap(const std::vector<float>& data, const QuaternionXyzw& rotation) {
  // updateCubemap: float [0,1] -> *255 stored through a Uint8Array into an RGBA image.
  Rgba8Image img(kModelInputWidth, kModelInputHeight);
  for (size_t i = 0; i < data.size() && i * 4 + 3 < img.px.size(); ++i) {
    const uint8_t c = JsToUint8(static_cast<double>(data[i]) * 255.0);
    img.px[4 * i + 0] = c;
    img.px[4 * i + 1] = c;
    img.px[4 * i + 2] = c;
    img.px[4 * i + 3] = 255;
  }
  if (renderer_ == nullptr) {
    return;
  }
  // c8EmAsm_semanticsControllerRenderToCubemap -> renderToCubemap.
  const c8::HMatrix mat =
      c8::xrRotationFromDeviceRotation(ToC8Quaternion(rotation)).toRotationMat();
  renderer_->UpdateSrcTexture(img);
  renderer_->RenderToCubemap(mat);
}

void LayersController::OnWorkerResult(const std::vector<float>& data, double percentage,
                                      const QuaternionXyzw& rotation, const Position3& /*position*/) {
  percentage_ = percentage;
  if (!data.empty() && debug_input_mask_) {
    debug_semantics_mask_ = Rgba8Image(kModelInputWidth, kModelInputHeight);
    for (size_t i = 0; i < data.size() && 4 * i + 3 < debug_semantics_mask_.px.size(); ++i) {
      const uint8_t c = JsToUint8(static_cast<double>(data[i]) * 255.0);
      debug_semantics_mask_.px[4 * i + 0] = c;
      debug_semantics_mask_.px[4 * i + 1] = c;
      debug_semantics_mask_.px[4 * i + 2] = c;
      debug_semantics_mask_.px[4 * i + 3] = 255;
    }
  }
  bool found_now = false;
  if (percentage_ > kLayerFoundPercentage && !have_seen_sky_) {
    have_seen_sky_ = true;
    found_now = true;
    Dispatch("layerfound", "{\"name\":\"sky\",\"percentage\":" + JsNumber(percentage) + "}");
  }
  is_invoke_ready_ = true;
  if (!data.empty()) {
    UpdateCubemap(data, rotation);
  }
  if (found_now) {
    // The options push of the layerfound branch runs as a promise callback, after updateCubemap.
    PushSemanticsOptions();
  }
}

SkyCpuResult LayersController::OnProcessCpu(const std::optional<SkyInvokeData>& gpu,
                                            bool repeat_frame) {
  SkyCpuResult result;
  if (!repeat_frame) {
    if (has_semantics_tex_ && renderer_ != nullptr) {
      ProcessStagedFrame();
    }
    if (is_invoke_ready_ && gpu.has_value()) {
      is_invoke_ready_ = false;
      Rgba8Image resized(kModelInputWidth, kModelInputHeight);
      if (env_orientation_ == 90) {
        ToImageArrayRotateCW90(gpu->pixels.px.data(), gpu->cols, gpu->rows, kModelInputWidth,
                               kModelInputHeight, resized.px.data());
      } else if (env_orientation_ == -90) {
        ToImageArrayRotateCCW90(gpu->pixels.px.data(), gpu->cols, gpu->rows, kModelInputWidth,
                                kModelInputHeight, resized.px.data());
      } else {
        ToImageArrayLetterbox(gpu->pixels.px.data(), gpu->cols, gpu->rows, kModelInputWidth,
                              kModelInputHeight, resized.px.data());
      }
      SkyWorkerRequest req;
      req.img = std::move(resized);
      req.rotation = gpu->rotation;
      req.position = Position3{0, 0, 0};
      result.request = std::move(req);
    }
  }
  if (!response_valid_) {
    return result;
  }
  LayersFrameOutput& out = result.output;
  out.valid = true;
  out.position = active_.position;
  out.rotation = active_.rotation;
  out.intrinsics = active_.intrinsics;
  out.sky.texture = semantics_tex_;
  out.sky.texture_width = semantics_tex_width_;
  out.sky.texture_height = semantics_tex_height_;
  out.sky.percentage = percentage_;
  if (debug_input_mask_ && !debug_semantics_mask_.empty()) {
    out.sky.has_input_mask = true;
    out.sky.input_mask = debug_semantics_mask_;
  }
  return result;
}

}  // namespace slam_refact::sky

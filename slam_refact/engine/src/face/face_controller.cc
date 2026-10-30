// FaceController core (US-018/US-019). See face_controller.h; line references are to
// PUB:reality/app/xr/js/facecontroller.cc unless noted.

#include "face/face_controller.h"

#include <algorithm>
#include <chrono>
#include <cstring>
#include <utility>

#include "c8/geometry/facemesh-data.h"
#include "c8/geometry/intrinsics-clip.h"
#include "common/check.h"
#include "face/face_coordinates.h"
#include "face/face_models.h"
#include "ml/interpreter.h"
#include "ml/tflite_model.h"
#include "reality/engine/faces/face-geometry.h"

namespace slam_refact::face {
namespace {

double MsSince(std::chrono::steady_clock::time_point t0) {
  return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
}

// Bytes must be the deployed model and build on the slam_refact interpreter (the shim aborts
// otherwise, as the original aborts on a failed TFLite build).
bool ValidateModel(const std::vector<uint8_t>& model, const char* expected_sha256,
                   const char* name, std::string* error) {
  const std::string sha = Sha256Hex(model);
  if (sha != expected_sha256) {
    *error = std::string(name) + ": sha256 " + sha + " != expected " + expected_sha256;
    return false;
  }
  std::string parse_error;
  auto parsed = ml::TfliteModel::FromBytes(model, &parse_error);
  if (parsed == nullptr) {
    *error = std::string(name) + ": " + parse_error;
    return false;
  }
  auto interpreter = ml::Interpreter::Create(*parsed, &parse_error);
  if (interpreter == nullptr) {
    *error = std::string(name) + ": " + parse_error;
    return false;
  }
  return true;
}

Position32f ToPosition(const c8::HPoint3& p) { return {p.x(), p.y(), p.z()}; }
Position32f ToPosition(const c8::HVector3& p) { return {p.x(), p.y(), p.z()}; }

AttachmentPointOut ToAttachmentPoint(const c8::AttachmentPoint& anchor) {
  // face-messages.cc setAttachmentPoint (:34-46).
  return {anchor.name, ToPosition(anchor.position),
          {anchor.rotation.w(), anchor.rotation.x(), anchor.rotation.y(), anchor.rotation.z()}};
}

RgbaImageData ToImageData(c8::ConstRGBA8888PlanePixels im) {
  // face-messages.cc fillRenderedImage (:48-56) / fillEarRenderedImage.
  RgbaImageData out;
  out.rows = im.rows();
  out.cols = im.cols();
  out.bytes_per_row = im.rowBytes();
  out.pixels.assign(im.pixels(), im.pixels() + static_cast<size_t>(im.rows()) * im.rowBytes());
  return out;
}

DetectedPointsMsg FillDetection(const c8::DetectedPoints& p) {
  // face-messages.cc fillDetection (:63-90).
  DetectedPointsMsg d;
  d.confidence = p.confidence;
  d.viewport_x = p.viewport.x;
  d.viewport_y = p.viewport.y;
  d.viewport_w = p.viewport.w;
  d.viewport_h = p.viewport.h;
  d.corners = {{{p.boundingBox.upperLeft.x(), p.boundingBox.upperLeft.y()},
                {p.boundingBox.upperRight.x(), p.boundingBox.upperRight.y()},
                {p.boundingBox.lowerLeft.x(), p.boundingBox.lowerLeft.y()},
                {p.boundingBox.lowerRight.x(), p.boundingBox.lowerRight.y()}}};
  const auto transform = c8::renderTexToImageTex(p.roi);
  for (int i = 0; i < 16; ++i) {
    d.render_tex_to_image_tex[i] = transform.data()[i];
  }
  for (const auto& pp : p.points) {
    d.points.push_back({pp.x(), pp.y(), pp.z()});
  }
  d.detected_class = p.detectedClass;
  return d;
}

FaceMsg FillFace(const c8::Face3d& face_data) {
  // face-messages.cc fillFace (:141-215).
  FaceMsg b;
  b.id = face_data.id;
  switch (face_data.status) {
    case c8::Face3d::TrackingStatus::FOUND:
      b.status = FaceTrackingStatus::kFound;
      break;
    case c8::Face3d::TrackingStatus::UPDATED:
      b.status = FaceTrackingStatus::kUpdated;
      break;
    case c8::Face3d::TrackingStatus::LOST:
      b.status = FaceTrackingStatus::kLost;
      break;
    default:
      break;
  }
  if (!(face_data.status == c8::Face3d::TrackingStatus::FOUND ||
        face_data.status == c8::Face3d::TrackingStatus::UPDATED)) {
    return b;
  }
  const auto& t = face_data.transform;
  b.transform.position = ToPosition(t.position);
  b.transform.rotation = {t.rotation.w(), t.rotation.x(), t.rotation.y(), t.rotation.z()};
  b.transform.scale = t.scale;
  b.transform.scaled_width = t.scaledWidth;
  b.transform.scaled_height = t.scaledHeight;
  b.transform.scaled_depth = t.scaledDepth;
  for (const auto& v : face_data.vertices) {
    b.vertices.push_back(ToPosition(v));
  }
  // The original sizes normals by normals.size() and fills vertices.size() entries; computed
  // normals have one entry per vertex.
  for (size_t i = 0; i < face_data.vertices.size() && i < face_data.normals.size(); ++i) {
    b.normals.push_back(ToPosition(face_data.normals[i]));
  }
  for (const auto& uv : face_data.uvsInCameraSpace) {
    b.uvs_in_camera_frame.push_back({uv.x(), uv.y()});
  }
  for (const auto& anchor : face_data.attachmentPoints) {
    b.attachment_points.push_back(ToAttachmentPoint(anchor));
  }
  const auto& e = face_data.expressionOutput;
  b.mouth_open = e.mouthOpen;
  b.left_eye_open = e.leftEyeOpen;
  b.right_eye_open = e.rightEyeOpen;
  b.left_eyebrow_raised = e.leftEyebrowRaised;
  b.right_eyebrow_raised = e.rightEyebrowRaised;
  b.interpupillary_distance_in_mm = face_data.interpupillaryDistanceInMM;
  return b;
}

void FillEar(const c8::Ear3d& ears_data, FaceMsg* b) {
  // face-messages.cc fillEar (:224-268).
  b->ear_vertices.clear();
  for (const auto& v : ears_data.leftVertices) {
    b->ear_vertices.push_back(ToPosition(v));
  }
  for (const auto& v : ears_data.rightVertices) {
    b->ear_vertices.push_back(ToPosition(v));
  }
  b->ear_attachment_points.clear();
  for (const auto& anchor : ears_data.leftAttachmentPoints) {
    b->ear_attachment_points.push_back(ToAttachmentPoint(anchor));
  }
  for (const auto& anchor : ears_data.rightAttachmentPoints) {
    b->ear_attachment_points.push_back(ToAttachmentPoint(anchor));
  }
  const auto& s = ears_data.earStatesOutput;
  b->left_ear_found = s.leftEarFound;
  b->right_ear_found = s.rightEarFound;
  b->left_lobe_found = s.leftLobeFound;
  b->left_canal_found = s.leftCanalFound;
  b->left_helix_found = s.leftHelixFound;
  b->right_lobe_found = s.rightLobeFound;
  b->right_canal_found = s.rightCanalFound;
  b->right_helix_found = s.rightHelixFound;
}

}  // namespace

FaceOptions DefaultFaceOptions() {
  FaceOptions o;
  o.has_debug = false;
  o.near_clip = 0.01f;
  o.far_clip = 1000.0f;
  o.mesh_geometry = {MeshGeometryOption::kFace};
  o.use_standard_uvs = true;
  o.max_detections = 1;
  o.coordinates.origin_position = {0.0f, 0.0f, 0.0f};
  o.coordinates.origin_rotation = {1.0f, 0.0f, 0.0f, 0.0f};
  o.coordinates.mirrored_display = false;
  o.coordinates.scale = 1.0f;
  o.coordinates.axes = Axes::kRightHanded;
  o.enable_ears = false;
  return o;
}

c8_PixelPinholeCameraModel DefaultDeviceIntrinsics() {
  return c8_PixelPinholeCameraModel{480, 640, 239.5f, 319.5f, 506.4104f, 506.4104f};
}

std::array<float, 16> ToClipSpace(const c8_PixelPinholeCameraModel& intrinsics, float kn,
                                  float kf) {
  // intrinsics.cc:81-106 (column-major).
  const float c00 = 2 * intrinsics.focalLengthHorizontal / intrinsics.pixelsWidth;
  const float c11 = 2 * intrinsics.focalLengthVertical / intrinsics.pixelsHeight;
  const float c02 = 1.0f - (2.0f * intrinsics.centerPointX + 1.0f) / intrinsics.pixelsWidth;
  const float c12 = -1.0f + (2.0f * intrinsics.centerPointY + 1.0f) / intrinsics.pixelsHeight;
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

bool FaceController::LoadFaceDetectModel(const std::vector<uint8_t>& model, std::string* error) {
  if (!ValidateModel(model, kFaceDetectModelSha256, "face-model", error)) {
    return false;
  }
  tracker_.setFaceDetectModel(model.data(), static_cast<int>(model.size()));
  return true;
}

bool FaceController::LoadFaceMeshModel(const std::vector<uint8_t>& model, std::string* error) {
  if (!ValidateModel(model, kFaceMeshModelSha256, "face-mesh-model", error)) {
    return false;
  }
  // FaceTracker::setFaceMeshModel copies the current EarConfig into FaceDetectorLocal, so
  // enableEars must be set (UpdateFaceOptions) before this call, as the JS side does.
  tracker_.setFaceMeshModel(model.data(), static_cast<int>(model.size()));
  return true;
}

bool FaceController::LoadEarModel(const std::vector<uint8_t>& model, std::string* error) {
  if (!ValidateModel(model, kFaceEarModelSha256, "face-ear-model", error)) {
    return false;
  }
  tracker_.setEarModel(model.data(), static_cast<int>(model.size()));
  return true;
}

void FaceController::UpdateIntrinsics(const FaceCameraEnvironment& env,
                                      const FaceOptions& options) {
  // updateIntrinsics (:62-72).
  intrinsics_ = c8::Intrinsics::rotateCropAndScaleIntrinsics(env.device_intrinsics,
                                                             env.camera_width, env.camera_height);
  const auto kd =
      c8::Intrinsics::cropAndScaleIntrinsics(intrinsics_, env.display_width, env.display_height);
  projection_matrix_ = ToClipSpace(kd, options.near_clip, options.far_clip);
}

void FaceController::UpdateVideoGeometry(const FaceCameraEnvironment& env) {
  // updateVideoGeometry (:74-96).
  if (env.camera_width == env_.camera_width && env.camera_height == env_.camera_height &&
      face_renderer_ != nullptr) {
    return;
  }
  face_renderer_.reset();
  ear_renderer_.reset();
  if (env.camera_width == 0 || env.camera_height == 0) {
    return;
  }
  face_renderer_ = std::make_unique<FaceRoiRendererCpu>();
  face_renderer_->Initialize(env.camera_width, env.camera_height);
  ear_renderer_ = std::make_unique<EarRoiRendererCpu>();
  ear_renderer_->Initialize(env.camera_width, env.camera_height);
}

void FaceController::UpdateCameraEnvironment(const FaceCameraEnvironment& env) {
  // updateCameraEnvironment (:110-114).
  UpdateVideoGeometry(env);
  UpdateIntrinsics(env, opts_);
  env_ = env;
}

void FaceController::UpdateFaceOptions(const FaceOptions& options) {
  // updateFaceOptions (:116-120).
  UpdateIntrinsics(env_, options);
  opts_ = options;
  tracker_.setIsTrackingEars(opts_.enable_ears);
}

const ModelGeometryMsg& FaceController::GetFrameworkData() {
  // getFrameworkData (:262-292).
  if (!has_framework_data_ || framework_data_.max_detections == 0) {
    ModelGeometryMsg m;
    m.points_per_detection = NUM_FACEMESH_POINTS;
    m.max_detections = opts_.max_detections;
    c8::FaceTracker::setMaxTrackedFaces(opts_.max_detections);
    // face-messages.cc fillIndices (:281-310).
    c8::FaceMeshGeometryConfig config;
    for (const auto option : opts_.mesh_geometry) {
      if (option == MeshGeometryOption::kFace) {
        config.showFace = true;
      }
      if (option == MeshGeometryOption::kEyes) {
        config.showEyes = true;
      }
      if (option == MeshGeometryOption::kMouth) {
        config.showMouth = true;
      }
      if (option == MeshGeometryOption::kIris) {
        config.showIris = true;
      }
    }
    for (const auto& idx : c8::meshIndicesFromMeshGeometry(config)) {
      m.indices.push_back({static_cast<int32_t>(idx.a), static_cast<int32_t>(idx.b),
                           static_cast<int32_t>(idx.c)});
    }
    const auto& uvs = opts_.use_standard_uvs ? FACEMESH_UVS : FACEMESH_FLAT_UVS;
    for (int i = 0; i < NUM_FACEMESH_POINTS; ++i) {
      m.uvs.push_back({uvs[i].u, uvs[i].v});
    }
    RewriteCoordinateSystemFrameworkPostprocess(opts_.coordinates, &m);
    framework_data_ = std::move(m);
    has_framework_data_ = true;
  }
  return framework_data_;
}

bool FaceController::StageFrame(const RgbaImageView& frame, std::string* error) {
  // stageFaceControllerFrame (:131-145).
  const auto t0 = std::chrono::steady_clock::now();
  auto fail = [error](const char* message) {
    if (error != nullptr) {
      *error = message;
    }
    return false;
  };
  if (face_renderer_ == nullptr) {
    UpdateVideoGeometry(env_);
  }
  if (face_renderer_ == nullptr || ear_renderer_ == nullptr) {
    return fail("FaceController::StageFrame: camera environment has zero size");
  }
  if (frame.data == nullptr || frame.width != env_.camera_width ||
      frame.height != env_.camera_height || frame.stride < 4 * frame.width) {
    // The original re-creates its renderers in updateVideoGeometry() when the camera size
    // changes (UpdateCameraEnvironment); a frame that does not match the environment is rejected
    // instead of aborting, and nothing is queued.
    return fail("FaceController::StageFrame: frame size differs from the camera environment");
  }
  pipeline_.emplace_back();
  auto& p = pipeline_.back();
  p.capture_width = env_.camera_width;
  p.capture_height = env_.camera_height;
  p.capture_rotation = env_.orientation;
  face_renderer_->DrawDeferred(frame);
  ear_renderer_->DrawDeferred(frame);
  timings_ = FaceStageTimings();
  timings_.render_ms = MsSince(t0);
  return true;
}

const FaceResponse* FaceController::ProcessStagedFrame() {
  // processStagedFaceControllerFrame (:147-260).
  while (pipeline_.size() > 2) {
    pipeline_.pop_front();  // "Processing queue is too full, draining old frames."
  }
  if (pipeline_.size() == 2 && (face_renderer_ == nullptr || ear_renderer_ == nullptr)) {
    // UpdateCameraEnvironment() with a zero camera size between StageFrame() and here dropped
    // the renderers; there is no render to process, so the staged frames are discarded.
    pipeline_.clear();
  }
  if (pipeline_.size() == 2) {
    const auto& opts = opts_;
    auto t0 = std::chrono::steady_clock::now();
    const auto face_render_result = face_renderer_->Result();
    const c8::DebugOptions::Reader debug_reader(opts.has_debug && opts.debug.only_detect_faces);
    const auto track_result = tracker_.track(face_render_result, intrinsics_, debug_reader);
    face_renderer_->SetDetectedFaces(*track_result.globalFaces, *track_result.localFaces);
    timings_.track_ms = MsSince(t0);

    t0 = std::chrono::steady_clock::now();
    FaceResponse response;
    if (opts.has_debug) {
      response.has_debug = true;
      if (opts.debug.rendered_img) {
        response.debug.has_rendered_img = true;
        response.debug.rendered_img = ToImageData(face_render_result.rawPixels);
      }
      if (opts.debug.detections) {
        response.debug.detections.has_faces = true;
        for (const auto& f : *track_result.globalFaces) {
          response.debug.detections.faces.push_back(FillDetection(f));
        }
        for (const auto& m : *track_result.localFaces) {
          response.debug.detections.meshes.push_back(FillDetection(m));
        }
      }
    }
    for (const auto& face_data : *track_result.faceData) {
      response.faces.push_back(FillFace(face_data));
    }
    double response_ms = MsSince(t0);

    if (opts.enable_ears) {
      t0 = std::chrono::steady_clock::now();
      const auto ear_render_result = ear_renderer_->Result();
      const auto ear_track_result = tracker_.trackEars(ear_render_result, intrinsics_);
      timings_.ears_ms = MsSince(t0);
      t0 = std::chrono::steady_clock::now();
      if (opts.has_debug) {
        if (opts.debug.ear_rendered_img) {
          response.debug.has_ear_rendered_img = true;
          response.debug.ear_rendered_img = ToImageData(ear_render_result.rawPixels);
        }
        if (opts.debug.ear_detections) {
          // :188-222.
          std::vector<Position32f> ear_in_face_roi;
          std::vector<Position32f> ear_in_camera_feed;
          const auto& local_ears = *ear_track_result.localEars;
          if (!track_result.localFaces->empty() && !local_ears.empty()) {
            for (size_t e = 0; e < 2 && e < local_ears.size(); ++e) {
              const auto& local_ear = local_ears[e];
              const c8::HMatrix to_face =
                  c8::renderTexToRenderTex(local_ear.roi, (*track_result.localFaces)[0].roi);
              const c8::HMatrix to_image = c8::renderTexToImageTex(local_ear.roi);
              for (size_t i = 2; i < local_ear.points.size(); ++i) {
                ear_in_face_roi.push_back(ToPosition(to_face * local_ear.points[i]));
                ear_in_camera_feed.push_back(ToPosition(to_image * local_ear.points[i]));
              }
            }
          }
          // face-messages.cc fillEarDetections (:105-139).
          response.debug.detections.has_ears = true;
          for (const auto& ear : local_ears) {
            response.debug.detections.ears.push_back(FillDetection(ear));
          }
          response.debug.detections.ears_in_face_roi = ear_in_face_roi;
          response.debug.detections.ears_in_camera_feed = ear_in_camera_feed;
        }
      }
      // :224-227. Set the next ROIs from local ears if there are local faces.
      if (!track_result.localFaces->empty() && !ear_track_result.localEars->empty()) {
        ear_renderer_->SetDetectedEars(*ear_track_result.localEars);
      }
      // face-messages.cc fillEars (:270-279): ear data i goes to face i.
      const auto& ears_data = *ear_track_result.earData;
      SR_CHECK(response.faces.size() >= ears_data.size(),
               "[face-messages] FaceResponse has fewer faces than required.");
      for (size_t i = 0; i < ears_data.size(); ++i) {
        FillEar(ears_data[i], &response.faces[i]);
      }
      response_ms += MsSince(t0);
    }

    t0 = std::chrono::steady_clock::now();
    // :232-240. by default the capnp Quaternion32f is created with values 0.0w, 0.0x, 0.0y, 0.0z
    response.camera.intrinsic = projection_matrix_;
    response.camera.extrinsic_rotation.w = 1.0f;
    RewriteCoordinateSystemPostprocess(opts.coordinates, &response);
    response_ms += MsSince(t0);
    timings_.response_ms = response_ms;
    timings_.processed = true;

    active_ = std::move(pipeline_.front());
    active_.response = std::move(response);
    active_.has_response = true;
    pipeline_.pop_front();
  }
  return active_.has_response ? &active_.response : nullptr;
}

void FaceController::Cleanup() {
  // faceControllerCleanup (:122-130).
  tracker_.reset();
  face_renderer_.reset();
  ear_renderer_.reset();
  pipeline_.clear();
  active_ = PipelineData();
  has_framework_data_ = false;
  framework_data_ = ModelGeometryMsg();
}

}  // namespace slam_refact::face

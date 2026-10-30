// FaceController core without GL, Cap'n Proto or emscripten (US-018/US-019).
//
// Port of the C++ half of XR8.FaceController, PUB:reality/app/xr/js/facecontroller.cc, with
//   - FaceRoiRenderer / EarRoiRenderer (GL)  -> FaceRoiRendererCpu / EarRoiRendererCpu,
//   - the FaceTracker, detectors, TrackedFaceState and ear code -> the vendored public sources
//     (third_party/xr_source/reality/engine/{faces,ears,deepnets}) running the same .tflite models
//     on the slam_refact ML interpreter, src/ml (tflite-interpreter shim),
//   - capnp FaceResponse / FrameworkResponse -> face_messages.h, filled as face-messages.cc does,
//   - rewriteCoordinateSystemPostprocess for FaceResponse/FrameworkResponse
//     (PUB:reality/engine/geometry/orientation.cc:181-245, 383-432, 507-571) -> face_coordinates.
// The camera texture is an RGBA frame (row 0 = top). The two-entry pipeline of the original
// (stage frame N, process frame N-1) is kept: ProcessStagedFrame() returns the response for the
// frame staged one call earlier, or nullptr before the first one exists (the JS side returns null
// while window._c8.responseTexture is unset). Event emission (facefound, mouthopened, blinked,
// earfound, ...) is face_events.h.
//
// FaceTracker::setMaxTrackedFaces is a process-global static in the original and stays one here
// (GetFrameworkData sets it), so one FaceController per process is the supported use.
#ifndef SLAM_REFACT_FACE_FACE_CONTROLLER_H_
#define SLAM_REFACT_FACE_FACE_CONTROLLER_H_

#include <array>
#include <cstdint>
#include <deque>
#include <memory>
#include <string>
#include <vector>

#include "c8/geometry/pixel-pinhole-camera-model.h"
#include "face/face_messages.h"
#include "face/face_roi_renderer.h"
#include "reality/engine/faces/face-tracker.h"
#include "slam_refact/image_types.h"

namespace slam_refact::face {

// FaceOptions exactly as face-controller.ts updateOptions writes them for a configure() call that
// sets nothing (options_ defaults :343-354, updateOptions :621-690): debug unset, nearClip 0.01,
// farClip 1000, meshGeometry [FACE], useStandardUvs true, maxDetections 1, origin identity,
// mirroredDisplay false, scale 1, axes RIGHT_HANDED, enableEars false.
FaceOptions DefaultFaceOptions();

// Intrinsics::getCameraIntrinsics default branch (PUB:c8/geometry/intrinsics.cc:358-360, 470-472):
// 480x640, centre (239.5, 319.5), focal 506.4104. The JS side passes the device estimate; any
// device-specific focal can be given in FaceCameraEnvironment::device_intrinsics instead.
c8_PixelPinholeCameraModel DefaultDeviceIntrinsics();

// Intrinsics::toClipSpace (intrinsics.cc:81-118), the projection matrix of the response camera.
std::array<float, 16> ToClipSpace(const c8_PixelPinholeCameraModel& k, float near_clip,
                                  float far_clip);

// CameraEnvironment fields used by facecontroller.cc (camera = video texture, display = canvas).
struct FaceCameraEnvironment {
  int32_t camera_width = 0;
  int32_t camera_height = 0;
  int32_t display_width = 0;
  int32_t display_height = 0;
  int32_t orientation = 0;
  c8_PixelPinholeCameraModel device_intrinsics = DefaultDeviceIntrinsics();
};

// Wall time of the stages of one StageFrame + ProcessStagedFrame pair (milliseconds,
// steady_clock), for docs/spec/08-face-ear.md §7. render = CPU ROI rendering (GPU in the
// original), track = FaceTracker::track (detectors + 3D), ears = trackEars, response = message
// filling and the coordinate rewrite.
struct FaceStageTimings {
  double render_ms = 0.0;
  double track_ms = 0.0;
  double ears_ms = 0.0;
  double response_ms = 0.0;
  bool processed = false;
};

class FaceController {
 public:
  FaceController() = default;
  FaceController(const FaceController&) = delete;
  FaceController& operator=(const FaceController&) = delete;

  // c8EmAsm_loadFaceDetectModel / loadFaceMeshModel / loadEarModel. The bytes must be the deployed
  // models (sha256 of face-model, face-mesh-model, face-ear-model = ear_model_v1, see
  // face_models.h) and must build on the slam_refact interpreter; otherwise false with `error` and
  // no state change.
  bool LoadFaceDetectModel(const std::vector<uint8_t>& model, std::string* error);
  bool LoadFaceMeshModel(const std::vector<uint8_t>& model, std::string* error);
  bool LoadEarModel(const std::vector<uint8_t>& model, std::string* error);

  // c8EmAsm_updateCameraEnvironment / c8EmAsm_updateFaceOptions.
  void UpdateCameraEnvironment(const FaceCameraEnvironment& env);
  void UpdateFaceOptions(const FaceOptions& options);

  // c8EmAsm_getFrameworkData: model geometry (indices for meshGeometry, standard or flat uvs) after
  // the framework coordinate rewrite; also FaceTracker::setMaxTrackedFaces(maxDetections). Cached
  // until Cleanup(), like the original (rebuilt while maxDetections is 0).
  const ModelGeometryMsg& GetFrameworkData();

  // c8EmAsm_stageFaceControllerFrame: queue the frame and draw its face/ear crops (deferred read).
  // Returns false (nothing queued, `error` set when non-null) if the camera environment has zero
  // size or the frame does not match its camera size; the caller can re-send
  // UpdateCameraEnvironment() with the new size and stage again.
  bool StageFrame(const RgbaImageView& frame, std::string* error = nullptr);
  // c8EmAsm_processStagedFaceControllerFrame: process the previous staged frame when two are
  // queued and return the active response (nullptr until one exists).
  const FaceResponse* ProcessStagedFrame();

  // c8EmAsm_faceControllerCleanup.
  void Cleanup();

  const FaceStageTimings& last_timings() const { return timings_; }
  const c8_PixelPinholeCameraModel& intrinsics() const { return intrinsics_; }
  const std::array<float, 16>& projection_matrix() const { return projection_matrix_; }
  // Debug access for tests (FaceTracker::faceToUprightBBoxInRay etc.).
  const c8::FaceTracker& tracker() const { return tracker_; }
  const FaceRoiRendererCpu* face_renderer() const { return face_renderer_.get(); }
  const EarRoiRendererCpu* ear_renderer() const { return ear_renderer_.get(); }

 private:
  struct PipelineData {
    int32_t capture_width = 0;
    int32_t capture_height = 0;
    int32_t capture_rotation = 0;
    bool has_response = false;
    FaceResponse response;
  };

  void UpdateIntrinsics(const FaceCameraEnvironment& env, const FaceOptions& options);
  void UpdateVideoGeometry(const FaceCameraEnvironment& env);

  c8::FaceTracker tracker_;
  std::unique_ptr<FaceRoiRendererCpu> face_renderer_;
  std::unique_ptr<EarRoiRendererCpu> ear_renderer_;
  FaceCameraEnvironment env_;
  FaceOptions opts_;
  std::deque<PipelineData> pipeline_;
  PipelineData active_;
  c8_PixelPinholeCameraModel intrinsics_;
  std::array<float, 16> projection_matrix_{};
  bool has_framework_data_ = false;
  ModelGeometryMsg framework_data_;
  FaceStageTimings timings_;
};

}  // namespace slam_refact::face

#endif  // SLAM_REFACT_FACE_FACE_CONTROLLER_H_

// Plain-struct replacements of the face Cap'n Proto messages (US-018/US-019).
//
// Field names and order follow PUB:reality/engine/api/face.capnp (FaceOptions :23-45,
// FaceAnchorTransformMsg :47-54, ModelGeometryMsg :58-67, DebugDetections :69-75, DebugResponse
// :77-83, AttachmentPointMsg :85-127, FaceMsg :129-165, FaceResponse :167-171), mesh.capnp
// (DebugOptions, DetectedPointsMsg) and base/camera-intrinsics.capnp (CameraCoordinates,
// GraphicsCamera). Cap'n Proto zero defaults are kept (a Quaternion32f defaults to all zeros, not
// identity). Filling follows PUB:reality/engine/faces/face-messages.cc; docs/spec/08-face-ear.md
// §2 maps each field to the FaceController event detail.
#ifndef SLAM_REFACT_FACE_FACE_MESSAGES_H_
#define SLAM_REFACT_FACE_FACE_MESSAGES_H_

#include <array>
#include <cstdint>
#include <vector>

#include "reality/engine/api/face.capnp.h"

namespace slam_refact::face {

using AttachmentName = c8::AttachmentPointMsg::AttachmentName;

struct Position32f {
  float x = 0.0f;
  float y = 0.0f;
  float z = 0.0f;
};

struct Quaternion32f {
  float w = 0.0f;
  float x = 0.0f;
  float y = 0.0f;
  float z = 0.0f;
};

struct UvMsg {
  float u = 0.0f;
  float v = 0.0f;
};

// camera-intrinsics.capnp CameraCoordinates.
enum class Axes : uint16_t { kUnspecified = 0, kLeftHanded = 1, kRightHanded = 2 };

struct CameraCoordinates {
  Position32f origin_position;
  Quaternion32f origin_rotation;
  float scale = 0.0f;
  Axes axes = Axes::kUnspecified;
  bool mirrored_display = false;
};

// FaceOptions.meshGeometry entries.
enum class MeshGeometryOption : uint16_t {
  kUnspecified = 0,
  kFace = 1,
  kEyes = 2,
  kMouth = 3,
  kIris = 4
};

// mesh.capnp ExtraOutputMask + DebugOptions.
struct FaceDebugOptions {
  bool rendered_img = false;
  bool detections = false;
  bool ear_rendered_img = false;
  bool ear_detections = false;
  bool only_detect_faces = false;
};

// FaceOptions as written by face-controller.ts updateOptions (:621-690). Use
// FaceOptionsFromConfigure() for the JS defaults.
struct FaceOptions {
  bool has_debug = false;
  FaceDebugOptions debug;
  float near_clip = 0.0f;
  float far_clip = 0.0f;
  CameraCoordinates coordinates;
  std::vector<MeshGeometryOption> mesh_geometry;
  bool use_standard_uvs = false;
  int32_t max_detections = 0;
  bool enable_ears = false;
};

struct FaceAnchorTransformMsg {
  Position32f position;
  Quaternion32f rotation;
  float scale = 0.0f;
  float scaled_width = 0.0f;
  float scaled_height = 0.0f;
  float scaled_depth = 0.0f;
};

struct AttachmentPointOut {
  AttachmentName name = AttachmentName::UNSPECIFIED;
  Position32f position;
  Quaternion32f rotation;
};

enum class FaceTrackingStatus : uint16_t { kUnspecified = 0, kFound = 1, kUpdated = 2, kLost = 3 };

struct FaceMsg {
  int32_t id = 0;
  FaceTrackingStatus status = FaceTrackingStatus::kUnspecified;
  FaceAnchorTransformMsg transform;
  std::vector<Position32f> vertices;
  std::vector<Position32f> normals;
  std::vector<AttachmentPointOut> attachment_points;
  std::vector<UvMsg> uvs_in_camera_frame;

  bool mouth_open = false;
  bool left_eye_open = false;
  bool right_eye_open = false;
  bool left_eyebrow_raised = false;
  bool right_eyebrow_raised = false;

  float interpupillary_distance_in_mm = 0.0f;

  std::vector<Position32f> ear_vertices;
  std::vector<AttachmentPointOut> ear_attachment_points;

  bool left_ear_found = false;
  bool right_ear_found = false;
  bool left_lobe_found = false;
  bool left_canal_found = false;
  bool left_helix_found = false;
  bool right_lobe_found = false;
  bool right_canal_found = false;
  bool right_helix_found = false;
};

// mesh.capnp DetectedPointsMsg with geo-types ImageBoxRoi (corners + renderTexToImageTex).
struct DetectedPointsMsg {
  float confidence = 0.0f;
  float viewport_x = 0.0f;
  float viewport_y = 0.0f;
  float viewport_w = 0.0f;
  float viewport_h = 0.0f;
  // upperLeft, upperRight, lowerLeft, lowerRight; x, y.
  std::array<std::array<float, 2>, 4> corners{};
  std::array<float, 16> render_tex_to_image_tex{};
  std::vector<Position32f> points;
  int32_t detected_class = 0;
};

struct RgbaImageData {
  int32_t rows = 0;
  int32_t cols = 0;
  int32_t bytes_per_row = 0;
  std::vector<uint8_t> pixels;
};

struct DebugDetections {
  bool has_faces = false;
  std::vector<DetectedPointsMsg> faces;
  std::vector<DetectedPointsMsg> meshes;
  bool has_ears = false;
  std::vector<DetectedPointsMsg> ears;
  std::vector<Position32f> ears_in_face_roi;
  std::vector<Position32f> ears_in_camera_feed;
};

struct DebugResponse {
  bool has_rendered_img = false;
  RgbaImageData rendered_img;
  DebugDetections detections;
  bool has_ear_rendered_img = false;
  RgbaImageData ear_rendered_img;
};

// base/camera-intrinsics.capnp GraphicsCamera (extrinsic Transform32f, intrinsic matrix44f).
struct GraphicsCamera {
  Position32f extrinsic_position;
  Quaternion32f extrinsic_rotation;
  std::array<float, 16> intrinsic{};
};

struct FaceResponse {
  bool has_debug = false;
  DebugResponse debug;
  GraphicsCamera camera;
  std::vector<FaceMsg> faces;
};

struct MeshFaceIdxs {
  int32_t a = 0;
  int32_t b = 0;
  int32_t c = 0;
};

struct ModelGeometryMsg {
  int32_t max_detections = 0;
  int32_t points_per_detection = 0;
  std::vector<MeshFaceIdxs> indices;
  std::vector<UvMsg> uvs;
};

// face-controller.ts AttachmentPoints values (:35-72): the event detail key of each enum value,
// e.g. "forehead", "leftHelix". Empty for UNSPECIFIED.
const char* AttachmentJsName(AttachmentName name);

}  // namespace slam_refact::face

#endif  // SLAM_REFACT_FACE_FACE_MESSAGES_H_

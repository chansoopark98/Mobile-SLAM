// slam_refact public API: XrController-level types (contract FULL-1, spec 06).
//
// FROZEN CONTRACT. Changes follow slam_refact/docs/design/contract-changes.md.
// The types mirror the original XR8.XrController surface (configure, updateCameraProjectionMatrix,
// hitTest, reality.lighting, onCameraStatusChange); GlueRunner implements them.

#ifndef SLAM_REFACT_XR_CONTROLLER_H_
#define SLAM_REFACT_XR_CONTROLLER_H_

#include <array>
#include <cstdint>
#include <optional>
#include <string>

#include "slam_refact/types.h"

namespace slam_refact {

// Display (graphics) intrinsics input: the original JS `cam` object
// {pixelRectWidth, pixelRectHeight, nearClipPlane, farClipPlane}. A texture size of 0 means "the
// canvas size", which the glue takes as the capture size (spec 06 §1.2).
struct GraphicsIntrinsics {
  int texture_width = 0;
  int texture_height = 0;
  float near_clip = 0.01f;
  float far_clip = 1000.0f;
};

// updateCameraProjectionMatrix({cam, origin, facing, updateRecenterPoint}); every cam field is
// merged separately like Object.assign.
struct CameraProjectionUpdate {
  std::optional<int> pixel_rect_width;
  std::optional<int> pixel_rect_height;
  std::optional<float> near_clip_plane;
  std::optional<float> far_clip_plane;
  std::optional<Vec3f> origin;
  std::optional<QuatWxyz> facing;
  bool update_recenter_point = true;
};

// configure({...}); unset fields are left unchanged. absolute_scale = (scale == 'absolute').
struct XrConfigureOptions {
  std::optional<bool> enable_world_points;
  std::optional<bool> absolute_scale;
  std::optional<bool> enable_lighting;
  std::optional<bool> left_handed_axes;
  std::optional<bool> mirrored_display;
  std::optional<bool> disable_world_tracking;
};

// XrHitTestResult.ResultType (reality.capnp:307-313); the request mask bit is 1 << value.
enum class XrHitType : uint8_t {
  kUnspecified = 0,
  kFeaturePoint = 1,
  kEstimatedSurface = 2,
  kDetectedSurface = 3,
};

// One hitTest result in the published frame. rotation is what the original fills: nothing (the
// capnp default 0 quaternion, sign-flipped by the right-handed conversion).
struct XrHitResult {
  XrHitType type = XrHitType::kUnspecified;
  Vec3f position;
  QuatWxyz rotation{0.f, 0.f, 0.f, 0.f};
  float distance = 0.f;
};

// reality.lighting {exposure, temperature}.
struct LightingEstimate {
  float exposure = 0.f;
  float temperature = 0.f;
};

// XR8.XrConfig.camera() direction of onCameraStatusChange's cameraConfig.
enum class CameraDirection : uint8_t {
  kBack = 0,
  kFront = 1,
};

// Error of a configure / camera status call: empty code = no error. message is the original JS
// Error message; code is the original `err` field when it has one.
struct XrError {
  std::string code;
  std::string message;
  bool ok() const { return message.empty(); }
};

}  // namespace slam_refact

#endif  // SLAM_REFACT_XR_CONTROLLER_H_

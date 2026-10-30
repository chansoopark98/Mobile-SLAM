// slam_refact public API: image targets (contract FULL-3, full-parity PRD US-013,
// slam_refact/docs/design/image-target.md §4, §7).
//
// FROZEN CONTRACT. Changes follow slam_refact/docs/design/contract-changes.md.
// c8-free: the loaded detection images and the pyramid ROI lists are opaque engine types.

#ifndef SLAM_REFACT_IMAGE_TARGETS_H_
#define SLAM_REFACT_IMAGE_TARGETS_H_

#include <cstdint>
#include <string>

#include "slam_refact/image_types.h"
#include "slam_refact/types.h"

namespace slam_refact {

// Active (loaded) image targets. The original runtime does not check it; the reference runtime console
// limits a project to this many (PUB:reality/cloud/xrhome/src/shared/xrengine-config.js:3
// MAXIMUM_ACTIVE_IMAGE_TARGETS). GlueRunner::SetImageTargets rejects a larger set.
inline constexpr int kMaxActiveImageTargets = 32;
// Located images per frame (PUB:reality/engine/imagedetection/detection-image-tracker.cc:45
// MAX_NUM_TRACKED_IMAGES; enforced by the vendored tracker). Also the number of pyramid ROI slots.
inline constexpr int kMaxTrackedImageTargets = 4;

// Opaque engine types (src/image_target/): the loaded DetectionImageMap and a pyramid ROI list.
class ImageTargetSet;
struct ImageRoiList;

// One target to load: the CLI / Studio target JSON (image-target-cli output) and its decoded
// luminance image (the runtime has no JPEG/PNG decoder).
struct ImageTargetInput {
  std::string json;
  GrayImageView luminance{};
};

// ImageTargetTypeMsg (PUB:reality/engine/api/reality.capnp:350-354).
enum class DetectedImageType : int32_t {
  kUnspecified = 0,
  kPlanar = 1,
  kCurvy = 2,
};

// DetectedImage.TrackingStatus (reality.capnp:358-363). The C8 path never sets it
// (image-detection-request-executor.cc:181-272), so published images carry kUnspecified.
enum class DetectedImageTrackingStatus : int32_t {
  kUnspecified = 0,
  kFullTracking = 1,
  kLastKnownPose = 2,
  kNotTracking = 3,
};

// CurvyGeometry (reality.capnp:384-396); filled only for kCurvy.
struct CurvyGeometryOut {
  float curvy_circumference_top = 0.f;
  float curvy_circumference_bottom = 0.f;
  float curvy_side_length = 0.f;
  float target_circumference_top = 0.f;
  float height = 0.f;
  float top_radius = 0.f;
  float bottom_radius = 0.f;
  float arc_length_radians = 0.f;
  float arc_start_radians = 0.f;
};

// One LocatedImage of 938 (PUB:reality/engine/imagedetection/tracked-image.h:38-45) in internal
// XR coordinates after applyToLocatedImages (the yaw / metric transform of the returned pose).
struct LocatedImageOut {
  std::string name;
  int32_t id = -1;  // roi.faceId = TrackedImage::index (always -1 in the public source)
  HMatrix4f pose;   // world_from_target: target centre, target-view axes
  float width = 0.f;   // frameBounds width x scale (planar), 1 x scale (curvy)
  float height = 0.f;
  DetectedImageType type = DetectedImageType::kUnspecified;
  CurvyGeometryOut curvy_geometry;
  // targetSpace (debug): TrackedImage status (1 NOT_FOUND, 2 LOCATED, 3 TRACKED), lastSeen,
  // firstSeen, scaleGuess and the camera pose in the target view (TrackedImage::pose).
  int32_t status = 0;
  int32_t last_seen = -1;
  int32_t first_seen = 0;
  float scale = 1.f;
  HMatrix4f target_view_pose;
  // FULL-6 tracking-path debug (docs/design/image-target.md section 7.7). roi_feats: the local
  // tracker used this frame's IMAGE_TARGET ROI feats. local_inliers / global_inliers: non-zero
  // entries of the public DetectionImageTracker::inliers / globalInliers (the local / global match
  // pose solve of this frame). Meaningful only while c8::ImageTrackingState::AGGRESSIVELY_FREE_MEMORY
  // is false (default true: the engine clears that scratch right after tracking, both read 0).
  bool roi_feats = false;
  int32_t local_inliers = 0;
  int32_t global_inliers = 0;
};

// DetectedImage (reality.capnp:356-374) after 733 (egomotion with the response's initial
// transform) and 704 (axes, mirror, recenterAndScale), as reality.detectedImages delivers it.
struct DetectedImageOut {
  std::string name;
  int64_t id = -1;
  Vec3f position;   // place
  QuatWxyz rotation;
  float width_in_meters = 0.f;
  float height_in_meters = 0.f;
  DetectedImageTrackingStatus tracking_status = DetectedImageTrackingStatus::kUnspecified;
  DetectedImageType type = DetectedImageType::kUnspecified;
  CurvyGeometryOut curvy_geometry;
};

// FULL-6: one ROI the pyramid producer must draw into the next staged frame (the GlRealityFrame
// nextRois_ after processPipelineData: the located targets' ROIs, then HIRES_SCAN fillers). The
// numbering is c8::ImageRoi::Source.
enum class PyramidRoiSource : int32_t {
  kImageTarget = 2,
  kHiresScan = 3,
  kCurvyImageTarget = 6,
};

// FULL-9 (US-014): the extra uniforms of a CURVY_IMAGE_TARGET slot draw (gr8-stage1.frag
// getCurvyLiftedUv), computed from the ROI and the slot exactly like the host code of
// gl-reality-frame.cc:397-433 (the CPU draw uses the same function). `is_curvy` is false for the
// other sources; then the remaining fields are 0 and `isCurvy` = 0 (the planar draw). The matrices
// are 16 floats in c8::HMatrix storage order (column-major: gl.uniformMatrix4fv(loc, false, m)).
// For a curvy ROI `warp` is the identity (the vertex `mvp`), `rotation` replaces the slot's
// rotation uniform (90 for a landscape target, else 0) and `curvy_post_rotation` is the slot's
// original rotation.
struct PyramidRoiCurvy {
  bool is_curvy = false;
  int32_t rotation = 0;
  int32_t curvy_post_rotation = 0;
  float target_dims[2] = {0, 0};
  float roi_dims[2] = {0, 0};
  float search_dims[2] = {0, 0};
  float intrinsics[16] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  float global_pose_inverse[16] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  float scales[2] = {0, 0};
  float shift = 0.f;
  float radius = 0.f;
  float radius_bottom = 0.f;
};

struct PyramidRoiRequest {
  int32_t slot = 0;  // index into PyramidLayout::roi_slots; list order = slot order
  PyramidRoiSource source = PyramidRoiSource::kImageTarget;
  int32_t face_id = -1;
  std::string name;
  // The `mvp` uniform of gr8-rotate.vert exactly as the original uploads it
  // (glUniformMatrix4fv(mvp, GL_FALSE, roi.warp.data())): 16 floats in c8::HMatrix storage order,
  // which is column-major, i.e. directly usable with gl.uniformMatrix4fv(loc, false, warp).
  float warp[16] = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
  PyramidRoiCurvy curvy;  // FULL-9
};

// imagefound / imageupdated / imagelost (PUB:reality/app/xr/js/src/tracking-controller.ts:731-760).
enum class ImageTargetEventKind : int32_t {
  kFound = 0,
  kUpdated = 1,
  kLost = 2,
};

struct ImageTargetEvent {
  ImageTargetEventKind kind = ImageTargetEventKind::kFound;
  DetectedImageOut image;
};

}  // namespace slam_refact

#endif  // SLAM_REFACT_IMAGE_TARGETS_H_

// Structures exchanged between engine modules, defined in exactly one place
// (architecture.md §3.1, D-5, D-6, D-7, D-14). Spec byte offsets (wasm32) are kept as comments
// only; memory layout does not have to match the original, field meaning does.
//
// FROZEN CONTRACT (WP0b). Changes follow slam_refact/docs/design/contract-changes.md.

#ifndef SLAM_REFACT_COMMON_TRACKING_TYPES_H_
#define SLAM_REFACT_COMMON_TRACKING_TYPES_H_

#include <cstdint>
#include <map>
#include <vector>

#include "c8/geometry/pixel-pinhole-camera-model.h"
#include "c8/hmatrix.h"
#include "c8/hpoint.h"
#include "c8/hvector.h"
#include "c8/quaternion.h"
#include "reality/engine/features/frame-point.h"
#include "reality/engine/xr-id.h"
#include "slam_refact/sensor_types.h"

namespace slam_refact {

namespace map {

// Point type of a world map point (WorldPoint +0x40; S2 §2.3, S3 §2.5).
enum class PointSource : int32_t {
  kTriangulated = 1,
  kGround = 2,
  kSky = 3,
  kDepth = 4,
  kCloud = 5,
};

// WorldPoint +0x18 (S2 §2.3, S3 §2.5).
enum class PointState : int32_t {
  kNew = 0,
  kTenured = 1,
  kDead = 2,
};

// World map handles are the public c8 XrId types (SOURCE PUB:reality/engine/xr-id.h, S3 §2.5):
// {index (= pool slot), id}; id 0 marks an empty slot. The vendored FramePoint/FrameWithPoints
// store exactly these types, so no conversion exists at the vendor boundary.
// Ordering is the public XrId::operator<, i.e. (id, index) lexicographic. This is what std::map /
// std::set of these ids iterate in. (architecture.md §3.3 states "(slot, id)"; the public source
// orders by id first. Recorded as a WP0b deviation in CONTRACTS.md.)
using PointId = c8::WorldMapPointId;
using KeyframeId = c8::WorldMapKeyframeId;

}  // namespace map

// Post-647 c8::TrackingSensorEvent. One type for the public API and the engine (D-14): there is
// no conversion path from a raw (capnp-numbered) event into this type except func 647.
using TrackingSensorEvent = ImuEvent;

// Original TrackingSensorFrame (S4 §2.2) restricted to the fields world SLAM reads. The pyramid
// is passed to VioTracker::Track separately. srcY, lat/lon and debugData are not used in world
// SLAM.
struct TrackingSensorFrame {
  // +0 deviceModel (DeviceInfos::DeviceModel). Selects the MSE method-5 fixed lever arm (1196);
  // unused by the default method 1 (WP4-CC-1, kContractVersion 2).
  int32_t device_model = 0;
  int64_t time_ns = 0;                  // +8 timeNanos
  bool is_ios = false;                  // +4
  c8_PixelPinholeCameraModel intrinsic; // +16 processing (level-0) intrinsics
  c8::Quaternion device_pose;           // +104 uncorrected; needs xrRotationFromDeviceRotation
  std::vector<TrackingSensorEvent> sensor_events;  // +144 sorted, post-647
  // ACC-5 (accuracy mode only): metres per internal map unit, 0 = none.
  double metric_scale_hint = 0.0;
};

// Per-frame tracker input (S4 §2.5). Port of the public struct
// PUB:reality/engine/features/tracker-input.h:8-20 (framePoints, roiPoints, pose, timeNanos),
// which is not part of the vendored closure; field names follow engine style. The closed field
// at +916 (UNVERIFIED) is not modeled.
struct TrackerInput {
  TrackerInput(c8_PixelPinholeCameraModel intrinsic, c8::Quaternion xr_device_pose,
               int64_t time_nanos = 0)
      : frame_points(intrinsic), pose(xr_device_pose), time_ns(time_nanos) {
    frame_points.setXRDevicePose(pose);  // tracker-input.h:14-16
  }

  c8::FrameWithPoints frame_points;            // +0
  std::vector<c8::FrameWithPoints> roi_points; // +920 (empty in world SLAM, S1 §2.3)
  c8::Quaternion pose;                         // +932 xr device rotation
  int64_t time_ns = 0;                         // +952
};

// Pose history slot, MotionPrediction input and MapTrackerOutput+0xd8 copy (S4 §2.4, D-6).
struct FrameState {
  c8::HMatrix pose = c8::HMatrixGen::i();  // +0 world_from_camera, internal XR coordinates
  c8::HVector3 velocity{0.f, 0.f, 0.f};    // +132
  c8::HVector3 acceleration{0.f, 0.f, 0.f};  // +148
  int64_t time_ns = 0;                     // +168
  c8::Quaternion xr_device_rotation;       // +176 (x, y, z, w in memory)
  std::vector<TrackingSensorEvent> sensor_events;  // +192
};

// MapTracker::Track output (926 param5, VioTracker+6768; S2 §2.2, S3 §2.7, D-5).
// MapBuilder reads world_matches from TrackerInput::frame_points.worldPointIndices().size() (D-7);
// there is no field for it here or in TrackerInput. The original per-frame path never fills that
// set (WP2-PRE §6, INFERRED; runtime UNVERIFIED), so engine code must not add setWorldPoint calls
// the original does not make.
struct MapTrackerOutput {
  c8::HMatrix pose = c8::HMatrixGen::i();  // +0x00 final pose_
  int32_t state = 0;                       // +0x84 0 = LOST, 1 = TRACKING
  bool frame_tracker_ok = false;           // +0x88 (S3 "trk.valid", D-5)
  int64_t consecutive_frame_tracker_ok = 0;  // +0x90
  std::vector<uint32_t> ground_idx;        // +0x98 new point candidates (= ground candidates)
  std::vector<uint32_t> sky_idx;           // +0xa4
  std::vector<uint32_t> depth_idx;         // +0xb0
  std::vector<map::PointId> visible_ids;   // +0xbc visible map points of this frame
  std::vector<uint8_t> visible_matched;    // +0xc8 same order: matched flag
  FrameState predicted;                    // +0xd8 copy of the prediction input (incl. events)
  std::map<uint32_t, map::PointId> tracked;  // +0x1a8 frame point index -> map point
  // Depth-keyframe relocalization (path A). Never set in world SLAM (architecture.md §1.2 M2).
  bool depth_kf_reloc_success = false;     // +0x1b4
  map::KeyframeId depth_kf_reloc_id;       // +0x1b8
  bool depth_kf_reloc_latch = false;       // +0x1c0
  bool lost_reloc_attempted = false;       // +0x1c5 (meaning INFERRED)
};

// MapBuilder::Update output (vio+6600; S3 §2.6, S4 §3.5 / 664).
struct MapBuilderOutput {
  bool map_reset = false;                  // +0x00 set by 653
  bool keyframe_created = false;           // +0x01 664 -> map_initialized_pending_
  std::vector<map::PointId> new_points;    // +0x04 ground/sky points created this frame
  map::KeyframeId reloc_kf_id;             // +0x10 (VPS/depth reseed only)
  // +0x18. Always false in world SLAM; MapTracker::Track aborts if the resulting
  // reloc_kf_pending_ is ever set (architecture.md §1.2 M2).
  bool reloc_kf_request = false;
  bool world_transform_updated = false;    // +0x1c (S3: loop-close success; S4: +6628)
  // +0x20. C = P_old^-1 * P_new of the newest keyframe (660); 664 right-multiplies it onto pose_:
  // pose_ = pose_ * C (spec/12-loop-close.md §5.2; US025-CC-1).
  c8::HMatrix world_correction = c8::HMatrixGen::i();
  // Accuracy mode (ACC-1; never set by the original paths): the local bundle adjustment moved the
  // newest keyframe. pose_correction (new * old^-1 of that keyframe, world_from_camera) is
  // left-multiplied onto the tracker pose without resetting tracking state.
  bool pose_correction_applied = false;
  c8::HMatrix pose_correction = c8::HMatrixGen::i();
};

// relocalize (1274) / computeFramePoseUsingRobustP2P (1265) result (S2 §3.2).
struct RelocResult {
  bool success = false;
  c8::HMatrix pose = c8::HMatrixGen::i();
  float confidence = 0.f;
  std::vector<map::PointId> inlier_ids;
  std::vector<c8::HPoint3> inlier_world_points;
};

}  // namespace slam_refact

#endif  // SLAM_REFACT_COMMON_TRACKING_TYPES_H_

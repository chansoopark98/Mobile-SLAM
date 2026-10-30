// Frontend module public header (architecture.md §3.2; S1 §2-§5, §7).
//
// FROZEN CONTRACT (WP0b): every declaration in this file. Changes follow
// slam_refact/docs/design/contract-changes.md. The bodies of the `private:` sections are owned
// by WP1 and may change without a contract change (CONTRACTS.md "private sections").

#ifndef SLAM_REFACT_FRONTEND_FRONTEND_H_
#define SLAM_REFACT_FRONTEND_FRONTEND_H_

#include <array>
#include <cstdint>
#include <string_view>
#include <vector>

#include "c8/pixels/gr8-pyramid.h"
#include "c8/quaternion.h"
#include "reality/engine/features/detection-config.h"
#include "reality/engine/features/frame-point.h"
#include "reality/engine/features/gr8gl.h"  // FeatureFrontend private member (WP1)
#include "slam_refact/image_types.h"
#include "slam_refact/sensor_types.h"
#include "slam_refact/types.h"

namespace slam_refact::frontend {

// ---- Sensor intake: sensors.ts + onProcessGpu + prepareTrackingSensorFrame (S1 §3.1, §3.6) ----

// Result of one setEventQueue: post-647 events (sorted) and getQuaternion(alpha, beta, gamma).
struct StagedSensors {
  std::vector<ImuEvent> events;
  QuatWxyz q;
};

// One per GlueRunner. Reproduces the JS-side state (event queue, last orientation).
class SensorIntake {
 public:
  // is_ios = upper(manufacturer) == "APPLE" (S1 U10, INFERRED).
  explicit SensorIntake(bool is_ios);

  // 5e9 ns clear rule; LINEAR_ACCEL only when acceleration is present.
  void OnDeviceMotion(const DeviceMotionSample& s);
  void OnDeviceOrientation(const DeviceOrientationSample& s);

  // setEventQueue: empties the raw queue and runs PrepareTrackingSensorEvents (func 647).
  // This is the only caller of func 647 in the engine (D-14).
  StagedSensors TakeForStage();

 private:
  // Owned by WP1. JS state of sensors.ts (eventQueue) and tracking-controller.ts
  // (lastDevicePose_, initialised to 0/0/0 at :261).
  bool is_ios_;
  std::vector<RawSensorEvent> queue_;
  double alpha_deg_ = 0.0;
  double beta_deg_ = 0.0;
  double gamma_deg_ = 0.0;
};

// quaternion.ts:20-40 (ZXY).
c8::Quaternion GetQuaternion(double alpha_deg, double beta_deg, double gamma_deg);

// func 647. Accepts capnp-numbered RawSensorEvent only; there is deliberately no overload or
// conversion taking ImuEvent, so applying 647 twice does not compile (D-14).
std::vector<ImuEvent> PrepareTrackingSensorEvents(const std::vector<RawSensorEvent>& queue,
                                                  bool is_ios);

// ---- K rule (S1 §5.12) ----

// Device table (111 entries) + fallback 506.4104.
PinholeIntrinsics FullIntrinsicsForDevice(std::string_view manufacturer, std::string_view model);
PinholeIntrinsics CropAndScale(const PinholeIntrinsics& k, int w, int h);

struct IntrinsicsSet {
  PinholeIntrinsics full;
  PinholeIntrinsics capture;
  PinholeIntrinsics processing;  // = level 0 (S1 U7, INFERRED)
};

// ---- Pyramid (S1 §2.2, §5.1-5.4) ----

struct LevelLayout {
  int r = 0;
  int c = 0;
  int w = 0;
  int h = 0;
  bool rotated = false;
};

struct PyramidLayout {
  int texture_size = 0;
  int rows = 0;
  std::array<LevelLayout, 4> levels{};
  std::array<LevelLayout, 4> roi_slots{};
  int filtered_w = 0;
  int filtered_h = 0;
  bool rotate_input = false;
};

// func 459.
PyramidLayout ComputePyramidLayout(int camera_w, int camera_h, int pyramid_size = 1024);

struct ShaderParams {
  float gain = 1.f;
  float score_thresh = 4.f;
  float nms_tolerance = 1.f;
  bool skip_subpixel = false;
};

// RGBA atlas in GPU readback format: row 0 = GL row 0, row bytes = 4 * size.
// Channels: R gray, G 7x7 Gaussian, B NMES score, A subpixel (S1 §2.2).
struct RgbaAtlas {
  int size = 0;
  int rows = 0;
  std::vector<uint8_t> rgba;
};

// CPU port of the GPU pyramid shader (S1 §5.1).
class CpuGr8Pyramid {
 public:
  CpuGr8Pyramid(const PyramidLayout& layout, ShaderParams params, ShaderNumericModel model);

  void Build(const RgbaImageView& camera, int rotation_deg, RgbaAtlas* out) const;
  // H3 entry point.
  void RunStage2And3(RgbaAtlas* atlas_with_r) const;
  // H4 entry point.
  void RunFromLevel0(RgbaAtlas* atlas_with_l0) const;

 private:
  // Owned by WP1.
  PyramidLayout layout_;
  ShaderParams params_;
  ShaderNumericModel model_;
  // Storage of the intermediate textures, kept between calls so a frame does not reallocate them
  // (S2 review LOW-7). Only the capacity is reused: every texture is zero-filled to its size
  // before a stage writes it, so the output never depends on an earlier call. Not thread-safe
  // (the engine is single-threaded, architecture.md §4.6).
  mutable std::vector<uint8_t> filtered_store_;
  mutable std::vector<uint8_t> dest1_store_;
  mutable std::vector<uint8_t> blur_store_;
  mutable std::vector<uint8_t> dest2_store_;
};

// ---- Detection (S1 §3.5, §3.7) ----

// Counts of the last Detect call (TrackDebug num_keypoints / num_orb / num_gorb, §3.7).
struct DetectStats {
  int num_keypoints = 0;
  int num_orb = 0;
  int num_gorb = 0;
};

// Owns c8::Gr8Gl::create(1.44, 21, 2).
class FeatureFrontend {
 public:
  FeatureFrontend();
  ~FeatureFrontend();

  void Detect(const c8::Gr8Pyramid& pyr, const c8::DetectionConfig& cfg,
              c8::FrameWithPoints* frame);

  // FULL-3: the ROI part of FeatureDetector::detectFeatures (PUB:reality/engine/features/
  // feature-detector.cc:82-107; 784 scope "gr8gl-rois"): one FrameWithPoints per pyramid ROI with
  // the frame's intrinsic and xr pose, the ROI set, points in ROI pixels, no excluded edge.
  // `frame` is the frame of the preceding Detect call.
  void DetectRois(const c8::Gr8Pyramid& pyr, const c8::DetectionConfig& cfg,
                  const c8::FrameWithPoints& frame, std::vector<c8::FrameWithPoints>* rois);

  const DetectStats& last_stats() const;

 private:
  // Owned by WP1.
  c8::Gr8Gl gr8gl_;
  DetectStats last_stats_;
};

// 500 keypoints, allGorb, ORB gravity portion 192 (the 6208 flag is 0 in world SLAM, R17).
c8::DetectionConfig WorldSlamDetectionConfig();

}  // namespace slam_refact::frontend

#endif  // SLAM_REFACT_FRONTEND_FRONTEND_H_

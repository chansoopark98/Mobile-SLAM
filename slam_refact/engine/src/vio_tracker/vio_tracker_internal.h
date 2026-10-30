// VioTracker state and the pure pieces of 784 (WP4-private, not frozen): MotionPrediction with
// explicit settings, yaw 519, yaw offset, metric transform, status composition.

#ifndef SLAM_REFACT_VIO_TRACKER_VIO_TRACKER_INTERNAL_H_
#define SLAM_REFACT_VIO_TRACKER_VIO_TRACKER_INTERNAL_H_

#include <cstdint>
#include <memory>
#include <vector>

#include "c8/hmatrix.h"
#include "c8/quaternion.h"
#include "common/tracking_types.h"
#include "image_target/image_world_tracker.h"
#include "metric_scale/metric_scale_estimator.h"
#include "vio_tracker/tracker_modules.h"
#include "vio_tracker/vi_prediction.h"
#include "vio_tracker/vio_tracker.h"

namespace slam_refact::vio_tracker {

// S4 §3.2 with explicit MotionPrediction settings (PredictMotion reads them once per process).
// q_out / p_out receive the predicted rotation and position before cameraMotion (521).
FrameState PredictMotionWithParams(const FrameState& prev, const FrameRecord& now,
                                   float gravity_corrected_interpolation, float above_horizon_drag,
                                   c8::Quaternion* q_out = nullptr);

// S4 §5.5 (D:85120-85200): Y = R_y(-yaw(fromHMatrix(P))), stored as R(q)^T with inverse R(q).
c8::HMatrix YawOffsetFromPose(const c8::HMatrix& p);

// S4 §5.6: M = T(-(sYp)_x, s - 1, -(sYp)_z) * S(s) * Y.
c8::HMatrix MetricTransform(float s, const c8::HMatrix& y, const c8::HMatrix& p);

// S4 §3.9: cameraMotion(trsTranslation(M P), trsRotation(M P)).
c8::HMatrix MetricOutput(const c8::HMatrix& m, const c8::HMatrix& p);

// INITIALIZING flag pairs (S4 §6.2): set / acknowledged.
struct StatusFlags {
  bool scale_pending = false;    // +6376 estimateScale
  bool scale_ready = false;      // +6377 = scale latched
  bool vps_map_pending = false;  // +6378 (vpsMode server; never set in world SLAM)
  bool map_built = false;        // +6379
  bool wait_first_ok = false;    // +6380 (vpsMode device*; never set in world SLAM)
  bool got_first_ok = false;     // +6381
};

// writeStatus (D:85100-85119, D:85912-85932).
StatusOut ComposeStatus(const StatusFlags& flags, int loss_state);

struct VioTracker::Impl {
  Impl(bool estimate_scale_in, bool disable_vio_in, std::unique_ptr<TrackerModules> modules_in);

  std::unique_ptr<TrackerModules> modules;
  bool disable_vio;                                    // +4
  bool estimate_scale;                                 // +5
  metric_scale::MetricScaleEstimator mse;              // +8
  bool scale_latched = false;                          // +2840
  float latched_scale = 0.f;                           // +2844
  c8::HMatrix metric_transform = c8::HMatrixGen::i();  // +2848
  int64_t first_frame_time_ns = 0;                     // +3008 (VPS timer; kept for fidelity)
  c8::HMatrix yaw_offset = c8::HMatrixGen::i();        // +6212
  bool yaw_offset_initialized = false;                 // +6344
  LossStateMachine loss;                               // +6352..+6372
  StatusFlags flags;                                   // +6376..+6381
  std::unique_ptr<TrackerInput> input;                 // +6456
  c8::HMatrix tam_extrinsic = c8::HMatrixGen::i();     // +6464
  MapBuilderOutput mb_out;                             // +6600 (persists across frames)
  MapTrackerOutput mt_out;                             // +6768 (persists across frames)
  FrameState hist[2];                                  // +7296 prev, +7504 cur
  FrameRecord now;                                     // +7712
  FrameState prediction;                               // g+480 of the last Track
  ViPredictor vi;                                      // ACC-5 (accuracy mode only)
  int64_t vi_frames = 0;
  c8::DetectionImageTracker image_tracker;             // +7248 (FULL-3, 938)
  std::vector<c8::LocatedImage> located;               // +7280 locatedImages_ (938 output)
  std::vector<c8::LocatedImage> located_out;           // after applyToLocatedImages
  std::vector<LocatedImageDebug> located_debug;        // FULL-6: parallel to located
  // US-015 (FULL-12, on by default since WT-DEFAULT-ON): 994 behind the vendored c8::ImageWorldTracker hook.
  image_target::ImageWorldTrackerHook image_world_tracker;
};

// Test seams.
std::unique_ptr<VioTracker> NewVioTrackerWithModules(bool estimate_scale, bool disable_vio,
                                                     std::unique_ptr<TrackerModules> modules);

struct VioTrackerAccess {
  static VioTracker::Impl& state(VioTracker& t) { return *t.impl_; }
};

struct LossStateMachineAccess {
  static void Set(LossStateMachine* m, int state, int lost_frames, int frames, int64_t enter_ns) {
    m->state_ = state;
    m->lost_frames_ = lost_frames;
    m->frames_ = frames;
    m->state_enter_ns_ = enter_ns;
  }
  static int64_t enter_ns(const LossStateMachine& m) { return m.state_enter_ns_; }
};

}  // namespace slam_refact::vio_tracker

#endif  // SLAM_REFACT_VIO_TRACKER_VIO_TRACKER_INTERNAL_H_

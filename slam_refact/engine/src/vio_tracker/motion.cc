// Pure parts of 784: MotionPrediction (S4 §3.2), pose history (§3.4), loss state machine and
// status (§3.3), yaw offset (§5.5), metric transform and output (§3.9, §5.6).

#include <algorithm>
#include <cmath>

#include "c8/geometry/egomotion.h"
#include "c8/hpoint.h"
#include "c8/hvector.h"
#include "config/settings.h"
#include "map_tracker/map_tracker.h"
#include "vio_tracker/vio_tracker.h"
#include "vio_tracker/vio_tracker_internal.h"

namespace slam_refact::vio_tracker {

FrameState PredictMotionWithParams(const FrameState& prev, const FrameRecord& now,
                                   float gravity_corrected_interpolation, float above_horizon_drag,
                                   c8::Quaternion* q_out) {
  const c8::Quaternion q_cam = c8::Quaternion::fromHMatrix(prev.pose);              // 520
  // f784 wat (MotionPrediction): the previous device rotation is normalized once, as part of the
  // conjugate (= Quaternion::inverse()), then multiplied by the current one; there is no separate
  // normalize() before it (it would re-round an already normalized quaternion).
  const c8::Quaternion delta = prev.xr_device_rotation.inverse().times(now.xr_device_rotation);
  const c8::Quaternion q_pred = q_cam.times(delta);
  const float f_y = now.xr_device_rotation.toRotationMat()(1, 2);  // device forward, world y
  const float above = (3.0f * f_y < 0.0f) ? 0.0f : std::min(3.0f * f_y, 1.0f);
  float dt = static_cast<float>(static_cast<double>(now.time_ns - prev.time_ns) * 1e-9);
  if (dt == 0.0f) {
    dt = 1.0f;
  }
  const float k = 1.0f - above * above_horizon_drag;
  const float vx = k * prev.velocity.x();
  const float vy = k * prev.velocity.y();
  const float vz = k * prev.velocity.z();
  const c8::HMatrix& pp = prev.pose;
  const c8::HPoint3 p(pp(0, 3) + dt * vx, pp(1, 3) + dt * vy, pp(2, 3) + dt * vz);
  const float yaw_pred = map_tracker::YawOf(q_pred);  // 519 yawOf (S2 §5.5, G519)
  const float yaw_dev = map_tracker::YawOf(now.xr_device_rotation);
  const float h = (yaw_pred - yaw_dev) * 0.5f;
  const c8::Quaternion q_gc =
      c8::Quaternion(std::cos(h), 0.0f, std::sin(h), 0.0f).times(now.xr_device_rotation);
  const c8::Quaternion q = q_gc.interpolate(q_pred, gravity_corrected_interpolation);
  if (q_out != nullptr) {
    *q_out = q;
  }
  FrameState out;
  out.pose = c8::cameraMotion(p, q);  // 521
  out.velocity = c8::HVector3(vx, vy, vz);
  out.acceleration = c8::HVector3(0.0f, 0.0f, 0.0f);
  out.time_ns = now.time_ns;
  out.xr_device_rotation = now.xr_device_rotation;
  out.sensor_events = now.sensor_events;
  return out;
}

FrameState PredictMotion(const FrameState& prev, const FrameRecord& now) {
  // 784 caches the two MotionPrediction keys once per process (flag 611892, S4 §4).
  static const config::MotionPredictionSettings kSettings = config::MotionPredictionSettings::Get();
  return PredictMotionWithParams(prev, now, kSettings.gravity_corrected_interpolation,
                                 kSettings.above_horizon_drag);
}

void UpdateHistory(FrameState hist[2], const c8::HMatrix& p, const FrameRecord& now,
                   bool world_transform_updated) {
  FrameState next;
  next.pose = p;
  next.time_ns = now.time_ns;
  next.xr_device_rotation = now.xr_device_rotation;
  next.sensor_events = now.sensor_events;
  if (world_transform_updated) {  // 665: velocity / acceleration of hist[1] kept
    next.velocity = hist[1].velocity;
    next.acceleration = hist[1].acceleration;
  } else {
    const float dt = static_cast<float>(static_cast<double>(now.time_ns - hist[1].time_ns) * 1e-9);
    const float inv = dt == 0.0f ? 1.0f : 1.0f / dt;
    const c8::HMatrix& q = hist[1].pose;
    const float vx = inv * (p(0, 3) - q(0, 3));
    const float vy = inv * (p(1, 3) - q(1, 3));
    const float vz = inv * (p(2, 3) - q(2, 3));
    next.velocity = c8::HVector3(vx, vy, vz);
    next.acceleration = c8::HVector3(inv * (vx - hist[1].velocity.x()), inv * (vy - hist[1].velocity.y()),
                                     inv * (vz - hist[1].velocity.z()));
  }
  hist[0] = std::move(hist[1]);
  hist[1] = std::move(next);
}

c8::HMatrix YawOffsetFromPose(const c8::HMatrix& p) {
  const float yaw = map_tracker::YawOf(c8::Quaternion::fromHMatrix(p));
  const float c = std::cos(yaw);  // 462
  const float s = std::sin(yaw);  // 461
  const c8::HMatrix ry({c, 0.0f, s, 0.0f}, {0.0f, 1.0f, 0.0f, 0.0f}, {-s, 0.0f, c, 0.0f},
                       {0.0f, 0.0f, 0.0f, 1.0f}, {c, 0.0f, -s, 0.0f}, {0.0f, 1.0f, 0.0f, 0.0f},
                       {s, 0.0f, c, 0.0f}, {0.0f, 0.0f, 0.0f, 1.0f});
  // Matrix R(q)^T, inverse R(q): exactly toRotationMat().inv().
  return c8::Quaternion::fromHMatrix(ry).toRotationMat().inv();
}

c8::HMatrix MetricTransform(float s, const c8::HMatrix& y, const c8::HMatrix& p) {
  const c8::HMatrix a = c8::HMatrixGen::scale(s) * y;
  const c8::HMatrix ap = a * p;  // translation = s Y p
  return c8::HMatrixGen::translation(-ap(0, 3), s - 1.0f, -ap(2, 3)) * a;
}

c8::HMatrix MetricOutput(const c8::HMatrix& m, const c8::HMatrix& p) {
  const c8::HMatrix g = m * p;
  return c8::cameraMotion(c8::trsTranslation(g), c8::trsRotation(g));  // 945 + 521
}

StatusOut ComposeStatus(const StatusFlags& f, int loss_state) {
  StatusOut out;
  if ((f.scale_pending && !f.scale_ready) || (f.vps_map_pending && !f.map_built) ||
      (f.wait_first_ok && !f.got_first_ok)) {
    out.status = TrackingStatus::kLimited;
    out.reason = TrackingReason::kInitializing;
    return out;
  }
  out.reason = TrackingReason::kUnspecified;
  out.status = (loss_state == 2 || loss_state == 3) ? TrackingStatus::kLimited : TrackingStatus::kNormal;
  return out;
}

// ---- LossStateMachine (D:85056-85098) ----

void LossStateMachine::Update(bool ok, int64_t now_ns) {
  last_ok_ = ok;
  frames_ += 1;
  lost_frames_ += ok ? 0 : 1;
  int next = state_;
  bool change = false;
  const auto recover = [this]() {
    return (static_cast<float>(lost_frames_) / static_cast<float>(frames_) < 0.2f) ? 0 : 2;
  };
  switch (state_) {
    case 0:
      if (lost_frames_ != 0) {
        next = 1;
        change = true;
      }
      break;
    case 1:
      if (now_ns - state_enter_ns_ >= 2000000000LL) {
        next = recover();
        change = true;
      }
      break;
    case 2:
      if (lost_frames_ != frames_) {
        next = 3;
        change = true;
      }
      break;
    case 3:
      if (now_ns - state_enter_ns_ >= 2000000000LL) {
        next = recover();
        change = true;
      }
      break;
    default:  // unreachable in practice; same rule as state 0
      if (lost_frames_ != 0) {
        next = 1;
        change = true;
      }
      break;
  }
  if (change) {
    state_ = next;
    lost_frames_ = 0;
    frames_ = 0;
    state_enter_ns_ = now_ns;
  }
}

TrackingStatus LossStateMachine::status() const {
  return (state_ == 2 || state_ == 3) ? TrackingStatus::kLimited : TrackingStatus::kNormal;
}

int LossStateMachine::state() const { return state_; }
int LossStateMachine::lost_frames() const { return lost_frames_; }
int LossStateMachine::frames() const { return frames_; }
bool LossStateMachine::last_ok() const { return last_ok_; }

}  // namespace slam_refact::vio_tracker

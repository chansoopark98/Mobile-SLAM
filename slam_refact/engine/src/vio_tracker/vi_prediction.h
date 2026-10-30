// Accuracy mode ACC-5: tracking-stage visual-inertial coupling.
//
// The original MotionPrediction (S4 §3.2) predicts the next position from the finite-difference
// velocity of the last two tracker poses (constant velocity, no accelerometer). The MapTracker's
// PnP then holds its pose prior (Bundle RotationTarget / ZeroTarget) at that prediction, so every
// visual jitter of the previous poses is fed forward. ViPredictor replaces the predicted position:
//
//   propagation  for each accelerometer event of the frame (post-647, internal camera axes
//                x right, y up, z forward), a_world = R_world_camera(t) * a - (0, g, 0), with
//                R(t) = R_pose(prev) * slerp(I, q_dev(prev)^-1 q_dev(now), u) (the tracker keeps
//                the device tilt, so relative device rotation is the camera rotation change);
//                position and velocity are integrated in metres (piecewise-constant acceleration)
//   prediction   p_pred = p_prev + delta_p_metres / metres_per_unit (rotation keeps the original)
//   update       after tracking, v = v_prop + gain * (v_visual - v_prop), v_visual =
//                metres_per_unit * (p_now - p_prev) / dt (complementary velocity fusion: the IMU
//                carries the high-frequency motion, the visual track bounds the drift)
// Metres per internal unit come from the metric scale tracker (GlueRunner) through
// TrackingSensorFrame::metric_scale_hint. Discontinuities (map reset, world/pose corrections, gaps
// over 0.3 s, missing hint) restart the filter from the visual velocity.

#ifndef SLAM_REFACT_VIO_TRACKER_VI_PREDICTION_H_
#define SLAM_REFACT_VIO_TRACKER_VI_PREDICTION_H_

#include <array>
#include <cstddef>
#include <cstdint>
#include <vector>

#include "c8/hmatrix.h"
#include "c8/quaternion.h"
#include "common/tracking_types.h"

namespace slam_refact::vio_tracker {

class ViPredictor {
 public:
  void Reset();

  // Replaces pred->pose's translation with the IMU-propagated position when the filter is
  // initialised; returns whether it did. prev = the previous history slot (pose, time, device
  // rotation), now_* = this frame.
  bool Predict(const FrameState& prev, int64_t now_ns, const c8::Quaternion& now_device_rotation,
               const std::vector<TrackingSensorEvent>& events, double metres_per_unit,
               FrameState* pred);

  // After tracking: fuse the tracked pose's visual velocity. discontinuity restarts the filter.
  void Update(const FrameState& prev, const c8::HMatrix& pose_now, int64_t now_ns,
              double metres_per_unit, double gain, bool discontinuity);

  bool initialized() const { return initialized_; }
  // Median angle (deg) between the rotated specific force and world up over the last frames
  // (diagnostic of the axis convention; small when consistent).
  double gravity_angle_deg() const;

 private:
  bool initialized_ = false;
  std::array<double, 3> v_{0.0, 0.0, 0.0};        // fused velocity, metres/s, internal world
  std::array<double, 3> v_prop_{0.0, 0.0, 0.0};   // propagated velocity of the last Predict
  bool have_prop_ = false;
  std::array<double, 3> last_accel_{0.0, 0.0, 0.0};
  bool have_accel_ = false;
  std::vector<double> gravity_angles_;  // ring buffer, gravity_next_ = next slot
  size_t gravity_next_ = 0;
};

}  // namespace slam_refact::vio_tracker

#endif  // SLAM_REFACT_VIO_TRACKER_VI_PREDICTION_H_

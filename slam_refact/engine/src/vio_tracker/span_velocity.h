// P3 candidate R6 (AccuracyConfig::r6_span_velocity): velocity of the motion prediction estimated
// over a time span instead of the last frame pair.
//
// The original keeps hist[1].velocity = (p_now - p_prev) / (t_now - t_prev) of the last two tracker
// poses (S4 §3.4) and MotionPrediction extrapolates it (S4 §3.2). The tracker noise of a pose is
// fixed in position, so the velocity noise grows as 1 / (t_now - t_prev): an input pair delivered a
// few milliseconds apart (fixture S2, frame 287 -> 288, 3.3 ms) turns a normal pose jitter into a
// velocity several times the real one, and the next prediction step jumps.
//
// Estimator (not a clamp, nothing is dropped or limited): the reference for the newest pose is the
// earlier pose whose time separation is closest to the nominal frame interval, the median of the
// last intervals of the stream. Under uniform timing that is the previous pose, so the velocity is
// the original one bit for bit; an unusually short pair is replaced by the pair that spans one
// nominal interval; a long interval (gap) is already farther above nominal than any longer span, so
// it is used as it is. The velocity always divides by the real time between its two poses, and the
// prediction multiplies by the real time to the current frame (PredictMotion, unchanged). Poses
// from before a discontinuity (map reset, world transform, pose correction) are never used.

#ifndef SLAM_REFACT_VIO_TRACKER_SPAN_VELOCITY_H_
#define SLAM_REFACT_VIO_TRACKER_SPAN_VELOCITY_H_

#include <array>
#include <cstdint>

#include "c8/hvector.h"
#include "common/tracking_types.h"

namespace slam_refact::vio_tracker {

class SpanVelocity {
 public:
  // Intervals kept for the nominal one. A median of seven ignores up to three outliers; fewer than
  // kMinIntervals intervals cannot tell an outlier from the stream, so the original velocity stays.
  static constexpr int kIntervals = 7;
  static constexpr int kMinIntervals = 3;
  static constexpr int kPoses = kIntervals + 1;

  // Poses pushed before this call are not used for a velocity any more. The interval history is a
  // property of the camera stream and is kept.
  void Restart();
  // Newest tracker pose (position part) at time_ns; call once per frame.
  void Push(int64_t time_ns, float x, float y, float z);
  // Velocity of the newest pose over its reference pose; false when there is no reference or the
  // stream is too short for a nominal interval (the caller keeps the original velocity).
  bool Estimate(c8::HVector3* v) const;
  // Nominal interval in ns (0 until kMinIntervals intervals were seen).
  int64_t nominal_interval_ns() const;

 private:
  struct Entry {
    int64_t time_ns = 0;
    float p[3] = {0.f, 0.f, 0.f};
  };
  std::array<Entry, kPoses> poses_{};      // [0] newest
  int pose_count_ = 0;
  std::array<int64_t, kIntervals> intervals_{};  // [0] newest, only positive intervals
  int interval_count_ = 0;
  bool have_last_time_ = false;
  int64_t last_time_ns_ = 0;
};

// Run after UpdateHistory on the same hist: pushes hist[1] and, unless the world transform was
// updated (665 keeps the previous velocity), replaces hist[1].velocity and .acceleration by the
// span estimate. `restart` = map reset or world transform / pose correction on this frame.
void ApplySpanVelocity(SpanVelocity* span, FrameState hist[2], bool restart,
                       bool world_transform_updated);

}  // namespace slam_refact::vio_tracker

#endif  // SLAM_REFACT_VIO_TRACKER_SPAN_VELOCITY_H_

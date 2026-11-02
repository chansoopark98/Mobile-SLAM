#include "vio_tracker/span_velocity.h"

#include <algorithm>
#include <cmath>

namespace slam_refact::vio_tracker {

void SpanVelocity::Restart() { pose_count_ = 0; }

void SpanVelocity::Push(int64_t time_ns, float x, float y, float z) {
  if (have_last_time_ && time_ns > last_time_ns_) {
    std::copy_backward(intervals_.begin(), intervals_.end() - 1, intervals_.end());
    intervals_[0] = time_ns - last_time_ns_;
    interval_count_ = std::min(interval_count_ + 1, kIntervals);
  }
  have_last_time_ = true;
  last_time_ns_ = time_ns;
  std::copy_backward(poses_.begin(), poses_.end() - 1, poses_.end());
  Entry& e = poses_[0];
  e.time_ns = time_ns;
  e.p[0] = x;
  e.p[1] = y;
  e.p[2] = z;
  pose_count_ = std::min(pose_count_ + 1, kPoses);
}

int64_t SpanVelocity::nominal_interval_ns() const {
  if (interval_count_ < kMinIntervals) return 0;
  std::array<int64_t, kIntervals> sorted = intervals_;
  std::sort(sorted.begin(), sorted.begin() + interval_count_);
  return sorted[interval_count_ / 2];
}

bool SpanVelocity::Estimate(c8::HVector3* v) const {
  const int64_t nominal = nominal_interval_ns();
  if (nominal == 0 || pose_count_ < 2) return false;
  int best = -1;
  int64_t best_err = 0;
  for (int j = 1; j < pose_count_; ++j) {  // newest first: a tie keeps the shorter span
    const int64_t span = poses_[0].time_ns - poses_[j].time_ns;
    if (span <= 0) continue;
    const int64_t err = span > nominal ? span - nominal : nominal - span;
    if (best < 0 || err < best_err) {
      best = j;
      best_err = err;
    }
  }
  if (best < 0) return false;
  // Same expression as UpdateHistory (S4 §3.4) so that the previous pose gives the original value.
  const float dt =
      static_cast<float>(static_cast<double>(poses_[0].time_ns - poses_[best].time_ns) * 1e-9);
  const float inv = 1.0f / dt;
  *v = c8::HVector3(inv * (poses_[0].p[0] - poses_[best].p[0]),
                    inv * (poses_[0].p[1] - poses_[best].p[1]),
                    inv * (poses_[0].p[2] - poses_[best].p[2]));
  return true;
}

void ApplySpanVelocity(SpanVelocity* span, FrameState hist[2], bool restart,
                       bool world_transform_updated) {
  if (restart) span->Restart();
  const c8::HMatrix& p = hist[1].pose;
  span->Push(hist[1].time_ns, p(0, 3), p(1, 3), p(2, 3));
  if (world_transform_updated) return;
  c8::HVector3 v;
  if (!span->Estimate(&v)) return;
  const float dt = static_cast<float>(static_cast<double>(hist[1].time_ns - hist[0].time_ns) * 1e-9);
  const float inv = dt == 0.0f ? 1.0f : 1.0f / dt;
  hist[1].velocity = v;
  hist[1].acceleration =
      c8::HVector3(inv * (v.x() - hist[0].velocity.x()), inv * (v.y() - hist[0].velocity.y()),
                   inv * (v.z() - hist[0].velocity.z()));
}

}  // namespace slam_refact::vio_tracker

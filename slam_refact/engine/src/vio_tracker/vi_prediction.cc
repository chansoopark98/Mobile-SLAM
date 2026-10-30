#include "vio_tracker/vi_prediction.h"

#include <algorithm>
#include <cmath>

#include "c8/geometry/egomotion.h"
#include "c8/hpoint.h"

namespace slam_refact::vio_tracker {
namespace {

constexpr double kGravity = 9.80665;
constexpr double kMaxGapS = 0.3;
constexpr double kPi = 3.14159265358979323846;
constexpr size_t kGravityAngleWindow = 600;

std::array<double, 3> Rotate(const c8::HMatrix& r, const std::array<double, 3>& v) {
  const c8::HPoint3 p = r * c8::HPoint3(static_cast<float>(v[0]), static_cast<float>(v[1]),
                                        static_cast<float>(v[2]));
  return {p.x(), p.y(), p.z()};
}

}  // namespace

void ViPredictor::Reset() {
  initialized_ = false;
  have_prop_ = false;
  have_accel_ = false;
  v_ = {0.0, 0.0, 0.0};
  gravity_angles_.clear();
  gravity_next_ = 0;
}

double ViPredictor::gravity_angle_deg() const {
  if (gravity_angles_.empty()) return 0.0;
  std::vector<double> a = gravity_angles_;
  std::nth_element(a.begin(), a.begin() + static_cast<std::ptrdiff_t>(a.size() / 2), a.end());
  return a[a.size() / 2];
}

bool ViPredictor::Predict(const FrameState& prev, int64_t now_ns,
                          const c8::Quaternion& now_device_rotation,
                          const std::vector<TrackingSensorEvent>& events, double metres_per_unit,
                          FrameState* pred) {
  have_prop_ = false;
  const double dt_total = static_cast<double>(now_ns - prev.time_ns) * 1e-9;
  // Accelerometer samples of this frame (also kept when the filter is not initialised).
  std::vector<std::pair<double, std::array<double, 3>>> acc;
  for (const TrackingSensorEvent& e : events) {
    if (e.kind != TrackingSensorEvent::Kind::kAccelerometer) continue;
    acc.push_back({static_cast<double>(e.event_time_ns - prev.time_ns) * 1e-9,
                   {static_cast<double>(e.x), static_cast<double>(e.y), static_cast<double>(e.z)}});
  }
  if (!initialized_ || !(metres_per_unit > 0.0) || !(dt_total > 0.0) || dt_total > kMaxGapS ||
      (!have_accel_ && acc.empty())) {
    if (!acc.empty()) {
      last_accel_ = acc.back().second;
      have_accel_ = true;
    }
    return false;
  }
  const c8::HMatrix r_prev = c8::Quaternion::fromHMatrix(prev.pose).toRotationMat();
  const c8::Quaternion q_rel = prev.xr_device_rotation.inverse().times(now_device_rotation);
  const c8::Quaternion q_id;
  // Piecewise-constant acceleration: the last sample before the frame holds until the first new
  // one, each new sample until the next (sample times clamped to [0, dt]).
  std::array<double, 3> p{0.0, 0.0, 0.0};
  std::array<double, 3> v = v_;
  double t = 0.0;
  std::array<double, 3> a_cam = last_accel_;
  size_t k = 0;
  while (t < dt_total) {
    const double t_next =
        k < acc.size() ? std::clamp(acc[k].first, t, dt_total) : dt_total;
    const double h = t_next - t;
    if (h > 0.0) {
      const double u = std::clamp((t + 0.5 * h) / dt_total, 0.0, 1.0);
      const c8::HMatrix r_t = r_prev * q_id.interpolate(q_rel, static_cast<float>(u)).toRotationMat();
      const std::array<double, 3> f = Rotate(r_t, a_cam);
      const double fn = std::sqrt(f[0] * f[0] + f[1] * f[1] + f[2] * f[2]);
      if (fn > 1.0) {
        const double angle = std::acos(std::clamp(f[1] / fn, -1.0, 1.0)) * 180.0 / kPi;
        if (gravity_angles_.size() < kGravityAngleWindow) {
          gravity_angles_.push_back(angle);
        } else {
          gravity_angles_[gravity_next_] = angle;  // ring buffer of the last samples
        }
        gravity_next_ = (gravity_next_ + 1) % kGravityAngleWindow;
      }
      const std::array<double, 3> a = {f[0], f[1] - kGravity, f[2]};
      for (int c = 0; c < 3; ++c) {
        p[c] += v[c] * h + 0.5 * a[c] * h * h;
        v[c] += a[c] * h;
      }
      t = t_next;
    }
    if (k < acc.size()) {
      a_cam = acc[k].second;
      ++k;
    } else {
      break;
    }
  }
  last_accel_ = a_cam;
  have_accel_ = true;
  v_prop_ = v;
  have_prop_ = true;
  const c8::HMatrix& pp = prev.pose;
  const float x = pp(0, 3) + static_cast<float>(p[0] / metres_per_unit);
  const float y = pp(1, 3) + static_cast<float>(p[1] / metres_per_unit);
  const float z = pp(2, 3) + static_cast<float>(p[2] / metres_per_unit);
  pred->pose = c8::cameraMotion(c8::HPoint3(x, y, z), c8::Quaternion::fromHMatrix(pred->pose));
  return true;
}

void ViPredictor::Update(const FrameState& prev, const c8::HMatrix& pose_now, int64_t now_ns,
                         double metres_per_unit, double gain, bool discontinuity) {
  const double dt = static_cast<double>(now_ns - prev.time_ns) * 1e-9;
  if (discontinuity || !(metres_per_unit > 0.0) || !(dt > 0.0) || dt > kMaxGapS) {
    initialized_ = false;
    have_prop_ = false;
    return;
  }
  const std::array<double, 3> vv = {
      metres_per_unit * static_cast<double>(pose_now(0, 3) - prev.pose(0, 3)) / dt,
      metres_per_unit * static_cast<double>(pose_now(1, 3) - prev.pose(1, 3)) / dt,
      metres_per_unit * static_cast<double>(pose_now(2, 3) - prev.pose(2, 3)) / dt};
  if (!initialized_ || !have_prop_) {
    v_ = vv;
    initialized_ = true;
  } else {
    for (int c = 0; c < 3; ++c) v_[c] = v_prop_[c] + gain * (vv[c] - v_prop_[c]);
  }
  have_prop_ = false;
}

}  // namespace slam_refact::vio_tracker

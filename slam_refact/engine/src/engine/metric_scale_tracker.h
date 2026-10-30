// Accuracy mode (ACC-2): continuous metric scale of the published trajectory.
//
// The original publishes responsive units (camera height at start = origin.y) whose scale drifts
// with the map (about 30 % in 30 s on ARCore-referenced Flip6 fixtures), and its absolute mode
// latches the MetricScaleEstimator once. This tracker re-estimates metres per published unit
// continuously from the camera's visual acceleration and the accelerometer, and the glue turns the
// published displacement into metres with the current estimate (slam_refact/docs/reports/
// accuracy.md §4).
//
// Estimate, every `update_period_s` of frame time, over the last `window_s` seconds:
//   1. resample published camera positions/rotations and accelerationIncludingGravity (W3C device
//      frame = published GL camera frame, VERIFIED median gravity error 2.9 deg) onto a 60 Hz grid;
//   2. visual acceleration = Savitzky-Golay second derivative (15 taps, cubic) of the positions;
//      inertial acceleration = R_world_camera * aIG - (0, g, 0);
//   3. band-pass both (Butterworth order 2, 0.4-2.5 Hz, forward-backward) to remove bias / tilt
//      error (low) and differentiation noise (high);
//   4. IMU time lag: the lag in [-0.30, 0.30] s with the highest correlation (positive = the IMU is
//      stamped later than the frames; the Flip6 Chrome records show about 0.1-0.15 s of offset
//      between DeviceMotion receive times and frame times, its sign depends on the page clock);
//   5. window scale = sqrt(sum |a_imu|^2 / sum |a_vis|^2) (geometric mean of both regression
//      slopes, so noise on either side does not attenuate it), accepted when the correlation is
//      >= min_correlation and the band-passed inertial RMS is >= min_excitation;
//   6. fuse log(scale) in a scalar Kalman filter (random walk process noise).
// The tracker is initialised after `min_accepted` accepted windows with sigma(log s) below
// `init_sigma`.

#ifndef SLAM_REFACT_ENGINE_METRIC_SCALE_TRACKER_H_
#define SLAM_REFACT_ENGINE_METRIC_SCALE_TRACKER_H_

#include <array>
#include <limits>
#include <cstdint>
#include <deque>

namespace slam_refact {

struct MetricScaleParams {
  double window_s = 4.0;
  double update_period_s = 0.5;
  double prior_scale = 0.7;        // metres per published unit before any estimate
  double prior_sigma_log = 0.5;
  double process_sigma_log = 0.03;  // per sqrt(second)
  double min_correlation = 0.5;
  double min_excitation = 0.15;     // m/s^2 RMS of the band-passed inertial acceleration
  int min_accepted = 2;
  double init_sigma = 0.15;
  // Low band (accuracy mode experimental option): 0.1-0.8 Hz instead of 0.4-2.5 Hz, for camera
  // streams whose image motion above ~1 Hz is stabilized (EIS) and does not match the IMU
  // (slam_refact/docs/reports/camera-xr_reference.md). Longer filter margins (8 s history, 3 s
  // look-back, so the estimate lags by 3 s) and a measurement variance floor of 0.1^2 (log
  // scale) from the offline evaluation on the ARCore fixtures. Use with window_s >= 6.
  bool low_band = false;
  // Fixed IMU lag in seconds instead of the +/-0.30 s search when finite (the low-band
  // correlation is flat in lag; the devices show about -0.09 s).
  double fixed_lag_s = std::numeric_limits<double>::quiet_NaN();
};

class MetricScaleTracker {
 public:
  explicit MetricScaleTracker(const MetricScaleParams& params = {});

  void Reset();
  // Map reset (the responsive unit is redefined): inflates the uncertainty and drops the
  // convergence; the current estimate is kept as the starting value.
  void Perturb();
  // accelerationIncludingGravity, W3C device frame, m/s^2, at receive time.
  void PushImu(int64_t time_ns, const std::array<double, 3>& accel_including_gravity);
  // Published camera pose (responsive units, GL camera, world y-up) of a fresh tracked frame.
  void PushFrame(int64_t time_ns, const std::array<double, 3>& position,
                 const std::array<double, 4>& rotation_wxyz);

  bool initialized() const { return initialized_; }
  double scale() const;
  double sigma_log() const;
  double lag_s() const { return lag_s_; }
  double last_correlation() const { return last_correlation_; }
  int accepted_windows() const { return accepted_; }

 private:
  // Seconds of history kept beyond the window (filter margins + lag search).
  double Retention() const { return params_.low_band ? 12.5 : 6.0; }

 public:

 private:
  struct Frame {
    double t;
    std::array<double, 3> p;
    std::array<double, 4> q;
  };
  struct Imu {
    double t;
    std::array<double, 3> a;
  };
  void Update(double now);

  MetricScaleParams params_;
  std::deque<Frame> frames_;
  std::deque<Imu> imu_;
  double log_scale_ = 0.0;
  double var_ = 0.0;
  double last_update_t_ = -1e300;
  double last_frame_t_ = 0.0;
  double lag_s_ = 0.0;
  double last_correlation_ = 0.0;
  int accepted_ = 0;
  bool initialized_ = false;
};

}  // namespace slam_refact

#endif  // SLAM_REFACT_ENGINE_METRIC_SCALE_TRACKER_H_

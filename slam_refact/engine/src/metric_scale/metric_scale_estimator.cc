#include "metric_scale/metric_scale_estimator.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <optional>
#include <vector>

#include "c8/hvector.h"
#include "common/check.h"
#include "common/stage_timer.h"
#include "config/settings.h"
#include "metric_scale/metric_scale_estimator_internal.h"

namespace slam_refact::metric_scale {
namespace {

std::vector<float> Axis(const std::vector<Vec3>& v, int j) {
  std::vector<float> out;
  out.reserve(v.size());
  for (const Vec3& s : v) {
    out.push_back(s[j]);
  }
  return out;
}

// One axis of the temporal-alignment peak search (S5 §3.6): best lag and |xcorr| at it.
struct AxisPeak {
  float lag = 0.f;
  float value = 0.f;
  bool valid = false;
};

AxisPeak FindPeak(const std::vector<float>& xc, const std::vector<float>& lags, float cutoff,
                  bool favor) {
  float score = std::numeric_limits<float>::min();  // FLT_MIN; strictly greater wins
  AxisPeak peak;
  for (size_t j = 0; j < lags.size(); ++j) {
    const float lag = lags[j];
    if (std::fabs(lag) >= cutoff) {
      continue;
    }
    const float w = favor ? std::max(lag * -3.6f * lag + 1.0f, 0.1f) : 1.0f;
    const float s = w * std::fabs(xc[j]);
    if (s > score) {
      score = s;
      peak.lag = lag;
      peak.value = std::fabs(xc[j]);
    }
  }
  peak.valid = std::fabs(peak.lag) < cutoff;
  return peak;
}

}  // namespace

void MetricScaleEstimator::Impl::Reset() {
  kf = PositionKalman();
  imu = ImuBuffer();
  acc_sum = 0.f;
  active = false;
  high_accel_seen = false;
  have_estimate = false;
  scale = 1.f;
  fit_residual = 0.f;
}

std::optional<ScaleFit> EstimateScale(const PositionKalman& kf, const ImuBuffer& imu,
                                      int32_t device_model) {
  const config::MetricScaleEstimatorSettings& ms = config::MetricScaleEstimatorSettings::Get();
  const config::AlignmentSettings& al = config::AlignmentSettings::Get();

  // smooth-vio-accel (§3.4): smoothed acceleration rotated into the camera frame.
  std::vector<float> t_v;
  std::vector<Vec3> a_v;
  {
    StageTimer timer("smooth-vio-accel");
    const std::vector<KalmanState> sm = RtsSmoothPosition(kf.states, kf.q);
    const int keep = static_cast<int>(sm.size()) - ms.num_end_frames_to_skip;
    for (int m = 0; m < keep; ++m) {
      // R_wc^T a_world with the stored inverse (no translation), in the original's order
      // (folded wat of 944): v = (C2 az + (C0 ax + ay C1)) + (0, 0, 0, 1), then v.xyz / v.w.
      const float ax = sm[m].x(6);
      const float ay = sm[m].x(7);
      const float az = sm[m].x(8);
      const c8::HMatrix pose_inv = kf.poses[m].inv();
      const std::array<float, 16>& inv = pose_inv.data();  // column-major
      float c[3];
      for (int r = 0; r < 3; ++r) {
        c[r] = (inv[8 + r] * az + (inv[r] * ax + ay * inv[4 + r])) + 0.0f;
      }
      const float w = (0.0f * az + (0.0f * ax + ay * 0.0f)) + 1.0f;
      t_v.push_back(kf.frame_times[m]);
      a_v.push_back({c[0] / w, c[1] / w, c[2] / w});
    }
  }
  if (t_v.empty()) {
    return std::nullopt;
  }

  // IMU extraction (1178, §3.5).
  const float t0 = kf.frame_times.front();
  const float t1 = t_v.back();
  std::vector<float> t_i;
  std::vector<Vec3> a_i;
  for (size_t j = 0; j < imu.lin_times.size(); ++j) {
    if (t0 <= imu.lin_times[j] && imu.lin_times[j] <= t1) {
      t_i.push_back(imu.lin_times[j]);
      a_i.push_back(imu.lin_values[j]);
    }
  }
  const int method = ms.scale_estimation_method;
  std::vector<float> t_g;
  std::vector<Vec3> w_g;
  std::vector<Vec3> wdot_g;
  if (method == 4 || method == 5) {
    const std::vector<KalmanState> smw = RtsSmooth(imu.states, imu.q);
    for (size_t j = 0; j < imu.gyro_times.size() && j < smw.size(); ++j) {
      if (t0 <= imu.gyro_times[j] && imu.gyro_times[j] <= t1) {
        t_g.push_back(imu.gyro_times[j]);
        w_g.push_back({smw[j].x(0), smw[j].x(1), smw[j].x(2)});
        wdot_g.push_back({smw[j].x(3), smw[j].x(4), smw[j].x(5)});
      }
    }
  }

  // temporal-alignment-imu (§3.6).
  {
    StageTimer timer("temporal-alignment-imu");
    if (ms.temporal_alignment) {
      float fs;
      if (ms.place_with_inteval && imu.interval_ns != 0) {
        fs = (1.0f / static_cast<float>(imu.interval_ns)) * 1e9f;
      } else if (ms.fallback_to60_hz) {
        fs = 60.0f;
      } else {
        fs = 1.0f / MeanDt(t_i);
      }
      t_i = PlaceOnGrid(t_i, fs, al.place_sensor_events_start_at_end);
      t_g = PlaceOnGrid(t_g, fs, al.place_sensor_events_start_at_end);
      float shift = 0.f;
      if (!t_v.empty() && t_i.size() >= 2 && t_v.size() == a_v.size() && t_i.size() == a_i.size()) {
        const std::vector<Vec3> v_i = Interpolate3(t_v, a_v, t_i);
        std::array<std::vector<float>, 3> xc;
        for (int c = 0; c < 3; ++c) {
          xc[c] = CrossCorrelate(Axis(a_i, c), Axis(v_i, c));
        }
        const float dt = t_i[1] - t_i[0];
        const int r = static_cast<int>(xc[0].size()) >> 1;
        std::vector<float> lags(xc[0].size());
        for (int m = 0; m < static_cast<int>(lags.size()); ++m) {
          lags[m] = dt * static_cast<float>(m - r);
        }
        std::array<AxisPeak, 3> pk;
        for (int c = 0; c < 3; ++c) {
          pk[c] = FindPeak(xc[c], lags, al.cross_correlation_timeoffset_cutoff,
                           al.favor_cross_correlation_peak_near_zero);
        }
        const float den = ((pk[0].valid ? pk[0].value + 0.0f : 0.0f) + (pk[1].valid ? pk[1].value : -0.0f)) +
                          (pk[2].valid ? pk[2].value : -0.0f);
        if (den != 0.0f) {
          const float s1 = pk[0].valid ? pk[0].lag * pk[0].value + 0.0f : 0.0f;
          const float s2 = pk[1].valid ? pk[1].lag * pk[1].value + s1 : s1;
          const float s3 = pk[2].valid ? pk[2].lag * pk[2].value + s2 : s2;
          shift = -s3 / den;
        }
      }
      ShiftTimes(shift, &t_i);
      ShiftTimes(shift, &t_g);
    }
  }
  if (t_i.empty()) {
    return std::nullopt;
  }
  const float lo = std::max(t_v.front(), t_i.front());
  const float hi = std::min(t_v.back(), t_i.back());
  CropToRange(lo, hi, &t_v, &a_v);
  CropToRange(lo, hi, &t_i, &a_i);
  std::vector<float> t_g2 = t_g;
  CropToRange(lo, hi, &t_g, &w_g);
  CropToRange(lo, hi, &t_g2, &wdot_g);
  if (t_v.empty() || t_i.empty()) {
    return std::nullopt;
  }
  const std::vector<float> t_c = Dedupe(MergeTimes(t_i, t_g), 0.001f);
  const std::vector<Vec3> v = Interpolate3(t_v, a_v, t_c);
  const std::vector<Vec3> i_c = Interpolate3(t_i, a_i, t_c);
  const std::vector<Vec3> om = Interpolate3(t_g, w_g, t_c);
  const std::vector<Vec3> omd = Interpolate3(t_g2, wdot_g, t_c);
  const float sigma = ms.imu_gaussian_sigma;
  const std::vector<Vec3> i_s =
      sigma != 0.0f ? GaussianSmooth3(i_c, sigma, static_cast<int>(sigma * 3.0f)) : i_c;

  switch (method) {
    case 1: {
      StageTimer timer("scale-estimation-camera-space");
      return ScaleCameraSpace(v, i_s);
    }
    case 2: {
      StageTimer timer("scale-estimation-l2-camera-space");
      return ScaleL2(v, i_s);
    }
    case 3: {
      StageTimer timer("scale-estimation-frequency");
      return ScaleFrequency(t_v, a_v, i_c, t_c, ms.max_freq_for_alignment);
    }
    case 4: {
      StageTimer timer("scale-estimation-spatial-alignment");
      return ScaleSpatial(v, i_s, om, omd);
    }
    case 5: {
      StageTimer timer("scale-estimation-fixed-spatial-alignment");
      return ScaleFixedSpatial(v, i_s, om, omd, FixedLeverArm(device_model).translation);
    }
    default:
      SR_CHECK(false, "Scale estimation method unknown");
  }
  return std::nullopt;
}

MetricScaleEstimator::MetricScaleEstimator(float movement_check_period_s)
    : impl_(std::make_unique<Impl>(movement_check_period_s)) {}

MetricScaleEstimator::~MetricScaleEstimator() = default;

bool MetricScaleEstimator::PushFrame(const TrackingSensorFrame& f, const c8::HMatrix& pose,
                                     int64_t time_ns, int tracker_state, bool reset_flag) {
  StageTimer timer("metric-scale-estimator-push-frame");
  Impl& s = *impl_;
  const config::MetricScaleEstimatorSettings& ms = config::MetricScaleEstimatorSettings::Get();

  // (A) sensor statistics, every call.
  for (const TrackingSensorEvent& e : f.sensor_events) {
    if (s.first_event_ever_ns == std::numeric_limits<int64_t>::max()) {
      s.first_event_ever_ns = e.event_time_ns;
    }
    const double t = static_cast<double>(static_cast<float>(e.event_time_ns - s.first_event_ever_ns)) * 1e-9;
    const float mag = std::sqrt(e.z * e.z + (e.x * e.x + e.y * e.y));
    if (e.kind == ImuEvent::Kind::kLinearAcceleration) {
      s.lin_acc_mag.Push(static_cast<float>(t), mag);
      s.min_lin_acc = mag < s.min_lin_acc ? mag : s.min_lin_acc;
      const float m = mag - (ms.imu_remove_bias ? s.min_lin_acc : 0.0f);
      if (m >= ms.add_accel_to_sum_threshold) {
        // Weighted by the elapsed time since the first event (Q12, S5 U11; original behavior).
        s.acc_sum = static_cast<float>(t * static_cast<double>(m) + static_cast<double>(s.acc_sum));
      }
    } else if (e.kind == ImuEvent::Kind::kGyroscope) {
      s.gyro_mag.Push(static_cast<float>(t), mag);
    }
  }
  if (ms.imu_high_accel_threshold < s.lin_acc_mag.Mean()) {
    s.high_accel_seen = true;
  }
  const bool gyro_reset =
      ms.gyro_reset_threshold > 0.0f && ms.gyro_reset_threshold < s.gyro_mag.Mean();

  // (B) time bases.
  bool was_initialized;
  if (s.imu_time_base_ns == std::numeric_limits<int64_t>::max()) {
    if (!f.sensor_events.empty()) {
      s.imu_time_base_ns = f.sensor_events[0].event_time_ns;
    }
    was_initialized = false;
  } else {
    was_initialized = s.frame_time_base_ns != std::numeric_limits<int64_t>::max();
  }
  if (s.frame_time_base_ns == std::numeric_limits<int64_t>::max()) {
    s.frame_time_base_ns = time_ns;
  }

  // (C) reset conditions.
  if (tracker_state == 0 || reset_flag || !was_initialized || gyro_reset) {
    s.Reset();
    return false;
  }
  if (s.have_estimate && !ms.estimate_scale_after_have_estimate) {
    return true;
  }

  // (D) both pushes always run.
  const bool ready = s.kf.Push(pose, s.frame_time_base_ns, time_ns, ms.max_frames_in_queue,
                               ms.num_end_frames_to_skip, ms.scale_prediction_frame_number,
                               ms.use_fps_scaling_if_ios && f.is_ios);
  const float t_oldest = ready ? s.kf.frame_times.front() : 0.0f;
  const bool imu_ok = s.imu.Push(f.sensor_events, s.imu_time_base_ns, t_oldest, ms.max_frames_in_queue);
  if (!(ready && imu_ok)) {
    return false;
  }

  // (E) gate.
  if (ms.active_threshold_check) {
    const float m = s.lin_acc_mag.Mean();
    if (ms.active_start_threshold < m && !s.active) {
      s.active = true;
      return false;
    }
    if (!(ms.active_end_threshold > m) || !s.active) {
      return false;
    }
    s.active = false;  // estimate at the end of a motion burst
  } else if (ms.sum_check) {
    if (!(s.acc_sum > ms.sum_check_threshold) || !s.high_accel_seen) {
      return false;
    }
  }

  // (F) estimate. frame+0 deviceModel selects the method-5 lever arm (WP4-CC-1).
  const std::optional<ScaleFit> fit = EstimateScale(s.kf, s.imu, f.device_model);
  if (!fit) {
    return false;
  }
  // (G) bounds. A NaN scale passes (every comparison is false).
  if ((ms.min_scale_allowed > 0.0f && ms.min_scale_allowed > fit->scale) ||
      (ms.max_scale_allowed > 0.0f && ms.max_scale_allowed < fit->scale)) {
    if (ms.out_of_bound_reset) {
      s.Reset();
    }
    return false;
  }
  s.scale = fit->scale;
  s.fit_residual = fit->residual;
  s.have_estimate = true;
  return true;
}

float MetricScaleEstimator::scale() const { return impl_->scale; }

float MetricScaleEstimator::residual() const { return impl_->fit_residual; }

void MetricScaleEstimator::Reset() { impl_->Reset(); }

}  // namespace slam_refact::metric_scale

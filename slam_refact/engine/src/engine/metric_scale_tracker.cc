#include "engine/metric_scale_tracker.h"

#include <algorithm>
#include <cmath>
#include <vector>

namespace slam_refact {
namespace {

constexpr double kGravity = 9.80665;
constexpr double kGridHz = 60.0;

// Butterworth order 2 band-pass 0.4-2.5 Hz at 60 Hz as second-order sections
// (scipy.signal.butter(2, [0.4, 2.5], 'band', fs=60, output='sos')): b0 b1 b2 a0 a1 a2.
constexpr double kSos[2][6] = {
    {0.010432413371093413, 0.020864826742186825, 0.010432413371093413, 1.0, -1.7235714232560098,
     0.77188592052907},
    {1.0, -2.0, 1.0, 1.0, -1.9471234212063775, 0.9492672568371078},
};

// Low band 0.1-0.8 Hz at 60 Hz (scipy.signal.butter(2, [0.1, 0.8], 'band', fs=60, output='sos')).
constexpr double kSosLow[2][6] = {
    {0.0012766473625026938, 0.0025532947250053875, 0.0012766473625026938, 1.0,
     -1.9084474808154683, 0.9139605724456182},
    {1.0, -2.0, 1.0, 1.0, -1.9862614093647324, 0.9863940612283523},
};

// Savitzky-Golay second derivative, 15 taps, cubic, delta = 1/60 s
// (scipy.signal.savgol_coeffs(15, 3, deriv=2, delta=1/60, use='dot')); symmetric.
constexpr double kSavGol[15] = {
    52.94117647058828,  30.252100840336087, 11.053652230122772, -4.654169360051764,
    -16.87136393018752, -25.597931480284494, -30.833872010342674, -32.579185520362074,
    -30.83387201034268, -25.597931480284508, -16.87136393018753, -4.654169360051766,
    11.05365223012279,  30.25210084033614,  52.94117647058829};

using Vec3 = std::array<double, 3>;

void Rotate(const std::array<double, 4>& q, const Vec3& v, Vec3* out) {
  const double w = q[0], x = q[1], y = q[2], z = q[3];
  const double r[3][3] = {{1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)},
                          {2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)},
                          {2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)}};
  for (int i = 0; i < 3; ++i) {
    (*out)[static_cast<size_t>(i)] = r[i][0] * v[0] + r[i][1] * v[1] + r[i][2] * v[2];
  }
}

// One channel, forward-backward through both sections, odd extension of `pad` samples.
std::vector<double> FiltFilt(const std::vector<double>& x, const double (&sos)[2][6]) {
  const size_t n = x.size();
  const size_t pad = std::min<size_t>(n - 1, 60);
  std::vector<double> ext;
  ext.reserve(n + 2 * pad);
  for (size_t k = pad; k >= 1; --k) ext.push_back(2.0 * x[0] - x[k]);
  ext.insert(ext.end(), x.begin(), x.end());
  for (size_t k = 1; k <= pad; ++k) ext.push_back(2.0 * x[n - 1] - x[n - 1 - k]);
  const auto pass = [&sos](std::vector<double>* v) {
    for (const auto& s : sos) {
      double z1 = 0.0, z2 = 0.0;  // direct form II transposed
      for (double& sample : *v) {
        const double in = sample;
        const double out = s[0] * in + z1;
        z1 = s[1] * in - s[4] * out + z2;
        z2 = s[2] * in - s[5] * out;
        sample = out;
      }
    }
  };
  pass(&ext);
  std::reverse(ext.begin(), ext.end());
  pass(&ext);
  std::reverse(ext.begin(), ext.end());
  return std::vector<double>(ext.begin() + static_cast<std::ptrdiff_t>(pad),
                             ext.begin() + static_cast<std::ptrdiff_t>(pad + n));
}

}  // namespace

MetricScaleTracker::MetricScaleTracker(const MetricScaleParams& params) : params_(params) {
  Reset();
}

void MetricScaleTracker::Reset() {
  frames_.clear();
  imu_.clear();
  log_scale_ = std::log(params_.prior_scale);
  var_ = params_.prior_sigma_log * params_.prior_sigma_log;
  last_update_t_ = -1e300;
  lag_s_ = 0.0;
  last_correlation_ = 0.0;
  accepted_ = 0;
  initialized_ = false;
}

void MetricScaleTracker::Perturb() {
  // The responsive unit is redefined: keep the estimate as a starting value only.
  var_ = std::max(var_, params_.prior_sigma_log * params_.prior_sigma_log);
  frames_.clear();
  accepted_ = 0;
  initialized_ = false;
}

double MetricScaleTracker::scale() const { return std::exp(log_scale_); }
double MetricScaleTracker::sigma_log() const { return std::sqrt(var_); }

void MetricScaleTracker::PushImu(int64_t time_ns, const std::array<double, 3>& a) {
  const double t = static_cast<double>(time_ns) * 1e-9;
  if (!imu_.empty() && t <= imu_.back().t) return;
  imu_.push_back({t, a});
  while (!imu_.empty() && imu_.front().t < t - params_.window_s - Retention()) imu_.pop_front();
}

void MetricScaleTracker::PushFrame(int64_t time_ns, const std::array<double, 3>& p,
                                   const std::array<double, 4>& q) {
  const double t = static_cast<double>(time_ns) * 1e-9;
  if (!frames_.empty() && t <= frames_.back().t) return;
  frames_.push_back({t, p, q});
  while (!frames_.empty() && frames_.front().t < t - params_.window_s - Retention()) frames_.pop_front();
  last_frame_t_ = t;
  if (t - last_update_t_ >= params_.update_period_s) {
    Update(t);
  }
}

void MetricScaleTracker::Update(double now) {
  const double dt_since = last_update_t_ < -1e299 ? 0.0 : now - last_update_t_;
  last_update_t_ = now;
  var_ += params_.process_sigma_log * params_.process_sigma_log * dt_since;
  if (frames_.size() < 10 || imu_.size() < 10) return;
  // Filter span = left margin (history) + window + right margin; energies only over the window,
  // so the band-pass edge transients stay outside it (the estimate lags by the right margin).
  const double kLeftMargin = params_.low_band ? 8.0 : 3.0;
  const double kRightMargin = params_.low_band ? 3.0 : 1.0;
  const auto& sos = params_.low_band ? kSosLow : kSos;
  const double band_lo = params_.low_band ? 0.1 : 0.4, band_hi = params_.low_band ? 0.8 : 2.5;
  const bool fixed_lag = std::isfinite(params_.fixed_lag_s);
  // Every lag candidate must find IMU samples over the whole span.
  constexpr double kMaxLag = 0.30;
  const double t_end = std::min(frames_.back().t, imu_.back().t - kMaxLag) - 0.05;
  const double t_begin = std::max(frames_.front().t, imu_.front().t + kMaxLag) + 0.05;
  const double span = std::min(kLeftMargin + params_.window_s + kRightMargin, t_end - t_begin);
  if (span < params_.window_s + kRightMargin + 0.5) return;
  const double t0 = t_end - span;
  const size_t n = static_cast<size_t>(span * kGridHz);
  const size_t w_end = n - static_cast<size_t>(kRightMargin * kGridHz);
  const size_t w_begin = w_end - static_cast<size_t>(params_.window_s * kGridHz);
  std::vector<double> tg(n);
  for (size_t k = 0; k < n; ++k) tg[k] = t0 + static_cast<double>(k) / kGridHz;

  // Positions and rotations on the grid (linear / normalised linear interpolation).
  std::vector<Vec3> pos(n);
  std::vector<std::array<double, 4>> rot(n);
  size_t j = 0;
  for (size_t k = 0; k < n; ++k) {
    while (j + 2 < frames_.size() && frames_[j + 1].t < tg[k]) ++j;
    const Frame& a = frames_[j];
    const Frame& b = frames_[j + 1];
    const double u = std::clamp((tg[k] - a.t) / std::max(1e-9, b.t - a.t), 0.0, 1.0);
    for (int c = 0; c < 3; ++c) {
      pos[k][static_cast<size_t>(c)] = a.p[static_cast<size_t>(c)] +
                                       u * (b.p[static_cast<size_t>(c)] - a.p[static_cast<size_t>(c)]);
    }
    const double dot = a.q[0] * b.q[0] + a.q[1] * b.q[1] + a.q[2] * b.q[2] + a.q[3] * b.q[3];
    const double sgn = dot < 0.0 ? -1.0 : 1.0;
    double norm = 0.0;
    for (int c = 0; c < 4; ++c) {
      rot[k][static_cast<size_t>(c)] = a.q[static_cast<size_t>(c)] +
                                       u * (sgn * b.q[static_cast<size_t>(c)] - a.q[static_cast<size_t>(c)]);
      norm += rot[k][static_cast<size_t>(c)] * rot[k][static_cast<size_t>(c)];
    }
    norm = std::sqrt(norm);
    for (int c = 0; c < 4; ++c) rot[k][static_cast<size_t>(c)] /= norm;
  }

  // Visual acceleration (published units / s^2), band-passed per axis.
  std::array<std::vector<double>, 3> vis;
  for (int c = 0; c < 3; ++c) {
    std::vector<double> acc(n, 0.0);
    for (size_t k = 0; k < n; ++k) {
      double s = 0.0;
      for (int m = 0; m < 15; ++m) {
        const long idx = std::clamp(static_cast<long>(k) + m - 7, 0L, static_cast<long>(n) - 1);
        s += kSavGol[m] * pos[static_cast<size_t>(idx)][static_cast<size_t>(c)];
      }
      acc[k] = s;
    }
    vis[static_cast<size_t>(c)] = FiltFilt(acc, sos);
  }
  double sxx = 0.0;
  for (int c = 0; c < 3; ++c) {
    for (size_t k = w_begin; k < w_end; ++k) {
      sxx += vis[static_cast<size_t>(c)][k] * vis[static_cast<size_t>(c)][k];
    }
  }
  if (!(sxx > 0.0)) return;

  // Inertial acceleration for each candidate lag; keep the best correlated.
  double best_rho = -2.0, best_syy = 0.0, best_sxy = 0.0, best_lag = 0.0;
  const int lag_first = fixed_lag ? 0 : -static_cast<int>(kMaxLag * 1000);
  const int lag_last = fixed_lag ? 0 : static_cast<int>(kMaxLag * 1000);
  for (int lag_ms = lag_first; lag_ms <= lag_last; lag_ms += 10) {
    const double lag = fixed_lag ? params_.fixed_lag_s : lag_ms * 1e-3;
    std::array<std::vector<double>, 3> imu;
    for (auto& v : imu) v.assign(n, 0.0);
    size_t i = 0;
    bool ok = true;
    for (size_t k = 0; k < n; ++k) {
      const double ts = tg[k] + lag;
      if (ts < imu_.front().t || ts > imu_.back().t) {
        ok = false;
        break;
      }
      while (i + 2 < imu_.size() && imu_[i + 1].t < ts) ++i;
      const Imu& a = imu_[i];
      const Imu& b = imu_[i + 1];
      const double u = std::clamp((ts - a.t) / std::max(1e-9, b.t - a.t), 0.0, 1.0);
      Vec3 f;
      for (int c = 0; c < 3; ++c) {
        f[static_cast<size_t>(c)] = a.a[static_cast<size_t>(c)] +
                                    u * (b.a[static_cast<size_t>(c)] - a.a[static_cast<size_t>(c)]);
      }
      Vec3 w;
      Rotate(rot[k], f, &w);
      imu[0][k] = w[0];
      imu[1][k] = w[1] - kGravity;
      imu[2][k] = w[2];
    }
    if (!ok) continue;
    double sxy = 0.0, syy = 0.0;
    for (int c = 0; c < 3; ++c) {
      const std::vector<double> f = FiltFilt(imu[static_cast<size_t>(c)], sos);
      for (size_t k = w_begin; k < w_end; ++k) {
        sxy += f[k] * vis[static_cast<size_t>(c)][k];
        syy += f[k] * f[k];
      }
    }
    if (!(syy > 0.0)) continue;
    const double rho = sxy / std::sqrt(sxx * syy);
    if (rho > best_rho) {
      best_rho = rho;
      best_syy = syy;
      best_sxy = sxy;
      best_lag = lag;
    }
  }
  (void)best_sxy;
  if (best_rho < -1.0) return;
  // A maximum on the search boundary means the true lag is outside it: reject the window.
  if (!fixed_lag && std::fabs(std::fabs(best_lag) - kMaxLag) < 1e-6) return;
  last_correlation_ = best_rho;
  const double rms = std::sqrt(best_syy / static_cast<double>(3 * (w_end - w_begin)));
  if (best_rho < params_.min_correlation || rms < params_.min_excitation) return;
  lag_s_ = best_lag;
  const double window_scale = std::sqrt(best_syy / sxx);
  const double neff = params_.window_s * 2.0 * (band_hi - band_lo);
  const double floor_sigma = params_.low_band ? 0.10 : 0.03;
  const double meas_var =
      (1.0 - best_rho * best_rho) / (best_rho * best_rho * neff) + floor_sigma * floor_sigma;
  const double gain = var_ / (var_ + meas_var);
  log_scale_ += gain * (std::log(window_scale) - log_scale_);
  var_ *= (1.0 - gain);
  ++accepted_;
  if (!initialized_ && accepted_ >= params_.min_accepted && std::sqrt(var_) < params_.init_sigma) {
    initialized_ = true;
  }
}

}  // namespace slam_refact

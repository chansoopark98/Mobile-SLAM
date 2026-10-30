// Pure alignment / scale-fit functions of the MetricScaleEstimator (S5 §3.4-§3.7, §5.4-§5.8):
// funcs 1179-1200 and FftCompute (1240/1241). Module-private header (WP4, not frozen).
//
// Numeric policy (S5 §2.4): float everywhere except the listed double spots. Where the spec text
// and the wasm instruction stream disagree on the association of a float sum, the instruction
// stream (objdump of the func) wins; such places are marked "objdump".

#ifndef SLAM_REFACT_METRIC_SCALE_ALIGNMENT_MATH_H_
#define SLAM_REFACT_METRIC_SCALE_ALIGNMENT_MATH_H_

#include <array>
#include <cstdint>
#include <vector>

namespace slam_refact::metric_scale {

// (x, y, z) sample; the original stores Vec4 with w = 0.
using Vec3 = std::array<float, 3>;

// Result of a scale fit (S5 §3.7). residual = -1 for methods 3 and 4.
struct ScaleFit {
  float scale = 1.f;
  float residual = 0.f;
};

// 1179: mean of consecutive differences, accumulated as ((acc + t[i]) - t[i-1]).
float MeanDt(const std::vector<float>& t);

// 1181: quantizes times (s) to a grid of rate fs (Hz). Alignment.placeSensorEventsStartAtEnd
// selects the end-first slot assignment (default) or the front-first variant (S5 §5.4).
std::vector<float> PlaceOnGrid(const std::vector<float>& t, float fs, bool start_at_end);

// 1198: 1-D linear interpolation of (ts, xs) at td, extrapolating at both ends.
std::vector<float> Interpolate(const std::vector<float>& ts, const std::vector<float>& xs,
                               const std::vector<float>& td);

// 1182: Interpolate per axis. Empty when the source is empty or sizes mismatch.
std::vector<Vec3> Interpolate3(const std::vector<float>& ts, const std::vector<Vec3>& xs,
                               const std::vector<float>& td);

// 1183: full cross-correlation, size n + m - 1 (empty if either input is empty).
std::vector<float> CrossCorrelate(const std::vector<float>& x, const std::vector<float>& y);

// 1185: t += dt.
void ShiftTimes(float dt, std::vector<float>* t);

// 1186: keeps samples with lo <= t <= hi (data follows the times when given).
void CropToRange(float lo, float hi, std::vector<float>* t, std::vector<Vec3>* data);

// 1187: merge of two sorted time lists; A is taken only when a < b.
std::vector<float> MergeTimes(const std::vector<float>& a, const std::vector<float>& b);

// 1188: keeps a time only when it exceeds the last kept one by more than eps.
std::vector<float> Dedupe(const std::vector<float>& t, float eps);

// 1199: normalized gaussian kernel of length 2 * (size / 2) + 1.
std::vector<float> GaussianKernel(float sigma, int size);

// 1200: normalized convolution (edge samples divided by the valid kernel weight).
std::vector<float> GaussianSmooth(const std::vector<float>& x, float sigma, int size);

// 1190: GaussianSmooth per axis.
std::vector<Vec3> GaussianSmooth3(const std::vector<Vec3>& x, float sigma, int size);

// 1191 method 1 "camera-space": s = |sum V.I / sum V.V|, residual = sum |I - sV|^2 / N.
ScaleFit ScaleCameraSpace(const std::vector<Vec3>& v, const std::vector<Vec3>& imu);

// 1192 method 2 "l2-camera-space": s = sum |V||I| / sum |V|^2.
ScaleFit ScaleL2(const std::vector<Vec3>& v, const std::vector<Vec3>& imu);

// 1240/1241 (public reality/engine/tracking/fft-compute.cc semantics, Eigen FFT backend, S5
// §7.3): complex FFT of real input, bins k in [0, N/2) with DC, amp = |X_k| / N,
// freq = k * fs / N.
class FftCompute {
 public:
  explicit FftCompute(int n);
  void Compute(const std::vector<float>& data, float sampling_rate);
  const std::vector<float>& frequencies() const { return frequencies_; }
  const std::vector<float>& amplitudes() const { return amplitudes_; }

 private:
  int n_;
  std::vector<float> frequencies_;
  std::vector<float> amplitudes_;
};

// 1193 method 3 "frequency" (S5 §5.7). v_t / v_a: cropped VIO samples; imu: IMU on t_c
// (before smoothing); max_freq: MetricScaleEstimator.maxFreqForAlignment.
ScaleFit ScaleFrequency(const std::vector<float>& v_t, const std::vector<Vec3>& v_a,
                        const std::vector<Vec3>& imu, const std::vector<float>& t_c,
                        float max_freq);

// 1195 method 4 "spatial-alignment": I_k ~ M_k p + s V_k, M_k = [W]x^2 + [Wdot]x, 4x4 normal
// equations solved with Eigen LDLT (|D| <= FLT_MIN -> 0). Scale = |x_3|.
ScaleFit ScaleSpatial(const std::vector<Vec3>& v, const std::vector<Vec3>& imu,
                      const std::vector<Vec3>& omega, const std::vector<Vec3>& omega_dot);

// 1196: fixed lever arm of a device model (translation r in m, quaternion x,y,z,w). Devices not
// in the table: r = 0, q = identity.
struct LeverArm {
  Vec3 translation{0.f, 0.f, 0.f};
  std::array<float, 4> rotation_xyzw{0.f, 0.f, 0.f, 1.f};
};
LeverArm FixedLeverArm(int32_t device_model);

// 1197 method 5: V' = V + Wdot x r + W x (W x r), then method 1.
ScaleFit ScaleFixedSpatial(const std::vector<Vec3>& v, const std::vector<Vec3>& imu,
                           const std::vector<Vec3>& omega, const std::vector<Vec3>& omega_dot,
                           const Vec3& lever_arm);

}  // namespace slam_refact::metric_scale

#endif  // SLAM_REFACT_METRIC_SCALE_ALIGNMENT_MATH_H_

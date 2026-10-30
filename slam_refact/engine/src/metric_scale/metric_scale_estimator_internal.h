// MetricScaleEstimator state (S5 §2.1, offsets relative to MSE = VioTracker+8). Module-private
// header (WP4, not frozen).

#ifndef SLAM_REFACT_METRIC_SCALE_METRIC_SCALE_ESTIMATOR_INTERNAL_H_
#define SLAM_REFACT_METRIC_SCALE_METRIC_SCALE_ESTIMATOR_INTERNAL_H_

#include <cstdint>
#include <limits>
#include <optional>

#include "metric_scale/alignment_math.h"
#include "metric_scale/kalman.h"
#include "metric_scale/metric_scale_estimator.h"

namespace slam_refact::metric_scale {

struct MetricScaleEstimator::Impl {
  explicit Impl(float movement_check_period_s)
      : lin_acc_mag(movement_check_period_s), gyro_mag(movement_check_period_s) {}

  // 1163: the filters, the gate state and the outputs go back to their defaults. The three time
  // bases, the two rolling windows and min_lin_acc are kept.
  void Reset();

  int64_t imu_time_base_ns = std::numeric_limits<int64_t>::max();    // +0
  int64_t frame_time_base_ns = std::numeric_limits<int64_t>::max();  // +8
  PositionKalman kf;                                                  // +16
  ImuBuffer imu;                                                      // +1744
  int64_t first_event_ever_ns = std::numeric_limits<int64_t>::max();  // +2720
  RollingWindowSum lin_acc_mag;                                       // +2728
  RollingWindowSum gyro_mag;                                          // +2768
  float min_lin_acc = std::numeric_limits<float>::max();              // +2808
  bool active = false;                                                // +2812
  float acc_sum = 0.f;                                                // +2816
  bool high_accel_seen = false;                                       // +2820
  bool have_estimate = false;                                         // +2821
  float scale = 1.f;                                                  // +2824
  float fit_residual = 0.f;                                           // +2828
};

// Steps (F) of pushFrame: smooth-vio-accel (§3.4), IMU extraction (§3.5), temporal alignment
// (§3.6) and the scale fit (§3.7). nullopt when the cropped series are empty.
// device_model selects the method-5 lever arm (1196).
std::optional<ScaleFit> EstimateScale(const PositionKalman& kf, const ImuBuffer& imu,
                                      int32_t device_model);

// Test access to the private state.
struct MetricScaleEstimatorAccess {
  static const MetricScaleEstimator::Impl& state(const MetricScaleEstimator& m) { return *m.impl_; }
};

}  // namespace slam_refact::metric_scale

#endif  // SLAM_REFACT_METRIC_SCALE_METRIC_SCALE_ESTIMATOR_INTERNAL_H_

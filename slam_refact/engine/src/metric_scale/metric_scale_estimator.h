// MetricScaleEstimator module public header (architecture.md §3.8; S5).
//
// FROZEN CONTRACT (WP0b): every declaration in this file. Changes follow
// slam_refact/docs/design/contract-changes.md. `private:` sections are owned by WP4.
//
// Not frozen here (listed by name only in the design; WP4 declares them in private headers):
// the pure alignment functions 1179-1200 and FftCompute.

#ifndef SLAM_REFACT_METRIC_SCALE_METRIC_SCALE_ESTIMATOR_H_
#define SLAM_REFACT_METRIC_SCALE_METRIC_SCALE_ESTIMATOR_H_

#include <cstdint>

#include "c8/hmatrix.h"
#include "common/tracking_types.h"

namespace slam_refact::metric_scale {

// 944 (this = VioTracker+8).
class MetricScaleEstimator {
 public:
  // MetricScaleEstimator.movementCheckPeriodInSec is read in the constructor only (S5 §4).
  explicit MetricScaleEstimator(float movement_check_period_s);
  ~MetricScaleEstimator();

  MetricScaleEstimator(const MetricScaleEstimator&) = delete;
  MetricScaleEstimator& operator=(const MetricScaleEstimator&) = delete;

  // S5 §3.1. tracker_state = MapTracker state (0 LOST, 1 TRACKING); reset_flag = map_reset.
  // Returns true when a scale is available.
  bool PushFrame(const TrackingSensorFrame& f, const c8::HMatrix& pose, int64_t time_ns,
                 int tracker_state, bool reset_flag);

  float scale() const;     // +2824
  float residual() const;  // +2828
  void Reset();            // 1163

 private:
  // Owned by WP4. The whole estimator state lives in Impl
  // (metric_scale/metric_scale_estimator_internal.h); MetricScaleEstimatorAccess gives the unit
  // tests read access to it.
  friend struct MetricScaleEstimatorAccess;
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace slam_refact::metric_scale

#endif  // SLAM_REFACT_METRIC_SCALE_METRIC_SCALE_ESTIMATOR_H_

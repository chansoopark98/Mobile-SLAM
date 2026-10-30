// MetricScaleEstimator building blocks (S5 §2.1-§2.3, §3.2-§3.5, §5.1-§5.3): RollingWindowSum
// (1168), the 9-state position Kalman filter (1169), the IMU buffer with the 6-state angular
// Kalman filter (1170) and the RTS smoother used by 944 and 1178. Module-private header (WP4).
//
// Matrices are stored as dynamic Eigen::MatrixXf. The original uses fixed-size matrices whose
// products have their own float order (S5 §7.2); the 9-state filter step and the 944 RTS smoother
// reproduce it explicitly (fixed_products.h). Members are public: the estimator owns these objects
// and the unit tests (P7, MsePrecision) read the filter state directly.

#ifndef SLAM_REFACT_METRIC_SCALE_KALMAN_H_
#define SLAM_REFACT_METRIC_SCALE_KALMAN_H_

#include <cstddef>
#include <cstdint>
#include <deque>
#include <utility>
#include <vector>

#include <Eigen/Dense>

#include "c8/hmatrix.h"
#include "common/tracking_types.h"
#include "metric_scale/alignment_math.h"

namespace slam_refact::metric_scale {

// One filtered step: posterior state, posterior covariance and the transition into this step.
struct KalmanState {
  Eigen::VectorXf x;
  Eigen::MatrixXf p;
  Eigen::MatrixXf f;
};

// RTS smoother over a copy of the filtered history (S5 §3.4). Returns smoothed (x, P) per step;
// the f member of the result is the input transition.
std::vector<KalmanState> RtsSmooth(const std::deque<KalmanState>& states, const Eigen::MatrixXf& q);

// The 9-state RTS smoother inlined in 944 (smooth-vio-accel) in the original's float order
// (fixed-size GEMMs 1173/1174, PartialPivLU inverse with TRSM 1175/1176, GEMV 1177).
std::vector<KalmanState> RtsSmoothPosition(const std::deque<KalmanState>& states,
                                           const Eigen::MatrixXf& q);

// 1168: rolling sum over a time window (S5 §5.1).
class RollingWindowSum {
 public:
  explicit RollingWindowSum(float window) : window_(window) {}
  void Push(float t, float v);
  // float(sum / size); NaN when empty.
  float Mean() const;
  size_t size() const { return q_.size(); }
  double sum() const { return sum_; }

 private:
  std::deque<std::pair<float, float>> q_;
  float window_;
  double sum_ = 0.0;
};

// 1169: position Kalman filter (p, v, a) on the VIO camera position plus its history.
struct PositionKalman {
  // Returns pred_n < N - skip (N = stored frames). The first call only initializes.
  bool Push(const c8::HMatrix& pose, int64_t base_ns, int64_t t_ns, int max_frames, int skip,
            int pred_n, bool ios_fps_scaling);

  float rho = 0.f;
  Eigen::VectorXf x = Eigen::VectorXf::Zero(9);
  Eigen::VectorXf x_pred = Eigen::VectorXf::Zero(9);
  Eigen::MatrixXf p = Eigen::MatrixXf::Zero(9, 9);
  Eigen::MatrixXf p_pred = Eigen::MatrixXf::Zero(9, 9);
  Eigen::MatrixXf h = Eigen::MatrixXf::Zero(3, 9);
  Eigen::MatrixXf f = Eigen::MatrixXf::Zero(9, 9);
  Eigen::MatrixXf q = Eigen::MatrixXf::Zero(9, 9);
  Eigen::MatrixXf r = Eigen::MatrixXf::Zero(3, 3);
  Eigen::MatrixXf k = Eigen::MatrixXf::Zero(9, 3);
  float last_t = -1.f;  // -1 = not initialized
  std::deque<float> frame_times;     // s since the frame time base
  std::deque<c8::HMatrix> poses;
  std::deque<KalmanState> states;
};

// 1170: IMU sample buffer and angular-rate Kalman filter (omega, alpha).
struct ImuBuffer {
  // Returns !lin_times.empty(). Events earlier than t_oldest (s) are skipped.
  bool Push(const std::vector<TrackingSensorEvent>& events, int64_t base_ns, float t_oldest,
            int max_frames);

  // Grow until reset (Q12): lin_*, gyro_values, all_event_times.
  std::deque<float> lin_times;
  std::deque<Vec3> lin_values;
  std::deque<float> gyro_times;  // popped together with states
  std::deque<Vec3> gyro_values;
  std::deque<float> all_event_times;
  float rho = 0.f;
  Eigen::VectorXf x = Eigen::VectorXf::Zero(6);
  Eigen::VectorXf x_pred = Eigen::VectorXf::Zero(6);
  Eigen::MatrixXf p = Eigen::MatrixXf::Zero(6, 6);
  Eigen::MatrixXf p_pred = Eigen::MatrixXf::Zero(6, 6);
  Eigen::MatrixXf h = Eigen::MatrixXf::Zero(3, 6);
  Eigen::MatrixXf f = Eigen::MatrixXf::Zero(6, 6);
  Eigen::MatrixXf q = Eigen::MatrixXf::Zero(6, 6);
  Eigen::MatrixXf r = Eigen::MatrixXf::Zero(3, 3);
  Eigen::MatrixXf k = Eigen::MatrixXf::Zero(6, 3);
  std::deque<KalmanState> states;
  float last_t = -1.f;
  int64_t interval_ns = 0;
};

}  // namespace slam_refact::metric_scale

#endif  // SLAM_REFACT_METRIC_SCALE_KALMAN_H_

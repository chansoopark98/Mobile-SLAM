#include "metric_scale/kalman.h"

#include <algorithm>
#include <cmath>
#include <limits>

#include "config/settings.h"
#include "metric_scale/fixed_products.h"

namespace slam_refact::metric_scale {
namespace {

using Eigen::MatrixXf;
using Eigen::VectorXf;

// Standard predict/update with the Joseph-form covariance update (S5 §3.2). S (3x3) is inverted
// with the fixed-size cofactor formula like the original. Used by the 6-state angular filter
// (1170), whose float order is not decoded; its state matches the original bit for bit on the
// MsePrecision golden (recorded MS-RT inputs) only.
void KalmanStep(const MatrixXf& f, const MatrixXf& q, const MatrixXf& h, const MatrixXf& r,
                const VectorXf& z, VectorXf* x, VectorXf* x_pred, MatrixXf* p, MatrixXf* p_pred,
                MatrixXf* k) {
  *x_pred = f * *x;
  *p_pred = f * *p * f.transpose() + q;
  const Eigen::Matrix3f s = h * *p_pred * h.transpose() + r;
  const MatrixXf s_inv = s.inverse();
  *k = *p_pred * h.transpose() * s_inv;
  *x = *x_pred + *k * (z - h * *x_pred);
  const MatrixXf ikh = MatrixXf::Identity(f.rows(), f.cols()) - *k * h;
  *p = ikh * *p_pred * ikh.transpose() + *k * r * k->transpose();
}

// The predict/update of 1169 in the original's float order (WP5b MSE precision; folded wat of
// func 1169, fixed-size 9-state matrices):
//   x_pred = F x                            GEMV 1177, k in order from +0
//   P_pred(r, c) = (F (F P)^T)(c, r) + Q    GEMM 1173 then 1174 (Gebp), transposed copy
//   PHt = P_pred H^T (9x3), HP = H P_pred   1174 / 1173 (Gebp)
//   S(i, j) = TreeSum_k HP(i, k) H(j, k) + R(i, j)        func 1205
//   S^-1                                    Eigen 3x3 cofactor inverse (as decoded, below)
//   K(r, c) = PHt(r,0) Si(0,c) + (PHt(r,1) Si(1,c) + PHt(r,2) Si(2,c))
//   y = z - TreeSum_k H(r, k) x_pred(k);   x(r) = x_pred(r) + (y0 K(r,0) + (y1 K(r,1) + y2 K(r,2)))
//   IKH = I - K H (1173, alpha -1);  T = IKH (IKH P_pred)^T + K (K R)^T (1173, 1174, 1174)
//   P(r, c) = T(c, r)
void PositionStep(const MatrixXf& f, const MatrixXf& q, const MatrixXf& h, const MatrixXf& r,
                  const VectorXf& z, VectorXf* x, VectorXf* x_pred, MatrixXf* p, MatrixXf* p_pred,
                  MatrixXf* k) {
  using fixed_products::Gebp;
  using fixed_products::TreeSum;
  constexpr int kN = 9;
  for (int i = 0; i < kN; ++i) {
    float acc = 0.0f;
    for (int j = 0; j < kN; ++j) {
      acc = acc + f(i, j) * (*x)(j);
    }
    (*x_pred)(i) = acc * 1.0f + 0.0f;
  }
  Eigen::Matrix<float, kN, kN> fp = Eigen::Matrix<float, kN, kN>::Zero();
  Gebp(kN, kN, kN, f, *p, 1.0f, fp.data(), kN);
  Eigen::Matrix<float, kN, kN> ffpt = Eigen::Matrix<float, kN, kN>::Zero();
  Gebp(kN, kN, kN, f, [&](int kk, int j) { return fp(j, kk); }, 1.0f, ffpt.data(), kN);
  for (int c = 0; c < kN; ++c) {
    for (int rr = 0; rr < kN; ++rr) {
      (*p_pred)(rr, c) = ffpt(c, rr) + q(rr, c);
    }
  }
  const MatrixXf& pp = *p_pred;
  Eigen::Matrix<float, kN, 3> pht = Eigen::Matrix<float, kN, 3>::Zero();
  Gebp(kN, 3, kN, pp, [&](int kk, int j) { return h(j, kk); }, 1.0f, pht.data(), kN);
  Eigen::Matrix<float, 3, kN> hp = Eigen::Matrix<float, 3, kN>::Zero();
  Gebp(3, kN, kN, h, pp, 1.0f, hp.data(), 3);
  float s[3][3];
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      s[i][j] = TreeSum(0, kN, [&](int kk) { return hp(i, kk) * h(j, kk); }) + r(i, j);
    }
  }
  // Cofactor inverse (Eigen compute_inverse_size3_helper; det = c00 s00 + (c20 s20 + c10 s10)).
  const float l0 = s[2][1] * s[0][2] - s[0][1] * s[2][2];
  const float l1 = s[0][1] * s[1][2] - s[1][1] * s[0][2];
  const float l2 = s[1][2] * s[2][0] - s[2][2] * s[1][0];
  const float l3 = s[2][2] * s[0][0] - s[0][2] * s[2][0];
  const float c00 = s[1][1] * s[2][2] - s[2][1] * s[1][2];
  const float inv_det = 1.0f / (s[0][0] * c00 + (l1 * s[2][0] + s[1][0] * l0));
  float si[3][3];
  si[0][0] = c00 * inv_det;
  si[1][0] = l2 * inv_det;
  si[2][0] = (s[1][0] * s[2][1] - s[2][0] * s[1][1]) * inv_det;
  si[0][1] = l0 * inv_det;
  si[1][1] = l3 * inv_det;
  si[2][1] = (s[2][0] * s[0][1] - s[0][0] * s[2][1]) * inv_det;
  si[0][2] = l1 * inv_det;
  si[1][2] = (s[0][2] * s[1][0] - s[1][2] * s[0][0]) * inv_det;
  si[2][2] = (s[0][0] * s[1][1] - s[1][0] * s[0][1]) * inv_det;
  for (int c = 0; c < 3; ++c) {
    for (int rr = 0; rr < kN; ++rr) {
      (*k)(rr, c) = pht(rr, 0) * si[0][c] + (pht(rr, 1) * si[1][c] + pht(rr, 2) * si[2][c]);
    }
  }
  float y[3];
  for (int i = 0; i < 3; ++i) {
    y[i] = z(i) - TreeSum(0, kN, [&](int kk) { return h(i, kk) * (*x_pred)(kk); });
  }
  const MatrixXf& kg = *k;
  for (int i = 0; i < kN; ++i) {
    (*x)(i) = (*x_pred)(i) + (y[0] * kg(i, 0) + (y[1] * kg(i, 1) + y[2] * kg(i, 2)));
  }
  Eigen::Matrix<float, kN, kN> ikh = Eigen::Matrix<float, kN, kN>::Identity();
  Gebp(kN, kN, 3, kg, h, -1.0f, ikh.data(), kN);
  Eigen::Matrix<float, kN, kN> ikh_pp = Eigen::Matrix<float, kN, kN>::Zero();
  Gebp(kN, kN, kN, ikh, pp, 1.0f, ikh_pp.data(), kN);
  Eigen::Matrix<float, kN, kN> t = Eigen::Matrix<float, kN, kN>::Zero();
  Gebp(kN, kN, kN, ikh, [&](int kk, int j) { return ikh_pp(j, kk); }, 1.0f, t.data(), kN);
  Eigen::Matrix<float, kN, 3> kr;
  for (int c = 0; c < 3; ++c) {
    for (int rr = 0; rr < kN; ++rr) {
      kr(rr, c) = kg(rr, 0) * r(0, c) + (kg(rr, 1) * r(1, c) + kg(rr, 2) * r(2, c));
    }
  }
  Gebp(kN, kN, 3, kg, [&](int kk, int j) { return kr(j, kk); }, 1.0f, t.data(), kN);
  for (int c = 0; c < kN; ++c) {
    for (int rr = 0; rr < kN; ++rr) {
      (*p)(rr, c) = t(c, rr);
    }
  }
}

// 1201 static initializer: kAccelVar = sigma_a^2 with the binary default (read before any
// parameter override can run).
float AccelVarAtStaticInit() {
  const float sigma = config::AlignmentSettings{}.kalman_position_process_noise_accel_std_dev;
  return sigma * sigma;
}

}  // namespace

namespace {

using Mat9 = Eigen::Matrix<float, 9, 9>;

// Eigen PartialPivLU<Matrix<float, 9, 9>>::inverse() of 944 in the original's order: unblocked
// LU (first largest |pivot|, full row swap, column / pivot, a(i,j) - u(k,j) l(i,k)), then
// P I solved by TRSM 1175 (unit lower) and 1176 (upper) with panels of 4 rows: inside a panel
// each row update is r = (-b) t + r; the rows outside the panel get r = acc * -1 + r once per
// panel with acc = 0 + t0 b0 + t1 b1 + ... over the panel rows in increasing order (gebp 1236,
// depth <= 4). The upper solve scales by a = 1 / u(i,i) (b = a * x) and walks the panels upward.
Mat9 InverseLu(const Mat9& a) {
  constexpr int kN = 9;
  constexpr int kPanel = 4;
  Mat9 lu = a;
  int transpositions[kN];
  for (int k = 0; k < kN; ++k) {
    int best = 0;
    float biggest = std::fabs(lu(k, k));
    for (int i = 1; i < kN - k; ++i) {
      const float v = std::fabs(lu(k + i, k));
      if (v > biggest) {
        biggest = v;
        best = i;
      }
    }
    transpositions[k] = k + best;
    if (biggest != 0.0f) {
      if (best != 0) {
        lu.row(k).swap(lu.row(k + best));
      }
      const float pivot = lu(k, k);
      for (int i = k + 1; i < kN; ++i) {
        lu(i, k) = lu(i, k) / pivot;
      }
    }
    for (int j = k + 1; j < kN; ++j) {
      const float u = lu(k, j);
      for (int i = k + 1; i < kN; ++i) {
        lu(i, j) = lu(i, j) - u * lu(i, k);
      }
    }
  }
  int perm[kN];
  for (int i = 0; i < kN; ++i) {
    perm[i] = i;
  }
  for (int k = kN - 1; k >= 0; --k) {
    std::swap(perm[k], perm[transpositions[k]]);
  }
  Mat9 b = Mat9::Zero();
  for (int i = 0; i < kN; ++i) {
    b(perm[i], i) = 1.0f;
  }
  for (int j = 0; j < kN; ++j) {
    for (int p0 = 0; p0 < kN; p0 += kPanel) {  // L (unit diagonal), top-down
      const int w = std::min(kPanel, kN - p0);
      for (int i = p0; i < p0 + w; ++i) {
        const float nb = -b(i, j);
        for (int r = i + 1; r < p0 + w; ++r) {
          b(r, j) = nb * lu(r, i) + b(r, j);
        }
      }
      for (int r = p0 + w; r < kN; ++r) {
        float acc = 0.0f;
        for (int kk = p0; kk < p0 + w; ++kk) {
          acc = acc + lu(r, kk) * b(kk, j);
        }
        b(r, j) = acc * -1.0f + b(r, j);
      }
    }
    for (int k1 = 0; k1 < kN; k1 += kPanel) {  // U, bottom-up
      const int w = std::min(kPanel, kN - k1);
      const int top = kN - k1 - w;
      for (int i = kN - k1 - 1; i >= top; --i) {
        const float s = 1.0f / lu(i, i);
        const float v = s * b(i, j);
        b(i, j) = v;
        const float nv = -v;
        for (int r = top; r < i; ++r) {
          b(r, j) = nv * lu(r, i) + b(r, j);
        }
      }
      for (int r = 0; r < top; ++r) {
        float acc = 0.0f;
        for (int kk = top; kk < top + w; ++kk) {
          acc = acc + lu(r, kk) * b(kk, j);
        }
        b(r, j) = acc * -1.0f + b(r, j);
      }
    }
  }
  return b;
}

}  // namespace

std::vector<KalmanState> RtsSmoothPosition(const std::deque<KalmanState>& states,
                                           const MatrixXf& q) {
  using fixed_products::Gebp;
  constexpr int kN = 9;
  std::vector<KalmanState> sm(states.begin(), states.end());
  for (int k = static_cast<int>(sm.size()) - 2; k >= 0; --k) {
    const MatrixXf& f1 = states[k + 1].f;
    const MatrixXf& pk = sm[k].p;
    Mat9 fp = Mat9::Zero();
    Gebp(kN, kN, kN, f1, pk, 1.0f, fp.data(), kN);
    Mat9 ffpt = Mat9::Zero();
    Gebp(kN, kN, kN, f1, [&](int kk, int j) { return fp(j, kk); }, 1.0f, ffpt.data(), kN);
    Mat9 p_pred;
    for (int c = 0; c < kN; ++c) {
      for (int r = 0; r < kN; ++r) {
        p_pred(r, c) = ffpt(c, r) + q(r, c);
      }
    }
    Mat9 pft = Mat9::Zero();
    Gebp(kN, kN, kN, pk, [&](int kk, int j) { return f1(j, kk); }, 1.0f, pft.data(), kN);
    const Mat9 inv = InverseLu(p_pred);
    Mat9 g = Mat9::Zero();
    Gebp(kN, kN, kN, pft, inv, 1.0f, g.data(), kN);
    // GEMV 1177: dx = x_{k+1} + (-1) (F x_k); x_k += (G dx + 0).
    float dx[kN];
    for (int i = 0; i < kN; ++i) {
      float acc = 0.0f;
      for (int j = 0; j < kN; ++j) {
        acc = acc + f1(i, j) * sm[k].x(j);
      }
      dx[i] = acc * -1.0f + sm[k + 1].x(i);
    }
    float gdx[kN];
    for (int i = 0; i < kN; ++i) {
      float acc = 0.0f;
      for (int j = 0; j < kN; ++j) {
        acc = acc + g(i, j) * dx[j];
      }
      gdx[i] = acc * 1.0f + 0.0f;
    }
    for (int i = 0; i < kN; ++i) {
      sm[k].x(i) = gdx[i] + sm[k].x(i);
    }
    Mat9 dp;
    for (int c = 0; c < kN; ++c) {
      for (int r = 0; r < kN; ++r) {
        dp(r, c) = sm[k + 1].p(r, c) - p_pred(r, c);
      }
    }
    Mat9 gdp = Mat9::Zero();
    Gebp(kN, kN, kN, g, dp, 1.0f, gdp.data(), kN);
    Mat9 t = Mat9::Zero();
    Gebp(kN, kN, kN, g, [&](int kk, int j) { return gdp(j, kk); }, 1.0f, t.data(), kN);
    for (int c = 0; c < kN; ++c) {
      for (int r = 0; r < kN; ++r) {
        sm[k].p(r, c) = t(c, r) + sm[k].p(r, c);
      }
    }
  }
  return sm;
}

std::vector<KalmanState> RtsSmooth(const std::deque<KalmanState>& states, const MatrixXf& q) {
  std::vector<KalmanState> sm(states.begin(), states.end());
  for (int k = static_cast<int>(sm.size()) - 2; k >= 0; --k) {
    const MatrixXf& f1 = states[k + 1].f;
    const MatrixXf p_pred = f1 * sm[k].p * f1.transpose() + q;
    const MatrixXf g = (sm[k].p * f1.transpose()) * p_pred.inverse();  // PartialPivLU
    sm[k].x += g * (sm[k + 1].x - f1 * sm[k].x);
    sm[k].p += g * (sm[k + 1].p - p_pred) * g.transpose();
  }
  return sm;
}

void RollingWindowSum::Push(float t, float v) {
  q_.emplace_back(t, v);
  sum_ += static_cast<double>(v);
  while (q_.size() >= 2 && q_.back().first - q_.front().first >= window_) {
    sum_ -= static_cast<double>(q_.front().second);
    q_.pop_front();
  }
}

float RollingWindowSum::Mean() const {
  if (q_.empty()) {
    return std::numeric_limits<float>::quiet_NaN();
  }
  return static_cast<float>(sum_ / static_cast<double>(q_.size()));
}

bool PositionKalman::Push(const c8::HMatrix& pose, int64_t base_ns, int64_t t_ns, int max_frames,
                          int skip, int pred_n, bool ios_fps_scaling) {
  const double t = static_cast<double>(t_ns - base_ns) * static_cast<double>(1e-9f);
  VectorXf z(3);
  z << pose.data()[12], pose.data()[13], pose.data()[14];
  if (last_t == -1.0f) {  // init frame: nothing is stored
    const config::AlignmentSettings& s = config::AlignmentSettings::Get();
    last_t = static_cast<float>(t);
    x = VectorXf::Zero(9);
    x.head<3>() = z;
    x_pred = x;
    p = MatrixXf::Identity(9, 9);
    p_pred = p;
    const float vp = s.kalman_position_process_noise_pos_std_dev * s.kalman_position_process_noise_pos_std_dev;
    const float vv = s.kalman_position_process_noise_vel_std_dev * s.kalman_position_process_noise_vel_std_dev;
    const float va =
        s.kalman_position_process_noise_accel_std_dev * s.kalman_position_process_noise_accel_std_dev;
    const float vm =
        s.kalman_position_measurement_noise_std_dev * s.kalman_position_measurement_noise_std_dev;
    q = MatrixXf::Zero(9, 9);
    for (int i = 0; i < 3; ++i) {
      q(i, i) = vp;
      q(3 + i, 3 + i) = vv;
      q(6 + i, 6 + i) = va;
    }
    r = MatrixXf::Identity(3, 3) * vm;
    h = MatrixXf::Zero(3, 9);
    h.leftCols(3) = MatrixXf::Identity(3, 3);
    rho = s.kalman_position_accel_transition;
    return false;
  }
  const double dt_d = t - static_cast<double>(last_t);
  if (ios_fps_scaling) {
    static const float kAccelVar = AccelVarAtStaticInit();
    const float fps_alpha = config::AlignmentSettings::Get().fps_alpha;
    const float qa = static_cast<float>(std::pow(1.0 / dt_d, -1.5) * static_cast<double>(fps_alpha) *
                                        static_cast<double>(kAccelVar));
    for (int i = 6; i < 9; ++i) {
      q(i, i) = qa;
    }
  }
  const float dt = static_cast<float>(dt_d);
  f = MatrixXf::Identity(9, 9);
  for (int i = 0; i < 3; ++i) {
    f(i, 3 + i) = dt;
    f(3 + i, 6 + i) = dt;
    f(i, 6 + i) = (dt * 0.5f) * dt;
    f(6 + i, 6 + i) = rho;
  }
  PositionStep(f, q, h, r, z, &x, &x_pred, &p, &p_pred, &k);
  states.push_back({x, p, f});
  frame_times.push_back(static_cast<float>(t));
  poses.push_back(pose);
  last_t = static_cast<float>(t);
  if (static_cast<int>(frame_times.size()) > max_frames) {
    states.pop_front();
    frame_times.pop_front();
    poses.pop_front();
  }
  return pred_n < static_cast<int>(frame_times.size()) - skip;
}

bool ImuBuffer::Push(const std::vector<TrackingSensorEvent>& events, int64_t base_ns, float t_oldest,
                     int max_frames) {
  constexpr double kPi = 3.141592653589793;
  for (const TrackingSensorEvent& e : events) {
    const double t = static_cast<double>(e.event_time_ns - base_ns) * static_cast<double>(1e-9f);
    if (t < static_cast<double>(t_oldest)) {
      continue;
    }
    all_event_times.push_back(static_cast<float>(t));
    interval_ns = e.interval_ns;
    if (e.kind == ImuEvent::Kind::kLinearAcceleration) {
      lin_values.push_back({e.x, e.y, e.z});
      lin_times.push_back(static_cast<float>(t));
    } else if (e.kind == ImuEvent::Kind::kGyroscope) {
      if (last_t == -1.0f) {  // init: x = 0 (the measurement is not used), P = I
        const config::AlignmentSettings& s = config::AlignmentSettings::Get();
        last_t = static_cast<float>(t);
        x = VectorXf::Zero(6);
        x_pred = x;
        p = MatrixXf::Identity(6, 6);
        p_pred = p;
        const float vv =
            s.kalman_angular_process_noise_vel_std_dev * s.kalman_angular_process_noise_vel_std_dev;
        const float va =
            s.kalman_angular_process_noise_accel_std_dev * s.kalman_angular_process_noise_accel_std_dev;
        const float vm =
            s.kalman_angular_measurement_noise_std_dev * s.kalman_angular_measurement_noise_std_dev;
        q = MatrixXf::Zero(6, 6);
        for (int i = 0; i < 3; ++i) {
          q(i, i) = vv;
          q(3 + i, 3 + i) = va;
        }
        r = MatrixXf::Identity(3, 3) * vm;
        h = MatrixXf::Zero(3, 6);
        h.leftCols(3) = MatrixXf::Identity(3, 3);
        rho = s.kalman_angular_accel_transition;
        continue;
      }
      gyro_values.push_back({e.x, e.y, e.z});
      gyro_times.push_back(static_cast<float>(t));
      VectorXf z(3);
      z << static_cast<float>(static_cast<double>(e.x) * kPi / 180.0),
          static_cast<float>(static_cast<double>(e.y) * kPi / 180.0),
          static_cast<float>(static_cast<double>(e.z) * kPi / 180.0);
      const float dt = static_cast<float>(t - static_cast<double>(last_t));
      f = MatrixXf::Identity(6, 6);
      for (int i = 0; i < 3; ++i) {
        f(i, 3 + i) = dt;
        f(3 + i, 3 + i) = rho;
      }
      KalmanStep(f, q, h, r, z, &x, &x_pred, &p, &p_pred, &k);
      states.push_back({x, p, f});
      last_t = static_cast<float>(t);
      if (static_cast<int>(states.size()) > max_frames) {
        states.pop_front();
        gyro_times.pop_front();
      }
    }
  }
  return !lin_times.empty();
}

}  // namespace slam_refact::metric_scale

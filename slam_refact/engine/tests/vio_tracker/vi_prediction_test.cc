// Accuracy mode ACC-5: ViPredictor (tracking-stage visual-inertial coupling) on synthetic motion.

#include "vio_tracker/vi_prediction.h"

#include <cmath>
#include <vector>

#include "c8/geometry/egomotion.h"
#include "c8/hpoint.h"
#include "gtest/gtest.h"

namespace slam_refact::vio_tracker {
namespace {

constexpr double kG = 9.80665;
constexpr int64_t kFrameNs = 33333333;

FrameState StateAt(int64_t t_ns, float x, float y, float z) {
  FrameState s;
  s.pose = c8::cameraMotion(c8::HPoint3(x, y, z), c8::Quaternion());
  s.time_ns = t_ns;
  return s;
}

TrackingSensorEvent Accel(int64_t t_ns, double ax, double ay, double az) {
  TrackingSensorEvent e;
  e.kind = TrackingSensorEvent::Kind::kAccelerometer;
  e.event_time_ns = t_ns;
  e.interval_ns = 16666667;
  e.x = static_cast<float>(ax);
  e.y = static_cast<float>(ay);
  e.z = static_cast<float>(az);
  return e;
}

// Level camera, identity device rotation: the specific force (0, g, a) is a forward acceleration a.
TEST(ViPredictor, IntegratesConstantAcceleration) {
  constexpr double kMpu = 0.5;  // metres per internal unit
  constexpr double kA = 1.0;    // m/s^2 along +z (forward)
  ViPredictor vi;
  FrameState f0 = StateAt(0, 0.f, 0.f, 0.f);
  FrameState pred = StateAt(kFrameNs, 0.f, 0.f, 0.f);
  // Not initialised: no prediction, but the accelerometer sample is kept.
  EXPECT_FALSE(vi.Predict(f0, kFrameNs, c8::Quaternion(), {Accel(kFrameNs / 2, 0, kG, kA)}, kMpu, &pred));
  FrameState f1 = StateAt(kFrameNs, 0.f, 0.f, 0.f);
  vi.Update(f0, f1.pose, kFrameNs, kMpu, 0.2, false);  // visual velocity 0
  ASSERT_TRUE(vi.initialized());

  pred = StateAt(2 * kFrameNs, 0.f, 0.f, 0.f);
  const std::vector<TrackingSensorEvent> ev = {Accel(kFrameNs + 10000000, 0, kG, kA),
                                               Accel(kFrameNs + 26666667, 0, kG, kA)};
  ASSERT_TRUE(vi.Predict(f1, 2 * kFrameNs, c8::Quaternion(), ev, kMpu, &pred));
  const double dt = kFrameNs * 1e-9;
  EXPECT_NEAR(pred.pose(2, 3), 0.5 * kA * dt * dt / kMpu, 2e-6);
  EXPECT_NEAR(pred.pose(0, 3), 0.0, 2e-6);
  EXPECT_NEAR(pred.pose(1, 3), 0.0, 2e-6);  // gravity removed
  EXPECT_LT(vi.gravity_angle_deg(), 7.0);   // atan(1/g) = 5.8 deg with the forward push
}

TEST(ViPredictor, NoPredictionWithoutScaleHint) {
  ViPredictor vi;
  FrameState f0 = StateAt(0, 0.f, 0.f, 0.f);
  FrameState f1 = StateAt(kFrameNs, 0.f, 0.f, 0.f);
  vi.Update(f0, f1.pose, kFrameNs, 0.0, 0.2, false);  // missing hint restarts
  EXPECT_FALSE(vi.initialized());
  FrameState pred = StateAt(2 * kFrameNs, 1.f, 2.f, 3.f);
  EXPECT_FALSE(vi.Predict(f1, 2 * kFrameNs, c8::Quaternion(), {Accel(kFrameNs + 1, 0, kG, 0)}, 0.0, &pred));
  EXPECT_FLOAT_EQ(pred.pose(0, 3), 1.f);  // original prediction untouched
}

TEST(ViPredictor, DiscontinuityAndGapRestart) {
  constexpr double kMpu = 0.5;
  ViPredictor vi;
  FrameState f0 = StateAt(0, 0.f, 0.f, 0.f);
  FrameState f1 = StateAt(kFrameNs, 0.1f, 0.f, 0.f);
  vi.Update(f0, f1.pose, kFrameNs, kMpu, 0.2, false);
  ASSERT_TRUE(vi.initialized());
  vi.Update(f0, f1.pose, kFrameNs, kMpu, 0.2, true);  // map reset / world transform update
  EXPECT_FALSE(vi.initialized());
  vi.Update(f0, f1.pose, kFrameNs, kMpu, 0.2, false);
  ASSERT_TRUE(vi.initialized());
  vi.Update(f0, f1.pose, 400000000, kMpu, 0.2, false);  // 0.4 s gap
  EXPECT_FALSE(vi.initialized());
}

// The visual velocity seeds the filter: with zero acceleration the prediction continues it.
TEST(ViPredictor, ContinuesVisualVelocity) {
  constexpr double kMpu = 0.25;
  ViPredictor vi;
  FrameState f0 = StateAt(0, 0.f, 0.f, 0.f);
  FrameState f1 = StateAt(kFrameNs, 0.f, 0.f, -0.02f);  // 0.15 m/s backwards
  FrameState pred = f1;
  vi.Predict(f0, kFrameNs, c8::Quaternion(), {Accel(kFrameNs / 2, 0, kG, 0)}, kMpu, &pred);
  vi.Update(f0, f1.pose, kFrameNs, kMpu, 0.2, false);
  pred = StateAt(2 * kFrameNs, 0.f, 0.f, 0.f);
  ASSERT_TRUE(vi.Predict(f1, 2 * kFrameNs, c8::Quaternion(), {Accel(kFrameNs + 5, 0, kG, 0)}, kMpu, &pred));
  EXPECT_NEAR(pred.pose(2, 3), -0.04f, 1e-5);
}

c8::HMatrix RotationOf(const c8::Quaternion& q) { return q.toRotationMat(); }

std::array<double, 3> Apply(const c8::HMatrix& r, double x, double y, double z) {
  const c8::HPoint3 p = r * c8::HPoint3(static_cast<float>(x), static_cast<float>(y), static_cast<float>(z));
  return {p.x(), p.y(), p.z()};
}

// Camera pitched 20 deg down, at rest: the accelerometer reads gravity in the camera frame
// (R^T * (0, g, 0)); the prediction must not move and the specific force must point up.
TEST(ViPredictor, TiltedCameraAtRestStaysPut) {
  constexpr double kMpu = 0.5;
  const c8::Quaternion tilt = c8::Quaternion::xDegrees(-20.f);
  const std::array<double, 3> a = Apply(RotationOf(tilt.inverse()), 0.0, kG, 0.0);
  ViPredictor vi;
  FrameState f0;
  f0.pose = c8::cameraMotion(c8::HPoint3(0.f, 1.f, 0.f), tilt);
  f0.time_ns = 0;
  FrameState f1 = f0;
  f1.time_ns = kFrameNs;
  FrameState pred = f1;
  vi.Predict(f0, kFrameNs, c8::Quaternion(), {Accel(kFrameNs / 2, a[0], a[1], a[2])}, kMpu, &pred);
  vi.Update(f0, f1.pose, kFrameNs, kMpu, 0.2, false);
  pred = f1;
  pred.time_ns = 2 * kFrameNs;
  ASSERT_TRUE(vi.Predict(f1, 2 * kFrameNs, c8::Quaternion(), {Accel(kFrameNs + 9000000, a[0], a[1], a[2])}, kMpu, &pred));
  EXPECT_NEAR(pred.pose(0, 3), 0.0, 1e-5);
  EXPECT_NEAR(pred.pose(1, 3), 1.0, 1e-5);
  EXPECT_NEAR(pred.pose(2, 3), 0.0, 1e-5);
  EXPECT_LT(vi.gravity_angle_deg(), 0.05);
}

// The device turns 90 deg about +y during the frame while pushing forward (camera +z) at kA: the
// world acceleration turns with it (R(t) = R_pose(prev) * slerp(I, q_rel, u)), so the displacement
// is the double integral of R_y(theta(t)) * (0, 0, kA), theta linear in time.
TEST(ViPredictor, AccelerationFollowsTheRotationWithinTheFrame) {
  constexpr double kMpu = 1.0;
  constexpr double kA = 2.0;
  const c8::Quaternion turn = c8::Quaternion::yDegrees(90.f);
  ViPredictor vi;
  FrameState f0 = StateAt(0, 0.f, 0.f, 0.f);
  FrameState f1 = StateAt(kFrameNs, 0.f, 0.f, 0.f);
  FrameState pred = f1;
  vi.Predict(f0, kFrameNs, c8::Quaternion(), {Accel(kFrameNs - 1, 0, kG, kA)}, kMpu, &pred);
  vi.Update(f0, f1.pose, kFrameNs, kMpu, 0.2, false);  // v = 0
  // 64 samples over the frame (the first one at its start), constant camera-frame acceleration.
  std::vector<TrackingSensorEvent> ev;
  for (int k = 0; k < 64; ++k) ev.push_back(Accel(kFrameNs + k * (kFrameNs / 64), 0, kG, kA));
  pred = StateAt(2 * kFrameNs, 0.f, 0.f, 0.f);
  ASSERT_TRUE(vi.Predict(f1, 2 * kFrameNs, turn, ev, kMpu, &pred));
  // Analytic, with d0 = +z (start direction of camera +z) and d1 = camera +z after the full turn
  // (perpendicular to d0 in the x-z plane): p = kA T^2 * (d0 (1 - cos k) + d1 (k - sin k)) / k^2,
  // k = pi/2.
  const double t = kFrameNs * 1e-9, k = 0.5 * 3.14159265358979323846;
  const std::array<double, 3> end_z = Apply(RotationOf(turn), 0.0, 0.0, 1.0);  // camera +z after the turn
  const double side = kA * t * t * (k - std::sin(k)) / (k * k);
  const double fwd = kA * t * t * (1.0 - std::cos(k)) / (k * k);
  EXPECT_NEAR(pred.pose(0, 3), side * end_z[0], 0.03 * fwd);
  EXPECT_NEAR(pred.pose(2, 3), fwd + side * end_z[2], 0.03 * fwd);
  EXPECT_NEAR(pred.pose(1, 3), 0.0, 1e-6);
  EXPECT_NEAR(std::fabs(end_z[0]), 1.0, 1e-5);  // the turn really is 90 deg about y
}

}  // namespace
}  // namespace slam_refact::vio_tracker

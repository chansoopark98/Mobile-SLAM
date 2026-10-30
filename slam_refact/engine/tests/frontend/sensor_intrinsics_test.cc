// WP1 sensor intake (S1 §3.1, §3.6; D-14) and K rule (F-K, S1 §5.12) tests, plus F-H8 on the
// checked-in capture excerpt.

#include <cmath>
#include <cstring>
#include <string>
#include <cstdint>
#include <limits>
#include <type_traits>
#include <vector>

#include "c8/quaternion.h"
#include "frontend/frontend.h"
#include "frontend/intrinsics.h"
#include "frontend/sensor_intake.h"
#include "gtest/gtest.h"
#include "test_util.h"

namespace slam_refact::frontend {
namespace {

using testing::Json;

constexpr double kPi = 3.14159265358979323846;

// ---- D-14 compile-time guard: func 647 cannot be applied to post-647 events ----
static_assert(!std::is_invocable_v<decltype(&PrepareTrackingSensorEvents), std::vector<ImuEvent>,
                                   bool>,
              "PrepareTrackingSensorEvents must not accept ImuEvent (D-14)");
static_assert(!std::is_convertible_v<ImuEvent, RawSensorEvent>,
              "no ImuEvent -> RawSensorEvent conversion may exist (D-14)");

DeviceMotionSample Motion(int64_t t, std::optional<Vec3f> lin, std::optional<Vec3f> acc,
                          std::optional<Vec3f> gyro, double interval_ms = 16.0) {
  DeviceMotionSample s;
  s.receive_time_ns = t;
  s.acceleration = lin;
  s.acceleration_including_gravity = acc;
  s.rotation_rate_abg = gyro;
  s.interval_ms = interval_ms;
  return s;
}

TEST(SensorIntake, AndroidSignsKindsIntervalAndOrder) {
  SensorIntake intake(/*is_ios=*/false);
  intake.OnDeviceMotion(Motion(1000, Vec3f{0.1f, 0.2f, 0.3f}, Vec3f{1.f, 2.f, 9.8f},
                               Vec3f{10.f, 20.f, 30.f}));
  const StagedSensors staged = intake.TakeForStage();
  ASSERT_EQ(staged.events.size(), 3u);
  // Sorted by (time, post-647 kind): ACCEL(1), GYRO(3), LINEAR_ACCEL(4).
  EXPECT_EQ(staged.events[0].kind, ImuEvent::Kind::kAccelerometer);
  EXPECT_EQ(staged.events[1].kind, ImuEvent::Kind::kGyroscope);
  EXPECT_EQ(staged.events[2].kind, ImuEvent::Kind::kLinearAcceleration);
  for (const ImuEvent& e : staged.events) {
    EXPECT_EQ(e.event_time_ns, 1000);
    EXPECT_EQ(e.interval_ns, 16666667);  // Android 16000000 -> 16666667
  }
  // Android: accelerations z = -z; gyro (x, y) = (-alpha, -beta), z = gamma.
  EXPECT_EQ(staged.events[0].x, 1.f);
  EXPECT_EQ(staged.events[0].y, 2.f);
  EXPECT_EQ(staged.events[0].z, -9.8f);
  EXPECT_EQ(staged.events[1].x, -10.f);
  EXPECT_EQ(staged.events[1].y, -20.f);
  EXPECT_EQ(staged.events[1].z, 30.f);
  EXPECT_EQ(staged.events[2].x, 0.1f);
  EXPECT_EQ(staged.events[2].y, 0.2f);
  EXPECT_EQ(staged.events[2].z, -0.3f);
  ValidatePostPrepareEvents(staged.events);
}

TEST(SensorIntake, IosSignsAndInterval) {
  SensorIntake intake(/*is_ios=*/true);
  intake.OnDeviceMotion(Motion(5, std::nullopt, Vec3f{1.f, 2.f, 3.f}, Vec3f{4.f, 5.f, 6.f}, 16.0));
  const StagedSensors staged = intake.TakeForStage();
  ASSERT_EQ(staged.events.size(), 2u);
  EXPECT_EQ(staged.events[0].kind, ImuEvent::Kind::kAccelerometer);
  EXPECT_EQ(staged.events[0].x, -1.f);
  EXPECT_EQ(staged.events[0].y, -2.f);
  EXPECT_EQ(staged.events[0].z, 3.f);
  EXPECT_EQ(staged.events[0].interval_ns, 16000000LL * 1000);  // iOS: interval * 1000
  EXPECT_EQ(staged.events[1].kind, ImuEvent::Kind::kGyroscope);
  EXPECT_EQ(staged.events[1].x, -4.f);
  EXPECT_EQ(staged.events[1].y, -5.f);
  EXPECT_EQ(staged.events[1].z, 6.f);
}

TEST(SensorIntake, LinearAccelerationOnlyWhenPresent) {
  SensorIntake intake(false);
  intake.OnDeviceMotion(Motion(1, std::nullopt, Vec3f{0.f, 0.f, 9.8f}, std::nullopt));
  const StagedSensors staged = intake.TakeForStage();
  ASSERT_EQ(staged.events.size(), 1u);
  EXPECT_EQ(staged.events[0].kind, ImuEvent::Kind::kAccelerometer);
}

TEST(SensorIntake, IntervalConversionTruncatesLikeCapnpInt64) {
  EXPECT_EQ(IntervalMsToNanos(16.0), 16000000);
  EXPECT_EQ(IntervalMsToNanos(16.67), 16670000);        // 16670000.000000002 -> trunc
  EXPECT_EQ(IntervalMsToNanos(0.0000009), 0);           // 0.9 ns -> 0
  EXPECT_EQ(IntervalMsToNanos(16.6666666), 16666666);   // fraction dropped
  EXPECT_EQ(IntervalMsToNanos(-1.5e-6), -1);            // magnitude truncated, then signed
  SensorIntake intake(false);
  intake.OnDeviceMotion(Motion(1, std::nullopt, Vec3f{}, std::nullopt, 16.67));
  EXPECT_EQ(intake.TakeForStage().events[0].interval_ns, 16670000);
}

// capnp-ts Int64.setValue edge cases (S2 review LOW-3, VERIFIED from the deployed bundle):
// NaN -> 0, [2^63, 2^64] wraps modulo 2^64, beyond 2^64 / Infinity throws (abort here).
TEST(SensorIntake, IntervalConversionNonFiniteAndHugeLikeCapnpInt64) {
  EXPECT_EQ(IntervalMsToNanos(std::numeric_limits<double>::quiet_NaN()), 0);
  EXPECT_EQ(IntervalMsToNanos(-std::numeric_limits<double>::quiet_NaN()), 0);
  // 1e13 ms -> 1e19 ns (exact in double), 2^63 < 1e19 < 2^64: low 64 bits, as int64.
  EXPECT_EQ(IntervalMsToNanos(1e13), static_cast<int64_t>(10000000000000000000ULL));
  EXPECT_EQ(IntervalMsToNanos(-1e13), -static_cast<int64_t>(10000000000000000000ULL));
  // Below 2^63 the result is plain truncation toward zero.
  EXPECT_EQ(IntervalMsToNanos(9e12), 9000000000000000000LL);
}

TEST(SensorIntakeDeathTest, IntervalOutsideInt64RangeAborts) {
  EXPECT_DEATH(IntervalMsToNanos(2e13), "outside Int64 range");  // 2e19 > 2^64
  EXPECT_DEATH(IntervalMsToNanos(std::numeric_limits<double>::infinity()), "outside Int64 range");
  EXPECT_DEATH(IntervalMsToNanos(-std::numeric_limits<double>::infinity()), "outside Int64 range");
}

// func 647 sorts with TrackingSensorEvent::operator<, which is not a strict weak order once a
// component is NaN. The engine orders NaN after every number (S2 review LOW-3, INFERRED rule) and
// keeps the original order for finite values.
TEST(SensorIntake, PrepareSortsNaNComponentsLastAndDeterministically) {
  const float nan = std::numeric_limits<float>::quiet_NaN();
  std::vector<RawSensorEvent> queue;
  const float xs[] = {2.0f, nan, -1.0f, nan, 0.5f, 1.0f, -0.0f, 0.0f};
  for (size_t i = 0; i < sizeof(xs) / sizeof(xs[0]); ++i) {
    RawSensorEvent e;
    e.kind = RawSensorKind::kAccelerometer;
    e.timestamp_ns = 1000;
    e.interval_ns = static_cast<int64_t>(i + 1);  // identifies the input event
    e.x = xs[i];
    queue.push_back(e);
  }
  const std::vector<ImuEvent> a = PrepareTrackingSensorEvents(queue, false);
  const std::vector<ImuEvent> b = PrepareTrackingSensorEvents(queue, false);
  ASSERT_EQ(a.size(), queue.size());
  // A NaN x spreads to y and z through the identity sensor rotation (0 * NaN).
  const float finite_x[] = {-1.0f, 0.0f, 0.0f, 0.5f, 1.0f, 2.0f};
  for (size_t i = 0; i < 6; ++i) {
    EXPECT_EQ(a[i].x, finite_x[i]) << i;
    EXPECT_FALSE(std::isnan(a[i].y)) << i;
  }
  EXPECT_TRUE(std::isnan(a[6].x));
  EXPECT_TRUE(std::isnan(a[7].x));
  for (size_t i = 0; i < a.size(); ++i) {
    EXPECT_EQ(a[i].interval_ns, b[i].interval_ns) << i;
  }
  // Earlier time still wins over a NaN component.
  queue[1].timestamp_ns = 999;
  const std::vector<ImuEvent> c = PrepareTrackingSensorEvents(queue, false);
  EXPECT_EQ(c[0].interval_ns, 2);
}

TEST(SensorIntake, QueueClearedAfterMoreThan5Seconds) {
  SensorIntake intake(false);
  const Vec3f a{0.f, 0.f, 9.8f};
  intake.OnDeviceMotion(Motion(1000, std::nullopt, a, std::nullopt));
  // Exactly 5e9 after the first queued event: kept (strict '>').
  intake.OnDeviceMotion(Motion(1000 + kEventQueueClearNanos, std::nullopt, a, std::nullopt));
  StagedSensors staged = intake.TakeForStage();
  EXPECT_EQ(staged.events.size(), 2u);
  intake.OnDeviceMotion(Motion(1000, std::nullopt, a, std::nullopt));
  intake.OnDeviceMotion(Motion(1001 + kEventQueueClearNanos, std::nullopt, a, std::nullopt));
  staged = intake.TakeForStage();
  ASSERT_EQ(staged.events.size(), 1u);  // the first event was dropped
  EXPECT_EQ(staged.events[0].event_time_ns, 1001 + kEventQueueClearNanos);
}

TEST(SensorIntake, TakeForStageEmptiesQueue) {
  SensorIntake intake(false);
  intake.OnDeviceMotion(Motion(1, std::nullopt, Vec3f{}, Vec3f{}));
  EXPECT_EQ(intake.TakeForStage().events.size(), 2u);
  EXPECT_TRUE(intake.TakeForStage().events.empty());
}

TEST(SensorIntake, SortTieBreakOnValues) {
  // Same time and kind: ordered by x, then y, then z (tracking-sensor-event.h:42-56).
  std::vector<RawSensorEvent> raw;
  raw.push_back({RawSensorKind::kGyroscope, 7, 0, -1.f, 0.f, 0.f});  // post-647 x = 1
  raw.push_back({RawSensorKind::kGyroscope, 7, 0, 2.f, 0.f, 0.f});   // post-647 x = -2
  raw.push_back({RawSensorKind::kAccelerometer, 7, 0, 0.f, 0.f, 0.f});
  raw.push_back({RawSensorKind::kGyroscope, 3, 0, 0.f, 0.f, 0.f});
  const std::vector<ImuEvent> e = PrepareTrackingSensorEvents(raw, false);
  ASSERT_EQ(e.size(), 4u);
  EXPECT_EQ(e[0].event_time_ns, 3);
  EXPECT_EQ(e[1].kind, ImuEvent::Kind::kAccelerometer);
  EXPECT_EQ(e[2].x, -2.f);
  EXPECT_EQ(e[3].x, 1.f);
}

TEST(SensorIntake, IntakePathEqualsManual647Path) {
  // D-14 test 4: SensorIntake::TakeForStage == PrepareTrackingSensorEvents on the raw queue the
  // JS handler builds (capnp kind numbers, receive time, interval in ns).
  SensorIntake intake(false);
  intake.OnDeviceMotion(Motion(100, Vec3f{0.5f, -0.5f, 1.f}, Vec3f{0.f, 9.8f, 0.1f},
                               Vec3f{1.f, -2.f, 3.f}, 16.0));
  intake.OnDeviceMotion(Motion(117, std::nullopt, Vec3f{0.f, 9.7f, 0.2f}, Vec3f{1.5f, -2.f, 3.f},
                               16.0));
  const std::vector<RawSensorEvent> raw = {
      {RawSensorKind::kLinearAcceleration, 100, 16000000, 0.5f, -0.5f, 1.f},
      {RawSensorKind::kAccelerometer, 100, 16000000, 0.f, 9.8f, 0.1f},
      {RawSensorKind::kGyroscope, 100, 16000000, 1.f, -2.f, 3.f},
      {RawSensorKind::kAccelerometer, 117, 16000000, 0.f, 9.7f, 0.2f},
      {RawSensorKind::kGyroscope, 117, 16000000, 1.5f, -2.f, 3.f},
  };
  const std::vector<ImuEvent> manual = PrepareTrackingSensorEvents(raw, false);
  const std::vector<ImuEvent> staged = intake.TakeForStage().events;
  ASSERT_EQ(staged.size(), manual.size());
  for (size_t i = 0; i < manual.size(); ++i) {
    EXPECT_EQ(staged[i].kind, manual[i].kind) << i;
    EXPECT_EQ(staged[i].event_time_ns, manual[i].event_time_ns) << i;
    EXPECT_EQ(staged[i].interval_ns, manual[i].interval_ns) << i;
    EXPECT_EQ(staged[i].x, manual[i].x) << i;
    EXPECT_EQ(staged[i].y, manual[i].y) << i;
    EXPECT_EQ(staged[i].z, manual[i].z) << i;
  }
}

TEST(SensorIntakeDeathTest, RawGyroNumberingAborts) {
  // D-14 test 2: a capnp-numbered gyroscope event (kind 2) reaching the post-647 contract.
  std::vector<ImuEvent> events(1);
  events[0].kind = static_cast<ImuEvent::Kind>(static_cast<int32_t>(RawSensorKind::kGyroscope));
  EXPECT_DEATH(ValidatePostPrepareEvents(events), "MAGNETOMETER");
}

TEST(SensorIntakeDeathTest, TimeReversedEventsAbort) {
  // D-14 test 3.
  std::vector<ImuEvent> events(2);
  events[0].kind = ImuEvent::Kind::kAccelerometer;
  events[0].event_time_ns = 20;
  events[1].kind = ImuEvent::Kind::kGyroscope;
  events[1].event_time_ns = 10;
  EXPECT_DEATH(ValidatePostPrepareEvents(events), "not sorted");
}

TEST(SensorIntake, OrientationQuaternion) {
  SensorIntake intake(false);
  // tracking-controller.ts:261: lastDevicePose_ starts at 0/0/0 -> identity.
  StagedSensors staged = intake.TakeForStage();
  EXPECT_EQ(staged.q.w, 1.f);
  EXPECT_EQ(staged.q.x, 0.f);
  EXPECT_EQ(staged.q.y, 0.f);
  EXPECT_EQ(staged.q.z, 0.f);
  DeviceOrientationSample o;
  o.alpha_deg = 90.0;
  o.beta_deg = 0.0;
  o.gamma_deg = 0.0;
  intake.OnDeviceOrientation(o);
  staged = intake.TakeForStage();
  EXPECT_FLOAT_EQ(staged.q.w, static_cast<float>(std::cos(kPi / 4)));
  EXPECT_FLOAT_EQ(staged.q.z, static_cast<float>(std::sin(kPi / 4)));
  EXPECT_EQ(staged.q.x, 0.f);
  EXPECT_EQ(staged.q.y, 0.f);
  // JS `x || 0`: NaN angles act as 0.
  const c8::Quaternion q = GetQuaternion(std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0);
  EXPECT_EQ(q.w(), 1.f);
  EXPECT_EQ(q.z(), 0.f);
}

// ---- US-024: iOS branch of func 647 and the iPhone K lookup (synthetic packets, SOURCE / decompile expectations) ----
// Expectations are spelled out from PUB:reality/engine/tracking/tracking-sensor-event.cc:106-148 and from the decompile of
// the original func 647 (build/slam-refact-20261015/re/wabt/xr-slam.dcmp f_xx lines 52399-53643: isIos = upper(manufacturer)
// compared with the C string "APPLE" at memory 48780 (checked in the wasm data segment), iOS interval x1000, iOS (x, y) of
// accelerations negated, Android z negated, gyroscope (x, y) negated). The comparison with the running original is the
// preregistered gate orient-us024-gate.v1 (tests/engine/orient_gate_test.cc); nothing here replaces it.

struct SignCase {
  RawSensorKind kind;
  bool ios;
  float in[3];
  float want[3];
};

TEST(IosPath, SignRulesAllKinds) {
  const float a[3] = {1.5f, -2.25f, 9.5f};
  const SignCase cases[] = {
      {RawSensorKind::kAccelerometer, false, {a[0], a[1], a[2]}, {a[0], a[1], -a[2]}},
      {RawSensorKind::kAccelerometer, true, {a[0], a[1], a[2]}, {-a[0], -a[1], a[2]}},
      {RawSensorKind::kLinearAcceleration, false, {a[0], a[1], a[2]}, {a[0], a[1], -a[2]}},
      {RawSensorKind::kLinearAcceleration, true, {a[0], a[1], a[2]}, {-a[0], -a[1], a[2]}},
      {RawSensorKind::kGyroscope, false, {a[0], a[1], a[2]}, {-a[0], -a[1], a[2]}},
      {RawSensorKind::kGyroscope, true, {a[0], a[1], a[2]}, {-a[0], -a[1], a[2]}},
  };
  for (const SignCase& c : cases) {
    const std::vector<RawSensorEvent> raw = {{c.kind, 1000, 16000000, c.in[0], c.in[1], c.in[2]}};
    const std::vector<ImuEvent> e = PrepareTrackingSensorEvents(raw, c.ios);
    ASSERT_EQ(e.size(), 1u);
    EXPECT_EQ(e[0].x, c.want[0]) << static_cast<int>(c.kind) << " ios " << c.ios;
    EXPECT_EQ(e[0].y, c.want[1]) << static_cast<int>(c.kind) << " ios " << c.ios;
    EXPECT_EQ(e[0].z, c.want[2]) << static_cast<int>(c.kind) << " ios " << c.ios;
  }
}

TEST(IosPath, IntervalIsMultipliedBy1000OnIosAndMappedOnAndroid) {
  // iOS reports `interval` in seconds: sensors.ts x1e6 gives microseconds, func 647 x1000 gives nanoseconds.
  EXPECT_EQ(IntervalMsToNanos(0.016667) * 1000, 16667LL * 1000);
  const std::vector<RawSensorEvent> ios = {{RawSensorKind::kAccelerometer, 5, IntervalMsToNanos(0.016667), 0.f, 0.f, 1.f}};
  EXPECT_EQ(PrepareTrackingSensorEvents(ios, true)[0].interval_ns, 16667000LL);
  // iOS keeps 16000000 * 1000: no 60 Hz correction.
  const std::vector<RawSensorEvent> ios16 = {{RawSensorKind::kAccelerometer, 5, 16000000, 0.f, 0.f, 1.f}};
  EXPECT_EQ(PrepareTrackingSensorEvents(ios16, true)[0].interval_ns, 16000000000LL);
  EXPECT_EQ(PrepareTrackingSensorEvents(ios16, false)[0].interval_ns, 16666667LL);
  // Only exactly 16000000 is corrected on Android.
  const std::vector<RawSensorEvent> other = {{RawSensorKind::kAccelerometer, 5, 16000001, 0.f, 0.f, 1.f}};
  EXPECT_EQ(PrepareTrackingSensorEvents(other, false)[0].interval_ns, 16000001LL);
}

TEST(IosPath, ZeroSignFollowsTheBinaryPrediction) {
  // Registered prediction (orient-us024-gate.v2, from the wat of func 647, f647.wat:1567-1582 and 1691-1745): the rotation is a
  // multiply-add with the identity plus the constant (0, 0, 0, 1), so every raw zero, +0 or -0, leaves it as +0; the sign rule is
  // an f32.neg after that, so a zero component that the rule negates is 0x80000000 and every other zero is 0x00000000.
  struct Row { RawSensorKind kind; bool ios; bool neg[3]; };
  const Row rows[] = {
      {RawSensorKind::kAccelerometer, false, {false, false, true}},
      {RawSensorKind::kAccelerometer, true, {true, true, false}},
      {RawSensorKind::kLinearAcceleration, false, {false, false, true}},
      {RawSensorKind::kLinearAcceleration, true, {true, true, false}},
      {RawSensorKind::kGyroscope, false, {true, true, false}},
      {RawSensorKind::kGyroscope, true, {true, true, false}},
  };
  for (const Row& r : rows) {
    for (const float zero : {0.0f, -0.0f}) {
      const std::vector<RawSensorEvent> raw = {{r.kind, 1, 16000000, zero, zero, zero}};
      const std::vector<ImuEvent> e = PrepareTrackingSensorEvents(raw, r.ios);
      const float v[3] = {e[0].x, e[0].y, e[0].z};
      for (int k = 0; k < 3; ++k) {
        uint32_t bits;
        std::memcpy(&bits, &v[k], 4);
        EXPECT_EQ(bits, r.neg[k] ? 0x80000000u : 0u)
            << "kind " << static_cast<int>(r.kind) << " ios " << r.ios << " input sign " << std::signbit(zero) << " component " << k;
      }
    }
  }
}

TEST(IosPath, IntervalProductWrapsLikeI64Mul) {
  // The binary's i64.mul wraps; the engine must not hit signed overflow.
  const std::vector<RawSensorEvent> raw = {{RawSensorKind::kAccelerometer, 1, INT64_MAX / 1000 + 1, 0.f, 0.f, 1.f}};
  const uint64_t want = (static_cast<uint64_t>(INT64_MAX / 1000 + 1)) * 1000u;
  EXPECT_EQ(static_cast<uint64_t>(PrepareTrackingSensorEvents(raw, true)[0].interval_ns), want);
}

TEST(IosPath, IphoneKLookup) {
  // The model strings XR8.XrDevice.deviceEstimate() builds for iPhones from screen.height * devicePixelRatio when the
  // WebGL renderer is not an "Apple A## GPU" (xr.js TA / qA; recorded from the original with iPhone UAs in
  // orient-us024-gate.v1): the exact keys ("iPhone13,4") and the web fallback keys of device-infos.cc:163-174 are table
  // entries; Intrinsics::getFallbackCameraIntrinsics (the string compare of intrinsics.cc:477-497) is reached only for
  // unknown models.
  struct Row { const char* model; float focal; const char* why; };
  const Row rows[] = {
      {"iPhone13,4", 496.f, "IPHONE13,4 -> APPLE_IPHONE_12_PRO_MAX (device-infos.cc:150, intrinsics.cc:392)"},
      {"iPhone15,3", 449.4042f, "IPHONE15,3 -> APPLE_IPHONE_14_PRO (device-infos.cc:157)"},
      {"iPhone15,4", 459.4768f, "IPHONE15,4 -> APPLE_IPHONE_14_PRO_MAX (device-infos.cc:158, intrinsics.cc:401)"},
      {"iPhone 13/13Pro", 477.786f, "web fallback key IPHONE 13/13PRO -> APPLE_IPHONE_13 (device-infos.cc:163)"},
      {"iPhone XR/11", 488.1437f, "web fallback key IPHONE XR/11 -> APPLE_IPHONE_XR (device-infos.cc:166)"},
      {"iPhone 6/6s/7/8", 538.0105f, "web fallback key IPHONE 6/6S/7/8 -> APPLE_IPHONE_7 (device-infos.cc:171)"},
      {"iPhone 5/5s/5c/SE", 546.5731f, "web fallback key IPHONE 5/5S/5C/SE -> APPLE_IPHONE_SE (device-infos.cc:172)"},
      {"iPhone", 506.4104f, "generic string: default"},
  };
  for (const Row& r : rows) {
    SCOPED_TRACE(r.why);
    const PinholeIntrinsics full = FullIntrinsicsForDevice("Apple", r.model);
    EXPECT_EQ(full.fx, r.focal) << r.model;
    EXPECT_EQ(full.fy, r.focal) << r.model;
    EXPECT_EQ(full.cx, 239.5f);
    EXPECT_EQ(full.cy, 319.5f);
    // The processing K of a 480x640 capture is the table K itself.
    const IntrinsicsSet set = ComputeIntrinsicsSet("Apple", r.model, 480, 640, 480, 640);
    EXPECT_EQ(set.processing.fx, r.focal) << r.model;
    // A landscape capture is exchanged to portrait first: the same K.
    const IntrinsicsSet land = ComputeIntrinsicsSet("Apple", r.model, 640, 480, 480, 640);
    EXPECT_EQ(land.processing.fx, r.focal) << r.model;
  }
  // The manufacturer comparison is case-insensitive like the original (toUpperCase).
  EXPECT_EQ(FullIntrinsicsForDevice("APPLE", "iphone13,4").fx, 496.f);
  EXPECT_EQ(FullIntrinsicsForDevice("apple", "IPHONE13,4").fx, 496.f);
}

// ---- F-H8 (unit): GetQuaternion vs captured stageFrame q (meta excerpt, 3 captures) ----

TEST(FrontendQuaternion, MatchesCapturedStageFrameArgs) {
  Json meta;
  ASSERT_TRUE(testing::ReadJsonFile(
      testing::TestDataDir() + "/frontend/capture/tum-room1-full/c2/meta-excerpt.json", &meta));
  const Json& caps = meta["captures"];
  ASSERT_EQ(caps.size(), 3u);
  for (size_t i = 0; i < caps.size(); ++i) {
    const Json& args = caps[i]["stageArgs"];
    const c8::Quaternion q = GetQuaternion(args["alphaBetaGamma"][0].number(),
                                           args["alphaBetaGamma"][1].number(),
                                           args["alphaBetaGamma"][2].number());
    EXPECT_EQ(q.w(), static_cast<float>(args["qwxyz"][0].number())) << "capture " << i;
    EXPECT_EQ(q.x(), static_cast<float>(args["qwxyz"][1].number())) << "capture " << i;
    EXPECT_EQ(q.y(), static_cast<float>(args["qwxyz"][2].number())) << "capture " << i;
    EXPECT_EQ(q.z(), static_cast<float>(args["qwxyz"][3].number())) << "capture " << i;
  }
}

// ---- F-K: K rule ----

TEST(FrontendIntrinsics, TumPixel7) {
  const PinholeIntrinsics full = FullIntrinsicsForDevice("Google", "Pixel 7");
  EXPECT_EQ(full.width, 480);
  EXPECT_EQ(full.height, 640);
  EXPECT_EQ(full.cx, 239.5f);
  EXPECT_EQ(full.cy, 319.5f);
  EXPECT_EQ(full.fx, 521.8225f);
  EXPECT_EQ(full.fy, 521.8225f);
  const IntrinsicsSet set = ComputeIntrinsicsSet("Google", "Pixel 7", 240, 320, 480, 640);
  EXPECT_NEAR(set.capture.fx, 260.91125f, 1e-4f);
  EXPECT_EQ(set.capture.fx, 521.8225f * 0.5f);
  EXPECT_EQ(set.capture.width, 240);
  EXPECT_EQ(set.capture.height, 320);
  EXPECT_EQ(set.capture.cx, 119.5f);
  EXPECT_EQ(set.capture.cy, 159.5f);
  EXPECT_EQ(set.processing.fx, 521.8225f);
  EXPECT_EQ(set.processing.fy, 521.8225f);
  EXPECT_EQ(set.processing.cx, 239.5f);
  EXPECT_EQ(set.processing.cy, 319.5f);
  // JS projection check of S1 §5.12: intrinsics[0] = 2 f / w = 2.17426 at 240x320.
  EXPECT_NEAR(2.f * set.capture.fx / 240.f, 2.17426f, 1e-5f);
  // A landscape capture geometry is turned portrait first.
  const IntrinsicsSet land = ComputeIntrinsicsSet("Google", "Pixel 7", 320, 240, 480, 640);
  EXPECT_EQ(land.capture.fx, set.capture.fx);
  EXPECT_EQ(land.capture.width, 240);
}

TEST(FrontendIntrinsics, Flip6FallsBackToIphoneX) {
  // Chrome reduced UA model "K" (S1 §5.12): unregistered -> fallback -> APPLE_IPHONE_X 506.4104.
  for (const char* manufacturer : {"", "Samsung", "samsung", "Google"}) {
    const PinholeIntrinsics k = FullIntrinsicsForDevice(manufacturer, "K");
    EXPECT_EQ(k.fx, 506.4104f) << manufacturer;
  }
  // The real Flip6 model string is not in the table either.
  EXPECT_EQ(FullIntrinsicsForDevice("samsung", "SM-F741N").fx, 506.4104f);
  const IntrinsicsSet set = ComputeIntrinsicsSet("", "K", 480, 640, 480, 640);
  EXPECT_EQ(set.processing.fx, 506.4104f);
  EXPECT_NEAR(2.f * set.capture.fx / 480.f, 2.11004f, 1e-5f);
}

TEST(FrontendIntrinsics, DeviceMapRules) {
  // Samsung model base = first 7 characters.
  EXPECT_EQ(GetDeviceModel("samsung", "SM-G973F"), device_infos::SAMSUNG_GALAXY_S10);
  EXPECT_EQ(FullIntrinsicsForDevice("samsung", "SM-G973F").fx, 492.1022f);
  // Empty manufacturer: model-only map, SM- prefix, Huawei prefix with '-'.
  EXPECT_EQ(GetDeviceModel("", "SM-G973U"), device_infos::SAMSUNG_GALAXY_S10);
  EXPECT_EQ(GetDeviceModel("", "VOG-L29"), device_infos::HUAWEI_P30);
  EXPECT_EQ(GetDeviceModel("", "LE2115"), device_infos::ONEPLUS_9_5G);
  EXPECT_EQ(GetDeviceModel("huawei", "VOG-L29"), device_infos::HUAWEI_P30);
  // Case-insensitive manufacturer and model.
  EXPECT_EQ(GetDeviceModel("google", "pixel 7"), device_infos::GOOGLE_PIXEL7);
  // Uncalibrated models get the default focal length.
  EXPECT_EQ(FullIntrinsicsForDevice("Apple", "iPhone16,1").fx, 506.4104f);
  EXPECT_EQ(GetDeviceModel("Apple", "iPhone16,1"), device_infos::APPLE_IPHONE_15);
  // Web fallback strings registered in the Apple map.
  EXPECT_EQ(FullIntrinsicsForDevice("Apple", "iPhone 12/12Pro").fx, 474.6663f);
  // getFallbackCameraIntrinsics (unknown manufacturer/model pair).
  EXPECT_EQ(FallbackCameraIntrinsics("iphone xr/11").fx, 488.1437f);
  EXPECT_EQ(FallbackCameraIntrinsics("IPHONE 5/5S/5C/SE").fx, 506.4104f);  // APPLE_IPHONE_5 default
  EXPECT_EQ(GetDeviceModel("Nokia", "3310"), device_infos::NOT_SPECIFIED);
}

TEST(FrontendIntrinsics, CropAndScaleFormula) {
  const PinholeIntrinsics k{480, 640, 238.5f, 320.5f, 500.f, 501.f};
  const PinholeIntrinsics c = CropAndScale(k, 360, 640);  // scale = max(0.75, 1) = 1
  EXPECT_EQ(c.fx, 500.f);
  EXPECT_EQ(c.fy, 501.f);
  EXPECT_EQ(c.cx, (360 - 1) / 2.0f + (238.5f - (480 - 1) / 2.0f) * 1.f);
  EXPECT_EQ(c.cy, (640 - 1) / 2.0f + (320.5f - (640 - 1) / 2.0f) * 1.f);
}

}  // namespace
}  // namespace slam_refact::frontend

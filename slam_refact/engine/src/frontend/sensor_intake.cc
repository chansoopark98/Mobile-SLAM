// Sensor intake: JS sensors.ts / tracking-controller.ts onProcessGpu state and func 647
// prepareTrackingSensorFrame (S1 §3.1, §3.6, §5.13).
//
// Sources (SOURCE = public xr_reference/xr_reference@4edfa484; VERIFIED per S1 §0.7 against the deployed
// xr-slam.js strings and the func 647 decompile):
//   reality/app/xr/js/src/sensors.ts:48-79            updateDeviceMotion, 5e9 ns clear rule
//   reality/app/xr/js/src/tracking-controller.ts:261, 373-374, 438-458   lastDevicePose_, onProcessGpu
//   reality/app/xr/js/src/quaternion.ts:20-40          getQuaternion (ZXY)
//   reality/engine/tracking/tracking-sensor-event.cc:70-172  prepareTrackingSensorFrame (event part)
//   reality/engine/tracking/tracking-sensor-event.h:42-56    TrackingSensorEvent::operator<

#include <cmath>
#include <cstdint>

#include "c8/hmatrix.h"
#include "c8/hpoint.h"
#include "c8/quaternion.h"
#include "common/c8_adapters.h"
#include "common/check.h"
#include "compat/libcxx_algorithms.h"
#include "frontend/frontend.h"
#include "frontend/sensor_intake.h"

namespace slam_refact::frontend {
namespace {

// One float key of TrackingSensorEvent::operator< (`if (a != b) return a < b;` then the next key)
// as a three-way result. For non-NaN values this is the original comparison exactly (+0 and -0
// compare equal, as with !=). The original is not a strict weak order once a NaN is involved
// (NaN != v but neither NaN < v nor v < NaN), so libc++ std::sort (func 647) has undefined
// behaviour on such input. Here a NaN sorts after every number and equal to another NaN, then the
// next key decides (S2 review LOW-3; INFERRED replacement, the original order is not reproducible).
int CompareFloatKey(float a, float b) {
  if (a < b) {
    return -1;
  }
  if (b < a) {
    return 1;
  }
  const bool a_nan = std::isnan(a);
  const bool b_nan = std::isnan(b);
  if (a_nan == b_nan) {
    return 0;
  }
  return a_nan ? 1 : -1;
}

// tracking-sensor-event.h:42-56 (post-647 kind numbering), NaN-safe (see CompareFloatKey).
bool TrackingSensorEventLess(const ImuEvent& a, const ImuEvent& b) {
  if (a.event_time_ns != b.event_time_ns) {
    return a.event_time_ns < b.event_time_ns;
  }
  if (a.kind != b.kind) {
    return static_cast<int32_t>(a.kind) < static_cast<int32_t>(b.kind);
  }
  if (const int c = CompareFloatKey(a.x, b.x); c != 0) {
    return c < 0;
  }
  if (const int c = CompareFloatKey(a.y, b.y); c != 0) {
    return c < 0;
  }
  return CompareFloatKey(a.z, b.z) < 0;
}

// JS Math.PI.
constexpr double kJsPi = 3.141592653589793;

// JS `deg * degToRad || 0` with degToRad = Math.PI / 180: NaN and +-0 both become +0.
double JsRadiansOrZero(double deg) {
  const double deg_to_rad = kJsPi / 180;
  const double v = deg * deg_to_rad;
  return (v == 0.0 || std::isnan(v)) ? 0.0 : v;
}

void PushRaw(RawSensorKind kind, int64_t timestamp_ns, int64_t interval_ns, const Vec3f& v,
             std::vector<RawSensorEvent>* queue) {
  RawSensorEvent e;
  e.kind = kind;
  e.timestamp_ns = timestamp_ns;
  e.interval_ns = interval_ns;
  e.x = v.x;
  e.y = v.y;
  e.z = v.z;
  queue->push_back(e);
}

}  // namespace

int64_t IntervalMsToNanos(double interval_ms) {
  const double ns = interval_ms * 1000 * 1000;  // sensors.ts:56, evaluated left to right
  // capnp-ts Int64.setValue(v) as deployed in xr-slam.js (VERIFIED, read from the bundle; S2 review
  // LOW-3): neg = v < 0; m = |v|; lo = m % 2^32; hi = m / 2^32; RangeError when hi > 2^32;
  // hi >>>= 0; bytes 0-3 from lo (ToInt32 / >>> 8, i.e. trunc(lo)), bytes 4-7 from hi; negate
  // (two's complement) when neg. Consequences: NaN -> 0 (NaN % 2^32 is NaN and 255 & NaN = 0),
  // |v| in [2^63, 2^64] wraps modulo 2^64, |v| > 2^64 and +-Infinity throw. For |v| < 2^63 this
  // is trunc toward zero with the sign applied.
  if (std::isnan(ns)) {
    return 0;
  }
  constexpr double kTwo32 = 4294967296.0;
  const bool negative = ns < 0;
  const double magnitude = std::fabs(ns);
  // The thrown RangeError aborts setEventQueue in the original; there is no recoverable path here.
  SR_CHECK(!(magnitude / kTwo32 > kTwo32),
           "DeviceMotion interval * 1e6 is outside Int64 range (capnp-ts RangeError)");
  const uint64_t lo = static_cast<uint64_t>(std::fmod(magnitude, kTwo32));
  const uint64_t hi = static_cast<uint64_t>(std::fmod(std::trunc(magnitude / kTwo32), kTwo32));
  const uint64_t bits = (hi << 32) | lo;
  return static_cast<int64_t>(negative ? ~bits + 1u : bits);
}

void ValidatePostPrepareEvents(const std::vector<ImuEvent>& events) {
  for (size_t i = 0; i < events.size(); ++i) {
    SR_CHECK(events[i].kind != ImuEvent::Kind::kMagnetometer,
             "post-647 event of kind 2 (MAGNETOMETER): a capnp-numbered raw event (gyroscope = 2) "
             "reached the engine without func 647 (D-14)");
    if (i > 0) {
      SR_CHECK(events[i - 1].event_time_ns <= events[i].event_time_ns,
               "post-647 events are not sorted by time (D-14)");
    }
  }
}

SensorIntake::SensorIntake(bool is_ios) : is_ios_(is_ios) {}

void SensorIntake::OnDeviceMotion(const DeviceMotionSample& s) {
  const int64_t timestamp_ns = s.receive_time_ns;
  const int64_t interval_ns = IntervalMsToNanos(s.interval_ms);
  // sensors.ts:61-64.
  if (!queue_.empty() && (timestamp_ns - queue_.front().timestamp_ns) > kEventQueueClearNanos) {
    queue_.clear();
  }
  // sensors.ts:66-78 (push order LINEAR_ACCELERATION, ACCELEROMETER, GYROSCOPE).
  if (s.acceleration.has_value()) {
    PushRaw(RawSensorKind::kLinearAcceleration, timestamp_ns, interval_ns, *s.acceleration,
            &queue_);
  }
  if (s.acceleration_including_gravity.has_value()) {
    PushRaw(RawSensorKind::kAccelerometer, timestamp_ns, interval_ns,
            *s.acceleration_including_gravity, &queue_);
  }
  if (s.rotation_rate_abg.has_value()) {
    // x = alpha, y = beta, z = gamma (deg/s).
    PushRaw(RawSensorKind::kGyroscope, timestamp_ns, interval_ns, *s.rotation_rate_abg, &queue_);
  }
}

void SensorIntake::OnDeviceOrientation(const DeviceOrientationSample& s) {
  // tracking-controller.ts:373-374.
  alpha_deg_ = s.alpha_deg;
  beta_deg_ = s.beta_deg;
  gamma_deg_ = s.gamma_deg;
}

StagedSensors SensorIntake::TakeForStage() {
  // tracking-controller.ts:453-458: quaternion first, then setEventQueue (which empties the queue).
  StagedSensors staged;
  staged.q = ToQuatWxyz(GetQuaternion(alpha_deg_, beta_deg_, gamma_deg_));
  staged.events = PrepareTrackingSensorEvents(queue_, is_ios_);
  queue_.clear();
  return staged;
}

c8::Quaternion GetQuaternion(double alpha_deg, double beta_deg, double gamma_deg) {
  // quaternion.ts:20-40 in double precision; stageFrame receives the components as float.
  const double x = JsRadiansOrZero(beta_deg);
  const double y = JsRadiansOrZero(gamma_deg);
  const double z = JsRadiansOrZero(alpha_deg);
  const double cx = std::cos(x / 2);
  const double cy = std::cos(y / 2);
  const double cz = std::cos(z / 2);
  const double sx = std::sin(x / 2);
  const double sy = std::sin(y / 2);
  const double sz = std::sin(z / 2);
  const double w = cx * cy * cz - sx * sy * sz;
  const double qx = sx * cy * cz - cx * sy * sz;
  const double qy = cx * sy * cz + sx * cy * sz;
  const double qz = cx * cy * sz + sx * sy * cz;
  return c8::Quaternion(static_cast<float>(w), static_cast<float>(qx), static_cast<float>(qy),
                        static_cast<float>(qz));
}

std::vector<ImuEvent> PrepareTrackingSensorEvents(const std::vector<RawSensorEvent>& queue,
                                                  bool is_ios) {
  // tracking-sensor-event.cc:77-80: sensorRotationToPortrait is not set on web -> identity.
  const c8::HMatrix rotation = c8::Quaternion().toRotationMat();
  std::vector<ImuEvent> events;
  events.reserve(queue.size());
  for (const RawSensorEvent& re : queue) {
    ImuEvent se;
    // :101-105. The handler receive time (timestampNanos) is used; eventTimestampNanos is dropped.
    const c8::HPoint3 rotated = rotation * c8::HPoint3(re.x, re.y, re.z);
    se.x = rotated.x();
    se.y = rotated.y();
    se.z = rotated.z();
    se.event_time_ns = re.timestamp_ns;
    // :106-113.
    if (is_ios) {
      // i64.mul of the binary wraps; the signed product would be undefined behaviour for |interval| > INT64_MAX / 1000.
      se.interval_ns = static_cast<int64_t>(static_cast<uint64_t>(re.interval_ns) * 1000u);
    } else {
      se.interval_ns = re.interval_ns == 16000000 ? 16666667 : re.interval_ns;
    }
    // :116-148 sign fixes.
    switch (re.kind) {
      case RawSensorKind::kAccelerometer:
      case RawSensorKind::kLinearAcceleration:
        if (is_ios) {
          se.x = -se.x;
          se.y = -se.y;
        } else {
          se.z = -se.z;
        }
        break;
      case RawSensorKind::kGyroscope:
        se.x = -se.x;
        se.y = -se.y;
        break;
      case RawSensorKind::kMagnetometer:
        break;
    }
    // :151-167 kind renumbering (capnp -> TrackingSensorEvent).
    switch (re.kind) {
      case RawSensorKind::kAccelerometer:
        se.kind = ImuEvent::Kind::kAccelerometer;
        break;
      case RawSensorKind::kGyroscope:
        se.kind = ImuEvent::Kind::kGyroscope;
        break;
      case RawSensorKind::kMagnetometer:
        se.kind = ImuEvent::Kind::kMagnetometer;
        break;
      case RawSensorKind::kLinearAcceleration:
        se.kind = ImuEvent::Kind::kLinearAcceleration;
        break;
    }
    events.push_back(se);
  }
  // :172 std::sort (libc++, binary func 647).
  libcxx::sort(events.begin(), events.end(), TrackingSensorEventLess);
  return events;
}

}  // namespace slam_refact::frontend

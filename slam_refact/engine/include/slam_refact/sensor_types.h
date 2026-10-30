// slam_refact public API: sensor input types (architecture.md §4.1, D-14).
//
// FROZEN CONTRACT (WP0b). Changes follow slam_refact/docs/design/contract-changes.md.

#ifndef SLAM_REFACT_SENSOR_TYPES_H_
#define SLAM_REFACT_SENSOR_TYPES_H_

#include <cstdint>
#include <optional>

#include "slam_refact/types.h"

namespace slam_refact {

// W3C DeviceMotionEvent values as received by the JS handler. Time = handler receive time
// (not event.timeStamp, S1 §0.7).
struct DeviceMotionSample {
  int64_t receive_time_ns = 0;
  // m/s^2 without gravity. When absent, no LINEAR_ACCELERATION event is produced.
  std::optional<Vec3f> acceleration;
  // m/s^2 including gravity.
  std::optional<Vec3f> acceleration_including_gravity;
  // deg/s as (alpha, beta, gamma).
  std::optional<Vec3f> rotation_rate_abg;
  double interval_ms = 0.0;
};

// W3C DeviceOrientationEvent values, degrees.
struct DeviceOrientationSample {
  int64_t receive_time_ns = 0;
  double alpha_deg = 0.0;
  double beta_deg = 0.0;
  double gamma_deg = 0.0;
};

// Raw event with capnp kind numbers (input of func 647 only). Used inside SensorIntake and in
// tests; never an input of Engine or GlueRunner::Publish.
enum class RawSensorKind : int32_t {
  kAccelerometer = 1,
  kGyroscope = 2,
  kMagnetometer = 3,
  kLinearAcceleration = 4,
};

struct RawSensorEvent {
  RawSensorKind kind = RawSensorKind::kAccelerometer;
  int64_t timestamp_ns = 0;
  int64_t interval_ns = 0;
  float x = 0.f;
  float y = 0.f;
  float z = 0.f;
};

// Post-647 event = original c8::TrackingSensorEvent (40 bytes; S2 §2.0, S4 §2.1).
// Kind numbers are the post-647 numbering; kind 2 (MAGNETOMETER) never occurs on web and is
// rejected by Engine::Track (D-14 runtime guard).
struct ImuEvent {
  enum class Kind : int32_t {
    kAccelerometer = 1,
    kMagnetometer = 2,
    kGyroscope = 3,
    kLinearAcceleration = 4,
  };
  Kind kind = Kind::kAccelerometer;
  int64_t event_time_ns = 0;
  // Android 16000000 is already corrected to 16666667.
  int64_t interval_ns = 0;
  // Accelerometer: m/s^2 (Android z negated). Gyroscope: deg/s as (-alpha, -beta, gamma).
  float x = 0.f;
  float y = 0.f;
  float z = 0.f;
};

}  // namespace slam_refact

#endif  // SLAM_REFACT_SENSOR_TYPES_H_

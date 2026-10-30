// Frontend private header: sensor intake helpers beyond the frozen frontend.h declarations
// (S1 §3.1, §3.6; architecture.md §4.1 "IMU 계약", D-14).

#ifndef SLAM_REFACT_FRONTEND_SENSOR_INTAKE_H_
#define SLAM_REFACT_FRONTEND_SENSOR_INTAKE_H_

#include <cstdint>
#include <vector>

#include "slam_refact/sensor_types.h"

namespace slam_refact::frontend {

// sensors.ts:61: the JS queue is cleared when the new event is more than 5e9 ns after the first
// queued event (strictly greater).
inline constexpr int64_t kEventQueueClearNanos = 5000000000LL;

// capnp-ts Int64.fromNumber(interval * 1000 * 1000) (sensors.ts:56, xr-slam.js Int64.setValue):
// the magnitude is truncated toward zero, then the sign is applied; NaN gives 0, magnitudes in
// [2^63, 2^64] wrap modulo 2^64, larger magnitudes and infinities abort (capnp-ts RangeError).
// Source: Int64 `I.prototype.setValue` in assets/references/xr_reference/analysis-runtime/standalone/
// xr-slam.js, with VAL32 = 4294967296 (2^32).
int64_t IntervalMsToNanos(double interval_ms);

// Post-647 contract guard (D-14 runtime half). Engine::Track (WP5) calls this on
// FrameInput::imu_events before building TrackingSensorFrame. Aborts (SR_CHECK) when
//   - an event has kind 2 (post-647 MAGNETOMETER): web never produces magnetometer events, so a
//     kind-2 event means a capnp-numbered (gyro = 2) raw event skipped func 647;
//   - event times decrease: func 647 sorts by (eventTimeNanos, kind, x, y, z), so a post-647 list
//     is non-decreasing in time.
void ValidatePostPrepareEvents(const std::vector<ImuEvent>& events);

}  // namespace slam_refact::frontend

#endif  // SLAM_REFACT_FRONTEND_SENSOR_INTAKE_H_

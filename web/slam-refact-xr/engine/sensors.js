// DeviceMotion queue of the slam_refact xr-slam chunk: the original sensors.ts logic
// (ARC:reality/app/xr/js/src/sensors.ts:15-85; deployed SDK/xr-slam.js offset ~5,494,950 `k=function(A){...}`)
// without capnp.
//
// The original pushes one queue entry per sensor kind, and setEventQueue() serialises the queue
// for c8EmAsm_stageFrame and empties it. Here one entry is one DeviceMotion event (the engine's
// GlueRunner::PushDeviceMotion splits it into LINEAR_ACCELERATION, ACCELEROMETER, GYROSCOPE in the
// same order, frontend/sensor_intake.cc:126-146), and takeForStage() hands the entries staged with a
// frame to the caller, which pushes them into the engine right before that frame is processed.
// The 5e9 ns clear rule is applied here exactly as in the original; GlueRunner's SensorIntake
// applies the same rule to the same push sequence, so it never fires a second time.
export function createSensors(isInvalid) {
  const eventQueue = [];

  // updateDeviceMotion: timestampNanos = 1e3 * performance.now() * 1e3 (same float operation order).
  const updateDeviceMotion = ({ acceleration, accelerationIncludingGravity, rotationRate, timeStamp, interval }) => {
    if (isInvalid()) return;
    const timestampNanos = 1e3 * window.performance.now() * 1e3;
    const eventTimestampNanos = 1e3 * timeStamp * 1e3;
    if (eventQueue.length && timestampNanos - eventQueue[0].timestampNanos > 5e9) {
      eventQueue.length = 0;  // clear the queue for large lag times
    }
    eventQueue.push({ timestampNanos, eventTimestampNanos, interval, acceleration, accelerationIncludingGravity, rotationRate });
  };

  // setEventQueue: returns the staged entries and empties the queue (nothing when invalid, like the original).
  const takeForStage = () => {
    if (isInvalid()) return null;
    const staged = eventQueue.slice();
    eventQueue.length = 0;
    return staged;
  };

  return { updateDeviceMotion, takeForStage, queueLength: () => eventQueue.length };
}

// Pushes one staged DeviceMotion entry into SlamRefactGlue.pushDeviceMotionEvent (FULL-7, spec 06 section 6.1): the
// binding does what the original sensors.ts updateDeviceMotion does - each of acceleration, accelerationIncludingGravity
// and rotationRate that is truthy becomes its own sensor event (LINEAR_ACCELERATION, ACCELEROMETER, GYROSCOPE) and the
// numbers go through Number(...) like capnp's float setters. The sub-objects are handed over as they are, so a device
// without a gyroscope (rotationRate null) still delivers its accelerometer (RM-4c; before this the whole event was
// skipped).
//
// Returns false for the one event the engine does not take: `interval * 1e6` outside the capnp Int64 range (or infinite) on
// an event that has a sensor. The engine binding then throws a JS error ("interval is outside the Int64 nanosecond range",
// VERIFIED on the engine pinned for RM-4d, probe: sensor + 1e14 ms / Infinity throw, the same interval on an event without
// any sensor is a no-op, NaN is taken as 0 ns like capnp-ts); an engine built before RM-3c aborted the WASM module instead
// (RM-4c review m4: the older comment and spec 07 section 3 said "abort for events with or without sensors"). The original
// throws a RangeError from setEventQueue for such an event (its whole stage fails and the queue is retried until the 5e9 ns rule
// clears it); the chunk drops that one event and counts it, so a bad event cannot fail the frame. The check is the same
// for events without a sensor (dropped here, a no-op there). The order of the fields the binding reads inside each sensor
// object (alpha/beta/gamma, x/y/z) cannot be seen from JS: the objects are handed over untouched; the engine side owns that
// evidence (RM-3c), this module only proves that nothing is swapped on the way in.
const TWO_POW_32 = 4294967296;
export function pushMotion(glue, e) {
  const ns = e.interval * 1000 * 1000;  // sensors.ts:56, evaluated left to right
  if (!Number.isNaN(ns) && !(Math.abs(ns) / TWO_POW_32 <= TWO_POW_32)) return false;
  glue.pushDeviceMotionEvent(e.timestampNanos, {
    acceleration: e.acceleration,
    accelerationIncludingGravity: e.accelerationIncludingGravity,
    rotationRate: e.rotationRate,
    interval: e.interval,
  });
  return true;
}

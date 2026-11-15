// IMU sources of the webxr-engine prototype: rate statistics, Generic Sensor API -> DeviceMotion shape, orientation quaternion ->
// DeviceOrientation angles, source choice. Pure module (no DOM): the page feeds readings, the engine gets what the chunk would give it.
//
// What the engine takes (sensors.js pushMotion -> SlamRefactGlue.pushDeviceMotionEvent; engine/src/frontend/sensor_intake.cc):
//   {timestampNanos, interval (ms), acceleration {x,y,z} m/s^2 without gravity | null, accelerationIncludingGravity {x,y,z} m/s^2 | null,
//    rotationRate {alpha, beta, gamma} deg/s | null}, device axes (x right, y up, z out of the screen; W3C DeviceMotionEvent), pushed in
//   time order before the frame they belong to; the engine uses timestampNanos (the original's handler receive time) as the event time.
//   pushDeviceOrientation(ns, alpha, beta, gamma): degrees, W3C Z-X'-Y'' Euler (R = Rz(alpha) Rx(beta) Ry(gamma), device -> world).
// Generic Sensor API (W3C): Accelerometer / LinearAccelerationSensor x, y, z in m/s^2 and Gyroscope x, y, z in rad/s, all in the same
// device frame as DeviceMotion with the default referenceFrame 'device'; Accelerometer includes gravity with the same sign as
// accelerationIncludingGravity (+9.8 on z lying face up); timestamp = DOMHighResTimeStamp of the reading (ms, performance.now() clock).
// Conversion (createGenericMotionFuser): one DeviceMotion-shaped entry per Gyroscope reading (as Chrome builds DeviceMotion from the latest
// accelerometer + gyroscope values), rotationRate = {alpha: z, beta: x, gamma: y} * 180/pi (DeviceMotion: alpha about z, beta about x, gamma
// about y), accelerationIncludingGravity = latest Accelerometer, acceleration = latest LinearAccelerationSensor (null until one arrived).
// timestampNanos = reading.timestamp * 1e6: the reading time, not the (main-thread delayed) receive time - DEVIATION from the DeviceMotion
// path, on purpose (p0-imu-starvation: the main thread delivers bursts). interval = 16 ms at 60 Hz (what Chrome DeviceMotion reports; the
// engine maps 16 ms to 16666667 ns), else 1000 / frequency.
import { percentile } from '../webxr-probe/analysis.js';

export const RAD_TO_DEG = 180 / Math.PI;

// timesMs: event times of one source (ms). Rate over the observed span and dt percentiles.
export function rateStats(timesMs, windowMs = null) {
  const t = timesMs.filter(Number.isFinite);
  const dt = [];
  for (let i = 1; i < t.length; i++) dt.push(t[i] - t[i - 1]);
  const span = windowMs ?? (t.length > 1 ? t[t.length - 1] - t[0] : 0);
  return { count: t.length, rateHz: span > 0 ? (windowMs ? t.length : t.length - 1) / (span / 1000) : 0,
    dtP50Ms: percentile(dt, 50), dtP95Ms: percentile(dt, 95), dtMaxMs: dt.length ? maxOf(dt) : NaN };
}

export const gyroToRotationRate = (g) => ({ alpha: g.z * RAD_TO_DEG, beta: g.x * RAD_TO_DEG, gamma: g.y * RAD_TO_DEG });
export const intervalMsFor = (frequency) => (frequency === 60 ? 16 : 1000 / frequency);

// Generic Sensor readings -> DeviceMotion-shaped queue. Readings: {x, y, z, timestamp}. take() hands over the queued entries (time order)
// and empties the queue; the 5e9 ns clear rule of sensors.js applies (a queue older than 5 s is dropped, counted).
export function createGenericMotionFuser({ frequency = 60 } = {}) {
  const interval = intervalMsFor(frequency);
  const queue = [];
  let accel = null, linear = null, lastNs = -Infinity;
  const stats = { gyro: 0, accel: 0, linear: 0, queued: 0, nonMonotonic: 0, cleared: 0 };
  const vec = (r) => ({ x: r.x, y: r.y, z: r.z });
  return {
    onAccelerometer(r) { stats.accel++; accel = vec(r); },
    onLinearAcceleration(r) { stats.linear++; linear = vec(r); },
    onGyroscope(r) {
      stats.gyro++;
      const timestampNanos = r.timestamp * 1e6;
      // Gyroscope timestamps are monotonic per sensor; a reading that is not (never seen) would break the engine's time order: dropped, counted.
      if (!(timestampNanos > lastNs)) { stats.nonMonotonic++; return; }
      lastNs = timestampNanos;
      if (queue.length && timestampNanos - queue[0].timestampNanos > 5e9) { stats.cleared += queue.length; queue.length = 0; }
      queue.push({ timestampNanos, interval, acceleration: linear && { ...linear }, accelerationIncludingGravity: accel && { ...accel },
        rotationRate: gyroToRotationRate(r) });
      stats.queued++;
    },
    take() { const out = queue.slice(); queue.length = 0; return out; },
    queueLength: () => queue.length,
    stats,
  };
}

// Engine GetQuaternion (sensor_intake.cc, quaternion.ts): DeviceOrientation degrees -> [w, x, y, z] = qz(alpha) qx(beta) qy(gamma).
export function deviceOrientationToQuat(alpha, beta, gamma) {
  const r = Math.PI / 180, x = beta * r, y = gamma * r, z = alpha * r;
  const cx = Math.cos(x / 2), cy = Math.cos(y / 2), cz = Math.cos(z / 2), sx = Math.sin(x / 2), sy = Math.sin(y / 2), sz = Math.sin(z / 2);
  return [cx * cy * cz - sx * sy * sz, sx * cy * cz - cx * sy * sz, cx * sy * cz + sx * cy * sz, cx * cy * sz + sx * sy * cz];
}

// OrientationSensor quaternion [x, y, z, w] (device -> world, as DeviceOrientation's R) -> {alpha [0, 360), beta [-180, 180), gamma [-90, 90)}.
// R = Rz(a) Rx(b) Ry(g): R[2][1] = sin b, R[0][1] = -cos b sin a, R[1][1] = cos b cos a, R[2][0] = -cos b sin g, R[2][2] = cos b cos g.
export function quatToDeviceOrientation([x, y, z, w]) {
  const n = Math.hypot(x, y, z, w);
  x /= n; y /= n; z /= n; w /= n;
  const r01 = 2 * (x * y - w * z), r11 = 1 - 2 * (x * x + z * z), r21 = 2 * (y * z + w * x), r20 = 2 * (x * z - w * y), r22 = 1 - 2 * (x * x + y * y);
  let beta = Math.asin(Math.max(-1, Math.min(1, r21))) * RAD_TO_DEG;  // [-90, 90], cos b >= 0
  let alpha = Math.atan2(-r01, r11) * RAD_TO_DEG;
  let gamma = Math.atan2(-r20, r22) * RAD_TO_DEG;
  // The W3C ranges take gamma in [-90, 90): the equivalent solution (a + 180, 180 - b, g + 180) when |gamma| >= 90.
  if (gamma >= 90 || gamma < -90) { alpha += 180; beta = 180 - beta; gamma += 180; }
  const wrap = (v, lo, span) => ((((v - lo) % span) + span) % span) + lo;
  return { alpha: wrap(alpha, 0, 360), beta: wrap(beta, -180, 360), gamma: wrap(gamma, -90, 180) };
}

// Source choice from a phase-A / warm-up report: {devicemotion: {rateHz, withRotationRate}, gyroscope: {rateHz}, accelerometer: {rateHz},
// deviceorientation: {rateHz}, relativeOrientation: {rateHz}, absoluteOrientation: {rateHz}}. The motion source with the higher gyro rate
// wins if it reaches minHz (generic needs an accelerometer too); none -> disableVio. force: 'devicemotion' | 'generic' | 'none' | 'auto'.
export function chooseImuSource(report, { force = 'auto', minHz = 5 } = {}) {
  const hz = (k) => (report && report[k] && Number.isFinite(report[k].rateHz) ? report[k].rateHz : 0);
  const reasons = [];
  const dmHz = report && report.devicemotion && report.devicemotion.withRotationRate === 0 ? 0 : hz('devicemotion');
  const genHz = hz('accelerometer') >= minHz ? hz('gyroscope') : 0;
  let motion;
  if (force && force !== 'auto') { motion = force; reasons.push(`forced by ?imu=${force}`); } else if (Math.max(dmHz, genHz) < minHz) {
    motion = 'none'; reasons.push(`no motion source reached ${minHz} Hz (DeviceMotion ${dmHz.toFixed(1)} Hz, Gyroscope+Accelerometer ${genHz.toFixed(1)} Hz)`);
  } else {
    motion = genHz > dmHz ? 'generic' : 'devicemotion';
    reasons.push(`DeviceMotion ${dmHz.toFixed(1)} Hz, Gyroscope ${genHz.toFixed(1)} Hz -> ${motion}`);
  }
  const orientation = hz('deviceorientation') >= minHz ? 'deviceorientation' : hz('relativeOrientation') >= minHz ? 'relative'
    : hz('absoluteOrientation') >= minHz ? 'absolute' : 'none';
  if (orientation === 'none') reasons.push('no orientation source: (0, 0, 0) is pushed, as xr-slam.js does before the first deviceorientation event');
  return { motion, orientation, disableVio: motion === 'none', reasons };
}

// Iterative max/min (a spread into Math.max/min throws RangeError on large arrays and gives -/+Infinity on empty ones). Empty input -> null.
function maxOf(xs) { if (!xs.length) return null; let m = xs[0]; for (const x of xs) if (x > m || Number.isNaN(x)) m = x; return m; }
function minOf(xs) { if (!xs.length) return null; let m = xs[0]; for (const x of xs) if (x < m || Number.isNaN(x)) m = x; return m; }

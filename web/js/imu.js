/**
 * IMU capture module with high-frequency sensor support.
 *
 * Primary path: Generic Sensor API (Accelerometer + Gyroscope) with explicit
 * frequency control — available on Chrome Android 67+.
 * Fallback path: DeviceMotionEvent — iOS Safari, Firefox, older browsers.
 *
 * Data is stored in a pre-allocated Float64Array ring buffer for zero-copy
 * transfer to the VIO worker via postMessage transferable.
 *
 * Ring buffer layout per slot (7 Float64 values):
 *   [timestamp_s, acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z]
 */

/** Number of slots in the ring buffer */
const RING_CAPACITY = 512;
/** Fields per IMU reading */
const FIELDS_PER_READING = 7;
/** Total Float64 elements in the ring buffer */
const RING_ELEMENTS = RING_CAPACITY * FIELDS_PER_READING;
/**
 * Default sensor frequency to request (Hz).
 *
 * Platform caps (as of 2024):
 *   Chrome Android  — Generic Sensor API hard-capped at 60Hz regardless of
 *                     what you request.  Requesting >60Hz produces a console
 *                     warning and Chrome still delivers ≤60Hz.
 *   iOS Safari      — DeviceMotionEvent only; interval ≈16-10ms → 60-100Hz
 *                     depending on device; no programmatic rate control.
 *   Firefox Android — DeviceMotionEvent; ~60Hz, not configurable.
 *
 * We request 60Hz so Chrome delivers the full 60Hz without warnings.
 * The `start(frequency)` parameter lets callers override for future-proofing
 * or non-Chrome environments that honour higher rates.
 */
const DEFAULT_FREQUENCY = 60;

const DEG_TO_RAD = Math.PI / 180;

// Source timestamps and callback arrival share the document performance clock.
// Legacy DeviceMotion implementations can expose Unix epoch milliseconds.
export function normalizeEventTimestamp(timestampMs, timeOriginMs = performance.timeOrigin) {
    if (!Number.isFinite(timestampMs) || timestampMs < 0) return null;
    const epoch = timestampMs >= 1e12;
    if (epoch && !Number.isFinite(timeOriginMs)) return null;
    const relativeMs = epoch ? timestampMs - timeOriginMs : timestampMs;
    return relativeMs >= 0 ? { timestampS: relativeMs / 1000, clock: epoch ? 'epoch_minus_timeOrigin' : 'timeOrigin_relative' } : null;
}

const finiteVector = vector => vector && [vector.x, vector.y, vector.z].every(Number.isFinite);

/**
 * Detect if running on iOS (Safari, Chrome on iOS, etc.)
 * iOS uses WebKit which reports accelerationIncludingGravity with inverted signs
 * compared to the Generic Sensor API convention.
 * @returns {boolean}
 */
function isIOS() {
    return /iPad|iPhone|iPod/.test(navigator.userAgent) ||
        (navigator.platform === 'MacIntel' && navigator.maxTouchPoints > 1);
}

/**
 * Sensor API type currently in use.
 * @enum {string}
 */
export const SensorType = {
    NONE: 'none',
    GENERIC_SENSOR: 'generic_sensor',
    DEVICE_MOTION: 'device_motion',
};

export class IMU {
    constructor() {
        /** @type {Float64Array} Pre-allocated ring buffer */
        this._ring = new Float64Array(RING_ELEMENTS);
        /** Write index (monotonically increasing, mod RING_CAPACITY for position) */
        this._writeIdx = 0;
        /** Read index (monotonically increasing) */
        this._readIdx = 0;

        this.running = false;
        this._sensorType = SensorType.NONE;

        // Generic Sensor API handles
        this._accel = null;
        this._gyro = null;
        // Latest readings from each sensor (for cross-sensor sampling)
        this._latestGyro = null;
        this._latestAccel = null;
        // Strict ordering of emitted source samples
        this._lastSampleTime = -Infinity;
        this._pendingGyro = null;

        // DeviceMotionEvent handler
        this._motionHandler = null;
        this._lastMotionTimestamp = -Infinity;

        // Generic Sensor timestamp monotonicity guard
        this._lastGenericTimestamp = -Infinity;

        // Platform detection: iOS inverts accelerationIncludingGravity signs
        // iOS Safari: stationary phone reports acc_y ~= -9.81 (gravity opposes +Y)
        // Android Generic Sensor: stationary phone reports acc_y ~= +9.81
        // We need consistent convention: gravity along +Y when phone upright
        this._isIOS = isIOS();
        this._iosAccSign = this._isIOS ? -1 : 1;

        // Rate measurement
        this._rateCount = 0;
        this._rateStartTime = null;
        this._currentRate = 0;

        // Latest reading for UI display
        this.latest = { acc_x: 0, acc_y: 0, acc_z: 0, gyro_x: 0, gyro_y: 0, gyro_z: 0 };

        // Requested frequency
        this._frequency = DEFAULT_FREQUENCY;

        // Gyroscope bias calibration
        // Mobile MEMS gyros have large bias offsets (0.01-0.1 rad/s) that
        // overwhelm VIO pre-integration if not compensated. Calibrate by
        // collecting samples while stationary and computing average gyro.
        this._gyroBias = { x: 0, y: 0, z: 0 };
        this._calibrated = false;

        // Hardware gravity estimate from LINEAR_ACCELERATION
        // gravity = accelerationIncludingGravity - acceleration (DeviceMotion only)
        this._gravityEstimate = null;  // {x, y, z} in device frame, or null if unavailable
        this._gravityEstimateCount = 0;
        this._gravitySumX = 0;
        this._gravitySumY = 0;
        this._gravitySumZ = 0;
        this._diagnostics = this._newDiagnostics();
        this.permission = { state: 'not_requested', api: null, error: null };
    }

    _newDiagnostics() {
        return { receivedAccel: 0, receivedGyro: 0, receivedMotion: 0, emitted: 0, flushed: 0,
            drops: { overflow: 0, stale: 0, nonfinite: 0, timestamp: 0, outOfOrder: 0, pairing: 0, discarded: 0 },
            status: 'stopped', error: null, lastSample: null, calibration: 'not_calibrated' };
    }

    getDiagnostics() {
        return { ...this._diagnostics, drops: { ...this._diagnostics.drops }, permission: { ...this.permission },
            buffered: this._writeIdx - this._readIdx, requestedHz: this._frequency,
            emissionPolicy: 'gyro timestamp; latest accel held within 1.5 requested periods; one emission per gyro',
            maxPairSkewS: 1.5 / this._frequency, timeOriginMs: performance.timeOrigin };
    }

    discard(reason = 'pause') {
        this._diagnostics.drops.discarded += this._writeIdx - this._readIdx;
        this._readIdx = this._writeIdx;
        if (this._pendingGyro) this._diagnostics.drops.pairing++;
        this._pendingGyro = null;
        this._latestAccel = null;
        this._latestGyro = null;
        this._diagnostics.status = reason;
    }

    /**
     * Request device motion permission (required for iOS 13+).
     * Must be called from a user gesture (e.g., button click).
     * Also checks Generic Sensor API permissions on Android.
     * @returns {Promise<boolean>} True if permission granted
     */
    async requestPermission() {
        const record = (state, error = null) => {
            this.permission.state = state;
            this.permission.error = error ? { name: error.name, message: error.message } : null;
            this.permission.resolvedAtMs = performance.now();
        };
        this.permission = { state: 'requested', api: null, error: null, requestedAtMs: performance.now(),
            activationActive: navigator.userActivation?.isActive ?? null };
        if (typeof DeviceMotionEvent !== 'undefined' && typeof DeviceMotionEvent.requestPermission === 'function') {
            this.permission.api = 'DeviceMotionEvent.requestPermission';
            try {
                // Native invocation precedes the first await: preserve Start activation.
                const nativeRequest = DeviceMotionEvent.requestPermission();
                const permission = await nativeRequest;
                record(permission === 'granted' ? 'granted' : 'denied');
                return permission === 'granted';
            } catch (error) { record('error', error); return false; }
        }
        if (typeof Accelerometer !== 'undefined' && typeof Gyroscope !== 'undefined') {
            this.permission.api = 'Permissions.query';
            try {
                const results = await Promise.all(['accelerometer', 'gyroscope'].map(name => navigator.permissions.query({ name })));
                const denied = results.some(result => result.state === 'denied');
                record(denied ? 'denied' : results.every(result => result.state === 'granted') ? 'granted' : 'prompt');
                return !denied;
            } catch (error) {
                // A missing query API permits trying start(), not a claim of permission grant.
                record('query_unavailable', error);
                return true;
            }
        }
        this.permission.api = 'DeviceMotionEvent';
        const available = typeof DeviceMotionEvent !== 'undefined';
        record(available ? 'not_required' : 'unsupported');
        return available;
    }

    /**
     * Calibrate gyroscope bias by collecting stationary samples.
     * MUST be called after start() while device is held still.
     * Computes average gyro reading as bias estimate and validates
     * accelerometer magnitude (~9.81) to confirm stationarity.
     *
     * @param {number} [durationMs=1500] Calibration duration in ms
     * @returns {Promise<{bias: {x,y,z}, gravMag: number, sampleCount: number}>}
     */
    async calibrate(durationMs = 1500) {
        if (!this.running) {
            console.warn('[IMU] Cannot calibrate: not running');
            return null;
        }

        console.log(`[IMU] Calibrating gyro bias (${durationMs}ms, keep device still)...`);

        // Flush any stale data
        this.flush();

        // Collect samples for the specified duration
        await new Promise(resolve => setTimeout(resolve, durationMs));

        const { data, count } = this.flush();
        if (!data || count < 10) {
            console.warn(`[IMU] Calibration failed: only ${count} samples collected`);
            return null;
        }

        // Compute average gyro and accelerometer magnitude
        let sumGx = 0, sumGy = 0, sumGz = 0;
        let sumAx = 0, sumAy = 0, sumAz = 0;
        for (let i = 0; i < count; i++) {
            const off = i * FIELDS_PER_READING;
            sumAx += data[off + 1];
            sumAy += data[off + 2];
            sumAz += data[off + 3];
            sumGx += data[off + 4];
            sumGy += data[off + 5];
            sumGz += data[off + 6];
        }

        const bias = {
            x: sumGx / count,
            y: sumGy / count,
            z: sumGz / count,
        };

        const avgAcc = {
            x: sumAx / count,
            y: sumAy / count,
            z: sumAz / count,
        };
        const gravMag = Math.sqrt(avgAcc.x ** 2 + avgAcc.y ** 2 + avgAcc.z ** 2);

        const biasMag = Math.hypot(bias.x, bias.y, bias.z);
        if (!Number.isFinite(gravMag) || gravMag < 8.5 || gravMag > 11.0 || biasMag > 0.35) {
            this._calibrated = false;
            this._diagnostics.calibration = 'rejected_motion_or_gravity';
            console.warn('[IMU] Calibration rejected: hold still and retry', { gravMag, biasMag });
            return null;
        }
        this._gyroBias = bias;
        this._calibrated = true;
        this._diagnostics.calibration = 'stationary_bias_candidate';

        console.log(`[IMU] Gyro bias calibrated from ${count} samples:`);
        console.log(`[IMU]   bias = (${bias.x.toFixed(5)}, ${bias.y.toFixed(5)}, ${bias.z.toFixed(5)}) rad/s`);
        console.log(`[IMU]   |bias| = ${Math.sqrt(bias.x**2 + bias.y**2 + bias.z**2).toFixed(5)} rad/s`);
        console.log(`[IMU]   |acc| = ${gravMag.toFixed(3)} m/s² (gravity validation)`);

        return { bias, gravMag, sampleCount: count };
    }

    /**
     * Check if gyroscope bias has been calibrated.
     * @returns {boolean}
     */
    isCalibrated() {
        return this._calibrated;
    }

    /**
     * Get current gyroscope bias estimate.
     * @returns {{x: number, y: number, z: number}}
     */
    getGyroBias() {
        return { ...this._gyroBias };
    }

    /**
     * Get hardware gravity estimate from LINEAR_ACCELERATION sensor.
     * Available only on DeviceMotion path (iOS, Firefox, Android fallback).
     * Computed as: gravity = accelerationIncludingGravity - acceleration
     * @returns {{x: number, y: number, z: number} | null} Gravity in device frame, or null
     */
    getGravityEstimate() {
        return this._gravityEstimate ? { ...this._gravityEstimate } : null;
    }

    /**
     * Start capturing IMU data.
     *
     * Strategy:
     * - Android: Prefer Generic Sensor API (configurable frequency up to 200Hz,
     *   higher IMU rate → better VIO pre-integration). DeviceMotionEvent on
     *   Android is capped at ~60Hz and cannot be configured.
     * - iOS: Prefer DeviceMotionEvent (Generic Sensor API is unavailable on
     *   Safari; DeviceMotionEvent provides synchronized accel+gyro at ~60Hz).
     *
     * Generic sources are asynchronous. Emit once at each gyro source timestamp
     * using a finite latest acceleration within 1.5 requested periods; if gyro
     * arrives first it waits for accel. This hold policy is observable and does
     * not establish hardware synchronization or physical sampling accuracy.
     *
     * @param {number} [frequency=60] Requested sensor frequency in Hz.
     *   Chrome Android will cap delivery at 60Hz regardless of this value.
     *   Pass a higher value only for non-Chrome environments that honour it.
     */
    start(frequency = DEFAULT_FREQUENCY) {
        if (this.running) return;

        if (!Number.isFinite(frequency) || frequency <= 0) throw new Error('Invalid IMU frequency');
        this._frequency = frequency;
        this._diagnostics = this._newDiagnostics();
        this._diagnostics.status = 'starting';
        this._latestAccel = null;
        this._latestGyro = null;
        this._pendingGyro = null;
        this._lastGenericTimestamp = -Infinity;
        this._lastSampleTime = -Infinity;
        this._writeIdx = 0;
        this._readIdx = 0;
        this._rateCount = 0;
        this._rateStartTime = null;
        this._currentRate = 0;
        this.running = true;

        if (this._isIOS) {
            // iOS: DeviceMotionEvent only (no Generic Sensor API on Safari)
            if (this._tryDeviceMotion()) {
                this._sensorType = SensorType.DEVICE_MOTION;
                this._diagnostics.status = 'listening';
                console.log('[IMU] Using DeviceMotionEvent (iOS, synchronized accel+gyro)');
                return;
            }
        } else {
            // Android: Prefer Generic Sensor API for higher configurable rate
            if (this._tryGenericSensor(frequency)) {
                this._sensorType = SensorType.GENERIC_SENSOR;
                this._diagnostics.status = 'listening';
                console.log(`[IMU] Using Generic Sensor API @ ${frequency}Hz`);
                return;
            }

            // Fallback to DeviceMotionEvent (~60Hz, not configurable)
            if (this._tryDeviceMotion()) {
                this._sensorType = SensorType.DEVICE_MOTION;
                this._diagnostics.status = 'listening';
                console.warn('[IMU] Fallback to DeviceMotionEvent (~60Hz, rate not configurable)');
                return;
            }
        }

        console.error('[IMU] No IMU sensor API available');
        this.running = false;
        this._sensorType = SensorType.NONE;
        this._diagnostics.status = 'unsupported';
    }

    /**
     * Attempt to start Generic Sensor API (Accelerometer + Gyroscope).
     * @param {number} frequency Requested Hz
     * @returns {boolean} True if successfully started
     * @private
     */
    _tryGenericSensor(frequency) {
        if (typeof Accelerometer === 'undefined' || typeof Gyroscope === 'undefined') {
            return false;
        }

        try {
            // Chrome Android hard-caps Generic Sensor at 60Hz regardless of
            // the requested value. Non-Chrome browsers (Chromium forks, future
            // specs) may honour higher rates, so we pass the requested frequency
            // through without clamping. Chrome will silently cap on its side.
            this._accel = new Accelerometer({ frequency: frequency, referenceFrame: 'device' });
            this._gyro = new Gyroscope({ frequency: frequency, referenceFrame: 'device' });

            const read = (sensor, axis) => {
                if (!this.running) return;
                this._diagnostics[axis === 'accel' ? 'receivedAccel' : 'receivedGyro']++;
                const timestampS = Number.isFinite(sensor.timestamp) && sensor.timestamp >= 0 ? sensor.timestamp / 1000 : null;
                if (timestampS === null) { this._diagnostics.drops.timestamp++; return; }
                if (!finiteVector(sensor)) { this._diagnostics.drops.nonfinite++; return; }
                const previous = axis === 'accel' ? this._latestAccel : this._latestGyro;
                if (previous && timestampS <= previous.timestampS) { this._diagnostics.drops.outOfOrder++; return; }
                const sample = { x: sensor.x, y: sensor.y, z: sensor.z, timestampS, arrivalMs: performance.now() };
                if (axis === 'accel') this._latestAccel = sample;
                else {
                    if (this._pendingGyro && this._pendingGyro.timestampS > this._lastGenericTimestamp) this._diagnostics.drops.pairing++;
                    this._latestGyro = sample;
                    this._pendingGyro = sample;
                }
                this._emitGeneric();
            };
            this._accel.addEventListener('reading', () => read(this._accel, 'accel'));
            this._gyro.addEventListener('reading', () => read(this._gyro, 'gyro'));
            const fail = event => {
                this._diagnostics.error = event.error?.message || 'Generic sensor error';
                this._stopGenericSensor();
                this._latestAccel = this._latestGyro = this._pendingGyro = null;
                if (this.running && this._tryDeviceMotion()) {
                    this._sensorType = SensorType.DEVICE_MOTION;
                    this._diagnostics.status = 'fallback_motion';
                } else { this.running = false; this._sensorType = SensorType.NONE; this._diagnostics.status = 'error'; }
            };
            this._accel.addEventListener('error', fail);
            this._gyro.addEventListener('error', fail);

            this._accel.start();
            this._gyro.start();
            return true;
        } catch (e) {
            console.warn('[IMU] Generic Sensor API failed to start:', e.message);
            this._stopGenericSensor();
            return false;
        }
    }

    _emitGeneric() {
        const acc = this._latestAccel;
        const gyro = this._pendingGyro;
        if (!acc || !gyro || gyro.timestampS <= this._lastGenericTimestamp) return;
        const skewS = acc.timestampS - gyro.timestampS;
        if (Math.abs(skewS) > 1.5 / this._frequency) { this._diagnostics.status = 'waiting_pair'; return; }
        this._lastGenericTimestamp = gyro.timestampS;
        this._pendingGyro = null;
        this._diagnostics.lastSample = { source: 'GenericSensor.timestamp', timestampS: gyro.timestampS,
            accelTimestampS: acc.timestampS, gyroTimestampS: gyro.timestampS, skewS, arrivalMs: performance.now() };
        this._pushSample(gyro.timestampS, acc.x, acc.y, acc.z, gyro.x, gyro.y, gyro.z);
    }

    /**
     * Attempt to start DeviceMotionEvent listener.
     * @returns {boolean} True if successfully started
     * @private
     */
    _tryDeviceMotion() {
        if (typeof DeviceMotionEvent === 'undefined') {
            return false;
        }

        this._lastMotionTimestamp = -Infinity;

        this._motionHandler = (event) => {
            if (!this.running) return;

            const acc = event.accelerationIncludingGravity;
            const rot = event.rotationRate;
            this._diagnostics.receivedMotion++;
            if (!finiteVector(acc) || !rot || ![rot.beta, rot.gamma, rot.alpha].every(Number.isFinite)) {
                this._diagnostics.drops.nonfinite++; return;
            }
            const normalized = normalizeEventTimestamp(event.timeStamp);
            if (!normalized) { this._diagnostics.drops.timestamp++; return; }
            const timestamp = normalized.timestampS;

            // Monotonicity check
            if (timestamp <= this._lastMotionTimestamp) { this._diagnostics.drops.outOfOrder++; return; }
            this._lastMotionTimestamp = timestamp;
            this._diagnostics.lastSample = { source: 'DeviceMotionEvent.timeStamp', clock: normalized.clock,
                timestampS: timestamp, eventTimestampMs: event.timeStamp, arrivalMs: performance.now() };

            // DeviceMotion rotationRate is in deg/s -> convert to rad/s
            // W3C spec: beta=x-axis, gamma=y-axis, alpha=z-axis
            // iOS inverts accelerationIncludingGravity signs vs Android convention
            const s = this._iosAccSign;
            this._pushSample(
                timestamp,
                (acc.x ?? 0) * s, (acc.y ?? 0) * s, (acc.z ?? 0) * s,
                (rot.beta ?? 0) * DEG_TO_RAD,
                (rot.gamma ?? 0) * DEG_TO_RAD,
                (rot.alpha ?? 0) * DEG_TO_RAD
            );

            // Compute hardware gravity estimate from LINEAR_ACCELERATION
            // gravity = accelerationIncludingGravity - acceleration
            const linAcc = event.acceleration;
            if (finiteVector(linAcc)) {
                const gx = (acc.x ?? 0) * s - (linAcc.x ?? 0) * s;
                const gy = (acc.y ?? 0) * s - (linAcc.y ?? 0) * s;
                const gz = (acc.z ?? 0) * s - (linAcc.z ?? 0) * s;
                this._gravitySumX += gx;
                this._gravitySumY += gy;
                this._gravitySumZ += gz;
                this._gravityEstimateCount++;
                // Running average (updated every sample for smoothness)
                const n = this._gravityEstimateCount;
                this._gravityEstimate = {
                    x: this._gravitySumX / n,
                    y: this._gravitySumY / n,
                    z: this._gravitySumZ / n,
                };
            }
        };

        window.addEventListener('devicemotion', this._motionHandler, true);
        return true;
    }

    /**
     * Push a single IMU sample into the ring buffer.
     * @private
     */
    _pushSample(timestamp, ax, ay, az, gx, gy, gz) {
        if (![timestamp, ax, ay, az, gx, gy, gz].every(Number.isFinite)) {
            this._diagnostics.drops.nonfinite++; return false;
        }
        if (timestamp < 0 || timestamp <= this._lastSampleTime) {
            this._diagnostics.drops.outOfOrder++; return false;
        }
        this._lastSampleTime = timestamp;
        if (this._writeIdx > this._readIdx) {
            const oldest = this._ring[(this._readIdx % RING_CAPACITY) * FIELDS_PER_READING];
            if (timestamp - oldest > 5) {
                this._diagnostics.drops.stale += this._writeIdx - this._readIdx;
                this._readIdx = this._writeIdx;
            }
        }
        if (this._writeIdx - this._readIdx >= RING_CAPACITY) {
            this._readIdx++;
            this._diagnostics.drops.overflow++;
        }

        // Subtract calibrated gyroscope bias
        // Mobile MEMS gyros have persistent bias offsets that cause
        // VIO pre-integration drift if not compensated.
        const bx = this._gyroBias.x;
        const by = this._gyroBias.y;
        const bz = this._gyroBias.z;

        const slot = (this._writeIdx % RING_CAPACITY) * FIELDS_PER_READING;
        this._ring[slot + 0] = timestamp;
        this._ring[slot + 1] = ax;
        this._ring[slot + 2] = ay;
        this._ring[slot + 3] = az;
        this._ring[slot + 4] = gx - bx;
        this._ring[slot + 5] = gy - by;
        this._ring[slot + 6] = gz - bz;
        this._writeIdx++;
        this._diagnostics.emitted++;
        this._diagnostics.status = 'streaming';

        // Update latest for UI display (bias-corrected gyro)
        this.latest.acc_x = ax;
        this.latest.acc_y = ay;
        this.latest.acc_z = az;
        this.latest.gyro_x = gx - bx;
        this.latest.gyro_y = gy - by;
        this.latest.gyro_z = gz - bz;

        // Rate measurement — reuse the timestamp already computed by the
        // caller (converting from seconds back to ms) to avoid a second
        // performance.now() call per sample.
        const nowMs = timestamp * 1000.0;
        if (this._rateStartTime === null) this._rateStartTime = nowMs;
        else this._rateCount++;
        const elapsed = nowMs - this._rateStartTime;
        if (elapsed >= 1000) {
            this._currentRate = (this._rateCount / elapsed) * 1000;
            this._rateCount = 0;
            this._rateStartTime = nowMs;
        }
    }

    /**
     * Flush all buffered IMU readings since the last flush.
     * Returns a Float64Array that can be transferred to a worker (zero-copy).
     *
     * @returns {{ data: Float64Array, count: number }}
     *   data: Flat array of [timestamp, ax, ay, az, gx, gy, gz] × count
     *   count: Number of IMU readings
     */
    flush() {
        const available = this._writeIdx - this._readIdx;
        if (available <= 0) {
            return { data: null, count: 0 };
        }

        // Clamp to ring capacity (if buffer overflowed, skip oldest)
        const count = Math.min(available, RING_CAPACITY);
        const startIdx = this._writeIdx - count;

        // Copy readings into a new transferable Float64Array
        const result = new Float64Array(count * FIELDS_PER_READING);
        for (let i = 0; i < count; i++) {
            const srcSlot = ((startIdx + i) % RING_CAPACITY) * FIELDS_PER_READING;
            const dstSlot = i * FIELDS_PER_READING;
            result[dstSlot + 0] = this._ring[srcSlot + 0];
            result[dstSlot + 1] = this._ring[srcSlot + 1];
            result[dstSlot + 2] = this._ring[srcSlot + 2];
            result[dstSlot + 3] = this._ring[srcSlot + 3];
            result[dstSlot + 4] = this._ring[srcSlot + 4];
            result[dstSlot + 5] = this._ring[srcSlot + 5];
            result[dstSlot + 6] = this._ring[srcSlot + 6];
        }

        this._readIdx = this._writeIdx;
        this._diagnostics.flushed += count;
        return { data: result, count };
    }

    /**
     * Get the current measured IMU data rate in Hz.
     * @returns {number}
     */
    getRate() {
        return this._currentRate;
    }

    /**
     * Get the sensor API type currently in use.
     * @returns {string} One of SensorType values
     */
    getSensorType() {
        return this._sensorType;
    }

    /**
     * Check if IMU is available on this device.
     * @returns {boolean}
     */
    static isAvailable() {
        return (typeof Accelerometer !== 'undefined' && typeof Gyroscope !== 'undefined') ||
               typeof DeviceMotionEvent !== 'undefined';
    }

    /** Stop generic sensor API handles. @private */
    _stopGenericSensor() {
        if (this._accel) {
            try { this._accel.stop(); } catch (_) {}
            this._accel = null;
        }
        if (this._gyro) {
            try { this._gyro.stop(); } catch (_) {}
            this._gyro = null;
        }
    }

    /** Stop capturing IMU data. */
    stop() {
        this.running = false;
        this.discard('stopped');

        this._stopGenericSensor();

        if (this._motionHandler) {
            window.removeEventListener('devicemotion', this._motionHandler, true);
            this._motionHandler = null;
        }

        this._writeIdx = 0;
        this._readIdx = 0;
        this._lastMotionTimestamp = -Infinity;
        this._lastGenericTimestamp = -Infinity;
        this._lastSampleTime = -Infinity;
        this._pendingGyro = null;
        this._latestAccel = null;
        this._latestGyro = null;
        this._sensorType = SensorType.NONE;
        this._currentRate = 0;
        this._gyroBias = { x: 0, y: 0, z: 0 };
        this._calibrated = false;
        this._gravityEstimate = null;
        this._gravityEstimateCount = 0;
        this._gravitySumX = 0;
        this._gravitySumY = 0;
        this._gravitySumZ = 0;
    }
}

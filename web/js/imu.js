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
/** Default sensor frequency to request (Hz) */
const DEFAULT_FREQUENCY = 100;

const DEG_TO_RAD = Math.PI / 180;

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
        // Latest gyro reading (accel callback writes the combined sample)
        this._latestGyro = { x: 0, y: 0, z: 0 };

        // DeviceMotionEvent handler
        this._motionHandler = null;
        this._lastMotionTimestamp = 0;

        // Generic Sensor timestamp monotonicity guard
        this._lastGenericTimestamp = 0;

        // Platform detection: iOS inverts accelerationIncludingGravity signs
        // iOS Safari: stationary phone reports acc_y ~= -9.81 (gravity opposes +Y)
        // Android Generic Sensor: stationary phone reports acc_y ~= +9.81
        // We need consistent convention: gravity along +Y when phone upright
        this._isIOS = isIOS();
        this._iosAccSign = this._isIOS ? -1 : 1;

        // Rate measurement
        this._rateCount = 0;
        this._rateStartTime = 0;
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
    }

    /**
     * Request device motion permission (required for iOS 13+).
     * Must be called from a user gesture (e.g., button click).
     * Also checks Generic Sensor API permissions on Android.
     * @returns {Promise<boolean>} True if permission granted
     */
    async requestPermission() {
        // iOS 13+ DeviceMotionEvent permission
        if (typeof DeviceMotionEvent !== 'undefined' &&
            typeof DeviceMotionEvent.requestPermission === 'function') {
            try {
                const permission = await DeviceMotionEvent.requestPermission();
                return permission === 'granted';
            } catch (e) {
                console.error('[IMU] iOS permission request failed:', e);
                return false;
            }
        }

        // Generic Sensor API permission check (Android Chrome)
        if (typeof Accelerometer !== 'undefined') {
            try {
                const results = await Promise.all([
                    navigator.permissions.query({ name: 'accelerometer' }),
                    navigator.permissions.query({ name: 'gyroscope' }),
                ]);
                if (results[0].state === 'denied' || results[1].state === 'denied') {
                    console.warn('[IMU] Sensor permissions denied');
                    return false;
                }
                return true;
            } catch (_) {
                // Permissions API not available; assume granted
                return true;
            }
        }

        // Legacy: no permission needed
        return true;
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

        // Validate: gravity magnitude should be ~9.81 if stationary
        if (gravMag < 8.5 || gravMag > 11.0) {
            console.warn(`[IMU] Calibration suspect: |acc|=${gravMag.toFixed(2)} (expected ~9.81). Device may be moving.`);
        }

        this._gyroBias = bias;
        this._calibrated = true;

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
     * Start capturing IMU data.
     * Prefers DeviceMotionEvent (synchronized accel+gyro in single event),
     * falls back to Generic Sensor API if DeviceMotion is unavailable.
     *
     * Rationale: Generic Sensor API fires Accelerometer and Gyroscope
     * independently, causing temporal misalignment between accel and gyro
     * readings. DeviceMotionEvent provides both from the same measurement
     * instant, which is critical for VIO pre-integration accuracy.
     * (Reference: 8th Wall uses DeviceMotionEvent exclusively)
     *
     * @param {number} [frequency=100] Requested sensor frequency in Hz
     */
    start(frequency = DEFAULT_FREQUENCY) {
        if (this.running) return;

        this._frequency = frequency;
        this._writeIdx = 0;
        this._readIdx = 0;
        this._rateCount = 0;
        this._rateStartTime = performance.now();
        this._currentRate = 0;
        this.running = true;

        // Try DeviceMotionEvent first (synchronized accel+gyro in single event)
        if (this._tryDeviceMotion()) {
            this._sensorType = SensorType.DEVICE_MOTION;
            console.log('[IMU] Using DeviceMotionEvent (synchronized accel+gyro)');
            return;
        }

        // Fallback to Generic Sensor API (Chrome Android 67+)
        // WARNING: accel and gyro fire independently — temporal misalignment possible
        if (this._tryGenericSensor(frequency)) {
            this._sensorType = SensorType.GENERIC_SENSOR;
            console.warn(`[IMU] Fallback to Generic Sensor API @ ${frequency}Hz (accel-gyro may desync)`);
            return;
        }

        console.error('[IMU] No IMU sensor API available');
        this.running = false;
        this._sensorType = SensorType.NONE;
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
            this._accel = new Accelerometer({ frequency, referenceFrame: 'device' });
            this._gyro = new Gyroscope({ frequency, referenceFrame: 'device' });

            // Gyroscope: just cache latest values (it fires at the same rate)
            this._gyro.addEventListener('reading', () => {
                this._latestGyro.x = this._gyro.x;
                this._latestGyro.y = this._gyro.y;
                this._latestGyro.z = this._gyro.z;
            });

            // Accelerometer: write combined sample on each reading
            this._accel.addEventListener('reading', () => {
                if (!this.running) return;

                const timestamp = performance.now() / 1000.0;

                // Monotonicity guard: reject out-of-order or duplicate timestamps
                // (prevents negative dt in VIO pre-integration → divergence)
                if (timestamp <= this._lastGenericTimestamp) return;
                this._lastGenericTimestamp = timestamp;

                // Generic Sensor API Accelerometer includes gravity by default
                // and reports in m/s^2. Gyroscope reports in rad/s.
                // WARNING: gyro values are cached from a separate sensor and may
                // be from a slightly different measurement instant (~10ms at 100Hz).
                this._pushSample(
                    timestamp,
                    this._accel.x, this._accel.y, this._accel.z,
                    this._latestGyro.x, this._latestGyro.y, this._latestGyro.z
                );
            });

            this._accel.addEventListener('error', (e) => {
                console.warn('[IMU] Accelerometer error:', e.error.message);
                // Fall back to DeviceMotionEvent
                this._stopGenericSensor();
                if (this.running && this._sensorType !== SensorType.DEVICE_MOTION) {
                    if (this._tryDeviceMotion()) {
                        this._sensorType = SensorType.DEVICE_MOTION;
                        console.log('[IMU] Fell back to DeviceMotionEvent');
                    }
                }
            });

            this._gyro.addEventListener('error', (e) => {
                console.warn('[IMU] Gyroscope error:', e.error.message);
            });

            this._accel.start();
            this._gyro.start();
            return true;
        } catch (e) {
            console.warn('[IMU] Generic Sensor API failed to start:', e.message);
            this._stopGenericSensor();
            return false;
        }
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

        this._lastMotionTimestamp = 0;

        this._motionHandler = (event) => {
            if (!this.running) return;

            const acc = event.accelerationIncludingGravity;
            const rot = event.rotationRate;
            if (!acc || !rot) return;

            const timestamp = performance.now() / 1000.0;

            // Monotonicity check
            if (timestamp <= this._lastMotionTimestamp) return;
            this._lastMotionTimestamp = timestamp;

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
        };

        window.addEventListener('devicemotion', this._motionHandler, true);
        return true;
    }

    /**
     * Push a single IMU sample into the ring buffer.
     * @private
     */
    _pushSample(timestamp, ax, ay, az, gx, gy, gz) {
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

        // Update latest for UI display (bias-corrected gyro)
        this.latest.acc_x = ax;
        this.latest.acc_y = ay;
        this.latest.acc_z = az;
        this.latest.gyro_x = gx - bx;
        this.latest.gyro_y = gy - by;
        this.latest.gyro_z = gz - bz;

        // Rate measurement
        this._rateCount++;
        const now = performance.now();
        const elapsed = now - this._rateStartTime;
        if (elapsed >= 1000) {
            this._currentRate = (this._rateCount / elapsed) * 1000;
            this._rateCount = 0;
            this._rateStartTime = now;
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
        return typeof Accelerometer !== 'undefined' ||
               typeof Gyroscope !== 'undefined' ||
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

        this._stopGenericSensor();

        if (this._motionHandler) {
            window.removeEventListener('devicemotion', this._motionHandler, true);
            this._motionHandler = null;
        }

        this._writeIdx = 0;
        this._readIdx = 0;
        this._lastMotionTimestamp = 0;
        this._lastGenericTimestamp = 0;
        this._sensorType = SensorType.NONE;
        this._currentRate = 0;
        this._gyroBias = { x: 0, y: 0, z: 0 };
        this._calibrated = false;
    }
}

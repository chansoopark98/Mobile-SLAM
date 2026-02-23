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
     * Start capturing IMU data.
     * Tries Generic Sensor API first, falls back to DeviceMotionEvent.
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

        // Try Generic Sensor API first (Chrome Android 67+)
        if (this._tryGenericSensor(frequency)) {
            this._sensorType = SensorType.GENERIC_SENSOR;
            console.log(`[IMU] Using Generic Sensor API @ ${frequency}Hz requested`);
            return;
        }

        // Fallback to DeviceMotionEvent
        if (this._tryDeviceMotion()) {
            this._sensorType = SensorType.DEVICE_MOTION;
            console.log('[IMU] Using DeviceMotionEvent fallback (browser-controlled rate)');
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

                // Generic Sensor API Accelerometer includes gravity by default
                // and reports in m/s^2. Gyroscope reports in rad/s.
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
        const slot = (this._writeIdx % RING_CAPACITY) * FIELDS_PER_READING;
        this._ring[slot + 0] = timestamp;
        this._ring[slot + 1] = ax;
        this._ring[slot + 2] = ay;
        this._ring[slot + 3] = az;
        this._ring[slot + 4] = gx;
        this._ring[slot + 5] = gy;
        this._ring[slot + 6] = gz;
        this._writeIdx++;

        // Update latest for UI display
        this.latest.acc_x = ax;
        this.latest.acc_y = ay;
        this.latest.acc_z = az;
        this.latest.gyro_x = gx;
        this.latest.gyro_y = gy;
        this.latest.gyro_z = gz;

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
        this._sensorType = SensorType.NONE;
        this._currentRate = 0;
    }
}

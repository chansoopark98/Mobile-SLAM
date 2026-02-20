/**
 * IMU capture module using DeviceMotionEvent and DeviceOrientationEvent.
 * Buffers readings between camera frames for batch processing by VIO.
 */
export class IMU {
    constructor() {
        this.buffer = [];
        this.running = false;
        this._motionHandler = null;
        this.startTime = 0;
    }

    /**
     * Request device motion permission (required for iOS 13+).
     * Must be called from a user gesture (e.g., button click).
     * @returns {Promise<boolean>} True if permission granted
     */
    async requestPermission() {
        if (typeof DeviceMotionEvent !== 'undefined' &&
            typeof DeviceMotionEvent.requestPermission === 'function') {
            try {
                const permission = await DeviceMotionEvent.requestPermission();
                return permission === 'granted';
            } catch (e) {
                console.error('IMU permission request failed:', e);
                return false;
            }
        }
        // Non-iOS: permission not needed
        return true;
    }

    /**
     * Start capturing IMU data.
     * Readings are buffered and can be drained with flush().
     */
    start() {
        if (this.running) return;

        this.startTime = performance.now() / 1000.0;
        this.buffer = [];
        this.running = true;

        this._motionHandler = (event) => {
            if (!this.running) return;

            const acc = event.accelerationIncludingGravity;
            const rot = event.rotationRate;

            if (!acc || !rot) return;

            const timestamp = performance.now() / 1000.0;

            this.buffer.push({
                timestamp: timestamp,
                acc_x: acc.x || 0,
                acc_y: acc.y || 0,
                acc_z: acc.z || 0,
                // DeviceMotion rotationRate: alpha=z-axis, beta=x-axis, gamma=y-axis
                gyro_x: (rot.beta || 0) * Math.PI / 180,   // deg/s -> rad/s
                gyro_y: (rot.gamma || 0) * Math.PI / 180,
                gyro_z: (rot.alpha || 0) * Math.PI / 180,
            });
        };

        window.addEventListener('devicemotion', this._motionHandler, true);
    }

    /**
     * Flush and return all buffered IMU readings since last flush.
     * @returns {Array<{timestamp, acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z}>}
     */
    flush() {
        const readings = this.buffer;
        this.buffer = [];
        return readings;
    }

    /**
     * Check if IMU is available on this device.
     * @returns {boolean}
     */
    static isAvailable() {
        return typeof DeviceMotionEvent !== 'undefined';
    }

    /** Stop capturing IMU data. */
    stop() {
        this.running = false;
        if (this._motionHandler) {
            window.removeEventListener('devicemotion', this._motionHandler, true);
            this._motionHandler = null;
        }
        this.buffer = [];
    }
}

/**
 * VIOWrapper - High-level JavaScript API wrapping the WASM VIO engine.
 * Manages SharedMemory buffers and provides a clean interface for the app.
 */
import { SharedMemory } from './shared-memory.js';

export class VIOWrapper {
    constructor() {
        this.wasm = null;
        this.engine = null;
        this.configured = false;

        // Pre-allocated shared memory buffers
        this.memImage = null;
        this.memIMU = null;
        this.memPose = null;
        this.memMapPoints = null;
        this.memExtrinsicR = null;
        this.memExtrinsicT = null;

        this.imageWidth = 0;
        this.imageHeight = 0;
        this.maxIMUReadings = 256;
        this.maxMapPoints = 2000;
    }

    /**
     * Load and initialize the WASM module.
     * @param {string} wasmPath - Path to the vio_engine.js module
     * @returns {Promise<void>}
     */
    async load(wasmPath = './vio_engine.js') {
        const VIOWasm = (await import(wasmPath)).default;
        this.wasm = await VIOWasm();
        this.engine = new this.wasm.VIOEngine();
    }

    /**
     * Configure the VIO engine with camera and IMU parameters.
     * @param {Object} params - Configuration parameters
     * @param {number} params.width - Image width
     * @param {number} params.height - Image height
     * @param {number} params.fx - Focal length x
     * @param {number} params.fy - Focal length y
     * @param {number} params.cx - Principal point x
     * @param {number} params.cy - Principal point y
     * @param {number} [params.modelType=0] - Camera model (0=PINHOLE, 1=KANNALA_BRANDT)
     * @param {number} [params.k2=0] - Distortion k2
     * @param {number} [params.k3=0] - Distortion k3
     * @param {number} [params.k4=0] - Distortion k4
     * @param {number} [params.k5=0] - Distortion k5
     * @param {number[]} [params.r_ic] - Extrinsic rotation (9 doubles, row-major)
     * @param {number[]} [params.t_ic] - Extrinsic translation (3 doubles)
     * @param {number} [params.acc_n=0.08] - Accelerometer noise
     * @param {number} [params.acc_w=0.004] - Accelerometer random walk
     * @param {number} [params.gyr_n=0.004] - Gyroscope noise
     * @param {number} [params.gyr_w=0.0002] - Gyroscope random walk
     * @param {number} [params.g_norm=9.81] - Gravity magnitude
     * @returns {boolean} True if configuration succeeded
     */
    configure(params) {
        if (!this.wasm || !this.engine) {
            throw new Error('WASM module not loaded. Call load() first.');
        }

        this.imageWidth = params.width;
        this.imageHeight = params.height;

        // Allocate shared memory buffers
        this._allocateBuffers();

        // Write extrinsic parameters
        const r_ic = params.r_ic || [1, 0, 0, 0, 1, 0, 0, 0, 1]; // identity
        const t_ic = params.t_ic || [0, 0, 0];
        this.memExtrinsicR.write(new Float64Array(r_ic));
        this.memExtrinsicT.write(new Float64Array(t_ic));

        const result = this.engine.configure(
            params.width, params.height,
            params.fx, params.fy, params.cx, params.cy,
            params.modelType || 0,
            params.k2 || 0, params.k3 || 0, params.k4 || 0, params.k5 || 0,
            this.memExtrinsicR.byteOffset,
            this.memExtrinsicT.byteOffset,
            params.acc_n || 0.08,
            params.acc_w || 0.004,
            params.gyr_n || 0.004,
            params.gyr_w || 0.0002,
            params.g_norm || 9.81
        );

        this.configured = result;
        return result;
    }

    /** Allocate WASM heap buffers */
    _allocateBuffers() {
        this._disposeBuffers();

        // Grayscale image buffer
        this.memImage = new SharedMemory(
            this.wasm, this.wasm.HEAPU8,
            this.imageWidth * this.imageHeight
        );

        // IMU readings buffer (7 doubles per reading: timestamp, acc_xyz, gyro_xyz)
        this.memIMU = new SharedMemory(
            this.wasm, this.wasm.HEAPF64,
            this.maxIMUReadings * 7
        );

        // Pose output buffer (16 doubles for 4x4 matrix)
        this.memPose = new SharedMemory(
            this.wasm, this.wasm.HEAPF64,
            16
        );

        // Map points buffer (3 doubles per point)
        this.memMapPoints = new SharedMemory(
            this.wasm, this.wasm.HEAPF64,
            this.maxMapPoints * 3
        );

        // Extrinsic parameters
        this.memExtrinsicR = new SharedMemory(this.wasm, this.wasm.HEAPF64, 9);
        this.memExtrinsicT = new SharedMemory(this.wasm, this.wasm.HEAPF64, 3);
    }

    /**
     * Process a camera frame with IMU readings.
     * @param {Uint8Array} grayImage - Grayscale image data
     * @param {Array<Object>} imuReadings - Array of {timestamp, acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z}
     * @returns {{pose: Float64Array|null, initialized: boolean, featureCount: number}}
     */
    processFrame(grayImage, imuReadings = [], imageTimestamp = 0) {
        if (!this.configured) {
            throw new Error('VIO not configured. Call configure() first.');
        }

        // Write image to WASM heap
        this.memImage.write(grayImage);

        // Write IMU readings to WASM heap (packed as 7 doubles per reading)
        const imuCount = Math.min(imuReadings.length, this.maxIMUReadings);
        if (imuCount > 0) {
            const imuData = new Float64Array(imuCount * 7);
            for (let i = 0; i < imuCount; i++) {
                const r = imuReadings[i];
                imuData[i * 7 + 0] = r.timestamp;
                imuData[i * 7 + 1] = r.acc_x;
                imuData[i * 7 + 2] = r.acc_y;
                imuData[i * 7 + 3] = r.acc_z;
                imuData[i * 7 + 4] = r.gyro_x;
                imuData[i * 7 + 5] = r.gyro_y;
                imuData[i * 7 + 6] = r.gyro_z;
            }
            this.memIMU.write(imuData);
        }

        // Call WASM processFrame with explicit image timestamp
        const hasPose = this.engine.processFrame(
            this.memImage.byteOffset,
            this.imageWidth,
            this.imageHeight,
            this.memIMU.byteOffset,
            imuCount,
            imageTimestamp,
            this.memPose.byteOffset
        );

        const result = {
            pose: null,
            initialized: this.engine.isInitialized(),
            featureCount: this.engine.getFeaturePointCount(),
        };

        if (hasPose) {
            result.pose = new Float64Array(this.memPose.read(16));
        }

        return result;
    }

    /**
     * Get 3D map points from the VIO engine.
     * @returns {{points: Float64Array, count: number}}
     */
    getMapPoints() {
        if (!this.configured) return { points: null, count: 0 };

        const count = this.engine.getMapPoints(
            this.memMapPoints.byteOffset,
            this.maxMapPoints
        );

        return {
            points: count > 0 ? new Float64Array(this.memMapPoints.read(count * 3)) : null,
            count: count,
        };
    }

    /** Check if VIO has initialized */
    isInitialized() {
        return this.engine ? this.engine.isInitialized() : false;
    }

    /** Reset VIO state */
    reset() {
        if (this.engine) {
            this.engine.reset();
        }
    }

    /** Free all shared memory */
    _disposeBuffers() {
        [this.memImage, this.memIMU, this.memPose, this.memMapPoints,
         this.memExtrinsicR, this.memExtrinsicT].forEach(m => {
            if (m) m.dispose();
        });
    }

    /** Clean up */
    dispose() {
        this._disposeBuffers();
        if (this.engine) {
            this.engine.delete();
            this.engine = null;
        }
        this.wasm = null;
        this.configured = false;
    }
}

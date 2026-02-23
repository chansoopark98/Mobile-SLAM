/**
 * VIOWrapper - Async Worker-based API wrapping the WASM VIO engine.
 * Main thread remains unblocked; all VIO processing runs in a Web Worker.
 *
 * IMU data is sent independently from camera frames via sendIMU().
 * The worker accumulates IMU internally and drains on each frame.
 */

export class VIOWrapper {
    constructor() {
        this.worker = null;
        this.configured = false;
        this.workerBusy = false;

        // Latest result from worker (updated asynchronously)
        this._latestResult = null;
        this._latestMapPoints = null;

        // Pending promises for init/configure/setMobileParams
        this._pendingInit = null;
        this._pendingConfigure = null;
        this._pendingSetMobileParams = null;

        // WASM C++ stdout/stderr log callback: (level, msg) => {}
        this.onWasmLog = null;
    }

    /**
     * Load and initialize the WASM module inside a Web Worker.
     * @param {string} wasmPath - Path to the vio_engine_worker.js WASM module
     * @returns {Promise<void>}
     */
    async load(wasmPath = '/vio_engine.js') {
        return new Promise((resolve, reject) => {
            this.worker = new Worker('/js/vio-worker.js', { type: 'module' });

            this.worker.onmessage = (e) => {
                this._handleWorkerMessage(e.data);
            };

            this.worker.onerror = (err) => {
                console.error('[VIO Wrapper] Worker error:', err);
                if (this._pendingInit) {
                    this._pendingInit.reject(err);
                    this._pendingInit = null;
                }
            };

            this._pendingInit = { resolve, reject };
            this.worker.postMessage({ type: 'init', data: { wasmPath } });
        });
    }

    /**
     * Configure the VIO engine with camera and IMU parameters.
     * @param {Object} params - Configuration parameters
     * @returns {Promise<boolean>}
     */
    async configure(params) {
        return new Promise((resolve, reject) => {
            this._pendingConfigure = { resolve, reject };
            this.worker.postMessage({ type: 'configure', data: params });
        });
    }

    /**
     * Set mobile-optimized solver parameters (call after configure).
     * @param {number} solverTime - Max solver time in seconds
     * @param {number} numIterations - Max solver iterations
     * @param {number} maxFeatures - Max tracked features
     * @returns {Promise<boolean>}
     */
    async setMobileParams(solverTime, numIterations, maxFeatures) {
        return new Promise((resolve, reject) => {
            this._pendingSetMobileParams = { resolve, reject };
            this.worker.postMessage({
                type: 'setMobileParams',
                data: { solver_time: solverTime, num_iterations: numIterations, max_features: maxFeatures },
            });
        });
    }

    /**
     * Send IMU data to the worker independently from camera frames.
     * This is NOT blocked by workerBusy — IMU always gets through.
     *
     * @param {Float64Array} imuData - Flat array: [ts, ax, ay, az, gx, gy, gz] × count
     * @param {number} count - Number of IMU readings
     * @returns {boolean} true if sent
     */
    sendIMU(imuData, count) {
        if (!this.configured || !this.worker || !imuData || count <= 0) {
            return false;
        }

        // Transfer the buffer for zero-copy delivery
        const buffer = imuData.buffer;
        this.worker.postMessage(
            {
                type: 'imu',
                data: { imuData: buffer, count },
            },
            [buffer]  // Transferable
        );
        return true;
    }

    /**
     * Send a camera frame to the worker for processing (non-blocking).
     * If the worker is busy, the frame is dropped (but IMU is NOT lost).
     *
     * @param {Uint8Array} grayImage - Grayscale image data
     * @param {number} timestamp - Image timestamp in seconds
     * @returns {boolean} true if frame was sent, false if dropped
     */
    sendFrame(grayImage, timestamp = 0) {
        if (!this.configured || !this.worker || this.workerBusy) {
            return false;
        }

        this.workerBusy = true;

        // Transfer the image buffer for zero-copy
        const grayBuffer = grayImage.buffer.slice(
            grayImage.byteOffset,
            grayImage.byteOffset + grayImage.byteLength
        );

        this.worker.postMessage(
            {
                type: 'frame',
                data: {
                    gray: grayBuffer,
                    timestamp: timestamp,
                },
            },
            [grayBuffer]  // Transferable
        );
        return true;
    }

    /**
     * Get the latest result from the worker (synchronous, non-blocking).
     * @returns {{pose: Float64Array|null, initialized: boolean, featureCount: number}|null}
     */
    getLatestResult() {
        return this._latestResult;
    }

    /**
     * Get the latest map points from the worker.
     * @returns {{points: Float64Array|null, count: number}}
     */
    getMapPoints() {
        if (this._latestMapPoints) {
            return this._latestMapPoints;
        }
        return { points: null, count: 0 };
    }

    /** Check if VIO has initialized */
    isInitialized() {
        return this._latestResult ? this._latestResult.initialized : false;
    }

    /** Reset VIO state */
    reset() {
        if (this.worker) {
            this.workerBusy = false;
            this._latestResult = null;
            this._latestMapPoints = null;
            this.worker.postMessage({ type: 'reset' });
        }
    }

    /** Clean up worker and resources */
    dispose() {
        if (this.worker) {
            this.worker.postMessage({ type: 'dispose' });
            this.worker.terminate();
            this.worker = null;
        }
        this.configured = false;
        this.workerBusy = false;
        this._latestResult = null;
        this._latestMapPoints = null;
    }

    /** Handle messages from the worker */
    _handleWorkerMessage(msg) {
        switch (msg.type) {
            case 'init':
                if (this._pendingInit) {
                    if (msg.success) {
                        this._pendingInit.resolve();
                    } else {
                        this._pendingInit.reject(new Error(msg.error || 'Worker init failed'));
                    }
                    this._pendingInit = null;
                }
                break;

            case 'configure':
                this.configured = msg.success;
                if (this._pendingConfigure) {
                    this._pendingConfigure.resolve(msg.success);
                    this._pendingConfigure = null;
                }
                break;

            case 'setMobileParams':
                if (this._pendingSetMobileParams) {
                    this._pendingSetMobileParams.resolve(msg.success);
                    this._pendingSetMobileParams = null;
                }
                break;

            case 'result':
                this.workerBusy = false;
                if (msg.data) {
                    this._latestResult = {
                        pose: msg.data.pose,
                        initialized: msg.data.initialized,
                        featureCount: msg.data.featureCount,
                        statusCode: msg.data.statusCode,
                    };
                    if (msg.data.mapPoints && msg.data.mapPointCount > 0) {
                        this._latestMapPoints = {
                            points: msg.data.mapPoints,
                            count: msg.data.mapPointCount,
                        };
                    }
                }
                break;

            case 'wasm_log':
                if (this.onWasmLog && msg.data) {
                    this.onWasmLog(msg.data.level, msg.data.msg);
                }
                break;

            case 'reset':
                this.workerBusy = false;
                break;

            case 'dispose':
                break;
        }
    }
}

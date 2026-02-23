/**
 * VIO Web Worker - Runs WASM VIO processing off the main thread.
 *
 * Architecture:
 * - IMU data arrives independently via 'imu' messages and is accumulated
 *   in a worker-side ring buffer.
 * - Camera frames arrive via 'frame' messages. On each frame, all
 *   accumulated IMU readings are drained and fed to the WASM engine
 *   together with the image.
 * - This decoupling prevents IMU data loss when frames are dropped.
 */

// Worker-local state
let wasm = null;
let engine = null;
let configured = false;

// Shared memory buffers (allocated on WASM heap)
let memImage = null;
let memIMU = null;
let memPose = null;
let memMapPoints = null;
let memExtrinsicR = null;
let memExtrinsicT = null;

let imageWidth = 0;
let imageHeight = 0;
const maxIMUReadings = 512;
const maxMapPoints = 2000;

let processing = false;

// ── Worker-side IMU accumulation ring buffer ──
const IMU_FIELDS = 7;
const IMU_RING_CAPACITY = 1024;
const imuRing = new Float64Array(IMU_RING_CAPACITY * IMU_FIELDS);
let imuRingWriteIdx = 0;
let imuRingReadIdx = 0;

/**
 * Append IMU readings from a flat Float64Array into the worker ring buffer.
 * @param {Float64Array} data - Flat [ts, ax, ay, az, gx, gy, gz] × count
 * @param {number} count - Number of readings
 */
function appendIMU(data, count) {
    for (let i = 0; i < count; i++) {
        const srcBase = i * IMU_FIELDS;
        const dstSlot = (imuRingWriteIdx % IMU_RING_CAPACITY) * IMU_FIELDS;
        imuRing[dstSlot + 0] = data[srcBase + 0];
        imuRing[dstSlot + 1] = data[srcBase + 1];
        imuRing[dstSlot + 2] = data[srcBase + 2];
        imuRing[dstSlot + 3] = data[srcBase + 3];
        imuRing[dstSlot + 4] = data[srcBase + 4];
        imuRing[dstSlot + 5] = data[srcBase + 5];
        imuRing[dstSlot + 6] = data[srcBase + 6];
        imuRingWriteIdx++;
    }
}

/**
 * Drain all accumulated IMU readings into the WASM heap buffer.
 * @returns {number} Number of readings written to memIMU
 */
function drainIMUToWasm() {
    const available = imuRingWriteIdx - imuRingReadIdx;
    if (available <= 0 || !memIMU) return 0;

    // Clamp to ring capacity and max WASM buffer
    const count = Math.min(available, IMU_RING_CAPACITY, maxIMUReadings);
    const startIdx = imuRingWriteIdx - count;

    // Write directly into WASM heap (memIMU is Float64, 7 fields per reading)
    const heapOffset = memIMU.ptr / 8;  // Float64 index
    for (let i = 0; i < count; i++) {
        const srcSlot = ((startIdx + i) % IMU_RING_CAPACITY) * IMU_FIELDS;
        const dstBase = heapOffset + i * IMU_FIELDS;
        wasm.HEAPF64[dstBase + 0] = imuRing[srcSlot + 0];
        wasm.HEAPF64[dstBase + 1] = imuRing[srcSlot + 1];
        wasm.HEAPF64[dstBase + 2] = imuRing[srcSlot + 2];
        wasm.HEAPF64[dstBase + 3] = imuRing[srcSlot + 3];
        wasm.HEAPF64[dstBase + 4] = imuRing[srcSlot + 4];
        wasm.HEAPF64[dstBase + 5] = imuRing[srcSlot + 5];
        wasm.HEAPF64[dstBase + 6] = imuRing[srcSlot + 6];
    }

    imuRingReadIdx = imuRingWriteIdx;
    return count;
}

/**
 * Simple SharedMemory helper for Worker context.
 * Manages a typed array backed by WASM heap memory.
 */
class WorkerSharedMemory {
    constructor(wasmModule, heapArray, count) {
        this.wasm = wasmModule;
        this.heap = heapArray;
        this.count = count;
        this.byteSize = count * heapArray.BYTES_PER_ELEMENT;
        this.ptr = wasmModule._malloc(this.byteSize);
        this.byteOffset = this.ptr;
        if (heapArray === wasmModule.HEAPF64) {
            this.byteOffset = this.ptr / 8;  // Float64 index
        }
    }

    write(typedArray) {
        if (this.heap === this.wasm.HEAPU8) {
            this.wasm.HEAPU8.set(typedArray, this.ptr);
        } else if (this.heap === this.wasm.HEAPF64) {
            this.wasm.HEAPF64.set(typedArray, this.ptr / 8);
        }
    }

    read(count) {
        if (this.heap === this.wasm.HEAPF64) {
            return new Float64Array(this.wasm.HEAPF64.buffer, this.ptr, count);
        }
        return new Uint8Array(this.wasm.HEAPU8.buffer, this.ptr, count);
    }

    dispose() {
        if (this.ptr) {
            this.wasm._free(this.ptr);
            this.ptr = 0;
        }
    }
}

function allocateBuffers() {
    disposeBuffers();

    memImage = new WorkerSharedMemory(wasm, wasm.HEAPU8, imageWidth * imageHeight);
    memIMU = new WorkerSharedMemory(wasm, wasm.HEAPF64, maxIMUReadings * IMU_FIELDS);
    memPose = new WorkerSharedMemory(wasm, wasm.HEAPF64, 16);
    memMapPoints = new WorkerSharedMemory(wasm, wasm.HEAPF64, maxMapPoints * 3);
    memExtrinsicR = new WorkerSharedMemory(wasm, wasm.HEAPF64, 9);
    memExtrinsicT = new WorkerSharedMemory(wasm, wasm.HEAPF64, 3);
}

function disposeBuffers() {
    [memImage, memIMU, memPose, memMapPoints, memExtrinsicR, memExtrinsicT].forEach(m => {
        if (m) m.dispose();
    });
    memImage = memIMU = memPose = memMapPoints = memExtrinsicR = memExtrinsicT = null;
}

function processFrame(gray, timestamp) {
    if (!configured || !engine) return null;

    // Write image to WASM heap
    memImage.write(gray);

    // Drain accumulated IMU readings into WASM heap
    const imuCount = drainIMUToWasm();

    // Process frame
    let hasPose = false;
    try {
        hasPose = engine.processFrame(
            memImage.ptr,
            imageWidth, imageHeight,
            memIMU.ptr,
            imuCount,
            timestamp,
            memPose.ptr
        );
    } catch (e) {
        console.error('[VIO Worker] processFrame error:', e.message);
        try {
            engine.reset();
        } catch (resetErr) {
            console.error('[VIO Worker] reset also failed:', resetErr.message);
        }
        return { pose: null, initialized: false, featureCount: 0, statusCode: 3, mapPoints: null, mapPointCount: 0 };
    }

    const result = {
        pose: null,
        initialized: engine.isInitialized(),
        featureCount: engine.getFeaturePointCount(),
        statusCode: engine.getStatusCode(),
        mapPoints: null,
        mapPointCount: 0,
    };

    if (hasPose) {
        const poseData = memPose.read(16);
        result.pose = new Float64Array(poseData);
    }

    // Get map points
    try {
        const count = engine.getMapPoints(memMapPoints.ptr, maxMapPoints);
        if (count > 0) {
            const pointsData = memMapPoints.read(count * 3);
            result.mapPoints = new Float64Array(pointsData);
            result.mapPointCount = count;
        }
    } catch (e) {
        // Non-fatal
    }

    return result;
}

// Message handler
self.onmessage = async function(e) {
    const { type, data } = e.data;

    switch (type) {
        case 'init': {
            try {
                // Try importScripts first (non-ES6 worker build),
                // fall back to dynamic import() for ES6 module builds
                let VIOWasmFactory;
                try {
                    importScripts(data.wasmPath);
                    VIOWasmFactory = self.VIOWasm || VIOWasm;
                } catch (_) {
                    const module = await import(data.wasmPath);
                    VIOWasmFactory = module.default;
                }
                wasm = await VIOWasmFactory();
                engine = new wasm.VIOEngine();
                self.postMessage({ type: 'init', success: true });
            } catch (err) {
                self.postMessage({ type: 'init', success: false, error: err.message });
            }
            break;
        }

        case 'configure': {
            try {
                const p = data;
                imageWidth = p.width;
                imageHeight = p.height;

                allocateBuffers();

                // Reset IMU accumulation
                imuRingWriteIdx = 0;
                imuRingReadIdx = 0;

                const r_ic = p.r_ic || [1, 0, 0, 0, 1, 0, 0, 0, 1];
                const t_ic = p.t_ic || [0, 0, 0];
                memExtrinsicR.write(new Float64Array(r_ic));
                memExtrinsicT.write(new Float64Array(t_ic));

                const result = engine.configure(
                    p.width, p.height,
                    p.fx, p.fy, p.cx, p.cy,
                    p.modelType || 0,
                    p.k2 || 0, p.k3 || 0, p.k4 || 0, p.k5 || 0,
                    memExtrinsicR.ptr,
                    memExtrinsicT.ptr,
                    p.acc_n || 0.08,
                    p.acc_w || 0.004,
                    p.gyr_n || 0.004,
                    p.gyr_w || 0.0002,
                    p.g_norm || 9.81
                );

                configured = result;
                self.postMessage({ type: 'configure', success: result });
            } catch (err) {
                self.postMessage({ type: 'configure', success: false, error: err.message });
            }
            break;
        }

        case 'setMobileParams': {
            try {
                const { solver_time, num_iterations, max_features } = data;
                engine.setMobileParams(solver_time, num_iterations, max_features);
                self.postMessage({ type: 'setMobileParams', success: true });
            } catch (err) {
                self.postMessage({ type: 'setMobileParams', success: false, error: err.message });
            }
            break;
        }

        case 'imu': {
            // Independent IMU delivery — accumulate in worker ring buffer
            if (configured && data.imuData && data.count > 0) {
                const imuArray = new Float64Array(data.imuData);
                appendIMU(imuArray, data.count);
            }
            break;
        }

        case 'frame': {
            if (processing) {
                // Drop frame if still processing previous one
                // IMU data is NOT lost — it stays in the accumulation buffer
                return;
            }
            processing = true;
            try {
                const { gray, timestamp } = data;
                const grayArray = new Uint8Array(gray);
                const result = processFrame(grayArray, timestamp);
                self.postMessage({
                    type: 'result',
                    data: result,
                });
            } catch (err) {
                self.postMessage({
                    type: 'result',
                    data: { pose: null, initialized: false, featureCount: 0, statusCode: 3, mapPoints: null, mapPointCount: 0 },
                });
            }
            processing = false;
            break;
        }

        case 'reset': {
            if (engine) {
                try { engine.reset(); } catch (e) {}
            }
            processing = false;
            imuRingWriteIdx = 0;
            imuRingReadIdx = 0;
            self.postMessage({ type: 'reset', success: true });
            break;
        }

        case 'dispose': {
            disposeBuffers();
            if (engine) {
                try { engine.delete(); } catch (e) {}
                engine = null;
            }
            wasm = null;
            configured = false;
            processing = false;
            imuRingWriteIdx = 0;
            imuRingReadIdx = 0;
            self.postMessage({ type: 'dispose', success: true });
            break;
        }
    }
};

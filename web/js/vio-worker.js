/** Single-engine module Worker. Engine owns future IMU carry and pose freshness. */
let wasm = null;
let engine = null;
let configured = false;
let processing = false;
let clientEpoch = 0;
let lastSequence = 0;
let engineEpoch = null;
let activeRequest = null;
let frameWait = null;
const IMU_BRACKET_WAIT_MS = 40;
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
const IMU_FIELDS = 7;
const IMU_RING_CAPACITY = 1024;
const MAX_IMU_AGE_S = 0.5;
const imuRing = new Float64Array(IMU_RING_CAPACITY * IMU_FIELDS);
let imuRingWriteIdx = 0;
let imuRingReadIdx = 0;
let lastIMUTimestamp = null;
let intervalLoss = null;
const imuDrops = {
    received: 0, accepted: 0, overflow: 0, stale: 0, capacity: 0,
    invalid: 0, order: 0, reset: 0, invalidBatches: 0, gapCount: 0, lastGapSeconds: null, lossResets: 0,
};

function clearIMURing(resetCounters = false) {
    if (resetCounters) {
        for (const key of Object.keys(imuDrops)) imuDrops[key] = key === 'lastGapSeconds' ? null : 0;
    } else imuDrops.reset += imuRingWriteIdx - imuRingReadIdx;
    imuRingWriteIdx = imuRingReadIdx = 0;
    lastIMUTimestamp = null;
    intervalLoss = null;
}

function recordIntervalLoss(reason, count, firstTimestamp, lastTimestamp) {
    if (count <= 0) return;
    if (!intervalLoss) intervalLoss = { count: 0, reasons: { overflow: 0, stale: 0, capacity: 0 }, droppedTimestampMin: firstTimestamp, droppedTimestampMax: lastTimestamp };
    intervalLoss.count += count;
    intervalLoss.reasons[reason] += count;
    intervalLoss.droppedTimestampMin = Math.min(intervalLoss.droppedTimestampMin, firstTimestamp);
    intervalLoss.droppedTimestampMax = Math.max(intervalLoss.droppedTimestampMax, lastTimestamp);
}

function finishBracketWait(wait, cancelled = false) {
    if (frameWait !== wait) return;
    clearTimeout(wait.timer);
    frameWait = null;
    wait.resolve({ cancelled, imuWaitMs: performance.now() - wait.startedAtMs,
        bracket: !cancelled && lastIMUTimestamp !== null && lastIMUTimestamp >= wait.timestamp &&
            performance.now() - wait.startedAtMs <= IMU_BRACKET_WAIT_MS });
}

function cancelBracketWait() {
    if (!frameWait) return;
    const waitingRequest = frameWait.request;
    finishBracketWait(frameWait, true);
    if (activeRequest === waitingRequest) { activeRequest = null; processing = false; }
}

function waitForBracket(request, timestamp) {
    if (lastIMUTimestamp !== null && lastIMUTimestamp >= timestamp) return Promise.resolve({ bracket: true, cancelled: false, imuWaitMs: 0 });
    return new Promise(resolve => {
        const wait = { request, timestamp, resolve, startedAtMs: performance.now(), timer: null };
        wait.timer = setTimeout(() => finishBracketWait(wait), IMU_BRACKET_WAIT_MS);
        frameWait = wait;
    });
}

/** Bound read cursor on each append; reject malformed/order-invalid samples explicitly. */
function appendIMU(data, count) {
    if (Number.isInteger(count) && count >= 0 && count <= 4096) imuDrops.received += count;
    if (!(data instanceof Float64Array) || !Number.isInteger(count) || count < 0 || count > 4096 || count * IMU_FIELDS > data.length) {
        imuDrops.invalidBatches++;
        imuDrops.invalid += Number.isInteger(count) && count > 0 && count <= 4096 ? count : 0;
        return false;
    }
    let valid = true;
    for (let i = 0; i < count; i++) {
        const base = i * IMU_FIELDS;
        let finite = true;
        for (let j = 0; j < IMU_FIELDS; j++) finite = finite && Number.isFinite(data[base + j]);
        if (!finite) { imuDrops.invalid++; valid = false; continue; }
        const timestamp = data[base];
        if (lastIMUTimestamp !== null && timestamp <= lastIMUTimestamp) {
            imuDrops.order++;
            valid = false;
            continue;
        }
        if (lastIMUTimestamp !== null && timestamp - lastIMUTimestamp > MAX_IMU_AGE_S) {
            imuDrops.gapCount++;
            imuDrops.lastGapSeconds = timestamp - lastIMUTimestamp;
        }
        lastIMUTimestamp = timestamp;
        if (imuRingWriteIdx - imuRingReadIdx === IMU_RING_CAPACITY) {
            const droppedTimestamp = imuRing[(imuRingReadIdx % IMU_RING_CAPACITY) * IMU_FIELDS];
            recordIntervalLoss('overflow', 1, droppedTimestamp, droppedTimestamp);
            imuRingReadIdx++;
            imuDrops.overflow++;
        }
        const slot = (imuRingWriteIdx % IMU_RING_CAPACITY) * IMU_FIELDS;
        imuRing.set(data.subarray(base, base + IMU_FIELDS), slot);
        imuRingWriteIdx++;
        imuDrops.accepted++;
    }
    if (frameWait && lastIMUTimestamp !== null && lastIMUTimestamp >= frameWait.timestamp) finishBracketWait(frameWait);
    return valid;
}

/** Send at most one future bracket once. Only Engine retains that bracket afterward. */
function drainIMUToWasm(frameTs) {
    if (!memIMU || !wasm || !Number.isFinite(frameTs)) return 0;
    const cutoff = frameTs - MAX_IMU_AGE_S;
    while (imuRingReadIdx < imuRingWriteIdx && imuRing[(imuRingReadIdx % IMU_RING_CAPACITY) * IMU_FIELDS] < cutoff) {
        const droppedTimestamp = imuRing[(imuRingReadIdx % IMU_RING_CAPACITY) * IMU_FIELDS];
        recordIntervalLoss('stale', 1, droppedTimestamp, droppedTimestamp);
        imuRingReadIdx++;
        imuDrops.stale++;
    }
    let endIdx = imuRingReadIdx;
    while (endIdx < imuRingWriteIdx) {
        const timestamp = imuRing[(endIdx % IMU_RING_CAPACITY) * IMU_FIELDS];
        endIdx++;
        if (timestamp > frameTs) break;
    }
    const available = endIdx - imuRingReadIdx;
    const count = Math.min(available, maxIMUReadings);
    const startIdx = endIdx - count;
    if (startIdx > imuRingReadIdx) recordIntervalLoss('capacity', startIdx - imuRingReadIdx,
        imuRing[(imuRingReadIdx % IMU_RING_CAPACITY) * IMU_FIELDS], imuRing[((startIdx - 1) % IMU_RING_CAPACITY) * IMU_FIELDS]);
    imuDrops.capacity += startIdx - imuRingReadIdx;
    for (let i = 0; i < count; i++) {
        const slot = ((startIdx + i) % IMU_RING_CAPACITY) * IMU_FIELDS;
        wasm.HEAPF64.set(imuRing.subarray(slot, slot + IMU_FIELDS), memIMU.ptr / 8 + i * IMU_FIELDS);
    }
    imuRingReadIdx = endIdx;
    return count;
}
class WorkerSharedMemory {
    constructor(wasmModule, heapArray, count) {
        this.wasm = wasmModule;
        this.count = count;
        this.byteSize = count * heapArray.BYTES_PER_ELEMENT;
        // Classify before malloc: growth replaces module heap views during allocation.
        this._heapType = heapArray === wasmModule.HEAPF64 ? 'f64' : 'u8';
        this.ptr = wasmModule._malloc(this.byteSize);
        if (!this.ptr) throw new Error('WASM buffer allocation failed');
    }

    write(typedArray) {
        if (typedArray.length > this.count) throw new Error('WASM write exceeds allocated buffer');
        // Read current heap views on every access; Emscripten may have grown memory.
        if (this._heapType === 'u8') {
            this.wasm.HEAPU8.set(typedArray, this.ptr);
        } else {
            this.wasm.HEAPF64.set(typedArray, this.ptr / 8);
        }
    }

    read(count) {
        if (!Number.isInteger(count) || count < 0 || count > this.count) throw new Error('WASM read exceeds allocated buffer');
        if (this._heapType === 'f64') {
            return new Float64Array(
                this.wasm.HEAPF64.buffer.slice(this.ptr, this.ptr + count * 8)
            );
        }
        return new Uint8Array(
            this.wasm.HEAPU8.buffer.slice(this.ptr, this.ptr + count)
        );
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

function engineValue(name, fallback) {
    try { return engine && typeof engine[name] === 'function' ? engine[name]() : fallback; }
    catch (_) { return fallback; }
}

function engineState() {
    const epoch = Number(engineValue('getEpoch', NaN));
    const frameTimestamp = engineValue('getFrameTimestamp', null);
    const benchmarkSolverProfile = engineValue('getBenchmarkSolverProfile', null);
    return {
        engineEpoch: Number.isSafeInteger(epoch) && epoch >= 0 ? epoch : null,
        engineFrameTimestamp: Number.isFinite(frameTimestamp) && frameTimestamp >= 0 ? frameTimestamp : null,
        poseFrame: 'camera',
        initialized: engineValue('isInitialized', false),
        featureCount: engineValue('getFeaturePointCount', 0),
        statusCode: engineValue('getStatusCode', 0),
        reason: engineValue('getLastReason', 'engine_status_unavailable'),
        solverIterations: engineValue('getLastSolverIterations', null),
        solverTermination: engineValue('getLastSolverTermination', null),
        benchmarkSolverProfile,
        solverProfile: benchmarkSolverProfile === true ? 'max10_zero_positive_tolerances' : benchmarkSolverProfile === false ? 'default' : 'unavailable',
        executionSeed: engineValue('getExecutionSeed', null),
        cvThreads: engineValue('getCVThreadCount', null),
        imuEndpointTimestamp: engineValue('getIMUEndpointTimestamp', null),
    };
}

function emptyResult(timestamp, reason, imuCount = 0) {
    return { ...engineState(), pose: null, poseValid: false, poseFresh: false, poseTimestamp: null,
        inputTimestamp: Number.isFinite(timestamp) ? timestamp : null, timestamp: Number.isFinite(timestamp) ? timestamp : null,
        reason, imuCount, mapPoints: null, mapPointCount: 0, imuDrops: { ...imuDrops }, engineProcessingMs: 0 };
}

function rejectRequest(request, responseType, error) {
    const fields = { success: false, error };
    if (responseType === 'result') fields.data = emptyResult(request.data?.timestamp, error);
    reply(request, responseType, fields);
}

function processFrame(gray, timestamp) {
    if (!configured || !engine) return emptyResult(timestamp, 'not_configured');
    if (!Number.isFinite(timestamp) || gray.length !== imageWidth * imageHeight) {
        return emptyResult(timestamp, 'invalid_frame');
    }
    memImage.write(gray);
    const imuCount = drainIMUToWasm(timestamp);
    let loss = null;
    if (intervalLoss) {
        const endpointAvailable = typeof engine.getIMUEndpointTimestamp === 'function';
        const endpoint = engineValue('getIMUEndpointTimestamp', null);
        const knownEndpoint = Number.isFinite(endpoint) && endpoint >= 0;
        loss = { ...intervalLoss, reasons: { ...intervalLoss.reasons }, preexistingEngineEndpointTimestamp: knownEndpoint ? endpoint : null,
            engineEndpointDisconnected: endpointAvailable ? knownEndpoint : null,
            endpointStatus: knownEndpoint ? 'known' : endpointAvailable ? 'none' : 'unavailable', engineEpochBeforeReset: engineState().engineEpoch };
        engine.reset();
        engineEpoch = engineState().engineEpoch;
        loss.engineEpochAfterReset = engineEpoch;
        intervalLoss = null;
        imuDrops.lossResets++;
    }
    const started = performance.now();
    // Even an empty batch may be covered by Engine's pending future bracket.
    const hasPose = engine.processFrame(memImage.ptr, imageWidth, imageHeight, memIMU.ptr, imuCount, timestamp, memPose.ptr);
    const engineProcessingMs = performance.now() - started;
    const state = engineState();
    if (state.engineEpoch !== engineEpoch) clearIMURing();
    engineEpoch = state.engineEpoch;
    const poseTimestamp = engineValue('getPoseTimestamp', null);
    const poseFresh = hasPose && engineValue('getPoseFresh', false) === true;
    const poseValid = poseFresh && engineValue('getPoseValid', false) === true && Number.isFinite(poseTimestamp) && poseTimestamp >= 0;
    const result = { ...emptyResult(timestamp, state.reason, imuCount), ...state, engineProcessingMs, imuIntervalLoss: loss };
    if (loss) {
        // Recent/future samples seed the new lifecycle; this frame cannot connect to its old endpoint.
        result.engineReason = state.reason;
        result.reason = 'imu_transport_loss';
        result.initialized = false;
        result.statusCode = 1;
        return result;
    }
    if (poseValid) {
        const pose = memPose.read(16);
        if (pose.every(Number.isFinite)) {
            result.pose = pose;
            result.poseTimestamp = poseTimestamp;
            result.poseFresh = result.poseValid = true;
        } else result.reason = 'nonfinite_pose';
    }
    if (result.poseValid) {
        const count = engine.getMapPoints(memMapPoints.ptr, maxMapPoints);
        if (!Number.isInteger(count) || count < 0 || count > maxMapPoints) throw new Error('Invalid map point count');
        if (count > 0) {
            const points = memMapPoints.read(count * 3);
            if (points.every(Number.isFinite)) {
                result.mapPoints = points;
                result.mapPointCount = count;
            } else result.reason = 'nonfinite_map';
        }
    }
    return result;
}

function reply(request, type, fields = {}) {
    self.postMessage({ type, requestId: request.requestId, clientEpoch: request.clientEpoch,
        sequence: request.sequence, ...fields });
}

function nowMs() { return performance.timeOrigin + performance.now(); }

const parameterCalls = {
    setMobileParams: ['solver_time', 'num_iterations', 'max_features'],
    setFThreshold: ['f_threshold'],
    setTrackingParams: ['lk_window', 'lk_pyramid', 'min_dist', 'f_edge_factor'],
    setPnPParams: ['enable_pnp', 'freq'],
};

// init is async; its captured epoch must still be current before installing the engine.
self.onmessage = async function(event) {
    const request = event.data || {};
    const { type, data } = request;
    const receivedAtMs = nowMs();
    const responseType = type === 'frame' ? 'result' : type;
    const lifecycle = ['init', 'configure', 'reset', 'dispose'].includes(type);
    if (!Number.isInteger(request.requestId) || request.requestId < 1 || !Number.isInteger(request.clientEpoch) || request.clientEpoch < 0 ||
        !Number.isInteger(request.sequence) || request.sequence < 1) {
        rejectRequest(request, responseType, 'invalid_request');
        return;
    }
    if (request.clientEpoch < clientEpoch || (request.clientEpoch > clientEpoch && !lifecycle)) {
        rejectRequest(request, responseType, 'stale_epoch');
        return;
    }
    if (request.clientEpoch > clientEpoch) { clientEpoch = request.clientEpoch; lastSequence = 0; }
    if (request.sequence <= lastSequence) {
        rejectRequest(request, responseType, 'stale_sequence');
        return;
    }
    lastSequence = request.sequence;
    if (lifecycle) cancelBracketWait();
    if (type === 'frame' && processing) {
        rejectRequest(request, 'result', 'worker_busy');
        return;
    }
    const started = performance.now();
    let processingStarted = started;
    let imuWaitMs = 0;
    try {
        if (type === 'init') {
            activeRequest = request;
            const module = await import(data.wasmPath);
            if (typeof module.default !== 'function') throw new Error('WASM artifact must export an ES module factory');
            const noise = /detect_structure|block_sparse_matrix|schur_eliminator|callbacks\.cc|trust_region_minimizer|Schur complement|Dynamic .* block size/;
            const loadedWasm = await module.default({
                print: text => { if (!noise.test(text)) reply(request, 'wasm_log', { data: { level: 'info', msg: text } }); },
                printErr: text => { if (!noise.test(text)) reply(request, 'wasm_log', { data: { level: 'warn', msg: text } }); },
            });
            if (request.clientEpoch !== clientEpoch) {
                reply(request, 'init', { success: false, error: 'stale_epoch' });
                return;
            }
            disposeBuffers();
            if (engine) engine.delete();
            wasm = loadedWasm;
            engine = new wasm.VIOEngine();
            configured = false;
            clearIMURing(true);
            reply(request, 'init', { success: true });
        } else if (type === 'configure') {
            configured = false;
            if (!engine || !Number.isInteger(data?.width) || !Number.isInteger(data?.height) ||
                data.width < 1 || data.height < 1 || data.width * data.height > 16777216) throw new Error('Invalid image dimensions');
            const r = data.r_ic || [1, 0, 0, 0, 1, 0, 0, 0, 1];
            const t = data.t_ic || [0, 0, 0];
            if (r.length !== 9 || t.length !== 3 || !Array.from(r).every(Number.isFinite) || !Array.from(t).every(Number.isFinite)) throw new Error('Invalid extrinsics');
            imageWidth = data.width;
            imageHeight = data.height;
            allocateBuffers();
            clearIMURing();
            memExtrinsicR.write(new Float64Array(r));
            memExtrinsicT.write(new Float64Array(t));
            configured = engine.configure(imageWidth, imageHeight, data.fx, data.fy, data.cx, data.cy,
                data.modelType ?? 2, data.k2 ?? 0, data.k3 ?? 0, data.k4 ?? 0, data.k5 ?? 0,
                memExtrinsicR.ptr, memExtrinsicT.ptr, data.acc_n ?? 0.08, data.acc_w ?? 0.0004,
                data.gyr_n ?? 0.01, data.gyr_w ?? 0.0001, data.g_norm ?? 9.81) === true;
            if (configured) {
                const requested = data.benchmark_zero_positive_tolerances === true;
                const profileAPI = typeof engine.setBenchmarkSolverProfile === 'function' && typeof engine.getBenchmarkSolverProfile === 'function';
                if (requested && !profileAPI) throw new Error('Benchmark solver profile API unavailable');
                if (typeof engine.setBenchmarkSolverProfile === 'function') engine.setBenchmarkSolverProfile(requested);
                if (profileAPI && engine.getBenchmarkSolverProfile() !== requested) throw new Error('Benchmark solver profile not applied');
            }
            if (configured && typeof engine.setExecutionParams === 'function') engine.setExecutionParams(data.execution_seed ?? 0, data.cv_threads ?? 1);
            engineEpoch = engineState().engineEpoch;
            reply(request, type, { success: configured, benchmarkSolverProfile: engineState().benchmarkSolverProfile, imuDrops: { ...imuDrops } });
        } else if (Object.hasOwn(parameterCalls, type)) {
            if (!engine) throw new Error('Engine is not loaded');
            engine[type](...parameterCalls[type].map(name => data?.[name]));
            reply(request, type, { success: true });
        } else if (type === 'imu') {
            if (!configured) throw new Error('Engine is not configured');
            if (!(data?.imuData instanceof ArrayBuffer) || data.imuData.byteLength % 8 !== 0) {
                const count = Number.isInteger(data?.count) && data.count > 0 && data.count <= 4096 ? data.count : 0;
                imuDrops.received += count;
                imuDrops.invalid += count;
                imuDrops.invalidBatches++;
                throw new Error('Invalid IMU buffer');
            }
            const success = appendIMU(new Float64Array(data.imuData), data.count);
            reply(request, type, { success, error: success ? undefined : 'invalid_imu_batch_or_sample', imuDrops: { ...imuDrops } });
        } else if (type === 'frame') {
            processing = true;
            activeRequest = request;
            if (!(data?.gray instanceof ArrayBuffer)) throw new Error('Invalid image buffer');
            const gray = new Uint8Array(data.gray);
            let bracket = true;
            if (configured && engine && Number.isFinite(data.timestamp) && gray.length === imageWidth * imageHeight) {
                const wait = await waitForBracket(request, data.timestamp);
                imuWaitMs = wait.imuWaitMs;
                if (wait.cancelled || request.clientEpoch !== clientEpoch || activeRequest !== request) {
                    const result = emptyResult(data.timestamp, 'stale_epoch');
                    result.imuWaitMs = imuWaitMs;
                    result.workerQueueMs = Number.isFinite(request.sentAtMs) ? Math.max(0, receivedAtMs - request.sentAtMs) : null;
                    result.workerProcessingMs = 0;
                    reply(request, 'result', { success: false, error: 'stale_epoch', data: result });
                    return;
                }
                bracket = wait.bracket;
            }
            processingStarted = performance.now();
            const result = bracket ? processFrame(gray, data.timestamp) : emptyResult(data.timestamp, 'missing_bracket');
            if (!bracket) result.statusCode = result.initialized ? 3 : 1;
            result.imuWaitMs = imuWaitMs;
            result.workerQueueMs = Number.isFinite(request.sentAtMs) ? Math.max(0, receivedAtMs - request.sentAtMs) : null;
            result.workerProcessingMs = performance.now() - processingStarted;
            reply(request, 'result', { success: true, data: result });
        } else if (type === 'reset') {
            if (engine) engine.reset();
            clearIMURing();
            engineEpoch = engineState().engineEpoch;
            reply(request, type, { success: true, imuDrops: { ...imuDrops } });
        } else if (type === 'dispose') {
            configured = false;
            disposeBuffers();
            if (engine) engine.delete();
            engine = wasm = null;
            clearIMURing();
            engineEpoch = null;
            reply(request, type, { success: true });
        } else throw new Error('Unknown worker request');
    } catch (error) {
        if (type === 'configure') configured = false;
        if (type === 'frame') {
            try { if (engine) engine.reset(); } catch (_) { configured = false; }
            clearIMURing();
            engineEpoch = engineState().engineEpoch;
            const result = emptyResult(data?.timestamp, 'worker_exception');
            result.error = error.message;
            result.workerQueueMs = Number.isFinite(request.sentAtMs) ? Math.max(0, receivedAtMs - request.sentAtMs) : null;
            result.imuWaitMs = imuWaitMs;
            result.workerProcessingMs = performance.now() - processingStarted;
            reply(request, 'result', { success: false, error: error.message, data: result });
        } else reply(request, responseType, { success: false, error: error.message, imuDrops: type === 'imu' ? { ...imuDrops } : undefined });
    } finally {
        if (activeRequest === request) {
            if (type === 'frame') processing = false;
            activeRequest = null;
        }
    }
};

self.onunhandledrejection = function(event) {
    self.postMessage({ type: 'runtime_error', clientEpoch: activeRequest?.clientEpoch ?? clientEpoch, requestId: activeRequest?.requestId ?? null,
        sequence: activeRequest?.sequence ?? null, error: event.reason?.message || String(event.reason) });
    event.preventDefault();
    cancelBracketWait();
};

/** Async API for the single VIO Worker. Frames may be dropped while IMU remains deliverable. */
function validPipelineQuality(pipeline) {
    const q = pipeline?.quality, s = pipeline?.solver;
    const count = value => Number.isSafeInteger(value) && value >= 0;
    const nullable = value => value === null || Number.isFinite(value) && value >= 0;
    if (pipeline?.schema !== 'mobile-slam-pipeline-v2' || !q ||
        !['metric', 'projective_limited', 'none'].includes(q.mode) ||
        !['visual_inertial_metric', 'imu_model_only', 'none'].includes(q.translationSupport) ||
        !['rotationVisualSupported', 'metricTranslationSupported', 'fallbackUsed', 'recoveryMode', 'uncertaintyAvailable'].every(key => typeof q[key] === 'boolean') ||
        !['currentMetricFactors', 'currentProjectiveFactors', 'provisionalObservationsRetired'].every(key => count(q[key])) ||
        !['limitedDurationS', 'uncertaintyAnchorTimestamp', 'position3SigmaM', 'rotation3SigmaRad', 'rotationConsistencyScore'].every(key => nullable(q[key])) ||
        !(q.visualRotationRank === null || count(q.visualRotationRank) && q.visualRotationRank <= 3) ||
        q.rotationConsistencyScoreThreshold !== 16.2663 || q.consistencyScoreIsProbabilistic !== false ||
        q.relativeCovarianceOrdering !== 'anchor_body_position_xyz_m,right_relative_rotation_xyz_rad' ||
        !['imu_propagation_joint_15d', 'unavailable'].includes(q.uncertaintyModel) ||
        !['last_metric_raw_graph', 'unavailable'].includes(q.uncertaintySource) || typeof q.reason !== 'string') return false;
    if (s === null) {
        if (q.mode === 'projective_limited' || q.currentMetricFactors !== 0 || q.currentProjectiveFactors !== 0) return false;
    } else if (!s || !['eligibleFeatures', 'structuralSlots', 'currentMetricFactors', 'currentProjectiveFactors', 'provisionalSlots', 'excludedSlots'].every(key => count(s[key])) ||
        s.structuralSlots !== s.eligibleFeatures || s.structuralSlots > 1000 ||
        q.currentMetricFactors !== s.currentMetricFactors || q.currentProjectiveFactors !== s.currentProjectiveFactors) return false;
    const covariance = q.relativeCovariance6x6;
    return covariance === null || Array.isArray(covariance) && covariance.length === 36 && covariance.every(Number.isFinite) &&
        [0, 7, 14, 21, 28, 35].every(index => covariance[index] >= 0);
}

function validLimitedQuality(quality, timestamp) {
    return quality?.mode === 'projective_limited' && quality.rotationVisualSupported === true &&
        quality.metricTranslationSupported === false && quality.translationSupport === 'imu_model_only' &&
        quality.fallbackUsed === true && quality.recoveryMode === true && quality.currentProjectiveFactors >= 2 &&
        quality.visualRotationRank === 3 && quality.uncertaintyAvailable === true &&
        quality.uncertaintyModel === 'imu_propagation_joint_15d' && quality.uncertaintySource === 'last_metric_raw_graph' &&
        Array.isArray(quality.relativeCovariance6x6) && Number.isFinite(quality.uncertaintyAnchorTimestamp) &&
        quality.uncertaintyAnchorTimestamp <= timestamp &&
        Number.isFinite(quality.limitedDurationS) && quality.limitedDurationS <= 2 &&
        Number.isFinite(quality.position3SigmaM) && quality.position3SigmaM <= .25 + 1e-12 &&
        Number.isFinite(quality.rotation3SigmaRad) && quality.rotation3SigmaRad <= 5 * Math.PI / 180 + 1e-12 &&
        Number.isFinite(quality.rotationConsistencyScore) && quality.rotationConsistencyScore <= quality.rotationConsistencyScoreThreshold;
}

function trackingState(statusCode) {
    if (!Number.isInteger(statusCode)) return 'unknown';
    return ({ 0: 'not_configured', 1: 'initializing', 2: 'tracking', 3: 'lost', 4: 'cooldown', 5: 'tracking_limited' })[statusCode] || 'unknown';
}
export class VIOWrapper {
    constructor({ requestTimeoutMs = 15000, frameTimeoutMs = 30000 } = {}) {
        this.worker = null;
        this.configured = false;
        this.workerBusy = false;
        this.onFrameComplete = null;
        this.onWasmLog = null;
        this._latestResult = null;
        this._latestMapPoints = null;
        this._engineEpoch = null;
        this._clientEpoch = 0;
        this._nextRequestId = 1;
        this._sequence = 0;
        this._pending = new Map();
        this._freeWaiters = new Set();
        this._activeFrame = null;
        this._requestTimeoutMs = this._boundedTimeout(requestTimeoutMs, 15000);
        this._frameTimeoutMs = this._boundedTimeout(frameTimeoutMs, 30000);
        this._metrics = {
            framesSubmitted: 0, framesCompleted: 0, framesDropped: 0, framesTimedOut: 0, framesCancelled: 0,
            frameDropReasons: { busy: 0, notConfigured: 0, invalid: 0, postMessage: 0 },
            imuReadingsSubmitted: 0, imuReadingsRejected: 0, staleReplies: 0,
            workerErrors: 0, imuDrops: null, lastError: null,
        };
    }

    _boundedTimeout(value, fallback) {
        return Number.isFinite(value) && value > 0 ? Math.min(value, 60000) : fallback;
    }

    _nowMs() {
        return performance.timeOrigin + performance.now();
    }

    _message(type, data) {
        return { type, data, requestId: this._nextRequestId++, clientEpoch: this._clientEpoch,
            sequence: ++this._sequence, sentAtMs: this._nowMs() };
    }

    /** One pending table prevents concurrent calls of the same RPC from overwriting each other. */
    _request(type, data) {
        if (!this.worker) return Promise.reject(new Error('Worker is not loaded'));
        const message = this._message(type, data);
        return new Promise((resolve, reject) => {
            const timer = setTimeout(() => {
                if (!this._pending.delete(message.requestId)) return;
                const error = new Error(`Worker ${type} timeout`);
                reject(error);
                // Unknown configure/reset completion cannot safely admit another frame.
                if (type === 'init' || type === 'configure' || type === 'reset') this._workerFailure(error);
            }, this._requestTimeoutMs);
            this._pending.set(message.requestId, { ...message, resolve, reject, timer });
            try {
                this.worker.postMessage(message);
            } catch (error) {
                clearTimeout(timer);
                this._pending.delete(message.requestId);
                reject(error);
            }
        });
    }

    _invalidate(error) {
        if (this._activeFrame && !this._activeFrame.timedOut) this._metrics.framesCancelled++;
        this._clientEpoch++;
        this._sequence = 0;
        this._latestResult = null;
        this._latestMapPoints = null;
        this._engineEpoch = null;
        for (const pending of this._pending.values()) {
            clearTimeout(pending.timer);
            pending.reject(error);
        }
        this._pending.clear();
        this._finishFrame(error);
    }

    async load(wasmPath = '/vio_engine.js') {
        this._invalidate(new Error('Worker replaced'));
        this.configured = false;
        if (this.worker) this.worker.terminate();
        const worker = new Worker('/js/vio-worker.js', { type: 'module' });
        this.worker = worker;
        worker.onmessage = event => {
            if (this.worker === worker) this._handleWorkerMessage(event.data);
        };
        worker.onerror = error => {
            if (this.worker === worker) this._workerFailure(new Error(error.message || 'Worker runtime error'));
        };
        worker.onmessageerror = () => {
            if (this.worker === worker) this._workerFailure(new Error('Worker message deserialization failed'));
        };
        const success = await this._request('init', { wasmPath });
        if (!success) throw new Error('Worker init failed');
    }

    async configure(params) {
        this._invalidate(new Error('Worker reconfigured'));
        this.configured = false;
        return this._request('configure', params);
    }

    async setMobileParams(solverTime, numIterations, maxFeatures) {
        return this._request('setMobileParams', {
            solver_time: solverTime, num_iterations: numIterations, max_features: maxFeatures,
        });
    }

    async setFThreshold(fThreshold) {
        return this._request('setFThreshold', { f_threshold: fThreshold });
    }

    /** Read the configured engine's effective parameters without advancing its lifecycle. */
    async getEffectiveConfig() {
        if (!this.configured) throw new Error('Engine is not configured');
        return this._request('getEffectiveConfig');
    }

    async setTrackingParams(lkWindow, lkPyramid, minDist, fEdgeFactor) {
        return this._request('setTrackingParams', {
            lk_window: lkWindow, lk_pyramid: lkPyramid, min_dist: minDist, f_edge_factor: fEdgeFactor,
        });
    }

    async setPnPParams(enablePnP, freq = 3) {
        return this._request('setPnPParams', { enable_pnp: enablePnP, freq });
    }

    /** Flat [timestamp seconds, acceleration m/s², angular velocity rad/s] × count. */
    sendIMU(imuData, count) {
        if (!this.configured || !this.worker) return false;
        if (!(imuData instanceof Float64Array) || !Number.isInteger(count) || count < 1 ||
            count > 4096 || count * 7 > imuData.length) {
            this._metrics.imuReadingsRejected += Number.isInteger(count) && count > 0 ? count : 0;
            this._metrics.lastError = 'invalid_imu_batch';
            return false;
        }
        for (let i = 0; i < count * 7; i++) {
            if (!Number.isFinite(imuData[i]) || (i % 7 === 0 && i >= 7 && imuData[i] <= imuData[i - 7])) {
                this._metrics.imuReadingsRejected += count;
                this._metrics.lastError = 'invalid_imu_sample';
                return false;
            }
        }
        // Copy the declared view: transferring its backing buffer would detach unrelated readings.
        const buffer = imuData.buffer.slice(imuData.byteOffset, imuData.byteOffset + count * 7 * 8);
        try {
            this.worker.postMessage(this._message('imu', { imuData: buffer, count }), [buffer]);
            this._metrics.imuReadingsSubmitted += count;
            return true;
        } catch (error) {
            this._metrics.imuReadingsRejected += count;
            this._metrics.lastError = error.message;
            return false;
        }
    }

    /** Submit exact grayscale bytes. Explicitly owned full views may transfer their storage. */
    sendFrame(grayImage, timestamp = 0, { transferOwnership = false, sourceTiming = null } = {}) {
        if (!this.configured || !this.worker || this.workerBusy ||
            !(grayImage instanceof Uint8Array) || !Number.isFinite(timestamp)) {
            this._metrics.framesDropped++;
            const reason = !this.configured || !this.worker ? 'notConfigured' : this.workerBusy ? 'busy' : 'invalid';
            this._metrics.frameDropReasons[reason]++;
            return false;
        }
        const copyStarted = performance.now();
        const canTransfer = transferOwnership === true && grayImage.buffer instanceof ArrayBuffer &&
            grayImage.byteOffset === 0 && grayImage.byteLength === grayImage.buffer.byteLength;
        const gray = canTransfer ? grayImage.buffer :
            grayImage.buffer.slice(grayImage.byteOffset, grayImage.byteOffset + grayImage.byteLength);
        const frameCopyMs = canTransfer ? 0 : performance.now() - copyStarted;
        const message = this._message('frame', { gray, timestamp });
        this.workerBusy = true;
        const timer = setTimeout(() => {
            if (this._activeFrame?.requestId !== message.requestId) return;
            this._metrics.framesTimedOut++;
            this._activeFrame.timedOut = true;
            this._metrics.lastError = 'frame_timeout';
            this._workerFailure(new Error('Worker frame timeout'));
        }, this._frameTimeoutMs);
        this._activeFrame = { ...message, timer, frameCopyMs, sourceTiming: sourceTiming ? { ...sourceTiming } : null };
        try {
            this.worker.postMessage(message, [gray]);
            this._metrics.framesSubmitted++;
            return true;
        } catch (error) {
            this._metrics.framesDropped++;
            this._metrics.frameDropReasons.postMessage++;
            this._metrics.lastError = error.message;
            this._latestResult = this._latestMapPoints = null;
            this._finishFrame(error);
            return false;
        }
    }

    getLatestResult() { return this._latestResult; }
    getMapPoints() { return this._latestMapPoints || { points: null, count: 0 }; }
    isInitialized() { return this._latestResult?.initialized || false; }
    getMetrics() {
        return { ...this._metrics, imuDrops: this._metrics.imuDrops ? { ...this._metrics.imuDrops } : null,
            frameDropReasons: { ...this._metrics.frameDropReasons },
            workerBusy: this.workerBusy, clientEpoch: this._clientEpoch };
    }

    reset() {
        const wasConfigured = this.configured;
        this._invalidate(new Error('Worker reset'));
        const epoch = this._clientEpoch;
        this.configured = false;
        if (this.worker) this._request('reset').then(success => {
            if (this._clientEpoch === epoch) this.configured = success && wasConfigured;
        }, error => {
            if (this._clientEpoch === epoch) this._metrics.lastError = error.message;
        });
    }

    dispose() {
        this._invalidate(new Error('Worker disposed'));
        this.configured = false;
        if (this.worker) {
            try { this.worker.postMessage(this._message('dispose')); } catch (_) { /* Already unavailable. */ }
            this.worker.terminate();
            this.worker = null;
        }
    }

    /** Each consumer has its own bounded deadline and always settles on completion or invalidation. */
    waitForFree(timeoutMs = 5000) {
        if (!this.workerBusy) return Promise.resolve();
        return new Promise((resolve, reject) => {
            const waiter = { resolve, reject, timer: null };
            waiter.timer = setTimeout(() => {
                this._freeWaiters.delete(waiter);
                reject(new Error('Worker timeout'));
            }, this._boundedTimeout(timeoutMs, 5000));
            this._freeWaiters.add(waiter);
        });
    }

    _finishFrame(error = null) {
        if (this._activeFrame) clearTimeout(this._activeFrame.timer);
        this._activeFrame = null;
        this.workerBusy = false;
        for (const waiter of this._freeWaiters) {
            clearTimeout(waiter.timer);
            if (error) waiter.reject(error); else waiter.resolve();
        }
        this._freeWaiters.clear();
    }

    _workerFailure(error) {
        this._metrics.workerErrors++;
        this._metrics.lastError = error.message;
        this._invalidate(error);
        this.configured = false;
        if (this.worker) this.worker.terminate();
        this.worker = null;
    }

    _matches(message, pending) {
        return pending && message.clientEpoch === this._clientEpoch &&
            message.clientEpoch === pending.clientEpoch && message.requestId === pending.requestId &&
            message.sequence === pending.sequence;
    }

    _handleWorkerMessage(message) {
        if (!message || message.clientEpoch !== this._clientEpoch) {
            this._metrics.staleReplies++;
            return;
        }
        if (message.type === 'wasm_log') {
            if (this.onWasmLog && message.data) this.onWasmLog(message.data.level, message.data.msg);
            return;
        }
        if (message.type === 'runtime_error') {
            this._workerFailure(new Error(message.error || 'Worker runtime rejection'));
            return;
        }
        if (message.type === 'imu') {
            if (message.imuDrops) this._metrics.imuDrops = message.imuDrops;
            if (!message.success) this._metrics.lastError = message.error;
            return;
        }
        if (message.type === 'result') {
            if (!this._matches(message, this._activeFrame)) {
                this._metrics.staleReplies++;
                return;
            }
            const roundtripMs = this._nowMs() - this._activeFrame.sentAtMs;
            const frameCopyMs = this._activeFrame.frameCopyMs;
            const sourceTiming = this._activeFrame.sourceTiming;
            const data = message.data || { inputTimestamp: this._activeFrame.data.timestamp,
                pose: null, poseValid: false, poseFresh: false, poseTimestamp: null,
                mapPoints: null, mapPointCount: 0, reason: message.error || 'empty_result' };
            this._metrics.framesCompleted++;
            const inputTimestamp = this._activeFrame.data.timestamp;
            const wrongEpoch = this._engineEpoch !== null && Number.isFinite(data.engineEpoch) && data.engineEpoch < this._engineEpoch;
            const error = message.success !== true ? new Error(message.error || data.reason || 'Worker frame failed') :
                data.inputTimestamp !== inputTimestamp ? new Error('Frame timestamp mismatch') :
                wrongEpoch ? new Error('Stale engine epoch') : null;
            if (error) this._metrics.lastError = error.message;
            if (data.engineEpoch !== this._engineEpoch) this._latestMapPoints = null;
            if (!wrongEpoch) this._engineEpoch = data.engineEpoch ?? null;
            const poseTimestamp = Number.isFinite(data.poseTimestamp) && data.poseTimestamp >= 0 ? data.poseTimestamp : null;
            const futurePose = !error && data.poseFresh === true && data.poseValid === true && poseTimestamp !== null && poseTimestamp > inputTimestamp;
            const observedPipeline = data.pipeline;
            const pipeline = !error && observedPipeline?.frameTimestamp === inputTimestamp && observedPipeline.engineEpoch === data.engineEpoch &&
                (observedPipeline.schema === 'mobile-slam-pipeline-v1' || validPipelineQuality(observedPipeline)) ? observedPipeline : null;
            const quality = pipeline?.schema === 'mobile-slam-pipeline-v2' && data.quality &&
                JSON.stringify(data.quality) === JSON.stringify(pipeline.quality) ? data.quality : null;
            const limitedCurrent = data.statusCode === 5 && data.initialized === true && poseTimestamp === inputTimestamp &&
                data.engineFrameTimestamp === inputTimestamp && validLimitedQuality(quality, inputTimestamp);
            const poseValid = !error && !futurePose && (data.statusCode === 2 || limitedCurrent) && data.poseValid === true && data.poseFresh === true && poseTimestamp !== null &&
                data.pose instanceof Float64Array && data.pose.length === 16 && data.pose.every(Number.isFinite);
            const count = data.mapPointCount;
            const mapValid = poseValid && data.mapPoints instanceof Float64Array && Number.isInteger(count) &&
                count > 0 && count <= 2000 && data.mapPoints.length === count * 3 && data.mapPoints.every(Number.isFinite);
            this._latestResult = {
                ...data, pose: poseValid ? data.pose : null, poseValid, poseFresh: poseValid,
                trackingState: error ? 'unknown' : trackingState(data.statusCode), pipeline, quality,
                poseTimestamp: poseValid ? poseTimestamp : null,
                inputTimestamp, timestamp: inputTimestamp, requestId: message.requestId,
                clientEpoch: message.clientEpoch, sequence: message.sequence, roundtripMs, frameCopyMs,
                sourceTiming: error ? null : sourceTiming,
                ...(futurePose ? { reason: 'future_pose_timestamp', engineReason: data.engineReason ?? data.reason } : {}),
                poseAgeSeconds: poseValid ? inputTimestamp - poseTimestamp : null,
                mapPoints: mapValid ? data.mapPoints : null, mapPointCount: mapValid ? count : 0,
            };
            this._latestMapPoints = mapValid
                ? { points: data.mapPoints, count } : null;
            if (data.imuDrops) this._metrics.imuDrops = data.imuDrops;
            this._finishFrame(error);
            if (!error) this.onFrameComplete?.(this._latestResult);
            return;
        }
        const pending = this._pending.get(message.requestId);
        if (!this._matches(message, pending) || message.type !== pending.type) {
            this._metrics.staleReplies++;
            return;
        }
        clearTimeout(pending.timer);
        this._pending.delete(message.requestId);
        if (message.imuDrops) this._metrics.imuDrops = message.imuDrops;
        if (message.type === 'configure') this.configured = message.success === true;
        if (message.error) {
            this._metrics.lastError = message.error;
            pending.reject(new Error(message.error));
        }
        else if (message.type === 'getEffectiveConfig') {
            const data = message.data;
            if (message.success !== true || data?.schema !== 'mobile-slam-effective-config-v1' ||
                !data.params || typeof data.params !== 'object' || Array.isArray(data.params) || !Object.keys(data.params).length ||
                !Object.values(data.params).every(Number.isFinite) || !Number.isSafeInteger(data.engineEpoch) || data.engineEpoch < 0 ||
                !Number.isFinite(data.frameTimestamp) || typeof data.opencvVersion !== 'string' || !data.opencvVersion ||
                !Number.isInteger(data.executionSeed) || !Number.isInteger(data.cvThreads) || data.cvThreads < 1) {
                this._metrics.lastError = 'Invalid effective configuration response';
                pending.reject(new Error(this._metrics.lastError));
            } else pending.resolve(data);
        } else pending.resolve(message.success === true);
    }
}

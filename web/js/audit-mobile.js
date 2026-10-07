/** Development-only observation of the unchanged live app; local bounded JSON export. */
const CHANNEL_LIMIT = 10000;
const TOTAL_LIMIT = 40000;

export function poseDelta(a, b) {
    if (!a || !b || a.length !== 16 || b.length !== 16 || ![...a, ...b].every(Number.isFinite)) return null;
    let trace = 0;
    for (const i of [0, 1, 2, 4, 5, 6, 8, 9, 10]) trace += a[i] * b[i];
    return { translationM: Math.hypot(b[3] - a[3], b[7] - a[7], b[11] - a[11]), rotationDeg: Math.acos(Math.max(-1, Math.min(1, (trace - 1) / 2))) * 180 / Math.PI };
}

export function createRecorder({ now = () => performance.now(), maxSamples = CHANNEL_LIMIT, maxDurationMs = 60000, maxTotal = TOTAL_LIMIT, onStop = () => {} } = {}) {
    if (!Number.isInteger(maxSamples) || maxSamples < 1 || maxSamples > CHANNEL_LIMIT || !Number.isFinite(maxDurationMs) || maxDurationMs <= 0 || maxDurationMs > 120000 || !Number.isInteger(maxTotal) || maxTotal < 1 || maxTotal > TOTAL_LIMIT) throw new Error('Invalid recorder limits');
    let report = null;
    let active = false;
    let count = 0;
    const copy = value => typeof structuredClone === 'function' ? structuredClone(value) : JSON.parse(JSON.stringify(value));
    const recorder = {
        start(metadata = {}) {
            if (active) throw new Error('Recording already active');
            active = true;
            count = 0;
            report = { schema: 'mobile-slam-observation-v1', createdAt: new Date().toISOString(), startedAtMs: now(), stoppedAtMs: null, stopReason: null, metadata: copy(metadata), limits: { maxDurationMs, maxSamplesPerChannel: maxSamples, maxTotal, units: { imuAcceleration: 'm/s^2', imuGyro: 'rad/s', rawDeviceMotionGyro: 'deg/s', translation: 'm', rotation: 'deg', performanceClock: 'ms', submittedFrameTimestamp: 's' }, scope: 'Observed existing live app; no independent accuracy/phone validation claim. Pose-age proxy is not physical camera-to-pose latency. Raw images, video, device IDs, track labels and credentials omitted; nonfinite JSON numbers become null with finite flags.' }, samples: {}, dropped: {}, totalSamples: 0 };
        },
        push(channel, sample) {
            if (!active) return false;
            if (now() - report.startedAtMs >= maxDurationMs) { recorder.stop('duration_limit'); return false; }
            if (count >= maxTotal) { recorder.stop('total_sample_limit'); return false; }
            const rows = report.samples[channel] ||= [];
            const channelLimit = channel.endsWith('logs') ? Math.min(maxSamples, 1000) : maxSamples;
            if (rows.length >= channelLimit) { report.dropped[channel] = (report.dropped[channel] || 0) + 1; return false; }
            rows.push({ atMs: now(), ...copy(sample) });
            report.totalSamples = ++count;
            return true;
        },
        stop(reason = 'manual') {
            if (!active) return;
            active = false;
            report.stoppedAtMs = now();
            report.stopReason = reason;
            onStop(reason);
        },
        get active() { return active; },
        summary() { return summarize(report); },
        export() { return report ? copy(report) : null; },
    };
    return recorder;
}

const percentile = (values, p) => values.length ? [...values].sort((a, b) => a - b)[Math.ceil(values.length * p) - 1] : null;

export function summarize(report) {
    if (!report) return null;
    const frames = report.samples.frames || [];
    const results = report.samples.results || [];
    const imu = report.samples.device_imu || [];
    const intervals = imu.slice(1).map((row, i) => (row.timestampS - imu[i].timestampS) * 1000);
    const positiveIntervals = intervals.filter(value => value > 0 && Number.isFinite(value));
    const poses = results.filter(row => row.pose);
    return { frameAttempts: frames.length, framesAccepted: frames.filter(row => row.accepted).length, framesDropped: frames.filter(row => !row.accepted).length, completed: results.length, finitePoseOutputs: poses.filter(row => row.finitePose).length, missingOutputTimestamp: results.filter(row => row.engineTimestampS === null).length, explicitResetCalls: (report.samples.resets || []).length, staleResultsIgnored: (report.samples.stale_results || []).length, engineEpochChanges: results.filter(row => row.engineEpochChanged).length, trackingInterruptions: results.filter(row => row.trackingInterrupted).length, poseJumpMarkers: poses.filter(row => row.jumpMarker).length, roundTripMs: { p50: percentile(results.map(row => row.roundTripMs).filter(Number.isFinite), 0.5), p95: percentile(results.map(row => row.roundTripMs).filter(Number.isFinite), 0.95), p99: percentile(results.map(row => row.roundTripMs).filter(Number.isFinite), 0.99) }, imu: { samples: imu.length, intervalP50Ms: percentile(positiveIntervals, 0.5), intervalP95Ms: percentile(positiveIntervals, 0.95), gapOver50Ms: intervals.filter(value => value > 50).length, nonMonotonicIntervals: intervals.filter(value => value <= 0).length, rateHz: imu.length > 1 && imu.at(-1).timestampS > imu[0].timestampS ? (imu.length - 1) / (imu.at(-1).timestampS - imu[0].timestampS) : null } };
}

const trackKeys = ['width', 'height', 'aspectRatio', 'frameRate', 'facingMode', 'resizeMode', 'zoom', 'focusMode', 'focusDistance', 'fieldOfView', 'exposureMode', 'whiteBalanceMode'];
const safeTrack = object => Object.fromEntries(trackKeys.filter(key => object && key in object).map(key => [key, object[key]]));

export async function installInRealm(win, recorder) {
    const appScript = win.document.querySelector('script[src*="app.js"]');
    if (!appScript) throw new Error('Existing app module was not found');
    const appUrl = new URL(appScript.src, win.location.href);
    const source = await (await win.fetch(appUrl)).text();
    const moduleUrl = file => {
        const match = [...source.matchAll(/from\s+['"]([^'"]+)['"]/g)].find(entry => entry[1].split('?')[0].endsWith('/' + file));
        if (!match) throw new Error(`App import missing: ${file}`);
        return new URL(match[1], appUrl).href;
    };
    const [{ VIOWrapper }, { IMU }, { Camera }] = await Promise.all([import(moduleUrl('vio-wrapper.js')), import(moduleUrl('imu.js')), import(moduleUrl('camera.js'))]);
    const restore = [];
    const cameras = new Set();
    const imus = new Set();
    const wrappers = new Set();
    const stopResources = () => { const errors = []; for (const object of [...cameras, ...imus, ...wrappers]) { try { if (wrappers.has(object)) object.dispose(); else object.stop(); } catch (error) { errors.push(error.message); } } return errors; };
    const detach = () => { for (const undo of restore.reverse()) { try { undo(); } catch {} } restore.length = 0; };
    try {
    const state = new WeakMap();
    const wrapperState = object => { if (!state.has(object)) state.set(object, { generation: 0, sequence: 0, pending: [], previousPose: null, previousAnyPose: null, previousStatus: null, previousFrameTimestamp: null, previousEngineEpoch: null }); return state.get(object); };
    const hook = (prototype, name, wrap) => { const original = prototype[name]; prototype[name] = wrap(original); restore.push(() => { prototype[name] = original; }); };
    const clock = () => win.performance.now();
    const overhead = { pushCalls: 0, pushTotalMs: 0, maxPushMs: 0, limits: 'Push/copy overhead only; wrapper/listener/summary/GC overhead not fully measured. Compare with normal app on the same physical device.' };
    const record = (channel, value) => { const started = clock(); try { return recorder.push(channel, value); } catch { return false; } finally { const elapsed = clock() - started; overhead.pushCalls++; overhead.pushTotalMs += elapsed; overhead.maxPushMs = Math.max(overhead.maxPushMs, elapsed); } };
    let blockedLogWrites = 0;
    const beacon = win.navigator.sendBeacon;
    win.navigator.sendBeacon = () => { blockedLogWrites++; return true; };
    restore.push(() => { win.navigator.sendBeacon = beacon; });
    if (typeof win.DeviceMotionEvent?.requestPermission === 'function') hook(win.DeviceMotionEvent, 'requestPermission', original => function (...args) {
        record('permission_native', { phase: 'request', requestedAtMs: clock(), userActivationActive: win.navigator.userActivation?.isActive ?? null, userActivationHasBeenActive: win.navigator.userActivation?.hasBeenActive ?? null });
        let result;
        try { result = original.apply(this, args); }
        catch (error) { record('permission_native', { phase: 'error', resolvedAtMs: clock(), name: error.name, message: error.message, synchronous: true }); throw error; }
        result.then(permission => record('permission_native', { phase: 'result', resolvedAtMs: clock(), permission }), error => record('permission_native', { phase: 'error', resolvedAtMs: clock(), name: error.name, message: error.message }));
        return result;
    });
    const motion = event => record('raw_motion', { eventTimestampMs: event.timeStamp, arrivalMs: clock(), intervalMs: event.interval, accelerationIncludingGravity: event.accelerationIncludingGravity && { x: event.accelerationIncludingGravity.x, y: event.accelerationIncludingGravity.y, z: event.accelerationIncludingGravity.z }, linearAcceleration: event.acceleration && { x: event.acceleration.x, y: event.acceleration.y, z: event.acceleration.z }, rotationDegS: event.rotationRate && { alpha: event.rotationRate.alpha, beta: event.rotationRate.beta, gamma: event.rotationRate.gamma } });
    const orientation = event => record('raw_orientation', { eventTimestampMs: event.timeStamp, arrivalMs: clock(), alpha: event.alpha, beta: event.beta, gamma: event.gamma, absolute: event.absolute });
    win.addEventListener('devicemotion', motion, true);
    win.addEventListener('deviceorientation', orientation, true);
    restore.push(() => win.removeEventListener('devicemotion', motion, true), () => win.removeEventListener('deviceorientation', orientation, true));
    for (const name of ['visibilitychange', 'blur', 'focus', 'orientationchange', 'error']) {
        const target = name === 'visibilitychange' ? win.document : win;
        const listener = event => record('lifecycle', { event: name, visibility: win.document.visibilityState, orientation: win.screen.orientation?.type || null, angle: win.screen.orientation?.angle ?? win.orientation ?? null, error: event.message || null });
        target.addEventListener(name, listener);
        restore.push(() => target.removeEventListener(name, listener));
    }
    hook(IMU.prototype, '_pushSample', original => function (timestamp, ax, ay, az, gx, gy, gz) {
        imus.add(this);
        const sensorInfo = { sourceTiming: this.getDiagnostics?.().lastSample ?? null, api: this.getSensorType(), ios: this._isIOS, iosAccSign: this._iosAccSign, accelSensorTimestampMs: this._accel?.timestamp ?? null, gyroSensorTimestampMs: this._gyro?.timestamp ?? null, callbackArrivalMs: clock() };
        record('device_imu_raw', { timestampS: timestamp, acceleration: [ax, ay, az], gyroRadS: [gx, gy, gz], gyroBias: { ...this._gyroBias }, ...sensorInfo });
        const emitted = this.getDiagnostics?.().emitted;
        const value = original.call(this, timestamp, ax, ay, az, gx, gy, gz);
        if (emitted === undefined || this.getDiagnostics?.().emitted > emitted) record('device_imu', { timestampS: timestamp, acceleration: [this.latest.acc_x, this.latest.acc_y, this.latest.acc_z], gyroRadS: [this.latest.gyro_x, this.latest.gyro_y, this.latest.gyro_z], ...sensorInfo });
        return value;
    });
    hook(IMU.prototype, 'start', original => function (...args) { imus.add(this); const value = original.apply(this, args); record('sensor_config', { requestedHz: args[0] ?? 60, selectedApi: this.getSensorType(), diagnostics: this.getDiagnostics?.() ?? null, ios: this._isIOS, iosAccSign: this._iosAccSign }); return value; });
    hook(IMU.prototype, 'requestPermission', original => function (...args) {
        record('permission', { phase: 'request', requestedAtMs: clock(), api: typeof win.DeviceMotionEvent?.requestPermission === 'function' ? 'DeviceMotionEvent.requestPermission' : 'Generic/legacy permission check', userActivationActive: win.navigator.userActivation?.isActive ?? null, userActivationHasBeenActive: win.navigator.userActivation?.hasBeenActive ?? null, ios: this._isIOS });
        const promise = original.apply(this, args);
        promise.then(granted => record('permission', { phase: 'result', resolvedAtMs: clock(), granted, outcome: this.permission ?? null, userActivationActive: win.navigator.userActivation?.isActive ?? null }), error => record('permission', { phase: 'result', resolvedAtMs: clock(), error: error.message }));
        return promise;
    });
    hook(IMU.prototype, 'calibrate', original => async function (...args) { const value = await original.apply(this, args); record('gyro_calibration', { result: value, calibrated: this.isCalibrated(), durationRequestedMs: args[0] ?? 1500 }); return value; });
    hook(IMU.prototype, 'flush', original => function (...args) { const result = original.apply(this, args); for (let i = 0; result.data && i < result.count; i++) record('flushed_device_imu', { timestampS: result.data[i * 7], values: Array.from(result.data.subarray(i * 7 + 1, i * 7 + 7)) }); return result; });
    hook(IMU.prototype, '_tryGenericSensor', original => function (...args) {
        const result = original.apply(this, args);
        if (result) for (const [axis, sensor] of [['accel', this._accel], ['gyro', this._gyro]]) {
            const listener = () => record('raw_generic', { sensor: axis, sensorTimestampMs: sensor.timestamp ?? null, arrivalMs: clock(), values: [sensor.x, sensor.y, sensor.z] });
            sensor.addEventListener('reading', listener);
            restore.push(() => sensor.removeEventListener('reading', listener));
        }
        return result;
    });
    const cameraSnapshot = camera => {
        let settings = null;
        let capabilities = null;
        try { const track = camera.getVideoTrack?.(); settings = safeTrack(track?.getSettings?.()); capabilities = safeTrack(track?.getCapabilities?.()); } catch {}
        return { native: [camera._nativeWidth, camera._nativeHeight], portrait: [camera._portraitWidth, camera._portraitHeight], output: [camera.width, camera.height], rotation: camera._rotateMode, crop: camera._cropMode, cropOffsetY: camera._cropOffsetY, dimsSwapped: camera._dimsSwapped, grayscale: camera._useWebGL ? 'webgl' : 'cpu', settings, capabilities };
    };
    for (const name of ['initialize', 'redetectOrientation']) hook(Camera.prototype, name, original => async function (...args) {
        cameras.add(this);
        const value = await original.apply(this, args);
        record('camera_config', { method: name, ...cameraSnapshot(this) });
        if (name === 'initialize' && this.video?.requestVideoFrameCallback) {
            const video = this.video;
            let handle;
            let enabled = true;
            const observe = (callbackNow, meta) => { if (!enabled) return; record('video_frames', { callbackNowMs: callbackNow, arrivalMs: clock(), mediaTimeS: meta.mediaTime, presentationTimeMs: meta.presentationTime ?? null, expectedDisplayTimeMs: meta.expectedDisplayTime ?? null, captureTimeMs: meta.captureTime ?? null, receiveTimeMs: meta.receiveTime ?? null, presentedFrames: meta.presentedFrames, width: meta.width, height: meta.height }); handle = video.requestVideoFrameCallback(observe); };
            handle = video.requestVideoFrameCallback(observe);
            restore.push(() => { enabled = false; video.cancelVideoFrameCallback?.(handle); });
        }
        return value;
    });
    hook(Camera.prototype, 'enableLandscapeCrop', original => function (...args) { const value = original.apply(this, args); record('camera_config', { method: 'enableLandscapeCrop', ...cameraSnapshot(this) }); return value; });
    hook(Camera.prototype, 'captureGrayscale', original => function (...args) { const startedAt = clock(); const result = original.apply(this, args); record('captures', { startedAtMs: startedAt, completedAtMs: clock(), bytes: result?.byteLength || 0, videoCurrentTimeS: this.video?.currentTime ?? null, width: this.width, height: this.height }); return result; });
    for (const name of ['configure', 'setMobileParams', 'setTrackingParams', 'setFThreshold', 'setPnPParams']) hook(VIOWrapper.prototype, name, original => function (...args) { wrappers.add(this); record('vio_config', { method: name, args }); const result = original.apply(this, args); result.then(success => record('vio_config_result', { method: name, success }), error => record('vio_config_result', { method: name, error: error.message })); return result; });
    hook(VIOWrapper.prototype, 'sendIMU', original => function (data, count) { const readings = []; for (let i = 0; data && i < count; i++) readings.push({ timestampS: data[i * 7], values: Array.from(data.subarray(i * 7 + 1, i * 7 + 7)) }); const accepted = original.call(this, data, count); for (const row of readings) record('transformed_vio_imu', { ...row, accepted }); return accepted; });
    hook(VIOWrapper.prototype, 'sendFrame', original => function (image, timestamp) {
        const current = wrapperState(this);
        const sentAt = clock();
        const accepted = original.call(this, image, timestamp);
        const attemptSequence = ++current.sequence;
        const envelope = accepted ? this._activeFrame : null;
        const sequence = envelope?.sequence ?? attemptSequence;
        const gapS = current.previousFrameTimestamp === null ? null : timestamp - current.previousFrameTimestamp;
        record('frames', { attemptSequence, requestId: envelope?.requestId ?? null, clientEpoch: envelope?.clientEpoch ?? null, sourceTiming: win.__mobileSLAM?.app.lastFrameTiming ?? null, calibration: win.__mobileSLAM?.app.calibrationProvenance ?? null, sequence, generation: current.generation, submittedTimestampS: timestamp, arrivalMs: sentAt, accepted, grayBytes: image?.byteLength || 0, timestampProvenance: 'video presentation/callback or canvas read arrival proxy; physical exposure unverified', gapSinceLastAcceptedS: gapS });
        if (accepted) { current.pending.push({ sequence, requestId: envelope?.requestId, clientEpoch: envelope?.clientEpoch, timestamp, sentAt, gapS, generation: current.generation }); current.previousFrameTimestamp = timestamp; }
        return accepted;
    });
    hook(VIOWrapper.prototype, '_handleWorkerMessage', original => function (message) {
        const current = wrapperState(this);
        const previous = this.getLatestResult();
        const value = original.call(this, message);
        if (message.type === 'result') {
            const data = this.getLatestResult();
            const matched = data && data !== previous && data.requestId === message.requestId && data.clientEpoch === message.clientEpoch && data.sequence === message.sequence;
            if (!matched) { record('stale_results', { requestId: message.requestId, clientEpoch: message.clientEpoch, sequence: message.sequence, receivedAtMs: clock() }); return value; }
            const index = current.pending.findIndex(row => row.requestId === message.requestId && row.clientEpoch === message.clientEpoch);
            const pending = index >= 0 ? current.pending.splice(index, 1)[0] : null;
            const pose = data.pose ? Array.from(data.pose) : null;
            const interruption = current.previousStatus === 2 && data.statusCode !== 2;
            const epochChanged = current.previousEngineEpoch !== null && current.previousEngineEpoch !== data.engineEpoch;
            if (interruption || epochChanged || !data.poseFresh || !data.poseValid) current.previousPose = null;
            const delta = data.poseFresh && data.poseValid ? poseDelta(current.previousPose, pose) : null;
            record('results', { sequence: data.sequence, requestId: data.requestId, clientEpoch: data.clientEpoch, engineEpoch: data.engineEpoch,
                generation: pending?.generation ?? current.generation, staleGeneration: false, association: 'matching requestId/clientEpoch/sequence from Worker',
                receivedAtMs: clock(), submittedTimestampS: data.inputTimestamp, engineTimestampS: data.poseTimestamp,
                roundTripMs: data.roundtripMs ?? (pending ? clock() - pending.sentAt : null),
                submittedTimestampAgeProxyMs: Number.isFinite(data.inputTimestamp) ? clock() - data.inputTimestamp * 1000 : null,
                poseTimestampAgeProxyMs: Number.isFinite(data.poseTimestamp) ? clock() - data.poseTimestamp * 1000 : null,
                physicalCaptureToPoseMs: null, poseTimestampProvenance: 'actual engine fresh poseTimestamp; input camera time remains exposure-unverified',
                status: data.statusCode, reason: data.reason, initialized: data.initialized, features: data.featureCount,
                poseFresh: data.poseFresh, poseValid: data.poseValid, imuCount: data.imuCount ?? null, imuDrops: data.imuDrops,
                solverIterations: data.solverIterations, solverTermination: data.solverTermination,
                pose, finitePose: pose ? pose.every(Number.isFinite) : false, delta,
                jumpMarker: !!delta && (delta.translationM > 0.5 || delta.rotationDeg > 20),
                jumpMarkerThresholds: { translationM: 0.5, rotationDeg: 20, scope: 'Exploratory marker, not an accuracy/validity gate' },
                deltaAcrossReset: poseDelta(current.previousAnyPose, pose), trackingInterrupted: interruption, engineEpochChanged: epochChanged,
                inferredFrameGapGuard: !!pending && pending.gapS > 1.5 && data.statusCode === 1, transport: this.getMetrics?.() });
            if (pose && data.poseFresh && data.poseValid) { current.previousPose = pose; current.previousAnyPose = pose; }
            current.previousStatus = data.statusCode;
            current.previousEngineEpoch = data.engineEpoch;
        } else if (message.type === 'wasm_log' && message.data) record('wasm_logs', { level: message.data.level, text: String(message.data.msg).slice(0, 2000) });
        return value;
    });
    hook(VIOWrapper.prototype, 'reset', original => function (...args) { const current = wrapperState(this); current.generation++; current.previousPose = null; current.previousFrameTimestamp = null; record('resets', { method: 'wrapper.reset', generation: current.generation, pendingSequences: current.pending.map(row => row.sequence) }); current.pending = []; current.previousEngineEpoch = null; return original.apply(this, args); });
    const manifest = {};
    const wasmFile = new URL(win.location.href).searchParams.get('wasm') || 'vio_engine.js';
    if (win.crypto?.subtle) for (const url of [new URL('/index.html', win.location.href).href, appUrl.href, moduleUrl('camera.js'), moduleUrl('imu.js'), moduleUrl('vio-wrapper.js'), new URL('/js/vio-worker.js', win.location.href).href, new URL('/' + wasmFile, win.location.href).href]) {
        try { const bytes = await (await win.fetch(url)).arrayBuffer(); manifest[new URL(url).pathname + new URL(url).search] = { bytes: bytes.byteLength, sha256: [...new Uint8Array(await win.crypto.subtle.digest('SHA-256', bytes))].map(byte => byte.toString(16).padStart(2, '0')).join('') }; } catch (error) { manifest[url] = { error: error.message }; }
    }
    return { moduleUrls: { app: appUrl.href, camera: moduleUrl('camera.js'), imu: moduleUrl('imu.js'), wrapper: moduleUrl('vio-wrapper.js') }, manifest, overhead, get blockedLogWrites() { return blockedLogWrites; }, stopResources, detach };
    } catch (error) { stopResources(); detach(); throw error; }
}

async function setupPage() {
    const ui = Object.fromEntries(['open', 'record', 'stop', 'export', 'seconds', 'status', 'summary', 'app'].map(id => [id, document.getElementById(id)]));
    let recorder = null;
    let bridge = null;
    let timer = null;
    let durationTimer = null;
    let saved = null;
    const finish = reason => {
        if (!recorder) return;
        clearInterval(timer);
        clearTimeout(durationTimer);
        saved = recorder.export();
        if (saved) { saved.summary = summarize(saved); saved.metadata.blockedRemoteLogWrites = bridge?.blockedLogWrites || 0; saved.metadata.observerOverhead = bridge?.overhead || null; }
        const resourceErrors = bridge?.stopResources() || [];
        if (saved) saved.metadata.stopResourceErrors = resourceErrors;
        bridge?.detach();
        bridge = null;
        ui.app.src = 'about:blank';
        ui.record.disabled = true;
        ui.stop.disabled = true;
        ui.export.disabled = !saved;
        ui.open.disabled = false;
        ui.status.textContent = `기록 종료: ${reason}. JSON은 이 브라우저에만 저장.`;
        ui.summary.textContent = JSON.stringify(saved?.summary || {}, null, 2);
    };
    const load = async () => {
        ui.open.disabled = true;
        ui.record.disabled = true;
        ui.app.style.pointerEvents = 'none';
        ui.status.textContent = '기존 앱과 관측 모듈 로딩…';
        try {
            await new Promise(resolve => { ui.app.onload = resolve; ui.app.src = '/index.html' + window.location.search; });
            const child = ui.app.contentWindow;
            recorder = createRecorder({ maxDurationMs: Math.min(120, Math.max(10, Number(ui.seconds.value) || 60)) * 1000, onStop: finish });
            child.__mobileAuditRecorder = { push: (...args) => recorder.push(...args) };
            bridge = await child.eval("import('/js/audit-mobile.js?audit-realm=1').then(module => module.installInRealm(window, window.__mobileAuditRecorder))");
        } catch (error) { bridge?.stopResources(); bridge?.detach(); bridge = null; clearInterval(timer); clearTimeout(durationTimer); ui.app.src = 'about:blank'; ui.status.textContent = `로딩 실패: ${error.message}`; ui.open.disabled = false; return; }
        ui.record.disabled = false;
        ui.status.textContent = '기록 시작 → 아래 기존 앱의 Start → 기기 권한 허용.';
    };
    ui.open.addEventListener('click', load);
    ui.record.addEventListener('click', () => {
        recorder.start({ page: 'existing /index.html in same-origin iframe', userAgent: navigator.userAgent, platform: navigator.platform, touchPoints: navigator.maxTouchPoints, secureContext: isSecureContext, crossOriginIsolated, clocks: { parentPerformanceTimeOriginMs: performance.timeOrigin, appPerformanceTimeOriginMs: ui.app.contentWindow.performance.timeOrigin, recorderAtMs: 'Parent audit page performance.now arrival; add parentPerformanceTimeOriginMs for epoch mapping', hookTimestamps: 'App iframe performance.now domain; add appPerformanceTimeOriginMs for epoch mapping', sensorEventTimestamps: 'Browser/API-provided raw timestamp fields; their sampling semantics are unverified', physicalHardwareCapture: 'Not established by performance.now, timeOrigin, or submitted-frame association' }, screen: { width: screen.width, height: screen.height, pixelRatio: devicePixelRatio }, capabilities: { mediaDevices: !!navigator.mediaDevices?.getUserMedia, DeviceMotionEvent: 'DeviceMotionEvent' in window, Accelerometer: 'Accelerometer' in window, Gyroscope: 'Gyroscope' in window, requestVideoFrameCallback: 'requestVideoFrameCallback' in HTMLVideoElement.prototype }, moduleUrls: bridge.moduleUrls, manifest: bridge.manifest });
        ui.record.disabled = true;
        ui.open.disabled = true;
        ui.stop.disabled = false;
        ui.export.disabled = true;
        ui.app.style.pointerEvents = 'auto';
        ui.status.textContent = '기록 중. 아래 앱의 Start를 직접 누르고 먼저 기기를 정지.';
        durationTimer = setTimeout(() => recorder.stop('duration_limit'), recorder.export().limits.maxDurationMs);
        timer = setInterval(() => { ui.summary.textContent = JSON.stringify(recorder.summary(), null, 2); }, 1000);
    });
    ui.stop.addEventListener('click', () => recorder.stop('manual'));
    ui.export.addEventListener('click', () => {
        const url = URL.createObjectURL(new Blob([JSON.stringify(saved, null, 2)], { type: 'application/json' }));
        const link = document.createElement('a');
        link.href = url;
        link.download = `mobile-slam-audit-${new Date().toISOString().replace(/[:.]/g, '-')}.json`;
        link.click();
        setTimeout(() => URL.revokeObjectURL(url), 1000);
    });
    window.__mobileAudit = { export: () => recorder?.active ? recorder.export() : saved, summary: () => recorder?.active ? recorder.summary() : saved?.summary, get ready() { return !!bridge; }, get recording() { return !!recorder?.active; } };
    window.addEventListener('beforeunload', () => { clearInterval(timer); clearTimeout(durationTimer); bridge?.stopResources(); bridge?.detach(); });
}

if (typeof document !== 'undefined' && document.body?.dataset.mobileAudit === 'page') setupPage();

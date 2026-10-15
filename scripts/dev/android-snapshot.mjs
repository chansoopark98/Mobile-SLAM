import fs from 'node:fs/promises';
import path from 'node:path';
import { parseArgs } from 'node:util';
import { randomUUID } from 'node:crypto';
import { copyLowFeatureMetadata } from './android-input-record.mjs';

// Read only the existing Mobile-SLAM page. Do not launch or navigate Chrome.
const { values } = parseArgs({ options: {
    cdp: { type: 'string', default: 'http://127.0.0.1:9222' },
    origin: { type: 'string', default: 'https://dev.serdic.com:7002' },
    samples: { type: 'string', default: '8' },
    'record-seconds': { type: 'string', default: '0' },
    timer: { type: 'boolean', default: false },
    output: { type: 'string' },
} });
const endpoint = new URL(values.cdp);
if (!['127.0.0.1', 'localhost', '[::1]'].includes(endpoint.hostname) || endpoint.protocol !== 'http:') {
    throw new Error('CDP must use a local HTTP endpoint forwarded by ADB');
}
const origin = new URL(values.origin).origin;
const count = Number(values.samples);
if (!Number.isInteger(count) || count < 1 || count > 120) throw new Error('samples must be 1..120');
const seconds = Number(values['record-seconds']);
if (!Number.isInteger(seconds) || seconds < 0 || seconds > 60) throw new Error('record-seconds must be 0..60');
if (values.timer && seconds !== 30) throw new Error('--timer requires --record-seconds 30');
const output = values.output || `build/dev-android/snapshot-${new Date().toISOString().replace(/[:.]/g, '-')}.json`;
const token = randomUUID();
const report = { schema: 'mobile-slam-android-cdp-v1', startedAt: new Date().toISOString(), origin,
    scope: 'Read-only numeric observations of one existing real-device tab; no image capture or independent accuracy ground truth', samples: [] };
const targets = await (await fetch(new URL('/json/list', endpoint), { signal: AbortSignal.timeout(5000) })).json();
const matches = targets.filter(target => {
    try { const url = new URL(target.url); return target.type === 'page' && url.origin === origin && ['/', '/index.html'].includes(url.pathname); }
    catch { return false; }
});
if (matches.length !== 1) throw new Error(`Expected one Mobile-SLAM tab, found ${matches.length}`);
const socket = new WebSocket(matches[0].webSocketDebuggerUrl);
let nextId = 0;
function call(method, params) {
    const id = ++nextId;
    return new Promise((resolve, reject) => {
        const timer = setTimeout(() => { socket.removeEventListener('message', receive); reject(new Error(`CDP timeout: ${method}`)); }, 5000);
        function receive(event) {
            const message = JSON.parse(event.data);
            if (message.id !== id) return;
            clearTimeout(timer); socket.removeEventListener('message', receive);
            if (message.error) reject(new Error(JSON.stringify(message.error)));
            else resolve(message.result);
        }
        socket.addEventListener('message', receive);
        socket.send(JSON.stringify({ id, method, params }));
    });
}
function snapshot(copyMetadata = copyLowFeatureMetadata) {
    const app = window.__mobileSLAM?.app;
    if (!app) return { available: false, visibility: document.visibilityState };
    const result = app.vio.getLatestResult();
    const canvas = document.getElementById('canvas-3d');
    const box = canvas?.getBoundingClientRect();
    const settings = app.camera.getVideoTrack?.()?.getSettings();
    return { available: true, nowMs: performance.now(), timeOriginMs: performance.timeOrigin,
        status: document.getElementById('status')?.textContent, visibility: document.visibilityState, focused: document.hasFocus(),
        running: app.running, blurPaused: app._blurPaused, reconfiguring: app._reconfiguring,
        sensor: app.imu.getSensorType(), diagnostics: window.__mobileSLAM.diagnostics(),
        result: result && Object.fromEntries(['clientEpoch', 'engineEpoch', 'sequence', 'statusCode', 'reason', 'initialized',
            'featureCount', 'imuCount', 'inputTimestamp', 'engineFrameTimestamp', 'imuEndpointTimestamp',
            'poseTimestamp', 'poseFresh', 'poseValid', 'engineProcessingMs', 'imuWaitMs', 'imuWaitBudgetMs', 'workerProcessingMs',
            'roundtripMs', 'imuIntervalLoss'].map(key => [key, result[key]])),
        pose: result?.pose ? Array.from(result.pose) : null, ...copyMetadata(result),
        lowFeatureDisplay: { ...app.lowFeatureDisplay },
        camera: settings && Object.fromEntries(['width', 'height', 'frameRate', 'facingMode', 'aspectRatio'].map(key => [key, settings[key]])),
        renderer: { trajectoryPoints: app.renderer?._trajCount, pixelRatio: app.renderer?.renderer.getPixelRatio(),
            css: box && { width: box.width, height: box.height, bottom: box.bottom },
            buffer: canvas && { width: canvas.width, height: canvas.height }, viewportHeight: innerHeight, devicePixelRatio } };
}
function startRecording(seconds, token, showTimer = false, copyMetadata = copyLowFeatureMetadata) {
    const app = window.__mobileSLAM?.app, worker = app?.vio.worker;
    if (!app || !worker || !app.running) throw new Error('Start Mobile-SLAM before recording');
    if (window.__mobileSLAMAndroidRecording || window.__mobileSLAMInputCapture) throw new Error('A recording is already active');
    const settings = app.camera.getVideoTrack?.()?.getSettings();
    const capabilities = app.camera.getVideoTrack?.()?.getCapabilities();
    const record = { startedAtMs: performance.now(), timeOriginMs: performance.timeOrigin,
        config: structuredClone(app._lastConfigParams), calibration: structuredClone(app.calibrationProvenance),
        gyroBias: structuredClone(app.imu.getGyroBias()), gyroBiasApplied: app.imu.getDiagnostics().gyroBiasApplied ?? null,
        camera: { settings: settings && Object.fromEntries(['width', 'height', 'frameRate', 'facingMode', 'aspectRatio', 'zoom', 'focusDistance'].map(key => [key, settings[key]])),
            capabilities: capabilities && Object.fromEntries(['width', 'height', 'frameRate', 'zoom', 'focusDistance', 'focusMode'].map(key => [key, capabilities[key]])),
            rotation: app.camera.getRotateMode(), processing: [app._processWidth, app._processHeight] },
        results: [], imu: [], dropped: { results: 0, imu: 0 }, stoppedAtMs: null,
        observationLoad: 'Copies numeric IMU rows and result metadata; optional timer and submission/UI hooks add main-thread work.' };
    const frameKey = (clientEpoch, requestId, sequence) => `${clientEpoch}:${requestId}:${sequence}`;
    const submitted = new Map(), received = new Map();
    const receive = event => {
        if (event.data.type !== 'result') return;
        const data = event.data.data;
        if (!data) return;
        const key = frameKey(event.data.clientEpoch, event.data.requestId, event.data.sequence);
        const frame = submitted.get(key);
        submitted.delete(key);
        if (record.results.length >= 1500) { record.dropped.results++; return; }
        const arrivalMs = performance.now();
        const row = { arrivalMs, requestId: event.data.requestId,
            clientEpoch: event.data.clientEpoch, sequence: event.data.sequence,
            sentAtMs: frame?.sentAtMs ?? null, sourceTiming: frame?.sourceTiming ?? null,
            frameCopyMs: frame?.frameCopyMs ?? null,
            roundtripMs: frame ? arrivalMs - frame.sentAtMs : null, uiStateUpdatedAtMs: null,
            ...Object.fromEntries(['engineEpoch', 'initialized', 'statusCode', 'reason', 'featureCount', 'imuCount',
                'inputTimestamp', 'engineFrameTimestamp', 'imuEndpointTimestamp', 'poseTimestamp', 'poseFresh', 'poseValid',
                'engineProcessingMs', 'imuWaitMs', 'imuWaitBudgetMs', 'workerProcessingMs', 'workerQueueMs', 'solverIterations',
                'solverTermination', 'imuDrops', 'imuIntervalLoss', 'motionDiscontinuity', 'pipeline', 'candidatePipeline'].map(key => [key, data[key]])),
            pose: data.pose ? Array.from(data.pose) : null, ...copyMetadata(data) };
        record.results.push(row);
        received.set(key, row);
    };
    const originalIMU = app.vio.sendIMU, originalFrame = app.vio.sendFrame, originalConsume = app._consumeResult;
    function sendIMU(data, count) {
        for (let index = 0; index < count; index++) {
            if (record.imu.length >= 12000) { record.dropped.imu += count - index; break; }
            record.imu.push(Array.from(data.subarray(index * 7, index * 7 + 7)));
        }
        return originalIMU.apply(this, arguments);
    }
    function sendFrame(image, timestamp) {
        const accepted = originalFrame.apply(this, arguments);
        if (accepted && this._activeFrame) {
            const frame = this._activeFrame;
            const timing = app.lastFrameTiming;
            submitted.set(frameKey(frame.clientEpoch, frame.requestId, frame.sequence), {
                sentAtMs: frame.sentAtMs - performance.timeOrigin,
                frameCopyMs: frame.frameCopyMs,
                sourceTiming: timing?.timestampS === timestamp &&
                    (timing.clientEpoch == null || timing.clientEpoch === frame.clientEpoch) ? structuredClone(timing) : null,
            });
        }
        return accepted;
    }
    function consumeResult(result) {
        const value = originalConsume.apply(this, arguments);
        if (result) {
            const key = frameKey(result.clientEpoch, result.requestId, result.sequence);
            const row = received.get(key);
            if (row) { row.uiStateUpdatedAtMs = performance.now(); row.lowFeatureDisplay = { ...app.lowFeatureDisplay }; received.delete(key); }
        }
        return value;
    }
    worker.addEventListener('message', receive);
    app.vio.sendIMU = sendIMU;
    app.vio.sendFrame = sendFrame;
    app._consumeResult = consumeResult;
    let timer, countdown = null, countdownTimer = null;
    const stop = (reason = 'host_complete') => {
        if (record.stoppedAtMs !== null) return;
        worker.removeEventListener('message', receive);
        if (app.vio.sendIMU === sendIMU) app.vio.sendIMU = originalIMU;
        if (app.vio.sendFrame === sendFrame) app.vio.sendFrame = originalFrame;
        if (app._consumeResult === consumeResult) app._consumeResult = originalConsume;
        submitted.clear(); received.clear();
        record.stoppedAtMs = performance.now();
        record.stopReason = reason;
        clearTimeout(timer); clearInterval(countdownTimer);
        countdown?.remove();
    };
    timer = setTimeout(() => stop('duration'), seconds * 1000);
    window.__mobileSLAMAndroidRecording = { record, stop, token };
    if (showTimer) {
        try {
            countdown = document.createElement('div');
            countdown.style.cssText = 'position:fixed;top:48px;left:12px;z-index:10001;padding:8px 12px;background:#111;color:#00ff88;font:16px sans-serif;pointer-events:none;border:1px solid #00ff88;border-radius:4px';
            document.body.appendChild(countdown);
            const update = () => {
                const elapsed = (performance.now() - record.startedAtMs) / 1000;
                const instruction = elapsed < 10 ? '정지' : elapsed < 20 ? '평소 Lost가 발생하는 정도로 이동·회전' : '다시 정지';
                countdown.textContent = `${Math.max(0, Math.ceil(seconds - elapsed))}초 · ${instruction}`;
            };
            update(); countdownTimer = setInterval(update, 250);
        } catch (error) {
            stop('setup_error');
            delete window.__mobileSLAMAndroidRecording;
            throw error;
        }
    }
    return { startedAtMs: record.startedAtMs, seconds };
}
async function finishRecording(reason = 'host_complete') {
    const expression = `(() => {
        const capture = window.__mobileSLAMAndroidRecording;
        if (!capture || capture.token !== ${JSON.stringify(token)}) return null;
        capture.stop(${JSON.stringify(reason)}); delete window.__mobileSLAMAndroidRecording; return capture.record;
    })()`;
    const result = await call('Runtime.evaluate', { expression, returnByValue: true, userGesture: false });
    if (result.exceptionDetails) throw new Error(JSON.stringify(result.exceptionDetails));
    return result.result.value;
}
try {
    await new Promise((resolve, reject) => {
        const timer = setTimeout(() => reject(new Error('CDP connection timeout')), 5000);
        socket.addEventListener('open', () => { clearTimeout(timer); resolve(); }, { once: true });
        socket.addEventListener('error', () => { clearTimeout(timer); reject(new Error('CDP connection failed')); }, { once: true });
    });
    if (seconds) {
        const started = await call('Runtime.evaluate', { expression: `(${startRecording.toString()})(${seconds}, ${JSON.stringify(token)}, ${values.timer}, ${copyLowFeatureMetadata.toString()})`, returnByValue: true, userGesture: false });
        if (started.exceptionDetails) throw new Error(JSON.stringify(started.exceptionDetails));
    }
    for (let index = 0; index < count; index++) {
        const result = await call('Runtime.evaluate', { expression: `(${snapshot.toString()})(${copyLowFeatureMetadata.toString()})`, returnByValue: true, userGesture: false });
        if (result.exceptionDetails) throw new Error(JSON.stringify(result.exceptionDetails));
        report.samples.push(result.result.value);
        if (index + 1 < count) await new Promise(resolve => setTimeout(resolve, 1000));
    }
    if (seconds) {
        const remainingMs = Math.max(0, seconds * 1000 - count * 1000 + 1500);
        await new Promise(resolve => setTimeout(resolve, remainingMs));
        report.recording = await finishRecording();
        if (!report.recording) throw new Error('Recording was lost after page reload or navigation');
        if (!report.recording.results.length) throw new Error('Recording has no completed Worker frames');
    }
    report.completed = true;
} catch (error) {
    report.completed = false; report.error = error.message; process.exitCode = 1;
} finally {
    if (seconds && !report.recording && socket.readyState === WebSocket.OPEN) {
        try { report.recording = await finishRecording('host_error'); }
        catch (error) { report.cleanupError = error.message; }
    }
    socket.close();
    await fs.mkdir(path.dirname(output), { recursive: true });
    // Keep prior observations intact if an explicit output path is reused.
    await fs.writeFile(output, JSON.stringify(report, null, 2) + '\n', { flag: 'wx' });
}
console.log(JSON.stringify({ output, completed: report.completed, samples: report.samples.length,
    reasons: [...new Set(report.samples.map(sample => sample.result?.reason))],
    sensor: report.samples.at(-1)?.sensor, initialized: report.samples.at(-1)?.result?.initialized,
    error: report.error }));

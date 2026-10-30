import fs from 'node:fs/promises';
import path from 'node:path';
import { pathToFileURL } from 'node:url';
import { parseArgs } from 'node:util';
import { randomUUID, createHash } from 'node:crypto';

/** Bounded observation copy, not acceptance validation; visual result fields stay untouched. */
export function copyLowFeatureMetadata(data) {
    const copy = (value, fields, arrays) => {
        if (!value || typeof value !== 'object') return null;
        const result = {};
        for (const key of fields) {
            const item = value[key];
            if (item === null || typeof item === 'number' || typeof item === 'boolean') result[key] = item;
            else if (typeof item === 'string') result[key] = item.slice(0, 128);
        }
        for (const [key, length] of arrays) {
            const item = value[key];
            result[key] = (Array.isArray(item) || ArrayBuffer.isView(item)) && item.length === length
                ? Array.from(item, number => typeof number === 'number' ? number : null) : null;
        }
        return result;
    };
    return {
        inertialPrediction: copy(data?.inertialPrediction,
            ['schema','mode','poseFrame','engineEpoch','timestamp','anchorTimestamp','maxHorizonSeconds',
                'visualSupported','metricTranslationSupported','positionUncertain'], [['pose',16]]),
        groundPlane: copy(data?.groundPlane,
            ['schema','kind','semanticClassification','state','engineEpoch','clientEpoch','sourceTimestamp',
                'sourceSequence','offset','supportCount','sourcePointCount','rms','maxAgeSeconds'],
            [['normal',3],['centroid',3],['corners',12]]),
    };
}

export function begin(seconds, maxFrames, token, showTimer, copyMetadata = copyLowFeatureMetadata, queueMiB = 16) {
    const app = window.__mobileSLAM?.app, worker = app?.vio.worker;
    if (!app?.running || !worker) throw new Error('Start Mobile-SLAM before arming input capture');
    if (window.__mobileSLAMInputCapture || window.__mobileSLAMAndroidRecording) throw new Error('Finish the existing recording first');
    const frameBytes = app._processWidth * app._processHeight;
    const maxQueuedBytes = Math.min(queueMiB * 1024 * 1024, maxFrames * frameBytes);
    if (!Number.isSafeInteger(frameBytes) || frameBytes <= 0 || frameBytes > maxQueuedBytes) throw new Error('Invalid or oversized grayscale frame');
    const record = { startedAtMs: performance.now(), timeOriginMs: performance.timeOrigin,
        width: app._processWidth, height: app._processHeight, config: structuredClone(app._lastConfigParams),
        profile: app.activeConfig, calibration: structuredClone(app.calibrationProvenance),
        imuDiagnostics: app.imu.getDiagnostics(), frames: [], acceptedFrameMetadata: [], imu: [], results: [],
        capturePolicy: { mode: 'host_stream', maxQueuedBytes, maxQueuedFrames: maxFrames, maxBatchBytes: 4 * 1024 * 1024,
            metadataLimit: 10000, acknowledgeAfter: 'host_file_sync' },
        estimatorLifecycle: { resetRequested: true, clientEpochBeforeReset: app.vio._clientEpoch,
            clientEpochAtStart: null, initializedBeforeReset: app.vio.isInitialized() },
        counters: { acceptedFrames: 0, droppedFrames: 0, uncopiedFrames: 0, imuOverflow: 0, resultOverflow: 0,
            queueOverflow: 0, invalidPayload: 0, metadataOverflow: 0, queuedBytes: 0, peakQueuedBytes: 0 }, stoppedAtMs: null };
    const queue = [], acceptedRequests = new Set(), frameMethod = app.vio.sendFrame, imuMethod = app.vio.sendIMU;
    const configuration = JSON.stringify(app._lastConfigParams);
    let countdown = null, countdownTimer = null;
    if (showTimer) {
        countdown = document.createElement('div');
        countdown.style.cssText = 'position:fixed;top:48px;left:12px;z-index:10001;padding:8px 12px;background:#111;color:#00ff88;font:16px sans-serif;pointer-events:none;border:1px solid #00ff88;border-radius:4px';
        document.body.appendChild(countdown);
        const update = () => {
            const elapsed = (performance.now() - record.startedAtMs) / 1000;
            const instruction = elapsed < 10 ? '정지' : elapsed < seconds - 10 ? '천천히 좌우 왕복·가벼운 회전' : '다시 정지';
            countdown.textContent = `${Math.max(0, Math.ceil(seconds - elapsed))}초 · ${instruction}`;
        };
        update(); countdownTimer = setInterval(update, 250);
    }
    function sendFrame(gray, timestamp) {
        if (app._processWidth !== record.width || app._processHeight !== record.height ||
            JSON.stringify(app._lastConfigParams) !== configuration) {
            stop('configuration_changed'); return frameMethod.apply(this, arguments);
        }
        // Snapshot before the original method can transfer/detach or reuse its input.
        const validPayload = gray?.byteLength === frameBytes;
        const hasRoom = validPayload && record.acceptedFrameMetadata.length < record.capturePolicy.metadataLimit &&
            queue.length < maxFrames && record.counters.queuedBytes + frameBytes <= maxQueuedBytes;
        const copy = hasRoom ? gray.slice() : null;
        const accepted = frameMethod.apply(this, arguments);
        if (!accepted) { record.counters.droppedFrames++; return accepted; }
        const index = record.counters.acceptedFrames++;
        if (record.acceptedFrameMetadata.length >= record.capturePolicy.metadataLimit) {
            record.counters.metadataOverflow++; record.counters.uncopiedFrames++; return accepted;
        }
        const metadata = { index, timestamp, submittedAtMs: performance.now(),
            requestId: this._activeFrame?.requestId ?? null, clientEpoch: this._activeFrame?.clientEpoch ?? null,
            sourceTiming: structuredClone(app.lastFrameTiming), payloadStatus: hasRoom ? 'queued' : validPayload ? 'queue_overflow' : 'invalid_payload' };
        record.acceptedFrameMetadata.push(metadata);
        acceptedRequests.add(`${metadata.clientEpoch}:${metadata.requestId}`);
        if (!hasRoom) { record.counters[validPayload ? 'queueOverflow' : 'invalidPayload']++; record.counters.uncopiedFrames++; return accepted; }
        queue.push({ metadata, gray: copy });
        record.counters.queuedBytes += copy.byteLength;
        record.counters.peakQueuedBytes = Math.max(record.counters.peakQueuedBytes, record.counters.queuedBytes);
        return accepted;
    }
    function sendIMU(data, count) {
        // Copy before transfer; this is a record of submission, not engine acceptance.
        const copy = data.slice(0, count * 7);
        const accepted = imuMethod.apply(this, arguments);
        if (accepted) for (let index = 0; index < count; index++) {
            if (record.imu.length >= 12000) { record.counters.imuOverflow += count - index; break; }
            record.imu.push(Array.from(copy.subarray(index * 7, index * 7 + 7)));
        }
        return accepted;
    }
    const receive = event => {
        if (event.data.type !== 'result' || !event.data.data) return;
        if (record.results.length >= 10000) { record.counters.resultOverflow++; return; }
        const data = event.data.data;
        record.results.push({ captureAccepted: acceptedRequests.has(`${event.data.clientEpoch}:${event.data.requestId}`),
            arrivalMs: performance.now(), requestId: event.data.requestId, clientEpoch: event.data.clientEpoch, sequence: event.data.sequence,
            ...Object.fromEntries(['engineEpoch', 'initialized', 'statusCode', 'reason', 'featureCount', 'imuCount', 'inputTimestamp',
                'poseTimestamp', 'poseFresh', 'poseValid', 'imuEndpointTimestamp', 'engineProcessingMs', 'workerProcessingMs',
                'imuWaitMs', 'imuWaitBudgetMs', 'workerQueueMs', 'imuDrops', 'motionDiscontinuity', 'solverIterations', 'solverTermination',
                'pipeline', 'candidatePipeline'].map(key => [key, data[key]])),
            pose: data.pose ? Array.from(data.pose) : null, ...copyMetadata(data) });
    };
    app.vio.sendFrame = sendFrame; app.vio.sendIMU = sendIMU; worker.addEventListener('message', receive);
    const stop = (reason = 'duration') => {
        if (record.stoppedAtMs !== null) return;
        if (app.vio.sendFrame === sendFrame) app.vio.sendFrame = frameMethod;
        if (app.vio.sendIMU === sendIMU) app.vio.sendIMU = imuMethod;
        worker.removeEventListener('message', receive); clearTimeout(timer);
        clearInterval(countdownTimer);
        if (countdown) countdown.remove();
        record.stoppedAtMs = performance.now(); record.stopReason = reason;
        record.finalDiagnostics = window.__mobileSLAM.diagnostics();
    };
    const timer = setTimeout(() => stop('duration'), seconds * 1000);
    function readBatch() {
        const frames = []; let bytes = 0;
        for (const item of queue) {
            if (frames.length && bytes + item.gray.byteLength > record.capturePolicy.maxBatchBytes) break;
            const parts = [];
            for (let i = 0; i < item.gray.length; i += 16384) parts.push(String.fromCharCode(...item.gray.subarray(i, i + 16384)));
            frames.push({ ...item.metadata, base64: btoa(parts.join('')) });
            bytes += item.gray.byteLength;
        }
        return { frames, width: record.width, height: record.height, stopped: record.stoppedAtMs !== null };
    }
    function ack(receipts) {
        // Validate the whole prefix before releasing any bytes. Retrying a read is non-destructive.
        if (receipts.some((receipt, index) => receipt.index !== queue[index]?.metadata.index || !receipt.file || !receipt.sha256)) throw new Error('Invalid capture acknowledgment');
        for (const receipt of receipts) {
            const item = queue.shift();
            Object.assign(item.metadata, receipt, { payloadStatus: 'saved' });
            record.frames.push(item.metadata);
            record.counters.queuedBytes -= item.gray.byteLength;
        }
    }
    window.__mobileSLAMInputCapture = { token, record, stop, readBatch, ack };
    // The recorder owns this deliberate lifecycle boundary; earlier replies remain separately identifiable.
    app.resetVIO();
    record.estimatorLifecycle.clientEpochAtStart = app.vio._clientEpoch;
    return { startedAtMs: record.startedAtMs, seconds, maxFrames, maxQueuedBytes, initialRecording: structuredClone(record) };
}

export async function drainCapture(call, token, cameraDirectory, savedFrames) {
    const batch = await call(`(() => { const c=window.__mobileSLAMInputCapture; if(c?.token!==${JSON.stringify(token)}) throw new Error('Capture lost after navigation'); return c.readBatch(); })()`);
    const receipts = [];
    for (const frame of batch.frames) {
        if (!Number.isSafeInteger(frame.index) || frame.index < 0) throw new Error('Invalid capture frame index');
        if (!savedFrames.has(frame.index)) {
            const gray = Buffer.from(frame.base64, 'base64');
            if (gray.length !== batch.width * batch.height) throw new Error('Incorrect grayscale byte count');
            const file = `${String(frame.index).padStart(5, '0')}.pgm`;
            const handle = await fs.open(path.join(cameraDirectory, file), 'wx', 0o600);
            try {
                await handle.writeFile(Buffer.concat([Buffer.from(`P5\n${batch.width} ${batch.height}\n255\n`), gray]));
                await handle.sync();
            } finally { await handle.close(); }
            const { base64, ...metadata } = frame;
            const receipt = { ...metadata, file, sha256: createHash('sha256').update(gray).digest('hex'), payloadStatus: 'saved' };
            const journal = await fs.open(path.join(cameraDirectory, '../received-frames.jsonl'), 'a', 0o600);
            try { await journal.writeFile(JSON.stringify(receipt) + '\n'); await journal.sync(); }
            finally { await journal.close(); }
            savedFrames.set(frame.index, receipt);
        }
        receipts.push(savedFrames.get(frame.index));
    }
    if (receipts.length) await call(`(() => { const c=window.__mobileSLAMInputCapture; if(c?.token!==${JSON.stringify(token)}) throw new Error('Wrong capture'); c.ack(${JSON.stringify(receipts)}); return true; })()`);
    return { frames: batch.frames.length, stopped: batch.stopped };
}


export function captureCompletion(record, savedFrames, { error, cleanupError } = {}) {
    const results = new Set(record.results.map(row => `${row.clientEpoch}:${row.requestId}`));
    const accepted = new Set(record.acceptedFrameMetadata.map(row => `${row.clientEpoch}:${row.requestId}`));
    const pendingResults = record.acceptedFrameMetadata.filter(row => !results.has(`${row.clientEpoch}:${row.requestId}`)).length;
    const partial = Boolean(error || cleanupError || record.stopReason !== 'duration' || !record.counters.acceptedFrames ||
        !record.imu.length || record.counters.acceptedFrames !== savedFrames.size || record.counters.imuOverflow ||
        record.counters.resultOverflow || record.counters.metadataOverflow);
    const imuCoversImages = Boolean(record.imu.length && record.acceptedFrameMetadata.length &&
        record.imu[0][0] <= record.acceptedFrameMetadata[0].timestamp &&
        record.imu.at(-1)[0] >= record.acceptedFrameMetadata.at(-1).timestamp);
    return { savedFrames: savedFrames.size, missingPayloads: record.counters.acceptedFrames - savedFrames.size,
        unmatchedResults: record.results.filter(row => !accepted.has(`${row.clientEpoch}:${row.requestId}`)).length,
        pendingResults, resultsComplete: pendingResults === 0, partial, imuCoversImages,
        replayReady: !partial && imuCoversImages };
}

async function main() {
    const { values } = parseArgs({ options: { cdp: { type: 'string', default: 'http://127.0.0.1:9222' },
        seconds: { type: 'string', default: '60' }, frames: { type: 'string', default: '24' },
        timer: { type: 'boolean', default: false }, output: { type: 'string' },
        'queue-mib': { type: 'string', default: '16' } } });
    const seconds = Number(values.seconds), maxFrames = Number(values.frames), queueMiB = Number(values['queue-mib']);
    if (!Number.isInteger(seconds) || seconds < 1 || seconds > 60 || !Number.isInteger(maxFrames) || maxFrames < 1 || maxFrames > 450) throw new Error('seconds 1..60; frames 1..450 (maximum queued frames, not total capture)');
    // Wireless ADB drains ~14 fps of 640x480 gray; a larger phone-side queue keeps 30 s captures replay-ready.
    if (!Number.isInteger(queueMiB) || queueMiB < 16 || queueMiB > 128) throw new Error('queue-mib 16..128');
    const endpoint = new URL(values.cdp);
    if (!['127.0.0.1', 'localhost', '[::1]'].includes(endpoint.hostname) || endpoint.protocol !== 'http:') throw new Error('Local ADB CDP endpoint required');
    const directory = values.output || `build/dev-android/input-${new Date().toISOString().replace(/[:.]/g, '-')}`;
    await fs.mkdir(directory, { recursive: false });
    const cameraDirectory = path.join(directory, 'mav0/cam0/data'), imuDirectory = path.join(directory, 'mav0/imu0');
    const token = randomUUID(), savedFrames = new Map();
    const report = { schema: 'mobile-slam-android-input-v1', scope: 'Accepted grayscale copies, submitted IMU and numeric Worker outputs; controlled human motion, no independent ground truth',
        completed: false, recoveryToken: token };
    let socket, sequence = 0;
    function call(expression) {
        const id = ++sequence;
        return new Promise((resolve, reject) => {
            const cleanup = () => { clearTimeout(timer); socket.removeEventListener('message', receive); socket.removeEventListener('close', closed); };
            const timer = setTimeout(() => { cleanup(); reject(new Error('CDP timeout')); }, 10000);
            const closed = () => { cleanup(); reject(new Error('CDP connection closed')); };
            function receive(event) {
                const message = JSON.parse(event.data); if (message.id !== id) return;
                cleanup();
                if (message.error || message.result?.exceptionDetails) reject(new Error(JSON.stringify(message.error || message.result.exceptionDetails)));
                else resolve(message.result.result.value);
            }
            socket.addEventListener('message', receive); socket.addEventListener('close', closed);
            try { socket.send(JSON.stringify({ id, method: 'Runtime.evaluate', params: { expression, returnByValue: true, userGesture: false } })); }
            catch (error) { cleanup(); reject(error); }
        });
    }
    try {
        await fs.mkdir(cameraDirectory, { recursive: true }); await fs.mkdir(imuDirectory, { recursive: true });
        const list = await (await fetch(new URL('/json/list', endpoint), { signal: AbortSignal.timeout(8000) })).json();
        const targets = list.filter(target => { try { return target.type === 'page' && new URL(target.url).origin === 'https://dev.serdic.com:7002' && ['/', '/index.html'].includes(new URL(target.url).pathname); } catch { return false; } });
        if (targets.length !== 1) throw new Error('Expected one Mobile-SLAM tab');
        socket = new WebSocket(targets[0].webSocketDebuggerUrl);
        await new Promise((resolve, reject) => {
            const timer = setTimeout(() => reject(new Error('CDP connection timeout')), 8000);
            socket.addEventListener('open', () => { clearTimeout(timer); resolve(); }, { once: true });
            socket.addEventListener('error', () => { clearTimeout(timer); reject(new Error('CDP connection failed')); }, { once: true });
        });
        report.armed = await call(`(${begin.toString()})(${seconds}, ${maxFrames}, ${JSON.stringify(token)}, ${values.timer}, ${copyLowFeatureMetadata.toString()}, ${queueMiB})`);
        report.initialRecording = report.armed.initialRecording; delete report.armed.initialRecording;
        const initial = await fs.open(path.join(directory, 'capture-start.json'), 'wx', 0o600);
        try { await initial.writeFile(JSON.stringify(report.initialRecording, null, 2) + '\n'); await initial.sync(); }
        finally { await initial.close(); }
        console.log(JSON.stringify({ signal: 'ARMED', ...report.armed }));
        const deadline = performance.now() + seconds * 1000 + 20000;
        while (true) {
            if (performance.now() > deadline) throw new Error('Capture drain deadline exceeded');
            const batch = await drainCapture(call, token, cameraDirectory, savedFrames);
            if (batch.stopped && batch.frames === 0) break;
            if (!batch.stopped) await new Promise(resolve => setTimeout(resolve, 100));
        }
    } catch (error) { report.error = error.message; process.exitCode = 1; }
    finally {
        if (socket?.readyState === WebSocket.OPEN) {
            try {
                report.recording = await call(`(() => {const c=window.__mobileSLAMInputCapture;if(c?.token!==${JSON.stringify(token)})return null;c.stop(${JSON.stringify(report.error ? 'host_error' : 'host_complete')});return c.record;})()`);
            } catch (error) { report.cleanupError = error.message; process.exitCode = 1; }
        }
        report.hostSavedFrames = [...savedFrames.values()];
        report.receiptJournal = 'mav0/cam0/received-frames.jsonl';
        report.recoveryPolicy = 'Owned stopped capture retained until host record.json is durable; cleanup.json records removal';
        report.completionMeaning = 'All accepted input images and submitted IMU were captured; result completion is reported separately';
        if (report.recording) {
            const r = report.recording;
            r.completion = captureCompletion(r, savedFrames, report);
            try {
                const saved = r.acceptedFrameMetadata.filter(frame => savedFrames.has(frame.index));
                await fs.writeFile(path.join(cameraDirectory, '../data.csv'), ['#timestamp [ns],filename',
                    ...saved.map(frame => `${Math.round(frame.timestamp * 1e9)},${savedFrames.get(frame.index).file}`)].join('\n') + '\n', { flag: 'wx', mode: 0o600 });
                await fs.writeFile(path.join(imuDirectory, 'data.csv'), ['#timestamp [ns],w_RS_S_x [rad s^-1],w_RS_S_y [rad s^-1],w_RS_S_z [rad s^-1],a_RS_S_x [m s^-2],a_RS_S_y [m s^-2],a_RS_S_z [m s^-2]',
                    ...r.imu.map(row => [Math.round(row[0] * 1e9), ...row.slice(4, 7), ...row.slice(1, 4)].join(','))].join('\n') + '\n', { flag: 'wx', mode: 0o600 });
                report.completed = !r.completion.partial;
            } catch (error) { report.error = error.message; r.completion.partial = true; }
        }
        if (!report.completed) process.exitCode = 1;
        try {
            const recordFile = await fs.open(path.join(directory, 'record.json'), 'wx', 0o600);
            try { await recordFile.writeFile(JSON.stringify(report, null, 2) + '\n'); await recordFile.sync(); }
            finally { await recordFile.close(); }
            const cleanup = { removed: false, recoveryToken: token };
            if (report.completed && socket?.readyState === WebSocket.OPEN) {
                try {
                    cleanup.removed = await call(`(() => {if(window.__mobileSLAMInputCapture?.token!==${JSON.stringify(token)})return false;delete window.__mobileSLAMInputCapture;return true;})()`);
                } catch (error) { cleanup.error = error.message; process.exitCode = 1; }
            }
            await fs.writeFile(path.join(directory, 'cleanup.json'), JSON.stringify(cleanup, null, 2) + '\n', { flag: 'wx', mode: 0o600 });
        } finally { socket?.close(); }
    }
    console.log(JSON.stringify({ output: directory, completed: report.completed, frames: savedFrames.size, error: report.error }));
}

if (process.argv[1] && import.meta.url === pathToFileURL(path.resolve(process.argv[1])).href) {
    main().catch(error => { console.error(error.message); process.exitCode = 1; });
}

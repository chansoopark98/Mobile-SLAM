#!/usr/bin/env node
// Low-overhead Flip6 input recorder. During capture the page only copies each accepted gray frame once and
// transfers it to a Worker that appends it to an OPFS file; IMU rows and a minimal result subset stay in memory.
// Bytes are pulled over CDP only after capture stops, so base64 encoding never competes with live tracking.
// Output keeps the mobile-slam-android-input-v1 layout consumed by replay-android-input.mjs.
import { createHash, randomUUID } from 'node:crypto';
import fs from 'node:fs/promises';
import path from 'node:path';
import { parseArgs } from 'node:util';

const kResultFields = ['engineEpoch', 'initialized', 'statusCode', 'reason', 'featureCount', 'imuCount', 'inputTimestamp',
    'poseTimestamp', 'poseFresh', 'poseValid', 'engineProcessingMs', 'workerProcessingMs', 'imuWaitMs', 'imuDrops',
    'solverIterations'];

export function leanBegin(seconds, token, label, resultFields) {
    const app = window.__mobileSLAM?.app, worker = app?.vio.worker;
    if (!app?.running || !worker) throw new Error('Start Mobile-SLAM before arming input capture');
    if (window.__mobileSLAMInputCapture || window.__mobileSLAMAndroidRecording || window.__mobileSLAMLeanCapture) throw new Error('Finish the existing recording first');
    const frameBytes = app._processWidth * app._processHeight;
    if (!Number.isSafeInteger(frameBytes) || frameBytes <= 0) throw new Error('Invalid grayscale frame');
    const fileName = `lean-${token}.bin`;
    const writerSource = `let handle,count=0,size=0,chain=Promise.resolve();
onmessage=e=>{const m=e.data;chain=chain.then(async()=>{
 if(m.t==='open'){const root=await navigator.storage.getDirectory();const fh=await root.getFileHandle(m.name,{create:true});handle=await fh.createSyncAccessHandle();handle.truncate(0);size=m.size;}
 else if(m.t==='f'){handle.write(new Uint8Array(m.b),{at:m.i*size});count++;}
 else if(m.t==='meta'){const root=await navigator.storage.getDirectory();const mh=await (await root.getFileHandle(m.name,{create:true})).createSyncAccessHandle();const b=new TextEncoder().encode(m.json);mh.truncate(0);mh.write(b,{at:0});mh.flush();mh.close();}
 else if(m.t==='close'){handle.flush();const bytes=handle.getSize();handle.close();postMessage({t:'closed',count,bytes});}
}).catch(error=>postMessage({t:'error',message:String(error)}));};`;
    const writer = new Worker(URL.createObjectURL(new Blob([writerSource], { type: 'text/javascript' })));
    writer.postMessage({ t: 'open', name: fileName, size: frameBytes });
    const record = { startedAtMs: performance.now(), timeOriginMs: performance.timeOrigin,
        width: app._processWidth, height: app._processHeight, config: structuredClone(app._lastConfigParams),
        profile: app.activeConfig, calibration: structuredClone(app.calibrationProvenance),
        imuDiagnostics: app.imu.getDiagnostics(), frames: [], imu: [], results: [],
        capturePolicy: { mode: 'opfs_deferred', recorder: 'lean', resultFields, fileName },
        estimatorLifecycle: { resetRequested: true, clientEpochBeforeReset: app.vio._clientEpoch,
            clientEpochAtStart: null, initializedBeforeReset: app.vio.isInitialized() },
        counters: { acceptedFrames: 0, droppedFrames: 0, invalidPayload: 0, preImuFrames: 0, imuOverflow: 0, resultOverflow: 0 },
        writer: null, writerError: null, stoppedAtMs: null };
    const configuration = JSON.stringify(app._lastConfigParams);
    const frameMethod = app.vio.sendFrame, imuMethod = app.vio.sendIMU;
    let badge = null;
    if (label) {
        // Static badge only; no per-frame DOM work while recording.
        badge = document.createElement('div');
        badge.style.cssText = 'position:fixed;top:48px;left:12px;z-index:10001;padding:6px 10px;background:#111;color:#ff5555;font:15px sans-serif;pointer-events:none;border:1px solid #ff5555;border-radius:4px';
        badge.textContent = `● REC ${label} · ${seconds}s`;
        document.body.appendChild(badge);
    }
    function sendFrame(gray, timestamp) {
        if (app._processWidth !== record.width || app._processHeight !== record.height ||
            JSON.stringify(app._lastConfigParams) !== configuration) {
            stop('configuration_changed'); return frameMethod.apply(this, arguments);
        }
        // After the reset no IMU has been submitted yet; the engine reports imu_missing_past for these frames and
        // replay needs a past IMU bracket, so they are counted but not stored.
        if (!record.imu.length) { record.counters.preImuFrames++; return frameMethod.apply(this, arguments); }
        // Copy before the original method transfers/detaches its input.
        const copy = gray?.byteLength === frameBytes ? gray.slice() : null;
        const accepted = frameMethod.apply(this, arguments);
        if (!accepted) { record.counters.droppedFrames++; return accepted; }
        if (!copy) { record.counters.invalidPayload++; return accepted; }
        const index = record.counters.acceptedFrames++;
        const timing = app.lastFrameTiming;
        record.frames.push({ index, timestamp, requestId: this._activeFrame?.requestId ?? null,
            clientEpoch: this._activeFrame?.clientEpoch ?? null, submittedAtMs: performance.now(),
            sourceTiming: timing ? { source: timing.source, captureTimeMs: timing.captureTimeMs,
                pixelReadStartMs: timing.pixelReadStartMs, pixelReadEndMs: timing.pixelReadEndMs } : null });
        writer.postMessage({ t: 'f', i: index, b: copy.buffer }, [copy.buffer]);
        return accepted;
    }
    function sendIMU(data, count) {
        const copy = data.slice(0, count * 7);
        const accepted = imuMethod.apply(this, arguments);
        if (accepted) for (let index = 0; index < count; index++) {
            if (record.imu.length >= 20000) { record.counters.imuOverflow += count - index; break; }
            record.imu.push(Array.from(copy.subarray(index * 7, index * 7 + 7)));
        }
        return accepted;
    }
    const receive = event => {
        if (event.data.type !== 'result' || !event.data.data) return;
        if (record.results.length >= 10000) { record.counters.resultOverflow++; return; }
        const data = event.data.data, row = { arrivalMs: performance.now(), requestId: event.data.requestId,
            clientEpoch: event.data.clientEpoch, sequence: event.data.sequence };
        for (const key of resultFields) row[key] = data[key];
        row.pose = data.pose ? Array.from(data.pose) : null;
        record.results.push(row);
    };
    const closed = new Promise(resolve => {
        writer.onmessage = event => {
            if (event.data.t === 'error') record.writerError = event.data.message;
            if (event.data.t === 'closed') { record.writer = event.data; resolve(); }
        };
    });
    const stop = (reason = 'duration') => {
        if (record.stoppedAtMs !== null) return;
        if (app.vio.sendFrame === sendFrame) app.vio.sendFrame = frameMethod;
        if (app.vio.sendIMU === sendIMU) app.vio.sendIMU = imuMethod;
        worker.removeEventListener('message', receive); clearTimeout(timer);
        record.stoppedAtMs = performance.now(); record.stopReason = reason;
        record.finalDiagnostics = window.__mobileSLAM.diagnostics();
        // Persist metadata next to the frames so an accidental reload during the pull stays recoverable.
        writer.postMessage({ t: 'meta', name: `${fileName}.json`, json: JSON.stringify(record) });
        writer.postMessage({ t: 'close' });
        if (badge) { badge.style.color = badge.style.borderColor = '#ffcc00'; badge.textContent = '⇪ PC로 전송 중 · 화면 조작/새로고침 금지'; }
    };
    const timer = setTimeout(() => stop('duration'), seconds * 1000);
    async function finished() { await closed; writer.terminate(); return record.writer; }
    async function readChunk(firstIndex, count) {
        const root = await navigator.storage.getDirectory();
        const file = await (await root.getFileHandle(fileName)).getFile();
        const bytes = new Uint8Array(await file.slice(firstIndex * frameBytes, (firstIndex + count) * frameBytes).arrayBuffer());
        const parts = [];
        for (let i = 0; i < bytes.length; i += 16384) parts.push(String.fromCharCode(...bytes.subarray(i, i + 16384)));
        return btoa(parts.join(''));
    }
    async function discard() {
        const root = await navigator.storage.getDirectory();
        await root.removeEntry(fileName);
        await root.removeEntry(`${fileName}.json`).catch(() => {});
        if (badge) badge.remove();
        delete window.__mobileSLAMLeanCapture;
        return true;
    }
    window.__mobileSLAMLeanCapture = { token, record, stop, finished, readChunk, discard };
    app.vio.sendFrame = sendFrame; app.vio.sendIMU = sendIMU; worker.addEventListener('message', receive);
    // Same deliberate lifecycle boundary as android-input-record.mjs so initialization is inside the capture.
    app.resetVIO();
    record.estimatorLifecycle.clientEpochAtStart = app.vio._clientEpoch;
    return { startedAtMs: record.startedAtMs, seconds, frameBytes, fileName };
}

async function main() {
    const { values } = parseArgs({ options: { cdp: { type: 'string', default: 'http://127.0.0.1:9222' },
        seconds: { type: 'string', default: '30' }, label: { type: 'string', default: '' },
        'keep-phone-copy': { type: 'boolean', default: false }, 'with-pipeline': { type: 'boolean', default: false },
        output: { type: 'string' } } });
    const seconds = Number(values.seconds);
    if (!Number.isInteger(seconds) || seconds < 1 || seconds > 120) throw new Error('seconds 1..120');
    if (!/^[\w .-]{0,32}$/.test(values.label)) throw new Error('label: up to 32 word/space/dot/dash characters');
    const endpoint = new URL(values.cdp);
    if (!['127.0.0.1', 'localhost', '[::1]'].includes(endpoint.hostname) || endpoint.protocol !== 'http:') throw new Error('Local ADB CDP endpoint required');
    if (!values.output) throw new Error('--output required');
    const directory = values.output;
    await fs.mkdir(directory, { recursive: false });
    const cameraDirectory = path.join(directory, 'mav0/cam0/data'), imuDirectory = path.join(directory, 'mav0/imu0');
    const token = randomUUID();
    const report = { schema: 'mobile-slam-android-input-v1', recorder: 'android-lean-record',
        scope: 'Accepted grayscale frames (OPFS, pulled after stop), submitted IMU and a minimal Worker result subset; controlled human motion, no independent ground truth',
        completed: false, recoveryToken: token };
    const list = await (await fetch(new URL('/json/list', endpoint))).json();
    const target = list.find(item => item.type === 'page' && /^https:\/\/[^/]+(:\d+)?\/(index\.html)?(\?.*)?$/.test(item.url));
    if (!target) throw new Error('Mobile-SLAM page tab not found');
    const socket = new WebSocket(target.webSocketDebuggerUrl);
    await new Promise((resolve, reject) => { socket.addEventListener('open', resolve, { once: true }); socket.addEventListener('error', reject, { once: true }); });
    let sequence = 0;
    const call = (expression, timeoutMs = 20000) => new Promise((resolve, reject) => {
        const id = ++sequence;
        const timer = setTimeout(() => { socket.removeEventListener('message', receive); reject(new Error('CDP timeout')); }, timeoutMs);
        const receive = event => {
            const message = JSON.parse(event.data);
            if (message.id !== id) return;
            clearTimeout(timer); socket.removeEventListener('message', receive);
            if (message.error) reject(new Error(message.error.message));
            else if (message.result.exceptionDetails) reject(new Error(message.result.exceptionDetails.exception?.description ?? 'page exception'));
            else resolve(message.result.result.value);
        };
        socket.addEventListener('message', receive);
        socket.send(JSON.stringify({ id, method: 'Runtime.evaluate', params: { expression, awaitPromise: true, returnByValue: true } }));
    });
    const capture = `window.__mobileSLAMLeanCapture?.token===${JSON.stringify(token)}?window.__mobileSLAMLeanCapture:(()=>{throw new Error('Capture lost')})()`;
    try {
        report.armed = await call(`(${leanBegin.toString()})(${seconds}, ${JSON.stringify(token)}, ${JSON.stringify(values.label)}, ${JSON.stringify(values['with-pipeline'] ? [...kResultFields, 'pipeline'] : kResultFields)})`);
        console.log(JSON.stringify({ signal: 'ARMED', ...report.armed }));
        // Host stays idle during capture; one tiny poll per second.
        while (!(await call(`(${capture}).record.stoppedAtMs!==null`))) await new Promise(resolve => setTimeout(resolve, 1000));
        report.writer = await call(`(${capture}).finished()`, 60000);
        const recording = await call(`(${capture}).record`, 60000);
        const frameBytes = recording.width * recording.height;
        if (recording.writerError) throw new Error(`OPFS writer: ${recording.writerError}`);
        if (report.writer.count !== recording.frames.length || report.writer.bytes !== frameBytes * recording.frames.length) throw new Error('OPFS frame count/bytes mismatch');
        await fs.mkdir(cameraDirectory, { recursive: true }); await fs.mkdir(imuDirectory, { recursive: true });
        const header = Buffer.from(`P5\n${recording.width} ${recording.height}\n255\n`);
        const kChunk = 12;
        for (let first = 0; first < recording.frames.length; first += kChunk) {
            const count = Math.min(kChunk, recording.frames.length - first);
            const bytes = Buffer.from(await call(`(${capture}).readChunk(${first}, ${count})`, 60000), 'base64');
            if (bytes.length !== count * frameBytes) throw new Error(`Short OPFS read at ${first}`);
            for (let k = 0; k < count; k++) {
                const frame = recording.frames[first + k], gray = bytes.subarray(k * frameBytes, (k + 1) * frameBytes);
                frame.file = `${String(frame.index).padStart(5, '0')}.pgm`;
                frame.sha256 = createHash('sha256').update(gray).digest('hex');
                frame.payloadStatus = 'saved';
                await fs.writeFile(path.join(cameraDirectory, frame.file), Buffer.concat([header, gray]), { flag: 'wx', mode: 0o600 });
            }
        }
        recording.acceptedFrameMetadata = recording.frames.map(frame => ({ ...frame }));
        const imu = recording.imu, frames = recording.frames;
        recording.completion = { savedFrames: frames.length, missingPayloads: recording.counters.invalidPayload,
            partial: recording.stopReason !== 'duration' || !frames.length || !imu.length || recording.counters.imuOverflow > 0 ||
                recording.counters.resultOverflow > 0 || recording.counters.invalidPayload > 0,
            imuCoversImages: Boolean(imu.length && frames.length && imu[0][0] <= frames[0].timestamp && imu.at(-1)[0] >= frames.at(-1).timestamp) };
        // The final frame often has no future IMU sample yet; replay can drop it, so report both.
        recording.completion.imuCoversAllButLast = Boolean(imu.length && frames.length > 1 && imu[0][0] <= frames[0].timestamp && imu.at(-1)[0] >= frames.at(-2).timestamp);
        // Leading/trailing frames outside the IMU span are expected after the reset; lean-replay-view.py drops them.
        recording.completion.framesInsideImuSpan = frames.filter(frame => imu.length && imu[0][0] <= frame.timestamp && frame.timestamp <= imu.at(-1)[0]).length;
        recording.completion.replayReady = !recording.completion.partial && recording.completion.imuCoversImages;
        recording.completion.replayViewNeeded = !recording.completion.partial && !recording.completion.imuCoversImages && recording.completion.framesInsideImuSpan >= 2;
        await fs.writeFile(path.join(cameraDirectory, '../data.csv'), ['#timestamp [ns],filename',
            ...frames.map(frame => `${Math.round(frame.timestamp * 1e9)},${frame.file}`)].join('\n') + '\n', { flag: 'wx', mode: 0o600 });
        await fs.writeFile(path.join(imuDirectory, 'data.csv'), ['#timestamp [ns],w_RS_S_x [rad s^-1],w_RS_S_y [rad s^-1],w_RS_S_z [rad s^-1],a_RS_S_x [m s^-2],a_RS_S_y [m s^-2],a_RS_S_z [m s^-2]',
            ...imu.map(row => [Math.round(row[0] * 1e9), ...row.slice(4, 7), ...row.slice(1, 4)].join(','))].join('\n') + '\n', { flag: 'wx', mode: 0o600 });
        report.recording = recording;
        report.completed = !recording.completion.partial;
    } catch (error) { report.error = error.message; process.exitCode = 1; }
    await fs.writeFile(path.join(directory, 'record.json'), JSON.stringify(report, null, 2) + '\n', { flag: 'wx', mode: 0o600 });
    const cleanup = { removed: false, recoveryToken: token };
    if (report.completed && !values['keep-phone-copy']) {
        try { cleanup.removed = await call(`(${capture}).discard()`); } catch (error) { cleanup.error = error.message; }
    } else cleanup.reason = report.completed ? 'keep-phone-copy' : 'incomplete capture retained for recovery';
    await fs.writeFile(path.join(directory, 'cleanup.json'), JSON.stringify(cleanup, null, 2) + '\n', { flag: 'wx', mode: 0o600 });
    socket.close();
    console.log(JSON.stringify({ output: directory, completed: report.completed, frames: report.recording?.frames.length ?? 0,
        replayReady: report.recording?.completion.replayReady ?? false, error: report.error }));
}

if (import.meta.url === `file://${process.argv[1]}`) await main();

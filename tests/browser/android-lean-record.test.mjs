import test from 'node:test';
import assert from 'node:assert/strict';
import vm from 'node:vm';
import { leanBegin } from '../../scripts/dev/android-lean-record.mjs';

// Page-side contract of the low-overhead recorder: one copy per accepted frame, transferred to the OPFS writer,
// frames before the first IMU are counted but not stored, and stop restores the original hooks.
function fixture({ transfer = true } = {}) {
    let requestId = 0, resets = 0;
    const timers = new Map(), listeners = new Map(), posted = [];
    const originalFrame = function (gray) {
        this._activeFrame = { requestId: ++requestId, clientEpoch: 7 };
        if (transfer) structuredClone(gray, { transfer: [gray.buffer] });
        return true;
    };
    const originalIMU = () => true;
    const app = { running: true, _processWidth: 4, _processHeight: 1, _lastConfigParams: {}, activeConfig: 'mobile_default',
        calibrationProvenance: {}, imu: { getDiagnostics: () => ({}) }, lastFrameTiming: { source: 'video', captureTimeMs: 1 },
        resetVIO() { resets++; this.vio._clientEpoch++; },
        vio: { sendFrame: originalFrame, sendIMU: originalIMU, _clientEpoch: 7, isInitialized: () => false,
            worker: { addEventListener: (name, fn) => listeners.set(name, fn), removeEventListener: (name, fn) => { if (listeners.get(name) === fn) listeners.delete(name); } } } };
    class FakeWorker { constructor() { this.onmessage = null; } postMessage(message, transferList) {
        posted.push({ message, transferred: transferList?.length ?? 0, bytes: message.b ? new Uint8Array(message.b.slice(0)) : null });
        if (message.t === 'close') queueMicrotask(() => this.onmessage?.({ data: { t: 'closed', count: posted.filter(p => p.message.t === 'f').length, bytes: 0 } }));
    } terminate() {} }
    const context = vm.createContext({ Uint8Array, structuredClone, JSON, Array, Promise, queueMicrotask, Blob: class {},
        URL: { createObjectURL: () => 'blob:x' }, Worker: FakeWorker, performance: { now: () => 0, timeOrigin: 0 },
        setTimeout: fn => { const id = timers.size + 1; timers.set(id, fn); return id; }, clearTimeout: id => timers.delete(id),
        document: { createElement: () => ({ style: {}, remove() {} }), body: { appendChild() {} } },
        window: { __mobileSLAM: { app, diagnostics: () => ({}) } } });
    context.leanBegin = leanBegin;
    vm.runInContext(`(${leanBegin.toString()})(30, 'tok', '', ['statusCode'])`, context);
    return { app, capture: context.window.__mobileSLAMLeanCapture, posted, listeners, originalFrame, originalIMU, get resets() { return resets; } };
}

test('frames before the first IMU are not stored; later frames are copied before transfer', () => {
    const { app, capture, posted } = fixture();
    assert.equal(app.vio.sendFrame(new Uint8Array([9, 9, 9, 9]), 0.1), true);
    app.vio.sendIMU(new Float64Array(7).fill(1), 1);
    const gray = new Uint8Array([1, 2, 3, 4]);
    assert.equal(app.vio.sendFrame(gray, 0.2), true);
    assert.equal(gray.byteLength, 0, 'original method transferred its own input');
    const frames = posted.filter(p => p.message.t === 'f');
    assert.equal(capture.record.counters.preImuFrames, 1);
    assert.equal(frames.length, 1);
    assert.deepEqual([...frames[0].bytes], [1, 2, 3, 4]);
    assert.equal(frames[0].transferred, 1, 'copy is transferred, not cloned');
    assert.equal(JSON.stringify(capture.record.frames.map(f => [f.index, f.timestamp, f.requestId])), '[[0,0.2,2]]');
});

test('reset happens at arm time and stop restores hooks and persists metadata', async () => {
    const harness = fixture();
    const { app, capture, posted, listeners } = harness;
    assert.equal(harness.resets, 1);
    assert.equal(capture.record.estimatorLifecycle.clientEpochAtStart, 8);
    listeners.get('message')({ data: { type: 'result', requestId: 1, clientEpoch: 8, data: { statusCode: 2, pose: [1], pipeline: { x: 1 } } } });
    assert.deepEqual([...Object.keys(capture.record.results[0])].sort(), ['arrivalMs', 'clientEpoch', 'pose', 'requestId', 'sequence', 'statusCode']);
    capture.stop();
    assert.equal(app.vio.sendFrame, harness.originalFrame);
    assert.equal(app.vio.sendIMU, harness.originalIMU);
    assert.equal(listeners.size, 0);
    const kinds = posted.map(p => p.message.t);
    assert.deepEqual([...kinds.slice(-2)], ['meta', 'close']);
    assert.equal(JSON.parse(posted.at(-2).message.json).stopReason, 'duration');
    assert.equal((await capture.finished()).t, 'closed');
});

test('a second capture is refused while one is armed', () => {
    const { app } = fixture();
    assert.throws(() => vm.runInContext(`(${leanBegin.toString()})(30, 'tok2', '', [])`, vm.createContext({ window: { __mobileSLAM: { app }, __mobileSLAMLeanCapture: {} } })), /Finish the existing recording first/);
});

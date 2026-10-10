import test from 'node:test';
import assert from 'node:assert/strict';
import fs from 'node:fs/promises';
import vm from 'node:vm';

const source = await fs.readFile(new URL('../../scripts/dev/android-input-record.mjs', import.meta.url), 'utf8');
const start = source.indexOf('function begin(');
const end = source.indexOf('\nexport async function drainCapture', start) >= 0
    ? source.indexOf('\nexport async function drainCapture', start) : source.indexOf('\ntry {', start);
const beginSource = source.slice(start, end);
function fixture({ transfer = false, frameBytes = 4 } = {}) {
    let now = 0, requestId = 0, resets = 0;
    const timers = new Map(), listeners = new Map();
    const originalFrame = function (gray) {
        this._activeFrame = { requestId: ++requestId, clientEpoch: 7 };
        if (transfer) structuredClone(gray, { transfer: [gray.buffer] });
        return true;
    };
    const originalIMU = () => true;
    const app = { running: true, _processWidth: frameBytes, _processHeight: 1,
        _lastConfigParams: {}, activeConfig: 'mobile_default', calibrationProvenance: {},
        imu: { getDiagnostics: () => ({}) }, resetVIO() { resets++; },
        vio: { sendFrame: originalFrame, sendIMU: originalIMU, _clientEpoch: 7, isInitialized: () => false,
            worker: { addEventListener: (name, fn) => listeners.set(name, fn),
                removeEventListener: (name, fn) => { if (listeners.get(name) === fn) listeners.delete(name); } } } };
    const context = vm.createContext({ Uint8Array, structuredClone, Math, Number, Object,
        btoa: value => Buffer.from(value, 'binary').toString('base64'),
        performance: { now: () => now, timeOrigin: 10000 },
        setTimeout: fn => { const id = timers.size + 1; timers.set(id, fn); return id; },
        clearTimeout: id => timers.delete(id), setInterval: () => 1, clearInterval() {},
        window: { __mobileSLAM: { app, diagnostics: () => ({}) } } });
    vm.runInContext(beginSource, context);
    context.begin(60, 2, 'owned-token', false);
    return { context, app, capture: context.window.__mobileSLAMInputCapture, originalFrame, originalIMU,
        listeners, timers, get resets() { return resets; }, setNow: value => { now = value; } };
}

test('accepted frames after 300 remain recordable when batches are acknowledged', () => {
    const { app, capture } = fixture();
    for (let index = 0; index < 350; index++) {
        app.vio.sendFrame(new Uint8Array([index % 256, 2, 3, 4]), index / 10);
        const batch = capture.readBatch();
        assert.equal(batch.frames.length, 1);
        capture.ack([{ index, file: `${index}.pgm`, sha256: `digest-${index}` }]);
    }
    assert.equal(capture.record.frames.length, 350);
    assert.equal(capture.record.acceptedFrameMetadata.length, 350);
    assert.equal(capture.record.counters.uncopiedFrames, 0);
    assert.equal(capture.record.counters.queuedBytes, 0);
});

test('delayed drain bounds phone bytes and keeps metadata for missing payloads', () => {
    const { app, capture } = fixture();
    for (let i = 0; i < 5; i++) app.vio.sendFrame(new Uint8Array(4), i);
    assert.equal(capture.record.counters.queuedBytes, 8);
    assert.equal(capture.record.counters.queueOverflow, 3);
    assert.equal(capture.record.acceptedFrameMetadata.length, 5);
    assert.deepEqual(Array.from(capture.record.acceptedFrameMetadata.slice(2), frame => frame.payloadStatus),
        ['queue_overflow', 'queue_overflow', 'queue_overflow']);
    assert.equal(capture.readBatch().frames.length, 2);
    assert.equal(capture.readBatch().frames.length, 2, 'reading never removes unacknowledged bytes');
});

test('grayscale snapshot precedes a transferring frame submission', () => {
    const { app, capture } = fixture({ transfer: true });
    const input = new Uint8Array([7, 6, 5, 4]);
    app.vio.sendFrame(input, 1);
    assert.equal(input.byteLength, 0);
    assert.deepEqual(Buffer.from(capture.readBatch().frames[0].base64, 'base64'), Buffer.from([7, 6, 5, 4]));
});

test('recording resets its estimator once and cleanup restores only owned hooks', () => {
    const f = fixture();
    assert.equal(f.resets, 1);
    assert.equal(f.capture.record.estimatorLifecycle.resetRequested, true);
    const otherHook = () => false;
    f.app.vio.sendFrame = otherHook;
    f.capture.stop('host_error');
    assert.equal(f.app.vio.sendFrame, otherHook);
    assert.equal(f.app.vio.sendIMU, f.originalIMU);
    assert.equal(f.listeners.size, 0);
    assert.equal(f.capture.record.stopReason, 'host_error');
});

test('timer expiry stops hooks and retains unacknowledged recoverable bytes', () => {
    const f = fixture(); f.app.vio.sendFrame(new Uint8Array([1, 2, 3, 4]), 1);
    f.setNow(60000);
    [...f.timers.values()][0]();
    assert.equal(f.app.vio.sendFrame, f.originalFrame);
    assert.equal(f.app.vio.sendIMU, f.originalIMU);
    assert.equal(f.listeners.size, 0);
    assert.equal(f.capture.readBatch().stopped, true);
    assert.equal(f.capture.readBatch().frames.length, 1);
});

test('configuration changes end the capture before mixed-calibration images are recorded', () => {
    const f = fixture();
    f.app.vio.sendFrame(new Uint8Array(4), 1);
    f.app._lastConfigParams = { fx: 123 };
    f.app.vio.sendFrame(new Uint8Array(4), 2);
    assert.equal(f.capture.record.stopReason, 'configuration_changed');
    assert.equal(f.capture.record.acceptedFrameMetadata.length, 1);
    assert.equal(f.app.vio.sendFrame, f.originalFrame);
    assert.equal(f.app.vio.sendIMU, f.originalIMU);
});

test('invalid acknowledgments cannot partially release a queue', () => {
    const { app, capture } = fixture();
    app.vio.sendFrame(new Uint8Array(4), 1); app.vio.sendFrame(new Uint8Array(4), 2);
    assert.throws(() => capture.ack([{ index: 0, file: '0.pgm', sha256: 'hash' },
        { index: 99, file: '1.pgm', sha256: 'hash' }]), /Invalid capture acknowledgment/);
    assert.equal(capture.record.counters.queuedBytes, 8);
    assert.equal(capture.record.frames.length, 0);
});

const { drainCapture, captureCompletion } = await import('../../scripts/dev/android-input-record.mjs');
const { tmpdir } = await import('node:os');
const { default: path } = await import('node:path');
const { createHash } = await import('node:crypto');
async function hostFixture(t) {
    const root = await fs.mkdtemp(path.join(tmpdir(), 'mobile-slam-recorder-'));
    t.after(() => fs.rm(root, { recursive: true, force: true }));
    const cameraDirectory = path.join(root, 'mav0/cam0/data');
    await fs.mkdir(cameraDirectory, { recursive: true });
    return { ...fixture(), cameraDirectory };
}

test('host image and metadata receipt exist before the phone releases bytes', async t => {
    const f = await hostFixture(t), saved = new Map();
    f.app.vio.sendFrame(new Uint8Array([1, 2, 3, 4]), 12.25);
    let acknowledged = false;
    const call = async expression => {
        if (expression.includes('c.ack(')) {
            const bytes = await fs.readFile(path.join(f.cameraDirectory, '00000.pgm'));
            assert.deepEqual(bytes.subarray(-4), Buffer.from([1, 2, 3, 4]));
            const receipt = JSON.parse(await fs.readFile(path.join(f.cameraDirectory, '../received-frames.jsonl'), 'utf8'));
            assert.equal(receipt.timestamp, 12.25);
            assert.equal(receipt.sha256, createHash('sha256').update(Buffer.from([1, 2, 3, 4])).digest('hex'));
            assert.equal(f.capture.record.counters.queuedBytes, 4);
            acknowledged = true;
        }
        return vm.runInContext(expression, f.context);
    };
    await drainCapture(call, 'owned-token', f.cameraDirectory, saved);
    assert.equal(acknowledged, true);
    assert.equal(f.capture.record.counters.queuedBytes, 0);
    assert.equal(f.capture.record.frames[0].timestamp, 12.25);
});

test('host filesystem failure keeps payload queued and does not overwrite existing files', async t => {
    const f = await hostFixture(t);
    const existing = path.join(f.cameraDirectory, '00000.pgm');
    await fs.writeFile(existing, 'preserve');
    f.app.vio.sendFrame(new Uint8Array(4), 1);
    await assert.rejects(drainCapture(async expression => vm.runInContext(expression, f.context),
        'owned-token', f.cameraDirectory, new Map()), /EEXIST/);
    assert.equal(await fs.readFile(existing, 'utf8'), 'preserve');
    assert.equal(f.capture.record.counters.queuedBytes, 4);
    assert.equal(f.capture.record.frames.length, 0);
});

test('retry after a lost acknowledgment request reuses durable receipts without duplicate files', async t => {
    const f = await hostFixture(t), saved = new Map();
    f.app.vio.sendFrame(new Uint8Array(4), 1);
    await assert.rejects(drainCapture(async expression => {
        if (expression.includes('c.ack(')) throw new Error('CDP timeout');
        return vm.runInContext(expression, f.context);
    }, 'owned-token', f.cameraDirectory, saved), /CDP timeout/);
    assert.equal(f.capture.record.counters.queuedBytes, 4);
    await drainCapture(async expression => vm.runInContext(expression, f.context), 'owned-token', f.cameraDirectory, saved);
    assert.equal(f.capture.record.counters.queuedBytes, 0);
    const lines = (await fs.readFile(path.join(f.cameraDirectory, '../received-frames.jsonl'), 'utf8')).trim().split('\n');
    assert.equal(lines.length, 1);
});


test('oversized grayscale never exceeds the phone byte limit', () => {
    const { app, capture } = fixture();
    app.vio.sendFrame(new Uint8Array(100), 1);
    assert.equal(capture.record.counters.queuedBytes, 0);
    assert.equal(capture.record.counters.invalidPayload, 1);
    assert.equal(capture.record.acceptedFrameMetadata[0].payloadStatus, 'invalid_payload');
});

test('completion separates foreign replies, pending result, missing IMU, and future bracket', () => {
    const { app, capture } = fixture();
    app.vio.sendFrame(new Uint8Array(4), 1);
    capture.ack([{ index: 0, file: '00000.pgm', sha256: 'hash' }]);
    capture.record.results.push({ clientEpoch: 6, requestId: 1 });
    capture.stop('duration');
    const saved = new Map([[0, {}]]);
    let completion = captureCompletion(capture.record, saved);
    assert.equal(completion.partial, true, 'no submitted IMU cannot be a complete input capture');
    assert.equal(completion.unmatchedResults, 1);
    assert.equal(completion.pendingResults, 1);
    assert.equal(completion.resultsComplete, false);
    capture.record.imu.push([0.9]);
    completion = captureCompletion(capture.record, saved);
    assert.equal(completion.partial, false);
    assert.equal(completion.replayReady, false, 'end image has no future bracket');
    capture.record.imu.push([1.1]);
    assert.equal(captureCompletion(capture.record, saved).replayReady, true);
    assert.equal(captureCompletion(capture.record, saved, { error: 'disk error' }).partial, true);
});

test('wrong token cannot acknowledge or remove another recording', async t => {
    const f = await hostFixture(t);
    f.app.vio.sendFrame(new Uint8Array(4), 1);
    await assert.rejects(drainCapture(async expression => vm.runInContext(expression, f.context),
        'unrelated-token', f.cameraDirectory, new Map()), /Capture lost after navigation/);
    assert.equal(f.capture.record.counters.queuedBytes, 4);
    assert.equal(f.context.window.__mobileSLAMInputCapture.token, 'owned-token');
});

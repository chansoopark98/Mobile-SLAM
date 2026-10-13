import test from 'node:test';
import assert from 'node:assert/strict';
import fs from 'node:fs/promises';
import vm from 'node:vm';
import { spawnSync } from 'node:child_process';

const source = await fs.readFile(new URL('../../scripts/dev/android-snapshot.mjs', import.meta.url), 'utf8');
const start = source.indexOf('function startRecording(');
const end = source.indexOf('\nasync function finishRecording(', start);
const beginSource = source.slice(start, end);

function fixture() {
    let now = 1000, nextRequestId = 0;
    const listeners = new Map(), timeouts = new Map(), intervals = new Map(), children = [];
    const frameMethod = function (image, timestamp) {
        this._activeFrame = { requestId: ++nextRequestId, clientEpoch: 2, sequence: nextRequestId,
            sentAtMs: 9000 + now, frameCopyMs: 0.5 };
        return true;
    };
    const imuMethod = () => true;
    const consumeMethod = function (result) { this.lastConsumed = result; };
    const app = { running: true, _lastConfigParams: { width: 2, height: 2 }, calibrationProvenance: {},
        lastFrameTiming: { timestampS: 0.9, presentedFrames: 7 },
        camera: { getVideoTrack: () => ({ getSettings: () => ({}), getCapabilities: () => ({}) }), getRotateMode: () => 'none' },
        _processWidth: 2, _processHeight: 2,
        imu: { getGyroBias: () => ({}), getDiagnostics: () => ({}), getSensorType: () => 'motion' },
        vio: { worker: { addEventListener: (name, fn) => listeners.set(name, fn),
            removeEventListener: (name, fn) => { if (listeners.get(name) === fn) listeners.delete(name); } },
            sendFrame: frameMethod, sendIMU: imuMethod }, _consumeResult: consumeMethod };
    const context = vm.createContext({ window: { __mobileSLAM: { app } }, document: {
        createElement: () => ({ style: {}, textContent: '', remove() { children.splice(children.indexOf(this), 1); } }),
        body: { appendChild: node => children.push(node) } },
        performance: { now: () => now, timeOrigin: 9000 }, structuredClone, Object, Number, Array, Map,
        setTimeout: fn => { const id = timeouts.size + 1; timeouts.set(id, fn); return id; },
        clearTimeout: id => timeouts.delete(id),
        setInterval: fn => { const id = intervals.size + 1; intervals.set(id, fn); return id; },
        clearInterval: id => intervals.delete(id) });
    vm.runInContext(beginSource + '\nglobalThis.begin = startRecording;', context);
    return { context, app, children, listeners, timeouts, intervals, frameMethod, imuMethod, consumeMethod,
        setNow: value => { now = value; for (const fn of intervals.values()) fn(); },
        result: (request, fields = {}) => listeners.get('message')({ data: { type: 'result', ...request,
            data: { inputTimestamp: 0.9, poseTimestamp: 0.9, pipeline: { frontendMs: 4 }, ...fields } } }) };
}

test('Timer uses the recording start clock and restores UI and hooks on stop', () => {
    const f = fixture(); f.context.begin(30, 'token', true);
    const capture = f.context.window.__mobileSLAMAndroidRecording;
    assert.equal(f.children.length, 1);
    assert.match(f.children[0].textContent, /정지/);
    f.setNow(11000); assert.match(f.children[0].textContent, /이동/);
    f.setNow(21000); assert.match(f.children[0].textContent, /다시 정지/);
    capture.stop('host_error');
    assert.equal(f.children.length, 0);
    assert.equal(f.intervals.size, 0);
    assert.equal(f.listeners.size, 0);
    assert.equal(f.app.vio.sendFrame, f.frameMethod);
    assert.equal(f.app.vio.sendIMU, f.imuMethod);
    assert.equal(f.app._consumeResult, f.consumeMethod);
    assert.equal(capture.record.stopReason, 'host_error');
});

test('Result metadata is joined to its submitted frame, never the later camera snapshot', () => {
    const f = fixture(); f.context.begin(30, 'token', false);
    f.app.vio.sendFrame(new Uint8Array(4), 0.9);
    const request = { requestId: 1, clientEpoch: 2, sequence: 1 };
    f.app.lastFrameTiming = { timestampS: 1.5, presentedFrames: 8 };
    f.setNow(1050); f.result(request);
    const row = f.context.window.__mobileSLAMAndroidRecording.record.results[0];
    assert.equal(row.sourceTiming.presentedFrames, 7);
    assert.equal(row.sentAtMs, 1000);
    assert.equal(row.frameCopyMs, 0.5);
    assert.equal(row.roundtripMs, 50);
    assert.equal(row.pipeline.frontendMs, 4);
    assert.equal(row.uiStateUpdatedAtMs, null);
    f.setNow(1070); f.app._consumeResult({ ...request });
    assert.equal(row.uiStateUpdatedAtMs, 1070);
    f.context.window.__mobileSLAMAndroidRecording.stop();
});

test('Unmatched result stays unpaired and recorder limits remain enforced', () => {
    const f = fixture(); f.context.begin(30, 'token', false);
    f.result({ requestId: 999, clientEpoch: 2, sequence: 999 });
    const capture = f.context.window.__mobileSLAMAndroidRecording;
    assert.equal(capture.record.results[0].sourceTiming, null);
    assert.equal(capture.record.results[0].uiStateUpdatedAtMs, null);
    capture.record.results.length = 1500;
    f.result({ requestId: 1000, clientEpoch: 2, sequence: 1000 });
    assert.equal(capture.record.dropped.results, 1);
    capture.record.imu.length = 12000;
    f.app.vio.sendIMU(new Float64Array(14), 2);
    assert.equal(capture.record.dropped.imu, 2);
    capture.stop();
});

test('A source timestamp from another Worker epoch is never attached to the result', () => {
    const f = fixture(); f.context.begin(30, 'token', false);
    f.app.lastFrameTiming = { timestampS: 0.9, clientEpoch: 99, presentedFrames: 7 };
    f.app.vio.sendFrame(new Uint8Array(4), 0.9);
    f.result({ requestId: 1, clientEpoch: 2, sequence: 1 });
    assert.equal(f.context.window.__mobileSLAMAndroidRecording.record.results[0].sourceTiming, null);
    f.context.window.__mobileSLAMAndroidRecording.stop();
});

test('Setup error and later hook ownership restore only the recorder hooks', () => {
    const failed = fixture();
    failed.context.document.createElement = () => { throw new Error('overlay unavailable'); };
    assert.throws(() => failed.context.begin(30, 'token', true), /overlay unavailable/);
    assert.equal(failed.app.vio.sendFrame, failed.frameMethod);
    assert.equal(failed.app.vio.sendIMU, failed.imuMethod);
    assert.equal(failed.app._consumeResult, failed.consumeMethod);
    assert.equal(failed.listeners.size, 0);
    assert.equal(failed.timeouts.size, 0);

    const replaced = fixture(); replaced.context.begin(30, 'token', false);
    const otherHook = () => false;
    replaced.app.vio.sendFrame = otherHook;
    replaced.app._consumeResult = otherHook;
    replaced.context.window.__mobileSLAMAndroidRecording.stop();
    assert.equal(replaced.app.vio.sendFrame, otherHook);
    assert.equal(replaced.app._consumeResult, otherHook);
    assert.equal(replaced.app.vio.sendIMU, replaced.imuMethod);
});

test('Optional timer requires an exact 30-second recording before any CDP access', () => {
    const run = spawnSync(process.execPath, ['scripts/dev/android-snapshot.mjs', '--timer', '--record-seconds', '29'],
        { cwd: new URL('../..', import.meta.url), encoding: 'utf8', timeout: 5000 });
    assert.notEqual(run.status, 0);
    assert.match(run.stderr, /--timer requires --record-seconds 30/);
});

import test from 'node:test';
import assert from 'node:assert/strict';
import fs from 'node:fs/promises';
import { Worker } from 'node:worker_threads';

const source = await fs.readFile(new URL('../../web/js/vio-worker.js', import.meta.url), 'utf8');
const asModule = text => 'data:text/javascript;base64,' + Buffer.from(text).toString('base64');
function fixture({ api = true, getter = true, mismatch = false } = {}) {
    const factory = asModule(`export default function () {
        const memory = new ArrayBuffer(1024 * 1024); let offset = 8;
        const module = { HEAPU8: new Uint8Array(memory), HEAPF64: new Float64Array(memory),
            _malloc(size) { const pointer = offset; offset += Math.ceil(size / 8) * 8; return pointer; }, _free() {},
            VIOEngine: class {
                constructor() { this.rotation = true; this.epoch = 0; this.timestamp = -1; }
                configure() { this.epoch++; this.timestamp = -1; return true; }
                ${api ? `setRotationRescue(value) { this.rotation = value; }
                ${getter ? `getRotationRescueEnabled() { return ${mismatch ? 'false' : 'this.rotation'}; }` : ''}` : ''}
                getEpoch() { return this.epoch; } getFrameTimestamp() { return this.timestamp; }
                isInitialized() { return this.timestamp >= 0; } getStatusCode() { return this.timestamp >= 0 ? 2 : 1; }
                getFeaturePointCount() { return 0; } getLastReason() { return this.timestamp >= 0 ? 'tracking' : 'initializing'; }
                getMapPoints() { return 0; }
                processFrame(image, width, height, imu, count, timestamp, pose) {
                    this.timestamp = timestamp;
                    module.HEAPF64.set([1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1], pose / 8);
                    return true;
                }
                getPoseValid() { return this.timestamp >= 0; } getPoseFresh() { return this.timestamp >= 0; }
                getPoseTimestamp() { return this.timestamp; }
                reset() { this.epoch++; this.timestamp = -1; } delete() {}
            }
        };
        return module;
    }`);
    const bootstrap = asModule(`import { parentPort } from 'node:worker_threads';
        globalThis.self = { postMessage(message) { parentPort.postMessage(message); } };
        await import(${JSON.stringify(asModule(source))});
        parentPort.on('message', message => self.onmessage({ data: message }));`);
    const worker = new Worker(new URL(bootstrap), { type: 'module' });
    const pending = new Map(); let sequence = 0, clientEpoch = 1;
    worker.on('message', message => {
        const entry = pending.get(message.requestId);
        if (entry && message.type !== 'wasm_log') {
            clearTimeout(entry.timer); pending.delete(message.requestId); entry.resolve(message);
        }
    });
    const call = (type, data) => new Promise((resolve, reject) => {
        if (type === 'configure' || type === 'reset') clientEpoch++;
        const requestId = ++sequence;
        const timer = setTimeout(() => { pending.delete(requestId); reject(new Error('Rotation mode fixture timeout')); }, 3000);
        pending.set(requestId, { resolve, timer });
        worker.postMessage({ type, data, sequence, requestId, clientEpoch });
    });
    const configure = rotationRescue => call('configure', { width: 2, height: 2, fx: 1, fy: 1, cx: 1, cy: 1,
        ...(rotationRescue === undefined ? {} : { rotationRescue }) });
    const frame = async timestamp => {
        await call('imu', { imuData: new Float64Array([timestamp-.01,0,0,9.81,0,0,0,timestamp+.01,0,0,9.81,0,0,0]).buffer, count: 2 });
        return call('frame', { gray: new ArrayBuffer(4), timestamp });
    };
    return { worker, call, configure, frame, initialize: () => call('init', { wasmPath: factory }) };
}

test('Actual Worker defaults rotation rescue off and accepts only an explicit boolean true', async t => {
    const f = fixture(); t.after(() => f.worker.terminate()); await f.initialize();
    for (const requested of [undefined, false, 'true', 1, true]) {
        const configured = await f.configure(requested);
        assert.equal(configured.success, true, configured.error);
        assert.equal(configured.rotationRescueEnabled, requested === true);
        const result = await f.frame(1);
        assert.equal(result.data.rotationRescueEnabled, requested === true);
    }
});

test('Actual Worker reconfiguration disables a stale enabled rotation rescue mode', async t => {
    const f = fixture(); t.after(() => f.worker.terminate()); await f.initialize();
    const enabled = await f.configure(true); assert.equal(enabled.rotationRescueEnabled, true);
    const before = await f.frame(1); assert.equal(before.data.poseFresh, true);
    const defaults = await f.configure(); assert.equal(defaults.rotationRescueEnabled, false);
    const after = await f.frame(2);
    assert.equal(after.data.rotationRescueEnabled, false);
    assert.equal(after.data.engineEpoch, before.data.engineEpoch + 1);
    assert.equal(after.data.poseTimestamp, 2); assert.equal(after.data.poseFresh, true);
});

test('Actual Worker reset preserves explicit rescue choice while invalidating the old pose epoch', async t => {
    const f = fixture(); t.after(() => f.worker.terminate()); await f.initialize(); await f.configure(true);
    const before = await f.frame(1); await f.call('reset');
    const after = await f.frame(2);
    assert.equal(after.data.rotationRescueEnabled, true);
    assert.equal(after.data.engineEpoch, before.data.engineEpoch + 1);
    assert.equal(after.data.poseTimestamp, 2); assert.equal(after.data.poseFresh, true);
});

test('Actual Worker accepts old engines with default rescue off and rejects explicit unsupported rescue', async t => {
    const f = fixture({ api: false }); t.after(() => f.worker.terminate()); await f.initialize();
    const defaults = await f.configure(); assert.equal(defaults.success, true, defaults.error);
    assert.equal(defaults.rotationRescueEnabled, null, 'missing getter cannot prove actual engine mode');
    const unsupported = await f.configure(true);
    assert.equal(unsupported.success, false); assert.match(unsupported.error, /rotation.*unavailable/i);
    const imu = await f.call('imu', { imuData: new Float64Array([1,0,0,9.81,0,0,0]).buffer, count: 1 });
    assert.equal(imu.success, false); assert.match(imu.error, /not configured/i);
});

test('Actual Worker rejects a getter that does not confirm requested rescue mode', async t => {
    const f = fixture({ mismatch: true }); t.after(() => f.worker.terminate()); await f.initialize();
    const reply = await f.configure(true);
    assert.equal(reply.success, false); assert.match(reply.error, /rotation.*not applied/i);
});

test('Actual Worker fails closed when rescue setter exists but requested mode cannot be read back', async t => {
    const f = fixture({ getter: false }); t.after(() => f.worker.terminate()); await f.initialize();
    const defaults = await f.configure(); assert.equal(defaults.success, true, defaults.error);
    assert.equal(defaults.rotationRescueEnabled, null);
    const requested = await f.configure(true);
    assert.equal(requested.success, false); assert.match(requested.error, /rotation.*unavailable/i);
    const frame = await f.call('frame', { gray: new ArrayBuffer(4), timestamp: 1 });
    assert.equal(frame.data.poseFresh, false); assert.equal(frame.data.reason, 'not_configured');
});

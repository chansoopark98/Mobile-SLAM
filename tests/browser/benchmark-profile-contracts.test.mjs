import test from 'node:test';
import assert from 'node:assert/strict';
import fs from 'node:fs/promises';
import vm from 'node:vm';
import { Worker } from 'node:worker_threads';

const workerSource = await fs.readFile(new URL('../../web/js/vio-worker.js', import.meta.url), 'utf8');
const replaySource = await fs.readFile(new URL('../../web/js/test-tumvi-app.js', import.meta.url), 'utf8');
const asModule = source => 'data:text/javascript;base64,' + Buffer.from(source).toString('base64');
const request = (type, data, sequence, clientEpoch) => ({ type, data, sequence, requestId: sequence, clientEpoch });

function actualWorker({ api = true, getter = true, mismatch = false } = {}) {
    const factory = asModule(`export default function () {
        const memory = new ArrayBuffer(1024 * 1024); let offset = 8;
        return { HEAPU8: new Uint8Array(memory), HEAPF64: new Float64Array(memory),
            _malloc(size) { const p = offset; offset += Math.ceil(size / 8) * 8; return p; }, _free() {},
            VIOEngine: class {
                constructor() { this.profile = true; }
                configure() { return true; }
                ${api ? `setBenchmarkSolverProfile(value) { this.profile = value; }
                ${getter ? `getBenchmarkSolverProfile() { return ${mismatch ? 'false' : 'this.profile'}; }` : ''}` : ''}
                getEpoch() { return 1; } isInitialized() { return false; }
                getStatusCode() { return 1; } getFeaturePointCount() { return 0; }
                getLastReason() { return 'initializing'; } getMapPoints() { return 0; }
                processFrame() { return false; } getPoseValid() { return false; } getPoseFresh() { return false; }
                getPoseTimestamp() { return -1; } reset() {} delete() {}
            }
        };
    }`);
    const bootstrap = asModule(`import { parentPort } from 'node:worker_threads';
        globalThis.self = { postMessage(value) { parentPort.postMessage(value); } };
        await import(${JSON.stringify(asModule(workerSource))});
        parentPort.on('message', value => self.onmessage({ data: value }));`);
    const worker = new Worker(new URL(bootstrap), { type: 'module' });
    const pending = new Map();
    worker.on('message', value => {
        const target = pending.get(value.requestId);
        if (target && value.type !== 'wasm_log') { clearTimeout(target.timer); pending.delete(value.requestId); target.resolve(value); }
    });
    const call = value => new Promise((resolve, reject) => {
        const timer = setTimeout(() => { pending.delete(value.requestId); reject(new Error('Worker profile fixture timeout')); }, 3000);
        pending.set(value.requestId, { resolve, timer }); worker.postMessage(value);
    });
    return { worker, call, factory };
}

test('Actual Worker explicitly clears a persisted benchmark mode for default and nonboolean requests', async () => {
    const { worker, call, factory } = actualWorker();
    try {
        await call(request('init', { wasmPath: factory }, 1, 1));
        const config = { width: 2, height: 2, fx: 1, fy: 1, cx: 1, cy: 1 };
        const enabled = await call(request('configure', { ...config, benchmark_zero_positive_tolerances: true }, 2, 2));
        assert.equal(enabled.success, true); assert.equal(enabled.benchmarkSolverProfile, true);
        const defaults = await call(request('configure', config, 3, 3));
        assert.equal(defaults.success, true); assert.equal(defaults.benchmarkSolverProfile, false);
        const nonboolean = await call(request('configure', { ...config, benchmark_zero_positive_tolerances: 'true' }, 4, 4));
        assert.equal(nonboolean.benchmarkSolverProfile, false);
        await call(request('imu', { imuData: new Float64Array([1,0,0,9.81,0,0,0,1.01,0,0,9.81,0,0,0]).buffer, count: 2 }, 5, 4));
        const result = await call(request('frame', { gray: new ArrayBuffer(4), timestamp: 1 }, 6, 4));
        assert.equal(result.data.benchmarkSolverProfile, false); assert.equal(result.data.solverProfile, 'default');
    } finally { await worker.terminate(); }
});

for (const missing of [{ api: false }, { getter: false }]) test(`Actual Worker rejects explicit numerical mode with missing API ${JSON.stringify(missing)}`, async () => {
    const { worker, call, factory } = actualWorker(missing);
    try {
        await call(request('init', { wasmPath: factory }, 1, 1));
        const config = { width: 2, height: 2, fx: 1, fy: 1, cx: 1, cy: 1 };
        const defaults = await call(request('configure', config, 2, 2));
        assert.equal(defaults.success, true); assert.equal(defaults.benchmarkSolverProfile, null);
        const requested = await call(request('configure', { ...config, benchmark_zero_positive_tolerances: true }, 3, 3));
        assert.equal(requested.success, false); assert.match(requested.error, /benchmark.*unavailable/i);
    } finally { await worker.terminate(); }
});

test('Actual Worker fails configuration when the engine getter does not confirm the requested profile', async () => {
    const { worker, call, factory } = actualWorker({ mismatch: true });
    try {
        await call(request('init', { wasmPath: factory }, 1, 1));
        const reply = await call(request('configure', { width: 2, height: 2, benchmark_zero_positive_tolerances: true }, 2, 2));
        assert.equal(reply.success, false); assert.match(reply.error, /benchmark.*not applied/i);
    } finally { await worker.terminate(); }
});

function replayFixture(search) {
    const configurations = [];
    const context = vm.createContext({ console: { log() {}, warn() {}, error() {} }, Float64Array, Uint8Array, URLSearchParams,
        window: { location: { search } }, document: { addEventListener() {} }, performance: { now: () => 1, timeOrigin: 0 },
        fetch: async url => ({ ok: true, text: async () => url.includes('/cam0/') ? '1000000000,1000000000.png\n' :
            url.includes('/imu0/') ? '990000000,0,0,0,0,0,9.81\n1010000000,0,0,0,0,0,9.81\n' : '1000000000,0,0,0,1,0,0,0\n' }),
        VIOWrapper: class { async configure(value) { configurations.push(value); return true; }
            async setMobileParams() {} async setTrackingParams() {} async setFThreshold() {} async setPnPParams() {} getMetrics() { return {}; } } });
    vm.runInContext(replaySource.replace(/^import .*$/gm, '').replace(/const app = new TUMVITestApp\(\);[\s\S]*$/, '') + '\nglobalThis.Replay = TUMVITestApp;', context);
    const app = new context.Replay();
    app.ui = { startBtn: {}, pauseBtn: {}, stepBtn: {}, resetBtn: {}, speedSelect: { value: '0' } };
    app.setStatus = () => {}; app.log = () => {}; app.updateProgress = () => {};
    return { app, configurations };
}

for (const [query, requested] of [['?speed=0', false], ['?speed=0&benchmarkZeroTolerances=1', true], ['?speed=0&benchmarkZeroTolerances=0', false]]) {
    test(`Replay actual start sends an explicit profile and exports request independently: ${query}`, async () => {
        const { app, configurations } = replayFixture(query);
        await app.start();
        assert.equal(configurations.length, 1);
        assert.equal(configurations[0].benchmark_zero_positive_tolerances, requested);
        const report = app.exportReport();
        assert.equal(report.benchmarkSolverProfile.requested, requested);
        assert.equal(report.benchmarkSolverProfile.actual, null, 'No frame/getter evidence must remain unavailable');
        assert.equal(report.solver.solver_time, 0.1); assert.equal(report.solver.num_iterations, 10);
        app.rows = [{ frame: 0, benchmarkSolverProfile: !requested }];
        assert.equal(app.exportReport().benchmarkSolverProfile.actual, !requested, 'Actual row evidence must not mirror the request');
    });
}

test('Replay rejects an unsupported numerical profile query instead of silently selecting defaults', () => {
    assert.throws(() => replayFixture('?benchmarkZeroTolerances=2'), /benchmarkZeroTolerances/);
});

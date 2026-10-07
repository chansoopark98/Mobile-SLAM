import test from 'node:test';
import assert from 'node:assert/strict';
import fs from 'node:fs/promises';
import vm from 'node:vm';
import { Worker as NodeWorker } from 'node:worker_threads';

const wrapperSource = await fs.readFile(new URL('../../web/js/vio-wrapper.js', import.meta.url), 'utf8');
const workerSource = await fs.readFile(new URL('../../web/js/vio-worker.js', import.meta.url), 'utf8');
const asModule = source => 'data:text/javascript;base64,' + Buffer.from(source).toString('base64');
const { VIOWrapper } = await import(asModule(wrapperSource));

class FakeWorker {
    static instances = [];
    constructor() { this.messages = []; this.terminated = false; FakeWorker.instances.push(this); }
    postMessage(message, transfer = []) { this.messages.push(structuredClone(message, { transfer })); }
    terminate() { this.terminated = true; }
    reply(request, fields = {}) { this.onmessage({ data: { ...request, success: true, ...fields } }); }
    last(type) { return this.messages.findLast(message => message.type === type); }
}

async function loaded(options) {
    const wrapper = new VIOWrapper(options);
    const promise = wrapper.load('/vio_engine.js');
    const worker = FakeWorker.instances.at(-1);
    worker.reply(worker.last('init'));
    await promise;
    const configured = wrapper.configure({ width: 2, height: 2 });
    worker.reply(worker.last('configure'));
    assert.equal(await configured, true);
    return { wrapper, worker };
}

function result(worker, request, overrides = {}) {
    worker.reply(request, { type: 'result', data: {
        inputTimestamp: request.data.timestamp, poseTimestamp: request.data.timestamp,
        engineEpoch: 1, pose: new Float64Array(16), poseFresh: true, poseValid: true,
        initialized: true, featureCount: 20, statusCode: 2, imuCount: 5,
        mapPoints: new Float64Array([1, 2, 3]), mapPointCount: 1, ...overrides,
    } });
}

const previousWorker = globalThis.Worker;
globalThis.Worker = FakeWorker;
test.after(() => { globalThis.Worker = previousWorker; });

test('Concurrent identical RPCs correlate out-of-order replies independently', async () => {
    const { wrapper, worker } = await loaded();
    const first = wrapper.setFThreshold(1);
    const second = wrapper.setFThreshold(2);
    const requests = worker.messages.filter(message => message.type === 'setFThreshold');
    assert.notEqual(requests[0].requestId, requests[1].requestId);
    worker.reply(requests[1], { success: false });
    worker.reply(requests[0]);
    assert.deepEqual(await Promise.all([first, second]), [true, false]);
    wrapper.dispose();
});

test('Wrong request ID, sequence and epoch cannot release current frame or update pose', async () => {
    const { wrapper, worker } = await loaded();
    wrapper.sendFrame(new Uint8Array(4), 1);
    const request = worker.last('frame');
    result(worker, { ...request, requestId: request.requestId + 1 });
    result(worker, { ...request, sequence: request.sequence + 1 });
    result(worker, { ...request, clientEpoch: request.clientEpoch - 1 });
    assert.equal(wrapper.workerBusy, true);
    assert.equal(wrapper.getLatestResult(), null);
    assert.equal(wrapper.getMetrics().staleReplies, 3);
    result(worker, request);
    assert.equal(wrapper.workerBusy, false);
    assert.equal(wrapper.getLatestResult().requestId, request.requestId);
    wrapper.dispose();
});

test('Reset rejects every RPC and waiter; late old frame cannot complete new frame', async () => {
    const { wrapper, worker } = await loaded();
    const rpc = wrapper.setMobileParams(0.1, 3, 20);
    const rpcRejected = assert.rejects(rpc, /reset/);
    wrapper.sendFrame(new Uint8Array(4), 1);
    const oldFrame = worker.last('frame');
    const waiter1 = assert.rejects(wrapper.waitForFree(100), /reset/);
    const waiter2 = assert.rejects(wrapper.waitForFree(100), /reset/);
    wrapper.reset();
    worker.reply(worker.last('reset'));
    await Promise.all([rpcRejected, waiter1, waiter2]);
    assert.equal(wrapper.getMetrics().framesCancelled, 1);
    assert.equal(wrapper.getLatestResult(), null);
    assert.equal(wrapper.getMapPoints().count, 0);
    assert.equal(wrapper.sendFrame(new Uint8Array(4), 2), true);
    const newFrame = worker.last('frame');
    result(worker, oldFrame);
    assert.equal(wrapper.workerBusy, true);
    assert.equal(wrapper.getLatestResult(), null);
    result(worker, newFrame);
    assert.equal(wrapper.getLatestResult().inputTimestamp, 2);
    const metrics = wrapper.getMetrics();
    assert.equal(metrics.framesSubmitted, metrics.framesCompleted + metrics.framesCancelled + metrics.framesTimedOut);
    assert.ok(wrapper.getLatestResult().frameCopyMs >= 0);
    wrapper.dispose();
});

test('Reconfigure invalidates pending work and ignores superseded configure responses', async () => {
    const { wrapper, worker } = await loaded();
    wrapper.sendFrame(new Uint8Array(4), 1);
    const oldFrame = worker.last('frame');
    const wait = assert.rejects(wrapper.waitForFree(100), /reconfigured/);
    const first = wrapper.configure({ width: 2, height: 2 });
    const firstRequest = worker.last('configure');
    const cancelled = assert.rejects(first, /reconfigured/);
    const second = wrapper.configure({ width: 2, height: 2 });
    worker.reply(firstRequest);
    assert.equal(wrapper.configured, false);
    result(worker, oldFrame);
    assert.equal(wrapper.getLatestResult(), null);
    worker.reply(worker.last('configure'));
    await Promise.all([wait, cancelled]);
    assert.equal(await second, true);
    wrapper.dispose();
});

test('Reset gates frames until its matching acknowledgment and failed reset keeps output invalid', async () => {
    const { wrapper, worker } = await loaded();
    wrapper.reset();
    assert.equal(wrapper.sendFrame(new Uint8Array(4), 1), false);
    worker.reply(worker.last('reset'));
    await Promise.resolve();
    assert.equal(wrapper.configured, true);
    wrapper.reset();
    worker.reply(worker.last('reset'), { success: false, error: 'reset fault' });
    await Promise.resolve();
    assert.equal(wrapper.configured, false);
    assert.equal(wrapper.getLatestResult(), null);
    assert.equal(wrapper.getMetrics().lastError, 'reset fault');
    wrapper.dispose();
});

test('Dispose during load rejects init; replaced worker messages have no authority', async () => {
    const wrapper = new VIOWrapper();
    const init = wrapper.load('/vio_engine.js');
    const worker = FakeWorker.instances.at(-1);
    const rejection = assert.rejects(init, /disposed/);
    const request = worker.last('init');
    wrapper.dispose();
    worker.reply(request);
    await rejection;
    assert.equal(worker.terminated, true);
    assert.equal(wrapper.worker, null);
    assert.equal(wrapper.configured, false);
});

test('Dispose settles every active setter and frame waiter exactly once', async () => {
    const { wrapper, worker } = await loaded();
    const setter1 = assert.rejects(wrapper.setFThreshold(1), /disposed/);
    const setter2 = assert.rejects(wrapper.setFThreshold(2), /disposed/);
    wrapper.sendFrame(new Uint8Array(4), 1);
    const waiter1 = assert.rejects(wrapper.waitForFree(100), /disposed/);
    const waiter2 = assert.rejects(wrapper.waitForFree(100), /disposed/);
    wrapper.dispose();
    await Promise.all([setter1, setter2, waiter1, waiter2]);
    assert.equal(wrapper._pending.size, 0);
    assert.equal(wrapper._freeWaiters.size, 0);
    assert.equal(wrapper.getMetrics().framesCancelled, 1);
    assert.equal(worker.terminated, true);
});

test('Load replacement rejects prior init and binds callbacks to the new worker only', async () => {
    const wrapper = new VIOWrapper();
    const first = wrapper.load('/first.js');
    const oldWorker = FakeWorker.instances.at(-1);
    const cancelled = assert.rejects(first, /replaced/);
    const second = wrapper.load('/second.js');
    const newWorker = FakeWorker.instances.at(-1);
    oldWorker.reply(oldWorker.last('init'));
    newWorker.reply(newWorker.last('init'));
    await Promise.all([cancelled, second]);
    assert.equal(wrapper.worker, newWorker);
    assert.equal(oldWorker.terminated, true);
    wrapper.dispose();
});

test('Worker runtime rejection clears busy/caches and rejects all pending consumers', async () => {
    const { wrapper, worker } = await loaded();
    wrapper.sendFrame(new Uint8Array(4), 1);
    result(worker, worker.last('frame'));
    const rpc1 = assert.rejects(wrapper.setFThreshold(2), /runtime/);
    const rpc2 = assert.rejects(wrapper.setPnPParams(false), /runtime/);
    wrapper.sendFrame(new Uint8Array(4), 2);
    const waiter = assert.rejects(wrapper.waitForFree(100), /runtime/);
    worker.onmessage({ data: { type: 'runtime_error', clientEpoch: wrapper.getMetrics().clientEpoch, error: 'runtime rejection' } });
    await Promise.all([rpc1, rpc2, waiter]);
    assert.equal(wrapper.workerBusy, false);
    assert.equal(wrapper.configured, false);
    assert.equal(wrapper.getLatestResult(), null);
    assert.equal(wrapper.getMapPoints().count, 0);
    assert.equal(worker.terminated, true);
});

test('Native Worker error during a setter rejects every RPC and waiter', async () => {
    const { wrapper, worker } = await loaded();
    const rpc = assert.rejects(wrapper.setTrackingParams(21, 3, 20, 0), /engine crashed/);
    wrapper.sendFrame(new Uint8Array(4), 1);
    const waiter = assert.rejects(wrapper.waitForFree(100), /engine crashed/);
    worker.onerror({ message: 'engine crashed' });
    await Promise.all([rpc, waiter]);
    assert.equal(wrapper.workerBusy, false);
    assert.equal(wrapper.getMetrics().workerErrors, 1);
});

test('RPC deadline rejects a missing response without leaving pending entries', async () => {
    const { wrapper } = await loaded({ requestTimeoutMs: 10 });
    await assert.rejects(wrapper.setFThreshold(2), /timeout/);
    assert.equal(wrapper._pending.size, 0);
    wrapper.dispose();
});

test('Initialization deadline releases worker resources and rejects the bounded load', async () => {
    const wrapper = new VIOWrapper({ requestTimeoutMs: 10 });
    const loading = wrapper.load('/never-replies.js');
    const worker = FakeWorker.instances.at(-1);
    await assert.rejects(loading, /init timeout/);
    assert.equal(wrapper._pending.size, 0);
    assert.equal(wrapper.worker, null);
    assert.equal(worker.terminated, true);
});

test('Frame deadline settles all waiters and prevents accepting the late result', async () => {
    const { wrapper, worker } = await loaded({ frameTimeoutMs: 10 });
    wrapper.sendFrame(new Uint8Array(4), 1);
    const frame = worker.last('frame');
    const waiter1 = assert.rejects(wrapper.waitForFree(100), /frame timeout/);
    const waiter2 = assert.rejects(wrapper.waitForFree(100), /frame timeout/);
    await Promise.all([waiter1, waiter2]);
    assert.equal(wrapper.workerBusy, false);
    assert.equal(wrapper.configured, false);
    assert.equal(wrapper.getMetrics().framesTimedOut, 1);
    assert.equal(wrapper.getMetrics().framesCancelled, 0);
    result(worker, frame);
    assert.equal(wrapper.getLatestResult(), null);
});

test('Waiter deadlines are independent and do not corrupt the remaining completion waiter', async () => {
    const { wrapper, worker } = await loaded();
    wrapper.sendFrame(new Uint8Array(4), 1);
    const early = assert.rejects(wrapper.waitForFree(5), /timeout/);
    const later = wrapper.waitForFree(100);
    await early;
    result(worker, worker.last('frame'));
    await later;
    assert.equal(wrapper._freeWaiters.size, 0);
    wrapper.dispose();
});

test('Failure result retains correlation and input time while clearing pose/map freshness', async () => {
    const { wrapper, worker } = await loaded();
    wrapper.sendFrame(new Uint8Array(4), 3);
    const frame = worker.last('frame');
    const waiter = assert.rejects(wrapper.waitForFree(100), /failed solve/);
    worker.reply(frame, { type: 'result', success: false, error: 'failed solve', data: {
        inputTimestamp: 3, pose: new Float64Array(16), poseTimestamp: 3,
        poseFresh: true, poseValid: true, reason: 'worker_exception', engineEpoch: 2,
    } });
    await waiter;
    assert.equal(wrapper.getLatestResult().requestId, frame.requestId);
    assert.equal(wrapper.getLatestResult().inputTimestamp, 3);
    assert.equal(wrapper.getLatestResult().pose, null);
    assert.equal(wrapper.getLatestResult().poseTimestamp, null);
    assert.equal(wrapper.getLatestResult().poseFresh, false);
    assert.equal(wrapper.getMapPoints().count, 0);
    wrapper.dispose();
});

test('Engine epoch change and tracking loss clear old pose/map; metadata does not fake freshness', async () => {
    const { wrapper, worker } = await loaded();
    wrapper.sendFrame(new Uint8Array(4), 1);
    result(worker, worker.last('frame'));
    assert.equal(wrapper.getMapPoints().count, 1);
    wrapper.sendFrame(new Uint8Array(4), 2);
    result(worker, worker.last('frame'), { engineEpoch: 2, initialized: false, poseFresh: false, poseValid: false, reason: 'frame_gap' });
    assert.equal(wrapper.getLatestResult().engineEpoch, 2);
    assert.equal(wrapper.getLatestResult().pose, null);
    assert.equal(wrapper.getLatestResult().poseTimestamp, null);
    assert.equal(wrapper.getMapPoints().count, 0);
    wrapper.sendFrame(new Uint8Array(4), 3);
    result(worker, worker.last('frame'), { engineEpoch: 2, poseTimestamp: null });
    assert.equal(wrapper.getLatestResult().poseValid, false);
    wrapper.dispose();
});

test('Correlated replies with wrong input time or regressing engine epoch cannot expose a pose', async () => {
    const { wrapper, worker } = await loaded();
    wrapper.sendFrame(new Uint8Array(4), 1);
    result(worker, worker.last('frame'), { engineEpoch: 4 });
    wrapper.sendFrame(new Uint8Array(4), 2);
    const mismatch = assert.rejects(wrapper.waitForFree(100), /timestamp mismatch/);
    result(worker, worker.last('frame'), { inputTimestamp: 99, engineEpoch: 4 });
    await mismatch;
    assert.equal(wrapper.getLatestResult().inputTimestamp, 2);
    assert.equal(wrapper.getLatestResult().pose, null);
    wrapper.sendFrame(new Uint8Array(4), 3);
    const stale = assert.rejects(wrapper.waitForFree(100), /Stale engine epoch/);
    result(worker, worker.last('frame'), { engineEpoch: 3 });
    await stale;
    assert.equal(wrapper.getLatestResult().pose, null);
    assert.equal(wrapper.getMapPoints().count, 0);
    wrapper.dispose();
});

test('Structured-clone message failure rejects current consumers and invalidates cache', async () => {
    const { wrapper, worker } = await loaded();
    wrapper.sendFrame(new Uint8Array(4), 1);
    result(worker, worker.last('frame'));
    const rpc = assert.rejects(wrapper.setFThreshold(2), /deserialization/);
    wrapper.sendFrame(new Uint8Array(4), 2);
    const waiter = assert.rejects(wrapper.waitForFree(100), /deserialization/);
    worker.onmessageerror();
    await Promise.all([rpc, waiter]);
    assert.equal(wrapper.workerBusy, false);
    assert.equal(wrapper.getMapPoints().count, 0);
    assert.equal(wrapper.getLatestResult(), null);
});

test('Exact IMU declared-count view transfer preserves prefix/suffix and input storage', async () => {
    const { wrapper, worker } = await loaded();
    const readings = new Float64Array([99, 0, 0, 0, 0, 0, 0, 1, 1, 2, 3, 4, 5, 6, 2, 7, 8, 9, 10, 11, 12]);
    assert.equal(wrapper.sendIMU(readings.subarray(7), 1), true);
    assert.deepEqual(Array.from(new Float64Array(worker.last('imu').data.imuData)), [1, 1, 2, 3, 4, 5, 6]);
    assert.equal(readings.byteLength, 21 * 8);
    assert.equal(wrapper.sendIMU(readings, 3.1), false);
    assert.equal(wrapper.sendIMU(readings, 4), false);
    assert.equal(wrapper.sendIMU(new Float64Array([1, NaN, 0, 0, 0, 0, 0]), 1), false);
    assert.equal(wrapper.sendIMU(new Float64Array([1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0]), 2), false);
    wrapper.dispose();
});

function workerFixture() {
    const replies = [];
    const context = vm.createContext({ self: { postMessage: message => replies.push(message) },
        Float64Array, Uint8Array, ArrayBuffer, console, performance, setTimeout, clearTimeout });
    vm.runInContext(workerSource + `\nglobalThis.audit = {
        appendIMU, drainIMUToWasm,
        memory(value, heap, count) { return new WorkerSharedMemory(value, heap, count); },
        stats() { return { ...imuDrops, size: imuRingWriteIdx - imuRingReadIdx }; },
        setEngine(value, heap) {
            engine = value; wasm = { HEAPF64: heap }; configured = true; imageWidth = 2; imageHeight = 2;
            engineEpoch = engineState().engineEpoch; memImage = { ptr: 0, write() {} }; memIMU = { ptr: 8 };
            memPose = { ptr: 0, read() { return new Float64Array(16); } };
            memMapPoints = { ptr: 0, read() { return new Float64Array([1,2,3]); } };
        }
    };`, context);
    const heap = new Float64Array(512 * 7 + 8);
    return { context, audit: context.audit, heap, replies };
}

function mockEngine(overrides = {}) {
    return {
        processFrame() { return false; }, getEpoch() { return 1; }, isInitialized() { return false; },
        getFeaturePointCount() { return 0; }, getStatusCode() { return 1; },
        getLastReason() { return 'imu_interval_uncovered'; }, getPoseTimestamp() { return -1; },
        getPoseFresh() { return false; }, getPoseValid() { return false; },
        getMapPoints() { return 0; }, reset() {}, ...overrides,
    };
}

const request = (type, data, sequence = 1, epoch = 0) => ({ type, data, requestId: sequence,
    sequence, clientEpoch: epoch, sentAtMs: performance.timeOrigin + performance.now() });

for (const [name, data] of [['empty IMU', { gray: new ArrayBuffer(4), timestamp: 1 }],
    ['invalid image', { gray: new ArrayBuffer(3), timestamp: 2 }], ['invalid timestamp', { gray: new ArrayBuffer(4), timestamp: NaN }]]) {
    test(`Worker ${name} result preserves request IDs and null pose time`, async () => {
        const { context, audit, heap, replies } = workerFixture();
        let calls = 0;
        audit.setEngine(mockEngine({ processFrame() { calls++; return false; } }), heap);
        const sent = request('frame', data, 7);
        await context.self.onmessage({ data: sent });
        const reply = replies.at(-1);
        assert.equal(reply.requestId, 7);
        assert.equal(reply.clientEpoch, 0);
        assert.equal(reply.sequence, 7);
        assert.equal(reply.data.poseTimestamp, null);
        assert.equal(reply.data.poseFresh, false);
        assert.equal(reply.data.inputTimestamp, Number.isFinite(data.timestamp) ? data.timestamp : null);
        assert.equal(calls, 0);
        if (name === 'empty IMU') assert.equal(reply.data.reason, 'missing_bracket');
        assert.ok(Number.isFinite(reply.data.workerProcessingMs));
    });
}

test('Worker exception resets engine and echoes frame correlation with failure reason', async () => {
    const { context, audit, heap, replies } = workerFixture();
    let resets = 0;
    audit.setEngine(mockEngine({ processFrame() { throw new Error('test fault'); }, reset() { resets++; } }), heap);
    audit.appendIMU(new Float64Array([4,0,0,9.81,0,0,0,4.01,0,0,9.81,0,0,0]), 2);
    await context.self.onmessage({ data: request('frame', { gray: new ArrayBuffer(4), timestamp: 4 }, 9) });
    const reply = replies.at(-1);
    assert.equal(reply.requestId, 9);
    assert.equal(reply.success, false);
    assert.equal(reply.error, 'test fault');
    assert.equal(reply.data.inputTimestamp, 4);
    assert.equal(reply.data.reason, 'worker_exception');
    assert.equal(reply.data.poseTimestamp, null);
    assert.equal(resets, 1);
});

test('Worker rejects duplicate sequence/old epoch without calling engine or losing frame input ID', async () => {
    const { context, audit, heap, replies } = workerFixture();
    let calls = 0;
    audit.setEngine(mockEngine({ processFrame() { calls++; return false; } }), heap);
    audit.appendIMU(new Float64Array([1,0,0,9.81,0,0,0,1.01,0,0,9.81,0,0,0]), 2);
    const first = request('frame', { gray: new ArrayBuffer(4), timestamp: 1 }, 3);
    await context.self.onmessage({ data: first });
    await context.self.onmessage({ data: first });
    assert.equal(replies.at(-1).error, 'stale_sequence');
    assert.equal(replies.at(-1).data.inputTimestamp, 1);
    await context.self.onmessage({ data: request('reset', undefined, 1, 1) });
    await context.self.onmessage({ data: first });
    assert.equal(replies.at(-1).error, 'stale_epoch');
    assert.equal(replies.at(-1).requestId, first.requestId);
    assert.equal(calls, 1);
});

test('Ring validates counts, finite fields/order and reports every discarded sample', () => {
    const { audit, heap } = workerFixture();
    audit.setEngine(mockEngine(), heap);
    assert.equal(audit.appendIMU(new Float64Array(7), 2), false);
    assert.equal(audit.appendIMU(new Float64Array([1, 0, 0, 9.81, 0, 0, 0, 1, 0, 0, 9.81, 0, 0, 0, 0.5, 0, 0, 9.81, 0, 0, 0, 2, NaN, 0, 0, 0, 0, 0, 3, 0, 0, 9.81, 0, 0, 0]), 5), false);
    const stats = audit.stats();
    assert.equal(stats.invalidBatches, 1);
    assert.equal(stats.invalid, 3);
    assert.equal(stats.order, 2);
    assert.equal(stats.accepted, 2);
    assert.equal(stats.received, stats.accepted + stats.invalid + stats.order);
    assert.equal(stats.gapCount, 1);
    assert.equal(stats.lastGapSeconds, 2);
    assert.equal(audit.drainIMUToWasm(3), 1);
    assert.equal(heap[1], 3);
    assert.equal(audit.stats().stale, 1);
});

test('WASM shared buffers use current heap after growth and enforce allocated read/write bounds', () => {
    const { audit } = workerFixture();
    const initial = new ArrayBuffer(64);
    const module = { HEAPU8: new Uint8Array(initial), HEAPF64: new Float64Array(initial), _malloc() { return 8; }, _free() {} };
    const memory = audit.memory(module, module.HEAPF64, 2);
    memory.write(new Float64Array([1, 2]));
    const grown = new ArrayBuffer(128);
    new Uint8Array(grown).set(module.HEAPU8);
    module.HEAPU8 = new Uint8Array(grown);
    module.HEAPF64 = new Float64Array(grown);
    memory.write(new Float64Array([3, 4]));
    assert.deepEqual(Array.from(memory.read(2)), [3, 4]);
    assert.deepEqual(Array.from(new Float64Array(initial).subarray(1, 3)), [1, 2]);
    assert.throws(() => memory.write(new Float64Array(3)), /exceeds/);
    assert.throws(() => memory.read(3), /exceeds/);
    assert.throws(() => memory.read(-1), /exceeds/);
    memory.dispose();
    assert.equal(memory.ptr, 0);
});

test('Allocation-triggered heap growth preserves Float64 buffer element type', () => {
    const { audit } = workerFixture();
    const initial = new ArrayBuffer(64);
    const module = { HEAPU8: new Uint8Array(initial), HEAPF64: new Float64Array(initial), _free() {}, _malloc() {
        const grown = new ArrayBuffer(128);
        module.HEAPU8 = new Uint8Array(grown); module.HEAPF64 = new Float64Array(grown);
        return 8;
    } };
    const memory = audit.memory(module, module.HEAPF64, 2);
    memory.write(new Float64Array([3, 4]));
    assert.equal(module.HEAPF64[1], 3);
    assert.equal(module.HEAPF64[2], 4);
    assert.ok(memory.read(2) instanceof Float64Array);
});

test('Malformed Worker IMU buffer echoes request IDs and counts every declared rejected reading', async () => {
    const { context, audit, heap, replies } = workerFixture();
    audit.setEngine(mockEngine(), heap);
    await context.self.onmessage({ data: request('imu', { imuData: new ArrayBuffer(9), count: 2 }, 7) });
    const reply = replies.at(-1);
    assert.equal(reply.requestId, 7);
    assert.equal(reply.sequence, 7);
    assert.equal(reply.success, false);
    assert.equal(reply.imuDrops.received, 2);
    assert.equal(reply.imuDrops.invalid, 2);
    assert.equal(reply.imuDrops.invalidBatches, 1);
});

test('Reset preserves cumulative Worker drop counters and records discarded future IMU', async () => {
    const { context, audit, heap, replies } = workerFixture();
    audit.setEngine(mockEngine(), heap);
    audit.appendIMU(new Float64Array([1,0,0,9.81,0,0,0,1.01,0,0,9.81,0,0,0]), 2);
    await context.self.onmessage({ data: request('reset', undefined, 1, 1) });
    assert.equal(replies.at(-1).imuDrops.reset, 2);
    assert.equal(replies.at(-1).imuDrops.accepted, 2);
    assert.equal(audit.stats().size, 0);
});

test('Sub-0.5s capacity loss disconnects a seeded engine before admitting retained IMU', async () => {
    const { context, audit, heap, replies } = workerFixture();
    let epoch = 1, resets = 0, calls = 0;
    audit.setEngine(mockEngine({
        getEpoch() { return epoch; }, getIMUEndpointTimestamp() { return 1; },
        reset() { resets++; epoch++; },
        processFrame() { assert.equal(resets, 1); assert.equal(epoch, 2); calls++; return true; }, isInitialized() { return true; },
        getPoseFresh() { return true; }, getPoseValid() { return true; }, getPoseTimestamp() { return 1.3495; },
        getStatusCode() { return 2; }, getLastReason() { return 'tracking'; },
    }), heap);
    const values = new Float64Array(700 * 7);
    for (let i = 0; i < 700; i++) values.set([1 + i * 0.0005,0,0,9.81,0,0,0], i * 7);
    audit.appendIMU(values, 700);
    await context.self.onmessage({ data: request('frame', { gray: new ArrayBuffer(4), timestamp: 1.3495 }, 1) });
    const result = replies.at(-1).data;
    assert.equal(resets, 1);
    assert.equal(calls, 1);
    assert.equal(result.reason, 'imu_transport_loss');
    assert.equal(result.engineEpoch, 2);
    assert.equal(result.pose, null); assert.equal(result.poseFresh, false); assert.equal(result.poseValid, false);
    assert.equal(result.mapPointCount, 0);
    assert.equal(result.imuIntervalLoss.reasons.capacity, 188);
    assert.equal(result.imuIntervalLoss.engineEndpointDisconnected, true);
    assert.equal(result.imuIntervalLoss.preexistingEngineEndpointTimestamp, 1);
    assert.ok(result.imuIntervalLoss.droppedTimestampMax - 1 < .5);
});

test('Overflow/stale loss coalesces to one reset, while duplicate rejection alone preserves lifecycle', async () => {
    const { context, audit, heap, replies } = workerFixture();
    let epoch = 1, resets = 0;
    audit.setEngine(mockEngine({ getEpoch() { return epoch; }, getIMUEndpointTimestamp() { return -1; }, reset() { epoch++; resets++; } }), heap);
    const values = new Float64Array(1100 * 7);
    for (let i = 0; i < 1100; i++) values.set([1 + i*.005,0,0,9.81,0,0,0], i*7);
    audit.appendIMU(values, 1100);
    await context.self.onmessage({ data: request('frame', { gray: new ArrayBuffer(4), timestamp: 5.8 }, 1) });
    const result = replies.at(-1).data;
    assert.equal(resets, 1);
    assert.equal(result.imuIntervalLoss.reasons.overflow, 76);
    assert.ok(result.imuIntervalLoss.reasons.stale > 0);
    assert.equal(result.imuIntervalLoss.engineEndpointDisconnected, false);
    assert.ok(audit.stats().size > 0, 'Ordered future readings remain available only to the new lifecycle');
    audit.appendIMU(values.subarray(1099*7), 1);
    await context.self.onmessage({ data: request('frame', { gray: new ArrayBuffer(4), timestamp: 5.805 }, 2) });
    assert.equal(resets, 1, 'Duplicate sample alone must not reset a healthy interval');
    assert.equal(replies.at(-1).data.imuIntervalLoss, null);
});

test('WASM IMU capacity truncation counts lost prefix and retains order', () => {
    const { audit, heap } = workerFixture();
    audit.setEngine(mockEngine(), heap);
    const batch = new Float64Array(700 * 7);
    for (let i = 0; i < 700; i++) batch.set([1 + i * 0.0005, 0, 0, 9.81, 0, 0, 0], i * 7);
    assert.equal(audit.appendIMU(batch, 700), true);
    assert.equal(audit.drainIMUToWasm(1.35), 512);
    assert.equal(audit.stats().capacity, 188);
    assert.equal(audit.stats().size, 0);
    assert.equal(heap[1], 1 + 188 * 0.0005);
    for (let i = 1; i < 512; i++) assert.ok(heap[1 + i * 7] > heap[1 + (i - 1) * 7]);
});

test('Numeric engine epoch normalization avoids spurious BigInt reset and clears queued IMU on real reset', async () => {
    const { context, audit, heap, replies } = workerFixture();
    let epoch = 1n;
    audit.setEngine(mockEngine({ getEpoch() { return epoch; } }), heap);
    audit.appendIMU(new Float64Array([1,0,0,9.81,0,0,0,1.01,0,0,9.81,0,0,0,1.02,0,0,9.81,0,0,0]), 3);
    await context.self.onmessage({ data: request('frame', { gray: new ArrayBuffer(4), timestamp: 1 }, 1) });
    assert.equal(replies.at(-1).data.engineEpoch, 1);
    assert.equal(audit.stats().size, 1);
    assert.equal(audit.stats().reset, 0);
    epoch = 2n;
    await context.self.onmessage({ data: request('frame', { gray: new ArrayBuffer(4), timestamp: 1.005 }, 2) });
    assert.equal(replies.at(-1).data.engineEpoch, 2);
    assert.equal(audit.stats().size, 0);
    assert.doesNotThrow(() => JSON.stringify(replies.at(-1)));
});

// This executes the actual transport Worker script in an isolated Worker thread;
// the controlled engine is a protocol fixture, not estimator/accuracy evidence.
const fakeWasm = asModule(`export default function () {
    const memory = new ArrayBuffer(1024 * 1024);
    const HEAPU8 = new Uint8Array(memory), HEAPF64 = new Float64Array(memory);
    let next = 8;
    return { HEAPU8, HEAPF64, _malloc(size) { const p = next; next += Math.ceil(size/8)*8; return p; }, _free() {},
        VIOEngine: class {
            constructor() { this.epoch=0; this.timestamp=-1; this.fresh=false; }
            configure() { this.epoch++; return true; }
            processFrame(image,w,h,imu,count,ts,pose) {
                this.fresh=false;
                if (ts===99) throw new Error('injected worker engine fault');
                if (!count) return false;
                this.timestamp=ts-0.01; this.fresh=true;
                HEAPF64.set([1,0,0,1,0,1,0,2,0,0,1,3,0,0,0,1],pose/8); return true;
            }
            getEpoch() { return this.epoch; } getFrameTimestamp() { return this.timestamp+0.01; }
            getIMUEndpointTimestamp() { return this.fresh ? this.timestamp+0.01 : -1; }
            getPoseTimestamp() { return this.timestamp; } getPoseFresh() { return this.fresh; }
            getPoseValid() { return this.fresh; } isInitialized() { return this.fresh; }
            getFeaturePointCount() { return 20; } getStatusCode() { return this.fresh?2:1; }
            getLastReason() { return this.fresh?'tracking':'imu_interval_uncovered'; }
            getLastSolverIterations() { return 3; } getLastSolverTermination() { return 'CONVERGENCE'; }
            getMapPoints() { return 0; } reset() { this.epoch++; this.fresh=false; } delete() {}
        }
    };
}`);

function realWorker() {
    const bootstrap = `import { parentPort } from 'node:worker_threads';
        globalThis.self = { postMessage(message) { parentPort.postMessage(message); } };
        await import(${JSON.stringify(asModule(workerSource))});
        parentPort.on('message', message => self.onmessage({ data: message }));`;
    const worker = new NodeWorker(new URL(asModule(bootstrap)), { type: 'module' });
    const waiting = new Map();
    worker.on('message', message => {
        const pending = waiting.get(message.requestId);
        if (pending && message.type !== 'wasm_log') { clearTimeout(pending.timer); waiting.delete(message.requestId); pending.resolve(message); }
    });
    return {
        worker,
        call(message) {
            return new Promise((resolve, reject) => {
                const timer = setTimeout(() => { waiting.delete(message.requestId); reject(new Error('real Worker test timeout')); }, 3000);
                waiting.set(message.requestId, { resolve, reject, timer });
                worker.postMessage(message);
            });
        },
    };
}

test('Actual Worker dispatch echoes lifecycle/frame/error IDs and reports separate pose timing', async () => {
    const { worker, call } = realWorker();
    try {
        assert.equal((await call(request('init', { wasmPath: fakeWasm }, 1, 1))).success, true);
        assert.equal((await call(request('configure', { width: 2, height: 2, fx: 1, fy: 1, cx: 1, cy: 1 }, 2, 2))).success, true);
        const imu = new Float64Array([1,0,0,9.81,0,0,0,1.01,0,0,9.81,0,0,0]);
        assert.equal((await call(request('imu', { imuData: imu.buffer, count: 2 }, 3, 2))).success, true);
        const frame = await call(request('frame', { gray: new ArrayBuffer(4), timestamp: 1 }, 4, 2));
        assert.equal(frame.requestId, 4);
        assert.equal(frame.data.inputTimestamp, 1);
        assert.equal(frame.data.poseTimestamp, 0.99);
        assert.equal(frame.data.poseFresh, true);
        assert.equal(frame.data.engineEpoch, 1);
        assert.equal(frame.data.solverIterations, 3);
        assert.ok(frame.data.workerQueueMs >= 0);
        assert.ok(frame.data.workerProcessingMs >= frame.data.engineProcessingMs);
        await call(request('imu', { imuData: new Float64Array([99,0,0,9.81,0,0,0,99.01,0,0,9.81,0,0,0]).buffer, count: 2 }, 5, 2));
        const fault = await call(request('frame', { gray: new ArrayBuffer(4), timestamp: 99 }, 6, 2));
        assert.equal(fault.requestId, 6);
        assert.equal(fault.success, false);
        assert.equal(fault.data.inputTimestamp, 99);
        assert.equal(fault.data.poseTimestamp, null);
        assert.equal(fault.data.engineEpoch, 2);
        assert.equal(fault.error, 'injected worker engine fault');
        const reset = await call(request('reset', undefined, 7, 3));
        assert.equal(reset.success, true);
        const old = await call(request('frame', { gray: new ArrayBuffer(4), timestamp: 2 }, 8, 2));
        assert.equal(old.error, 'stale_epoch');
        assert.equal(old.data.inputTimestamp, 2);
        assert.equal((await call(request('dispose', undefined, 9, 4))).success, true);
    } finally { await worker.terminate(); }
});

test('Actual Worker reset during async module init prevents old-generation engine installation', async () => {
    const { worker, call } = realWorker();
    const delayedFactory = asModule(`export default async function () {
        await new Promise(resolve => setTimeout(resolve, 30));
        return { VIOEngine: class { configure() { return true; } delete() {} } };
    }`);
    try {
        const init = call(request('init', { wasmPath: delayedFactory }, 1, 1));
        assert.equal((await call(request('reset', undefined, 2, 2))).success, true);
        const stale = await init;
        assert.equal(stale.requestId, 1);
        assert.equal(stale.error, 'stale_epoch');
        const config = await call(request('configure', { width: 2, height: 2 }, 3, 2));
        assert.equal(config.success, false);
        assert.equal(config.error, 'Invalid image dimensions');
    } finally { await worker.terminate(); }
});

test('Actual Worker waits for independently delivered late right bracket before processing once', async () => {
    const { worker, call } = realWorker();
    try {
        await call(request('init', { wasmPath: fakeWasm }, 1, 1));
        await call(request('configure', { width: 2, height: 2, fx: 1, fy: 1, cx: 1, cy: 1 }, 2, 2));
        const frame = call(request('frame', { gray: new ArrayBuffer(4), timestamp: 1 }, 3, 2));
        await new Promise(resolve => setTimeout(resolve, 8));
        await call(request('imu', { imuData: new Float64Array([0.99,0,0,9.81,0,0,0,1.01,0,0,9.81,0,0,0]).buffer, count: 2 }, 4, 2));
        const reply = await frame;
        assert.equal(reply.requestId, 3);
        assert.equal(reply.success, true);
        assert.equal(reply.data.poseFresh, true);
        assert.equal(reply.data.poseTimestamp, 0.99);
        assert.equal(reply.data.imuCount, 2);
        assert.ok(reply.data.imuWaitMs > 0 && reply.data.imuWaitMs < 40);
        assert.ok(reply.data.workerProcessingMs >= reply.data.engineProcessingMs);
    } finally { await worker.terminate(); }
});

test('Actual Worker missing-bracket deadline returns correlated null freshness without extrapolation', async () => {
    const { worker, call } = realWorker();
    try {
        await call(request('init', { wasmPath: fakeWasm }, 1, 1));
        await call(request('configure', { width: 2, height: 2, fx: 1, fy: 1, cx: 1, cy: 1 }, 2, 2));
        const reply = await call(request('frame', { gray: new ArrayBuffer(4), timestamp: 1 }, 3, 2));
        assert.equal(reply.requestId, 3);
        assert.equal(reply.data.reason, 'missing_bracket');
        assert.equal(reply.data.inputTimestamp, 1);
        assert.equal(reply.data.poseTimestamp, null);
        assert.equal(reply.data.poseFresh, false);
        assert.equal(reply.data.poseValid, false);
        assert.equal(reply.data.engineFrameTimestamp, null);
        assert.ok(reply.data.imuWaitMs >= 35);
        assert.equal(reply.data.engineProcessingMs, 0);
    } finally { await worker.terminate(); }
});

test('Actual Worker reset cancels bracket wait and old completion cannot release a newer frame', async () => {
    const { worker, call } = realWorker();
    try {
        await call(request('init', { wasmPath: fakeWasm }, 1, 1));
        await call(request('configure', { width: 2, height: 2, fx: 1, fy: 1, cx: 1, cy: 1 }, 2, 2));
        const oldFrame = call(request('frame', { gray: new ArrayBuffer(4), timestamp: 1 }, 3, 2));
        await new Promise(resolve => setTimeout(resolve, 8));
        const reset = await call({ ...request('reset', undefined, 1, 3), requestId: 4 });
        assert.equal(reset.success, true);
        const cancelled = await oldFrame;
        assert.equal(cancelled.requestId, 3);
        assert.equal(cancelled.error, 'stale_epoch');
        assert.equal(cancelled.data.poseFresh, false);
        const newFrame = call({ ...request('frame', { gray: new ArrayBuffer(4), timestamp: 2 }, 2, 3), requestId: 5 });
        await call({ ...request('imu', { imuData: new Float64Array([1.99,0,0,9.81,0,0,0,2.01,0,0,9.81,0,0,0]).buffer, count: 2 }, 3, 3), requestId: 6 });
        const completed = await newFrame;
        assert.equal(completed.requestId, 5);
        assert.equal(completed.clientEpoch, 3);
        assert.equal(completed.data.poseFresh, true);
        assert.equal(completed.data.inputTimestamp, 2);
    } finally { await worker.terminate(); }
});

test('Actual Worker capacity loss invalidates established pose and disconnects its actual endpoint', async () => {
    const { worker, call } = realWorker();
    try {
        await call(request('init', { wasmPath: fakeWasm }, 1, 1));
        await call(request('configure', { width: 2, height: 2, fx: 1, fy: 1, cx: 1, cy: 1 }, 2, 2));
        await call(request('imu', { imuData: new Float64Array([1,0,0,9.81,0,0,0,1.01,0,0,9.81,0,0,0]).buffer, count: 2 }, 3, 2));
        const established = await call(request('frame', { gray: new ArrayBuffer(4), timestamp: 1 }, 4, 2));
        assert.equal(established.data.poseFresh, true);
        const samples = new Float64Array(700 * 7);
        for (let i = 0; i < 700; i++) samples.set([1.011+i*.00048,0,0,9.81,0,0,0], i*7);
        const timestamp = samples[699*7];
        await call(request('imu', { imuData: samples.buffer, count: 700 }, 5, 2));
        const lost = await call(request('frame', { gray: new ArrayBuffer(4), timestamp }, 6, 2));
        assert.equal(lost.data.reason, 'imu_transport_loss');
        assert.equal(lost.data.pose, null); assert.equal(lost.data.poseTimestamp, null);
        assert.equal(lost.data.poseFresh, false); assert.equal(lost.data.mapPointCount, 0);
        assert.equal(lost.data.engineEpoch, established.data.engineEpoch+1);
        assert.equal(lost.data.imuIntervalLoss.reasons.capacity, 188);
        assert.equal(lost.data.imuIntervalLoss.engineEndpointDisconnected, true);
        assert.equal(lost.data.imuDrops.lossResets, 1);
    } finally { await worker.terminate(); }
});

test('Actual module Worker rejects unsupported classic/default-less artifacts with correlated error', async () => {
    const { worker, call } = realWorker();
    try {
        const init = await call(request('init', { wasmPath: asModule('export const VIOWasm = () => {};') }, 1, 1));
        assert.equal(init.requestId, 1);
        assert.equal(init.success, false);
        assert.match(init.error, /ES module factory/);
    } finally { await worker.terminate(); }
});

import test from 'node:test';
import assert from 'node:assert/strict';
import fs from 'node:fs/promises';
import vm from 'node:vm';

const source = await fs.readFile(new URL('../../web/js/vio-wrapper.js', import.meta.url), 'utf8');
const { VIOWrapper } = await import('data:text/javascript;base64,' + Buffer.from(source).toString('base64'));
const workerSource = await fs.readFile(new URL('../../web/js/vio-worker.js', import.meta.url), 'utf8');

function configuredWrapper() {
    const wrapper = new VIOWrapper();
    const messages = [];
    wrapper.configured = true;
    wrapper.worker = { postMessage: (...args) => messages.push(args), terminate() {} };
    return { wrapper, messages };
}

function complete(wrapper, messages, data = {}) {
    const request = messages.findLast(([message]) => message.type === 'frame')[0];
    wrapper._handleWorkerMessage({ ...request, type: 'result', success: true, data: {
        inputTimestamp: request.data.timestamp, poseTimestamp: request.data.timestamp,
        engineEpoch: 1, pose: new Float64Array(16), poseFresh: true, poseValid: true,
        initialized: true, featureCount: 50, statusCode: 2, imuCount: 10,
        mapPoints: null, mapPointCount: 0, ...data,
    } });
}

test('Busy worker rejects new frame while independent IMU remains deliverable', () => {
    const { wrapper, messages } = configuredWrapper();
    wrapper.workerBusy = true;
    assert.equal(wrapper.sendFrame(new Uint8Array(4), 1), false);
    assert.equal(wrapper.sendIMU(new Float64Array([1, 0, 0, 9.81, 0, 0, 0]), 1), true);
    assert.equal(messages.length, 1);
    assert.equal(messages[0][0].type, 'imu');
    wrapper.dispose();
});

test('Frame subarray submits exactly its bytes and retains camera reusable buffer', () => {
    const { wrapper, messages } = configuredWrapper();
    const pixels = new Uint8Array([1, 2, 3, 4, 5]);
    assert.equal(wrapper.sendFrame(pixels.subarray(1, 4), 2), true);
    assert.deepEqual(Array.from(new Uint8Array(messages[0][0].data.gray)), [2, 3, 4]);
    assert.notEqual(messages[0][0].data.gray, pixels.buffer);
    assert.equal(wrapper.workerBusy, true);
    wrapper.dispose();
});

test('Frame completion resolves waiting consumer and retains worker pose', async () => {
    const { wrapper, messages } = configuredWrapper();
    wrapper.sendFrame(new Uint8Array(4), 1);
    const completion = wrapper.waitForFree(100);
    complete(wrapper, messages);
    await completion;
    assert.equal(wrapper.workerBusy, false);
    assert.equal(wrapper.getLatestResult().imuCount, 10);
    wrapper.dispose();
});

test('Pose result preserves input timestamp separately from actual pose timestamp', () => {
    const { wrapper, messages } = configuredWrapper();
    wrapper.sendFrame(new Uint8Array(4), 3);
    complete(wrapper, messages, { poseTimestamp: 2.95 });
    assert.equal(wrapper.getLatestResult().timestamp, 3);
    assert.equal(wrapper.getLatestResult().inputTimestamp, 3);
    assert.equal(wrapper.getLatestResult().poseTimestamp, 2.95);
    wrapper.dispose();
});

test('Concurrent worker completion waiters all settle', async () => {
    const { wrapper, messages } = configuredWrapper();
    wrapper.sendFrame(new Uint8Array(4), 3);
    const first = wrapper.waitForFree(100);
    const second = wrapper.waitForFree(100);
    complete(wrapper, messages);
    await Promise.all([first, second]);
    assert.equal(wrapper.workerBusy, false);
    wrapper.dispose();
});

test('Empty map result clears previous points', () => {
    const { wrapper, messages } = configuredWrapper();
    wrapper.sendFrame(new Uint8Array(4), 1);
    complete(wrapper, messages, { mapPoints: new Float64Array([1, 2, 3]), mapPointCount: 1 });
    assert.equal(wrapper.getMapPoints().count, 1);
    wrapper.sendFrame(new Uint8Array(4), 2);
    complete(wrapper, messages, { mapPoints: null, mapPointCount: 0, pose: null, poseFresh: false, poseValid: false, statusCode: 3 });
    assert.equal(wrapper.getMapPoints().count, 0);
    assert.equal(wrapper.getLatestResult().poseTimestamp, null);
    wrapper.dispose();
});

test('IMU subarray transfers exactly its declared readings', () => {
    const { wrapper, messages } = configuredWrapper();
    const imu = new Float64Array([1, 1, 2, 3, 4, 5, 6, 2, 7, 8, 9, 10, 11, 12]);
    assert.equal(wrapper.sendIMU(imu.subarray(7), 1), true);
    const buffer = messages[0][0].data.imuData;
    assert.equal(buffer.byteLength, 7 * 8);
    assert.deepEqual(Array.from(new Float64Array(buffer)), [2, 7, 8, 9, 10, 11, 12]);
    assert.equal(imu.byteLength, 14 * 8);
    wrapper.dispose();
});

function ringFixture() {
    const heap = new ArrayBuffer(512 * 7 * 8 + 64);
    const context = vm.createContext({ self: {}, Float64Array, Uint8Array, console });
    vm.runInContext(workerSource + '\nglobalThis.ringAudit = { appendIMU, drainIMUToWasm, stats() { return { ...imuDrops, size: imuRingWriteIdx-imuRingReadIdx }; }, setHeap(value) { wasm = { HEAPF64: value }; memIMU = { ptr: 8 }; } };', context);
    const heapView = new Float64Array(heap);
    context.ringAudit.setHeap(heapView);
    return { ring: context.ringAudit, heapView };
}

test('Worker consumes one future bracket once and retains later IMU', () => {
    const { ring, heapView } = ringFixture();
    ring.appendIMU(new Float64Array([1, 0, 0, 9.81, 0, 0, 0, 1.05, 0, 0, 9.81, 0, 0, 0, 1.1, 0, 0, 9.81, 0, 0, 0]), 3);
    assert.equal(ring.drainIMUToWasm(1), 2);
    assert.equal(heapView[1], 1);
    assert.equal(heapView[8], 1.05);
    assert.equal(ring.drainIMUToWasm(1.1), 1);
    assert.equal(heapView[1], 1.1);
    assert.equal(ring.drainIMUToWasm(1.2), 0);
});

test('Ring overflow retains ordered recent IMU across delayed camera frame', () => {
    const { ring, heapView } = ringFixture();
    const data = new Float64Array(1100 * 7);
    for (let i = 0; i < 1100; i++) data.set([1 + i * 0.005, 0, 0, 9.81, 0, 0, 0], i * 7);
    ring.appendIMU(data, 1100);
    assert.equal(ring.stats().size, 1024);
    assert.equal(ring.stats().overflow, 76);
    const count = ring.drainIMUToWasm(5.8);
    assert.ok(count >= 99 && count <= 102, `expected about 100 readings in half-second window, got ${count}`);
    assert.ok(heapView[1] >= 5.3 && heapView[1] < 5.31);
    for (let i = 1; i < count; i++) assert.ok(heapView[1 + i * 7] > heapView[1 + (i - 1) * 7]);
    assert.equal(ring.stats().overflow + ring.stats().stale + count + ring.stats().size, 1100);
});

import { loadBrowserSource } from './browser-source-loader.mjs';
import { copyLowFeatureMetadata } from '../../scripts/dev/android-input-record.mjs';
import test from 'node:test';
import assert from 'node:assert/strict';
import fs from 'node:fs/promises';
import vm from 'node:vm';
import { limitedPipeline } from './limited-fixture.mjs';
import { Worker as NodeWorker } from 'node:worker_threads';

const wrapperSource = await loadBrowserSource('vio-wrapper');
const workerSource = await loadBrowserSource('vio-worker');
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

async function loaded(options, extraConfig = {}) {
    const wrapper = new VIOWrapper(options);
    const promise = wrapper.load('/vio_engine.js');
    const worker = FakeWorker.instances.at(-1);
    worker.reply(worker.last('init'));
    await promise;
    const configured = wrapper.configure({ width: 2, height: 2, ...extraConfig });
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

test('Only a matched result notifies after Worker becomes free, including ordinary Lost', async () => {
    const { wrapper, worker } = await loaded();
    const completed = [];
    wrapper.onFrameComplete = row => completed.push({ busy: wrapper.workerBusy, reason: row.reason });
    wrapper.sendFrame(new Uint8Array(4), 1);
    const request = worker.last('frame');
    result(worker, { ...request, requestId: request.requestId + 1 });
    assert.deepEqual(completed, []);
    result(worker, request, { poseFresh: false, poseValid: false, reason: 'insufficient_current_features' });
    assert.deepEqual(completed, [{ busy: false, reason: 'insufficient_current_features' }]);
    wrapper.dispose();
});

test('Lost and unknown statuses cannot publish a finite pose by claiming freshness', async () => {
    for (const statusCode of [0,1,3,4,99,'2','5',null]) {
        const { wrapper, worker } = await loaded();
        wrapper.sendFrame(new Uint8Array(4), 1);
        result(worker,worker.last('frame'),{statusCode,reason:'finite_but_not_tracking'});
        const latest=wrapper.getLatestResult();
        assert.equal(latest.statusCode,statusCode);
        assert.equal(latest.poseValid,false,`status ${statusCode}`);
        assert.equal(latest.poseFresh,false);
        assert.equal(latest.pose,null);
        assert.equal(latest.mapPointCount,0);
        wrapper.dispose();
    }
});

test('Limited keeps its current full pose, epoch/sequence and conditional quality without becoming normal Tracking', async () => {
    const {wrapper,worker}=await loaded();
    wrapper.sendFrame(new Uint8Array(4),1);
    const request=worker.last('frame'),pipeline=limitedPipeline();
    result(worker,request,{statusCode:5,reason:'tracking_limited',engineFrameTimestamp:1,pipeline,quality:pipeline.quality});
    const latest=wrapper.getLatestResult();
    assert.equal(latest.trackingState,'tracking_limited');
    assert.equal(latest.statusCode,5);
    assert.equal(latest.poseValid,true);
    assert.equal(latest.poseTimestamp,1);
    assert.equal(latest.sequence,request.sequence);
    assert.equal(latest.engineEpoch,1);
    assert.deepEqual(latest.quality,pipeline.quality);
    assert.equal(latest.quality.metricTranslationSupported,false);
    wrapper.dispose();
});

test('Limited rejects stale timestamps and incomplete or conflicting provenance instead of relabeling old finite poses', async () => {
    for(const mutate of [
        data=>{data.poseTimestamp=.9}, data=>{data.engineFrameTimestamp=.9},
        data=>{data.poseFresh=false}, data=>{data.quality=null},
        data=>{data.pipeline.schema='mobile-slam-pipeline-v1'},
        data=>{data.quality={...data.quality,translationSupport:'visual_inertial_metric'}},
        data=>{data.pipeline.quality.relativeCovariance6x6=null;data.quality=data.pipeline.quality},
    ]) {
        const {wrapper,worker}=await loaded();wrapper.sendFrame(new Uint8Array(4),1);
        const pipeline=limitedPipeline(),data={statusCode:5,reason:'tracking_limited',engineFrameTimestamp:1,pipeline,quality:pipeline.quality};
        mutate(data);result(worker,worker.last('frame'),data);
        const latest=wrapper.getLatestResult();
        assert.equal(latest.statusCode,5);
        assert.equal(latest.poseFresh,false);
        assert.equal(latest.poseValid,false);
        assert.equal(latest.pose,null);
        assert.equal(latest.mapPointCount,0);
        wrapper.dispose();
    }
});

test('Completion callback sees settled old frame and can keep a newly submitted frame active', async () => {
    const { wrapper, worker } = await loaded();
    wrapper.sendFrame(new Uint8Array(4), 1);
    const first = worker.last('frame');
    const waiter = wrapper.waitForFree(100);
    let next;
    wrapper.onFrameComplete = row => {
        assert.strictEqual(row, wrapper.getLatestResult());
        assert.equal(wrapper.workerBusy, false);
        assert.equal(wrapper._freeWaiters.size, 0);
        wrapper.onFrameComplete = null;
        assert.equal(wrapper.sendFrame(new Uint8Array(4), 2), true);
        next = worker.last('frame');
    };
    result(worker, first);
    await waiter;
    assert.ok(next.requestId > first.requestId);
    assert.equal(wrapper.workerBusy, true);
    assert.equal(wrapper._activeFrame.requestId, next.requestId);
    result(worker, next);
    wrapper.dispose();
});

test('Frame source timing is an owned request snapshot across caller mutation and reentrant dispatch', async t => {
    const { wrapper, worker } = await loaded();
    t.after(() => wrapper.dispose());
    const sourceTiming = { source: 'video_capture_time', timestampS: 1, presentedFrames: 10,
        association: 'video_frame_callback_snapshot', physicalExposureVerified: false };
    assert.equal(wrapper.sendFrame(new Uint8Array(4), 1, { sourceTiming }), true);
    const first = worker.last('frame');
    sourceTiming.source = 'caller_mutated';
    sourceTiming.timestampS = 99;
    let completed;
    wrapper.onFrameComplete = row => {
        completed = row;
        wrapper.onFrameComplete = null;
        assert.equal(wrapper.sendFrame(new Uint8Array(4), 2, {
            sourceTiming: { source: 'video_presentation', timestampS: 2 } }), true);
    };
    result(worker, first, { sourceTiming: { source: 'worker_payload_override' } });
    assert.deepEqual(completed.sourceTiming, { source: 'video_capture_time', timestampS: 1, presentedFrames: 10,
        association: 'video_frame_callback_snapshot', physicalExposureVerified: false });
    assert.notStrictEqual(completed.sourceTiming, sourceTiming);
    assert.strictEqual(wrapper.getLatestResult(), completed);
    assert.equal(wrapper._activeFrame.sourceTiming.source, 'video_presentation');
    result(worker, worker.last('frame'));
    assert.equal(wrapper.getLatestResult().sourceTiming.source, 'video_presentation');
    assert.equal(completed.sourceTiming.source, 'video_capture_time');
});

test('Rejected, reset and stale submissions cannot lend source timing to another result', async t => {
    const { wrapper, worker } = await loaded();
    t.after(() => wrapper.dispose());
    wrapper.sendFrame(new Uint8Array(4), 1, { sourceTiming: { source: 'cancelled' } });
    const oldFrame = worker.last('frame');
    assert.equal(wrapper.sendFrame(new Uint8Array(4), 2, { sourceTiming: { source: 'busy_rejected' } }), false);
    wrapper.reset();
    assert.equal(wrapper._activeFrame, null);
    assert.equal(wrapper.sendFrame(new Uint8Array(4), 2, { sourceTiming: { source: 'reset_rejected' } }), false);
    worker.reply(worker.last('reset'));
    await Promise.resolve();
    assert.equal(wrapper.sendFrame(null, 2, { sourceTiming: { source: 'invalid_rejected' } }), false);
    wrapper.sendFrame(new Uint8Array(4), 2, { sourceTiming: { source: 'current' } });
    const current = worker.last('frame');
    result(worker, oldFrame);
    result(worker, { ...current, sequence: current.sequence + 1 });
    assert.equal(wrapper.getLatestResult(), null);
    result(worker, current, { engineEpoch: 4 });
    assert.equal(wrapper.getLatestResult().sourceTiming.source, 'current');
    wrapper.sendFrame(new Uint8Array(4), 3, { sourceTiming: { source: 'stale_engine' } });
    result(worker, worker.last('frame'), { engineEpoch: 3 });
    assert.equal(wrapper.getLatestResult().sourceTiming, null);
    for (const options of [undefined, { sourceTiming: null }]) {
        wrapper.sendFrame(new Uint8Array(4), 4, options);
        result(worker, worker.last('frame'), { engineEpoch: 4, sourceTiming: { source: 'worker_cannot_supply_it' } });
        assert.equal(wrapper.getLatestResult().sourceTiming, null);
    }
});

test('Input recorder forwards timing and transfer options to the actual wrapper', async t => {
    const { wrapper, worker } = await loaded();
    t.after(() => wrapper.dispose());
    worker.addEventListener = () => {};
    worker.removeEventListener = () => {};
    const timing = { source: 'video_capture_time', timestampS: 1, physicalExposureVerified: false };
    const app = { running: true, _processWidth: 2, _processHeight: 2, _lastConfigParams: {},
        activeConfig: 'mobile_default', calibrationProvenance: {}, lastFrameTiming: timing,
        imu: { getDiagnostics: () => ({}) }, vio: wrapper, resetVIO() {} };
    const recorder = await fs.readFile(new URL('../../scripts/dev/android-input-record.mjs', import.meta.url), 'utf8');
    const begin = recorder.slice(recorder.indexOf('function begin('), recorder.indexOf('\nexport async function drainCapture'));
    const context = vm.createContext({ Uint8Array, structuredClone, performance, copyLowFeatureMetadata,
        btoa: value => Buffer.from(value, 'binary').toString('base64'),
        setTimeout: () => 1, clearTimeout() {}, setInterval: () => 1, clearInterval() {},
        window: { __mobileSLAM: { app, diagnostics: () => ({}) } } });
    vm.runInContext(begin, context);
    context.begin(60, 2, 'timing-options-fixture', false);
    const pixels = new Uint8Array([10, 20, 30, 40]);
    assert.equal(app.vio.sendFrame(pixels, 1, { transferOwnership: true, sourceTiming: timing }), true);
    assert.equal(pixels.byteLength, 0, 'Recorder preserves the explicit ownership option');
    timing.source = 'caller_mutated';
    result(worker, worker.last('frame'));
    assert.equal(wrapper.getLatestResult().sourceTiming.source, 'video_capture_time');
    assert.equal(context.window.__mobileSLAMInputCapture.record.acceptedFrameMetadata[0].sourceTiming.source, 'video_capture_time');
});

test('A postMessage rejection drops its timing snapshot before the next accepted request', async t => {
    const { wrapper, worker } = await loaded();
    t.after(() => wrapper.dispose());
    worker.postMessage = () => { throw new Error('fixture transfer rejected'); };
    assert.equal(wrapper.sendFrame(new Uint8Array(4), 1, { sourceTiming: { source: 'post_rejected' } }), false);
    assert.equal(wrapper._activeFrame, null);
    worker.postMessage = FakeWorker.prototype.postMessage;
    wrapper.sendFrame(new Uint8Array(4), 2);
    result(worker, worker.last('frame'));
    assert.equal(wrapper.getLatestResult().sourceTiming, null);
});

test('Wrapper rejects future pose timestamps and clears maps without rejecting separate past pose time', async t => {
    const { wrapper, worker } = await loaded();
    t.after(() => wrapper.dispose());
    wrapper.sendFrame(new Uint8Array(4), 1);
    result(worker, worker.last('frame'), { poseTimestamp: 0.99 });
    assert.equal(wrapper.getLatestResult().poseTimestamp, 0.99);
    assert.equal(wrapper.getLatestResult().poseValid, true);
    assert.equal(wrapper.getMapPoints().count, 1);
    wrapper.sendFrame(new Uint8Array(4), 2);
    const pending = worker.last('frame');
    result(worker, pending, { poseTimestamp: 2.1, reason: 'tracking', engineReason: 'original_engine_reason' });
    const row = wrapper.getLatestResult();
    assert.equal(row.inputTimestamp, 2);
    assert.equal(row.requestId, pending.requestId);
    assert.equal(row.clientEpoch, pending.clientEpoch);
    assert.equal(row.sequence, pending.sequence);
    assert.equal(row.engineEpoch, 1);
    assert.equal(row.pose, null);
    assert.equal(row.poseTimestamp, null);
    assert.equal(row.poseValid, false);
    assert.equal(row.poseFresh, false);
    assert.equal(row.poseAgeSeconds, null);
    assert.equal(row.reason, 'future_pose_timestamp');
    assert.equal(row.engineReason, 'original_engine_reason');
    assert.equal(row.mapPoints, null);
    assert.equal(row.mapPointCount, 0);
    assert.equal(wrapper.getMapPoints().count, 0);
});

test('Transport errors suppress immediate dispatch while successful pose-less loss still notifies', async () => {
    const { wrapper, worker } = await loaded();
    const rows = [];
    wrapper.onFrameComplete = row => rows.push(row);
    wrapper.sendFrame(new Uint8Array(4), 1);
    result(worker, worker.last('frame'), { poseFresh: false, poseValid: false, reason: 'missing_bracket' });
    assert.equal(rows.length, 1);

    wrapper.sendFrame(new Uint8Array(4), 2);
    result(worker, worker.last('frame'), { inputTimestamp: 99 });
    assert.equal(rows.length, 1, 'Timestamp mismatch is a transport error');

    wrapper.sendFrame(new Uint8Array(4), 3);
    worker.reply(worker.last('frame'), { type: 'result', success: false, error: 'failed',
        data: { inputTimestamp: 3, engineEpoch: 1, reason: 'worker_exception' } });
    assert.equal(rows.length, 1, 'Unsuccessful Worker result cannot immediately dispatch');

    wrapper.sendFrame(new Uint8Array(4), 4);
    result(worker, worker.last('frame'), { engineEpoch: 0 });
    assert.equal(rows.length, 1, 'Regressing engine epoch cannot immediately dispatch');
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

test('Only an explicitly owned full grayscale view transfers without another copy', async () => {
    const { wrapper, worker } = await loaded();
    const owned = new Uint8Array([10, 20, 30, 40]);
    assert.equal(wrapper.sendFrame(owned, 1, { transferOwnership: true }), true);
    assert.equal(owned.byteLength, 0, 'The owned image must transfer directly');
    assert.deepEqual(Array.from(new Uint8Array(worker.last('frame').data.gray)), [10, 20, 30, 40]);
    result(worker, worker.last('frame'));

    const reusable = new Uint8Array([1, 2, 3, 4]);
    assert.equal(wrapper.sendFrame(reusable, 2), true);
    reusable[0] = 99;
    assert.equal(reusable.byteLength, 4, 'The default path retains reusable camera storage');
    assert.deepEqual(Array.from(new Uint8Array(worker.last('frame').data.gray)), [1, 2, 3, 4]);
    result(worker, worker.last('frame'));

    const backing = new Uint8Array([9, 5, 6, 7, 8, 9]);
    assert.equal(wrapper.sendFrame(backing.subarray(1, 5), 3, { transferOwnership: true }), true);
    assert.equal(backing.byteLength, 6, 'A partial view cannot transfer unrelated bytes');
    assert.deepEqual(Array.from(new Uint8Array(worker.last('frame').data.gray)), [5, 6, 7, 8]);
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

function workerFixture(clock = {}) {
    const replies = [];
    const context = vm.createContext({ self: { postMessage: message => replies.push(message) },
        Float64Array, Uint8Array, ArrayBuffer, console, performance, setTimeout, clearTimeout, ...clock });
    vm.runInContext(workerSource + `\nglobalThis.audit = {
        appendIMU, drainIMUToWasm,
        memory(value, heap, count) { return new WorkerSharedMemory(value, heap, count); },
        stats() { return { ...imuDrops, size: imuRingWriteIdx - imuRingReadIdx }; },
        enableLowFeature() { lowFeatureEnabled = true; },
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

function deterministicClock() {
    let now = 0, nextId = 0;
    const timers = new Map();
    return {
        performance: { timeOrigin: 0, now: () => now },
        setTimeout(callback, delay) { const id = ++nextId; timers.set(id, { callback, at: now + delay }); return id; },
        clearTimeout(id) { timers.delete(id); },
        advance(ms) {
            const target = now + ms;
            for (;;) {
                const due = [...timers].filter(([, timer]) => timer.at <= target).sort((a, b) => a[1].at - b[1].at)[0];
                if (!due) break;
                now = due[1].at; timers.delete(due[0]); due[1].callback();
            }
            now = target;
        },
        get timers() { return timers.size; },
    };
}

function imuSamples(timestamps) {
    return new Float64Array(timestamps.flatMap(timestamp => [timestamp, 0, 0, 9.81, 0, 0, 0]));
}

const cameraPose = [1,0,0,0,0,1,0,0,0,0,1,1,0,0,0,1];
function inertialPrediction(timestamp = 1, engineEpoch = 1, anchorTimestamp = .9) {
    return { schema: 'mobile-slam-inertial-prediction-v1', mode: 'inertial-only', poseFrame: 'camera',
        engineEpoch, timestamp, anchorTimestamp, maxHorizonSeconds: .5, pose: [...cameraPose],
        visualSupported: false, metricTranslationSupported: false, positionUncertain: true };
}
function groundPlane(clientEpoch, sequence, timestamp = 1) {
    return { schema: 'mobile-slam-ground-plane-v1', kind: 'horizontal-surface-candidate', semanticClassification: 'unknown',
        state: 'observed', engineEpoch: 1, clientEpoch, sourceTimestamp: timestamp, sourceSequence: sequence,
        normal: [0,0,1], offset: 0, centroid: [0,0,0], corners: [-.5,-.5,0,.5,-.5,0,.5,.5,0,-.5,.5,0],
        supportCount: 25, sourcePointCount: 25, rms: 0, maxAgeSeconds: 1 };
}

test('Wrapper keeps low-feature extras off unless configure opts in explicitly', async () => {
    const { wrapper, worker } = await loaded();
    assert.equal('lowFeature' in worker.last('configure').data, false);
    wrapper.sendFrame(new Uint8Array(4), 1);
    const first = worker.last('frame');
    result(worker, first, { statusCode: 3, reason: 'empty_features', engineFrameTimestamp: 1, solverQualityReason: '',
        inertialPrediction: inertialPrediction(), groundPlane: groundPlane(first.clientEpoch, first.sequence) });
    assert.equal(wrapper.getLatestResult().inertialPrediction, null);
    assert.equal(wrapper.getLatestResult().groundPlane, null);
    const requests = worker.messages.length;
    assert.equal(wrapper.sendFrame(new Uint8Array(4), Number.NaN), false);
    assert.equal(worker.messages.slice(requests).some(message => message.type === 'invalidateLowFeature'), false);
    const reconfigured = wrapper.configure({ width: 2, height: 2, lowFeature: 'true' });
    worker.reply(worker.last('configure')); await reconfigured;
    wrapper.sendFrame(new Uint8Array(4), 1.1);
    result(worker, worker.last('frame'), { statusCode: 3, reason: 'empty_features', engineFrameTimestamp: 1.1, solverQualityReason: '',
        inertialPrediction: inertialPrediction(1.1) });
    assert.equal(wrapper.getLatestResult().inertialPrediction, null, 'only boolean true opts in');
    wrapper.dispose();
    const enabled = await loaded(undefined, { lowFeature: true });
    assert.equal(enabled.worker.last('configure').data.lowFeature, true);
    enabled.wrapper.sendFrame(new Uint8Array(4), 1);
    result(enabled.worker, enabled.worker.last('frame'), { statusCode: 3, reason: 'empty_features', engineFrameTimestamp: 1,
        solverQualityReason: '', inertialPrediction: inertialPrediction() });
    assert.deepEqual(enabled.wrapper.getLatestResult().inertialPrediction, inertialPrediction());
    enabled.wrapper.dispose();
});

test('Worker without low-feature opt-in neither reads prediction nor ground support', async () => {
    const f = workerFixture(); let reads = 0, supportReads = 0;
    f.audit.setEngine(mockEngine({ getFrameTimestamp: () => 1, isInitialized: () => true, getStatusCode: () => 3,
        getLastReason: () => 'empty_features', getLastSolverQualityReason: () => '',
        getInertialPrediction: () => { reads++; return JSON.stringify(inertialPrediction()); },
        invalidateInertialPrediction() {}, getGroundSupportPoints() { supportReads++; return 0; } }), f.heap);
    f.audit.appendIMU(imuSamples([.99,1.01]), 2);
    await f.context.self.onmessage({ data: request('frame', { gray: new ArrayBuffer(4), timestamp: 1 }) });
    const row = f.replies.at(-1).data;
    assert.equal(row.inertialPrediction, null); assert.equal(row.groundPlane, null);
    assert.equal(reads, 0); assert.equal(supportReads, 0);
    assert.equal(row.predictionReadMs ?? null, null); assert.equal(row.groundPlaneUpdateMs ?? null, null);
});

test('Wrapper transports a separately validated inertial prediction without promoting visual pose or map', async () => {
    const { wrapper, worker } = await loaded(undefined, { lowFeature: true });
    wrapper.sendFrame(new Uint8Array(4), 1);
    result(worker, worker.last('frame'), { statusCode: 3, reason: 'empty_features', engineFrameTimestamp: 1,
        solverQualityReason: '', inertialPrediction: inertialPrediction() });
    const row = wrapper.getLatestResult();
    assert.deepEqual(row.inertialPrediction, inertialPrediction());
    assert.equal(row.groundPlane, null);
    assert.equal(row.poseFresh, false); assert.equal(row.poseValid, false); assert.equal(row.pose, null);
    assert.equal(row.statusCode, 3); assert.equal(row.trackingState, 'lost');
    assert.deepEqual(wrapper.getMapPoints(), { points: null, count: 0 });
    wrapper.dispose();
});

test('Wrapper strips malformed, uncorrelated, forbidden and failed prediction payloads from raw result fields', async () => {
    for (const mutate of [
        data => { data.inertialPrediction.engineEpoch = 2; },
        data => { data.inertialPrediction.timestamp = 1.001; },
        data => { data.inertialPrediction.anchorTimestamp = .49; },
        data => { data.inertialPrediction.pose[0] = NaN; },
        data => { data.inertialPrediction.pose[0] = 2; },
        data => { data.inertialPrediction.confidence = 1; },
        data => { data.statusCode = 2; },
        data => { data.reason = 'invalid_pose'; },
        data => { data.solverQualityReason = 'relative_translation_threshold'; },
        data => { delete data.solverQualityReason; },
        data => { data.engineFrameTimestamp = .99; },
        data => { data.motionDiscontinuity = {}; },
        data => { data.imuIntervalLoss = {}; },
    ]) {
        const { wrapper, worker } = await loaded(undefined, { lowFeature: true });
        wrapper.sendFrame(new Uint8Array(4), 1);
        const data = { statusCode: 3, reason: 'empty_features', engineFrameTimestamp: 1,
            solverQualityReason: '', inertialPrediction: inertialPrediction(), groundPlane: { injected: true } };
        mutate(data); result(worker, worker.last('frame'), data);
        assert.equal(wrapper.getLatestResult().inertialPrediction, null);
        assert.equal(wrapper.getLatestResult().groundPlane, null);
        wrapper.dispose();
    }
});

test('Wrapper validates floor source identity and keeps late/reset metadata out of the current lifecycle', async () => {
    const { wrapper, worker } = await loaded(undefined, { lowFeature: true });
    wrapper.sendFrame(new Uint8Array(4), 1);
    const first = worker.last('frame'), plane = groundPlane(first.clientEpoch, first.sequence);
    result(worker, first, { reason: 'tracking', engineFrameTimestamp: 1, solverQualityReason: '', pose: new Float64Array(cameraPose), groundPlane: plane });
    assert.deepEqual(wrapper.getLatestResult().groundPlane, plane);
    wrapper.sendFrame(new Uint8Array(4), 1.1);
    const second = worker.last('frame');
    result(worker, first, { groundPlane: plane, inertialPrediction: inertialPrediction() });
    assert.equal(wrapper.workerBusy, true);
    result(worker, second, { statusCode: 3, reason: 'empty_features', engineFrameTimestamp: 1.1,
        solverQualityReason: '', inertialPrediction: inertialPrediction(1.1), groundPlane: { ...plane, state: 'retained' } });
    assert.equal(wrapper.getLatestResult().groundPlane.sourceTimestamp, 1);
    wrapper.reset(); worker.reply(worker.last('reset')); await Promise.resolve();
    assert.equal(wrapper.getLatestResult(), null);
    result(worker, second, { inertialPrediction: inertialPrediction(1.1), groundPlane: plane });
    assert.equal(wrapper.getLatestResult(), null);
    wrapper.dispose();
});

test('Locally rejected IMU fences an in-flight result and requires a later clean visual frame to restore extras', async () => {
    const { wrapper, worker } = await loaded(undefined, { lowFeature: true });
    const lost = timestamp => ({ statusCode: 3, reason: 'empty_features', engineFrameTimestamp: timestamp,
        solverQualityReason: '', inertialPrediction: inertialPrediction(timestamp, 1, 1) });
    wrapper.sendFrame(new Uint8Array(4), 1);
    result(worker, worker.last('frame'), lost(1));
    wrapper.sendFrame(new Uint8Array(4), 1.1);
    const inFlight = worker.last('frame');
    assert.equal(wrapper.sendIMU(new Float64Array([1.1,NaN,0,9.81,0,0,0]), 1), false);
    const invalidation = worker.last('invalidateLowFeature');
    assert.ok(invalidation); assert.equal(wrapper.getLatestResult().inertialPrediction, null);
    worker.reply(invalidation); await Promise.resolve();
    result(worker, inFlight, lost(1.1));
    assert.equal(wrapper.getLatestResult().inertialPrediction, null);
    wrapper.sendFrame(new Uint8Array(4), 1.2); result(worker, worker.last('frame'), lost(1.2));
    assert.equal(wrapper.getLatestResult().inertialPrediction, null);
    wrapper.sendFrame(new Uint8Array(4), 1.3);
    result(worker, worker.last('frame'), { reason: 'tracking', engineFrameTimestamp: 1.3,
        solverQualityReason: '', pose: new Float64Array(cameraPose) });
    assert.equal(wrapper.getLatestResult().poseFresh, true);
    wrapper.sendFrame(new Uint8Array(4), 1.4); result(worker, worker.last('frame'), lost(1.4));
    assert.ok(wrapper.getLatestResult().inertialPrediction);
    wrapper.dispose();
});

test('Ordinary busy drops preserve extras but malformed local frames send predictor-only invalidation', async () => {
    const { wrapper, worker } = await loaded(undefined, { lowFeature: true });
    wrapper.sendFrame(new Uint8Array(4), 1);
    result(worker, worker.last('frame'), { statusCode: 3, reason: 'empty_features', engineFrameTimestamp: 1,
        solverQualityReason: '', inertialPrediction: inertialPrediction() });
    wrapper.sendFrame(new Uint8Array(4), 1.1);
    assert.equal(wrapper.sendFrame(new Uint8Array(4), 1.2), false);
    assert.equal(worker.last('invalidateLowFeature'), undefined);
    assert.ok(wrapper.getLatestResult().inertialPrediction);
    assert.equal(wrapper.sendFrame(null, 1.2), false);
    assert.ok(worker.last('invalidateLowFeature'));
    assert.equal(wrapper.getLatestResult().inertialPrediction, null);
    worker.reply(worker.last('invalidateLowFeature'));
    wrapper.dispose();
});

test('A frame postMessage failure invalidates the shadow before a later blank frame can resume it', async t => {
    const { wrapper, worker } = await loaded(undefined, { lowFeature: true }); t.after(() => wrapper.dispose());
    wrapper.sendFrame(new Uint8Array(4), 1);
    result(worker, worker.last('frame'), { statusCode: 3, reason: 'empty_features', engineFrameTimestamp: 1,
        solverQualityReason: '', inertialPrediction: inertialPrediction() });
    const postMessage = worker.postMessage;
    worker.postMessage = function(message, transfer) {
        if (message.type === 'frame') throw new Error('fixture frame transfer failure');
        return postMessage.call(this, message, transfer);
    };
    assert.equal(wrapper.sendFrame(new Uint8Array(4), 1.1), false);
    assert.ok(worker.last('invalidateLowFeature'));
    worker.reply(worker.last('invalidateLowFeature')); await Promise.resolve();
    worker.postMessage = postMessage;
    wrapper.sendFrame(new Uint8Array(4), 1.2);
    result(worker, worker.last('frame'), { statusCode: 3, reason: 'empty_features', engineFrameTimestamp: 1.2,
        solverQualityReason: '', inertialPrediction: inertialPrediction(1.2) });
    assert.equal(wrapper.getLatestResult().inertialPrediction, null);
    assert.equal(wrapper.getLatestResult().poseFresh, false);
});

test('Worker prediction capability requires both getter and invalidation API and exact clear Lost context', async () => {
    for (const mode of ['valid', 'old', 'getter-only', 'warning', 'frame-mismatch', 'forbidden']) {
        const f = workerFixture(); let reads = 0;
        const core = mockEngine({ getFrameTimestamp: () => mode === 'frame-mismatch' ? .99 : 1,
            isInitialized: () => true, getStatusCode: () => 3,
            getLastReason: () => mode === 'forbidden' ? 'invalid_pose' : 'empty_features',
            getLastSolverQualityReason: () => mode === 'warning' ? 'relative_translation_threshold' : '' });
        if (mode !== 'old') core.getInertialPrediction = () => { reads++; return JSON.stringify(inertialPrediction()); };
        if (mode !== 'getter-only' && mode !== 'old') core.invalidateInertialPrediction = () => {};
        f.audit.setEngine(core, f.heap); f.audit.enableLowFeature(); f.audit.appendIMU(imuSamples([.99,1.01]), 2);
        await f.context.self.onmessage({ data: request('frame', { gray: new ArrayBuffer(4), timestamp: 1 }) });
        const row = f.replies.at(-1).data;
        if (mode === 'valid') {
            assert.deepEqual(JSON.parse(JSON.stringify(row.inertialPrediction)), inertialPrediction());
            assert.equal(reads, 1); assert.ok(Number.isFinite(row.predictionReadMs));
        } else { assert.equal(row.inertialPrediction, null, mode); assert.equal(reads, 0, mode); }
        assert.equal(row.poseFresh, false); assert.equal(row.pose, null); assert.equal(row.mapPointCount, 0);
    }
});

test('Missing bracket invalidates only the predictor even though the engine frame call is skipped', async () => {
    const clock = deterministicClock(), f = workerFixture(clock);
    let anchor = true, calls = 0, invalidations = 0, resets = 0, timestamp = .9;
    f.audit.setEngine(mockEngine({ processFrame(...args) { calls++; timestamp = args[5]; return false; },
        getFrameTimestamp: () => timestamp, isInitialized: () => true, getStatusCode: () => 3,
        getLastReason: () => 'empty_features', getLastSolverQualityReason: () => '',
        getInertialPrediction: () => JSON.stringify(anchor ? inertialPrediction(timestamp) : null),
        invalidateInertialPrediction() { anchor = false; invalidations++; }, reset() { resets++; } }), f.heap);
    const missing = f.context.self.onmessage({ data: request('frame', { gray: new ArrayBuffer(4), timestamp: 1 }) });
    clock.advance(41); await missing;
    assert.equal(calls, 0); assert.equal(resets, 0); assert.ok(invalidations > 0);
    assert.equal(f.replies.at(-1).data.inertialPrediction, null);
    f.audit.appendIMU(imuSamples([1.09,1.11]), 2);
    await f.context.self.onmessage({ data: request('frame', { gray: new ArrayBuffer(4), timestamp: 1.1 }, 2) });
    assert.equal(calls, 1); assert.equal(f.replies.at(-1).data.inertialPrediction, null);
});

test('Worker invalidation RPC quarantines an earlier waiting frame without resetting the metric engine', async () => {
    const clock = deterministicClock(), f = workerFixture(clock);
    let anchor = true, resets = 0, calls = 0, timestamp = 1;
    f.audit.setEngine(mockEngine({ processFrame(...args) { calls++; timestamp = args[5]; if (calls === 1) anchor = true; return false; },
        getFrameTimestamp: () => timestamp, isInitialized: () => true, getStatusCode: () => 3,
        getLastReason: () => 'empty_features', getLastSolverQualityReason: () => '',
        getInertialPrediction: () => JSON.stringify(anchor ? inertialPrediction(timestamp) : null),
        invalidateInertialPrediction() { anchor = false; }, reset() { resets++; } }), f.heap);
    const old = f.context.self.onmessage({ data: request('frame', { gray: new ArrayBuffer(4), timestamp: 1 }) });
    await f.context.self.onmessage({ data: request('invalidateLowFeature', undefined, 2) });
    assert.equal(f.replies.at(-1).success, true);
    await f.context.self.onmessage({ data: request('imu', { imuData: imuSamples([.99,1.01]).buffer, count: 2 }, 3) });
    await old;
    assert.equal(f.replies.at(-1).data.inertialPrediction, null);
    assert.equal(anchor, false); assert.equal(resets, 0);
    f.audit.appendIMU(imuSamples([1.09,1.11]), 2);
    await f.context.self.onmessage({ data: request('frame', { gray: new ArrayBuffer(4), timestamp: 1.1 }, 4) });
    assert.equal(calls, 2); assert.equal(f.replies.at(-1).data.inertialPrediction, null);
});

test('Worker malformed IMU received during a frame wait fences that older frame before publication', async () => {
    const clock = deterministicClock(), f = workerFixture(clock);
    let anchor = true, invalidations = 0;
    f.audit.setEngine(mockEngine({ processFrame() { anchor = true; return false; }, getFrameTimestamp: () => 1,
        isInitialized: () => true, getStatusCode: () => 3, getLastReason: () => 'empty_features', getLastSolverQualityReason: () => '',
        getInertialPrediction: () => JSON.stringify(anchor ? inertialPrediction() : null),
        invalidateInertialPrediction() { anchor = false; invalidations++; } }), f.heap);
    const old = f.context.self.onmessage({ data: request('frame', { gray: new ArrayBuffer(4), timestamp: 1 }) });
    await f.context.self.onmessage({ data: request('imu', { imuData: imuSamples([NaN]).buffer, count: 1 }, 2) });
    assert.equal(f.replies.at(-1).success, false);
    await f.context.self.onmessage({ data: request('imu', { imuData: imuSamples([.99,1.01]).buffer, count: 2 }, 3) });
    await old;
    assert.equal(f.replies.at(-1).data.inertialPrediction, null);
    assert.equal(anchor, false); assert.ok(invalidations >= 2);
});

test('Worker rejects invalid IMU and transport loss without permitting predictor reads or stale anchor reuse', async () => {
    for (const loss of ['invalid', 'overflow']) {
        const f = workerFixture(); let anchor = true, reads = 0, resets = 0;
        f.audit.setEngine(mockEngine({ getFrameTimestamp: () => 1.1, isInitialized: () => true, getStatusCode: () => 3,
            getLastReason: () => 'empty_features', getLastSolverQualityReason: () => '',
            getInertialPrediction() { reads++; return JSON.stringify(anchor ? inertialPrediction(1.1) : null); },
            invalidateInertialPrediction() { anchor = false; }, reset() { resets++; } }), f.heap);
        if (loss === 'invalid') { assert.equal(f.audit.appendIMU(imuSamples([NaN]), 1), false); f.audit.appendIMU(imuSamples([1.09,1.11]), 2); }
        else f.audit.appendIMU(imuSamples(Array.from({ length: 1025 }, (_, i) => .9 + i * .0002)), 1025);
        await f.context.self.onmessage({ data: request('frame', { gray: new ArrayBuffer(4), timestamp: 1.1 }) });
        const row = f.replies.at(-1).data;
        assert.equal(anchor, false); assert.equal(row.inertialPrediction, null); assert.equal(row.groundPlane, null);
        if (loss === 'overflow') { assert.equal(reads, 0); assert.equal(resets, 1); assert.equal(row.reason, 'imu_transport_loss'); }
    }
});

test('Worker current support fitting is bounded and Lost retains its source without reading or fitting again', async () => {
    const f = workerFixture(); let timestamp = 1, visual = true, reads = 0;
    const points = []; for (let x = -2; x <= 2; x++) for (let y = -2; y <= 2; y++) points.push(x * .25, y * .25, 0);
    f.context.groundFixture = points; f.context.poseFixture = cameraPose;
    f.audit.setEngine(mockEngine({ processFrame(...args) { timestamp = args[5]; return visual; }, getFrameTimestamp: () => timestamp,
        isInitialized: () => true, getStatusCode: () => visual ? 2 : 3, getLastReason: () => visual ? 'tracking' : 'empty_features',
        getLastSolverQualityReason: () => '', getPoseTimestamp: () => timestamp, getPoseFresh: () => visual, getPoseValid: () => visual,
        getGroundSupportPoints(ptr, maxCount) { reads++; assert.equal(maxCount, 512); return 25; } }), f.heap); f.audit.enableLowFeature();
    vm.runInContext('memPose.read = () => new Float64Array(poseFixture); memGroundSupport = { ptr: 0, read: count => new Float64Array(groundFixture.slice(0, count)) };', f.context);
    f.audit.appendIMU(imuSamples([.99,1.01]), 2);
    await f.context.self.onmessage({ data: request('frame', { gray: new ArrayBuffer(4), timestamp: 1 }) });
    const observed = f.replies.at(-1).data;
    assert.equal(observed.groundPlane?.state, 'observed');
    assert.equal(observed.groundSupportPointCount, 25);
    assert.ok(Number.isFinite(observed.groundFitMs)); assert.ok(Number.isFinite(observed.groundSupportReadMs));
    visual = false; f.audit.appendIMU(imuSamples([1.09,1.11]), 2);
    await f.context.self.onmessage({ data: request('frame', { gray: new ArrayBuffer(4), timestamp: 1.1 }, 2) });
    const retained = f.replies.at(-1).data;
    assert.equal(reads, 1); assert.equal(retained.groundPlane?.state, 'retained');
    assert.equal(retained.groundSupportPointCount, null);
    assert.equal(retained.groundPlane.sourceTimestamp, 1); assert.equal(retained.groundPlane.sourceSequence, 1);
    assert.equal(retained.groundFitMs, null); assert.equal(retained.groundSupportReadMs, null);
    await f.context.self.onmessage({ data: request('imu', { imuData: imuSamples([NaN]).buffer, count: 1 }, 3) });
    f.audit.appendIMU(imuSamples([1.19,1.21]), 2);
    await f.context.self.onmessage({ data: request('frame', { gray: new ArrayBuffer(4), timestamp: 1.2 }, 4) });
    assert.equal(f.replies.at(-1).data.groundPlane, null, 'Input rejection clears retained geometry');
    await f.context.self.onmessage({ data: request('reset', undefined, 1, 1) });
    f.audit.appendIMU(imuSamples([1.29,1.31]), 2);
    await f.context.self.onmessage({ data: request('frame', { gray: new ArrayBuffer(4), timestamp: 1.3 }, 2, 1) });
    assert.equal(f.replies.at(-1).data.groundPlane, null);
});

test('Worker observed slow or jittering cadence admits a real bracket at 52ms and fixes the wait budget at entry', async () => {
    for (const [times, budget] of [[ [.90, .93, .96, .99], 60 ], [ [.906, .922, .938, .99], 80 ]]) {
        const clock = deterministicClock();
        const { context, audit, heap, replies } = workerFixture(clock);
        let calls = 0;
        audit.setEngine(mockEngine({ processFrame() { calls++; return false; } }), heap);
        audit.appendIMU(imuSamples(times), 4);
        const frame = context.self.onmessage({ data: request('frame', { gray: new ArrayBuffer(4), timestamp: 1 }) });
        clock.advance(52);
        audit.appendIMU(imuSamples([1.02]), 1);
        await frame;
        const result = replies.at(-1).data;
        assert.equal(calls, 1);
        assert.equal(result.imuWaitMs, 52);
        assert.ok(Math.abs(result.imuWaitBudgetMs - budget) < 1e-9);
        assert.equal(result.imuCount, 5);
        assert.equal(clock.timers, 0);
    }
});

test('Worker default, short history and steady 60Hz waits keep 40ms and never extrapolate after their deadline', async () => {
    for (const times of [[], [.7, .8, .9], [.90, .916, .932, .948]]) {
        const clock = deterministicClock();
        const { context, audit, heap, replies } = workerFixture(clock);
        let calls = 0;
        audit.setEngine(mockEngine({ processFrame() { calls++; return false; } }), heap);
        if (times.length) audit.appendIMU(imuSamples(times), times.length);
        const frame = context.self.onmessage({ data: request('frame', { gray: new ArrayBuffer(4), timestamp: 1 }) });
        clock.advance(41);
        audit.appendIMU(imuSamples([1.01]), 1);
        await frame;
        const result = replies.at(-1).data;
        assert.equal(calls, 0);
        assert.equal(result.reason, 'missing_bracket');
        assert.equal(result.imuWaitBudgetMs, 40);
        assert.equal(result.poseFresh, false);
        assert.equal(result.pipeline, null);
        assert.equal(clock.timers, 0);
    }
});

test('Worker adaptive deadline is capped at 80ms and cannot be extended by later samples', async () => {
    const clock = deterministicClock();
    const { context, audit, heap, replies } = workerFixture(clock);
    let calls = 0;
    audit.setEngine(mockEngine({ processFrame() { calls++; return false; } }), heap);
    audit.appendIMU(imuSamples([.6, .7, .8, .9]), 4);
    const frame = context.self.onmessage({ data: request('frame', { gray: new ArrayBuffer(4), timestamp: 1 }) });
    clock.advance(79);
    audit.appendIMU(imuSamples([.95]), 1);
    clock.advance(1);
    await frame;
    const result = replies.at(-1).data;
    assert.equal(calls, 0);
    assert.equal(result.reason, 'missing_bracket');
    assert.equal(result.imuWaitBudgetMs, 80);
    assert.equal(result.imuWaitMs, 80);
});

test('Worker reset cancels adaptive wait, clears cadence, and preserves new frame ownership', async () => {
    const clock = deterministicClock();
    const { context, audit, heap, replies } = workerFixture(clock);
    let calls = 0, resets = 0;
    audit.setEngine(mockEngine({ processFrame() { calls++; return false; }, reset() { resets++; } }), heap);
    audit.appendIMU(imuSamples([.6, .7, .8, .9]), 4);
    const oldFrame = context.self.onmessage({ data: request('frame', { gray: new ArrayBuffer(4), timestamp: 1 }) });
    clock.advance(20);
    await context.self.onmessage({ data: request('reset', undefined, 1, 1) });
    await oldFrame;
    const cancelled = replies.find(reply => reply.type === 'result');
    assert.equal(cancelled.data.reason, 'stale_epoch');
    assert.equal(cancelled.data.imuWaitBudgetMs, 80);
    assert.equal(cancelled.data.pipeline, null);
    assert.equal(resets, 1);
    assert.equal(clock.timers, 0);
    const newFrame = context.self.onmessage({ data: request('frame', { gray: new ArrayBuffer(4), timestamp: 2 }, 2, 1) });
    await context.self.onmessage({ data: request('imu', { imuData: imuSamples([2.01]).buffer, count: 1 }, 3, 0) });
    assert.equal(replies.at(-1).error, 'stale_epoch');
    clock.advance(41);
    await newFrame;
    assert.equal(replies.at(-1).data.imuWaitBudgetMs, 40);
    assert.equal(replies.at(-1).data.reason, 'missing_bracket');
    assert.equal(calls, 0);
});

test('Worker rejected IMU timestamps do not influence adaptive cadence or create brackets', async () => {
    const clock = deterministicClock();
    const { context, audit, heap, replies } = workerFixture(clock);
    let calls = 0;
    audit.setEngine(mockEngine({ processFrame() { calls++; return false; } }), heap);
    assert.equal(audit.appendIMU(imuSamples([-1]), 1), false);
    audit.appendIMU(imuSamples([.90, .916, .932, .948]), 4);
    assert.equal(audit.appendIMU(imuSamples([.948, .7, NaN, Infinity]), 4), false);
    assert.equal(audit.appendIMU(new Float64Array([1.02,NaN,0,9.81,0,0,0]), 1), false);
    const frame = context.self.onmessage({ data: request('frame', { gray: new ArrayBuffer(4), timestamp: 1 }) });
    clock.advance(40);
    await frame;
    assert.equal(replies.at(-1).data.reason, 'missing_bracket');
    assert.equal(replies.at(-1).data.imuWaitBudgetMs, 40);
    assert.equal(calls, 0);
});

test('Worker bounded cadence history forgets old slowdown and resets across sensor gaps', async () => {
    for (const discontinuity of [false, true]) {
        const clock = deterministicClock();
        const { context, audit, heap, replies } = workerFixture(clock);
        audit.setEngine(mockEngine(), heap);
        audit.appendIMU(imuSamples([.6, .7, .8, .9]), 4);
        const samples = discontinuity ? [2] : Array.from({ length: 16 }, (_, i) => .9 + (i + 1) * .016);
        audit.appendIMU(imuSamples(samples), samples.length);
        const timestamp = samples.at(-1) + .01;
        const frame = context.self.onmessage({ data: request('frame', { gray: new ArrayBuffer(4), timestamp }) });
        clock.advance(40);
        await frame;
        assert.equal(replies.at(-1).data.imuWaitBudgetMs, 40);
        assert.equal(replies.at(-1).data.reason, 'missing_bracket');
    }
});

test('Worker pipeline is frame-correlated, optional, and read exactly once per actual engine result', async () => {
    const { context, audit, heap, replies } = workerFixture();
    let reads = 0;
    let pipeline = { schema: 'mobile-slam-pipeline-v1', frameTimestamp: 1, engineEpoch: 1, solver: null };
    audit.setEngine(mockEngine({ getPipelineDiagnostics() { reads++; return typeof pipeline === 'string' ? pipeline : JSON.stringify(pipeline); } }), heap);
    audit.appendIMU(imuSamples([1, 1.01]), 2);
    await context.self.onmessage({ data: request('frame', { gray: new ArrayBuffer(4), timestamp: 1 }, 1) });
    assert.deepEqual(JSON.parse(JSON.stringify(replies.at(-1).data.pipeline)), pipeline);
    assert.equal(reads, 1);
    for (const invalid of [
        { ...pipeline, frameTimestamp: 0 }, { ...pipeline, engineEpoch: 2 }, '{bad', null,
    ]) {
        pipeline = invalid;
        await context.self.onmessage({ data: request('frame', { gray: new ArrayBuffer(4), timestamp: 1 }, reads + 1) });
        assert.equal(replies.at(-1).data.pipeline, null);
    }
    assert.equal(reads, 5);
});

test('Worker non-tracking statuses retain their raw reason and cannot publish finite fresh candidates', async () => {
    for(const statusCode of [3,99]) {
        const {context,audit,heap,replies}=workerFixture();
        audit.setEngine(mockEngine({processFrame(){return true},isInitialized(){return true},getStatusCode(){return statusCode},
            getLastReason(){return 'finite_but_not_tracking'},getPoseTimestamp(){return 1},getPoseFresh(){return true},getPoseValid(){return true}}),heap);
        audit.appendIMU(imuSamples([1,1.01]),2);
        await context.self.onmessage({data:request('frame',{gray:new ArrayBuffer(4),timestamp:1},1)});
        const latest=replies.at(-1).data;
        assert.equal(latest.statusCode,statusCode);
        assert.equal(latest.reason,'finite_but_not_tracking');
        assert.equal(latest.poseValid,false);
        assert.equal(latest.poseFresh,false);
        assert.equal(latest.pose,null);
    }
});

test('Worker publishes correlated pipeline-v2 and Limited quality, preserving explicit factor meanings', async () => {
    const {context,audit,heap,replies}=workerFixture(),pipeline=limitedPipeline();
    audit.setEngine(mockEngine({processFrame(){return true},isInitialized(){return true},getStatusCode(){return 5},
        getFrameTimestamp(){return 1},getLastReason(){return 'tracking_limited'},getPoseTimestamp(){return 1},
        getPoseFresh(){return true},getPoseValid(){return true},getPipelineDiagnostics(){return JSON.stringify(pipeline)}}),heap);
    audit.appendIMU(imuSamples([1,1.01]),2);
    await context.self.onmessage({data:request('frame',{gray:new ArrayBuffer(4),timestamp:1},1)});
    const latest=replies.at(-1).data;
    assert.equal(latest.trackingState,'tracking_limited');
    assert.equal(latest.poseFresh,true);
    assert.deepEqual(JSON.parse(JSON.stringify(latest.quality)),pipeline.quality);
    assert.equal(latest.pipeline.solver.eligibleFeatures,12,'Historical structural slots retain their old meaning');
    assert.equal(latest.pipeline.solver.structuralSlots,12);
    assert.equal(latest.pipeline.solver.currentMetricFactors,1);
    assert.equal(latest.pipeline.solver.currentProjectiveFactors,3);
});

test('Worker cannot certify Limited with missing/rank-deficient/unavailable or probabilistic quality', async () => {
    const cases=[
        p=>{delete p.quality},p=>{p.quality.rotationVisualSupported=false},p=>{p.quality.metricTranslationSupported=true},
        p=>{p.quality.visualRotationRank=2},p=>{p.quality.uncertaintyAvailable=false},p=>{p.quality.relativeCovariance6x6=null},
        p=>{p.quality.relativeCovariance6x6[0]=NaN},p=>{p.quality.limitedDurationS=null},
        p=>{p.quality.consistencyScoreIsProbabilistic=true},p=>{p.quality.uncertaintySource='unavailable'},
        p=>{p.quality.rotationConsistencyScore=100},p=>{p.solver.currentProjectiveFactors=2},
        p=>{p.quality.relativeCovarianceOrdering='unknown_units'},
    ];
    for(const mutate of cases) {
        const {context,audit,heap,replies}=workerFixture(),pipeline=limitedPipeline();mutate(pipeline);
        audit.setEngine(mockEngine({processFrame(){return true},isInitialized(){return true},getStatusCode(){return 5},
            getFrameTimestamp(){return 1},getLastReason(){return 'tracking_limited'},getPoseTimestamp(){return 1},
            getPoseFresh(){return true},getPoseValid(){return true},getPipelineDiagnostics(){return JSON.stringify(pipeline)}}),heap);
        audit.appendIMU(imuSamples([1,1.01]),2);
        await context.self.onmessage({data:request('frame',{gray:new ArrayBuffer(4),timestamp:1},1)});
        assert.equal(replies.at(-1).data.poseValid,false);
        assert.equal(replies.at(-1).data.poseFresh,false);
        assert.equal(replies.at(-1).data.pose,null);
    }
});

test('Pipeline-v2 keeps unrun solver null and unavailable uncertainty null without borrowing stale counts', async () => {
    const {context,audit,heap,replies}=workerFixture(),pipeline=limitedPipeline();
    pipeline.solver=null;
    Object.assign(pipeline.quality,{mode:'none',rotationVisualSupported:false,translationSupport:'none',fallbackUsed:false,recoveryMode:false,
        currentMetricFactors:0,currentProjectiveFactors:0,limitedDurationS:null,uncertaintyAvailable:false,uncertaintyAnchorTimestamp:null,
        relativeCovariance6x6:null,position3SigmaM:null,rotation3SigmaRad:null,rotationConsistencyScore:null,
        uncertaintyModel:'unavailable',uncertaintySource:'unavailable',visualRotationRank:null,reason:'not_run'});
    audit.setEngine(mockEngine({getPipelineDiagnostics(){return JSON.stringify(pipeline)}}),heap);
    audit.appendIMU(imuSamples([1,1.01]),2);
    await context.self.onmessage({data:request('frame',{gray:new ArrayBuffer(4),timestamp:1},1)});
    assert.equal(replies.at(-1).data.pipeline.schema,'mobile-slam-pipeline-v2');
    assert.equal(replies.at(-1).data.pipeline.solver,null);assert.equal(replies.at(-1).data.quality.relativeCovariance6x6,null);
    pipeline.quality.currentMetricFactors=1;
    await context.self.onmessage({data:request('frame',{gray:new ArrayBuffer(4),timestamp:1},2)});
    assert.equal(replies.at(-1).data.pipeline,null);assert.equal(replies.at(-1).data.quality,null);
});

test('Missing bracket after a Limited engine state publishes ordinary Lost with no stale quality alias', async () => {
    const clock=deterministicClock(),{context,audit,heap,replies}=workerFixture(clock);
    audit.setEngine(mockEngine({isInitialized(){return true},getStatusCode(){return 5},getLastReason(){return 'tracking_limited'}}),heap);
    audit.appendIMU(imuSamples([1.99]),1);
    const pending=context.self.onmessage({data:request('frame',{gray:new ArrayBuffer(4),timestamp:2},1)});
    clock.advance(40);await pending;
    const row=replies.at(-1).data;
    assert.equal(row.statusCode,3);assert.equal(row.trackingState,'lost');assert.equal(row.reason,'missing_bracket');
    assert.equal(row.poseFresh,false);assert.equal(row.pipeline,null);assert.equal(row.quality,null);
});

test('Worker invalid frame timestamp never enters bracket wait or the engine', async () => {
    for (const timestamp of [NaN, Infinity, -1]) {
        const clock = deterministicClock();
        const { context, audit, heap, replies } = workerFixture(clock);
        let calls = 0;
        audit.setEngine(mockEngine({ processFrame() { calls++; return false; } }), heap);
        await context.self.onmessage({ data: request('frame', { gray: new ArrayBuffer(4), timestamp }) });
        assert.equal(calls, 0);
        assert.equal(replies.at(-1).data.reason, 'invalid_frame');
        assert.equal(replies.at(-1).data.imuWaitMs, 0);
        assert.equal(replies.at(-1).data.imuWaitBudgetMs, null);
        assert.equal(clock.timers, 0);
    }
});

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
function fakeWasmModule(poseOffset = -0.01, mapPointCount = 0) {
    return asModule(`export default function () {
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
                this.frameTimestamp=ts; this.timestamp=ts+${poseOffset}; this.fresh=true;
                HEAPF64.set([1,0,0,1,0,1,0,2,0,0,1,3,0,0,0,1],pose/8); return true;
            }
            getEpoch() { return this.epoch; } getFrameTimestamp() { return this.frameTimestamp ?? -1; }
            getIMUEndpointTimestamp() { return this.fresh ? this.frameTimestamp : -1; }
            getPoseTimestamp() { return this.timestamp; } getPoseFresh() { return this.fresh; }
            getPoseValid() { return this.fresh; } isInitialized() { return this.fresh; }
            getFeaturePointCount() { return 20; } getStatusCode() { return this.fresh?2:1; }
            getLastReason() { return this.fresh?'tracking':'imu_interval_uncovered'; }
            getLastSolverIterations() { return 3; } getLastSolverTermination() { return 'CONVERGENCE'; }
            getMapPoints(ptr) { if (${mapPointCount}) HEAPF64.set([1,2,3],ptr/8); return ${mapPointCount}; }
            reset() { this.epoch++; this.fresh=false; } delete() {}
        }
    };
}`);
}
const fakeWasm = fakeWasmModule();

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

test('Actual Worker defaults low-feature off and reconfiguration clears a stale opt-in', async () => {
    const { worker, call } = realWorker();
    try {
        assert.equal((await call(request('init', { wasmPath: fakeWasm }, 1, 1))).success, true);
        let sequence = 2;
        for (const [requested, expected] of [[undefined, false], ['true', false], [1, false], [true, true], [undefined, false]]) {
            const data = { width: 2, height: 2, fx: 1, fy: 1, cx: 1, cy: 1, ...(requested === undefined ? {} : { lowFeature: requested }) };
            const configured = await call(request('configure', data, sequence, sequence)); sequence++;
            assert.equal(configured.success, true);
            assert.equal(configured.lowFeatureEnabled, expected, String(requested));
        }
    } finally { await worker.terminate(); }
});

test('Actual Worker rejects a future engine pose before publishing its map and retains engine reason', async () => {
    const { worker, call } = realWorker();
    try {
        assert.equal((await call(request('init', { wasmPath: fakeWasmModule(0.1, 1) }, 1, 1))).success, true);
        assert.equal((await call(request('configure', { width: 2, height: 2, fx: 1, fy: 1, cx: 1, cy: 1 }, 2, 2))).success, true);
        const imu = imuSamples([1, 1.01]);
        await call(request('imu', { imuData: imu.buffer, count: 2 }, 3, 2));
        const frame = await call(request('frame', { gray: new ArrayBuffer(4), timestamp: 1 }, 4, 2));
        assert.equal(frame.success, true, 'Publication rejection is still a completed frame');
        assert.equal(frame.requestId, 4);
        assert.equal(frame.sequence, 4);
        assert.equal(frame.clientEpoch, 2);
        assert.equal(frame.data.inputTimestamp, 1);
        assert.equal(frame.data.engineFrameTimestamp, 1);
        assert.equal(frame.data.engineEpoch, 1);
        assert.equal(frame.data.pose, null);
        assert.equal(frame.data.poseTimestamp, null);
        assert.equal(frame.data.poseFresh, false);
        assert.equal(frame.data.poseValid, false);
        assert.equal(frame.data.mapPoints, null);
        assert.equal(frame.data.mapPointCount, 0);
        assert.equal(frame.data.reason, 'future_pose_timestamp');
        assert.equal(frame.data.engineReason, 'tracking');
    } finally { await worker.terminate(); }
});

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

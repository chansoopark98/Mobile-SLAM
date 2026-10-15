import { loadBrowserSource } from './browser-source-loader.mjs';
import test from 'node:test';
import assert from 'node:assert/strict';
import fs from 'node:fs/promises';
import vm from 'node:vm';

const workerSource = await loadBrowserSource('vio-worker');
const wrapperSource = await loadBrowserSource('vio-wrapper');
const { VIOWrapper } = await import('data:text/javascript;base64,' + Buffer.from(wrapperSource).toString('base64'));
const snapshot = { enabled: true, epoch: 7, frameTimestamp: -1, opencvVersion: 'fixture-opencv', executionSeed: 19, cvThreads: 2,
    params: { width: 640, height: 480, min_dist: 30, solver_time: 0.06, lk_iterations: 20, projection_sqrt_info_00: 307.3 } };
const effective = { schema: 'mobile-slam-effective-config-v1', engineEpoch: 7, frameTimestamp: -1,
    opencvVersion: snapshot.opencvVersion, executionSeed: 19, cvThreads: 2, params: snapshot.params };

function workerFixture({ configured = true, missingAPI = false, enableThrows = false, disableThrows = false, readThrows = false, raw = JSON.stringify(snapshot) } = {}) {
    const messages = [], toggles = [], frames = [];
    const memory = new ArrayBuffer(1024 * 1024); let offset = 8, sequence = 0;
    const wasm = { HEAPU8: new Uint8Array(memory), HEAPF64: new Float64Array(memory),
        _malloc(size) { const pointer = offset; offset += Math.ceil(size / 8) * 8; return pointer; }, _free() {} };
    const engine = { capture: false, resets: 0,
        setDiagnosticCapture(enabled) { toggles.push(enabled); this.capture = enabled;
            if (enabled && enableThrows) throw new Error('fixture enable failed');
            if (!enabled && disableThrows) throw new Error('fixture disable failed'); },
        getFeatureDiagnostics() { if (readThrows) throw new Error('fixture read failed'); return raw; },
        getEpoch: () => 7, getFrameTimestamp: () => -1, getIMUEndpointTimestamp: () => -1,
        getPoseTimestamp: () => -1, getPoseFresh: () => false, getPoseValid: () => false,
        isInitialized: () => false, getStatusCode: () => 1, getLastReason: () => 'initializing',
        processFrame(image, width, height, imu, count, timestamp) { assert.equal(this.capture,false); frames.push({ count, timestamp }); return false; },
        reset() { this.resets++; } };
    if (missingAPI === 'setter') delete engine.setDiagnosticCapture;
    else if (missingAPI) delete engine.getFeatureDiagnostics;
    const context = vm.createContext({ Float64Array, Uint8Array, ArrayBuffer, performance, setTimeout, clearTimeout,
        self: { postMessage: message => messages.push(message) }, mockWasm: wasm, mockEngine: engine });
    vm.runInContext(workerSource + `\nwasm=mockWasm; engine=mockEngine; configured=${configured}; imageWidth=2; imageHeight=2; clientEpoch=1; engineEpoch=7; allocateBuffers();`, context);
    const call = async (type, data) => {
        const requestId = ++sequence;
        await context.self.onmessage({ data: { type, data, clientEpoch: 1, sequence, requestId } });
        return messages.find(message => message.requestId === requestId);
    };
    return { call, engine, toggles, frames, context };
}

test('Worker returns actual diagnostic parameters and disables capture without resetting IMU or epoch', async () => {
    const f = workerFixture();
    await f.call('imu', { imuData: new Float64Array([1,0,0,9.81,0,0,0,1.01,0,0,9.81,0,0,0]).buffer, count: 2 });
    const result = await f.call('getEffectiveConfig');
    assert.equal(result.success,true,result.error); assert.deepEqual(JSON.parse(JSON.stringify(result.data)),effective);
    assert.deepEqual(f.toggles,[true,false]); assert.equal(f.engine.capture,false); assert.equal(f.engine.resets,0);
    const frame = await f.call('frame', { gray: new ArrayBuffer(4), timestamp: 1 });
    assert.equal(frame.data.imuCount,2); assert.equal(frame.data.engineEpoch,7);
    assert.deepEqual(f.frames,[{count:2,timestamp:1}]);
});

test('Worker explicitly rejects unconfigured and unsupported diagnostic reads', async () => {
    for (const options of [{ configured: false }, { missingAPI: true }, { missingAPI: 'setter' }]) {
        const f = workerFixture(options); const reply = await f.call('getEffectiveConfig');
        assert.equal(reply.success,false); assert.match(reply.error,/configured|unavailable/i);
        assert.deepEqual(f.toggles,[]); assert.equal(f.engine.resets,0);
    }
});

test('Repeated profile reads observe current effective values without enabling persistent capture', async () => {
    const f = workerFixture();
    const first = await f.call('getEffectiveConfig');
    f.engine.getFeatureDiagnostics = () => JSON.stringify({ ...snapshot, params: { ...snapshot.params, min_dist: 20 } });
    const second = await f.call('getEffectiveConfig');
    assert.equal(first.data.params.min_dist,30); assert.equal(second.data.params.min_dist,20);
    assert.deepEqual(f.toggles,[true,false,true,false]); assert.equal(f.engine.capture,false); assert.equal(f.engine.resets,0);
});

test('Worker always disables capture after enable, read, parse or snapshot failure', async () => {
    for (const options of [{ enableThrows: true }, { readThrows: true }, { raw: 'bad-json' },
        { raw: JSON.stringify({ enabled: false }) }, { raw: JSON.stringify({ ...snapshot, params: {} }) }]) {
        const f = workerFixture(options); const reply = await f.call('getEffectiveConfig');
        assert.equal(reply.success,false); assert.equal(typeof reply.error,'string');
        assert.deepEqual(f.toggles,[true,false]); assert.equal(f.engine.capture,false); assert.equal(f.engine.resets,0);
        assert.equal(vm.runInContext('configured',f.context),true);
    }
});

test('Disable failure rejects the read instead of publishing a successful profile', async () => {
    const f = workerFixture({ disableThrows: true }); const reply = await f.call('getEffectiveConfig');
    assert.equal(reply.success,false); assert.match(reply.error,/disable failed/);
    assert.equal(reply.data,undefined); assert.equal(f.engine.resets,0);
});

test('A profile read cannot cancel a pending frame bracket wait', async () => {
    const f = workerFixture(); const framePromise = f.call('frame', { gray: new ArrayBuffer(4), timestamp: 2 });
    const profile = await f.call('getEffectiveConfig'); assert.equal(profile.success,true,profile.error);
    assert.equal(vm.runInContext('processing',f.context),true);
    await f.call('imu', { imuData: new Float64Array([1.9,0,0,9.81,0,0,0,2.01,0,0,9.81,0,0,0]).buffer, count: 2 });
    const frame = await framePromise;
    assert.equal(frame.success,true); assert.equal(frame.data.inputTimestamp,2); assert.equal(frame.data.imuCount,2);
});

class FakeWorker {
    static instances = [];
    constructor() { this.messages = []; FakeWorker.instances.push(this); }
    postMessage(message) { this.messages.push(message); }
    terminate() {}
    last(type) { return this.messages.findLast(message=>message.type===type); }
    reply(request,fields={}) { this.onmessage({data:{...request,success:true,...fields}}); }
}
const previousWorker = globalThis.Worker;
globalThis.Worker = FakeWorker;
test.after(()=>{globalThis.Worker=previousWorker;});
async function wrapperFixture(t) {
    const vio = new VIOWrapper(); t.after(()=>vio.dispose());
    const loaded = vio.load('/fixture.js'), worker = FakeWorker.instances.at(-1);
    worker.reply(worker.last('init')); await loaded;
    const configured = vio.configure({width:2,height:2}); worker.reply(worker.last('configure')); await configured;
    return {vio,worker};
}

test('Wrapper correlates concurrent readback data without releasing a busy frame', async t => {
    const {vio,worker}=await wrapperFixture(t); vio.sendFrame(new Uint8Array(4),1);
    const first=vio.getEffectiveConfig(), second=vio.getEffectiveConfig();
    const requests=worker.messages.filter(message=>message.type==='getEffectiveConfig');
    worker.reply(requests[0],{sequence:requests[0].sequence+99,data:effective});
    worker.reply(requests[1],{data:{...effective,executionSeed:20}}); worker.reply(requests[0],{data:effective});
    assert.deepEqual(await first,effective); assert.equal((await second).executionSeed,20);
    assert.equal(vio.workerBusy,true); assert.equal(vio.getLatestResult(),null); assert.equal(vio.getMetrics().framesCompleted,0);
});

test('Wrapper rejects unconfigured, false, missing and malformed readback data', async t => {
    const empty=new VIOWrapper(); await assert.rejects(empty.getEffectiveConfig(),/configured/);
    const {vio,worker}=await wrapperFixture(t);
    for(const fields of [{success:false},{data:undefined},{data:{}},{data:{...effective,params:{}}},{error:'fixture unavailable'}]){
        const pending=vio.getEffectiveConfig(); worker.reply(worker.last('getEffectiveConfig'),fields);
        await assert.rejects(pending,/profile|configuration|unavailable/i);
        assert.equal(vio.configured,true);
    }
});

test('Reset rejects pending profile reads and late replies do not alter the new lifecycle', async t => {
    const {vio,worker}=await wrapperFixture(t); const pending=vio.getEffectiveConfig(), old=worker.last('getEffectiveConfig');
    const rejected=assert.rejects(pending,/reset/); vio.reset(); worker.reply(worker.last('reset')); await rejected; await Promise.resolve();
    worker.reply(old,{data:effective});
    assert.equal(vio.configured,true); assert.equal(vio.getLatestResult(),null); assert.equal(vio.getMetrics().staleReplies,1);
});

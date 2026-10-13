import test from 'node:test';
import assert from 'node:assert/strict';
import fs from 'node:fs/promises';
import { Worker } from 'node:worker_threads';
import { limitedPipeline } from './limited-fixture.mjs';

const workerSource = await fs.readFile(new URL('../../web/js/vio-worker.js', import.meta.url), 'utf8');
const moduleURL = source => 'data:text/javascript;base64,' + Buffer.from(source).toString('base64');

// Actual Worker dispatch, with a controlled engine at the existing WASM boundary.
// Coordinates are independent expected inputs; no estimator/physical-accuracy claim.
async function fixture(t, scenarios, mobileMotionGuard) {
    const wasmPath = moduleURL(`export default function() {
        const memory = new ArrayBuffer(262144), HEAPU8 = new Uint8Array(memory), HEAPF64 = new Float64Array(memory);
        const scenarios = ${JSON.stringify(scenarios)};
        let cursor = 8;
        return { HEAPU8, HEAPF64, _malloc(n) { const p=cursor; cursor+=Math.ceil(n/8)*8; return p; }, _free() {},
            VIOEngine: class {
                constructor() { this.epoch=0; this.index=0; this.timestamp=-1; this.fresh=false; this.reason='initializing'; this.row={}; }
                configure() { this.reset(); return true; }
                processFrame(image,w,h,imu,count,timestamp,pose) {
                    const row=scenarios[this.index++] || {}; this.row=row; this.timestamp=timestamp;
                    if(row.epochChange) this.epoch++;
                    this.fresh=row.valid!==false; this.reason=row.reason || (this.fresh?'tracking':'insufficient_current_features');
                    const p=row.position || [0,0,0];
                    HEAPF64.set([1,0,0,p[0],0,1,0,p[1],0,0,1,p[2],0,0,0,1],pose/8);
                    return this.fresh;
                }
                getEpoch() { return this.epoch; } getFrameTimestamp() { return this.timestamp; }
                getIMUEndpointTimestamp() { return this.timestamp; } getPoseTimestamp() { return this.fresh?(this.row.poseTimestamp??this.timestamp):-1; }
                getPoseFresh() { return this.fresh; } getPoseValid() { return this.fresh; }
                isInitialized() { return this.row.initialized??this.fresh; } getStatusCode() { return this.row.statusCode??(this.fresh?2:1); }
                getFeaturePointCount() { return 20; } getLastReason() { return this.reason; }
                getPipelineDiagnostics() { return JSON.stringify(this.row.pipeline??{schema:'mobile-slam-pipeline-v1',frameTimestamp:this.timestamp,engineEpoch:this.epoch,frontendMs:1,backendMs:2,solver:null}); }
                getLastSolverIterations() { return 3; } getLastSolverTermination() { return 'NO_CONVERGENCE'; }
                getMapPoints(pointer) { HEAPF64.set([1,2,3],pointer/8); return 1; }
                reset() { this.epoch++; this.timestamp=-1; this.fresh=false; this.reason='external_reset'; this.row={}; }
                delete() {}
            }
        };
    }`);
    const bootstrap = `import { parentPort } from 'node:worker_threads';
        globalThis.self={ postMessage(message) { parentPort.postMessage(message); } };
        await import(${JSON.stringify(moduleURL(workerSource))});
        parentPort.on('message', message => self.onmessage({ data:message }));`;
    const worker = new Worker(new URL(moduleURL(bootstrap)), { type: 'module' });
    t.after(() => worker.terminate());
    const waiting = new Map();
    worker.on('message', message => {
        const pending = waiting.get(message.requestId);
        if (pending && message.type !== 'wasm_log') {
            clearTimeout(pending.timer); waiting.delete(message.requestId); pending.resolve(message);
        }
    });
    worker.on('error', error => {
        for (const pending of waiting.values()) { clearTimeout(pending.timer); pending.reject(error); }
        waiting.clear();
    });
    let sequence = 0, frameIndex = 0;
    const call = (type, data) => {
        const requestId = ++sequence;
        return new Promise((resolve, reject) => {
            const timer = setTimeout(() => { waiting.delete(requestId); reject(new Error('Worker fixture timeout')); }, 3000);
            waiting.set(requestId, { resolve, reject, timer });
            worker.postMessage({ type, data, requestId, sequence, clientEpoch: 1, sentAtMs: performance.timeOrigin + performance.now() });
        });
    };
    const params = { width: 2, height: 2, fx: 1, fy: 1, cx: .5, cy: .5, mobileMotionGuard };
    assert.equal((await call('init', { wasmPath })).success, true);
    const configured = await call('configure', params);
    assert.equal(configured.success, true, configured.error);
    return {
        call, params, wasmPath,
        async frame() {
            const timestamp = 1 + frameIndex++ * .1;
            const imuData = new Float64Array([timestamp,0,0,9.81,0,0,0,timestamp+.01,0,0,9.81,0,0,0]);
            assert.equal((await call('imu', { imuData: imuData.buffer, count: 2 })).success, true);
            return call('frame', { gray: new ArrayBuffer(4), timestamp });
        },
    };
}

function rejected(reply, oldEpoch = 1) {
    const result = reply.data;
    assert.equal(reply.success, true);
    assert.equal(reply.sequence, reply.requestId);
    assert.equal(reply.clientEpoch, 1);
    assert.equal(result.reason, 'pose_translation_discontinuity');
    assert.equal(result.engineEpoch, oldEpoch + 1);
    assert.equal(result.initialized, false);
    assert.equal(result.statusCode, 1);
    assert.equal(result.pose, null);
    assert.equal(result.poseTimestamp, null);
    assert.equal(result.poseFresh, false);
    assert.equal(result.poseValid, false);
    assert.equal(result.mapPoints, null);
    assert.equal(result.mapPointCount, 0);
    assert.equal(result.motionDiscontinuity.engineEpochBeforeReset, oldEpoch);
    assert.equal(result.motionDiscontinuity.engineEpochAfterReset, oldEpoch + 1);
    assert.equal(result.motionDiscontinuity.thresholdM, 5);
    return result;
}

test('Mobile guard defaults off: measured warning and large movement remain numerical poses', async t => {
    const f = await fixture(t, [{ reason: 'tracking:relative_translation_threshold' }, { position: [100,0,0] }]);
    for (let i = 0; i < 2; i++) {
        const result = (await f.frame()).data;
        assert.equal(result.poseValid, true);
        assert.equal(result.engineEpoch, 1);
        assert.equal(result.mapPointCount, 1);
    }
});

test('Guard reset preserves the rejected engine attempt with its pre-reset pipeline identity', async t => {
    const f = await fixture(t, [{ reason: 'tracking:relative_translation_threshold' }], true);
    const result = rejected(await f.frame());
    assert.equal(result.pipeline, null);
    assert.equal(result.candidatePipeline.engineEpoch, result.motionDiscontinuity.engineEpochBeforeReset);
    assert.equal(result.candidatePipeline.frameTimestamp, result.inputTimestamp);
    assert.equal(result.candidatePipeline.frontendMs, 1);
    assert.equal(result.candidatePipeline.backendMs, 2);
});

test('First exact translation warning synchronously resets and exposes no pose or map', async t => {
    const reason = 'tracking:marginalization_allocation_failed;relative_translation_threshold';
    const f = await fixture(t, [{ position: [51800,0,0], reason }, { position: [100,0,0] }], true);
    const result = rejected(await f.frame());
    assert.equal(result.inputTimestamp, 1);
    assert.equal(result.motionDiscontinuity.trigger, 'engine_relative_translation_threshold');
    assert.equal(result.motionDiscontinuity.engineReason, reason);
    assert.equal(result.motionDiscontinuity.translationM, null);
    assert.equal(result.motionDiscontinuity.previousPoseTimestamp, null);
    assert.equal(result.motionDiscontinuity.candidatePoseTimestamp, 1);
    const recovered = (await f.frame()).data;
    assert.equal(recovered.engineEpoch, 2);
    assert.equal(recovered.poseValid, true);
    assert.equal(recovered.pose[3], 100);
    assert.equal(recovered.mapPointCount, 1);
});

test('Independent 3-4-5 translation boundary is permitted', async t => {
    const f = await fixture(t, [{ position: [0,0,0] }, { position: [3,4,0] }], true);
    await f.frame();
    const result = (await f.frame()).data;
    assert.equal(result.poseValid, true);
    assert.equal(result.engineEpoch, 1);
    assert.equal(result.pose[3], 3);
    assert.equal(result.pose[7], 4);
});

test('Independent sqrt(25.01) translation is rejected without an engine motion warning', async t => {
    const f = await fixture(t, [{ position: [0,0,0] }, { position: [3,4,.1], reason: 'tracking:acc_bias_threshold' }], true);
    await f.frame();
    const result = rejected(await f.frame());
    assert.ok(Math.abs(result.motionDiscontinuity.translationM - Math.sqrt(25.01)) < 1e-12);
    assert.equal(result.motionDiscontinuity.trigger, 'published_pose_translation');
    assert.equal(result.motionDiscontinuity.engineReason, 'tracking:acc_bias_threshold');
    assert.equal(result.motionDiscontinuity.previousPoseTimestamp, 1);
    assert.equal(result.motionDiscontinuity.candidatePoseTimestamp, 1.1);
});

test('Invalid image updates cannot advance the last published pose baseline', async t => {
    const f = await fixture(t, [{ position: [0,0,0] }, { position: [4,0,0], valid: false }, { position: [8,0,0] }], true);
    await f.frame();
    assert.equal((await f.frame()).data.poseValid, false);
    const result = rejected(await f.frame());
    assert.equal(result.motionDiscontinuity.translationM, 8);
    assert.equal(result.motionDiscontinuity.previousPoseTimestamp, 1);
});

test('Marginalization, rotation and near-token warnings retain normal finite pose publication', async t => {
    const f = await fixture(t, [
        { position: [10000,20000,0], reason: 'tracking:marginalization_allocation_failed' },
        { position: [10001,20000,0], reason: 'tracking:relative_translation_threshold_extra' },
        { position: [10002,20000,0], reason: 'tracking:relative_rotation_threshold' },
    ], true);
    for (let i = 0; i < 3; i++) {
        const result = (await f.frame()).data;
        assert.equal(result.poseValid, true);
        assert.equal(result.engineEpoch, 1);
        assert.equal(result.solverTermination, 'NO_CONVERGENCE');
    }
});

test('Reset, configure and internal engine epoch changes discard the prior pose baseline', async t => {
    const f = await fixture(t, [
        { position: [0,0,0] }, { position: [100,0,0] }, { position: [-100,0,0] }, { position: [200,0,0], epochChange: true },
    ], true);
    await f.frame();
    assert.equal((await f.call('reset')).success, true);
    assert.equal((await f.frame()).data.poseValid, true);
    assert.equal((await f.call('configure', f.params)).success, true);
    assert.equal((await f.frame()).data.poseValid, true);
    const final = (await f.frame()).data;
    assert.equal(final.engineEpoch, 4);
    assert.equal(final.poseValid, true);
});

test('Transport loss clears the pose baseline before the new engine lifecycle', async t => {
    const f = await fixture(t, [{ position: [0,0,0] }, { position: [100,0,0] }, { position: [100,0,0] }], true);
    await f.frame();
    const imu = new Float64Array(700 * 7);
    for (let i = 0; i < 700; i++) imu.set([1.02 + i * .0001,0,0,9.81,0,0,0], i * 7);
    await f.call('imu', { imuData: imu.buffer, count: 700 });
    const loss = (await f.call('frame', { gray: new ArrayBuffer(4), timestamp: 1.089 })).data;
    assert.equal(loss.reason, 'imu_transport_loss');
    assert.equal(loss.poseValid, false);
    assert.equal(loss.engineEpoch, 2);
    assert.equal((await f.frame()).data.poseValid, true);
});

test('Reconfiguration without the opt-in flag disables the mobile publication policy', async t => {
    const f = await fixture(t, [{ position: [0,0,0] }, { position: [100,0,0], reason: 'tracking:relative_translation_threshold' }], true);
    await f.frame();
    const { mobileMotionGuard: _enabled, ...params } = f.params;
    assert.equal((await f.call('configure', params)).success, true);
    const result = (await f.frame()).data;
    assert.equal(result.engineEpoch, 2);
    assert.equal(result.poseValid, true);
    assert.equal(result.mapPointCount, 1);
});

test('Dispose and replacement engine cannot reuse the old same-numbered epoch pose', async t => {
    const f = await fixture(t, [{ position: [0,0,0] }, { position: [4,0,0] }, { position: [8,0,0] }], true);
    for (let i = 0; i < 3; i++) assert.equal((await f.frame()).data.poseValid, true);
    assert.equal((await f.call('dispose')).success, true);
    assert.equal((await f.call('init', { wasmPath: f.wasmPath })).success, true);
    assert.equal((await f.call('configure', f.params)).success, true);
    const result = (await f.frame()).data;
    assert.equal(result.engineEpoch, 1);
    assert.equal(result.poseValid, true);
    assert.equal(result.pose[3], 0);
});

test('Actual Worker preserves Limited full poses and cumulative quality across ordinary Lost', async t => {
    const first=limitedPipeline(1),lost=limitedPipeline(1.1),last=limitedPipeline(1.2);
    Object.assign(lost.quality,{mode:'none',rotationVisualSupported:false,translationSupport:'none',visualRotationRank:null,
        currentProjectiveFactors:0,uncertaintyAnchorTimestamp:.75,limitedDurationS:.35});
    lost.solver.currentProjectiveFactors=0;
    Object.assign(last.quality,{uncertaintyAnchorTimestamp:.75,limitedDurationS:.45});
    const f=await fixture(t,[
        {statusCode:5,reason:'tracking_limited',position:[.1,.2,.3],pipeline:first},
        {statusCode:3,initialized:true,valid:false,reason:'insufficient_current_features',pipeline:lost},
        {statusCode:5,reason:'tracking_limited',position:[.2,.3,.4],pipeline:last},
    ],true);
    const a=(await f.frame()).data,b=(await f.frame()).data,c=(await f.frame()).data;
    assert.equal(a.trackingState,'tracking_limited');assert.equal(a.poseFresh,true);assert.equal(a.poseTimestamp,1);
    assert.deepEqual([a.pose[3],a.pose[7],a.pose[11]],[.1,.2,.3]);
    assert.equal(b.trackingState,'lost');assert.equal(b.poseFresh,false);assert.equal(b.pose,null);
    assert.equal(c.trackingState,'tracking_limited');assert.equal(c.poseFresh,true);assert.equal(c.poseTimestamp,1.2);
    assert.equal(c.engineEpoch,a.engineEpoch);assert.equal(c.quality.uncertaintyAnchorTimestamp,.75);
    assert.equal(c.quality.limitedDurationS,.45);assert.equal(c.quality.metricTranslationSupported,false);
});

test('Actual Worker applies the same jump and engine-warning guard to Limited', async t => {
    for(const [position,reason] of [[[6,0,0],'tracking_limited'],[[.1,0,0],'tracking_limited:relative_translation_threshold']]) {
        const f=await fixture(t,[{position:[0,0,0]},
            {statusCode:5,reason,position,pipeline:limitedPipeline(1.1)}],true);
        await f.frame();const row=(await f.frame()).data;
        assert.equal(row.reason,'pose_translation_discontinuity');assert.equal(row.poseFresh,false);
        assert.equal(row.trackingState,'initializing');assert.equal(row.pipeline,null);
        assert.equal(row.candidatePipeline.schema,'mobile-slam-pipeline-v2');
        assert.equal(row.candidatePipeline.quality.mode,'projective_limited');
    }
});

test('Actual Worker refuses a stale finite Limited pose and a v1-only Limited claim', async t => {
    const f=await fixture(t,[
        {statusCode:5,reason:'tracking_limited',poseTimestamp:.9,pipeline:limitedPipeline(1)},
        {statusCode:5,reason:'tracking_limited'},
    ],false);
    for(let i=0;i<2;i++) {
        const row=(await f.frame()).data;
        assert.equal(row.poseValid,false);assert.equal(row.poseFresh,false);assert.equal(row.pose,null);
        assert.equal(row.reason,'limited_quality_unavailable');assert.equal(row.engineReason,'tracking_limited');
    }
});

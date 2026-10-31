// M0 passive accounting/source-time contracts. Fake GL is not the B1 display-lifetime feasibility gate.
import assert from 'node:assert/strict';
import fs from 'node:fs';
import path from 'node:path';
import test from 'node:test';
import { createRequire } from 'node:module';
import { pathToFileURL } from 'node:url';
import { createSensors, pushMotion } from '../sensors.js';
import { fakeGl } from '../tools/or1/players.mjs';

const repo = path.resolve(import.meta.dirname, '../../..');
const chunkDir = process.env.SLAM_REFACT_CHUNK_DIR ?? path.join(repo, 'slam_refact/web-chunk');
const engineDir = process.env.SLAM_REFACT_CHUNK_ENGINE_DIR
  ?? path.join(repo, 'build/slam-refact-full-20261019/chunk/rm4d-20261020/engine-pinned');
const event = (timeStamp, interval = 16) => ({ type: 'devicemotion', timeStamp, interval,
  acceleration: null, accelerationIncludingGravity: { x: 0, y: 9.8, z: 0 }, rotationRate: { alpha: 0, beta: 0, gamma: 0 } });

test('D1 event accounting includes queued, staged and in-flight ownership without changing payloads', () => {
  const before = globalThis.window; let now = 10, invalid = false;
  globalThis.window = { performance: { now: () => now } };
  try {
    const s = createSensors(() => invalid, { enabled: true, maxEntries: 4 });
    const original = [event(1), event(2), event(3, Infinity)];
    for (const e of original) { s.updateDeviceMotion(e); now += 16; }
    const first = s.takeForStage();
    assert.deepEqual(s.delivery.snapshot().pending, { queued: 0, staged: 3, inFlight: 0 });
    s.updateDeviceMotion(event(4)); s.updateDeviceMotion(event(5));
    s.delivery.process(first);
    const calls = [];
    for (const e of first) {
      if (pushMotion({ pushDeviceMotionEvent: (...args) => calls.push(args) }, e)) s.delivery.pushed(e);
      else s.delivery.drop([e], 'invalid-interval');
    }
    assert.equal(calls[0][1].accelerationIncludingGravity, original[0].accelerationIncludingGravity);
    assert.equal(calls[1][1].rotationRate, original[1].rotationRate);
    invalid = true; s.updateDeviceMotion(event(6));
    const d = s.delivery.snapshot();
    assert.equal(d.received, 6); assert.equal(d.pushed, 2); assert.equal(d.dropped, 2);
    assert.deepEqual(d.pending, { queued: 2, staged: 0, inFlight: 0 });
    assert.deepEqual(d.droppedByReason, { 'invalid-interval': 1, 'invalid-context': 1 });
    assert.equal(d.accountingError, 0); assert.equal(d.preReceiveLoss, 'unknown');
    assert.equal(JSON.stringify(d).includes('accelerationIncludingGravity'), false);
  } finally { globalThis.window = before; }
});

test('D4 five-second clear and detail truncation retain exact cumulative drop counts', () => {
  const before = globalThis.window; let now = 0;
  globalThis.window = { performance: { now: () => now } };
  try {
    const s = createSensors(() => false, { enabled: true, maxEntries: 4 });
    for (let i = 0; i < 100; i++) { now = i * 10; s.updateDeviceMotion(event(now)); }
    now = 6001; s.updateDeviceMotion(event(now));
    const d = s.delivery.snapshot();
    assert.equal(d.received, 101); assert.equal(d.dropped, 100); assert.equal(d.pending.queued, 1);
    assert.deepEqual(d.dropRanges.entries, [{ reason: 'queue-age-clear', firstSeq: 1, lastSeq: 100, count: 100 }]);
    assert.equal(d.timings.entries.length, 4); assert.equal(d.timings.overwritten, 97);
    assert.equal(d.transitions.entries.length, 4); assert.equal(d.transitions.overwritten, 96);
    assert.equal(d.accountingError, 0);
    d.pending.queued = 1000; d.timings.entries[0].arrivalTimeNanos = -1;
    assert.equal(s.delivery.snapshot().pending.queued, 1, 'snapshot mutation cannot change private counters');
  } finally { globalThis.window = before; }
});

test('D4 a throwing push remains uncertain and a later batch discard never counts acknowledged events twice', () => {
  const before = globalThis.window;
  globalThis.window = { performance: { now: () => 10 } };
  try {
    const s = createSensors(() => false, { enabled: true });
    for (let i = 0; i < 3; i++) s.updateDeviceMotion(event(i));
    const batch = s.takeForStage(); s.delivery.process(batch);
    s.delivery.pushed(batch[0]); s.delivery.uncertain(batch[1]); s.delivery.drop(batch, 'process-error');
    const d = s.delivery.snapshot();
    assert.equal(d.pushed, 1); assert.equal(d.uncertainPushes, 1); assert.equal(d.dropped, 1);
    assert.equal(d.duplicateSettlements, 0); assert.equal(d.accountingError, 0);
    assert.deepEqual(d.dropRanges.entries, [{ reason: 'process-error', firstSeq: 3, lastSeq: 3, count: 1 }]);
  } finally { globalThis.window = before; }
});

let serial = 0;
async function fixture(enabled, check, sourceDir = chunkDir) {
  const saved = new Map(), listeners = new Map(), calls = [], errors = [], clockCalls = [], poses = [];
  const set = (key, value) => { saved.set(key, Object.getOwnPropertyDescriptor(globalThis, key));
    Object.defineProperty(globalThis, key, { configurable: true, writable: true, value }); };
  let now = 1000, failProcess = false, failRead = false, failStage = false, failPush = false,
    failWorldPoints = false, failDispatch = false, lost = false, paused = false;
  set('window', globalThis); set('performance', { now: () => { clockCalls.push(now); return now; } });
  set('document', { visibilityState: 'visible', hasFocus: () => true });
  set('addEventListener', (name, cb) => { const a = listeners.get(name) ?? []; if (!a.includes(cb)) a.push(cb); listeners.set(name, a); });
  set('removeEventListener', (name, cb) => listeners.set(name, (listeners.get(name) ?? []).filter((f) => f !== cb)));
  set('_c8', { checkAllZeroes: 0 });
  set('XR8', { isPaused: () => paused, XrDevice: { deviceEstimate: () => ({ manufacturer: '', model: 'K' }), isDeviceBrowserCompatible: () => true },
    XrConfig: { camera: () => ({ FRONT: 'front', BACK: 'back' }), device: () => ({ MOBILE: 'mobile', ANY: 'any' }) },
    XrPermissions: { permissions: () => ({ CAMERA: 'camera', DEVICE_MOTION: 'devicemotion', DEVICE_ORIENTATION: 'deviceorientation' }) },
    drawTexForComputeTex: (t) => t });
  const base = new URL('slam-refact-engine/', pathToFileURL(`${sourceDir}/`)).href;
  set('fetch', async (url) => { assert.ok(String(url).startsWith(base)); const b = fs.readFileSync(path.join(engineDir, String(url).slice(base.length)));
    return { ok: true, arrayBuffer: async () => b.buffer.slice(b.byteOffset, b.byteOffset + b.byteLength) }; });
  set('Blob', class { constructor(parts) { this.parts = parts; } });
  set('require', createRequire(path.join(engineDir, 'slam_refact_engine.js'))); set('__dirname', engineDir);
  const url = { create: URL.createObjectURL, revoke: URL.revokeObjectURL };
  URL.createObjectURL = (blob) => `data:text/javascript;base64,${Buffer.concat(blob.parts.map((p) => Buffer.from(typeof p === 'string' ? p : new Uint8Array(p)))).toString('base64')}`;
  URL.revokeObjectURL = () => {};
  set('SLAM_REFACT_CHUNK_OPTIONS', { deliveryDiagnostics: { enabled, maxEntries: 4 }, createGlue: (Module, config) => {
    const glue = new Module.SlamRefactGlue(config);
    return new Proxy(glue, { get(target, name) { const value = target[name]; if (typeof value !== 'function') return value;
      return (...args) => {
        if (name === 'getRequestedRois' && failStage) { failStage = false; throw new Error('injected stage failure'); }
        if (name === 'pushDeviceMotionEvent' && failPush) { failPush = false; throw new Error('injected push failure'); }
        if (name === 'getWorldPoints' && failWorldPoints) { failWorldPoints = false; throw new Error('injected world-points failure'); }
        if (name === 'processFramePyramid' && failProcess) { failProcess = false; throw new Error('injected before stage'); }
        if (name === 'pushDeviceMotionEvent') calls.push({ timestampNanos: args[0], payload: args[1] });
        const result = value.apply(target, args);
        if (name === 'processFramePyramid') poses.push({ videoFrameIndex: result.videoFrameIndex, sourceFrameIndex: result.sourceFrameIndex,
          repeated: result.repeated, inputTimeNanos: args[0], inputVideoTimeNanos: args[1] });
        return result;
      }; } });
  } });
  const rawGl = fakeGl({ atlas: new Uint8Array(1024 * 1024 * 4) });
  const gl = new Proxy(rawGl, { get(target, name) { const value = target[name];
    if (name === 'isContextLost') return () => lost;
    if (name !== 'getBufferSubData') return value;
    return (...args) => { if (failRead) { failRead = false; throw new Error('injected read failure'); } return value(...args); }; } });
  const framework = { dispatchEvent: (name) => {
    if (name === 'trackingstatus' && failDispatch) { failDispatch = false; throw new Error('injected dispatch failure'); }
  } }; let pm;
  try {
    const mod = await import(`${pathToFileURL(path.join(sourceDir, 'xr-slam.js')).href}?delivery=${++serial}`);
    const ctrl = await mod.XrControllerFactory({ paused: false }, Promise.resolve({}), () => () => {}, (e) => errors.push(String(e)));
    pm = ctrl.pipelineModule();
    ctrl.updateCameraProjectionMatrix({ cam: { pixelRectWidth: 240, pixelRectHeight: 320, nearClipPlane: 0.01, farClipPlane: 1000 } });
    pm.onBeforeSessionInitialize({ sessionAttributes: { fillsCameraTexture: true, controlsCamera: false } });
    pm.onStart({ computeCtx: gl, canvasWidth: 240, canvasHeight: 320 });
    pm.onSessionAttach({ videoWidth: 240, videoHeight: 320, canvasWidth: 240, canvasHeight: 320, rotation: 0,
      computeCtx: gl, framework, GLctx: gl, sessionAttributes: { fillsCameraTexture: true } });
    const frame = (k) => ({ computeTexture: { id: k }, cameraTexture: { id: k }, textureWidth: 240, textureHeight: 320,
      orientation: 0, repeatFrame: false, videoTime: 20 + k / 30, frameTime: k === 0 ? 0 : 1 + k / 30 });
    const dispatch = (e) => { for (const cb of listeners.get(e.type) ?? []) cb(e); };
    await check({ ctrl, pm, gl, framework, frame, calls, poses, errors, clockCalls,
      setNow: (v) => { now = v; }, sensor: (t, interval) => { now = t; dispatch(event(t - 5, interval)); },
      stage: (f) => pm.onProcessGpu({ framework, frameStartResult: f }),
      cpu: (f) => pm.onProcessCpu({ framework, frameStartResult: f }),
      snapshot: () => globalThis.__slamRefactChunk.delivery,
      latest: () => globalThis.__slamRefactChunk.sourceTiming,
      failProcess: () => { failProcess = true; }, failRead: () => { failRead = true; },
      failStage: () => { failStage = true; }, failPush: () => { failPush = true; },
      failWorldPoints: () => { failWorldPoints = true; }, failDispatch: () => { failDispatch = true; },
      loseContext: () => { lost = true; }, pause: (v) => { paused = v; } });
  } finally {
    if (pm) { pm.onSessionDetach({}); pm.onDetach({}); }
    for (const [k, d] of saved) { if (d) Object.defineProperty(globalThis, k, d); else delete globalThis[k]; }
    URL.createObjectURL = url.create; URL.revokeObjectURL = url.revoke;
  }
}

test('source timing follows actual engine indices, including fallback frame time and callback repeats', async () => {
  await fixture(true, async (s) => {
    s.sensor(995); s.setNow(1000); s.stage(s.frame(0));
    assert.equal(s.cpu(s.frame(1)), null);
    assert.equal(s.snapshot().sourceTiming.fresh, false);
    s.sensor(1028); s.setNow(1033); s.stage(s.frame(1)); s.cpu(s.frame(2));
    const first = s.snapshot().sourceTiming;
    assert.equal(first.sourceFrameIndex, 0); assert.equal(first.sourceVideoTimeS, 20); assert.equal(first.sourceFrameTimeS, 0);
    assert.equal(first.sourceTimeNanosInput, 1e9); assert.equal(first.sourceTimeNanos, 1e9);
    assert.equal(first.callbackVideoTimeS, s.frame(2).videoTime); assert.equal(first.fresh, true); assert.equal(first.repeated, false);
    assert.equal(first.sessionEpoch, 1); assert.equal(first.currentSessionEpoch, 1);
    const fast = s.latest(); assert.deepEqual(fast, first, 'fast and full getters expose the same latest receipt');
    fast.sourceFrameIndex = 999; fast.fresh = false;
    assert.deepEqual(s.latest(), first, 'fast getter returns a detached copy');
    s.cpu({ ...s.frame(3), repeatFrame: true });
    const repeated = s.snapshot().sourceTiming;
    assert.equal(repeated.sourceFrameIndex, first.sourceFrameIndex); assert.equal(repeated.repeated, false, 'engine flag is preserved');
    assert.equal(repeated.fresh, false); assert.equal(repeated.publicationRepeated, true);
    s.sensor(1060); s.stage(s.frame(2)); s.cpu(s.frame(4));
    const normal = s.snapshot().sourceTiming;
    const input = s.poses.find((p) => p.videoFrameIndex === normal.sourceFrameIndex);
    assert.equal(normal.sourceFrameIndex, 1);
    assert.equal(normal.sourceTimeNanosInput, input.inputTimeNanos, 'metadata names the argument actually handed to the glue');
    assert.equal(normal.sourceTimeNanos, Math.trunc(input.inputTimeNanos));
    assert.equal(normal.sourceVideoTimeS, s.frame(1).videoTime);
    assert.equal(s.snapshot().sensors.accountingError, 0);
  });
});

test('D4 stage replacement carries sensors; process failure after pushes does not replay them or mislabel source time', async () => {
  await fixture(true, async (s) => {
    s.sensor(990); s.stage(s.frame(0)); s.sensor(1010); s.stage(s.frame(1));
    assert.deepEqual(s.snapshot().sensors.pending, { queued: 0, staged: 2, inFlight: 0 });
    s.failProcess(); assert.equal(s.cpu(s.frame(2)), null);
    assert.equal(s.calls.length, 2); assert.equal(s.snapshot().sensors.pushed, 2);
    s.sensor(1040); s.stage(s.frame(2)); s.cpu(s.frame(3));
    s.sensor(1070); s.stage(s.frame(3)); s.cpu(s.frame(4));
    const timing = s.snapshot().sourceTiming;
    assert.equal(timing.sourceFrameIndex, 0, 'the failed engine call did not advance its index');
    assert.equal(timing.sourceVideoTimeS, s.frame(2).videoTime, 'actual index 0 belongs to the later successful stage');
    assert.equal(timing.sourceTimeNanosInput, s.frame(2).frameTime * 1e9);
    assert.equal(s.calls.length, 4); assert.equal(s.snapshot().sensors.accountingError, 0);
    assert.equal(s.snapshot().sensors.duplicateSettlements, 0);
  });
});

test('D4 read failure and session disposal account only for unpushed events', async () => {
  await fixture(true, async (s) => {
    s.sensor(990); s.stage(s.frame(0)); s.failRead(); s.cpu(s.frame(1));
    assert.equal(s.snapshot().sensors.droppedByReason['process-error'], 1);
    s.sensor(1020); s.stage(s.frame(1)); s.pm.onSessionDetach({});
    const d = s.snapshot().sensors;
    assert.equal(d.droppedByReason['session-discard'], 1); assert.equal(d.pushed, 0);
    assert.equal(d.accountingError, 0); assert.deepEqual(d.pending, { queued: 0, staged: 0, inFlight: 0 });
  });
});

test('D5 state model keeps held source epoch across rebuild and bounds detached response snapshots', async () => {
  await fixture(true, async (s) => {
    for (let k = 0; k < 3; k++) { s.sensor(990 + 33 * k); s.stage(s.frame(k)); s.cpu(s.frame(k + 1)); }
    const old = s.snapshot().sourceTiming;
    s.pm.onVideoSizeChange({ videoWidth: 270, videoHeight: 360, orientation: 0, computeCtx: s.gl, framework: s.framework });
    s.cpu(s.frame(4));
    const held = s.snapshot().sourceTiming;
    assert.equal(held.sessionEpoch, old.sessionEpoch); assert.equal(held.currentSessionEpoch, old.sessionEpoch + 1);
    assert.equal(held.sourceVideoTimeS, old.sourceVideoTimeS); assert.equal(held.fresh, false);
    for (let k = 4; k < 10; k++) s.cpu({ ...s.frame(k), repeatFrame: true });
    const d = s.snapshot(); assert.equal(d.responses.entries.length, 4); assert.equal(d.responses.overwritten, 6);
    d.sourceTiming.fresh = true; d.responses.entries[0].sessionEpoch = 999;
    assert.equal(s.snapshot().sourceTiming.fresh, false); assert.notEqual(s.snapshot().responses.entries[0].sessionEpoch, 999);
    s.pm.onDeviceOrientationChange({ videoWidth: 270, videoHeight: 360, orientation: 45, computeCtx: s.gl, framework: s.framework });
    s.cpu(s.frame(11));
    assert.equal(s.snapshot().sourceTiming.sessionEpoch, old.sessionEpoch);
    assert.equal(s.snapshot().sourceTiming.currentSessionEpoch, null);
    assert.equal(s.snapshot().sourceTiming.fresh, false);
  });
});

test('D4 a failed replacement drops only new events while the old staged batch remains owned', async () => {
  await fixture(true, async (s) => {
    s.sensor(990); s.stage(s.frame(0)); s.sensor(1010); s.failStage(); s.stage(s.frame(1));
    assert.deepEqual(s.snapshot().sensors.pending, { queued: 0, staged: 1, inFlight: 0 });
    assert.equal(s.snapshot().sensors.droppedByReason['stage-error'], 1);
    s.cpu(s.frame(2));
    assert.equal(s.calls.length, 1); assert.equal(s.calls[0].timestampNanos, 990e6);
    assert.equal(s.snapshot().sensors.accountingError, 0);
  });
});

test('D4 unavailable session, context loss and push exceptions retain distinct accounting', async () => {
  await fixture(true, async (s) => {
    s.sensor(990); s.stage(s.frame(0)); s.loseContext(); s.sensor(1010); s.stage(s.frame(1));
    assert.equal(s.snapshot().sensors.droppedByReason['context-lost'], 1);
    assert.equal(s.snapshot().sensors.pending.staged, 1);
    s.pm.onSessionDetach({}); s.sensor(1030); s.stage(s.frame(2));
    const d = s.snapshot().sensors;
    assert.equal(d.droppedByReason['session-discard'], 1); assert.equal(d.droppedByReason['no-session'], 1);
    assert.equal(d.accountingError, 0);
  });
  await fixture(true, async (s) => {
    s.sensor(990); s.sensor(991); s.stage(s.frame(0)); s.failPush(); s.cpu(s.frame(1));
    const d = s.snapshot().sensors;
    assert.equal(d.uncertainPushes, 1); assert.equal(d.droppedByReason['process-error'], 1);
    assert.equal(d.pushed, 0); assert.equal(d.accountingError, 0);
  });
});

test('source metadata retains at most 64 successful staged inputs', async () => {
  await fixture(true, async (s) => {
    for (let k = 0; k < 70; k++) { s.sensor(990 + k * 33); s.stage(s.frame(k)); s.cpu(s.frame(k + 1)); }
    const d = s.snapshot(); assert.equal(d.sourceMetadata.entries, 64); assert.equal(d.sourceMetadata.evicted, 6);
    assert.equal(d.responses.entries.length, 4); assert.equal(d.responses.overwritten, 66);
    assert.equal(d.sourceTiming.fresh, true); assert.equal(d.sensors.accountingError, 0);
  });
});

test('publication freshness starts at the first successful return after getWorldPoints throws', async () => {
  await fixture(true, async (s) => {
    s.sensor(990); s.stage(s.frame(0)); s.cpu(s.frame(1));
    s.sensor(1020); s.stage(s.frame(1)); s.failWorldPoints();
    assert.equal(s.cpu(s.frame(2)), null);
    assert.equal(s.snapshot().sourceTiming.hasPose, false); assert.equal(s.snapshot().sourceTiming.fresh, false);
    assert.equal(s.snapshot().sourceTiming.engineFresh, true, 'engine success and publication success are separate');
    assert.ok(s.cpu({ ...s.frame(3), repeatFrame: true }));
    const first = s.snapshot().sourceTiming;
    assert.equal(first.sourceFrameIndex, 0); assert.equal(first.repeated, false);
    assert.equal(first.fresh, true); assert.equal(first.publicationRepeated, false);
    assert.equal(first.newEngineResponse, false); assert.equal(first.engineFresh, false);
    s.cpu({ ...s.frame(4), repeatFrame: true });
    assert.equal(s.snapshot().sourceTiming.fresh, false); assert.equal(s.snapshot().sourceTiming.publicationRepeated, true);
  });
});

test('a throwing framework dispatch cannot commit a successful publication receipt', async () => {
  await fixture(true, async (s) => {
    s.sensor(990); s.stage(s.frame(0)); s.cpu(s.frame(1));
    s.sensor(1020); s.stage(s.frame(1)); s.failDispatch();
    assert.throws(() => s.cpu(s.frame(2)), /injected dispatch failure/);
    assert.equal(s.snapshot().sourceTiming.hasPose, false); assert.equal(s.snapshot().sourceTiming.fresh, false);
    assert.equal(s.snapshot().sourceTiming.engineFresh, true);
    assert.ok(s.cpu({ ...s.frame(3), repeatFrame: true }));
    assert.equal(s.snapshot().sourceTiming.fresh, true); assert.equal(s.snapshot().sourceTiming.publicationRepeated, false);
  });
});

test('M0 OFF and ON preserve sensor payloads, clock calls and 20-frame default poses', async () => {
  const run = async (enabled, sourceDir) => {
    let result;
    await fixture(enabled, async (s) => {
      const answers = [];
      for (let k = 0; k < 20; k++) {
        s.sensor(990 + 33 * k); s.setNow(1000 + 33 * k); s.stage(s.frame(k));
        const r = s.cpu(s.frame(k + 1));
        answers.push(r ? { position: r.position, rotation: r.rotation, intrinsics: r.intrinsics,
          status: r.trackingStatus, reason: r.trackingReason, worldPoints: r.worldPoints } : null);
      }
      result = { answers, calls: s.calls, clockCalls: s.clockCalls.slice(), poses: s.poses };
      if (enabled) { assert.equal(s.snapshot().sensors.received, 20); assert.equal(s.snapshot().sensors.accountingError, 0); }
      else {
        assert.equal(s.snapshot(), undefined);
        assert.equal(s.latest(), sourceDir === chunkDir ? null : undefined, 'the pre-M0 source has no latest-only getter');
      }
    }, sourceDir);
    return result;
  };
  const off = await run(false, chunkDir), on = await run(true, chunkDir);
  assert.deepEqual(on, off);
  if (process.env.IMU_DELIVERY_BASELINE_CHUNK_DIR) assert.deepEqual(off, await run(false, process.env.IMU_DELIVERY_BASELINE_CHUNK_DIR));
});

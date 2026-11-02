// node --test slam_refact/web-chunk/tests/engine-worker-protocol.test.mjs
// R1 engine Worker: the message protocol between engine-worker-client.js (main thread) and engine-worker.js (Worker side), with a fake
// engine and a loopback transport that clones messages and detaches transferred buffers like a real Worker. No WASM, no GL.
import assert from 'node:assert/strict';
import test from 'node:test';
import { createEngineWorkerClient, engineWorkerRequested, engineWorkerUnsupportedReason, loadEngineInWorker } from '../engine-worker-client.js';
import { toPlain } from '../engine-worker.js';
import { loopbackWorker } from './engine-worker-harness.mjs';

const LAYOUT = { textureSize: 8, rows: 4, roiAtlasRows: 4, levels: [{ r: 0, c: 0, w: 8, h: 4 }, {}, {}, { r: 0, c: 0, w: 2, h: 2 }] };

// A fake engine: records every call in order and answers processFramePyramid with the frame index.
function fakeEngine({ layout = LAYOUT, failFrame = null, failOp = null, compile = { loopClose: true }, drift = false } = {}) {
  const calls = [];
  let frames = 0, args = null, built = 0;
  class FakeGlue {
    constructor(config) { calls.push(['new', config]); this.config = config; this.n = ++built; }
    pyramidLayout() { return drift && this.n > 1 ? { ...layout, rows: layout.rows + 1 } : layout; }
    getCaptureIntrinsics() { return [1, 2, 3, 4]; }
    configure(a) { if (failOp === 'configure') throw new Error('bad configure'); calls.push(['configure', a]); }
    updateCameraProjectionMatrix(a) { calls.push(['updateCameraProjectionMatrix', a]); }
    recenter() { calls.push(['recenter']); }
    pushDeviceMotionEvent(t, e) { calls.push(['pushDeviceMotionEvent', t, e]); }
    pushDeviceOrientation(...a) { calls.push(['pushDeviceOrientation', ...a]); }
    processFramePyramid(timeNanos, videoNs, atlas, size, rows) {
      frames++;
      if (failFrame === frames) throw new Error(`engine rejected frame ${frames}`);
      calls.push(['processFramePyramid', timeNanos, videoNs, atlas.length, size, rows, atlas[0]]);
      return { videoFrameIndex: frames, sourceFrameIndex: frames - 1, repeated: false, position: new Float32Array([frames, 0, 0]),
        rotationWxyz: [1, 0, 0, 0], intrinsics: new Float32Array(16), status: 'NORMAL', reason: 'UNSPECIFIED', detectedImages: [] };
    }
    getWorldPoints() { return new Float32Array([1, 0.5, frames, 0, 0]); }
    getFramesUntilRecenter() { return Math.max(0, 5 - frames); }
    getRequestedRois() { return []; }
    delete() { calls.push(['delete']); }
  }
  FakeGlue.getCompileFeatures = () => compile;
  const loadEngine = async (url, moduleArgs) => {
    args = moduleArgs;
    moduleArgs.print('hello'); moduleArgs.printErr('careful');
    return { Module: { SlamRefactGlue: FakeGlue }, loaded: { jsSha256: 'a', wasmSha256: 'b', matchesManifest: true, manifest: { jsSha256: 'a', wasmSha256: 'b', build: null } } };
  };
  return { calls, loadEngine, abortNow: (what) => args.onAbort(what) };
}

async function start({ engine = fakeEngine(), delay, clientOptions = {} } = {}) {
  const loop = loopbackWorker({ coreDeps: { loadEngine: engine.loadEngine }, delay });
  const logs = [], warnings = [], fatals = [];
  const moduleArgs = { print: (t) => logs.push(['out', t]), printErr: (t) => logs.push(['err', t]), onAbort: (w) => logs.push(['abort', w]) };
  const r = await loadEngineInWorker('file:///engine/', moduleArgs, { createWorker: () => loop.worker, warn: (m) => warnings.push(m), onFatal: (e) => fatals.push(e), ...clientOptions });
  return { ...r, loop, engine, logs, warnings, fatals, calls: engine.calls };
}
const config = { captureWidth: 8, captureHeight: 4, rotationDeg: 0, manufacturer: '', model: '' };

test('R1-P1: the mode is off by default and on for the option or ?engineWorker=1 only', () => {
  assert.equal(engineWorkerRequested({}, ''), false);
  assert.equal(engineWorkerRequested(undefined, ''), false);
  assert.equal(engineWorkerRequested({ engineWorker: true }, ''), true);
  assert.equal(engineWorkerRequested({ engineWorker: { enabled: true } }, ''), true);
  assert.equal(engineWorkerRequested({}, '?engineWorker=1'), true);
  assert.equal(engineWorkerRequested({}, '?a=1&engineWorker=1'), true);
  assert.equal(engineWorkerRequested({}, '?engineWorker=0'), false);
  assert.equal(engineWorkerRequested({}, '?engineWorker=true'), false);
  assert.equal(engineWorkerRequested({}, '?capture=worker'), false, '?capture=worker is a different, unrelated switch');
  assert.equal(engineWorkerRequested({ engineWorker: false }, '?engineWorker=1'), false, 'an explicit option wins over the URL');
});

test('R1-P2: options the Worker mode cannot honour are named', () => {
  assert.equal(engineWorkerUnsupportedReason({}), null);
  assert.match(engineWorkerUnsupportedReason({ frameScheduler: { enabled: true } }), /frameScheduler/);
  assert.match(engineWorkerUnsupportedReason({ createGlue() {} }), /createGlue/);
  assert.match(engineWorkerUnsupportedReason({ gpuCheck: {} }), /gpuCheck/);
  assert.match(engineWorkerUnsupportedReason({ roiCheck: {} }), /roiCheck/);
});

test('R1-P3: load handshake returns the loader shape; engine output and compile features come across', async () => {
  const w = await start();
  assert.equal(w.loaded.matchesManifest, true);
  assert.equal(typeof w.loaded.workerLoadMs, 'number');
  assert.equal(typeof w.Module.SlamRefactGlue, 'function');
  assert.deepEqual(w.Module.SlamRefactGlue.getCompileFeatures(), { loopClose: true });
  assert.deepEqual(w.logs, [['out', 'hello'], ['err', 'careful']]);
  assert.deepEqual(w.loop.state.toWorker[0], { type: 'load', baseUrl: 'file:///engine/' });
});

test('R1-P4: a post-load engine abort is ONE fatal report (onFatal), not also an onAbort call', async () => {
  const engine = fakeEngine();
  const w = await start({ engine });
  engine.abortNow('Aborted(boom)');
  await w.loop.idle();
  assert.deepEqual(w.logs.filter((l) => l[0] === 'abort'), [], 'moduleArgs.onAbort is not called: the fatal report is the single report');
  assert.equal(w.fatals.length, 1);
  assert.equal(w.fatals[0].code, 'engine-worker-abort');
  assert.throws(() => new w.Module.SlamRefactGlue(config), /Aborted\(boom\)/, 'nothing more is posted to a dead Worker');
});

test('R1-P4b: a failure BEFORE the load completes only rejects the load: no onFatal, no onAbort (the chunk falls back quietly)', async () => {
  for (const kind of ['script-error', 'abort-during-load', 'load-error']) {
    const logs = [], fatals = [];
    const base = fakeEngine();
    const loadEngine = async (url, args) => {
      if (kind === 'abort-during-load') { args.onAbort('Aborted(early)'); await new Promise((r) => setTimeout(r, 5)); }
      if (kind === 'load-error') throw Object.assign(new Error('bad bytes'), { code: 'engine-simd-unsupported' });
      return base.loadEngine(url, args);
    };
    const loop = loopbackWorker({ coreDeps: { loadEngine } });
    const worker = kind === 'script-error'
      ? { postMessage() { queueMicrotask(() => worker.onerror?.({ message: 'Failed to fetch module script' })); }, terminate() {} } : loop.worker;
    await assert.rejects(loadEngineInWorker('file:///engine/', { onAbort: (w) => logs.push(w) }, { createWorker: () => worker, onFatal: (e) => fatals.push(e) }),
      (e) => (kind === 'load-error' ? e.code === 'engine-simd-unsupported' : /Worker failed|aborted in the Worker/.test(e.message)), kind);
    await loop.idle();
    assert.deepEqual(logs, [], `${kind}: onAbort not called`);
    assert.deepEqual(fatals, [], `${kind}: onFatal not called`);
  }
});

test('R1-P5: the layout is probed once per capture; the glue answers it synchronously; a changed layout is detected', async () => {
  const w = await start();
  assert.equal(w.worker.hasLayout(8, 4, 0), false);
  const layout = await w.worker.requestLayout(8, 4, 0);
  assert.deepEqual(layout, LAYOUT);
  assert.equal(w.worker.hasLayout(8, 4, 0), true);
  assert.equal(await w.worker.requestLayout(8, 4, 0), layout, 'cached');
  assert.equal(w.worker.stats.layoutProbes, 1);
  const glue = new w.Module.SlamRefactGlue(config);
  assert.deepEqual(glue.pyramidLayout(), LAYOUT);
  assert.throws(() => new w.Module.SlamRefactGlue({ ...config, captureWidth: 16 }).pyramidLayout(), (e) => e.code === 'engine-worker-layout-missing');
  await w.loop.idle();
  assert.equal(glue.fatal, null);
});

test('R1-P5b: a created glue whose layout differs from the probed layout is failed, not silently used', async () => {
  const w = await start({ engine: fakeEngine({ drift: true }) });
  await w.worker.requestLayout(8, 4, 0);
  const glue = new w.Module.SlamRefactGlue(config);
  await w.loop.idle();
  assert.equal(glue.fatal.code, 'engine-worker-layout-mismatch');
  assert.equal(w.worker.stats.layoutMismatches, 1);
});

test('R1-P6: ordered operations reach the engine in call order; DeviceMotion sub-objects are copied to plain data', async () => {
  const w = await start();
  await w.worker.requestLayout(8, 4, 0);
  w.calls.length = 0;  // the probe glue is not part of this test
  const glue = new w.Module.SlamRefactGlue(config);
  glue.updateCameraProjectionMatrix({ origin: { x: 0, y: 2, z: 0 }, updateRecenterPoint: true });
  glue.configure({ enableWorldPoints: true });
  glue.recenter();
  // WebIDL-like objects: values behind prototype getters are lost by structured clone unless copied field by field
  class Accel { get x() { return 0.1; } get y() { return 0.2; } get z() { return 0.3; } }
  class Rate { get alpha() { return 1; } get beta() { return 2; } get gamma() { return 3; } }
  glue.pushDeviceMotionEvent(5, { acceleration: null, accelerationIncludingGravity: new Accel(), rotationRate: new Rate(), interval: 16 });
  glue.pushDeviceOrientation(6, 1, 2, 3);
  await w.loop.idle();
  assert.deepEqual(w.calls.map((c) => c[0]), ['new', 'updateCameraProjectionMatrix', 'configure', 'recenter', 'pushDeviceMotionEvent', 'pushDeviceOrientation']);
  assert.deepEqual(w.calls[4], ['pushDeviceMotionEvent', 5, { acceleration: null, accelerationIncludingGravity: { x: 0.1, y: 0.2, z: 0.3 },
    rotationRate: { alpha: 1, beta: 2, gamma: 3 }, interval: 16 }]);
  assert.equal(w.worker.stats.opErrors, 0);
});

test('R1-P7: a frame transfers its atlas (detached here), carries IMU + orientation, and the buffer comes back for reuse', async () => {
  const w = await start();
  await w.worker.requestLayout(8, 4, 0);
  w.calls.length = 0;
  const glue = new w.Module.SlamRefactGlue(config);
  const dst = glue.acquireAtlas(8 * 4 * 4);
  assert.equal(w.worker.stats.buffersAllocated, 1);
  dst.fill(7);
  const buffer = dst.buffer;
  const seq = glue.submitFrame({ motion: [[5, { acceleration: null, accelerationIncludingGravity: { x: 1, y: 2, z: 3 }, rotationRate: null, interval: 16 }]],
    orientation: [6, 10, 20, 30], timeNanos: 1000, videoNs: 2000, atlas: dst.subarray(0, 8 * 4 * 4), size: 8, rows: 4 });
  assert.equal(seq, 1);
  assert.equal(buffer.byteLength, 0, 'the atlas buffer was transferred, not copied');
  assert.equal(glue.inflight, 1);
  await w.loop.idle();
  assert.equal(glue.inflight, 0);
  assert.deepEqual(w.calls.slice(1).map((c) => c[0]), ['pushDeviceMotionEvent', 'pushDeviceOrientation', 'processFramePyramid']);
  assert.deepEqual(w.calls.at(-1), ['processFramePyramid', 1000, 2000, 8 * 4 * 4, 8, 4, 7], 'the engine saw the bytes that were read back');
  const [r] = glue.takeResults();
  assert.equal(r.seq, 1);
  assert.equal(r.pose.videoFrameIndex, 1);
  assert.deepEqual(Array.from(r.pose.position), [1, 0, 0]);
  assert.deepEqual(Array.from(r.world), [1, 0.5, 1, 0, 0]);
  assert.equal(glue.getFramesUntilRecenter(), 4);
  assert.deepEqual(glue.takeResults(), []);
  const again = glue.acquireAtlas(8 * 4 * 4);
  assert.equal(w.worker.stats.buffersReused, 1);
  assert.equal(w.worker.stats.buffersAllocated, 1, 'steady state allocates nothing');
  assert.equal(again.byteLength, 8 * 4 * 4);
});

test('R1-P8: frames are processed in submission order even when the Worker is slow and uneven; none is dropped', async () => {
  let n = 0;
  const w = await start({ delay: () => (++n % 3 === 0 ? 8 : 0), clientOptions: { maxInFlight: 100 } });
  await w.worker.requestLayout(8, 4, 0);
  const glue = new w.Module.SlamRefactGlue(config);
  const seqs = [];
  for (let k = 0; k < 30; k++) {
    const a = glue.acquireAtlas(8 * 4 * 4); a.fill(k);
    seqs.push(glue.submitFrame({ motion: [], orientation: [k, 0, 0, 0], timeNanos: k, videoNs: k, atlas: a, size: 8, rows: 4 }));
  }
  assert.equal(glue.inflight, 30);
  await w.loop.idle();
  const results = glue.takeResults();
  assert.deepEqual(results.map((r) => r.seq), seqs);
  assert.deepEqual(results.map((r) => r.pose.videoFrameIndex), Array.from({ length: 30 }, (_, i) => i + 1));
  const processed = w.calls.filter((c) => c[0] === 'processFramePyramid');
  assert.deepEqual(processed.map((c) => c[6]), Array.from({ length: 30 }, (_, k) => k), 'the engine saw the atlases in capture order');
  assert.equal(w.worker.stats.framesSubmitted, 30);
  assert.equal(w.worker.stats.resultsReceived, 30);
  assert.equal(glue.inflight, 0);
});

test('R1-P9: an engine error on a frame is a per-frame result error (later frames still run); an operation error is counted and warned', async () => {
  const w = await start({ engine: fakeEngine({ failFrame: 2, failOp: 'configure' }) });
  await w.worker.requestLayout(8, 4, 0);
  const glue = new w.Module.SlamRefactGlue(config);
  glue.configure({ enableWorldPoints: true });
  for (let k = 0; k < 3; k++) glue.submitFrame({ motion: [], orientation: [k, 0, 0, 0], timeNanos: k, videoNs: k, atlas: glue.acquireAtlas(128), size: 8, rows: 4 });
  await w.loop.idle();
  const results = glue.takeResults();
  assert.equal(results.length, 3);
  assert.equal(results[0].error, undefined);
  assert.match(results[1].error.message, /engine rejected frame 2/);
  assert.equal(results[2].pose.videoFrameIndex, 3, 'the frame after a failed one is processed');
  assert.equal(w.worker.stats.opErrors, 1);
  assert.match(w.warnings[0], /configure failed: bad configure/);
});

test('R1-P10: a crashed Worker is reported once, marks every glue fatal and refuses further posts', async () => {
  const w = await start();
  await w.worker.requestLayout(8, 4, 0);
  const glue = new w.Module.SlamRefactGlue(config);
  w.loop.crash('RuntimeError: unreachable');
  w.loop.crash('again');
  assert.equal(w.fatals.length, 1);
  assert.equal(w.fatals[0].code, 'engine-worker-error');
  assert.match(w.fatals[0].message, /unreachable/);
  assert.equal(glue.fatal, w.fatals[0]);
  assert.throws(() => glue.submitFrame({ motion: [], orientation: [0, 0, 0, 0], timeNanos: 0, videoNs: 0, atlas: glue.acquireAtlas(128), size: 8, rows: 4 }), /unreachable/);
  await assert.rejects(w.worker.requestLayout(16, 8, 0), /unreachable/);
});

test('R1-P11: unsupported queries answer plainly: hitTest has no hit (counted, warned), image targets are refused, no ROIs', async () => {
  const w = await start();
  await w.worker.requestLayout(8, 4, 0);
  const glue = new w.Module.SlamRefactGlue(config);
  assert.deepEqual(glue.xrHitTest(0.5, 0.5, []), []);
  assert.equal(w.worker.stats.hitTestMisses, 1);
  assert.match(w.warnings[0], /hitTest answers no hit/);
  assert.throws(() => glue.setImageTargets([{}]), /image targets are not supported with engineWorker/);
  assert.equal(w.worker.stats.imageTargetRefusals, 1);
  assert.deepEqual(glue.getRequestedRois(), []);
  assert.equal(glue.getFramesUntilRecenter(), 5);
  assert.equal(glue.getWorldPoints().length, 0);
});

test('R1-P12: delete is ordered after earlier work and frees the Worker-side glue; a second delete is a no-op', async () => {
  const w = await start();
  await w.worker.requestLayout(8, 4, 0);
  w.calls.length = 0;
  const glue = new w.Module.SlamRefactGlue(config);
  glue.recenter();
  glue.delete();
  glue.delete();
  glue.recenter();  // after delete: ignored
  await w.loop.idle();
  assert.deepEqual(w.calls.map((c) => c[0]), ['new', 'recenter', 'delete']);
  assert.equal(w.loop.core.glueCount(), 0);
});

test('R1-P13: toPlain flattens engine answers to cloneable data (typed arrays copied, functions dropped)', () => {
  const ta = new Float32Array([1, 2]);
  const plain = toPlain({ a: ta, b: [new Uint8Array([3]), { c: 4 }], f() {}, n: null, s: 'x' });
  assert.notEqual(plain.a, ta);
  assert.deepEqual(Array.from(plain.a), [1, 2]);
  assert.deepEqual(Array.from(plain.b[0]), [3]);
  assert.equal(plain.b[1].c, 4);
  assert.equal('f' in plain, false);
  assert.equal(plain.n, null);
  structuredClone(plain);
});

test('R1-P14: the client works on any transport object (createEngineWorkerClient with a bare stub)', () => {
  const posted = [];
  const stub = { postMessage: (m) => posted.push(m) };
  const client = createEngineWorkerClient(stub);
  client.requestLayout(1, 2, 90).catch(() => {});
  assert.equal(posted[0].type, 'probe');
  assert.deepEqual(posted[0].config, { captureWidth: 1, captureHeight: 2, rotationDeg: 90, manufacturer: '', model: '' });
  stub.onmessage({ data: { type: 'probed', id: posted[0].id, layout: { ok: 1 } } });
  assert.equal(client.hasLayout(1, 2, 90), true);
});

test('R1-P15: buffers smaller than the next request are dropped from the pool (a capture size change cannot clog it)', async () => {
  const w = await start({ clientOptions: { maxInFlight: 4 } });
  await w.worker.requestLayout(8, 4, 0);
  const glue = new w.Module.SlamRefactGlue(config);
  const small = glue.acquireAtlas(64); small.fill(1);
  glue.submitFrame({ motion: [], orientation: [0, 0, 0, 0], timeNanos: 0, videoNs: 0, atlas: small.subarray(0, 8 * 4 * 4), size: 8, rows: 4 });
  await w.loop.idle();
  const big = glue.acquireAtlas(4096);
  assert.equal(big.byteLength, 4096);
  assert.equal(w.worker.stats.buffersDropped, 1, 'the 64-byte buffer was dropped, not kept');
  assert.equal(w.worker.stats.buffersReused, 0);
  assert.equal(w.worker.stats.buffersAllocated, 2);
});

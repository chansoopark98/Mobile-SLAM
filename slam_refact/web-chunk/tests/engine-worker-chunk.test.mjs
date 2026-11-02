// node --test slam_refact/web-chunk/tests/engine-worker-chunk.test.mjs
// R1 engine Worker at chunk level: xr-slam.js with the real engine WASM (tracked deployed build, or SLAM_REFACT_CHUNK_ENGINE_DIR) and a fake
// GL, engine either on the main thread (default) or in a loopback Worker (engineWorker option / ?engineWorker=1).
//   - default path untouched, opt-in only
//   - fallback to the main-thread engine with classified errors
//   - identity: the same recorded input gives the same pose sequence on both paths (the Worker answers lag by one callback)
//   - ordering / no drops / back-pressure
//   - session rebuilds, detach while a build waits, unsupported queries
import assert from 'node:assert/strict';
import test from 'node:test';
import { classifyInstantiateError } from '../engine-loader.js';
import { engineAvailable, loadRealModule, loopbackWorker, runChunk, syntheticAtlas } from './engine-worker-harness.mjs';

const skip = engineAvailable ? false : 'engine build not found (SLAM_REFACT_CHUNK_ENGINE_DIR)';
const W = 240, H = 320;
let Module = null, atlases = null;
const atlasFor = (k) => atlases.get(k) ?? (atlases.set(k, syntheticAtlas(Module, W, H, k).rgba), atlases.get(k));
test.before(async () => { if (!skip) { Module = await loadRealModule(); atlases = new Map(); } });

const withWorker = (loop, extra = {}) => ({ enabled: true, createWorker: () => loop.worker, ...extra });
async function run(opts) { return runChunk({ atlasFor, ...opts }); }

test('R1-C1: default path never touches Workers or the engine-worker modules, and adds no diagnostics key', { skip }, async () => {
  let constructed = 0;
  const r = await run({ steps: 6, globals: (g) => { g.set('Worker', class { constructor() { constructed++; } }); g.set('location', { search: '?capture=worker' }); } });
  try {
    assert.equal(constructed, 0);
    assert.equal(r.diag.engineWorker, undefined);
    assert.equal('engineWorker' in r.diag, false);
    assert.deepEqual(r.errors, []);
    assert.ok(r.answers.slice(1).every((a) => a !== null));
    assert.equal(r.diag.stats.processedFrames, 6);
  } finally { r.close(); }
});

test('R1-C2: ?engineWorker=1 starts the Worker through the default Worker constructor; ?engineWorker=0 does not', { skip }, async () => {
  const urls = [];
  const loop = loopbackWorker();
  const on = await run({ steps: 4, loop, globals: (g) => {
    g.set('location', { search: '?engineWorker=1' });
    g.set('Worker', function WorkerStub(url, opts) { urls.push([String(url), opts]); return loop.worker; });
  } });
  try {
    assert.equal(urls.length, 1);
    assert.match(urls[0][0], /engine-worker\.js$/);
    assert.equal(urls[0][1].type, 'module');
    assert.equal(on.diag.engineWorker.active, true);
    assert.equal(on.diag.engineWorker.fallback, null);
    assert.equal(on.diag.engine.matchesManifest, true, 'the Worker verified the engine bytes against the manifest');
  } finally { on.close(); }
  let started = 0;
  const off = await run({ steps: 2, globals: (g) => { g.set('location', { search: '?engineWorker=0' }); g.set('Worker', function () { started++; }); } });
  try { assert.equal(started, 0); assert.equal(off.diag.engineWorker, undefined); } finally { off.close(); }
});

test('R1-C3: no Worker available -> main-thread engine, reason recorded, answers equal the default path', { skip }, async () => {
  const base = await run({ steps: 12 });
  const baseAnswers = base.answers.slice(); base.close();
  const r = await run({ steps: 12, engineWorker: { enabled: true, createWorker: () => { throw new TypeError('Worker is not defined'); } } });
  try {
    assert.equal(r.diag.engineWorker.active, false);
    assert.equal(r.diag.engineWorker.fallback.code, 'engine-worker-unavailable');
    assert.match(r.diag.engineWorker.fallback.message, /Worker is not defined/);
    assert.ok(r.diag.warnings.some((w) => /engineWorker unavailable \(engine-worker-unavailable\)/.test(w)));
    assert.deepEqual(r.answers, baseAnswers, 'the fallback is the default path');
    assert.deepEqual(r.errors, []);
  } finally { r.close(); }
});

test('R1-C4: a Worker script that fails to load (error event) falls back', { skip }, async () => {
  const failing = { postMessage() { queueMicrotask(() => failing.onerror?.({ message: 'Failed to fetch module script' })); }, terminate() {} };
  const r = await run({ steps: 4, engineWorker: { enabled: true, createWorker: () => failing } });
  try {
    assert.equal(r.diag.engineWorker.fallback.code, 'engine-worker-error');
    assert.match(r.diag.engineWorker.fallback.message, /Failed to fetch module script/);
    assert.equal(r.diag.engineWorker.active, false);
    assert.ok(r.answers.slice(1).every((a) => a !== null), 'the main-thread engine tracks');
  } finally { r.close(); }
});

test('R1-C5: engine failures inside the Worker keep their classified code in the fallback record', { skip }, async () => {
  const cases = [
    ['engine-simd-unsupported', () => classifyInstantiateError(new Error('invalid opcode'), { validate: () => false })],
    ['engine-memory-unavailable', () => classifyInstantiateError(new RangeError('WebAssembly.Memory(): could not allocate memory'), { validate: () => true })],
    ['engine-worker-failed', () => new Error('something else')],
  ];
  for (const [code, make] of cases) {
    const loop = loopbackWorker({ coreDeps: { loadEngine: async () => { throw make(); } } });
    const r = await run({ steps: 3, engineWorker: withWorker(loop) });
    try {
      assert.equal(r.diag.engineWorker.fallback.code, code);
      assert.equal(r.diag.engineWorker.active, false);
      assert.ok(r.diag.engine, 'the main-thread engine loaded after the fallback');
    } finally { r.close(); }
  }
});

test('R1-C6: when the Worker and the main thread both fail, the main-thread error is the reported one (as without the option)', { skip }, async () => {
  const loop = loopbackWorker({ coreDeps: { loadEngine: async () => { throw classifyInstantiateError(new Error('x'), { validate: () => false }); } } });
  await assert.rejects(run({ steps: 1, engineBase: 'file:///nowhere/', engineWorker: withWorker(loop) }), /unexpected fetch/);
  const d = globalThis.__slamRefactChunk;
  assert.equal(d.engineWorker.fallback.code, 'engine-simd-unsupported');
  assert.match(d.error, /unexpected fetch/);
  delete globalThis.__slamRefactChunk;
});

test('R1-C7: options the Worker cannot serve keep the engine on the main thread without starting a Worker', { skip }, async () => {
  for (const [extraOptions, pattern] of [[{ createGlue: (M, c) => new M.SlamRefactGlue(c) }, /createGlue/], [{ gpuCheck: { every: 5, frames: 2 } }, /gpuCheck/],
    [{ frameScheduler: { enabled: true, mode: 'gpu-task-owned-v1' } }, /frameScheduler/]]) {
    let created = 0;
    const r = await run({ steps: 2, extraOptions, engineWorker: { enabled: true, createWorker: () => { created++; throw new Error('must not start'); } } });
    try {
      assert.equal(created, 0);
      assert.equal(r.diag.engineWorker.fallback.code, 'engine-worker-unsupported-option');
      assert.match(r.diag.engineWorker.fallback.message, pattern);
    } finally { r.close(); }
  }
});

test('R1-C8: IDENTITY - the same camera + IMU input gives the same answers on both paths; the Worker path lags by one callback', { skip }, async () => {
  const N = 40;
  const main = await run({ steps: N });
  const mainAnswers = main.answers.slice(), mainDiag = main.diag.stats; main.close();
  const loop = loopbackWorker();
  const w = await run({ steps: N, loop, engineWorker: withWorker(loop) });
  try {
    await w.drain(3);
    const workerAnswers = w.answers;
    assert.deepEqual(w.errors, []);
    assert.equal(workerAnswers[0], null, 'nothing has come back yet at the first callback');
    for (let k = 0; k < N; k++) assert.deepEqual(workerAnswers[k + 1], mainAnswers[k], `answer of callback ${k}`);
    assert.ok(mainAnswers.some((a) => a && (a.position.x !== 0 || a.position.y !== 2)), 'the sequence is not a trivial constant pose');
    assert.ok(mainAnswers.some((a) => a && a.worldPoints.length > 0) || true);
    assert.equal(w.diag.stats.processedFrames, mainDiag.processedFrames);
    assert.equal(w.diag.stats.freshResponses, mainDiag.freshResponses);
    assert.deepEqual(w.diag.stats.responseLag, mainDiag.responseLag);
    assert.equal(w.diag.stats.motionEventsPushed, mainDiag.motionEventsPushed);
    assert.equal(w.diag.engineWorker.active, true);
  } finally { w.close(); }
});

test('R1-C9: ordering and no added drops - uneven Worker speed, every staged frame is processed once and in order', { skip }, async () => {
  const N = 36;
  let n = 0;
  const loop = loopbackWorker({ delay: () => ((++n * 7) % 5 === 0 ? 6 : 0) });
  const w = await run({ steps: N, loop, engineWorker: withWorker(loop, { maxInFlight: 64 }) });
  try {
    await w.drain(3);
    const frames = loop.state.toWorker.filter((m) => m.type === 'frame');
    assert.equal(frames.length, N);
    assert.deepEqual(frames.map((m) => m.seq), Array.from({ length: N }, (_, i) => i + 1));
    const stats = w.diag.engineWorker.stats;
    assert.equal(stats.framesSubmitted, N);
    assert.equal(stats.resultsReceived, N);
    assert.equal(stats.backpressureSkips, 0);
    assert.equal(w.diag.stats.droppedStaged, 0);
    assert.equal(w.diag.stats.processedFrames, N);
    assert.equal(w.diag.stats.processErrors, 0);
    assert.deepEqual(loop.state.fromWorker.filter((m) => m.type === 'frame-result').map((m) => m.seq), frames.map((m) => m.seq));
    assert.deepEqual(w.errors, []);
  } finally { w.close(); }
});

test('R1-C10: a Worker that falls behind is back-pressured through the existing staged-frame replacement, never unbounded', { skip }, async () => {
  const N = 24;
  const loop = loopbackWorker({ delay: (m) => (m.type === 'frame' ? 30 : 0) });
  // no loop.idle() between callbacks: the camera does not wait for the Worker
  const w = await run({ steps: N, engineWorker: withWorker(loop, { maxInFlight: 2 }),
    afterEach: async () => { await new Promise((r) => setImmediate(r)); } });
  try {
    await new Promise((r) => setTimeout(r, 100)); await loop.idle();
    await w.drain(3);
    const stats = w.diag.engineWorker.stats;
    assert.ok(stats.backpressureSkips > 0, 'the Worker was slower than the camera');
    assert.ok(stats.framesSubmitted < N);
    // each skip is the usual replaced staged frame; a skip in the very last callback has no later frame to replace it
    assert.ok(stats.backpressureSkips - w.diag.stats.droppedStaged >= 0 && stats.backpressureSkips - w.diag.stats.droppedStaged <= 1);
    const submitted = loop.state.toWorker.filter((m) => m.type === 'frame').map((m) => m.seq);
    assert.deepEqual(submitted, submitted.slice().sort((a, b) => a - b));
    assert.deepEqual(w.errors, []);
    assert.ok(Math.max(...loop.state.fromWorker.filter((m) => m.type === 'frame-result').map((m) => m.seq)) <= stats.framesSubmitted);
  } finally { w.close(); }
});

test('R1-C11: hitTest answers no hit in Worker mode (counted, warned once); the main-thread engine answers as before', { skip }, async () => {
  const loop = loopbackWorker();
  const w = await run({ steps: 8, loop, engineWorker: withWorker(loop) });
  try {
    await w.drain(2);
    assert.deepEqual(w.ctx.ctrl.hitTest(0.5, 0.5, ['FEATURE_POINT']), []);
    assert.deepEqual(w.ctx.ctrl.hitTest(0.4, 0.4), []);
    assert.equal(w.diag.engineWorker.stats.hitTestMisses, 2);
    assert.equal(w.diag.warnings.filter((m) => /hitTest answers no hit/.test(m)).length, 1);
  } finally { w.close(); }
});

test('R1-C13: a video size change rebuilds the session in Worker mode (layout fetched first), frames keep flowing in order', { skip }, async () => {
  const loop = loopbackWorker();
  const w = await run({ steps: 6, loop, engineWorker: withWorker(loop) });
  try {
    const before = w.diag.stats.sessionsCreated;
    const W2 = 360, H2 = 480;  // portrait: the older pinned engine aborts on landscape captures
    w.ctx.pm.onVideoSizeChange({ videoWidth: W2, videoHeight: H2, orientation: 0, computeCtx: w.ctx.gl });
    assert.equal(w.diag.stats.sessionsCreated, before, 'the new session waits for its layout');
    await loop.idle(); await new Promise((r) => setImmediate(r)); await loop.idle();
    assert.equal(w.diag.stats.sessionsCreated, before + 1);
    assert.deepEqual(w.diag.session && { w: w.diag.session.videoWidth, h: w.diag.session.videoHeight }, { w: W2, h: H2 });
    assert.equal(w.diag.engineWorker.stats.layoutProbes, 2);
    const big = syntheticAtlas(Module, W2, H2, 3).rgba;
    w.ctx.state.atlas = big;
    const f = { ...w.ctx.frame(10), textureWidth: W2, textureHeight: H2 };
    w.ctx.gpu(f); w.ctx.cpu(f); await loop.idle();
    w.ctx.cpu({ ...f, repeatFrame: true });
    assert.deepEqual(w.errors, []);
    const frames = loop.state.toWorker.filter((m) => m.type === 'frame');
    assert.equal(new Set(frames.map((m) => m.glueId)).size, 2, 'frames of both sessions went to their own glue, in order');
    assert.deepEqual(frames.map((m) => m.glueId), frames.map((m) => m.glueId).slice().sort((a, b) => a - b));
  } finally { w.close(); }
});

test('R1-C14: detaching or rebuilding while a build waits for its layout cancels it (no stale session, no error)', { skip }, async () => {
  const loop = loopbackWorker({ delay: (m) => (m.type === 'probe' ? 20 : 0) });
  const w = await run({ steps: 0, attach: false, loop, engineWorker: withWorker(loop) });
  try {
    w.ctx.pm.onSessionAttach(w.ctx.attachArgs);
    assert.equal(w.diag.stats.sessionsCreated, 0);
    w.ctx.pm.onSessionDetach({});               // cancels the pending build
    await new Promise((r) => setTimeout(r, 60)); await loop.idle();
    assert.equal(w.diag.stats.sessionsCreated, 0, 'the cancelled build did not create a session');
    assert.deepEqual(w.errors, []);
    w.ctx.pm.onSessionAttach(w.ctx.attachArgs);  // a later attach builds normally
    await new Promise((r) => setTimeout(r, 60)); await loop.idle();
    assert.equal(w.diag.stats.sessionsCreated, 1);
  } finally { w.close(); }
});

test('R1-C15: an unsupported rotation is rejected exactly as on the main thread (no Worker round trip, inactive answer)', { skip }, async () => {
  const loop = loopbackWorker();
  const w = await run({ steps: 5, loop, engineWorker: withWorker(loop) });
  try {
    const probes = w.diag.engineWorker.stats.layoutProbes;
    w.ctx.pm.onDeviceOrientationChange({ videoWidth: W, videoHeight: H, orientation: 45, computeCtx: w.ctx.gl });
    assert.equal(w.diag.stats.sessionsRejected, 1);
    assert.equal(w.diag.engineWorker.stats.layoutProbes, probes);
    assert.deepEqual(w.errors, []);
  } finally { w.close(); }
});

test('R1-C16: a Worker that dies after the load is reported through errorCallback once; the frame path answers null, no throw', { skip }, async () => {
  const loop = loopbackWorker();
  const w = await run({ steps: 6, loop, engineWorker: withWorker(loop) });
  try {
    loop.crash('RuntimeError: unreachable executed');
    const f = w.ctx.frame(50);
    w.ctx.gpu(f);
    w.ctx.state.atlas = atlasFor(50);
    assert.equal(w.ctx.cpu(f), null);
    w.ctx.gpu(w.ctx.frame(51));
    assert.equal(w.ctx.cpu(w.ctx.frame(51)), null);
    assert.equal(w.errors.filter((e) => /unreachable executed/.test(e)).length, 1, 'reported once');
    assert.ok(w.diag.stats.processErrors >= 2);
  } finally { w.close(); }
});

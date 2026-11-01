// node --test slam_refact/web-chunk/tests/ios-chunk-hardening.test.mjs
// Chunk-level iPhone hardening (engine-loader failure causes, raw DeviceMotion interval record) and the B1' notes
// (GL-error report-once, attach failure). iOS WebKit execution is UNVERIFIED on this host: these tests only prove the
// observable behaviour with injected errors in node.
import test from 'node:test';
import assert from 'node:assert/strict';
import { createHash } from 'node:crypto';
import { classifyInstantiateError, loadEngine } from '../engine-loader.js';
import { createSensors } from '../sensors.js';
import { fixture } from './frame-scheduler-fixture.mjs';

const turn = () => new Promise((r) => setTimeout(r, 5));

test('classifyInstantiateError tells SIMD, memory and other failures apart', () => {
  const noSimd = { validate: () => false }, simd = { validate: () => true };
  const a = classifyInstantiateError(new Error('WebAssembly.instantiate(): invalid value type 0x7b'), noSimd);
  assert.equal(a.code, 'engine-simd-unsupported'); assert.match(a.message, /SIMD/); assert.match(a.cause.message, /0x7b/);
  const b = classifyInstantiateError(new RangeError('WebAssembly.Memory(): could not allocate memory'), simd);
  assert.equal(b.code, 'engine-memory-unavailable'); assert.match(b.message, /256 MB/);
  assert.equal(classifyInstantiateError(new Error('Out of memory: Cannot allocate Wasm memory for new instance'), simd).code, 'engine-memory-unavailable');
  const other = new Error('Aborted(unrelated)');
  assert.equal(classifyInstantiateError(other, simd), other, 'unrelated errors are rethrown unchanged');
  assert.equal(classifyInstantiateError(other, null).code, 'engine-simd-unsupported', 'no WebAssembly at all');
});

async function loaderWith(factorySource, validate) {
  const sha = (b) => createHash('sha256').update(b).digest('hex');
  const js = Buffer.from(factorySource), wasm = Buffer.from([0, 97, 115, 109, 1, 0, 0, 0]);
  const manifest = Buffer.from(JSON.stringify({ schema: 'slam-refact-chunk-engine-v1',
    artifacts: { js: { file: 'e.js', sha256: sha(js) }, wasm: { file: 'e.wasm', sha256: sha(wasm) } } }));
  const files = { 'manifest.json': manifest, 'e.js': js, 'e.wasm': wasm };
  const saved = { fetch: globalThis.fetch, Blob: globalThis.Blob, create: URL.createObjectURL, revoke: URL.revokeObjectURL, validate: WebAssembly.validate };
  globalThis.fetch = async (url) => { const b = files[String(url).split('/').pop()];
    return { ok: true, status: 200, arrayBuffer: async () => b.buffer.slice(b.byteOffset, b.byteOffset + b.byteLength) }; };
  globalThis.Blob = class { constructor(parts) { this.parts = parts; } };
  URL.createObjectURL = (blob) => `data:text/javascript;base64,${Buffer.concat(blob.parts.map((p) => Buffer.from(typeof p === 'string' ? p : new Uint8Array(p)))).toString('base64')}`;
  URL.revokeObjectURL = () => {};
  if (validate) WebAssembly.validate = validate;
  try { return await loadEngine('http://x.test/engine/'); }
  finally { Object.assign(globalThis, { fetch: saved.fetch, Blob: saved.Blob });
    URL.createObjectURL = saved.create; URL.revokeObjectURL = saved.revoke; WebAssembly.validate = saved.validate; }
}

test('loadEngine surfaces a memory allocation failure and a SIMD-less runtime as distinct errors; success path is unchanged', async () => {
  await assert.rejects(loaderWith('var SlamRefactEngine = async () => { throw new RangeError("WebAssembly.Memory(): could not allocate memory"); };'),
    (e) => e.code === 'engine-memory-unavailable' && e.cause instanceof RangeError);
  await assert.rejects(loaderWith('var SlamRefactEngine = async () => { throw new Error("invalid value type 0x7b"); };', () => false),
    (e) => e.code === 'engine-simd-unsupported');
  await assert.rejects(loaderWith('var SlamRefactEngine = async () => { throw new Error("boom"); };'),
    (e) => e.message === 'boom' && e.code === undefined);
  const ok = await loaderWith('var SlamRefactEngine = async (args) => ({ args });');
  assert.equal(ok.Module.args.wasmBinary.length, 8); assert.equal(ok.loaded.matchesManifest, true);
});

test('delivery diagnostics keep the raw DeviceMotion interval; the default queue entry is untouched', () => {
  globalThis.window = globalThis;
  const on = createSensors(() => false, { enabled: true, maxEntries: 8 }), off = createSensors(() => false);
  const mk = (interval) => ({ timeStamp: 1000, interval, acceleration: null, accelerationIncludingGravity: { x: 0, y: 0, z: -9.8 }, rotationRate: null });
  for (const v of [16, 0.016, undefined, 'x']) on.updateDeviceMotion(mk(v));
  assert.deepEqual(on.delivery.snapshot().timings.entries.map((e) => e.rawInterval), [16, 0.016, null, null]);
  off.updateDeviceMotion(mk(16));
  const [entry] = off.takeForStage();
  assert.deepEqual(Object.keys(entry), ['timestampNanos', 'eventTimestampNanos', 'interval', 'acceleration', 'accelerationIncludingGravity', 'rotationRate']);
  assert.equal(off.delivery, undefined);
});

test('B1prime persistent preexisting GL error: dropped every frame, reported once, reset after a clean capture', async () => {
  await fixture(true, async (s) => {
    s.drawError(0x0502, true);
    for (let k = 0; k < 5; k++) s.stage(s.frame(k));
    const c = s.scheduler().counters;
    assert.equal(c.droppedGlError, 5); assert.equal(c.barrierResets, 0);
    assert.equal(s.errors.filter((e) => /preexisting GL error/.test(e)).length, 1);
    assert.equal(s.scheduler().failures.filter((f) => f.where === 'scheduler-capture-gl-preexisting').length, 1);
    s.drawError(0); s.stage(s.frame(5));
    for (let i = 0; i < 100 && !s.scheduler().ready; i++) await turn();
    assert.equal(s.scheduler().counters.admitted, 1);
    s.cpu(s.frame(6));  // priming ACK frees the next admission
    s.drawError(0x0502, true); s.stage(s.frame(7));
    assert.equal(s.scheduler().counters.droppedGlError, 6);
    assert.equal(s.scheduler().failures.filter((f) => f.where === 'scheduler-capture-gl-preexisting').length, 2, 'a new run is recorded again');
    assert.equal(s.errors.filter((e) => /preexisting GL error/.test(e)).length, 1, 'reportOnce dedups the same message');
  });
});

test('B1prime attach failure is recorded, does not throw, leaves no session and a later attach recovers', async () => {
  await fixture(true, async (s) => {
    assert.equal(s.errors.filter((e) => /injected createGlue failure/.test(e)).length, 1);
    const sc = s.scheduler();
    assert.equal(sc.epoch, null); assert.equal(sc.counters.errors, 1);
    assert.ok(sc.failures.some((f) => f.where === 'scheduler-attach'));
    assert.doesNotThrow(() => { s.stage(s.frame(0)); assert.equal(s.cpu(s.frame(1)), null); });
    s.pm.onSessionDetach({});
    s.pm.onSessionAttach({ videoWidth: 240, videoHeight: 320, canvasWidth: 240, canvasHeight: 320, rotation: 0,
      computeCtx: s.gl, framework: s.framework, GLctx: s.gl, sessionAttributes: { fillsCameraTexture: true } });
    assert.notEqual(s.scheduler().epoch, null);
    assert.equal(s.errors.filter((e) => /injected createGlue failure/.test(e)).length, 1);
  }, undefined, { failAttachGlue: true });
});

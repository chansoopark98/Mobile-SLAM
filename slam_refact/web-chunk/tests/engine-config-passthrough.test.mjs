// node --test slam_refact/web-chunk/tests/engine-config-passthrough.test.mjs
// A-1a: SLAM_REFACT_CHUNK_OPTIONS.engineConfig = {intrinsics, accuracy} reaches the glue constructor unchanged (opt-in, default OFF),
// the sent values are readable in diag.session, malformed values are `[XR]` errors, and without the option the constructor
// argument is exactly what it was before. Uses the real engine WASM from the harness (skipped when it is not available).
import assert from 'node:assert/strict';
import fs from 'node:fs';
import path from 'node:path';
import test from 'node:test';
import { engineAvailable, loadRealModule, loopbackWorker, runChunk, syntheticAtlas, chunkDir } from './engine-worker-harness.mjs';

const skip = engineAvailable ? false : 'engine build not found (SLAM_REFACT_CHUNK_ENGINE_DIR)';
const W = 240, H = 320;
let Module = null;
const atlases = new Map();
test.before(async () => { if (!skip) Module = await loadRealModule(); });
const atlasFor = (k) => atlases.get(k) ?? (atlases.set(k, syntheticAtlas(Module, W, H, k).rgba), atlases.get(k));

// Runs the chunk and returns the config objects the glue constructor received (createGlue hook wraps the real glue).
async function runCapturing({ engineConfig, steps = 4, ...rest }) {
  const seen = [];
  const createGlue = (M, config) => { seen.push(structuredClone(config)); return new M.SlamRefactGlue(config); };
  const extraOptions = { createGlue, ...(engineConfig === undefined ? {} : { engineConfig }) };
  const r = await runChunk({ atlasFor, steps, extraOptions, ...rest });
  return { r, seen };
}

const BASELINE_GLUE_CONFIG = { captureWidth: W, captureHeight: H, rotationDeg: 0, manufacturer: 'Google', model: 'Pixel 7',  // the harness's fake XR8.XrDevice.deviceEstimate
  estimateScale: false, disableVio: false, leftHandedAxes: false, mirroredDisplay: false };

test('A-1a-1: without engineConfig the glue constructor argument and the session diagnostics are unchanged', { skip }, async () => {
  const { r, seen } = await runCapturing({});
  try {
    assert.deepEqual(r.errors, []);
    assert.ok(seen.length >= 1);
    for (const c of seen) assert.deepEqual(c, { ...BASELINE_GLUE_CONFIG, ...(c.framesUntilRecenter === undefined ? {} : { framesUntilRecenter: c.framesUntilRecenter }) });
    assert.equal('intrinsics' in seen[0], false);
    assert.equal('accuracy' in seen[0], false);
    assert.deepEqual(r.diag.session.glueConfig, seen[0]);
    assert.equal('engineConfig' in r.diag.session, false);
  } finally { r.close(); }
});

test('A-1a-1b: an engineConfig that carries no key (null values / empty) changes nothing the engine receives', { skip }, async () => {
  for (const engineConfig of [{}, { intrinsics: null, accuracy: undefined }]) {
    const { r, seen } = await runCapturing({ engineConfig });
    try {
      assert.deepEqual(r.errors, []);
      assert.deepEqual(seen[0], BASELINE_GLUE_CONFIG);
      assert.deepEqual(r.diag.session.engineConfig.keys, []);
    } finally { r.close(); }
  }
});

test('A-1a-2: engineConfig is passed to the glue as given, and the sent copy is in diag.session', { skip }, async () => {
  const engineConfig = { intrinsics: [201.5, 202.5, 119.5, 159.5], accuracy: { enabled: false, baWindow: 8 } };
  const given = structuredClone(engineConfig);
  const { r, seen } = await runCapturing({ engineConfig });
  try {
    assert.deepEqual(r.errors, []);
    assert.deepEqual(seen[0].intrinsics, given.intrinsics);
    assert.deepEqual(seen[0].accuracy, given.accuracy);
    assert.deepEqual({ ...seen[0], intrinsics: undefined, accuracy: undefined }, { ...BASELINE_GLUE_CONFIG, intrinsics: undefined, accuracy: undefined });
    const s = r.diag.session;
    assert.deepEqual(s.glueConfig.intrinsics, given.intrinsics);
    assert.deepEqual(s.glueConfig.accuracy, given.accuracy);
    assert.deepEqual(s.engineConfig, { keys: ['intrinsics', 'accuracy'], transport: 'main-thread' });
    // The sent copy is detached: mutating the diagnostics or the caller's object cannot alter what later sessions send.
    s.glueConfig.accuracy.baWindow = 99;
    assert.deepEqual(engineConfig, given);
    // The engine took the intrinsics when it exposes the readback.
    if (s.configurationReadback.captureIntrinsics) assert.equal(s.configurationReadback.captureIntrinsics.fx, 201.5);
  } finally { r.close(); }
});

test('A-1a-3: malformed engineConfig values are [XR] errors', { skip }, async () => {
  const bad = [
    [null, /engineConfig must be an object/], [[1, 2], /engineConfig must be an object/], ['x', /engineConfig must be an object/],
    [{ intrinsics: [1, 2, 3] }, /intrinsics must be \[fx, fy, cx, cy\]/], [{ intrinsics: [1, 2, 3, NaN] }, /four finite numbers/],
    [{ intrinsics: '1,2,3,4' }, /intrinsics must be/], [{ accuracy: 5 }, /accuracy must be an object/], [{ accuracy: [] }, /accuracy must be an object/],
    [{ focal: 500 }, /unsupported key\(s\) focal/],
  ];
  for (const [engineConfig, pattern] of bad) {
    let message = null;
    let r = null;
    try {
      ({ r } = await runCapturing({ engineConfig, steps: 2 }));
      message = r.errors.join('\n') + '\n' + (r.diag.errors ?? []).join('\n');
    } catch (error) { message = String(error && error.message || error); }
    finally { r?.close(); }
    assert.match(message, /\[XR\]/, JSON.stringify(engineConfig));
    assert.match(message, pattern, JSON.stringify(engineConfig));
  }
});

test('A-1a-4: engineWorker mode forwards engineConfig in the create message and reports the transport', { skip }, async () => {
  const engineConfig = { intrinsics: [201.5, 202.5, 119.5, 159.5], accuracy: { enabled: false } };
  const loop = loopbackWorker();
  const r = await runChunk({ atlasFor, steps: 4, loop, engineWorker: { enabled: true, createWorker: () => loop.worker }, extraOptions: { engineConfig } });
  try {
    assert.deepEqual(r.errors, []);
    assert.equal(r.diag.engineWorker.active, true);
    const creates = loop.state.toWorker.filter((m) => m.type === 'create');
    assert.ok(creates.length >= 1);
    assert.deepEqual(creates[0].config.intrinsics, engineConfig.intrinsics);
    assert.deepEqual(creates[0].config.accuracy, engineConfig.accuracy);
    assert.deepEqual(r.diag.session.engineConfig, { keys: ['intrinsics', 'accuracy'], transport: 'engine-worker' });
    assert.deepEqual(r.diag.session.glueConfig.intrinsics, engineConfig.intrinsics);
  } finally { r.close(); }
});

test('A-1a-5: engineWorker requested but unavailable -> main-thread engine receives engineConfig, the fallback reason is reported', { skip }, async () => {
  const engineConfig = { intrinsics: [201.5, 202.5, 119.5, 159.5] };
  const r = await runChunk({ atlasFor, steps: 3, engineWorker: { enabled: true, createWorker: () => { throw new TypeError('Worker is not defined'); } }, extraOptions: { engineConfig } });
  try {
    assert.deepEqual(r.errors, []);
    assert.equal(r.diag.engineWorker.active, false);
    assert.equal(r.diag.engineWorker.fallback.code, 'engine-worker-unavailable');
    assert.deepEqual(r.diag.session.engineConfig, { keys: ['intrinsics'], transport: 'main-thread' });
    assert.deepEqual(r.diag.session.glueConfig.intrinsics, engineConfig.intrinsics);
  } finally { r.close(); }
});

test('A-1a-6: no shipped sample or page turns engineConfig on by default', () => {
  const hits = [];
  const walk = (dir, root) => {
    for (const e of fs.readdirSync(dir, { withFileTypes: true })) {
      const p = path.join(dir, e.name);
      if (e.isDirectory()) { if (!['node_modules', 'tests', '__pycache__'].includes(e.name)) walk(p, root); }
      else if (/\.(js|mjs|html)$/.test(e.name) && /engineConfig/.test(fs.readFileSync(p, 'utf8'))) hits.push(path.relative(root, p));
    }
  };
  walk(chunkDir, chunkDir);
  const smokeDir = path.resolve(chunkDir, '../web-smoke');
  if (fs.existsSync(smokeDir)) walk(smokeDir, path.resolve(chunkDir, '..'));
  assert.deepEqual(hits, ['xr-slam.js']);
});

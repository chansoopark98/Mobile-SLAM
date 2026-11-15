// node --test slam_refact/web-chunk/tests/engine-config-effective.test.mjs
// A-1a integration with the real engine: what the chunk sent (diag.session.glueConfig) is what the engine reports it uses.
// Needs an engine that has SlamRefactGlue#getEffectiveConfig (FX-D3a); otherwise every test is skipped.
//   SLAM_REFACT_CHUNK_ENGINE_DIR=<dir with manifest.json + slam_refact_engine.{js,wasm}> (e.g. the G2 build's wasm directory)
// The shape read here follows fix-plan FX-D3a: intrinsics (4 values), accuracy.enabled, ignoredKeys.
import assert from 'node:assert/strict';
import test from 'node:test';
import { engineAvailable, loadRealModule, runChunk, syntheticAtlas } from './engine-worker-harness.mjs';

const W = 240, H = 320;
let Module = null, skip = engineAvailable ? false : 'engine build not found (SLAM_REFACT_CHUNK_ENGINE_DIR)';
const atlases = new Map();
test.before(async () => {
  if (skip) return;
  Module = await loadRealModule();
  if (typeof Module.SlamRefactGlue.prototype.getEffectiveConfig !== 'function') skip = 'this engine has no getEffectiveConfig (FX-D3a)';
});
const atlasFor = (k) => atlases.get(k) ?? (atlases.set(k, syntheticAtlas(Module, W, H, k).rgba), atlases.get(k));

async function effectiveOf(engineConfig) {
  const glues = [];
  const createGlue = (M, config) => { const g = new M.SlamRefactGlue(config); glues.push(g); return g; };
  const r = await runChunk({ atlasFor, steps: 3, extraOptions: { createGlue, ...(engineConfig ? { engineConfig } : {}) } });
  try {
    assert.deepEqual(r.errors, []);
    return { sent: structuredClone(r.diag.session.glueConfig), effective: JSON.parse(JSON.stringify(glues[0].getEffectiveConfig())) };
  } finally { r.close(); }
}
const numbersIn = (v) => (Array.isArray(v) ? v : Object.values(v)).filter((x) => typeof x === 'number');

// `skip` is read when the test body starts, after test.before has run.
const guarded = (name, fn) => test(name, async (t) => { if (skip) return t.skip(skip); await fn(); });

guarded('A-1a-7: engine-reported intrinsics and accuracy equal the values the chunk sent', async () => {
  const engineConfig = { intrinsics: [201.5, 202.5, 119.5, 159.5], accuracy: { enabled: true, metricOutput: true } };
  const { sent, effective } = await effectiveOf(engineConfig);
  assert.deepEqual(sent.intrinsics, engineConfig.intrinsics);
  assert.deepEqual(numbersIn(effective.intrinsics).slice(0, 4), engineConfig.intrinsics);
  assert.equal(effective.accuracy.enabled, true);
  assert.equal(effective.accuracy.metricOutput, true);
  assert.deepEqual(effective.ignoredKeys, []);
});

guarded('A-1a-8: a key the engine ignores is visible in ignoredKeys (the chunk sends it unchanged)', async () => {
  const { sent, effective } = await effectiveOf({ accuracy: { enabled: true, notAKey: 1 } });
  assert.equal(sent.accuracy.notAKey, 1);
  assert.deepEqual(effective.ignoredKeys, ['accuracy.notAKey']);
});

guarded('A-1a-9: without engineConfig the engine reports accuracy off and nothing ignored', async () => {
  const { sent, effective } = await effectiveOf(null);
  assert.equal('accuracy' in sent, false);
  assert.equal(effective.accuracy.enabled, false);
  assert.deepEqual(effective.ignoredKeys, []);
});

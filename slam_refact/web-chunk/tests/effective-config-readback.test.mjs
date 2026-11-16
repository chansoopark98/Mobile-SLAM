// node --test slam_refact/web-chunk/tests/effective-config-readback.test.mjs
// The chunk keeps the engine's getEffectiveConfig() result at diag.session.configurationReadback.effectiveConfig (one call per engine
// construction) and the phone page's readEngineConfigReadback() turns it into "which accuracy flags the engine really runs with".
// The glue is the real engine glue behind a Proxy (createGlue hook) whose getEffectiveConfig is scripted, so the tests do not depend on
// whether the engine WASM knows k1KeepMap. Real engine WASM from the harness (skipped when it is not available).
import assert from 'node:assert/strict';
import test from 'node:test';
import { engineAvailable, loadRealModule, loopbackWorker, runChunk, syntheticAtlas } from './engine-worker-harness.mjs';
import { readEngineConfigReadback } from '../samples/phone/metrics.js';
import { EXPERIMENTAL_ENGINE_CONFIGS, resolveEngineOptions } from '../samples/phone/product-config.js';

const skip = engineAvailable ? false : 'engine build not found (SLAM_REFACT_CHUNK_ENGINE_DIR)';
const W = 240, H = 320;
const K1 = EXPERIMENTAL_ENGINE_CONFIGS['1'].accuracy, K1F = EXPERIMENTAL_ENGINE_CONFIGS.fast.accuracy;
const SENT = { ...K1 };
let Module = null;
const atlases = new Map();
test.before(async () => { if (!skip) Module = await loadRealModule(); });
const atlasFor = (k) => atlases.get(k) ?? (atlases.set(k, syntheticAtlas(Module, W, H, k).rgba), atlases.get(k));

// effective(config) scripts the read-back; undefined removes the method (an engine build without it).
function scripted(effective, calls = []) {
  return (M, config) => {
    const real = new M.SlamRefactGlue(config);
    return new Proxy(real, { get(target, name) {
      if (name === 'getEffectiveConfig') return effective === undefined ? undefined : () => { calls.push(config.captureWidth); return effective(config); };
      const v = target[name];
      return typeof v === 'function' ? v.bind(target) : v;
    } });
  };
}
const echo = (ignoredKeys = []) => (config) => ({ captureWidth: config.captureWidth, accuracy: { enabled: false, k1KeepMap: false, ...(config.accuracy || {}) }, ignoredKeys });
async function readback(effective, engineConfig, extra = {}) {
  const calls = [];
  const r = await runChunk({ atlasFor, steps: 2, extraOptions: { createGlue: scripted(effective, calls), ...(engineConfig ? { engineConfig } : {}) }, ...extra });
  try { assert.deepEqual(r.errors, []); return { r, calls, session: JSON.parse(JSON.stringify(r.diag.session)) }; } finally { r.close(); }
}

test('RB-1: the keepmap=1 (K1) config reaches the engine and its read-back is exposed with k1KeepMap true (one call per engine construction)', { skip }, async () => {
  const { accuracy } = resolveEngineOptions('?keepmap=1');
  const { calls, session } = await readback(echo(), { accuracy });
  assert.deepEqual(calls, [W]);
  const eff = session.configurationReadback.effectiveConfig;
  assert.equal(eff.accuracy.k1KeepMap, true);
  assert.equal(eff.accuracy.enabled, K1.enabled);
  assert.deepEqual(readEngineConfigReadback({ session }, SENT), {
    available: true, k1KeepMap: true, ignoredKeys: [], k1KeepMapActive: true, allProductFlagsActive: true,
    flags: Object.fromEntries(Object.entries(K1).map(([k, v]) => [k, { sent: v, effective: true, ignored: false }])) });
});

test('RB-2: an engine that lists accuracy.k1KeepMap in ignoredKeys is never reported as running it', { skip }, async () => {
  const { session } = await readback(echo(['accuracy.k1KeepMap']), { accuracy: { ...K1 } });
  assert.deepEqual(session.configurationReadback.effectiveConfig.ignoredKeys, ['accuracy.k1KeepMap']);
  const info = readEngineConfigReadback({ session }, SENT);
  assert.equal(info.k1KeepMapActive, false);
  assert.equal(info.allProductFlagsActive, false);
  assert.equal(info.flags.k1KeepMap.ignored, true);
});

test('RB-3: a glue without getEffectiveConfig gives available:false', { skip }, async () => {
  const { session } = await readback(undefined, { accuracy: { ...K1 } });
  assert.deepEqual(session.configurationReadback.effectiveConfig, { available: false, reason: 'engine has no getEffectiveConfig' });
  assert.deepEqual(readEngineConfigReadback({ session }, SENT), { available: false, reason: 'engine has no getEffectiveConfig' });
});

test('RB-4: default mode (no keepmap, no engineConfig) reads back accuracy off', { skip }, async () => {
  assert.equal(resolveEngineOptions('').accuracy, null);
  const { session } = await readback(echo());
  assert.equal(session.configurationReadback.effectiveConfig.accuracy.enabled, false);
  assert.deepEqual(readEngineConfigReadback({ session }, null), { available: true, k1KeepMap: false, ignoredKeys: [], k1KeepMapActive: false, flags: {}, allProductFlagsActive: false });
});

test('RB-5: an engine rebuild (video size change) replaces the stored copy', { skip }, async () => {
  const calls = [];
  let before = null, after = null;
  const r = await runChunk({ atlasFor, steps: 2, extraOptions: { createGlue: scripted(echo(), calls), engineConfig: { accuracy: { ...K1 } } },
    afterEach: (k, ctx) => {
      if (k !== 1) return;
      before = ctx.diag().session.configurationReadback.effectiveConfig;
      ctx.pm.onVideoSizeChange({ videoWidth: 360, videoHeight: 480, orientation: 0, computeCtx: ctx.gl, framework: ctx.framework });
      after = ctx.diag().session.configurationReadback.effectiveConfig;
    } });
  try {
    assert.deepEqual(r.errors, []);
    assert.deepEqual(calls, [W, 360]);
    assert.equal(before.captureWidth, W);
    assert.equal(after.captureWidth, 360);
    assert.notEqual(before, after);
  } finally { r.close(); }
});

test('RB-6: the engine Worker path says so instead of guessing', { skip }, async () => {
  const loop = loopbackWorker();
  const r = await runChunk({ atlasFor, steps: 2, loop, engineWorker: { enabled: true, createWorker: () => loop.worker }, extraOptions: { engineConfig: { accuracy: { ...K1 } } } });
  try {
    assert.deepEqual(r.errors, []);
    assert.deepEqual(readEngineConfigReadback({ session: r.diag.session }), { available: false, reason: 'engine-worker' });
  } finally { r.close(); }
});

test('RB-7: the real engine glue (no scripted read-back) exposes a plain-data copy', { skip }, async () => {
  const r = await runChunk({ atlasFor, steps: 2, extraOptions: { engineConfig: { accuracy: { ...K1 } } } });
  try {
    assert.deepEqual(r.errors, []);
    const info = readEngineConfigReadback({ session: r.diag.session }, SENT);
    if (info.available === false) { assert.equal(info.reason, 'engine has no getEffectiveConfig'); return; }
    assert.ok(Array.isArray(info.ignoredKeys));
    // An engine build that does not know k1KeepMap lists it in ignoredKeys: then it must not be reported as active.
    assert.equal(info.allProductFlagsActive, Object.values(info.flags).every((f) => f.effective === f.sent && f.ignored === false));
    assert.equal(info.k1KeepMapActive, info.k1KeepMap === true && !info.ignoredKeys.includes('accuracy.k1KeepMap'));
  } finally { r.close(); }
});

test('RB-8: keepmap=fast (K1F) sends all three flags and reads them back; a build that ignores k1FastExtend is not reported active', { skip }, async () => {
  const { accuracy } = resolveEngineOptions('?keepmap=fast');
  assert.deepEqual(accuracy, K1F);
  const ok = await readback(echo(), { accuracy });
  const info = readEngineConfigReadback({ session: ok.session }, K1F);
  assert.deepEqual([info.allProductFlagsActive, info.flags.k1FastExtend], [true, { sent: true, effective: true, ignored: false }]);
  const ign = await readback(echo(['accuracy.k1FastExtend']), { accuracy });
  const info2 = readEngineConfigReadback({ session: ign.session }, K1F);
  assert.deepEqual([info2.k1KeepMapActive, info2.allProductFlagsActive, info2.flags.k1FastExtend.ignored], [true, false, true]);
});

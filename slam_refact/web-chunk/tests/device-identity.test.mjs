import assert from 'node:assert/strict';
import fs from 'node:fs';
import path from 'node:path';
import { createRequire } from 'node:module';
import { pathToFileURL } from 'node:url';
import test from 'node:test';
import { fakeGl, loadEngineModule } from '../tools/or1/players.mjs';

const repo = path.resolve(import.meta.dirname, '../../..');
const chunkDir = process.env.SLAM_REFACT_CHUNK_DIR ?? path.join(repo, 'slam_refact/web-chunk');
const engineDir = process.env.SLAM_REFACT_CHUNK_ENGINE_DIR
  ?? path.join(repo, 'build/slam-refact-full-20261019/chunk/rm4d-20261020/engine-pinned');
let serial = 0;

// Real loaded WASM, fake XR8/GL; the constructor arguments and session diagnostics are observed at their public boundary.
async function fixture(identity, navigatorObject, check, selectedChunkDir = chunkDir) {
  const saved = new Map();
  const set = (key, value) => {
    saved.set(key, Object.getOwnPropertyDescriptor(globalThis, key));
    Object.defineProperty(globalThis, key, { configurable: true, writable: true, value });
  };
  const listeners = new Map(), configs = [], errors = [];
  set('window', globalThis);
  set('navigator', navigatorObject);
  set('addEventListener', (name, cb) => { const list = listeners.get(name) ?? []; list.push(cb); listeners.set(name, list); });
  set('removeEventListener', (name, cb) => { listeners.set(name, (listeners.get(name) ?? []).filter((v) => v !== cb)); });
  set('_c8', { checkAllZeroes: 0 });
  set('XR8', { isPaused: () => false,
    XrDevice: { deviceEstimate: () => ({ manufacturer: '', model: 'K' }), isDeviceBrowserCompatible: () => true },
    XrConfig: { camera: () => ({ FRONT: 'front', BACK: 'back' }), device: () => ({ MOBILE: 'mobile', ANY: 'any' }) },
    XrPermissions: { permissions: () => ({ CAMERA: 'camera', DEVICE_MOTION: 'devicemotion', DEVICE_ORIENTATION: 'deviceorientation' }) },
    drawTexForComputeTex: (texture) => texture });
  const baseUrl = new URL('slam-refact-engine/', pathToFileURL(`${selectedChunkDir}/`)).href;
  set('fetch', async (url) => {
    assert.ok(String(url).startsWith(baseUrl));
    const bytes = fs.readFileSync(path.join(engineDir, String(url).slice(baseUrl.length)));
    return { ok: true, arrayBuffer: async () => bytes.buffer.slice(bytes.byteOffset, bytes.byteOffset + bytes.byteLength) };
  });
  set('Blob', class { constructor(parts) { this.parts = parts; } });
  set('require', createRequire(path.join(engineDir, 'slam_refact_engine.js')));
  set('__dirname', engineDir);
  const urls = { create: URL.createObjectURL, revoke: URL.revokeObjectURL };
  URL.createObjectURL = (blob) => `data:text/javascript;base64,${Buffer.concat(blob.parts.map((p) => Buffer.from(typeof p === 'string' ? p : new Uint8Array(p)))).toString('base64')}`;
  URL.revokeObjectURL = () => {};
  set('SLAM_REFACT_CHUNK_OPTIONS', { deviceIdentity: identity,
    createGlue: (Module, config) => { configs.push({ ...config }); return new Module.SlamRefactGlue(config); } });
  const framework = { dispatchEvent: () => {} };
  const gl = fakeGl({ atlas: new Uint8Array(1024 * 660 * 4) });
  let ctrl;
  try {
    const mod = await import(`${pathToFileURL(path.join(selectedChunkDir, 'xr-slam.js')).href}?identity=${++serial}`);
    const ready = mod.XrControllerFactory({ paused: false }, Promise.resolve({}), () => () => {}, (e) => errors.push(e));
    const attach = async () => {
      ctrl = await ready;
      ctrl.updateCameraProjectionMatrix({ cam: { pixelRectWidth: 240, pixelRectHeight: 320, nearClipPlane: 0.01, farClipPlane: 1000 } });
      const pm = ctrl.pipelineModule();
      pm.onBeforeSessionInitialize({ sessionAttributes: { fillsCameraTexture: true, controlsCamera: false } });
      pm.onStart({ computeCtx: gl, canvasWidth: 240, canvasHeight: 320 });
      pm.onSessionAttach({ videoWidth: 240, videoHeight: 320, canvasWidth: 240, canvasHeight: 320, rotation: 0,
        computeCtx: gl, framework, GLctx: gl, sessionAttributes: { fillsCameraTexture: true } });
      return { ctrl, pm };
    };
    await check({ ready, attach, configs, errors, framework, gl, listeners });
  } finally {
    if (ctrl) { ctrl.pipelineModule().onSessionDetach({}); ctrl.pipelineModule().onDetach({}); }
    for (const [key, descriptor] of saved) {
      if (descriptor) Object.defineProperty(globalThis, key, descriptor); else delete globalThis[key];
    }
    URL.createObjectURL = urls.create; URL.revokeObjectURL = urls.revoke;
  }
}

test('default parity never reads or waits for UA data and keeps SDK K', async () => {
  let reads = 0;
  await fixture(undefined, { get userAgentData() { reads++; throw new Error('must not read'); } }, async ({ attach, configs, errors }) => {
    await attach();
    assert.equal(reads, 0);
    assert.equal(configs[0].model, 'K');
    assert.equal('accuracy' in configs[0], false);
    assert.equal('intrinsics' in configs[0], false);
    assert.deepEqual(globalThis.__slamRefactChunk.session.device, { manufacturer: '', model: 'K', source: 'XR8.XrDevice.deviceEstimate' });
    assert.deepEqual(errors, []);
  });
});

test('opt-in resolves UA model before exposing controller and preserves identity through rebuild', async () => {
  let complete;
  const pending = new Promise((resolve) => { complete = resolve; });
  let calls = 0;
  await fixture({ enabled: true, fallback: { model: 'explicit' } }, { userAgentData: {
    getHighEntropyValues: (keys) => { calls++; assert.deepEqual(keys, ['model']); return pending; },
  } }, async ({ ready, attach, configs, framework, gl }) => {
    let exposed = false; ready.then(() => { exposed = true; });
    await new Promise((resolve) => setTimeout(resolve, 5));
    assert.equal(exposed, false);
    complete({ model: 'SM-F741N' });
    const { ctrl, pm } = await attach();
    assert.equal(calls, 1);
    assert.equal(configs[0].model, 'SM-F741N');
    assert.equal(configs[0].manufacturer, '');
    assert.equal('accuracy' in configs[0], false, 'identity does not enable accuracy');
    assert.equal(globalThis.__slamRefactChunk.session.device.source, 'userAgentData.model');
    globalThis.__slamRefactChunk.deviceIdentity.selected.model = 'diagnostic-mutation';
    globalThis.SLAM_REFACT_CHUNK_OPTIONS.deviceIdentity = { enabled: true, fallback: { model: 'late-option' } };
    ctrl.configure({ deviceIdentity: { enabled: true, model: 'not-a-runtime-option' } });
    pm.onVideoSizeChange({ videoWidth: 270, videoHeight: 360, orientation: 0, computeCtx: gl, framework });
    assert.equal(configs.at(-1).model, 'SM-F741N');
    assert.equal(calls, 1, 'resize does not issue a second UA request');
  });
});

test('empty UA model uses a copied explicit identity, with profile ID only as provenance', async () => {
  const option = { enabled: true, fallback: { model: 'SM-F741N', manufacturer: 'Samsung', profileId: 'unverified-profile-id' } };
  await fixture(option, { userAgentData: { getHighEntropyValues: async () => ({ model: '' }) } }, async ({ attach, configs }) => {
    option.fallback.model = 'changed-after-factory';
    await attach();
    assert.equal(configs[0].model, 'SM-F741N');
    assert.equal(configs[0].manufacturer, 'Samsung');
    assert.equal('profileId' in configs[0], false);
    assert.equal('intrinsics' in configs[0], false);
    const d = globalThis.__slamRefactChunk;
    assert.equal(d.session.device.source, 'explicit-fallback');
    assert.equal(d.deviceIdentity.highEntropyStatus, 'empty-or-invalid-model');
    assert.equal(d.deviceIdentity.selected.profileId, 'unverified-profile-id');
  });
});

for (const [status, navigatorObject] of [
  ['unavailable', {}],
  ['rejected', { userAgentData: { getHighEntropyValues: async () => { throw new Error('denied'); } } }],
]) test(`${status} UA falls back to unchanged SDK identity`, async () => {
  await fixture({ enabled: true }, navigatorObject, async ({ attach, configs }) => {
    await attach();
    assert.equal(configs[0].model, 'K');
    assert.equal(globalThis.__slamRefactChunk.deviceIdentity.highEntropyStatus, status);
    assert.equal(globalThis.__slamRefactChunk.session.device.source, 'XR8.XrDevice.deviceEstimate');
  });
});

test('timed-out UA cannot change a created or rebuilt session when it resolves late', async () => {
  let complete;
  const pending = new Promise((resolve) => { complete = resolve; });
  await fixture({ enabled: true, timeoutMs: 5, fallback: { model: 'Pixel 7' } },
    { userAgentData: { getHighEntropyValues: () => pending } }, async ({ attach, configs, framework, gl }) => {
      const { pm } = await attach();
      assert.equal(configs[0].model, 'Pixel 7');
      assert.equal(globalThis.__slamRefactChunk.deviceIdentity.highEntropyStatus, 'timed-out');
      complete({ model: 'SM-F741N' });
      await Promise.resolve();
      pm.onVideoSizeChange({ videoWidth: 270, videoHeight: 360, orientation: 0, computeCtx: gl, framework });
      assert.equal(configs.at(-1).model, 'Pixel 7');
    });
});

test('model selection alone does not change accuracy flag, and explicit K retains precedence in WASM', async () => {
  const Module = await loadEngineModule(engineDir);
  const cfg = { captureWidth: 480, captureHeight: 640, manufacturer: '' };
  const getK = (options) => {
    const glue = new Module.SlamRefactGlue({ ...cfg, ...options });
    try { return typeof glue.getCaptureIntrinsics === 'function' ? glue.getCaptureIntrinsics() : null; }
    finally { glue.delete(); }
  };
  const normal = getK({ model: 'SM-F741N' });
  if (!normal) {
    assert.notEqual(process.env.REQUIRE_P0_READBACK, '1', 'fresh WASM must expose capture readback');
    return; // legacy compatibility is also exercised by configuration-readback.test.mjs
  }
  const close = (a, b) => assert.ok(Math.abs(a - b) < 0.0001, `${a} != ${b}`);
  close(normal.fx, 506.4104);
  close(getK({ model: 'K', accuracy: { enabled: true } }).fx, 506.4104);
  close(getK({ model: 'SM-F741N', accuracy: { enabled: true } }).fx, 540);
  assert.deepEqual(getK({ model: 'SM-F741N', accuracy: { enabled: true }, intrinsics: [421, 422, 231, 312] }),
    { width: 480, height: 640, fx: 421, fy: 422, cx: 231, cy: 312 });
});

test('invalid opt-in settings are rejected before engine construction', async () => {
  for (const option of [{ enabled: true, timeoutMs: 0 }, { enabled: true, timeoutMs: Infinity },
    { enabled: true, fallback: { model: '' } }]) {
    await assert.rejects(fixture(option, {}, async ({ attach }) => { await attach(); }), /deviceIdentity/);
  }
});

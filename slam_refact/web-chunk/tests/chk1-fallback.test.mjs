// node --test slam_refact/web-chunk/tests/chk1-fallback.test.mjs
// CHK-1 fallback (pre-review K-1): the engine time is `frameTime ? frameTime * 1e9 : timeNanos` (xr-slam.js processStagedFrame). tests/chk1-engine-time.test.mjs
// plays recorded bundles, where the chunk driver always supplies a non-zero frameTime, so it cannot reach the branch "no frameTime -> the stage time".
// This file drives the chunk itself (real pipeline module, real pinned engine, fake GL / clock) with frameTime 0, undefined and non-zero and reads the first
// argument of every processFramePyramid call. It imports only the fake GL and the clock helper of the OR-1 tools (read-only).
//   SLAM_REFACT_CHUNK_DIR / SLAM_REFACT_CHUNK_ENGINE_DIR as in chunk-contract.test.mjs
import assert from 'node:assert/strict';
import fs from 'node:fs';
import path from 'node:path';
import { createRequire } from 'node:module';
import { pathToFileURL } from 'node:url';
import test from 'node:test';

const repo = path.resolve(import.meta.dirname, '../../..');
const chunkDir = process.env.SLAM_REFACT_CHUNK_DIR ?? path.join(repo, 'slam_refact/web-chunk');
const engineDir = process.env.SLAM_REFACT_CHUNK_ENGINE_DIR ?? path.join(repo, 'build/slam-refact-full-20261019/chunk/rm4d-20261020/engine-pinned');
const { fakeGl, msForNs } = await import(pathToFileURL(path.join(import.meta.dirname, '../tools/or1/players.mjs')).href);

const W = 240, H = 320;
globalThis.window = globalThis;
globalThis.addEventListener = () => {};
globalThis.removeEventListener = () => {};
globalThis._c8 = { checkAllZeroes: 0 };
globalThis.XR8 = { isPaused: () => false, drawTexForComputeTex: (t) => ({ drawFor: t }),
  XrDevice: { deviceEstimate: () => ({ manufacturer: 'Google', model: 'Pixel 7' }), isDeviceBrowserCompatible: () => true },
  XrConfig: { camera: () => ({ BACK: 'back' }), device: () => ({ MOBILE: 'mobile' }) }, XrPermissions: { permissions: () => ({}) } };
const engineUrlDir = new URL('slam-refact-engine/', pathToFileURL(`${chunkDir}/`)).href;
globalThis.fetch = async (url) => {
  const href = String(url);
  if (!href.startsWith(engineUrlDir)) throw new Error(`unexpected fetch ${href}`);
  const bytes = fs.readFileSync(path.join(engineDir, href.slice(engineUrlDir.length)));
  return { ok: true, status: 200, arrayBuffer: async () => bytes.buffer.slice(bytes.byteOffset, bytes.byteOffset + bytes.byteLength) };
};
globalThis.Blob = class { constructor(parts) { this.parts = parts; } };
URL.createObjectURL = (blob) => `data:text/javascript;base64,${Buffer.concat(blob.parts.map((p) => Buffer.from(typeof p === 'string' ? p : new Uint8Array(p)))).toString('base64')}`;
URL.revokeObjectURL = () => {};
globalThis.require = createRequire(path.join(engineDir, 'slam_refact_engine.js'));
globalThis.__dirname = engineDir;

// frames: [frameTime as given to onProcessGpu (undefined = key absent), expected engine time]. T(k) = the stage time of frame k in ns.
const T = (k) => 5_000_000_000 + k * 33_333_333;
const FRAMES = [
  { ft: 0, expect: (k) => T(k), what: 'frameTime 0 -> stage time' },
  { ft: undefined, expect: (k) => T(k), what: 'frameTime absent -> stage time' },
  { ft: 12.5, expect: () => 12_500_000_000, what: 'frameTime 12.5 s -> trunc(frameTime * 1e9)' },
  { ft: 0.000000001, expect: () => 1, what: 'frameTime 1 ns -> 1 (non-zero is used even when tiny)' },
  { ft: 0, expect: (k) => T(k), what: 'frameTime 0 again -> stage time' },
  { ft: 3.0000000009999, expect: () => 3_000_000_000, what: 'frameTime * 1e9 = 3000000000.9999 -> truncated (not rounded) like the binding' },
];

test('CHK1-F1: processFramePyramid gets frameTime * 1e9 when frameTime is non-zero and the stage time when it is 0 or absent', async () => {
  const calls = [];
  const state = { atlas: null };
  const createGlue = (Module, config) => {
    const real = new Module.SlamRefactGlue(config);
    state.atlas = new Uint8Array(real.pyramidLayout().rows * 1024 * 4);
    return new Proxy(real, { get(target, prop) {
      const v = target[prop];
      if (typeof v !== 'function') return v;
      return (...args) => { if (prop === 'processFramePyramid') calls.push(args[0]); return v.apply(target, args); };
    } });
  };
  const clock = { ms: 0 };
  globalThis.performance = { now: () => clock.ms };
  globalThis.SLAM_REFACT_CHUNK_OPTIONS = { createGlue };
  const errors = [];
  let ctrl;
  try {
    const mod = await import(`${pathToFileURL(path.join(chunkDir, 'xr-slam.js')).href}?chk1-fallback=1`);
    ctrl = await mod.XrControllerFactory({ paused: false }, Promise.resolve({}), () => () => {}, (e) => errors.push(e));
  } finally { delete globalThis.SLAM_REFACT_CHUNK_OPTIONS; }
  const pm = ctrl.pipelineModule();
  const framework = { dispatchEvent: () => {} };
  const gl = fakeGl(state);
  ctrl.configure({ enableWorldPoints: true });
  ctrl.updateCameraProjectionMatrix({ cam: { pixelRectWidth: W, pixelRectHeight: H, nearClipPlane: 0.01, farClipPlane: 1000 }, origin: { x: 0, y: 2, z: 0 }, facing: { w: 1, x: 0, y: 0, z: 0 } });
  pm.onBeforeSessionInitialize({ sessionAttributes: { fillsCameraTexture: true, controlsCamera: false } });
  pm.onStart({ computeCtx: gl, canvasWidth: W, canvasHeight: H });
  pm.onSessionAttach({ videoWidth: W, videoHeight: H, canvasWidth: W, canvasHeight: H, rotation: 0, computeCtx: gl, framework, GLctx: fakeGl({}), sessionAttributes: { fillsCameraTexture: true } });
  FRAMES.forEach((f, k) => {
    clock.ms = msForNs(T(k)).value;
    const fsr = { computeTexture: { name: k }, cameraTexture: { cam: k }, textureWidth: W, textureHeight: H, orientation: 0, repeatFrame: false, videoTime: 1 + k / 30 };
    if (f.ft !== undefined) fsr.frameTime = f.ft;
    pm.onProcessGpu({ framework, frameStartResult: fsr });
    pm.onProcessCpu({ framework, frameStartResult: fsr });
  });
  pm.onSessionDetach({}); pm.onDetach({});
  assert.deepEqual(errors, [], 'no errorCallback (a NaN / zero engine time would be rejected by the binding)');
  assert.equal(calls.length, FRAMES.length);
  FRAMES.forEach((f, k) => assert.equal(Math.trunc(calls[k]), f.expect(k), `frame ${k}: ${f.what} (got ${calls[k]})`));
});

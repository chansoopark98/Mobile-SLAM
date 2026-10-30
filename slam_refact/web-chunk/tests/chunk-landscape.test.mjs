// node --test slam_refact/web-chunk/tests/chunk-landscape.test.mjs
// PKG-A (US-023, chunk side): landscape and rotation 0 / 90 / 180 / 270 (and the negatives) captures start a session and reach the engine;
// the GL pyramid sets exactly the uniforms the engine's CPU port uses (gr8_cpu_shader.cc Stage1 / InputStage, validated against the original
// by the orient gate); the portrait rotation-0 draw is the draw it was before PKG-A. Real engine WASM + recording fake WebGL2, desktop node only.
//   SLAM_REFACT_CHUNK_ENGINE_DIR=<engine dir WITH FULL-8 (rotation_deg / rotateInput)>  (the pre-FULL-8 engine-pinned refuses a landscape glue)
//   SLAM_REFACT_CHUNK_DIR=<chunk files> (default slam_refact/web-chunk)
// What this does not show: tracking quality on a landscape camera feed (needs real pixels; see docs/design/pkg-a-landscape-report.md).
import assert from 'node:assert/strict';
import fs from 'node:fs';
import path from 'node:path';
import { createRequire } from 'node:module';
import { pathToFileURL } from 'node:url';
import test from 'node:test';

const repo = path.resolve(import.meta.dirname, '../../..');
const chunkDir = process.env.SLAM_REFACT_CHUNK_DIR ?? path.join(repo, 'slam_refact/web-chunk');
const engineDir = process.env.SLAM_REFACT_CHUNK_ENGINE_DIR ?? path.join(repo, 'build/slam-refact-or1-20261020/wasm/wasm');

// ---- the node environment of chunk-contract.test.mjs (engine bytes through file fetch) ----
const listeners = new Map();
function installGlobals() {
  globalThis.window = globalThis;
  globalThis.addEventListener = (n, cb) => { if (!listeners.has(n)) listeners.set(n, []); listeners.get(n).push(cb); };
  globalThis.removeEventListener = (n, cb) => { const l = listeners.get(n) || []; const i = l.indexOf(cb); if (i >= 0) l.splice(i, 1); };
  globalThis.dispatchEvent = (e) => { for (const cb of listeners.get(e.type) || []) cb(e); };
  globalThis._c8 = { checkAllZeroes: 0 };
  globalThis.XR8 = {
    isPaused: () => false,
    XrDevice: { deviceEstimate: () => ({ manufacturer: 'Google', model: 'Pixel 7', os: 'Android', osVersion: '13' }), isDeviceBrowserCompatible: () => true },
    XrConfig: { camera: () => ({ FRONT: 'front', BACK: 'back', ANY: 'any' }), device: () => ({ MOBILE: 'mobile', MOBILE_AND_HEADSETS: 'mobile-and-headsets', ANY: 'any' }) },
    XrPermissions: { permissions: () => ({ CAMERA: 'camera', DEVICE_MOTION: 'devicemotion', DEVICE_ORIENTATION: 'deviceorientation', DEVICE_GPS: 'geolocation', MICROPHONE: 'microphone' }) },
    drawTexForComputeTex: (t) => ({ drawFor: t }),
  };
  const engineUrlDir = new URL('slam-refact-engine/', pathToFileURL(chunkDir + '/')).href;
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
}
installGlobals();

// recording fake WebGL2: every call recorded, uniform locations carry their name, creations return objects
function fakeGl() {
  const calls = [], created = {};
  let objects = 0;
  const consts = new Map([['NO_ERROR', 0], ['TEXTURE0', 0x84c0]]);
  const constant = (name) => { if (!consts.has(name)) consts.set(name, 0x10000 + consts.size); return consts.get(name); };
  const handlers = {
    getParameter: (p) => (p === constant('VIEWPORT') ? [0, 0, 1, 1] : p === constant('ACTIVE_TEXTURE') ? constant('TEXTURE0')
      : p === constant('PACK_ALIGNMENT') ? 4 : p === constant('COLOR_WRITEMASK') ? [true, true, true, true] : p === constant('VERSION') ? 'fake' : null),
    isEnabled: () => false, isContextLost: () => false, getShaderParameter: () => true, getProgramParameter: () => true,
    checkFramebufferStatus: () => constant('FRAMEBUFFER_COMPLETE'), getError: () => 0, getExtension: () => null,
    clientWaitSync: () => constant('ALREADY_SIGNALED'), getUniformLocation: (_p, name) => ({ uniform: name }),
  };
  const gl = new Proxy({}, { get(_, prop) {
    if (typeof prop !== 'string') return undefined;
    if (/^[A-Z0-9_]+$/.test(prop)) return constant(prop);
    return (...args) => {
      calls.push([prop, ...args]);
      if (handlers[prop]) return handlers[prop](...args);
      if (prop.startsWith('create') || prop === 'fenceSync') { const obj = { id: ++objects, kind: prop, index: (created[prop] ??= []).length + 1 }; created[prop].push(obj); return obj; }
      return undefined;
    };
  } });
  return { gl, calls, created };
}

const IDENT = [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1];
const engineUrl = new URL('slam-refact-engine/', pathToFileURL(chunkDir + '/'));
let engineP;
const engine = () => (engineP ??= import(pathToFileURL(path.join(chunkDir, 'engine-loader.js')).href).then((m) => m.loadEngine(engineUrl)));
const pyramidModule = () => import(pathToFileURL(path.join(chunkDir, 'gr8-compute-pyramid.js')).href);
const layoutOf = async (w, h, rotationDeg) => {
  const { Module } = await engine();
  const glue = new Module.SlamRefactGlue({ captureWidth: w, captureHeight: h, rotationDeg, manufacturer: 'Google', model: 'Pixel 7' });
  const layout = glue.pyramidLayout();
  glue.delete();
  return layout;
};

// Uniform values of one stage(): the input stage scale, and the stage-1 `rotation` / viewport / `scale` per draw (program order of the
// constructor: input, stage1, blur, decimate, stage2, stage3).
function stageUniforms(fake, pyramid) {
  fake.calls.length = 0;
  pyramid.stage({ name: 1 });
  const [input, stage1, , decimate] = [1, 2, 3, 4].map((i) => fake.created.createProgram[i - 1]);
  const u = {};
  let current = null, viewport = null;
  const out = { input: null, stage1: [], decimate: [] };
  for (const [name, ...a] of fake.calls) {
    if (name === 'useProgram') current = a[0];
    else if (name === 'viewport') viewport = a;
    else if (name === 'uniform1i' && a[0]?.uniform) u[`${current?.id}:${a[0].uniform}`] = a[1];
    else if (name === 'uniform2f' && a[0]?.uniform) u[`${current?.id}:${a[0].uniform}`] = [a[1], a[2]];
    else if (name === 'drawElements') {
      if (current === input) out.input = { scale: u[`${input.id}:scale`], viewport };
      else if (current === stage1) out.stage1.push({ rotation: u[`${stage1.id}:rotation`], viewport, pixScale: u[`${stage1.id}:pixScale`], scale: u[`${stage1.id}:scale`] });
      else if (current === decimate) out.decimate.push({ rotation: u[`${decimate.id}:rotation`] });
    }
  }
  return out;
}

// The engine CPU port's rule (engine/src/frontend/gr8_cpu_shader.cc InputStage, Stage1, PyramidDecimate), written down from the source (same formula as the implementation; the independent check is the engine's own roiSlots[].shaderRotation below).
const cpuRule = (layout, rotation, w, h) => {
  const rot = (l) => (l.rotated ? rotation - 90 : rotation);
  const iw = layout.rotateInput ? h : w, ih = layout.rotateInput ? w : h;
  return { inputScale: [1 / iw, 1 / ih], level0Rotation: rot(layout.levels[0]),
    slotRotations: layout.roiSlots.map(rot), decimate: [0, 1, 2].map((i) => rot(layout.levels[i + 1]) - rot(layout.levels[i])) };
};

const CASES = [
  { w: 240, h: 320, rotation: 0, landscape: false }, { w: 240, h: 320, rotation: 180, landscape: false },
  { w: 320, h: 240, rotation: 90, landscape: true }, { w: 320, h: 240, rotation: 270, landscape: true },
  { w: 320, h: 240, rotation: -90, landscape: true }, { w: 320, h: 240, rotation: 180, landscape: true }, { w: 320, h: 240, rotation: 0, landscape: true },
  { w: 640, h: 480, rotation: 90, landscape: true },
  // -270 on a rotated level gives -360, which neither the GLSL nor RotateUv maps (identity); -270 = 90 so the rule stays the engine's
  { w: 240, h: 320, rotation: -180, landscape: false }, { w: 240, h: 320, rotation: -270, landscape: false }, { w: 240, h: 320, rotation: 270, landscape: false },
  { w: 320, h: 240, rotation: -270, landscape: true },
];

for (const c of CASES) {
  test(`L1 pyramid uniforms = engine CPU port rule: ${c.w}x${c.h} rotation ${c.rotation}`, async () => {
    const layout = await layoutOf(c.w, c.h, c.rotation);
    assert.equal(!!layout.rotateInput, c.landscape, 'the engine marks a landscape capture');
    const { Gr8ComputePyramid } = await pyramidModule();
    const fake = fakeGl();
    const pyramid = new Gr8ComputePyramid(fake.gl, layout, { cameraWidth: c.w, cameraHeight: c.h, rotation: c.rotation });
    const got = stageUniforms(fake, pyramid);
    const want = cpuRule(layout, c.rotation, c.w, c.h);
    assert.deepEqual(got.input.scale, want.inputScale, 'gr8-input.frag scale is 1 / the portrait-frame size');
    assert.deepEqual(got.input.viewport, [0, 0, layout.filteredWidth, layout.filteredHeight]);
    assert.equal(got.stage1[0].rotation, want.level0Rotation, 'level 0 draw rotation');
    assert.deepEqual(got.stage1.slice(1).map((d) => d.rotation), want.slotRotations, 'ROI slot clear draws use the slot rotation of the engine');
    assert.deepEqual(layout.roiSlots.map((l) => l.shaderRotation), want.slotRotations, 'the engine layout reports the same slot rotations');
    assert.deepEqual(got.decimate.map((d) => d.rotation), want.decimate);
    assert.deepEqual(got.stage1[0].pixScale, [layout.filteredWidth, layout.filteredHeight]);
    pyramid.dispose();
  });
}

test('L2 portrait rotation 0 draws what it drew before PKG-A (level 0 rotation 0 or -90 never anything else, input scale 1 / camera size)', async () => {
  const layout = await layoutOf(240, 320, 0);
  const { Gr8ComputePyramid } = await pyramidModule();
  const fake = fakeGl();
  const pyramid = new Gr8ComputePyramid(fake.gl, layout, { cameraWidth: 240, cameraHeight: 320 });  // rotation defaults to 0, as before
  const got = stageUniforms(fake, pyramid);
  assert.deepEqual(got.input.scale, [1 / 240, 1 / 320]);
  const before = (l) => (l.rotated ? -90 : 0);  // the pre-PKG-A function
  assert.equal(got.stage1[0].rotation, before(layout.levels[0]));
  assert.deepEqual(got.stage1.slice(1).map((d) => d.rotation), layout.roiSlots.map(before));
  assert.deepEqual(got.decimate.map((d) => d.rotation), [0, 1, 2].map((i) => before(layout.levels[i + 1]) - before(layout.levels[i])));
  pyramid.dispose();
});

test('L3 an unsupported rotation or a layout that disagrees with the capture size is refused by the pyramid', async () => {
  const layout = await layoutOf(240, 320, 0);
  const { Gr8ComputePyramid } = await pyramidModule();
  assert.throws(() => new Gr8ComputePyramid(fakeGl().gl, layout, { cameraWidth: 240, cameraHeight: 320, rotation: 45 }), /unsupported capture rotation 45/);
  assert.throws(() => new Gr8ComputePyramid(fakeGl().gl, layout, { cameraWidth: 320, cameraHeight: 240, rotation: 90 }), /disagree/);
});

// ---- sessions through the controller ----
const W = 240, H = 320;
async function makeController(tag, options) {
  const mod = await import(`${pathToFileURL(path.join(chunkDir, 'xr-slam.js')).href}?instance=${tag}`);
  const errors = [];
  if (options) globalThis.SLAM_REFACT_CHUNK_OPTIONS = options;
  let ctrl;
  try { ctrl = await mod.XrControllerFactory({ paused: false }, Promise.resolve({}), () => () => {}, (e) => errors.push(e)); } finally { delete globalThis.SLAM_REFACT_CHUNK_OPTIONS; }
  return { ctrl, errors };
}
async function attach(tag, { w, h, rotation }, options = {}) {
  const glues = [];
  const { ctrl, errors } = await makeController(tag, { ...options, createGlue: (Module, config) => { const g = new Module.SlamRefactGlue(config); glues.push({ config, glue: g }); return g; } });
  const pm = ctrl.pipelineModule();
  const events = [];
  const framework = { dispatchEvent: (name, detail) => events.push([name, detail]) };
  ctrl.configure({ enableWorldPoints: true });
  ctrl.updateCameraProjectionMatrix({ cam: { pixelRectWidth: W, pixelRectHeight: H, nearClipPlane: 0.01, farClipPlane: 1000 }, origin: { x: 0, y: 2, z: 0 }, facing: { w: 1, x: 0, y: 0, z: 0 } });
  pm.onBeforeSessionInitialize({ sessionAttributes: { fillsCameraTexture: true, controlsCamera: false } });
  const fake = fakeGl();
  pm.onStart({ computeCtx: fake.gl, canvasWidth: W, canvasHeight: H });
  pm.onSessionAttach({ videoWidth: w, videoHeight: h, canvasWidth: W, canvasHeight: H, rotation, computeCtx: fake.gl, framework, GLctx: fakeGl().gl, sessionAttributes: { fillsCameraTexture: true } });
  return { ctrl, pm, errors, events, framework, fake, glues, k: 0 };
}
const frame = (s, w, h, rotation) => {
  const k = s.k++;
  const fsr = { computeTexture: { name: 100 + k }, cameraTexture: { cam: 100 + k }, textureWidth: w, textureHeight: h, orientation: rotation, repeatFrame: false, videoTime: 1 + k / 30, frameTime: 1 + k / 30 };
  const r = s.pm.onProcessCpu({ framework: s.framework, frameStartResult: fsr });
  for (let j = 0; j < 3; j++) globalThis.dispatchEvent({ type: 'devicemotion', timeStamp: 1000 + k * 33 + j, interval: 0.016, acceleration: { x: 0, y: 0, z: 0 },
    accelerationIncludingGravity: { x: 0, y: 9.81, z: 0 }, rotationRate: { alpha: 0, beta: 0, gamma: 0 } });
  s.pm.onProcessGpu({ framework: s.framework, frameStartResult: fsr });
  return r;
};
const detach = (s) => { s.pm.onSessionDetach({}); s.pm.onDetach({}); };

for (const c of [{ w: H, h: W, rotation: 90 }, { w: H, h: W, rotation: 270 }, { w: H, h: W, rotation: -90 }, { w: H, h: W, rotation: 180 }, { w: H, h: W, rotation: 0 }, { w: W, h: H, rotation: 180 }]) {
  test(`L4 attach ${c.w}x${c.h} rotation ${c.rotation} starts a session with the capture and rotation, frames flow and answer`, async () => {
    const s = await attach(`l4-${c.w}-${c.rotation}`, c);
    assert.deepEqual(s.errors, []);
    assert.equal(s.glues.length, 1, 'a session is created, not rejected');
    assert.equal(s.glues[0].config.captureWidth, c.w);
    assert.equal(s.glues[0].config.captureHeight, c.h);
    assert.equal(s.glues[0].config.rotationDeg, c.rotation);
    const results = Array.from({ length: 8 }, () => frame(s, c.w, c.h, c.rotation));
    const diag = globalThis.__slamRefactChunk;
    assert.ok(results.slice(2).every(Boolean), 'onProcessCpu answers');
    assert.equal(diag.stats.sessionsRejected, 0);
    assert.equal(diag.stats.inactiveAnswers, 0);
    assert.ok(diag.stats.stagedFrames >= 8 && diag.stats.processedFrames >= 7, 'frames are staged and processed by the engine');
    assert.equal(diag.stats.processErrors + diag.stats.stageErrors, 0);
    // the session's own pyramid was built with the session rotation: its first stage-1 draw (level 0) carries the engine CPU port's rotation
    const layout = await layoutOf(c.w, c.h, c.rotation);
    const stage1 = s.fake.created.createProgram[1];
    let current = null, rot = null, first = null;
    for (const [name, ...a] of s.fake.calls) {
      if (name === 'useProgram') current = a[0];
      else if (name === 'uniform1i' && current === stage1 && a[0]?.uniform === 'rotation') rot = a[1];
      else if (name === 'drawElements' && current === stage1 && first === null) first = rot;
    }
    assert.equal(first, cpuRule(layout, c.rotation, c.w, c.h).level0Rotation, 'level 0 draw rotation of the session pyramid');
    assert.ok(results.at(-1).intrinsics.length === 16 && results.at(-1).intrinsics.every(Number.isFinite), 'the engine answers an intrinsics matrix');
    detach(s);
  });
}

test('L5 a landscape session answers a different intrinsics matrix than the portrait one (differs only; equality with the original is the AB-I of the headless run)', async () => {
  const p = await attach('l5-p', { w: W, h: H, rotation: 0 });
  const l = await attach('l5-l', { w: H, h: W, rotation: 90 });
  const rp = Array.from({ length: 6 }, () => frame(p, W, H, 0)).at(-1);
  const rl = Array.from({ length: 6 }, () => frame(l, H, W, 90)).at(-1);
  assert.notDeepEqual(rp.intrinsics, rl.intrinsics);
  detach(p); detach(l);
});

test('L6 portrait -> landscape(90) -> portrait: each orientation change rebuilds the session with its rotation; none is rejected; no errorCallback', async () => {
  const s = await attach('l6', { w: W, h: H, rotation: 0 });
  for (let i = 0; i < 6; i++) frame(s, W, H, 0);
  const args = (w, h, o) => ({ videoWidth: w, videoHeight: h, orientation: o, canvasWidth: w, canvasHeight: h, computeCtx: s.fake.gl, GLctx: s.fake.gl });
  s.pm.onDeviceOrientationChange(args(W, H, 90));
  s.pm.onVideoSizeChange(args(H, W, 90));
  const land = Array.from({ length: 8 }, () => frame(s, H, W, 90));
  assert.ok(land.slice(3).every(Boolean));
  assert.deepEqual(s.glues.map((g) => [g.config.captureWidth, g.config.captureHeight, g.config.rotationDeg]), [[W, H, 0], [W, H, 90], [H, W, 90]]);
  s.pm.onDeviceOrientationChange(args(H, W, 0));
  s.pm.onVideoSizeChange(args(W, H, 0));
  const back = Array.from({ length: 8 }, () => frame(s, W, H, 0));
  assert.ok(back.slice(3).every(Boolean));
  assert.deepEqual(s.glues.map((g) => [g.config.captureWidth, g.config.captureHeight, g.config.rotationDeg]),
    [[W, H, 0], [W, H, 90], [H, W, 90], [H, W, 0], [W, H, 0]], 'the way back passes a landscape rotation-0 glue (xr.js sends the old size first)');
  const diag = globalThis.__slamRefactChunk;
  assert.equal(diag.stats.sessionsRejected, 0);
  assert.equal(diag.stats.inactiveAnswers, 0);
  assert.deepEqual(s.errors, []);
  detach(s);
});

test('L7 a rotation that is not 0 / 90 / 180 / 270 (and the negatives) is still refused: attach throws, a change goes inactive with one warn', async () => {
  const realWarn = console.warn, warns = [];
  console.warn = (...a) => warns.push(a.join(' '));
  try {
    const { ctrl } = await makeController('l7a');
    const pm = ctrl.pipelineModule();
    const fw = { dispatchEvent: () => {} };
    assert.throws(() => pm.onSessionAttach({ videoWidth: W, videoHeight: H, canvasWidth: W, canvasHeight: H, rotation: 45, computeCtx: fakeGl().gl, framework: fw, GLctx: fakeGl().gl,
      sessionAttributes: { fillsCameraTexture: true } }), /supports rotation 0, 90, 180, 270/);
    const s = await attach('l7b', { w: W, h: H, rotation: 0 });
    for (let i = 0; i < 4; i++) frame(s, W, H, 0);
    s.pm.onDeviceOrientationChange({ videoWidth: W, videoHeight: H, orientation: 45, canvasWidth: W, canvasHeight: H, computeCtx: s.fake.gl, GLctx: s.fake.gl });
    const r = frame(s, W, H, 45);
    assert.equal(r.trackingStatus, 'LIMITED');
    assert.equal(warns.filter((w) => /supports rotation 0, 90, 180, 270/.test(w)).length, 1);
    assert.deepEqual(s.errors, []);
    detach(s);
  } finally { console.warn = realWarn; }
});

test('L8 gpuCheck compares with the CPU port at rotation 0 only: a rotated session is not checked (UNVERIFIED for rotation != 0)', async () => {
  const s0 = await attach('l8-0', { w: W, h: H, rotation: 0 }, { gpuCheck: { every: 1, frames: 3 } });
  for (let i = 0; i < 6; i++) frame(s0, W, H, 0);
  assert.ok(globalThis.__slamRefactChunk.gpuCheck.length > 0, 'rotation 0: checked');
  detach(s0);
  const s1 = await attach('l8-90', { w: H, h: W, rotation: 90 }, { gpuCheck: { every: 1, frames: 3 } });
  for (let i = 0; i < 6; i++) frame(s1, H, W, 90);
  assert.equal(globalThis.__slamRefactChunk.gpuCheck.length, 0, 'rotation 90: not checked');
  detach(s1);
});

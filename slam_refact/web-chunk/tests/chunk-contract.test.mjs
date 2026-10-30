// node --test slam_refact/web-chunk/tests/chunk-contract.test.mjs
// Contract tests of the slam_refact xr-slam chunk (spec docs/spec/07-xr-slam-chunk.md C7-1, C8-2 (a), C9-1, C9-3):
// the chunk runs in node with the real engine WASM, a fake XR8 framework and a recording fake WebGL2 context, and its
// export / pipeline / XrController / onProcessCpu key sets are compared with the deployed original xr-slam.js.
//   SLAM_REFACT_CHUNK_ENGINE_DIR=<dir with manifest.json + slam_refact_engine.{js,wasm}> (default
//   build/slam-refact-full-20261019/chunk/rm4d-20261020/engine-pinned)
//   SLAM_REFACT_CHUNK_DIR=<dir with the chunk files> (default slam_refact/web-chunk; the RM-4a mutation runs point it
//   at modified copies under build/ to check that the new tests fail on them)
import assert from 'node:assert/strict';
import fs from 'node:fs';
import path from 'node:path';
import { createRequire } from 'node:module';
import { pathToFileURL } from 'node:url';
import test from 'node:test';

const repo = path.resolve(import.meta.dirname, '../../..');
const chunkDir = process.env.SLAM_REFACT_CHUNK_DIR ?? path.join(repo, 'slam_refact/web-chunk');
// RM-4e (review G-10): the default is the pinned, write-protected copy of the engine the gate is judged on, not the build directory that a rebuild replaces.
const engineDir = process.env.SLAM_REFACT_CHUNK_ENGINE_DIR ?? path.join(repo, 'build/slam-refact-full-20261019/chunk/rm4d-20261020/engine-pinned');
const sdkChunk = fs.readFileSync(path.join(repo, 'assets/references/xr_reference/analysis-runtime/standalone/xr-slam.js'), 'utf8');

// ---- original key sets, read from the deployed chunk source ----
function topLevelKeys(objectBody) {
  const keys = [];
  let depth = 0, start = 0;
  for (let i = 0; i <= objectBody.length; i++) {
    const c = objectBody[i];
    if (c === '"' || c === "'" || c === '`') { i = objectBody.indexOf(c, i + 1); continue; }
    if (c === '(' || c === '{' || c === '[') depth++;
    else if (c === ')' || c === '}' || c === ']') depth--;
    else if ((c === ',' && depth === 0) || i === objectBody.length) {
      const seg = objectBody.slice(start, i).trim();
      if (seg) keys.push(seg.split(':')[0].trim());
      start = i + 1;
    }
  }
  return keys;
}
function objectAfter(src, marker) {
  const at = src.indexOf(marker);
  assert.ok(at >= 0, `marker ${marker} not found in the original chunk`);
  let i = src.indexOf('{', at), depth = 0;
  for (let j = i; j < src.length; j++) {
    const c = src[j];
    if (c === '"' || c === "'" || c === '`') { j = src.indexOf(c, j + 1); continue; }
    if (c === '{') depth++;
    else if (c === '}' && --depth === 0) return src.slice(i + 1, j);
  }
  throw new Error('unbalanced');
}
const ORIGINAL = {
  exports: [...sdkChunk.matchAll(/export\{([^}]*)\}/g)].at(-1)[1].split(',').map((s) => s.split(' as ')[1].trim()).sort(),
  pipeline: topLevelKeys(objectAfter(sdkChunk, 'WA=()=>({')).sort(),
  controller: topLevelKeys(objectAfter(sdkChunk, 'return G.then((()=>({')).sort(),
  processCpu: topLevelKeys(objectAfter(sdkChunk, 'return{rotation:G,')).sort(),
};

// ---- fake WebGL2 context: records calls, returns plausible values ----
// opts (RM-4a): params {CONSTANT_NAME: value} for getParameter, enabled [CONSTANT_NAME] for isEnabled,
// failCompileAt / failLinkAt (1-based creation index of the shader / program), failFramebufferAt (1-based
// checkFramebufferStatus call), glError (value returned by getError), fillReadback (RM-4d). `created` lists the objects of every create*.
// RM-4a2: opts.state (see statefulGlState) makes the binding / enable / viewport / colorMask / pixelStorei calls change a real
// state that getParameter / isEnabled read back and records the state at every draw / read / upload (`snapshots`);
// lose() / restore() switch a lost-context mode (isContextLost true, getParameter null, getError CONTEXT_LOST_WEBGL 0x9242).
function fakeGl(opts = {}) {
  const consts = new Map([['NO_ERROR', 0], ['TEXTURE0', 0x84c0]]);
  const calls = [];
  const created = {};
  const snapshots = [];
  let objects = 0, fbChecks = 0, lost = false;
  const constant = (name) => { if (!consts.has(name)) consts.set(name, 0x10000 + consts.size); return consts.get(name); };
  const st = opts.state ? statefulGlState(constant, opts.state) : null;
  const handlers = {
    getParameter: (p) => {
      if (lost) return null;
      if (st) return st.get(p);
      for (const [name, value] of Object.entries(opts.params ?? {})) if (p === constant(name)) return value;
      return p === constant('VIEWPORT') ? [0, 0, 1, 1] : p === constant('ACTIVE_TEXTURE') ? constant('TEXTURE0')
        : p === constant('PACK_ALIGNMENT') ? 4 : p === constant('COLOR_WRITEMASK') ? [true, true, true, true]
          : p === constant('VERSION') ? 'fake' : null;
    },
    isEnabled: (c) => (st ? st.isEnabled(c) : (opts.enabled ?? []).some((n) => constant(n) === c)),
    isContextLost: () => lost,
    getShaderParameter: (sh) => !(opts.failCompileAt && sh.index === opts.failCompileAt),
    getProgramParameter: (prog) => !(opts.failLinkAt && prog.index === opts.failLinkAt),
    checkFramebufferStatus: () => (opts.failFramebufferAt && ++fbChecks === opts.failFramebufferAt ? 0 : constant('FRAMEBUFFER_COMPLETE')),
    getError: () => (lost ? 0x9242 : opts.glError ?? 0),
    getExtension: () => null,
    clientWaitSync: () => constant('ALREADY_SIGNALED'),
    getUniformLocation: () => ({}),
    // RM-4d: opts.fillReadback = a byte value the readback buffer is filled with (default: untouched, i.e. zeros), so a test can tell
    // pyramid rows from rows the chunk added.
    getBufferSubData: (_target, _offset, dst) => { if (opts.fillReadback !== undefined) dst.fill(opts.fillReadback); },
  };
  const gl = new Proxy({}, {
    get(_, prop) {
      if (typeof prop !== 'string') return undefined;
      if (/^[A-Z0-9_]+$/.test(prop)) return constant(prop);
      return (...args) => {
        calls.push([prop, ...args]);
        if (st) {
          if (['drawElements', 'readPixels', 'texImage2D'].includes(prop)) snapshots.push({ call: prop, index: calls.length - 1, state: st.snapshot() });
          st.apply(prop, args);
        }
        if (handlers[prop]) return handlers[prop](...args);
        if (prop.startsWith('create') || prop === 'fenceSync') {
          const obj = { id: ++objects, kind: prop, index: (created[prop] ??= []).length + 1 };
          created[prop].push(obj);
          return obj;
        }
        return undefined;
      };
    },
  });
  return { gl, calls, constant, created, snapshots, state: st, lose: () => { lost = true; }, restore: () => { lost = false; } };
}

// State of the fake: what Gr8ComputePyramid saves, neutralises and restores (init: see the glstate test).
function statefulGlState(constant, init) {
  const CAPS = ['BLEND', 'SCISSOR_TEST', 'DEPTH_TEST', 'STENCIL_TEST', 'CULL_FACE', 'RASTERIZER_DISCARD'];
  const capName = (c) => CAPS.find((n) => constant(n) === c);
  const s = {
    activeTexture: init.activeTexture, tex: { ...init.tex }, samplers: { ...init.samplers },
    drawFb: init.drawFb, readFb: init.readFb, program: init.program, vao: init.vao, arrayBuffer: init.arrayBuffer,
    packBuffer: init.packBuffer, unpackBuffer: init.unpackBuffer, packAlignment: init.packAlignment,
    colorMask: [...init.colorMask], viewport: [...init.viewport], caps: { ...init.caps },
  };
  const unit = () => s.activeTexture;
  const snapshot = () => ({ ...s, tex: { ...s.tex }, samplers: { ...s.samplers }, colorMask: [...s.colorMask], viewport: [...s.viewport], caps: { ...s.caps } });
  const get = (p) => {
    const table = {
      ACTIVE_TEXTURE: () => s.activeTexture, TEXTURE_BINDING_2D: () => s.tex[unit()] ?? null, SAMPLER_BINDING: () => s.samplers[unit()] ?? null,
      DRAW_FRAMEBUFFER_BINDING: () => s.drawFb, READ_FRAMEBUFFER_BINDING: () => s.readFb, CURRENT_PROGRAM: () => s.program,
      VERTEX_ARRAY_BINDING: () => s.vao, ARRAY_BUFFER_BINDING: () => s.arrayBuffer, PIXEL_PACK_BUFFER_BINDING: () => s.packBuffer,
      PIXEL_UNPACK_BUFFER_BINDING: () => s.unpackBuffer, PACK_ALIGNMENT: () => s.packAlignment, COLOR_WRITEMASK: () => [...s.colorMask],
      VIEWPORT: () => [...s.viewport], VERSION: () => 'fake',
    };
    for (const [name, fn] of Object.entries(table)) if (p === constant(name)) return fn();
    return null;
  };
  const apply = (prop, a) => {
    switch (prop) {
      case 'activeTexture': s.activeTexture = a[0]; break;
      case 'bindTexture': if (a[0] === constant('TEXTURE_2D')) s.tex[unit()] = a[1]; break;
      case 'bindSampler': s.samplers[0x84c0 + a[0]] = a[1]; break;
      case 'bindFramebuffer':
        if (a[0] === constant('FRAMEBUFFER') || a[0] === constant('DRAW_FRAMEBUFFER')) s.drawFb = a[1];
        if (a[0] === constant('FRAMEBUFFER') || a[0] === constant('READ_FRAMEBUFFER')) s.readFb = a[1];
        break;
      case 'bindBuffer':
        if (a[0] === constant('ARRAY_BUFFER')) s.arrayBuffer = a[1];
        if (a[0] === constant('PIXEL_PACK_BUFFER')) s.packBuffer = a[1];
        if (a[0] === constant('PIXEL_UNPACK_BUFFER')) s.unpackBuffer = a[1];
        break;
      case 'bindVertexArray': s.vao = a[0]; break;
      case 'useProgram': s.program = a[0]; break;
      case 'viewport': s.viewport = [a[0], a[1], a[2], a[3]]; break;
      case 'pixelStorei': if (a[0] === constant('PACK_ALIGNMENT')) s.packAlignment = a[1]; break;
      case 'colorMask': s.colorMask = [a[0], a[1], a[2], a[3]]; break;
      case 'enable': if (capName(a[0])) s.caps[capName(a[0])] = true; break;
      case 'disable': if (capName(a[0])) s.caps[capName(a[0])] = false; break;
      default: break;
    }
  };
  return { get, apply, snapshot, isEnabled: (c) => !!s.caps[capName(c)] };
}

// ---- fake browser globals + XR8 framework ----
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
  // Engine bytes through fetch (file URLs) and a data: module for the blob import (node has no blob: imports).
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

const W = 240, H = 320;
const quiet = () => () => {};

async function makeController(tag, chunkOptions) {
  const mod = await import(`${pathToFileURL(path.join(chunkDir, 'xr-slam.js')).href}?instance=${tag}`);
  const errors = [];
  if (chunkOptions) globalThis.SLAM_REFACT_CHUNK_OPTIONS = chunkOptions;  // read synchronously by the factory
  let ctrl;
  try {
    ctrl = await mod.XrControllerFactory({ paused: false }, Promise.resolve({}), quiet, (e) => errors.push(e));
  } finally {
    delete globalThis.SLAM_REFACT_CHUNK_OPTIONS;
  }
  return { mod, ctrl, errors };
}

function motion(t) {
  return { type: 'devicemotion', acceleration: null, accelerationIncludingGravity: { x: 0, y: 9.81, z: 0 },
    rotationRate: { alpha: 0, beta: 0, gamma: 0 }, timeStamp: t, interval: 16.666 };
}

test('export names equal the original chunk exports (C7-1)', async () => {
  const mod = await import(pathToFileURL(path.join(chunkDir, 'xr-slam.js')).href);
  assert.deepEqual(Object.keys(mod).sort(), ORIGINAL.exports);
  assert.deepEqual(ORIGINAL.exports, ['ResourceUrls', 'XrControllerFactory']);
  assert.throws(() => mod.ResourceUrls.getBaseUrl(), /Base URL has not been set yet/);
  assert.throws(() => mod.ResourceUrls.setBaseUrl(''), /Base URL cannot be empty/);
  mod.ResourceUrls.setBaseUrl('http://x/engine/');
  assert.equal(mod.ResourceUrls.resolveMediaWorker(), 'http://x/engine/resources/media-worker.js');
});

test('fake framework run: keys, frame lag, events, configure errors, PBO order (C7-1, C8-2a, C9-1, C9-3)', async () => {
  const { ctrl, errors } = await makeController('a');
  const pm = ctrl.pipelineModule();
  assert.deepEqual(Object.keys(pm).sort(), ORIGINAL.pipeline, 'pipeline module keys');
  const ctrlKeys = Object.keys(ctrl).sort();
  const missing = ORIGINAL.controller.filter((k) => !ctrlKeys.includes(k));
  const extra = ctrlKeys.filter((k) => !ORIGINAL.controller.includes(k));
  // RM-4a: the three data recorder functions exist as stubs (spec 07 §3), so the key set equals the original.
  assert.deepEqual(missing, []);
  assert.deepEqual(extra, []);
  assert.equal(pm.name, 'reality');

  const events = [];
  const framework = { dispatchEvent: (name, detail) => events.push([name, JSON.parse(JSON.stringify(detail ?? null))]) };
  // Front camera with world tracking: the original error object (spec 06 §5).
  assert.throws(() => pm.onCameraStatusChange({ status: 'requesting', cameraConfig: { direction: 'front' } }),
    (e) => e.err === 'slam-front-camera-unsupported' && e.type === 'configuration' && e.source === 'reality'
      && e.message === '[XR] World tracking is only supported on the back camera');
  // After onCameraStatusChange the session counts as started: scale / world tracking changes throw.
  assert.throws(() => ctrl.configure({ scale: 'absolute' }), { message: '[XR] Scale can only be changed before calling XR8.run()' });
  assert.throws(() => ctrl.configure({ disableWorldTracking: true }), { message: '[XR] World tracking can only be changed before calling XR8.run()' });
  assert.throws(() => ctrl.configure({ imageTargetData: 'x' }), { message: '[XR] Expected array for arg: imageTargetData' });
  ctrl.configure({ enableWorldPoints: true, enableLighting: true });
  ctrl.updateCameraProjectionMatrix({ cam: { pixelRectWidth: W, pixelRectHeight: H, nearClipPlane: 0.01, farClipPlane: 1000 },
    origin: { x: 0, y: 2, z: 0 }, facing: { w: 1, x: 0, y: 0, z: 0 } });
  pm.onBeforeSessionInitialize({ sessionAttributes: { fillsCameraTexture: true, controlsCamera: false } });
  assert.deepEqual(pm.requiredPermissions(), ['camera', 'deviceorientation', 'devicemotion']);
  const { gl, calls, constant } = fakeGl();
  pm.onStart({ computeCtx: gl, canvasWidth: W, canvasHeight: H });
  pm.onSessionAttach({ videoWidth: W, videoHeight: H, canvasWidth: W, canvasHeight: H, rotation: 0, computeCtx: gl, framework,
    GLctx: fakeGl().gl, sessionAttributes: { fillsCameraTexture: true } });
  assert.ok(events.some(([n, d]) => n === 'cameraconfigured' && d.recenter === false), 'cameraconfigured at attach');
  const initCalls = calls.length;

  const frame = (k) => ({ computeTexture: { name: 100 + k }, textureWidth: W, textureHeight: H, orientation: 0, repeatFrame: false,
    videoTime: 1 + k / 30, frameTime: 1 + k / 30 });
  const results = [];
  const diag = globalThis.__slamRefactChunk;
  let pushedBefore = 0;
  for (let k = 0; k < 12; k++) {
    for (let j = 0; j < 3; j++) globalThis.dispatchEvent(motion(1000 + k * 33 + j));
    pm.onProcessGpu({ framework, frameStartResult: frame(k) });
    globalThis.dispatchEvent(motion(1000 + k * 33 + 20));  // arrives after the stage: belongs to the next frame
    results.push(pm.onProcessCpu({ framework, frameStartResult: frame(k) }));
    // IMU staged with frame k = 3 events (+ the late one of frame k-1).
    const pushed = diag.stats.motionEventsPushed - pushedBefore;
    assert.equal(pushed, k === 0 ? 3 : 4, `frame ${k} IMU events`);
    pushedBefore = diag.stats.motionEventsPushed;
  }
  assert.equal(results[0], null, 'no response before the pipeline holds two frames (original returns null)');
  for (let k = 1; k < results.length; k++) {
    const r = results[k];
    assert.ok(r, `response at frame ${k}`);
    assert.deepEqual(Object.keys(r).sort(), ORIGINAL.processCpu, 'onProcessCpu keys');
    assert.equal(r.intrinsics.length, 16);
    assert.equal(r.lighting.temperature, 6500);
    // Startup recenter: the 5th processed frame rebuilds the core, the next call repeats the previous response.
    const expectedSource = k === 6 ? 4 : k - 1;
    assert.deepEqual(r.realityTexture, { drawFor: { name: 100 + expectedSource } }, `realityTexture of frame ${k}`);
  }
  assert.deepEqual(diag.stats.responseLag, { 1: 10 });
  // Repeat frame: same response, no new engine call.
  const processed = diag.stats.processedFrames;
  const rep = pm.onProcessCpu({ framework, frameStartResult: { ...frame(11), repeatFrame: true } });
  assert.equal(diag.stats.processedFrames, processed);
  assert.deepEqual(rep.position, results.at(-1).position);

  // trackingstatus: one event per (status, reason) change of the responses, in order.
  const changes = [];
  let last = ['UNSPECIFIED', 'UNSPECIFIED'];
  for (const r of results.filter(Boolean)) {
    if (r.trackingStatus !== last[0] || r.trackingReason !== last[1]) { last = [r.trackingStatus, r.trackingReason]; changes.push({ status: last[0], reason: last[1] }); }
  }
  assert.deepEqual(events.filter(([n]) => n === 'trackingstatus').map(([, d]) => d), changes);
  assert.ok(changes.length >= 1);

  // PBO / fence order (spec 07 §2.2 table): init = bind + bufferData(DYNAMIC_READ); stage = bind + readPixels(..., 0) +
  // fenceSync; read = clientWaitSync(sync, 0, 0) + deleteSync + bind + getBufferSubData.
  const PPB = constant('PIXEL_PACK_BUFFER');
  const pbo = calls.find(([n, t, b]) => n === 'bufferData' && t === PPB && b instanceof Uint8Array) && calls.find(([n, t, b]) => n === 'bindBuffer' && t === PPB && b)[2];
  const ops = calls.filter(([n, t, b]) => (n === 'bindBuffer' && t === PPB && b === pbo) || (n === 'bufferData' && t === PPB) || n === 'fenceSync'
    || n === 'clientWaitSync' || n === 'deleteSync' || n === 'getBufferSubData' || (n === 'readPixels' && calls.length && b !== undefined))
    .map(([n, ...a]) => (n === 'readPixels' ? `readPixels:${a[6] === 0 ? 'pbo' : 'cpu'}` : n === 'clientWaitSync' ? `clientWaitSync:${a[1]},${a[2]}`
      : n === 'bufferData' ? `bufferData:${a[2] === constant('DYNAMIC_READ') ? 'DYNAMIC_READ' : a[2]}` : n));
  const stage = ['bindBuffer', 'readPixels:pbo', 'fenceSync'], read = ['clientWaitSync:0,0', 'deleteSync', 'bindBuffer', 'getBufferSubData'];
  const expected = ['bindBuffer', 'bufferData:DYNAMIC_READ'];
  for (let k = 0; k < 12; k++) expected.push(...stage, ...read);
  assert.deepEqual(ops, expected);
  assert.ok(calls.slice(initCalls).every(([n]) => n !== 'finish'), 'no glFinish');

  // hitTest: wired to the engine (empty arrays without map points; surfaces are empty in the original web runtime).
  assert.deepEqual(ctrl.hitTest(0.5, 0.5, ['ESTIMATED_SURFACE', 'DETECTED_SURFACE']), []);
  assert.ok(Array.isArray(ctrl.hitTest(0.5, 0.5)));
  assert.deepEqual(ctrl.getIntrinsic(), results.at(-1).intrinsics);

  // recenter: cameraconfigured with recenter = true, then the rebuilt engine repeats the last response once.
  ctrl.recenter();
  assert.ok(events.some(([n, d]) => n === 'cameraconfigured' && d.recenter === true));
  pm.onSessionDetach({});
  pm.onDetach({});
  assert.equal(globalThis._c8.isAllZeroes, null);
  assert.deepEqual(errors, []);
});

// PKG-A (US-023): R16 is lifted. Was: 'rotation / landscape capture is rejected explicitly (R16)' (rotation 90 attach threw /portrait capture only/).
// Now only a rotation the engine does not map to a device orientation is rejected; that a landscape / rotation 90 capture attaches is
// chunk-landscape.test.mjs L4 (it needs an engine with FULL-8; this file's default engine-pinned is the pre-FULL-8 one).
test('a rotation the engine does not take is rejected explicitly at attach (R16 lifted for 0 / 90 / 180 / 270, PKG-A)', async () => {
  const { ctrl } = await makeController('b');
  const pm = ctrl.pipelineModule();
  const framework = { dispatchEvent: () => {} };
  for (const rotation of [45, NaN, 360]) {
    assert.throws(() => pm.onSessionAttach({ videoWidth: H, videoHeight: W, canvasWidth: W, canvasHeight: H, rotation,
      computeCtx: fakeGl().gl, framework, GLctx: fakeGl().gl, sessionAttributes: { fillsCameraTexture: true } }), /supports rotation 0, 90, 180, 270/);
  }
});

test('engine bytes that do not match the manifest are not executed', async () => {
  const { loadEngine } = await import(pathToFileURL(path.join(chunkDir, 'engine-loader.js')).href);
  const realFetch = globalThis.fetch;
  globalThis.fetch = async (url) => {
    const r = await realFetch(url);
    if (!String(url).endsWith('manifest.json')) return r;
    const m = JSON.parse(new TextDecoder().decode(await r.arrayBuffer()));
    m.artifacts.wasm.sha256 = '0'.repeat(64);
    const b = Buffer.from(JSON.stringify(m));
    return { ok: true, status: 200, arrayBuffer: async () => b.buffer.slice(b.byteOffset, b.byteOffset + b.byteLength) };
  };
  try {
    await assert.rejects(loadEngine(new URL('slam-refact-engine/', pathToFileURL(chunkDir + '/'))),
      (e) => /do not match/.test(e.message) && e.loaded.matchesManifest === false);
  } finally {
    globalThis.fetch = realFetch;
  }
});

test('image events: found / updated / lost diff of the original', async () => {
  const { dispatchImageEvents } = await import(pathToFileURL(path.join(chunkDir, 'image-events.js')).href);
  const map = {}, out = [];
  const d = (n, x) => out.push([n, x.name, x.position?.x]);
  dispatchImageEvents(map, [{ name: 'a', position: { x: 1 } }], d);
  dispatchImageEvents(map, [{ name: 'a', position: { x: 1 } }, { name: 'b', position: { x: 0 } }], d);
  dispatchImageEvents(map, [{ name: 'a', position: { x: 2 } }], d);
  dispatchImageEvents(map, [], d);
  assert.deepEqual(out, [['imagefound', 'a', 1], ['imagefound', 'b', 0], ['imageupdated', 'a', 2], ['imagelost', 'b', 0], ['imagelost', 'a', 2]]);
});

test('sensor queue: original 5e9 ns clear rule and staging', async () => {
  const { createSensors } = await import(pathToFileURL(path.join(chunkDir, 'sensors.js')).href);
  const realPerf = globalThis.performance;
  let now = 0;
  globalThis.window.performance = { now: () => now };
  try {
    const s = createSensors(() => false);
    s.updateDeviceMotion(motion(0)); now = 4000; s.updateDeviceMotion(motion(1));
    assert.equal(s.queueLength(), 2);
    now = 5001; s.updateDeviceMotion(motion(2));  // 5.001e9 ns after the first entry: queue cleared first
    assert.equal(s.queueLength(), 1);
    const staged = s.takeForStage();
    assert.equal(staged.length, 1);
    assert.equal(staged[0].timestampNanos, 1e3 * 5001 * 1e3);
    assert.equal(s.queueLength(), 0);
    assert.equal(createSensors(() => true).takeForStage(), null);
  } finally {
    globalThis.window.performance = realPerf;
  }
});

// =====================================================================================================
// RM-4a (gates G2-G6 of build/slam-refact-full-20261019/preregistration/rm4a-chunk-gate.v1.md)
// =====================================================================================================

// Real engine glue behind a recording Proxy; `hooks` can replace single methods or the constructor.
function glueRecorder(hooks = {}) {
  const glues = [];
  const createGlue = (Module, config) => {
    if (hooks.beforeCreate) hooks.beforeCreate(glues.length);
    const real = new Module.SlamRefactGlue(config);
    const rec = { config, real, calls: [], deleted: 0, idx: glues.length };
    glues.push(rec);
    rec.proxy = new Proxy(real, {
      get(target, prop) {
        const value = target[prop];
        if (typeof value !== 'function') return value;
        return (...args) => {
          if (prop === 'delete') rec.deleted++; else rec.calls.push([prop, ...args]);
          if (hooks.override?.[prop]) return hooks.override[prop](rec, ...args);
          return value.apply(target, args);
        };
      },
    });
    return rec.proxy;
  };
  return { glues, createGlue };
}

// Attached session on a fake framework with the recording glue (portrait W x H).
async function attachedSession(tag, { hooks, chunkOptions = {}, glOpts } = {}) {
  const rec = glueRecorder(hooks);
  const { ctrl, errors } = await makeController(tag, { createGlue: rec.createGlue, ...chunkOptions });
  const pm = ctrl.pipelineModule();
  const events = [];
  const framework = { dispatchEvent: (name, detail) => events.push([name, JSON.parse(JSON.stringify(detail ?? null))]) };
  ctrl.configure({ enableWorldPoints: true });
  ctrl.updateCameraProjectionMatrix({ cam: { pixelRectWidth: W, pixelRectHeight: H, nearClipPlane: 0.01, farClipPlane: 1000 },
    origin: { x: 0, y: 2, z: 0 }, facing: { w: 1, x: 0, y: 0, z: 0 } });
  pm.onBeforeSessionInitialize({ sessionAttributes: { fillsCameraTexture: true, controlsCamera: false } });
  const fake = fakeGl(glOpts);
  pm.onStart({ computeCtx: fake.gl, canvasWidth: W, canvasHeight: H });
  pm.onSessionAttach({ videoWidth: W, videoHeight: H, canvasWidth: W, canvasHeight: H, rotation: 0, computeCtx: fake.gl, framework,
    GLctx: fakeGl().gl, sessionAttributes: { fillsCameraTexture: true } });
  return { ctrl, pm, errors, events, framework, fake, glues: rec.glues, k: 0, updates: 0, exceptions: [] };
}

// One xr.js camera frame: pre-render (size / orientation callbacks first, then onProcessCpu, then the rest of the
// try block = onUpdate), post-render (onProcessGpu). An exception in the pre-render try block skips the update, as
// in xr.js (`try{X(),Z(),_(),j(),T()}catch(A){J(A)}`), and is recorded.
function xrFrame(s, { w = W, h = H, before } = {}) {
  const k = s.k++;
  const fsr = { computeTexture: { name: 100 + k }, cameraTexture: { cam: 100 + k }, textureWidth: w, textureHeight: h, orientation: 0,
    repeatFrame: false, videoTime: 1 + k / 30, frameTime: 1 + k / 30 };
  let result;
  try {
    if (before) before();
    result = s.pm.onProcessCpu({ framework: s.framework, frameStartResult: fsr });
    s.updates++;
  } catch (error) {
    s.exceptions.push(error);
  }
  try {
    for (let j = 0; j < 3; j++) globalThis.dispatchEvent(motion(1000 + k * 33 + j));
    s.pm.onProcessGpu({ framework: s.framework, frameStartResult: fsr });
  } catch (error) {
    s.exceptions.push(error);
  }
  return result;
}
// PKG-A (US-023): landscape and rotation 90 / 180 / 270 are supported now (R16 lifted); the inactive-capture episodes of the tests below are
// driven by a rotation the engine does not take (a device orientation the original does not map either).
const UNSUPPORTED_ROTATION = 45;
const sizeChangeArgs = (s, w, h, orientation = 0) => ({ videoWidth: w, videoHeight: h, orientation, canvasWidth: w, canvasHeight: h,
  computeCtx: s.fake.gl, GLctx: s.fake.gl });
const runFrames = (s, n, opts) => Array.from({ length: n }, () => xrFrame(s, opts));
const detach = (s) => { s.pm.onSessionDetach({}); s.pm.onDetach({}); };

test('M6: video size change rebuilds the session without throwing; pre-render keeps running (G2.1, G2.2)', async () => {
  const s = await attachedSession('m6-size');
  const first = runFrames(s, 9);
  assert.ok(first.at(-1), 'responses before the change');
  const last = first.at(-1).position;
  assert.equal(s.glues.length, 1);
  const W2 = 270, H2 = 360;
  const results = [];
  results.push(xrFrame(s, { w: W2, h: H2, before: () => s.pm.onVideoSizeChange(sizeChangeArgs(s, W2, H2)) }));
  results.push(...runFrames(s, 7, { w: W2, h: H2 }));
  assert.deepEqual(s.exceptions, [], 'no callback throws');
  assert.equal(s.updates, 17, 'the pre-render try block ran to its end on every frame');
  assert.deepEqual(s.errors, []);
  assert.equal(s.glues.length, 2, 'new glue = engineInit again');
  assert.equal(s.glues[0].deleted, 1);
  assert.equal(s.glues[1].deleted, 0);
  assert.equal(s.glues[1].config.captureWidth, W2);
  assert.equal(s.glues[1].config.captureHeight, H2);
  const diag = globalThis.__slamRefactChunk;
  assert.equal(diag.session.videoWidth, W2);
  assert.equal(diag.stats.sessionsCreated, 2);
  assert.equal(diag.stats.sessionsDestroyed, 1);
  // RM-4a2 E1 (gate rm4a2 G1): while the new pipeline fills (original: window._c8.reality survives engineInit) the previous
  // session's last response is repeated; null would only be right for a session that never had a response.
  assert.ok(results[0] && results[1], 'the last response of the old session is repeated while the new pipeline fills');
  assert.deepEqual(results[0], first.at(-1), 'repeat of the last response (pose, intrinsics, status, texture)');
  assert.deepEqual(results[1], first.at(-1));
  assert.ok(results.slice(2).every(Boolean), 'responses resume once the new pipeline holds two frames');
  // resetOriginToLastPose (tracking-controller.cc:413-436): one origin update with the last pose, recenter point untouched.
  const resets = s.glues[1].calls.filter(([n, a]) => n === 'updateCameraProjectionMatrix' && a.updateRecenterPoint === false && a.cam === undefined);
  assert.equal(resets.length, 1, 'exactly one origin reset (the other two updates of createSession carry the recenter point / cam)');
  const arg = resets[0][1];
  assert.equal(arg.updateRecenterPoint, false);
  assert.deepEqual([arg.origin.x, arg.origin.z], [last.x, last.z]);
  assert.equal(arg.origin.y, last.y > 1e-3 ? last.y : 2);
  assert.equal(globalThis.__slamRefactChunk.stats.originResets, 1);
  detach(s);
});

test('M6: orientation change = engineInit again without the origin reset; landscape / rotated capture deactivates (G2.2, G2.3)', async () => {
  const s = await attachedSession('m6-orient');
  runFrames(s, 9);
  const W2 = 270, H2 = 360;
  xrFrame(s, { w: W2, h: H2, before: () => s.pm.onDeviceOrientationChange(sizeChangeArgs(s, W2, H2)) });
  runFrames(s, 6, { w: W2, h: H2 });
  assert.equal(s.glues.length, 2);
  assert.equal(s.glues[1].calls.filter(([n, a]) => n === 'updateCameraProjectionMatrix' && a.updateRecenterPoint === false && a.cam === undefined).length, 0,
    'the original configureXr after engineInit overrides the reset origin');
  assert.deepEqual(s.errors, []);

  // The device is rotated to a rotation the engine does not take (PKG-A: 90 is supported now, R16 lifted; 45 stands for the unsupported case).
  // RM-4a2 E2 (gate rm4a2 G1, M3): errorCallback ends the session in apps with the XRExtras RuntimeError module, so the
  // expected inactivation is a single console.warn and a LIMITED response with the last pose and the live camera texture.
  const warns = [];
  const realWarn = console.warn;
  console.warn = (...a) => warns.push(a.join(' '));
  let r90, inactive;
  try {
    r90 = xrFrame(s, { w: H2, h: W2, before: () => s.pm.onDeviceOrientationChange(sizeChangeArgs(s, H2, W2, UNSUPPORTED_ROTATION)) });
    runFrames(s, 4, { w: H2, h: W2 });
    inactive = runFrames(s, 3, { w: H2, h: W2 });
  } finally {
    console.warn = realWarn;
  }
  assert.ok(r90, 'a response exists while inactive');
  assert.equal(r90.trackingStatus, 'LIMITED');
  assert.deepEqual(s.exceptions, [], 'no callback throws while inactive');
  assert.equal(s.glues.length, 2, 'no new glue for an unsupported capture');
  assert.equal(s.glues[1].deleted, 1);
  assert.deepEqual(s.errors, [], 'errorCallback is not used for the expected inactivation');
  assert.equal(warns.filter((w) => /supports rotation 0, 90, 180, 270/.test(w)).length, 1);
  assert.equal(globalThis.__slamRefactChunk.stats.sessionsRejected, 1);
  assert.ok(inactive.every((r) => r && r.trackingStatus === 'LIMITED'), 'onProcessCpu answers LIMITED while inactive');
  assert.equal(s.updates, s.k, 'every pre-render frame completed');

  // Back to portrait: the session is rebuilt and responses resume.
  xrFrame(s, { w: W2, h: H2, before: () => s.pm.onDeviceOrientationChange(sizeChangeArgs(s, W2, H2, 0)) });
  const back = runFrames(s, 6, { w: W2, h: H2 });
  assert.equal(s.glues.length, 3);
  assert.ok(back.slice(2).every(Boolean));
  assert.deepEqual(s.exceptions, []);
  detach(s);
});

test('M6: a frame with a stale texture size is dropped without throwing (G2.4)', async () => {
  const s = await attachedSession('m6-stale');
  runFrames(s, 3);
  const diag = globalThis.__slamRefactChunk;
  const staged = diag.stats.stagedFrames;
  xrFrame(s, { w: 270, h: 360 });  // camera texture changed, callback not delivered yet
  assert.deepEqual(s.exceptions, []);
  assert.equal(diag.stats.sizeMismatchFrames, 1);
  assert.equal(diag.stats.stagedFrames, staged, 'the mismatching frame is not staged');
  assert.deepEqual(s.errors, []);
  // RM-4a2 E4 (rm4a v1 G2.4 said "null"; the real and original-like behaviour repeats the previous response).
  const again = runFrames(s, 1)[0];
  assert.ok(again, 'the previous response is repeated');
  detach(s);
});

test('M6: glue construction failure during a rebuild is reported, not thrown (G2.5)', async () => {
  const s = await attachedSession('m6-ctor', { hooks: { beforeCreate: (n) => { if (n === 1) throw new Error('engine init failed'); } } });
  runFrames(s, 4);
  xrFrame(s, { w: 270, h: 360, before: () => s.pm.onVideoSizeChange(sizeChangeArgs(s, 270, 360)) });
  const after = runFrames(s, 4, { w: 270, h: 360 });
  assert.deepEqual(s.exceptions, []);
  assert.equal(s.errors.length, 1);
  assert.equal(s.errors[0].message, 'engine init failed');
  // RM-4a2 E3: no session (construction failed): same LIMITED answer as an inactive capture.
  assert.ok(after.every((r) => r && r.trackingStatus === 'LIMITED'));
  assert.equal(s.glues[0].deleted, 1);
  detach(s);
});

test('M6: an engine call that throws inside onProcessCpu is reported once and answered with null (G2.6)', async () => {
  const s = await attachedSession('m6-engine', { hooks: { override: { processFramePyramid: () => { throw new Error('engine boom'); } } } });
  const results = runFrames(s, 8);
  assert.deepEqual(s.exceptions, []);
  assert.ok(results.every((r) => r === null));
  assert.equal(s.errors.length, 1, 'the same message is reported once');
  assert.equal(s.errors[0].message, 'engine boom');
  assert.ok(globalThis.__slamRefactChunk.stats.processErrors >= 7);
  assert.equal(s.updates, 8);
  detach(s);
});

test('readback is limited to the atlas rows the engine uses (G4)', async () => {
  const s = await attachedSession('rows');
  const layout = s.glues[0].real.pyramidLayout();
  assert.ok(layout.rows < layout.textureSize, 'the test needs rows < textureSize to be discriminating');
  runFrames(s, 6);
  const bytes = layout.textureSize * layout.rows * 4;
  const calls = s.fake.calls, PPB = s.fake.constant('PIXEL_PACK_BUFFER');
  const reads = calls.filter(([n, , , , , , , off]) => n === 'readPixels' && off === 0);
  assert.equal(reads.length, 6);
  for (const [, x, y, w, h] of reads) assert.deepEqual([x, y, w, h], [0, 0, layout.textureSize, layout.rows]);
  assert.ok(calls.some(([n, t, b]) => n === 'bufferData' && t === PPB && b instanceof Uint8Array && b.byteLength === bytes));
  for (const [, t, , dst] of calls.filter(([n]) => n === 'getBufferSubData')) assert.equal(dst.byteLength, bytes, `${t}`);
  const pyramidCalls = s.glues[0].calls.filter(([n]) => n === 'processFramePyramid');
  assert.equal(pyramidCalls.length, 5, 'the last staged frame is not processed yet');
  for (const [, , , atlas, size, rows] of pyramidCalls) assert.deepEqual([atlas.byteLength, size, rows], [bytes, layout.textureSize, layout.rows]);
  assert.equal(globalThis.__slamRefactChunk.session.readback.readbackBytes, bytes);
  detach(s);
});

test('Gr8ComputePyramid saves, neutralises and restores sampler, colorMask, unpack buffer and RASTERIZER_DISCARD (G3.1)', async () => {
  const s = await attachedSession('glstate');
  const layout = s.glues[0].real.pyramidLayout();
  const { Gr8ComputePyramid } = await import(pathToFileURL(path.join(chunkDir, 'gr8-compute-pyramid.js')).href);
  const sampler = { sentinel: 'sampler' }, unpack = { sentinel: 'unpack' };
  const { gl, calls, constant } = fakeGl({
    params: { SAMPLER_BINDING: sampler, PIXEL_UNPACK_BUFFER_BINDING: unpack, COLOR_WRITEMASK: [true, false, true, false] },
    enabled: ['RASTERIZER_DISCARD'],
  });
  const pyramid = new Gr8ComputePyramid(gl, layout, { cameraWidth: W, cameraHeight: H });
  const UNPACK = constant('PIXEL_UNPACK_BUFFER'), RD = constant('RASTERIZER_DISCARD');
  const check = (from, label, markers = ['drawElements', 'readPixels']) => {
    const part = calls.slice(from);
    const firstDraw = part.findIndex(([n]) => markers.includes(n));
    assert.ok(firstDraw > 0, `${label}: something was drawn, read or uploaded`);
    const before = part.slice(0, firstDraw);
    assert.ok(before.some(([n, u, v]) => n === 'bindSampler' && u === 0 && v === null), `${label}: sampler neutral while drawing`);
    assert.ok(before.some(([n, ...m]) => n === 'colorMask' && m.every((x) => x === true)), `${label}: colorMask open while drawing`);
    assert.ok(before.some(([n, t, b]) => n === 'bindBuffer' && t === UNPACK && b === null), `${label}: unpack buffer unbound while drawing`);
    assert.ok(before.some(([n, c]) => n === 'disable' && c === RD), `${label}: RASTERIZER_DISCARD off while drawing`);
    const lastOf = (pred) => part.filter(pred).at(-1);
    assert.deepEqual(lastOf(([n]) => n === 'bindSampler').slice(1), [0, sampler], `${label}: sampler restored`);
    assert.deepEqual(lastOf(([n]) => n === 'colorMask').slice(1), [true, false, true, false], `${label}: colorMask restored`);
    assert.equal(lastOf(([n, t]) => n === 'bindBuffer' && t === UNPACK)[2], unpack, `${label}: unpack buffer restored`);
    assert.equal(lastOf(([n, c]) => (n === 'enable' || n === 'disable') && c === RD)[0], 'enable', `${label}: RASTERIZER_DISCARD restored`);
  };
  check(0, 'constructor', ['texImage2D']);
  let from = calls.length;
  pyramid.stage({ name: 7 });
  check(from, 'stage');
  from = calls.length;
  pyramid.readTextureRgba({ name: 7 }, 4, 4);
  check(from, 'readTextureRgba');
  pyramid.dispose();
  detach(s);
});

test('Gr8ComputePyramid releases every GL object when its constructor fails half-way (G3.2)', async () => {
  const s = await attachedSession('glleak');
  const layout = s.glues[0].real.pyramidLayout();
  const { Gr8ComputePyramid } = await import(pathToFileURL(path.join(chunkDir, 'gr8-compute-pyramid.js')).href);
  const kinds = [['createShader', 'deleteShader'], ['createProgram', 'deleteProgram'], ['createTexture', 'deleteTexture'],
    ['createFramebuffer', 'deleteFramebuffer'], ['createBuffer', 'deleteBuffer'], ['createVertexArray', 'deleteVertexArray']];
  const balanced = ({ calls, created }, label) => {
    for (const [create, del] of kinds) {
      const deleted = calls.filter(([n]) => n === del).map(([, o]) => o);
      const made = created[create] ?? [];
      assert.deepEqual(new Set(deleted), new Set(made), `${label}: every ${create} object deleted`);
      assert.equal(deleted.length, made.length, `${label}: ${create} objects deleted exactly once`);
    }
  };
  const failures = [
    ...[1, 2, 3, 4, 5, 6].map((n) => [`link of program ${n}`, { failLinkAt: n }]),
    ['compile of shader 3', { failCompileAt: 3 }],
    ['incomplete framebuffer 1', { failFramebufferAt: 1 }],
    ['incomplete framebuffer 3', { failFramebufferAt: 3 }],
    ['GL error after init', { glError: 0x502 }],
  ];
  for (const [label, opts] of failures) {
    const fake = fakeGl(opts);
    assert.throws(() => new Gr8ComputePyramid(fake.gl, layout, { cameraWidth: W, cameraHeight: H }), undefined, label);
    balanced(fake, label);
    assert.ok((fake.created.createProgram ?? []).length > 0 || label.includes('shader'), label);
  }
  // A good instance releases everything in dispose().
  const ok = fakeGl();
  new Gr8ComputePyramid(ok.gl, layout, { cameraWidth: W, cameraHeight: H }).dispose();
  balanced(ok, 'dispose');
  detach(s);
});

test('diagnostic arrays are bounded (G5)', async () => {
  const s = await attachedSession('diagcap', { chunkOptions: { diagMaxSamples: 8 } });
  runFrames(s, 40);
  const st = globalThis.__slamRefactChunk.stats;
  for (const key of ['stageMs', 'readMs', 'engineMs']) assert.ok(st[key].length <= 16 && st[key].length >= 8, `${key} length ${st[key].length}`);
  assert.ok(st.samplesDropped > 0);
  assert.equal(st.stagedFrames, 40);
  assert.deepEqual(s.exceptions, []);
  detach(s);
});

test('engine manifest `file` fields are confined to the engine directory before anything is fetched (G5)', async () => {
  const { loadEngine, resolveManifestFile } = await import(pathToFileURL(path.join(chunkDir, 'engine-loader.js')).href);
  const base = new URL('slam-refact-engine/', pathToFileURL(chunkDir + '/'));
  const good = JSON.parse(fs.readFileSync(path.join(engineDir, 'manifest.json'), 'utf8'));
  const hex = 'a'.repeat(64);
  assert.ok(resolveManifestFile(base, { file: 'slam_refact_engine.js', sha256: hex }, 'js').href.startsWith(base.href));
  assert.ok(resolveManifestFile(base, { file: 'sub/x.wasm', sha256: hex }, 'wasm').href.startsWith(base.href));
  const badFiles = ['../x.js', 'a/../../x.js', '/abs.js', '//h/x.js', 'https://h/x.js', 'data:text/javascript,1', 'a\\b.js', '', '.', '..',
    'a//b.js', '.hidden.js', 'x.js?y', 'x%2e%2e.js', 7, null, undefined, {}];
  for (const file of badFiles) {
    assert.throws(() => resolveManifestFile(base, { file, sha256: hex }, 'js'), /relative path inside the engine directory/, JSON.stringify(file));
  }
  for (const sha256 of [undefined, null, 5, 'A'.repeat(64), 'a'.repeat(63), 'g'.repeat(64)]) {
    assert.throws(() => resolveManifestFile(base, { file: 'x.js', sha256 }, 'js'), /sha256 must be 64 lowercase hex/, String(sha256));
  }
  const realFetch = globalThis.fetch;
  try {
    for (const [artifact, file] of [['js', '../x.js'], ['wasm', 'https://h/x.wasm'], ['js', '/abs.js'], ['wasm', 'a\\b']]) {
      const urls = [];
      globalThis.fetch = async (url) => {
        urls.push(String(url));
        const m = structuredClone(good);
        m.artifacts[artifact].file = file;
        const b = Buffer.from(JSON.stringify(m));
        return { ok: true, status: 200, arrayBuffer: async () => b.buffer.slice(b.byteOffset, b.byteOffset + b.byteLength) };
      };
      await assert.rejects(loadEngine(base), /relative path inside the engine directory/);
      assert.deepEqual(urls, [new URL('manifest.json', base).href], `${artifact}:${file}: only the manifest was fetched`);
    }
    for (const broken of [{ ...good, artifacts: undefined }, { ...good, artifacts: { js: good.artifacts.js } }]) {
      globalThis.fetch = async () => {
        const b = Buffer.from(JSON.stringify(broken));
        return { ok: true, status: 200, arrayBuffer: async () => b.buffer.slice(b.byteOffset, b.byteOffset + b.byteLength) };
      };
      await assert.rejects(loadEngine(base), /relative path inside the engine directory/);
    }
  } finally {
    globalThis.fetch = realFetch;
  }
});

test('data recorder API: original behaviour of a recorder that is not enabled (G6)', async () => {
  const { ctrl } = await makeController('recorder');
  const warnings = [];
  const realWarn = console.warn;
  console.warn = (...a) => warnings.push(a.join(' '));
  try {
    assert.equal(ctrl.getIsDataRecording(), false);
    assert.equal(ctrl.startDatarecorder(), undefined);
    assert.deepEqual(warnings, ['[XR] Data Recorder not enabled. Enable it first before starting.']);
    assert.equal(ctrl.getIsDataRecording(), false);
    assert.equal(ctrl.stopDatarecorder(), undefined);
    warnings.length = 0;
    ctrl.configure({ enableDataRecorder: true });
    warnings.length = 0;
    assert.equal(ctrl.startDatarecorder(), undefined);
    assert.equal(warnings.length, 1);
    assert.match(warnings[0], /data recorder is not supported/);
    assert.equal(ctrl.getIsDataRecording(), false, 'never reports a recording that does not exist');
    assert.equal(ctrl.stopDatarecorder(), undefined);
  } finally {
    console.warn = realWarn;
  }
});

// =====================================================================================================
// RM-4a2 (gates G2-G9 of build/slam-refact-full-20261019/preregistration/rm4a2-chunk-gate.v1.md)
// =====================================================================================================

// A last pose that is NOT the configured origin (review M2: the fake atlas is all zeros, so the engine's own pose equals the
// configured origin (0, 2, 0) and "origin = last pose" cannot be told apart from "origin = configured origin").
const P1 = [1.5, 1.2, -0.7], Q1 = [0.5, 0.5, 0.5, -0.5];  // position, rotation (w, x, y, z)
const P1_FLAT = [1.5, 0, -0.7];                              // y <= 1e-3: the configured origin height is kept

// Glue hooks: the FIRST glue reports the given pose / status; every glue logs its responses (log: stage index -> frame k).
function injectPose(position, rotationWxyz, status = 'NORMAL') {
  const log = [];
  return {
    log,
    override: {
      processFramePyramid: (rec, ...args) => {
        const real = rec.real.processFramePyramid(...args);
        rec.lastPose = real;
        const k = Math.round((args[1] / 1e9 - 1) * 30);
        log.push({ glue: rec.idx, k, videoFrameIndex: real.videoFrameIndex, sourceFrameIndex: real.sourceFrameIndex, repeated: real.repeated });
        return rec.idx === 0 ? { ...real, position, rotationWxyz, status, reason: 'UNSPECIFIED' } : real;
      },
    },
  };
}

const poseOf = (r) => ({ position: { ...r.position }, rotation: { ...r.rotation } });

// Responses of a session that is configured with the given origin / facing from the start (control, no size change).
async function controlPose(tag, { origin, facing } = {}) {
  const s = await attachedSession(tag);
  if (origin) s.ctrl.updateCameraProjectionMatrix({ origin, facing });
  const responses = runFrames(s, 14).filter(Boolean).map(poseOf);
  detach(s);
  assert.ok(responses.length >= 8, 'control has responses');
  assert.equal(new Set(responses.map((r) => JSON.stringify(r))).size, 1, 'the control response is constant (zero atlas)');
  return responses[0];
}
const xyz = (a) => ({ x: a[0], y: a[1], z: a[2] });
const wxyz = (a) => ({ w: a[0], x: a[1], y: a[2], z: a[3] });
const W2 = 270, H2 = 360;
const resize = (s, w = W2, h = H2) => xrFrame(s, { w, h, before: () => s.pm.onVideoSizeChange(sizeChangeArgs(s, w, h)) });
const resetCalls = (rec) => rec.calls.filter(([n, a]) => n === 'updateCameraProjectionMatrix' && a.updateRecenterPoint === false && a.cam === undefined);
const fresh = (results) => results.filter(Boolean).slice(-4).map(poseOf);  // after the 2-frame gap + startup window

test('G2 (M1, M2): the origin reset after a size change = the last pose (non-origin pose; control session decides the effect)', async () => {
  const ctlDefault = await controlPose('ctl-default');
  const ctlDefault2 = await controlPose('ctl-default-2');
  assert.deepEqual(ctlDefault, ctlDefault2, 'null-run: two control sessions are bit identical');
  const ctlP1 = await controlPose('ctl-p1', { origin: xyz(P1), facing: wxyz(Q1) });
  assert.notDeepEqual(ctlP1, ctlDefault, 'the test is not degenerate: P1/q1 changes the published pose');
  const ctlFlat = await controlPose('ctl-flat', { origin: { x: 1.5, y: 2, z: -0.7 }, facing: wxyz(Q1) });
  assert.notDeepEqual(ctlFlat, ctlP1);

  // 1 + 2: the reset call and its effect.
  const hooks = injectPose(P1, Q1);
  const s = await attachedSession('g2-size', { hooks });
  runFrames(s, 9);
  const results = [resize(s), ...runFrames(s, 11, { w: W2, h: H2 })];
  assert.deepEqual(s.exceptions, []);
  assert.deepEqual(s.errors, []);
  const resets = resetCalls(s.glues[1]);
  assert.equal(resets.length, 1);
  assert.deepEqual(resets[0][1], { origin: xyz(P1), facing: wxyz(Q1), updateRecenterPoint: false });
  assert.deepEqual(fresh(results).map((r) => JSON.stringify(r)), Array(4).fill(JSON.stringify(ctlP1)), 'effect: published pose = control started with P1/q1');
  detach(s);

  // 3: y <= 1e-3 keeps the configured origin height.
  const flat = await attachedSession('g2-flat', { hooks: injectPose(P1_FLAT, Q1) });
  runFrames(flat, 9);
  const flatResults = [resize(flat), ...runFrames(flat, 11, { w: W2, h: H2 })];
  const flatReset = resetCalls(flat.glues[1]);
  assert.equal(flatReset.length, 1);
  assert.deepEqual(flatReset[0][1].origin, { x: 1.5, y: 2, z: -0.7 });
  assert.deepEqual(fresh(flatResults).map((r) => JSON.stringify(r)), Array(4).fill(JSON.stringify(ctlFlat)));
  detach(flat);

  // 4: an orientation change does not reset (the original configureXr that follows overrides it).
  const orient = await attachedSession('g2-orient', { hooks: injectPose(P1, Q1) });
  runFrames(orient, 9);
  const orientResults = [xrFrame(orient, { w: W2, h: H2, before: () => orient.pm.onDeviceOrientationChange(sizeChangeArgs(orient, W2, H2)) }),
    ...runFrames(orient, 11, { w: W2, h: H2 })];
  assert.equal(resetCalls(orient.glues[1]).length, 0);
  assert.deepEqual(fresh(orientResults).map((r) => JSON.stringify(r)), Array(4).fill(JSON.stringify(ctlDefault)));
  detach(orient);

  // 5: a second size change resets to the response of the glue that was running (published pose, not the first injection).
  const two = await attachedSession('g2-two', { hooks: injectPose(P1, Q1) });
  runFrames(two, 9);
  resize(two);
  runFrames(two, 8, { w: W2, h: H2 });
  const published = two.glues[1].lastPose;
  assert.ok(published && !published.repeated);
  xrFrame(two, { w: 300, h: 400, before: () => two.pm.onVideoSizeChange(sizeChangeArgs(two, 300, 400)) });
  const second = resetCalls(two.glues[2]);
  assert.equal(second.length, 1);
  assert.deepEqual(second[0][1].origin, { x: published.position[0], y: published.position[1], z: published.position[2] });
  assert.deepEqual(second[0][1].facing, wxyz(Array.from(published.rotationWxyz)));
  detach(two);
});

test('G5 (E1): while the new pipeline fills, the last response of the old session is repeated', async () => {
  const gap = await attachedSession('g5-gap', { hooks: injectPose(P1, Q1) });
  const before = runFrames(gap, 9);
  const gapResults = [resize(gap), ...runFrames(gap, 3, { w: W2, h: H2 })];
  assert.deepEqual(gapResults[0], before.at(-1));
  assert.deepEqual(gapResults[1], before.at(-1));
  assert.equal(gapResults[0].position.x, 1.5);
  assert.deepEqual(gap.errors, []);
  detach(gap);
});

async function g3Setup(tag, after) {
  const ctlDefault = await controlPose(`${tag}-ctl-default`);
  const ctlP1 = await controlPose(`${tag}-ctl-p1`, { origin: xyz(P1), facing: wxyz(Q1) });
  const s = await attachedSession(tag, { hooks: injectPose(P1, Q1) });
  runFrames(s, 9);
  resize(s);
  const settled = runFrames(s, 7, { w: W2, h: H2 });
  assert.deepEqual(fresh(settled).map((r) => JSON.stringify(r)), Array(4).fill(JSON.stringify(ctlP1)), 'the reset is in effect before the call under test');
  const rs = after(s);
  assert.deepEqual(s.exceptions, []);
  assert.deepEqual(s.errors, []);
  return { s, rs, ctlDefault, ctlP1 };
}
const sameAs = (rs, ref, label) => assert.deepEqual(fresh(rs).map((r) => JSON.stringify(r)), Array(4).fill(JSON.stringify(ref)), label);

test('G3a (M1): recenter() after a size change returns to the user origin (rebuildEngine applies the configured coordinate system)', async () => {
  const r = await g3Setup('g3a', (s) => { s.ctrl.recenter(); return runFrames(s, 9, { w: W2, h: H2 }); });
  sameAs(r.rs, r.ctlDefault, 'after recenter');
  detach(r.s);
});

test('G3b (M1): configure() after a size change re-sends the JS origin (original configureXr)', async () => {
  const r = await g3Setup('g3b', (s) => { s.ctrl.configure({ enableLighting: true }); return runFrames(s, 9, { w: W2, h: H2 }); });
  sameAs(r.rs, r.ctlDefault, 'after configure');
  detach(r.s);
});

test('G3c (M1): updateCameraProjectionMatrix({cam}) after a size change re-sends the JS origin', async () => {
  const r = await g3Setup('g3c', (s) => {
    s.ctrl.updateCameraProjectionMatrix({ cam: { pixelRectWidth: 250, pixelRectHeight: 330, nearClipPlane: 0.01, farClipPlane: 1000 } });
    return runFrames(s, 9, { w: W2, h: H2 });
  });
  sameAs(r.rs, r.ctlDefault, 'after updateCameraProjectionMatrix');
  detach(r.s);
});

test('G3d (M1): onCanvasSizeChange sends the camera mask only - the reset coordinate system stays', async () => {
  const r = await g3Setup('g3d', (s) => {
    s.pm.onCanvasSizeChange({ canvasWidth: 250, canvasHeight: 330 });
    return runFrames(s, 9, { w: W2, h: H2 });
  });
  sameAs(r.rs, r.ctlP1, 'after onCanvasSizeChange');
  detach(r.s);
});

test('G3e (M1): recenter, second size change, recenter; the JS origin in cameraconfigured never moves', async () => {
  const r = await g3Setup('g3e', (s) => {
    s.ctrl.recenter();
    runFrames(s, 9, { w: W2, h: H2 });
    xrFrame(s, { w: 300, h: 400, before: () => s.pm.onVideoSizeChange(sizeChangeArgs(s, 300, 400)) });
    runFrames(s, 6, { w: 300, h: 400 });
    s.ctrl.recenter();
    return runFrames(s, 9, { w: 300, h: 400 });
  });
  sameAs(r.rs, r.ctlDefault, 'after the second recenter');
  const cc = r.s.events.filter(([n]) => n === 'cameraconfigured');
  assert.ok(cc.length >= 3 && cc.some(([, d]) => d.recenter === true));
  for (const [, d] of cc) assert.deepEqual(d.origin, { position: { x: 0, y: 2, z: 0 }, rotation: { w: 1, x: 0, y: 0, z: 0 } });
  detach(r.s);
});

test('G3f (M1): a size change before any response starts the new engine with the configured coordinate system, not the JS origin', async () => {
  // updateCameraProjectionMatrix(updateRecenterPoint: false) moves the configured system (origin A) but not the recenter point;
  // recenter() then puts the JS origin back to the recenter point (0, 2, 0) without a configureXr, so configured (A) != JS (P).
  const A = { x: 3, y: 2, z: 1 };
  const ctlA = await controlPose('g3f-ctl-a', { origin: A, facing: wxyz([1, 0, 0, 0]) });
  const ctlDefault = await controlPose('g3f-ctl-default');
  assert.notDeepEqual(ctlA, ctlDefault);
  const s = await attachedSession('g3f');
  s.ctrl.updateCameraProjectionMatrix({ origin: A, facing: { w: 1, x: 0, y: 0, z: 0 }, updateRecenterPoint: false });
  s.ctrl.recenter();
  const results = [resize(s), ...runFrames(s, 11, { w: W2, h: H2 })];  // no response existed: no origin reset
  assert.equal(resetCalls(s.glues[1]).length, 0);
  sameAs(results, ctlA, 'engineInit applies configuredCoordinateSystem');
  assert.deepEqual(s.errors, []);
  detach(s);
});

test('G4 (M3): an unsupported capture is inactive without errorCallback - one warn per episode, LIMITED last pose, live camera texture', async () => {
  const s = await attachedSession('g4-inactive', { hooks: injectPose(P1, Q1) });
  const warns = [];
  const realWarn = console.warn;
  console.warn = (...a) => warns.push(a.join(' '));
  try {
    const before = runFrames(s, 9);
    const last = before.at(-1);
    assert.equal(last.trackingStatus, 'NORMAL');
    const created = globalThis.__slamRefactChunk.stats.sessionsCreated;
    const stagedBefore = globalThis.__slamRefactChunk.stats.stagedFrames;
    const callsBefore = s.glues[0].calls.filter(([n]) => n === 'processFramePyramid').length;
    const portrait = { w: W, h: H }, landscape = { w: H, h: W };
    const results = [];
    // one rotation as xr.js delivers it (review exp3): orientationchange with the OLD size, then the size change, then the way back.
    results.push(xrFrame(s, { ...portrait, before: () => s.pm.onDeviceOrientationChange(sizeChangeArgs(s, W, H, UNSUPPORTED_ROTATION)) }));
    results.push(xrFrame(s, { ...landscape, before: () => s.pm.onVideoSizeChange(sizeChangeArgs(s, H, W, UNSUPPORTED_ROTATION)) }));
    results.push(...runFrames(s, 4, landscape));
    results.push(xrFrame(s, { ...landscape, before: () => s.pm.onDeviceOrientationChange(sizeChangeArgs(s, H, W, UNSUPPORTED_ROTATION)) }));
    results.push(...runFrames(s, 2, landscape));
    assert.deepEqual(s.exceptions, [], 'no callback throws');
    assert.deepEqual(s.errors, [], 'errorCallback is not called (XRExtras RuntimeError would stop the session)');
    assert.equal(warns.filter((w) => /supports rotation 0, 90, 180, 270/.test(w)).length, 1, 'one warn per episode');
    assert.ok(!warns.some((w) => /\d{3}x\d{3}/.test(w)), 'the warn text has no sizes (so it cannot differ between the three callbacks)');
    results.forEach((r, i) => {
      assert.ok(r, `inactive response ${i}`);
      assert.equal(r.trackingStatus, 'LIMITED');
      assert.equal(r.trackingReason, 'UNSPECIFIED');
      assert.deepEqual(r.worldPoints, []);
      assert.deepEqual(r.position, last.position);
      assert.deepEqual(r.rotation, last.rotation);
      assert.deepEqual(r.intrinsics, last.intrinsics);
    });
    results.slice(2).forEach((r, i) => assert.deepEqual(r.realityTexture, { cam: 100 + 9 + 2 + i }, 'live camera texture of this frame'));
    assert.equal(globalThis.__slamRefactChunk.stats.sessionsCreated, created, 'no session while inactive');
    assert.equal(globalThis.__slamRefactChunk.stats.stagedFrames, stagedBefore, 'no staging while inactive');
    assert.equal(s.glues[0].calls.filter(([n]) => n === 'processFramePyramid').length, callsBefore, 'no engine call while inactive');
    // back to portrait: the session is rebuilt, responses resume.
    const back = [xrFrame(s, { ...portrait, before: () => s.pm.onVideoSizeChange(sizeChangeArgs(s, W, H, 0)) }), ...runFrames(s, 7, portrait)];
    assert.equal(s.glues.length, 2);
    assert.equal(globalThis.__slamRefactChunk.stats.sessionsCreated, created + 1);
    assert.ok(back.every(Boolean));
    assert.deepEqual(s.errors, []);
    const statuses = s.events.filter(([n]) => n === 'trackingstatus').map(([, d]) => d.status);
    assert.deepEqual(statuses.slice(0, 3), ['NORMAL', 'LIMITED', 'NORMAL'], 'LIMITED once on entering, normal again after the way back');
    // second rotation = new episode = one more warn
    xrFrame(s, { ...portrait, before: () => s.pm.onDeviceOrientationChange(sizeChangeArgs(s, W, H, UNSUPPORTED_ROTATION)) });
    runFrames(s, 3, landscape);
    assert.equal(warns.filter((w) => /supports rotation 0, 90, 180, 270/.test(w)).length, 2);
    assert.deepEqual(s.errors, []);
    assert.deepEqual(s.exceptions, []);
  } finally {
    console.warn = realWarn;
  }
  detach(s);
});

test('G4b (M3): a session that never had a response answers null while inactive', async () => {
  const s = await attachedSession('g4-never');
  const r = xrFrame(s, { w: H, h: W, before: () => s.pm.onDeviceOrientationChange(sizeChangeArgs(s, H, W, UNSUPPORTED_ROTATION)) });
  assert.equal(r, null);
  assert.deepEqual(runFrames(s, 3, { w: H, h: W }), [null, null, null]);
  assert.deepEqual(s.errors, []);
  assert.deepEqual(s.exceptions, []);
  detach(s);
});

test('G6 (m3): a transient engine exception does not desynchronise the stage index', async () => {
  let calls = 0;
  const log = [];
  const hooks = { override: { processFramePyramid: (rec, ...args) => {
    if (++calls === 7) throw new Error('transient engine failure');
    const real = rec.real.processFramePyramid(...args);
    log.push({ k: Math.round((args[1] / 1e9 - 1) * 30), videoFrameIndex: real.videoFrameIndex, sourceFrameIndex: real.sourceFrameIndex, repeated: real.repeated });
    return real;
  } } };
  const s = await attachedSession('g6-transient', { hooks });
  const results = [];
  const seen = [];
  for (let i = 0; i < 24; i++) {
    const before = log.length;
    results.push(xrFrame(s));
    seen.push(log.length > before ? log.at(-1) : null);
  }
  assert.deepEqual(s.exceptions, []);
  assert.equal(s.errors.length, 1, 'one errorCallback for the one failure');
  assert.equal(s.errors[0].message, 'transient engine failure');
  assert.ok(globalThis.__slamRefactChunk.warnings.every((w) => !/stage index/.test(w)), 'no index-carrying message (it would defeat the once-per-message report)');
  const failedAt = results.findIndex((_, i) => i > 0 && seen[i] === null && i > 3);
  assert.ok(failedAt > 0);
  assert.ok(results.slice(failedAt + 3).every(Boolean), 'responses resume within 3 frames and stay non-null');
  // fresh responses carry the texture of the frame the engine tracked
  const kOfStage = new Map(log.map((e) => [e.videoFrameIndex, e.k]));
  let checked = 0;
  results.forEach((r, i) => {
    const e = seen[i];
    if (!r || !e || e.repeated || i < failedAt) return;
    assert.deepEqual(r.realityTexture, { drawFor: { name: 100 + kOfStage.get(e.sourceFrameIndex) } }, `frame ${i}`);
    checked++;
  });
  assert.ok(checked >= 8, `${checked} fresh responses checked`);
  detach(s);
});

test('G8: a lost WebGL context is reported once and stops tracking instead of repeating a stale pose', async () => {
  let calls = 0;
  const hooks = { override: { processFramePyramid: (rec, ...args) => { calls++; return rec.real.processFramePyramid(...args); } } };
  const s = await attachedSession('g8-lost', { hooks });
  const before = runFrames(s, 8);
  assert.ok(before.at(-1));
  const callsBefore = calls;
  s.fake.lose();
  const lostResults = runFrames(s, 5);
  assert.deepEqual(s.exceptions, []);
  assert.equal(s.errors.length, 1, 'one report');
  assert.match(s.errors[0].message, /context was lost/);
  assert.ok(calls - callsBefore <= 1, 'at most the frame that was staged before the loss is processed');
  assert.ok(lostResults.slice(1).every((r) => r === null), 'no stale pose / texture is repeated after the loss');
  const afterLoss = calls;
  runFrames(s, 3);
  assert.equal(calls, afterLoss, 'no engine call while lost');
  // the context comes back (XR8 itself does not restore it) and the session is rebuilt, e.g. by a size change
  s.fake.restore();
  const resumed = [resize(s), ...runFrames(s, 7, { w: W2, h: H2 })];
  assert.deepEqual(s.exceptions, []);
  assert.ok(resumed.slice(2).every(Boolean), 'responses resume after a rebuild');
  assert.equal(s.errors.length, 1);
  detach(s);
});

test('G7: Gr8ComputePyramid restores the complete GL state (stateful fake) and draws with a neutral state', async () => {
  const s = await attachedSession('g7-state');
  const layout = s.glues[0].real.pyramidLayout();
  const { Gr8ComputePyramid } = await import(pathToFileURL(path.join(chunkDir, 'gr8-compute-pyramid.js')).href);
  const T0 = 0x84c0, T3 = 0x84c3;
  const init = {
    activeTexture: T3, tex: { [T0]: { tex: 'unit0' }, [T3]: { tex: 'unit3' } }, samplers: { [T0]: { sampler: 'unit0' }, [T3]: { sampler: 'unit3' } },
    drawFb: { fb: 'draw' }, readFb: { fb: 'read' }, program: { program: 'app' }, vao: { vao: 'app' }, arrayBuffer: { buffer: 'array' },
    packBuffer: { buffer: 'pack' }, unpackBuffer: { buffer: 'unpack' }, packAlignment: 1, colorMask: [true, false, true, false],
    viewport: [3, 4, 50, 60],
    caps: { BLEND: true, SCISSOR_TEST: true, DEPTH_TEST: false, STENCIL_TEST: true, CULL_FACE: false, RASTERIZER_DISCARD: true },
  };
  const fake = fakeGl({ state: init });
  const initial = fake.state.snapshot();
  const neutral = (snap, label) => {
    assert.equal(snap.state.samplers[T0], null, `${label}: no sampler object on unit 0 while drawing`);
    assert.deepEqual(snap.state.colorMask, [true, true, true, true], `${label}: colorMask open`);
    assert.equal(snap.state.unpackBuffer, null, `${label}: no unpack buffer`);
    assert.equal(snap.state.caps.RASTERIZER_DISCARD, false, `${label}: RASTERIZER_DISCARD off`);
    for (const c of ['BLEND', 'SCISSOR_TEST', 'DEPTH_TEST', 'STENCIL_TEST', 'CULL_FACE']) assert.equal(snap.state.caps[c], false, `${label}: ${c} off`);
    assert.equal(snap.state.packAlignment, 4, `${label}: pack alignment`);
  };
  const firstOf = (from, call) => fake.snapshots.find((x) => x.index >= from && x.call === call);
  const pyramid = new Gr8ComputePyramid(fake.gl, layout, { cameraWidth: W, cameraHeight: H });
  assert.deepEqual(fake.state.snapshot(), initial, 'constructor restores the state');
  neutral(firstOf(0, 'texImage2D'), 'constructor');
  let from = fake.calls.length;
  pyramid.stage({ name: 7 });
  assert.deepEqual(fake.state.snapshot(), initial, 'stage() restores the state');
  neutral(firstOf(from, 'drawElements'), 'stage');
  from = fake.calls.length;
  pyramid.readTextureRgba({ name: 7 }, 4, 4);
  assert.deepEqual(fake.state.snapshot(), initial, 'readTextureRgba() restores the state');
  neutral(firstOf(from, 'readPixels'), 'readTextureRgba');
  from = fake.calls.length;
  pyramid.read();
  assert.deepEqual(fake.state.snapshot(), initial, 'read() leaves the pack buffer binding as it found it');
  pyramid.dispose();
  // a constructor that fails half-way restores the state too
  const failing = fakeGl({ state: init, failLinkAt: 4 });
  assert.throws(() => new Gr8ComputePyramid(failing.gl, layout, { cameraWidth: W, cameraHeight: H }));
  assert.deepEqual(failing.state.snapshot(), initial, 'a failed constructor restores the state');
  detach(s);
});

test('G9: engine fetches refuse redirects; a replaced glue is flagged; diagnostic limits are fixed and enforced', async () => {
  const { loadEngine } = await import(pathToFileURL(path.join(chunkDir, 'engine-loader.js')).href);
  const base = new URL('slam-refact-engine/', pathToFileURL(chunkDir + '/'));
  const realFetch = globalThis.fetch;
  const inits = [];
  globalThis.fetch = async (url, init) => { inits.push(init); return realFetch(url, init); };
  try {
    await loadEngine(base);
  } finally {
    globalThis.fetch = realFetch;
  }
  assert.equal(inits.length, 3, 'manifest + js + wasm');
  for (const init of inits) assert.deepEqual(init, { cache: 'no-store', credentials: 'same-origin', redirect: 'error' });

  const plain = await makeController('g9-plain');
  assert.equal(globalThis.__slamRefactChunk.engine.glueReplaced, false);
  assert.deepEqual(globalThis.__slamRefactChunk.limits, { maxSamples: 20000, maxEntries: 100 });
  void plain;
  const s = await attachedSession('g9-hook');
  assert.equal(globalThis.__slamRefactChunk.engine.glueReplaced, true, 'options.createGlue is flagged in the engine diagnostics');
  detach(s);

  // more than 100 distinct reports: the arrays stay at 100 and the overflow is counted
  let n = 0;
  const flood = await attachedSession('g9-flood', { hooks: { override: { processFramePyramid: () => { throw new Error(`distinct failure ${++n}`); } } } });
  runFrames(flood, 160);
  const diag = globalThis.__slamRefactChunk;
  assert.equal(diag.warnings.length, 100);
  assert.ok(flood.errors.length <= 100);
  assert.ok(diag.stats.warningsDropped >= 50, `warningsDropped ${diag.stats.warningsDropped}`);
  assert.deepEqual(flood.exceptions, []);
  detach(flood);
});

// =====================================================================================================
// RM-4c (gate build/slam-refact-full-20261019/preregistration/rm4c-chunk-gate.v1.md): RM-4a3 (review RM-4a2) + RM-4b JS
// =====================================================================================================
import { spawnSync } from 'node:child_process';
import { createHash } from 'node:crypto';

const sha256Of = (file) => createHash('sha256').update(fs.readFileSync(file)).digest('hex');
const poseJson = (rs) => fresh(rs).map((r) => JSON.stringify(r));
const sameAsRef = (rs, ref, label) => assert.deepEqual(poseJson(rs), Array(4).fill(JSON.stringify(ref)), label);

// ---- G-A: configured coordinate system C != JS origin P != recenter point p (review MAJOR-1; probes REV-T1..T8) ----
// updateCameraProjectionMatrix({origin: A, updateRecenterPoint: false}) moves C (= P) to A, recenter() puts P back to p =
// (0, 2, 0) without a configureXr: C = A, P = p = (0, 2, 0).
const A_R = { x: 3, y: 2, z: 1 };
const FACING_ID = { w: 1, x: 0, y: 0, z: 0 };
async function cpSetup(tag) {
  const ctlA = await controlPose(`${tag}-ctl-a`, { origin: A_R, facing: wxyz([1, 0, 0, 0]) });
  const ctlDefault = await controlPose(`${tag}-ctl-default`);
  assert.notDeepEqual(ctlA, ctlDefault, 'the scenario is not degenerate: C != P gives different published poses');
  const s = await attachedSession(tag, { hooks: injectPose(P1, Q1) });
  s.ctrl.updateCameraProjectionMatrix({ origin: A_R, facing: FACING_ID, updateRecenterPoint: false });
  s.ctrl.recenter();
  runFrames(s, 9);
  return { s, ctlA, ctlDefault };
}

test('RM4C-A1a: recenter() after a size change goes to the CONFIGURED coordinate system (C != P != p)', async () => {
  const { s, ctlA } = await cpSetup('rm4c-a1a');
  resize(s);
  runFrames(s, 7, { w: W2, h: H2 });
  s.ctrl.recenter();
  sameAsRef(runFrames(s, 9, { w: W2, h: H2 }), ctlA, 'after recenter: C = A');
  assert.deepEqual(s.errors, []);
  detach(s);
});

test('RM4C-A1b: configure() after a size change re-sends the JS origin P (not C)', async () => {
  const { s, ctlDefault } = await cpSetup('rm4c-a1b');
  resize(s);
  runFrames(s, 7, { w: W2, h: H2 });
  s.ctrl.configure({ enableLighting: true });
  sameAsRef(runFrames(s, 9, { w: W2, h: H2 }), ctlDefault, 'after configure: P = p');
  detach(s);
});

test('RM4C-A1c: updateCameraProjectionMatrix({cam}) after a size change re-sends the JS origin P', async () => {
  const { s, ctlDefault } = await cpSetup('rm4c-a1c');
  resize(s);
  runFrames(s, 7, { w: W2, h: H2 });
  s.ctrl.updateCameraProjectionMatrix({ cam: { pixelRectWidth: W, pixelRectHeight: H, nearClipPlane: 0.01, farClipPlane: 1000 } });
  sameAsRef(runFrames(s, 9, { w: W2, h: H2 }), ctlDefault, 'after updateCameraProjectionMatrix({cam}): P');
  detach(s);
});

test('RM4C-A1d: an orientation change (portrait) starts from the JS origin P, not C', async () => {
  const { s, ctlDefault } = await cpSetup('rm4c-a1d');
  const rs = [xrFrame(s, { w: W2, h: H2, before: () => s.pm.onDeviceOrientationChange(sizeChangeArgs(s, W2, H2)) }), ...runFrames(s, 11, { w: W2, h: H2 })];
  sameAsRef(rs, ctlDefault, 'after the orientation change: P');
  // the new glue is created with P (engineInit + configureXr(P)); a creation with C that is corrected by a re-send afterwards
  // ends in the same pose, so the creation call itself is checked too
  const created = s.glues[1].calls.find(([n, a]) => n === 'updateCameraProjectionMatrix' && a.cam !== undefined && a.origin !== undefined);
  assert.deepEqual(created[1].origin, { x: 0, y: 2, z: 0 }, 'created with the JS origin');
  detach(s);
});

test('RM4C-A1e: the y <= 1e-3 rule of the origin reset takes the CONFIGURED origin height', async () => {
  const s = await attachedSession('rm4c-a1e', { hooks: injectPose(P1_FLAT, Q1) });
  s.ctrl.updateCameraProjectionMatrix({ origin: { x: 0, y: 3, z: 0 }, facing: FACING_ID, updateRecenterPoint: false });
  s.ctrl.recenter();  // P.y = 2 (recenter point), C.y = 3
  runFrames(s, 9);
  resize(s);
  const resets = resetCalls(s.glues[1]);
  assert.equal(resets.length, 1);
  assert.equal(resets[0][1].origin.y, 3, 'configuredCoordinateSystem origin.y, not the JS 2');
  detach(s);
});

test('RM4C-A1f: the first attach starts at the CONFIGURED coordinate system; the next configure() re-sends P', async () => {
  const ctlA = await controlPose('rm4c-a1f-ctl-a', { origin: A_R, facing: wxyz([1, 0, 0, 0]) });
  const ctlDefault = await controlPose('rm4c-a1f-ctl-default');
  const rec = glueRecorder();
  const { ctrl, errors } = await makeController('rm4c-a1f', { createGlue: rec.createGlue });
  const pm = ctrl.pipelineModule();
  const framework = { dispatchEvent: () => {} };
  ctrl.configure({ enableWorldPoints: true });
  ctrl.updateCameraProjectionMatrix({ cam: { pixelRectWidth: W, pixelRectHeight: H, nearClipPlane: 0.01, farClipPlane: 1000 },
    origin: A_R, facing: FACING_ID, updateRecenterPoint: false });
  ctrl.recenter();  // P = p = (0, 2, 0); C = A
  pm.onBeforeSessionInitialize({ sessionAttributes: { fillsCameraTexture: true, controlsCamera: false } });
  const fake = fakeGl();
  pm.onStart({ computeCtx: fake.gl, canvasWidth: W, canvasHeight: H });
  pm.onSessionAttach({ videoWidth: W, videoHeight: H, canvasWidth: W, canvasHeight: H, rotation: 0, computeCtx: fake.gl, framework,
    GLctx: fakeGl().gl, sessionAttributes: { fillsCameraTexture: true } });
  const s = { ctrl, pm, errors, events: [], framework, fake, glues: rec.glues, k: 0, updates: 0, exceptions: [] };
  sameAsRef(runFrames(s, 12), ctlA, 'engineInit applies the configured coordinate system');
  ctrl.configure({ enableLighting: true });
  sameAsRef(runFrames(s, 9), ctlDefault, 'configureXr sends P');
  assert.deepEqual(s.exceptions, []);
  assert.deepEqual(errors, []);
  detach(s);
});

test('RM4C-A2: recenter() while no session exists (inactive capture) drops the pose of the old session as the next origin', async () => {
  const ctlDefault = await controlPose('rm4c-a2-ctl-default');
  const ctlP1 = await controlPose('rm4c-a2-ctl-p1', { origin: xyz(P1), facing: wxyz(Q1) });
  assert.notDeepEqual(ctlDefault, ctlP1);
  const s = await attachedSession('rm4c-a2', { hooks: injectPose(P1, Q1) });
  runFrames(s, 9);
  const warn = console.warn; console.warn = () => {};
  let got;
  try {
    xrFrame(s, { w: W, h: H, before: () => s.pm.onDeviceOrientationChange(sizeChangeArgs(s, W, H, UNSUPPORTED_ROTATION)) });
    xrFrame(s, { w: H, h: W, before: () => s.pm.onVideoSizeChange(sizeChangeArgs(s, H, W, UNSUPPORTED_ROTATION)) });
    runFrames(s, 2, { w: H, h: W });
    s.ctrl.recenter();
    runFrames(s, 2, { w: H, h: W });
    xrFrame(s, { w: H, h: W, before: () => s.pm.onDeviceOrientationChange(sizeChangeArgs(s, H, W, UNSUPPORTED_ROTATION)) });
    const back = [xrFrame(s, { w: W, h: H, before: () => s.pm.onVideoSizeChange(sizeChangeArgs(s, W, H, 0)) }), ...runFrames(s, 11)];
    got = back;
  } finally {
    console.warn = warn;
  }
  sameAsRef(got, ctlDefault, 'back in portrait the world restarts at the configured origin, not at the pose before the rotation');
  assert.equal(s.events.filter(([n, d]) => n === 'cameraconfigured' && d.recenter === true).length, 1);
  assert.deepEqual(s.errors, []);
  detach(s);
});

test('RM4C-A3: the gap right after an inactive episode is LIMITED with the live camera; NORMAL comes with the first fresh response', async () => {
  const s = await attachedSession('rm4c-a3', { hooks: injectPose(P1, Q1) });
  const before = runFrames(s, 9);
  assert.equal(before.at(-1).trackingStatus, 'NORMAL');
  const warn = console.warn; console.warn = () => {};
  let back, kBack;
  try {
    xrFrame(s, { w: W, h: H, before: () => s.pm.onDeviceOrientationChange(sizeChangeArgs(s, W, H, UNSUPPORTED_ROTATION)) });
    xrFrame(s, { w: H, h: W, before: () => s.pm.onVideoSizeChange(sizeChangeArgs(s, H, W, UNSUPPORTED_ROTATION)) });
    runFrames(s, 20, { w: H, h: W });
    xrFrame(s, { w: H, h: W, before: () => s.pm.onDeviceOrientationChange(sizeChangeArgs(s, H, W, UNSUPPORTED_ROTATION)) });
    kBack = s.k;
    back = [xrFrame(s, { w: W, h: H, before: () => s.pm.onVideoSizeChange(sizeChangeArgs(s, W, H, 0)) }), ...runFrames(s, 4)];
  } finally {
    console.warn = warn;
  }
  for (let i = 0; i < 2; i++) {
    assert.equal(back[i].trackingStatus, 'LIMITED', `gap answer ${i}`);
    assert.equal(back[i].trackingReason, 'UNSPECIFIED');
    assert.deepEqual(back[i].realityTexture, { cam: 100 + kBack + i }, `gap answer ${i}: the live camera texture of this frame, not the one from before the rotation`);
    assert.deepEqual(back[i].worldPoints, []);
  }
  assert.equal(back[2].trackingStatus, 'NORMAL', 'first fresh response');
  assert.deepEqual(back[2].realityTexture.drawFor !== undefined, true, 'a pipeline texture again');
  assert.deepEqual(s.events.filter(([n]) => n === 'trackingstatus').map(([, d]) => d.status), ['NORMAL', 'LIMITED', 'NORMAL']);
  // an ordinary size change (no inactive episode) still repeats the last response (E1, tested above): its gap is not LIMITED.
  assert.deepEqual(s.exceptions, []);
  detach(s);
});

test('RM4C-A4: a size change while the compute context is still lost reports once, answers null, and resumes after the context came back', async () => {
  const s = await attachedSession('rm4c-a4');
  runFrames(s, 8);
  s.fake.lose();
  runFrames(s, 3);
  assert.equal(s.errors.length, 1, 'the loss is reported once');
  const lostResize = [resize(s), ...runFrames(s, 4, { w: W2, h: H2 })];
  assert.deepEqual(s.exceptions, []);
  assert.equal(s.errors.length, 1, 'no second report (no GL error 0x9242 from a session rebuild attempt)');
  assert.ok(s.errors.every((e) => /context was lost/.test(e.message)));
  assert.ok(lostResize.every((r) => r === null), 'null while lost, with or without a session');
  assert.equal(globalThis.__slamRefactChunk.session.videoWidth, W, 'no session was built while lost');
  s.fake.restore();
  const resumed = [resize(s, 300, 400), ...runFrames(s, 7, { w: 300, h: 400 })];
  assert.deepEqual(s.exceptions, []);
  assert.equal(s.errors.length, 1);
  assert.ok(resumed.slice(2).every(Boolean), 'responses resume once the context is back and the session is rebuilt');
  detach(s);
});

test('RM4C-A5: glueReplaced is decided at every session creation, not once at engine load', async () => {
  const opts = {};
  const { ctrl } = await makeController('rm4c-a5', opts);
  const pm = ctrl.pipelineModule();
  const attach = () => {
    const fake = fakeGl();
    pm.onBeforeSessionInitialize({ sessionAttributes: { fillsCameraTexture: true, controlsCamera: false } });
    pm.onStart({ computeCtx: fake.gl, canvasWidth: W, canvasHeight: H });
    pm.onSessionAttach({ videoWidth: W, videoHeight: H, canvasWidth: W, canvasHeight: H, rotation: 0, computeCtx: fake.gl,
      framework: { dispatchEvent: () => {} }, GLctx: fakeGl().gl, sessionAttributes: { fillsCameraTexture: true } });
  };
  attach();
  assert.equal(globalThis.__slamRefactChunk.engine.glueReplaced, false, 'no hook, no replaced glue');
  pm.onSessionDetach({}); pm.onDetach({});
  const rec = glueRecorder();
  opts.createGlue = rec.createGlue;  // injected AFTER the engine was loaded
  attach();
  assert.equal(rec.glues.length, 1, 'the hook was used');
  assert.equal(globalThis.__slamRefactChunk.engine.glueReplaced, true, 'and the evidence says so');
  pm.onSessionDetach({}); pm.onDetach({});
});

test('RM4C-A6: record-provenance names the engine bytes, the chunk sources and the engine source tree, and fails on a manifest mismatch', async () => {
  const tool = path.join(chunkDir, 'tools/record-provenance.mjs');
  const run = (dir) => spawnSync(process.execPath, [tool, '--engine-dir', path.relative(repo, dir)], { cwd: repo, encoding: 'utf8', env: { ...process.env, SLAM_REFACT_ROOT: repo } });
  const r = run(engineDir);
  assert.equal(r.status, 0, r.stderr);
  const p = JSON.parse(r.stdout);
  assert.equal(p.engine.matchesManifest, true);
  assert.equal(p.engine.wasm.sha256, sha256Of(path.join(engineDir, 'slam_refact_engine.wasm')));
  assert.equal(p.engine.js.sha256, sha256Of(path.join(engineDir, 'slam_refact_engine.js')));
  assert.equal(p.engine.manifestSha256, sha256Of(path.join(engineDir, 'manifest.json')));
  assert.equal(p.chunk.files['slam_refact/web-chunk/xr-slam.js'], sha256Of(path.join(repo, 'slam_refact/web-chunk/xr-slam.js')));
  assert.ok(Object.keys(p.chunk.files).length >= 8);
  // RM-4d (review RM-4c m5): the tree as it is at recording time is labelled so; the build's own state comes from the build provenance
  assert.ok(p.engineSources.atRecord.files > 900 && /^[0-9a-f]{64}$/.test(p.engineSources.atRecord.sha256));
  assert.equal(p.engineSources.sha256, undefined, 'the unlabelled record-time field is gone');
  assert.match(p.git.head, /^[0-9a-f]{40}$/);
  // a scratch directory under build/ that is not an evidence directory (an evidence directory can be made read-only)
  const tmpRoot = path.join(repo, 'build/slam-refact-full-chunk-test-tmp');
  fs.mkdirSync(tmpRoot, { recursive: true });
  const tmp = fs.mkdtempSync(path.join(tmpRoot, 'tmp-prov-'));
  try {
    for (const f of ['manifest.json', 'slam_refact_engine.js', 'slam_refact_engine.wasm']) {
      fs.copyFileSync(path.join(engineDir, f), path.join(tmp, f));
      fs.chmodSync(path.join(tmp, f), 0o644);  // the engine directory may be read-only (pinned); the copy is ours to edit
    }
    // a wasm that differs from its manifest: exit code 1 and matchesManifest false
    const wasm = fs.readFileSync(path.join(tmp, 'slam_refact_engine.wasm'));
    wasm[wasm.length - 1] ^= 1;
    fs.writeFileSync(path.join(tmp, 'slam_refact_engine.wasm'), wasm);
    const bad = run(tmp);
    assert.equal(bad.status, 1);
    assert.equal(JSON.parse(bad.stdout).engine.matchesManifest, false);
    assert.notEqual(JSON.parse(bad.stdout).engine.wasm.sha256, JSON.parse(bad.stdout).engine.wasm.manifestSha256);
    fs.copyFileSync(path.join(engineDir, 'slam_refact_engine.wasm'), path.join(tmp, 'slam_refact_engine.wasm'));
    fs.chmodSync(path.join(tmp, 'slam_refact_engine.wasm'), 0o644);
    // build-time source record: a synthetic build provenance names one unchanged file, one file whose sha differs, one that is gone
    const prov = { source_files: { 'CMakeLists.txt': sha256Of(path.join(repo, 'slam_refact/engine/CMakeLists.txt')), 'src/CMakeLists.txt': '0'.repeat(64), 'src/gone.cc': '1'.repeat(64) },
      source_manifest_sha256: '2'.repeat(64) };
    const provFile = path.join(tmp, 'build-provenance.json');
    fs.writeFileSync(provFile, JSON.stringify(prov));
    const manifest = JSON.parse(fs.readFileSync(path.join(tmp, 'manifest.json'), 'utf8'));
    manifest.build = { ...manifest.build, provenance: { file: path.relative(repo, provFile), sha256: sha256Of(provFile) } };
    fs.writeFileSync(path.join(tmp, 'manifest.json'), JSON.stringify(manifest));
    const withProv = run(tmp);
    assert.equal(withProv.status, 0, withProv.stderr);
    const q = JSON.parse(withProv.stdout).engineSources;
    assert.equal(q.atBuild.sourceManifestSha256, '2'.repeat(64));
    assert.equal(q.atBuild.manifestNamesThisProvenance, true);
    assert.equal(q.atBuild.files, 3);
    assert.deepEqual(q.buildScope.changedSinceBuild, ['src/CMakeLists.txt']);
    assert.deepEqual(q.buildScope.removedSinceBuild, ['src/gone.cc']);
    assert.ok(q.buildScope.addedSinceBuild.length > 100 && !q.buildScope.addedSinceBuild.includes('CMakeLists.txt'));
    assert.equal(q.buildScope.clean, false);
    // a provenance file that is not the one the manifest names is said so
    fs.writeFileSync(provFile, JSON.stringify({ ...prov, source_manifest_sha256: '3'.repeat(64) }));
    assert.equal(JSON.parse(run(tmp).stdout).engineSources.atBuild.manifestNamesThisProvenance, false);
  } finally {
    fs.rmSync(tmp, { recursive: true, force: true });
  }
});

// ---- G-B1: optional DeviceMotion sensors ----
const sentinelEvent = () => ({ timestampNanos: 123456789, eventTimestampNanos: 5, interval: 11,
  acceleration: { x: 1, y: 2, z: 3 }, accelerationIncludingGravity: { x: 4, y: 5, z: 6 }, rotationRate: { alpha: 7, beta: 8, gamma: 9 } });

test('RM4C-B1a: pushMotion hands the sensors of the event to pushDeviceMotionEvent as they are, whatever is missing', async () => {
  const { pushMotion } = await import(pathToFileURL(path.join(chunkDir, 'sensors.js')).href);
  const calls = [];
  const glue = { pushDeviceMotionEvent: (...a) => calls.push(a), pushDeviceMotion: () => assert.fail('the fixed-signature push must not be used') };
  const base = sentinelEvent();
  const variants = {
    all: {}, noAcceleration: { acceleration: null }, noGravity: { accelerationIncludingGravity: null }, noRotation: { rotationRate: null },
    onlyAcceleration: { accelerationIncludingGravity: null, rotationRate: null }, none: { acceleration: undefined, accelerationIncludingGravity: undefined, rotationRate: undefined },
  };
  for (const [name, patch] of Object.entries(variants)) {
    calls.length = 0;
    const e = { ...sentinelEvent(), ...patch };
    assert.equal(pushMotion(glue, e), true, name);
    assert.equal(calls.length, 1, `${name}: one push`);
    const [t, arg] = calls[0];
    assert.equal(t, 123456789, `${name}: receive time`);
    assert.deepEqual(Object.keys(arg).sort(), ['acceleration', 'accelerationIncludingGravity', 'interval', 'rotationRate'], `${name}: keys`);
    assert.equal(arg.acceleration, e.acceleration, `${name}: acceleration is the acceleration`);
    assert.equal(arg.accelerationIncludingGravity, e.accelerationIncludingGravity, `${name}: gravity is the gravity`);
    assert.equal(arg.rotationRate, e.rotationRate, `${name}: rotationRate is the rotationRate (alpha/beta/gamma untouched)`);
    assert.equal(arg.interval, 11, `${name}: interval`);
  }
  // values stay where they are: a swap of two sensors is visible through the sentinel numbers
  calls.length = 0;
  pushMotion(glue, base);
  assert.deepEqual(calls[0][1].acceleration, { x: 1, y: 2, z: 3 });
  assert.deepEqual(calls[0][1].accelerationIncludingGravity, { x: 4, y: 5, z: 6 });
  assert.deepEqual(calls[0][1].rotationRate, { alpha: 7, beta: 8, gamma: 9 });
});

test('RM4C-B1b: a device without gyroscope (rotationRate null) still delivers its accelerometer through the real engine', async () => {
  const s = await attachedSession('rm4c-b1b');
  const diag = globalThis.__slamRefactChunk;
  const noGyro = (t) => ({ type: 'devicemotion', acceleration: { x: 0.1, y: 0, z: 0 }, accelerationIncludingGravity: { x: 0, y: 9.81, z: 0 }, rotationRate: null, timeStamp: t, interval: 16.666 });
  const noAccel = (t) => ({ type: 'devicemotion', acceleration: null, accelerationIncludingGravity: null, rotationRate: { alpha: 1, beta: 2, gamma: 3 }, timeStamp: t, interval: 16.666 });
  let dispatched = 0;
  for (let k = 0; k < 8; k++) {
    for (let j = 0; j < 2; j++) { globalThis.dispatchEvent(noGyro(2000 + k * 33 + j)); globalThis.dispatchEvent(noAccel(2000 + k * 33 + j + 0.5)); dispatched += 2; }
    xrFrame(s);
  }
  const pushes = s.glues[0].calls.filter(([n]) => n === 'pushDeviceMotionEvent');
  const legacy = s.glues[0].calls.filter(([n]) => n === 'pushDeviceMotion');
  assert.equal(legacy.length, 0, 'the all-fields-required push is not used');
  assert.equal(diag.stats.motionEventsSkipped, 0, 'no event is skipped for missing sensors');
  assert.equal(pushes.length, diag.stats.motionEventsPushed);
  assert.ok(pushes.length >= dispatched - 4, `pushed ${pushes.length} of ${dispatched} dispatched (the last frame\'s are still queued)`);
  assert.ok(pushes.some(([, , e]) => e.rotationRate === null && e.accelerationIncludingGravity), 'an accelerometer-only event reached the engine');
  assert.ok(pushes.some(([, , e]) => e.rotationRate && e.accelerationIncludingGravity === null), 'a gyroscope-only event reached the engine');
  assert.deepEqual(s.exceptions, []);
  assert.deepEqual(s.errors, []);
  detach(s);
});

test('RM4C-B1c: an event whose interval would abort the engine is skipped and counted; NaN and the largest valid interval are not', async () => {
  const s = await attachedSession('rm4c-b1c');
  const diag = globalThis.__slamRefactChunk;
  const ev = (interval, t) => ({ type: 'devicemotion', acceleration: null, accelerationIncludingGravity: { x: 0, y: 9.81, z: 0 }, rotationRate: { alpha: 0, beta: 0, gamma: 0 }, timeStamp: t, interval });
  const warns = []; const warn = console.warn; console.warn = (...a) => warns.push(a.join(' '));
  try {
    for (const [i, interval] of [Infinity, -Infinity, 1e14, 1.8e13, NaN, 16.666].entries()) globalThis.dispatchEvent(ev(interval, 3000 + i));
    xrFrame(s);  // stages the six events
    xrFrame(s);  // the staged frame is processed: pushes happen here
  } finally {
    console.warn = warn;
  }
  assert.equal(diag.stats.motionEventsSkipped, 3, 'Infinity, -Infinity and 1e14 ms (1e20 ns > 2^64)');
  assert.equal(warns.filter((w) => /interval outside the 64-bit/.test(w)).length, 1, 'one warning');
  assert.equal(s.glues[0].calls.filter(([n]) => n === 'pushDeviceMotionEvent').length - 3 /* frame events */ * 2 >= 0, true);
  assert.ok(diag.stats.motionEventsPushed >= 3);
  assert.deepEqual(s.exceptions, [], 'the engine did not abort');
  assert.deepEqual(s.errors, []);
  detach(s);
});

// ---- G-B2: startup recenter count continues across engineInit again ----
test('RM4C-B2a: a rebuilt glue gets the startup recenter count the old one had left; a new run starts with 5', async () => {
  // after the window (count 0)
  const s = await attachedSession('rm4c-b2a');
  runFrames(s, 9);
  assert.equal(s.glues[0].real.getFramesUntilRecenter(), 0);
  resize(s);
  assert.equal(s.glues[1].config.framesUntilRecenter, 0, 'size change after the startup window: 0');
  // inside the window
  const t = await attachedSession('rm4c-b2a-mid');
  runFrames(t, 3);
  const left = t.glues[0].real.getFramesUntilRecenter();
  assert.ok(left > 0 && left < 5, `count left ${left}`);
  xrFrame(t, { w: W2, h: H2, before: () => t.pm.onDeviceOrientationChange(sizeChangeArgs(t, W2, H2)) });
  assert.equal(t.glues[1].config.framesUntilRecenter, left, 'orientation change inside the window keeps the count');
  detach(t);
  // through an inactive episode and a failed rebuild
  const warn = console.warn; console.warn = () => {};
  try {
    xrFrame(s, { w: W, h: H, before: () => s.pm.onDeviceOrientationChange(sizeChangeArgs(s, W, H, UNSUPPORTED_ROTATION)) });
    runFrames(s, 2, { w: H, h: W });
    xrFrame(s, { w: H, h: W, before: () => s.pm.onDeviceOrientationChange(sizeChangeArgs(s, H, W, UNSUPPORTED_ROTATION)) });
    xrFrame(s, { w: W, h: H, before: () => s.pm.onVideoSizeChange(sizeChangeArgs(s, W, H, 0)) });
  } finally {
    console.warn = warn;
  }
  assert.equal(s.glues.at(-1).config.framesUntilRecenter, 0, 'the last known count survives the inactive episode');
  // a new run (detach, attach): the singleton is gone, the default 5 (no option)
  detach(s);
  const before = s.glues.length;
  s.pm.onBeforeSessionInitialize({ sessionAttributes: { fillsCameraTexture: true, controlsCamera: false } });
  s.pm.onStart({ computeCtx: s.fake.gl, canvasWidth: W, canvasHeight: H });
  s.pm.onSessionAttach({ videoWidth: W, videoHeight: H, canvasWidth: W, canvasHeight: H, rotation: 0, computeCtx: s.fake.gl,
    framework: s.framework, GLctx: fakeGl().gl, sessionAttributes: { fillsCameraTexture: true } });
  assert.equal(s.glues.length, before + 1);
  assert.equal('framesUntilRecenter' in s.glues.at(-1).config, false, 'a fresh engineInit leaves the default (5)');
  assert.equal(s.glues.at(-1).real.getFramesUntilRecenter(), 5);
  detach(s);
});

test('RM4C-B2b: after a size change the textures of the responses advance by exactly one frame each (no repeat, no skip)', async () => {
  const s = await attachedSession('rm4c-b2b');
  runFrames(s, 9);
  const results = [resize(s), ...runFrames(s, 14, { w: W2, h: H2 })];
  assert.deepEqual(s.exceptions, []);
  const names = results.slice(2).map((r) => r.realityTexture.drawFor.name);
  assert.equal(names.length, 13);
  names.slice(1).forEach((n, i) => assert.equal(n - names[i], 1, `response ${i + 3}: ${names[i]} -> ${n}`));
  detach(s);
});

// ---- G-B3: image targets (node: real engine glue, injected image loader and detections) ----
const IT_W = 120, IT_H = 160;
function itLuminance(seed) {
  const img = new Uint8Array(IT_W * IT_H);
  let s = seed >>> 0 || 1;
  const next = () => { s ^= s << 13; s >>>= 0; s ^= s >>> 17; s ^= s << 5; s >>>= 0; return s; };
  for (let i = 0; i < img.length; i++) img[i] = 80 + ((i % IT_W) >> 2);
  for (let k = 0; k < 220; k++) {
    const x0 = next() % IT_W, y0 = next() % IT_H, w = 3 + (next() % 14), h = 3 + (next() % 14), v = next() % 256;
    for (let y = y0; y < Math.min(IT_H, y0 + h); y++) for (let x = x0; x < Math.min(IT_W, x0 + w); x++) img[y * IT_W + x] = v;
  }
  return img;
}
const itProps = (extra = {}) => ({ left: 0, top: 0, width: IT_W, height: IT_H, isRotated: false, originalWidth: IT_W, originalHeight: IT_H,
  cylinderCircumferenceTop: 100, cylinderCircumferenceBottom: 100, cylinderSideLength: 50, targetCircumferenceTop: 50, ...extra });
const itTarget = (name, extra = {}) => ({ type: 'PLANAR', name, imagePath: `image-targets/${name}_luminance.png`, metadata: JSON.stringify({ tag: name }),
  userMetadataIsJson: true, properties: itProps(extra.properties), ...Object.fromEntries(Object.entries(extra).filter(([k]) => k !== 'properties')) });
// A curved target whose curved metadata is incomplete (cylinderSideLength missing): the engine takes curved targets (US-014) but
// refuses this one - ParseImageTargetJson: "properties.cylinderSideLength must be a finite number".
const itRefusedCurvy = (name) => itTarget(name, { type: 'CYLINDER', properties: { cylinderSideLength: undefined } });
const REFUSED_CURVY = (name) => new RegExp(`"${name}" was rejected by the engine.*cylinderSideLength must be a finite number`);
const itLoader = (order = [], failFor = new Set()) => async (url, target) => {
  order.push(target.name);
  await new Promise((r) => setTimeout(r, 1));
  if (failFor.has(target.name)) throw new Error(`image failed to load: ${url}`);
  return { width: IT_W, height: IT_H, luminance: itLuminance(target.name.charCodeAt(1) + 7) };
};
const detImage = (name, x, over = {}) => ({ name, id: -1, type: 'FLAT', position: { x, y: 0.5, z: -1 }, rotation: { x: 0, y: 0, z: 0, w: 1 },
  widthInMeters: 0.3, heightInMeters: 0.4, scale: 0.4, trackingStatus: 0, ...over });
const imageEventsOf = (s) => s.events.filter(([n]) => /^image/.test(n));

// attachBudgetMs: Infinity keeps the rebuild replay of the loaded targets one engine call (the RM-4c behaviour these tests were
// written for); the RM4D tests below set the budget explicitly.
async function itSession(tag, { targets = [itTarget('tA'), itTarget('tB')], hooks, loaderOrder, failFor, preConfigure = true, chunkOptions = {} } = {}) {
  const s = await attachedSession(tag, { hooks, chunkOptions: { loadLuminance: itLoader(loaderOrder, failFor), attachBudgetMs: Infinity, ...chunkOptions } });
  if (preConfigure) s.ctrl.configure({ imageTargetData: targets });
  s.pm.onAppResourcesLoaded({ imageTargets: undefined, framework: s.framework });
  await globalThis.__slamRefactChunk.imageTargets.whenIdle();
  return s;
}

test('RM4C-B3a: configure({imageTargetData}) validates like the original; 33 targets and a session without camera texture are refused', async () => {
  const s = await attachedSession('rm4c-b3a', { chunkOptions: { loadLuminance: itLoader() } });
  const diag = globalThis.__slamRefactChunk;
  assert.throws(() => s.ctrl.configure({ imageTargetData: 'x' }), { message: '[XR] Expected array for arg: imageTargetData' });
  const many = Array.from({ length: 33 }, (_, i) => itTarget(`t${i}`));
  assert.throws(() => s.ctrl.configure({ imageTargetData: many }), { message: '[XR] Too many active image targets: 33 > 32' });
  s.ctrl.configure({ imageTargetData: Array.from({ length: 32 }, (_, i) => itTarget(`t${i}`)) });  // 32 is fine (loading is not started before onAppResourcesLoaded)
  assert.equal(diag.imageTargets.loadedNames().length, 0);
  assert.ok(!diag.warnings.some((w) => /configure options not supported/.test(w)), 'imageTargetData is a supported option now');
  // the refused 33-array did not replace the configured data: the 32 are what loads
  s.ctrl.configure({ imageTargetData: [itTarget('keep')] });
  assert.throws(() => s.ctrl.configure({ imageTargetData: many }), /Too many/);
  s.pm.onAppResourcesLoaded({ imageTargets: undefined, framework: s.framework });
  await diag.imageTargets.whenIdle();
  assert.deepEqual(diag.imageTargets.loadedNames(), ['keep'], 'state unchanged by the refused call');
  detach(s);
  // deprecated imageTargets
  const warns = []; const w0 = console.warn; console.warn = (...a) => warns.push(a.join(' '));
  try { s.ctrl.configure({ imageTargets: ['a'] }); } finally { console.warn = w0; }
  assert.deepEqual(warns, ['[XR] imageTargets is deprecated, please use imageTargetData instead.']);
  // camera-less session
  const { ctrl } = await makeController('rm4c-b3a2');
  const pm = ctrl.pipelineModule();
  ctrl.configure({ imageTargetData: [itTarget('tA')] });
  assert.throws(() => pm.onBeforeSessionInitialize({ sessionAttributes: { fillsCameraTexture: false, controlsCamera: false } }),
    { message: '[XR] Image Targets require a session that can provide a camera texture' });
  const { ctrl: c3 } = await makeController('rm4c-b3a3');
  const p3 = c3.pipelineModule();
  p3.onBeforeSessionInitialize({ sessionAttributes: { fillsCameraTexture: false, controlsCamera: false } });
  const warns2 = []; console.warn = (...a) => warns2.push(a.join(' '));
  try { c3.configure({ imageTargetData: [itTarget('tA')] }); } finally { console.warn = w0; }
  assert.deepEqual(warns2, ['[XR] Image Targets are not supported in the current session.']);
});

test('RM4C-B3b: loading queue, imageloading / imagescanning, metadata and geometry, engine calls, unload', async () => {
  const order = [];
  const tA = itTarget('tA'), tB = itTarget('tB', { properties: { isRotated: true, width: 90, height: 120 } });
  const s = await itSession('rm4c-b3b', { targets: [tA, tB], loaderOrder: order });
  const ev = imageEventsOf(s);
  assert.deepEqual(ev.map(([n]) => n), ['imageloading', 'imagescanning']);
  assert.deepEqual(order, ['tA', 'tB'], 'loaded in the order of the list');
  const loading = ev[0][1].imageTargets;
  assert.deepEqual(loading.map((t) => t.name), ['tA', 'tB']);
  assert.deepEqual(loading[0], { name: 'tA', type: 'FLAT', metadata: { tag: 'tA' }, properties: { ...itProps() } }, 'metadata parsed (userMetadataIsJson), type apiName');
  const scanning = ev[1][1].imageTargets;
  assert.deepEqual(scanning[0].geometry, { scaledWidth: IT_W / IT_H, scaledHeight: 1 });
  assert.deepEqual(scanning[1].geometry, { scaledWidth: 1, scaledHeight: 90 / 120 }, 'isRotated swaps width / height');
  // the engine was given exactly what was configured, cumulative
  const sets = s.glues[0].calls.filter(([n]) => n === 'setImageTargets');
  assert.equal(sets.length, 2);
  assert.deepEqual(sets[0][1].map((x) => x.json), [JSON.stringify(tA)]);
  assert.deepEqual(sets[1][1].map((x) => x.json), [JSON.stringify(tA), JSON.stringify(tB)]);
  assert.deepEqual([sets[1][1][0].width, sets[1][1][0].height, sets[1][1][0].luminance.length], [IT_W, IT_H, IT_W * IT_H]);
  assert.deepEqual(Array.from(sets[1][1][0].luminance), Array.from(itLuminance('tA'.charCodeAt(1) + 7)), 'the luminance of the loaded image, untouched');
  assert.deepEqual(s.glues[0].real.getImageTargetNames().sort(), ['tA', 'tB']);
  // the same imagePath is not loaded again; a target that is no longer listed is unloaded from the engine
  s.events.length = 0; order.length = 0;
  s.ctrl.configure({ imageTargetData: [tA] });
  await globalThis.__slamRefactChunk.imageTargets.whenIdle();
  assert.deepEqual(order, [], 'nothing to load');
  assert.deepEqual(imageEventsOf(s), [], 'no imageloading when nothing is new');
  const unloads = s.glues[0].calls.filter(([n]) => n === 'unloadImageTargets');
  assert.deepEqual(unloads.map(([, names]) => names), [['tB']]);
  assert.deepEqual(s.glues[0].real.getImageTargetNames(), ['tA']);
  assert.deepEqual(globalThis.__slamRefactChunk.imageTargets.loadedNames(), ['tA']);
  detach(s);
});

test('RM4C-B3c: found / updated / lost are dispatched exactly once per change; engine imageEvents are ignored; unknown names are skipped', async () => {
  let call = 0;
  const script = (n) => {  // n = 1-based processFramePyramid call
    if (n < 9) return [];
    if (n <= 12) return [detImage('tA', 1)];
    if (n <= 14) return [detImage('tA', 1), detImage('ghost', 5)];  // unchanged A, unknown name
    if (n === 15) return [detImage('tA', 2)];
    if (n === 16) return [detImage('tA', 2), detImage('tB', 0)];
    return [];
  };
  const hooks = { override: { processFramePyramid: (rec, ...args) => {
    const real = rec.real.processFramePyramid(...args);
    const n = ++call;
    // the engine's own C++ tracker would report these; they must not reach the app a second time
    return { ...real, repeated: false, detectedImages: script(n),
      imageEvents: n === 9 ? [{ name: 'imagefound', detail: detImage('tA', 1) }] : n === 17 ? [{ name: 'imagelost', detail: detImage('tA', 2) }] : [] };
  } } };
  const s = await itSession('rm4c-b3c', { hooks });
  const results = runFrames(s, 22);
  assert.deepEqual(s.exceptions, []);
  assert.deepEqual(s.errors, []);
  const ev = imageEventsOf(s).filter(([n]) => /^image(found|updated|lost)$/.test(n));
  assert.deepEqual(ev.map(([n, d]) => [n, d.name, d.position?.x]), [
    ['imagefound', 'tA', 1], ['imageupdated', 'tA', 2], ['imagefound', 'tB', 0], ['imagelost', 'tA', 2], ['imagelost', 'tB', 0]]);
  // independent recomputation from the answers the app saw: one event per change, none missing
  const seen = new Map(); const expected = [];
  for (const r of results.filter(Boolean)) {
    const names = new Set();
    for (const img of r.detectedImages) {
      names.add(img.name);
      const old = seen.get(img.name);
      if (!old) expected.push(['imagefound', img.name]); else if (JSON.stringify(old) !== JSON.stringify(img)) expected.push(['imageupdated', img.name]);
      seen.set(img.name, img);
    }
    for (const k of [...seen.keys()]) if (!names.has(k)) { expected.push(['imagelost', k]); seen.delete(k); }
  }
  assert.deepEqual(ev.map(([n, d]) => [n, d.name]), expected);
  assert.ok(results.filter(Boolean).every((r) => r.detectedImages.every((i) => i.name !== 'ghost')), 'no detection without metadata');
  const found = ev[0][1];
  assert.deepEqual(Object.keys(found), ['name', 'metadata', 'position', 'rotation', 'scale', 'type', 'properties', 'scaledWidth', 'scaledHeight']);
  assert.deepEqual(found.metadata, { tag: 'tA' });
  assert.equal(found.type, 'FLAT');
  assert.equal(found.scale, 0.4);
  assert.deepEqual(found.properties, { width: IT_W, height: IT_H, top: 0, left: 0, originalWidth: IT_W, originalHeight: IT_H, isRotated: false });
  assert.ok(globalThis.__slamRefactChunk.stats.engineImageEventsIgnored >= 2, 'the engine events were counted and dropped');
  detach(s);
});

test('RM4C-B3d: a rebuilt session gets the loaded targets again without a second imagefound; inactivity reports imagelost; detach clears the state', async () => {
  const hooks = { override: { processFramePyramid: (rec, ...args) => {
    const real = rec.real.processFramePyramid(...args);
    return { ...real, detectedImages: [detImage('tA', 1)] };
  } } };
  const s = await itSession('rm4c-b3d', { hooks });
  runFrames(s, 8);
  assert.equal(imageEventsOf(s).filter(([n]) => n === 'imagefound').length, 1);
  resize(s);
  runFrames(s, 8, { w: W2, h: H2 });
  const sets = s.glues[1].calls.filter(([n]) => n === 'setImageTargets');
  assert.equal(sets.length, 1, 'the new glue was given the loaded targets');
  assert.deepEqual(sets[0][1].map((x) => x.json), [JSON.stringify(itTarget('tA')), JSON.stringify(itTarget('tB'))]);
  assert.deepEqual(s.glues[1].real.getImageTargetNames().sort(), ['tA', 'tB']);
  assert.equal(imageEventsOf(s).filter(([n]) => n === 'imagefound').length, 1, 'no second imagefound after the rebuild');
  assert.equal(imageEventsOf(s).filter(([n]) => n === 'imagelost').length, 0);
  // inactive capture: the engine does not run, the target is not seen
  const warn = console.warn; console.warn = () => {};
  try {
    xrFrame(s, { w: W2, h: H2, before: () => s.pm.onDeviceOrientationChange(sizeChangeArgs(s, W2, H2, UNSUPPORTED_ROTATION)) });
    runFrames(s, 3, { w: H2, h: W2 });
  } finally { console.warn = warn; }
  assert.equal(imageEventsOf(s).filter(([n]) => n === 'imagelost').length, 1, 'imagelost on entering the inactive state');
  xrFrame(s, { w: H2, h: W2, before: () => s.pm.onDeviceOrientationChange(sizeChangeArgs(s, H2, W2, UNSUPPORTED_ROTATION)) });
  xrFrame(s, { w: W2, h: H2, before: () => s.pm.onVideoSizeChange(sizeChangeArgs(s, W2, H2, 0)) });
  runFrames(s, 8, { w: W2, h: H2 });
  assert.equal(imageEventsOf(s).filter(([n]) => n === 'imagefound').length, 2, 'found again after the way back');
  detach(s);
  assert.deepEqual(globalThis.__slamRefactChunk.imageTargets.loadedNames(), [], 'onDetach forgets the loaded targets (the original clears currentTargetUrls_)');
});

test('RM4C-B3e: an image that cannot be loaded and a target the engine refuses are reported, the others keep loading, imagescanning still fires once', async () => {
  const errors = []; const e0 = console.error; console.error = (...a) => errors.push(a.join(' '));
  let s;
  try {
    const bad = itTarget('tBad'), curvy = itRefusedCurvy('tCurvy');
    s = await itSession('rm4c-b3e', { targets: [itTarget('tA'), bad, curvy, itTarget('tB')], failFor: new Set(['tBad']) });
  } finally { console.error = e0; }
  const diag = globalThis.__slamRefactChunk;
  const ev = imageEventsOf(s);
  assert.deepEqual(ev.map(([n]) => n), ['imageloading', 'imagescanning'], 'one scanning event when the queue is empty');
  assert.deepEqual(diag.imageTargets.loadedNames().sort(), ['tA', 'tB']);
  assert.equal(diag.imageTargets.loadErrors, 2);
  assert.ok(errors.some((m) => /"tBad" failed to load/.test(m)), errors.join('|'));
  assert.ok(errors.some((m) => REFUSED_CURVY('tCurvy').test(m)), errors.join('|'));
  assert.deepEqual(ev[1][1].imageTargets.map((t) => t.name).sort(), ['tA', 'tB'], 'failed targets have no metadata');
  assert.deepEqual(s.errors, [], 'errorCallback is not used (it would stop the session in XRExtras apps)');
  assert.deepEqual(s.glues[0].real.getImageTargetNames().sort(), ['tA', 'tB']);
  detach(s);
  // every target fails: the queue is empty after the failures, so the failure itself ends the scanning phase
  const e1 = console.error; console.error = () => {};
  let only;
  try { only = await itSession('rm4c-b3e2', { targets: [itTarget('tOnly')], failFor: new Set(['tOnly']) }); } finally { console.error = e1; }
  assert.deepEqual(imageEventsOf(only).map(([n]) => n), ['imageloading', 'imagescanning']);
  assert.deepEqual(imageEventsOf(only)[1][1].imageTargets, [], 'no metadata of a target that did not load');
  assert.deepEqual(globalThis.__slamRefactChunk.imageTargets.loadedNames(), []);
  detach(only);
});

test('RM4C-B3f: a new configure({imageTargetData}) batch stops the loads of the previous one', async () => {
  const order = [];
  const s = await attachedSession('rm4c-b3f', { chunkOptions: { loadLuminance: itLoader(order) } });
  const diag = globalThis.__slamRefactChunk;
  s.ctrl.configure({ imageTargetData: [itTarget('tA'), itTarget('tB')] });
  s.pm.onAppResourcesLoaded({ imageTargets: undefined, framework: s.framework });  // batch 1 starts loading
  s.ctrl.configure({ imageTargetData: [itTarget('tC')] });                           // batch 2 before batch 1 finished
  await diag.imageTargets.whenIdle();
  assert.deepEqual(diag.imageTargets.loadedNames(), ['tC'], 'nothing of the stopped batch was given to the engine');
  assert.deepEqual(imageEventsOf(s).map(([n]) => n), ['imageloading', 'imageloading', 'imagescanning'], 'one scanning event, after the last batch');
  assert.deepEqual(imageEventsOf(s)[1][1].imageTargets.map((t) => t.name), ['tC']);
  assert.deepEqual(s.glues[0].real.getImageTargetNames(), ['tC']);
  detach(s);
});

// ---- G-B4: ROI draw (fake GL, real engine layout) ----
function namedFakeGl(opts) {
  const f = fakeGl(opts);
  const gl = new Proxy(f.gl, { get(t, prop) {
    if (prop === 'getUniformLocation') return (prog, name) => ({ uniform: name });
    return t[prop];
  } });
  return { ...f, gl };
}
function stage1Draws(fake) {
  const prog = fake.created.createProgram[1];  // input, stage1, blur, decimate, stage2, stage3 (Gr8ComputePyramid constructor order)
  const u = {}; let current = null, viewport = null;
  const draws = [];
  for (const [name, ...a] of fake.calls) {
    if (name === 'useProgram') current = a[0];
    else if (name === 'viewport') viewport = a;
    else if (name === 'uniform1i' && a[0]?.uniform) u[a[0].uniform] = a[1];
    else if (name === 'uniformMatrix4fv' && a[0]?.uniform) u[a[0].uniform] = Array.from(a[2]);
    else if (name === 'drawElements' && current === prog) draws.push({ viewport, mvp: u.mvp, clear: u.clear, isCurvy: u.isCurvy, rotation: u.rotation });
  }
  return draws;
}
const IDENT = [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1];
const warpOf = (k) => Float32Array.from(IDENT.map((v, i) => v + (i === 12 ? 0.01 * k : 0) + (i === 0 ? 0.1 * k : 0)));

async function layoutOf(tag, w = W, h = H) {
  const { ctrl } = await makeController(tag);
  void ctrl;
  const Module = (await import(pathToFileURL(path.join(chunkDir, 'engine-loader.js')).href));
  const { Module: M } = await Module.loadEngine(new URL('slam-refact-engine/', pathToFileURL(chunkDir + '/')));
  const glue = new M.SlamRefactGlue({ captureWidth: w, captureHeight: h, manufacturer: 'Google', model: 'Pixel 7' });
  const layout = glue.pyramidLayout();
  glue.delete();
  return layout;
}

test('RM4C-B4a: stage(texture, rois) draws clear + warp per ROI in list order, then the clear draw of the other slots', async () => {
  const layout = await layoutOf('rm4c-b4a');
  assert.equal(layout.roiSlots.length, 4);
  const { Gr8ComputePyramid } = await import(pathToFileURL(path.join(chunkDir, 'gr8-compute-pyramid.js')).href);
  const fake = namedFakeGl();
  const warns = [];
  const pyramid = new Gr8ComputePyramid(fake.gl, layout, { cameraWidth: W, cameraHeight: H, warn: (m) => warns.push(m) });
  pyramid.stage({ name: 1 });  // no ROIs: level 0 + four clear draws
  const none = stage1Draws(fake);
  assert.equal(none.length, 5);
  assert.deepEqual(none.slice(1).map((d) => [d.clear, d.mvp]), Array(4).fill([1, IDENT]));
  assert.deepEqual(none.slice(1).map((d) => d.viewport), layout.roiSlots.map((l) => [l.c, l.r, l.w, l.h]));
  fake.calls.length = 0;
  const rois = [{ slot: 2, source: 'IMAGE_TARGET', name: 'tA', warp: warpOf(1) }, { slot: 0, source: 'HIRES_SCAN', name: 'tA', warp: warpOf(2) }];
  pyramid.stage({ name: 2 }, rois);
  const d = stage1Draws(fake);
  // level 0, then ROI list order: slot 2 (clear, warp), slot 0 (clear, warp), then slots 1 and 3 (clear only)
  assert.equal(d.length, 1 + 2 * 2 + 2);
  const slotVp = (i) => [layout.roiSlots[i].c, layout.roiSlots[i].r, layout.roiSlots[i].w, layout.roiSlots[i].h];
  assert.deepEqual(d.slice(1).map((x) => x.viewport), [slotVp(2), slotVp(2), slotVp(0), slotVp(0), slotVp(1), slotVp(3)]);
  assert.deepEqual(d.slice(1).map((x) => x.clear), [1, 0, 1, 0, 1, 1]);
  assert.deepEqual(d.slice(1).map((x) => x.mvp), [IDENT, Array.from(rois[0].warp), IDENT, Array.from(rois[1].warp), IDENT, IDENT], 'the warp is the engine\'s array, column-major, untouched');
  assert.equal(d[2].isCurvy, 0);
  assert.equal(d[4].isCurvy, 0);
  assert.deepEqual(d.slice(1).map((x) => x.rotation), [0, 0, 0, 0, 0, 0].map((_, i) => (layout.roiSlots[[2, 2, 0, 0, 1, 3][i]].rotated ? -90 : 0)));
  // a curved ROI is not drawn (slot stays cleared) and says so once
  fake.calls.length = 0;
  pyramid.stage({ name: 3 }, [{ slot: 1, source: 'CURVY_IMAGE_TARGET', name: 'c', warp: warpOf(3) }]);
  pyramid.stage({ name: 4 }, [{ slot: 1, source: 'CURVY_IMAGE_TARGET', name: 'c', warp: warpOf(3) }]);
  const dc = stage1Draws(fake);
  assert.ok(dc.every((x) => x.clear === 1 || x === dc[0] || x === dc[5]), 'only clear draws besides the pyramid level');
  assert.equal(pyramid.stats.roiDraws, 2);
  assert.ok(warns.length >= 1 && warns.every((m) => /curved image target/.test(m)));
  pyramid.dispose();
});

test('RM4C-B4b: a stage with ROIs reads [0, roiAtlasRows); a stage without ROIs still reads [0, layout.rows)', async () => {
  const layout = await layoutOf('rm4c-b4b');
  assert.ok(layout.roiAtlasRows > layout.rows, `roiAtlasRows ${layout.roiAtlasRows} rows ${layout.rows}`);
  const { Gr8ComputePyramid } = await import(pathToFileURL(path.join(chunkDir, 'gr8-compute-pyramid.js')).href);
  const fake = namedFakeGl();
  const pyramid = new Gr8ComputePyramid(fake.gl, layout, { cameraWidth: W, cameraHeight: H });
  const PPB = fake.constant('PIXEL_PACK_BUFFER');
  const bytesFor = (rows) => layout.textureSize * rows * 4;
  const readsOf = (from) => fake.calls.slice(from).filter(([n, , , , , , , off]) => n === 'readPixels' && off === 0).map(([, x, y, w, h]) => [x, y, w, h]);
  let from = fake.calls.length;
  pyramid.stage({ name: 1 });
  const a0 = pyramid.read();
  assert.deepEqual(readsOf(from), [[0, 0, layout.textureSize, layout.rows]]);
  assert.equal(a0.byteLength, bytesFor(layout.rows));
  assert.equal(pyramid.lastReadRows, layout.rows);
  from = fake.calls.length;
  pyramid.stage({ name: 2 }, [{ slot: 0, source: 'IMAGE_TARGET', name: 't', warp: warpOf(1) }]);
  const a1 = pyramid.read();
  assert.deepEqual(readsOf(from), [[0, 0, layout.textureSize, layout.roiAtlasRows]]);
  assert.equal(a1.byteLength, bytesFor(layout.roiAtlasRows));
  assert.equal(pyramid.lastReadRows, layout.roiAtlasRows);
  const grew = fake.calls.slice(from).filter(([n, t, b]) => n === 'bufferData' && t === PPB && b instanceof Uint8Array);
  assert.deepEqual(grew.map(([, , b]) => b.byteLength), [bytesFor(layout.roiAtlasRows)], 'the pixel buffer grows once');
  assert.deepEqual(fake.calls.slice(from).filter(([n]) => n === 'getBufferSubData').map(([, , , dst]) => dst.byteLength), [bytesFor(layout.roiAtlasRows)]);
  from = fake.calls.length;
  pyramid.stage({ name: 3 });
  const a2 = pyramid.read();
  assert.deepEqual(readsOf(from), [[0, 0, layout.textureSize, layout.rows]], 'back to the small read');
  assert.equal(a2.byteLength, bytesFor(layout.rows));
  assert.equal(fake.calls.slice(from).filter(([n, t]) => n === 'bufferData' && t === PPB).length, 0, 'no buffer re-specification when it is big enough');
  assert.equal(pyramid.stats.readbackBytes, bytesFor(layout.rows));
  pyramid.dispose();
});

test('RM4C-B4c: through the chunk frames are staged and read with the ROI rows once the engine asks for ROIs; no targets = the old rows', async () => {
  const log = [];
  // RM-4e (review A-1): the ROIs the engine's list names at every stage (the ask the chunk makes before the draw) are summed here and compared,
  // at the end, with what the pyramid drew or skipped. A stage that hands the pyramid only part of the list (the reviewer's mutant MV6)
  // leaves the two totals apart.
  let askedRois = 0;
  const hooks = { override: {
    getRequestedRois: (rec) => { const list = rec.real.getRequestedRois(); askedRois += list.length; return list; },
    processFramePyramid: (rec, ...args) => {
      log.push({ asks: rec.real.getRequestedRois().length, bytes: args[2].byteLength, size: args[3], rows: args[4] });
      return rec.real.processFramePyramid(...args);
    } } };
  const s = await attachedSession('rm4c-b4c', { hooks, chunkOptions: { loadLuminance: itLoader() } });
  const diag = globalThis.__slamRefactChunk;
  const layout = s.glues[0].real.pyramidLayout();
  runFrames(s, 4);
  assert.ok(log.every((c) => c.asks === 0 && c.rows === layout.rows && c.bytes === layout.textureSize * layout.rows * 4), 'no targets: unchanged');
  assert.equal(diag.session.readback.roiStages, 0);
  // RM-4d (review RM-4c B-1): this assertion used to be `getRequestedRois` calls === 0 ("the engine is not asked for ROIs without
  // targets"), which pinned the defect: the engine's list is fixed by the last processed frame and outlives the targets. The engine is
  // asked on every staged frame now, and without targets its answer is empty, so rows / bytes stay the old ones (checked above).
  assert.ok(s.glues[0].calls.filter(([n]) => n === 'getRequestedRois').length >= 4, 'the engine is asked for ROIs on every staged frame, with or without targets');
  s.ctrl.configure({ imageTargetData: [itTarget('tA')] });
  s.pm.onAppResourcesLoaded({ imageTargets: undefined, framework: s.framework });
  await diag.imageTargets.whenIdle();
  let waited = 0;
  while (s.glues[0].real.getRequestedRois().length === 0 && waited < 30) { xrFrame(s); waited++; }
  assert.ok(s.glues[0].real.getRequestedRois().length > 0, `the engine asks for ROIs after the target was loaded (waited ${waited} frames)`);
  const from = log.length;
  runFrames(s, 8);
  const later = log.slice(from);
  assert.equal(later.length, 8);
  assert.ok(later.filter((c) => c.asks > 0).length >= 5, `the engine asked for ROIs in ${later.filter((c) => c.asks > 0).length} of 8 processed frames`);
  for (const c of later) {
    if (c.asks > 0) assert.deepEqual([c.rows, c.bytes], [layout.roiAtlasRows, layout.textureSize * layout.roiAtlasRows * 4], 'ROI rows whenever the engine wants ROIs');
    else assert.equal(c.rows <= layout.roiAtlasRows && c.rows >= layout.rows, true);
  }
  assert.equal(diag.stats.roiPaddedFrames, 0, 'staged with the ROI rows, nothing to pad');
  assert.ok(diag.session.readback.roiStages >= 5 && diag.session.readback.roiDraws >= 5, JSON.stringify(diag.session.readback));
  assert.ok(askedRois >= 5, `the engine named ${askedRois} ROIs over the staged frames`);
  assert.equal(diag.session.readback.roiDraws + diag.session.readback.roiSkipped, askedRois,
    'every ROI of the engine\'s list at every stage reached the pyramid (drawn or skipped by its own rules): none was dropped on the way');
  assert.deepEqual(s.exceptions, []);
  assert.deepEqual(s.errors, []);
  detach(s);
});

test('RM4C-B4d: slots the engine asks for after a frame was staged are zero-padded (the original cleared them), not an error', async () => {
  let asks = 0;
  const fakeRois = (rec) => rec.real.pyramidLayout().roiSlots.slice(0, 2).map((l, i) => ({ slot: i, source: 'IMAGE_TARGET', name: 'tA', warp: warpOf(i + 1) }));
  // getRequestedRois alternates: the stage-time call sees no ROI, the process-time call sees two
  const hooks = { override: { getRequestedRois: (rec) => (++asks % 2 === 1 ? [] : fakeRois(rec)) } };
  // RM-4d (review RM-4c m2, mutant K8): the fake readback is filled with 7, so the pyramid rows are non-zero and a padding that
  // drops them (`padded.set(atlas)` removed) is seen; a node atlas of zeros hid that.
  const s = await attachedSession('rm4c-b4d', { hooks, chunkOptions: { loadLuminance: itLoader() }, glOpts: { fillReadback: 7 } });
  const diag = globalThis.__slamRefactChunk;
  const layout = s.glues[0].real.pyramidLayout();
  s.ctrl.configure({ imageTargetData: [itTarget('tA')] });
  s.pm.onAppResourcesLoaded({ imageTargets: undefined, framework: s.framework });
  await diag.imageTargets.whenIdle();
  asks = 0;
  runFrames(s, 6);
  const processed = s.glues[0].calls.filter(([n]) => n === 'processFramePyramid').slice(-4);
  assert.ok(processed.length === 4);
  for (const [, , , atlas, size, rows] of processed) {
    assert.equal(rows, layout.roiAtlasRows);
    assert.equal(atlas.byteLength, layout.textureSize * layout.roiAtlasRows * 4);
    assert.ok(atlas.subarray(layout.textureSize * layout.rows * 4).every((v) => v === 0), 'the padded rows are zero');
    assert.ok(atlas.subarray(0, layout.textureSize * layout.rows * 4).every((v) => v === 7), 'the pyramid rows are kept');
  }
  assert.ok(diag.stats.roiPaddedFrames >= 4);
  assert.deepEqual(s.exceptions, []);
  assert.deepEqual(s.errors, []);
  detach(s);
});

// =====================================================================================================
// RM-4d (review RM-4c: B-1 BLOCKER, M-1, M-2 and the MINORs m1-m5; gate build/slam-refact-full-20261019/preregistration/rm4c-chunk-gate.v3.md)
// =====================================================================================================

// One xr.js frame where app code (onUpdate, an imagefound / imagelost listener, render) runs between onProcessCpu and onProcessGpu.
function splitFrame(s, between, { w = W, h = H } = {}) {
  const k = s.k++;
  const fsr = { computeTexture: { name: 100 + k }, cameraTexture: { cam: 100 + k }, textureWidth: w, textureHeight: h, orientation: 0,
    repeatFrame: false, videoTime: 1 + k / 30, frameTime: 1 + k / 30 };
  let result;
  try { result = s.pm.onProcessCpu({ framework: s.framework, frameStartResult: fsr }); } catch (e) { s.exceptions.push(e); }
  if (between) between();
  try { s.pm.onProcessGpu({ framework: s.framework, frameStartResult: fsr }); } catch (e) { s.exceptions.push(e); }
  return result;
}

// Session with target tA loaded and the engine asking for ROIs; `rows` logs the atlas rows of every processFramePyramid call.
async function roiSession(tag, { loader = itLoader(), extraOptions = {} } = {}) {
  const rows = [];
  const hooks = { override: { processFramePyramid: (rec, ...args) => { rows.push(args[4]); return rec.real.processFramePyramid(...args); } } };
  const s = await attachedSession(tag, { hooks, chunkOptions: { loadLuminance: loader, attachBudgetMs: Infinity, ...extraOptions } });
  s.rows = rows;
  s.ctrl.configure({ imageTargetData: [itTarget('tA')] });
  s.pm.onAppResourcesLoaded({ imageTargets: undefined, framework: s.framework });
  await globalThis.__slamRefactChunk.imageTargets.whenIdle();
  let waited = 0;
  while (s.glues[0].real.getRequestedRois().length === 0 && waited < 30) { xrFrame(s); waited++; }
  assert.ok(s.glues[0].real.getRequestedRois().length > 0, `the engine asks for ROIs after the target was loaded (waited ${waited})`);
  return s;
}

test('RM4D-A1a (BLOCKER B-1): clearing every image target between onProcessCpu and onProcessGpu does not kill the session', async () => {
  const s = await roiSession('rm4d-a1a');
  const diag = globalThis.__slamRefactChunk;
  const layout = s.glues[0].real.pyramidLayout();
  assert.ok(runFrames(s, 3).every(Boolean));
  let askedAtClear = -1;
  const first = splitFrame(s, () => {
    s.ctrl.configure({ imageTargetData: [] });
    askedAtClear = s.glues[0].real.getRequestedRois().length;
  });
  assert.ok(first, 'the frame the app cleared the targets in was answered');
  assert.deepEqual(diag.imageTargets.loadedNames(), [], 'the targets are gone from the JS side');
  assert.ok(askedAtClear > 0, 'the scenario is real: the engine still holds ROIs for the next draw after the targets were cleared (its list is fixed by the last processed frame)');
  const fromRow = s.rows.length;
  const post = runFrames(s, 6);
  assert.ok(post.every(Boolean), `no frame lost: ${post.map(Boolean)}`);
  assert.equal(diag.stats.processErrors, 0, 'processFramePyramid never threw (atlas rows do not contain the requested ROI slots)');
  assert.equal(diag.stats.stageErrors, 0);
  assert.deepEqual(s.errors, [], 'errorCallback was not used');
  assert.deepEqual(s.exceptions, []);
  assert.ok(s.rows.slice(fromRow - 1, fromRow + 1).includes(layout.roiAtlasRows), 'the draw that still had ROIs was read with the ROI rows');
  assert.equal(s.rows.at(-1), layout.rows, 'once the engine list is empty the old rows are used again');
  detach(s);
});

test('RM4D-A1b (BLOCKER B-1): replacing the image targets from app code while the new ones are still loading does not kill the session', async () => {
  let release = null;
  const loader = async (url, target) => {
    if (target.name === 'tC') await new Promise((r) => { release = r; });
    return { width: IT_W, height: IT_H, luminance: itLuminance(target.name.charCodeAt(1) + 7) };
  };
  const s = await roiSession('rm4d-a1b', { loader });
  const diag = globalThis.__slamRefactChunk;
  runFrames(s, 2);
  splitFrame(s, () => s.ctrl.configure({ imageTargetData: [itTarget('tC')] }));
  const during = runFrames(s, 5);
  assert.ok(during.every(Boolean), 'answers while the replacement loads');
  assert.deepEqual(s.errors, []);
  assert.equal(diag.stats.processErrors, 0);
  await new Promise((r) => setTimeout(r, 20));
  release();
  await diag.imageTargets.whenIdle();
  assert.ok(runFrames(s, 5).every(Boolean));
  assert.deepEqual(diag.imageTargets.loadedNames(), ['tC']);
  assert.deepEqual(s.errors, []);
  assert.deepEqual(s.exceptions, []);
  detach(s);
});

test('RM4D-A1c: the ROI-row padding does not depend on the JS side having targets loaded', async () => {
  let asks = 0;
  const fakeRois = (rec) => rec.real.pyramidLayout().roiSlots.slice(0, 2).map((l, i) => ({ slot: i, source: 'IMAGE_TARGET', name: 'tA', warp: warpOf(i + 1) }));
  const hooks = { override: { getRequestedRois: (rec) => (++asks % 2 === 1 ? [] : fakeRois(rec)) } };
  const s = await attachedSession('rm4d-a1c', { hooks, glOpts: { fillReadback: 7 } });  // no image target configured at all
  const diag = globalThis.__slamRefactChunk;
  const layout = s.glues[0].real.pyramidLayout();
  runFrames(s, 6);
  const processed = s.glues[0].calls.filter(([n]) => n === 'processFramePyramid').slice(-4);
  for (const [, , , atlas, , rows] of processed) assert.equal(rows, layout.roiAtlasRows, 'padded to the ROI rows although no target is loaded');
  assert.ok(diag.stats.roiPaddedFrames >= 2, `padded ${diag.stats.roiPaddedFrames}`);
  assert.deepEqual(s.exceptions, []);
  detach(s);
});

// portrait -> landscape (inactive) -> portrait, as the frames xr.js delivers for a phone turned there and back
const turnToLandscape = (s, w, h) => {
  xrFrame(s, { w, h, before: () => s.pm.onDeviceOrientationChange(sizeChangeArgs(s, w, h, UNSUPPORTED_ROTATION)) });
  xrFrame(s, { w: h, h: w, before: () => s.pm.onVideoSizeChange(sizeChangeArgs(s, h, w, UNSUPPORTED_ROTATION)) });
  runFrames(s, 2, { w: h, h: w });
};
const turnToPortrait = (s, w, h) => {
  xrFrame(s, { w: h, h: w, before: () => s.pm.onDeviceOrientationChange(sizeChangeArgs(s, h, w, UNSUPPORTED_ROTATION)) });
  return [xrFrame(s, { w, h, before: () => s.pm.onVideoSizeChange(sizeChangeArgs(s, w, h, 0)) }), ...runFrames(s, 10)];
};
const rotateAndBack = (s, w, h) => { turnToLandscape(s, w, h); return turnToPortrait(s, w, h); };

test('RM4D-A2 (M-1): a target the engine refuses that was loaded while no session existed is dropped alone; the portrait rebuild survives', async () => {
  const s = await attachedSession('rm4d-a2', { chunkOptions: { loadLuminance: itLoader(), attachBudgetMs: Infinity } });
  const diag = globalThis.__slamRefactChunk;
  runFrames(s, 9);
  s.pm.onAppResourcesLoaded({ imageTargets: undefined, framework: s.framework });
  const errors = []; const e0 = console.error; console.error = (...a) => errors.push(a.join(' '));
  const warn = console.warn; console.warn = () => {};
  let back, again;
  try {
    turnToLandscape(s, W, H);
    // no session exists: the loaded targets are not validated by any engine
    s.ctrl.configure({ imageTargetData: [itTarget('tA'), itRefusedCurvy('tCurvy')] });
    await diag.imageTargets.whenIdle();
    assert.deepEqual(diag.imageTargets.loadedNames(), ['tA', 'tCurvy'], 'both count as loaded while nobody could refuse them');
    assert.equal(errors.length, 0);
    back = turnToPortrait(s, W, H);
    again = rotateAndBack(s, W, H);  // and the next rotation is fine too
  } finally { console.error = e0; console.warn = warn; }
  assert.deepEqual(s.errors, [], 'the session was not failed through errorCallback');
  assert.deepEqual(s.exceptions, []);
  assert.ok(errors.some((m) => REFUSED_CURVY('tCurvy').test(m)), errors.join('|'));
  assert.deepEqual(diag.imageTargets.loadedNames(), ['tA'], 'only the refused target is gone');
  assert.deepEqual(s.glues.at(-1).real.getImageTargetNames(), ['tA']);
  assert.ok(back.slice(-4).every(Boolean) && again.slice(-4).every(Boolean), 'answers after the rebuilds');
  const scanning = s.events.filter(([n]) => n === 'imagescanning');
  assert.deepEqual(scanning.at(-1)[1].imageTargets.map((t) => t.name), ['tA'], 'the app is told the target set that really is active');
  assert.equal(globalThis.__slamRefactChunk.session.videoWidth, W);
  // the refused target is not remembered as loaded: configuring the same list tries it again (and it is refused again), as for a
  // target refused at load time
  const errorsBefore = diag.imageTargets.loadErrors;
  const e1 = console.error; console.error = () => {};
  try {
    s.ctrl.configure({ imageTargetData: [itTarget('tA'), itRefusedCurvy('tCurvy')] });
    await diag.imageTargets.whenIdle();
  } finally { console.error = e1; }
  assert.equal(diag.imageTargets.loadErrors, errorsBefore + 1, 'tried again');
  assert.deepEqual(diag.imageTargets.loadedNames(), ['tA']);
  detach(s);
});

test('RM4D-A3 (M-2): the rebuild replays the loaded targets spread over frames by a time budget; 0 = one per frame, Infinity = one call', async () => {
  const names = (rec) => rec.calls.filter(([n]) => n === 'setImageTargets').map(([, list]) => list.map((x) => JSON.parse(x.json).name));
  const three = ['tA', 'tB', 'tC'].map((n) => itTarget(n));
  const s = await itSession('rm4d-a3', { targets: three, chunkOptions: { attachBudgetMs: 0 } });
  const pending = () => globalThis.__slamRefactChunk.imageTargets.replayPending();
  runFrames(s, 9);
  resize(s);  // onVideoSizeChange: the first target; the stage of the same frame: the second
  assert.deepEqual(names(s.glues[1]), [['tA'], ['tA', 'tB']]);
  assert.equal(pending(), 1);
  // a size change while the replay is pending: the new glue starts its own replay, the old one is dropped
  resize(s, 300, 400);
  assert.deepEqual(names(s.glues[2]), [['tA'], ['tA', 'tB']]);
  // the app unloads tB and loads a new tD meanwhile: tD reaches the engine at once, the not yet replayed tC does not jump the queue
  s.ctrl.configure({ imageTargetData: [three[0], three[2], itTarget('tD')] });
  await globalThis.__slamRefactChunk.imageTargets.whenIdle();
  assert.deepEqual(s.glues[2].real.getImageTargetNames().sort(), ['tA', 'tD'], 'tB unloaded, tD loaded, tC still waiting for its turn');
  runFrames(s, 2, { w: 300, h: 400 });
  assert.equal(pending(), 0);
  assert.deepEqual(s.glues[2].real.getImageTargetNames().sort(), ['tA', 'tC', 'tD'], 'the replay finished with the current set');
  assert.deepEqual(globalThis.__slamRefactChunk.imageTargets.loadedNames().sort(), ['tA', 'tC', 'tD']);
  assert.deepEqual(s.errors, []);
  assert.deepEqual(s.exceptions, []);
  detach(s);
  // Infinity: one engine call with everything
  const t = await itSession('rm4d-a3b', { targets: three, chunkOptions: { attachBudgetMs: Infinity } });
  runFrames(t, 9);
  resize(t);
  assert.deepEqual(names(t.glues[1]), [['tA', 'tB', 'tC']]);
  assert.equal(globalThis.__slamRefactChunk.imageTargets.replayPending(), 0);
  detach(t);
});

test('RM4D-A4 (m1): a refused 33-target configure changes nothing - no earlier option of the same call is applied or left unforwarded', async () => {
  const s = await attachedSession('rm4d-a4', { chunkOptions: { loadLuminance: itLoader() } });
  runFrames(s, 9);
  const many = Array.from({ length: 33 }, (_, i) => itTarget(`t${i}`));
  const configures = () => s.glues[0].calls.filter(([n]) => n === 'configure').length;
  const c0 = configures();
  assert.throws(() => s.ctrl.configure({ enableLighting: true, imageTargetData: many }), { message: '[XR] Too many active image targets: 33 > 32' });
  assert.equal(configures(), c0, 'nothing was forwarded to the glue');
  const rs = runFrames(s, 4);
  assert.equal(rs.at(-1).lighting, undefined, 'enableLighting was not applied on the JS side either (JS state and glue agree)');
  s.ctrl.configure({ enableLighting: true });
  assert.equal(runFrames(s, 4).at(-1).lighting.temperature, 6500, 'a clean configure applies it');
  detach(s);
});

test('RM4D-A5 (m3): a compute context that comes back without another size change gets its session back', async () => {
  const s = await attachedSession('rm4d-a5', { hooks: injectPose(P1, Q1) });
  const diag = globalThis.__slamRefactChunk;
  runFrames(s, 8);
  s.fake.lose();
  runFrames(s, 2);
  const lostResize = [resize(s), ...runFrames(s, 2, { w: W2, h: H2 })];
  assert.ok(lostResize.every((r) => r === null), 'null while the context is lost');
  assert.ok(diag.sessionLog.some((e) => e.event === 'deferred'));
  const created = diag.stats.sessionsCreated;
  s.fake.restore();
  const rs = runFrames(s, 8, { w: W2, h: H2 });  // no size change event
  assert.ok(diag.sessionLog.some((e) => e.event === 'resumed'), 'the postponed rebuild was made');
  assert.equal(diag.stats.sessionsCreated, created + 1, 'exactly one session');
  assert.equal(diag.session.videoWidth, W2);
  assert.ok(rs.slice(-4).every(Boolean), 'answers again');
  // RM-4e (review A-2, mutant MV1): the postponed rebuild is the videoSize kind it was asked for - the origin of the new glue is reset to the
  // last pose like a direct size change does (an orientation-kind rebuild would send no reset at all)
  const resets = resetCalls(s.glues[1]);
  assert.equal(resets.length, 1, 'resetOriginToLastPose ran for the postponed videoSize rebuild');
  assert.deepEqual(resets[0][1], { origin: xyz(P1), facing: wxyz(Q1), updateRecenterPoint: false });
  assert.equal(s.errors.length, 1, 'only the loss itself was reported');
  assert.deepEqual(s.exceptions, []);
  detach(s);
  // a detach while the rebuild is postponed forgets it
  const u = await attachedSession('rm4d-a5b');
  const diagU = globalThis.__slamRefactChunk;  // each controller has its own diagnostics object, this is u's
  runFrames(u, 3);
  u.fake.lose(); runFrames(u, 1);
  resize(u);
  assert.ok(diagU.sessionLog.some((e) => e.event === 'deferred'));
  detach(u);
  u.fake.restore();
  const before = diagU.stats.sessionsCreated;
  u.pm.onProcessCpu({ framework: u.framework, frameStartResult: { computeTexture: {}, cameraTexture: {}, textureWidth: W2, textureHeight: H2, orientation: 0, repeatFrame: false, videoTime: 9, frameTime: 9 } });
  assert.equal(diagU.stats.sessionsCreated, before, 'no session after a detach');
});

test('RM4E-A3 (v3 review A-3, mutant MV2): a size change while the context is lost cancels the postponed rebuild of the older size', async () => {
  const s = await attachedSession('rm4e-a3');
  const diag = globalThis.__slamRefactChunk;
  runFrames(s, 8);
  s.fake.lose();
  runFrames(s, 1);
  resize(s);  // portrait, postponed
  assert.ok(diag.sessionLog.some((e) => e.event === 'deferred'));
  const warn = console.warn;
  console.warn = () => {};
  try {
    xrFrame(s, { w: H2, h: W2, before: () => s.pm.onVideoSizeChange(sizeChangeArgs(s, H2, W2, UNSUPPORTED_ROTATION)) });  // landscape now, the context is still lost
    const created = diag.stats.sessionsCreated;
    s.fake.restore();
    runFrames(s, 6, { w: H2, h: W2 });
    assert.equal(diag.stats.sessionsCreated, created, 'no session for the stale portrait size when the context comes back');
    assert.ok(!diag.sessionLog.some((e) => e.event === 'resumed' && e.kind === 'videoSize'), 'the older postponed rebuild was not resumed');
  } finally { console.warn = warn; }
  detach(s);
});

test('RM4E-A5 (v3 review A-5): the targets that are detected right now are replayed first, so a rebuild with a time-sliced replay does not make them disappear', async () => {
  const names = ['tA', 'tB', 'tC', 'tD', 'tE', 'tF'];
  const run = async (tag, budget) => {
    // the engine reports tF detected as long as the glue of the moment holds it (a stand-in for a real detection)
    const hooks = { override: { processFramePyramid: (rec, ...args) => {
      const real = rec.real.processFramePyramid(...args);
      return { ...real, detectedImages: rec.real.getImageTargetNames().includes('tF') ? [detImage('tF', 1)] : [] };
    } } };
    const s = await itSession(tag, { targets: names.map((n) => itTarget(n)), hooks, chunkOptions: { attachBudgetMs: budget } });
    runFrames(s, 10);
    const before = imageEventsOf(s).filter(([n]) => /found|lost/.test(n)).map(([n]) => n);
    resize(s);
    const answers = runFrames(s, 12, { w: W2, h: H2 }).map((r) => r && r.detectedImages.length);
    const order = s.glues[1].calls.filter(([n]) => n === 'setImageTargets').map(([, list]) => list.map((x) => JSON.parse(x.json).name).at(-1));
    const all = imageEventsOf(s).filter(([n]) => /found|lost/.test(n)).map(([n]) => n);
    detach(s);
    return { before, all, answers, order };
  };
  for (const budget of [0, 8]) {
    const r = await run(`rm4e-a5-${budget}`, budget);
    assert.deepEqual(r.before, ['imagefound'], 'detected before the rebuild');
    assert.deepEqual(r.all, ['imagefound'], `budget ${budget}: no imagelost / imagefound pair while the replay runs`);
    assert.equal(r.order[0], 'tF', 'the detected target is the first one given to the new glue');
  }
  // with nothing detected the replay keeps the load order
  const plain = await itSession('rm4e-a5-plain', { targets: names.map((n) => itTarget(n)), chunkOptions: { attachBudgetMs: 0 } });
  runFrames(plain, 9);
  resize(plain);
  assert.deepEqual(plain.glues[1].calls.filter(([n]) => n === 'setImageTargets').map(([, list]) => list.map((x) => JSON.parse(x.json).name).at(-1)), ['tA', 'tB']);
  detach(plain);
});

test('RM4D-A6 (m2, mutant K1): after an inactive episode and a fresh session, an ordinary size change repeats the last pipeline response again', async () => {
  const s = await attachedSession('rm4d-a6', { hooks: injectPose(P1, Q1) });
  runFrames(s, 9);
  const warn = console.warn; console.warn = () => {};
  let before, gap;
  try {
    rotateAndBack(s, W, H);
    before = runFrames(s, 9);
    gap = [resize(s), ...runFrames(s, 2, { w: W2, h: H2 })];
  } finally { console.warn = warn; }
  const last = before.at(-1);
  assert.ok(last.realityTexture.drawFor !== undefined, 'the last answer before the size change is a pipeline answer');
  for (const [i, r] of gap.slice(0, 2).entries()) {
    assert.deepEqual(r.realityTexture, last.realityTexture, `gap answer ${i} repeats the last pipeline response (texture)`);
    assert.equal(r.trackingStatus, last.trackingStatus, `gap answer ${i}: status of the repeated response, not the inactive LIMITED`);
  }
  assert.deepEqual(s.exceptions, []);
  detach(s);
});

const nextUp = (x) => { const f = new Float64Array([x]); const u = new BigUint64Array(f.buffer); u[0] += 1n; return f[0]; };
const nextDown = (x) => { const f = new Float64Array([x]); const u = new BigUint64Array(f.buffer); u[0] -= 1n; return f[0]; };

test('RM4D-A7 (m2, mutant K3): the interval limit is exactly 2^64 ns, the engine rule, from both sides', async () => {
  const { pushMotion } = await import(pathToFileURL(path.join(chunkDir, 'sensors.js')).href);
  const s = await attachedSession('rm4d-a7');
  const real = s.glues[0].real;
  const TWO_64 = 2n ** 64n;
  const ev = (interval) => ({ timestampNanos: 1e9, interval, acceleration: null, accelerationIncludingGravity: { x: 0, y: 9.8, z: 0 }, rotationRate: null });
  const engineAccepts = (interval) => { try { real.pushDeviceMotionEvent(1e9, ev(interval)); return true; } catch (e) { return false; } };
  const around = 18446744073709551616 / 1e6;  // 2^64 ns in ms
  let ms = around;
  for (let i = 0; i < 3; i++) ms = nextDown(ms);
  const seen = [];
  for (let i = 0; i < 7; i++, ms = nextUp(ms)) {
    const ns = ms * 1000 * 1000;  // sensors.ts:56
    const accept = BigInt(ns) <= TWO_64;  // the capnp Int64 range: |ns| / 2^32 <= 2^32
    seen.push([ms, accept]);
    assert.equal(pushMotion({ pushDeviceMotionEvent: () => {} }, ev(ms)), accept, `pushMotion at ${ms} ms (${ns} ns)`);
    assert.equal(engineAccepts(ms), accept, `the engine binding at ${ms} ms agrees`);
  }
  assert.ok(seen.some(([, a]) => a) && seen.some(([, a]) => !a), 'both sides of the boundary were covered');
  assert.equal(BigInt(around * 1000 * 1000), TWO_64, 'one of the values is exactly 2^64 ns and is accepted (a "<" instead of "<=" rejects it)');
  assert.equal(pushMotion({ pushDeviceMotionEvent: () => {} }, ev(around)), true);
  assert.equal(pushMotion({ pushDeviceMotionEvent: () => {} }, ev(-around)), true, 'negative magnitudes use the same limit');
  assert.equal(pushMotion({ pushDeviceMotionEvent: () => {} }, ev(nextUp(around))), false);
  detach(s);
});

test('RM4D-A8 (m2, mutant K2/X1): an attach that follows an attach without a detach continues the startup recenter count', async () => {
  const s = await attachedSession('rm4d-a8');
  runFrames(s, 3);
  const left = s.glues[0].real.getFramesUntilRecenter();
  assert.ok(left > 0 && left < 5, `count left ${left}`);
  s.pm.onSessionAttach({ videoWidth: W, videoHeight: H, canvasWidth: W, canvasHeight: H, rotation: 0, computeCtx: s.fake.gl, framework: s.framework,
    GLctx: fakeGl().gl, sessionAttributes: { fillsCameraTexture: true } });  // no onSessionDetach in between (engineInit again)
  assert.equal(s.glues.length, 2);
  assert.equal(s.glues[1].config.framesUntilRecenter, left, 'the count is continued like any engineInit again (xr.js calling attach twice is UNVERIFIED)');
  detach(s);
  const n = s.glues.length;
  s.pm.onSessionAttach({ videoWidth: W, videoHeight: H, canvasWidth: W, canvasHeight: H, rotation: 0, computeCtx: s.fake.gl, framework: s.framework,
    GLctx: fakeGl().gl, sessionAttributes: { fillsCameraTexture: true } });
  assert.equal('framesUntilRecenter' in s.glues[n].config, false, 'after a detach the default 5 again');
  detach(s);
});

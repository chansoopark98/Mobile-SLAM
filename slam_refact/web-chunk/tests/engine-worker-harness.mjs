// Shared by the R1 engine-Worker tests: a loopback transport that behaves like a Worker (asynchronous delivery, structured-clone
// copies, Transferable buffers detached on send), the real engine (current tracked build by default), a deterministic synthetic
// camera sequence, and a runner that drives the chunk's pipeline module in node with a fake GL (the same approach as
// frame-scheduler-fixture.mjs, parametrised by the engineWorker option).
import fs from 'node:fs';
import path from 'node:path';
import { createRequire } from 'node:module';
import { pathToFileURL } from 'node:url';
import { fakeGl } from '../tools/or1/players.mjs';
import { createEngineWorkerCore } from '../engine-worker.js';

export const repo = path.resolve(import.meta.dirname, '../../..');
export const chunkDir = path.join(repo, 'slam_refact/web-chunk');
// The tracked engine of the deployed XR build, unless a directory is given (manifest.json + slam_refact_engine.{js,wasm}).
export const engineDir = process.env.SLAM_REFACT_CHUNK_ENGINE_DIR ?? path.join(repo, 'web/slam-refact-xr/engine/slam-refact-engine');
export const engineUrl = new URL('slam-refact-engine/', pathToFileURL(`${chunkDir}/`));
export const engineAvailable = fs.existsSync(path.join(engineDir, 'manifest.json'));

// Loopback Worker: postMessage clones (and detaches transfers) like a real one, delivery is asynchronous and ordered.
// delay(msg) -> ms (0 = setImmediate) models a slow engine thread. core deps are injectable (fake engine).
export function loopbackWorker({ coreDeps = {}, delay = () => 0, onToWorker = null } = {}) {
  const state = { toWorker: [], fromWorker: [], inFlight: 0, worker: null, killed: false };
  const idleWaiters = [];
  const settle = () => { if (state.inFlight === 0) while (idleWaiters.length) idleWaiters.shift()(); };
  const later = (ms, fn) => (ms > 0 ? setTimeout(fn, ms) : setImmediate(fn));
  // The worker side processes messages one at a time in arrival order (a chain), whatever the per-message delay.
  let workerChain = Promise.resolve();
  const worker = {
    onmessage: null, onerror: null, onmessageerror: null,
    postMessage(message, transfer = []) {
      if (state.killed) return;
      const clone = structuredClone(message, { transfer });
      state.toWorker.push(clone);
      onToWorker?.(clone);
      state.inFlight++;
      workerChain = workerChain.then(() => new Promise((resolve) => later(delay(clone), async () => {
        try { await core.handle(clone); } finally { state.inFlight--; resolve(); settle(); }
      })));
    },
    terminate() { state.killed = true; },
  };
  const core = createEngineWorkerCore({ deps: coreDeps, post: (message, transfer = []) => {
    if (state.killed) return;
    const clone = structuredClone(message, { transfer });
    state.fromWorker.push(clone);
    state.inFlight++;
    setImmediate(() => { try { worker.onmessage?.({ data: clone }); } finally { state.inFlight--; settle(); } });
  } });
  state.worker = worker;
  return { worker, state, core,
    idle: () => (state.inFlight === 0 ? Promise.resolve() : new Promise((resolve) => idleWaiters.push(resolve))),
    crash: (message) => worker.onerror?.({ message }) };
}

// ---- node globals the chunk and engine-loader expect (restored by the returned function) ----
export function installGlobals({ clock = { now: 1000 }, engineBase = null } = {}) {
  const saved = new Map(), listeners = new Map();
  const set = (key, value) => { saved.set(key, Object.getOwnPropertyDescriptor(globalThis, key));
    Object.defineProperty(globalThis, key, { configurable: true, writable: true, value }); };
  const url = { create: URL.createObjectURL, revoke: URL.revokeObjectURL };
  set('window', globalThis);
  set('performance', { now: () => clock.now, timeOrigin: 1.7e12 });
  set('document', { visibilityState: 'visible', hasFocus: () => true });
  set('addEventListener', (name, cb) => { const a = listeners.get(name) ?? []; if (!a.includes(cb)) a.push(cb); listeners.set(name, a); });
  set('removeEventListener', (name, cb) => listeners.set(name, (listeners.get(name) ?? []).filter((f) => f !== cb)));
  set('_c8', { checkAllZeroes: 0 });
  set('XR8', { isPaused: () => false,
    XrDevice: { deviceEstimate: () => ({ manufacturer: 'Google', model: 'Pixel 7' }), isDeviceBrowserCompatible: () => true },
    XrConfig: { camera: () => ({ FRONT: 'front', BACK: 'back' }), device: () => ({ MOBILE: 'mobile', ANY: 'any' }) },
    XrPermissions: { permissions: () => ({ CAMERA: 'camera', DEVICE_MOTION: 'devicemotion', DEVICE_ORIENTATION: 'deviceorientation' }) },
    drawTexForComputeTex: (t) => t });
  const base = engineBase ?? new URL('slam-refact-engine/', pathToFileURL(`${chunkDir}/`)).href;
  set('fetch', async (u) => {
    const href = String(u);
    if (!href.startsWith(base)) throw new Error(`unexpected fetch ${href}`);
    const b = fs.readFileSync(path.join(engineDir, href.slice(base.length)));
    return { ok: true, arrayBuffer: async () => b.buffer.slice(b.byteOffset, b.byteOffset + b.byteLength) };
  });
  set('Blob', class { constructor(parts) { this.parts = parts; } });
  set('require', createRequire(path.join(engineDir, 'slam_refact_engine.js'))); set('__dirname', engineDir);
  URL.createObjectURL = (blob) => `data:text/javascript;base64,${Buffer.concat(blob.parts.map((p) => Buffer.from(typeof p === 'string' ? p : new Uint8Array(p)))).toString('base64')}`;
  URL.revokeObjectURL = () => {};
  return { clock, listeners, set,
    dispatch: (e) => { for (const cb of listeners.get(e.type) ?? []) cb(e); },
    restore() {
      for (const [k, d] of saved) { if (d) Object.defineProperty(globalThis, k, d); else delete globalThis[k]; }
      URL.createObjectURL = url.create; URL.revokeObjectURL = url.revoke;
    } };
}

// ---- deterministic synthetic camera: a textured plane under a slow translation, as CPU-port pyramid atlases ----
export function syntheticImage(width, height, k) {
  const rgba = new Uint8Array(width * height * 4);
  const dx = 0.6 * k, dy = 0.35 * k;
  for (let y = 0; y < height; y++) {
    for (let x = 0; x < width; x++) {
      const u = x + dx, v = y + dy;
      // blobs + checker edges + a hashed speckle: plenty of corners, fully deterministic
      const h = ((Math.floor(u / 3) * 73856093) ^ (Math.floor(v / 3) * 19349663)) >>> 0;
      let g = 110 + 60 * Math.sin(u * 0.11) * Math.cos(v * 0.09) + ((h % 97) - 48) * 0.9 + (((Math.floor(u / 16) + Math.floor(v / 16)) & 1) ? 22 : -22);
      g = Math.max(0, Math.min(255, Math.round(g)));
      const i = (y * width + x) * 4;
      rgba[i] = rgba[i + 1] = rgba[i + 2] = g; rgba[i + 3] = 255;
    }
  }
  return rgba;
}

export async function loadRealModule() {
  const require_ = createRequire(path.join(engineDir, 'slam_refact_engine.js'));
  return require_(path.join(engineDir, 'slam_refact_engine.js'))();
}

// Atlas bytes of the synthetic frame k, as the GPU pyramid would read back (CPU port, 'ieee').
export function syntheticAtlas(Module, width, height, k) {
  const cpu = Module.SlamRefactGlue.cpuPyramidAtlas(syntheticImage(width, height, k), width, height, 'ieee');
  return { rgba: cpu.rgba, size: cpu.size, rows: cpu.rows };
}

// IMU of step k: a small deterministic motion (all three sensors).
export function syntheticMotion(k) {
  const t = k * 0.033;
  return { type: 'devicemotion', timeStamp: 1000 + k * 33 - 5, interval: 16,
    acceleration: { x: 0.05 * Math.sin(t), y: 0.02, z: 0.03 * Math.cos(t) },
    accelerationIncludingGravity: { x: 0.05 * Math.sin(t), y: 9.78, z: 0.03 * Math.cos(t) },
    rotationRate: { alpha: 0.01 * Math.cos(t), beta: 0.02, gamma: 0.005 * Math.sin(t) } };
}

let serial = 0;
// Runs the chunk with the real engine through `steps` camera frames. engineWorker: false (default path) or an options object
// (true / {enabled: true, createWorker, maxInFlight}). Returns the answers of every onProcessCpu plus the diag object.
export async function runChunk({ steps, engineWorker = false, extraOptions = {}, width = 240, height = 320, atlasFor, afterEach, loop = null,
  attach = true, engineBase = null, globals = null }) {
  const g = installGlobals({ engineBase });
  globals?.(g);
  const state = { atlas: null };
  const gl = new Proxy(fakeGl(state), { get(target, name) {
    const value = target[name];
    if (name === 'getParameter') return (key) => key === target.MAX_TEXTURE_SIZE ? 8192 : value(key);
    return value;
  } });
  const errors = [], answers = [];
  let pm, ctrl;
  try {
    window.SLAM_REFACT_CHUNK_OPTIONS = { ...extraOptions, ...(engineWorker ? { engineWorker } : {}) };
    const mod = await import(`${pathToFileURL(path.join(chunkDir, 'xr-slam.js')).href}?r1=${++serial}`);
    ctrl = await mod.XrControllerFactory({ paused: false }, Promise.resolve({}), () => () => {}, (e) => { errors.push(String(e && e.message || e)); });
    pm = ctrl.pipelineModule();
    ctrl.updateCameraProjectionMatrix({ cam: { pixelRectWidth: width, pixelRectHeight: height, nearClipPlane: 0.01, farClipPlane: 1000 } });
    pm.onBeforeSessionInitialize({ sessionAttributes: { fillsCameraTexture: true, controlsCamera: false } });
    pm.onStart({ computeCtx: gl, canvasWidth: width, canvasHeight: height });
    const framework = { dispatchEvent: () => {} };
    const attachArgs = { videoWidth: width, videoHeight: height, canvasWidth: width, canvasHeight: height, rotation: 0,
      computeCtx: gl, framework, GLctx: gl, sessionAttributes: { fillsCameraTexture: true } };
    if (attach) pm.onSessionAttach(attachArgs);
    const frame = (k, repeat = false) => ({ computeTexture: { id: k }, cameraTexture: { id: k }, textureWidth: width, textureHeight: height,
      orientation: 0, repeatFrame: repeat, videoTime: 20 + k / 30, frameTime: k === 0 ? 0 : 1 + k / 30 });
    const ctx = { ctrl, pm, gl, framework, errors, answers, frame, attachArgs, state, g,
      diag: () => window.__slamRefactChunk,
      gpu: (f) => pm.onProcessGpu({ framework, frameStartResult: f }),
      cpu: (f) => pm.onProcessCpu({ framework, frameStartResult: f }) };
    const summarize = (r) => (r ? { position: r.position, rotation: r.rotation, intrinsics: r.intrinsics, trackingStatus: r.trackingStatus,
      trackingReason: r.trackingReason, worldPoints: r.worldPoints, texture: r.realityTexture?.id ?? null } : null);
    if (loop) await loop.idle?.();
    for (let k = 0; k < steps; k++) {
      g.clock.now = 1000 + k * 33;
      g.dispatch(syntheticMotion(k));
      const f = frame(k);
      ctx.gpu(f);
      state.atlas = atlasFor(k);
      answers.push(summarize(ctx.cpu(f)));
      await loop?.idle();
      await afterEach?.(k, ctx);
    }
    ctx.drain = async (n = 3) => {  // repeated callbacks take the results that are still on their way
      for (let i = 0; i < n; i++) { await loop?.idle(); answers.push(summarize(ctx.cpu(frame(steps + i, true)))); }
    };
    ctx.finish = async () => { const d = ctx.diag(); return d; };
    return { ctx, answers, errors, diag: window.__slamRefactChunk,
      async drain(n) { await ctx.drain(n); return answers; },
      close() { try { pm.onSessionDetach({}); pm.onDetach({}); } finally { g.restore(); } } };
  } catch (error) {
    g.restore();
    throw error;
  }
}

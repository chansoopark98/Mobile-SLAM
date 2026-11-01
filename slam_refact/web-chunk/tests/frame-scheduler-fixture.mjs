// B1 scheduler control-flow fixture: real pinned WASM, fake GL. Pixel/renderer evidence belongs to desktop D5.
import assert from 'node:assert/strict';
import fs from 'node:fs';
import path from 'node:path';
import { createRequire } from 'node:module';
import { pathToFileURL } from 'node:url';
import { createSensors, pushMotion } from '../sensors.js';
import { Gr8ComputePyramid } from '../gr8-compute-pyramid.js';
import { fakeGl } from '../tools/or1/players.mjs';

const repo = path.resolve(import.meta.dirname, '../../..');
const chunkDir = process.env.SLAM_REFACT_CHUNK_DIR ?? path.join(repo, 'slam_refact/web-chunk');
const engineDir = process.env.SLAM_REFACT_CHUNK_ENGINE_DIR
  ?? path.join(repo, 'build/slam-refact-full-20261019/chunk/rm4d-20261020/engine-pinned');
const event = (timeStamp, interval = 16) => ({ type: 'devicemotion', timeStamp, interval,
  acceleration: null, accelerationIncludingGravity: { x: 0, y: 9.8, z: 0 }, rotationRate: { alpha: 0, beta: 0, gamma: 0 } });

let serial = 0;
export async function fixture(enabled, check, sourceDir = chunkDir, { failAttachGlue = false, ...extraOptions } = {}) {
  const saved = new Map(), listeners = new Map(), calls = [], errors = [], clockCalls = [], poses = [];
  const set = (key, value) => { saved.set(key, Object.getOwnPropertyDescriptor(globalThis, key));
    Object.defineProperty(globalThis, key, { configurable: true, writable: true, value }); };
  const deletedTextures = new Set();
  let now = 1000, failProcess = false, failRead = false, failStage = false, failPush = false,
    failWorldPoints = false, failDispatch = false, failCreateGlue = failAttachGlue, lost = false, paused = false,
    readyFence = true, drawFenceReady = true, drawFinishFail = false, drawLost = false, drawError = 0, drawErrorSticky = false,
    hooks = {}, reads = 0, lastGlue = null, noReadRequest = false, throwErrorCallback = false;
  const originalReadReady = Gr8ComputePyramid.prototype.isReadReady;
  Gr8ComputePyramid.prototype.isReadReady = function () {
    if (noReadRequest) this.needsRead = false;
    return originalReadReady.call(this);
  };
  set('window', globalThis); set('performance', { now: () => { clockCalls.push(now); return now; } });
  set('document', { visibilityState: 'visible', hasFocus: () => true });
  set('addEventListener', (name, cb) => { const a = listeners.get(name) ?? []; if (!a.includes(cb)) a.push(cb); listeners.set(name, a); });
  set('removeEventListener', (name, cb) => listeners.set(name, (listeners.get(name) ?? []).filter((f) => f !== cb)));
  set('_c8', { checkAllZeroes: 0 });
  set('XR8', { isPaused: () => paused, XrDevice: { deviceEstimate: () => ({ manufacturer: '', model: 'K' }), isDeviceBrowserCompatible: () => true },
    XrConfig: { camera: () => ({ FRONT: 'front', BACK: 'back' }), device: () => ({ MOBILE: 'mobile', ANY: 'any' }) },
    XrPermissions: { permissions: () => ({ CAMERA: 'camera', DEVICE_MOTION: 'devicemotion', DEVICE_ORIENTATION: 'deviceorientation' }) },
    drawTexForComputeTex: (t) => t });
  const base = new URL('slam-refact-engine/', pathToFileURL(`${sourceDir}/`)).href;
  set('fetch', async (url) => { assert.ok(String(url).startsWith(base)); const b = fs.readFileSync(path.join(engineDir, String(url).slice(base.length)));
    return { ok: true, arrayBuffer: async () => b.buffer.slice(b.byteOffset, b.byteOffset + b.byteLength) }; });
  set('Blob', class { constructor(parts) { this.parts = parts; } });
  set('require', createRequire(path.join(engineDir, 'slam_refact_engine.js'))); set('__dirname', engineDir);
  const url = { create: URL.createObjectURL, revoke: URL.revokeObjectURL };
  URL.createObjectURL = (blob) => `data:text/javascript;base64,${Buffer.concat(blob.parts.map((p) => Buffer.from(typeof p === 'string' ? p : new Uint8Array(p)))).toString('base64')}`;
  URL.revokeObjectURL = () => {};
  set('SLAM_REFACT_CHUNK_OPTIONS', { ...extraOptions, frameScheduler: { enabled, mode: 'gpu-task-owned-v1' }, deliveryDiagnostics: { enabled, maxEntries: 4 }, createGlue: (Module, config) => {
    if (failCreateGlue) { failCreateGlue = false; throw new Error('injected createGlue failure'); }
    const glue = new Module.SlamRefactGlue(config); lastGlue = glue;
    return new Proxy(glue, { get(target, name) { const value = target[name]; if (typeof value !== 'function') return value;
      return (...args) => {
        if (name === 'getRequestedRois' && failStage) { failStage = false; throw new Error('injected stage failure'); }
        if (name === 'pushDeviceMotionEvent' && failPush) { failPush = false; throw new Error('injected push failure'); }
        if (name === 'getWorldPoints' && failWorldPoints) { failWorldPoints = false; throw new Error('injected world-points failure'); }
        if (name === 'processFramePyramid' && failProcess) { failProcess = false; throw new Error('injected before stage'); }
        if (name === 'pushDeviceMotionEvent') calls.push({ timestampNanos: args[0], payload: args[1] });
        if (hooks[name]) hooks[name](...args);
        let result = value.apply(target, args); if (hooks.result) result = hooks.result(name, result);
        if (name === 'processFramePyramid') poses.push({ videoFrameIndex: result.videoFrameIndex, sourceFrameIndex: result.sourceFrameIndex,
          repeated: result.repeated, inputTimeNanos: args[0], inputVideoTimeNanos: args[1] });
        return result;
      }; } });
  } });
  const rawGl = fakeGl({ atlas: new Uint8Array(1024 * 1024 * 4) });
  const gl = new Proxy(rawGl, { get(target, name) { const value = target[name];
    if (name === 'isContextLost') return () => lost;
    if (name === 'isTexture') return () => true;
    if (name === 'getParameter') return (key) => key === target.MAX_TEXTURE_SIZE ? 8192 : value(key);
    if (name === 'clientWaitSync') return () => readyFence ? target.ALREADY_SIGNALED : target.TIMEOUT_EXPIRED;
    if (name === 'deleteTexture') return (texture) => { deletedTextures.add(texture); return value(texture); };
    if (name !== 'getBufferSubData') return value;
    return (...args) => { reads++; if (failRead) { failRead = false; throw new Error('injected read failure'); } return value(...args); }; } });
  const drawGl = new Proxy(gl, { get(target, name) { return name === 'isContextLost' ? () => drawLost
    : name === 'clientWaitSync' ? () => drawFenceReady ? target.ALREADY_SIGNALED : target.TIMEOUT_EXPIRED
      : name === 'finish' ? () => { if (drawFinishFail) throw new Error('injected draw finish failure'); return target.finish(); }
        : name === 'getError' ? () => { if (drawError) { const error = drawError; if (!drawErrorSticky) drawError = 0; return error; } return target.getError(); }
        : target[name]; } });
  const framework = { dispatchEvent: (name) => {
    if (hooks.dispatch) hooks.dispatch(name);
    if (name === 'trackingstatus' && failDispatch) { failDispatch = false; throw new Error('injected dispatch failure'); }
  } }; let pm;
  try {
    const mod = await import(`${pathToFileURL(path.join(sourceDir, 'xr-slam.js')).href}?delivery=${++serial}`);
    const ctrl = await mod.XrControllerFactory({ paused: false }, Promise.resolve({}), () => () => {}, (e) => {
      errors.push(String(e));
      if (throwErrorCallback) throw new Error('injected errorCallback failure');
    });
    pm = ctrl.pipelineModule();
    ctrl.updateCameraProjectionMatrix({ cam: { pixelRectWidth: 240, pixelRectHeight: 320, nearClipPlane: 0.01, farClipPlane: 1000 } });
    pm.onBeforeSessionInitialize({ sessionAttributes: { fillsCameraTexture: true, controlsCamera: false } });
    pm.onStart({ computeCtx: gl, canvasWidth: 240, canvasHeight: 320 });
    pm.onSessionAttach({ videoWidth: 240, videoHeight: 320, canvasWidth: 240, canvasHeight: 320, rotation: 0,
      computeCtx: gl, framework, GLctx: drawGl, sessionAttributes: { fillsCameraTexture: true } });
    const frame = (k) => ({ computeTexture: { id: k }, cameraTexture: { id: k }, textureWidth: 240, textureHeight: 320,
      orientation: 0, repeatFrame: false, videoTime: 20 + k / 30, frameTime: k === 0 ? 0 : 1 + k / 30 });
    const dispatch = (e) => { for (const cb of listeners.get(e.type) ?? []) cb(e); };
    await check({ ctrl, pm, gl, framework, frame, calls, poses, errors, clockCalls, deletedTextures,
      setNow: (v) => { now = v; }, sensor: (t, interval) => { now = t; dispatch(event(t - 5, interval)); },
      stage: (f) => pm.onProcessGpu({ framework, frameStartResult: f }),
      cpu: (f) => pm.onProcessCpu({ framework, frameStartResult: f }),
      snapshot: () => globalThis.__slamRefactChunk.delivery,
      latest: () => globalThis.__slamRefactChunk.sourceTiming, scheduler: () => globalThis.__slamRefactChunk.scheduler, nativeNames: () => lastGlue.getImageTargetNames(),
      drawFenceReady: (v) => { drawFenceReady = v; }, drawFinishFail: (v) => { drawFinishFail = v; },
      drawError: (v, sticky = false) => { drawError = v; drawErrorSticky = sticky; }, failCreateGlue: () => { failCreateGlue = true; },
      readyFence: (v) => { readyFence = v; }, reads: () => reads, hooks,
      noReadRequest: (v) => { noReadRequest = v; },
      errorCallbackThrows: (v) => { throwErrorCallback = v; },
      failProcess: () => { failProcess = true; }, failRead: () => { failRead = true; },
      failStage: () => { failStage = true; }, failPush: () => { failPush = true; },
      failWorldPoints: () => { failWorldPoints = true; }, failDispatch: () => { failDispatch = true; },
      loseDraw: () => { drawLost = true; }, loseContext: () => { lost = true; }, pause: (v) => { paused = v; } });
  } finally {
    Gr8ComputePyramid.prototype.isReadReady = originalReadReady;
    if (pm) { pm.onSessionDetach({}); pm.onDetach({}); }
    for (const [k, d] of saved) { if (d) Object.defineProperty(globalThis, k, d); else delete globalThis[k]; }
    URL.createObjectURL = url.create; URL.revokeObjectURL = url.revoke;
  }
}

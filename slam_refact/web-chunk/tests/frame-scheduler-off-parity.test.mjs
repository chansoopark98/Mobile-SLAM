import assert from 'node:assert/strict';
import { createHash } from 'node:crypto';
import fs from 'node:fs';
import path from 'node:path';
import { createRequire } from 'node:module';
import test from 'node:test';
import { pathToFileURL } from 'node:url';
import { fakeGl } from '../tools/or1/players.mjs';

// Frozen M0 source and engine are the oracle for D0. This is a local 20-frame
// contract test; the desktop gate records separate provenance and receipts.
const repo = path.resolve(import.meta.dirname, '../../..');
const frozen = path.join(repo, 'build/xr-accuracy-research-20261030/i2-b1prime/baseline');
const baseline = path.join(frozen, 'slam_refact/web-chunk');
const candidate = path.join(repo, 'slam_refact/web-chunk');
const engine = path.join(frozen, 'build/xr-accuracy-research-20261030/p0-x1/wasm-build/wasm');
const hash = (bytes) => createHash('sha256').update(bytes).digest('hex');
let serial = 0;

function stable(value) {
  if (typeof value === 'function') return { functionSource: String(value) };
  if (ArrayBuffer.isView(value)) return Array.from(value);
  if (Array.isArray(value)) return value.map(stable);
  if (value && typeof value === 'object') return Object.fromEntries(Object.keys(value).sort().map((key) => [
    key, key === 'stageMs' ? timingShape(value[key]) : stable(value[key]),
  ]));
  return value;
}
function timingShape(value) {
  if (typeof value === 'number') return { durationType: 'number' };
  if (Array.isArray(value)) return value.map(timingShape);
  if (value && typeof value === 'object') return Object.fromEntries(Object.keys(value).sort().map((key) => [key, timingShape(value[key])]));
  return value;
}

async function offTrace(sourceDir, options) {
  const saved = new Map(), listeners = new Map(), clock = [], native = [], configs = [], events = [], glCalls = {};
  const tasks = { construct: 0, post: 0 }; let now = 1000;
  const set = (key, value) => {
    saved.set(key, Object.getOwnPropertyDescriptor(globalThis, key));
    Object.defineProperty(globalThis, key, { configurable: true, writable: true, value });
  };
  set('window', globalThis);
  set('performance', { now: () => { clock.push(now); return now; } });
  set('document', { visibilityState: 'visible', hasFocus: () => true });
  set('addEventListener', (name, cb) => {
    const list = listeners.get(name) ?? []; if (!list.includes(cb)) list.push(cb); listeners.set(name, list);
  });
  set('removeEventListener', (name, cb) => listeners.set(name, (listeners.get(name) ?? []).filter((f) => f !== cb)));
  set('_c8', { checkAllZeroes: 0 });
  set('XR8', {
    isPaused: () => false,
    XrDevice: { deviceEstimate: () => ({ manufacturer: '', model: 'K' }), isDeviceBrowserCompatible: () => true },
    XrConfig: { camera: () => ({ FRONT: 'front', BACK: 'back' }), device: () => ({ MOBILE: 'mobile', ANY: 'any' }) },
    XrPermissions: { permissions: () => ({ CAMERA: 'camera', DEVICE_MOTION: 'devicemotion', DEVICE_ORIENTATION: 'deviceorientation' }) },
    drawTexForComputeTex: (texture) => ({ draw: texture }),
  });
  const OriginalMessageChannel = globalThis.MessageChannel;
  set('MessageChannel', class extends OriginalMessageChannel {
    constructor() {
      super(); tasks.construct++;
      const post = this.port2.postMessage.bind(this.port2);
      this.port2.postMessage = (...args) => { tasks.post++; return post(...args); };
    }
  });
  const base = new URL('slam-refact-engine/', pathToFileURL(`${sourceDir}/`)).href;
  set('fetch', async (url) => {
    assert.ok(String(url).startsWith(base));
    const bytes = fs.readFileSync(path.join(engine, String(url).slice(base.length)));
    return { ok: true, arrayBuffer: async () => bytes.buffer.slice(bytes.byteOffset, bytes.byteOffset + bytes.byteLength) };
  });
  set('Blob', class { constructor(parts) { this.parts = parts; } });
  set('require', createRequire(path.join(engine, 'slam_refact_engine.js')));
  set('__dirname', engine);
  const originalUrl = { create: URL.createObjectURL, revoke: URL.revokeObjectURL };
  URL.createObjectURL = (blob) => `data:text/javascript;base64,${Buffer.concat(blob.parts.map((p) => Buffer.from(typeof p === 'string' ? p : new Uint8Array(p)))).toString('base64')}`;
  URL.revokeObjectURL = () => {};
  set('SLAM_REFACT_CHUNK_OPTIONS', {
    ...options,
    createGlue: (Module, config) => {
      configs.push(stable(config));
      const glue = new Module.SlamRefactGlue(config);
      return new Proxy(glue, { get(target, name) {
        const value = target[name]; if (typeof value !== 'function') return value;
        return (...args) => {
          const record = { method: name, args: args.map((arg) => ArrayBuffer.isView(arg)
            ? { bytes: arg.byteLength, sha256: hash(arg) } : stable(arg)) };
          const result = value.apply(target, args);
          if (name === 'processFramePyramid') record.result = stable(result);
          native.push(record); return result;
        };
      } });
    },
  });
  const rawGl = fakeGl({ atlas: new Uint8Array(1024 * 1024 * 4) });
  const gl = new Proxy(rawGl, { get(target, name) {
    const value = target[name]; if (typeof value !== 'function') return value;
    return (...args) => { glCalls[name] = (glCalls[name] ?? 0) + 1; return value(...args); };
  } });
  const framework = { dispatchEvent: (name, detail) => events.push({ name, detail: stable(detail) }) };
  const answers = []; let pm;
  try {
    const { XrControllerFactory } = await import(`${pathToFileURL(path.join(sourceDir, 'xr-slam.js')).href}?d0-regression=${++serial}`);
    const ctrl = await XrControllerFactory({ paused: false }, Promise.resolve({}), () => () => {}, (error) => { throw error; });
    pm = ctrl.pipelineModule();
    ctrl.updateCameraProjectionMatrix({ cam: { pixelRectWidth: 240, pixelRectHeight: 320, nearClipPlane: 0.01, farClipPlane: 1000 } });
    pm.onBeforeSessionInitialize({ sessionAttributes: { fillsCameraTexture: true, controlsCamera: false } });
    pm.onStart({ computeCtx: gl, canvasWidth: 240, canvasHeight: 320 });
    pm.onSessionAttach({ videoWidth: 240, videoHeight: 320, canvasWidth: 240, canvasHeight: 320, rotation: 0,
      computeCtx: gl, framework, GLctx: gl, sessionAttributes: { fillsCameraTexture: true } });
    for (let k = 0; k < 20; k++) {
      now = 990 + k * 33;
      const event = { type: 'devicemotion', timeStamp: now - 5, interval: 16, acceleration: null,
        accelerationIncludingGravity: { x: 0, y: 9.8, z: 0 }, rotationRate: { alpha: 0, beta: 0, gamma: 0 } };
      for (const cb of listeners.get(event.type) ?? []) cb(event);
      now = 1000 + k * 33;
      const frame = { computeTexture: { id: k }, cameraTexture: { id: k }, textureWidth: 240, textureHeight: 320,
        orientation: 0, repeatFrame: false, videoTime: 20 + k / 30, frameTime: k === 0 ? 0 : 1 + k / 30 };
      pm.onProcessGpu({ framework, frameStartResult: frame });
      answers.push(stable(pm.onProcessCpu({ framework, frameStartResult: { ...frame,
        videoTime: 20 + (k + 1) / 30, frameTime: 1 + (k + 1) / 30 } })));
    }
    assert.equal(globalThis.__slamRefactChunk.scheduler, undefined);
    return { configs, native, answers, events, clock, tasks, glCalls };
  } finally {
    if (pm) { pm.onSessionDetach({}); pm.onDetach({}); }
    URL.createObjectURL = originalUrl.create; URL.revokeObjectURL = originalUrl.revoke;
    for (const [key, descriptor] of saved) {
      if (descriptor) Object.defineProperty(globalThis, key, descriptor); else delete globalThis[key];
    }
  }
}

test('D0 frozen M0 and B1prime default OFF have identical 20-frame trace bytes', async () => {
  assert.equal(hash(fs.readFileSync(path.join(baseline, 'xr-slam.js'))),
    'b8c10251b72d08bf65d3807a7ce979c57235c580725f786b8cd9b17b17d52067');
  assert.equal(hash(fs.readFileSync(path.join(baseline, 'sensors.js'))),
    '4a3bd250a9d4c4e14ed263601ea4e72665edd88c5b34cbefd3581ada238b6ba5');
  assert.equal(hash(fs.readFileSync(path.join(baseline, 'gr8-compute-pyramid.js'))),
    '92ffe5259a80db403d68eeeb0c06b573453f77756a4f01d3eaa6486e9b0c5fd8');
  assert.equal(hash(fs.readFileSync(path.join(engine, 'slam_refact_engine.js'))),
    '5f5eae695ff3a15267bbd35623b8fead6208b8bce9482acc4d63091a4114c0b9');
  assert.equal(hash(fs.readFileSync(path.join(engine, 'slam_refact_engine.wasm'))),
    'ffcd6435c9c58aa5ecb764cdae4b9a0f050849501626e66fa1b923d40dfd0c24');
  for (const options of [
    { deliveryDiagnostics: { enabled: false, maxEntries: 64 } },
    { frameScheduler: { enabled: false, mode: 'gpu-task-owned-v1' }, deliveryDiagnostics: { enabled: false, maxEntries: 64 } },
    { frameScheduler: { enabled: false, mode: 'gpu-task-owned-v1' }, deliveryDiagnostics: { enabled: true, maxEntries: 64 } },
  ]) {
    const frozenTrace = await offTrace(baseline, options);
    const currentTrace = await offTrace(candidate, options);
    assert.deepEqual(Buffer.from(JSON.stringify(currentTrace)), Buffer.from(JSON.stringify(frozenTrace)));
    assert.deepEqual(currentTrace.tasks, { construct: 0, post: 0 });
    assert.equal(currentTrace.answers.length, 20);
  }
});

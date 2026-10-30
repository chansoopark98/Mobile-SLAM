// OR-1 arms. One bundle (bundle.mjs) is played by
//   W  playDirect: the WASM engine glue called exactly like tools/harness/replay/replay_main.cc calls GlueRunner (flat pushDeviceMotion,
//      one pushDeviceOrientation per sensor packet, processFramePyramid per frame);
//   C  playChunk:  the slam_refact xr-slam.js chunk, in node, on the same WASM engine, driven like xr.js drives a pipeline module
//      (DeviceMotion / DeviceOrientation window events, onProcessGpu / onProcessCpu per frame) on a fake WebGL2 context whose pixel
//      pack buffer read returns the bundle's recorded atlas, with a fake clock that makes performance.now() return the bundle's times;
//   N  runNative:  the native slam_refact_replay on the same bundle (native.mjs).
// Each arm returns per-call "frame records" (the inputs the engine was given, in canonical form) and "rows" (the engine outputs as the
// canonical form of the pose object processFramePyramid returned), so inputs and outputs can both be compared bit for bit.
import fs from 'node:fs';
import path from 'node:path';
import { createRequire } from 'node:module';
import { pathToFileURL } from 'node:url';
import { canon, f64hex, sha256 } from './canon.mjs';
import { readAtlas } from './bundle.mjs';

const num = (s) => Number(s);

export function glueConfigOf(bundle) {
  const c = (k) => bundle.config.get(k);
  return {
    captureWidth: num(c('capture_width')), captureHeight: num(c('capture_height')), rotationDeg: num(c('rotation_deg')),
    manufacturer: c('manufacturer') ?? '', model: c('model') ?? '', estimateScale: c('estimate_scale') === '1', disableVio: c('disable_vio') === '1',
    leftHandedAxes: c('left_handed_axes') === '1', mirroredDisplay: c('mirrored_display') === '1', pyramidModel: c('pyramid_model'),
    origin: c('origin').split(' ').map(num), facing: c('facing_wxyz').split(' ').map(num),
  };
}

const absPath = (repoRoot, p) => (path.isAbsolute(p) ? p : path.join(repoRoot, p));

// ---- the per-call input record both arms produce (the thing that must be equal) ----
// motion: [{t, aig, lin, rate, interval}] pushed since the previous process call; orientation: the alpha/beta/gamma the engine had when it
// processed (SensorIntake keeps the latest, so one push per frame and one per packet are the same input); atlas: sha256 of the first
// `layoutRows` rows (what the chunk reads) and the number of rows the arm handed over.
function newRecorder(layoutRowsOf) {
  const frames = [];
  let motion = [], orient = null;
  return {
    frames,
    motion(rec) { motion.push({ ...rec, t: Math.trunc(rec.t) }); },  // the binding's Ns(): static_cast<int64_t>, i.e. truncation
    orientation(a, b, g, t) { orient = { a, b, g, t }; },
    process(timeNs, videoTimeNs, atlas, size, rows, index) {
      const used = atlas.subarray(0, layoutRowsOf() * size * 4);
      frames.push({ index, timeNs: Math.trunc(timeNs), videoTimeNs: Math.trunc(videoTimeNs), size, rowsGiven: rows, atlasPrefixSha256: sha256(used), atlasPrefixBytes: used.length,
        motion, orientation: orient ? { a: orient.a, b: orient.b, g: orient.g } : null, orientationTimeNs: orient ? orient.t : null });
      motion = [];
    },
  };
}
export const comparableFrame = (f) => canon({ timeNs: f.timeNs, videoTimeNs: f.videoTimeNs, size: f.size, atlasPrefixSha256: f.atlasPrefixSha256,
  atlasPrefixBytes: f.atlasPrefixBytes, motion: f.motion, orientation: f.orientation });

// pose object of processFramePyramid -> canonical row (every field; typed arrays by sha256)
// Timing keys are wall-clock measurements of the run, not engine outputs: `stageMs` is the only one the pose object has.
export const TIMING_KEYS = ['stageMs'];
export const poseRow = (pose) => canon(Object.fromEntries(Object.entries(pose).filter(([k]) => !TIMING_KEYS.includes(k))));

export async function loadEngineModule(engineDir) {
  const require = createRequire(import.meta.url);
  return require(path.join(engineDir, 'slam_refact_engine.js'))();
}

// ===================================================================================================
// W: direct WASM player (replay_main.cc order)
// ===================================================================================================
export async function playDirect({ engineDir, bundle, repoRoot, onFrame }) {
  const Module = await loadEngineModule(engineDir);
  const cfg = glueConfigOf(bundle);
  const glue = new Module.SlamRefactGlue(cfg);
  const layout = glue.pyramidLayout();
  const rec = newRecorder(() => layout.rows);
  const rows = [];
  let next = 0;
  const push = (s) => {
    glue.pushDeviceMotion(s.timeNs, s.aig[0], s.aig[1], s.aig[2], s.lin !== null, ...(s.lin ?? [0, 0, 0]), s.rate[0], s.rate[1], s.rate[2], s.intervalMs);
    glue.pushDeviceOrientation(s.timeNs, s.orient[0], s.orient[1], s.orient[2]);
    rec.motion({ t: s.timeNs, aig: s.aig, lin: s.lin, rate: s.rate, interval: s.intervalMs });
    rec.orientation(s.orient[0], s.orient[1], s.orient[2], s.timeNs);
  };
  const run = (f) => {
    const atlas = readAtlas(absPath(repoRoot, f.pyramid));
    if (atlas.rows < layout.rows) throw new Error(`atlas of frame ${f.index} has ${atlas.rows} rows < layout ${layout.rows}`);
    // func 647 picks frameTimestampNanos when it is non-zero, else timestampNanos; the engine's time argument is that value (replay_main.cc
    // MakeFrameInput). Sensor rows keep their own receive times.
    const engineTimeNs = f.frameTimeNs !== 0 ? f.frameTimeNs : f.timeNs;
    rec.process(engineTimeNs, f.videoTimeNs, atlas.rgba, 1024, atlas.rows, f.index);
    return glue.processFramePyramid(engineTimeNs, f.videoTimeNs, atlas.rgba, 1024, atlas.rows);
  };
  while (next < bundle.sensors.length && bundle.sensors[next].bootstrap) push(bundle.sensors[next++]);
  run(bundle.frames[0]);  // bootstrap stage: output discarded
  for (const f of bundle.frames.slice(1)) {
    while (next < bundle.sensors.length && bundle.sensors[next].timeNs <= f.timeNs) push(bundle.sensors[next++]);
    if (!f.process) { rows.push({ index: f.index, fresh: false }); continue; }
    const pose = run(f);
    rows.push({ index: f.index, fresh: true, pose: poseRow(pose), raw: { position: pose.position, rotationWxyz: pose.rotationWxyz, status: pose.status,
      reason: pose.reason, repeated: pose.repeated, sourceFrameIndex: pose.sourceFrameIndex, worldPoints: pose.worldPoints } });
    if (onFrame) onFrame(f.index);
  }
  glue.delete();
  return { arm: 'W', layout: { rows: layout.rows, textureSize: layout.textureSize }, glueConfig: canon(cfg), frames: rec.frames, rows };
}

// ===================================================================================================
// C: the chunk in node
// ===================================================================================================
// fake WebGL2 context: records nothing, answers like a context that works; the pixel pack buffer read returns the current atlas.
export function fakeGl(state) {
  const consts = new Map([['NO_ERROR', 0], ['TEXTURE0', 0x84c0]]);
  const constant = (name) => { if (!consts.has(name)) consts.set(name, 0x10000 + consts.size); return consts.get(name); };
  let objects = 0;
  const handlers = {
    getParameter: (p) => (p === constant('VIEWPORT') ? [0, 0, 1, 1] : p === constant('ACTIVE_TEXTURE') ? constant('TEXTURE0') : p === constant('PACK_ALIGNMENT') ? 4
      : p === constant('COLOR_WRITEMASK') ? [true, true, true, true] : p === constant('VERSION') ? 'or1-fake' : null),
    isEnabled: () => false, isContextLost: () => false, getShaderParameter: () => true, getProgramParameter: () => true,
    checkFramebufferStatus: () => constant('FRAMEBUFFER_COMPLETE'), getError: () => 0, getExtension: () => null,
    clientWaitSync: () => constant('ALREADY_SIGNALED'), getUniformLocation: () => ({}),
    getBufferSubData: (_target, _offset, dst) => {
      const atlas = state.atlas;
      if (!atlas) throw new Error('or1 fake GL: no atlas set for this readback');
      if (atlas.length < dst.length) throw new Error(`or1 fake GL: atlas ${atlas.length} B < readback ${dst.length} B`);
      state.readBytes = dst.length;
      dst.set(atlas.subarray(0, dst.length));
    },
  };
  return new Proxy({}, {
    get(_, prop) {
      if (typeof prop !== 'string') return undefined;
      if (/^[A-Z0-9_]+$/.test(prop)) return constant(prop);
      return (...args) => {
        if (handlers[prop]) return handlers[prop](...args);
        if (prop.startsWith('create') || prop === 'fenceSync') return { id: ++objects, kind: prop };
        return undefined;
      };
    },
  });
}

// The chunk's clock reads are `1e3 * performance.now() * 1e3` (timeNanos, sensor receive time) and `videoTime * 1e9`, and the engine binding
// takes int64 by truncation (Ns). Not every integer ns is the exact result of those products for some double, so the fake clock returns a
// double whose product is exactly `ns` when one exists, else one whose product truncates to `ns`; the engine then sees exactly `ns`.
const searchDoubles = (c, ok) => {
  if (ok(c)) return c;
  let lo = c, hi = c;
  for (let i = 0; i < 4096; i++) {
    lo = nextDown(lo); hi = nextUp(hi);
    if (ok(lo)) return lo;
    if (ok(hi)) return hi;
  }
  return null;
};
export function msForNs(ns) {
  const c = ns / 1e6;
  const exact = searchDoubles(c, (x) => 1e3 * x * 1e3 === ns);
  if (exact !== null) return { value: exact, exact: true, truncMatches: true };
  const t = searchDoubles(c, (x) => Math.trunc(1e3 * x * 1e3) === ns);
  return t !== null ? { value: t, exact: false, truncMatches: true } : { value: c, exact: false, truncMatches: false };
}
export function secForNs(ns) {
  const c = ns / 1e9;
  const exact = searchDoubles(c, (x) => x * 1e9 === ns);
  if (exact !== null) return { value: exact, exact: true, truncMatches: true };
  const t = searchDoubles(c, (x) => Math.trunc(x * 1e9) === ns);
  return t !== null ? { value: t, exact: false, truncMatches: true } : { value: c, exact: false, truncMatches: false };
}
const f64 = new Float64Array(1), u64 = new BigInt64Array(f64.buffer);
function nextUp(x) { f64[0] = x; u64[0] += x >= 0 ? 1n : -1n; return f64[0]; }
function nextDown(x) { f64[0] = x; u64[0] += x > 0 ? -1n : 1n; return f64[0]; }

// The rotations the chunk takes (xr-slam.js SUPPORTED_ROTATIONS; PKG-A lifted R16: any capture size, rotation 0 / 90 / 180 / 270 and the negatives). The
// harness passes the bundle's capture size and rotation through unchanged (G4 / OR-1 v4); for a portrait rotation-0 bundle that is what it always did.
export const CHUNK_ROTATIONS = [0, 90, 180, 270, -90, -180, -270];

let chunkInstance = 0;
export async function playChunk({ chunkDir, engineDir, bundle, repoRoot, onFrame }) {
  const cfg = glueConfigOf(bundle);
  if (!CHUNK_ROTATIONS.includes(cfg.rotationDeg)) throw new Error(`the chunk takes rotation 0, 90, 180, 270 and the negatives only (got ${cfg.rotationDeg})`);
  const listeners = new Map();
  const saved = {};
  const set = (k, v) => { if (!(k in saved)) saved[k] = Object.getOwnPropertyDescriptor(globalThis, k); Object.defineProperty(globalThis, k, { value: v, configurable: true, writable: true }); };
  set('window', globalThis);
  set('addEventListener', (n, cb) => { if (!listeners.has(n)) listeners.set(n, []); listeners.get(n).push(cb); });
  set('removeEventListener', (n, cb) => { const l = listeners.get(n) || []; const i = l.indexOf(cb); if (i >= 0) l.splice(i, 1); });
  const dispatch = (e) => { for (const cb of listeners.get(e.type) || []) cb(e); };
  set('_c8', { checkAllZeroes: 0 });
  set('XR8', {
    isPaused: () => false,
    XrDevice: { deviceEstimate: () => ({ manufacturer: cfg.manufacturer, model: cfg.model, os: 'Android', osVersion: '13' }), isDeviceBrowserCompatible: () => true },
    XrConfig: { camera: () => ({ FRONT: 'front', BACK: 'back', ANY: 'any' }), device: () => ({ MOBILE: 'mobile', ANY: 'any' }) },
    XrPermissions: { permissions: () => ({ CAMERA: 'camera', DEVICE_MOTION: 'devicemotion', DEVICE_ORIENTATION: 'deviceorientation' }) },
    drawTexForComputeTex: (t) => ({ drawFor: t }),
  });
  const engineUrlDir = new URL('slam-refact-engine/', pathToFileURL(`${chunkDir}/`)).href;
  set('fetch', async (url) => {
    const href = String(url);
    if (!href.startsWith(engineUrlDir)) throw new Error(`or1: unexpected fetch ${href}`);
    const bytes = fs.readFileSync(path.join(engineDir, href.slice(engineUrlDir.length)));
    return { ok: true, status: 200, arrayBuffer: async () => bytes.buffer.slice(bytes.byteOffset, bytes.byteOffset + bytes.byteLength) };
  });
  set('Blob', class { constructor(parts) { this.parts = parts; } });
  const urlSaved = { c: URL.createObjectURL, r: URL.revokeObjectURL };
  URL.createObjectURL = (blob) => `data:text/javascript;base64,${Buffer.concat(blob.parts.map((p) => Buffer.from(typeof p === 'string' ? p : new Uint8Array(p)))).toString('base64')}`;
  URL.revokeObjectURL = () => {};
  set('require', createRequire(path.join(engineDir, 'slam_refact_engine.js')));
  set('__dirname', engineDir);

  // the recording glue: every call is logged, processFramePyramid returns are kept
  let layoutRows = 0;  // known when the chunk creates its glue
  const rec = newRecorder(() => layoutRows);
  const calls = [];
  const poses = [];
  const state = { atlas: null, readBytes: 0 };
  let currentFrame = null;
  let glueConfig = null;  // what the chunk built its glue from (V-11: recorded, reported next to W's; not part of IN-CW because pyramidModel differs by design, md section 11)
  const createGlue = (Module, config) => {
    glueConfig = canon(config);
    const real = new Module.SlamRefactGlue(config);
    layoutRows = real.pyramidLayout().rows;
    return new Proxy(real, {
      get(target, prop) {
        const value = target[prop];
        if (typeof value !== 'function' || typeof prop !== 'string') return value;
        return (...args) => {
          calls.push(prop);
          if (prop === 'pushDeviceMotionEvent') {
            const [t, e] = args;
            const v3 = (o, k) => (o ? k.map((x) => Number(o[x])) : null);
            rec.motion({ t, aig: v3(e.accelerationIncludingGravity, ['x', 'y', 'z']), lin: v3(e.acceleration, ['x', 'y', 'z']),
              rate: v3(e.rotationRate, ['alpha', 'beta', 'gamma']), interval: Number(e.interval) });
          } else if (prop === 'pushDeviceOrientation') {
            rec.orientation(args[1], args[2], args[3], args[0]);
          } else if (prop === 'processFramePyramid') {
            const [timeNs, videoTimeNs, atlas, size, rows] = args;
            rec.process(timeNs, videoTimeNs, atlas, size, rows, currentFrame?.index ?? null);
          }
          const out = value.apply(target, args);
          if (prop === 'processFramePyramid') poses.push(out);
          return out;
        };
      },
    });
  };

  const clock = { ms: 0 };
  set('performance', { now: () => clock.ms });
  const errors = [];
  globalThis.SLAM_REFACT_CHUNK_OPTIONS = { createGlue };
  let ctrl;
  try {
    const mod = await import(`${pathToFileURL(path.join(chunkDir, 'xr-slam.js')).href}?or1=${++chunkInstance}`);
    ctrl = await mod.XrControllerFactory({ paused: false }, Promise.resolve({}), () => () => {}, (e) => errors.push(e));
  } finally {
    delete globalThis.SLAM_REFACT_CHUNK_OPTIONS;
  }
  const pm = ctrl.pipelineModule();
  const events = [];
  const framework = { dispatchEvent: (name, detail) => events.push(name) };
  const gl = fakeGl(state), drawGl = fakeGl({});
  const W = cfg.captureWidth, H = cfg.captureHeight;
  const cc = {};
  if (cfg.estimateScale) cc.scale = 'absolute';
  if (cfg.disableVio) cc.disableWorldTracking = true;
  if (cfg.leftHandedAxes) cc.leftHandedAxes = true;
  if (cfg.mirroredDisplay) cc.mirroredDisplay = true;
  ctrl.configure({ enableWorldPoints: true, ...cc });
  ctrl.updateCameraProjectionMatrix({ cam: { pixelRectWidth: W, pixelRectHeight: H, nearClipPlane: 0.01, farClipPlane: 1000 },
    origin: { x: cfg.origin[0], y: cfg.origin[1], z: cfg.origin[2] }, facing: { w: cfg.facing[0], x: cfg.facing[1], y: cfg.facing[2], z: cfg.facing[3] } });
  pm.onBeforeSessionInitialize({ sessionAttributes: { fillsCameraTexture: true, controlsCamera: false } });
  pm.onStart({ computeCtx: gl, canvasWidth: W, canvasHeight: H });
  pm.onSessionAttach({ videoWidth: W, videoHeight: H, canvasWidth: W, canvasHeight: H, rotation: cfg.rotationDeg, computeCtx: gl, framework, GLctx: drawGl,
    sessionAttributes: { fillsCameraTexture: true } });
  const inexact = { clock: 0, video: 0, clockLost: 0, videoLost: 0 };  // not exact (engine still sees the same integer) / not even truncating to it
  const setClock = (ns) => { const m = msForNs(ns); if (!m.exact) inexact.clock++; if (!m.truncMatches) inexact.clockLost++; clock.ms = m.value; };
  const dispatchSensor = (s) => {
    setClock(s.timeNs);
    dispatch({ type: 'devicemotion', acceleration: s.lin ? { x: s.lin[0], y: s.lin[1], z: s.lin[2] } : null,
      accelerationIncludingGravity: { x: s.aig[0], y: s.aig[1], z: s.aig[2] }, rotationRate: { alpha: s.rate[0], beta: s.rate[1], gamma: s.rate[2] },
      timeStamp: s.timeNs / 1e6, interval: s.intervalMs });
    dispatch({ type: 'deviceorientation', timeStamp: s.timeNs / 1e6, alpha: s.orient[0], beta: s.orient[1], gamma: s.orient[2] });
  };
  const answers = [];
  const rows = [];
  let pending = null;  // the frame staged by onProcessGpu, processed by the next onProcessCpu (xr.js: pre-render of the next rAF)
  const fsr = (f) => {
    const v = secForNs(f.videoTimeNs); if (!v.exact) inexact.video++; if (!v.truncMatches) inexact.videoLost++;
    // XR8's frameStartResult: videoTime = the video frame time, frameTime = performance.now() / 1000 of the camera loop (camera-loop.ts:324),
    // which is what the original's stageFrame turns into frameTimestampNanos. The bundle records it in column 9.
    // Column 9 = 0: W takes the stage time (column 3) as the engine time, so the chunk must be given NO frameTime (0) and fall back to its own stage time
    // (xr-slam.js `f.frameTime ? ... : f.timeNanos`); passing the video time here (v2) made C differ from W by construction on a bundle without column 9 (K-3).
    const ft = f.frameTimeNs !== 0 ? secForNs(f.frameTimeNs) : { value: 0, exact: true, truncMatches: true };
    if (!ft.exact) inexact.video++; if (!ft.truncMatches) inexact.videoLost++;
    return { computeTexture: { name: f.index }, cameraTexture: { cam: f.index }, textureWidth: W, textureHeight: H, orientation: cfg.rotationDeg, repeatFrame: false,
      videoTime: v.value, frameTime: ft.value };
  };
  const finish = (f) => {
    state.atlas = readAtlas(absPath(repoRoot, f.pyramid)).rgba;
    currentFrame = f;
    const before = poses.length;
    const answer = pm.onProcessCpu({ framework, frameStartResult: fsr(f) });
    if (poses.length > before + 1) throw new Error(`frame ${f.index}: ${poses.length - before} engine responses (expected 1)`);
    if (poses.length === before) {  // the chunk handed nothing to the engine for this frame: a result for the comparators, not a crash of the driver
      answers.push({ index: f.index, answer: null });
      if (!f.bootstrap) rows.push({ index: f.index, fresh: false, noEngineCall: true });
      return;
    }
    const pose = poses.at(-1);
    answers.push({ index: f.index, answer: answer === null ? null : canon({ position: answer.position, rotation: answer.rotation, trackingStatus: answer.trackingStatus,
      trackingReason: answer.trackingReason, worldPoints: answer.worldPoints.length }) });
    if (!f.bootstrap) rows.push({ index: f.index, fresh: true, pose: poseRow(pose), raw: { position: pose.position, rotationWxyz: pose.rotationWxyz, status: pose.status,
      reason: pose.reason, repeated: pose.repeated, sourceFrameIndex: pose.sourceFrameIndex, worldPoints: pose.worldPoints } });
    if (onFrame && !f.bootstrap) onFrame(f.index);
  };
  const stage = (f) => {
    setClock(f.timeNs);
    pm.onProcessGpu({ framework, frameStartResult: fsr(f) });
    pending = f;
  };
  try {
    let next = 0;
    const boot = bundle.frames[0];
    while (next < bundle.sensors.length && bundle.sensors[next].bootstrap) dispatchSensor(bundle.sensors[next++]);
    stage(boot);
    for (const f of bundle.frames.slice(1)) {
      while (next < bundle.sensors.length && bundle.sensors[next].timeNs <= f.timeNs) dispatchSensor(bundle.sensors[next++]);
      if (!f.process) { rows.push({ index: f.index, fresh: false }); continue; }
      finish(pending);  // pre-render of this rAF: the previously staged frame is read and processed
      stage(f);         // post-render: this frame is staged
    }
    finish(pending);
  } finally {
    pm.onSessionDetach({});
    pm.onDetach({});
    for (const [k, d] of Object.entries(saved)) { if (d) Object.defineProperty(globalThis, k, d); else delete globalThis[k]; }
    URL.createObjectURL = urlSaved.c; URL.revokeObjectURL = urlSaved.r;
  }
  rows.sort((a, b) => a.index - b.index);  // the rows of dropped frames were added while later frames were staged
  return { arm: 'C', layout: { rows: layoutRows }, glueConfig, frames: rec.frames, rows, answers, calls, errors, events, inexact };
}

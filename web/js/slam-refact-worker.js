// Engine worker of the slam_refact measurement page (module Worker).
// Loads /slam-refact/slam_refact_engine.{js,wasm}, checks their sha256 against the deployed
// manifest, and runs SlamRefactGlue (US-010 API) one frame at a time. All timestamps it receives
// were taken on the page's main thread (performance.now() there), so the engine sees one clock.
// Worker-side times are reported as absolute ms (performance.timeOrigin + performance.now()).
//
// Frontend (config.frontend):
//   'gpu'  (default) the original reference runtime GPU pyramid: the shaders embedded in the original
//          xr-slam.wasm run on a WebGL2 OffscreenCanvas here (slam-refact-gr8gl.js) and the
//          readback atlas is injected into the engine (processFramePyramid, H7). Falls back to
//          'cpu' when WebGL2 is unavailable (reported, never silent).
//   'cpu'  the engine's CPU port of the same shaders on the RGBA frame (cpuPyramidAtlas with the
//          configured pyramidModel, injected like the GPU atlas), so both frontends see the colour
//          frame as the original's camera texture; much slower.
//   gpuCheck: also build the CPU port atlas of sampled frames and compare it byte for byte with
//          the GPU atlas (diagnostic; costs a CPU pyramid per checked frame).
import { Gr8GlPyramid } from './slam-refact-gr8gl.js';
import { compareAtlases } from './slam-refact-core.js';

let glue = null;
let Module = null;
let fatal = null;
let gpu = null;          // Gr8GlPyramid when the GPU frontend is active
let layout = null;
let frontend = { requested: 'cpu', active: 'cpu', error: null, renderer: null };
let check = null;        // { every, left, canvas, ctx } for gpuCheck
let cpuModel = 'ieee';   // CPU-port numeric model of the 'cpu' frontend (config.pyramidModel)
let debugFatalLeft = 0;  // ?debug-fatal-after (diagnostic): frames until an injected fatal
let detectSurfaces = false;  // config.detectSurfaces (engine ACC-3)
let surfaceTick = 0;

const absNow = () => performance.timeOrigin + performance.now();

async function sha256Hex(buf) {
  const d = new Uint8Array(await crypto.subtle.digest('SHA-256', buf));
  return Array.from(d, (b) => b.toString(16).padStart(2, '0')).join('');
}

async function fetchBytes(url) {
  const r = await fetch(url, { cache: 'no-store', credentials: 'same-origin' });
  if (!r.ok) throw new Error(`${url}: HTTP ${r.status}`);
  return r.arrayBuffer();
}

// The exact bytes that were hashed are the ones executed (blob module + wasmBinary). The engine
// JS is an Emscripten MODULARIZE script; the only bytes added are a fixed export line.
async function loadEngine(msg) {
  const [jsBuf, wasmBuf] = await Promise.all([fetchBytes(msg.engineJsUrl), fetchBytes(msg.engineWasmUrl)]);
  const loaded = { jsSha256: await sha256Hex(jsBuf), wasmSha256: await sha256Hex(wasmBuf),
    jsBytes: jsBuf.byteLength, wasmBytes: wasmBuf.byteLength };
  const expected = msg.expected ?? {};
  loaded.matchesManifest = loaded.jsSha256 === expected.jsSha256 && loaded.wasmSha256 === expected.wasmSha256;
  if (!loaded.matchesManifest) {
    throw Object.assign(new Error('engine bytes do not match web/slam-refact/manifest.json'), { loaded });
  }
  const blobUrl = URL.createObjectURL(new Blob([jsBuf, '\nexport default SlamRefactEngine;\n'], { type: 'text/javascript' }));
  let factory;
  try { factory = (await import(blobUrl)).default; } finally { URL.revokeObjectURL(blobUrl); }
  const t0 = performance.now();
  Module = await factory({ wasmBinary: new Uint8Array(wasmBuf),
    print: (s) => self.postMessage({ type: 'log', level: 'info', text: String(s) }),
    printErr: (s) => self.postMessage({ type: 'log', level: 'warn', text: String(s) }) });
  loaded.instantiateMs = performance.now() - t0;
  return loaded;
}

function setupFrontend(config) {
  if (gpu) { gpu.dispose(); gpu = null; }
  frontend = { requested: config.frontend === 'cpu' ? 'cpu' : 'gpu', active: 'cpu', error: null, renderer: null };
  layout = glue.pyramidLayout();
  cpuModel = config.pyramidModel === 'swiftshader-approx' ? 'swiftshader-approx' : 'ieee';
  debugFatalLeft = Math.max(0, config.debugFatalAfterFrames | 0);
  check = null;
  if (frontend.requested !== 'gpu') return;
  try {
    if (typeof OffscreenCanvas === 'undefined') throw new Error('OffscreenCanvas unavailable in Worker');
    const gl = new OffscreenCanvas(1, 1).getContext('webgl2', { antialias: false, depth: false, stencil: false,
      alpha: true, premultipliedAlpha: false, preserveDrawingBuffer: false, powerPreference: 'high-performance' });
    if (!gl) throw new Error('WebGL2 unavailable in Worker');
    const dbg = gl.getExtension('WEBGL_debug_renderer_info');
    frontend.renderer = { renderer: gl.getParameter(dbg ? dbg.UNMASKED_RENDERER_WEBGL : gl.RENDERER),
      vendor: gl.getParameter(dbg ? dbg.UNMASKED_VENDOR_WEBGL : gl.VENDOR), version: gl.getParameter(gl.VERSION),
      fragmentMediumpFloat: precisionOf(gl, gl.FRAGMENT_SHADER, gl.MEDIUM_FLOAT) };
    gpu = new Gr8GlPyramid(gl, layout, { cameraWidth: config.captureWidth, cameraHeight: config.captureHeight, rotation: config.rotationDeg });
    frontend.active = 'gpu';
    if (config.gpuCheck) {
      const canvas = new OffscreenCanvas(config.captureWidth, config.captureHeight);
      check = { every: Math.max(1, config.gpuCheck.every | 0 || 10), left: Math.max(0, config.gpuCheck.frames | 0 || 30),
        canvas, ctx: canvas.getContext('2d', { willReadFrequently: true }), n: 0 };
    }
  } catch (e) {
    if (gpu) { gpu.dispose(); gpu = null; }
    frontend.error = String(e && e.message || e);
  }
}

function precisionOf(gl, shader, type) {
  const p = gl.getShaderPrecisionFormat(shader, type);
  return p ? { rangeMin: p.rangeMin, rangeMax: p.rangeMax, precision: p.precision } : null;
}

function createGlue(config) {
  if (glue) { glue.delete(); glue = null; }
  glue = new Module.SlamRefactGlue(config);
  // An engine built before the accuracy mode (contract < 3) has no surface API; the page then runs
  // without surfaces instead of failing.
  detectSurfaces = !!config.detectSurfaces && typeof glue.getSurfaces === 'function';
  surfaceTick = 0;
  setupFrontend(config);
}

// Sensor rows in arrival order: ['m', tNs, gx, gy, gz, hasAcc, ax, ay, az, ra, rb, rg, intervalMs]
// (DeviceMotion, W3C units) or ['o', tNs, alpha, beta, gamma] (DeviceOrientation, degrees).
function pushSensors(rows) {
  let motion = 0, orientation = 0;
  for (const s of rows) {
    if (s[0] === 'm') {
      glue.pushDeviceMotion(s[1], s[2], s[3], s[4], s[5], s[6], s[7], s[8], s[9], s[10], s[11], s[12]);
      motion++;
    } else if (s[0] === 'o') {
      glue.pushDeviceOrientation(s[1], s[2], s[3], s[4]);
      orientation++;
    }
  }
  return { motion, orientation };
}

// GPU vs CPU-port atlas of one frame (both models), over the engine's atlas rows.
function gpuCheckFrame(bitmapLike, atlas) {
  check.ctx.drawImage(bitmapLike, 0, 0);
  const rgba = check.ctx.getImageData(0, 0, check.canvas.width, check.canvas.height).data;
  const out = {};
  for (const model of ['swiftshader-approx', 'ieee']) {
    const cpu = Module.SlamRefactGlue.cpuPyramidAtlas(rgba, check.canvas.width, check.canvas.height, model);
    out[model] = compareAtlases(atlas, cpu.rgba, layout);
  }
  return out;
}

function onFrame(msg) {
  try {
    return processFrame(msg);
  } finally {
    if (msg.bitmap) msg.bitmap.close();   // every path, including a throw (no-op if already closed)
  }
}

function processFrame(msg) {
  if (debugFatalLeft > 0 && --debugFatalLeft === 0) throw new Error('injected fatal (diagnostic ?debug-fatal-after)');
  const recvAbsMs = absNow();
  const sensors = pushSensors(msg.sensors);
  const t = { };
  let pose;
  let gpuCheck = null;
  const t0 = performance.now();
  if (gpu && msg.bitmap) {
    gpu.upload(msg.bitmap);
    const t1 = performance.now();
    gpu.draw();
    const t2 = performance.now();
    const atlas = gpu.read();
    const t3 = performance.now();
    t.uploadMs = t1 - t0; t.drawMs = t2 - t1; t.readMs = t3 - t2;
    if (check && check.left > 0 && check.n++ % check.every === 0) {
      check.left--;
      gpuCheck = gpuCheckFrame(msg.bitmap, atlas);
    }
    const t4 = performance.now();
    pose = glue.processFramePyramid(msg.timeNs, msg.videoTimeNs, atlas, layout.textureSize, layout.rows);
    t.engineMs = performance.now() - t4;
  } else {
    // The page sends RGBA bytes whenever the GPU frontend is not active (captureMode 'rgba').
    if (!msg.rgba) throw new Error('CPU frontend frame without RGBA bytes');
    const rgba = new Uint8Array(msg.rgba);
    const cpu = Module.SlamRefactGlue.cpuPyramidAtlas(rgba, msg.width, msg.height, cpuModel);
    const t1 = performance.now();
    t.cpuPyramidMs = t1 - t0;
    pose = glue.processFramePyramid(msg.timeNs, msg.videoTimeNs, cpu.rgba, cpu.size, cpu.rows);
    t.engineMs = performance.now() - t1;
  }
  const t5 = performance.now();
  // World points of a fresh response for the 3D view (display only; [id, conf, x, y, z] * n).
  const worldPoints = pose.repeated ? null : glue.getWorldPoints();
  // Floor / horizontal surfaces (engine ACC-3), every third fresh response.
  const surfaces = detectSurfaces && !pose.repeated && surfaceTick++ % 3 === 0 ? glue.getSurfaces() : null;
  t.worldPointsMs = performance.now() - t5;
  const transfer = [];
  if (worldPoints) transfer.push(worldPoints.buffer);
  if (msg.rgba) transfer.push(msg.rgba);
  self.postMessage({ type: 'result', id: msg.id, pose, sensors, ...t, frontend: gpu ? 'gpu' : 'cpu', gpuCheck,
    worldPoints, surfaces, recvAbsMs, doneAbsMs: absNow(), rgba: msg.rgba }, transfer);
}

function fail(e, id) {
  fatal = String(e && e.stack || e);
  self.postMessage({ type: 'fatal', id, error: fatal, loaded: e && e.loaded });
}

self.onmessage = async (ev) => {
  const msg = ev.data;
  if (fatal && msg.type !== 'init') {
    if (msg.type === 'frame') {
      if (msg.bitmap) msg.bitmap.close();
      self.postMessage({ type: 'fatal', id: msg.id, error: fatal, rgba: msg.rgba }, msg.rgba ? [msg.rgba] : []);
    }
    return;
  }
  try {
    if (msg.type === 'init') {
      const loaded = Module ? null : await loadEngine(msg);
      createGlue(msg.config);
      self.postMessage({ type: 'ready', loaded, frontend, layout,
        engineFeatures: { surfaces: typeof glue.getSurfaces === 'function', hitTest: typeof glue.hitTest === 'function' } });
    } else if (msg.type === 'frame') {
      if (!glue) throw new Error('frame before the engine is ready');
      onFrame(msg);
    } else if (msg.type === 'hitTest') {
      // u, v in [0, 1] of the 480x640 engine input; mask = OR of FEATURE_POINT 1,
      // ESTIMATED_SURFACE 2, DETECTED_SURFACE 4.
      const hits = glue && typeof glue.hitTest === 'function' ? glue.hitTest(msg.u, msg.v, msg.mask | 0) : [];
      self.postMessage({ type: 'hit', id: msg.id, u: msg.u, v: msg.v, hits });
    } else if (msg.type === 'reset') {
      glue.reset();
      self.postMessage({ type: 'resetDone' });
    } else if (msg.type === 'dispose') {
      if (gpu) { gpu.dispose(); gpu = null; }
      if (glue) { glue.delete(); glue = null; }
      self.postMessage({ type: 'disposed' });
    }
  } catch (e) {
    fail(e, msg.id);
  }
};

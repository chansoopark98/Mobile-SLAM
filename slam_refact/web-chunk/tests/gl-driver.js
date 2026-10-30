// Page-side driver of run-chunk-gl.mjs (RM-4c gate G-GL). Served as /driver.js; the chunk files are served under /chunk/ and the
// engine under /chunk/slam-refact-engine/. Nothing here touches the network beyond the ephemeral 127.0.0.1 server.
import { loadEngine } from '/chunk/engine-loader.js';
import { Gr8ComputePyramid } from '/chunk/gr8-compute-pyramid.js';
import { readLuminanceFromImage } from '/chunk/image-targets.js';

const b64ToBytes = (s) => Uint8Array.from(atob(s), (c) => c.charCodeAt(0));
const bytesToB64 = (u8) => {
  let s = '';
  for (let i = 0; i < u8.length; i += 0x8000) s += String.fromCharCode.apply(null, u8.subarray(i, i + 0x8000));
  return btoa(s);
};

function makeGl(width, height) {
  const canvas = document.createElement('canvas');
  canvas.width = width;
  canvas.height = height;
  const gl = canvas.getContext('webgl2', { antialias: false, preserveDrawingBuffer: true });
  if (!gl) throw new Error('no WebGL2 context');
  return gl;
}

// XR8's compute camera texture: RGBA, bilinear, clamped (the CPU pyramid model assumes bilinear sampling of it).
function uploadGray(gl, tex, gray, w, h) {
  const rgba = new Uint8Array(w * h * 4);
  for (let i = 0; i < w * h; i++) { rgba[i * 4] = gray[i]; rgba[i * 4 + 1] = gray[i]; rgba[i * 4 + 2] = gray[i]; rgba[i * 4 + 3] = 255; }
  gl.activeTexture(gl.TEXTURE0);
  gl.bindTexture(gl.TEXTURE_2D, tex);
  gl.texImage2D(gl.TEXTURE_2D, 0, gl.RGBA, w, h, 0, gl.RGBA, gl.UNSIGNED_BYTE, rgba);
  gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MIN_FILTER, gl.LINEAR);
  gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MAG_FILTER, gl.LINEAR);
  gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_WRAP_S, gl.CLAMP_TO_EDGE);
  gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_WRAP_T, gl.CLAMP_TO_EDGE);
}

function parsePgm(bytes) {
  const text = new TextDecoder('latin1').decode(bytes.subarray(0, 64));
  const m = /^P5\s+(\d+)\s+(\d+)\s+255\s/.exec(text);
  if (!m) throw new Error('not a P5 PGM');
  const w = Number(m[1]), h = Number(m[2]);
  return { w, h, gray: bytes.subarray(bytes.length - w * h) };
}

const slotPlane = (atlas, size, l) => {
  const out = new Uint8Array(l.w * l.h);
  for (let y = 0; y < l.h; y++) for (let x = 0; x < l.w; x++) out[y * l.w + x] = atlas[((l.r + y) * size + l.c + x) * 4];
  return out;
};

// G-GL: the real Gr8ComputePyramid on a real WebGL2 context draws the ROI slots of the oracle's warps.
window.runRoi = async (job) => {
  const { Module } = await loadEngine(new URL('/chunk/slam-refact-engine/', location.href));
  const g0 = new Module.SlamRefactGlue({ captureWidth: job.width, captureHeight: job.height, manufacturer: 'Google', model: 'Pixel 7' });
  const layout = g0.pyramidLayout();
  g0.delete();
  const gl = makeGl(64, 64);
  const dbg = gl.getExtension('WEBGL_debug_renderer_info');
  const renderer = gl.getParameter(dbg ? dbg.UNMASKED_RENDERER_WEBGL : gl.RENDERER);
  const tex = gl.createTexture();
  uploadGray(gl, tex, b64ToBytes(job.camera_b64), job.width, job.height);
  const pyramid = new Gr8ComputePyramid(gl, layout, { cameraWidth: job.width, cameraHeight: job.height });
  const rois = job.warps.map((w, i) => ({ slot: i, source: 'IMAGE_TARGET', name: 'x', warp: Float32Array.from(w) }));
  pyramid.stage(tex, rois);
  const atlas = pyramid.read();
  const a = layout.roiSlots.map((l, i) => ({ slot: i, w: l.w, h: l.h, r_b64: bytesToB64(slotPlane(atlas, layout.textureSize, l)) }));
  // A4: the same GL state (the slots keep what was drawn), then only two ROIs: slots 2 and 3 must be cleared again.
  pyramid.stage(tex, rois);
  pyramid.read();
  pyramid.stage(tex, rois.slice(0, 2));
  const atlas2 = pyramid.read();
  const a4 = [2, 3].map((i) => ({ slot: i, w: layout.roiSlots[i].w, h: layout.roiSlots[i].h, r_b64: bytesToB64(slotPlane(atlas2, layout.textureSize, layout.roiSlots[i])) }));
  const result = { renderer, layout: { textureSize: layout.textureSize, rows: layout.rows, roiAtlasRows: layout.roiAtlasRows, roiSlots: layout.roiSlots }, a, a4,
    stats: pyramid.stats };
  pyramid.dispose();
  return result;
};

const fakeXr8 = () => ({
  isPaused: () => false,
  XrDevice: { deviceEstimate: () => ({ manufacturer: 'Google', model: 'Pixel 7', os: 'Android', osVersion: '13' }), isDeviceBrowserCompatible: () => true },
  XrConfig: { camera: () => ({ FRONT: 'front', BACK: 'back', ANY: 'any' }), device: () => ({ MOBILE: 'mobile', MOBILE_AND_HEADSETS: 'mobile-and-headsets', ANY: 'any' }) },
  XrPermissions: { permissions: () => ({ CAMERA: 'camera', DEVICE_MOTION: 'devicemotion', DEVICE_ORIENTATION: 'deviceorientation', DEVICE_GPS: 'geolocation', MICROPHONE: 'microphone' }) },
  drawTexForComputeTex: (t) => ({ drawFor: true, t }),
});

const summarise = (r) => (r ? { status: r.trackingStatus, reason: r.trackingReason,
  images: r.detectedImages.map((i) => ({ name: i.name, position: i.position, rotation: i.rotation, scale: i.scale, type: i.type })) } : null);

// RM-4e GL-E6 probe (gate v4): the ROI wiring of the chunk path is measured here, outside the chunk, so a chunk that hands too few ROIs to
// the pyramid (or draws fewer) cannot be hidden by its own counters. Per Gr8ComputePyramid.stage call it records the list the engine's
// getRequestedRois() returned immediately before (a forwarding Proxy of the real glue, options.createGlue), the list stage() received, and the
// number of drawElements calls the compute context saw during the stage. The engine and the pyramid themselves are the real ones.
const roiSig = (r) => ({ slot: r.slot, source: r.source, name: r.name, warp: Array.from(r.warp ?? []) });

function wiringProbe(gl) {
  const stages = [];
  let lastAsked = null, counting = null, askedCalls = 0, nSlots = 0;
  const hadOwnDraw = Object.prototype.hasOwnProperty.call(gl, 'drawElements');
  const drawElements = gl.drawElements;
  gl.drawElements = function (...args) { if (counting) counting.draws++; return drawElements.apply(this, args); };
  const stageOriginal = Gr8ComputePyramid.prototype.stage;
  Gr8ComputePyramid.prototype.stage = function (cameraTexture, rois = []) {
    const entry = { asked: lastAsked, passed: rois.map(roiSig), draws: 0 };
    lastAsked = null;  // an ask is consumed by one stage; a stage without an ask of its own shows up as asked: null
    counting = entry;
    nSlots = this.layout.roiSlots.length;
    try { return stageOriginal.call(this, cameraTexture, rois); } finally { counting = null; stages.push(entry); }
  };
  const createGlue = (Module, config) => new Proxy(new Module.SlamRefactGlue(config), {
    get(target, prop) {
      const value = target[prop];
      if (prop === 'getRequestedRois') return (...args) => { const r = value.apply(target, args); askedCalls++; lastAsked = r.map(roiSig); return r; };
      return typeof value === 'function' ? value.bind(target) : value;
    } });
  return { stages, createGlue, askedCalls: () => askedCalls, nSlots: () => nSlots,
    restore() { Gr8ComputePyramid.prototype.stage = stageOriginal; if (hadOwnDraw) gl.drawElements = drawElements; else delete gl.drawElements; } };
}

// Chunk path: the real XrControllerFactory + Gr8ComputePyramid + engine on SwiftShader, 480x640 synthetic frames, image target
// loaded through the default loader (<img> + texImage2D + readback). Returns per-frame answers, events and the chunk diagnostics.
async function chunkPath(job, gl, drawGl, frames, fetchBytes) {
  window.XR8 = fakeXr8();
  window._c8 = { checkAllZeroes: 0 };
  const probe = wiringProbe(gl);
  window.SLAM_REFACT_CHUNK_OPTIONS = { roiCheck: { every: 1, frames: job.roiCheckFrames ?? 40 }, createGlue: probe.createGlue };
  const slam = await import('/chunk/xr-slam.js');
  slam.ResourceUrls.setBaseUrl('/chunk/');
  const errors = [];
  const ctrl = await slam.XrControllerFactory({ paused: false }, Promise.resolve({}), () => () => {}, (e) => errors.push(String(e && e.message || e)));
  delete window.SLAM_REFACT_CHUNK_OPTIONS;
  const pm = ctrl.pipelineModule();
  const events = [];
  let k = -1;
  const framework = { dispatchEvent: (name, detail) => events.push({ k, name, detail: JSON.parse(JSON.stringify(detail ?? null)),
    source: window.__slamRefactChunk.stats.lastSourceFrameIndex }) };
  ctrl.configure({ disableWorldTracking: true, enableWorldPoints: false, ...(job.imageTargetData ? { imageTargetData: job.imageTargetData } : {}) });
  ctrl.updateCameraProjectionMatrix({ cam: { pixelRectWidth: job.width, pixelRectHeight: job.height, nearClipPlane: 0.01, farClipPlane: 1000 },
    origin: { x: 0, y: 2, z: 0 }, facing: { w: 1, x: 0, y: 0, z: 0 } });
  pm.onBeforeSessionInitialize({ sessionAttributes: { fillsCameraTexture: true, controlsCamera: false } });
  pm.onStart({ computeCtx: gl, canvasWidth: job.width, canvasHeight: job.height });
  pm.onSessionAttach({ videoWidth: job.width, videoHeight: job.height, canvasWidth: job.width, canvasHeight: job.height, rotation: 0, computeCtx: gl, framework,
    GLctx: drawGl, sessionAttributes: { fillsCameraTexture: true } });
  pm.onAppResourcesLoaded({ imageTargets: undefined, framework });
  await window.__slamRefactChunk.imageTargets.whenIdle();
  const tex = gl.createTexture();
  const answers = [];
  for (let i = 0; i < frames.length; i++) {
    k = i;
    uploadGray(gl, tex, frames[i], job.width, job.height);
    const fsr = { computeTexture: tex, cameraTexture: tex, textureWidth: job.width, textureHeight: job.height, orientation: 0, repeatFrame: false,
      videoTime: 1 + i / 30, frameTime: 1 + i / 30 };
    answers.push(summarise(pm.onProcessCpu({ framework, frameStartResult: fsr })));
    pm.onProcessGpu({ framework, frameStartResult: fsr });
    answers.at(-1) && (answers.at(-1).source = window.__slamRefactChunk.stats.lastSourceFrameIndex);
  }
  const diag = window.__slamRefactChunk;
  const roiCheck = diag.roiCheck.map((s) => ({ stageIndex: s.stageIndex, rois: s.rois,
    slots: s.slots.map((x) => ({ slot: x.slot, w: x.w, h: x.h, r_b64: bytesToB64(Uint8Array.from(x.r)) })) }));
  const out = { errors, events, answers, roiCheck, stages: probe.stages, nSlots: probe.nSlots(), askedCalls: probe.askedCalls(),
    stats: { ...diag.stats, stageMs: undefined, readMs: undefined, engineMs: undefined },
    readback: diag.session && diag.session.readback, warnings: diag.warnings, engine: diag.engine, imageTargets: { loadErrors: diag.imageTargets.loadErrors, loaded: diag.imageTargets.loadedNames() } };
  pm.onSessionDetach({});
  pm.onDetach({});
  probe.restore();
  return out;
}

// Engine CPU path of the same frames (processFrame, swiftshader-approx pyramid, same targets): the reference of GL-E2.
async function cpuPath(job, frames, targets) {
  const { Module } = await loadEngine(new URL('/chunk/slam-refact-engine/', location.href));
  const glue = new Module.SlamRefactGlue({ captureWidth: job.width, captureHeight: job.height, manufacturer: 'Google', model: 'Pixel 7', disableVio: true,
    pyramidModel: 'swiftshader-approx' });
  if (targets.length) glue.setImageTargets(targets);
  const events = [];
  const detections = [];  // RM-4d GL-E5: the detections of every fresh response, by source frame (the chunk path's counterpart)
  for (let i = 0; i < frames.length; i++) {
    const t = 1e9 + Math.round(i * 1e9 / 30);
    const p = glue.processFrame(t, t, frames[i], job.width, job.height);
    for (const e of p.imageEvents) events.push({ k: i, source: p.sourceFrameIndex, name: e.name, image: e.detail.name });
    if (!p.repeated) {
      detections.push({ k: i, source: p.sourceFrameIndex,
        images: p.detectedImages.map((d) => ({ name: d.name, position: d.position, rotation: d.rotation, scale: d.scale, type: d.type })) });
    }
  }
  glue.delete();
  return { events, detections };
}

window.runIt = async (job) => {
  const fetchBytes = async (url) => new Uint8Array(await (await fetch(url)).arrayBuffer());
  const frames = [];
  for (let i = 0; i < job.frameCount; i++) frames.push(parsePgm(await fetchBytes(`/data/frames/${String(i).padStart(4, '0')}.pgm`)).gray);
  const gl = makeGl(64, 64), drawGl = makeGl(64, 64);
  const dbg = gl.getExtension('WEBGL_debug_renderer_info');
  const result = { renderer: gl.getParameter(dbg ? dbg.UNMASKED_RENDERER_WEBGL : gl.RENDERER) };
  const lum = parsePgm(await fetchBytes('/data/synth-flat_luminance.pgm'));
  if (job.imageTargetData) {
    // the default loader path on the PNG that was made from the PGM: lossless gray
    const img = await new Promise((resolve, reject) => { const el = document.createElement('img'); el.onload = () => resolve(el); el.onerror = reject; el.setAttribute('crossorigin', 'anonymous'); el.setAttribute('src', job.imageTargetData[0].imagePath); });
    const read = readLuminanceFromImage(gl, img);
    result.pngRoundTrip = { width: read.width, height: read.height, equal: read.width === lum.w && read.height === lum.h && read.luminance.every((v, i) => v === lum.gray[i]) };
  }
  result.chunk = await chunkPath(job, gl, drawGl, frames, fetchBytes);
  const targets = job.imageTargetData ? [{ json: JSON.stringify(job.imageTargetData[0]), luminance: lum.gray, width: lum.w, height: lum.h }] : [];
  result.cpu = await cpuPath(job, frames, targets);
  return result;
};

window.__driverReady = true;

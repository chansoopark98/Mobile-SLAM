// Main thread of the slam_refact measurement page (web/slam-refact.html).
//
// Nothing is acquired before the user presses Start. Inputs follow the original reference runtime page
// (slam_refact/docs/spec/01-frontend.md §3.1, S1 §0.7): DeviceMotion/DeviceOrientation events are
// queued with the handler receive time performance.now() * 1e6 ns (not event.timeStamp) in W3C
// units, untouched (the engine glue applies func 647); a frame's time is performance.now() at the
// camera read. All of those times are taken on this thread and sent to the engine worker with the
// frame, so the worker's own clock is never mixed in. Latest frame wins: a video frame that arrives
// while the worker is busy is dropped and counted; queued sensor events stay queued for the next
// frame (the glue takes everything queued at stage time).
//
// Privacy: no network writes. By default video, raw IMU and images are never stored or sent; the
// metrics JSON (pose/status/timing, sensor counts and rates only) is created only when the user
// presses Download. Exception, explicit opt-in only: with ?record=1 (red REC badge and banner) the
// frames posted to the engine and their sensor rows are stored in this device's OPFS
// (slam-refact-recorder.js) for PC replay; they leave the device only through the user's click on
// "녹화 다운로드" (or a CDP pull by the operator), never over the network.
import { CAPTURE_HEIGHT, CAPTURE_WIDTH, HIT_TYPES, backCameraNumber, cameraRequestFor, capturePlan, clientHintModel,
  configForCamera, engineConfigFor, originOf, pickBackCamera, videoRequestsFor, percentiles, summarize } from './slam-refact-core.js';
import { TrajectoryView3D } from './slam-refact-view3d.js';
import { InputRecorder, recordingApi } from './slam-refact-recorder.js';

const $ = (id) => document.getElementById(id);
const ui = { start: $('start'), stop: $('stop'), reset: $('reset'), download: $('download'),
  video: $('video'), traj: $('traj'), status: $('status'), state: $('state'), pose: $('pose'),
  timing: $('timing'), notes: $('notes'), engine: $('engine'), view3dBox: $('view3dBox'),
  view3d: $('view3d'), view3dStats: $('view3dStats'), viewOrbit: $('viewOrbit'), viewFollow: $('viewFollow'),
  viewTop: $('viewTop'), viewFit: $('viewFit'), preview: $('preview'), tracker: $('tracker'),
  accuracy: $('accuracy'), hit: $('hit'), downloadRec: $('downloadRec'), recBadge: $('recBadge'),
  recBanner: $('recBanner') };

const MAX_RECORDS = 60000;          // about 33 min at 30 fps
const MAX_PENDING_SENSORS = 4000;   // the glue clears events older than 5 s anyway

const search = location.search;
// Rebuilt in boot() with the Client Hints model (e.g. SM-F741N) when the browser provides it.
let baseConfig = engineConfigFor(search, navigator.userAgent);
// Per run: baseConfig plus the selected camera's focal (configForCamera in start()).
let config = baseConfig;
const debugFatalAfter = Math.max(0, Number(new URLSearchParams(search).get('debug-fatal-after')) | 0);
const origin = originOf(config);

let state = 'idle';
let manifest = null;
let loaded = null;
let worker = null;
let workerReady = false;
let stream = null;
let plan = null;
let canvas = null, ctx = null;
let inflight = null;
let nextId = 0;
let pendingSensors = [];
let records = [];
let resets = [];
let wakeLock = null;
let startedAtMs = null, stoppedAtMs = null;
let lastError = null;
const notes = new Set();
const counters = { videoFrames: 0, droppedBusy: 0, posted: 0, results: 0, fresh: 0, repeated: 0,
  motionEvents: 0, motionWithAcceleration: 0, motionIncomplete: 0, orientationEvents: 0,
  orientationIncomplete: 0, sensorQueueOverflow: 0, recordsTruncated: 0, resets: 0 };
const sensorTimes = { motion: [], orientation: [], interval: [] };   // receive ms (bounded) for rates
let readyResolve = null;
let recorder = null;         // InputRecorder while ?record=1 (opt-in), else null
let recSummary = null;       // recording summary after Stop (files, frames, bytes, sha256 of the meta JSON)
let recUnposted = [];        // sensor rows queued at Stop that were never posted with a frame
let frontendInfo = null;     // worker frontend { requested, active, error, renderer }
let pyramidLayout = null;    // engine pyramid layout (worker 'ready')
const gpuChecks = [];        // GPU vs CPU-port atlas comparisons (?gpu-check=1)
const engineLog = { count: 0, lines: [] };   // last 200 engine print/printErr lines
let view3d = null;           // TrajectoryView3D, or null (then the 2D top-down canvas is drawn)
let lastSurfaces = null;     // engine ACC-3 getSurfaces() (worker, every third fresh response)
let lastWorldPointsInfo = null;  // diagnostic: entries / shown of the last world point list
let engineFeatures = null;   // worker 'ready': which accuracy-mode APIs the loaded engine has
const hits = [];             // hit tests requested by tapping the engine input preview
let hitId = 0;
let lastView3dRender = 0;
const VIEW3D_INTERVAL_MS = 66;   // ~15 Hz: the view shares the main thread with capture

function setState(s, detail = '') {
  state = s;
  ui.start.disabled = !((workerReady || !worker) && (s === 'idle' || s === 'stopped' || s === 'error')) || !!recorder?.finalizing;
  ui.stop.disabled = s !== 'running';
  ui.reset.disabled = s !== 'running';
  ui.download.disabled = records.length === 0;
  ui.downloadRec.disabled = !(recSummary && recSummary.ok) || !!recorder?.finalizing;
  ui.state.textContent = `state: ${s}${detail ? ` — ${detail}` : ''}`;
}

function note(text) { notes.add(text); ui.notes.textContent = [...notes].join('\n'); }

function fail(text) {
  lastError = text;
  ui.state.classList.add('err');
  setState('error', text);
}

// ---- engine worker -------------------------------------------------------------------------

let workerGeneration = 0;   // workers created so far (a fatal drops the current one)

function startWorker() {
  workerGeneration++;
  worker = new Worker('js/slam-refact-worker.js', { type: 'module' });
  worker.onmessage = (ev) => onWorkerMessage(ev.data);
  worker.onerror = (ev) => { dropWorker(); stopCapture(); endRecording(); fail(`worker error: ${ev.message}; press Start to reload the engine`); };
}

// After an engine fatal (an SR_CHECK abort or a JS error) the module state is not trusted: the
// worker is dropped and the next Start creates a new one (bytes verified again).
function dropWorker() {
  if (worker) worker.terminate();
  worker = null;
  workerReady = false;
  inflight = null;
  const r = readyResolve; readyResolve = null;
  if (r) r(false);
}

async function loadManifest() {
  const r = await fetch('slam-refact/manifest.json', { cache: 'no-store' });
  if (!r.ok) throw new Error(`manifest HTTP ${r.status}`);
  manifest = await r.json();
  showEngine();
}

// Resolves true when the worker reports ready, false when it was dropped (fatal). A missing
// manifest (failed boot load) is fetched first, before any worker exists, so a retry by Start works.
async function initEngine() {
  if (!manifest) await loadManifest();
  if (!worker) startWorker();
  workerReady = false;
  return new Promise((resolve) => {
    readyResolve = resolve;
    // ?debug-fatal-after=N (diagnostic, smoke only): the first worker fails after N frames so the
    // fatal-recovery path can be exercised; later workers never inject.
    const debugFatalAfterFrames = workerGeneration === 1 ? debugFatalAfter : 0;
    worker.postMessage({ type: 'init', config: { ...config, debugFatalAfterFrames }, engineJsUrl: new URL('slam-refact/slam_refact_engine.js', location.href).href,
      engineWasmUrl: new URL('slam-refact/slam_refact_engine.wasm', location.href).href,
      expected: { jsSha256: manifest.artifacts.js.sha256, wasmSha256: manifest.artifacts.wasm.sha256 } });
  });
}

function onWorkerMessage(m) {
  if (m.type === 'ready') {
    if (m.loaded) loaded = m.loaded;
    frontendInfo = m.frontend ?? null;
    pyramidLayout = m.layout ?? null;
    if (frontendInfo?.error) note(`GPU frontend unavailable (${frontendInfo.error}); using the engine's CPU pyramid (much slower)`);
    engineFeatures = m.engineFeatures ?? { surfaces: false, hitTest: false };
    if (config.detectSurfaces && !engineFeatures.surfaces) note('deployed engine has no surface API (accuracy-mode engine not deployed yet): floor detection / hit test off');
    workerReady = true;
    showEngine();
    const r = readyResolve; readyResolve = null;
    if (state === 'idle') setState('idle', 'engine verified and loaded; press Start');
    if (r) r(true);
  } else if (m.type === 'result') {
    onResult(m);
  } else if (m.type === 'hit') {
    onHit(m);
  } else if (m.type === 'fatal') {
    if (m.loaded) { loaded = m.loaded; showEngine(); }
    const loadFailure = m.loaded && !m.loaded.matchesManifest;
    dropWorker();
    stopCapture();
    if (state === 'running') stoppedAtMs = performance.now();
    endRecording();
    fail(`engine fatal: ${m.error}${loadFailure ? '' : '; press Start to reload the engine (the run so far stays downloadable)'}`);
  } else if (m.type === 'log') {
    // Engine stdout/stderr (Ceres/miniglog lines): kept in the metrics, counted on the page.
    engineLog.count++;
    engineLog.lines.push({ atMs: performance.now(), level: m.level, text: m.text });
    if (engineLog.lines.length > 200) engineLog.lines.shift();
  }
}

// ---- sensors -------------------------------------------------------------------------------

function keepTime(list, t) { list.push(t); if (list.length > 2000) list.splice(0, list.length - 2000); }

function queueSensor(row) {
  pendingSensors.push(row);
  if (pendingSensors.length > MAX_PENDING_SENSORS) {
    pendingSensors.splice(0, pendingSensors.length - MAX_PENDING_SENSORS);
    counters.sensorQueueOverflow++;
  }
}

function onMotion(e) {
  if (state !== 'running') return;
  const nowMs = performance.now();
  const g = e.accelerationIncludingGravity, r = e.rotationRate, a = e.acceleration;
  const gOk = g && [g.x, g.y, g.z].every(Number.isFinite);
  const rOk = r && [r.alpha, r.beta, r.gamma].every(Number.isFinite);
  if (!gOk || !rOk) { counters.motionIncomplete++; return; }
  const hasAcc = !!a && [a.x, a.y, a.z].every(Number.isFinite);
  counters.motionEvents++;
  if (hasAcc) counters.motionWithAcceleration++;
  keepTime(sensorTimes.motion, nowMs);
  if (Number.isFinite(e.interval)) keepTime(sensorTimes.interval, e.interval);
  queueSensor(['m', Math.round(nowMs * 1e6), g.x, g.y, g.z, hasAcc, hasAcc ? a.x : 0, hasAcc ? a.y : 0,
    hasAcc ? a.z : 0, r.alpha, r.beta, r.gamma, Number.isFinite(e.interval) ? e.interval : 0]);
}

function onOrientation(e) {
  if (state !== 'running') return;
  const nowMs = performance.now();
  if (![e.alpha, e.beta, e.gamma].every(Number.isFinite)) { counters.orientationIncomplete++; return; }
  counters.orientationEvents++;
  keepTime(sensorTimes.orientation, nowMs);
  queueSensor(['o', Math.round(nowMs * 1e6), e.alpha, e.beta, e.gamma]);
}

async function requestMotionPermission() {
  // Needed on iOS; newer Chrome also exposes it (Android grants motion sensors by default). A
  // refusal is shown, not fatal: the motion rate on the page then stays at 0 Hz.
  for (const C of [globalThis.DeviceMotionEvent, globalThis.DeviceOrientationEvent]) {
    if (C && typeof C.requestPermission === 'function') {
      try {
        const r = await C.requestPermission();
        if (r !== 'granted') note(`${C.name}.requestPermission: ${r} (no motion events expected)`);
      } catch (e) { note(`${C.name}.requestPermission failed: ${e.message}`); }
    }
  }
}

// ---- camera --------------------------------------------------------------------------------

// Opens the back camera picked by cameraRequestFor (default: the lowest-numbered back camera, the
// main camera). Camera labels are hidden until the first permission grant, so the page first opens
// facingMode 'environment', lists the devices, and reopens by deviceId when the browser's choice is
// a different camera. A requested id that is missing refuses the run; when labels stay hidden the
// browser's choice is kept with a note (camera UNVERIFIED).
let cameraSelection = null;
// getUserMedia with the first accepted size request (videoRequestsFor: ?res=); returns the stream
// and the request that opened it. OverconstrainedError / NotReadableError try the next request.
async function openWithSizes(base) {
  let lastError = null;
  for (const size of videoRequestsFor(search)) {
    try {
      return { stream: await navigator.mediaDevices.getUserMedia({ audio: false, video: { ...base, ...size } }), size };
    } catch (e) {
      lastError = e;
      if (!(e && (e.name === 'OverconstrainedError' || e.name === 'NotReadableError' || e.name === 'AbortError'))) break;
    }
  }
  throw lastError ?? new Error('no video size accepted');
}
async function acquireCamera() {
  const request = cameraRequestFor(search);
  let opened = await openWithSizes({ facingMode: { ideal: 'environment' } });
  stream = opened.stream;
  let devices = [];
  try { devices = await navigator.mediaDevices.enumerateDevices(); } catch (e) { note(`enumerateDevices failed: ${e.message}`); }
  const videoInputs = devices.filter((d) => d.kind === 'videoinput').map((d) => d.label);
  const current = stream.getVideoTracks()[0];
  const currentId = current?.getSettings?.().deviceId ?? '';
  const pick = pickBackCamera(devices, request);
  if (request.mode === 'number' && !pick) throw new Error(`back camera ${request.number} is missing or ambiguous (cameras: ${videoInputs.join(' | ') || 'labels hidden'})`);
  if (pick && pick.deviceId !== currentId) {
    for (const t of stream.getTracks()) t.stop();
    stream = null;
    opened = await openWithSizes({ deviceId: { exact: pick.deviceId } });
    stream = opened.stream;
    if (stream.getVideoTracks()[0]?.getSettings?.().deviceId !== pick.deviceId) throw new Error(`camera ${pick.number} was requested but another device opened`);
  }
  const label = stream.getVideoTracks()[0]?.label ?? '';
  const number = backCameraNumber(label);
  cameraSelection = { request: request.mode === 'number' ? `camera ${request.number}` : request.mode, label, number,
    switched: Boolean(pick && pick.deviceId !== currentId), browserChoice: current?.label ?? '', available: videoInputs,
    sizeRequest: opened.size };
  if (request.mode === 'main' && !pick) note(`main back camera not identified (labels: ${videoInputs.join(' | ') || 'hidden'}); using the browser's choice "${label}" (camera UNVERIFIED)`);
  const track = stream.getVideoTracks()[0];
  try {
    const caps = track.getCapabilities?.() ?? {};
    if (caps.zoom && caps.zoom.min <= 1 && caps.zoom.max >= 1) await track.applyConstraints({ advanced: [{ zoom: 1 }] });
  } catch (e) { note(`zoom 1.0 not applied: ${e.message}`); }
  ui.video.srcObject = stream;
  await ui.video.play();
  if (!(ui.video.videoWidth > 0)) await new Promise((r) => ui.video.addEventListener('loadedmetadata', r, { once: true }));
  plan = capturePlan(ui.video.videoWidth, ui.video.videoHeight, search);
  if (plan.rotate !== 'none') note(`landscape stream ${ui.video.videoWidth}x${ui.video.videoHeight}: rotated ${plan.rotate} to portrait (UNVERIFIED direction; hold the phone in portrait)`);
  if (plan.cropped) note(`stream ${ui.video.videoWidth}x${ui.video.videoHeight} center-cropped to 3:4`);
  if (plan.scaled) note(`stream ${ui.video.videoWidth}x${ui.video.videoHeight} resampled to ${CAPTURE_WIDTH}x${CAPTURE_HEIGHT}: the engine's K rule assumes the device's native field of view at this size (UNVERIFIED focal length)`);
}

// After the engine is ready (the capture path depends on the worker's frontend).
function setupCapture() {
  // GPU frontend: the frame goes to the worker as an ImageBitmap (no CPU readback here); CPU
  // frontend: RGBA bytes are read back here (getImageData).
  const bitmapCapture = frontendInfo?.active === 'gpu' && typeof OffscreenCanvas !== 'undefined';
  canvas = bitmapCapture ? new OffscreenCanvas(CAPTURE_WIDTH, CAPTURE_HEIGHT) : document.createElement('canvas');
  canvas.width = CAPTURE_WIDTH; canvas.height = CAPTURE_HEIGHT;
  ctx = canvas.getContext('2d', bitmapCapture ? { alpha: false } : { willReadFrequently: true, alpha: false });
  captureMode = bitmapCapture ? 'bitmap' : 'rgba';
  camera = cameraInfo();
}

// Taken once the stream runs (the metrics are downloaded after Stop, when the track is gone).
let camera = null;
let captureMode = 'rgba';
function cameraInfo() {
  const track = stream?.getVideoTracks()[0];
  if (!track) return null;
  const { deviceId, groupId, ...settings } = track.getSettings?.() ?? {};
  return { label: track.label, settings, video: { width: ui.video.videoWidth, height: ui.video.videoHeight },
    selection: cameraSelection, focal: { intrinsics: config.intrinsics, source: config.focalSource ?? null } };
}

function drawCapture() {
  const p = plan;
  ctx.save();
  if (p.rotate === 'cw') { ctx.translate(p.dw, 0); ctx.rotate(Math.PI / 2); }
  else if (p.rotate === 'ccw') { ctx.translate(0, p.dh); ctx.rotate(-Math.PI / 2); }
  const [w, h] = p.rotate === 'none' ? [p.dw, p.dh] : [p.dh, p.dw];
  ctx.drawImage(ui.video, p.sx, p.sy, p.sw, p.sh, 0, 0, w, h);
  ctx.restore();
}

// Engine input preview: the rotated/cropped 480x640 frame the engine gets, at a low rate (the
// capture canvas itself is handed to the worker as a bitmap).
const PREVIEW_EVERY = 6;
function drawPreview() {
  const c = ui.preview, g = c.getContext('2d');
  g.drawImage(canvas, 0, 0, c.width, c.height);
}

const hasRvfc = 'requestVideoFrameCallback' in HTMLVideoElement.prototype;
let rafHandle = 0, lastVideoTime = -1;

function scheduleFrame() {
  if (hasRvfc) ui.video.requestVideoFrameCallback(onVideoFrame);
  else rafHandle = requestAnimationFrame(() => {
    if (ui.video.currentTime !== lastVideoTime) { lastVideoTime = ui.video.currentTime; onVideoFrame(performance.now(), null); }
    else if (state === 'running') scheduleFrame();
  });
}

function onVideoFrame(_now, meta) {
  if (state !== 'running') return;
  scheduleFrame();
  counters.videoFrames++;
  if (inflight || !workerReady) { counters.droppedBusy++; return; }
  const readMs = performance.now();
  drawCapture();
  if (nextId % PREVIEW_EVERY === 0) drawPreview();   // before the bitmap transfer clears the canvas
  let rgba = null, bitmap = null;
  // ?record=1: a snapshot (GPU) or copy (CPU) of exactly these 480x640 pixels, taken before the
  // transfer/detach below; readback, gray and the OPFS write happen in the recorder's writer Worker.
  const rec = recorder && recorder.active ? recorder : null;
  const recStartMs = rec ? performance.now() : 0;
  let recSrc = null;
  if (captureMode === 'bitmap') { if (rec) recSrc = rec.take({ canvas }); bitmap = canvas.transferToImageBitmap(); }
  else {
    const data = ctx.getImageData(0, 0, CAPTURE_WIDTH, CAPTURE_HEIGHT).data;
    if (rec) recSrc = rec.take({ rgba: data });
    rgba = data.buffer;
  }
  const videoTimeS = meta ? meta.mediaTime : ui.video.currentTime;
  const sensors = pendingSensors;
  pendingSensors = [];
  const id = nextId++;
  try {
    worker.postMessage({ type: 'frame', id, timeNs: Math.round(readMs * 1e6),
      videoTimeNs: Math.round(videoTimeS * 1e9), width: CAPTURE_WIDTH, height: CAPTURE_HEIGHT, rgba, bitmap, sensors },
    bitmap ? [bitmap] : [rgba]);
  } catch (e) {
    if (rec) rec.discard(recSrc);   // the snapshot of a frame the engine never got is released
    throw e;
  }
  if (rec) {
    // The row holds the very values posted above (sensor rows by reference: pendingSensors was replaced).
    rec.add(recSrc, { id, timeNs: Math.round(readMs * 1e6), videoTimeNs: Math.round(videoTimeS * 1e9), readMs, sensors }, recStartMs);
  }
  const postMs = performance.now();
  counters.posted++;
  inflight = { id, readMs, postMs, captureMs: meta && Number.isFinite(meta.captureTime) ? meta.captureTime : null,
    videoTimeS };
}

// ---- results -------------------------------------------------------------------------------

function onResult(m) {
  const f = inflight;
  inflight = null;
  if (!f || f.id !== m.id) return;
  const tRes = performance.now();
  const p = m.pose;
  const rec = { id: m.id, readMs: f.readMs, videoTimeS: f.videoTimeS,
    captureToPostMs: f.postMs - f.readMs,
    queueMs: m.recvAbsMs - (performance.timeOrigin + f.postMs),
    engineMs: m.engineMs, frontend: m.frontend,
    gpuUploadMs: m.uploadMs ?? null, gpuDrawMs: m.drawMs ?? null, gpuReadMs: m.readMs ?? null,
    cpuPyramidMs: m.cpuPyramidMs ?? null, tracker: p.tracker ?? null,
    returnMs: (performance.timeOrigin + tRes) - m.doneAbsMs,
    fresh: !p.repeated, status: p.status, reason: p.reason, position: p.position,
    rotationWxyz: p.rotationWxyz, worldPoints: p.worldPoints, keypoints: p.keypoints ?? null,
    worldPointsMs: m.worldPointsMs,
    stageMs: p.stageMs ?? null, sourceFrameIndex: p.sourceFrameIndex, videoFrameIndex: p.videoFrameIndex,
    sensorsMotion: m.sensors.motion, sensorsOrientation: m.sensors.orientation,
    cameraCaptureToReadMs: f.captureMs === null ? null : f.readMs - f.captureMs,
    metric: p.metric ?? null };
  if (m.surfaces) {
    lastSurfaces = m.surfaces;
    const fl = m.surfaces.floor;
    rec.floor = m.surfaces.floorDetected
      ? { height: fl.height, inliers: fl.inliers, area: fl.area, others: m.surfaces.others.length }
      : { height: null, inliers: 0, area: null, others: m.surfaces.others.length };
    if (view3d) view3d.setSurfaces(m.surfaces);
  }
  counters.results++;
  if (m.gpuCheck) gpuChecks.push({ id: m.id, ...m.gpuCheck });
  if (rec.fresh) counters.fresh++; else counters.repeated++;
  const r0 = performance.now();
  renderResult(rec);
  if (view3d) {
    view3d.addPose(rec);
    if (m.worldPoints) {
      rec.worldPointsShown = view3d.setWorldPoints(m.worldPoints);
      lastWorldPointsInfo = { entries: Math.floor(m.worldPoints.length / 5), shown: rec.worldPointsShown,
        first: Array.from(m.worldPoints.slice(0, 5)) };
    }
  }
  const r1 = performance.now();
  rec.renderMs = r1 - r0;
  rec.poseAgeMs = r1 - (f.captureMs ?? f.readMs);
  records.push(rec);
  if (records.length > MAX_RECORDS) { records.shift(); counters.recordsTruncated++; }
  if (records.length === 1) ui.download.disabled = false;
}

const fmt = (v, d = 3) => (Number.isFinite(v) ? v.toFixed(d) : '-');

const LOSS = ['tracking', 'lost', 'lost (long)'];
function renderTracker(t) {
  if (!t) return;
  const ps = t.pointsBySource;
  ui.tracker.textContent = `tracker: map state ${t.mapTrackerState}  frame tracker ${t.frameTrackerOk ? 'ok' : '-'}` +
    `  loss ${LOSS[t.lossState] ?? t.lossState} (${t.lostFrames} lost)  keyframes ${t.numKeyframes}` +
    `  points T${ps.triangulated}/G${ps.ground}/S${ps.sky}  orb ${t.numOrb}/gorb ${t.numGorb}` +
    `${t.scaleLatched ? `  scale latched ${fmt(t.latchedScale, 4)}` : ''}${t.mapReset ? '  MAP RESET' : ''}`;
}

function renderAccuracy(r) {
  if (!config.accuracy.enabled && !config.detectSurfaces) { ui.accuracy.textContent = 'accuracy mode off (?accuracy=0)'; return; }
  const m = r.metric;
  const scaleLine = m ? `metric scale ${fmt(m.scale, 3)} m/unit ±${fmt(100 * m.sigmaLog, 1)}%` +
    ` ${m.initialized ? 'CONVERGED (positions in metres)' : 'converging — move the phone (until then: camera height 1.4 m assumed)'}` +
    `  IMU lag ${fmt(1000 * m.lagS, 0)} ms` : `metric output off${config.estimateScale ? ' (?scale=absolute: original latch)' : ''}`;
  let floorLine = 'floor: detector off (?surfaces=0)';
  if (config.detectSurfaces) {
    const s = lastSurfaces;
    floorLine = !s ? 'floor: waiting for map points'
      : s.floorDetected ? `floor y ${fmt(s.floor.height, 2)}  camera above floor ${fmt(r.position[1] - s.floor.height, 2)}` +
        `  inliers ${s.floor.inliers}  area ${fmt(s.floor.area, 2)}  other surfaces ${s.others.length}`
        : `floor: not detected yet (${s.cloudPoints} points)`;
  }
  ui.accuracy.textContent = `${scaleLine}\n${floorLine}`;
}

function renderResult(r) {
  renderTracker(r.tracker);
  renderAccuracy(r);
  ui.status.textContent = `${r.status}${r.reason !== 'UNSPECIFIED' ? ` / ${r.reason}` : ''}`;
  ui.pose.textContent = `pos  ${r.position.map((v) => fmt(v)).join(', ')}\n` +
    `quat ${r.rotationWxyz.map((v) => fmt(v, 4)).join(', ')} (wxyz)\n` +
    `world points ${r.worldPoints}  keypoints ${r.keypoints ?? '-'}  ${r.fresh ? 'fresh' : 'repeated'}`;
}

function drawTrajectory() {
  const c = ui.traj, g = c.getContext('2d');
  g.fillStyle = '#000'; g.fillRect(0, 0, c.width, c.height);
  const pts = [];
  for (const r of records) if (r.fresh) pts.push([r.position[0], r.position[2]]);
  if (!pts.length) return;
  let [minX, maxX, minZ, maxZ] = [Infinity, -Infinity, Infinity, -Infinity];
  for (const [x, z] of pts) { minX = Math.min(minX, x); maxX = Math.max(maxX, x); minZ = Math.min(minZ, z); maxZ = Math.max(maxZ, z); }
  const span = Math.max(maxX - minX, maxZ - minZ, 0.5);
  const s = (c.width - 30) / span, cx = (minX + maxX) / 2, cz = (minZ + maxZ) / 2;
  const X = (x) => c.width / 2 + (x - cx) * s, Y = (z) => c.height / 2 + (z - cz) * s;
  g.strokeStyle = '#4fc3f7'; g.lineWidth = 2; g.beginPath();
  pts.forEach(([x, z], i) => (i ? g.lineTo(X(x), Y(z)) : g.moveTo(X(x), Y(z))));
  g.stroke();
  const [lx, lz] = pts[pts.length - 1];
  g.fillStyle = '#ff5252'; g.beginPath(); g.arc(X(lx), Y(lz), 5, 0, 2 * Math.PI); g.fill();
  g.fillStyle = '#aaa'; g.font = '16px sans-serif'; g.fillText(`top-down x/z  ${span.toFixed(2)} u`, 6, 18);
}

const VIEW_MODES = { orbit: 'viewOrbit', follow: 'viewFollow', top: 'viewTop' };

function setViewMode(mode) {
  if (!view3d) return;
  view3d.setMode(mode);
  for (const [k, id] of Object.entries(VIEW_MODES)) ui[id].classList.toggle('on', k === mode);
}

function renderView3dStats() {
  const s = view3d.stats();
  const u = config.estimateScale ? 'm' : 'u (responsive scale, not metric)';
  ui.view3dStats.textContent = `poses ${s.poses}  path ${fmt(s.pathLength, 2)}  end→start ${fmt(s.endToStart, 2)}\n` +
    `height ${s.heightRange ? `${fmt(s.heightRange[0], 2)}…${fmt(s.heightRange[1], 2)}` : '-'}  map points ${s.points}  units ${u}`;
}

function view3dLoop(t) {
  requestAnimationFrame(view3dLoop);
  if (!view3d || document.hidden || t - lastView3dRender < VIEW3D_INTERVAL_MS) return;
  lastView3dRender = t;
  view3d.render();
}

function loadView3d() {
  try {
    view3d = new TrajectoryView3D(ui.view3d, { origin });
  } catch (e) {
    view3d = null;
    note(`3D view unavailable (${e && e.message || e}); showing the 2D top-down view`);
    return;
  }
  ui.view3dBox.hidden = false;
  document.getElementById('view3dLegend').hidden = false;
  ui.traj.hidden = true;
  view3d.resize();
  new ResizeObserver(() => view3d.resize()).observe(ui.view3d);
  ui.viewOrbit.addEventListener('click', () => setViewMode('orbit'));
  ui.viewFollow.addEventListener('click', () => setViewMode('follow'));
  ui.viewTop.addEventListener('click', () => setViewMode('top'));
  ui.viewFit.addEventListener('click', () => view3d.fit());
  requestAnimationFrame(view3dLoop);
}

// ?gpu-check=1 summary: frames whose GPU atlas equals the CPU port atlas byte for byte, per model.
function gpuCheckLine() {
  const parts = ['swiftshader-approx', 'ieee'].map((m) => {
    const c = gpuChecks.map((g) => g[m]);
    const same = c.filter((x) => x.identical).length;
    const minJ = Math.min(...c.map((x) => x.scoreJaccard));
    return `${m} identical ${same}/${c.length}, min score Jaccard ${fmt(minJ, 3)}`;
  });
  return `gpu-check: ${parts.join(' · ')}`;
}

function rateHz(times) {
  if (times.length < 2) return null;
  return (times.length - 1) / ((times[times.length - 1] - times[0]) / 1000);
}

function renderPeriodic() {
  if (!records.length && state !== 'running') return;
  if (view3d) renderView3dStats(); else drawTrajectory();
  const s = summarize(records, origin);
  const line = (name, x) => `${name.padEnd(13)} p50 ${fmt(x.p50, 1)}  p95 ${fmt(x.p95, 1)}  p99 ${fmt(x.p99, 1)}  (n ${x.n})`;
  const block = (title, b) => !b.frames ? `${title}: no frames` : [`${title}: ${b.frames} results, ${b.fresh} fresh`,
    line('capture→post', b.captureToPostMs), line('queue', b.queueMs),
    ...(b.gpuDrawMs.n ? [line('gpu upload', b.gpuUploadMs), line('gpu draw', b.gpuDrawMs), line('gpu read', b.gpuReadMs)] : []),
    ...(b.cpuPyramidMs.n ? [line('cpu pyramid', b.cpuPyramidMs)] : []),
    line('engine', b.engineMs),
    line('render', b.renderMs), line('pose age', b.poseAgeMs)].join('\n');
  ui.timing.textContent = [
    `video frames ${counters.videoFrames}  processed ${counters.results}  dropped(busy) ${counters.droppedBusy}  resets ${counters.resets}`,
    `motion ${counters.motionEvents} (${fmt(rateHz(sensorTimes.motion), 1)} Hz)  orientation ${counters.orientationEvents} (${fmt(rateHz(sensorTimes.orientation), 1)} Hz)`,
    s.firstTranslation ? `first translation: result #${s.firstTranslation.recordIndex} (${fmt(s.firstTranslation.sinceStartMs / 1000, 1)} s)` : 'first translation: not yet',
    block('pre-init', s.preInit), block('post-init', s.postInit),
    ...(gpuChecks.length ? [gpuCheckLine()] : []),
    `engine log lines ${engineLog.count} (in the metrics JSON)`,
    'timing in ms on this device; desktop numbers are not phone numbers',
  ].join('\n');
}

// ---- controls ------------------------------------------------------------------------------

function addSensorListeners() {
  window.addEventListener('devicemotion', onMotion);
  window.addEventListener('deviceorientation', onOrientation);
}

function removeSensorListeners() {
  window.removeEventListener('devicemotion', onMotion);
  window.removeEventListener('deviceorientation', onOrientation);
}

function stopCapture() {
  removeSensorListeners();
  if (rafHandle) cancelAnimationFrame(rafHandle);
  if (stream) for (const t of stream.getTracks()) t.stop();
  stream = null;
  ui.video.srcObject = null;
  if (wakeLock) wakeLock.release().catch(() => {});
  wakeLock = null;
}

async function start() {
  // After a fatal the worker is gone (worker === null) and initEngine below creates a new one.
  if (!(state === 'idle' || state === 'stopped' || state === 'error') || !(workerReady || !worker)) return;
  if (recorder?.finalizing) return;
  ui.start.disabled = true;
  ui.state.classList.remove('err');
  notes.clear(); ui.notes.textContent = ''; lastError = null;
  setState('starting', 'requesting camera and motion sensors');
  try {
    await requestMotionPermission();
    if (screen.orientation && screen.orientation.angle !== 0) note(`screen orientation angle ${screen.orientation.angle}: the engine is portrait-only (R16); hold the phone upright`);
    camera = null; plan = null; cameraSelection = null;
    // The camera comes first: its id selects the accuracy-mode focal (ACC-4) of this run's engine.
    await acquireCamera();
    config = configForCamera(baseConfig, search, cameraSelection, plan);
    showEngine();
    // Fresh engine instance per measurement run (a new worker after a fatal).
    if (!(await initEngine())) throw new Error('engine did not load');
    records = []; resets = []; gpuChecks.length = 0; pendingSensors = []; inflight = null; nextId = 0;
    recSummary = null; recUnposted = []; recorder = null;
    for (const k of Object.keys(counters)) counters[k] = 0;
    for (const k of Object.keys(sensorTimes)) sensorTimes[k] = [];
    if (view3d) view3d.clear();
    setupCapture();
    if (config.record) {
      // Explicit opt-in: a failure to open the OPFS file refuses the run (no silent unrecorded run).
      recorder = new InputRecorder({ width: CAPTURE_WIDTH, height: CAPTURE_HEIGHT,
        verifyEvery: Math.max(0, Number(new URLSearchParams(search).get('rec-verify')) | 0) });
      // A stop (limit, writer backlog, writer/OPFS error) is shown at once, not only after Stop.
      recorder.onStopped = (r) => {
        note(`recording stopped (${r.reason}): ${r.message}; frames from ${r.atFrame} on are NOT recorded (the engine run continues)`);
        ui.recBadge.textContent = r.reason === 'error' ? '✕ REC FAILED' : '■ REC STOPPED';
      };
      const space = await recorder.begin();
      ui.recBadge.textContent = '● REC';
      note(`● REC: frames and sensors are being stored on this device (OPFS ${recorder.names.frames}); nothing is sent` +
        (space ? `; storage free ${(space.freeBytes / 1e9).toFixed(1)} GB (~${space.frames} frames)${space.persisted ? '' : ', not persisted'}` : ''));
    }
    try { wakeLock = await navigator.wakeLock?.request('screen'); } catch { wakeLock = null; }
    startedAtMs = performance.now(); stoppedAtMs = null;
    setState('running', `capture ${CAPTURE_WIDTH}x${CAPTURE_HEIGHT} from ${ui.video.videoWidth}x${ui.video.videoHeight} (${plan.rotate})${hasRvfc ? '' : ', no requestVideoFrameCallback'}`);
    addSensorListeners();
    scheduleFrame();
  } catch (e) {
    stopCapture();
    endRecording();
    fail(`start failed: ${e && e.message || e}`);
  }
}

function stop() {
  if (state !== 'running') return;
  stoppedAtMs = performance.now();
  stopCapture();
  setState('stopped', `${counters.results} results; press Download to save metrics`);
  endRecording();
  renderPeriodic();
}

// Finishes the OPFS recording (drains the writer, writes the meta JSON, hashes it). Idempotent.
async function endRecording() {
  const rec = recorder;
  if (!rec || rec.summary || rec.finalizing || !rec.writer) return;
  // Sensor rows after the last recorded frame; meaningless when the recording stopped early.
  recUnposted = rec.stopReason ? [] : pendingSensors; pendingSensors = [];
  rec.active = false;
  rec.finalizing = true;      // visible to setState right away; finish() keeps it until done
  setState(state, 'finalizing recording');
  recSummary = await rec.finish((stats) => recordingMeta(rec, stats));
  if (state === 'error') {   // a failed start keeps its own message; the recording result is a note
    setState('error', lastError);  // finish cleared finalizing; refresh restart/download controls
    note(recSummary.ok ? `recording saved on this device: ${recSummary.frames} frames` : `recording failed: ${recSummary.error}`);
    return;
  }
  setState(state, recSummary.ok
    ? `recording saved on this device: ${recSummary.frames} frames; press 녹화 다운로드 to save the files`
    : `recording FAILED: ${recSummary.error}`);
  if (!recSummary.ok) note(`recording failed: ${recSummary.error}`);
}

// The meta JSON of a recording: everything the PC replay needs (replay_device_recording.mjs).
function recordingMeta(rec, stats) {
  return {
    schema: 'slam-refact-input-recording-v1',
    generatedAt: new Date().toISOString(),
    note: 'frames.bin = raw gray8 frames concatenated (width*height bytes each, row-major, frame k at k*width*height); ' +
      'gray = round(0.299R+0.587G+0.114B) of the 480x640 engine-input pixels. The engine GPU path converts colour itself, ' +
      'so this gray is an approximation for a CPU-pyramid replay. frames[k].sensors are the exact rows posted with frame k ' +
      "(['m', tNs, gx,gy,gz, hasAcc, ax,ay,az, ra,rb,rg, intervalMs] / ['o', tNs, alpha,beta,gamma]); tNs and timeNs share the page's performance.now() ns clock.",
    width: rec.width, height: rec.height, pixelFormat: 'gray8', frameBytes: rec.frameBytes,
    grayFormula: 'round(0.299*R + 0.587*G + 0.114*B)',
    files: { frames: rec.names.frames, meta: rec.names.meta },
    page: { path: location.pathname, search, userAgent: navigator.userAgent },
    config,
    engine: { manifestSha256: { js: manifest?.artifacts.js.sha256 ?? null, wasm: manifest?.artifacts.wasm.sha256 ?? null },
      manifest, loaded },
    capturePlan: plan, camera, captureMode, frontend: frontendInfo,
    counters: { ...counters }, resets: stats.resets,
    recording: { frames: stats.frames, framesBytes: stats.framesBytes, maxQueuedFrames: stats.maxQueuedFrames,
      limitReached: stats.limitReached, maxFrames: stats.maxFrames, stopReason: stats.stopReason,
      mainThreadCostMs: percentiles(rec.costsMs), writerCostMs: stats.writerCostMs, verify: stats.verify },
    unpostedSensors: recUnposted,
    frames: rec.rows.map(({ index, id, timeNs, videoTimeNs, readMs, sensors }) => ({ index, id, timeNs, videoTimeNs, readMs, sensors })),
  };
}

function reset() {
  if (state !== 'running') return;
  counters.resets++;
  resets.push({ afterRecord: records.length, atMs: performance.now() });
  if (recorder) recorder.noteReset(performance.now());
  if (view3d) view3d.breakTrajectory();
  worker.postMessage({ type: 'reset' });
}

// Tap on the engine input preview: hit test of that point of the 480x640 frame (engine ACC-3).
function onPreviewTap(e) {
  if (!worker || !workerReady || !config.detectSurfaces || !engineFeatures?.hitTest) return;
  const r = ui.preview.getBoundingClientRect();
  const u = (e.clientX - r.left) / r.width, v = (e.clientY - r.top) / r.height;
  if (!(u >= 0 && u <= 1 && v >= 0 && v <= 1)) return;
  worker.postMessage({ type: 'hitTest', id: ++hitId, u, v,
    mask: HIT_TYPES.FEATURE_POINT | HIT_TYPES.ESTIMATED_SURFACE | HIT_TYPES.DETECTED_SURFACE });
}

function onHit(m) {
  const best = m.hits[0] ?? null;
  hits.push({ atMs: performance.now(), u: m.u, v: m.v, hits: m.hits });
  if (hits.length > 500) hits.shift();
  ui.hit.textContent = best
    ? `hit (${fmt(m.u, 2)}, ${fmt(m.v, 2)}): ${best.type} at ${best.position.map((x) => fmt(x, 2)).join(', ')}` +
      `  distance ${fmt(best.distance, 2)}${m.hits.length > 1 ? `  (+${m.hits.length - 1} more)` : ''}`
    : `hit (${fmt(m.u, 2)}, ${fmt(m.v, 2)}): nothing (no floor/surface/feature point on this ray yet)`;
  if (view3d) view3d.setHit(best);
}

// Recording summary of the metrics JSON: file names, frame count, bytes, sha256 of the meta JSON
// (null when ?record=1 is off). The recording itself is never part of the metrics JSON.
function recorderSummary() {
  if (!config.record) return null;
  if (recSummary) return { ...recSummary, mainThreadCostMs: percentiles(recorder?.costsMs ?? []) };
  return { enabled: true, ok: null, active: !!recorder?.active, frames: recorder?.count ?? 0,
    files: recorder?.names ?? null, error: recorder?.error ?? null };
}

function metricsJson() {
  const intervals = percentiles(sensorTimes.interval, [50]);
  return {
    schema: 'slam-refact-live-metrics-v1',
    note: 'pose/status/timing only; no images, no video, no raw IMU samples',
    generatedAt: new Date().toISOString(),
    page: { path: location.pathname, search, userAgent: navigator.userAgent,
      screen: { width: screen.width, height: screen.height, dpr: devicePixelRatio,
        orientationType: screen.orientation?.type ?? null, orientationAngle: screen.orientation?.angle ?? null },
      requestVideoFrameCallback: hasRvfc, crossOriginIsolated: globalThis.crossOriginIsolated === true },
    engine: { manifest, loaded },
    config, origin, capturePlan: plan, camera,
    run: { state, startedAtMs, stoppedAtMs, durationS: startedAtMs === null ? null
      : ((stoppedAtMs ?? performance.now()) - startedAtMs) / 1000, lastError },
    counters,
    sensors: { motionHz: rateHz(sensorTimes.motion), orientationHz: rateHz(sensorTimes.orientation),
      motionIntervalMsP50: intervals.p50 },
    summary: summarize(records, origin),
    frontend: frontendInfo, pyramidLayout, gpuChecks, engineLog,
    surfaces: lastSurfaces, hits,
    resets, notes: [...notes], records,
    recording: recorderSummary(),
  };
}

// The only place a file is handed to the browser's download (always from a user click handler).
function saveUrl(url, name) {
  const a = document.createElement('a');
  a.href = url;
  a.download = name;
  document.body.appendChild(a); a.click(); a.remove();
  setTimeout(() => URL.revokeObjectURL(url), 60000);
}

function download() {
  if (!records.length) return;
  saveUrl(URL.createObjectURL(new Blob([JSON.stringify(metricsJson())], { type: 'application/json' })),
    `slam-refact-metrics-${new Date().toISOString().replace(/[:.]/g, '-')}.json`);
}

// "녹화 다운로드" (user click only): the two OPFS files as object URLs of their File objects.
async function downloadRecording() {
  if (!recSummary || !recSummary.ok) return;
  const api = recordingApi();
  for (const name of [recSummary.files.frames, recSummary.files.meta]) saveUrl(URL.createObjectURL(await api.file(name)), name);
}

function showEngine() {
  if (!manifest) return;
  const m = manifest;
  const v = loaded ? (loaded.matchesManifest ? 'bytes verified against manifest' : 'BYTES DO NOT MATCH MANIFEST') : 'not loaded yet';
  ui.engine.textContent = `engine slam_refact @ ${m.git.head.slice(0, 12)}${m.git.engine_dirty ? ' (engine tree DIRTY)' : ''}` +
    ` · wasm ${m.artifacts.wasm.sha256.slice(0, 16)}… · js ${m.artifacts.js.sha256.slice(0, 16)}…` +
    ` · deployed ${m.deployed_at} · ${v}\nconfig model "${config.model}" pyramid ${config.pyramidModel}` +
    ` scale ${config.estimateScale ? 'absolute' : config.accuracy.metricOutput ? 'metric (accuracy mode)' : 'responsive'}` +
    ` accuracy ${config.accuracy.enabled ? 'on' : 'off'} surfaces ${config.detectSurfaces ? 'on' : 'off'}` +
    `${config.intrinsics ? ` fx ${config.intrinsics[0]}` : ''}${config.focalSource ? ` (focal: ${config.focalSource})` : ''}` +
    `${cameraSelection ? ` · camera "${cameraSelection.label}"${cameraSelection.switched ? ` (browser chose "${cameraSelection.browserChoice}")` : ''}` : ''} capture ${CAPTURE_WIDTH}x${CAPTURE_HEIGHT}` +
    `\nfrontend ${frontendInfo ? `${frontendInfo.active}${frontendInfo.requested !== frontendInfo.active ? ` (requested ${frontendInfo.requested})` : ''}` : '-'}` +
    `${frontendInfo?.renderer ? ` · ${frontendInfo.renderer.renderer}` : ''}`;
}

// Read-only snapshot for the headless smoke (slam_refact/web-smoke/run-live-smoke.mjs).
window.__slamRefactSnapshot = () => ({ state, workerReady, counters: { ...counters }, records: records.length,
  loaded, lastError, cameraOpen: stream !== null, frontend: frontendInfo, view3d: view3d ? view3d.stats() : null,
  config: { accuracy: config.accuracy, detectSurfaces: config.detectSurfaces, model: config.model },
  lastMetric: records.length ? records[records.length - 1].metric : null, surfaces: lastSurfaces, hits: hits.length,
  lastWorldPoints: lastWorldPointsInfo,
  recording: config.record ? recorderSummary() : null });
// Diagnostic for the headless smoke: request a hit test without a pointer event.
window.__slamRefactHitTest = (u, v) => onPreviewTap({ clientX: ui.preview.getBoundingClientRect().left + u * ui.preview.getBoundingClientRect().width,
  clientY: ui.preview.getBoundingClientRect().top + v * ui.preview.getBoundingClientRect().height });

async function boot() {
  // Client Hints model (Chrome Android sends the reduced UA "K"): selects the engine's calibrated
  // device focal (accuracy mode). The rest of the configuration does not depend on it.
  const hintModel = await clientHintModel(navigator);
  if (hintModel) config = baseConfig = engineConfigFor(search, navigator.userAgent, hintModel);
  ui.preview.addEventListener('click', onPreviewTap);
  ui.start.addEventListener('click', start);
  ui.stop.addEventListener('click', stop);
  ui.reset.addEventListener('click', reset);
  ui.download.addEventListener('click', download);
  if (config.record) {
    // Explicit opt-in (?record=1): REC badge + banner, download button, OPFS read API for CDP pulling.
    ui.recBadge.hidden = false; ui.recBanner.hidden = false; ui.downloadRec.hidden = false;
    ui.downloadRec.addEventListener('click', downloadRecording);
    window.__slamRefactRecording = recordingApi();
  }
  setState('idle', 'loading engine manifest');
  // View and periodic rendering do not depend on the engine (a failed load can be retried by Start).
  loadView3d();
  setInterval(renderPeriodic, 500);
  if (!window.isSecureContext) note('not a secure context: camera and motion sensors need HTTPS');
  try {
    if (!(await initEngine())) return;   // the fatal handler has shown the error
  } catch (e) {
    fail(`engine unavailable: ${e && e.message || e}`);
    return;
  }
  setState('idle', 'engine verified and loaded; press Start');
}

boot();

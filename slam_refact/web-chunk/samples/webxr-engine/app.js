// WebXR camera-access -> slam_refact engine prototype (Galaxy Z Flip6 / Chrome). Nothing runs before a button tap.
//  A: inside an immersive-ar session for `phaseA` s: rate and dt of every IMU source (DeviceMotion, DeviceOrientation, Generic Sensor API at
//     `freq` Hz), XR pose rate, and the camera texture row order / mirror check (image shift vs XR pose rotation, xr-camera.js).
//  B: the engine (WASM SlamRefactGlue, loaded with the chunk's manifest hash check) on camera-access frames in the XR WebGL2 context:
//     camera texture -> own capture texture (crop + exact box decimation + row-order flip, xr-camera.js header) -> Gr8ComputePyramid ->
//     processFramePyramid, IMU from the best source of a `warmup` s measurement in the same session. Each frame logs the engine pose next to
//     the ARCore viewer pose of the same XR frame (reference estimator, NOT ground truth), in memory only; JSON export on click.
// Frame pipeline (as xr-slam.js): XR frame N copies + stages N (the camera texture is valid only inside this callback, so the copy is made
// here), then frame N+1 reads N's pyramid, pushes the IMU staged with N and calls processFramePyramid(N); the glue answers for its previous
// staged frame (pose.sourceFrameIndex), which is matched back to that frame's record by the glue's stage index.
import { loadEngine } from '../../engine/engine-loader.js';
import { Gr8ComputePyramid } from '../../engine/gr8-compute-pyramid.js';
import { createSensors, pushMotion } from '../../engine/sensors.js';
import { estimateShiftSAD, percentile } from '../webxr-probe/analysis.js';
import { buildGlueConfig, imageOrientationCheck, intrinsicsFromXrProjection, planEngineInput, relativeRotationVector } from './xr-camera.js';
import { chooseImuSource, createGenericMotionFuser, quatToDeviceOrientation, rateStats } from './imu-sources.js';
import { REFERENCE_LABEL, compareTrajectories, pairsFromRecords, xyzwToWxyz } from './trajectory-align.js';

const $ = (id) => document.getElementById(id);
const params = new URLSearchParams(location.search);
const num = (k, d) => (params.has(k) && Number.isFinite(Number(params.get(k))) ? Number(params.get(k)) : d);
const PHASE_A_MS = num('phaseA', 5) * 1000;
const WARMUP_MS = num('warmup', 2) * 1000;
const FREQ = num('freq', 60);
const MAX_RECORDS = Math.max(100, num('maxRecords', 36000) | 0);
const IMU_FORCE = params.get('imu') ?? 'auto';
const FLIP_Y = params.get('flipY') ?? 'auto';
const FLIP_X = params.get('flipX') ?? 'auto';
const FRAME_MODE = params.get('frame') ?? 'full';
const ESTIMATE_SCALE = (params.get('scale') ?? 'absolute') === 'absolute';
const ACCURACY = params.get('accuracy') === '1' ? { enabled: true } : null;
const ENGINE_URL = new URL('../../engine/slam-refact-engine/', import.meta.url).href;
const SMALL_W = 64;
const results = { schema: 'webxr-engine/1', label: REFERENCE_LABEL, phaseA: null, phaseB: null };
let busy = false, stopFn = null;

const fmt = (v, d = 1) => (typeof v === 'number' && Number.isFinite(v) ? v.toFixed(d) : '-');
const setStatus = (s) => { $('status').textContent = s; };
const errText = (e) => `${e && e.name || 'Error'}: ${e && e.message || e}`;
const r6 = (v) => Math.round(v * 1e6) / 1e6;

async function getModel() {
  if (!navigator.userAgentData || typeof navigator.userAgentData.getHighEntropyValues !== 'function') return { model: '', source: 'unavailable' };
  try {
    const h = await navigator.userAgentData.getHighEntropyValues(['model', 'platformVersion']);
    return { model: h.model || '', platformVersion: h.platformVersion, source: 'userAgentData.getHighEntropyValues' };
  } catch (e) { return { model: '', source: `error ${errText(e)}` }; }
}

// requestSession is the first await of a click handler (user activation).
async function startXrSession() {
  if (!navigator.xr) throw new Error('navigator.xr 없음');
  const canvas = document.createElement('canvas');
  const gl = canvas.getContext('webgl2', { xrCompatible: true, alpha: true, antialias: false, depth: false });
  if (!gl) throw new Error('WebGL2 사용 불가');
  const session = await navigator.xr.requestSession('immersive-ar',
    { requiredFeatures: ['local', 'camera-access'], optionalFeatures: ['dom-overlay'], domOverlay: { root: $('overlay') } });
  const refSpace = await session.requestReferenceSpace('local');
  const layer = new XRWebGLLayer(session, gl, { alpha: true, antialias: false, depth: false });
  await session.updateRenderState({ baseLayer: layer });
  const binding = new XRWebGLBinding(session, gl);
  return { session, gl, refSpace, layer, binding, enabledFeatures: session.enabledFeatures ? Array.from(session.enabledFeatures) : null };
}

const GENERIC = [['gyroscope', 'Gyroscope'], ['accelerometer', 'Accelerometer'], ['linearAcceleration', 'LinearAccelerationSensor'],
  ['relativeOrientation', 'RelativeOrientationSensor'], ['absoluteOrientation', 'AbsoluteOrientationSensor']];

// Every IMU source at once. While `recording`, event times (DOM: event.timeStamp, Generic: reading timestamp) and receive times
// (performance.now() in the handler) are kept; the callbacks always run.
function startSensorMonitor({ freq, onDeviceMotion, onDeviceOrientation, onGeneric }) {
  const mon = { recording: false, t0: performance.now(), src: {}, stopped: false };
  const entry = (name, extra = {}) => (mon.src[name] = { ts: [], recv: [], ...extra });
  const dm = entry('devicemotion', { withRotationRate: 0, withAccelerationIncludingGravity: 0 });
  const dor = entry('deviceorientation', { withAngles: 0 });
  const onMotion = (e) => {
    if (mon.recording) {
      dm.ts.push(e.timeStamp); dm.recv.push(performance.now());
      if (e.rotationRate && e.rotationRate.alpha != null) dm.withRotationRate++;
      if (e.accelerationIncludingGravity && e.accelerationIncludingGravity.x != null) dm.withAccelerationIncludingGravity++;
    }
    if (onDeviceMotion) onDeviceMotion(e);
  };
  const onOrient = (e) => {
    if (mon.recording) { dor.ts.push(e.timeStamp); dor.recv.push(performance.now()); if (e.alpha != null) dor.withAngles++; }
    if (onDeviceOrientation) onDeviceOrientation(e);
  };
  window.addEventListener('devicemotion', onMotion, true);
  window.addEventListener('deviceorientation', onOrient, true);
  const sensors = {};
  for (const [name, ctor] of GENERIC) {
    const st = entry(name, { available: typeof window[ctor] === 'function', started: false, error: null });
    if (!st.available) continue;
    try {
      const s = new window[ctor]({ frequency: freq });
      s.addEventListener('reading', () => {
        if (mon.recording) { st.ts.push(s.timestamp); st.recv.push(performance.now()); }
        if (onGeneric) onGeneric(name, s);
      });
      s.addEventListener('error', (e) => { st.error = errText(e.error || e); });
      s.start();
      st.started = true;
      sensors[name] = s;
    } catch (e) { st.error = errText(e); }
  }
  mon.stopGeneric = (keep = []) => { for (const [name, s] of Object.entries(sensors)) if (!keep.includes(name)) { try { s.stop(); } catch { /* stopped */ } delete sensors[name]; } };
  mon.stop = () => {
    if (mon.stopped) return;
    mon.stopped = true;
    window.removeEventListener('devicemotion', onMotion, true);
    window.removeEventListener('deviceorientation', onOrient, true);
    mon.stopGeneric();
  };
  // windowMs: the recording window (rate = events / window, so a silent source reports 0 Hz, not NaN).
  mon.report = (windowMs) => {
    const out = {};
    for (const [name, s] of Object.entries(mon.src)) {
      const { ts, recv, ...rest } = s;
      out[name] = { ...rest, ...rateStats(ts, windowMs), receive: rateStats(recv, windowMs) };
    }
    return out;
  };
  return mon;
}

async function queryPermissions() {
  const out = {};
  for (const name of ['accelerometer', 'gyroscope', 'magnetometer']) {
    try { out[name] = (await navigator.permissions.query({ name })).state; } catch (e) { out[name] = `unsupported (${e.name})`; }
  }
  return out;
}

function glProgram(gl, vs, fs) {
  const sh = (type, src) => {
    const s = gl.createShader(type); gl.shaderSource(s, src); gl.compileShader(s);
    if (!gl.getShaderParameter(s, gl.COMPILE_STATUS)) throw new Error(gl.getShaderInfoLog(s));
    return s;
  };
  const p = gl.createProgram();
  gl.attachShader(p, sh(gl.VERTEX_SHADER, vs)); gl.attachShader(p, sh(gl.FRAGMENT_SHADER, fs));
  gl.linkProgram(p);
  if (!gl.getProgramParameter(p, gl.LINK_STATUS)) throw new Error(gl.getProgramInfoLog(p));
  return p;
}
const FULLSCREEN_VS = `#version 300 es
void main() { vec2 p = vec2(float((gl_VertexID << 1) & 2), float(gl_VertexID & 2)); gl_Position = vec4(p * 2.0 - 1.0, 0.0, 1.0); }`;

function targetTexture(gl, w, h, filter) {
  const tex = gl.createTexture();
  gl.bindTexture(gl.TEXTURE_2D, tex);
  gl.texImage2D(gl.TEXTURE_2D, 0, gl.RGBA8, w, h, 0, gl.RGBA, gl.UNSIGNED_BYTE, null);
  for (const [k, v] of [[gl.TEXTURE_MIN_FILTER, filter], [gl.TEXTURE_MAG_FILTER, filter], [gl.TEXTURE_WRAP_S, gl.CLAMP_TO_EDGE], [gl.TEXTURE_WRAP_T, gl.CLAMP_TO_EDGE]]) gl.texParameteri(gl.TEXTURE_2D, k, v);
  const fbo = gl.createFramebuffer();
  gl.bindFramebuffer(gl.FRAMEBUFFER, fbo);
  gl.framebufferTexture2D(gl.FRAMEBUFFER, gl.COLOR_ATTACHMENT0, gl.TEXTURE_2D, tex, 0);
  if (gl.checkFramebufferStatus(gl.FRAMEBUFFER) !== gl.FRAMEBUFFER_COMPLETE) throw new Error('framebuffer incomplete');
  gl.bindTexture(gl.TEXTURE_2D, null);
  return { tex, fbo };
}

function drawFullscreen(gl, prog, fbo, w, h, cameraTex, vao) {
  gl.bindFramebuffer(gl.FRAMEBUFFER, fbo);
  gl.viewport(0, 0, w, h);
  for (const c of [gl.BLEND, gl.DEPTH_TEST, gl.SCISSOR_TEST, gl.STENCIL_TEST, gl.CULL_FACE]) gl.disable(c);
  gl.colorMask(true, true, true, true);
  gl.useProgram(prog);
  gl.bindVertexArray(vao);
  gl.activeTexture(gl.TEXTURE0);
  gl.bindTexture(gl.TEXTURE_2D, cameraTex);
  gl.uniform1i(gl.getUniformLocation(prog, 'uTex'), 0);
  gl.drawArrays(gl.TRIANGLES, 0, 3);
  gl.bindTexture(gl.TEXTURE_2D, null);
  gl.bindVertexArray(null);
}

// Phase A: box-average luminance of the camera texture into sw x sh with texel fetches; row r of the result = texture rows r * foot (texture
// row order, whatever the image orientation is - that is what imageOrientationCheck measures).
function makeLumaReader(gl, sw, sh) {
  const prog = glProgram(gl, FULLSCREEN_VS, `#version 300 es
precision highp float;
uniform sampler2D uTex;
out vec4 o;
void main() {
  vec2 foot = vec2(textureSize(uTex, 0)) / vec2(${sw}.0, ${sh}.0);
  vec2 origin = floor(gl_FragCoord.xy) * foot;
  float acc = 0.0;
  for (int j = 0; j < 8; j++) for (int i = 0; i < 8; i++) acc += dot(texelFetch(uTex, ivec2(origin + (vec2(float(i), float(j)) + 0.5) / 8.0 * foot), 0).rgb, vec3(0.299, 0.587, 0.114));
  o = vec4(acc / 64.0, 0.0, 0.0, 1.0);
}`);
  const { fbo } = targetTexture(gl, sw, sh, gl.NEAREST);
  const vao = gl.createVertexArray();
  const rgba = new Uint8Array(sw * sh * 4);
  return (cameraTex) => {
    drawFullscreen(gl, prog, fbo, sw, sh, cameraTex, vao);
    gl.readPixels(0, 0, sw, sh, gl.RGBA, gl.UNSIGNED_BYTE, rgba);
    const luma = new Uint8Array(sw * sh);
    for (let i = 0; i < luma.length; i++) luma[i] = rgba[i * 4];
    return luma;
  };
}

// Phase B: camera texture -> engine capture texture (plan from planEngineInput). Output pixel (xo, yo), yo counted from the image top =
// GL row yo of the output (row 0 = image top, the engine's convention), averages the 6 x 6 sub-samples of its source footprint
// [x0 + xo d, x0 + (xo + 1) d) x [y0 + yo d, ...) in top-down image coordinates; for d = 3 that is an exact 3 x 3 box (2 x 2 fetches per
// source pixel). flipY: the camera texture's row 0 is the image bottom; flipX: it is mirrored. The output keeps RGB like XR8's camera texture.
function makeCaptureCopier(gl, plan, { flipY, flipX }) {
  const prog = glProgram(gl, FULLSCREEN_VS, `#version 300 es
precision highp float;
precision highp int;
uniform sampler2D uTex;
uniform vec2 uOrigin;
uniform float uFoot;
uniform int uFlipY;
uniform int uFlipX;
out vec4 o;
void main() {
  ivec2 size = textureSize(uTex, 0);
  vec2 outPx = floor(gl_FragCoord.xy);
  vec3 acc = vec3(0.0);
  for (int j = 0; j < 6; j++) for (int i = 0; i < 6; i++) {
    ivec2 p = clamp(ivec2(floor(uOrigin + (outPx + (vec2(float(i), float(j)) + 0.5) / 6.0) * uFoot)), ivec2(0), size - 1);
    if (uFlipX == 1) p.x = size.x - 1 - p.x;
    if (uFlipY == 1) p.y = size.y - 1 - p.y;
    acc += texelFetch(uTex, p, 0).rgb;
  }
  o = vec4(acc / 36.0, 1.0);
}`);
  const { width, height } = plan.output;
  const target = targetTexture(gl, width, height, gl.LINEAR);
  const vao = gl.createVertexArray();
  gl.useProgram(prog);
  gl.uniform2f(gl.getUniformLocation(prog, 'uOrigin'), plan.crop.x0, plan.crop.y0);
  gl.uniform1f(gl.getUniformLocation(prog, 'uFoot'), plan.footprint);
  gl.uniform1i(gl.getUniformLocation(prog, 'uFlipY'), flipY ? 1 : 0);
  gl.uniform1i(gl.getUniformLocation(prog, 'uFlipX'), flipX ? 1 : 0);
  gl.useProgram(null);
  return {
    texture: target.tex,
    draw: (cameraTex) => drawFullscreen(gl, prog, target.fbo, width, height, cameraTex, vao),
    dispose: () => { gl.deleteProgram(prog); gl.deleteTexture(target.tex); gl.deleteFramebuffer(target.fbo); gl.deleteVertexArray(vao); },
  };
}

const poseOf = (pose) => {
  if (!pose) return null;
  const { position: p, orientation: o } = pose.transform;
  return { p: [p.x, p.y, p.z], q: xyzwToWxyz(o), emulated: !!pose.emulatedPosition };
};

function clearLayer(gl, layer) {
  gl.bindFramebuffer(gl.FRAMEBUFFER, layer.framebuffer);
  gl.clearColor(0, 0, 0, 0);
  gl.clear(gl.COLOR_BUFFER_BIT);
}

// ---- phase A -------------------------------------------------------------------------------------------------------------------
async function runPhaseA() {
  const xr = await startXrSession();
  const modelP = getModel();
  const mon = startSensorMonitor({ freq: FREQ });
  mon.recording = true;
  const permsP = queryPermissions();
  const xrTimes = [], shifts = [], camSizes = {};
  let reader = null, sw = 0, sh = 0, prevLuma = null, prevQ = null, K = null, t0 = null, lastUi = 0, imageErrors = 0, lastImageError = null;
  const tStart = performance.now();
  await new Promise((resolve) => {
    let done = false;
    const finish = () => { if (done) return; done = true; stopFn = null; resolve(); };
    stopFn = () => xr.session.end().catch(() => {});
    xr.session.addEventListener('end', finish);
    const onFrame = (time, frame) => {
      if (done) return;
      if (t0 === null) t0 = time;
      if (time - t0 >= PHASE_A_MS) { xr.session.end().catch(() => {}); return; }
      xr.session.requestAnimationFrame(onFrame);
      xrTimes.push(time);
      if (time - lastUi > 250) { lastUi = time; setStatus(`A: ${Math.ceil((PHASE_A_MS - (time - t0)) / 1000)} s - 폰을 위아래·좌우로 돌리세요`); }
      const pose = frame.getViewerPose(xr.refSpace);
      const view = pose && pose.views[0];
      const q = view ? xyzwToWxyz(view.transform.orientation) : null;
      let luma = null;
      if (view && view.camera) {
        const cam = view.camera;
        camSizes[`${cam.width}x${cam.height}`] = (camSizes[`${cam.width}x${cam.height}`] || 0) + 1;
        if (!K) K = intrinsicsFromXrProjection(view.projectionMatrix, cam.width, cam.height);
        try {
          const tex = xr.binding.getCameraImage(cam);
          if (tex) {
            if (!reader) { sw = SMALL_W; sh = Math.max(8, Math.round(SMALL_W * cam.height / cam.width)); reader = makeLumaReader(xr.gl, sw, sh); }
            luma = reader(tex);
          }
        } catch (e) { imageErrors++; lastImageError = errText(e); }
      }
      if (luma && prevLuma && q && prevQ) {
        const s = estimateShiftSAD(prevLuma, luma, sw, sh);
        shifts.push({ dx: s.dx, dy: s.dy, ok: s.ok && !s.saturated, theta: relativeRotationVector(prevQ, q) });
      }
      prevLuma = luma; prevQ = q;
      clearLayer(xr.gl, xr.layer);
    };
    xr.session.requestAnimationFrame(onFrame);
  });
  const windowMs = performance.now() - tStart;
  mon.recording = false;
  mon.stop();
  const sources = mon.report(windowMs);
  const orientationCheck = K && sw ? imageOrientationCheck(shifts, K.fx * sw / K.width, K.fy * sh / K.height) : null;
  results.phaseA = {
    windowMs, freqRequested: FREQ, enabledFeatures: xr.enabledFeatures, device: await modelP, permissions: await permsP,
    xrPose: rateStats(xrTimes), camera: { sizes: camSizes, K, imageErrors, lastImageError, plan: K ? safePlan(K) : null },
    sources, orientationCheck, choice: chooseImuSource(sources, { force: IMU_FORCE }),
  };
  renderA();
}

const safePlan = (K) => { try { return planEngineInput(K.width, K.height, K, { mode: FRAME_MODE }); } catch (e) { return { error: errText(e) }; } };

function renderA() {
  const a = results.phaseA;
  if (!a) return;
  const lines = [`window ${fmt(a.windowMs / 1000, 1)} s, model ${a.device.model || '(없음)'} (${a.device.source}), XR pose ${fmt(a.xrPose.rateHz)} Hz (dt p50 ${fmt(a.xrPose.dtP50Ms)} / p95 ${fmt(a.xrPose.dtP95Ms)} ms)`,
    `permissions ${JSON.stringify(a.permissions)}`, '', 'source                rate Hz  dt p50/p95 ms (event time) | receive rate Hz  dt p50/p95 ms | note'];
  for (const [name, s] of Object.entries(a.sources)) {
    const note = s.available === false ? 'API 없음' : s.error ? s.error : s.started === false ? '시작 안 됨' : name === 'devicemotion' ? `rotationRate ${s.withRotationRate}` : '';
    lines.push(`${name.padEnd(20)} ${fmt(s.rateHz).padStart(7)}  ${fmt(s.dtP50Ms)}/${fmt(s.dtP95Ms)} | ${fmt(s.receive.rateHz).padStart(7)}  ${fmt(s.receive.dtP50Ms)}/${fmt(s.receive.dtP95Ms)} | ${note}`);
  }
  const k = a.camera.K;
  lines.push('', `camera ${JSON.stringify(a.camera.sizes)}${k ? `, K (index conv.) fx ${fmt(k.fx)} fy ${fmt(k.fy)} cx ${fmt(k.cx)} cy ${fmt(k.cy)}` : ''}`);
  if (a.camera.plan && a.camera.plan.K) {
    const p = a.camera.plan;
    lines.push(`engine capture ${p.output.width}x${p.output.height} (crop x0 ${p.crop.x0} w ${p.crop.width}, footprint ${fmt(p.footprint, 3)}), K' fx ${fmt(p.K.fx, 2)} fy ${fmt(p.K.fy, 2)} cx ${fmt(p.K.cx, 2)} cy ${fmt(p.K.cy, 2)}`);
  }
  const oc = a.orientationCheck;
  if (oc) lines.push(`orientation check: y slope ${fmt(oc.y.slope, 2)} r ${fmt(oc.y.corr, 2)} n ${oc.y.n}, x slope ${fmt(oc.x.slope, 2)} r ${fmt(oc.x.corr, 2)} n ${oc.x.n} -> flipY ${oc.flipY}, flipX ${oc.flipX}${oc.conclusive ? '' : ' (불확정: 더 크게 돌려서 다시)'}`);
  lines.push(`IMU choice: ${a.choice.motion} / orientation ${a.choice.orientation}${a.choice.disableVio ? ' -> disableVio' : ''}; ${a.choice.reasons.join('; ')}`);
  $('report-a').textContent = lines.join('\n');
}

// ---- phase B -------------------------------------------------------------------------------------------------------------------
function resolveFlips() {
  const oc = results.phaseA && results.phaseA.orientationCheck;
  const pick = (param, measured, fallback) => (param === '0' || param === '1' ? { value: param === '1', source: `?=${param}` }
    : measured !== null && measured !== undefined ? { value: measured, source: 'phase A orientation check' } : { value: fallback, source: 'default (unverified)' });
  // Default flipY = 1: assumes the WebXR camera texture follows the GL convention (row 0 = image bottom). UNVERIFIED until phase A measures it.
  return { flipY: pick(FLIP_Y, oc && oc.flipY, true), flipX: pick(FLIP_X, oc && oc.flipX === true ? true : null, false) };
}

async function runPhaseB() {
  const engineP = loadEngine(ENGINE_URL);
  engineP.catch(() => {});  // handled in the frame loop
  const xr = await startXrSession();
  const modelP = getModel();
  const flips = resolveFlips();
  const st = { framesSeen: 0, staged: 0, processed: 0, fresh: 0, noCamera: 0, noImage: 0, sizeMismatch: 0, stageErrors: 0, processErrors: 0,
    unmatchedResponses: 0, motionPushed: 0, motionSkipped: 0, firstError: null };
  const log = { records: [], dropped: 0 };
  const byStage = new Map();
  let engine = null, engineError = null, model = null, choice = null, plan = null, glue = null, layout = null, pyramid = null, copier = null;
  let glueConfig = null, readback = null, pending = null, lastPose = null, t0 = null, lastUi = 0, warmStart = null;
  const orientation = { alpha: 0, beta: 0, gamma: 0, events: 0 };
  const fuser = createGenericMotionFuser({ frequency: FREQ });
  let running = false;
  const dmQueue = createSensors(() => !(running && choice && choice.motion === 'devicemotion'));
  engineP.then((e) => { engine = e; }, (e) => { engineError = e; });
  modelP.then((m) => { model = m; });
  const mon = startSensorMonitor({
    freq: FREQ,
    onDeviceMotion: (e) => dmQueue.updateDeviceMotion(e),
    onDeviceOrientation: (e) => {
      if (!(running && choice.orientation === 'deviceorientation') || e.alpha == null) return;
      orientation.alpha = e.alpha; orientation.beta = e.beta; orientation.gamma = e.gamma; orientation.events++;
    },
    onGeneric: (name, s) => {
      if (!running) return;
      if (choice.motion === 'generic') {
        if (name === 'gyroscope') fuser.onGyroscope(s);
        else if (name === 'accelerometer') fuser.onAccelerometer(s);
        else if (name === 'linearAcceleration') fuser.onLinearAcceleration(s);
      }
      if ((name === 'relativeOrientation' && choice.orientation === 'relative') || (name === 'absoluteOrientation' && choice.orientation === 'absolute')) {
        Object.assign(orientation, quatToDeviceOrientation(s.quaternion)); orientation.events++;
      }
    },
  });
  mon.recording = true;
  const xrTimes = [], engineMs = [], pushTimes = [];
  const fail = (where, e) => { if (!st.firstError) st.firstError = `${where}: ${errText(e)}`; };

  const startEngine = (cam, view, time) => {
    const windowMs = time - warmStart;
    mon.recording = false;
    const warm = mon.report(windowMs);
    choice = chooseImuSource(warm, { force: IMU_FORCE });
    const keep = [...(choice.motion === 'generic' ? ['gyroscope', 'accelerometer', 'linearAcceleration'] : []),
      ...(choice.orientation === 'relative' ? ['relativeOrientation'] : choice.orientation === 'absolute' ? ['absoluteOrientation'] : [])];
    mon.stopGeneric(keep);
    const K = intrinsicsFromXrProjection(view.projectionMatrix, cam.width, cam.height);
    plan = planEngineInput(cam.width, cam.height, K, { mode: FRAME_MODE });
    glueConfig = buildGlueConfig({ plan, model: model ? model.model : '', disableVio: choice.disableVio, estimateScale: ESTIMATE_SCALE, accuracy: ACCURACY });
    const { Module, loaded } = engine;
    glue = new Module.SlamRefactGlue(glueConfig);
    const coords = { origin: { x: 0, y: 2, z: 0 }, facing: { w: 1, x: 0, y: 0, z: 0 } };
    // Same order as xr-slam.js createSession: recenter point, configure, then the camera and the current origin / facing.
    glue.updateCameraProjectionMatrix({ ...coords, updateRecenterPoint: true });
    glue.configure({ enableWorldPoints: true, enableLighting: false, leftHandedAxes: false, mirroredDisplay: false });
    glue.updateCameraProjectionMatrix({ cam: { pixelRectWidth: xr.layer.framebufferWidth, pixelRectHeight: xr.layer.framebufferHeight, nearClipPlane: 0.01, farClipPlane: 1000 },
      ...coords, updateRecenterPoint: false });
    layout = glue.pyramidLayout();
    readback = { level0: layout.levels[0], textureSize: layout.textureSize, rows: layout.rows,
      captureIntrinsics: typeof glue.getCaptureIntrinsics === 'function' ? { ...glue.getCaptureIntrinsics() } : 'unavailable in this WASM' };
    pyramid = new Gr8ComputePyramid(xr.gl, layout, { cameraWidth: plan.output.width, cameraHeight: plan.output.height, rotation: 0, warn: (m) => fail('pyramid', m) });
    copier = makeCaptureCopier(xr.gl, plan, { flipY: flips.flipY.value, flipX: flips.flipX.value });
    results.phaseB.config = { glueConfig, plan, flips, choice, warmup: { windowMs, sources: warm }, readback, device: model,
      engine: { url: ENGINE_URL, jsSha256: loaded.jsSha256, wasmSha256: loaded.wasmSha256, matchesManifest: loaded.matchesManifest, build: loaded.manifest.build } };
    running = true;
  };

  const processPending = () => {
    const f = pending;
    pending = null;
    const t0p = performance.now();
    const atlas = pyramid.read();
    const rows = pyramid.lastReadRows;
    const t1 = performance.now();
    let pushed = 0, skipped = 0;
    for (const e of f.events) { if (pushMotion(glue, e)) pushed++; else skipped++; }
    glue.pushDeviceOrientation(f.timeNs, Number(f.orient.alpha), Number(f.orient.beta), Number(f.orient.gamma));
    const tNs = Math.round(f.rec.xrTime * 1e6);
    const pose = glue.processFramePyramid(tNs, tNs, atlas, layout.textureSize, rows);
    const t2 = performance.now();
    st.processed++; st.motionPushed += pushed; st.motionSkipped += skipped;
    for (let i = 0; i < pushed; i++) pushTimes.push(t2);
    f.rec.ms.read = t1 - t0p; f.rec.ms.engine = t2 - t1;
    f.rec.imu = { pushed, skipped, orientationEvents: f.orientEvents };
    engineMs.push(t2 - t1);
    f.rec.stageIndex = pose.videoFrameIndex;
    byStage.set(pose.videoFrameIndex, f.rec);
    while (byStage.size > 64) byStage.delete(byStage.keys().next().value);
    if (!pose.repeated) {
      st.fresh++;
      const src = byStage.get(pose.sourceFrameIndex);
      const out = { status: pose.status, reason: pose.reason, p: Array.from(pose.position, r6), q: Array.from(pose.rotationWxyz, r6),
        worldPoints: pose.worldPoints, answeredAtStage: pose.videoFrameIndex };
      if (src) src.engine = out; else st.unmatchedResponses++;
    }
    lastPose = pose;
  };

  const hud = (time) => {
    const win = (arr, ms) => { while (arr.length && arr[0] < time - ms) arr.shift(); return arr.length; };
    const fps = win(xrTimes, 2000) / 2;
    while (pushTimes.length && pushTimes[0] < performance.now() - 2000) pushTimes.shift();
    if (engineMs.length > 150) engineMs.splice(0, engineMs.length - 150);
    const s = lastPose ? `${lastPose.status}/${lastPose.reason}` : running ? '(첫 응답 대기)' : `warm-up${engine ? '' : ' / 엔진 로딩'}`;
    $('hud').textContent = [`engine ${s}  wp ${lastPose ? lastPose.worldPoints : '-'}`,
      `XR ${fmt(fps)} fps  engine ms p50 ${fmt(percentile(engineMs, 50))} p95 ${fmt(percentile(engineMs, 95))}`,
      `IMU ${choice ? choice.motion : '-'} ${fmt(pushTimes.length / 2)} ev/s  orient ${choice ? choice.orientation : '-'}  flipY ${flips.flipY.value ? 1 : 0}`,
      `frames ${st.framesSeen} staged ${st.staged} fresh ${st.fresh} log ${log.records.length}${log.dropped ? ` (+${log.dropped} dropped)` : ''}${st.firstError ? `\n! ${st.firstError}` : ''}`].join('\n');
  };

  results.phaseB = { config: null, stats: st, log, alignment: null };
  await new Promise((resolve) => {
    let done = false;
    const finish = () => { if (done) return; done = true; stopFn = null; resolve(); };
    stopFn = () => xr.session.end().catch(() => {});
    xr.session.addEventListener('end', finish);
    const onFrame = (time, frame) => {
      if (done) return;
      xr.session.requestAnimationFrame(onFrame);
      if (t0 === null) { t0 = time; warmStart = time; }
      st.framesSeen++;
      xrTimes.push(time);
      const pose = frame.getViewerPose(xr.refSpace);
      const view = pose && pose.views[0];
      const cam = view && view.camera;
      if (engineError) { fail('engine load', engineError); xr.session.end().catch(() => {}); return; }
      if (!running) {
        if (cam && engine && model && time - warmStart >= WARMUP_MS) {
          try { startEngine(cam, view, time); } catch (e) { fail('engine start', e); xr.session.end().catch(() => {}); return; }
        } else { if (time - lastUi > 250) { lastUi = time; hud(time); } clearLayer(xr.gl, xr.layer); return; }
      }
      const rec = { i: st.framesSeen, t: performance.now(), xrTime: time, stageIndex: null, ref: poseOf(view), engine: null, ms: {}, imu: null };
      if (log.records.length < MAX_RECORDS) log.records.push(rec); else log.dropped++;
      if (pending) { try { processPending(); } catch (e) { st.processErrors++; fail('process', e); } }
      if (!cam) st.noCamera++;
      else if (cam.width !== plan.source.width || cam.height !== plan.source.height) st.sizeMismatch++;
      else {
        try {
          const tex = xr.binding.getCameraImage(cam);
          if (!tex) st.noImage++;
          else {
            const a = performance.now();
            copier.draw(tex);
            const b = performance.now();
            pyramid.stage(copier.texture, []);
            rec.ms.copy = b - a; rec.ms.stage = performance.now() - b;
            const events = choice.motion === 'devicemotion' ? dmQueue.takeForStage() || [] : choice.motion === 'generic' ? fuser.take() : [];
            pending = { rec, events, orient: { ...orientation }, orientEvents: orientation.events, timeNs: 1e3 * performance.now() * 1e3 };
            st.staged++;
          }
        } catch (e) { st.stageErrors++; fail('stage', e); }
      }
      clearLayer(xr.gl, xr.layer);
      if (time - lastUi > 250) { lastUi = time; hud(time); }
    };
    xr.session.requestAnimationFrame(onFrame);
  });
  mon.stop();
  st.fuser = fuser.stats;
  st.pyramid = pyramid ? { ...pyramid.stats } : null;
  try { if (copier) copier.dispose(); if (pyramid) pyramid.dispose(); if (glue) glue.delete(); } catch (e) { fail('dispose', e); }
  const pairs = pairsFromRecords(log.records);
  results.phaseB.alignment = compareTrajectories(pairs, { refFrames: log.records.filter((r) => r.ref).length });
  renderB();
}

function renderB() {
  const b = results.phaseB;
  if (!b) return;
  const c = b.config, s = b.stats, al = b.alignment;
  const statusCounts = {};
  for (const r of b.log.records) if (r.engine) statusCounts[`${r.engine.status}/${r.engine.reason}`] = (statusCounts[`${r.engine.status}/${r.engine.reason}`] || 0) + 1;
  const lines = [];
  if (c) {
    lines.push(`engine capture ${c.plan.output.width}x${c.plan.output.height} from ${c.plan.source.width}x${c.plan.source.height}, K' [${c.glueConfig.intrinsics.map((v) => fmt(v, 2)).join(', ')}], model '${c.glueConfig.model}'`,
      `flipY ${c.flips.flipY.value} (${c.flips.flipY.source}), flipX ${c.flips.flipX.value} (${c.flips.flipX.source}), IMU ${c.choice.motion} / ${c.choice.orientation}${c.choice.disableVio ? ' (disableVio)' : ''}, estimateScale ${c.glueConfig.estimateScale}`,
      `engine wasm ${c.engine.wasmSha256.slice(0, 12)} manifest ${c.engine.matchesManifest ? 'ok' : 'MISMATCH'}, level0 ${c.readback.level0.w}x${c.readback.level0.h}`);
  }
  lines.push(`frames ${s.framesSeen} staged ${s.staged} processed ${s.processed} fresh ${s.fresh}; noCamera ${s.noCamera} noImage ${s.noImage} sizeMismatch ${s.sizeMismatch}; errors stage ${s.stageErrors} process ${s.processErrors}${s.firstError ? ` (${s.firstError})` : ''}`,
    `IMU pushed ${s.motionPushed} skipped ${s.motionSkipped}; engine status ${JSON.stringify(statusCounts)}`, '', `${REFERENCE_LABEL}:`);
  if (!al || al.insufficient) lines.push(`  ${al ? al.insufficient : '-'}`);
  else {
    const e = (x, u, d = 3) => `rmse ${fmt(x.rmse, d)} mean ${fmt(x.mean, d)} median ${fmt(x.median, d)} max ${fmt(x.max, d)} ${u}`;
    lines.push(`  pairs ${al.pairs} / ref frames ${al.refFrames} (coverage ${fmt(al.coverage * 100)} %), ${fmt(al.durationS)} s${al.degenerate ? ', DEGENERATE geometry' : ''}`,
      `  SE(3) ATE trans ${e(al.se3.ate.trans, 'm')}`, `  SE(3) ATE rot   ${e(al.se3.ate.rotDeg, 'deg', 2)} (body offset diag ${fmt(al.se3.ate.bodyOffsetDiag.angleDeg, 2)} deg)`,
      `  SE(3) RPE 1 s trans ${e(al.se3.rpe.trans, 'm')} rot ${fmt(al.se3.rpe.rotDeg.rmse, 2)} deg rmse (n ${al.se3.rpe.trans.n})`,
      `  Sim(3) scale ${fmt(al.sim3.transform.scale, 3)} (diagnostic), ATE trans rmse ${fmt(al.sim3.ate.trans.rmse, 3)} m, RPE 1 s trans rmse ${fmt(al.sim3.rpe.trans.rmse, 3)} m`);
    for (const n of al.notes) lines.push(`  note: ${n}`);
  }
  $('report-b').textContent = lines.join('\n');
}

// ---- wiring --------------------------------------------------------------------------------------------------------------------
async function guarded(label, fn) {
  if (busy) return;
  busy = true;
  for (const id of ['btn-a', 'btn-b', 'btn-export']) $(id).disabled = true;
  $('stop').disabled = false;
  $('status').classList.remove('bad');
  try {
    setStatus(`${label} 시작`);
    await fn();
    setStatus(`${label} 완료`);
  } catch (e) {
    setStatus(`${label} 실패: ${errText(e)}`);
    $('status').classList.add('bad');
  } finally {
    busy = false;
    stopFn = null;
    $('stop').disabled = true;
    for (const id of ['btn-a', 'btn-b']) $(id).disabled = false;
    $('btn-export').disabled = !(results.phaseA || results.phaseB);
    renderA(); renderB();
  }
}

$('btn-a').addEventListener('click', () => guarded('A', runPhaseA));
$('btn-b').addEventListener('click', () => guarded('B', runPhaseB));
$('stop').addEventListener('click', () => stopFn && stopFn());
$('btn-export').addEventListener('click', () => {
  const out = { ...results, exportedAt: new Date().toISOString(), page: location.pathname + location.search, userAgent: navigator.userAgent };
  const blob = new Blob([JSON.stringify(out)], { type: 'application/json' });
  const a = document.createElement('a');
  a.href = URL.createObjectURL(blob);
  a.download = `webxr-engine-${new Date().toISOString().replace(/[:.]/g, '-')}.json`;
  a.click();
  setTimeout(() => URL.revokeObjectURL(a.href), 5000);
});

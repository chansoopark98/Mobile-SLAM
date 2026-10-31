// WebXR camera-access / EIS probe (P0-C2). Nothing runs before a button tap; nothing is stored or sent. Results are on screen and
// downloadable as JSON only on click. Camera frames are reduced to a 64-px-wide luminance image and discarded; no image is kept.
import { intrinsicsFromProjection, fpsStats, estimateShiftSAD, analyzeRun } from './analysis.js';

const $ = (id) => document.getElementById(id);
const params = new URLSearchParams(location.search);
const SETTLE_MS = Number(params.get('settle') ?? 4) * 1000; // before recording (get the phone moving)
const RECORD_MS = Number(params.get('dur') ?? 20) * 1000;
const SMALL_W = 64;
const results = { schema: 'webxr-probe/1', note: 'device measurements, not accuracy', caps: null, xr: null, gum: null };
let busy = false;
let stopFn = null;

const fmt = (v, d = 2) => (typeof v === 'number' && Number.isFinite(v) ? v.toFixed(d) : '-');
const setStatus = (s) => { $('status').textContent = s; };
const buzz = (ms) => { try { navigator.vibrate?.(ms); } catch { /* optional */ } };

// --- capabilities ------------------------------------------------------------------------------------------------------------
async function checkCaps() {
  const c = {
    secureContext: window.isSecureContext,
    userAgent: navigator.userAgent,
    hasNavigatorXr: !!navigator.xr,
    hasXRWebGLBinding: typeof XRWebGLBinding !== 'undefined',
    hasGetCameraImage: typeof XRWebGLBinding !== 'undefined' && 'getCameraImage' in XRWebGLBinding.prototype,
    hasDeviceMotion: typeof DeviceMotionEvent !== 'undefined',
    hasRequestVideoFrameCallback: 'requestVideoFrameCallback' in HTMLVideoElement.prototype,
    hasGetUserMedia: !!navigator.mediaDevices?.getUserMedia,
  };
  try { c.immersiveArSupported = navigator.xr ? await navigator.xr.isSessionSupported('immersive-ar') : false; } catch (e) { c.immersiveArSupported = `error: ${e}`; }
  if (navigator.userAgentData) {
    try {
      const h = await navigator.userAgentData.getHighEntropyValues(['model', 'platformVersion', 'fullVersionList']);
      c.userAgentData = { mobile: navigator.userAgentData.mobile, platform: navigator.userAgentData.platform, model: h.model, platformVersion: h.platformVersion };
    } catch (e) { c.userAgentData = `error: ${e}`; }
  }
  results.caps = c;
  $('caps').textContent = JSON.stringify(c, null, 2);
  return c;
}

// --- shared run bookkeeping ----------------------------------------------------------------------------------------------------
function newRun(kind) {
  return { kind, frames: [], frameTimes: [], gyro: { t: [], x: [], y: [], z: [] }, gyroNull: 0, recording: false };
}

async function startGyro(run) {
  try { if (typeof DeviceMotionEvent?.requestPermission === 'function') await DeviceMotionEvent.requestPermission(); } catch { /* not iOS */ }
  const onMotion = (e) => {
    if (!run.recording) return;
    const r = e.rotationRate;
    if (!r || r.alpha == null || r.beta == null || r.gamma == null) { run.gyroNull++; return; }
    run.gyro.t.push(e.timeStamp); run.gyro.x.push(r.beta); run.gyro.y.push(r.gamma); run.gyro.z.push(r.alpha); // deg/s about device x, y, z
  };
  window.addEventListener('devicemotion', onMotion);
  return () => window.removeEventListener('devicemotion', onMotion);
}

// Phase from elapsed ms: 'settle' -> 'record' -> 'done'. Updates the on-screen countdown.
function phaseOf(elapsed) {
  if (elapsed < SETTLE_MS) { $('countdown').textContent = `준비 ${Math.ceil((SETTLE_MS - elapsed) / 1000)}`; return 'settle'; }
  if (elapsed < SETTLE_MS + RECORD_MS) { $('countdown').textContent = `흔드세요 ${Math.ceil((SETTLE_MS + RECORD_MS - elapsed) / 1000)}`; return 'record'; }
  return 'done';
}

// Shift between consecutive small luma images; writes the pair onto the newer frame record.
function pairShift(rec, prev, luma, w, h) {
  if (!prev || !luma) return;
  const s = estimateShiftSAD(prev, luma, w, h);
  rec.dx = s.dx; rec.dy = s.dy; rec.ok = s.ok && !s.saturated;
  rec.sat = s.saturated;
}

function summarize(run, extra) {
  const gyro = run.gyro.t.length ? run.gyro : null;
  const ok = run.frames.filter((f) => f.ok).length;
  const sat = run.frames.filter((f) => f.sat).length;
  const a = analyzeRun({ frames: run.frames, gyro, fxSmallPx: extra.fxSmallRef ?? null });
  return {
    ...extra, record: { settleMs: SETTLE_MS, durationMs: RECORD_MS },
    fps: fpsStats(run.frames.map((f) => f.t)),
    shiftFrames: { total: run.frames.length, validShift: ok, saturated: sat },
    deviceMotion: { samples: run.gyro.t.length, nullSamples: run.gyroNull },
    analysis: a,
    raw: { // numbers only, no images
      frames: run.frames.map((f) => [Math.round(f.t * 100) / 100, round(f.dx), round(f.dy), f.ok ? 1 : 0, f.q ? f.q.map((v) => round(v, 5)) : null]),
      gyro: { t: run.gyro.t.map((v) => Math.round(v * 100) / 100), x: run.gyro.x.map((v) => round(v)), y: run.gyro.y.map((v) => round(v)), z: run.gyro.z.map((v) => round(v)) },
    },
  };
}
const round = (v, d = 3) => (Number.isFinite(v) ? Math.round(v * 10 ** d) / 10 ** d : null);

// --- WebXR path ---------------------------------------------------------------------------------------------------------------
const VS = `#version 300 es
void main() { vec2 p = vec2(float((gl_VertexID << 1) & 2), float(gl_VertexID & 2)); gl_Position = vec4(p * 2.0 - 1.0, 0.0, 1.0); }`;
// Box-average the camera texture into the small image with nearest texel fetches (independent of the texture's filter state).
const fsSrc = (sw, sh) => `#version 300 es
precision highp float;
uniform sampler2D uTex;
out vec4 o;
void main() {
  vec2 foot = vec2(textureSize(uTex, 0)) / vec2(${sw}.0, ${sh}.0);
  vec2 origin = floor(gl_FragCoord.xy) * foot;
  float acc = 0.0;
  for (int j = 0; j < 8; j++) for (int i = 0; i < 8; i++) {
    vec3 c = texelFetch(uTex, ivec2(origin + (vec2(float(i), float(j)) + 0.5) / 8.0 * foot), 0).rgb;
    acc += dot(c, vec3(0.299, 0.587, 0.114));
  }
  o = vec4(acc / 64.0, 0.0, 0.0, 1.0);
}`;

function makeReader(gl, sw, sh) {
  const compile = (type, src) => {
    const s = gl.createShader(type); gl.shaderSource(s, src); gl.compileShader(s);
    if (!gl.getShaderParameter(s, gl.COMPILE_STATUS)) throw new Error(gl.getShaderInfoLog(s));
    return s;
  };
  const prog = gl.createProgram();
  gl.attachShader(prog, compile(gl.VERTEX_SHADER, VS));
  gl.attachShader(prog, compile(gl.FRAGMENT_SHADER, fsSrc(sw, sh)));
  gl.linkProgram(prog);
  if (!gl.getProgramParameter(prog, gl.LINK_STATUS)) throw new Error(gl.getProgramInfoLog(prog));
  const tex = gl.createTexture();
  gl.bindTexture(gl.TEXTURE_2D, tex);
  gl.texImage2D(gl.TEXTURE_2D, 0, gl.RGBA8, sw, sh, 0, gl.RGBA, gl.UNSIGNED_BYTE, null);
  const fbo = gl.createFramebuffer();
  gl.bindFramebuffer(gl.FRAMEBUFFER, fbo);
  gl.framebufferTexture2D(gl.FRAMEBUFFER, gl.COLOR_ATTACHMENT0, gl.TEXTURE_2D, tex, 0);
  const rgba = new Uint8Array(sw * sh * 4);
  return (cameraTex, restoreFbo) => {
    gl.bindFramebuffer(gl.FRAMEBUFFER, fbo);
    gl.viewport(0, 0, sw, sh);
    gl.disable(gl.BLEND); gl.disable(gl.DEPTH_TEST); gl.disable(gl.SCISSOR_TEST);
    gl.useProgram(prog);
    gl.activeTexture(gl.TEXTURE0);
    gl.bindTexture(gl.TEXTURE_2D, cameraTex);
    gl.uniform1i(gl.getUniformLocation(prog, 'uTex'), 0);
    gl.drawArrays(gl.TRIANGLES, 0, 3);
    gl.readPixels(0, 0, sw, sh, gl.RGBA, gl.UNSIGNED_BYTE, rgba);
    gl.bindTexture(gl.TEXTURE_2D, null);
    gl.bindFramebuffer(gl.FRAMEBUFFER, restoreFbo);
    const luma = new Uint8Array(sw * sh);
    for (let i = 0; i < luma.length; i++) luma[i] = rgba[i * 4];
    return luma;
  };
}

async function runXr() {
  const run = newRun('webxr');
  const meta = { attempts: [], notes: [] };
  if (!navigator.xr) throw new Error('navigator.xr 없음');
  const canvas = document.createElement('canvas');
  const gl = canvas.getContext('webgl2', { xrCompatible: true, alpha: true, antialias: false, depth: false });
  if (!gl) throw new Error('WebGL2 사용 불가');
  const attempts = [
    { name: 'camera-access required', init: { requiredFeatures: ['local', 'camera-access'], optionalFeatures: ['dom-overlay'] } },
    { name: 'camera-access optional', init: { requiredFeatures: ['local'], optionalFeatures: ['camera-access', 'dom-overlay'] } },
  ];
  let session = null;
  for (const a of attempts) {
    try {
      session = await navigator.xr.requestSession('immersive-ar', { ...a.init, domOverlay: { root: $('overlay') } });
      meta.attempts.push({ attempt: a.name, ok: true });
      break;
    } catch (e) { meta.attempts.push({ attempt: a.name, ok: false, error: `${e.name}: ${e.message}` }); }
  }
  if (!session) { results.xr = { meta, error: 'immersive-ar session refused' }; throw new Error(`세션 거부: ${JSON.stringify(meta.attempts)}`); }
  meta.enabledFeatures = session.enabledFeatures ? Array.from(session.enabledFeatures) : 'unavailable';
  meta.domOverlayState = session.domOverlayState?.type ?? null;
  const stopGyro = await startGyro(run);

  const refSpace = await session.requestReferenceSpace('local');
  const layer = new XRWebGLLayer(session, gl, { alpha: true, antialias: false, depth: false });
  await session.updateRenderState({ baseLayer: layer });
  const binding = typeof XRWebGLBinding !== 'undefined' ? new XRWebGLBinding(session, gl) : null;

  let reader = null; let sw = 0; let sh = 0;
  let prevLuma = null;
  let t0 = null; let phase = 'settle'; let lastUi = 0;
  const camStat = { frames: 0, withViewCamera: 0, withImage: 0, errors: 0, lastError: null, sizes: {} };
  const intr = { first: null, fxMin: Infinity, fxMax: -Infinity, fyMin: Infinity, fyMax: -Infinity, views: 0 };
  const recFrameTimes = [];

  return new Promise((resolve) => {
    let finished = false;
    const finish = async (reason) => {
      if (finished) return;
      finished = true;
      run.recording = false;
      stopGyro();
      stopFn = null;
      try { await session.end(); } catch { /* already ended */ }
      const fxSmallRef = intr.first && sw ? intr.first.fx * (sw / intr.first.width) : null;
      const sum = summarize(run, { reason, camera: camStat, intrinsics: { ...intr, fxMin: round(intr.fxMin), fxMax: round(intr.fxMax), fyMin: round(intr.fyMin), fyMax: round(intr.fyMax) }, smallImage: { width: sw, height: sh }, fxSmallRef, meta });
      sum.fps = fpsStats(recFrameTimes);
      results.xr = sum;
      resolve(sum);
    };
    stopFn = () => finish('user stop');
    session.addEventListener('end', () => finish('session ended'));

    const onFrame = (time, frame) => {
      if (finished) return;
      session.requestAnimationFrame(onFrame);
      if (t0 == null) t0 = time;
      const elapsed = time - t0;
      phase = phaseOf(elapsed);
      if (phase === 'done') { buzz([80, 60, 80]); finish('complete'); return; }
      if (phase === 'record' && !run.recording) { run.recording = true; buzz(200); }
      if (time - lastUi > 250) { lastUi = time; setStatus(`WebXR ${phase === 'record' ? '기록 중' : '준비'} (프레임 ${camStat.frames}, 이미지 ${camStat.withImage})`); }

      const pose = frame.getViewerPose(refSpace);
      const view = pose?.views?.[0];
      camStat.frames++;
      let luma = null;
      if (view) {
        const cam = view.camera;
        if (cam) {
          camStat.withViewCamera++;
          const key = `${cam.width}x${cam.height}`;
          camStat.sizes[key] = (camStat.sizes[key] ?? 0) + 1;
          const intrinsics = intrinsicsFromProjection(view.projectionMatrix, cam.width, cam.height);
          intr.views = pose.views.length;
          if (!intr.first) intr.first = intrinsics;
          intr.fxMin = Math.min(intr.fxMin, intrinsics.fx); intr.fxMax = Math.max(intr.fxMax, intrinsics.fx);
          intr.fyMin = Math.min(intr.fyMin, intrinsics.fy); intr.fyMax = Math.max(intr.fyMax, intrinsics.fy);
          if (binding) {
            try {
              const tex = binding.getCameraImage(cam);
              if (tex) {
                if (!reader) { sw = SMALL_W; sh = Math.max(8, Math.round((SMALL_W * cam.height) / cam.width)); reader = makeReader(gl, sw, sh); }
                luma = reader(tex, layer.framebuffer);
                camStat.withImage++;
              }
            } catch (e) { camStat.errors++; camStat.lastError = `${e.name}: ${e.message}`; }
          }
        }
      }
      if (phase === 'record') {
        const o = pose?.transform?.orientation;
        const rec = { t: time, q: o ? [o.x, o.y, o.z, o.w] : null, dx: NaN, dy: NaN, ok: false, sat: false };
        pairShift(rec, prevLuma, luma, sw, sh);
        run.frames.push(rec);
        recFrameTimes.push(time);
      }
      prevLuma = luma;
      gl.bindFramebuffer(gl.FRAMEBUFFER, layer.framebuffer);
      gl.clearColor(0, 0, 0, 0);
      gl.clear(gl.COLOR_BUFFER_BIT);
    };
    session.requestAnimationFrame(onFrame);
  });
}

// --- getUserMedia path --------------------------------------------------------------------------------------------------------
async function runGum() {
  const run = newRun('getUserMedia');
  const stream = await navigator.mediaDevices.getUserMedia({ video: { facingMode: { ideal: 'environment' }, width: { ideal: 1280 }, height: { ideal: 720 }, frameRate: { ideal: 30 } }, audio: false });
  const track = stream.getVideoTracks()[0];
  // deviceId / groupId identify the camera across sessions: keep only their presence, never the values (the export is a file the user shares).
  const { deviceId, groupId, ...settings } = track.getSettings();
  settings.hasDeviceId = !!deviceId; settings.hasGroupId = !!groupId;
  let capabilityKeys = null;
  try { capabilityKeys = Object.keys(track.getCapabilities?.() ?? {}); } catch { /* optional */ }
  const video = document.createElement('video');
  video.muted = true; video.playsInline = true; video.srcObject = stream;
  await video.play();
  const vw = video.videoWidth; const vh = video.videoHeight;
  const sw = SMALL_W; const sh = Math.max(8, Math.round((SMALL_W * vh) / vw));
  const big = document.createElement('canvas'); big.width = sw * 4; big.height = sh * 4;
  const small = document.createElement('canvas'); small.width = sw; small.height = sh;
  const bctx = big.getContext('2d', { willReadFrequently: true });
  const sctx = small.getContext('2d', { willReadFrequently: true });
  bctx.imageSmoothingQuality = 'high'; sctx.imageSmoothingQuality = 'high';
  const stopGyro = await startGyro(run);
  const tsSource = {};
  let prevLuma = null; let t0 = null; let lastUi = 0; let lastMediaTime = -1; let finished = false;
  const recFrameTimes = [];

  return new Promise((resolve) => {
    const finish = (reason) => {
      if (finished) return;
      finished = true;
      run.recording = false;
      stopGyro();
      stopFn = null;
      stream.getTracks().forEach((t) => t.stop());
      const fxNorm = results.xr?.intrinsics?.first ? results.xr.intrinsics.first.fx / results.xr.intrinsics.first.width : null;
      const sum = summarize(run, {
        reason, settings, capabilityKeys, timestampSource: tsSource, smallImage: { width: sw, height: sh },
        fxSmallRef: fxNorm ? fxNorm * sw : null,
        fxRefNote: fxNorm ? 'WebXR camera fx/width assumed equal for this stream (same FOV). If the FOV differs, the amplitude ratio is not comparable.' : 'run the WebXR test first to get a reference focal length',
      });
      sum.fps = fpsStats(recFrameTimes);
      results.gum = sum;
      resolve(sum);
    };
    stopFn = () => finish('user stop');

    const onFrame = (now, metadata) => {
      if (finished) return;
      let t = now; let src = 'callbackNow';
      if (metadata?.captureTime != null) { t = metadata.captureTime; src = 'captureTime'; } else if (metadata?.presentationTime != null) { t = metadata.presentationTime; src = 'presentationTime'; }
      tsSource[src] = (tsSource[src] ?? 0) + 1;
      if (t0 == null) t0 = now;
      const phase = phaseOf(now - t0);
      if (phase === 'done') { buzz([80, 60, 80]); finish('complete'); return; }
      if (phase === 'record' && !run.recording) { run.recording = true; buzz(200); }
      if (now - lastUi > 250) { lastUi = now; setStatus(`getUserMedia ${phase === 'record' ? '기록 중' : '준비'} (${vw}x${vh})`); }
      if (phase === 'record') {
        bctx.drawImage(video, 0, 0, big.width, big.height);
        sctx.drawImage(big, 0, 0, sw, sh);
        const d = sctx.getImageData(0, 0, sw, sh).data;
        const luma = new Uint8Array(sw * sh);
        for (let i = 0; i < luma.length; i++) luma[i] = 0.299 * d[i * 4] + 0.587 * d[i * 4 + 1] + 0.114 * d[i * 4 + 2];
        const rec = { t, q: null, dx: NaN, dy: NaN, ok: false, sat: false };
        pairShift(rec, prevLuma, luma, sw, sh);
        run.frames.push(rec);
        recFrameTimes.push(t);
        prevLuma = luma;
      }
      schedule();
    };
    const schedule = () => {
      if (video.requestVideoFrameCallback) video.requestVideoFrameCallback(onFrame);
      else requestAnimationFrame((now) => { if (video.currentTime !== lastMediaTime) { lastMediaTime = video.currentTime; onFrame(now, null); } else schedule(); });
    };
    schedule();
  });
}

// --- rendering ----------------------------------------------------------------------------------------------------------------
function cell(v) { const td = document.createElement('td'); td.className = 'n'; td.textContent = v; return td; }

function bestOf(run, ref) { return run?.analysis?.refs?.[ref]?.best ?? null; }

function renderCompare() {
  const rows = [];
  const safe = (f) => (r) => (r && r.error && !r.fps ? `오류: ${r.error}` : f(r));
  const add = (label, f) => rows.push([label, safe(f)(results.xr), safe(f)(results.gum)]);
  add('해상도 (카메라)', (r) => (r ? (r.camera ? Object.keys(r.camera.sizes).join(',') || '이미지 없음' : r.settings ? `${r.settings.width}x${r.settings.height}` : '-') : '-'));
  add('fps p50 / p95-dt 기준', (r) => (r ? `${fmt(r.fps.fpsP50, 1)} / ${fmt(r.fps.fpsAtP95Dt, 1)}` : '-'));
  add('프레임 수 (기록) / 유효 shift', (r) => (r ? `${r.shiftFrames.total} / ${r.shiftFrames.validShift} (포화 ${r.shiftFrames.saturated})` : '-'));
  add('카메라 이미지 확보 프레임', (r) => (r?.camera ? `${r.camera.withImage}/${r.camera.frames}` : r ? '(해당 없음)' : '-'));
  add('DeviceMotion 샘플', (r) => (r ? `${r.deviceMotion.samples} (null ${r.deviceMotion.nullSamples})` : '-'));
  const coh = (ref) => (r) => { const b = bestOf(r, ref); return b ? `${fmt(b.bands.mid.coherence)} / ${fmt(b.bands.mid.coherenceW)} [${b.img}~${b.axis}, seg ${b.nSegments}]` : (r?.analysis?.insufficient ?? '-'); };
  add('3-6Hz coherence 평균/가중, vs DeviceMotion', coh('gyro'));
  add('3-6Hz coherence 평균/가중, vs XR pose', coh('pose'));
  const lowhi = (r) => { const b = bestOf(r, 'gyro'); return b ? `1-3: ${fmt(b.bands.low.coherence)}  6-12: ${fmt(b.bands.high.coherence)}` : '-'; };
  add('다른 대역 coherence (DeviceMotion)', lowhi);
  add('진폭비 3-6Hz (영상/자이로, 1.0이면 결합 유지)', (r) => { const b = bestOf(r, 'gyro'); return b ? fmt(b.bands.mid.ampRatio) : '-'; });
  add('유효 fx (small px)  / 기준 fx', (r) => { const b = bestOf(r, 'gyro'); return b ? `${fmt(b.bands.mid.effFxPx, 1)} / ${fmt(r.fxSmallRef, 1)}` : '-'; });
  add('XR pose 기준 진폭비', (r) => { const b = bestOf(r, 'pose'); return b ? fmt(b.bands.mid.ampRatio) : '-'; });
  const table = document.createElement('table');
  const head = table.createTHead().insertRow();
  for (const h of ['항목', 'WebXR', 'getUserMedia']) { const th = document.createElement('th'); th.textContent = h; head.appendChild(th); }
  const body = table.createTBody();
  for (const [label, a, b] of rows) { const tr = body.insertRow(); const th = document.createElement('td'); th.textContent = label; tr.appendChild(th); tr.appendChild(cell(a)); tr.appendChild(cell(b)); }
  $('compare').replaceChildren(table);
  const slim = (r) => r && { ...r, raw: undefined };
  const x = results.xr;
  const lines = [];
  if (x?.intrinsics?.first) {
    const i = x.intrinsics.first;
    lines.push(`WebXR projection 유도 intrinsics (카메라 ${i.width}x${i.height} 기준): fx=${fmt(i.fx, 1)} fy=${fmt(i.fy, 1)} cx=${fmt(i.cx, 1)} cy=${fmt(i.cy, 1)} hfov=${fmt(i.hfovDeg, 1)} vfov=${fmt(i.vfovDeg, 1)} fy/fx=${fmt(i.squarePixelRatio, 3)} (1에서 멀면 투영이 카메라 이미지가 아니라 화면 기준일 수 있음)`);
  }
  lines.push('', JSON.stringify({ xr: slim(x), gum: slim(results.gum) }, null, 1));
  $('detail').textContent = lines.join('\n');
  $('btn-export').disabled = !(results.xr || results.gum);
}

// --- wiring -------------------------------------------------------------------------------------------------------------------
async function guarded(label, fn) {
  if (busy) return;
  busy = true;
  for (const id of ['btn-caps', 'btn-xr', 'btn-gum']) $(id).disabled = true;
  $('stop').disabled = false;
  $('status').classList.remove('bad');
  try {
    setStatus(`${label} 시작`);
    await fn();
    setStatus(`${label} 완료`);
  } catch (e) {
    setStatus(`${label} 실패: ${e.name ?? ''} ${e.message ?? e}`);
    $('status').classList.add('bad');
    results[label === 'WebXR' ? 'xr' : 'gum'] ??= { error: String(e) };
  } finally {
    $('countdown').textContent = '';
    busy = false;
    stopFn = null;
    $('stop').disabled = true;
    for (const id of ['btn-caps', 'btn-xr', 'btn-gum']) $(id).disabled = false;
    renderCompare();
  }
}

$('btn-caps').addEventListener('click', () => guarded('기능 확인', checkCaps));
$('btn-xr').addEventListener('click', () => guarded('WebXR', async () => { const caps = checkCaps(); await runXr(); await caps; }));
$('btn-gum').addEventListener('click', () => guarded('getUserMedia', async () => { const caps = checkCaps(); await runGum(); await caps; }));
$('stop').addEventListener('click', () => stopFn?.());
$('btn-export').addEventListener('click', () => {
  const blob = new Blob([JSON.stringify(results, null, 1)], { type: 'application/json' });
  const a = document.createElement('a');
  a.href = URL.createObjectURL(blob);
  a.download = `webxr-probe-${new Date().toISOString().replace(/[:.]/g, '-')}.json`;
  a.click();
  setTimeout(() => URL.revokeObjectURL(a.href), 5000);
});

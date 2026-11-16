// slam_refact phone page: Start button -> XR8 (deployed SDK + the slam_refact chunk) -> status panel, tap-to-place (hitTest), image targets,
// orientation display, error display, opt-in metrics download. Nothing is stored; nothing leaves the page. The chunk and engine are used as they are.
// Query options (all opt-in): ?keepmap=1|fast 실험용: 지도 유지 엔진 옵션, gate 미통과 | ?record=1&diag=1 위치 기록과 추적 진단 |
// ?fx=&cx=&cy= 초점 거리 진단 override.
import * as THREE from '../../three/three.module.min.js';
import { createMetrics, buildReceipt, readEngineConfigReadback, effectiveMode } from './metrics.js';
import { loadBundledManifest, loadBundledTarget, loadPickedTarget, MAX_TARGETS } from './targets.js';
import { trackingGuide, tapOutcome, createMapLostFilter } from './guidance.js';
import { resolveEngineOptions } from './product-config.js';
import { createHud } from './hud.js';
import { runStartFlow, StartError } from './start-flow.js';
import { detectPlatform, preflight, describeCameraFailure } from '../common/platform.js';

window.THREE = THREE;  // XR8.Threejs reads window.THREE
const $ = (id) => document.getElementById(id);
const ui = {
  start: $('start'), startNote: $('start-note'), startScreen: $('start-screen'), hud: $('hud'), errors: $('errors'), errorBox: $('error-box'), canvas: $('camerafeed'),
  status: $('h-status'), reason: $('h-reason'), points: $('h-points'), ups: $('h-ups'), orient: $('h-orient'), video: $('h-video'), targets: $('h-targets'), msg: $('h-msg'),
  clear: $('b-clear'), recenter: $('b-recenter'), stop: $('b-stop'), restart: $('b-restart'), metrics: $('b-metrics'), panel: $('panel'),
  demo: $('demo-targets'), fJson: $('f-json'), fImage: $('f-image'), add: $('b-add-target'), list: $('target-list'),
};
const RECORD = new URLSearchParams(window.location.search).get('record') === '1';  // opt-in pose log; off = the page and its metrics JSON are unchanged
// record=1 also turns on the chunk's delivery diagnostics (source timing + sensor accounting); &delivery=0 keeps them off so the
// diagnostics' own cost can be measured (ABBA). The option is read when the chunk factory is created, i.e. after Start.
const DELIVERY = RECORD && new URLSearchParams(window.location.search).get('delivery') !== '0';
if (DELIVERY) window.SLAM_REFACT_CHUNK_OPTIONS = { ...(window.SLAM_REFACT_CHUNK_OPTIONS || {}), deliveryDiagnostics: { enabled: true, maxEntries: 256 } };
// Tracker diagnostics (opt-in, default off, only with record=1): &diag=1 turns on the chunk's trackerDiagnostics, so every pose row also
// carries the engine tracker state (map tracked / map discarded / keyframe created) and the per-frame DeviceMotion summary the engine's
// motion rules compare (event count, peak gyro and linear-acceleration magnitude; no raw sample). Without it nothing changes.
const TRACKER_DIAG = RECORD && new URLSearchParams(window.location.search).get('diag') === '1';
if (TRACKER_DIAG) window.SLAM_REFACT_CHUNK_OPTIONS = { ...(window.SLAM_REFACT_CHUNK_OPTIONS || {}), trackerDiagnostics: { enabled: true } };
// Diagnostic focal A/B (opt-in, default off): ?fx=&cx=&cy= (capture pixels, all three) passes engineConfig.intrinsics =
// [fx, fx, cx, cy] through the A-1a chunk option, replacing the engine's device rule. Partial or invalid values start nothing
// with an override and show an error instead. The values go into the metrics JSON as intrinsicsOverride.
const intrinsicsQuery = (() => {
  const q = new URLSearchParams(window.location.search);
  const given = ['fx', 'cx', 'cy'].filter((k) => q.has(k));
  if (!given.length) return { override: null, error: null };
  const [fx, cx, cy] = ['fx', 'cx', 'cy'].map((k) => (q.has(k) && q.get(k).trim() !== '' ? Number(q.get(k)) : NaN));
  if (given.length !== 3 || ![fx, cx, cy].every(Number.isFinite) || !(fx > 0)) {
    return { override: null, error: 'fx, cx, cy를 모두 유한한 숫자로 주어야 합니다 (fx > 0). 초점 override 없이 실행합니다.' };
  }
  return { override: [fx, fx, cx, cy], error: null };
})();
if (intrinsicsQuery.override) {
  const prev = window.SLAM_REFACT_CHUNK_OPTIONS || {};
  window.SLAM_REFACT_CHUNK_OPTIONS = { ...prev, engineConfig: { ...(prev.engineConfig || {}), intrinsics: intrinsicsQuery.override } };
}
// Experimental engine configuration (samples/phone/product-config.js; ?keepmap=1|fast, default = none, both REJECTED by the gate: build/xr-commercial-20261116/verdict-K1-final.json, verdict-K1F-final.json). The only other place
// engineConfig is set is the focal query above. The opt-in modes also turn on the chunk's trackerDiagnostics (latest-only scalar state) so the page can
// tell when the map tracker is lost; &record=1&diag=1 alone does not drive the hint.
const ENGINE_OPT = resolveEngineOptions(window.location.search);
if (ENGINE_OPT.accuracy) {
  const prev = window.SLAM_REFACT_CHUNK_OPTIONS || {};
  window.SLAM_REFACT_CHUNK_OPTIONS = { ...prev, engineConfig: { ...(prev.engineConfig || {}), accuracy: ENGINE_OPT.accuracy }, trackerDiagnostics: prev.trackerDiagnostics || { enabled: true } };
}
const mapLostFilter = createMapLostFilter();
const hud = createHud();
$('start-note').setAttribute('role', 'status');  // not in the HTML: deploy_xr.py anchors on the exact <p id="start-note"> tag
const platform = detectPlatform(navigator);
const metrics = createMetrics(() => performance.now(), { record: RECORD, trackerDiag: TRACKER_DIAG });
let cameraTrack = null;  // the live camera video track (record=1 only: its getSettings() go into the receipt)
const MAX_OBJECTS = 20;
const XR_SRC = '../../engine/xr.js';  // loaded only after the Start click (the chunk registers sensor listeners when it loads)

const state = {
  phase: 'idle',           // idle -> starting -> running -> stopped | error
  startClicked: false,  // true only after the Start click
  status: 'UNSPECIFIED', reason: 'UNSPECIFIED', worldPoints: 0, videoSize: null, screenOrientation: null,
  targets: [],             // [{id, source, label, entry, revoke}] active image targets
  seenTargets: {},         // name -> 'found' | 'lost'
  errors: [], chunkWarnings: [], objectCount: 0,
  notTrackingSince: null,  // performance.now() of the last moment tracking was not NORMAL (page guidance only; not in the metrics JSON)
  mapLost: false,          // keepmap opt-in modes only: the engine map tracker has been lost for a few tracked frames (guidance text only; never touches pose or objects)
};
// Which engine configuration this page ran with (same data as the metrics JSON): engineReadback is what the engine itself reports it uses.
// engineMode = what was requested; engineModeEffective = the same name only when the engine read every sent flag back as effective and not ignored.
const engineInfo = () => {
  const engineReadback = readEngineConfigReadback(window.__slamRefactChunk, ENGINE_OPT.accuracy);
  return { engineMode: ENGINE_OPT.mode, engineModeEffective: effectiveMode(ENGINE_OPT, engineReadback), engineAccuracy: ENGINE_OPT.accuracy, engineReadback };
};
window.__phoneApp = { hud, getEngineInfo: engineInfo, getState: () => JSON.parse(JSON.stringify({ ...state, targets: state.targets.map((t) => ({ id: t.id, source: t.source, label: t.label, name: t.entry.name })) })) };

// ---- errors ----------------------------------------------------------------------------------------------------------------------------
const showError = (where, message) => {
  metrics.error();
  const line = `[${where}] ${message}`;
  state.errors.push(line);
  if (state.errors.length > 20) state.errors.shift();
  ui.errorBox.hidden = false;
  ui.errors.textContent = state.errors.join('\n');
};
$('error-dismiss').addEventListener('click', () => { state.errors = []; ui.errors.textContent = ''; ui.errorBox.hidden = true; });
if (intrinsicsQuery.error) showError('초점', intrinsicsQuery.error);
if (ENGINE_OPT.error) showError('keepmap', ENGINE_OPT.error);
window.addEventListener('error', (e) => showError('오류', e.message));
window.addEventListener('unhandledrejection', (e) => showError('오류', String(e.reason && e.reason.message || e.reason)));
// The chunk reports unsupported cases with console.warn ("[XR] ..."); keep the original behaviour and also show it.
for (const level of ['warn', 'error']) {
  const orig = console[level].bind(console);
  console[level] = (...args) => {
    orig(...args);
    const text = args.map(String).join(' ');
    if (/^\[XR\]|slam_refact/.test(text) && state.chunkWarnings.length < 10 && !state.chunkWarnings.includes(text)) {
      state.chunkWarnings.push(text);
      showError(level === 'warn' ? 'chunk 경고' : 'chunk 오류', text);
    }
  };
}

// ---- screen orientation (display only; the chunk decides what it supports) -------------------------------------------------------------
const readScreenOrientation = () => {
  const o = screen.orientation;
  state.screenOrientation = { type: o ? o.type : (innerWidth > innerHeight ? 'landscape' : 'portrait'), angle: o ? o.angle : (window.orientation ?? null), width: innerWidth, height: innerHeight };
  ui.orient.textContent = `${state.screenOrientation.type} ${state.screenOrientation.angle ?? '?'}° (${innerWidth}×${innerHeight})`;
};
readScreenOrientation();
const onOrientation = () => { readScreenOrientation(); metrics.event('screenOrientation', { type: state.screenOrientation.type, angle: state.screenOrientation.angle }); fitCanvas(); };
window.addEventListener('resize', onOrientation);
if (screen.orientation) screen.orientation.addEventListener('change', onOrientation);

const fitCanvas = () => {
  const dpr = window.devicePixelRatio || 1;
  ui.canvas.width = Math.round(innerWidth * dpr);
  ui.canvas.height = Math.round(innerHeight * dpr);
};

// ---- image targets ---------------------------------------------------------------------------------------------------------------------
let nextTargetId = 1;
const renderTargetList = () => {
  ui.list.replaceChildren(...state.targets.map((t) => {
    const li = document.createElement('li');
    li.textContent = `${t.label} `;
    const rm = document.createElement('button');
    rm.type = 'button'; rm.textContent = '제거';
    rm.addEventListener('click', () => removeTarget(t.id));
    li.append(rm);
    return li;
  }));
  ui.targets.textContent = state.targets.length
    ? state.targets.map((t) => `${t.entry.name}: ${state.seenTargets[t.entry.name] === 'found' ? '보임' : '찾는 중'}`).join(', ')
    : '없음';
};
const applyTargets = () => {
  renderTargetList();
  if (state.phase !== 'running' && state.phase !== 'starting') return;  // applied at Start
  try { XR8.XrController.configure({ imageTargetData: state.targets.map((t) => t.entry) }); } catch (e) { showError('타깃', e.message); }
};
const addTarget = (source, label, entry, revoke) => {
  if (state.targets.some((t) => t.entry.name === entry.name)) { revoke && revoke(); showError('타깃', `같은 이름의 타깃이 이미 있습니다: ${entry.name}`); return false; }
  if (state.targets.length >= MAX_TARGETS) { revoke && revoke(); showError('타깃', `타깃은 최대 ${MAX_TARGETS}개입니다`); return false; }
  state.targets.push({ id: nextTargetId++, source, label, entry, revoke });
  applyTargets();
  return true;
};
function removeTarget(id) {
  const t = state.targets.find((x) => x.id === id);
  if (!t) return;
  state.targets = state.targets.filter((x) => x.id !== id);
  if (t.source === 'demo') { const box = ui.demo.querySelector(`input[data-name="${CSS.escape(t.entry.name)}"]`); if (box) box.checked = false; }
  applyTargets();
  if (t.revoke) setTimeout(t.revoke, 2000);  // after the engine has had the chance to drop it
  delete state.seenTargets[t.entry.name];
  disposeTargetObject(t.entry.name);
  renderTargetList();
}
ui.add.addEventListener('click', async () => {
  try {
    const { entry, revoke } = await loadPickedTarget(ui.fJson.files[0], ui.fImage.files[0]);
    addTarget('file', `${entry.name} (파일)`, entry, revoke);
    ui.fJson.value = ''; ui.fImage.value = '';
  } catch (e) { showError('타깃', e.message); }
});
(async () => {
  try {
    const items = await loadBundledManifest();
    if (!items.length) { ui.demo.textContent = '번들된 데모 타깃이 아직 없습니다(아래에서 파일을 직접 선택할 수 있습니다).'; return; }
    ui.demo.replaceChildren(...items.map((item) => {
      const label = document.createElement('label');
      const box = document.createElement('input');
      box.type = 'checkbox'; box.dataset.name = item.name;
      box.addEventListener('change', async () => {
        if (!box.checked) { const t = state.targets.find((x) => x.source === 'demo' && x.label === item.label); if (t) removeTarget(t.id); return; }
        try { const entry = await loadBundledTarget(item); if (!addTarget('demo', item.label, entry, null)) box.checked = false; } catch (e) { box.checked = false; showError('타깃', e.message); }
      });
      label.append(box, ' ', item.label);
      return label;
    }));
  } catch (e) { ui.demo.textContent = '번들 타깃 목록을 읽지 못했습니다.'; showError('타깃', e.message); }
})();

// ---- three.js scene --------------------------------------------------------------------------------------------------------------------
let scene = null, objects = [];
const targetMeshes = new Map();  // name -> THREE.Group
const hideTargetObject = (name) => { const g = targetMeshes.get(name); if (g) g.visible = false; };
const disposeTargetObject = (name) => { const g = targetMeshes.get(name); if (!g) return; scene && scene.remove(g); g.children.forEach((m) => { m.geometry.dispose(); m.material.dispose(); }); targetMeshes.delete(name); };
const placeObject = (position) => {
  const mesh = new THREE.Mesh(new THREE.BoxGeometry(0.15, 0.15, 0.15), new THREE.MeshBasicMaterial({ color: 0xff00ff }));
  mesh.position.set(position.x, position.y + 0.075, position.z);
  scene.add(mesh);
  objects.push(mesh);
  if (objects.length > MAX_OBJECTS) { const old = objects.shift(); scene.remove(old); old.geometry.dispose(); old.material.dispose(); }
  metrics.placed();
  state.objectCount = objects.length;
  hud.setCount(state.objectCount);
};
const clearObjects = () => { for (const o of objects) { scene.remove(o); o.geometry.dispose(); o.material.dispose(); } objects = []; state.objectCount = 0; hud.setCount(0); hud.flash({ tone: 'ok', title: '물체를 지웠어요', body: '화면을 탭하면 다시 놓을 수 있어요.' }, 2500); };
const updateTargetObject = (d) => {
  if (!scene) return;
  let g = targetMeshes.get(d.name);
  if (!g) {
    g = new THREE.Group();
    g.add(new THREE.Mesh(new THREE.PlaneGeometry(1, 1), new THREE.MeshBasicMaterial({ color: 0x00e5ff, transparent: true, opacity: 0.35, side: THREE.DoubleSide })));
    scene.add(g); targetMeshes.set(d.name, g);
  }
  const w = d.scaledWidth || 1, h = d.scaledHeight || 1;
  g.children[0].scale.set(w, h, 1);
  g.position.set(d.position.x, d.position.y, d.position.z);
  g.quaternion.set(d.rotation.x, d.rotation.y, d.rotation.z, d.rotation.w);
  g.scale.setScalar(d.scale || 1);
  g.visible = true;
};

// tap -> hit test (FEATURE_POINT). distance 0 means "no feature near the ray", not a surface (see samples/threejs/app.js).
const onTap = (event) => {
  if (state.phase !== 'running' || !scene) return;
  const rect = ui.canvas.getBoundingClientRect();
  const hits = XR8.XrController.hitTest((event.clientX - rect.left) / rect.width, (event.clientY - rect.top) / rect.height, ['FEATURE_POINT']);
  const ok = hits.length > 0 && hits[0].distance > 0;
  metrics.hit(ok);
  hud.tapRing(event.clientX, event.clientY, ok);
  if (!ok) { hud.flash(tapOutcome(false)); return; }
  placeObject(hits[0].position);
  hud.flash(tapOutcome(true, { objectCount: state.objectCount }));
};
const onCanvasKey = (event) => {
  if (event.key !== 'Enter' && event.key !== ' ') return;
  event.preventDefault();
  const rect = ui.canvas.getBoundingClientRect();
  onTap({ clientX: rect.left + rect.width / 2, clientY: rect.top + rect.height / 2 });
};

// ---- record=1: receipt and HUD hint ----------------------------------------------------------------------------------------------------
const makeReceipt = () => buildReceipt({
  track: cameraTrack, win: window, nav: navigator, scr: screen, canvas: ui.canvas,
  fetchFn: typeof fetch === 'function' ? fetch.bind(window) : null, subtle: window.crypto && window.crypto.subtle || null,
});
let recHud = null;
if (RECORD) {
  recHud = document.createElement('div');
  recHud.id = 'h-rec'; recHud.setAttribute('role', 'status');
  // Below the status pill (#hud is a fixed full-screen layer; an in-flow element would sit under the absolutely placed pill).
  recHud.style.cssText = 'position:absolute;top:calc(max(16px, env(safe-area-inset-top)) + 44px);left:max(16px, env(safe-area-inset-left));right:max(16px, env(safe-area-inset-right));padding:4px 8px;border-radius:8px;background:rgba(160,0,0,.75);color:#fff;font-size:12px;pointer-events:none';
  recHud.textContent = '기록 중 (record=1): 위치 시계열을 이 페이지 메모리에만 모읍니다. 도구 > 지표 JSON 받기를 눌러야 파일로 내려받습니다.';
  ui.hud.append(recHud);
}
const updateRecHud = () => {
  if (!recHud) return;
  const c = metrics.poseLogCounts();
  const td = TRACKER_DIAG && window.__slamRefactChunk ? window.__slamRefactChunk.trackerDiag : null;
  const diagText = !TRACKER_DIAG ? '' : td ? ` 진단(diag=1): 지도 reset ${td.totalMapResets}회(첫 지도 생성 포함), keyframe ${td.numKeyframes}개.` : ' 진단(diag=1): 아직 값 없음.';
  recHud.textContent = `기록 중 (record=1): ${c.rows}행${c.droppedRows ? `, 한도 초과로 ${c.droppedRows}행 버려짐` : ''}. 메모리에만 있고, 지표 JSON 받기를 눌러야 내려받습니다.${diagText}`;
};

// ---- XR8 pipeline ----------------------------------------------------------------------------------------------------------------------
let upsCount = 0, upsWindowStart = 0;
const setImageSeen = (name, v) => { state.seenTargets[name] = v; renderTargetList(); };
const pageModule = () => ({
  name: 'phone-page',
  listeners: [
    { event: 'reality.imagefound', process: ({ detail }) => { metrics.image('found'); metrics.event('imagefound', { name: detail.name }); setImageSeen(detail.name, 'found'); updateTargetObject(detail); } },
    { event: 'reality.imageupdated', process: ({ detail }) => { metrics.image('updated'); updateTargetObject(detail); } },
    { event: 'reality.imagelost', process: ({ detail }) => { metrics.image('lost'); metrics.event('imagelost', { name: detail.name }); setImageSeen(detail.name, 'lost'); hideTargetObject(detail.name); } },
  ],
  onStart: (args) => {
    if (args && args.videoWidth) { state.videoSize = { videoWidth: args.videoWidth, videoHeight: args.videoHeight, orientation: args.orientation }; showVideo(); }
    const x = XR8.Threejs.xrScene();
    scene = x.scene;
    x.renderer.setClearColor(0x000000, 0);
    x.camera.position.set(0, 1.5, 0);
    XR8.XrController.updateCameraProjectionMatrix({ origin: x.camera.position, facing: x.camera.quaternion });
    ui.canvas.addEventListener('click', onTap);
    ui.canvas.addEventListener('keydown', onCanvasKey);
    ui.canvas.tabIndex = 0;
    metrics.start(); upsWindowStart = performance.now();
    if (RECORD) makeReceipt().then((r) => metrics.setReceiptAtStart(r), (e) => metrics.setReceiptAtStart({ unavailable: String(e && e.message || e) }));
    state.phase = 'running';
    state.notTrackingSince = performance.now();
    ui.startScreen.hidden = true; ui.hud.hidden = false; ui.panel.hidden = false; hud.clearStartSteps();
    renderGuide();
    for (const b of [ui.clear, ui.recenter, ui.stop, ui.metrics]) b.disabled = false;
  },
  onCameraStatusChange: ({ status, stream, video }) => {
    if (!RECORD) return;
    const src = status === 'hasStream' ? stream : status === 'hasVideo' ? video && video.srcObject : null;
    const track = src && src.getVideoTracks && src.getVideoTracks()[0];
    if (track) cameraTrack = track;
  },
  onVideoSizeChange: ({ videoWidth, videoHeight }) => { state.videoSize = { ...(state.videoSize || {}), videoWidth, videoHeight }; showVideo(); },
  onDeviceOrientationChange: ({ videoWidth, videoHeight, orientation }) => {
    state.videoSize = { videoWidth, videoHeight, orientation };
    metrics.event('deviceOrientationChange', { videoWidth, videoHeight, orientation });
    showVideo();
  },
  onException: (error) => {
    const detail = error && error.message || String(error);
    showError('카메라 또는 엔진', detail);
    state.phase = 'error'; ui.canvas.tabIndex = -1;
    renderGuide();
    for (const b of [ui.clear, ui.recenter]) b.disabled = true;
    ui.stop.disabled = false; ui.restart.hidden = false;
    hud.showRecovery(describeCameraFailure(error, platform), detail);
  },
  onUpdate: ({ processCpuResult, frameStartResult }) => {
    if (state.phase !== 'running') return;
    const r = processCpuResult && processCpuResult.reality;
    const pts = r && r.worldPoints ? r.worldPoints.length : 0;
    const chunk = window.__slamRefactChunk;  // sourceTiming: latest-only scalar copy (null when delivery diagnostics are off)
    metrics.update(r || null, pts, frameStartResult, chunk && chunk.stats ? chunk.stats.lastSourceFrameIndex : null, RECORD && chunk ? chunk.sourceTiming : null,
      TRACKER_DIAG && chunk ? chunk.trackerDiag : null);
    if (!r) return;
    if (r.trackingStatus !== state.status || r.trackingReason !== state.reason) {
      state.status = r.trackingStatus; state.reason = r.trackingReason;
      metrics.transition(state.status, state.reason);
    }
    if (state.status === 'NORMAL') state.notTrackingSince = null; else if (state.notTrackingSince === null) state.notTrackingSince = performance.now();
    state.worldPoints = pts;
    state.mapLost = !!ENGINE_OPT.accuracy && mapLostFilter.update(chunk && chunk.trackerDiag);  // default mode or no tracker state (Worker / scheduler path): no hint
    upsCount++;
    const t = performance.now();
    if (t - upsWindowStart >= 1000) { updateRecHud(); ui.ups.textContent = String(Math.round(upsCount * 1000 / (t - upsWindowStart))); upsCount = 0; upsWindowStart = t; }
    hud.setReason(state.reason);
    ui.points.textContent = String(pts);
    renderGuide();
  },
  onDetach: () => { ui.canvas.tabIndex = -1; ui.canvas.removeEventListener('click', onTap); ui.canvas.removeEventListener('keydown', onCanvasKey); },
});
// One place that turns the page state into the guidance card, pill and reticle.
function renderGuide() {
  hud.render(trackingGuide({
    phase: state.phase, status: state.status, reason: state.reason, points: state.worldPoints, objectCount: state.objectCount,
    landscape: !!(state.screenOrientation && /landscape/.test(state.screenOrientation.type)), mapLost: state.mapLost,
    notTrackingMs: state.notTrackingSince === null ? 0 : performance.now() - state.notTrackingSince,
  }));
}
const showVideo = () => {
  const v = state.videoSize;
  ui.video.textContent = v ? `영상 ${v.videoWidth}×${v.videoHeight}${v.orientation !== undefined ? ` · 회전 ${v.orientation}` : ''}` : '';
};

// ---- Start / Stop ----------------------------------------------------------------------------------------------------------------------
const loadXr8 = () => new Promise((resolve, reject) => {
  if (window.XR8) { resolve(); return; }
  window.addEventListener('xrloaded', () => resolve(), { once: true });
  document.querySelectorAll('script[data-xr-loader]').forEach((e) => e.remove());
  const el = document.createElement('script');
  el.dataset.xrLoader = '1';
  el.src = XR_SRC; el.async = true; el.dataset.preloadChunks = 'slam';
  el.onerror = () => reject(new Error('엔진(xr.js)을 불러오지 못했습니다. 서버 주소와 연결을 확인하세요.'));
  document.head.append(el);
});
// Requirements that cannot be fixed on this page (HTTPS, camera API, WASM SIMD) disable Start up front and say why.
const blocked = preflight({ win: window, nav: navigator, wa: typeof WebAssembly === 'undefined' ? null : WebAssembly });
if (blocked) { ui.start.disabled = true; hud.startAlert(blocked, 'block'); }
else if (platform.inAppBrowser) hud.startAlert({ title: '앱 안의 브라우저로 열려 있어요', body: '카메라가 막힐 수 있어요. 안 되면 Safari나 Chrome에서 이 주소를 직접 열어 주세요.' }, 'info');

const startRun = async () => {
  if (state.phase !== 'idle') return;
  state.errors = []; ui.errors.textContent = ''; ui.errorBox.hidden = true; ui.startNote.textContent = '';
  state.phase = 'starting'; state.startClicked = true;
  ui.start.disabled = true;
  hud.startStep('permission');
  try {
    // runStartFlow requests the iOS motion permissions synchronously inside this tap (before its first await), then loads xr.js, then runs.
    await runStartFlow({ win: window, nav: navigator, wa: typeof WebAssembly === 'undefined' ? null : WebAssembly }, {
      onStep: (name) => hud.startStep(name),
      loadXr8,
      run: () => {
        fitCanvas();
        XR8.XrController.configure({ enableWorldPoints: true, imageTargetData: state.targets.map((t) => t.entry) });
        XR8.addCameraPipelineModules([
          XR8.GlTextureRenderer.pipelineModule(),
          XR8.Threejs.pipelineModule(),
          XR8.XrController.pipelineModule(),
          pageModule(),
        ]);
        XR8.run({ canvas: ui.canvas });  // this is where the camera is requested (the camera permission prompt comes after the Start tap)
      },
    });
  } catch (e) {
    try { if (window.XR8) { XR8.stop(); XR8.clearCameraPipelineModules(); } } catch (_) { /* nothing was running */ }
    state.phase = 'idle'; state.startClicked = false;
    ui.startScreen.hidden = false; ui.hud.hidden = true; ui.panel.hidden = true; ui.start.disabled = false;
    hud.clearStartSteps();
    showError('시작', e.message);
    if (e instanceof StartError) {
      ui.errorBox.hidden = true;  // the start screen / recovery sheet already says it
      if (/^motion-/.test(e.info.code)) hud.showRecovery(e.info, e.message);
      else { hud.startAlert(e.info, 'block'); ui.start.disabled = true; }
    } else ui.startNote.textContent = '시작하지 못했어요. 연결을 확인하고 다시 눌러 주세요. 자세한 내용은 화면 위쪽 오류 상자에 있어요.';
  }
};
ui.start.addEventListener('click', startRun);
ui.stop.addEventListener('click', () => {
  try { XR8.stop(); } catch (e) {
    showError('카메라 종료', e.message);
    state.phase = 'error';
    hud.flash({ tone: 'error', pill: 'error', title: '카메라를 끄지 못했어요', body: '페이지를 다시 열어 주세요.' }, 600000);
    ui.restart.hidden = false; ui.panel.open = true;
    return;
  }
  state.phase = 'stopped';
  ui.canvas.tabIndex = -1;
  for (const b of [ui.clear, ui.recenter, ui.stop]) b.disabled = true;
  renderGuide();
  ui.restart.hidden = false; ui.panel.open = true;
});
ui.restart.addEventListener('click', () => window.location.reload());
$('rec-primary').addEventListener('click', () => window.location.reload());
ui.clear.addEventListener('click', () => scene && clearObjects());
ui.recenter.addEventListener('click', () => { try { XR8.XrController.recenter(); hud.flash({ tone: 'ok', title: '원점을 다시 잡았어요', body: '카메라를 원점으로 되돌렸어요.' }, 2500); } catch (e) { showError('원점', e.message); } });
// The only place data leaves the page: the user clicks, the browser downloads a small JSON file to the user's own device.
ui.metrics.addEventListener('click', async () => {
  const receiptNow = RECORD ? await makeReceipt().catch((e) => ({ unavailable: String(e && e.message || e) })) : null;  // receipt again at download: the session may have changed since the start
  const json = metrics.snapshot({ createdAtMs: Date.now(), userAgent: navigator.userAgent, screen: state.screenOrientation, video: state.videoSize, camera: window.XrCameraResolution || null, finalStatus: state.status, finalReason: state.reason, chunkWarnings: state.chunkWarnings, intrinsicsOverride: intrinsicsQuery.override,
    ...engineInfo() }, receiptNow);
  const url = URL.createObjectURL(new Blob([JSON.stringify(json, null, 1)], { type: 'application/json' }));
  const a = document.createElement('a');
  a.href = url; a.download = `slam-refact-phone-metrics-${new Date().toISOString().replace(/[:.]/g, '-')}.json`;
  document.body.append(a); a.click(); a.remove();
  setTimeout(() => URL.revokeObjectURL(url), 5000);
});

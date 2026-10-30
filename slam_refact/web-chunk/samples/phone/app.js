// slam_refact phone page: Start button -> XR8 (deployed SDK + the slam_refact chunk) -> status panel, tap-to-place (hitTest), image targets,
// orientation display, error display, opt-in metrics download. Nothing is stored; nothing leaves the page. The chunk and engine are used as they are.
import * as THREE from '../../three/three.module.min.js';
import { createMetrics } from './metrics.js';
import { loadBundledManifest, loadBundledTarget, loadPickedTarget, MAX_TARGETS } from './targets.js';

window.THREE = THREE;  // XR8.Threejs reads window.THREE
const $ = (id) => document.getElementById(id);
const ui = {
  start: $('start'), startNote: $('start-note'), startScreen: $('start-screen'), hud: $('hud'), errors: $('errors'), errorBox: $('error-box'), canvas: $('camerafeed'),
  status: $('h-status'), reason: $('h-reason'), points: $('h-points'), ups: $('h-ups'), orient: $('h-orient'), video: $('h-video'), targets: $('h-targets'), msg: $('h-msg'),
  clear: $('b-clear'), recenter: $('b-recenter'), stop: $('b-stop'), restart: $('b-restart'), metrics: $('b-metrics'), panel: $('panel'),
  demo: $('demo-targets'), fJson: $('f-json'), fImage: $('f-image'), add: $('b-add-target'), list: $('target-list'),
};
const metrics = createMetrics();
const MAX_OBJECTS = 20;
const XR_SRC = '../../engine/xr.js';  // loaded only after the Start click (the chunk registers sensor listeners when it loads)

const state = {
  phase: 'idle',           // idle -> starting -> running -> stopped | error
  startClicked: false,  // true only after the Start click
  status: 'UNSPECIFIED', reason: 'UNSPECIFIED', worldPoints: 0, videoSize: null, screenOrientation: null,
  targets: [],             // [{id, source, label, entry, revoke}] active image targets
  seenTargets: {},         // name -> 'found' | 'lost'
  errors: [], chunkWarnings: [], objectCount: 0,
};
window.__phoneApp = { getState: () => JSON.parse(JSON.stringify({ ...state, targets: state.targets.map((t) => ({ id: t.id, source: t.source, label: t.label, name: t.entry.name })) })) };

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
};
const clearObjects = () => { for (const o of objects) { scene.remove(o); o.geometry.dispose(); o.material.dispose(); } objects = []; state.objectCount = 0; ui.msg.textContent = '물체를 지웠습니다. 화면을 탭하면 다시 놓습니다.'; };
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
  if (!ok) { ui.msg.textContent = '탭한 곳에서 기준점을 찾지 못했습니다. 휴대폰을 천천히 움직인 뒤 다시 시도하세요.'; return; }
  placeObject(hits[0].position);
  ui.msg.textContent = `물체를 놓았습니다(거리 ${hits[0].distance.toFixed(2)}). 다시 탭하면 하나 더 놓습니다.`;
};
const onCanvasKey = (event) => {
  if (event.key !== 'Enter' && event.key !== ' ') return;
  event.preventDefault();
  const rect = ui.canvas.getBoundingClientRect();
  onTap({ clientX: rect.left + rect.width / 2, clientY: rect.top + rect.height / 2 });
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
    state.phase = 'running';
    ui.msg.textContent = '휴대폰을 천천히 움직여 주변을 살펴보세요.';
    for (const b of [ui.clear, ui.recenter, ui.stop, ui.metrics]) b.disabled = false;
  },
  onVideoSizeChange: ({ videoWidth, videoHeight }) => { state.videoSize = { ...(state.videoSize || {}), videoWidth, videoHeight }; showVideo(); },
  onDeviceOrientationChange: ({ videoWidth, videoHeight, orientation }) => {
    state.videoSize = { videoWidth, videoHeight, orientation };
    metrics.event('deviceOrientationChange', { videoWidth, videoHeight, orientation });
    showVideo();
  },
  onException: (error) => { showError('카메라 또는 엔진', error && error.message || String(error)); state.phase = 'error'; ui.canvas.tabIndex = -1; ui.status.textContent = '시작할 수 없음'; ui.status.parentElement.dataset.state = 'error'; ui.msg.textContent = '권한과 연결을 확인한 뒤 다시 시작하세요.'; for (const b of [ui.clear, ui.recenter]) b.disabled = true; ui.stop.disabled = false; ui.restart.hidden = false; ui.panel.open = true; },
  onUpdate: ({ processCpuResult }) => {
    if (state.phase !== 'running') return;
    const r = processCpuResult && processCpuResult.reality;
    const pts = r && r.worldPoints ? r.worldPoints.length : 0;
    metrics.update(r || null, pts);
    if (!r) return;
    if (r.trackingStatus !== state.status || r.trackingReason !== state.reason) {
      state.status = r.trackingStatus; state.reason = r.trackingReason;
      metrics.transition(state.status, state.reason);
      ui.msg.textContent = state.status === 'NORMAL' ? '화면을 탭해 물체를 놓아보세요.' : '휴대폰을 천천히 움직여 주변을 살펴보세요.';
    }
    state.worldPoints = pts;
    upsCount++;
    const t = performance.now();
    if (t - upsWindowStart >= 1000) { ui.ups.textContent = String(Math.round(upsCount * 1000 / (t - upsWindowStart))); upsCount = 0; upsWindowStart = t; }
    ui.status.textContent = state.status === 'NORMAL' ? '추적 중' : state.status === 'LIMITED' ? '주변을 찾는 중' : '준비 중';
    ui.status.parentElement.dataset.state = state.status === 'NORMAL' ? 'normal' : 'pending';
    ui.reason.textContent = state.reason && state.reason !== 'UNSPECIFIED' ? `(${state.reason})` : '';
    ui.points.textContent = String(pts);
    if (state.status !== 'NORMAL' && state.screenOrientation && /landscape/.test(state.screenOrientation.type)) {
      ui.msg.textContent = '가로 화면은 아직 실기기 검증 전이라 추적이 멈출 수 있습니다. 폰을 세로로 돌려 보세요.';
    }
  },
  onDetach: () => { ui.canvas.tabIndex = -1; ui.canvas.removeEventListener('click', onTap); ui.canvas.removeEventListener('keydown', onCanvasKey); },
});
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
const startRun = async () => {
  if (state.phase !== 'idle') return;
  state.errors = []; ui.errors.textContent = ''; ui.errorBox.hidden = true; ui.startNote.textContent = '';
  state.phase = 'starting'; state.startClicked = true;
  ui.start.disabled = true;
  ui.startNote.textContent = '엔진을 불러오는 중…';
  try {
    // iOS Safari only: the motion permission prompt needs the tap that started this function (other browsers have no such function).
    if (typeof DeviceMotionEvent !== 'undefined' && typeof DeviceMotionEvent.requestPermission === 'function') {
      const p = await DeviceMotionEvent.requestPermission();
      if (p !== 'granted') throw new Error('모션 센서 권한이 허용되지 않았습니다');
    }
    await loadXr8();
    ui.startScreen.hidden = true; ui.hud.hidden = false; ui.panel.hidden = false;
    fitCanvas();
    XR8.XrController.configure({ enableWorldPoints: true, imageTargetData: state.targets.map((t) => t.entry) });
    XR8.addCameraPipelineModules([
      XR8.GlTextureRenderer.pipelineModule(),
      XR8.Threejs.pipelineModule(),
      XR8.XrController.pipelineModule(),
      pageModule(),
    ]);
    XR8.run({ canvas: ui.canvas });  // this is where the camera is requested (permission prompt comes after the Start tap)
  } catch (e) {
    try { if (window.XR8) { XR8.stop(); XR8.clearCameraPipelineModules(); } } catch (_) { /* nothing was running */ }
    state.phase = 'idle'; state.startClicked = false;
    ui.startScreen.hidden = false; ui.hud.hidden = true; ui.panel.hidden = true; ui.start.disabled = false;
    ui.startNote.textContent = '시작하지 못했습니다. 아래 오류를 확인하세요.';
    showError('시작', e.message);
  }
};
ui.start.addEventListener('click', startRun);
ui.stop.addEventListener('click', () => {
  try { XR8.stop(); } catch (e) {
    showError('카메라 종료', e.message);
    state.phase = 'error';
    ui.status.textContent = '종료 실패'; ui.status.parentElement.dataset.state = 'error';
    ui.msg.textContent = '카메라를 종료하지 못했습니다. 페이지를 다시 열어주세요.';
    ui.restart.hidden = false; ui.panel.open = true;
    return;
  }
  state.phase = 'stopped';
  ui.canvas.tabIndex = -1;
  for (const b of [ui.clear, ui.recenter, ui.stop]) b.disabled = true;
  ui.status.textContent = '종료 요청됨'; ui.status.parentElement.dataset.state = 'pending';
  ui.msg.textContent = '카메라 종료를 요청했습니다. 다시 시작할 수 있습니다.';
  ui.restart.hidden = false; ui.panel.open = true;
});
ui.restart.addEventListener('click', () => window.location.reload());
ui.clear.addEventListener('click', () => scene && clearObjects());
ui.recenter.addEventListener('click', () => { try { XR8.XrController.recenter(); ui.msg.textContent = '원점을 다시 잡았습니다.'; } catch (e) { showError('원점', e.message); } });
// The only place data leaves the page: the user clicks, the browser downloads a small JSON file to the user's own device.
ui.metrics.addEventListener('click', () => {
  const json = metrics.snapshot({ createdAtMs: Date.now(), userAgent: navigator.userAgent, screen: state.screenOrientation, video: state.videoSize, finalStatus: state.status, finalReason: state.reason, chunkWarnings: state.chunkWarnings });
  const url = URL.createObjectURL(new Blob([JSON.stringify(json, null, 1)], { type: 'application/json' }));
  const a = document.createElement('a');
  a.href = url; a.download = `slam-refact-phone-metrics-${new Date().toISOString().replace(/[:.]/g, '-')}.json`;
  document.body.append(a); a.click(); a.remove();
  setTimeout(() => URL.revokeObjectURL(url), 5000);
});

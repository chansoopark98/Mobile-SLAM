// Pure helpers of the slam_refact measurement page (web/slam-refact.html). No DOM, no network:
// shared by web/js/slam-refact-app.js and the node contract tests
// (tests/browser/slam-refact-page-contracts.test.mjs).

// Capture geometry fed to the engine: portrait 480x640 like the Flip6 replays (R16: rotation 0,
// width < height). The engine aborts on any other frame size, so every frame is resampled to this.
export const CAPTURE_WIDTH = 480;
export const CAPTURE_HEIGHT = 640;

// Device model string for the K rule (S1 §5.12), taken from the user agent like the original page.
// Chrome on Android sends the reduced UA "Linux; Android 10; K", so the Flip6 yields "K" (the value
// the Flip6 replays used; K falls back to 506.41 px, rule F-K).
export function deviceModelFromUserAgent(ua) {
  const m = /\bAndroid [^;)]*;\s*([^;)]+?)(?:\s+Build\/[^;)]*)?\)/.exec(String(ua ?? ''));
  return m ? m[1].trim() : '';
}

// Accuracy mode (engine contract ACC-1..4, slam_refact/docs/reports/accuracy.md). Default on:
//   metric output     positions in metres from a continuous accelerometer scale estimate (before it
//                     converges: camera height 1.4 m assumed; pose.metric.initialized tells which)
//   surfaces          floor / horizontal surface detection and hit test
//   device focal      focal of the selected back camera for known devices (cameraFocalFor; the
//                     engine's device table otherwise)
// ?accuracy=0 runs the bit-exact original behaviour; ?surfaces=0 turns the surface detector off;
// ?fx=<px> overrides the focal length of the 480x640 capture (diagnostic).
export function accuracyConfigFor(search) {
  const q = new URLSearchParams(search ?? '');
  const enabled = q.get('accuracy') !== '0';
  return { enabled, metricOutput: enabled && q.get('scale') !== 'absolute', localBa: false,
    multiviewTriangulation: false };
}

// Engine configuration (SlamRefactGlue constructor) plus the worker's frontend choice. Query
// overrides are diagnostic only:
//   ?frontend=cpu                 CPU port of the gr8 shaders on the colour frame (default: gpu, the
//                                 original's WebGL2 pyramid injected into the engine)
//   ?gpu-check=1                  compare the GPU atlas with the CPU port atlas on sampled frames
//   ?pyramid=swiftshader-approx   CPU-port numeric model (default: ieee, the engine default)
//   ?scale=absolute               metric scale estimation (default: responsive, origin y = 2)
//   ?record=1                     explicit opt-in input recorder (frames + sensors stored in this
//                                 device's OPFS, never sent; default off, see slam-refact-recorder.js)
// uaModel: navigator.userAgentData high-entropy `model` (e.g. "SM-F741N") when the browser gives it;
// the reduced UA model ("K") otherwise.
export function engineConfigFor(search, ua, uaModel = '') {
  const q = new URLSearchParams(search ?? '');
  const pyramid = q.get('pyramid') === 'swiftshader-approx' ? 'swiftshader-approx' : 'ieee';
  const estimateScale = q.get('scale') === 'absolute';
  const fx = Number(q.get('fx'));
  return {
    estimateScale, disableVio: false, captureWidth: CAPTURE_WIDTH, captureHeight: CAPTURE_HEIGHT,
    rotationDeg: 0, manufacturer: '', model: String(uaModel || '') || deviceModelFromUserAgent(ua),
    pyramidModel: pyramid, origin: [0, 2, 0], leftHandedAxes: false, mirroredDisplay: false,
    accuracy: accuracyConfigFor(search),
    detectSurfaces: q.get('surfaces') !== '0',
    intrinsics: Number.isFinite(fx) && fx > 0
      ? [fx, fx, (CAPTURE_WIDTH - 1) / 2, (CAPTURE_HEIGHT - 1) / 2] : null,
    frontend: q.get('frontend') === 'cpu' ? 'cpu' : 'gpu',
    gpuCheck: q.get('gpu-check') === '1' ? { every: 10, frames: 30 } : null,
    record: q.get('record') === '1',
  };
}

// Back camera selection. Chrome Android labels its cameras "camera <n>, facing back" (older
// builds: "camera2 <n>, facing back"); <n> is the Android camera id, and id 0 is the primary back
// camera by Android convention. facingMode 'environment' alone lets Chrome pick any back camera: the
// Flip6 run 1 (2026-10-19) got "camera 2, facing back", a wider stream (focal about 310 px at
// 480x640, slam_refact/docs/reports/accuracy.md §10.2). The page therefore picks a camera by id:
//   (default)       the lowest-numbered back camera (the main camera, normally id 0)
//   ?camera=<n>     back camera id n (diagnostic)
//   ?camera=default facingMode 'environment' only, the browser's choice (the old behaviour; with
//                   ?accuracy=0 this is what reproduces the original page input exactly)
export function cameraRequestFor(search) {
  const v = new URLSearchParams(search ?? '').get('camera');
  if (v === 'default') return { mode: 'default', number: null };
  if (v !== null && /^(0|[1-9]\d*)$/.test(v)) return { mode: 'number', number: Number(v) };
  return { mode: 'main', number: null };
}

// Video size requests, tried in order until getUserMedia accepts one (diagnostic ?res=):
//   (default)   640x480 ideal, 30 fps ideal (the page's measured configuration)
//   ?res=8w     the original reference runtime Android cascade (reality/app/xr/js/src/constraints-helper.ts):
//               960x720 exact first ("preferred android resolution for performance"), then min
//               1920x1440, 1600x1200, 1440x1080, 1280x960, 960x720, 640x480, 320x240 (no frame rate)
//   ?res=WxH    exactly W x H (no fallback), e.g. ?res=1920x1440
// The stream size changes the field of view on some phones (reference runtime forces sizes per model for
// that reason) and may change the stabilization; the camera table focal applies only to the
// default 640x480 stream (configForCamera falls back for resampled streams).
export const XR_REFERENCE_ANDROID_RESOLUTIONS = [
  { width: { exact: 960 }, height: { exact: 720 } },
  { width: { min: 1920 }, height: { min: 1440 } },
  { width: { min: 1600 }, height: { min: 1200 } },
  { width: { min: 1440 }, height: { min: 1080 } },
  { width: { min: 1280 }, height: { min: 960 } },
  { width: { min: 960 }, height: { min: 720 } },
  { width: { min: 640 }, height: { min: 480 } },
  { width: { min: 320 }, height: { min: 240 } },
];
export function videoRequestsFor(search) {
  const v = new URLSearchParams(search ?? '').get('res');
  if (v === '8w') return XR_REFERENCE_ANDROID_RESOLUTIONS.map((r) => ({ ...r }));
  const m = /^([1-9]\d{1,4})x([1-9]\d{1,4})$/.exec(v ?? '');
  if (m) return [{ width: { exact: Number(m[1]) }, height: { exact: Number(m[2]) } }];
  return [{ width: { ideal: 640 }, height: { ideal: 480 }, frameRate: { ideal: 30 } }];
}

export function backCameraNumber(label) {
  const m = /^camera2? (\d+), facing back$/.exec(String(label ?? '').trim());
  return m ? Number(m[1]) : null;
}

// devices: MediaDeviceInfo-like {kind, label, deviceId}. Returns {deviceId, label, number} or null
// (labels hidden, no labelled back camera, or the requested id missing / ambiguous).
export function pickBackCamera(devices, request) {
  const backs = [];
  for (const d of devices ?? []) {
    if (d.kind !== 'videoinput' || !d.deviceId) continue;
    const number = backCameraNumber(d.label);
    if (number !== null) backs.push({ deviceId: d.deviceId, label: d.label, number });
  }
  if (!request || request.mode === 'default' || !backs.length) return null;
  if (request.mode === 'number') {
    const m = backs.filter((b) => b.number === request.number);
    return m.length === 1 ? m[0] : null;
  }
  backs.sort((a, b) => a.number - b.number);
  return backs.length > 1 && backs[0].number === backs[1].number ? null : backs[0];
}

// Accuracy mode ACC-4 per camera: focal (px) of the 480x640 capture for a known device and back
// camera id; null when unknown (the engine then uses its device table, which is 540 for SM-F741
// and does not know which camera produced the frames). Values:
//   SM-F741 camera 0  540: the Chrome stream of the main camera is NOT ARCore's 4:3 image (ARCore
//                     calibration 425.46 at 480x640 does not apply). Run 2 (2026-10-19, camera 0
//                     verified by label) gives 554 from the gyro-vs-optical-flow fit and the same
//                     replay quality at 540 and 580 (height std 0.87, LIMITED 47) against 425
//                     (std 2.07, LIMITED 165); the earlier Chrome loop-closure sweep gave 535-550.
//                     Equal to the engine's device table (540). INFERRED (no independent GT).
//   SM-F741 camera 2  310: run 1 stream (camera 2), gyro-vs-optical-flow fit 303 (the fit reads
//                     about 5 % low on ARCore-calibrated data). INFERRED.
const CAMERA_FOCAL = { 'SM-F741': { 0: 540, 2: 310 } };
export function cameraFocalFor(model, number) {
  const base = /^SM-F741/i.test(String(model ?? '')) ? 'SM-F741' : String(model ?? '').toUpperCase();
  const t = CAMERA_FOCAL[base];
  return t && Number.isInteger(number) && t[number] !== undefined ? t[number] : null;
}

// Engine config for one run once the camera is known. ?fx= keeps precedence (diagnostic); the
// camera focal applies only with the accuracy mode on (the original K rule otherwise).
// The table values hold for a 4:3 stream of the whole sensor drawn 1:1 into 480x640: a cropped or
// resampled stream (capturePlan cropped / scaled) changes the focal per pixel, so the table is not
// used then (engine device table, focalSource says why).
export function configForCamera(config, search, camera, plan = null) {
  const explicitFx = Number(new URLSearchParams(search ?? '').get('fx'));
  if (Number.isFinite(explicitFx) && explicitFx > 0) return { ...config, focalSource: 'query ?fx' };
  const on = Boolean(config.accuracy && config.accuracy.enabled);
  const f = on && camera ? cameraFocalFor(config.model, camera.number) : null;
  if (f !== null && plan && (plan.cropped || plan.scaled)) {
    return { ...config, intrinsics: null, focalSource: `engine device table (camera ${camera.number} table not used: stream ${plan.cropped ? 'cropped' : 'resampled'}, focal UNVERIFIED)` };
  }
  if (f === null) return { ...config, intrinsics: null, focalSource: on ? 'engine device table' : 'original K rule' };
  return { ...config, intrinsics: [f, f, (CAPTURE_WIDTH - 1) / 2, (CAPTURE_HEIGHT - 1) / 2],
    focalSource: `camera ${camera.number} table (ACC-4)` };
}

// High-entropy device model (Chrome User-Agent Client Hints); '' when unavailable.
export async function clientHintModel(nav) {
  try {
    const d = nav && nav.userAgentData;
    if (!d || typeof d.getHighEntropyValues !== 'function') return '';
    const v = await d.getHighEntropyValues(['model']);
    return typeof v.model === 'string' ? v.model : '';
  } catch {
    return '';
  }
}

// Byte comparison of a GPU readback atlas with the CPU port atlas over the engine's rows, per
// level rectangle and channel (R gray, G Gaussian, B NMES score, A subpixel; S1 §2.2), plus the
// Jaccard index of the score > 0 pixels (keypoint candidates) inside the levels.
export function compareAtlases(gpu, cpu, layout) {
  const size = layout.textureSize, rows = layout.rows;
  const n = size * rows * 4;
  if (gpu.length < n || cpu.length < n) throw new Error('atlas shorter than the layout rows');
  let equalBytes = 0, maxAbs = 0;
  for (let i = 0; i < n; i++) {
    const d = Math.abs(gpu[i] - cpu[i]);
    if (d === 0) equalBytes++; else if (d > maxAbs) maxAbs = d;
  }
  let inter = 0, union = 0;
  const levels = layout.levels.map((l) => {
    const eq = [0, 0, 0, 0];
    let px = 0;
    for (let y = l.r; y < l.r + l.h; y++) {
      for (let x = l.c; x < l.c + l.w; x++) {
        const k = (y * size + x) * 4;
        for (let ch = 0; ch < 4; ch++) if (gpu[k + ch] === cpu[k + ch]) eq[ch]++;
        const a = gpu[k + 2] > 0, b = cpu[k + 2] > 0;
        if (a && b) inter++;
        if (a || b) union++;
        px++;
      }
    }
    return { r: eq[0] / px, g: eq[1] / px, b: eq[2] / px, a: eq[3] / px };
  });
  return { identical: equalBytes === n, equalByteRatio: equalBytes / n, maxAbsDiff: maxAbs, levels,
    scoreJaccard: union ? inter / union : 1 };
}

// Published origin before the first translation (the glue forces y = 1 for absolute scale; the
// metric output starts at origin * metricPriorScale = 0.7 m per responsive unit).
export const METRIC_PRIOR_SCALE = 0.7;
export function originOf(config) {
  if (config.estimateScale) return [config.origin[0], 1, config.origin[2]];
  const s = config.accuracy && config.accuracy.metricOutput ? METRIC_PRIOR_SCALE : 1;
  return [config.origin[0] * s, config.origin[1] * s, config.origin[2] * s];
}

export const HIT_TYPES = { FEATURE_POINT: 1, ESTIMATED_SURFACE: 2, DETECTED_SURFACE: 4 };

// How a video frame of vw x vh is turned into the 480x640 portrait capture. A portrait stream is
// center-cropped to 3:4; a landscape stream is center-cropped to 4:3 and rotated by 90 degrees
// (direction from ?rotate=cw|ccw, default cw; UNVERIFIED on the device, so the page shows it).
export function capturePlan(vw, vh, search) {
  if (!(vw > 0 && vh > 0)) throw new Error(`invalid video size ${vw}x${vh}`);
  const q = new URLSearchParams(search ?? '');
  const landscape = vw > vh;
  const rotate = landscape ? (q.get('rotate') === 'ccw' ? 'ccw' : 'cw') : 'none';
  // Source aspect wanted (width / height in source pixels).
  const want = landscape ? CAPTURE_HEIGHT / CAPTURE_WIDTH : CAPTURE_WIDTH / CAPTURE_HEIGHT;
  let sw = vw, sh = vh;
  if (vw / vh > want) sw = Math.round(vh * want); else sh = Math.round(vw / want);
  return { rotate, sx: Math.floor((vw - sw) / 2), sy: Math.floor((vh - sh) / 2), sw, sh,
    dw: CAPTURE_WIDTH, dh: CAPTURE_HEIGHT, cropped: sw !== vw || sh !== vh,
    scaled: (landscape ? sh : sw) !== CAPTURE_WIDTH };
}

// Nearest-rank percentiles of finite values; null when empty.
export function percentiles(values, ps = [50, 95, 99]) {
  const v = values.filter(Number.isFinite).sort((a, b) => a - b);
  const out = { n: v.length };
  for (const p of ps) out[`p${p}`] = v.length ? v[Math.max(0, Math.ceil(p / 100 * v.length) - 1)] : null;
  out.max = v.length ? v[v.length - 1] : null;
  return out;
}

// Index of the first fresh record whose position left the origin (first translation), or -1.
export function firstTranslationIndex(records, origin) {
  return records.findIndex((r) => r.fresh && r.position && r.position.some((x, k) => x !== origin[k]));
}

// Trajectory colours of the 3D view by published tracking status.
export const STATUS_COLORS = { NORMAL: 0x3ddc84, LIMITED: 0xffb300, NOT_AVAILABLE: 0xff5252,
  UNSPECIFIED: 0x9e9e9e };

// getWorldPoints pads the 631 list to its pre-filter size with [id 0, confidence 0] entries at the
// post-704 origin (quirk Q10, architecture.md §1.3); they are not map points.
export function isPaddingPoint(id, confidence) { return id === 0 && confidence === 0; }

// Rotates v by the unit quaternion q = [w, x, y, z] (published rotationWxyz).
export function quatRotate(q, v) {
  const [w, x, y, z] = q;
  const tx = 2 * (y * v[2] - z * v[1]), ty = 2 * (z * v[0] - x * v[2]), tz = 2 * (x * v[1] - y * v[0]);
  return [v[0] + w * tx + (y * tz - z * ty), v[1] + w * ty + (z * tx - x * tz), v[2] + w * tz + (x * ty - y * tx)];
}

// Path length, start->end displacement and height range of fresh positions (display units: the
// responsive scale is not metric). origin is the published start position.
export function trajectoryStats(positions, origin) {
  let length = 0;
  let minY = Infinity, maxY = -Infinity;
  for (let i = 0; i < positions.length; i++) {
    const p = positions[i];
    minY = Math.min(minY, p[1]); maxY = Math.max(maxY, p[1]);
    if (i) length += Math.hypot(p[0] - positions[i - 1][0], p[1] - positions[i - 1][1], p[2] - positions[i - 1][2]);
  }
  const last = positions[positions.length - 1];
  return { poses: positions.length, pathLength: length,
    endToStart: last ? Math.hypot(last[0] - origin[0], last[1] - origin[1], last[2] - origin[2]) : null,
    heightRange: positions.length ? [minY, maxY] : null };
}

const STAGES = ['captureToPostMs', 'queueMs', 'gpuUploadMs', 'gpuDrawMs', 'gpuReadMs', 'cpuPyramidMs', 'engineMs',
  'renderMs', 'poseAgeMs'];

// Timing summary split before/after the first translation (records = page frame records).
export function summarize(records, origin) {
  const init = firstTranslationIndex(records, origin);
  const split = { preInit: init < 0 ? records : records.slice(0, init),
    postInit: init < 0 ? [] : records.slice(init) };
  const out = { firstTranslation: init < 0 ? null : { recordIndex: init, frameId: records[init].id,
    sinceStartMs: records[init].readMs - records[0].readMs } };
  for (const [name, rs] of Object.entries(split)) {
    const s = { frames: rs.length, fresh: rs.filter((r) => r.fresh).length };
    for (const k of STAGES) s[k] = percentiles(rs.map((r) => r[k]));
    s.status = {};
    for (const r of rs) s.status[r.status] = (s.status[r.status] ?? 0) + 1;
    out[name] = s;
  }
  return out;
}

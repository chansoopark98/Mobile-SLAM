// Contracts of the slam_refact measurement page (web/slam-refact.html, web/js/slam-refact-*.js).
import test from 'node:test';
import assert from 'node:assert/strict';
import fs from 'node:fs/promises';
import { HIT_TYPES, METRIC_PRIOR_SCALE, accuracyConfigFor, backCameraNumber, cameraFocalFor, cameraRequestFor,
  capturePlan, clientHintModel, configForCamera, pickBackCamera, videoRequestsFor,
  deviceModelFromUserAgent, engineConfigFor, firstTranslationIndex, originOf, percentiles,
  summarize } from '../../web/js/slam-refact-core.js';

const read = (p) => fs.readFile(new URL(`../../${p}`, import.meta.url), 'utf8');
const html = await read('web/slam-refact.html');
const app = await read('web/js/slam-refact-app.js');
const worker = await read('web/js/slam-refact-worker.js');
const recorderSrc = await read('web/js/slam-refact-recorder.js');
const recWriter = await read('web/js/slam-refact-rec-writer.js');
const grayModule = await read('web/js/slam-refact-gray.js');

test('device model comes from the user agent (reduced Chrome UA of the Flip6 gives K)', () => {
  assert.equal(deviceModelFromUserAgent('Mozilla/5.0 (Linux; Android 10; K) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/141.0.0.0 Mobile Safari/537.36'), 'K');
  assert.equal(deviceModelFromUserAgent('Mozilla/5.0 (Linux; Android 14; SM-F741N Build/UP1A.231005.007) AppleWebKit/537.36'), 'SM-F741N');
  assert.equal(deviceModelFromUserAgent('Mozilla/5.0 (X11; Linux x86_64) AppleWebKit/537.36'), '');
  assert.equal(deviceModelFromUserAgent(undefined), '');
});

test('engine config is portrait 480x640, rotation 0, ieee pyramid, accuracy mode on by default', () => {
  const ua = 'Mozilla/5.0 (Linux; Android 10; K) AppleWebKit/537.36';
  const c = engineConfigFor('', ua);
  assert.deepEqual(c, { estimateScale: false, disableVio: false, captureWidth: 480, captureHeight: 640,
    rotationDeg: 0, manufacturer: '', model: 'K', pyramidModel: 'ieee', origin: [0, 2, 0],
    leftHandedAxes: false, mirroredDisplay: false,
    accuracy: { enabled: true, metricOutput: true, localBa: false, multiviewTriangulation: false },
    detectSurfaces: true, intrinsics: null, frontend: 'gpu', gpuCheck: null, record: false });
  assert.equal(engineConfigFor('?frontend=cpu', ua).frontend, 'cpu');
  assert.equal(engineConfigFor('?frontend=webgpu', ua).frontend, 'gpu');
  assert.deepEqual(engineConfigFor('?gpu-check=1', ua).gpuCheck, { every: 10, frames: 30 });
  // Metric output starts at origin * 0.7 m per responsive unit.
  assert.equal(METRIC_PRIOR_SCALE, 0.7);
  assert.deepEqual(originOf(c).map((v) => Number(v.toFixed(6))), [0, 1.4, 0]);
  // ?accuracy=0 is the original behaviour (responsive origin, no metric output).
  const f = engineConfigFor('?accuracy=0', ua);
  assert.deepEqual(f.accuracy, { enabled: false, metricOutput: false, localBa: false, multiviewTriangulation: false });
  assert.deepEqual(originOf(f), [0, 2, 0]);
  assert.equal(engineConfigFor('?surfaces=0', ua).detectSurfaces, false);
  assert.deepEqual(engineConfigFor('?fx=540', ua).intrinsics, [540, 540, 239.5, 319.5]);
  assert.equal(engineConfigFor('?fx=-3', ua).intrinsics, null);
  assert.equal(engineConfigFor('', ua, 'SM-F741N').model, 'SM-F741N');
  assert.equal(accuracyConfigFor('?scale=absolute').metricOutput, false);
  assert.deepEqual(HIT_TYPES, { FEATURE_POINT: 1, ESTIMATED_SURFACE: 2, DETECTED_SURFACE: 4 });
  const d = engineConfigFor('?pyramid=swiftshader-approx&scale=absolute', ua);
  assert.equal(d.pyramidModel, 'swiftshader-approx');
  assert.equal(d.estimateScale, true);
  assert.deepEqual(originOf(d), [0, 1, 0]);
  assert.equal(engineConfigFor('?pyramid=gpu', ua).pyramidModel, 'ieee');
});

test('client hints model: high-entropy model when available, empty otherwise', async () => {
  assert.equal(await clientHintModel({ userAgentData: { getHighEntropyValues: async () => ({ model: 'SM-F741N' }) } }), 'SM-F741N');
  assert.equal(await clientHintModel({ userAgentData: { getHighEntropyValues: async () => { throw new Error('denied'); } } }), '');
  assert.equal(await clientHintModel({}), '');
  assert.equal(await clientHintModel(undefined), '');
});

test('surfaces and hit test stay display-only and go through the worker', () => {
  assert.match(worker, /glue\.getSurfaces\(\)/);
  // Older deployed engines (no surface API) must not make the worker fail.
  assert.match(worker, /typeof glue\.getSurfaces === 'function'/);
  assert.match(worker, /typeof glue\.hitTest === 'function'/);
  assert.match(worker, /glue\.hitTest\(msg\.u, msg\.v, msg\.mask \| 0\)/);
  assert.match(app, /ui\.preview\.addEventListener\('click', onPreviewTap\)/);
  assert.match(html, /id="accuracy"/);
  assert.match(html, /id="hit"/);
});

test('capture plan always yields 480x640 portrait; landscape streams rotate (cw default)', () => {
  assert.deepEqual(capturePlan(480, 640, ''), { rotate: 'none', sx: 0, sy: 0, sw: 480, sh: 640, dw: 480, dh: 640, cropped: false, scaled: false });
  const l = capturePlan(640, 480, '');
  assert.equal(l.rotate, 'cw'); assert.equal(l.sw, 640); assert.equal(l.sh, 480); assert.equal(l.cropped, false);
  assert.equal(capturePlan(640, 480, '?rotate=ccw').rotate, 'ccw');
  const tall = capturePlan(720, 1280, '');
  assert.equal(tall.rotate, 'none'); assert.equal(tall.sw, 720); assert.equal(tall.sh, 960); assert.equal(tall.sy, 160);
  assert.equal(tall.cropped, true); assert.equal(tall.scaled, true);
  assert.throws(() => capturePlan(0, 640, ''));
});

test('percentiles and the pre/post first-translation split', () => {
  assert.deepEqual(percentiles([3, 1, 2, NaN, 4]), { n: 4, p50: 2, p95: 4, p99: 4, max: 4 });
  assert.deepEqual(percentiles([]), { n: 0, p50: null, p95: null, p99: null, max: null });
  const o = [0, 2, 0];
  const rec = (id, fresh, position) => ({ id, readMs: id * 100, fresh, position, status: 'NORMAL',
    captureToPostMs: 1, queueMs: 0, engineMs: id, renderMs: 0, poseAgeMs: 2 });
  const rs = [rec(0, false, o), rec(1, true, o), rec(2, false, [1, 2, 0]), rec(3, true, [0.5, 2, 0]), rec(4, true, [1, 2, 0])];
  assert.equal(firstTranslationIndex(rs, o), 3);
  const s = summarize(rs, o);
  assert.deepEqual(s.firstTranslation, { recordIndex: 3, frameId: 3, sinceStartMs: 300 });
  assert.equal(s.preInit.frames, 3); assert.equal(s.postInit.frames, 2);
  assert.equal(s.postInit.engineMs.p50, 3);
  assert.equal(summarize(rs.slice(0, 2), o).firstTranslation, null);
});

test('nothing is acquired before Start; camera and sensors only from the start handler', () => {
  // facingMode open (labels need the grant) + reopen by deviceId when another back camera is picked.
  assert.equal((app.match(/getUserMedia\(/g) ?? []).length, 1);   // openWithSizes (facingMode open + deviceId reopen)
  assert.match(app, /async function acquireCamera\(\) \{\s+const request = cameraRequestFor\(search\);\s+let opened = await openWithSizes\(\{ facingMode: \{ ideal: 'environment' \} \}\);/);
  assert.equal((app.match(/acquireCamera\(\)/g) ?? []).length, 2);  // definition + start()
  assert.equal((app.match(/enumerateDevices\(/g) ?? []).length, 1);
  assert.equal((app.match(/addSensorListeners\(\)/g) ?? []).length, 2);
  const startBody = app.slice(app.indexOf('async function start()'), app.indexOf('function stop()'));
  // The camera is opened before the engine so its id selects the focal of this run (ACC-4).
  assert.ok(startBody.indexOf('await acquireCamera()') < startBody.indexOf('configForCamera(baseConfig'));
  assert.ok(startBody.indexOf('configForCamera(baseConfig') < startBody.indexOf('await initEngine()'));
  assert.ok(startBody.indexOf('await initEngine()') < startBody.indexOf('setupCapture()'));
  assert.match(startBody, /addSensorListeners\(\)/);
  const bootBody = app.slice(app.indexOf('async function boot()'));
  assert.doesNotMatch(bootBody, /acquireCamera|enumerateDevices|addSensorListeners|start\(\)/);
  assert.doesNotMatch(html, /autoplay/);
  assert.match(html, /<button id="start" disabled>/);
});

test('back camera: main (lowest id) by default, ?camera=<n> exact, ?camera=default browser choice', () => {
  assert.deepEqual(cameraRequestFor(''), { mode: 'main', number: null });
  assert.deepEqual(cameraRequestFor('?camera=2'), { mode: 'number', number: 2 });
  assert.deepEqual(cameraRequestFor('?camera=default'), { mode: 'default', number: null });
  assert.deepEqual(cameraRequestFor('?camera=-1'), { mode: 'main', number: null });
  assert.equal(backCameraNumber('camera 2, facing back'), 2);
  assert.equal(backCameraNumber('camera2 0, facing back'), 0);
  assert.equal(backCameraNumber('camera 1, facing front'), null);
  assert.equal(backCameraNumber(''), null);
  const flip6 = [
    { kind: 'audioinput', label: 'mic', deviceId: 'a' },
    { kind: 'videoinput', label: 'camera 2, facing back', deviceId: 'd2' },
    { kind: 'videoinput', label: 'camera 1, facing front', deviceId: 'd1' },
    { kind: 'videoinput', label: 'camera 0, facing back', deviceId: 'd0' },
  ];
  assert.deepEqual(pickBackCamera(flip6, cameraRequestFor('')), { deviceId: 'd0', label: 'camera 0, facing back', number: 0 });
  assert.equal(pickBackCamera(flip6, cameraRequestFor('?camera=2')).deviceId, 'd2');
  assert.equal(pickBackCamera(flip6, cameraRequestFor('?camera=5')), null);
  assert.equal(pickBackCamera(flip6, cameraRequestFor('?camera=default')), null);
  // Labels hidden (no grant yet): nothing to pick.
  assert.equal(pickBackCamera(flip6.map((d) => ({ ...d, label: '' })), cameraRequestFor('')), null);
  assert.match(app, /back camera \$\{request\.number\} is missing or ambiguous/);
  assert.match(app, /openWithSizes\(\{ deviceId: \{ exact: pick\.deviceId \} \}\)/);
  // ?res=: default 640x480 ideal; 8w = the original reference runtime Android cascade; WxH exact.
  assert.deepEqual(videoRequestsFor(''), [{ width: { ideal: 640 }, height: { ideal: 480 }, frameRate: { ideal: 30 } }]);
  assert.deepEqual(videoRequestsFor('?res=8w')[0], { width: { exact: 960 }, height: { exact: 720 } });
  assert.equal(videoRequestsFor('?res=8w').length, 8);
  assert.deepEqual(videoRequestsFor('?res=1920x1440'), [{ width: { exact: 1920 }, height: { exact: 1440 } }]);
  assert.deepEqual(videoRequestsFor('?res=junk'), videoRequestsFor(''));
  assert.match(app, /sizeRequest: opened\.size/);
});

test('ACC-4 focal per camera: Flip6 main 540, camera 2 310, ?fx wins, accuracy off keeps the K rule', () => {
  assert.equal(cameraFocalFor('SM-F741N', 0), 540);
  assert.equal(cameraFocalFor('SM-F741N', 2), 310);
  assert.equal(cameraFocalFor('SM-F741N', 1), null);
  assert.equal(cameraFocalFor('K', 0), null);
  assert.equal(cameraFocalFor('SM-F741N', null), null);
  const ua = 'Mozilla/5.0 (Linux; Android 10; K) AppleWebKit/537.36';
  const base = engineConfigFor('', ua, 'SM-F741N');
  const main = configForCamera(base, '', { number: 0 });
  assert.deepEqual(main.intrinsics, [540, 540, 239.5, 319.5]);
  assert.match(main.focalSource, /camera 0/);
  assert.equal(base.intrinsics, null);  // the base config is not mutated
  assert.equal(configForCamera(base, '', null).intrinsics, null);
  assert.equal(configForCamera(engineConfigFor('', ua, 'K'), '', { number: 0 }).intrinsics, null);
  assert.deepEqual(configForCamera(engineConfigFor('?fx=500', ua, 'SM-F741N'), '?fx=500', { number: 0 }).intrinsics, [500, 500, 239.5, 319.5]);
  assert.equal(configForCamera(engineConfigFor('?accuracy=0', ua, 'SM-F741N'), '?accuracy=0', { number: 0 }).intrinsics, null);
  // A cropped / resampled stream does not get the table focal (it holds for the 4:3 full-sensor stream).
  const cropped = configForCamera(base, '', { number: 0 }, { cropped: true, scaled: false });
  assert.equal(cropped.intrinsics, null);
  assert.match(cropped.focalSource, /not used: stream cropped, focal UNVERIFIED/);
  assert.equal(configForCamera(base, '', { number: 0 }, { cropped: false, scaled: true }).intrinsics, null);
  assert.deepEqual(configForCamera(base, '', { number: 0 }, { cropped: false, scaled: false }).intrinsics, [540, 540, 239.5, 319.5]);
  assert.match(app, /configForCamera\(baseConfig, search, cameraSelection, plan\)/);
  // The metrics JSON records which camera ran and where its focal came from.
  assert.match(app, /selection: cameraSelection, focal: \{ intrinsics: config\.intrinsics, source: config\.focalSource/);
});

test('no network writes, no persistent storage, no MP4 recording; download is user-click only', () => {
  for (const [name, src] of [['app', app], ['worker', worker], ['html', html]]) {
    assert.doesNotMatch(src, /XMLHttpRequest|sendBeacon|WebSocket|EventSource|MediaRecorder|localStorage|sessionStorage|indexedDB|method:\s*['"](?:POST|PUT)/i, name);
  }
  const fetches = [...app.matchAll(/fetch\(([^)]*)\)/g)].map((m) => m[1]);
  assert.deepEqual(fetches, ["'slam-refact/manifest.json', { cache: 'no-store' }"]);   // loadManifest only
  assert.equal((app.match(/a\.click\(\)/g) ?? []).length, 1);
  assert.match(app, /ui\.download\.addEventListener\('click', download\)/);
  assert.doesNotMatch(html, /vio_engine/);
});

test('sensor and frame times are main-thread performance.now() ns, not event.timeStamp', () => {
  const code = app.replace(/^\s*\/\/.*$/gm, '');
  assert.doesNotMatch(code, /\.timeStamp\b/);
  assert.match(app, /queueSensor\(\['m', Math\.round\(nowMs \* 1e6\)/);
  assert.match(app, /queueSensor\(\['o', Math\.round\(nowMs \* 1e6\)/);
  assert.match(app, /timeNs: Math\.round\(readMs \* 1e6\)/);
  // The worker never invents times: it forwards the main-thread values to the engine.
  const engineCalls = [...worker.matchAll(/glue\.processFrame\w*\(([^,]+), ([^,]+),/g)].map((m) => [m[1], m[2]]);
  assert.ok(engineCalls.length >= 2);
  for (const c of engineCalls) assert.deepEqual(c, ['msg.timeNs', 'msg.videoTimeNs']);
  assert.doesNotMatch(worker, /pushDevice\w+\(performance/);
});

test('worker runs only engine bytes that match the deployed manifest', () => {
  assert.match(worker, /loaded\.matchesManifest = loaded\.jsSha256 === expected\.jsSha256 && loaded\.wasmSha256 === expected\.wasmSha256/);
  assert.match(worker, /if \(!loaded\.matchesManifest\) \{\s+throw/);
  assert.match(worker, /wasmBinary: new Uint8Array\(wasmBuf\)/);
});

// ---- 3D view and GPU frontend (2026-10-18) ----
import { compareAtlases, isPaddingPoint, quatRotate, trajectoryStats, STATUS_COLORS } from '../../web/js/slam-refact-core.js';
import { gridStep } from '../../web/js/slam-refact-view3d.js';
import { subpixelParams } from '../../web/js/slam-refact-gr8gl.js';
import * as shaders from '../../web/js/slam-refact-gr8-shaders.js';
import crypto from 'node:crypto';

const view3d = await read('web/js/slam-refact-view3d.js');
const gr8gl = await read('web/js/slam-refact-gr8gl.js');

test('quatRotate applies wxyz rotations (GL camera -Z forward stays -Z under identity)', () => {
  const close = (a, b) => a.forEach((v, i) => assert.ok(Math.abs(v - b[i]) < 1e-12, `${a} vs ${b}`));
  close(quatRotate([1, 0, 0, 0], [0, 0, -1]), [0, 0, -1]);
  const h = Math.SQRT1_2;
  close(quatRotate([h, 0, h, 0], [0, 0, -1]), [-1, 0, 0]);   // +90 deg about +Y: forward -> -X
  close(quatRotate([h, h, 0, 0], [0, 1, 0]), [0, 0, 1]);     // +90 deg about +X: up -> +Z
  close(quatRotate([0, 0, 0, 1], [1, 0, 0]), [-1, 0, 0]);    // 180 deg about +Z
});

test('trajectory stats and padding points', () => {
  const s = trajectoryStats([[0, 2, 0], [3, 2, 0], [3, 2, 4]], [0, 2, 0]);
  assert.equal(s.poses, 3); assert.equal(s.pathLength, 7); assert.equal(s.endToStart, 5);
  assert.deepEqual(s.heightRange, [2, 2]);
  assert.deepEqual(trajectoryStats([], [0, 2, 0]), { poses: 0, pathLength: 0, endToStart: null, heightRange: null });
  assert.equal(isPaddingPoint(0, 0), true);
  assert.equal(isPaddingPoint(0, 0.5), false);
  assert.equal(isPaddingPoint(7, 0), false);
  for (const k of ['NORMAL', 'LIMITED', 'NOT_AVAILABLE', 'UNSPECIFIED']) assert.equal(typeof STATUS_COLORS[k], 'number');
  assert.equal(gridStep(6), 1); assert.equal(gridStep(60), 10); assert.equal(gridStep(1.2), 0.2);
});

test('3D view is dependency-free and draws the published GL-camera convention', () => {
  assert.doesNotMatch(html, /importmap|cdn\.|https?:\/\//);
  assert.doesNotMatch(view3d, /from ['"]three|https?:\/\//);
  assert.match(view3d, /-Z forward, \+Y up/);
  assert.match(app, /view3d\.addPose\(rec\)/);
  assert.match(app, /view3d\.breakTrajectory\(\)/);
  assert.match(worker, /pose\.repeated \? null : glue\.getWorldPoints\(\)/);
});

test('atlas comparison reports identity, ratios and score Jaccard over the layout rows', () => {
  const layout = { textureSize: 4, rows: 2, levels: [{ r: 0, c: 0, w: 2, h: 2 }] };
  const a = new Uint8Array(4 * 2 * 4), b = new Uint8Array(4 * 2 * 4);
  a[2] = 9; b[2] = 9;                           // pixel (0,0) score > 0 in both
  let r = compareAtlases(a, b, layout);
  assert.equal(r.identical, true); assert.equal(r.scoreJaccard, 1); assert.equal(r.maxAbsDiff, 0);
  b[(1 * 4 + 1) * 4 + 2] = 5;                   // pixel (1,1) score only in b
  r = compareAtlases(a, b, layout);
  assert.equal(r.identical, false); assert.equal(r.maxAbsDiff, 5); assert.equal(r.scoreJaccard, 0.5);
  assert.equal(r.levels[0].b, 0.75);
  assert.throws(() => compareAtlases(a.subarray(0, 8), b, layout));
});

test('GPU frontend runs the original wasm shaders verbatim in the GlRealityFrame order', () => {
  assert.equal(shaders.GR8_SHADER_SOURCE_WASM_SHA256, '780731fe294a725acf1b7717ed5ffb07ad9f0929e408a1d74f47450c291ec422');
  for (const [name, m] of Object.entries(shaders.GR8_SHADER_META)) {
    const src = shaders[name];
    assert.match(src, /^#version 300 es/, name);
    assert.equal(crypto.createHash('sha256').update(src).digest('hex'), m.sha256, name);
  }
  // createSubpixelParams (gr8-feature-shader.cc:111-140) as float32.
  const q = subpixelParams();
  assert.equal(q.length, 6);
  assert.ok(q.every(Number.isFinite));
  // drawTexture order: input, stage 1 (+ROI clears), 3 x (blur, decimate), stage 2, stage 3.
  const order = ['drawInputStage', 'drawInitialStage', 'drawPyramidBlurStage', 'drawPyramidDecimateStage', "drawStage(dest1 -> dest2, stage 2)"];
  let at = gr8gl.indexOf('draw() {');
  for (const o of order) { const i = gr8gl.indexOf(o, at); assert.ok(i > at, o); at = i; }
  assert.match(worker, /glue\.processFramePyramid\(msg\.timeNs, msg\.videoTimeNs, atlas, layout\.textureSize, layout\.rows\)/);
  assert.match(worker, /layout = glue\.pyramidLayout\(\)/);
  assert.match(app, /new Worker\('js\/slam-refact-worker\.js', \{ type: 'module' \}\)/);
  // A WebGL2 failure is reported, never silent.
  assert.match(app, /GPU frontend unavailable/);
  // The CPU frontend takes the colour frame through the CPU port (no JS gray approximation).
  assert.match(worker, /cpuPyramidAtlas\(rgba, msg\.width, msg\.height, cpuModel\)/);
  assert.doesNotMatch(worker, /0\.299 \*/);
});

test('a fatal drops the worker and the next Start recreates it; fatal injection is first-worker only', () => {
  const startBody = app.slice(app.indexOf('async function start()'), app.indexOf('function stop()'));
  assert.match(startBody, /state === 'error'\) \|\| !\(workerReady \|\| !worker\)\) return;/);
  assert.match(startBody, /if \(!\(await initEngine\(\)\)\) throw/);
  assert.match(app, /async function initEngine\(\) \{\s+if \(!manifest\) await loadManifest\(\);\s+if \(!worker\) startWorker\(\);/);
  assert.match(app, /} else if \(m\.type === 'fatal'\) \{[\s\S]{0,200}dropWorker\(\);/);
  assert.match(app, /const debugFatalAfterFrames = workerGeneration === 1 \? debugFatalAfter : 0;/);
  // The view and periodic rendering are installed before the engine load (a failed load is retryable).
  const bootBody = app.slice(app.indexOf('async function boot()'));
  assert.ok(bootBody.indexOf('loadView3d()') < bootBody.indexOf('await initEngine()'));
  // Every frame path closes its ImageBitmap.
  assert.match(worker, /finally \{\s+if \(msg\.bitmap\) msg\.bitmap\.close\(\);/);
  // Fit uses the narrower field of view (portrait phone canvas).
  assert.match(view3d, /Math\.min\(this\.fovY \/ 2, Math\.atan\(Math\.tan\(this\.fovY \/ 2\) \* aspect\)\)/);
});

// ---- opt-in input recorder (?record=1, 2026-10-19) ----
import { InputRecorder, REC_PREFIX, grayFromRgba, recordingStamp } from '../../web/js/slam-refact-recorder.js';

test('recording is off by default; ?record=1 is the only switch and it is a strict opt-in', () => {
  const ua = 'Mozilla/5.0 (Linux; Android 10; K) AppleWebKit/537.36';
  assert.equal(engineConfigFor('', ua).record, false);
  assert.equal(engineConfigFor('?record=0', ua).record, false);
  assert.equal(engineConfigFor('?record=true', ua).record, false);
  assert.equal(engineConfigFor('?accuracy=0&surfaces=0', ua).record, false);
  assert.equal(engineConfigFor('?record=1', ua).record, true);
  assert.equal(engineConfigFor('?frontend=cpu&record=1', ua).record, true);
  // REC badge, banner and the recording download are hidden unless the flag is on.
  assert.match(html, /id="recBadge" hidden>● REC</);
  assert.match(html, /id="recBanner" hidden>/);
  assert.match(html, /id="downloadRec" disabled hidden>녹화 다운로드</);
  assert.match(app, /if \(config\.record\) \{[\s\S]{0,200}ui\.recBadge\.hidden = false; ui\.recBanner\.hidden = false; ui\.downloadRec\.hidden = false;/);
  // A recorder exists only inside the opt-in branch of start(); the frame path checks it.
  assert.equal((app.match(/new InputRecorder\(/g) ?? []).length, 1);
  assert.match(app, /if \(config\.record\) \{[\s\S]{0,300}recorder = new InputRecorder\(/);
  assert.match(app, /const rec = recorder && recorder\.active \? recorder : null;/);
  assert.match(html, /\? ?record=1|\?record=1/);
});

test('recorder stores only the posted frame, with the exact posted sensor rows, via OPFS and a Worker', () => {
  const frameBody = app.slice(app.indexOf('function onVideoFrame('), app.indexOf('// ---- results'));
  // Busy-dropped frames return before anything is recorded.
  assert.ok(frameBody.indexOf('counters.droppedBusy++; return;') < frameBody.indexOf('rec.add('));
  // The frame source is taken before the bitmap transfer clears the canvas / before the rgba buffer
  // is detached; the main thread takes only a snapshot (GPU) or a copy (CPU), no readback, no gray.
  assert.ok(frameBody.indexOf('rec.take({ canvas })') < frameBody.indexOf('canvas.transferToImageBitmap()'));
  assert.ok(frameBody.indexOf('rec.take({ rgba: data })') < frameBody.indexOf('rgba = data.buffer'));
  assert.doesNotMatch(frameBody, /grayFrom|getImageData\(0, 0, CAPTURE_WIDTH, CAPTURE_HEIGHT\)\.data;\s+if \(rec\) recGray/);
  assert.match(recorderSrc, /return \{ bitmap: createImageBitmap\(canvas\), ref \};/);
  assert.match(recorderSrc, /if \(rgba\) return \{ rgba: rgba\.slice\(\), ref \};/);
  // The row carries the same id / times / sensors object that is posted to the engine worker.
  assert.match(frameBody, /rec\.add\(recSrc, \{ id, timeNs: Math\.round\(readMs \* 1e6\), videoTimeNs: Math\.round\(videoTimeS \* 1e9\), readMs, sensors \}, recStartMs\)/);
  assert.match(frameBody, /worker\.postMessage\(\{ type: 'frame', id, timeNs: Math\.round\(readMs \* 1e6\),\s+videoTimeNs: Math\.round\(videoTimeS \* 1e9\), width: CAPTURE_WIDTH, height: CAPTURE_HEIGHT, rgba, bitmap, sensors \}/);
  assert.ok(frameBody.indexOf("type: 'frame'") < frameBody.indexOf('rec.add('));
  // OPFS through a dedicated module Worker with a sync access handle; readback + gray happen there;
  // snapshots / copies are transferred and every bitmap is closed.
  assert.match(recorderSrc, /new Worker\('js\/slam-refact-rec-writer\.js', \{ type: 'module' \}\)/);
  assert.match(recorderSrc, /post\(\{ bitmap, \.\.\.ref \}, \[bitmap, \.\.\.refT\]\)/);
  assert.match(recorderSrc, /post\(\{ rgba: src\.rgba\.buffer, \.\.\.ref \}, \[src\.rgba\.buffer, \.\.\.refT\]\)/);
  assert.match(recWriter, /navigator\.storage\.getDirectory\(\)/);
  assert.match(recWriter, /createSyncAccessHandle\(\)/);
  assert.match(recWriter, /try \{ ctx\.drawImage\(m\.bitmap, 0, 0\); \} finally \{ m\.bitmap\.close\(\); \}/);
  assert.match(recWriter, /writeAll\(handle, gray, m\.i \* frameBytes\)/);
  // finish() drains the pending snapshots before closing the writer.
  const finishBody = recorderSrc.slice(recorderSrc.indexOf('async finish('));
  assert.ok(finishBody.indexOf('await Promise.allSettled([...this.pending])') < finishBody.indexOf("postMessage({ t: 'close' })"));
  assert.match(grayModule, /\(0\.299 \* rgba\[j\] \+ 0\.587 \* rgba\[j \+ 1\] \+ 0\.114 \* rgba\[j \+ 2\] \+ 0\.5\) \| 0/);
  assert.match(recorderSrc, /APPROXIMATION/);
  assert.match(app, /window\.__slamRefactRecording = recordingApi\(\)/);
});

test('recorder never touches the network and only the user-click handler downloads', () => {
  for (const [name, src] of [['recorder', recorderSrc], ['writer', recWriter], ['gray', grayModule]]) {
    const code = src.replace(/^\s*\/\/.*$/gm, '');
    assert.doesNotMatch(code, /\bfetch\(|XMLHttpRequest|sendBeacon|WebSocket|EventSource|MediaRecorder|localStorage|sessionStorage|indexedDB|https?:\/\/|\.click\(\)|\bdownload\b/, name);
  }
  // Still exactly one fetch (manifest) and one programmatic anchor click in the page.
  assert.deepEqual([...app.matchAll(/fetch\(([^)]*)\)/g)].map((m) => m[1]), ["'slam-refact/manifest.json', { cache: 'no-store' }"]);
  assert.equal((app.match(/a\.click\(\)/g) ?? []).length, 1);
  // saveUrl is reached only from the two click handlers (download, downloadRecording).
  assert.equal((app.match(/saveUrl\(/g) ?? []).length, 3);   // definition + 2 call sites
  const clicks = [...app.matchAll(/ui\.(\w+)\.addEventListener\('click', (\w+)\)/g)].map((m) => `${m[1]}:${m[2]}`);
  assert.ok(clicks.includes('download:download') && clicks.includes('downloadRec:downloadRecording'), clicks.join());
  const saveCallers = ['function download()', 'async function downloadRecording()'];
  for (const c of saveCallers) assert.ok(app.includes(c), c);
  assert.ok(app.indexOf('function saveUrl(') < app.indexOf('function download()'));
  assert.match(app, /ui\.downloadRec\.disabled = !\(recSummary && recSummary\.ok\)/);
  assert.match(html, /<button id="downloadRec" disabled hidden>/);
});

test('recorder helpers: gray rounding, stamp, frame cap, backlog stop and reset bookkeeping', async () => {
  const rgba = new Uint8ClampedArray([255, 255, 255, 255, 0, 0, 0, 255, 255, 0, 0, 255, 10, 200, 30, 255]);
  assert.deepEqual([...grayFromRgba(rgba)], [255, 0, 76, Math.round(0.299 * 10 + 0.587 * 200 + 0.114 * 30)]);
  // (x + 0.5) | 0 equals Math.round on every RGB triple (the recording format is unchanged).
  const all = new Uint8Array(4 << 16);
  for (const b of [0, 37, 128, 255]) {
    for (let k = 0; k < 1 << 16; k++) { all[4 * k] = k >> 8; all[4 * k + 1] = k & 255; all[4 * k + 2] = b; }
    const g = grayFromRgba(all);
    for (let k = 0; k < 1 << 16; k++) assert.equal(g[k], Math.round(0.299 * (k >> 8) + 0.587 * (k & 255) + 0.114 * b));
  }
  assert.match(recordingStamp(new Date('2026-10-19T01:02:03.456Z')), /^2026-10-19T01-02-03-456Z$/);
  assert.equal(REC_PREFIX, 'slam-refact-rec-');
  const posted = [];
  const r = new InputRecorder({ width: 4, height: 2, maxFrames: 2 });
  assert.equal(r.active, false);
  assert.equal(r.take({ rgba: new Uint8Array(32) }), null);       // not begun: nothing is taken
  assert.equal(r.add(null, { id: 0 }, 0), false);
  r.active = true; r.writer = { postMessage(m) { posted.push(m); } };
  r.noteReset(1);
  const src = new Uint8Array(32).fill(7);
  const t0 = r.take({ rgba: src });
  assert.notEqual(t0.rgba.buffer, src.buffer);                     // a copy: the engine buffer stays intact
  assert.equal(r.add(t0, { id: 0, sensors: [] }, performance.now()), true);
  r.noteReset(2);
  assert.equal(r.add(r.take({ rgba: src }), { id: 1, sensors: [] }, performance.now()), true);
  assert.equal(r.take({ rgba: src }), null);
  assert.equal(r.limitReached, true); assert.equal(r.count, 2);
  assert.equal(r.stopReason.reason, 'limit');
  assert.deepEqual(r.resets.map((x) => x.beforeFrameIndex), [0, 1]);
  assert.deepEqual(r.rows.map((x) => x.index), [0, 1]);
  assert.deepEqual(posted.map((m) => [m.t, m.i, m.rgba.byteLength]), [['f', 0, 32], ['f', 1, 32]]);
  // A writer that falls maxBacklog frames behind stops the recording, never the caller.
  const b = new InputRecorder({ width: 4, height: 2, maxBacklog: 2 });
  b.active = true; b.writer = { postMessage() {} };
  assert.ok(b.add(b.take({ rgba: src }), { id: 0 }, 0));
  assert.ok(b.add(b.take({ rgba: src }), { id: 1 }, 0));
  assert.equal(b.take({ rgba: src }), null);
  assert.equal(b.active, false);
  assert.deepEqual([b.stopReason.reason, b.stopReason.atFrame], ['backlog', 2]);
  b.onWriter({ t: 'wrote', count: 2 });
  assert.equal(b.take({ rgba: src }), null);                       // stopped stays stopped
  // ?rec-verify: every N-th frame carries the reference gray of the old synchronous path.
  const v = new InputRecorder({ width: 4, height: 2, verifyEvery: 2 });
  v.active = true; const vp = []; v.writer = { postMessage(m) { vp.push(m); } };
  for (let i = 0; i < 3; i++) v.add(v.take({ rgba: src }), { id: i }, 0);
  assert.deepEqual(vp.map((m) => !!m.ref), [true, false, true]);
  assert.deepEqual([...new Uint8Array(vp[0].ref)], [...grayFromRgba(src)]);
  // Early stops are reported once through onStopped (shown on the page at once).
  const seen = [];
  const o = new InputRecorder({ width: 4, height: 2, maxBacklog: 1 });
  o.active = true; o.writer = { postMessage() {} }; o.onStopped = (r) => seen.push(r.reason);
  o.add(o.take({ rgba: src }), { id: 0 }, 0);
  o.take({ rgba: src }); o.take({ rgba: src });
  assert.deepEqual(seen, ['backlog']);
  assert.match(app, /recorder\.onStopped = \(r\) => \{/);
  // discard() releases a snapshot that was never added.
  let closed = 0;
  o.discard({ bitmap: Promise.resolve({ close() { closed++; } }) });
  await new Promise((r) => setTimeout(r, 0));
  assert.equal(closed, 1);
  // A writer error fails every waiter and stops the recording.
  const e = new InputRecorder({ width: 4, height: 2 });
  e.active = true;
  const waiting = e.wait((m) => m.t === 'closed');
  e.onWriter({ t: 'error', message: 'disk full' });
  await assert.rejects(waiting, /disk full/);
  assert.equal(e.active, false);
  assert.equal(e.stopReason.reason, 'error');
  await assert.rejects(e.wait(() => true), /disk full/);
});

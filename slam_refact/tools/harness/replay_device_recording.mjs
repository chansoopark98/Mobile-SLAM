#!/usr/bin/env node
// Replays a real-device input recording (slam_refact measurement page, ?record=1) through a WASM
// engine on the PC with exactly the recorded inputs.
//
//   node slam_refact/tools/harness/replay_device_recording.mjs --dir <recording dir> --out <new dir under build/>
//        [--meta <x.meta.json> --frames <x.frames.bin>] [--engine-dir web/slam-refact]
//        [--pyramid-model ieee|swiftshader-approx] [--max-frames N] [--sensors-only-check]
//
// Inputs: <prefix>.meta.json + <prefix>.frames.bin written by web/js/slam-refact-recorder.js
// (frames.bin = raw gray8 frames, width*height bytes each; meta.frames[k].sensors = the exact sensor
// rows posted with frame k). For every frame the recorded sensor rows are pushed in order with their
// recorded tNs (pushDeviceMotion / pushDeviceOrientation), recorded resets are applied before their
// frame, then processFrame(timeNs, videoTimeNs, gray, width, height) runs (CPU pyramid model "ieee"
// unless --pyramid-model). Output (never overwrites; --out must not exist and must be under build/):
// trace.jsonl in the reference runtime reference schema of slam_refact/tools/harness/eval/* plus engine-only
// "sr*" fields (srMetric*, srFloor), commands.txt and summary.json.
//
// LIMIT: the recorded gray is round(0.299R+0.587G+0.114B) of the engine-input pixels. The device ran
// the original GPU pyramid on the colour frame, so a CPU-pyramid replay is an approximation of the
// device run (sensors, timing, frame order and config are exact).
import crypto from 'node:crypto';
import fs from 'node:fs';
import path from 'node:path';
import { createRequire } from 'node:module';
import { parseArgs } from 'node:util';

const repo = path.resolve(import.meta.dirname, '../../..');
const { values: a } = parseArgs({ options: {
  dir: { type: 'string' }, meta: { type: 'string' }, frames: { type: 'string' }, out: { type: 'string' },
  'engine-dir': { type: 'string' }, 'pyramid-model': { type: 'string' }, 'max-frames': { type: 'string' },
  'sensors-only-check': { type: 'boolean', default: false },
  // Diagnostic overrides of the recorded configuration (recorded in commands.txt / summary.json).
  fx: { type: 'string' }, 'accuracy-off': { type: 'boolean', default: false },
  vi: { type: 'boolean', default: false },
  'metric-low': { type: 'boolean', default: false } } });

function die(msg) { console.error(`replay_device_recording: ${msg}`); process.exit(2); }

let metaPath = a.meta, framesPath = a.frames;
if (a.dir) {
  const dir = path.resolve(a.dir);
  const metas = fs.readdirSync(dir).filter((f) => f.endsWith('.meta.json'));
  if (metas.length !== 1 && !a.meta) die(`${dir} must hold exactly one *.meta.json (found ${metas.length}); use --meta`);
  metaPath = metaPath ?? path.join(dir, metas[0]);
}
if (!metaPath) die('--dir or --meta required');
metaPath = path.resolve(metaPath);
framesPath = path.resolve(framesPath ?? metaPath.replace(/\.meta\.json$/, '.frames.bin'));

const metaText = fs.readFileSync(metaPath);
const meta = JSON.parse(metaText.toString('utf8'));
const metaSha256 = crypto.createHash('sha256').update(metaText).digest('hex');
if (meta.schema !== 'slam-refact-input-recording-v1') die(`unexpected schema ${meta.schema}`);
const W = meta.width, H = meta.height, frameBytes = W * H;
const rows = meta.frames;
const framesSize = fs.statSync(framesPath).size;

// ---- validation (also the whole of --sensors-only-check) ----
function validate() {
  const problems = [];
  if (frameBytes !== meta.frameBytes) problems.push(`frameBytes ${meta.frameBytes} != ${W}*${H}`);
  if (framesSize !== rows.length * frameBytes) problems.push(`frames.bin ${framesSize} bytes != ${rows.length} frames * ${frameBytes}`);
  let motion = 0, orientation = 0, lastT = -Infinity, lastTime = -Infinity, sensorsAfterFrame = 0, nonMono = 0;
  rows.forEach((r, k) => {
    if (r.index !== k) problems.push(`frame row ${k}: index ${r.index}`);
    if (!(Number.isFinite(r.timeNs) && Number.isFinite(r.videoTimeNs))) problems.push(`frame row ${k}: non-finite time`);
    if (r.timeNs <= lastTime) problems.push(`frame row ${k}: timeNs not increasing`);
    lastTime = r.timeNs;
    for (const s of r.sensors) {
      const len = s[0] === 'm' ? 13 : s[0] === 'o' ? 5 : -1;
      if (s.length !== len || !s.slice(1).every((v) => typeof v === 'boolean' || Number.isFinite(v))) { problems.push(`frame row ${k}: bad sensor row ${JSON.stringify(s)}`); continue; }
      if (s[0] === 'm') motion++; else orientation++;
      if (s[1] < lastT) nonMono++;
      lastT = Math.max(lastT, s[1]);
      if (s[1] > r.timeNs) sensorsAfterFrame++;
    }
  });
  if (nonMono) problems.push(`${nonMono} sensor rows go back in time`);
  return { problems, frames: rows.length, motion, orientation, unpostedSensors: (meta.unpostedSensors ?? []).length,
    sensorsAfterFrame, resets: (meta.resets ?? []).length, framesBytes: framesSize, metaSha256,
    firstTimeNs: rows[0]?.timeNs ?? null, lastTimeNs: rows.at(-1)?.timeNs ?? null,
    durationS: rows.length ? (rows.at(-1).timeNs - rows[0].timeNs) / 1e9 : 0 };
}
const check = validate();
if (a['sensors-only-check']) {
  console.log(JSON.stringify({ mode: 'sensors-only-check', meta: metaPath, frames: framesPath, ok: check.problems.length === 0, ...check,
    problems: check.problems.slice(0, 20) }, null, 1));
  process.exit(check.problems.length === 0 ? 0 : 1);
}
if (check.problems.length) die(`invalid recording: ${check.problems.slice(0, 5).join('; ')}`);
if (!rows.length) die('recording has no frames');

if (!a.out) die('--out <new dir under build/> required');
const outDir = path.resolve(a.out);
const buildRoot = path.join(repo, 'build') + path.sep;
if (!(outDir + path.sep).startsWith(buildRoot)) die(`--out must be under ${buildRoot}`);
if (fs.existsSync(outDir)) die(`refusing to overwrite ${outDir}`);
const pyramidModel = a['pyramid-model'] ?? 'ieee';
if (!['ieee', 'swiftshader-approx'].includes(pyramidModel)) die('--pyramid-model ieee|swiftshader-approx');
const maxFrames = a['max-frames'] ? Number(a['max-frames']) : rows.length;
if (!(Number.isInteger(maxFrames) && maxFrames > 0)) die('--max-frames <positive int>');
const engineDir = path.resolve(a['engine-dir'] ?? path.join(repo, 'web/slam-refact'));

const require = createRequire(import.meta.url);
let engineLogLines = 0;   // Ceres/miniglog lines of the engine; kept off stdout
const Module = await require(path.join(engineDir, 'slam_refact_engine.js'))({
  print: () => { engineLogLines++; }, printErr: () => { engineLogLines++; } });
const engineSha = (f) => crypto.createHash('sha256').update(fs.readFileSync(path.join(engineDir, f))).digest('hex');

// Recorded config -> engine config (the page's worker-side fields frontend/gpuCheck/record are dropped).
const c = meta.config;
const glue = new Module.SlamRefactGlue({
  estimateScale: c.estimateScale, disableVio: c.disableVio, captureWidth: W, captureHeight: H,
  rotationDeg: c.rotationDeg ?? 0, manufacturer: c.manufacturer ?? '', model: c.model ?? '', pyramidModel,
  origin: c.origin, leftHandedAxes: c.leftHandedAxes, mirroredDisplay: c.mirroredDisplay,
  accuracy: a['accuracy-off'] ? { enabled: false } : { ...c.accuracy, ...(a.vi ? { viPrediction: true } : {}),
    ...(a['metric-low'] ? { metricLowBand: true, metricWindowS: 8, metricFixedLagS: -0.09 } : {}) },
  detectSurfaces: c.detectSurfaces,
  intrinsics: a.fx ? [Number(a.fx), Number(a.fx), (W - 1) / 2, (H - 1) / 2] : (c.intrinsics ?? null) });
const hasSurfaces = !!c.detectSurfaces && typeof glue.getSurfaces === 'function';

fs.mkdirSync(outDir, { recursive: true });
const traceFd = fs.openSync(path.join(outDir, 'trace.jsonl'), 'wx');
const framesFd = fs.openSync(framesPath, 'r');
const gray = new Uint8Array(frameBytes);
const resets = [...(meta.resets ?? [])].sort((x, y) => x.beforeFrameIndex - y.beforeFrameIndex);
let resetAt = 0, lastIds = new Set();
const t0 = rows[0].timeNs;
const stats = { processed: 0, fresh: 0, finite: 0, nonFinite: 0, statuses: {}, resetsApplied: 0, maxFrameMs: 0 };
const num = (v) => (Number.isFinite(v) ? v : null);

const started = performance.now();
const n = Math.min(maxFrames, rows.length);
for (let k = 0; k < n; k++) {
  const r = rows[k];
  while (resetAt < resets.length && resets[resetAt].beforeFrameIndex <= k) { glue.reset(); lastIds = new Set(); stats.resetsApplied++; resetAt++; }
  for (const s of r.sensors) {
    if (s[0] === 'm') glue.pushDeviceMotion(s[1], s[2], s[3], s[4], s[5], s[6], s[7], s[8], s[9], s[10], s[11], s[12]);
    else glue.pushDeviceOrientation(s[1], s[2], s[3], s[4]);
  }
  if (fs.readSync(framesFd, gray, 0, frameBytes, k * frameBytes) !== frameBytes) die(`short read of frame ${k}`);
  const f0 = performance.now();
  const pose = glue.processFrame(r.timeNs, r.videoTimeNs, gray, W, H);
  stats.maxFrameMs = Math.max(stats.maxFrameMs, performance.now() - f0);
  stats.processed++;
  const row = { i: k, t: (r.timeNs - t0) / 1e9, videoTimeUs: Math.round(r.videoTimeNs / 1000), publishMs: null, acceptedMs: null,
    fresh: !pose.repeated };
  if (pose.repeated) {
    Object.assign(row, { callbackMs: null, latencyMs: null, status: null, reason: null, p: null, q_wxyz: null,
      worldPoints: null, wpAdded: null, wpRemoved: null, wpMeanConfidence: null, xrGpuStageMs: null, xrCpuStageMs: null });
  } else {
    stats.fresh++;
    const wp = glue.getWorldPoints();   // [id, confidence, x, y, z] * n
    const ids = new Set();
    let conf = 0, added = 0;
    for (let q = 0; q < wp.length; q += 5) { ids.add(wp[q]); conf += wp[q + 1]; if (!lastIds.has(wp[q])) added++; }
    let removed = 0;
    for (const id of lastIds) if (!ids.has(id)) removed++;
    lastIds = ids;
    const finite = [...pose.position, ...pose.rotationWxyz].every(Number.isFinite);
    if (finite) stats.finite++; else stats.nonFinite++;
    stats.statuses[pose.status] = (stats.statuses[pose.status] ?? 0) + 1;
    Object.assign(row, { callbackMs: null, latencyMs: null, status: pose.status, reason: pose.reason,
      p: finite ? [...pose.position] : null, q_wxyz: finite ? [...pose.rotationWxyz] : null,
      worldPoints: wp.length / 5, wpAdded: added, wpRemoved: removed,
      wpMeanConfidence: wp.length ? conf / (wp.length / 5) : null, xrGpuStageMs: null, xrCpuStageMs: null });
    if (pose.metric) Object.assign(row, { srMetricScale: num(pose.metric.scale), srMetricSigmaLog: num(pose.metric.sigmaLog),
      srMetricInitialized: pose.metric.initialized, srMetricLagS: num(pose.metric.lagS) });
    if (hasSurfaces) {
      const sf = glue.getSurfaces();
      row.srFloor = { detected: sf.floorDetected, height: sf.floorDetected ? sf.floor.height : null,
        inliers: sf.floorDetected ? sf.floor.inliers : 0, area: sf.floorDetected ? sf.floor.area : null,
        others: sf.others.length, cloud: sf.cloudPoints };
    }
  }
  fs.writeSync(traceFd, JSON.stringify(row) + '\n');
}
const elapsedS = (performance.now() - started) / 1000;
fs.closeSync(traceFd); fs.closeSync(framesFd);
glue.delete();

const summary = { schema: 'slam-refact-device-replay-v1', recording: { meta: metaPath, frames: framesPath, metaSha256,
  frames: rows.length, width: W, height: H, pageUserAgent: meta.page?.userAgent ?? null, capturePlan: meta.capturePlan,
  engineManifestSha256: meta.engine?.manifestSha256 ?? null, recordedConfig: c },
  replay: { engineDir, engineSha256: { js: engineSha('slam_refact_engine.js'), wasm: engineSha('slam_refact_engine.wasm') },
    pyramidModel, engineLogLines, framesReplayed: n, framesNotReplayed: rows.length - n, elapsedS, ...stats },
  note: 'CPU-pyramid replay of recorded gray frames (approximation of the device GPU pyramid); sensors/timing/order/config exact',
  check: { ...check, problems: [] } };
fs.writeFileSync(path.join(outDir, 'summary.json'), JSON.stringify(summary, null, 1) + '\n', { flag: 'wx' });
fs.writeFileSync(path.join(outDir, 'commands.txt'), `${['node', path.relative(repo, import.meta.filename), ...process.argv.slice(2)].join(' ')}\n` +
  `# engine ${path.relative(repo, engineDir)} js ${summary.replay.engineSha256.js} wasm ${summary.replay.engineSha256.wasm}\n` +
  `# recording meta sha256 ${metaSha256}\n`, { flag: 'wx' });
console.log(JSON.stringify({ out: outDir, frames: n, fresh: stats.fresh, finite: stats.finite, nonFinite: stats.nonFinite,
  statuses: stats.statuses, resetsApplied: stats.resetsApplied, elapsedS, maxFrameMs: stats.maxFrameMs }, null, 1));
process.exit(stats.nonFinite === 0 ? 0 : 1);

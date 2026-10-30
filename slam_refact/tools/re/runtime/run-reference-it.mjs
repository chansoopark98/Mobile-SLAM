// COPY of run-reference.mjs for the OR-4 gate (full-parity plan Rev B OR-4 (b) asks for a copy: run-reference.mjs belongs to other lanes);
// the only differences are the --image-targets hunks below (+ this header and the run-reference-it.mjs source hash).
// Lane C (US-004): run the ORIGINAL reference runtime SDK headless on a recorded sequence and write a per-frame trace.
// Reuses the side-by-side approach (ephemeral 127.0.0.1 server, external HTTP blocked, synthetic camera track +
// untrusted DeviceMotion/DeviceOrientation, public XrController output) without modifying existing scripts.
//
//   node slam_refact/tools/re/runtime/run-reference.mjs --scenario tum-room1-full --label r1 [--profile] [--wrap | --memprobe]
//
// --memprobe: memprobe.js reads xr-slam linear memory (read-only) after every c8EmAsm_processStagedFrame and writes
// memprobe.jsonl.gz (line 1 = header with field list, then one object per processStagedFrame call; `i` = input frame
// index of the fresh callback that reported that call, null for bootstrap/repeat).  See memprobe.js / MEMPROBE.md.
// --memprobe-paths (with --memprobe): append the unreachable-path fields (memprobe.js PATH_FIELDS) to every record.
//
// --record-inputs: input-recorder.js records, per call, the c8EmAsm_stageFrame arguments (alpha/beta/gamma, quaternion,
// videoTime, frameTime, timeNanos, ...), the capnp IMU event queue bytes handed to c8EmAsm_setEventQueue, and the
// processStagedFrame order (same export-wrapping mechanism as spec_frontend/capture_window.mjs; no code or SDK byte
// changes).  Writes recorded-inputs.jsonl.gz (Addendum 6 item 4; consumed by harness/divergence/make_variant.py).
//
// --api-oracle '{"ab":{...}}' (FULL-7, RM-3b): additionally records the A/B hit sequence inside one onUpdate (api-oracle.js header).
// --api-oracle '<json>': api-oracle.js (full-parity PRD US-002/003/005) merges `configure` options, re-sends
// updateCameraProjectionMatrix with `cam`/`origin`/`facing`, and records reality.lighting per fresh callback and
// XrController.hitTest results with the same callback's reality fields. Writes api-oracle.json.gz. Output may also go
// under build/slam-refact-full-20261019 (--out-root).
//
// --image-targets <dir> (OR-4): image-target-hook.js merges `imageTargetData` into page-runtime's XrController.configure
// call; <dir> holds <name>.json (image-target-cli format) + <name>_luminance.png (+ optional <name>_luminance.pgm for
// the sha cross-check with the engine input). The harness origin serves the PNGs at /targets/ and rewrites imagePath to
// that URL. Writes image-targets.json (events with the frame they belong to, decode check, configure calls). Needs a new label
// and an --out-root under build/slam-refact-full-20261019. Without the option nothing changes.
//
// Output: build/slam-refact-20261015/reference/runs/<scenario>/<label>/{meta.json,trace.jsonl,callbacks.jsonl,...}
import fs from 'node:fs/promises';
import {createWriteStream} from 'node:fs';
import path from 'node:path';
import http from 'node:http';
import zlib from 'node:zlib';
import os from 'node:os';
import {createHash} from 'node:crypto';
import {fileURLToPath} from 'node:url';
import {parseArgs} from 'node:util';
import {chromium} from '../../../../scripts/dev/node_modules/playwright/index.mjs';
import {parsePGM} from '../../../../scripts/dev/replay-android-input.mjs';
import {buildSensorPackets} from '../../../../scripts/dev/references/xr_reference/sensor_adapter.mjs';
import {decodeEventQueue} from './capnp-event-queue.mjs';

const here = path.dirname(fileURLToPath(import.meta.url));
const root = path.resolve(here, '../../../..');
const engineDir = path.join(root, 'assets/references/xr_reference/analysis-runtime/standalone');
const XR_SLAM_WASM_SHA256 = '780731fe294a725acf1b7717ed5ffb07ad9f0929e408a1d74f47450c291ec422';
const UA = {
  pixel7: 'Mozilla/5.0 (Linux; Android 13; Pixel 7) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/140.0.0.0 Mobile Safari/537.36',
  // Chrome on Android sends the reduced UA (model hidden), which is what reference runtime saw on the Flip6.
  reduced: 'Mozilla/5.0 (Linux; Android 10; K) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/140.0.0.0 Mobile Safari/537.36',
};

const {values} = parseArgs({options: {
  scenario: {type: 'string'},
  label: {type: 'string'},
  'out-root': {type: 'string', default: 'build/slam-refact-20261015/reference/runs'},
  'max-frames': {type: 'string', default: '0'},
  profile: {type: 'boolean', default: false},
  'profile-interval-us': {type: 'string', default: '500'},
  'js-flags': {type: 'string', default: ''},
  wrap: {type: 'boolean', default: false},
  memprobe: {type: 'boolean', default: false},
  'memprobe-paths': {type: 'boolean', default: false},
  'record-inputs': {type: 'boolean', default: false},
  'world-point-sample-every': {type: 'string', default: '50'},
  'api-oracle': {type: 'string'},
  'image-targets': {type: 'string'},
}});
const scenarios = JSON.parse(await fs.readFile(path.join(here, 'scenarios.json'), 'utf8'));
const scenario = scenarios[values.scenario];
if (!scenario) throw new Error('--scenario one of ' + Object.keys(scenarios).join('|'));
if (!/^[a-z0-9-]+$/.test(values.label || '')) throw new Error('--label [a-z0-9-]+ required (new label per attempt)');
if (values['memprobe-paths'] && !values.memprobe) throw new Error('--memprobe-paths requires --memprobe');
if (values.wrap && values.memprobe) throw new Error('--wrap and --memprobe are exclusive (both replace the exports object)');
if (values.wrap && values['record-inputs']) throw new Error('--wrap and --record-inputs are exclusive');
const outRoot = path.resolve(root, values['out-root']);
const owned = path.join(root, 'build/slam-refact-20261015/reference');
const ownedFull = path.join(root, 'build/slam-refact-full-20261019');
if (![owned, ownedFull].some(d => outRoot === d || outRoot.startsWith(d + path.sep))) throw new Error('output must stay under ' + owned + ' or ' + ownedFull);
const apiOracle = values['api-oracle'] ? JSON.parse(values['api-oracle']) : null;
const imageTargetsDir = values['image-targets'] ? path.resolve(root, values['image-targets']) : null;
if (imageTargetsDir && outRoot !== ownedFull && !outRoot.startsWith(ownedFull + path.sep)) throw new Error('--image-targets output must stay under ' + ownedFull);
const out = path.join(outRoot, values.scenario, values.label);
await fs.mkdir(path.dirname(out), {recursive: true});
await fs.mkdir(out, {recursive: false});
const sha = bytes => createHash('sha256').update(bytes).digest('hex');
const stats = xs => {
  const s = xs.filter(Number.isFinite).sort((a, b) => a - b);
  const q = p => s.length ? s[Math.min(s.length - 1, Math.floor((s.length - 1) * p))] : null;
  return {n: s.length, min: s[0] ?? null, p50: q(0.5), p95: q(0.95), p99: q(0.99), max: s.at(-1) ?? null,
    mean: s.length ? s.reduce((a, b) => a + b, 0) / s.length : null};
};

// ---------------------------------------------------------------------------------------------- dataset loading
async function loadDataset() {
  const dir = path.resolve(root, scenario.dataset);
  if (scenario.kind === 'tum') {
    const bytes = await fs.readFile(path.join(dir, 'input.json'));
    const input = JSON.parse(bytes);
    if (input.schema !== 'mobile-slam-synthetic-vio-v1') throw new Error('unexpected fixture schema');
    return {dir, sourceFile: path.join(dir, 'input.json'), sourceSha256: sha(bytes), width: input.config.width, height: input.config.height,
      frames: input.frames.map(f => ({index: f.index, t: f.physicalTimestamp, file: path.join(dir, 'mav0/cam0/data', f.file), sha256: f.sha256})),
      imu: input.imu, timeBase: 'fixture seconds (t=0 at the IMU sample preceding the first frame)'};
  }
  if (scenario.kind === 'flip6') {
    const bytes = await fs.readFile(path.join(dir, 'record.json'));
    const record = JSON.parse(bytes);
    if (record.schema !== 'mobile-slam-android-input-v1') throw new Error('unexpected record schema');
    const rec = record.recording;
    const firstFrame = rec.frames[0].timestamp;
    const t0 = Math.max(...rec.imu.filter(r => r[0] <= firstFrame).map(r => r[0]));
    if (!Number.isFinite(t0)) throw new Error('no IMU sample precedes the first frame');
    const lastFrame = rec.frames.at(-1).timestamp;
    return {dir, sourceFile: path.join(dir, 'record.json'), sourceSha256: sha(bytes), width: rec.width, height: rec.height,
      frames: rec.frames.map((f, i) => ({index: i, t: +(f.timestamp - t0).toFixed(9), file: path.join(dir, 'mav0/cam0/data', f.file), sha256: f.sha256,
        sourceTimestamp: f.timestamp})),
      imu: rec.imu.filter(r => r[0] >= t0 && r[0] <= lastFrame + 0.1).map(r => [+(r[0] - t0).toFixed(9), ...r.slice(1)]),
      t0Source: t0, timeBase: 'record seconds minus t0Source (IMU sample preceding the first frame); record clock = phone performance.now()/1000',
      recordConfig: rec.config, recordCalibration: rec.calibration};
  }
  throw new Error('unknown scenario kind');
}

// recorded-inputs.jsonl.gz: line 1 = header (epoch, publications = page-runtime frames [index, videoTimeUs, publishedMs,
// acceptedMs], field names); then one object per recorded call in call order.  Stage calls carry `i` (input frame by
// videoTime, same rule as capture_window.mjs) and the decoded event queue staged with them (`Q` precedes `S` in the glue).
async function writeRecordedInputs(rec, observed, frames) {
  const byUs = new Map(observed.frames.map(([index, us]) => [us, index]));
  const frameOf = v => { const us = Math.round(v * 1e6); return byUs.get(us) ?? byUs.get(us - 1) ?? byUs.get(us + 1) ?? null; };
  const tOf = new Map(frames.map(f => [f.index, f.t]));
  const stageArgNames = ['texture', 'width', 'height', 'rotation', 'alpha', 'beta', 'gamma', 'qw', 'qx', 'qy', 'qz',
    'videoTimeSeconds', 'frameTimeSeconds', 'timeNanos', 'latitude', 'longitude', 'accuracy'];
  const lines = [];
  let pendingQueue = null, lastStage = null, stages = 0, matched = 0, queueEvents = 0, decodeErrors = 0;
  const seen = new Set(), dupFrames = [];
  for (const c of rec.calls) {
    const [seq, tag, at] = c;
    if (tag === 'Q') {
      const bytes = c[4] == null ? null : Buffer.from(c[4], 'base64');
      let events = null, decodeError = null;
      try { events = bytes ? decodeEventQueue(bytes) : null; } catch (error) { decodeError = String(error.message || error); decodeErrors++; }
      if (events) queueEvents += events.length;
      pendingQueue = {seq, ptr: c[3][0], len: c[3][1], sha256: bytes ? sha(bytes) : null};
      lines.push({seq, fn: 'setEventQueue', at: +at.toFixed(3), len: c[3][1], b64: c[4], events, decodeError});
    } else if (tag === 'S') {
      const args = c[3];
      const i = frameOf(args[11]);
      stages++;
      if (i != null) { matched++; if (seen.has(i)) dupFrames.push(i); seen.add(i); }
      lastStage = {seq, i};
      lines.push({seq, fn: 'stageFrame', at: +at.toFixed(3), i, t: i == null ? null : tOf.get(i), args, queueSeq: pendingQueue?.seq ?? null});
      pendingQueue = null;
    } else if (tag === 'P') {
      lines.push({seq, fn: 'processStagedFrame', at: +at.toFixed(3), args: c[3], probeSeq: c[4], stageSeq: lastStage?.seq ?? null, i: lastStage?.i ?? null});
    } else lines.push({seq, fn: {I: 'engineInit', O: 'engineOrientationChange', V: 'engineVideoSizeChange'}[tag], at: +at.toFixed(3), args: c[3]});
  }
  const header = {schema: 'slam-refact-xr_reference-recorded-inputs-v1', scenario: values.scenario, label: values.label, wrapped: rec.wrapped,
    error: rec.error, droppedCalls: rec.dropped, calls: rec.calls.length, epochWallMs: observed.epochWallMs, publications: observed.frames,
    stageArgNames, eventQueueFields: 'decoded from the capnp bytes (capnp-event-queue.mjs): kind (PositionalSensorKind enum), timestampNanos (glue performance.now at the handler), eventTimestampNanos (event.timeStamp), intervalNanos, x, y, z',
    association: 'stageFrame.i = input frame whose published videoTime (us, +-1) equals args.videoTimeSeconds; processStagedFrame.i = i of the preceding stageFrame; processStagedFrame.probeSeq = memprobe seq'};
  const text = [header, ...lines].map(o => JSON.stringify(o)).join('\n') + '\n';
  await fs.writeFile(path.join(out, 'recorded-inputs.jsonl.gz'), zlib.gzipSync(text, {level: 9, mtime: 0}));
  return {file: 'recorded-inputs.jsonl.gz', wrapped: rec.wrapped, error: rec.error, calls: rec.calls.length, droppedCalls: rec.dropped,
    stageCalls: stages, stageCallsMatched: matched, duplicateStagedFrames: dupFrames.length, queueEvents, queueDecodeErrors: decodeErrors};
}

const report = {schema: 'slam-refact-xr_reference-reference-run-v1', scenario: values.scenario, scenarioSpec: scenario, label: values.label,
  startedAt: new Date().toISOString(), argv: process.argv, node: process.version, loadAvgAtStart: os.loadavg(), cpus: os.cpus().length,
  implementation: 'Original SDK bytes (xr.js + xr-slam.js), public XR8 API, synthetic camera track and untrusted sensor events',
  requests: [], blockedExternal: [], console: [], pageErrors: [], requestFailures: [], pixelHashMismatches: []};
let server, browser;
try {
  const data = await loadDataset();
  const maxFrames = Number(values['max-frames']);
  const frames = maxFrames > 0 ? data.frames.slice(0, maxFrames) : data.frames;
  const imu = data.imu.filter(r => r[0] <= frames.at(-1).t + 0.05);
  const sensors = buildSensorPackets(imu);
  const dts = imu.slice(1).map((r, i) => r[0] - imu[i][0]).sort((a, b) => a - b);
  const intervalMs = +(dts[Math.floor(dts.length / 2)] * 1000).toFixed(3);
  const frameDts = frames.slice(1).map((f, i) => f.t - frames[i].t).sort((a, b) => a - b);
  const nominalFps = Math.round(1 / frameDts[Math.floor(frameDts.length / 2)]);
  const fixture = {config: {width: data.width, height: data.height}, nominalFps, scale: scenario.scale || 'responsive',
    worldPointSampleEvery: Number(values['world-point-sample-every']),
    frames: frames.map(f => ({index: f.index, physicalTimestamp: f.t})),
    sensorPackets: sensors.packets.map(p => ({...p, intervalMs}))};
  report.input = {dataset: data.dir, sourceFile: data.sourceFile, sourceSha256: data.sourceSha256, width: data.width, height: data.height,
    frames: frames.length, firstT: frames[0].t, lastT: frames.at(-1).t, imuRows: imu.length, imuIntervalMsMedian: intervalMs, nominalFps,
    timeBase: data.timeBase, t0Source: data.t0Source ?? null, recordConfig: data.recordConfig ?? null, recordCalibration: data.recordCalibration ?? null,
    sensorProvenance: sensors.provenance, groundTruthProvidedToEngine: false};
  // Engine-input sensor stream (device frame, exactly what is dispatched), stored once per scenario.
  const sensorFile = path.join(path.dirname(out), `engine-input-sensors${maxFrames > 0 ? '-first' + maxFrames : ''}.jsonl.gz`);
  const sensorLines = fixture.sensorPackets.map(p => JSON.stringify({t: p.sourceTimestamp, accelIncludingGravity: p.deviceAcceleration,
    rotationRateDegS: [p.rotationRate.alpha, p.rotationRate.beta, p.rotationRate.gamma], orientation: [p.orientation.alpha, p.orientation.beta, p.orientation.gamma],
    qEarthFromDevice: p.quaternionEarthFromDevice})).join('\n') + '\n';
  const sensorGz = zlib.gzipSync(sensorLines, {level: 9, mtime: 0});
  try { await fs.writeFile(sensorFile, sensorGz, {flag: 'wx'}); } catch (error) { if (error.code !== 'EEXIST') throw error; }
  report.input.engineInputSensors = {file: path.relative(root, sensorFile), sha256Uncompressed: sha(Buffer.from(sensorLines)),
    sha256File: sha(await fs.readFile(sensorFile)),
    fields: 't = fixture seconds; W3C device frame; accelIncludingGravity m/s^2; rotationRate deg/s (alpha=x,beta=y,gamma=z); orientation deg Z-X-Y; dispatched at epoch + t*1000 with event.timeStamp set to the same value'};

  const stateScript = `window.__probe={ready:false,started:false,done:false,errors:[],events:[],rows:[],frames:[],imuDispatched:0,cameraRequests:[],intrinsicsChanges:[],worldPointSamples:[]};
window.addEventListener('error',e=>window.__probe.errors.push({kind:'window',message:e.message,stack:e.error?.stack}));
window.addEventListener('unhandledrejection',e=>window.__probe.errors.push({kind:'promise',message:String(e.reason),stack:e.reason?.stack}));`;
  const html = `<!doctype html><html><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>reference runtime reference run</title><script src="/state.js"></script><script src="/page-runtime.js" defer></script><script src="/engine/xr.js" async data-preload-chunks="slam"></script></head><body style="margin:0"><button id="start" style="position:absolute">start</button><canvas id="view" width="${data.width}" height="${data.height}"></canvas></body></html>`;
  const runtimeBytes = await fs.readFile(path.join(here, 'page-runtime.js'));
  const served = {'index.html': Buffer.from(html), 'state.js': Buffer.from(stateScript), 'page-runtime.js': runtimeBytes};
  report.sourceHashes = {};
  for (const name of ['run-reference.mjs', 'run-reference-it.mjs', 'page-runtime.js', 'wasm-wrap.js', 'memprobe.js', 'input-recorder.js', 'capnp-event-queue.mjs', 'scenarios.json', 'api-oracle.js']) report.sourceHashes[name] = sha(await fs.readFile(path.join(here, name)));
  // OR-4 image targets: the files served at /targets/ and the imageTargetData handed to configure (imagePath rewritten to the served URL).
  let imageTargets = null;
  if (imageTargetsDir) {
    imageTargets = {files: new Map(), data: [], sources: {}};
    for (const f of (await fs.readdir(imageTargetsDir)).filter(n => n.endsWith('.json')).sort()) {
      const jsonBytes = await fs.readFile(path.join(imageTargetsDir, f));
      const target = JSON.parse(jsonBytes);
      const stem = f.slice(0, -5), pngName = `${stem}_luminance.png`, pgmName = `${stem}_luminance.pgm`;
      const png = await fs.readFile(path.join(imageTargetsDir, pngName));
      imageTargets.files.set(pngName, png);
      const pgmBytes = await fs.readFile(path.join(imageTargetsDir, pgmName)).catch(() => null);
      imageTargets.data.push({...target, imagePath: '/targets/' + pngName});
      imageTargets.sources[stem] = {json: sha(jsonBytes), png: sha(png), pgm: pgmBytes ? sha(pgmBytes) : null, servedAs: '/targets/' + pngName};
    }
    if (!imageTargets.data.length) throw new Error('--image-targets: no *.json in ' + imageTargetsDir);
    report.sourceHashes['image-target-hook.js'] = sha(await fs.readFile(path.join(here, 'image-target-hook.js')));
    report.imageTargets = {dir: path.relative(root, imageTargetsDir), sources: imageTargets.sources};
  }
  report.sourceHashes['sensor_adapter.mjs'] = sha(await fs.readFile(path.join(root, 'scripts/dev/references/xr_reference/sensor_adapter.mjs')));
  report.sourceHashes['index.html'] = sha(served['index.html']);
  report.sourceHashes['state.js'] = sha(served['state.js']);
  report.engineAssets = {};
  for (const name of ['xr.js', 'xr-slam.js']) { const b = await fs.readFile(path.join(engineDir, name)); report.engineAssets[name] = {bytes: b.length, sha256: sha(b)}; }

  const serve = async (req, res) => {
    try {
      const pathname = new URL(req.url, 'http://localhost').pathname;
      let bytes, type = 'application/octet-stream';
      if (pathname === '/fixture.json') { bytes = Buffer.from(JSON.stringify(fixture)); type = 'application/json'; }
      else if (/^\/gray\/\d+$/.test(pathname)) {
        const f = frames[Number(pathname.split('/').at(-1))];
        if (!f) throw new Error('unknown frame');
        bytes = parsePGM(await fs.readFile(f.file), data.width, data.height);
        if (sha(bytes) !== f.sha256) report.pixelHashMismatches.push(f.index);
      } else if (pathname === '/' || Object.hasOwn(served, pathname.slice(1))) {
        const name = pathname === '/' ? 'index.html' : pathname.slice(1);
        bytes = served[name]; type = name.endsWith('.html') ? 'text/html' : 'text/javascript';
      } else if (imageTargets && pathname.startsWith('/targets/') && imageTargets.files.has(pathname.slice(9))) {
        bytes = imageTargets.files.get(pathname.slice(9)); type = 'image/png';
      } else if (pathname.startsWith('/engine/')) {
        const file = path.resolve(engineDir, pathname.slice(8));
        if (!file.startsWith(engineDir + path.sep)) throw new Error('unserved path');
        bytes = await fs.readFile(file); type = file.endsWith('.js') ? 'text/javascript' : 'application/octet-stream';
      } else throw new Error('unserved path');
      res.writeHead(200, {'Content-Type': type, 'Cache-Control': 'no-store'}); res.end(bytes);
    } catch (error) { res.writeHead(404); res.end(String(error)); }
  };
  server = http.createServer((req, res) => void serve(req, res));
  await new Promise(resolve => server.listen(0, '127.0.0.1', resolve));
  const origin = `http://127.0.0.1:${server.address().port}`;
  report.origin = origin;
  const args = ['--enable-unsafe-swiftshader', '--autoplay-policy=no-user-gesture-required', '--use-fake-ui-for-media-stream', '--use-fake-device-for-media-stream'];
  if (values['js-flags']) args.push('--js-flags=' + values['js-flags']);
  const launch = {executablePath: process.env.BROWSER_EXECUTABLE || '/usr/bin/google-chrome', headless: true, args};
  report.browserLaunch = launch;
  browser = await chromium.launch(launch);
  report.browserVersion = browser.version();
  const viewport = scenario.viewport ? {width: scenario.viewport[0], height: scenario.viewport[1]} : {width: data.width, height: data.height};
  const contextOptions = {viewport, permissions: ['camera', 'microphone'], isMobile: true, hasTouch: true, deviceScaleFactor: 1, userAgent: UA[scenario.ua || 'pixel7']};
  report.browserContext = contextOptions;
  const context = await browser.newContext(contextOptions);
  const page = await context.newPage();
  if (values.wrap) {
    const wrapSource = await fs.readFile(path.join(here, 'wasm-wrap.js'), 'utf8');
    await page.addInitScript(`${wrapSource}\nwindow.__wasmWrapInit(${JSON.stringify({targetSha256: XR_SLAM_WASM_SHA256, maxCallLog: 200000})});`);
  }
  if (values['record-inputs']) {  // before memprobe.js: memprobe instantiates the compiled Module through this wrapper
    const recSource = await fs.readFile(path.join(here, 'input-recorder.js'), 'utf8');
    await page.addInitScript(`${recSource}\nwindow.__inputRecInit(${JSON.stringify({targetSha256: XR_SLAM_WASM_SHA256})});`);
  }
  if (values.memprobe) {
    const probeSource = await fs.readFile(path.join(here, 'memprobe.js'), 'utf8');
    const probeOptions = values['memprobe-paths'] ? {targetSha256: XR_SLAM_WASM_SHA256, paths: true} : {targetSha256: XR_SLAM_WASM_SHA256};
    await page.addInitScript(`${probeSource}\nwindow.__memProbeInit(${JSON.stringify(probeOptions)});`);
  }
  if (apiOracle) {
    const oracleSource = await fs.readFile(path.join(here, 'api-oracle.js'), 'utf8');
    await page.addInitScript(`${oracleSource}\nwindow.__apiOracleInit(${JSON.stringify(apiOracle)});`);
    report.apiOracle = {options: apiOracle};
  }
  if (imageTargets) {
    const hookSource = await fs.readFile(path.join(here, 'image-target-hook.js'), 'utf8');
    await page.addInitScript(`${hookSource}\nwindow.__imageTargetHookInit(${JSON.stringify({imageTargetData: imageTargets.data})});`);
  }
  const cdp = await context.newCDPSession(page);
  await page.route('**/*', route => {
    const url = route.request().url();
    if (!url.startsWith(origin + '/') && !url.startsWith('data:') && !url.startsWith('blob:')) { report.blockedExternal.push(url); return route.abort('blockedbyclient'); }
    return route.continue();
  });
  page.on('response', r => report.requests.push({url: r.url().replace(origin, ''), status: r.status()}));
  page.on('requestfailed', r => report.requestFailures.push({url: r.url(), failure: r.failure()}));
  page.on('console', m => { if (report.console.length < 400) report.console.push({type: m.type(), text: m.text().slice(0, 500)}); });
  page.on('pageerror', e => report.pageErrors.push({message: e.message, stack: e.stack}));
  await page.goto(origin + '/index.html');
  await page.waitForFunction(() => window.__probe?.ready, null, {timeout: 60000});
  report.publicApi = await page.evaluate(() => ({version: XR8.version(), deviceEstimate: XR8.XrDevice.deviceEstimate(),
    screen: {w: screen.width, h: screen.height, orientationType: screen.orientation?.type, orientationAngle: screen.orientation?.angle, windowOrientation: window.orientation, dpr: devicePixelRatio},
    userAgent: navigator.userAgent}));
  if (values.profile) {
    await cdp.send('Profiler.enable');
    await cdp.send('Profiler.setSamplingInterval', {interval: Number(values['profile-interval-us'])});
    await page.exposeFunction('__refProfilerStart', async () => { await cdp.send('Profiler.start'); report.profilerStartedAt = Date.now(); });
    await page.evaluate(() => { window.__refBegin = () => { void window.__refProfilerStart(); }; });
  }
  const wallStart = Date.now();
  await page.locator('#start').click();
  const lastT = frames.at(-1).t;
  try { await page.waitForFunction(() => window.__probe?.done, null, {timeout: Math.round(lastT * 1000) + 120000, polling: 500}); }
  catch (error) { report.waitError = String(error); }
  report.wallSeconds = (Date.now() - wallStart) / 1000;
  if (values.profile) {
    const {profile} = await cdp.send('Profiler.stop');
    await fs.writeFile(path.join(out, 'profile.cpuprofile'), JSON.stringify(profile));
    report.profile = {file: 'profile.cpuprofile', intervalUs: Number(values['profile-interval-us']), nodes: profile.nodes.length, samples: profile.samples.length,
      durationMs: (profile.endTime - profile.startTime) / 1000, jsFlags: values['js-flags'] || null};
    // Map wasm script ids to module bytes (sha256) so the xr-slam module can be identified in the profile.
    const scripts = [];
    cdp.on('Debugger.scriptParsed', e => { if (e.scriptLanguage === 'WebAssembly' || e.url.startsWith('wasm://')) scripts.push(e); });
    await cdp.send('Debugger.enable');
    await new Promise(resolve => setTimeout(resolve, 1000));
    const wasmScripts = [];
    for (const s of scripts) {
      const src = await cdp.send('Debugger.getScriptSource', {scriptId: s.scriptId});
      const bytes = src.bytecode ? Buffer.from(src.bytecode, 'base64') : null;
      wasmScripts.push({scriptId: s.scriptId, url: s.url, codeOffset: s.codeOffset ?? null, bytes: bytes?.length ?? null, sha256: bytes ? sha(bytes) : null,
        isXrSlam: bytes ? sha(bytes) === XR_SLAM_WASM_SHA256 : false});
    }
    await cdp.send('Debugger.disable');
    await fs.writeFile(path.join(out, 'wasm-scripts.json'), JSON.stringify(wasmScripts, null, 1) + '\n');
    report.profile.wasmScripts = wasmScripts;
  }
  if (values.wrap) {
    const wrap = await page.evaluate(() => window.__wasmWrap);
    await fs.writeFile(path.join(out, 'wasm-wrap.json'), JSON.stringify(wrap));
    report.wrap = {file: 'wasm-wrap.json', modules: wrap?.modules, error: wrap?.error, frames: wrap?.frameSeq, calls: wrap?.calls?.length};
  }
  const memProbe = values.memprobe ? await page.evaluate(() => window.__memProbe) : null;
  const inputRec = values['record-inputs'] ? await page.evaluate(() => window.__inputRec) : null;
  const oracle = apiOracle ? await page.evaluate(() => window.__apiOracle) : null;
  const itHook = imageTargets ? await page.evaluate(() => window.__imageTargetHook) : null;
  try { await page.evaluate(() => window.__finishProbe?.()); } catch (error) { report.cleanupError = String(error); }
  const observed = await page.evaluate(() => window.__probe);
  report.memory = await page.evaluate(() => performance.memory ? {usedJSHeapSize: performance.memory.usedJSHeapSize, totalJSHeapSize: performance.memory.totalJSHeapSize} : null);
  await page.screenshot({path: path.join(out, 'screenshot.png')});

  // ------------------------------------------------------------------------------------------ trace assembly
  const epoch = observed.epochWallMs;
  const byUs = new Map(observed.frames.map(([index, us, published, accepted]) => [us, {index, us, published, accepted}]));
  const callbackKeys = ['wallMs', 'videoTime', 'repeat', 'status', 'reason', 'px', 'py', 'pz', 'qw', 'qx', 'qy', 'qz', 'worldPoints', 'wpAdded', 'wpRemoved',
    'wpMeanConfidence', 'gpuMs', 'cpuMs', 'frameTime', 'textureWidth', 'textureHeight', 'orientation'];
  if (values.memprobe) callbackKeys.push('probeSeq');
  const seqToFrame = new Map();
  const cbStream = createWriteStream(path.join(out, 'callbacks.jsonl'));
  const fresh = new Map();
  let bootstrapCallbacks = 0, unmatchedFresh = 0;
  for (const row of observed.rows) {
    const obj = Object.fromEntries(callbackKeys.map((k, i) => [k, row[i] ?? null]));
    const us = Math.round(obj.videoTime * 1e6);
    let match = byUs.get(us) || byUs.get(us - 1) || byUs.get(us + 1);
    obj.frameIndex = match ? match.index : null;
    if (!match) bootstrapCallbacks++;
    cbStream.write(JSON.stringify(obj) + '\n');
    if (!obj.repeat && match) {
      if (!fresh.has(match.index)) fresh.set(match.index, obj);
      if (obj.probeSeq != null && !seqToFrame.has(obj.probeSeq)) seqToFrame.set(obj.probeSeq, match.index);
    }
    else if (!obj.repeat && !match && obj.videoTime * 1e6 >= Math.round(epoch * 1000)) unmatchedFresh++;
  }
  await new Promise(resolve => cbStream.end(resolve));
  const traceStream = createWriteStream(path.join(out, 'trace.jsonl'));
  const traceRows = [];
  const publications = new Map(observed.frames.map(([index, us, published, accepted]) => [index, {us, published, accepted}]));
  for (const f of frames) {
    const pub = publications.get(f.index);
    const cb = fresh.get(f.index);
    const finite = cb && [cb.px, cb.py, cb.pz, cb.qw, cb.qx, cb.qy, cb.qz].every(Number.isFinite);
    const row = {i: f.index, t: f.t, videoTimeUs: pub?.us ?? null,
      publishMs: pub ? +(pub.published - epoch).toFixed(3) : null, acceptedMs: pub && pub.accepted > 0 ? +(pub.accepted - epoch).toFixed(3) : null,
      fresh: !!cb, callbackMs: cb ? +(cb.wallMs - epoch).toFixed(3) : null, latencyMs: cb && pub ? +(cb.wallMs - pub.published).toFixed(3) : null,
      status: cb?.status ?? null, reason: cb?.reason ?? null,
      p: finite ? [cb.px, cb.py, cb.pz] : null, q_wxyz: finite ? [cb.qw, cb.qx, cb.qy, cb.qz] : null,
      worldPoints: cb?.worldPoints ?? null, wpAdded: cb?.wpAdded ?? null, wpRemoved: cb?.wpRemoved ?? null, wpMeanConfidence: cb?.wpMeanConfidence ?? null,
      xrGpuStageMs: cb ? +cb.gpuMs.toFixed(3) : null, xrCpuStageMs: cb ? +cb.cpuMs.toFixed(3) : null};
    traceRows.push(row);
    traceStream.write(JSON.stringify(row) + '\n');
  }
  await new Promise(resolve => traceStream.end(resolve));
  await fs.writeFile(path.join(out, 'events.json'), JSON.stringify({events: observed.events, intrinsicsChanges: observed.intrinsicsChanges,
    cameraRequests: observed.cameraRequests, errors: observed.errors}, null, 1) + '\n');
  if (observed.worldPointSamples.length) await fs.writeFile(path.join(out, 'worldpoints-sampled.jsonl'),
    observed.worldPointSamples.map(s => JSON.stringify({frameIndex: (byUs.get(Math.round(s.videoTime * 1e6)) || {}).index ?? null, ...s})).join('\n') + '\n');

  if (memProbe) {
    const tOf = new Map(frames.map(f => [f.index, f.t]));
    const lines = [JSON.stringify({schema: 'slam-refact-xr_reference-memprobe-v1', scenario: values.scenario, label: values.label,
      wasmSha256: memProbe.target?.sha256 ?? null, vtable: memProbe.vtable, fields: memProbe.fields, error: memProbe.error,
      modules: memProbe.modules, scans: memProbe.scans, rootHistory: memProbe.rootHistory, records: memProbe.records.length,
      association: 'i = input frame index of the first fresh (non-repeat) callback whose last processStagedFrame seq is this record'})];
    for (const rec of memProbe.records) {
      const obj = Object.fromEntries(memProbe.fields.map((k, j) => [k, rec[j]]));
      const i = seqToFrame.get(obj.seq);
      lines.push(JSON.stringify({i: i ?? null, t: i == null ? null : tOf.get(i), ...obj}));
    }
    await fs.writeFile(path.join(out, 'memprobe.jsonl.gz'), zlib.gzipSync(lines.join('\n') + '\n', {level: 9, mtime: 0}));
    const recs = memProbe.records;
    report.memprobe = {file: 'memprobe.jsonl.gz', records: recs.length, associated: seqToFrame.size, error: memProbe.error,
      rootScans: memProbe.scans.length, rootHistory: memProbe.rootHistory, wasmSha256: memProbe.target?.sha256 ?? null,
      probeMs: stats(recs.map(r => r[3])), callMs: stats(recs.map(r => r[2]))};
  }
  if (inputRec) report.recordedInputs = await writeRecordedInputs(inputRec, observed, frames);
  if (itHook) {
    // Events carry the videoTime of the onUpdate that follows their dispatch; frame association as the trace (us, +-1).
    const frameOfIt = v => { const us = Math.round(v * 1e6); const m = byUs.get(us) || byUs.get(us - 1) || byUs.get(us + 1); return m ? m.index : null; };
    for (const e of itHook.events) e.frameIndex = frameOfIt(e.videoTime);
    for (const r of itHook.detectedImagesRows) r.frameIndex = frameOfIt(r.videoTime);
    await fs.writeFile(path.join(out, 'image-targets.json'), JSON.stringify({schema: 'slam-refact-xr_reference-image-targets-v1', scenario: values.scenario,
      label: values.label, sources: imageTargets.sources, ...itHook}, null, 1) + '\n');
    const count = k => itHook.events.filter(e => e.name === k).length;
    report.imageTargetsRun = {file: 'image-targets.json', installed: itHook.installed, error: itHook.error, configureCalls: itHook.configureCalls,
      decode: itHook.decode, realityKeys: itHook.realityKeys, detectedImagesCallbacks: itHook.detectedImagesSeen,
      events: {loading: count('reality.imageloading'), scanning: count('reality.imagescanning'), found: count('reality.imagefound'),
        updated: count('reality.imageupdated'), lost: count('reality.imagelost')}};
  }
  if (oracle) {
    // Frame association as the trace: input frame whose published videoTime (us, +-1) equals the callback's.
    const byUsOracle = new Map(observed.frames.map(([index, us]) => [us, index]));
    const frameOf = v => { const us = Math.round(v * 1e6); return byUsOracle.get(us) ?? byUsOracle.get(us - 1) ?? byUsOracle.get(us + 1) ?? null; };
    for (const row of oracle.lighting) row.frameIndex = frameOf(row.videoTime);
    for (const row of oracle.hits) row.frameIndex = frameOf(row.videoTime);
    for (const row of oracle.ab || []) row.frameIndex = frameOf(row.videoTime);
    await fs.writeFile(path.join(out, 'api-oracle.json.gz'), zlib.gzipSync(JSON.stringify({schema: 'slam-refact-xr_reference-api-oracle-v1',
      scenario: values.scenario, label: values.label, ...oracle}), {level: 9, mtime: 0}));
    report.apiOracle = {...report.apiOracle, file: 'api-oracle.json.gz', installed: oracle.installed, error: oracle.error,
      fresh: oracle.fresh, lightingRows: oracle.lighting.length, hitSamples: oracle.hits.length,
      queries: oracle.hits.reduce((n, h) => n + h.queries.length, 0), abSamples: (oracle.ab || []).length,
      abQueries: (oracle.ab || []).reduce((n, a) => n + a.steps.reduce((m, st) => m + st.queries.length, 0), 0), configureCalls: oracle.configureCalls, projectionCalls: oracle.projectionCalls};
  }
  const freshRows = traceRows.filter(r => r.fresh);
  const statusCounts = {};
  for (const r of traceRows) { const k = r.fresh ? `${r.status}/${r.reason}` : 'NO_FRESH_CALLBACK'; statusCounts[k] = (statusCounts[k] || 0) + 1; }
  const firstPose = traceRows.find(r => r.p);
  const firstNormal = traceRows.find(r => r.status === 'NORMAL' && r.p);
  const firstPoints = traceRows.find(r => r.worldPoints > 0);
  const repeatCallbacks = observed.rows.filter(r => r[2] === 1).length;
  report.summary = {
    inputFrames: frames.length, publishedFrames: observed.frames.length, freshCallbacks: freshRows.length, unmatchedFreshCallbacks: unmatchedFresh,
    repeatCallbacks, bootstrapCallbacks, totalCallbacks: observed.rows.length, imuDispatched: observed.imuDispatched, imuRows: imu.length,
    statusCounts, normalRatio: traceRows.filter(r => r.status === 'NORMAL').length / frames.length,
    limitedRatio: traceRows.filter(r => r.status === 'LIMITED').length / frames.length,
    finitePoseRatio: traceRows.filter(r => r.p).length / frames.length,
    timeToFirstPose: firstPose ? {frame: firstPose.i, t: firstPose.t, sinceFirstFrameS: firstPose.t - frames[0].t, wallSinceEpochMs: firstPose.callbackMs} : null,
    timeToFirstNormal: firstNormal ? {frame: firstNormal.i, t: firstNormal.t, sinceFirstFrameS: firstNormal.t - frames[0].t} : null,
    timeToFirstWorldPoints: firstPoints ? {frame: firstPoints.i, t: firstPoints.t, sinceFirstFrameS: firstPoints.t - frames[0].t} : null,
    publishToCallbackMs: stats(freshRows.map(r => r.latencyMs)),
    xrCpuStageMs: stats(freshRows.map(r => r.xrCpuStageMs)), xrGpuStageMs: stats(freshRows.map(r => r.xrGpuStageMs)),
    worldPoints: stats(freshRows.map(r => r.worldPoints)),
    publishLagMs: stats(traceRows.filter(r => r.publishMs !== null).map(r => r.publishMs - r.t * 1000)),
    intrinsics: observed.intrinsicsChanges.map(c => ({textureWidth: c.textureWidth, textureHeight: c.textureHeight, orientation: c.orientation,
      fxPx: c.intrinsics[0] * c.textureWidth / 2, fyPx: c.intrinsics[5] * c.textureHeight / 2, m00: c.intrinsics[0], m11: c.intrinsics[5], m02: c.intrinsics[8], m12: c.intrinsics[9]})),
    onStart: observed.events.find(e => e.name === 'onStart') || null,
    errors: observed.errors.length, pageErrors: report.pageErrors.length, blockedExternal: report.blockedExternal.length,
    pixelHashMismatches: report.pixelHashMismatches.length, resourceCleanup: observed.ownedResourcesClosed === true,
  };
  report.status = report.pageErrors.length || observed.errors.length ? 'COMPLETED_WITH_ERRORS' : freshRows.length ? 'COMPLETED' : 'NO_FRESH_CALLBACKS';
} catch (error) {
  report.status = 'FAILED'; report.error = {message: error.message, stack: error.stack};
} finally {
  if (browser) { await browser.close(); report.browserClosed = true; }
  if (server) { await new Promise(resolve => server.close(resolve)); report.serverClosed = true; }
  report.finishedAt = new Date().toISOString();
  report.loadAvgAtEnd = os.loadavg();
  await fs.writeFile(path.join(out, 'meta.json'), JSON.stringify(report, null, 1) + '\n');
  const s = report.summary || {};
  console.log(JSON.stringify({status: report.status, out: path.relative(root, out), frames: s.inputFrames, fresh: s.freshCallbacks, statusCounts: s.statusCounts,
    timeToFirstPose: s.timeToFirstPose, intrinsics: s.intrinsics, error: report.error?.message, waitError: report.waitError}, null, 1));
  process.exitCode = report.status === 'FAILED' ? 1 : report.status === 'NO_FRESH_CALLBACKS' ? 2 : 0;
}

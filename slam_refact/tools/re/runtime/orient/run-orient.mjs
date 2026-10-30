// US-023 / US-024: run the ORIGINAL reference runtime SDK headless in a chosen screen orientation / device UA and record what
// the orientation and iOS paths do. New harness (copy-based; run-reference.mjs / capture_window.mjs are not modified):
//
//   node slam_refact/tools/re/runtime/orient/run-orient.mjs --scenario flip6-s2 --label <new label> \
//        --orientation 90 [--ua pixel7|reduced|iphone-<name>] [--max-frames N] [--max-captures K --from-stage S] \
//        [--probe-wasm <instrumented xr-slam.wasm>] [--out-root build/slam-refact-full-20261019/orient/original]
//
// --orientation A (0|90|180|270): the camera frames served to the page are the scenario's portrait frames P turned so
//   that the original's stage-1 shader (rotation A, S1 §5.1.1) maps them back to P:
//     90 : L(x', y') = P(Wp-1-y', x')    (landscape, Wp x Hp -> Hp x Wp)
//     180: L(x', y') = P(Wp-1-x', Hp-1-y')
//     270: L(x', y') = P(y', Hp-1-x')
//   The viewport follows the frame size, screen.orientation.angle / window.orientation are set to A, the IMU stream is
//   the scenario's device-frame stream unchanged (the engine's device pose carries no screen-orientation term).
// --ua iphone-<name>: iPhone Safari UA, navigator.platform iPhone, screen / dpr of the named device (see IPHONES).
//
// Output: <out-root>/<scenario>/<label>/{meta.json, trace.jsonl, callbacks-raw.json, cap-NNN.rgba.gz, calls.json,
//   intrinsics.json, frames-served.json}. Existing labels are never overwritten.
import fs from 'node:fs/promises';
import path from 'node:path';
import http from 'node:http';
import zlib from 'node:zlib';
import os from 'node:os';
import {createHash} from 'node:crypto';
import {fileURLToPath} from 'node:url';
import {parseArgs} from 'node:util';
import {chromium} from '../../../../../scripts/dev/node_modules/playwright/index.mjs';
import {parsePGM} from '../../../../../scripts/dev/replay-android-input.mjs';
import {intrinsicsFile, probeFile} from './channel.mjs';
import {buildSensorPackets} from '../../../../../scripts/dev/references/xr_reference/sensor_adapter.mjs';

const here = path.dirname(fileURLToPath(import.meta.url));
const root = path.resolve(here, '../../../../..');
const laneC = path.resolve(here, '..');
const engineDir = path.join(root, 'assets/references/xr_reference/analysis-runtime/standalone');
const XR_SLAM_WASM_SHA256 = '780731fe294a725acf1b7717ed5ffb07ad9f0929e408a1d74f47450c291ec422';
const IPHONE_UA = 'Mozilla/5.0 (iPhone; CPU iPhone OS 17_5 like Mac OS X) AppleWebKit/605.1.15 (KHTML, like Gecko) Version/17.5 Mobile/15E148 Safari/604.1';
const UA = {
  pixel7: 'Mozilla/5.0 (Linux; Android 13; Pixel 7) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/140.0.0.0 Mobile Safari/537.36',
  reduced: 'Mozilla/5.0 (Linux; Android 10; K) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/140.0.0.0 Mobile Safari/537.36',
};
// iPhone screen (CSS px, portrait) and devicePixelRatio. deviceEstimate() reads screen.height * devicePixelRatio
// (xr.js TA/B): the model string depends on that product only, because the unmasked WebGL renderer of this headless
// browser (SwiftShader) is never "Apple A## GPU".
const IPHONES = {
  'iphone-se1': {w: 320, h: 568, dpr: 2},        // 1136  -> "iPhone 5/5s/5c/SE"
  'iphone-8': {w: 375, h: 667, dpr: 2},          // 1334  -> "iPhone 6/6s/7/8"
  'iphone-xr': {w: 414, h: 896, dpr: 2},         // 1792  -> "iPhone XR/11"
  'iphone-12': {w: 390, h: 844, dpr: 3},         // 2532  -> "iPhone 13/13Pro"
  'iphone-12promax': {w: 428, h: 926, dpr: 3},   // 2778  -> "iPhone13,4"
  'iphone-14pro': {w: 393, h: 852, dpr: 3},      // 2556  -> "iPhone15,3"
  'iphone-15plus': {w: 430, h: 932, dpr: 3},     // 2796  -> "iPhone15,4"
};

const {values} = parseArgs({options: {
  scenario: {type: 'string'}, label: {type: 'string'},
  orientation: {type: 'string', default: '0'}, ua: {type: 'string', default: ''},
  'max-frames': {type: 'string', default: '60'}, 'max-captures': {type: 'string', default: '0'},
  rows: {type: 'string', default: '660'}, 'from-stage': {type: 'string', default: '1'},
  'probe-wasm': {type: 'string'},
  'out-root': {type: 'string', default: 'build/slam-refact-full-20261019/orient/original'},
  'world-point-sample-every': {type: 'string', default: '0'},
  // --interval-pattern a,b,c: DeviceMotion `interval` (the W3C number the page dispatches: ms on Android, seconds on
  // iOS) of packet i is pattern[i % n] instead of the median IMU dt (synthetic packets for the func 647 checks).
  'interval-pattern': {type: 'string', default: ''},
  // --zero-every N: packets with index % N == 1 carry exact zeros (accelerationIncludingGravity and rotationRate), the sign
  // behaviour of func 647 at zero (JSON cannot carry -0).
  // --save-frames: write every served frame as served-NNNNN.pgm (exact-input replay of the recording, orient_replay_main.cc).
  'save-frames': {type: 'boolean', default: false},
  'zero-every': {type: 'string', default: '0'},
  // --zero-value v: the number used instead of 0 (-1e-320 is a double that becomes -0.0f in the capnp float32 field; JSON cannot carry -0).
  'zero-value': {type: 'string', default: '0'},
}});
const scenarios = JSON.parse(await fs.readFile(path.join(laneC, 'scenarios.json'), 'utf8'));
const scenario = scenarios[values.scenario];
if (!scenario) throw new Error('--scenario one of ' + Object.keys(scenarios).join('|'));
if (!/^[a-z0-9-]+$/.test(values.label || '')) throw new Error('--label [a-z0-9-]+ required (new label per attempt)');
const orientation = Number(values.orientation);
if (![0, 90, 180, 270].includes(orientation)) throw new Error('--orientation 0|90|180|270');
const uaName = values.ua || scenario.ua || 'pixel7';
const iphone = IPHONES[uaName] || null;
if (!iphone && !UA[uaName]) throw new Error('--ua one of ' + [...Object.keys(UA), ...Object.keys(IPHONES)].join('|'));
const outRoot = path.resolve(root, values['out-root']);
const ownedFull = path.join(root, 'build/slam-refact-full-20261019');
if (!(outRoot === ownedFull || outRoot.startsWith(ownedFull + path.sep))) throw new Error('output must stay under ' + ownedFull);
const out = path.join(outRoot, values.scenario, values.label);
await fs.mkdir(path.dirname(out), {recursive: true});
await fs.mkdir(out, {recursive: false});
const sha = bytes => createHash('sha256').update(bytes).digest('hex');

async function loadDataset() {  // same rules as run-reference.mjs loadDataset()
  const dir = path.resolve(root, scenario.dataset);
  if (scenario.kind === 'tum') {
    const input = JSON.parse(await fs.readFile(path.join(dir, 'input.json')));
    return {dir, width: input.config.width, height: input.config.height,
      frames: input.frames.map(f => ({index: f.index, t: f.physicalTimestamp, file: path.join(dir, 'mav0/cam0/data', f.file), sha256: f.sha256})),
      imu: input.imu};
  }
  const record = JSON.parse(await fs.readFile(path.join(dir, 'record.json')));
  const rec = record.recording;
  const firstFrame = rec.frames[0].timestamp;
  const t0 = Math.max(...rec.imu.filter(r => r[0] <= firstFrame).map(r => r[0]));
  const lastFrame = rec.frames.at(-1).timestamp;
  return {dir, width: rec.width, height: rec.height,
    frames: rec.frames.map((f, i) => ({index: i, t: +(f.timestamp - t0).toFixed(9), file: path.join(dir, 'mav0/cam0/data', f.file), sha256: f.sha256})),
    imu: rec.imu.filter(r => r[0] >= t0 && r[0] <= lastFrame + 0.1).map(r => [+(r[0] - t0).toFixed(9), ...r.slice(1)])};
}

// P (wp x hp gray) -> the frame the page serves for `orientation` (see the header).
function turnFrame(p, wp, hp, a) {
  if (a === 0) return {pixels: p, width: wp, height: hp};
  if (a === 180) {
    const q = new Uint8Array(p.length);
    for (let y = 0; y < hp; y++) for (let x = 0; x < wp; x++) q[y * wp + x] = p[(hp - 1 - y) * wp + (wp - 1 - x)];
    return {pixels: q, width: wp, height: hp};
  }
  const q = new Uint8Array(p.length);  // landscape: width hp, height wp
  for (let y = 0; y < wp; y++) {
    for (let x = 0; x < hp; x++) {
      q[y * hp + x] = a === 90 ? p[x * wp + (wp - 1 - y)] : p[(hp - 1 - x) * wp + y];
    }
  }
  return {pixels: q, width: hp, height: wp};
}

const report = {schema: 'slam-refact-orient-original-run-v1', scenario: values.scenario, label: values.label, orientation, ua: uaName,
  argv: process.argv, startedAt: new Date().toISOString(), node: process.version, loadAvgAtStart: os.loadavg(),
  blockedExternal: [], pageErrors: [], pixelHashMismatches: [], console: []};
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
  const intervalPattern = values['interval-pattern'] ? values['interval-pattern'].split(',').map(Number) : [];
  if (intervalPattern.some(v => !Number.isFinite(v))) throw new Error('--interval-pattern numbers');
  const zeroEvery = Number(values['zero-every']);
  const zeroValue = Number(values['zero-value']);
  const sizeOf = (w, h) => (orientation === 90 || orientation === 270) ? {width: h, height: w} : {width: w, height: h};
  const served = sizeOf(data.width, data.height);
  const fixture = {config: {width: served.width, height: served.height}, nominalFps: Math.round(1 / frameDts[Math.floor(frameDts.length / 2)]),
    scale: scenario.scale || 'responsive', worldPointSampleEvery: Number(values['world-point-sample-every']),
    frames: frames.map(f => ({index: f.index, physicalTimestamp: f.t})),
    sensorPackets: sensors.packets.map((p, i) => {
      const q = {...p, intervalMs: intervalPattern.length ? intervalPattern[i % intervalPattern.length] : intervalMs};
      if (zeroEvery > 0 && i % zeroEvery === 1) { q.deviceAcceleration = [zeroValue, zeroValue, zeroValue]; q.rotationRate = {alpha: zeroValue, beta: zeroValue, gamma: zeroValue}; }
      return q;
    })};
  report.input = {dataset: data.dir, portraitWidth: data.width, portraitHeight: data.height, servedWidth: served.width, servedHeight: served.height,
    frames: frames.length, imuRows: imu.length, imuIntervalMsMedian: intervalMs, nominalFps: fixture.nominalFps, scale: fixture.scale};

  const stateScript = `window.__probe={ready:false,started:false,done:false,errors:[],events:[],rows:[],frames:[],imuDispatched:0,cameraRequests:[],intrinsicsChanges:[],worldPointSamples:[]};`;
  const html = `<!doctype html><html><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>orient</title><script src="/state.js"></script><script src="/page-runtime.js" defer></script><script src="/engine/xr.js" async data-preload-chunks="slam"></script></head><body style="margin:0"><button id="start" style="position:absolute">start</button><canvas id="view" width="${served.width}" height="${served.height}"></canvas></body></html>`;
  const runtimeBytes = await fs.readFile(path.join(laneC, 'page-runtime.js'));
  const probeBytes = await fs.readFile(path.join(here, 'orient-probe.js'));
  report.sourceHashes = {'page-runtime.js': sha(runtimeBytes), 'orient-probe.js': sha(probeBytes), 'run-orient.mjs': sha(await fs.readFile(fileURLToPath(import.meta.url))),
    'sensor_adapter.mjs': sha(await fs.readFile(path.join(root, 'scripts/dev/references/xr_reference/sensor_adapter.mjs')))};
  report.engineAssets = {};
  for (const name of ['xr.js', 'xr-slam.js']) { const b = await fs.readFile(path.join(engineDir, name)); report.engineAssets[name] = {bytes: b.length, sha256: sha(b)}; }
  const files = {'index.html': Buffer.from(html), 'state.js': Buffer.from(stateScript), 'page-runtime.js': runtimeBytes};
  const servedFrameHashes = new Map();  // index -> sha256 of the served (turned) frame
  const servedFrameBytes = new Map();   // index -> served pixels (written as PGM for the captured frames)
  let probeWasmBytes = null;
  if (values['probe-wasm']) {
    probeWasmBytes = await fs.readFile(values['probe-wasm']);
    report.probeWasm = {path: values['probe-wasm'], sha256: sha(probeWasmBytes), note: 'DIAGNOSTIC: instrumented build of the original (only added memory copies); the run is then not a byte-identical original run'};
  }
  server = http.createServer(async (req, res) => {
    try {
      const pathname = new URL(req.url, 'http://localhost').pathname;
      let bytes, type = 'application/octet-stream';
      if (pathname === '/fixture.json') { bytes = Buffer.from(JSON.stringify(fixture)); type = 'application/json'; }
      else if (/^\/gray\/\d+$/.test(pathname)) {
        const f = frames[Number(pathname.split('/').at(-1))];
        if (!f) throw new Error('unknown frame');
        const p = parsePGM(await fs.readFile(f.file), data.width, data.height);
        if (sha(p) !== f.sha256) report.pixelHashMismatches.push(f.index);
        const turned = turnFrame(p, data.width, data.height, orientation);
        bytes = Buffer.from(turned.pixels);
        servedFrameHashes.set(f.index, sha(bytes));
        if (Number(values['max-captures']) > 0 || values['save-frames']) servedFrameBytes.set(f.index, bytes);
      } else if (pathname === '/' || Object.hasOwn(files, pathname.slice(1))) {
        const name = pathname === '/' ? 'index.html' : pathname.slice(1);
        bytes = files[name]; type = name.endsWith('.html') ? 'text/html' : 'text/javascript';
      } else if (pathname === '/__probe/xr-probe.wasm' && probeWasmBytes) { bytes = probeWasmBytes; type = 'application/wasm'; }
      else if (pathname.startsWith('/engine/')) {
        const file = path.resolve(engineDir, pathname.slice(8));
        if (!file.startsWith(engineDir + path.sep)) throw new Error('unserved path');
        bytes = await fs.readFile(file); type = file.endsWith('.js') ? 'text/javascript' : 'application/octet-stream';
      } else throw new Error('unserved path');
      res.writeHead(200, {'Content-Type': type, 'Cache-Control': 'no-store'}); res.end(bytes);
    } catch (error) { res.writeHead(404); res.end(String(error)); }
  });
  await new Promise(resolve => server.listen(0, '127.0.0.1', resolve));
  const origin = `http://127.0.0.1:${server.address().port}`;
  report.origin = origin;
  const args = ['--enable-unsafe-swiftshader', '--autoplay-policy=no-user-gesture-required', '--use-fake-ui-for-media-stream', '--use-fake-device-for-media-stream'];
  browser = await chromium.launch({executablePath: process.env.BROWSER_EXECUTABLE || '/usr/bin/google-chrome', headless: true, args});
  report.browserVersion = browser.version();
  const ctxOptions = iphone
    ? {viewport: sizeOf(iphone.w, iphone.h), screen: {width: iphone.w, height: iphone.h}, deviceScaleFactor: iphone.dpr, userAgent: IPHONE_UA}
    : {viewport: served, deviceScaleFactor: 1, userAgent: UA[uaName]};
  const contextOptions = {...ctxOptions, permissions: ['camera', 'microphone'], isMobile: true, hasTouch: true};
  if (iphone && (orientation === 90 || orientation === 270)) contextOptions.viewport = {width: iphone.h, height: iphone.w};
  report.browserContext = contextOptions;
  const context = await browser.newContext(contextOptions);
  const page = await context.newPage();
  const windowOrientation = {0: 0, 90: 90, 180: 180, 270: -90}[orientation];
  await page.addInitScript(`${probeBytes}\nwindow.__orientInit(${JSON.stringify({targetSha256: XR_SLAM_WASM_SHA256, maxCaptures: Number(values['max-captures']),
    rows: Number(values.rows), fromStage: Number(values['from-stage']), probeWasmUrl: probeWasmBytes ? '/__probe/xr-probe.wasm' : null,
    screenAngle: orientation === 0 && !iphone ? null : orientation, windowOrientation, platform: iphone ? 'iPhone' : null})});`);
  await page.route('**/*', route => {
    const url = route.request().url();
    if (!url.startsWith(origin + '/') && !url.startsWith('data:') && !url.startsWith('blob:')) { report.blockedExternal.push(url); return route.abort('blockedbyclient'); }
    return route.continue();
  });
  page.on('console', m => { if (report.console.length < 200) report.console.push({type: m.type(), text: m.text().slice(0, 300)}); });
  page.on('pageerror', e => report.pageErrors.push({message: e.message}));
  await page.goto(origin + '/index.html');
  await page.waitForFunction(() => window.__probe?.ready, null, {timeout: 60000});
  report.publicApi = await page.evaluate(() => ({version: XR8.version(), deviceEstimate: XR8.XrDevice.deviceEstimate(),
    screen: {w: screen.width, h: screen.height, orientationType: screen.orientation?.type, orientationAngle: screen.orientation?.angle,
      windowOrientation: window.orientation, dpr: devicePixelRatio, innerWidth, innerHeight}, userAgent: navigator.userAgent, platform: navigator.platform}));
  await page.locator('#start').click();
  try { await page.waitForFunction(() => window.__probe?.done, null, {timeout: Math.round(frames.at(-1).t * 1000) + 120000, polling: 250}); }
  catch (error) { report.waitError = String(error); }
  const probe = await page.evaluate(() => window.__orientCollect());
  const cap = await page.evaluate(() => {
    const C = window.__orient;
    return {wrapped: C.wrapped, error: C.error, glInfo: C.glInfo, calls: C.calls, reads: C.reads, ncaptures: C.captures.length, probeWasm: C.probeWasm,
      probe647: C.probe647};
  });
  for (let i = 0; i < cap.ncaptures; i++) {
    const b64 = await page.evaluate(i => {
      const a = window.__orient.captures[i];
      let s = '';
      for (let o = 0; o < a.length; o += 0x8000) s += String.fromCharCode.apply(null, a.subarray(o, o + 0x8000));
      return btoa(s);
    }, i);
    const raw = Buffer.from(b64, 'base64');
    await fs.writeFile(path.join(out, `cap-${String(i).padStart(3, '0')}.rgba.gz`), zlib.gzipSync(raw, {level: 6, mtime: 0}));
    cap.reads.find(r => r.capture === i).rawSha256 = sha(raw);
  }
  try { await page.evaluate(() => window.__finishProbe?.()); } catch (error) { report.cleanupError = String(error); }
  const byUs = new Map(probe.frames.map(([index, us]) => [us, index]));
  const frameOf = v => { const us = Math.round(v * 1e6); return byUs.get(us) ?? byUs.get(us - 1) ?? byUs.get(us + 1) ?? null; };
  const stageCalls = cap.calls.filter(c => c[1] === 'S');
  const captures = [];
  for (const ev of cap.reads.filter(r => r.capture !== null && r.capture !== undefined)) {
    const k = stageCalls.findIndex(s => s[0] === ev.call);
    const prev = k > 0 ? stageCalls[k - 1] : null;
    captures.push({capture: ev.capture, readInCall: ev.call, readInCallName: ev.callName, readKind: ev.kind, pyramidDrawnInStageCall: prev ? prev[0] : null,
      inputFrame: prev ? frameOf(prev[3][11]) : null, stageArgs: prev ? prev[3] : null, rawSha256: ev.rawSha256});
  }
  const wanted = values['save-frames'] ? [...servedFrameBytes.keys()] : captures.filter(c => c.inputFrame !== null).map(c => c.inputFrame);
  for (const index of new Set(wanted)) {
    const bytes = servedFrameBytes.get(index);
    if (!bytes) continue;
    await fs.writeFile(path.join(out, `served-${String(index).padStart(5, '0')}.pgm`),
      Buffer.concat([Buffer.from(`P5\n${served.width} ${served.height}\n255\n`), bytes]));
  }
  const epoch = probe.epochWallMs;
  const fresh = new Map();
  for (const r of probe.rows) {
    const idx = frameOf(r[1]);
    if (!r[2] && idx !== null && !fresh.has(idx)) fresh.set(idx, r);
  }
  const traceLines = frames.map(f => {
    const r = fresh.get(f.index);
    const ok = r && r[3] !== null && [5, 6, 7, 8, 9, 10, 11].every(k => Number.isFinite(r[k]));
    return JSON.stringify({i: f.index, t: f.t, fresh: !!r, callbackMs: r ? +(r[0] - epoch).toFixed(3) : null, status: r ? r[3] : null,
      reason: r ? r[4] : null, p: ok ? [r[5], r[6], r[7]] : null, q_wxyz: ok ? [r[8], r[9], r[10], r[11]] : null, worldPoints: r ? r[12] : null,
      textureWidth: r ? r[19] : null, textureHeight: r ? r[20] : null, orientation: r ? r[21] : null});
  });
  await fs.writeFile(path.join(out, 'trace.jsonl'), traceLines.join('\n') + '\n');
  await fs.writeFile(path.join(out, 'callbacks-raw.json'), JSON.stringify(probe.rows));
  await fs.writeFile(path.join(out, 'calls.json'), JSON.stringify(cap.calls));
  await fs.writeFile(path.join(out, 'intrinsics.json'), intrinsicsFile(probe.intrinsicsChanges));
  await fs.writeFile(path.join(out, 'frames-served.json'), JSON.stringify([...servedFrameHashes.entries()].sort((a, b) => a[0] - b[0])));
  await fs.writeFile(path.join(out, 'packets.json'), JSON.stringify(fixture.sensorPackets));
  await fs.writeFile(path.join(out, 'published-frames.json'), JSON.stringify(probe.frames));
  if (cap.probe647) await fs.writeFile(path.join(out, 'probe647.json'), probeFile(cap.probe647));
  Object.assign(report, {wrapped: cap.wrapped, wrapError: cap.error, glInfo: cap.glInfo, probeWasmLoaded: cap.probeWasm, captures,
    reads: cap.reads.map(r => ({...r, rawSha256: undefined})), startEvents: probe.events.filter(e => e.name === 'onStart'),
    deviceEstimate: report.publicApi.deviceEstimate, cameraRequests: probe.cameraRequests, pageErrorsProbe: probe.errors, imuDispatched: probe.imuDispatched,
    freshFrames: fresh.size,
    statusCounts: Object.fromEntries([...fresh.values()].reduce((m, r) => m.set(r[3], (m.get(r[3]) || 0) + 1), new Map())),
    engineInitCalls: cap.calls.filter(c => ['I', 'O', 'V'].includes(c[1])).map(c => [c[1], c[3]]),
    intrinsicsChanges: probe.intrinsicsChanges.length});
  report.status = cap.wrapped && fresh.size ? 'COMPLETED' : 'INCOMPLETE';
} catch (error) {
  report.status = 'FAILED'; report.error = {message: error.message, stack: error.stack};
} finally {
  if (browser) await browser.close();
  if (server) await new Promise(resolve => server.close(resolve));
  report.finishedAt = new Date().toISOString();
  report.loadAvgAtEnd = os.loadavg();
  await fs.writeFile(path.join(out, 'meta.json'), JSON.stringify(report, null, 1) + '\n');
  console.log(JSON.stringify({status: report.status, out: path.relative(root, out), fresh: report.freshFrames, statusCounts: report.statusCounts,
    deviceEstimate: report.deviceEstimate, engineInit: report.engineInitCalls, intrinsics: report.intrinsicsChanges, error: report.error?.message, waitError: report.waitError}, null, 1));
  process.exitCode = report.status === 'COMPLETED' ? 0 : 1;
}

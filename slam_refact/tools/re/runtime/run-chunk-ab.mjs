// Full-parity PRD US-007..US-009 (spec docs/spec/07-xr-slam-chunk.md): run the deployed reference runtime xr.js headless with
// either the ORIGINAL xr-slam.js chunk or the slam_refact chunk (slam_refact/web-chunk/xr-slam.js) on the same recorded
// input, through the same page driver (page-runtime.js, unmodified) as run-reference.mjs, and write the same trace files.
// Chunks run one after another (one headless Chromium at a time); each gets its own ephemeral 127.0.0.1 server.
//
//   node slam_refact/tools/re/runtime/run-chunk-ab.mjs --scenario flip6-s2 --label ab1 [--chunks original,refact]
//     [--max-frames N] [--gpu-check 10,30] [--api-oracle '{"configure":{"enableLighting":true}}']
//     [--engine-dir build/slam-refact-full-chunk-wasm/wasm]
//
// Output: build/slam-refact-full-20261019/chunk/runs/<scenario>/<label>/<chunk>/{meta.json, trace.jsonl, callbacks.jsonl,
//   events.json, api-oracle.json.gz, chunk-diag.json (refact), screenshot.png}. Existing labels are never overwritten.
// Nothing is written under web/; the deployed SDK files are served read-only from their reference directory.
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

const here = path.dirname(fileURLToPath(import.meta.url));
const root = path.resolve(here, '../../../..');
const sdkDir = path.join(root, 'assets/references/xr_reference/analysis-runtime/standalone');
const chunkDir = path.join(root, 'slam_refact/web-chunk');
const SDK_SHA256 = {'xr.js': '1e4c00ab32d0eef25110891ef2aba6ad41cac76fac5e27f1361fd694a5c05563',
  'xr-slam.js': 'd5b059d8676bfccdb9c317881fa640bcaddefd95a8c5337c32965ec220cf1e85'};
const CHUNK_FILES = ['xr-slam.js', 'engine-loader.js', 'gr8-compute-pyramid.js', 'gr8-shaders.js', 'image-events.js', 'image-targets.js', 'sensors.js'];
const UA = {
  pixel7: 'Mozilla/5.0 (Linux; Android 13; Pixel 7) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/140.0.0.0 Mobile Safari/537.36',
  reduced: 'Mozilla/5.0 (Linux; Android 10; K) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/140.0.0.0 Mobile Safari/537.36',
};

const {values} = parseArgs({options: {
  scenario: {type: 'string'},
  label: {type: 'string'},
  chunks: {type: 'string', default: 'original,refact'},
  'out-root': {type: 'string', default: 'build/slam-refact-full-20261019/chunk/runs'},
  'engine-dir': {type: 'string', default: 'build/slam-refact-full-chunk-wasm/wasm'},
  'max-frames': {type: 'string', default: '0'},
  'gpu-check': {type: 'string'},
  'api-oracle': {type: 'string'},
  'world-point-sample-every': {type: 'string', default: '50'},
}});
const scenarios = JSON.parse(await fs.readFile(path.join(here, 'scenarios.json'), 'utf8'));
const scenario = scenarios[values.scenario];
if (!scenario) throw new Error('--scenario one of ' + Object.keys(scenarios).join('|'));
if (!/^[a-z0-9-]+$/.test(values.label || '')) throw new Error('--label [a-z0-9-]+ required (new label per attempt)');
const chunks = values.chunks.split(',');
if (!chunks.length || chunks.some(c => !['original', 'refact'].includes(c))) throw new Error('--chunks: original,refact');
const outRoot = path.resolve(root, values['out-root']);
const owned = path.join(root, 'build/slam-refact-full-20261019/chunk');
if (!(outRoot === owned || outRoot.startsWith(owned + path.sep))) throw new Error('output must stay under ' + owned);
const engineDir = path.resolve(root, values['engine-dir']);
if (!engineDir.startsWith(path.join(root, 'build') + path.sep)) throw new Error('--engine-dir must be under build/');
const gpuCheck = values['gpu-check'] ? (([every, frames]) => ({every: Number(every), frames: Number(frames)}))(values['gpu-check'].split(',')) : null;
const apiOracle = values['api-oracle'] ? JSON.parse(values['api-oracle']) : null;
const labelDir = path.join(outRoot, values.scenario, values.label);
await fs.mkdir(path.dirname(labelDir), {recursive: true});
await fs.mkdir(labelDir, {recursive: false});
const sha = bytes => createHash('sha256').update(bytes).digest('hex');
const stats = xs => {
  const s = xs.filter(Number.isFinite).sort((a, b) => a - b);
  const q = p => s.length ? s[Math.min(s.length - 1, Math.floor((s.length - 1) * p))] : null;
  return {n: s.length, min: s[0] ?? null, p50: q(0.5), p95: q(0.95), p99: q(0.99), max: s.at(-1) ?? null,
    mean: s.length ? s.reduce((a, b) => a + b, 0) / s.length : null};
};

// Dataset loading: same rules as run-reference.mjs loadDataset (tum / flip6 fixtures).
async function loadDataset() {
  const dir = path.resolve(root, scenario.dataset);
  if (scenario.kind === 'tum') {
    const bytes = await fs.readFile(path.join(dir, 'input.json'));
    const input = JSON.parse(bytes);
    if (input.schema !== 'mobile-slam-synthetic-vio-v1') throw new Error('unexpected fixture schema');
    return {dir, sourceSha256: sha(bytes), width: input.config.width, height: input.config.height,
      frames: input.frames.map(f => ({index: f.index, t: f.physicalTimestamp, file: path.join(dir, 'mav0/cam0/data', f.file), sha256: f.sha256})),
      imu: input.imu};
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
    return {dir, sourceSha256: sha(bytes), width: rec.width, height: rec.height,
      frames: rec.frames.map((f, i) => ({index: i, t: +(f.timestamp - t0).toFixed(9), file: path.join(dir, 'mav0/cam0/data', f.file), sha256: f.sha256})),
      imu: rec.imu.filter(r => r[0] >= t0 && r[0] <= lastFrame + 0.1).map(r => [+(r[0] - t0).toFixed(9), ...r.slice(1)])};
  }
  throw new Error('unknown scenario kind');
}

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
const fixtureBytes = Buffer.from(JSON.stringify(fixture));

async function runChunk(chunk) {
  const out = path.join(labelDir, chunk);
  await fs.mkdir(out, {recursive: false});
  const report = {schema: 'slam-refact-chunk-ab-run-v1', scenario: values.scenario, label: values.label, chunk,
    startedAt: new Date().toISOString(), argv: process.argv, node: process.version, loadAvgAtStart: os.loadavg(),
    input: {dataset: path.relative(root, data.dir), sourceSha256: data.sourceSha256, frames: frames.length, imuRows: imu.length,
      fixtureSha256: sha(fixtureBytes), nominalFps, intervalMs},
    requests: [], blockedExternal: [], console: [], pageErrors: [], requestFailures: [], pixelHashMismatches: []};
  let server, browser;
  try {
    const stateScript = `window.__probe={ready:false,started:false,done:false,errors:[],events:[],rows:[],frames:[],imuDispatched:0,cameraRequests:[],intrinsicsChanges:[],worldPointSamples:[]};
window.addEventListener('error',e=>window.__probe.errors.push({kind:'window',message:e.message,stack:e.error?.stack}));
window.addEventListener('unhandledrejection',e=>window.__probe.errors.push({kind:'promise',message:String(e.reason),stack:e.reason?.stack}));`;
    const html = `<!doctype html><html><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>reference runtime chunk A/B run</title><script src="/state.js"></script><script src="/page-runtime.js" defer></script><script src="/engine/xr.js" async data-preload-chunks="slam"></script></head><body style="margin:0"><button id="start" style="position:absolute">start</button><canvas id="view" width="${data.width}" height="${data.height}"></canvas></body></html>`;
    const served = {'index.html': Buffer.from(html), 'state.js': Buffer.from(stateScript),
      'page-runtime.js': await fs.readFile(path.join(here, 'page-runtime.js'))};
    report.sourceHashes = {'run-chunk-ab.mjs': sha(await fs.readFile(fileURLToPath(import.meta.url))),
      'page-runtime.js': sha(served['page-runtime.js']), 'api-oracle.js': sha(await fs.readFile(path.join(here, 'api-oracle.js')))};
    for (const name of ['xr.js', 'xr-slam.js']) {
      const b = await fs.readFile(path.join(sdkDir, name));
      if (sha(b) !== SDK_SHA256[name]) throw new Error(`SDK ${name} sha256 differs from the pinned value`);
    }
    report.served = {'xr.js': {sha256: SDK_SHA256['xr.js'], from: path.relative(root, path.join(sdkDir, 'xr.js'))}};
    if (chunk === 'original') report.served['xr-slam.js'] = {sha256: SDK_SHA256['xr-slam.js'], from: path.relative(root, path.join(sdkDir, 'xr-slam.js'))};
    else {
      for (const f of CHUNK_FILES) report.served[f] = {sha256: sha(await fs.readFile(path.join(chunkDir, f))), from: path.relative(root, path.join(chunkDir, f))};
      const manifest = JSON.parse(await fs.readFile(path.join(engineDir, 'manifest.json'), 'utf8'));
      report.served.engineManifest = {from: path.relative(root, path.join(engineDir, 'manifest.json')), manifest};
    }
    const resolveEngine = (rel) => {
      if (chunk === 'refact') {
        if (CHUNK_FILES.includes(rel)) return path.join(chunkDir, rel);
        if (rel.startsWith('slam-refact-engine/')) {
          const name = rel.slice('slam-refact-engine/'.length);
          if (['manifest.json', 'slam_refact_engine.js', 'slam_refact_engine.wasm'].includes(name)) return path.join(engineDir, name);
          throw new Error('unserved engine path');
        }
      }
      const file = path.resolve(sdkDir, rel);
      if (!file.startsWith(sdkDir + path.sep)) throw new Error('unserved path');
      return file;
    };
    const serve = async (req, res) => {
      try {
        const pathname = new URL(req.url, 'http://localhost').pathname;
        let bytes, type = 'application/octet-stream';
        if (pathname === '/fixture.json') { bytes = fixtureBytes; type = 'application/json'; }
        else if (/^\/gray\/\d+$/.test(pathname)) {
          const f = frames[Number(pathname.split('/').at(-1))];
          if (!f) throw new Error('unknown frame');
          bytes = parsePGM(await fs.readFile(f.file), data.width, data.height);
          if (sha(bytes) !== f.sha256) report.pixelHashMismatches.push(f.index);
        } else if (pathname === '/' || Object.hasOwn(served, pathname.slice(1))) {
          const name = pathname === '/' ? 'index.html' : pathname.slice(1);
          bytes = served[name]; type = name.endsWith('.html') ? 'text/html' : 'text/javascript';
        } else if (pathname.startsWith('/engine/')) {
          const file = resolveEngine(pathname.slice(8));
          bytes = await fs.readFile(file);
          type = file.endsWith('.js') ? 'text/javascript' : file.endsWith('.json') ? 'application/json' : file.endsWith('.wasm') ? 'application/wasm' : 'application/octet-stream';
        } else throw new Error('unserved path');
        res.writeHead(200, {'Content-Type': type, 'Cache-Control': 'no-store'}); res.end(bytes);
      } catch (error) { res.writeHead(404); res.end(String(error)); }
    };
    server = http.createServer((req, res) => void serve(req, res));
    await new Promise(resolve => server.listen(0, '127.0.0.1', resolve));
    const origin = `http://127.0.0.1:${server.address().port}`;
    report.origin = origin;
    const launch = {executablePath: process.env.BROWSER_EXECUTABLE || '/usr/bin/google-chrome', headless: true,
      args: ['--enable-unsafe-swiftshader', '--autoplay-policy=no-user-gesture-required', '--use-fake-ui-for-media-stream', '--use-fake-device-for-media-stream']};
    report.browserLaunch = launch;
    browser = await chromium.launch(launch);
    report.browserVersion = browser.version();
    const viewport = scenario.viewport ? {width: scenario.viewport[0], height: scenario.viewport[1]} : {width: data.width, height: data.height};
    const context = await browser.newContext({viewport, permissions: ['camera', 'microphone'], isMobile: true, hasTouch: true,
      deviceScaleFactor: 1, userAgent: UA[scenario.ua || 'pixel7']});
    const page = await context.newPage();
    if (chunk === 'refact' && gpuCheck) await page.addInitScript(`window.SLAM_REFACT_CHUNK_OPTIONS = ${JSON.stringify({gpuCheck})};`);
    if (apiOracle) {
      const oracleSource = await fs.readFile(path.join(here, 'api-oracle.js'), 'utf8');
      await page.addInitScript(`${oracleSource}\nwindow.__apiOracleInit(${JSON.stringify(apiOracle)});`);
      report.apiOracle = {options: apiOracle};
    }
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
      xrControllerKeys: Object.keys(XR8.XrController).sort(), pipelineModuleKeys: Object.keys(XR8.XrController.pipelineModule()).sort()}));
    const wallStart = Date.now();
    await page.locator('#start').click();
    const lastT = frames.at(-1).t;
    try { await page.waitForFunction(() => window.__probe?.done, null, {timeout: Math.round(lastT * 1000) + 120000, polling: 500}); }
    catch (error) { report.waitError = String(error); }
    report.wallSeconds = (Date.now() - wallStart) / 1000;
    const oracle = apiOracle ? await page.evaluate(() => window.__apiOracle) : null;
    const chunkDiag = chunk === 'refact' ? await page.evaluate(() => window.__slamRefactChunk ?? null) : null;
    try { await page.evaluate(() => window.__finishProbe?.()); } catch (error) { report.cleanupError = String(error); }
    const observed = await page.evaluate(() => window.__probe);
    await page.screenshot({path: path.join(out, 'screenshot.png')});

    // Trace assembly (same row schema as run-reference.mjs; association by published videoTime, us +-1).
    const epoch = observed.epochWallMs;
    const byUs = new Map(observed.frames.map(([index, us, published, accepted]) => [us, {index, us, published, accepted}]));
    const callbackKeys = ['wallMs', 'videoTime', 'repeat', 'status', 'reason', 'px', 'py', 'pz', 'qw', 'qx', 'qy', 'qz', 'worldPoints', 'wpAdded', 'wpRemoved',
      'wpMeanConfidence', 'gpuMs', 'cpuMs', 'frameTime', 'textureWidth', 'textureHeight', 'orientation'];
    const cbStream = createWriteStream(path.join(out, 'callbacks.jsonl'));
    const fresh = new Map();
    const callbacks = [];
    for (const row of observed.rows) {
      const obj = Object.fromEntries(callbackKeys.map((k, i) => [k, row[i] ?? null]));
      const us = Math.round(obj.videoTime * 1e6);
      const match = byUs.get(us) || byUs.get(us - 1) || byUs.get(us + 1);
      obj.frameIndex = match ? match.index : null;
      cbStream.write(JSON.stringify(obj) + '\n');
      callbacks.push(obj);
      if (!obj.repeat && match && !fresh.has(match.index)) fresh.set(match.index, obj);
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
    if (oracle) {
      const frameOf = v => { const us = Math.round(v * 1e6); return byUs.get(us)?.index ?? byUs.get(us - 1)?.index ?? byUs.get(us + 1)?.index ?? null; };
      for (const row of oracle.lighting) row.frameIndex = frameOf(row.videoTime);
      for (const row of oracle.hits) row.frameIndex = frameOf(row.videoTime);
      await fs.writeFile(path.join(out, 'api-oracle.json.gz'), zlib.gzipSync(JSON.stringify({schema: 'slam-refact-xr_reference-api-oracle-v1',
        scenario: values.scenario, label: values.label, chunk, ...oracle}), {level: 9, mtime: 0}));
      report.apiOracle = {...report.apiOracle, file: 'api-oracle.json.gz', installed: oracle.installed, error: oracle.error,
        lightingRows: oracle.lighting.length, hitSamples: oracle.hits.length};
    }
    if (chunkDiag) {
      await fs.writeFile(path.join(out, 'chunk-diag.json'), JSON.stringify(chunkDiag) + '\n');
      const st = chunkDiag.stats;
      report.chunkDiag = {file: 'chunk-diag.json', engine: chunkDiag.engine, error: chunkDiag.error, session: chunkDiag.session, warnings: chunkDiag.warnings,
        stagedFrames: st.stagedFrames, processedFrames: st.processedFrames, freshResponses: st.freshResponses, droppedStaged: st.droppedStaged,
        motionEventsPushed: st.motionEventsPushed, motionEventsSkipped: st.motionEventsSkipped, responseLag: st.responseLag,
        sizeMismatchFrames: st.sizeMismatchFrames, inactiveAnswers: st.inactiveAnswers, gapAnswers: st.gapAnswers, contextLostFrames: st.contextLostFrames,
        sessionsCreated: st.sessionsCreated, limits: chunkDiag.limits,
        stageMs: stats(st.stageMs), readMs: stats(st.readMs), engineMs: stats(st.engineMs),
        gpuCheck: {frames: chunkDiag.gpuCheck.length, swiftshaderApproxIdentical: chunkDiag.gpuCheck.filter(g => g.swiftshaderApprox.identical).length,
          ieeeIdentical: chunkDiag.gpuCheck.filter(g => g.ieee.identical).length}};
    }
    const freshRows = traceRows.filter(r => r.fresh);
    const statusCounts = {};
    for (const r of traceRows) { const k = r.fresh ? `${r.status}/${r.reason}` : 'NO_FRESH_CALLBACK'; statusCounts[k] = (statusCounts[k] || 0) + 1; }
    report.summary = {inputFrames: frames.length, publishedFrames: observed.frames.length, freshCallbacks: freshRows.length,
      totalCallbacks: observed.rows.length, nonNullReality: callbacks.filter(c => c.status !== null).length, imuDispatched: observed.imuDispatched,
      statusCounts, normalRatio: traceRows.filter(r => r.status === 'NORMAL').length / frames.length,
      onStart: observed.events.find(e => e.name === 'onStart') || null,
      trackingstatusEvents: observed.events.filter(e => e.name === 'reality.trackingstatus').length,
      publishToCallbackMs: stats(freshRows.map(r => r.latencyMs)), xrCpuStageMs: stats(freshRows.map(r => r.xrCpuStageMs)),
      xrGpuStageMs: stats(freshRows.map(r => r.xrGpuStageMs)), worldPoints: stats(freshRows.map(r => r.worldPoints)),
      errors: observed.errors.length, pageErrors: report.pageErrors.length, blockedExternal: report.blockedExternal.length,
      chunkRequested: report.requests.some(r => r.url === '/engine/xr-slam.js' && r.status === 200),
      resourceCleanup: observed.ownedResourcesClosed === true};
    report.status = report.pageErrors.length || observed.errors.length ? 'COMPLETED_WITH_ERRORS' : freshRows.length ? 'COMPLETED' : 'NO_FRESH_CALLBACKS';
  } catch (error) {
    report.status = 'FAILED'; report.error = {message: error.message, stack: error.stack};
  } finally {
    if (browser) { await browser.close(); report.browserClosed = true; }
    if (server) { await new Promise(resolve => server.close(resolve)); report.serverClosed = true; }
    report.finishedAt = new Date().toISOString();
    await fs.writeFile(path.join(out, 'meta.json'), JSON.stringify(report, null, 1) + '\n');
  }
  const s = report.summary || {};
  console.log(JSON.stringify({chunk, status: report.status, out: path.relative(root, out), fresh: s.freshCallbacks, statusCounts: s.statusCounts,
    onStart: !!s.onStart, error: report.error?.message, waitError: report.waitError, chunkDiag: report.chunkDiag?.gpuCheck}, null, 1));
  return report.status;
}

const statuses = {};
for (const chunk of chunks) statuses[chunk] = await runChunk(chunk);
process.exitCode = Object.values(statuses).some(s => s === 'FAILED') ? 1 : 0;

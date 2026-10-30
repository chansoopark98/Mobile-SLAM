#!/usr/bin/env node
// Headless smoke of the live measurement page web/slam-refact.html (desktop Chromium, NOT a phone).
//
// Serves web/ through the production request handler (web/server.js createApp, so the allowlist and
// COOP/COEP headers are the real ones) over plain HTTP on a one-off 127.0.0.1 server with a random
// port, started and stopped here (the user's HTTPS server is not used). Chromium gets a fake camera
// from a Y4M made of Flip6 replay frames (480x640 gray, played at 30 fps, looped) and synthetic
// DeviceMotion/DeviceOrientation events dispatched in the page from the same replay bundle's sensor
// rows. Checks: nothing runs before Start, the worker verifies the engine bytes against the
// manifest, frames are processed, no console/page errors, no non-GET or external requests, and the
// metrics JSON is downloadable and carries no image or raw IMU data. Writes
// build/slam-refact-web-20261017/live-smoke/<label>/{result.json,metrics.json} (never overwritten).
//
//   node slam_refact/web-smoke/run-live-smoke.mjs --label <[a-z0-9-]+> [--seconds 10] [--frames 300]
//        [--bundle <prepare_inputs bundle dir>] [--query '?gpu-check=1'] [--engine-dir <wasm build dir>]
//        [--rec-out <dir under build/>]
//
// With --query '?record=1' (the opt-in input recorder) the smoke also: clicks "녹화 다운로드" (two
// downloads), pulls the same two OPFS files through window.__slamRefactRecording into --rec-out
// (default build/slam-refact-rec-20261019/<label>), checks both copies are byte-identical, the meta
// sha256 and frame counts, and reports the recorder's main-thread cost. OPFS (origin-private, on
// this machine) is the ONLY storage the recorder uses; the default run (no ?record=1) checks that
// nothing was recorded and that no recording API exists.
//
// --engine-dir serves a candidate engine (slam_refact_engine.{js,wasm} of a build dir) and a
// manifest made from its bytes in place of web/slam-refact/ through request interception only;
// web/ is never written (deployment stays slam_refact/tools/deploy_web.py with authorization).
import crypto from 'node:crypto';
import fs from 'node:fs';
import http from 'node:http';
import path from 'node:path';
import { createRequire } from 'node:module';
import { readBundle, readPgm } from '../engine/tests/wasm/bundle_io.mjs';
import { worldPointDisplayEvidence } from './world-point-display.mjs';

const here = import.meta.dirname;
const repo = path.resolve(here, '../..');
const args = Object.fromEntries(process.argv.slice(2).reduce((acc, a, i, all) =>
  (a.startsWith('--') ? [...acc, [a.slice(2), all[i + 1]]] : acc), []));
if (!args.label || !/^[a-z0-9-]+$/.test(args.label)) throw new Error('--label <[a-z0-9-]+> required');
const seconds = Number(args.seconds ?? 10);
const maxFrames = Number(args.frames ?? 300);
const bundleDir = args.bundle ?? path.join(repo, 'build/slam-refact-20261015/harness/bundles/flip6-s2/wp5b-r2');
const outDir = path.join(repo, 'build/slam-refact-web-20261017/live-smoke', args.label);
const recordOn = /[?&]record=1(?:&|$)/.test(args.query ?? '');
const recOut = path.resolve(args['rec-out'] ?? path.join(repo, 'build/slam-refact-rec-20261019', args.label));
if (!(recOut + path.sep).startsWith(path.join(repo, 'build') + path.sep)) throw new Error('--rec-out must be under build/');
if (recordOn && fs.existsSync(recOut)) throw new Error(`refusing to overwrite ${recOut}`);
// --fatal-recovery N: the first worker throws after N frames (?debug-fatal-after); the smoke then
// checks the error state, presses Start again and runs the normal checks on the second run.
const fatalAfter = args['fatal-recovery'] ? Number(args['fatal-recovery']) : 0;
if (args['fatal-recovery'] && !(Number.isInteger(fatalAfter) && fatalAfter > 0)) throw new Error('--fatal-recovery <frames > 0>');
const baseQuery = args.query ?? '';
if (baseQuery && !/^\?[A-Za-z0-9=&_.-]*$/.test(baseQuery)) throw new Error('--query must look like ?a=b&c=d');
const query = fatalAfter ? `${baseQuery ? `${baseQuery}&` : '?'}debug-fatal-after=${fatalAfter}` : baseQuery;
let candidate = null;
if (args['engine-dir']) {
  const dir = path.resolve(args['engine-dir']);
  const js = fs.readFileSync(path.join(dir, 'slam_refact_engine.js'));
  const wasm = fs.readFileSync(path.join(dir, 'slam_refact_engine.wasm'));
  const sha = (b) => crypto.createHash('sha256').update(b).digest('hex');
  const deployed = JSON.parse(fs.readFileSync(path.join(repo, 'web/slam-refact/manifest.json'), 'utf8'));
  candidate = { dir, js, wasm, manifest: Buffer.from(JSON.stringify({ ...deployed, schema: 'slam-refact-web-candidate-v1',
    git: { ...deployed.git, head: 'candidate-not-deployed', engine_dirty: true },
    artifacts: { js: { file: 'slam_refact_engine.js', sha256: sha(js), bytes: js.length },
      wasm: { file: 'slam_refact_engine.wasm', sha256: sha(wasm), bytes: wasm.length } },
    build: { dir: path.relative(repo, dir) }, deployed_at: 'not deployed (smoke route)' })) };
}
if (fs.existsSync(outDir)) throw new Error(`refusing to overwrite ${outDir}`);
fs.mkdirSync(outDir, { recursive: true });

// Fake camera: the 480x640 portrait PGM frames rotated 90 degrees counter-clockwise into a 640x480
// landscape Y plane (U = V = 128, C420, 30 fps). The page requests 640x480, so Chrome's fake device
// delivers these frames unscaled, and the page's default clockwise rotation restores the original
// portrait frame exactly (a portrait Y4M would be center-cropped by Chrome to 480x480).
const bundle = readBundle(bundleDir);
const framePaths = bundle.frames.filter((f) => !f.bootstrap).slice(0, maxFrames).map((f) => f.image);
const first = readPgm(framePaths[0]);
const { width: PW, height: PH } = first;
const [W, H] = [PH, PW];
const y4m = path.join(outDir, 'fake-camera.y4m');
const fd = fs.openSync(y4m, 'w');
fs.writeSync(fd, `YUV4MPEG2 W${W} H${H} F30:1 Ip A1:1 C420jpeg\n`);
const chroma = Buffer.alloc((W / 2) * (H / 2) * 2, 128);
const luma = Buffer.alloc(W * H);
for (const p of framePaths) {
  const img = readPgm(p);
  if (img.width !== PW || img.height !== PH) throw new Error(`frame size mismatch ${p}`);
  for (let i = 0; i < H; i++) for (let j = 0; j < W; j++) luma[i * W + j] = img.pixels[j * PW + (PW - 1 - i)];
  fs.writeSync(fd, 'FRAME\n'); fs.writeSync(fd, luma); fs.writeSync(fd, chroma);
}
fs.closeSync(fd);
const sensorRows = bundle.sensors.filter((s) => !s.bootstrap).map((s) => ({
  dtMs: 0, t: s.t, aig: s.aig, lin: s.lin, rate: s.rate, intervalMs: s.intervalMs, orient: s.orient }));
for (let i = 1; i < sensorRows.length; i++) sensorRows[i].dtMs = Math.max(1, (sensorRows[i].t - sensorRows[i - 1].t) / 1e6);

// Production handler (allowlist + headers), plain HTTP on loopback.
const require = createRequire(path.join(repo, 'scripts/dev/package.json'));
const { createApp } = require(path.join(repo, 'web/server.js'));
const app = createApp({ webRoot: path.join(repo, 'web'), hostname: '127.0.0.1' });
const served = [];
const server = http.createServer((req, res) => { served.push(`${req.method} ${req.url}`); app(req, res); });
await new Promise((r) => server.listen(0, '127.0.0.1', r));
const origin = `http://127.0.0.1:${server.address().port}`;

const { chromium } = require('playwright');
const browser = await chromium.launch({ headless: true, args: ['--enable-unsafe-swiftshader', '--use-fake-ui-for-media-stream',
  '--use-fake-device-for-media-stream', `--use-file-for-fake-video-capture=${y4m}`] });
const blocked = [], nonGet = [], consoleErrors = [], pageErrors = [];
let result;
try {
  const context = await browser.newContext({ acceptDownloads: true, viewport: { width: 412, height: 915 },
    // Reduced Chrome-on-Android UA of the Flip6 (model "K", as in the Flip6 replays) for the K rule.
    userAgent: 'Mozilla/5.0 (Linux; Android 10; K) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/141.0.0.0 Mobile Safari/537.36' });
  await context.grantPermissions(['camera', 'accelerometer', 'gyroscope'], { origin });
  // Every request of the context (page, module imports, workers), not only page.route traffic.
  const contextRequests = [];
  context.on('request', (req) => contextRequests.push(req.url()));
  const page = await context.newPage();
  await page.route('**/*', (route) => {
    const req = route.request();
    if (req.method() !== 'GET') nonGet.push(`${req.method()} ${req.url()}`);
    if (candidate) {
      const body = { [`${origin}/slam-refact/slam_refact_engine.js`]: [candidate.js, 'text/javascript'],
        [`${origin}/slam-refact/slam_refact_engine.wasm`]: [candidate.wasm, 'application/wasm'],
        [`${origin}/slam-refact/manifest.json`]: [candidate.manifest, 'application/json'] }[req.url().split('?')[0]];
      if (body) return route.fulfill({ status: 200, body: body[0], contentType: body[1] });
    }
    if (req.url().startsWith(origin + '/') || req.url().startsWith('blob:')) return route.continue();
    blocked.push(req.url());
    return route.abort();
  });
  page.on('console', (m) => { if (m.type() === 'error') consoleErrors.push(m.text()); });
  page.on('pageerror', (e) => pageErrors.push(String(e)));
  await page.goto(origin + '/slam-refact.html' + query);
  await page.waitForFunction(() => window.__slamRefactSnapshot?.().workerReady === true, null, { timeout: 120000 });
  await page.waitForTimeout(1000);
  const beforeStart = await page.evaluate(() => window.__slamRefactSnapshot());
  const engineLabel = await page.textContent('#engine');
  const downloadDisabledBeforeStart = await page.isDisabled('#download');

  // Synthetic sensors (page context; stand-in for the phone's events).
  await page.evaluate((rows) => {
    let i = 0;
    const tick = () => {
      const s = rows[i % rows.length]; i++;
      const v = (a) => ({ x: a[0], y: a[1], z: a[2] });
      window.dispatchEvent(new DeviceMotionEvent('devicemotion', { accelerationIncludingGravity: v(s.aig),
        acceleration: s.lin ? v(s.lin) : null, rotationRate: { alpha: s.rate[0], beta: s.rate[1], gamma: s.rate[2] },
        interval: s.intervalMs }));
      window.dispatchEvent(new DeviceOrientationEvent('deviceorientation', { alpha: s.orient[0], beta: s.orient[1], gamma: s.orient[2] }));
      setTimeout(tick, rows[i % rows.length].dtMs || 16);
    };
    tick();
  }, sensorRows);
  await page.click('#start');
  await page.waitForFunction(() => window.__slamRefactSnapshot().state === 'running', null, { timeout: 30000 });
  let fatalRecovery = null;
  if (fatalAfter) {
    await page.waitForFunction(() => window.__slamRefactSnapshot().state === 'error', null, { timeout: 180000 });
    const beforeRecordingFinalized = await page.evaluate(() => window.__slamRefactSnapshot());
    const originalStateText = await page.textContent('#state');
    if (recordOn) {
      await page.waitForFunction(() => typeof window.__slamRefactSnapshot().recording?.ok === 'boolean',
        null, { timeout: 60000 });
    }
    const atFatal = await page.evaluate(() => window.__slamRefactSnapshot());
    fatalRecovery = { atFatal, downloadEnabledAfterFatal: !(await page.isDisabled('#download')),
      startEnabledAfterFatal: !(await page.isDisabled('#start')), stateText: await page.textContent('#state'),
      beforeRecordingFinalized, originalStateText,
      recordingDownloadEnabledAfterFatal: recordOn ? !(await page.isDisabled('#downloadRec')) : null };
    await page.click('#start');
    await page.waitForFunction(() => window.__slamRefactSnapshot().state === 'running', null, { timeout: 120000 });
  }
  await page.waitForTimeout(seconds * 1000);
  await page.click('#stop');
  await page.waitForTimeout(500);
  const afterStop = await page.evaluate(() => window.__slamRefactSnapshot());
  const pageResources = await page.evaluate(() => performance.getEntriesByType('resource').map((e) => e.name));
  if (args['view'] === 'fit') { await page.click('#viewFit'); await page.waitForTimeout(300); }
  let recording = null;
  if (recordOn) {
    // Wait for the writer to drain and the meta JSON to be hashed.
    await page.waitForFunction(() => window.__slamRefactSnapshot().recording?.ok !== null, null, { timeout: 60000 });
    const summary = await page.evaluate(() => window.__slamRefactSnapshot().recording);
    const downloadEnabled = !(await page.isDisabled('#downloadRec'));
    fs.mkdirSync(path.join(recOut, 'downloaded'), { recursive: true });
    // User-click download: two files.
    const saved = [];
    const onDl = async (d) => { const f = path.join(recOut, 'downloaded', d.suggestedFilename()); await d.saveAs(f); saved.push(f); };
    page.on('download', onDl);
    await page.click('#downloadRec');
    for (let i = 0; i < 100 && saved.length < 2; i++) await page.waitForTimeout(100);
    page.off('download', onDl);
    // CDP/evaluate pull through window.__slamRefactRecording.
    const names = await page.evaluate(() => window.__slamRefactRecording.files());
    const pulled = {};
    for (const name of names) {
      const size = await page.evaluate((n) => window.__slamRefactRecording.size(n), name);
      const chunks = [];
      for (let off = 0; off < size; off += 4 << 20) {
        chunks.push(Buffer.from(await page.evaluate(([n, o, l]) => window.__slamRefactRecording.readChunk(n, o, l), [name, off, 4 << 20]), 'base64'));
      }
      const buf = Buffer.concat(chunks);
      fs.writeFileSync(path.join(recOut, name), buf);
      pulled[name] = { bytes: buf.length, sha256: crypto.createHash('sha256').update(buf).digest('hex') };
    }
    const sameAsDownload = saved.length === 2 && saved.every((f) => pulled[path.basename(f)]?.sha256
      === crypto.createHash('sha256').update(fs.readFileSync(f)).digest('hex'));
    // OPFS retains an earlier fatal run too; verify the current recording's named metadata.
    const metaName = names.includes(summary.files?.meta) ? summary.files.meta : null;
    const metaSha = metaName ? pulled[metaName].sha256 : null;
    const recMeta = metaName ? JSON.parse(fs.readFileSync(path.join(recOut, metaName), 'utf8')) : null;
    recording = { summary, downloadEnabled, downloadedFiles: saved.map((f) => path.basename(f)), pulled, names,
      downloadMatchesPull: sameAsDownload, metaSha256Matches: metaSha !== null && metaSha === summary.metaSha256,
      framesBytesMatch: summary.ok && pulled[summary.files.frames]?.bytes === summary.frames * 480 * 640,
      metaFrames: recMeta?.frames.length ?? null, mainThreadCostMs: recMeta?.recording.mainThreadCostMs ?? null,
      outDir: recOut };
  } else {
    recording = { offByDefault: await page.evaluate(async () => ({ snapshot: window.__slamRefactSnapshot().recording,
      api: typeof window.__slamRefactRecording, badgeHidden: document.getElementById('recBadge').hidden,
      bannerHidden: document.getElementById('recBanner').hidden, downloadRecHidden: document.getElementById('downloadRec').hidden,
      opfsRecordingFiles: await (async () => { try { const r = await navigator.storage.getDirectory(); const o = [];
        for await (const [n] of r.entries()) if (n.startsWith('slam-refact-rec-')) o.push(n); return o; } catch { return null; } })() })) };
  }
  const [download] = await Promise.all([page.waitForEvent('download'), page.click('#download')]);
  const metricsPath = path.join(outDir, 'metrics.json');
  await download.saveAs(metricsPath);
  const metrics = JSON.parse(fs.readFileSync(metricsPath, 'utf8'));
  const timingText = await page.textContent('#timing');
  const recordKeys = [...new Set(metrics.records.flatMap((r) => Object.keys(r)))].sort();
  const forbidden = recordKeys.filter((k) => /rgba|^gray$|pixel|image|accel|gyro|aig|rotationRate/i.test(k));
  const s = metrics.summary;
  result = {
    schema: 'slam-refact-live-smoke-v1', label: args.label, origin, browser: browser.version(),
    environment: 'headless desktop Chromium, fake Y4M camera + synthetic sensor events; NOT phone performance',
    bundleDir, fakeCameraFrames: framePaths.length, seconds, query, fatalRecovery,
    candidateEngine: candidate ? { dir: path.relative(repo, candidate.dir), manifest: JSON.parse(candidate.manifest) } : null,
    frontend: metrics.frontend, pyramidLayout: metrics.pyramidLayout, gpuChecks: metrics.gpuChecks,
    engineLabel, beforeStart, afterStop, recording,
    checks: {
      idleBeforeStart: beforeStart.state === 'idle' && beforeStart.counters.posted === 0 && !beforeStart.cameraOpen,
      downloadDisabledBeforeStart,
      engineBytesMatchManifest: metrics.engine.loaded?.matchesManifest === true,
      loadedWasmSha256: metrics.engine.loaded?.wasmSha256, manifestWasmSha256: metrics.engine.manifest.artifacts.wasm.sha256,
      processedFrames: metrics.counters.results, freshFrames: metrics.counters.fresh,
      droppedBusy: metrics.counters.droppedBusy, videoFrames: metrics.counters.videoFrames,
      motionEvents: metrics.counters.motionEvents, orientationEvents: metrics.counters.orientationEvents,
      statuses: [...new Set(metrics.records.map((r) => r.status))],
      finitePoses: metrics.records.every((r) => [...r.position, ...r.rotationWxyz].every(Number.isFinite)),
      firstTranslation: s.firstTranslation, consoleErrors, pageErrors, externalRequestsBlocked: blocked,
      nonGetRequests: nonGet, metricsForbiddenKeys: forbidden, crossOriginIsolated: metrics.page.crossOriginIsolated,
      nonOriginRequests: [...new Set([...contextRequests, ...pageResources])].filter((u) => !u.startsWith(origin + '/') && !u.startsWith('blob:') && !u.startsWith('data:')),
      frontendRequested: metrics.frontend?.requested ?? null, frontendActive: metrics.frontend?.active ?? null,
      view3d: afterStop.view3d,
      worldPointDisplay: worldPointDisplayEvidence(metrics.records),
      gpuCheckFrames: (metrics.gpuChecks ?? []).length,
      gpuCheckIdenticalSwiftShader: (metrics.gpuChecks ?? []).filter((g) => g['swiftshader-approx']?.identical).length,
      trackerStateExported: metrics.records.some((r) => r.tracker && Number.isFinite(r.tracker.numKeyframes)),
      // Keep the historical maximum as a diagnostic. Point display is checked against keyframes
      // of the same fresh record above; the final snapshot can be empty after a later map reset.
      maxKeyframes: Math.max(0, ...metrics.records.map((r) => r.tracker?.numKeyframes ?? 0)),
      camera: metrics.camera?.video, capturePlan: metrics.capturePlan,
    },
    timingMs: { preInit: s.preInit, postInit: s.postInit },
    timingText, recordKeys, servedRequests: served,
  };
  const c = result.checks;
  result.pass = c.idleBeforeStart && c.downloadDisabledBeforeStart && c.engineBytesMatchManifest
    && c.processedFrames > 20 && c.freshFrames > 0 && c.finitePoses && c.motionEvents > 0
    && consoleErrors.length === 0 && pageErrors.length === 0 && blocked.length === 0 && nonGet.length === 0
    && c.nonOriginRequests.length === 0 && c.frontendActive === c.frontendRequested
    && c.view3d && c.view3d.poses > 0 && c.worldPointDisplay.passed && c.trackerStateExported
    && c.gpuCheckIdenticalSwiftShader === c.gpuCheckFrames
    && (!/gpu-check=1/.test(query) || c.gpuCheckFrames >= 10)
    && (!fatalAfter || (fatalRecovery.atFatal.records > 0 && fatalRecovery.atFatal.cameraOpen === false
      && fatalRecovery.downloadEnabledAfterFatal && fatalRecovery.startEnabledAfterFatal
      && /injected fatal/.test(fatalRecovery.stateText) && fatalRecovery.stateText === fatalRecovery.originalStateText
      && (!recordOn || (fatalRecovery.atFatal.recording?.ok === true && fatalRecovery.recordingDownloadEnabledAfterFatal))
      && afterStop.lastError === null))
    && forbidden.length === 0 && c.capturePlan?.rotate === 'cw' && c.capturePlan?.cropped === false
    && c.camera?.width === W && c.camera?.height === H
    && (recordOn
      ? recording.summary.ok === true && recording.downloadEnabled && recording.downloadedFiles.length === 2
        && recording.downloadMatchesPull && recording.metaSha256Matches && recording.framesBytesMatch
        && recording.metaFrames === recording.summary.frames && recording.summary.frames === metrics.counters.posted
        && metrics.recording?.metaSha256 === recording.summary.metaSha256
      : recording.offByDefault.snapshot === null && recording.offByDefault.api === 'undefined'
        && recording.offByDefault.badgeHidden && recording.offByDefault.bannerHidden && recording.offByDefault.downloadRecHidden
        && recording.offByDefault.opfsRecordingFiles?.length === 0);
  await page.screenshot({ path: path.join(outDir, 'page.png'), fullPage: true });
} finally {
  await browser.close();
  server.close();
}
fs.unlinkSync(y4m);  // derived from build/ replay frames; not kept as evidence (size)
fs.writeFileSync(path.join(outDir, 'result.json'), JSON.stringify(result, null, 1) + '\n', { flag: 'wx' });
console.log(JSON.stringify({ pass: result.pass, out: path.join(outDir, 'result.json'), checks: result.checks,
  engineMsPost: result.timingMs.postInit.engineMs, engineMsPre: result.timingMs.preInit.engineMs }, null, 1));
process.exit(result.pass ? 0 : 1);

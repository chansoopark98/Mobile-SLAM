// Headless smoke of the US-010 samples (slam_refact/web-chunk/samples/{threejs,aframe}) under the deployed xr.js + the slam_refact chunk.
// Preregistration: build/slam-refact-full-20261019/preregistration/us010-samples.v1.md (REPORT grade; criteria S1..S13 in
// sample-smoke-criteria.mjs). One headless Chromium, an ephemeral 127.0.0.1 server, external requests blocked and counted, nothing written
// under web/. The fake camera is a MediaStreamTrackGenerator fed from the recorded flip6-s2 frames (tests/sample-input.js).
//
//   node slam_refact/web-chunk/tests/run-sample-smoke.mjs --sample threejs|aframe --label <new label>
//     [--max-frames 300] [--control N0|M1|M2|M3|M4] [--engine-dir <dir>] [--timing] [--out-root <dir under build/>]
//   --control: negative control of section 2 of the preregistration (N0 fake chunk, M1 camera-lag, M2 hit-shape, M3 no-hit chunk copy, M4 frozen-pose chunk copy)
//   --timing:  no screenshots (they perturb frame times); S13 is then reported as not evaluated
// Output: <out-root>/<sample>/<label>/{meta.json,result.json,state.json,pre.png,post.png,final.png}. A label is never reused.
import fs from 'node:fs/promises';
import os from 'node:os';
import path from 'node:path';
import { createHash } from 'node:crypto';
import { parseArgs } from 'node:util';
import { chromium } from '../../../scripts/dev/node_modules/playwright/index.mjs';
import { startSampleServer, CHUNK_FILES, SDK_DIR, THREE_DIR, AFRAME_DIR, DEFAULT_ENGINE_DIR } from '../samples/serve.mjs';
import { loadFixture, UA } from './sample-fixture.mjs';
import { evaluate, CONTROL_EXPECTATIONS, controlOutcome } from './sample-smoke-criteria.mjs';
import { timingSummary } from './sample-timing.mjs';

const repo = path.resolve(import.meta.dirname, '../../..');
const chunkDir = path.join(repo, 'slam_refact/web-chunk');
const SDK_XR_SHA256 = '1e4c00ab32d0eef25110891ef2aba6ad41cac76fac5e27f1361fd694a5c05563';
const sha = (bytes) => createHash('sha256').update(bytes).digest('hex');

// The text the mutated chunk copies are made from (replaced exactly once or the run stops).
const MUTANT_PATCHES = {
  M3: [["  const hitTest = (x, y, includedTypes = []) => {\n    if (!engine_) return [];", "  const hitTest = (x, y, includedTypes = []) => {\n    return [];\n    if (!engine_) return [];"]],
  M4: [["      rotation: { x, y, z, w }, position: { x: d.position[0], y: d.position[1], z: d.position[2] },",
    "      rotation: (globalThis.__us010Frozen ??= { rotation: { x, y, z, w }, position: { x: d.position[0], y: d.position[1], z: d.position[2] } }).rotation, position: globalThis.__us010Frozen.position,"]],
  // v3: the chunk reports a distorted rotation (x and z swapped) / a scaled position; both copies stay consistent with themselves.
  M5: [["      rotation: { x, y, z, w }, position: { x: d.position[0], y: d.position[1], z: d.position[2] },",
    "      rotation: { x: z, y, z: x, w }, position: { x: d.position[0], y: d.position[1], z: d.position[2] },"]],
  M6: [["      rotation: { x, y, z, w }, position: { x: d.position[0], y: d.position[1], z: d.position[2] },",
    "      rotation: { x, y, z, w }, position: { x: 0.5 * d.position[0], y: 0.5 * d.position[1], z: 0.5 * d.position[2] },"]],
};

export async function makeMutantChunk(control, dir) {
  await fs.mkdir(dir, { recursive: true });
  for (const f of CHUNK_FILES) await fs.copyFile(path.join(chunkDir, f), path.join(dir, f));
  let text = await fs.readFile(path.join(dir, 'xr-slam.js'), 'utf8');
  for (const [from, to] of MUTANT_PATCHES[control]) {
    if (text.split(from).length !== 2) throw new Error(`mutant ${control}: the patched text must occur exactly once`);
    text = text.replace(from, to);
  }
  await fs.writeFile(path.join(dir, 'xr-slam.js'), text);
  return dir;
}

// Counts magenta and non-black pixels of a PNG, decoded by the browser itself (no image dependency).
async function analyzePng(page, png) {
  return page.evaluate(async (b64) => {
    const bytes = Uint8Array.from(atob(b64), (c) => c.charCodeAt(0));
    const bmp = await createImageBitmap(new Blob([bytes], { type: 'image/png' }));
    const c = new OffscreenCanvas(bmp.width, bmp.height);
    const g = c.getContext('2d');
    g.drawImage(bmp, 0, 0);
    const d = g.getImageData(0, 0, bmp.width, bmp.height).data;
    let magenta = 0, nonBlack = 0;
    for (let i = 0; i < d.length; i += 4) {
      const r = d[i], gr = d[i + 1], b = d[i + 2];
      if (r >= 200 && gr <= 60 && b >= 200) magenta++;
      if (0.299 * r + 0.587 * gr + 0.114 * b > 8) nonBlack++;
    }
    return { width: bmp.width, height: bmp.height, magenta, nonBlackFraction: nonBlack / (d.length / 4) };
  }, png.toString('base64'));
}

export async function main(argv = process.argv.slice(2)) {
  const { values } = parseArgs({ args: argv, options: {
    sample: { type: 'string' }, label: { type: 'string' }, 'max-frames': { type: 'string', default: '300' }, control: { type: 'string' },
    'engine-dir': { type: 'string', default: DEFAULT_ENGINE_DIR }, 'out-root': { type: 'string', default: 'build/slam-refact-full-20261019/chunk/us010-20261020/runs' },
    timing: { type: 'boolean', default: false }, scenario: { type: 'string', default: 'flip6-s2' },
    'no-tap': { type: 'boolean', default: false }, 'probe-extra': { type: 'string', default: '' },
    // v4: the evidence root the output must stay under, the registered control expectations (v3 / v4), and the driver script (its sha256 is recorded)
    'owned-root': { type: 'string', default: 'build/slam-refact-full-20261019/chunk' }, expectations: { type: 'string', default: 'v3' }, driver: { type: 'string' },
  } });
  if (!['threejs', 'aframe'].includes(values.sample)) throw new Error('--sample threejs|aframe');
  if (!/^[a-z0-9-]+$/.test(values.label || '')) throw new Error('--label [a-z0-9-]+ required (new label per attempt)');
  const expectations = CONTROL_EXPECTATIONS[values.expectations];
  if (!expectations) throw new Error(`--expectations ${Object.keys(CONTROL_EXPECTATIONS).join('|')}`);
  if (values.control && !expectations.fail[values.control]) throw new Error(`--control ${Object.keys(expectations.fail).join('|')}`);
  const owned = path.resolve(repo, values['owned-root']);
  if (!owned.startsWith(path.join(repo, 'build') + path.sep)) throw new Error('--owned-root must be under build/');
  const outRoot = path.resolve(repo, values['out-root']);
  if (!outRoot.startsWith(owned + path.sep)) throw new Error('output must stay under ' + owned);
  const engineDir = path.resolve(repo, values['engine-dir']);
  if (!engineDir.startsWith(path.join(repo, 'build') + path.sep)) throw new Error('--engine-dir must be under build/');
  const out = path.join(outRoot, values.sample, values.label);
  await fs.mkdir(path.dirname(out), { recursive: true });
  await fs.mkdir(out, { recursive: false });

  const data = await loadFixture(values.scenario, Number(values['max-frames']));
  const control = values.control ?? null;
  let servedChunkDir = chunkDir;
  if (control === 'N0') servedChunkDir = path.join(chunkDir, 'tests/fake-chunk');
  if (['M3', 'M4', 'M5', 'M6'].includes(control)) servedChunkDir = await makeMutantChunk(control, path.join(path.dirname(out), `mutant-chunk-${control.toLowerCase()}-${values.label}`));
  const mutate = control === 'M1' ? 'camera-lag' : control === 'M2' ? 'hit-shape' : '';

  const driver = values.driver ? { file: values.driver, sha256: sha(await fs.readFile(path.resolve(repo, values.driver))) } : null;
  const report = { schema: 'slam-refact-sample-smoke-v1', expectations: values.expectations, driver, sample: values.sample, label: values.label, control, timing: values.timing, noTap: values['no-tap'], probeExtra: values['probe-extra'], argv: process.argv,
    startedAt: new Date().toISOString(), node: process.version, loadAvgAtStart: os.loadavg(),
    input: { scenario: values.scenario, frames: data.frames.length, imuRows: data.imuRows, fixtureSha256: data.fixtureSha256, sourceSha256: data.sourceSha256,
      nominalFps: data.nominalFps, intervalMs: data.intervalMs },
    requests: [], requestsOffOrigin: [], blockedExternal: [], console: [], consoleErrors: 0, pageErrors: [], requestFailures: [], served: {} };
  let server, browser;
  try {
    for (const f of CHUNK_FILES) {
      try { report.served[f] = sha(await fs.readFile(path.join(servedChunkDir, f))); } catch { /* the fake chunk has xr-slam.js only */ }
    }
    const xrBytes = await fs.readFile(path.join(SDK_DIR, 'xr.js'));
    if (sha(xrBytes) !== SDK_XR_SHA256) throw new Error('SDK xr.js sha256 differs from the pinned value');
    report.served['xr.js'] = SDK_XR_SHA256;
    report.served['three.module.min.js'] = sha(await fs.readFile(path.join(THREE_DIR, 'three.module.min.js')));
    report.served['three.core.min.js'] = sha(await fs.readFile(path.join(THREE_DIR, 'three.core.min.js')));
    report.served['aframe-v1.5.0.min.js'] = sha(await fs.readFile(path.join(AFRAME_DIR, 'aframe-v1.5.0.min.js')));
    const engineManifest = JSON.parse(await fs.readFile(path.join(engineDir, 'manifest.json'), 'utf8'));
    report.engineManifest = engineManifest.artifacts;

    const inputScript = await fs.readFile(path.join(import.meta.dirname, 'sample-input.js'), 'utf8');
    const extra = async (pathname) => {
      if (pathname === '/input/fixture.json') return { body: data.fixtureBytes, type: 'application/json' };
      const m = /^\/input\/gray\/(\d+)$/.exec(pathname);
      if (m) return { body: await data.gray(Number(m[1])), type: 'application/octet-stream' };
      return null;
    };
    const srv = await startSampleServer({ dirs: { chunk: servedChunkDir, engine: engineDir }, extra });
    server = srv;
    const origin = srv.origin;
    report.origin = origin;

    const launch = { executablePath: process.env.BROWSER_EXECUTABLE || '/usr/bin/google-chrome', headless: true,
      args: ['--enable-unsafe-swiftshader', '--autoplay-policy=no-user-gesture-required', '--use-fake-ui-for-media-stream', '--use-fake-device-for-media-stream'] };
    report.browserLaunch = launch;
    browser = await chromium.launch(launch);
    report.browserVersion = browser.version();
    const context = await browser.newContext({ viewport: { width: data.width, height: data.height }, permissions: ['camera', 'microphone'], isMobile: true,
      hasTouch: true, deviceScaleFactor: 1, userAgent: UA[data.ua] });
    const page = await context.newPage();
    await page.addInitScript(inputScript);
    await page.route('**/*', (route) => {
      const url = route.request().url();
      if (!url.startsWith(origin + '/') && !url.startsWith('data:') && !url.startsWith('blob:')) { report.blockedExternal.push(url); return route.abort('blockedbyclient'); }
      return route.continue();
    });
    page.on('response', (r) => { const u = r.url(); report.requests.push({ url: u.replace(origin, ''), status: r.status() }); if (!u.startsWith(origin + '/') && !u.startsWith('data:') && !u.startsWith('blob:')) report.requestsOffOrigin.push(u); });
    page.on('requestfailed', (r) => report.requestFailures.push({ url: r.url(), failure: r.failure() }));
    page.on('console', (m) => { if (m.type() === 'error') report.consoleErrors++; if (report.console.length < 400) report.console.push({ type: m.type(), text: m.text().slice(0, 500) }); });
    page.on('pageerror', (e) => report.pageErrors.push({ message: e.message, stack: e.stack }));

    const url = `${origin}/samples/${values.sample}/index.html?probe=1${mutate ? '&mutate=' + mutate : ''}${values['probe-extra'] ? '&' + values['probe-extra'] : ''}`;
    report.url = url;
    await page.goto(url);
    await page.waitForFunction(() => window.__xrSample?.ready === true, null, { timeout: 90000 });
    const lastT = data.frames.at(-1).t;
    const total = Math.round(lastT * 1000) + 120000;
    const shots = {};
    const tapWait = await page.waitForFunction(() => window.__xrSample.tapDue || window.__input.done, null, { timeout: total, polling: 100 }).then(() => true, () => false);
    // v2 tap rule (preregistration us010-samples.v2.md): up to 5 taps, each at a probe-armed `tapDue` (centre hit with distance > 0); the first tap
    // that makes the sample show the cube ends the tapping. The pre-tap screenshot is taken before the first tap only.
    let tapDue = !values['no-tap'] && tapWait && await page.evaluate(() => window.__xrSample.tapDue);
    report.taps = [];
    for (let attempt = 1; tapDue && attempt <= 5; attempt++) {
      if (attempt === 1 && !values.timing) {
        const png = await page.screenshot({ type: 'png' });
        await fs.writeFile(path.join(out, 'pre.png'), png);
        shots.pre = await analyzePng(page, png);
      }
      const at = await page.evaluate(() => { const s = window.__xrSample; s.tapDue = false; s.lastTapUpdate = s.realityUpdates; s.taps++; return { wall: performance.now(), update: s.realityUpdates }; });
      report.taps.push({ attempt, ...at });
      await page.touchscreen.tap(data.width / 2, data.height / 2);
      const outcome = await page.waitForFunction((from) => (window.__xrSample.placement ? 'placed' : window.__xrSample.realityUpdates > from + 15 ? 'retry' : window.__input.done ? 'done' : false),
        at.update, { timeout: 60000, polling: 50 }).then((h) => h.jsonValue(), () => 'timeout');
      report.taps.at(-1).outcome = outcome;
      if (outcome === 'placed') {
        await page.waitForFunction(() => window.__xrSample.screenshotDue || window.__input.done, null, { timeout: 60000, polling: 50 }).catch(() => {});
        if (!values.timing) {
          const png = await page.screenshot({ type: 'png' });
          await fs.writeFile(path.join(out, 'post.png'), png);
          shots.post = await analyzePng(page, png);
        }
        break;
      }
      if (outcome !== 'retry') break;
      tapDue = await page.waitForFunction(() => window.__xrSample.tapDue || window.__input.done, null, { timeout: total, polling: 100 }).then(() => page.evaluate(() => window.__xrSample.tapDue), () => false);
    }
    const doneWait = await page.waitForFunction(() => window.__input.done, null, { timeout: total, polling: 500 }).then(() => true, () => false);
    report.runCompleted = doneWait;
    if (!values.timing) await page.screenshot({ type: 'png', path: path.join(out, 'final.png') });
    const sample = await page.evaluate(() => window.__xrSample);
    const input = await page.evaluate(() => window.__input);
    const chunkDiag = await page.evaluate(() => window.__slamRefactChunk ?? null);
    try { await page.evaluate(() => window.__finishInput?.()); } catch (error) { report.cleanupError = String(error); }
    await fs.writeFile(path.join(out, 'state.json'), JSON.stringify({ sample, input, chunkDiag }) + '\n');

    const result = evaluate({ kind: values.sample, pageErrors: report.pageErrors, consoleErrors: report.consoleErrors, blockedExternal: report.blockedExternal.length,
      requestsOffOrigin: report.requestsOffOrigin, requests: report.requests, chunkDiag, input, sample, screenshots: values.timing ? {} : shots,
      sensors: { epochWallMs: input.epochWallMs, packets: data.fixture.sensorPackets.map((p) => [p.sourceTimestamp, p.deviceAcceleration]) } });
    if (values.timing) result.checks.S13 = { pass: null, detail: 'not evaluated in --timing runs (screenshots perturb frame times)' };
    const failed = Object.entries(result.checks).filter(([, c]) => c.pass === false).map(([k]) => k);
    result.failed = failed;
    Object.assign(result, controlOutcome({ checks: result.checks, failed, expectations, control }));
    result.pass = values.timing ? failed.length === 0 : result.pass;
    report.result = { pass: result.pass, failed, controlDetected: result.controlDetected, controlMissed: result.controlMissed };
    await fs.writeFile(path.join(out, 'result.json'), JSON.stringify(result, null, 1) + '\n');
    report.timing = timingSummary({ sample, input, chunkDiag });
    report.screenshots = shots;
    report.chunkDiagSummary = chunkDiag && { engine: chunkDiag.engine, error: chunkDiag.error, warnings: chunkDiag.warnings, sessionLog: chunkDiag.sessionLog,
      stats: Object.fromEntries(Object.entries(chunkDiag.stats).filter(([k]) => !Array.isArray(chunkDiag.stats[k]))) };
    report.status = report.pageErrors.length || sample.errors.length ? 'COMPLETED_WITH_ERRORS' : 'COMPLETED';
  } catch (error) {
    report.status = 'FAILED'; report.error = { message: error.message, stack: error.stack };
  } finally {
    if (browser) { await browser.close(); report.browserClosed = true; }
    if (server) { await server.close(); report.serverClosed = true; }
    report.finishedAt = new Date().toISOString();
    await fs.writeFile(path.join(out, 'meta.json'), JSON.stringify(report, null, 1) + '\n');
  }
  console.log(JSON.stringify({ sample: values.sample, label: values.label, control, status: report.status, out: path.relative(repo, out), result: report.result,
    error: report.error?.message, browserClosed: report.browserClosed, serverClosed: report.serverClosed }, null, 1));
  process.exitCode = report.status === 'FAILED' ? 1 : (control ? (report.result?.controlDetected ? 0 : 1) : (report.result?.pass ? 0 : 1));
  return report;
}

if (process.argv[1] && path.resolve(process.argv[1]) === import.meta.filename) await main();

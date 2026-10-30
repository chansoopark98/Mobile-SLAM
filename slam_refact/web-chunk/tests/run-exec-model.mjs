// US-010 execution-model experiment (preregistration us010-samples.v2.md section 3.2): the engine on the main thread (what the chunk does) versus the
// engine in a Worker, on the same recorded input. Desktop headless Chromium + SwiftShader only; not a phone's numbers.
//   node slam_refact/web-chunk/tests/run-exec-model.mjs --phase capture --label c1
//        runs the three.js sample once on the recorded flip6-s2 frames with the createGlue hook (worker-replay/capture-hook.js) and stores every glue call and
//        atlas under build/slam-refact-full-20261019/chunk/us010-20261020/capture-<label>/ (atlas-<k>.bin, ops.json, index.json with sha256 of every file)
//   node slam_refact/web-chunk/tests/run-exec-model.mjs --phase replay --capture c1 --mode sync|worker|idle --label <label>
//        replays that input at the recorded cadence (worker-replay/replay-page.js) -> .../replay/<label>/{replay.json,summary.json}
// One headless Chromium at a time, ephemeral 127.0.0.1 server, external requests blocked, nothing under web/. A label is never reused.
import fs from 'node:fs/promises';
import http from 'node:http';
import os from 'node:os';
import path from 'node:path';
import { createHash } from 'node:crypto';
import zlib from 'node:zlib';
import { parseArgs } from 'node:util';
import { chromium } from '../../../scripts/dev/node_modules/playwright/index.mjs';
import { createSampleHandler, DEFAULT_ENGINE_DIR } from '../samples/serve.mjs';
import { loadFixture, UA } from './sample-fixture.mjs';
import { stats } from './sample-timing.mjs';

const repo = path.resolve(import.meta.dirname, '../../..');
const replayDir = path.join(import.meta.dirname, 'worker-replay');
const sha = (bytes) => createHash('sha256').update(bytes).digest('hex');
const { values } = parseArgs({ options: { phase: { type: 'string' }, label: { type: 'string' }, capture: { type: 'string' }, mode: { type: 'string' },
  'engine-dir': { type: 'string', default: DEFAULT_ENGINE_DIR }, 'max-frames': { type: 'string', default: '300' },
  // v4: the evidence root (default: the v3 one) and the driver script whose sha256 is recorded in the report (the replay loop, run.sh of the gate)
  'evidence-root': { type: 'string', default: 'build/slam-refact-full-20261019/chunk/us010-20261020' }, driver: { type: 'string' } } });
const evidence = path.resolve(repo, values['evidence-root']);
if (!evidence.startsWith(path.join(repo, 'build') + path.sep)) throw new Error('--evidence-root must be under build/');
const driver = async () => (values.driver ? { file: values.driver, sha256: sha(await fs.readFile(path.resolve(repo, values.driver))) } : null);
if (!/^[a-z0-9-]+$/.test(values.label || '')) throw new Error('--label [a-z0-9-]+ required');
const engineDir = path.resolve(repo, values['engine-dir']);
if (!engineDir.startsWith(path.join(repo, 'build') + path.sep)) throw new Error('--engine-dir must be under build/');

const listen = async (handler) => {
  const server = http.createServer(handler);
  await new Promise((resolve, reject) => { server.once('error', reject); server.listen(0, '127.0.0.1', resolve); });
  return { origin: `http://127.0.0.1:${server.address().port}`, close: () => new Promise((resolve) => server.close(resolve)) };
};
const readBody = (req) => new Promise((resolve, reject) => { const c = []; req.on('data', (d) => c.push(d)); req.on('end', () => resolve(Buffer.concat(c))); req.on('error', reject); });

async function launch(origin, report, viewport, ua) {
  const browser = await chromium.launch({ executablePath: process.env.BROWSER_EXECUTABLE || '/usr/bin/google-chrome', headless: true,
    args: ['--enable-unsafe-swiftshader', '--autoplay-policy=no-user-gesture-required', '--use-fake-ui-for-media-stream', '--use-fake-device-for-media-stream'] });
  report.browserVersion = browser.version();
  const context = await browser.newContext({ viewport, permissions: ['camera', 'microphone'], isMobile: true, hasTouch: true, deviceScaleFactor: 1, userAgent: ua });
  const page = await context.newPage();
  report.blockedExternal = []; report.pageErrors = []; report.console = [];
  await page.route('**/*', (route) => {
    const url = route.request().url();
    if (!url.startsWith(origin + '/') && !url.startsWith('data:') && !url.startsWith('blob:')) { report.blockedExternal.push(url); return route.abort('blockedbyclient'); }
    return route.continue();
  });
  page.on('pageerror', (e) => report.pageErrors.push({ message: e.message, stack: e.stack }));
  page.on('console', (m) => { if (m.type() === 'error' && report.console.length < 100) report.console.push(m.text().slice(0, 300)); });
  return { browser, page };
}

async function capturePhase() {
  const out = path.join(evidence, `capture-${values.label}`);
  await fs.mkdir(out, { recursive: false });
  const data = await loadFixture('flip6-s2', Number(values['max-frames']));
  const report = { phase: 'capture', driver: await driver(), label: values.label, startedAt: new Date().toISOString(), node: process.version, loadAvgAtStart: os.loadavg(), engineDir: path.relative(repo, engineDir) };
  const sampleHandler = createSampleHandler({ dirs: { engine: engineDir }, extra: async (p) => {
    if (p === '/input/fixture.json') return { body: data.fixtureBytes, type: 'application/json' };
    const m = /^\/input\/gray\/(\d+)$/.exec(p);
    return m ? { body: await data.gray(Number(m[1])), type: 'application/octet-stream' } : null;
  } });
  const srv = await listen(async (req, res) => {
    if (req.method !== 'POST') return sampleHandler(req, res);
    const p = new URL(req.url, 'http://x').pathname;
    const body = await readBody(req);
    let m;
    if ((m = /^\/__capture\/atlas\/(\d+)$/.exec(p))) await fs.writeFile(path.join(out, `atlas-${m[1]}.bin.gz`), zlib.gzipSync(body, { level: 1 }));
    else if (p === '/__capture/ops') await fs.writeFile(path.join(out, 'ops.json'), body);
    else { res.writeHead(404); res.end(); return; }
    res.writeHead(204); res.end();
  });
  let browser;
  try {
    const b = await launch(srv.origin, report, { width: data.width, height: data.height }, UA[data.ua]);
    browser = b.browser;
    const page = b.page;
    await page.addInitScript(await fs.readFile(path.join(import.meta.dirname, 'sample-input.js'), 'utf8'));
    await page.addInitScript(await fs.readFile(path.join(replayDir, 'capture-hook.js'), 'utf8'));
    await page.goto(`${srv.origin}/samples/threejs/index.html?probe=1`);
    await page.waitForFunction(() => window.__xrSample?.ready === true, null, { timeout: 90000 });
    await page.waitForFunction(() => window.__input.done, null, { timeout: Math.round(data.frames.at(-1).t * 1000) + 120000, polling: 500 });
    await page.evaluate(() => window.__capture.flush());
    report.captureErrors = await page.evaluate(() => window.__capture.errors);
    report.atlases = await page.evaluate(() => window.__capture.atlases);
    report.firstWp = await page.evaluate(() => window.__xrSample.firstWp);
    report.pageUpdates = await page.evaluate(() => window.__xrSample.realityUpdates);
    await page.evaluate(() => window.__finishInput?.());
    const index = { schema: 'us010-capture-v1', label: values.label, atlases: [], inputFixtureSha256: data.fixtureSha256, firstWp: report.firstWp };
    for (let k = 0; k < report.atlases; k++) { const gz = await fs.readFile(path.join(out, `atlas-${k}.bin.gz`)); const bytes = zlib.gunzipSync(gz); index.atlases.push({ k, bytes: bytes.length, gzBytes: gz.length, sha256: sha(bytes) }); }
    index.opsSha256 = sha(await fs.readFile(path.join(out, 'ops.json')));
    // v3 validity assertions (preregistration us010-samples.v3.md 3.2): the capture is only usable when it is a faithful, continuous input
    const opsDoc = JSON.parse(await fs.readFile(path.join(out, 'ops.json'), 'utf8'));
    const groups = []; let cur = [];
    for (const op of opsDoc.ops) { cur.push(op); if (op.name === 'processFramePyramid') { groups.push({ ops: cur, timeNanos: op.args[0], videoNs: op.args[1] }); cur = []; } }
    const span = (k) => (groups.at(-1)[k] - groups[0][k]) / 1e9;
    const motion = groups.map((g) => g.ops.filter((o) => o.name === 'pushDeviceMotionEvent').length);
    const zeroAfter5 = motion.slice(5).filter((n) => n === 0).length;
    index.validity = { groups: groups.length, timeNanosSpanS: span('timeNanos'), videoNsSpanS: span('videoNs'), spanRatio: span('timeNanos') / span('videoNs'),
      motionEvents: motion.reduce((a, b) => a + b, 0), zeroMotionGroupsAfter5: zeroAfter5, zeroMotionFraction: zeroAfter5 / Math.max(1, groups.length - 5),
      hitCalls: groups.reduce((a, g) => a + g.ops.filter((o) => o.name === 'xrHitTest').length, 0), motionPerGroupQuartiles: [0, 0.25, 0.5, 0.75, 1].map((q) => [...motion].sort((a, b) => a - b)[Math.floor((motion.length - 1) * q)]) };
    const v = index.validity;
    index.validity.checks = { spanRatio_0_9_to_1_1: v.spanRatio >= 0.9 && v.spanRatio <= 1.1, zeroMotionGroups_le_10pct: v.zeroMotionFraction <= 0.1, motionEvents_ge_700: v.motionEvents >= 700, groups_ge_290: v.groups >= 290 };
    index.validity.valid = Object.values(index.validity.checks).every(Boolean);
    index.captureSha256 = sha(JSON.stringify(index));
    report.validity = index.validity;
    await fs.writeFile(path.join(out, 'index.json'), JSON.stringify(index, null, 1) + '\n');
    report.captureSha256 = index.captureSha256; report.status = !index.validity.valid ? 'INVALID_CAPTURE' : report.pageErrors.length || report.captureErrors.length ? 'COMPLETED_WITH_ERRORS' : 'COMPLETED';
  } catch (error) {
    report.status = 'FAILED'; report.error = { message: error.message, stack: error.stack };
  } finally {
    if (browser) { await browser.close(); report.browserClosed = true; }
    await srv.close(); report.serverClosed = true;
    report.finishedAt = new Date().toISOString();
    await fs.writeFile(path.join(out, 'meta.json'), JSON.stringify(report, null, 1) + '\n');
  }
  console.log(JSON.stringify({ phase: 'capture', label: values.label, status: report.status, atlases: report.atlases, captureSha256: report.captureSha256, validity: report.validity, error: report.error?.message }, null, 1));
  process.exitCode = report.status === 'FAILED' || report.status === 'INVALID_CAPTURE' ? 1 : 0;
}

const rafStats = (rafTimes, from, to) => {
  const t = rafTimes.filter((x) => x >= from && x <= to);
  const iv = t.slice(1).map((x, i) => x - t[i]);
  return { ...stats(iv), over25: iv.filter((x) => x > 25).length, over50: iv.filter((x) => x > 50).length };
};
const longtasks = (lt, a, b) => { const l = lt.filter((x) => x.start >= a && x.start <= b); return { count: l.length, totalMs: l.reduce((s, x) => s + x.duration, 0), maxMs: l.length ? Math.max(...l.map((x) => x.duration)) : 0 }; };

function summarize(rep, firstWp, split) {
  // v3 initialization: the first replayed frame whose answer has a non-padding world point (engine runs); idle uses the split of the capture
  const own = rep.groups.findIndex((r) => (r.nreal ?? 0) >= 1);
  const cp = own >= 0 ? own : (split?.firstRealGroup ?? firstWp?.chunkProcessed ?? 0);
  const g = rep.groups.filter((x) => x.runAt);
  const tReady = g[cp]?.runAt ?? null;
  const t0 = rep.startedAt, t1 = rep.finishedAt - 300;
  const phase = (rows, a, b) => ({
    frames: rows.length, raf: rafStats(rep.rafTimes, a, b), longtasks: longtasks(rep.longtasks, a, b),
    syncMs: stats(rows.map((r) => r.syncMs)), engineMs: stats(rows.map((r) => r.engineMs)), hitMs: stats(rows.map((r) => r.hitMs)),
    mirrorHitMs: stats(rows.map((r) => r.mirrorHitMs)), transitMs: stats(rows.map((r) => r.transitMs)), hitCalls: rows.reduce((a, r) => a + (r.hitCalls ?? 0), 0),
    workerMs: stats(rows.map((r) => r.workerMs)), rttMs: stats(rows.map((r) => r.rttMs)),
    allocMs: stats(rows.map((r) => r.allocMs)), postMs: stats(rows.map((r) => r.postMs)), handlerMs: stats(rows.map((r) => r.handlerMs)),
    workerMainThreadMs: stats(rows.map((r) => (r.allocMs ?? NaN) + (r.postMs ?? NaN) + (r.handlerMs ?? NaN) + (r.mirrorHitMs ?? NaN))),
    lateStartMs: stats(rows.map((r) => r.runAt - r.scheduledAt)),
  });
  return { mode: rep.mode, groups: g.length, firstWpGroup: cp, firstRealGroupOwn: own, engineLoadMs: rep.engineLoadMs, matchesManifest: rep.matchesManifest ?? null, wasmSha256: rep.wasmSha256 ?? null,
    all: phase(g, t0, t1), pre: tReady === null ? null : phase(g.slice(0, cp), t0, tReady), post: tReady === null ? null : phase(g.slice(cp), tReady, t1) };
}

async function replayPhase() {
  if (!['sync', 'worker', 'idle'].includes(values.mode)) throw new Error('--mode sync|worker|idle');
  const capDir = path.join(evidence, `capture-${values.capture}`);
  const index = JSON.parse(await fs.readFile(path.join(capDir, 'index.json'), 'utf8'));
  const out = path.join(evidence, 'replay', values.label);
  await fs.mkdir(path.dirname(out), { recursive: true });
  await fs.mkdir(out, { recursive: false });
  const report = { phase: 'replay', driver: await driver(), label: values.label, mode: values.mode, capture: values.capture, captureSha256: index.captureSha256, startedAt: new Date().toISOString(),
    node: process.version, loadAvgAtStart: os.loadavg(), engineDir: path.relative(repo, engineDir) };
  const sampleHandler = createSampleHandler({ dirs: { engine: engineDir }, extra: async (p) => {
    if (p === '/__replay/page.html') return { body: Buffer.from('<!doctype html><meta charset="utf-8"><title>replay</title><script type="module" src="/__replay/replay-page.js"></script>'), type: 'text/html' };
    let m;
    if ((m = /^\/__replay\/(replay-page|replay-core|engine-worker)\.js$/.exec(p))) return { body: await fs.readFile(path.join(replayDir, `${m[1]}.js`)), type: 'text/javascript' };
    if (p === '/__replay/ops.json') return { body: await fs.readFile(path.join(capDir, 'ops.json')), type: 'application/json' };
    if ((m = /^\/__replay\/atlas\/(\d+)$/.exec(p))) return { body: zlib.gunzipSync(await fs.readFile(path.join(capDir, `atlas-${m[1]}.bin.gz`))), type: 'application/octet-stream' };
    return null;
  } });
  const srv = await listen(sampleHandler);
  let browser;
  try {
    const b = await launch(srv.origin, report, { width: 480, height: 640 }, UA.reduced);
    browser = b.browser;
    await b.page.goto(`${srv.origin}/__replay/page.html?mode=${values.mode}`);
    await b.page.waitForFunction(() => window.__replay?.done === true, null, { timeout: 180000, polling: 250 });
    const rep = await b.page.evaluate(() => window.__replay);
    await fs.writeFile(path.join(out, 'replay.json'), JSON.stringify(rep) + '\n');
    if (rep.error) throw new Error(rep.error);
    let split = null; try { split = JSON.parse(await fs.readFile(path.join(capDir, 'split.json'), 'utf8')); } catch { /* none yet */ }
    const summary = summarize(rep, index.firstWp, split);
    await fs.writeFile(path.join(out, 'summary.json'), JSON.stringify(summary, null, 1) + '\n');
    report.summary = summary;
    report.status = report.pageErrors.length ? 'COMPLETED_WITH_ERRORS' : 'COMPLETED';
  } catch (error) {
    report.status = 'FAILED'; report.error = { message: error.message, stack: error.stack };
  } finally {
    if (browser) { await browser.close(); report.browserClosed = true; }
    await srv.close(); report.serverClosed = true;
    report.finishedAt = new Date().toISOString();
    await fs.writeFile(path.join(out, 'meta.json'), JSON.stringify(report, null, 1) + '\n');
  }
  const f = (s) => (s && s.n ? [s.p50, s.p95, s.p99, s.max].map((v) => +v.toFixed(2)) : null);
  const s = report.summary;
  console.log(JSON.stringify({ phase: 'replay', label: values.label, mode: values.mode, status: report.status, error: report.error?.message,
    postRaf: s && f(s.post?.raf), postSync: s && f(s.post?.syncMs), postWorkerMs: s && f(s.post?.workerMs), postRtt: s && f(s.post?.rttMs), postMainWorker: s && f(s.post?.workerMainThreadMs) }));
  process.exitCode = report.status === 'FAILED' ? 1 : 0;
}

if (values.phase === 'capture') await capturePhase();
else if (values.phase === 'replay') await replayPhase();
else throw new Error('--phase capture|replay');

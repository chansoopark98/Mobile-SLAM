#!/usr/bin/env node
// R1 engine-Worker A/B harness (preregistration section R1): headless Chromium, desktop only (SwiftShader GL; never a phone result).
// Arms: A = main = default chunk; B = worker = the same page with ?engineWorker=1. Order ABABAB (ABAB x3), every run in a fresh browser.
//
// --mode perf      LIVE path: the Three.js sample under the deployed xr.js + the slam_refact chunk, fed the recorded flip6-s2 camera frames and
//                  synthetic DeviceMotion at their recorded cadence (sample-input.js). The sample runs WITHOUT ?probe=1, the page carries only the passive
//                  tests/r1-instrument.js (no polling, no rAF loop, one one-shot timer after the input is over).
//   PRIMARY (gate) main-thread busy ms / frame (preregistration v4.5): CDP Tracing of the page's renderer main thread (CrRendererMain), the sum of
//                  ALL top-level task (RunTask) durations in [trace start, sr-frame-done:N] / N. The trace is started before the page (and with it the
//                  session start, XR8.run) exists, so everything the page does from its start is inside the window. Input = exactly N = 30 + 2m
//                  frames (m >= 120, --m, default 120), then feeding stops. The chunk emits (option frameDoneMarks, set in both arms)
//                  performance.mark('sr-frame-start:<seq>') at the first main-thread work of frame seq (its frame-callback entry) and
//                  performance.mark('sr-frame-done:<seq>') once ALL main-thread handling of the engine response to frame seq has returned (what
//                  processing frame seq returned, fresh or a startup repeat: every processed frame has one, the last included; applied + every
//                  module's update/render callbacks); seq = 1, 2, 3, ... in recording order (the page's own pre-recording frames are numbered apart). Boundary tasks count only inside the window; nested
//                  tasks once; idle not at all; the 3000 ms after the window end are summed for the report only. The v4.4 middle window
//                  [sr-frame-start:30, sr-frame-done:30+2m] / (2m+1) is computed from the same trace and is REPORT-ONLY.
//   validity       any violation makes the run not valid (R1 FAIL by the registered rules, scorer/r1_eval.py reads the same marks): a frame 1..N
//                  without exactly one start and one done mark; start or done marks not strictly increasing; start_k >= done_k; a frame without
//                  exactly one render callback (the update that consumes its response) inside [start_k, done_k]; a mark of a frame beyond N; and (harness check of the mark
//                  placement) a done mark before the render callbacks of the loop callback that applied the result.
//   run record     runs/perf-<n>-<arm>.r1run.json = {start, done, work, render, tasks, traceStart} on the trace clock (ms) for scorer/r1_eval.py;
//                  `work` is {} (the harness does not instrument postMessage / receive-handler intervals; see the report).
//   e2e latency    per measured frame: capture callback entry (onProcessGpu of the first pipeline module) -> that frame's done mark;
//                  p95 with numpy-linear interpolation per run. Includes the Worker's one-callback delay.
//   REPORT-ONLY    chunk stage/read/engine-or-submit ms, handed-to-page latency, long tasks, drops, the engine output per source frame and its
//                  live identity (live runs are not bit-deterministic even main vs main: the wall clock stamps the IMU and stage times).
// --mode identity DETERMINISTIC REPLAY: tests/r1-replay.html drives the real chunk (real WebGL2 pyramid) with the same recorded input under a
//                  virtual clock and a fake XR8, no real-time pacing. The answers (keyed by engine source frame) of all six runs must be the same
//                  bytes, and non-empty. This is the identity evidence ((a) of the gate); it needs >= 300 recorded frames outside --smoke.
// --mode both     (default) perf, then identity, in one output directory.
//
// Own server: startSampleServer on 127.0.0.1 with a free port (port 0), one per run, closed after it; external requests are blocked and
// counted. Nothing is written under web/. No other server is touched.
//
//   node slam_refact/web-chunk/tests/run-engine-worker-ab.mjs --smoke --label <new label> [--m 25]
//   node slam_refact/web-chunk/tests/run-engine-worker-ab.mjs --preregistered <file> --label <new label> [--m 120] [--replay-frames 300] [--mode both|perf|identity]
// --smoke proves the harness on a tiny input; its output is marked `smoke: true, judgment: "discard"`. Without --smoke the run needs
// --preregistered <existing file> (its sha256 is recorded), m >= 120 and a >= 300 frame replay: the final measurement is not started by accident.
// Output (a label is never reused): <out-root>/<label>/{result.json, runs/*.json, runs/*.trace.json.gz}; out-root must be under build/.
// result.json records the sha256 of this file and of every file the harness reads or serves (`provenance`).
import fs from 'node:fs/promises';
import os from 'node:os';
import path from 'node:path';
import zlib from 'node:zlib';
import { createHash } from 'node:crypto';
import { parseArgs } from 'node:util';
import { chromium } from '../../../scripts/dev/node_modules/playwright/index.mjs';
import { startSampleServer, CHUNK_FILES, DEFAULT_ENGINE_DIR } from '../samples/serve.mjs';
import { loadFixture, UA } from './sample-fixture.mjs';
import { aggregate, aggregatePrimary, digest, frameWindow, fullWindow, outputsBySource, primaryOfRun, replayIdentity, scorerRun, summarizeRun } from './engine-worker-ab-metrics.mjs';

const repo = path.resolve(import.meta.dirname, '../../..');
const here = import.meta.dirname;
const sha = (bytes) => createHash('sha256').update(bytes).digest('hex');
const M_MIN = 120;  // registered (v4.5): N = 30 + 2m frames, m >= 120
const REPLAY_FRAMES_MIN = 300;  // the deterministic replay needs >= 300 recorded frames
const TRACE_CATEGORIES = ['toplevel', 'devtools.timeline', 'disabled-by-default-devtools.timeline', 'blink.user_timing'];
const LAUNCH_ARGS = ['--enable-unsafe-swiftshader', '--autoplay-policy=no-user-gesture-required', '--use-fake-ui-for-media-stream', '--use-fake-device-for-media-stream'];
// frameDoneMarks: the chunk's measurement option, performance.mark('sr-frame-done:<seq>') after all main-thread handling of a frame's result (both arms)
const OPTIONS_SCRIPT = 'window.SLAM_REFACT_CHUNK_OPTIONS = { deliveryDiagnostics: { enabled: true, maxEntries: 1024 }, frameDoneMarks: true, diagMaxSamples: 100000 };';
const HARNESS_FILES = ['run-engine-worker-ab.mjs', 'engine-worker-ab-metrics.mjs', 'r1-instrument.js', 'r1-replay.js', 'r1-replay.html', 'sample-input.js', 'sample-fixture.mjs'];

async function provenance(engineDir) {
  const files = {};
  for (const f of HARNESS_FILES) files[`slam_refact/web-chunk/tests/${f}`] = sha(await fs.readFile(path.join(here, f)));
  for (const f of CHUNK_FILES) files[`slam_refact/web-chunk/${f}`] = sha(await fs.readFile(path.join(here, '..', f)));
  files['slam_refact/web-chunk/samples/serve.mjs'] = sha(await fs.readFile(path.join(here, '../samples/serve.mjs')));
  files['slam_refact/web-chunk/samples/threejs/app.js'] = sha(await fs.readFile(path.join(here, '../samples/threejs/app.js')));
  files['engine/manifest.json'] = sha(await fs.readFile(path.join(engineDir, 'manifest.json')));
  const bundle = sha(Object.entries(files).sort().map(([k, v]) => `${k}\0${v}\n`).join(''));
  return { harnessSha256: files['slam_refact/web-chunk/tests/run-engine-worker-ab.mjs'], bundleSha256: bundle, files };
}

export async function main(argv = process.argv.slice(2)) {
  const { values } = parseArgs({ args: argv, options: {
    label: { type: 'string' }, smoke: { type: 'boolean', default: false }, preregistered: { type: 'string' }, 'replay-frames': { type: 'string' },
    rounds: { type: 'string', default: '3' }, m: { type: 'string' }, mode: { type: 'string', default: 'both' }, scenario: { type: 'string', default: 'flip6-s2' },
    'engine-dir': { type: 'string', default: DEFAULT_ENGINE_DIR }, 'out-root': { type: 'string', default: 'build/slam-refact-r1-engine-worker' },
  } });
  if (!/^[a-z0-9-]+$/.test(values.label || '')) throw new Error('--label [a-z0-9-]+ required (a label is never reused)');
  if (!['perf', 'identity', 'both'].includes(values.mode)) throw new Error('--mode perf|identity|both');
  let preregistration = null;
  if (!values.smoke) {
    if (!values.preregistered) throw new Error('the measurement needs --preregistered <file>; use --smoke to try the harness on a tiny input');
    const file = path.resolve(repo, values.preregistered);
    preregistration = { file: path.relative(repo, file), sha256: sha(await fs.readFile(file)) };
    if (values.m !== undefined && Number(values.m) < M_MIN) throw new Error(`the measurement needs m >= ${M_MIN}`);
    if (values['replay-frames'] !== undefined && Number(values['replay-frames']) < REPLAY_FRAMES_MIN) throw new Error(`the deterministic replay needs >= ${REPLAY_FRAMES_MIN} frames`);
  }
  const rounds = Number(values.rounds);
  if (!Number.isInteger(rounds) || rounds < 1 || rounds > 5) throw new Error('--rounds 1..5');
  const m = Number(values.m ?? (values.smoke ? '25' : String(M_MIN)));
  if (!Number.isInteger(m) || m < 1) throw new Error('--m <integer >= 1>');
  const N = 30 + 2 * m;  // the registered input: exactly N frames, then feeding stops
  const replayFrames = Number(values['replay-frames'] ?? (values.smoke ? String(Math.max(N, 60)) : String(Math.max(REPLAY_FRAMES_MIN, N))));
  const outRoot = path.resolve(repo, values['out-root']);
  if (!outRoot.startsWith(path.join(repo, 'build') + path.sep)) throw new Error('--out-root must be under build/');
  const engineDir = path.resolve(repo, values['engine-dir']);
  const out = path.join(outRoot, values.label);
  await fs.mkdir(outRoot, { recursive: true });
  await fs.mkdir(out, { recursive: false });
  await fs.mkdir(path.join(out, 'runs'));

  const data = await loadFixture(values.scenario, N);  // exactly N frames
  const replayData = replayFrames === N ? data : await loadFixture(values.scenario, replayFrames);
  const ctx = { data, engineDir, out, m, nFrames: N,
    inputScript: await fs.readFile(path.join(here, 'sample-input.js'), 'utf8'), instrumentScript: await fs.readFile(path.join(here, 'r1-instrument.js'), 'utf8') };
  const order = Array.from({ length: rounds }, () => ['main', 'worker']).flat();
  const meta = { schema: 'slam-refact-r1-engine-worker-ab-v2', label: values.label, argv: process.argv, smoke: values.smoke, mode: values.mode,
    ...(values.smoke ? { judgment: 'discard', note: 'smoke run of the harness on a tiny input; these numbers are not evidence' } : { preregistration }),
    startedAt: new Date().toISOString(), node: process.version, loadAvgAtStart: os.loadavg(), cpus: os.cpus().length,
    inputKind: 'SYNTHETIC/RECORDED desktop input (flip6-s2 frames + synthetic DeviceMotion), headless Chromium + SwiftShader; not a phone; desktop-headless only',
    m, n: N, windowFrames: N, replayFrames, traceCategories: TRACE_CATEGORIES, order,
    input: { scenario: values.scenario, frames: data.frames.length, n: N, m, windowFrames: N, inputSha256: data.fixtureSha256, recordSha256: data.sourceSha256, nominalFps: data.nominalFps },
    chunkOptions: 'deliveryDiagnostics{enabled,maxEntries:1024} in both arms (the delivery records give the source frame of each pose)',
    engineManifest: JSON.parse(await fs.readFile(path.join(engineDir, 'manifest.json'), 'utf8')).artifacts,
    provenance: await provenance(engineDir), runs: [] };
  try {
    const perfRuns = [], idRuns = [];
    if (values.mode !== 'identity') for (const [n, arm] of order.entries()) { const r = await perfRun({ ...ctx, arm, n }); perfRuns.push(r); meta.runs.push({ mode: 'perf', n, arm, status: r.status, file: `runs/perf-${n}-${arm}.json` }); }
    if (values.mode !== 'perf') for (const [n, arm] of order.entries()) { const r = await replayRun({ ...ctx, data: replayData, arm, n, frames: replayData.frames.length }); idRuns.push(r); meta.runs.push({ mode: 'identity', n, arm, status: r.status, file: `runs/identity-${n}-${arm}.json` }); }
    const result = {};
    if (perfRuns.length) {
      const ok = perfRuns.filter((r) => r.primary);
      result.perf = { primary: aggregatePrimary(ok), reportOnly: aggregate(ok.filter((r) => r.summary).map((r) => ({ arm: r.arm, summary: r.summary, outputs: r.outputs }))),
        allRunsValid: perfRuns.length === order.length && perfRuns.every((r) => r.primary?.valid && r.armVerified) };
    }
    if (idRuns.length) {
      const done = idRuns.filter((r) => r.digest);
      result.identity = { ...replayIdentity(done.map((r) => ({ arm: r.arm, digest: r.digest, count: r.count }))), armsVerified: idRuns.every((r) => r.armVerified),
        perRun: idRuns.map((r) => ({ n: r.n, arm: r.arm, count: r.count, digest: r.digest, status: r.status })) };
    }
    meta.status = [...perfRuns, ...idRuns].every((r) => r.status === 'COMPLETED') ? 'COMPLETED' : 'COMPLETED_WITH_ERRORS';
    meta.finishedAt = new Date().toISOString();
    await fs.writeFile(path.join(out, 'result.json'), JSON.stringify({ ...meta, result }, null, 1) + '\n');
    console.log(JSON.stringify({ label: values.label, smoke: values.smoke, status: meta.status, out: path.relative(repo, out), harnessSha256: meta.provenance.harnessSha256,
      bundleSha256: meta.provenance.bundleSha256, perfPrimary: result.perf?.primary, perfAllRunsValid: result.perf?.allRunsValid, identity: result.identity }, null, 1));
  } catch (error) {
    meta.status = 'FAILED'; meta.error = { message: error.message, stack: error.stack };
    await fs.writeFile(path.join(out, 'result.json'), JSON.stringify(meta, null, 1) + '\n');
    console.error(error);
    process.exitCode = 1;
  }
  return meta;
}

// ---- one browser + one server per run ----
async function withPage({ data, engineDir, arm, rec, extraRoutes = {}, scripts }, body) {
  let server, browser;
  try {
    const extra = async (pathname) => {
      if (pathname === '/input/fixture.json') return { body: data.fixtureBytes, type: 'application/json' };
      const m = /^\/input\/gray\/(\d+)$/.exec(pathname);
      if (m) return { body: await data.gray(Number(m[1])), type: 'application/octet-stream' };
      if (extraRoutes[pathname]) return { body: await fs.readFile(path.join(here, extraRoutes[pathname].file)), type: extraRoutes[pathname].type };
      return null;
    };
    server = await startSampleServer({ dirs: { engine: engineDir }, extra });  // port 0: a free port, 127.0.0.1 only
    const origin = server.origin;
    rec.server = { origin, closedAfterRun: false };
    browser = await chromium.launch({ executablePath: process.env.BROWSER_EXECUTABLE || '/usr/bin/google-chrome', headless: true, args: LAUNCH_ARGS });
    rec.browserVersion = browser.version();
    const context = await browser.newContext({ viewport: { width: data.width, height: data.height }, permissions: ['camera', 'microphone'], isMobile: true,
      hasTouch: true, deviceScaleFactor: 1, userAgent: UA[data.ua] });
    const page = await context.newPage();
    for (const s of scripts) await page.addInitScript(s);
    await page.route('**/*', (route) => {
      const url = route.request().url();
      if (!url.startsWith(origin + '/') && !url.startsWith('data:') && !url.startsWith('blob:')) { rec.blockedExternal.push(url); return route.abort('blockedbyclient'); }
      return route.continue();
    });
    page.on('requestfailed', (r) => rec.requestFailures.push({ url: r.url(), failure: r.failure() }));
    page.on('console', (m) => { if (m.type() === 'error' && rec.consoleErrors.length < 50) rec.consoleErrors.push(m.text().slice(0, 300)); });
    page.on('pageerror', (e) => rec.pageErrors.push({ message: e.message }));
    await body({ page, browser, origin });
  } catch (error) {
    rec.error = { message: error.message, stack: error.stack };
  } finally {
    if (browser) { await browser.close(); rec.browserClosed = true; }
    if (server) { await server.close(); if (rec.server) rec.server.closedAfterRun = true; }
  }
}

const newRec = (n, arm, mode) => ({ mode, n, arm, status: 'FAILED', server: null, blockedExternal: [], pageErrors: [], consoleErrors: [], requestFailures: [] });

// ---- LIVE run ----
async function perfRun({ data, engineDir, out, m, nFrames, arm, n, inputScript, instrumentScript }) {
  const rec = newRec(n, arm, 'perf');
  rec.primary = null; rec.summary = null; rec.outputs = null;
  await withPage({ data, engineDir, arm, rec, scripts: [OPTIONS_SCRIPT, inputScript, instrumentScript] }, async ({ page, browser, origin }) => {
    let signal;
    const finished = new Promise((resolve) => { signal = resolve; });
    await page.exposeFunction('__r1Signal', () => signal());
    const url = `${origin}/samples/threejs/index.html${arm === 'worker' ? '?engineWorker=1' : ''}`;
    rec.url = url;
    await browser.startTracing(page, { categories: TRACE_CATEGORIES });
    let traceBuffer = null;
    try {
      await page.goto(url);
      await page.waitForFunction(() => window.__r1 && window.__r1.ready === true, null, { timeout: 90000 });
      const total = Math.round(data.frames.at(-1).t * 1000) + 120000;
      await Promise.race([finished, new Promise((_, reject) => setTimeout(() => reject(new Error('the window did not end (no pose applied after the warm-up?)')), total))]);
    } finally { traceBuffer = await browser.stopTracing(); }
    const events = JSON.parse(traceBuffer.toString('utf8'));
    const traceEvents = Array.isArray(events) ? events : events.traceEvents;
    await fs.writeFile(path.join(out, 'runs', `perf-${n}-${arm}.trace.json.gz`), zlib.gzipSync(traceBuffer));
    const state = await page.evaluate(() => {
      const d = window.__slamRefactChunk, ew = d && d.engineWorker;
      return { r1: window.__r1, marks: performance.getEntriesByType('mark').map((e) => [e.name, e.startTime]), frames: window.__input.frames, inputErrors: window.__input.errors, errors: window.__r1.errors,
        responses: d && d.delivery ? d.delivery.responses.entries : [], stats: d ? { ...d.stats } : null,
        engineWorker: ew ? { active: ew.active, fallback: ew.fallback, limits: ew.limits, stats: ew.stats } : null, engine: d ? d.engine : null,
        chunkError: d ? d.error : null, warnings: d ? d.warnings : [], sessionLog: d ? d.sessionLog : [] };
    });
    try { await page.evaluate(() => window.__finishInput && window.__finishInput()); } catch (error) { rec.cleanupError = String(error); }
    const r1 = state.r1;
    // v4.5 primary window: [trace start, sr-frame-done:N], frames 1..N; the v4.4 middle window (frames 30 .. 30+2m) is report-only
    const window_ = fullWindow({ applied: r1.applied, marks: state.marks, renders: r1.renders, captures: r1.captures, totalFrames: data.frames.length, n: nFrames });
    const middle = frameWindow({ applied: r1.applied, marks: state.marks, renders: r1.renders, captures: r1.captures, renderCallbacks: r1.renderCallbacks, totalFrames: data.frames.length, m });
    rec.window = window_; rec.middleWindowReportOnly = middle;
    rec.primary = primaryOfRun({ trace: traceEvents, window: window_, middle });
    // the run record scorer/r1_eval.py reads (one clock: the trace's), and the page records the window checks were made from
    const sr = scorerRun(traceEvents, { pageMarks: state.marks, applied: r1.applied });
    await fs.writeFile(path.join(out, 'runs', `perf-${n}-${arm}.r1run.json`), JSON.stringify(sr) + '\n');
    rec.scorerRunFile = `runs/perf-${n}-${arm}.r1run.json`; rec.scorerRunError = sr.error ?? null;
    await fs.writeFile(path.join(out, 'runs', `perf-${n}-${arm}.pagerecords.json`), JSON.stringify({ marks: state.marks.filter(([name]) => name.startsWith('sr-frame-')), applied: r1.applied, captures: r1.captures, renders: r1.renders, longtasks: r1.longtasks }) + '\n');
    rec.summary = summarizeRun({ updates: r1.rows, frames: state.frames, responses: state.responses, chunkStats: state.stats, longtasks: r1.longtasks, rafTimes: [], engineWorker: state.engineWorker });
    rec.outputs = outputsBySource({ updates: r1.rows, frames: state.frames, responses: state.responses });
    rec.engine = state.engine && { matchesManifest: state.engine.matchesManifest, wasmSha256: state.engine.wasmSha256, instantiateMs: state.engine.instantiateMs, workerLoadMs: state.engine.workerLoadMs ?? null };
    rec.warnings = state.warnings; rec.sessionLog = state.sessionLog; rec.chunkError = state.chunkError; rec.pageStateErrors = [...state.inputErrors, ...state.errors];
    // the arm must be what it claims to be (a Worker run that fell back to the main thread is not a Worker run)
    rec.armVerified = arm === 'worker' ? state.engineWorker?.active === true : !state.engineWorker;
    rec.status = rec.pageErrors.length || rec.pageStateErrors.length || !rec.armVerified || !rec.primary.valid || sr.error ? 'COMPLETED_WITH_ERRORS' : 'COMPLETED';
  });
  await writeRun(out, `perf-${n}-${arm}`, rec);
  return rec;
}

// ---- DETERMINISTIC REPLAY run ----
async function replayRun({ data, engineDir, out, arm, n, frames }) {
  const rec = newRec(n, arm, 'identity');
  rec.digest = null; rec.count = null;
  await withPage({ data, engineDir, arm, rec, scripts: [OPTIONS_SCRIPT],
    extraRoutes: { '/r1/replay.html': { file: 'r1-replay.html', type: 'text/html; charset=utf-8' }, '/r1/replay.js': { file: 'r1-replay.js', type: 'text/javascript' } } }, async ({ page, origin }) => {
    const url = `${origin}/r1/replay.html?frames=${frames}${arm === 'worker' ? '&engineWorker=1' : ''}`;
    rec.url = url;
    await page.goto(url);
    await page.waitForFunction(() => window.__r1replay && window.__r1replay.done === true, null, { timeout: 600000, polling: 250 });
    const r = await page.evaluate(() => window.__r1replay);
    if (r.error) throw new Error('replay page: ' + r.error);
    const answers = r.answers.slice().sort((a, b) => a.src - b.src);
    rec.count = answers.length;
    rec.digest = sha(JSON.stringify(answers));
    rec.sources = { first: answers[0]?.src ?? null, last: answers.at(-1)?.src ?? null, distinct: new Set(answers.map((a) => a.src)).size };
    rec.nonTrivial = { distinctPositions: new Set(answers.map((a) => a.pos.join(','))).size, answersWithWorldPoints: answers.filter((a) => a.wp.length > 0).length };
    rec.pageErrorsChunk = r.errors; rec.engineWorker = r.engineWorker; rec.engine = r.engine; rec.chunkStats = r.chunkStats; rec.warnings = r.warnings;
    rec.armVerified = arm === 'worker' ? r.engineWorker?.active === true : !r.engineWorker;
    rec.status = rec.pageErrors.length || r.errors.length || !rec.armVerified || !answers.length || rec.sources.distinct !== answers.length ? 'COMPLETED_WITH_ERRORS' : 'COMPLETED';
  });
  await writeRun(out, `identity-${n}-${arm}`, rec);
  return rec;
}

async function writeRun(out, name, rec) {
  const { outputs, ...serializable } = rec;
  await fs.writeFile(path.join(out, 'runs', `${name}.json`), JSON.stringify({ ...serializable, outputDigest: outputs ? digest(outputs) : null, outputSourceFrames: outputs ? outputs.size : null }, null, 1) + '\n');
}

if (process.argv[1] && path.resolve(process.argv[1]) === import.meta.filename) await main();

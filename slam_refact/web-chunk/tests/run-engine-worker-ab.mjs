#!/usr/bin/env node
// R1 engine-Worker A/B harness: headless Chromium, the Three.js sample under the deployed xr.js + the slam_refact chunk, the recorded flip6-s2
// camera frames and synthetic DeviceMotion of sample-input.js (a SYNTHETIC/RECORDED desktop input, SwiftShader GL, never a phone result).
// Arms: main = default chunk; worker = same page with ?engineWorker=1. Order ABABAB (ABAB x3): main, worker, main, worker, main, worker, each
// run in a fresh browser. Per run: main-thread busy time per frame (stage + read + engine call, or frame submit in Worker mode), the XR8 loop's
// CPU time, end-to-end pose latency (camera frame handed to the page -> pipeline update that carries its pose), rAF / long tasks, drops,
// and the engine output per source frame (identity: main vs main is the noise floor of a live run, Worker vs main is read against it).
//
// Own server: startSampleServer on 127.0.0.1 with a free port (port 0), closed at the end; external requests are blocked and counted. Nothing is
// written under web/. No other server is touched.
//
//   node slam_refact/web-chunk/tests/run-engine-worker-ab.mjs --smoke --label <new label> [--max-frames 60]
//   node slam_refact/web-chunk/tests/run-engine-worker-ab.mjs --preregistered <file> --label <new label> [--max-frames 300] [--rounds 3]
// --smoke: proves the harness works on a tiny input; the output is marked `smoke: true, judgment: "discard"` and its numbers are not evidence.
// Without --smoke the run needs --preregistered <existing file> (its sha256 is recorded): the final measurement is not started by accident.
// Output (a label is never reused): <out-root>/<label>/{result.json, runs/<n>-<arm>.json}; out-root must be under build/.
import fs from 'node:fs/promises';
import os from 'node:os';
import path from 'node:path';
import { createHash } from 'node:crypto';
import { parseArgs } from 'node:util';
import { chromium } from '../../../scripts/dev/node_modules/playwright/index.mjs';
import { startSampleServer, DEFAULT_ENGINE_DIR } from '../samples/serve.mjs';
import { loadFixture, UA } from './sample-fixture.mjs';
import { aggregate, digest, outputsBySource, summarizeRun } from './engine-worker-ab-metrics.mjs';

const repo = path.resolve(import.meta.dirname, '../../..');
const sha = (bytes) => createHash('sha256').update(bytes).digest('hex');

export async function main(argv = process.argv.slice(2)) {
  const { values } = parseArgs({ args: argv, options: {
    label: { type: 'string' }, smoke: { type: 'boolean', default: false }, preregistered: { type: 'string' }, 'max-frames': { type: 'string', default: '300' },
    rounds: { type: 'string', default: '3' }, sample: { type: 'string', default: 'threejs' }, scenario: { type: 'string', default: 'flip6-s2' },
    'engine-dir': { type: 'string', default: DEFAULT_ENGINE_DIR }, 'out-root': { type: 'string', default: 'build/slam-refact-r1-engine-worker' },
    'probe-extra': { type: 'string', default: '' },
  } });
  if (!/^[a-z0-9-]+$/.test(values.label || '')) throw new Error('--label [a-z0-9-]+ required (a label is never reused)');
  if (!['threejs', 'aframe'].includes(values.sample)) throw new Error('--sample threejs|aframe');
  let preregistration = null;
  if (!values.smoke) {
    if (!values.preregistered) throw new Error('the measurement needs --preregistered <file>; use --smoke to try the harness on a tiny input');
    const file = path.resolve(repo, values.preregistered);
    preregistration = { file: path.relative(repo, file), sha256: sha(await fs.readFile(file)) };
  }
  const maxFrames = values.smoke ? Math.min(Number(values['max-frames']), 90) : Number(values['max-frames']);
  const rounds = Number(values.rounds);
  if (!Number.isInteger(rounds) || rounds < 1 || rounds > 5) throw new Error('--rounds 1..5');
  const outRoot = path.resolve(repo, values['out-root']);
  if (!outRoot.startsWith(path.join(repo, 'build') + path.sep)) throw new Error('--out-root must be under build/');
  const engineDir = path.resolve(repo, values['engine-dir']);
  const out = path.join(outRoot, values.label);
  await fs.mkdir(outRoot, { recursive: true });
  await fs.mkdir(out, { recursive: false });
  await fs.mkdir(path.join(out, 'runs'));

  const data = await loadFixture(values.scenario, maxFrames);
  const inputScript = await fs.readFile(path.join(import.meta.dirname, 'sample-input.js'), 'utf8');
  const optionsScript = 'window.SLAM_REFACT_CHUNK_OPTIONS = { deliveryDiagnostics: { enabled: true, maxEntries: 1024 }, diagMaxSamples: 100000 };';
  const meta = { schema: 'slam-refact-r1-engine-worker-ab-v1', label: values.label, argv: process.argv, smoke: values.smoke,
    ...(values.smoke ? { judgment: 'discard', note: 'smoke run of the harness on a tiny input; these numbers are not evidence' } : { preregistration }),
    startedAt: new Date().toISOString(), node: process.version, loadAvgAtStart: os.loadavg(), cpus: os.cpus().length,
    inputKind: 'SYNTHETIC/RECORDED desktop input (flip6-s2 frames + synthetic DeviceMotion), headless Chromium + SwiftShader; not a phone',
    sample: values.sample, rounds, order: Array.from({ length: rounds }, () => ['main', 'worker']).flat(),
    input: { scenario: values.scenario, frames: data.frames.length, fixtureSha256: data.fixtureSha256, nominalFps: data.nominalFps },
    chunkOptions: 'deliveryDiagnostics{enabled,maxEntries:1024} in both arms (the delivery records give the source frame of each pose)',
    engineManifest: JSON.parse(await fs.readFile(path.join(engineDir, 'manifest.json'), 'utf8')).artifacts,
    servers: [], runs: [] };
  const runs = [];
  try {
    for (const [n, arm] of meta.order.entries()) {
      const run = await runOnce({ arm, n, data, inputScript, optionsScript, engineDir, sample: values.sample, probeExtra: values['probe-extra'], out });
      runs.push(run);
      meta.runs.push({ n, arm, status: run.status, file: `runs/${n}-${arm}.json`, server: run.server });
    }
    const result = aggregate(runs.filter((r) => r.summary).map((r) => ({ arm: r.arm, summary: r.summary, outputs: r.outputs })));
    meta.status = runs.every((r) => r.status === 'COMPLETED') ? 'COMPLETED' : 'COMPLETED_WITH_ERRORS';
    await fs.writeFile(path.join(out, 'result.json'), JSON.stringify({ ...meta, result, outputDigests: runs.map((r) => ({ n: r.n, arm: r.arm, sha256: r.outputs ? digest(r.outputs) : null })) }, null, 1) + '\n');
    console.log(JSON.stringify({ label: values.label, smoke: values.smoke, status: meta.status, out: path.relative(repo, out), main: result.main, worker: result.worker, identity: result.identity }, null, 1));
  } catch (error) {
    meta.status = 'FAILED'; meta.error = { message: error.message, stack: error.stack };
    await fs.writeFile(path.join(out, 'result.json'), JSON.stringify(meta, null, 1) + '\n');
    console.error(error);
    process.exitCode = 1;
  }
  return meta;
}

async function runOnce({ arm, n, data, inputScript, optionsScript, engineDir, sample, probeExtra, out }) {
  const rec = { n, arm, status: 'FAILED', server: null, blockedExternal: [], pageErrors: [], consoleErrors: [], requestFailures: [], summary: null, outputs: null };
  let server, browser;
  try {
    const extra = async (pathname) => {
      if (pathname === '/input/fixture.json') return { body: data.fixtureBytes, type: 'application/json' };
      const m = /^\/input\/gray\/(\d+)$/.exec(pathname);
      if (m) return { body: await data.gray(Number(m[1])), type: 'application/octet-stream' };
      return null;
    };
    server = await startSampleServer({ dirs: { engine: engineDir }, extra });  // port 0: a free port, 127.0.0.1 only
    const origin = server.origin;
    rec.server = { origin, closedAfterRun: false };
    browser = await chromium.launch({ executablePath: process.env.BROWSER_EXECUTABLE || '/usr/bin/google-chrome', headless: true,
      args: ['--enable-unsafe-swiftshader', '--autoplay-policy=no-user-gesture-required', '--use-fake-ui-for-media-stream', '--use-fake-device-for-media-stream'] });
    const context = await browser.newContext({ viewport: { width: data.width, height: data.height }, permissions: ['camera', 'microphone'], isMobile: true,
      hasTouch: true, deviceScaleFactor: 1, userAgent: UA[data.ua] });
    const page = await context.newPage();
    await page.addInitScript(optionsScript);
    await page.addInitScript(inputScript);
    await page.route('**/*', (route) => {
      const url = route.request().url();
      if (!url.startsWith(origin + '/') && !url.startsWith('data:') && !url.startsWith('blob:')) { rec.blockedExternal.push(url); return route.abort('blockedbyclient'); }
      return route.continue();
    });
    page.on('requestfailed', (r) => rec.requestFailures.push({ url: r.url(), failure: r.failure() }));
    page.on('console', (m) => { if (m.type() === 'error' && rec.consoleErrors.length < 50) rec.consoleErrors.push(m.text().slice(0, 300)); });
    page.on('pageerror', (e) => rec.pageErrors.push({ message: e.message }));
    const url = `${origin}/samples/${sample}/index.html?probe=1&hits=0${arm === 'worker' ? '&engineWorker=1' : ''}${probeExtra ? '&' + probeExtra : ''}`;
    rec.url = url;
    await page.goto(url);
    await page.waitForFunction(() => window.__xrSample?.ready === true, null, { timeout: 90000 });
    const total = Math.round(data.frames.at(-1).t * 1000) + 120000;
    await page.waitForFunction(() => window.__input.done, null, { timeout: total, polling: 500 });
    const state = await page.evaluate(() => {
      const d = window.__slamRefactChunk;
      const ew = d?.engineWorker;
      return { updates: window.__xrSample.updates, frames: window.__input.frames, longtasks: window.__xrSample.longtasks, rafTimes: window.__xrSample.rafTimes,
        errors: window.__xrSample.errors, inputErrors: window.__input.errors, responses: d?.delivery?.responses?.entries ?? [], stats: d ? { ...d.stats } : null,
        engineWorker: ew ? { active: ew.active, fallback: ew.fallback, limits: ew.limits, stats: ew.stats } : null, engine: d?.engine ?? null, chunkError: d?.error ?? null,
        warnings: d?.warnings ?? [], sessionLog: d?.sessionLog ?? [], href: location.href };
    });
    try { await page.evaluate(() => window.__finishInput?.()); } catch (error) { rec.cleanupError = String(error); }
    rec.summary = summarizeRun({ updates: state.updates, frames: state.frames, responses: state.responses, chunkStats: state.stats, longtasks: state.longtasks,
      rafTimes: state.rafTimes, engineWorker: state.engineWorker });
    rec.outputs = outputsBySource({ updates: state.updates, frames: state.frames, responses: state.responses });
    rec.engine = state.engine && { matchesManifest: state.engine.matchesManifest, wasmSha256: state.engine.wasmSha256, instantiateMs: state.engine.instantiateMs, workerLoadMs: state.engine.workerLoadMs ?? null };
    rec.warnings = state.warnings; rec.sessionLog = state.sessionLog; rec.chunkError = state.chunkError;
    rec.pageStateErrors = [...state.errors, ...state.inputErrors];
    // the arm must be what it claims to be (a Worker run that fell back to the main thread is not a Worker run)
    rec.armVerified = arm === 'worker' ? state.engineWorker?.active === true : !state.engineWorker;
    rec.status = rec.pageErrors.length || rec.pageStateErrors.length || !rec.armVerified ? 'COMPLETED_WITH_ERRORS' : 'COMPLETED';
  } catch (error) {
    rec.error = { message: error.message, stack: error.stack };
  } finally {
    if (browser) { await browser.close(); rec.browserClosed = true; }
    if (server) { await server.close(); rec.server.closedAfterRun = true; }
    const { outputs, ...serializable } = rec;
    await fs.writeFile(path.join(out, 'runs', `${n}-${arm}.json`), JSON.stringify({ ...serializable, outputDigest: outputs ? digest(outputs) : null, outputSourceFrames: outputs ? outputs.size : null }, null, 1) + '\n');
  }
  return rec;
}

if (process.argv[1] && path.resolve(process.argv[1]) === import.meta.filename) await main();

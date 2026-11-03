#!/usr/bin/env node
// R1 engine-Worker A/B harness (preregistration section R1): headless Chromium, desktop only (SwiftShader GL by default, hardware GPU with --gpu hardware; never a phone result).
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
// P4 (preregistration v5) options, all recorded in result.json (`provenance.gpu`, `provenance.fps`, `idle`, and per run `gpu` / `idle`):
//   --gpu swiftshader|hardware   default swiftshader (the previous behaviour). hardware = headless Chromium on ANGLE gl-egl on the machine's GPU with the
//                  software rasterizer disabled; before every run's page the WebGL2 UNMASKED_RENDERER_WEBGL string is read and recorded, and a renderer that
//                  is not NVIDIA makes that run invalid (R1-INFRA: the run does not continue and the remaining runs of this invocation are not started).
//                  There is no SwiftShader fallback in hardware mode.
//   --fps <number>               default: the recorded timestamps as before. With --fps the recorded flip6 frames are resampled onto a regular grid,
//                  interval 1000/fps ms (grid time -> the recorded frame nearest in time; the grid time is the frame's physical timestamp; IMU packets
//                  keep their recorded times); the offsets and repeated source frames of the resample are recorded in `input.fps`.
//   --idle-check / --no-idle-check   default on with --preregistered, off with --smoke. Before every run (perf and identity): load1 <= 1.0 and no
//                  cmake/make/ninja/ctest/`node --test`/drift_probe/slam_refact_replay process; otherwise re-check every 30 s for at most 10 min, then the
//                  run is invalid (INVALID_IDLE, not executed). load1 at the start and the end of every run, the wait and the processes found are recorded.
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
import { execFile } from 'node:child_process';
import { startSampleServer, CHUNK_FILES, DEFAULT_ENGINE_DIR } from '../samples/serve.mjs';
import { loadFixture, UA } from './sample-fixture.mjs';
import { aggregate, aggregatePrimary, digest, frameWindow, fullWindow, outputsBySource, primaryOfRun, replayIdentity, scorerRun, summarizeRun } from './engine-worker-ab-metrics.mjs';

const repo = path.resolve(import.meta.dirname, '../../..');
const here = import.meta.dirname;
const sha = (bytes) => createHash('sha256').update(bytes).digest('hex');
const M_MIN = 120;  // registered (v4.5): N = 30 + 2m frames, m >= 120
const REPLAY_FRAMES_MIN = 300;  // the deterministic replay needs >= 300 recorded frames
const TRACE_CATEGORIES = ['toplevel', 'devtools.timeline', 'disabled-by-default-devtools.timeline', 'blink.user_timing'];
const COMMON_LAUNCH_ARGS = ['--autoplay-policy=no-user-gesture-required', '--use-fake-ui-for-media-stream', '--use-fake-device-for-media-stream'];
// swiftshader: the previous argument list, same order. hardware: ANGLE on the machine's GPU through EGL (checked in headless Chromium 144 to report the
// NVIDIA renderer with and without DISPLAY); --disable-software-rasterizer so that a failed GPU init does not silently fall back to SwiftShader.
const GPU_LAUNCH_ARGS = {
  swiftshader: ['--enable-unsafe-swiftshader'],
  hardware: ['--ignore-gpu-blocklist', '--enable-gpu', '--use-gl=angle', '--use-angle=gl-egl', '--disable-software-rasterizer'],
};
const GPU_MODES = Object.keys(GPU_LAUNCH_ARGS);
export const launchArgsFor = (gpu) => {
  if (!GPU_LAUNCH_ARGS[gpu]) throw new Error('--gpu swiftshader|hardware');
  return [...GPU_LAUNCH_ARGS[gpu], ...COMMON_LAUNCH_ARGS];
};
// R1 idle rule (preregistration v5): load1 <= 1.0 and none of these processes; re-check every 30 s for at most 10 min, then the run is invalid.
export const IDLE_RULE = { loadMax: 1.0, pollMs: 30000, maxWaitMs: 600000,
  processes: ['cmake', 'make', 'ninja', 'ctest', 'node --test', 'drift_probe', 'slam_refact_replay'] };
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

// ---- command line (no filesystem access: unit-tested) ----
export function parseCli(argv) {
  const { values } = parseArgs({ args: argv, allowNegative: true, options: {
    label: { type: 'string' }, smoke: { type: 'boolean', default: false }, preregistered: { type: 'string' }, 'replay-frames': { type: 'string' },
    rounds: { type: 'string', default: '3' }, m: { type: 'string' }, mode: { type: 'string', default: 'both' }, scenario: { type: 'string', default: 'flip6-s2' },
    'engine-dir': { type: 'string', default: DEFAULT_ENGINE_DIR }, 'out-root': { type: 'string', default: 'build/slam-refact-r1-engine-worker' },
    gpu: { type: 'string', default: 'swiftshader' }, fps: { type: 'string' }, 'idle-check': { type: 'boolean' },
  } });
  if (!/^[a-z0-9-]+$/.test(values.label || '')) throw new Error('--label [a-z0-9-]+ required (a label is never reused)');
  if (!['perf', 'identity', 'both'].includes(values.mode)) throw new Error('--mode perf|identity|both');
  if (!GPU_MODES.includes(values.gpu)) throw new Error('--gpu swiftshader|hardware');
  let fps = null;
  if (values.fps !== undefined) {
    fps = Number(values.fps);
    if (!/^\d+(\.\d+)?$/.test(values.fps) || !(fps > 0) || fps > 120) throw new Error('--fps <number in (0, 120]>');
  }
  if (!values.smoke) {
    if (!values.preregistered) throw new Error('the measurement needs --preregistered <file>; use --smoke to try the harness on a tiny input');
    if (values.m !== undefined && Number(values.m) < M_MIN) throw new Error(`the measurement needs m >= ${M_MIN}`);
    if (values['replay-frames'] !== undefined && Number(values['replay-frames']) < REPLAY_FRAMES_MIN) throw new Error(`the deterministic replay needs >= ${REPLAY_FRAMES_MIN} frames`);
  }
  const rounds = Number(values.rounds);
  if (!Number.isInteger(rounds) || rounds < 1 || rounds > 5) throw new Error('--rounds 1..5');
  const m = Number(values.m ?? (values.smoke ? '25' : String(M_MIN)));
  if (!Number.isInteger(m) || m < 1) throw new Error('--m <integer >= 1>');
  const n = 30 + 2 * m;  // the registered input: exactly N frames, then feeding stops
  const replayFrames = Number(values['replay-frames'] ?? (values.smoke ? String(Math.max(n, 60)) : String(Math.max(REPLAY_FRAMES_MIN, n))));
  const idleCheck = values['idle-check'] ?? !values.smoke;  // on for the measurement, off for a smoke unless asked
  return { values, rounds, m, n, replayFrames, gpu: values.gpu, fps, idleCheck };
}

// ---- regular frame supply (--fps): pure ----
// frameTimes = the recorded physical timestamps (s, ascending). Returns n grid slots at t0 + i/fps (t0 = the first recorded timestamp); every slot takes the
// recorded frame nearest in time (ties: the earlier one). The slot time is the frame's physical timestamp from then on.
export function resampleFrames(frameTimes, fps, n) {
  if (!(fps > 0) || !Number.isInteger(n) || n < 1) throw new Error('resampleFrames(frameTimes, fps > 0, n >= 1)');
  const t0 = frameTimes[0], dt = 1 / fps;
  const last = t0 + (n - 1) * dt;
  if (!(frameTimes.at(-1) >= last)) throw new Error(`the recording (${frameTimes.length} frames, ${(frameTimes.at(-1) - t0).toFixed(1)} s) is shorter than ${n} frames at ${fps} fps`);
  const slots = [];
  let k = 0;
  for (let i = 0; i < n; i++) {
    const t = +(t0 + i * dt).toFixed(9);
    while (k + 1 < frameTimes.length && Math.abs(frameTimes[k + 1] - t) < Math.abs(frameTimes[k] - t)) k++;
    slots.push({ slot: i, t, sourceIndex: k, sourceT: frameTimes[k], offsetMs: +((frameTimes[k] - t) * 1000).toFixed(3) });
  }
  const offs = slots.map((x) => Math.abs(x.offsetMs)).sort((a, b) => a - b);
  const distinct = new Set(slots.map((x) => x.sourceIndex)).size;
  return { slots, report: { fps, intervalMs: 1000 / fps, frames: n, mode: 'regular-grid-nearest-recorded-frame',
    absOffsetMs: { median: offs[(offs.length - 1) >> 1], p95: offs[Math.min(offs.length - 1, Math.ceil(0.95 * offs.length) - 1)], max: offs.at(-1) },
    repeatedSourceFrames: n - distinct, sourceFirst: slots[0].sourceIndex, sourceLast: slots.at(-1).sourceIndex } };
}

// The loaded fixture (all recorded frames) -> the same shape with n regular slots. Sensor packets keep their recorded times (cut 50 ms after the last slot,
// as loadFixture cuts after the last frame).
export function withRegularFps(all, fps, n) {
  const { slots, report } = resampleFrames(all.frames.map((f) => f.t), fps, n);
  const frames = slots.map((s) => ({ index: s.sourceIndex, t: s.t, file: all.frames[s.sourceIndex].file, sha256: all.frames[s.sourceIndex].sha256 }));
  const lastT = frames.at(-1).t;
  const fixture = { ...all.fixture, nominalFps: fps, frames: frames.map((f) => ({ index: f.index, physicalTimestamp: f.t })),
    sensorPackets: all.fixture.sensorPackets.filter((p) => p.sourceTimestamp <= lastT + 0.05) };
  const fixtureBytes = Buffer.from(JSON.stringify(fixture));
  return { ...all, frames, fixture, fixtureBytes, fixtureSha256: sha(fixtureBytes), nominalFps: fps, fpsReport: report,
    gray: (i) => all.gray(frames[i].index) };
}

// ---- idle rule: pure classification + a wait loop with injected effects ----
const stripExt = (t) => path.basename(t).replace(/\.[a-z0-9]+$/i, '');
const BUILD_TOOLS = new Set(['cmake', 'make', 'gmake', 'ninja', 'ctest']);
const REPLAY_TOOLS = new Set(['drift_probe', 'slam_refact_replay']);
// One `ps -eo pid=,args=` line -> the busy-process name it matches (or null). Only the executable and, for an interpreter, its own script/flags count,
// so a shell whose command STRING mentions a tool does not match.
export function busyProcessName(args) {
  const tokens = String(args).trim().split(/\s+/).filter(Boolean);
  if (!tokens.length) return null;
  const exe = path.basename(tokens[0]);
  if (BUILD_TOOLS.has(exe)) return exe === 'gmake' ? 'make' : exe;
  if (REPLAY_TOOLS.has(stripExt(tokens[0]))) return stripExt(tokens[0]);
  const rest = tokens.slice(1);
  const script = rest.find((t) => !t.startsWith('-'));
  if (exe === 'node' || exe === 'nodejs') {
    const flags = rest.slice(0, script ? rest.indexOf(script) : rest.length);
    return flags.includes('--test') ? 'node --test' : null;
  }
  if (/^(python[0-9.]*|bash|sh|dash)$/.test(exe) && script && !rest.includes('-c') && REPLAY_TOOLS.has(stripExt(script))) return stripExt(script);
  return null;
}
export function findBusyProcesses(psOutput, selfPids = []) {
  const found = [];
  for (const line of String(psOutput).split('\n')) {
    const m = /^\s*(\d+)\s+(.*\S)\s*$/.exec(line);
    if (!m || selfPids.includes(Number(m[1]))) continue;
    const name = busyProcessName(m[2]);
    if (name) found.push({ pid: Number(m[1]), name, args: m[2].slice(0, 200) });
  }
  return found;
}
export function idleVerdict({ load1, busy }, rule = IDLE_RULE) {
  const reasons = [];
  if (!(load1 <= rule.loadMax)) reasons.push(`load1 ${Number.isFinite(load1) ? load1.toFixed(2) : load1} > ${rule.loadMax}`);
  for (const b of busy) reasons.push(`process ${b.name} (pid ${b.pid})`);
  return { ok: reasons.length === 0, reasons };
}
// Checks at once; if not idle re-checks every pollMs until maxWaitMs have passed (checks at 0, pollMs, ... maxWaitMs), then gives up. Effects are injected.
export async function waitForIdle({ readLoad, listBusy, sleep, now, rule = IDLE_RULE }) {
  const t0 = now();
  const checks = [];
  for (;;) {
    const sample = { load1: readLoad(), busy: await listBusy() };
    const verdict = idleVerdict(sample, rule);
    checks.push({ atMs: now() - t0, load1: sample.load1, busy: sample.busy, ok: verdict.ok, reasons: verdict.reasons });
    if (verdict.ok) break;
    if (now() - t0 + rule.pollMs > rule.maxWaitMs) break;
    await sleep(rule.pollMs);
  }
  const last = checks.at(-1);
  return { enabled: true, ok: last.ok, waitedMs: last.atMs, attempts: checks.length, loadAtPass: last.ok ? last.load1 : null,
    found: [...new Map(checks.flatMap((c) => c.busy).map((b) => [`${b.pid}:${b.name}`, b])).values()], checks };
}
const listBusyProcesses = () => new Promise((resolve, reject) => {
  execFile('ps', ['-eo', 'pid=,args='], { maxBuffer: 16 << 20 }, (error, stdout) => {
    if (error) reject(error);  // an unreadable process table is not "idle"
    else resolve(findBusyProcesses(stdout, [process.pid]));
  });
});
const gateIdle = () => waitForIdle({ readLoad: () => os.loadavg()[0], listBusy: listBusyProcesses, sleep: (ms) => new Promise((r) => setTimeout(r, ms)), now: () => Date.now() });

// ---- the GPU the page really runs on ----
// hardware must report an NVIDIA renderer, anything else is R1-INFRA; swiftshader only records what it got.
export function rendererVerdict(gpu, renderer) {
  const text = String(renderer ?? '');
  if (gpu !== 'hardware') return { valid: true, reason: null };
  if (!text) return { valid: false, reason: 'no WebGL2 renderer string' };
  if (/swiftshader|llvmpipe|software/i.test(text)) return { valid: false, reason: `software renderer: ${text}` };
  if (!/nvidia/i.test(text)) return { valid: false, reason: `not an NVIDIA renderer: ${text}` };
  return { valid: true, reason: null };
}

export async function main(argv = process.argv.slice(2)) {
  const { values, rounds, m, n: N, replayFrames, gpu, fps, idleCheck } = parseCli(argv);
  let preregistration = null;
  if (!values.smoke) {
    const file = path.resolve(repo, values.preregistered);
    preregistration = { file: path.relative(repo, file), sha256: sha(await fs.readFile(file)) };
  }
  const outRoot = path.resolve(repo, values['out-root']);
  if (!outRoot.startsWith(path.join(repo, 'build') + path.sep)) throw new Error('--out-root must be under build/');
  const engineDir = path.resolve(repo, values['engine-dir']);
  const out = path.join(outRoot, values.label);
  await fs.mkdir(outRoot, { recursive: true });
  await fs.mkdir(out, { recursive: false });
  await fs.mkdir(path.join(out, 'runs'));

  // default: the first N recorded frames at their recorded timestamps; --fps: N slots of a regular grid over the whole recording (withRegularFps)
  const recorded = fps === null ? null : await loadFixture(values.scenario, 0);
  const loadInput = async (frames) => (fps === null ? loadFixture(values.scenario, frames) : withRegularFps(recorded, fps, frames));  // exactly `frames` frames
  const data = await loadInput(N);
  const replayData = replayFrames === N ? data : await loadInput(replayFrames);
  const ctx = { data, engineDir, out, m, nFrames: N, gpu, idleCheck,
    inputScript: await fs.readFile(path.join(here, 'sample-input.js'), 'utf8'), instrumentScript: await fs.readFile(path.join(here, 'r1-instrument.js'), 'utf8') };
  const order = Array.from({ length: rounds }, () => ['main', 'worker']).flat();
  const meta = { schema: 'slam-refact-r1-engine-worker-ab-v2', label: values.label, argv: process.argv, smoke: values.smoke, mode: values.mode,
    ...(values.smoke ? { judgment: 'discard', note: 'smoke run of the harness on a tiny input; these numbers are not evidence' } : { preregistration }),
    startedAt: new Date().toISOString(), node: process.version, loadAvgAtStart: os.loadavg(), cpus: os.cpus().length,
    inputKind: `SYNTHETIC/RECORDED desktop input (flip6-s2 frames + synthetic DeviceMotion), headless Chromium + ${gpu === 'hardware' ? 'hardware GPU (ANGLE gl-egl)' : 'SwiftShader'}; not a phone; desktop-headless only`,
    m, n: N, windowFrames: N, replayFrames, traceCategories: TRACE_CATEGORIES, order,
    input: { scenario: values.scenario, frames: data.frames.length, n: N, m, windowFrames: N, inputSha256: data.fixtureSha256, recordSha256: data.sourceSha256, nominalFps: data.nominalFps,
      fps: fps === null ? { requested: null, mode: 'recorded-timestamps' } : { requested: fps, ...data.fpsReport, replay: replayData.fpsReport } },
    chunkOptions: 'deliveryDiagnostics{enabled,maxEntries:1024} in both arms (the delivery records give the source frame of each pose)',
    engineManifest: JSON.parse(await fs.readFile(path.join(engineDir, 'manifest.json'), 'utf8')).artifacts,
    provenance: { ...(await provenance(engineDir)), gpu: { mode: gpu, launchArgs: launchArgsFor(gpu), renderers: [] }, fps: fps === null ? { requested: null, mode: 'recorded-timestamps', nominalFps: data.nominalFps } : { requested: fps, intervalMs: 1000 / fps } },
    idle: { enabled: idleCheck, rule: IDLE_RULE, runs: [] }, runs: [] };
  try {
    const perfRuns = [], idRuns = [];
    let abort = null;
    // every run: idle gate -> run -> record loads / GPU; an invalid GPU (R1-INFRA) stops the invocation, an invalid idle only skips that run
    const slot = async (mode, n, arm, run) => {
      const idle = idleCheck ? await gateIdle() : { enabled: false };
      idle.loadAtStart = os.loadavg();
      const rec = idle.enabled && !idle.ok ? await skippedRun({ out, n, arm, mode, idle }) : await run(idle);
      meta.runs.push({ mode, n, arm, status: rec.status, invalid: rec.invalid ?? null, file: `runs/${mode}-${n}-${arm}.json` });
      const { checks, ...idleSummary } = rec.idle ?? idle;
      meta.idle.runs.push({ mode, n, arm, ...idleSummary });
      if (rec.gpu) meta.provenance.gpu.renderers.push({ mode, n, arm, renderer: rec.gpu.renderer, valid: rec.gpu.valid });
      if (rec.invalid === 'R1-INFRA') abort = { reason: 'R1-INFRA', mode, n, arm, detail: rec.gpu?.reason ?? null };
      return rec;
    };
    if (values.mode !== 'identity') for (const [n, arm] of order.entries()) { if (abort) break; perfRuns.push(await slot('perf', n, arm, (idle) => perfRun({ ...ctx, arm, n, idle }))); }
    if (values.mode !== 'perf') for (const [n, arm] of order.entries()) { if (abort) break; idRuns.push(await slot('identity', n, arm, (idle) => replayRun({ ...ctx, data: replayData, arm, n, frames: replayData.frames.length, idle }))); }
    meta.provenance.gpu.rendererDistinct = [...new Set(meta.provenance.gpu.renderers.map((r) => r.renderer))];
    if (abort) meta.aborted = abort;
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
    meta.status = abort ? 'ABORTED_R1_INFRA' : [...perfRuns, ...idRuns].every((r) => r.status === 'COMPLETED') ? 'COMPLETED' : 'COMPLETED_WITH_ERRORS';
    meta.finishedAt = new Date().toISOString();
    await fs.writeFile(path.join(out, 'result.json'), JSON.stringify({ ...meta, result }, null, 1) + '\n');
    console.log(JSON.stringify({ label: values.label, smoke: values.smoke, status: meta.status, out: path.relative(repo, out), harnessSha256: meta.provenance.harnessSha256,
      gpu: { mode: gpu, renderers: meta.provenance.gpu.rendererDistinct }, fps: meta.input.fps.requested, idle: meta.idle.runs.map((r) => ({ ok: r.ok ?? null, waitedMs: r.waitedMs ?? null })), aborted: meta.aborted ?? null,
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
async function withPage({ data, engineDir, arm, rec, gpu, idle, extraRoutes = {}, scripts }, body) {
  let server, browser;
  rec.idle = idle;
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
    const { chromium } = await import('../../../scripts/dev/node_modules/playwright/index.mjs');  // lazy: the pure helpers above import without it
    const args = launchArgsFor(gpu);
    browser = await chromium.launch({ executablePath: process.env.BROWSER_EXECUTABLE || '/usr/bin/google-chrome', headless: true, args });
    rec.browserVersion = browser.version();
    const context = await browser.newContext({ viewport: { width: data.width, height: data.height }, permissions: ['camera', 'microphone'], isMobile: true,
      hasTouch: true, deviceScaleFactor: 1, userAgent: UA[data.ua] });
    // the GPU the page will run on: the WebGL2 renderer of a throwaway page of this browser, read before the run's own page exists
    const probe = await context.newPage();
    const probed = await probe.evaluate(() => {
      const gl = document.createElement('canvas').getContext('webgl2');
      if (!gl) return { webgl2: false, renderer: null, vendor: null };
      const ext = gl.getExtension('WEBGL_debug_renderer_info');
      return { webgl2: true, renderer: ext ? gl.getParameter(ext.UNMASKED_RENDERER_WEBGL) : null, vendor: ext ? gl.getParameter(ext.UNMASKED_VENDOR_WEBGL) : null };
    });
    await probe.close();
    const verdict = rendererVerdict(gpu, probed.renderer);
    rec.gpu = { mode: gpu, launchArgs: args, ...probed, valid: verdict.valid, reason: verdict.reason };
    if (!verdict.valid) { rec.status = 'INVALID_R1_INFRA'; rec.invalid = 'R1-INFRA'; return; }  // the run does not continue
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
    idle.loadAtEnd = os.loadavg();
  }
}

const newRec = (n, arm, mode) => ({ mode, n, arm, status: 'FAILED', invalid: null, server: null, blockedExternal: [], pageErrors: [], consoleErrors: [], requestFailures: [] });

// a run that was not executed because the machine was not idle (INVALID_IDLE); its record is kept
async function skippedRun({ out, n, arm, mode, idle }) {
  const rec = newRec(n, arm, mode);
  rec.status = 'INVALID_IDLE'; rec.invalid = 'R1-IDLE'; rec.idle = idle;
  await writeRun(out, `${mode}-${n}-${arm}`, rec);
  return rec;
}

// ---- LIVE run ----
async function perfRun({ data, engineDir, out, m, nFrames, arm, n, inputScript, instrumentScript, gpu, idle }) {
  const rec = newRec(n, arm, 'perf');
  rec.primary = null; rec.summary = null; rec.outputs = null;
  await withPage({ data, engineDir, arm, rec, gpu, idle, scripts: [OPTIONS_SCRIPT, inputScript, instrumentScript] }, async ({ page, browser, origin }) => {
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
async function replayRun({ data, engineDir, out, arm, n, frames, gpu, idle }) {
  const rec = newRec(n, arm, 'identity');
  rec.digest = null; rec.count = null;
  await withPage({ data, engineDir, arm, rec, gpu, idle, scripts: [OPTIONS_SCRIPT],
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

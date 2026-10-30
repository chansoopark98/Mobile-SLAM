// us025-lc-e2e-gate.v1 runner. FORK of slam_refact/tools/gates/us026-ba-e2e-v2/gate_e2e.mjs (sha256 of the fork source is recorded in the registration
// document). What changed against that file, and why (registration document section 9):
//   L1  one gate (key `lc`), version 1, gate id us025-lc-e2e-gate; no `mse`, no dev kind (this gate never runs a dev scenario; `precheck` is the no-marker smoke)
//   L2  per scenario: R original lc=1 runs (ON and OFF-versus-ON arms) and N original lc=0 runs (NULL arm) instead of two arms; every original run is
//       NEW (recorded after the marker) by run_lcgold.mjs under build/slam-refact-pkgh-20261022; nothing recorded earlier is reused
//   L3  comparator compare_us025.py (one process per run and arm, so a crash is a TOOL_ERROR of that run, never a FAIL) and judge.mjs decide
//   L4  the pinned replay binary is slam_refact_replay_lc6a (lc6a_replay_main.cc: replay_main.cc plus a linker --wrap capture of map_builder::LoopClose);
//       engine replays run with a scrubbed environment (PATH only) so no experiment variable can reach them (final-a-review-harness M-6)
//   L5  the hold also refuses (final-a-review-harness M-4, M-5): any PRECHECK_* variable in the environment, a non-empty `git status` of the F paths
//       (checked here, in code, and again by run.sh), an engine pin that is PENDING-PIN, an engine source that differs from the pinned commit
//   L6  per-run infrastructure failure (the original SDK hangs or fails: INFRA) is recorded and the scenario goes on; judge.mjs applies the registered rule
//
//   node gate_lc.mjs precheck --scenario S <GATE ARGS>        every HELD check, no marker, nothing written (the dry smoke)
//   node gate_lc.mjs run --scenario S <GATE ARGS> --out-root <dir> --run-log <file>
//   node gate_lc.mjs verdict --manifest <manifest> --reports <report.json>,...    exit 0 PASS, 1 FAIL, 4 BLOCKED/UNEVALUATED, 6 INFRA, 7 INSUFFICIENT, 8 INVALID
//   node gate_lc.mjs audit <GATE ARGS without --designation-env>
//   node gate_lc.mjs pin --manifest <manifest>      |      node gate_lc.mjs print-pins
//   GATE ARGS: --prereg-dir <dir> --gate-id <id> --gate-version <N> --designation-env <NAME>
// Exit codes of `run`: 0 report written (any class), 3 HELD (no marker), 5 resources not ready before the marker (no marker), 6 infrastructure failure after
// the marker (the run is used; coordinator decision + new version), 1 other error.
//
// WHAT THIS IS NOT: an accident-prevention device, not an authentication (protocol 12.6). Whoever can write the preregistration directory and set the
// environment variable can satisfy every check; the marker can be deleted by hand.
import fs from 'node:fs';
import os from 'node:os';
import path from 'node:path';
import { spawn, spawnSync } from 'node:child_process';
import { createHash } from 'node:crypto';
import { parseArgs } from 'node:util';
import { fileURLToPath } from 'node:url';
import { SR_HELD_FILE, claim, flatPins, gateArgs, gateMdPath, manifestPath, releaseCheck, request, sr } from '../../../web-chunk/tools/gate-common/held.mjs';
import { RunLog, envRecord } from '../../../web-chunk/tools/gate-common/runlog.mjs';
import { copyInto, makeReadOnly, readOnlyFiles, writeSums } from '../../../web-chunk/tools/gate-common/freeze.mjs';
import { JUDGE_CONSTANTS, SCENARIO_RUNS, VERDICT_EXIT, aggregate, classifyLc0Run, classifyLc1Run, classifyScenario } from './judge.mjs';

const here = path.dirname(fileURLToPath(import.meta.url));
export const root = path.resolve(here, '../../../..');
const rel = (p) => path.relative(root, path.resolve(root, p));
const abs = (p) => path.resolve(root, p);

// ---- fixed configuration (the manifest records the same values; a gate run refuses a manifest that differs) ------------------------------
export const GATE_ID = 'us025-lc-e2e-gate';
export const GATE_VERSION = 1;
export const DEV_SCENARIOS = ['flip6-n1', 'flip6-arcore-s3-absolute'];   // never gate scenarios (they were used for the LC-0 survey and the LC-G goldens)
export const LOAD_MAX = 8;                // 1-min loadavg ceiling right before every original run (lc6a precedent: runs started below 8; stricter than ba-e2e's 12)
export const WAIT_MAX_S = 4 * 3600;       // longest wait for resources (before the marker: exit 5; after it: that run is INFRA and the scenario stops)
export const POLL_S = 30;
export const REPLAY_JOBS = 2;
export const LOCK = 'build/c1-chromium.lock';
export const PRECHECK = 'slam_refact/docs/design/tools/lane-precheck.sh';
export const RUNNER = 'slam_refact/tools/re/runtime/us025/run_lcgold.mjs';
export const MAKE_BUNDLES = 'slam_refact/tools/harness/us026/make_bundles.sh';
export const COMPARE = 'slam_refact/tools/harness/us025/compare_us025.py';
export const PIN_DIR = 'build/slam-refact-pkgh-20261022/us025/lc-e2e-v1-pin';
export const REPLAY_BIN = `${PIN_DIR}/native/slam_refact_replay_lc6a`;
export const BASES = 'build/slam-refact-full-20261019/us026/gate-inputs/bundles-base';
export const OUT_ALLOWED = 'build/slam-refact-pkgh-20261022';   // run_lcgold.mjs writes only below it
export const OUT_DEFAULT = `${OUT_ALLOWED}/us025/gate-runs/${GATE_ID}-v${GATE_VERSION}`;
export const RUN_OPTIONS_LC1 = ['--memprobe', '--memprobe-lc', '--memprobe-detail', '12', '--record-inputs', '--lcgold'];
export const RUN_OPTIONS_LC0 = ['--memprobe', '--memprobe-lc', '--memprobe-detail', '12', '--record-inputs'];
export const FREEZE_PATHS = ['slam_refact/engine', 'slam_refact/web-chunk', 'slam_refact/web-smoke', 'slam_refact/tools', 'wasm', 'src'];
export const ENGINE_BASE_COMMIT = '0a5c5c4';   // the commit that added loop close; the registration records the engine diff since it
export const ON_PARAMS = 'MapBuilder.loopCloseEnabled=b:1';
export const PENDING = 'PENDING-PIN';
// Watchdog for the ORIGINAL SDK runs (prereview C1). A hung original run (renderer main thread at ~98% CPU, page.evaluate never returns: us025-lcg-golden.md 9)
// would hold the Chromium lock and the one-run marker forever. Fixed before any result: a run that exceeds RUN_WALL_MAX_S after its runner started gets its
// hung renderer(s) killed (the runner then records FAILED, as in LC-G); if the runner is still alive WATCHDOG_GRACE_S later its whole process group is killed.
// Any run the watchdog touched is INFRA of that run. 600 s = about 2.5 x the longest COMPLETED dev run (233 s, flip6-s3-lowtex at load 12; others <= 143 s).
export const RUN_WALL_MAX_S = 600;
export const WATCHDOG_GRACE_S = 120;
export const RENDERER_CPU_MIN = 90;
// Engine replays (prereview C2): deterministic, so an abnormal exit is a FAIL (US025-M1/M2) unless the replay log shows a harness cause. SIGKILL (OOM killer)
// is retried ONCE with the same bundle before it is judged.
export const HARNESS_SIGNATURES = ['ENOSPC', 'No space left on device'];
export const REPLAY_RETRY_ON_SIGKILL = 1;
// Start condition of run.sh (prereview C4, coordinator decision): disk usage of / at the start of a real run, in percent. The in-run precheck limit stays 95.
export const START_DISK_PCT_MAX = 90;

// files whose sha256 a gate run pins (repo-relative; held.pins keys them by base name)
export const PINNED_FILES = () => [
  'slam_refact/tools/gates/us025-lc-e2e-v1/gate_lc.mjs', 'slam_refact/tools/gates/us025-lc-e2e-v1/judge.mjs', 'slam_refact/tools/gates/us025-lc-e2e-v1/run.sh',
  COMPARE, 'slam_refact/tools/harness/us026/compare_us026.py',
  MAKE_BUNDLES, 'slam_refact/tools/harness/us026/add_params.py', 'slam_refact/tools/harness/divergence/make_variant.py',
  RUNNER, 'slam_refact/tools/re/runtime/us025/lcgold.js', 'slam_refact/tools/re/runtime/page-runtime.js',
  'slam_refact/tools/re/runtime/memprobe.js', 'slam_refact/tools/re/runtime/input-recorder.js', 'slam_refact/tools/re/runtime/capnp-event-queue.mjs',
  'slam_refact/tools/re/runtime/scenarios.json', 'slam_refact/tools/re/runtime/us026/config-poke.js', 'scripts/dev/references/xr_reference/sensor_adapter.mjs',
  'scripts/dev/replay-android-input.mjs', 'assets/references/xr_reference/analysis-runtime/standalone/xr.js',
  'assets/references/xr_reference/analysis-runtime/standalone/xr-slam.js', PRECHECK,
  'slam_refact/web-chunk/tools/gate-common/held.mjs', 'slam_refact/web-chunk/tools/gate-common/runlog.mjs', 'slam_refact/web-chunk/tools/gate-common/freeze.mjs',
];
// the replay binary is pinned separately: its sha256 is part of the engine pin (manifest engine_pin), PENDING-PIN until the coordinator fills it
export const pinnedFilesWithBinary = (withBinary) => (withBinary ? [...PINNED_FILES(), REPLAY_BIN] : PINNED_FILES());

// ---- hashing -------------------------------------------------------------------------------------------------------------------------------
export function sha256File(file) {
  const h = createHash('sha256');
  const fd = fs.openSync(file, 'r');
  try {
    const buf = Buffer.allocUnsafe(1 << 20);
    for (let n = fs.readSync(fd, buf, 0, buf.length, null); n > 0; n = fs.readSync(fd, buf, 0, buf.length, null)) h.update(buf.subarray(0, n));
  } finally { fs.closeSync(fd); }
  return h.digest('hex');
}
const sha256 = (s) => createHash('sha256').update(s).digest('hex');

// package tree sha256 with the definition of web-chunk/tests/chunk-hold.mjs treeSha256 (sorted names, symlink = its target, no __pycache__ / *.pyc)
export function pkgTreeSha(dir) {
  const lines = [];
  const walk = (d, r0) => {
    for (const e of fs.readdirSync(d, { withFileTypes: true }).sort((a, b) => (a.name < b.name ? -1 : a.name > b.name ? 1 : 0))) {
      const r = r0 ? `${r0}/${e.name}` : e.name;
      if (e.name === '__pycache__' || e.name.endsWith('.pyc')) continue;
      const full = path.join(d, e.name);
      if (e.isDirectory()) walk(full, r);
      else if (e.isSymbolicLink()) lines.push(`${r}\0symlink:${fs.readlinkSync(full)}`);
      else if (e.isFile()) lines.push(`${r}\0${sha256File(full)}`);
    }
  };
  walk(dir, '');
  return sha256(lines.join('\n'));
}

const which = (cmd) => {
  const r = spawnSync('bash', ['-c', `command -v ${cmd}`], { encoding: 'utf8' });
  return r.status === 0 ? r.stdout.trim() : null;
};

export function environmentFiles() {
  const py = which('python3');
  const chromeCmd = fs.realpathSync('/usr/bin/google-chrome');
  const wrapper = fs.readFileSync(chromeCmd).subarray(0, 2).toString() === '#!';
  return { node: process.execPath, python: py ? fs.realpathSync(py) : null, chromeWrapper: wrapper ? chromeCmd : null,
    chrome: wrapper ? path.join(path.dirname(chromeCmd), 'chrome') : chromeCmd };
}

export function liveTrees() {
  const m = abs('scripts/dev/node_modules');
  return {
    playwright: sha256(`playwright=${pkgTreeSha(path.join(m, 'playwright'))}\nplaywright-core=${pkgTreeSha(path.join(m, 'playwright-core'))}`),
    engine_standalone: pkgTreeSha(abs('assets/references/xr_reference/analysis-runtime/standalone')),
  };
}

export function versions() {
  const v = (cmd, args) => { const r = spawnSync(cmd, args, { encoding: 'utf8' }); return r.status === 0 ? (r.stdout + r.stderr).trim() : 'unavailable'; };
  return { node: process.version, python: v('python3', ['--version']), chrome: v('/usr/bin/google-chrome', ['--version']),
    playwright: JSON.parse(fs.readFileSync(abs('scripts/dev/node_modules/playwright/package.json'), 'utf8')).version };
}

// [absolute path, sha256] of every pinned file: the repo files, the device this process loaded, and the executables
export function livePinnedFiles({ withBinary }) {
  const files = pinnedFilesWithBinary(withBinary).map((f) => [abs(f), sha256File(abs(f))]);
  files.push([SR_HELD_FILE, sha256File(SR_HELD_FILE)]);
  const env = environmentFiles();
  for (const k of ['node', 'python', 'chromeWrapper', 'chrome']) if (env[k]) files.push([env[k], sha256File(env[k])]);
  return files;
}

export function baseDirOf(scenario) {
  const d = path.join(abs(BASES), scenario);
  const subs = fs.existsSync(d) ? fs.readdirSync(d).filter((n) => fs.statSync(path.join(d, n)).isDirectory()) : [];
  if (subs.length !== 1) throw new Error(`${rel(d)}: expected exactly one base bundle directory, found ${subs.length}`);
  return path.join(d, subs[0]);
}

export function baseSensorsSha(baseDir) {
  const b = JSON.parse(fs.readFileSync(path.join(baseDir, 'bundle.json'), 'utf8'));
  const f = abs(b.sensors.file);
  return sha256(spawnSync('gzip', ['-dc', f], { maxBuffer: 1 << 30 }).stdout);
}

export function scenarioRecord(scenario) {
  const scen = JSON.parse(fs.readFileSync(abs('slam_refact/tools/re/runtime/scenarios.json'), 'utf8'))[scenario];
  if (!scen) throw new Error(`unknown scenario ${scenario}`);
  const base = baseDirOf(scenario);
  const t = sr.treeSha(abs(scen.dataset));
  if (t.code) throw new Error(`dataset ${scen.dataset}: ${t.code}`);
  const bt = sr.treeSha(base);
  if (bt.code) throw new Error(`base ${rel(base)}: ${bt.code}`);
  return { name: scenario, dataset_dir: scen.dataset, dataset_tree_sha256: t.sha, kind: scen.kind, scale: scen.scale, ua: scen.ua,
    base_dir: rel(base), base_tree_sha256: bt.sha, sensors_sha256_uncompressed: baseSensorsSha(base), runs: SCENARIO_RUNS[scenario] ?? null };
}

export const decisionConstants = (python = 'python3') => {
  const r = spawnSync(python, ['-B', '-I', abs(COMPARE), 'constants'], { encoding: 'utf8' });
  if (r.status !== 0) throw new Error(`compare_us025.py constants failed: ${r.stderr}`);
  return { compare_us025: JSON.parse(r.stdout), judge: JUDGE_CONSTANTS, verdict_exit: VERDICT_EXIT };
};

export const configRecord = () => ({ scenario_runs: SCENARIO_RUNS, run_options_lc1: RUN_OPTIONS_LC1, run_options_lc0: RUN_OPTIONS_LC0, on_params: ON_PARAMS,
  load_max: LOAD_MAX, wait_max_s: WAIT_MAX_S, replay_jobs: REPLAY_JOBS, lock: LOCK, precheck_env: { PRECHECK_MAX_HEADLESS: '0', PRECHECK_MAX_BUILDS: '0' },
  out_default: OUT_DEFAULT, replay_env: ['PATH'], run_wall_max_s: RUN_WALL_MAX_S, watchdog_grace_s: WATCHDOG_GRACE_S, renderer_cpu_min: RENDERER_CPU_MIN,
  harness_signatures: HARNESS_SIGNATURES, replay_retry_on_sigkill: REPLAY_RETRY_ON_SIGKILL, start_disk_pct_max: START_DISK_PCT_MAX });

const held = (check, why) => new Error(`HELD: ${check}: ${why}`);
const canonJson = (v) => JSON.stringify(v, (k, x) => (x && typeof x === 'object' && !Array.isArray(x) ? Object.fromEntries(Object.keys(x).sort().map((y) => [y, x[y]])) : x));

// ---- git facts the hold needs (injectable for tests) ---------------------------------------------------------------------------------------
export const gitReal = (...args) => { const r = spawnSync('git', args, { cwd: root, encoding: 'utf8' }); return { status: r.status, out: (r.stdout ?? '').trim(), err: (r.stderr ?? '').trim() }; };

export function freezeDirty(git = gitReal) {
  const r = git('status', '--porcelain', '--', ...FREEZE_PATHS);
  if (r.status !== 0) return `git status failed: ${r.err}`;   // a git failure is not "clean"
  return r.out;
}

// the engine pin: the commit the replay binary was built from. The engine SOURCE (everything under slam_refact/engine except tests) at HEAD must equal it.
export function checkEnginePin(pin, { git = gitReal, binarySha = () => sha256File(abs(REPLAY_BIN)) } = {}) {
  if (!pin || typeof pin !== 'object') throw held('pins', 'manifest engine_pin is missing');
  if (pin.commit === PENDING || pin.replay_binary_sha256 === PENDING) throw held('pins', `manifest engine_pin is ${PENDING}: the coordinator must fill it (make_lc_manifest.mjs --engine-commit <sha> --replay-binary <path>) before a real run`);
  if (!/^[0-9a-f]{40}$/.test(pin.commit ?? '')) throw held('pins', `engine_pin.commit ${pin.commit} is not a 40-hex commit`);
  if (!/^[0-9a-f]{64}$/.test(pin.replay_binary_sha256 ?? '')) throw held('pins', 'engine_pin.replay_binary_sha256 is not a sha256');
  const anc = git('merge-base', '--is-ancestor', pin.commit, 'HEAD');
  if (anc.status !== 0) throw held('pins', `engine_pin.commit ${pin.commit} is not an ancestor of HEAD`);
  const diff = git('diff', '--name-only', pin.commit, 'HEAD', '--', 'slam_refact/engine', ':(exclude)slam_refact/engine/tests');
  if (diff.status !== 0) throw held('pins', `git diff failed: ${diff.err}`);
  if (diff.out) throw held('pins', `engine sources changed since the pinned commit ${pin.commit}: ${diff.out.split('\n').slice(0, 5).join(', ')}`);
  let live;
  try { live = binarySha(); } catch (error) { throw held('pins', `replay binary ${REPLAY_BIN}: ${error.message}`); }
  if (live !== pin.replay_binary_sha256) throw held('pins', `replay binary sha256 ${live} differs from the pinned ${pin.replay_binary_sha256}`);
  return true;
}

// ---- the hold (everything before the marker) ------------------------------------------------------------------------------------------------
export function checkHold({ gate, scenario, outDir, env = process.env, git = gitReal, engineCheck = checkEnginePin }) {
  if (gate.gateId !== GATE_ID || gate.version !== GATE_VERSION) throw held('args', `this runner is ${GATE_ID}.v${GATE_VERSION}, not ${gate.gateId}.v${gate.version}`);
  if (DEV_SCENARIOS.includes(scenario)) throw held('args', `${scenario} is a dev scenario`);
  if (!SCENARIO_RUNS[scenario]) throw held('args', `${scenario} is not a scenario of this gate`);
  for (const k of ['BROWSER_EXECUTABLE', 'NODE_OPTIONS']) if (env[k]) throw held('env', `${k} is set (the pinned chrome command and node are used as they are)`);
  const pre = Object.keys(env).filter((k) => k.startsWith('PRECHECK_'));
  if (pre.length) throw held('env', `PRECHECK_* is set (${pre.join(', ')}): the resource thresholds are not steerable in a gate run (run.sh removes them; remove them here)`);
  const dirty = freezeDirty(git);
  if (dirty) throw held('dirty', `git status of the freeze paths is not empty (commit or restore first):\n${dirty.split('\n').slice(0, 10).join('\n')}`);
  const mf = manifestPath(gate);
  if (!fs.existsSync(mf)) throw held('manifest', `${rel(mf)} does not exist`);
  let manifest;
  try { manifest = JSON.parse(fs.readFileSync(mf, 'utf8')); } catch (error) { throw held('manifest', `not JSON: ${error.message}`); }
  const reg = manifest.registered_run ?? {};
  const datum = (reg.scenarios ?? []).find((s) => s.name === scenario);
  if (!datum) throw held('data', `${scenario} is not a registered scenario of ${GATE_ID}.v${GATE_VERSION}`);
  if (canonJson(reg.config) !== canonJson(configRecord())) throw held('config', 'manifest registered_run.config differs from the runner configuration');
  engineCheck(manifest.engine_pin);
  const env0 = environmentFiles();
  if (!env0.python) throw held('pins', 'python3 not found');
  let constants;
  try { constants = decisionConstants(env0.python); } catch (error) { throw held('constants', error.message); }
  if (canonJson(constants) !== canonJson(reg.decision_constants)) throw held('constants', 'compare_us025.py / judge.mjs decision constants differ from manifest registered_run.decision_constants');
  const files = livePinnedFiles({ withBinary: true });
  const recorded = { ...(manifest.pinned?.files ?? {}) };
  for (const [k, v] of Object.entries(manifest.pinned?.executables ?? {})) recorded[k] = v;
  let flat;
  try { flat = flatPins(files); } catch (error) { throw held('pins', error.message); }
  void flat;
  const recordedFlat = Object.fromEntries(Object.entries(recorded).map(([k, v]) => [path.basename(k), v]));
  if (canonJson(recordedFlat) !== canonJson(manifest.held?.pins ?? null)) throw held('pins', 'manifest held.pins is not the flat form of manifest.pinned.files + executables');
  if (process.version !== manifest.pinned?.versions?.node) throw held('pins', `node ${process.version} is not the pinned ${manifest.pinned?.versions?.node}`);
  const trees = liveTrees();
  for (const [k, v] of Object.entries(manifest.pinned?.trees ?? {})) if (trees[k] !== v) throw held('pins', `tree ${k} ${trees[k]} differs from the pinned ${v}`);
  if (Object.keys(trees).sort().join() !== Object.keys(manifest.pinned?.trees ?? {}).sort().join()) throw held('pins', 'pinned tree set differs');
  const live = scenarioRecord(scenario);
  for (const k of ['dataset_dir', 'dataset_tree_sha256', 'base_dir', 'base_tree_sha256', 'sensors_sha256_uncompressed']) {
    if (live[k] !== datum[k]) throw held('data', `${scenario} ${k} ${live[k]} differs from the registered ${datum[k]}`);
  }
  if (fs.existsSync(outDir)) throw held('out', `${rel(outDir)} exists (checked before the marker so that a wrong --out does not use the first run)`);
  const rq = request(gate, { kind: scenario, env: env[gate.designationEnv], pins: files.map(([f]) => f), dataDir: abs(datum.dataset_dir) });
  const outcome = releaseCheck(rq);
  const marker = path.join(gate.preregDir, sr.markerName(rq));
  if (fs.existsSync(marker)) throw held('first-run', `${rel(marker)} exists: the first real run of ${scenario} was already taken`);
  return { manifest, datum, rq, outcome, log: outcome.log, python: env0.python };
}

// ---- resources ---------------------------------------------------------------------------------------------------------------------------
export function resourceCheck() {
  const env = Object.fromEntries(Object.entries(process.env).filter(([k]) => !k.startsWith('PRECHECK_')));
  env.PRECHECK_MAX_HEADLESS = '0';
  env.PRECHECK_MAX_BUILDS = '0';
  const r = spawnSync('bash', [abs(PRECHECK), '--json'], { cwd: root, env, encoding: 'utf8' });
  let pre = null;
  try { pre = JSON.parse(r.stdout); } catch { pre = { unparsable: r.stdout.slice(0, 2000) }; }
  const load = os.loadavg();
  return { at: new Date().toISOString(), precheckExit: r.status, precheck: pre, loadavg: load, loadMax: LOAD_MAX, ok: r.status === 0 && load[0] <= LOAD_MAX };
}

const sleep = (s) => new Promise((resolve) => setTimeout(resolve, s * 1000));

async function waitResources(log, what) {
  const t0 = Date.now();
  let attempts = 0;
  for (;;) {
    const c = resourceCheck();
    attempts++;
    if (c.ok) return { ...c, attempts, waitedS: (Date.now() - t0) / 1000 };
    log?.write('resource-wait', { what, attempt: attempts, precheckExit: c.precheckExit, loadavg: c.loadavg, reasons: c.precheck?.blocked_reasons });
    if ((Date.now() - t0) / 1000 > WAIT_MAX_S) return { ...c, attempts, waitedS: (Date.now() - t0) / 1000, timedOut: true };
    await sleep(POLL_S);
  }
}

// ---- children ------------------------------------------------------------------------------------------------------------------------------
function runChild(cmd, args, { logFile, env = process.env } = {}) {
  return new Promise((resolve) => {
    const out = fs.openSync(logFile, 'wx');
    const t0 = Date.now();
    const p = spawn(cmd, args, { cwd: root, env, stdio: ['ignore', out, out] });
    p.on('close', (code, signal) => { fs.closeSync(out); resolve({ exit: code, signal, seconds: (Date.now() - t0) / 1000 }); });
    p.on('error', (error) => { try { fs.closeSync(out); } catch { /* closed */ } resolve({ exit: null, error: String(error), seconds: (Date.now() - t0) / 1000 }); });
  });
}

async function pool(items, jobs, fn) {
  const results = new Array(items.length);
  let next = 0;
  await Promise.all(Array.from({ length: Math.min(jobs, items.length) }, async () => {
    while (next < items.length) { const k = next++; results[k] = await fn(items[k]); }
  }));
  return results;
}

// One original run inside the Chromium lock: `flock LOCK node gate_lc.mjs _locked <job.json>`; the child checks the resources again INSIDE the lock, writes
// prerun/<run>.json and starts the runner, or exits 75 (not ready: the parent waits and retries, never relaxing).
async function lockedRun(job, log) {
  const t0 = Date.now();
  for (let attempt = 1; ; attempt++) {
    const jobFile = `${job.prerunFile}.job${attempt}.json`;
    fs.writeFileSync(jobFile, `${JSON.stringify(job)}\n`, { flag: 'wx' });
    const r = await runChild('flock', ['-w', String(WAIT_MAX_S), abs(LOCK), process.execPath, fileURLToPath(import.meta.url), '_locked', jobFile],
      { logFile: `${job.logFile}.attempt${attempt}` });
    if (r.exit !== 75) return { ...r, attempts: attempt };
    log?.write('resource-wait', { what: `inside lock before ${job.run}`, attempt });
    if ((Date.now() - t0) / 1000 > WAIT_MAX_S) return { ...r, attempts: attempt, timedOut: true };
    await sleep(POLL_S);
  }
}

// ps table: pid -> {ppid, etimes, pcpu, args}
export function parsePs(text) {
  const t = new Map();
  for (const line of text.split('\n')) {
    const p = line.trim().split(/\s+/);
    if (p.length < 5 || !/^\d+$/.test(p[0])) continue;
    t.set(Number(p[0]), { ppid: Number(p[1]), etimes: Number(p[2]), pcpu: Number(p[3]), args: p.slice(4).join(' ') });
  }
  return t;
}
const psTable = () => parsePs(spawnSync('ps', ['-eo', 'pid=,ppid=,etimes=,pcpu=,args='], { encoding: 'utf8' }).stdout ?? '');

// renderer processes that are descendants of rootPid and burn at least minCpu percent (the LC-6b watchdog.py rule, restricted to this run's own tree)
export function findHungRenderers(table, rootPid, minCpu) {
  const out = [];
  for (const [pid, v] of table) {
    if (!v.args.includes('--type=renderer') || v.pcpu < minCpu) continue;
    let q = v.ppid;
    for (let hop = 0; hop < 16 && table.has(q); hop++) { if (q === rootPid) { out.push(pid); break; } q = table.get(q).ppid; }
  }
  return out;
}

// Runs a child in its own process group under the wall-clock cap. Returns {status, signal, watchdog:{fired, killedRenderers, groupKilled}}.
export function supervise({ cmd, args, options = {}, capS, graceS, minCpu = RENDERER_CPU_MIN, ps = psTable }) {
  return new Promise((resolve) => {
    const child = spawn(cmd, args, { ...options, detached: true });
    const wd = { fired: false, killedRenderers: [], groupKilled: false };
    let done = false;
    const timers = [];
    timers.push(setTimeout(() => {
      if (done) return;
      wd.fired = true;
      for (const pid of findHungRenderers(ps(), child.pid, minCpu)) { try { process.kill(pid, 'SIGKILL'); wd.killedRenderers.push(pid); } catch { /* gone */ } }
      timers.push(setTimeout(() => {
        if (done) return;
        wd.groupKilled = true;
        try { process.kill(-child.pid, 'SIGKILL'); } catch { /* gone */ }
      }, graceS * 1000));
    }, capS * 1000));
    const finish = (status, signal, error) => { done = true; for (const t of timers) clearTimeout(t); resolve({ status, signal, watchdog: wd, error }); };
    child.on('close', (code, signal) => finish(code, signal));
    child.on('error', (error) => finish(null, null, String(error)));
  });
}

async function lockedChild(jobFile) {
  const job = JSON.parse(fs.readFileSync(jobFile, 'utf8'));
  const c = resourceCheck();
  if (!c.ok) { fs.writeFileSync(`${jobFile}.not-ready.json`, `${JSON.stringify(c)}\n`, { flag: 'wx' }); return 75; }
  fs.writeFileSync(job.prerunFile, `${JSON.stringify({ ...c, lock: LOCK, run: job.run })}\n`, { flag: 'wx' });
  const env = { ...process.env };
  delete env.BROWSER_EXECUTABLE;
  const opts = job.lc === 1 ? RUN_OPTIONS_LC1 : RUN_OPTIONS_LC0;
  const r = await supervise({ cmd: process.execPath, capS: RUN_WALL_MAX_S, graceS: WATCHDOG_GRACE_S,
    args: [abs(RUNNER), '--scenario', job.scenario, '--label', job.label, '--out-root', job.runsRoot, ...opts, '--poke', JSON.stringify({ lc: job.lc })],
    options: { cwd: root, env, stdio: 'inherit' } });
  if (r.watchdog.fired) fs.writeFileSync(`${job.prerunFile}.watchdog.json`, `${JSON.stringify({ ...r.watchdog, runnerStatus: r.status, runnerSignal: r.signal, capS: RUN_WALL_MAX_S })}\n`, { flag: 'wx' });
  return r.status ?? 1;
}

// an original run is usable when the runner exited 0, finished (COMPLETED, or COMPLETED_WITH_ERRORS which the original SDK shows after a loop close
// success) and wrote the recorded inputs and the memprobe file the comparator needs. Anything else is INFRA of that run (never a FAIL).
export function originalRunInfra({ runnerExit, status, hasInputs, timedOut = false, prerunOk = true, watchdog = null }) {
  if (timedOut || !prerunOk) return `resources or lock not available within ${WAIT_MAX_S} s`;
  if (watchdog?.fired) return `watchdog: the run exceeded ${RUN_WALL_MAX_S} s (killed renderers ${watchdog.killedRenderers?.length ?? 0}, group killed ${!!watchdog.groupKilled}); runner exit ${runnerExit}, status ${status}`;
  if (runnerExit !== 0) return `runner exit ${runnerExit}, status ${status}`;
  if (status !== 'COMPLETED' && status !== 'COMPLETED_WITH_ERRORS') return `original run status ${status}`;
  if (!hasInputs) return 'recorded-inputs.jsonl.gz or memprobe.jsonl.gz missing';
  return null;
}

const compactChecks = (res) => (res?.checks ?? []).map((c) => ({ name: c.name, compared: c.compared ?? null, match: c.match ?? null, mismatch: c.mismatch ?? null,
  declaredU2: c.declaredU2 ?? null, status: c.status ?? null, first: c.first ?? null, firstDifferenceFrame: c.firstDifferenceFrame ?? null }));
export const compactResult = (res) => (res ? { verdict: res.verdict, reason: res.reason ?? null, checks: compactChecks(res), detail: res.detail ?? null,
  firstDifferenceFrame: res.firstDifferenceFrame ?? null, successFrame: res.successFrame ?? null } : null);

// ---- one scenario -------------------------------------------------------------------------------------------------------------------------
const isoLocal = () => { const r = spawnSync('date', ['+%Y-%m-%dT%H:%M:%S%z'], { encoding: 'utf8' }); return r.stdout.trim(); };

export async function cmdRun(values) {
  const scenario = values.scenario;
  if (!/^[a-z0-9-]+$/.test(scenario ?? '')) throw new Error('--scenario <name>');
  const outRoot = abs(values['out-root'] ?? OUT_DEFAULT);
  if (!(outRoot + path.sep).startsWith(abs(OUT_ALLOWED) + path.sep)) throw new Error(`--out-root must be under ${OUT_ALLOWED} (run_lcgold.mjs writes only there)`);
  const outDir = path.join(outRoot, GATE_ID, scenario);
  if (!values['run-log']) throw new Error('--run-log <new file outside the output directory> is required');
  const log = new RunLog(abs(values['run-log']), { out: outDir });
  const startedAt = new Date().toISOString();
  log.write('start', { argv: process.argv.slice(2), node: process.version, gate: 'lc', scenario, env: envRecord(process.env, values['designation-env'] ? [values['designation-env']] : []) });
  let hold = null;
  try {
    const gate = gateArgs(values, root);
    try { hold = checkHold({ gate, scenario, outDir }); } catch (error) { log.write('held', { message: error.message }); throw error; }
    log.write('hold-checks', { lines: hold.log });
    const python = hold.python;
    const datum = hold.datum;
    const planned = SCENARIO_RUNS[scenario];
    const before = await waitResources(log, 'before the marker');
    log.write('resources-before-marker', { ok: before.ok, attempts: before.attempts, waitedS: before.waitedS, loadavg: before.loadavg, timedOut: before.timedOut ?? false });
    if (!before.ok) { log.close('NOT-READY'); return { exit: 5, why: 'resources not ready before the marker (no marker created)' }; }
    const outcome = claim(hold.rq);
    const marker = outcome.marker;
    log.write('first-run-claimed', { marker: rel(marker), scenario, designationSha256: outcome.designationSha256 });
    fs.mkdirSync(outDir, { recursive: true });
    for (const d of ['runs', 'bundles', 'replays', 'prerun', 'logs', 'compare']) fs.mkdirSync(path.join(outDir, d));
    // ---- the original runs: lc=1 runs first, then lc=0 runs; fixed order, fixed count, no re-run
    const plan = [...Array.from({ length: planned.lc1 }, (_, i) => ({ lc: 1, name: `gate-lc1-${String(i + 1).padStart(2, '0')}` })),
      ...Array.from({ length: planned.lc0 }, (_, i) => ({ lc: 0, name: `gate-lc0-${String(i + 1).padStart(2, '0')}` }))];
    const runs = {};
    let infraStop = null;
    for (const p of plan) {
      const rec = { name: p.name, lc: p.lc, dir: rel(path.join(outDir, 'runs', scenario, p.name)) };
      runs[p.name] = rec;
      if (infraStop) { rec.infra = `not started: ${infraStop}`; rec.notRun = true; continue; }
      const job = { scenario, run: p.name, label: p.name, lc: p.lc, runsRoot: rel(path.join(outDir, 'runs')),
        prerunFile: path.join(outDir, 'prerun', `${p.name}.json`), logFile: path.join(outDir, 'logs', `run-${p.name}.log`) };
      log.write('original-run-start', { run: p.name, lc: p.lc });
      const r = await lockedRun(job, log);
      const dir = abs(rec.dir);
      let status = 'NO-META';
      let loadAvgAtStart = null;
      try { const m = JSON.parse(fs.readFileSync(path.join(dir, 'meta.json'), 'utf8')); status = m.status; loadAvgAtStart = m.loadAvgAtStart ?? null; } catch { /* NO-META */ }
      const prerun = fs.existsSync(job.prerunFile) ? JSON.parse(fs.readFileSync(job.prerunFile, 'utf8')) : null;
      const hasInputs = fs.existsSync(path.join(dir, 'recorded-inputs.jsonl.gz')) && fs.existsSync(path.join(dir, 'memprobe.jsonl.gz'));
      rec.runnerExit = r.exit; rec.signal = r.signal ?? null; rec.seconds = r.seconds; rec.attempts = r.attempts; rec.status = status; rec.loadAvgAtStart = loadAvgAtStart;
      rec.prerun = prerun ? { ok: prerun.ok, at: prerun.at, loadavg: prerun.loadavg, precheckExit: prerun.precheckExit, file: rel(job.prerunFile) } : { ok: false };
      const wdFile = `${job.prerunFile}.watchdog.json`;
      rec.watchdog = fs.existsSync(wdFile) ? JSON.parse(fs.readFileSync(wdFile, 'utf8')) : null;
      rec.infra = originalRunInfra({ runnerExit: r.exit, status, hasInputs, timedOut: r.timedOut, prerunOk: !!prerun, watchdog: rec.watchdog });
      log.write('original-run-end', { run: p.name, exit: r.exit, status, seconds: r.seconds, infra: rec.infra, timedOut: r.timedOut ?? false });
      if (r.timedOut || !prerun) infraStop = `resources or lock not available for ${p.name}`;   // resources never ready: the remaining runs cannot start
    }
    // ---- exact-input bundles: one make_bundles.sh call per usable original run
    const benv = { ...process.env, US026_RUNS: rel(path.join(outDir, 'runs')), US026_BUNDLES: rel(path.join(outDir, 'bundles')), US026_BASE_ROOT: rel(path.join(outDir, 'bundles-base-unused')), US026_BASE: datum.base_dir };
    const jobs = [];
    for (const rec of Object.values(runs)) {
      if (rec.infra) continue;
      const variants = rec.lc === 1 ? [`on=${ON_PARAMS}`] : [];
      const r = await runChild('bash', [abs(MAKE_BUNDLES), scenario, rec.name, ...variants], { logFile: path.join(outDir, 'logs', `bundles-${rec.name}.log`), env: benv });
      log.write('bundles', { run: rec.name, exit: r.exit });
      rec.bundles = { makeExit: r.exit };
      const off = path.join(outDir, 'bundles', scenario, rec.name);
      const on = path.join(outDir, 'bundles', scenario, `${rec.name}-on`);
      if (r.exit !== 0 || !fs.existsSync(off) || (rec.lc === 1 && !fs.existsSync(on))) { rec.infra = `bundle construction failed (exit ${r.exit})`; continue; }
      jobs.push({ run: rec.name, arm: rec.lc === 1 ? 'off' : 'null', bundle: off });
      if (rec.lc === 1) jobs.push({ run: rec.name, arm: 'on', bundle: on });
    }
    // ---- engine replays with the pinned binary, environment scrubbed to PATH. SIGKILL is retried once (REPLAY_RETRY_ON_SIGKILL); a log with ENOSPC marks a harness cause.
    const replayEnv = { PATH: process.env.PATH ?? '/usr/bin:/bin' };
    const harnessCause = (file) => { try { const t = fs.readFileSync(file, 'utf8').slice(-20000); return HARNESS_SIGNATURES.some((x) => t.includes(x)); } catch { return false; } };
    const res = await pool(jobs, REPLAY_JOBS, async (j) => {
      let out = path.join(outDir, 'replays', `${j.run}-${j.arm}`);
      let logFile = path.join(outDir, 'logs', `replay-${j.run}-${j.arm}.log`);
      let r = await runChild(abs(REPLAY_BIN), ['--bundle', j.bundle, '--out', out], { logFile, env: replayEnv });
      let retried = 0;
      while (r.signal === 'SIGKILL' && retried < REPLAY_RETRY_ON_SIGKILL && !harnessCause(logFile)) {
        retried++;
        out = path.join(outDir, 'replays', `${j.run}-${j.arm}-retry${retried}`);
        logFile = path.join(outDir, 'logs', `replay-${j.run}-${j.arm}-retry${retried}.log`);
        r = await runChild(abs(REPLAY_BIN), ['--bundle', j.bundle, '--out', out], { logFile, env: replayEnv });
      }
      return { ...j, out, retried, harness: (r.exit !== 0) && harnessCause(logFile), ...r };
    });
    for (const r of res) { runs[r.run].replays ??= {}; runs[r.run].replays[r.arm] = { dir: rel(r.out), exit: r.exit, signal: r.signal ?? null, seconds: r.seconds, retried: r.retried, harness: r.harness }; }
    log.write('replays', { exits: Object.fromEntries(res.map((r) => [`${r.run}-${r.arm}`, r.exit])) });
    // ---- the comparator: one process per run and arm
    const cmp = async (rec, arm) => {
      const rp = rec.replays?.[arm];
      const outFile = path.join(outDir, 'compare', `${rec.name}-${arm}.json`);
      const r = await runChild(python, ['-B', '-I', abs(COMPARE), arm, '--original', abs(rec.dir), '--engine', abs(rp?.dir ?? 'does-not-exist'), '--out', outFile],
        { logFile: path.join(outDir, 'logs', `compare-${rec.name}-${arm}.log`) });
      let result = null;
      if (fs.existsSync(outFile)) { try { result = JSON.parse(fs.readFileSync(outFile, 'utf8')); } catch { result = null; } }
      // exit 4 (TOOL_ERROR), 1 (unhandled Python exception), a missing/garbled output file, or a verdict that disagrees with the exit code is a TOOL_ERROR
      // of this run and arm, never a FAIL (final-a-review-harness M-3)
      const EXIT_OF = { PASS: 0, PASS_DECLARED: 0, FAIL: 10, INVALID: 11, BLOCKED: 13 };
      if (!result || EXIT_OF[result.verdict] !== r.exit) return { verdict: 'TOOL_ERROR', reason: `compare_us025.py ${arm} exit ${r.exit}${result ? `, verdict ${result.verdict}` : ', no output'}`, checks: [] };
      return result;
    };
    const lc1 = [];
    const lc0 = [];
    for (const rec of Object.values(runs)) {
      if (rec.infra) {
        (rec.lc === 1 ? lc1 : lc0).push(rec.lc === 1 ? classifyLc1Run({ name: rec.name, infra: rec.infra }) : classifyLc0Run({ name: rec.name, infra: rec.infra }));
        continue;
      }
      if (rec.lc === 1) {
        // a replay that did not exit 0 is NOT compared: judge.mjs turns it into FAIL (US025-M1/M2) or, for a harness cause, INFRA
        const on = rec.replays?.on?.exit === 0 ? await cmp(rec, 'on') : null;
        const off = rec.replays?.off?.exit === 0 ? await cmp(rec, 'off') : null;
        rec.compare = { on: compactResult(on), off: compactResult(off) };
        lc1.push(classifyLc1Run({ name: rec.name, infra: null, replays: { on: rec.replays?.on, off: rec.replays?.off }, on: compactResult(on), off: compactResult(off) }));
      } else {
        const nl = rec.replays?.null?.exit === 0 ? await cmp(rec, 'null') : null;
        rec.compare = { null: compactResult(nl) };
        lc0.push(classifyLc0Run({ name: rec.name, infra: null, replay: rec.replays?.null, null: compactResult(nl) }));
      }
    }
    const result = classifyScenario({ scenario, lc1, lc0, planned, infraFailure: infraStop });
    const pins = Object.fromEntries(livePinnedFiles({ withBinary: true }).map(([f, s]) => [path.basename(f), s]));
    const report = { schema: 'slam-refact-us025-lc-e2e-report-v1', gate: { id: GATE_ID, version: GATE_VERSION, key: 'lc' }, kind: 'gate', scenario, startedAt,
      finishedAt: new Date().toISOString(), argv: process.argv.slice(2), versions: versions(), trees: liveTrees(), pins, marker: rel(marker), datum,
      resourcesBeforeMarker: { ok: before.ok, at: before.at, loadavg: before.loadavg, attempts: before.attempts, waitedS: before.waitedS },
      planned, runs, classified: { lc1, lc0 }, infraFailure: infraStop, result,
      gitHead: gitReal('rev-parse', 'HEAD').out, enginePin: hold.manifest.engine_pin };
    const mreg = hold.manifest.review ?? {};
    report.registeredCopies = copyInto(outDir, 'registered', [gateMdPath(gate), manifestPath(gate), path.join(gate.preregDir, mreg.designation_file), path.join(gate.preregDir, mreg.file), marker]);
    fs.writeFileSync(path.join(outDir, 'report.json'), `${JSON.stringify(report, null, 1)}\n`, { flag: 'wx' });
    const sums = writeSums(outDir);
    const entries = makeReadOnly(outDir);
    readOnlyFiles([gateMdPath(gate)]);
    log.write('finished', { report: rel(path.join(outDir, 'report.json')), result, sums, readOnlyEntries: entries, infraFailure: infraStop });
    log.close(infraStop ? 'INFRA-FAILURE' : 'OK');
    console.log(JSON.stringify({ scenario, result, report: rel(path.join(outDir, 'report.json')), infraFailure: infraStop }));
    return { exit: infraStop ? 6 : 0 };
  } catch (error) {
    if (!log.closed) { log.write('error', { message: String(error?.message ?? error) }); log.close(String(error?.message ?? error).startsWith('HELD:') ? 'HELD' : 'ERROR'); }
    throw error;
  }
}

function cmdVerdict(values) {
  const manifest = JSON.parse(fs.readFileSync(abs(values.manifest), 'utf8'));
  const reports = (values.reports ?? '').split(',').filter(Boolean).map((f) => JSON.parse(fs.readFileSync(abs(f), 'utf8')));
  const agg = aggregate(manifest, reports);
  console.log(JSON.stringify(agg, null, 1));
  return VERDICT_EXIT[agg.verdict];
}

function cmdPrecheck(values) {
  const gate = gateArgs(values, root);
  const outDir = path.join(abs(values['out-root'] ?? OUT_DEFAULT), GATE_ID, values.scenario);
  const hold = checkHold({ gate, scenario: values.scenario, outDir });
  console.log(`${hold.log.join('\n')}\nRELEASED (precheck: no marker created, nothing run) scenario ${values.scenario}`);
  return 0;
}

function cmdAudit(values) {
  const gate = gateArgs({ ...values, 'designation-env': 'UNUSED_BY_AUDIT' }, root);
  const manifest = JSON.parse(fs.readFileSync(manifestPath(gate), 'utf8'));
  let bad = 0;
  const kinds = manifest.held?.kinds ?? [];
  if (!kinds.length) { console.log('AUDIT FAILED: manifest held.kinds is empty'); return 3; }
  for (const kind of kinds) {
    const o = sr.audit({ preregDir: gate.preregDir, gateId: gate.gateId, version: gate.version, kind });
    console.log(`${kind}: ${o.ok ? 'AUDIT OK' : `AUDIT FAILED [${o.code}] ${o.why}`}`);
    if (!o.ok) bad++;
  }
  // the audit also compares the pins with the files NOW (sr_held's audit does not look at held.pins; final-a-review-harness M-1)
  const files = livePinnedFiles({ withBinary: !(manifest.engine_pin?.commit === PENDING) });
  const recorded = { ...(manifest.pinned?.files ?? {}) };
  for (const [k, v] of Object.entries(manifest.pinned?.executables ?? {})) recorded[k] = v;
  const recordedFlat = Object.fromEntries(Object.entries(recorded).map(([k, v]) => [path.basename(k), v]));
  const liveFlat = Object.fromEntries(files.map(([f, s]) => [path.basename(f), s]));
  const drift = Object.keys({ ...recordedFlat, ...liveFlat }).filter((k) => recordedFlat[k] !== liveFlat[k]);
  console.log(drift.length ? `PIN DRIFT since registration: ${drift.join(', ')}` : 'pins: all pinned files equal the registered sha256');
  return bad || drift.length ? 3 : 0;
}

function cmdPin(values) {
  const bytes = fs.readFileSync(abs(values.manifest));
  const parsed = sr.parseJson(bytes);
  if (parsed.err !== undefined) throw new Error(`manifest is not strict JSON: ${parsed.err}`);
  console.log(JSON.stringify({ gate_md_sha256: JSON.parse(bytes.toString('utf8')).sha256, manifest_before_review_sha256: sr.canonManifestSha(parsed.json) }, null, 1));
  return 0;
}

// the live values a registration manifest records. withBinary=false is the PENDING-PIN form: the replay binary is not pinned yet
export function livePins({ withBinary }) {
  const files = {};
  for (const f of pinnedFilesWithBinary(withBinary)) files[f] = sha256File(abs(f));
  files[rel(SR_HELD_FILE)] = sha256File(SR_HELD_FILE);
  const env = environmentFiles();
  const executables = {};
  for (const k of ['node', 'python', 'chromeWrapper', 'chrome']) if (env[k]) executables[env[k]] = sha256File(env[k]);
  const all = { ...files, ...executables };
  return { files, executables, trees: liveTrees(), versions: versions(), held_pins: Object.fromEntries(Object.entries(all).map(([k, v]) => [path.basename(k), v])),
    config: configRecord(), decision_constants: decisionConstants() };
}

const isMain = process.argv[1] && path.resolve(process.argv[1]) === fileURLToPath(import.meta.url);
if (isMain) {
  const [cmd, ...rest] = process.argv.slice(2);
  try {
    if (cmd === '_locked') {
      process.exitCode = await lockedChild(rest[0]);
    } else {
      const { values } = parseArgs({ args: rest, options: { scenario: { type: 'string' }, 'out-root': { type: 'string' }, 'run-log': { type: 'string' }, 'prereg-dir': { type: 'string' },
        'gate-id': { type: 'string' }, 'gate-version': { type: 'string' }, 'designation-env': { type: 'string' }, manifest: { type: 'string' }, reports: { type: 'string' } } });
      if (cmd === 'run') process.exitCode = (await cmdRun(values)).exit;
      else if (cmd === 'precheck') process.exitCode = cmdPrecheck(values);
      else if (cmd === 'verdict') process.exitCode = cmdVerdict(values);
      else if (cmd === 'audit') process.exitCode = cmdAudit(values);
      else if (cmd === 'pin') process.exitCode = cmdPin(values);
      else if (cmd === 'print-pins') console.log(JSON.stringify(livePins({ withBinary: fs.existsSync(abs(REPLAY_BIN)) }), null, 1));
      else { console.error('usage: gate_lc.mjs precheck|run|verdict|audit|pin|print-pins ... (see the header)'); process.exitCode = 2; }
    }
  } catch (error) {
    console.error(String(error?.message ?? error));
    process.exitCode = String(error?.message).startsWith('HELD:') ? 3 : 1;
  }
}

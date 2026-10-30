// FORK for us026-ba-e2e-gate.v2 of slam_refact/tools/harness/us026/gate_e2e.mjs (the runner of ba-e2e v1 / mse-e2e v1, sha256 04f0321e...f060).
// The v1 runner hard-codes version 1, the v1 replay binary and the v1 run.sh path, so v2 cannot use it unchanged. The ONLY differences from the
// v1 runner (gate document us026-ba-e2e-gate.v2.md section 2.3, "changes from v1"):
//   D1  GATES.ba.version 1 -> 2
//   D2  REPLAY_BIN: the pinned snapshot build of git HEAD 19fd011 (engine fix US026-CC-1) instead of the v1 binary 24fdba2a
//   D3  PINNED_FILES: this fork and the v2 run.sh replace the v1 gate_e2e.mjs / run.sh paths (ba only; the mse entries are untouched)
//   D4  engineOnExitRule(): the v1 pre-declared condition US026B-M1 moved into code (an engine BA-on replay that exits abnormally while the
//       BA-off and null replays exit 0 is a FAIL of the engine, not INVALID); applied to the scenario result after compare_us026.py
//   D5  header comments
// The decision code (compare_us026.py), the arms, bundles, tolerances, load limit, wait limit and every other line are the v1 text.
// Original header follows.
//
// US-026 end-to-end gate runner: us026-ba-e2e-gate.v1 (S8 iterative BA) and us026-mse-e2e-gate.v1 (S10 MSE method 2-5).
// Design: slam_refact/docs/design/us026-design.md §2-§5 and appendix L; gate documents
// build/slam-refact-full-20261019/preregistration/us026-{ba,mse}-e2e-gate.v1.md. Node only; it starts the ORIGINAL reference runtime SDK through
// run-reference.mjs (ephemeral 127.0.0.1 server, headless Chromium, external HTTP blocked) and the pinned native slam_refact_replay.
//
//   node gate_e2e.mjs precheck --gate ba|mse --scenario S <GATE ARGS>          every HELD check of a gate run, no marker, nothing written
//   node gate_e2e.mjs run --gate ba|mse --scenario S --kind gate <GATE ARGS> --out-root <dir> --run-log <file>
//   node gate_e2e.mjs run --gate ba|mse --scenario S --kind dev --out-root <dir> --run-log <file> --purpose <text>   (dev scenarios only, not held)
//   node gate_e2e.mjs verdict --gate ba|mse --manifest <manifest> --reports <report.json>,...   exit 0 PASS, 1 FAIL, 4 BLOCKED / UNEVALUATED
//   node gate_e2e.mjs audit <GATE ARGS without --designation-env>
//   node gate_e2e.mjs pin --manifest <manifest>                                 values a designation file pins
//   node gate_e2e.mjs print-pins --gate ba|mse                                  the live pins (what the registration manifest records)
//   GATE ARGS: --prereg-dir <dir> --gate-id <id> --gate-version <N> --designation-env <NAME>
//
// One scenario per process. Order of `run` (gate kind): (1) arguments and the hold (shared device sr_held through gate-common/held.mjs:
// manifest, review, designation, env value, held.pins, held.inputs[scenario] = tree sha256 of the scenario dataset; plus the checks below),
// (2) resources: lane-precheck.sh (PRECHECK_MAX_HEADLESS=0 PRECHECK_MAX_BUILDS=0, other thresholds at their defaults) and 1-min loadavg
// <= LOAD_MAX, waiting (never relaxing) up to WAIT_MAX_S, (3) the one-run marker of kind = scenario (O_EXCL), (4) only then the output
// directory, (5) every original run under `flock build/c1-chromium.lock`, each preceded INSIDE the lock by the same resource check
// (recorded in prerun/<arm>.json), (6) exact-input bundles (make_bundles.sh with the pinned base bundle), (7) the pinned replay binary,
// (8) compare_us026.py (the decision rules), (9) report.json, registered-file copies, SHA256SUMS, read-only tree.
// Exit codes: 0 report written (any class), 3 HELD (no marker), 5 resources not ready before the marker (no marker), 6 infrastructure
// failure after the marker (the run is used; gate document: coordinator decision + new version), 1 other error.
//
// WHAT THIS IS NOT: an accident-prevention device, not an authentication (protocol 12.6). Whoever can write the preregistration directory
// and set the environment variable can satisfy every check; the marker can be deleted by hand.
import fs from 'node:fs';
import os from 'node:os';
import path from 'node:path';
import { spawn, spawnSync } from 'node:child_process';
import { createHash } from 'node:crypto';
import { parseArgs } from 'node:util';
import { fileURLToPath } from 'node:url';
import { SR_HELD_FILE, claim, flatPins, gateArgs, gateMdPath, manifestPath, refuseSealedData, releaseCheck, request, sr } from '../../../web-chunk/tools/gate-common/held.mjs';
import { RunLog, envRecord } from '../../../web-chunk/tools/gate-common/runlog.mjs';
import { copyInto, makeReadOnly, readOnlyFiles, writeSums } from '../../../web-chunk/tools/gate-common/freeze.mjs';

const here = path.dirname(fileURLToPath(import.meta.url));
export const root = path.resolve(here, '../../../..');
const rel = (p) => path.relative(root, path.resolve(root, p));
const abs = (p) => path.resolve(root, p);

// ---- fixed configuration (the manifests record the same values; a gate run refuses a manifest that differs) -------------------------------
export const LOAD_MAX = 12;               // 1-min loadavg ceiling right before every original run (lead decision; dev: 29-31 diverged, 3.1-20 identical)
export const WAIT_MAX_S = 4 * 3600;       // longest wait for resources (before the marker: exit 5; after it: infrastructure failure)
export const POLL_S = 30;
export const REPLAY_JOBS = 2;
export const LOCK = 'build/c1-chromium.lock';
export const PRECHECK = 'slam_refact/docs/design/tools/lane-precheck.sh';
export const RUNNER = 'slam_refact/tools/re/runtime/run-reference.mjs';
export const MAKE_BUNDLES = 'slam_refact/tools/harness/us026/make_bundles.sh';
export const COMPARE = 'slam_refact/tools/harness/us026/compare_us026.py';
export const REPLAY_BIN = 'build/slam-refact-full-20261019/us026/ba-e2e-v2-pin/native/harness/slam_refact_replay';  // D2
export const BASES = 'build/slam-refact-full-20261019/us026/gate-inputs/bundles-base';
export const DEV_LOG = 'build/slam-refact-full-20261019/us026-draft/dev-run-log.tsv';
export const OUT_ALLOWED = 'build/slam-refact-full-20261019';  // run-reference.mjs writes only below it
export const RUN_OPTIONS = ['--memprobe', '--record-inputs', '--memprobe-detail', '12'];
const BA_ON = 'MapBuilder.bundleAdjustmentIterativeEnabled=b:1';
const method = (n) => `MetricScaleEstimator.scaleEstimationMethod=i:${n}`;

export const GATES = {
  ba: {
    gateId: 'us026-ba-e2e-gate', version: 2, compare: 'ba-scenario', devScenarios: ['flip6-n1'],
    arms: [{ name: 'poke', poke: { ba: 1 } }, { name: 'null', poke: { ba: 0, mse: 1 } }],
    bundles: [{ name: 'on', run: 'poke', params: BA_ON }, { name: 'off', run: 'poke', params: null }, { name: 'null', run: 'null', params: null }],
  },
  mse: {
    gateId: 'us026-mse-e2e-gate', version: 1, compare: 'mse-scenario', devScenarios: ['flip6-arcore-s3-absolute'],
    arms: [{ name: 'null', poke: { ba: 0, mse: 1 } }, ...[2, 3, 4, 5].map((n) => ({ name: `m${n}`, poke: { mse: n } }))],
    bundles: [{ name: 'null', run: 'null', params: null },
      ...[2, 3, 4, 5].flatMap((n) => [{ name: `m${n}-on`, run: `m${n}`, params: method(n) }, { name: `m${n}-m1`, run: `m${n}`, params: null }])],
  },
};

// files whose sha256 a gate run pins (repo-relative; held.pins keys them by base name)
export const PINNED_FILES = (g) => [
  g === 'ba' ? 'slam_refact/tools/gates/us026-ba-e2e-v2/gate_e2e.mjs' : 'slam_refact/tools/harness/us026/gate_e2e.mjs', COMPARE,  // D3
  MAKE_BUNDLES, 'slam_refact/tools/harness/us026/add_params.py',
  'slam_refact/tools/harness/divergence/make_variant.py', RUNNER, 'slam_refact/tools/re/runtime/page-runtime.js',
  'slam_refact/tools/re/runtime/memprobe.js', 'slam_refact/tools/re/runtime/input-recorder.js', 'slam_refact/tools/re/runtime/capnp-event-queue.mjs',
  'slam_refact/tools/re/runtime/scenarios.json', 'slam_refact/tools/re/runtime/us026/config-poke.js', 'scripts/dev/references/xr_reference/sensor_adapter.mjs',
  'scripts/dev/replay-android-input.mjs', 'assets/references/xr_reference/analysis-runtime/standalone/xr.js',
  'assets/references/xr_reference/analysis-runtime/standalone/xr-slam.js', PRECHECK, `slam_refact/tools/gates/${g === 'ba' ? 'us026-ba-e2e-v2' : 'us026-mse-e2e-v1'}/run.sh`,
  'slam_refact/web-chunk/tools/gate-common/held.mjs', 'slam_refact/web-chunk/tools/gate-common/runlog.mjs', 'slam_refact/web-chunk/tools/gate-common/freeze.mjs',
  REPLAY_BIN,
];

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

// the executables a gate run uses: node (this process), python3 on PATH, the chrome command (a wrapper script) and the binary it starts
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
export function livePinnedFiles(g) {
  const files = PINNED_FILES(g).map((f) => [abs(f), sha256File(abs(f))]);
  files.push([SR_HELD_FILE, sha256File(SR_HELD_FILE)]);
  const env = environmentFiles();
  for (const k of ['node', 'python', 'chromeWrapper', 'chrome']) if (env[k]) files.push([env[k], sha256File(env[k])]);
  return files;
}

// the base bundle of a scenario: the single directory below gate-inputs/bundles-base/<scenario>/
export function baseDirOf(scenario) {
  const d = path.join(abs(BASES), scenario);
  const subs = fs.existsSync(d) ? fs.readdirSync(d).filter((n) => fs.statSync(path.join(d, n)).isDirectory()) : [];
  if (subs.length !== 1) throw new Error(`${rel(d)}: expected exactly one base bundle directory, found ${subs.length}`);
  return path.join(d, subs[0]);
}

// sha256 of the uncompressed sensor stream the base bundle was made from (the original run must dispatch the same one)
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
    base_dir: rel(base), base_tree_sha256: bt.sha, sensors_sha256_uncompressed: baseSensorsSha(base) };
}

export const decisionConstants = (python = 'python3') => {
  const r = spawnSync(python, ['-I', abs(COMPARE), 'print-constants'], { encoding: 'utf8' });
  if (r.status !== 0) throw new Error(`compare_us026.py print-constants failed: ${r.stderr}`);
  return JSON.parse(r.stdout);
};

export const configRecord = (g) => ({ arms: GATES[g].arms, bundles: GATES[g].bundles, run_options: RUN_OPTIONS, load_max: LOAD_MAX, wait_max_s: WAIT_MAX_S,
  replay_jobs: REPLAY_JOBS, lock: LOCK, precheck_env: { PRECHECK_MAX_HEADLESS: '0', PRECHECK_MAX_BUILDS: '0' } });

const held = (check, why) => new Error(`HELD: ${check}: ${why}`);
const canonJson = (v) => JSON.stringify(v, (k, x) => (x && typeof x === 'object' && !Array.isArray(x) ? Object.fromEntries(Object.keys(x).sort().map((y) => [y, x[y]])) : x));

// ---- the hold (gate kind): everything before the marker ----------------------------------------------------------------------------------
export function checkHold({ g, gate, scenario, outDir, env = process.env }) {
  const cfg = GATES[g];
  if (gate.gateId !== cfg.gateId || gate.version !== cfg.version) throw held('args', `--gate ${g} is ${cfg.gateId}.v${cfg.version}, not ${gate.gateId}.v${gate.version}`);
  if (cfg.devScenarios.includes(scenario)) throw held('args', `${scenario} is a dev scenario`);
  for (const k of ['BROWSER_EXECUTABLE', 'NODE_OPTIONS']) if (env[k]) throw held('env', `${k} is set (the pinned chrome command and node are used as they are)`);
  const mf = manifestPath(gate);
  if (!fs.existsSync(mf)) throw held('manifest', `${rel(mf)} does not exist`);
  let manifest;
  try { manifest = JSON.parse(fs.readFileSync(mf, 'utf8')); } catch (error) { throw held('manifest', `not JSON: ${error.message}`); }
  const reg = manifest.registered_run ?? {};
  const datum = (reg.scenarios ?? []).find((s) => s.name === scenario);
  if (!datum) throw held('data', `${scenario} is not a registered scenario of ${cfg.gateId}.v${cfg.version}`);
  if (canonJson(reg.config) !== canonJson(configRecord(g))) throw held('config', 'manifest registered_run.config differs from the runner configuration');
  const env0 = environmentFiles();
  if (!env0.python) throw held('pins', 'python3 not found');
  let constants;
  try { constants = decisionConstants(env0.python); } catch (error) { throw held('constants', error.message); }
  if (canonJson(constants) !== canonJson(reg.decision_constants)) throw held('constants', 'compare_us026.py decision constants differ from manifest registered_run.decision_constants');
  // the pinned record equals the device's flat form, and every presented file is the pinned set
  const files = livePinnedFiles(g);
  const recorded = { ...(manifest.pinned?.files ?? {}) };
  for (const [k, v] of Object.entries(manifest.pinned?.executables ?? {})) recorded[k] = v;
  let flat;
  try { flat = flatPins(files); } catch (error) { throw held('pins', error.message); }
  const recordedFlat = Object.fromEntries(Object.entries(recorded).map(([k, v]) => [path.basename(k), v]));
  if (canonJson(recordedFlat) !== canonJson(manifest.held?.pins ?? null)) throw held('pins', 'manifest held.pins is not the flat form of manifest.pinned.files + executables');
  if (process.version !== manifest.pinned?.versions?.node) throw held('pins', `node ${process.version} is not the pinned ${manifest.pinned?.versions?.node}`);
  const trees = liveTrees();
  for (const [k, v] of Object.entries(manifest.pinned?.trees ?? {})) if (trees[k] !== v) throw held('pins', `tree ${k} ${trees[k]} differs from the pinned ${v}`);
  if (Object.keys(trees).sort().join() !== Object.keys(manifest.pinned?.trees ?? {}).sort().join()) throw held('pins', 'pinned tree set differs');
  // the registered scenario: base bundle tree, base sensors, dataset tree (the device checks the dataset as held.inputs[scenario])
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
  return { at: new Date().toISOString(), precheckExit: r.status, precheck: pre, loadavg: load, loadMax: LOAD_MAX,
    ok: r.status === 0 && load[0] <= LOAD_MAX };
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

// One original run inside the Chromium lock: `flock LOCK node gate_e2e.mjs _locked <job.json>`; the child checks the resources again
// INSIDE the lock, writes prerun/<arm>.json and starts run-reference.mjs, or exits 75 (not ready: the parent waits and retries).
async function lockedRun(job, log) {
  const t0 = Date.now();
  for (let attempt = 1; ; attempt++) {
    const jobFile = `${job.prerunFile}.job${attempt}.json`;
    fs.writeFileSync(jobFile, `${JSON.stringify(job)}\n`, { flag: 'wx' });
    const r = await runChild('flock', ['-w', String(WAIT_MAX_S), abs(LOCK), process.execPath, fileURLToPath(import.meta.url), '_locked', jobFile],
      { logFile: `${job.logFile}.attempt${attempt}` });
    if (r.exit !== 75) return { ...r, attempts: attempt };
    log?.write('resource-wait', { what: `inside lock before ${job.arm}`, attempt });
    if ((Date.now() - t0) / 1000 > WAIT_MAX_S) return { ...r, attempts: attempt, timedOut: true };
    await sleep(POLL_S);
  }
}

function lockedChild(jobFile) {
  const job = JSON.parse(fs.readFileSync(jobFile, 'utf8'));
  const c = resourceCheck();
  if (!c.ok) { fs.writeFileSync(`${jobFile}.not-ready.json`, `${JSON.stringify(c)}\n`, { flag: 'wx' }); return 75; }
  fs.writeFileSync(job.prerunFile, `${JSON.stringify({ ...c, lock: LOCK, arm: job.arm })}\n`, { flag: 'wx' });
  const env = { ...process.env };
  delete env.BROWSER_EXECUTABLE;
  const r = spawnSync(process.execPath, [abs(RUNNER), '--scenario', job.scenario, '--label', job.label, '--out-root', job.runsRoot, ...RUN_OPTIONS,
    '--poke', JSON.stringify(job.poke)], { cwd: root, env, stdio: 'inherit' });
  return r.status ?? 1;
}

// ---- US026B-M1 in code (D4) --------------------------------------------------------------------------------------------------------------
// v1 pre-run condition US026B-M1 (designation of us026-ba-e2e-gate.v1): "replay `on` exit != 0 and replay `off` and `null` exit = 0: independent
// of the comparator INVALID label, the scenario is a FAIL (engine BA-on replay abnormal exit), counted as evaluated (k), gate FAIL. Only `off` or
// `null` abnormal -> INVALID (device)". Applied only to a scenario the comparator classified INVALID (a BLOCKED original run has no replays).
export function engineOnExitRule(result, replays) {
  if (!result || result.class !== 'INVALID') return result;
  const exit = (n) => replays?.[n]?.exit;
  const abnormal = (v) => v === null || (typeof v === 'number' && v !== 0);   // killed by a signal (null) or a non-zero exit code; 'no-bundle' is not an engine exit
  if (abnormal(exit('on')) && exit('off') === 0 && exit('null') === 0) {
    return { ...result, class: 'EVALUATED', verdict: 'FAIL', rule: 'US026B-M1',
      classReason: [`US026B-M1: engine BA-on replay exited abnormally (exit ${exit('on')}) while the BA-off and null replays exited 0: counted as a FAIL, not INVALID`,
        ...(Array.isArray(result.classReason) ? result.classReason : [result.classReason])] };
  }
  return result;
}

// ---- one scenario -------------------------------------------------------------------------------------------------------------------------
function appendDevLog(entry) {
  const f = abs(DEV_LOG);
  const n = fs.existsSync(f) ? fs.readFileSync(f, 'utf8').split('\n').filter(Boolean).length : 0;
  const line = [n, entry.started, entry.finished, entry.scenario, entry.label, 'dev', entry.runnerSha, entry.options, entry.purpose, entry.exit, entry.status, entry.out]
    .map((x) => String(x ?? '').replace(/[\t\n]/g, ' ')).join('\t');
  fs.appendFileSync(f, `${line}\n`);
}

const stamp = () => new Date().toISOString().replace(/[-:]/g, '').replace(/\..*$/, '').toLowerCase();
const isoLocal = () => { const r = spawnSync('date', ['+%Y-%m-%dT%H:%M:%S%z'], { encoding: 'utf8' }); return r.stdout.trim(); };

export async function cmdRun(values) {
  const g = values.gate;
  const cfg = GATES[g];
  if (!cfg) throw new Error('--gate ba|mse');
  const kind = values.kind;
  if (kind !== 'gate' && kind !== 'dev') throw new Error('--kind gate|dev');
  const scenario = values.scenario;
  if (!/^[a-z0-9-]+$/.test(scenario ?? '')) throw new Error('--scenario <name>');
  const outRoot = abs(values['out-root'] ?? '');
  if (!(outRoot + path.sep).startsWith(abs(OUT_ALLOWED) + path.sep)) throw new Error(`--out-root must be under ${OUT_ALLOWED} (run-reference.mjs writes only there)`);
  const outDir = path.join(outRoot, cfg.gateId, scenario);
  if (!values['run-log']) throw new Error('--run-log <new file outside the output directory> is required');
  const log = new RunLog(abs(values['run-log']), { out: outDir });
  const startedAt = new Date().toISOString();
  log.write('start', { argv: process.argv.slice(2), node: process.version, kind, gate: g, scenario, env: envRecord(process.env, values['designation-env'] ? [values['designation-env']] : []) });
  let hold = null;
  let gate = null;
  let python = 'python3';
  try {
    if (kind === 'gate') {
      gate = gateArgs(values, root);
      try { hold = checkHold({ g, gate, scenario, outDir }); } catch (error) { log.write('held', { message: error.message }); throw error; }
      python = hold.python;
      log.write('hold-checks', { lines: hold.log });
    } else {
      if (!cfg.devScenarios.includes(scenario)) throw new Error(`refused: --kind dev runs only the dev scenarios ${cfg.devScenarios} (registration inputs stay unobserved)`);
      if (!values.purpose) throw new Error('--purpose <text> is required for --kind dev (dev-run-log.tsv)');
      if (values['prereg-dir'] && values['gate-id']) {
        const gd = gateArgs({ ...values, 'designation-env': values['designation-env'] ?? 'UNUSED_BY_DEV' }, root);
        const scen = JSON.parse(fs.readFileSync(abs('slam_refact/tools/re/runtime/scenarios.json'), 'utf8'))[scenario];
        refuseSealedData(gd, abs(scen.dataset));
      }
      if (fs.existsSync(outDir)) throw new Error(`${rel(outDir)} exists`);
    }
    const datum = kind === 'gate' ? hold.datum : scenarioRecord(scenario);
    // resources before the marker (a refusal here does not use the run)
    const before = await waitResources(log, 'before the marker');
    log.write('resources-before-marker', { ok: before.ok, attempts: before.attempts, waitedS: before.waitedS, loadavg: before.loadavg, timedOut: before.timedOut ?? false });
    if (!before.ok) { log.close('NOT-READY'); return { exit: 5, why: 'resources not ready before the marker (no marker created)' }; }
    let marker = null;
    if (kind === 'gate') {
      const outcome = claim(hold.rq);
      marker = outcome.marker;
      hold.outcome = outcome;
      log.write('first-run-claimed', { marker: rel(marker), scenario, designationSha256: outcome.designationSha256 });
    }
    fs.mkdirSync(outDir, { recursive: true });
    for (const d of ['runs', 'bundles', 'replays', 'prerun', 'logs']) fs.mkdirSync(path.join(outDir, d));
    const prefix = kind === 'gate' ? 'gate' : `dev-${stamp()}`;
    const runnerSha = sha256File(abs(RUNNER));
    const runs = {};
    let infra = null;
    for (const arm of cfg.arms) {
      const label = `${prefix}-${arm.name}`;
      const job = { scenario, arm: arm.name, label, poke: arm.poke, runsRoot: rel(path.join(outDir, 'runs')),
        prerunFile: path.join(outDir, 'prerun', `${arm.name}.json`), logFile: path.join(outDir, 'logs', `run-${arm.name}.log`) };
      const started = isoLocal();
      log.write('original-run-start', { arm: arm.name, label, poke: arm.poke });
      const r = await lockedRun(job, log);
      const dir = path.join(outDir, 'runs', scenario, label);
      let status = null;
      try { status = JSON.parse(fs.readFileSync(path.join(dir, 'meta.json'), 'utf8')).status; } catch { status = 'NO-META'; }
      const prerun = fs.existsSync(job.prerunFile) ? JSON.parse(fs.readFileSync(job.prerunFile, 'utf8')) : null;
      runs[arm.name] = { dir: rel(dir), label, poke: arm.poke, runnerExit: r.exit, signal: r.signal ?? null, seconds: r.seconds, attempts: r.attempts, status,
        prerun: prerun ? { ok: prerun.ok, at: prerun.at, loadavg: prerun.loadavg, precheckExit: prerun.precheckExit, file: rel(job.prerunFile) } : { ok: false } };
      log.write('original-run-end', { arm: arm.name, exit: r.exit, status, seconds: r.seconds, timedOut: r.timedOut ?? false });
      if (kind === 'dev') appendDevLog({ started, finished: isoLocal(), scenario, label, runnerSha, options: `${RUN_OPTIONS.join(' ')} --poke ${JSON.stringify(arm.poke)} (gate_e2e.mjs ${g} dev)`, purpose: values.purpose, exit: r.exit, status, out: rel(dir) });
      if (r.timedOut || !prerun) { infra = `resources or lock not available for ${arm.name} within ${WAIT_MAX_S} s`; break; }
    }
    // bundles: one make_bundles.sh call per original run, with the variants of that run
    const bundles = {};
    if (!infra) {
      const benv = { ...process.env, US026_RUNS: rel(path.join(outDir, 'runs')), US026_BUNDLES: rel(path.join(outDir, 'bundles')), US026_BASE_ROOT: rel(path.join(outDir, 'bundles-base-unused')), US026_BASE: datum.base_dir };
      for (const arm of cfg.arms) {
        const label = runs[arm.name].label;
        const variants = cfg.bundles.filter((b) => b.run === arm.name && b.params).map((b) => `${b.name}=${b.params}`);
        const r = await runChild('bash', [abs(MAKE_BUNDLES), scenario, label, ...variants], { logFile: path.join(outDir, 'logs', `bundles-${arm.name}.log`), env: benv });
        log.write('bundles', { arm: arm.name, exit: r.exit });
        for (const b of cfg.bundles.filter((x) => x.run === arm.name)) {
          bundles[b.name] = { dir: rel(path.join(outDir, 'bundles', scenario, b.params ? `${label}-${b.name}` : label)), run: arm.name, params: b.params, makeExit: r.exit };
        }
      }
    }
    // replays with the pinned binary
    const replays = {};
    if (!infra) {
      const items = cfg.bundles.map((b) => b.name);
      const res = await pool(items, REPLAY_JOBS, async (name) => {
        const b = bundles[name];
        const out = path.join(outDir, 'replays', name);
        if (b.makeExit !== 0 || !fs.existsSync(abs(b.dir))) return { name, exit: 'no-bundle' };
        const r = await runChild(abs(REPLAY_BIN), ['--bundle', abs(b.dir), '--out', out], { logFile: path.join(outDir, 'logs', `replay-${name}.log`) });
        return { name, ...r };
      });
      for (const r of res) replays[r.name] = { dir: rel(path.join(outDir, 'replays', r.name)), exit: r.exit, seconds: r.seconds ?? null, run: bundles[r.name].run, params: bundles[r.name].params };
      log.write('replays', { exits: Object.fromEntries(res.map((r) => [r.name, r.exit])) });
    }
    // the decision rules
    const spec = { gate: g, scenario, kind, runs, replays, sensors: { expectedSha256Uncompressed: datum.sensors_sha256_uncompressed }, infraFailure: infra };
    fs.writeFileSync(path.join(outDir, 'spec.json'), `${JSON.stringify(spec, null, 1)}\n`, { flag: 'wx' });
    let compare = null;
    if (!infra) {
      const r = await runChild(python, ['-I', abs(COMPARE), cfg.compare, '--spec', path.join(outDir, 'spec.json'), '--out', path.join(outDir, 'compare.json')],
        { logFile: path.join(outDir, 'logs', 'compare.log') });
      if (r.exit === 0) compare = JSON.parse(fs.readFileSync(path.join(outDir, 'compare.json'), 'utf8'));
      else infra = `compare_us026.py exit ${r.exit}`;
    }
    const scenarioResult = compare ? engineOnExitRule({ class: compare.class, verdict: compare.verdict ?? null, classReason: compare.classReason ?? null,
      pairs: compare.pairs ? Object.fromEntries(Object.entries(compare.pairs).map(([k, v]) => [k, { class: v.class, verdict: v.verdict ?? null }])) : null,
      mid5: compare['M-ID-5'] ? { evaluated: compare['M-ID-5'].evaluated, verdict: compare['M-ID-5'].verdict ?? null } : null }, g === 'ba' ? replays : null) : { class: 'INVALID', classReason: [`infrastructure failure: ${infra}`] };
    const pins = Object.fromEntries(livePinnedFiles(g).map(([f, s]) => [path.basename(f), s]));
    const report = { schema: 'slam-refact-us026-gate-e2e-report-v1', gate: { id: cfg.gateId, version: cfg.version, key: g }, kind, scenario, startedAt,
      finishedAt: new Date().toISOString(), argv: process.argv.slice(2), versions: versions(), trees: liveTrees(), pins, marker: marker ? rel(marker) : null,
      datum, resourcesBeforeMarker: { ok: before.ok, at: before.at, loadavg: before.loadavg, attempts: before.attempts, waitedS: before.waitedS },
      runs, bundles, replays, infraFailure: infra, result: scenarioResult, compareFile: compare ? 'compare.json' : null,
      gitHead: spawnSync('git', ['rev-parse', 'HEAD'], { cwd: root, encoding: 'utf8' }).stdout.trim() };
    if (kind === 'gate') {
      const reg = hold.manifest.review ?? {};
      const files = [gateMdPath(gate), manifestPath(gate), path.join(gate.preregDir, reg.designation_file), path.join(gate.preregDir, reg.file), marker];
      report.registeredCopies = copyInto(outDir, 'registered', files);
    }
    fs.writeFileSync(path.join(outDir, 'report.json'), `${JSON.stringify(report, null, 1)}\n`, { flag: 'wx' });
    const sums = writeSums(outDir);
    const entries = makeReadOnly(outDir);
    if (kind === 'gate') readOnlyFiles([gateMdPath(gate)]);
    log.write('finished', { report: rel(path.join(outDir, 'report.json')), result: scenarioResult, sums, readOnlyEntries: entries, infraFailure: infra });
    log.close(infra ? 'INFRA-FAILURE' : 'OK');
    console.log(JSON.stringify({ scenario, kind, result: scenarioResult, report: rel(path.join(outDir, 'report.json')), infraFailure: infra }));
    return { exit: infra ? 6 : 0 };
  } catch (error) {
    if (!log.closed) { log.write('error', { message: String(error?.message ?? error) }); log.close(String(error?.message ?? error).startsWith('HELD:') ? 'HELD' : 'ERROR'); }
    throw error;
  }
}

// ---- verdict over the registered scenarios -------------------------------------------------------------------------------------------------
export function aggregate(g, manifest, reports) {
  const names = (manifest.registered_run?.scenarios ?? []).map((s) => s.name);
  const by = {};
  for (const r of reports) {
    if (r.gate?.key !== g || r.kind !== 'gate') throw new Error(`report of ${r.gate?.key}/${r.kind} given to the ${g} gate verdict`);
    if (!names.includes(r.scenario)) throw new Error(`${r.scenario} is not a registered scenario`);
    by[r.scenario] = r.result;
  }
  const out = { gate: GATES[g].gateId, version: GATES[g].version, n_independent: manifest.independent_samples?.n ?? null, scenarios: {}, unevaluated: [] };
  for (const n of names) { if (!by[n]) out.unevaluated.push(n); out.scenarios[n] = by[n] ?? { class: 'NOT-RUN' }; }
  const inv = Object.entries(out.scenarios).filter(([, r]) => r.class === 'INVALID').map(([n, r]) => ({ scenario: n, reason: r.classReason }));
  out.invalid = inv;
  if (g === 'ba') {
    const ev = Object.entries(out.scenarios).filter(([, r]) => r.class === 'EVALUATED');
    out.blocked = Object.entries(out.scenarios).filter(([, r]) => r.class === 'BLOCKED').map(([n, r]) => ({ scenario: n, reason: r.classReason }));
    out.k = ev.length;
    out.pass = ev.filter(([, r]) => r.verdict === 'PASS').map(([n]) => n);
    out.fail = ev.filter(([, r]) => r.verdict !== 'PASS').map(([n]) => n);
    out.verdict = out.fail.length ? 'FAIL' : out.unevaluated.length ? 'UNEVALUATED' : out.k === 0 ? 'BLOCKED' : 'PASS';
  } else {
    const pairs = [];
    const mid = [];
    for (const [n, r] of Object.entries(out.scenarios)) {
      if (r.class === 'EVALUATED') {
        for (const [m, p] of Object.entries(r.pairs ?? {})) pairs.push({ scenario: n, method: m, class: p.class, verdict: p.verdict });
        if (r.mid5) mid.push({ scenario: n, ...r.mid5 });
      }
    }
    const ev = pairs.filter((p) => p.class === 'EVALUATED');
    out.pairs = pairs;
    out.k = ev.length;
    out.mid5 = mid;
    out.fail = [...ev.filter((p) => p.verdict !== 'PASS').map((p) => `${p.scenario}/${p.method}`), ...mid.filter((x) => x.evaluated && x.verdict !== 'PASS').map((x) => `${x.scenario}/M-ID-5`)];
    const midEvaluated = mid.filter((x) => x.evaluated).length;
    out.mid5Evaluated = midEvaluated;
    out.verdict = out.fail.length ? 'FAIL' : out.unevaluated.length ? 'UNEVALUATED' : out.k === 0 ? 'BLOCKED' : midEvaluated === 0 ? 'BLOCKED' : 'PASS';
    if (out.verdict === 'BLOCKED' && out.k > 0) out.blockedReason = 'no M-ID-5 evaluated';
  }
  return out;
}

function cmdVerdict(values) {
  const g = values.gate;
  const manifest = JSON.parse(fs.readFileSync(abs(values.manifest), 'utf8'));
  const reports = (values.reports ?? '').split(',').filter(Boolean).map((f) => JSON.parse(fs.readFileSync(abs(f), 'utf8')));
  const agg = aggregate(g, manifest, reports);
  console.log(JSON.stringify(agg, null, 1));
  return agg.verdict === 'PASS' ? 0 : agg.verdict === 'FAIL' ? 1 : 4;
}

function cmdPrecheck(values) {
  const g = values.gate;
  const cfg = GATES[g];
  if (!cfg) throw new Error('--gate ba|mse');
  const gate = gateArgs(values, root);
  const outDir = path.join(abs(values['out-root'] ?? `${OUT_ALLOWED}/us026/gate-runs`), cfg.gateId, values.scenario);
  const hold = checkHold({ g, gate, scenario: values.scenario, outDir });
  console.log(`${hold.log.join('\n')}\nRELEASED (precheck: no marker created, nothing run) scenario ${values.scenario}`);
  return 0;
}

function cmdAudit(values) {
  const gate = gateArgs({ ...values, 'designation-env': 'UNUSED_BY_AUDIT' }, root);
  const manifest = JSON.parse(fs.readFileSync(manifestPath(gate), 'utf8'));
  let bad = 0;
  for (const kind of manifest.held?.kinds ?? []) {
    const o = sr.audit({ preregDir: gate.preregDir, gateId: gate.gateId, version: gate.version, kind });
    console.log(`${kind}: ${o.ok ? 'AUDIT OK' : `AUDIT FAILED [${o.code}] ${o.why}`}`);
    if (!o.ok) bad++;
  }
  return bad ? 3 : 0;
}

function cmdPin(values) {
  const bytes = fs.readFileSync(abs(values.manifest));
  const parsed = sr.parseJson(bytes);
  if (parsed.err !== undefined) throw new Error(`manifest is not strict JSON: ${parsed.err}`);
  console.log(JSON.stringify({ gate_md_sha256: JSON.parse(bytes.toString('utf8')).sha256, manifest_before_review_sha256: sr.canonManifestSha(parsed.json) }, null, 1));
  return 0;
}

// the live values a registration manifest records (pinned.files, pinned.executables, pinned.trees, pinned.versions, held.pins, config)
export function livePins(g) {
  const files = {};
  for (const f of PINNED_FILES(g)) files[f] = sha256File(abs(f));
  files[rel(SR_HELD_FILE)] = sha256File(SR_HELD_FILE);
  const env = environmentFiles();
  const executables = {};
  for (const k of ['node', 'python', 'chromeWrapper', 'chrome']) if (env[k]) executables[env[k]] = sha256File(env[k]);
  const all = { ...files, ...executables };
  return { files, executables, trees: liveTrees(), versions: versions(), held_pins: Object.fromEntries(Object.entries(all).map(([k, v]) => [path.basename(k), v])),
    config: configRecord(g), decision_constants: decisionConstants() };
}

const isMain = process.argv[1] && path.resolve(process.argv[1]) === fileURLToPath(import.meta.url);
if (isMain) {
  const [cmd, ...rest] = process.argv.slice(2);
  try {
    if (cmd === '_locked') {
      process.exitCode = lockedChild(rest[0]);
    } else {
      const { values } = parseArgs({ args: rest, options: { gate: { type: 'string' }, scenario: { type: 'string' }, kind: { type: 'string' }, 'out-root': { type: 'string' },
        'run-log': { type: 'string' }, purpose: { type: 'string' }, 'prereg-dir': { type: 'string' }, 'gate-id': { type: 'string' }, 'gate-version': { type: 'string' },
        'designation-env': { type: 'string' }, manifest: { type: 'string' }, reports: { type: 'string' } } });
      if (cmd === 'run') process.exitCode = (await cmdRun(values)).exit;
      else if (cmd === 'precheck') process.exitCode = cmdPrecheck(values);
      else if (cmd === 'verdict') process.exitCode = cmdVerdict(values);
      else if (cmd === 'audit') process.exitCode = cmdAudit(values);
      else if (cmd === 'pin') process.exitCode = cmdPin(values);
      else if (cmd === 'print-pins') console.log(JSON.stringify(livePins(values.gate), null, 1));
      else { console.error('usage: gate_e2e.mjs precheck|run|verdict|audit|pin|print-pins ... (see the header)'); process.exitCode = 2; }
    }
  } catch (error) {
    console.error(String(error?.message ?? error));
    process.exitCode = String(error?.message).startsWith('HELD:') ? 3 : 1;
  }
}

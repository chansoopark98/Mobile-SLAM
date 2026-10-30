// us025-lc-e2e-gate.v3 runner. FORK of slam_refact/tools/gates/us025-lc-e2e-v2/gate_lc.mjs (v2 itself forks v1, v1 forks us026-ba-e2e-v2/gate_e2e.mjs).
// WHAT CHANGED AGAINST THE v2 FILE (design: slam_refact/docs/design/us025-lc-e2e-v3-parallel-design.md, decisions D-V3-0..3 and D-V3-LOCK in orchestrator-decisions-2026-10-23.md 7.7).
// Only the way the work is EXECUTED changes. judge.mjs, compare_full_us025.py, test_compare_full_us025.py, judge.test.mjs and make_lc_manifest.mjs are byte-identical copies of the v2 files
// (gate_lc.test.mjs checks the sha256 of each against the v2 file); every decision constant, run count, tolerance and verdict rule is the v2 value.
//   V3-1  gate version 3, gate id unchanged (us025-lc-e2e-gate); v3 names (tool path, designation variable, default output path, report schema ...-v3). The v1 pin directory (engine commit,
//         replay binary) is reused as in v2 (D-V2-3).
//   V3-2  TWO PHASES, not overlapped. Phase A: the original runs of ALL scenarios (v2: 36 + 10 run per scenario one after another) go into ONE global queue (manifest order of the
//         scenarios; inside a scenario the lc=1 runs in label order, then the lc=0 runs, as in v2) and are executed by SLOTS slots. Phase B starts only after every phase A run has ended
//         and the Chromium and slot locks are released: bundles, engine replays and comparator run in a global pool of J jobs. Why not overlapped: replay load must not change the
//         environment of an original run (the original SDK is load sensitive).
//   V3-3  new command `run-all` (all scenarios in one process). `run --scenario S` is kept as `run-all` with one scenario. The marker of a scenario is claimed right before the FIRST
//         original run of that scenario starts (after the first per-run readiness check), as before one marker per scenario. A scenario that was never started has no marker and no report.
//   V3-4  locks. build/c1-chromium.lock is held by this process for the whole of phase A (v2: per run, via `flock ... _locked`). Between the slots the gate uses its own locks
//         build/us025-lc-v3-slot-<k>.lock (one per slot, held for phase A). `_locked` is gone; a lock holder is a `flock` child whose stdin is the lock lifetime (the lock is released when this
//         process ends in any way).
//   V3-5  slots. SLOTS = 1 (the registered value; the real run refuses another --slots with HELD) is the v2 path: no taskset, load <= 8 rechecked before every run, same order. With SLOTS > 1 every
//         original run is pinned with `taskset -c <slot cpu set>`. The host is a hybrid CPU (i9-13900K: threads 0-15 = 8 P cores x 2 HT, 16-31 = 16 E cores), so every slot gets the SAME mix: the P threads
//         and the E cores of its share (V3-12).
//   V3-6  readiness is checked per run, in the run's slot, right before the run: lane-precheck.sh with the same thresholds (PRECHECK_MAX_HEADLESS is the number of headless Chromium
//         processes that belong to THIS gate's other running slots, so a foreign Chromium still blocks), 1-min load <= LOAD_MAX + L_SLOT * (other slots running now), and (SLOTS > 1)
//         the slot's CPU set at most SLOT_CPU_UTIL_MAX busy over the last SLOT_CPU_WINDOW_S seconds. Wait, never relax (WAIT_MAX_S per run, as v2).
//   V3-7  the watchdog is per slot and touches only its own run's process tree (renderers below the runner pid; the process group of that runner). `supervise` got an onSpawn hook (for the
//         slot registry); its kill rules are the v2 rules.
//   V3-8  phase B: bundles are built serially as in v2 (same order, same commands.txt); replays and comparators run in a pool of J (default 6, at most 8) over ALL scenarios; J does not change any output byte.
//   V3-10 run_lcgold.mjs (pinned, not modified) writes <runs>/<scenario>/engine-input-sensors.jsonl.gz once per scenario and reads it back at once; two runs of one scenario starting together could read
//         a half-written file. The first run of a scenario starts alone and the others wait until that file is complete (waitStableFile). Chrome profile (playwright random /tmp dir), HTTP port (listen(0))
//         and DevTools (pipe) are already private to each run (measured, implementation notes 3.3).
//   V3-9  `dev` command (no marker, no preregistration, no hold; output only below build/slam-refact-pkgh-20261022/us025/lc-e2e-v3-dev/): arms S / P<k> in blocks (--blocks S:6,P2:6,...) for the
//         equivalence experiment, phase B included, one summary JSON. `phase-b` command: replays and compares EXISTING bundles of a finished gate output (read-only input) for the J bit-identity test.
//   V3-12 (prereview-us025-lc-v3-equivalence.md C-1, decision D-V3-8; execution machinery only): slotCpuSets splits the P threads (CPU_P_FIRST..CPU_P_LAST) and the E cores (CPU_E_FIRST..CPU_E_LAST) of
//         the range separately and gives slot k the k-th share of each, e.g. N = 2: `0-7,16-23` and `8-15,24-31`; N = 4: `0-3,16-19`, `4-7,20-23`, `8-11,24-27`, `12-15,28-31`. HT sibling pairs (2i, 2i+1) are never
//         split. checkCpuTopology() refuses to start a pinned run if /sys does not show that layout. SLOT_CPU_FIRST/LAST = 0/31 (the dev default and the experiment's sets) so the real run uses the same sets.
//   V3-11 (review-us025-lc-e2e-v3-runner.md, decision D-V3-7; execution machinery only, no verdict rule): C-1 holdLock().release() returns at once when the flock child is already dead and a lock holder that
//         ends early is reported (lock.lost / onLost): phase A stops starting runs and the started scenarios get INFRA reports. C-2 the dev command defaults to cores 0-31 (DEV_CPU_FIRST/LAST).
//         C-3 phase B is isolated per scenario: every step catches its own error (INFRA `phase B error (...)` of that run and scenario), each report is written as soon as ITS scenario is complete, a
//         report that cannot be written is an error of that scenario only (exit 1 at the end). R-1 runQueue lets the other slots finish their run and throws the first worker error afterwards;
//         run-all turns it into abort + reports. R-2 recheckPins() between phase A and phase B (freeze paths clean, pinned files/executables/trees equal the registration, engine pin): on drift no
//         replay is made and the started scenarios get INFRA reports.
// What the v2 file changed against the v1 file (registration document us025-lc-e2e-gate-v2-registration.md section 9): version 2 and the v2 names; THIN class and the whole-run comparison of
// NO_TRIGGER runs (D-V2-1, D-V2-2: ./judge.mjs, ./compare_full_us025.py, this file's `full` arm); the pinned file list. Every other value is the v1 value.
// What the v1 file changed against us026-ba-e2e-v2/gate_e2e.mjs (registration document v1 section 9):
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
//   node gate_lc.mjs run-all --scenarios S1,S2,... <GATE ARGS> --out-root <dir> --run-log <file> [--slots N] [--jobs J]
//   node gate_lc.mjs run --scenario S <GATE ARGS> --out-root <dir> --run-log <file>      (= run-all with one scenario)
//   node gate_lc.mjs verdict --manifest <manifest> --reports <report.json>,...    exit 0 PASS, 1 FAIL, 4 BLOCKED/UNEVALUATED, 6 INFRA, 7 INSUFFICIENT, 8 INVALID
//   node gate_lc.mjs audit <GATE ARGS without --designation-env>
//   node gate_lc.mjs pin --manifest <manifest>      |      node gate_lc.mjs print-pins
//   node gate_lc.mjs dev --scenarios S1,S2 --blocks S:6,P2:6,P4:6 --out-root <dir below lc-e2e-v3-dev> --run-log <file> [--lc0 K] [--jobs J] [--no-phase-b] [--cpu-first A --cpu-last B]
//   node gate_lc.mjs compare-trees --ref <dir> --out <dir> [--save <new json>]     (byte comparison of two phase B trees; wall-clock fields reported separately)
//   node gate_lc.mjs phase-b --from <finished gate scenario dir> --out <dir below lc-e2e-v3-dev> --jobs J [--compare-to <dir>]
//   GATE ARGS: --prereg-dir <dir> --gate-id <id> --gate-version <N> --designation-env <NAME>
// Exit codes of `run` / `run-all`: 0 reports written (any class), 3 HELD (no marker), 5 resources not ready before a marker (no marker for that scenario; scenarios already started
// are finished), 6 infrastructure failure after a marker (that run is used; coordinator decision + new version), 1 other error.
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
import { copyInto, listFiles, makeReadOnly, readOnlyFiles, writeSums } from '../../../web-chunk/tools/gate-common/freeze.mjs';
import { JUDGE_CONSTANTS, SCENARIO_RUNS, VERDICT_EXIT, aggregate, classifyLc0Run, classifyLc1Run, classifyScenario } from './judge.mjs';

const here = path.dirname(fileURLToPath(import.meta.url));
export const root = path.resolve(here, '../../../..');
const rel = (p) => path.relative(root, path.resolve(root, p));
const abs = (p) => path.resolve(root, p);

// ---- fixed configuration (the manifest records the same values; a gate run refuses a manifest that differs) ------------------------------
export const GATE_ID = 'us025-lc-e2e-gate';
export const GATE_VERSION = 3;
export const DEV_SCENARIOS = ['flip6-n1', 'flip6-arcore-s3-absolute'];   // never gate scenarios (they were used for the LC-0 survey and the LC-G goldens)
export const LOAD_MAX = 8;                // 1-min loadavg ceiling right before every original run (lc6a precedent: runs started below 8; stricter than ba-e2e's 12)
export const WAIT_MAX_S = 4 * 3600;       // longest wait for resources (before the marker: exit 5; after it: that run is INFRA and the scenario stops)
export const POLL_S = 30;
export const LOCK = 'build/c1-chromium.lock';
export const PRECHECK = 'slam_refact/docs/design/tools/lane-precheck.sh';
export const RUNNER = 'slam_refact/tools/re/runtime/us025/run_lcgold.mjs';
export const MAKE_BUNDLES = 'slam_refact/tools/harness/us026/make_bundles.sh';
export const COMPARE = 'slam_refact/tools/harness/us025/compare_us025.py';
export const COMPARE_FULL = 'slam_refact/tools/gates/us025-lc-e2e-v3/compare_full_us025.py';   // byte-identical copy of the v2 file (D-V2-2): whole-run ON comparison of a run without an original success
export const PIN_DIR = 'build/slam-refact-pkgh-20261022/us025/lc-e2e-v1-pin';   // v2 pins the SAME engine commit and replay binary as v1 (D-V2-3): the v1 pin directory is reused, not rebuilt
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
// ---- v3 execution-mode constants (V3-5, V3-6, V3-8). They change HOW the work is run, never what is judged; the manifest records them with the rest of the configuration. ----
export const SLOTS = 1;                          // phase A slots of the REAL run. 1 = the v2 path (no taskset). Set to the largest N that passes the equivalence experiment (D-V3-2) BEFORE the pins are taken.
export const SLOTS_SUPPORTED = [1, 2, 4, 8];     // D-V3-8 C-1: the P threads (16) and the E cores (16) must each split evenly into HT pairs per slot, so 3 and 6 are gone
export const SLOT_CPU_FIRST = 0;                 // V3-12: the real run uses the same symmetric slot CPU sets as the equivalence experiment (all 32 CPUs), see slotCpuSets
export const SLOT_CPU_LAST = 31;
export const CPU_P_FIRST = 0;                   // hybrid topology of the host (lscpu / sysfs, checked by checkCpuTopology): P-core threads 0-15 (HT siblings 2i,2i+1), E cores 16-31 (no SMT)
export const CPU_P_LAST = 15;
export const CPU_E_FIRST = 16;
export const CPU_E_LAST = 31;
export const L_SLOT = 10;                        // load contribution of ONE running original run: measure_run.mjs, per-run mean of runnable threads 5.0-9.4 over 8 completed runs, max 9.4, rounded up (implementation notes 3.2); load limit = LOAD_MAX + L_SLOT * (other slots running)
export const DEV_CPU_FIRST = 0;                  // C-2: the dev command (equivalence experiment arms P2 = 0-7,16-23 / 8-15,24-31, P4 = 0-3,16-19 / 4-7,20-23 / 8-11,24-27 / 12-15,28-31, D-V3-8) uses all 32 CPUs by default; the real run's SLOT_CPU_FIRST/LAST must equal this range (checked by a test)
export const DEV_CPU_LAST = 31;
export const SLOT_CPU_UTIL_MAX = 0.25;           // the slot's CPU set may be at most this busy over the last SLOT_CPU_WINDOW_S seconds (only with SLOTS > 1)
export const SLOT_CPU_WINDOW_S = 5;
export const SLOT_LOCK_PREFIX = 'build/us025-lc-v3-slot-';   // + <k> + .lock
export const REPLAY_JOBS_DEFAULT = 6;            // phase B pool (bundles are serial; replays and comparators run J at a time); J does not change an output byte
export const REPLAY_JOBS_MAX = 8;
export const DEV_ROOT = `${OUT_ALLOWED}/us025/lc-e2e-v3-dev`;   // the only place the dev and phase-b commands write

// files whose sha256 a gate run pins (repo-relative; held.pins keys them by base name)
export const PINNED_FILES = () => [
  'slam_refact/tools/gates/us025-lc-e2e-v3/gate_lc.mjs', 'slam_refact/tools/gates/us025-lc-e2e-v3/judge.mjs', 'slam_refact/tools/gates/us025-lc-e2e-v3/run.sh',
  COMPARE, COMPARE_FULL, 'slam_refact/tools/harness/us026/compare_us026.py',
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
  const f = spawnSync(python, ['-B', '-I', abs(COMPARE_FULL), 'constants'], { encoding: 'utf8' });
  if (f.status !== 0) throw new Error(`compare_full_us025.py constants failed: ${f.stderr}`);
  return { compare_us025: JSON.parse(r.stdout), compare_full_us025: JSON.parse(f.stdout), judge: JUDGE_CONSTANTS, verdict_exit: VERDICT_EXIT };
};

export const configRecord = () => ({ scenario_runs: SCENARIO_RUNS, run_options_lc1: RUN_OPTIONS_LC1, run_options_lc0: RUN_OPTIONS_LC0, on_params: ON_PARAMS,
  load_max: LOAD_MAX, wait_max_s: WAIT_MAX_S, replay_jobs_default: REPLAY_JOBS_DEFAULT, replay_jobs_max: REPLAY_JOBS_MAX, lock: LOCK,
  slots: SLOTS, slots_supported: SLOTS_SUPPORTED, slot_cpu_sets: slotCpuSets(SLOTS), l_slot: L_SLOT, slot_cpu_util_max: SLOT_CPU_UTIL_MAX, slot_cpu_window_s: SLOT_CPU_WINDOW_S, slot_lock_prefix: SLOT_LOCK_PREFIX,
  phases: 'A: all original runs, one global queue over the slots, Chromium lock held; B: bundles, replays, comparator in a pool after A ended', precheck_env: { PRECHECK_MAX_HEADLESS: '0', PRECHECK_MAX_BUILDS: '0' },
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
  if (canonJson(constants) !== canonJson(reg.decision_constants)) throw held('constants', 'compare_us025.py / compare_full_us025.py / judge.mjs decision constants differ from manifest registered_run.decision_constants');
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

// D-V3-7 / review R-2: the check that runs between phase A and phase B. checkHold looks once, before the first marker; phase A takes hours. Returns the list of problems (empty = ok):
// the freeze paths are clean, every pinned file and executable still has the sha256 of the registration (manifest held.pins), the pinned trees are unchanged, the engine pin still holds
// (engine sources at HEAD equal the pinned commit, replay binary sha256).
export function recheckPins(manifest, { git = gitReal, live = livePinnedFiles, trees = liveTrees, engineCheck = checkEnginePin } = {}) {
  const problems = [];
  const dirty = freezeDirty(git);
  if (dirty) problems.push(`git status of the freeze paths is not empty: ${dirty.split('\n').slice(0, 5).join(' | ')}`);
  const recorded = manifest.held?.pins ?? {};
  const liveFlat = Object.fromEntries(live({ withBinary: true }).map(([f, sha]) => [path.basename(f), sha]));
  const drift = Object.keys({ ...recorded, ...liveFlat }).filter((k) => recorded[k] !== liveFlat[k]);
  if (drift.length) problems.push(`pinned files differ from the registration: ${drift.join(', ')}`);
  const t = trees();
  for (const [k, v] of Object.entries(manifest.pinned?.trees ?? {})) if (t[k] !== v) problems.push(`tree ${k} ${t[k]} differs from the pinned ${v}`);
  try { engineCheck(manifest.engine_pin); } catch (error) { problems.push(String(error?.message ?? error)); }
  return problems;
}

// ---- resources (V3-5, V3-6) -------------------------------------------------------------------------------------------------------------
export const parseCpuList = (s) => s.split(',').flatMap((part) => { const [a, b] = part.split('-').map(Number); return b === undefined ? [a] : Array.from({ length: b - a + 1 }, (_, i) => a + i); });

// `n` slots over the CPUs first..last, each slot with the SAME mix of P threads and E cores (D-V3-8 C-1). The P threads of the range (CPU_P_FIRST..CPU_P_LAST) and its E cores (CPU_E_FIRST..CPU_E_LAST) are
// cut into n equal consecutive shares and slot k gets share k of each: n = 2 over 0-31 -> `0-7,16-23`, `8-15,24-31`. The P share is made of whole HT pairs (even start, even size). n = 1 has no CPU set
// (null: no taskset, the v2 path). A range whose P and E parts differ in size (e.g. 8-31) is refused: it could not be split symmetrically.
export function slotCpuSets(n, first = SLOT_CPU_FIRST, last = SLOT_CPU_LAST) {
  if (!SLOTS_SUPPORTED.includes(n)) throw new Error(`slots ${n}: supported ${SLOTS_SUPPORTED.join(',')}`);
  if (n === 1) return [null];
  const pFirst = Math.max(first, CPU_P_FIRST);
  const pLast = Math.min(last, CPU_P_LAST);
  const eFirst = Math.max(first, CPU_E_FIRST);
  const eLast = Math.min(last, CPU_E_LAST);
  const pCount = pLast - pFirst + 1;
  const eCount = eLast - eFirst + 1;
  if (pCount <= 0 || eCount <= 0 || pCount !== eCount) throw new Error(`cpus ${first}-${last}: ${Math.max(pCount, 0)} P threads and ${Math.max(eCount, 0)} E cores cannot give every slot the same mix`);
  if (pCount % n) throw new Error(`${pCount} P threads and ${eCount} E cores cannot be split evenly over ${n} slots`);
  const size = pCount / n;
  if (size % 2 || pFirst % 2) throw new Error(`${n} slots would split a hyper-threading pair of the P cores`);
  return Array.from({ length: n }, (_, k) => `${pFirst + k * size}-${pFirst + (k + 1) * size - 1},${eFirst + k * size}-${eFirst + (k + 1) * size - 1}`);
}

// the layout slotCpuSets assumes, read from sysfs: cpu_core = 0-15 with HT sibling pairs (2i, 2i+1), cpu_atom = 16-31 without SMT. Returns the list of problems (empty = as assumed).
export function checkCpuTopology(sysRoot = '/sys/devices') {
  const problems = [];
  const read = (f) => { try { return fs.readFileSync(path.join(sysRoot, f), 'utf8').trim(); } catch { return null; } };
  const core = read('cpu_core/cpus');
  const atom = read('cpu_atom/cpus');
  if (core !== `${CPU_P_FIRST}-${CPU_P_LAST}`) problems.push(`cpu_core/cpus is ${core}, expected ${CPU_P_FIRST}-${CPU_P_LAST}`);
  if (atom !== `${CPU_E_FIRST}-${CPU_E_LAST}`) problems.push(`cpu_atom/cpus is ${atom}, expected ${CPU_E_FIRST}-${CPU_E_LAST}`);
  for (let c = CPU_P_FIRST; c <= CPU_P_LAST; c++) {
    const sib = read(`system/cpu/cpu${c}/topology/thread_siblings_list`);
    const lo = c - (c % 2);
    if (sib !== `${lo}-${lo + 1}`) problems.push(`cpu${c} thread_siblings_list is ${sib}, expected ${lo}-${lo + 1}`);
  }
  for (let c = CPU_E_FIRST; c <= CPU_E_LAST; c++) {
    const sib = read(`system/cpu/cpu${c}/topology/thread_siblings_list`);
    if (sib !== String(c)) problems.push(`cpu${c} thread_siblings_list is ${sib}, expected ${c}`);
  }
  return problems;
}

// /proc/stat -> per-cpu {busy, total} jiffies (busy = everything but idle and iowait)
export function parseProcStat(text) {
  const per = [];
  for (const line of text.split('\n')) {
    const m = /^cpu(\d+) (.*)$/.exec(line);
    if (!m) continue;
    const v = m[2].trim().split(/\s+/).map(Number);
    per[Number(m[1])] = { busy: v[0] + v[1] + v[2] + v[5] + v[6] + (v[7] ?? 0), total: v.slice(0, 8).reduce((a, b) => a + b, 0) };
  }
  return per;
}
// busy fraction of a CPU set between two /proc/stat samples
export function cpuSetUtil(a, b, cpus) {
  let busy = 0;
  let total = 0;
  for (const c of cpus) { busy += b[c].busy - a[c].busy; total += b[c].total - a[c].total; }
  return total > 0 ? busy / total : 0;
}
const sampleCpuUtil = async (cpuList, windowS) => {
  const cpus = parseCpuList(cpuList);
  const a = parseProcStat(fs.readFileSync('/proc/stat', 'utf8'));
  await sleep(windowS);
  return cpuSetUtil(a, parseProcStat(fs.readFileSync('/proc/stat', 'utf8')), cpus);
};

export const loadLimit = (othersActive) => LOAD_MAX + L_SLOT * othersActive;
// the whole readiness decision as a pure function: lane-precheck exit 0, 1-min load <= LOAD_MAX + L_SLOT * others, and (slot CPU set given) its utilisation <= SLOT_CPU_UTIL_MAX
export function readiness({ precheckExit, load1, othersActive = 0, cpuUtil = null }) {
  const limit = loadLimit(othersActive);
  const precheckOk = precheckExit === 0;
  const loadOk = load1 <= limit;
  const cpuOk = cpuUtil === null || cpuUtil <= SLOT_CPU_UTIL_MAX;
  return { ok: precheckOk && loadOk && cpuOk, precheckOk, loadOk, cpuOk, loadLimit: limit };
}

// headless Chromium MAIN processes (--headless, no --type=) that are, or descend from, one of rootPids: the gate's own browsers. A foreign one is not counted and still blocks.
export function ownHeadless(table, rootPids) {
  const roots = new Set(rootPids);
  let n = 0;
  for (const [pid, v] of table) {
    if (!v.args.includes('--headless') || v.args.includes('--type=')) continue;
    let q = pid;
    for (let hop = 0; hop < 16 && table.has(q); hop++) { if (roots.has(q)) { n++; break; } q = table.get(q).ppid; }
  }
  return n;
}

export function runPrecheck(maxHeadless = 0) {
  const env = Object.fromEntries(Object.entries(process.env).filter(([k]) => !k.startsWith('PRECHECK_')));
  env.PRECHECK_MAX_HEADLESS = String(maxHeadless);   // v2: '0'. v3: the number of THIS gate's own headless Chromium processes (0 when no other slot runs)
  env.PRECHECK_MAX_BUILDS = '0';
  const r = spawnSync('bash', [abs(PRECHECK), '--json'], { cwd: root, env, encoding: 'utf8' });
  let pre = null;
  try { pre = JSON.parse(r.stdout); } catch { pre = { unparsable: r.stdout.slice(0, 2000) }; }
  return { status: r.status, pre };
}

// v2-shaped check for one runner without slots (kept for the tests and tools that call it)
export function resourceCheck({ maxHeadless = 0 } = {}) {
  const p = runPrecheck(maxHeadless);
  const load = os.loadavg();
  const r = readiness({ precheckExit: p.status, load1: load[0], othersActive: 0 });
  return { at: new Date().toISOString(), precheckExit: p.status, precheck: p.pre, loadavg: load, loadMax: LOAD_MAX, ok: r.ok };
}

// the per-run, per-slot check (V3-6). deps are injectable for tests: ps, precheck(maxHeadless), loadavg(), cpuUtil(cpuList, windowS)
export async function slotCheck({ cpus = null, othersActive = 0, ownRoots = [], deps = {} } = {}) {
  const own = ownHeadless((deps.ps ?? psTable)(), ownRoots);
  const p = (deps.precheck ?? runPrecheck)(own);
  const load = (deps.loadavg ?? os.loadavg)();
  const cpuUtil = cpus ? await (deps.cpuUtil ?? sampleCpuUtil)(cpus, SLOT_CPU_WINDOW_S) : null;
  const r = readiness({ precheckExit: p.status, load1: load[0], othersActive, cpuUtil });
  return { at: new Date().toISOString(), precheckExit: p.status, precheck: p.pre, loadavg: load, loadMax: LOAD_MAX, loadLimit: r.loadLimit, othersActive, ownHeadless: own,
    cpuUtil, cpuUtilMax: cpus ? SLOT_CPU_UTIL_MAX : null, ok: r.ok };
}

const sleep = (s) => new Promise((resolve) => setTimeout(resolve, s * 1000));

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

export async function pool(items, jobs, fn) {
  const results = new Array(items.length);
  let next = 0;
  await Promise.all(Array.from({ length: Math.min(jobs, items.length) }, async () => {
    while (next < items.length) { const k = next++; results[k] = await fn(items[k]); }
  }));
  return results;
}

// A lock held by this process for as long as it wants (V3-4): a `flock` child that keeps the lock while its stdin is open. If this process dies in any way the pipe closes and the lock is
// released. Resolves to {file, pid, lost, release(), onLost(cb)} or null when the lock was not obtained within waitS seconds.
// C-1 (review-us025-lc-e2e-v3-runner): the exit of the flock child is remembered, so release() returns at once when the child is already gone (a `once('exit')` registered after the event
// never fires) and has a kill fallback if the child ignores the closed stdin. A child that ends BEFORE release() means the exclusion is gone: lock.lost is set and onLost callbacks run
// (the caller stops starting runs).
export const LOCK_RELEASE_KILL_S = 10;
export function holdLock(file, waitS) {
  fs.mkdirSync(path.dirname(file), { recursive: true });
  return new Promise((resolve) => {
    const p = spawn('flock', ['-w', String(waitS), file, 'bash', '-c', 'echo locked; cat >/dev/null'], { stdio: ['pipe', 'pipe', 'ignore'] });
    p.stdin.on('error', () => {});
    let got = false;
    let dead = false;
    const callbacks = [];
    const lock = {
      file, pid: p.pid, lost: false, released: false, lostWhy: null,
      onLost: (cb) => { if (lock.lost) cb(lock.lostWhy); else callbacks.push(cb); },
      release: () => new Promise((done) => {
        lock.released = true;
        if (dead) { done(); return; }
        const killer = setTimeout(() => { try { p.kill('SIGKILL'); } catch { /* gone */ } }, LOCK_RELEASE_KILL_S * 1000);
        killer.unref?.();
        p.once('exit', () => { clearTimeout(killer); done(); });
        try { p.stdin.end(); } catch { /* closed */ }
      }),
    };
    p.stdout.once('data', () => { got = true; resolve(lock); });
    p.once('exit', (code, signal) => {
      dead = true;
      if (!got) { resolve(null); return; }
      if (lock.released) return;
      lock.lost = true;
      lock.lostWhy = `lock holder of ${file} ended before the lock was released (exit ${code}, signal ${signal})`;
      for (const cb of callbacks.splice(0)) { try { cb(lock.lostWhy); } catch { /* the callback must not break the exit handler */ } }
    });
  });
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
export function supervise({ cmd, args, options = {}, capS, graceS, minCpu = RENDERER_CPU_MIN, ps = psTable, onSpawn = null }) {
  return new Promise((resolve) => {
    const child = spawn(cmd, args, { ...options, detached: true });
    onSpawn?.(child.pid);   // v3: the slot registry learns the runner pid (the own-Chromium count of the precheck)
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

// ---- one scenario's record, the global queue and phase A (V3-2, V3-3, V3-5, V3-6, V3-7) ---------------------------------------------------------
const pad2 = (i) => String(i).padStart(2, '0');
export const SCENARIO_SUBDIRS = ['runs', 'bundles', 'replays', 'prerun', 'logs', 'compare'];

// specs: [{scenario, lc1, lc0, prefix = 'gate'}] -> the queue in the registered order: scenarios in the given order, inside a scenario the lc=1 runs in label order, then the lc=0 runs (v2 order)
export function buildQueue(specs) {
  const q = [];
  for (const s of specs) {
    const prefix = s.prefix ?? 'gate';
    for (let i = 1; i <= s.lc1; i++) q.push({ scenario: s.scenario, lc: 1, name: `${prefix}-lc1-${pad2(i)}`, ...(s.tag ?? {}) });
    for (let i = 1; i <= s.lc0; i++) q.push({ scenario: s.scenario, lc: 0, name: `${prefix}-lc0-${pad2(i)}`, ...(s.tag ?? {}) });
  }
  return q;
}

// n slots pull from one queue in order; worker(item, slot, index). The slot a run gets is the slot that was free; at most n workers run at any time.
// R-1 (review): a worker that throws does not reject the pool at once (the other slots would run on unobserved, without the lock handling of the caller). The failing slot stops, no slot pulls
// a new item, every slot finishes the run it has, and the FIRST error is thrown after all slots have returned.
export async function runQueue(items, n, worker) {
  let next = 0;
  let failure = null;
  await Promise.all(Array.from({ length: Math.min(n, items.length) }, async (_, slot) => {
    while (next < items.length && !failure) {
      const k = next++;
      try { await worker(items[k], slot, k); } catch (error) { failure ??= { error }; }
    }
  }));
  if (failure) throw failure.error;
}

export const newScenarioState = (name, outDir, extra = {}) => ({ name, outDir, runs: {}, claimPromise: null, claimed: false, infraStop: null, preMarkerTimeout: false, marker: null, before: null, sensorsReady: null, sensorsDone: null, ...extra });

// phase A context. Everything the slots share. Injectable (check, startRun, claim, sleep, now) so that the scheduling can be tested without a browser.
export function newPhaseA({ states, items, nSlots, cpuSets, log = null, check = slotCheck, startRun = startRealRun, claimScenario, waitMaxS = WAIT_MAX_S, pollS = POLL_S, sleepFn = sleep, now = Date.now, extraRun = () => ({}), sensorsWaitS = 300, sensorsPollS = 0.2 }) {
  return { states, items, nSlots, cpuSets, log, check, startRun, claimScenario, waitMaxS, pollS, sleepFn, now, extraRun, sensorsWaitS, sensorsPollS, running: new Set(), roots: new Map(), stopNew: false, aborted: null, order: [] };
}

// C-1 / R-1: stop phase A without losing the markers that are already claimed. No new run starts; the runs in flight finish (a started original run is not killed here: the watchdog
// still covers it). A claimed scenario gets infraStop (its report is INFRA, exit 6); a scenario that was never claimed keeps no marker and no report (exit 5).
export function abortPhaseA(ctx, reason) {
  if (ctx.aborted) return;
  ctx.aborted = reason;
  ctx.stopNew = true;
  ctx.log?.write('phase-a-abort', { reason });
  for (const sc of ctx.states.values()) {
    if (sc.claimed || sc.claimPromise) sc.infraStop ??= `phase A aborted: ${reason}`;
    else sc.preMarkerTimeout = true;
  }
}

// a run that never reached its end (not started, or the worker threw) is INFRA of that run, not a missing record
export function settleUnfinishedRuns(states, why) {
  for (const sc of states.values()) for (const rec of Object.values(sc.runs)) {
    if (rec.attempts === undefined && !rec.notRun) { rec.infra = `not completed: ${why}`; rec.notRun = true; }
  }
}

// resolves when the file exists with a non-zero size that did not change over one poll, or after waitS seconds, or when stopped() (the owner's run ended)
export async function waitStableFile(file, waitS, pollS, stopped = () => false) {
  const t0 = Date.now();
  let last = -1;
  while ((Date.now() - t0) / 1000 < waitS && !stopped()) {
    let size = -2;
    try { size = fs.statSync(file).size; } catch { /* not there yet */ }
    if (size > 0 && size === last) return true;
    last = size;
    await sleep(pollS);
  }
  return false;
}

async function waitSlotReady(ctx, slot, what) {
  const t0 = ctx.now();
  let attempts = 0;
  for (;;) {
    const others = ctx.running.size - (ctx.running.has(slot) ? 1 : 0);
    const c = await ctx.check({ slot, cpus: ctx.cpuSets[slot], othersActive: others, ownRoots: [...ctx.roots.values()] });
    attempts++;
    const waitedS = (ctx.now() - t0) / 1000;
    if (c.ok) return { ...c, attempts, waitedS };
    ctx.log?.write('resource-wait', { what, slot, attempt: attempts, precheckExit: c.precheckExit, loadavg: c.loadavg, loadLimit: c.loadLimit, othersActive: c.othersActive, cpuUtil: c.cpuUtil, reasons: c.precheck?.blocked_reasons });
    if (ctx.aborted) return { ...c, attempts, waitedS, aborted: true };
    if (waitedS > ctx.waitMaxS) return { ...c, attempts, waitedS, timedOut: true };
    await ctx.sleepFn(ctx.pollS);
  }
}

// the first slot to pull a run of a scenario waits for readiness, then claims the marker; slots that pull a later run of the same scenario wait for that. Returns true once claimed.
async function claimScenarioOnce(ctx, sc, slot) {
  if (ctx.stopNew) return false;
  const before = await waitSlotReady(ctx, slot, `before the marker of ${sc.name}`);
  if (before.timedOut) { ctx.stopNew = true; sc.preMarkerTimeout = true; ctx.log?.write('resources-before-marker', { scenario: sc.name, ok: false, timedOut: true, attempts: before.attempts, waitedS: before.waitedS }); return false; }
  if (ctx.aborted) { sc.preMarkerTimeout = true; return false; }   // aborted while waiting: no marker
  ctx.log?.write('resources-before-marker', { scenario: sc.name, ok: true, attempts: before.attempts, waitedS: before.waitedS, loadavg: before.loadavg, slot });
  sc.before = { ok: before.ok, at: before.at, loadavg: before.loadavg, attempts: before.attempts, waitedS: before.waitedS };
  ctx.claimScenario(sc);   // may throw (HELD): the run ends with that error, nothing else is started
  sc.claimed = true;
  return true;
}

async function originalWorker(ctx, item, slot) {
  const sc = ctx.states.get(item.scenario);
  const rec = sc.runs[item.name];
  const notRun = (why) => { rec.infra = `not started: ${why}`; rec.notRun = true; };
  if (sc.infraStop) return notRun(sc.infraStop);
  if (!sc.claimPromise) sc.claimPromise = claimScenarioOnce(ctx, sc, slot);
  if (!(await sc.claimPromise)) return notRun(ctx.aborted ? `phase A aborted: ${ctx.aborted}` : ctx.stopNew ? 'resources were not ready before the marker of this scenario (no marker)' : 'scenario not claimed');
  if (sc.infraStop) return notRun(sc.infraStop);
  const outDir = sc.outDir;
  const prerunFile = path.join(outDir, 'prerun', `${item.name}.json`);
  const logFile = path.join(outDir, 'logs', `run-${item.name}.log`);
  rec.slot = slot; rec.cpus = ctx.cpuSets[slot];
  ctx.log?.write('original-run-start', { scenario: sc.name, run: item.name, lc: item.lc, slot, cpus: rec.cpus });
  const c = await waitSlotReady(ctx, slot, `before ${item.name}`);
  if (ctx.aborted) return notRun(`phase A aborted: ${ctx.aborted}`);
  let r = { status: null, signal: null, watchdog: { fired: false, killedRenderers: [], groupKilled: false } };
  const t0 = ctx.now();
  rec.startedAt = new Date().toISOString();
  if (!c.timedOut) {
    // run_lcgold.mjs writes <runs>/<scenario>/engine-input-sensors.jsonl.gz ONCE per scenario (open 'wx', EEXIST tolerated) and reads it back at once: two runs of a scenario that start together
    // could read a half-written file (its sha256File in meta.json). The first run of a scenario therefore goes alone until that file is complete; later runs wait for it (V3-10).
    const owner = !sc.sensorsReady;
    if (owner) sc.sensorsReady = new Promise((resolve) => { sc.sensorsDone = resolve; });
    else await sc.sensorsReady;
    ctx.running.add(slot);
    fs.writeFileSync(prerunFile, `${JSON.stringify({ ...c, lock: LOCK, slotLock: `${SLOT_LOCK_PREFIX}${slot}.lock`, run: item.name, slot, cpus: rec.cpus })}\n`, { flag: 'wx' });
    let ended = false;
    try {
      const running = ctx.startRun({ scenario: sc.name, name: item.name, lc: item.lc, runsRoot: rel(path.join(outDir, 'runs')), logFile, cpus: rec.cpus, slot, onSpawn: (pid) => ctx.roots.set(slot, pid), extra: ctx.extraRun(item) });
      if (owner) waitStableFile(path.join(path.dirname(abs(rec.dir)), 'engine-input-sensors.jsonl.gz'), ctx.sensorsWaitS, ctx.sensorsPollS, () => ended).then(() => sc.sensorsDone());
      r = await running;
    } finally { ended = true; ctx.running.delete(slot); ctx.roots.delete(slot); if (owner) sc.sensorsDone(); }
  }
  rec.endedAt = new Date().toISOString();
  const seconds = (ctx.now() - t0) / 1000;
  const dir = abs(rec.dir);
  let status = 'NO-META';
  let loadAvgAtStart = null;
  try { const m = JSON.parse(fs.readFileSync(path.join(dir, 'meta.json'), 'utf8')); status = m.status; loadAvgAtStart = m.loadAvgAtStart ?? null; } catch { /* NO-META */ }
  const prerun = fs.existsSync(prerunFile) ? JSON.parse(fs.readFileSync(prerunFile, 'utf8')) : null;
  const hasInputs = fs.existsSync(path.join(dir, 'recorded-inputs.jsonl.gz')) && fs.existsSync(path.join(dir, 'memprobe.jsonl.gz'));
  const runnerExit = r.status ?? 1;
  rec.runnerExit = c.timedOut ? null : runnerExit; rec.signal = r.signal ?? null; rec.seconds = seconds; rec.attempts = 1; rec.status = status; rec.loadAvgAtStart = loadAvgAtStart;
  rec.prerun = prerun ? { ok: prerun.ok, at: prerun.at, loadavg: prerun.loadavg, precheckExit: prerun.precheckExit, loadLimit: prerun.loadLimit, othersActive: prerun.othersActive, cpuUtil: prerun.cpuUtil, file: rel(prerunFile) } : { ok: false };
  if (r.watchdog.fired) {
    rec.watchdog = { ...r.watchdog, runnerStatus: r.status, runnerSignal: r.signal, capS: RUN_WALL_MAX_S };
    fs.writeFileSync(`${prerunFile}.watchdog.json`, `${JSON.stringify(rec.watchdog)}\n`, { flag: 'wx' });
  } else rec.watchdog = null;
  rec.infra = originalRunInfra({ runnerExit, status, hasInputs, timedOut: c.timedOut, prerunOk: !!prerun, watchdog: rec.watchdog });
  ctx.log?.write('original-run-end', { scenario: sc.name, run: item.name, slot, exit: runnerExit, status, seconds, infra: rec.infra, timedOut: c.timedOut ?? false });
  if (c.timedOut || !prerun) sc.infraStop = `resources or lock not available for ${item.name}`;   // resources never ready: the remaining runs of this scenario cannot start
  ctx.order.push(item.name);
}

// the real start of one original run: the runner under the watchdog, pinned to the slot's CPU set when there is one
export function runnerCommand(job) {
  const runner = [abs(RUNNER), '--scenario', job.scenario, '--label', job.name, '--out-root', job.runsRoot, ...(job.lc === 1 ? RUN_OPTIONS_LC1 : RUN_OPTIONS_LC0), '--poke', JSON.stringify({ lc: job.lc })];
  return job.cpus ? ['taskset', ['-c', job.cpus, process.execPath, ...runner]] : [process.execPath, runner];   // no CPU set (SLOTS = 1) = the v2 command, no taskset
}
export function startRealRun(job) {
  const env = { ...process.env };
  delete env.BROWSER_EXECUTABLE;
  const [cmd, args] = runnerCommand(job);
  const fd = fs.openSync(job.logFile, 'wx');
  return supervise({ cmd, args, capS: RUN_WALL_MAX_S, graceS: WATCHDOG_GRACE_S, onSpawn: job.onSpawn, options: { cwd: root, env, stdio: ['ignore', fd, fd] } }).finally(() => fs.closeSync(fd));
}

export async function runPhaseA(ctx) {
  await runQueue(ctx.items, ctx.nSlots, (item, slot) => originalWorker(ctx, item, slot));
}

// ---- phase B (V3-2, V3-8): bundles serial, replays and comparators in a pool of J over ALL scenarios --------------------------------------------
export const EXIT_OF = { PASS: 0, PASS_DECLARED: 0, FAIL: 10, INVALID: 11, BLOCKED: 13, ORIGINAL_UNUSABLE: 14 };
const harnessCause = (file) => { try { const t = fs.readFileSync(file, 'utf8').slice(-20000); return HARNESS_SIGNATURES.some((x) => t.includes(x)); } catch { return false; } };

// default bundle step of the gate: one make_bundles.sh call per usable original run (as v2). Returns the replay jobs of that run.
export async function makeBundlesFor(sc, rec, { datumBaseDir, log }) {
  const outDir = sc.outDir;
  const benv = { ...process.env, US026_RUNS: rel(path.join(outDir, 'runs')), US026_BUNDLES: rel(path.join(outDir, 'bundles')), US026_BASE_ROOT: rel(path.join(outDir, 'bundles-base-unused')), US026_BASE: datumBaseDir };
  const variants = rec.lc === 1 ? [`on=${ON_PARAMS}`] : [];
  const r = await runChild('bash', [abs(MAKE_BUNDLES), sc.name, rec.name, ...variants], { logFile: path.join(outDir, 'logs', `bundles-${rec.name}.log`), env: benv });
  log?.write('bundles', { scenario: sc.name, run: rec.name, exit: r.exit });
  rec.bundles = { makeExit: r.exit };
  const off = path.join(outDir, 'bundles', sc.name, rec.name);
  const on = path.join(outDir, 'bundles', sc.name, `${rec.name}-on`);
  if (r.exit !== 0 || !fs.existsSync(off) || (rec.lc === 1 && !fs.existsSync(on))) { rec.infra = `bundle construction failed (exit ${r.exit})`; return []; }
  return rec.lc === 1 ? [{ run: rec.name, arm: 'off', bundle: off }, { run: rec.name, arm: 'on', bundle: on }] : [{ run: rec.name, arm: 'null', bundle: off }];
}

// classification in plan order (judge.mjs; the same calls as v2) of ONE scenario
export function classifyScenarioRuns(sc) {
  sc.lc1 = [];
  sc.lc0 = [];
  for (const rec of Object.values(sc.runs)) {
    if (rec.infra) { (rec.lc === 1 ? sc.lc1 : sc.lc0).push(rec.lc === 1 ? classifyLc1Run({ name: rec.name, infra: rec.infra }) : classifyLc0Run({ name: rec.name, infra: rec.infra })); continue; }
    if (rec.lc === 1) sc.lc1.push(classifyLc1Run({ name: rec.name, infra: null, replays: { on: rec.replays?.on, off: rec.replays?.off }, on: rec.compare.on, off: rec.compare.off, full: rec.compare.full }));
    else sc.lc0.push(classifyLc0Run({ name: rec.name, infra: null, replay: rec.replays?.null, null: rec.compare.null }));
  }
}

// ctx: {states: [sc...] (in order), python, jobs, log, bundlesFor(sc, rec) -> replay jobs, onScenarioDone(sc) -> optional, called once per scenario as soon as ITS comparators are done}
// C-3 (review): one scenario's exception must not stop the replays, the classification or the report of another scenario (the markers of all started scenarios are already consumed). Every
// step below catches its own error, records it as `phase B error` INFRA of that run (and of the scenario: sc.infraStop), and goes on; the pools never reject.
export async function runPhaseB(ctx) {
  const { states, python, jobs, log } = ctx;
  const softFail = (sc, rec, step, error) => {
    const why = `phase B error (${step}): ${String(error?.message ?? error)}`;
    if (rec) rec.infra = why;
    sc.phaseBErrors = [...(sc.phaseBErrors ?? []), { run: rec?.name ?? null, step, message: String(error?.message ?? error) }];
    sc.infraStop ??= why;
    log?.write('phase-b-error', { scenario: sc.name, run: rec?.name ?? null, step, message: String(error?.message ?? error) });
  };
  const replayJobs = [];
  for (const sc of states) {
    for (const rec of Object.values(sc.runs)) {
      if (rec.infra) continue;
      try { for (const j of await ctx.bundlesFor(sc, rec)) replayJobs.push({ ...j, sc }); } catch (error) { softFail(sc, rec, 'bundles', error); }
    }
  }
  // engine replays with the pinned binary, environment scrubbed to PATH. SIGKILL is retried once (REPLAY_RETRY_ON_SIGKILL); a log with ENOSPC marks a harness cause.
  const replayEnv = { PATH: process.env.PATH ?? '/usr/bin:/bin' };
  const res = await pool(replayJobs, jobs, async (j) => {
    try {
      const outDir = j.sc.outDir;
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
    } catch (error) { return { ...j, out: null, retried: 0, harness: true, exit: null, signal: null, seconds: 0, failure: error }; }
  });
  for (const r of res) {
    const rec = r.sc.runs[r.run];
    if (r.failure) { softFail(r.sc, rec, `replay ${r.arm}`, r.failure); continue; }
    rec.replays ??= {};
    rec.replays[r.arm] = { dir: rel(r.out), exit: r.exit, signal: r.signal ?? null, seconds: r.seconds, retried: r.retried, harness: r.harness };
  }
  log?.write('replays', { jobs, exits: Object.fromEntries(res.map((r) => [`${r.sc.name}/${r.run}-${r.arm}`, r.exit])) });
  // the comparator: one process per run and arm; arm `full` (v2, D-V2-2) is compare_full_us025.py over the engine ON replay, the other arms are compare_us025.py
  const cmp = async (sc, rec, arm) => {
    const rp = rec.replays?.[arm === 'full' ? 'on' : arm];
    const outFile = path.join(sc.outDir, 'compare', `${rec.name}-${arm}.json`);
    const r = await runChild(python, ['-B', '-I', abs(arm === 'full' ? COMPARE_FULL : COMPARE), arm, '--original', abs(rec.dir), '--engine', abs(rp?.dir ?? 'does-not-exist'), '--out', outFile],
      { logFile: path.join(sc.outDir, 'logs', `compare-${rec.name}-${arm}.log`) });
    let result = null;
    if (fs.existsSync(outFile)) { try { result = JSON.parse(fs.readFileSync(outFile, 'utf8')); } catch { result = null; } }
    // exit 4 (TOOL_ERROR), 1 (unhandled Python exception), a missing/garbled output file, or a verdict that disagrees with the exit code is a TOOL_ERROR
    // of this run and arm, never a FAIL (final-a-review-harness M-3)
    if (!result || EXIT_OF[result.verdict] !== r.exit) return { verdict: 'TOOL_ERROR', reason: `${arm === 'full' ? 'compare_full_us025.py' : 'compare_us025.py'} ${arm} exit ${r.exit}${result ? `, verdict ${result.verdict}` : ', no output'}`, checks: [] };
    return result;
  };
  const tasks = states.flatMap((sc) => Object.values(sc.runs).filter((rec) => !rec.infra).map((rec) => ({ sc, rec })));
  const remaining = new Map(states.map((sc) => [sc, tasks.filter((t) => t.sc === sc).length]));
  const finish = async (sc) => {
    try { classifyScenarioRuns(sc); } catch (error) { sc.lc1 ??= []; sc.lc0 ??= []; softFail(sc, null, 'classification', error); }
    try { await ctx.onScenarioDone?.(sc); } catch (error) { softFail(sc, null, 'report', error); }
  };
  await pool(tasks, jobs, async ({ sc, rec }) => {
    try {
      if (rec.lc === 1) {
        // a replay that did not exit 0 is NOT compared: judge.mjs turns it into FAIL (US025-M1/M2) or, for a harness cause, INFRA
        const on = rec.replays?.on?.exit === 0 ? await cmp(sc, rec, 'on') : null;
        const off = rec.replays?.off?.exit === 0 ? await cmp(sc, rec, 'off') : null;
        // D-V2-2: an ON arm that is BLOCKED (the original never succeeded) is followed by the whole-run comparison of the engine ON replay
        const full = on?.verdict === 'BLOCKED' ? await cmp(sc, rec, 'full') : null;
        rec.originalRun = on?.originalRun ?? null;
        rec.compare = { on: compactResult(on), off: compactResult(off), full: compactResult(full) };
      } else {
        const nl = rec.replays?.null?.exit === 0 ? await cmp(sc, rec, 'null') : null;
        rec.originalRun = nl?.originalRun ?? null;
        rec.compare = { null: compactResult(nl) };
      }
    } catch (error) { softFail(sc, rec, 'comparator', error); }
    remaining.set(sc, remaining.get(sc) - 1);
    if (remaining.get(sc) === 0) await finish(sc);   // this scenario is complete: classify it and (real run) write its report now
  });
  for (const sc of states) if (remaining.get(sc) === 0 && !sc.lc1) await finish(sc);   // scenarios without any usable run
}

// ---- the real gate run: run-all (V3-2 .. V3-8) -------------------------------------------------------------------------------------------------
const heldArgs = (why) => held('args', why);
export function checkSlotsAndJobs(values) {
  const nSlots = Number(values.slots ?? SLOTS);
  if (nSlots !== SLOTS) throw heldArgs(`--slots ${values.slots} differs from the registered SLOTS = ${SLOTS} (the real run does not take another value)`);
  const jobs = Number(values.jobs ?? REPLAY_JOBS_DEFAULT);
  if (!Number.isInteger(jobs) || jobs < 1 || jobs > REPLAY_JOBS_MAX) throw new Error(`--jobs must be an integer 1..${REPLAY_JOBS_MAX}`);
  return { nSlots, jobs };
}

export async function cmdRunAll(values, deps = {}) {
  const names = (values.scenarios ?? values.scenario ?? '').split(',').filter(Boolean);
  if (!names.length || names.some((s) => !/^[a-z0-9-]+$/.test(s))) throw new Error('--scenario <name> (or --scenarios a,b,...)');
  if (new Set(names).size !== names.length) throw new Error('--scenarios: a scenario is listed twice');
  const outRoot = abs(values['out-root'] ?? OUT_DEFAULT);
  if (!(outRoot + path.sep).startsWith(abs(OUT_ALLOWED) + path.sep)) throw new Error(`--out-root must be under ${OUT_ALLOWED} (run_lcgold.mjs writes only there)`);
  const gateRoot = path.join(outRoot, GATE_ID);
  if (!values['run-log']) throw new Error('--run-log <new file outside the output directory> is required');
  const log = new RunLog(abs(values['run-log']), { out: gateRoot });
  const startedAt = new Date().toISOString();
  log.write('start', { argv: process.argv.slice(2), node: process.version, gate: 'lc', scenarios: names, env: envRecord(process.env, values['designation-env'] ? [values['designation-env']] : []) });
  const locks = [];
  try {
    const { nSlots, jobs } = checkSlotsAndJobs(values);
    const gate = gateArgs(values, root);
    const holds = new Map();
    for (const scenario of names) {
      try { holds.set(scenario, (deps.checkHold ?? checkHold)({ gate, scenario, outDir: path.join(gateRoot, scenario) })); } catch (error) { log.write('held', { scenario, message: error.message }); throw error; }
      log.write('hold-checks', { scenario, lines: holds.get(scenario).log });
    }
    const python = holds.get(names[0]).python;
    const cpuSets = slotCpuSets(nSlots);
    if (cpuSets[0] !== null && os.cpus().length <= SLOT_CPU_LAST) throw held('args', `${os.cpus().length} cpus: the slot CPU sets need cores up to ${SLOT_CPU_LAST}`);
    if (cpuSets[0] !== null) { const topo = (deps.checkTopology ?? checkCpuTopology)(); if (topo.length) throw held('args', `CPU topology differs from the slot CPU sets' assumption: ${topo.slice(0, 3).join('; ')}`); }
    // ---- locks (V3-4): the Chromium lock for the whole of phase A, one slot lock per slot
    const lock = await (deps.holdLock ?? holdLock)(abs(LOCK), WAIT_MAX_S);
    if (!lock) { log.write('lock-timeout', { lock: LOCK }); log.close('NOT-READY'); return { exit: 5, why: `${LOCK} not obtained (no marker created)` }; }
    locks.push(lock);
    for (let k = 0; k < nSlots; k++) {
      const sl = await (deps.holdLock ?? holdLock)(abs(`${SLOT_LOCK_PREFIX}${k}.lock`), WAIT_MAX_S);
      if (!sl) { log.write('lock-timeout', { lock: `${SLOT_LOCK_PREFIX}${k}.lock` }); log.close('NOT-READY'); return { exit: 5, why: `slot lock ${k} not obtained (no marker created)` }; }
      locks.push(sl);
    }
    log.write('locks-held', { chromium: LOCK, slots: nSlots, cpuSets });
    // ---- the global queue
    const specs = names.map((scenario) => ({ scenario, lc1: SCENARIO_RUNS[scenario].lc1, lc0: SCENARIO_RUNS[scenario].lc0 }));
    const items = buildQueue(specs);
    const states = new Map();
    for (const scenario of names) {
      const outDir = path.join(gateRoot, scenario);
      const sc = newScenarioState(scenario, outDir, { hold: holds.get(scenario), planned: SCENARIO_RUNS[scenario] });
      for (const it of items.filter((i) => i.scenario === scenario)) sc.runs[it.name] = { name: it.name, lc: it.lc, dir: rel(path.join(outDir, 'runs', scenario, it.name)) };
      states.set(scenario, sc);
    }
    const ctx = newPhaseA({ states, items, nSlots, cpuSets, log, check: deps.check ?? slotCheck, startRun: deps.startRun ?? startRealRun,
      claimScenario: (sc) => {
        const outcome = (deps.claim ?? claim)(sc.hold.rq);
        sc.marker = outcome.marker; sc.designationSha256 = outcome.designationSha256;
        log.write('first-run-claimed', { marker: rel(sc.marker), scenario: sc.name, designationSha256: outcome.designationSha256 });
        fs.mkdirSync(sc.outDir, { recursive: true });
        for (const d of SCENARIO_SUBDIRS) fs.mkdirSync(path.join(sc.outDir, d));
      } });
    // C-1: a lock holder that ends before the release means the exclusion is gone: stop starting runs (the runs in flight finish)
    for (const l of locks) l.onLost?.((why) => { log.write('lock-lost', { lock: l.file, why }); abortPhaseA(ctx, why); });
    const phaseAStart = new Date().toISOString();
    log.write('phase-a-start', { items: items.length, slots: nSlots, queue: items.map((i) => `${i.scenario}/${i.name}`) });
    const errors = [];
    // R-1: a worker error (claim HELD, a write that fails) stops new work, lets the other slots finish their run, then arrives here. The markers already claimed are not wasted: the reports are written (INFRA).
    try { await runPhaseA(ctx); } catch (error) {
      log.write('phase-a-error', { message: String(error?.message ?? error) });
      errors.push(`phase A: ${String(error?.message ?? error)}`);
      abortPhaseA(ctx, `phase A error: ${String(error?.message ?? error)}`);
    }
    settleUnfinishedRuns(states, ctx.aborted ?? 'phase A ended before this run');
    const phaseAEnd = new Date().toISOString();
    for (const l of locks.splice(0).reverse()) await l.release();   // phase B never overlaps phase A and holds no Chromium lock
    log.write('phase-a-end', { at: phaseAEnd, order: ctx.order, locksReleased: true, aborted: ctx.aborted });
    // ---- before phase B (D-V3-7, review R-2): the pinned files, the engine pin and the freeze paths are re-checked; phase A takes hours. A drift means the pinned tools changed
    // under the run: no replay is made with them. Every started scenario gets its report with the drift as an infrastructure failure (the marker is consumed either way).
    const started = [...states.values()].filter((s) => s.claimed);
    let drift = [];
    if (started.length) {
      try { drift = (deps.recheck ?? recheckPins)(started[0].hold.manifest); } catch (error) { drift = [`re-check failed: ${String(error?.message ?? error)}`]; }
      log.write('pin-recheck', { ok: !drift.length, problems: drift });
    }
    if (drift.length) {
      for (const sc of started) {
        sc.infraStop ??= `pin/freeze drift before phase B: ${drift.join('; ')}`;
        for (const rec of Object.values(sc.runs)) if (!rec.infra) rec.infra = `phase B not run: ${sc.infraStop}`;
      }
    }
    // ---- phase B, with the report of every scenario written as soon as that scenario is complete (C-3)
    const phaseBStart = new Date().toISOString();
    log.write('phase-b-start', { scenarios: started.map((s) => s.name), jobs, skipped: drift.length > 0 });
    const reported = [];
    const writeReport = async (sc) => {
      if (sc.reported || sc.reportError) return;
      const result = classifyScenario({ scenario: sc.name, lc1: sc.lc1 ?? [], lc0: sc.lc0 ?? [], planned: sc.planned, infraFailure: sc.infraStop });
      const pins = Object.fromEntries(livePinnedFiles({ withBinary: true }).map(([f, s]) => [path.basename(f), s]));
      const report = { schema: 'slam-refact-us025-lc-e2e-report-v3', gate: { id: GATE_ID, version: GATE_VERSION, key: 'lc' }, kind: 'gate', scenario: sc.name, startedAt,
        finishedAt: new Date().toISOString(), argv: process.argv.slice(2), versions: versions(), trees: liveTrees(), pins, marker: rel(sc.marker), datum: sc.hold.datum,
        resourcesBeforeMarker: sc.before,
        execution: { mode: 'v3-two-phase', slots: nSlots, cpuSets, replayJobs: jobs, loadMax: LOAD_MAX, lSlot: L_SLOT, slotCpuUtilMax: SLOT_CPU_UTIL_MAX, scenarios: names, phaseA: { startedAt: phaseAStart, endedAt: phaseAEnd, aborted: ctx.aborted }, phaseB: { startedAt: phaseBStart, endedAt: new Date().toISOString(), endedAtMeaning: 'when THIS scenario was complete (reports are written per scenario)', skippedForPinDrift: drift.length > 0 }, pinRecheck: { ok: !drift.length, problems: drift }, phaseBErrors: sc.phaseBErrors ?? [], startOrder: ctx.order },
        planned: sc.planned, runs: sc.runs, classified: { lc1: sc.lc1 ?? [], lc0: sc.lc0 ?? [] }, infraFailure: sc.infraStop, result,
        gitHead: gitReal('rev-parse', 'HEAD').out, enginePin: sc.hold.manifest.engine_pin };
      const mreg = sc.hold.manifest.review ?? {};
      report.registeredCopies = copyInto(sc.outDir, 'registered', [gateMdPath(gate), manifestPath(gate), path.join(gate.preregDir, mreg.designation_file), path.join(gate.preregDir, mreg.file), sc.marker]);
      fs.writeFileSync(path.join(sc.outDir, 'report.json'), `${JSON.stringify(report, null, 1)}\n`, { flag: 'wx' });
      const sums = writeSums(sc.outDir);
      const entries = makeReadOnly(sc.outDir);
      readOnlyFiles([gateMdPath(gate)]);
      sc.reported = true;
      log.write('finished', { scenario: sc.name, report: rel(path.join(sc.outDir, 'report.json')), result, sums, readOnlyEntries: entries, infraFailure: sc.infraStop });
      console.log(JSON.stringify({ scenario: sc.name, result, report: rel(path.join(sc.outDir, 'report.json')), infraFailure: sc.infraStop }));
      reported.push(sc.name);
    };
    const safeReport = async (sc) => {
      try { await writeReport(sc); } catch (error) {
        sc.reportError = String(error?.message ?? error);
        log.write('report-error', { scenario: sc.name, message: sc.reportError });
        errors.push(`report ${sc.name}: ${sc.reportError}`);
      }
    };
    const onScenarioDone = async (sc) => { await safeReport(sc); };
    if (drift.length) {
      for (const sc of started) { try { classifyScenarioRuns(sc); } catch { sc.lc1 = []; sc.lc0 = []; } await safeReport(sc); }
    } else {
      try { await (deps.runPhaseB ?? runPhaseB)({ states: started, python, jobs, log, onScenarioDone, bundlesFor: (sc, rec) => makeBundlesFor(sc, rec, { datumBaseDir: sc.hold.datum.base_dir, log }) }); } catch (error) {
        // not expected (the real runPhaseB catches per step): every scenario that has no report yet gets one as an infrastructure failure
        log.write('phase-b-fatal', { message: String(error?.message ?? error) });
        errors.push(`phase B: ${String(error?.message ?? error)}`);
        for (const sc of started) sc.infraStop ??= `phase B failed: ${String(error?.message ?? error)}`;
      }
      for (const sc of started) {   // a scenario the phase B function did not report (it returned early, or an injected runPhaseB)
        if (sc.reported || sc.reportError) continue;
        if (!sc.lc1) { try { classifyScenarioRuns(sc); } catch { sc.lc1 = []; sc.lc0 = []; } }
        await safeReport(sc);
      }
    }
    log.write('phase-b-end', { at: new Date().toISOString(), reported, errors });
    const anyInfra = started.some((s) => s.infraStop);
    const preMarker = [...states.values()].some((s) => s.preMarkerTimeout);
    const exit = errors.length ? 1 : preMarker ? 5 : anyInfra ? 6 : 0;
    log.close(errors.length ? 'ERROR' : preMarker ? 'NOT-READY' : anyInfra ? 'INFRA-FAILURE' : 'OK');
    return { exit, scenarios: reported, errors };
  } catch (error) {
    if (!log.closed) { log.write('error', { message: String(error?.message ?? error) }); log.close(String(error?.message ?? error).startsWith('HELD:') ? 'HELD' : 'ERROR'); }
    throw error;
  } finally { for (const l of locks.splice(0).reverse()) await l.release(); }
}

export const cmdRun = (values, deps) => cmdRunAll({ ...values, scenarios: values.scenario }, deps);

// ---- dev mode (V3-9): no hold, no marker, no preregistration; output only below DEV_ROOT. For the equivalence experiment and the phase A smoke. ----------------
export function parseBlocks(spec) {
  const blocks = (spec ?? '').split(',').filter(Boolean).map((b) => {
    const m = /^(S|P(\d+)):(\d+)$/.exec(b);
    if (!m) throw new Error(`--blocks: "${b}" is not S:<runs> or P<slots>:<runs>`);
    const n = m[1] === 'S' ? 1 : Number(m[2]);
    if (m[1] !== 'S' && n === 1) throw new Error('--blocks: P1 is the arm S');
    if (!SLOTS_SUPPORTED.includes(n)) throw new Error(`--blocks: ${n} slots not supported (${SLOTS_SUPPORTED.join(',')})`);
    return { arm: m[1] === 'S' ? 'S' : `P${n}`, n, count: Number(m[3]) };
  });
  if (!blocks.length) throw new Error('--blocks S:6,P2:6,... is required');
  return blocks;
}

const quantile = (arr, p) => { const s = arr.filter((x) => typeof x === 'number').sort((a, b) => a - b); return s.length ? s[Math.min(s.length - 1, Math.floor(p * s.length))] : null; };
const mean = (arr) => { const s = arr.filter((x) => typeof x === 'number'); return s.length ? s.reduce((a, b) => a + b, 0) / s.length : null; };

// what the equivalence experiment compares for one original run (preregistered item d): runner seconds, frames / fresh callbacks / NO_FRESH_CALLBACK, load at start
export function runMetrics(runDir) {
  let m = null;
  try { m = JSON.parse(fs.readFileSync(path.join(runDir, 'meta.json'), 'utf8')); } catch { return null; }
  const s = m.summary ?? {};
  return { status: m.status ?? null, wallSeconds: m.wallSeconds ?? null, inputFrames: s.inputFrames ?? null, publishedFrames: s.publishedFrames ?? null, freshCallbacks: s.freshCallbacks ?? null,
    noFreshCallback: s.statusCounts?.NO_FRESH_CALLBACK ?? 0, statusCounts: s.statusCounts ?? null, publishToCallbackMsP50: s.publishToCallbackMs?.p50 ?? null, publishToCallbackMsP95: s.publishToCallbackMs?.p95 ?? null,
    loadAvgAtStart: m.loadAvgAtStart ?? null, loadAvgAtEnd: m.loadAvgAtEnd ?? null };
}

export function summarizeDev(states, blocks) {
  const runs = [];
  for (const sc of states) {
    for (const rec of Object.values(sc.runs)) {
      const met = rec.notRun ? null : runMetrics(abs(rec.dir));
      runs.push({ scenario: sc.name, run: rec.name, block: rec.block, arm: rec.arm, lc: rec.lc, slot: rec.slot ?? null, cpus: rec.cpus ?? null, startedAt: rec.startedAt ?? null, endedAt: rec.endedAt ?? null,
        runnerSeconds: rec.seconds ?? null, runnerExit: rec.runnerExit ?? null, status: rec.status ?? null, infra: rec.infra ?? null, watchdog: rec.watchdog ? { killedRenderers: rec.watchdog.killedRenderers?.length ?? 0, groupKilled: !!rec.watchdog.groupKilled } : null,
        loadAvgAtStart: rec.loadAvgAtStart ?? null, startCheck: rec.prerun ?? null, metrics: met,
        originalSuccess: rec.originalRun?.success ?? null, originalAttempts: rec.originalRun?.attempts ?? null,
        verdicts: { on: rec.compare?.on?.verdict ?? null, off: rec.compare?.off?.verdict ?? null, full: rec.compare?.full?.verdict ?? null, null: rec.compare?.null?.verdict ?? null },
        replaySeconds: Object.fromEntries(Object.entries(rec.replays ?? {}).map(([a, v]) => [a, v.seconds])) });
    }
  }
  const byArm = {};
  for (const arm of [...new Set(blocks.map((b) => b.arm))]) {
    for (const scenario of [...new Set(runs.map((r) => r.scenario))]) {
      const rs = runs.filter((r) => r.arm === arm && r.scenario === scenario && r.lc === 1);
      if (!rs.length) continue;
      const usable = rs.filter((r) => !r.infra);
      const succ = usable.filter((r) => r.originalSuccess === true);
      (byArm[arm] ??= {})[scenario] = {
        lc1Runs: rs.length, usable: usable.length, infra: rs.filter((r) => r.infra).length, watchdogRuns: rs.filter((r) => r.watchdog).length,
        lcSuccessRuns: succ.length, lcSuccessOfUsable: usable.length ? succ.length / usable.length : null,
        onVerdicts: countBy(usable.map((r) => r.verdicts.on)), offVerdicts: countBy(usable.map((r) => r.verdicts.off)), fullVerdicts: countBy(usable.map((r) => r.verdicts.full).filter(Boolean)),
        onFailOnSuccess: succ.filter((r) => r.verdicts.on !== 'PASS' && r.verdicts.on !== 'PASS_DECLARED').length,
        runnerSeconds: { n: rs.length, p10: quantile(rs.map((r) => r.runnerSeconds), 0.1), p50: quantile(rs.map((r) => r.runnerSeconds), 0.5), p90: quantile(rs.map((r) => r.runnerSeconds), 0.9), mean: mean(rs.map((r) => r.runnerSeconds)) },
        framesInput: countBy(usable.map((r) => r.metrics?.inputFrames)), freshCallbacks: { mean: mean(usable.map((r) => r.metrics?.freshCallbacks)), min: quantile(usable.map((r) => r.metrics?.freshCallbacks), 0) },
        noFreshCallback: { sum: usable.reduce((a, r) => a + (r.metrics?.noFreshCallback ?? 0), 0), mean: mean(usable.map((r) => r.metrics?.noFreshCallback)) },
        loadAtStart1min: { mean: mean(rs.map((r) => r.loadAvgAtStart?.[0])), max: Math.max(...rs.map((r) => r.loadAvgAtStart?.[0] ?? 0)) },
        publishToCallbackMsP50: { mean: mean(usable.map((r) => r.metrics?.publishToCallbackMsP50)) },
      };
    }
  }
  return { runs, byArm };
}
const countBy = (arr) => arr.reduce((o, x) => { const k = String(x); o[k] = (o[k] ?? 0) + 1; return o; }, {});

export async function cmdDev(values, deps = {}) {
  const names = (values.scenarios ?? '').split(',').filter(Boolean);
  const known = JSON.parse(fs.readFileSync(abs('slam_refact/tools/re/runtime/scenarios.json'), 'utf8'));
  if (!names.length || names.some((s) => !/^[a-z0-9-]+$/.test(s) || !known[s])) throw new Error('--scenarios a,b (names from scenarios.json)');
  const outRoot = abs(values['out-root'] ?? '');
  if (!values['out-root'] || !(outRoot + path.sep).startsWith(abs(DEV_ROOT) + path.sep)) throw new Error(`--out-root must be below ${DEV_ROOT}`);
  if (!values['run-log']) throw new Error('--run-log <new file outside the output directory> is required');
  const blocks = parseBlocks(values.blocks);
  const lc0 = Number(values.lc0 ?? 0);
  if (!Number.isInteger(lc0) || lc0 < 0) throw new Error('--lc0 K (null-arm runs per scenario per block)');
  // dev only: the core range of the slot CPU sets (default DEV_CPU_FIRST..DEV_CPU_LAST = 0..31, C-2; the real run uses SLOT_CPU_FIRST..SLOT_CPU_LAST), recorded in the summary
  const cpuFirst = Number(values['cpu-first'] ?? DEV_CPU_FIRST);
  const cpuLast = Number(values['cpu-last'] ?? DEV_CPU_LAST);
  if (!Number.isInteger(cpuFirst) || !Number.isInteger(cpuLast) || cpuFirst < 0 || cpuLast < cpuFirst) throw new Error('--cpu-first / --cpu-last');
  const jobs = Number(values.jobs ?? REPLAY_JOBS_DEFAULT);
  if (!Number.isInteger(jobs) || jobs < 1 || jobs > REPLAY_JOBS_MAX) throw new Error(`--jobs must be an integer 1..${REPLAY_JOBS_MAX}`);
  const gateRoot = path.join(outRoot, GATE_ID);
  const log = new RunLog(abs(values['run-log']), { out: outRoot });
  const startedAt = new Date().toISOString();
  log.write('start', { argv: process.argv.slice(2), node: process.version, mode: 'dev', scenarios: names, blocks, lc0, jobs });
  const locks = [];
  try {
    // fail before any run, not hours into the experiment: the CPU sets of every block and, as soon as one block is pinned, the CPU topology (V3-12)
    for (const b of blocks) slotCpuSets(b.n, cpuFirst, cpuLast);
    if (blocks.some((b) => b.n > 1)) {
      if (os.cpus().length <= cpuLast) throw new Error(`${os.cpus().length} cpus: the slot CPU sets need cores up to ${cpuLast}`);
      const topo = (deps.checkTopology ?? checkCpuTopology)();
      if (topo.length) throw new Error(`CPU topology differs from the slot CPU sets' assumption: ${topo.slice(0, 3).join('; ')}`);
    }
    const python = environmentFiles().python;
    const states = new Map();
    for (const scenario of names) {
      const sc = newScenarioState(scenario, path.join(gateRoot, scenario), { baseDir: rel(baseDirOf(scenario)) });
      fs.mkdirSync(sc.outDir, { recursive: true });
      for (const d of SCENARIO_SUBDIRS) fs.mkdirSync(path.join(sc.outDir, d));
      states.set(scenario, sc);
    }
    const chrome = await (deps.holdLock ?? holdLock)(abs(LOCK), WAIT_MAX_S);
    if (!chrome) { log.close('NOT-READY'); return { exit: 5 }; }
    locks.push(chrome);
    // C-1: a lock holder that ends early stops the experiment (no new run; INCOMPLETE for the analysis)
    let lostWhy = null;
    let curCtx = null;
    const onLost = (why) => { lostWhy ??= why; log.write('lock-lost', { why }); if (curCtx) abortPhaseA(curCtx, why); };
    chrome.onLost?.(onLost);
    const blockInfo = [];
    let infraStop = false;
    for (let k = 0; k < blocks.length; k++) {
      const b = blocks[k];
      const cpuSets = slotCpuSets(b.n, cpuFirst, cpuLast);
      const slotLocks = [];
      for (let s = 0; s < b.n; s++) {
        const sl = await (deps.holdLock ?? holdLock)(abs(`${SLOT_LOCK_PREFIX}${s}.lock`), WAIT_MAX_S);
        if (!sl) { log.close('NOT-READY'); return { exit: 5 }; }
        slotLocks.push(sl);
        locks.push(sl);
        sl.onLost?.(onLost);
      }
      const tag = { block: k + 1, arm: b.arm };
      const items = buildQueue(names.map((scenario) => ({ scenario, lc1: b.count, lc0, prefix: `b${pad2(k + 1)}-${b.arm.toLowerCase()}`, tag })));
      for (const it of items) {
        const sc = states.get(it.scenario);
        sc.runs[it.name] = { name: it.name, lc: it.lc, block: it.block, arm: it.arm, dir: rel(path.join(sc.outDir, 'runs', it.scenario, it.name)) };
        sc.claimPromise = null; sc.claimed = false; sc.infraStop = null;
      }
      const ctx = newPhaseA({ states, items, nSlots: b.n, cpuSets, log, check: deps.check ?? slotCheck, startRun: deps.startRun ?? startRealRun, claimScenario: () => {} });
      const t0 = new Date().toISOString();
      curCtx = ctx;
      if (lostWhy) abortPhaseA(ctx, lostWhy);
      log.write('block-start', { block: k + 1, arm: b.arm, slots: b.n, cpuSets, items: items.map((i) => `${i.scenario}/${i.name}`) });
      await runPhaseA(ctx);
      for (const sl of slotLocks.reverse()) await sl.release();
      blockInfo.push({ block: k + 1, arm: b.arm, slots: b.n, cpuSets, startedAt: t0, endedAt: new Date().toISOString(), runs: items.length, startOrder: ctx.order });
      log.write('block-end', blockInfo.at(-1));
      if ([...states.values()].some((s) => s.infraStop) || ctx.stopNew || lostWhy) infraStop = true;
      if (lostWhy) { settleUnfinishedRuns(states, lostWhy); break; }   // the remaining blocks are not started: the summary says infraStop
    }
    for (const l of locks.splice(0).reverse()) await l.release();
    log.write('phase-a-end', { locksReleased: true });
    let phaseB = null;
    if (!values['no-phase-b']) {
      const t0 = new Date().toISOString();
      await (deps.runPhaseB ?? runPhaseB)({ states: [...states.values()], python, jobs, log, bundlesFor: (sc, rec) => makeBundlesFor(sc, rec, { datumBaseDir: sc.baseDir, log }) });
      phaseB = { startedAt: t0, endedAt: new Date().toISOString(), jobs };
    }
    const { runs, byArm } = summarizeDev([...states.values()], blocks);
    const summary = { schema: 'slam-refact-us025-lc-e2e-v3-dev-summary-v1', gate: { id: GATE_ID, version: GATE_VERSION }, dev: true, startedAt, finishedAt: new Date().toISOString(),
      argv: process.argv.slice(2), host: { cpus: os.cpus().length, node: process.version }, scenarios: names, cpuRange: [cpuFirst, cpuLast], blocks: blockInfo, phaseB, lSlot: L_SLOT, loadMax: LOAD_MAX, slotCpuUtilMax: SLOT_CPU_UTIL_MAX, byArm, runs, infraStop };
    const file = path.join(outRoot, 'dev-summary.json');
    fs.writeFileSync(file, `${JSON.stringify(summary, null, 1)}\n`, { flag: 'wx' });
    log.write('finished', { summary: rel(file), infraStop });
    log.close(infraStop ? 'INFRA-FAILURE' : 'OK');
    console.log(JSON.stringify({ summary: rel(file), byArm, infraStop }, null, 1));
    return { exit: 0, summary };
  } catch (error) {
    if (!log.closed) { log.write('error', { message: String(error?.message ?? error) }); log.close('ERROR'); }
    throw error;
  } finally { for (const l of locks.splice(0).reverse()) await l.release(); }
}

// ---- phase-b: replay + compare the EXISTING bundles of a finished gate scenario directory (read-only input), V3-8 / design test 2 --------------------------
// Two replay trees made from the same bundles are compared file by file. The replay writes TWO wall-clock fields that differ between any two executions, even of the same binary at the same J:
// `srStageMs` (milliseconds per pipeline stage, in every record of trace.jsonl) and `wallSeconds` (meta.json). Nothing reads them (the comparators and judge.mjs never look at them; the
// compare JSON of every run is identical, below). A file whose only difference is those fields is counted as wallClockOnly, every other difference as a real mismatch.
export const WALL_CLOCK_FIELDS = { 'trace.jsonl': 'srStageMs (per record)', 'meta.json': 'wallSeconds' };
export function dropWallClock(name, text) {
  if (name === 'trace.jsonl') return text.split('\n').filter(Boolean).map((l) => { const o = JSON.parse(l); delete o.srStageMs; return JSON.stringify(o); }).join('\n');
  if (name === 'meta.json') { const o = JSON.parse(text); delete o.wallSeconds; return JSON.stringify(o); }
  return text;
}
// compare/*.json: the output path is replaced by the reference path first
export function compareTrees(refDir, outDir) {
  const res = { ref: refDir, out: outDir, wallClockFields: WALL_CLOCK_FIELDS,
    replays: { dirs: 0, files: 0, bytesEqual: 0, wallClockOnly: [], mismatches: [], missingInOut: [], extraInOut: [] },
    compare: { files: 0, equalAfterPathNormalization: 0, mismatches: [], missingInOut: [], extraInOut: [] } };
  const listDirs = (d) => (fs.existsSync(d) ? fs.readdirSync(d, { withFileTypes: true }).filter((e) => e.isDirectory()).map((e) => e.name).sort() : []);
  const refDirs = listDirs(path.join(refDir, 'replays'));
  const outDirs = listDirs(path.join(outDir, 'replays'));
  for (const d of refDirs) {
    if (!outDirs.includes(d)) { res.replays.missingInOut.push(d); continue; }
    res.replays.dirs++;
    const a = listFiles(path.join(refDir, 'replays', d));
    const b = listFiles(path.join(outDir, 'replays', d));
    for (const f of a) {
      if (!b.includes(f)) { res.replays.missingInOut.push(`${d}/${f}`); continue; }
      res.replays.files++;
      const pa = path.join(refDir, 'replays', d, f);
      const pb = path.join(outDir, 'replays', d, f);
      if (sha256File(pa) === sha256File(pb)) { res.replays.bytesEqual++; continue; }
      let sameWithoutClock = false;
      if (f in WALL_CLOCK_FIELDS) { try { sameWithoutClock = dropWallClock(f, fs.readFileSync(pa, 'utf8')) === dropWallClock(f, fs.readFileSync(pb, 'utf8')); } catch { sameWithoutClock = false; } }   // not JSON = a real difference
      if (sameWithoutClock) res.replays.wallClockOnly.push(`${d}/${f}`);
      else res.replays.mismatches.push(`${d}/${f}`);
    }
    for (const f of b) if (!a.includes(f)) res.replays.extraInOut.push(`${d}/${f}`);
  }
  for (const d of outDirs) if (!refDirs.includes(d)) res.replays.extraInOut.push(d);
  const jsons = (d) => (fs.existsSync(d) ? fs.readdirSync(d).filter((n) => n.endsWith('.json')).sort() : []);
  const refJ = jsons(path.join(refDir, 'compare'));
  const outJ = jsons(path.join(outDir, 'compare'));
  for (const n of refJ) {
    if (!outJ.includes(n)) { res.compare.missingInOut.push(n); continue; }
    res.compare.files++;
    const text = fs.readFileSync(path.join(outDir, 'compare', n), 'utf8').split(outDir).join(refDir);
    if (text === fs.readFileSync(path.join(refDir, 'compare', n), 'utf8')) res.compare.equalAfterPathNormalization++; else res.compare.mismatches.push(n);
  }
  for (const n of outJ) if (!refJ.includes(n)) res.compare.extraInOut.push(n);
  res.identical = !res.replays.mismatches.length && !res.replays.missingInOut.length && !res.compare.mismatches.length && !res.compare.missingInOut.length;
  res.strictlyByteIdentical = res.identical && !res.replays.wallClockOnly.length;
  return res;
}

export function cmdCompareTrees(values) {
  if (!values.ref || !values.out) throw new Error('--ref <dir> --out <dir>');
  const res = compareTrees(abs(values.ref), abs(values.out));
  if (values.save) fs.writeFileSync(abs(values.save), `${JSON.stringify(res, null, 1)}\n`, { flag: 'wx' });
  console.log(JSON.stringify({ ref: rel(res.ref), out: rel(res.out), replays: { dirs: res.replays.dirs, files: res.replays.files, bytesEqual: res.replays.bytesEqual, wallClockOnly: res.replays.wallClockOnly.length, mismatches: res.replays.mismatches, missingInOut: res.replays.missingInOut, extraInOut: res.replays.extraInOut },
    compare: res.compare, identical: res.identical, strictlyByteIdentical: res.strictlyByteIdentical }, null, 1));
  return { exit: res.identical ? 0 : 1 };
}

export async function cmdPhaseB(values) {
  const from = abs(values.from ?? '');
  const out = abs(values.out ?? '');
  if (!values.from || !fs.existsSync(path.join(from, 'report.json'))) throw new Error('--from <finished gate scenario directory with report.json>');
  if (!values.out || !(out + path.sep).startsWith(abs(DEV_ROOT) + path.sep)) throw new Error(`--out must be below ${DEV_ROOT}`);
  const jobs = Number(values.jobs ?? REPLAY_JOBS_DEFAULT);
  if (!Number.isInteger(jobs) || jobs < 1 || jobs > REPLAY_JOBS_MAX) throw new Error(`--jobs must be an integer 1..${REPLAY_JOBS_MAX}`);
  const only = values.runs ? new Set(values.runs.split(',')) : null;
  const rep = JSON.parse(fs.readFileSync(path.join(from, 'report.json'), 'utf8'));
  const sc = newScenarioState(rep.scenario, out);
  fs.mkdirSync(out, { recursive: true });
  for (const d of ['replays', 'logs', 'compare']) fs.mkdirSync(path.join(out, d));
  for (const [name, r] of Object.entries(rep.runs)) {
    if (r.infra || (only && !only.has(name))) continue;
    sc.runs[name] = { name, lc: r.lc, dir: r.dir };
  }
  const log = new RunLog(abs(values['run-log'] ?? path.join(path.dirname(out), `${path.basename(out)}.run.log`)), { out });
  log.write('start', { argv: process.argv.slice(2), mode: 'phase-b', from: rel(from), out: rel(out), jobs, runs: Object.keys(sc.runs).length });
  const t0 = Date.now();
  const bundlesFor = (s, rec) => {
    const off = path.join(from, 'bundles', rep.scenario, rec.name);
    const on = path.join(from, 'bundles', rep.scenario, `${rec.name}-on`);
    return rec.lc === 1 ? [{ run: rec.name, arm: 'off', bundle: off }, { run: rec.name, arm: 'on', bundle: on }] : [{ run: rec.name, arm: 'null', bundle: off }];
  };
  await runPhaseB({ states: [sc], python: environmentFiles().python, jobs, log, bundlesFor });
  const seconds = (Date.now() - t0) / 1000;
  const cmpTo = (values['compare-to'] ? [abs(values['compare-to'])] : []);
  const comparisons = {};
  comparisons.v1StoredOutput = compareTrees(from, out);
  for (const c of cmpTo) comparisons[`other:${rel(c)}`] = compareTrees(c, out);
  const replayExits = Object.fromEntries(Object.values(sc.runs).flatMap((r) => Object.entries(r.replays ?? {}).map(([arm, v]) => [`${r.name}-${arm}`, v.exit])));
  const summary = { schema: 'slam-refact-us025-lc-e2e-v3-phaseb-bitexact-v1', scenario: rep.scenario, from: rel(from), out: rel(out), jobs, seconds, runs: Object.keys(sc.runs).length, replayExits,
    replayNonZero: Object.entries(replayExits).filter(([, e]) => e !== 0).map(([k]) => k), comparisons };
  const file = path.join(out, 'phaseb-summary.json');
  fs.writeFileSync(file, `${JSON.stringify(summary, null, 1)}\n`, { flag: 'wx' });
  log.write('finished', { summary: rel(file), identical: comparisons.v1StoredOutput.identical });
  log.close('OK');
  console.log(JSON.stringify({ scenario: rep.scenario, jobs, seconds, runs: summary.runs, replayNonZero: summary.replayNonZero, identicalToV1: comparisons.v1StoredOutput.identical,
    replays: comparisons.v1StoredOutput.replays, compare: comparisons.v1StoredOutput.compare }, null, 1));
  return { exit: summary.replayNonZero.length || !comparisons.v1StoredOutput.identical ? 1 : 0, summary };
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
    const { values } = parseArgs({ args: rest, options: { scenario: { type: 'string' }, scenarios: { type: 'string' }, 'out-root': { type: 'string' }, 'run-log': { type: 'string' }, 'prereg-dir': { type: 'string' },
      'gate-id': { type: 'string' }, 'gate-version': { type: 'string' }, 'designation-env': { type: 'string' }, manifest: { type: 'string' }, reports: { type: 'string' },
      slots: { type: 'string' }, jobs: { type: 'string' }, blocks: { type: 'string' }, lc0: { type: 'string' }, 'no-phase-b': { type: 'boolean', default: false },
      from: { type: 'string' }, out: { type: 'string' }, 'compare-to': { type: 'string' }, runs: { type: 'string' }, 'cpu-first': { type: 'string' }, 'cpu-last': { type: 'string' }, ref: { type: 'string' }, save: { type: 'string' } } });
    if (cmd === 'run') process.exitCode = (await cmdRun(values)).exit;
    else if (cmd === 'run-all') process.exitCode = (await cmdRunAll(values)).exit;
    else if (cmd === 'dev') process.exitCode = (await cmdDev(values)).exit;
    else if (cmd === 'phase-b') process.exitCode = (await cmdPhaseB(values)).exit;
    else if (cmd === 'compare-trees') process.exitCode = cmdCompareTrees(values).exit;
    else if (cmd === 'precheck') process.exitCode = cmdPrecheck(values);
    else if (cmd === 'verdict') process.exitCode = cmdVerdict(values);
    else if (cmd === 'audit') process.exitCode = cmdAudit(values);
    else if (cmd === 'pin') process.exitCode = cmdPin(values);
    else if (cmd === 'print-pins') console.log(JSON.stringify(livePins({ withBinary: fs.existsSync(abs(REPLAY_BIN)) }), null, 1));
    else { console.error('usage: gate_lc.mjs precheck|run|run-all|verdict|audit|pin|print-pins|dev|phase-b ... (see the header)'); process.exitCode = 2; }
  } catch (error) {
    console.error(String(error?.message ?? error));
    process.exitCode = String(error?.message).startsWith('HELD:') ? 3 : 1;
  }
}

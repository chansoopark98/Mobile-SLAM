// node --test slam_refact/tools/gates/us025-lc-e2e-v3/gate_lc.test.mjs
// Synthetic checks of the v3 runner. Part 1 = the v2 tests (the runner's refusals before a marker, the engine pin rules, the per-run infrastructure rule, the watchdog) adapted to
// version 3; part 2 (appended below) = the v3 execution machinery: queue order, slot assignment, marker claim time, the N = 1 path, per-slot readiness, watchdog isolation,
// phase A before phase B, locks, byte identity of the judgement files with v2. No original SDK, no Chromium, no replay, no real git state (git and every runner are injected).
import test from 'node:test';
import assert from 'node:assert/strict';
import fs from 'node:fs';
import os from 'node:os';
import path from 'node:path';
import { spawnSync } from 'node:child_process';
import { COMPARE_FULL, DEV_SCENARIOS, PINNED_FILES, decisionConstants, FREEZE_PATHS, GATE_ID, GATE_VERSION, OUT_ALLOWED, PENDING, REPLAY_BIN, RUN_WALL_MAX_S, START_DISK_PCT_MAX, checkEnginePin, checkHold, cmdRun, configRecord, findHungRenderers, freezeDirty, originalRunInfra, parsePs, root, supervise } from './gate_lc.mjs';
import { JUDGE_CONSTANTS, SCENARIO_RUNS } from './judge.mjs';

const tmp = () => fs.mkdtempSync(path.join(os.tmpdir(), 'us025-lc-'));
const gateOf = (dir) => ({ gateId: GATE_ID, version: GATE_VERSION, preregDir: dir, designationEnv: 'SLAM_REFACT_US025_TEST_DESIGNATION' });
const clean = () => ({ status: 0, out: '', err: '' });
const git = (map) => (...args) => map(args.join(' ')) ?? clean();
const SHA = 'a'.repeat(40);
const BIN = 'b'.repeat(64);

test('freezeDirty: clean, dirty and a git failure (which is not clean)', () => {
  assert.equal(freezeDirty(() => clean()), '');
  assert.match(freezeDirty(() => ({ status: 0, out: ' M slam_refact/engine/x.cc', err: '' })), /engine/);
  assert.match(freezeDirty(() => ({ status: 128, out: '', err: 'not a git repository' })), /git status failed/);
  let seen = '';
  freezeDirty((...a) => { seen = a.join(' '); return clean(); });
  for (const p of FREEZE_PATHS) assert.ok(seen.includes(p), p);
});

test('hold: wrong gate, dev scenario, unknown scenario, steering env, PRECHECK_*, dirty tree, no manifest, all before anything exists', () => {
  const dir = tmp();
  const outDir = path.join(dir, 'out-must-not-exist');
  const h = (over = {}) => checkHold({ gate: gateOf(dir), scenario: 'flip6-s3-lowtex', outDir, env: {}, git, ...over });
  assert.throws(() => h({ gate: { ...gateOf(dir), gateId: 'other-gate' } }), /HELD: args/);
  assert.throws(() => h({ gate: { ...gateOf(dir), version: 1 } }), /HELD: args/);   // v1 and v2 are not this runner
  assert.throws(() => h({ gate: { ...gateOf(dir), version: 2 } }), /HELD: args/);
  for (const s of DEV_SCENARIOS) assert.throws(() => h({ scenario: s }), /HELD: args/);
  assert.throws(() => h({ scenario: 'tum-room3-full' }), /HELD: args/);
  assert.throws(() => h({ env: { BROWSER_EXECUTABLE: '/bin/true' } }), /HELD: env/);
  assert.throws(() => h({ env: { NODE_OPTIONS: '--x' } }), /HELD: env/);
  assert.throws(() => h({ env: { PRECHECK_MAX_DISK_PCT: '97' } }), /HELD: env.*PRECHECK_MAX_DISK_PCT/);
  assert.throws(() => h({ git: () => ({ status: 0, out: '?? slam_refact/tools/gates/x', err: '' }) }), /HELD: dirty/);
  assert.throws(() => h({ git: () => ({ status: 128, out: '', err: 'boom' }) }), /HELD: dirty/);
  assert.throws(() => h({ git: () => clean() }), /HELD: manifest/);
  assert.equal(fs.existsSync(outDir), false);
  assert.deepEqual(fs.readdirSync(dir), []);
});

test('hold: unregistered scenario, config mismatch, PENDING-PIN engine pin', () => {
  const dir = tmp();
  const outDir = path.join(dir, 'out-must-not-exist');
  const mf = path.join(dir, `${GATE_ID}.v${GATE_VERSION}.manifest.json`);
  const h = () => checkHold({ gate: gateOf(dir), scenario: 'flip6-s3-lowtex', outDir, env: {}, git: () => clean() });
  fs.writeFileSync(mf, JSON.stringify({ registered_run: { scenarios: [{ name: 'flip6-s2' }] } }));
  assert.throws(h, /HELD: data/);
  fs.writeFileSync(mf, JSON.stringify({ registered_run: { scenarios: [{ name: 'flip6-s3-lowtex' }] } }));
  assert.throws(h, /HELD: config/);
  fs.writeFileSync(mf, JSON.stringify({ registered_run: { scenarios: [{ name: 'flip6-s3-lowtex' }], config: configRecord() }, engine_pin: { commit: PENDING, replay_binary_sha256: PENDING } }));
  assert.throws(h, /HELD: pins.*PENDING-PIN/);
  assert.equal(fs.existsSync(outDir), false);
  assert.deepEqual(fs.readdirSync(dir), [path.basename(mf)]);
});

test('engine pin: PENDING, malformed, not an ancestor, engine source changed, binary changed, and the passing case', () => {
  const good = { commit: SHA, replay_binary_sha256: BIN };
  const opts = (map, sha = BIN) => ({ git: git(map), binarySha: () => sha });
  assert.throws(() => checkEnginePin(undefined, opts(() => null)), /HELD: pins/);
  assert.throws(() => checkEnginePin({ commit: PENDING, replay_binary_sha256: PENDING }, opts(() => null)), /PENDING-PIN/);
  assert.throws(() => checkEnginePin({ commit: SHA, replay_binary_sha256: PENDING }, opts(() => null)), /PENDING-PIN/);
  assert.throws(() => checkEnginePin({ commit: 'abc123', replay_binary_sha256: BIN }, opts(() => null)), /40-hex/);
  assert.throws(() => checkEnginePin({ commit: SHA, replay_binary_sha256: 'x' }, opts(() => null)), /sha256/);
  assert.throws(() => checkEnginePin(good, opts((c) => (c.startsWith('merge-base') ? { status: 1, out: '', err: '' } : null))), /ancestor/);
  assert.throws(() => checkEnginePin(good, opts((c) => (c.startsWith('diff') ? { status: 0, out: 'slam_refact/engine/src/map_builder/loop_close.cc', err: '' } : null))), /engine sources changed/);
  assert.throws(() => checkEnginePin(good, opts(() => null, 'c'.repeat(64))), /replay binary sha256/);
  assert.throws(() => checkEnginePin(good, { git: git(() => null), binarySha: () => { throw new Error('ENOENT'); } }), /ENOENT/);
  assert.equal(checkEnginePin(good, opts(() => null)), true);
  // the diff the pin rule asks git for excludes the test tree and covers the engine tree
  let diffArgs = '';
  checkEnginePin(good, opts((c) => { if (c.startsWith('diff')) diffArgs = c; return null; }));
  assert.match(diffArgs, /slam_refact\/engine :\(exclude\)slam_refact\/engine\/tests/);
});

test('original run infrastructure rule: a hang or crash of the ORIGINAL SDK is INFRA of that run, never a FAIL', () => {
  const base = { runnerExit: 0, status: 'COMPLETED', hasInputs: true };
  assert.equal(originalRunInfra(base), null);
  assert.equal(originalRunInfra({ ...base, status: 'COMPLETED_WITH_ERRORS' }), null);   // the original crashes after a loop close success; the runner still exits 0
  assert.match(originalRunInfra({ ...base, runnerExit: 1, status: 'FAILED' }), /runner exit 1/);
  assert.match(originalRunInfra({ ...base, status: 'FAILED' }), /status FAILED/);
  assert.match(originalRunInfra({ ...base, status: 'NO-META' }), /status NO-META/);
  assert.match(originalRunInfra({ ...base, hasInputs: false }), /missing/);
  assert.match(originalRunInfra({ ...base, timedOut: true }), /not available/);
  assert.match(originalRunInfra({ ...base, prerunOk: false }), /not available/);
});

test('configuration: dev scenarios are never gate scenarios, outputs stay below the allowed root, the record carries the run counts', () => {
  for (const s of DEV_SCENARIOS) assert.ok(!(s in SCENARIO_RUNS), s);
  assert.deepEqual(configRecord().scenario_runs, SCENARIO_RUNS);
  assert.equal(OUT_ALLOWED, 'build/slam-refact-pkgh-20261022');
  assert.match(REPLAY_BIN, /^build\/slam-refact-pkgh-20261022\/us025\/lc-e2e-v1-pin\/native\/slam_refact_replay_lc6a$/);
  assert.deepEqual(configRecord().replay_env, ['PATH']);
  assert.deepEqual(configRecord().precheck_env, { PRECHECK_MAX_HEADLESS: '0', PRECHECK_MAX_BUILDS: '0' });
});

test('run refuses a wrong out-root and a dev scenario before any output or marker exists', async () => {
  const dir = tmp();
  const log = (n) => path.join(dir, n);
  const gv = { 'prereg-dir': dir, 'gate-id': GATE_ID, 'gate-version': '3', 'designation-env': 'SLAM_REFACT_US025_TEST_DESIGNATION' };
  await assert.rejects(cmdRun({ scenario: 'flip6-s3-lowtex', 'out-root': 'build/elsewhere', 'run-log': log('a.log'), ...gv }), /out-root/);
  await assert.rejects(cmdRun({ scenario: 'flip6-s3-lowtex', 'out-root': `${OUT_ALLOWED}/../x`, 'run-log': log('b.log'), ...gv }), /out-root/);
  await assert.rejects(cmdRun({ scenario: 'flip6-s3-lowtex', ...gv }), /run-log/);
  const outRoot = `${OUT_ALLOWED}/us025/gate-runs/refusal-test-never-created`;
  await assert.rejects(cmdRun({ scenario: 'flip6-n1', 'out-root': outRoot, 'run-log': log('c.log'), ...gv }), /HELD: args/);
  await assert.rejects(cmdRun({ scenario: 'flip6-s3-lowtex', 'out-root': outRoot, 'run-log': log('d.log'), ...gv }), /HELD/);   // dirty tree, PENDING pin or no manifest: some HELD
  assert.equal(fs.existsSync(path.join(root, outRoot)), false);
  assert.deepEqual(fs.readdirSync(dir).filter((f) => f.endsWith('.marker')), []);
});

test('run.sh and gate_lc.mjs agree on the freeze paths and the PRECHECK_* handling', () => {
  const sh = fs.readFileSync(path.join(path.dirname(new URL(import.meta.url).pathname), 'run.sh'), 'utf8');
  for (const p of FREEZE_PATHS) assert.ok(sh.includes(p), `run.sh freeze path ${p}`);
  assert.match(sh, /grep '\^PRECHECK_'/);
  assert.match(sh, /CLEAN\+=\(-u/);
  assert.equal(spawnSync('bash', ['-n', path.join(path.dirname(new URL(import.meta.url).pathname), 'run.sh')]).status, 0);
});

test('watchdog config is part of the registered configuration the hold compares (prereview C1, C2, C4)', () => {
  const c = configRecord();
  assert.equal(c.run_wall_max_s, RUN_WALL_MAX_S);
  assert.equal(c.run_wall_max_s, 600);
  assert.equal(c.watchdog_grace_s, 120);
  assert.equal(c.renderer_cpu_min, 90);
  assert.deepEqual(c.harness_signatures, ['ENOSPC', 'No space left on device']);
  assert.equal(c.replay_retry_on_sigkill, 1);
  assert.equal(c.start_disk_pct_max, START_DISK_PCT_MAX);
  assert.equal(START_DISK_PCT_MAX, 90);
});

test('watchdog: findHungRenderers picks only busy renderers inside the run tree', () => {
  const ps = parsePs([
    ' 100 1 500 0.0 node run_lcgold.mjs --scenario x',
    ' 101 100 500 1.0 /opt/chrome --headless',
    ' 102 101 500 98.5 /opt/chrome --type=renderer --foo',
    ' 103 101 500 12.0 /opt/chrome --type=renderer --bar',
    ' 104 101 500 99.0 /opt/chrome --type=gpu-process',
    ' 200 1 500 0.0 node other-lane.mjs',
    ' 201 200 500 97.0 /opt/chrome --type=renderer --other-lane',
  ].join('\n'));
  assert.equal(ps.get(102).pcpu, 98.5);
  assert.deepEqual(findHungRenderers(ps, 100, 90), [102]);
  assert.deepEqual(findHungRenderers(ps, 100, 10), [102, 103]);
  assert.deepEqual(findHungRenderers(ps, 999, 90), []);
});

test('watchdog: a hung child is killed (renderer path and process-group path), a normal child is left alone; both are recorded', async () => {
  const hang = ['-e', 'setInterval(() => {}, 1000)'];
  const quick = await supervise({ cmd: process.execPath, args: ['-e', 'process.exit(3)'], capS: 5, graceS: 1, ps: () => new Map() });
  assert.equal(quick.status, 3);
  assert.equal(quick.watchdog.fired, false);
  const t0 = Date.now();
  const hung = await supervise({ cmd: process.execPath, args: hang, capS: 0.3, graceS: 0.3, ps: () => new Map() });
  assert.equal(hung.watchdog.fired, true);
  assert.equal(hung.watchdog.groupKilled, true);
  assert.equal(hung.signal, 'SIGKILL');
  assert.ok(Date.now() - t0 < 5000);
  // the renderer path with a REAL decoy: the supervised process starts a busy child whose command line contains --type=renderer
  const parent = "const { spawn } = require('child_process'); spawn(process.execPath, ['-e', 'while (true) {}', '--', '--type=renderer'], { stdio: 'ignore' }); setInterval(() => {}, 1000);";
  const r = await supervise({ cmd: process.execPath, args: ['-e', parent], capS: 2.5, graceS: 0.5, minCpu: 50 });
  assert.equal(r.watchdog.fired, true);
  assert.equal(r.watchdog.killedRenderers.length, 1);
  assert.equal(r.watchdog.groupKilled, true);
  // a run the watchdog touched is INFRA even when the runner then exited 0 with a COMPLETED status
  assert.match(originalRunInfra({ runnerExit: 0, status: 'COMPLETED', hasInputs: true, watchdog: hung.watchdog }), /watchdog/);
  assert.match(originalRunInfra({ runnerExit: 1, status: 'FAILED', hasInputs: false, watchdog: r.watchdog }), /watchdog/);
});

test('run.sh: resume, start disk condition, continue after exit 6 are written down (prereview C3, C4, R3)', () => {
  const sh = fs.readFileSync(path.join(path.dirname(new URL(import.meta.url).pathname), 'run.sh'), 'utf8');
  assert.match(sh, /--resume/);
  assert.match(sh, /START_DISK_PCT_MAX=90/);
  assert.match(sh, /rc" -eq 6/);
});

const gateDir = path.dirname(new URL(import.meta.url).pathname);

test('v3 identity and pins: version 3, the v3 tool paths, the whole-run comparator and judge.mjs are pinned', () => {
  assert.equal(GATE_VERSION, 3);
  assert.equal(GATE_ID, 'us025-lc-e2e-gate');
  const pins = PINNED_FILES();
  for (const f of ['gate_lc.mjs', 'judge.mjs', 'run.sh', 'compare_full_us025.py']) assert.ok(pins.includes(`slam_refact/tools/gates/us025-lc-e2e-v3/${f}`), f);
  assert.ok(!pins.some((f) => f.includes('us025-lc-e2e-v1') || f.includes('us025-lc-e2e-v2')), 'no v1 or v2 gate file is pinned');
  assert.ok(pins.includes('slam_refact/tools/harness/us025/compare_us025.py') && pins.includes('slam_refact/tools/harness/us026/compare_us026.py'));
  assert.equal(COMPARE_FULL, 'slam_refact/tools/gates/us025-lc-e2e-v3/compare_full_us025.py');
  assert.ok(fs.existsSync(path.join(root, COMPARE_FULL)));
  assert.match(configRecord().out_default, /us025-lc-e2e-gate-v3$/);
});

test('v3 values that D-V2-3 / D-V3-LOCK keep from v1 and v2', () => {
  assert.deepEqual(configRecord().scenario_runs, { 'flip6-s3-lowtex': { lc1: 16, lc0: 2 }, 'flip6-s2': { lc1: 8, lc0: 2 }, 'tum-room1-full': { lc1: 4, lc0: 2 }, 'tum-room2-full': { lc1: 4, lc0: 2 }, 'tum-room4-full': { lc1: 4, lc0: 2 } });
  assert.equal(JUDGE_CONSTANTS.K_MIN, 3);
  assert.equal(RUN_WALL_MAX_S, 600);
  assert.equal(configRecord().watchdog_grace_s, 120);
  assert.equal(START_DISK_PCT_MAX, 90);
  assert.equal(JUDGE_CONSTANTS.MAX_THIN_FRACTION, 0.25);
});

test('the registered decision constants carry the whole-run comparator (the hold compares them with the manifest)', () => {
  const c = decisionConstants();
  assert.equal(c.compare_full_us025.MIN_TRACE_FRAMES, 30);
  assert.equal(c.compare_us025.PNP_ULP, 3);
  assert.equal(c.judge.MAX_THIN_FRACTION, 0.25);
});

test('run.sh is the v3 command: v3 designation variable and tool, run-all, the disk and resume rules unchanged', () => {
  const sh = fs.readFileSync(path.join(gateDir, 'run.sh'), 'utf8');
  assert.match(sh, /^GATE_VERSION=3$/m);
  assert.match(sh, /SLAM_REFACT_US025_LC_V3_DESIGNATION_SHA256/);
  assert.match(sh, /TOOL=slam_refact\/tools\/gates\/us025-lc-e2e-v3\/gate_lc\.mjs/);
  assert.doesNotMatch(sh, /LC_V1_DESIGNATION|LC_V2_DESIGNATION|us025-lc-e2e-v[12]\/gate_lc/);
  assert.match(sh, /gate-runs\/us025-lc-e2e-gate-v3/);
  assert.match(sh, /START_DISK_PCT_MAX=90/);
  assert.match(sh, /node "\$TOOL" run-all /);
  assert.match(sh, /--jobs/);
});

test('compare_full_us025.py: its synthetic unit tests pass (15)', () => {
  const r = spawnSync('python3', ['-B', path.join(gateDir, 'test_compare_full_us025.py')], { encoding: 'utf8' });
  assert.equal(r.status, 0, r.stderr);
  assert.match(r.stderr, /Ran 15 tests/);
});

// =====================================================================================================================================================
// part 2: the v3 execution machinery
// =====================================================================================================================================================
import { createHash } from 'node:crypto';
import { pathToFileURL } from 'node:url';
import { CPU_E_FIRST, CPU_P_LAST, DEV_CPU_FIRST, DEV_CPU_LAST, L_SLOT, LOAD_MAX, LOCK, REPLAY_JOBS_DEFAULT, REPLAY_JOBS_MAX, SLOTS, SLOTS_SUPPORTED, SLOT_CPU_FIRST, SLOT_CPU_LAST, SLOT_CPU_UTIL_MAX, SLOT_LOCK_PREFIX, DEV_ROOT,
  abortPhaseA, buildQueue, checkCpuTopology, checkSlotsAndJobs, classifyScenarioRuns, cmdDev, cmdPhaseB, cmdRunAll, compareTrees, cpuSetUtil, holdLock, loadLimit, newPhaseA, newScenarioState, ownHeadless, parseBlocks, parseCpuList,
  parseProcStat, readiness, recheckPins, runPhaseA, runPhaseB, runQueue, settleUnfinishedRuns, runnerCommand, runMetrics, slotCheck, slotCpuSets, summarizeDev } from './gate_lc.mjs';
import { makeWritable } from '../../../web-chunk/tools/gate-common/freeze.mjs';

const v2Dir = path.resolve(gateDir, '../us025-lc-e2e-v2');
const shaOf = (f) => createHash('sha256').update(fs.readFileSync(f)).digest('hex');
const rel2 = (p) => path.relative(root, p);
const sleepMs = (ms) => new Promise((r) => setTimeout(r, ms));

test('D-V3-LOCK: the judgement files are byte-identical to the v2 files (sha256 against the v2 directory)', () => {
  for (const f of ['judge.mjs', 'compare_full_us025.py', 'test_compare_full_us025.py', 'judge.test.mjs', 'make_lc_manifest.mjs']) {
    assert.ok(fs.existsSync(path.join(v2Dir, f)), `v2 ${f}`);
    assert.equal(shaOf(path.join(gateDir, f)), shaOf(path.join(v2Dir, f)), `${f} differs from the v2 file`);
  }
});

test('D-V3-LOCK: decision constants, run counts, watchdog and tolerances are the v2 values; only the execution keys of the configuration differ', async () => {
  const G2 = await import(pathToFileURL(path.join(v2Dir, 'gate_lc.mjs')).href);
  assert.deepEqual(decisionConstants().judge, G2.decisionConstants().judge);
  assert.deepEqual(decisionConstants().compare_us025, G2.decisionConstants().compare_us025);
  assert.deepEqual(decisionConstants().compare_full_us025, G2.decisionConstants().compare_full_us025);
  assert.deepEqual(decisionConstants().verdict_exit, G2.decisionConstants().verdict_exit);
  const a = configRecord();
  const b = G2.configRecord();
  const changed = Object.keys({ ...a, ...b }).filter((k) => JSON.stringify(a[k]) !== JSON.stringify(b[k])).sort();
  assert.deepEqual(changed, ['out_default', 'phases', 'replay_jobs', 'replay_jobs_default', 'replay_jobs_max', 'l_slot', 'slot_cpu_sets', 'slot_cpu_util_max', 'slot_cpu_window_s', 'slot_lock_prefix', 'slots', 'slots_supported'].sort());
  for (const k of ['scenario_runs', 'run_options_lc1', 'run_options_lc0', 'on_params', 'load_max', 'wait_max_s', 'lock', 'precheck_env', 'replay_env', 'run_wall_max_s', 'watchdog_grace_s', 'renderer_cpu_min', 'harness_signatures', 'replay_retry_on_sigkill', 'start_disk_pct_max']) {
    assert.deepEqual(a[k], b[k], k);
  }
  assert.equal(LOAD_MAX, 8);
  assert.equal(SLOTS, 1);
  assert.deepEqual(a.slot_cpu_sets, [null]);
  assert.deepEqual(G2.RUN_OPTIONS_LC1, a.run_options_lc1);
});

test('slot CPU sets (D-V3-8 C-1): N = 1 has none (no taskset); N = 2 / 4 / 8 give every slot the same P/E mix; unsupported N and asymmetric ranges are refused', () => {
  assert.deepEqual(slotCpuSets(1), [null]);
  assert.deepEqual(slotCpuSets(2), ['0-7,16-23', '8-15,24-31']);
  assert.deepEqual(slotCpuSets(4), ['0-3,16-19', '4-7,20-23', '8-11,24-27', '12-15,28-31']);
  assert.deepEqual(slotCpuSets(8), ['0-1,16-17', '2-3,18-19', '4-5,20-21', '6-7,22-23', '8-9,24-25', '10-11,26-27', '12-13,28-29', '14-15,30-31']);
  assert.throws(() => slotCpuSets(3), /supported/);
  assert.throws(() => slotCpuSets(6), /supported/);
  assert.throws(() => slotCpuSets(5), /supported/);
  assert.throws(() => slotCpuSets(0), /supported/);
  assert.throws(() => slotCpuSets(2, 8, 31), /same mix/);        // 8 P threads but 16 E cores: no symmetric split
  assert.throws(() => slotCpuSets(2, 0, 15), /same mix/);        // no E cores at all
  assert.throws(() => slotCpuSets(2, 1, 30), /evenly/);
  for (const n of SLOTS_SUPPORTED.filter((x) => x > 1)) {
    const sets = slotCpuSets(n);
    const cores = sets.flatMap(parseCpuList);
    assert.deepEqual(cores.slice().sort((x, y) => x - y), Array.from({ length: 32 }, (_, i) => i));   // disjoint and complete
    for (const set of sets) {
      const cpus = parseCpuList(set);
      assert.equal(cpus.filter((c) => c <= CPU_P_LAST).length, cpus.filter((c) => c >= CPU_E_FIRST).length, `${set}: as many P threads as E cores`);
      for (const c of cpus.filter((x) => x <= CPU_P_LAST)) assert.ok(cpus.includes(c ^ 1), `${set}: HT sibling of cpu${c} is in the same slot`);   // 2i and 2i+1 together
    }
  }
  assert.deepEqual(parseCpuList('8-10,14'), [8, 9, 10, 14]);
  assert.deepEqual(parseCpuList('0-7,16-23'), [0, 1, 2, 3, 4, 5, 6, 7, 16, 17, 18, 19, 20, 21, 22, 23]);
});

test('C-2: the dev range is all 32 CPUs (P2 = 0-7,16-23 / 8-15,24-31, P4 = four 4+4 sets, D-V3-8); the real run range is the same, so the adopted N runs on the experiment\'s sets', () => {
  assert.deepEqual([DEV_CPU_FIRST, DEV_CPU_LAST], [0, 31]);
  assert.deepEqual([SLOT_CPU_FIRST, SLOT_CPU_LAST], [DEV_CPU_FIRST, DEV_CPU_LAST]);
  assert.deepEqual(slotCpuSets(2, DEV_CPU_FIRST, DEV_CPU_LAST), ['0-7,16-23', '8-15,24-31']);
  assert.deepEqual(slotCpuSets(4, DEV_CPU_FIRST, DEV_CPU_LAST), ['0-3,16-19', '4-7,20-23', '8-11,24-27', '12-15,28-31']);
  for (const n of [2, 4]) assert.deepEqual(slotCpuSets(n), slotCpuSets(n, DEV_CPU_FIRST, DEV_CPU_LAST));
});

test('checkCpuTopology: this host is 0-15 = P threads in HT pairs, 16-31 = E cores (lscpu / sysfs); a fake sysfs with another layout is reported', () => {
  assert.deepEqual(checkCpuTopology(), []);
  const mk = (patch = {}) => {
    const d = tmp();
    const put = (f, text) => { fs.mkdirSync(path.dirname(path.join(d, f)), { recursive: true }); fs.writeFileSync(path.join(d, f), `${text}\n`); };
    put('cpu_core/cpus', patch.core ?? '0-15');
    put('cpu_atom/cpus', patch.atom ?? '16-31');
    for (let c = 0; c < 32; c++) put(`system/cpu/cpu${c}/topology/thread_siblings_list`, patch[`sib${c}`] ?? (c <= 15 ? `${c - (c % 2)}-${c - (c % 2) + 1}` : String(c)));
    return d;
  };
  assert.deepEqual(checkCpuTopology(mk()), []);
  assert.match(checkCpuTopology(mk({ core: '0-7' })).join(), /cpu_core\/cpus is 0-7/);
  assert.match(checkCpuTopology(mk({ atom: '16-23' })).join(), /cpu_atom\/cpus is 16-23/);
  assert.match(checkCpuTopology(mk({ sib3: '3' })).join(), /cpu3 thread_siblings_list is 3, expected 2-3/);
  assert.match(checkCpuTopology(mk({ sib20: '20-21' })).join(), /cpu20 thread_siblings_list is 20-21, expected 20/);
  assert.match(checkCpuTopology(path.join(tmp(), 'nothing')).join(), /is null/);   // no hybrid topology file at all
});

test('readiness: load limit grows by L_SLOT per other running slot; precheck failure and a busy CPU set block; nothing is relaxed', () => {
  assert.equal(loadLimit(0), 8);
  assert.equal(loadLimit(3), 8 + 3 * L_SLOT);
  const ok = { precheckExit: 0, load1: 8, othersActive: 0, cpuUtil: null };
  assert.equal(readiness(ok).ok, true);
  assert.equal(readiness({ ...ok, load1: 8.01 }).ok, false);   // N = 1 path: exactly the v2 rule load <= 8
  assert.equal(readiness({ ...ok, load1: 8 + L_SLOT, othersActive: 1 }).ok, true);
  assert.equal(readiness({ ...ok, load1: 8 + L_SLOT + 0.01, othersActive: 1 }).ok, false);
  assert.equal(readiness({ ...ok, precheckExit: 1 }).ok, false);
  assert.equal(readiness({ ...ok, cpuUtil: SLOT_CPU_UTIL_MAX }).ok, true);
  assert.equal(readiness({ ...ok, cpuUtil: SLOT_CPU_UTIL_MAX + 0.01 }).ok, false);
  assert.equal(SLOT_CPU_UTIL_MAX, 0.25);
});

test('/proc/stat parsing and the CPU-set utilisation', () => {
  const a = parseProcStat('cpu  1 1 1 1 1 1 1 1\ncpu0 100 0 50 800 50 0 0 0\ncpu1 0 0 0 1000 0 0 0 0\ncpu2 10 0 10 980 0 0 0 0\n');
  const b = parseProcStat('cpu  1 1 1 1 1 1 1 1\ncpu0 200 0 100 850 50 0 0 0\ncpu1 0 0 0 1100 0 0 0 0\ncpu2 10 0 10 1080 0 0 0 0\n');
  assert.equal(a.length, 3);
  assert.equal(cpuSetUtil(a, b, [0]), 150 / 200);   // busy 150 of 200 jiffies
  assert.equal(cpuSetUtil(a, b, [1, 2]), 0);
  assert.equal(cpuSetUtil(a, a, [0]), 0);
});

test('own headless Chromium: only browsers below the gate\'s own runner pids are counted, a foreign one is not', () => {
  const ps = parsePs([
    ' 100 1 50 0.0 node run_lcgold.mjs',
    ' 101 100 50 1.0 /opt/google/chrome/chrome --headless --foo',
    ' 102 101 50 98.0 /opt/google/chrome/chrome --type=renderer --headless',
    ' 200 1 50 0.0 node other-lane.mjs',
    ' 201 200 50 1.0 /opt/google/chrome/chrome --headless --foreign',
    ' 300 1 50 0.0 taskset -c 20-31 node run_lcgold.mjs',
    ' 301 300 50 1.0 /opt/google/chrome/chrome --headless --bar',
  ].join('\n'));
  assert.equal(ownHeadless(ps, []), 0);
  assert.equal(ownHeadless(ps, [100]), 1);
  assert.equal(ownHeadless(ps, [100, 300]), 2);
  assert.equal(ownHeadless(ps, [999]), 0);
});

test('slotCheck: N = 1 samples no CPU set and gives the precheck 0 own headless (the v2 rule); N > 1 passes the own count and samples the slot CPU set', async () => {
  const seen = { max: null, cpu: null };
  const deps = (over = {}) => ({ ps: () => parsePs(' 100 1 5 0 node runner\n 101 100 5 1 chrome --headless'), precheck: (m) => { seen.max = m; return { status: 0, pre: {} }; }, loadavg: () => [3, 3, 3],
    cpuUtil: async (c) => { seen.cpu = c; return 0.1; }, ...over });
  const one = await slotCheck({ cpus: null, othersActive: 0, ownRoots: [], deps: deps() });
  assert.equal(seen.max, 0);
  assert.equal(seen.cpu, null);
  assert.equal(one.ok, true);
  assert.equal(one.cpuUtil, null);
  const two = await slotCheck({ cpus: '20-31', othersActive: 1, ownRoots: [100], deps: deps() });
  assert.equal(seen.max, 1);
  assert.equal(seen.cpu, '20-31');
  assert.equal(two.ok, true);
  assert.equal(two.loadLimit, 8 + L_SLOT);
  assert.equal((await slotCheck({ cpus: '20-31', ownRoots: [100], deps: deps({ cpuUtil: async () => 0.5 }) })).ok, false);
  assert.equal((await slotCheck({ cpus: '20-31', ownRoots: [100], deps: deps({ loadavg: () => [30, 1, 1] }) })).ok, false);
  assert.equal((await slotCheck({ ownRoots: [], deps: deps({ precheck: () => ({ status: 1, pre: {} }) }) })).ok, false);
});

test('queue: manifest order of the scenarios, lc=1 labels then lc=0 labels inside a scenario, 36 + 10 original runs', () => {
  const specs = Object.entries(SCENARIO_RUNS).map(([scenario, r]) => ({ scenario, ...r }));
  const q = buildQueue(specs);
  assert.equal(q.length, 46);
  assert.equal(q.filter((i) => i.lc === 1).length, 36);
  assert.equal(q.filter((i) => i.lc === 0).length, 10);
  assert.deepEqual(q.slice(0, 2).map((i) => `${i.scenario}/${i.name}`), ['flip6-s3-lowtex/gate-lc1-01', 'flip6-s3-lowtex/gate-lc1-02']);
  const s3 = q.filter((i) => i.scenario === 'flip6-s3-lowtex');
  assert.deepEqual(s3.map((i) => i.name), [...Array.from({ length: 16 }, (_, k) => `gate-lc1-${String(k + 1).padStart(2, '0')}`), 'gate-lc0-01', 'gate-lc0-02']);
  assert.deepEqual([...new Set(q.map((i) => i.scenario))], Object.keys(SCENARIO_RUNS));   // scenarios are contiguous and in order
  // contiguity: a scenario's runs are adjacent
  let last = '';
  const seen = new Set();
  for (const i of q) { if (i.scenario !== last) { assert.ok(!seen.has(i.scenario), i.scenario); seen.add(i.scenario); last = i.scenario; } }
  assert.equal(buildQueue([{ scenario: 'x', lc1: 1, lc0: 1, prefix: 'b01-p2', tag: { block: 1, arm: 'P2' } }])[0].name, 'b01-p2-lc1-01');
});

test('runQueue: runs are pulled in queue order, never more than N at once, slot numbers are 0..N-1, N = 1 is strictly sequential', async () => {
  for (const n of [1, 2, 4]) {
    const items = Array.from({ length: 11 }, (_, i) => ({ i }));
    const started = [];
    let active = 0;
    let maxActive = 0;
    const slots = new Set();
    await runQueue(items, n, async (item, slot, k) => {
      assert.equal(item.i, k);
      started.push(item.i); slots.add(slot);
      active++; maxActive = Math.max(maxActive, active);
      await sleepMs(3 + (item.i % 3) * 4);
      active--;
    });
    assert.deepEqual(started, items.map((i) => i.i));
    assert.equal(maxActive, n);
    assert.ok([...slots].every((s) => s >= 0 && s < n));
    if (n === 1) assert.deepEqual([...slots], [0]);
  }
});

// ---- phase A with fake runners -------------------------------------------------------------------------------------------------------------
function fakeWorld({ n = 2, specs, okCheck = () => true, runMs = () => 5, waitMaxS = 100, extra = {}, sensorsAfterMs = 0 }) {
  const dir = tmp();
  const events = [];
  const states = new Map();
  for (const s of specs) {
    const outDir = path.join(dir, s.scenario);
    const sc = newScenarioState(s.scenario, outDir, { planned: { lc1: s.lc1, lc0: s.lc0 } });
    states.set(s.scenario, sc);
  }
  const items = buildQueue(specs);
  for (const it of items) {
    const sc = states.get(it.scenario);
    sc.runs[it.name] = { name: it.name, lc: it.lc, dir: rel2(path.join(sc.outDir, 'runs', it.scenario, it.name)) };
  }
  let active = 0;
  let maxActive = 0;
  const cpusSeen = {};
  const ctx = newPhaseA({ states, items, nSlots: n, cpuSets: slotCpuSets(n), waitMaxS, pollS: 1, sensorsWaitS: 5, sensorsPollS: 0.02, sleepFn: async () => { await sleepMs(1); }, now: (() => { let t = 0; return () => (t += 1000); })(),
    check: async (o) => ({ ok: okCheck(o), at: 'x', loadavg: [1, 1, 1], precheckExit: 0, loadLimit: 8, othersActive: o.othersActive, cpuUtil: null }),
    claimScenario: (sc) => { events.push(`claim ${sc.name}`); fs.mkdirSync(sc.outDir, { recursive: true }); for (const d of SCENARIO_SUBDIRS_T) fs.mkdirSync(path.join(sc.outDir, d)); },
    startRun: async (job) => {
      events.push(`start ${job.scenario}/${job.name} slot${job.slot} cpus=${job.cpus}`);
      cpusSeen[job.slot] = job.cpus;
      active++; maxActive = Math.max(maxActive, active);
      const rdir = path.join(root, rel2(path.join(states.get(job.scenario).outDir, 'runs', job.scenario, job.name)));
      fs.mkdirSync(rdir, { recursive: true });
      const sensors = path.join(path.dirname(rdir), 'engine-input-sensors.jsonl.gz');
      if (!fs.existsSync(sensors)) { await sleepMs(sensorsAfterMs); fs.writeFileSync(sensors, 'sensors'); events.push(`sensors ${job.scenario}`); }
      fs.writeFileSync(path.join(rdir, 'meta.json'), JSON.stringify({ status: 'COMPLETED', loadAvgAtStart: [1, 1, 1] }));
      fs.writeFileSync(path.join(rdir, 'recorded-inputs.jsonl.gz'), 'x');
      fs.writeFileSync(path.join(rdir, 'memprobe.jsonl.gz'), 'x');
      await sleepMs(runMs(job));
      active--;
      events.push(`end ${job.scenario}/${job.name}`);
      return { status: 0, signal: null, watchdog: { fired: false, killedRenderers: [], groupKilled: false }, ...extra };
    } });
  return { dir, events, states, items, ctx, get maxActive() { return maxActive; }, cpusSeen };
}
const SCENARIO_SUBDIRS_T = ['runs', 'bundles', 'replays', 'prerun', 'logs', 'compare'];

test('phase A: the marker of a scenario is claimed once, right before its first original run starts, and no run of an unclaimed scenario starts', async () => {
  const w = fakeWorld({ n: 2, specs: [{ scenario: 'sA', lc1: 3, lc0: 1 }, { scenario: 'sB', lc1: 2, lc0: 1 }] });
  await runPhaseA(w.ctx);
  const claims = w.events.filter((e) => e.startsWith('claim'));
  assert.deepEqual(claims, ['claim sA', 'claim sB']);
  for (const s of ['sA', 'sB']) {
    const ci = w.events.indexOf(`claim ${s}`);
    const firstStart = w.events.findIndex((e) => e.startsWith(`start ${s}/`));
    assert.ok(ci >= 0 && ci < firstStart, `${s}: claim before the first start`);
    assert.equal(w.events.slice(0, ci).some((e) => e.startsWith(`start ${s}/`)), false);
  }
  // sB is claimed only when the first sB run is pulled, i.e. after all sA runs were pulled (queue order)
  const lastAStart = Math.max(...w.events.map((e, i) => (e.startsWith('start sA/') ? i : -1)));
  assert.ok(w.events.indexOf('claim sB') > lastAStart - 0, 'sB is claimed after every sA run started');
  assert.ok(w.maxActive <= 2 && w.maxActive >= 1);
  // all 7 runs done with infra null
  for (const sc of w.states.values()) for (const rec of Object.values(sc.runs)) { assert.equal(rec.infra, null, rec.name); assert.equal(rec.attempts, 1); }
  // slot CPU sets were handed to the runs
  assert.deepEqual(Object.values(w.cpusSeen).sort(), ['0-7,16-23', '8-15,24-31']);
});

test('phase A: the first run of a scenario goes alone until the once-per-scenario sensors file is complete (run_lcgold.mjs race), later runs start after it; other scenarios are not held', async () => {
  const w = fakeWorld({ n: 2, specs: [{ scenario: 'sA', lc1: 3, lc0: 0 }], sensorsAfterMs: 120, runMs: () => 300 });
  await runPhaseA(w.ctx);
  const ev = w.events.filter((e) => /^(start|sensors)/.test(e)).map((e) => e.replace(/ slot.*/, ''));
  assert.deepEqual(ev.slice(0, 3), ['start sA/gate-lc1-01', 'sensors sA', 'start sA/gate-lc1-02']);
  for (const rec of Object.values(w.states.get('sA').runs)) assert.equal(rec.infra, null);
});

test('phase A with N = 1: sequential, in queue order, no CPU set (the v2 path)', async () => {
  const w = fakeWorld({ n: 1, specs: [{ scenario: 'sA', lc1: 2, lc0: 1 }, { scenario: 'sB', lc1: 1, lc0: 0 }] });
  await runPhaseA(w.ctx);
  assert.equal(w.maxActive, 1);
  assert.deepEqual(w.events.filter((e) => /^(start|claim)/.test(e)).map((e) => e.replace(/ slot\d+ cpus=.*/, '')),
    ['claim sA', 'start sA/gate-lc1-01', 'start sA/gate-lc1-02', 'start sA/gate-lc0-01', 'claim sB', 'start sB/gate-lc1-01']);
  assert.ok(w.events.every((e) => !e.startsWith('start') || e.endsWith('cpus=null')));
  // each end precedes the next start (strictly sequential)
  const seq = w.events.filter((e) => /^(start|end)/.test(e)).map((e) => e.split(' ')[0]);
  assert.deepEqual(seq, ['start', 'end', 'start', 'end', 'start', 'end', 'start', 'end']);
});

test('phase A: resources never ready before a marker -> no marker, later scenarios never start; after a marker -> that run is INFRA and the rest of the scenario is not started', async () => {
  // never ready from the start: the first scenario cannot be claimed
  const w1 = fakeWorld({ n: 2, specs: [{ scenario: 'sA', lc1: 2, lc0: 0 }, { scenario: 'sB', lc1: 1, lc0: 0 }], okCheck: () => false, waitMaxS: 3 });
  await runPhaseA(w1.ctx);
  assert.equal(w1.events.length, 0);
  assert.equal(w1.ctx.stopNew, true);
  assert.equal(w1.states.get('sA').preMarkerTimeout, true);
  for (const sc of w1.states.values()) for (const rec of Object.values(sc.runs)) assert.match(rec.infra, /not started/);
  // ready for the marker check and the first run, then never ready again: run 2 is INFRA (resources), run 3 not started
  let calls = 0;
  const w2 = fakeWorld({ n: 1, specs: [{ scenario: 'sA', lc1: 3, lc0: 0 }], okCheck: () => ++calls <= 2, waitMaxS: 3 });
  await runPhaseA(w2.ctx);
  const recs = Object.values(w2.states.get('sA').runs);
  assert.equal(recs[0].infra, null);
  assert.match(recs[1].infra, /not available/);
  assert.match(recs[2].infra, /not started/);
  assert.match(w2.states.get('sA').infraStop, /resources or lock not available/);
});

test('phase A: a slot\'s readiness sees the other running slots (load limit grows by L_SLOT each, own-browser registry)', async () => {
  const seenOthers = [];
  const w = fakeWorld({ n: 2, specs: [{ scenario: 'sA', lc1: 4, lc0: 0 }], runMs: () => 20, okCheck: (o) => { seenOthers.push(o.othersActive); return true; } });
  await runPhaseA(w.ctx);
  assert.ok(seenOthers.every((x) => x === 0 || x === 1));
  assert.ok(seenOthers.includes(1), 'a slot saw its neighbour running');
  assert.equal(w.ctx.running.size, 0);
  assert.equal(w.ctx.roots.size, 0);
});

test('original run command: N = 1 is the plain node command (no taskset); a slot CPU set is taskset -c <set> node ...', () => {
  const job = { scenario: 'flip6-s3-lowtex', name: 'gate-lc1-01', lc: 1, runsRoot: 'x/runs' };
  const [c1, a1] = runnerCommand({ ...job, cpus: null });
  assert.equal(c1, process.execPath);
  assert.ok(!a1.includes('taskset'));
  assert.ok(a1.includes('--lcgold') && a1.includes('--poke') && a1.includes(JSON.stringify({ lc: 1 })));
  const [c2, a2] = runnerCommand({ ...job, cpus: '20-31' });
  assert.equal(c2, 'taskset');
  assert.deepEqual(a2.slice(0, 3), ['-c', '20-31', process.execPath]);
  assert.deepEqual(a2.slice(3), a1);   // identical runner arguments, only the prefix differs
  const [, a0] = runnerCommand({ ...job, lc: 0, cpus: null });
  assert.ok(!a0.includes('--lcgold') && a0.includes(JSON.stringify({ lc: 0 })));
});

test('watchdog isolation: a slot\'s watchdog kills only the hung renderer and process group of ITS OWN run, the neighbour slot is untouched', async () => {
  const decoyParent = (ms) => `const { spawn } = require('child_process'); const d = spawn(process.execPath, ['-e', 'while (true) {}', '--', '--type=renderer'], { stdio: 'ignore' }); let killedEarly = false; d.on('exit', (c, s) => { if (s === 'SIGKILL') killedEarly = true; });
    setTimeout(() => { process.exitCode = killedEarly ? 7 : 0; d.kill('SIGKILL'); setTimeout(() => process.exit(process.exitCode), 50); }, ${ms}); `;
  const [a, b] = await Promise.all([
    supervise({ cmd: process.execPath, args: ['-e', decoyParent(20000)], capS: 2.5, graceS: 0.5, minCpu: 50 }),   // slot A: hangs, the watchdog fires
    supervise({ cmd: process.execPath, args: ['-e', decoyParent(4500)], capS: 60, graceS: 1, minCpu: 50 }),   // slot B: busy renderer decoy too, but within its cap
  ]);
  assert.equal(a.watchdog.fired, true);
  assert.equal(a.watchdog.killedRenderers.length, 1);
  assert.equal(a.watchdog.groupKilled, true);
  assert.equal(b.watchdog.fired, false);
  assert.deepEqual(b.watchdog.killedRenderers, []);
  assert.equal(b.status, 0, 'the neighbour run was not touched (exit 7 = its renderer was killed early)');
});

test('supervise reports the runner pid through onSpawn (the own-browser registry of the slots)', async () => {
  let pid = null;
  const r = await supervise({ cmd: process.execPath, args: ['-e', 'process.exit(0)'], capS: 5, graceS: 1, ps: () => new Map(), onSpawn: (p) => { pid = p; } });
  assert.equal(r.status, 0);
  assert.ok(Number.isInteger(pid) && pid > 0);
});

test('locks: a held lock blocks a second holder until it is released; release lets the next one in; a missing wait gives null', async () => {
  const dir = tmp();
  const f = path.join(dir, 'x.lock');
  const l1 = await holdLock(f, 5);
  assert.ok(l1);
  const t0 = Date.now();
  assert.equal(await holdLock(f, 1), null);
  assert.ok(Date.now() - t0 >= 900);
  await l1.release();
  const l2 = await holdLock(f, 5);
  assert.ok(l2);
  await l2.release();
  // the lock dies with its holder's stdin: a child that is killed releases it
  const l3 = await holdLock(f, 5);
  process.kill(l3.pid, 'SIGKILL');
  await sleepMs(200);
  const l4 = await holdLock(f, 2);
  assert.ok(l4, 'a killed lock holder releases the lock');
  await l4.release();
});

test('registered execution values: SLOTS is 1 (v2 path), jobs default 6 / max 8, slot lock names, the real run refuses another --slots with HELD', () => {
  assert.equal(SLOTS, 1);
  assert.equal(REPLAY_JOBS_DEFAULT, 6);
  assert.equal(REPLAY_JOBS_MAX, 8);
  assert.equal(SLOT_LOCK_PREFIX, 'build/us025-lc-v3-slot-');
  assert.equal(LOCK, 'build/c1-chromium.lock');
  assert.deepEqual(checkSlotsAndJobs({}), { nSlots: 1, jobs: 6 });
  assert.deepEqual(checkSlotsAndJobs({ jobs: '8' }), { nSlots: 1, jobs: 8 });
  assert.throws(() => checkSlotsAndJobs({ slots: '2' }), /HELD: args/);
  assert.throws(() => checkSlotsAndJobs({ jobs: '9' }), /--jobs/);
  assert.throws(() => checkSlotsAndJobs({ jobs: '0' }), /--jobs/);
  assert.throws(() => checkSlotsAndJobs({ jobs: '1.5' }), /--jobs/);
});

test('run-all: the Chromium and slot locks are released before phase B starts, phase B starts after every original run ended, and no marker exists before the first run', async () => {
  const dir = tmp();
  const events = [];
  const held = [];
  const outRoot = `${OUT_ALLOWED}/us025/gate-runs/unit-test-never-created-${process.pid}`;
  const fakeHold = { log: [], python: 'python3', datum: { base_dir: 'b' }, rq: {}, manifest: { engine_pin: {}, review: {} } };
  let active = 0;
  const deps = {
    checkHold: ({ scenario }) => { events.push(`hold ${scenario}`); return fakeHold; },
    holdLock: async (f) => { events.push(`lock ${path.basename(f)}`); held.push(f); return { file: f, release: async () => { events.push(`unlock ${path.basename(f)}`); held.splice(held.indexOf(f), 1); } }; },
    check: async () => ({ ok: true, at: 'x', loadavg: [1, 1, 1], precheckExit: 0, loadLimit: 8, othersActive: 0, cpuUtil: null }),
    claim: (rq) => { events.push('claim'); return { marker: path.join(dir, 'm.marker'), designationSha256: 'x' }; },
    startRun: async (job) => {
      events.push(`run ${job.scenario}/${job.name}`); active++;
      assert.equal(events.includes('phaseB'), false, 'no phase B while an original run is active');
      const rdir = path.join(root, job.runsRoot, job.scenario, job.name);
      fs.mkdirSync(rdir, { recursive: true });
      fs.writeFileSync(path.join(rdir, 'meta.json'), JSON.stringify({ status: 'COMPLETED' }));
      for (const f of ['recorded-inputs.jsonl.gz', 'memprobe.jsonl.gz']) fs.writeFileSync(path.join(rdir, f), 'x');
      await sleepMs(5); active--;
      return { status: 0, signal: null, watchdog: { fired: false, killedRenderers: [], groupKilled: false } };
    },
    recheck: () => { events.push('recheck'); assert.deepEqual(held, [], 'the pins are re-checked after the locks were released and before phase B'); return []; },
    runPhaseB: async () => { events.push('phaseB'); assert.equal(active, 0); assert.deepEqual(held, [], 'every lock was released before phase B'); throw new Error('stop-after-phase-b'); },
  };
  const values = { scenarios: 'flip6-s3-lowtex,flip6-s2', 'out-root': outRoot, 'run-log': path.join(dir, 'run.log'), 'prereg-dir': dir, 'gate-id': GATE_ID, 'gate-version': '3', 'designation-env': 'SLAM_REFACT_US025_TEST_DESIGNATION' };
  try {
    const outcome = await cmdRunAll(values, deps);   // C-3: an unexpected phase B error is recorded (exit 1), it is not thrown past the reports of the started scenarios
    assert.equal(outcome.exit, 1);
    assert.match(outcome.errors.join('\n'), /phase B: stop-after-phase-b/);
    const i = (e) => events.indexOf(e);
    assert.ok(i('recheck') > events.lastIndexOf(events.filter((e) => e.startsWith('run ')).at(-1)) && i('recheck') < i('phaseB'));
    assert.deepEqual(events.slice(0, 2), ['hold flip6-s3-lowtex', 'hold flip6-s2']);   // every hold before anything else
    assert.equal(events[2], 'lock c1-chromium.lock');
    assert.equal(events[3], 'lock us025-lc-v3-slot-0.lock');
    assert.ok(i('claim') > i('lock us025-lc-v3-slot-0.lock'));
    const runs = events.filter((e) => e.startsWith('run '));
    assert.equal(runs.length, 18 + 10);   // s3-lowtex 16+2, s2 8+2
    assert.ok(i('phaseB') > events.lastIndexOf(runs.at(-1)));
    assert.ok(i('unlock us025-lc-v3-slot-0.lock') < i('phaseB') && i('unlock c1-chromium.lock') < i('phaseB'));
    assert.equal(events.filter((e) => e === 'claim').length, 2);   // a marker per scenario
    assert.equal(events.indexOf('claim') < i(runs[0]), true);
  } finally {
    fs.rmSync(path.join(root, outRoot), { recursive: true, force: true });
  }
});

// ---- phase B / dev -----------------------------------------------------------------------------------------------------------------------------
test('dev and phase-b refuse an output directory outside the dev root, and dev takes only known scenarios', async () => {
  const dir = tmp();
  await assert.rejects(cmdDev({ scenarios: 'flip6-n1', blocks: 'S:1', 'out-root': 'build/elsewhere', 'run-log': path.join(dir, 'a.log') }), /out-root/);
  await assert.rejects(cmdDev({ scenarios: 'flip6-n1', blocks: 'S:1', 'out-root': `${OUT_ALLOWED}/us025/gate-runs/x`, 'run-log': path.join(dir, 'b.log') }), /out-root/);
  await assert.rejects(cmdDev({ scenarios: 'no-such', blocks: 'S:1', 'out-root': `${DEV_ROOT}/x`, 'run-log': path.join(dir, 'c.log') }), /scenarios/);
  await assert.rejects(cmdDev({ scenarios: 'flip6-n1', blocks: 'S:1', 'out-root': `${DEV_ROOT}/x` }), /run-log/);
  await assert.rejects(cmdDev({ scenarios: 'flip6-n1', blocks: 'P1:2', 'out-root': `${DEV_ROOT}/x`, 'run-log': path.join(dir, 'd.log') }), /blocks/);
  await assert.rejects(cmdPhaseB({ from: dir, out: `${DEV_ROOT}/x` }), /--from/);
  assert.equal(fs.existsSync(path.join(root, DEV_ROOT, 'x')), false);
});

test('dev blocks: arms S / P<k>, block order is kept, run names carry the block and the arm', () => {
  assert.deepEqual(parseBlocks('S:6,P2:6,P4:6,P4:6,P2:6,S:6').map((b) => `${b.arm}/${b.n}/${b.count}`), ['S/1/6', 'P2/2/6', 'P4/4/6', 'P4/4/6', 'P2/2/6', 'S/1/6']);
  assert.throws(() => parseBlocks('X:3'), /blocks/);
  assert.throws(() => parseBlocks('P5:3'), /not supported/);
  assert.throws(() => parseBlocks(''), /required/);
});

test('dev with fake runners: blocks run one after the other with the arm\'s slot count; phase B is called after the last block; summary has per-arm figures', async () => {
  const dir = tmp();
  const events = [];
  const outRoot = `${DEV_ROOT}/unit-test-${process.pid}`;
  let active = 0;
  let maxActiveBlock = 0;
  const deps = {
    holdLock: async (f) => { events.push(`lock ${path.basename(f)}`); return { release: async () => { events.push(`unlock ${path.basename(f)}`); } }; },
    check: async () => ({ ok: true, at: 'x', loadavg: [1, 1, 1], precheckExit: 0, loadLimit: 8, othersActive: 0, cpuUtil: null }),
    startRun: async (job) => {
      events.push(`run ${job.name} cpus=${job.cpus}`); active++; maxActiveBlock = Math.max(maxActiveBlock, active);
      const rdir = path.join(root, job.runsRoot, job.scenario, job.name);
      fs.mkdirSync(rdir, { recursive: true });
      fs.writeFileSync(path.join(rdir, 'meta.json'), JSON.stringify({ status: 'COMPLETED', wallSeconds: 3, summary: { inputFrames: 10, freshCallbacks: 9, statusCounts: { NO_FRESH_CALLBACK: 1 } }, loadAvgAtStart: [2, 2, 2] }));
      for (const f of ['recorded-inputs.jsonl.gz', 'memprobe.jsonl.gz']) fs.writeFileSync(path.join(rdir, f), 'x');
      await sleepMs(8); active--;
      return { status: 0, signal: null, watchdog: { fired: false, killedRenderers: [], groupKilled: false } };
    },
    runPhaseB: async ({ states }) => { events.push('phaseB'); for (const sc of states) for (const rec of Object.values(sc.runs)) { rec.originalRun = { success: rec.name.includes('lc1-01') }; rec.compare = { on: { verdict: 'PASS' }, off: { verdict: 'PASS' }, full: null }; } },
  };
  try {
    const { exit, summary } = await cmdDev({ scenarios: 'flip6-n1', blocks: 'S:2,P2:2', 'out-root': outRoot, 'run-log': path.join(dir, 'dev.log') }, deps);
    assert.equal(exit, 0);
    assert.deepEqual(summary.blocks.map((b) => [b.arm, b.slots, b.runs]), [['S', 1, 2], ['P2', 2, 2]]);
    const runEvents = events.filter((e) => e.startsWith('run '));
    assert.deepEqual(runEvents.map((e) => e.split(' ')[1]), ['b01-s-lc1-01', 'b01-s-lc1-02', 'b02-p2-lc1-01', 'b02-p2-lc1-02']);
    assert.ok(runEvents.slice(0, 2).every((e) => e.endsWith('cpus=null')));
    assert.ok(runEvents.slice(2).every((e) => /cpus=(0-7,16-23|8-15,24-31)$/.test(e)), 'C-2: the dev default is 0-31, P2 = 0-7,16-23 / 8-15,24-31 (D-V3-8)');
    assert.deepEqual(summary.cpuRange, [0, 31]);
    assert.equal(events.at(-1), 'phaseB');
    assert.ok(events.indexOf('unlock c1-chromium.lock') < events.indexOf('phaseB'));
    assert.equal(summary.byArm.S['flip6-n1'].lc1Runs, 2);
    assert.equal(summary.byArm.P2['flip6-n1'].lcSuccessRuns, 1);   // b02-p2-lc1-01 only (its name carries lc1-01)
    assert.equal(summary.byArm.S['flip6-n1'].noFreshCallback.sum, 2);
    assert.ok(fs.existsSync(path.join(root, outRoot, 'dev-summary.json')));
    assert.deepEqual(fs.readdirSync(dir).filter((f) => f.endsWith('.marker')), []);   // dev creates no marker anywhere
  } finally {
    makeWritable(path.join(root, outRoot));
    fs.rmSync(path.join(root, outRoot), { recursive: true, force: true });
  }
});

test('dev refuses a pinned arm when the CPU topology is not the assumed one BEFORE any run (also not the S block in front of it)', async () => {
  const dir = tmp();
  const outRoot = `${DEV_ROOT}/unit-test-topo-${process.pid}`;
  const started = [];
  const deps = {
    holdLock: async () => ({ release: async () => {} }),
    checkTopology: () => ['cpu_core/cpus is 0-7, expected 0-15'],
    check: async () => ({ ok: true, at: 'x', loadavg: [1, 1, 1], precheckExit: 0, loadLimit: 8, othersActive: 0, cpuUtil: null }),
    startRun: async (job) => { started.push(job.name); throw new Error('must not start'); },
    runPhaseB: async () => {},
  };
  try {
    await assert.rejects(() => cmdDev({ scenarios: 'flip6-n1', blocks: 'S:1,P2:1', 'out-root': outRoot, 'run-log': path.join(dir, 'dev.log') }, deps), /CPU topology differs.*cpu_core\/cpus is 0-7/);
    assert.deepEqual(started, []);
  } finally {
    makeWritable(path.join(root, outRoot));
    fs.rmSync(path.join(root, outRoot), { recursive: true, force: true });
  }
});

test('compareTrees: equal trees are identical, a changed replay byte or compare field is reported, the output path is normalised in compare JSON', () => {
  const a = tmp();
  const b = tmp();
  for (const [d, base] of [[a, a], [b, b]]) {
    fs.mkdirSync(path.join(d, 'replays/r1'), { recursive: true });
    fs.mkdirSync(path.join(d, 'compare'));
    fs.writeFileSync(path.join(d, 'replays/r1/trace.jsonl'), 'same\n');
    fs.writeFileSync(path.join(d, 'compare/r1-on.json'), JSON.stringify({ engine: `${base}/replays/r1`, v: 1 }));
  }
  assert.equal(compareTrees(a, b).identical, true);
  fs.writeFileSync(path.join(b, 'replays/r1/trace.jsonl'), 'diff\n');
  const bad = compareTrees(a, b);
  assert.equal(bad.identical, false);
  assert.deepEqual(bad.replays.mismatches, ['r1/trace.jsonl']);
  fs.writeFileSync(path.join(b, 'replays/r1/trace.jsonl'), 'same\n');
  fs.writeFileSync(path.join(b, 'compare/r1-on.json'), JSON.stringify({ engine: `${b}/replays/r1`, v: 2 }));
  assert.deepEqual(compareTrees(a, b).compare.mismatches, ['r1-on.json']);
  fs.rmSync(path.join(b, 'replays/r1'), { recursive: true });
  assert.deepEqual(compareTrees(a, b).replays.missingInOut, ['r1']);
});

test('runMetrics reads frames, fresh callbacks and NO_FRESH_CALLBACK from meta.json; summarizeDev groups by arm and scenario', () => {
  const dir = tmp();
  fs.writeFileSync(path.join(dir, 'meta.json'), JSON.stringify({ status: 'COMPLETED', wallSeconds: 12, summary: { inputFrames: 381, freshCallbacks: 379, statusCounts: { NO_FRESH_CALLBACK: 2 }, publishToCallbackMs: { p50: 50, p95: 90 } }, loadAvgAtStart: [1.5, 1, 1] }));
  const m = runMetrics(dir);
  assert.deepEqual([m.inputFrames, m.freshCallbacks, m.noFreshCallback, m.publishToCallbackMsP50], [381, 379, 2, 50]);
  assert.equal(runMetrics(path.join(dir, 'nope')), null);
});

test('run-all writes the report of every started scenario (v2 form + execution record), SHA256SUMS, read-only; all runs INFRA -> exit 6 and the INFRA class; nothing outside the out-root', async () => {
  const dir = tmp();
  for (const f of [`${GATE_ID}.v3.md`, `${GATE_ID}.v3.manifest.json`, 'd.json', 'r.md', 'm.marker']) fs.writeFileSync(path.join(dir, f), `${f}\n`);
  const outRoot = `${OUT_ALLOWED}/us025/gate-runs/unit-test-report-${process.pid}`;
  const fakeHold = { log: ['fake hold'], python: 'python3', datum: { name: 'x', base_dir: 'b' }, rq: {}, manifest: { engine_pin: { commit: 'c' }, review: { designation_file: 'd.json', file: 'r.md' } } };
  const deps = {
    checkHold: () => fakeHold,
    holdLock: async (f) => ({ file: f, release: async () => {} }),
    check: async () => ({ ok: true, at: 'x', loadavg: [1, 1, 1], precheckExit: 0, loadLimit: 8, othersActive: 0, cpuUtil: null }),
    claim: () => ({ marker: path.join(dir, 'm.marker'), designationSha256: 'x' }),
    recheck: () => [],
    startRun: async () => ({ status: 1, signal: null, watchdog: { fired: false, killedRenderers: [], groupKilled: false } }),   // every original run fails: INFRA of that run, no replay is needed
  };
  const values = { scenarios: 'flip6-s2', 'out-root': outRoot, 'run-log': path.join(dir, 'run.log'), 'prereg-dir': dir, 'gate-id': GATE_ID, 'gate-version': '3', 'designation-env': 'SLAM_REFACT_US025_TEST_DESIGNATION' };
  try {
    const r = await cmdRunAll(values, deps);
    assert.equal(r.exit, 0);   // runs INFRA one by one are NOT an infrastructure STOP (only unavailable resources are); the verdict rules decide
    const rep = JSON.parse(fs.readFileSync(path.join(root, outRoot, GATE_ID, 'flip6-s2', 'report.json'), 'utf8'));
    assert.equal(rep.schema, 'slam-refact-us025-lc-e2e-report-v3');
    assert.deepEqual([rep.gate.id, rep.gate.version, rep.gate.key, rep.kind, rep.scenario], [GATE_ID, 3, 'lc', 'gate', 'flip6-s2']);
    assert.equal(Object.keys(rep.runs).length, 10);
    assert.ok(Object.values(rep.runs).every((x) => /runner exit 1/.test(x.infra)));
    assert.deepEqual([rep.execution.mode, rep.execution.slots, rep.execution.replayJobs], ['v3-two-phase', 1, 6]);
    assert.equal(rep.execution.startOrder.length, 10);
    assert.deepEqual(rep.execution.cpuSets, [null]);
    assert.ok(rep.registeredCopies.length === 5);
    assert.ok(fs.existsSync(path.join(root, outRoot, GATE_ID, 'flip6-s2', 'SHA256SUMS')));
    assert.equal(fs.statSync(path.join(root, outRoot, GATE_ID, 'flip6-s2', 'report.json')).mode & 0o222, 0);
    assert.equal(rep.classified.lc1.length, 8);
    assert.ok(rep.classified.lc1.every((c) => c.class === 'INFRA'));
    assert.deepEqual(fs.readdirSync(dir).filter((f) => f.endsWith('.log') === false && !['d.json', 'r.md', 'm.marker'].includes(f) && !f.startsWith(GATE_ID)), []);
  } finally {
    makeWritable(path.join(root, outRoot));
    fs.chmodSync(path.join(dir, `${GATE_ID}.v3.md`), 0o644);
    fs.rmSync(path.join(root, outRoot), { recursive: true, force: true });
  }
});

test('compareTrees: the wall-clock fields (trace srStageMs, meta wallSeconds) are reported as wallClockOnly, a real field difference is a mismatch, the strict flag drops', () => {
  const a = tmp();
  const b = tmp();
  const put = (d, trace, meta) => { fs.mkdirSync(path.join(d, 'replays/r1'), { recursive: true }); fs.writeFileSync(path.join(d, 'replays/r1/trace.jsonl'), trace); fs.writeFileSync(path.join(d, 'replays/r1/meta.json'), JSON.stringify(meta)); fs.writeFileSync(path.join(d, 'replays/r1/other.jsonl'), 'x\n'); };
  const rec = (i, ms, p) => JSON.stringify({ i, p, srStageMs: { frontend: ms } });
  put(a, `${rec(0, 1.5, 3)}\n${rec(1, 2.5, 4)}\n`, { status: 'ok', wallSeconds: 59.3 });
  put(b, `${rec(0, 1.7, 3)}\n${rec(1, 2.4, 4)}\n`, { status: 'ok', wallSeconds: 59.2 });
  let r = compareTrees(a, b);
  assert.deepEqual(r.replays.wallClockOnly.sort(), ['r1/meta.json', 'r1/trace.jsonl']);
  assert.deepEqual(r.replays.mismatches, []);
  assert.equal(r.identical, true);
  assert.equal(r.strictlyByteIdentical, false);
  put(b, `${rec(0, 1.7, 3)}\n${rec(1, 2.4, 5)}\n`, { status: 'ok', wallSeconds: 59.2 });   // p differs: a real difference
  fs.rmSync(path.join(b, 'replays/r1'), { recursive: true });
  put(b, `${rec(0, 1.7, 3)}\n${rec(1, 2.4, 5)}\n`, { status: 'ok', wallSeconds: 59.2 });
  r = compareTrees(a, b);
  assert.deepEqual(r.replays.mismatches, ['r1/trace.jsonl']);
  assert.equal(r.identical, false);
  fs.rmSync(path.join(b, 'replays/r1'), { recursive: true });
  put(b, `${rec(0, 1.5, 3)}\n${rec(1, 2.5, 4)}\n`, { status: 'ok', wallSeconds: 59.3 });
  r = compareTrees(a, b);
  assert.equal(r.strictlyByteIdentical, true);
});

// =====================================================================================================================================================
// part 3: review-us025-lc-e2e-v3-runner.md C-1, C-2 (above), C-3, R-1, R-2 (D-V3-7)
// =====================================================================================================================================================
const withTimeout = (promise, ms, what) => Promise.race([promise, new Promise((_, reject) => setTimeout(() => reject(new Error(`timeout: ${what}`)), ms))]);

test('C-1: release() of a lock whose flock child is already dead returns at once; the loss is reported (lost, onLost); a normal release is not a loss', async () => {
  const dir = tmp();
  const f = path.join(dir, 'c1.lock');
  // the reproduction of the review: kill the flock child, then release()
  const l1 = await holdLock(f, 5);
  const lostCalls = [];
  l1.onLost((why) => lostCalls.push(why));
  process.kill(l1.pid, 'SIGKILL');
  await sleepMs(300);
  assert.equal(l1.lost, true);
  assert.equal(lostCalls.length, 1);
  assert.match(lostCalls[0], /ended before the lock was released/);
  await withTimeout(l1.release(), 3000, 'release of a dead lock holder');   // before the fix this never returned
  await withTimeout(l1.release(), 3000, 'second release');
  // a callback registered after the loss is called at once
  const late = [];
  l1.onLost((why) => late.push(why));
  assert.equal(late.length, 1);
  // the lock can be taken again (the killed holder released it)
  const l2 = await holdLock(f, 3);
  assert.ok(l2);
  // killed WHILE release() waits for the exit: still returns
  const pending = l2.release();
  await withTimeout(pending, 3000, 'normal release');
  assert.equal(l2.lost, false);
  assert.equal(l2.released, true);
  // release() right after the child died but before any other event: the exit was already seen
  const l3 = await holdLock(f, 3);
  process.kill(l3.pid, 'SIGKILL');
  await sleepMs(100);
  await withTimeout(l3.release(), 3000, 'release after the death was observed');
});

test('R-1: a worker that throws stops new work, the other slots finish their run, and only then the first error is thrown', async () => {
  const started = [];
  const finished = [];
  let thrownAt = 0;
  const t0 = Date.now();
  await assert.rejects(runQueue(Array.from({ length: 8 }, (_, i) => ({ i })), 2, async (item, slot) => {
    started.push(item.i);
    if (item.i === 1) { thrownAt = Date.now() - t0; throw new Error('worker-boom'); }
    await sleepMs(item.i === 0 ? 150 : 5);   // the neighbour slot is busy for 150 ms
    finished.push(item.i);
  }), /worker-boom/);
  assert.ok(finished.includes(0), 'the neighbour slot finished its run before the pool rejected');
  assert.ok(Date.now() - t0 >= 140 && thrownAt < 100, 'the rejection waited for the neighbour');
  assert.deepEqual(started, [0, 1], 'no new item was pulled after the failure');
});

test('R-1 / C-1 in run-all: a claim that throws (phase A error) or a lost lock stop the queue, the claimed scenario still gets its report (INFRA), the unclaimed one gets no marker', async () => {
  const dir = tmp();
  for (const f of [`${GATE_ID}.v3.md`, `${GATE_ID}.v3.manifest.json`, 'd.json', 'r.md', 'm.marker']) fs.writeFileSync(path.join(dir, f), `${f}\n`);
  const outRoot = `${OUT_ALLOWED}/us025/gate-runs/unit-test-r1-${process.pid}`;
  const fakeHold = { log: [], python: 'python3', datum: { name: 'x', base_dir: 'b' }, rq: {}, manifest: { engine_pin: { commit: 'c' }, review: { designation_file: 'd.json', file: 'r.md' } } };
  const base = (extra) => ({
    checkHold: () => fakeHold, check: async () => ({ ok: true, at: 'x', loadavg: [1, 1, 1], precheckExit: 0, loadLimit: 8, othersActive: 0, cpuUtil: null }),
    recheck: () => [], ...extra,
  });
  const failedRun = async () => ({ status: 1, signal: null, watchdog: { fired: false, killedRenderers: [], groupKilled: false } });
  const values = (tag) => ({ scenarios: 'flip6-s3-lowtex,flip6-s2', 'out-root': `${outRoot}-${tag}`, 'run-log': path.join(dir, `${tag}.log`), 'prereg-dir': dir, 'gate-id': GATE_ID, 'gate-version': '3', 'designation-env': 'SLAM_REFACT_US025_TEST_DESIGNATION' });
  try {
    // (1) the claim of the second scenario throws: worker error
    let claims = 0;
    const r1 = await cmdRunAll(values('a'), base({
      holdLock: async (f) => ({ file: f, release: async () => {} }),
      claim: () => { if (++claims === 2) throw new Error('HELD: first-run: marker taken'); return { marker: path.join(dir, 'm.marker'), designationSha256: 'x' }; },
      startRun: failedRun,
    }));
    assert.equal(r1.exit, 1);
    assert.match(r1.errors[0], /phase A: HELD: first-run/);
    assert.deepEqual(r1.scenarios, ['flip6-s3-lowtex'], 'the scenario with a consumed marker got its report');
    assert.ok(fs.existsSync(path.join(root, `${outRoot}-a`, GATE_ID, 'flip6-s3-lowtex', 'report.json')));
    assert.equal(fs.existsSync(path.join(root, `${outRoot}-a`, GATE_ID, 'flip6-s2', 'report.json')), false);
    // (2) the lock holder dies during phase A: no new run starts, the report says INFRA; the unclaimed scenario has no marker
    let lostCb = null;
    const started = [];
    const r2 = await cmdRunAll(values('b'), base({
      holdLock: async (f) => ({ file: f, release: async () => {}, onLost: (cb) => { if (path.basename(f) === 'c1-chromium.lock') lostCb = cb; } }),
      claim: () => ({ marker: path.join(dir, 'm.marker'), designationSha256: 'x' }),
      startRun: async (job) => { started.push(job.name); if (started.length === 2) lostCb('lock holder died'); return failedRun(); },
    }));
    assert.equal(started.length, 2, 'no run starts after the lock was lost');
    assert.equal(r2.exit, 5, 'a scenario without a marker makes the exit 5 (it takes precedence over 6, as in v2); the claimed scenario still has its INFRA report');
    assert.deepEqual(r2.scenarios, ['flip6-s3-lowtex']);
    const rep = JSON.parse(fs.readFileSync(path.join(root, `${outRoot}-b`, GATE_ID, 'flip6-s3-lowtex', 'report.json'), 'utf8'));
    assert.match(rep.infraFailure, /phase A aborted: lock holder died/);
    assert.equal(rep.result.class, 'INFRA');
    assert.equal(rep.execution.phaseA.aborted, 'lock holder died');
    assert.ok(Object.values(rep.runs).every((x) => x.infra), 'every run is INFRA or not started');
    assert.equal(fs.existsSync(path.join(root, `${outRoot}-b`, GATE_ID, 'flip6-s2')), false, 'the unclaimed scenario has no directory, marker or report');
  } finally {
    for (const t of ['a', 'b']) makeWritable(path.join(root, `${outRoot}-${t}`));
    try { fs.chmodSync(path.join(dir, `${GATE_ID}.v3.md`), 0o644); } catch { /* not made read-only */ }
    for (const t of ['a', 'b']) fs.rmSync(path.join(root, `${outRoot}-${t}`), { recursive: true, force: true });
  }
});

test('abortPhaseA / settleUnfinishedRuns: claimed scenarios get infraStop, unclaimed ones are pre-marker; runs without an end record become INFRA', () => {
  const sa = newScenarioState('sA', '/x', { planned: { lc1: 1, lc0: 0 } });
  sa.claimed = true; sa.claimPromise = Promise.resolve(true);
  sa.runs.r1 = { name: 'r1', lc: 1, attempts: 1 }; sa.runs.r2 = { name: 'r2', lc: 1 };
  const sb = newScenarioState('sB', '/y', { planned: { lc1: 1, lc0: 0 } });
  sb.runs.r1 = { name: 'r1', lc: 1 };
  const states = new Map([['sA', sa], ['sB', sb]]);
  const ctx = newPhaseA({ states, items: [], nSlots: 1, cpuSets: [null], claimScenario: () => {} });
  abortPhaseA(ctx, 'why');
  abortPhaseA(ctx, 'second reason is ignored');
  assert.equal(ctx.aborted, 'why');
  assert.match(sa.infraStop, /phase A aborted: why/);
  assert.equal(sb.infraStop, null);
  assert.equal(sb.preMarkerTimeout, true);
  settleUnfinishedRuns(states, 'why');
  assert.equal(sa.runs.r1.infra, undefined);
  assert.match(sa.runs.r2.infra, /not completed: why/);
  assert.match(sb.runs.r1.infra, /not completed/);
});

test('C-3: phase B isolates scenarios - an error in one scenario is recorded as INFRA of that scenario, the other scenario is still classified, and every report hook runs (also when a hook throws)', async () => {
  const dir = tmp();
  const mk = (name, runs) => {
    const sc = newScenarioState(name, path.join(dir, name), { planned: { lc1: runs.length, lc0: 0 } });
    for (const d of ['replays', 'logs', 'compare']) fs.mkdirSync(path.join(sc.outDir, d), { recursive: true });
    for (const r of runs) sc.runs[r] = { name: r, lc: 1, dir: `x/${name}/${r}` };
    return sc;
  };
  const a = mk('sA', ['a1', 'a2']);
  const b = mk('sB', ['b1']);
  const c = mk('sC', ['c1']);
  // sA: the bundle step of a1 throws; a2: its replay log already exists (runChild opens it with 'wx': a thrown error inside the replay job)
  fs.writeFileSync(path.join(a.outDir, 'logs', 'replay-a2-off.log'), 'exists');
  const hooked = [];
  await runPhaseB({
    states: [a, b, c], python: 'python3', jobs: 2, log: null,
    bundlesFor: async (sc, rec) => {
      if (rec.name === 'a1') throw new Error('bundle boom');
      if (rec.name === 'a2') return [{ run: 'a2', arm: 'off', bundle: '/nonexistent' }];
      return [];   // sB, sC: nothing to replay
    },
    onScenarioDone: async (sc) => { hooked.push(sc.name); if (sc.name === 'sB') throw new Error('report boom'); },
  });
  assert.deepEqual(hooked.sort(), ['sA', 'sB', 'sC'], 'every scenario reached its report hook');
  assert.match(a.runs.a1.infra, /phase B error \(bundles\): bundle boom/);
  assert.match(a.runs.a2.infra, /phase B error \(replay off\)/);
  assert.match(a.infraStop, /phase B error/);
  assert.equal(a.phaseBErrors.length, 2);
  assert.equal(a.lc1.length, 2);
  assert.ok(a.lc1.every((x) => x.class === 'INFRA'));
  assert.match(b.infraStop, /phase B error \(report\): report boom/);   // a failing report hook is recorded, not thrown
  assert.equal(c.infraStop, null, 'the untouched scenario is not marked');
  assert.ok(Array.isArray(c.lc1));
});

test('C-3 in run-all: a report that cannot be written for one scenario (exit 1) does not stop the report of the other scenario', async () => {
  const dir = tmp();
  for (const f of [`${GATE_ID}.v3.md`, `${GATE_ID}.v3.manifest.json`, 'd.json', 'r.md', 'mB.marker']) fs.writeFileSync(path.join(dir, f), `${f}\n`);   // the marker of the first scenario is missing: its copyInto throws
  const outRoot = `${OUT_ALLOWED}/us025/gate-runs/unit-test-c3-${process.pid}`;
  const fakeHold = { log: [], python: 'python3', datum: { name: 'x', base_dir: 'b' }, rq: {}, manifest: { engine_pin: { commit: 'c' }, review: { designation_file: 'd.json', file: 'r.md' } } };
  let claims = 0;
  const deps = {
    checkHold: () => fakeHold, holdLock: async (f) => ({ file: f, release: async () => {} }), recheck: () => [],
    check: async () => ({ ok: true, at: 'x', loadavg: [1, 1, 1], precheckExit: 0, loadLimit: 8, othersActive: 0, cpuUtil: null }),
    claim: () => ({ marker: path.join(dir, ++claims === 1 ? 'mA.marker' : 'mB.marker'), designationSha256: 'x' }),
    startRun: async () => ({ status: 1, signal: null, watchdog: { fired: false, killedRenderers: [], groupKilled: false } }),
  };
  try {
    const r = await cmdRunAll({ scenarios: 'flip6-s3-lowtex,flip6-s2', 'out-root': outRoot, 'run-log': path.join(dir, 'c3.log'), 'prereg-dir': dir, 'gate-id': GATE_ID, 'gate-version': '3', 'designation-env': 'SLAM_REFACT_US025_TEST_DESIGNATION' }, deps);
    assert.equal(r.exit, 1);
    assert.match(r.errors.join('\n'), /report flip6-s3-lowtex/);
    assert.deepEqual(r.scenarios, ['flip6-s2']);
    assert.ok(fs.existsSync(path.join(root, outRoot, GATE_ID, 'flip6-s2', 'report.json')));
    assert.equal(fs.existsSync(path.join(root, outRoot, GATE_ID, 'flip6-s3-lowtex', 'report.json')), false);
  } finally {
    makeWritable(path.join(root, outRoot));
    try { fs.chmodSync(path.join(dir, `${GATE_ID}.v3.md`), 0o644); } catch { /* not made read-only */ }
    fs.rmSync(path.join(root, outRoot), { recursive: true, force: true });
  }
});

test('R-2: recheckPins reports a dirty freeze path, a drifted pinned file or tree, and a broken engine pin; a clean state is empty', () => {
  const manifest = { held: { pins: { 'a.mjs': 'sha-a', 'b.py': 'sha-b' } }, pinned: { trees: { engine: 't1' } }, engine_pin: { commit: 'c' } };
  const live = (over = {}) => () => Object.entries({ '/x/a.mjs': 'sha-a', '/y/b.py': 'sha-b', ...over });
  const ok = { git: () => clean(), live: live(), trees: () => ({ engine: 't1' }), engineCheck: () => true };
  assert.deepEqual(recheckPins(manifest, ok), []);
  assert.match(recheckPins(manifest, { ...ok, git: () => ({ status: 0, out: ' M slam_refact/engine/x.cc', err: '' }) }).join(), /freeze paths/);
  assert.match(recheckPins(manifest, { ...ok, git: () => ({ status: 128, out: '', err: 'boom' }) }).join(), /git status failed/);
  assert.match(recheckPins(manifest, { ...ok, live: live({ '/x/a.mjs': 'changed' }) }).join(), /pinned files differ.*a\.mjs/);
  assert.match(recheckPins(manifest, { ...ok, live: () => [['/x/a.mjs', 'sha-a']] }).join(), /b\.py/);   // a recorded pin that is gone
  assert.match(recheckPins(manifest, { ...ok, live: () => [['/x/a.mjs', 'sha-a'], ['/y/b.py', 'sha-b'], ['/z/c.js', 'new']] }).join(), /c\.js/);
  assert.match(recheckPins(manifest, { ...ok, trees: () => ({ engine: 't2' }) }).join(), /tree engine/);
  assert.match(recheckPins(manifest, { ...ok, engineCheck: () => { throw new Error('HELD: pins: replay binary sha256 differs'); } }).join(), /replay binary/);
});

test('R-2 in run-all: a pin drift between phase A and phase B skips phase B (no replay with changed tools) and every started scenario gets an INFRA report (exit 6)', async () => {
  const dir = tmp();
  for (const f of [`${GATE_ID}.v3.md`, `${GATE_ID}.v3.manifest.json`, 'd.json', 'r.md', 'm.marker']) fs.writeFileSync(path.join(dir, f), `${f}\n`);
  const outRoot = `${OUT_ALLOWED}/us025/gate-runs/unit-test-r2-${process.pid}`;
  const fakeHold = { log: [], python: 'python3', datum: { name: 'x', base_dir: 'b' }, rq: {}, manifest: { engine_pin: { commit: 'c' }, review: { designation_file: 'd.json', file: 'r.md' } } };
  const events = [];
  const deps = {
    checkHold: () => fakeHold, holdLock: async (f) => ({ file: f, release: async () => { events.push('unlock'); } }),
    check: async () => ({ ok: true, at: 'x', loadavg: [1, 1, 1], precheckExit: 0, loadLimit: 8, othersActive: 0, cpuUtil: null }),
    claim: () => ({ marker: path.join(dir, 'm.marker'), designationSha256: 'x' }),
    startRun: async (job) => {
      const rdir = path.join(root, job.runsRoot, job.scenario, job.name);
      fs.mkdirSync(rdir, { recursive: true });
      fs.writeFileSync(path.join(rdir, 'meta.json'), JSON.stringify({ status: 'COMPLETED' }));
      for (const f of ['recorded-inputs.jsonl.gz', 'memprobe.jsonl.gz']) fs.writeFileSync(path.join(rdir, f), 'x');
      return { status: 0, signal: null, watchdog: { fired: false, killedRenderers: [], groupKilled: false } };
    },
    recheck: () => { events.push('recheck'); return ['pinned files differ from the registration: judge.mjs']; },
    runPhaseB: async () => { events.push('phaseB'); },
  };
  try {
    const r = await cmdRunAll({ scenarios: 'flip6-s2', 'out-root': outRoot, 'run-log': path.join(dir, 'r2.log'), 'prereg-dir': dir, 'gate-id': GATE_ID, 'gate-version': '3', 'designation-env': 'SLAM_REFACT_US025_TEST_DESIGNATION' }, deps);
    assert.equal(r.exit, 6);
    assert.equal(events.includes('phaseB'), false, 'phase B did not run');
    assert.ok(events.indexOf('unlock') < events.indexOf('recheck'));
    const rep = JSON.parse(fs.readFileSync(path.join(root, outRoot, GATE_ID, 'flip6-s2', 'report.json'), 'utf8'));
    assert.match(rep.infraFailure, /pin\/freeze drift before phase B.*judge\.mjs/);
    assert.equal(rep.result.class, 'INFRA');
    assert.deepEqual(rep.execution.pinRecheck, { ok: false, problems: ['pinned files differ from the registration: judge.mjs'] });
    assert.equal(rep.execution.phaseB.skippedForPinDrift, true);
    assert.ok(Object.values(rep.runs).every((x) => /phase B not run/.test(x.infra)));
  } finally {
    makeWritable(path.join(root, outRoot));
    try { fs.chmodSync(path.join(dir, `${GATE_ID}.v3.md`), 0o644); } catch { /* not made read-only */ }
    fs.rmSync(path.join(root, outRoot, '..', `unit-test-r2-${process.pid}`), { recursive: true, force: true });
  }
});

test('C-1 in dev: a lost lock stops the experiment (infraStop in the summary, later blocks never start) and the release does not hang', async () => {
  const dir = tmp();
  const outRoot = `${DEV_ROOT}/unit-test-lost-${process.pid}`;
  let lostCb = null;
  const started = [];
  const deps = {
    holdLock: async (f) => ({ file: f, release: async () => {}, onLost: (cb) => { if (path.basename(f) === 'c1-chromium.lock') lostCb = cb; } }),
    check: async () => ({ ok: true, at: 'x', loadavg: [1, 1, 1], precheckExit: 0, loadLimit: 8, othersActive: 0, cpuUtil: null }),
    startRun: async (job) => {
      started.push(job.name);
      const rdir = path.join(root, job.runsRoot, job.scenario, job.name);
      fs.mkdirSync(rdir, { recursive: true });
      fs.writeFileSync(path.join(rdir, 'meta.json'), JSON.stringify({ status: 'COMPLETED' }));
      for (const f of ['recorded-inputs.jsonl.gz', 'memprobe.jsonl.gz']) fs.writeFileSync(path.join(rdir, f), 'x');
      if (started.length === 2) lostCb('lock holder died');
      return { status: 0, signal: null, watchdog: { fired: false, killedRenderers: [], groupKilled: false } };
    },
    runPhaseB: async () => {},
  };
  try {
    const { summary } = await cmdDev({ scenarios: 'flip6-n1', blocks: 'S:4,S:4', 'out-root': outRoot, 'run-log': path.join(dir, 'lost.log') }, deps);
    assert.equal(summary.infraStop, true);
    assert.equal(started.length, 2, 'no run after the loss, no second block');
    assert.equal(summary.blocks.length, 1);
  } finally {
    makeWritable(path.join(root, outRoot));
    fs.rmSync(path.join(root, outRoot), { recursive: true, force: true });
  }
});

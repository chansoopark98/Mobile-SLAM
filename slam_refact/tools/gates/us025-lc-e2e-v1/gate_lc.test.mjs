// node --test slam_refact/tools/gates/us025-lc-e2e-v1/gate_lc.test.mjs
// Synthetic checks of the runner's refusals: everything that must happen BEFORE a marker, an output directory or a browser exists, the engine pin rules and the
// per-run infrastructure rule. No original SDK, no Chromium, no replay, no real git state (git is injected).
import test from 'node:test';
import assert from 'node:assert/strict';
import fs from 'node:fs';
import os from 'node:os';
import path from 'node:path';
import { spawnSync } from 'node:child_process';
import { DEV_SCENARIOS, FREEZE_PATHS, GATE_ID, GATE_VERSION, OUT_ALLOWED, PENDING, REPLAY_BIN, RUN_WALL_MAX_S, START_DISK_PCT_MAX, checkEnginePin, checkHold, cmdRun, configRecord, findHungRenderers, freezeDirty, originalRunInfra, parsePs, root, supervise } from './gate_lc.mjs';
import { SCENARIO_RUNS } from './judge.mjs';

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
  const gv = { 'prereg-dir': dir, 'gate-id': GATE_ID, 'gate-version': '1', 'designation-env': 'SLAM_REFACT_US025_TEST_DESIGNATION' };
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

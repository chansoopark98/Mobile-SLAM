// node --test slam_refact/tools/gates/us025-lc-e2e-v4/gate_lc_v4.test.mjs
// Regression tests of the v4 execution-machinery fixes (FIXES.md maps every defect id to the test names below). Every test is named with its defect id (#n = the 15 findings of
// scratchpad/fable-gate-findings.txt, G-nn = opus-architecture-review-gate-docs-2026-10-29.md section 2).
// The tests load the runner from US025_LC_IMPL_DIR (default: this directory). Pointing it at ../us025-lc-e2e-v3 runs the SAME tests against the v3 code: that is how "fails before the fix" was observed
// (FIXES.md, "baseline"). Nothing here starts Chromium, a replay, or touches git state: every runner, lock, claim and check is injected.
import test from 'node:test';
import assert from 'node:assert/strict';
import fs from 'node:fs';
import os from 'node:os';
import path from 'node:path';
import { spawn, spawnSync } from 'node:child_process';
import { pathToFileURL } from 'node:url';

const here = path.dirname(new URL(import.meta.url).pathname);
const implDir = path.resolve(here, process.env.US025_LC_IMPL_DIR ?? '.');
const G = await import(pathToFileURL(path.join(implDir, 'gate_lc.mjs')).href);
const { GATE_ID, GATE_VERSION, OUT_ALLOWED, DEV_ROOT, root, buildQueue, newScenarioState, newPhaseA, runPhaseA, slotCpuSets, cmdRunAll, cmdDev, holdLock, supervise, compareTrees, runPhaseB, summarizeDev } = G;
const { makeWritable } = await import(pathToFileURL(path.resolve(here, '../../../web-chunk/tools/gate-common/freeze.mjs')).href);

const tmp = () => fs.mkdtempSync(path.join(os.tmpdir(), 'us025-lc-v4-'));
const rel = (p) => path.relative(root, p);
const sleepMs = (ms) => new Promise((r) => setTimeout(r, ms));
const SUBDIRS = ['runs', 'bundles', 'replays', 'prerun', 'logs', 'compare'];
const okCheck = async (o = {}) => ({ ok: true, at: 'x', loadavg: [1, 1, 1], precheckExit: 0, loadLimit: 8, othersActive: o.othersActive ?? 0, cpuUtil: null });
const noWd = () => ({ fired: false, killedRenderers: [], groupKilled: false });
const withTimeout = (p, ms, what) => Promise.race([p, new Promise((_, rej) => setTimeout(() => rej(new Error(`TIMEOUT (hang): ${what}`)), ms))]);

// a phase A world with fake runners; `claimHook(sc)` runs inside the claim, `startRun(job, helpers)` can be replaced
function world({ n = 1, specs, check = okCheck, claimHook = null, startRun = null, sensorsAfterMs = 0, runMs = 5, waitMaxS = 100 }) {
  const dir = tmp();
  const events = [];
  const states = new Map();
  for (const s of specs) states.set(s.scenario, newScenarioState(s.scenario, path.join(dir, s.scenario), { planned: { lc1: s.lc1, lc0: s.lc0 } }));
  const items = buildQueue(specs);
  for (const it of items) { const sc = states.get(it.scenario); sc.runs[it.name] = { name: it.name, lc: it.lc, dir: rel(path.join(sc.outDir, 'runs', it.scenario, it.name)) }; }
  const stamps = {};
  const defaultRun = async (job) => {
    events.push(`start ${job.scenario}/${job.name}`);
    stamps[job.name] = { startedAtMs: Date.now() };
    const rdir = path.join(root, job.runsRoot, job.scenario, job.name);
    fs.mkdirSync(rdir, { recursive: true });
    const sensors = path.join(path.dirname(rdir), 'engine-input-sensors.jsonl.gz');
    if (!fs.existsSync(sensors)) { await sleepMs(sensorsAfterMs); fs.writeFileSync(sensors, 'sensors'); stamps.sensorsAtMs = Date.now(); }
    fs.writeFileSync(path.join(rdir, 'meta.json'), JSON.stringify({ status: 'COMPLETED', loadAvgAtStart: [1, 1, 1] }));
    for (const f of ['recorded-inputs.jsonl.gz', 'memprobe.jsonl.gz']) fs.writeFileSync(path.join(rdir, f), 'x');
    await sleepMs(runMs);
    return { status: 0, signal: null, watchdog: noWd() };
  };
  const ctx = newPhaseA({ states, items, nSlots: n, cpuSets: slotCpuSets(n), waitMaxS, pollS: 1, sensorsWaitS: 5, sensorsPollS: 0.02, sleepFn: async () => { await sleepMs(1); },
    check, startRun: startRun ? (job) => startRun(job, { events, defaultRun, stamps }) : defaultRun,
    claimScenario: (sc) => { events.push(`claim ${sc.name}`); fs.mkdirSync(sc.outDir, { recursive: true }); for (const d of SUBDIRS) fs.mkdirSync(path.join(sc.outDir, d)); claimHook?.(sc); } });
  return { dir, events, states, items, ctx, stamps };
}

// ---------------------------------------------------------------------------------------------------------------------------------------------------
test('#1 the sensors owner throws at the exclusive prerun write (2 slots): the waiting slot is released (no hang), the failure is reported, no slot stays registered as running', async () => {
  const w = world({ n: 2, specs: [{ scenario: 'sA', lc1: 3, lc0: 0 }], claimHook: (sc) => fs.writeFileSync(path.join(sc.outDir, 'prerun', 'gate-lc1-01.json'), 'taken') });
  await assert.rejects(() => withTimeout(runPhaseA(w.ctx), 4000, 'runPhaseA with a throwing sensors owner'), /EEXIST/);
  assert.equal(w.ctx.running.size, 0, 'the owner slot is not left in ctx.running');
  assert.equal(w.events.filter((e) => e.startsWith('start')).length, 0, 'no run is started inside an already failed phase A');
});

test('#13 the run record of a slot that waited for the owner\'s sensors file starts AFTER that file is complete (startedAt, and so runnerSeconds and the G9 window)', async () => {
  const w = world({ n: 2, specs: [{ scenario: 'sA', lc1: 2, lc0: 0 }], sensorsAfterMs: 300, runMs: 400 });
  await runPhaseA(w.ctx);
  const a = w.states.get('sA').runs;
  assert.ok(Date.parse(a['gate-lc1-02'].startedAt) >= w.stamps.sensorsAtMs - 2, 'non-owner startedAt is not before the sensors file was written');
  assert.ok(a['gate-lc1-02'].sensorsWaitS >= 0, 'the wait is recorded separately');
});

test('G-16 a slot that waited for the sensors file starts no run when phase A was aborted meanwhile, and re-tests its readiness after the wait', async () => {
  const checks = [];
  const w = world({ n: 2, specs: [{ scenario: 'sA', lc1: 2, lc0: 0 }], sensorsAfterMs: 300, runMs: 50, check: async (o) => { checks.push(o.slot); return okCheck(o); } });
  setTimeout(() => G.abortPhaseA(w.ctx, 'lock lost during the wait'), 100);
  await runPhaseA(w.ctx);
  assert.deepEqual(w.events.filter((e) => e.startsWith('start')), ['start sA/gate-lc1-01'], 'the waiting slot did not start its run');
  assert.match(w.states.get('sA').runs['gate-lc1-02'].infra, /not started: phase A aborted: lock lost/);
  const w2 = world({ n: 2, specs: [{ scenario: 'sA', lc1: 2, lc0: 0 }], sensorsAfterMs: 200, runMs: 50, check: async (o) => { checks.push(`w2-${o.slot}`); return okCheck(o); } });
  await runPhaseA(w2.ctx);
  assert.equal(checks.filter((c) => c === 'w2-1').length, 2, 'slot 1: one check before the wait and one after it (v3: only the stale one)');
});

test('#4 runChild: a failed spawn resolves once with the error and the second (close) event does not close the fd again (EBADF = uncaught exception)', async () => {
  const dir = tmp();
  const runChild = G.runChild;
  assert.equal(typeof runChild, 'function', 'runChild is exported for this test');
  const r = await withTimeout(runChild('/nonexistent/binary-v4-test', [], { logFile: path.join(dir, 'x.log') }), 4000, 'runChild');
  assert.equal(r.exit, null);
  assert.match(r.error, /ENOENT/);
  await sleepMs(100);   // the late close event must not throw: node:test would fail on an uncaught exception
});

test('#9 holdLock: a flock that cannot be spawned gives null (the documented "lock not obtained"), not an uncaught exception or a promise that never settles', async () => {
  const saved = process.env.PATH;
  process.env.PATH = '/nonexistent-path-v4-test';
  try {
    const lock = await withTimeout(holdLock(path.join(tmp(), 'x.lock'), 1), 4000, 'holdLock without flock');
    assert.equal(lock, null);
  } finally { process.env.PATH = saved; }
});

// ---- run-all tests ---------------------------------------------------------------------------------------------------------------------------------
function runAllWorld(tag, over = {}) {
  const dir = tmp();
  for (const f of [`${GATE_ID}.v${GATE_VERSION}.md`, `${GATE_ID}.v${GATE_VERSION}.manifest.json`, 'd.json', 'r.md', 'm.marker']) fs.writeFileSync(path.join(dir, f), `${f}\n`);
  const outRoot = `${OUT_ALLOWED}/us025/gate-runs/unit-test-v4-${tag}-${process.pid}`;
  const fakeHold = { log: [], python: 'python3', datum: { name: 'x', base_dir: 'b' }, rq: {}, manifest: { engine_pin: { commit: 'c' }, review: { designation_file: 'd.json', file: 'r.md' } } };
  const deps = { checkHold: () => fakeHold, holdLock: async (f) => ({ file: f, release: async () => {} }), check: okCheck, recheck: () => [],
    claim: () => ({ marker: path.join(dir, 'm.marker'), designationSha256: 'x' }), startRun: async () => ({ status: 1, signal: null, watchdog: noWd() }), runPhaseB: async () => {}, ...over };
  const values = (scenarios) => ({ scenarios, 'out-root': outRoot, 'run-log': path.join(dir, `${tag}.log`), 'prereg-dir': dir, 'gate-id': GATE_ID, 'gate-version': String(GATE_VERSION), 'designation-env': 'SLAM_REFACT_US025_TEST_DESIGNATION' });
  const reportPath = (s) => path.join(root, outRoot, GATE_ID, s, 'report.json');
  const cleanup = () => {
    makeWritable(path.join(root, outRoot));
    try { fs.chmodSync(path.join(dir, `${GATE_ID}.v${GATE_VERSION}.md`), 0o644); } catch { /* not made read-only */ }
    fs.rmSync(path.join(root, outRoot), { recursive: true, force: true });
  };
  return { dir, outRoot, deps, values, reportPath, cleanup, scDir: (s) => path.join(root, outRoot, GATE_ID, s) };
}

test('#5 a directory that appears after the hold check (EEXIST on a scenario subdirectory) does not leave a consumed marker without a report', async () => {
  const w = runAllWorld('claim-eexist');
  w.deps.claim = () => { fs.mkdirSync(path.join(w.scDir('flip6-s2'), 'runs'), { recursive: true }); return { marker: path.join(w.dir, 'm.marker'), designationSha256: 'x' }; };
  try {
    const r = await cmdRunAll(w.values('flip6-s2'), w.deps);
    assert.deepEqual(r.scenarios, ['flip6-s2']);
    assert.ok(fs.existsSync(w.reportPath('flip6-s2')), 'the scenario has its report');
  } finally { w.cleanup(); }
});

test('#5 whatever fails AFTER the marker was taken (here: mkdir of a scenario subdirectory that is a file), the scenario is claimed: it gets an INFRA report and the exit is not 0', async () => {
  const w = runAllWorld('claim-after');
  w.deps.claim = () => { fs.mkdirSync(w.scDir('flip6-s2'), { recursive: true }); fs.writeFileSync(path.join(w.scDir('flip6-s2'), 'runs'), 'a file where a directory is needed'); return { marker: path.join(w.dir, 'm.marker'), designationSha256: 'x' }; };
  try {
    const r = await cmdRunAll(w.values('flip6-s2'), w.deps);
    assert.notEqual(r.exit, 0);
    assert.ok(fs.existsSync(w.reportPath('flip6-s2')), 'consumed marker => report');
    assert.deepEqual(r.consumed, ['flip6-s2']);
  } finally { w.cleanup(); }
});

test('#6 a claim that fails for the NEXT scenario does not flag a scenario whose planned runs all completed as INFRA (no scenario-level infrastructure failure)', async () => {
  let claims = 0;
  const w = runAllWorld('abort-complete', {});
  w.deps.claim = () => { if (++claims === 2) throw new Error('HELD: first-run: marker taken'); return { marker: path.join(w.dir, 'm.marker'), designationSha256: 'x' }; };
  try {
    const r = await cmdRunAll(w.values('flip6-s3-lowtex,flip6-s2'), w.deps);
    assert.deepEqual(r.scenarios, ['flip6-s3-lowtex']);
    const rep = JSON.parse(fs.readFileSync(w.reportPath('flip6-s3-lowtex'), 'utf8'));
    assert.equal(rep.infraFailure, null, 'all 18 planned runs completed: no "phase A aborted" flag');
    assert.equal(Object.values(rep.runs).filter((x) => x.notRun).length, 0);
  } finally { w.cleanup(); }
});

test('#7 the pins are re-checked right before EVERY marker: a drift that appears during phase A is a HELD (exit 3) before the next marker is taken', async () => {
  let rechecks = 0;
  let claims = 0;
  const w = runAllWorld('claim-recheck', {});
  w.deps.recheck = () => (++rechecks === 1 ? [] : ['pinned files differ from the registration: gate_lc.mjs']);
  w.deps.claim = () => { claims++; return { marker: path.join(w.dir, 'm.marker'), designationSha256: 'x' }; };
  try {
    const r = await cmdRunAll(w.values('flip6-s3-lowtex,flip6-s2'), w.deps);
    assert.equal(claims, 1, 'the second marker was never taken');
    assert.equal(r.exit, 3);
    assert.match(r.errors[0], /HELD: pins: before the marker of flip6-s2.*gate_lc\.mjs/);
    assert.equal(fs.existsSync(w.scDir('flip6-s2')), false);
  } finally { w.cleanup(); }
});

test('#15 a HELD raised at claim time keeps its class: exit 3 (not 1)', async () => {
  const w = runAllWorld('held-class');
  w.deps.claim = () => { throw new Error('HELD: first-run: marker exists'); };
  try {
    const r = await cmdRunAll(w.values('flip6-s2'), w.deps);
    assert.equal(r.exit, 3);
    assert.deepEqual(r.consumed, []);
  } finally { w.cleanup(); }
});

test('#8 a worker failure stops the claim of a slot that is waiting for readiness: no marker is taken inside an already failed phase A', async () => {
  let release = false;
  const w = world({ n: 2, specs: [{ scenario: 'sA', lc1: 1, lc0: 0 }, { scenario: 'sB', lc1: 2, lc0: 0 }],
    claimHook: (sc) => { if (sc.name === 'sA') { fs.writeFileSync(path.join(sc.outDir, 'prerun', 'gate-lc1-01.json'), 'taken'); setTimeout(() => { release = true; }, 60); } },
    check: async (o) => ({ ...(await okCheck(o)), ok: o.slot === 0 || release }) });
  await assert.rejects(() => withTimeout(runPhaseA(w.ctx), 4000, 'runPhaseA'), /EEXIST/);
  assert.equal(w.events.includes('claim sB'), false, 'the waiting slot did not take the marker of sB');
  assert.equal(w.events.filter((e) => e.startsWith('start')).length, 0);
});

test('G-23 the readiness check right before the marker is the check of the first run: no second wait (up to 4 h) after the marker', async () => {
  const seen = [];
  const w = world({ n: 1, specs: [{ scenario: 'sA', lc1: 2, lc0: 0 }], check: async (o) => { seen.push(1); return okCheck(o); } });
  await runPhaseA(w.ctx);
  assert.equal(seen.length, 2, 'run 1 reuses the marker check, run 2 has its own');
  assert.equal(w.states.get('sA').runs['gate-lc1-01'].prerun.ok, true);
});

test('#14 a failed readiness check logs WHICH process blocked (the precheck process list), not only the reason text', async () => {
  const logged = [];
  const w = world({ n: 1, specs: [{ scenario: 'sA', lc1: 1, lc0: 0 }], waitMaxS: 2,
    check: async () => ({ ...(await okCheck()), ok: false, precheck: { blocked_reasons: ['headless Chromium'], processes: { headless_chromium: [{ pid: 4242, comm: 'chrome', args: '--headless' }] } }, ownHeadless: 0 }) });
  w.ctx.log = { write: (ev, data) => logged.push([ev, data]) };
  await runPhaseA(w.ctx);
  const ev = logged.find(([e]) => e === 'resource-wait');
  assert.ok(ev, 'a resource-wait event exists');
  assert.equal(ev[1].processes.headless_chromium[0].pid, 4242);
});

test('#10 dev: a worker error in a later block does not lose the summary of the runs that completed (dev-summary.json is written, infraStop true, the error is listed)', async () => {
  const dir = tmp();
  const outRoot = `${DEV_ROOT}/unit-test-v4-dev-${process.pid}`;
  let checks = 0;
  const deps = {
    holdLock: async () => ({ release: async () => {} }),
    check: async () => { if (++checks >= 2) throw new Error('boom in block 2'); return okCheck(); },
    startRun: async (job) => {
      const rdir = path.join(root, job.runsRoot, job.scenario, job.name);
      fs.mkdirSync(rdir, { recursive: true });
      fs.writeFileSync(path.join(rdir, 'meta.json'), JSON.stringify({ status: 'COMPLETED', wallSeconds: 3, summary: { inputFrames: 10 }, loadAvgAtStart: [1, 1, 1] }));
      for (const f of ['recorded-inputs.jsonl.gz', 'memprobe.jsonl.gz']) fs.writeFileSync(path.join(rdir, f), 'x');
      return { status: 0, signal: null, watchdog: noWd() };
    },
    runPhaseB: async () => {},
  };
  try {
    const { summary } = await cmdDev({ scenarios: 'flip6-n1', blocks: 'S:1,S:1', 'out-root': outRoot, 'run-log': path.join(dir, 'dev.log') }, deps);
    assert.equal(summary.infraStop, true);
    assert.match(summary.errors[0], /block 2: boom in block 2/);
    assert.ok(fs.existsSync(path.join(root, outRoot, 'dev-summary.json')));
    assert.equal(summary.runs.filter((r) => r.infra === null).length, 1, 'the run of block 1 is in the summary');
  } finally { makeWritable(path.join(root, outRoot)); fs.rmSync(path.join(root, outRoot), { recursive: true, force: true }); }
});

// ---- #3 ---------------------------------------------------------------------------------------------------------------------------------------------
test('#3 phase B works scenario by scenario: the report hook of the first scenario runs before the bundles of the second are made', async () => {
  const dir = tmp();
  const mk = (name) => { const sc = newScenarioState(name, path.join(dir, name), { planned: { lc1: 1, lc0: 0 } }); for (const d of ['replays', 'logs', 'compare']) fs.mkdirSync(path.join(sc.outDir, d), { recursive: true }); sc.runs.r1 = { name: 'r1', lc: 1, dir: `x/${name}/r1` }; return sc; };
  const events = [];
  await runPhaseB({ states: [mk('sA'), mk('sB')], python: 'python3', jobs: 2, log: null, originalInfo: async () => null,
    bundlesFor: async (sc) => { events.push(`bundles ${sc.name}`); return []; }, onScenarioDone: async (sc) => { events.push(`report ${sc.name}`); } });
  assert.deepEqual(events, ['bundles sA', 'report sA', 'bundles sB', 'report sB']);
});

test('#3 phase A leaves a fsynced journal line per original run (phase-a.jsonl), also for a run that did not run', async () => {
  const w = world({ n: 1, specs: [{ scenario: 'sA', lc1: 2, lc0: 1 }] });
  await runPhaseA(w.ctx);
  const lines = fs.readFileSync(path.join(w.states.get('sA').outDir, 'phase-a.jsonl'), 'utf8').trim().split('\n').map((l) => JSON.parse(l));
  assert.deepEqual(lines.map((l) => l.run), ['gate-lc1-01', 'gate-lc1-02', 'gate-lc0-01']);
  assert.ok(lines.every((l) => l.runnerExit === 0 && l.status === 'COMPLETED'));
});

test('#3 a failure after phase A (here: a lock that cannot be released) still gives every started scenario its INFRA report instead of a consumed marker without any report', async () => {
  let lockNo = 0;
  const w = runAllWorld('emergency', {});
  w.deps.holdLock = async (f) => { const k = ++lockNo; return { file: f, release: async () => { if (k === 1) throw new Error('release failed'); } }; };
  try {
    await assert.rejects(() => cmdRunAll(w.values('flip6-s2'), w.deps), /release failed/);
    assert.ok(fs.existsSync(w.reportPath('flip6-s2')), 'the report exists');
    assert.match(JSON.parse(fs.readFileSync(w.reportPath('flip6-s2'), 'utf8')).infraFailure, /run-all failed before the report/);
  } finally { w.cleanup(); }
});

// ---- #2 / G-18 -----------------------------------------------------------------------------------------------------------------------------------------
test('G-18 / #2 the original run\'s success comes from the original run data when the ON comparator did not run (ON replay exit != 0), and the dev summary exports the replay exit codes', async () => {
  const dir = tmp();
  const sc = newScenarioState('sA', path.join(dir, 'sA'), { planned: { lc1: 1, lc0: 0 } });
  for (const d of ['replays', 'logs', 'compare']) fs.mkdirSync(path.join(sc.outDir, d), { recursive: true });
  sc.runs.r1 = { name: 'r1', lc: 1, dir: 'x/sA/r1', arm: 'P2', block: 1, replays: { on: { exit: 2, seconds: 1 }, off: { exit: 0, seconds: 1 } } };
  // the replay already ran: skip bundles / replays and go to the comparator step through runPhaseB with no replay jobs
  sc.runs.r1.replays = undefined;
  await runPhaseB({ states: [sc], python: 'python3', jobs: 1, log: null, bundlesFor: async () => [], originalInfo: async () => ({ success: true, attempts: 3 }) });
  assert.deepEqual(sc.runs.r1.originalRun, { success: true, attempts: 3 });
  sc.runs.r1.replays = { on: { exit: 2, seconds: 1 }, off: { exit: 0, seconds: 1 } };
  const run = summarizeDev([sc], [{ arm: 'P2', n: 2, count: 1 }]).runs[0];
  assert.deepEqual(run.replayExits, { on: 2, off: 0 });
  assert.equal(run.originalSuccess, true);
});

// ---- #11 / G-26 -----------------------------------------------------------------------------------------------------------------------------------------
test('#11 the watchdog\'s second stage kills the DESCENDANTS of the runner (also a child in its own process group, as playwright starts Chromium), not only the runner\'s group', async () => {
  const dir = tmp();
  const pidFile = path.join(dir, 'child.pid');
  const script = `const {spawn}=require('node:child_process');const c=spawn('sleep',['60'],{detached:true,stdio:'ignore'});require('node:fs').writeFileSync(${JSON.stringify(pidFile)},String(c.pid));setInterval(()=>{},1000);`;
  const r = await supervise({ cmd: process.execPath, args: ['-e', script], capS: 1, graceS: 1, options: { stdio: 'ignore' } });
  assert.equal(r.watchdog.groupKilled, true);
  const pid = Number(fs.readFileSync(pidFile, 'utf8'));
  let alive = true;
  for (let i = 0; i < 40 && alive; i++) { try { process.kill(pid, 0); await sleepMs(50); } catch { alive = false; } }
  if (alive) { try { process.kill(pid, 'SIGKILL'); } catch { /* gone */ } }
  assert.equal(alive, false, 'the detached grandchild did not survive the watchdog');
});

test('#11 a renderer that wedged late (low lifetime pcpu, high CPU in the last second) is found by the recent-CPU sample and killed in the first stage', async () => {
  const fake = spawn('sleep', ['60'], { stdio: 'ignore' });   // a real process standing in for the renderer: the watchdog really signals it
  let runnerPid = 0;
  try {
    const run = supervise({ cmd: 'sleep', args: ['3'], capS: 0.2, graceS: 5, options: { stdio: 'ignore' }, onSpawn: (pid) => { runnerPid = pid; },
      ps: () => new Map([[runnerPid, { ppid: 1, etimes: 10, pcpu: 0, args: 'sleep 3' }], [fake.pid, { ppid: runnerPid, etimes: 10, pcpu: 12, args: 'chrome --type=renderer' }]]),
      recentCpu: async (pids) => new Map(pids.map((p) => [p, 99])) });
    await withTimeout(new Promise((resolve) => fake.on('exit', resolve)), 4000, 'renderer kill');
    assert.ok(fake.signalCode === 'SIGKILL');
    const r = await run;
    assert.deepEqual(r.watchdog.killedRenderers, [fake.pid]);
  } finally { try { fake.kill('SIGKILL'); } catch { /* gone */ } }
});

// ---- G-22 -----------------------------------------------------------------------------------------------------------------------------------------------
test('G-22 compareTrees with a run filter (phase-b --runs) compares only those runs: the runs that were not replayed are not "missing"', () => {
  const a = tmp();
  const b = tmp();
  for (const [d, runs] of [[a, ['r1', 'r2']], [b, ['r1']]]) {
    fs.mkdirSync(path.join(d, 'compare'));
    for (const r of runs) { fs.mkdirSync(path.join(d, 'replays', `${r}-on`), { recursive: true }); fs.writeFileSync(path.join(d, 'replays', `${r}-on`, 'trace.jsonl'), 'same\n'); fs.writeFileSync(path.join(d, 'compare', `${r}-on.json`), JSON.stringify({ v: 1 })); }
  }
  assert.equal(compareTrees(a, b).identical, false, 'without a filter r2 is missing');
  const res = compareTrees(a, b, { runs: ['r1'] });
  assert.equal(res.identical, true);
  assert.deepEqual(res.replays.missingInOut, []);
});

// ---- bundles, pins, run.sh ----------------------------------------------------------------------------------------------------------------------------
test('bundles: the pinned set names the v4 make_bundles.sh and make_variant_v2.py (not the shared make_variant.py), the original-run tool, and every pinned file exists', () => {
  const pins = G.PINNED_FILES();
  assert.ok(pins.includes('slam_refact/tools/gates/us025-lc-e2e-v4/make_bundles.sh'));
  assert.ok(pins.includes('slam_refact/tools/harness/divergence/make_variant_v2.py'));
  assert.ok(pins.includes('slam_refact/tools/gates/us025-lc-e2e-v4/original_run_info.py'));
  assert.ok(!pins.includes('slam_refact/tools/harness/divergence/make_variant.py'));
  assert.ok(!pins.includes('slam_refact/tools/harness/us026/make_bundles.sh'));
  for (const f of pins) assert.ok(fs.existsSync(path.join(root, f)), `${f} exists`);
});

test('bundles: make_bundles.sh of v4 is the us026 script with ONE change, the call of make_variant_v2.py (same flags, same arguments)', () => {
  const v4 = fs.readFileSync(path.join(implDir, 'make_bundles.sh'), 'utf8').split('\n').filter((l) => !l.startsWith('# us025-lc-e2e-gate.v4') && !l.startsWith('# monotonic input times'));
  const old = fs.readFileSync(path.join(root, 'slam_refact/tools/harness/us026/make_bundles.sh'), 'utf8').split('\n');
  const diff = v4.map((l, i) => [l, old[i]]).filter(([x, y]) => x !== y);
  assert.equal(v4.length, old.length);
  assert.equal(diff.length, 1);
  assert.equal(diff[0][0].replace('make_variant_v2.py', 'make_variant.py'), diff[0][1]);
  assert.match(diff[0][0], /--capture-binding --capture-wallclock --queue-times --capture-id-seed|--base "\$base" --out "\$exact" --capture "\$run"/);
  assert.equal(spawnSync('bash', ['-n', path.join(here, 'make_bundles.sh')]).status, 0);
});

test('G-21 run.sh: an exit 5 of run-all (a scenario without marker) no longer hides the scenarios whose marker was consumed: the verdict step still runs over their reports and the exit stays 5', () => {
  const sh = fs.readFileSync(path.join(implDir, 'run.sh'), 'utf8');
  assert.match(sh, /elif \[ "\$rc" -eq 5 \]; then[\s\S]*?ANY5=1/, 'rc 5 is remembered instead of exiting');
  assert.match(sh, /\[ "\$ANY5" = 1 \][\s\S]*exit 5/, 'the final exit is 5 when a scenario had no marker');
  assert.match(sh, /\[ ! -f "\$rep" \]/, 'only reports that exist enter the verdict');
});

// ---- G-26 (measure_run.mjs) -------------------------------------------------------------------------------------------------------------------------
test('G-26 the measurement tool\'s staged watchdog: a renderer BELOW 90 % CPU is not killed at the cap, but the whole tree and the group are killed after the grace (the Chromium lock cannot be held for ever)', () => {
  const kills = [];
  let t = 0;
  const table = new Map([[100, { ppid: 1, etimes: 0, pcpu: 0, args: 'node runner' }], [101, { ppid: 100, etimes: 0, pcpu: 20, args: 'chrome --type=renderer' }], [102, { ppid: 101, etimes: 0, pcpu: 0, args: 'chrome --type=gpu-process' }]]);
  const wd = G.makeWatchdog({ rootPid: 100, capS: 10, graceS: 5, ps: () => table, kill: (pid, sig) => kills.push([pid, sig]), now: () => t * 1000 });
  t = 5; wd.tick();
  assert.deepEqual(kills, []);
  t = 11; wd.tick();
  assert.deepEqual(kills, [], 'the renderer is at 20 %, below the 90 % rule: nothing at the cap');
  t = 14; wd.tick();
  assert.deepEqual(kills, []);
  t = 16; wd.tick();
  assert.equal(wd.state.stage, 2);
  assert.deepEqual(kills.map(([pid]) => pid).sort((a, b) => a - b), [-100, 101, 102], 'the descendants and the process group');
  t = 30; wd.tick();
  assert.equal(kills.length, 3, 'the tree is killed once');
});

test('G-26 / #9 measure_run.mjs uses the staged watchdog of the gate and handles a flock that cannot be spawned', () => {
  const src = fs.readFileSync(path.join(implDir, 'measure_run.mjs'), 'utf8');
  assert.match(src, /makeWatchdog\(\{ rootPid: child\.pid, capS: RUN_WALL_MAX_S, graceS: WATCHDOG_GRACE_S \}\)/);
  assert.match(src, /lockProc\.once\('error'/);
  assert.equal(spawnSync(process.execPath, ['--check', path.join(here, 'measure_run.mjs')]).status, 0);
});

// ---- coordinator decisions M-1, M-2 and the non-blocking review items N-4, N-5 -------------------------------------------------------------------------
const { aggregate, classifyScenario } = await import(pathToFileURL(path.join(implDir, 'judge.mjs')).href);

test('M-1 end to end: make_bundles exit 21 (non-monotonic input times) makes the SCENARIO class INFRA (infraFailure), no replay is made, the gate verdict is not PASS and not FAIL', async () => {
  const w = runAllWorld('m1-exit21');
  const fake = path.join(w.dir, 'make_bundles_exit21.sh');
  fs.writeFileSync(fake, '#!/usr/bin/env bash\nexit 21\n');
  w.deps.startRun = async (job) => {
    const rdir = path.join(root, job.runsRoot, job.scenario, job.name);
    fs.mkdirSync(rdir, { recursive: true });
    fs.writeFileSync(path.join(rdir, 'meta.json'), JSON.stringify({ status: 'COMPLETED', loadAvgAtStart: [1, 1, 1] }));
    for (const f of ['recorded-inputs.jsonl.gz', 'memprobe.jsonl.gz']) fs.writeFileSync(path.join(rdir, f), 'x');
    return { status: 0, signal: null, watchdog: noWd() };
  };
  w.deps.runPhaseB = (ctx) => runPhaseB({ ...ctx, originalInfo: async () => null, bundlesFor: (sc, rec) => G.makeBundlesFor(sc, rec, { datumBaseDir: 'b', log: null, makeBundles: fake }) });
  try {
    const r = await cmdRunAll(w.values('flip6-s2'), w.deps);
    const rep = JSON.parse(fs.readFileSync(w.reportPath('flip6-s2'), 'utf8'));
    assert.equal(rep.result.class, 'INFRA');
    assert.match(rep.infraFailure, /not monotonic in time \(make_variant_v2\.py exit 21\)/);
    assert.ok(Object.values(rep.runs).every((x) => /bundle construction failed \(exit 21/.test(x.infra) && !x.replays), 'every run is INFRA, nothing was replayed');
    assert.ok(rep.classified.lc1.every((c) => c.class === 'INFRA'), 'no run is an engine FAIL');
    assert.equal(r.exit, 6);
    const verdict = aggregate({ registered_run: { scenarios: [{ name: 'flip6-s2' }] } }, [rep]);
    assert.equal(verdict.verdict, 'INFRA');
    assert.notEqual(verdict.verdict, 'PASS');
  } finally { w.cleanup(); }
});

test('M-2 a lock lost during a run: the run in flight ends as INFRA even when it is the last run of its scenario (it is not evaluated normally)', async () => {
  let w;
  w = world({ n: 1, specs: [{ scenario: 'sA', lc1: 1, lc0: 0 }], startRun: async (job, h) => { const done = h.defaultRun(job); G.abortPhaseA(w.ctx, 'lock holder died', { lockLost: true }); return done; } });
  await runPhaseA(w.ctx);
  const rec = w.states.get('sA').runs['gate-lc1-01'];
  assert.match(rec.infra, /lock lost during this run \(lock holder died\)/);
  G.classifyScenarioRuns(w.states.get('sA'));
  assert.equal(w.states.get('sA').lc1[0].class, 'INFRA');
  // scenario level (coordinator decision): the scenario of the in-flight run is infrastructure-failed, the gate cannot PASS
  G.settleUnfinishedRuns(w.states, 'lock holder died');
  G.finalizeAbort(w.ctx);
  const sA = w.states.get('sA');
  assert.match(sA.infraStop, /phase A aborted: lock holder died/);
  const result = classifyScenario({ scenario: 'sA', lc1: sA.lc1, lc0: sA.lc0, planned: { lc1: 1, lc0: 0 }, infraFailure: sA.infraStop });
  assert.equal(result.class, 'INFRA');
  assert.equal(aggregate({ registered_run: { scenarios: [{ name: 'sA' }] } }, [{ gate: { key: 'lc' }, kind: 'gate', scenario: 'sA', result }]).verdict, 'INFRA');
  // a scenario whose runs all completed BEFORE the abort keeps its own classification
  let w3;
  w3 = world({ n: 1, specs: [{ scenario: 'sA', lc1: 1, lc0: 0 }, { scenario: 'sB', lc1: 1, lc0: 0 }], startRun: async (job, h) => { const done = await h.defaultRun(job); if (job.scenario === 'sB') G.abortPhaseA(w3.ctx, 'lock holder died', { lockLost: true }); return done; } });
  await runPhaseA(w3.ctx);
  G.settleUnfinishedRuns(w3.states, 'lock holder died');
  G.finalizeAbort(w3.ctx);
  assert.equal(w3.states.get('sA').infraStop, null, 'sA finished before the abort');
  assert.match(w3.states.get('sB').infraStop, /phase A aborted/);
  // an abort that is not a lost lock leaves the run in flight alone
  let w2;
  w2 = world({ n: 1, specs: [{ scenario: 'sA', lc1: 1, lc0: 0 }], startRun: async (job, h) => { const done = h.defaultRun(job); G.abortPhaseA(w2.ctx, 'phase A error: other slot'); return done; } });
  await runPhaseA(w2.ctx);
  assert.equal(w2.states.get('sA').runs['gate-lc1-01'].infra, null);
});

test('N-4 / #15 runQueue throws the first error and carries the errors of ALL slots; the run-all error list has one entry per error (a HELD cannot hide another error)', async () => {
  const err = await runQueue2();
  assert.equal(err.allErrors.length, 2);
  const msgs = G.phaseAErrorMessages(err);
  assert.deepEqual(msgs, ['phase A: HELD: pins: x', 'phase A: other failure']);
  assert.equal(msgs.every((m) => /^phase A: HELD:/.test(m)), false, 'heldOnly is false: the exit stays 1');
  assert.deepEqual(G.phaseAErrorMessages(new Error('solo')), ['phase A: solo']);
});
async function runQueue2() {
  try {
    await G.runQueue([1, 2], 2, async (item) => { await sleepMs(5); throw new Error(item === 1 ? 'HELD: pins: x' : 'other failure'); });
  } catch (error) { return error; }
  throw new Error('runQueue did not throw');
}

test('N-5 / G-21 run.sh executed (steps 4 and 5 of the real script text, with a stubbed node tool): exit 5 of run-all keeps the exit code 5, still runs the verdict over the reports that exist, skips the one without', () => {
  const sh = fs.readFileSync(path.join(implDir, 'run.sh'), 'utf8');
  const tail = sh.slice(sh.indexOf('step "4 '));
  const dir = tmp();
  const outRoot = path.join(dir, 'out');
  fs.mkdirSync(path.join(outRoot, 'gate', 'sA'), { recursive: true });
  fs.writeFileSync(path.join(outRoot, 'gate', 'sA', 'report.json'), '{}');
  const script = `set -u -o pipefail
step() { echo "STEP $*"; }
logged() { echo "STUB $3 $*"; case "$3" in run-all) return "$RUNALL_RC" ;; verdict) return 4 ;; *) return 0 ;; esac; }
GATE_ID=gate; GATE_VERSION=4; OUT_ROOT=${outRoot}; TOOL=tool.mjs; JOBS=""; LOGDIR=${dir}; MANIFEST=m.json; PREREG=${dir}; REPORTS=""; gate_args=()
PENDING_SCENARIOS=$'sA\\nsB'
${tail}`;
  const run = (rc) => spawnSync('bash', ['-c', script], { encoding: 'utf8', env: { ...process.env, RUNALL_RC: String(rc) } });
  const r5 = run(5);
  assert.equal(r5.status, 5, r5.stdout + r5.stderr);
  assert.match(r5.stdout, /STUB verdict .*--reports [^ ]*\/gate\/sA\/report\.json( |$)/m);
  assert.doesNotMatch(r5.stdout, /sB\/report\.json/);
  assert.match(r5.stdout, /sB has no report/);
  const r0 = run(0);
  assert.equal(r0.status, 4, 'rc 0: the exit code is the verdict exit code');
  assert.match(r0.stdout, /sB\/report\.json/);
  const r1 = run(1);
  assert.equal(r1.status, 1, 'any other non-zero exit still stops the script before the verdict');
  assert.doesNotMatch(r1.stdout, /STUB verdict/);
});

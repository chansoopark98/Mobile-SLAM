// FORK of slam_refact/tools/harness/us026/gate_e2e.test.mjs for the v2 runner (version 2 / v2 manifest name; the engineOnExitRule tests are new).
// node --test slam_refact/tools/gates/us026-ba-e2e-v2/gate_e2e.test.mjs
// Synthetic checks of the US-026 e2e gate runner: the verdict aggregation rules and the refusals that must happen before any marker,
// output directory or browser exists. No original SDK, no Chromium, no replay.
import test from 'node:test';
import assert from 'node:assert/strict';
import fs from 'node:fs';
import os from 'node:os';
import path from 'node:path';
import { GATES, aggregate, checkHold, cmdRun, engineOnExitRule, root } from './gate_e2e.mjs';

const manifest = (names, n = names.length) => ({ registered_run: { scenarios: names.map((name) => ({ name })) }, independent_samples: { n } });
const rep = (g, scenario, result, kind = 'gate') => ({ gate: { key: g }, kind, scenario, result });
const ba = (verdict, cls = 'EVALUATED') => ({ class: cls, verdict });
const mse = (pairs, mid5) => ({ class: 'EVALUATED', pairs, mid5 });

test('ba verdict: all PASS / one FAIL / all BLOCKED / one missing / INVALID listed', () => {
  const m = manifest(['a', 'b', 'c']);
  assert.equal(aggregate('ba', m, [rep('ba', 'a', ba('PASS')), rep('ba', 'b', ba('PASS')), rep('ba', 'c', ba('PASS'))]).verdict, 'PASS');
  assert.equal(aggregate('ba', m, [rep('ba', 'a', ba('PASS')), rep('ba', 'b', ba('FAIL')), rep('ba', 'c', ba('PASS'))]).verdict, 'FAIL');
  assert.equal(aggregate('ba', m, ['a', 'b', 'c'].map((s) => rep('ba', s, ba(null, 'BLOCKED')))).verdict, 'BLOCKED');
  assert.equal(aggregate('ba', m, [rep('ba', 'a', ba('PASS')), rep('ba', 'b', ba('PASS'))]).verdict, 'UNEVALUATED');
  const r = aggregate('ba', m, [rep('ba', 'a', ba('PASS')), rep('ba', 'b', { class: 'INVALID', classReason: ['x'] }), rep('ba', 'c', ba(null, 'BLOCKED'))]);
  assert.equal(r.verdict, 'PASS');
  assert.equal(r.k, 1);
  assert.deepEqual(r.invalid.map((x) => x.scenario), ['b']);
  assert.equal(r.n_independent, 3);
});

test('verdict refuses dev reports, foreign gates and unregistered scenarios', () => {
  const m = manifest(['a']);
  assert.throws(() => aggregate('ba', m, [rep('ba', 'a', ba('PASS'), 'dev')]));
  assert.throws(() => aggregate('ba', m, [rep('mse', 'a', ba('PASS'))]));
  assert.throws(() => aggregate('ba', m, [rep('ba', 'z', ba('PASS'))]));
});

test('mse verdict: pairs and M-ID-5', () => {
  const m = manifest(['a', 'b']);
  const ok = { m2: { class: 'EVALUATED', verdict: 'PASS' }, m3: { class: 'BLOCKED', verdict: null } };
  const pass5 = { evaluated: true, verdict: 'PASS' };
  assert.equal(aggregate('mse', m, [rep('mse', 'a', mse(ok, pass5)), rep('mse', 'b', mse(ok, pass5))]).verdict, 'PASS');
  assert.equal(aggregate('mse', m, [rep('mse', 'a', mse(ok, { evaluated: true, verdict: 'FAIL' })), rep('mse', 'b', mse(ok, pass5))]).verdict, 'FAIL');
  assert.equal(aggregate('mse', m, [rep('mse', 'a', mse(ok, { evaluated: false })), rep('mse', 'b', mse(ok, { evaluated: false }))]).verdict, 'BLOCKED');
  const none = { m2: { class: 'BLOCKED', verdict: null } };
  assert.equal(aggregate('mse', m, [rep('mse', 'a', mse(none, pass5)), rep('mse', 'b', mse(none, pass5))]).verdict, 'BLOCKED');
  const fail = { m4: { class: 'EVALUATED', verdict: 'FAIL' } };
  assert.equal(aggregate('mse', m, [rep('mse', 'a', mse(fail, pass5)), rep('mse', 'b', mse(ok, pass5))]).verdict, 'FAIL');
});

const tmpPrereg = () => fs.mkdtempSync(path.join(os.tmpdir(), 'us026-hold-'));
const gateOf = (dir, g = 'ba') => ({ gateId: GATES[g].gateId, version: GATES[g].version, preregDir: dir, designationEnv: 'SLAM_REFACT_US026_TEST_DESIGNATION' });

test('hold: no manifest, dev scenario, steering env, unregistered scenario -> HELD before anything', () => {
  const dir = tmpPrereg();
  const outDir = path.join(dir, 'out-must-not-exist');
  assert.throws(() => checkHold({ g: 'ba', gate: gateOf(dir), scenario: 'tum-room1-full', outDir, env: {} }), /HELD: manifest/);
  assert.throws(() => checkHold({ g: 'ba', gate: gateOf(dir), scenario: 'flip6-n1', outDir, env: {} }), /HELD: args/);
  assert.throws(() => checkHold({ g: 'ba', gate: gateOf(dir), scenario: 'tum-room1-full', outDir, env: { BROWSER_EXECUTABLE: '/bin/true' } }), /HELD: env/);
  assert.throws(() => checkHold({ g: 'ba', gate: { ...gateOf(dir), gateId: 'other-gate' }, scenario: 'tum-room1-full', outDir, env: {} }), /HELD: args/);
  fs.writeFileSync(path.join(dir, 'us026-ba-e2e-gate.v2.manifest.json'), JSON.stringify({ registered_run: { scenarios: [{ name: 'flip6-s2' }] } }));
  assert.throws(() => checkHold({ g: 'ba', gate: gateOf(dir), scenario: 'tum-room1-full', outDir, env: {} }), /HELD: data/);
  assert.throws(() => checkHold({ g: 'ba', gate: gateOf(dir), scenario: 'flip6-s2', outDir, env: {} }), /HELD: config/);
  assert.equal(fs.existsSync(outDir), false);
  assert.deepEqual(fs.readdirSync(dir), ['us026-ba-e2e-gate.v2.manifest.json']);
});

test('dev kind refuses a registration scenario before any output exists', async () => {
  const tmp = fs.mkdtempSync(path.join(os.tmpdir(), 'us026-dev-'));
  const outRoot = 'build/slam-refact-full-20261019/us026/gate-dev-refusal-test-never-created';
  await assert.rejects(cmdRun({ gate: 'ba', kind: 'dev', scenario: 'tum-room1-full', 'out-root': outRoot, 'run-log': path.join(tmp, 'r.log'), purpose: 't' }), /refused/);
  await assert.rejects(cmdRun({ gate: 'mse', kind: 'dev', scenario: 'flip6-s2-absolute', 'out-root': outRoot, 'run-log': path.join(tmp, 'r2.log'), purpose: 't' }), /refused/);
  await assert.rejects(cmdRun({ gate: 'ba', kind: 'dev', scenario: 'flip6-n1', 'out-root': 'build/elsewhere', 'run-log': path.join(tmp, 'r3.log'), purpose: 't' }), /out-root/);
  assert.equal(fs.existsSync(path.join(root, outRoot)), false);
});

test('configuration: the arms and bundles the gate documents name', () => {
  assert.deepEqual(GATES.ba.arms.map((a) => a.name), ['poke', 'null']);
  assert.deepEqual(GATES.ba.bundles.map((b) => b.name), ['on', 'off', 'null']);
  assert.deepEqual(GATES.mse.arms.map((a) => a.name), ['null', 'm2', 'm3', 'm4', 'm5']);
  assert.equal(GATES.mse.bundles.length, 9);
  assert.deepEqual(GATES.ba.devScenarios, ['flip6-n1']);
  assert.deepEqual(GATES.mse.devScenarios, ['flip6-arcore-s3-absolute']);
});

test('v2 configuration: ba is version 2, mse untouched', () => {
  assert.equal(GATES.ba.version, 2);
  assert.equal(GATES.mse.version, 1);
});

test('US026B-M1 in code: only an abnormal BA-on replay with BA-off and null replays at exit 0 turns INVALID into a counted FAIL', () => {
  const inv = { class: 'INVALID', classReason: ['replay on exit 139'] };
  const rp = (on, off, nul) => ({ on: { exit: on }, off: { exit: off }, null: { exit: nul } });
  const r = engineOnExitRule(inv, rp(139, 0, 0));
  assert.equal(r.class, 'EVALUATED');
  assert.equal(r.verdict, 'FAIL');
  assert.equal(r.rule, 'US026B-M1');
  assert.equal(r.classReason.length, 2);
  assert.equal(engineOnExitRule(inv, rp(null, 0, 0)).class, 'EVALUATED');          // killed by a signal
  assert.equal(engineOnExitRule(inv, rp(1, 1, 0)).class, 'INVALID');               // off also abnormal: device
  assert.equal(engineOnExitRule(inv, rp(1, 0, 1)).class, 'INVALID');               // null also abnormal: device
  assert.equal(engineOnExitRule(inv, rp('no-bundle', 0, 0)).class, 'INVALID');     // no bundle is not an engine exit
  assert.equal(engineOnExitRule(inv, rp(0, 0, 0)).class, 'INVALID');               // on exited 0: not this rule
  assert.equal(engineOnExitRule(inv, {}).class, 'INVALID');                        // infrastructure failure: no replays
  const ok = { class: 'EVALUATED', verdict: 'PASS' };
  assert.equal(engineOnExitRule(ok, rp(1, 0, 0)), ok);                             // only INVALID results are touched
  const blocked = { class: 'BLOCKED', classReason: 'x' };
  assert.equal(engineOnExitRule(blocked, rp(1, 0, 0)), blocked);
});

// node --test slam_refact/tools/gates/us025-lc-e2e-v1/judge.test.mjs
// Synthetic checks of the decision rules above the comparator (judge.mjs). No original SDK, no Chromium, no replay, no comparator run.
import test from 'node:test';
import assert from 'node:assert/strict';
import { JUDGE_CONSTANTS, SCENARIO_RUNS, VERDICT_EXIT, aggregate, classifyLc0Run, classifyLc1Run, classifyScenario, engineOnExitRule, engineReplayVerdict } from './judge.mjs';

const C = JUDGE_CONSTANTS;
const goodChecks = () => [
  { name: 'attemptFrameSet', compared: C.MIN_ATTEMPTS_COMPARED, mismatch: 0, declaredU2: 0 },
  { name: 'candidateId', compared: C.MIN_UNDECLARED_PICKS + 1, mismatch: 0, declaredU2: 1 },
  { name: 'successCall', compared: 1, mismatch: 0, declaredU2: 0 },
  { name: 'traceAndWorldPointsBeforeSuccess', compared: C.MIN_TRACE_FRAMES, mismatch: 0, declaredU2: 0 },
];
const on = (verdict = 'PASS', over = {}) => ({ verdict, checks: goodChecks(), ...over });
const off = (verdict = 'PASS', over = {}) => ({ verdict, checks: [], ...over });
const ok = { on: { exit: 0 }, off: { exit: 0 } };
const run = (o = {}) => ({ name: 'r', infra: null, replays: ok, on: on(), off: off(), ...o });

test('lc1 run: EVALUATED only when ON passes, OFF reacts and every floor is met', () => {
  assert.equal(classifyLc1Run(run()).class, 'EVALUATED');
  assert.equal(classifyLc1Run(run({ on: on('PASS_DECLARED') })).class, 'EVALUATED');
  assert.equal(classifyLc1Run(run({ on: on('PASS_DECLARED') })).declaredU2, 1);
});

test('lc1 run: FAIL beats INVALID and TOOL_ERROR; INFRA is never a FAIL (M-3)', () => {
  assert.equal(classifyLc1Run(run({ on: on('FAIL') })).class, 'FAIL');
  assert.equal(classifyLc1Run(run({ off: off('FAIL') })).class, 'FAIL');
  assert.equal(classifyLc1Run(run({ on: on('FAIL'), off: off('TOOL_ERROR') })).class, 'FAIL');
  assert.equal(classifyLc1Run(run({ on: { verdict: 'TOOL_ERROR', checks: [] } })).class, 'TOOL_ERROR');
  assert.equal(classifyLc1Run(run({ on: { verdict: 'INVALID', checks: [], reason: 'x' } })).class, 'INVALID');
  assert.equal(classifyLc1Run({ name: 'r', infra: 'runner exit 1, status FAILED' }).class, 'INFRA');
  assert.equal(classifyLc1Run(run({ on: null })).class, 'TOOL_ERROR');   // the comparator produced nothing: not a FAIL, not a PASS
});

test('lc1 run: NO_TRIGGER, INSENSITIVE and inconsistent BLOCKED', () => {
  assert.equal(classifyLc1Run(run({ on: on('BLOCKED'), off: off('BLOCKED') })).class, 'NO_TRIGGER');
  assert.equal(classifyLc1Run(run({ on: on('BLOCKED'), off: off('PASS') })).class, 'INVALID');
  assert.equal(classifyLc1Run(run({ on: on('PASS'), off: off('BLOCKED') })).class, 'INSENSITIVE');
});

test('US025-M1: an abnormal engine ON replay with a clean OFF replay is a FAIL, whatever the comparator says', () => {
  for (const exit of [139, 1, null]) {
    const r = classifyLc1Run(run({ replays: { on: { exit }, off: { exit: 0 } }, on: null }));
    assert.equal(r.class, 'FAIL');
    assert.equal(r.rule, 'US025-M1');
  }
  assert.equal(engineOnExitRule({ on: { exit: 0 }, off: { exit: 0 } }), false);
  assert.equal(engineOnExitRule({ on: { exit: 1 }, off: { exit: 1 } }), false);   // OFF abnormal too: the runner makes this INFRA, not this rule
  assert.equal(engineOnExitRule({}), false);
});

test('US025-M2: ANY abnormal engine replay exit is a FAIL, not INFRA (prereview C2); only a harness cause (ENOSPC) is INFRA', () => {
  const fl = (r) => classifyLc1Run(run({ replays: r, on: on(), off: off() }));
  assert.equal(fl({ on: { exit: 0 }, off: { exit: 139 } }).class, 'FAIL');                      // the OFF (default) path crashes: used to be INFRA
  assert.equal(fl({ on: { exit: 0 }, off: { exit: 139 } }).rule, 'US025-M2');
  assert.equal(fl({ on: { exit: 1 }, off: { exit: 1 } }).class, 'FAIL');                        // both die
  assert.equal(fl({ on: { exit: 1 }, off: { exit: 1 } }).rule, 'US025-M2');
  assert.equal(fl({ on: { exit: null }, off: { exit: 0 } }).rule, 'US025-M1');                  // ON only: the M1 subset keeps its name
  assert.equal(fl({ on: { exit: 1, harness: true }, off: { exit: 0 } }).class, 'INFRA');        // no space left on device
  assert.equal(fl({ on: { exit: 1, harness: true }, off: { exit: 2 } }).class, 'FAIL');         // a real engine crash next to a harness one: FAIL first
  assert.equal(engineReplayVerdict({ on: { exit: 0 }, off: { exit: 0 } }), null);
  const nl = (replay) => classifyLc0Run({ name: 'z', infra: null, replay, null: null });
  assert.equal(nl({ exit: 139 }).class, 'FAIL');
  assert.equal(nl({ exit: 139 }).rule, 'US025-M2');
  assert.equal(nl({ exit: 1, harness: true }).class, 'INFRA');
});

test('selfCandidate and the PASS claim carry the N1 caveat and the per-scenario INFRA counts (prereview C6)', () => {
  const m = manifest(['a', 'b']);
  const scn = (self, n) => ({ class: 'EVALUATED', verdict: 'PASS', planned: { lc1: 4, lc0: 2 }, lc1: { EVALUATED: n, INFRA: 1 }, declaredU2: 0, selfCandidate: self });
  const allSelf = aggregate(m, [rep('a', scn(3, 3)), rep('b', cls('BLOCKED'))]);
  assert.equal(allSelf.verdict, 'PASS');
  assert.match(allSelf.claim, /3 of 3 successes were SELF-candidates/);
  assert.match(allSelf.claim, /UNVERIFIED/);
  assert.match(allSelf.claim, /not compared on the engine ON side/);
  assert.match(allSelf.claim, /"a":1/);
  const mixed = aggregate(m, [rep('a', scn(2, 3)), rep('b', cls('BLOCKED'))]);
  assert.doesNotMatch(mixed.claim, /non-self-candidate PnP/);
  const e = classifyLc1Run(run({ on: on('PASS', { detail: { selfCandidate: true } }) }));
  assert.equal(e.selfCandidate, true);
  assert.equal(classifyLc1Run(run()).selfCandidate, false);
});

test('floors: a degenerate comparison cannot be EVALUATED (M-7)', () => {
  const mod = (name, over) => on('PASS', { checks: goodChecks().map((c) => (c.name === name ? { ...c, ...over } : c)) });
  assert.equal(classifyLc1Run(run({ on: mod('attemptFrameSet', { compared: C.MIN_ATTEMPTS_COMPARED - 1 }) })).class, 'INVALID');
  assert.equal(classifyLc1Run(run({ on: mod('candidateId', { compared: 3, declaredU2: 2 }) })).class, 'INVALID');
  assert.equal(classifyLc1Run(run({ on: mod('traceAndWorldPointsBeforeSuccess', { compared: C.MIN_TRACE_FRAMES - 1 }) })).class, 'INVALID');
  assert.equal(classifyLc1Run(run({ on: mod('successCall', { compared: 0 }) })).class, 'INVALID');
  assert.equal(classifyLc1Run(run({ on: { verdict: 'PASS', checks: [] } })).class, 'INVALID');   // no checks at all
});

test('lc0 run (NULL arm)', () => {
  const nl = (verdict, c = C.MIN_TRACE_FRAMES) => ({ verdict, checks: [{ name: 'traceAndWorldPointsWholeRun', compared: c, mismatch: verdict === 'FAIL' ? 1 : 0 }] });
  assert.equal(classifyLc0Run({ name: 'z', infra: null, null: nl('PASS') }).class, 'NULL_ZERO');
  assert.equal(classifyLc0Run({ name: 'z', infra: null, null: nl('FAIL') }).class, 'FAIL');
  assert.equal(classifyLc0Run({ name: 'z', infra: null, null: nl('PASS', 0) }).class, 'INVALID');   // zero frames compared is not a pass
  assert.equal(classifyLc0Run({ name: 'z', infra: null, null: { verdict: 'INVALID', reason: 'status', checks: [] } }).class, 'INVALID');
  assert.equal(classifyLc0Run({ name: 'z', infra: null, null: { verdict: 'TOOL_ERROR', checks: [] } }).class, 'TOOL_ERROR');
  assert.equal(classifyLc0Run({ name: 'z', infra: 'status FAILED' }).class, 'INFRA');
  assert.equal(classifyLc0Run({ name: 'z', infra: null, null: null }).class, 'TOOL_ERROR');
});

const mk = (cls, n) => Array.from({ length: n }, (_, i) => ({ name: `${cls}-${i}`, class: cls, reasons: [], declaredU2: 0 }));
const sc = (planned, parts1, parts0, extra = {}) => classifyScenario({ scenario: 's', planned,
  lc1: Object.entries(parts1).flatMap(([c, n]) => mk(c, n)), lc0: Object.entries(parts0).flatMap(([c, n]) => mk(c, n)), ...extra });

test('scenario: EVALUATED / BLOCKED / INVALID / FAIL / INFRA', () => {
  const p = { lc1: 4, lc0: 2 };
  assert.equal(sc(p, { EVALUATED: 1, NO_TRIGGER: 3 }, { NULL_ZERO: 2 }).class, 'EVALUATED');
  assert.equal(sc(p, { NO_TRIGGER: 4 }, { NULL_ZERO: 2 }).class, 'BLOCKED');
  assert.equal(sc(p, { INSENSITIVE: 1, NO_TRIGGER: 3 }, { NULL_ZERO: 1, INFRA: 1 }).class, 'BLOCKED');
  assert.equal(sc(p, { INFRA: 4 }, { INFRA: 2 }).class, 'BLOCKED');                                              // nothing usable: reported, not a verdict
  assert.equal(sc(p, { EVALUATED: 1, INVALID: 1, NO_TRIGGER: 2 }, { NULL_ZERO: 2 }).class, 'INVALID');           // one INVALID run blocks the scenario (M-2)
  assert.equal(sc(p, { EVALUATED: 1, INFRA: 3 }, { NULL_ZERO: 2 }).class, 'EVALUATED');                           // INFRA runs are listed; the gate caps their share
  assert.equal(sc(p, { EVALUATED: 2, NO_TRIGGER: 2 }, { INFRA: 2 }).class, 'INVALID');                           // evaluated without a usable NULL run
  assert.equal(sc(p, { NO_TRIGGER: 4 }, { INFRA: 2 }).class, 'BLOCKED');                                         // no evaluation, no floor needed
  assert.equal(sc(p, { EVALUATED: 2, NO_TRIGGER: 1, FAIL: 1 }, { NULL_ZERO: 2 }).class, 'FAIL');
  assert.equal(sc(p, { EVALUATED: 2, NO_TRIGGER: 2 }, { NULL_ZERO: 1, FAIL: 1 }).class, 'FAIL');
  assert.equal(sc(p, { EVALUATED: 2, NO_TRIGGER: 2 }, { NULL_ZERO: 1, TOOL_ERROR: 1 }).class, 'INFRA');
  assert.equal(sc(p, { EVALUATED: 2, NO_TRIGGER: 2 }, { NULL_ZERO: 2 }, { infraFailure: 'resources' }).class, 'INFRA');
  assert.equal(sc(p, { EVALUATED: 3 }, { NULL_ZERO: 2 }).class, 'INVALID');                                      // fewer runs recorded than planned
  assert.equal(sc(p, { FAIL: 1, NO_TRIGGER: 3 }, { NULL_ZERO: 2 }, { infraFailure: 'x' }).class, 'FAIL');       // FAIL outranks an infrastructure failure
});

const manifest = (names) => ({ registered_run: { scenarios: names.map((name) => ({ name })) }, independent_samples: { n: 1 } });
const rep = (scenario, result, over = {}) => ({ gate: { key: 'lc' }, kind: 'gate', scenario, result, ...over });
const ev = (n) => ({ class: 'EVALUATED', verdict: 'PASS', lc1: { EVALUATED: n }, declaredU2: 0 });
const cls = (c) => ({ class: c, reasons: [c] });

test('gate verdict: PASS needs K_MIN evaluated runs; fewer is INSUFFICIENT; none is BLOCKED', () => {
  const m = manifest(['a', 'b']);
  assert.equal(aggregate(m, [rep('a', ev(C.K_MIN)), rep('b', cls('BLOCKED'))]).verdict, 'PASS');
  assert.equal(aggregate(m, [rep('a', ev(1)), rep('b', ev(1))]).verdict, 'INSUFFICIENT');
  assert.equal(aggregate(m, [rep('a', ev(C.K_MIN - 1)), rep('b', cls('BLOCKED'))]).verdict, 'INSUFFICIENT');
  assert.equal(aggregate(m, [rep('a', cls('BLOCKED')), rep('b', cls('BLOCKED'))]).verdict, 'BLOCKED');
  const pass = aggregate(m, [rep('a', ev(C.K_MIN)), rep('b', cls('BLOCKED'))]);
  assert.equal(pass.k_runs, C.K_MIN);
  assert.equal(pass.k_scenarios, 1);
  assert.match(pass.claim, /ONE dataset only/);
  assert.equal(aggregate(m, [rep('a', ev(2)), rep('b', ev(2))]).k_scenarios, 2);
});

test('gate verdict: an INVALID or INFRA scenario makes PASS impossible, FAIL outranks everything (M-2)', () => {
  const m = manifest(['a', 'b', 'c']);
  const base = [rep('a', ev(C.K_MIN + 5)), rep('b', cls('BLOCKED'))];
  assert.equal(aggregate(m, [...base, rep('c', cls('INVALID'))]).verdict, 'INVALID');
  assert.equal(aggregate(m, [...base, rep('c', cls('INFRA'))]).verdict, 'INFRA');
  assert.equal(aggregate(m, [...base, rep('c', cls('FAIL'))]).verdict, 'FAIL');
  assert.equal(aggregate(m, base).verdict, 'UNEVALUATED');
  assert.deepEqual(aggregate(m, base).unevaluated, ['c']);
  assert.equal(aggregate(m, [rep('a', cls('FAIL')), rep('b', cls('INFRA')), rep('c', cls('INVALID'))]).verdict, 'FAIL');
});

test('gate verdict: too many INFRA lc=1 runs (more than half of all planned) is INFRA, not PASS', () => {
  const m = manifest(['a', 'b']);
  const scn = (evaluated, infra, planned1) => ({ class: 'EVALUATED', verdict: 'PASS', planned: { lc1: planned1, lc0: 2 }, lc1: { EVALUATED: evaluated, INFRA: infra }, declaredU2: 0 });
  assert.equal(aggregate(m, [rep('a', scn(C.K_MIN, 5, 10)), rep('b', scn(0, 0, 10))]).verdict, 'PASS');       // 5 of 20
  assert.equal(aggregate(m, [rep('a', scn(C.K_MIN, 10, 10)), rep('b', scn(0, 1, 10))]).verdict, 'INFRA');     // 11 of 20
  const r = aggregate(m, [rep('a', scn(C.K_MIN, 2, 10)), rep('b', scn(0, 0, 10))]);
  assert.match(r.claim, /2 of 20 lc=1 runs were INFRA/);
});

test('gate verdict: refuses foreign reports, unregistered scenarios and duplicates', () => {
  const m = manifest(['a']);
  assert.throws(() => aggregate(m, [rep('a', ev(3), { kind: 'dev' })]));
  assert.throws(() => aggregate(m, [rep('a', ev(3), { gate: { key: 'ba' } })]));
  assert.throws(() => aggregate(m, [rep('z', ev(3))]));
  assert.throws(() => aggregate(m, [rep('a', ev(3)), rep('a', ev(3))]));
});

test('configuration the registration document names', () => {
  assert.deepEqual(Object.keys(SCENARIO_RUNS), ['flip6-s3-lowtex', 'flip6-s2', 'tum-room1-full', 'tum-room2-full', 'tum-room4-full']);
  assert.equal(SCENARIO_RUNS['flip6-s3-lowtex'].lc1, 16);
  assert.ok(Object.values(SCENARIO_RUNS).every((r) => r.lc0 === 2 && r.lc1 >= 4));
  assert.equal(SCENARIO_RUNS['flip6-s2'].lc1, 8);
  assert.equal(C.K_MIN, 3);
  assert.deepEqual(VERDICT_EXIT, { PASS: 0, FAIL: 1, BLOCKED: 4, UNEVALUATED: 4, INFRA: 6, INSUFFICIENT: 7, INVALID: 8 });
});

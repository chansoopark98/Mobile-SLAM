// node --test slam_refact/tools/gates/us025-lc-e2e-v2/judge.test.mjs
// Synthetic checks of the decision rules above the comparators (judge.mjs): the v1 tests adapted to v2 (floors are THIN, NO_TRIGGER needs the whole-run comparison) and
// the v2 cases (THIN, the 25 percent bound, the whole-run FAIL, the re-judgement of the v1 first-run shape). No original SDK, no Chromium, no replay, no comparator run.
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
const full = (verdict = 'PASS', over = {}) => ({ verdict, checks: [{ name: 'engineSuccessCalls', compared: 20, mismatch: 0, declaredU2: 0 }, { name: 'ringState', compared: 20, mismatch: 0, declaredU2: 2 }], ...over });
const run = (o = {}) => ({ name: 'r', infra: null, replays: ok, on: on(), off: off(), ...o });
const noTrigger = (o = {}) => run({ on: on('BLOCKED'), off: off('BLOCKED'), full: full(), ...o });

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
  assert.equal(classifyLc1Run(noTrigger()).class, 'NO_TRIGGER');
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
  assert.match(allSelf.claim, /compared over the whole run/);
  assert.match(allSelf.claim, /THIN/);
  assert.match(allSelf.claim, /"a":1/);
  const mixed = aggregate(m, [rep('a', scn(2, 3)), rep('b', cls('BLOCKED'))]);
  assert.doesNotMatch(mixed.claim, /non-self-candidate PnP/);
  const e = classifyLc1Run(run({ on: on('PASS', { detail: { selfCandidate: true } }) }));
  assert.equal(e.selfCandidate, true);
  assert.equal(classifyLc1Run(run()).selfCandidate, false);
});

test('floors (D-V2-1): a degenerate comparison cannot be EVALUATED; it is THIN, not INVALID, and never PASS (M-7)', () => {
  const mod = (name, over) => on('PASS', { checks: goodChecks().map((c) => (c.name === name ? { ...c, ...over } : c)) });
  const thin = (o) => classifyLc1Run(run({ on: o }));
  assert.equal(thin(mod('attemptFrameSet', { compared: C.MIN_ATTEMPTS_COMPARED - 1 })).class, 'THIN');
  assert.equal(thin(mod('candidateId', { compared: 3, declaredU2: 2 })).class, 'THIN');
  assert.equal(thin(mod('candidateId', { compared: 1, declaredU2: 0 })).class, 'THIN');            // the v1 flip6-s2 shape: one undeclared pick
  assert.equal(thin(mod('traceAndWorldPointsBeforeSuccess', { compared: C.MIN_TRACE_FRAMES - 1 })).class, 'THIN');
  assert.equal(thin(mod('successCall', { compared: 0 })).class, 'THIN');
  assert.equal(thin(on('PASS', { checks: [] })).class, 'THIN');   // no checks at all
  assert.match(thin(mod('candidateId', { compared: 1, declaredU2: 0 })).reasons[0], /undeclared candidate picks compared 1 < 2/);
  assert.equal(thin(mod('candidateId', { compared: C.MIN_UNDECLARED_PICKS, declaredU2: 0 })).class, 'EVALUATED');   // exactly at each floor is not thin
  assert.equal(thin(mod('attemptFrameSet', { compared: C.MIN_ATTEMPTS_COMPARED })).class, 'EVALUATED');
  // THIN is only the floor outcome: every other comparator outcome keeps the v1 class
  assert.equal(thin(on('FAIL')).class, 'FAIL');
  assert.equal(classifyLc1Run(run({ on: mod('candidateId', { compared: 1 }), off: off('FAIL') })).class, 'FAIL');   // FAIL outranks THIN (either arm)
  assert.equal(classifyLc1Run(run({ on: mod('candidateId', { compared: 1 }), off: off('INVALID') })).class, 'INVALID');
  assert.equal(classifyLc1Run(run({ on: mod('candidateId', { compared: 1 }), replays: { on: { exit: 0 }, off: { exit: 139 } } })).class, 'FAIL');   // US025-M2 outranks THIN
  assert.equal(classifyLc1Run(run({ on: mod('candidateId', { compared: 1 }), off: off('BLOCKED') })).class, 'INSENSITIVE');
  assert.equal(classifyLc1Run(run({ on: { verdict: 'INVALID', checks: [], reason: 'x' } })).class, 'INVALID');
});

test('D-V2-2: a NO_TRIGGER run needs the whole-run engine ON comparison; its FAIL, INVALID and TOOL_ERROR are the run class', () => {
  const nt = (f) => classifyLc1Run(noTrigger({ full: f }));
  assert.equal(nt(full('PASS')).class, 'NO_TRIGGER');
  const ok = nt(full('PASS_DECLARED'));
  assert.equal(ok.class, 'NO_TRIGGER');
  assert.equal(ok.wholeRunChecked, true);
  assert.equal(ok.fullDeclaredU2, 2);
  assert.equal(nt(full('FAIL', { reason: 'engineSuccessCalls' })).class, 'FAIL');       // the engine succeeds (or differs) where the original never did
  assert.equal(nt(full('FAIL')).rule, 'D-V2-2');
  assert.equal(nt(full('INVALID', { reason: 'status' })).class, 'INVALID');
  assert.equal(nt(full('BLOCKED')).class, 'INVALID');                                    // ON said no success, the whole-run arm says the original succeeded: inconsistent
  assert.equal(nt(full('TOOL_ERROR', { reason: 'exit 4' })).class, 'TOOL_ERROR');
  assert.equal(nt(null).class, 'TOOL_ERROR');                                            // no comparison is never a silent NO_TRIGGER
  assert.equal(nt(undefined).class, 'TOOL_ERROR');
  // the whole-run result is not consulted for any other class
  assert.equal(classifyLc1Run(run({ full: full('FAIL') })).class, 'EVALUATED');
  assert.equal(classifyLc1Run(run({ on: on('PASS'), off: off('BLOCKED'), full: full('FAIL') })).class, 'INSENSITIVE');
  // an engine ON replay crash is still a FAIL before any comparison (US025-M1)
  assert.equal(classifyLc1Run(noTrigger({ replays: { on: { exit: 139 }, off: { exit: 0 } }, on: null, off: null, full: null })).rule, 'US025-M1');
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

test('scenario (D-V2-1, C1): THIN never makes a scenario INVALID; THIN > EVALUATED is not EVALUATED (BLOCKED); a scenario of only THIN / NO_TRIGGER runs is BLOCKED; FAIL still wins', () => {
  const p = { lc1: 8, lc0: 2 };
  const s1 = sc(p, { EVALUATED: 1, THIN: 2, NO_TRIGGER: 2, INFRA: 3 }, { NULL_ZERO: 2 });          // the v1 flip6-s2 shape: THIN 2 > EVALUATED 1
  assert.equal(s1.class, 'BLOCKED');
  assert.match(s1.reasons[0], /THIN runs 2 > EVALUATED runs 1/);
  assert.equal(s1.lc1.THIN, 2);
  assert.equal(sc(p, { EVALUATED: 2, THIN: 2, NO_TRIGGER: 2, INFRA: 2 }, { NULL_ZERO: 2 }).class, 'EVALUATED');   // THIN == EVALUATED is still evaluated (boundary)
  assert.equal(sc(p, { EVALUATED: 3, THIN: 2, NO_TRIGGER: 3 }, { NULL_ZERO: 2 }).class, 'EVALUATED');
  assert.equal(sc(p, { EVALUATED: 2, THIN: 3, NO_TRIGGER: 3 }, { NULL_ZERO: 2 }).class, 'BLOCKED');
  assert.equal(sc(p, { THIN: 3, NO_TRIGGER: 5 }, { NULL_ZERO: 2 }).class, 'BLOCKED');
  assert.match(sc(p, { THIN: 3, NO_TRIGGER: 5 }, { NULL_ZERO: 2 }).reasons[0], /thin 3/);
  assert.equal(sc(p, { THIN: 8 }, { NULL_ZERO: 2 }).class, 'BLOCKED');
  assert.equal(sc(p, { EVALUATED: 1, THIN: 1, INVALID: 1, NO_TRIGGER: 5 }, { NULL_ZERO: 2 }).class, 'INVALID');   // any other INVALID cause is unchanged
  assert.equal(sc(p, { EVALUATED: 1, THIN: 1, FAIL: 1, NO_TRIGGER: 5 }, { NULL_ZERO: 2 }).class, 'FAIL');
  assert.equal(sc(p, { EVALUATED: 1, THIN: 1, NO_TRIGGER: 5 }, { NULL_ZERO: 2 }).class, 'INVALID');                 // 7 runs recorded, planned 8: run count rule unchanged
  assert.equal(sc(p, { EVALUATED: 1, THIN: 1, NO_TRIGGER: 6 }, { NULL_ZERO: 0, INFRA: 2 }).class, 'INVALID');       // evaluated without a usable NULL run: unchanged
  const w = classifyScenario({ scenario: 's', planned: p, lc0: mk('NULL_ZERO', 2), lc1: [...mk('NO_TRIGGER', 5).map((r) => ({ ...r, wholeRunChecked: true, fullDeclaredPicks: 2, fullDeclaredRing: 3 })), ...mk('EVALUATED', 3)] });
  assert.equal(w.wholeRunChecked, 5);
  assert.equal(w.wholeRunDeclaredPicks, 10);
  assert.equal(w.wholeRunDeclaredRing, 15);
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

test('gate verdict (D-V2-1, C1): THIN is not counted in k_runs; more than 25 percent of the runs that REACHED a success (EVALUATED + THIN + INSENSITIVE) THIN makes the gate INVALID; NO_TRIGGER and INFRA are not in the denominator', () => {
  const m = manifest(['a', 'b']);
  const scn = (evaluated, thin, insensitive, noTrigger, infra) => ({ class: 'EVALUATED', verdict: 'PASS', planned: { lc1: 20, lc0: 2 },
    lc1: { runs: evaluated + thin + insensitive + noTrigger + infra, EVALUATED: evaluated, THIN: thin, INSENSITIVE: insensitive, NO_TRIGGER: noTrigger, INFRA: infra }, declaredU2: 0, wholeRunChecked: noTrigger });
  const g = (a, b) => aggregate(m, [rep('a', a), rep('b', b)]);
  const r = g(scn(C.K_MIN + 3, 1, 0, 6, 0), scn(0, 0, 0, 10, 0));
  assert.equal(r.verdict, 'PASS');
  assert.equal(r.k_runs, C.K_MIN + 3);                                                   // THIN is not in k_runs
  assert.equal(r.thin_lc1_runs, 1);
  assert.equal(r.reached_success_lc1_runs, C.K_MIN + 4);
  assert.match(r.claim, /1 of 7 lc=1 runs that reached a success were THIN/);
  assert.equal(C.MAX_THIN_FRACTION, 0.25);
  assert.equal(g(scn(6, 2, 0, 0, 0), scn(0, 0, 0, 0, 0)).verdict, 'PASS');              // 2 of 8 = 25 percent exactly: not above the bound
  assert.equal(g(scn(6, 3, 0, 0, 0), scn(0, 0, 0, 0, 0)).verdict, 'INVALID');           // 3 of 9 = 33 percent
  assert.equal(g(scn(6, 3, 0, 0, 0), scn(0, 0, 0, 0, 0)).thin_runs_too_many, true);
  assert.match(g(scn(6, 3, 0, 0, 0), scn(0, 0, 0, 0, 0)).gate_invalid_reasons[0], /THIN lc=1 runs 3 of 9 runs that reached a success/);
  // the dilution C1 removes: 20 NO_TRIGGER runs next to 3 THIN of 9 reached no longer hide the bound (the old usable-run denominator gave 3 of 29 = 10 percent)
  assert.equal(g(scn(6, 3, 0, 20, 0), scn(0, 0, 0, 0, 0)).verdict, 'INVALID');
  // INSENSITIVE runs reached a success: they are in the denominator
  assert.equal(g(scn(6, 2, 4, 0, 0), scn(0, 0, 0, 0, 0)).verdict, 'PASS');              // 2 of 12
  assert.equal(g(scn(3, 2, 0, 0, 0), scn(0, 0, 0, 0, 0)).verdict, 'INVALID');           // 2 of 5 = 40 percent
  assert.equal(g(scn(6, 3, 0, 0, 7), scn(0, 0, 0, 0, 0)).verdict, 'INVALID');           // INFRA does not change it
});

test('gate verdict (D-V2-1): THIN outranks nothing above it: FAIL and INFRA stay first; INVALID stays above BLOCKED and INSUFFICIENT', () => {
  const m = manifest(['a', 'b']);
  const scn = (cl, thin, evaluated) => ({ class: cl, verdict: cl === 'EVALUATED' ? 'PASS' : undefined, planned: { lc1: 8, lc0: 2 }, lc1: { runs: 8, EVALUATED: evaluated, THIN: thin, INFRA: 0 }, declaredU2: 0, reasons: [cl] });
  assert.equal(aggregate(m, [rep('a', scn('FAIL', 8, 0)), rep('b', scn('EVALUATED', 0, 3))]).verdict, 'FAIL');
  assert.equal(aggregate(m, [rep('a', scn('INFRA', 8, 0)), rep('b', scn('EVALUATED', 0, 3))]).verdict, 'INFRA');
  assert.equal(aggregate(m, [rep('a', scn('BLOCKED', 4, 0)), rep('b', scn('BLOCKED', 0, 0))]).verdict, 'INVALID');   // nothing evaluated, every reached run THIN: INVALID, not BLOCKED
  assert.equal(aggregate(m, [rep('a', scn('BLOCKED', 1, 0)), rep('b', scn('BLOCKED', 0, 0))]).verdict, 'INVALID');   // 1 of 1 reached
  assert.equal(aggregate(m, [rep('a', scn('BLOCKED', 0, 0)), rep('b', scn('BLOCKED', 0, 0))]).verdict, 'BLOCKED');
  const few = { class: 'EVALUATED', verdict: 'PASS', planned: { lc1: 8, lc0: 2 }, lc1: { runs: 8, EVALUATED: 2, THIN: 0, INFRA: 0 }, declaredU2: 0 };
  assert.equal(aggregate(m, [rep('a', few), rep('b', scn('BLOCKED', 0, 0))]).verdict, 'INSUFFICIENT');
});

test('C3: the PASS claim carries the v1 INVALID fact, the after-the-fact rules and the declared-U2 count; C5: an unusable original is INFRA', () => {
  const m = manifest(['a']);
  const sc1 = { class: 'EVALUATED', verdict: 'PASS', planned: { lc1: 4, lc0: 2 }, lc1: { runs: 4, EVALUATED: 3, THIN: 0, INFRA: 0, NO_TRIGGER: 1 }, declaredU2: 0, wholeRunChecked: 1, wholeRunDeclaredPicks: 4, wholeRunDeclaredRing: 6 };
  const c = aggregate(m, [rep('a', sc1)]).claim;
  assert.match(c, /supersedes us025-lc-e2e-gate v1 \(verdict INVALID\)/);
  assert.match(c, /THIN rule and the carried-U2 rule were written after the v1 results were known/);
  assert.match(c, /identical to the original except 10 declared-U2 items: 4 candidate picks, 6 ring states/);
  assert.doesNotMatch(c, /identical to the original\)/);
  assert.equal(classifyLc1Run(noTrigger({ full: full('ORIGINAL_UNUSABLE', { reason: 'status COMPLETED_WITH_ERRORS' }) })).class, 'INFRA');
  assert.match(classifyLc1Run(noTrigger({ full: full('ORIGINAL_UNUSABLE', { reason: 'frames staged twice' }) })).reasons[0], /frames staged twice/);
  assert.equal(classifyLc1Run(noTrigger({ full: full('INVALID', { reason: 'engine made no LoopClose call' }) })).class, 'INVALID');   // engine side stays INVALID
  const f = classifyLc1Run(noTrigger({ full: full('PASS_DECLARED', { checks: [{ name: 'candidateId', compared: 9, declaredU2: 3 }, { name: 'ringState', compared: 9, declaredU2: 5 }] }) }));
  assert.equal(f.fullDeclaredPicks, 3);
  assert.equal(f.fullDeclaredRing, 5);
});

test('re-judgement of the v1 first-run shape with the v2 rules (C1 applied): flip6-s2 has THIN 2 > EVALUATED 1 and is not evaluated; PASS with k_runs 6, k_scenarios 1 (shape only; the registration records the data run)', () => {
  const m = manifest(['flip6-s3-lowtex', 'flip6-s2', 'tum-room1-full', 'tum-room2-full', 'tum-room4-full']);
  const nt = (n) => mk('NO_TRIGGER', n).map((r) => ({ ...r, wholeRunChecked: true }));
  const planned = (lc1) => ({ lc1, lc0: 2 });
  const s3 = classifyScenario({ scenario: 'flip6-s3-lowtex', planned: planned(16), lc0: mk('NULL_ZERO', 2), lc1: [...mk('EVALUATED', 6), ...nt(8), ...mk('INFRA', 2)] });
  const s2 = classifyScenario({ scenario: 'flip6-s2', planned: planned(8), lc0: mk('NULL_ZERO', 2), lc1: [...mk('EVALUATED', 1), ...mk('THIN', 2), ...nt(2), ...mk('INFRA', 3)] });
  const room = (n) => classifyScenario({ scenario: n, planned: planned(4), lc0: mk('NULL_ZERO', 2), lc1: nt(4) });
  assert.equal(s3.class, 'EVALUATED');
  assert.equal(s2.class, 'BLOCKED');
  assert.equal(room('tum-room1-full').class, 'BLOCKED');
  const rs = [rep('flip6-s3-lowtex', s3), rep('flip6-s2', s2), rep('tum-room1-full', room('tum-room1-full')), rep('tum-room2-full', room('tum-room2-full')), rep('tum-room4-full', room('tum-room4-full'))];
  const g = aggregate(m, rs);
  assert.equal(g.verdict, 'PASS');
  assert.equal(g.k_runs, 6);
  assert.equal(g.k_scenarios, 1);
  assert.match(g.claim, /ONE dataset only/);
  assert.equal(g.reached_success_lc1_runs, 9);
  assert.equal(g.thin_lc1_runs, 2);
  assert.equal(g.usable_lc1_runs, 31);
  assert.equal(g.infra_lc1_runs, 5);
  assert.equal(g.wholeRunCheckedNoTrigger, 22);
  // the same shape with one whole-run FAIL in a NO_TRIGGER run is a FAIL gate
  const s3f = classifyScenario({ scenario: 'flip6-s3-lowtex', planned: planned(16), lc0: mk('NULL_ZERO', 2), lc1: [...mk('EVALUATED', 6), ...nt(7), ...mk('FAIL', 1), ...mk('INFRA', 2)] });
  assert.equal(s3f.class, 'FAIL');
  assert.equal(aggregate(m, [rep('flip6-s3-lowtex', s3f), ...rs.slice(1)]).verdict, 'FAIL');
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
  assert.equal(C.MAX_THIN_FRACTION, 0.25);
  assert.equal(C.MAX_INFRA_LC1_FRACTION, 0.5);
  assert.equal(C.MIN_ATTEMPTS_COMPARED, 5);
  assert.equal(C.MIN_UNDECLARED_PICKS, 2);
  assert.equal(C.MIN_TRACE_FRAMES, 30);
  assert.equal(C.MIN_NULL_ZERO, 1);
  assert.deepEqual(VERDICT_EXIT, { PASS: 0, FAIL: 1, BLOCKED: 4, UNEVALUATED: 4, INFRA: 6, INSUFFICIENT: 7, INVALID: 8 });
});

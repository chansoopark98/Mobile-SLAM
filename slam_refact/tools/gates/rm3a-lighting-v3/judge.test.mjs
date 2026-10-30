// Unit test of judge.mjs on synthetic gtest logs (the registered output forms of xr_api_evidence_test.cc). Each mutant of the log must leave the PASS verdict.
//   node --test slam_refact/tools/gates/rm3a-lighting-v3/judge.test.mjs
import test from 'node:test';
import assert from 'node:assert/strict';
import { GATE_TESTS, NULLMUT_TESTS, gtestFilter, judgeGate, judgeNullMutation } from './judge.mjs';

const wrap = (t, body) => `[ RUN      ] XrApiEvidence.${t}\n${body}[       OK ] XrApiEvidence.${t} (12 ms)\n`;
const L1 = (id, label, checked, exact, bad) => `[${id} ${label}] registered rule gate: checked=${checked} bit_exact=${exact} -> ${exact === checked ? 'PASS' : 'FAIL'} (registered gate needs every sampled callback); mismatches:[${bad}]\n`;
const L2 = (id, label, n, exact = n, t = n, h = n, bad = '') => `[${id} ${label}] callbacks=${n} bit_exact=${exact} temperature_6500=${t} has_lighting=${h} mismatches:[${bad}]\n`;
const gateLog = (o = {}) => [
  wrap(GATE_TESTS[0], o.l1a ?? L1('G-L1a', 'api1', 137, 136, '6')),
  wrap(GATE_TESTS[1], o.l1b ?? L1('G-L1b', 'api3', 120, 119, '6')),
  wrap(GATE_TESTS[2], o.l2 ?? L2('G-L2', 'api1', 1009)),
  wrap(GATE_TESTS[3], o.l3 ?? L2('G-L3', 'api3', 600)),
  '[  PASSED  ] 4 tests.\n',
].join('');

test('registered output form: PASS, the registered rule gate is reported as FAIL 136/137', () => {
  const j = judgeGate(gateLog(), 0);
  assert.equal(j.verdict, 'PASS');
  assert.deepEqual(Object.values(j.items).map((i) => i.status), ['PASS', 'PASS', 'PASS', 'PASS']);
  assert.equal(j.reported.registered_rule_gate_api1, '136/137 FAIL');
});

test('mutants of the log are never PASS', () => {
  const cases = {
    'G-L2 one mismatching callback': [{ l2: L2('G-L2', 'api1', 1009, 1008, 1009, 1009, '777') }, 'FAIL'],
    'G-L3 one mismatching callback': [{ l3: L2('G-L3', 'api3', 600, 599, 600, 600, '5') }, 'FAIL'],
    'G-L2 temperature not 6500': [{ l2: L2('G-L2', 'api1', 1009, 1009, 1008) }, 'FAIL'],
    'G-L2 has_lighting missing': [{ l2: L2('G-L2', 'api1', 1009, 1009, 1009, 1008) }, 'FAIL'],
    'G-L2 truncated to 160 callbacks (v1 Addendum)': [{ l2: L2('G-L2', 'api1', 160) }, 'FAIL'],
    'G-L1a mismatch set {7}': [{ l1a: L1('G-L1a', 'api1', 137, 136, '7') }, 'FAIL'],
    'G-L1a mismatch set {6,7}': [{ l1a: L1('G-L1a', 'api1', 137, 135, '6,7') }, 'FAIL'],
    'G-L1a mismatch set empty': [{ l1a: L1('G-L1a', 'api1', 137, 137, '') }, 'FAIL'],
    'G-L1b subset 119': [{ l1b: L1('G-L1b', 'api3', 119, 118, '6') }, 'FAIL'],
    'G-L2 line missing': [{ l2: '' }, 'INVALID'],
    'G-L3 line twice': [{ l3: L2('G-L3', 'api3', 600) + L2('G-L3', 'api3', 600) }, 'INVALID'],
    'G-L2 label of the other data set': [{ l2: L2('G-L2', 'api3', 1009) }, 'INVALID'],
  };
  for (const [name, [o, want]] of Object.entries(cases)) {
    const j = judgeGate(gateLog(o), 1);
    assert.equal(j.verdict, want, name);
  }
});

test('structure: exit code, extra or missing or failed tests', () => {
  assert.equal(judgeGate(gateLog(), 1).verdict, 'INVALID');                // all lines PASS but a non-zero exit code
  assert.equal(judgeGate(gateLog(), null).verdict, 'INVALID');
  assert.equal(judgeGate(gateLog() + wrap('LightingGateMutationOneUlp', ''), 0).verdict, 'INVALID');   // a test outside the registered four
  assert.equal(judgeGate(gateLog().replace('[ RUN      ] XrApiEvidence.LightingMatchesOriginalBitExactTum', '[ RUN      ] XrApiEvidence.Other'), 0).verdict, 'INVALID');
  const failedTest = gateLog().replace(`[       OK ] XrApiEvidence.${GATE_TESTS[2]} (12 ms)`, `[  FAILED  ] XrApiEvidence.${GATE_TESTS[2]} (12 ms)`);
  assert.equal(judgeGate(failedTest, 1).verdict, 'FAIL');                  // a failed test with a printed line that agrees: the in-test EXPECT (e.g. the callback-6 equals callback-5 check) is part of the gate
  assert.equal(judgeGate('', 0).verdict, 'INVALID');
  assert.equal(judgeGate('crash', 139).verdict, 'INVALID');
});

test('the filters are disjoint: the real run has no null-run/mutation test and the dry run no gate test', () => {
  assert.equal(GATE_TESTS.length, 4);
  assert.equal(NULLMUT_TESTS.length, 7);
  assert.equal(new Set([...GATE_TESTS, ...NULLMUT_TESTS]).size, 11);
  assert.ok(!gtestFilter(NULLMUT_TESTS).includes('LightingMatchesOriginal'));
  assert.ok(!gtestFilter(NULLMUT_TESTS).includes('LightingRegisteredAssociation'));
  assert.ok(!gtestFilter(GATE_TESTS).includes('Mutation') && !gtestFilter(GATE_TESTS).includes('NullRun'));
});

const nmLog = (o = {}) => [
  wrap(NULLMUT_TESTS[0], o.n1 ?? '[N1 api1] rows=1009 mismatches=0\n[N1 api3] rows=600 mismatches=0\n'),
  wrap(NULLMUT_TESTS[1], o.n2 ?? '[N2 api3] callbacks=60 differing_between_two_runs=0\n'),
  wrap(NULLMUT_TESTS[2], o.n2b ?? '[N2b api1] callbacks=60 differing_between_two_runs=0\n'),
  wrap(NULLMUT_TESTS[3], o.m1 ?? '[M1 api3] frame+1 input: mismatching callbacks=58 of 60 [1,2,3,4,5,6,7,8,9,10,11,12,... (58 total)]\n'),
  wrap(NULLMUT_TESTS[4], o.m2 ?? '[M2 api1] one ulp at callback 501: mismatches=[501]\n'),
  wrap(NULLMUT_TESTS[5], o.m3 ?? '[M3 api1] own-frame rule: checked=137 mismatches=132\n'),
  wrap(NULLMUT_TESTS[6], o.m3b ?? '[M3b api3] own-frame rule: checked=120 mismatches=118\n'),
  '[  PASSED  ] 7 tests.\n',
].join('');

test('null-run and mutation log: registered form PASS, an undetected mutation or a noisy null-run is not PASS', () => {
  assert.equal(judgeNullMutation(nmLog(), 0).verdict, 'PASS');
  assert.equal(judgeNullMutation(nmLog({ n1: '[N1 api1] rows=1009 mismatches=0\n[N1 api3] rows=600 mismatches=1\n' }), 1).verdict, 'FAIL');
  assert.equal(judgeNullMutation(nmLog({ n1: '[N1 api1] rows=1009 mismatches=0\n' }), 0).verdict, 'INVALID');
  assert.equal(judgeNullMutation(nmLog({ n2: '[N2 api3] callbacks=60 differing_between_two_runs=1\n' }), 1).verdict, 'FAIL');
  assert.equal(judgeNullMutation(nmLog({ m1: '[M1 api3] frame+1 input: mismatching callbacks=0 of 60 []\n' }), 1).verdict, 'FAIL');
  assert.equal(judgeNullMutation(nmLog({ m2: '[M2 api1] one ulp at callback 501: mismatches=[]\n' }), 1).verdict, 'FAIL');
  assert.equal(judgeNullMutation(nmLog({ m2: '[M2 api1] one ulp at callback 501: mismatches=[501,502]\n' }), 1).verdict, 'FAIL');
  assert.equal(judgeNullMutation(nmLog({ m3: '[M3 api1] own-frame rule: checked=137 mismatches=1\n' }), 1).verdict, 'FAIL');
  assert.equal(judgeNullMutation(nmLog({ m3b: '' }), 0).verdict, 'INVALID');
  assert.equal(judgeNullMutation(nmLog(), 1).verdict, 'INVALID');
});

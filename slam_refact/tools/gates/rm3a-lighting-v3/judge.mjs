// rm3a-lighting-gate.v3: verdict of a gtest log of sr_tests_xr_api_evidence. Pure functions (no I/O); the driver run.mjs calls them.
//
//   judgeGate(logText, exitCode)          the four registered gates G-L1a, G-L1b, G-L2, G-L3 (gate file section 3)
//   judgeNullMutation(logText, exitCode)  the null-runs N1, N2, N2b and the mutations M1, M2, M3, M3b (gate file section 5)
//
// Result: { verdict: 'PASS' | 'FAIL' | 'INVALID', items: {...}, reasons: [...], reported: {...} }.  FAIL-first: one judged item that is FAIL makes the
// verdict FAIL even if another item is INVALID; INVALID (a log that cannot be read as the registered output) is never a PASS and never a FAIL.
// The numbers below are the ones of gate file section 3 and 5; they are copied from the md, not derived from a result.

export const GATE_TESTS = [
  'LightingRegisteredAssociationReport',        // G-L1a
  'LightingRegisteredAssociationReportTum',     // G-L1b
  'LightingMatchesOriginalBitExact',            // G-L2
  'LightingMatchesOriginalBitExactTum',         // G-L3
];
export const NULLMUT_TESTS = [
  'LightingGateNullRunComparatorSelfIdentity',          // N1
  'LightingGateNullRunPipelineDeterminism',             // N2
  'LightingGateNullRunPipelineDeterminismApi1',         // N2b
  'LightingGateMutationShiftedInputFrames',             // M1
  'LightingGateMutationOneUlp',                         // M2
  'LightingGateMutationWrongAssociationRule',           // M3
  'LightingGateMutationWrongAssociationRuleTum',        // M3b
];
export const gtestFilter = (tests) => tests.map((t) => `XrApiEvidence.${t}`).join(':');

const lines = (t) => String(t).split('\n').map((l) => l.replace(/\r$/, ''));
const ints = (s) => (s === '' ? [] : s.split(',').map(Number));

// the tests gtest ran and their status: "[ RUN      ] XrApiEvidence.X", "[       OK ] XrApiEvidence.X (n ms)", "[  FAILED  ] XrApiEvidence.X (n ms)"
function testStatus(ls) {
  const ran = [], ok = [], failed = [];
  for (const l of ls) {
    let m;
    if ((m = /^\[ RUN      \] XrApiEvidence\.(\w+)$/.exec(l))) ran.push(m[1]);
    else if ((m = /^\[       OK \] XrApiEvidence\.(\w+) \(\d+ ms\)$/.exec(l))) ok.push(m[1]);
    else if ((m = /^\[  FAILED  \] XrApiEvidence\.(\w+) \(\d+ ms\)$/.exec(l))) failed.push(m[1]);
  }
  return { ran, ok, failed };
}

// exactly one line of the pattern; returns the match or null
function one(ls, re, reasons, what) {
  const hits = ls.map((l) => re.exec(l)).filter(Boolean);
  if (hits.length !== 1) { reasons.push(`${what}: ${hits.length} lines of the registered output form, expected exactly 1`); return null; }
  return hits[0];
}

function structure(ls, exitCode, tests, reasons) {
  const st = testStatus(ls);
  const want = new Set(tests);
  const extra = st.ran.filter((t) => !want.has(t));
  const missing = tests.filter((t) => !st.ran.includes(t));
  if (extra.length) reasons.push(`tests ran that are not registered for this run: ${extra.join(',')}`);
  if (missing.length) reasons.push(`registered tests that did not run: ${missing.join(',')}`);
  if (new Set(st.ran).size !== st.ran.length) reasons.push('a test ran more than once');
  if (!Number.isInteger(exitCode)) reasons.push(`the test binary has no exit code (${exitCode})`);
  return { st, ok: extra.length === 0 && missing.length === 0 && Number.isInteger(exitCode) && new Set(st.ran).size === st.ran.length };
}

export function judgeGate(logText, exitCode) {
  const ls = lines(logText);
  const reasons = [];
  const { st, ok: structOk } = structure(ls, exitCode, GATE_TESTS, reasons);
  const items = {};
  const reported = {};
  const testOk = (t) => st.ok.includes(t);
  // G-L1a / G-L1b: registered rule, registered subset, mismatch set exactly {6}
  for (const [id, label, test, subset] of [['G-L1a', 'api1', GATE_TESTS[0], 137], ['G-L1b', 'api3', GATE_TESTS[1], 120]]) {
    const r = [];
    const m = one(ls, new RegExp(`^\\[${id} ${label}\\] registered rule gate: checked=(\\d+) bit_exact=(\\d+) -> (PASS|FAIL) \\(registered gate needs every sampled callback\\); mismatches:\\[([0-9,]*)\\]$`), r, id);
    if (m === null) { items[id] = { status: 'INVALID', reasons: r }; continue; }
    const checked = Number(m[1]), exact = Number(m[2]), bad = ints(m[4]);
    reported[`registered_rule_gate_${label}`] = `${exact}/${checked} ${m[3]}`;   // the registered rule as registered (137/137 or nothing): reported unchanged
    const pass = checked === subset && bad.length === 1 && bad[0] === 6 && exact === checked - 1 && testOk(test);
    items[id] = { status: pass ? 'PASS' : 'FAIL', checked, bit_exact: exact, mismatches: bad, test_ok: testOk(test) };
  }
  // G-L2 / G-L3: engine pipeline, every callback, bit-equal exposure, temperature 6500, has_lighting
  for (const [id, label, test, n] of [['G-L2', 'api1', GATE_TESTS[2], 1009], ['G-L3', 'api3', GATE_TESTS[3], 600]]) {
    const r = [];
    const m = one(ls, new RegExp(`^\\[${id} ${label}\\] callbacks=(\\d+) bit_exact=(\\d+) temperature_6500=(\\d+) has_lighting=(\\d+) mismatches:\\[([0-9,]*)\\]$`), r, id);
    if (m === null) { items[id] = { status: 'INVALID', reasons: r }; continue; }
    const [cb, exact, t6500, hasL] = m.slice(1, 5).map(Number);
    const bad = ints(m[5]);
    const pass = cb === n && exact === n && t6500 === n && hasL === n && bad.length === 0 && testOk(test);
    items[id] = { status: pass ? 'PASS' : 'FAIL', callbacks: cb, bit_exact: exact, temperature_6500: t6500, has_lighting: hasL, mismatches: bad, test_ok: testOk(test) };
  }
  // a gate whose line is absent although its test failed (e.g. an input file missing: ASSERT before the print) is INVALID, not FAIL
  const statuses = Object.values(items).map((i) => i.status);
  const exitConsistent = (exitCode === 0) === (statuses.every((s) => s === 'PASS') && st.failed.length === 0);
  if (!exitConsistent && statuses.every((s) => s === 'PASS')) reasons.push(`all four items PASS but the exit code is ${exitCode} or a test is FAILED`);
  let verdict;
  if (statuses.includes('FAIL')) verdict = 'FAIL';
  else if (!structOk || statuses.includes('INVALID') || !exitConsistent) verdict = 'INVALID';
  else verdict = 'PASS';
  for (const [k, v] of Object.entries(items)) if (v.reasons) reasons.push(...v.reasons.map((x) => `${k}: ${x}`));
  return { verdict, items, reasons, reported, tests: st };
}

export function judgeNullMutation(logText, exitCode) {
  const ls = lines(logText);
  const reasons = [];
  const { st, ok: structOk } = structure(ls, exitCode, NULLMUT_TESTS, reasons);
  const items = {};
  const need = (id, re, check) => {
    const hits = ls.map((l) => re.exec(l)).filter(Boolean);
    if (hits.length === 0) { items[id] = { status: 'INVALID', reasons: [`${id}: no line of the registered output form`] }; return; }
    const results = hits.map((m) => check(m));
    items[id] = { status: results.every(Boolean) ? 'PASS' : 'FAIL', lines: hits.map((m) => m[0]) };
  };
  need('N1', /^\[N1 (api1|api3)\] rows=(\d+) mismatches=(\d+)$/, (m) => m[3] === '0' && (m[1] === 'api1' ? m[2] === '1009' : m[2] === '600'));
  if (items.N1.status === 'PASS' && items.N1.lines.length !== 2) items.N1 = { status: 'INVALID', reasons: ['N1: expected one line for api1 and one for api3'] };
  need('N2', /^\[N2 api3\] callbacks=60 differing_between_two_runs=(\d+)$/, (m) => m[1] === '0');
  need('N2b', /^\[N2b api1\] callbacks=60 differing_between_two_runs=(\d+)$/, (m) => m[1] === '0');
  need('M1', /^\[M1 api3\] frame\+1 input: mismatching callbacks=(\d+) of 60 \[[0-9,.() a-z]*\]$/, (m) => Number(m[1]) >= 1);
  need('M2', /^\[M2 api1\] one ulp at callback (\d+): mismatches=\[([0-9,]*)\]$/, (m) => m[2] === m[1]);
  need('M3', /^\[M3 api1\] own-frame rule: checked=(\d+) mismatches=(\d+)$/, (m) => m[1] === '137' && Number(m[2]) > 1);
  need('M3b', /^\[M3b api3\] own-frame rule: checked=(\d+) mismatches=(\d+)$/, (m) => m[1] === '120' && Number(m[2]) > 1);
  const statuses = Object.values(items).map((i) => i.status);
  let verdict;
  if (statuses.includes('FAIL')) verdict = 'FAIL';
  else if (!structOk || statuses.includes('INVALID')) verdict = 'INVALID';
  else if (exitCode !== 0 || st.failed.length) { reasons.push(`items PASS but exit code ${exitCode} / failed tests ${st.failed.join(',')}`); verdict = 'INVALID'; }
  else verdict = 'PASS';
  for (const [k, v] of Object.entries(items)) if (v.reasons) reasons.push(...v.reasons);
  return { verdict, items, reasons, tests: st };
}

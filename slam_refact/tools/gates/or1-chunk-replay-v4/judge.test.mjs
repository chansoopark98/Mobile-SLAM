// node --test slam_refact/tools/gates/or1-chunk-replay-v4/judge.test.mjs
// Unit tests of the v4 judge on synthetic reports (no engine, no browser). SLAM_REFACT_OR1V4_JUDGE=<path> runs the same tests against a mutated copy of
// judge.mjs (judge-mutants.mjs: the tests must fail on every mutant).
import assert from 'node:assert/strict';
import { spawnSync } from 'node:child_process';
import fs from 'node:fs';
import path from 'node:path';
import test from 'node:test';
import { pathToFileURL } from 'node:url';

const judgeFile = process.env.SLAM_REFACT_OR1V4_JUDGE ?? path.join(import.meta.dirname, 'judge.mjs');
const { judgeScenario, judgeGate, EXIT, runCli } = await import(pathToFileURL(judgeFile).href);
const { canon } = await import(pathToFileURL(path.join(import.meta.dirname, '../../../web-chunk/tools/or1/canon.mjs')).href);

const repo = path.resolve(import.meta.dirname, '../../../..');
const scratch = path.join(repo, 'build/slam-refact-g4-20261025/unit-tmp', `judge-${process.pid}`);
fs.mkdirSync(scratch, { recursive: true });
process.on('exit', () => fs.rmSync(scratch, { recursive: true, force: true }));

const rotationSpec = (name, w, h, rot) => ({ name, family: 'rotation', record: 'rec-r', capture: { width: w, height: h, rotation: rot }, expected: { calls: 61, frames: 60 },
  floors: { c4: null, min_fresh_nonrepeated: 48, min_distinct_rotations: 36 } });
const portraitSpec = (name, record) => ({ name, family: 'portrait', record, capture: { width: 480, height: 640, rotation: 0 }, expected: { calls: 381, frames: 380 },
  floors: { c4: { r_min: 0.1, k_min: 2 }, min_fresh_nonrepeated: 342, min_distinct_rotations: null } });
const SPECS = [portraitSpec('P1', 'A'), portraitSpec('P2', 'A'), portraitSpec('P3', 'B'), portraitSpec('P4', 'B'),
  rotationSpec('R0', 480, 640, 0), rotationSpec('R90', 640, 480, 90), rotationSpec('R180', 480, 640, 180), rotationSpec('R270', 640, 480, -90)];
const CTX = { beforeReviewSha256: 'a'.repeat(64), kMin: { rotationScenarios: 4, portraitRecords: 2 } };

function goodReport(spec, { limited = 0.2, k = 4 } = {}) {
  const { calls, frames } = spec.expected;
  const gc = { captureWidth: canon(spec.capture.width), captureHeight: canon(spec.capture.height), rotationDeg: canon(spec.capture.rotation), manufacturer: '', model: 'K',
    estimateScale: false, disableVio: false, leftHandedAxes: false, mirroredDisplay: false };
  const rep = (n) => Array.from({ length: n }, (_, i) => ({ rep: i + 1, identical: true, outputs: { frames, identicalFrames: frames, firstDiff: null }, inputs: { identical: true, mismatches: 0 } }));
  const sha = (c) => [c, c, c];
  return {
    meta: { gate: { id: 'or1-chunk-replay-gate', version: 4, datum: spec.name, manifest: { beforeReviewSha256: CTX.beforeReviewSha256 } }, kind: 'gate', arms: 'WCN', reps: 3 },
    gates: {
      'IN-CW': { calls, countA: calls, countB: calls, identical: true, mismatches: 0, first: null },
      'OUT-CW': { frames, identicalFrames: frames, identical: true, firstDiff: null, mismatchedFrames: 0 },
      ANS: { answers: calls, nullAnswers: 1, compared: frames, mismatches: 0, identical: true, first: null }, chunkErrors: [],
      clock: { clock: 3, video: 2, clockLost: 0, videoLost: 0 }, REP: { W: rep(2), C: rep(2), N: rep(2) },
      inputStats: { rows: frames, fresh: frames, repeated: 1, distinctRotations: frames - 2, status: {}, reason: {}, positionRange: [] },
      glueConfig: { W: { ...gc, facing: [1] }, C: { ...gc }, diffPaths: ['facing'] },
      C4: { rows: frames - 1, limited: 1, r: limited, k, degenerate: limited < 0.1 || k < 2 },
    },
    rowsSha256: { W: sha('w'), C: sha('w'), N: sha('n') },
  };
}
const allGood = () => Object.fromEntries(SPECS.map((s) => [s.name, goodReport(s)]));
const mutated = (name, fn) => { const r = allGood(); fn(r[name].gates, r[name]); return r; };
const judge = (reports) => judgeGate(SPECS, reports, CTX);
const byName = (res, name) => res.perScenario.find((p) => p.name === name);

test('J1: eight good reports are PASS, exit 0, the counts are the registered ones', () => {
  const r = judge(allGood());
  assert.equal(r.verdict, 'PASS'); assert.equal(r.exit, 0); assert.equal(EXIT.PASS, 0);
  assert.deepEqual([r.counts.registered, r.counts.pass, r.counts.fail, r.counts.invalid, r.counts.missing], [8, 8, 0, 0, 0]);
});

test('J2: a mismatch in any graded comparison of a valid report is FAIL (exit 1) and names the comparison', () => {
  const cases = [['IN-CW', (g) => { g['IN-CW'].identical = false; g['IN-CW'].mismatches = 1; }], ['OUT-CW', (g) => { g['OUT-CW'].identical = false; g['OUT-CW'].mismatchedFrames = 2; }],
    ['ANS', (g) => { g.ANS.identical = false; g.ANS.mismatches = 1; }], ['C7', (g) => { g.clock.clockLost = 1; }],
    ['REP', (g) => { g.REP.C[1].identical = false; }], ['REP-N', (g) => { g.REP.N[0].identical = false; }], ['REP-W', (g) => { g.REP.W[1].identical = false; }], ['SES', (g) => { g.glueConfig.C.rotationDeg = canon(0); }],
    ['chunk error', (g) => { g.chunkErrors = ['boom']; }]];
  for (const [label, fn] of cases) {
    const res = judge(mutated('R90', fn));
    assert.equal(res.verdict, 'FAIL', label); assert.equal(res.exit, 1, label);
    assert.equal(byName(res, 'R90').status, 'FAIL', label);
    assert.equal(byName(res, 'R0').status, 'PASS', 'the other scenarios keep their status');
  }
  const sesOnly = judge(mutated('R90', (g) => { g.glueConfig.C.rotationDeg = canon(0); }));
  assert.match(byName(sesOnly, 'R90').reasons.join(' '), /SES.*rotationDeg/);
});

test('J3: a pass that is not strictly `true` is not a pass (undefined, 1, "true")', () => {
  for (const v of [undefined, 1, 'true', null]) {
    const res = judge(mutated('R180', (g) => { g['IN-CW'].identical = v; }));
    assert.equal(res.verdict, 'FAIL', String(v));
  }
});

test('J4: the registered call and frame counts are required exactly; a chunk that drops a frame (fewer calls) is FAIL, a direct arm with the wrong count is INVALID', () => {
  let res = judge(mutated('R90', (g) => { g['IN-CW'].countA = 60; g['IN-CW'].identical = true; g['IN-CW'].mismatches = 0; }));
  assert.equal(byName(res, 'R90').status, 'FAIL');
  res = judge(mutated('R90', (g) => { g['OUT-CW'].frames = 59; g['OUT-CW'].identicalFrames = 59; }));
  assert.equal(byName(res, 'R90').status, 'FAIL');
  res = judge(mutated('R90', (g) => { g.ANS.answers = 60; g.ANS.compared = 59; }));  // internally consistent answers, but not the registered number
  assert.equal(byName(res, 'R90').status, 'FAIL');
  res = judge(mutated('R90', (g) => { g['IN-CW'].countB = 60; g['IN-CW'].countA = 60; }));
  assert.equal(byName(res, 'R90').status, 'INVALID');
  assert.equal(res.verdict, 'INVALID'); assert.equal(res.exit, 4);
});

test('J5: the direct arm built with another capture size / rotation than registered: INVALID (apparatus), never PASS', () => {
  const res = judge(mutated('R270', (g) => { g.glueConfig.W.rotationDeg = canon(90); g.glueConfig.C.rotationDeg = canon(90); }));
  assert.equal(byName(res, 'R270').status, 'INVALID');
  assert.match(byName(res, 'R270').reasons.join(' '), /direct arm W built its glue/);
  assert.equal(res.verdict, 'INVALID');
});

test('J6 (M-2): INVALID and MISSING scenarios are never dropped: one PASS and seven INVALID / MISSING is INVALID, not PASS', () => {
  const reports = Object.fromEntries(SPECS.map((s) => [s.name, null]));
  reports.P1 = goodReport(SPECS[0]);
  let res = judge(reports);
  assert.equal(res.verdict, 'INVALID'); assert.equal(res.exit, 4);
  assert.equal(res.counts.missing, 7); assert.equal(res.counts.pass, 1);
  const rr = allGood(); rr.R0 = { meta: {}, gates: null };
  res = judge(rr);
  assert.equal(byName(res, 'R0').status, 'INVALID'); assert.equal(res.verdict, 'INVALID');
  const unreadable = allGood(); unreadable.R0 = { __unreadable: 'Unexpected token' };
  assert.equal(judge(unreadable).verdict, 'INVALID');
});

test('J7 (M-3): a missing report is MISSING (INVALID), never FAIL; a FAIL elsewhere still decides FAIL', () => {
  const missing = allGood(); missing.R90 = null;
  const res = judge(missing);
  assert.equal(byName(res, 'R90').status, 'MISSING'); assert.equal(res.verdict, 'INVALID'); assert.notEqual(res.exit, EXIT.FAIL);
  const both = mutated('R0', (g) => { g['OUT-CW'].identical = false; g['OUT-CW'].mismatchedFrames = 1; }); both.R90 = null;
  assert.equal(judge(both).verdict, 'FAIL');
});

test('J8 (M-7): a degenerate input removes PASS evidence and never a FAIL; rotation scenarios must all have evidence, portrait needs two valid records', () => {
  // a rotation scenario with too few fresh responses: not a PASS
  let res = judge(mutated('R180', (g) => { g.inputStats.fresh = 40; }));
  assert.equal(byName(res, 'R180').status, 'PASS_NO_EVIDENCE'); assert.equal(res.verdict, 'INVALID');
  res = judge(mutated('R180', (g) => { g.inputStats.distinctRotations = 10; }));
  assert.equal(byName(res, 'R180').status, 'PASS_NO_EVIDENCE'); assert.equal(res.verdict, 'INVALID');
  // the same degenerate scenario with a mismatch is FAIL
  res = judge(mutated('R180', (g) => { g.inputStats.fresh = 40; g['OUT-CW'].identical = false; g['OUT-CW'].mismatchedFrames = 1; }));
  assert.equal(res.verdict, 'FAIL');
  // portrait: one degenerate bundle in a record with a valid one is fine (v3 record rule) ...
  const ok = allGood(); ok.P2 = goodReport(SPECS[1], { limited: 0, k: 0 });
  res = judge(ok);
  assert.equal(byName(res, 'P2').status, 'PASS_NO_EVIDENCE'); assert.equal(res.verdict, 'PASS');
  // ... a record with no valid bundle is not
  const bad = allGood(); bad.P3 = goodReport(SPECS[2], { limited: 0, k: 0 }); bad.P4 = goodReport(SPECS[3], { limited: 0.5, k: 1 });
  res = judge(bad);
  assert.equal(res.verdict, 'INVALID'); assert.match(res.reasons.join(' '), /portrait records with PASS evidence 1 < registered 2/);
  assert.equal(res.counts.portraitValidRecords, 1);
});

test('J9: a report of another gate / version / datum / manifest, or without the arms, is INVALID', () => {
  const edit = [(r) => { r.meta.gate.id = 'or1-chunk-replay-gate-x'; }, (r) => { r.meta.gate.version = 3; }, (r) => { r.meta.gate.datum = 'R0'; }, (r) => { r.meta.kind = 'dev'; },
    (r) => { r.meta.arms = 'WC'; }, (r) => { r.meta.reps = 1; }, (r) => { r.meta.gate.manifest.beforeReviewSha256 = 'b'.repeat(64); }, (r) => { delete r.gates.inputStats; }, (r) => { delete r.rowsSha256; }];
  for (const [i, fn] of edit.entries()) {
    const res = judge(mutated('R90', (g, r) => fn(r)));
    assert.equal(byName(res, 'R90').status, 'INVALID', `edit ${i}`);
  }
});

test('J10: an internally inconsistent report (OUT-CW identical but different rows sha256 for W and C) is INVALID', () => {
  const res = judge(mutated('R90', (g, r) => { r.rowsSha256.C = ['x', 'x', 'x']; }));
  assert.equal(byName(res, 'R90').status, 'INVALID');
});

test('J11: the repetitions must be identical in the rows sha256 as well (a drifting arm is FAIL even when its REP flags say identical)', () => {
  const res = judge(mutated('R90', (g, r) => { r.rowsSha256.N = ['n', 'n', 'm']; }));
  assert.equal(byName(res, 'R90').status, 'FAIL');
});

test('J12: no floor registered for a scenario: INVALID (the judge does not invent one)', () => {
  const specs = SPECS.map((s) => (s.name === 'R90' ? { ...s, floors: { c4: null } } : s));
  const res = judgeGate(specs, allGood(), CTX);
  assert.equal(res.perScenario.find((p) => p.name === 'R90').status, 'INVALID');
});

test('J13: the command line: exit codes 0 PASS, 1 FAIL, 4 INVALID, 5 for a judge error (a crash is never a FAIL); the report path is <out-root>/<scenario>/report.json', () => {
  const manifestFile = path.join(scratch, 'manifest.json');
  const mk = (extra = {}) => ({ gate_id: 'or1-chunk-replay-gate', version: 4, registered_run: { scenarios: SPECS, k_min: CTX.kMin, ...extra }, review: {}, first_real_run_at: null });
  fs.writeFileSync(manifestFile, JSON.stringify(mk()));
  const { manifest_before_review_sha256: pin } = JSON.parse(spawnSync(process.execPath, ['--input-type=module', '-e', `import { designationPins } from ${JSON.stringify(pathToFileURL(path.join(repo, 'slam_refact/web-chunk/tools/or1/hold.mjs')).href)};
    console.log(JSON.stringify(designationPins(${JSON.stringify(manifestFile)})));`], { encoding: 'utf8' }).stdout);
  const writeReports = (dir, reports) => { for (const [n, r] of Object.entries(reports)) if (r) { fs.mkdirSync(path.join(dir, n), { recursive: true }); fs.writeFileSync(path.join(dir, n, 'report.json'), JSON.stringify(r)); } };
  const run = (dir) => spawnSync(process.execPath, [judgeFile, '--manifest', manifestFile, '--out-root', dir], { encoding: 'utf8' });
  const good = allGood();
  for (const r of Object.values(good)) r.meta.gate.manifest.beforeReviewSha256 = pin;
  const d1 = path.join(scratch, 'good'); writeReports(d1, good);
  assert.equal(run(d1).status, 0, run(d1).stdout);
  const failing = structuredClone(good); failing.R0.gates['OUT-CW'].identical = false; failing.R0.gates['OUT-CW'].mismatchedFrames = 1;
  const d2 = path.join(scratch, 'fail'); writeReports(d2, failing);
  assert.equal(run(d2).status, 1);
  const missing = structuredClone(good); missing.R0 = null;
  const d3 = path.join(scratch, 'missing'); writeReports(d3, missing);
  assert.equal(run(d3).status, 4);
  const broken = path.join(scratch, 'broken'); writeReports(broken, structuredClone(good)); fs.writeFileSync(path.join(broken, 'R0/report.json'), '{not json');
  assert.equal(run(broken).status, 4, 'an unreadable report is INVALID');
  // judge errors: no such manifest, a manifest without scenarios
  assert.equal(spawnSync(process.execPath, [judgeFile, '--manifest', path.join(scratch, 'nope.json'), '--out-root', d1], { encoding: 'utf8' }).status, 5);
  fs.writeFileSync(path.join(scratch, 'empty.json'), '{}');
  const noScen = spawnSync(process.execPath, [judgeFile, '--manifest', path.join(scratch, 'empty.json'), '--out-root', d1], { encoding: 'utf8' });
  assert.equal(noScen.status, 5); assert.match(noScen.stderr, /JUDGE ERROR/);
  assert.equal(runCli.length, 1);
});

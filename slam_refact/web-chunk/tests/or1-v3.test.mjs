// node --test slam_refact/web-chunk/tests/or1-v3.test.mjs
// Unit tests of the OR-1 v3 corrections (list C12 of the v2 pre-review and F-12 of the v2 first-run review; closure audit B): the raw inputs and answers are
// stored with their sha256 (F-1), the report carries C4 / C6 / C7 / C8 (V-4, V-15), the gate is aggregated with the record rule and the C9 asymmetry,
// the outputs are summed and read-only (F-10), the run log lives outside the output directory (F-9), runMeta names the registered gate file of the
// version it ran (V-8), and the registration tool and the hold agree (a manifest the tool wrote passes the hold).
import assert from 'node:assert/strict';
import fs from 'node:fs';
import os from 'node:os';
import path from 'node:path';
import test from 'node:test';
import { pathToFileURL } from 'node:url';

const toolsDir = process.env.SLAM_REFACT_OR1_TOOLS_DIR ?? path.join(import.meta.dirname, '../tools/or1');
const load = (...p) => import(pathToFileURL(path.join(toolsDir, ...p)).href);
const { canon } = await load('canon.mjs');
const { comparableFrame } = await load('players.mjs');
const { evaluate, aggregateGate, conditions, degeneracy, expectedCounts, C4_R_MIN, C4_K_MIN } = await load('run.mjs');
const { writeRunOutputs } = await load('outputs.mjs');
const { checkHold, claimFirstRun, designationPins, fileSetSha256, sha256File } = await load('hold.mjs');
const { buildManifest } = await load('make-gate-manifest.mjs');
const { derive, readBundle, inputFiles } = await load('bundle.mjs');
const freeze = await load('../gate-common/freeze.mjs');
const { RunLog, envRecord } = await load('../gate-common/runlog.mjs');
const { SR_HELD_FILE, gateArgs, sr } = await load('../gate-common/held.mjs');

const repo = path.resolve(import.meta.dirname, '../../..');
const base = path.join(repo, 'build/slam-refact-full-20261019/evidence/OR-1', `unit-tmp-v3-${process.pid}`);
fs.mkdirSync(base, { recursive: true });
let n = 0;
const fresh = (tag) => path.join(base, `${tag}-${++n}`);
test.after(() => { freeze.makeWritable(base); fs.rmSync(base, { recursive: true, force: true }); });

// ---- synthetic runs --------------------------------------------------------------------------------------------------------------
const row = (i, status = 'NORMAL', reason = 'TRACKING') => ({ index: i, fresh: true, pose: canon({ position: [i, 0, 0], status }),
  raw: { position: [i, 0, 0], rotationWxyz: [1, 0, 0, 0], status, reason, repeated: false, sourceFrameIndex: i, worldPoints: 3 } });
const frameRec = (i) => ({ index: i, timeNs: 1000 + i, videoTimeNs: 900 + i, size: 1024, rowsGiven: 660, atlasPrefixSha256: `${i}`.repeat(4), atlasPrefixBytes: 8,
  motion: [{ t: 10 + i, aig: [0, 9.8, 0], lin: null, rate: [0.1, 0.2, 0.3], interval: 16.6 }], orientation: { a: 1, b: 2, g: 3 }, orientationTimeNs: 10 + i });
const answerOf = (r) => ({ index: r.index, answer: canon({ position: { x: r.raw.position[0], y: r.raw.position[1], z: r.raw.position[2] },
  rotation: { x: 0, y: 0, z: 0, w: 1 }, trackingStatus: r.raw.status, trackingReason: r.raw.reason, worldPoints: r.raw.worldPoints }) });
// m processed frames after the bootstrap call; statuses follow `statusAt`
function runs({ m = 20, statusAt = (i) => (i % 5 < 2 ? 'LIMITED' : 'NORMAL'), reps = 2, mutateC } = {}) {
  const rows = Array.from({ length: m }, (_, i) => row(i, statusAt(i), 'TRACKING'));
  const frames = Array.from({ length: m + 1 }, (_, i) => frameRec(i - 1));
  const answers = [{ index: -1, answer: null }, ...rows.map(answerOf)];
  const mk = (arm) => Array.from({ length: reps }, () => ({ arm, layout: { rows: 650 }, glueConfig: canon({ pyramidModel: arm === 'W' ? 'swiftshader-approx' : 'ieee', captureWidth: 480 }),
    frames: frames.map((f) => ({ ...f })), rows: rows.map((r) => ({ ...r })), wallMs: 1,
    ...(arm === 'C' ? { answers: answers.map((a) => ({ ...a })), errors: [], events: [], calls: [], inexact: { clock: 3, video: 2, clockLost: 0, videoLost: 0 } } : {}) }));
  const r = { W: mk('W'), C: mk('C') };
  if (mutateC) mutateC(r.C);
  return r;
}
const bundleOf = (m) => ({ frames: [{ bootstrap: true, process: true }, ...Array.from({ length: m + 3 }, (_, i) => ({ bootstrap: false, process: i < m }))] });

test('OR1V3-1: C4 degeneracy from W rows: r over fresh, not repeated rows (LIMITED without INITIALIZING), k = status transitions; the registered floors are 0.10 and 2', () => {
  assert.equal(C4_R_MIN, 0.10); assert.equal(C4_K_MIN, 2);
  const mix = degeneracy(runs().W[0].rows);
  assert.deepEqual([mix.rows, mix.limited, mix.k, mix.degenerate], [20, 8, 7, false]);
  assert.ok(Math.abs(mix.r - 0.4) < 1e-12);
  assert.equal(degeneracy(runs({ statusAt: () => 'NORMAL' }).W[0].rows).degenerate, true, 'r = 0');
  assert.equal(degeneracy(runs({ statusAt: (i) => (i >= 10 ? 'LIMITED' : 'NORMAL') }).W[0].rows).degenerate, true, 'r = 0.5 but k = 1 (one transition)');
  const edge = degeneracy(runs({ statusAt: (i) => (i === 3 || i === 11 ? 'LIMITED' : 'NORMAL') }).W[0].rows);
  assert.deepEqual([edge.r, edge.k, edge.degenerate], [0.1, 4, false], 'r = 0.10 exactly is not below the floor');
  assert.equal(degeneracy(runs({ statusAt: (i) => (i === 3 ? 'LIMITED' : 'NORMAL') }).W[0].rows).degenerate, true, 'r = 0.05');
  // INITIALIZING rows do not count as LIMITED; repeated rows are not used
  const rows = runs().W[0].rows.map((r) => ({ ...r, raw: { ...r.raw, status: 'LIMITED', reason: 'INITIALIZING' } }));
  assert.equal(degeneracy(rows).limited, 0);
  const rep = runs().W[0].rows.map((r, i) => ({ ...r, raw: { ...r.raw, repeated: i % 2 === 0 } }));
  assert.equal(degeneracy(rep).rows, 10);
  assert.equal(degeneracy(undefined), null);
});

test('OR1V3-2: C6 / C7 / C8 are computed into the report and make the verdict (a vacuous ANS passes compareAnswers but not C6)', () => {
  const ok = evaluate({ runs: runs(), expected: expectedCounts(bundleOf(20)) });
  assert.deepEqual(expectedCounts(bundleOf(20)), { calls: 21, frames: 20 });
  assert.equal(ok.verdict['IN-CW'], true); assert.equal(ok.verdict['OUT-CW'], true); assert.equal(ok.verdict.ANS, true); assert.equal(ok.verdict.REP, true);
  assert.equal(ok.verdict.C7, true); assert.equal(ok.verdict.C8, true);
  assert.equal(ok.C6.pass, true); assert.equal(ok.ANS.nullAnswers, 1); assert.equal(ok.ANS.compared, 20);
  // V-4 (reviewer mutant R1): every answer null after the first. compareAnswers says identical, C6 does not
  const vacuous = evaluate({ runs: runs({ mutateC: (C) => C.forEach((c) => { c.answers = c.answers.map((a) => ({ index: a.index, answer: null })); }) }), expected: expectedCounts(bundleOf(20)) });
  assert.equal(vacuous.ANS.identical, true, 'the comparator alone is blind to this');
  assert.equal(vacuous.verdict.ANS, false); assert.match(vacuous.C6.reasons.join(' '), /nullAnswers 21/);
  // fewer answers than output frames (the chunk stopped answering): every answer given matches, nullAnswers + compared == answers, only compared != OUT-CW.frames shows it
  const short = evaluate({ runs: runs({ mutateC: (C) => C.forEach((c) => { c.answers = c.answers.slice(0, -3); }) }), expected: expectedCounts(bundleOf(20)) });
  assert.equal(short.ANS.identical, true); assert.equal(short.ANS.compared, 17); assert.equal(short.verdict.ANS, false);
  assert.match(short.C6.reasons.join(' '), /compared 17 != OUT-CW\.frames 20/);
  // a chunk error reported through the error callback fails C6 although every answer matches
  const errored = evaluate({ runs: runs({ mutateC: (C) => C.forEach((c) => { c.errors = [new Error('boom')]; }) }), expected: expectedCounts(bundleOf(20)) });
  assert.equal(errored.verdict.ANS, false); assert.match(errored.C6.reasons.join(' '), /chunk error/);
  // V-15: a clock value the fake clock could not represent
  const lost = evaluate({ runs: runs({ mutateC: (C) => C.forEach((c) => { c.inexact = { clock: 3, video: 2, clockLost: 1, videoLost: 0 }; }) }), expected: expectedCounts(bundleOf(20)) });
  assert.equal(lost.verdict.C7, false); assert.equal(lost.C7.clockLost, 1);
  // C8: counts come from the bundle, not from the arms
  assert.equal(evaluate({ runs: runs(), expected: { calls: 22, frames: 20 } }).verdict.C8, false);
  assert.equal(evaluate({ runs: runs(), expected: { calls: 21, frames: 19 } }).verdict.C8, false);
  assert.equal(evaluate({ runs: runs() }).verdict.C8, undefined, 'without the bundle C8 is not evaluated');
  // the ctx form used by runAll() results
  assert.equal(evaluate({ runs: runs(), ctx: { bundle: bundleOf(20) } }).verdict.C8, true);
  // V-11: the glue configs are recorded and their difference reported (REPORT), not compared by IN-CW
  assert.deepEqual(ok.glueConfig.diffPaths, ['pyramidModel']);
  assert.equal(ok.verdict['IN-CW'], true);
  assert.equal(conditions({ ANS: null, 'OUT-CW': null, chunkErrors: [] }, null).C6, null);
});

test('OR1V3-3: the gate over its bundles - C9 asymmetry (a degenerate bundle still fails the gate), the record rule of C4 and the verdict names', () => {
  const g = (over = {}, status = (i) => (i % 5 < 2 ? 'LIMITED' : 'NORMAL')) => ({ ...evaluate({ runs: runs({ statusAt: status }), expected: expectedCounts(bundleOf(20)) }), ...over });
  const good = (name, record, status) => ({ name, record, g: g({}, status) });
  const degenerate = () => () => 'NORMAL';
  const four = (extra) => [good('G1', 'A'), good('G2', 'A'), good('G3', 'B'), good('G4', 'B')].map((b, i) => (extra && extra[i] ? extra[i] : b));
  assert.equal(aggregateGate(four()).verdict, 'PASS');
  // a degenerate bundle only removes PASS evidence: the record stays valid through its sibling
  assert.equal(aggregateGate(four([good('G1', 'A', degenerate())])).verdict, 'PASS');
  // both bundles of a record degenerate: one valid record only
  const one = aggregateGate(four([good('G1', 'A', degenerate()), good('G2', 'A', degenerate())]));
  assert.equal(one.verdict, 'INSUFFICIENT'); assert.equal(one.validRecords, 1);
  // C9: a mismatch in a degenerate bundle fails the gate; so does one in an ordinary bundle
  const bad = good('G1', 'A', degenerate());
  bad.g.verdict = { ...bad.g.verdict, 'OUT-CW': false };
  const f = aggregateGate(four([bad]));
  assert.equal(f.verdict, 'FAIL'); assert.deepEqual(f.fails, [{ bundle: 'G1', failed: ['OUT-CW'] }]);
  for (const key of ['IN-CW', 'ANS', 'REP', 'C7', 'C8']) {
    const b = good('G3', 'B'); b.g.verdict = { ...b.g.verdict, [key]: false };
    assert.equal(aggregateGate(four([undefined, undefined, b])).verdict, 'FAIL', key);
  }
  // NW and CN are REPORT: they do not fail the gate
  const nw = good('G4', 'B'); nw.g.verdict = { ...nw.g.verdict, NW: false, CN: false };
  assert.equal(aggregateGate(four([undefined, undefined, undefined, nw])).verdict, 'PASS');
  // REP missing (one repetition) or C8 not evaluated: the gate is not evaluated, no PASS
  const norep = good('G2', 'A'); norep.g.verdict = { ...norep.g.verdict, REP: undefined };
  assert.equal(aggregateGate(four([undefined, norep])).verdict, 'UNEVALUATED');
});

test('OR1V3-4: the raw inputs and answers are written, summed in the report, and the rows sha256 is the in-memory one (F-1, F-8)', () => {
  const out = fresh('out'); fs.mkdirSync(out);
  const r = runs();
  const g = evaluate({ runs: r, expected: expectedCounts(bundleOf(20)) });
  const report = writeRunOutputs({ outDir: out, runs: r, g, meta: { schema: 'test' } });
  assert.deepEqual(Object.keys(report.files.inputs).sort(), ['inputs-C-0.json', 'inputs-C-1.json', 'inputs-W-0.json', 'inputs-W-1.json']);
  assert.deepEqual(Object.keys(report.files.answers).sort(), ['answers-C-0.json', 'answers-C-1.json']);
  assert.deepEqual(Object.keys(report.files.rows).sort(), ['rows-C-0.json', 'rows-C-1.json', 'rows-W-0.json', 'rows-W-1.json']);
  for (const group of Object.values(report.files)) for (const [name, sha] of Object.entries(group)) assert.equal(sha256File(path.join(out, name)), sha, name);
  // the stored canonical forms are the ones IN-CW compares: recomputing IN-CW from the files gives the same verdict
  const inputsC = JSON.parse(fs.readFileSync(path.join(out, 'inputs-C-0.json'), 'utf8')), inputsW = JSON.parse(fs.readFileSync(path.join(out, 'inputs-W-0.json'), 'utf8'));
  assert.equal(inputsC.calls.length, 21);
  assert.deepEqual(inputsC.calls.map((c) => c.canonical), inputsW.calls.map((c) => c.canonical));
  assert.deepEqual(inputsC.calls[3].canonical, comparableFrame(r.C[0].frames[3]));
  assert.equal(inputsC.calls[3].raw.timeNs, 1002, 'the raw record is kept next to the canonical form');
  const answers = JSON.parse(fs.readFileSync(path.join(out, 'answers-C-0.json'), 'utf8'));
  assert.equal(answers.answers.length, 21); assert.equal(answers.answers[0].answer, null);
  assert.equal(report.rowsSha256.W.length, 2);
  assert.notEqual(report.rowsSha256.W[0], report.files.rows['rows-W-0.json'], 'rowsSha256 is the sha256 of JSON.stringify(rows), not of the file');
  assert.match(report.rowsSha256Definition, /in memory/);
  // a tampered input record is visible: the stored sha256 no longer matches
  const f = path.join(out, 'inputs-C-0.json');
  fs.appendFileSync(f, ' ');
  assert.notEqual(sha256File(f), report.files.inputs['inputs-C-0.json']);
  assert.throws(() => writeRunOutputs({ outDir: out, runs: r, g, meta: {} }), { code: 'EEXIST' }, 'nothing is overwritten');
  // the report itself is written exclusively too: a report.json that is already there is not replaced
  const out2 = fresh('out'); fs.mkdirSync(out2);
  fs.writeFileSync(path.join(out2, 'report.json'), 'earlier report');
  assert.throws(() => writeRunOutputs({ outDir: out2, runs: r, g, meta: {} }), { code: 'EEXIST' });
  assert.equal(fs.readFileSync(path.join(out2, 'report.json'), 'utf8'), 'earlier report');
});

test('OR1V3-5: SHA256SUMS covers every file, then the tree is read-only; registered copies keep their sha256 (F-10)', () => {
  const out = fresh('frozen'); fs.mkdirSync(path.join(out, 'sub'), { recursive: true });
  fs.writeFileSync(path.join(out, 'a.json'), 'a'); fs.writeFileSync(path.join(out, 'sub/b.json'), 'b');
  const src = fresh('reg'); fs.mkdirSync(src);
  const gateMd = path.join(src, 'g.md'); fs.writeFileSync(gateMd, 'gate');
  const copies = freeze.copyInto(out, 'registered', [gateMd]);
  assert.equal(copies[0].sha256, sha256File(gateMd));
  assert.throws(() => freeze.copyInto(out, 'registered', [gateMd]), { code: 'EEXIST' });
  const sums = freeze.writeSums(out);
  assert.equal(sums.files, 3);
  const text = fs.readFileSync(path.join(out, 'SHA256SUMS'), 'utf8').trim().split('\n');
  assert.deepEqual(text.map((l) => l.split('  ')[1]), ['a.json', 'registered/g.md', 'sub/b.json']);
  for (const l of text) { const [sha, name] = l.split('  '); assert.equal(sha256File(path.join(out, name)), sha, name); }
  assert.throws(() => freeze.writeSums(out), { code: 'EEXIST' }, 'SUMS are written once');
  const changed = freeze.makeReadOnly(out);
  assert.ok(changed >= 6);
  for (const f of ['a.json', 'sub/b.json', 'registered/g.md', 'SHA256SUMS']) assert.equal(fs.statSync(path.join(out, f)).mode & 0o222, 0, f);
  assert.equal(fs.statSync(path.join(out, 'sub')).mode & 0o222, 0, 'directories too');
  assert.throws(() => fs.writeFileSync(path.join(out, 'new.json'), 'x'), (e) => e.code === 'EACCES' || e.code === 'EPERM');
  assert.deepEqual(freeze.readOnlyFiles([gateMd]).map((x) => x.ok), [true]);
  assert.equal(fs.statSync(gateMd).mode & 0o222, 0);
  assert.equal(freeze.readOnlyFiles([path.join(src, 'missing')])[0].ok, false);
  freeze.makeWritable(out);
});

test('OR1V3-6: the run log is created exclusively outside the output directory, records names (never values) of steering variables, and is read-only after close (F-9)', () => {
  const out = fresh('o'); const file = fresh('run') + '.log';
  assert.throws(() => new RunLog(path.join(out, 'inside.log'), { out }), /outside the output directory/);
  const log = new RunLog(file, { out });
  assert.throws(() => new RunLog(file, { out }), { code: 'EEXIST' }, 'a log is never reused');
  const env = { SLAM_REFACT_OR1_X_SHA256: 'secret-value', HOME: '/home/x', SLAM_REFACT_EMPTY: '' };
  const rec = envRecord(env, ['HOME']);
  assert.deepEqual(Object.keys(rec).sort(), ['HOME', 'SLAM_REFACT_EMPTY', 'SLAM_REFACT_OR1_X_SHA256']);
  assert.equal(rec.SLAM_REFACT_OR1_X_SHA256.set, true); assert.equal(rec.SLAM_REFACT_EMPTY.set, false);
  assert.ok(!JSON.stringify(rec).includes('secret-value'), 'only the sha256 of a value is recorded');
  log.write('start', { env: rec, argv: ['run'] });
  log.write('hold-checks', { lines: ['[SR-HELD] pin node sha256 actual=x registered=x'] });
  log.close('OK');
  const lines = fs.readFileSync(file, 'utf8').trim().split('\n').map((l) => JSON.parse(l));
  assert.deepEqual(lines.map((l) => l.event), ['start', 'hold-checks', 'end']);
  assert.equal(lines.at(-1).status, 'OK');
  assert.equal(fs.statSync(file).mode & 0o222, 0);
  assert.throws(() => log.write('late'), /closed/);
});

test('OR1V3-7: the gate arguments are required, validated, and nothing is defaulted (v2 pinned the id, version, directory and the variable name in the code)', () => {
  const ok = { 'prereg-dir': 'pre', 'gate-id': 'or1-chunk-replay-gate', 'gate-version': '3', 'designation-env': 'SLAM_REFACT_OR1_V3_DESIGNATION_SHA256' };
  const g = gateArgs(ok, '/r');
  assert.deepEqual([g.gateId, g.version, g.preregDir, g.designationEnv], ['or1-chunk-replay-gate', 3, '/r/pre', 'SLAM_REFACT_OR1_V3_DESIGNATION_SHA256']);
  for (const k of Object.keys(ok)) assert.throws(() => gateArgs({ ...ok, [k]: undefined }, '/r'), new RegExp(`--${k}`));
  assert.throws(() => gateArgs({ ...ok, 'gate-version': '3.5' }, '/r'), /positive integer/);
  assert.throws(() => gateArgs({ ...ok, 'gate-version': 'v3' }, '/r'), /positive integer/);
  assert.throws(() => gateArgs({ ...ok, 'designation-env': 'abc123def' }, '/r'), /environment variable NAME/);
  assert.throws(() => gateArgs({ ...ok, 'designation-env': 'a'.repeat(3) }, '/r'), /environment variable NAME/);
});

test('OR1V3-8: runMeta names the registered gate file of the version it ran, the manifest / designation / review, the device and the harness (V-8, C12 item 3)', async (t) => {
  const real = {
    engine: path.join(repo, 'build/slam-refact-or1-20261020/wasm/wasm'), native: 'build/slam-refact-or1-20261020/native/harness/slam_refact_replay',
  };
  if (!fs.existsSync(path.join(repo, real.native)) || !fs.existsSync(path.join(real.engine, 'slam_refact_engine.js'))) { t.skip('the OR-1 pair is not in this tree'); return; }
  const { runMeta } = await import(pathToFileURL(path.join(import.meta.dirname, '../tools/or1/or1.mjs')).href);
  const prereg = fresh('prereg'); fs.mkdirSync(prereg);
  const gate = { gateId: 'or1-meta-gate', version: 7, preregDir: prereg, designationEnv: 'SLAM_REFACT_OR1_META_SHA256' };
  fs.writeFileSync(path.join(prereg, 'or1-meta-gate.v7.md'), '# v7\n');
  const ctx = { arms: ['W'], reps: 1, bundleDir: path.join(repo, 'x'), nativeBundleDir: null, engineDir: real.engine, nativeBin: real.native, chunkDir: path.join(repo, 'slam_refact/web-chunk') };
  const meta = runMeta({ ctx, values: { kind: 'dev' }, derivedMeta: {}, atlasSetSha256: 'z', gate });
  assert.equal(meta.gate.id, 'or1-meta-gate'); assert.equal(meta.gate.version, 7);
  assert.equal(meta.gate.gateMd.sha256, sha256File(path.join(prereg, 'or1-meta-gate.v7.md')));
  assert.equal(meta.gateMdSha256, meta.gate.gateMd.sha256, 'not the sha256 of a v1 file');
  assert.equal(meta.gate.designationEnvName, 'SLAM_REFACT_OR1_META_SHA256');
  assert.equal(meta.gate.srHeld.sha256, sha256File(SR_HELD_FILE));
  assert.ok(meta.harness['slam_refact/web-chunk/tools/or1/hold.mjs'] && meta.harness['slam_refact/web-chunk/tools/or1/outputs.mjs']);
  assert.ok(meta.gateCommon['slam_refact/web-chunk/tools/gate-common/held.mjs']);
  assert.equal(meta.schema, 'or1-run-meta-v2');
  // no gate md in the directory: the field is null, not another version's sha
  const none = runMeta({ ctx, values: { kind: 'dev' }, derivedMeta: {}, atlasSetSha256: 'z', gate: { ...gate, version: 8 } });
  assert.equal(none.gateMdSha256, null);
});

// ---- the registration tool and the hold agree --------------------------------------------------------------------------------------
// a synthetic repository root with everything make-gate-manifest reads (harness, gate-common, chunk, tests, spec, pair, development sources, evidence summaries)
function registrationTree({ nullFailures = 0, passes = true, survivors = [] } = {}) {
  const root = fresh('reg-root'); fs.mkdirSync(root, { recursive: true });
  const put = (rel, text) => { const f = path.join(root, rel); fs.mkdirSync(path.dirname(f), { recursive: true }); fs.writeFileSync(f, text); return f; };
  put('slam_refact/web-chunk/tools/or1/run.mjs', 'run'); put('slam_refact/web-chunk/tools/or1/build-pair.sh', 'sh');
  put('slam_refact/web-chunk/tools/gate-common/freeze.mjs', 'freeze'); put('slam_refact/web-chunk/xr-slam.js', 'chunk');
  put('slam_refact/web-chunk/tests/t1.test.mjs', 't1'); put('slam_refact/docs/spec/07-xr-slam-chunk.md', 'spec');
  put('pairdir/pair.json', JSON.stringify({ snapshot_sources: 'pairdir/snapshot-sources.sha256', wasm: { file: 'pairdir/wasm/slam_refact_engine.wasm' }, native_replay: { file: 'pairdir/native/replay' } }));
  put('pairdir/snapshot-sources.sha256', 'sums'); put('pairdir/native/replay', 'bin');
  for (const f of ['slam_refact_engine.js', 'slam_refact_engine.wasm', 'manifest.json']) put(`pairdir/wasm/${f}`, `engine ${f}`);
  put('atlas/a0.gz', 'atlas0'); put('atlas/a1.gz', 'atlas1');
  const srcBundle = (name, offset) => {
    put(`srcs/${name}/engine-config.tsv`, ['capture_width\t480', 'id_generator_seed\t5'].map((l) => `${l}\n`).join(''));
    const sensor = (kind, t) => [kind, t, 0, 9.8, 0, '-', '-', '-', 1, 2, 3, 16.6, 10, 20, 30].join('\t');
    put(`srcs/${name}/sensors.tsv`, [sensor('bootstrap', 90 + offset), sensor('run', 100 + offset)].map((l) => `${l}\n`).join(''));
    const frame = (kind, i, t, pyr) => [kind, i, t, t, t / 1e9, 1, `/img/${i}.pgm`, pyr, t - 10].join('\t');
    put(`srcs/${name}/frames.tsv`, [frame('bootstrap', -1, 90 + offset, 'atlas/a0.gz'), frame('frame', 0, 200 + offset, 'atlas/a1.gz')].map((l) => `${l}\n`).join(''));
  };
  srcBundle('G1', 0); srcBundle('G2', 1);
  const prereg = path.join(root, 'prereg');
  const gate = { gateId: 'or1-reg-gate', version: 3, preregDir: prereg, designationEnv: 'SLAM_REFACT_OR1_REG_V3_SHA256' };
  put('prereg/or1-reg-gate.v3.md', '# or1 reg gate v3\n');
  put('prereg/or1-reg-gate.v2.md', '# previous\n'); put('prereg/or1-reg-gate.v2.manifest.json', '{"first_real_run_at":"2026-10-20T04:41:35.461Z"}');
  put('evidence/selftest/selftest.json', JSON.stringify({ datasets: { D1: {} }, summary: { nullFailures, passes, chunkMutantsDetectedByAChunkGate: 3, chunkMutantsRun: 3, equivalentMutantFalseAlarms: 0, rowMutationsDetected: 1, rowMutations: 1 } }));
  put('evidence/tool/summary.json', JSON.stringify({ survivors, detected: 5, mutants: 5 }));
  const spec = [{ name: 'G1-test', src: 'srcs/G1', record: 'R1' }, { name: 'G2-test', src: 'srcs/G2', record: 'R2' }];
  const args = { root, gate, gateDataSpec: spec, pairFile: 'pairdir/pair.json', derivedOut: 'build/derived', selftestDir: 'evidence/selftest', toolMutantsDir: 'evidence/tool',
    supersedesVersion: 2, classification: 'test', tests: ['t1.test.mjs'], now: '2026-10-20T10:00:00+09:00', gitHead: 'abc' };
  return { root, put, gate, prereg, args };
}

test('OR1V3-9: a manifest written by make-gate-manifest passes the hold (registration tool and device agree), and the tool refuses to overwrite and to pin two files with one name', () => {
  const { root, put, gate, prereg, args } = registrationTree();
  const { manifest, manifestFile } = buildManifest(args);
  assert.deepEqual(manifest.held.kinds, ['G1-test', 'G2-test']);
  assert.equal(manifest.file, 'or1-reg-gate.v3.md'); assert.equal(manifest.gate_id, 'or1-reg-gate'); assert.equal(manifest.version, 3);
  assert.deepEqual(manifest.review, {}); assert.equal(manifest.first_real_run_at, null);
  assert.equal(manifest.supersedes.first_real_run_at_in_previous, '2026-10-20T04:41:35.461Z');
  // C12 item 6: the bundles were run by the previous version, so the manifest and (through runMeta) the report say that their arm outputs are observed data
  assert.equal(manifest.data_reuse.observed_in_previous_version, true);
  assert.equal(manifest.data_reuse.previous_first_real_run_at, '2026-10-20T04:41:35.461Z');
  assert.match(manifest.data_reuse.statement, /G1-test, G2-test\) were run by or1-reg-gate v2; every arm output on them has been observed/);
  assert.match(manifest.not_seen, /data_reuse/);
  assert.equal(manifest.mutation.detected, true); assert.equal(manifest.null_run.result, 'PASS');
  assert.equal(manifest.pinned.gate_data[0].record, 'R1');
  assert.equal(manifest.held.inputs['G1-test'], sr.treeSha(path.join(root, manifest.pinned.gate_data[0].derived_dir)).sha);
  assert.ok('sr_held.mjs' in manifest.held.pins && 'node' in manifest.held.pins && 'run.mjs' in manifest.held.pins && 'xr-slam.js' in manifest.held.pins);
  // the files of the snapshot sum correctly from inside the snapshot (v2: one line pointed at the preregistration path, V-12)
  const snap = path.join(prereg, 'or1-reg-gate.v3.snapshot');
  for (const l of fs.readFileSync(path.join(snap, 'SHA256SUMS'), 'utf8').trim().split('\n')) { const [sha, name] = l.split('  '); assert.equal(sha256File(path.join(snap, name)), sha, name); }
  assert.throws(() => buildManifest(args), /refusing to overwrite/);
  // the coordinator's act: review file, designation file, env
  const review = put('prereg/rev.md', 'review\n');
  const pins = designationPins(manifestFile);
  const designation = put('prereg/des.json', JSON.stringify({ gate: 'or1-reg-gate.v3', first_real_run_permitted: true, pinned: { ...pins, review_file_sha256: sha256File(review) } }));
  const m = JSON.parse(fs.readFileSync(manifestFile, 'utf8'));
  m.review = { reviewer: 'r', at: '2026-10-20T11:00:00+09:00', file: 'rev.md', sha256: sha256File(review), designation_file: 'des.json', designation_file_sha256: sha256File(designation) };
  fs.writeFileSync(manifestFile, JSON.stringify(m, null, 1));
  const env = { [gate.designationEnv]: sha256File(designation) };
  const datum = m.pinned.gate_data[1];
  const derivedBundle = path.join(root, datum.derived_dir);
  const derivedMeta = JSON.parse(fs.readFileSync(path.join(derivedBundle, 'or1-derived.json'), 'utf8'));
  const atlasSetSha256 = fileSetSha256(root, inputFiles(readBundle(derivedBundle), root).map((f) => path.relative(root, f)));
  const values = { kind: 'gate', engineDir: 'pairdir/wasm', nativeBin: 'pairdir/native/replay', arms: 'WCN', reps: 3, atlasSetSha256, bundle: datum.derived_dir };
  const hold = checkHold({ root, gate, values, derivedMeta, env, now: new Date('2026-10-20T12:00:00+09:00') });
  assert.equal(hold.datum.name, 'G2-test'); assert.equal(hold.datum.record, 'R2');
  assert.ok(claimFirstRun({ hold }).endsWith('or1-reg-gate.v3.first-run.G2-test.marker'));
  // two pinned files with one base name cannot be registered
  put('slam_refact/web-chunk/tools/gate-common/run.mjs', 'same name as a harness file');
  fs.rmSync(manifestFile); fs.rmSync(snap, { recursive: true });
  assert.throws(() => buildManifest({ ...args, derivedOut: 'build/derived2' }), /share the base name run\.mjs/);
});

test('OR1V3-10: derive output is a tree sr_held can register (regular files only) and the tree sha256 changes with any byte of frames.tsv', () => {
  const src = fresh('src'); fs.mkdirSync(src);
  fs.writeFileSync(path.join(src, 'engine-config.tsv'), 'capture_width\t480\n');
  fs.writeFileSync(path.join(src, 'sensors.tsv'), ['bootstrap', 90, 0, 9.8, 0, '-', '-', '-', 1, 2, 3, 16.6, 10, 20, 30].join('\t') + '\n');
  fs.writeFileSync(path.join(src, 'frames.tsv'), ['bootstrap', -1, 90, 90, 9e-8, 1, '/i.pgm', 'a.gz', 80].join('\t') + '\n');
  const d = derive(src, fresh('d'), { sourceLabel: 's' });
  const a = sr.treeSha(d.outDir).sha;
  assert.match(a, /^[0-9a-f]{64}$/);
  fs.appendFileSync(path.join(d.outDir, 'frames.tsv'), ' ');
  assert.notEqual(sr.treeSha(d.outDir).sha, a);
  assert.equal(os.platform(), 'linux');
});

test('OR1V3-11: the manifest says mutation.detected only when the self-test passed and no tool mutant survived, and null_run PASS only without null failures (checked before the first run)', () => {
  const m = (opts) => buildManifest(registrationTree(opts).args).manifest;
  const good = m();
  assert.equal(good.mutation.detected, true); assert.equal(good.null_run.result, 'PASS');
  assert.equal(m({ survivors: ['H9-derived-path-not-compared'] }).mutation.detected, false, 'a surviving tool mutant');
  assert.equal(m({ passes: false }).mutation.detected, false, 'the self-test did not pass');
  const nf = m({ nullFailures: 2 });
  assert.equal(nf.null_run.result, 'FAIL'); assert.equal(nf.null_run.detail.nullFailures, 2);
  // no previous run on record: nothing was observed before
  const t = registrationTree(); t.put('prereg/or1-reg-gate.v2.manifest.json', '{"first_real_run_at":null}');
  assert.equal(buildManifest(t.args).manifest.data_reuse.observed_in_previous_version, false);
});

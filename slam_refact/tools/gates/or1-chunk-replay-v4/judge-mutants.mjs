// Mutation check of the v4 judge (protocol section 4: a gate tool that does not react to a defect is not evidence). Every mutant is a modified COPY of judge.mjs under
// build/ (in the repository's layout, with a link to web-chunk so that its relative imports resolve); the repo file is never touched. judge.test.mjs must pass on the
// unmodified copy (null-run) and fail on every mutant.
//   node judge-mutants.mjs --out build/<lane>/or1-v4-judge-mutants
import fs from 'node:fs';
import path from 'node:path';
import { spawnSync } from 'node:child_process';
import { parseArgs } from 'node:util';

const repo = path.resolve(import.meta.dirname, '../../../..');
const { values } = parseArgs({ options: { out: { type: 'string' }, only: { type: 'string' } } });
const out = path.resolve(repo, values.out ?? '');
if (!values.out || !out.startsWith(path.join(repo, 'build') + path.sep)) throw new Error('--out must be a new directory under build/');
if (fs.existsSync(out)) throw new Error(`refusing to overwrite ${out}`);
const src = fs.readFileSync(path.join(import.meta.dirname, 'judge.mjs'), 'utf8');
const test = path.join(import.meta.dirname, 'judge.test.mjs');

// [id, from, to]: `from` occurs exactly once in judge.mjs
export const MUTANTS = [
  ['JM01-identical-truthy', "checks['IN-CW'] = inCw.identical === true &&", "checks['IN-CW'] = inCw.identical &&"],
  ['JM02-invalid-is-a-pass', "verdict = 'INVALID'; reasons.push(`${invalids.length} of", "verdict = 'PASS'; reasons.push(`${invalids.length} of"],
  ['JM03-invalid-exit-is-fail', 'INVALID: 4, JUDGE_ERROR: 5', 'INVALID: 1, JUDGE_ERROR: 5'],
  ['JM04-judge-crash-is-fail', 'process.exitCode = EXIT.JUDGE_ERROR;', 'process.exitCode = EXIT.FAIL;'],
  ['JM05-missing-report-is-fail', "return out('MISSING');", "return out('FAIL');"],
  ['JM06-degenerate-input-removes-a-fail', "if (failed.length) return out('FAIL');", ''],
  ['JM07-calls-not-registered', 'inCw.countA === exp.calls && inCw.countB === exp.calls;', 'true;'],
  ['JM08-direct-arm-count-not-checked', 'if (!checks.wCalls) invalid.push', 'if (false) invalid.push'],
  ['JM09-session-not-graded', 'if (!checks.SES) failed.push', 'if (false) failed.push'],
  ['JM10-no-evidence-counts-as-valid-record', "if (p.status === 'PASS') r.valid = true;", "if (p.status === 'PASS' || p.status === 'PASS_NO_EVIDENCE') r.valid = true;"],
  ['JM11-rotation-evidence-not-required', 'const kRot = need.rotationScenarios ?? rotation.length,', 'const kRot = 0,'],
  ['JM12-manifest-binding-dropped', 'if (ctx.beforeReviewSha256 && meta.gate', 'if (false && meta.gate'],
  ['JM13-consistency-not-checked', "if (!checks.consistent) invalid.push", 'if (false) invalid.push'],
  ['JM14-rows-sha-over-repetitions-ignored', 'new Set(report.rowsSha256[arm]).size === 1;', 'true;'],
  ['JM15-clock-loss-not-graded', "if (!checks.C7) failed.push", 'if (false) failed.push'],
  ['JM16-arm-n-repetition-ignored', "repOk('W') && repOk('C') && repOk('N') && shaOk('W')", "repOk('W') && repOk('C') && shaOk('W')"],
  ['JM17-arm-w-repetition-ignored', "checks.REP = repOk('W') && repOk('C')", "checks.REP = repOk('C')"],
  ['JM18-chunk-errors-ignored', 'Array.isArray(g.chunkErrors) && g.chunkErrors.length === 0;', 'true;'],
  ['JM19-fresh-floor-zero', 'freshNonRepeated >= fl.min_fresh_nonrepeated;', 'freshNonRepeated >= 0;'],
  ['JM20-rotation-floor-ignored', 'st.distinctRotations >= fl.min_distinct_rotations;', 'true;'],
  ['JM21-c4-ignored', 'if (fl.c4) {', 'if (false) {'],
  ['JM22-portrait-records-not-required', 'kRec = need.portraitRecords ?? portraitRecords.length;', 'kRec = 0;'],
  ['JM23-missing-floor-invented', "if (!isNum(fl.min_fresh_nonrepeated)) { invalid.push('the registration has no floor min_fresh_nonrepeated for this scenario'); return out('INVALID'); }", ''],
  ['JM24-out-cw-frames-not-registered', 'outCw.frames === exp.frames && outCw.identicalFrames === exp.frames;', 'true;'],
  ['JM25-ans-counts-not-registered', 'ans.compared === exp.frames && ans.answers === exp.calls &&', ''],
  ['JM26-gate-version-not-checked', 'meta.gate?.version !== gateVersion', 'false'],
  ['JM30-gate-id-not-checked', 'meta.gate?.id !== gateId', 'false'],
  ['JM27-datum-not-checked', 'meta.gate?.datum !== spec.name', 'false'],
  ['JM28-arms-not-checked', "meta.arms !== 'WCN'", 'false'],
  ['JM29-direct-arm-glue-not-checked', 'if (JSON.stringify(gw[k]) !== JSON.stringify(want[k])) invalid.push', 'if (false) invalid.push'],
];

fs.mkdirSync(out, { recursive: true });
const run = (dir, file) => spawnSync(process.execPath, ['--test', test], { cwd: repo, encoding: 'utf8', env: { ...process.env, SLAM_REFACT_OR1V4_JUDGE: file } });
function place(id, text) {
  const base = path.join(out, id);
  const dir = path.join(base, 'slam_refact/tools/gates/or1-chunk-replay-v4');
  fs.mkdirSync(dir, { recursive: true });
  fs.symlinkSync(path.join(repo, 'slam_refact/web-chunk'), path.join(base, 'slam_refact/web-chunk'));
  fs.writeFileSync(path.join(dir, 'judge.mjs'), text);
  return path.join(dir, 'judge.mjs');
}
const only = values.only ? new Set(values.only.split(',')) : null;
const nullRun = run(out, place('NULL', src));
const results = [];
for (const [id, from, to] of MUTANTS) {
  if (only && !only.has(id)) continue;
  const n = src.split(from).length - 1;
  if (n !== 1) { results.push({ id, error: `pattern occurs ${n} times` }); continue; }
  const r = run(out, place(id, src.replace(from, () => to)));
  const failed = (r.stdout.match(/^not ok /gm) ?? []).length;
  results.push({ id, detected: r.status !== 0, failedTests: failed });
}
const summary = { nullRunPassed: nullRun.status === 0, mutants: results.length, detected: results.filter((r) => r.detected).length, survivors: results.filter((r) => !r.detected).map((r) => r.id), results };
fs.writeFileSync(path.join(out, 'summary.json'), `${JSON.stringify(summary, null, 1)}\n`);
console.log(JSON.stringify({ ...summary, results: undefined }));
process.exitCode = summary.nullRunPassed && summary.survivors.length === 0 ? 0 : 1;

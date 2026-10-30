// Mutation check of the US-010 v4 harness code (tests/us010-v4.mjs, the v4 expectations of sample-smoke-criteria.mjs) against tests/us010-v4.test.mjs: every mutant is a
// modified COPY of the three source directories under build/ (the repo files are never touched); the test must fail on every mutant.
//   node slam_refact/web-chunk/tests/run-us010-v4-mutants.mjs --out build/<lane>/us010-v4-mutants
import fs from 'node:fs';
import path from 'node:path';
import { spawnSync } from 'node:child_process';
import { parseArgs } from 'node:util';

const repo = path.resolve(import.meta.dirname, '../../..');
const { values } = parseArgs({ options: { out: { type: 'string' } } });
const out = path.resolve(repo, values.out ?? '');
if (!values.out || !out.startsWith(path.join(repo, 'build') + path.sep)) throw new Error('--out must be a path under build/');
if (fs.existsSync(out)) throw new Error(`refusing to overwrite ${out}`);
const src = path.join(repo, 'slam_refact/web-chunk');
const V4 = 'slam_refact/web-chunk/tests/us010-v4.mjs', CRIT = 'slam_refact/web-chunk/tests/sample-smoke-criteria.mjs';
// [id, file, from, to]: `from` must occur exactly once
const M = [
  ['U1-pin-list-overwritable', V4, "flag: 'wx', mode: 0o444", "flag: 'w', mode: 0o644"],
  ['U2-mutation-detected-loosened', V4, 'm.mutation?.detected === true', 'm.mutation?.detected !== undefined'],
  ['U3-existing-list-not-refused', V4, 'if (fs.existsSync(out)) throw', 'if (false) throw'],
  ['U4-added-files-not-reported', V4, 'const added = coreFiles(repo, roots).filter((f) => !(f in rec.files));', 'const added = [];'],
  ['U5-driver-not-required', V4, 'if (!files.includes(driver)) throw', 'if (false) throw'],
  ['U6-null-run-not-checked', V4, "ok: m.null_run?.result === 'PASS'", 'ok: true'],
  ['U7-external-not-compared', V4, 'if (!(k in now) || now[k].sha256 !== e.sha256 || now[k].file !== e.file)', 'if (false)'],
  ['U8-first-run-on-record-allowed', V4, "ok: m.first_real_run_at === null || m.first_real_run_at === undefined", 'ok: true'],
  ['U9-existing-snapshot-not-refused', V4, 'if (snapshotDir && fs.existsSync(snapshotDir)) throw', 'if (false) throw'],
  ['U10-m6-expectation-is-the-v3-one', CRIT, "fail: { ...EXPECTED_FAIL, M6: ['S10', 'S10b'] }", 'fail: { ...EXPECTED_FAIL }'],
  ['U11-m6-must-pass-list-dropped', CRIT, "pass: { M5: ['S7', 'S8', 'S9'], M6: ['S7', 'S8', 'S9'] },", "pass: { M5: ['S7', 'S8', 'S9'], M6: [] },"],
  ['U12-control-outcome-ignores-must-pass', CRIT, 'expected.every((k) => failed.includes(k)) && passViolations.length === 0', 'expected.every((k) => failed.includes(k))'],
];
const results = [];
const run = (id, file, from, to) => {
  const root = path.join(out, id);
  for (const rel of ['tests', 'samples', 'tools/gate-common']) fs.cpSync(path.join(src, rel), path.join(root, 'slam_refact/web-chunk', rel), { recursive: true });
  if (from !== null) {
    const target = path.join(root, file);
    const text = fs.readFileSync(target, 'utf8');
    const n = text.split(from).length - 1;
    if (n !== 1) throw new Error(`${id}: pattern occurs ${n} times in ${file}`);
    fs.writeFileSync(target, text.replace(from, () => to));
  }
  const r = spawnSync(process.execPath, ['--test', path.join(root, 'slam_refact/web-chunk/tests/us010-v4.test.mjs')], { encoding: 'utf8' });
  fs.mkdirSync(out, { recursive: true });
  fs.writeFileSync(path.join(out, `${id}.log`), r.stdout + r.stderr);
  return { id, exit: r.status, failed: [...r.stdout.matchAll(/^not ok \d+ - (\S+)/gm)].map((m) => m[1].replace(/:$/, '')) };
};
results.push({ ...run('NULL', null, null, null), expected: 'pass' });
for (const [id, file, from, to] of M) results.push({ ...run(id, file, from, to), expected: 'fail' });
const detected = results.slice(1).filter((r) => r.exit !== 0 && r.failed.length > 0);
const summary = { schema: 'us010-v4-tool-mutants-v1', at: new Date().toISOString(), nullRunPasses: results[0].exit === 0, mutants: M.length, detected: detected.length,
  survivors: results.slice(1).filter((r) => !(r.exit !== 0 && r.failed.length > 0)).map((r) => r.id), results };
fs.writeFileSync(path.join(out, 'summary.json'), `${JSON.stringify(summary, null, 1)}\n`);
for (const r of results) console.log(`${r.id.padEnd(40)} exit ${r.exit} ${r.expected === 'pass' ? '(null-run)' : `failed: ${r.failed.join(', ')}`}`);
const ok = summary.nullRunPasses && summary.survivors.length === 0;
console.log(ok ? `OK: null-run passes, ${summary.detected}/${summary.mutants} mutants detected` : `FAILED: survivors ${summary.survivors.join(', ')}`);
process.exitCode = ok ? 0 : 1;

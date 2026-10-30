// rm3a-lighting-gate.v3 first-run driver (RM-3a / US-005). The measurer is the unchanged gtest sr_tests_xr_api_evidence (slam_refact/engine/tests/engine/xr_api_evidence_test.cc);
// this driver adds the HELD device (shared sr_held.mjs, protocol 12.5), the pins and the verdict (judge.mjs).  An accident guard, not an authentication (protocol 12.6).
//
//   node run.mjs --prereg DIR --gate rm3a-lighting-gate.v3 --env-var NAME --bin HELD_BIN --data DATA_DIR --out NEWDIR     real first run (marker, then the four gates)
//   node run.mjs --prereg DIR --gate rm3a-lighting-gate.v3 --env-var NAME --bin HELD_BIN --data DATA_DIR --check          sr_held CheckRelease only (no marker, nothing runs)
//   node run.mjs --bin HELD_BIN --data DATA_DIR --null-mutation --out NEWDIR                                               marker-free: the 7 null-run/mutation tests only (N1 N2 N2b M1 M2 M3 M3b)
//   node run.mjs --bin HELD_BIN --print-pins                                                                              the pin set {basename: sha256}
//
// Exit codes: 0 PASS; 1 FAIL; 2 usage; 3 HELD (no marker: not a run); 4 INVALID after the marker (or a failed null-run/mutation).
// The real run never runs a test outside the four gate tests; --null-mutation never runs a gate test (the filters are constants of judge.mjs).
import fs from 'node:fs';
import path from 'node:path';
import { spawnSync } from 'node:child_process';
import { fileURLToPath } from 'node:url';
import { checkRelease, heldGate, sha256 } from '../../prereg/sr_held.mjs';
import { GATE_TESTS, NULLMUT_TESTS, gtestFilter, judgeGate, judgeNullMutation } from './judge.mjs';

const here = path.dirname(fileURLToPath(import.meta.url));
const repo = path.resolve(here, '../../../..');
const argv = process.argv.slice(2);
const args = {};
for (let i = 0; i < argv.length; i++) {
  const a = argv[i];
  if (['--check', '--print-pins', '--null-mutation'].includes(a)) args[a.slice(2)] = true;
  else if (['--prereg', '--gate', '--env-var', '--bin', '--data', '--out'].includes(a)) args[a.slice(2)] = argv[++i];
  else { console.error(`unknown argument ${a}`); process.exit(2); }
}
const usage = (why) => { console.error(`usage error: ${why}`); process.exit(2); };
const SNAP = path.join(repo, 'build/slam-refact-rm3a-v3-20261023');
const DATA_ABS = path.join(SNAP, 'src/build');          // = SR_REPO_ROOT/build of the held binary (compile-time paths)
const OUT_ROOT = path.join(SNAP, 'runs');
if (!args.bin) usage('--bin is required');
const pinFiles = [args.bin, path.join(here, 'run.mjs'), path.join(here, 'judge.mjs'), path.join(here, 'run.sh'),
  path.join(repo, 'slam_refact/tools/prereg/sr_held.mjs'), process.execPath];
if (args['print-pins']) {
  const o = {};
  for (const f of pinFiles) o[path.basename(f)] = sha256(fs.readFileSync(f));
  console.log(JSON.stringify(o, null, 1));
  process.exit(0);
}
if (!args.data) usage('--data is required');
if (fs.realpathSync(args.data) !== DATA_ABS) usage(`--data must be ${DATA_ABS}`);
// the held binary reads its inputs through compile-time paths: every ".../slam-refact-full-20261019/oracle" path inside it must be the registered data dir (checked before any marker)
{
  const buf = fs.readFileSync(args.bin);
  const hits = new Set();
  const re = /(\/[\x21-\x7e]+?)\/slam-refact-full-20261019\/oracle/g;
  for (const m of buf.toString('latin1').matchAll(re)) hits.add(m[1]);
  if (hits.size !== 1 || ![...hits][0].endsWith('/build') || [...hits][0] !== DATA_ABS) usage(`the binary's compile-time oracle path(s) ${JSON.stringify([...hits])} are not the registered data dir ${DATA_ABS}`);
}
// a minimal environment: no GTEST_* variable (filter, shuffle, repeat) can change what runs
const childEnv = { PATH: process.env.PATH ?? '/usr/bin:/bin', HOME: process.env.HOME ?? '/', LANG: 'C', LC_ALL: 'C' };
const runGtest = (tests, logFile) => {
  const r = spawnSync(args.bin, [`--gtest_filter=${gtestFilter(tests)}`, '--gtest_color=no', '--gtest_shuffle=0', '--gtest_repeat=1'],
    { env: childEnv, encoding: 'utf8', maxBuffer: 1 << 28, cwd: DATA_ABS });
  const text = `${r.stdout ?? ''}${r.stderr ? `\n[stderr]\n${r.stderr}` : ''}`;
  if (logFile) fs.writeFileSync(logFile, text, { flag: 'wx' });
  return { text: r.stdout ?? '', status: r.status ?? (r.signal ? 128 : 4), signal: r.signal };
};
const newOut = () => {
  if (!args.out) usage('--out NEWDIR is required');
  const o = path.resolve(args.out);
  if (!o.startsWith(OUT_ROOT + path.sep)) usage(`--out must be a new directory under ${OUT_ROOT}`);
  if (fs.existsSync(o)) usage(`${o} exists (a new directory is required)`);
  return o;
};
const sums = (out) => {
  const files = [];
  const walk = (d) => { for (const e of fs.readdirSync(d, { withFileTypes: true })) { const p = path.join(d, e.name); e.isDirectory() ? walk(p) : files.push(p); } };
  walk(out);
  files.sort();
  fs.writeFileSync(path.join(out, 'SHA256SUMS'), files.map((x) => `${sha256(fs.readFileSync(x))}  ${path.relative(out, x)}\n`).join(''), { flag: 'wx' });
};

if (args['null-mutation']) {
  if (args.check || args['env-var'] || args.prereg || args.gate) usage('--null-mutation takes only --bin --data --out');
  const out = newOut();
  fs.mkdirSync(out, { recursive: true });
  const r = runGtest(NULLMUT_TESTS, path.join(out, 'gtest-nullmut.log'));
  const j = judgeNullMutation(r.text, r.status);
  fs.writeFileSync(path.join(out, 'nullmut-verdict.json'), JSON.stringify({ binary_sha256: sha256(fs.readFileSync(args.bin)), exit: r.status, ...j }, null, 1), { flag: 'wx' });
  sums(out);
  console.log(`[RM3A-V3-NULLMUT] ${j.verdict} ${JSON.stringify(Object.fromEntries(Object.entries(j.items).map(([k, v]) => [k, v.status])))}`);
  process.exit(j.verdict === 'PASS' ? 0 : 4);
}

for (const k of ['prereg', 'gate', 'env-var']) if (!args[k]) usage(`--${k} is required`);
const gm = /^(.+)\.v([1-9][0-9]{0,5})$/.exec(args.gate);
if (!gm) usage('--gate must be <gate_id>.v<N>');
const rq = {
  preregDir: path.resolve(args.prereg), gateId: gm[1], version: Number(gm[2]), kind: 'main',
  env: process.env[args['env-var']] ?? '', pins: pinFiles, dataDir: DATA_ABS, wantSeed: false,
};
if (args.check) {
  const o = checkRelease(rq);
  console.log(`${o.log.join('\n')}${o.log.length ? '\n' : ''}${o.ok ? 'RELEASED (check only: no marker created)' : `HELD [${o.code}]: ${o.why}`}`);
  process.exit(o.ok ? 0 : 3);
}
const out = newOut();                        // a usage error here is raised BEFORE the marker
const held = heldGate(rq);
console.log(held.log.join('\n'));
if (!held.ok) { console.error(`HELD [${held.code}]: ${held.why}`); process.exit(3); }
// ---- from here on the marker exists: this is the run ----
let code = 4;
try {
  fs.mkdirSync(out, { recursive: true });
  fs.writeFileSync(path.join(out, 'held.log'), held.log.join('\n') + '\n', { flag: 'wx' });
  const r = runGtest(GATE_TESTS, path.join(out, 'gtest-gates.log'));
  const j = judgeGate(r.text, r.status);
  const result = { gate: args.gate, binary_sha256: sha256(fs.readFileSync(args.bin)), designation_sha256: held.designationSha256, exit: r.status, signal: r.signal ?? null, ...j };
  fs.writeFileSync(path.join(out, 'result.json'), JSON.stringify(result, null, 1), { flag: 'wx' });
  sums(out);
  console.log(`[RM3A-V3-GATE] ${j.verdict} ${JSON.stringify(Object.fromEntries(Object.entries(j.items).map(([k, v]) => [k, v.status])))} reported=${JSON.stringify(j.reported)}`);
  code = { PASS: 0, FAIL: 1, INVALID: 4 }[j.verdict];
} catch (e) {
  console.error(`INVALID after the marker: ${e && e.stack ? e.stack : e}`);
  code = 4;
}
process.exit(code);

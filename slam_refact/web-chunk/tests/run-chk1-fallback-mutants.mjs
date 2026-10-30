// Mutation check of tests/chk1-fallback.test.mjs on copies of the chunk under build/ (the repo chunk is never touched): the null copy must pass, every mutant fail.
//   node slam_refact/web-chunk/tests/run-chk1-fallback-mutants.mjs --out build/slam-refact-full-20261019/evidence/OR-1/chk1-fallback-mutants
import fs from 'node:fs';
import path from 'node:path';
import { spawnSync } from 'node:child_process';
import { parseArgs } from 'node:util';

const repo = path.resolve(import.meta.dirname, '../../..');
const { values } = parseArgs({ options: { out: { type: 'string' } } });
const out = path.resolve(repo, values.out ?? '');
if (!values.out || !out.startsWith(path.join(repo, 'build') + path.sep)) throw new Error('--out must be under build/');
if (fs.existsSync(out)) throw new Error(`refusing to overwrite ${out}`);
const src = path.join(repo, 'slam_refact/web-chunk');
const T = 'f.frameTime ? f.frameTime * 1e9 : f.timeNanos,';
const M = [
  ['R3-fallback-deleted', T, 'f.frameTime * 1e9,'],
  ['R4-always-stage-time (CHK-1 reverted)', T, 'f.timeNanos,'],
  ['R5-fallback-to-zero', T, 'f.frameTime ? f.frameTime * 1e9 : 0,'],
  ['R6-fallback-to-video-time', T, 'f.frameTime ? f.frameTime * 1e9 : f.videoTime * 1e9,'],
  ['R7-frame-time-in-ms', T, 'f.frameTime ? f.frameTime * 1e6 : f.timeNanos,'],
  ['R8-frame-time-rounded', T, 'f.frameTime ? Math.round(f.frameTime * 1e9) : f.timeNanos,'],
];
fs.mkdirSync(out, { recursive: true });
const run = (id, dir) => {
  const r = spawnSync(process.execPath, ['--test', path.join(src, 'tests/chk1-fallback.test.mjs')], { env: { ...process.env, SLAM_REFACT_CHUNK_DIR: dir }, encoding: 'utf8' });
  fs.writeFileSync(path.join(out, `${id.split(' ')[0]}.log`), r.stdout + r.stderr);
  return { id, exit: r.status };
};
const copy = (id) => { const d = path.join(out, id.split(' ')[0], 'chunk'); fs.mkdirSync(d, { recursive: true }); for (const f of fs.readdirSync(src).filter((x) => x.endsWith('.js'))) fs.copyFileSync(path.join(src, f), path.join(d, f)); return d; };
const results = [{ ...run('NULL', copy('NULL')), expected: 'pass' }];
for (const [id, from, to] of M) {
  const d = copy(id), f = path.join(d, 'xr-slam.js'), t = fs.readFileSync(f, 'utf8');
  if (t.split(from).length - 1 !== 1) throw new Error(`${id}: pattern count`);
  fs.writeFileSync(f, t.replace(from, () => to));
  results.push({ ...run(id, d), expected: 'fail' });
}
const ok = results[0].exit === 0 && results.slice(1).every((r) => r.exit !== 0);
fs.writeFileSync(path.join(out, 'summary.json'), `${JSON.stringify({ at: new Date().toISOString(), ok, results }, null, 1)}\n`);
for (const r of results) console.log(`${r.id.padEnd(42)} exit ${r.exit} (${r.expected})`);
console.log(ok ? `OK: null passes, ${results.length - 1}/${results.length - 1} mutants detected` : 'FAILED');
process.exitCode = ok ? 0 : 1;

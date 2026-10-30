// Mutation check of tests/us030-curvy-geometry.test.mjs: every mutant is a modified COPY of the chunk files under build/ (the repo files are
// never touched); the test must pass on the unmodified copy (null-run) and fail on every mutant.
//   node slam_refact/web-chunk/tests/run-us030-mutants.mjs --out build/slam-refact-full-20261019/evidence/US-030/mutants
// Exit code 0 only if the null-run passes and all mutants are detected. Refuses to write outside build/ or into an existing --out.
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
const files = fs.readdirSync(src).filter((f) => f.endsWith('.js'));

// [id, file, from, to]: `from` must occur exactly once
const MUTANTS = [
  ['M1-swap-radius-top-bottom', 'image-targets.js', 'radiusTop: g.topRadius,\n    radiusBottom: g.bottomRadius,', 'radiusTop: g.bottomRadius,\n    radiusBottom: g.topRadius,'],
  ['M2-keep-capnp-names', 'image-targets.js', 'radiusTop: g.topRadius,\n    radiusBottom: g.bottomRadius,', 'topRadius: g.topRadius,\n    bottomRadius: g.bottomRadius,'],
  ['M3-forward-all-nine-fields', 'image-targets.js', 'return {\n    height: g.height,', 'return {\n    ...g,\n    height: g.height,'],
  ['M4-swap-arc-fields', 'image-targets.js', 'arcLengthRadians: g.arcLengthRadians,\n    arcStartRadians: g.arcStartRadians,', 'arcLengthRadians: g.arcStartRadians,\n    arcStartRadians: g.arcLengthRadians,'],
  ['M5-height-from-side-length', 'image-targets.js', 'height: g.height,', 'height: g.curvySideLength,'],
  ['M6-key-order-height-last', 'image-targets.js', 'return {\n    height: g.height,\n    radiusTop: g.topRadius,\n    radiusBottom: g.bottomRadius,\n    arcLengthRadians: g.arcLengthRadians,\n    arcStartRadians: g.arcStartRadians,\n  };',
    'return {\n    radiusTop: g.topRadius,\n    radiusBottom: g.bottomRadius,\n    arcLengthRadians: g.arcLengthRadians,\n    arcStartRadians: g.arcStartRadians,\n    height: g.height,\n  };'],
  ['M7-flat-geometry-for-curved', 'image-targets.js', 'if (itMetadata && !curved) {', 'if (itMetadata) {'],
  ['M8-curved-geometry-not-merged', 'image-targets.js', 'Object.assign(image, stored.geometry);', 'if (!msg.curvyGeometry) Object.assign(image, stored.geometry);'],
  ['M9-geometry-refreshed-every-detection', 'image-targets.js', 'if (stored.geometry === undefined && msg.curvyGeometry)', 'if (msg.curvyGeometry)'],
  ['M10-geometry-never-stored', 'image-targets.js', 'stored.geometry = curvyGeometryToJs(msg.curvyGeometry);', 'Object.assign(image, curvyGeometryToJs(msg.curvyGeometry));'],
];

fs.mkdirSync(out, { recursive: true });
const run = (id, dir) => {
  const r = spawnSync(process.execPath, ['--test', path.join(src, 'tests/us030-curvy-geometry.test.mjs')],
    { env: { ...process.env, SLAM_REFACT_CHUNK_DIR: dir }, encoding: 'utf8' });
  fs.writeFileSync(path.join(out, `${id}.log`), r.stdout + r.stderr);
  const failed = [...r.stdout.matchAll(/^not ok \d+ - (\S+)/gm)].map((m) => m[1].replace(/:$/, ''));
  return { id, exit: r.status, failedTests: failed, passCount: Number((r.stdout.match(/^# pass (\d+)/m) || [])[1]) };
};
const copy = (id) => {
  const dir = path.join(out, id, 'chunk');
  fs.mkdirSync(dir, { recursive: true });
  for (const f of files) fs.copyFileSync(path.join(src, f), path.join(dir, f));
  return dir;
};
const results = [];
results.push({ ...run('NULL', copy('NULL')), expected: 'pass' });
for (const [id, file, from, to] of MUTANTS) {
  const dir = copy(id);
  const text = fs.readFileSync(path.join(dir, file), 'utf8');
  const n = text.split(from).length - 1;
  if (n !== 1) throw new Error(`${id}: pattern occurs ${n} times in ${file}`);
  fs.writeFileSync(path.join(dir, file), text.replace(from, () => to));
  results.push({ ...run(id, dir), expected: 'fail' });
}
const ok = results[0].exit === 0 && results.slice(1).every((r) => r.exit !== 0 && r.failedTests.length > 0);
const summary = { schema: 'us030-mutants-v1', at: new Date().toISOString(), nullRunPasses: results[0].exit === 0, mutants: results.length - 1,
  detected: results.slice(1).filter((r) => r.exit !== 0 && r.failedTests.length > 0).length, results };
fs.writeFileSync(path.join(out, 'summary.json'), JSON.stringify(summary, null, 1) + '\n');
for (const r of results) console.log(`${r.id.padEnd(40)} exit ${r.exit} ${r.expected === 'pass' ? '(null-run)' : `failed: ${r.failedTests.join(', ')}`}`);
console.log(ok ? `OK: null-run passes, ${summary.detected}/${summary.mutants} mutants detected` : 'FAILED');
process.exitCode = ok ? 0 : 1;

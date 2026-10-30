// Mutation check of the rotation support of the OR-1 players (slam_refact/web-chunk/tools/or1/players.mjs), by the tests of web-chunk/tests/or1-rotation.test.mjs.
// Every mutant is a modified COPY of tools/or1 under build/ (SLAM_REFACT_OR1_TOOLS_DIR points the tests at it); the repo files are never touched. The tests must pass on the
// unmodified copy (null-run) and fail on every mutant except the ones listed as equivalent (a defect the harness cannot show: frameStartResult.orientation is not read by the chunk).
//   node players-mutants.mjs --out build/<lane>/or1-v4-players-mutants
import fs from 'node:fs';
import path from 'node:path';
import { spawnSync } from 'node:child_process';
import { parseArgs } from 'node:util';

const repo = path.resolve(import.meta.dirname, '../../../..');
const { values } = parseArgs({ options: { out: { type: 'string' } } });
const out = path.resolve(repo, values.out ?? '');
if (!values.out || !out.startsWith(path.join(repo, 'build') + path.sep)) throw new Error('--out must be a new directory under build/');
if (fs.existsSync(out)) throw new Error(`refusing to overwrite ${out}`);
const toolsSrc = path.join(repo, 'slam_refact/web-chunk/tools/or1');
const test = path.join(repo, 'slam_refact/web-chunk/tests/or1-rotation.test.mjs');

// [id, from, to, equivalent?]: `from` occurs exactly once in players.mjs
const M = [
  ['PM1-attach-rotation-zero', 'rotation: cfg.rotationDeg, computeCtx: gl', 'rotation: 0, computeCtx: gl', false],
  ['PM2-guard-allows-anything', 'if (!CHUNK_ROTATIONS.includes(cfg.rotationDeg)) throw', 'if (false) throw', false],
  ['PM3-portrait-only-guard-back', 'if (!CHUNK_ROTATIONS.includes(cfg.rotationDeg)) throw', 'if (cfg.rotationDeg !== 0 || cfg.captureWidth > cfg.captureHeight) throw', false],
  ['PM4-supported-set-without-270', 'export const CHUNK_ROTATIONS = [0, 90, 180, 270, -90, -180, -270];', 'export const CHUNK_ROTATIONS = [0, 90, 180, -90, -180, -270];', false],
  ['PM5-supported-set-with-45', 'export const CHUNK_ROTATIONS = [0, 90, 180, 270, -90, -180, -270];', 'export const CHUNK_ROTATIONS = [0, 45, 90, 180, 270, -90, -180, -270];', false],
  ['PM6-session-size-swapped', 'pm.onSessionAttach({ videoWidth: W, videoHeight: H, canvasWidth: W, canvasHeight: H,', 'pm.onSessionAttach({ videoWidth: H, videoHeight: W, canvasWidth: W, canvasHeight: H,', false],
  ['PM7-frame-texture-size-swapped', 'textureWidth: W, textureHeight: H, orientation: cfg.rotationDeg,', 'textureWidth: H, textureHeight: W, orientation: cfg.rotationDeg,', false],
  ['PM8-frame-orientation-zero', 'textureWidth: W, textureHeight: H, orientation: cfg.rotationDeg,', 'textureWidth: W, textureHeight: H, orientation: 0,', true],
  ['PM9-rotation-from-capture-shape', 'rotation: cfg.rotationDeg, computeCtx: gl', 'rotation: W > H ? 90 : 0, computeCtx: gl', false],
];
fs.mkdirSync(out, { recursive: true });
const run = (dir) => spawnSync(process.execPath, ['--test', test], { cwd: repo, encoding: 'utf8', timeout: 600000, env: { ...process.env, SLAM_REFACT_OR1_TOOLS_DIR: dir } });
function place(id, patch) {
  const dir = path.join(out, id);
  fs.mkdirSync(dir, { recursive: true });
  for (const f of fs.readdirSync(toolsSrc).filter((x) => /\.(mjs|sh)$/.test(x))) fs.copyFileSync(path.join(toolsSrc, f), path.join(dir, f));
  if (patch) {
    const file = path.join(dir, 'players.mjs');
    const text = fs.readFileSync(file, 'utf8');
    const n = text.split(patch[0]).length - 1;
    if (n !== 1) throw new Error(`${id}: pattern occurs ${n} times`);
    fs.writeFileSync(file, text.replace(patch[0], () => patch[1]));
  }
  return dir;
}
const nullRun = run(place('NULL', null));
const results = M.map(([id, from, to, equivalent]) => {
  const r = run(place(id, [from, to]));
  return { id, equivalent, detected: r.status !== 0, failedTests: (r.stdout.match(/^not ok /gm) ?? []).length };
});
const survivors = results.filter((r) => !r.equivalent && !r.detected).map((r) => r.id);
const summary = { nullRunPassed: nullRun.status === 0, mutants: results.filter((r) => !r.equivalent).length, detected: results.filter((r) => !r.equivalent && r.detected).length, survivors,
  equivalentMutants: results.filter((r) => r.equivalent).map((r) => ({ id: r.id, detectedAnyway: r.detected })), results };
fs.writeFileSync(path.join(out, 'summary.json'), `${JSON.stringify(summary, null, 1)}\n`);
console.log(JSON.stringify({ ...summary, results: undefined }));
process.exitCode = summary.nullRunPassed && survivors.length === 0 ? 0 : 1;

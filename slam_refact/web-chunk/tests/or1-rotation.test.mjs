// node --test slam_refact/web-chunk/tests/or1-rotation.test.mjs
// G4 / OR-1 v4: the replay harness plays landscape and rotated (90 / 180 / 270 and the negatives) bundles through the chunk (tools/or1/players.mjs
// lifted its "portrait capture only (R16)" guard) and plays portrait rotation-0 bundles exactly as before.
//   ROT-1  the rotation guard: the chunk's supported set, a rotation the chunk refuses, no engine needed
//   ROT-2  W, C, N-free: a synthetic bundle per rotation reaches the engine through the chunk (glue config, sizes, IN-CW / OUT-CW / ANS identical)
//   ROT-3  a rotated bundle is not the portrait one: W's engine output differs between rotations (the metamorphic pair of the gate)
//   ROT-4  compat: for a portrait rotation-0 bundle the arms' complete output (frames, rows, answers, calls, glue config, inexact counters) is byte identical
//          to the one of the pre-G4 players.mjs (git 36f969c, sha256 fa88239e...): the portrait default output did not change
// Engine: SLAM_REFACT_OR1_ENGINE_DIR (default build/slam-refact-or1-20261020/wasm/wasm; it must contain FULL-8, rotation_deg); the tests that need it are
// skipped when it is absent. No browser, no GPU, no network, no registered gate data (the bundles are synthetic or the development bundle D1).
import assert from 'node:assert/strict';
import { spawnSync } from 'node:child_process';
import fs from 'node:fs';
import path from 'node:path';
import test from 'node:test';
import zlib from 'node:zlib';
import { pathToFileURL } from 'node:url';

const repo = path.resolve(import.meta.dirname, '../../..');
const toolsDir = process.env.SLAM_REFACT_OR1_TOOLS_DIR ?? path.join(repo, 'slam_refact/web-chunk/tools/or1');
const chunkDir = path.join(repo, 'slam_refact/web-chunk');
const engineDir = path.resolve(repo, process.env.SLAM_REFACT_OR1_ENGINE_DIR ?? 'build/slam-refact-or1-20261020/wasm/wasm');
const haveEngine = fs.existsSync(path.join(engineDir, 'slam_refact_engine.js'));
const load = (f) => import(pathToFileURL(path.join(toolsDir, f)).href);
const { CHUNK_ROTATIONS, playChunk, playDirect, loadEngineModule } = await load('players.mjs');
const { readBundle } = await load('bundle.mjs');
const { evaluate } = await load('run.mjs');
const { canon, sha256 } = await load('canon.mjs');

const scratch = path.join(repo, 'build/slam-refact-g4-20261025/unit-tmp', `rotation-${process.pid}`);
fs.mkdirSync(scratch, { recursive: true });
process.on('exit', () => fs.rmSync(scratch, { recursive: true, force: true }));
let n = 0;
const fresh = (tag) => path.join(scratch, `${tag}-${++n}`);

// ---- a synthetic bundle in the format of prepare_inputs.py (engine-config.tsv, sensors.tsv, frames.tsv, atlas files) ----
// The atlas is deterministic pseudo-random bytes of the engine's own layout (rows), so the engine runs its real code on it; it carries no scene.
function lcg(seed) { let x = seed >>> 0; return () => { x = (Math.imul(x, 1664525) + 1013904223) >>> 0; return x >>> 24; }; }
async function layoutRowsOf(w, h, rot) {
  const Module = await loadEngineModule(engineDir);
  const glue = new Module.SlamRefactGlue({ captureWidth: w, captureHeight: h, rotationDeg: rot, manufacturer: '', model: 'K', pyramidModel: 'swiftshader-approx' });
  const rows = glue.pyramidLayout().rows;
  glue.delete();
  return rows;
}
async function syntheticBundle(tag, { w, h, rot, frames = 10, seed = 7 }) {
  const dir = fresh(tag);
  fs.mkdirSync(dir, { recursive: true });
  const rows = await layoutRowsOf(w, h, rot);
  const rnd = lcg(seed);
  const atlas = path.join(dir, 'atlas.rgba.gz');
  const bytes = new Uint8Array(1024 * rows * 4);
  for (let i = 0; i < bytes.length; i++) bytes[i] = rnd();
  fs.writeFileSync(atlas, zlib.gzipSync(bytes));
  const cfg = [['scenario', 'synthetic'], ['label', tag], ['capture_width', w], ['capture_height', h], ['rotation_deg', rot], ['manufacturer', ''], ['model', 'K'],
    ['estimate_scale', 0], ['disable_vio', 0], ['pyramid_model', 'swiftshader-approx'], ['origin', '0 2 0'], ['facing_wxyz', '1 0 0 0'], ['left_handed_axes', 0],
    ['mirrored_display', 0], ['t0_s', '1.0']];
  fs.writeFileSync(path.join(dir, 'engine-config.tsv'), cfg.map(([k, v]) => `${k}\t${v}\n`).join(''));
  const T0 = 1_000_000_000, STEP = 33_333_333;
  const sensor = (phase, t, k) => [phase, t, 0.1 * k % 1, 9.7, 0.5 + 0.01 * k, '-', '-', '-', 0.2 * k % 3, -0.3, 0.4, 24.03, 359.2 + k * 0.1, 63.2, 1.3].join('\t');
  const srows = [sensor('bootstrap', T0 - 16_000_000, 0)];
  for (let k = 0; k < frames * 3 + 3; k++) srows.push(sensor('run', T0 + k * 11_000_000, k + 1));
  fs.writeFileSync(path.join(dir, 'sensors.tsv'), `${srows.join('\n')}\n`);
  const frow = (kind, idx, t, ft) => [kind, idx, t, t - 5_000_000, (t - T0) / 1e9, 1, '-', atlas, ft].join('\t');
  const frows = [frow('bootstrap', -1, T0 - 16_000_000, 0)];
  for (let k = 0; k < frames; k++) frows.push(frow('frame', k, T0 + (k + 1) * STEP, T0 + (k + 1) * STEP - 2_000_000));
  fs.writeFileSync(path.join(dir, 'frames.tsv'), `${frows.join('\n')}\n`);
  return { dir, bundle: readBundle(dir), rows };
}

// ---- ROT-1 ----
test('ROT-1: CHUNK_ROTATIONS is the chunk\'s own supported set; a rotation the chunk refuses is refused before any engine work', async () => {
  const chunk = fs.readFileSync(path.join(chunkDir, 'xr-slam.js'), 'utf8');
  const m = chunk.match(/const SUPPORTED_ROTATIONS = (\[[^\]]*\]);/);
  assert.ok(m, 'xr-slam.js SUPPORTED_ROTATIONS not found');
  assert.deepEqual(JSON.parse(m[1]), CHUNK_ROTATIONS);
  const cfg = (rot) => new Map(Object.entries({ capture_width: '640', capture_height: '480', rotation_deg: String(rot), manufacturer: '', model: 'K', estimate_scale: '0',
    disable_vio: '0', left_handed_axes: '0', mirrored_display: '0', pyramid_model: 'swiftshader-approx', origin: '0 2 0', facing_wxyz: '1 0 0 0' }));
  for (const rot of [45, 360, 1, -45, Number.NaN]) {
    await assert.rejects(playChunk({ chunkDir: '/nonexistent', engineDir: '/nonexistent', bundle: { config: cfg(rot) }, repoRoot: repo }), /rotation 0, 90, 180, 270/, `rotation ${rot}`);
  }
});

// ---- ROT-2 / ROT-3 ----
const CASES = [['portrait-0', 480, 640, 0], ['landscape-0', 640, 480, 0], ['landscape-90', 640, 480, 90], ['portrait-180', 480, 640, 180], ['landscape-270', 640, 480, 270],
  ['landscape-m90', 640, 480, -90], ['portrait-m180', 480, 640, -180]];
const results = new Map();
const play = async (label, w, h, rot) => {
  if (results.has(label)) return results.get(label);
  const { bundle } = await syntheticBundle(label, { w, h, rot });
  const W = await playDirect({ engineDir, bundle, repoRoot: repo });
  const C = await playChunk({ chunkDir, engineDir, bundle, repoRoot: repo });
  const out = { W, C, rot, w, h, g: evaluate({ runs: { W: [{ ...W, wallMs: 0 }], C: [{ ...C, wallMs: 0 }] }, expected: { calls: 11, frames: 10 } }) };
  results.set(label, out);
  return out;
};

for (const [label, w, h, rot] of CASES) {
  test(`ROT-2: ${label} (${w}x${h}, rotation ${rot}) reaches the engine through the chunk: the session is built from the bundle, IN-CW / OUT-CW / ANS identical`,
    { skip: !haveEngine && 'engine dir with FULL-8 not present', timeout: 120000 }, async () => {
      const { C, W, g } = await play(label, w, h, rot);
      assert.deepEqual(C.errors, [], 'no error callback');
      assert.equal(C.glueConfig.captureWidth, canon(w), 'captureWidth');
      assert.equal(C.glueConfig.rotationDeg, canon(rot), 'rotationDeg');
      assert.deepEqual([C.glueConfig.captureWidth, C.glueConfig.captureHeight, C.glueConfig.rotationDeg], [W.glueConfig.captureWidth, W.glueConfig.captureHeight, W.glueConfig.rotationDeg],
        'the glue the chunk built has the bundle\'s capture size and rotation');
      assert.equal(g['IN-CW'].identical, true, JSON.stringify(g['IN-CW'].first));
      assert.equal(g['OUT-CW'].identical, true, JSON.stringify(g['OUT-CW'].firstDiff));
      assert.equal(g.C6.pass, true, JSON.stringify(g.C6));
      assert.equal(g.C7.pass, true);
      assert.equal(g.C8.pass, true, JSON.stringify(g.C8));
      assert.equal(g['OUT-CW'].frames, 10);
      assert.equal(C.rows.filter((r) => r.fresh).length, 10, 'every frame reached the engine');
    });
}

test('ROT-3: a rotated bundle is not the portrait one (metamorphic pair): the engine\'s complete output differs between rotation 0 and rotation 90 / 180',
  { skip: !haveEngine && 'engine dir with FULL-8 not present', timeout: 120000 }, async () => {
    const p0 = await play('portrait-0', 480, 640, 0);
    const p180 = await play('portrait-180', 480, 640, 180);
    const l90 = await play('landscape-90', 640, 480, 90);
    const key = (r) => sha256(JSON.stringify(r.W.rows.map((x) => x.pose)));
    assert.notEqual(key(p0), key(p180), 'rotation 180 changes the output of the same sensor stream');
    assert.notEqual(key(p0), key(l90));
  });

// ---- ROT-4 ----
// One process per players.mjs (tools/gates/or1-chunk-replay-v4/compat-check.mjs): two copies of players.mjs in one process share the chunk module URL.
const OLD_COMMIT = '36f969c';
const OLD_SHA = 'fa88239e3fdb8fe6968c9faaf9dc4efadfde5c0d3b5fcc2c912f1e86f262b362';
const compat = path.join(repo, 'slam_refact/tools/gates/or1-chunk-replay-v4/compat-check.mjs');
function oldPlayersDir() {
  const r = spawnSync('git', ['show', `${OLD_COMMIT}:slam_refact/web-chunk/tools/or1/players.mjs`], { cwd: repo, encoding: 'buffer', maxBuffer: 1 << 24 });
  if (r.status !== 0 || sha256(r.stdout) !== OLD_SHA) return null;
  const dir = fresh('old-players');
  fs.mkdirSync(dir, { recursive: true });
  fs.writeFileSync(path.join(dir, 'players.mjs'), r.stdout);
  for (const f of ['canon.mjs', 'bundle.mjs']) fs.copyFileSync(path.join(toolsDir, f), path.join(dir, f));  // unchanged by G4 (pinned in v3)
  return dir;
}
const compatRun = (playersDir, bundleDir) => {
  const r = spawnSync(process.execPath, [compat, '--players-dir', playersDir, '--bundle', bundleDir, '--engine-dir', engineDir], { cwd: repo, encoding: 'utf8', maxBuffer: 1 << 26 });
  assert.equal(r.status, 0, r.stderr);
  return JSON.parse(r.stdout);
};
const sameAsOld = (bundleDir, minFresh) => {
  const oldDir = oldPlayersDir();
  if (!oldDir) return null;
  const a = compatRun(oldDir, bundleDir), b = compatRun(toolsDir, bundleDir);
  assert.equal(a.playersSha256, OLD_SHA);
  assert.equal(b.W, a.W, 'arm W output differs from the pre-G4 players.mjs');
  assert.equal(b.C, a.C, 'arm C output differs from the pre-G4 players.mjs');
  assert.ok(a.freshRowsC >= minFresh && a.freshRowsW >= minFresh, 'the comparison is not vacuous');
  return b;
};

test('ROT-4: portrait rotation-0 output of the arms is byte identical to the pre-G4 players.mjs (synthetic portrait bundles, 2 seeds)', { skip: !haveEngine && 'engine dir with FULL-8 not present', timeout: 300000 }, async (t) => {
  for (const seed of [7, 99]) {
    const { dir } = await syntheticBundle(`compat-${seed}`, { w: 480, h: 640, rot: 0, frames: 12, seed });
    if (sameAsOld(dir, 12) === null) { t.skip(`git ${OLD_COMMIT}:players.mjs is not available (or its sha256 is not ${OLD_SHA})`); return; }
  }
});

test('ROT-4b: on the development bundle D1 (flip6-s2, portrait, 20 frames) the output of both arms is byte identical to the pre-G4 players.mjs', { skip: !haveEngine && 'engine dir with FULL-8 not present', timeout: 300000 }, async (t) => {
  const src = path.join(repo, 'build/slam-refact-20261016/fix/round2/bundles-seed/flip6-s2/pmr3a');
  if (!fs.existsSync(path.join(src, 'frames.tsv'))) { t.skip('the development bundle D1 is not present'); return; }
  const { derive } = await load('bundle.mjs');
  const derived = fresh('d1');
  derive(src, derived, { frames: 20, sourceLabel: 'D1' });
  if (sameAsOld(derived, 18) === null) t.skip(`git ${OLD_COMMIT}:players.mjs is not available (or its sha256 is not ${OLD_SHA})`);
});

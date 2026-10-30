// node --test slam_refact/web-chunk/tests/chk1-engine-time.test.mjs
// CHK-1 (spec 07 section 12): the time argument the chunk hands to processFramePyramid is the time func 647 picks - frameTimestampNanos
// (trunc(frameTime * 1e9)) when frameStartResult.frameTime is non-zero, else the stage time (timeNanos). The chunk is played through the OR-1 chunk
// driver on a small synthetic bundle (zero atlases, no sensors), the engine inputs are read from the recording glue.
//   SLAM_REFACT_CHUNK_DIR / SLAM_REFACT_CHUNK_ENGINE_DIR as in chunk-contract.test.mjs
import assert from 'node:assert/strict';
import fs from 'node:fs';
import path from 'node:path';
import test from 'node:test';
import zlib from 'node:zlib';
import { pathToFileURL } from 'node:url';

const repo = path.resolve(import.meta.dirname, '../../..');
const chunkDir = process.env.SLAM_REFACT_CHUNK_DIR ?? path.join(repo, 'slam_refact/web-chunk');
const engineDir = process.env.SLAM_REFACT_CHUNK_ENGINE_DIR ?? path.join(repo, 'build/slam-refact-full-20261019/chunk/rm4d-20261020/engine-pinned');
// SLAM_REFACT_OR1_TOOLS_DIR: a modified copy of tools/or1 (run-or1-mutants.mjs shows that CHK1-2 fails on a harness that hands the chunk the video time as frameTime)
const toolsDir = process.env.SLAM_REFACT_OR1_TOOLS_DIR ?? path.join(import.meta.dirname, '../tools/or1');
const { readBundle } = await import(pathToFileURL(path.join(toolsDir, 'bundle.mjs')).href);
const { playChunk, playDirect } = await import(pathToFileURL(path.join(toolsDir, 'players.mjs')).href);

const dir = path.join(repo, 'build/slam-refact-full-20261019/evidence/OR-1', `unit-tmp-chk1-${process.pid}`);
test.after(() => fs.rmSync(dir, { recursive: true, force: true }));

function bundle() {
  fs.mkdirSync(dir, { recursive: true });
  const atlas = path.join(dir, 'zero.rgba.gz');
  fs.writeFileSync(atlas, zlib.gzipSync(Buffer.alloc(1024 * 660 * 4)));
  fs.writeFileSync(path.join(dir, 'engine-config.tsv'), ['capture_width\t240', 'capture_height\t320', 'rotation_deg\t0', 'manufacturer\tGoogle', 'model\tPixel 7', 'estimate_scale\t0',
    'disable_vio\t0', 'pyramid_model\tswiftshader-approx', 'origin\t0 2 0', 'facing_wxyz\t1 0 0 0', 'left_handed_axes\t0', 'mirrored_display\t0'].map((l) => `${l}\n`).join(''));
  fs.writeFileSync(path.join(dir, 'sensors.tsv'), `bootstrap\t990000000\t0\t9.8\t0\t-\t-\t-\t0\t0\t0\t16\t0\t90\t0\n`);
  // time_ns (column 3) and frame_time_ns (column 9): 0 = not recorded
  const row = (kind, i, t, ft) => [kind, i, t, t, t / 1e9, 1, 'x.pgm', atlas, ft].join('\t');
  fs.writeFileSync(path.join(dir, 'frames.tsv'), [row('bootstrap', -1, 1000000000, 997500000), row('frame', 0, 1033333333, 1030100000), row('frame', 1, 1066666666, 0), row('frame', 2, 1100000000, 1096999999)]
    .map((l) => `${l}\n`).join(''));
  return readBundle(dir);
}

test('CHK1-1: the engine time is frameTime * 1e9 when frameTime is non-zero, else the stage time', async () => {
  const b = bundle();
  const c = await playChunk({ chunkDir, engineDir, bundle: b, repoRoot: repo });
  assert.deepEqual(c.errors, []);
  assert.deepEqual(c.frames.map((f) => f.timeNs), [997500000, 1030100000, 1066666666, 1096999999],
    'bootstrap and frames 0 and 2 use the recorded frame time (column 9); frame 1 has none and uses the stage time (column 3)');
  assert.deepEqual(c.frames.map((f) => f.videoTimeNs), [1000000000, 1033333333, 1066666666, 1100000000]);
});

// K-3 (OR-1 v2 pre-review): a bundle without column 9 (0) and a video time that differs from the stage time. W takes the stage time (column 3); the C driver
// must not hand the chunk the video time as frameTime (v2 did), or C differs from W by construction.
test('CHK1-2: column 9 = 0 gives the engine the stage time in C exactly as in W, also when the video time is not the stage time (K-3)', async () => {
  const b = bundle();
  const frames = fs.readFileSync(path.join(dir, 'frames.tsv'), 'utf8').split('\n').filter(Boolean).map((l) => l.split('\t'));
  frames[2][3] = String(Number(frames[2][2]) - 5_000_000);  // frame 1: video time 5 ms before the stage time, column 9 = 0
  fs.writeFileSync(path.join(dir, 'frames.tsv'), frames.map((c) => `${c.join('\t')}\n`).join(''));
  const b2 = readBundle(dir);
  assert.equal(b2.frames[2].frameTimeNs, 0);
  const c = await playChunk({ chunkDir, engineDir, bundle: b2, repoRoot: repo });
  const w = await playDirect({ engineDir, bundle: b2, repoRoot: repo });
  assert.deepEqual(c.errors, []);
  assert.equal(c.frames[2].videoTimeNs, 1066666666 - 5_000_000, 'the video time is passed through');
  assert.equal(c.frames[2].timeNs, 1066666666, 'engine time = stage time (column 3), not the video time');
  assert.deepEqual(c.frames.map((f) => f.timeNs), w.frames.map((f) => f.timeNs), 'C and W give the engine the same times');
  assert.deepEqual(c.frames.map((f) => f.videoTimeNs), w.frames.map((f) => f.videoTimeNs));
  assert.equal(b.frames.length, b2.frames.length);
});

// WASM bindings of the GPU frontend (web/slam-refact.html): pyramidLayout, cpuPyramidAtlas,
// processFramePyramid, the JS-error input guards and the tracker state export.
//   node --test slam_refact/engine/tests/wasm/   (after building build/slam-refact-wasm)
import assert from 'node:assert/strict';
import path from 'node:path';
import { createRequire } from 'node:module';
import test from 'node:test';

const repo = path.resolve(import.meta.dirname, '../../../..');
const wasmDir = process.env.SLAM_REFACT_WASM_DIR ?? path.join(repo, 'build/slam-refact-wasm/wasm');
const require = createRequire(import.meta.url);
const W = 240, H = 320;
const CONFIG = { captureWidth: W, captureHeight: H, manufacturer: 'Google', model: 'Pixel 7' };

function frameGray(k) {
  const img = new Uint8Array(W * H);
  for (let y = 0; y < H; y++) {
    for (let x = 0; x < W; x++) img[y * W + x] = ((((x + k) >> 3) ^ (y >> 3)) & 1) ? 200 : 40 + ((x * 7 + y * 13) % 31);
  }
  return img;
}

function grayToRgba(g) {
  const out = new Uint8Array(g.length * 4);
  for (let i = 0; i < g.length; i++) { out[4 * i] = out[4 * i + 1] = out[4 * i + 2] = g[i]; out[4 * i + 3] = 255; }
  return out;
}

function pushSensors(glue, t) {
  for (let j = 2; j >= 0; j--) {
    const ts = t - j * 16666667;
    glue.pushDeviceMotion(ts, 0, 9.81, 0, false, 0, 0, 0, 0, 0, 0, 16.666);
    glue.pushDeviceOrientation(ts, 0, 90, 0);
  }
}

const strip = (p) => ({ position: p.position, rotationWxyz: p.rotationWxyz, status: p.status, reason: p.reason,
  repeated: p.repeated, worldPoints: p.worldPoints, keypoints: p.keypoints, tracker: p.tracker });

test('injecting the CPU-port atlas reproduces the internal CPU path bit for bit', async () => {
  const Module = await require(path.join(wasmDir, 'slam_refact_engine.js'))();
  const a = new Module.SlamRefactGlue(CONFIG);
  const b = new Module.SlamRefactGlue(CONFIG);
  const layout = a.pyramidLayout();
  assert.equal(layout.textureSize, 1024);
  assert.equal(layout.levels.length, 4);
  assert.equal(layout.roiSlots.length, 4);
  assert.equal(layout.levels[1].rotated, true);
  assert.ok(layout.rows > 0 && layout.rows <= layout.textureSize);
  assert.deepEqual(layout.shaderParams, { gain: 1, scoreThresh: 4, nmsTolerance: 1, skipSubpixel: false });
  for (let k = 0; k < 41; k++) {
    const t = 1e9 + k * 50e6;
    pushSensors(a, t); pushSensors(b, t);
    const g = frameGray(k);
    const pa = a.processFrame(t, t, g, W, H);
    const atlas = Module.SlamRefactGlue.cpuPyramidAtlas(grayToRgba(g), W, H, 'ieee');
    assert.equal(atlas.size, layout.textureSize);
    assert.equal(atlas.rows, layout.rows);
    assert.equal(atlas.rgba.length, atlas.size * atlas.rows * 4);
    const pb = b.processFramePyramid(t, t, atlas.rgba, atlas.size, atlas.rows);
    assert.deepEqual(strip(pb), strip(pa), `frame ${k}`);
    assert.deepEqual(Array.from(b.getWorldPoints()), Array.from(a.getWorldPoints()), `frame ${k} world points`);
  }
  const last = a.getPose();
  assert.equal(typeof last.tracker.numKeyframes, 'number');
  assert.equal(typeof last.tracker.pointsBySource.triangulated, 'number');
  a.delete(); b.delete();
});

test('input errors throw JS errors and leave the module usable', async () => {
  const Module = await require(path.join(wasmDir, 'slam_refact_engine.js'))();
  const glue = new Module.SlamRefactGlue(CONFIG);
  const layout = glue.pyramidLayout();
  assert.throws(() => glue.processFrame(1e9, 1e9, new Uint8Array(10), W, H), /gray length/);
  assert.throws(() => glue.processFrame(Number.NaN, 0, frameGray(0), W, H), /finite/);
  assert.throws(() => glue.processFramePyramid(1e9, 1e9, new Uint8Array(16), layout.textureSize, layout.rows), /atlas length/);
  assert.throws(() => glue.processFramePyramid(1e9, 1e9, new Uint8Array(16), 512, 4), /pyramidLayout/);
  assert.throws(() => glue.pushDeviceOrientation(Infinity, 0, 90, 0), /finite/);
  assert.throws(() => Module.SlamRefactGlue.cpuPyramidAtlas(new Uint8Array(8), W, H, 'ieee'), /rgba length/);
  // Still usable after the errors.
  pushSensors(glue, 2e9);
  const p = glue.processFrame(2e9, 2e9, frameGray(1), W, H);
  assert.ok([...p.position, ...p.rotationWxyz].every(Number.isFinite));
  glue.delete();
});

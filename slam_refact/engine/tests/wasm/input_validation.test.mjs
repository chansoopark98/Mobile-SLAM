// FX-D2: invalid inputs at the JS surface throw a JS Error instead of aborting the module. After each
// error the SAME module instance must still construct a glue and process a frame (an abort would
// leave the module unusable).
//   SLAM_REFACT_WASM_DIR=<abs dir> node --test slam_refact/engine/tests/wasm/input_validation.test.mjs
import assert from 'node:assert/strict';
import path from 'node:path';
import { createRequire } from 'node:module';
import test from 'node:test';

const repo = path.resolve(import.meta.dirname, '../../../..');
const wasmDir = process.env.SLAM_REFACT_WASM_DIR ?? path.join(repo, 'build/slam-refact-wasm/wasm');
const require = createRequire(import.meta.url);
const W = 240, H = 320;
const base = { captureWidth: W, captureHeight: H, manufacturer: 'Google', model: 'Pixel 7' };

const img = new Uint8Array(W * H);
function step(glue, k) {
  const t = 1e9 + k * 50e6;
  for (let j = 2; j >= 0; j--) {
    const ts = t - j * 16666667;
    glue.pushDeviceMotion(ts, 0, 9.81, 0, false, 0, 0, 0, 0, 0, 0, 16.666);
    glue.pushDeviceOrientation(ts, 0, 90, 0);
  }
  for (let y = 0; y < H; y++) {
    for (let x = 0; x < W; x++) img[y * W + x] = ((((x + k) >> 3) ^ (y >> 3)) & 1) ? 200 : 40;
  }
  return glue.processFrame(t, t, img, W, H);
}

// A fresh glue of the same module still works: two frames give a published pose object.
function assertModuleUsable(Module, what) {
  const g = new Module.SlamRefactGlue(base);
  step(g, 0);
  const p = step(g, 1);
  assert.equal(typeof p, 'object', what);
  assert.ok(Array.isArray(p.intrinsics) && p.intrinsics.length === 16, what);
  g.delete();
}

function assertJsError(Module, fn, pattern, what) {
  assert.throws(fn, (e) => e instanceof Error && pattern.test(e.message), what);
  assertModuleUsable(Module, `after ${what}`);
}

test('constructor: capture size, rotation and int keys throw a JS Error, module stays usable', async () => {
  const Module = await require(path.join(wasmDir, 'slam_refact_engine.js'))();
  const cases = [
    [undefined, /config object/, 'no config'],
    [{}, /captureWidth and captureHeight/, 'capture size missing'],
    [{ ...base, captureWidth: 0 }, /captureWidth and captureHeight/, 'captureWidth 0'],
    [{ ...base, captureHeight: -320 }, /captureWidth and captureHeight/, 'captureHeight < 0'],
    [{ ...base, captureWidth: 240.5 }, /captureWidth must be an integer/, 'captureWidth fractional'],
    [{ ...base, captureHeight: 1e12 }, /captureHeight must be an integer/, 'captureHeight out of int range'],
    [{ ...base, captureWidth: NaN }, /captureWidth must be an integer/, 'captureWidth NaN'],
    [{ ...base, rotationDeg: 45 }, /rotationDeg must be/, 'rotationDeg 45'],
    [{ ...base, rotationDeg: 360 }, /rotationDeg must be/, 'rotationDeg 360'],
    [{ ...base, accuracy: { enabled: true, baWindow: 8.5 } }, /baWindow must be an integer/, 'baWindow fractional'],
  ];
  for (const [cfg, pattern, what] of cases) {
    assertJsError(Module, () => new Module.SlamRefactGlue(cfg), pattern, what);
  }
  // Every rotation the engine supports is accepted.
  for (const rotationDeg of [0, 90, 180, 270, -90, -180, -270]) {
    new Module.SlamRefactGlue({ ...base, rotationDeg }).delete();
  }
});

test('constructor: origin null is "not given" like facing null', async () => {
  const Module = await require(path.join(wasmDir, 'slam_refact_engine.js'))();
  const a = new Module.SlamRefactGlue({ ...base, origin: null, facing: null });
  const b = new Module.SlamRefactGlue(base);
  const noTiming = ({ stageMs, ...rest }) => rest;
  for (let k = 0; k < 4; k++) assert.deepEqual(noTiming(step(a, k)), noTiming(step(b, k)));
  a.delete(); b.delete();
});

test('image targets: unload argument form and target size bound throw a JS Error', async () => {
  const Module = await require(path.join(wasmDir, 'slam_refact_engine.js'))();
  const g = new Module.SlamRefactGlue(base);
  assertJsError(Module, () => g.unloadImageTargets(undefined), /array of names/, 'unloadImageTargets(undefined)');
  assertJsError(Module, () => g.unloadImageTargets('a'), /array of names/, 'unloadImageTargets(string)');
  assertJsError(Module, () => g.unloadImageTargets([1]), /must be strings/, 'unloadImageTargets([1])');
  g.unloadImageTargets([]);
  g.unloadImageTargets(['not-loaded']);
  const lum = new Uint8Array(16);
  assertJsError(Module, () => g.setImageTargets([{ json: '{}', luminance: lum, width: 4096, height: 4 }]),
    /at most 2048/, 'target width above the bound');
  assertJsError(Module, () => g.setImageTargets([{ json: '{}', luminance: lum, width: 2049, height: 4 }]),
    /at most 2048/, 'target width one above the bound');
  assertJsError(Module, () => g.setImageTargets([{ json: '{}', luminance: lum, width: 4, height: 2049 }]),
    /at most 2048/, 'target height one above the bound');
  assertJsError(Module, () => g.setImageTargets([{ json: '{}', luminance: lum, width: 4, height: 2.5 }]),
    /height must be an integer/, 'target height fractional');
  // The glue that saw the errors keeps working too.
  step(g, 0);
  assert.equal(typeof step(g, 1), 'object');
  assert.deepEqual(g.getImageTargetNames(), []);
  g.delete();
});

test('processFrame: a frame size other than the capture size throws a JS Error, module stays usable', async () => {
  const Module = await require(path.join(wasmDir, 'slam_refact_engine.js'))();
  const g = new Module.SlamRefactGlue(base);
  step(g, 0);
  // Same byte count, swapped dims (e.g. a camera restart in the other orientation).
  assertJsError(Module, () => g.processFrame(2e9, 2e9, new Uint8Array(W * H), H, W),
    /must equal captureWidth and captureHeight/, 'processFrame swapped dims');
  assertJsError(Module, () => g.processFrame(2e9, 2e9, new Uint8Array(4), 2, 2),
    /must equal captureWidth and captureHeight/, 'processFrame smaller frame');
  // The glue that saw the errors keeps working too.
  assert.equal(typeof step(g, 1), 'object');
  g.delete();
});

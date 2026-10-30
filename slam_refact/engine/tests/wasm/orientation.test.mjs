// WASM landscape / rotation (contract FULL-8, US-023, unit): a landscape capture with engineInit rotation 90 / 180 / 270 loads in
// node, gives finite poses, the reordered display projection and the shaderRotation of the pyramid layout. Unsupported
// rotations abort the module (SR_CHECK), so only supported values are exercised here. The comparison with the running
// original is the preregistered gate orient-us023-gate.v1 (native).
//   node --test slam_refact/engine/tests/wasm/   (after building build/slam-refact-wasm)
import assert from 'node:assert/strict';
import path from 'node:path';
import { createRequire } from 'node:module';
import test from 'node:test';

const repo = path.resolve(import.meta.dirname, '../../../..');
const wasmDir = process.env.SLAM_REFACT_WASM_DIR ?? path.join(repo, 'build/slam-refact-wasm/wasm');
const require = createRequire(import.meta.url);

function frame(w, h, k) {
  const img = new Uint8Array(w * h);
  for (let y = 0; y < h; y++) {
    for (let x = 0; x < w; x++) img[y * w + x] = ((((x + k) >> 3) ^ (y >> 3)) & 1) ? 200 : 40 + ((x * 7 + y * 13) % 31);
  }
  return img;
}

async function run(Module, cfg, frames) {
  const glue = new Module.SlamRefactGlue({ manufacturer: '', model: 'K', ...cfg });
  const w = cfg.captureWidth, h = cfg.captureHeight;
  const poses = [];
  for (let k = 0; k < frames; k++) {
    const t = 1e9 + k * 50e6;
    for (let j = 2; j >= 0; j--) {
      const ts = t - j * 16666667;
      glue.pushDeviceMotion(ts, 0, 9.81, 0, false, 0, 0, 0, 0, 0, 0, 16.666);
      glue.pushDeviceOrientation(ts, 0, 90, 0);
    }
    poses.push(glue.processFrame(t, t, frame(w, h, k), w, h));
  }
  const layout = glue.pyramidLayout();
  glue.delete();
  return { poses, layout };
}

test('landscape captures run with rotation 90 / 180 / 270 and publish the corrected projection', async () => {
  const Module = await require(path.join(wasmDir, 'slam_refact_engine.js'))();
  const portrait = await run(Module, { captureWidth: 480, captureHeight: 640, rotationDeg: 0 }, 12);
  const p = portrait.poses.at(-1).intrinsics;
  assert.ok(p[0] > 0 && p[5] > 0);
  for (const rotation of [90, 270, 180]) {
    const landscape = rotation !== 180;
    const { poses, layout } = await run(Module, {
      captureWidth: landscape ? 640 : 480, captureHeight: landscape ? 480 : 640, rotationDeg: rotation }, 12);
    for (const q of poses.slice(1)) {
      assert.ok([...q.position, ...q.rotationWxyz].every(Number.isFinite), `rotation ${rotation}`);
    }
    const m = poses.at(-1).intrinsics;
    // The portrait projection reordered: landscape exchanges m00 and m11, upside-down keeps them.
    if (landscape) {
      assert.equal(m[0], p[5], `m00 rotation ${rotation}`);
      assert.equal(m[5], p[0], `m11 rotation ${rotation}`);
    } else {
      assert.equal(m[0], p[0]);
      assert.equal(m[5], p[5]);
    }
    assert.equal(layout.captureWidth, landscape ? 640 : 480);
    assert.equal(layout.rotateInput, landscape);
    // gl-reality-frame.cc:379: shaderRotation = rotated level ? rotation - 90 : rotation.
    for (const slot of layout.roiSlots) assert.equal(slot.shaderRotation, slot.rotated ? rotation - 90 : rotation);
  }
});

test('the portrait default path is unchanged by the rotation support', async () => {
  const Module = await require(path.join(wasmDir, 'slam_refact_engine.js'))();
  const a = await run(Module, { captureWidth: 240, captureHeight: 320 }, 10);
  const b = await run(Module, { captureWidth: 240, captureHeight: 320, rotationDeg: 0 }, 10);
  assert.deepEqual(a.poses.map((q) => q.position), b.poses.map((q) => q.position));
  assert.deepEqual(a.layout.roiSlots.map((s) => s.shaderRotation), [0, 0, 0, 0]);
});

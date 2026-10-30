// WASM smoke (US-012, B-3, unit): the engine module loads in node and returns finite poses and
// statuses for 60 procedurally generated frames with synthetic IMU (no dataset needed).
//   node --test slam_refact/engine/tests/wasm/   (after building build/slam-refact-wasm)
import assert from 'node:assert/strict';
import path from 'node:path';
import { createRequire } from 'node:module';
import test from 'node:test';

const repo = path.resolve(import.meta.dirname, '../../../..');
const wasmDir = process.env.SLAM_REFACT_WASM_DIR ?? path.join(repo, 'build/slam-refact-wasm/wasm');
const require = createRequire(import.meta.url);

test('synthetic 60-frame clip gives finite poses and statuses', async () => {
  const Module = await require(path.join(wasmDir, 'slam_refact_engine.js'))();
  const glue = new Module.SlamRefactGlue({
    captureWidth: 240, captureHeight: 320, manufacturer: 'Google', model: 'Pixel 7',
  });
  const w = 240, h = 320;
  const img = new Uint8Array(w * h);
  const poses = [];
  for (let k = 0; k < 61; k++) {
    const t = 1e9 + k * 50e6;
    for (let j = 2; j >= 0; j--) {
      const ts = t - j * 16666667;
      glue.pushDeviceMotion(ts, 0, 9.81, 0, false, 0, 0, 0, 0, 0, 0, 16.666);
      glue.pushDeviceOrientation(ts, 0, 90, 0);
    }
    // Textured checkerboard moving 1 px per frame.
    for (let y = 0; y < h; y++) {
      for (let x = 0; x < w; x++) {
        img[y * w + x] = ((((x + k) >> 3) ^ (y >> 3)) & 1) ? 200 : 40 + ((x * 7 + y * 13) % 31);
      }
    }
    poses.push(glue.processFrame(t, t, img, w, h));
  }
  assert.equal(poses[0].repeated, true);
  // poses[0] repeats the initial (empty, UNSPECIFIED) response: nothing has been processed yet.
  for (const p of poses.slice(1)) {
    assert.ok([...p.position, ...p.rotationWxyz].every(Number.isFinite), JSON.stringify(p));
    assert.ok(['NORMAL', 'LIMITED', 'NOT_AVAILABLE'].includes(p.status), p.status);
  }
  for (let i = 1; i <= 5; i++) assert.deepEqual(poses[i].position, [0, 2, 0]);
  assert.equal(poses[6].repeated, true);
  assert.ok(poses.at(-1).keypoints > 0);
  glue.reset();
  assert.ok(glue.getWorldPoints() instanceof Float64Array);
  glue.delete();
});

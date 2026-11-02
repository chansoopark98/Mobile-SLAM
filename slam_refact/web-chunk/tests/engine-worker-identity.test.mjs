// node --test slam_refact/web-chunk/tests/engine-worker-identity.test.mjs
// R1 identity at engine level: one recorded input (camera pyramids from a deterministic synthetic sequence, IMU events, orientation,
// coordinate-system and recenter calls) is run through (A) the engine glue called directly, as the main-thread chunk does, and (B)
// the Worker facade over a second WASM instance behind a cloning loopback transport. Every frame's pose, world points and startup
// counter must be identical. Real engine WASM, no GL. The chunk-level counterpart is R1-C8 in engine-worker-chunk.test.mjs.
import assert from 'node:assert/strict';
import test from 'node:test';
import { toPlain } from '../engine-worker.js';
import { loadEngineInWorker } from '../engine-worker-client.js';
import { engineAvailable, engineUrl, installGlobals, loadRealModule, loopbackWorker, syntheticAtlas, syntheticMotion } from './engine-worker-harness.mjs';

const skip = engineAvailable ? false : 'engine build not found (SLAM_REFACT_CHUNK_ENGINE_DIR)';
const W = 240, H = 320, N = 60;
const config = { captureWidth: W, captureHeight: H, rotationDeg: 0, manufacturer: 'Google', model: 'Pixel 7' };

test('R1-I1: direct glue and Worker glue give identical poses, world points and counters for the same recorded input', { skip }, async () => {
  const Module = await loadRealModule();
  const g = installGlobals();
  try {
    const loop = loopbackWorker();
    const { worker } = await loadEngineInWorker(engineUrl, {}, { createWorker: () => loop.worker, maxInFlight: 1000 });
    await worker.requestLayout(W, H, 0);
    const direct = new Module.SlamRefactGlue(config);
    const facade = new worker.Module.SlamRefactGlue(config);
    assert.deepEqual(facade.pyramidLayout(), toPlain(direct.pyramidLayout()), 'the probed layout is the engine layout');

    const ops = (glue, k) => {
      if (k === 0) {
        glue.updateCameraProjectionMatrix({ origin: { x: 0, y: 2, z: 0 }, facing: { w: 1, x: 0, y: 0, z: 0 }, updateRecenterPoint: true });
        glue.configure({ enableWorldPoints: true, enableLighting: false, leftHandedAxes: false, mirroredDisplay: false });
        glue.updateCameraProjectionMatrix({ cam: { pixelRectWidth: W, pixelRectHeight: H, nearClipPlane: 0.01, farClipPlane: 1000 },
          origin: { x: 0, y: 2, z: 0 }, facing: { w: 1, x: 0, y: 0, z: 0 }, updateRecenterPoint: false });
      }
      if (k === 30) glue.recenter();
      if (k === 40) glue.updateCameraProjectionMatrix({ cam: { pixelRectWidth: 480, pixelRectHeight: 640 }, updateRecenterPoint: false });
    };
    const expected = [];
    for (let k = 0; k < N; k++) {
      ops(direct, k);
      const atlas = syntheticAtlas(Module, W, H, k);
      const motion = syntheticMotion(k);
      const t = 1e9 * (1 + k / 30);
      direct.pushDeviceMotionEvent(t - 5e6, motion);
      direct.pushDeviceOrientation(t, 0, 90, 0);
      const pose = toPlain(direct.processFramePyramid(t, 1e9 * (20 + k / 30), atlas.rgba, atlas.size, atlas.rows));
      expected.push({ pose, world: toPlain(direct.getWorldPoints()), framesUntilRecenter: direct.getFramesUntilRecenter() });

      ops(facade, k);
      const dst = facade.acquireAtlas(atlas.rgba.length);
      dst.set(atlas.rgba);
      facade.submitFrame({ motion: [[t - 5e6, motion]], orientation: [t, 0, 90, 0], timeNanos: t, videoNs: 1e9 * (20 + k / 30),
        atlas: dst.subarray(0, atlas.rgba.length), size: atlas.size, rows: atlas.rows });
    }
    await loop.idle();
    const got = facade.takeResults();
    assert.equal(got.length, N);
    assert.equal(worker.stats.opErrors, 0);
    for (let k = 0; k < N; k++) {
      assert.equal(got[k].error, undefined, `frame ${k}`);
      assert.deepEqual(got[k].pose, expected[k].pose, `pose of frame ${k}`);
      assert.deepEqual(got[k].world, expected[k].world, `world points of frame ${k}`);
    }
    assert.equal(facade.getFramesUntilRecenter(), expected.at(-1).framesUntilRecenter);
    assert.ok(expected.some((e) => e.world.length > 0), 'the sequence produced world points');
    assert.ok(new Set(expected.map((e) => e.pose.position.join(','))).size > 5, 'the sequence moved');
  } finally { g.restore(); }
});

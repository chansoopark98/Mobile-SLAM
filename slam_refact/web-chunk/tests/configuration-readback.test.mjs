// Read-only configuration diagnostics, on the selected real WASM and the existing fake-GL replay driver.
// REQUIRE_P0_READBACK=1 makes the fresh-binding check mandatory; old WASM remains usable with unknown fields.
import assert from 'node:assert/strict';
import fs from 'node:fs';
import path from 'node:path';
import test from 'node:test';
import zlib from 'node:zlib';
import { readBundle } from '../tools/or1/bundle.mjs';
import { loadEngineModule, playChunk } from '../tools/or1/players.mjs';

const repo = path.resolve(import.meta.dirname, '../../..');
const chunkDir = process.env.SLAM_REFACT_CHUNK_DIR ?? path.join(repo, 'slam_refact/web-chunk');
const engineDir = process.env.SLAM_REFACT_CHUNK_ENGINE_DIR
  ?? path.join(repo, 'build/slam-refact-full-20261019/chunk/rm4d-20261020/engine-pinned');
const dir = fs.mkdtempSync(path.join(repo, 'build/p0-config-readback-'));
test.after(() => fs.rmSync(dir, { recursive: true, force: true }));
const Module = await loadEngineModule(engineDir);
const hasReadback = typeof Module.SlamRefactGlue.getCompileFeatures === 'function';
if (process.env.REQUIRE_P0_READBACK === '1') assert.ok(hasReadback, 'fresh binding must expose compile features');

function bundle() {
  const atlas = path.join(dir, 'zero.rgba.gz');
  fs.writeFileSync(atlas, zlib.gzipSync(Buffer.alloc(1024 * 660 * 4)));
  fs.writeFileSync(path.join(dir, 'engine-config.tsv'), [
    'capture_width\t240', 'capture_height\t320', 'rotation_deg\t0', 'manufacturer\t', 'model\tK',
    'estimate_scale\t0', 'disable_vio\t0', 'pyramid_model\tieee', 'origin\t0 2 0', 'facing_wxyz\t1 0 0 0',
    'left_handed_axes\t0', 'mirrored_display\t0',
  ].join('\n') + '\n');
  fs.writeFileSync(path.join(dir, 'sensors.tsv'), 'bootstrap\t990000000\t0\t9.8\t0\t-\t-\t-\t0\t0\t0\t16\t0\t90\t0\n');
  const frames = Array.from({ length: 10 }, (_, k) => {
    const t = 1000000000 + k * 33333333;
    return [k ? 'frame' : 'bootstrap', k - 1, t, t, t / 1e9, 1, 'unused.pgm', atlas, t].join('\t');
  });
  fs.writeFileSync(path.join(dir, 'frames.tsv'), frames.join('\n') + '\n');
  return readBundle(dir);
}

test('session reports constructor config, device source and manifest hashes without enabling accuracy', async () => {
  const run = await playChunk({ chunkDir, engineDir, bundle: bundle(), repoRoot: repo });
  assert.deepEqual(run.errors, []);
  const s = globalThis.__slamRefactChunk.session;
  assert.deepEqual(s.glueConfig, { captureWidth: 240, captureHeight: 320, rotationDeg: 0,
    manufacturer: '', model: 'K', estimateScale: false, disableVio: false, leftHandedAxes: false, mirroredDisplay: false });
  assert.equal('accuracy' in s.glueConfig, false);
  assert.equal('intrinsics' in s.glueConfig, false);
  assert.deepEqual(s.device, { manufacturer: '', model: 'K', source: 'XR8.XrDevice.deviceEstimate' });
  const manifest = JSON.parse(fs.readFileSync(path.join(engineDir, 'manifest.json'), 'utf8'));
  assert.deepEqual(s.engineManifest, { jsSha256: manifest.artifacts.js.sha256, wasmSha256: manifest.artifacts.wasm.sha256,
    matchesLoadedBytes: true, glueReplaced: true }); // existing replay driver wraps the real glue
  const rb = s.configurationReadback;
  assert.deepEqual(rb.errors, []);
  if (hasReadback) {
    assert.equal(rb.compileFeatures.loopClose, false, 'this test candidate is built with LC OFF');
    assert.equal(rb.captureIntrinsics.width, 240);
    assert.equal(rb.captureIntrinsics.height, 320);
    assert.ok(Math.abs(rb.captureIntrinsics.fx - 506.4104 / 2) < 0.00003);
    assert.equal(rb.captureIntrinsics.cx, 119.5);
    assert.equal(rb.captureIntrinsics.cy, 159.5);
    assert.equal(rb.captureIntrinsicsSource, 'SlamRefactGlue.getCaptureIntrinsics');
  } else {
    assert.deepEqual(rb.compileFeatures, { loopClose: 'unknown' });
    assert.equal(rb.compileFeaturesSource, 'unavailable');
    assert.equal(rb.captureIntrinsics, null, 'display projection must not be mislabeled as capture K');
  }
});

test('diagnostic mutation cannot alter subsequent engine inputs or poses', async () => {
  const b = bundle();
  const control = await playChunk({ chunkDir, engineDir, bundle: b, repoRoot: repo });
  const changed = await playChunk({ chunkDir, engineDir, bundle: b, repoRoot: repo, onFrame: () => {
    const s = globalThis.__slamRefactChunk.session;
    s.glueConfig.model = 'SM-F741N';
    s.glueConfig.accuracy = { enabled: true, localBa: true };
    s.device.model = 'mutated-diagnostic';
    s.engineManifest.wasmSha256 = 'not-an-engine-input';
    if (s.configurationReadback.captureIntrinsics) s.configurationReadback.captureIntrinsics.fx = 1;
    s.configurationReadback.compileFeatures.loopClose = true;
  } });
  assert.deepEqual(changed.errors, []);
  assert.deepEqual(changed.glueConfig, control.glueConfig);
  assert.deepEqual(changed.frames, control.frames);
  assert.deepEqual(changed.rows, control.rows);
  assert.deepEqual(changed.answers, control.answers);
});

test('fresh capture getter returns configured pixel K independently of display projection', () => {
  const glue = new Module.SlamRefactGlue({ captureWidth: 240, captureHeight: 320,
    intrinsics: [311, 312, 101.5, 140.5] });
  try {
    if (!hasReadback) {
      assert.equal(typeof glue.getCaptureIntrinsics, 'undefined', 'legacy engine does not advertise capture K');
      return;
    }
    assert.equal(typeof glue.getCaptureIntrinsics, 'function');
    const expected = { width: 240, height: 320, fx: 311, fy: 312, cx: 101.5, cy: 140.5 };
    const first = glue.getCaptureIntrinsics();
    assert.deepEqual(first, expected);
    first.fx = 1;
    glue.updateCameraProjectionMatrix({ cam: { pixelRectWidth: 1000, pixelRectHeight: 1000, nearClipPlane: 0.1, farClipPlane: 20 } });
    assert.deepEqual(glue.getCaptureIntrinsics(), expected, 'display viewport and diagnostic mutation do not change capture K');
    assert.equal(glue.getIntrinsic().length, 16, 'the old getter remains a 4x4 projection');
  } finally { glue.delete(); }
});

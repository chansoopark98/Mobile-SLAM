// US-030 / D-1 / US-014 web branch, chunk code against the REAL engine WASM (desktop, node, no GL, no camera):
// createImageTargets (image-targets.js) loads the phone demo cylinder into the engine and the imagescanning payload carries the five
// curved fields the engine computed at load time. Detections on rendered frames are not covered here (native curvy gates, UNVERIFIED on web).
//   SLAM_REFACT_WASM_DIR=<dir with slam_refact_engine.js> SLAM_REFACT_DEMO_PGM_DIR=<dir with demo-*.pgm> \
//     node --test slam_refact/web-chunk/tests/us030-wasm-load-geometry.test.mjs
import assert from 'node:assert/strict';
import fs from 'node:fs';
import path from 'node:path';
import { createRequire } from 'node:module';
import { pathToFileURL } from 'node:url';
import test from 'node:test';

const repo = path.resolve(import.meta.dirname, '../../..');
const wasmDir = process.env.SLAM_REFACT_WASM_DIR;
const pgmDir = process.env.SLAM_REFACT_DEMO_PGM_DIR;
const chunkDir = process.env.SLAM_REFACT_CHUNK_DIR ?? path.join(repo, 'slam_refact/web-chunk');
const targetsDir = path.join(chunkDir, 'samples/phone/targets');
const skip = !wasmDir || !pgmDir ? 'SLAM_REFACT_WASM_DIR / SLAM_REFACT_DEMO_PGM_DIR not set' : false;
const require = createRequire(import.meta.url);
const { createImageTargets, curvyGeometryToJs } = await import(pathToFileURL(path.join(chunkDir, 'image-targets.js')));

const readPgm = (file) => {
  const b = fs.readFileSync(file);
  const m = /^P5\s+(\d+)\s+(\d+)\s+255\s/.exec(b.subarray(0, 32).toString('latin1'));
  const width = +m[1], height = +m[2];
  return { width, height, luminance: new Uint8Array(b.subarray(b.length - width * height)) };
};

test('US030-E1: the demo cylinder and the demo flat card load into the real engine; the cylinder gets its geometry at load', { skip }, async () => {
  const Module = await require(path.join(wasmDir, 'slam_refact_engine.js'))();
  const glue = new Module.SlamRefactGlue({ captureWidth: 480, captureHeight: 640, manufacturer: 'Google', model: 'Pixel 7' });
  const events = [];
  const framework = { dispatchEvent: (n, d) => events.push([n, d]) };
  const diag = { warnings: [] };
  const it = createImageTargets({
    options: { loadLuminance: async (url) => readPgm(path.join(pgmDir, path.basename(url).replace('_luminance.png', '.pgm'))), nextFrame: async () => {} },
    diag, pushEntry: (arr, v) => arr.push(v), getComputeCtx: () => null, getGlue: () => glue,
  });
  const targets = ['demo-flat', 'demo-cylinder'].map((n) => JSON.parse(fs.readFileSync(path.join(targetsDir, `${n}.json`), 'utf8')));
  it.configure(targets, { cameraProcessingDisabled: false, appResourcesLoaded: true, framework });
  await diag.imageTargets.whenIdle();
  assert.deepEqual(diag.warnings, [], 'no load error');
  assert.deepEqual(glue.getImageTargetNames(), ['demo-cylinder', 'demo-flat']);
  const scanning = events.find(([n]) => n === 'imagescanning')[1].imageTargets;
  const cyl = scanning.find((t) => t.name === 'demo-cylinder');
  const flat = scanning.find((t) => t.name === 'demo-flat');
  assert.equal(cyl.type, 'CYLINDRICAL');
  assert.deepEqual(cyl.geometry, curvyGeometryToJs(glue.getImageTargetGeometry('demo-cylinder')));
  assert.deepEqual(Object.keys(cyl.geometry), ['height', 'radiusTop', 'radiusBottom', 'arcLengthRadians', 'arcStartRadians']);
  assert.equal(cyl.geometry.radiusTop, cyl.geometry.radiusBottom);
  assert.ok(Math.abs(cyl.geometry.arcLengthRadians - 2 * Math.PI * 160 / 220) < 1e-3);
  assert.deepEqual(Object.keys(flat.geometry), ['scaledWidth', 'scaledHeight']);
  glue.delete?.();
});

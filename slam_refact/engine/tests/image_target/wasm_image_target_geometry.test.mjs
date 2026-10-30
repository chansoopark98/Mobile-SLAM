// US-030 / D-1 on the WASM build: SlamRefactGlue.getImageTargetGeometry(name) -> the nine capnp CurvyGeometry fields of a loaded curvy
// target, undefined for a flat or unknown target. Uses the phone demo targets (web-chunk/samples/phone/targets, binary PGM of the
// luminance image made by tools/image_target/to_pgm.mjs).
//   SLAM_REFACT_WASM_DIR=<dir with slam_refact_engine.js> SLAM_REFACT_DEMO_PGM_DIR=<dir with demo-*.pgm> \
//     node --test slam_refact/engine/tests/image_target/wasm_image_target_geometry.test.mjs
import assert from 'node:assert/strict';
import fs from 'node:fs';
import path from 'node:path';
import { createRequire } from 'node:module';
import test from 'node:test';

const repo = path.resolve(import.meta.dirname, '../../../..');
const wasmDir = process.env.SLAM_REFACT_WASM_DIR;
const pgmDir = process.env.SLAM_REFACT_DEMO_PGM_DIR;
const targetsDir = path.join(repo, 'slam_refact/web-chunk/samples/phone/targets');
const skip = !wasmDir || !pgmDir ? 'SLAM_REFACT_WASM_DIR / SLAM_REFACT_DEMO_PGM_DIR not set' : false;
const require = createRequire(import.meta.url);

const readPgm = (file) => {  // P5 <w> <h> 255 <raster>
  const b = fs.readFileSync(file);
  const m = /^P5\s+(\d+)\s+(\d+)\s+255\s/.exec(b.subarray(0, 32).toString('latin1'));
  assert.ok(m, 'binary PGM');
  const width = +m[1], height = +m[2];
  return { width, height, luminance: new Uint8Array(b.subarray(b.length - width * height)) };
};
const entry = (name) => ({ json: fs.readFileSync(path.join(targetsDir, `${name}.json`), 'utf8'), ...readPgm(path.join(pgmDir, `${name}.pgm`)) });
const FIELDS = ['curvyCircumferenceTop', 'curvyCircumferenceBottom', 'curvySideLength', 'targetCircumferenceTop', 'height', 'topRadius',
  'bottomRadius', 'arcLengthRadians', 'arcStartRadians'];

test('getImageTargetGeometry: curvy demo target -> nine finite fields; flat and unknown -> undefined; unload -> undefined', { skip }, async () => {
  const Module = await require(path.join(wasmDir, 'slam_refact_engine.js'))();
  const glue = new Module.SlamRefactGlue({ captureWidth: 480, captureHeight: 640, manufacturer: 'Google', model: 'Pixel 7' });
  glue.setImageTargets([entry('demo-cylinder'), entry('demo-flat')]);
  const g = glue.getImageTargetGeometry('demo-cylinder');
  assert.deepEqual(Object.keys(g), FIELDS);
  for (const f of FIELDS) assert.ok(Number.isFinite(g[f]), f);
  assert.equal(g.topRadius, g.bottomRadius, 'a cylinder');
  assert.ok(Math.abs(g.arcLengthRadians - 2 * Math.PI * 160 / 220) < 1e-3, 'label 160 mm of a 220 mm circumference');
  assert.equal(glue.getImageTargetGeometry('demo-flat'), undefined);
  assert.equal(glue.getImageTargetGeometry('missing'), undefined);
  glue.unloadImageTargets(['demo-cylinder']);
  assert.equal(glue.getImageTargetGeometry('demo-cylinder'), undefined);
  glue.delete?.();
});

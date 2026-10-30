// WASM curved image targets (contract FULL-9, US-014): a cylinder and a cone loaded through
// setImageTargets, found in the synthetic ray-cast sequences of render_synthetic.py --shape, reported as
// CYLINDRICAL / CONICAL with their curvyGeometry, and the curvy ROI uniforms of getRequestedRois().
//   SLAM_REFACT_WASM_DIR=<abs dir> SLAM_REFACT_CURVY_HELD_CHECK=<sr_curvy_held_check> \
//   SLAM_REFACT_IT_CYL_DIR=<dir> SLAM_REFACT_IT_CONE_DIR=<dir> \
//     node --test slam_refact/engine/tests/wasm/image_target_curvy.test.mjs
// DATA SEPARATION and HELD (curvy-it-gate, since v3 review B-1 / M-1): the checker and the data directories are named
// explicitly, nothing is defaulted (no build directory, gate id or version is written in this file). The checker reads
//   SLAM_REFACT_CURVY_GATE_ID / _GATE_VERSION / _PREREG_DIR / _DESIGNATION / _PINS    (curvy_sr_held.h)
// and every run first asks it (designation file, review record, the whole pin set incl. the checker, .js and .wasm, input
// tree sha256, one-run marker per data kind). A directory that holds the DEV variant (gt.tsv) is fine unless a gate is
// selected and the directory IS sealed gate data; a registered directory is HELD like any other. A missing variable fails the
// test (BLOCKED, not skipped). So the standard command `node --test tests/wasm/*.test.mjs` fails these 2 tests until the
// directories, the checker and the gate are named and designated, and 20 others pass. The checker's output (the release
// log) is kept as a test diagnostic on success too (audit F-6). Synthetic data: no device accuracy.
import assert from 'node:assert/strict';
import fs from 'node:fs';
import path from 'node:path';
import { createRequire } from 'node:module';
import { execFileSync } from 'node:child_process';
import test from 'node:test';

const repo = path.resolve(import.meta.dirname, '../../../..');
const wasmDir = process.env.SLAM_REFACT_WASM_DIR ?? path.join(repo, 'build/slam-refact-wasm/wasm');
const require = createRequire(import.meta.url);
const W = 480, H = 640;
const FRAMES = 36;  // the target is found at frame 15 and tracked

const heldCheck = process.env.SLAM_REFACT_CURVY_HELD_CHECK ?? '';
const dirs = {
  cylinder: process.env.SLAM_REFACT_IT_CYL_DIR ?? '',
  cone: process.env.SLAM_REFACT_IT_CONE_DIR ?? '',
};

// { reason, log }: reason is '' when the engine may run on `dir` (the C++ HELD device decides; no environment variable
// releases it), else why not; log is the checker's output, on success too.
function heldReason(shape, dir) {
  if (!heldCheck || !fs.existsSync(heldCheck)) {
    return { reason: `HELD: SLAM_REFACT_CURVY_HELD_CHECK (${heldCheck || 'not set'}) is not an existing sr_curvy_held_check`, log: '' };
  }
  if (!dir) {
    return { reason: `BLOCKED: the data directory of ${shape} is not set (SLAM_REFACT_IT_${shape === 'cylinder' ? 'CYL' : 'CONE'}_DIR)`, log: '' };
  }
  try {
    const out = execFileSync(heldCheck, ['--kind', `wasm-${shape}`, '--data-dir', dir,
      '--pin', path.join(wasmDir, 'slam_refact_engine.js'), '--pin', path.join(wasmDir, 'slam_refact_engine.wasm')],
    { encoding: 'utf8', stdio: ['ignore', 'pipe', 'pipe'] });
    return { reason: '', log: out.trim() };
  } catch (e) {
    const out = `${e.stdout ?? ''}${e.stderr ?? ''}`.trim();
    return { reason: out || `HELD: ${e.message}`, log: out };
  }
}

function readPgm(file) {
  const b = fs.readFileSync(file);
  let at = 0;
  const tok = () => {
    while (b[at] === 0x20 || b[at] === 0x0a || b[at] === 0x0d || b[at] === 0x09) at++;
    const s = at;
    while (b[at] > 0x20) at++;
    return b.toString('latin1', s, at);
  };
  assert.equal(tok(), 'P5');
  const w = Number(tok()), h = Number(tok());
  assert.equal(Number(tok()), 255);
  at++;
  return { w, h, data: new Uint8Array(b.buffer, b.byteOffset + at, w * h) };
}

function gtTimes(dir) {
  return fs.readFileSync(path.join(dir, 'gt.tsv'), 'utf8').split('\n')
    .filter((l) => /^\d/.test(l)).map((l) => Number(l.split('\t')[2]));
}

async function run(shape, t) {
  const dir = dirs[shape];
  const held = heldReason(shape, dir);
  t.diagnostic(`held check (${shape}): ${held.reason === '' ? 'released' : 'refused'} | ${held.log.replace(/\n/g, ' | ')}`);
  assert.equal(held.reason, '', held.reason);
  assert.ok(fs.existsSync(path.join(dir, 'gt.tsv')), `BLOCKED: ${dir}/gt.tsv missing (ctest fixture sr_image_target_curvy_render_${shape})`);
  const name = `synth-${shape}`;
  const Module = await require(path.join(wasmDir, 'slam_refact_engine.js'))();
  const g = new Module.SlamRefactGlue({ captureWidth: W, captureHeight: H, manufacturer: 'Google', model: 'Pixel 7', disableVio: true });
  const lum = readPgm(path.join(dir, `${name}_luminance.pgm`));
  g.setImageTargets([{ json: fs.readFileSync(path.join(dir, `${name}.json`), 'utf8'), luminance: lum.data, width: lum.w, height: lum.h }]);
  assert.deepEqual(g.getImageTargetNames(), [name]);
  const times = gtTimes(dir);
  const events = [];
  let detected = null;
  const roiLog = [];
  for (let k = 0; k < FRAMES; k++) {
    const t = times[k];
    for (let j = 1; j >= 0; j--) {
      const ts = t - j * 16666667;
      g.pushDeviceMotion(ts, 0, 9.81, 0, false, 0, 0, 0, 0, 0, 0, 16.666);
      g.pushDeviceOrientation(ts, 0, 90, 0);
    }
    const frame = readPgm(path.join(dir, 'frames', `${String(k).padStart(4, '0')}.pgm`));
    const p = g.processFrame(t, t, frame.data, W, H);
    for (const e of p.imageEvents) events.push([k, e.name]);
    if (p.detectedImages.length) detected = p.detectedImages[0];
    roiLog.push(g.getRequestedRois());
  }
  const geometry = JSON.parse(fs.readFileSync(path.join(dir, 'geometry.json'), 'utf8'));
  g.delete();
  return { events, detected, roiLog, geometry, name };
}

for (const [shape, apiType] of [['cylinder', 'CYLINDRICAL'], ['cone', 'CONICAL']]) {
  test(`${shape}: found, reported as ${apiType} with its curvyGeometry, curvy ROI uniforms exported`, async (t) => {
    const r = await run(shape, t);
    t.diagnostic(JSON.stringify({ events: r.events, detected: r.detected }));
    const found = r.events.filter((e) => e[1] === 'imagefound');
    // The embind surface is checked, not the transitions (the native gate does that): with the device orientation fed
    // the dev cone was lost for 4 frames after its first find (observed 2026-10-20, REPORT in curvy-it-gate.v3).
    assert.ok(found.length >= 1, 'at least one imagefound');
    assert.ok(found[0][0] >= 12 && found[0][0] <= 24, `found at frame ${found[0][0]}`);
    assert.ok(r.detected !== null);
    assert.equal(r.detected.name, r.name);
    assert.equal(r.detected.type, apiType);
    assert.deepEqual(Object.keys(r.detected).sort(),
      ['curvyGeometry', 'heightInMeters', 'id', 'name', 'position', 'rotation', 'scale', 'trackingStatus', 'type', 'widthInMeters']);
    const expected = r.geometry.expected_curvy_geometry;
    assert.deepEqual(Object.keys(r.detected.curvyGeometry).sort(), Object.keys(expected).sort());
    for (const [key, value] of Object.entries(expected)) {
      const got = r.detected.curvyGeometry[key];
      assert.ok(Math.abs(got - value) <= 1e-4 * Math.max(Math.abs(value), 1e-12), `${key}: ${got} vs ${value}`);
    }
    // the ROI requests of the frames after the find carry the curvy slot uniforms
    const curvy = r.roiLog.flat().filter((x) => x.source === 'CURVY_IMAGE_TARGET');
    assert.ok(curvy.length > 0, 'a CURVY_IMAGE_TARGET ROI is requested after the find');
    for (const roi of curvy) {
      assert.equal(roi.sourceId, 6);
      assert.equal(roi.name, r.name);
      assert.ok(roi.warp instanceof Float32Array);
      assert.deepEqual([...roi.warp], [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1], 'curvy ROI warp = identity');
      const c = roi.curvy;
      assert.ok(c && c.intrinsics instanceof Float32Array && c.globalPoseInverse instanceof Float32Array);
      assert.equal(c.intrinsics.length, 16);
      assert.equal(c.globalPoseInverse.length, 16);
      assert.ok([...c.intrinsics, ...c.globalPoseInverse, ...c.targetDims, ...c.roiDims, ...c.searchDims, ...c.scales,
        c.shift, c.radius, c.radiusBottom].every(Number.isFinite));
      assert.equal(c.rotation, 0, 'portrait target: vertex rotation 0');
      assert.equal(c.curvyPostRotation, 0);
      assert.deepEqual([...c.targetDims], [480, 640]);
      assert.deepEqual([...c.searchDims], [480, 640]);
      assert.ok(Math.abs(c.roiDims[0] - 480) < 1e-3 && Math.abs(c.roiDims[1] - 640) < 1e-3);
      // the tracker draws the ROI with a 0.04 radius margin (SHADER_ROI_RADIUS_INCREASE)
      assert.ok(Math.abs(c.radius - (r.geometry.rt + 0.04)) < 1e-4, `radius ${c.radius}`);
      assert.ok(Math.abs(c.radiusBottom - (r.geometry.rb + 0.04)) < 1e-4, `radiusBottom ${c.radiusBottom}`);
      assert.ok(Math.abs(c.shift - r.geometry.left) < 1e-5);
      assert.ok(Math.abs(c.scales[0] - (r.geometry.right - r.geometry.left) / 479) < 1e-7);
      assert.ok(Math.abs(c.scales[1] - 1 / 639) < 1e-7);
    }
    // the planar ROIs of the same request list (HIRES_SCAN fillers) have no curvy block
    for (const roi of r.roiLog.flat().filter((x) => x.source !== 'CURVY_IMAGE_TARGET')) {
      assert.equal(roi.curvy, undefined);
    }
  });
}

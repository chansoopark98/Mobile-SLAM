// WASM image target API (contract FULL-3, docs/design/image-target.md §7.4 IT-N1):
// setImageTargets / unloadImageTargets / clearImageTargets / getImageTargetNames, and
// getPose().detectedImages / imageEvents on a target pasted into synthetic frames.
//   SLAM_REFACT_WASM_DIR=<abs dir> node --test slam_refact/engine/tests/wasm/image_target.test.mjs
import assert from 'node:assert/strict';
import path from 'node:path';
import { createRequire } from 'node:module';
import test from 'node:test';

const repo = path.resolve(import.meta.dirname, '../../../..');
const wasmDir = process.env.SLAM_REFACT_WASM_DIR ?? path.join(repo, 'build/slam-refact-wasm/wasm');
const require = createRequire(import.meta.url);
const W = 240, H = 320;       // camera
const TW = 120, TH = 160;     // target luminance

async function makeGlue(extra = {}) {
  const Module = await require(path.join(wasmDir, 'slam_refact_engine.js'))();
  return new Module.SlamRefactGlue({ captureWidth: W, captureHeight: H, manufacturer: 'Google', model: 'Pixel 7', ...extra });
}

// xorshift32 texture of rectangles (deterministic).
function targetLuminance(seed) {
  const img = new Uint8Array(TW * TH);
  let s = seed >>> 0 || 1;
  const next = () => { s ^= s << 13; s >>>= 0; s ^= s >>> 17; s ^= s << 5; s >>>= 0; return s; };
  for (let i = 0; i < img.length; i++) img[i] = 80 + ((i % TW) >> 2);
  for (let k = 0; k < 220; k++) {
    const x0 = next() % TW, y0 = next() % TH, w = 3 + (next() % 14), h = 3 + (next() % 14), v = next() % 256;
    for (let y = y0; y < Math.min(TH, y0 + h); y++) for (let x = x0; x < Math.min(TW, x0 + w); x++) img[y * TW + x] = v;
  }
  return img;
}

function targetJson(name, type = 'PLANAR') {
  return JSON.stringify({
    type, name, imagePath: `image-targets/${name}_luminance.png`, metadata: null,
    properties: { left: 0, top: 0, width: TW, height: TH, isRotated: false, originalWidth: TW, originalHeight: TH,
      cylinderCircumferenceTop: 100, cylinderCircumferenceBottom: 100, cylinderSideLength: 50, targetCircumferenceTop: 50 },
  });
}

const lum = targetLuminance(7);
const frame = new Uint8Array(W * H);
function step(glue, k, withTarget) {
  const t = 1e9 + k * 33333333;
  for (let j = 1; j >= 0; j--) {
    const ts = t - j * 16666667;
    glue.pushDeviceMotion(ts, 0, 9.81, 0, false, 0, 0, 0, 0, 0, 0, 16.666);
    glue.pushDeviceOrientation(ts, 0, 90, 0);
  }
  for (let y = 0; y < H; y++) for (let x = 0; x < W; x++) frame[y * W + x] = 100 + ((x + y) >> 4);
  if (withTarget) {
    const ox = (W - TW) >> 1, oy = (H - TH) >> 1;
    for (let y = 0; y < TH; y++) for (let x = 0; x < TW; x++) frame[(oy + y) * W + ox + x] = lum[y * TW + x];
  }
  return glue.processFrame(t, t, frame, W, H);
}

test('setImageTargets: names, limit 32, curvy and input errors leave the set unchanged', async () => {
  const g = await makeGlue();
  assert.deepEqual(g.getImageTargetNames(), []);
  g.setImageTargets([{ json: targetJson('a'), luminance: lum, width: TW, height: TH },
    { json: targetJson('b'), luminance: lum, width: TW, height: TH }]);
  assert.deepEqual(g.getImageTargetNames(), ['a', 'b']);
  const many = Array.from({ length: 33 }, (_, i) => ({ json: targetJson(`t${i}`), luminance: lum, width: TW, height: TH }));
  assert.throws(() => g.setImageTargets(many), /too many active image targets/);
  // Curvy (CYLINDER / CONICAL) targets load since US-014 (FULL-9, image_target_curvy.test.mjs); a curvy JSON
  // without its cylinder* properties is the loader's JSON error (it was the US-014 refusal before).
  const noCylinder = targetJson('c', 'CYLINDER').replace('"cylinderCircumferenceTop":100,', '');
  assert.throws(() => g.setImageTargets([{ json: noCylinder, luminance: lum, width: TW, height: TH }]), /cylinderCircumferenceTop/);
  assert.throws(() => g.setImageTargets([{ json: targetJson('d'), luminance: lum.subarray(1), width: TW, height: TH }]), /luminance length/);
  assert.throws(() => g.setImageTargets([{ json: '{', luminance: lum, width: TW, height: TH }]), /image target 0/);
  assert.deepEqual(g.getImageTargetNames(), ['a', 'b']);
  g.unloadImageTargets(['a']);
  assert.deepEqual(g.getImageTargetNames(), ['b']);
  g.clearImageTargets();
  assert.deepEqual(g.getImageTargetNames(), []);
  const p = step(g, 0, false);
  assert.deepEqual(p.detectedImages, []);
  assert.deepEqual(p.imageEvents, []);
  g.delete();
});

test('a pasted target is found, reported as FLAT and lost when it leaves the frame', async (t) => {
  const g = await makeGlue({ disableVio: true });
  g.setImageTargets([{ json: targetJson('pasted'), luminance: lum, width: TW, height: TH }]);
  const events = [];
  let detected = null;
  // The target enters after the startup recenter (5 processed frames, rebuildEngine resets the image
  // tracker like the original) and leaves at frame 32.
  for (let k = 0; k < 44; k++) {
    const p = step(g, k, k >= 8 && k < 32);
    for (const e of p.imageEvents) events.push([k, e.name, e.detail.name]);
    if (p.detectedImages.length) detected = p.detectedImages[0];
    if (p.repeated) assert.deepEqual(p.imageEvents, []);
  }
  t.diagnostic(JSON.stringify({ events: events.map((e) => e.slice(0, 2)), detected }));
  const found = events.filter((e) => e[1] === 'imagefound');
  const lost = events.filter((e) => e[1] === 'imagelost');
  assert.equal(found.length, 1, 'one imagefound');
  assert.ok(found[0][0] >= 8 && found[0][0] <= 16, `found at frame ${found[0][0]}`);
  assert.equal(lost.length, 1, 'one imagelost');
  assert.ok(lost[0][0] >= 32 && lost[0][0] <= 37, `lost at frame ${lost[0][0]}`);
  assert.ok(detected !== null);
  assert.equal(detected.name, 'pasted');
  assert.equal(detected.type, 'FLAT');
  assert.equal(detected.id, -1);
  assert.equal(detected.trackingStatus, 0);
  assert.deepEqual(Object.keys(detected).sort(),
    ['heightInMeters', 'id', 'name', 'position', 'rotation', 'scale', 'trackingStatus', 'type', 'widthInMeters']);
  assert.ok([detected.position.x, detected.position.y, detected.position.z, detected.widthInMeters,
    detected.heightInMeters].every(Number.isFinite));
  assert.equal(detected.scale, Math.max(detected.widthInMeters, detected.heightInMeters));
  g.delete();
});

// Contract FULL-6 (RM-6, review M3): the ROI list of the next draw, for a GPU pyramid producer.
test('getRequestedRois exports the ROIs of the next draw with slot, source and warp', async () => {
  const g = await makeGlue({ disableVio: true });
  const layout = g.pyramidLayout();
  assert.equal(layout.captureWidth, W);
  assert.equal(layout.captureHeight, H);
  assert.equal(layout.roiSlots.length, 4);
  for (const slot of layout.roiSlots) {
    assert.ok(slot.w > 0 && slot.h > 0 && slot.r >= 0 && slot.c >= 0);
    assert.equal(slot.rotated, false);
    assert.equal(slot.shaderRotation, 0, 'portrait pyramid: rotation_ 0, slot not rotated');
  }

  // No target: nothing to draw.
  step(g, 0, false); step(g, 1, false); step(g, 2, false);
  assert.deepEqual(g.getRequestedRois(), []);

  g.setImageTargets([{ json: targetJson('pasted'), luminance: lum, width: TW, height: TH }]);
  const checkCommon = (rois) => {
    assert.ok(rois.length >= 1 && rois.length <= 4, `roi count ${rois.length}`);
    rois.forEach((r, i) => {
      assert.equal(r.slot, i, 'list order = slot order');
      assert.ok(r.slot < layout.roiSlots.length);
      assert.ok(['IMAGE_TARGET', 'HIRES_SCAN'].includes(r.source), r.source);
      assert.equal(r.sourceId, r.source === 'IMAGE_TARGET' ? 2 : 3);
      assert.ok(r.warp instanceof Float32Array && r.warp.length === 16);
      assert.ok([...r.warp].every(Number.isFinite));
      assert.notDeepEqual([...r.warp], [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1], 'a real warp, not identity');
    });
  };

  // Before the target appears the whole list is HIRES_SCAN fillers (faceId -1..-4).
  let early = null;
  for (let k = 3; k < 8; k++) { step(g, k, false); early = g.getRequestedRois(); }
  checkCommon(early);
  assert.equal(early.length, 4);
  early.forEach((r, i) => {
    assert.equal(r.source, 'HIRES_SCAN');
    assert.equal(r.faceId, -(i + 1));
    assert.equal(r.name, 'INTERNAL_HIRES_SCAN_ROI');
  });

  // While the target is tracked, its ROI comes first, then the HIRES fillers continue at -1.
  let tracked = null;
  for (let k = 8; k < 26; k++) {
    const p = step(g, k, true);
    if (k >= 20) tracked = g.getRequestedRois();
    assert.ok(p.detectedImages.length <= 4);
  }
  checkCommon(tracked);
  assert.equal(tracked.length, 4);
  assert.equal(tracked[0].source, 'IMAGE_TARGET');
  assert.equal(tracked[0].name, 'pasted');
  assert.equal(tracked[0].faceId, -1);
  tracked.slice(1).forEach((r, i) => {
    assert.equal(r.source, 'HIRES_SCAN');
    assert.equal(r.faceId, -(i + 1));
  });
  // The ROI warp of a tracked target differs from every HIRES filler (its centre is the target).
  assert.notDeepEqual([...tracked[0].warp], [...tracked[1].warp]);

  g.clearImageTargets();
  g.delete();
});

// Contract FULL-7 (RM-4b): an injected atlas has to contain the ROI slots while ROIs are requested.
test('processFramePyramid needs the ROI slot rows while image targets request ROIs', async () => {
  const g = await makeGlue({ disableVio: true });
  const layout = g.pyramidLayout();
  assert.ok(Number.isInteger(layout.roiAtlasRows));
  assert.ok(layout.roiAtlasRows > layout.rows && layout.roiAtlasRows <= layout.textureSize,
    `roiAtlasRows ${layout.roiAtlasRows} rows ${layout.rows}`);
  for (const slot of layout.roiSlots) assert.ok(slot.r + slot.h <= layout.roiAtlasRows);

  const rgba = new Uint8Array(W * H * 4);
  for (let p = 0; p < W * H; p++) { rgba.set([120, 120, 120, 255], 4 * p); }
  const cpu = g.constructor.cpuPyramidAtlas(rgba, W, H, 'ieee');
  const atlasOf = (rows) => {
    const a = new Uint8Array(layout.textureSize * rows * 4);
    a.set(cpu.rgba.subarray(0, Math.min(cpu.rgba.length, a.length)));
    return a;
  };
  const push = (k, rows) => {
    const t = 1e9 + k * 33333333;
    g.pushDeviceMotion(t, 0, 9.81, 0, false, 0, 0, 0, 0, 0, 0, 16.666);
    g.pushDeviceOrientation(t, 0, 90, 0);
    return g.processFramePyramid(t, t, atlasOf(rows), layout.textureSize, rows);
  };
  // Without targets: the world SLAM rows range is unchanged.
  for (let k = 0; k < 3; k++) push(k, layout.rows);
  assert.deepEqual(g.getRequestedRois(), []);
  g.setImageTargets([{ json: targetJson('pasted'), luminance: lum, width: TW, height: TH }]);
  push(3, layout.rows);   // the first response with targets loaded requests the ROIs of the next draw
  assert.ok(g.getRequestedRois().length > 0, 'ROIs requested once targets are loaded');
  assert.throws(() => push(4, layout.rows), /ROI slots/);
  const p = push(5, layout.roiAtlasRows);  // the producer drew the slot rows: accepted
  assert.ok(p.videoFrameIndex >= 0);
  assert.ok(g.getRequestedRois().length > 0);
  assert.throws(() => push(6, layout.roiAtlasRows - 1), /ROI slots/);
  push(7, layout.textureSize);  // any rows up to the texture height are fine
  g.clearImageTargets();
  g.delete();
});

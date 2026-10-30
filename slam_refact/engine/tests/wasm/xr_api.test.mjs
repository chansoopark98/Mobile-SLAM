// WASM XrController API (contract FULL-1, docs/spec/06-xr-controller-api.md): getPose().intrinsics /
// getIntrinsic (US-002), xrHitTest (US-003), configure / updateCameraProjectionMatrix / recenter
// (US-004), lighting (US-005), trackingStatusChanged and the front camera error (US-006).
//   SLAM_REFACT_WASM_DIR=<abs dir> node --test slam_refact/engine/tests/wasm/xr_api.test.mjs
import assert from 'node:assert/strict';
import path from 'node:path';
import { createRequire } from 'node:module';
import test from 'node:test';

const repo = path.resolve(import.meta.dirname, '../../../..');
const wasmDir = process.env.SLAM_REFACT_WASM_DIR ?? path.join(repo, 'build/slam-refact-wasm/wasm');
const require = createRequire(import.meta.url);
const W = 240, H = 320;

// build/slam-refact-20261015/reference/runs/tum-room1-full/r1/events.json intrinsicsChanges[0]
// (texture 240x320, near 0.01, far 1000, Google / Pixel 7).
const TUM_R1 = [2.174260377883911, 0, 0, 0, 0, 1.6306953430175781, 0, 0, 0, 0, -1.0000200271606445, -1,
  0, 0, -0.02000020071864128, 0];

async function makeGlue(extra = {}) {
  const Module = await require(path.join(wasmDir, 'slam_refact_engine.js'))();
  return new Module.SlamRefactGlue({ captureWidth: W, captureHeight: H, manufacturer: 'Google', model: 'Pixel 7', ...extra });
}

const img = new Uint8Array(W * H);
function step(glue, k) {
  const t = 1e9 + k * 50e6;
  for (let j = 2; j >= 0; j--) {
    const ts = t - j * 16666667;
    glue.pushDeviceMotion(ts, 0, 9.81, 0, false, 0, 0, 0, 0, 0, 0, 16.666);
    glue.pushDeviceOrientation(ts, 0, 90, 0);
  }
  for (let y = 0; y < H; y++) {
    for (let x = 0; x < W; x++) {
      img[y * W + x] = ((((x + k) >> 3) ^ (y >> 3)) & 1) ? 200 : 40 + ((x * 7 + y * 13) % 31);
    }
  }
  return glue.processFrame(t, t, img, W, H);
}

function near(a, b, tol, what) {
  assert.equal(a.length, b.length, what);
  a.forEach((v, i) => assert.ok(Math.abs(v - b[i]) <= tol, `${what}[${i}] ${v} vs ${b[i]}`));
}

test('reality.intrinsics matches the original TUM run and follows updateCameraProjectionMatrix', async () => {
  const glue = await makeGlue();
  const p0 = step(glue, 0);
  assert.deepEqual(p0.intrinsics, new Array(16).fill(0));  // no response yet
  const p1 = step(glue, 1);
  near(p1.intrinsics, TUM_R1, 1e-6, 'intrinsics');
  near(glue.getIntrinsic(), TUM_R1, 1e-6, 'getIntrinsic');
  glue.updateCameraProjectionMatrix({ cam: { pixelRectWidth: 180 } });  // crop: 180x320 display
  const p2 = step(glue, 2);
  assert.ok(Math.abs(p2.intrinsics[0] - 2 * 260.91125 / 180) < 1e-5, String(p2.intrinsics[0]));
  assert.ok(Math.abs(p2.intrinsics[5] - TUM_R1[5]) < 1e-6);
  assert.throws(() => glue.updateCameraProjectionMatrix({ cam: { pixelRectWidth: -1 } }), /pixelRectWidth/);
  glue.delete();
});

test('configure: runtime options, start-time errors and world points toggle', async () => {
  const on = await makeGlue();
  const off = await makeGlue();
  off.configure({ enableWorldPoints: false });
  // This synthetic scene never produces world points, so the toggle itself is only exercised on a
  // real map (flip6-s2 bundle test below); here the call must be accepted and keep the list empty.
  for (let k = 0; k < 40; k++) {
    step(on, k);
    assert.equal(step(off, k).worldPoints, 0);
  }
  assert.equal(off.getWorldPoints().length, 0);
  assert.throws(() => on.configure({ scale: 'absolute' }), (e) => e.message === '[XR] Scale can only be changed before calling XR8.run()');
  assert.throws(() => on.configure({ disableWorldTracking: true }),
    (e) => e.message === '[XR] World tracking can only be changed before calling XR8.run()');
  on.configure({ leftHandedAxes: true, mirroredDisplay: true, enableLighting: true });  // allowed at runtime
  const lit = step(on, 40);
  assert.ok(lit.lighting, 'lighting present after enableLighting');
  assert.ok(Number.isFinite(lit.lighting.exposure) && lit.lighting.exposure >= -1 && lit.lighting.exposure <= 1);
  assert.equal(lit.lighting.temperature, 6500);
  assert.equal(step(off, 40).lighting, undefined);
  on.delete();
  off.delete();
});

test('configure before start: absolute scale + disableWorldTracking error; origin used by the startup recenter', async () => {
  const g = await makeGlue();
  g.configure({ scale: 'absolute' });
  assert.throws(() => g.configure({ disableWorldTracking: true }),
    (e) => e.message === '[XR] Unable to perform metric scale estimation if world tracking is disabled.');
  const h = await makeGlue();
  h.updateCameraProjectionMatrix({ origin: { x: 1, y: 3, z: -2 }, facing: { w: 1, x: 0, y: 0, z: 0 } });
  const poses = [];
  for (let k = 0; k < 6; k++) poses.push(step(h, k));
  for (let i = 1; i <= 5; i++) assert.deepEqual(poses[i].position, [1, 3, -2]);
  g.delete();
  h.delete();
});

test('xrHitTest: surface types empty by default, no hit without a map, recenter clears lastCall', async () => {
  const g = await makeGlue();
  for (let k = 0; k < 40; k++) step(g, k);
  assert.deepEqual(g.xrHitTest(0.5, 0.5, ['ESTIMATED_SURFACE']), []);
  assert.deepEqual(g.xrHitTest(0.5, 0.5, ['DETECTED_SURFACE', 'ESTIMATED_SURFACE']), []);
  // The synthetic scene has no world points: FEATURE_POINT has nothing to average. The result
  // shape and values are checked on a real map in the flip6-s2 bundle test below.
  assert.equal(g.getWorldPoints().length, 0);
  assert.deepEqual(g.xrHitTest(0.5, 0.5, []), []);
  assert.deepEqual(g.xrHitTest(0.5, 0.5, ['FEATURE_POINT']), []);
  g.recenter();
  assert.deepEqual(g.xrHitTest(0.5, 0.5, []), []);
  assert.throws(() => g.xrHitTest(NaN, 0.5, []), /finite/);
  g.delete();
});

test('trackingStatusChanged and the front camera rejection', async () => {
  const g = await makeGlue();
  const p = [];
  for (let k = 0; k < 8; k++) p.push(step(g, k));
  assert.equal(p[0].trackingStatusChanged, false);  // no response
  assert.equal(p[1].trackingStatusChanged, true);   // UNSPECIFIED -> first status
  for (let i = 2; i < 8; i++) {
    const changed = p[i].status !== p[i - 1].status || p[i].reason !== p[i - 1].reason;
    assert.equal(p[i].trackingStatusChanged, changed, `frame ${i}`);
  }
  g.onCameraStatusChange('requesting', 'back');
  assert.throws(() => g.onCameraStatusChange('requesting', 'front'), (e) =>
    e.message === '[XR] World tracking is only supported on the back camera' && e.err === 'slam-front-camera-unsupported' &&
    e.type === 'configuration' && e.source === 'reality');
  const d = await makeGlue();
  d.configure({ disableWorldTracking: true });
  d.onCameraStatusChange('requesting', 'front');
  g.delete();
  d.delete();
});

// Recorded data (evidence tier): the flip6-s2 replay bundle has world points from frame 13 on, so
// the world point toggle, the hit test and the tracking status events are exercised with a real map.
test('flip6-s2 bundle: world points toggle, FEATURE_POINT hits and status events on a real map', async (t) => {
  const { readBundle, readPgm } = await import('./bundle_io.mjs');
  const { engineConfig } = await import('./bundle_replay.mjs');
  const fs = await import('node:fs');
  const bundleDir = process.env.SLAM_REFACT_FLIP6_BUNDLE ??
    path.join(repo, 'build/slam-refact-20261015/harness/bundles/flip6-s2/wp5b-r1');
  if (!fs.existsSync(path.join(bundleDir, 'frames.tsv'))) {
    assert.fail(`BLOCKED: bundle ${bundleDir} not present`);
  }
  const bundle = readBundle(bundleDir);
  const Module = await require(path.join(wasmDir, 'slam_refact_engine.js'))();
  const on = new Module.SlamRefactGlue(engineConfig(bundle.config));
  const off = new Module.SlamRefactGlue(engineConfig(bundle.config));
  off.configure({ enableWorldPoints: false });
  const push = (g, s) => {
    g.pushDeviceMotion(s.t, s.aig[0], s.aig[1], s.aig[2], s.lin !== null, ...(s.lin ?? [0, 0, 0]), s.rate[0], s.rate[1], s.rate[2], s.intervalMs);
    g.pushDeviceOrientation(s.t, s.orient[0], s.orient[1], s.orient[2]);
  };
  let next = 0;
  while (next < bundle.sensors.length && bundle.sensors[next].bootstrap) { push(on, bundle.sensors[next]); push(off, bundle.sensors[next]); next++; }
  const boot = readPgm(bundle.frames[0].image);
  on.processFrame(bundle.frames[0].timeNs, bundle.frames[0].videoTimeNs, boot.pixels, boot.width, boot.height);
  off.processFrame(bundle.frames[0].timeNs, bundle.frames[0].videoTimeNs, boot.pixels, boot.width, boot.height);
  let maxOn = 0;
  const responses = [];  // [status, reason, trackingStatusChanged] of every processed frame
  // 80 frames: the hit distance is 0 (degenerate) until the map has depth, about frame 50 here.
  for (const f of bundle.frames.slice(1, 81)) {
    while (next < bundle.sensors.length && bundle.sensors[next].t <= f.timeNs) { push(on, bundle.sensors[next]); push(off, bundle.sensors[next]); next++; }
    if (!f.process) continue;
    const frame = readPgm(f.image);
    const a = on.processFrame(f.timeNs, f.videoTimeNs, frame.pixels, frame.width, frame.height);
    const b = off.processFrame(f.timeNs, f.videoTimeNs, frame.pixels, frame.width, frame.height);
    maxOn = Math.max(maxOn, a.worldPoints);
    assert.equal(b.worldPoints, 0);
    assert.deepEqual(b.position, a.position);  // the mask changes the response, not the tracking
    responses.push([a.status, a.reason, a.trackingStatusChanged]);
  }
  assert.ok(maxOn > 0, 'world points published with the default configuration');

  // Hit test on a real map: result shape, non-degenerate values, the screen point matters.
  const hits = on.xrHitTest(0.5, 0.5, []);
  assert.equal(hits.length, 1);
  const shape = (h) => {
    assert.equal(h.type, 'FEATURE_POINT');
    assert.deepEqual(Object.keys(h).sort(), ['distance', 'position', 'rotation', 'type']);
    assert.ok([h.position.x, h.position.y, h.position.z, h.distance].every(Number.isFinite));
    assert.ok(h.distance > 0.01, `degenerate hit distance ${h.distance}`);
    // Original: rotation never written, makeRightHanded negates w and z of the zero quaternion.
    assert.ok(Object.is(h.rotation.w, -0) && Object.is(h.rotation.z, -0) && h.rotation.x === 0 && h.rotation.y === 0);
  };
  shape(hits[0]);
  const spread = [[0.5, 0.5], [0.25, 0.4], [0.75, 0.6]].map(([x, y]) => {
    const r = on.xrHitTest(x, y, ['FEATURE_POINT']);
    assert.equal(r.length, 1);
    shape(r[0]);
    return r[0].position;
  });
  assert.ok(spread.some((p) => Math.abs(p.x - spread[0].x) + Math.abs(p.y - spread[0].y) > 1e-3),
    'different screen points give different hits');
  // Metamorphic pair: at x = 0.5 the mirrored query ray is the same ray, so only the published x flips.
  assert.ok(Math.abs(hits[0].position.x) > 1e-4, 'the metamorphic pair needs a non-zero x');
  on.configure({ mirroredDisplay: true });
  const mirrored = on.xrHitTest(0.5, 0.5, []);
  assert.equal(mirrored.length, 1);
  assert.ok(Math.abs(mirrored[0].position.x + hits[0].position.x) < 1e-5);
  assert.ok(Math.abs(mirrored[0].position.y - hits[0].position.y) < 1e-5);
  assert.ok(Math.abs(mirrored[0].position.z - hits[0].position.z) < 1e-5);
  on.configure({ mirroredDisplay: false });
  // With world points disabled the original takes the feature set from a closed executor
  // (featureSetExecutor), which is UNVERIFIED against the original: no expectation is fixed here.
  assert.ok(Array.isArray(off.xrHitTest(0.5, 0.5, ['FEATURE_POINT'])));

  // trackingStatusChanged on real data: true exactly where (status, reason) differs from the
  // previous response (initial UNSPECIFIED/UNSPECIFIED). This clip is NORMAL throughout, so it has
  // only the first transition (observed 1 in 80 frames); transitions back and forth are covered
  // with scripted statuses by xr_api_test.cc TrackingStatusEvent.RaisedOnlyWhenStatusOrReasonChanges.
  let prev = ['UNSPECIFIED', 'UNSPECIFIED'];
  let transitions = 0;
  for (const [status, reason, changed] of responses) {
    const want = status !== prev[0] || reason !== prev[1];
    assert.equal(changed, want, `trackingStatusChanged for ${status}/${reason}`);
    if (want) transitions++;
    prev = [status, reason];
  }
  assert.ok(transitions >= 1, `only ${transitions} status transitions in the clip`);
  assert.ok(responses.length > 10, 'enough processed responses');
  t.diagnostic(`max world points ${maxOn}, hit ${JSON.stringify(hits[0])}, transitions ${transitions}`);
  on.delete();
  off.delete();
});

// ---- RM-3a2 (FULL-7): non-finite origin / facing, real status transitions ----

test('origin and facing must be finite (constructor and updateCameraProjectionMatrix)', async () => {
  const Module = await require(path.join(wasmDir, 'slam_refact_engine.js'))();
  const cfg = { captureWidth: W, captureHeight: H, manufacturer: 'Google', model: 'Pixel 7' };
  for (const [origin, what] of [[[0, NaN, 0], /origin\[1\] must be finite/], [[NaN, 2, 0], /origin\[0\]/],
    [[0, 2, Infinity], /origin\[2\]/], [[0, -Infinity, 0], /origin\[1\]/]]) {
    assert.throws(() => new Module.SlamRefactGlue({ ...cfg, origin }), what);
  }
  assert.throws(() => new Module.SlamRefactGlue({ ...cfg, facing: [1, 0, NaN, 0] }), /facing\[2\] must be finite/);
  const g = new Module.SlamRefactGlue({ ...cfg, origin: [0, 0, 0] });  // origin.y = 0 stays valid
  assert.throws(() => g.updateCameraProjectionMatrix({ origin: { x: 0, y: NaN, z: 0 } }), /origin\.y must be finite/);
  assert.throws(() => g.updateCameraProjectionMatrix({ origin: { x: Infinity, y: 2, z: 0 } }), /origin\.x/);
  assert.throws(() => g.updateCameraProjectionMatrix({ facing: { w: 1, x: 0, y: 0, z: NaN } }), /facing\.z/);
  // RM-3c (API-2 review MINOR-1): finite doubles that are not finite float32 values used to pass the
  // double check and abort the module (SR_CHECK in the glue) after the cast.
  assert.throws(() => new Module.SlamRefactGlue({ ...cfg, origin: [1e39, 2, 0] }), /origin\[0\] must be finite/);
  assert.throws(() => new Module.SlamRefactGlue({ ...cfg, facing: [1, 0, -1e39, 0] }), /facing\[2\] must be finite/);
  assert.throws(() => g.updateCameraProjectionMatrix({ origin: { x: 1e39, y: 2, z: 0 } }), /origin\.x must be finite/);
  assert.throws(() => g.updateCameraProjectionMatrix({ origin: { x: 0, y: -1e300, z: 0 } }), /origin\.y/);
  assert.throws(() => g.updateCameraProjectionMatrix({ facing: { w: 1, x: 0, y: 1e39, z: 0 } }), /facing\.y/);
  assert.throws(() => g.updateCameraProjectionMatrix({ cam: { nearClipPlane: 1e39 } }), /nearClipPlane/);
  g.updateCameraProjectionMatrix({ origin: { x: 3.4e38, y: 2, z: 3 } });  // the largest finite float32 magnitude is accepted
  g.updateCameraProjectionMatrix({ origin: { x: 1, y: 2, z: 3 } });  // still usable after the rejected calls
  g.delete();
});

// A1: a real clip with NORMAL <-> LIMITED transitions (the flip6-s2 clip above has only the first
// one). The expectation is the rule of the original JS (status or reason differs from the previous
// response); the count is a non-vacuity guard fixed from the native replay trace of this bundle
// (build/slam-refact-full-20261019/regress/rm3a/flip6-s3-lowtex__pmr3a: LIMITED at frame 71, NORMAL at
// 108, LIMITED at 198, NORMAL at 233), not a tolerance on the engine.
test('flip6-s3-lowtex bundle: trackingStatusChanged on real NORMAL/LIMITED transitions', async (t) => {
  const { readBundle, readPgm } = await import('./bundle_io.mjs');
  const { engineConfig } = await import('./bundle_replay.mjs');
  const fs = await import('node:fs');
  const bundleDir = process.env.SLAM_REFACT_FLIP6_LOWTEX_BUNDLE ??
    path.join(repo, 'build/slam-refact-20261016/fix/round2/bundles-seed/flip6-s3-lowtex/pmr3a');
  if (!fs.existsSync(path.join(bundleDir, 'frames.tsv'))) assert.fail(`BLOCKED: bundle ${bundleDir} not present`);
  const bundle = readBundle(bundleDir);
  const Module = await require(path.join(wasmDir, 'slam_refact_engine.js'))();
  const g = new Module.SlamRefactGlue(engineConfig(bundle.config));
  let next = 0;
  const push = (s) => {
    g.pushDeviceMotion(s.t, s.aig[0], s.aig[1], s.aig[2], s.lin !== null, ...(s.lin ?? [0, 0, 0]), s.rate[0], s.rate[1], s.rate[2], s.intervalMs);
    g.pushDeviceOrientation(s.t, s.orient[0], s.orient[1], s.orient[2]);
  };
  while (next < bundle.sensors.length && bundle.sensors[next].bootstrap) push(bundle.sensors[next++]);
  const boot = readPgm(bundle.frames[0].image);
  g.processFrame(bundle.frames[0].timeNs, bundle.frames[0].videoTimeNs, boot.pixels, boot.width, boot.height);
  let prev = ['UNSPECIFIED', 'UNSPECIFIED'];
  const transitions = [];
  let responses = 0;
  for (const f of bundle.frames.slice(1, 250)) {
    while (next < bundle.sensors.length && bundle.sensors[next].t <= f.timeNs) push(bundle.sensors[next++]);
    if (!f.process) continue;
    const img = readPgm(f.image);
    const p = g.processFrame(f.timeNs, f.videoTimeNs, img.pixels, img.width, img.height);
    responses++;
    const want = p.status !== prev[0] || p.reason !== prev[1];
    assert.equal(p.trackingStatusChanged, want, `frame ${f.index} ${p.status}/${p.reason}`);
    if (want) transitions.push([f.index, p.status]);
    prev = [p.status, p.reason];
  }
  t.diagnostic(`responses ${responses}, transitions ${JSON.stringify(transitions)}`);
  assert.ok(transitions.length >= 4, `only ${transitions.length} transitions`);
  assert.ok(transitions.some(([, s]) => s === 'LIMITED') && transitions.some(([, s]) => s === 'NORMAL'));
  g.delete();
});

// ---- RM-4b (FULL-7): optional DeviceMotion sensors, startup recenter counter ----

// The binding conversion of pushDeviceMotionEvent (RM-3c, API-2 review MAJOR-1). The IMU reaches the
// public output only through MapTracker's extreme-motion test (map_tracker.cc EventExceedsLimits): a
// GYROSCOPE event above MapTracker.maxGyroMagnitude (70) or a LINEAR_ACCELERATION event above
// MapTracker.maxLinearAccelerationMagnitude (1.7) turns the inertial frame-to-frame prediction off
// for that frame, which changes the published pose. The recorded flip6-s2 clip has neither (gyro and
// linear acceleration stay below the limits), so its output does not depend on the events (this is
// why the earlier full-vs-legacy comparison could not see any conversion error). The streams below
// add a limit-exceeding component to the linear acceleration or to the gyro; the comparison
// full(binding) vs legacy(pushDeviceMotion, which has no conversion) then pins:
//   which field becomes which kind    (acceleration -> LINEAR_ACCELERATION, rotationRate -> GYROSCOPE)
//   that a missing / null field adds no event of that kind
//   the value conversion              (numbers given as strings go through Number())
// What the output cannot see: the ORDER of alpha / beta / gamma and of x / y / z (both tests use the
// vector magnitude only), the value of interval, and accelerationIncludingGravity (no limit). Those
// parts of the conversion stay UNVERIFIED by output here; they are only checked to be accepted
// (below) and by reading (spec 06 section 6.1).
test('pushDeviceMotionEvent: the binding conversion is pinned by IMU-dependent output', async () => {
  const { readBundle, readPgm } = await import('./bundle_io.mjs');
  const { engineConfig } = await import('./bundle_replay.mjs');
  const fs = await import('node:fs');
  const bundleDir = process.env.SLAM_REFACT_FLIP6_BUNDLE ??
    path.join(repo, 'build/slam-refact-20261015/harness/bundles/flip6-s2/wp5b-r1');
  if (!fs.existsSync(path.join(bundleDir, 'frames.tsv'))) assert.fail(`BLOCKED: bundle ${bundleDir} not present`);
  const bundle = readBundle(bundleDir);
  const Module = await require(path.join(wasmDir, 'slam_refact_engine.js'))();
  const FRAMES = 45;  // the effect of the stressed streams starts at frame 18-25 (measured)
  const vec = (v) => ({ x: v[0], y: v[1], z: v[2] });
  // Streams: per recorded sample the values (lin may be null), plus a limit-exceeding component.
  const streams = {
    base: (s) => ({ lin: s.lin, aig: s.aig, rate: s.rate }),
    stressLin: (s) => ({ lin: (s.lin ?? [0, 0, 0]).map((v, i) => v + (i === 0 ? 3 : 0)), aig: s.aig, rate: s.rate }),
    stressGyro: (s) => ({ lin: s.lin, aig: s.aig, rate: s.rate.map((v, i) => v + (i === 0 ? 100 : 0)) }),
  };
  const legacyPush = (g, s, v, swap = false) => (swap
    ? g.pushDeviceMotion(s.t, ...(v.lin ?? [0, 0, 0]), true, ...v.aig, ...v.rate, s.intervalMs)
    : g.pushDeviceMotion(s.t, ...v.aig, v.lin !== null, ...(v.lin ?? [0, 0, 0]), ...v.rate, s.intervalMs));
  const eventOf = (s, v) => ({
    acceleration: v.lin ? vec(v.lin) : null,
    accelerationIncludingGravity: vec(v.aig),
    rotationRate: { alpha: v.rate[0], beta: v.rate[1], gamma: v.rate[2] },
    interval: s.intervalMs,
  });
  const pushers = {
    legacy: (g, s, v) => legacyPush(g, s, v),
    full: (g, s, v) => g.pushDeviceMotionEvent(s.t, eventOf(s, v)),
    // lin / aig exchanged through the legacy signature = what a binding that swaps the two would do.
    legacySwapped: (g, s, v) => legacyPush(g, s, v, true),
    noGyro: (g, s, v) => g.pushDeviceMotionEvent(s.t, { ...eventOf(s, v), rotationRate: null }),
    noLin: (g, s, v) => g.pushDeviceMotionEvent(s.t, { ...eventOf(s, v), acceleration: null }),
    linAsStrings: (g, s, v) => g.pushDeviceMotionEvent(s.t, { ...eventOf(s, v),
      acceleration: v.lin ? { x: String(v.lin[0]), y: String(v.lin[1]), z: String(v.lin[2]) } : null }),
  };
  const run = (pusher, stream) => {
    const g = new Module.SlamRefactGlue(engineConfig(bundle.config));
    let next = 0;
    const push = (s) => { pushers[pusher](g, s, streams[stream](s)); g.pushDeviceOrientation(s.t, s.orient[0], s.orient[1], s.orient[2]); };
    while (next < bundle.sensors.length && bundle.sensors[next].bootstrap) push(bundle.sensors[next++]);
    const boot = readPgm(bundle.frames[0].image);
    g.processFrame(bundle.frames[0].timeNs, bundle.frames[0].videoTimeNs, boot.pixels, boot.width, boot.height);
    const poses = [];
    for (const f of bundle.frames.slice(1, FRAMES + 1)) {
      while (next < bundle.sensors.length && bundle.sensors[next].t <= f.timeNs) push(bundle.sensors[next++]);
      if (!f.process) continue;
      const img = readPgm(f.image);
      const p = g.processFrame(f.timeNs, f.videoTimeNs, img.pixels, img.width, img.height);
      poses.push(JSON.stringify([p.position, p.rotationWxyz, p.status, p.reason, p.worldPoints]));
    }
    g.delete();
    return poses;
  };
  const differing = (a, b) => a.reduce((n, x, i) => n + (x === b[i] ? 0 : 1), 0);
  const legacy = Object.fromEntries(Object.keys(streams).map((k) => [k, run('legacy', k)]));
  assert.ok(legacy.base.length > 30);
  // Power of the scenario (non-vacuity): the stressed streams and the swapped kinds change the output.
  const POWER = 5;  // measured 20 / 27 / 20 differing frames of 45
  assert.ok(differing(legacy.stressLin, legacy.base) >= POWER, 'a linear acceleration above the limit changes the output');
  assert.ok(differing(legacy.stressGyro, legacy.base) >= POWER, 'a gyro above the limit changes the output');
  assert.ok(differing(run('legacySwapped', 'base'), legacy.base) >= POWER, 'linear acceleration and accelerationIncludingGravity are distinguishable');
  // The binding: a full event is the bit-identical engine input of the legacy call, in every stream.
  for (const k of Object.keys(streams)) {
    assert.deepEqual(run('full', k), legacy[k], `a full event gives the bit-identical engine input (${k})`);
  }
  // Missing / null fields add no event of that kind: without the exceeding field the output is the
  // base output again (the base events themselves are below the limits), and differs from the stream.
  const noGyro = run('noGyro', 'stressGyro');
  assert.deepEqual(noGyro, legacy.base, 'rotationRate: null adds no GYROSCOPE event');
  assert.ok(differing(noGyro, legacy.stressGyro) >= POWER);
  const noLin = run('noLin', 'stressLin');
  assert.deepEqual(noLin, legacy.base, 'acceleration: null adds no LINEAR_ACCELERATION event');
  assert.ok(differing(noLin, legacy.stressLin) >= POWER);
  // Number() semantics of the capnp float setters: numeric strings convert. (A binding that dropped the
  // explicit Number(...) and let embind coerce the value gives the same numbers, so this pins the
  // result, not the call: that mutant is equivalent, see evidence/RM-3c/binding-mutants.log.)
  assert.deepEqual(run('linAsStrings', 'stressLin'), legacy.stressLin, 'numeric strings convert like Number(...)');
  // accelerationIncludingGravity has no consumer limit: a missing one is accepted and the output stays finite.
  const noGravity = await (async () => {
    const g = new Module.SlamRefactGlue(engineConfig(bundle.config));
    let next = 0;
    const push = (s) => {
      g.pushDeviceMotionEvent(s.t, { ...eventOf(s, streams.base(s)), accelerationIncludingGravity: undefined });
      g.pushDeviceOrientation(s.t, s.orient[0], s.orient[1], s.orient[2]);
    };
    while (next < bundle.sensors.length && bundle.sensors[next].bootstrap) push(bundle.sensors[next++]);
    const boot = readPgm(bundle.frames[0].image);
    g.processFrame(bundle.frames[0].timeNs, bundle.frames[0].videoTimeNs, boot.pixels, boot.width, boot.height);
    let last;
    for (const f of bundle.frames.slice(1, 31)) {
      while (next < bundle.sensors.length && bundle.sensors[next].t <= f.timeNs) push(bundle.sensors[next++]);
      if (!f.process) continue;
      const img = readPgm(f.image);
      last = g.processFrame(f.timeNs, f.videoTimeNs, img.pixels, img.width, img.height);
    }
    g.delete();
    return last;
  })();
  assert.ok(noGravity.position.every(Number.isFinite) && noGravity.status === 'NORMAL');
  const g = await makeGlue();
  // JS Number() semantics of the capnp float setters: null -> 0.
  g.pushDeviceMotionEvent(1e9, { accelerationIncludingGravity: { x: null, y: 9.81, z: 0 }, rotationRate: { alpha: 0, beta: 0, gamma: 0 }, interval: 16.666 });
  g.pushDeviceMotionEvent(1e9, {});          // no sensor at all: nothing is pushed, nothing throws
  g.pushDeviceMotionEvent(1e9, { interval: undefined, rotationRate: { alpha: 1, beta: 2, gamma: 3 } });  // NaN interval -> 0 ns
  // RM-3c (API-2 review MINOR-2): the interval is stored only with a sensor; without one any value is a no-op.
  g.pushDeviceMotionEvent(1e9, { interval: Infinity });
  g.pushDeviceMotionEvent(1e9, { interval: 1e300, acceleration: null });
  // With a sensor an interval outside the Int64 nanosecond range is a JS error (the original throws a
  // RangeError in setEventQueue; the engine used to abort the module here).
  assert.throws(() => g.pushDeviceMotionEvent(1e9, { interval: Infinity, rotationRate: { alpha: 1, beta: 2, gamma: 3 } }), /interval/);
  assert.throws(() => g.pushDeviceMotionEvent(1e9, { interval: -1e300, accelerationIncludingGravity: { x: 0, y: 9.81, z: 0 } }), /interval/);
  assert.throws(() => g.pushDeviceMotion(1e9, 0, 9.81, 0, false, 0, 0, 0, 1, 2, 3, Infinity), /interval/);
  g.pushDeviceMotionEvent(1e9, { interval: 16.666, rotationRate: { alpha: 1, beta: 2, gamma: 3 } });  // still usable
  assert.throws(() => g.pushDeviceMotionEvent(1e9, null), /event object/);
  assert.throws(() => g.pushDeviceMotionEvent(1e9, 5), /event object/);
  assert.throws(() => g.pushDeviceMotionEvent(NaN, {}), /finite/);
  g.delete();
});

test('framesUntilRecenter: counter, accessor and validation', async () => {
  const Module = await require(path.join(wasmDir, 'slam_refact_engine.js'))();
  const base = { captureWidth: W, captureHeight: H, manufacturer: 'Google', model: 'Pixel 7' };
  const dflt = new Module.SlamRefactGlue(base);
  assert.equal(dflt.getFramesUntilRecenter(), 5);
  const zero = new Module.SlamRefactGlue({ ...base, framesUntilRecenter: 0 });
  const three = new Module.SlamRefactGlue({ ...base, framesUntilRecenter: 3 });
  assert.equal(zero.getFramesUntilRecenter(), 0);
  const seen = { dflt: [], zero: [], three: [] };
  for (let k = 0; k < 8; k++) {
    step(dflt, k); step(zero, k); step(three, k);
    seen.dflt.push(dflt.getFramesUntilRecenter());
    seen.zero.push(zero.getFramesUntilRecenter());
    seen.three.push(three.getFramesUntilRecenter());
  }
  // Frame 0 only fills the pipeline; frames 1.. are processed.
  assert.deepEqual(seen.dflt, [5, 4, 3, 2, 1, 0, 0, 0]);
  assert.deepEqual(seen.three, [3, 2, 1, 0, 0, 0, 0, 0]);
  assert.deepEqual(seen.zero, [0, 0, 0, 0, 0, 0, 0, 0]);
  for (const bad of [-1, 1.5, NaN, Infinity, 2e6]) {
    assert.throws(() => new Module.SlamRefactGlue({ ...base, framesUntilRecenter: bad }), /framesUntilRecenter/);
  }
  dflt.delete(); zero.delete(); three.delete();
});

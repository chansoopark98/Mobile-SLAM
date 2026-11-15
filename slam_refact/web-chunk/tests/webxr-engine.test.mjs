// samples/webxr-engine: trajectory alignment (synthetic known SE(3) / Sim(3)), quaternion conventions, IMU shape conversion, camera crop /
// K math for 785x1920, texture orientation check, served import paths, and the glue config / IMU shapes against the deployed engine WASM.
import test from 'node:test';
import assert from 'node:assert/strict';
import fs from 'node:fs';
import path from 'node:path';
import { createRequire } from 'node:module';
import { fileURLToPath } from 'node:url';
import { compareTrajectories, jacobiEigen, matToQ, matVec, pairsFromRecords, qangle, qconj, qmul, qnormalize, qToMat, relativeErrors,
  umeyama, xyzwToWxyz } from '../samples/webxr-engine/trajectory-align.js';
import { buildGlueConfig, imageOrientationCheck, intrinsicsFromXrProjection, level0Size, mapK, outputToSource, planEngineInput,
  relativeRotationVector } from '../samples/webxr-engine/xr-camera.js';
import { RAD_TO_DEG, chooseImuSource, createGenericMotionFuser, deviceOrientationToQuat, gyroToRotationRate, quatToDeviceOrientation, rateStats }
  from '../samples/webxr-engine/imu-sources.js';
import { pushMotion } from '../sensors.js';
import { resolveSamplePath, DEFAULT_ENGINE_DIR } from '../samples/serve.mjs';

const here = path.dirname(fileURLToPath(import.meta.url));
const near = (a, b, tol, msg) => assert.ok(Math.abs(a - b) <= tol, `${msg ?? ''} ${a} vs ${b} (tol ${tol})`);
function mulberry32(seed) {
  let a = seed >>> 0;
  return () => { a = (a + 0x6d2b79f5) >>> 0; let t = a; t = Math.imul(t ^ (t >>> 15), t | 1); t ^= t + Math.imul(t ^ (t >>> 7), t | 61); return ((t ^ (t >>> 14)) >>> 0) / 4294967296; };
}
const axisAngle = (axis, deg) => { const n = Math.hypot(...axis), h = deg * Math.PI / 360; return [Math.cos(h), ...axis.map((v) => v / n * Math.sin(h))]; };

// Reference trajectory: 20 s at 30 Hz, a 3D loop with turning camera.
function refTrajectory() {
  const out = [];
  for (let k = 0; k < 600; k++) {
    const t = k / 30;
    out.push({ t, p: [Math.cos(t * 0.5) * 1.5, 0.3 * Math.sin(t * 1.3), Math.sin(t * 0.5) * 1.0],
      q: qnormalize(qmul(axisAngle([0, 1, 0], 20 * t), axisAngle([1, 0.2, 0], 10 * Math.sin(t)))) });
  }
  return out;
}
// Engine = inverse of a known world transform (ref = s R eng + t), optional body offset.
function engineFrom(ref, { R, q, s = 1, t = [0, 0, 0], body = [1, 0, 0, 0] }) {
  const Rt = qToMat(qconj(q));
  return ref.map((r) => ({ t: r.t, eng: { p: matVec(Rt, [r.p[0] - t[0], r.p[1] - t[1], r.p[2] - t[2]]).map((v) => v / s), q: qmul(qmul(qconj(q), r.q), body) }, ref: r }));
}

test('quaternion conventions: wxyz, matrix round trip, WebXR xyzw, rotation of a vector', () => {
  const q = qnormalize([0.9, 0.1, -0.3, 0.2]);
  const R = qToMat(q);
  const back = matToQ(R);
  for (let i = 0; i < 4; i++) near(back[i], q[i], 1e-12);
  assert.deepEqual(xyzwToWxyz({ x: 1, y: 2, z: 3, w: 4 }), [4, 1, 2, 3]);
  // 90 deg about +y (GL up) takes the GL forward -z to -x (turning left).
  const v = matVec(qToMat(axisAngle([0, 1, 0], 90)), [0, 0, -1]);
  near(v[0], -1, 1e-12); near(v[2], 0, 1e-12);
  near(qangle(qmul(qconj(q), q)), 0, 1e-12);
  near(qangle(axisAngle([1, 1, 0], 33)) * 180 / Math.PI, 33, 1e-9);
  const { values } = jacobiEigen([[4, 1, 0], [1, 3, 0], [0, 0, 1]]);
  near(maxOf(values), (7 + Math.sqrt(5)) / 2, 1e-12);
});

test('umeyama / compareTrajectories: exact SE(3) recovers the transform with ~0 ATE and RPE', () => {
  const q = axisAngle([0.3, 1, -0.2], 70), R = qToMat(q), t = [2, -1, 0.5];
  const pairs = engineFrom(refTrajectory(), { R, q, t });
  const al = umeyama(pairs.map((p) => p.eng.p), pairs.map((p) => p.ref.p), false);
  near(qangle(qmul(qconj(al.q), q)), 0, 1e-9, 'rotation');
  for (let i = 0; i < 3; i++) near(al.t[i], t[i], 1e-9, 'translation');
  const rep = compareTrajectories(pairs);
  assert.match(rep.label, /not ground truth/);
  assert.equal(rep.degenerate, false);
  near(rep.se3.ate.trans.rmse, 0, 1e-9); near(rep.se3.ate.rotDeg.max, 0, 1e-5);
  near(rep.se3.rpe.trans.rmse, 0, 1e-9); near(rep.se3.rpe.rotDeg.rmse, 0, 1e-5);
  assert.ok(rep.se3.rpe.trans.n > 500, `rpe pairs ${rep.se3.rpe.trans.n}`);
  near(rep.sim3.transform.scale, 1, 1e-9);
});

test('scale error: SE(3) ATE shows it, Sim(3) recovers the scale as a separate diagnostic', () => {
  const q = axisAngle([0, 1, 0], -40);
  const pairs = engineFrom(refTrajectory(), { R: qToMat(q), q, s: 2, t: [0.1, 0.2, 0.3] });  // engine is half size
  const rep = compareTrajectories(pairs);
  assert.ok(rep.se3.ate.trans.rmse > 0.2, `se3 ate ${rep.se3.ate.trans.rmse}`);
  near(rep.sim3.transform.scale, 2, 1e-9);
  near(rep.sim3.ate.trans.rmse, 0, 1e-9);
  near(rep.sim3.rpe.trans.rmse, 0, 1e-9);
  assert.ok(rep.se3.rpe.trans.rmse > 0.01);
  near(rep.se3.ate.rotDeg.rmse, 0, 1e-5, 'rotation unaffected by scale');
});

test('constant body-frame offset is reported as rotation error, with the offset diagnostic', () => {
  const q = axisAngle([1, 0, 0], 15);
  const pairs = engineFrom(refTrajectory(), { R: qToMat(q), q, body: axisAngle([0, 0, 1], 10) });
  const rep = compareTrajectories(pairs);
  near(rep.se3.ate.rotDeg.median, 10, 1e-6);
  near(rep.se3.ate.bodyOffsetDiag.angleDeg, 10, 1e-6);
  near(rep.se3.ate.bodyOffsetDiag.spreadDeg.max, 0, 1e-5);
});

test('noise, pairing from records, coverage, gaps and degenerate geometry', () => {
  const rnd = mulberry32(7);
  const q = axisAngle([0, 1, 0], 25);
  const pairs = engineFrom(refTrajectory(), { R: qToMat(q), q });
  const records = pairs.map((p, k) => ({ xrTime: p.t * 1000, ref: { p: p.ref.p, q: p.ref.q },
    engine: k % 10 === 0 ? { status: 'LIMITED', p: p.eng.p, q: p.eng.q } : { status: 'NORMAL', p: p.eng.p.map((v) => v + (rnd() - 0.5) * 0.02), q: p.eng.q } }));
  records.push({ xrTime: 1e6, ref: { p: [0, 0, 0], q: [1, 0, 0, 0] }, engine: null });
  const paired = pairsFromRecords(records);
  assert.equal(paired.length, 540);
  const rep = compareTrajectories(paired, { refFrames: records.filter((r) => r.ref).length });
  near(rep.coverage, 540 / 601, 1e-12);
  assert.ok(rep.se3.ate.trans.rmse > 0.003 && rep.se3.ate.trans.rmse < 0.012, `noisy ate ${rep.se3.ate.trans.rmse}`);
  // a 2 s gap: no RPE window spans it
  const gap = paired.filter((p) => p.t < 5 || p.t > 7);
  const rpe = relativeErrors(gap, { delta: 1, tol: 0.1 });
  assert.ok(rpe.trans.n < relativeErrors(paired).trans.n);
  // a line: degenerate
  const line = paired.map((p) => ({ ...p, ref: { ...p.ref, p: [p.t * 0.1, 0, 0] }, eng: { ...p.eng, p: [p.t * 0.1, 0, 0] } }));
  assert.equal(compareTrajectories(line).degenerate, true);
  assert.match(compareTrajectories(paired.slice(0, 5)).insufficient, /only 5/);
  assert.equal(pairsFromRecords([{ xrTime: 0, ref: { p: [0, 0, NaN], q: [1, 0, 0, 0] }, engine: { status: 'NORMAL', p: [0, 0, 0], q: [1, 0, 0, 0] } }]).length, 0);
});

// Column-major GL projection from pixel-EDGE intrinsics (inverse of the probe's intrinsicsFromProjection).
function projectionFrom(fx, fy, cxEdge, cyEdge, W, H, n = 0.1, f = 100) {
  const P = new Float32Array(16);
  P[0] = (2 * fx) / W; P[5] = (2 * fy) / H; P[8] = 1 - (2 * cxEdge) / W; P[9] = (2 * cyEdge) / H - 1;
  P[10] = -(f + n) / (f - n); P[11] = -1; P[14] = (-2 * f * n) / (f - n);
  return P;
}

test('crop / K for the measured 785x1920 camera image: 261x640 capture == level 0, K / 3 with the half-pixel shift', () => {
  // measured (p0-c2-eis-result.md): fx 1275.6, fy 1276.3, cx 392.5, cy 960 in edge coordinates at 785x1920
  const K = intrinsicsFromXrProjection(projectionFrom(1275.6, 1276.3, 392.5, 960, 785, 1920), 785, 1920);
  near(K.fx, 1275.6, 1e-3); near(K.cx, 392, 1e-4); near(K.cy, 959.5, 1e-4);
  assert.deepEqual(level0Size(785, 1920), { w: 261, h: 640 });  // what the engine would stretch the raw image to (VERIFIED on WASM below)
  const plan = planEngineInput(785, 1920, K);
  assert.deepEqual(plan.output, { width: 261, height: 640 });
  assert.deepEqual(plan.crop, { x0: 1, y0: 0, width: 783, height: 1920 });
  assert.equal(plan.footprint, 3); assert.equal(plan.integerFootprint, true);
  assert.deepEqual(plan.level0, { w: 261, h: 640 }, 'capture equals level 0, so the override is the processing K at the right size');
  near(plan.K.fx, 425.2, 1e-3); near(plan.K.fy, 1276.3 / 3, 1e-3);
  near(plan.K.cx, 130, 1e-4, 'centred'); near(plan.K.cy, 319.5, 1e-4, 'centred');
  // projection consistency: a ray projected with K, mapped through the crop/decimation, equals the projection with K'
  for (const [x, y, z] of [[0.1, -0.3, 1], [-0.2, 0.6, 1.3], [0.12, 0.7, 0.9]]) {
    const u = K.fx * x / z + K.cx, v = K.fy * y / z + K.cy;
    const uo = (u + 0.5 - plan.crop.x0) / plan.footprint - 0.5, vo = (v + 0.5 - plan.crop.y0) / plan.footprint - 0.5;
    near(plan.K.fx * x / z + plan.K.cx, uo, 1e-9); near(plan.K.fy * y / z + plan.K.cy, vo, 1e-9);
    const [us, vs] = outputToSource(plan, uo, vo);
    near(us, u, 1e-9); near(vs, v, 1e-9);
  }
  // output pixel 0 covers source columns 1..3 (index), centre 2; last covers 781..783
  assert.deepEqual(outputToSource(plan, 0, 0), [2, 1]);
  assert.deepEqual(outputToSource(plan, 260, 639), [782, 1918]);
  // 3:4 mode: centre crop 785 x 1046.67, 480 x 640 output, same angular K
  const p34 = planEngineInput(785, 1920, K, { mode: '34' });
  assert.deepEqual(p34.output, { width: 480, height: 640 });
  near(p34.crop.height, 785 * 4 / 3, 1e-9); near(p34.K.fx / p34.K.fy, K.fx / K.fy, 1e-12);
  near(p34.K.cy, 319.5, 1e-6); near(p34.K.cx, 239.5, 1e-6);
  assert.throws(() => planEngineInput(1920, 785, K), /landscape/);
  // mapK is the same map for any K
  assert.deepEqual(mapK({ fx: 3, fy: 3, cx: 2, cy: 1 }, plan), { fx: 1, fy: 1, cx: 0, cy: 0, width: 261, height: 640 });
  const cfg = buildGlueConfig({ plan, model: 'SM-F741N', disableVio: true, estimateScale: true });
  assert.equal(cfg.estimateScale, false, 'no scale estimation without VIO');
  assert.deepEqual([cfg.captureWidth, cfg.captureHeight, cfg.rotationDeg], [261, 640, 0]);
});

test('level0Size matches the engine float rule (3:4 clamp only when wider)', () => {
  assert.deepEqual(level0Size(480, 640), { w: 480, h: 640 });
  assert.deepEqual(level0Size(1440, 1920), { w: 480, h: 640 });
  assert.deepEqual(level0Size(720, 1280), { w: 360, h: 640 });
  assert.deepEqual(level0Size(1080, 1080), { w: 480, h: 480 });  // clamped: wider than 3:4 (the stretch case a 3:4 crop would fix)
  for (const w of [255, 258, 260, 261, 262, 263]) assert.equal(level0Size(w, 640).w, w);
});

test('texture orientation check: GL-convention texture (row 0 = bottom) -> flipY, upright -> no flip, noise -> inconclusive', () => {
  const rnd = mulberry32(3);
  const mk = (sy, sx = 1) => Array.from({ length: 80 }, () => {
    const theta = [(rnd() - 0.5) * 0.04, (rnd() - 0.5) * 0.04, (rnd() - 0.5) * 0.01];
    return { dx: sx * 50 * theta[1] + (rnd() - 0.5) * 0.2, dy: sy * 52 * theta[0] + (rnd() - 0.5) * 0.2, ok: true, theta };
  });
  const flipped = imageOrientationCheck(mk(-1), 50, 52);
  assert.equal(flipped.flipY, true); assert.equal(flipped.flipX, false); assert.equal(flipped.conclusive, true);
  near(flipped.y.slope, -1, 0.05);
  const upright = imageOrientationCheck(mk(1), 50, 52);
  assert.equal(upright.flipY, false);
  assert.equal(imageOrientationCheck(mk(1, -1), 50, 52).flipX, true);
  const noise = Array.from({ length: 80 }, () => ({ dx: rnd() - 0.5, dy: rnd() - 0.5, ok: true, theta: [(rnd() - 0.5) * 0.04, (rnd() - 0.5) * 0.04, 0] }));
  assert.equal(imageOrientationCheck(noise, 50, 52).flipY, null);
  assert.equal(imageOrientationCheck(mk(1).slice(0, 5), 50, 52).conclusive, false);
  // relativeRotationVector: body-frame increment of q_prev -> q_prev * dq
  const qp = axisAngle([0.2, 1, 0.1], 50), dq = axisAngle([1, 0, 0], 2);
  const w = relativeRotationVector(qp, qmul(qp, dq));
  near(w[0], 2 * Math.PI / 180, 1e-12); near(w[1], 0, 1e-12); near(w[2], 0, 1e-12);
});

test('IMU shapes: Generic Sensor -> DeviceMotion entries, units and axes, 5e9 ns rule, pushMotion call', () => {
  const rr = gyroToRotationRate({ x: 1, y: -0.5, z: 0.25 });
  near(rr.alpha, 0.25 * 180 / Math.PI, 1e-12); near(rr.beta, 180 / Math.PI, 1e-12); near(rr.gamma, -0.5 * 180 / Math.PI, 1e-12);
  const f = createGenericMotionFuser({ frequency: 60 });
  f.onGyroscope({ x: 0.1, y: 0, z: 0, timestamp: 1000 });  // before any accelerometer reading
  f.onAccelerometer({ x: 0, y: 9.8, z: 0.1, timestamp: 1001 });
  f.onLinearAcceleration({ x: 0.01, y: 0.02, z: 0.03, timestamp: 1001 });
  f.onGyroscope({ x: 0, y: 0.2, z: 0, timestamp: 1016.5 });
  f.onGyroscope({ x: 0, y: 0.2, z: 0, timestamp: 1016.5 });  // repeated timestamp: dropped
  const e = f.take();
  assert.equal(e.length, 2); assert.equal(f.stats.nonMonotonic, 1); assert.equal(f.take().length, 0);
  assert.deepEqual(e[0], { timestampNanos: 1e9, interval: 16, acceleration: null, accelerationIncludingGravity: null,
    rotationRate: { alpha: 0, beta: 0.1 * RAD_TO_DEG, gamma: 0 } });
  assert.equal(e[1].timestampNanos, 1016.5e6);
  assert.deepEqual(e[1].accelerationIncludingGravity, { x: 0, y: 9.8, z: 0.1 });
  assert.deepEqual(e[1].acceleration, { x: 0.01, y: 0.02, z: 0.03 });
  const calls = [];
  const glue = { pushDeviceMotionEvent: (ns, ev) => calls.push([ns, ev]) };
  assert.equal(pushMotion(glue, e[1]), true);
  assert.deepEqual(calls[0], [1016.5e6, { acceleration: e[1].acceleration, accelerationIncludingGravity: e[1].accelerationIncludingGravity, rotationRate: e[1].rotationRate, interval: 16 }]);
  const g = createGenericMotionFuser({ frequency: 100 });
  g.onGyroscope({ x: 0, y: 0, z: 0, timestamp: 0 });
  g.onGyroscope({ x: 0, y: 0, z: 0, timestamp: 5001 });
  assert.equal(g.take().length, 1); assert.equal(g.stats.cleared, 1);
  assert.equal(createGenericMotionFuser({ frequency: 100 }).take().length, 0);
});

test('orientation quaternion -> DeviceOrientation angles round trip through the engine GetQuaternion, W3C ranges', () => {
  const rnd = mulberry32(11);
  for (let k = 0; k < 500; k++) {
    const a = rnd() * 360, b = rnd() * 360 - 180, g = rnd() * 180 - 90;
    const [w, x, y, z] = deviceOrientationToQuat(a, b, g);
    const o = quatToDeviceOrientation([x, y, z, w]);
    assert.ok(o.alpha >= 0 && o.alpha < 360 && o.beta >= -180 && o.beta < 180 && o.gamma >= -90 && o.gamma < 90, JSON.stringify(o));
    const q2 = deviceOrientationToQuat(o.alpha, o.beta, o.gamma);
    near(qangle(qmul(qconj([w, x, y, z]), q2)), 0, 1e-6, `angles ${a} ${b} ${g} -> ${JSON.stringify(o)}`);
  }
  const flat = quatToDeviceOrientation([0, 0, 0, 1]);
  assert.deepEqual([flat.alpha, flat.beta, flat.gamma].map((v) => Math.abs(v)), [0, 0, 0]);
  const upright = quatToDeviceOrientation([Math.sin(Math.PI / 4), 0, 0, Math.cos(Math.PI / 4)]);  // 90 deg about x: phone upright
  near(upright.beta, 90, 1e-9);
});

test('rate statistics and IMU source choice', () => {
  const s = rateStats(Array.from({ length: 61 }, (_, i) => i * 16.6667), 1000);
  near(s.rateHz, 61, 1e-9); near(s.dtP50Ms, 16.6667, 1e-6);
  assert.equal(rateStats([], 5000).rateHz, 0);
  const rep = (dm, gyro, acc, dor = 0, rel = 0) => ({ devicemotion: { rateHz: dm, withRotationRate: dm ? 1 : 0 }, gyroscope: { rateHz: gyro },
    accelerometer: { rateHz: acc }, deviceorientation: { rateHz: dor }, relativeOrientation: { rateHz: rel }, absoluteOrientation: { rateHz: 0 } });
  assert.deepEqual(chooseImuSource(rep(0, 60, 60, 0, 60)).motion, 'generic');
  assert.equal(chooseImuSource(rep(0, 60, 60, 0, 60)).orientation, 'relative');
  assert.equal(chooseImuSource(rep(59, 20, 20, 59)).motion, 'devicemotion');
  assert.equal(chooseImuSource(rep(0, 60, 0)).motion, 'none', 'gyro without accelerometer is not enough');
  const none = chooseImuSource(rep(0, 0, 0));
  assert.equal(none.disableVio, true); assert.equal(none.orientation, 'none');
  assert.equal(chooseImuSource(rep(0, 0, 0), { force: 'generic' }).motion, 'generic');
  assert.equal(chooseImuSource({ devicemotion: { rateHz: 60, withRotationRate: 0 }, gyroscope: { rateHz: 0 }, accelerometer: { rateHz: 0 } }).motion, 'none',
    'DeviceMotion events without rotationRate do not count');
});

test('served paths: every module the page imports resolves through serve.mjs to an existing file that exports the imported names', async () => {
  const dir = path.join(here, '../samples/webxr-engine');
  const html = fs.readFileSync(path.join(dir, 'index.html'), 'utf8');
  assert.match(html, /<script type="module" src="app\.js">/);
  const files = ['app.js', 'xr-camera.js', 'imu-sources.js', 'trajectory-align.js'];
  for (const f of files) {
    const src = fs.readFileSync(path.join(dir, f), 'utf8');
    for (const m of src.matchAll(/^import \{([^}]*)\}\s*from '([^']+)'/gm)) {
      const url = new URL(m[2], `http://x/samples/webxr-engine/${f}`);
      const file = resolveSamplePath(url.pathname);
      assert.ok(file && fs.existsSync(file), `${f}: ${m[2]} -> ${url.pathname} -> ${file}`);
      const mod = await import(file);  // every dependency is DOM-free at import time
      for (const name of m[1].split(',').map((s) => s.trim()).filter(Boolean)) assert.ok(name in mod, `${f}: ${m[2]} does not export ${name}`);
    }
  }
  const engineUrl = new URL('../../engine/slam-refact-engine/manifest.json', 'http://x/samples/webxr-engine/app.js');
  assert.equal(engineUrl.pathname, '/engine/slam-refact-engine/manifest.json');
  assert.ok(resolveSamplePath(engineUrl.pathname));
  for (const f of files) assert.doesNotMatch(fs.readFileSync(path.join(dir, f), 'utf8'), /\bdebugger\b|console\.log\(|TODO|localStorage/, f);
});

const engineJs = path.join(DEFAULT_ENGINE_DIR, 'slam_refact_engine.js');
test('deployed engine WASM: plan capture is level 0, the glue takes the config, generic IMU entries and zero atlases', { skip: !fs.existsSync(engineJs) && 'no engine at the serve.mjs default dir' }, async () => {
  const Module = await createRequire(import.meta.url)(engineJs)();
  const K = intrinsicsFromXrProjection(projectionFrom(1275.6, 1276.3, 392.5, 960, 785, 1920), 785, 1920);
  // the raw camera size would be stretched to 261 x 640 (the reason for the plan)
  const raw = new Module.SlamRefactGlue({ captureWidth: 785, captureHeight: 1920, rotationDeg: 0 });
  assert.deepEqual({ w: raw.pyramidLayout().levels[0].w, h: raw.pyramidLayout().levels[0].h }, level0Size(785, 1920));
  raw.delete();
  for (const mode of ['full', '34']) {
    const plan = planEngineInput(785, 1920, K, { mode });
    const glue = new Module.SlamRefactGlue(buildGlueConfig({ plan, model: 'SM-F741N' }));
    try {
      const layout = glue.pyramidLayout();
      assert.equal(layout.levels[0].w, plan.output.width, mode); assert.equal(layout.levels[0].h, plan.output.height, mode);
      assert.equal(layout.captureWidth, plan.output.width);
      if (typeof glue.getCaptureIntrinsics === 'function') {
        const k = glue.getCaptureIntrinsics();
        near(k.fx, plan.K.fx, 1e-3); near(k.cx, plan.K.cx, 1e-3);
      }
      if (mode !== 'full') continue;
      glue.configure({ enableWorldPoints: true, enableLighting: false, leftHandedAxes: false, mirroredDisplay: false });
      const fuser = createGenericMotionFuser({ frequency: 60 });
      const atlas = new Uint8Array(layout.textureSize * layout.rows * 4);
      let ts = 1000;
      for (let k = 0; k < 8; k++) {
        for (let j = 0; j < 2; j++, ts += 16.6667) {
          fuser.onAccelerometer({ x: 0, y: 9.81, z: 0, timestamp: ts }); fuser.onGyroscope({ x: 0.01, y: 0, z: 0, timestamp: ts });
        }
        for (const e of fuser.take()) assert.equal(pushMotion(glue, e), true);
        const o = quatToDeviceOrientation([Math.sin(Math.PI / 4), 0, 0, Math.cos(Math.PI / 4)]);
        glue.pushDeviceOrientation(ts * 1e6, o.alpha, o.beta, o.gamma);
        const pose = glue.processFramePyramid(Math.round(ts * 1e6), Math.round(ts * 1e6), atlas, layout.textureSize, layout.rows);
        assert.equal(pose.position.length, 3); assert.equal(pose.rotationWxyz.length, 4);
        assert.equal(typeof pose.status, 'string'); assert.equal(typeof pose.worldPoints, 'number');
        assert.equal(pose.videoFrameIndex, k);
      }
    } finally { glue.delete(); }
  }
});

// Iterative max/min (a spread into Math.max/min throws RangeError on large arrays and gives -/+Infinity on empty ones). Empty input -> null.
function maxOf(xs) { if (!xs.length) return null; let m = xs[0]; for (const x of xs) if (x > m || Number.isNaN(x)) m = x; return m; }
function minOf(xs) { if (!xs.length) return null; let m = xs[0]; for (const x of xs) if (x < m || Number.isNaN(x)) m = x; return m; }

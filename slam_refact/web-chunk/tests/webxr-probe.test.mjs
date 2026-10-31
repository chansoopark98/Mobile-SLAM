// samples/webxr-probe/analysis.js: intrinsics from a known projection matrix, SAD shift estimator on synthetic shifted images,
// Welch coherence (identical -> ~1, independent -> low), and the end-to-end EIS comparison on synthetic shake data.
import test from 'node:test';
import assert from 'node:assert/strict';
import fs from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';
import {
  intrinsicsFromProjection, percentile, fpsStats, estimateShiftSAD, welchCoherence, bandStats, resampleUniform, intervalMean,
  angularVelocityBetween, analyzeRun,
} from '../samples/webxr-probe/analysis.js';
import { resolveSamplePath } from '../samples/serve.mjs';

const here = path.dirname(fileURLToPath(import.meta.url));

function mulberry32(seed) {
  let a = seed >>> 0;
  return () => {
    a = (a + 0x6d2b79f5) >>> 0;
    let t = a;
    t = Math.imul(t ^ (t >>> 15), t | 1);
    t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  };
}
const gauss = (rnd) => Math.sqrt(-2 * Math.log(1 - rnd())) * Math.cos(2 * Math.PI * rnd());

// Column-major OpenGL projection from OpenCV-style intrinsics (inverse of intrinsicsFromProjection).
function projectionFrom(fx, fy, cx, cy, W, H, n = 0.1, f = 100) {
  const P = new Float32Array(16);
  P[0] = (2 * fx) / W; P[5] = (2 * fy) / H; P[8] = 1 - (2 * cx) / W; P[9] = (2 * cy) / H - 1;
  P[10] = -(f + n) / (f - n); P[11] = -1; P[14] = (-2 * f * n) / (f - n);
  return P;
}

test('intrinsicsFromProjection recovers known intrinsics, including an off-center principal point', () => {
  const K = { fx: 500, fy: 502, cx: 330, cy: 235, W: 640, H: 480 };
  const r = intrinsicsFromProjection(projectionFrom(K.fx, K.fy, K.cx, K.cy, K.W, K.H), K.W, K.H);
  assert.ok(Math.abs(r.fx - K.fx) < 1e-3 && Math.abs(r.fy - K.fy) < 1e-3, JSON.stringify(r));
  assert.ok(Math.abs(r.cx - K.cx) < 1e-3 && Math.abs(r.cy - K.cy) < 1e-3, JSON.stringify(r));
  assert.ok(Math.abs(r.hfovDeg - 2 * Math.atan(320 / 500) * 180 / Math.PI) < 1e-3);
  assert.ok(Math.abs(r.squarePixelRatio - 502 / 500) < 1e-6);
  const c = intrinsicsFromProjection(projectionFrom(400, 400, 320, 240, 640, 480), 640, 480); // centered: P[8] = P[9] = 0
  assert.ok(Math.abs(c.cx - 320) < 1e-3 && Math.abs(c.cy - 240) < 1e-3);
});

test('percentile and fpsStats', () => {
  assert.equal(percentile([1, 2, 3, 4, 5], 50), 3);
  assert.equal(percentile([0, 10], 95), 9.5);
  assert.ok(Number.isNaN(percentile([], 50)));
  const times = Array.from({ length: 101 }, (_, i) => i * 33.3333);
  const s = fpsStats(times);
  assert.ok(Math.abs(s.fpsP50 - 30) < 0.01 && Math.abs(s.fpsAtP95Dt - 30) < 0.01, JSON.stringify(s));
  assert.equal(s.frames, 101);
});

// Smooth synthetic texture, continuous in (x, y) so fractional shifts can be sampled exactly.
const texture = (x, y) => 128 + 40 * Math.sin(0.31 * x + 0.17 * y) + 30 * Math.cos(0.23 * y - 0.11 * x + 1) + 20 * Math.sin(0.07 * x * 1.0 + 0.41 * y + 2);
const image = (w, h, sx, sy) => Uint8Array.from({ length: w * h }, (_, i) => Math.round(texture((i % w) - sx, Math.floor(i / w) - sy)));

test('estimateShiftSAD: integer and fractional content shifts, convention curr(x+dx) = prev(x)', () => {
  const w = 64;
  const h = 48;
  const prev = image(w, h, 0, 0);
  for (const [sx, sy] of [[0, 0], [3, 0], [-4, 2], [0, -5], [6, -6]]) {
    const r = estimateShiftSAD(prev, image(w, h, sx, sy), w, h);
    assert.ok(r.ok, JSON.stringify(r));
    assert.ok(Math.abs(r.dx - sx) < 0.2 && Math.abs(r.dy - sy) < 0.2, `shift (${sx},${sy}) -> (${r.dx},${r.dy})`);
  }
  for (const [sx, sy] of [[2.4, -1.3], [-3.6, 0.5], [1.5, 4.25]]) {
    const r = estimateShiftSAD(prev, image(w, h, sx, sy), w, h);
    assert.ok(Math.abs(r.dx - sx) < 0.3 && Math.abs(r.dy - sy) < 0.3, `shift (${sx},${sy}) -> (${r.dx},${r.dy})`);
  }
});

test('estimateShiftSAD: tolerates brightness change and rejects flat or unrelated images', () => {
  const w = 64;
  const h = 48;
  const prev = image(w, h, 0, 0);
  const bright = image(w, h, 2, 1).map((v) => Math.min(255, v + 25));
  const r = estimateShiftSAD(prev, bright, w, h);
  assert.ok(Math.abs(r.dx - 2) < 0.3 && Math.abs(r.dy - 1) < 0.3, JSON.stringify(r));
  const flat = new Uint8Array(w * h).fill(100);
  assert.equal(estimateShiftSAD(flat, flat, w, h).ok, false);
  const rnd = mulberry32(5);
  const noiseA = Uint8Array.from({ length: w * h }, () => Math.floor(rnd() * 256));
  const noiseB = Uint8Array.from({ length: w * h }, () => Math.floor(rnd() * 256));
  assert.equal(estimateShiftSAD(noiseA, noiseB, w, h).ok, false);
  assert.equal(estimateShiftSAD(prev, image(w, h, 6, 0), w, h).saturated, true);
});

test('welchCoherence: identical signals ~1 (also with a constant delay), independent noise low', () => {
  const fs = 30;
  const n = 600;
  const rnd = mulberry32(1);
  const x = Float64Array.from({ length: n }, () => gauss(rnd));
  const same = welchCoherence(x, x, fs);
  assert.ok(bandStats(same, 3, 6).coherence > 0.999);
  assert.ok(Math.abs(bandStats(same, 3, 6).gain - 1) < 1e-9);
  const delayed = new Float64Array(n);
  for (let i = 0; i < n; i++) delayed[i] = x[Math.max(0, i - 1)];
  assert.ok(bandStats(welchCoherence(x, delayed, fs), 3, 6).coherence > 0.8, 'one-sample delay keeps coherence high');
  const y = Float64Array.from({ length: n }, () => gauss(rnd));
  const indep = welchCoherence(x, y, fs);
  assert.ok(indep.nSegments >= 15);
  assert.ok(bandStats(indep, 3, 6).coherence < 0.35, `independent coherence ${bandStats(indep, 3, 6).coherence}`);
  const scaled = Float64Array.from(x, (v) => 0.6 * v);
  const g = bandStats(welchCoherence(scaled, x, fs), 3, 6);
  assert.ok(Math.abs(g.gain - 0.6) < 1e-9 && Math.abs(g.rmsRatio - 0.6) < 1e-9, JSON.stringify(g));
});

test('resampleUniform and intervalMean on a known ramp', () => {
  const t = [0, 0.1, 0.25, 0.4, 1];
  const v = t.map((s) => 2 * s + 1);
  const u = resampleUniform(t, v, 10, 0, 1);
  assert.equal(u.length, 11);
  assert.ok(u.every((val, i) => Math.abs(val - (2 * (i / 10) + 1)) < 1e-9));
  const m = intervalMean(Float64Array.from(t), Float64Array.from(v), [0.1, 0.3], [0.2, 0.9]);
  assert.ok(Math.abs(m[0] - 1.3) < 1e-9 && Math.abs(m[1] - 2.2) < 1e-9, Array.from(m).join());
  assert.ok(Number.isNaN(intervalMean(Float64Array.from(t), Float64Array.from(v), [-1], [0.5])[0]));
});

test('angularVelocityBetween: body-frame rate from two quaternions', () => {
  const a = 0.1;
  const qz = [0, 0, Math.sin(a / 2), Math.cos(a / 2)];
  const w = angularVelocityBetween([0, 0, 0, 1], qz, 0.01);
  assert.ok(Math.abs(w[2] - 10) < 1e-9 && Math.abs(w[0]) < 1e-12 && Math.abs(w[1]) < 1e-12, w.join());
  const wn = angularVelocityBetween([0, 0, 0, 1], qz.map((v) => -v), 0.01); // -q is the same rotation
  assert.ok(Math.abs(wn[2] - 10) < 1e-9, wn.join());
  assert.deepEqual(angularVelocityBetween([0, 0, 0, 1], [0, 0, 0, 1], 0.01), [0, 0, 0]);
});

// Synthetic shake: band-limited angular rate (rad/s) about the gyro x axis, camera frames at ~30 fps with jitter, gyro at ~100 Hz.
// Intact coupling: image rate (px/s) = fxSmall * rad/s. EIS-like: only the < 3 Hz components survive in the image.
function synth({ seed, eis = false, independent = false, seconds = 20, fxSmall = 50 }) {
  const rnd = mulberry32(seed);
  const comps = Array.from({ length: 12 }, () => ({ f: 0.8 + rnd() * 9.2, a: 0.05 + 0.05 * rnd(), p: rnd() * 2 * Math.PI }));
  const rate = (t, keep = () => true) => comps.filter(keep).reduce((s, c) => s + c.a * Math.sin(2 * Math.PI * c.f * t + c.p), 0);
  const gyro = { t: [], x: [], y: [], z: [] };
  for (let t = 0; t < seconds + 0.5; t += 0.01) {
    gyro.t.push(t * 1000 + gauss(rnd) * 0.5); gyro.x.push((rate(t) * 180) / Math.PI); gyro.y.push(gauss(rnd) * 3); gyro.z.push(gauss(rnd) * 3);
  }
  const imgRate = (tt) => {
    if (independent) return fxSmall * gauss(rnd) * 0.15;
    return fxSmall * rate(tt, eis ? (c) => c.f < 3 : () => true);
  };
  const frames = [];
  let t = 0.2;
  let prevT = null;
  while (t < seconds) {
    const dt = prevT == null ? 0 : t - prevT;
    const r = prevT == null ? 0 : imgRate(0.5 * (t + prevT)) + gauss(rnd) * 0.2; // midpoint rule + pixel noise
    frames.push({ t: t * 1000, dx: r * dt, dy: gauss(rnd) * 0.05, ok: prevT != null, q: null });
    prevT = t;
    t += 1 / 30 + (rnd() - 0.5) * 0.004;
  }
  return { frames, gyro };
}

test('analyzeRun: shake with intact video-gyro coupling gives high 3-6 Hz coherence and amplitude ratio ~1', () => {
  const { frames, gyro } = synth({ seed: 11 });
  const r = analyzeRun({ frames, gyro, fxSmallPx: 50 });
  assert.equal(r.insufficient, null);
  const best = r.refs.gyro.best;
  assert.equal(best.img, 'dx');
  assert.equal(best.axis, 'x');
  assert.ok(best.bands.mid.coherence > 0.9, JSON.stringify(best.bands.mid));
  assert.ok(Math.abs(best.bands.mid.ampRatio - 1) < 0.25, JSON.stringify(best.bands.mid));
  assert.ok(r.counts.okPairs > 500 && r.counts.gyroSamples > 1900);
});

test('analyzeRun: independent image motion gives low coherence; low-pass (EIS-like) video loses 3-6 Hz coherence', () => {
  const ind = analyzeRun({ ...synth({ seed: 11, independent: true }), fxSmallPx: 50 });
  assert.ok(ind.refs.gyro.best.bands.mid.coherence < 0.4, JSON.stringify(ind.refs.gyro.best.bands.mid));
  const clean = analyzeRun({ ...synth({ seed: 12 }), fxSmallPx: 50 }).refs.gyro.best.bands;
  const eis = analyzeRun({ ...synth({ seed: 12, eis: true }), fxSmallPx: 50 }).refs.gyro.best.bands;
  assert.ok(clean.mid.coherence > 0.8 && clean.mid.coherenceW > 0.95, JSON.stringify(clean.mid));
  assert.ok(eis.mid.coherenceW < 0.5, JSON.stringify(eis.mid));
  assert.ok(eis.mid.ampRatio < 0.5 * clean.mid.ampRatio, `eis ratio ${eis.mid.ampRatio} vs clean ${clean.mid.ampRatio}`);
});

test('analyzeRun: XR pose angular velocity reference, insufficient data, stalls', () => {
  const rnd = mulberry32(3);
  const frames = [];
  let ang = 0;
  const fx = 40;
  const phase = Array.from({ length: 8 }, () => rnd() * 6.28);
  for (let i = 0; i < 600; i++) {
    const t = i / 30;
    const w = phase.reduce((s, p, k) => s + Math.sin(2 * Math.PI * (0.8 + 1.2 * k) * t + p), 0); // rad/s about y
    const prevAng = ang;
    ang += w / 30;
    frames.push({ t: t * 1000, dx: i ? -fx * (ang - prevAng) : 0, dy: 0, ok: i > 0, q: [0, Math.sin(ang / 2), 0, Math.cos(ang / 2)] });
  }
  const r = analyzeRun({ frames, gyro: null, fxSmallPx: fx });
  assert.equal(Object.keys(r.refs).join(), 'pose');
  const b = r.refs.pose.best;
  assert.equal(b.img, 'dx');
  assert.equal(b.axis, 'y');
  assert.ok(b.bands.mid.coherence > 0.95 && Math.abs(b.bands.mid.ampRatio - 1) < 0.05, JSON.stringify(b.bands.mid));

  assert.ok(analyzeRun({ frames: frames.slice(0, 20), gyro: null }).insufficient);
  const stalled = frames.map((f, i) => (i === 300 ? { ...f, t: f.t + 400 } : f)).filter((f, i) => i < 300 || i >= 300);
  assert.ok(analyzeRun({ frames: stalled, gyro: null }).counts.stalls >= 1);
});

test('probe files are reachable through the dev server path map and exist on disk', () => {
  for (const name of ['index.html', 'app.js', 'analysis.js']) {
    const file = resolveSamplePath(`/samples/webxr-probe/${name}`);
    assert.ok(file && file.endsWith(path.join('samples', 'webxr-probe', name)), String(file));
    assert.ok(fs.existsSync(file), file);
  }
  assert.equal(path.resolve(resolveSamplePath('/samples/webxr-probe/') ?? ''), path.resolve(here, '../samples/webxr-probe/index.html'));
});

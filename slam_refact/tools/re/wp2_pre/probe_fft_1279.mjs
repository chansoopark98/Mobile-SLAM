// WP2-PRE items 1, 2 and 4: isolated execution of FrameFrameTracker RNG helpers and of func 1279
// (robustGroundHomography) in the ORIGINAL xr-slam.wasm, compared with
//   (a) rng_model.mjs (mt19937 + libc++ uniform_int_distribution + std::shuffle + 1279 sampling), and
//   (b) a reference of spec S4 §3.7.3 built from public c8/geometry/homography.cc:239-370.
//
// Method (same as spec_map_tracker/probe-yaw-replace.mjs): only the export section is patched
// (code section hash must stay identical); the module is instantiated with import stubs that throw,
// except emscripten_get_now (monotonic counter, needed by ScopeTimer) and emscripten_memcpy_big
// (plain memmove, as in the emscripten JS library). Objects are built by the
// binary itself: MapTracker = func 640 on memset(malloc(6296)) exactly as funcs 476/485 do.
//
// Pre-registered criteria (fixed before the first run; do not edit after seeing results):
const CRIT = {
  ctorRngState: 'MapTracker+0x1e0: 624 words + index equal std::mt19937(5489) model, exactly',
  ctorState: 'MapTracker+0x11f4 (state_) == 0 after func 640',
  uid805: 'func 805 outputs and post-call mt state equal model uid32/shuffle helper, exactly',
  uid895: 'func 895 with RandomNumbers::uniform32i param equals raw mt output, exactly',
  sampling1279: 'hSampleInds, ptSampleInds (n>100) and post-call mt state equal rng_model.groundCall, exactly',
  outputs1279: {
    okExact: true,          // return value == reference ok
    matAbs: 1e-5,           // max |H_bin - H_ref|, |camMotion_bin - camMotion_ref| (forward 4x4)
    maskExact: true,        // mask == reference mask (a mismatch whose reference residual lies within
                            // 1e-4 relative of the 1.1*best threshold is ALSO listed as 'boundary')
  },
};
// Usage: node probe_fft_1279.mjs --input xr-slam.wasm --output NEW_DIRECTORY
import fs from 'node:fs';
import crypto from 'node:crypto';
import path from 'node:path';
import { Mt19937, uid32, groundCall } from './rng_model.mjs';

const argv = process.argv.slice(2);
if (argv.length !== 4 || argv[0] !== '--input' || argv[2] !== '--output') {
  throw Error('Usage: --input xr-slam.wasm --output NEW_DIRECTORY');
}
const input = path.resolve(argv[1]);
const outDir = path.resolve(argv[3]);
if (outDir === '/mnt/backup/SLAM' || outDir.startsWith('/mnt/backup/SLAM/')) throw Error('read-only dataset root');
if (fs.existsSync(outDir)) throw Error('Output already exists; preserving evidence');
const original = fs.readFileSync(input);
const sha = (b) => crypto.createHash('sha256').update(b).digest('hex');
const PINNED = '780731fe294a725acf1b7717ed5ffb07ad9f0929e408a1d74f47450c291ec422';
if (sha(original) !== PINNED) throw Error('Input differs from pinned xr-slam.wasm');

// ---------- module patch (export section only) ----------
const leb = (n) => { const b = []; do { let v = n & 127; n >>>= 7; if (n) v |= 128; b.push(v); } while (n); return Buffer.from(b); };
const readLeb = (b, p) => { let n = 0, s = 0, v; do { v = b[p++]; n |= (v & 127) << s; s += 7; } while (v & 128); return [n >>> 0, p]; };
const EXPORTS = [['wp2Ctor640', 640], ['wp2Uid805', 805], ['wp2Uid895', 895], ['wp2Ground1279', 1279]];
const parts = [original.subarray(0, 8)];
let pos = 8, codeBefore = null, codeAfter = null;
while (pos < original.length) {
  const start = pos, id = original[pos++];
  const [len, p] = readLeb(original, pos);
  pos = p + len;
  let payload = original.subarray(p, pos);
  if (id === 10) codeBefore = sha(payload);
  if (id === 7) {
    const [count, rest] = readLeb(payload, 0);
    const extra = EXPORTS.map(([n, i]) => Buffer.concat([leb(n.length), Buffer.from(n), Buffer.from([0]), leb(i)]));
    payload = Buffer.concat([leb(count + EXPORTS.length), payload.subarray(rest), ...extra]);
    parts.push(Buffer.concat([Buffer.from([id]), leb(payload.length), payload]));
  } else {
    parts.push(original.subarray(start, pos));
  }
  if (id === 10) codeAfter = sha(payload);
}
if (codeBefore === null || codeBefore !== codeAfter) throw Error('code section changed');
const patched = Buffer.concat(parts);
const mod = new WebAssembly.Module(patched);
const invokedImports = {};
let clock = 0;
const stubs = {};
for (const imp of WebAssembly.Module.imports(mod)) {
  if (imp.kind !== 'function') continue;
  stubs[imp.module] ??= {};
  stubs[imp.module][imp.name] = (...a) => {
    invokedImports[imp.name] = (invokedImports[imp.name] || 0) + 1;
    if (imp.name === 'emscripten_get_now') { clock += 0.001; return clock; }
    // emscripten JS library: HEAPU8.copyWithin(dest, src, src + num) (large memcpy fast path)
    if (imp.name === 'emscripten_memcpy_big') { new Uint8Array(inst.exports.memory.buffer).copyWithin(a[0], a[1], a[1] + a[2]); return a[0]; }
    throw Error('import called: ' + imp.name);
  };
}
const inst = new WebAssembly.Instance(mod, stubs);
const X = inst.exports;
const mem = X.memory;
const u8 = () => new Uint8Array(mem.buffer);
const u32 = () => new Uint32Array(mem.buffer);
const f32 = () => new Float32Array(mem.buffer);
const malloc = (n) => { const p = X.malloc(n); if (!p) throw Error('malloc failed'); u8().fill(0, p, p + n); return p; };

// ---------- helpers ----------
const MT_OFF = 0x1e0;                 // RandomNumbers inside MapTracker (= FFT+0x158)
const SCRATCH_OFF = 0x138;            // RobustHomographyScratchSpace (= FFT+0xb0)
const readMt = (rng) => ({ x: Array.from(u32().subarray(rng / 4, rng / 4 + 624)), i: u32()[rng / 4 + 624] });
const sameMt = (bin, model) => bin.i === model.i && bin.x.every((v, k) => v === model.x[k]);
const newMapTracker = () => { const p = malloc(6296); X.wp2Ctor640(p); return p; };
const readVec = (addr, elemBytes) => { const b = u32()[addr / 4], e = u32()[addr / 4 + 1]; return { b, n: (e - b) / elemBytes }; };

const report = { criteria: CRIT, checks: {} };

// ---------- item 4 + 1: constructor ----------
const mt0 = newMapTracker();
{
  const bin = readMt(mt0 + MT_OFF);
  const model = new Mt19937(5489);
  const st = new DataView(mem.buffer).getInt32(mt0 + 0x11f4, true);
  const dv = new DataView(mem.buffer);
  report.checks.ctor = {
    rngStateEqualsMt19937_5489: sameMt(bin, model),
    mtIndex: bin.i,
    uniform32iParam: [dv.getUint32(mt0 + MT_OFF + 2524, true), dv.getUint32(mt0 + MT_OFF + 2528, true)],
    uniform32fParam: [dv.getFloat32(mt0 + MT_OFF + 2516, true), dv.getFloat32(mt0 + MT_OFF + 2520, true)],
    normalParam: [dv.getFloat32(mt0 + MT_OFF + 2500, true), dv.getFloat32(mt0 + MT_OFF + 2504, true)],
    heightAt0xbd8: dv.getFloat32(mt0 + 0xbd8, true),
    state_at0x11f4: st,
    frameTrackerOk_at0x11f8: u8()[mt0 + 0x11f8],
    frameCount_at0x11a0: dv.getBigInt64(mt0 + 0x11a0, true).toString(),
    lastStateChange_at0x11a8: dv.getBigInt64(mt0 + 0x11a8, true).toString(),
    field0x1850: dv.getInt32(mt0 + 0x1850, true),
  };
  report.checks.ctor.pass = report.checks.ctor.rngStateEqualsMt19937_5489 && st === 0;
}

// ---------- item 1: 805 / 895 directly ----------
{
  const mtA = newMapTracker();
  const rng = mtA + MT_OFF;
  const model = new Mt19937(5489);
  const ranges = [[0, 1], [0, 2], [0, 6], [0, 7], [0, 8], [0, 72], [0, 99], [0, 364], [0, 4949], [5, 9], [0, 0x7fffffff], [3, 3]];
  let ok = true; const rows = [];
  for (const [a, b] of ranges) {
    const outB = [], outM = [];
    for (let k = 0; k < 64; k++) { outB.push(X.wp2Uid805(rng, a, b) >>> 0); outM.push(uid32(model, a, b) >>> 0); }
    const same = outB.every((v, k) => v === outM[k]) && sameMt(readMt(rng), model);
    ok &&= same;
    rows.push({ a, b, same, first8: outB.slice(0, 8) });
  }
  // 895 with the RandomNumbers' own uniform32i param (rng+2524): nextUnsignedInt()
  const outB = [], outM = [];
  for (let k = 0; k < 200; k++) { outB.push(X.wp2Uid895(rng, rng + 2524) >>> 0); outM.push(model.next()); }
  const same895 = outB.every((v, k) => v === outM[k]) && sameMt(readMt(rng), model);
  report.checks.uid = { pass: ok && same895, uid805: rows, uid895RawEqualsMt: same895 };
}

// ---------- geometry (float32 c8 formulas, evaluated in float64 for the reference) ----------
const fr = Math.fround;
const quatToR = (q) => { // c8 Quaternion::toRotationMat (double products, float result), row-major
  const [x, y, z, w] = q;
  return [[fr(1 - 2 * (y * y + z * z)), fr(2 * (x * y - w * z)), fr(2 * (x * z + w * y))],
    [fr(2 * (x * y + w * z)), fr(1 - 2 * (x * x + z * z)), fr(2 * (y * z - w * x))],
    [fr(2 * (x * z - w * y)), fr(2 * (y * z + w * x)), fr(1 - 2 * (x * x + y * y))]];
};
const M4 = (R, t = [0, 0, 0]) => [[R[0][0], R[0][1], R[0][2], t[0]], [R[1][0], R[1][1], R[1][2], t[1]], [R[2][0], R[2][1], R[2][2], t[2]], [0, 0, 0, 1]];
const mul = (A, B) => A.map((row, i) => B[0].map((_, j) => row.reduce((s, v, k) => s + v * B[k][j], 0)));
const inv4 = (A) => { // general 4x4 inverse (Gauss-Jordan, float64)
  const n = 4, M = A.map((r, i) => [...r, ...[0, 1, 2, 3].map((j) => (i === j ? 1 : 0))]);
  for (let c = 0; c < n; c++) {
    let p = c; for (let r = c + 1; r < n; r++) if (Math.abs(M[r][c]) > Math.abs(M[p][c])) p = r;
    [M[c], M[p]] = [M[p], M[c]];
    const d = M[c][c]; for (let j = 0; j < 2 * n; j++) M[c][j] /= d;
    for (let r = 0; r < n; r++) if (r !== c) { const f = M[r][c]; for (let j = 0; j < 2 * n; j++) M[r][j] -= f * M[c][j]; }
  }
  return M.map((r) => r.slice(n));
};
const apply = (A, v) => [0, 1, 2, 3].map((i) => A[i][0] * v[0] + A[i][1] * v[1] + A[i][2] * v[2] + A[i][3] * v[3]);
const I4 = () => M4([[1, 0, 0], [0, 1, 0], [0, 0, 1]]);
const toRad = fr(fr(fr(Math.PI) * -90) / 180);
const XDEG_M90 = (() => { const c = fr(Math.cos(toRad)), s = fr(Math.sin(toRad)); return M4([[1, 0, 0], [0, c, -s], [0, s, c]]); })();
const zRad = (t) => { const c = Math.cos(t), s = Math.sin(t); return M4([[c, -s, 0], [s, c, 0], [0, 0, 1]]); };

// public homography.cc:239-319 (gravity-normal version), returns null when it would return identity
function groundPlanePair(QA, a, QB, b) {
  const as3 = a.map((p) => apply(QA, [p[0], p[1], p[2], 1]));
  const bs3 = b.map((p) => apply(QB, [p[0], p[1], p[2], 1]));
  if (as3[0][2] <= 0.2 || bs3[0][2] <= 0.2 || as3[1][2] <= 0.2 || bs3[1][2] <= 0.2) return null;
  const f2 = (p) => [p[0] / p[2], p[1] / p[2]];
  const a2 = as3.map(f2), b2 = bs3.map(f2);
  if ((a2[0][0] === a2[1][0] && a2[0][1] === a2[1][1]) || (b2[0][0] === b2[1][0] && b2[0][1] === b2[1][1])) return null;
  const center = (q2) => {
    const mx = (q2[0][0] + q2[1][0]) * 0.5, my = (q2[0][1] + q2[1][1]) * 0.5;
    const dx = q2[0][0] - mx, dy = q2[0][1] - my;
    const scale = 0.70710678118 * Math.sqrt(dx * dx + dy * dy);
    if (scale <= 1e-5) return null;
    return [[1, 0, -mx, 0], [0, 1, -my, 0], [0, 0, scale, 0], [0, 0, 0, 1]];
  };
  const Ta = center(a2), Tb = center(b2);
  if (!Ta || !Tb) return null;
  const ac = apply(Ta, as3[0]), bc = apply(Tb, bs3[0]);
  const theta = Math.atan2(bc[0] / bc[2], bc[1] / bc[2]) - Math.atan2(ac[0] / ac[2], ac[1] / ac[2]);
  const rot = zRad(theta);
  const hs = mul(mul(inv4(Tb), inv4(rot)), Ta);
  const inorm = 1 / Math.sqrt(hs[0][0] * hs[0][0] + hs[0][1] * hs[0][1]);
  const h = [[hs[0][0] * inorm, hs[0][1] * inorm, hs[0][2] * inorm, 0], [hs[1][0] * inorm, hs[1][1] * inorm, hs[1][2] * inorm, 0],
    [hs[2][0] * inorm, hs[2][1] * inorm, hs[2][2] * inorm, 0], [0, 0, 0, 1]];
  const m = mul(rot, M4([[1, 0, 0], [0, 1, 0], [0, 0, 1]], [-h[0][2], -h[1][2], 1 - h[2][2]]));
  return { H: mul(mul(inv4(QB), h), QA), cam: mul(mul(inv4(QA), m), QB) };
}
const resid2 = (H, a, b) => { const p = apply(H, [a[0], a[1], a[2], 1]); const dx = p[0] / p[2] - b[0] / b[2], dy = p[1] / p[2] - b[1] / b[2]; return dx * dx + dy * dy; };

// spec S4 §3.7.3 with sampling from rng_model (pairs, samples)
function reference1279(qA, A, qB, B, sampling) {
  const n = A.length;
  if (n < 15) return { ok: false, H: I4(), cam: I4(), mask: new Array(n).fill(0) };
  const QA = mul(XDEG_M90, M4(quatToR(qA))), QB = mul(XDEG_M90, M4(quatToR(qB)));
  const S = sampling.samples;
  const nS = S.length;
  const k30 = Math.floor(0.3 * nS);
  const kIdx = k30 > 4 ? k30 : Math.min(nS - 1, 4);
  let best = Infinity, bestI = -1; const hyps = [];
  sampling.pairs.forEach(([i, j], hi) => {
    const r = groundPlanePair(QA, [A[i], A[j]], QB, [B[i], B[j]]) || { H: I4(), cam: I4() };
    hyps.push(r);
    const res = S.map((s) => resid2(r.H, A[s], B[s])).sort((x, y) => x - y);
    const sc = res[kIdx];
    if (sc < best) { best = sc; bestI = hi; }
  });
  if (!(best <= 2e-4)) return { ok: false, H: I4(), cam: I4(), mask: null, best, bestI, kIdx };
  const H = hyps[bestI].H, cam = hyps[bestI].cam;
  const res = A.map((a, i) => resid2(H, a, B[i]));
  return { ok: true, H, cam, mask: res.map((r) => (r <= 1.1 * best ? 1 : 0)), res, best, bestI, kIdx };
}

// ---------- synthetic scenes ----------
let seed = 20261015 >>> 0;
const rnd = () => { seed = (Math.imul(seed, 1664525) + 1013904223) >>> 0; return seed / 4294967296; };
const gauss = () => Math.sqrt(-2 * Math.log(rnd() + 1e-12)) * Math.cos(2 * Math.PI * rnd());
const axq = (ax, ang) => { const s = Math.sin(ang / 2), n = Math.hypot(...ax); return [ax[0] / n * s, ax[1] / n * s, ax[2] / n * s, Math.cos(ang / 2)]; };
const qmul = (a, b) => { const [ax, ay, az, aw] = a, [bx, by, bz, bw] = b;
  return [aw * bx + ax * bw + ay * bz - az * by, aw * by - ax * bz + ay * bw + az * bx, aw * bz + ax * by - ay * bx + az * bw, aw * bw - ax * bx - ay * by - az * bz]; };
function scene({ n, noise = 0, outlier = 0, planar = true, motion = 0.15, height = 1.5 }) {
  const pitch = (40 + 25 * rnd()) * Math.PI / 180;
  const qA = qmul(axq([0, 1, 0], (rnd() - 0.5) * 2), axq([1, 0, 0], pitch)).map(fr);
  const qB = qmul(qA, qmul(axq([0, 1, 0], (rnd() - 0.5) * 0.1), axq([1, 0, 0], (rnd() - 0.5) * 0.1))).map(fr);
  const cA = [0, 0, 0];
  const cB = [(rnd() - 0.5) * 2 * motion, (rnd() - 0.5) * 0.4 * motion, (rnd() - 0.5) * 2 * motion];
  const RA = quatToR(qA), RB = quatToR(qB);
  const A = [], B = [];
  let guard = 0;
  while (A.length < n && guard++ < 100000) {
    const r = [(rnd() - 0.5) * 1.0, (rnd() - 0.5) * 1.0, 1];
    const d = [0, 1, 2].map((i) => RA[i][0] * r[0] + RA[i][1] * r[1] + RA[i][2] * r[2]);
    let X;
    if (planar) { if (d[1] > -0.25) continue; const t = (-height - cA[1]) / d[1]; X = d.map((v, i) => cA[i] + t * v); }
    else { const t = 1 + 6 * rnd(); X = d.map((v, i) => cA[i] + t * v); }
    const v = [0, 1, 2].map((i) => X[0] - cB[0]).map((_, i) => 0);
    const rel = [X[0] - cB[0], X[1] - cB[1], X[2] - cB[2]];
    for (let i = 0; i < 3; i++) v[i] = RB[0][i] * rel[0] + RB[1][i] * rel[1] + RB[2][i] * rel[2];
    if (v[2] < 0.2 || Math.abs(v[0] / v[2]) > 0.6 || Math.abs(v[1] / v[2]) > 0.6) continue;
    let bx = v[0] / v[2], by = v[1] / v[2];
    if (rnd() < outlier) { bx = (rnd() - 0.5) * 1.0; by = (rnd() - 0.5) * 1.0; }
    else if (noise) { bx += noise * gauss(); by += noise * gauss(); }
    A.push([fr(r[0]), fr(r[1]), 1]); B.push([fr(bx), fr(by), 1]);
  }
  // ground-truth relative motion candidates (translation in units of camera height)
  const tw = [cB[0] - cA[0], cB[1] - cA[1], cB[2] - cA[2]];
  const RAt = [0, 1, 2].map((i) => [RA[0][i], RA[1][i], RA[2][i]]);
  const Rrel = mul(M4(RAt), M4(RB)).slice(0, 3).map((r) => r.slice(0, 3));
  const trel = [0, 1, 2].map((i) => (RAt[i][0] * tw[0] + RAt[i][1] * tw[1] + RAt[i][2] * tw[2]) / height);
  return { qA, qB, A, B, truth: { a_from_b: M4(Rrel, trel) } };
}

// ---------- 1279 runs ----------
const mtG = newMapTracker();
const rngG = mtG + MT_OFF, scratch = mtG + SCRATCH_OFF;
const model = new Mt19937(5489);
const qAp = malloc(16), qBp = malloc(16), vecA = malloc(12), vecB = malloc(12), Hp = malloc(0x84), Cp = malloc(0x84), maskV = malloc(12);
const writeVec = (vec, pts) => {
  const buf = malloc(Math.max(12, pts.length * 12));
  const a = f32();
  pts.forEach((p, k) => { a[buf / 4 + 3 * k] = p[0]; a[buf / 4 + 3 * k + 1] = p[1]; a[buf / 4 + 3 * k + 2] = p[2]; });
  const w = u32(); w[vec / 4] = buf; w[vec / 4 + 1] = buf + pts.length * 12; w[vec / 4 + 2] = buf + pts.length * 12;
  return buf;
};
const readMat = (p) => { const a = f32(); const M = [[], [], [], []]; for (let c = 0; c < 4; c++) for (let r = 0; r < 4; r++) M[r][c] = a[p / 4 + 4 * c + r]; return M; };
const matErr = (A, B) => Math.max(...A.flatMap((r, i) => r.map((v, j) => Math.abs(v - B[i][j]))));

const plan = [
  { n: 10, kind: 'too-few' }, { n: 15, kind: 'clean' }, { n: 20, kind: 'clean' }, { n: 27, kind: 'noisy' },
  { n: 28, kind: 'noisy' }, { n: 40, kind: 'noisy' }, { n: 60, kind: 'noisy' }, { n: 100, kind: 'noisy' },
  { n: 101, kind: 'noisy' }, { n: 150, kind: 'noisy' }, { n: 40, kind: 'nonplanar' }, { n: 60, kind: 'noisy' },
  { n: 20, kind: 'noisy' }, { n: 120, kind: 'clean' }, { n: 33, kind: 'noisy' }, { n: 45, kind: 'noisy' },
  // appended after r1/r2 (cases 0-15 unchanged): all-outlier inputs exercise the reject path (best > 2e-4)
  { n: 30, kind: 'random' }, { n: 110, kind: 'random' },
];
const cases = [];
let nSamplingFail = 0, nOutFail = 0, nBoundary = 0;
plan.forEach((pl, ci) => {
  const sc = scene({ n: pl.n, noise: pl.kind === 'noisy' ? 1e-3 : 0, outlier: pl.kind === 'noisy' ? 0.2 : pl.kind === 'random' ? 1 : 0, planar: pl.kind !== 'nonplanar' });
  f32().set(sc.qA, qAp / 4); f32().set(sc.qB, qBp / 4);
  writeVec(vecA, sc.A); writeVec(vecB, sc.B);
  u32().fill(0, maskV / 4, maskV / 4 + 3);
  u8().fill(0xab, Hp, Hp + 0x84); u8().fill(0xab, Cp, Cp + 0x84);
  const before = readMt(rngG);
  const ret = X.wp2Ground1279(qAp, vecA, qBp, vecB, scratch, rngG, Hp, Cp, maskV);
  // model sampling for this call (model RNG continues across calls, as the FFT member does)
  const draws0 = model.draws;
  const smp = groundCall(model, pl.n);
  const after = readMt(rngG);
  // scratch: hSampleInds @ +0 (8 B pairs), ptSampleInds @ +120 (u32)
  const hv = readVec(scratch + 0, 8);
  const binPairs = []; for (let k = 0; k < hv.n; k++) binPairs.push([u32()[hv.b / 4 + 2 * k], u32()[hv.b / 4 + 2 * k + 1]]);
  const pv = readVec(scratch + 120, 4);
  const binPtInds = Array.from(u32().subarray(pv.b / 4, pv.b / 4 + pv.n));
  const pairsSame = pl.n < 15 ? true : (binPairs.length === smp.pairs.length && binPairs.every((p, k) => p[0] === smp.pairs[k][0] && p[1] === smp.pairs[k][1]));
  const samplesSame = pl.n <= 100 ? true : (binPtInds.length === pl.n && binPtInds.every((v, k) => v === smp.sampleShuffle[k]));
  const rngSame = sameMt(after, model);
  const samplingPass = pairsSame && samplesSame && rngSame;
  if (!samplingPass) nSamplingFail++;
  // outputs
  const Hb = readMat(Hp), Cb = readMat(Cp);
  const mv = readVec(maskV, 1); const maskB = Array.from(u8().subarray(mv.b, mv.b + mv.n));
  const ref = reference1279(sc.qA, sc.A, sc.qB, sc.B, smp);
  const okSame = (ret !== 0) === ref.ok;
  const eH = matErr(Hb, ref.H), eC = matErr(Cb, ref.cam);
  let maskSame = true, boundary = [];
  if (ref.ok) {
    maskSame = maskB.length === pl.n && maskB.every((v, k) => v === ref.mask[k]);
    if (!maskSame) {
      maskB.forEach((v, k) => { if (v !== ref.mask[k]) { const thr = 1.1 * ref.best; const rel = Math.abs(ref.res[k] - thr) / thr; boundary.push({ k, bin: v, ref: ref.mask[k], rel }); } });
      if (boundary.every((b) => b.rel <= 1e-4)) nBoundary++;
    }
  } else {
    maskSame = maskB.length === 0 || maskB.length === pl.n && maskB.every((v) => v === 0);
  }
  const outPass = okSame && eH <= CRIT.outputs1279.matAbs && eC <= CRIT.outputs1279.matAbs && maskSame;
  if (!outPass) nOutFail++;
  const truthErr = ref.ok || ret ? { camVsTruth: matErr(Cb, sc.truth.a_from_b), camVsTruthInv: matErr(Cb, inv4(sc.truth.a_from_b)) } : null;
  cases.push({
    i: ci, n: pl.n, kind: pl.kind, path: smp.path, attempts: smp.attempts, rawDraws: model.draws - draws0,
    mtIndexBefore: before.i, mtIndexAfter: after.i,
    sampling: { pairsSame, samplesSame, rngSame, pass: samplingPass, nPairs: binPairs.length, firstPairs: binPairs.slice(0, 6) },
    outputs: { ret, refOk: ref.ok, okSame, errH: eH, errCam: eC, maskSame, maskOnes: maskB.reduce((s, v) => s + v, 0), maskLen: maskB.length,
      refBest: ref.best, refBestIndex: ref.bestI, kIdx: ref.kIdx, boundary, pass: outPass,
      flagsH: [u8()[Hp + 0x80], u8()[Hp + 0x81]], flagsCam: [u8()[Cp + 0x80], u8()[Cp + 0x81]] },
    truth: truthErr,
    golden: { qA: sc.qA, qB: sc.qB, A: sc.A, B: sc.B, H: Hb, cam: Cb, mask: maskB, ret },
  });
});
report.checks.sampling1279 = { pass: nSamplingFail === 0, nCases: cases.length, nFail: nSamplingFail };
report.checks.outputs1279 = { pass: nOutFail === 0, nCases: cases.length, nFail: nOutFail, nBoundaryOnly: nBoundary };

fs.mkdirSync(outDir);
fs.copyFileSync(new URL(import.meta.url), path.join(outDir, 'probe_fft_1279.mjs'));
fs.copyFileSync(new URL('./rng_model.mjs', import.meta.url), path.join(outDir, 'rng_model.mjs'));
const summary = {
  wasm_sha256: PINNED, patched_sha256: sha(patched), code_section_sha256: codeBefore, exports: EXPORTS,
  invokedImports, node: process.version, ...report,
};
fs.writeFileSync(path.join(outDir, 'summary.json'), JSON.stringify(summary, null, 1));
fs.writeFileSync(path.join(outDir, 'cases.json'), JSON.stringify(cases.map(({ golden, ...c }) => c), null, 1));
fs.writeFileSync(path.join(outDir, 'golden-1279.json'), JSON.stringify(cases.map((c) => ({ i: c.i, n: c.n, kind: c.kind, mtIndexBefore: c.mtIndexBefore, ...c.golden })), null, 0));
console.log(JSON.stringify({ ctor: report.checks.ctor, uid: { pass: report.checks.uid.pass }, sampling1279: report.checks.sampling1279, outputs1279: report.checks.outputs1279, invokedImports }, null, 1));
for (const c of cases) console.log(c.i, c.n, c.kind, c.path, 'samp', c.sampling.pass, 'ret', c.outputs.ret, 'ref', c.outputs.refOk, 'eH', c.outputs.errH.toExponential(2), 'eC', c.outputs.errCam.toExponential(2), 'mask', c.outputs.maskSame, c.outputs.maskOnes + '/' + c.outputs.maskLen, 'truth', c.truth ? c.truth.camVsTruth.toExponential(2) + '/' + c.truth.camVsTruthInv.toExponential(2) : '-');
process.exitCode = report.checks.ctor.pass && report.checks.uid.pass && nSamplingFail === 0 && nOutFail === 0 ? 0 : 1;

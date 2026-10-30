#!/usr/bin/env node
// V-FFT (b) original check (review defect D1): runs the ORIGINAL xr-slam.wasm on the exact inputs of
// TEST FrameFrameTrackerVfft.AcceptedMotionAccuracy, dumped by the test itself
// (SR_VFFT_B_DUMP=<file> sr_fft_frame_frame_tracker_test, helper DumpVfftPair in
// slam_refact/engine/tests/frame_frame_tracker/frame_frame_tracker_test.cc).
//
// Isolation method = slam_refact/tools/re/wp2_pre/probe_fft_1279.mjs: only the export section is
// augmented (code section sha256 must stay identical), imports trap except emscripten_get_now
// (monotonic counter) and emscripten_memcpy_big (copyWithin, as the emscripten JS library).
// Per scene a fresh MapTracker is built by the binary's own constructor (func 640 on
// memset(malloc(6296))), so the 1279 generator (+0x1e0) and scratch (+0x138) are in their
// constructed state, which is the state 1266 hands to 1279 in the second Track call of the test
// (first call: no previous frame -> no matches -> 1279 with n = 0 draws nothing, WP2-PRE §1.3).
//
// Executed in the original per scene:
//   1267 (classifyHorizon) on all rays of frame a (q_a) and frame b (q_b)
//   1279 (robustGroundHomography) on the dumped ground-match ray lists (q_prev = q_a, q_cur = q_b)
// NOT executed in the original: the rest of 1266 (LocalMatcher match search, ground-match
// filtering, scaleTranslation, motion gate). The ground list is re-derived here from the dumped
// matches with the ORIGINAL 1267 classes (rule of our 1266 port) and must equal the dumped list.
//
// Pre-registered criteria (fixed before the first run; do not edit after seeing results):
const CRIT = {
  codeSection: 'code section sha256 unchanged by the export patch',
  ctorRng: 'every per-scene MapTracker+0x1e0 (624 words + index) equals the first constructed one',
  classes1267: 'original 1267 class vectors equal the dumped engine classes, exactly',
  groundList: 'ground list re-derived from matches + original classes equals the dumped ground_matches, exactly',
  iso1279: { retExact: true, matAbs: 1e-5, maskExact: true },  // V-FFT-ISO tolerance (04-wp2-pre §2)
  trackLink: 'dumped engine Track delta == scaleTranslation(height, engine camMotion) (f32), exactly',
  // V-FFT (b) thresholds as registered (thresholds-v1.json, architecture §6.6.4); evaluated on the
  // ORIGINAL output only to answer "does the original fail (b) too"; the gate verdict is not changed.
  gateB: { rotDeg: 0.05, dirDeg: 1.0 },
};
//   node slam_refact/tools/harness/vfft_b_original_probe.mjs --dump <vfft-b-inputs.json> --output <new dir>
//        [--input build/slam-refact-20261015/inputs/xr-slam.wasm]
import fs from 'node:fs';
import crypto from 'node:crypto';
import path from 'node:path';

const args = Object.fromEntries(process.argv.slice(2).reduce((acc, a, i, all) =>
  (a.startsWith('--') ? [...acc, [a.slice(2), all[i + 1]]] : acc), []));
const repo = path.resolve(import.meta.dirname, '../../..');
const input = path.resolve(args.input ?? path.join(repo, 'build/slam-refact-20261015/inputs/xr-slam.wasm'));
if (!args.dump || !args.output) throw Error('usage: --dump <vfft-b-inputs.json> --output <new dir> [--input xr-slam.wasm]');
const outDir = path.resolve(args.output);
if (outDir === '/mnt/backup/SLAM' || outDir.startsWith('/mnt/backup/SLAM/')) throw Error('read-only dataset root');
if (fs.existsSync(outDir)) throw Error('Output already exists; preserving evidence');
const sha = (b) => crypto.createHash('sha256').update(b).digest('hex');
const PINNED = '780731fe294a725acf1b7717ed5ffb07ad9f0929e408a1d74f47450c291ec422';
const original = fs.readFileSync(input);
if (sha(original) !== PINNED) throw Error('Input differs from pinned xr-slam.wasm');
const dumpBytes = fs.readFileSync(path.resolve(args.dump));
const dump = JSON.parse(dumpBytes.toString('utf8'));
// The dump writes floats with %.9g; restore the exact f32 values of every matrix before use.
const f32Mat = (M) => M.map((r) => r.map((v) => Math.fround(v)));
for (const sc of dump.scenes) {
  for (const k of ['delta', 'h', 'cam_motion']) sc.ours[k] = f32Mat(sc.ours[k]);
  sc.truth = f32Mat(sc.truth);
}

// ---------- module patch (export section only) ----------
const leb = (n) => { const b = []; do { let v = n & 127; n >>>= 7; if (n) v |= 128; b.push(v); } while (n); return Buffer.from(b); };
const readLeb = (b, p) => { let n = 0, s = 0, v; do { v = b[p++]; n |= (v & 127) << s; s += 7; } while (v & 128); return [n >>> 0, p]; };
const EXPORTS = [['d1Ctor640', 640], ['d1Horizon1267', 1267], ['d1Ground1279', 1279]];
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
let clock = 0, inst = null;
const stubs = {};
for (const imp of WebAssembly.Module.imports(mod)) {
  if (imp.kind !== 'function') continue;
  stubs[imp.module] ??= {};
  stubs[imp.module][imp.name] = (...a) => {
    invokedImports[imp.name] = (invokedImports[imp.name] || 0) + 1;
    if (imp.name === 'emscripten_get_now') { clock += 0.001; return clock; }
    if (imp.name === 'emscripten_memcpy_big') { new Uint8Array(inst.exports.memory.buffer).copyWithin(a[0], a[1], a[1] + a[2]); return a[0]; }
    throw Error('import called: ' + imp.name);
  };
}
inst = new WebAssembly.Instance(mod, stubs);
const X = inst.exports;
const mem = X.memory;
const u8 = () => new Uint8Array(mem.buffer);
const u32 = () => new Uint32Array(mem.buffer);
const i32 = () => new Int32Array(mem.buffer);
const f32 = () => new Float32Array(mem.buffer);
const malloc = (n) => { const p = X.malloc(n); if (!p) throw Error('malloc failed'); u8().fill(0, p, p + n); return p; };

const MT_OFF = 0x1e0, SCRATCH_OFF = 0x138;
const readMt = (rng) => Array.from(u32().subarray(rng / 4, rng / 4 + 625));
const newMapTracker = () => { const p = malloc(6296); X.d1Ctor640(p); return p; };
const writeVec = (vec, pts) => {
  const buf = malloc(Math.max(12, pts.length * 12));
  pts.forEach((p, k) => f32().set(p, buf / 4 + 3 * k));
  u32().set([buf, buf + pts.length * 12, buf + pts.length * 12], vec / 4);
};
// HMatrix: column-major float[16] at +0 (forward), float[16] inverse at +0x40, flags at +0x80.
const readMat = (p) => { const a = f32(); const M = [[], [], [], []]; for (let c = 0; c < 4; c++) for (let r = 0; r < 4; r++) M[r][c] = a[p / 4 + 4 * c + r]; return M; };
const matErr = (A, B) => Math.max(...A.flatMap((r, i) => r.map((v, j) => Math.abs(v - Math.fround(B[i][j])))));
const bitSame = (A, B) => A.every((r, i) => r.every((v, j) => Object.is(Math.fround(v), Math.fround(B[i][j]))));

// Metrics of the test (frame_frame_tracker_test.cc RotationErrorDeg / TranslationDirectionErrorDeg),
// evaluated in float64 on the f32 matrices.
const rotErrDeg = (a, b) => {
  let tr = 0;
  for (let i = 0; i < 3; i++) for (let k = 0; k < 3; k++) tr += a[k][i] * b[k][i];
  return Math.acos(Math.max(-1, Math.min(1, (tr - 1) / 2))) * 180 / Math.PI;
};
// Diagnostic only (not the gate metric): well-conditioned angle 2 asin(|R_a^T R_b - I|_F / sqrt 8).
const rotErrDegStable = (a, b) => {
  let f2 = 0;
  for (let i = 0; i < 3; i++) for (let j = 0; j < 3; j++) {
    let v = 0;
    for (let k = 0; k < 3; k++) v += a[k][i] * b[k][j];
    f2 += (v - (i === j ? 1 : 0)) ** 2;
  }
  return 2 * Math.asin(Math.min(1, Math.sqrt(f2 / 8))) * 180 / Math.PI;
};
const dirErrDeg = (a, b) => {
  const ta = [a[0][3], a[1][3], a[2][3]], tb = [b[0][3], b[1][3], b[2][3]];
  const c = (ta[0] * tb[0] + ta[1] * tb[1] + ta[2] * tb[2]) / (Math.hypot(...ta) * Math.hypot(...tb));
  return Math.acos(Math.max(-1, Math.min(1, c))) * 180 / Math.PI;
};
const fr = Math.fround;
// c8 scaleTranslation(s, cam) = cameraMotion(translation(cam) * s, rotation(cam)): the rotation goes
// through Quaternion::fromHMatrix (float sums/differences, double sqrt, float result) and
// Quaternion::toRotationMat (float products widened to double, float result), quaternion.cc:38-79,
// 174-200; translation = f32 product; T * R leaves R and t unchanged.
const fromHMatrix = (R) => {
  const q = [0, 0, 0, 0];
  const trace = fr(fr(R[0][0] + R[1][1]) + R[2][2]);
  if (trace >= 0) {
    let t = Math.sqrt(trace + 1);
    q[0] = 0.5 * t; t = 0.5 / t;
    q[1] = fr(R[2][1] - R[1][2]) * t; q[2] = fr(R[0][2] - R[2][0]) * t; q[3] = fr(R[1][0] - R[0][1]) * t;
  } else {
    const i = R[0][0] > R[1][1] ? (R[0][0] > R[2][2] ? 0 : 2) : (R[1][1] > R[2][2] ? 1 : 2);
    const j = (i + 1) % 3, k = (j + 1) % 3;
    let t = Math.sqrt(1 + R[i][i] - R[j][j] - R[k][k]);
    q[i + 1] = 0.5 * t; t = 0.5 / t;
    q[0] = fr(R[k][j] - R[j][k]) * t; q[j + 1] = fr(R[j][i] + R[i][j]) * t; q[k + 1] = fr(R[k][i] + R[i][k]) * t;
  }
  return q.map(fr);  // (w, x, y, z)
};
const toRotationMat = ([w, x, y, z]) => {
  const wx = fr(w * x), wy = fr(w * y), wz = fr(w * z), xx = fr(x * x), xy = fr(x * y), xz = fr(x * z);
  const yy = fr(y * y), yz = fr(y * z), zz = fr(z * z);
  return [[fr(1 - 2 * (yy + zz)), fr(2 * (xy - wz)), fr(2 * (xz + wy))],
    [fr(2 * (xy + wz)), fr(1 - 2 * (xx + zz)), fr(2 * (yz - wx))],
    [fr(2 * (xz - wy)), fr(2 * (yz + wx)), fr(1 - 2 * (xx + yy))]];
};
const scaleTranslation = (s, M) => {
  const R = toRotationMat(fromHMatrix(M));
  return [0, 1, 2].map((i) => [...R[i], fr(M[i][3] * s)]).concat([[0, 0, 0, 1]]);
};

const mtRef = readMt(newMapTracker() + MT_OFF);
const qAp = malloc(16), qBp = malloc(16), vecA = malloc(12), vecB = malloc(12), vout = malloc(12);
const Hp = malloc(0x84), Cp = malloc(0x84), maskV = malloc(12);
const results = [];
let failIso = 0, failCls = 0, failGround = 0, failCtor = 0, failLink = 0;
for (const sc of dump.scenes) {
  const mt = newMapTracker();
  const ctorSame = readMt(mt + MT_OFF).every((v, k) => v === mtRef[k]);
  if (!ctorSame) failCtor++;
  // 1267 on both frames
  const classify = (q, rays) => {
    f32().set(q, qAp / 4); writeVec(vecA, rays); u32().set([0, 0, 0], vout / 4);
    X.d1Horizon1267(vout, qAp, vecA);
    const b = u32()[vout / 4], e = u32()[vout / 4 + 1];
    return Array.from(i32().subarray(b / 4, e / 4));
  };
  const clsA = classify(sc.q_a, sc.rays_a), clsB = classify(sc.q_b, sc.rays_b);
  const clsSame = clsA.length === sc.classes_a.length && clsA.every((v, k) => v === sc.classes_a[k]) &&
    clsB.length === sc.classes_b.length && clsB.every((v, k) => v === sc.classes_b[k]);
  if (!clsSame) failCls++;
  const ground = sc.matches.filter(([w, d]) => clsA[d] === 1 && clsB[w] === 1);
  const groundSame = ground.length === sc.ground_matches.length &&
    ground.every(([w, d], k) => w === sc.ground_matches[k][0] && d === sc.ground_matches[k][1]);
  if (!groundSame) failGround++;
  const prevG = ground.map(([, d]) => sc.rays_a[d]), curG = ground.map(([w]) => sc.rays_b[w]);
  // 1279
  f32().set(sc.q_a, qAp / 4); f32().set(sc.q_b, qBp / 4);
  writeVec(vecA, prevG); writeVec(vecB, curG);
  u32().fill(0, maskV / 4, maskV / 4 + 3);
  u8().fill(0xab, Hp, Hp + 0x84); u8().fill(0xab, Cp, Cp + 0x84);
  const mtBefore = u32()[(mt + MT_OFF) / 4 + 624];
  const ret = X.d1Ground1279(qAp, vecA, qBp, vecB, mt + SCRATCH_OFF, mt + MT_OFF, Hp, Cp, maskV);
  const mtAfter = u32()[(mt + MT_OFF) / 4 + 624];
  const H = readMat(Hp), cam = readMat(Cp);
  const mb = u32()[maskV / 4], me = u32()[maskV / 4 + 1];
  const mask = Array.from(u8().subarray(mb, me));
  const o = sc.ours;
  const errH = matErr(H, o.h), errCam = matErr(cam, o.cam_motion);
  const maskSame = mask.length === o.mask.length && mask.every((v, k) => v === o.mask[k]);
  const isoPass = (ret !== 0) === o.ret1279 && errH <= CRIT.iso1279.matAbs && errCam <= CRIT.iso1279.matAbs && maskSame;
  if (!isoPass) failIso++;
  // 1266 tail (port rule, not executed in the original): delta = scaleTranslation(height, cam)
  const deltaOrig = scaleTranslation(fr(sc.height), cam);
  const deltaOursFromCam = scaleTranslation(fr(sc.height), o.cam_motion);
  const link = bitSame(deltaOursFromCam, o.delta);
  if (!link) failLink++;
  const tnorm = Math.hypot(deltaOrig[0][3], deltaOrig[1][3], deltaOrig[2][3]);
  const accepted = ret !== 0 && tnorm <= 0.5;  // maxMotionThreshold 0.5 (config_defaults.inc)
  const orig = { rotDeg: rotErrDeg(deltaOrig, sc.truth), dirDeg: dirErrDeg(deltaOrig, sc.truth), rotDegStable: rotErrDegStable(deltaOrig, sc.truth) };
  const oursJs = { rotDeg: rotErrDeg(o.delta, sc.truth), dirDeg: dirErrDeg(o.delta, sc.truth) };
  results.push({
    n: sc.n, kind: sc.kind, nMatches: sc.matches.length, nGround: ground.length,
    path1279: ground.length < 15 ? 'none' : ground.length * (ground.length - 1) / 2 <= 5 * 73 ? 'enumerate-shuffle' : 'random-set',
    ctorRngSame: ctorSame, classesSame: clsSame, groundSame, mtIndexBefore: mtBefore, mtIndexAfter: mtAfter,
    original: { ret, H, cam, maskOnes: mask.reduce((s, v) => s + v, 0), maskLen: mask.length, delta: deltaOrig,
      translationNorm: tnorm, accepted, ...orig,
      gateB: accepted ? (orig.rotDeg <= CRIT.gateB.rotDeg && orig.dirDeg <= CRIT.gateB.dirDeg ? 'PASS' : 'FAIL') : 'not-accepted' },
    ours: { ret: o.ret1279, trackSuccess: o.track_success, rotDegCpp: o.rot_err_deg, dirDegCpp: o.dir_err_deg,
      rotDegJs: oursJs.rotDeg, dirDegJs: oursJs.dirDeg },
    iso: { errH, errCam, bitIdenticalH: bitSame(H, o.h), bitIdenticalCam: bitSame(cam, o.cam_motion), maskSame, pass: isoPass },
    trackLinkBitSame: link,
  });
}
const noisy = results.filter((r) => r.kind === 'noisy');
const summary = {
  wasm_sha256: PINNED, patched_sha256: sha(patched), code_section_sha256: codeBefore, exports: EXPORTS,
  dump_sha256: sha(dumpBytes), node: process.version, invokedImports, criteria: CRIT,
  checks: {
    ctorRng: { pass: failCtor === 0, nFail: failCtor },
    classes1267: { pass: failCls === 0, nFail: failCls },
    groundList: { pass: failGround === 0, nFail: failGround },
    iso1279: { pass: failIso === 0, nFail: failIso, nScenes: results.length,
      maxErrH: Math.max(...results.map((r) => r.iso.errH)), maxErrCam: Math.max(...results.map((r) => r.iso.errCam)),
      bitIdenticalAll: results.every((r) => r.iso.bitIdenticalH && r.iso.bitIdenticalCam && r.iso.maskSame) },
    trackLink: { pass: failLink === 0, nFail: failLink },
  },
  originalGateB: {
    accepted: noisy.filter((r) => r.original.accepted).map((r) => r.n),
    fail: noisy.filter((r) => r.original.gateB === 'FAIL').map((r) => r.n),
    verdict: noisy.some((r) => r.original.gateB === 'FAIL') ? 'ORIGINAL FAILS (b)' : 'ORIGINAL PASSES (b)',
  },
};
fs.mkdirSync(outDir);
fs.copyFileSync(new URL(import.meta.url), path.join(outDir, 'vfft_b_original_probe.mjs'));
fs.writeFileSync(path.join(outDir, 'summary.json'), JSON.stringify(summary, null, 1));
fs.writeFileSync(path.join(outDir, 'scenes.json'), JSON.stringify(results, null, 1));
console.log(JSON.stringify({ checks: summary.checks, originalGateB: summary.originalGateB, invokedImports }, null, 1));
for (const r of results) {
  console.log(r.n, r.kind.padEnd(5), 'ground', r.nGround, r.path1279.padEnd(17), 'ret', r.original.ret,
    'orig rot', r.original.rotDeg.toFixed(4), '(stable', r.original.rotDegStable.toFixed(4) + ')', 'dir', r.original.dirDeg.toFixed(4), r.original.gateB.padEnd(12),
    'ours(cpp) rot', r.ours.rotDegCpp.toFixed(4), 'dir', r.ours.dirDegCpp.toFixed(4),
    'iso eH', r.iso.errH.toExponential(1), 'eC', r.iso.errCam.toExponential(1), 'bit', r.iso.bitIdenticalH && r.iso.bitIdenticalCam, 'mask', r.iso.maskSame,
    'deltaBit', r.trackLinkBitSame);
}
const allChecks = Object.values(summary.checks).every((c) => c.pass);
process.exitCode = allChecks ? 0 : 1;

// Isolated-function parity probe for MapTracker helpers in the original xr-slam.wasm.
//
// Spec: slam_refact/docs/spec/02-map-tracker.md sections 5.4 and 8.1.
//   func 519  = yawOf(Quaternion*)            -> atan2(m02, m22) with a degenerate retry
//   func 520  = Quaternion::fromHMatrix(out, HMatrix*)  (public c8/quaternion.cc:38)
//   func 1271 = replaceYaw(out HMatrix*, pose HMatrix*, deviceQuat*)
//               R_out = Ry(yaw(pose) - yaw(dev)) * R_dev, t_out = t_pose
// The module is re-exported (export section patched only; code section hash must be unchanged),
// instantiated with throwing import stubs, and called on deterministic test vectors.
//
// Tolerances were fixed before the first run (spec 8.1) and must not be changed afterwards:
const TOL = {
  yaw519Rad: 1e-5,          // |yaw_bin - yaw_ref| (wrapped), non-degenerate and degenerate inputs
  quat520: 1e-6,            // max |q_bin - q_ref| (sign-aligned)
  rot1271: 1e-5,            // max |R_bin - R_ref| over 3x3 forward and inverse blocks
  invTrans1271Rel: 1e-5,    // |t_inv_bin - t_inv_ref| <= tol * (1 + |t|)
  // translation of 1271 output must equal the pose translation exactly (float32 copy + 0.0f).
};
// Usage: node probe-yaw-replace.mjs --input xr-slam.wasm --output NEW_DIRECTORY
import fs from 'node:fs';
import crypto from 'node:crypto';
import path from 'node:path';

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

const leb = (n) => { const b = []; do { let v = n & 127; n >>>= 7; if (n) v |= 128; b.push(v); } while (n); return Buffer.from(b); };
const readLeb = (b, p) => { let n = 0, s = 0, v; do { v = b[p++]; n |= (v & 127) << s; s += 7; } while (v & 128); return [n >>> 0, p]; };
const EXPORTS = [['probeYaw519', 519], ['probeFromHMatrix520', 520], ['probeReplaceYaw1271', 1271]];
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
const invokedImports = [];
const stubs = {};
for (const imp of WebAssembly.Module.imports(mod)) {
  if (imp.kind !== 'function') continue;
  stubs[imp.module] ??= {};
  stubs[imp.module][imp.name] = (...a) => { invokedImports.push(imp.name); throw Error('import called: ' + imp.name); };
}
const inst = new WebAssembly.Instance(mod, stubs);
const mem = inst.exports.memory;
const BASE = 2 ** 21;
const POSE = BASE, QUAT = BASE + 0x100, OUT = BASE + 0x200, QOUT = BASE + 0x300;
const f32 = () => new Float32Array(mem.buffer);
const u8 = () => new Uint8Array(mem.buffer);
const fr = Math.fround;

// ---------- deterministic test vectors ----------
let seed = 20261015 >>> 0;
const rnd = () => { seed = (Math.imul(seed, 1664525) + 1013904223) >>> 0; return seed / 4294967296; };
const randQuat = () => {
  const u1 = rnd(), u2 = rnd() * 2 * Math.PI, u3 = rnd() * 2 * Math.PI;
  const a = Math.sqrt(1 - u1), b = Math.sqrt(u1);
  return [a * Math.sin(u2), a * Math.cos(u2), b * Math.sin(u3), b * Math.cos(u3)]; // x y z w
};
const quatMul = (a, b) => { // (x,y,z,w)
  const [ax, ay, az, aw] = a, [bx, by, bz, bw] = b;
  return [aw * bx + ax * bw + ay * bz - az * by, aw * by - ax * bz + ay * bw + az * bx,
    aw * bz + ax * by - ay * bx + az * bw, aw * bw - ax * bx - ay * by - az * bz];
};
const rotOf = (q) => { // row-major 3x3 from (x,y,z,w)
  const [x, y, z, w] = q;
  return [[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
    [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
    [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]];
};
const axisAngleQuat = (ax, ang) => { const s = Math.sin(ang / 2); return [ax[0] * s, ax[1] * s, ax[2] * s, Math.cos(ang / 2)]; };

const cases = [];
for (let k = 0; k < 200; k++) {
  cases.push({ kind: 'random', qPose: randQuat(), t: [rnd() * 10 - 5, rnd() * 10 - 5, rnd() * 10 - 5], qDev: randQuat() });
}
const ident = [0, 0, 0, 1];
cases.push({ kind: 'identity', qPose: ident, t: [0, 2, 0], qDev: ident });
for (const s of [1, -1]) { // device looking straight up/down: yaw degenerate for 519
  cases.push({ kind: 'dev-pitch' + (s > 0 ? '+90' : '-90'), qPose: randQuat(), t: [1, 2, 3], qDev: axisAngleQuat([1, 0, 0], s * Math.PI / 2) });
  cases.push({ kind: 'pose-pitch' + (s > 0 ? '+90' : '-90'), qPose: axisAngleQuat([1, 0, 0], s * Math.PI / 2), t: [-1, 0.5, 4], qDev: randQuat() });
}

// ---------- reference implementations (spec 5.4) ----------
const yawRef = (q0) => { // literal transcription of func 519 in float64
  let [x, y, z, w] = q0;
  for (let it = 0; it < 8; it++) {
    const m22 = 1 - 2 * (x * x + y * y);
    const m02 = 2 * (w * y + x * z);
    if (Math.sqrt(m22 * m22 + m02 * m02) > 1e-5) return { yaw: Math.atan2(m02, m22), retries: it };
    const r = Math.SQRT1_2;
    const nx = r * (x - w), ny = r * (y - z), nz = r * (z + y), nw = r * (w + x);
    const n = Math.hypot(nx, ny, nz, nw) || 1;
    [x, y, z, w] = [nx / n, ny / n, nz / n, nw / n];
  }
  return { yaw: NaN, retries: -1 };
};
const fromHMatrixRef = (R) => { // public c8/quaternion.cc:38 (double), R row-major 3x3
  const tr = R[0][0] + R[1][1] + R[2][2];
  let q;
  if (tr >= 0) {
    const t = Math.sqrt(tr + 1), s = 0.5 / t;
    q = [s * (R[2][1] - R[1][2]), s * (R[0][2] - R[2][0]), s * (R[1][0] - R[0][1]), 0.5 * t];
  } else {
    let i = 0; if (R[1][1] > R[0][0]) i = 1; if (R[2][2] > R[i][i]) i = 2;
    const j = (i + 1) % 3, k = (j + 1) % 3;
    const t = Math.sqrt(R[i][i] - R[j][j] - R[k][k] + 1), s = 0.5 / t;
    const v = [0, 0, 0]; v[i] = 0.5 * t; v[j] = s * (R[j][i] + R[i][j]); v[k] = s * (R[k][i] + R[i][k]);
    q = [v[0], v[1], v[2], s * (R[k][j] - R[j][k])];
  }
  return q;
};
const replaceYawRef = (Rpose, tPose, qDev) => {
  const qPose = fromHMatrixRef(Rpose);
  const d = (yawRef(qPose).yaw - yawRef(qDev).yaw) * 0.5;
  const qy = [0, Math.sin(d), 0, Math.cos(d)];
  let q = quatMul(qy, qDev);
  const n = Math.hypot(...q) || 1; q = q.map((v) => v / n);
  const R = rotOf(q);
  const tInv = [0, 1, 2].map((r) => -(R[0][r] * tPose[0] + R[1][r] * tPose[1] + R[2][r] * tPose[2]));
  return { R, tInv };
};

// ---------- memory helpers (HMatrix column-major, inverse at +0x40, flags at +0x80/+0x81) ----------
const writePose = (R, t) => {
  const a = f32(); const o = POSE / 4;
  for (let c = 0; c < 4; c++) for (let r = 0; r < 4; r++) {
    const v = c < 3 ? (r < 3 ? R[r][c] : 0) : (r < 3 ? t[r] : 1);
    a[o + 4 * c + r] = v;
  }
  const tInv = [0, 1, 2].map((r) => -(R[0][r] * t[0] + R[1][r] * t[1] + R[2][r] * t[2]));
  for (let c = 0; c < 4; c++) for (let r = 0; r < 4; r++) {
    const v = c < 3 ? (r < 3 ? R[c][r] : 0) : (r < 3 ? tInv[r] : 1);
    a[o + 16 + 4 * c + r] = v;
  }
  u8()[POSE + 0x80] = 0; u8()[POSE + 0x81] = 0;
};
const readMat = (addr) => { const a = f32(); return Array.from(a.subarray(addr / 4, addr / 4 + 32)); };
const wrap = (x) => Math.atan2(Math.sin(x), Math.cos(x));

const results = [];
let worst = { yaw519: 0, quat520: 0, rot1271: 0, invT1271: 0 }, nFail = 0;
for (const cs of cases) {
  const Rf = rotOf(cs.qPose).map((row) => row.map(fr));
  const tf = cs.t.map(fr);
  const qDevF = cs.qDev.map(fr);
  const rec = { kind: cs.kind };
  // 519 on device quaternion (copy, 519 may rewrite its argument)
  f32().set(qDevF, QUAT / 4);
  const yawBin = inst.exports.probeYaw519(QUAT);
  const yr = yawRef(qDevF);
  rec.yaw519 = { bin: yawBin, ref: yr.yaw, retries: yr.retries, err: Math.abs(wrap(yawBin - yr.yaw)) };
  // 520 on pose
  writePose(Rf, tf);
  inst.exports.probeFromHMatrix520(QOUT, POSE);
  const qBin = Array.from(f32().subarray(QOUT / 4, QOUT / 4 + 4));
  const qRef = fromHMatrixRef(Rf);
  const sgn = (qBin[3] * qRef[3] + qBin[0] * qRef[0] + qBin[1] * qRef[1] + qBin[2] * qRef[2]) < 0 ? -1 : 1;
  rec.quat520 = { bin: qBin, ref: qRef, err: Math.max(...qBin.map((v, i) => Math.abs(v - sgn * qRef[i]))) };
  // 1271
  writePose(Rf, tf);
  f32().set(qDevF, QUAT / 4);
  new Uint8Array(mem.buffer, OUT, 0x84).fill(0xAB);
  inst.exports.probeReplaceYaw1271(OUT, POSE, QUAT);
  const m = readMat(OUT);
  const ref = replaceYawRef(Rf, tf, qDevF);
  let eR = 0, eInvR = 0, eInvT = 0;
  for (let r = 0; r < 3; r++) for (let c = 0; c < 3; c++) {
    eR = Math.max(eR, Math.abs(m[4 * c + r] - ref.R[r][c]));
    eInvR = Math.max(eInvR, Math.abs(m[16 + 4 * c + r] - ref.R[c][r]));
  }
  for (let r = 0; r < 3; r++) eInvT = Math.max(eInvT, Math.abs(m[16 + 12 + r] - ref.tInv[r]) / (1 + Math.hypot(...tf)));
  const transExact = [0, 1, 2].every((r) => m[12 + r] === tf[r] + 0);
  const bottomExact = m[3] === 0 && m[7] === 0 && m[11] === 0 && m[15] === 1 && m[19] === 0 && m[23] === 0 && m[27] === 0 && m[31] === 1;
  const flags = [u8()[OUT + 0x80], u8()[OUT + 0x81]];
  rec.r1271 = { rotErr: Math.max(eR, eInvR), invTransRelErr: eInvT, transExact, bottomExact, flags };
  const ok = rec.yaw519.err <= TOL.yaw519Rad && rec.quat520.err <= TOL.quat520 && Math.max(eR, eInvR) <= TOL.rot1271
    && eInvT <= TOL.invTrans1271Rel && transExact && bottomExact && flags[0] === 0 && flags[1] === 0;
  rec.pass = ok;
  if (!ok) nFail++;
  worst = {
    yaw519: Math.max(worst.yaw519, rec.yaw519.err), quat520: Math.max(worst.quat520, rec.quat520.err),
    rot1271: Math.max(worst.rot1271, Math.max(eR, eInvR)), invT1271: Math.max(worst.invT1271, eInvT),
  };
  results.push(rec);
}
fs.mkdirSync(outDir);
fs.copyFileSync(new URL(import.meta.url), path.join(outDir, 'probe-yaw-replace.mjs'));
const summary = {
  wasm_sha256: PINNED, patched_sha256: sha(patched), code_section_sha256: codeBefore,
  exports: EXPORTS, tolerances: TOL, n_cases: cases.length, n_fail: nFail, worst, invokedImports,
  node: process.version,
};
fs.writeFileSync(path.join(outDir, 'summary.json'), JSON.stringify(summary, null, 1));
fs.writeFileSync(path.join(outDir, 'cases.json'), JSON.stringify(results, null, 1));
console.log(JSON.stringify(summary, null, 1));
process.exitCode = nFail === 0 ? 0 : 1;

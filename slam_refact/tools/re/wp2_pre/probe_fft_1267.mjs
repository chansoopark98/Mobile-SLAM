// WP2-PRE item 1 (S4 U6, second half): "1267 lift sign". Isolated execution of func 1267
// (FrameFrameTracker horizon classification, called by 1266 and 926) in the ORIGINAL xr-slam.wasm.
// Signature (dcmp f_tva / G1267): (std::vector<int32>* out, const Quaternion* q, const std::vector<HPoint2>* rays).
//
// Hypotheses compared (fixed before the first run):
//   H_A: v = z of  xDegrees(-90) * R(q) * (x, y, w, 1)   (gravity-normal lift, as in homography.cc:363)
//   H_B: v = z of  R(q) * xDegrees(-90) * (x, y, w, 1)
//   class(v) = 1 if v >= 0.2; 2 if 0 < v < 0.2; 3 if v == 0; 4 if -0.2 < v < 0; 5 otherwise (G1267:211-356)
// Criterion: the binary's class vector equals exactly one hypothesis on every case, except cases whose
// |v| lies within 1e-6 of a threshold (0.2 / 0 / -0.2), which are reported separately.
// Usage: node probe_fft_1267.mjs --input xr-slam.wasm --output NEW_DIRECTORY
import fs from 'node:fs';
import crypto from 'node:crypto';
import path from 'node:path';

const argv = process.argv.slice(2);
if (argv.length !== 4 || argv[0] !== '--input' || argv[2] !== '--output') throw Error('Usage: --input xr-slam.wasm --output NEW_DIRECTORY');
const outDir = path.resolve(argv[3]);
if (outDir.startsWith('/mnt/backup/SLAM')) throw Error('read-only dataset root');
if (fs.existsSync(outDir)) throw Error('Output already exists; preserving evidence');
const original = fs.readFileSync(path.resolve(argv[1]));
const sha = (b) => crypto.createHash('sha256').update(b).digest('hex');
const PINNED = '780731fe294a725acf1b7717ed5ffb07ad9f0929e408a1d74f47450c291ec422';
if (sha(original) !== PINNED) throw Error('Input differs from pinned xr-slam.wasm');
const leb = (n) => { const b = []; do { let v = n & 127; n >>>= 7; if (n) v |= 128; b.push(v); } while (n); return Buffer.from(b); };
const readLeb = (b, p) => { let n = 0, s = 0, v; do { v = b[p++]; n |= (v & 127) << s; s += 7; } while (v & 128); return [n >>> 0, p]; };
const EXPORTS = [['wp2Horizon1267', 1267]];
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
  } else parts.push(original.subarray(start, pos));
  if (id === 10) codeAfter = sha(payload);
}
if (codeBefore === null || codeBefore !== codeAfter) throw Error('code section changed');
const mod = new WebAssembly.Module(Buffer.concat(parts));
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
const u32 = () => new Uint32Array(X.memory.buffer);
const i32 = () => new Int32Array(X.memory.buffer);
const f32 = () => new Float32Array(X.memory.buffer);
const malloc = (n) => { const p = X.malloc(n); new Uint8Array(X.memory.buffer).fill(0, p, p + n); return p; };

const fr = Math.fround;
const quatToR = ([x, y, z, w]) => [[fr(1 - 2 * (y * y + z * z)), fr(2 * (x * y - w * z)), fr(2 * (x * z + w * y))],
  [fr(2 * (x * y + w * z)), fr(1 - 2 * (x * x + z * z)), fr(2 * (y * z - w * x))],
  [fr(2 * (x * z - w * y)), fr(2 * (y * z + w * x)), fr(1 - 2 * (x * x + y * y))]];
const rad = fr(fr(fr(Math.PI) * -90) / 180);
const c = fr(Math.cos(rad)), s = fr(Math.sin(rad));
const XD = [[1, 0, 0], [0, c, -s], [0, s, c]];
const mul3 = (A, B) => A.map((r) => [0, 1, 2].map((j) => r[0] * B[0][j] + r[1] * B[1][j] + r[2] * B[2][j]));
const cls = (v) => (v >= 0.2 ? 1 : v > 0 ? 2 : v === 0 ? 3 : v > -0.2 ? 4 : 5);
const near = (v) => [0.2, 0, -0.2].some((t) => Math.abs(v - t) <= 1e-6);

let seed = 1267;
const rnd = () => { seed = (Math.imul(seed, 1664525) + 1013904223) >>> 0; return seed / 4294967296; };
const randQuat = () => { const u1 = rnd(), u2 = rnd() * 2 * Math.PI, u3 = rnd() * 2 * Math.PI; const a = Math.sqrt(1 - u1), b = Math.sqrt(u1);
  return [a * Math.sin(u2), a * Math.cos(u2), b * Math.sin(u3), b * Math.cos(u3)].map(fr); };
const axq = (ax, ang) => [ax[0] * Math.sin(ang / 2), ax[1] * Math.sin(ang / 2), ax[2] * Math.sin(ang / 2), Math.cos(ang / 2)].map(fr);

const qp = malloc(16), vin = malloc(12), vout = malloc(12);
const cases = [];
const quats = [[0, 0, 0, 1], axq([1, 0, 0], Math.PI / 4), axq([1, 0, 0], -Math.PI / 4), axq([1, 0, 0], Math.PI / 2)];
for (let k = 0; k < 40; k++) quats.push(randQuat());
let nA = 0, nB = 0, nNear = 0, nTotal = 0;
for (const q of quats) {
  const rays = [];
  for (let k = 0; k < 64; k++) rays.push([fr((rnd() - 0.5) * 1.6), fr((rnd() - 0.5) * 1.6), 1]);
  rays.push([0, 0, 1], [0, fr(0.2), 1], [0, fr(-0.2), 1]);
  const buf = malloc(rays.length * 12);
  rays.forEach((r, k) => f32().set(r, buf / 4 + 3 * k));
  u32().set([buf, buf + rays.length * 12, buf + rays.length * 12], vin / 4);
  u32().set([0, 0, 0], vout / 4);
  f32().set(q, qp / 4);
  X.wp2Horizon1267(vout, qp, vin);
  const b = u32()[vout / 4], e = u32()[vout / 4 + 1];
  const bin = Array.from(i32().subarray(b / 4, e / 4));
  const R = quatToR(q);
  const MA = mul3(XD, R), MB = mul3(R, XD);
  const vA = rays.map((r) => MA[2][0] * r[0] + MA[2][1] * r[1] + MA[2][2] * r[2]);
  const vB = rays.map((r) => MB[2][0] * r[0] + MB[2][1] * r[1] + MB[2][2] * r[2]);
  let okA = bin.length === rays.length, okB = okA, nearHere = 0;
  rays.forEach((_, k) => {
    if (near(vA[k]) || near(vB[k])) { nearHere++; return; }
    if (bin[k] !== cls(vA[k])) okA = false;
    if (bin[k] !== cls(vB[k])) okB = false;
  });
  nTotal += rays.length; nNear += nearHere;
  if (okA) nA++;
  if (okB) nB++;
  cases.push({ q, nRays: rays.length, outLen: bin.length, okA, okB, nearSkipped: nearHere, hist: [1, 2, 3, 4, 5].map((c) => bin.filter((v) => v === c).length) });
}
const summary = { wasm_sha256: PINNED, code_section_sha256: codeBefore, exports: EXPORTS, invokedImports, node: process.version,
  nQuats: quats.length, nRays: nTotal, nearThresholdSkipped: nNear, quatsMatchingA: nA, quatsMatchingB: nB,
  verdict: nA === quats.length && nB < quats.length ? 'H_A' : nB === quats.length && nA < quats.length ? 'H_B' : 'UNDECIDED' };
fs.mkdirSync(outDir);
fs.copyFileSync(new URL(import.meta.url), path.join(outDir, 'probe_fft_1267.mjs'));
fs.writeFileSync(path.join(outDir, 'summary.json'), JSON.stringify(summary, null, 1));
fs.writeFileSync(path.join(outDir, 'cases.json'), JSON.stringify(cases, null, 1));
console.log(JSON.stringify(summary, null, 1));
process.exitCode = summary.verdict === 'UNDECIDED' ? 1 : 0;

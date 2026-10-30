// Isolated parity probe (lane S3): runs the ORIGINAL xr-slam.wasm function 1613
// (= c8::TriangulatePointMultiviewAnalytic::Evaluate, vtable slot 3 of RTTI
// N2c833TriangulatePointMultiviewAnalyticE) on fixed test vectors and compares it
// with the public formula (reality/engine/geometry/bundle-residual-analytic.cc).
//
// Only the export section of an in-memory/analytical copy is changed (adds one export);
// code bytes are hashed before/after to prove they are unchanged.
//
// Usage (cwd = repo root):
//   node slam_refact/tools/re/spec_map_builder/triangulate_probe.mjs \
//     --input build/slam-refact-20261015/inputs/xr-slam.wasm \
//     --output build/slam-refact-20261015/spec/map-builder/probes/triangulate-<label>
//
// Tolerance fixed BEFORE the first run (spec 03-map-builder.md section 8):
//   |wasm - ref| <= 1e-12 * max(1, |ref|) for both residuals and Jacobian entries.
import fs from 'node:fs';
import crypto from 'node:crypto';
import path from 'node:path';

const kPinnedSha = '780731fe294a725acf1b7717ed5ffb07ad9f0929e408a1d74f47450c291ec422';
const kFuncIndex = 1613;
const kRelTol = 1e-12;

const argv = process.argv.slice(2);
if (argv.length !== 4 || argv[0] !== '--input' || argv[2] !== '--output') {
  throw Error('Usage: --input xr-slam.wasm --output NEW_DIRECTORY');
}
const input = path.resolve(argv[1]);
const outputDir = path.resolve(argv[3]);
if (fs.existsSync(outputDir)) throw Error('Output already exists; preserving evidence');
const hash = (b) => crypto.createHash('sha256').update(b).digest('hex');
const original = fs.readFileSync(input);
if (hash(original) !== kPinnedSha) throw Error('Input differs from pinned xr-slam.wasm');

const leb = (n) => {
  const b = [];
  do { let v = n & 127; n >>>= 7; if (n) v |= 128; b.push(v); } while (n);
  return Buffer.from(b);
};
const readLeb = (b, pos) => {
  let n = 0, shift = 0, v;
  do { v = b[pos++]; n |= (v & 127) << shift; shift += 7; } while (v & 128);
  return [n >>> 0, pos];
};

// Append one function export.
const parts = [original.subarray(0, 8)];
let pos = 8, codeBefore = null, codeAfter = null;
while (pos < original.length) {
  const start = pos, id = original[pos++];
  const [length, p] = readLeb(original, pos);
  pos = p + length;
  let payload = original.subarray(p, pos);
  if (id === 10) codeBefore = hash(payload);
  if (id === 7) {
    const [count, rest] = readLeb(payload, 0);
    const name = 'probeTriangulateEvaluate';
    const extra = Buffer.concat([leb(name.length), Buffer.from(name), Buffer.from([0]), leb(kFuncIndex)]);
    payload = Buffer.concat([leb(count + 1), payload.subarray(rest), extra]);
    parts.push(Buffer.concat([Buffer.from([id]), leb(payload.length), payload]));
  } else {
    parts.push(original.subarray(start, pos));
  }
  if (id === 10) codeAfter = hash(payload);
}
const patched = Buffer.concat(parts);
const compiled = new WebAssembly.Module(patched);
const invokedImports = [];
const stubs = {};
for (const imp of WebAssembly.Module.imports(compiled)) {
  if (imp.kind !== 'function') throw Error('Unexpected non-function import ' + imp.name);
  stubs[imp.module] ??= {};
  stubs[imp.module][imp.name] = (...args) => {
    invokedImports.push({name: imp.name, args});
    throw Error('Unexpected imported call ' + imp.name);
  };
}
const instance = new WebAssembly.Instance(compiled, stubs);
const mem = () => new DataView(instance.exports.memory.buffer);

// Deterministic PRNG (mulberry32) so test vectors are fixed.
function mulberry32(seed) {
  return () => {
    seed |= 0; seed = (seed + 0x6D2B79F5) | 0;
    let t = Math.imul(seed ^ (seed >>> 15), 1 | seed);
    t = (t + Math.imul(t ^ (t >>> 7), 61 | t)) ^ t;
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  };
}
const rnd = mulberry32(20261015);
const u = (a, b) => a + (b - a) * rnd();

// Public reference (bundle-residual-analytic.cc TriangulatePointMultiviewAnalytic::Evaluate).
function reference(ray, proj, pt) {
  const [ptx, pty, ptz] = pt;
  const x0 = proj[10] * ptz + proj[11] + proj[8] * ptx + proj[9] * pty;
  const x1 = 1.0 / x0;
  const x2 = proj[0] * ptx + proj[1] * pty + proj[2] * ptz + proj[3] - ray[0] * x0;
  const x3 = proj[4] * ptx + proj[5] * pty + proj[6] * ptz + proj[7] - ray[1] * x0;
  const x4 = Math.pow(x0, -2);
  const r = [x1 * x2, x1 * x3];
  const j = [
    x4 * (-proj[8] * x2 + x0 * (proj[0] - proj[8] * ray[0])),
    x4 * (-proj[9] * x2 + x0 * (proj[1] - proj[9] * ray[0])),
    x4 * (-proj[10] * x2 + x0 * (-proj[10] * ray[0] + proj[2])),
    x4 * (-proj[8] * x3 + x0 * (proj[4] - proj[8] * ray[1])),
    x4 * (-proj[9] * x3 + x0 * (proj[5] - proj[9] * ray[1])),
    x4 * (-proj[10] * x3 + x0 * (-proj[10] * ray[1] + proj[6])),
  ];
  return {r, j};
}

// Test vectors: random camera (rotation via small Euler angles, translation), point in front.
function rotation(ax, ay, az) {
  const [cx, sx, cy, sy, cz, sz] = [Math.cos(ax), Math.sin(ax), Math.cos(ay), Math.sin(ay), Math.cos(az), Math.sin(az)];
  return [
    [cy * cz, -cy * sz, sy],
    [sx * sy * cz + cx * sz, -sx * sy * sz + cx * cz, -sx * cy],
    [-cx * sy * cz + sx * sz, cx * sy * sz + sx * cz, cx * cy],
  ];
}
const vectors = [];
for (let n = 0; n < 64; ++n) {
  const R = rotation(u(-0.5, 0.5), u(-0.5, 0.5), u(-0.5, 0.5));
  const t = [u(-1, 1), u(-1, 1), u(-1, 1)];
  // projection_ = [R | t] row-major 3x4 (camera_from_world), as built by the caller.
  const proj = [R[0][0], R[0][1], R[0][2], t[0], R[1][0], R[1][1], R[1][2], t[1], R[2][0], R[2][1], R[2][2], t[2]];
  const pt = [u(-2, 2), u(-2, 2), u(3, 8)];
  const ray = [u(-0.6, 0.6), u(-0.6, 0.6)];
  vectors.push({ray, proj, pt});
}

// Object layout recovered from func 1613 (VERIFIED by the decompile):
//   +0x00 vptr, +0x04..+0x13 ceres::CostFunction base, +0x18 ray_[2] (f64), +0x28 projection_[12] (f64)
const base = 2 ** 21;
const obj = base, params = base + 512, ptBuf = base + 640, resid = base + 768, jacArr = base + 896, jac = base + 1024;
const results = [];
for (const [k, v] of vectors.entries()) {
  for (const withJac of [false, true]) {
    new Uint8Array(instance.exports.memory.buffer, base, 2048).fill(0);
    const d = mem();
    d.setFloat64(obj + 0x18, v.ray[0], true);
    d.setFloat64(obj + 0x20, v.ray[1], true);
    v.proj.forEach((x, i) => d.setFloat64(obj + 0x28 + 8 * i, x, true));
    v.pt.forEach((x, i) => d.setFloat64(ptBuf + 8 * i, x, true));
    d.setUint32(params, ptBuf, true);
    d.setUint32(jacArr, jac, true);
    const ret = instance.exports.probeTriangulateEvaluate(obj, params, resid, withJac ? jacArr : 0);
    const d2 = mem();
    const r = [0, 1].map((i) => d2.getFloat64(resid + 8 * i, true));
    const j = withJac ? [0, 1, 2, 3, 4, 5].map((i) => d2.getFloat64(jac + 8 * i, true)) : null;
    const ref = reference(v.ray, v.proj, v.pt);
    const ok = (a, b) => Math.abs(a - b) <= kRelTol * Math.max(1, Math.abs(b));
    const pass = ret === 1 && r.every((x, i) => ok(x, ref.r[i])) && (!withJac || j.every((x, i) => ok(x, ref.j[i])));
    const bitExact = r.every((x, i) => x === ref.r[i]) && (!withJac || j.every((x, i) => x === ref.j[i]));
    results.push({k, withJac, ret, r, refR: ref.r, j, refJ: withJac ? ref.j : null, pass, bitExact});
  }
}
fs.mkdirSync(outputDir, {recursive: true});
const maxAbsDiff = Math.max(...results.flatMap((x) => [
  ...x.r.map((y, i) => Math.abs(y - x.refR[i])),
  ...(x.j ? x.j.map((y, i) => Math.abs(y - x.refJ[i])) : []),
]));
const summary = {
  func: kFuncIndex,
  rtti: 'c8::TriangulatePointMultiviewAnalytic::Evaluate',
  node: process.version,
  original_sha256: hash(original),
  code_section_unchanged: codeBefore === codeAfter,
  tolerance_rel: kRelTol,
  cases: results.length,
  passed: results.filter((x) => x.pass).length,
  bit_exact: results.filter((x) => x.bitExact).length,
  max_abs_diff: maxAbsDiff,
  invokedImports,
  pass: results.every((x) => x.pass) && invokedImports.length === 0 && codeBefore === codeAfter,
  limits: [
    'Component call with constructed object memory; no engine initialization',
    'Does not prove where/how often 936 builds this residual or its loss/solver options',
  ],
};
fs.writeFileSync(path.join(outputDir, 'result.json'), JSON.stringify({summary, results}, null, 1) + '\n');
fs.copyFileSync(new URL(import.meta.url), path.join(outputDir, 'triangulate_probe.mjs'));
console.log(JSON.stringify(summary, null, 2));
if (!summary.pass) process.exitCode = 1;

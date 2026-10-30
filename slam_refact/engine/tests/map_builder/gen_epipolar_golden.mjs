// MB-EPI golden generator (architecture.md §6.6.3, S3 §8.5).
//
// Runs functions of the ORIGINAL xr-slam.wasm in isolation (same export-adding method as
// slam_refact/tools/re/spec_map_builder/triangulate_probe.mjs: only the export section changes,
// the code section hash is checked before/after) on fixed inputs and records their raw outputs:
//   E1115  epipolarDepthImagePrework(wordsCam, dictionaryCam)            (public epipolar.cc)
//   E1116  epipolarDepthPointPrework(iw, wordsRay)                       (public epipolar.cc)
//   E1118  depthOfEpipolarPoint(iw, pw, dictionaryPt, scale)             (public epipolar.cc)
//   E1123  1122 triangulation prework(poseOwn, poseOther) + 1123 ray-ray point (closed)
//   S1121 / S1120  libc++ std::sort of 20-byte PointMatch records, comparator of 1112
// Struct inputs of E1116/E1118 are the ORIGINAL outputs of the previous stage (isolation): an
// E1116 input names its E1115 case (iw = that OUT), an E1118 input names its E1116 case (pw = that
// OUT, iw = the OUT of the E1115 case named by that E1116 case).
//
// Pre-registered gate (fixed before the first run, architecture.md §6.6.3 MB-EPI):
//   float/double fields: |ours - wasm| <= 1e-6 * max(1, |wasm|); Status, bools: exact;
//   E1123 output: same relative rule; S1120/S1121: output order exactly equal (inputs have
//   unique (dist, scale, wordsIdx, dictIdx) keys; tie order is not claimed, §5.3).
//
// Output (new directory; never overwritten):
//   golden.tsv    KIND <tab> case <tab> IN:<hex u32 words, comma> <tab> OUT:<hex u32 words, comma>
//   summary.json  hashes, counts, tolerance text, invoked imports
//
// Usage (cwd = repo root):
//   node slam_refact/engine/tests/map_builder/gen_epipolar_golden.mjs \
//     --input build/slam-refact-20261015/inputs/xr-slam.wasm \
//     --output slam_refact/engine/tests/data/map-builder/probes/epipolar-r1
import fs from 'node:fs';
import crypto from 'node:crypto';
import path from 'node:path';

const kPinnedSha = '780731fe294a725acf1b7717ed5ffb07ad9f0929e408a1d74f47450c291ec422';
const kTolerance = '|ours - wasm| <= 1e-6 * max(1, |wasm|) for float/double fields; ' +
  'Status/bool exact; sort order exact (unique keys)';
const kSeed = 20261016;

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

const kExports = {f1115: 1115, f1116: 1116, f1118: 1118, f1120: 1120, f1121: 1121, f1122: 1122,
                  f1123: 1123};
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
    const extras = Object.entries(kExports).map(([name, idx]) =>
      Buffer.concat([leb(name.length), Buffer.from(name), Buffer.from([0]), leb(idx)]));
    payload = Buffer.concat([leb(count + extras.length), payload.subarray(rest), ...extras]);
    parts.push(Buffer.concat([Buffer.from([id]), leb(payload.length), payload]));
  } else {
    parts.push(original.subarray(start, pos));
  }
  if (id === 10) codeAfter = hash(payload);
}
const compiled = new WebAssembly.Module(Buffer.concat(parts));
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
const E = instance.exports;
const view = () => new DataView(E.memory.buffer);
// Scratch area well above the static data and the initial stack pointer (global[0] = 5862416).
const kBase = 8 * 1024 * 1024;

function mulberry32(seed) {
  return () => {
    seed |= 0; seed = (seed + 0x6D2B79F5) | 0;
    let t = Math.imul(seed ^ (seed >>> 15), 1 | seed);
    t = (t + Math.imul(t ^ (t >>> 7), 61 | t)) ^ t;
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  };
}
const rnd = mulberry32(kSeed);
const u = (a, b) => a + (b - a) * rnd();
const f32 = Math.fround;

function rotation(ax, ay, az) {
  const [cx, sx, cy, sy, cz, sz] =
    [Math.cos(ax), Math.sin(ax), Math.cos(ay), Math.sin(ay), Math.cos(az), Math.sin(az)];
  return [
    [cy * cz, -cy * sz, sy],
    [sx * sy * cz + cx * sz, -sx * sy * sz + cx * cz, -sx * cy],
    [-cx * sy * cz + sx * sz, cx * sy * sz + sx * cz, cx * cy],
  ];
}
// world_from_camera HMatrix words: column-major forward, column-major inverse (float32).
function pose(R, t) {
  const m = new Array(16).fill(0), inv = new Array(16).fill(0);
  for (let r = 0; r < 3; ++r) {
    for (let c = 0; c < 3; ++c) { m[c * 4 + r] = f32(R[r][c]); inv[c * 4 + r] = f32(R[c][r]); }
    m[12 + r] = f32(t[r]);
    let s = 0;
    for (let c = 0; c < 3; ++c) s += R[c][r] * t[c];
    inv[12 + r] = f32(-s);
  }
  m[15] = 1; inv[15] = 1;
  return {R, t, words: [...m, ...inv]};
}
function randomPose(rotScale = 0.4, tScale = 1.0) {
  return pose(rotation(u(-rotScale, rotScale), u(-rotScale, rotScale), u(-rotScale, rotScale)),
              [u(-tScale, tScale), u(-tScale, tScale), u(-tScale, tScale)]);
}

const f2u = (x) => { const d = new DataView(new ArrayBuffer(4)); d.setFloat32(0, x, true); return d.getUint32(0, true); };
const hex = (w) => (w >>> 0).toString(16).padStart(8, '0');

function writeWords(addr, words, asFloat) {
  const d = view();
  words.forEach((x, i) => {
    if (asFloat) d.setFloat32(addr + 4 * i, x, true); else d.setUint32(addr + 4 * i, x >>> 0, true);
  });
}
function readWords(addr, n) {
  const d = view();
  return Array.from({length: n}, (_, i) => d.getUint32(addr + 4 * i, true));
}
function writeHMatrix(addr, words) {
  writeWords(addr, words, true);
  view().setUint8(addr + 128, 0);
  view().setUint8(addr + 129, 0);
}
const clear = () => new Uint8Array(E.memory.buffer, kBase, 1 << 16).fill(0);

const lines = [];
const counts = {};
const emit = (kind, k, inWords, outWords) => {
  counts[kind] = (counts[kind] ?? 0) + 1;
  lines.push(`${kind}\t${k}\tIN:${inWords.map(hex).join(',')}\tOUT:${outWords.map(hex).join(',')}`);
};

// ---- E1115 / E1116 / E1118 ----
const kIwWords = 73;   // 292 B EpipolarDepthImagePrework (wasm32)
const kPwWords = 20;   // 0x50 B EpipolarDepthPointPrework
const kResWords = 6;   // 24 B EpipolarDepthResult
let caseE1116 = 0, caseE1118 = 0;
for (let k = 0; k < 64; ++k) {
  const words = randomPose();
  let dict;
  if (k % 16 === 5) dict = pose(words.R, words.t);                        // pure rotation (same pose)
  else if (k % 16 === 9) dict = pose(rotation(u(-0.3, 0.3), 1.3, u(-0.3, 0.3)), words.t.map((x) => x + 0.2));  // low overlap
  else dict = pose(rotation(u(-0.3, 0.3), u(-0.3, 0.3), u(-0.3, 0.3)),
                   words.t.map((x) => x + u(-0.4, 0.4)));
  clear();
  const aW = kBase, aD = kBase + 256, out = kBase + 512;
  writeHMatrix(aW, words.words);
  writeHMatrix(aD, dict.words);
  E.f1115(out, aW, aD);
  const iw = readWords(out, kIwWords);
  emit('E1115', k, [...words.words, ...dict.words].map(f2u), iw);
  for (let j = 0; j < 4; ++j) {
    const ray = [f32(u(-0.6, 0.6)), f32(u(-0.45, 0.45))];
    clear();
    const aIw = kBase, aRay = kBase + 512, aPw = kBase + 1024;
    writeWords(aIw, iw, false);
    writeWords(aRay, [ray[0], ray[1], 1.0], true);
    E.f1116(aPw, aIw, aRay);
    const pw = readWords(aPw, kPwWords);
    const pwCase = caseE1116++;
    emit('E1116', pwCase, [k, f2u(ray[0]), f2u(ray[1])], pw);
    // Dictionary points: on the segment (with small noise) and off it; all four scales.
    const dv = view();
    const pwf = Array.from({length: kPwWords}, (_, i) => dv.getFloat32(aPw + 4 * i, true));
    for (let q = 0; q < 2; ++q) {
      const a = u(-0.1, 1.1);
      let x = pwf[0] + a * (pwf[3] - pwf[0]) + u(-0.004, 0.004);
      let y = pwf[1] + a * (pwf[4] - pwf[1]) + u(-0.004, 0.004);
      if (!Number.isFinite(x) || Math.abs(x) > 10 || q === 1 && j === 3) { x = u(-0.6, 0.6); y = u(-0.45, 0.45); }
      const pt = [f32(x), f32(y)];
      const scale = Math.floor(u(0, 4)) & 3;
      clear();
      const bIw = kBase, bPw = kBase + 512, bOut = kBase + 1024;
      writeWords(bIw, iw, false);
      writeWords(bPw, pw, false);
      E.f1118(bOut, bIw, bPw, pt[0], pt[1], scale);
      emit('E1118', caseE1118++, [pwCase, f2u(pt[0]), f2u(pt[1]), scale], readWords(bOut, kResWords));
    }
  }
}

// ---- E1123 (with 1122 prework): rays of a common world point seen by two cameras ----
const kPreWords = 136;  // 544 B
for (let k = 0; k < 128; ++k) {
  const A = randomPose(0.35, 0.8);
  const B = pose(rotation(u(-0.35, 0.35), u(-0.35, 0.35), u(-0.35, 0.35)),
                 A.t.map((x) => x + u(-0.5, 0.5)));
  // World point in front of A.
  const pc = [u(-1, 1), u(-0.8, 0.8), u(2, 6)];
  const X = [0, 1, 2].map((r) => A.R[r][0] * pc[0] + A.R[r][1] * pc[1] + A.R[r][2] * pc[2] + A.t[r]);
  const project = (P) => {
    const d = [0, 1, 2].map((i) => X[i] - P.t[i]);
    const c = [0, 1, 2].map((r) => P.R[0][r] * d[0] + P.R[1][r] * d[1] + P.R[2][r] * d[2]);
    return [f32(c[0] / c[2] + u(-0.002, 0.002)), f32(c[1] / c[2] + u(-0.002, 0.002))];
  };
  const rA = project(A), rB = project(B);
  clear();
  const aA = kBase, aB = kBase + 256, pre = kBase + 512, ra = kBase + 1200, rb = kBase + 1216,
        out = kBase + 1232;
  writeHMatrix(aA, A.words);
  writeHMatrix(aB, B.words);
  E.f1122(pre, aA, aB);
  writeWords(ra, [rA[0], rA[1], 1.0, 1.0], true);
  writeWords(rb, [rB[0], rB[1], 1.0, 1.0], true);
  E.f1123(out, ra, rb, pre);
  emit('E1123', k, [...A.words, ...B.words, rA[0], rA[1], rB[0], rB[1]].map(f2u),
       [...readWords(pre, kPreWords), ...readWords(out, 4)]);
}

// ---- S1121 / S1120: 20-byte {u32 wordsIdx, u32 dictIdx, f32 rotation, f32 dist, u8 scale} ----
const sizes = [2, 3, 4, 5, 6, 7, 8, 12, 17, 30, 31, 32, 33, 40, 64, 100, 129, 257, 600];
for (const [kind, fn] of [['S1121', E.f1121], ['S1120', E.f1120]]) {
  sizes.forEach((n, k) => {
    const seen = new Set();
    const recs = [];
    while (recs.length < n) {
      const r = {w: Math.floor(u(0, 60)), d: Math.floor(u(0, 60)), rot: f32(u(-3, 3)),
                 dist: Math.floor(u(0, 8)), s: Math.floor(u(0, 4))};
      const key = `${r.dist}/${r.s}/${r.w}/${r.d}`;
      if (seen.has(key)) continue;
      seen.add(key);
      recs.push(r);
    }
    const words = recs.flatMap((r) => [r.w, r.d, f2u(r.rot), f2u(r.dist), r.s]);
    clear();
    const first = kBase, last = kBase + 20 * n;
    writeWords(first, words, false);
    let depth = 0;
    if (20 * n > 20) {
      let m = n, i = 0, more;
      do { ++i; more = 3 < m; m >>>= 1; } while (more);
      depth = 2 * i;
    }
    fn(first, last, depth);
    const outWords = readWords(first, 5 * n).map((w, i) => (i % 5 === 4 ? w & 0xff : w));
    emit(kind, k, words, outWords);
  });
}

fs.mkdirSync(outputDir, {recursive: true});
fs.writeFileSync(path.join(outputDir, 'golden.tsv'),
                 '# MB-EPI golden (gen_epipolar_golden.mjs). Tolerance: ' + kTolerance + '\n' +
                 lines.join('\n') + '\n');
const summary = {
  generator: 'slam_refact/engine/tests/map_builder/gen_epipolar_golden.mjs',
  generator_sha256: hash(fs.readFileSync(new URL(import.meta.url))),
  node: process.version,
  original_sha256: hash(original),
  code_section_unchanged: codeBefore === codeAfter,
  seed: kSeed,
  tolerance: kTolerance,
  counts,
  invokedImports,
  pass: invokedImports.length === 0 && codeBefore === codeAfter,
};
fs.writeFileSync(path.join(outputDir, 'summary.json'), JSON.stringify(summary, null, 1) + '\n');
console.log(JSON.stringify(summary, null, 1));
if (!summary.pass) process.exitCode = 1;

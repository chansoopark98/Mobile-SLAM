#!/usr/bin/env node
// Divergence lane: run the ORIGINAL xr-slam.wasm 1267 (classifyHorizon) and 1279 (robustGroundHomography)
// on the exact per-call FFT inputs our engine logged (SR_DIV_FFT_LOG + SR_DIV_FFT_FULL, diagnostic build).
// Isolation method and module patch = vfft_b_original_probe.mjs (export section only, code section sha
// unchanged; imports trap except emscripten_get_now / emscripten_memcpy_big). 1279 runs on a freshly
// constructed MapTracker (func 640): its mt19937 is in the constructed state, which equals the engine's
// FFT generator state only when no earlier call of this engine instance drew from it (n >= 15 never
// happened before) - (superseded: see the generator note at mtObj below).
// Pre-registered checks: classes equal (exact); ground list equal (exact); 1279 ret equal; delta t equal
// to engine log within 1e-6 (log prints %.9g of f32).
//   node fft_inputs_original_probe.mjs --log <...-fft.jsonl> --output <new dir> [--input xr-slam.wasm]
import fs from 'node:fs';
import crypto from 'node:crypto';
import path from 'node:path';

const args = Object.fromEntries(process.argv.slice(2).reduce((acc, a, i, all) =>
  (a.startsWith('--') ? [...acc, [a.slice(2), all[i + 1]]] : acc), []));
const input = path.resolve(args.input ?? path.resolve(path.dirname(new URL(import.meta.url).pathname), '../../../../build/slam-refact-20261015/inputs/xr-slam.wasm'));
const outDir = path.resolve(args.output);
if (fs.existsSync(outDir)) throw Error('Output already exists');
const sha = (b) => crypto.createHash('sha256').update(b).digest('hex');
const original = fs.readFileSync(input);
if (sha(original) !== '780731fe294a725acf1b7717ed5ffb07ad9f0929e408a1d74f47450c291ec422') throw Error('not the pinned wasm');
const log = fs.readFileSync(args.log, 'utf8').trim().split('\n').map((l) => JSON.parse(l));

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
  } else parts.push(original.subarray(start, pos));
  if (id === 10) codeAfter = sha(payload);
}
if (codeBefore !== codeAfter) throw Error('code section changed');
const mod = new WebAssembly.Module(Buffer.concat(parts));
let clock = 0, inst = null;
const stubs = {};
for (const imp of WebAssembly.Module.imports(mod)) {
  if (imp.kind !== 'function') continue;
  stubs[imp.module] ??= {};
  stubs[imp.module][imp.name] = (...a) => {
    if (imp.name === 'emscripten_get_now') { clock += 0.001; return clock; }
    if (imp.name === 'emscripten_memcpy_big') { new Uint8Array(inst.exports.memory.buffer).copyWithin(a[0], a[1], a[1] + a[2]); return a[0]; }
    throw Error('import called: ' + imp.name);
  };
}
inst = new WebAssembly.Instance(mod, stubs);
const X = inst.exports;
const u8 = () => new Uint8Array(X.memory.buffer), u32 = () => new Uint32Array(X.memory.buffer);
const i32 = () => new Int32Array(X.memory.buffer), f32 = () => new Float32Array(X.memory.buffer);
const malloc = (n) => { const p = X.malloc(n); if (!p) throw Error('malloc'); u8().fill(0, p, p + n); return p; };
const writeVec = (vec, pts) => {
  const buf = malloc(Math.max(12, pts.length * 12));
  pts.forEach((p, k) => f32().set(p, buf / 4 + 3 * k));
  u32().set([buf, buf + pts.length * 12, buf + pts.length * 12], vec / 4);
};
const readMat = (p) => { const a = f32(); const M = [[], [], [], []]; for (let c = 0; c < 4; c++) for (let r = 0; r < 4; r++) M[r][c] = a[p / 4 + 4 * c + r]; return M; };
const qp = malloc(16), vecA = malloc(12), vecB = malloc(12), vout = malloc(12), qAp = malloc(16), qBp = malloc(16);
const Hp = malloc(0x84), Cp = malloc(0x84), maskV = malloc(12);
const classify = (q, rays) => {
  f32().set(q, qp / 4); writeVec(vecA, rays); u32().set([0, 0, 0], vout / 4);
  X.d1Horizon1267(vout, qp, vecA);
  const b = u32()[vout / 4], e = u32()[vout / 4 + 1];
  return Array.from(i32().subarray(b / 4, e / 4));
};
const eq = (a, b) => a.length === b.length && a.every((v, k) => v === b[k]);
const fr = Math.fround;
const results = [];
// One original MapTracker per engine instance; its mt19937 advances only through the 1279 calls made here,
// which equals the engine's generator as long as no call of the instance used depth egomotion
// (depth >= 10 draws pairs from the same generator) and every n >= 15 call was dumped in full.
let mtObj = null, rngValid = true;
let prevCall = 0;
let heightBefore = 1;  // the log prints height AFTER the call's update; 1266 scales with the value before it
for (const c of log) {
  if (c.call <= prevCall) { mtObj = null; rngValid = true; heightBefore = 1; }  // new engine instance (call counter restarted)
  prevCall = c.call;
  const r = { call: c.call, all: c.all, ground: c.ground, ok: c.ok, engineT: c.t };
  if (c.cur_rays) {
    const cc = classify(c.q, c.cur_rays), pc = classify(c.qprev, c.prev_rays);
    r.curClassesEqual = eq(cc, c.cur_classes);
    r.prevClassesEqual = eq(pc, c.prev_classes);
    const ground = c.matches.filter(([w, d]) => pc[d] === 1 && cc[w] === 1);
    r.groundOrig = ground.length;
    r.groundEqual = ground.length === c.ground;
    if (ground.length >= 15 && rngValid) {
      if (mtObj === null) { mtObj = malloc(6296); X.d1Ctor640(mtObj); }
      const mt = mtObj;
      f32().set(c.qprev, qAp / 4); f32().set(c.q, qBp / 4);
      writeVec(vecA, ground.map(([, d]) => c.prev_rays[d])); writeVec(vecB, ground.map(([w]) => c.cur_rays[w]));
      u32().fill(0, maskV / 4, maskV / 4 + 3);
      const ret = X.d1Ground1279(qAp, vecA, qBp, vecB, mt + 0x138, mt + 0x1e0, Hp, Cp, maskV);
      const cam = readMat(Cp);
      const t = [0, 1, 2].map((i) => fr(cam[i][3] * fr(heightBefore)));
      r.orig1279 = { ret, t, tNorm: Math.hypot(...t) };
      r.tMaxAbsDiff = ret ? Math.max(...t.map((v, k) => Math.abs(v - c.t[k]))) : null;
    } else if (ground.length >= 15) r.orig1279 = 'skipped: generator state unknown (earlier undumped n>=15 call or depth egomotion)';
  } else if (c.ground >= 15) rngValid = false;
  if (c.depth >= 10) rngValid = false;
  heightBefore = c.height;
  results.push(r);
}
fs.mkdirSync(outDir, { recursive: true });
fs.copyFileSync(new URL(import.meta.url), path.join(outDir, path.basename(new URL(import.meta.url).pathname)));
fs.writeFileSync(path.join(outDir, 'results.json'), JSON.stringify({ log: path.resolve(args.log), logSha256: sha(fs.readFileSync(args.log)),
  codeSectionSha256: codeBefore, results }, null, 1));
for (const r of results.filter((x) => x.curClassesEqual !== undefined)) console.log(JSON.stringify(r));

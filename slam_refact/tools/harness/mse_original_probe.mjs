#!/usr/bin/env node
// Differential MetricScaleEstimator check (WP5b, MS-RT root cause): runs the ORIGINAL func 944
// (pushFrame) of the pinned xr-slam.wasm on the engine's own 944 inputs recorded by
// slam_refact_mse_dump, and compares the first accepted frame and scale with the engine's latch.
// Same isolation method as slam_refact/tools/re/spec_metric_scale/mse-probe.mjs (export section
// augmented, code bytes unchanged, objects built in memory with the func485 field values, imports
// trap except the clock and memcpy/resize helpers). Default MSE settings (method 1).
//
//   node slam_refact/tools/harness/mse_original_probe.mjs --dump <mse-dump.jsonl> --output <new json>
import fs from 'node:fs';
import crypto from 'node:crypto';
import path from 'node:path';

const args = Object.fromEntries(process.argv.slice(2).reduce((acc, a, i, all) =>
  (a.startsWith('--') ? [...acc, [a.slice(2), all[i + 1]]] : acc), []));
const repo = path.resolve(import.meta.dirname, '../../..');
const input = args.input ?? path.join(repo, 'build/slam-refact-20261015/inputs/xr-slam.wasm');
if (!args.dump || !args.output) throw Error('usage: --dump <jsonl> --output <new json> [--input xr-slam.wasm]');
if (fs.existsSync(args.output)) throw Error('refusing to overwrite ' + args.output);
const kPinned = '780731fe294a725acf1b7717ed5ffb07ad9f0929e408a1d74f47450c291ec422';
const hash = (b) => crypto.createHash('sha256').update(b).digest('hex');
const original = fs.readFileSync(input);
if (hash(original) !== kPinned) throw Error('input differs from the pinned module');

const leb = (n) => { const b = []; do { let v = n & 127; n >>>= 7; if (n) v |= 128; b.push(v); } while (n); return Buffer.from(b); };
const readLeb = (b, pos) => { let n = 0, s = 0, v; do { v = b[pos++]; n |= (v & 127) << s; s += 7; } while (v & 128); return [n >>> 0, pos]; };
const exportsToAdd = [['probe_pushFrame', 944]];
const parts = [original.subarray(0, 8)];
let pos = 8, codeBefore, codeAfter;
while (pos < original.length) {
  const start = pos, id = original[pos++];
  const [length, p] = readLeb(original, pos);
  pos = p + length;
  let payload = original.subarray(p, pos);
  if (id === 10) { codeBefore = hash(payload); codeAfter = codeBefore; }
  if (id === 7) {
    const [count, rest] = readLeb(payload, 0);
    const extra = exportsToAdd.map(([name, index]) => Buffer.concat([leb(name.length), Buffer.from(name), Buffer.from([0]), leb(index)]));
    payload = Buffer.concat([leb(count + exportsToAdd.length), payload.subarray(rest), ...extra]);
    parts.push(Buffer.concat([Buffer.from([id]), leb(payload.length), payload]));
  } else {
    parts.push(original.subarray(start, pos));
  }
}
const compiled = new WebAssembly.Module(Buffer.concat(parts));
let clock = 0;
const stubs = {};
let instance;
for (const imp of WebAssembly.Module.imports(compiled)) {
  stubs[imp.module] ??= {};
  stubs[imp.module][imp.name] = (...a) => {
    if (imp.name === 'emscripten_get_now' || imp.name === 'emscripten_date_now') { clock += 0.001; return clock; }
    if (imp.name === 'emscripten_memcpy_big') { new Uint8Array(instance.exports.memory.buffer).copyWithin(a[0], a[1], a[1] + a[2]); return a[0]; }
    if (imp.name === 'emscripten_resize_heap') {
      const need = a[0] >>> 0, buf = instance.exports.memory.buffer;
      if (need <= buf.byteLength) return 1;
      try { instance.exports.memory.grow(Math.ceil((need - buf.byteLength) / 65536)); return 1; } catch (e) { return 0; }
    }
    throw Error('unexpected import ' + imp.name);
  };
}
instance = new WebAssembly.Instance(compiled, stubs);
const ex = instance.exports;
if (ex.emscripten_stack_init) ex.emscripten_stack_init();
const mem = () => new DataView(ex.memory.buffer);
const malloc = (n) => { const p = ex.malloc(Math.max(16, n)); if (!p) throw Error('malloc'); new Uint8Array(ex.memory.buffer, p, Math.max(16, n)).fill(0); return p; };
const putF32 = (p, arr) => arr.forEach((v, i) => mem().setFloat32(p + 4 * i, v, true));

// func485 field values (mse-probe.mjs newMse).
const kInt64Max = 0x7fffffffffffffffn;
const newMse = () => {
  const a = malloc(2840), dv = mem();
  dv.setBigInt64(a + 0, kInt64Max, true); dv.setBigInt64(a + 8, kInt64Max, true); dv.setFloat32(a + 16, 1.0, true);
  dv.setBigInt64(a + 1656, kInt64Max, true); dv.setFloat32(a + 1664, -1.0, true); dv.setFloat32(a + 1864, 1.0, true);
  dv.setFloat32(a + 2708, -1.0, true); dv.setBigInt64(a + 2720, kInt64Max, true); dv.setFloat32(a + 2752, 1.0, true);
  dv.setFloat32(a + 2792, 1.0, true); dv.setUint32(a + 2808, 0x7f7fffff, true); dv.setFloat32(a + 2824, 1.0, true);
  return a;
};
// HMatrix: forward @0, inverse @64 (rigid inverse R^T, -R^T t), flags @128/@129.
const putPose = (p, m) => {
  const inv = new Array(16).fill(0);
  for (let r = 0; r < 3; ++r) for (let c = 0; c < 3; ++c) inv[4 * c + r] = m[4 * r + c];
  for (let r = 0; r < 3; ++r) inv[12 + r] = Math.fround(-(m[4 * r] * m[12] + m[4 * r + 1] * m[13] + m[4 * r + 2] * m[14]));
  inv[15] = 1;
  putF32(p, m); putF32(p + 64, inv);
  mem().setUint8(p + 128, 0); mem().setUint8(p + 129, 0);
};

const rows = fs.readFileSync(args.dump, 'utf8').split('\n').filter((l) => l).map((l) => JSON.parse(l));
const frame = malloc(256), pose = malloc(136);
let evCap = 0, events = 0;
const perCore = {};
let mse = null, core = null;
for (const r of rows) {
  if (r.core !== core) { core = r.core; mse = newMse(); perCore[core] = { calls: 0, originalFirstAccept: null, engineFirstLatch: null }; }
  const st = perCore[core];
  if (r.events.length > evCap) { evCap = Math.max(64, 2 * r.events.length); events = malloc(40 * evCap); }
  r.events.forEach((e, k) => {
    const q = events + 40 * k;
    mem().setInt32(q, e[0], true); mem().setBigInt64(q + 8, BigInt(e[1]), true); mem().setBigInt64(q + 16, BigInt(e[2]), true);
    putF32(q + 24, [e[3], e[4], e[5]]);
  });
  mem().setInt32(frame, 0, true);   // deviceModel (not in the table; method 1 ignores it)
  mem().setUint8(frame + 4, 0);     // isIos
  mem().setUint32(frame + 144, events, true);
  mem().setUint32(frame + 148, events + 40 * r.events.length, true);
  putPose(pose, r.tam);
  const ret = ex.probe_pushFrame(mse, frame, pose, BigInt(r.timeNs), r.state, r.reset);
  const scale = mem().getFloat32(mse + 2824, true);
  if (ret && st.originalFirstAccept === null) st.originalFirstAccept = { call: r.call, timeNs: r.timeNs, scale, residual: mem().getFloat32(mse + 2828, true) };
  if (r.latched && st.engineFirstLatch === null) st.engineFirstLatch = { call: r.call, timeNs: r.timeNs, scale: r.latchedScale };
  st.calls++;
}
const result = { schema: 'slam-refact-mse-original-probe-v1', dump: args.dump, original_sha256: kPinned,
  code_bytes_unchanged: codeBefore === codeAfter, perCore,
  limits: ['original 944 isolated: default MSE settings, objects built from func485 values, deterministic clock',
    'inputs are the engine trajectory (tamExtrinsic) and events, not the original trajectory'] };
fs.writeFileSync(args.output, JSON.stringify(result, null, 1) + '\n', { flag: 'wx' });
console.log(JSON.stringify(perCore));

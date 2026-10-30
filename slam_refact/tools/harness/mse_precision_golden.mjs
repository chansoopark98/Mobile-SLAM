#!/usr/bin/env node
// MSE precision golden (WP5b MS-RT, worker-mse): runs the ORIGINAL func 944 (pushFrame) of the pinned
// xr-slam.wasm on recorded 944 inputs (an slam_refact_mse_dump jsonl, one core) and records, per call,
// the exact bits the C++ port must reproduce: the return value, scale (+2824), residual (+2828) and
// FNV-1a 64 hashes of the position Kalman state (x[9] @kf+4, P[81] @kf+76) and of the angular Kalman
// state (x[6] @imu+124, P[36] @imu+172). Same isolation as mse_original_probe.mjs (export section
// augmented, code bytes unchanged, objects built from the func485 field values, imports trap except
// the clock and memcpy/resize helpers, default MSE settings, deterministic clock).
// The inputs are stored in the golden as float bits so the unit test replays exactly them.
//
//   node slam_refact/tools/harness/mse_precision_golden.mjs --dump <mse-dump.jsonl> --core N
//        [--last-call K] --output <new json>
import fs from 'node:fs';
import crypto from 'node:crypto';
import path from 'node:path';

const args = Object.fromEntries(process.argv.slice(2).reduce((acc, a, i, all) =>
  (a.startsWith('--') ? [...acc, [a.slice(2), all[i + 1]]] : acc), []));
const repo = path.resolve(import.meta.dirname, '../../..');
const input = args.input ?? path.join(repo, 'build/slam-refact-20261015/inputs/xr-slam.wasm');
if (!args.dump || !args.output || args.core === undefined) {
  throw Error('usage: --dump <jsonl> --core N [--last-call K] --output <new json> [--input xr-slam.wasm]');
}
if (fs.existsSync(args.output)) throw Error('refusing to overwrite ' + args.output);
const kPinned = '780731fe294a725acf1b7717ed5ffb07ad9f0929e408a1d74f47450c291ec422';
const hash = (b) => crypto.createHash('sha256').update(b).digest('hex');
const original = fs.readFileSync(input);
if (hash(original) !== kPinned) throw Error('input differs from the pinned module');

const leb = (n) => { const b = []; do { let v = n & 127; n >>>= 7; if (n) v |= 128; b.push(v); } while (n); return Buffer.from(b); };
const readLeb = (b, pos) => { let n = 0, s = 0, v; do { v = b[pos++]; n |= (v & 127) << s; s += 7; } while (v & 128); return [n >>> 0, pos]; };
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
    const name = 'probe_pushFrame';
    const extra = Buffer.concat([leb(name.length), Buffer.from(name), Buffer.from([0]), leb(944)]);
    payload = Buffer.concat([leb(count + 1), payload.subarray(rest), extra]);
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
const f32bits = (v) => { const b = new DataView(new ArrayBuffer(4)); b.setFloat32(0, v, true); return b.getUint32(0, true); };
const hex = (u) => u.toString(16).padStart(8, '0');

const kInt64Max = 0x7fffffffffffffffn;
const newMse = () => {  // func485 field values (mse-probe.mjs newMse)
  const a = malloc(2840), dv = mem();
  dv.setBigInt64(a + 0, kInt64Max, true); dv.setBigInt64(a + 8, kInt64Max, true); dv.setFloat32(a + 16, 1.0, true);
  dv.setBigInt64(a + 1656, kInt64Max, true); dv.setFloat32(a + 1664, -1.0, true); dv.setFloat32(a + 1864, 1.0, true);
  dv.setFloat32(a + 2708, -1.0, true); dv.setBigInt64(a + 2720, kInt64Max, true); dv.setFloat32(a + 2752, 1.0, true);
  dv.setFloat32(a + 2792, 1.0, true); dv.setUint32(a + 2808, 0x7f7fffff, true); dv.setFloat32(a + 2824, 1.0, true);
  return a;
};
// HMatrix: forward @0, inverse @64 (rigid inverse R^T, -R^T t computed in double, rounded once), flags.
const inversePose = (m) => {
  const inv = new Array(16).fill(0);
  for (let r = 0; r < 3; ++r) for (let c = 0; c < 3; ++c) inv[4 * c + r] = m[4 * r + c];
  for (let r = 0; r < 3; ++r) inv[12 + r] = Math.fround(-(m[4 * r] * m[12] + m[4 * r + 1] * m[13] + m[4 * r + 2] * m[14]));
  inv[15] = 1;
  return inv;
};
// FNV-1a 64 over the little-endian bytes of n floats at p.
const fnv = (p, n) => {
  let h = 0xcbf29ce484222325n;
  const bytes = new Uint8Array(ex.memory.buffer, p, 4 * n);
  for (const b of bytes) { h ^= BigInt(b); h = (h * 0x100000001b3n) & 0xffffffffffffffffn; }
  return h.toString(16).padStart(16, '0');
};

const core = Number(args.core);
const lastCall = args['last-call'] === undefined ? Infinity : Number(args['last-call']);
const dumpText = fs.readFileSync(args.dump);
const rows = dumpText.toString('utf8').split('\n').filter((l) => l).map((l) => JSON.parse(l))
  .filter((r) => r.core === core && r.call <= lastCall);
if (!rows.length) throw Error('no rows for core ' + core);
const frame = malloc(256), pose = malloc(136);
let evCap = 0, events = 0;
const mse = newMse();
const calls = [];
for (const r of rows) {
  if (r.events.length > evCap) { evCap = Math.max(64, 2 * r.events.length); events = malloc(40 * evCap); }
  r.events.forEach((e, k) => {
    const q = events + 40 * k;
    mem().setInt32(q, e[0], true); mem().setBigInt64(q + 8, BigInt(e[1]), true); mem().setBigInt64(q + 16, BigInt(e[2]), true);
    putF32(q + 24, [e[3], e[4], e[5]]);
  });
  mem().setInt32(frame, 0, true);
  mem().setUint8(frame + 4, 0);
  mem().setUint32(frame + 144, events, true);
  mem().setUint32(frame + 148, events + 40 * r.events.length, true);
  const inv = inversePose(r.tam);
  putF32(pose, r.tam); putF32(pose + 64, inv);
  mem().setUint8(pose + 128, 0); mem().setUint8(pose + 129, 0);
  const ret = ex.probe_pushFrame(mse, frame, pose, BigInt(r.timeNs), r.state, r.reset);
  const dv = mem();
  calls.push({
    in: {
      timeNs: String(r.timeNs), state: r.state, reset: r.reset,
      pose: r.tam.map((v) => hex(f32bits(v))).join(''), inverse: inv.map((v) => hex(f32bits(v))).join(''),
      events: r.events.map((e) => [e[0], String(e[1]), String(e[2]), hex(f32bits(e[3])) + hex(f32bits(e[4])) + hex(f32bits(e[5]))]),
    },
    out: {
      ret, scale: hex(dv.getUint32(mse + 2824, true)), residual: hex(dv.getUint32(mse + 2828, true)),
      kf: fnv(mse + 16 + 4, 9) + fnv(mse + 16 + 76, 81), imu: fnv(mse + 1744 + 124, 6) + fnv(mse + 1744 + 172, 36),
    },
  });
}
const result = {
  schema: 'slam-refact-mse-precision-golden-v1', original_sha256: kPinned, code_bytes_unchanged: codeBefore === codeAfter,
  dump: path.relative(repo, path.resolve(args.dump)), dump_sha256: hash(dumpText), core, last_call: Number.isFinite(lastCall) ? lastCall : null,
  hash: 'FNV-1a 64 over little-endian float bytes; kf = fnv(x[9]) + fnv(P[81]), imu = fnv(x[6]) + fnv(P[36])',
  limits: ['original 944 isolated: default MSE settings, objects built from func485 values, deterministic clock',
    'pose inverse built by this tool (rigid inverse, translation rounded once from double)'],
  calls,
};
fs.writeFileSync(args.output, JSON.stringify(result) + '\n', { flag: 'wx' });
const firstAccept = calls.findIndex((c) => c.out.ret);
console.log(JSON.stringify({ calls: calls.length, firstAccept, scale: firstAccept >= 0 ? calls[firstAccept].out.scale : null }));

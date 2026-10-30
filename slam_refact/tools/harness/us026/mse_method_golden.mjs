#!/usr/bin/env node
// US-026 S10 G-MSE-U golden (design us026-design.md §4.4, §4.6 NR-MSE-3): runs the ORIGINAL func 944
// (pushFrame) of the pinned xr-slam.wasm on recorded 944 inputs (an slam_refact_mse_dump jsonl, one
// core) once per scaleEstimationMethod and records, per call, everything the C++ port must
// reproduce: the return value, scale (+2824), residual (+2828), acc_sum (+2816), have_estimate
// (+2821) and EVERY element of the position Kalman state (x[9] @kf+4, P[81] @kf+76) and of the
// angular Kalman state (x[6] @imu+124, P[36] @imu+172) as float bits (not FNV), so the unit test can
// measure ulp distances. Snapshots are deduplicated (states[], referenced by index).
//
// Isolation is the one of mse_precision_golden.mjs (export section augmented, code bytes unchanged,
// objects built from the func485 field values, imports trap except the clock and memcpy/resize
// helpers, deterministic clock). The method is set like mse-probe.mjs setCfg: the getters 637/1184
// run first (cache the defaults), then the global MetricScaleEstimator.scaleEstimationMethod
// (i32 @606476) is overwritten; it is read back before and after the run (the getter re-reads only
// when the Params version changes, which nothing does here). Each method runs in a fresh instance.
// frame+0 (deviceModel) is 0 unless a run is written M@D (0 is outside the 1196 lever-arm table, so
// method 5 there is metamorphically method 1; 5@52 selects the first table entry).
//
//   node slam_refact/tools/harness/us026/mse_method_golden.mjs --dump <mse-dump.jsonl> --core N
//        --methods 1,2,3,4,5[,5@52] [--last-call K] [--estimate-after-have 1] --output <new json>
//        [--input wasm] [--quiet 1]
//
// --estimate-after-have 1 (dev REPORT variant, not the default configuration): also pokes
// MetricScaleEstimator.estimateScaleAfterHaveEstimate (u8 @606452, getter 637 DCMP:49820/50050, read
// by 944 DCMP:110575) to 1, so every call after the first accept estimates again; the run records
// estimate_after_have: 1 and the unit test sets the same key on the C++ side.
//
// The output has no time stamps: running it twice on the same input must give identical bytes.
import fs from 'node:fs';
import crypto from 'node:crypto';
import path from 'node:path';

const args = Object.fromEntries(process.argv.slice(2).reduce((acc, a, i, all) =>
  (a.startsWith('--') ? [...acc, [a.slice(2), all[i + 1]]] : acc), []));
const repo = path.resolve(import.meta.dirname, '../../../..');
const input = args.input ?? path.join(repo, 'build/slam-refact-20261015/inputs/xr-slam.wasm');
if (!args.dump || !args.output || args.core === undefined || !args.methods) {
  throw Error('usage: --dump <jsonl> --core N --methods 1,2,3,4,5[,5@52] [--last-call K] ' +
    '--output <new json> [--input xr-slam.wasm]');
}
if (fs.existsSync(args.output)) throw Error('refusing to overwrite ' + args.output);
const kPinned = '780731fe294a725acf1b7717ed5ffb07ad9f0929e408a1d74f47450c291ec422';
const hash = (b) => crypto.createHash('sha256').update(b).digest('hex');
const original = fs.readFileSync(input);
if (hash(original) !== kPinned) throw Error('input differs from the pinned module');
const runs = args.methods.split(',').map((spec) => {
  const [m, d] = spec.split('@');
  const run = {key: spec, method: Number(m), deviceModel: d === undefined ? 0 : Number(d)};
  if (![1, 2, 3, 4, 5].includes(run.method) || !Number.isInteger(run.deviceModel)) throw Error('bad run ' + spec);
  return run;
});

// ---------------------------------------------------------------- export patch (code bytes unchanged)
const kTargets = {pushFrame: 944, mseParams: 637, alignmentParams: 1184};
const leb = (n) => { const b = []; do { let v = n & 127; n >>>= 7; if (n) v |= 128; b.push(v); } while (n); return Buffer.from(b); };
const readLeb = (b, pos) => { let n = 0, s = 0, v; do { v = b[pos++]; n |= (v & 127) << s; s += 7; } while (v & 128); return [n >>> 0, pos]; };
const parts = [original.subarray(0, 8)];
let pos = 8, codeBefore, codeAfter;
while (pos < original.length) {
  const start = pos, id = original[pos++];
  const [length, p] = readLeb(original, pos);
  pos = p + length;
  let payload = original.subarray(p, pos);
  if (id === 10) codeBefore = hash(payload);
  if (id === 7) {
    const [count, rest] = readLeb(payload, 0);
    const extra = Object.entries(kTargets).map(([n, i]) =>
      Buffer.concat([leb(('probe_' + n).length), Buffer.from('probe_' + n), Buffer.from([0]), leb(i)]));
    payload = Buffer.concat([leb(count + extra.length), payload.subarray(rest), ...extra]);
    parts.push(Buffer.concat([Buffer.from([id]), leb(payload.length), payload]));
  } else {
    parts.push(original.subarray(start, pos));
  }
}
const patched = Buffer.concat(parts);
const sectionHash = (buf, want) => {
  let q = 8;
  while (q < buf.length) {
    const id = buf[q++];
    const [length, p] = readLeb(buf, q);
    q = p + length;
    if (id === want) return hash(buf.subarray(p, q));
  }
  return null;
};
codeAfter = sectionHash(patched, 10);
const compiled = new WebAssembly.Module(patched);

const f32bits = (v) => { const b = new DataView(new ArrayBuffer(4)); b.setFloat32(0, v, true); return b.getUint32(0, true); };
const hex = (u) => (u >>> 0).toString(16).padStart(8, '0');
// HMatrix inverse exactly as mse_precision_golden.mjs (rigid inverse, translation rounded once).
const inversePose = (m) => {
  const inv = new Array(16).fill(0);
  for (let r = 0; r < 3; ++r) for (let c = 0; c < 3; ++c) inv[4 * c + r] = m[4 * r + c];
  for (let r = 0; r < 3; ++r) inv[12 + r] = Math.fround(-(m[4 * r] * m[12] + m[4 * r + 1] * m[13] + m[4 * r + 2] * m[14]));
  inv[15] = 1;
  return inv;
};

// ---------------------------------------------------------------- inputs
const core = Number(args.core);
const lastCall = args['last-call'] === undefined ? Infinity : Number(args['last-call']);
const dumpText = fs.readFileSync(args.dump);
const rows = dumpText.toString('utf8').split('\n').filter((l) => l).map((l) => JSON.parse(l))
  .filter((r) => r.core === core && r.call <= lastCall);
if (!rows.length) throw Error('no rows for core ' + core);
const inputs = rows.map((r) => ({
  timeNs: String(r.timeNs), state: r.state, reset: r.reset,
  pose: r.tam.map((v) => hex(f32bits(v))).join(''), inverse: inversePose(r.tam).map((v) => hex(f32bits(v))).join(''),
  events: r.events.map((e) => [e[0], String(e[1]), String(e[2]), hex(f32bits(e[3])) + hex(f32bits(e[4])) + hex(f32bits(e[5]))]),
}));

// ---------------------------------------------------------------- one isolated run per method
// Kalman state snapshots are stored once (deduplicated across calls and methods); a call refers to
// its snapshot by index s.
const states = [];
const stateIndex = new Map();
const internState = (st) => {
  const key = st.kfX + st.kfP + st.imuX + st.imuP;
  if (!stateIndex.has(key)) { stateIndex.set(key, states.length); states.push(st); }
  return stateIndex.get(key);
};
const kMethodAddr = 606476;
const kEstimateAfterHaveAddr = 606452;  // MetricScaleEstimator.estimateScaleAfterHaveEstimate (u8)
const estimateAfterHave = args['estimate-after-have'] === '1';  // MetricScaleEstimator.scaleEstimationMethod (us026-design.md §2.1)
const kInt64Max = 0x7fffffffffffffffn;
const runMethod = ({method, deviceModel}) => {
  let clock = 0;
  const stubs = {};
  const invokedImports = {};
  let instance;
  for (const imp of WebAssembly.Module.imports(compiled)) {
    stubs[imp.module] ??= {};
    stubs[imp.module][imp.name] = (...a) => {
      invokedImports[imp.name] = (invokedImports[imp.name] || 0) + 1;
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
  const hexAt = (p, n) => Array.from({length: n}, (_, i) => hex(mem().getUint32(p + 4 * i, true))).join('');
  // setCfg (mse-probe.mjs): cache the getters, then overwrite the method global.
  ex.probe_mseParams();
  ex.probe_alignmentParams();
  const methodBefore = mem().getInt32(kMethodAddr, true);
  mem().setInt32(kMethodAddr, method, true);
  if (estimateAfterHave) mem().setUint8(kEstimateAfterHaveAddr, 1);
  const a = malloc(2840), dv0 = mem();  // func485 field values (mse-probe.mjs newMse)
  dv0.setBigInt64(a + 0, kInt64Max, true); dv0.setBigInt64(a + 8, kInt64Max, true); dv0.setFloat32(a + 16, 1.0, true);
  dv0.setBigInt64(a + 1656, kInt64Max, true); dv0.setFloat32(a + 1664, -1.0, true); dv0.setFloat32(a + 1864, 1.0, true);
  dv0.setFloat32(a + 2708, -1.0, true); dv0.setBigInt64(a + 2720, kInt64Max, true); dv0.setFloat32(a + 2752, 1.0, true);
  dv0.setFloat32(a + 2792, 1.0, true); dv0.setUint32(a + 2808, 0x7f7fffff, true); dv0.setFloat32(a + 2824, 1.0, true);
  const frame = malloc(256), pose = malloc(136);
  let evCap = 0, events = 0;
  const calls = [];
  for (const r of rows) {
    if (r.events.length > evCap) { evCap = Math.max(64, 2 * r.events.length); events = malloc(40 * evCap); }
    r.events.forEach((e, k) => {
      const q = events + 40 * k;
      mem().setInt32(q, e[0], true); mem().setBigInt64(q + 8, BigInt(e[1]), true); mem().setBigInt64(q + 16, BigInt(e[2]), true);
      putF32(q + 24, [e[3], e[4], e[5]]);
    });
    mem().setInt32(frame, deviceModel, true);
    mem().setUint8(frame + 4, 0);
    mem().setUint32(frame + 144, events, true);
    mem().setUint32(frame + 148, events + 40 * r.events.length, true);
    putF32(pose, r.tam); putF32(pose + 64, inversePose(r.tam));
    mem().setUint8(pose + 128, 0); mem().setUint8(pose + 129, 0);
    const ret = ex.probe_pushFrame(a, frame, pose, BigInt(r.timeNs), r.state, r.reset);
    const dv = mem();
    calls.push({
      ret, scale: hex(dv.getUint32(a + 2824, true)), residual: hex(dv.getUint32(a + 2828, true)),
      accSum: hex(dv.getUint32(a + 2816, true)), haveEstimate: dv.getUint8(a + 2821),
      s: internState({kfX: hexAt(a + 16 + 4, 9), kfP: hexAt(a + 16 + 76, 81),
        imuX: hexAt(a + 1744 + 124, 6), imuP: hexAt(a + 1744 + 172, 36)}),
    });
  }
  const methodAfter = mem().getInt32(kMethodAddr, true);
  const reestimateAfter = mem().getUint8(kEstimateAfterHaveAddr);
  const firstAccept = calls.findIndex((c) => c.ret);
  return {
    method, device_model: deviceModel, poke: {addr: kMethodAddr, type: 'i32', default_read_back: methodBefore, written: method,
      read_back_after_run: methodAfter, held: methodAfter === method &&
        (!estimateAfterHave || reestimateAfter === 1)},
    ...(estimateAfterHave ? {estimate_after_have: 1, estimate_after_have_read_back_after_run: reestimateAfter} : {}),
    invokedImports, firstAccept, acceptCount: calls.filter((c) => c.ret).length, calls,
  };
};

const results = {};
for (const run of runs) results[run.key] = runMethod(run);
const result = {
  schema: 'slam-refact-us026-mse-method-golden-v1', original_sha256: kPinned,
  analytical_copy_sha256: hash(patched), code_bytes_unchanged: codeBefore === codeAfter,
  script_sha256: hash(fs.readFileSync(new URL(import.meta.url))), exports_added: kTargets,
  dump: path.relative(repo, path.resolve(args.dump)), dump_sha256: hash(dumpText), core,
  last_call: Number.isFinite(lastCall) ? lastCall : null,
  layout: 'state float bits as 8-hex little-endian u32 per element: kfX = x[9] @kf+4, kfP = P[81] col-major @kf+76, ' +
    'imuX = x[6] @imu+124, imuP = P[36] col-major @imu+172 (kf = MSE+16, imu = MSE+1744); call.s indexes states[] (deduplicated snapshots)',
  limits: ['original 944 isolated: default MSE settings except scaleEstimationMethod (poked after the getter cached it), ' +
    'objects built from func485 values, deterministic clock, frame+0 deviceModel as given',
  'pose inverse built by this tool (rigid inverse, translation rounded once from double)',
  'recorded inputs are the engine replay inputs of the dump (not the original SDK run inputs)'],
  inputs, states, methods: results,
};
fs.mkdirSync(path.dirname(path.resolve(args.output)), {recursive: true});
fs.writeFileSync(args.output, JSON.stringify(result) + '\n', {flag: 'wx'});
// --quiet 1 (registration runs): print only the output sha256, no result values.
if (args.quiet === '1') console.log(JSON.stringify({output: args.output, sha256: hash(fs.readFileSync(args.output))}));
else console.log(JSON.stringify(Object.fromEntries(Object.entries(results).map(([m, r]) =>
  [m, {firstAccept: r.firstAccept, acceptCount: r.acceptCount, held: r.poke.held,
    scale: r.firstAccept >= 0 ? r.calls[r.firstAccept].scale : null,
    residual: r.firstAccept >= 0 ? r.calls[r.firstAccept].residual : null}]))));

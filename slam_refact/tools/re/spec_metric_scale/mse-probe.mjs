// S5 (MetricScaleEstimator) isolated numeric probe against the original xr-slam.wasm.
//
// Same approach as scripts/dev/references/xr_reference/binary-residual-probe.mjs: an analytical
// copy of the pinned module gets extra exports appended to its export section (code section
// bytes unchanged), and closed functions are called directly on constructed memory.
// Imports trap, except the two clock imports used by profiling scope timers, which return a
// deterministic counter (recorded).
//
// Usage: node mse-probe.mjs --input build/slam-refact-20261015/inputs/xr-slam.wasm --output NEW_DIR
// Output: NEW_DIR/result.json (golden vectors: inputs + original outputs + fixed tolerances).
import fs from 'node:fs';
import crypto from 'node:crypto';
import path from 'node:path';

const argv = process.argv.slice(2);
if (argv.length !== 4 || argv[0] !== '--input' || argv[2] !== '--output') {
  throw Error('Usage: --input xr-slam.wasm --output NEW_DIRECTORY');
}
const input = path.resolve(argv[1]);
const outputPath = path.resolve(argv[3]);
if (fs.existsSync(outputPath)) throw Error('Output already exists; preserving evidence');
const kPinned = '780731fe294a725acf1b7717ed5ffb07ad9f0929e408a1d74f47450c291ec422';
const hash = (b) => crypto.createHash('sha256').update(b).digest('hex');
const original = fs.readFileSync(input);
if (hash(original) !== kPinned) throw Error('Input differs from pinned module');

// ---------------------------------------------------------------- export patch
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
const kTargets = {
  pushFrame: 944, reset: 1163, kalmanPush: 1169, imuPush: 1170, meanDt: 1179,
  quantizeTimes: 1181, xcorr: 1183, crop: 1186, mergeTimes: 1187, dedupe: 1188,
  scaleCameraSpace: 1191, scaleL2: 1192, interp: 1198, gaussKernel: 1199, gaussSmooth: 1200,
  fftCtor: 1240, fftCompute: 1241, mseParams: 637, alignmentParams: 1184,
};
const exportsToAdd = Object.entries(kTargets).map(([n, i]) => ['probe_' + n, i]);
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
    const extra = exportsToAdd.map(([name, index]) =>
      Buffer.concat([leb(name.length), Buffer.from(name), Buffer.from([0]), leb(index)]));
    payload = Buffer.concat([leb(count + exportsToAdd.length), payload.subarray(rest), ...extra]);
    parts.push(Buffer.concat([Buffer.from([id]), leb(payload.length), payload]));
  } else {
    parts.push(original.subarray(start, pos));
  }
  if (id === 10) codeAfter = hash(payload);
}
const patched = Buffer.concat(parts);
const compiled = new WebAssembly.Module(patched);

// ---------------------------------------------------------------- instance
const invokedImports = {};
let clock = 0;
const stubs = {};
for (const imp of WebAssembly.Module.imports(compiled)) {
  if (imp.kind !== 'function') throw Error('Unexpected nonfunction import ' + imp.name);
  stubs[imp.module] ??= {};
  stubs[imp.module][imp.name] = (...args) => {
    invokedImports[imp.name] = (invokedImports[imp.name] || 0) + 1;
    if (imp.name === 'emscripten_get_now') { clock += 0.001; return clock; }
    if (imp.name === 'emscripten_date_now') { clock += 0.001; return clock; }
    // Pure libc helpers (same semantics as the Emscripten JS glue).
    if (imp.name === 'emscripten_memcpy_big') {
      const [dest, src, num] = args;
      new Uint8Array(instance.exports.memory.buffer).copyWithin(dest, src, src + num);
      return dest;
    }
    if (imp.name === 'emscripten_resize_heap') {
      const need = args[0] >>> 0, buf = instance.exports.memory.buffer;
      if (need <= buf.byteLength) return 1;
      try { instance.exports.memory.grow(Math.ceil((need - buf.byteLength) / 65536)); return 1; } catch (e) { return 0; }
    }
    throw Error('Unexpected imported call ' + imp.name + ' ' + JSON.stringify(args.map(String)));
  };
}
let instance;
instance = new WebAssembly.Instance(compiled, stubs);
const ex = instance.exports;
if (ex.emscripten_stack_init) ex.emscripten_stack_init();
const mem = () => new DataView(ex.memory.buffer);
const f32 = (x) => Math.fround(x);

// ---------------------------------------------------------------- memory helpers
const allocs = [];
const malloc = (n) => {
  const p = ex.malloc(Math.max(16, n));
  if (!p) throw Error('malloc failed');
  new Uint8Array(ex.memory.buffer, p, Math.max(16, n)).fill(0);
  allocs.push(p);
  return p;
};
const putF32 = (p, arr) => arr.forEach((v, i) => mem().setFloat32(p + 4 * i, v, true));
const getF32 = (p, n) => Array.from({length: n}, (_, i) => mem().getFloat32(p + 4 * i, true));
// std::vector<T> (libc++): {begin, end, end_cap}
const vecF = (arr) => {
  const v = malloc(12), d = malloc(4 * arr.length);
  putF32(d, arr);
  mem().setUint32(v, d, true);
  mem().setUint32(v + 4, d + 4 * arr.length, true);
  mem().setUint32(v + 8, d + 4 * arr.length, true);
  return v;
};
const vecV3 = (arr) => {  // element = 4 floats (x,y,z,0)
  const v = malloc(12), d = malloc(16 * arr.length);
  arr.forEach((e, i) => putF32(d + 16 * i, [e[0], e[1], e[2], 0]));
  mem().setUint32(v, d, true);
  mem().setUint32(v + 4, d + 16 * arr.length, true);
  mem().setUint32(v + 8, d + 16 * arr.length, true);
  return v;
};
const emptyVec = () => malloc(12);
const readVecF = (v) => {
  const b = mem().getUint32(v, true), e = mem().getUint32(v + 4, true);
  return getF32(b, (e - b) / 4);
};
const bits = (x) => {
  const dv = new DataView(new ArrayBuffer(4)); dv.setFloat32(0, x, true);
  return dv.getUint32(0, true).toString(16).padStart(8, '0');
};

// deterministic LCG (seeded) for test vectors
const lcg = (seed) => {
  let s = seed >>> 0;
  return () => { s = (Math.imul(s, 1664525) + 1013904223) >>> 0; return s / 4294967296; };
};
const randArr = (rnd, n, lo = -1, hi = 1) => Array.from({length: n}, () => f32(lo + (hi - lo) * rnd()));

const results = {};
const record = (name, inputs, outputs) => {
  results[name] = {inputs, outputs, outputBits: JSON.parse(JSON.stringify(outputs, (k, v) =>
    typeof v === 'number' ? bits(v) : v))};
};

// ---------------------------------------------------------------- P1 xcorr (1183)
{
  const rnd = lcg(1);
  const cases = [
    {x: [1, 2, 3], y: [0, 1, 0.5]},
    {x: [1, 2, 3, 4, 5], y: [2, -1, 0.25]},
    {x: randArr(rnd, 64), y: randArr(rnd, 64)},
  ];
  const out = cases.map(({x, y}) => {
    const o = emptyVec();
    ex.probe_xcorr(o, vecF(x), vecF(y));
    return readVecF(o);
  });
  record('xcorr_1183', cases, out);
}

// ---------------------------------------------------------------- P2 scale fits (1191, 1192)
{
  const rnd = lcg(2);
  const n = 200, sTrue = 0.37;
  const a = Array.from({length: n}, () => randArr(rnd, 3, -2, 2));
  const c = a.map((v) => v.map((x) => f32(sTrue * x + 0.05 * (rnd() - 0.5))));
  const cases = [{a, c}, {a: a.slice(0, 1), c: c.slice(0, 1)}];
  const out = cases.map(({a, c}) => {
    const r = {};
    for (const fn of ['scaleCameraSpace', 'scaleL2']) {
      const va = vecV3(a), vc = vecV3(c), s = malloc(4), e = malloc(4);
      const b = mem().getUint32(va, true), en = mem().getUint32(va + 4, true);
      ex['probe_' + fn](b, en, mem().getUint32(vc, true), s, e);
      r[fn] = {scale: mem().getFloat32(s, true), residual: mem().getFloat32(e, true)};
    }
    return r;
  });
  record('scale_fit_1191_1192', {sTrue, n, seed: 2, noise: 0.05}, out);
}

// ---------------------------------------------------------------- P3 interpolation (1198)
{
  const rnd = lcg(3);
  let t = 0;
  const ts = Array.from({length: 50}, () => (t = f32(t + 0.01 + 0.03 * rnd())));
  const xs = randArr(rnd, 50, -3, 3);
  const td = Array.from({length: 80}, (_, i) => f32(-0.05 + i * (ts[49] + 0.1) / 79));
  const cases = [
    {ts, xs, td},
    {ts: [0.5], xs: [2.0], td: [0, 0.5, 1]},
    {ts: [0, 0.1, 0.1, 0.2], xs: [0, 1, 2, 3], td: [0.05, 0.1, 0.15]},
    {ts: [0, 1], xs: [1], td: [0.5]},
  ];
  const out = cases.map(({ts, xs, td}) => {
    const o = emptyVec();
    ex.probe_interp(o, vecF(ts), vecF(xs), vecF(td));
    return readVecF(o);
  });
  record('interp_1198', cases, out);
}

// ---------------------------------------------------------------- P4 mean dt, crop, dedupe, merge
{
  const rnd = lcg(4);
  let t = 1.0;
  const ts = Array.from({length: 37}, () => (t = f32(t + 0.0166 + 0.002 * (rnd() - 0.5))));
  const vts = vecF(ts);
  const meanDt = ex.probe_meanDt(mem().getUint32(vts, true), mem().getUint32(vts + 4, true));
  const data = ts.map((x, i) => [i, -i, f32(x)]);
  const ot = emptyVec(), od = emptyVec();
  ex.probe_crop(f32(1.1), f32(1.4), vecF(ts), vecV3(data), ot, od);
  const croppedT = readVecF(ot);
  const dd = [0, 0.0005, 0.001, 0.0021, 0.0025, 0.004, 0.004, 0.01];
  const od2 = emptyVec();
  ex.probe_dedupe(od2, vecF(dd));
  const ma = [0, 0.1, 0.2, 0.3], mb = [0.05, 0.1, 0.25, 0.4, 0.5];
  const om = emptyVec();
  ex.probe_mergeTimes(om, vecF(ma), vecF(mb));
  record('misc_1179_1186_1187_1188', {ts, cropLo: 1.1, cropHi: 1.4, dedupeIn: dd, mergeA: ma, mergeB: mb},
    {meanDt, croppedT, dedupe: readVecF(od2), merge: readVecF(om)});
}

// ---------------------------------------------------------------- P5 gaussian (1199, 1200)
{
  const rnd = lcg(5);
  const sig = randArr(rnd, 40, -1, 1);
  const out = [];
  for (const [sigma, size] of [[0.5, 1], [1.0, 3], [2.0, 6], [1.5, 4]]) {
    const k = emptyVec();
    ex.probe_gaussKernel(k, f32(sigma), size);
    const o = emptyVec();
    ex.probe_gaussSmooth(o, vecF(sig), f32(sigma), size);
    out.push({sigma, size, kernel: readVecF(k), smoothed: readVecF(o)});
  }
  record('gauss_1199_1200', {signal: sig}, out);
}

// ---------------------------------------------------------------- P6 FftCompute (1240, 1241)
{
  const rnd = lcg(6);
  const out = [];
  for (const n of [8, 64, 100, 97, 2000]) {
    const data = Array.from({length: n}, (_, i) =>
      f32(Math.sin(2 * Math.PI * 3.0 * i / 60) + 0.5 * Math.cos(2 * Math.PI * 7.5 * i / 60) + 0.1 * (rnd() - 0.5)));
    const obj = malloc(64);
    ex.probe_fftCtor(obj, n);
    const d = malloc(4 * n);
    putF32(d, data);
    ex.probe_fftCompute(obj, d, f32(60));
    const fb = mem().getUint32(obj + 24, true), fe = mem().getUint32(obj + 28, true);
    const ab = mem().getUint32(obj + 36, true), ae = mem().getUint32(obj + 40, true);
    out.push({n, fs: 60, numFftField: mem().getInt32(obj + 4, true),
      frequencies: getF32(fb, (fe - fb) / 4), amplitudes: getF32(ab, (ae - ab) / 4),
      dataSha256: hash(Buffer.from(new Float32Array(data).buffer))});
  }
  record('fft_1240_1241', {signal: 'sin(2pi*3t)+0.5cos(2pi*7.5t)+0.1*U(-.5,.5), seed 6, fs 60'}, out);
}

// ---------------------------------------------------------------- MSE object construction
// Field values from the VioTracker constructor (func485, dcmp 27966-28013; MSE = c+24).
const kMseSize = 2840;
const kInt64Max = 0x7fffffffffffffffn;
const newMse = () => {
  const a = malloc(kMseSize);
  const dv = mem();
  dv.setBigInt64(a + 0, kInt64Max, true);
  dv.setBigInt64(a + 8, kInt64Max, true);
  dv.setFloat32(a + 16, 1.0, true);
  dv.setBigInt64(a + 1656, kInt64Max, true);
  dv.setFloat32(a + 1664, -1.0, true);
  dv.setFloat32(a + 1864, 1.0, true);
  dv.setFloat32(a + 2708, -1.0, true);
  dv.setBigInt64(a + 2720, kInt64Max, true);
  dv.setFloat32(a + 2752, 1.0, true);  // movementCheckPeriodInSec default
  dv.setFloat32(a + 2792, 1.0, true);
  dv.setUint32(a + 2808, 0x7f7fffff, true);  // FLT_MAX
  dv.setFloat32(a + 2824, 1.0, true);
  return a;
};
// HMatrix (c8/hmatrix.h): column-major matrix[16] @0, inverse[16] @64, noInvert @128, invertFailed @129.
const putPose = (p, R, t) => {  // R row-major 3x3, t 3
  const m = new Array(16).fill(0), inv = new Array(16).fill(0);
  for (let r = 0; r < 3; ++r) for (let c = 0; c < 3; ++c) { m[4 * c + r] = R[3 * r + c]; inv[4 * c + r] = R[3 * c + r]; }
  m[12] = t[0]; m[13] = t[1]; m[14] = t[2]; m[15] = 1;
  for (let r = 0; r < 3; ++r) inv[12 + r] = -(R[r] * t[0] + R[3 + r] * t[1] + R[6 + r] * t[2]);
  inv[15] = 1;
  putF32(p, m); putF32(p + 64, inv);
  mem().setUint8(p + 128, 0); mem().setUint8(p + 129, 0);
};
const rotY = (th) => [Math.cos(th), 0, Math.sin(th), 0, 1, 0, -Math.sin(th), 0, Math.cos(th)].map(f32);

// ---------------------------------------------------------------- P7 VIO position Kalman (1169)
{
  const a = newMse();
  ex.probe_reset(a);
  const pose = malloc(136);
  const steps = [];
  const base = 1000000000n;
  for (let k = 0; k < 40; ++k) {
    const tSec = k / 30;
    const tns = base + BigInt(Math.round(tSec * 1e9));
    putPose(pose, rotY(0.2 * tSec), [f32(0.5 * Math.sin(2 * tSec)), f32(0.1 * tSec * tSec), f32(-0.3 * tSec)]);
    const ret = ex.probe_kalmanPush(pose, base, tns, 2000, 25, 50, 0, a + 16);
    const h = a + 16;
    steps.push({k, tns: String(tns), ret, x: getF32(h + 4, 9), xPred: getF32(h + 40, 9),
      Pdiag: Array.from({length: 9}, (_, i) => mem().getFloat32(h + 76 + 4 * (10 * i), true)),
      lastT: mem().getFloat32(h + 1648, true)});
  }
  const h = a + 16;
  record('kalman_position_1169', {frames: 40, fps: 30, base: String(base),
    pose: 'R=rotY(0.2t), t=(0.5 sin 2t, 0.1 t^2, -0.3 t)', maxFrames: 2000, skip: 25, predN: 50, ios: 0},
  {steps: steps.filter((s) => s.k < 4 || s.k % 13 === 0 || s.k === 39),
    final: {x: getF32(h + 4, 9), P: getF32(h + 76, 81), F: getF32(h + 832, 81), Q: getF32(h + 1156, 81),
      R: getF32(h + 1480, 9), H: getF32(h + 724, 27), rho: mem().getFloat32(h, true)}});
}

// ---------------------------------------------------------------- P8/P9 end-to-end pushFrame (944)
// Synthetic scene: VIO positions p(t) (units), camera rotation R(t)=rotY(0.3 sin 0.7t);
// metric accel = sTrue * p''(t) (m/s^2); LINEAR_ACCELERATION events (kind 4, 60 Hz, interval
// 16666667 ns) = R(t)^T * metric accel (camera frame == device frame by construction);
// GYROSCOPE events (kind 3, deg/s) = body rate of R(t). Frames at 30 Hz.
// imuOffsetSec shifts the IMU event timestamps (data still sampled at the true time).
// cfg pokes write MetricScaleEstimator globals after the getter cached them (func637 re-reads
// Params only when the Params version changes; nothing changes it in this probe).
const kCfgAddr = {method: [606476, 'i32'], sigma: [606480, 'f32'], activeCheck: [606484, 'u8'],
  sumCheck: [606500, 'u8'], temporalAlignment: [606468, 'u8']};
const kCfgDefault = {method: 1, sigma: 0, activeCheck: 0, sumCheck: 1, temporalAlignment: 1};
const setCfg = (cfg) => {
  ex.probe_mseParams();
  ex.probe_alignmentParams();
  for (const [k, v] of Object.entries({...kCfgDefault, ...cfg})) {
    const [addr, t] = kCfgAddr[k];
    if (t === 'i32') mem().setInt32(addr, v, true);
    else if (t === 'f32') mem().setFloat32(addr, v, true);
    else mem().setUint8(addr, v);
  }
};
const runScenario = ({sTrue = 0.5, imuOffsetSec = 0, imuStartSec = 0, cfg = {}, nFrames = 900, deviceModel = 0}) => {
  setCfg(cfg);
  const p = (t) => [Math.sin(1.3 * t) * 0.8, 0.3 * Math.sin(2.1 * t + 0.4), Math.cos(0.9 * t) * 0.6];
  const pdd = (t) => [-1.69 * Math.sin(1.3 * t) * 0.8, -4.41 * 0.3 * Math.sin(2.1 * t + 0.4), -0.81 * Math.cos(0.9 * t) * 0.6];
  const th = (t) => 0.3 * Math.sin(0.7 * t);
  const thd = (t) => 0.21 * Math.cos(0.7 * t);
  const a = newMse();
  const frame = malloc(256);
  const pose = malloc(136);
  const evMax = 64;
  const events = malloc(40 * evMax);
  const base = 5000000000n;
  const log = [];
  let firstAccept = null;
  let imuK = Math.round(imuStartSec * 60);
  for (let k = 0; k < nFrames; ++k) {
    const tf = k / 30;
    let ne = 0;
    while (imuK / 60 <= tf + 1e-9 && ne < evMax - 1) {
      const ti = imuK / 60;
      const R = rotY(th(ti));
      const am = pdd(ti).map((x) => sTrue * x);
      const ac = [0, 1, 2].map((c) => R[c] * am[0] + R[3 + c] * am[1] + R[6 + c] * am[2]);
      const tns = base + BigInt(Math.round((ti + imuOffsetSec) * 1e9));
      const e1 = events + 40 * ne++;
      mem().setInt32(e1, 4, true); mem().setBigInt64(e1 + 8, tns, true); mem().setBigInt64(e1 + 16, 16666667n, true);
      putF32(e1 + 24, ac.map(f32));
      const e2 = events + 40 * ne++;
      mem().setInt32(e2, 3, true); mem().setBigInt64(e2 + 8, tns, true); mem().setBigInt64(e2 + 16, 16666667n, true);
      putF32(e2 + 24, [0, f32(thd(ti) * 180 / Math.PI), 0]);
      ++imuK;
    }
    mem().setInt32(frame, deviceModel, true);
    mem().setUint8(frame + 4, 0);
    mem().setUint32(frame + 144, events, true);
    mem().setUint32(frame + 148, events + 40 * ne, true);
    putPose(pose, rotY(th(tf)), p(tf).map(f32));
    const tns = base + BigInt(Math.round(tf * 1e9));
    const ret = ex.probe_pushFrame(a, frame, pose, tns, 1, 0);
    const dv = mem();
    const st = {k, ret, scale: dv.getFloat32(a + 2824, true), residual: dv.getFloat32(a + 2828, true),
      sum: dv.getFloat32(a + 2816, true), highAccel: dv.getUint8(a + 2820), haveEstimate: dv.getUint8(a + 2821),
      active: dv.getUint8(a + 2812), minAccel: dv.getFloat32(a + 2808, true),
      accelWinSum: dv.getFloat64(a + 2760, true), accelWinN: dv.getInt32(a + 2748, true)};
    if (ret && firstAccept === null) firstAccept = st;
    if (k < 3 || k % 50 === 0 || (ret && log.filter((s) => s.ret).length < 3)) log.push(st);
  }
  setCfg({});
  return {inputs: {sTrue, imuOffsetSec, imuStartSec, cfg: {...kCfgDefault, ...cfg}, fps: 30, imuHz: 60, frames: nFrames,
    deviceModel, p: 'p=(0.8 sin1.3t, 0.3 sin(2.1t+0.4), 0.6 cos0.9t)', R: 'rotY(0.3 sin 0.7t)',
    trackerState: 1, resetFlag: 0}, outputs: {firstAccept, log}};
};
{
  const r = runScenario({});
  record('push_frame_944', r.inputs, r.outputs);
  const variants = {
    p9a_offset83ms_s1p7: {sTrue: 1.7, imuOffsetSec: 0.083},
    p9b_offset_minus120ms: {sTrue: 0.5, imuOffsetSec: -0.12},
    p9c_method2_l2: {cfg: {method: 2}},
    p9d_method3_frequency: {cfg: {method: 3}},
    p9e_method4_spatial: {cfg: {method: 4}},
    p9f_method5_fixed_spatial: {cfg: {method: 5}},
    p9g_sigma1: {cfg: {sigma: 1.0}},
    p9h_active_check: {cfg: {activeCheck: 1}},
    p9i_no_temporal_alignment_offset83ms: {imuOffsetSec: 0.083, cfg: {temporalAlignment: 0}},
    p9j_scale_out_of_bounds_s3: {sTrue: 3.0},
    p9k_preroll200ms: {imuStartSec: -0.2},
    p9l_preroll200ms_method3: {imuStartSec: -0.2, cfg: {method: 3}},
    p9m_active_check_s1p7: {sTrue: 1.7, cfg: {activeCheck: 1}},
    p9n_preroll450ms_s1p7: {sTrue: 1.7, imuStartSec: -0.45},
  };
  for (const [name, v] of Object.entries(variants)) {
    const rr = runScenario(v);
    record('push_frame_944_' + name, rr.inputs, rr.outputs);
  }
}

// ---------------------------------------------------------------- summary
const tolerances = {
  // Fixed 2026-10-15 BEFORE any C++ port result exists (spec 05 section 8). Applied to
  // port-vs-golden comparisons; probe self-repeatability must be bitwise.
  xcorr_1183: 'abs <= 4 ulp(max|out|) per element (scalar float, same order -> expected bitwise)',
  scale_fit_1191_1192: 'scale rel <= 1e-6, residual rel <= 1e-5; NaN must match NaN',
  interp_1198: 'abs <= 2 ulp(|out|)+1e-7; empty outputs must match exactly',
  misc_1179_1186_1187_1188: 'meanDt rel <= 1e-6; crop/dedupe/merge exact (bitwise)',
  gauss_1199_1200: 'kernel rel <= 2e-6 (expf may differ 1 ulp); smoothed abs <= 1e-6',
  fft_1240_1241: 'frequencies bitwise; amplitudes abs <= 1e-5*max(amp)+1e-7 (Eigen kissfft twiddles may differ 1 ulp)',
  kalman_position_1169: 'x, xPred abs <= 1e-5*(1+|x|); P diag rel <= 1e-4; ret exact; lastT bitwise',
  push_frame_944: 'first accepted frame index exact; scale rel <= 1e-3; residual rel <= 1e-2; sum rel <= 1e-6',
  push_frame_944_p9: 'same as push_frame_944 for every variant; residual compared only when >= 0 (methods 3/4 leave -1)',
};
const result = {
  node: process.version, original_sha256: hash(original), analytical_copy_sha256: hash(patched),
  code_bytes_unchanged: codeBefore === codeAfter, script_sha256: hash(fs.readFileSync(new URL(import.meta.url))),
  exports_added: Object.fromEntries(exportsToAdd), invokedImports, tolerances, results,
  limits: [
    'Analytical copy: only export section augmented; SDK bytes untouched',
    'Objects are constructed in memory (ctor field values from func485); engine init not run',
    'Clock imports return a deterministic counter; memcpy_big/resize_heap implemented as in Emscripten glue; any other import traps',
    'Synthetic inputs; says nothing about runtime dispatch frequency or device accuracy',
  ],
};
fs.mkdirSync(outputPath, {recursive: true});
fs.copyFileSync(new URL(import.meta.url), path.join(outputPath, 'mse-probe.mjs'));
fs.writeFileSync(path.join(outputPath, 'result.json'), JSON.stringify(result, null, 1) + '\n');
console.log(JSON.stringify({code_bytes_unchanged: result.code_bytes_unchanged, invokedImports,
  firstAccept: results.push_frame_944.outputs.firstAccept,
  xcorr0: results.xcorr_1183.outputs[0], scale: results.scale_fit_1191_1192.outputs[0]}, null, 1));

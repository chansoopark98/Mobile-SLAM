// US-026 S10 MS-P10 / MS-P11 golden (us026-design.md §4.5) from the ORIGINAL xr-slam.wasm, isolated.
//
// Same approach as mse-probe.mjs (this file does not modify it): an analytical copy of the pinned
// module gets extra exports appended to its export section (code section bytes unchanged, checked),
// closed functions are called directly on constructed memory, imports trap except the clock and the
// memcpy/resize helpers.
//
//   MS-P10  func 1170 ImuBuffer::push (6-state angular KF) alone, synthetic event streams: the init
//           gyro event (x = 0, P = I, not a measurement), irregular dt, equal time stamps (dt = 0),
//           ignored kinds 1/2, t_oldest skipping, no LINEAR_ACCELERATION (ret 0), more than
//           maxFramesInQueue = 2000 gyro events (State6 / gyro_times pop). Per call: ret, x[6],
//           P[36], F[36], deque sizes, last_t, interval.
//           1170 has no iOS input (6 parameters, call site DCMP:110599); the iOS fps scaling of the
//           design's list belongs to func 1169 (position KF, 7th argument, DCMP:110590), so the
//           ios off/on pair is recorded on 1169 with the static initializer 1201 (kAccelVar) run.
//   MS-P11  func 1178 ImuBuffer::extract(t0, t1, wantGyro) on buffers filled by 1170: the 6x6 RTS
//           smoother output W, Wdot (method 4/5 path) plus T_i, A_i, T_g. The filtered State6 deque
//           (x, P, F per entry) and Q are read out of the original's memory so the C++ test can run
//           its smoother on exactly the original's filter output (isolated) as well as on its own.
//
// Inputs are generated from a seed. Dev and registration inputs are separated by the seed: dev runs
// pass --class dev --seed N (the value is recorded); registration runs pass --class registration
// --seed-file F (a coordinator file this lane never reads; only its sha256 is recorded). The golden
// stores every generated event as bits, so the unit test needs no generator.
//
// Usage: node mse-probe-kf6.mjs [--input xr-slam.wasm] --class dev --seed N --output NEW.json [--quiet 1]
//        node mse-probe-kf6.mjs [--input xr-slam.wasm] --class registration --seed-file F --output NEW.json --quiet 1
import fs from 'node:fs';
import crypto from 'node:crypto';
import path from 'node:path';

const args = Object.fromEntries(process.argv.slice(2).reduce((acc, a, i, all) =>
  (a.startsWith('--') ? [...acc, [a.slice(2), all[i + 1]]] : acc), []));
const repo = path.resolve(import.meta.dirname, '../../../..');
const input = path.resolve(args.input ?? path.join(repo, 'build/slam-refact-20261015/inputs/xr-slam.wasm'));
const usage = 'usage: [--input wasm] --class dev --seed N --output NEW.json | --class registration --seed-file F --output NEW.json';
if (!args.output || !['dev', 'registration'].includes(args.class)) throw Error(usage);
if (args.class === 'dev' && (args.seed === undefined || args['seed-file'] !== undefined)) throw Error(usage);
if (args.class === 'registration' && (args['seed-file'] === undefined || args.seed !== undefined)) throw Error(usage);
const outputPath = path.resolve(args.output);
if (fs.existsSync(outputPath)) throw Error('refusing to overwrite ' + outputPath);
const kPinned = '780731fe294a725acf1b7717ed5ffb07ad9f0929e408a1d74f47450c291ec422';
const hash = (b) => crypto.createHash('sha256').update(b).digest('hex');
const original = fs.readFileSync(input);
if (hash(original) !== kPinned) throw Error('input differs from the pinned module');
let seed, seedRecord;
if (args.class === 'dev') {
  seed = Number(args.seed) >>> 0;
  seedRecord = {class: 'dev', seed};
} else {
  const text = fs.readFileSync(args['seed-file']);
  const parsed = text.toString('utf8').trim();
  if (!/^[0-9]+$/.test(parsed)) throw Error('seed file must hold a decimal integer');
  seed = Number(parsed) >>> 0;
  seedRecord = {class: 'registration', seed_file_sha256: hash(text)};
}

// ---------------------------------------------------------------- export patch
const kTargets = {imuPush: 1170, extract: 1178, kalmanPush: 1169, staticInit1201: 1201,
  mseParams: 637, alignmentParams: 1184};
const leb = (n) => { const b = []; do { let v = n & 127; n >>>= 7; if (n) v |= 128; b.push(v); } while (n); return Buffer.from(b); };
const readLeb = (b, p) => { let n = 0, s = 0, v; do { v = b[p++]; n |= (v & 127) << s; s += 7; } while (v & 128); return [n >>> 0, p]; };
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
const parts = [original.subarray(0, 8)];
for (let pos = 8; pos < original.length;) {
  const start = pos, id = original[pos++];
  const [length, p] = readLeb(original, pos);
  pos = p + length;
  if (id === 7) {
    const payload = original.subarray(p, pos);
    const [count, rest] = readLeb(payload, 0);
    const extra = Object.entries(kTargets).map(([n, i]) =>
      Buffer.concat([leb(('probe_' + n).length), Buffer.from('probe_' + n), Buffer.from([0]), leb(i)]));
    const np = Buffer.concat([leb(count + extra.length), payload.subarray(rest), ...extra]);
    parts.push(Buffer.concat([Buffer.from([id]), leb(np.length), np]));
  } else {
    parts.push(original.subarray(start, pos));
  }
}
const patched = Buffer.concat(parts);
const codeUnchanged = sectionHash(original, 10) === sectionHash(patched, 10);
const compiled = new WebAssembly.Module(patched);

// ---------------------------------------------------------------- instance
const invokedImports = {};
let clock = 0;
let instance;
const stubs = {};
for (const imp of WebAssembly.Module.imports(compiled)) {
  if (imp.kind !== 'function') throw Error('unexpected nonfunction import ' + imp.name);
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
    throw Error('unexpected imported call ' + imp.name);
  };
}
instance = new WebAssembly.Instance(compiled, stubs);
const ex = instance.exports;
if (ex.emscripten_stack_init) ex.emscripten_stack_init();
const mem = () => new DataView(ex.memory.buffer);
const f32 = Math.fround;
const malloc = (n) => { const p = ex.malloc(Math.max(16, n)); if (!p) throw Error('malloc'); new Uint8Array(ex.memory.buffer, p, Math.max(16, n)).fill(0); return p; };
const putF32 = (p, arr) => arr.forEach((v, i) => mem().setFloat32(p + 4 * i, v, true));
const bitsOf = (v) => { const b = new DataView(new ArrayBuffer(4)); b.setFloat32(0, v, true); return b.getUint32(0, true); };
const hex = (u) => (u >>> 0).toString(16).padStart(8, '0');
const hexAt = (p, n) => Array.from({length: n}, (_, i) => hex(mem().getUint32(p + 4 * i, true))).join('');
const lcg = (s0) => { let s = s0 >>> 0; return () => { s = (Math.imul(s, 1664525) + 1013904223) >>> 0; return s / 4294967296; }; };

// Settings: run the two getters once (they cache the binary defaults); nothing changes Params.
ex.probe_mseParams();
ex.probe_alignmentParams();

// libc++ deque<T> (24 B): map.first @0, map.begin @4, map.end @8, map.cap @12, start @16, size @20.
const dequeSize = (d) => mem().getUint32(d + 20, true);
const dequeElem = (d, j, elemSize, perBlock) => {
  const k = mem().getUint32(d + 16, true) + j;
  return mem().getUint32(mem().getUint32(d + 4, true) + 4 * Math.floor(k / perBlock), true) + (k % perBlock) * elemSize;
};
const readDeque = (d, elemSize, perBlock, words) =>
  Array.from({length: dequeSize(d)}, (_, j) => hexAt(dequeElem(d, j, elemSize, perBlock), words));

// MSE object (func485 values, mse-probe.mjs newMse); the ImuBuffer is MSE+1744 (last_t @+964 = -1).
const kInt64Max = 0x7fffffffffffffffn;
const newMse = () => {
  const a = malloc(2840), dv = mem();
  dv.setBigInt64(a + 0, kInt64Max, true); dv.setBigInt64(a + 8, kInt64Max, true); dv.setFloat32(a + 16, 1.0, true);
  dv.setBigInt64(a + 1656, kInt64Max, true); dv.setFloat32(a + 1664, -1.0, true); dv.setFloat32(a + 1864, 1.0, true);
  dv.setFloat32(a + 2708, -1.0, true); dv.setBigInt64(a + 2720, kInt64Max, true); dv.setFloat32(a + 2752, 1.0, true);
  dv.setFloat32(a + 2792, 1.0, true); dv.setUint32(a + 2808, 0x7f7fffff, true); dv.setFloat32(a + 2824, 1.0, true);
  return a;
};
// ImuBuffer field offsets (spec 05 §2.3).
const kImu = {linTimes: 0, linValues: 24, gyroTimes: 48, gyroValues: 72, allTimes: 96, rho: 120, x: 124,
  P: 172, F: 532, Q: 676, states: 940, lastT: 964, interval: 968};

// One 1170 call: events [[kind, tNs(BigInt), intervalNs(BigInt), x, y, z]].
let evBuf = 0, evCap = 0;
const pushImu = (imu, events, baseNs, tOldest, maxFrames) => {
  if (events.length > evCap) { evCap = Math.max(64, 2 * events.length); evBuf = malloc(40 * evCap); }
  events.forEach((e, k) => {
    const q = evBuf + 40 * k;
    mem().setInt32(q, e[0], true); mem().setBigInt64(q + 8, e[1], true); mem().setBigInt64(q + 16, e[2], true);
    putF32(q + 24, [e[3], e[4], e[5]]);
  });
  return ex.probe_imuPush(evBuf, evBuf + 40 * events.length, baseNs, f32(tOldest), maxFrames, imu);
};
const evRecord = (e) => [e[0], String(e[1]), String(e[2]), hex(bitsOf(e[3])) + hex(bitsOf(e[4])) + hex(bitsOf(e[5]))];
const snapshotImu = (imu, withF) => {
  const dv = mem();
  return {
    x: hexAt(imu + kImu.x, 6), P: hexAt(imu + kImu.P, 36), ...(withF ? {F: hexAt(imu + kImu.F, 36)} : {}),
    nLin: dequeSize(imu + kImu.linTimes), nGyroT: dequeSize(imu + kImu.gyroTimes),
    nGyroV: dequeSize(imu + kImu.gyroValues), nAll: dequeSize(imu + kImu.allTimes), nStates: dequeSize(imu + kImu.states),
    lastT: hex(dv.getUint32(imu + kImu.lastT, true)), interval: String(dv.getBigInt64(imu + kImu.interval, true)),
  };
};
// Full buffer contents for MS-P11 and the end of each MS-P10 case.
const dumpImu = (imu) => ({
  linTimes: readDeque(imu + kImu.linTimes, 4, 1024, 1), linValues: readDeque(imu + kImu.linValues, 16, 256, 4),
  gyroTimes: readDeque(imu + kImu.gyroTimes, 4, 1024, 1), allTimesCount: dequeSize(imu + kImu.allTimes),
  states: readDeque(imu + kImu.states, 312, 16, 78),  // x[6], P[36], F[36]
  Q: hexAt(imu + kImu.Q, 36), rho: hex(mem().getUint32(imu + kImu.rho, true)),
});

// ---------------------------------------------------------------- synthetic event streams
// Gyro (deg/s) = smooth body rates + noise; linear acceleration (m/s^2) = smooth + noise.
const kBase = 7000000000n;
const gyroAt = (t, rnd) => [f32(40 * Math.sin(1.1 * t) + 3 * (rnd() - 0.5)), f32(-25 * Math.cos(0.7 * t + 0.3) + 3 * (rnd() - 0.5)),
  f32(15 * Math.sin(2.3 * t + 1.0) + 3 * (rnd() - 0.5))];
const linAt = (t, rnd) => [f32(0.8 * Math.sin(1.7 * t) + 0.1 * (rnd() - 0.5)), f32(0.5 * Math.cos(2.9 * t) + 0.1 * (rnd() - 0.5)),
  f32(-0.6 * Math.sin(0.9 * t + 0.2) + 0.1 * (rnd() - 0.5))];
const ns = (tSec) => kBase + BigInt(Math.round(tSec * 1e9));
// Builds calls: list of event arrays. opts: n gyro events, dt(k) generator, perCall, linEvery, extra.
const makeStream = (rnd, {nGyro, dt, perCall, withLin = true, sameTimeEvery = 0, ignoredKinds = false, linStartGyro = 0}) => {
  const calls = [];
  let t = 0.0, cur = [];
  for (let k = 0; k < nGyro; ++k) {
    t += dt(k);
    const tn = ns(t);
    if (ignoredKinds && k % 7 === 3) cur.push([1, tn, 16666667n, f32(0.1), f32(9.8), f32(0.2)]);
    if (ignoredKinds && k % 11 === 5) cur.push([2, tn, 16666667n, f32(1), f32(2), f32(3)]);
    if (withLin && k >= linStartGyro) cur.push([4, tn, 16666667n, ...linAt(t, rnd)]);
    cur.push([3, tn, 16666667n, ...gyroAt(t, rnd)]);
    if (sameTimeEvery && k % sameTimeEvery === 0) cur.push([3, tn, 16666667n, ...gyroAt(t, rnd)]);  // dt = 0
    if (cur.length >= perCall(k)) { calls.push(cur); cur = []; }
  }
  if (cur.length) calls.push(cur);
  return calls;
};

const results = {};
// ---------------------------------------------------------------- MS-P10 cases (func 1170)
{
  const rnd = lcg(seed ^ 0x10);
  const cases = {
    init_regular_60hz: {stream: {nGyro: 80, dt: () => 1 / 60, perCall: () => 4}, tOldest: () => 0, maxFrames: 2000, fullF: true},
    irregular_dt: {stream: {nGyro: 150, dt: () => 0.002 + 0.043 * rnd(), perCall: () => 1 + Math.floor(4 * rnd())},
      tOldest: () => 0, maxFrames: 2000, fullF: true},
    same_time_and_ignored_kinds: {stream: {nGyro: 90, dt: () => 1 / 60, perCall: () => 5, sameTimeEvery: 3, ignoredKinds: true},
      tOldest: () => 0, maxFrames: 2000, fullF: true},
    gyro_only_then_lin: {stream: {nGyro: 60, dt: () => 1 / 60, perCall: () => 2, linStartGyro: 40},
      tOldest: () => 0, maxFrames: 2000, fullF: true},
    t_oldest_skip: {stream: {nGyro: 120, dt: () => 1 / 60, perCall: () => 6}, tOldest: (c) => (c === 0 ? 0 : 0.05 * c + 0.02),  // 3 gyro per call (0.05 s): skips the first pair
      maxFrames: 2000, fullF: true},
    pop_over_max_frames_2000: {stream: {nGyro: 2150, dt: () => 1 / 60 + 0.002 * (rnd() - 0.5), perCall: () => 6},
      tOldest: () => 0, maxFrames: 2000, fullF: false},
  };
  for (const [name, c] of Object.entries(cases)) {
    const calls = makeStream(rnd, c.stream);
    const a = newMse(), imu = a + 1744;
    const steps = calls.map((events, ci) => {
      const tOldest = f32(c.tOldest(ci));
      const ret = pushImu(imu, events, kBase, tOldest, c.maxFrames);
      return {in: {tOldest: hex(bitsOf(tOldest)), events: events.map(evRecord)}, ret, ...snapshotImu(imu, c.fullF)};
    });
    const end = dumpImu(imu);
    results['kf6_1170_' + name] = {
      base_ns: String(kBase), max_frames: c.maxFrames, steps,
      final: {F: hexAt(imu + kImu.F, 36), Q: end.Q, rho: end.rho, nStates: end.states.length,
        statesFront: end.states[0] ?? null, statesBack: end.states[end.states.length - 1] ?? null,
        gyroTimesFront: end.gyroTimes[0] ?? null, gyroTimesBack: end.gyroTimes[end.gyroTimes.length - 1] ?? null},
    };
  }
}

// ---------------------------------------------------------------- MS-P10 ios pair (func 1169)
{
  const kAccelVarAddr = 151634 * 4, kFpsAlphaAddr = 151647 * 4;  // F1201 store DCMP:156591; F1169 DCMP:148345
  const accelVarBefore = hex(mem().getUint32(kAccelVarAddr, true));
  ex.probe_staticInit1201();
  const accelVarAfter = hex(mem().getUint32(kAccelVarAddr, true));
  const rnd = lcg(seed ^ 0x20);
  const n = 90;  // > 76 frames so ret turns 1
  const times = [];
  let t = 0;
  for (let k = 0; k < n; ++k) { times.push(t); t += 0.022 + 0.03 * rnd(); }
  const posAt = (tt) => [f32(0.4 * Math.sin(1.3 * tt) + 0.01 * (rnd() - 0.5)), f32(0.2 * Math.cos(0.8 * tt)),
    f32(-0.3 * tt + 0.01 * (rnd() - 0.5))];
  const poses = times.map((tt) => { const th = 0.25 * Math.sin(0.6 * tt); return {th, p: posAt(tt)}; });
  const pose = malloc(136);
  for (const ios of [0, 1]) {
    const a = newMse(), kf = a + 16;
    const steps = poses.map((ps, k) => {
      const c = Math.cos(ps.th), s = Math.sin(ps.th);
      const R = [c, 0, s, 0, 1, 0, -s, 0, c].map(f32);  // row-major rotY
      const m = new Array(16).fill(0), inv = new Array(16).fill(0);
      for (let r = 0; r < 3; ++r) for (let cc = 0; cc < 3; ++cc) { m[4 * cc + r] = R[3 * r + cc]; inv[4 * cc + r] = R[3 * cc + r]; }
      m[12] = ps.p[0]; m[13] = ps.p[1]; m[14] = ps.p[2]; m[15] = 1;
      for (let r = 0; r < 3; ++r) inv[12 + r] = f32(-(R[r] * ps.p[0] + R[3 + r] * ps.p[1] + R[6 + r] * ps.p[2]));
      inv[15] = 1;
      putF32(pose, m); putF32(pose + 64, inv);
      mem().setUint8(pose + 128, 0); mem().setUint8(pose + 129, 0);
      const tns = ns(times[k]);
      const ret = ex.probe_kalmanPush(pose, kBase, tns, 2000, 25, 50, ios, kf);
      return {in: {timeNs: String(tns), pose: m.map((v) => hex(bitsOf(v))).join(''), inverse: inv.map((v) => hex(bitsOf(v))).join('')},
        ret, x: hexAt(kf + 4, 9), P: hexAt(kf + 76, 81), Qacc: hexAt(kf + 1156 + 4 * (9 * 6 + 6), 1),
        lastT: hex(mem().getUint32(kf + 1648, true))};
    });
    results['kf9_1169_ios_' + (ios ? 'on' : 'off')] = {base_ns: String(kBase), ios, max_frames: 2000, skip: 25, pred_n: 50,
      kAccelVar_before_1201: accelVarBefore, kAccelVar_after_1201: accelVarAfter,
      fpsAlpha: hex(mem().getUint32(kFpsAlphaAddr, true)), steps};
  }
}

// ---------------------------------------------------------------- MS-P11 cases (func 1178)
{
  const rnd = lcg(seed ^ 0x30);
  const cases = {
    rts_regular_300: {stream: {nGyro: 300, dt: () => 1 / 60 + 0.001 * (rnd() - 0.5), perCall: () => 4}, maxFrames: 2000, window: [0.9, 4.2]},
    rts_irregular_dt0: {stream: {nGyro: 160, dt: () => 0.004 + 0.04 * rnd(), perCall: () => 3, sameTimeEvery: 9}, maxFrames: 2000,
      window: [0.5, 3.0]},
    rts_after_pop_max100: {stream: {nGyro: 170, dt: () => 1 / 60, perCall: () => 4}, maxFrames: 100, window: [1.4, 2.6]},
  };
  const out = malloc(64);
  for (const [name, c] of Object.entries(cases)) {
    const calls = makeStream(rnd, c.stream);
    const a = newMse(), imu = a + 1744;
    for (const events of calls) pushImu(imu, events, kBase, 0, c.maxFrames);
    const buf = dumpImu(imu);
    const t0 = f32(c.window[0]), t1 = f32(c.window[1]);
    const extracted = {};
    for (const want of [1, 0]) {
      new Uint8Array(ex.memory.buffer, out, 64).fill(0);
      ex.probe_extract(out, imu, t0, t1, want);
      const vec = (off, words) => {
        const b = mem().getUint32(out + off, true), e = mem().getUint32(out + off + 4, true);
        const n = (e - b) / (4 * words);
        return Array.from({length: n}, (_, j) => hexAt(b + 4 * words * j, words));
      };
      extracted[want ? 'wantGyro1' : 'wantGyro0'] = {Ti: vec(0, 1), Ai: vec(12, 4), Tg: vec(24, 1), W: vec(36, 4), Wdot: vec(48, 4)};
    }
    results['rts6_1178_' + name] = {
      in: {events: calls.map((ev) => ev.map(evRecord)), base_ns: String(kBase), max_frames: c.maxFrames,
        t0: hex(bitsOf(t0)), t1: hex(bitsOf(t1))},
      buffer: buf, out: extracted,
    };
  }
}

const result = {
  schema: 'slam-refact-us026-mse-kf6-golden-v1', node: process.version, original_sha256: kPinned,
  analytical_copy_sha256: hash(patched), code_bytes_unchanged: codeUnchanged,
  script_sha256: hash(fs.readFileSync(new URL(import.meta.url))), exports_added: kTargets, seed: seedRecord, invokedImports,
  layout: 'floats as 8-hex little-endian u32 per element; matrices column-major; State6 = x[6] P[36] F[36]; ' +
    'Ai/W/Wdot elements are Vec4 (x, y, z, w); events [kind, eventTimeNanos, intervalNanos, xyz bits]',
  tolerances_candidate: 'us026-design.md §4.5 candidates (fixed in the gate file, not here): ret exact, x/P elements <= 3 ulp; ' +
    'MS-P11 tier decided from the dev REPORT',
  limits: ['analytical copy: only the export section augmented; SDK code bytes unchanged',
    'objects constructed in memory (func485 values); getters 637/1184 run once with binary defaults; engine init not run',
    'synthetic inputs; says nothing about runtime dispatch or device data',
    'iOS fps scaling recorded on 1169 (1170 has no iOS input); 1201 run explicitly before the ios pair'],
  results,
};
fs.mkdirSync(path.dirname(outputPath), {recursive: true});
fs.writeFileSync(outputPath, JSON.stringify(result) + '\n', {flag: 'wx'});
// --quiet 1 (registration runs): print only the output sha256, no result values.
if (args.quiet === '1') console.log(JSON.stringify({output: outputPath, sha256: hash(fs.readFileSync(outputPath))}));
else console.log(JSON.stringify({code_bytes_unchanged: codeUnchanged, invokedImports, seed: seedRecord,
  cases: Object.fromEntries(Object.entries(results).map(([k, v]) => [k, v.steps ? {steps: v.steps.length,
    rets: v.steps.reduce((s, x) => s + (x.ret ? 1 : 0), 0), last: v.steps[v.steps.length - 1].nStates ?? v.steps[v.steps.length - 1].lastT}
    : {nStates: v.buffer.states.length, W: v.out.wantGyro1.W.length, Ti: v.out.wantGyro1.Ti.length, Tg0: v.out.wantGyro0.Tg.length}]))}));

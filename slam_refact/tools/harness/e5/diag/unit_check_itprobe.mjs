// Function-level unit check of the instrumented original (instrument_it_wat.py --test-exports build). Node only: stub imports as in
// tools/re/iwt_oracle/run_f994.mjs; no browser, no SDK glue, no server, no network.
//   node unit_check_itprobe.mjs <xr-it-test.wasm>
// Checks: (1) the log helper (header, slot layout, state copy, overflow to the dummy slot, rng pointer),
//         (2) the func 1022 wrapper returns the original's return value and output bytes (camMotion, inliers vector) on synthetic inputs and logs
//             tag 1 {ret, n rays, non-zero inliers} correctly (n = (end-begin)/12), 40 cases incl. failing ones,
//         (3) the exports exist (sr_probe_it is a mutable global).
import fs from 'node:fs';
const module = new WebAssembly.Module(fs.readFileSync(process.argv[2]));
let inst = null;
const mem = () => new DataView(inst.exports.memory.buffer);
const impl = {
  'env.emscripten_get_now': () => performance.now(),
  'env.emscripten_date_now': () => Date.now(),
  'env.abort': () => { throw new Error('wasm abort'); },
  'env.emscripten_memcpy_big': (d, s, n) => { new Uint8Array(inst.exports.memory.buffer).copyWithin(d, s, s + n); },
  'env.emscripten_resize_heap': () => 0,
  'env._emscripten_throw_longjmp': () => { throw new Error('longjmp'); },
  'wasi_snapshot_preview1.environ_sizes_get': (a, b) => { mem().setUint32(a, 0, true); mem().setUint32(b, 0, true); return 0; },
  'wasi_snapshot_preview1.environ_get': () => 0,
  'env._tzset_js': (a, b, c, d) => { for (const p of [a, b]) mem().setInt32(p, 0, true); for (const p of [c, d]) if (p) mem().setUint32(p, 0, true); },
  'env._localtime_js': (t, ptm) => { new Uint8Array(inst.exports.memory.buffer, ptm, 44).fill(0); },
  'wasi_snapshot_preview1.fd_write': (fd, iov, n, pn) => { mem().setUint32(pn, 0, true); return 0; },
};
for (const sig of ['vi', 'ii', 'iiii', 'viiii', 'vii', 'viii']) impl[`env.invoke_${sig}`] = (idx, ...a) => inst.exports.__indirect_function_table.get(idx)(...a);
const imports = {};
for (const d of WebAssembly.Module.imports(module)) {
  imports[d.module] ??= {};
  imports[d.module][d.name] = (...a) => { if (impl[`${d.module}.${d.name}`]) return impl[`${d.module}.${d.name}`](...a); throw new Error('unimplemented import ' + d.module + '.' + d.name); };
}
inst = new WebAssembly.Instance(module, imports);
const X = inst.exports;
X.emscripten_stack_init(); X.__wasm_call_ctors();
let fails = 0;
const check = (name, ok, extra = '') => { if (!ok) fails++; console.log((ok ? 'PASS ' : 'FAIL ') + name + (extra ? ' ' + extra : '')); };
const malloc = n => { const p = X.malloc(Math.max(n, 4)); new Uint8Array(X.memory.buffer, p, Math.max(n, 4)).fill(0); return p; };
const u32 = (p, v) => mem().setUint32(p, v >>> 0, true), ru32 = p => mem().getUint32(p, true), f32 = (p, v) => mem().setFloat32(p, v, true);

check('exports', X.sr_probe_it instanceof WebAssembly.Global && typeof X.sr_w1022 === 'function' && typeof X.sr_orig_1022 === 'function');

// (1) log helper
const rng = malloc(2560);
const pat = new Uint8Array(2500).map((_, k) => (k * 7 + 3) & 255);
new Uint8Array(X.memory.buffer, rng, 2500).set(pat);
for (let k = 0; k < 14; k++) X.sr_log_state(rng, 3);
const buf = X.sr_probe_it.value;
check('buffer allocated', buf > 0);
check('count 14 (12 slots + 2 overflow)', ru32(buf) === 14, `count=${ru32(buf)}`);
check('rng pointer in header', ru32(buf + 4) === rng);
let ok = true;
for (let k = 0; k < 12; k++) { const s = buf + 64 + k * 2560; ok &&= ru32(s) === 3 && Buffer.compare(Buffer.from(X.memory.buffer, s + 32, 2500), Buffer.from(pat)) === 0; }
check('12 slots: tag 3 and byte-exact 2500-byte state copy', ok);
check('overflow writes only the dummy slot at +30784 (inside the 36000-byte buffer), slots 0..11 intact', ru32(buf + 30784) === 3 && ru32(buf + 33344) === 0 && ru32(buf + 64 + 11 * 2560) === 3);
check('log_state(null rng) logs the tag only', (() => { u32(buf, 0); const s = X.sr_log_state(0, 3); return s === buf + 64 && ru32(buf) === 1 && ru32(buf + 4) === rng; })());

// (2) func 1022 wrapper vs original
function lcg(seed) { let s = seed >>> 0; return () => { s = (Math.imul(s, 1664525) + 1013904223) >>> 0; return s / 4294967296; }; }
function makeInputs(seed, n, outlierFrac, shift) {
  const r = lcg(seed);
  const pts = (m) => { const p = malloc(Math.max(m * 12, 12)); return p; };
  const tgt = pts(n), cam = pts(n), w = malloc(n * 4);
  for (let k = 0; k < n; k++) {
    const x = r() - 0.5, y = r() - 0.5, out = r() < outlierFrac;
    f32(tgt + 12 * k, x); f32(tgt + 12 * k + 4, y); f32(tgt + 12 * k + 8, 1);
    const cx = out ? r() - 0.5 : x + shift + (r() - 0.5) * 1e-3, cy = out ? r() - 0.5 : y - shift * 0.5 + (r() - 0.5) * 1e-3;
    f32(cam + 12 * k, cx); f32(cam + 12 * k + 4, cy); f32(cam + 12 * k + 8, 1);
    f32(w + 4 * k, 1);
  }
  const vec = (b, m, es) => { const v = malloc(12); u32(v, b); u32(v + 4, b + m * es); u32(v + 8, b + m * es); return v; };
  const params = malloc(24);
  u32(params, 10); u32(params + 4, 20); f32(params + 8, 500e-6); f32(params + 12, 1e-2); f32(params + 16, 10.0); f32(params + 20, 0);
  const cm = malloc(0x84);
  for (let k = 0; k < 16; k++) f32(cm + 4 * k, k % 5 === 0 ? 1 : 0);
  for (let k = 0; k < 16; k++) f32(cm + 64 + 4 * k, k % 5 === 0 ? 1 : 0);
  const inl = malloc(12), scratch = malloc(2048);
  return {args: [vec(tgt, n, 12), vec(cam, n, 12), vec(w, n, 4), params, cm, inl, scratch], cm, inl, n};
}
const snapshot = (c) => ({cm: Buffer.from(Buffer.from(X.memory.buffer, c.cm, 0x84)), inl: Buffer.from(Buffer.from(X.memory.buffer, ru32(c.inl), ru32(c.inl + 4) - ru32(c.inl)))});
let same = 0, rets = {0: 0, 1: 0}, logOk = 0, total = 0;
for (let t = 0; t < 40; t++) {
  const n = [10, 19, 20, 25, 40, 64][t % 6], of = [0, 0.3, 0.9][t % 3], shift = [0.01, 0.1, 0.0][t % 3];
  const a = makeInputs(1000 + t, n, of, shift), b = makeInputs(1000 + t, n, of, shift);
  const ro = X.sr_orig_1022(...a.args);
  u32(X.sr_probe_it.value, 0);
  const rw = X.sr_w1022(...b.args);
  const sa = snapshot(a), sb = snapshot(b);
  total++;
  if (ro === rw && Buffer.compare(sa.cm, sb.cm) === 0 && Buffer.compare(sa.inl, sb.inl) === 0) same++;
  rets[rw]++;
  const s = X.sr_probe_it.value + 64;
  const nz = [...sb.inl].filter(v => v).length;
  if (ru32(X.sr_probe_it.value) === 1 && ru32(s) === 1 && ru32(s + 4) === rw && ru32(s + 8) === n && ru32(s + 12) === nz) logOk++;
}
check(`1022 wrapper == original (return value, camMotion bytes, inliers bytes) on ${total} cases`, same === total, `${same}/${total}; returns false/true = ${rets[0]}/${rets[1]}`);
check('tag-1 log {ret, n, non-zero inliers} matches', logOk === total, `${logOk}/${total}`);
check('both outcomes exercised', rets[0] > 0 && rets[1] > 0);

// (3) func 670 (glImageTargetWarp) wrapper vs original on synthetic poses: result bytes identical, tag-5 payload = inputs and result
{
  const f32r = (p) => mem().getFloat32(p, true);
  const mk = (seed) => {
    const r = lcg(seed);
    const tk = malloc(24), ck = malloc(24), pose = malloc(0x84), geom = malloc(16), out = malloc(0x84);
    for (const k of [tk, ck]) { u32(k, 480); u32(k + 4, 640); f32(k + 8, 239.5 + r()); f32(k + 12, 319.5 + r()); f32(k + 16, 500 + 10 * r()); f32(k + 20, 500 + 10 * r()); }
    // pose: rotation about z by a small angle + translation, with its exact-ish inverse
    const a = (r() - 0.5) * 0.3, c = Math.cos(a), s = Math.sin(a), t = [(r() - 0.5) * 0.4, (r() - 0.5) * 0.4, -1 - r()];
    const m = [c, s, 0, 0, -s, c, 0, 0, 0, 0, 1, 0, t[0], t[1], t[2], 1];
    const inv = [c, -s, 0, 0, s, c, 0, 0, 0, 0, 1, 0, -(c * t[0] - s * t[1]), -(s * t[0] + c * t[1]), -t[2], 1];
    m.forEach((v, k) => f32(pose + 4 * k, v)); inv.forEach((v, k) => f32(pose + 64 + 4 * k, v));
    mem().setUint8(geom, [0, 1][seed % 2]);
    return {tk, ck, pose, geom, out};
  };
  let same = 0, logok = 0, n = 20;
  for (let t = 0; t < n; t++) {
    const a = mk(500 + t), b = mk(500 + t);
    X.sr_orig_670(a.out, a.tk, a.ck, a.pose, a.geom);
    u32(X.sr_probe_it.value, 0);
    X.sr_w670(b.out, b.tk, b.ck, b.pose, b.geom);
    const bytes = (p, l) => Buffer.from(Buffer.from(X.memory.buffer, p, l));
    if (Buffer.compare(bytes(a.out, 0x84), bytes(b.out, 0x84)) === 0) same++;
    const s = X.sr_probe_it.value + 64;
    if (ru32(X.sr_probe_it.value) === 1 && ru32(s) === 5 && Buffer.compare(bytes(s + 32, 24), bytes(b.tk, 24)) === 0 && Buffer.compare(bytes(s + 56, 24), bytes(b.ck, 24)) === 0 &&
        Buffer.compare(bytes(s + 80, 0x84), bytes(b.pose, 0x84)) === 0 && Buffer.compare(bytes(s + 212, 0x84), bytes(b.out, 0x84)) === 0 && bytes(s + 344, 1)[0] === bytes(b.geom, 1)[0]) logok++;
  }
  check(`670 wrapper == original (result HMatrix bytes) on ${n} cases`, same === n, `${same}/${n}`);
  check('tag-5 payload (targetK, camK, pose, result, geometry) correct', logok === n, `${logok}/${n}`);
}

// (4) func 661 (scaleTranslation) wrapper vs original
{
  const r = lcg(77);
  let same = 0, logok = 0, n = 20;
  for (let t = 0; t < n; t++) {
    const mkcam = () => {
      const a = (r() - 0.5) * 1.2, b = (r() - 0.5) * 0.4, c = Math.cos(a), s = Math.sin(a), cb = Math.cos(b), sb = Math.sin(b);
      const m = [c, s * cb, s * sb, 0, -s, c * cb, c * sb, 0, 0, -sb, cb, 0, (r() - 0.5), (r() - 0.5), -1 - r(), 1];
      const p = malloc(0x84); m.forEach((v, k) => f32(p + 4 * k, v)); m.forEach((v, k) => f32(p + 64 + 4 * k, v * 0.5)); return p;
    };
    const cam1 = mkcam(); const cam2 = malloc(0x84); new Uint8Array(X.memory.buffer, cam2, 0x84).set(new Uint8Array(X.memory.buffer, cam1, 0x84));
    const o1 = malloc(0x84), o2 = malloc(0x84), sc = 0.5 + r() * 3;
    X.sr_orig_661(o1, sc, cam1);
    u32(X.sr_probe_it.value, 0);
    X.sr_w661(o2, sc, cam2);
    const bytes = (p, l) => Buffer.from(Buffer.from(X.memory.buffer, p, l));
    if (Buffer.compare(bytes(o1, 0x84), bytes(o2, 0x84)) === 0) same++;
    const s = X.sr_probe_it.value + 64;
    if (ru32(X.sr_probe_it.value) === 1 && ru32(s) === 7 && mem().getFloat32(s + 32, true) === Math.fround(sc) && Buffer.compare(bytes(s + 36, 0x84), bytes(cam2, 0x84)) === 0 && Buffer.compare(bytes(s + 168, 0x84), bytes(o2, 0x84)) === 0) logok++;
  }
  check(`661 wrapper == original on ${n} cases`, same === n, `${same}/${n}`);
  check('tag-7 payload (scale, cam, result) correct', logok === n, `${logok}/${n}`);
}
console.log(fails ? `FAILED ${fails}` : 'ALL PASS');
process.exit(fails ? 1 : 0);

// US-025 LC-G, G-B: func 1263 (AgateDescriptor extract, spec 12 s2.5) run alone on the ORIGINAL xr-slam.wasm, same approach as
// tools/re/spec_metric_scale/mse-probe.mjs: an analytical copy of the pinned module gets extra exports appended to its export
// section (code section bytes unchanged), imports are stubs that trap except the pure helpers the Emscripten glue provides, and
// the static initializers (`__wasm_call_ctors`, which includes the agate dictionary/PCA setup func 4043) are run once.
//
// Usage: node golden_1263.mjs --input build/slam-refact-20261015/inputs/xr-slam.wasm --output NEW_DIR [--ga-dir <G-A lcgold dir> ...]
//   --ga-dir: a run's lcgold/ directory (events.jsonl + blobs.bin from run_lcgold.mjs); every event's newest ORB store is also run
//             standalone and compared with the in-run descriptor (the agate 32 B the original wrote into WorldMap+0x184 + slot*0x20).
// Output: NEW_DIR/{cases.jsonl, blobs.bin, header.json}. Format: docs/design/us025-lcgold-golden.md section 4.
import fs from 'node:fs';
import crypto from 'node:crypto';
import path from 'node:path';

const argv = process.argv.slice(2);
const opt = (name, multi = false) => {
  const out = [];
  for (let i = 0; i < argv.length; i++) if (argv[i] === name) out.push(argv[++i]);
  return multi ? out : out[0];
};
const input = path.resolve(opt('--input') ?? '');
const outputPath = path.resolve(opt('--output') ?? '');
const gaDirs = opt('--ga-dir', true).map(d => path.resolve(d));
if (!opt('--input') || !opt('--output')) throw Error('Usage: --input xr-slam.wasm --output NEW_DIRECTORY [--ga-dir <lcgold dir> ...]');
if (fs.existsSync(outputPath)) throw Error('Output already exists; preserving evidence');
const kPinned = '780731fe294a725acf1b7717ed5ffb07ad9f0929e408a1d74f47450c291ec422';
const hash = b => crypto.createHash('sha256').update(b).digest('hex');
const original = fs.readFileSync(input);
if (hash(original) !== kPinned) throw Error('Input differs from pinned module');

const leb = n => { const b = []; do { let v = n & 127; n >>>= 7; if (n) v |= 128; b.push(v); } while (n); return Buffer.from(b); };
const readLeb = (b, pos) => { let n = 0, shift = 0, v; do { v = b[pos++]; n |= (v & 127) << shift; shift += 7; } while (v & 128); return [n >>> 0, pos]; };
const kTargets = {agate1263: 1263};
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
    const extra = exportsToAdd.map(([name, index]) => Buffer.concat([leb(name.length), Buffer.from(name), Buffer.from([0]), leb(index)]));
    payload = Buffer.concat([leb(count + exportsToAdd.length), payload.subarray(rest), ...extra]);
    parts.push(Buffer.concat([Buffer.from([id]), leb(payload.length), payload]));
  } else parts.push(original.subarray(start, pos));
  if (id === 10) codeAfter = hash(payload);
}
const patched = Buffer.concat(parts);
const compiled = new WebAssembly.Module(patched);

const invokedImports = {};
let clock = 0, instance;
const stubs = {};
for (const imp of WebAssembly.Module.imports(compiled)) {
  if (imp.kind !== 'function') throw Error('Unexpected nonfunction import ' + imp.name);
  stubs[imp.module] ??= {};
  stubs[imp.module][imp.name] = (...args) => {
    invokedImports[imp.name] = (invokedImports[imp.name] || 0) + 1;
    if (imp.name === 'emscripten_get_now' || imp.name === 'emscripten_date_now') { clock += 0.001; return clock; }
    if (imp.name === 'emscripten_memcpy_big') { const [d, s, n] = args; new Uint8Array(instance.exports.memory.buffer).copyWithin(d, s, s + n); return d; }
    if (imp.name === 'emscripten_resize_heap') {
      const need = args[0] >>> 0, buf = instance.exports.memory.buffer;
      if (need <= buf.byteLength) return 1;
      try { instance.exports.memory.grow(Math.ceil((need - buf.byteLength) / 65536)); return 1; } catch (e) { return 0; }
    }
    if (imp.name === 'environ_sizes_get') { const dv = new DataView(instance.exports.memory.buffer); dv.setUint32(args[0], 0, true); dv.setUint32(args[1], 0, true); return 0; }
    if (imp.name === 'environ_get') return 0;
    throw Error('Unexpected imported call ' + imp.name + ' ' + JSON.stringify(args.map(String)));
  };
}
instance = new WebAssembly.Instance(compiled, stubs);
const ex = instance.exports;
if (ex.emscripten_stack_init) ex.emscripten_stack_init();
ex.__wasm_call_ctors();
const mem = () => new DataView(ex.memory.buffer);
const u8 = () => new Uint8Array(ex.memory.buffer);

const malloc = n => { const p = ex.malloc(Math.max(16, n)); if (!p) throw Error('malloc failed'); u8().fill(0, p, p + Math.max(16, n)); return p; };
// std::vector<32 B> {begin, end, end_cap}; `nullEmpty`: begin = end = 0 (a default-constructed vector) instead of an empty heap block.
const makeVec = (bytes, nullEmpty = false) => {
  const v = malloc(12);
  if (bytes.length === 0 && nullEmpty) return v;
  const d = malloc(Math.max(32, bytes.length));
  u8().set(bytes, d);
  const dv = mem();
  dv.setUint32(v, d, true); dv.setUint32(v + 4, d + bytes.length, true); dv.setUint32(v + 8, d + bytes.length, true);
  return v;
};
const run1263 = (bytes, nullEmpty = false) => {
  const out = malloc(32);
  u8().fill(0xa5, out, out + 32);  // sentinel: an untouched output would be visible
  const vec = makeVec(bytes, nullEmpty);
  ex.probe_agate1263(out, vec);
  return Buffer.from(u8().subarray(out, out + 32));
};

// -------------------------------------------------------------------------------------------- cases
const lcg = seed => { let s = seed >>> 0; return () => { s = (Math.imul(s, 1664525) + 1013904223) >>> 0; return s >>> 24; }; };
const randBytes = (seed, n) => { const r = lcg(seed); return Buffer.from(Array.from({length: n * 32}, () => r())); };
const cases = [];
const add = (name, bytes, note, nullEmpty = false) => cases.push({name, n: bytes.length / 32, bytes: Buffer.from(bytes), note, nullEmpty});
add('empty-heap-vector', Buffer.alloc(0), 'begin == end != 0 (empty vector with storage)');
add('empty-null-vector', Buffer.alloc(0), 'begin == end == 0 (default-constructed vector)', true);
for (const n of [1, 2, 3, 5, 10, 31, 32, 33, 64, 100, 200, 300, 500, 1000]) add(`random-n${n}`, randBytes(1000 + n, n), `LCG seed ${1000 + n}, ${n} random 32 B descriptors`);
add('zeros-n5', Buffer.alloc(5 * 32), 'five all-zero descriptors');
add('ones-n5', Buffer.alloc(5 * 32, 0xff), 'five all-0xff descriptors');
add('dup-n20', Buffer.concat(Array.from({length: 20}, () => randBytes(77, 1))), 'one random descriptor repeated 20 times');
add('single-bit-n8', Buffer.concat(Array.from({length: 8}, (_, i) => { const b = Buffer.alloc(32); b[i * 4] = 1 << i; return b; })), 'eight descriptors with one set bit');

const blobs = [];
let off = 0;
const ref = bytes => { const r = {blob: 'blobs.bin', off, len: bytes.length, sha256: hash(bytes)}; const pad = (8 - (bytes.length & 7)) & 7; blobs.push(bytes, Buffer.alloc(pad)); off += bytes.length + pad; return r; };
const lines = [];
const resultsA = new Map();
for (const c of cases) {
  const o1 = run1263(c.bytes, c.nullEmpty), o2 = run1263(c.bytes, c.nullEmpty);
  resultsA.set(c.name, o1);
  lines.push({kind: 'synthetic', name: c.name, n: c.n, note: c.note, nullEmpty: c.nullEmpty, input: ref(c.bytes), output: ref(o1), outputHex: o1.toString('hex'),
    repeatSame: o1.equals(o2), outputWritten: !o1.every(x => x === 0xa5)});
}
// order independence: run the synthetic cases again in reverse order (state of the module-level scratch vectors must not leak)
const orderIndependent = [...cases].reverse().every(c => run1263(c.bytes, c.nullEmpty).equals(resultsA.get(c.name)));

// real stores from the in-run dumps
const gaSummary = [];
for (const dir of gaDirs) {
  const blobFile = fs.readFileSync(path.join(dir, 'blobs.bin'));
  const rd = r => blobFile.subarray(r.off, r.off + r.len);
  let match = 0, mismatch = 0, total = 0;
  for (const line of fs.readFileSync(path.join(dir, 'events.jsonl'), 'utf8').split('\n').filter(Boolean)) {
    const ev = JSON.parse(line);
    if (!ev.newestOrb || !ev.newestAgate) continue;
    const orb = rd(ev.newestOrb.bytes), want = rd(ev.newestAgate);
    const got = run1263(orb);
    total++;
    const same = got.equals(want);
    same ? match++ : mismatch++;
    lines.push({kind: 'real', source: path.relative(process.cwd(), dir), seq: ev.seq, i: ev.i, newestSlot: ev.newest[0], n: ev.newestOrb.n, input: ref(orb), output: ref(got),
      outputHex: got.toString('hex'), inRunAgateHex: want.toString('hex'), equalsInRun: same});
  }
  gaSummary.push({dir: path.relative(process.cwd(), dir), total, match, mismatch});
}

fs.mkdirSync(outputPath, {recursive: true});
fs.writeFileSync(path.join(outputPath, 'cases.jsonl'), lines.map(l => JSON.stringify(l)).join('\n') + '\n');
fs.writeFileSync(path.join(outputPath, 'blobs.bin'), Buffer.concat(blobs));
fs.writeFileSync(path.join(outputPath, 'header.json'), JSON.stringify({schema: 'slam-refact-us025-golden-1263-v1', wasmSha256: kPinned, patchedWasmSha256: hash(patched),
  codeSectionSha256Before: codeBefore, codeSectionSha256After: codeAfter, probeExport: 'probe_agate1263 = func 1263 (out*, vector<32 B>*)', cases: lines.length, synthetic: cases.length,
  orderIndependent, ga: gaSummary, invokedImports, ctors: '__wasm_call_ctors run once (includes func 4043 agate dictionary/PCA setup)',
  inputsFrom: gaDirs.map(d => path.relative(process.cwd(), d))}, null, 1) + '\n');
console.log(JSON.stringify({out: path.relative(process.cwd(), outputPath), cases: lines.length, orderIndependent, ga: gaSummary, invokedImports}));

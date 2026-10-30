// Unit check of itprobe_orig.js without a browser: a fake WebAssembly.instantiate returns the Node instance of the instrumented test module with a
// fake c8EmAsm_processStagedFrame that exercises the real wasm log helper (sr_log_state) and a fake RNG object. node unit_check_itprobe_js.mjs <xr-it-test.wasm>
import fs from 'node:fs';
const module = new WebAssembly.Module(fs.readFileSync(process.argv[2]));
let inst = null;
const mem = () => new DataView(inst.exports.memory.buffer);
const impl = {
  'env.emscripten_get_now': () => performance.now(), 'env.emscripten_date_now': () => Date.now(), 'env.abort': () => { throw new Error('abort'); },
  'env.emscripten_memcpy_big': (d, s, n) => { new Uint8Array(inst.exports.memory.buffer).copyWithin(d, s, s + n); }, 'env.emscripten_resize_heap': () => 0,
  'wasi_snapshot_preview1.environ_sizes_get': (a, b) => { mem().setUint32(a, 0, true); mem().setUint32(b, 0, true); return 0; }, 'wasi_snapshot_preview1.environ_get': () => 0,
  'env._tzset_js': () => {}, 'env._localtime_js': (t, p) => { new Uint8Array(inst.exports.memory.buffer, p, 44).fill(0); }, 'wasi_snapshot_preview1.fd_write': (a, b, c, d) => { mem().setUint32(d, 0, true); return 0; },
};
for (const sig of ['vi', 'ii', 'iiii', 'viiii', 'vii', 'viii']) impl[`env.invoke_${sig}`] = (idx, ...a) => inst.exports.__indirect_function_table.get(idx)(...a);
const imports = {};
for (const d of WebAssembly.Module.imports(module)) { imports[d.module] ??= {}; imports[d.module][d.name] = (...a) => { if (impl[`${d.module}.${d.name}`]) return impl[`${d.module}.${d.name}`](...a); throw new Error('unimplemented ' + d.name); }; }
inst = new WebAssembly.Instance(module, imports);
const X = inst.exports; X.emscripten_stack_init(); X.__wasm_call_ctors();
const rng = X.malloc(2560);
const set = (k) => { const w = new Uint32Array(X.memory.buffer, rng, 626); for (let i = 0; i < 624; i++) w[i] = (i * 2654435761 + k) >>> 0; w[624] = k % 624; };
let call = 0;
const fake = {memory: X.memory, sr_probe_it: X.sr_probe_it, other: 42, c8EmAsm_processStagedFrame: () => {
  call++;
  if (call !== 2) { set(call * 11); X.sr_log_state(rng, 3); }   // call 2: a frame without func 784 (no slot)
  if (call === 3) { set(500); const s = X.sr_log_state(rng, 2); new DataView(X.memory.buffer).setUint32(s + 4, 1, true); set(501); }
  return 7;
}};
globalThis.window = {};
globalThis.btoa = s => Buffer.from(s, 'latin1').toString('base64');
WebAssembly.instantiate = async () => ({module: {}, instance: {exports: fake}});
WebAssembly.instantiateStreaming = undefined;
await import('./itprobe_orig.js');
window.__itProbeInit();
const r = await WebAssembly.instantiate(null, {});
const ex = r.instance.exports;
const idx = b => Buffer.from(b, 'base64').readUInt32LE(2496);
let fails = 0; const check = (n, ok) => { if (!ok) fails++; console.log((ok ? 'PASS ' : 'FAIL ') + n); };
check('wrapped export passes other exports through', ex.other === 42 && ex.memory === X.memory);
check('return value passed through', ex.c8EmAsm_processStagedFrame() === 7);
ex.c8EmAsm_processStagedFrame(); ex.c8EmAsm_processStagedFrame();
const R = window.__srItProbe.records;
check('3 records, seq 1..3', R.length === 3 && R.map(x => x.seq).join() === '1,2,3');
check('record 1: one tag-3 slot with byte-exact state, post state present', R[0].slots.length === 1 && R[0].slots[0].tag === 3 && Buffer.from(R[0].slots[0].state, 'base64').length === 2500 && R[0].post && R[0].rngPtr === rng);
check('record 2: no slots after the reset (frame without func 784)', R[1].slots.length === 0 && R[1].count === 0);
check('record 3: tag-3 then tag-2 with a=1; tag-2 state is the state at logging time (500), post is the later state (501)', R[2].slots.length === 2 && R[2].slots[1].tag === 2 && R[2].slots[1].a === 1 &&
  idx(R[2].slots[1].state) === 500 % 624 && idx(R[2].post) === 501 % 624);
check('post(1) equals tag-3 state(1) bytes when nothing changed', R[0].post === R[0].slots[0].state);
console.log(fails ? 'FAILED ' + fails : 'ALL PASS'); process.exit(fails ? 1 : 0);

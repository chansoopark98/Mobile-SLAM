// US-025 LC-0: unit test of the loop-close additions to memprobe.js (lc option) and us026/config-poke.js (lc key) on a
// fake linear memory (no browser, no original Wasm). node --test slam_refact/tools/re/runtime/us025/lc_probe.test.mjs
import test from 'node:test';
import assert from 'node:assert/strict';
import fs from 'node:fs';
import vm from 'node:vm';
import path from 'node:path';
import {webcrypto, createHash} from 'node:crypto';
import {fileURLToPath} from 'node:url';

const here = path.dirname(fileURLToPath(import.meta.url));
const rt = path.resolve(here, '..');
const probeSrc = fs.readFileSync(path.join(rt, 'memprobe.js'), 'utf8');
const pokeSrc = fs.readFileSync(path.join(rt, 'us026/config-poke.js'), 'utf8');
const BYTES = new Uint8Array([0, 97, 115, 109, 1, 0, 0, 0]);
const SHA = createHash('sha256').update(BYTES).digest('hex');
const B = 0x20000, MB = 0x30000, MT = 0x31000, WM = 0x32000, KB = 0x34000, DESC = 0x38000;
const j = x => JSON.parse(JSON.stringify(x));  // cross-realm arrays: compare as plain JSON
const DEFAULT_FIELDS = 40;  // memprobe.js FIELDS without paths / lc

function setup({probe = {}, poke = null}) {
  const buffer = new ArrayBuffer(1 << 20);
  const u32 = new Uint32Array(buffer), u8 = new Uint8Array(buffer), f32 = new Float32Array(buffer);
  u32[B >> 2] = 0x250b0; u32[(B + 6596) >> 2] = MB; u32[(B + 6460) >> 2] = MT; u32[(B + 7232) >> 2] = WM;
  u32[(WM + 0xc) >> 2] = KB; u32[(WM + 0x10) >> 2] = KB + 3 * 984;       // 3 keyframe slots
  u32[(WM + 0x184) >> 2] = DESC; u32[(WM + 0x188) >> 2] = DESC + 2 * 32; // 2 descriptors
  for (let j = 0; j < 16; j++) { f32[(B + 6632 >> 2) + j] = j % 5 === 0 ? 1 : 0; f32[(B + 6632 >> 2) + 16 + j] = j % 5 === 0 ? 1 : 0; }
  const calls = [];
  const sandbox = {console, performance, crypto: webcrypto, fetch: async () => { throw new Error('no fetch'); },
    WebAssembly: {Module: class {}, compile: async bytes => ({bytes}),
      instantiate: async () => ({exports: {memory: {buffer}, c8EmAsm_processStagedFrame: (...a) => { calls.push(a); return 7; }}})}};
  sandbox.window = sandbox;
  const ctx = vm.createContext(sandbox);
  vm.runInContext(`${probeSrc}\nwindow.__memProbeInit(${JSON.stringify({targetSha256: SHA, ...probe})});`, ctx);
  if (poke) vm.runInContext(`${pokeSrc}\nwindow.__configPokeInit(${JSON.stringify({targetSha256: SHA, keys: poke})});`, ctx);
  return {ctx, sandbox, u32, u8, f32, buffer, calls};
}
async function callOnce(s) {
  const res = await s.sandbox.WebAssembly.instantiate(BYTES, {});
  return res.instance.exports.c8EmAsm_processStagedFrame();
}

test('default probe: 40 fields, no lc fields, record length 40', async () => {
  const s = setup({});
  await callOnce(s);
  const P = s.sandbox.__memProbe;
  assert.equal(P.error, null);
  assert.equal(P.fields.length, DEFAULT_FIELDS);
  assert.ok(!P.fields.includes('outLcOk'));
  assert.equal(P.records[0].length, DEFAULT_FIELDS);
});

test('paths probe without lc is unchanged (52 fields)', async () => {
  const s = setup({probe: {paths: true}});
  await callOnce(s);
  assert.equal(s.sandbox.__memProbe.fields.length, DEFAULT_FIELDS + 12);
});

test('lc probe: reads out+0x1c, matrix, ring, descriptor count, slots (read only)', async () => {
  const s = setup({probe: {lc: true}});
  s.u8[B + 6628] = 1; s.f32[(B + 6632 >> 2) + 12] = 2.5;
  for (let j = 0; j < 8; j++) s.u32[(MB + 0xa28 >> 2) + j] = 100 + j;
  const before = new Uint8Array(s.buffer.slice(0));
  await callOnce(s);
  const P = s.sandbox.__memProbe, f = P.fields, r = P.records[0];
  assert.equal(P.error, null);
  assert.deepEqual(j(f.slice(DEFAULT_FIELDS)), ['outLcOk', 'outLcMat', 'mbLcRing', 'wmKfDescCount', 'lcKfSlots']);
  assert.equal(r.length, f.length);
  assert.equal(r[f.indexOf('outLcOk')], 1);
  const mat = r[f.indexOf('outLcMat')];
  assert.equal(mat.length, 32);
  assert.equal(mat[12], new Uint32Array(new Float32Array([2.5]).buffer)[0].toString(16).padStart(8, '0'));
  assert.deepEqual(j(r[f.indexOf('mbLcRing')]), [100, 101, 102, 103, 104, 105, 106, 107]);
  assert.equal(r[f.indexOf('wmKfDescCount')], 2);
  assert.equal(r[f.indexOf('lcKfSlots')], 3);
  assert.deepEqual(new Uint8Array(s.buffer), before, 'probe must not write memory');
});

test('lc probe: identity correction matrix is recorded as null', async () => {
  const s = setup({probe: {lc: true}});
  await callOnce(s);
  const P = s.sandbox.__memProbe;
  assert.equal(P.records[0][P.fields.indexOf('outLcMat')], null);
  assert.equal(P.records[0][P.fields.indexOf('outLcOk')], 0);
});

test('lc probe appended after paths fields when both are on', async () => {
  const s = setup({probe: {paths: true, lc: true}});
  await callOnce(s);
  const P = s.sandbox.__memProbe;
  assert.equal(P.fields.length, DEFAULT_FIELDS + 12 + 5);
  assert.equal(P.fields[DEFAULT_FIELDS + 12], 'outLcOk');
  assert.equal(P.records[0][P.fields.indexOf('wmKfDescCount')], 2);
});

test('poke lc: writes u8 @606264 before the call, 6-word pre/post, layout ends with lc', async () => {
  const s = setup({probe: {}, poke: {lc: 1}});
  await callOnce(s);
  const K = s.sandbox.__configPoke;
  assert.equal(K.error, null);
  assert.deepEqual(j(K.prePostLayout), ['PV', 'ba', 'baCache', 'mse', 'mseCache', 'lc']);
  assert.deepEqual(j(K.writes.map(w => [w.key, w.addr, w.value])), [['lc', 606264, 1]]);
  const r = K.records[0];
  assert.deepEqual(j(r[3]), [0, 0, 0, 0, 0, 0]);  // pre: read before the write
  assert.deepEqual(j(r[4]), [0, 0, 0, 0, 0, 1]);  // post
  assert.deepEqual(j(r[5]), []);                    // no gap
  assert.equal(s.u8[606264], 1);
});

test('poke lc=0 is a write of 0 (null arm), no gap', async () => {
  const s = setup({probe: {}, poke: {lc: 0}});
  s.u8[606264] = 1;  // pretend memory held 1
  await callOnce(s);
  assert.equal(s.sandbox.__configPoke.records[0][4][5], 0);
  assert.deepEqual(j(s.sandbox.__configPoke.records[0][5]), []);
});

test('poke lc: a getter that overwrites the byte inside the call is reported as gap', async () => {
  const s = setup({probe: {}, poke: {lc: 1}});
  s.sandbox.WebAssembly.instantiate = async () => ({exports: {memory: {buffer: s.buffer},
    c8EmAsm_processStagedFrame: () => { s.u8[606264] = 0; }}});
  // install order is fixed by setup(); re-run the init scripts over the replaced instantiate
  vm.runInContext(`${probeSrc}\nwindow.__memProbeInit(${JSON.stringify({targetSha256: SHA})});` +
    `${pokeSrc}\nwindow.__configPokeInit(${JSON.stringify({targetSha256: SHA, keys: {lc: 1}})});`, s.ctx);
  await callOnce(s);
  const r = s.sandbox.__configPoke.records[0];
  assert.deepEqual(j(r[5]), ['lc']);
  assert.equal(r[4][5], 0);
});

test('poke without lc: unchanged 5-word layout and no write at 606264', async () => {
  const s = setup({probe: {}, poke: {ba: 1}});
  await callOnce(s);
  const K = s.sandbox.__configPoke;
  assert.deepEqual(j(K.prePostLayout), ['PV', 'ba', 'baCache', 'mse', 'mseCache']);
  assert.equal(K.records[0][3].length, 5);
  assert.equal(K.records[0][4].length, 5);
  assert.equal(s.u8[606264], 0);
});

test('poke refuses lc values outside 0|1 and unknown keys', () => {
  for (const poke of [{lc: 2}, {lc: 'on'}, {loopCloseEnabled: 1}]) {
    const s = setup({probe: {}, poke});
    assert.match(s.sandbox.__configPoke.error, /refused poke key/);
  }
});

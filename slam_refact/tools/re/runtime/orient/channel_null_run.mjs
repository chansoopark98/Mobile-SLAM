// Channel null-run of the bit-exact channel (orient-us02x-gate.v2; review B-1 of v1): known float32 bit patterns -- +0, -0,
// denormals, the smallest normals, infinities, ordinary values -- are put through the SAME code that carries the func 647 probe
// and reality.intrinsics from the page to the TSV files (orient-probe.js __orientReadProbe647 / __orientCollect in headless Chrome,
// channel.mjs, extract_orient.py) and must come out unchanged. No original SDK, no engine.
//
//   node channel_null_run.mjs --out <new directory>
// Writes <out>/{meta.json, calls.json, probe647.json, intrinsics.json, packets.json, tsv/*} (a fabricated recording that only
// has the probe and intrinsics channels) and prints PASS / FAIL. The checked-in copy under tests/data/orient/channel-null-run
// is read by tests/engine/orient_gate_test.cc (loader side of the same channel).
import fs from 'node:fs/promises';
import path from 'node:path';
import {execFileSync} from 'node:child_process';
import {fileURLToPath} from 'node:url';
import {parseArgs} from 'node:util';
import {chromium} from '../../../../../scripts/dev/node_modules/playwright/index.mjs';
import {intrinsicsFile, probeFile} from './channel.mjs';

const here = path.dirname(fileURLToPath(import.meta.url));
const {values} = parseArgs({options: {out: {type: 'string'}}});
if (!values.out) throw new Error('--out <new directory> required');
const out = path.resolve(values.out);
await fs.mkdir(out, {recursive: false});

// Known patterns (float32 bits, lower case hex). NaN is left out: a NaN payload is not guaranteed to survive a double.
const PATTERNS = ['00000000', '80000000', '00000001', '80000001', '007fffff', '807fffff', '00800000', '80800000', '3f800000', 'bf800000',
  'c08ccccd', '7f7fffff', 'ff7fffff', '7f800000', 'ff800000', '3dcccccd'];
const bitsToFloat = (h) => { const u = new Uint32Array(1); u[0] = parseInt(h, 16); return new Float32Array(u.buffer)[0]; };

const browser = await chromium.launch({executablePath: process.env.BROWSER_EXECUTABLE || '/usr/bin/google-chrome', headless: true,
  args: ['--enable-unsafe-swiftshader']});
let result;
try {
  const page = await (await browser.newContext()).newPage();
  await page.addInitScript(await fs.readFile(path.join(here, 'orient-probe.js'), 'utf8'));
  await page.goto('about:blank');
  result = await page.evaluate((patterns) => {
    const fromBits = (h) => { const u = new Uint32Array(1); u[0] = parseInt(h, 16); return new Float32Array(u.buffer)[0]; };
    // func 647 probe: the layout instrument_647_wat.py writes (header u32 call count; slot k at +16 + k * 10256: u32 count, events at +16,
    // 40 bytes each: i32 kind@0, i64 time@8, i64 interval@16, float32 x/y/z@24/28/32). The floats are written as raw uint32.
    const memory = new WebAssembly.Memory({initial: 2});
    const dv = new DataView(memory.buffer);
    const ptr = 1024;
    dv.setUint32(ptr, 1, true);
    const slot = ptr + 16;
    const events = 5;  // 15 of the 16 patterns
    dv.setUint32(slot, events, true);
    for (let j = 0; j < events; j++) {
      const e = slot + 16 + j * 40;
      dv.setInt32(e, 1 + (j % 4), true);
      dv.setBigInt64(e + 8, BigInt(1000 + j), true);
      dv.setBigInt64(e + 16, 16666667n, true);
      for (let c = 0; c < 3; c++) dv.setUint32(e + 24 + 4 * c, parseInt(patterns[3 * j + c], 16), true);
    }
    const C = {};
    window.__orientReadProbe647(C, {sr_probe_647: {value: ptr}}, memory, 7);
    // reality.intrinsics: what page-runtime.js onUpdate stores (Array.from of the Float32Array of the response).
    window.__probe = {frames: [], epochWallMs: 0, events: [], intrinsicsChanges: [{textureWidth: 480, textureHeight: 640, orientation: 0, videoTime: 0.5,
      intrinsics: Array.from(new Float32Array(patterns.map(fromBits)))}], errors: [], rows: [], imuDispatched: 0, cameraRequests: []};
    return {probe647: C.probe647, collected: window.__orientCollect(), probeEventsAfterRead: new DataView(memory.buffer).getUint32(ptr, true)};
  }, PATTERNS);
} finally {
  await browser.close();
}

await fs.writeFile(path.join(out, 'probe647.json'), probeFile(result.probe647));
await fs.writeFile(path.join(out, 'intrinsics.json'), intrinsicsFile(result.collected.intrinsicsChanges));
await fs.writeFile(path.join(out, 'packets.json'), '[]');
await fs.writeFile(path.join(out, 'calls.json'), JSON.stringify([[1, 'I', 0, [480, 640, 0], null]]));
await fs.writeFile(path.join(out, 'meta.json'), JSON.stringify({scenario: 'channel-null-run', label: 'channel-null-run', ua: 'none', orientation: 0,
  deviceEstimate: {manufacturer: '', model: 'K', os: 'Android', osVersion: '10'}, input: {servedWidth: 480, servedHeight: 640}, captures: []}));
execFileSync('python3', ['-I', path.join(here, 'extract_orient.py'), out], {stdio: 'inherit'});

// Verify: every pattern comes out unchanged in ev647.tsv (x, y, z) and in intrinsics.tsv (m0..m15).
const tsv = async (f) => (await fs.readFile(path.join(out, 'tsv', f), 'utf8')).split('\n').filter(l => l && !l.startsWith('#')).map(l => l.split('\t'));
const ev = await tsv('ev647.tsv');
const got647 = ev.flatMap(r => r.slice(5, 8));
const gotK = (await tsv('intrinsics.tsv'))[0].slice(3, 19);
const ok647 = JSON.stringify(got647) === JSON.stringify(PATTERNS.slice(0, 15)), okK = JSON.stringify(gotK) === JSON.stringify(PATTERNS);
const selfCheck = bitsToFloat('80000000') === 0 && Object.is(bitsToFloat('80000000'), -0);
console.log(JSON.stringify({probe647_events: ev.length, probe647_bits_unchanged: ok647, intrinsics_bits_unchanged: okK, counterexample_json_number_loses_the_sign: JSON.stringify(-0) === '0', selfCheck,
  verdict: ok647 && okK && selfCheck ? 'PASS' : 'FAIL'}));
process.exitCode = ok647 && okK ? 0 : 1;

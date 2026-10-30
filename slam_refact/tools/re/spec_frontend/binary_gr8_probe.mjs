// Spec lane S1: execute the ORIGINAL xr-slam.wasm Gr8Gl CPU-half functions in isolation on a captured
// GPU pyramid and print the kept keypoints per level, in output order.
//   func917 = keypointsForCropJs(ConstRGBA8888PlanePixels*, scale, roi, edgeThreshold, vector<ImagePointLocation>*)
//   func918 = Gr8Gl::retainBest(vector*, nPoints, cols, rows, edgeThreshold)   (quantizeTo256 folded to true)
// Technique as scripts/dev/references/xr_reference/binary-residual-probe.mjs: a copy of the pinned module with
// extra exports appended to the export section; code section unchanged (hash checked); imports stubbed to throw.
//
//   node binary_gr8_probe.mjs --capture <cap.rgba.gz> --rows 660 --levels '<json levels>' --budgets 199,138,96,67
//        [--edge 21] --out <json>
import fs from 'node:fs';
import zlib from 'node:zlib';
import crypto from 'node:crypto';
import path from 'node:path';
import {parseArgs} from 'node:util';
import {fileURLToPath} from 'node:url';

const here = path.dirname(fileURLToPath(import.meta.url));
const root = path.resolve(here, '../../../..');
const {values} = parseArgs({options: {
  capture: {type: 'string'}, rows: {type: 'string', default: '660'}, levels: {type: 'string'},
  budgets: {type: 'string', default: '199,138,96,67'}, edge: {type: 'string', default: '21'}, out: {type: 'string'},
  wasm: {type: 'string', default: 'build/slam-refact-20261015/inputs/xr-slam.wasm'},
}});
const PINNED = '780731fe294a725acf1b7717ed5ffb07ad9f0929e408a1d74f47450c291ec422';
const hash = b => crypto.createHash('sha256').update(b).digest('hex');
const original = fs.readFileSync(path.resolve(root, values.wasm));
if (hash(original) !== PINNED) throw Error('input differs from pinned xr-slam.wasm');
const leb = n => { const b = []; do { let v = n & 127; n >>>= 7; if (n) v |= 128; b.push(v); } while (n); return Buffer.from(b); };
const read = (b, pos) => { let n = 0, shift = 0, v; do { v = b[pos++]; n |= (v & 127) << shift; shift += 7; } while (v & 128); return [n >>> 0, pos]; };
const extraExports = [['probeKeypointsForCropJs', 917], ['probeRetainBest', 918]];
const parts = [original.subarray(0, 8)];
let pos = 8, codeBefore = null, codeAfter = null;
while (pos < original.length) {
  const start = pos, id = original[pos++];
  const [length, p] = read(original, pos);
  pos = p + length;
  let payload = original.subarray(p, pos);
  if (id === 10) codeBefore = hash(payload);
  if (id === 7) {
    const [count, rest] = read(payload, 0);
    const extra = extraExports.map(([name, index]) => Buffer.concat([leb(name.length), Buffer.from(name), Buffer.from([0]), leb(index)]));
    payload = Buffer.concat([leb(count + extraExports.length), payload.subarray(rest), ...extra]);
    parts.push(Buffer.concat([Buffer.from([id]), leb(payload.length), payload]));
  } else parts.push(original.subarray(start, pos));
  if (id === 10) codeAfter = hash(payload);
}
if (codeBefore !== codeAfter) throw Error('code section changed');
const module = new WebAssembly.Module(Buffer.concat(parts));
const stubs = {}, importCalls = [];
for (const imp of WebAssembly.Module.imports(module)) {
  stubs[imp.module] ??= {};
  if (imp.kind === 'function') stubs[imp.module][imp.name] = (...args) => { importCalls.push(imp.name); throw Error('unexpected import ' + imp.name); };
}
const instance = new WebAssembly.Instance(module, stubs);
const ex = instance.exports;
const mem = () => new DataView(ex.memory.buffer);
const u8 = () => new Uint8Array(ex.memory.buffer);

const rows = Number(values.rows);
const atlas = zlib.gunzipSync(fs.readFileSync(path.resolve(root, values.capture)));
if (atlas.length !== rows * 1024 * 4) throw Error('capture size mismatch');
const levels = JSON.parse(values.levels);
const budgets = values.budgets.split(',').map(Number);
const edge = Number(values.edge);
const atlasPtr = ex.malloc(atlas.length);
u8().set(atlas, atlasPtr);
const result = {schema: 'slam-refact-spec-frontend-binary-gr8-v1', wasmSha256: PINNED, codeSectionSha256: codeBefore,
  exportsAdded: extraExports, capture: values.capture, edge, budgets, levels: []};
for (let li = 0; li < levels.length; li++) {
  const l = levels[li];
  const pix = ex.malloc(32), vec = ex.malloc(16);
  const dv = mem();
  dv.setUint32(pix, 0, true);              // vtable (unused by func917)
  dv.setInt32(pix + 4, l.h, true);         // rows
  dv.setInt32(pix + 8, l.w, true);         // cols
  dv.setInt32(pix + 12, 1024 * 4, true);   // rowBytes
  dv.setUint32(pix + 16, atlasPtr + l.r * 4096 + 4 * l.c, true);  // pixels
  dv.setUint32(vec, 0, true); dv.setUint32(vec + 4, 0, true); dv.setUint32(vec + 8, 0, true);
  ex.probeKeypointsForCropJs(pix, li, -1, edge, vec);
  const before = (mem().getUint32(vec + 4, true) - mem().getUint32(vec, true)) / 32;
  ex.probeRetainBest(vec, budgets[li], l.w, l.h, edge);
  const d2 = mem();
  const b = d2.getUint32(vec, true), e = d2.getUint32(vec + 4, true);
  const kept = [];
  for (let p = b; p < e; p += 32) {
    kept.push([d2.getFloat32(p, true), d2.getFloat32(p + 4, true), d2.getFloat32(p + 8, true), d2.getUint8(p + 12),
      d2.getFloat32(p + 16, true), d2.getFloat32(p + 20, true), d2.getFloat32(p + 24, true), d2.getInt8(p + 28), d2.getUint8(p + 29)]);
  }
  result.levels.push({level: li, candidates: before, kept: kept.length,
    fields: 'x,y,size,scale,angle,gravityAngle,response,roi,gravityPortion', list: kept});
}
result.importCalls = importCalls;
fs.mkdirSync(path.dirname(path.resolve(root, values.out)), {recursive: true});
fs.writeFileSync(path.resolve(root, values.out), JSON.stringify(result) + '\n');
console.log(JSON.stringify({out: values.out, levels: result.levels.map(l => ({level: l.level, candidates: l.candidates, kept: l.kept})), importCalls}));

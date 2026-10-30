// Exact-input replay of a run-orient.mjs recording through the WASM engine (same stage order as tests/engine/orient_replay.h);
// REPORT tool: compares the WASM trace with a native slam_refact_orient_replay trace.
//   node replay_wasm.mjs --rec <recording dir with tsv/> --wasm-dir <dir with slam_refact_engine.js> [--native <trace.jsonl>] [--out <new file>]
import fs from 'node:fs';
import path from 'node:path';
import { createRequire } from 'node:module';
import { parseArgs } from 'node:util';

const { values } = parseArgs({ options: { rec: { type: 'string' }, 'wasm-dir': { type: 'string' }, native: { type: 'string' }, out: { type: 'string' } } });
const require = createRequire(import.meta.url);
const tsv = (file) => fs.readFileSync(file, 'utf8').split('\n').filter((l) => l && !l.startsWith('#')).map((l) => l.split('\t'));
const fb = (n) => { const b = Buffer.alloc(4); b.writeUInt32LE(n); return b.readFloatLE(0); };
const bits = (v) => { const b = Buffer.alloc(4); b.writeFloatLE(v); return b.readUInt32LE(0); };
function readPgm(file) {
  const b = fs.readFileSync(file);
  let pos = 0; const fields = [];
  while (fields.length < 4) {
    while (b[pos] <= 32) pos++;
    const s = pos; while (b[pos] > 32) pos++;
    fields.push(b.subarray(s, pos).toString());
  }
  pos++;
  return { w: Number(fields[1]), h: Number(fields[2]), pixels: new Uint8Array(b.subarray(pos)) };
}

const rec = values.rec;
const device = Object.fromEntries(tsv(`${rec}/tsv/device.tsv`).map((r) => [r[0], r[1]]));
const [w, h, rot] = device.engine_init.split(' ').map(Number);
const msgs = [];
for (const r of tsv(`${rec}/tsv/raw-events.tsv`)) { (msgs[+r[0]] ??= []).push({ kind: +r[2], ts: Number(r[3]), xb: parseInt(r[5], 16), yb: parseInt(r[6], 16), zb: parseInt(r[7], 16) }); }
const packets = tsv(`${rec}/tsv/packets.tsv`).map((r) => ({ interval: Number(r[1]), ax: Number(r[2]), ay: Number(r[3]), az: Number(r[4]), a: Number(r[5]), b: Number(r[6]), g: Number(r[7]) }));
const Module = await require(path.join(values['wasm-dir'], 'slam_refact_engine.js'))();
const glue = new Module.SlamRefactGlue({ captureWidth: w, captureHeight: h, rotationDeg: rot, manufacturer: device.manufacturer, model: device.model, pyramidModel: 'swiftshader-approx' });
glue.updateCameraProjectionMatrix({ cam: { pixelRectWidth: Number(device.served_width), pixelRectHeight: Number(device.served_height), nearClipPlane: 0.01, farClipPlane: 1000 }, origin: { x: 0, y: 2, z: 0 }, facing: { w: 1, x: 0, y: 0, z: 0 } });
let cursor = 0;
const rows = [];
let gray0 = readPgm(`${rec}/served-00000.pgm`);
for (const st of tsv(`${rec}/tsv/stages.tsv`)) {
  const msg = Number(st[1]), frame = Number(st[2]);
  for (const ev of (msgs[msg] ?? []).reduce((acc, e, i, all) => (i % 3 === 0 && i + 2 < all.length ? [...acc, [all[i], all[i + 1], all[i + 2]]] : acc), [])) {
    const [lin, acc, gyr] = ev;
    let best = -1;
    for (let c = cursor; c < Math.min(packets.length, cursor + 3); c++) {
      const p = packets[c];
      if (bits(p.ax) === acc.xb && bits(p.ay) === acc.yb && bits(p.az) === acc.zb && bits(p.a) === gyr.xb && bits(p.b) === gyr.yb && bits(p.g) === gyr.zb) best = c;
    }
    if (best < 0) continue;
    cursor = best;
    const p = packets[best];
    glue.pushDeviceMotion(acc.ts, fb(acc.xb), fb(acc.yb), fb(acc.zb), true, fb(lin.xb), fb(lin.yb), fb(lin.zb),
      fb(gyr.xb), fb(gyr.yb), fb(gyr.zb), p.interval);
  }
  glue.pushDeviceOrientation(1, Number(st[6]), Number(st[7]), Number(st[8]));
  const img = frame >= 0 ? readPgm(`${rec}/served-${String(frame).padStart(5, '0')}.pgm`) : gray0;
  const frameNs = Math.trunc(Number(st[4]) * 1e9);
  const pose = glue.processFrame(frameNs !== 0 ? frameNs : Math.trunc(Number(st[5])), Math.trunc(Number(st[3]) * 1e9), img.pixels, img.w, img.h);
  if (frame >= 0) rows.push({ i: frame, fresh: !pose.repeated, status: pose.status, p: Array.from(pose.position), q: Array.from(pose.rotationWxyz) });
}
glue.delete();
if (values.out) fs.writeFileSync(values.out, rows.map((r) => JSON.stringify({ i: r.i, fresh: r.fresh, status: r.status, p: r.p, q_wxyz: r.q })).join('\n') + '\n', { flag: 'wx' });
let compared = 0, maxP = 0, maxQ = 0, bitEqual = 0;
if (values.native) {
  const nat = new Map(fs.readFileSync(values.native, 'utf8').split('\n').filter(Boolean).map((l) => JSON.parse(l)).map((r) => [r.i, r]));
  for (const r of rows) {
    const n = nat.get(r.i);
    if (!n || !r.fresh || !n.fresh) continue;
    compared++;
    maxP = Math.max(maxP, ...r.p.map((v, k) => Math.abs(v - n.p[k])));
    maxQ = Math.max(maxQ, ...r.q.map((v, k) => Math.abs(v - n.q_wxyz[k])));
    if (r.p.every((v, k) => Math.abs(v - n.p[k]) < 1e-6 * Math.max(1, Math.abs(v))) && r.q.every((v, k) => Math.abs(v - n.q_wxyz[k]) < 1e-7)) bitEqual++;
  }
}
console.log(JSON.stringify({ rec: path.basename(rec), rotation: rot, rows: rows.length, fresh: rows.filter((r) => r.fresh).length, comparedWithNative: compared,
  maxPositionDiff: maxP, maxQuaternionDiff: maxQ, withinPrintRounding: bitEqual }));

#!/usr/bin/env node
// Per-stage timing of the WASM engine in node (V8) on a prepared replay bundle (prepare_inputs.py output). DESKTOP numbers, not a phone's.
// The GPU pyramid of the browser path is replaced by the engine's own CPU shader port, built OUTSIDE the engine timer (cpuPyramidAtlas) and injected
// with processFramePyramid (bit-identical to the internal CPU path, tests/wasm/pyramid_injection.test.mjs). So `engineMs` approximates the engine
// step of the production browser path (without GPU draw/readback); `pyramidCpuMs` is the CPU port, which the browser does not run.
//   node slam_refact/tools/perf/wasm_stage_timing.mjs --bundle <dir> --wasm-dir <dir with slam_refact_engine.js> --out <new file under build/> [--max-frames N]
import fs from 'node:fs';
import path from 'node:path';
import { createRequire } from 'node:module';
import { parseArgs } from 'node:util';
import { readBundle, readPgm } from '../../engine/tests/wasm/bundle_io.mjs';

const repo = path.resolve(import.meta.dirname, '../../..');
const { values: a } = parseArgs({ options: { bundle: { type: 'string' }, 'wasm-dir': { type: 'string' }, out: { type: 'string' }, 'max-frames': { type: 'string', default: '0' } } });
if (!a.bundle || !a['wasm-dir'] || !a.out) throw new Error('--bundle --wasm-dir --out required');
const out = path.resolve(repo, a.out);
if (!out.startsWith(path.join(repo, 'build') + path.sep)) throw new Error('--out must be under build/');
if (fs.existsSync(out)) throw new Error(`refusing to overwrite ${out}`);

const bundle = readBundle(path.resolve(repo, a.bundle));
const cfgRaw = bundle.config;
const cfg = { estimateScale: cfgRaw.estimate_scale === '1', disableVio: cfgRaw.disable_vio === '1', captureWidth: Number(cfgRaw.capture_width),
  captureHeight: Number(cfgRaw.capture_height), rotationDeg: Number(cfgRaw.rotation_deg), manufacturer: cfgRaw.manufacturer ?? '', model: cfgRaw.model ?? '',
  pyramidModel: cfgRaw.pyramid_model, origin: cfgRaw.origin.split(' ').map(Number) };
if (cfgRaw.intrinsics) cfg.intrinsics = cfgRaw.intrinsics.split(' ').map(Number);
const Module = await createRequire(import.meta.url)(path.resolve(repo, a['wasm-dir'], 'slam_refact_engine.js'))();
const glue = new Module.SlamRefactGlue(cfg);
const push = (s) => {
  glue.pushDeviceMotion(s.t, s.aig[0], s.aig[1], s.aig[2], s.lin !== null, ...(s.lin ?? [0, 0, 0]), s.rate[0], s.rate[1], s.rate[2], s.intervalMs);
  glue.pushDeviceOrientation(s.t, s.orient[0], s.orient[1], s.orient[2]);
};
const rgba = (g) => { const o = new Uint8Array(g.length * 4); for (let i = 0; i < g.length; i++) { o[4 * i] = o[4 * i + 1] = o[4 * i + 2] = g[i]; o[4 * i + 3] = 255; } return o; };
const runFrame = (f) => {
  const img = readPgm(f.image);                       // I/O outside every timer
  const rgbaBuf = rgba(img.pixels);
  const t0 = performance.now();
  const atlas = Module.SlamRefactGlue.cpuPyramidAtlas(rgbaBuf, img.width, img.height, cfg.pyramidModel);
  const t1 = performance.now();
  const pose = glue.processFramePyramid(f.timeNs, f.videoTimeNs, atlas.rgba, atlas.size, atlas.rows);
  const t2 = performance.now();
  return { pose, pyramidCpuMs: t1 - t0, engineMs: t2 - t1 };
};
let next = 0;
while (next < bundle.sensors.length && bundle.sensors[next].bootstrap) push(bundle.sensors[next++]);
runFrame(bundle.frames[0]);                           // bootstrap frame, discarded (same as replay_main.cc)
const rows = [];
const limit = Number(a['max-frames']) > 0 ? Number(a['max-frames']) : Infinity;
for (const f of bundle.frames.slice(1)) {
  if (rows.length >= limit) break;
  while (next < bundle.sensors.length && bundle.sensors[next].t <= f.timeNs) push(bundle.sensors[next++]);
  if (!f.process) { rows.push({ i: f.index, fresh: false }); continue; }
  const r = runFrame(f);
  const p = r.pose;
  rows.push({ i: f.index, fresh: true, repeated: p.repeated, engineMs: r.engineMs, pyramidCpuMs: r.pyramidCpuMs, stageMs: p.stageMs ?? null,
    numKeyframes: p.tracker?.numKeyframes ?? null, mapReset: p.tracker?.mapReset ?? null, worldPoints: p.worldPoints, status: p.status });
}
glue.delete();
fs.mkdirSync(path.dirname(out), { recursive: true });
fs.writeFileSync(out, JSON.stringify({ schema: 'xr-perf-wasm-stage-v1', node: process.version, bundle: a.bundle, wasmDir: a['wasm-dir'], config: cfg, loadAvgAtEnd: (await import('node:os')).loadavg(), rows }) + '\n');
console.log(`wrote ${out} (${rows.length} rows)`);

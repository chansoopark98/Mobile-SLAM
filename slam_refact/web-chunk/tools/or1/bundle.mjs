// OR-1 input: an exact-input replay bundle (slam_refact/tools/harness/prepare_inputs.py format, read by replay_main.cc) in the form all
// three arms use. Read-only on the source bundle; derive() writes a NEW directory.
//
// The same bundle is played by (N) the native slam_refact_replay, (W) a direct WASM player and (C) the chunk, so the arms see one input.
// The derived bundle removes the things the arms cannot share (each is a documented, hashed transformation, not a silent edit):
//   - id_generator_seed: the recorded original run's map-id seed. The native runner applies it (SetReplayIdGeneratorSeed), the WASM
//     binding has no such call; keeping it would make N differ from W/C by construction. Dropped -> every arm uses the engine default.
//   - a processed input frame that has no recorded atlas (the capture of the original run covers n-1 frames, flip6-s2 has gaps) cannot be
//     given to the chunk, whose only image path is the GPU atlas. It is set to process = 0 (drop-masked) in every arm.
// and optionally truncates to the first `frames` input frames (dev data).
import fs from 'node:fs';
import path from 'node:path';
import zlib from 'node:zlib';
import { sha256 } from './canon.mjs';

export const ATLAS_WIDTH = 1024;
const lines = (file) => fs.readFileSync(file, 'utf8').split('\n').filter((l) => l);
const tsv = (file) => lines(file).map((l) => l.split('\t'));

export function readBundle(dir) {
  const config = new Map(tsv(path.join(dir, 'engine-config.tsv')).map(([k, ...v]) => [k, v.join('\t')]));
  const sensors = tsv(path.join(dir, 'sensors.tsv')).map((c, i) => {
    if (c.length !== 15) throw new Error(`${dir}/sensors.tsv line ${i + 1}: ${c.length} columns`);
    return { bootstrap: c[0] === 'bootstrap', timeNs: Number(c[1]), raw: c,
      aig: [c[2], c[3], c[4]].map(Number), lin: c[5] === '-' ? null : [c[5], c[6], c[7]].map(Number), rate: [c[8], c[9], c[10]].map(Number),
      intervalMs: Number(c[11]), orient: [c[12], c[13], c[14]].map(Number) };
  });
  const frames = tsv(path.join(dir, 'frames.tsv')).map((c, i) => {
    if (c.length !== 8 && c.length !== 9) throw new Error(`${dir}/frames.tsv line ${i + 1}: ${c.length} columns`);
    return { bootstrap: c[0] === 'bootstrap', index: Number(c[1]), timeNs: Number(c[2]), videoTimeNs: Number(c[3]), t: Number(c[4]),
      process: c[5] === '1', image: c[6], pyramid: c[7] === '-' ? null : c[7], frameTimeNs: c.length === 9 ? Number(c[8]) : 0, raw: c };
  });
  return { dir, config, sensors, frames };
}

// gzip RGBA atlas of width 1024 -> {rgba: Uint8Array, rows}
export function readAtlas(file) {
  const rgba = new Uint8Array(zlib.gunzipSync(fs.readFileSync(file)));
  const rowBytes = 4 * ATLAS_WIDTH;
  if (rgba.length === 0 || rgba.length % rowBytes !== 0) throw new Error(`${file}: not a multiple of 1024 RGBA texels`);
  return { rgba, rows: rgba.length / rowBytes };
}

// Bundle -> list of the files it reads (for the input sha256 record). Paths are relative to the repo root unless absolute.
export function inputFiles(bundle, repoRoot) {
  const abs = (p) => (path.isAbsolute(p) ? p : path.join(repoRoot, p));
  const files = new Set();
  for (const f of bundle.frames) { if (f.pyramid) files.add(abs(f.pyramid)); }
  return [...files].sort();
}

// derive(srcDir, outDir, {frames}) -> {outDir, droppedKeys, frames, sensors, sha256: {engine-config, sensors, frames}, notes}
export function derive(srcDir, outDir, { frames: maxFrames = 0, sourceLabel = null } = {}) {
  if (fs.existsSync(outDir)) throw new Error(`refusing to overwrite ${outDir}`);
  const src = readBundle(srcDir);
  const keepFrames = [src.frames[0], ...(maxFrames > 0 ? src.frames.slice(1, 1 + maxFrames) : src.frames.slice(1))];
  const lastNs = keepFrames.at(-1).timeNs;
  const sensors = src.sensors.filter((s) => s.bootstrap || s.timeNs <= lastNs);
  fs.mkdirSync(outDir, { recursive: true });
  const dropped = [];
  const cfg = [...src.config].filter(([k]) => { if (k === 'id_generator_seed') { dropped.push(k); return false; } return true; });
  const write = (name, text) => fs.writeFileSync(path.join(outDir, name), text);
  write('engine-config.tsv', cfg.map(([k, v]) => `${k}\t${v}\n`).join(''));
  write('sensors.tsv', sensors.map((s) => `${s.raw.join('\t')}\n`).join(''));
  const noAtlas = [];
  const frameLine = (f) => {
    const c = [...f.raw];  // all columns: column 9 (frame_time_ns, what func 647 picks) is part of the recorded input and stays
    if (!f.bootstrap && f.process && !f.pyramid) { c[5] = '0'; noAtlas.push(f.index); }
    return `${c.join('\t')}\n`;
  };
  write('frames.tsv', keepFrames.map(frameLine).join(''));
  const sums = Object.fromEntries(['engine-config.tsv', 'sensors.tsv', 'frames.tsv'].map((n) => [n, sha256(fs.readFileSync(path.join(outDir, n)))]));
  const meta = { schema: 'or1-derived-bundle-v1', source: sourceLabel ?? srcDir, sourceSha256: Object.fromEntries(['engine-config.tsv', 'sensors.tsv', 'frames.tsv']
    .map((n) => [n, sha256(fs.readFileSync(path.join(srcDir, n)))])), derivedSha256: sums, droppedConfigKeys: dropped,
    frameTimeColumn: src.frames.some((f) => f.frameTimeNs !== 0), framesWithoutAtlasDisabled: noAtlas, frames: keepFrames.length - 1, sensorRows: sensors.length,
    truncatedTo: maxFrames > 0 ? maxFrames : null };
  write('or1-derived.json', `${JSON.stringify(meta, null, 1)}\n`);
  return { outDir, ...meta };
}

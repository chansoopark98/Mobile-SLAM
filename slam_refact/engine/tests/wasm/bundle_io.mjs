// Replay bundle I/O for node (prepare_inputs.py text bundle, binary PGM frames).
import fs from 'node:fs';
import path from 'node:path';

export function readBundle(dir) {
  const tsv = (name) => fs.readFileSync(path.join(dir, name), 'utf8').split('\n').filter((l) => l)
    .map((l) => l.split('\t'));
  const config = Object.fromEntries(tsv('engine-config.tsv').map(([k, v]) => [k, v]));
  const sensors = tsv('sensors.tsv').map((c) => ({
    bootstrap: c[0] === 'bootstrap', t: Number(c[1]),
    aig: [Number(c[2]), Number(c[3]), Number(c[4])],
    lin: c[5] === '-' ? null : [Number(c[5]), Number(c[6]), Number(c[7])],
    rate: [Number(c[8]), Number(c[9]), Number(c[10])], intervalMs: Number(c[11]),
    orient: [Number(c[12]), Number(c[13]), Number(c[14])],
  }));
  const frames = tsv('frames.tsv').map((c) => ({
    bootstrap: c[0] === 'bootstrap', index: Number(c[1]), timeNs: Number(c[2]),
    videoTimeNs: Number(c[3]), t: Number(c[4]), process: c[5] === '1', image: c[6],
  }));
  return { config, sensors, frames };
}

// Binary PGM (P5, maxval 255) -> {width, height, pixels}.
export function readPgm(file) {
  const data = fs.readFileSync(file);
  const fields = [];
  let pos = 0;
  while (fields.length < 4) {
    while (/\s/.test(String.fromCharCode(data[pos]))) pos++;
    if (data[pos] === 0x23) { while (data[pos] !== 0x0a) pos++; continue; }
    const start = pos;
    while (!/\s/.test(String.fromCharCode(data[pos]))) pos++;
    fields.push(data.subarray(start, pos).toString('latin1'));
  }
  pos++;
  const [magic, w, h] = [fields[0], Number(fields[1]), Number(fields[2])];
  if (magic !== 'P5' || fields[3] !== '255') throw new Error(`unexpected PGM ${file}`);
  return { width: w, height: h, pixels: new Uint8Array(data.subarray(pos, pos + w * h)) };
}

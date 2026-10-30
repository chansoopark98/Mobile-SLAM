// or1-chunk-replay-gate.v4 compat check (G4, items 4 and 5 of the registration's section "compat"): the arms W and C of a replay harness directory on one
// bundle, reduced to the sha256 of their complete output (frames, rows, answers, calls, glue config, layout, errors, events, inexact counters; wall times excluded).
// Run it once per players.mjs and compare the two lines: the portrait rotation-0 default output of the harness did not change when players.mjs learned
// rotation 90 / 180 / 270. One process per players.mjs (two copies of players.mjs in one process share the chunk module URL and the second would not get its
// glue hook, so this is a CLI and not a function).
//   node compat-check.mjs --players-dir <dir with players.mjs, canon.mjs, bundle.mjs> --bundle <bundle dir> --engine-dir <dir> [--chunk-dir <dir>] [--dump <new file>]
import fs from 'node:fs';
import path from 'node:path';
import { createHash } from 'node:crypto';
import { parseArgs } from 'node:util';
import { pathToFileURL } from 'node:url';

const root = path.resolve(import.meta.dirname, '../../../..');
const { values } = parseArgs({ options: { 'players-dir': { type: 'string' }, bundle: { type: 'string' }, 'engine-dir': { type: 'string' }, 'chunk-dir': { type: 'string' }, dump: { type: 'string' } } });
const dir = path.resolve(root, values['players-dir'] ?? 'slam_refact/web-chunk/tools/or1');
const players = await import(pathToFileURL(path.join(dir, 'players.mjs')).href);
const { readBundle } = await import(pathToFileURL(path.join(dir, 'bundle.mjs')).href);
const bundle = readBundle(path.resolve(root, values.bundle));
const engineDir = path.resolve(root, values['engine-dir']);
const chunkDir = path.resolve(root, values['chunk-dir'] ?? 'slam_refact/web-chunk');
const part = (r) => JSON.stringify({ arm: r.arm, layout: r.layout, glueConfig: r.glueConfig, frames: r.frames, rows: r.rows, answers: r.answers, calls: r.calls, errors: r.errors, events: r.events, inexact: r.inexact });
const W = await players.playDirect({ engineDir, bundle, repoRoot: root });
const C = await players.playChunk({ chunkDir, engineDir, bundle, repoRoot: root });
const sha = (s) => createHash('sha256').update(s).digest('hex');
const out = { playersSha256: sha(fs.readFileSync(path.join(dir, 'players.mjs'))), W: sha(part(W)), C: sha(part(C)), wBytes: part(W).length, cBytes: part(C).length,
  freshRowsW: W.rows.filter((r) => r.fresh).length, freshRowsC: C.rows.filter((r) => r.fresh).length, layoutRows: C.layout.rows };
if (values.dump) fs.writeFileSync(path.resolve(root, values.dump), `${JSON.stringify({ W: JSON.parse(part(W)), C: JSON.parse(part(C)) })}\n`, { flag: 'wx' });
console.log(JSON.stringify(out));

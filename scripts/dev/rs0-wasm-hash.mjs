// RS-0c worker: one (engine dir, bundle, maxFrames) through the WASM engine, hashed exactly like
// build/audit-run1-20261115/rs0/scripts/wasm-parity.mjs (rows without wallMs and pose.stageMs).
// Prints {sha256, rows, fresh}. Run by scripts/dev/rs0-gate.py, one process per item so items run in parallel.
//   node rs0-wasm-hash.mjs <engine dir with slam_refact_engine.js> <bundle dir> <maxFrames>
import path from 'node:path';
import { createHash } from 'node:crypto';
import { createRequire } from 'node:module';
import { fileURLToPath } from 'node:url';

const here = path.dirname(fileURLToPath(import.meta.url));
const wasmTests = path.join(here, '../../slam_refact/engine/tests/wasm');
const { readBundle, readPgm } = await import(path.join(wasmTests, 'bundle_io.mjs'));
const { replay } = await import(path.join(wasmTests, 'bundle_replay.mjs'));
const require = createRequire(import.meta.url);

const [dir, bundleDir, maxFrames] = process.argv.slice(2);
const n = Number(maxFrames);
if (!dir || !bundleDir || !Number.isInteger(n) || n < 1) {
  console.error('usage: node rs0-wasm-hash.mjs <engine dir> <bundle dir> <maxFrames>');
  process.exit(2);
}
const Module = await require(path.resolve(dir, 'slam_refact_engine.js'))();
const rows = replay(Module, readBundle(bundleDir), n, (f) => readPgm(f.image));
const x = createHash('sha256');
// Frames whose stage time reached the engine's solver time cap (pose estimation 1.0 s in mapTracker, iterative BA
// 0.2 s in mapBuilder): the solve may have stopped on wall-clock time. Reported separately, not hashed.
const caps = { mapTracker: 1000, mapBuilder: 200 };
const stallFrames = rows.filter((r) => r.pose?.stageMs && Object.entries(caps).some(([k, c]) => r.pose.stageMs[k] >= c)).length;
for (const r of rows) {
  const pose = r.pose ? { ...r.pose } : r.pose;
  if (pose) delete pose.stageMs;
  x.update(JSON.stringify({ i: r.i, fresh: r.fresh, pose }) + '\n');
}
console.log(JSON.stringify({ sha256: x.digest('hex'), rows: rows.length, fresh: rows.filter((r) => r.fresh).length, stallFrames }));

// WASM TUM clip (US-012, evidence tier): 120 input frames of a prepared TUM replay bundle through
// the WASM GlueRunner; poses must be finite and, compared with the native slam_refact_replay
// trace of the same bundle, frames 0-30 must stay within the V-PD TUM row (rotation p95/max
// 0.5/1.0 deg, position p95/max 0.02/0.066). This is native vs WASM, not original parity.
//   SLAM_REFACT_BUNDLE=<bundle dir> SLAM_REFACT_NATIVE_TRACE=<trace.jsonl> [SLAM_REFACT_RESULT=<json>]
import assert from 'node:assert/strict';
import fs from 'node:fs';
import path from 'node:path';
import { createRequire } from 'node:module';
import test from 'node:test';
import { readBundle, readPgm } from './bundle_io.mjs';
import { poseDiff, replay } from './bundle_replay.mjs';

const repo = path.resolve(import.meta.dirname, '../../../..');
const E = path.join(repo, 'build/slam-refact-20261015');
const wasmDir = process.env.SLAM_REFACT_WASM_DIR ?? path.join(repo, 'build/slam-refact-wasm/wasm');
const bundleDir = process.env.SLAM_REFACT_BUNDLE ?? path.join(E, 'harness/bundles/tum-room1-full/wp5b-r2');
const nativeTrace = process.env.SLAM_REFACT_NATIVE_TRACE ?? path.join(E, 'engine-runs/tum-room1-full/wp5b-r2/trace.jsonl');
const require = createRequire(import.meta.url);

function nearestRank(values, p) {
  const s = [...values].sort((a, b) => a - b);
  return s[Math.max(0, Math.ceil((p / 100) * s.length) - 1)];
}

test('TUM 120-frame clip: finite poses, frames 0-30 match native within the V-PD TUM row', async () => {
  const Module = await require(path.join(wasmDir, 'slam_refact_engine.js'))();
  const bundle = readBundle(bundleDir);
  const rows = replay(Module, bundle, 120, (f) => readPgm(f.image));
  const native = fs.readFileSync(nativeTrace, 'utf8').split('\n').filter((l) => l).map((l) => JSON.parse(l));
  const fresh = rows.filter((r) => r.fresh);
  assert.ok(fresh.length >= 100);
  for (const r of fresh) assert.ok([...r.pose.position, ...r.pose.rotationWxyz].every(Number.isFinite));
  const rot = [], pos = [];
  let statusEqual = 0;
  for (const r of rows.slice(0, 31)) {
    const n = native[r.i];
    assert.equal(n.i, r.i);
    if (!r.fresh || !n.fresh) continue;
    const d = poseDiff(r.pose, { position: n.p, rotationWxyz: n.q_wxyz });
    rot.push(d.rot); pos.push(d.pos);
    statusEqual += r.pose.status === n.status ? 1 : 0;
  }
  const all = [];
  for (const r of fresh) {
    const n = native[r.i];
    all.push(poseDiff(r.pose, { position: n.p, rotationWxyz: n.q_wxyz }));
  }
  const result = {
    wasmDir, bundleDir, nativeTrace, frames: rows.length, fresh: fresh.length,
    window0to30: { n: rot.length, rotP95: nearestRank(rot, 95), rotMax: Math.max(...rot),
      posP95: nearestRank(pos, 95), posMax: Math.max(...pos), statusEqual },
    all120: { rotMax: Math.max(...all.map((d) => d.rot)), posMax: Math.max(...all.map((d) => d.pos)),
      exactFrames: all.filter((d) => d.rot === 0 && d.pos === 0).length },
    wallMs: { p50: nearestRank(fresh.map((r) => r.wallMs), 50), p95: nearestRank(fresh.map((r) => r.wallMs), 95),
      p99: nearestRank(fresh.map((r) => r.wallMs), 99) },
  };
  if (process.env.SLAM_REFACT_RESULT) fs.writeFileSync(process.env.SLAM_REFACT_RESULT, JSON.stringify(result, null, 1) + '\n', { flag: 'wx' });
  console.log(JSON.stringify(result));
  assert.ok(result.window0to30.rotP95 <= 0.5 && result.window0to30.rotMax <= 1.0, JSON.stringify(result.window0to30));
  assert.ok(result.window0to30.posP95 <= 0.02 && result.window0to30.posMax <= 0.066, JSON.stringify(result.window0to30));
});

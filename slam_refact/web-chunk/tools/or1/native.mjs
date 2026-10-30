// OR-1 arm N: the native slam_refact_replay (tools/harness/replay/replay_main.cc) on a bundle. One scenario per process, a NEW output
// directory per run (the runner refuses an existing one). The trace it writes is the JS-Number text of the engine's float outputs, so
// reading a value back as a double is exact and its bits can be compared with the WASM arms' doubles.
import fs from 'node:fs';
import path from 'node:path';
import { spawnSync } from 'node:child_process';
import { sha256 } from './canon.mjs';

export function runNative({ bin, bundleDir, outDir, repoRoot, env = {} }) {
  const r = spawnSync(path.resolve(repoRoot, bin), ['--bundle', bundleDir, '--out', outDir], { cwd: repoRoot, encoding: 'utf8', env: { ...process.env, ...env } });
  if (r.status !== 0) throw new Error(`native replay failed (${r.status}): ${(r.stderr || r.stdout).slice(0, 600)}`);
  return readTrace(outDir);
}

// trace.jsonl -> [{index, fresh, raw: {...}}] (the same `raw` shape the WASM arms produce, for the fields the trace has)
export function readTrace(outDir) {
  const text = fs.readFileSync(path.join(outDir, 'trace.jsonl'), 'utf8');
  const rows = text.split('\n').filter((l) => l).map((l) => {
    const j = JSON.parse(l);
    if (!j.fresh) return { index: j.i, fresh: false };
    return { index: j.i, fresh: true, raw: { position: j.p, rotationWxyz: j.q_wxyz, status: j.status, reason: j.reason, repeated: j.srRepeated,
      sourceFrameIndex: j.srSourceFrame, worldPoints: j.worldPoints } };
  });
  return { rows, traceSha256: sha256(fs.readFileSync(path.join(outDir, 'trace.jsonl'))), meta: JSON.parse(fs.readFileSync(path.join(outDir, 'meta.json'), 'utf8')) };
}

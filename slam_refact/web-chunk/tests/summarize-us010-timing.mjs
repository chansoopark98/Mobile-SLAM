// Aggregates the US-010 timing evidence (3 runs per condition, per-run values and the median of the three) into
// build/slam-refact-full-20261019/chunk/us010-20261020/exec-model-summary.json and prints a compact table.
//   node slam_refact/web-chunk/tests/summarize-us010-timing.mjs
import fs from 'node:fs/promises';
import path from 'node:path';

const repo = path.resolve(import.meta.dirname, '../../..');
const E = path.join(repo, 'build/slam-refact-full-20261019/chunk/us010-20261020');
const read = async (p) => JSON.parse(await fs.readFile(path.join(E, p), 'utf8'));
const med = (xs) => { const s = xs.filter(Number.isFinite).sort((a, b) => a - b); return s.length ? s[Math.floor((s.length - 1) / 2)] : null; };
const r2 = (v) => (v === null || v === undefined ? null : +v.toFixed(2));
const st = (s) => (s && s.n ? { n: s.n, p50: r2(s.p50), p95: r2(s.p95), p99: r2(s.p99), max: r2(s.max) } : null);
const rafRow = (raf) => (raf ? { ...st(raf), over25: raf.over25, over50: raf.over50, over25Fraction: r2(raf.over25 / raf.n) } : null);
const medianOf = (rows, pick) => {
  const o = {};
  const first = pick(rows[0]);
  if (!first) return null;
  for (const k of Object.keys(first)) o[k] = r2(med(rows.map((r) => pick(r)?.[k])));
  return o;
};

const out = { schema: 'us010-exec-model-summary-v1', generatedAt: new Date().toISOString(), note: 'desktop headless Chromium + SwiftShader; a 32-logical-CPU machine with other work running (loadAvgAtStart per run); P/E cores not distinguished; not a phone' };

// A: the sample under XR8 with the chunk (timing-r1..3) and the render-only baseline (fake chunk, timing-base-r1..3)
for (const [key, prefix] of [['xr8SyncChunk', 'timing-r'], ['xr8FakeChunkBaseline', 'timing-base-r'], ['xr8NoEngineAblation', 'abl-noeng-r']]) {
  const runs = [];
  for (const i of [1, 2, 3]) {
    try { runs.push({ label: prefix + i, meta: await read(`runs/threejs/${prefix}${i}/meta.json`) }); } catch { /* missing */ }
  }
  if (!runs.length) continue;
  const phases = {};
  for (const ph of ['all', 'pre', 'post']) {
    const rows = runs.map((r) => ({ label: r.label, t: r.meta.timing?.[ph] })).filter((r) => r.t);
    if (!rows.length) continue;
    phases[ph] = {
      perRun: rows.map((r) => ({ label: r.label, freshUpdates: r.t.freshUpdates, raf: rafRow(r.t.raf), longtasks: r.t.longtasks, stageMs: st(r.t.stageMs), readMs: st(r.t.readMs),
        engineMs: st(r.t.engineMs), chunkMainThreadMs: st(r.t.mainThreadChunkMs), xrGpuMs: st(r.t.xrGpuMs), xrCpuMs: st(r.t.xrCpuMs) })),
      median: {
        raf: medianOf(rows, (r) => rafRow(r.t.raf)), chunkMainThreadMs: medianOf(rows, (r) => st(r.t.mainThreadChunkMs)), engineMs: medianOf(rows, (r) => st(r.t.engineMs)),
        stageMs: medianOf(rows, (r) => st(r.t.stageMs)), readMs: medianOf(rows, (r) => st(r.t.readMs)), xrCpuMs: medianOf(rows, (r) => st(r.t.xrCpuMs)),
      },
    };
  }
  out[key] = { runs: runs.map((r) => ({ label: r.label, loadAvgAtStart: r.meta.loadAvgAtStart, drops: r.meta.timing?.drops, firstWorldPoints: r.meta.timing?.firstWorldPoints,
    status: r.meta.status, result: r.meta.result })), phases };
}

// C: replay (idle, sync, worker)
out.replay = {};
for (const mode of ['idle', 'sync', 'worker']) {
  const runs = [];
  for (const i of [1, 2, 3]) {
    try { runs.push({ label: `rep${i}-${mode}`, meta: await read(`replay/rep${i}-${mode}/meta.json`), summary: await read(`replay/rep${i}-${mode}/summary.json`) }); } catch { /* missing */ }
  }
  if (!runs.length) continue;
  const phases = {};
  for (const ph of ['all', 'pre', 'post']) {
    const rows = runs.map((r) => r.summary[ph]).filter(Boolean);
    if (!rows.length) continue;
    const fields = mode === 'sync' ? ['syncMs'] : mode === 'worker' ? ['workerMs', 'rttMs', 'allocMs', 'postMs', 'handlerMs', 'workerMainThreadMs'] : [];
    phases[ph] = {
      perRun: runs.map((r) => ({ label: r.label, frames: r.summary[ph]?.frames, raf: rafRow(r.summary[ph]?.raf), longtasks: r.summary[ph]?.longtasks,
        ...Object.fromEntries(fields.map((f) => [f, st(r.summary[ph]?.[f])])), lateStartMs: st(r.summary[ph]?.lateStartMs) })),
      median: { raf: medianOf(rows, (r) => rafRow(r.raf)), ...Object.fromEntries(fields.map((f) => [f, medianOf(rows, (r) => st(r[f]))])) },
    };
  }
  out.replay[mode] = { runs: runs.map((r) => ({ label: r.label, loadAvgAtStart: r.meta.loadAvgAtStart, engineLoadMs: r.summary.engineLoadMs, matchesManifest: r.summary.matchesManifest,
    status: r.meta.status, captureSha256: r.meta.captureSha256 })), phases };
}
await fs.writeFile(path.join(E, 'exec-model-summary.json'), JSON.stringify(out, null, 1) + '\n');

const row = (name, s) => (s ? `${name.padEnd(34)} p50 ${String(s.p50).padStart(7)}  p95 ${String(s.p95).padStart(7)}  p99 ${String(s.p99).padStart(7)}  max ${String(s.max).padStart(7)}` : `${name.padEnd(34)} -`);
for (const key of ['xr8SyncChunk', 'xr8FakeChunkBaseline', 'xr8NoEngineAblation']) {
  const post = out[key]?.phases?.post ?? out[key]?.phases?.all;  // the baselines have no world points, so no pre/post split
  if (!post) continue;
  console.log(`\n== ${key} ${out[key]?.phases?.post ? 'post-init' : 'whole run'}, median of ${out[key].runs.length} runs; longtasks per run`, out[key].phases[out[key]?.phases?.post ? 'post' : 'all'].perRun.map((r) => r.longtasks.count).join('/'));
  console.log(row('rAF interval ms', post.median.raf), ' over25 fraction', post.median.raf?.over25Fraction, 'over50', post.median.raf?.over50);
  console.log(row('chunk main thread (stage+read+eng)', post.median.chunkMainThreadMs));
  console.log(row('  stage', post.median.stageMs)); console.log(row('  read', post.median.readMs)); console.log(row('  engine', post.median.engineMs));
  console.log(row('xr cpu cycle', post.median.xrCpuMs));
}
for (const mode of ['idle', 'sync', 'worker']) {
  const post = out.replay[mode]?.phases?.post;
  if (!post) continue;
  console.log(`\n== replay ${mode} post-init, median of ${out.replay[mode].runs.length} runs`);
  console.log(row('rAF interval ms', post.median.raf), ' over25', post.median.raf?.over25, 'over50', post.median.raf?.over50);
  for (const f of ['syncMs', 'workerMs', 'rttMs', 'allocMs', 'postMs', 'handlerMs', 'workerMainThreadMs']) if (post.median[f]) console.log(row(f, post.median[f]));
}

// Aggregates the US-010b evidence (preregistration us010-samples.v3.md section 3): the ABAB timing series (conditions S, Sx, N, F) and the replay
// series (idle / sync / worker), per run with its load, medians, the registered attribution rule, the four desktop budgets.
//   node slam_refact/web-chunk/tests/summarize-us010b.mjs --series t1 --replay-prefix v3rep
// Writes build/slam-refact-full-20261019/chunk/us010-20261020/exec-model-summary-v3.json
import fs from 'node:fs/promises';
import path from 'node:path';
import { parseArgs } from 'node:util';

const repo = path.resolve(import.meta.dirname, '../../..');
const { values } = parseArgs({ options: { series: { type: 'string' }, 'replay-prefix': { type: 'string' }, rounds: { type: 'string', default: '5' },
  // v4: the evidence root (default: the v3 one), the directory of the series inside it, and the name of the summary file
  'evidence-root': { type: 'string', default: 'build/slam-refact-full-20261019/chunk/us010-20261020' }, 'series-dir': { type: 'string', default: 'series-v3' },
  'summary-name': { type: 'string', default: 'exec-model-summary-v3.json' } } });
const E = path.resolve(repo, values['evidence-root']);
if (!E.startsWith(path.join(repo, 'build') + path.sep)) throw new Error('--evidence-root must be under build/');
const read = async (p) => JSON.parse(await fs.readFile(path.join(E, p), 'utf8'));
const med = (xs) => { const s = xs.filter(Number.isFinite).sort((a, b) => a - b); return s.length ? (s.length % 2 ? s[(s.length - 1) / 2] : (s[s.length / 2 - 1] + s[s.length / 2]) / 2) : null; };
const r2 = (v) => (v === null || v === undefined || !Number.isFinite(v) ? null : +v.toFixed(3));
const out = { schema: 'us010b-exec-model-summary-v1', generatedAt: new Date().toISOString(),
  note: 'desktop headless Chromium + SwiftShader; i9-13900K, node + Chromium pinned to P cores 0-15 (taskset); other work on the machine is not controlled (loadavg per run); not a phone' };

// ---- timing series
const series = await read(`${values['series-dir']}/${values.series}/series.json`);
const rows = [];
for (const r of series.runs) {
  const meta = await read(`runs/threejs/${r.label}/meta.json`);
  const state = await read(`runs/threejs/${r.label}/state.json`);
  const t = meta.timing;
  const phase = t?.post ?? t?.all;   // post = after the first non-padding world point; N and F never get one -> whole run
  const usedPhase = t?.post ? 'post' : 'all';
  const rb = state.chunkDiag?.session?.readback ?? null;
  rows.push({ ...r, usedPhase, rafIntervals: phase.raf.n, over25: phase.raf.over25, over25Fraction: r2(phase.raf.over25 / phase.raf.n), over50: phase.raf.over50,
    rafP50: r2(phase.raf.p50), rafP95: r2(phase.raf.p95), rafP99: r2(phase.raf.p99), rafMax: r2(phase.raf.max), longtasks: phase.longtasks.count, longtaskMs: r2(phase.longtasks.totalMs),
    chunkMainP99: r2(phase.mainThreadChunkMs?.p99), chunkMainP50: r2(phase.mainThreadChunkMs?.p50), engineP99: r2(phase.engineMs?.p99), stageP99: r2(phase.stageMs?.p99), readP99: r2(phase.readMs?.p99),
    notSignaledAtRead: rb?.notSignaledAtRead ?? null, readbackReads: rb?.read ?? null, freshUpdates: t.drops.freshUpdates, realityUpdates: meta.timing.drops.freshUpdates + meta.timing.drops.repeatUpdates,
    firstReal: t.firstWorldPoints, status: meta.status });
}
out.timingRuns = rows;
const by = (c) => rows.filter((x) => x.cond === c);
out.timingMedians = Object.fromEntries(['S', 'Sx', 'N', 'F'].map((c) => [c, { runs: by(c).length, over25Fraction: r2(med(by(c).map((x) => x.over25Fraction))), over50: med(by(c).map((x) => x.over50)), rafP99: r2(med(by(c).map((x) => x.rafP99))),
  longtasks: med(by(c).map((x) => x.longtasks)), loadavg1Start: r2(med(by(c).map((x) => x.loadavg1Start))), pCoreBusy: r2(med(by(c).map((x) => x.pCoreBusy))), eCoreBusy: r2(med(by(c).map((x) => x.eCoreBusy))),
  notSignaledFraction: r2(med(by(c).map((x) => (x.readbackReads ? x.notSignaledAtRead / x.readbackReads : NaN)))), chunkMainP99: r2(med(by(c).map((x) => x.chunkMainP99))),
  freshUpdates: med(by(c).map((x) => x.freshUpdates)), realityUpdates: med(by(c).map((x) => x.realityUpdates)) }]));
// registered attribution rule: same sign in >= 4 of 5 rounds, median difference >= 3 percentage points, median loadavg1 difference < 2
const rule = (a, b) => {
  const rounds = [...new Set(rows.map((x) => x.round))];
  const diffs = rounds.map((r) => { const x = rows.find((q) => q.round === r && q.cond === a), y = rows.find((q) => q.round === r && q.cond === b); return x && y ? { round: r, diff: r2(x.over25Fraction - y.over25Fraction), loadDiff: r2(x.loadavg1Start - y.loadavg1Start) } : null; }).filter(Boolean);
  const pos = diffs.filter((d) => d.diff > 0).length, neg = diffs.filter((d) => d.diff < 0).length;
  const medDiff = med(diffs.map((d) => d.diff)), medLoad = med(by(a).map((x) => x.loadavg1Start)) - med(by(b).map((x) => x.loadavg1Start));
  const sameSign = Math.max(pos, neg) >= 4;
  const attributable = sameSign && Math.abs(medDiff) >= 0.03 && Math.abs(medLoad) < 2;
  return { pair: `${a} - ${b}`, perRound: diffs, positiveRounds: pos, negativeRounds: neg, medianDiffOver25: r2(medDiff), medianLoadDiff: r2(medLoad), sameSignInAtLeast4: sameSign, attributable,
    verdict: attributable ? (medDiff > 0 ? `${a} worse than ${b}` : `${a} better than ${b}`) : 'unresolved (confounded or inconsistent)' };
};
out.attribution = { engine: rule('S', 'N'), probeHits: rule('S', 'Sx'), glueGlPath: rule('N', 'F') };
// desktop budgets (spec: no decision)
const p99 = med(by('S').map((x) => x.chunkMainP99));
out.desktopBudgets = { chunkMainThreadP99Ms_medianOfS: p99, '60Hz_full_16.7': p99 > 16.7, '120Hz_full_8.33': p99 > 8.33, '60Hz_half_8.33': p99 > 8.33, '120Hz_half_4.17': p99 > 4.17,
  note: 'desktop result decides nothing (v3 1.3); sync stays the default; the device rule applies only after the user starts a device measurement' };

// ---- replay series
if (values['replay-prefix']) {
  out.replay = {};
  for (const mode of ['idle', 'sync', 'worker']) {
    const runs = [];
    for (let i = 1; i <= Number(values.rounds); i++) {
      try { runs.push({ label: `${values['replay-prefix']}${i}-${mode}`, meta: await read(`replay/${values['replay-prefix']}${i}-${mode}/meta.json`), summary: await read(`replay/${values['replay-prefix']}${i}-${mode}/summary.json`) }); } catch { /* missing */ }
    }
    const f = (s) => (s && s.n ? { p50: r2(s.p50), p95: r2(s.p95), p99: r2(s.p99), max: r2(s.max), n: s.n } : null);
    const row = (r, ph) => { const p = r.summary[ph]; return p && { label: r.label, loadAvgAtStart: r.meta.loadAvgAtStart, frames: p.frames, rafOver25: p.raf.over25, rafIntervals: p.raf.n, raf: f(p.raf), longtasks: p.longtasks.count,
      syncMs: f(p.syncMs), engineMs: f(p.engineMs), hitMs: f(p.hitMs), hitCalls: p.hitCalls, workerMs: f(p.workerMs), rttMs: f(p.rttMs), transitMs: f(p.transitMs), allocMs: f(p.allocMs), postMs: f(p.postMs),
      handlerMs: f(p.handlerMs), mirrorHitMs: f(p.mirrorHitMs), workerMainThreadMs: f(p.workerMainThreadMs) }; };
    out.replay[mode] = { runs: runs.length, split: runs.map((r) => r.summary.firstWpGroup), post: runs.map((r) => row(r, 'post')), pre: runs.map((r) => row(r, 'pre')),
      medianPostP99: Object.fromEntries(['syncMs', 'engineMs', 'hitMs', 'workerMs', 'rttMs', 'transitMs', 'workerMainThreadMs'].map((k) => [k, r2(med(runs.map((r) => r.summary.post?.[k]?.p99)))])) };
  }
  const sp = out.replay.sync?.medianPostP99, wk = out.replay.worker?.medianPostP99;
  if (sp && wk) out.desktopRuleValues = { b_workerMainThreadP99: wk.workerMainThreadMs, b_syncEngineP99: sp.engineMs, b_ratio: r2(wk.workerMainThreadMs / sp.engineMs), b_halfOfSyncP99: r2(0.5 * sp.engineMs),
    c_rttP99: wk.rttMs, c_limitMs_30fps_2frames: 66.7 };
}
await fs.writeFile(path.join(E, values['summary-name']), JSON.stringify(out, null, 1) + '\n', { flag: 'wx' });
console.log(JSON.stringify({ timingMedians: out.timingMedians, attribution: Object.fromEntries(Object.entries(out.attribution).map(([k, v]) => [k, { diff: v.medianDiffOver25, loadDiff: v.medianLoadDiff, pos: v.positiveRounds, neg: v.negativeRounds, verdict: v.verdict }])), desktopBudgets: out.desktopBudgets,
  replay: out.replay && Object.fromEntries(Object.entries(out.replay).map(([k, v]) => [k, v.medianPostP99])), desktopRuleValues: out.desktopRuleValues }, null, 1));

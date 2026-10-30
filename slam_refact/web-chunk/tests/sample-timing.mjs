// Timing summary of a sample run (preregistration us010-samples.v1.md section 3.1). Pure function over window.__xrSample / __input /
// __slamRefactChunk. Desktop headless SwiftShader numbers; not a phone's.
const q = (s, p) => s[Math.min(s.length - 1, Math.floor((s.length - 1) * p))];
export const stats = (xs) => {
  const s = xs.filter(Number.isFinite).sort((a, b) => a - b);
  if (!s.length) return { n: 0, min: null, p50: null, p95: null, p99: null, max: null, mean: null };
  return { n: s.length, min: s[0], p50: q(s, 0.5), p95: q(s, 0.95), p99: q(s, 0.99), max: s.at(-1), mean: s.reduce((a, b) => a + b, 0) / s.length };
};

function rafStats(rafTimes, from, to) {
  const t = rafTimes.filter((x) => x >= from && x <= to);
  const iv = t.slice(1).map((x, i) => x - t[i]);
  return { ...stats(iv), over25: iv.filter((x) => x > 25).length, over50: iv.filter((x) => x > 50).length };
}

export function timingSummary({ sample, input, chunkDiag }) {
  const updates = sample?.updates ?? [];
  if (!updates.length) return null;
  const fresh = updates.filter((u) => !u.repeat);
  const firstWp = sample.firstReal ?? null;  // v3: initialization = the first fresh response with a non-padding world point (v2 used any world point)
  const t0 = updates[0].wall, t1 = updates.at(-1).wall;
  const tReady = firstWp ? firstWp.wall : null;
  const longtasks = (a, b) => { const l = (sample.longtasks ?? []).filter((x) => x.start >= a && x.start <= b); return { count: l.length, totalMs: l.reduce((s, x) => s + x.duration, 0), maxMs: l.length ? Math.max(...l.map((x) => x.duration)) : 0 }; };
  const phase = (a, b, freshRows, procRange, stagedRange) => ({
    raf: rafStats(sample.rafTimes ?? [], a, b), longtasks: longtasks(a, b),
    xrGpuMs: stats(freshRows.map((u) => u.xrGpuMs)), xrCpuMs: stats(freshRows.map((u) => u.xrCpuMs)),
    freshUpdates: freshRows.length,
    stageMs: stats((chunkDiag?.stats?.stageMs ?? []).slice(...stagedRange)),
    readMs: stats((chunkDiag?.stats?.readMs ?? []).slice(...procRange)),
    engineMs: stats((chunkDiag?.stats?.engineMs ?? []).slice(...procRange)),
    mainThreadChunkMs: stats((chunkDiag?.stats?.engineMs ?? []).slice(...procRange).map((e, i) => e + (chunkDiag.stats.readMs[procRange[0] + i] ?? NaN)
      + (chunkDiag.stats.stageMs[procRange[0] + i] ?? NaN))),
  });
  const cp = firstWp?.chunkProcessed ?? null, cs = firstWp?.chunkStaged ?? null;
  const st = chunkDiag?.stats ?? {};
  const out = {
    window: { fromMs: t0, toMs: t1, seconds: (t1 - t0) / 1000 },
    all: phase(t0, t1, fresh, [0], [0]),
    pre: tReady === null ? null : phase(t0, tReady, fresh.filter((u) => u.wall < tReady), [0, cp], [0, cs]),
    post: tReady === null ? null : phase(tReady, t1, fresh.filter((u) => u.wall >= tReady), [cp], [cs]),
    firstWorldPoints: firstWp, firstAnyWorldPoint: sample.firstWp ?? null,
    readback: chunkDiag?.session?.readback ?? null,
    drops: { inputFramesPublished: input?.frames?.length ?? null, freshUpdates: fresh.length, repeatUpdates: updates.length - fresh.length,
      chunkStaged: st.stagedFrames, chunkProcessed: st.processedFrames, chunkFreshResponses: st.freshResponses, droppedStaged: st.droppedStaged,
      sizeMismatchFrames: st.sizeMismatchFrames, responseLag: st.responseLag, stageErrors: st.stageErrors, processErrors: st.processErrors },
  };
  // mainThreadChunkMs pairs stage[i], read[i], engine[i] by frame index (one entry per processed frame)
  return out;
}

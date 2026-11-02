// Pure metrics of the engine-Worker A/B harness (run-engine-worker-ab.mjs): main-thread busy time per frame, end-to-end pose latency,
// output identity between runs. Inputs are the page's own records (window.__xrSample.updates, window.__input.frames, window.__slamRefactChunk),
// so everything here is testable without a browser (engine-worker-ab-metrics.test.mjs).
import { createHash } from 'node:crypto';

const q = (s, p) => s[Math.min(s.length - 1, Math.floor((s.length - 1) * p))];
export const stats = (xs) => {
  const s = xs.filter(Number.isFinite).sort((a, b) => a - b);
  if (!s.length) return { n: 0, min: null, p50: null, p95: null, p99: null, max: null, mean: null };
  return { n: s.length, min: s[0], p50: q(s, 0.5), p95: q(s, 0.95), p99: q(s, 0.99), max: s.at(-1), mean: s.reduce((a, b) => a + b, 0) / s.length };
};
export const median = (xs) => { const s = xs.filter(Number.isFinite).sort((a, b) => a - b); return s.length ? (s[(s.length - 1) >> 1] + s[s.length >> 1]) / 2 : null; };

// Main-thread cost of the chunk per processed frame: stage (draw + readback start) + read (fence + readback) + engine. In Worker mode the
// chunk records the cost of submitting the frame as `engineMs`, so the same sum is the main-thread cost in both modes.
// Pairing is by index like slam_refact/web-chunk/tests/sample-timing.mjs; length mismatch (a dropped staged frame) trims to the shortest.
export function mainThreadChunkMs(chunkStats) {
  const { stageMs = [], readMs = [], engineMs = [] } = chunkStats ?? {};
  const n = Math.min(stageMs.length, readMs.length, engineMs.length);
  return Array.from({ length: n }, (_, i) => stageMs[i] + readMs[i] + engineMs[i]);
}

// Camera frame nearest to a video time in seconds (frames: window.__input.frames rows [index, timestampUs, publishedWall, writtenWall]).
export function frameMapper(frames) {
  const sorted = frames.map((f) => ({ index: f[0], ts: f[1], published: f[2] })).sort((a, b) => a.ts - b.ts);
  return {
    size: sorted.length,
    nearest(seconds) {
      const us = Math.round(seconds * 1e6);
      let lo = 0, hi = sorted.length - 1;
      while (lo < hi) { const mid = (lo + hi) >> 1; if (sorted[mid].ts < us) lo = mid + 1; else hi = mid; }
      const a = sorted[lo], b = sorted[lo - 1];
      return !b || Math.abs(a.ts - us) <= Math.abs(b.ts - us) ? a : b;
    },
  };
}
const freshByCallback = (responses) => {
  const byCallback = new Map();
  for (const r of responses) if (r.hasPose && r.fresh && r.sourceMetadataFound && Number.isFinite(r.callbackVideoTimeS)) byCallback.set(r.callbackVideoTimeS, r);
  return byCallback;
};

// End-to-end pose latency as the app sees it: wall time of the pipeline update that carries a fresh pose minus the wall time the camera frame
// that pose was computed from was handed to the page. Update <-> response join: the callback's videoTime. Source frame <-> camera frame: the
// response's sourceVideoTimeS against each frame's timestamp (microseconds, seconds = us / 1e6), nearest wins; the residual of that match is
// reported, and a median residual above 1 ms flags the unit assumption as wrong (`mappingSuspect`).
export function poseLatency({ updates, frames, responses }) {
  const byCallback = freshByCallback(responses);
  const map = frameMapper(frames);
  const latencies = [], ageFrames = [], residualsUs = [], rows = [];
  for (const u of updates) {
    if (u.repeat) continue;
    const r = byCallback.get(u.videoTime);
    if (!r || !map.size) continue;
    const src = map.nearest(r.sourceVideoTimeS), cur = map.nearest(r.callbackVideoTimeS);
    residualsUs.push(Math.abs(src.ts - r.sourceVideoTimeS * 1e6));
    latencies.push(u.wall - src.published);
    ageFrames.push(cur.index - src.index);
    rows.push({ n: u.n, sourceIndex: src.index, callbackIndex: cur.index, latencyMs: u.wall - src.published });
  }
  const residual = median(residualsUs);
  return { latencyMs: stats(latencies), ageFrames: stats(ageFrames), matched: latencies.length, mapping: { medianResidualUs: residual, mappingSuspect: residual === null || residual > 1000 }, rows };
}

// The engine output of every distinct source frame of a run, keyed by the camera frame (fixture index) it was computed from: what two runs can be compared on.
export function outputsBySource({ updates, frames, responses }) {
  const byCallback = freshByCallback(responses);
  const map = frameMapper(frames);
  const out = new Map();
  for (const u of updates) {
    if (u.repeat) continue;
    const r = byCallback.get(u.videoTime);
    if (!r || !map.size) continue;
    const key = map.nearest(r.sourceVideoTimeS).index;
    if (out.has(key)) continue;
    out.set(key, { pos: u.pos, rot: u.rot, status: u.status, reason: u.reason, nwp: u.nwp });
  }
  return out;
}

export const digest = (outputs) => createHash('sha256').update(JSON.stringify([...outputs.entries()].sort((a, b) => a[0] - b[0]))).digest('hex');

// Run X against run Y on the source frames both produced. Live runs are scheduled by the browser, so which IMU events reach which frame is
// not fixed: main-vs-main is the noise floor that main-vs-Worker must be read against.
export function compareOutputs(x, y) {
  let common = 0, identical = 0, maxPos = 0, maxRot = 0;
  for (const [key, a] of x) {
    const b = y.get(key);
    if (!b) continue;
    common++;
    const same = a.pos.every((v, i) => v === b.pos[i]) && a.rot.every((v, i) => v === b.rot[i]) && a.status === b.status && a.reason === b.reason && a.nwp === b.nwp;
    if (same) identical++;
    for (let i = 0; i < 3; i++) maxPos = Math.max(maxPos, Math.abs(a.pos[i] - b.pos[i]));
    for (let i = 0; i < 4; i++) maxRot = Math.max(maxRot, Math.abs(a.rot[i] - b.rot[i]));
  }
  return { sourceFramesX: x.size, sourceFramesY: y.size, common, identical, identicalFraction: common ? identical / common : null, maxPosAbsDiff: maxPos, maxRotComponentAbsDiff: maxRot };
}

// One run's numbers. all arguments are the page's records.
export function summarizeRun({ updates, frames, responses, chunkStats, longtasks = [], rafTimes = [], engineWorker = null }) {
  const fresh = updates.filter((u) => !u.repeat);
  const t0 = updates[0]?.wall ?? 0, t1 = updates.at(-1)?.wall ?? 0;
  const raf = rafTimes.filter((t) => t >= t0 && t <= t1);
  const iv = raf.slice(1).map((t, i) => t - raf[i]);
  const lt = longtasks.filter((l) => l.start >= t0 && l.start <= t1);
  const firstWorld = fresh.find((u) => u.nreal >= 1);
  const latency = poseLatency({ updates, frames, responses });
  return {
    freshUpdates: fresh.length, repeatUpdates: updates.length - fresh.length, inputFrames: frames.length,
    mainThreadChunkMs: stats(mainThreadChunkMs(chunkStats)),
    stageMs: stats(chunkStats?.stageMs ?? []), readMs: stats(chunkStats?.readMs ?? []), engineSubmitOrCallMs: stats(chunkStats?.engineMs ?? []),
    xrCpuMs: stats(fresh.map((u) => u.xrCpuMs)), xrGpuMs: stats(fresh.map((u) => u.xrGpuMs)),
    poseLatencyMs: latency.latencyMs, poseAgeFrames: latency.ageFrames, latencyMatched: latency.matched, latencyMapping: latency.mapping,
    raf: { ...stats(iv), over25: iv.filter((x) => x > 25).length, over50: iv.filter((x) => x > 50).length },
    longtasks: { count: lt.length, totalMs: lt.reduce((s, l) => s + l.duration, 0), maxMs: lt.length ? Math.max(...lt.map((l) => l.duration)) : 0 },
    timeToFirstWorldPointMs: firstWorld ? firstWorld.wall - t0 : null,
    counts: { staged: chunkStats?.stagedFrames ?? null, processed: chunkStats?.processedFrames ?? null, freshResponses: chunkStats?.freshResponses ?? null,
      droppedStaged: chunkStats?.droppedStaged ?? null, processErrors: chunkStats?.processErrors ?? null, stageErrors: chunkStats?.stageErrors ?? null,
      sizeMismatchFrames: chunkStats?.sizeMismatchFrames ?? null, responseLag: chunkStats?.responseLag ?? null },
    engineWorker: engineWorker && { active: engineWorker.active, fallback: engineWorker.fallback, framesSubmitted: engineWorker.stats?.framesSubmitted ?? null,
      resultsReceived: engineWorker.stats?.resultsReceived ?? null, backpressureSkips: engineWorker.stats?.backpressureSkips ?? null,
      hitTestMisses: engineWorker.stats?.hitTestMisses ?? null, opErrors: engineWorker.stats?.opErrors ?? null,
      buffersAllocated: engineWorker.stats?.buffersAllocated ?? null, buffersReused: engineWorker.stats?.buffersReused ?? null,
      workerEngineMs: stats(engineWorker.stats?.workerEngineMs ?? []), rttMs: stats(engineWorker.stats?.rttMs ?? []), transitMs: stats(engineWorker.stats?.transitMs ?? []) },
  };
}

// ABAB x3: per-arm medians over the rounds of each statistic, plus the identity matrix. runs: [{arm: 'main'|'worker', summary, outputs}] in run order.
export function aggregate(runs) {
  const arm = (name) => runs.filter((r) => r.arm === name);
  const pick = (name, f) => median(arm(name).map((r) => f(r.summary)));
  const metric = (name) => ({
    mainThreadChunkMsP50: pick(name, (s) => s.mainThreadChunkMs.p50), mainThreadChunkMsP95: pick(name, (s) => s.mainThreadChunkMs.p95), mainThreadChunkMsP99: pick(name, (s) => s.mainThreadChunkMs.p99),
    xrCpuMsP50: pick(name, (s) => s.xrCpuMs.p50), xrCpuMsP95: pick(name, (s) => s.xrCpuMs.p95),
    poseLatencyMsP50: pick(name, (s) => s.poseLatencyMs.p50), poseLatencyMsP95: pick(name, (s) => s.poseLatencyMs.p95), poseLatencyMsP99: pick(name, (s) => s.poseLatencyMs.p99),
    poseAgeFramesP50: pick(name, (s) => s.poseAgeFrames.p50), rafOver25: pick(name, (s) => s.raf.over25), longtaskTotalMs: pick(name, (s) => s.longtasks.totalMs),
    processed: pick(name, (s) => s.counts.processed), droppedStaged: pick(name, (s) => s.counts.droppedStaged), runs: arm(name).length,
  });
  const mains = arm('main'), workers = arm('worker');
  const identity = { mainVsMain: [], workerVsMain: [] };
  for (let i = 1; i < mains.length; i++) identity.mainVsMain.push({ a: 0, b: i, ...compareOutputs(mains[0].outputs, mains[i].outputs) });
  for (let i = 0; i < Math.min(mains.length, workers.length); i++) identity.workerVsMain.push({ round: i, ...compareOutputs(workers[i].outputs, mains[i].outputs) });
  return { main: metric('main'), worker: metric('worker'), identity, order: runs.map((r) => r.arm) };
}

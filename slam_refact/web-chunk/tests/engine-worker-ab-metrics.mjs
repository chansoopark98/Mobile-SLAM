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

// ---------------------------------------------------------------------------------------------------------------------------------------
// R1 preregistered metrics (preregistration section R1). The older metrics above stay in the report as report-only.
//
// Frame accounting (preregistration v4.5). Frames are numbered 1, 2, 3, ... in recording order (seq = fixture index + 1). The chunk (option
// frameDoneMarks, both arms) emits performance.mark('sr-frame-start:<seq>') at the first main-thread work of frame seq (the entry of its frame
// callback) and performance.mark('sr-frame-done:<seq>') once ALL main-thread handling of the engine response to frame seq has returned (what
// processing frame seq returned: every processed frame has one, the last frame included; the Worker mode applies it one callback later).
// The input is exactly N = 30 + 2m frames (m >= 120) and then ends. PRIMARY window = [trace start, sr-frame-done:N], metric = busy / N.
// The v4.4 middle window [sr-frame-start:30, sr-frame-done:N] / (2m+1) is report-only; measured segment (rules (e)(f)) = frames 31 .. 30+m.
// ---------------------------------------------------------------------------------------------------------------------------------------

// numpy.percentile(xs, 100 * p) with the default 'linear' interpolation (the definition G7 uses).
export function percentileLinear(xs, p) {
  const s = xs.filter(Number.isFinite).sort((a, b) => a - b);
  if (!s.length) return null;
  const h = (s.length - 1) * p, lo = Math.floor(h), hi = Math.ceil(h);
  return s[lo] + (s[hi] - s[lo]) * (h - lo);
}

export const measuredFrameCount = (m) => 2 * m + 1;  // frames in the window (the denominator of the primary metric)
export const FIRST_SEQ = 30;

// PRIMARY METRIC. Sum of the durations of ALL top-level tasks (`RunTask`) of the renderer main thread (CrRendererMain) between the two marks,
// clipped to the window, nested/overlapping tasks counted once (union), idle time not counted. The marks are user-timing events of the page's main
// thread, so they also identify the thread. traceEvents: the Chrome trace.
export function traceStartUs(traceEvents) {
  const e = traceEvents.find((x) => x.name === 'TracingStartedInBrowser');
  if (e && Number.isFinite(e.ts)) return e.ts;
  const ts = traceEvents.filter((x) => x.ph !== 'M' && Number.isFinite(x.ts)).map((x) => x.ts);
  return ts.length ? Math.min(...ts) : null;
}

// startMark: a mark name, or null with fromTraceStart: true = the window starts when the trace starts (v4.5: the trace is started before the session start).
export function topLevelBusy(traceEvents, { startMark = null, endMark, fromTraceStart = false }) {
  const find = (name) => traceEvents.find((e) => e.name === name && e.ph !== 'M');
  const end = find(endMark);
  const start = fromTraceStart ? (end ? { ts: traceStartUs(traceEvents), pid: end.pid, tid: end.tid } : null) : find(startMark);
  if (!start || !end) return { error: `trace lacks the window mark ${!end ? endMark : startMark}` };
  if (start.ts === null || start.pid !== end.pid || start.tid !== end.tid) return { error: 'the window marks are on different threads' };
  const { pid, tid } = start;
  const threadName = traceEvents.find((e) => e.ph === 'M' && e.name === 'thread_name' && e.pid === pid && e.tid === tid)?.args?.name ?? null;
  const t0 = start.ts, t1 = end.ts;
  if (!(t1 > t0)) return { error: 'the window is empty', threadName };
  const onThread = traceEvents.filter((e) => e.pid === pid && e.tid === tid);
  const intervalsOf = (name) => {
    const out = [], stack = [];
    for (const e of onThread) {
      if (e.name !== name) continue;
      if (e.ph === 'X' && Number.isFinite(e.dur)) out.push([e.ts, e.ts + e.dur]);
      else if (e.ph === 'B') stack.push(e.ts);
      else if (e.ph === 'E' && stack.length) out.push([stack.pop(), e.ts]);
    }
    return out;
  };
  let source = 'RunTask', tasks = intervalsOf('RunTask');
  if (!tasks.length) { source = 'ThreadControllerImpl::RunTask'; tasks = intervalsOf(source); }
  if (!tasks.length) return { error: 'no top-level task events (RunTask) on the page main thread', threadName };
  tasks.sort((a, b) => a[0] - b[0]);
  const union = [];
  for (const [a, b] of tasks) {
    const last = union.at(-1);
    if (last && a <= last[1]) last[1] = Math.max(last[1], b); else union.push([a, b]);
  }
  let busyUs = 0, count = 0, clipped = 0, largestUs = 0;
  for (const [a, b] of union) {
    const lo = Math.max(a, t0), hi = Math.min(b, t1);
    if (hi <= lo) continue;
    if (lo !== a || hi !== b) clipped++;
    busyUs += hi - lo; count++; largestUs = Math.max(largestUs, hi - lo);
  }
  let afterUs = 0;
  for (const [a, b] of union) { const lo = Math.max(a, t1), hi = Math.min(b, t1 + 3_000_000); if (hi > lo) afterUs += hi - lo; }
  const traceEnd = Math.max(...onThread.map((e) => e.ts + (e.dur || 0)));
  return { windowStart: fromTraceStart ? 'trace-start' : startMark, windowEnd: endMark, busyMs: busyUs / 1000, windowMs: (t1 - t0) / 1000, afterWindow3000Ms: afterUs / 1000, afterWindowCovered: traceEnd >= t1 + 3_000_000, taskCount: count, boundaryClippedTasks: clipped, largestTaskMs: largestUs / 1000, source, threadName,
    mainThreadVerified: threadName === 'CrRendererMain' };
}

// v4.5 PRIMARY WINDOW. Input = exactly N = 30 + 2m frames (m >= 120), then feeding stops: no mark, work or render may exist for a frame beyond N.
// Window = [trace start, sr-frame-done:N]; the trace is started before the page (and with it XR8.run / the session start) exists. Every frame
// 1..N must have exactly one start and one done mark, both strictly increasing in k, start_k < done_k, and exactly one render callback (the update that
// consumes the engine response to frame k) inside [start_k, done_k]. Any violation = R1 FAIL by the registered rules.
export function fullWindow({ applied, marks, renders, captures, totalFrames, n }) {
  const out = { n, totalFrames, windowFrames: n, firstSeq: 1, lastSeq: n, startMark: null, endMark: `sr-frame-done:${n}`, errors: [] };
  if (totalFrames < n) { out.errors.push(`input too short: ${totalFrames} frames, ${n} required`); out.valid = false; return out; }
  const startAt = new Map(), doneAt = new Map(), startCount = new Map(), doneCount = new Map();
  for (const [name, t] of marks) {
    const sm = /^sr-frame-start:(\d+)$/.exec(name), dm = /^sr-frame-done:(\d+)$/.exec(name);
    if (sm) { const k = Number(sm[1]); startCount.set(k, (startCount.get(k) ?? 0) + 1); if (!startAt.has(k)) startAt.set(k, t); }
    if (dm) { const k = Number(dm[1]); doneCount.set(k, (doneCount.get(k) ?? 0) + 1); if (!doneAt.has(k)) doneAt.set(k, t); }
  }
  const renderCount = new Map(), renderAt = new Map();
  for (const row of applied) { renderCount.set(row[0] + 1, (renderCount.get(row[0] + 1) ?? 0) + 1); if (!renderAt.has(row[0] + 1)) renderAt.set(row[0] + 1, row[1]); }
  const appliedFirst = new Map();
  for (const row of applied) if (!appliedFirst.has(row[0] + 1)) appliedFirst.set(row[0] + 1, row);
  const renderExits = renders.map((r) => r[0]).sort((a, b) => a - b);
  const v = { missingStart: [], missingDone: [], duplicateStart: [], duplicateDone: [], startNotIncreasing: [], doneNotIncreasing: [], startNotBeforeDone: [],
    renderCallbackNotOnce: [], renderCallbackNotInside: [], markBeforeRenderReturned: [], framesBeyondN: [] };
  let maxDelayAfterRender = 0;
  const beyond = new Set();
  for (const [name] of marks) { const mm = /^sr-frame-(?:start|done):(\d+)$/.exec(name); if (mm && Number(mm[1]) > n && Number(mm[1]) < 1_000_000) beyond.add(Number(mm[1])); }
  for (const row of applied) if (row[0] + 1 > n) beyond.add(row[0] + 1);
  v.framesBeyondN = [...beyond].sort((a, b) => a - b);
  for (let k = 1; k <= n; k++) {
    const s0 = startAt.get(k), d0 = doneAt.get(k);
    if (s0 === undefined) v.missingStart.push(k);
    if (d0 === undefined) v.missingDone.push(k);
    if ((startCount.get(k) ?? 0) > 1) v.duplicateStart.push(k);
    if ((doneCount.get(k) ?? 0) > 1) v.duplicateDone.push(k);
    if (k > 1) {
      const ps = startAt.get(k - 1), pd = doneAt.get(k - 1);
      if (s0 !== undefined && ps !== undefined && !(s0 > ps)) v.startNotIncreasing.push(k);
      if (d0 !== undefined && pd !== undefined && !(d0 > pd)) v.doneNotIncreasing.push(k);
    }
    if (s0 !== undefined && d0 !== undefined && !(s0 < d0)) v.startNotBeforeDone.push(k);
    if ((renderCount.get(k) ?? 0) !== 1) v.renderCallbackNotOnce.push(k);
    else if (!(s0 !== undefined && d0 !== undefined && s0 < renderAt.get(k) && renderAt.get(k) < d0)) v.renderCallbackNotInside.push(k);
    const row = appliedFirst.get(k);
    if (row && d0 !== undefined) {
      const r = renderExits.find((x) => x >= row[1]);
      if (r !== undefined) { if (d0 < r) v.markBeforeRenderReturned.push(k); maxDelayAfterRender = Math.max(maxDelayAfterRender, d0 - r); }
    }
  }
  Object.assign(out, v, { maxMarkDelayAfterRenderMs: maxDelayAfterRender });
  const captureAt = new Map();
  captures.forEach(([index, t]) => { if (index >= 0 && !captureAt.has(index + 1)) captureAt.set(index + 1, t); });
  const all = [], measured = [];
  const m = (n - 30) / 2;
  for (let k = 1; k <= n; k++) {
    if (!captureAt.has(k) || doneAt.get(k) === undefined) continue;
    const lat = doneAt.get(k) - captureAt.get(k);
    all.push(lat);
    if (k >= 31 && k <= 30 + m) measured.push(lat);
  }
  out.latencyMs = { n: measured.length, p50: percentileLinear(measured, 0.5), p95: percentileLinear(measured, 0.95), max: measured.length ? Math.max(...measured) : null };
  out.latencyAllFramesMs = { n: all.length, p50: percentileLinear(all, 0.5), p95: percentileLinear(all, 0.95), max: all.length ? Math.max(...all) : null };
  out.violations = Object.fromEntries(Object.entries(v).map(([k, x]) => [k, x.length]));
  out.valid = out.errors.length === 0 && Object.values(v).every((x) => x.length === 0);
  return out;
}

// Window, per-frame start / done times and every check that makes a run valid (any violation = R1 FAIL by the registered rules; an input that is
// too short is R1-INFRA). Inputs are the page's own records:
//   captures        [[fixtureIndex or -1, captureEntryT]]  one row per non-repeat frame callback, in order
//   marks           [[name, startTime]]                    performance.getEntriesByType('mark') (page clock)
//   renderCallbacks [[fixtureIndex, t]]                    the update callbacks of non-repeat frames (report only)
//   applied         [[fixtureIndex, updateEntryT, callbackIndex, repeat, sourceSeq]]  the updates that consume a fresh pose (the render callback of that frame)
//   renders         [[exitT]]                              exits of the last module's onRender
export function frameWindow({ applied, marks, renders, captures, renderCallbacks, totalFrames, m }) {
  const first = FIRST_SEQ, last = FIRST_SEQ + 2 * m;           // seq (1-based) of the window
  const out = { totalFrames, m, windowFrames: measuredFrameCount(m), measuredFrames: measuredFrameCount(m), firstSeq: first, lastSeq: last,
    measuredSegment: [first + 1, first + m], trailingSegment: [first + m + 1, last], errors: [] };
  if (totalFrames < last) { out.errors.push(`input too short: ${totalFrames} frames, ${last} required`); out.valid = false; return out; }
  const startAt = new Map(), doneAt = new Map(), startCount = new Map(), doneCount = new Map();
  for (const [name, t] of marks) {
    const sm = /^sr-frame-start:(\d+)$/.exec(name), dm = /^sr-frame-done:(\d+)$/.exec(name);
    if (sm) { const k = Number(sm[1]); startCount.set(k, (startCount.get(k) ?? 0) + 1); if (!startAt.has(k)) startAt.set(k, t); }
    if (dm) { const k = Number(dm[1]); doneCount.set(k, (doneCount.get(k) ?? 0) + 1); if (!doneAt.has(k)) doneAt.set(k, t); }
  }
  out.startMark = `sr-frame-start:${first}`; out.endMark = `sr-frame-done:${last}`;
  const appliedFirst = new Map();
  for (const row of applied) if (!appliedFirst.has(row[0] + 1)) appliedFirst.set(row[0] + 1, row);
  const renderExits = renders.map((r) => r[0]).sort((a, b) => a - b);
  // the render callback of frame seq = the update that consumes the pose of frame seq (r1_eval: "render-callback executions that consumed the pose of frame seq")
  const renderCount = new Map(), renderAt = new Map();
  for (const row of applied) { renderCount.set(row[0] + 1, (renderCount.get(row[0] + 1) ?? 0) + 1); if (!renderAt.has(row[0] + 1)) renderAt.set(row[0] + 1, row[1]); }
  const v = { missingStart: [], missingDone: [], duplicateStart: [], duplicateDone: [], startNotIncreasing: [], doneNotIncreasing: [], startNotBeforeDone: [],
    measuredDoneAfterWindowEnd: [], renderCallbackNotOnce: [], renderCallbackNotInside: [], markBeforeRenderReturned: [] };
  let maxDelayAfterRender = 0;
  const windowEnd = doneAt.get(last);
  for (let k = first; k <= last; k++) {
    const s0 = startAt.get(k), d0 = doneAt.get(k);
    if (s0 === undefined) v.missingStart.push(k);
    if (d0 === undefined) v.missingDone.push(k);
    if ((startCount.get(k) ?? 0) > 1) v.duplicateStart.push(k);
    if ((doneCount.get(k) ?? 0) > 1) v.duplicateDone.push(k);
    if (k > first) {
      const ps = startAt.get(k - 1), pd = doneAt.get(k - 1);
      if (s0 !== undefined && ps !== undefined && !(s0 > ps)) v.startNotIncreasing.push(k);
      if (d0 !== undefined && pd !== undefined && !(d0 > pd)) v.doneNotIncreasing.push(k);
    }
    if (s0 !== undefined && d0 !== undefined && !(s0 < d0)) v.startNotBeforeDone.push(k);
    const inMeasured = k >= first + 1 && k <= first + m;
    if (inMeasured) {
      // (f): every measured frame is done before the window ends
      if (d0 === undefined || windowEnd === undefined || !(d0 < windowEnd)) v.measuredDoneAfterWindowEnd.push(k);
      // (e): exactly one render callback, and it runs inside [start_k, done_k]
      if ((renderCount.get(k) ?? 0) !== 1) v.renderCallbackNotOnce.push(k);
      else if (!(s0 !== undefined && d0 !== undefined && s0 < renderAt.get(k) && renderAt.get(k) < d0)) v.renderCallbackNotInside.push(k);
    }
    // the done mark must come after the render callbacks of the loop callback that applied the result
    const row = appliedFirst.get(k);
    if (row && d0 !== undefined) {
      const r = renderExits.find((x) => x >= row[1]);
      if (r !== undefined) { if (d0 < r) v.markBeforeRenderReturned.push(k); maxDelayAfterRender = Math.max(maxDelayAfterRender, d0 - r); }
    }
  }
  Object.assign(out, v, { maxMarkDelayAfterRenderMs: maxDelayAfterRender });
  out.windowMsByMarks = startAt.get(first) !== undefined && windowEnd !== undefined ? windowEnd - startAt.get(first) : null;
  // latency: capture callback entry -> done mark of that frame's result, measured segment only (frames 31 .. 30+m)
  const captureAt = new Map();
  captures.forEach(([index, t]) => { if (index >= 0 && !captureAt.has(index + 1)) captureAt.set(index + 1, t); });
  const values = [];
  for (let k = first + 1; k <= first + m; k++) if (captureAt.has(k) && doneAt.get(k) !== undefined) values.push(doneAt.get(k) - captureAt.get(k));
  out.latencyMs = { n: values.length, p50: percentileLinear(values, 0.5), p95: percentileLinear(values, 0.95), max: values.length ? Math.max(...values) : null };
  out.violations = Object.fromEntries(Object.entries(v).map(([k, x]) => [k, x.length]));
  out.valid = out.errors.length === 0 && Object.values(v).every((x) => x.length === 0);
  return out;
}

// The preregistered numbers of one live run (v4.5): PRIMARY = main-thread busy ms from the trace start to sr-frame-done:N, / N. The v4.4 middle window
// (frames 30 .. 30+2m, / (2m+1)) is computed from the same trace and is REPORT-ONLY (`middle`).
export function primaryOfRun({ trace, window: w, middle = null }) {
  const busy = w.endMark ? topLevelBusy(trace, { endMark: w.endMark, fromTraceStart: true }) : { error: 'no window end mark' };
  const mid = middle && middle.startMark && middle.endMark ? topLevelBusy(trace, { startMark: middle.startMark, endMark: middle.endMark }) : null;
  return { busy, busyMsPerFrame: busy.error ? null : busy.busyMs / w.windowFrames, measuredFrames: w.windowFrames,
    e2eLatencyP95Ms: w.latencyMs?.p95 ?? null, e2eLatencyP50Ms: w.latencyMs?.p50 ?? null, latencyFrames: w.latencyMs?.n ?? 0,
    violations: w.violations ?? null, windowErrors: w.errors,
    valid: !busy.error && busy.mainThreadVerified === true && w.valid === true,
    middle: middle && { busyMsPerFrame: mid && !mid.error ? mid.busyMs / middle.windowFrames : null, busy: mid, valid: middle.valid, violations: middle.violations, reportOnly: true } };
}

// ABAB x3 on the preregistered numbers: per-arm medians and the B/A ratios. No verdict: the gate is evaluated against the registered thresholds elsewhere.
export function aggregatePrimary(runs) {
  const arm = (name) => runs.filter((r) => r.arm === name);
  const med = (name, f) => median(arm(name).map((r) => f(r.primary)));
  const side = (name) => ({ runs: arm(name).length, allValid: arm(name).every((r) => r.primary.valid), busyMsPerFrame: med(name, (p) => p.busyMsPerFrame),
    e2eLatencyP95Ms: med(name, (p) => p.e2eLatencyP95Ms), measuredFrames: arm(name).map((r) => r.primary.measuredFrames) });
  const a = side('main'), b = side('worker');
  const framesEqual = [...a.measuredFrames, ...b.measuredFrames].every((n) => n === a.measuredFrames[0]);
  return { main: a, worker: b, framesEqualAcrossRuns: framesEqual,
    ratios: { busyWorkerOverMain: a.busyMsPerFrame ? b.busyMsPerFrame / a.busyMsPerFrame : null, latencyP95WorkerOverMain: a.e2eLatencyP95Ms ? b.e2eLatencyP95Ms / a.e2eLatencyP95Ms : null } };
}

// Deterministic replay identity: every run's answers (keyed by the engine source frame) must be the same bytes. runs: [{arm, digest, count}].
export function replayIdentity(runs) {
  const digests = [...new Set(runs.map((r) => r.digest))];
  const counts = [...new Set(runs.map((r) => r.count))];
  return { runs: runs.length, distinctDigests: digests.length, allIdentical: runs.length > 0 && digests.length === 1 && counts.length === 1 && counts[0] > 0,
    count: counts.length === 1 ? counts[0] : counts };
}

// The run record scorer/r1_eval.py reads (preregistration v4.4): {start: {seq: [t]}, done: {seq: [t]}, render: {seq: [t]}, work: {seq: [[b, e]]}, tasks: [[b, e]]},
// every time in ms on ONE clock. That clock is the trace's: the marks and the top-level tasks come from the trace; the page-clock times (the updates that
// consume a pose = the render callbacks) are moved onto it with the median offset between the same marks in both clocks.
// `work` (instrumented callback intervals per frame: postMessage send, receive handler, scheduled post-processing, ...) is NOT recorded by this harness: it
// is {} and r1_eval's work-placement rule is vacuous for these runs (the harness checks the placement it can see itself: render callbacks inside
// [start_k, done_k], done after the render callbacks that applied the result).
export function scorerRun(traceEvents, { pageMarks, applied }) {
  const first = traceEvents.find((e) => /^sr-frame-start:\d+$/.test(e.name) && e.ph !== 'M' && Number(e.name.split(':')[1]) < 1_000_000);
  if (!first) return { error: 'trace has no sr-frame-start mark of a recorded frame' };
  const { pid, tid } = first;
  const onThread = traceEvents.filter((e) => e.pid === pid && e.tid === tid);
  const start = {}, done = {};
  for (const e of onThread) {
    const sm = /^sr-frame-start:(\d+)$/.exec(e.name), dm = /^sr-frame-done:(\d+)$/.exec(e.name);
    if (sm && e.ph !== 'M' && Number(sm[1]) < 1_000_000) (start[sm[1]] ??= []).push(e.ts / 1000);
    if (dm && e.ph !== 'M' && Number(dm[1]) < 1_000_000) (done[dm[1]] ??= []).push(e.ts / 1000);
  }
  const offsets = [];
  const traceFirst = new Map();
  for (const e of onThread) if (e.ph !== 'M' && /^sr-frame-(start|done):/.test(e.name) && !traceFirst.has(e.name)) traceFirst.set(e.name, e.ts / 1000);
  const seenPage = new Set();
  for (const [name, t] of pageMarks) if (traceFirst.has(name) && !seenPage.has(name)) { seenPage.add(name); offsets.push(traceFirst.get(name) - t); }
  const offsetMs = median(offsets);
  if (offsetMs === null) return { error: 'no mark is in both clocks' };
  const raw = [];
  const open = [];
  for (const e of onThread) {
    if (e.name !== 'RunTask') continue;
    if (e.ph === 'X' && Number.isFinite(e.dur)) raw.push([e.ts / 1000, (e.ts + e.dur) / 1000]);
    else if (e.ph === 'B') open.push(e.ts); else if (e.ph === 'E' && open.length) raw.push([open.pop() / 1000, e.ts / 1000]);
  }
  raw.sort((a, b) => a[0] - b[0]);
  const tasks = [];
  for (const [b, e] of raw) { const last = tasks.at(-1); if (last && b <= last[1]) last[1] = Math.max(last[1], e); else tasks.push([b, e]); }
  const render = {};
  for (const row of applied) (render[row[0] + 1] ??= []).push(row[1] + offsetMs);
  return { clock: 'chrome trace, ms', trace_start: traceStartUs(traceEvents) / 1000, offsetMs, offsetSpreadMs: Math.max(...offsets) - Math.min(...offsets), start, done, render, work: {}, tasks };
}

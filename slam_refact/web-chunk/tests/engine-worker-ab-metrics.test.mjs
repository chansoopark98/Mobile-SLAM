// node --test slam_refact/web-chunk/tests/engine-worker-ab-metrics.test.mjs
// The arithmetic of the engine-Worker A/B harness on synthetic page records (no browser): main-thread cost per frame, end-to-end pose latency
// with the update <-> response <-> camera frame join, identity between runs, and the ABAB aggregation.
import assert from 'node:assert/strict';
import test from 'node:test';
import { aggregate, compareOutputs, digest, mainThreadChunkMs, median, outputsBySource, poseLatency, stats, summarizeRun } from './engine-worker-ab-metrics.mjs';

// 5 camera frames 33 ms apart, handed to the page at wall 1000 + 33 i; timestamps in microseconds from an arbitrary epoch.
const frames = Array.from({ length: 5 }, (_, i) => [i, 7_000_000 + i * 33_000, 1000 + 33 * i, 1001 + 33 * i]);
const sec = (i) => (7_000_000 + i * 33_000) / 1e6;
// update k carries the pose of source frame k-1 (lag of one) and arrives 80 ms after that frame was handed over
const updates = [1, 2, 3, 4].map((k) => ({ n: k, repeat: 0, videoTime: sec(k), wall: 1000 + 33 * (k - 1) + 80, pos: [k, 0, 0], rot: [1, 0, 0, 0], status: 'NORMAL', reason: 'UNSPECIFIED', nwp: 3, nreal: 2, xrCpuMs: 1, xrGpuMs: 1 }));
const responses = [1, 2, 3, 4].map((k) => ({ hasPose: true, fresh: true, sourceMetadataFound: true, callbackVideoTimeS: sec(k), sourceVideoTimeS: sec(k - 1) }));

test('R1-M1: stats and median ignore non-finite values and handle empty input', () => {
  assert.deepEqual(stats([]), { n: 0, min: null, p50: null, p95: null, p99: null, max: null, mean: null });
  assert.equal(stats([3, 1, NaN, 2]).n, 3);
  assert.equal(stats([1, 2, 3, 4, 5]).p50, 3);
  assert.equal(median([1, 2, 3, 10]), 2.5);
  assert.equal(median([]), null);
});

test('R1-M2: main-thread chunk cost is stage + read + engine call (or submit) per processed frame, trimmed to the shortest series', () => {
  assert.deepEqual(mainThreadChunkMs({ stageMs: [1, 2, 3], readMs: [10, 20], engineMs: [100, 200, 300] }), [111, 222]);
  assert.deepEqual(mainThreadChunkMs(null), []);
});

test('R1-M3: pose latency joins update -> response -> camera frame and reports the match residual', () => {
  const r = poseLatency({ updates, frames, responses });
  assert.equal(r.matched, 4);
  assert.deepEqual(r.latencyMs.p50, 80);
  assert.deepEqual(r.ageFrames.p50, 1);
  assert.equal(r.mapping.medianResidualUs, 0);
  assert.equal(r.mapping.mappingSuspect, false);
  assert.deepEqual(r.rows.map((x) => x.sourceIndex), [0, 1, 2, 3]);
});

test('R1-M4: a wrong unit assumption (video time not in the camera timestamp domain) is flagged, not silently used', () => {
  const skewed = responses.map((x) => ({ ...x, sourceVideoTimeS: x.sourceVideoTimeS + 0.0042 }));
  const r = poseLatency({ updates, frames, responses: skewed });
  assert.equal(r.mapping.mappingSuspect, true);
  assert.equal(poseLatency({ updates, frames: [], responses }).mapping.mappingSuspect, true);
});

test('R1-M5: repeated updates, unfresh responses and updates without a response are not counted', () => {
  const noisy = [{ ...updates[0], repeat: 1 }, { ...updates[1], videoTime: 999 }, updates[2]];
  const r = poseLatency({ updates: noisy, frames, responses: responses.map((x, i) => (i === 2 ? { ...x, fresh: false } : x)) });
  assert.equal(r.matched, 0);
});

test('R1-M6: outputs are keyed by the camera frame, so two runs with different clocks compare; identical runs are identical, a drift is measured', () => {
  const shift = 5_000_000;  // the other run's clock origin differs
  const frames2 = frames.map((f) => [f[0], f[1] + shift, f[2] + 17, f[3] + 17]);
  const updates2 = updates.map((u) => ({ ...u, videoTime: u.videoTime + shift / 1e6, wall: u.wall + 17 }));
  const responses2 = responses.map((x) => ({ ...x, callbackVideoTimeS: x.callbackVideoTimeS + shift / 1e6, sourceVideoTimeS: x.sourceVideoTimeS + shift / 1e6 }));
  const a = outputsBySource({ updates, frames, responses });
  const b = outputsBySource({ updates: updates2, frames: frames2, responses: responses2 });
  assert.deepEqual([...a.keys()], [0, 1, 2, 3]);
  assert.equal(digest(a), digest(b));
  assert.deepEqual(compareOutputs(a, b), { sourceFramesX: 4, sourceFramesY: 4, common: 4, identical: 4, identicalFraction: 1, maxPosAbsDiff: 0, maxRotComponentAbsDiff: 0 });
  const c = new Map([...b].map(([k, v]) => [k, k === 2 ? { ...v, pos: [v.pos[0] + 1e-6, 0, 0] } : v]));
  const cmp = compareOutputs(a, c);
  assert.equal(cmp.identical, 3);
  assert.ok(cmp.maxPosAbsDiff > 0 && cmp.maxPosAbsDiff < 2e-6);
  assert.notEqual(digest(a), digest(c));
});

test('R1-M7: a run summary carries counts, the Worker block and the first world point time', () => {
  const s = summarizeRun({ updates, frames, responses, longtasks: [{ start: 1100, duration: 60 }], rafTimes: [1085, 1100, 1160, 1170],
    chunkStats: { stageMs: [1, 1], readMs: [2, 2], engineMs: [3, 3], stagedFrames: 2, processedFrames: 2, freshResponses: 2, droppedStaged: 0 },
    engineWorker: { active: true, fallback: null, stats: { framesSubmitted: 2, resultsReceived: 2, workerEngineMs: [2, 4], rttMs: [5, 6], transitMs: [1, 1] } } });
  assert.equal(s.mainThreadChunkMs.p50, 6);
  assert.equal(s.freshUpdates, 4);
  assert.equal(s.timeToFirstWorldPointMs, 0);
  assert.equal(s.longtasks.count, 1);
  assert.equal(s.raf.over50, 1);
  assert.equal(s.engineWorker.workerEngineMs.p50, 2);
  assert.equal(s.counts.processed, 2);
});

test('R1-M8: ABAB x3 aggregation takes per-arm medians and builds the identity matrix (main vs main = noise floor)', () => {
  const mk = (arm, p50, outputs) => ({ arm, outputs, summary: summarizeRun({ updates, frames, responses,
    chunkStats: { stageMs: [p50], readMs: [0], engineMs: [0], processedFrames: 1, droppedStaged: 0 } }) });
  const out = outputsBySource({ updates, frames, responses });
  const runs = [mk('main', 10, out), mk('worker', 2, out), mk('main', 12, out), mk('worker', 3, out), mk('main', 11, out), mk('worker', 4, out)];
  const r = aggregate(runs);
  assert.deepEqual(r.order, ['main', 'worker', 'main', 'worker', 'main', 'worker']);
  assert.equal(r.main.mainThreadChunkMsP50, 11);
  assert.equal(r.worker.mainThreadChunkMsP50, 3);
  assert.equal(r.main.runs, 3);
  assert.equal(r.identity.mainVsMain.length, 2);
  assert.equal(r.identity.workerVsMain.length, 3);
  assert.ok(r.identity.workerVsMain.every((x) => x.identicalFraction === 1));
});

// ---- preregistered R1 metrics ----
import { aggregatePrimary, frameWindow, fullWindow, measuredFrameCount, percentileLinear, primaryOfRun, replayIdentity, scorerRun, topLevelBusy } from './engine-worker-ab-metrics.mjs';

test('R1-M9: percentileLinear is numpy linear interpolation', () => {
  assert.equal(percentileLinear([1, 2, 3, 4], 0.5), 2.5);
  assert.equal(percentileLinear([10, 20, 30, 40, 50], 0.95), 48);
  assert.equal(percentileLinear([5], 0.95), 5);
  assert.equal(percentileLinear([], 0.95), null);
  assert.equal(percentileLinear([3, NaN, 1], 0.5), 2);
});

const thread = (pid, tid, name) => ({ ph: 'M', name: 'thread_name', pid, tid, args: { name } });
const mark = (name, ts, pid = 1, tid = 2) => ({ ph: 'R', name, ts, pid, tid, cat: 'blink.user_timing' });
const task = (ts, dur, pid = 1, tid = 2) => ({ ph: 'X', name: 'RunTask', ts, dur, pid, tid, cat: 'disabled-by-default-devtools.timeline' });

test('R1-M10: top-level busy time = union of RunTask on the marked main thread, clipped to the mark window, other threads and idle excluded', () => {
  const trace = [thread(1, 2, 'CrRendererMain'), thread(1, 3, 'DedicatedWorker thread'),
    task(0, 50_000),                       // ends 50 ms in, window starts at 40 ms: 10 ms inside
    mark('w0', 40_000), task(60_000, 10_000), task(65_000, 10_000),   // overlap: union 60..75 = 15 ms
    task(100_000, 20_000, 1, 3),           // the Worker thread: not counted
    task(190_000, 30_000),                 // crosses the window end (200 ms): 10 ms inside
    mark('w1', 200_000), task(300_000, 40_000)];
  const r = topLevelBusy(trace, { startMark: 'w0', endMark: 'w1' });
  assert.equal(r.windowMs, 160);
  assert.equal(r.busyMs, 10 + 15 + 10);
  assert.equal(r.boundaryClippedTasks, 2);
  assert.equal(r.mainThreadVerified, true);
  assert.equal(r.source, 'RunTask');
  assert.match(topLevelBusy(trace, { startMark: 'nope', endMark: 'w1' }).error, /lacks the window mark nope/);
  assert.match(topLevelBusy([mark('a', 0), mark('b', 5, 1, 9)], { startMark: 'a', endMark: 'b' }).error, /different threads/);
  assert.match(topLevelBusy([mark('a', 0), mark('b', 5)], { startMark: 'a', endMark: 'b' }).error, /no top-level task/);
});

test('R1-M11: B/E task pairs and the ThreadControllerImpl::RunTask fallback are read too', () => {
  const trace = [mark('a', 0), { ph: 'B', name: 'ThreadControllerImpl::RunTask', ts: 10, pid: 1, tid: 2 }, { ph: 'E', name: 'ThreadControllerImpl::RunTask', ts: 4010, pid: 1, tid: 2 }, mark('b', 10_000)];
  const r = topLevelBusy(trace, { startMark: 'a', endMark: 'b' });
  assert.equal(r.busyMs, 4);
  assert.equal(r.source, 'ThreadControllerImpl::RunTask');
});

// A run of N = 30 + 2m frames (seq = index + 1), m = 2: the middle window is seq 30 .. 34 (5 frames), the measured segment 31..32, the trailing segment 33..34.
// `boot` frame callbacks precede the recording in the page (capture index -1, seq 1e6 + n). Frame k: capture entry at k * 33, start mark 1 ms later,
// the engine response to frame k is applied at callback k + lag (lag 0 = main thread, 1 = Worker; render exit +2 ms, done mark +3 ms).
function fixtureRun({ N = 34, m = 2, lag = 0, boot = 3, full = false, mutate = () => {} }) {
  const applied = [], marks = [], renders = [], captures = [], renderCallbacks = [];
  for (let b = 0; b < boot; b++) { captures.push([-1, -100 + b]); marks.push([`sr-frame-start:${1000001 + b}`, -99 + b]); }
  for (let k = 0; k < N; k++) {
    captures.push([k, k * 33]);
    marks.push([`sr-frame-start:${k + 1}`, k * 33 + 1]);
    renderCallbacks.push([k, k * 33 + 20]);
  }
  for (let k = 0; k <= N - 1; k++) {
    const cb = k + lag, t = cb * 33 + 20;
    applied.push([k, t, cb, 0, k]);
    renders.push([t + 2]);
    marks.push([`sr-frame-done:${k + 1}`, t + 3]);
  }
  const run = { applied, marks, renders, captures, renderCallbacks, totalFrames: N, m };
  mutate(run);
  return full ? fullWindow({ ...run, n: 30 + 2 * m }) : frameWindow(run);
}
const setMark = (run, name, fn) => { const e = run.marks.find(([n]) => n === name); e[1] = fn(e[1]); };

test('R1-M12: the window is frames 30 .. 30+2m (2m+1 frames, 1-based seq); a clean run is valid; boot frames do not matter', () => {
  assert.equal(measuredFrameCount(120), 241);
  const w = fixtureRun({});
  assert.equal(w.windowFrames, 5);
  assert.equal(w.startMark, 'sr-frame-start:30');
  assert.equal(w.endMark, 'sr-frame-done:34');
  assert.deepEqual(w.measuredSegment, [31, 32]);
  assert.deepEqual(w.trailingSegment, [33, 34]);
  assert.equal(w.valid, true, JSON.stringify(w));
  assert.equal(w.latencyMs.n, 2);
  assert.equal(w.latencyMs.p95, 20 + 3);   // capture k*33 -> done k*33 + 23 (lag 0)
  assert.equal(fixtureRun({ boot: 0 }).latencyMs.p95, w.latencyMs.p95);
});

test('R1-M13: the Worker lag (one more callback) keeps the run valid; the same frames are measured; latency is larger', () => {
  const a = fixtureRun({ lag: 0 }), b = fixtureRun({ lag: 1 });
  assert.equal(b.valid, true);
  assert.equal(b.windowFrames, a.windowFrames);
  assert.ok(b.latencyMs.p95 > a.latencyMs.p95);
});

test('R1-M14: every registered mark rule is enforced (exactly one start and done per frame, strictly increasing, start_k < done_k, measured frames done before the window end, one render callback inside [start_k, done_k])', () => {
  const bad = (mutate, key, frames) => { const w = fixtureRun({ mutate }); assert.equal(w.valid, false, key); assert.deepEqual(w[key], frames, key); };
  bad((r) => setMark(r, 'sr-frame-done:33', (t) => t - 60), 'doneNotIncreasing', [33]);        // frame 33 done before frame 32 done
  bad((r) => setMark(r, 'sr-frame-start:33', (t) => t - 40), 'startNotIncreasing', [33]);
  bad((r) => setMark(r, 'sr-frame-done:33', () => 0), 'startNotBeforeDone', [33]);
  bad((r) => { r.marks = r.marks.filter(([n]) => n !== 'sr-frame-start:32'); }, 'missingStart', [32]);
  bad((r) => { r.marks = r.marks.filter(([n]) => n !== 'sr-frame-done:32'); }, 'missingDone', [32]);
  bad((r) => r.marks.push(['sr-frame-start:31', 5000]), 'duplicateStart', [31]);
  bad((r) => r.marks.push(['sr-frame-done:31', 5000]), 'duplicateDone', [31]);
  bad((r) => setMark(r, 'sr-frame-done:32', () => 99999), 'measuredDoneAfterWindowEnd', [32]);   // measured frame finished after the window ended
  bad((r) => r.applied.push([30, 999, 40, 1, 30]), 'renderCallbackNotOnce', [31]);              // the pose of seq 31 consumed twice
  bad((r) => { r.applied = r.applied.filter(([i]) => i !== 31); }, 'renderCallbackNotOnce', [32]);
  bad((r) => { r.applied.find(([i]) => i === 30)[1] = 10_000; }, 'renderCallbackNotInside', [31]);   // outside [start, done]
  bad((r) => { r.applied.find(([i]) => i === 30)[1] = 0; }, 'renderCallbackNotInside', [31]);        // before its start mark
  bad((r) => { const e = r.marks.find(([n]) => n === 'sr-frame-done:32'); e[1] = r.renders[31][0] - 1; }, 'markBeforeRenderReturned', [32]);
  const short = fixtureRun({ N: 33 });
  assert.equal(short.valid, false);
  assert.match(short.errors[0], /input too short/);
  assert.equal(fixtureRun({ N: 34 }).valid, true, 'frame 34 has its response with no later frame');
});

test('R1-M15: render callbacks of trailing frames are not part of rule (e) (only the measured segment is)', () => {
  const w = fixtureRun({ mutate: (r) => r.applied.push([32, 999, 40, 1, 32]) });   // seq 33 is trailing
  assert.equal(w.valid, true);
});

test('R1-M16: primaryOfRun (v4.5) sums the main-thread tasks from the trace start to sr-frame-done:N, divides by N, reports the middle window and the 3000 ms after', () => {
  const w = fixtureRun({ N: 34, full: true }), mid = fixtureRun({ N: 35 });
  assert.equal(w.n, 34);
  assert.equal(w.valid, true, JSON.stringify(w.violations) + w.errors);
  const trace = [thread(1, 2, 'CrRendererMain'), { ph: 'R', name: 'TracingStartedInBrowser', ts: 100, pid: 9, tid: 9 }, task(500, 10_000), mark('sr-frame-start:30', 20_000), task(20_000, 5000),
    mark(mid.endMark, 70_000), mark(w.endMark, 90_000), task(60_000, 8000), task(95_000, 5000), task(3_200_000, 9000)];
  const p = primaryOfRun({ trace, window: w, middle: mid });
  assert.equal(p.busy.windowStart, 'trace-start');
  assert.equal(p.busy.busyMs, 10 + 5 + 8);                     // everything from the trace start to done:34, the task at 95 ms is after
  assert.equal(p.busyMsPerFrame, 23 / 34);
  assert.equal(p.busy.afterWindow3000Ms, 5);                   // 95..100 ms is within 3000 ms after; the task at 3.2 s is not
  assert.equal(p.measuredFrames, 34);
  assert.equal(p.valid, true);
  assert.equal(p.middle.reportOnly, true);
  assert.equal(p.middle.busyMsPerFrame, (5 + 8) / 5);          // [sr-frame-start:30 @20ms, sr-frame-done:34] only... see the marks above
  assert.equal(primaryOfRun({ trace: [thread(1, 2, 'Other'), ...trace.slice(1)], window: w, middle: mid }).valid, false);
  assert.equal(primaryOfRun({ trace: [], window: w, middle: mid }).valid, false);
  assert.equal(primaryOfRun({ trace, window: fixtureRun({ N: 34, full: true, mutate: (r) => setMark(r, 'sr-frame-done:10', () => 0) }), middle: mid }).valid, false);
});

test('R1-M19: fullWindow (v4.5) needs frames 1..N with exactly one start/done each, strictly increasing, start_k < done_k, one render callback inside [start_k, done_k], and nothing beyond N', () => {
  const ok = fixtureRun({ N: 34, full: true });
  assert.equal(ok.valid, true, JSON.stringify(ok.violations));
  assert.equal(ok.endMark, 'sr-frame-done:34');
  assert.equal(ok.startMark, null, 'the window starts at the trace start, not at a mark');
  assert.equal(ok.latencyMs.n, 2, 'latency p95 over the measured segment 31..30+m');
  assert.equal(ok.latencyAllFramesMs.n, 34);
  const bad = (mutate, key, frames) => { const w = fixtureRun({ N: 34, full: true, mutate }); assert.equal(w.valid, false, key); assert.deepEqual(w[key], frames, key); };
  bad((r) => { r.marks = r.marks.filter(([n]) => n !== 'sr-frame-start:1'); }, 'missingStart', [1]);   // a warm-up frame counts too
  bad((r) => { r.marks = r.marks.filter(([n]) => n !== 'sr-frame-done:34'); }, 'missingDone', [34]);
  bad((r) => r.marks.push(['sr-frame-start:5', 9999]), 'duplicateStart', [5]);
  bad((r) => setMark(r, 'sr-frame-done:20', (t) => t - 80), 'doneNotIncreasing', [20]);
  bad((r) => setMark(r, 'sr-frame-start:20', (t) => t - 80), 'startNotIncreasing', [20]);
  bad((r) => r.applied.push([2, 5, 3, 1, 2]), 'renderCallbackNotOnce', [3]);
  bad((r) => { r.applied = r.applied.filter(([i]) => i !== 8); }, 'renderCallbackNotOnce', [9]);
  bad((r) => { r.applied.find(([i]) => i === 8)[1] = 99999; }, 'renderCallbackNotInside', [9]);
  const beyond = fixtureRun({ N: 35, full: true });     // the input must END at N: a frame 35 has marks and a render callback
  assert.equal(beyond.valid, false);
  assert.deepEqual(beyond.framesBeyondN, [35]);
  assert.equal(fixtureRun({ N: 33, full: true }).valid, false);
});

test('R1-M17: aggregatePrimary takes per-arm medians, B/A ratios and checks the measured frames are equal across runs; replayIdentity needs one digest and a count', () => {
  const mk =(arm, busy, lat, frames = 5) => ({ arm, primary: { valid: true, busyMsPerFrame: busy, e2eLatencyP95Ms: lat, measuredFrames: frames } });
  const r = aggregatePrimary([mk('main', 10, 100), mk('worker', 5, 105), mk('main', 12, 110), mk('worker', 6, 120), mk('main', 11, 90), mk('worker', 7, 100)]);
  assert.equal(r.main.busyMsPerFrame, 11);
  assert.equal(r.worker.busyMsPerFrame, 6);
  assert.equal(r.framesEqualAcrossRuns, true);
  assert.ok(Math.abs(r.ratios.busyWorkerOverMain - 6 / 11) < 1e-12);
  assert.equal(aggregatePrimary([mk('main', 1, 1, 5), mk('worker', 1, 1, 6)]).framesEqualAcrossRuns, false);
  assert.equal(replayIdentity([{ arm: 'main', digest: 'x', count: 3 }, { arm: 'worker', digest: 'x', count: 3 }]).allIdentical, true);
  assert.equal(replayIdentity([{ arm: 'main', digest: 'x', count: 3 }, { arm: 'worker', digest: 'y', count: 3 }]).allIdentical, false);
  assert.equal(replayIdentity([{ arm: 'main', digest: 'x', count: 0 }]).allIdentical, false, 'empty output is not identity');
});

test('R1-M18: scorerRun builds the r1_eval run record on one clock (marks and tasks from the trace, render times moved by the median mark offset, nested tasks merged)', () => {
  const trace = [thread(1, 2, 'CrRendererMain'), mark('sr-frame-start:30', 1_000_000), mark('sr-frame-start:31', 1_033_000), mark('sr-frame-done:30', 1_050_000),
    mark('sr-frame-start:1000001', 900_000), task(1_000_000, 10_000), task(1_002_000, 3_000), task(1_040_000, 5_000), task(1_040_000, 5_000, 1, 3)];
  const r = scorerRun(trace, { pageMarks: [['sr-frame-start:30', 500], ['sr-frame-start:31', 533], ['sr-frame-done:30', 550]], applied: [[29, 549, 30, 0, 29]] });
  assert.equal(r.offsetMs, 500);
  assert.equal(r.offsetSpreadMs, 0);
  assert.deepEqual(r.start, { 30: [1000], 31: [1033] });         // the page-fed frame (seq >= 1e6) is not a recorded frame
  assert.deepEqual(r.done, { 30: [1050] });
  assert.deepEqual(r.render, { 30: [1049] });
  assert.deepEqual(r.tasks, [[1000, 1010], [1040, 1045]]);       // the nested task is merged, the other thread's is not included
  assert.deepEqual(r.work, {});
  assert.match(scorerRun([], { pageMarks: [], applied: [] }).error, /no sr-frame-start mark/);
  assert.equal(r.trace_start, 900, 'no TracingStartedInBrowser event: the earliest event (the page-fed frame mark)');
}
);

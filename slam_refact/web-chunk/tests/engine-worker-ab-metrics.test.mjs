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

// node --test slam_refact/web-chunk/tests/tracker-diagnostics.test.mjs
// A-0e: SLAM_REFACT_CHUNK_OPTIONS.trackerDiagnostics = {enabled: true} (opt-in, default OFF) exposes diag.trackerDiag: the engine's
// tracker step of the pose source frame joined with the DeviceMotion summary of that frame. Checked with the real engine WASM of the
// harness (skipped when it is not available): OFF adds nothing and ON changes no answer, the copied fields equal the engine response,
// the IMU summary belongs to the tracked frame (not the frame staged in the same call), counters follow the responses, and the
// engine Worker path stays null with a warning. The sample-page guard needs no engine.
import assert from 'node:assert/strict';
import fs from 'node:fs';
import path from 'node:path';
import test from 'node:test';
import { engineAvailable, loadRealModule, loopbackWorker, runChunk, syntheticAtlas, syntheticMotion, chunkDir } from './engine-worker-harness.mjs';

const skip = engineAvailable ? false : 'engine build not found (SLAM_REFACT_CHUNK_ENGINE_DIR)';
const W = 240, H = 320;
let Module = null;
const atlases = new Map();
test.before(async () => { if (!skip) Module = await loadRealModule(); });
const atlasFor = (k) => atlases.get(k) ?? (atlases.set(k, syntheticAtlas(Module, W, H, k).rgba), atlases.get(k));
const gyroOf = (e) => Math.hypot(e.rotationRate.alpha, e.rotationRate.beta, e.rotationRate.gamma);
const accOf = (e) => Math.hypot(e.acceleration.x, e.acceleration.y, e.acceleration.z);
const ON = { trackerDiagnostics: { enabled: true } };

// Wraps the real glue so every response of processFramePyramid is kept (a detached copy of the fields under test).
const capturingGlue = (seen) => (M, config) => {
  const glue = new M.SlamRefactGlue(config);
  return new Proxy(glue, { get(target, name) {
    const v = target[name];
    if (name !== 'processFramePyramid') return typeof v === 'function' ? v.bind(target) : v;
    return (...args) => {
      const pose = v.apply(target, args);
      seen.push({ repeated: pose.repeated, videoFrameIndex: pose.videoFrameIndex, sourceFrameIndex: pose.sourceFrameIndex,
        keypoints: pose.keypoints ?? null, tracker: pose.tracker ? structuredClone(pose.tracker) : null });
      return pose;
    };
  } });
};

test('TD-1: without the option there is no trackerDiag, and turning it on changes no answer', { skip }, async () => {
  const off = await runChunk({ atlasFor, steps: 12 });
  const offAnswers = JSON.stringify(off.answers), offStats = { ...off.diag.stats }, offWarnings = [...off.diag.warnings];
  const offHas = 'trackerDiag' in off.diag;
  off.close();
  const on = await runChunk({ atlasFor, steps: 12, extraOptions: ON });
  try {
    assert.deepEqual(off.errors, []);
    assert.deepEqual(on.errors, []);
    assert.equal(offHas, false, 'OFF must not define the property');
    assert.deepEqual(on.diag.warnings, offWarnings);
    assert.equal(JSON.stringify(on.answers), offAnswers, 'the published answers are the same with and without the option');
    for (const k of ['stagedFrames', 'processedFrames', 'freshResponses', 'motionEventsPushed', 'lastSourceFrameIndex']) {
      assert.equal(on.diag.stats[k], offStats[k], k);
    }
  } finally { on.close(); }
});

test('TD-2: the fields are the engine response of the source frame, sampled latest-only', { skip }, async () => {
  const seen = [], samples = [];
  const r = await runChunk({ atlasFor, steps: 14, extraOptions: { ...ON, createGlue: capturingGlue(seen) },
    afterEach: (k, ctx) => { samples.push({ k, td: ctx.diag().trackerDiag, last: seen[seen.length - 1] }); } });
  try {
    assert.deepEqual(r.errors, []);
    const fresh = samples.filter((s) => s.last && !s.last.repeated && s.last.tracker);
    assert.ok(fresh.length >= 8, `expected tracked frames, got ${fresh.length}`);
    for (const { td, last } of fresh) {
      const t = last.tracker;
      assert.equal(td.sourceFrameIndex, last.sourceFrameIndex);
      for (const k of ['mapTrackerState', 'frameTrackerOk', 'ok', 'lossState', 'lostFrames', 'frames', 'mapReset', 'keyframeCreated', 'numKeyframes', 'groundMatches']) {
        assert.deepEqual(td[k], t[k], k);
      }
      assert.equal(td.keypoints, last.keypoints);
      assert.deepEqual([td.pointsTriangulated, td.pointsGround, td.pointsSky, td.pointsDepth, td.pointsCloud],
        [t.pointsBySource.triangulated, t.pointsBySource.ground, t.pointsBySource.sky, t.pointsBySource.depth, t.pointsBySource.cloud]);
      assert.equal(td.session, 1);
    }
    // Before the first tracked frame the value is null, not a made-up row.
    const first = samples.findIndex((s) => s.td !== null);
    assert.ok(first >= 0);
    for (const s of samples.slice(0, first)) assert.equal(s.td, null);
    // Counters follow the engine responses.
    const end = samples[samples.length - 1].td;
    const steps = seen.filter((p) => !p.repeated && p.tracker);
    assert.equal(end.totalResponses, steps.length);
    assert.equal(end.totalResponses, r.diag.stats.freshResponses);
    assert.equal(end.totalMapResets, steps.filter((p) => p.tracker.mapReset).length);
    assert.equal(end.totalKeyframesCreated, steps.filter((p) => p.tracker.keyframeCreated).length);
    assert.equal(end.totalMapTrackerNotOk, steps.filter((p) => p.tracker.mapTrackerState !== 1).length);
    assert.equal(end.totalInputMissing, 0);
    // The getter hands out a detached copy.
    const a = r.diag.trackerDiag; a.totalMapResets = 999; a.mapReset = 'x';
    assert.notEqual(r.diag.trackerDiag.totalMapResets, 999);
    assert.notEqual(r.diag.trackerDiag.mapReset, 'x');
  } finally { r.close(); }
});

test('TD-2b: each counter counts its own flag (scripted tracker flags on the real responses)', { skip }, async () => {
  // The synthetic run resets the map and creates a keyframe the same number of times, so the flags are scripted here: the real glue
  // answers, and the three flags of every fresh response are replaced before the chunk reads them.
  // Frames 7..12 (frame 5 is never tracked in this run: the startup recenter repeats the previous response).
  const RESET = new Set([7, 11]), KEYFRAME = new Set([8]), NOT_TRACKED = new Set([9, 10, 11]);
  const scripted = (M, config) => {
    const glue = new M.SlamRefactGlue(config);
    return new Proxy(glue, { get(target, name) {
      const v = target[name];
      if (name !== 'processFramePyramid') return typeof v === 'function' ? v.bind(target) : v;
      return (...args) => {
        const pose = v.apply(target, args);
        if (!pose.repeated && pose.tracker) {
          pose.tracker.mapReset = RESET.has(pose.sourceFrameIndex);
          pose.tracker.keyframeCreated = KEYFRAME.has(pose.sourceFrameIndex);
          pose.tracker.mapTrackerState = NOT_TRACKED.has(pose.sourceFrameIndex) ? 0 : 1;
        }
        return pose;
      };
    } });
  };
  const samples = [];
  const r = await runChunk({ atlasFor, steps: 14, extraOptions: { ...ON, createGlue: scripted },
    afterEach: (k, ctx) => { samples.push(ctx.diag().trackerDiag); } });
  try {
    assert.deepEqual(r.errors, []);
    const bySource = new Map(samples.filter(Boolean).map((td) => [td.sourceFrameIndex, td]));
    const F = [7, 8, 9, 10, 11, 12];
    for (const i of F) assert.ok(bySource.has(i), `frame ${i} sampled`);
    assert.deepEqual(F.map((i) => bySource.get(i).totalMapResets), [1, 1, 1, 1, 2, 2]);
    assert.deepEqual(F.map((i) => bySource.get(i).totalKeyframesCreated), [0, 1, 1, 1, 1, 1]);
    assert.deepEqual(F.map((i) => bySource.get(i).totalMapTrackerNotOk), [0, 0, 1, 2, 3, 3]);
    assert.deepEqual([7, 8, 11].map((i) => [bySource.get(i).mapReset, bySource.get(i).keyframeCreated, bySource.get(i).mapTrackerState]),
      [[true, false, 1], [false, true, 1], [true, false, 0]]);
  } finally { r.close(); }
});

test('TD-3: the IMU summary is the one of the tracked frame (responses lag one frame)', { skip }, async () => {
  const BIG = 7;  // an extra event with a large rotation rate and acceleration is dispatched after step BIG-1: it is staged with frame BIG
  const big = { type: 'devicemotion', timeStamp: 2000, interval: 16, acceleration: { x: 3, y: 0, z: 4 },
    accelerationIncludingGravity: { x: 3, y: 9.8, z: 4 }, rotationRate: { alpha: 90, beta: 0, gamma: 120 } };
  const samples = [];
  const r = await runChunk({ atlasFor, steps: 14, extraOptions: ON,
    afterEach: (k, ctx) => { samples.push(ctx.diag().trackerDiag); if (k === BIG - 1) ctx.g.dispatch(big); } });
  try {
    assert.deepEqual(r.errors, []);
    const rows = samples.filter(Boolean);
    const bySource = new Map(rows.map((td) => [td.sourceFrameIndex, td]));
    assert.ok(bySource.has(BIG) && bySource.has(BIG - 1) && bySource.has(BIG + 1));
    for (const [idx, td] of bySource) {
      assert.equal(td.inputFound, true);
      if (idx === BIG) {
        assert.equal(td.motionEvents, 2);
        assert.equal(td.gyroPeakDegS, 150);   // hypot(90, 0, 120)
        assert.equal(td.linAccPeakMs2, 5);    // hypot(3, 0, 4)
      } else {
        assert.equal(td.motionEvents, 1, `frame ${idx}`);
        assert.ok(Math.abs(td.gyroPeakDegS - gyroOf(syntheticMotion(idx))) < 1e-12, `gyro of frame ${idx}`);
        assert.ok(Math.abs(td.linAccPeakMs2 - accOf(syntheticMotion(idx))) < 1e-12, `acceleration of frame ${idx}`);
      }
    }
  } finally { r.close(); }
});

test('TD-4: events without a gyroscope or with non-finite values give null peaks, not NaN', { skip }, async () => {
  const samples = [];
  const r = await runChunk({ atlasFor, steps: 8, extraOptions: ON,
    globals: (g) => {
      const dispatch = g.dispatch;
      g.dispatch = (e) => dispatch(e.type === 'devicemotion' ? { ...e, rotationRate: null, acceleration: { x: NaN, y: 0, z: 0 } } : e);
    },
    afterEach: (k, ctx) => { samples.push(ctx.diag().trackerDiag); } });
  try {
    assert.deepEqual(r.errors, []);
    const rows = samples.filter(Boolean);
    assert.ok(rows.length >= 3);
    for (const td of rows) {
      assert.equal(td.motionEvents, 1);
      assert.equal(td.gyroPeakDegS, null);
      assert.equal(td.linAccPeakMs2, null);
    }
  } finally { r.close(); }
});

test('TD-7: a rebuilt session gets a new session number and the old frame is not shown as current', { skip }, async () => {
  const samples = [];
  const r = await runChunk({ atlasFor, steps: 16, extraOptions: ON,
    afterEach: (k, ctx) => {
      samples.push({ k, td: ctx.diag().trackerDiag, created: ctx.diag().stats.sessionsCreated });
      if (k === 7) ctx.pm.onDeviceOrientationChange({ videoWidth: W, videoHeight: H, orientation: 0, computeCtx: ctx.gl, framework: ctx.framework });
    } });
  try {
    assert.deepEqual(r.errors, []);
    assert.equal(samples.at(-1).created, 2, 'the session was rebuilt once');
    const before = samples.filter((x) => x.k <= 7 && x.td), after = samples.filter((x) => x.k > 7);
    assert.ok(before.length >= 4 && before.every((x) => x.td.session === 1));
    // Right after the rebuild the new session has not tracked a frame: null, never the old session's last frame.
    assert.equal(after[0].td, null);
    const tracked = after.filter((x) => x.td);
    assert.ok(tracked.length >= 4);
    assert.ok(tracked.every((x) => x.td.session === 2));
    assert.ok(tracked[0].td.sourceFrameIndex <= before.at(-1).td.sourceFrameIndex, 'the frame index restarts, so (session, sourceFrameIndex) is the key');
    assert.ok(tracked.every((x) => x.td.inputFound), 'the input of a new session is not looked up in the old session');
    assert.ok(tracked.at(-1).td.totalResponses > before.at(-1).td.totalResponses, 'the counters go on over the rebuild');
  } finally { r.close(); }
});

test('TD-8: events pushed before a frame call that throws are counted with the frame that gets them', { skip }, async () => {
  const THROW_AT = 8;  // the 9th processFramePyramid call of the session throws once, after its DeviceMotion event was pushed
  let calls = 0;
  const throwing = (M, config) => {
    const glue = new M.SlamRefactGlue(config);
    return new Proxy(glue, { get(target, name) {
      const v = target[name];
      if (name !== 'processFramePyramid') return typeof v === 'function' ? v.bind(target) : v;
      return (...args) => { if (calls++ === THROW_AT) throw new Error('test: frame call failed'); return v.apply(target, args); };
    } });
  };
  const samples = [];
  const r = await runChunk({ atlasFor, steps: 16, extraOptions: { ...ON, createGlue: throwing },
    afterEach: (k, ctx) => { samples.push(ctx.diag().trackerDiag); } });
  try {
    assert.ok(r.errors.some((e) => /test: frame call failed/.test(e)), 'the failure is reported, not swallowed');
    const rows = [...new Map(samples.filter(Boolean).map((td) => [td.sourceFrameIndex, td])).values()];
    const two = rows.filter((td) => td.motionEvents === 2);
    assert.equal(two.length, 1, 'exactly one tracked frame carries the event of the failed call as well');
    // The failed call staged nothing, so the next call stages the glue's frame THROW_AT with both events.
    assert.equal(two[0].sourceFrameIndex, THROW_AT);
    assert.ok(rows.filter((td) => td !== two[0]).every((td) => td.motionEvents === 1));
    assert.equal(r.diag.stats.motionEventsPushed, 16);
    assert.equal(rows.reduce((n, td) => n + td.motionEvents, 0) <= 16, true);
  } finally { r.close(); }
});

test('TD-9: an event the engine did not take (invalid interval) is not counted', { skip }, async () => {
  const BAD = 6;
  const bad = { type: 'devicemotion', timeStamp: 3000, interval: Infinity, acceleration: { x: 30, y: 0, z: 40 },
    accelerationIncludingGravity: { x: 30, y: 9.8, z: 40 }, rotationRate: { alpha: 900, beta: 0, gamma: 1200 } };
  const samples = [];
  const r = await runChunk({ atlasFor, steps: 12, extraOptions: ON,
    afterEach: (k, ctx) => { samples.push(ctx.diag().trackerDiag); if (k === BAD - 1) ctx.g.dispatch(bad); } });
  try {
    assert.equal(r.diag.stats.motionEventsSkipped, 1);
    const td = samples.filter(Boolean).find((x) => x.sourceFrameIndex === BAD);
    assert.ok(td, 'frame tracked');
    assert.equal(td.motionEvents, 1, 'the skipped event is not an engine input');
    assert.ok(td.gyroPeakDegS < 1 && td.linAccPeakMs2 < 1, 'its magnitudes are not reported as peaks');
  } finally { r.close(); }
});

test('TD-5: with the engine Worker active trackerDiag stays null and the chunk says so', { skip }, async () => {
  const loop = loopbackWorker();
  const r = await runChunk({ atlasFor, steps: 8, loop, extraOptions: ON, engineWorker: { enabled: true, createWorker: () => loop.worker } });
  try {
    await r.drain(3);
    assert.deepEqual(r.errors, []);
    assert.equal(r.diag.engineWorker.active, true);
    assert.equal(r.diag.trackerDiag, null);
    assert.ok(r.diag.warnings.some((w) => /trackerDiagnostics records only when the engine runs on the main thread/.test(w)));
  } finally { r.close(); }
});

test('TD-6: no shipped sample or page turns trackerDiagnostics on by default', () => {
  const hits = [];
  const walk = (dir, root) => {
    for (const e of fs.readdirSync(dir, { withFileTypes: true })) {
      const p = path.join(dir, e.name);
      if (e.isDirectory()) { if (!['node_modules', 'tests', '__pycache__'].includes(e.name)) walk(p, root); }
      else if (/\.(js|mjs|html)$/.test(e.name) && /trackerDiagnostics/.test(fs.readFileSync(p, 'utf8'))) hits.push(path.relative(root, p));
    }
  };
  walk(chunkDir, chunkDir);
  const smokeDir = path.resolve(chunkDir, '../web-smoke');
  if (fs.existsSync(smokeDir)) walk(smokeDir, path.resolve(chunkDir, '..'));
  assert.deepEqual(hits.sort(), ['samples/phone/app.js', 'xr-slam.js']);
  const app = fs.readFileSync(path.join(chunkDir, 'samples/phone/app.js'), 'utf8');
  // Every non-comment use is the one assignment guarded by TRACKER_DIAG, and TRACKER_DIAG needs both record=1 and diag=1.
  const lines = app.split('\n').filter((l) => /trackerDiagnostics/.test(l) && !/^\s*\/\//.test(l));
  assert.equal(lines.length, 1);
  assert.match(lines[0], /^if \(TRACKER_DIAG\) window\.SLAM_REFACT_CHUNK_OPTIONS = /);
  assert.match(app, /const TRACKER_DIAG = RECORD && new URLSearchParams\(window\.location\.search\)\.get\('diag'\) === '1';/);
});

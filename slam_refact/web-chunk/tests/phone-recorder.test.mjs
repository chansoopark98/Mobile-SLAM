// samples/phone/metrics.js: the opt-in pose log (?record=1) and the receipt. Pure logic only: a fake clock, a fake camera track, a fake fetch.
// Not covered here (needs a device): that xr.js really passes videoTime/frameTime and a camera track to the page.
import test from 'node:test';
import assert from 'node:assert/strict';
import { webcrypto } from 'node:crypto';
import { createMetrics, createPoseRecorder, pickSourceTiming, pickTrackerDiag, TRACKER_DIAG_KEYS, buildReceipt, summarizeChunkStats, summarize, POSE_LOG_CAP } from '../samples/phone/metrics.js';

const clock = (step = 0.25) => { let t = 1000; return () => (t += step); };
const reality = (over = {}) => ({
  trackingStatus: 'NORMAL', trackingReason: 'UNSPECIFIED', position: { x: 1, y: 1.5, z: -2 }, rotation: { x: 0, y: 0, z: 0, w: 1 },
  worldPoints: [{}, {}, {}], ...over,
});

test('recorder: rows beyond the cap are counted as dropped, never silently lost', () => {
  const rec = createPoseRecorder(clock(), 3);
  for (let i = 0; i < 5; i++) rec.add(reality({ position: { x: i, y: 0, z: 0 } }), { videoTime: i, frameTime: i + 0.5 });
  const s = rec.snapshot();
  assert.equal(s.rows.length, 3);
  assert.deepEqual(s.rows.map((r) => r.position.x), [0, 1, 2], 'the first rows are kept, in order');
  assert.equal(s.droppedRows, 2);
  assert.equal(s.cap, 3);
  assert.deepEqual(rec.counts(), { rows: 3, droppedRows: 2 });
  assert.equal(s.overheadMs.n, 3, 'overhead is timed for stored rows only');
});

test('recorder: default cap is 40000 and the row has the planned fields', () => {
  assert.equal(POSE_LOG_CAP, 40000);
  const rec = createPoseRecorder(clock());
  rec.add(reality(), { videoTime: 12.5, frameTime: 99.25 });
  const s = rec.snapshot(4);
  assert.equal(s.schema, 'slam-refact-xr-poselog-v3');
  assert.equal(s.cap, 40000);
  assert.equal(s.nullUpdates, 4);
  assert.deepEqual(Object.keys(s.rows[0]), ['tPerfMs', 'videoTimeS', 'frameTimeS', 'repeatFrame', 'sourceFrameIndex', 'source', 'status', 'reason', 'position', 'rotation', 'worldPoints']);
  assert.deepEqual(s.rows[0], {
    tPerfMs: s.rows[0].tPerfMs, videoTimeS: 12.5, frameTimeS: 99.25, repeatFrame: null, sourceFrameIndex: null, source: null, status: 'NORMAL', reason: 'UNSPECIFIED',
    position: { x: 1, y: 1.5, z: -2 }, rotation: { x: 0, y: 0, z: 0, w: 1 }, worldPoints: 3,
  });
  assert.equal(typeof s.rows[0].tPerfMs, 'number');
});

test('recorder: missing xr.js times and non-finite numbers become null, a missing pose does not throw', () => {
  const rec = createPoseRecorder(clock());
  rec.add(reality({ position: { x: NaN, y: Infinity, z: 1 } }), undefined);
  rec.add(reality({ position: undefined, rotation: undefined, worldPoints: undefined }), { videoTime: undefined });
  const [a, b] = rec.snapshot().rows;
  assert.deepEqual([a.videoTimeS, a.frameTimeS], [null, null]);
  assert.deepEqual(a.position, { x: null, y: null, z: 1 });
  assert.equal(b.position, null);
  assert.equal(b.rotation, null);
  assert.equal(b.worldPoints, 0);
  assert.ok(JSON.parse(JSON.stringify(a)).position.x === null);
});

test('recorder: overhead is the measured time per stored row (p50/p95/max)', () => {
  const steps = [0, 1, 0, 2, 0, 3, 0, 10];  // now() is called twice per add: t0, then the end
  let i = 0, t = 0;
  const now = () => { t += steps[i++]; return t; };
  const rec = createPoseRecorder(now, 100);
  for (let k = 0; k < 4; k++) rec.add(reality(), null);
  const o = rec.snapshot().overheadMs;
  assert.equal(o.n, 4);
  assert.equal(o.max, 10);
  assert.equal(o.p50, 3);
});

test('summarize: percentiles of a sorted copy, empty gives nulls', () => {
  const v = Array.from({ length: 100 }, (_, k) => 100 - k);
  const s = summarize(v);
  assert.deepEqual([s.n, s.p50, s.p95, s.p99, s.max], [100, 51, 96, 100, 100]);
  assert.equal(v[0], 100, 'input not mutated');
  assert.deepEqual(summarize([]), { n: 0, p50: null, p95: null, p99: null, max: null });
});

// The snapshot of the page before the recorder existed: record off must keep exactly this shape.
const OLD_KEYS = ['schema', 'note', 'label', 'updates', 'nullUpdates', 'hitTests', 'hitMisses', 'placed', 'imageEvents', 'errors', 'timeToFirstNormalMs', 'updateIntervalMs', 'worldPoints', 'transitions', 'events'];
const OLD_NOTE = 'Counters and intervals only. No camera image and no IMU sample. Produced by the user clicking the download button. Interval unit: ms between pipeline onUpdate calls with a reality result (not engine time).';

test('snapshot with record off: same keys and note as before, no poseLog, no receipt, frameStart ignored', () => {
  const m = createMetrics(clock());
  m.start();
  m.update(reality(), 3, { videoTime: 1, frameTime: 2 });
  m.update(null, 0);
  m.setReceiptAtStart({ x: 1 });  // ignored when off
  const snap = m.snapshot();
  assert.deepEqual(Object.keys(snap), OLD_KEYS);
  assert.equal(snap.note, OLD_NOTE);
  assert.equal(snap.updates, 1);
  assert.equal(snap.nullUpdates, 1);
  assert.equal(m.recording, false);
  assert.equal(m.poseLogCounts(), null);
  assert.equal(JSON.stringify(m.snapshot({}, { leaked: true })).includes('leaked'), false);
});

test('snapshot with record on: poseLog and receipt added, note says so, null updates leave no row', () => {
  const m = createMetrics(clock(), { record: true, poseLogCap: 2 });
  m.start();
  m.setReceiptAtStart({ camera: 'start' });
  m.update(reality(), 3, { videoTime: 1, frameTime: 2 });
  m.update(null, 0, {});
  m.update(reality(), 3, { videoTime: 2, frameTime: 3 });
  m.update(reality(), 3, { videoTime: 3, frameTime: 4 });
  const snap = m.snapshot({ userAgent: 'ua' }, { camera: 'download' });
  assert.deepEqual(Object.keys(snap).filter((k) => !['poseLog', 'receipt'].includes(k)), [...OLD_KEYS.slice(0, 3), 'userAgent', ...OLD_KEYS.slice(3)]);
  assert.ok(snap.note.startsWith(OLD_NOTE) && snap.note.length > OLD_NOTE.length && snap.note.includes('record=1'));
  assert.equal(snap.poseLog.schema, 'slam-refact-xr-poselog-v3');
  assert.equal(snap.poseLog.rows.length, 2);
  assert.equal(snap.poseLog.droppedRows, 1);
  assert.equal(snap.poseLog.cap, 2);
  assert.equal(snap.poseLog.nullUpdates, 1);
  assert.ok(snap.poseLog.overheadMs.n === 2 && snap.poseLog.overheadMs.max >= 0);
  assert.deepEqual(snap.receipt, { atStart: { camera: 'start' }, atDownload: { camera: 'download' } });
  assert.equal(snap.updates, 3, 'the existing counters count every reality update, stored or dropped');
  assert.doesNotThrow(() => JSON.stringify(snap));
});

test('summarizeChunkStats: counts and timing percentiles, the large arrays are not copied', () => {
  const big = Array.from({ length: 5000 }, (_, k) => k);
  const out = summarizeChunkStats({
    stats: { stagedFrames: 7, stageMs: big, readMs: [1, 2, 3], engineMs: [], responseLag: { 0: 3 }, lastSourceFrameIndex: null, someList: [1, 2, 3] },
    limits: { maxSamples: 20000 }, error: null, warnings: ['a', 'b'],
  });
  assert.equal(out.stats.stagedFrames, 7);
  assert.equal(out.stats.stageMs.n, 5000);
  assert.equal(out.stats.stageMs.p50, 2500);
  assert.deepEqual(out.stats.readMs, { n: 3, p50: 2, p95: 3, p99: 3, max: 3 });
  assert.equal(out.stats.engineMs.n, 0);
  assert.deepEqual(out.stats.someList, { n: 3 });
  assert.deepEqual(out.stats.responseLag, { 0: 3 });
  assert.equal(out.warningsCount, 2);
  assert.ok(JSON.stringify(out).length < 1000);
  assert.equal(summarizeChunkStats(undefined), null);
});

const sha = async (text) => Array.from(new Uint8Array(await webcrypto.subtle.digest('SHA-256', new TextEncoder().encode(text))), (b) => b.toString(16).padStart(2, '0')).join('');
const files = {
  '../../deploy.json': { schema: 'slam-refact-xr-deploy-v1', engine: { js_sha256: 'aa', wasm_sha256: 'bb' }, xr_js: { sha256: 'cc' }, bundle_sha256: 'dd', files: [{ path: 'x' }] },
  '../../engine/slam-refact-engine/manifest.json': { schema: 'm', artifacts: { js: { sha256: 'ee' }, wasm: { sha256: 'ff' } } },
};
const fetchFn = async (url) => (files[url] ? { ok: true, status: 200, json: async () => files[url] } : { ok: false, status: 404, json: async () => ({}) });
const env = (over = {}) => ({
  track: { label: 'camera2 0, facing back', readyState: 'live', getSettings: () => ({ width: 1920, height: 1440, frameRate: 30, facingMode: 'environment', deviceId: 'SECRET-DEVICE', groupId: 'SECRET-GROUP', zoom: 1 }) },
  win: { innerWidth: 450, innerHeight: 930, devicePixelRatio: 2.4, XrCameraResolution: { requested: { width: 1920, height: 1440 }, granted: null }, performance: { now: () => 5 },
    __slamRefactChunk: { session: { videoWidth: 720, newReadback: { a: 1 }, fn() {} }, stats: { stagedFrames: 3, stageMs: [1, 2, 3], readMs: [], engineMs: [] }, limits: null, error: null, warnings: [] } },
  nav: { userAgent: 'UA', userAgentData: { getHighEntropyValues: async (k) => ({ model: 'SM-F741N', platformVersion: '15.0.0', asked: k }) } },
  scr: { orientation: { type: 'portrait-primary', angle: 0 }, width: 1080, height: 2640 }, canvas: { width: 1080, height: 2232 },
  fetchFn, subtle: webcrypto.subtle, nowMs: 42, ...over,
});

test('receipt: camera settings, hashed ids only, build hashes, session copied whole, stats summarized', async () => {
  const r = await buildReceipt(env());
  assert.equal(r.camera.settings.width, 1920);
  assert.equal(r.camera.settings.facingMode, 'environment');
  assert.equal(r.camera.deviceIdSha256, await sha('SECRET-DEVICE'));
  assert.equal(r.camera.groupIdSha256, await sha('SECRET-GROUP'));
  assert.equal(r.camera.label, 'camera2 0, facing back');
  assert.equal(JSON.stringify(r).includes('SECRET'), false, 'raw ids never appear');
  assert.deepEqual(r.cameraResolution, { requested: { width: 1920, height: 1440 }, granted: null });
  assert.deepEqual(r.deploy, { schema: 'slam-refact-xr-deploy-v1', engine: { js_sha256: 'aa', wasm_sha256: 'bb' }, xr_js: { sha256: 'cc' }, bundle_sha256: 'dd' });
  assert.deepEqual(r.engineManifest, { schema: 'm', artifacts: { js: { sha256: 'ee' }, wasm: { sha256: 'ff' } } });
  assert.deepEqual(r.session, { videoWidth: 720, newReadback: { a: 1 } }, 'whole object, no field list; functions drop out in the JSON copy');
  assert.equal(r.chunkStats.stats.stageMs.n, 3);
  assert.equal(r.userAgent, 'UA');
  assert.equal(r.userAgentData.model, 'SM-F741N');
  assert.deepEqual(r.screen, { orientation: { type: 'portrait-primary', angle: 0 }, width: 1080, height: 2640, innerWidth: 450, innerHeight: 930 });
  assert.equal(r.devicePixelRatio, 2.4);
  assert.deepEqual(r.canvas, { width: 1080, height: 2232 });
  assert.equal(r.capturedAtMs, 42);
  assert.doesNotThrow(() => JSON.stringify(r));
});

test('receipt: every missing part says why instead of failing the whole receipt', async () => {
  const r = await buildReceipt(env({
    track: null, subtle: null, fetchFn: async (u) => { if (u.endsWith('deploy.json')) throw new Error('offline'); return { ok: false, status: 404 }; },
    nav: { userAgent: 'UA' },
  }));
  assert.match(r.camera.unavailable, /no camera track/);
  assert.equal(r.deploy.unavailable, 'offline');
  assert.equal(r.engineManifest.unavailable, 'HTTP 404');
  assert.match(r.userAgentData.unavailable, /not available/);
  assert.equal(r.session.videoWidth, 720, 'unrelated parts still filled');
  const noHash = await buildReceipt(env({ subtle: null }));
  assert.match(noHash.camera.deviceIdSha256.unavailable, /crypto\.subtle/);
  assert.equal(JSON.stringify(noHash).includes('SECRET'), false);
  const noFetch = await buildReceipt(env({ fetchFn: null }));
  assert.match(noFetch.deploy.unavailable, /fetch/);
  const bad = await buildReceipt(env({ track: { getSettings() { throw new Error('track ended'); } } }));
  assert.equal(bad.camera.unavailable, 'track ended');
});

test('rows carry sourceFrameIndex and repeatFrame; the snapshot carries the field meanings', () => {
  const r = createPoseRecorder(() => 0, 10);
  r.add({ trackingStatus: 'NORMAL', position: { x: 0, y: 1, z: 0 }, rotation: { x: 0, y: 0, z: 0, w: 1 }, worldPoints: [] }, { videoTime: 1, frameTime: 2, repeatFrame: false }, 41);
  r.add({ trackingStatus: 'NORMAL', position: { x: 0, y: 1, z: 0 }, rotation: { x: 0, y: 0, z: 0, w: 1 }, worldPoints: [] }, { repeatFrame: true });
  const s = r.snapshot();
  assert.equal(s.rows[0].sourceFrameIndex, 41);
  assert.equal(s.rows[0].repeatFrame, false);
  assert.equal(s.rows[1].sourceFrameIndex, null);
  assert.equal(s.rows[1].repeatFrame, true);
  assert.match(s.meta.position, /NOT metres/);
  assert.match(s.meta.rotation, /\{x, y, z, w\}/);
});

test('rows keep the chunk sourceTiming scalars only (fixed keys, wrong types become null)', () => {
  const r = createPoseRecorder(() => 0, 10);
  const st = { kind: 'engine-response', fresh: true, repeated: false, sourceFrameIndex: 7, sessionEpoch: 1, sourceVideoTimeS: 0.25, sourceTimeNanos: 1e9, extra: { big: [1, 2, 3] }, engineReturnedAtMs: NaN };
  r.add({ trackingStatus: 'NORMAL', position: { x: 0, y: 1, z: 0 }, rotation: { x: 0, y: 0, z: 0, w: 1 }, worldPoints: [] }, { videoTime: 0.3 }, 7, st);
  const row = r.snapshot().rows[0];
  assert.equal(row.source.kind, 'engine-response');
  assert.equal(row.source.fresh, true);
  assert.equal(row.source.sourceFrameIndex, 7);
  assert.equal(row.source.sourceVideoTimeS, 0.25);
  assert.equal(row.source.engineReturnedAtMs, null);
  assert.equal('extra' in row.source, false);
  assert.equal(pickSourceTiming(null), null);
});

// ---- &diag=1 tracker diagnostics (A-0e) ----
const trackerDiag = (over = {}) => ({ session: 1, sourceFrameIndex: 41, mapTrackerState: 1, frameTrackerOk: true, ok: true, lossState: 0, lostFrames: 0, frames: 12,
  mapReset: false, keyframeCreated: true, numKeyframes: 3, groundMatches: 0, keypoints: 500, pointsTriangulated: 120, pointsGround: 40, pointsSky: 0, pointsDepth: 0, pointsCloud: 0,
  inputFound: true, motionEvents: 2, gyroPeakDegS: 91.5, linAccPeakMs2: 0.4, totalResponses: 40, totalMapResets: 1, totalKeyframesCreated: 3, totalMapTrackerNotOk: 2, totalInputMissing: 0, ...over });

test('tracker diag off: rows, schema, meta and note are exactly the v3 ones even when a tracker object is passed', () => {
  const off = createPoseRecorder(clock(), 10);
  off.add(reality(), { videoTime: 1 }, 41, null, trackerDiag());
  const s = off.snapshot();
  assert.equal(s.schema, 'slam-refact-xr-poselog-v3');
  assert.equal('tracker' in s.rows[0], false);
  assert.equal('tracker' in s.meta, false);
  const m = createMetrics(clock(), { record: true });
  m.update(reality(), 3, { videoTime: 1 }, 41, null, trackerDiag());
  const snap = m.snapshot();
  assert.equal(snap.poseLog.schema, 'slam-refact-xr-poselog-v3');
  assert.equal('tracker' in snap.poseLog.rows[0], false);
  assert.doesNotMatch(snap.note, /diag=1/);
  // trackerDiag without record records nothing at all
  const none = createMetrics(clock(), { record: false, trackerDiag: true });
  none.update(reality(), 3, { videoTime: 1 }, 41, null, trackerDiag());
  assert.equal('poseLog' in none.snapshot(), false);
});

test('tracker diag on: each row keeps the fixed scalar keys, schema v4, meta and note say what they are', () => {
  const m = createMetrics(clock(), { record: true, trackerDiag: true });
  m.update(reality(), 3, { videoTime: 1 }, 41, null, trackerDiag({ extra: { big: [1, 2] }, gyroPeakDegS: NaN, mapReset: 'yes' }));
  m.update(reality(), 3, { videoTime: 2 }, 42, null, null);  // the chunk had no value yet
  const snap = m.snapshot();
  assert.equal(snap.poseLog.schema, 'slam-refact-xr-poselog-v4');
  const [a, b] = snap.poseLog.rows;
  assert.deepEqual(Object.keys(a), ['tPerfMs', 'videoTimeS', 'frameTimeS', 'repeatFrame', 'sourceFrameIndex', 'source', 'status', 'reason', 'position', 'rotation', 'worldPoints', 'tracker']);
  assert.deepEqual(Object.keys(a.tracker), [...TRACKER_DIAG_KEYS]);
  assert.equal(a.tracker.sourceFrameIndex, 41);
  assert.equal(a.tracker.keyframeCreated, true);
  assert.equal(a.tracker.totalMapResets, 1);
  assert.equal(a.tracker.motionEvents, 2);
  assert.equal(a.tracker.gyroPeakDegS, null, 'a non-finite number becomes null');
  assert.equal(a.tracker.mapReset, null, 'a wrong type becomes null, never a string');
  assert.equal('extra' in a.tracker, false);
  assert.equal(b.tracker, null);
  assert.match(snap.poseLog.meta.tracker, /stored once per tracked frame/);
  assert.match(snap.poseLog.meta.trackerNull, /no new tracked frame/);
  assert.match(snap.poseLog.meta.trackerSession, /restarts sourceFrameIndex/);
  assert.match(snap.note, /diag=1/);
  assert.match(snap.note, /no raw IMU sample is stored/);
  assert.equal(pickTrackerDiag(null), null);
  assert.equal(pickTrackerDiag('x'), null);
});

test('tracker diag on: one stored copy per tracked frame; the same frame sampled again is null; a new session with the same index is new', () => {
  const m = createMetrics(clock(), { record: true, trackerDiag: true });
  const seq = [trackerDiag({ sourceFrameIndex: 5 }), trackerDiag({ sourceFrameIndex: 5 }), trackerDiag({ sourceFrameIndex: 6, mapReset: true }),
    null, trackerDiag({ sourceFrameIndex: 6, mapReset: true }), trackerDiag({ session: 2, sourceFrameIndex: 6 }), trackerDiag({ session: 2, sourceFrameIndex: 0 })];
  for (const td of seq) m.update(reality(), 3, { videoTime: 1 }, 41, null, td);
  const got = m.snapshot().poseLog.rows.map((r) => (r.tracker ? `${r.tracker.session}:${r.tracker.sourceFrameIndex}` : null));
  assert.deepEqual(got, ['1:5', null, '1:6', null, null, '2:6', '2:0']);
});

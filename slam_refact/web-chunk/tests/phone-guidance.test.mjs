// samples/phone/guidance.js: tracking state -> hint. Every state the page can show has a title and an action the user can take.
import test from 'node:test';
import assert from 'node:assert/strict';
import { trackingGuide, tapOutcome, SLOW_INIT_MS, POINTS_FULL_BAR, MAP_LOST_FRAMES, STATUS_LABEL, createMapLostFilter } from '../samples/phone/guidance.js';
import { readEngineConfigReadback, effectiveMode } from '../samples/phone/metrics.js';

const g = (o) => trackingGuide({ phase: 'running', status: 'LIMITED', reason: 'UNSPECIFIED', ...o });

test('NORMAL is ready, with the object count once something is placed', () => {
  assert.deepEqual([g({ status: 'NORMAL' }).tone, g({ status: 'NORMAL' }).pill, g({ status: 'NORMAL' }).progress], ['ok', 'normal', 1]);
  assert.match(g({ status: 'NORMAL', objectCount: 3 }).title, /3개/);
});

test('reasons turn into different, actionable hints', () => {
  assert.match(g({ reason: 'EXCESSIVE_MOTION' }).title, /빨리/);
  assert.match(g({ reason: 'INSUFFICIENT_FEATURES' }).body, /밝은/);
  assert.match(g({ reason: 'INITIALIZING' }).body, /천천히 옮기면서/);
  assert.equal(g({ reason: 'EXCESSIVE_MOTION' }).tone, 'warn');
});

test('progress is the point count against the display scale, capped below 1 until NORMAL; unknown while no points are visible', () => {
  assert.equal(g({ points: 0 }).progress, null);
  assert.equal(g({ points: POINTS_FULL_BAR / 2 }).progress, 0.5);
  assert.equal(g({ points: 10 * POINTS_FULL_BAR }).progress, 0.95);
});

test('a long wait adds the recenter hint, a short one does not', () => {
  assert.doesNotMatch(g({ notTrackingMs: SLOW_INIT_MS - 1 }).body, /원점 다시 잡기/);
  assert.match(g({ notTrackingMs: SLOW_INIT_MS }).body, /원점 다시 잡기/);
});

test('landscape asks for portrait; error and stopped phases have their own text', () => {
  assert.match(g({ landscape: true }).title, /세로/);
  assert.equal(trackingGuide({ phase: 'error' }).pill, 'error');
  assert.equal(trackingGuide({ phase: 'stopped' }).pill, 'stopped');
});

test('tap results: placed counts objects, a miss says what to do', () => {
  assert.match(tapOutcome(true, { objectCount: 2 }).title, /2개/);
  const miss = tapOutcome(false);
  assert.equal(miss.tone, 'warn');
  assert.match(miss.body, /천천히/);
});

// Map-lost hint: scripted tracker states (one entry per tracked frame; chunk diag.trackerDiag shape) -> the guidance text the page would show.
const frames = (flags, session = 1) => flags.map((f, i) => ({ session, sourceFrameIndex: i, mapTrackerState: f ? 1 : 0 }));
const run = (tds, { optIn = true, status = 'NORMAL' } = {}) => {
  const f = createMapLostFilter();
  return tds.map((td) => trackingGuide({ phase: 'running', status, reason: 'UNSPECIFIED', mapLost: optIn && f.update(td) }));
};

test('map-lost hint: shown from the 3rd consecutive lost tracked frame, not before; cleared on the first tracked frame', () => {
  assert.equal(MAP_LOST_FRAMES, 3);
  const out = run(frames([1, 1, 0, 0, 0, 0, 1, 1]));
  assert.deepEqual(out.map((o) => o.title === '주변을 다시 찾는 중'), [false, false, false, false, true, true, false, false]);
  const hint = out[4];
  assert.deepEqual([hint.tone, hint.pill, hint.title, hint.body], ['warn', 'mapLost', '주변을 다시 찾는 중', '천천히 움직이거나 처음 보던 곳을 다시 비춰 주세요.']);
  assert.notEqual(STATUS_LABEL[hint.pill], STATUS_LABEL.normal);
  assert.equal(STATUS_LABEL[hint.pill], hint.title, 'pill and title use the same words');
  assert.equal(out[1].pill, 'normal');
  assert.equal(out[7].title, '준비됐어요');
});

test('map-lost hint: a single lost frame does not flash; lost run restarts after recovery', () => {
  assert.ok(run(frames([1, 0, 1, 0, 0, 1, 0, 0])).every((o) => o.pill === 'normal'));
});

test('map-lost hint: repeated samples of the same tracked frame count once', () => {
  const [a, b, c, d] = frames([1, 0, 0, 0]);
  assert.deepEqual(run([a, b, b, b, b, c]).map((o) => o.pill), ['normal', 'normal', 'normal', 'normal', 'normal', 'normal']);
  assert.equal(run([a, b, c, d]).pop().pill, 'mapLost');
});

test('map-lost hint: never before the first tracked frame of a session (start-up), and a new session starts over', () => {
  assert.ok(run(frames([0, 0, 0, 0, 0, 0])).every((o) => o.pill === 'normal'));
  const f = createMapLostFilter();
  for (const td of frames([1, 0, 0, 0], 1)) f.update(td);
  assert.equal(f.update({ session: 1, sourceFrameIndex: 4, mapTrackerState: 0 }), true);
  assert.equal(f.update({ session: 2, sourceFrameIndex: 0, mapTrackerState: 0 }), false);
  assert.equal(f.update({ session: 2, sourceFrameIndex: 3, mapTrackerState: 0 }), false);
});

test('map-lost hint: unavailable tracker state (null / undefined / no mapTrackerState) -> no hint and no guess', () => {
  assert.ok(run([null, undefined, {}, { session: 1, sourceFrameIndex: 0 }]).every((o) => o.pill === 'normal'));
  const f = createMapLostFilter();
  for (const td of frames([1, 0, 0, 0])) f.update(td);
  assert.equal(f.update(null), false);
  assert.equal(f.update({ session: 1, sourceFrameIndex: 9, mapTrackerState: 0 }), false);  // reset: needs a tracked map again
});

test('map-lost hint: default mode (no keepmap, hint logic off) keeps the old text; only the NORMAL state is affected', () => {
  const lost = frames([1, 0, 0, 0, 0]);
  const parity = run(lost, { optIn: false });
  assert.ok(parity.every((o) => o.pill === 'normal' && o.title === '준비됐어요' && o.progress === 1));
  assert.deepEqual(parity[4], trackingGuide({ phase: 'running', status: 'NORMAL', reason: 'UNSPECIFIED' }));
  assert.deepEqual(trackingGuide({ phase: 'running', status: 'LIMITED', reason: 'INITIALIZING', mapLost: true }), g({ reason: 'INITIALIZING' }));
  assert.equal(trackingGuide({ phase: 'running', status: 'NORMAL', reason: 'UNSPECIFIED', objectCount: 2, mapLost: false }).title, '물체 2개를 놓았어요');
});

test('receipt read-back: every sent flag is reported generically; active only when read back equal and not ignored', () => {
  const chunk = (eff) => ({ session: { configurationReadback: { effectiveConfig: eff } } });
  const sent = { enabled: true, k1KeepMap: true, k1FastExtend: true };
  assert.equal(readEngineConfigReadback(null).available, false);
  assert.equal(readEngineConfigReadback({ session: { configurationReadback: {} } }).available, false);
  assert.deepEqual(readEngineConfigReadback(chunk({ available: false, reason: 'engine-worker' }), sent), { available: false, reason: 'engine-worker' });
  const ok = readEngineConfigReadback(chunk({ accuracy: { enabled: true, k1KeepMap: true, k1FastExtend: true }, ignoredKeys: [] }), sent);
  assert.deepEqual([ok.available, ok.k1KeepMap, ok.k1KeepMapActive, ok.allProductFlagsActive, ok.ignoredKeys], [true, true, true, true, []]);
  assert.deepEqual(ok.flags.k1FastExtend, { sent: true, effective: true, ignored: false });
  const ign = readEngineConfigReadback(chunk({ accuracy: { enabled: true, k1KeepMap: true, k1FastExtend: false }, ignoredKeys: ['accuracy.k1FastExtend'] }), sent);
  assert.deepEqual([ign.k1KeepMapActive, ign.allProductFlagsActive, ign.flags.k1FastExtend], [true, false, { sent: true, effective: false, ignored: true }]);
  assert.equal(readEngineConfigReadback(chunk({ accuracy: { k1KeepMap: true }, ignoredKeys: ['accuracy.k1KeepMap'] }), { k1KeepMap: true }).k1KeepMapActive, false);
  assert.equal(readEngineConfigReadback(chunk({ accuracy: { k1KeepMap: true } }), { k1KeepMap: true }).allProductFlagsActive, false);
  assert.deepEqual(readEngineConfigReadback(chunk({ accuracy: { enabled: false, k1KeepMap: false }, ignoredKeys: [] }), null).flags, {});
});

test('engineModeEffective: the requested name only when every sent flag is confirmed effective; default when nothing was sent', () => {
  const sent = { enabled: true, k1KeepMap: true };
  const rb = (eff) => readEngineConfigReadback({ session: { configurationReadback: { effectiveConfig: eff } } }, sent);
  assert.equal(effectiveMode({ mode: 'default', accuracy: null }, readEngineConfigReadback(null)), 'default');
  assert.equal(effectiveMode({ mode: 'K1', accuracy: sent }, rb({ accuracy: { enabled: true, k1KeepMap: true }, ignoredKeys: [] })), 'K1');
  assert.equal(effectiveMode({ mode: 'K1', accuracy: sent }, rb({ accuracy: { enabled: true, k1KeepMap: false }, ignoredKeys: ['accuracy.k1KeepMap'] })), 'requested-not-effective');
  assert.equal(effectiveMode({ mode: 'K1', accuracy: sent }, rb({ accuracy: { enabled: true, k1KeepMap: true }, ignoredKeys: ['accuracy.k1KeepMap'] })), 'requested-not-effective');
  assert.equal(effectiveMode({ mode: 'K1', accuracy: sent }, rb({ available: false, reason: 'engine-worker' })), 'requested-not-effective');
  assert.equal(effectiveMode({ mode: 'K1', accuracy: sent }, readEngineConfigReadback(null, sent)), 'requested-not-effective');
});

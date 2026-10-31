// samples/phone/guidance.js: tracking state -> hint. Every state the page can show has a title and an action the user can take.
import test from 'node:test';
import assert from 'node:assert/strict';
import { trackingGuide, tapOutcome, SLOW_INIT_MS, POINTS_FULL_BAR } from '../samples/phone/guidance.js';

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

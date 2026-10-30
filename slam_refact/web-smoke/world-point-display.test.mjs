import test from 'node:test';
import assert from 'node:assert/strict';
import { worldPointDisplayEvidence } from './world-point-display.mjs';

const frame = (keyframes, shown, fresh = true) => ({
  fresh, tracker: { numKeyframes: keyframes }, worldPointsShown: shown,
});

test('a displayed mature map remains observed when a later reset clears the final snapshot', () => {
  const evidence = worldPointDisplayEvidence([frame(2, 7), frame(1, 0)]);
  assert.equal(evidence.passed, true);
  assert.equal(evidence.matureFrames, 1);
  assert.equal(evidence.displayedFrames, 1);
  assert.equal(evidence.maxDisplayedPoints, 7);
});

test('a mature map that never displayed non-padding points fails', () => {
  assert.equal(worldPointDisplayEvidence([frame(1, 0), frame(2, 0), frame(1, 0)]).passed, false);
});

test('keyframes and displayed points from different frames cannot establish map display', () => {
  assert.equal(worldPointDisplayEvidence([frame(2, 0), frame(1, 7)]).passed, false);
});

test('raw list entries do not establish displayed non-padding points', () => {
  assert.equal(worldPointDisplayEvidence([{ ...frame(2, 0), worldPoints: 200 }]).passed, false);
});

test('a repeated pose does not establish a fresh point-display observation', () => {
  assert.equal(worldPointDisplayEvidence([frame(2, 7, false)]).passed, false);
});

test('a run without a mature map has no point-display requirement', () => {
  assert.equal(worldPointDisplayEvidence([frame(0, 0), frame(1, 0)]).passed, true);
});

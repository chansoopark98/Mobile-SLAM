import test from 'node:test';
import assert from 'node:assert/strict';
import { summarizeResults, summarizeLowFeatureResults } from '../../scripts/dev/replay-android-input.mjs';
import { poseStateSummary } from '../../scripts/dev/synthetic-browser.mjs';

test('Prediction and plane statistics remain separate from unchanged visual result/accuracy populations', () => {
    const rows = [
        { statusCode: 2, initialized: true, poseFresh: true, poseValid: true, poseTimestamp: 1, engineEpoch: 1, reason: 'tracking', pose: [1,0,0,0,0,1,0,0,0,0,1,1,0,0,0,1] },
        { statusCode: 3, initialized: true, poseFresh: false, poseValid: false, poseTimestamp: null, engineEpoch: 1, reason: 'empty_features', pose: null },
    ];
    const baselineSummary = summarizeResults(rows), baselineStates = poseStateSummary(rows);
    rows[0].groundPlane = { state: 'observed' };
    rows[1].inertialPrediction = { pose: rows[0].pose };
    rows[1].groundPlane = { state: 'retained' };
    assert.deepEqual(summarizeResults(rows), baselineSummary);
    assert.deepEqual(poseStateSummary(rows), baselineStates);
    assert.deepEqual(summarizeLowFeatureResults(rows), {
        visualFresh: 1, predictionAvailable: 1, predictionDisplayed: null, planeCandidate: 2, planeObserved: 1,
        planeRetained: 1, planeDisplayed: null, displayCountUnit: 'not-rendered', contributesToVisualAccuracy: false,
    });
    const live = summarizeLowFeatureResults(rows, [{ prediction: false, groundPlane: true }, { prediction: true, groundPlane: true }]);
    assert.equal(live.predictionDisplayed, 1); assert.equal(live.planeDisplayed, 2);
    assert.equal(live.visualFresh, 1); assert.equal(live.contributesToVisualAccuracy, false);
});

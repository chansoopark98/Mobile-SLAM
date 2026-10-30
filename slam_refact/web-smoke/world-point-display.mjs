// Evidence from the measurement page records; the final viewer snapshot is kept by the runner.
export function worldPointDisplayEvidence(records) {
  const mature = records.filter((r) => Number.isInteger(r.tracker?.numKeyframes) && r.tracker.numKeyframes >= 2);
  const displayed = mature.filter((r) => r.fresh === true && Number.isInteger(r.worldPointsShown) && r.worldPointsShown > 0);
  return {
    matureFrames: mature.length,
    displayedFrames: displayed.length,
    maxDisplayedPoints: Math.max(0, ...displayed.map((r) => r.worldPointsShown)),
    passed: displayed.length > 0 || mature.length === 0,
  };
}

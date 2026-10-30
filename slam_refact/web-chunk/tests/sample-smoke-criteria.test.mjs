// Unit tests of the US-010 smoke criteria (sample-smoke-criteria.mjs) and the timing summary on synthetic data: a passing report passes,
// and each single-field change makes exactly the registered check fail (the criteria code has discriminating power).
import test from 'node:test';
import assert from 'node:assert/strict';
import { evaluate, WASM_SHA256, TOL } from './sample-smoke-criteria.mjs';
import { timingSummary, stats } from './sample-timing.mjs';

const make = () => {
  const updates = [], hits = [];
  for (let n = 1; n <= 600; n++) {
    const repeat = n % 3 === 0 ? 0 : 1;
    updates.push({ n, wall: 1000 + 16.7 * n, repeat, status: 'NORMAL', nwp: n > 20 ? 100 : 0, nreal: n > 20 ? 100 : 0, videoTime: 1000 + n * 0.02 - 0.0, pos: [n * 0.01, 2, 0], rot: [1, 0, 0, 0],
      cam: [n * 0.01, 2, 0], camq: [1, 0, 0, 0], posErr: 0, rotErr: 0, projErr: 0, paired: 1, xrGpuMs: 1, xrCpuMs: 4 });
    hits.push({ n, nwp: n > 5 ? 100 : 0, nreal: n > 20 ? 100 : 0, camDist: 0, len: 1, esLen: 0, keys: 'distance,position,rotation,type', type: 'FEATURE_POINT', posFinite: true, rotFinite: true, distFinite: true, distance: n > 20 ? 3 : 0, distDiff: 0 });
  }
  return {
    kind: 'threejs', pageErrors: [], consoleErrors: 0, blockedExternal: 0, requestsOffOrigin: [],
    requests: [{ url: '/engine/xr-slam.js', status: 200 }, { url: '/engine/slam-refact-engine/slam_refact_engine.wasm', status: 200 }],
    chunkDiag: { engine: { matchesManifest: true, glueReplaced: false, wasmSha256: WASM_SHA256 } }, input: { errors: [] },
    sample: { ready: true, kind: 'threejs', objects: { camera: true, cube: true }, errors: [], updates, hits,
      appHitCalls: [{ result: [{ position: { x: 1, y: 0, z: -2 }, distance: 3 }] }],
      placement: { cubePosition: [1, 0, -2], lastAppHit: { result: [{ position: { x: 1, y: 0, z: -2 }, distance: 3 }] } }, post: { updates: 300, maxCubeDrift: 0, maxCamDisp: 1.5 }, cubeTransitions: 1, taps: 1 },
    screenshots: { pre: { magenta: 0, nonBlackFraction: 0.9 }, post: { magenta: 500 } },
    sensors: { epochWallMs: 1000 * 1000, packets: Array.from({ length: 1500 }, (_, i) => [i * 0.01, [0, 9.8, 0]]) },
  };
};
const failedOf = (r) => Object.entries(r.checks).filter(([, c]) => !c.pass).map(([k]) => k);

test('a consistent report passes every check', () => {
  const r = evaluate(make());
  assert.deepEqual(failedOf(r), []);
  assert.equal(r.pass, true);
});

const cases = [
  ['page error', (d) => { d.pageErrors.push({}); }, ['S1']],
  ['external request', (d) => { d.requestsOffOrigin.push('http://x/'); }, ['S2']],
  ['engine bytes differ', (d) => { d.chunkDiag.engine.wasmSha256 = '0'.repeat(64); }, ['S3']],
  ['no engine diag (fake chunk)', (d) => { d.chunkDiag = null; }, ['S3']],
  ['too few fresh updates', (d) => { d.sample.updates.forEach((u, i) => { u.repeat = i % 40 === 0 ? 0 : 1; }); }, ['S5', 'S14']],
  ['pose does not move', (d) => { d.sample.updates.forEach((u) => { u.pos = [0, 2, 0]; }); }, ['S6']],
  ['position off by 2e-6', (d) => { d.sample.updates[100].posErr = 2e-6; }, ['S7']],
  ['rotation off by 2e-6', (d) => { d.sample.updates[100].rotErr = 2e-6; }, ['S8']],
  ['projection off by 2e-6', (d) => { d.sample.updates[100].projErr = 2e-6; }, ['S9']],
  ['hit without distance', (d) => { d.sample.hits.forEach((h) => { h.keys = 'position,rotation,type'; }); }, ['S10']],
  ['hit distance inconsistent', (d) => { d.sample.hits[300].distDiff = 0.01; }, ['S10']],
  ['padding-only update with a non-zero distance', (d) => { d.sample.hits[10].distance = 1; d.sample.hits[10].camDist = 1; }, ['S10b']],
  ['padding-only update that does not return the camera position', (d) => { d.sample.hits[10].camDist = 0.5; }, ['S10b']],
  ['cube shown twice', (d) => { d.sample.cubeTransitions = 2; }, ['S11']],
  ['rotation x/z swapped (chunk side)', (d) => { d.sample.updates.forEach((u) => { u.rot = [0.9, 0.3, 0, 0.1]; }); }, ['S14']],
  ['too few real-point updates for S14', (d) => { d.sample.updates.forEach((u, i) => { if (i > 60) u.nreal = 0; }); }, ['S14']],
  ['six sample hit calls', (d) => { d.sample.appHitCalls.unshift(...Array(5).fill({ result: [] })); }, ['S11']],
  ['an earlier usable call did not place', (d) => { d.sample.appHitCalls.unshift({ result: [{ position: { x: 0, y: 0, z: 0 }, distance: 2 }] }); }, ['S11']],
  ['placed on a distance-0 hit', (d) => { d.sample.placement.lastAppHit.result[0].distance = 0; }, ['S11']],
  ['cube placed off the hit', (d) => { d.sample.placement.cubePosition = [1, 0, -2.001]; }, ['S11']],
  ['cube drifts', (d) => { d.sample.post.maxCubeDrift = 1e-6; }, ['S12']],
  ['camera did not move after placement', (d) => { d.sample.post.maxCamDisp = 0.1; }, ['S12']],
  ['no cube pixels', (d) => { d.screenshots.post.magenta = 99; }, ['S13']],
  ['magenta before the tap', (d) => { d.screenshots.pre.magenta = 1; }, ['S13']],
  ['black background', (d) => { d.screenshots.pre.nonBlackFraction = 0.4; }, ['S13']],
];
for (const [name, mutate, expect] of cases) {
  test(`criteria detect: ${name}`, () => {
    const d = make();
    mutate(d);
    assert.deepEqual(failedOf(evaluate(d)), expect);
  });
}

test('refusals before the placing call are fine (distance 0 and empty)', () => {
  const d = make();
  d.sample.appHitCalls.unshift({ result: [{ position: { x: 0, y: 0, z: 0 }, distance: 0 }] }, { result: [] });
  assert.deepEqual(failedOf(evaluate(d)), []);
});

test('padding-only updates are not in the S10 population', () => {
  const d = make();
  const r = evaluate(d);
  assert.equal(r.checks.S10.detail.updatesWithRealPoints, 580);
  assert.equal(r.checks.S10b.detail.paddingOnlyUpdates, 15);
});

test('tolerances are the registered ones', () => {
  assert.equal(TOL.pos, 1e-6); assert.equal(TOL.rot, 1e-6); assert.equal(TOL.proj, 1e-6);
  assert.equal(TOL.hitDist, 1e-3); assert.equal(TOL.cubePlace, 1e-6); assert.equal(TOL.cubeDrift, 1e-9);
});

test('timing summary splits pre / post at the first world points', () => {
  const d = make();
  d.sample.rafTimes = Array.from({ length: 700 }, (_, i) => 1000 + 16.7 * i);
  d.sample.longtasks = [];
  d.sample.firstReal = { n: 21, wall: d.sample.updates[20].wall, chunkProcessed: 7, chunkStaged: 7 };
  const mk = (n, v) => Array.from({ length: n }, () => v);
  const t = timingSummary({ sample: d.sample, input: { frames: mk(300, 0) }, chunkDiag: { stats: { stageMs: mk(30, 1), readMs: mk(30, 2), engineMs: mk(30, 3), stagedFrames: 30, processedFrames: 30 } } });
  assert.equal(t.pre.freshUpdates + t.post.freshUpdates, t.all.freshUpdates);
  assert.equal(t.pre.engineMs.n, 7); assert.equal(t.post.engineMs.n, 23);
  assert.equal(t.post.mainThreadChunkMs.p50, 6);
  assert.equal(stats([1, 2, 3, 4]).p50, 2);
});

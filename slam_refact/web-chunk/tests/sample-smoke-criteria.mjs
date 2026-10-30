// Pass criteria S1..S14 (with S10b) of the US-010 sample smoke as registered in build/slam-refact-full-20261019/preregistration/us010-samples.v3.md
// (v3 changes S10's population, adds S10b and S14, counts S11's cube transitions; a change here is a new registration version, never an edit).
// S7, S8, S9, S11.placeErr and S12.maxCubeDrift compare a copy with its source (probe.js, v3 section 1.4): they prove the comparator and the XR8 / sample
// wiring, not the chunk's pose. S14 is the criterion that carries chunk information (orientation against the recorded gravity direction).
// Pure functions over the data the harness collected (window.__xrSample of samples/common/probe.js, the page events, two screenshots).
export const TOL = { pos: 1e-6, rot: 1e-6, proj: 1e-6, hitDist: 1e-3, cubePlace: 1e-6, cubeDrift: 1e-9 };
export const GRAVITY = { maxAngleDeg: 10, fraction: 0.95, minUpdates: 100, windowS: 0.25 };
export const MIN = { fresh: 200, updates: 500, bboxDiag: 1.0, normalFraction: 0.9, pairs: 200, hits: 100, camDisp: 0.3, magenta: 100, nonBlack: 0.5 };
export const WASM_SHA256 = 'c03977a1cd3e05db9c6f135773c2eda5851f1042b4603f3f60332f8afc346992';
const HIT_KEYS = 'distance,position,rotation,type';

const check = (pass, detail) => ({ pass: !!pass, detail });
const max = (xs) => xs.length ? Math.max(...xs) : null;

// report: { kind, pageErrors: [], consoleErrors: number, blockedExternal: number, requestsOffOrigin: [], requests: [{url,status}],
//   chunkDiag: {engine}|null, input: window.__input, sample: window.__xrSample, screenshots: {pre, post} }
export function evaluate(report) {
  const s = report.sample ?? {};
  const updates = s.updates ?? [];
  const fresh = updates.filter((u) => !u.repeat);
  const paired = updates.filter((u) => u.paired);
  const probeErrors = (s.errors?.length ?? 0) + (report.input?.errors?.length ?? 0);
  const eng = report.chunkDiag?.engine ?? null;
  const reqOk = (url) => report.requests?.some((r) => r.url === url && r.status === 200);
  const checks = {};

  checks.S1 = check(report.pageErrors.length === 0 && report.consoleErrors === 0 && probeErrors === 0,
    { pageErrors: report.pageErrors.length, consoleErrors: report.consoleErrors, probeErrors });
  checks.S2 = check(report.blockedExternal === 0 && report.requestsOffOrigin.length === 0,
    { blockedExternal: report.blockedExternal, requestsOffOrigin: report.requestsOffOrigin.length });
  checks.S3 = check(reqOk('/engine/xr-slam.js') && reqOk('/engine/slam-refact-engine/slam_refact_engine.wasm')
    && eng?.matchesManifest === true && eng?.glueReplaced === false && eng?.wasmSha256 === WASM_SHA256,
    { chunk200: reqOk('/engine/xr-slam.js'), wasm200: reqOk('/engine/slam-refact-engine/slam_refact_engine.wasm'), matchesManifest: eng?.matchesManifest ?? null,
      glueReplaced: eng?.glueReplaced ?? null, wasmSha256: eng?.wasmSha256 ?? null });
  checks.S4 = check(s.ready === true && s.kind === report.kind && s.objects?.camera === true && s.objects?.cube === true,
    { ready: s.ready ?? null, kind: s.kind ?? null, objects: s.objects ?? null });
  checks.S5 = check(fresh.length >= MIN.fresh && updates.length >= MIN.updates, { freshUpdates: fresh.length, realityUpdates: updates.length });

  const P = fresh.map((u) => u.pos);
  const diag = P.length ? Math.hypot(...[0, 1, 2].map((k) => Math.max(...P.map((p) => p[k])) - Math.min(...P.map((p) => p[k])))) : 0;
  const normalFraction = fresh.length ? fresh.filter((u) => u.status === 'NORMAL').length / fresh.length : 0;
  checks.S6 = check(diag >= MIN.bboxDiag && normalFraction >= MIN.normalFraction, { bboxDiagonal: diag, normalFraction });

  const posMax = max(paired.map((u) => u.posErr)), rotMax = max(paired.map((u) => u.rotErr)), projMax = max(paired.map((u) => u.projErr));
  checks.S7 = check(paired.length >= MIN.pairs && posMax <= TOL.pos, { pairs: paired.length, maxPosErr: posMax });
  checks.S8 = check(paired.length >= MIN.pairs && rotMax <= TOL.rot, { pairs: paired.length, maxRotErr: rotMax });
  checks.S9 = check(paired.length >= MIN.pairs && projMax <= TOL.proj, { pairs: paired.length, maxProjErr: projMax });

  const withPoints = (s.hits ?? []).filter((h) => h.nreal >= 1);
  const badHit = withPoints.filter((h) => !(h.len === 1 && h.keys === HIT_KEYS && h.type === 'FEATURE_POINT' && h.posFinite === true
    && h.rotFinite === true && h.distFinite === true && h.distDiff !== null && h.distDiff <= TOL.hitDist && h.esLen === 0));
  checks.S10 = check(withPoints.length >= MIN.hits && badHit.length === 0,
    { updatesWithRealPoints: withPoints.length, paddingOnlyUpdates: (s.hits ?? []).filter((h) => h.nwp >= 1 && h.nreal === 0).length,
    distance0AmongReal: withPoints.filter((h) => h.distance === 0).length, badHits: badHit.length, firstBad: badHit[0] ?? null, maxDistDiff: max(withPoints.map((h) => h.distDiff ?? Infinity)) });

  // S10b: padding-only updates (nwp >= 1, nreal = 0): the hit is the camera position with distance 0
  const paddingOnly = (s.hits ?? []).filter((h) => h.nwp >= 1 && h.nreal === 0);
  const badPadding = paddingOnly.filter((h) => !(h.len === 1 && h.distance === 0 && h.camDist !== null && h.camDist <= 1e-6));
  checks.S10b = check(badPadding.length === 0, { paddingOnlyUpdates: paddingOnly.length, bad: badPadding.length, firstBad: badPadding[0] ?? null });

  // S11 (v3): cube visible exactly once (transitions counted); 1..5 sample hitTest calls; the call that placed it had length 1 and distance > 0 and its position is the cube's;
  // the calls before it (if any) returned nothing usable (length 0 or distance 0).
  const pl = s.placement ?? null;
  const calls = s.appHitCalls ?? [];
  const hit = pl?.lastAppHit?.result?.[0] ?? null;
  const placeErr = pl && hit ? Math.hypot(pl.cubePosition[0] - hit.position.x, pl.cubePosition[1] - hit.position.y, pl.cubePosition[2] - hit.position.z) : null;
  const before = calls.slice(0, Math.max(0, calls.length - 1));
  const usableBefore = before.filter((c) => c.result?.length === 1 && c.result[0].distance > 0).length;
  checks.S11 = check(pl && calls.length >= 1 && calls.length <= 5 && pl.lastAppHit?.result?.length === 1 && hit.distance > 0 && placeErr !== null
    && placeErr <= TOL.cubePlace && usableBefore === 0 && s.cubeTransitions === 1,
  { placed: !!pl, cubeTransitions: s.cubeTransitions ?? null, taps: s.taps ?? null, appHitCalls: calls.length, hitLength: pl?.lastAppHit?.result?.length ?? null, hitDistance: hit?.distance ?? null, placeErr, usableBefore,
    refusedBefore: before.length });
  checks.S12 = check(pl && s.post && s.post.maxCubeDrift <= TOL.cubeDrift && s.post.maxCamDisp >= MIN.camDisp,
    { maxCubeDrift: s.post?.maxCubeDrift ?? null, maxCamDisp: s.post?.maxCamDisp ?? null, updatesAfterPlacement: s.post?.updates ?? 0 });
  const pre = report.screenshots?.pre ?? null, post = report.screenshots?.post ?? null;
  checks.S13 = check(pre && post && pre.magenta === 0 && pre.nonBlackFraction >= MIN.nonBlack && post.magenta >= MIN.magenta,
    { preMagenta: pre?.magenta ?? null, preNonBlackFraction: pre?.nonBlackFraction ?? null, postMagenta: post?.magenta ?? null });

  // S14: orientation against the recorded gravity direction (independent of the chunk): angle between R(reality.rotation)^T (0,1,0) and the mean
  // accelerationIncludingGravity direction within +-windowS of the update's physical time. packets: [[t, [ax, ay, az]]].
  const packets = report.sensors?.packets ?? [], epochS = (report.sensors?.epochWallMs ?? 0) / 1000;
  const angles = [];
  for (const u of fresh.filter((x) => x.nreal >= 1)) {
    const t = u.videoTime - epochS;
    let n = 0, gx = 0, gy = 0, gz = 0;
    for (const [ts, a] of packets) if (Math.abs(ts - t) <= GRAVITY.windowS) { gx += a[0]; gy += a[1]; gz += a[2]; n++; }
    if (!n) continue;
    const gn = Math.hypot(gx, gy, gz);
    const [w, x, y, z] = u.rot;
    const up = [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)];  // R^T (0,1,0): the second row of R
    const d = (gx * up[0] + gy * up[1] + gz * up[2]) / gn;
    angles.push(Math.acos(Math.max(-1, Math.min(1, d))) * 180 / Math.PI);
  }
  const within = angles.filter((a) => a <= GRAVITY.maxAngleDeg).length;
  const sorted = angles.slice().sort((a, b) => a - b);
  checks.S14 = check(angles.length >= GRAVITY.minUpdates && within / angles.length >= GRAVITY.fraction,
    { updates: angles.length, fractionWithin10deg: angles.length ? within / angles.length : null, medianDeg: sorted[Math.floor(sorted.length / 2)] ?? null,
      p90Deg: sorted[Math.floor(sorted.length * 0.9)] ?? null, maxDeg: sorted.at(-1) ?? null });

  return { pass: Object.values(checks).every((c) => c.pass), checks };
}

// Negative controls (registered expectation): the checks that must FAIL for each control.
export const EXPECTED_FAIL = {
  N0: ['S3', 'S6', 'S10', 'S11', 'S13'],
  M1: ['S7', 'S8'],
  M2: ['S10'],
  M3: ['S10', 'S11', 'S13'],
  M4: ['S6', 'S12'],
  M5: ['S14'],  // S7-S9 must stay PASS (they copy the distorted rotation on both sides): they are not chunk detectors
  M6: [],       // nothing is expected to fail: position / scale errors have no detector in this smoke
};
// Checks that must stay PASS for the control (the documented blind spots).
export const EXPECTED_PASS = { M5: ['S7', 'S8', 'S9'], M6: ['S1', 'S2', 'S3', 'S4', 'S5', 'S6', 'S7', 'S8', 'S9', 'S10', 'S10b', 'S11', 'S12', 'S13', 'S14'] };

// Registered expectations per gate version (run-sample-smoke.mjs --expectations). v3 is the registration the first run used, kept as it was (its M6 line is the
// one the result contradicted). v4 is a NEW registration: its M6 line is written from the v3 observation - a position scaled by 0.5 on the pose only made S10
// and S10b FAIL in both samples (three.js also S13, A-Frame not) - and is labelled as such: it was fixed after seeing the v3 result, for a new version, not
// substituted into the v3 expectation. S13 is neither expected nor required for M6 (it failed in one sample only). The checks that stay PASS for M6 are the
// self-consistency ones (S7-S9: the probe copies the scaled pose onto both sides), exactly as for M5; the remaining checks are not constrained.
export const CONTROL_EXPECTATIONS = {
  v3: { fail: EXPECTED_FAIL, pass: EXPECTED_PASS },
  v4: { fail: { ...EXPECTED_FAIL, M6: ['S10', 'S10b'] }, pass: { M5: ['S7', 'S8', 'S9'], M6: ['S7', 'S8', 'S9'] },
    basis: { M6: 'v3 reg2 observation: three.js FAIL S10 S10b S13, A-Frame FAIL S10 S10b; registered for v4 after that result (post-result registration of a new version)' } },
};

// The verdict of a negative control against the registered expectations: every expected check FAILs and every must-pass check stays PASS. `checks` = evaluate().checks,
// `failed` = the names of the checks that failed in the run. Without a control nothing is expected.
export function controlOutcome({ checks, failed, expectations, control }) {
  const expected = control ? expectations.fail[control] : null;
  const mustPass = control ? (expectations.pass[control] ?? []) : [];
  const passViolations = mustPass.filter((k) => checks[k].pass !== true);
  return { controlExpected: expected, controlMustPass: mustPass, controlPassViolations: passViolations,
    controlDetected: expected ? expected.every((k) => failed.includes(k)) && passViolations.length === 0 : null,
    controlMissed: expected ? expected.filter((k) => !failed.includes(k)) : null };
}

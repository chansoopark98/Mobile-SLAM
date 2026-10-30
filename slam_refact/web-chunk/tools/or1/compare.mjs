// OR-1 comparators. Everything is decided on the bits of the doubles (hex); the numeric deviations (metres, degrees, ulps) are
// reported next to the verdict, never instead of it.
import { canon, diffPaths, f64hex, ulpDistance } from './canon.mjs';
import { comparableFrame } from './players.mjs';

const sameBits = (a, b) => f64hex(a) === f64hex(b);
// angle of the relative rotation conj(a) * b, 2 * atan2(|xyz|, |w|): exactly 0 for equal quaternions (acos(dot) would report ~1e-2 degrees of
// rounding noise for them, the failure mode of IT-P3's first measure)
export function quatAngleDeg(a, b) {
  const [aw, ax, ay, az] = a, [bw, bx, by, bz] = b;
  const w = aw * bw + ax * bx + ay * by + az * bz;
  const x = aw * bx - ax * bw - ay * bz + az * by;
  const y = aw * by + ax * bz - ay * bw - az * bx;
  const z = aw * bz - ax * by + ay * bx - az * bw;
  return 2 * Math.atan2(Math.hypot(x, y, z), Math.abs(w)) * 180 / Math.PI;
}
const arrBits = (a, b) => a.length === b.length && a.every((x, i) => sameBits(x, b[i]));

// per-call input records of two arms (inputs the engine was given): identical = bit-identical canonical forms
export function compareInputs(framesA, framesB) {
  const n = Math.max(framesA.length, framesB.length);
  const mismatches = [];
  for (let i = 0; i < n; i++) {
    const a = framesA[i] ? comparableFrame(framesA[i]) : null, b = framesB[i] ? comparableFrame(framesB[i]) : null;
    const paths = a && b ? diffPaths(a, b) : ['(missing call)'];
    if (paths.length) mismatches.push({ call: i, frame: framesA[i]?.index ?? framesB[i]?.index ?? null, paths });
  }
  return { calls: n, countA: framesA.length, countB: framesB.length, identical: mismatches.length === 0, mismatches: mismatches.length,
    first: mismatches[0] ?? null, sample: mismatches.slice(0, 5) };
}

// engine output rows of two arms. `full` compares the complete canonical pose object (only arms that have it, i.e. the WASM arms);
// the headline fields (position, rotation, status, reason, repeated, sourceFrameIndex, worldPoints) are compared for every pair.
export function compareOutputs(rowsA, rowsB, { full = false } = {}) {
  const byIndex = new Map(rowsB.map((r) => [r.index, r]));
  let frames = 0, identical = 0, firstDiff = null, maxPos = 0, maxRotDeg = 0, maxUlp = 0, nonFiniteDiff = 0;
  const fieldCounts = {};
  const mismatchedFrames = [];
  for (const a of rowsA) {
    const b = byIndex.get(a.index);
    if (!b || a.fresh !== b.fresh) { frames++; fieldCounts.freshOrMissing = (fieldCounts.freshOrMissing ?? 0) + 1; mismatchedFrames.push(a.index); firstDiff ??= { index: a.index, fields: ['fresh/missing'] }; continue; }
    if (!a.fresh) continue;
    frames++;
    const fields = [];
    const ra = a.raw, rb = b.raw;
    if (!arrBits(ra.position, rb.position)) fields.push('position');
    if (!arrBits(ra.rotationWxyz, rb.rotationWxyz)) fields.push('rotationWxyz');
    for (const k of ['status', 'reason', 'repeated', 'sourceFrameIndex', 'worldPoints']) if (ra[k] !== rb[k]) fields.push(k);
    if (full && a.pose && b.pose) for (const p of diffPaths(a.pose, b.pose)) { const top = p.split('.')[0]; if (!fields.includes(top)) fields.push(top); }
    const dPos = Math.hypot(...ra.position.map((x, i) => x - rb.position[i]));
    const dRot = arrBits(ra.rotationWxyz, rb.rotationWxyz) ? 0 : quatAngleDeg(ra.rotationWxyz, rb.rotationWxyz);
    if (Number.isFinite(dPos)) maxPos = Math.max(maxPos, dPos); else nonFiniteDiff++;
    if (Number.isFinite(dRot)) maxRotDeg = Math.max(maxRotDeg, dRot);
    for (let i = 0; i < 3; i++) maxUlp = Math.max(maxUlp, ulpDistance(ra.position[i], rb.position[i]));
    for (let i = 0; i < 4; i++) maxUlp = Math.max(maxUlp, ulpDistance(ra.rotationWxyz[i], rb.rotationWxyz[i]));
    if (fields.length) {
      for (const f of fields) fieldCounts[f] = (fieldCounts[f] ?? 0) + 1;
      mismatchedFrames.push(a.index);
      firstDiff ??= { index: a.index, fields, dPos, dRotDeg: dRot };
    } else identical++;
  }
  return { frames, identicalFrames: identical, identical: identical === frames && rowsA.length === rowsB.length, firstDiff, fieldCounts,
    maxPositionDelta: maxPos, maxRotationDeltaDeg: maxRotDeg, maxUlp, nonFiniteDiff, mismatchedFramesSample: mismatchedFrames.slice(0, 10),
    mismatchedFrames: mismatchedFrames.length };
}

// the answers the chunk gave the app against the engine pose of the same call
export function compareAnswers(answers, rows) {
  const byIndex = new Map(rows.map((r) => [r.index, r]));
  let compared = 0, mismatches = 0, nulls = 0, first = null;
  for (const a of answers) {
    if (a.answer === null) { nulls++; continue; }
    const r = byIndex.get(a.index);
    if (!r || !r.fresh) continue;
    compared++;
    const p = r.raw;
    const expected = canon({ position: { x: p.position[0], y: p.position[1], z: p.position[2] },
      rotation: { x: p.rotationWxyz[1], y: p.rotationWxyz[2], z: p.rotationWxyz[3], w: p.rotationWxyz[0] }, trackingStatus: p.status, trackingReason: p.reason,
      worldPoints: p.reason !== 'INITIALIZING' ? p.worldPoints : 0 });
    const paths = diffPaths(a.answer, expected);
    if (paths.length) { mismatches++; first ??= { index: a.index, paths }; }
  }
  return { answers: answers.length, nullAnswers: nulls, compared, mismatches, identical: mismatches === 0, first };
}

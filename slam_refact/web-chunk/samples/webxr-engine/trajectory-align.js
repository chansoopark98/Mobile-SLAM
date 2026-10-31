// Offline trajectory comparison of the webxr-engine prototype: the slam_refact engine pose against the WebXR (ARCore) viewer pose of the
// same XR frame. Pure module (no DOM): the page computes a summary at export, align-cli.mjs recomputes it from an exported JSON.
//
// The reference is ARCore's own VIO estimate, NOT ground truth: every number here is "engine vs ARCore", and agreement does not prove
// accuracy (both can drift the same way; ARCore uses the same camera images). Units: positions in the frame of each trajectory (ARCore
// 'local' space = metres; the engine = its own world, metric only with estimateScale), times in seconds, angles in degrees.
//
// Conventions: quaternions are [w, x, y, z] (the engine's rotationWxyz; WebXR DOMPoint {x, y, z, w} is converted with xyzwToWxyz),
// camera-to-world (both poses are GL cameras: x right, y up, looking down -z). Rotation matrices are row-major 3x3 arrays.
//
// SE(3): Umeyama/Horn rigid alignment engine -> ARCore on time-matched pairs (scale fixed to 1) - the primary report. Sim(3): the same with
// a free scale, reported separately as a scale diagnostic only. RPE over 1 s windows uses the raw relative motions (SE(3): engine scale 1;
// Sim(3): engine scaled by the Sim(3) scale).
export const REFERENCE_LABEL = 'ARCore reference (WebXR viewer pose), not ground truth';
const DEG = 180 / Math.PI;

// ---- quaternion / matrix helpers ([w, x, y, z]) ----
export const xyzwToWxyz = (o) => [o.w, o.x, o.y, o.z];
export const qmul = (a, b) => [
  a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3],
  a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2],
  a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1],
  a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0],
];
export const qconj = (q) => [q[0], -q[1], -q[2], -q[3]];
export const qnormalize = (q) => { const n = Math.hypot(...q); return q.map((v) => v / n); };
// Rotation angle of q in radians, [0, pi].
export const qangle = (q) => 2 * Math.atan2(Math.hypot(q[1], q[2], q[3]), Math.abs(q[0]));
export function qToMat(q) {
  const [w, x, y, z] = qnormalize(q);
  return [
    [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
    [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
    [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
  ];
}
export function matToQ(R) {
  const tr = R[0][0] + R[1][1] + R[2][2];
  let q;
  if (tr > 0) {
    const s = 2 * Math.sqrt(tr + 1);
    q = [s / 4, (R[2][1] - R[1][2]) / s, (R[0][2] - R[2][0]) / s, (R[1][0] - R[0][1]) / s];
  } else if (R[0][0] > R[1][1] && R[0][0] > R[2][2]) {
    const s = 2 * Math.sqrt(1 + R[0][0] - R[1][1] - R[2][2]);
    q = [(R[2][1] - R[1][2]) / s, s / 4, (R[0][1] + R[1][0]) / s, (R[0][2] + R[2][0]) / s];
  } else if (R[1][1] > R[2][2]) {
    const s = 2 * Math.sqrt(1 + R[1][1] - R[0][0] - R[2][2]);
    q = [(R[0][2] - R[2][0]) / s, (R[0][1] + R[1][0]) / s, s / 4, (R[1][2] + R[2][1]) / s];
  } else {
    const s = 2 * Math.sqrt(1 + R[2][2] - R[0][0] - R[1][1]);
    q = [(R[1][0] - R[0][1]) / s, (R[0][2] + R[2][0]) / s, (R[1][2] + R[2][1]) / s, s / 4];
  }
  return qnormalize(q[0] < 0 ? q.map((v) => -v) : q);
}
export const matVec = (R, v) => [0, 1, 2].map((i) => R[i][0] * v[0] + R[i][1] * v[1] + R[i][2] * v[2]);
const sub = (a, b) => [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
const norm = (v) => Math.hypot(v[0], v[1], v[2]);

// Cyclic Jacobi eigen decomposition of a symmetric n x n matrix: {values, vectors} with vectors[k] the k-th eigenvector.
export function jacobiEigen(A) {
  const n = A.length, a = A.map((r) => r.slice()), V = a.map((_, i) => a.map((__, j) => (i === j ? 1 : 0)));
  for (let sweep = 0; sweep < 100; sweep++) {
    let off = 0;
    for (let i = 0; i < n; i++) for (let j = i + 1; j < n; j++) off += a[i][j] * a[i][j];
    if (off < 1e-30) break;
    for (let p = 0; p < n; p++) for (let q = p + 1; q < n; q++) {
      if (Math.abs(a[p][q]) < 1e-300) continue;
      const theta = (a[q][q] - a[p][p]) / (2 * a[p][q]);
      const t = Math.sign(theta || 1) / (Math.abs(theta) + Math.sqrt(theta * theta + 1));
      const c = 1 / Math.sqrt(t * t + 1), s = t * c;
      for (let k = 0; k < n; k++) {
        const akp = a[k][p], akq = a[k][q];
        a[k][p] = c * akp - s * akq; a[k][q] = s * akp + c * akq;
      }
      for (let k = 0; k < n; k++) {
        const apk = a[p][k], aqk = a[q][k];
        a[p][k] = c * apk - s * aqk; a[q][k] = s * apk + c * aqk;
      }
      for (let k = 0; k < n; k++) {
        const vkp = V[k][p], vkq = V[k][q];
        V[k][p] = c * vkp - s * vkq; V[k][q] = s * vkp + c * vkq;
      }
    }
  }
  return { values: a.map((r, i) => r[i]), vectors: a.map((_, k) => V.map((r) => r[k])) };
}

// Least-squares dst ~ s * R * src + t (Horn's quaternion solution of the rotation, Umeyama's scale). withScale = false fixes s = 1 (SE(3)).
export function umeyama(src, dst, withScale) {
  const n = src.length;
  const mean = (P) => [0, 1, 2].map((k) => P.reduce((acc, p) => acc + p[k], 0) / n);
  const ms = mean(src), md = mean(dst);
  const S = [[0, 0, 0], [0, 0, 0], [0, 0, 0]];
  let varSrc = 0;
  for (let i = 0; i < n; i++) {
    const a = sub(src[i], ms), b = sub(dst[i], md);
    varSrc += a[0] * a[0] + a[1] * a[1] + a[2] * a[2];
    for (let r = 0; r < 3; r++) for (let c = 0; c < 3; c++) S[r][c] += a[r] * b[c];
  }
  const [[xx, xy, xz], [yx, yy, yz], [zx, zy, zz]] = S;
  const N = [
    [xx + yy + zz, yz - zy, zx - xz, xy - yx],
    [yz - zy, xx - yy - zz, xy + yx, zx + xz],
    [zx - xz, xy + yx, -xx + yy - zz, yz + zy],
    [xy - yx, zx + xz, yz + zy, -xx - yy + zz],
  ];
  const { values, vectors } = jacobiEigen(N);
  const q = vectors[values.indexOf(Math.max(...values))];
  const R = qToMat(q);
  let s = 1;
  if (withScale) {
    let num = 0;
    for (let i = 0; i < n; i++) { const ra = matVec(R, sub(src[i], ms)), b = sub(dst[i], md); num += ra[0] * b[0] + ra[1] * b[1] + ra[2] * b[2]; }
    s = varSrc > 0 ? num / varSrc : 1;
  }
  const rms = matVec(R, ms);
  return { R, q: matToQ(R), s, t: [md[0] - s * rms[0], md[1] - s * rms[1], md[2] - s * rms[2]] };
}

export function errorStats(values) {
  const a = values.filter(Number.isFinite).sort((x, y) => x - y);
  if (!a.length) return { n: 0, rmse: NaN, mean: NaN, median: NaN, max: NaN };
  const mid = (a.length - 1) / 2;
  return { n: a.length, rmse: Math.sqrt(a.reduce((acc, v) => acc + v * v, 0) / a.length), mean: a.reduce((acc, v) => acc + v, 0) / a.length,
    median: (a[Math.floor(mid)] + a[Math.ceil(mid)]) / 2, max: a[a.length - 1] };
}

// Relative pose errors over windows of `delta` s (pairs sorted by t). Pair i is matched with the first j with t_j - t_i >= delta, kept when
// t_j - t_i <= delta + tol (no tracking gap inside the window). Translation error = |dE_t - dG_t| (both in the frame of camera i).
export function relativeErrors(pairs, { delta = 1, tol = 0.1, scale = 1 } = {}) {
  const trans = [], rot = [];
  let j = 0;
  for (let i = 0; i < pairs.length; i++) {
    if (j <= i) j = i + 1;
    while (j < pairs.length && pairs[j].t - pairs[i].t < delta) j++;
    if (j >= pairs.length) break;
    if (pairs[j].t - pairs[i].t > delta + tol) continue;
    const a = pairs[i], b = pairs[j];
    const dEq = qmul(qconj(a.eng.q), b.eng.q), dGq = qmul(qconj(a.ref.q), b.ref.q);
    const dEt = matVec(qToMat(qconj(a.eng.q)), sub(b.eng.p, a.eng.p)).map((v) => v * scale);
    const dGt = matVec(qToMat(qconj(a.ref.q)), sub(b.ref.p, a.ref.p));
    trans.push(norm(sub(dEt, dGt)));
    rot.push(qangle(qmul(qconj(dGq), dEq)) * DEG);
  }
  return { deltaS: delta, trans: errorStats(trans), rotDeg: errorStats(rot) };
}

// Absolute errors after the alignment {R, q, s, t}, plus the constant body-frame rotation offset (chordal mean of R_ref^T R R_eng) as a
// diagnostic: a large mean with a small spread means the two camera frame conventions differ, not that the engine rotates wrongly.
function absoluteErrors(pairs, al) {
  const trans = [], rot = [], resid = [];
  for (const p of pairs) {
    const pe = matVec(al.R, p.eng.p);
    trans.push(norm(sub([al.s * pe[0] + al.t[0], al.s * pe[1] + al.t[1], al.s * pe[2] + al.t[2]], p.ref.p)));
    const e = qmul(qconj(p.ref.q), qmul(al.q, p.eng.q));
    resid.push(e[0] < 0 ? e.map((v) => -v) : e);
    rot.push(qangle(e) * DEG);
  }
  const m = qnormalize([0, 1, 2, 3].map((k) => resid.reduce((acc, e) => acc + e[k], 0)));
  const spread = resid.map((e) => qangle(qmul(qconj(m), e)) * DEG);
  return { trans: errorStats(trans), rotDeg: errorStats(rot), bodyOffsetDiag: { angleDeg: qangle(m) * DEG, spreadDeg: errorStats(spread) } };
}

// records: exported frame records ({xrTime ms, ref: {p, q}, engine: {status, p, q}}). Pairs = frames with both poses, finite, engine status
// in `statuses`. Sorted by time.
export function pairsFromRecords(records, { statuses = ['NORMAL'] } = {}) {
  const finite = (a) => Array.isArray(a) && a.every(Number.isFinite);
  return records
    .filter((r) => r && r.ref && r.engine && statuses.includes(r.engine.status) && finite(r.ref.p) && finite(r.ref.q)
      && finite(r.engine.p) && finite(r.engine.q) && Number.isFinite(r.xrTime))
    .map((r) => ({ t: r.xrTime / 1000, eng: { p: r.engine.p, q: qnormalize(r.engine.q) }, ref: { p: r.ref.p, q: qnormalize(r.ref.q) } }))
    .sort((a, b) => a.t - b.t);
}

// Full report on time-matched pairs. refFrames = number of frames with a reference pose (coverage denominator).
export function compareTrajectories(pairs, { refFrames = pairs.length, rpeDelta = 1, minPairs = 10, minSpreadM = 0.05 } = {}) {
  const out = { label: REFERENCE_LABEL, pairs: pairs.length, refFrames, coverage: refFrames ? pairs.length / refFrames : 0, notes: [] };
  if (pairs.length < minPairs) { out.insufficient = `only ${pairs.length} time-matched pairs (< ${minPairs})`; return out; }
  out.durationS = pairs[pairs.length - 1].t - pairs[0].t;
  // Degeneracy: alignment rotation is undetermined when the reference positions do not span at least a plane.
  const mean = [0, 1, 2].map((k) => pairs.reduce((acc, p) => acc + p.ref.p[k], 0) / pairs.length);
  const C = [[0, 0, 0], [0, 0, 0], [0, 0, 0]];
  for (const p of pairs) { const d = sub(p.ref.p, mean); for (let r = 0; r < 3; r++) for (let c = 0; c < 3; c++) C[r][c] += d[r] * d[c] / pairs.length; }
  const ev = jacobiEigen(C).values.sort((a, b) => b - a);
  out.refSpreadM = ev.map((v) => Math.sqrt(Math.max(0, v)));
  out.degenerate = out.refSpreadM[0] < minSpreadM || out.refSpreadM[1] < 0.1 * out.refSpreadM[0];
  if (out.degenerate) out.notes.push('reference positions do not span a plane (or move < minSpreadM): the alignment rotation is weakly determined; prefer RPE');
  const src = pairs.map((p) => p.eng.p), dst = pairs.map((p) => p.ref.p);
  const se3 = umeyama(src, dst, false), sim3 = umeyama(src, dst, true);
  out.se3 = { transform: { q: se3.q, t: se3.t }, ate: absoluteErrors(pairs, se3), rpe: relativeErrors(pairs, { delta: rpeDelta }) };
  out.sim3 = { note: 'scale diagnostic only (not an accuracy result)', transform: { q: sim3.q, t: sim3.t, scale: sim3.s },
    ate: absoluteErrors(pairs, sim3), rpe: relativeErrors(pairs, { delta: rpeDelta, scale: sim3.s }) };
  return out;
}

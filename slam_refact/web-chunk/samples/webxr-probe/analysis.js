// Pure functions for the WebXR camera-access / EIS probe (P0-C2). No DOM, no WebGL: the page (app.js) collects samples, this module analyses them.
//
// Units: functions that take explicit `t` arrays use SECONDS; analyzeRun() takes DOMHighResTimeStamp MILLISECONDS (what the page records)
// and converts. Image shift is in pixels of the small (e.g. 64 px wide) analysis image; rotation rates are deg/s at the page boundary
// and rad/s inside.
//
// Everything here is a device measurement helper, not an accuracy metric.

// ---------------------------------------------------------------------------------------------------------------------------------
// Intrinsics from a WebXR projection matrix
//
// WebXR gives `view.projectionMatrix` as a column-major 4x4 OpenGL projection P (camera looks down -Z, +Y up). For a point
// (Xc, Yc, Zc) in GL camera coordinates and OpenCV-style camera coordinates (x = Xc, y = -Yc, z = -Zc, y down):
//   x_ndc = P[0]*Xc/(-Zc) - P[8]          =>  u = W/2 * (x_ndc + 1) = (W/2*P[0]) * x/z + W/2*(1 - P[8])
//   y_ndc = P[5]*Yc/(-Zc) - P[9]          =>  v = H/2 * (1 - y_ndc) = (H/2*P[5]) * y/z + H/2*(1 + P[9])
// hence   fx = W/2 * P[0],  fy = H/2 * P[5],  cx = W/2 * (1 - P[8]),  cy = H/2 * (1 + P[9]).
// This is valid only if the projection describes the full W x H camera image. In handheld AR the projection may be matched to the display
// (cropped/aspect-adjusted) instead; then fy/fx is not ~1 and the result is a display-projection value, not a camera-image intrinsic.
export function intrinsicsFromProjection(P, width, height) {
  const fx = (width / 2) * P[0];
  const fy = (height / 2) * P[5];
  const cx = (width / 2) * (1 - P[8]);
  const cy = (height / 2) * (1 + P[9]);
  const deg = 180 / Math.PI;
  return {
    fx, fy, cx, cy, width, height,
    hfovDeg: 2 * Math.atan(width / (2 * fx)) * deg,
    vfovDeg: 2 * Math.atan(height / (2 * fy)) * deg,
    squarePixelRatio: fy / fx, // ~1 when the projection matches the camera image
  };
}

// ---------------------------------------------------------------------------------------------------------------------------------
// Statistics
export function percentile(values, p) {
  const a = Array.from(values).filter(Number.isFinite).sort((x, y) => x - y);
  if (!a.length) return NaN;
  const r = (p / 100) * (a.length - 1);
  const lo = Math.floor(r);
  const hi = Math.ceil(r);
  return a[lo] + (a[hi] - a[lo]) * (r - lo);
}

// timesMs: frame timestamps (ms). fpsP50 = 1000/median dt; fpsAtP95Dt = 1000/p95 dt (the slow tail).
export function fpsStats(timesMs) {
  const dt = [];
  for (let i = 1; i < timesMs.length; i++) dt.push(timesMs[i] - timesMs[i - 1]);
  const p50 = percentile(dt, 50);
  const p95 = percentile(dt, 95);
  return {
    frames: timesMs.length,
    dtP50Ms: p50, dtP95Ms: p95, dtP99Ms: percentile(dt, 99), dtMaxMs: dt.length ? maxOf(dt) : NaN,
    fpsP50: 1000 / p50, fpsAtP95Dt: 1000 / p95,
  };
}

// ---------------------------------------------------------------------------------------------------------------------------------
// Global image shift between two luminance images by brute-force SAD over +-maxShift px.
// Convention: content moved by (dx, dy) from prev to curr, i.e. curr(x + dx, y + dy) ~ prev(x, y). Sub-pixel by a symmetric V fit on the
// SAD minimum (SAD of smooth images is V-shaped, not parabolic). Images are mean-removed first (auto exposure). `ok` needs texture
// (std >= minStd) and a distinct minimum (best/mean SAD < minDistinct).
export function estimateShiftSAD(prev, curr, w, h, { maxShift = 6, minStd = 2, minDistinct = 0.9 } = {}) {
  const m = maxShift;
  const S = 2 * m + 1;
  const n = w * h;
  let ma = 0;
  let mb = 0;
  for (let i = 0; i < n; i++) { ma += prev[i]; mb += curr[i]; }
  ma /= n; mb /= n;
  const a = new Float32Array(n);
  const b = new Float32Array(n);
  for (let i = 0; i < n; i++) { a[i] = prev[i] - ma; b[i] = curr[i] - mb; }

  const x0 = m; const x1 = w - m; const y0 = m; const y1 = h - m;
  const count = (x1 - x0) * (y1 - y0);
  if (count <= 0) return { dx: 0, dy: 0, sad: NaN, ratio: NaN, std: 0, ok: false, saturated: false };
  let ss = 0;
  for (let y = y0; y < y1; y++) for (let x = x0; x < x1; x++) ss += a[y * w + x] * a[y * w + x];
  const std = Math.sqrt(ss / count);

  const sad = new Float64Array(S * S);
  let best = Infinity;
  let bi = 0;
  let sum = 0;
  for (let sy = -m; sy <= m; sy++) {
    for (let sx = -m; sx <= m; sx++) {
      let s = 0;
      for (let y = y0; y < y1; y++) {
        const ra = y * w;
        const rb = (y + sy) * w + sx;
        for (let x = x0; x < x1; x++) s += Math.abs(a[ra + x] - b[rb + x]);
      }
      s /= count;
      const idx = (sy + m) * S + (sx + m);
      sad[idx] = s;
      sum += s;
      if (s < best) { best = s; bi = idx; }
    }
  }
  const ix = (bi % S) - m;
  const iy = Math.floor(bi / S) - m;
  const sub = (lo, hi) => {
    const d = Math.max(lo, hi) - best;
    return d > 1e-12 ? (0.5 * (lo - hi)) / d : 0;
  };
  let fx = 0;
  let fy = 0;
  if (Math.abs(ix) < m) fx = sub(sad[bi - 1], sad[bi + 1]);
  if (Math.abs(iy) < m) fy = sub(sad[bi - S], sad[bi + S]);
  const mean = sum / (S * S);
  const ratio = mean > 1e-12 ? best / mean : 1;
  return { dx: ix + fx, dy: iy + fy, sad: best, ratio, std, ok: std >= minStd && ratio < minDistinct, saturated: Math.abs(ix) === m || Math.abs(iy) === m };
}

// ---------------------------------------------------------------------------------------------------------------------------------
// Spectral analysis
export function fft(re, im) { // in place, radix-2, length power of 2
  const n = re.length;
  for (let i = 1, j = 0; i < n; i++) {
    let bit = n >> 1;
    for (; j & bit; bit >>= 1) j ^= bit;
    j ^= bit;
    if (i < j) { [re[i], re[j]] = [re[j], re[i]]; [im[i], im[j]] = [im[j], im[i]]; }
  }
  for (let len = 2; len <= n; len <<= 1) {
    const ang = (-2 * Math.PI) / len;
    const wr = Math.cos(ang);
    const wi = Math.sin(ang);
    for (let i = 0; i < n; i += len) {
      let cr = 1;
      let ci = 0;
      for (let k = 0; k < len / 2; k++) {
        const ur = re[i + k]; const ui = im[i + k];
        const vr = re[i + k + len / 2] * cr - im[i + k + len / 2] * ci;
        const vi = re[i + k + len / 2] * ci + im[i + k + len / 2] * cr;
        re[i + k] = ur + vr; im[i + k] = ui + vi;
        re[i + k + len / 2] = ur - vr; im[i + k + len / 2] = ui - vi;
        const nr = cr * wr - ci * wi;
        ci = cr * wi + ci * wr;
        cr = nr;
      }
    }
  }
}

// Welch magnitude-squared coherence between x and y (uniformly sampled at fs Hz). Hann window, mean removed per segment, 50% overlap.
// Constant time delay between x and y does not change the magnitude (it only rotates the cross-spectrum phase).
// Note: with K segments, independent signals still show a coherence floor of about 1/K.
export function welchCoherence(x, y, fs, { segLen = 64, overlap = 0.5 } = {}) {
  const n = Math.min(x.length, y.length);
  const step = Math.max(1, Math.round(segLen * (1 - overlap)));
  const nSeg = n >= segLen ? Math.floor((n - segLen) / step) + 1 : 0;
  const nb = segLen / 2 + 1;
  const sxx = new Float64Array(nb);
  const syy = new Float64Array(nb);
  const sxyRe = new Float64Array(nb);
  const sxyIm = new Float64Array(nb);
  const win = new Float64Array(segLen);
  for (let i = 0; i < segLen; i++) win[i] = 0.5 - 0.5 * Math.cos((2 * Math.PI * i) / segLen);
  const xr = new Float64Array(segLen); const xi = new Float64Array(segLen);
  const yr = new Float64Array(segLen); const yi = new Float64Array(segLen);
  for (let s = 0; s < nSeg; s++) {
    const o = s * step;
    let mx = 0; let my = 0;
    for (let i = 0; i < segLen; i++) { mx += x[o + i]; my += y[o + i]; }
    mx /= segLen; my /= segLen;
    for (let i = 0; i < segLen; i++) { xr[i] = (x[o + i] - mx) * win[i]; yr[i] = (y[o + i] - my) * win[i]; xi[i] = 0; yi[i] = 0; }
    fft(xr, xi); fft(yr, yi);
    for (let k = 0; k < nb; k++) {
      sxx[k] += xr[k] * xr[k] + xi[k] * xi[k];
      syy[k] += yr[k] * yr[k] + yi[k] * yi[k];
      sxyRe[k] += xr[k] * yr[k] + xi[k] * yi[k]; // X * conj(Y)
      sxyIm[k] += xi[k] * yr[k] - xr[k] * yi[k];
    }
  }
  const freqs = new Float64Array(nb);
  const coh = new Float64Array(nb);
  for (let k = 0; k < nb; k++) {
    freqs[k] = (k * fs) / segLen;
    const d = sxx[k] * syy[k];
    coh[k] = d > 0 ? (sxyRe[k] * sxyRe[k] + sxyIm[k] * sxyIm[k]) / d : 0;
  }
  return { freqs, coh, sxx, syy, sxyRe, sxyIm, nSegments: nSeg, segLen, fs };
}

// Band summary of welchCoherence(x = image, y = reference). coherence = mean of the per-bin coherence; coherenceW = band-aggregate
// coherence |sum Sxy|^2 / (sum Sxx * sum Syy), which is dominated by the bins that carry energy (robust to line-like spectra); gain = |sum Sxy| / sum Syy
// (x units per y unit); rmsRatio = sqrt(sum Sxx / sum Syy).
export function bandStats(w, f0, f1) {
  let c = 0; let bins = 0; let sxx = 0; let syy = 0; let re = 0; let im = 0;
  for (let k = 0; k < w.freqs.length; k++) {
    if (w.freqs[k] < f0 || w.freqs[k] > f1) continue;
    c += w.coh[k]; bins++;
    sxx += w.sxx[k]; syy += w.syy[k]; re += w.sxyRe[k]; im += w.sxyIm[k];
  }
  if (!bins || !(syy > 0)) return { coherence: NaN, coherenceW: NaN, gain: NaN, rmsRatio: NaN, bins, nSegments: w.nSegments };
  return { coherence: c / bins, coherenceW: (re * re + im * im) / (sxx * syy), gain: Math.hypot(re, im) / syy, rmsRatio: Math.sqrt(sxx / syy), bins, nSegments: w.nSegments };
}

// Linear interpolation of (t, v) onto t0, t0 + 1/fs, ... <= t1. t ascending (seconds).
export function resampleUniform(t, v, fs, t0, t1) {
  const n = Math.max(0, Math.floor((t1 - t0) * fs) + 1);
  const out = new Float64Array(n);
  let j = 0;
  for (let i = 0; i < n; i++) {
    const tt = t0 + i / fs;
    while (j < t.length - 2 && t[j + 1] < tt) j++;
    const dt = t[j + 1] - t[j];
    const a = dt > 0 ? Math.min(1, Math.max(0, (tt - t[j]) / dt)) : 0;
    out[i] = v[j] + (v[j + 1] - v[j]) * a;
  }
  return out;
}

// Mean of the piecewise-linear signal (tg, vg) over each interval [ta[i], tb[i]] (exact trapezoid integral / duration).
// NaN where the interval is not covered by the signal.
export function intervalMean(tg, vg, ta, tb) {
  const n = tg.length;
  const cum = new Float64Array(n);
  for (let i = 1; i < n; i++) cum[i] = cum[i - 1] + 0.5 * (vg[i] + vg[i - 1]) * (tg[i] - tg[i - 1]);
  const cumAt = (t) => {
    let lo = 0; let hi = n - 1;
    while (hi - lo > 1) { const mid = (lo + hi) >> 1; if (tg[mid] <= t) lo = mid; else hi = mid; }
    const dt = tg[hi] - tg[lo];
    const a = dt > 0 ? (t - tg[lo]) / dt : 0;
    const vt = vg[lo] + (vg[hi] - vg[lo]) * a;
    return cum[lo] + 0.5 * (vg[lo] + vt) * (t - tg[lo]);
  };
  const out = new Float64Array(ta.length);
  for (let i = 0; i < ta.length; i++) {
    out[i] = n >= 2 && ta[i] >= tg[0] && tb[i] <= tg[n - 1] && tb[i] > ta[i] ? (cumAt(tb[i]) - cumAt(ta[i])) / (tb[i] - ta[i]) : NaN;
  }
  return out;
}

// Body-frame angular velocity (rad/s) between two unit quaternions q = [x, y, z, w]: omega = axis * angle / dt of conj(q0) * q1.
export function angularVelocityBetween(q0, q1, dt) {
  const [ax, ay, az, aw] = [-q0[0], -q0[1], -q0[2], q0[3]];
  const [bx, by, bz, bw] = q1;
  let x = aw * bx + ax * bw + ay * bz - az * by;
  let y = aw * by - ax * bz + ay * bw + az * bx;
  let z = aw * bz + ax * by - ay * bx + az * bw;
  let w = aw * bw - ax * bx - ay * by - az * bz;
  if (w < 0) { x = -x; y = -y; z = -z; w = -w; }
  const s = Math.hypot(x, y, z);
  if (s < 1e-12 || !(dt > 0)) return [0, 0, 0];
  const k = (2 * Math.atan2(s, w)) / (s * dt);
  return [x * k, y * k, z * k];
}

// ---------------------------------------------------------------------------------------------------------------------------------
// Whole-run analysis.
//   frames: [{ t (ms), dx, dy (shift px vs the previous frame, small image), ok (shift valid), q?: [x,y,z,w] (XR viewer orientation) }]
//   gyro:   { t: ms[], x: deg/s[], y: deg/s[], z: deg/s[] } (DeviceMotion rotationRate) or null
//   fxSmallPx: reference focal length in small-image pixels (for the amplitude ratio) or null
// For every reference (gyro, XR pose) the image shift rate is compared with every (image component, rotation axis) pair, because the
// sensor/screen/device axis mapping is not assumed; the best pair by 3-6 Hz coherence is reported ("best"). Coherence is sign-agnostic.
export const BANDS = { low: [1, 3], mid: [3, 6], high: [6, 12] };

export function analyzeRun({ frames, gyro = null, fxSmallPx = null, fs = 30, segLen = 64 }) {
  const out = {
    fs, segLen, counts: { frames: frames.length, pairs: 0, okPairs: 0, stalls: 0, gyroSamples: gyro ? gyro.t.length : 0 },
    okFraction: NaN, refs: {}, insufficient: null,
  };
  // pairs between consecutive frames
  const dts = [];
  for (let i = 1; i < frames.length; i++) dts.push((frames[i].t - frames[i - 1].t) / 1000);
  const medDt = percentile(dts, 50);
  const pairs = [];
  for (let i = 1; i < frames.length; i++) {
    out.counts.pairs++;
    const dt = dts[i - 1];
    const f = frames[i];
    if (!(dt > 0) || dt > 3 * medDt) { out.counts.stalls++; continue; }
    if (!f.ok || !Number.isFinite(f.dx) || !Number.isFinite(f.dy)) continue;
    pairs.push({ ta: frames[i - 1].t / 1000, tb: f.t / 1000, dt, dx: f.dx / dt, dy: f.dy / dt, q0: frames[i - 1].q ?? null, q1: f.q ?? null });
  }
  out.counts.okPairs = pairs.length;
  out.okFraction = out.counts.pairs ? pairs.length / out.counts.pairs : NaN;
  if (pairs.length < segLen + 8) { out.insufficient = `valid frame pairs ${pairs.length} < ${segLen + 8}`; return out; }

  const tMid = pairs.map((p) => 0.5 * (p.ta + p.tb));
  const t0 = tMid[0];
  const t1 = tMid[tMid.length - 1];
  const img = { dx: Float64Array.from(pairs, (p) => p.dx), dy: Float64Array.from(pairs, (p) => p.dy) }; // px/s in the small image
  const ta = pairs.map((p) => p.ta);
  const tb = pairs.map((p) => p.tb);

  const refSeries = {};
  if (gyro && gyro.t.length > 2) {
    const tg = Float64Array.from(gyro.t, (v) => v / 1000);
    const r = Math.PI / 180;
    refSeries.gyro = { x: intervalMean(tg, Float64Array.from(gyro.x, (v) => v * r), ta, tb), y: intervalMean(tg, Float64Array.from(gyro.y, (v) => v * r), ta, tb), z: intervalMean(tg, Float64Array.from(gyro.z, (v) => v * r), ta, tb) };
  }
  if (pairs.some((p) => p.q0 && p.q1)) {
    const w = pairs.map((p) => (p.q0 && p.q1 ? angularVelocityBetween(p.q0, p.q1, p.dt) : [NaN, NaN, NaN]));
    refSeries.pose = { x: Float64Array.from(w, (v) => v[0]), y: Float64Array.from(w, (v) => v[1]), z: Float64Array.from(w, (v) => v[2]) };
  }

  for (const [refName, axes] of Object.entries(refSeries)) {
    const rows = [];
    for (const comp of ['dx', 'dy']) {
      for (const axis of ['x', 'y', 'z']) {
        const tt = []; const xv = []; const yv = [];
        for (let i = 0; i < pairs.length; i++) {
          if (Number.isFinite(axes[axis][i])) { tt.push(tMid[i]); xv.push(img[comp][i]); yv.push(axes[axis][i]); }
        }
        if (tt.length < segLen + 8) continue;
        const u = resampleUniform(tt, xv, fs, tt[0], tt[tt.length - 1]);
        const v = resampleUniform(tt, yv, fs, tt[0], tt[tt.length - 1]);
        const wc = welchCoherence(u, v, fs, { segLen });
        const bands = {};
        for (const [name, [f0, f1]] of Object.entries(BANDS)) {
          const b = bandStats(wc, f0, f1);
          b.effFxPx = b.gain; // image px/s per rad/s = focal length in small-image px
          b.ampRatio = fxSmallPx ? b.gain / fxSmallPx : null;
          bands[name] = b;
        }
        rows.push({ img: comp, axis, samples: tt.length, nUniform: u.length, nSegments: wc.nSegments, bands });
      }
    }
    rows.sort((a, b) => (b.bands.mid.coherence || 0) - (a.bands.mid.coherence || 0));
    out.refs[refName] = { pairs: rows, best: rows[0] ?? null, pairsWithData: pairs.length };
  }
  out.durationS = t1 - t0;
  return out;
}

// Iterative max/min (a spread into Math.max/min throws RangeError on large arrays and gives -/+Infinity on empty ones). Empty input -> null.
function maxOf(xs) { if (!xs.length) return null; let m = xs[0]; for (const x of xs) if (x > m || Number.isNaN(x)) m = x; return m; }
function minOf(xs) { if (!xs.length) return null; let m = xs[0]; for (const x of xs) if (x < m || Number.isNaN(x)) m = x; return m; }

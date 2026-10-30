#!/usr/bin/env python3
"""Check the S5 spec formulas (docs/spec/05-metric-scale.md) against the original-WASM golden
vectors produced by mse-probe.mjs. This is an independent float32 re-derivation from the spec
text, not the C++ port. Tolerances are the ones fixed in mse-probe.mjs / spec section 8.

Usage: python3 -I spec_check.py build/slam-refact-20261015/spec/metric-scale/probe-a/result.json
"""
import json
import math
import sys

import numpy as np

F = np.float32


def ulp(x):
  return float(np.spacing(F(abs(x))))


def xcorr(x, y):
  x = np.asarray(x, F)
  y = np.asarray(y, F)
  n, m = len(x), len(y)
  k = max(n, m)
  out = []
  for h in range(n + m - 1):
    o = k - 1 - h
    t = F(0)
    for i in range(n):
      j = o + i
      v = y[j] if 0 <= j < m else F(0)
      t = F(x[i] * v + t)
    out.append(float(t))
  return out


def scale_camera_space(a, c):
  a = np.asarray(a, F)
  c = np.asarray(c, F)
  num = F(0)
  den = F(0)
  for i in range(len(a)):
    num = F(num + F(F(F(a[i, 2] * c[i, 2]) + F(a[i, 1] * c[i, 1])) + F(a[i, 0] * c[i, 0])))
    den = F(den + F(F(F(a[i, 2] * a[i, 2]) + F(a[i, 1] * a[i, 1])) + F(a[i, 0] * a[i, 0])))
  s = F(abs(F(num / den)))
  r = F(0)
  for i in range(len(a)):
    d = c[i] - s * a[i]
    r = F(r + F(F(d[2] * d[2]) + F(d[1] * d[1]) + F(d[0] * d[0])))
  return float(s), float(F(r / F(len(a))))


def scale_l2(a, c):
  a = np.asarray(a, F)
  c = np.asarray(c, F)
  na = np.sqrt(np.sum(a * a, axis=1, dtype=F)).astype(F)
  nc = np.sqrt(np.sum(c * c, axis=1, dtype=F)).astype(F)
  s = F(F(np.sum(na * nc, dtype=F)) / F(np.sum(na * na, dtype=F)))
  r = 0.0
  for i in range(len(a)):
    d = float(F(-s * na[i] + nc[i]))
    r = float(F(d * d + r))
  return float(s), float(F(F(r) / F(len(a))))


def interp(ts, xs, td):
  ts = [F(t) for t in ts]
  xs = [F(x) for x in xs]
  if not ts or len(ts) != len(xs) or not td:
    return []
  if len(ts) == 1:
    return [float(xs[0])] * len(td)
  out = []
  n = len(ts)
  for s in td:
    s = F(s)
    lo = 0
    while lo < n and ts[lo] < s:
      lo += 1
    if lo == 0:
      f = 0
    elif lo == n:
      f = n - 1
    else:
      f = lo
      if abs(F(s - ts[lo - 1])) < abs(F(s - ts[lo])):
        f = lo - 1
    t = ts[f]
    if t > s:
      i0, i1 = (f - 1, f) if f >= 1 else (f, f + 1)
    else:
      i0, i1 = (f, f + 1) if f < n - 1 else (f - 1, f)
    dt = F(ts[i1] - ts[i0])
    w = F(0) if dt == 0 else F(F(xs[i1] - xs[i0]) / dt)
    out.append(float(F(F(s * w) + F(xs[f] - F(t * w)))))  # intercept first (VERIFIED vs golden)
  return out


def gauss_kernel(sigma, size):
  half = size // 2
  sigma = F(sigma)
  denom = F(F(sigma * F(-2.0)) * sigma)
  norm = (1.0 / float(sigma)) * 2.5066282746310002
  k = []
  tot = F(0)
  for d in range(2 * half + 1):
    i = d - half
    e = F(math.exp(float(F(F(i * i) / denom))))
    o = norm * float(e)
    k.append(F(o))
    tot = F(o + float(tot))
  return [float(F(v / tot)) for v in k]


def gauss_smooth(sig, sigma, size):
  k = gauss_kernel(sigma, size)
  half = (len(k) - 1) // 2
  out = []
  for n in range(len(sig)):
    c = F(0)
    s = F(0)
    for j, kv in enumerate(k):
      d = n - half + j
      if 0 <= d < len(sig):
        c = F(F(F(kv) * F(sig[d])) + c)
        s = F(s + F(kv))
    out.append(float(F(c / s)))
  return out


def fft_amp(data, fs):
  n = len(data)
  x = np.fft.fft(np.asarray(data, np.float64))
  amp = (np.abs(x[: n // 2]) / n).astype(F)
  freqs = [float(F(F(F(i) * F(fs)) / F(n))) for i in range(n // 2)]
  return freqs, [float(v) for v in amp]


def main():
  res = json.load(open(sys.argv[1]))["results"]
  report = {}

  out = res["xcorr_1183"]["outputs"]
  ok = True
  for case, golden in zip(res["xcorr_1183"]["inputs"], out):
    mine = xcorr(case["x"], case["y"])
    tol = 4 * ulp(max(abs(v) for v in golden) or 1)
    ok &= len(mine) == len(golden) and all(abs(a - b) <= tol for a, b in zip(mine, golden))
  report["xcorr_1183"] = ok

  # scale fit: regenerate inputs exactly as the probe (same LCG)
  def lcg(seed):
    s = [seed & 0xFFFFFFFF]

    def r():
      s[0] = (s[0] * 1664525 + 1013904223) & 0xFFFFFFFF
      return s[0] / 4294967296.0
    return r
  rnd = lcg(2)
  a = [[float(F(-2 + 4 * rnd())) for _ in range(3)] for _ in range(200)]
  c = [[float(F(0.37 * x + 0.05 * (rnd() - 0.5))) for x in v] for v in a]
  g = res["scale_fit_1191_1192"]["outputs"][0]
  s1, r1 = scale_camera_space(a, c)
  s2, r2 = scale_l2(a, c)
  report["scale_cameraSpace_1191"] = (abs(s1 - g["scaleCameraSpace"]["scale"]) <= 1e-6 * abs(s1) and
                                      abs(r1 - g["scaleCameraSpace"]["residual"]) <= 1e-5 * abs(r1))
  report["scale_l2_1192"] = (abs(s2 - g["scaleL2"]["scale"]) <= 1e-6 * abs(s2) and
                             abs(r2 - g["scaleL2"]["residual"]) <= 1e-5 * abs(r2))
  report["scale_values"] = {"cam": [s1, r1, g["scaleCameraSpace"]], "l2": [s2, r2, g["scaleL2"]]}

  ok = True
  for case, golden in zip(res["interp_1198"]["inputs"], res["interp_1198"]["outputs"]):
    mine = interp(case["ts"], case["xs"], case["td"])
    ok &= len(mine) == len(golden) and all(abs(x - y) <= 2 * ulp(y) + 1e-7 for x, y in zip(mine, golden))
  report["interp_1198"] = ok

  m = res["misc_1179_1186_1187_1188"]
  ts = [F(t) for t in m["inputs"]["ts"]]
  acc = F(0)
  for i in range(1, len(ts)):
    acc = F(F(acc + ts[i]) - ts[i - 1])
  md = float(F(acc / F(len(ts) - 1)))
  crop = [float(t) for t in ts if F(1.1) <= t <= F(1.4)]
  dd = [F(v) for v in m["inputs"]["dedupeIn"]]
  ded = [dd[0]]
  for v in dd[1:]:
    if F(v - ded[-1]) > F(0.001):
      ded.append(v)
  merge = sorted([F(v) for v in m["inputs"]["mergeA"]] + [F(v) for v in m["inputs"]["mergeB"]])
  o = m["outputs"]
  report["meanDt_1179"] = abs(md - o["meanDt"]) <= 1e-6 * abs(md)
  report["crop_1186"] = crop == o["croppedT"]
  report["dedupe_1188"] = [float(v) for v in ded] == o["dedupe"]
  report["merge_1187"] = [float(v) for v in merge] == o["merge"]

  sig = res["gauss_1199_1200"]["inputs"]["signal"]
  ok_k = ok_s = True
  for g in res["gauss_1199_1200"]["outputs"]:
    k = gauss_kernel(g["sigma"], g["size"])
    ok_k &= len(k) == len(g["kernel"]) and all(abs(x - y) <= 2e-6 * abs(y) for x, y in zip(k, g["kernel"]))
    s = gauss_smooth(sig, g["sigma"], g["size"])
    ok_s &= all(abs(x - y) <= 1e-6 for x, y in zip(s, g["smoothed"]))
  report["gaussKernel_1199"] = ok_k
  report["gaussSmooth_1200"] = ok_s

  ok_f = ok_a = True
  worst = 0.0
  rnd = lcg(6)
  for g in res["fft_1240_1241"]["outputs"]:
    n = g["n"]
    data = [float(F(math.sin(2 * math.pi * 3.0 * i / 60) + 0.5 * math.cos(2 * math.pi * 7.5 * i / 60) +
                    0.1 * (rnd() - 0.5))) for i in range(n)]
    fr, am = fft_amp(data, 60.0)
    ok_f &= fr == g["frequencies"]
    tol = 1e-5 * max(g["amplitudes"]) + 1e-7
    d = max(abs(x - y) for x, y in zip(am, g["amplitudes"]))
    worst = max(worst, d / max(g["amplitudes"]))
    ok_a &= d <= tol and len(am) == len(g["amplitudes"])
  report["fft_frequencies_1241"] = ok_f
  report["fft_amplitudes_1241_vs_float64_reference"] = ok_a
  report["fft_worst_rel_err"] = worst

  # VIO position Kalman (spec section 5.2): F, Q, R, H, Joseph update, from spec text only.
  kf = res["kalman_position_1169"]
  dt_scale = float(F(1e-9))
  rho, sp, sv, sa, sm = F(0.9), F(0.0), F(0.0), F(0.9), F(0.1)
  q = np.diag([sp * sp] * 3 + [sv * sv] * 3 + [sa * sa] * 3).astype(F)
  rr = (np.eye(3) * (sm * sm)).astype(F)
  hh = np.zeros((3, 9), F)
  hh[:, :3] = np.eye(3)
  x = p = None
  last = None
  ok = True
  golden = {st["k"]: st for st in kf["outputs"]["steps"]}
  for k in range(40):
    t = k / 30
    tns = int(round(t * 1e9))
    wa = float(tns) * dt_scale
    z = np.array([F(0.5 * math.sin(2 * t)), F(0.1 * t * t), F(-0.3 * t)], F)
    if last is None:
      x = np.zeros(9, F)
      x[:3] = z
      p = np.eye(9, dtype=F)
      last = F(wa)
      continue
    dt = F(wa - float(last))
    f = np.eye(9, dtype=F)
    for i in range(3):
      f[i, 3 + i] = dt
      f[3 + i, 6 + i] = dt
      f[i, 6 + i] = F(F(dt * F(0.5)) * dt)
      f[6 + i, 6 + i] = rho
    xp = f @ x
    pp = f @ p @ f.T + q
    sinv = np.linalg.inv((hh @ pp @ hh.T + rr).astype(np.float64)).astype(F)
    kg = pp @ hh.T @ sinv
    x = xp + kg @ (z - hh @ xp)
    ikh = np.eye(9, dtype=F) - kg @ hh
    p = ikh @ pp @ ikh.T + kg @ rr @ kg.T
    last = F(wa)
    if k in golden:
      gx = np.array(golden[k]["x"], F)
      gp = np.array(golden[k]["Pdiag"], F)
      ok &= bool(np.all(np.abs(x - gx) <= 1e-5 * (1 + np.abs(gx))))
      ok &= bool(np.all(np.abs(np.diag(p) - gp) <= 1e-4 * np.abs(gp)))
  report["kalman_position_1169"] = ok

  print(json.dumps(report, indent=1))
  if not all(v for k, v in report.items() if isinstance(v, bool)):
    sys.exit(1)


if __name__ == "__main__":
  main()

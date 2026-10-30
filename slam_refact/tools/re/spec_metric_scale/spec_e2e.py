#!/usr/bin/env python3
"""End-to-end re-derivation of MetricScaleEstimator::pushFrame (func 944) written only from
docs/spec/05-metric-scale.md section 3/5, run on the synthetic scenarios of mse-probe.mjs
(P8 push_frame_944 and P9 variants), and compared to the original-WASM golden outputs.

Tolerances (fixed in mse-probe.mjs before results): first accepted frame index exact (or both
never accept), scale rel <= 1e-3, residual rel <= 1e-2 (only when golden residual >= 0),
sum rel <= 1e-6.

Usage: python3 -I spec_e2e.py build/slam-refact-20261015/spec/metric-scale/probe-e/result.json
"""
import collections
import json
import math
import sys

import numpy as np

F = np.float32
FLT_MIN = F(np.finfo(np.float32).tiny)
FLT_MAX = F(np.finfo(np.float32).max)
INT64_MAX = 2**63 - 1
NS_F32 = float(F(1e-9))  # (double)(float)1e-9 used by Kalman / IMU push
DEG2RAD = math.pi / 180.0


def base_cfg():
  return dict(method=1, sigma=0.0, activeCheck=0, sumCheck=1, temporalAlignment=1,
              placeWithInteval=1, fallbackTo60Hz=1, max_frames=2000, skip=25, pred_n=50,
              sum_thr=F(600.0), add_thr=F(0.8), high_thr=F(0.8), act_start=F(1.0), act_end=F(0.5),
              min_s=F(0.05), max_s=F(2.25), max_freq=F(5.0), cutoff=F(0.5), favor=1, window=F(1.0),
              rho=F(0.9), sig_p=F(0.0), sig_v=F(0.0), sig_a=F(0.9), sig_m=F(0.1),
              rho_w=F(0.9), sig_wv=F(0.2), sig_wa=F(0.5), sig_wm=F(0.5))


class Window:  # func1168
  def __init__(self, length):
    self.q = collections.deque()
    self.sum = 0.0
    self.len = F(length)

  def push(self, t, v):
    self.q.append((F(t), F(v)))
    self.sum += float(F(v))
    while len(self.q) >= 2 and F(self.q[-1][0] - self.q[0][0]) >= self.len:
      self.sum -= float(self.q[0][1])
      self.q.popleft()

  def mean(self):
    return F(self.sum / len(self.q)) if self.q else F(float("nan"))


def xcorr(x, y):  # func1183
  n, m = len(x), len(y)
  k = max(n, m)
  out = np.zeros(n + m - 1, F)
  for h in range(n + m - 1):
    o = k - 1 - h
    t = F(0)
    for i in range(n):
      j = o + i
      if 0 <= j < m:
        t = F(F(x[i] * y[j]) + t)
    out[h] = t
  return out


def interp(ts, xs, td):  # func1198
  n = len(ts)
  if n == 0 or n != len(xs) or len(td) == 0:
    return np.zeros(0, F)
  if n == 1:
    return np.full(len(td), xs[0], F)
  out = np.zeros(len(td), F)
  for idx, s in enumerate(td):
    lo = int(np.searchsorted(ts, s, side="left"))
    if lo == 0:
      f = 0
    elif lo == n:
      f = n - 1
    else:
      f = lo - 1 if abs(F(s - ts[lo - 1])) < abs(F(s - ts[lo])) else lo
    t = ts[f]
    if t > s:
      i0, i1 = (f - 1, f) if f >= 1 else (f, f + 1)
    else:
      i0, i1 = (f, f + 1) if f < n - 1 else (f - 1, f)
    dt = F(ts[i1] - ts[i0])
    w = F(0) if dt == 0 else F(F(xs[i1] - xs[i0]) / dt)
    out[idx] = F(F(s * w) + F(xs[f] - F(t * w)))
  return out


def interp3(ts, xs, td):  # func1182
  if len(xs) == 0:
    return np.zeros((0, 3), F)
  cols = [interp(ts, xs[:, j], td) for j in range(3)]
  if len(cols[0]) == 0:
    return np.zeros((0, 3), F)
  return np.stack(cols, 1)


def quantize(ts, fs):  # func1181, placeSensorEventsStartAtEnd = 1
  if fs <= 0:
    return np.array(ts, F)
  m = int(float(F(F(1.0) / F(fs))) * 1e9)
  ns = [int(F(F(t) * F(1e9))) for t in ts]
  out = [0] * len(ts)
  f = 0xFFFFFFFF
  for i in range(len(ts) - 1, -1, -1):
    q = int(ns[i] / m) & 0xFFFFFFFF
    f = f if f < q else q
    out[i] = f * m
    f = (f - 1) & 0xFFFFFFFF
    if f == 0:
      break
  return np.array([F(F(o) * F(1e-9)) for o in out], F)


def gauss_smooth3(xs, sigma, size):  # func1190/1199/1200
  half = size // 2
  if 2 * half + 1 <= 0:
    return xs.copy()
  denom = F(F(F(sigma) * F(-2.0)) * F(sigma))
  norm = (1.0 / float(F(sigma))) * 2.5066282746310002
  k, tot = [], F(0)
  for d in range(2 * half + 1):
    i = d - half
    o = norm * float(F(math.exp(float(F(F(i * i) / denom)))))
    k.append(F(o))
    tot = F(o + float(tot))
  k = [F(v / tot) for v in k]
  out = np.zeros_like(xs)
  for j in range(3):
    for n in range(len(xs)):
      c = s = F(0)
      for jj, kv in enumerate(k):
        d = n - half + jj
        if 0 <= d < len(xs):
          c = F(F(kv * xs[d, j]) + c)
          s = F(s + kv)
      out[n, j] = F(c / s)
  return out


def rts(states, q):  # smooth-vio-accel / angular smoothing in func1178
  sm = [(x.copy(), p.copy()) for x, p, _ in states]
  for k in range(len(sm) - 2, -1, -1):
    fk1 = states[k + 1][2]
    xk, pk = sm[k]
    pp = fk1 @ pk @ fk1.T + q
    g = (pk @ fk1.T) @ np.linalg.inv(pp.astype(np.float64)).astype(F)
    xs1, ps1 = sm[k + 1]
    sm[k] = ((xk + g @ (xs1 - fk1 @ xk)).astype(F), (pk + g @ (ps1 - pp) @ g.T).astype(F))
  return sm


def kf_step(x, p, fm, q, h, r, z):
  xp = fm @ x
  pp = fm @ p @ fm.T + q
  s = h @ pp @ h.T + r
  kg = pp @ h.T @ np.linalg.inv(s.astype(np.float64)).astype(F)
  x = (xp + kg @ (z - h @ xp)).astype(F)
  ikh = np.eye(len(x), dtype=F) - kg @ h
  p = (ikh @ pp @ ikh.T + kg @ r @ kg.T).astype(F)
  return x, p


def mean_std(v):
  if len(v) == 0:
    return F(0), F(0)
  acc = F(0)
  for d in v:
    acc = F(acc + d)
  mu = F(acc / F(len(v)))
  acc = F(0)
  for d in v:
    acc = F(acc + F(F(d - mu) * F(d - mu)))
  return mu, F(math.sqrt(float(F(acc / F(len(v))))))


def fft_amp(data, fs):  # FftCompute (public source fft-compute.cc)
  n = len(data)
  x = np.fft.fft(np.asarray(data, np.float64))
  amp = (np.abs(x[: n // 2]) / n).astype(F)
  freqs = np.array([F(F(F(i) * F(fs)) / F(n)) for i in range(n // 2)], F)
  return freqs, amp


def ldlt_solve(a, b):  # Eigen 3.4 LDLT<Lower> unblocked + solve (tolerance FLT_MIN)
  a = a.astype(F).copy()
  n = len(a)
  perm = list(range(n))
  trans = []
  for k in range(n):
    idx = k + int(np.argmax(np.abs(np.diag(a)[k:])))
    trans.append(idx)
    if idx != k:
      a[[k, idx], :] = a[[idx, k], :]
      a[:, [k, idx]] = a[:, [idx, k]]
    if k > 0:
      temp = (a[k, :k] * np.diag(a)[:k]).astype(F)
      a[k, k] = F(a[k, k] - F(np.dot(a[k, :k], temp)))
      if k + 1 < n:
        a[k + 1:, k] = (a[k + 1:, k] - a[k + 1:, :k] @ temp).astype(F)
    if k + 1 < n and abs(a[k, k]) > 0:
      a[k + 1:, k] = (a[k + 1:, k] / a[k, k]).astype(F)
  x = b.astype(F).copy()
  for k, idx in enumerate(trans):
    x[[k, idx]] = x[[idx, k]]
  for i in range(n):
    x[i] = F(x[i] - F(np.dot(a[i, :i], x[:i])))
  for i in range(n):
    x[i] = F(x[i] / a[i, i]) if abs(a[i, i]) > FLT_MIN else F(0)
  for i in range(n - 1, -1, -1):
    x[i] = F(x[i] - F(np.dot(a[i + 1:, i], x[i + 1:])))
  for k in range(n - 1, -1, -1):
    idx = trans[k]
    x[[k, idx]] = x[[idx, k]]
  return x


class Mse:
  def __init__(self, cfg):
    self.c = cfg
    self.t_imu0 = INT64_MAX
    self.t_frame0 = INT64_MAX
    self.t_first_event = INT64_MAX
    self.acc_win = Window(cfg["window"])
    self.gyr_win = Window(cfg["window"])
    self.min_acc = FLT_MAX
    self.reset()

  def reset(self):
    c = self.c
    self.sum = F(0)
    self.high = False
    self.active = False
    self.have = False
    self.scale, self.resid = F(1), F(0)
    self.kf_last = F(-1)
    self.states, self.times, self.poses = collections.deque(), collections.deque(), collections.deque()
    self.lin_t, self.lin_v = [], []
    self.gyr_t = collections.deque()
    self.w_states = collections.deque()
    self.w_last = F(-1)
    self.interval = 0
    self.q = np.diag([c["sig_p"] ** 2] * 3 + [c["sig_v"] ** 2] * 3 + [c["sig_a"] ** 2] * 3).astype(F)
    self.r = (np.eye(3) * F(c["sig_m"] * c["sig_m"])).astype(F)
    self.h = np.zeros((3, 9), F)
    self.h[:, :3] = np.eye(3)
    self.qw = np.diag([c["sig_wv"] ** 2] * 3 + [c["sig_wa"] ** 2] * 3).astype(F)
    self.rw = (np.eye(3) * F(c["sig_wm"] * c["sig_wm"])).astype(F)
    self.hw = np.zeros((3, 6), F)
    self.hw[:, :3] = np.eye(3)

  def kalman_push(self, z, rinv, t_ns):  # func1169
    c = self.c
    wa = float(t_ns - self.t_frame0) * NS_F32
    if self.kf_last == F(-1):
      self.kf_last = F(wa)
      self.x = np.zeros(9, F)
      self.x[:3] = z
      self.p = np.eye(9, dtype=F)
      return False
    dt = F(wa - float(self.kf_last))
    fm = np.eye(9, dtype=F)
    for i in range(3):
      fm[i, 3 + i] = dt
      fm[3 + i, 6 + i] = dt
      fm[i, 6 + i] = F(F(dt * F(0.5)) * dt)
      fm[6 + i, 6 + i] = c["rho"]
    self.x, self.p = kf_step(self.x, self.p, fm, self.q, self.h, self.r, z)
    self.states.append((self.x.copy(), self.p.copy(), fm))
    self.times.append(F(wa))
    self.poses.append(rinv)
    self.kf_last = F(wa)
    if len(self.times) > c["max_frames"]:
      self.states.popleft()
      self.times.popleft()
      self.poses.popleft()
    return c["pred_n"] < len(self.times) - c["skip"]

  def imu_push(self, events, za):  # func1170
    c = self.c
    for kind, t, interval, v in events:
      ld = float(t - self.t_imu0) * NS_F32
      if ld < float(za):
        continue
      qd = F(ld)
      self.interval = interval
      if kind == 4:
        self.lin_t.append(qd)
        self.lin_v.append(np.array(v, F))
      elif kind == 3:
        if self.w_last == F(-1):
          self.w_last = qd
          self.wx = np.zeros(6, F)
          self.wp = np.eye(6, dtype=F)
          continue
        z = np.array([F(float(F(vv)) * DEG2RAD) for vv in v], F)
        dt = F(ld - float(self.w_last))
        fm = np.eye(6, dtype=F)
        for i in range(3):
          fm[i, 3 + i] = dt
          fm[3 + i, 3 + i] = c["rho_w"]
        self.wx, self.wp = kf_step(self.wx, self.wp, fm, self.qw, self.hw, self.rw, z)
        self.w_states.append((self.wx.copy(), self.wp.copy(), fm))
        self.gyr_t.append(qd)
        self.w_last = qd
        if len(self.w_states) > c["max_frames"]:
          self.w_states.popleft()
          self.gyr_t.popleft()
    return len(self.lin_t) != 0

  def compute(self):
    c = self.c
    sm = rts(list(self.states), self.q)
    n = len(sm)
    acc = np.array([self.poses[m] @ sm[m][0][6:9] for m in range(n)], F)
    keep = n - c["skip"]
    tv = np.array(list(self.times)[:keep], F)
    av = acc[:keep]
    za, last = self.times[0], tv[-1]
    sel = [i for i, t in enumerate(self.lin_t) if za <= t <= last]
    ti = np.array([self.lin_t[i] for i in sel], F)
    ai = np.array([self.lin_v[i] for i in sel], F).reshape(-1, 3)
    gyro = c["method"] in (4, 5)
    tg = np.zeros(0, F)
    gw = ga = np.zeros((0, 3), F)
    if gyro and len(self.w_states) >= 2:
      wsm = rts(list(self.w_states), self.qw)
      idx = [i for i, t in enumerate(self.gyr_t) if za <= t <= last]
      tg = np.array([self.gyr_t[i] for i in idx], F)
      gw = np.array([wsm[i][0][:3] for i in idx], F).reshape(-1, 3)
      ga = np.array([wsm[i][0][3:] for i in idx], F).reshape(-1, 3)
    xa = F(0)
    if c["temporalAlignment"]:
      if c["placeWithInteval"] and self.interval:
        fs = F(F(F(1.0) / F(self.interval)) * F(1e9))
      elif c["fallbackTo60Hz"]:
        fs = F(60.0)
      else:
        acc_dt = F(0)
        for i in range(1, len(ti)):
          acc_dt = F(F(acc_dt + ti[i]) - ti[i - 1])
        fs = F(F(1.0) / F(acc_dt / F(len(ti) - 1)))
      ti = quantize(ti, fs)
      tg = quantize(tg, fs)
      if len(tv) and len(ti) >= 2:
        avi = interp3(tv, av, ti)
        xcs = [xcorr(ai[:, j], avi[:, j]) for j in range(3)]
        dt = F(ti[1] - ti[0])
        r = len(xcs[0]) >> 1
        lags = np.array([F(dt * F(m - r)) for m in range(len(xcs[0]))], F)
        best = []
        for xc in xcs:
          sc, lb, vb = FLT_MIN, F(0), F(0)
          for j, lag in enumerate(lags):
            if abs(lag) >= c["cutoff"]:
              continue
            w = max(F(F(F(lag * F(-3.6)) * lag) + F(1)), F(0.1)) if c["favor"] else F(1)
            s = F(w * abs(xc[j]))
            if s > sc:
              sc, lb, vb = s, lag, abs(xc[j])
          best.append((lb, vb, abs(lb) < c["cutoff"]))
        den = F(F((best[0][1] + F(0)) if best[0][2] else F(0)) + (best[1][1] if best[1][2] else F(-0.0)))
        den = F(den + (best[2][1] if best[2][2] else F(-0.0)))
        if den != 0:
          t1 = F(best[0][0] * best[0][1] + F(0)) if best[0][2] else F(0)
          t2 = F(best[1][0] * best[1][1] + t1) if best[1][2] else t1
          t3 = F(best[2][0] * best[2][1] + t2) if best[2][2] else t2
          xa = F(-t3 / den)
      ti = (ti + xa).astype(F)
      tg = (tg + xa).astype(F)
    if len(ti) == 0:
      return None
    lo = max(tv[0], ti[0])
    hi = min(tv[-1], ti[-1])
    crop = lambda t: (t >= lo) & (t <= hi)
    mv, mi, mg = crop(tv), crop(ti), crop(tg)
    tv_c, av_c, ti_c, ai_c, tg_c = tv[mv], av[mv], ti[mi], ai[mi], tg[mg]
    gw_c, ga_c = gw[mg], ga[mg]
    if len(tv_c) == 0 or len(ti_c) == 0:
      return None
    merged = np.array(sorted(list(ti_c) + list(tg_c), key=float), F) if len(tg_c) else ti_c
    tc = [merged[0]]
    for t in merged[1:]:
      if F(t - tc[-1]) > F(0.001):
        tc.append(t)
    tc = np.array(tc, F)
    vv = interp3(tv_c, av_c, tc)
    ii = interp3(ti_c, ai_c, tc)
    gg = interp3(tg_c, gw_c, tc)
    g2 = interp3(tg_c, ga_c, tc)
    iis = gauss_smooth3(ii, c["sigma"], int(F(c["sigma"]) * F(3.0))) if c["sigma"] != 0 else ii
    method = c["method"]
    if method in (1, 5):  # method 5: deviceModel 0 is not in the F1196 table -> lever arm 0 -> V unchanged
      a = vv
      num = den = F(0)
      for k in range(len(a)):
        num = F(num + F(F(F(a[k, 2] * iis[k, 2]) + F(a[k, 1] * iis[k, 1])) + F(a[k, 0] * iis[k, 0])))
        den = F(den + F(F(F(a[k, 2] * a[k, 2]) + F(a[k, 1] * a[k, 1])) + F(a[k, 0] * a[k, 0])))
      s = F(abs(F(num / den)))
      res = F(0)
      for k in range(len(a)):
        d = (iis[k] - s * a[k]).astype(F)
        res = F(res + F(F(d[2] * d[2]) + F(d[1] * d[1]) + F(d[0] * d[0])))
      return s, F(res / F(len(a)))
    if method == 2:
      na = np.sqrt(np.sum(vv * vv, axis=1, dtype=F)).astype(F)
      nc = np.sqrt(np.sum(iis * iis, axis=1, dtype=F)).astype(F)
      s0 = s1 = F(0)
      for k in range(len(na)):
        s0 = F(s0 + F(na[k] * nc[k]))
        s1 = F(s1 + F(na[k] * na[k]))
      s = F(s0 / s1)
      res = 0.0
      for k in range(len(na)):
        d = float(F(F(-s * na[k]) + nc[k]))
        res = float(F(d * d + res))
      return s, F(F(res) / F(len(na)))
    if method == 3:
      vt = interp3(tv_c, av_c, tc)
      dti = [F(tc[h] - tc[h - 1]) for h in range(1, len(tc))]
      mu_i, _ = mean_std(dti)
      fs = F(F(1.0) / mu_i)
      qa = va = F(0)
      num = [F(0)] * 3
      den = [F(0)] * 3
      for j in range(3):
        fi, ai_amp = fft_amp(ii[:, j], fs)
        fv, av_amp = fft_amp(vt[:, j], fs)
        for b in range(len(fi)):
          if fi[b] > c["max_freq"]:
            continue
          num[j] = F(F(ai_amp[b] * av_amp[b]) + num[j])
          den[j] = F(F(av_amp[b] * av_amp[b]) + den[j])
      return F(F(F(num[0] + num[1]) + num[2]) / F(F(den[0] + den[1]) + den[2])), F(-1)
    if method == 4:  # normal equations (float32) solved with Eigen-style LDLT (diag pivoting)
      ata = np.zeros((4, 4), F)
      atb = np.zeros(4, F)
      for k in range(len(vv)):
        w, al = gg[k], g2[k]
        wx = np.array([[0, -w[2], w[1]], [w[2], 0, -w[0]], [-w[1], w[0], 0]], F)
        ax = np.array([[0, -al[2], al[1]], [al[2], 0, -al[0]], [-al[1], al[0], 0]], F)
        m = (wx @ wx + ax).astype(F)
        row = np.concatenate([m, vv[k].reshape(3, 1)], 1).astype(F)
        ata = (ata + row.T @ row).astype(F)
        atb = (atb + row.T @ iis[k]).astype(F)
      sol = ldlt_solve(ata, atb)
      return F(abs(sol[3])), F(-1)
    raise ValueError("Scale estimation method unknown")

  def push_frame(self, events, z, rinv, t_ns, tracker_state=1, reset_flag=0):
    c = self.c
    for kind, t, _, v in events:
      if self.t_first_event == INT64_MAX:
        self.t_first_event = t
      wa = float(F(t - self.t_first_event)) * 1e-9
      mag = F(math.sqrt(float(F(F(v[2] * v[2]) + F(F(v[0] * v[0]) + F(v[1] * v[1]))))))
      if kind == 4:
        self.acc_win.push(wa, mag)
        self.min_acc = mag if mag < self.min_acc else self.min_acc
        if mag >= c["add_thr"]:
          self.sum = F(wa * float(mag) + float(self.sum))
      elif kind == 3:
        self.gyr_win.push(wa, mag)
    if c["high_thr"] < self.acc_win.mean():
      self.high = True
    if self.t_imu0 == INT64_MAX:
      if events:
        self.t_imu0 = events[0][1]
      ready_before = False
    else:
      ready_before = self.t_frame0 != INT64_MAX
    if self.t_frame0 == INT64_MAX:
      self.t_frame0 = t_ns
    if not tracker_state or reset_flag or not ready_before:
      self.reset()
      return 0
    if self.have:
      return 1
    ready = self.kalman_push(z, rinv, t_ns)
    za = self.times[0] if ready else F(0)
    imu_ok = self.imu_push(events, za)
    if not (ready and imu_ok):
      return 0
    if c["activeCheck"]:
      m = self.acc_win.mean()
      if c["act_start"] < m and not self.active:
        self.active = True
        return 0
      if not (c["act_end"] > m and self.active):
        return 0
      self.active = False
    elif c["sumCheck"]:
      if not (self.sum > c["sum_thr"]) or not self.high:
        return 0
    out = self.compute()
    if out is None:
      return 0
    s, res = out
    if (c["min_s"] > 0 and c["min_s"] > s) or (c["max_s"] > 0 and c["max_s"] < s):
      self.reset()
      return 0
    self.scale, self.resid, self.have = s, res, True
    return 1


def rot_y(th):
  return np.array([[math.cos(th), 0, math.sin(th)], [0, 1, 0], [-math.sin(th), 0, math.cos(th)]], F)


def run(inp):
  cfg = base_cfg()
  for k in ("method", "sigma", "activeCheck", "sumCheck", "temporalAlignment"):
    cfg[k] = inp["cfg"][k]
  s_true, off, start = inp["sTrue"], inp.get("imuOffsetSec", 0), inp.get("imuStartSec", 0)
  p = lambda t: [math.sin(1.3 * t) * 0.8, 0.3 * math.sin(2.1 * t + 0.4), math.cos(0.9 * t) * 0.6]
  pdd = lambda t: [-1.69 * math.sin(1.3 * t) * 0.8, -4.41 * 0.3 * math.sin(2.1 * t + 0.4),
                   -0.81 * math.cos(0.9 * t) * 0.6]
  th = lambda t: 0.3 * math.sin(0.7 * t)
  thd = lambda t: 0.21 * math.cos(0.7 * t)
  base = 5000000000
  mse = Mse(cfg)
  imu_k = round(start * 60)
  for k in range(inp["frames"]):
    tf = k / 30
    events = []
    while imu_k / 60 <= tf + 1e-9 and len(events) < 63:
      ti = imu_k / 60
      r = rot_y(th(ti))
      am = [s_true * x for x in pdd(ti)]
      ac = [float(F(float(r[0, cc]) * am[0] + float(r[1, cc]) * am[1] + float(r[2, cc]) * am[2]))
            for cc in range(3)]
      tns = base + round((ti + off) * 1e9)
      events.append((4, tns, 16666667, ac))
      events.append((3, tns, 16666667, [0.0, float(F(thd(ti) * 180 / math.pi)), 0.0]))
      imu_k += 1
    r = rot_y(th(tf))
    z = np.array([F(v) for v in p(tf)], F)
    if mse.push_frame(events, z, r.T.copy(), base + round(tf * 1e9)):
      return {"k": k, "scale": float(mse.scale), "residual": float(mse.resid), "sum": float(mse.sum)}
  return None


def main():
  res = json.load(open(sys.argv[1]))["results"]
  report = {}
  all_ok = True
  for name, v in res.items():
    if not name.startswith("push_frame_944"):
      continue
    g = v["outputs"]["firstAccept"]
    mine = run(v["inputs"])
    if g is None or mine is None:
      ok = g is None and mine is None
    else:
      ok = (mine["k"] == g["k"] and abs(mine["scale"] - g["scale"]) <= 1e-3 * g["scale"] and
            abs(mine["sum"] - g["sum"]) <= 1e-6 * g["sum"] and
            (g["residual"] < 0 or abs(mine["residual"] - g["residual"]) <= 1e-2 * g["residual"]))
    report[name] = {"pass": ok, "spec": mine, "golden": g and {k: g[k] for k in ("k", "scale", "residual", "sum")}}
    all_ok &= ok
  print(json.dumps(report, indent=1))
  if not all_ok:
    sys.exit(1)


if __name__ == "__main__":
  main()

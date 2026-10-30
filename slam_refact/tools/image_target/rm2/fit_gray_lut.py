#!/usr/bin/env python3
"""B6 / RM-2: how the sharp .grayscale() function was identified, and how well the fitted function holds.
Run with `python3 -I`. Inputs: sharp_gray_probe.mjs output dir, and the gray-stages dirs (sharp_gray_stages.mjs).

Candidate A  y = enc(Y), Y = 0.2126 R + 0.7152 G + 0.0722 B on linearised channels, rounded       (continuous)
Candidate B  same Y, but enc() is a 256-entry table T[k] = rint(255 * enc(k / 255)) looked up at x = 255 * Y with
             LINEAR INTERPOLATION between T[floor(x)] and T[floor(x) + 1], then round half up      (fitted)
Evidence 1  pure channel ramps (256 x 3 pixels): fraction of exact matches for A and B.
Evidence 2  every distinct RGB triple that occurs in the gray-stages inputs (sharp's own full-resolution gray D
            against the cropped RGB): exact fraction and maximum error for A and B, per directory.
Usage: fit_gray_lut.py <probe-dir> <stages-dir> [<stages-dir> ...]
"""
import glob
import json
import os
import sys

import numpy as np

W = np.array([0.2126, 0.7152, 0.0722])


def s2l(c):
  return np.where(c <= 0.04045, c / 12.92, ((c + 0.055) / 1.055) ** 2.4)


def l2s(y):
  return np.where(y <= 0.0031308, 12.92 * y, 1.055 * np.power(np.maximum(y, 0), 1 / 2.4) - 0.055)


T = np.rint(l2s(np.arange(256) / 255.0) * 255.0)


def cand_a(rgb):
  return np.rint(np.clip(l2s(s2l(rgb / 255.0) @ W), 0, 1) * 255).astype(int)


def cand_b(rgb):
  x = np.clip(s2l(rgb / 255.0) @ W, 0, 1) * 255.0
  i = np.minimum(np.floor(x).astype(int), 254)
  f = x - i
  return np.floor(T[i] * (1 - f) + T[i + 1] * f + 0.5).astype(int)


def rd(d, n, w, h, c):
  a = np.fromfile(os.path.join(d, n), dtype=np.uint8)
  return a.reshape(h, w, c) if c > 1 else a.reshape(h, w)


probe = sys.argv[1]
out = {}
g = np.fromfile(os.path.join(probe, 'pure_channels.gray.raw'), dtype=np.uint8).reshape(4, 256)
lv = np.arange(256)
ev1 = {}
for ci, name in enumerate('RGB'):
  rgb = np.zeros((256, 3))
  rgb[:, ci] = lv
  ev1[name] = {'A_exact': float((cand_a(rgb) == g[ci]).mean()), 'B_exact': float((cand_b(rgb) == g[ci]).mean())}
ev1['gray_ramp_identity'] = bool((g[3] == lv).all())
rgbg = np.fromfile(os.path.join(probe, 'grid.rgb.raw'), dtype=np.uint8).reshape(-1, 3).astype(float)
gg = np.fromfile(os.path.join(probe, 'grid.gray.raw'), dtype=np.uint8)
ev1['grid17^3'] = {'A_exact': float((cand_a(rgbg) == gg).mean()), 'B_exact': float((cand_b(rgbg) == gg).mean()),
                   'B_bad': int((cand_b(rgbg) != gg).sum()), 'n': int(gg.size)}
out['evidence1_probe'] = ev1
print('evidence 1', json.dumps(ev1))

for sd in sys.argv[2:]:
  tab = {}
  for d in sorted(glob.glob(os.path.join(sd, '*/'))):
    m = json.load(open(d + 'meta.json'))
    ci, Di = m['cropped'], m['D']
    rgb = rd(d, 'cropped_rgb.raw', ci['width'], ci['height'], ci['channels'])[..., :3].reshape(-1, 3)
    D = rd(d, 'D_gray_fullres.raw', Di['width'], Di['height'], 1).reshape(-1)
    k = (rgb[:, 0].astype(np.int64) << 16) | (rgb[:, 1].astype(np.int64) << 8) | rgb[:, 2]
    u, idx = np.unique(k, return_index=True)
    for kk, i in zip(u, idx):
      tab.setdefault(int(kk), int(D[i]))
  keys = np.array(sorted(tab))
  vals = np.array([tab[k] for k in keys])
  rgb = np.stack([(keys >> 16) & 255, (keys >> 8) & 255, keys & 255], 1).astype(float)
  a, b = cand_a(rgb), cand_b(rgb)
  r = {'distinct_rgb_triples': int(keys.size), 'A_exact': round(float((a == vals).mean()), 6),
       'A_max_err': int(np.abs(a - vals).max()), 'B_exact': round(float((b == vals).mean()), 6),
       'B_bad': int((b != vals).sum()), 'B_max_err': int(np.abs(b - vals).max())}
  out['evidence2_' + os.path.basename(os.path.normpath(sd))] = r
  print('evidence 2', sd, json.dumps(r))
json.dump(out, open(os.path.join(probe, '..', 'fit_gray_lut.json'), 'w'), indent=1)

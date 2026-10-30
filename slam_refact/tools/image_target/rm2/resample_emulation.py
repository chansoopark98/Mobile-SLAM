#!/usr/bin/env python3
"""B6 / RM-2: can the resize step of sharp (libvips) be reproduced from PIL or a small numpy emulation?
Run with `python3 -I`. Input: the gray-stages directory written by sharp_gray_stages.mjs. For every image
whose crop is valid for the public CLI (>= 480x640): take D (sharp's own gray at full resolution) and compare
sharp's final luminance A = resize(D) with
  (a) PIL Image.resize(LANCZOS)
  (b) separable Lanczos3, centre convention (o + 0.5) * scale - 0.5, support 3 * scale, weights normalised,
      vertical pass first, rounded to uint8 between passes (the best of the variants tried; the variants
      and their numbers are in the output file).
Result in the README: neither is bit exact; (b) is closer on some images, not all. This is an analysis, not
a claim about libvips internals (no libvips source was read).
Usage: resample_emulation.py <gray-stages-dir> <out-prefix>
"""
import json
import os
import sys

import numpy as np
from PIL import Image

S, OUT = sys.argv[1], sys.argv[2]


def rd(d, n, w, h, c):
  a = np.fromfile(os.path.join(d, n), dtype=np.uint8)
  return a.reshape(h, w, c) if c > 1 else a.reshape(h, w)


def lanczos3(x):
  x = np.abs(x)
  out = np.where(x < 1e-12, 1.0, np.sinc(x) * np.sinc(x / 3))
  return np.where(x < 3, out, 0.0)


def weights(n_in, n_out):
  scale = n_in / n_out
  W = np.zeros((n_out, n_in))
  for o in range(n_out):
    c = (o + 0.5) * scale - 0.5
    idx = np.arange(int(np.floor(c - 3 * scale)) + 1, int(np.floor(c + 3 * scale)) + 1)
    w = lanczos3((idx - c) / scale)
    w /= w.sum()
    np.add.at(W[o], np.clip(idx, 0, n_in - 1), w)
  return W


def emulate(im, order, round_between):
  H, Wd = im.shape
  oh, ow = 640, int(np.floor(Wd * 640 / H + 0.5))
  wy, wx = weights(H, oh), weights(Wd, ow)
  a = im.astype(np.float64)
  if order == 'vh':
    t = wy @ a
    if round_between:
      t = np.clip(np.rint(t), 0, 255)
    o = t @ wx.T
  else:
    t = a @ wx.T
    if round_between:
      t = np.clip(np.rint(t), 0, 255)
    o = wy @ t
  return np.clip(np.rint(o), 0, 255).astype(np.uint8)


def st(x, A):
  d = np.abs(x.astype(int) - A.astype(int))
  return {'mean': round(float(d.mean()), 4), 'p95': float(np.percentile(d, 95)), 'max': int(d.max()),
          'exact': round(float((d == 0).mean()), 4)}


res, lines = {}, []
for id_ in sorted(os.listdir(S)):
  d = os.path.join(S, id_)
  mp = os.path.join(d, 'meta.json')
  if not os.path.exists(mp):
    continue
  m = json.load(open(mp))
  if m['cropped']['width'] < 480 or m['cropped']['height'] < 640:
    continue  # outside the public CLI contract (validateCrop)
  if (m['cropped']['width'], m['cropped']['height']) == (m['A']['width'], m['A']['height']):
    continue  # no resampling happens (crop is already 480x640)
  D = rd(d, 'D_gray_fullres.raw', m['D']['width'], m['D']['height'], 1)
  A = rd(d, 'A_final_gray.raw', m['A']['width'], m['A']['height'], 1)
  pil = np.asarray(Image.fromarray(D).resize((A.shape[1], A.shape[0]), Image.Resampling.LANCZOS))
  r = {'image': m['image'].split('lfs-files/')[-1], 'scale': round(m['D']['height'] / 640, 3),
       'PIL_LANCZOS': st(pil, A)}
  for order in ('vh', 'hv'):
    for rb in (False, True):
      r['emu_%s_%s' % (order, 'round' if rb else 'float')] = st(emulate(D, order, rb), A)
  res[id_] = r
  lines.append('%s (scale %.2f): PIL %s | emu vh+round %s' % (
      r['image'], r['scale'], r['PIL_LANCZOS'], r['emu_vh_round']))
json.dump(res, open(OUT + '.json', 'w'), indent=1)
open(OUT + '.txt', 'w').write('\n'.join(lines) + '\n')
print('\n'.join(lines))

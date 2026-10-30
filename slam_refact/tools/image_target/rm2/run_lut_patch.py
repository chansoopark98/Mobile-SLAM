#!/usr/bin/env python3
"""B6 / RM-2: what the PROPOSED change to image_target_cli.py would do, without editing that file.
Run with `python3 -I`.

Imports image_target_cli (read-only), replaces its private `_to_gray_libvips` with the fitted sharp
grayscale (256-entry sRGB table, linear interpolation on linear-light Y; see analyze_gray.py:gray_vips_lut and
rm2/README.md), and runs the 13 public answer scripts (+3 docs cases) with `--gray-mode libvips` in
<out>/runs/libvips_lut/. compare_runs.py then compares those outputs with the public CLI's.
Usage: run_lut_patch.py <rm2-results dir>
"""
import importlib.util
import os
import sys

import numpy as np
from PIL import Image

HERE = os.path.dirname(os.path.abspath(__file__))
CLI = os.path.join(HERE, '..', 'image_target_cli.py')
OUT = os.path.abspath(sys.argv[1])

spec = importlib.util.spec_from_file_location('image_target_cli', CLI)
cli = importlib.util.module_from_spec(spec)
spec.loader.exec_module(cli)

_LUT = None


def _lut():
  global _LUT
  if _LUT is None:
    v = np.arange(256) / 255.0
    enc = np.where(v <= 0.0031308, 12.92 * v, 1.055 * np.power(v, 1 / 2.4) - 0.055)
    _LUT = np.rint(enc * 255.0)
  return _LUT


def to_gray_libvips_fitted(img):
  """Proposed body of image_target_cli._to_gray_libvips (same early returns for 16-bit and L)."""
  if img.mode in cli._SIXTEEN_BIT_MODES or img.mode == 'L':
    return cli._to_gray8(img)
  c = np.asarray(img.convert('RGB'), dtype=np.float64) / 255.0
  lin = np.where(c <= 0.04045, c / 12.92, ((c + 0.055) / 1.055) ** 2.4)
  x = np.clip(lin @ np.array([0.2126, 0.7152, 0.0722]), 0.0, 1.0) * 255.0
  i = np.minimum(np.floor(x).astype(np.int64), 254)
  f = x - i
  t = _lut()
  v = t[i] * (1.0 - f) + t[i + 1] * f
  return Image.fromarray(np.floor(v + 0.5).astype(np.uint8), 'L')


cli._to_gray_libvips = to_gray_libvips_fitted

W = os.path.join(OUT, 'runs', 'libvips_lut')
os.makedirs(os.path.join(W, 'logs'), exist_ok=True)
os.makedirs(os.path.join(W, 'test', 'exif'), exist_ok=True)
for sub in ('', 'exif'):
  src = os.path.join(OUT, 'runs', 'orig', 'test', sub)
  for f in sorted(os.listdir(src)):
    dst = os.path.join(W, 'test', sub, f)
    if os.path.isfile(os.path.join(src, f)) and not os.path.lexists(dst):
      os.symlink(os.path.realpath(os.path.join(src, f)), dst)
os.chdir(W)
rc_all = 0
for f in sorted(os.listdir('test')):
  if not f.endswith('.txt'):
    continue
  rc = cli.main(['--script', 'test/' + f, '--overwrite', '--timestamp', '0', '--gray-mode', 'libvips'])
  print('libvips_lut', f[:-4], 'exit=%d' % rc)
  rc_all |= rc
sys.exit(rc_all)

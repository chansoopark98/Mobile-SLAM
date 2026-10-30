#!/usr/bin/env python3
"""Input-variation mutants of the hand gate data (US-021; hand-us021.v1.md §6, protocol §4).

  make_hand_mutants.py --g1 <dir> --g2 <dir> --g3 <dir> --g3-tensors <dir> --out <new dir>

For every mutant writes <out>/<name>/{g1,g2,g3,g3t}: the unchanged sets are symlinks, the touched one is a copy with ONE
changed expected value. <out>/EXPECTATIONS.tsv lists name and the gates that must FAIL ('-' = control, nothing may fail).
The shared sources and the original data are never edited. numpy only; run with python3 -I.
"""
import argparse
import os
import shutil

import numpy as np


def main():
  ap = argparse.ArgumentParser()
  for k in ('g1', 'g2', 'g3', 'g3_tensors', 'out'):
    ap.add_argument('--' + k.replace('_', '-'), required=True)
  a = ap.parse_args()
  base = {'g1': os.path.abspath(a.g1), 'g2': os.path.abspath(a.g2), 'g3': os.path.abspath(a.g3), 'g3t': os.path.abspath(a.g3_tensors)}
  os.makedirs(a.out, exist_ok=False)
  expect = []

  def new(name, fails, touched, edit):
    d = os.path.join(a.out, name)
    os.makedirs(d)
    for k, v in base.items():
      if k == touched:
        shutil.copytree(v, os.path.join(d, k))
      else:
        os.symlink(v, os.path.join(d, k))
    edit(os.path.join(d, touched))
    expect.append('%s\t%s' % (name, fails))

  def bump_bytes(fname, delta, fraction=None, count=None):
    def f(dir_):
      p = os.path.join(dir_, fname)
      v = np.fromfile(p, np.uint8).astype(np.int32)
      if fraction is None:
        v = np.clip(v + delta, 0, 255)
      else:  # change whole pixels (3 bytes) at evenly spaced positions
        n = len(v) // 3
        sel = np.linspace(0, n - 1, max(1, int(n * fraction))).astype(int)
        for c in range(3):
          v[sel * 3 + c] = (v[sel * 3 + c] + 90) % 256
      v.astype(np.uint8).tofile(p)
    return f

  def edit_tsv(fname, row, col, fn, skip_prefix=None):
    def f(dir_):
      p = os.path.join(dir_, fname)
      lines = open(p).read().split('\n')
      idx = [i for i, l in enumerate(lines) if l and not l.startswith('#')]
      i = idx[row]
      t = lines[i].split('\t')
      t[col] = fn(t[col])
      lines[i] = '\t'.join(t)
      open(p, 'w').write('\n'.join(lines))
    return f

  plus = lambda d: (lambda s: repr(float(s) + d))

  def ulp_padding(dir_):  # one float32 ulp on the first non-zero letterbox padding of fit_cases.tsv
    p = os.path.join(dir_, 'fit_cases.tsv')
    lines = open(p).read().split('\n')
    for i, l in enumerate(lines):
      if not l or l.startswith('#'):
        continue
      t = l.split('\t')
      for col in range(5, 9):
        v = np.float32(float(t[col]))
        if v != 0:
          t[col] = repr(float(np.nextafter(v, np.float32(np.inf))))
          lines[i] = '\t'.join(t)
          open(p, 'w').write('\n'.join(lines))
          return
    raise SystemExit('no non-zero padding to move')

  new('g1_resize_plus4', 'G1', 'g1', bump_bytes('resize-3.rgb', 4))
  new('g1_resize_plus1_control', '-', 'g1', bump_bytes('resize-3.rgb', 1))
  new('g1_fit_plus4', 'G1', 'g1', bump_bytes('fit-2.rgb', 4))
  new('g1_fit_padding', 'G1', 'g1', edit_tsv('fit_cases.tsv', 2, 5, plus(0.01)))
  new('g1_fit_padding_1ulp', 'G1', 'g1', ulp_padding)
  new('g1_crop_5percent_pixels', 'G1', 'g1', bump_bytes('crop-5.rgb', 0, fraction=0.05))
  new('g1_crop_half_percent_control', '-', 'g1', bump_bytes('crop-5.rgb', 0, fraction=0.005))
  new('g2_detect_plus2e-5', 'G2', 'g2', edit_tsv('detect_cases.tsv', 1, 2, plus(2e-5)))
  new('g2_detect_plus3e-6_control', '-', 'g2', edit_tsv('detect_cases.tsv', 1, 2, plus(3e-6)))
  new('g2_decoded_count_plus1', 'G2', 'g2', edit_tsv('detect_cases.tsv', 0, 2, lambda s: str(int(s) + 1)))
  new('g2_rect_plus2e-5', 'G2', 'g2', edit_tsv('rect_cases.tsv', 3, 30, plus(2e-5)))
  new('g2_landmark_plus2e-5', 'G2', 'g2', edit_tsv('landmark_cases.tsv', 4, 80, plus(2e-5)))

  def anchor(dir_):
    p = os.path.join(dir_, 'anchors.f32')
    v = np.fromfile(p, np.float32)
    v[1000] += np.float32(1e-4)
    v.tofile(p)
  new('g2_anchor_plus1e-4', 'G2', 'g2', anchor)
  new('g3_detection_plus2e-5', 'G3', 'g3', edit_tsv('real_cases.tsv', 1, 2, plus(2e-5)))
  new('g3_detection_plus3e-6_control', '-', 'g3', edit_tsv('real_cases.tsv', 1, 2, plus(3e-6)))
  new('g3_count_plus1', 'G3', 'g3', edit_tsv('real_cases.tsv', 0, 3, lambda s: str(int(s) + 1)))
  open(os.path.join(a.out, 'EXPECTATIONS.tsv'), 'w').write('# mutant\tgates that must FAIL (- : control, nothing may fail)\n' + '\n'.join(expect) + '\n')
  print(len(expect), 'mutants')


if __name__ == '__main__':
  main()

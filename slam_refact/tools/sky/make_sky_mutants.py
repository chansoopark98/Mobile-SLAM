#!/usr/bin/env python3
"""Input-variation mutants of a recorded sky reference (US-020; sky-us020.v1.md §6, protocol §4).

  make_sky_mutants.py --record <original record> --out <new dir>

Writes <out>/<name>/ for every mutant below: symlinks to the original cam/, resized/, invoke/, sky/ and a changed replay.txt
(plus copies of the tampered data files in mut/). The shared sources and the original record are never edited. Each mutant
carries a documented expectation in <out>/EXPECTATIONS.tsv: name, rules that must FAIL ('-' = the control must PASS everything).

Mutants perturb the EXPECTED side by one amount just beyond a registered tolerance (and controls just inside it), i.e. they model
an engine that differs from the original by that amount. numpy only; run with python3 -I.
"""
import argparse
import os
import shutil

import numpy as np


def load(rec):
  return [l.rstrip('\n') for l in open(os.path.join(rec, 'replay.txt'))]


def main():
  ap = argparse.ArgumentParser()
  ap.add_argument('--record', required=True)
  ap.add_argument('--out', required=True)
  a = ap.parse_args()
  rec = os.path.abspath(a.record)
  os.makedirs(a.out, exist_ok=False)
  base = load(rec)
  expect = []

  def new(name, fails, edit):
    d = os.path.join(a.out, name)
    os.makedirs(os.path.join(d, 'mut'))
    for sub in ('cam', 'resized', 'invoke', 'sky'):
      if os.path.exists(os.path.join(rec, sub)):
        os.symlink(os.path.join(rec, sub), os.path.join(d, sub))
    lines = edit(d, list(base))
    open(os.path.join(d, 'replay.txt'), 'w').write('\n'.join(lines) + '\n')
    expect.append('%s\t%s' % (name, fails))

  def idx(lines, tag, n=0):
    return [i for i, l in enumerate(lines) if l.split() and l.split()[0] == tag][n]

  def tamper_file(d, rel, fn, dtype):
    data = np.fromfile(os.path.join(rec, rel), dtype).copy()
    data = fn(data)
    out = os.path.join(d, 'mut', os.path.basename(rel))
    data.tofile(out)
    return 'mut/' + os.path.basename(rel)

  def add_levels(delta, channels=(0, 1, 2, 3)):
    def f(px):
      v = px.reshape(-1, 4).astype(np.int32)
      for c in channels:
        v[:, c] = np.clip(v[:, c] + delta, 0, 255)
      return v.astype(np.uint8).reshape(-1)
    return f

  # R1: the camera frame our pipeline resizes (the frame of recorded invoke n = 5) shifted by +4 levels (fail) / +1 level
  # (control; the unmutated baseline of R1 is 1 level). The recorded invoke image is NOT touched here: it is also the input
  # of the open-loop check R2.
  for name, delta, fails in (('r1_cam_plus4', 4, 'R1'), ('r1_cam_plus1_control', 1, '-')):
    def edit(d, lines, delta=delta):
      k = lines[idx(lines, 'invoke', 5)].split()[1]
      i = [j for j, l in enumerate(lines) if l.split() and l.split()[0] == 'frame' and l.split()[1] == k][0]
      t = lines[i].split()
      t[4] = tamper_file(d, t[4], add_levels(delta), np.uint8)
      lines[i] = ' '.join(t)
      return lines
    new(name, fails, edit)

  # R2: recorded model output (n = 5): one pixel +1.5e-4 (fail), one pixel +5e-5 (control)
  for name, delta, fails in (('r2_plus1p5e4', 1.5e-4, 'R2'), ('r2_plus5e5_control', 5e-5, '-')):
    def edit(d, lines, delta=delta):
      i = idx(lines, 'invoke', 5)
      t = lines[i].split()
      def f(v):
        v[len(v) // 3] = np.float32(v[len(v) // 3] + np.float32(delta)) if v[len(v) // 3] < 0.2 else np.float32(v[len(v) // 3] - np.float32(delta))
        return v
      t[10] = tamper_file(d, t[10], f, np.float32)
      lines[i] = ' '.join(t)
      return lines
    new(name, fails, edit)

  # R3: recorded texture (update n = 10): all channels +4 (fail); R channel +1 only (control)
  for name, delta, ch, fails in (('r3_plus4_all', 4, (0, 1, 2, 3), 'R3'), ('r3_plus1_red_control', 1, (0,), '-')):
    def edit(d, lines, delta=delta, ch=ch):
      i = idx(lines, 'update', 10)
      t = lines[i].split()
      t[3] = tamper_file(d, t[3], add_levels(delta, ch), np.uint8)
      lines[i] = ' '.join(t)
      return lines
    new(name, fails, edit)

  # R4: response camera / rotation of update n = 10 (tokens: 7..22 intrinsics, 23..26 rotation, 27..29 position)
  def r4(token, rel_or_abs, amount, fails, name):
    def edit(d, lines):
      i = idx(lines, 'update', 10)
      t = lines[i].split()
      v = float(t[token])
      t[token] = repr(v * (1 + amount) if rel_or_abs == 'rel' else v + amount)
      lines[i] = ' '.join(t)
      return lines
    new(name, fails, edit)
  r4(7, 'rel', 2e-6, 'R4', 'r4_intrinsics_rel2e-6')
  r4(7, 'rel', 3e-7, '-', 'r4_intrinsics_rel3e-7_control')
  r4(23, 'abs', 2e-5, 'R4', 'r4_rotation_abs2e-5')
  r4(23, 'abs', 3e-6, '-', 'r4_rotation_abs3e-6_control')
  r4(27, 'abs', 2e-5, 'R4', 'r4_position_abs2e-5')

  # R5: events
  def drop_scanning(d, lines):
    i = [j for j, l in enumerate(lines) if l.startswith('event') and l.split()[2] == 'layerscanning'][0]
    del lines[i]
    return lines
  new('r5_drop_layerscanning', 'R5', drop_scanning)
  def pct_plus_one_count(d, lines):
    js = [j for j, l in enumerate(lines) if l.startswith('event') and l.split()[2] == 'layerfound']
    if not js:
      return lines
    t = lines[js[0]].split()
    t[3] = repr(float(t[3]) + 1.0 / (144 * 256))
    lines[js[0]] = ' '.join(t)
    return lines
  new('r5_layerfound_plus_one_pixel', 'R5' if any(l.startswith('event') and l.split()[2] == 'layerfound' for l in base) else '-', pct_plus_one_count)

  # R6: a missing recorded request (count differs), a rotated request
  def drop_invoke(d, lines):
    del lines[idx(lines, 'invoke', 7)]
    return lines
  new('r6_drop_invoke', 'R6', drop_invoke)
  def rot_invoke(d, lines):
    i = idx(lines, 'invoke', 3)
    t = lines[i].split()
    t[4] = repr(float(t[4]) + 1e-6)
    lines[i] = ' '.join(t)
    return lines
  new('r6_rotation_plus1e-6', 'R6', rot_invoke)

  open(os.path.join(a.out, 'EXPECTATIONS.tsv'), 'w').write('# mutant\trules that must FAIL (- : control, nothing may fail)\n' + '\n'.join(expect) + '\n')
  print(len(expect), 'mutants')


if __name__ == '__main__':
  main()

#!/usr/bin/env python3
"""Input-validity preconditions of a recorded sky gate set (US-020; sky-us020.v1.md §2). Reads ONLY the record of
the original runtime (replay.txt and the sky textures it lists); never runs or reads anything of slam_refact.

  check_sky_gate_set.py <record dir> --min-updates N [--need-layerfound] [--need-content]

P1  at least N 'update' lines (processed frames with an output texture)
P2  (--need-layerfound) at least one 'event layerfound' line
P3  (--need-content) the last texture is not degenerate: at least 2 % of its pixels differ from its most common RGBA value
Exit 1 and a message per violated precondition. Run with python3 -I (numpy only).
"""
import argparse
import os
import sys

import numpy as np


def main():
  ap = argparse.ArgumentParser()
  ap.add_argument('record')
  ap.add_argument('--min-updates', type=int, required=True)
  ap.add_argument('--need-layerfound', action='store_true')
  ap.add_argument('--need-content', action='store_true')
  a = ap.parse_args()
  lines = [l.split() for l in open(os.path.join(a.record, 'replay.txt')) if l.strip() and not l.startswith('#')]
  updates = [l for l in lines if l[0] == 'update']
  errs = []
  if len(updates) < a.min_updates:
    errs.append('P1: %d updates < %d' % (len(updates), a.min_updates))
  if a.need_layerfound and not any(l[0] == 'event' and l[2] == 'layerfound' for l in lines):
    errs.append('P2: no layerfound event')
  frac = None
  if a.need_content:
    if not updates:
      errs.append('P3: no update')
    else:
      u = updates[-1]
      w, h = int(u[4]), int(u[5])
      px = np.fromfile(os.path.join(a.record, u[3]), np.uint8).reshape(-1, 4)
      if px.shape[0] != w * h:
        errs.append('P3: texture size %d != %dx%d' % (px.shape[0], w, h))
      else:
        _, counts = np.unique(px.view(np.uint32), return_counts=True)
        frac = 1.0 - counts.max() / px.shape[0]
        if frac < 0.02:
          errs.append('P3: only %.4f of the pixels differ from the most common value' % frac)
  print('%s: updates %d, content fraction %s -> %s' % (a.record, len(updates), 'n/a' if frac is None else '%.4f' % frac, 'OK' if not errs else 'VIOLATED'))
  for e in errs:
    print('  ' + e)
  sys.exit(1 if errs else 0)


if __name__ == '__main__':
  main()

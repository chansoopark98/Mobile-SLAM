#!/usr/bin/env python3
"""Run-to-run reproducibility of the ORIGINAL sky runtime recorder (US-020; sky-us020.v1.md §5, N1).

  compare_sky_runs.py <record A> <record B>

Compares two recordings of the same input by what the gate consumes, not by file names (update/invoke file indices depend on
asynchronous scheduling): processed frames (camera texture bytes, orientation), invoke n (rotation, percentage, image bytes,
model output bytes, and the staged frame it was sent at / answered at), updates by processed frame k (texture bytes,
intrinsics, rotation, position, percentage; the set of k that have an update), and events (name, staged frame, percentage).
Prints one line per category with the number of differences; exit 1 if any. numpy only; run with python3 -I.
"""
import os
import sys

import numpy as np


def load(rec):
  lines = [l.split() for l in open(os.path.join(rec, 'replay.txt')) if l.strip() and not l.startswith('#')]
  d = {'meta': None, 'config': None, 'frame': {}, 'invoke': [], 'update': {}, 'event': []}
  for t in lines:
    if t[0] in ('meta', 'config'):
      d[t[0]] = t[1:]
    elif t[0] == 'frame':
      d['frame'][int(t[1])] = t
    elif t[0] == 'invoke':
      d['invoke'].append(t)
    elif t[0] == 'update':
      d['update'][int(t[1])] = t
    elif t[0] == 'event':
      d['event'].append(t[1:])
  return d


def blob(rec, rel):
  return open(os.path.join(rec, rel), 'rb').read()


def main():
  a, b = sys.argv[1], sys.argv[2]
  A, B = load(a), load(b)
  diffs = {}

  def add(cat, n=1):
    diffs[cat] = diffs.get(cat, 0) + n

  for k in ('meta', 'config'):
    if A[k] != B[k]:
      add(k)
  if set(A['frame']) != set(B['frame']):
    add('frame set')
  for k in set(A['frame']) & set(B['frame']):
    ta, tb = A['frame'][k], B['frame'][k]
    if ta[1:4] != tb[1:4] or ta[5:] != tb[5:] or blob(a, ta[4]) != blob(b, tb[4]):
      add('frame (camera bytes / orientation)')
  if len(A['invoke']) != len(B['invoke']):
    add('invoke count')
  for ia, ib in zip(A['invoke'], B['invoke']):
    if ia[1:9] != ib[1:9]:
      add('invoke fields (k, sent/answered staged, rotation, percentage)')
    if blob(a, ia[9]) != blob(b, ib[9]):
      add('invoke image bytes')
    if blob(a, ia[10]) != blob(b, ib[10]):
      add('invoke model output bytes')
  if set(A['update']) != set(B['update']):
    add('update set (frames with an output texture)', len(set(A['update']) ^ set(B['update'])))
  for k in set(A['update']) & set(B['update']):
    ua, ub = A['update'][k], B['update'][k]
    if ua[4:] != ub[4:]:
      add('update numeric fields (size, percentage, intrinsics, rotation, position)')
    if blob(a, ua[3]) != blob(b, ub[3]):
      add('update texture bytes')
  if A['event'] != B['event']:
    add('events')
  if not diffs:
    print('IDENTICAL by content: %d frames, %d invokes, %d updates, %d events' % (len(A['frame']), len(A['invoke']), len(A['update']), len(A['event'])))
    return 0
  for c, n in sorted(diffs.items()):
    print('DIFF %-70s %d' % (c, n))
  return 1


if __name__ == '__main__':
  sys.exit(main())

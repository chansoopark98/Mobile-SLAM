#!/usr/bin/env python3
"""Compare a sr_sky_replay output with the recorded original run (docs/spec/09-sky.md §6).

  compare_sky_reference.py --record <reference dir> --replay <replay dir> [--json <new file>]

Per processed frame k that has an original layers.sky texture: pixel difference statistics of our
texture against the original (all four channels, 8-bit levels), the fraction of pixels within
1 / 3 level(s), the maximum, and the percentage difference; plus the event sequences. numpy only.
Run with python3 -I. This is a measurement tool; pass/fail rules are in the preregistered gate files.
"""
import argparse
import json
import os
import sys

import numpy as np


def load_updates(rec):
  ups = {}
  for line in open(os.path.join(rec, 'replay.txt')):
    t = line.split()
    if t and t[0] == 'update':
      k, idx, f, tw, th, pct = int(t[1]), int(t[2]), t[3], int(t[4]), int(t[5]), float(t[6])
      ups[k] = dict(idx=idx, file=f, tw=tw, th=th, pct=pct)
  return ups


def main():
  ap = argparse.ArgumentParser()
  ap.add_argument('--record', required=True)
  ap.add_argument('--replay', required=True)
  ap.add_argument('--json')
  a = ap.parse_args()
  ups = load_updates(a.record)
  rows = []
  for k in sorted(ups):
    u = ups[k]
    orig = np.fromfile(os.path.join(a.record, u['file']), np.uint8).reshape(u['th'], u['tw'], 4)
    mine_path = os.path.join(a.replay, 'sky', 'sky-%d.rgba' % k)
    if not os.path.exists(mine_path):
      rows.append(dict(k=k, status='missing'))
      continue
    mine = np.fromfile(mine_path, np.uint8)
    if mine.size != orig.size:
      rows.append(dict(k=k, status='size', mine=int(mine.size), orig=int(orig.size)))
      continue
    mine = mine.reshape(orig.shape)
    d = np.abs(mine.astype(int) - orig.astype(int))
    rows.append(dict(k=k, status='ok', max=int(d.max()), mean=float(d.mean()),
                     le1=float((d <= 1).mean()), le3=float((d <= 3).mean()),
                     per_channel_max=[int(d[..., c].max()) for c in range(4)],
                     orig_nonzero=int((orig[..., :3] > 0).any(axis=-1).sum())))
  ev_o = [l.split()[1:] for l in open(os.path.join(a.record, 'replay.txt')) if l.startswith('event')]
  ev_m = [l.rstrip('\n').split('\t') for l in open(os.path.join(a.replay, 'events.tsv'))]
  out = dict(frames=rows, events_original=ev_o, events_ours=ev_m)
  ok = [r for r in rows if r['status'] == 'ok']
  if ok:
    out['summary'] = dict(n=len(ok), worst_max=max(r['max'] for r in ok),
                          worst_le3=min(r['le3'] for r in ok), worst_le1=min(r['le1'] for r in ok),
                          mean_of_means=float(np.mean([r['mean'] for r in ok])))
  print(json.dumps(out.get('summary'), indent=1))
  for r in rows:
    print(r)
  print('events original', ev_o)
  print('events ours', ev_m)
  if a.json:
    if os.path.exists(a.json):
      sys.exit('refusing to overwrite ' + a.json)
    json.dump(out, open(a.json, 'w'), indent=1)


if __name__ == '__main__':
  main()

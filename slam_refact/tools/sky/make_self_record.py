#!/usr/bin/env python3
"""Self record for the null-run of the sky gate measurer (US-020; sky-us020.v1.md §5, N3).

  make_self_record.py --record <original record> --ours <sr_sky_replay inject-worker out dir> --out <new dir>
                      [--infer <sr_sky_replay infer out dir>]

Builds a record in which the 'expected' side is OUR OWN output: the update lines (texture file, response intrinsics,
rotation, position) and the event lines come from `--ours`, the invoke images from our requests (invoke/req-<n>.img.rgba),
or, with --infer instead, the invoke data / percentage from our model run on the ORIGINAL images (infer mode; the invoke
images stay the original ones, so R1 is not a self-null there). Everything else is linked to the original record. Feeding it to the gate measurer in report mode must give exactly zero differences in
R1, R3, R4, R5, R6 (and R2 with --infer): a non-zero value would be a plumbing defect of the measurer (wrong frame pairing,
wrong plane, wrong field), not a property of the engine. Run with python3 -I.
"""
import argparse
import json
import os
import sys


def main():
  ap = argparse.ArgumentParser()
  ap.add_argument('--record', required=True)
  ap.add_argument('--ours', required=True)
  ap.add_argument('--infer')
  ap.add_argument('--out', required=True)
  a = ap.parse_args()
  rec, ours, out = (os.path.abspath(p) for p in (a.record, a.ours, a.out))
  os.makedirs(out, exist_ok=False)
  for name in ('cam', 'resized'):
    os.symlink(os.path.join(rec, name), os.path.join(out, name))
  os.symlink(os.path.join(ours, 'sky'), os.path.join(out, 'selfsky'))
  os.symlink(os.path.join(ours, 'invoke'), os.path.join(out, 'selfreq'))
  os.symlink(os.path.join(rec, 'invoke'), os.path.join(out, 'invoke'))
  if a.infer:
    os.symlink(os.path.join(os.path.abspath(a.infer), 'invoke'), os.path.join(out, 'selfinfer'))
  pose = {}
  for l in open(os.path.join(ours, 'pose.tsv')):
    t = l.split()
    pose[int(t[0])] = t[1:]
  infer_pct = {}
  if a.infer:
    for l in list(open(os.path.join(os.path.abspath(a.infer), 'infer.tsv')))[1:]:
      k, p = l.split()
      infer_pct[int(k)] = p
  lines = []
  n_invoke = 0
  events_done = False
  for l in open(os.path.join(rec, 'replay.txt')):
    if not l.strip() or l.startswith('#'):
      lines.append(l.rstrip('\n'))
      continue
    t = l.split()
    if t[0] == 'event':
      if not events_done:
        events_done = True
        for e in open(os.path.join(ours, 'events.tsv')):
          k, name, detail = e.rstrip('\n').split('\t', 2)
          if name == 'layerfound':
            lines.append('event %s layerfound %r' % (k, json.loads(detail)['percentage']))
          else:
            lines.append('event %s %s' % (k, name))
      continue
    if t[0] == 'update':
      k = int(t[1])
      if k not in pose:
        sys.exit('our run has no valid output at k=%d' % k)
      t[3] = 'selfsky/sky-%d.rgba' % k
      t[7:30] = pose[k]
      lines.append(' '.join(t))
      continue
    if t[0] == 'invoke':
      k = int(t[1])
      if a.infer:  # R2 self record: the original image, OUR model's output for it
        t[10] = 'selfinfer/invoke-%d.data.f32' % k
        t[8] = infer_pct[k]
      else:  # R1 self record: OUR request image
        t[9] = 'selfreq/req-%d.img.rgba' % n_invoke
      n_invoke += 1
      lines.append(' '.join(t))
      continue
    lines.append(l.rstrip('\n'))
  open(os.path.join(out, 'replay.txt'), 'w').write('\n'.join(lines) + '\n')
  print('self record', out, 'invokes', n_invoke)


if __name__ == '__main__':
  main()

#!/usr/bin/env python3
"""Ear model exact-input comparison against the original runtime (docs/spec/08-face-ear.md §5.5, RM-7 G3).

extract: from a run-face-reference.mjs --ears 1 recording, cut the 112x160 left/right ear crops that
  the original GPU rendered (ear-rendered-%04d.rgba, viewport of each detection) for every left/right
  pair whose detections carry landmarks (5 points: 2 ROI hints + 3 landmarks {x, y, visibility}),
  and write pairs.txt for sr_face_earloop.
compare: sr_face_earloop output vs the original ear detections. Tolerances (registered in
  build/slam-refact-full-20261019/preregistration/face-ear-rm7.v1.md before the first run):
  landmark x, y <= 0.01; visibility <= 0.02; confidence <= 0.02 (v1; 'pass'). Version 2
  (face-ear-rm7.v2.md, after the first run: the original ear confidence is a constant 1, not the
  model output) drops the confidence item ('pass_v2'). --self-check null replaces "ours" by the
  original values (null-run).
crop-diff: GPU ear crops vs the CPU renderer crops (sr_face_earloop --cpu-crops-dir).
numpy only; run with python3 -I.
"""
import argparse
import json
import os
import sys

import numpy as np

W, H = 112, 160
LM_TOL, VIS_TOL, CONF_TOL = 0.01, 0.02, 0.02


def pairs_of(ref_dir):
  """Left/right ear pairs whose detections carry model landmarks.

  The ear crops analysed in a callback were drawn with the ear ROIs set two callbacks earlier
  (docs/spec/08-face-ear.md §3.1), so the GPU crop of ear i sits at EarViewportForCrop(i) of the
  callback's ear render (one face: left = slot 0, right = slot 1; the detection list of the
  callback carries the *new* ROIs, whose `viewport` field is the face viewport and is not used).
  The ROI that drew the crop is the one of the detections of callback staged - 2 (cpu_roi), used
  only for the CPU-renderer crop comparison.
  """
  state = json.load(open(os.path.join(ref_dir, 'result.json')))['state']
  seen = set()
  by_staged = {}
  out = []
  for row_index, row in enumerate(state['updates']):
    if row['repeat'] or not row['hasOutput'] or row['staged'] in seen or 'earDetections' not in row:
      continue
    seen.add(row['staged'])
    ears = json.loads(row['earDetections'])['ears']
    by_staged[row['staged']] = ears
    if 'earRendered' not in row:
      continue
    assert len(ears) <= 2, 'multi-face ear lists are not supported by this extractor'
    if len(ears) != 2 or len(ears[0]['points']) != 5 or len(ears[1]['points']) != 5:
      continue
    earlier = by_staged.get(row['staged'] - 2)
    out.append({'pair': len(out), 'row': row_index, 'staged': row['staged'],
                'texture': row['staged'] - 2, 'left': ears[0], 'right': ears[1],
                'cpu_roi': earlier if earlier and len(earlier) == 2 else None})
  return out


def landmarks(det):
  return np.array([[p['x'], p['y'], p['z']] for p in det['points'][2:5]], dtype=np.float64)


def main():
  ap = argparse.ArgumentParser(description=__doc__)
  ap.add_argument('mode', choices=['extract', 'compare', 'crop-diff'])
  ap.add_argument('--reference', required=True)
  ap.add_argument('--out-dir')
  ap.add_argument('--ours')
  ap.add_argument('--gpu-crops')
  ap.add_argument('--cpu-crops')
  ap.add_argument('--self-check', choices=['null'])
  ap.add_argument('--out')
  a = ap.parse_args()
  pairs = pairs_of(a.reference)
  if a.mode == 'extract':
    os.makedirs(a.out_dir, exist_ok=False)
    with open(os.path.join(a.out_dir, 'pairs.txt'), 'x') as f:
      for p in pairs:
        img = np.fromfile(os.path.join(a.reference, 'ear-rendered', 'ear-rendered-%04d.rgba' % p['row']),
                          dtype=np.uint8).reshape(320, 336, 4)
        for side, name in (('left', 'L'), ('right', 'R')):
          x, y = 0, (0 if side == 'left' else H)  # EarViewportForCrop(0) and (1)
          img[y:y + H, x:x + W].tofile(os.path.join(a.out_dir, 'ear-pair-%04d-%s.rgba' % (p['pair'], name)))
        src = p['cpu_roi'] or [p['left'], p['right']]  # without the earlier ROI the CPU crop is unused
        ml = src[0]['roi']['renderTexToImageTexMatrix44f']
        mr = src[1]['roi']['renderTexToImageTexMatrix44f']
        f.write('%d %d %s %s\n' % (p['pair'], p['texture'],
                                   ' '.join('%.9g' % v for v in ml), ' '.join('%.9g' % v for v in mr)))
    json.dump([{'pair': p['pair'], 'row': p['row'], 'staged': p['staged'], 'texture': p['texture'],
                'cpu_roi_available': p['cpu_roi'] is not None} for p in pairs],
              open(os.path.join(a.out_dir, 'pairs.json'), 'x'), indent=1)
    print(json.dumps({'pairs': len(pairs), 'with_cpu_roi': sum(p['cpu_roi'] is not None for p in pairs),
                      'out_dir': a.out_dir}))
    return
  if os.path.exists(a.out):
    sys.exit('output exists')
  if a.mode == 'crop-diff':
    rows = []
    for p in pairs:
      if p['cpu_roi'] is None:
        continue
      for name in ('L', 'R'):
        g = np.fromfile(os.path.join(a.gpu_crops, 'ear-pair-%04d-%s.rgba' % (p['pair'], name)),
                        dtype=np.uint8).reshape(H, W, 4)[..., :3].astype(np.int32)
        c = np.fromfile(os.path.join(a.cpu_crops, 'cpu-ear-%04d-%s.rgba' % (p['pair'], name)),
                        dtype=np.uint8).reshape(H, W, 4)[..., :3].astype(np.int32)
        d = np.abs(g - c)
        rows.append({'pair': p['pair'], 'side': name, 'frac_equal': float((d == 0).mean()),
                     'frac_le1': float((d <= 1).mean()), 'max': int(d.max()),
                     'pixels_gt3': int((d.max(axis=2) > 3).sum())})
    summary = {'crops': len(rows), 'min_frac_equal': min(r['frac_equal'] for r in rows),
               'min_frac_le1': min(r['frac_le1'] for r in rows), 'max': max(r['max'] for r in rows),
               'crops_with_pixel_gt3': sum(r['pixels_gt3'] > 0 for r in rows), 'rows': rows}
    json.dump(summary, open(a.out, 'w'), indent=1)
    print(json.dumps({k: v for k, v in summary.items() if k != 'rows'}, indent=1))
    return
  ours = {}
  if a.self_check == 'null':
    for p in pairs:
      ours[p['pair']] = {side: {'confidence': p[side]['confidence'],
                                'points': landmarks(p[side]).tolist()} for side in ('left', 'right')}
  else:
    for line in open(a.ours):
      o = json.loads(line)
      ours[o['pair']] = o
  rows = []
  for p in pairs:
    o = ours[p['pair']]
    row = {'pair': p['pair'], 'staged': p['staged']}
    ok = True       # v1: landmark, visibility and confidence
    ok_v2 = True    # v2 (face-ear-rm7.v2.md): the original's ear confidence is a constant ROI hint
    for side in ('left', 'right'):
      want = landmarks(p[side])
      got = np.array(o[side]['points'], dtype=np.float64)
      lm = float(np.abs(want[:, :2] - got[:, :2]).max())
      vis = float(np.abs(want[:, 2] - got[:, 2]).max())
      conf = abs(p[side]['confidence'] - o[side]['confidence'])
      row[side] = {'max_landmark': lm, 'max_visibility': vis, 'confidence_diff': conf}
      ok = ok and lm <= LM_TOL and vis <= VIS_TOL and conf <= CONF_TOL
      ok_v2 = ok_v2 and lm <= LM_TOL and vis <= VIS_TOL
    row['pass'] = ok
    row['pass_v2'] = ok_v2
    rows.append(row)
  worst = lambda key: max(max(r['left'][key], r['right'][key]) for r in rows) if rows else None
  summary = {'pairs': len(rows), 'pass': sum(r['pass'] for r in rows),
             'pass_v2': sum(r['pass_v2'] for r in rows),
             'max_landmark': worst('max_landmark'), 'max_visibility': worst('max_visibility'),
             'max_confidence_diff': worst('confidence_diff'),
             'original_visibility_max': max((float(landmarks(p[s])[:, 2].max()) for p in pairs for s in ('left', 'right')), default=None),
             'tolerances': {'landmark': LM_TOL, 'visibility': VIS_TOL, 'confidence': CONF_TOL},
             'rows': rows}
  json.dump(summary, open(a.out, 'w'), indent=1)
  print(json.dumps({k: v for k, v in summary.items() if k != 'rows'}, indent=1))


if __name__ == '__main__':
  main()

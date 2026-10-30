#!/usr/bin/env python3
"""Open-loop mesh diagnostics (docs/spec/08-face-ear.md §5.2 T7e and §5.5 RM-7 G2).

extract: original mesh detections -> rois.txt for sr_face_openloop.
extract-gpu-crops: the 192x192 crop of every original mesh detection cut out of the original GPU
  render (rendered-%04d.rgba, viewport of the detection) -> gpu-crop-%04d.rgba, the exact input of
  the original FaceDetectorLocal (sr_face_openloop --gpu-crops-dir).
crop-diff: per-crop pixel difference between those GPU crops and the CPU re-rendered crops
  (sr_face_openloop --crops-dir).
compare: sr_face_openloop output vs the original mesh detections (tolerances of T7c). --self-check
  null replaces "ours" by the original points (null-run); --self-check shift1 pairs detection i
  with the output of detection i+1 (mutation: the tolerances must then reject).
numpy only; run with python3 -I.
"""
import argparse
import json
import os
import sys

import numpy as np

COORD, CONF = 0.01, 0.1


def originals(ref_dir):
  state = json.load(open(os.path.join(ref_dir, 'result.json')))['state']
  seen = set()
  out = []
  for row_index, row in enumerate(state['updates']):
    if row['repeat'] or not row['hasOutput'] or row['staged'] in seen or 'detections' not in row:
      continue
    seen.add(row['staged'])
    for d in json.loads(row['detections'])['meshes']:
      # The callback with staged = k processed texture k - 2 (two-stage pipeline).
      out.append({'id': len(out), 'texture': row['staged'] - 2, 'row': row_index, 'det': d})
  return out


def main():
  ap = argparse.ArgumentParser(description=__doc__)
  ap.add_argument('mode', choices=['extract', 'extract-gpu-crops', 'crop-diff', 'compare'])
  ap.add_argument('--reference', required=True)
  ap.add_argument('--rois')
  ap.add_argument('--openloop')
  ap.add_argument('--out')
  ap.add_argument('--out-dir')
  ap.add_argument('--gpu-crops')
  ap.add_argument('--cpu-crops')
  ap.add_argument('--self-check', choices=['null', 'shift1'])
  a = ap.parse_args()
  dets = originals(a.reference)
  if a.mode == 'extract-gpu-crops':
    os.makedirs(a.out_dir, exist_ok=False)
    for d in dets:
      vp = d['det']['viewport']
      img = np.fromfile(os.path.join(a.reference, 'rendered', 'rendered-%04d.rgba' % d['row']),
                        dtype=np.uint8).reshape(384, 384, 4)
      x, y = int(vp['offsetX']), int(vp['offsetY'])
      assert vp['width'] == 192 and vp['height'] == 192
      img[y:y + 192, x:x + 192].tofile(os.path.join(a.out_dir, 'gpu-crop-%04d.rgba' % d['id']))
    print(json.dumps({'crops': len(dets), 'out_dir': a.out_dir}))
    return
  if a.mode == 'crop-diff':
    rows = []
    for d in dets:
      g = np.fromfile(os.path.join(a.gpu_crops, 'gpu-crop-%04d.rgba' % d['id']),
                      dtype=np.uint8).reshape(192, 192, 4)[..., :3].astype(np.int32)
      cpu = np.fromfile(os.path.join(a.cpu_crops, 'crop-%04d.rgba' % d['id']),
                        dtype=np.uint8).reshape(192, 192, 4)[..., :3].astype(np.int32)
      diff = np.abs(g - cpu)
      rows.append({'id': d['id'], 'row': d['row'], 'texture': d['texture'],
                   'frac_equal': float((diff == 0).mean()), 'frac_le1': float((diff <= 1).mean()),
                   'max': int(diff.max()), 'pixels_gt3': int((diff.max(axis=2) > 3).sum())})
    summary = {'crops': len(rows), 'min_frac_equal': min(r['frac_equal'] for r in rows),
               'min_frac_le1': min(r['frac_le1'] for r in rows), 'max': max(r['max'] for r in rows),
               'crops_with_pixel_gt3': sum(r['pixels_gt3'] > 0 for r in rows),
               'crops_all_equal': sum(r['max'] == 0 for r in rows), 'rows': rows}
    if os.path.exists(a.out):
      sys.exit('output exists')
    json.dump(summary, open(a.out, 'w'), indent=1)
    print(json.dumps({k: v for k, v in summary.items() if k != 'rows'}, indent=1))
    return
  if a.mode == 'extract':
    with open(a.rois, 'x') as f:
      for d in dets:
        m = d['det']['roi']['renderTexToImageTexMatrix44f']
        f.write('%d %d %s\n' % (d['id'], d['texture'], ' '.join('%.9g' % v for v in m)))
    return
  ours = {}
  if a.self_check == 'null':
    for d in dets:
      ours[d['id']] = {'id': d['id'], 'detected': True, 'confidence': d['det']['confidence'],
                       'points': [[p['x'], p['y']] for p in d['det']['points']]}
  else:
    for line in open(a.openloop):
      o = json.loads(line)
      ours[o['id']] = o
    if a.self_check == 'shift1':
      ours = {i: ours[(i + 1) % len(ours)] for i in ours}
  rows = []
  for d in dets:
    o = ours.get(d['id'])
    pa = np.array([[p['x'], p['y']] for p in d['det']['points']])
    row = {'id': d['id'], 'texture': d['texture'], 'orig_conf': d['det']['confidence']}
    if o is None or not o['detected']:
      row['detected'] = False
    else:
      pb = np.array(o['points'])
      row.update(detected=True, conf=o['confidence'],
                 conf_diff=abs(o['confidence'] - d['det']['confidence']),
                 max_coord=float(np.abs(pa - pb).max()), mean_coord=float(np.abs(pa - pb).mean()))
    row['pass'] = row['detected'] and row['max_coord'] <= COORD and row['conf_diff'] <= CONF
    rows.append(row)
  summary = {'detections': len(rows), 'pass': sum(r['pass'] for r in rows),
             'not_detected': sum(not r['detected'] for r in rows),
             'max_coord': max((r.get('max_coord', 0.0) for r in rows), default=0.0),
             'max_conf_diff': max((r.get('conf_diff', 0.0) for r in rows), default=0.0),
             'tolerances': [COORD, CONF], 'rows': rows}
  if os.path.exists(a.out):
    sys.exit('output exists')
  json.dump(summary, open(a.out, 'w'), indent=1)
  print(json.dumps({k: v for k, v in summary.items() if k != 'rows'}, indent=1))


if __name__ == '__main__':
  main()

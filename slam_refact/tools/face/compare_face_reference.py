#!/usr/bin/env python3
"""Compare the original xr-face recording with the slam_refact replay (docs/spec/08-face-ear.md §5.1).

Inputs:
  --reference <dir>   run-face-reference.mjs output (result.json, rendered/rendered-%04d.rgba)
  --replay <jsonl>    sr_face_replay output on the recorded textures of that run
  --replay-rendered <dir>  sr_face_replay --rendered-dir output
  --out <json>        report (new file)
Pairing: an original non-repeat callback with staged = k pairs with replay line index k - 1.
Tolerances are the pre-registered T7a-T7d values; this script does not take tolerance arguments.
numpy only; run with python3 -I.
"""
import argparse
import json
import math
import os
import sys

import numpy as np

T7A_FRAC_LE1 = 0.99
T7A_MAX = 3
T7B_COORD = 0.01
T7B_CONF = 0.02
T7C_COORD = 0.01
T7C_CONF = 0.1
T7D_VERT = 0.01
T7D_POS = 0.005
T7D_ROT_DEG = 1.0

FACE_EVENTS = ('facecontroller.facefound', 'facecontroller.faceupdated', 'facecontroller.facelost')


def orig_detection(d):
  c = d['roi']['corners']
  corners = [[c[k]['x'], c[k]['y']] for k in ('upperLeft', 'upperRight', 'lowerLeft', 'lowerRight')]
  return {'confidence': d['confidence'], 'corners': corners,
          'points': [[p['x'], p['y'], p['z']] for p in d['points']]}


def ours_detection(d):
  return {'confidence': d['confidence'], 'corners': d['corners'], 'points': d['points']}


def compare_detections(a, b, coord_tol, conf_tol):
  res = {'count_a': len(a), 'count_b': len(b), 'max_coord': 0.0, 'max_conf': 0.0}
  if len(a) != len(b):
    res['pass'] = False
    return res
  for x, y in zip(a, b):
    res['max_conf'] = max(res['max_conf'], abs(x['confidence'] - y['confidence']))
    ca = np.array(x['corners'], dtype=np.float64)
    cb = np.array(y['corners'], dtype=np.float64)
    res['max_coord'] = max(res['max_coord'], float(np.abs(ca - cb).max()))
    pa = np.array(x['points'], dtype=np.float64)[:, :2]
    pb = np.array(y['points'], dtype=np.float64)[:, :2]
    if pa.shape != pb.shape:
      res['pass'] = False
      return res
    if pa.size:
      res['max_coord'] = max(res['max_coord'], float(np.abs(pa - pb).max()))
  res['pass'] = res['max_coord'] <= coord_tol and res['max_conf'] <= conf_tol
  return res


def region_stats(a, b, x, y, w, h):
  ra = a[y:y + h, x:x + w, :3].astype(np.int32)
  rb = b[y:y + h, x:x + w, :3].astype(np.int32)
  d = np.abs(ra - rb).max(axis=2)
  return {'frac_le1': float((d <= 1).mean()), 'max': int(d.max()), 'mean': float(d.mean())}


def quat_angle_deg(a, b):
  qa = np.array([a['w'], a['x'], a['y'], a['z']], dtype=np.float64)
  qb = np.array([b['w'], b['x'], b['y'], b['z']], dtype=np.float64)
  qa /= np.linalg.norm(qa)
  qb /= np.linalg.norm(qb)
  return math.degrees(2.0 * math.acos(min(1.0, abs(float(qa @ qb)))))


def main():
  ap = argparse.ArgumentParser(description=__doc__)
  ap.add_argument('--reference', required=True)
  ap.add_argument('--replay', required=True)
  ap.add_argument('--replay-rendered', required=True)
  ap.add_argument('--out', required=True)
  a = ap.parse_args()
  if os.path.exists(a.out):
    sys.exit('output exists: ' + a.out)
  ref = json.load(open(os.path.join(a.reference, 'result.json')))
  state = ref['state']
  replay = {}
  for line in open(a.replay):
    d = json.loads(line)
    if 'index' in d:
      replay[d['index']] = d

  report = {'reference': a.reference, 'replay': a.replay, 'tolerances': {
      'T7a': [T7A_FRAC_LE1, T7A_MAX], 'T7b': [T7B_COORD, T7B_CONF], 'T7c': [T7C_COORD, T7C_CONF],
      'T7d': [T7D_VERT, T7D_POS, T7D_ROT_DEG]}}
  t7a = {'frames': 0, 'fail_frames': [], 'full': {'min_frac_le1': 1.0, 'max': 0, 'mean_sum': 0.0},
         'crops': {'n': 0, 'min_frac_le1': 1.0, 'max': 0}}
  t7b = {'frames': 0, 'fail_frames': [], 'max_coord': 0.0, 'max_conf': 0.0, 'detections': 0}
  t7c = {'frames': 0, 'fail_frames': [], 'max_coord': 0.0, 'max_conf': 0.0, 'detections': 0}
  seen_staged = set()
  for row_index, row in enumerate(state['updates']):
    if row['repeat'] or not row['hasOutput'] or row['staged'] in seen_staged:
      continue
    seen_staged.add(row['staged'])
    i = row['staged'] - 1
    ours = replay.get(i)
    if ours is None or ours['response'] is None:
      continue
    # T7a: rendered images.
    rpath = os.path.join(a.reference, 'rendered', 'rendered-%04d.rgba' % row_index)
    opath = os.path.join(a.replay_rendered, 'rendered-%04d.rgba' % i)
    if 'rendered' in row and os.path.exists(rpath) and os.path.exists(opath):
      ra = np.fromfile(rpath, dtype=np.uint8).reshape(384, 384, 4)
      rb = np.fromfile(opath, dtype=np.uint8).reshape(384, 384, 4)
      full = region_stats(ra, rb, 16, 0, 96, 128)
      t7a['frames'] += 1
      t7a['full']['min_frac_le1'] = min(t7a['full']['min_frac_le1'], full['frac_le1'])
      t7a['full']['max'] = max(t7a['full']['max'], full['max'])
      t7a['full']['mean_sum'] += full['mean']
      ok = full['frac_le1'] >= T7A_FRAC_LE1 and full['max'] <= T7A_MAX
      # Crop slots: compare every 192x192 slot that is not black in either image.
      for (x, y) in ((192, 0), (0, 192), (192, 192)):
        if ra[y:y + 192, x:x + 192, :3].any() or rb[y:y + 192, x:x + 192, :3].any():
          c = region_stats(ra, rb, x, y, 192, 192)
          t7a['crops']['n'] += 1
          t7a['crops']['min_frac_le1'] = min(t7a['crops']['min_frac_le1'], c['frac_le1'])
          t7a['crops']['max'] = max(t7a['crops']['max'], c['max'])
          ok = ok and c['frac_le1'] >= T7A_FRAC_LE1 and c['max'] <= T7A_MAX
      if not ok:
        t7a['fail_frames'].append(i)
    # T7b / T7c: detections.
    dets = json.loads(row.get('detections', '{"faces":[],"meshes":[]}'))
    g = compare_detections([orig_detection(d) for d in dets['faces']],
                           [ours_detection(d) for d in ours['response']['globalDetections']],
                           T7B_COORD, T7B_CONF)
    t7b['frames'] += 1
    t7b['detections'] += g['count_a']
    t7b['max_coord'] = max(t7b['max_coord'], g['max_coord'])
    t7b['max_conf'] = max(t7b['max_conf'], g['max_conf'])
    if not g['pass']:
      t7b['fail_frames'].append({'index': i, **g})
    m = compare_detections([orig_detection(d) for d in dets['meshes']],
                           [ours_detection(d) for d in ours['response']['meshDetections']],
                           T7C_COORD, T7C_CONF)
    t7c['frames'] += 1
    t7c['detections'] += m['count_a']
    t7c['max_coord'] = max(t7c['max_coord'], m['max_coord'])
    t7c['max_conf'] = max(t7c['max_conf'], m['max_conf'])
    if not m['pass']:
      t7c['fail_frames'].append({'index': i, **m})
  if t7a['frames']:
    t7a['full']['mean'] = t7a['full'].pop('mean_sum') / t7a['frames']
  t7a['pass'] = t7a['frames'] > 0 and not t7a['fail_frames']
  t7b['pass'] = t7b['frames'] > 0 and not t7b['fail_frames']
  t7c['pass'] = t7c['frames'] > 0 and not t7c['fail_frames']

  # T7d: events. Original face events re-dispatched on repeat callbacks (same staged count, same
  # detail) are collapsed; faceloading/facescanning (attach) are compared separately.
  orig = []
  last_face_event = {}
  for e in state['events']:
    if e['name'] in ('facecontroller.faceloading', 'facecontroller.facescanning'):
      continue
    if e['name'] in FACE_EVENTS:
      key = (e['staged'], e['name'], e['detail'])
      if key in last_face_event:
        continue
      last_face_event[key] = True
    orig.append(e)
  ours_events = []
  for i in sorted(replay):
    for e in replay[i]['events']:
      ours_events.append({'index': i, 'name': e['name'], 'detail': e['detail']})
  names_a = [e['name'] for e in orig]
  names_b = [e['name'] for e in ours_events]
  t7d = {'events_original': len(names_a), 'events_ours': len(names_b),
         'names_equal': names_a == names_b, 'max_vertex': 0.0, 'max_position': 0.0,
         'max_rotation_deg': 0.0, 'face_events_compared': 0}
  def compare_face_events(pairs, out):
    for x, y in pairs:
      if x['name'] not in ('facecontroller.facefound', 'facecontroller.faceupdated'):
        continue
      da = json.loads(x['detail'])
      db = y['detail']
      va = np.array([[v['x'], v['y'], v['z']] for v in da['vertices']])
      vb = np.array([[v['x'], v['y'], v['z']] for v in db['vertices']])
      out['max_vertex'] = max(out['max_vertex'], float(np.abs(va - vb).max()))
      pa = da['transform']['position']
      pb = db['transform']['position']
      out['max_position'] = max(out['max_position'],
                                max(abs(pa[k] - pb[k]) for k in ('x', 'y', 'z')))
      out['max_rotation_deg'] = max(out['max_rotation_deg'],
                                    quat_angle_deg(da['transform']['rotation'],
                                                   db['transform']['rotation']))
      out['face_events_compared'] += 1

  if names_a != names_b:
    first = next((k for k in range(min(len(names_a), len(names_b))) if names_a[k] != names_b[k]),
                 min(len(names_a), len(names_b)))
    t7d['first_mismatch'] = {'position': first, 'original': names_a[first:first + 6],
                             'ours': names_b[first:first + 6]}
    # RM-7: numeric comparison of the facefound/faceupdated events before the first name
    # mismatch (report only; the T7d verdict stays names_equal and the registered tolerances).
    prefix = {'events': first, 'max_vertex': 0.0, 'max_position': 0.0, 'max_rotation_deg': 0.0,
              'face_events_compared': 0,
              'same_staged_pairing': all(orig[k]['staged'] - 1 == ours_events[k]['index']
                                         for k in range(first))}
    compare_face_events(zip(orig[:first], ours_events[:first]), prefix)
    t7d['prefix_before_first_mismatch'] = prefix
  else:
    compare_face_events(zip(orig, ours_events), t7d)
  t7d['pass'] = (t7d['names_equal'] and t7d['max_vertex'] <= T7D_VERT and
                 t7d['max_position'] <= T7D_POS and t7d['max_rotation_deg'] <= T7D_ROT_DEG)
  report.update({'T7a': t7a, 'T7b': t7b, 'T7c': t7c, 'T7d': t7d})
  with open(a.out, 'w') as f:
    json.dump(report, f, indent=1)
  print(json.dumps({k: {kk: vv for kk, vv in report[k].items() if kk != 'fail_frames'} | {
      'fail_count': len(report[k].get('fail_frames', []))} for k in ('T7a', 'T7b', 'T7c', 'T7d')},
                   indent=1))


if __name__ == '__main__':
  main()

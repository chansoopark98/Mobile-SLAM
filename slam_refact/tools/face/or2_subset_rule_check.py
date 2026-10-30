#!/usr/bin/env python3
"""OR-2: execution check of the Landmarks2TransformMatrix odd-subset rule (docs/spec/08-face-ear.md §5.3).

The face-mesh model refines the eyes (and irises) on crops placed by Landmarks2TransformMatrix ops.
Their option `subset_idxs` is a flat index list turned into pairs; for an odd length the parser of
the original xr-face.wasm (function 685, decompiled) appends (last, last). This script runs the
model with the numpy reference (tools/ml_ref) under two rules on the crops the original runtime
used (CPU re-render of the recorded camera texture with the original ROI; sr_face_openloop
--crops-dir), and compares the eye/iris points with the original runtime's mesh detections:
  rule A = (last, last) appended (current implementation), rule B = trailing index dropped.
For each detection: N = noise floor = max |orig - A| over the mesh points the attention heads do
not refine (unaffected by the rule); D = max |A - B| over eye+iris points; E_A, E_B = max error of
each rule against the original over eye+iris points. Verdict rules are fixed in §5.3.
numpy only; run with python3 -I.
"""
import argparse
import json
import os
import re
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, '..', '..', '..'))
sys.path.insert(0, os.path.join(REPO, 'slam_refact', 'tools', 'ml_ref'))
import ref_ops  # noqa: E402
from tflite_reader import read_model  # noqa: E402

FACEMESH_DATA = os.path.join(REPO, 'slam_refact/engine/third_party/xr_source/c8/geometry/facemesh-data.h')
MESH_MODEL = os.path.join(REPO, 'assets/references/xr_reference/engine/resources/face-mesh-model.tflite')


def index_list(name):
  text = open(FACEMESH_DATA).read()
  m = re.search(r'const Vector<int> %s = \{([^}]*)\}' % name, text)
  return [int(v) for v in re.findall(r'-?\d+', re.sub(r'//[^\n]*', '', m.group(1)))]


def make_variant(original):
  def drop_last(lm, p):
    q = dict(p)
    s = list(q['subset_idxs'])
    if len(s) % 2:
      s = s[:-1]
    q['subset_idxs'] = s
    return original(lm, q)
  return drop_last


def main():
  ap = argparse.ArgumentParser(description=__doc__)
  ap.add_argument('--reference', required=True, help='run-face-reference.mjs output dir')
  ap.add_argument('--crops', required=True, help='sr_face_openloop --crops-dir output')
  ap.add_argument('--out', required=True)
  a = ap.parse_args()
  if os.path.exists(a.out):
    sys.exit('output exists: ' + a.out)
  sys.path.insert(0, HERE)
  from openloop_mesh import originals  # noqa: E402

  lips = index_list('FACEMESH_ATTENTION_LIPS_INDICES')
  leye = index_list('FACEMESH_ATTENTION_LEFT_EYE_INDICES')
  reye = index_list('FACEMESH_ATTENTION_RIGHT_EYE_INDICES')
  assert len(lips) == 80 and len(leye) == 71 and len(reye) == 71
  model = read_model(MESH_MODEL)
  subsets = []
  for op in model.ops:
    if op.name == 'Landmarks2TransformMatrix':
      subsets.append(len(ref_ops.flex_decode(op.custom_options)['subset_idxs']))
  interp = ref_ops.RefInterpreter(model)
  outs = model.outputs  # mesh, lips, left eye, right eye, left iris, right iris, faceflag
  rule_a = ref_ops.landmarks_to_transform_matrix
  rule_b = make_variant(rule_a)
  refined = set(lips) | set(leye) | set(reye)
  plain = [i for i in range(468) if i not in refined]
  eye_iris = leye + reye + list(range(468, 478))

  def points(vals):
    scale = 1.0 / 191.0
    mesh = vals[outs[0]].reshape(-1, 3)[:, :2] * scale
    pts = np.zeros((478, 2))
    pts[:468] = mesh
    for idxs, o in ((lips, 1), (leye, 2), (reye, 3)):
      pts[idxs] = vals[outs[o]].reshape(-1, 2) * scale
    pts[468:473] = vals[outs[4]].reshape(-1, 2) * scale
    pts[473:478] = vals[outs[5]].reshape(-1, 2) * scale
    return pts

  rows = []
  for d in originals(a.reference):
    crop = np.fromfile(os.path.join(a.crops, 'crop-%04d.rgba' % d['id']), dtype=np.uint8)
    rgb = crop.reshape(192, 192, 4)[..., :3].astype(np.float32) / np.float32(255.0)
    x = rgb.reshape(1, 192, 192, 3)
    ref_ops.landmarks_to_transform_matrix = rule_a
    pa = points(interp.run([x]))
    ref_ops.landmarks_to_transform_matrix = rule_b
    pb = points(interp.run([x]))
    ref_ops.landmarks_to_transform_matrix = rule_a
    po = np.array([[p['x'], p['y']] for p in d['det']['points']])
    rows.append({
        'id': d['id'], 'texture': d['texture'],
        'N': float(np.abs(po[plain] - pa[plain]).max()),
        'D': float(np.abs(pa[eye_iris] - pb[eye_iris]).max()),
        'E_A': float(np.abs(po[eye_iris] - pa[eye_iris]).max()),
        'E_B': float(np.abs(po[eye_iris] - pb[eye_iris]).max()),
    })
  disc = [r for r in rows if r['D'] > 3.0 * r['N']]
  verdict = ('NON_DISCRIMINATING' if not disc else
             'A_CONFIRMED' if all(r['E_A'] < r['E_B'] for r in disc) else
             'B_CONFIRMED' if all(r['E_B'] < r['E_A'] for r in disc) else 'MIXED')
  report = {'subset_lengths': subsets, 'detections': len(rows), 'discriminating': len(disc),
            'verdict': verdict, 'rows': rows}
  json.dump(report, open(a.out, 'w'), indent=1)
  print(json.dumps({k: v for k, v in report.items() if k != 'rows'}, indent=1))


if __name__ == '__main__':
  main()

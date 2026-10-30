#!/usr/bin/env python3
"""Expected post-processing results for REAL palm model outputs (US-021, gate HAND-G3).

  gen_real_tensor_expected.py --dump <dir written by sr_hand_run --dump> --size <w> <h> --out <new dir>

For every frame k with <dir>/palm-boxes-k.f32 and palm-scores-k.f32 (the raw outputs of the palm model on
a real frame, written by the C++ engine), runs hand_ref.py (numpy second implementation) decode + weighted NMS +
letterbox removal + DetectionsToRects + RectTransformation and writes <out>/real_cases.tsv. The C++ gate
recomputes the same from the same raw tensors.
The sealing step (seal_gate_set.sh) writes the SHA256SUMS of each gate data directory.
numpy only; run with python3 -I.
"""
import argparse
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import hand_ref as H  # noqa: E402

f32 = np.float32


def main():
  ap = argparse.ArgumentParser()
  ap.add_argument('--dump', required=True)
  ap.add_argument('--size', required=True, nargs=2, type=int)
  ap.add_argument('--out', required=True)
  a = ap.parse_args()
  w, h = a.size
  os.makedirs(a.out, exist_ok=False)
  anchors = H.generate_anchors()
  ia = f32(w) / f32(h)
  pad = [0.0, 0.0, 0.0, 0.0]
  if ia < 1:
    pad[0] = pad[2] = float((f32(1) - ia / f32(1)) / f32(2))
  elif ia > 1:
    pad[1] = pad[3] = float((f32(1) - f32(1) / ia) / f32(2))
  ks = sorted(int(f.split('-')[2].split('.')[0]) for f in os.listdir(a.dump) if f.startswith('palm-scores-'))
  lines = ['# frame k, #decoded, #retained | letterbox padding l t r b | w h | det lines: score xmin ymin w h kp0x..kp6y | rect lines: palm rect x y w h rot']
  for k in ks:
    bpath = os.path.join(a.dump, 'palm-boxes-%d.f32' % k)
    spath = os.path.join(a.dump, 'palm-scores-%d.f32' % k)
    boxes = np.fromfile(bpath, np.float32).reshape(2944, 18)
    scores = np.fromfile(spath, np.float32).reshape(2944)
    dets = H.decode_and_filter(boxes, scores, anchors)
    final = H.remove_letterbox_det(H.weighted_nms(dets), pad)
    lines.append('frame\t%d\t%d\t%d\t%r\t%r\t%r\t%r\t%d\t%d' % (k, len(dets), len(final), *pad, w, h))
    for d in final:
      lines.append('det\t%r\t%r\t%r\t%r\t%r\t' % (d['score'], d['xmin'], d['ymin'], d['w'], d['h']) + '\t'.join('%r\t%r' % kp for kp in d['kp']))
    if final:
      r = H.detection_to_rect(final[0], w, h, 0, 2, 90.0)
      t = H.transform_rect(r, w, h, 2.6, 2.6, 0.0, -0.5, True)
    else:
      t = dict(x=0.0, y=0.0, w=0.0, h=0.0, rot=0.0)
      t = H.transform_rect(t, w, h, 2.6, 2.6, 0.0, -0.5, True)
    lines.append('rect\t%r\t%r\t%r\t%r\t%r' % (t['x'], t['y'], t['w'], t['h'], t['rot']))
  open(os.path.join(a.out, 'real_cases.tsv'), 'w').write('\n'.join(lines) + '\n')
  print('frames', len(ks))


if __name__ == '__main__':
  main()

#!/usr/bin/env python3
"""B6 / RM-2 (exploratory): replay the body of the public test TestInvoke of
PUB:reality/engine/deepnets/tflite-interpreter-test.cc with the real faces.jpg (LFS) in numpy.
Run with `python3 -I`.

Public test: crop rows 0..128, cols 110..238 of faces.jpg (244x128), x/127.5 - 1, face_detection_front.tflite,
count class logits > -log((1-0.75)/0.75) -> expected 10 hits; black image -> 0; white image -> 0.
Deviation (important): the model the public test loads (third_party/mediapipe/models/face_detection_front.tflite,
LFS pointer oid 0d800dd3..., 423,340 B) was NOT fetched (B1c models are excluded). The substitute is the
mediapipe-fork copy (423,256 B, sha256 a31cce73...), a different file. The runtime is the numpy float64 reference
interpreter of tools/ml_ref, not the TFLite C++ interpreter. So an agreement says "consistent with the public
expected value", not "the public test passes".
Usage: blazeface_probe.py <faces.jpg> <out.json>
"""
import hashlib
import json
import math
import os
import sys

import numpy as np
from PIL import Image

HERE = os.path.dirname(os.path.abspath(__file__))
TOOLS = os.path.abspath(os.path.join(HERE, '..', '..'))
REPO = os.path.abspath(os.path.join(TOOLS, '..', '..'))
sys.path.insert(0, os.path.join(TOOLS, 'ml_ref'))
from ref_ops import RefInterpreter  # noqa: E402
from tflite_reader import read_model  # noqa: E402

MODEL = os.path.join(REPO, 'assets/references/xr_reference/official/mediapipe-fork/mediapipe/models/face_detection_front.tflite')


def hits(interp, m, x, st):
  vals = interp.run([x])
  cls = None
  for oi in m.outputs:
    v = vals[oi]
    if v.ndim == 3 and v.shape[2] == 1:
      cls = v.reshape(-1)
  assert cls is not None
  order = np.sort(cls)[::-1]
  return int((cls > st).sum()), [round(float(v), 3) for v in order[:12]]


def main():
  img_path, out_path = sys.argv[1:3]
  m = read_model(MODEL)
  interp = RefInterpreter(m)
  st = -math.log((1.0 - 0.75) / 0.75)
  with Image.open(img_path) as im:
    rgb = np.asarray(im.convert('RGB'))
  crop = rgb[0:128, 110:238]
  assert crop.shape == (128, 128, 3), crop.shape
  res = {'model': os.path.relpath(MODEL, REPO), 'model_sha256': hashlib.sha256(open(MODEL, 'rb').read()).hexdigest(),
         'image': 'faces.jpg', 'image_size': [rgb.shape[1], rgb.shape[0]], 'threshold_logit': round(st, 6),
         'public_expected': {'face_crop_hits': 10, 'black_hits': 0, 'white_hits': 0}}
  for name, arr in (('face_crop', crop.astype(np.float64) / 127.5 - 1.0),
                    ('black', np.full((128, 128, 3), -1.0)), ('white', np.full((128, 128, 3), 1.0))):
    n, top = hits(interp, m, arr[None], st)
    res[name] = {'hits': n, 'top12_logits': top}
    print(name, n, top[:6], flush=True)
  json.dump(res, open(out_path, 'w'), indent=1)


if __name__ == '__main__':
  main()

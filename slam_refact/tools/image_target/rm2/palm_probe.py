#!/usr/bin/env python3
"""B6 / RM-2 (exploratory, NOT a gate, NOT parity evidence): does the palm detection model react to the
real hand photographs that arrived with the LFS fetch? Run with `python3 -I`.

Pipeline = what src/hand/hand_tracker.cc does before/after the model, re-done in numpy/PIL:
  FIT-letterbox the RGB image to 256x256 (black padding), x/127.5 - 1, palm_detection.tflite through the
  numpy reference interpreter (tools/ml_ref), decode + weighted NMS + letterbox removal with
  tools/hand/hand_ref.py (score >= 0.5).
Deviations from the C++ engine (so this is a probe, not a re-run of the engine): PIL bilinear instead of
OpenCV INTER_LINEAR for the FIT resize; float64 numpy interpreter instead of the C++ interpreter; the
same executor wrote hand_ref.py (T3 evidence at best).
Usage: palm_probe.py <out.json> <image> [<image> ...]   (hand photographs plus hand-free controls)
"""
import json
import os
import sys
import time

import numpy as np
from PIL import Image, ImageDraw, ImageOps

HERE = os.path.dirname(os.path.abspath(__file__))
TOOLS = os.path.abspath(os.path.join(HERE, '..', '..'))
REPO = os.path.abspath(os.path.join(TOOLS, '..', '..'))
sys.path.insert(0, os.path.join(TOOLS, 'ml_ref'))
sys.path.insert(0, os.path.join(TOOLS, 'hand'))
from ref_ops import RefInterpreter  # noqa: E402
from tflite_reader import read_model  # noqa: E402
import hand_ref as H  # noqa: E402

PALM = os.path.join(REPO, 'assets/references/xr_reference/official/mediapipe-fork/mediapipe/models/palm_detection.tflite')
SIZE = 256


def fit(img):
  w, h = img.size
  ia = w / h
  pad = [0.0, 0.0, 0.0, 0.0]
  if ia < 1:
    nw, nh = max(1, int(round(SIZE * ia))), SIZE
    pad[0] = pad[2] = (1 - ia) / 2
  elif ia > 1:
    nw, nh = SIZE, max(1, int(round(SIZE / ia)))
    pad[1] = pad[3] = (1 - 1 / ia) / 2
  else:
    nw, nh = SIZE, SIZE
  canvas = Image.new('RGB', (SIZE, SIZE), (0, 0, 0))
  canvas.paste(img.resize((nw, nh), Image.Resampling.BILINEAR), ((SIZE - nw) // 2, (SIZE - nh) // 2))
  return canvas, pad


def main():
  out_path, images = sys.argv[1], sys.argv[2:]
  m = read_model(PALM)
  interp = RefInterpreter(m)
  anchors = H.generate_anchors()
  res = []
  tiles = []
  for p in images:
    with Image.open(p) as im:
      im = ImageOps.exif_transpose(im).convert('RGB')
    w, h = im.size
    fitted, pad = fit(im)
    x = (np.asarray(fitted, dtype=np.float64) / 127.5 - 1.0)[None]
    t0 = time.time()
    vals = interp.run([x])
    dt = time.time() - t0
    boxes = scores = None
    for oi in m.outputs:
      v = vals[oi]
      if v.ndim == 3 and v.shape[1] == 2944 and v.shape[2] == 18:
        boxes = v
      if v.ndim == 3 and v.shape[1] == 2944 and v.shape[2] == 1:
        scores = v
    assert boxes is not None and scores is not None, [vals[o].shape for o in m.outputs]
    s = 1.0 / (1.0 + np.exp(-np.clip(scores.reshape(-1), -100, 100)))
    dets = H.decode_and_filter(boxes.reshape(2944, 18), scores.reshape(2944), anchors)
    final = H.remove_letterbox_det(H.weighted_nms(dets), pad)
    r = {'image': p.split('lfs-files/')[-1], 'size': [w, h], 'seconds': round(dt, 1),
         'max_sigmoid_score': round(float(s.max()), 4), 'n_ge_0.5': int((s >= 0.5).sum()),
         'n_ge_0.1': int((s >= 0.1).sum()), 'detections_after_nms': len(final),
         'dets': [{'score': round(d['score'], 4),
                   'box_xywh_px': [round(d['xmin'] * w), round(d['ymin'] * h), round(d['w'] * w), round(d['h'] * h)]}
                  for d in final]}
    res.append(r)
    print(json.dumps(r), flush=True)
    tile = im.copy()
    draw = ImageDraw.Draw(tile)
    for i, d in enumerate(r['dets']):
      x0, y0, bw, bh = d['box_xywh_px']
      draw.rectangle([x0, y0, x0 + bw, y0 + bh], outline=(255, 0, 0) if i == 0 else (0, 255, 0),
                     width=max(1, max(w, h) // 128))
    k = 200 / max(w, h)
    tiles.append(tile.resize((max(1, int(w * k)), max(1, int(h * k)))))
  canvas = Image.new('RGB', (sum(t.size[0] + 6 for t in tiles), max(t.size[1] for t in tiles)), (40, 40, 40))
  x0 = 0
  for t in tiles:
    canvas.paste(t, (x0, 0))
    x0 += t.size[0] + 6
  canvas.save(os.path.splitext(out_path)[0] + '_overlay.png')  # red = best detection, green = second
  json.dump({'model': 'palm_detection.tflite', 'tool': 'palm_probe.py (exploratory)', 'results': res},
            open(out_path, 'w'), indent=1)


if __name__ == '__main__':
  main()

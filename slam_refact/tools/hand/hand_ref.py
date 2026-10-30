#!/usr/bin/env python3
"""Independent numpy implementation of the calculator math of the MediaPipe hand graph (US-021;
docs/spec/10-hand.md §3, gate HAND-G2). Written separately from src/hand/hand_geometry.cc (vectorised
numpy, float32 scalars) from the same public sources:
  mediapipe/calculators/tflite/ssd_anchors_calculator.cc, tflite_tensors_to_detections_calculator.cc,
  util/non_max_suppression_calculator.cc, detection_letterbox_removal_calculator.cc,
  landmark_letterbox_removal_calculator.cc, detections_to_rects_calculator.cc,
  rect_transformation_calculator.cc, landmarks_to_detection_calculator.cc, landmark_projection_calculator.cc.
Same executor, different tool: this is a second implementation, not an independent author (T3).
numpy only; run with python3 -I.
"""
import math

import numpy as np

f32 = np.float32


def generate_anchors(num_layers=5, min_scale=0.1171875, max_scale=0.75, input_h=256, input_w=256,
                     strides=(8, 16, 32, 32, 32), aspect_ratios=(1.0,), interpolated=1.0, fixed=True):
  def scale_of(i, n):
    return f32(min_scale) + (f32(max_scale) - f32(min_scale)) * f32(1.0) * f32(i) / (f32(n) - f32(1.0))

  anchors = []
  n = len(strides)
  layer = 0
  while layer < n:
    ratios, scales = [], []
    last = layer
    while last < n and strides[last] == strides[layer]:
      sc = scale_of(last, n)
      for ar in aspect_ratios:
        ratios.append(f32(ar))
        scales.append(sc)
      if interpolated > 0.0:
        nxt = f32(1.0) if last == n - 1 else scale_of(last + 1, n)
        scales.append(np.sqrt(sc * nxt))
        ratios.append(f32(interpolated))
      last += 1
    stride = strides[layer]
    fh = int(math.ceil(f32(1.0) * f32(input_h) / f32(stride)))
    fw = int(math.ceil(f32(1.0) * f32(input_w) / f32(stride)))
    per_cell = len(ratios)
    ys, xs = np.meshgrid(np.arange(fh), np.arange(fw), indexing='ij')
    xc = (xs.astype(np.float32) + f32(0.5)) * f32(1.0) / f32(fw)
    yc = (ys.astype(np.float32) + f32(0.5)) * f32(1.0) / f32(fh)
    for y in range(fh):
      for x in range(fw):
        for a in range(per_cell):
          rs = np.sqrt(ratios[a])
          w = f32(1.0) if fixed else scales[a] * rs
          h = f32(1.0) if fixed else scales[a] / rs
          anchors.append((xc[y, x], yc[y, x], w, h))
    layer = last
  return np.array(anchors, dtype=np.float32)


def decode_and_filter(raw_boxes, raw_scores, anchors, min_score=0.5, clip=100.0):
  """TfLiteTensorsToDetections (palm graph): returns list of dict(score, xmin, ymin, w, h, kp[7][2])."""
  n = raw_boxes.shape[0]
  rb = raw_boxes.astype(np.float32).reshape(n, 18)
  xc = rb[:, 0] / f32(256) * anchors[:, 2] + anchors[:, 0]
  yc = rb[:, 1] / f32(256) * anchors[:, 3] + anchors[:, 1]
  w = rb[:, 2] / f32(256) * anchors[:, 2]
  h = rb[:, 3] / f32(256) * anchors[:, 3]
  ymin = yc - h / f32(2)
  xmin = xc - w / f32(2)
  ymax = yc + h / f32(2)
  xmax = xc + w / f32(2)
  kp = np.zeros((n, 7, 2), np.float32)
  for k in range(7):
    kp[:, k, 0] = rb[:, 4 + 2 * k] / f32(256) * anchors[:, 2] + anchors[:, 0]
    kp[:, k, 1] = rb[:, 5 + 2 * k] / f32(256) * anchors[:, 3] + anchors[:, 1]
  s = np.clip(raw_scores.astype(np.float32).reshape(n), f32(-clip), f32(clip))
  score = f32(1.0) / (f32(1.0) + np.exp(-s))
  dets = []
  for i in range(n):
    if score[i] < min_score:
      continue
    dets.append(dict(score=float(score[i]), xmin=float(xmin[i]), ymin=float(ymin[i]),
                     w=float(xmax[i] - xmin[i]), h=float(ymax[i] - ymin[i]), kp=[(float(a), float(b)) for a, b in kp[i]]))
  return dets


def _iou(a, b):
  ax2, ay2 = f32(a['xmin']) + f32(a['w']), f32(a['ymin']) + f32(a['h'])
  bx2, by2 = f32(b['xmin']) + f32(b['w']), f32(b['ymin']) + f32(b['h'])
  if not (f32(a['xmin']) < bx2 and f32(b['xmin']) < ax2 and f32(a['ymin']) < by2 and f32(b['ymin']) < ay2):
    return f32(0)
  ix = min(ax2, bx2) - max(f32(a['xmin']), f32(b['xmin']))
  iy = min(ay2, by2) - max(f32(a['ymin']), f32(b['ymin']))
  inter = ix * iy
  area_a = (ax2 - f32(a['xmin'])) * (ay2 - f32(a['ymin']))
  area_b = (bx2 - f32(b['xmin'])) * (by2 - f32(b['ymin']))
  norm = area_a + area_b - inter
  return inter / norm if norm > 0 else f32(0)


def weighted_nms(dets, min_suppression=0.3, min_score=0.5):
  order = sorted(range(len(dets)), key=lambda i: -dets[i]['score'])  # stable
  remained = order
  out = []
  while remained:
    first = dets[remained[0]]
    if min_score > 0 and first['score'] < min_score:
      break
    cand, rest = [], []
    for i in remained:
      (cand if _iou(dets[i], first) > min_suppression else rest).append(i)
    w = dict(first)
    if cand:
      tot = f32(0)
      wx0 = wy0 = wx1 = wy1 = f32(0)
      kps = [[f32(0), f32(0)] for _ in first['kp']]
      for i in cand:
        d = dets[i]
        sc = f32(d['score'])
        tot += sc
        wx0 += f32(d['xmin']) * sc
        wy0 += f32(d['ymin']) * sc
        wx1 += (f32(d['xmin']) + f32(d['w'])) * sc
        wy1 += (f32(d['ymin']) + f32(d['h'])) * sc
        for j, (x, y) in enumerate(d['kp']):
          kps[j][0] += f32(x) * sc
          kps[j][1] += f32(y) * sc
      w['xmin'] = float(wx0 / tot)
      w['ymin'] = float(wy0 / tot)
      w['w'] = float(wx1 / tot - f32(w['xmin']))
      w['h'] = float(wy1 / tot - f32(w['ymin']))
      w['kp'] = [(float(a / tot), float(b / tot)) for a, b in kps]
    remained = rest
    out.append(w)
  return out


def remove_letterbox_det(dets, pad):
  l, t = f32(pad[0]), f32(pad[1])
  lr, tb = f32(pad[0]) + f32(pad[2]), f32(pad[1]) + f32(pad[3])
  out = []
  for d in dets:
    nd = dict(d)
    nd['xmin'] = float((f32(d['xmin']) - l) / (f32(1) - lr))
    nd['ymin'] = float((f32(d['ymin']) - t) / (f32(1) - tb))
    nd['w'] = float(f32(d['w']) / (f32(1) - lr))
    nd['h'] = float(f32(d['h']) / (f32(1) - tb))
    nd['kp'] = [(float((f32(x) - l) / (f32(1) - lr)), float((f32(y) - t) / (f32(1) - tb))) for x, y in d['kp']]
    out.append(nd)
  return out


def normalize_radians(a):
  a = float(f32(a))
  return float(f32(a - 2 * math.pi * math.floor((a + math.pi) / (2 * math.pi))))


def detection_to_rect(d, iw, ih, start, end, target_deg):
  xc = f32(d['xmin']) + f32(d['w']) / f32(2)
  yc = f32(d['ymin']) + f32(d['h']) / f32(2)
  target = f32(math.pi * target_deg / f32(180.0))
  x0 = f32(d['kp'][start][0]) * f32(iw)
  y0 = f32(d['kp'][start][1]) * f32(ih)
  x1 = f32(d['kp'][end][0]) * f32(iw)
  y1 = f32(d['kp'][end][1]) * f32(ih)
  ang = np.arctan2(f32(-(y1 - y0)), f32(x1 - x0))
  rot = normalize_radians(f32(target - ang))
  return dict(x=float(xc), y=float(yc), w=float(d['w']), h=float(d['h']), rot=rot)


def transform_rect(r, iw, ih, scale_x=1.0, scale_y=1.0, shift_x=0.0, shift_y=0.0, square_long=False):
  w, h, rot = f32(r['w']), f32(r['h']), f32(r['rot'])
  x, y = f32(r['x']), f32(r['y'])
  if rot == 0:
    x = x + w * f32(shift_x)
    y = y + h * f32(shift_y)
  else:
    c, s = np.cos(rot), np.sin(rot)
    x = x + (f32(iw) * w * f32(shift_x) * c - f32(ih) * h * f32(shift_y) * s) / f32(iw)
    y = y + (f32(iw) * w * f32(shift_x) * s + f32(ih) * h * f32(shift_y) * c) / f32(ih)
  if square_long:
    long_side = max(w * f32(iw), h * f32(ih))
    w = long_side / f32(iw)
    h = long_side / f32(ih)
  return dict(x=float(x), y=float(y), w=float(w * f32(scale_x)), h=float(h * f32(scale_y)), rot=float(r['rot']))


def landmarks_to_detection(lms):
  xs = [f32(l[0]) for l in lms]
  ys = [f32(l[1]) for l in lms]
  xmin, xmax = min(xs), max(max(xs), np.finfo(np.float32).tiny)
  ymin, ymax = min(ys), max(max(ys), np.finfo(np.float32).tiny)
  return dict(score=0.0, xmin=float(xmin), ymin=float(ymin), w=float(xmax - xmin), h=float(ymax - ymin),
              kp=[(float(l[0]), float(l[1])) for l in lms])


def project_landmarks(lms, r):
  out = []
  c, s = np.cos(f32(r['rot'])), np.sin(f32(r['rot']))
  for l in lms:
    x, y = f32(l[0]) - f32(0.5), f32(l[1]) - f32(0.5)
    nx = c * x - s * y
    ny = s * x + c * y
    out.append((float(nx * f32(r['w']) + f32(r['x'])), float(ny * f32(r['h']) + f32(r['y'])), float(l[2])))
  return out


def remove_letterbox_landmarks(lms, pad):
  l, t = f32(pad[0]), f32(pad[1])
  lr, tb = f32(pad[0]) + f32(pad[2]), f32(pad[1]) + f32(pad[3])
  return [(float((f32(x) - l) / (f32(1) - lr)), float((f32(y) - t) / (f32(1) - tb)), float(z)) for x, y, z in lms]

#!/usr/bin/env python3
"""Golden data for the image operations of the hand module (US-021; docs/spec/10-hand.md §2, gate HAND-G1).

  gen_image_golden.py --out <new dir>

The expected values come from OpenCV (cv2), the library the original MediaPipe calculators call, executed
here by a different tool than the C++ under test (the C++ never links OpenCV):
  resize_cases.tsv    sw sh dw dh seed sha256(cv2.resize INTER_LINEAR output)
  fit_cases.tsv       sw sh ow oh seed pad_l pad_t pad_r pad_b sha256(FIT output)  (cv2.resize + copyMakeBorder)
  crop_cases.tsv      sw sh cx cy w h rot seed sha256(crop)  (cv2.boxPoints + getPerspectiveTransform + warpPerspective
                      flags=0 BORDER_REPLICATE exactly as image_cropping_calculator.cc RenderCpu)
  boxpoints_cases.tsv, perspective_cases.tsv   numeric cases (float64 columns)
Inputs are 32-bit LCG bytes (s = s*1664525 + 1013904223, byte = s >> 24), RGB, row 0 = top.
cv2 / numpy versions and the input rule are written to PROVENANCE.txt. Run with the miniconda python
(has cv2); NOT part of the product, never imported by tests.
"""
import argparse
import hashlib
import math
import os

import cv2
import numpy as np


def lcg(n, seed):
  out = np.empty(n, np.uint8)
  s = seed & 0xFFFFFFFF
  for i in range(n):
    s = (s * 1664525 + 1013904223) & 0xFFFFFFFF
    out[i] = s >> 24
  return out


def image(w, h, seed):
  # Smooth-ish content so interpolation matters: LCG noise blended with a gradient.
  noise = lcg(w * h * 3, seed).reshape(h, w, 3).astype(np.float64)
  yy, xx = np.mgrid[0:h, 0:w]
  grad = (xx * 255.0 / max(1, w - 1) + yy * 255.0 / max(1, h - 1)) / 2.0
  img = 0.5 * noise + 0.5 * grad[..., None]
  return np.clip(img, 0, 255).astype(np.uint8)


def sha(a):
  return hashlib.sha256(np.ascontiguousarray(a).tobytes()).hexdigest()


def fit(img, ow, oh):
  ih, iw = img.shape[:2]
  scale = min(np.float32(ow) / np.float32(iw), np.float32(oh) / np.float32(ih))
  tw = int(np.round(np.float32(iw) * np.float32(scale)))
  th = int(np.round(np.float32(ih) * np.float32(scale)))
  inter = cv2.resize(img, (tw, th))
  top = (oh - th) // 2
  bottom = oh - th - top
  left = (ow - tw) // 2
  right = ow - tw - left
  out = cv2.copyMakeBorder(inter, top, bottom, left, right, cv2.BORDER_CONSTANT)
  pad = [0.0, 0.0, 0.0, 0.0]
  ia = np.float32(iw) / np.float32(ih)
  oa = np.float32(ow) / np.float32(oh)
  if ia < oa:
    pad[0] = pad[2] = float((np.float32(1) - ia / oa) / np.float32(2))
  elif oa < ia:
    pad[1] = pad[3] = float((np.float32(1) - oa / ia) / np.float32(2))
  return out, pad


def crop(img, cx, cy, w, h, rot):
  # image_cropping_calculator.cc:215-262
  min_rect = ((float(cx), float(cy)), (float(w), float(h)), float(np.float32(rot) * np.float32(180.0) / np.float32(math.pi)))
  src = cv2.boxPoints(min_rect)
  dst = np.array([[0, h - 1], [0, 0], [w - 1, 0], [w - 1, h - 1]], np.float32)
  m = cv2.getPerspectiveTransform(src.astype(np.float32), dst)
  return cv2.warpPerspective(img, m, (w, h), flags=0, borderMode=cv2.BORDER_REPLICATE), src, m


def main():
  ap = argparse.ArgumentParser()
  ap.add_argument('--out', required=True)
  ap.add_argument('--mode', choices=['golden', 'gate'], default='golden',
                  help='golden: the fixed small cases checked in under tests/hand/golden; gate: fresh random cases '
                       'drawn from numpy default_rng(--seed) (preregistered gate HAND-G1)')
  ap.add_argument('--seed', type=int, default=0)
  ap.add_argument('--resize-count', type=int, default=30)
  ap.add_argument('--fit-count', type=int, default=20)
  ap.add_argument('--crop-count', type=int, default=30)
  a = ap.parse_args()
  os.makedirs(a.out, exist_ok=False)

  def save(name, i, arr):
    open(os.path.join(a.out, '%s-%d.rgb' % (name, i)), 'wb').write(np.ascontiguousarray(arr).tobytes())

  rs, fs, cs, bp, pt = [], [], [], [], []
  seed = 1
  if a.mode == 'gate':
    rng = np.random.default_rng(a.seed)
    resize_cases = [tuple(int(v) for v in (rng.integers(5, 401), rng.integers(5, 401), rng.integers(2, 301), rng.integers(2, 301)))
                    for _ in range(a.resize_count)]
    fit_cases = [tuple(int(v) for v in (rng.integers(20, 401), rng.integers(20, 401), 256, 256)) for _ in range(a.fit_count)]
    crop_cases = []
    for _ in range(a.crop_count):
      sw, sh = int(rng.integers(60, 401)), int(rng.integers(60, 401))
      crop_cases.append((sw, sh, int(rng.integers(0, sw)), int(rng.integers(0, sh)), int(rng.integers(20, 301)),
                         int(rng.integers(20, 301)), float(rng.uniform(-math.pi, math.pi))))
    seed = int(a.seed) * 1000 + 1
  else:
    resize_cases = [(75, 141, 34, 64), (60, 80, 48, 64), (80, 60, 64, 48), (25, 25, 64, 64), (64, 64, 64, 64),
                    (31, 17, 60, 41), (17, 31, 5, 9), (100, 70, 64, 45), (7, 7, 64, 64), (75, 141, 35, 67),
                    (33, 77, 12, 27), (3, 3, 2, 2)]
    fit_cases = [(75, 141, 64, 64), (60, 80, 64, 64), (80, 60, 64, 64), (64, 64, 64, 64), (50, 50, 64, 64),
                 (54, 54, 64, 64), (55, 54, 64, 64), (20, 32, 64, 64)]
    crop_cases = [(75, 141, 38, 70, 54, 54, 0.0), (75, 141, 55, 39, 54, 54, 0.199),
                  (75, 141, 25, 100, 40, 40, -1.15), (60, 80, 30, 40, 50, 50, 0.7853982),
                  (60, 80, 5, 8, 40, 40, 2.5), (60, 80, 57, 78, 48, 48, -0.3),
                  (75, 141, 38, 70, 20, 30, 1.0), (75, 141, 38, 70, 120, 120, 0.4),
                  (80, 60, 40, 30, 50, 50, 3.0), (75, 141, 38, 70, 54, 54, math.pi / 2)]
  for (sw, sh, dw, dh) in resize_cases:
    out = cv2.resize(image(sw, sh, seed), (dw, dh))
    save('resize', len(rs), out)
    rs.append((sw, sh, dw, dh, seed))
    seed += 1
  for (sw, sh, ow, oh) in fit_cases:
    out, pad = fit(image(sw, sh, seed), ow, oh)
    save('fit', len(fs), out)
    fs.append((sw, sh, ow, oh, seed, *[repr(p) for p in pad]))
    seed += 1
  for (sw, sh, cx, cy, w, h, rot) in crop_cases:
    out, src, m = crop(image(sw, sh, seed), cx, cy, w, h, rot)
    save('crop', len(cs), out)
    cs.append((sw, sh, cx, cy, w, h, repr(rot), seed))
    ang = float(np.float32(rot) * np.float32(180.0) / np.float32(math.pi))
    bp.append((cx, cy, w, h, repr(ang), *[repr(float(v)) for v in src.reshape(-1)]))
    pt.append((*[repr(float(v)) for v in src.reshape(-1)], w, h, *[repr(float(v)) for v in m.reshape(-1)]))
    seed += 1
  open(os.path.join(a.out, 'resize_cases.tsv'), 'w').write(
      '# sw\tsh\tdw\tdh\tseed   (expected: resize-<row>.rgb = cv2.resize INTER_LINEAR, RGB8)\n' + '\n'.join('\t'.join(map(str, r)) for r in rs) + '\n')
  open(os.path.join(a.out, 'fit_cases.tsv'), 'w').write(
      '# sw\tsh\tow\toh\tseed\tpad_l\tpad_t\tpad_r\tpad_b   (expected: fit-<row>.rgb)\n' + '\n'.join('\t'.join(map(str, r)) for r in fs) + '\n')
  open(os.path.join(a.out, 'crop_cases.tsv'), 'w').write(
      '# sw\tsh\tcx\tcy\tw\th\trot\tseed   (expected: crop-<row>.rgb)\n' + '\n'.join('\t'.join(map(str, r)) for r in cs) + '\n')
  open(os.path.join(a.out, 'boxpoints_cases.tsv'), 'w').write(
      '# cx\tcy\tw\th\tangle_deg(float32 of rot*180/pi)\tp0x p0y p1x p1y p2x p2y p3x p3y (cv2.boxPoints, float32 as double)\n'
      + '\n'.join('\t'.join(map(str, r)) for r in bp) + '\n')
  open(os.path.join(a.out, 'perspective_cases.tsv'), 'w').write(
      '# src points (8 floats: p0x p0y .. p3y), w, h, then the 9 entries of cv2.getPerspectiveTransform (float64)\n'
      + '\n'.join('\t'.join(map(str, r)) for r in pt) + '\n')
  open(os.path.join(a.out, 'PROVENANCE.txt'), 'w').write(
      'generated by slam_refact/tools/hand/gen_image_golden.py\ncv2 %s numpy %s\n'
      'inputs: image(w, h, seed) = clip(0.5 * LCG noise + 0.5 * diagonal gradient), RGB8, row 0 = top\n'
      'LCG: s = s * 1664525 + 1013904223 (mod 2^32), byte = s >> 24, seed per case in the tsv\n'
      'expected outputs: the raw cv2 results <kind>-<row>.rgb\n' % (cv2.__version__, np.__version__))
  print('resize', len(rs), 'fit', len(fs), 'crop', len(cs))


if __name__ == '__main__':
  main()

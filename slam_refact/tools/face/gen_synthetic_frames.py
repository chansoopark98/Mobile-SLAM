#!/usr/bin/env python3
"""Synthetic RGBA frames for the face pipeline (US-018/US-019, docs/spec/08-face-ear.md §8).

Writes <out>/frame-%04d.rgba (RGBA8, row 0 = top), <out>/frame-%04d.png-free raw only, and
<out>/meta.json {width, height, count, timestamps_ms, generator}. The pattern is the one of
tests/face/face_models_evidence_test.cc SyntheticFrame (sin/cos texture + 32-bit LCG noise, gray):
  v = 128 + 70 sin(0.05 c + 0.01 seed) cos(0.04 r) + 20 (noise/255 - 0.5), seed = frame index.
No real face is involved. numpy only. Run with python3 -I.
"""
import argparse
import json
import os

import numpy as np


def lcg_noise(n, seed):
  """noise[i] = (s_{i+1} >> 24), s_{k+1} = a s_k + c mod 2^32, s_0 = seed (row-major order)."""
  a, c, mask = 1664525, 1013904223, 0xFFFFFFFF
  lanes = 1024
  first = np.empty(lanes, dtype=np.uint64)
  s = seed & mask
  for i in range(lanes):
    s = (s * a + c) & mask
    first[i] = s
  # Jump of `lanes` steps: s_{k+lanes} = A s_k + C.
  big_a, big_c = 1, 0
  for _ in range(lanes):
    big_a, big_c = (big_a * a) & mask, (big_c * a + c) & mask
  out = np.empty(((n + lanes - 1) // lanes) * lanes, dtype=np.uint64)
  cur = first
  for b in range(len(out) // lanes):
    out[b * lanes:(b + 1) * lanes] = cur
    cur = (cur * np.uint64(big_a) + np.uint64(big_c)) & np.uint64(mask)
  return (out[:n] >> np.uint64(24)).astype(np.uint32)


def frame(w, h, seed):
  noise = lcg_noise(w * h, seed)
  rr, cc = np.mgrid[0:h, 0:w]
  v = 128.0 + 70.0 * np.sin(0.05 * cc + 0.01 * seed) * np.cos(0.04 * rr) + \
      20.0 * (noise.reshape(h, w).astype(np.float64) / 255.0 - 0.5)
  g = np.clip(v, 0.0, 255.0).astype(np.uint8)
  rgba = np.empty((h, w, 4), dtype=np.uint8)
  rgba[..., 0] = g
  rgba[..., 1] = g
  rgba[..., 2] = g
  rgba[..., 3] = 255
  return rgba


def main():
  ap = argparse.ArgumentParser(description=__doc__)
  ap.add_argument('--out', required=True)
  ap.add_argument('--count', type=int, default=60)
  ap.add_argument('--width', type=int, default=480)
  ap.add_argument('--height', type=int, default=640)
  ap.add_argument('--fps', type=float, default=20.0)
  a = ap.parse_args()
  os.makedirs(a.out, exist_ok=False)
  for i in range(a.count):
    frame(a.width, a.height, i).tofile(os.path.join(a.out, 'frame-%04d.rgba' % i))
  meta = {'width': a.width, 'height': a.height, 'count': a.count,
          'timestamps_ms': [1000.0 * i / a.fps for i in range(a.count)],
          'generator': 'slam_refact/tools/face/gen_synthetic_frames.py (sin/cos + LCG gray)'}
  with open(os.path.join(a.out, 'meta.json'), 'w') as f:
    json.dump(meta, f)


if __name__ == '__main__':
  main()

#!/usr/bin/env python3
"""Synthetic and real-photo frame sets for the sky pipeline (US-020, docs/spec/09-sky.md).

Writes <out>/frame-%04d.rgba (RGBA8, row 0 = top), <out>/meta.json {width, height, count, kind,
orientations [{alpha,beta,gamma} per processed frame], generator, seed}. numpy (+PIL for pgm) only.
Run with python3 -I.

kinds
  scene   : deterministic synthetic RGB "outdoor-like" scene (sky gradient + value-noise clouds above
            a horizon, green/grey ground with box and blob "objects"), camera pans with the frame
            index. NOT a real sky: the model's response to it is an input property, not accuracy.
  flip6   : real Galaxy Z Flip6 camera frames (portrait 480x640 gray PGM from the ARCore fixtures
            build/slam-refact-20261015/reference/fixtures/flip6-arcore-*), gray expanded to RGB.
            Indoor office, no sky: a REAL-INPUT sample, not a sky accuracy dataset.
  scene-land : like scene but landscape (640x480), for the orientation 90 / -90 preprocessing paths.
orientation sequence (device orientation alpha/beta/gamma in degrees, one per processed frame k):
  sweep   : portrait held upright, beta 70 -> 110 (tilt up through the horizon) while alpha sweeps
            0 -> 120, gamma 0 -> 10: exercises cubemap faces +/-Z, +/-X and the horizon rows.
  still   : constant (alpha 0, beta 90, gamma 0).
  random  : seeded random walk within beta [50, 130], gamma [-30, 30], alpha [0, 360).
  poles   : one full oscillation of beta 90 +- 75 (camera pointing 75 degrees above, then below the horizon: the cube
            map's top and bottom faces), alpha 0 -> 360, gamma 8 sin(4 pi u).
"""
import argparse
import json
import math
import os

import numpy as np


def value_noise(h, w, rng, cells):
  """Smooth value noise in [0,1]: bilinear upsample of a random grid (cells x cells*aspect)."""
  gh = cells + 2
  gw = int(cells * w / h) + 3
  grid = rng.random((gh, gw))
  ys = np.linspace(0, gh - 3, h)
  xs = np.linspace(0, gw - 3, w)
  y0 = np.floor(ys).astype(int)
  x0 = np.floor(xs).astype(int)
  fy = (ys - y0)[:, None]
  fx = (xs - x0)[None, :]
  fy = fy * fy * (3 - 2 * fy)
  fx = fx * fx * (3 - 2 * fx)
  a = grid[y0][:, x0]
  b = grid[y0][:, x0 + 1]
  c = grid[y0 + 1][:, x0]
  d = grid[y0 + 1][:, x0 + 1]
  return (a * (1 - fx) + b * fx) * (1 - fy) + (c * (1 - fx) + d * fx) * fy


def scene(w, h, k, seed):
  rng = np.random.default_rng(seed)
  # Bigger virtual canvas so the camera can pan by 6 px/frame.
  pad = 6 * 64
  W = w + pad
  horizon = int(0.52 * h)
  yy = np.linspace(0, 1, h)[:, None]
  sky = np.empty((h, W, 3))
  top = np.array([70, 120, 215.0])
  bot = np.array([190, 215, 245.0])
  t = np.clip(yy / (horizon / h), 0, 1)
  base = top[None, None, :] * (1 - t[..., None]) + bot[None, None, :] * t[..., None]
  sky[:] = base
  cloud = 0.55 * value_noise(h, W, rng, 5) + 0.30 * value_noise(h, W, rng, 11) + 0.15 * value_noise(h, W, rng, 23)
  cl = np.clip((cloud - 0.52) * 4.0, 0, 1)[..., None]
  sky = sky * (1 - cl) + np.array([248, 248, 250.0]) * cl
  ground = np.empty((h, W, 3))
  gn = 0.6 * value_noise(h, W, rng, 17) + 0.4 * value_noise(h, W, rng, 41)
  grass = np.array([60, 110, 45.0]) * (0.7 + 0.6 * gn[..., None])
  rows = np.arange(h)[:, None]
  ground[:] = grass
  # A road, a building and tree blobs on the ground side.
  road = (rows > horizon + 0.35 * (h - horizon)) & (rows < horizon + 0.55 * (h - horizon))
  ground[np.broadcast_to(road, (h, W))] = np.array([95, 95, 100.0]) * (0.8 + 0.4 * gn[np.broadcast_to(road, (h, W))][:, None])
  xs = np.arange(W)[None, :]
  for (cx, wd, ht, col) in [(250, 120, 0.17, (150, 120, 100)), (700, 90, 0.12, (120, 110, 130))]:
    b = (xs > cx - wd // 2) & (xs < cx + wd // 2) & (rows > horizon - int(ht * h)) & (rows < horizon + 20)
    ground[np.broadcast_to(b, (h, W))] = np.array(col, dtype=float)
  for (cx, cy, r) in [(120, horizon - 10, 40), (520, horizon - 25, 55), (900, horizon - 5, 35)]:
    m = (xs - cx) ** 2 + (rows - cy) ** 2 < r * r
    ground[np.broadcast_to(m, (h, W))] = np.array([30, 80, 30.0])
  img = np.where((rows < horizon)[..., None], sky, ground)
  # Slight vignette-free, lightly noisy camera.
  img = img + rng.normal(0, 1.5, img.shape)
  off = (6 * k) % pad
  return np.clip(img[:, off:off + w], 0, 255).astype(np.uint8)


def orientations(kind, n, seed):
  rng = np.random.default_rng(seed + 1000)
  out = []
  a, b, g = 0.0, 90.0, 0.0
  for k in range(n):
    if kind == 'sweep':
      u = k / max(1, n - 1)
      out.append({'alpha': 120.0 * u, 'beta': 70.0 + 40.0 * u, 'gamma': 10.0 * u})
    elif kind == 'still':
      out.append({'alpha': 0.0, 'beta': 90.0, 'gamma': 0.0})
    elif kind == 'poles':
      u = k / max(1, n - 1)
      out.append({'alpha': 360.0 * u, 'beta': 90.0 + 75.0 * math.sin(2 * math.pi * u), 'gamma': 8.0 * math.sin(4 * math.pi * u)})
    elif kind == 'random':
      a = (a + rng.normal(0, 12)) % 360.0
      b = float(np.clip(b + rng.normal(0, 8), 50, 130))
      g = float(np.clip(g + rng.normal(0, 5), -30, 30))
      out.append({'alpha': a, 'beta': b, 'gamma': g})
    else:
      raise SystemExit('unknown orientation kind ' + kind)
  return out


def read_pgm(path):
  from PIL import Image
  return np.asarray(Image.open(path).convert('L'))


def main():
  ap = argparse.ArgumentParser(description=__doc__)
  ap.add_argument('--out', required=True)
  ap.add_argument('--kind', choices=['scene', 'flip6', 'scene-land'], default='scene')
  ap.add_argument('--count', type=int, default=48)
  ap.add_argument('--seed', type=int, default=1)
  ap.add_argument('--orientation', choices=['sweep', 'still', 'random', 'poles'], default='sweep')
  ap.add_argument('--fixture', default='flip6-arcore-s2')
  ap.add_argument('--fixture-root', default='build/slam-refact-20261015/reference/fixtures')
  ap.add_argument('--stride', type=int, default=17)
  ap.add_argument('--oframes', type=int, default=0, help='length of the orientation sequence (default count + 4)')
  a = ap.parse_args()
  os.makedirs(a.out, exist_ok=False)
  frames = []
  if a.kind == 'flip6':
    d = os.path.join(a.fixture_root, a.fixture, 'mav0/cam0/data')
    files = sorted(os.listdir(d))
    sel = [files[(i * a.stride) % len(files)] for i in range(a.count)]
    for fn in sel:
      g = read_pgm(os.path.join(d, fn))
      frames.append(np.stack([g, g, g], axis=-1))
    h, w = frames[0].shape[:2]
    src = {'fixture': a.fixture, 'files': sel}
  else:
    w, h = (480, 640) if a.kind == 'scene' else (640, 480)
    for i in range(a.count):
      frames.append(scene(w, h, i, a.seed))
    src = {}
  for i, f in enumerate(frames):
    rgba = np.empty(f.shape[:2] + (4,), np.uint8)
    rgba[..., :3] = f
    rgba[..., 3] = 255
    rgba.tofile(os.path.join(a.out, 'frame-%04d.rgba' % i))
  meta = {'width': w, 'height': h, 'count': a.count, 'kind': a.kind, 'seed': a.seed,
          'orientation_kind': a.orientation,
          'orientations': orientations(a.orientation, a.oframes or a.count + 4, a.seed),
          'generator': 'slam_refact/tools/sky/gen_sky_frames.py', 'source': src}
  with open(os.path.join(a.out, 'meta.json'), 'w') as fh:
    json.dump(meta, fh)


if __name__ == '__main__':
  main()

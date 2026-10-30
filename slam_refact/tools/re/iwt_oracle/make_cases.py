#!/usr/bin/env python3
"""Synthetic inputs for the func994 (image-world-tracker) oracle and the engine replay (US-015).

Writes one JSON file read by both sides:
  * run_f994.mjs (original xr-slam.wasm func994 in Node), and
  * sr_iwt_replay (engine ImageWorldTracker), through the flat text written by --flat.
The cases are designed to reach every branch read in the 994 decompile (spec/11 section 9):
planar and curvy lifting, visibility, the tracked-map lookup (missing key, id 0, id mismatch,
non-TRIANGULATED source), the < 6 correspondence fallback, the Ceres scale solve (inliers,
outliers, a non-finite residual that makes the solve unusable) and a non-identity map transform.
The geometry is constructed (camera at a known pose in front of the target); it says nothing about
real camera data.

  python3 -I make_cases.py OUT.json [--flat OUT.txt]                       # dev: 9 designed cases
  python3 -I make_cases.py OUT.json --variant random --seed S --n N [--flat OUT.txt]
The random variant draws N cases (planar / cylinder / cone, k, poses, pair counts, background
features, outliers, lookup filters, map transform) from random.Random(S). Gate it-world-gate.v1 uses
it with the executor's dev seed before the review and with the coordinator's seed for the one real
run (preregistration/it-world-gate.v1.md).
Stdlib only. Deterministic (random.Random(seed)).
"""
import json
import math
import random
import struct
import sys

F32 = struct.Struct('<f')


def f32(x):
  return F32.unpack(F32.pack(x))[0]


def matmul(a, b):
  return [[sum(a[r][k] * b[k][c] for k in range(4)) for c in range(4)] for r in range(4)]


def inverse(a):
  n = 4
  m = [list(map(float, row)) + [1.0 if r == c else 0.0 for c in range(n)] for r, row in enumerate(a)]
  for c in range(n):
    p = max(range(c, n), key=lambda r: abs(m[r][c]))
    m[c], m[p] = m[p], m[c]
    d = m[c][c]
    m[c] = [v / d for v in m[c]]
    for r in range(n):
      if r != c:
        f = m[r][c]
        m[r] = [vr - f * vc for vr, vc in zip(m[r], m[c])]
  return [row[n:] for row in m]


def rot(axis, ang):
  c, s = math.cos(ang), math.sin(ang)
  if axis == 'x':
    r = [[1, 0, 0], [0, c, -s], [0, s, c]]
  elif axis == 'y':
    r = [[c, 0, s], [0, 1, 0], [-s, 0, c]]
  else:
    r = [[c, -s, 0], [s, c, 0], [0, 0, 1]]
  return [r[0] + [0], r[1] + [0], r[2] + [0], [0, 0, 0, 1]]


def trans(t):
  return [[1, 0, 0, t[0]], [0, 1, 0, t[1]], [0, 0, 1, t[2]], [0, 0, 0, 1]]


def hm(a):
  """Row-major 4x4 -> HMatrix JSON (column-major m and inv, float32)."""
  inv = inverse(a)
  return {'m': [f32(a[r][c]) for c in range(4) for r in range(4)],
          'inv': [f32(inv[r][c]) for c in range(4) for r in range(4)],
          'noInvert': False, 'invertFailed': False}


def apply(a, p):
  v = [sum(a[r][k] * (list(p) + [1.0])[k] for k in range(4)) for r in range(4)]
  return [v[0] / v[3], v[1] / v[3], v[2] / v[3]]


def identity():
  return [[1.0 if r == c else 0.0 for c in range(4)] for r in range(4)]


def scale_translation(s, a):
  b = [row[:] for row in a]
  for r in range(3):
    b[r][3] *= s
  return b


def base_case(name, rng, k, n_target, n_off, curvy=False, scale_state=0.5, scale_guess=0.5,
              p=None, tam=None, cone_bottom=None):
  cam_k = [480, 640, 239.5, 319.5, 500.0, 500.0]
  target_k = [480, 640, 239.5, 319.5, 480.0, 480.0]
  # Target view pose P: camera pose in the target model frame (plane z = 1 for planar; curvy model
  # axis along y through the origin).
  if p is None and curvy:
    p = matmul(trans([0.05, 0.02, -1.6]), matmul(rot('y', 0.15), rot('x', -0.1)))
  elif p is None:
    p = matmul(trans([0.08, -0.05, -0.35]), matmul(rot('y', 0.2), rot('x', 0.12)))
  if tam is None:
    tam = matmul(trans([0.4, 1.3, -0.7]), matmul(rot('y', 0.9), rot('z', 0.05)))
  pinv = inverse(p)
  rays, pixels, model_pts = [], [], []
  geom = None
  if curvy:
    rb = 0.4 if cone_bottom is None else cone_bottom
    geom = {'radius': 0.4, 'height': 1.0, 'left': 0.1, 'right': 0.9, 'top': 0.0, 'bottom': 1.0,
            'src_rows': 640, 'src_cols': 480, 'is_cone': cone_bottom is not None, 'radius_bottom': rb}
  for i in range(n_target):
    if curvy:
      th = rng.uniform(-0.9, 0.9)  # front of the surface, facing the camera at z < 0
      y = rng.uniform(-0.45, 0.45)
      # cone radius at height y (parameterized-geometry.cc cameraRaysToTargetWorld): (r + rb) / 2 + y (r - rb)
      r_y = 0.5 * (geom['radius'] + geom['radius_bottom']) + y * (geom['radius'] - geom['radius_bottom'])
      xm = [r_y * math.sin(th), y, -r_y * math.cos(th)]
    else:
      xm = [rng.uniform(-0.3, 0.3), rng.uniform(-0.4, 0.4), 1.0]
    model_pts.append(xm)
  for i in range(n_off):  # features off the target (background)
    model_pts.append(None)
  for xm in model_pts:
    if xm is None:
      x, y = rng.uniform(-0.6, 0.6), rng.uniform(-0.7, 0.7)
    else:
      xc = apply(pinv, xm)
      x, y = xc[0] / xc[2], xc[1] / xc[2]
    rays.append([f32(x), f32(y)])
    pixels.append([f32(cam_k[4] * x + cam_k[2]), f32(cam_k[3] - cam_k[5] * y), 1.0])
  # World map: the true world point of every on-target feature is tam * (k * Xc).
  pool0, tracked = [], []
  slot = 0
  for i, xm in enumerate(model_pts):
    if xm is None:
      continue
    xc = apply(pinv, xm)
    xw = apply(tam, [k * v for v in xc])
    pool0.append({'id': 1000 + slot, 'pos': [f32(v) for v in xw] + [1.0], 'source': 1})
    tracked.append([i, slot, 1000 + slot])
    slot += 1
  st_pose = hm(matmul(trans([9.0, 9.0, 9.0]), identity()))  # previous worldPose (marker values)
  return {
      'name': name,
      'state': {'scale': scale_state, 'world_pose': st_pose},
      'map_pools': [pool0, [], [], [], []],
      'norm': {'scale': 1.0, 'offset': [0.0, 0.0, 0.0]},
      'tracked': tracked,
      'image': {'type': 2 if curvy else 0, 'target_k': target_k, 'geom': geom},
      'target_pose': hm(p),
      'tam': hm(tam),
      'frame': {'rays': rays, 'pixels': pixels, 'cam_k': cam_k},
      'scale_guess': scale_guess,
      'truth': {'k': k},
  }


def cases():
  rng = random.Random(15015)
  out = []
  out.append(base_case('planar-k072', rng, 0.72, 40, 20))
  out.append(base_case('planar-k19-guess08', rng, 1.9, 30, 10, scale_state=1.2, scale_guess=0.8))
  c = base_case('planar-fallback-5', rng, 0.72, 5, 10, scale_guess=0.61)
  out.append(c)
  c = base_case('planar-outliers', rng, 0.38, 40, 5)
  for j in range(0, 40, 5):  # 8 of 40 world points moved by 0.5 m
    c['map_pools'][0][j]['pos'][1] = f32(c['map_pools'][0][j]['pos'][1] + 0.5)
  out.append(c)
  c = base_case('planar-lookup-filters', rng, 1.15, 12, 6)
  pool = c['map_pools'][0]
  c['tracked'][0][2] = 0                       # id 0 -> not a point
  c['tracked'][1][2] = 77777                   # id mismatch with the slot
  pool[2]['source'] = 2                        # GROUND point: excluded
  c['map_pools'][1] = [{'id': 1003, 'pos': [0.0, 0.0, 0.0, 1.0], 'source': 2}] * 4  # pool 1 slot 3 id 1003
  pool[3]['id'] = 5555                         # slot 3 of pool 0 no longer has id 1003 -> found in pool 1 (GROUND)
  c['tracked'].append([200, 4, 1004])          # frame index without a feature: never visible
  c['tracked'].append([13, 5, 1005])           # off-target feature tied to an on-target map point
  out.append(c)
  c = base_case('planar-nonfinite', rng, 0.72, 20, 0)
  c['map_pools'][0][4]['pos'][3] = 0.0         # w = 0 -> non-finite model point
  out.append(c)
  c = base_case('planar-map-transform', rng, 0.9, 25, 5)
  c['norm'] = {'scale': 2.0, 'offset': [0.1, -0.2, 0.3]}
  out.append(c)
  out.append(base_case('curvy-k05', rng, 0.5, 40, 15, curvy=True))
  out.append(base_case('curvy-fallback', rng, 0.5, 4, 15, curvy=True, scale_guess=0.33))
  return out


def random_cases(seed, n):
  """N cases drawn from random.Random(seed): every branch of 994 with random geometry."""
  rng = random.Random(seed)
  out = []
  for i in range(n):
    shape = rng.choice(['planar', 'planar', 'cylinder', 'cone'])
    k = math.exp(rng.uniform(math.log(0.2), math.log(3.0)))
    n_on = rng.choice([rng.randint(0, 5), rng.randint(6, 12), rng.randint(13, 60)])
    n_off = rng.randint(0, 25)
    if shape == 'planar':
      p = matmul(trans([rng.uniform(-0.15, 0.15), rng.uniform(-0.15, 0.15), rng.uniform(-0.6, -0.15)]),
                 matmul(rot('y', rng.uniform(-0.4, 0.4)), rot('x', rng.uniform(-0.4, 0.4))))
    else:
      p = matmul(trans([rng.uniform(-0.1, 0.1), rng.uniform(-0.1, 0.1), rng.uniform(-2.2, -1.3)]),
                 matmul(rot('y', rng.uniform(-0.3, 0.3)), rot('x', rng.uniform(-0.2, 0.2))))
    tam = matmul(trans([rng.uniform(-2, 2), rng.uniform(-1, 2), rng.uniform(-2, 2)]),
                 matmul(rot('y', rng.uniform(-math.pi, math.pi)),
                        matmul(rot('x', rng.uniform(-0.3, 0.3)), rot('z', rng.uniform(-0.2, 0.2)))))
    c = base_case('r%03d-%s' % (i, shape), rng, k, n_on, n_off, curvy=shape != 'planar',
                  scale_state=rng.uniform(0.3, 2.0), scale_guess=rng.uniform(0.3, 2.0), p=p, tam=tam,
                  cone_bottom=rng.uniform(0.25, 0.38) if shape == 'cone' else None)
    pool = c['map_pools'][0]
    if pool and rng.random() < 0.4:  # outliers: up to 30 % of the world points moved
      for j in rng.sample(range(len(pool)), max(1, int(len(pool) * rng.uniform(0.0, 0.3)))):
        pool[j]['pos'][rng.randrange(3)] = f32(pool[j]['pos'][rng.randrange(3)] + rng.uniform(-0.8, 0.8))
    if c['tracked'] and rng.random() < 0.4:  # lookup filters
      t = rng.choice(c['tracked'])
      t[2] = rng.choice([0, 999999])
      if pool:
        pool[rng.randrange(len(pool))]['source'] = rng.choice([2, 3, 4])
    if rng.random() < 0.2:
      c['norm'] = {'scale': rng.uniform(0.5, 2.0), 'offset': [rng.uniform(-0.5, 0.5) for _ in range(3)]}
    out.append(c)
  return out


def flat(cs):
  """Whitespace-separated text for sr_iwt_replay (fixed field order; see tests/image_target/iwt_replay_main.cc)."""
  lines = ['iwt-cases 1 %d' % len(cs)]
  def h(x):
    return ' '.join(repr(v) for v in x['m'] + x['inv']) + ' %d %d' % (x['noInvert'], x['invertFailed'])
  for c in cs:
    lines.append('case %s' % c['name'])
    lines.append('state %r %s' % (c['state']['scale'], h(c['state']['world_pose'])))
    for pi, pool in enumerate(c['map_pools']):
      lines.append('pool %d %d' % (pi, len(pool)))
      for pt in pool:
        lines.append('pt %d %s %d' % (pt['id'], ' '.join(repr(v) for v in pt['pos']), pt['source']))
    lines.append('norm %r %s' % (c['norm']['scale'], ' '.join(repr(v) for v in c['norm']['offset'])))
    lines.append('tracked %d' % len(c['tracked']))
    for t in c['tracked']:
      lines.append('t %d %d %d' % tuple(t))
    g = c['image']['geom'] or {'radius': 0, 'height': 0, 'left': 0, 'right': 0, 'top': 0, 'bottom': 0,
                               'src_rows': 0, 'src_cols': 0, 'is_cone': False, 'radius_bottom': 0}
    lines.append('image %d %s %r %r %r %r %r %r %d %d %d %r' % (
        c['image']['type'], ' '.join(repr(v) for v in c['image']['target_k']), g['radius'], g['height'],
        g['left'], g['right'], g['top'], g['bottom'], g['src_rows'], g['src_cols'], int(g['is_cone']),
        g['radius_bottom']))
    lines.append('pose %s' % h(c['target_pose']))
    lines.append('tam %s' % h(c['tam']))
    fr = c['frame']
    lines.append('frame %d %s' % (len(fr['rays']), ' '.join(repr(v) for v in fr['cam_k'])))
    for r, px in zip(fr['rays'], fr['pixels']):
      lines.append('f %r %r %r %r %r' % (r[0], r[1], px[0], px[1], px[2]))
    lines.append('guess %r' % c['scale_guess'])
  return '\n'.join(lines) + '\n'


def main():
  if '--variant' in sys.argv and sys.argv[sys.argv.index('--variant') + 1] == 'random':
    cs = random_cases(int(sys.argv[sys.argv.index('--seed') + 1]), int(sys.argv[sys.argv.index('--n') + 1]))
  else:
    cs = cases()
  json.dump({'cases': cs}, open(sys.argv[1], 'w'), indent=1)
  if '--flat' in sys.argv:
    open(sys.argv[sys.argv.index('--flat') + 1], 'w').write(flat(cs))
  print(len(cs), 'cases')


if __name__ == '__main__':
  main()

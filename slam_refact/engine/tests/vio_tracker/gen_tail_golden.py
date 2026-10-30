#!/usr/bin/env python3
"""Per-case golden vectors for the WP4 VioTracker gates V-H1/H2/H3, V-XR, V-3DOF, V-LATCH.

The spec lane probes (slam_refact/tools/re/spec_vio_tracker/probe_func784*.py) emulate the real
func784 WAT instructions but only store aggregate errors. This script imports those probes
unchanged and re-runs the same emulations, storing every case's inputs and emulated outputs so the
C++ unit tests can compare against the original instructions case by case. Tolerances are the
pre-registered ones of spec 04 §8.1 and live in the C++ tests, not here.

Usage (repo root):
  python3 -I slam_refact/engine/tests/vio_tracker/gen_tail_golden.py \
    --wat build/rea-analysis-20261015/xr_reference/func-784.wat.txt --out <new dir>/wp4-tail-golden.json
"""
import argparse
import hashlib
import json
import os
import random
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
PROBES = os.path.normpath(os.path.join(HERE, '..', '..', '..', 'tools', 're', 'spec_vio_tracker'))
sys.path.insert(0, PROBES)

import sympy as sp  # noqa: E402
import probe_func784 as mp  # noqa: E402
import probe_func784_tail as tail  # noqa: E402
from wat_emu import Emu, parse, camera_motion, f32  # noqa: E402

B = sp.Symbol('b')
A = sp.Symbol('a')


def pose_arrays(p, q):
  m, inv = camera_motion(p, q)
  return [f32(v) for v in m], [f32(v) for v in inv]


def read16(emu, base, off):
  return [emu.lf(base, off + 4 * k) for k in range(16)]


def yaw_cases(lines, rng, n):
  """H1 (first frame: Y from tam, output Y*P) and H2 (later frame: Y kept, output Y*P2)."""
  cases = []
  for _ in range(n):
    s = dict(p=[rng.uniform(-2, 2) for _ in range(3)], q=tail.rand_quat(rng), p_prev=[0, 0, 0],
             q_prev=(0, 0, 0, 1), v_prev=[0, 0, 0], t_prev=1_000_000_000, t_cur=1_050_000_000, yaw_init=0)
    emu, _, out, st = tail.run(lines, s)
    y = read16(emu, B, 6212)
    yinv = read16(emu, B, 6276)
    s2 = dict(s, p=[rng.uniform(-2, 2) for _ in range(3)], q=tail.rand_quat(rng), yaw_init=1, Y=y, Yinv=yinv)
    _, _, out2, _ = tail.run(lines, s2)
    m1, i1 = pose_arrays(s['p'], s['q'])
    m2, i2 = pose_arrays(s2['p'], s2['q'])
    cases.append(dict(tam1=m1, tam1_inv=i1, tam2=m2, tam2_inv=i2, out1=out, yaw_offset=y,
                      yaw_offset_inv=yinv, out2=out2, status1=list(st)))
  return cases


def velocity_cases(lines, rng, n):
  """H3 (history/velocity/acceleration) and H3b (world-transform frame keeps the velocity)."""
  cases = []
  for _ in range(n):
    dt_ns = rng.choice([0, rng.randint(10, 90) * 1_000_000])
    s = dict(p=[rng.uniform(-2, 2) for _ in range(3)], q=tail.rand_quat(rng),
             p_prev=[rng.uniform(-2, 2) for _ in range(3)], q_prev=tail.rand_quat(rng),
             v_prev=[rng.uniform(-1, 1) for _ in range(3)], t_prev=2_000_000_000, t_cur=2_000_000_000 + dt_ns,
             yaw_init=1)
    case = dict(t_prev=s['t_prev'], t_cur=s['t_cur'], v_prev=[f32(v) for v in s['v_prev']])
    case['prev_pose'], case['prev_pose_inv'] = pose_arrays(s['p_prev'], s['q_prev'])
    case['pose'], case['pose_inv'] = pose_arrays(s['p'], s['q'])
    for jump in (0, 1):
      emu, _, _, _ = tail.run(lines, dict(s, pose_jump=jump))
      key = 'jump' if jump else 'normal'
      case[key] = dict(velocity=[emu.lf(B, 7636 + 4 * k) for k in range(3)],
                       acceleration=[emu.lf(B, 7652 + 4 * k) for k in range(3)],
                       hist1_pose=read16(emu, B, 7504))
    cases.append(case)
  return cases


def xr_cases(lines, rng, n):
  """V-XR: xrRotationFromDeviceRotation as executed inside the MotionPrediction block."""
  cases = []
  for t in range(n):
    _, _, scen, _, _ = mp.run_one(lines, rng, 0.0, 1.0, t)
    init = mp.build_init(None, scen)  # build_init draws nothing from its rng argument
    emu = Emu(lines, ['a', 'b', 'c', 'd', 'e', 'f'], init, {27401: 0.0, 30496: 1.0})
    emu.run(mp.START, mp.END, mp.JUMPS)
    xr = [emu.lf(B, 7720 + 4 * k) for k in range(4)]
    cases.append(dict(device_pose_raw_xyzw=[f32(v) for v in scen['dev_raw']], xr_xyzw=xr))
  return cases


def disable_vio_cases(lines, rng, n):
  """V-3DOF: tamExtrinsic written by the disableVio branch (forward + inverse, 32 floats)."""
  cases = []
  for _ in range(n):
    q = mp.rand_quat(rng)
    init = {('b + 4', 'i32.load8_u'): 1}
    for k in range(4):
      init[(f'c + {104 + 4 * k}', 'f32')] = f32(q[k])
    emu = Emu(lines, ['a', 'b', 'c', 'd', 'e', 'f'], init)
    emu.locals[173] = (0.0, 0.0, 0.0, 0.0)
    emu.run(0x08BB7D, 0x08BEDB)
    cases.append(dict(device_pose_raw_xyzw=[f32(v) for v in q], tam=read16(emu, B, 6464),
                      tam_inv=read16(emu, B, 6528)))
  return cases


def latch_cases(lines, rng, n):
  """V-LATCH: estimateScale frame where MSE first reports a scale (944 stubbed to true)."""
  cases = []
  for _ in range(n):
    sc = rng.uniform(0.2, 3.0)
    s = dict(p=[rng.uniform(-2, 2) for _ in range(3)], q=tail.rand_quat(rng), p_prev=[0, 0, 0],
             q_prev=(0, 0, 0, 1), v_prev=[0, 0, 0], t_prev=1_000_000_000, t_cur=1_050_000_000, yaw_init=1,
             estimate_scale=1, scale=sc, scale_ready=0)
    emu, _, out, st = tail.run(lines, s, mse_result=1)
    tam, tam_inv = pose_arrays(s['p'], s['q'])
    cases.append(dict(mse_scale=f32(sc), tam=tam, tam_inv=tam_inv, out=out, metric=read16(emu, B, 2848),
                      metric_inv=read16(emu, B, 2912), latched_scale=emu.lf(B, 2844),
                      yaw_offset=read16(emu, B, 6212), yaw_offset_inv=read16(emu, B, 6276), status=list(st)))
  return cases


def main():
  ap = argparse.ArgumentParser()
  ap.add_argument('--wat', required=True)
  ap.add_argument('--trials', type=int, default=30)
  ap.add_argument('--out', required=True)
  a = ap.parse_args()
  if os.path.exists(a.out):
    sys.exit(f'refusing to overwrite {a.out}')
  lines = parse(a.wat)
  rng = random.Random(20261016)
  with open(a.wat, 'rb') as f:
    wat_sha = hashlib.sha256(f.read()).hexdigest()
  res = dict(source='func784 WAT emulation via spec_vio_tracker probes (wat_emu.py), per-case vectors',
             wat=a.wat, wat_sha256=wat_sha, seed=20261016, trials=a.trials,
             note='helper calls 519/520/521/461/462/945 modelled with public c8 semantics (see wat_emu.py)')
  res['yaw_output'] = yaw_cases(lines, rng, a.trials)
  res['velocity'] = velocity_cases(lines, rng, a.trials)
  res['xr_rotation'] = xr_cases(lines, rng, a.trials)
  res['disable_vio'] = disable_vio_cases(lines, rng, a.trials)
  res['metric_latch'] = latch_cases(lines, rng, max(4, a.trials // 3))
  os.makedirs(os.path.dirname(os.path.abspath(a.out)), exist_ok=True)
  with open(a.out, 'w') as f:
    json.dump(res, f)
  print(json.dumps({k: len(v) for k, v in res.items() if isinstance(v, list)}))


if __name__ == '__main__':
  main()

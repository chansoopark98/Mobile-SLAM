#!/usr/bin/env python3
"""Decode the inlined MotionPrediction block of func784 (VioTracker::track) by emulation.

Runs the real WAT instructions of func784 from its entry up to the memcpy of the predicted
pose (0x087116), with explicit inputs, and compares the produced predicted pose (frame+480)
with candidate closed-form formulas. Pre-registered pass criterion (fixed before running):
max |emulated - formula| <= 1e-5 for every quaternion/position component over all random
trials.

Usage (repo root):  python3 -I slam_refact/tools/re/spec_vio_tracker/probe_func784.py \
    --wat build/rea-analysis-20261015/xr_reference/func-784.wat.txt [--trials 200] [--out FILE.json]
"""
import argparse
import json
import math
import os
import random
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from wat_emu import (Emu, parse, quat_mul, quat_norm, quat_to_rotmat,  # noqa: E402
                     ground_plane_rotation_rads, camera_motion, f32)

START, END = 0x0864F7, 0x087119
JUMPS = {0x086561: 0x0865BB,   # std::vector<TrackingSensorEvent> copy (heap)
         0x0866F2: 0x086713,   # free old event vector
         0x086846: 0x08685F}   # vector assign (func896)
FRAME = 'G0 - 3168'


def rand_quat(rng):
    q = [rng.gauss(0, 1) for _ in range(4)]
    return quat_norm(q)


def xr_from_device(q):
    """public device-pose.cc:79 xrRotationFromDeviceRotation, (x,y,z,w) order."""
    x, y, z, w = quat_norm(q)
    inv = (-x, -y, -z, w)
    a = (0.7071067811865476, 0.0, 0.0, 0.7071067811865476)  # Quaternion(w=.7071, x=.7071)
    q1 = quat_norm(quat_mul(a, inv))
    return (q1[0], q1[2], q1[1], q1[3])  # Quaternion(w, x, z, y)


def slerp(a, b, t):
    """public quaternion.cc:388 Quaternion::interpolate."""
    a = quat_norm(a)
    b = quat_norm(b)
    dot = sum(x * y for x, y in zip(a, b))
    if dot < 0:
        b = tuple(-v for v in b)
        dot = -dot
    if dot > 0.995:
        return quat_norm(tuple(x + t * (y - x) for x, y in zip(a, b)))
    s2 = 1.0 - dot * dot
    sh = math.sqrt(s2)
    r0 = math.atan2(sh, dot)
    sr0 = math.sin(r0)
    r = t * r0
    sr = math.sin(r)
    ta = math.cos(r) - dot * sr / sr0
    tb = sr / sr0
    return quat_norm(tuple(ta * x + tb * y for x, y in zip(a, b)))


def yaw_quat(theta):
    return (0.0, math.sin(theta / 2), 0.0, math.cos(theta / 2))


def build_init(rng, scen):
    init = {}

    def put(addr, kind, v):
        init[(addr, kind)] = v

    def putq(base, q):
        for k in range(4):
            put(f'{base} + {4 * k}' if 4 * k else base, 'f32', f32(q[k]))
    # frame input c (TrackingSensorFrame)
    put('c + 8', 'i64.load', scen['t_cur'])
    for k, v in enumerate(scen['dev_raw']):
        put(f'c + {104 + 4 * k}', 'f32', f32(v))
    put('c + 144', 'i32.load', 0)
    put('c + 148', 'i32.load', 0)
    # tracker state b
    put('b + 3008', 'i64.load', scen['t_first'])
    put('b + 7672', 'i64.load', scen['t_prev'])
    for k, v in enumerate(scen['dev_prev_xr']):
        put(f'b + {7680 + 4 * k}', 'f32', f32(v))
    m, inv = camera_motion(scen['p_prev'], scen['q_prev'])
    for k in range(16):
        put(f'b + {7504 + 4 * k}', 'f32', f32(m[k]))
        put(f'b + {7568 + 4 * k}', 'f32', f32(inv[k]))
    for k, v in enumerate(scen['v_prev']):
        put(f'b + {7636 + 4 * k}', 'f32', f32(v))
    put('b + 7736', 'i32.load', 0)
    put('b + 7712', 'i64.load', 0)
    put('611892', 'i32.load8_u', 0)  # params not yet cached -> getter path
    return init


def expected(scen, gci, drag):
    """Candidate decoding (written before the comparison, see module doc)."""
    q_dev = xr_from_device(scen['dev_raw'])
    q_dev_prev = quat_norm(scen['dev_prev_xr'])
    inv_prev = (-q_dev_prev[0], -q_dev_prev[1], -q_dev_prev[2], q_dev_prev[3])
    delta = quat_norm(quat_mul(inv_prev, q_dev))           # dev_prev^-1 * dev_cur
    q_pred = quat_norm(quat_mul(quat_norm(scen['q_prev']), delta))  # cam_prev * delta
    yaw_pred = ground_plane_rotation_rads(q_pred)
    yaw_dev = ground_plane_rotation_rads(q_dev)
    q_gc = quat_norm(quat_mul(yaw_quat(yaw_pred - yaw_dev), q_dev))  # R_y(dyaw) * dev
    q_out = slerp(q_gc, q_pred, gci)
    # above-horizon factor from the device forward (+z) world-y component
    fy = quat_to_rotmat(q_dev)[1][2]
    above = 0.0 if 3 * fy < 0 else min(3 * fy, 1.0)
    k = 1.0 - above * drag
    dt = (scen['t_cur'] - scen['t_prev']) * 1e-9
    dt = 1.0 if dt == 0 else dt
    p_out = tuple(p + dt * k * v for p, v in zip(scen['p_prev'], scen['v_prev']))
    return q_out, p_out, dict(q_dev=q_dev, q_pred=q_pred, q_gc=q_gc, above=above)


def run_one(lines, rng, gci, drag, trial):
    t_first = 10_000_000_000
    t_prev = t_first + rng.randint(1, 200) * 1_000_000
    t_cur = t_prev + rng.choice([0, rng.randint(10, 80) * 1_000_000])
    dev_raw = rand_quat(rng)
    # previous device quaternion: near the current one most of the time
    if trial % 3 == 0:
        dev_prev_xr = rand_quat(rng)
    else:
        q = xr_from_device(dev_raw)
        dev_prev_xr = quat_norm(tuple(v + rng.gauss(0, 0.05) for v in q))
    scen = dict(t_first=t_first, t_prev=t_prev, t_cur=t_cur, dev_raw=dev_raw, dev_prev_xr=dev_prev_xr,
                q_prev=rand_quat(rng), p_prev=tuple(rng.uniform(-2, 2) for _ in range(3)),
                v_prev=tuple(rng.uniform(-1, 1) for _ in range(3)))
    overrides = {27401: gci, 30496: drag}
    emu = Emu(lines, ['a', 'b', 'c', 'd', 'e', 'f'], build_init(rng, scen), overrides)
    emu.run(START, END, JUMPS)
    base = f'{FRAME} + 480'
    import sympy as sp
    m = [emu.lf(sp.sympify(base), 4 * k) for k in range(16)]
    from wat_emu import quat_from_colmajor
    q_emu = quat_norm(quat_from_colmajor(m))
    p_emu = (m[12], m[13], m[14])
    q_exp, p_exp, extra = expected(scen, gci, drag)
    if sum(a * b for a, b in zip(q_emu, q_exp)) < 0:
        q_exp = tuple(-v for v in q_exp)
    dq = max(abs(a - b) for a, b in zip(q_emu, q_exp))
    dp = max(abs(a - b) for a, b in zip(p_emu, p_exp))
    case = dict(inputs=dict(t_first_ns=scen['t_first'], t_prev_ns=scen['t_prev'], t_cur_ns=scen['t_cur'],
                            device_pose_raw_xyzw=list(scen['dev_raw']), prev_device_xr_xyzw=list(scen['dev_prev_xr']),
                            prev_cam_q_xyzw=list(scen['q_prev']), prev_cam_p=list(scen['p_prev']),
                            prev_velocity=list(scen['v_prev']), gravityCorrectedInterpolation=gci, aboveHorizonDrag=drag),
                outputs_emulated=dict(pred_q_xyzw=list(q_emu), pred_p=list(p_emu), pred_matrix_colmajor=m))
    return dq, dp, scen, extra, case


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--wat', required=True)
    ap.add_argument('--trials', type=int, default=200)
    ap.add_argument('--seed', type=int, default=20261015)
    ap.add_argument('--out')
    ap.add_argument('--golden', help='write emulated input/output cases (oracle vectors for C++ tests)')
    ap.add_argument('--golden-per-setting', type=int, default=50)
    a = ap.parse_args()
    lines = parse(a.wat)
    rng = random.Random(a.seed)
    results = []
    golden = []
    for gci, drag in [(0.0, 1.0), (0.3, 1.0), (1.0, 0.5), (0.7, 0.0)]:
        worst_q = worst_p = 0.0
        n_above = 0
        for t in range(a.trials):
            dq, dp, scen, extra, case = run_one(lines, rng, gci, drag, t)
            if t < a.golden_per_setting:
                golden.append(case)
            worst_q = max(worst_q, dq)
            worst_p = max(worst_p, dp)
            n_above += extra['above'] > 0
        ok = worst_q <= 1e-5 and worst_p <= 1e-5
        results.append(dict(gravityCorrectedInterpolation=gci, aboveHorizonDrag=drag, trials=a.trials,
                            max_abs_quat_err=worst_q, max_abs_pos_err=worst_p, trials_with_above_horizon=n_above,
                            pass_=ok))
        print(json.dumps(results[-1]))
    if a.out:
        json.dump(dict(start=hex(START), end=hex(END), jumps={hex(k): hex(v) for k, v in JUMPS.items()},
                       seed=a.seed, tolerance=1e-5, results=results), open(a.out, 'w'), indent=1)
    if a.golden:
        json.dump(dict(source='func784 WAT emulation (wat_emu.py), MotionPrediction block 0x0864f7..0x087119',
                       note='float32 rounding emulated; helper calls 519/520/521/461/462/897-899 modelled with public c8 semantics',
                       cases=golden), open(a.golden, 'w'))


if __name__ == '__main__':
    main()

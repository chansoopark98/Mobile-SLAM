#!/usr/bin/env python3
"""Decode the tail of func784 (VioTracker::track): pose history/velocity update, tracking-status
state machine, first-frame yaw offset, responsive/absolute output composition.

Emulates the real WAT instructions from 0x090143 (dcmp 84939, label B_p) to the function return
(0x0944f6) with explicit inputs and checks pre-registered hypotheses (tolerance 1e-5, fixed
before running). Helper calls are modelled with public c8 semantics (see wat_emu.py); the
MetricScaleEstimator call (944) is stubbed: it returns a chosen bool and the estimated scale is
pre-seeded at tracker+2832.

Usage: python3 -I probe_func784_tail.py --wat build/rea-analysis-20261015/xr_reference/func-784.wat.txt [--out F.json]
"""
import argparse
import json
import math
import os
import random
import sys

import sympy as sp

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import wat_emu  # noqa: E402
from wat_emu import (Emu, parse, quat_norm, quat_to_rotmat, quat_from_colmajor,  # noqa: E402
                     ground_plane_rotation_rads, camera_motion, f32)

START, END = 0x090143, 0x0944F6
G = sp.sympify('G0 - 3168')
B = sp.Symbol('b')
A = sp.Symbol('a')
STATUS = sp.Symbol('status')


class TailEmu(Emu):
    """Adds heap/no-op calls and a stub for MetricScaleEstimator::pushFrame (944)."""

    def __init__(self, *args, mse_result=0, **kw):
        super().__init__(*args, **kw)
        self.mse_result = mse_result
        self.heap = 0

    def call(self, fn, addr):
        if fn == 370:  # operator new
            self.pop()
            self.heap += 1
            return sp.Symbol(f'heap{self.heap}')
        if fn in (381, 598):  # operator delete / tree destroy
            self.pop()
            self.calls.append((hex(addr), fn))
            return None
        if fn == 665:  # history shift (decompile ram:80059807_unnamed_function_665.c), empty vectors only
            out, qa, pose, n = self.popn(4)
            self._copy(out, qa + 208, 164)
            self._copy(out + 184, qa + 392, 8)
            self._copy(out + 168, qa + 376, 16)
            for off in (192, 196, 200, 400, 404, 408):
                self.store(out + off, 'i32.load', 0)
            self._copy(out + 208, pose, 130)
            self._copy(out + 340, qa + 340, 32)
            self._copy(out + 384, n + 8, 16)
            self._copy(out + 376, n, 8)
            self.calls.append((hex(addr), fn))
            return None
        if fn == 944:
            self.popn(6)
            self.calls.append((hex(addr), fn))
            return self.mse_result
        if fn == 945:  # trsRotation(HMatrix) -> Quaternion out
            out, m = self.popn(2)
            mat = [self.lf(m, 4 * k) for k in range(16)]
            q = trs_rotation(mat)
            for k in range(4):
                self.store(out + 4 * k, 'f32', f32(q[k]))
            self.calls.append((hex(addr), fn))
            return None
        return super().call(fn, addr)


def _copy(self, dst, src, n):
    db, do = self.split(dst)
    sb, so = self.split(src)
    for k in range(n):
        self.bytes.pop((db, do + k), None)
        b = self.bytes.get((sb, so + k))
        if b is not None:
            self.bytes[(db, do + k)] = b


TailEmu._copy = _copy


def trs_rotation(m):
    """public egomotion.cc:71/91 trsRotation = rotation(cam * trsScaleMat(cam).inv())."""
    cols = [[m[4 * c + r] for r in range(3)] for c in range(3)]
    s = [math.sqrt(sum(v * v for v in col)) for col in cols]
    det = (cols[0][0] * (cols[1][1] * cols[2][2] - cols[2][1] * cols[1][2])
           - cols[1][0] * (cols[0][1] * cols[2][2] - cols[2][1] * cols[0][2])
           + cols[2][0] * (cols[0][1] * cols[1][2] - cols[1][1] * cols[0][2]))
    if det < 0:
        s[0] = -s[0]
    mm = list(m)
    for c in range(3):
        for r in range(3):
            mm[4 * c + r] = m[4 * c + r] / s[c]
    return quat_from_colmajor(mm)


def mat_mul(x, y):
    """column-major 4x4 product x*y."""
    out = [0.0] * 16
    for c in range(4):
        for r in range(4):
            out[4 * c + r] = sum(x[4 * k + r] * y[4 * c + k] for k in range(4))
    return out


def rand_quat(rng):
    return quat_norm([rng.gauss(0, 1) for _ in range(4)])


def ry(theta):
    c, s = math.cos(theta), math.sin(theta)
    m = [0.0] * 16
    m[0], m[2], m[8], m[10], m[5], m[15] = c, -s, s, c, 1.0, 1.0
    return m


def base_init(rng, P, Pinv, s):
    init = {}

    def put(addr, kind, v):
        init[(str(addr), kind)] = v
    for k in range(16):
        put(B + 6464 + 4 * k, 'f32', f32(P[k]))
        put(B + 6528 + 4 * k, 'f32', f32(Pinv[k]))
    put(B + 6592, 'i32.load8_u', 0)
    put(B + 6593, 'i32.load8_u', 0)
    put(B + 6628, 'i32.load8_u', s.get('pose_jump', 0))
    # history slot "cur" (previous frame): pose HMatrix at b+7504 (+inverse), vel/acc, ts, quat
    Pm, Pminv = camera_motion(s['p_prev'], s['q_prev'])
    for k in range(16):
        put(B + 7504 + 4 * k, 'f32', f32(Pm[k]))
        put(B + 7568 + 4 * k, 'f32', f32(Pminv[k]))
    put(B + 7632, 'i32.load16_u', 0)
    for k in range(3):
        put(B + 7636 + 4 * k, 'f32', f32(s['v_prev'][k]))
        put(B + 7652 + 4 * k, 'f32', 0.0)
    put(B + 7648, 'i32.load', 0)
    put(B + 7664, 'i32.load', 0)
    put(B + 7668, 'i32.load', 0)
    put(B + 7672, 'i64.load', s['t_prev'])
    for k in range(4):
        put(B + 7680 + 4 * k, 'f32', 0.0)
    for off in (7488, 7492, 7496, 7696, 7700, 7704, 7280, 7284):
        put(B + off, 'i32.load', 0)
    # current-frame record n = frame+648 : ts, xr quat, imu vector
    put(G + 648, 'i64.load', s['t_cur'])
    for k in range(4):
        put(G + 656 + 4 * k, 'f32', 0.0)
    for off in (672, 676, 680, 448, 452, 456):
        put(G + off, 'i32.load', 0)
    put(sp.Symbol('c') + 8, 'i64.load', s['t_cur'])
    # flags / counters / status machine
    put(B + 2980, 'i32.load16_u', 0)          # vpsMode = 0
    put(B + 6904, 'i32.load8_u', s.get('maptracker_flag136', 0))
    put(B + 6900, 'i32.load', s.get('maptracker_state', 1))
    put(B + 6368, 'i32.load', s.get('lost', 0))
    put(B + 6372, 'i32.load', s.get('frames', 0))
    put(B + 6352, 'i32.load', s.get('state', 0))
    put(B + 6360, 'i64.load', s.get('t_state', 0))
    for off in (6376, 6377, 6378, 6379, 6380, 6381):
        put(B + off, 'i32.load8_u', s.get(f'flag{off}', 0))
    put(sp.Symbol('f') + 8, 'i32.load', STATUS)
    put(B + 6344, 'i32.load8_u', s.get('yaw_init', 0))
    Y = s.get('Y', ry(0.0))
    Yinv = s.get('Yinv', ry(0.0))
    for k in range(16):
        put(B + 6212 + 4 * k, 'f32', f32(Y[k]))
        put(B + 6276 + 4 * k, 'f32', f32(Yinv[k]))
    put(B + 6340, 'i32.load8_u', 0)
    put(B + 6341, 'i32.load8_u', 0)
    put(B + 6600, 'i32.load8_u', 0)
    put(B + 4, 'i32.load8_u', s.get('disable_vio', 0))
    put(B + 5, 'i32.load8_u', s.get('estimate_scale', 0))
    put(B + 2840, 'i32.load8_u', s.get('scale_ready', 0))
    put(B + 2832, 'f32', f32(s.get('scale', 1.0)))
    put(B + 2844, 'f32', f32(s.get('scale_latched', 1.0)))
    put(B + 2976, 'i32.load8_u', 0)
    put(B + 2977, 'i32.load8_u', 0)
    M = s.get('M', [1.0 if k in (0, 5, 10, 15) else 0.0 for k in range(16)])
    for k in range(32):
        put(B + 2848 + 4 * k, 'f32', f32(M[k % 16] if k < 16 else (M[k - 16] if 'Minv' not in s else s['Minv'][k - 16])))
    return init


def run(lines, s, mse_result=0):
    rng = random.Random(1)
    P, Pinv = camera_motion(s['p'], s['q'])
    emu = TailEmu(lines, ['a', 'b', 'c', 'd', 'e', 'f'], base_init(rng, P, Pinv, s), {}, mse_result=mse_result)
    emu.locals.update({6: G, 12: B + 7504, 13: G + 648, 14: B + 7672, 42: B + 7296, 43: s.get("ra", 1),
                       173: (0.0, 0.0, 0.0, 0.0)})
    emu.run(START, END)
    out = [emu.lf(A, 4 * k) for k in range(16)]
    st = (emu.load(STATUS, 'i32.load16_u'), emu.load(STATUS + 2, 'i32.load16_u'))
    return emu, P, out, st


def check_yaw_and_output(lines, rng, n):
    """H1: first frame -> Y = R_y(-yaw(P)); output = Y * P. H2: Y kept afterwards; output = Y*P."""
    worst1 = worst2 = 0.0
    for _ in range(n):
        s = dict(p=[rng.uniform(-2, 2) for _ in range(3)], q=rand_quat(rng), p_prev=[0, 0, 0], q_prev=(0, 0, 0, 1),
                 v_prev=[0, 0, 0], t_prev=1_000_000_000, t_cur=1_050_000_000, yaw_init=0)
        emu, P, out, _ = run(lines, s)
        yaw0 = ground_plane_rotation_rads(s['q'])
        exp = mat_mul(ry(-yaw0), P)
        worst1 = max(worst1, max(abs(x - y) for x, y in zip(out, exp)))
        Y = [emu.lf(B, 6212 + 4 * k) for k in range(16)]
        # H2: second frame with a different P, Y as stored
        s2 = dict(s, p=[rng.uniform(-2, 2) for _ in range(3)], q=rand_quat(rng), yaw_init=1, Y=Y,
                  Yinv=[emu.lf(B, 6276 + 4 * k) for k in range(16)])
        _, P2, out2, _ = run(lines, s2)
        exp2 = mat_mul(Y, P2)
        worst2 = max(worst2, max(abs(x - y) for x, y in zip(out2, exp2)))
    return dict(H1_first_frame_yaw_offset_output_max_err=worst1, H2_later_output_max_err=worst2)


def check_velocity(lines, rng, n):
    """H3: v_new = (p_new - p_prev)/dt (dt==0 -> 1/dt := 1), acc = (v_new - v_prev)/dt;
    pose_jump (b+6628) -> velocity copied unchanged."""
    worst = 0.0
    worst_jump = 0.0
    for _ in range(n):
        dt_ns = rng.choice([0, rng.randint(10, 90) * 1_000_000])
        s = dict(p=[rng.uniform(-2, 2) for _ in range(3)], q=rand_quat(rng), p_prev=[rng.uniform(-2, 2) for _ in range(3)],
                 q_prev=rand_quat(rng), v_prev=[rng.uniform(-1, 1) for _ in range(3)], t_prev=2_000_000_000,
                 t_cur=2_000_000_000 + dt_ns, yaw_init=1)
        emu, P, out, _ = run(lines, s)
        inv_dt = 1.0 if dt_ns == 0 else 1.0 / (dt_ns * 1e-9)
        v = [(s['p'][k] - s['p_prev'][k]) * inv_dt for k in range(3)]
        acc = [(v[k] - s['v_prev'][k]) * inv_dt for k in range(3)]
        got_v = [emu.lf(B, 7636 + 4 * k) for k in range(3)]
        got_a = [emu.lf(B, 7652 + 4 * k) for k in range(3)]
        got_p = [emu.lf(B, 7504 + 48 + 4 * k) for k in range(3)]
        rel = max(abs(x - y) / max(1.0, abs(y)) for x, y in zip(got_v + got_a + got_p, v + acc + s['p']))
        worst = max(worst, rel)
        s_j = dict(s, pose_jump=1)
        emu_j, _, _, _ = run(lines, s_j)
        got_vj = [emu_j.lf(B, 7636 + 4 * k) for k in range(3)]
        worst_jump = max(worst_jump, max(abs(x - y) for x, y in zip(got_vj, s['v_prev'])))
    return dict(H3_velocity_acc_history_max_rel_err=worst, H3b_pose_jump_keeps_velocity_max_err=worst_jump)


def status_table(lines):
    """Enumerate the loss state machine (no hypothesis: recorded as decoded table)."""
    rows = []
    base = dict(p=[0.1, 0.2, 0.3], q=(0, 0, 0, 1), p_prev=[0, 0, 0], q_prev=(0, 0, 0, 1), v_prev=[0, 0, 0],
                t_prev=10_000_000_000, yaw_init=1)
    for state in range(5):
        for ra in (0, 1):
            for lost, frames in ((0, 0), (0, 9), (2, 9), (9, 9), (1, 4)):
                for dt_state in (1_000_000_000, 2_500_000_000):
                    s = dict(base, state=state, ra=ra, lost=lost, frames=frames, t_cur=10_000_000_000 + 50_000_000,
                             t_state=10_050_000_000 - dt_state)
                    emu, _, _, st = run(lines, s)
                    rows.append(dict(state_in=state, ra=ra, lost_in=lost, frames_in=frames, since_state_ns=dt_state,
                                     state_out=emu.load(B + 6352, 'i32.load'), lost_out=emu.load(B + 6368, 'i32.load'),
                                     frames_out=emu.load(B + 6372, 'i32.load'), status=st[0], reason=st[1]))
    return rows


def spec_state_machine(state, ok, lost, frames, since_ns):
    """Spec 04 §3.3 pseudo-code (pre-registered); returns (state, lost, frames, status, reason)."""
    frames += 1
    lost += 0 if ok else 1
    nxt, change = state, False
    if state == 0:
        if lost > 0:
            nxt, change = 1, True
    elif state in (1, 3):
        if since_ns >= 2_000_000_000:
            nxt, change = (0 if lost / frames < 0.2 else 2), True
    elif state == 2:
        if lost != frames:
            nxt, change = 3, True
    elif lost > 0:
        nxt, change = 1, True
    if change:
        lost = frames = 0
    return nxt, lost, frames, (2 if nxt in (2, 3) else 3), 0


def check_state_machine(rows):
    """H4: emulated table == spec pseudo-code, exact."""
    bad = 0
    for r in rows:
        exp = spec_state_machine(r['state_in'], r['ra'], r['lost_in'], r['frames_in'], r['since_state_ns'])
        got = (r['state_out'], r['lost_out'], r['frames_out'], r['status'], r['reason'])
        bad += exp != got
    return dict(H4_state_machine_rows=len(rows), H4_state_machine_mismatches=bad)


def flags_table(lines):
    rows = []
    base = dict(p=[0.1, 0.2, 0.3], q=(0, 0, 0, 1), p_prev=[0, 0, 0], q_prev=(0, 0, 0, 1), v_prev=[0, 0, 0],
                t_prev=10_000_000_000, t_cur=10_050_000_000, yaw_init=1, state=0, ra=1)
    for off in (6376, 6378, 6380):
        for a_ in (0, 1):
            for b_ in (0, 1):
                s = dict(base, **{f'flag{off}': a_, f'flag{off + 1}': b_})
                _, _, _, st = run(lines, s)
                rows.append(dict(flag=off, set=a_, ack=b_, status=st[0], reason=st[1]))
    return rows


def absolute_probe(lines, rng, n):
    """Record what the estimateScale path does when MSE first reports a scale (b+2840 0->1)."""
    out_rows = []
    for _ in range(n):
        sc = rng.uniform(0.2, 3.0)
        s = dict(p=[rng.uniform(-2, 2) for _ in range(3)], q=rand_quat(rng), p_prev=[0, 0, 0], q_prev=(0, 0, 0, 1),
                 v_prev=[0, 0, 0], t_prev=1_000_000_000, t_cur=1_050_000_000, yaw_init=1, estimate_scale=1,
                 scale=sc, scale_ready=0)
        emu, P, out, st = run(lines, s, mse_result=1)
        M = [emu.lf(B, 2848 + 4 * k) for k in range(16)]
        yaw = ground_plane_rotation_rads(s['q'])
        latched = emu.lf(B, 2844)
        # candidate: M = S(1/s) * T(-p_x, 0, -p_z)?  record residuals of a few candidates
        G_ = mat_mul(M, P)
        exp_out_t = [G_[12], G_[13], G_[14]]
        out_rows.append(dict(scale=sc, latched=latched, yaw=yaw, p=s['p'], M=M, out_t=[out[12], out[13], out[14]],
                             MP_t=exp_out_t, status=st))
    return out_rows


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--wat', required=True)
    ap.add_argument('--trials', type=int, default=30)
    ap.add_argument('--out')
    a = ap.parse_args()
    lines = parse(a.wat)
    rng = random.Random(20261015)
    res = {}
    res.update(check_yaw_and_output(lines, rng, a.trials))
    res.update(check_velocity(lines, rng, a.trials))
    res['status_table'] = status_table(lines)
    res.update(check_state_machine(res['status_table']))
    res['pass'] = (all(v <= 1e-5 for k, v in res.items() if k.startswith('H') and 'max' in k)
                   and res['H4_state_machine_mismatches'] == 0)
    print(json.dumps({k: v for k, v in res.items() if k != 'status_table'}, indent=1))
    res['flags_table'] = flags_table(lines)
    res['absolute_first_scale'] = absolute_probe(lines, rng, 4)
    if a.out:
        json.dump(res, open(a.out, 'w'), indent=1)


if __name__ == '__main__':
    main()

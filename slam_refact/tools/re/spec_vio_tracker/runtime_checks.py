#!/usr/bin/env python3
"""Check VioTracker output-composition predictions (spec 04) against the original reference runtime
reference traces (build/slam-refact-20261015/reference/runs/<scenario>/<run>/trace.jsonl).

Predictions (written into the spec BEFORE running this script; tolerances fixed here):
  P1  every LIMITED run (except a run still open at the end) lasts >= 2.0 s - TOL
      (state 2 -> 3 needs a tracked frame, state 3 holds >= 2 s; decoded state machine).
  P2  every NORMAL run that starts after a LIMITED run lasts >= 2.0 s - TOL (state 0 -> 1 -> 2
      needs >= 2 s in state 1), except a run open at the end.
  P3  the first LIMITED frame time >= 2.0 s - TOL.
  P4  reason == UNSPECIFIED on every fresh frame (responsive mode, no VPS / estimateScale).
  P5  position == (0, 2, 0) exactly on fresh frames 0..4 (startup recenter countdown in 508).
  P6  report only: heading of the published camera forward (-Z of the GL camera) on the first
      frames, atan2(f.x, -f.z) in degrees; expected near 0 (yawOffset + identity facing).
  P7  report only: fraction of consecutive fresh frames with exactly zero translation step,
      split by status.
  P8  report only: world-point samples (worldpoints-sampled.jsonl) that are exactly
      [id 0, confidence 0, (0, 2, 0)] = zero-initialised list entries left unfilled by
      getWorldPoints (func631 allocates the unfiltered count when <= maxPoints).
TOL = 0.15 s (one 20 Hz frame + harness jitter).  Times use the fixture time `t` of the frame.

Usage: python3 -I runtime_checks.py --runs build/slam-refact-20261015/reference/runs [--out F.json]
"""
import argparse
import glob
import json
import math
import os

TOL = 0.15
MIN_HOLD = 2.0


def quat_rot(q):
    w, x, y, z = q
    return [[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
            [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
            [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]]


def heading_deg(q):
    r = quat_rot(q)
    f = (-r[0][2], -r[1][2], -r[2][2])  # world direction of camera -Z
    return math.degrees(math.atan2(f[0], -f[2])), math.degrees(math.asin(max(-1, min(1, f[1]))))


def runs_of(frames):
    out = []
    cur = None
    for fr in frames:
        if cur is None or fr['status'] != cur['status']:
            if cur is not None:
                cur['end_t'] = fr['t']
                out.append(cur)
            cur = dict(status=fr['status'], start_t=fr['t'], start_i=fr['i'], end_t=None)
    if cur is not None:
        cur['end_t'] = None  # open at end
        out.append(cur)
    return out


def check_run(path):
    frames = [json.loads(l) for l in open(path)]
    fresh = [f for f in frames if f.get('fresh') and f.get('p') is not None]
    res = dict(trace=path, n_fresh=len(fresh))
    rr = runs_of(fresh)
    lim = [r for r in rr if r['status'] == 'LIMITED']
    closed_lim = [r for r in lim if r['end_t'] is not None]
    durs = [r['end_t'] - r['start_t'] for r in closed_lim]
    res['P1_limited_runs_closed'] = len(closed_lim)
    res['P1_min_limited_run_s'] = min(durs) if durs else None
    res['P1_violations'] = [dict(start_i=r['start_i'], dur=round(r['end_t'] - r['start_t'], 3))
                            for r in closed_lim if r['end_t'] - r['start_t'] < MIN_HOLD - TOL]
    normals_after = []
    for k in range(1, len(rr)):
        if rr[k]['status'] == 'NORMAL' and rr[k - 1]['status'] == 'LIMITED' and rr[k]['end_t'] is not None:
            normals_after.append(rr[k])
    nd = [r['end_t'] - r['start_t'] for r in normals_after]
    res['P2_normal_after_limited_closed'] = len(normals_after)
    res['P2_min_normal_run_s'] = min(nd) if nd else None
    res['P2_violations'] = [dict(start_i=r['start_i'], dur=round(r['end_t'] - r['start_t'], 3))
                            for r in normals_after if r['end_t'] - r['start_t'] < MIN_HOLD - TOL]
    first_lim = next((f for f in fresh if f['status'] == 'LIMITED'), None)
    res['P3_first_limited'] = None if first_lim is None else dict(i=first_lim['i'], t=first_lim['t'])
    res['P3_ok'] = first_lim is None or first_lim['t'] >= MIN_HOLD - TOL
    reasons = sorted({f['reason'] for f in fresh})
    res['P4_reasons'] = reasons
    res['P4_ok'] = reasons == ['UNSPECIFIED']
    first5 = [f for f in fresh if f['i'] <= 4]
    res['P5_first_positions'] = [f['p'] for f in first5]
    res['P5_ok'] = all(f['p'] == [0, 2, 0] for f in first5)
    res['P6_heading_pitch_deg_first'] = [dict(i=f['i'], heading_pitch=[round(v, 3) for v in heading_deg(f['q_wxyz'])])
                                         for f in fresh[:12]]
    first_move = next((f for f in fresh if f['p'] != [0, 2, 0]), None)
    res['first_translation'] = None if first_move is None else dict(i=first_move['i'], t=first_move['t'])
    zero = {}
    for a, b in zip(fresh, fresh[1:]):
        if b['i'] != a['i'] + 1:
            continue
        st = b['status']
        z = zero.setdefault(st, [0, 0])
        z[1] += 1
        if a['p'] == b['p']:
            z[0] += 1
    res['P7_zero_translation_step_fraction'] = {k: dict(zero=v[0], total=v[1], frac=round(v[0] / v[1], 4))
                                                for k, v in zero.items()}
    wp = os.path.join(os.path.dirname(path), 'worldpoints-sampled.jsonl')
    if os.path.exists(wp):
        tot = pad = 0
        for l in open(wp):
            pts = json.loads(l)['points']
            tot += len(pts)
            pad += sum(1 for x in pts if x == [0, 0, 0, 2, 0])
        res['P8_worldpoint_padding'] = dict(points=tot, padding=pad, frac=round(pad / tot, 4) if tot else None)
    res['status_runs'] = [dict(status=r['status'], start_i=r['start_i'], start_t=round(r['start_t'], 3),
                               dur=None if r['end_t'] is None else round(r['end_t'] - r['start_t'], 3)) for r in rr]
    return res


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--runs', required=True)
    ap.add_argument('--out')
    a = ap.parse_args()
    results = []
    for path in sorted(glob.glob(os.path.join(a.runs, '*', '*', 'trace.jsonl'))):
        if '/wrap1/' in path:  # documented as not a reference trace (03 §9.2)
            continue
        r = check_run(path)
        results.append(r)
        summ = {k: r[k] for k in ('trace', 'n_fresh', 'P1_limited_runs_closed', 'P1_min_limited_run_s', 'P1_violations',
                                  'P2_normal_after_limited_closed', 'P2_min_normal_run_s', 'P2_violations',
                                  'P3_first_limited', 'P3_ok', 'P4_ok', 'P5_ok', 'first_translation',
                                  'P7_zero_translation_step_fraction', 'P8_worldpoint_padding')}
        print(json.dumps(summ))
    if a.out:
        json.dump(dict(tolerance_s=TOL, min_hold_s=MIN_HOLD, results=results), open(a.out, 'w'), indent=1)


if __name__ == '__main__':
    main()

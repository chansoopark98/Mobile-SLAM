#!/usr/bin/env python3
"""Inter-run variability of the original reference runtime reference traces in the pre-divergence window.

For each scenario with runs r1 and r2: frame-matched (same input index, both fresh) comparison of
published position (L2, responsive units) and rotation (geodesic angle, degrees) over frames
[0, first index where position difference > 0.1). Output p50/p95/max. These numbers are the
basis of the per-frame parity tolerance in spec 04 §8 (fixed before any engine output exists).
Usage: python3 -I reference_variability.py --runs build/slam-refact-20261015/reference/runs [--out F.json]
"""
import argparse
import json
import math
import os


def load(p):
    return {r['i']: r for r in map(json.loads, open(p)) if r.get('fresh') and r.get('p') is not None}


def ang(q1, q2):
    d = abs(sum(a * b for a, b in zip(q1, q2)))
    return math.degrees(2 * math.acos(min(1.0, d)))


def pct(v, p):
    if not v:
        return None
    s = sorted(v)
    k = min(len(s) - 1, max(0, int(math.ceil(p / 100 * len(s))) - 1))
    return s[k]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--runs', required=True)
    ap.add_argument('--out')
    a = ap.parse_args()
    out = {}
    for sc in sorted(os.listdir(a.runs)):
        d = os.path.join(a.runs, sc)
        if not os.path.isdir(d) or not os.path.exists(os.path.join(d, 'r2', 'trace.jsonl')):
            continue
        r1, r2 = load(os.path.join(d, 'r1', 'trace.jsonl')), load(os.path.join(d, 'r2', 'trace.jsonl'))
        common = sorted(set(r1) & set(r2))
        dp, dr, div = [], [], None
        for i in common:
            e = math.dist(r1[i]['p'], r2[i]['p'])
            if e > 0.1:
                div = i
                break
            dp.append(e)
            dr.append(ang(r1[i]['q_wxyz'], r2[i]['q_wxyz']))
        out[sc] = dict(window_frames=len(dp), first_divergent_index=div,
                       pos_p50=pct(dp, 50), pos_p95=pct(dp, 95), pos_max=max(dp) if dp else None,
                       rot_deg_p50=pct(dr, 50), rot_deg_p95=pct(dr, 95), rot_deg_max=max(dr) if dr else None)
        print(sc, json.dumps(out[sc]))
    if a.out:
        json.dump(out, open(a.out, 'w'), indent=1)


if __name__ == '__main__':
    main()

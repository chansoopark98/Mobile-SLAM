#!/usr/bin/env python3
"""US-023 pose-level REPORT (not a gate): original recordings vs the exact-input replay of the engine.
Metrics R1..R5 as registered in orient-us023-gate.v1.md section 6 (declared before the first pose-level comparison):

  R1  first frame whose published position differs from the origin (0, 2, 0)
  R2  status counts over the fresh frames
  R3  geodesic angle between the published camera rotations of two traces at the same input frame: p50 / p95 / max over
      frames 0..N-1 (N = --head, default 60) and over all common frames
  R4  Euclidean distance between the published positions at the same frame (same windows)
  R5  equivariance: trace of orientation O against the trace of orientation 0 of the same source, R_O vs R_0 * Rz(theta_O)
      (theta: rotation 90 -> 270 deg, 180 -> 180, 270 -> 90, the in-plane angle of updateCameraExtrinsics after makeRightHanded
      commutes with the z rotation), same windows

usage: orient_report.py --orig <label>=<trace.jsonl> ... --engine <label>=<trace.jsonl> ... --pair a=b ... [--equivariance O=<label of O>:<label of 0>]
Output: a JSON object on stdout. No pass/fail: the criterion applied by the report text is 'engine-vs-original p95 of R3 over
the first N frames <= 3 x the original-vs-original p95' (consistent with the original's run-to-run noise)."""
import argparse
import json
import math
import sys

import numpy as np


def load(path):
    rows = {}
    for line in open(path):
        r = json.loads(line)
        if r.get('fresh') and r.get('p') and r.get('q_wxyz'):
            rows[r['i']] = r
    return rows


def qmat(q):
    w, x, y, z = q
    n = math.sqrt(w * w + x * x + y * y + z * z)
    w, x, y, z = w / n, x / n, y / n, z / n
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def angle(ra, rb):
    c = (np.trace(ra.T @ rb) - 1) / 2
    return math.degrees(math.acos(max(-1.0, min(1.0, c))))


def rz(deg):
    c = {0: 1, 90: 0, 180: -1, 270: 0}[deg]
    s = {0: 0, 90: 1, 180: 0, 270: -1}[deg]
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]], float)


def quant(xs):
    if not xs:
        return None
    a = np.array(xs)
    return {'n': len(xs), 'p50': float(np.percentile(a, 50)), 'p95': float(np.percentile(a, 95)), 'max': float(a.max())}


def first_translation(rows):
    for i in sorted(rows):
        p = rows[i]['p']
        if abs(p[0]) > 1e-6 or abs(p[1] - 2.0) > 1e-6 or abs(p[2]) > 1e-6:
            return i
    return None


def status_counts(rows):
    out = {}
    for r in rows.values():
        out[r['status']] = out.get(r['status'], 0) + 1
    return out


def compare(a, b, head, rot_post=None):
    common = sorted(set(a) & set(b))
    rot, pos = {'head': [], 'all': []}, {'head': [], 'all': []}
    for i in common:
        ra, rb = qmat(a[i]['q_wxyz']), qmat(b[i]['q_wxyz'])
        if rot_post is not None:
            rb = rb @ rot_post
        d_rot = angle(ra, rb)
        d_pos = float(np.linalg.norm(np.array(a[i]['p']) - np.array(b[i]['p'])))
        rot['all'].append(d_rot)
        pos['all'].append(d_pos)
        if i < head:
            rot['head'].append(d_rot)
            pos['head'].append(d_pos)
    return {'common': len(common), 'rotation_deg': {k: quant(v) for k, v in rot.items()}, 'position': {k: quant(v) for k, v in pos.items()}}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--orig', action='append', default=[])
    ap.add_argument('--engine', action='append', default=[])
    ap.add_argument('--pair', action='append', default=[])
    ap.add_argument('--equivariance', action='append', default=[], help='O=<label of O>:<label of 0>')
    ap.add_argument('--head', type=int, default=60)
    a = ap.parse_args()
    traces = {}
    for kv in a.orig + a.engine:
        k, v = kv.split('=', 1)
        traces[k] = load(v)
    out = {'head': a.head, 'traces': {}, 'pairs': {}, 'equivariance': {}}
    for k, rows in traces.items():
        out['traces'][k] = {'fresh': len(rows), 'first_translation_frame': first_translation(rows), 'status': status_counts(rows)}
    for pr in a.pair:
        x, y = pr.split('=')
        out['pairs'][pr] = compare(traces[x], traces[y], a.head)
    for eq in a.equivariance:
        o, labels = eq.split('=')
        lo, l0 = labels.split(':')
        theta = {'90': 270, '180': 180, '270': 90}[o]
        out['equivariance'][eq] = compare(traces[lo], traces[l0], a.head, rot_post=rz(theta))
    json.dump(out, sys.stdout, indent=1)
    print()


if __name__ == '__main__':
    main()

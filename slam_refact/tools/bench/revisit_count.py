#!/usr/bin/env python3
"""A-0b: count revisits in an ARCore reference trajectory (report only, not a verdict).

Frozen parameters (set before any result was seen): d = 0.5 m, theta = 30 deg, T = 5 s.
A revisit pair is (i, j) with t_j - t_i > T, |p_i - p_j| < d and the angle between the camera viewing
directions < theta. Reference poses come from groundtruth-body.csv (ARCore reference, NOT independent GT);
no engine output is read. Reported: raw sample pairs, revisit events (runs of revisiting samples, merged when
less than EVENT_MERGE_S apart), and how many of each occur after the 64th keyframe.

APPROXIMATION: the engine's keyframe times are not an input here, so keyframe 64 is approximated as
t0 + 64 * KF_PERIOD_S (default 0.25 s). Output states this explicitly.
"""
import argparse
import json
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import metrics  # noqa: E402

D_M, THETA_DEG, T_S = 0.5, 30.0, 5.0
EVENT_MERGE_S = 2.0
KF_PERIOD_S = 0.25
KF_LIMIT = 64


def revisits(t, p, view, d=D_M, theta_deg=THETA_DEG, t_gap=T_S):
    """Returns list of (i, j). t sorted ascending; view = unit viewing directions (N,3)."""
    cos_min = math.cos(math.radians(theta_deg))
    pairs = []
    for j in range(len(t)):
        i_max = int(np.searchsorted(t, t[j] - t_gap, side="left"))  # t_i < t_j - T  (strict > T below)
        for i in range(i_max):
            if t[j] - t[i] > t_gap and np.linalg.norm(p[i] - p[j]) < d and float(view[i] @ view[j]) > cos_min:
                pairs.append((i, j))
    return pairs


def count(t, p, view, kf_period=KF_PERIOD_S, merge_s=EVENT_MERGE_S):
    pairs = revisits(t, p, view)
    t_kf = float(t[0]) + KF_LIMIT * kf_period
    js = sorted({j for _, j in pairs})
    events = []
    for j in js:
        if events and t[j] - events[-1][1] < merge_s:
            events[-1][1] = t[j]
        else:
            events.append([t[j], t[j]])
    return {"params": {"d_m": D_M, "theta_deg": THETA_DEG, "T_s": T_S, "event_merge_s": merge_s},
            "samples": int(len(t)), "revisit_pairs": len(pairs),
            "revisit_events": len(events),
            "pairs_after_kf64": sum(1 for _, j in pairs if t[j] >= t_kf),
            "events_after_kf64": sum(1 for a, _ in events if a >= t_kf),
            "kf64_time_s": t_kf, "kf64_basis": "APPROXIMATION: t0 + 64*%.3f s, engine keyframes not used" % kf_period}


def load(fixture_dir):
    with open(os.path.join(fixture_dir, "manifest.json"), encoding="utf-8") as f:
        man = json.load(f)
    gt = np.loadtxt(os.path.join(fixture_dir, "groundtruth-body.csv"), delimiter=",")
    gt = gt[np.argsort(gt[:, 0], kind="stable")]
    fwd_body = np.array(man["R_BFromCam"], float)[:, 2]  # camera optical axis (+Z, OpenCV) expressed in body
    view = np.array([metrics.quat_xyzw_to_matrix(r[4:8]) @ fwd_body for r in gt])
    return gt[:, 0], gt[:, 1:4], view


def main(argv=None):
    ap = argparse.ArgumentParser(description="A-0b revisit pair count (report only)")
    ap.add_argument("--fixture-root", default=os.environ.get("SLAM_REFACT_BENCH_FIXTURE_ROOT"),
                    help="directory containing slam-refact-20261015/reference/fixtures (or env SLAM_REFACT_BENCH_FIXTURE_ROOT)")
    ap.add_argument("--fixtures", nargs="+", default=["flip6-arcore-s2", "flip6-arcore-s3", "flip6-arcore-l1"])
    ap.add_argument("--kf-period-s", type=float, default=KF_PERIOD_S)
    a = ap.parse_args(argv)
    if not a.fixture_root:
        raise SystemExit("set SLAM_REFACT_BENCH_FIXTURE_ROOT or --fixture-root")
    out = {"verdict": "report-only", "gt_note": "ARCore reference pose (reference VIO), not independent GT",
           "fixtures": {}}
    for name in a.fixtures:
        t, p, view = load(os.path.join(a.fixture_root, "slam-refact-20261015/reference/fixtures", name))
        out["fixtures"][name] = count(t, p, view, kf_period=a.kf_period_s)
    print(json.dumps(out, sort_keys=True, indent=1))
    return 0


if __name__ == "__main__":
    sys.exit(main())

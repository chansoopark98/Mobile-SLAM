"""Frame-matched comparison of two published traces (same input index i, both fresh with a pose).

Used by V-PD / V-H7 / V-H7b (pre-divergence window, nearest-rank percentiles as
spec_vio_tracker/reference_variability.py that produced the S4 §8.5 numbers), MT-P7 ④ (whole run,
linear percentile as analyze_reference.repeatability that produced the RE-C §1 numbers) and V-EV3
(status agreement over frames fresh in both traces, as analyze_reference.repeatability).
"""
import math

from common import percentile, quat_angle_deg, quat_wxyz_to_matrix, rotation_angle_deg


def _posed(rows):
    return {r['i']: r for r in rows if r.get('fresh') and r.get('p') is not None}


def normalized_angle_deg(q1, q2):
    """Angle of R(q1)^T R(q2) with unit-normalised quaternions (analyze_reference.repeatability)."""
    return rotation_angle_deg(quat_wxyz_to_matrix(q1).T @ quat_wxyz_to_matrix(q2))


def matched_diffs(rows_a, rows_b, first=None, last=None, angle=quat_angle_deg):
    """Position L2 and rotation angle (deg) per input index fresh+posed in both traces. The default
    angle is reference_variability.py's 2*acos(|q1.q2|) on the stored (float32-rounded, not
    re-normalised) quaternions; MT-P7 ④ passes normalized_angle_deg."""
    a, b = _posed(rows_a), _posed(rows_b)
    idx = sorted(set(a) & set(b))
    if first is not None:
        idx = [i for i in idx if i >= first]
    if last is not None:
        idx = [i for i in idx if i <= last]
    dp = [math.dist(a[i]['p'], b[i]['p']) for i in idx]
    dr = [angle(a[i]['q_wxyz'], b[i]['q_wxyz']) for i in idx]
    return idx, dp, dr


def window_stats(rows_a, rows_b, first, last, method='nearest_rank'):
    idx, dp, dr = matched_diffs(rows_a, rows_b, first, last)
    window_len = last - first + 1
    return {'frames_compared': len(idx), 'window_frames': window_len,
            'missing_frames': sorted(set(range(first, last + 1)) - set(idx)),
            'pos_p95': percentile(dp, 95, method), 'pos_max': max(dp) if dp else None,
            'rot_deg_p95': percentile(dr, 95, method), 'rot_deg_max': max(dr) if dr else None,
            'percentile': method}


def status_agreement(rows_a, rows_b):
    b = {r['i']: r for r in rows_b}
    both = same = 0
    for r in rows_a:
        s = b.get(r['i'])
        if not s or not (r.get('fresh') and s.get('fresh')):
            continue
        both += 1
        same += r['status'] == s['status']
    return {'both_fresh': both, 'agreement': same / both if both else None}

#!/usr/bin/env python3
"""E5: gauge-free comparison of two image-target series (e5_extract.py form) and the sequence GT.

World frames and world scale of a monocular VIO are not comparable between two runs (the original differs from itself by
metres between runs), so place and scale are compared through quantities that do not depend on the world gauge:
  rho   = |p_target - p_camera| / scale          (target-centre distance in units of the target size; GT: dist / 0.8 m)
  dirc  = unit vector R_wc^T (p_target - p_camera)   (target direction in the camera frame; compared as an angle)
  relR  = R_wc^T R_target                         (target orientation in the camera frame; compared as an angle)
The camera pose is the one the same run published for the same frame (trace.jsonl p / q_wxyz).  All three are invariant under a
similarity transform of the world, so they compare what the image-target tracker adds (found/lost frames, place, scale).
Development/gate tool; numpy is not required.
"""
import json, math

TARGET_CENTRE_ROOM = (0.0, 1.1, 0.1)   # room frame, e5 scene (scene.py D_WALL, target centre Z)
TARGET_HEIGHT_M = 0.8


def _qmat(q):
    w, x, y, z = q
    n = math.sqrt(w * w + x * x + y * y + z * z) or 1.0
    w, x, y, z = w / n, x / n, y / n, z / n
    return [[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]]


def _mt(a):
    return [[a[j][i] for j in range(3)] for i in range(3)]


def _mm(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)] for i in range(3)]


def _mv(a, v):
    return [sum(a[i][k] * v[k] for k in range(3)) for i in range(3)]


def _ang_mat(a, b):
    r = _mm(_mt(a), b)
    c = (r[0][0] + r[1][1] + r[2][2] - 1) / 2
    return math.degrees(math.acos(max(-1.0, min(1.0, c))))


def _ang_vec(u, v):
    nu, nv = math.sqrt(sum(t * t for t in u)), math.sqrt(sum(t * t for t in v))
    c = sum(a * b for a, b in zip(u, v)) / (nu * nv)
    return math.degrees(math.acos(max(-1.0, min(1.0, c))))


def gauge_free(s, i, cam_q):
    """(rho, dir_cam, relR) of frame i or None. cam_q[i] = the camera quaternion (w,x,y,z) the run published for frame i."""
    if not (s['detected'][i] and s['pose'][i] and cam_q[i] and s['scale'][i]):
        return None
    d = [s['pos'][i][k] - s['pose'][i][k] for k in range(3)]
    Rc = _qmat(cam_q[i])
    rho = math.sqrt(sum(t * t for t in d)) / s['scale'][i]
    dirc = _mv(_mt(Rc), d)
    relR = _mm(_mt(Rc), _qmat(s['quat'][i]))
    return rho, dirc, relR


def _q(xs, p):
    xs = sorted(xs)
    return xs[min(len(xs) - 1, int(round((len(xs) - 1) * p)))] if xs else None


def stats(xs):
    return {'n': len(xs), 'p50': _q(xs, 0.5), 'p95': _q(xs, 0.95), 'max': max(xs) if xs else None}


def pair(a, b, a_cam_q, b_cam_q, frames=None):
    """a, b: series; *_cam_q: per-frame camera quaternions. frames: optional frame set to restrict to."""
    n = min(a['n'], b['n'])
    idx = [i for i in range(n) if a['processed'][i] and b['processed'][i] and (frames is None or i in frames)]
    dis = [i for i in idx if a['detected'][i] != b['detected'][i]]
    rho, dirs, rots = [], [], []
    for i in idx:
        ga, gb = gauge_free(a, i, a_cam_q), gauge_free(b, i, b_cam_q)
        if ga is None or gb is None:
            continue
        rho.append(abs(gb[0] / ga[0] - 1))
        dirs.append(_ang_vec(ga[1], gb[1]))
        rots.append(_ang_mat(ga[2], gb[2]))
    return {'frames_compared': len(idx), 'state_disagree': len(dis), 'state_disagree_frames': dis,
            'rho_rel': stats(rho), 'dir_deg': stats(dirs), 'relrot_deg': stats(rots)}


def vs_gt(s, cam_q, gt_frames):
    out = []
    for i, g in enumerate(gt_frames[:s['n']]):
        r = gauge_free(s, i, cam_q)
        if r is None:
            continue
        rho_gt = math.dist(TARGET_CENTRE_ROOM, g['p']) / TARGET_HEIGHT_M
        out.append(abs(r[0] / rho_gt - 1))
    return stats(out)


def gt_vis(gt_json_path):
    return [f['target_visible_fraction'] for f in json.load(open(gt_json_path))['frames']]


def gt_runs(vis, min_out=8, min_in=15):
    """[(kind, first, last)] with kind 'out' (fraction 0 for >= min_out frames) or 'in' (fraction >= 0.95 for >= min_in frames)."""
    def kind(v):
        return 'out' if v == 0 else ('in' if v >= 0.95 else 'mid')
    runs, i, n = [], 0, len(vis)
    while i < n:
        k = kind(vis[i]); j = i
        while j + 1 < n and kind(vis[j + 1]) == k:
            j += 1
        if (k == 'out' and j - i + 1 >= min_out) or (k == 'in' and j - i + 1 >= min_in):
            runs.append((k, i, j))
        i = j + 1
    return runs

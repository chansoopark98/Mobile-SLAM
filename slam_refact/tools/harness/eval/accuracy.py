"""TUM GT accuracy of a published trace (A-1..A-3, US-011 columns), numpy only.

Same definitions and frame conversions as slam_refact/tools/re/runtime/analyze_reference.py (lane C),
re-implemented without scipy so the harness depends on numpy only; tests/test_eval.py checks that the
numbers equal the lane C analysis.json of the original reference runs.

Frames (03-runtime-reference.md §6.2):
  published pose (p, q_wxyz) = world_from_camera, right-handed y-up world, OpenGL camera (-Z forward).
  OpenCV camera:  R_W_cv = R(q) * diag(1, -1, -1).
  Phone body B:   R_B_cv = R_PHONE = [[1,0,0],[0,0,1],[0,-1,0]], camera origin t_ic in B (fixture manifest).
  Body pose:      R_W_B = R_W_cv * R_PHONE^T,  p_B = p_c - R_W_B * t_ic.
Association: GT row nearest in time to the frame's fixture time t, accepted when |dt| <= 6 ms (1:1, no
interpolation). Alignment: SE(3) Umeyama with scale fixed to 1 (primary) and Sim(3) Umeyama (scale
diagnostic only; responsive units are not metres). Rotation APE uses a rotation-only world alignment
(chordal mean). RPE over 1 s (next frame at >= t+1 s, skipped when > 1.1 s).
"""
import numpy as np

from common import quat_wxyz_to_matrix, rotation_angle_deg, summary

R_PHONE = np.array([[1, 0, 0], [0, 0, 1], [0, -1, 0]], float)
D_GL_CV = np.diag([1.0, -1.0, -1.0])
MAX_DT = 0.006


def load_gt(path):
    """groundtruth-body.csv: t, px, py, pz, qx, qy, qz, qw (body B in mocap world)."""
    gt = np.loadtxt(path, delimiter=',')
    return gt


def gt_rot(row):
    qx, qy, qz, qw = row[4:8]
    return quat_wxyz_to_matrix((qw, qx, qy, qz))


def cam_cv(row):
    return quat_wxyz_to_matrix(row['q_wxyz']) @ D_GL_CV, np.asarray(row['p'], float)


def body_pose(row, t_ic):
    rc, pc = cam_cv(row)
    rb = rc @ R_PHONE.T
    return rb, pc - rb @ t_ic


def gt_lookup(gt, t):
    i = int(np.clip(np.searchsorted(gt[:, 0], t), 1, len(gt) - 1))
    j = i if abs(gt[i, 0] - t) < abs(gt[i - 1, 0] - t) else i - 1
    if abs(gt[j, 0] - t) > MAX_DT:
        return None
    return gt_rot(gt[j]), gt[j, 1:4]


def umeyama(src, dst, with_scale):
    mu_s, mu_d = src.mean(0), dst.mean(0)
    s0, d0 = src - mu_s, dst - mu_d
    cov = d0.T @ s0 / len(src)
    u, sv, vt = np.linalg.svd(cov)
    s = np.eye(3)
    if np.linalg.det(u) * np.linalg.det(vt) < 0:
        s[2, 2] = -1
    rw = u @ s @ vt
    c = np.trace(np.diag(sv) @ s) / ((s0 ** 2).sum() / len(src)) if with_scale else 1.0
    t = mu_d - c * rw @ mu_s
    return rw, t, c


def best_gt_frame_lag(rows, gt):
    """Fixed-convention lag diagnostic of analyze_reference.convention_checks (no fitted parameters)."""
    by_i = {r['i']: r for r in rows}
    step, lag = 5, {}
    for k in range(-3, 4):
        errs = []
        for r in rows:
            a, b = r, by_i.get(r['i'] + step)
            if not b or not a.get('q_wxyz') or not b.get('q_wxyz'):
                continue
            ga, gb = by_i.get(a['i'] + k), by_i.get(b['i'] + k)
            if not ga or not gb:
                continue
            g_a, g_b = gt_lookup(gt, ga['t']), gt_lookup(gt, gb['t'])
            if g_a is None or g_b is None:
                continue
            rel8 = cam_cv(a)[0].T @ cam_cv(b)[0]
            relg = (g_a[0] @ R_PHONE).T @ (g_b[0] @ R_PHONE)
            if rotation_angle_deg(relg) < 1.0:
                continue
            errs.append(rotation_angle_deg(rel8.T @ relg))
        lag[str(k)] = summary(errs)
    valid = [(v['p50'], int(k)) for k, v in lag.items() if v]
    return (min(valid)[1] if valid else None), lag


def accuracy(rows, gt, t_ic, lag_frames=0):
    t_ic = np.asarray(t_ic, float)
    by_i = {r['i']: r for r in rows}
    e, g, er, gr, ts = [], [], [], [], []
    for r in rows:
        if not r.get('p'):
            continue
        ref = by_i.get(r['i'] + lag_frames)
        if not ref:
            continue
        hit = gt_lookup(gt, ref['t'])
        if hit is None:
            continue
        rb, pb = body_pose(r, t_ic)
        e.append(pb)
        er.append(rb)
        g.append(hit[1])
        gr.append(hit[0])
        ts.append(r['t'])
    e, g, ts = np.asarray(e), np.asarray(g), np.asarray(ts)
    er, gr = np.asarray(er), np.asarray(gr)
    out = {'associatedPoses': int(len(e)), 'coverage': len(e) / len(rows) if rows else 0.0, 'gtFrameLag': lag_frames}
    if len(e) < 10:
        return out
    for name, with_scale in (('se3', False), ('sim3', True)):
        rw, tw, c = umeyama(e, g, with_scale)
        aligned = (c * (rw @ e.T)).T + tw
        out[name] = {'scale': float(c), 'ateTranslation': summary(np.linalg.norm(aligned - g, axis=1))}
    u, _, vt = np.linalg.svd(np.einsum('nij,nkj->ik', gr, er))
    rw = u @ np.diag([1, 1, np.linalg.det(u @ vt)]) @ vt
    out['apeRotationDegRotationOnlyAlignment'] = summary([rotation_angle_deg(gg.T @ rw @ ee) for ee, gg in zip(er, gr)])
    c = out['sim3']['scale']
    rpe_t_se3, rpe_t_sim3, rpe_r = [], [], []
    j = 0
    for i in range(len(ts)):
        while j < len(ts) and ts[j] < ts[i] + 1.0:
            j += 1
        if j >= len(ts) or ts[j] - ts[i] > 1.1:
            continue
        de = er[i].T @ (e[j] - e[i])
        dg = gr[i].T @ (g[j] - g[i])
        rpe_t_se3.append(np.linalg.norm(de - dg))
        rpe_t_sim3.append(np.linalg.norm(c * de - dg))
        rpe_r.append(rotation_angle_deg((gr[i].T @ gr[j]).T @ (er[i].T @ er[j])))
    out['rpe1s'] = {'translationSe3': summary(rpe_t_se3), 'translationSim3Scaled': summary(rpe_t_sim3),
                    'rotationDeg': summary(rpe_r)}
    out['timeSpanS'] = [float(ts[0]), float(ts[-1])]
    return out

#!/usr/bin/env python3
"""Summarize headless reference runtime reference runs (lane C, US-004).

For every run directory of a scenario: per-run summary, TUM-format trajectory exports in a documented frame,
convention checks against GT (gravity direction in camera frame, rotation increments vs frame lag), accuracy
(SE3 scale=1, Sim3 diagnostic, rotation APE, RPE 1 s, coverage) and run-to-run repeatability.

Frame conventions (see slam_refact/docs/re/03-runtime-reference.md):
  reference runtime public pose (p, q_wxyz) = world_from_camera in a right-handed, y-up world (three.js style); the
  camera looks along its local -Z with +Y up (OpenGL camera).  OpenCV camera (x right, y down, z forward):
  R_W_cv = R(q) * diag(1, -1, -1).  Phone body B: R_B_cv = R_PHONE = [[1,0,0],[0,0,1],[0,-1,0]], camera origin at
  t_c_in_B (fixture manifest t_ic).  Body pose: R_W_B = R_W_cv * R_PHONE^T, p_B = p_c - R_W_B * t_c_in_B.
Positions are in reference runtime responsive units (initial camera height = 2, not metres).

Usage: analyze_reference.py --scenario-dir build/slam-refact-20261015/reference/runs/<scenario> [--gt <csv> --t-ic x y z]
"""
import argparse
import json
import pathlib

import numpy as np
from scipy.spatial.transform import Rotation as Rot

R_PHONE = np.array([[1, 0, 0], [0, 0, 1], [0, -1, 0]], float)
D_GL_CV = np.diag([1.0, -1.0, -1.0])
MAX_DT = 0.006


def load_trace(run):
    return [json.loads(line) for line in open(run / 'trace.jsonl')]


def cam_cv(row):
    w, x, y, z = row['q_wxyz']
    return Rot.from_quat([x, y, z, w]).as_matrix() @ D_GL_CV, np.asarray(row['p'], float)


def body_pose(row, t_ic):
    Rc, pc = cam_cv(row)
    Rb = Rc @ R_PHONE.T
    return Rb, pc - Rb @ t_ic


def stats(xs):
    xs = np.asarray([x for x in xs if x is not None and np.isfinite(x)], float)
    if not len(xs):
        return None
    return {'n': int(len(xs)), 'mean': float(xs.mean()), 'rmse': float(np.sqrt(np.mean(xs ** 2))), 'p50': float(np.median(xs)),
            'p95': float(np.percentile(xs, 95)), 'max': float(xs.max())}


def umeyama(src, dst, with_scale):
    mu_s, mu_d = src.mean(0), dst.mean(0)
    s0, d0 = src - mu_s, dst - mu_d
    cov = d0.T @ s0 / len(src)
    u, sv, vt = np.linalg.svd(cov)
    S = np.eye(3)
    if np.linalg.det(u) * np.linalg.det(vt) < 0:
        S[2, 2] = -1
    Rw = u @ S @ vt
    c = np.trace(np.diag(sv) @ S) / ((s0 ** 2).sum() / len(src)) if with_scale else 1.0
    t = mu_d - c * Rw @ mu_s
    return Rw, t, c


def gt_lookup(gt, t):
    i = int(np.clip(np.searchsorted(gt[:, 0], t), 1, len(gt) - 1))
    j = i if abs(gt[i, 0] - t) < abs(gt[i - 1, 0] - t) else i - 1
    if abs(gt[j, 0] - t) > MAX_DT:
        return None
    return Rot.from_quat(gt[j, 4:8]).as_matrix(), gt[j, 1:4]


def convention_checks(rows, gt):
    """Fixed-convention checks (no fitted parameters)."""
    g_err = []
    for r in rows:
        if not r['q_wxyz']:
            continue
        g = gt_lookup(gt, r['t'])
        if g is None:
            continue
        Rc, _ = cam_cv(r)
        g8 = Rc.T @ np.array([0, -1.0, 0])                 # reference runtime world down (-Y) in camera
        gg = (g[0] @ R_PHONE).T @ np.array([0, 0, -1.0])   # mocap world down (-Z) in camera
        g_err.append(np.degrees(np.arccos(np.clip(g8 @ gg, -1, 1))))
    lag = {}
    by_i = {r['i']: r for r in rows}
    step = 5
    for k in range(-3, 4):
        errs = []
        for r in rows:
            a, b = r, by_i.get(r['i'] + step)
            if not b or not a['q_wxyz'] or not b['q_wxyz']:
                continue
            ga, gb = by_i.get(a['i'] + k), by_i.get(b['i'] + k)
            if not ga or not gb:
                continue
            Ga, Gb = gt_lookup(gt, ga['t']), gt_lookup(gt, gb['t'])
            if Ga is None or Gb is None:
                continue
            rel8 = cam_cv(a)[0].T @ cam_cv(b)[0]
            relg = (Ga[0] @ R_PHONE).T @ (Gb[0] @ R_PHONE)
            if np.degrees(Rot.from_matrix(relg).magnitude()) < 1.0:
                continue
            errs.append(np.degrees(Rot.from_matrix(rel8.T @ relg).magnitude()))
        lag[str(k)] = stats(errs)
    best = min((v['p50'], int(k)) for k, v in lag.items() if v)[1] if any(lag.values()) else None
    return {'gravityInCameraDeg': stats(g_err),
            'rotationIncrementErrorDegByGtFrameLag': lag,
            'bestGtFrameLag': best,
            'definition': 'GT frame lag k: reference runtime pose published for input frame i is compared with GT at input frame i+k; '
                          'increments over 5 frames with GT rotation >= 1 deg; no fitted parameters'}


def accuracy(rows, gt, t_ic, lag_frames, duration):
    by_i = {r['i']: r for r in rows}
    E, G, ER, GR, T = [], [], [], [], []
    for r in rows:
        if not r['p']:
            continue
        ref = by_i.get(r['i'] + lag_frames)
        if not ref:
            continue
        g = gt_lookup(gt, ref['t'])
        if g is None:
            continue
        Rb, pb = body_pose(r, t_ic)
        E.append(pb); ER.append(Rb); G.append(g[1]); GR.append(g[0]); T.append(r['t'])
    E, G, T = np.asarray(E), np.asarray(G), np.asarray(T)
    ER, GR = np.asarray(ER), np.asarray(GR)
    out = {'associatedPoses': int(len(E)), 'coverage': len(E) / len(rows) if rows else 0.0, 'gtFrameLag': lag_frames}
    if len(E) < 10:
        return out
    for name, scale in (('se3', False), ('sim3', True)):
        Rw, tw, c = umeyama(E, G, scale)
        aligned = (c * (Rw @ E.T)).T + tw
        err_t = np.linalg.norm(aligned - G, axis=1)
        out[name] = {'scale': float(c), 'ateTranslation': stats(err_t)}
    # Rotation APE with a rotation-only world alignment (chordal mean of R_G * R_E^T); position-independent.
    u, _, vt = np.linalg.svd(np.einsum('nij,nkj->ik', GR, ER))
    Rw = u @ np.diag([1, 1, np.linalg.det(u @ vt)]) @ vt
    err_r = [np.degrees(Rot.from_matrix(g.T @ Rw @ e).magnitude()) for e, g in zip(ER, GR)]
    out['apeRotationDegRotationOnlyAlignment'] = stats(err_r)
    # RPE over 1 s (translation in engine units * Sim3 scale, and raw SE3; rotation in degrees)
    c = out['sim3']['scale']
    rpe_t_se3, rpe_t_sim3, rpe_r = [], [], []
    j = 0
    for i in range(len(T)):
        while j < len(T) and T[j] < T[i] + 1.0:
            j += 1
        if j >= len(T) or T[j] - T[i] > 1.1:
            continue
        dE = ER[i].T @ (E[j] - E[i]); dG = GR[i].T @ (G[j] - G[i])
        rpe_t_se3.append(np.linalg.norm(dE - dG)); rpe_t_sim3.append(np.linalg.norm(c * dE - dG))
        rpe_r.append(np.degrees(Rot.from_matrix((GR[i].T @ GR[j]).T @ (ER[i].T @ ER[j])).magnitude()))
    out['rpe1s'] = {'translationSe3': stats(rpe_t_se3), 'translationSim3Scaled': stats(rpe_t_sim3), 'rotationDeg': stats(rpe_r)}
    out['timeSpanS'] = [float(T[0]), float(T[-1])]
    out['sequenceDurationS'] = duration
    return out


def write_tum(rows, path, t_ic, frame):
    with open(path, 'w') as f:
        f.write(f'# t_fixture_s x y z qx qy qz qw ; frame={frame}; reference runtime world (y-up, responsive units)\n')
        for r in rows:
            if not r['p']:
                continue
            if frame == 'body':
                Rm, p = body_pose(r, t_ic)
            else:
                Rm, p = cam_cv(r)
            q = Rot.from_matrix(Rm).as_quat()
            f.write(f"{r['t']:.9f} {p[0]:.7f} {p[1]:.7f} {p[2]:.7f} {q[0]:.7f} {q[1]:.7f} {q[2]:.7f} {q[3]:.7f}\n")


def repeatability(a_rows, b_rows):
    bi = {r['i']: r for r in b_rows}
    dp, dr, same_status, both_fresh, fresh_xor = [], [], 0, 0, 0
    first = {}
    for r in a_rows:
        s = bi.get(r['i'])
        if not (s and r['p'] and s['p']):
            continue
        d = np.linalg.norm(np.subtract(r['p'], s['p']))
        ang = np.degrees(Rot.from_matrix(cam_cv(r)[0].T @ cam_cv(s)[0]).magnitude())
        for key, hit in (('anyPoseDifference', d > 0 or ang > 0), ('position>1e-3', d > 1e-3), ('position>0.1', d > 0.1),
                         ('rotation>0.5deg', ang > 0.5), ('status', r['status'] != s['status'])):
            if hit and key not in first:
                first[key] = {'frame': r['i'], 't': r['t']}
    for r in a_rows:
        s = bi.get(r['i'])
        if not s:
            continue
        if r['fresh'] != s['fresh']:
            fresh_xor += 1
        if not (r['fresh'] and s['fresh']):
            continue
        both_fresh += 1
        same_status += r['status'] == s['status']
        if r['p'] and s['p']:
            dp.append(np.linalg.norm(np.subtract(r['p'], s['p'])))
            Ra, Rb = cam_cv(r)[0], cam_cv(s)[0]
            dr.append(np.degrees(Rot.from_matrix(Ra.T @ Rb).magnitude()))
    return {'bothFresh': both_fresh, 'freshMismatch': fresh_xor, 'statusAgreement': same_status / both_fresh if both_fresh else None,
            'positionDiffUnits': stats(dp), 'rotationDiffDeg': stats(dr), 'firstDivergence': first}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--scenario-dir', required=True, type=pathlib.Path)
    ap.add_argument('--gt', type=pathlib.Path)
    ap.add_argument('--t-ic', type=float, nargs=3, default=[0.0, 0.0, 0.0], help='camera origin in body B (fixture t_ic)')
    ap.add_argument('--runs', nargs='*', help='run labels (default: all with meta.json and status COMPLETED*)')
    ap.add_argument('--output', type=pathlib.Path)
    a = ap.parse_args()
    t_ic = np.asarray(a.t_ic)
    gt = np.loadtxt(a.gt, delimiter=',') if a.gt else None
    runs = []
    for d in sorted(p for p in a.scenario_dir.iterdir() if (p / 'meta.json').exists()):
        if a.runs and d.name not in a.runs:
            continue
        meta = json.loads((d / 'meta.json').read_text())
        if not str(meta.get('status', '')).startswith('COMPLETED'):
            continue
        runs.append((d, meta, load_trace(d)))
    result = {'scenarioDir': str(a.scenario_dir), 'gt': str(a.gt) if a.gt else None, 'tIcBody': t_ic.tolist(), 'runs': {}, 'repeatability': {}}
    for d, meta, rows in runs:
        s = meta['summary']
        info = {k: s[k] for k in ('inputFrames', 'freshCallbacks', 'statusCounts', 'normalRatio', 'limitedRatio', 'finitePoseRatio', 'timeToFirstPose',
                                  'timeToFirstNormal', 'timeToFirstWorldPoints', 'publishToCallbackMs', 'xrCpuStageMs', 'xrGpuStageMs', 'worldPoints', 'publishLagMs')}
        info['flags'] = {'profile': '--profile' in meta['argv'], 'wrap': '--wrap' in meta['argv']}
        info['loadAvg'] = [meta.get('loadAvgAtStart'), meta.get('loadAvgAtEnd')]
        info['wallSeconds'] = meta.get('wallSeconds')
        write_tum(rows, d / 'trajectory-camera-cv.tum', t_ic, 'camera-opencv')
        write_tum(rows, d / 'trajectory-body.tum', t_ic, 'body')
        if gt is not None:
            conv = convention_checks(rows, gt)
            info['conventionChecks'] = conv
            duration = rows[-1]['t'] - rows[0]['t']
            info['accuracyLag0'] = accuracy(rows, gt, t_ic, 0, duration)
            if conv['bestGtFrameLag'] not in (None, 0):
                info['accuracyBestLagDiagnostic'] = accuracy(rows, gt, t_ic, conv['bestGtFrameLag'], duration)
        result['runs'][d.name] = info
    plain = [(d, rows) for d, meta, rows in runs if '--profile' not in meta['argv'] and '--wrap' not in meta['argv']]
    for i in range(len(plain)):
        for j in range(i + 1, len(plain)):
            result['repeatability'][f'{plain[i][0].name}~{plain[j][0].name}'] = repeatability(plain[i][1], plain[j][1])
    text = json.dumps(result, indent=1)
    (a.output or a.scenario_dir / 'analysis.json').write_text(text + '\n')
    print(text)


if __name__ == '__main__':
    main()

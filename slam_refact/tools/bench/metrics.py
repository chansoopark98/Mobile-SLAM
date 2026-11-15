"""xr-bench scoring primitives (report-only). numpy only.

Evidence rules (CLAUDE.md section 3): body-frame poses, 1:1 timestamp association, SE(3) with scale fixed
to 1 is the primary alignment, Sim(3) scale is a separate diagnostic, coverage is reported against the
expected frame count, and zero poses / zero associations make the arm INVALID (never a pass).
There are no pass thresholds anywhere in this module.
"""
import math

import numpy as np

MIN_ASSOCIATIONS = 3  # below this the alignment is degenerate (a validity condition, not a pass threshold)


def rotation_angle_rad(r):
    """Angle of rotation matrix r; atan2 form stays accurate near 0."""
    vee = np.array([r[2, 1] - r[1, 2], r[0, 2] - r[2, 0], r[1, 0] - r[0, 1]]) * 0.5
    return math.atan2(float(np.linalg.norm(vee)), float((np.trace(r) - 1.0) * 0.5))


def quat_xyzw_to_matrix(q):
    x, y, z, w = q / np.linalg.norm(q)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def umeyama(src, dst, with_scale):
    """dst ~= s * R @ src + t (least squares). Returns (s, R, t)."""
    mu_s, mu_d = src.mean(0), dst.mean(0)
    s0, d0 = src - mu_s, dst - mu_d
    cov = d0.T @ s0 / len(src)
    u, sv, vt = np.linalg.svd(cov)
    sgn = np.eye(3)
    if np.linalg.det(u) * np.linalg.det(vt) < 0:
        sgn[2, 2] = -1.0
    rot = u @ sgn @ vt
    var_s = (s0 ** 2).sum() / len(src)
    scale = float(np.trace(np.diag(sv) @ sgn) / var_s) if with_scale and var_s > 0 else 1.0
    return scale, rot, mu_d - scale * rot @ mu_s


def associate(est_t, gt_t, max_dt):
    """1:1 nearest-in-time association. Returns (est_idx, gt_idx) arrays; each GT row is used at most once
    (the estimate with the smaller |dt| keeps it; ties go to the earlier estimate). Deterministic."""
    est_t, gt_t = np.asarray(est_t, float), np.asarray(gt_t, float)
    if len(est_t) == 0 or len(gt_t) == 0:
        return np.zeros(0, int), np.zeros(0, int)
    pos = np.clip(np.searchsorted(gt_t, est_t), 1, len(gt_t) - 1) if len(gt_t) > 1 else np.zeros(len(est_t), int)
    best = {}
    for ei in range(len(est_t)):
        cands = [pos[ei]] if len(gt_t) == 1 else [pos[ei] - 1, pos[ei]]
        gi = min(cands, key=lambda g: (abs(gt_t[g] - est_t[ei]), g))
        dt = abs(gt_t[gi] - est_t[ei])
        if dt > max_dt:
            continue
        if gi not in best or dt < best[gi][0]:
            best[gi] = (dt, ei)
    pairs = sorted((ei, gi) for gi, (_, ei) in best.items())
    return np.array([p[0] for p in pairs], int), np.array([p[1] for p in pairs], int)


def _rmse(v):
    v = np.asarray(v, float)
    return float(np.sqrt(np.mean(v ** 2))) if len(v) else None


def _rpe(t, r_est, p_est, r_gt, p_gt, rpe_dt, rpe_tol):
    pos_err, rot_err = [], []
    for i in range(len(t)):
        j = int(np.searchsorted(t, t[i] + rpe_dt))
        cands = [k for k in (j - 1, j) if 0 <= k < len(t) and k > i and abs(t[k] - t[i] - rpe_dt) <= rpe_tol]
        if not cands:
            continue
        k = min(cands, key=lambda c: (abs(t[c] - t[i] - rpe_dt), c))
        de = r_est[i].T @ (p_est[k] - p_est[i])
        dg = r_gt[i].T @ (p_gt[k] - p_gt[i])
        pos_err.append(np.linalg.norm(de - dg))
        rot_err.append(rotation_angle_rad((r_gt[i].T @ r_gt[k]).T @ (r_est[i].T @ r_est[k])))
    return {"pairs": len(pos_err), "pos_rmse_m": _rmse(pos_err),
            "rot_rmse_deg": None if not rot_err else math.degrees(_rmse(rot_err))}


def score_trajectory(est, gt, expected_frames, max_dt=0.006, rpe_dt=1.0, rpe_tol=0.1):
    """est/gt: dicts with t (N,), R (N,3,3) world_from_body, p (N,3). `est` must already contain ONLY accepted
    (fresh, non-LIMITED, non-predicted) poses; `expected_frames` is the full frame count (coverage denominator).
    Returns a plain-JSON dict with `valid` and `invalid_reason`; metrics are None when invalid."""
    out = {"expected_frames": int(expected_frames), "accepted_poses": int(len(est["t"])),
           "alignment_primary": "SE(3) scale=1, fit over all associated poses",
           "association": {"rule": "1:1 nearest time", "max_dt_s": max_dt}}
    out.update({"associated": 0, "coverage": 0.0})
    t = np.asarray(est["t"], float)
    ok = np.array([np.all(np.isfinite(est["R"][i])) and np.all(np.isfinite(est["p"][i])) and
                   abs(np.linalg.det(est["R"][i]) - 1.0) < 1e-3 for i in range(len(t))], bool)
    out["non_finite_or_non_rotation_poses"] = int((~ok).sum())
    order = np.argsort(t, kind="stable")
    order = order[ok[order]]
    e = {k: np.asarray(est[k])[order] for k in ("t", "R", "p")}
    if len(e["t"]) == 0:
        return _invalid(out, "zero_poses")
    ei, gi = associate(e["t"], gt["t"], max_dt)
    out["associated"] = int(len(ei))
    out["coverage"] = len(ei) / expected_frames if expected_frames else 0.0
    if len(ei) == 0:
        return _invalid(out, "zero_associations")
    if len(ei) < MIN_ASSOCIATIONS:
        return _invalid(out, "too_few_associations")
    tt, re_, pe = e["t"][ei], e["R"][ei], e["p"][ei]
    rg, pg = np.asarray(gt["R"])[gi], np.asarray(gt["p"])[gi]
    if np.allclose(pe, pe[0]):
        return _invalid(out, "zero_motion_pose")  # every accepted position identical: no trajectory
    _, rot, tr = umeyama(pe, pg, with_scale=False)
    pos_err = np.linalg.norm((pe @ rot.T + tr) - pg, axis=1)
    # rotation-only world alignment: chordal mean of R_gt R_est^T
    m = sum(rg[i] @ re_[i].T for i in range(len(tt)))
    u, _, vt = np.linalg.svd(m)
    sg = np.eye(3)
    if np.linalg.det(u) * np.linalg.det(vt) < 0:
        sg[2, 2] = -1.0
    r_al = u @ sg @ vt
    rot_err = [rotation_angle_rad(rg[i].T @ r_al @ re_[i]) for i in range(len(tt))]
    s_sim, rot_s, tr_s = umeyama(pe, pg, with_scale=True)
    sim_err = np.linalg.norm((s_sim * pe @ rot_s.T + tr_s) - pg, axis=1)
    out.update({
        "valid": True, "invalid_reason": None,
        "ate_pos_rmse_m": _rmse(pos_err),
        "ate_rot_rmse_deg": math.degrees(_rmse(rot_err)),
        "rpe_1s": _rpe(tt, re_, pe, rg, pg, rpe_dt, rpe_tol),
        "sim3_diagnostic": {"ate_pos_rmse_m": _rmse(sim_err),
                            "scale_est_over_gt": (1.0 / s_sim) if s_sim > 0 else None,
                            "note": "diagnostic only; not used in any comparison"},
    })
    return out


def _invalid(out, reason):
    out.update({"valid": False, "invalid_reason": reason, "ate_pos_rmse_m": None, "ate_rot_rmse_deg": None,
                "rpe_1s": None, "sim3_diagnostic": None})
    return out

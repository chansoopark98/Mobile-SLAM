#!/usr/bin/env python3
"""Strict pose evaluation for audit replays. SE(3) preserves metric scale."""

import argparse
import json
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation


def validate_poses(poses):
    poses = np.asarray(poses, dtype=float)
    if poses.ndim != 2 or poses.shape[1] != 8 or len(poses) < 3:
        raise ValueError("Need at least three poses: t x y z qx qy qz qw")
    if not np.isfinite(poses).all():
        raise ValueError("Non-finite pose")
    if not np.all(np.diff(poses[:, 0]) > 0):
        raise ValueError("Timestamps must be strictly increasing")
    norms = np.linalg.norm(poses[:, 4:8], axis=1)
    if np.any(np.abs(norms - 1) > 0.01):
        raise ValueError("Quaternion norm differs from one by more than 0.01")
    poses = poses.copy()
    poses[:, 4:8] /= norms[:, None]
    return poses


def load_poses(path, fmt="tum"):
    rows = []
    for line_no, line in enumerate(Path(path).read_text().splitlines(), 1):
        line = line.strip()
        if not line or line.startswith("#"):
            continue
        fields = line.replace(",", " ").split()
        try:
            if (fmt == "tum" and len(fields) != 8) or len(fields) < 8:
                raise ValueError("wrong column count")
            values = [float(x) for x in fields[:8]]
        except ValueError as exc:
            raise ValueError(f"{path}:{line_no}: {exc}") from exc
        if fmt == "euroc":  # ns, xyz, qw qx qy qz (TUM-VI/EuRoC CSV)
            values = [values[0] * 1e-9, *values[1:4], *values[5:8], values[4]]
        rows.append(values)
    return validate_poses(rows)


def camera_to_body(poses, config):
    import yaml

    class Loader(yaml.SafeLoader):
        pass

    Loader.add_constructor("tag:yaml.org,2002:opencv-matrix",
                           lambda loader, node: loader.construct_mapping(node, deep=True))
    text = Path(config).read_text()
    if text.startswith("%YAML:1.0"):
        text = "\n".join(text.splitlines()[1:])
    cfg = yaml.load(text, Loader=Loader)
    r_ic = np.asarray(cfg["extrinsicRotation"]["data"], dtype=float).reshape(3, 3)
    t_ic = np.asarray(cfg["extrinsicTranslation"]["data"], dtype=float).reshape(3)
    if not np.allclose(r_ic.T @ r_ic, np.eye(3), atol=1e-5) or not np.isclose(np.linalg.det(r_ic), 1, atol=1e-5):
        raise ValueError("Invalid camera-to-IMU rotation")
    result = poses.copy()
    r_wc = Rotation.from_quat(poses[:, 4:8]).as_matrix()
    r_wi = r_wc @ r_ic.T
    result[:, 1:4] -= np.einsum("nij,j->ni", r_wi, t_ic)
    result[:, 4:8] = Rotation.from_matrix(r_wi).as_quat()
    return validate_poses(result)


def associate(est, gt, max_dt):
    if not np.isfinite(max_dt) or max_dt < 0:
        raise ValueError("max_dt must be finite and nonnegative")
    candidates = []
    for i, t in enumerate(est[:, 0]):
        j = np.searchsorted(gt[:, 0], t)
        for k in (j - 1, j):
            if 0 <= k < len(gt) and abs(gt[k, 0] - t) <= max_dt:
                candidates.append((abs(gt[k, 0] - t), i, k))
    used_est, used_gt, pairs = set(), set(), []
    for _, i, j in sorted(candidates):
        if i not in used_est and j not in used_gt:
            pairs.append((i, j))
            used_est.add(i)
            used_gt.add(j)
    return sorted(pairs)


def align_positions(src, dst, with_scale=False):
    src_mean, dst_mean = src.mean(axis=0), dst.mean(axis=0)
    x, y = src - src_mean, dst - dst_mean
    u, d, vt = np.linalg.svd(y.T @ x / len(src))
    s = np.eye(3)
    s[-1, -1] = np.sign(np.linalg.det(u @ vt))
    r = u @ s @ vt
    variance = np.mean(np.sum(x * x, axis=1))
    scale = float(np.sum(d * np.diag(s)) / variance) if with_scale and variance > 1e-12 else 1.0
    translation = dst_mean - scale * (r @ src_mean)
    return scale * (src @ r.T) + translation, r, scale


def stats(errors):
    return {"rmse": float(np.sqrt(np.mean(errors ** 2))),
            "median": float(np.median(errors)), "p95": float(np.percentile(errors, 95)),
            "max": float(np.max(errors))}


def evaluate(est, gt, expected_frames=None, max_dt=0.01, delta=1.0, rpe_tolerance=0.05):
    est, gt = validate_poses(est), validate_poses(gt)
    if not np.isfinite(delta) or delta <= 0 or not np.isfinite(rpe_tolerance) or rpe_tolerance < 0:
        raise ValueError("Invalid RPE interval/tolerance")
    if expected_frames is not None and expected_frames < len(est):
        raise ValueError("Expected frame count cannot be smaller than pose count")
    pairs = associate(est, gt, max_dt)
    if len(pairs) < 3:
        raise ValueError("Fewer than three unique GT associations")
    est_idx, gt_idx = np.asarray(pairs).T
    e, g = est[est_idx], gt[gt_idx]
    aligned, r_align, _ = align_positions(e[:, 1:4], g[:, 1:4])
    sim3, _, sim3_scale = align_positions(e[:, 1:4], g[:, 1:4], with_scale=True)
    covariance_rank = int(np.linalg.matrix_rank(
        (g[:, 1:4] - g[:, 1:4].mean(axis=0)).T @ (e[:, 1:4] - e[:, 1:4].mean(axis=0))))
    re = Rotation.from_quat(e[:, 4:8]).as_matrix()
    rg = Rotation.from_quat(g[:, 4:8]).as_matrix()
    orientation_error = Rotation.from_matrix(np.transpose(rg, (0, 2, 1)) @ r_align @ re).magnitude()
    trans_errors, rot_errors = [], []
    for i, t in enumerate(e[:, 0]):
        insertion = np.searchsorted(e[:, 0], t + delta)
        choices = [j for j in (insertion - 1, insertion) if i < j < len(e)]
        if not choices:
            continue
        j = min(choices, key=lambda k: abs(e[k, 0] - t - delta))
        if abs(e[j, 0] - t - delta) > rpe_tolerance:
            continue
        r_est = re[i].T @ re[j]
        r_gt = rg[i].T @ rg[j]
        t_est = re[i].T @ (e[j, 1:4] - e[i, 1:4])
        t_gt = rg[i].T @ (g[j, 1:4] - g[i, 1:4])
        trans_errors.append(np.linalg.norm(r_gt.T @ (t_est - t_gt)))
        rot_errors.append(np.degrees(Rotation.from_matrix(r_gt.T @ r_est).magnitude()))
    rpe = None
    if trans_errors:
        rpe = {"delta_s": delta, "tolerance_s": rpe_tolerance, "pairs": len(trans_errors),
               "translation_m": stats(np.asarray(trans_errors)),
               "rotation_deg": stats(np.asarray(rot_errors))}
    return {"assessment": "metrics_only_not_production_acceptance", "alignment": "SE3_scale_fixed_1",
            "estimated_poses": len(est), "expected_frames": expected_frames,
            "pose_coverage": len(est) / expected_frames if expected_frames is not None else None,
            "associated_poses": len(pairs), "association_coverage": len(pairs) / len(est),
            "association_max_dt_s": max_dt,
            "association_dt_s": stats(np.abs(e[:, 0] - g[:, 0])),
            "alignment_position_rank": int(np.linalg.matrix_rank(e[:, 1:4] - e[:, 1:4].mean(axis=0))),
            "ate_translation_m": stats(np.linalg.norm(aligned - g[:, 1:4], axis=1)),
            "alignment_covariance_rank": covariance_rank,
            "orientation_alignment": "observable_from_positions" if covariance_rank >= 2 else "ambiguous_from_positions",
            "ape_rotation_deg": stats(np.degrees(orientation_error)) if covariance_rank >= 2 else None,
            "rpe": rpe,
            "diagnostic_sim3_only": {"fitted_scale": sim3_scale,
                                     "ate_rmse_m": stats(np.linalg.norm(sim3 - g[:, 1:4], axis=1))["rmse"]}}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("estimate")
    parser.add_argument("ground_truth")
    parser.add_argument("--gt-format", choices=["tum", "euroc"], default="euroc")
    parser.add_argument("--estimate-frame", choices=["camera", "body"], required=True)
    parser.add_argument("--config", help="Required for camera-frame output")
    parser.add_argument("--expected-frames", type=int)
    parser.add_argument("--max-dt", type=float, default=0.01)
    parser.add_argument("--rpe-delta", type=float, default=1.0)
    parser.add_argument("--rpe-tolerance", type=float, default=0.05)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    try:
        est = load_poses(args.estimate)
        if args.estimate_frame == "camera":
            if not args.config:
                raise ValueError("--config is required for camera-frame output")
            est = camera_to_body(est, args.config)
        gt = load_poses(args.ground_truth, args.gt_format)
        result = evaluate(est, gt, args.expected_frames, args.max_dt, args.rpe_delta, args.rpe_tolerance)
        result["inputs"] = {"estimate": str(Path(args.estimate).resolve()),
                            "ground_truth": str(Path(args.ground_truth).resolve()),
                            "estimate_frame": args.estimate_frame, "gt_format": args.gt_format}
    except (ValueError, OSError, KeyError) as exc:
        parser.exit(2, f"Evaluation rejected: {exc}\n")
    text = json.dumps(result, indent=2, allow_nan=False) + "\n"
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(text)
    print(text, end="")


if __name__ == "__main__":
    main()

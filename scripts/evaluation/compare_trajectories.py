#!/usr/bin/env python3
"""
Trajectory Evaluation Script — ATE + RPE for TUM VI / EuRoC
============================================================
Computes standard VIO evaluation metrics against ground truth.

ATE (Absolute Trajectory Error):
  - Umeyama Sim(3) alignment (rotation + translation + scale)
  - RMSE / mean / median / std / max of per-pose position error

RPE (Relative Pose Error):
  - SE(3) relative pose errors at delta = 1s and 5s sub-sequences
  - Translation RMSE [m] and rotation RMSE [deg]

Frame conventions:
  - VIO output: camera frame  (body_pos + body_rot * t_ic)
  - GT (mocap0 / state_gt): IMU/body frame
  => VIO must be converted to body frame before comparison.

Usage:
    python compare_trajectories.py [experiment_path] [--save] [--no-display]

    experiment_path: path to logs/TIMESTAMP/ directory.
                     Auto-selects latest in logs/ if omitted.
"""

import os
import sys
import argparse
import warnings
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use('Agg')  # non-interactive backend — safe for headless servers
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy.spatial.transform import Rotation
import yaml

warnings.filterwarnings('ignore')


# ---------------------------------------------------------------------------
# YAML loading helper (handles OpenCV %YAML:1.0 + !!opencv-matrix tags)
# ---------------------------------------------------------------------------
class _IgnoreTags(yaml.SafeLoader):
    pass

_IgnoreTags.add_constructor(None, lambda loader, node: None)


def _load_yaml(path: str) -> dict:
    with open(path, 'r') as f:
        content = f.read()
    if content.startswith('%YAML:1.0'):
        content = '\n'.join(content.splitlines()[1:])
    return yaml.load(content, Loader=_IgnoreTags) or {}


def _parse_opencv_matrix(path: str, key: str) -> np.ndarray | None:
    """
    Manually parse an !!opencv-matrix node from a YAML file.
    Handles:
      key: !!opencv-matrix
         rows: R
         cols: C
         dt: d
         data: [v0, v1, ...]
    Returns ndarray of shape (R, C), or None if not found.
    """
    import re
    with open(path) as f:
        text = f.read()

    # Find the block starting at 'key:'
    pattern = re.compile(
        r'^' + re.escape(key) + r'\s*:.*?data\s*:\s*\[([^\]]*)\]',
        re.MULTILINE | re.DOTALL
    )
    m = pattern.search(text)
    if not m:
        return None

    data_str = m.group(1)
    vals = [float(v.strip()) for v in data_str.split(',') if v.strip()]

    # Also get rows/cols from the same block
    block = m.group(0)
    rows_m = re.search(r'rows\s*:\s*(\d+)', block)
    cols_m = re.search(r'cols\s*:\s*(\d+)', block)
    if rows_m and cols_m:
        rows = int(rows_m.group(1))
        cols = int(cols_m.group(1))
    else:
        # Guess square
        n = len(vals)
        side = int(round(n ** 0.5))
        rows = cols = side

    return np.array(vals).reshape(rows, cols)


# ---------------------------------------------------------------------------
# Pose utilities
# ---------------------------------------------------------------------------
def quat_to_rot(q_xyzw: np.ndarray) -> np.ndarray:
    """[qx, qy, qz, qw] → 3×3 rotation matrix."""
    return Rotation.from_quat(q_xyzw).as_matrix()


def rot_to_quat(R: np.ndarray) -> np.ndarray:
    """3×3 → [qx, qy, qz, qw]."""
    return Rotation.from_matrix(R).as_quat()


def pose_inverse(R: np.ndarray, t: np.ndarray):
    """Invert SE(3): (R, t) → (R^T, -R^T t)."""
    Ri = R.T
    ti = -Ri @ t
    return Ri, ti


def pose_compose(R1, t1, R2, t2):
    """Compose SE(3): T1 * T2."""
    return R1 @ R2, R1 @ t2 + t1


# ---------------------------------------------------------------------------
# Umeyama similarity alignment  (Sim(3): scale + rotation + translation)
# ---------------------------------------------------------------------------
def umeyama_alignment(src: np.ndarray, dst: np.ndarray, with_scale: bool = True):
    """
    Align src trajectory to dst using the Umeyama method.

    src, dst: (N, 3) position arrays
    Returns: scale s, rotation R (3×3), translation t (3,)
    such that dst ≈ s * R @ src + t
    """
    assert src.shape == dst.shape and src.ndim == 2 and src.shape[1] == 3

    n = src.shape[0]
    mu_src = src.mean(axis=0)
    mu_dst = dst.mean(axis=0)

    src_c = src - mu_src
    dst_c = dst - mu_dst

    sigma_src = (src_c ** 2).sum() / n
    cov = (dst_c.T @ src_c) / n

    U, D, Vt = np.linalg.svd(cov)
    S = np.eye(3)
    if np.linalg.det(U) * np.linalg.det(Vt) < 0:
        S[2, 2] = -1

    R = U @ S @ Vt
    if with_scale and sigma_src > 1e-10:
        scale = (D * S.diagonal()).sum() / sigma_src
    else:
        scale = 1.0

    t = mu_dst - scale * R @ mu_src
    return scale, R, t


# ---------------------------------------------------------------------------
# Data loading
# ---------------------------------------------------------------------------
def load_vio_trajectory(path: str) -> pd.DataFrame:
    """
    Load VIO output file.
    Format: # timestamp tx ty tz qx qy qz qw
    Returns DataFrame with columns: timestamp, x, y, z, qx, qy, qz, qw
    """
    rows = []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith('#'):
                continue
            parts = line.split()
            if len(parts) != 8:
                continue
            rows.append([float(p) for p in parts])
    df = pd.DataFrame(rows, columns=['timestamp', 'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw'])
    print(f"  VIO: {len(df)} poses, t=[{df.timestamp.min():.3f}, {df.timestamp.max():.3f}]s")
    return df


def _find_gt_file(dataset_path: str) -> str:
    """Locate ground-truth CSV (TUM VI or EuRoC)."""
    candidates = [
        os.path.join(dataset_path, 'mav0', 'mocap0', 'data.csv'),           # TUM VI
        os.path.join(dataset_path, 'mav0', 'state_groundtruth_estimate0', 'data.csv'),  # EuRoC
        os.path.join(dataset_path, 'dso', 'gt_imu.csv'),                    # TUM VI DSO
    ]
    for c in candidates:
        if os.path.exists(c):
            return c
    raise FileNotFoundError(f"No GT file found in {dataset_path}. Tried: {candidates}")


def load_ground_truth(dataset_path: str) -> pd.DataFrame:
    """
    Load GT from TUM VI (mocap0) or EuRoC (state_groundtruth_estimate0).
    Returns DataFrame: timestamp[s], x, y, z, qx, qy, qz, qw
    """
    gt_path = _find_gt_file(dataset_path)
    print(f"  GT file: {gt_path}")

    rows = []
    with open(gt_path) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith('#'):
                continue
            # Split by comma or whitespace
            parts = line.replace(',', ' ').split()
            if len(parts) < 8:
                continue
            try:
                vals = [float(p) for p in parts[:8]]
            except ValueError:
                continue

            # Detect nanosecond timestamps (> 1e15)
            ts = vals[0]
            if ts > 1e15:
                ts /= 1e9  # ns → s

            # Detect column order: mocap0/state_gt use qw-first after position
            # Format: timestamp px py pz qw qx qy qz
            # Reorder to qx qy qz qw for internal consistency
            px, py, pz = vals[1], vals[2], vals[3]
            qw, qx, qy, qz = vals[4], vals[5], vals[6], vals[7]

            rows.append([ts, px, py, pz, qx, qy, qz, qw])

    df = pd.DataFrame(rows, columns=['timestamp', 'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw'])
    df.sort_values('timestamp', inplace=True)
    df.reset_index(drop=True, inplace=True)
    print(f"  GT:  {len(df)} poses, t=[{df.timestamp.min():.3f}, {df.timestamp.max():.3f}]s")
    return df


# ---------------------------------------------------------------------------
# Frame transform: VIO camera → IMU/body
# ---------------------------------------------------------------------------
def load_extrinsics(config_path: str):
    """
    Load R_ic (camera→IMU rotation) and t_ic (camera translation in IMU frame)
    from VIO config yaml.  Handles !!opencv-matrix tags via direct text parsing.
    Returns (R_ic [3×3], t_ic [3,]) or (None, None) if not found.
    """
    r_ic = _parse_opencv_matrix(config_path, 'extrinsicRotation')
    t_mat = _parse_opencv_matrix(config_path, 'extrinsicTranslation')
    t_ic = t_mat.flatten() if t_mat is not None else None

    if r_ic is not None and t_ic is not None:
        print(f"  R_ic loaded, t_ic=[{t_ic[0]:.4f}, {t_ic[1]:.4f}, {t_ic[2]:.4f}]")
    else:
        print("  WARNING: extrinsics not found in config, skipping frame transform")

    return r_ic, t_ic


def transform_camera_to_body(vio_df: pd.DataFrame, r_ic: np.ndarray, t_ic: np.ndarray) -> pd.DataFrame:
    """
    Convert VIO camera-frame poses to body/IMU frame.

    VIO saves:  P_cam = P_body + R_body * t_ic
                R_cam = R_body * R_ic

    Inverse:    R_body = R_cam * R_ic^T
                P_body = P_cam - R_body * t_ic
    """
    r_ic_T = r_ic.T
    out_rows = []
    for _, row in vio_df.iterrows():
        R_cam = quat_to_rot(np.array([row.qx, row.qy, row.qz, row.qw]))
        P_cam = np.array([row.x, row.y, row.z])

        R_body = R_cam @ r_ic_T
        P_body = P_cam - R_body @ t_ic

        q = rot_to_quat(R_body)  # [qx, qy, qz, qw]
        out_rows.append([row.timestamp, P_body[0], P_body[1], P_body[2],
                         q[0], q[1], q[2], q[3]])

    return pd.DataFrame(out_rows, columns=['timestamp', 'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw'])


# ---------------------------------------------------------------------------
# Trajectory association
# ---------------------------------------------------------------------------
def associate_trajectories(est_df: pd.DataFrame, gt_df: pd.DataFrame,
                            max_dt: float = 0.05):
    """
    Associate estimated and GT poses by nearest timestamp.
    Returns (est_matched, gt_matched) as (N,) arrays of indices.
    """
    gt_ts = gt_df['timestamp'].values
    est_ts = est_df['timestamp'].values

    est_idx = []
    gt_idx = []

    for i, t in enumerate(est_ts):
        j = np.searchsorted(gt_ts, t)
        # Check neighbors
        candidates = [j - 1, j, j + 1]
        best_j, best_dt = -1, max_dt + 1
        for c in candidates:
            if 0 <= c < len(gt_ts):
                dt = abs(gt_ts[c] - t)
                if dt < best_dt:
                    best_dt, best_j = dt, c
        if best_j >= 0 and best_dt <= max_dt:
            est_idx.append(i)
            gt_idx.append(best_j)

    print(f"  Associated {len(est_idx)} pairs (max_dt={max_dt}s)")
    return np.array(est_idx), np.array(gt_idx)


# ---------------------------------------------------------------------------
# ATE
# ---------------------------------------------------------------------------
def compute_ate(est_pos: np.ndarray, gt_pos: np.ndarray):
    """
    Compute ATE with Umeyama Sim(3) alignment.
    est_pos, gt_pos: (N, 3)
    Returns dict with RMSE, mean, median, std, max, scale.
    """
    scale, R, t = umeyama_alignment(est_pos, gt_pos, with_scale=True)
    est_aligned = scale * (R @ est_pos.T).T + t

    errors = np.linalg.norm(est_aligned - gt_pos, axis=1)
    return {
        'rmse':   float(np.sqrt(np.mean(errors ** 2))),
        'mean':   float(errors.mean()),
        'median': float(np.median(errors)),
        'std':    float(errors.std()),
        'max':    float(errors.max()),
        'scale':  float(scale),
        'n':      len(errors),
        'aligned_est': est_aligned,
    }


# ---------------------------------------------------------------------------
# RPE
# ---------------------------------------------------------------------------
def compute_rpe(est_df: pd.DataFrame, gt_df: pd.DataFrame,
                gt_idx: np.ndarray, est_idx: np.ndarray,
                delta: float = 1.0, tol: float = 0.1):
    """
    Compute RPE at sub-sequence length delta [seconds].

    Uses the matched index arrays so we know which GT row corresponds
    to each estimated pose.

    Returns dict with rmse_trans [m], rmse_rot [deg], n.
    """
    est_ts = est_df['timestamp'].values

    trans_errors = []
    rot_errors_deg = []

    for k, i in enumerate(est_idx):
        # Find j in est_idx such that est_ts[j] - est_ts[i] ≈ delta
        target_t = est_ts[i] + delta
        best_k2 = -1
        best_dt = tol + 1
        for k2 in range(k + 1, len(est_idx)):
            j = est_idx[k2]
            dt = abs(est_ts[j] - target_t)
            if dt < best_dt:
                best_dt = dt
                best_k2 = k2
            if est_ts[est_idx[k2]] > target_t + tol:
                break

        if best_k2 < 0 or best_dt > tol:
            continue

        k2 = best_k2
        i2 = est_idx[k2]
        gi, gi2 = gt_idx[k], gt_idx[k2]

        # Estimated relative pose
        row_ei = est_df.iloc[i]
        row_ej = est_df.iloc[i2]
        R_ei = quat_to_rot(np.array([row_ei.qx, row_ei.qy, row_ei.qz, row_ei.qw]))
        t_ei = np.array([row_ei.x, row_ei.y, row_ei.z])
        R_ej = quat_to_rot(np.array([row_ej.qx, row_ej.qy, row_ej.qz, row_ej.qw]))
        t_ej = np.array([row_ej.x, row_ej.y, row_ej.z])

        # GT relative pose
        row_gi = gt_df.iloc[gi]
        row_gj = gt_df.iloc[gi2]
        R_gi = quat_to_rot(np.array([row_gi.qx, row_gi.qy, row_gi.qz, row_gi.qw]))
        t_gi = np.array([row_gi.x, row_gi.y, row_gi.z])
        R_gj = quat_to_rot(np.array([row_gj.qx, row_gj.qy, row_gj.qz, row_gj.qw]))
        t_gj = np.array([row_gj.x, row_gj.y, row_gj.z])

        # Relative transforms
        R_ei_inv, t_ei_inv = pose_inverse(R_ei, t_ei)
        R_est_rel, t_est_rel = pose_compose(R_ei_inv, t_ei_inv, R_ej, t_ej)

        R_gi_inv, t_gi_inv = pose_inverse(R_gi, t_gi)
        R_gt_rel, t_gt_rel = pose_compose(R_gi_inv, t_gi_inv, R_gj, t_gj)

        # Error: E = T_gt_rel^{-1} * T_est_rel
        R_gt_inv, t_gt_inv = pose_inverse(R_gt_rel, t_gt_rel)
        R_err, t_err = pose_compose(R_gt_inv, t_gt_inv, R_est_rel, t_est_rel)

        trans_errors.append(np.linalg.norm(t_err))

        # Rotation error in degrees
        angle = Rotation.from_matrix(R_err).magnitude()
        rot_errors_deg.append(np.degrees(angle))

    if not trans_errors:
        return {'rmse_trans': float('nan'), 'rmse_rot': float('nan'), 'n': 0, 'delta': delta}

    te = np.array(trans_errors)
    re = np.array(rot_errors_deg)
    return {
        'rmse_trans': float(np.sqrt(np.mean(te ** 2))),
        'mean_trans': float(te.mean()),
        'rmse_rot':   float(np.sqrt(np.mean(re ** 2))),
        'mean_rot':   float(re.mean()),
        'n':          len(te),
        'delta':      delta,
    }


# ---------------------------------------------------------------------------
# Plotting
# ---------------------------------------------------------------------------
def plot_trajectories(gt_pos: np.ndarray, est_raw: np.ndarray,
                      est_aligned: np.ndarray, output_dir: str,
                      save: bool, show: bool, tag: str = ''):
    fig = plt.figure(figsize=(18, 6))

    titles = ['3D', 'XY (top)', 'XZ (side)']
    for col, (i1, i2, zlabel) in enumerate([(0,1,'Y'), (0,2,'Z'), (1,2,'Z')]):
        ax = fig.add_subplot(1, 3, col + 1,
                             projection='3d' if col == 0 else None)
        if col == 0:
            ax.plot(gt_pos[:,0], gt_pos[:,1], gt_pos[:,2],
                    'b-', lw=1.5, label='GT', alpha=0.8)
            ax.plot(est_aligned[:,0], est_aligned[:,1], est_aligned[:,2],
                    'r-', lw=1.5, label='VIO (aligned)', alpha=0.8)
            ax.set_xlabel('X'); ax.set_ylabel('Y'); ax.set_zlabel('Z')
        else:
            ax.plot(gt_pos[:,i1], gt_pos[:,i2], 'b-', lw=1.5, label='GT', alpha=0.8)
            ax.plot(est_aligned[:,i1], est_aligned[:,i2],
                    'r-', lw=1.5, label='VIO (aligned)', alpha=0.8)
            ax.set_xlabel('XYZ'[i1]); ax.set_ylabel('XYZ'[i2])
            ax.axis('equal'); ax.grid(True)

        ax.set_title(titles[col])
        ax.legend(fontsize=8)

    plt.suptitle(f'Trajectory Comparison{" — " + tag if tag else ""}', fontsize=11)
    plt.tight_layout()

    if save:
        out = os.path.join(output_dir, 'trajectory_comparison.png')
        plt.savefig(out, dpi=150, bbox_inches='tight')
        print(f"  Plot saved: {out}")

    if show:
        plt.show()
    else:
        plt.close()


# ---------------------------------------------------------------------------
# Main entry point
# ---------------------------------------------------------------------------
def evaluate(experiment_path: str, save: bool = False, show: bool = True):
    print(f"\n{'='*60}")
    print(f"  Experiment: {experiment_path}")
    print(f"{'='*60}")

    # --- locate files ---
    traj_file = os.path.join(experiment_path, 'trajectory_pose.txt')
    if not os.path.exists(traj_file):
        raise FileNotFoundError(f"Trajectory not found: {traj_file}")

    config_files = [f for f in os.listdir(experiment_path)
                    if f.endswith(('.yaml', '.yml'))]
    if not config_files:
        raise FileNotFoundError(f"No yaml config in {experiment_path}")
    config_file = os.path.join(experiment_path, config_files[0])

    cfg = _load_yaml(config_file)
    dataset_path = cfg.get('dataset_path')
    if not dataset_path:
        raise ValueError(f"'dataset_path' not in {config_file}")
    print(f"  Dataset: {dataset_path}")

    # --- load data ---
    print("\n[1] Loading trajectories...")
    vio_df   = load_vio_trajectory(traj_file)
    gt_df    = load_ground_truth(dataset_path)

    # --- frame transform: camera → body ---
    print("\n[2] Transforming VIO to body frame...")
    r_ic, t_ic = load_extrinsics(config_file)
    if r_ic is not None and t_ic is not None:
        body_df = transform_camera_to_body(vio_df, r_ic, t_ic)
    else:
        print("  Skipping frame transform (no extrinsics)")
        body_df = vio_df.copy()

    # --- associate ---
    print("\n[3] Associating trajectories...")
    est_idx, gt_idx = associate_trajectories(body_df, gt_df, max_dt=0.05)
    if len(est_idx) < 10:
        raise ValueError(f"Too few associations ({len(est_idx)}). Check timestamp alignment.")

    est_pos = body_df[['x','y','z']].values[est_idx]
    gt_pos  = gt_df[['x','y','z']].values[gt_idx]

    # --- ATE ---
    print("\n[4] Computing ATE...")
    ate = compute_ate(est_pos, gt_pos)
    print(f"  Scale:  {ate['scale']:.4f}")
    print(f"  RMSE:   {ate['rmse']:.4f} m")
    print(f"  Mean:   {ate['mean']:.4f} m")
    print(f"  Median: {ate['median']:.4f} m")
    print(f"  Std:    {ate['std']:.4f} m")
    print(f"  Max:    {ate['max']:.4f} m")
    print(f"  N:      {ate['n']}")

    # --- RPE ---
    print("\n[5] Computing RPE...")
    rpe_results = {}
    for delta in [1.0, 5.0]:
        rpe = compute_rpe(body_df, gt_df, gt_idx, est_idx, delta=delta)
        rpe_results[delta] = rpe
        print(f"  delta={delta:.0f}s: trans RMSE={rpe['rmse_trans']:.4f}m  "
              f"rot RMSE={rpe['rmse_rot']:.2f}deg  N={rpe['n']}")

    # --- plot ---
    print("\n[6] Plotting...")
    plot_trajectories(
        gt_pos, est_pos, ate['aligned_est'],
        experiment_path, save=save, show=show,
        tag=os.path.basename(experiment_path)
    )

    # --- save results ---
    if save:
        out = os.path.join(experiment_path, 'evaluation.txt')
        with open(out, 'w') as f:
            f.write("Trajectory Evaluation\n")
            f.write("=====================\n\n")
            f.write(f"Dataset:    {dataset_path}\n")
            f.write(f"Trajectory: {traj_file}\n")
            f.write(f"N poses:    {ate['n']}\n\n")
            f.write("ATE (Umeyama Sim(3) alignment):\n")
            f.write(f"  Scale:  {ate['scale']:.6f}\n")
            f.write(f"  RMSE:   {ate['rmse']:.6f} m\n")
            f.write(f"  Mean:   {ate['mean']:.6f} m\n")
            f.write(f"  Median: {ate['median']:.6f} m\n")
            f.write(f"  Std:    {ate['std']:.6f} m\n")
            f.write(f"  Max:    {ate['max']:.6f} m\n\n")
            f.write("RPE:\n")
            for delta, rpe in rpe_results.items():
                f.write(f"  delta={delta:.0f}s: trans_rmse={rpe['rmse_trans']:.6f}m  "
                        f"rot_rmse={rpe['rmse_rot']:.4f}deg  N={rpe['n']}\n")
        print(f"  Results saved: {out}")

    return ate, rpe_results


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawTextHelpFormatter)
    parser.add_argument('experiment_path', nargs='?', default=None)
    parser.add_argument('--save', action='store_true')
    parser.add_argument('--no-display', action='store_true')
    args = parser.parse_args()

    exp_path = args.experiment_path
    if not exp_path:
        log_root = 'logs'
        if not os.path.isdir(log_root):
            print("ERROR: 'logs/' not found"); sys.exit(1)
        dirs = [os.path.join(log_root, d) for d in os.listdir(log_root)
                if os.path.isdir(os.path.join(log_root, d))]
        if not dirs:
            print("ERROR: 'logs/' is empty"); sys.exit(1)
        exp_path = max(dirs, key=os.path.getmtime)
        print(f"Using latest log: {exp_path}")

    if not os.path.isdir(exp_path):
        print(f"ERROR: not a directory: {exp_path}"); sys.exit(1)

    ate, rpe = evaluate(exp_path, save=args.save, show=not args.no_display)

    print(f"\n{'='*60}")
    print("SUMMARY")
    print(f"{'='*60}")
    print(f"ATE RMSE:          {ate['rmse']:.4f} m  (scale={ate['scale']:.3f})")
    rpe1 = rpe.get(1.0, {})
    rpe5 = rpe.get(5.0, {})
    print(f"RPE(1s) trans:     {rpe1.get('rmse_trans', float('nan')):.4f} m  "
          f"rot: {rpe1.get('rmse_rot', float('nan')):.2f} deg")
    print(f"RPE(5s) trans:     {rpe5.get('rmse_trans', float('nan')):.4f} m  "
          f"rot: {rpe5.get('rmse_rot', float('nan')):.2f} deg")
    print(f"{'='*60}\n")


if __name__ == '__main__':
    main()

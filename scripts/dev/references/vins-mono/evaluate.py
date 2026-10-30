#!/usr/bin/env python3
"""Reuse strict project metrics for full and timestamp-matched VINS/current poses."""
import argparse
import hashlib
import importlib.util
import json
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[4]
spec = importlib.util.spec_from_file_location("audit_metrics", ROOT / "scripts/evaluation/audit_metrics.py")
metrics = importlib.util.module_from_spec(spec)
spec.loader.exec_module(metrics)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--vins", type=Path, required=True)
    parser.add_argument("--native", type=Path, required=True)
    parser.add_argument("--ground-truth", type=Path, required=True)
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    runtime_path = args.vins / "runtime-summary.json"
    runtime = json.loads(runtime_path.read_text())
    summary = json.loads((args.native / "summary.json").read_text())
    denominator = runtime["input"]["images"]
    if denominator != summary["frames_processed"]:
        raise SystemExit("Different frame denominators")
    vins = metrics.validate_poses(np.asarray([[r["timestamp_ns"] * 1e-9, *r["position"], *r["quaternion_xyzw"]]
                                             for r in runtime["odometry"]]))
    current = metrics.camera_to_body(metrics.load_poses(args.native / "trajectory-camera.txt"), args.config)
    gt = metrics.load_poses(args.ground_truth, "euroc")
    pairs = metrics.associate(vins, current, 1e-6)
    if len(pairs) < 3:
        raise SystemExit("Fewer than three common engine timestamps")
    vins_indices, native_indices = np.asarray(pairs).T
    report = {
        "claim": "Bounded desktop TUM-VI input execution and metric comparison; no production or phone acceptance",
        "expected_frames": denominator, "common_pose_timestamps": len(pairs),
        "vins_full": metrics.evaluate(vins, gt, denominator, max_dt=0.01),
        "current_full": metrics.evaluate(current, gt, denominator, max_dt=0.01),
        "vins_common": metrics.evaluate(vins[vins_indices], gt, denominator, max_dt=0.01),
        "current_common": metrics.evaluate(current[native_indices], gt, denominator, max_dt=0.01),
        "first_pose_elapsed_dataset_s": {"vins": runtime["first_pose_elapsed_dataset_s"],
                                         "current": summary["first_pose_elapsed_dataset_s"]},
        "inputs_sha256": {str(p.resolve()): hashlib.sha256(p.read_bytes()).hexdigest()
                          for p in [runtime_path, args.native / "trajectory-camera.txt", args.ground_truth, args.config]},
        "frame_contract": "VINS nav_msgs odometry is T_W_I; current T_W_C converted using the fixed T_I_C",
        "speed_comparison_accepted": False,
        "speed_limitation": "VINS 0.5x ROS paced in 2-CPU container; current direct unpaced native and other host workloads",
    }
    with args.output.open("x") as output:
        json.dump(report, output, indent=2, allow_nan=False)
    for name in ["vins_full", "current_full", "vins_common", "current_common"]:
        row = report[name]
        print(name, "poses", row["estimated_poses"], "ATE_RMSE_m", row["ate_translation_m"]["rmse"],
              "rotation_RMSE_deg", row["ape_rotation_deg"]["rmse"], "GT", row["associated_poses"])


if __name__ == "__main__":
    main()

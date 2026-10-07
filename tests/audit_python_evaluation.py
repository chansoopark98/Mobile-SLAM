#!/usr/bin/env python3
"""Compare current Python metrics with the standalone C++ audit fixtures."""

import importlib.util
from pathlib import Path
import sys

import numpy as np
import pandas as pd


def main() -> None:
    if len(sys.argv) != 2:
        raise SystemExit("Usage: audit_python_evaluation.py <fixture-output-dir>")
    root = Path(__file__).resolve().parents[1]
    spec = importlib.util.spec_from_file_location(
        "current_evaluator", root / "scripts/evaluation/compare_trajectories.py"
    )
    evaluator = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(evaluator)
    directory = Path(sys.argv[1])
    gt = pd.read_csv(directory / "gt.csv", comment="#", header=None)
    gt.columns = ["timestamp", "x", "y", "z", "qw", "qx", "qy", "qz"]
    gt["timestamp"] /= 1e9

    for name in ["rotation90", "unmatched_prefix"]:
        est = evaluator.load_vio_trajectory(str(directory / f"{name}.txt"))
        ei, gi = evaluator.associate_trajectories(est, gt, max_dt=0.001)
        rpe = evaluator.compute_rpe(est, gt, gi, ei, delta=1, tol=0.001)
        expected = 90 if name == "rotation90" else 0
        assert rpe["n"] == 2, rpe
        assert np.isclose(rpe["rmse_rot"], expected), rpe
        print(f"PYTHON_ORACLE_MATCH {name} rotation_deg={rpe['rmse_rot']} pairs={rpe['n']}")

    est = evaluator.load_vio_trajectory(str(directory / "double_scale.txt"))
    ate = evaluator.compute_ate(est[["x", "y", "z"]].values, gt[["x", "y", "z"]].values)
    assert np.isclose(ate["scale"], 0.5), ate
    assert ate["rmse"] < 1e-12, ate
    print(f"METRIC_LIMIT python_Sim3_scale={ate['scale']} ATE_m={ate['rmse']} metric_SE3_oracle_m={2/3}")


if __name__ == "__main__":
    main()

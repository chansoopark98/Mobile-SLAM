#!/usr/bin/env python3
"""Export browser audit camera poses; timestamps are submitted-frame proxies."""

import argparse
import hashlib
import json
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation


def export(report_path, output_path):
    raw = report_path.read_bytes()
    report = json.loads(raw)
    pages = [page for page in report.get("pages", []) if page["pathname"] == "/test-tumvi.html"]
    if len(pages) != 1:
        raise ValueError("Need exactly one TUM replay page")
    frames = pages[0].get("replayFrames", [])
    poses = []
    for row in frames:
        if row.get("pose") is None:
            continue
        matrix = np.asarray(row["pose"], dtype=float)
        if matrix.shape != (16,) or not np.isfinite(matrix).all() or not np.isfinite(row["timestamp"]):
            raise ValueError("Non-finite or malformed pose/timestamp")
        matrix = matrix.reshape(4, 4)
        rotation = matrix[:3, :3]
        if not np.allclose(matrix[3], [0, 0, 0, 1], atol=1e-6):
            raise ValueError("Invalid homogeneous matrix")
        if not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-4) or not np.isclose(np.linalg.det(rotation), 1, atol=1e-4):
            raise ValueError("Invalid rotation matrix")
        quaternion = Rotation.from_matrix(rotation).as_quat()
        poses.append([row["timestamp"], *matrix[:3, 3], *quaternion])
    if len(poses) > 1 and not np.all(np.diff(np.asarray(poses)[:, 0]) > 0):
        raise ValueError("Submitted timestamps must be strictly increasing")
    output_path.parent.mkdir(parents=True, exist_ok=True)
    header = "# camera pose: submitted_frame_timestamp x y z qx qy qz qw\n"
    output_path.write_text(header + "".join(" ".join(f"{value:.12f}" for value in row) + "\n" for row in poses))
    metadata = {
        "input": str(report_path.resolve()),
        "input_sha256": hashlib.sha256(raw).hexdigest(),
        "wasm_variant": report.get("wasmVariant"),
        "output": str(output_path.resolve()),
        "estimated_poses": len(poses),
        "processed_frames": len(frames),
        "timestamp_source": "audit-submitted dataset camera timestamp, not engine-reported pose timestamp",
        "estimate_frame": "camera",
        "limits": "Bounded replay only; no live device clock/latency or production acceptance evidence",
    }
    output_path.with_suffix(output_path.suffix + ".json").write_text(json.dumps(metadata, indent=2) + "\n")
    return metadata


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("report", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    try:
        metadata = export(args.report, args.output)
    except (ValueError, OSError, KeyError) as error:
        parser.exit(2, f"Export rejected: {error}\n")
    print(json.dumps(metadata, indent=2))


if __name__ == "__main__":
    main()

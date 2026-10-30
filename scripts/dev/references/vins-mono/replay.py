#!/usr/bin/env python3
"""Run pinned VINS-Mono against a bounded, read-only TUM-VI room fixture."""
import argparse
import hashlib
import json
import os
import subprocess
from pathlib import Path

ROOT = Path(__file__).resolve().parents[4]
IMAGE = "sha256:0b13b81efd04612178cc6a5bfb5e6d85e49c7e1d7b17fd6571ad81bc672f5ba9"
REVISION = "90dabb5ec79946ae42fd2e1e91d4e69aabe1e25d"


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--frames", type=int, default=600)
    parser.add_argument("--rate", type=float, default=0.5)
    args = parser.parse_args()
    if not 1 <= args.frames <= 3000 or not 0 < args.rate <= 1:
        parser.error("frames must be 1..3000 and rate must be in (0, 1]")
    dataset = args.dataset.resolve()
    output = args.output.resolve()
    config_path = ROOT / "config/tum_vi_room1.yaml"
    camchain = dataset / "dso/camchain.yaml"
    if digest(camchain) != digest(ROOT / "assets/datasets/tum/dataset-room1_512_16/dso/camchain.yaml"):
        raise SystemExit("Only the matching TUM-VI room1 calibration is supported.")
    if output.exists():
        raise SystemExit("Refusing to replace existing replay evidence: " + str(output))
    image_info = json.loads(subprocess.check_output(["docker", "image", "inspect", IMAGE], text=True))[0]
    if image_info["Config"]["Labels"]["org.opencontainers.image.revision"] != REVISION:
        raise SystemExit("Unexpected VINS-Mono image source revision")
    output.mkdir(parents=True)
    config = config_path.read_text() + """
imu_topic: /imu0
image_topic: /cam0/image_raw
output_path: /output
estimate_extrinsic: 0
freq: 0
loop_closure: 0
load_previous_pose_graph: 0
fast_relocalization: 0
pose_graph_save_path: /output/pose_graph
estimate_td: 0
td: 0.0
rolling_shutter: 0
rolling_shutter_tr: 0.0
save_image: 0
visualize_imu_forward: 0
visualize_camera_size: 0.4
"""
    (output / "config.yaml").write_text(config)
    sources = subprocess.check_output(["git", "ls-files", "src", "include", "config", "CMakeLists.txt"], cwd=ROOT, text=True).splitlines()
    manifest = {
        "image_id": image_info["Id"], "upstream_revision": REVISION,
        "current_head": subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=ROOT, text=True).strip(),
        "dataset": str(dataset), "dataset_read_only": True, "frames_requested": args.frames,
        "playback_rate": args.rate, "calibration_sha256": digest(camchain),
        "config_sha256": digest(output / "config.yaml"),
        "current_source_sha256": {name: digest(ROOT / name) for name in sources},
        "wrapper_sha256": {p.name: digest(p) for p in [Path(__file__), Path(__file__).with_name("ros_replay.py")]},
        "image_inspect": image_info,
    }
    command = [
        "docker", "run", "--rm", "--network", "none", "--cpus", "2", "--user",
        str(os.getuid()) + ":" + str(os.getgid()),
        "-e", "HOME=/output", "-e", "ROS_HOME=/output/.ros", "-e", "ROS_HOSTNAME=127.0.0.1",
        "-e", "ROS_MASTER_URI=http://127.0.0.1:11311", "-e", "OMP_NUM_THREADS=1",
        "-v", str(dataset) + ":/dataset:ro", "-v", str(output) + ":/output",
        "-v", str(Path(__file__).parent.resolve()) + ":/wrapper:ro", IMAGE,
        "bash", "-c", "source /opt/ros/kinetic/setup.bash; source /opt/vins_ws/devel/setup.bash; "
        "exec python /wrapper/ros_replay.py " + str(args.frames) + " " + str(args.rate),
    ]
    manifest["command"] = command
    (output / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
    with (output / "container.log").open("w") as log:
        result = subprocess.run(command, stdout=log, stderr=subprocess.STDOUT)
    print(output)
    raise SystemExit(result.returncode)


if __name__ == "__main__":
    main()

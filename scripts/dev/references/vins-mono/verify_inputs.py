#!/usr/bin/env python
"""Compare actual ROS bag pixels/IMU with native_replay's binary input oracle."""
from __future__ import print_function
import argparse
import hashlib
import json
import os
import struct
import subprocess
import sys


def bag_digest():
    import rosbag
    images, imus = hashlib.sha256(), hashlib.sha256()
    image_count = imu_count = 0
    with rosbag.Bag("/vins/input.bag") as bag:
        for topic, msg, unused in bag.read_messages():
            if topic == "/cam0/image_raw":
                images.update(msg.data)
                image_count += 1
            elif topic == "/imu0":
                a, g = msg.linear_acceleration, msg.angular_velocity
                imus.update(struct.pack("<7d", float(msg.header.stamp.to_nsec()) * 1e-9,
                                        a.x, a.y, a.z, g.x, g.y, g.z))
                imu_count += 1
    return {"image_count": image_count, "imu_count": imu_count,
            "pixels_sha256": images.hexdigest(), "imu_sha256": imus.hexdigest()}


def native_digest(path):
    images, imus = hashlib.sha256(), hashlib.sha256()
    image_count = imu_count = 0
    with open(path, "rb") as source:
        while True:
            length = source.read(4)
            if not length:
                break
            if len(length) != 4:
                raise ValueError("Truncated native image-length header")
            pixel_size = struct.unpack("<I", length)[0]
            pixels = source.read(pixel_size)
            if len(pixels) != pixel_size:
                raise ValueError("Truncated native grayscale image")
            images.update(pixels)
            image_count += 1
            count = struct.unpack("<I", source.read(4))[0]
            samples = source.read(count * 56)
            if len(samples) != count * 56:
                raise ValueError("Truncated native IMU packet")
            imus.update(samples)
            imu_count += count
    return {"image_count": image_count, "imu_count": imu_count,
            "pixels_sha256": images.hexdigest(), "imu_sha256": imus.hexdigest()}


def main():
    if sys.argv[1:] == ["--ros-mode"]:
        print(json.dumps(bag_digest()))
        return 0
    from pathlib import Path
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--vins", type=Path, required=True)
    parser.add_argument("--native", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    manifest = json.loads((args.vins / "manifest.json").read_text())
    command = ["docker", "run", "--rm", "--network", "none", "--user", str(os.getuid()) + ":" + str(os.getgid()),
               "-v", str(args.vins.resolve()) + ":/vins:ro",
               "-v", str(Path(__file__).parent.resolve()) + ":/wrapper:ro", manifest["image_id"],
               "bash", "-c", "source /opt/ros/kinetic/setup.bash; python /wrapper/verify_inputs.py --ros-mode"]
    ros = json.loads(subprocess.check_output(command))
    native = native_digest(str(args.native / "input-oracle.bin"))
    report = {"passed": ros == native, "ros": ros, "native": native,
              "comparison": "All decoded grayscale pixel bytes and ordered unique 7-double IMU samples; packet scheduling differs"}
    with args.output.open("x") as output:
        json.dump(report, output, indent=2)
    print(json.dumps(report, indent=2))
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    sys.exit(main())

#!/usr/bin/env python
"""ROS Kinetic payload; Python 2.7 compatible, no upstream source edits."""
from __future__ import print_function
import csv
import hashlib
import json
import os
import signal
import subprocess
import sys
import time
import xmlrpclib

import cv2
import rosbag
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, Imu, PointCloud


def stamp(value):
    value = int(value)
    return rospy.Time(value // 1000000000, value % 1000000000)


def rows(path):
    with open(path) as source:
        return [r for r in csv.reader(source) if r and not r[0].startswith("#")]


def main():
    frame_limit, rate = int(sys.argv[1]), float(sys.argv[2])
    images = rows("/dataset/mav0/cam0/data.csv")[:frame_limit]
    imus = rows("/dataset/mav0/imu0/data.csv")
    first, last = int(images[0][0]), int(images[-1][0])
    after = next(i for i, r in enumerate(imus) if int(r[0]) > last)
    # Match native_replay's bracket policy: preserve all leading samples,
    # deliver every unique sample once, and retain the first sample after
    # the last image. ROS and direct-call scheduling remain different.
    imus = imus[:after + 1]
    events = [(int(r[0]), 0, r) for r in imus] + [(int(r[0]), 1, r) for r in images]
    events.sort(key=lambda item: (item[0], item[1]))
    selected_images = hashlib.sha256()
    cv2.setNumThreads(1)
    with rosbag.Bag("/output/input.bag", "w") as bag:
        for timestamp, kind, row in events:
            if kind == 0:
                msg = Imu()
                msg.header.frame_id = "imu0"
                msg.header.stamp = stamp(timestamp)
                msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z = map(float, row[1:4])
                msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z = map(float, row[4:7])
                msg.orientation_covariance[0] = -1
                topic = "/imu0"
            else:
                path = "/dataset/mav0/cam0/data/" + row[1]
                with open(path, "rb") as source:
                    selected_images.update(source.read())
                pixels = cv2.imread(path, cv2.IMREAD_GRAYSCALE)
                if pixels is None or pixels.shape != (512, 512):
                    raise RuntimeError("Missing or unexpected TUM-VI grayscale frame: " + path)
                msg = Image()
                msg.header.frame_id = "cam0"
                msg.header.stamp = stamp(timestamp)
                msg.height, msg.width = pixels.shape
                msg.encoding, msg.is_bigendian, msg.step = "mono8", 0, pixels.shape[1]
                msg.data = pixels.tostring()
                topic = "/cam0/image_raw"
            bag.write(topic, msg, msg.header.stamp)
    input_summary = {
        "images": len(images), "imu_samples": len(imus), "first_image_ns": first,
        "last_image_ns": last, "duration_s": (last - first) * 1e-9,
        "selected_png_bytes_sha256": selected_images.hexdigest(),
        "calibration": "current config/tum_vi_room1.yaml; camchain inverse, fixed extrinsic",
    }
    with open("/output/input-summary.json", "w") as output:
        json.dump(input_summary, output, indent=2)
    counts = {"images": 0, "imu": 0, "features": 0, "odometry": 0}
    feature_counts, odometry = [], []
    processes, logs = [], []

    def launch(command, filename):
        log = open("/output/" + filename, "w")
        logs.append(log)
        process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, preexec_fn=os.setsid)
        processes.append(process)
        return process

    def count_message(name):
        def callback(msg):
            counts[name] += 1
        return callback

    def feature_callback(msg):
        counts["features"] += 1
        feature_counts.append({"timestamp_ns": msg.header.stamp.to_nsec(), "count": len(msg.points)})

    def odometry_callback(msg):
        counts["odometry"] += 1
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        odometry.append({"timestamp_ns": msg.header.stamp.to_nsec(), "position": [p.x, p.y, p.z],
                         "quaternion_xyzw": [q.x, q.y, q.z, q.w]})

    begin = time.time()
    result = 1
    try:
        launch(["roscore"], "roscore.log")
        master = xmlrpclib.ServerProxy("http://127.0.0.1:11311")
        ready = False
        for unused in range(100):
            try:
                if master.getPid("reference_replay")[0] == 1:
                    ready = True
                    break
            except Exception:
                pass
            time.sleep(0.1)
        if not ready:
            raise RuntimeError("Isolated roscore did not start")
        rospy.init_node("reference_observer", anonymous=False, disable_signals=True)
        subscribers = [
            rospy.Subscriber("/cam0/image_raw", Image, count_message("images"), queue_size=2000),
            rospy.Subscriber("/imu0", Imu, count_message("imu"), queue_size=20000),
            rospy.Subscriber("/feature_tracker/feature", PointCloud, feature_callback, queue_size=2000),
            rospy.Subscriber("/vins_estimator/odometry", Odometry, odometry_callback, queue_size=2000),
        ]
        for package in ["feature_tracker", "vins_estimator"]:
            launch(["rosrun", package, package, "_config_file:=/output/config.yaml",
                    "_vins_folder:=/opt/vins_ws/src/VINS-Mono/"], package + ".log")
        time.sleep(3)
        if any(p.poll() is not None for p in processes):
            raise RuntimeError("A reference process exited before receiving input")
        player = launch(["rosbag", "play", "--clock", "--delay=2", "--rate=" + str(rate), "/output/input.bag"], "rosbag-play.log")
        play_begin = time.time()
        deadline = play_begin + max(60, input_summary["duration_s"] / rate + 30)
        while player.poll() is None and time.time() < deadline:
            time.sleep(0.1)
        if player.poll() is None:
            raise RuntimeError("Bounded rosbag playback timed out")
        time.sleep(5)
        result = 0 if player.returncode == 0 and odometry else 4
        summary = {
            "input": input_summary, "observed_topic_counts": counts,
            "feature_counts": feature_counts, "odometry": odometry,
            "playback_wall_s": time.time() - play_begin, "playback_rate": rate,
            "engine_source_unmodified": True, "loop_closure_enabled": False,
            "initialization_observed": bool(odometry),
            "first_pose_elapsed_dataset_s": (odometry[0]["timestamp_ns"] - first) * 1e-9 if odometry else None,
            "accuracy_accepted": False, "accuracy_limitation": "GT evaluation is separate; trajectory existence is not accuracy",
            "process_returncodes_before_shutdown": [p.poll() for p in processes],
        }
        with open("/output/runtime-summary.json", "w") as output:
            json.dump(summary, output, indent=2)
        print("Reference replay: input images=%d features=%d odometry=%d wall_s=%.3f" %
              (counts["images"], counts["features"], counts["odometry"], time.time() - begin))
    finally:
        for process in reversed(processes):
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
        deadline = time.time() + 10
        while any(p.poll() is None for p in processes) and time.time() < deadline:
            time.sleep(0.1)
        for process in processes:
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGKILL)
            process.wait()
        with open("/output/shutdown-summary.json", "w") as output:
            json.dump({"requested_signal": "SIGINT", "process_returncodes": [p.returncode for p in processes],
                       "known_upstream_shutdown_abort": any(p.returncode == -signal.SIGABRT for p in processes)}, output, indent=2)
        for log in logs:
            log.close()
        rospy.signal_shutdown("bounded replay complete")
    return result


if __name__ == "__main__":
    sys.exit(main())

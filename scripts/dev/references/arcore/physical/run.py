#!/usr/bin/env python3
"""Supervise one explicitly authorized owned-app experiment; preserve all output."""
import argparse
import hashlib
import json
from pathlib import Path
import re
import subprocess
import time

PACKAGE = "com.mobileslam.reference.arcore"
COMPONENT = PACKAGE + "/com.google.ar.core.examples.java.helloar.MeasurementInstrumentation"


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--serial", required=True)
    parser.add_argument("--build", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--run-id", required=True)
    parser.add_argument("--mode", choices=("live", "playback"), required=True)
    parser.add_argument("--source-run-id")
    parser.add_argument("--duration", type=int, choices=range(1, 31), required=True)
    for flag in ("save-images", "save-imu", "save-recording"):
        parser.add_argument("--" + flag, choices=("true", "false"), required=True)
    parser.add_argument("--authorization-note", required=True, help="Record the current authorization; never substitute this flag for human consent")
    args = parser.parse_args()
    for value in (args.run_id, args.source_run_id):
        if value is not None and not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9_-]{0,63}", value):
            parser.error("Unsafe run ID")
    if args.mode == "playback" and (not args.source_run_id or args.save_imu == "true" or args.save_recording == "true"):
        parser.error("Playback requires an owned source run and cannot record live IMU/new MP4")
    out = args.output.resolve()
    if out.is_relative_to(Path("/mnt/backup/SLAM")):
        parser.error("Canonical dataset is read-only")
    token = (args.build / "start-token.txt").read_text().strip()
    out.mkdir(parents=True, exist_ok=False)
    (out / "runner.py").write_bytes(Path(__file__).read_bytes())
    (out / "request.json").write_text(json.dumps(vars(args), default=str, ensure_ascii=False, indent=2) + "\n")
    adb = ["adb", "-s", args.serial]
    fields = {"token": token, "mode": args.mode, "runId": args.run_id, "durationSeconds": str(args.duration),
              "saveImages": args.save_images, "saveImu": args.save_imu, "saveArcoreRecording": args.save_recording}
    if args.source_run_id:
        fields["sourceRunId"] = args.source_run_id
    argv = adb + ["shell", "am", "instrument", "-w"]
    for key, value in fields.items():
        argv += ["-e", key, value]
    argv.append(COMPONENT)
    (out / "command.json").write_text(json.dumps(["<build-start-token>" if x == token else x for x in argv], indent=2) + "\n")
    started = time.monotonic()
    forced = False
    with (out / "instrumentation.stdout").open("xb") as stdout, (out / "instrumentation.stderr").open("xb") as stderr:
        process = subprocess.Popen(argv, stdout=stdout, stderr=stderr)
        try:
            process.wait(timeout=args.duration + 6)
        except subprocess.TimeoutExpired:
            forced = True
        finally:
            stop = subprocess.run(adb + ["shell", "am", "force-stop", PACKAGE], capture_output=True, timeout=10)
            (out / "stop.stdout").write_bytes(stop.stdout)
            (out / "stop.stderr").write_bytes(stop.stderr)
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.terminate()
                process.wait(timeout=5)
    result = {"runId": args.run_id, "instrumentationExit": process.returncode, "hostForcedAtDeadline": forced,
              "hostElapsedS": time.monotonic() - started, "stopExit": stop.returncode, "mode": args.mode}
    for name in ("summary.json", "lifecycle.json"):
        response = subprocess.run(adb + ["exec-out", "run-as", PACKAGE, "cat", "files/reference-runs/" + args.run_id + "/" + name], capture_output=True, timeout=10)
        (out / name).write_bytes(response.stdout)
        (out / (name + ".stderr")).write_bytes(response.stderr)
        result[name + "ReadExit"] = response.returncode
    ops = subprocess.run(adb + ["shell", "cmd", "appops", "get", PACKAGE, "CAMERA"], capture_output=True, timeout=10)
    (out / "camera-appops-after.txt").write_bytes(ops.stdout + ops.stderr)
    instrument_text = (out / "instrumentation.stdout").read_text(errors="replace")
    try:
        summary = json.loads((out / "summary.json").read_text())
        lifecycle = json.loads((out / "lifecycle.json").read_text())
        complete = (process.returncode == 0 and "status=finished" in instrument_text and "INSTRUMENTATION_CODE: -1" in instrument_text
                    and summary.get("runId") == args.run_id and summary.get("mode") == args.mode
                    and summary.get("frames", 0) > 0 and summary.get("setupError") is None
                    and summary.get("cleanupError") is None and lifecycle.get("sessionClosed") is True
                    and lifecycle.get("sessionCleanupError") is None
                    and all(summary.get(key) is (value == "true") for key, value in fields.items()
                            if key in ("saveImages", "saveImu", "saveArcoreRecording")))
    except (ValueError, OSError):
        complete = False
    result["sdkExecutionComplete"] = complete
    result["gateMeaning"] = "current instrumentation finished, nonzero current run, requested storage flags and Session closure verified; tracking/accuracy evaluated separately"
    result["runnerSha256"] = hashlib.sha256((out / "runner.py").read_bytes()).hexdigest()
    (out / "execution.json").write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps(result, indent=2))
    if forced or stop.returncode or not complete or result["summary.jsonReadExit"] or result["lifecycle.jsonReadExit"]:
        raise SystemExit(1)


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""Summarize actual ARCore SDK rows, including incomplete/timed-out sessions."""
import argparse
import collections
import hashlib
import json
import pathlib
import tarfile
from analyze import compare, stats


def summarize(directory, mode):
    result = json.loads((directory / (mode + "-result.json")).read_text())
    rows, malformed = [], []
    for index, line in enumerate((directory / (mode + "-poses.log")).read_text().splitlines()):
        try:
            rows.append(json.loads(line))
        except json.JSONDecodeError:
            malformed.append(index)
    frames = [r for r in rows if r.get("event") == "frame"]
    times = [int(r["timestampNs"]) for r in frames]
    arrivals = [int(r["arrivalElapsedNs"]) for r in frames]
    tracked = [r for r in frames if r["trackingState"] == "TRACKING"]
    images = [r for r in frames if "imagePayloadSha256" in r]
    checked, failed = 0, []
    if (directory / (mode + "-images.tar.log")).exists():
        try:
            with tarfile.open(directory / (mode + "-images.tar.log")) as archive:
                for row in images:
                    name = f'reference-{mode}-images/{row["index"]:06d}.pgm'
                    data = archive.extractfile(name).read().split(b"\n", 3)[3]
                    checked += 1
                    if hashlib.sha256(data).hexdigest() != row["imagePayloadSha256"]:
                        failed.append(row["index"])
        except (tarfile.TarError, KeyError, AttributeError) as error:
            failed.append(str(error))
    imu_counts = collections.Counter()
    imu_malformed = []
    for index, line in enumerate((directory / (mode + "-imu.log")).read_text().splitlines()):
        try:
            imu_counts[json.loads(line)["sensorType"]] += 1
        except (json.JSONDecodeError, KeyError):
            imu_malformed.append(index)
    summary = result["sdkSummary"]
    return {**result, "setupRows": [r for r in rows if r.get("event") == "setup"],
            "frameRowsObserved": len(frames), "trackingRowsObserved": len(tracked),
            "frameRowsMatchCompletedSummary": len(frames) == summary["frames"] if summary else None,
            "trackingStates": dict(collections.Counter(r["trackingState"] for r in frames)),
            "failureReasons": dict(collections.Counter(r["trackingFailureReason"] for r in frames)),
            "firstTrackingFrame": tracked[0]["index"] if tracked else None,
            "timestampStrictlyIncreasing": all(b > a for a, b in zip(times, times[1:])) if times else None,
            "uniqueTimestamps": len(set(times)),
            "sourceDurationS": (times[-1] - times[0]) / 1e9 if len(times) > 1 else None,
            "observedArrivalDurationS": (arrivals[-1] - arrivals[0]) / 1e9 if len(arrivals) > 1 else None,
            "cameraIntervalsMs": stats([(b - a) / 1e6 for a, b in zip(times, times[1:])]),
            "arrivalIntervalsMs": stats([(b - a) / 1e6 for a, b in zip(arrivals, arrivals[1:])]),
            "blockingSessionUpdateApiMs": stats([int(r["sessionUpdateDurationNs"]) / 1e6 for r in frames]),
            "pointCloudPoints": stats([r["pointCloudPoints"] for r in frames]),
            "imageTimestampMinusFrameNs": stats([int(r["imageTimestampNs"]) - int(r["timestampNs"]) for r in images]),
            "imagePayloadsChecked": checked, "imageHashFailures": failed,
            "imuRowsBySensorType": dict(imu_counts), "imuMalformedLines": imu_malformed,
            "malformedLines": malformed,
            "externalImuIdenticalToPrivateARCoreImu": None,
            "accuracy": None, "independentGT": None, "privateSolverTiming": None}, frames


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("directory", type=pathlib.Path)
    parser.add_argument("--output", type=pathlib.Path, help="New summary path; default is directory/analysis.json")
    args = parser.parse_args()
    destination = (args.output or args.directory / "analysis.json").resolve()
    if destination.is_relative_to(pathlib.Path("/mnt/backup/SLAM")):
        parser.error("Canonical dataset is read-only")
    if destination.exists():
        parser.error("Existing analysis is preserved; choose a new --output")
    runs = {mode: summarize(args.directory, mode) for mode in ("live", "record", "playback1", "playback2")
            if (args.directory / (mode + "-result.json")).exists()}
    output = {"runs": {mode: data[0] for mode, data in runs.items()}, "accuracy": None}
    if "playback1" in runs and "playback2" in runs:
        output["pairedPlaybackAgreement"] = compare(runs["playback1"][1], runs["playback2"][1])
    recording = args.directory / "recording.mp4.log"
    if recording.exists():
        output["recording"] = {"bytes": recording.stat().st_size,
                               "sha256": hashlib.sha256(recording.read_bytes()).hexdigest()}
    with destination.open("x") as summary_file:
        summary_file.write(json.dumps(output, indent=2) + "\n")
    print(json.dumps({mode: {k: data[0][k] for k in ("completed", "frameRowsObserved", "trackingRowsObserved")}
                      for mode, data in runs.items()}))


if __name__ == "__main__":
    main()

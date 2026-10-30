#!/usr/bin/env python3
"""Run the real PC entry and direct engine on one immutable fixture/profile."""
import argparse
import hashlib
import importlib.util
import json
import re
import subprocess
from pathlib import Path


def sha256(path):
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset", type=Path, required=True)
    parser.add_argument("--pc", type=Path, required=True)
    parser.add_argument("--native", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--timeout", type=int, default=900)
    args = parser.parse_args()
    dataset, pc, native, output = (p.resolve() for p in (args.dataset, args.pc, args.native, args.output))
    original_config = dataset / "config.yaml"
    config_text = original_config.read_text()
    # Override only the adapter's location; algorithm parameters stay byte-for-byte unchanged.
    config_text, replacements = re.subn(r"^dataset_path:.*$", lambda _: "dataset_path: " + json.dumps(str(dataset)), config_text, flags=re.M)
    if replacements != 1:
        raise ValueError("Fixture must contain exactly one dataset_path")
    def scalar(name, fallback):
        match = re.search(r"^" + re.escape(name) + r":\s*([^#\n]+)", config_text, flags=re.M)
        return match.group(1).strip() if match else fallback
    if int(scalar("frame_skip", "2")) != 0 or int(scalar("start_frame", "0")) != 0 or int(scalar("end_frame", "-1")) != -1:
        raise ValueError("Parity fixture must process all source frames")
    source_timestamps = [float(int(line.split(",", 1)[0])) * 1e-9
                         for line in (dataset / "mav0/cam0/data.csv").read_text().splitlines()
                         if line and not line.startswith("#")]
    frame_count = len(source_timestamps)
    if frame_count < 1:
        raise ValueError("No source frames")
    output.mkdir(parents=True, exist_ok=False)
    config = output / "config.pc.yaml"
    config.write_text(config_text)
    pc_work = output / "pc"
    pc_work.mkdir()
    commands = {
        "pc": [str(pc), str(config), "--headless", "--seed", "0", "--cv-threads", "1", "--dump-inputs"],
        "native": [str(native), str(dataset), str(config), str(output / "native"), str(frame_count),
                   "bracket", "0", "1", scalar("max_num_iterations", "10"), scalar("max_solver_time", ".06"),
                   "1", "0", "0", str(frame_count), "yaml"],
    }
    provenance = {"commands": commands, "dataset": str(dataset), "expected_frames": frame_count,
                  "original_config_sha256": sha256(original_config), "run_config_sha256": sha256(config),
                  "pc_binary_sha256": sha256(pc), "native_binary_sha256": sha256(native),
                  "script_sha256": sha256(Path(__file__)), "pose_frame": "T_W_C", "alignment": "none"}
    manifest = dataset / "manifest.json"
    if manifest.is_file():
        provenance["fixture_manifest_sha256"] = sha256(manifest)
    (output / "provenance.json").write_text(json.dumps(provenance, indent=2) + "\n")
    exits = {}
    for route, command in commands.items():
        with (output / (route + ".log")).open("w") as log:
            try:
                exits[route] = subprocess.run(command, cwd=pc_work if route == "pc" else output,
                                             stdout=log, stderr=subprocess.STDOUT, timeout=args.timeout).returncode
            except subprocess.TimeoutExpired:
                exits[route] = "timeout"
    result = {"status": "fail", "exit_codes": exits, "failures": [],
              "scope": "exact-input PC application/direct native parity; not physical mobile accuracy"}
    try:
        if exits["pc"] != 0 or exits["native"] not in (0, 4):
            raise ValueError("A replay failed before normal completion")
        directories = list((pc_work / "logs").iterdir())
        if len(directories) != 1:
            raise ValueError("Expected one unique PC log directory")
        pc_log, direct_log = directories[0], output / "native"
        result["pc_log_directory"] = str(pc_log)
        pc_summary = json.loads((pc_log / "summary.json").read_text())
        direct_summary = json.loads((direct_log / "summary.json").read_text())
        result["pc_summary"] = pc_summary
        for key in ("frames_processed", "imu_available", "imu_delivered", "pose_count", "first_pose_frame"):
            if pc_summary[key] != direct_summary[key]:
                result["failures"].append(key)
        if pc_summary["frames_processed"] != frame_count or pc_summary["missing_images"]:
            result["failures"].append("incomplete_images")
        readbacks = [json.loads((directory / "effective-config.json").read_text()) for directory in (pc_log, direct_log)]
        for key in ("params", "executionSeed", "cvThreads", "opencvVersion"):
            if readbacks[0][key] != readbacks[1][key]:
                result["failures"].append("effective_config." + key)
        input_hashes = [sha256(directory / "input-oracle.bin") for directory in (pc_log, direct_log)]
        result["input_oracle_sha256"] = dict(zip(("pc", "native"), input_hashes))
        if input_hashes[0] != input_hashes[1]:
            result["failures"].append("input_bytes_or_imu_differ")
        for route, directory in (("pc", pc_log), ("native", direct_log)):
            rows = json.loads((directory / "results.json").read_text())["frames"]
            if len(rows) != frame_count or any(
                    row["frame"] != index or row["inputTimestamp"] != timestamp or
                    (row["poseValid"] and row["poseTimestamp"] != timestamp)
                    for index, (row, timestamp) in enumerate(zip(rows, source_timestamps))):
                result["failures"].append(route + ".source_timestamp_or_frame_identity")
        spec = importlib.util.spec_from_file_location("compare_replay", Path(__file__).with_name("compare-replay.py"))
        comparison = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(comparison)
        metrics = comparison.compare(direct_log / "results.json", pc_log / "results.json", frame_count)
        result["parity"] = metrics
        if metrics["state_differences"] or metrics["init_frame_difference"] or metrics["coverage_difference"]:
            result["failures"].append("state_or_freshness_difference")
        if metrics["translation_m"]["max"] > 1e-4 or metrics["rotation_deg"]["max"] > 1e-4 * 180 / 3.141592653589793:
            result["failures"].append("native_numeric_cap")
        result["status"] = "fail" if result["failures"] else "pass"
    except (OSError, ValueError, KeyError) as error:
        result["failures"].append(str(error))
    (output / "comparison.json").write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps(result, indent=2))
    return 0 if result["status"] == "pass" else 1


if __name__ == "__main__":
    raise SystemExit(main())

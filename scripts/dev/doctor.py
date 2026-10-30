#!/usr/bin/env python3
"""Read-only project tool inventory. Status describes probes, not SLAM accuracy."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
from importlib.metadata import PackageNotFoundError, version
from datetime import datetime, timezone
from itertools import islice

ROOT = Path(__file__).resolve().parents[2]
DEV = ROOT / "scripts/dev"
DATA_ROOT = Path(os.environ.get("MOBILE_SLAM_DATA_ROOT") or "/mnt/backup/SLAM").expanduser()


def shallow_inventory(path, limit=16):
    """At most root plus one child level; never read or hash dataset contents."""
    try:
        with os.scandir(path) as directory:
            entries = list(islice(directory, limit + 1))
        result = {"status": "VERIFIED", "truncated": len(entries) > limit, "entries": []}
        for entry in entries[:limit]:
            kind = "directory" if entry.is_dir(follow_symlinks=False) else "symlink" if entry.is_symlink() else "file"
            result["entries"].append({"name": entry.name, "kind": kind})
        return result
    except OSError as error:
        return {"status": "BLOCKED", "reason": str(error), "entries": []}


def dataset_inventory():
    path = DATA_ROOT.absolute()
    shared = {"path": str(path), "scope": "canonical_read_only", **shallow_inventory(path)}
    for entry in shared["entries"]:
        if entry["kind"] == "directory":
            entry["children"] = shallow_inventory(path / entry["name"], limit=8)
    fixture = ROOT / "assets/datasets/tum/dataset-room1_512_16"
    return {"canonical": shared, "repo_official_room1_fixture": {
        "path": str(fixture), "scope": "separately_restored_official_fixture",
        "csv_presence": {sensor: (fixture / "mav0" / sensor / "data.csv").is_file()
                         for sensor in ("cam0", "imu0", "mocap0")},
    }, "limits": {"depth": 2, "root_entries": 16, "children_per_directory": 8,
                   "recursive_scan": False, "content_read_or_hash": False, "writes_or_downloads": False}}


def run(argv, timeout=15):
    try:
        result = subprocess.run(argv, cwd=ROOT, capture_output=True, text=True, timeout=timeout)
        return {"exit_code": result.returncode, "output": (result.stdout + result.stderr).strip()[:1600]}
    except (OSError, subprocess.TimeoutExpired) as error:
        return {"exit_code": None, "output": str(error)}


def executable(name, fallback=None):
    path = shutil.which(name)
    if not path and fallback and fallback.is_file():
        path = str(fallback)
    if not path:
        return {"status": "BLOCKED", "reason": "not on PATH or at known local path"}
    result = run([path, "--version"])
    return {"path": path, "status": "VERIFIED" if result["exit_code"] == 0 else "BLOCKED", **result}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, help="Save JSON evidence")
    parser.add_argument("--strict", action="store_true", help="Fail when required native/JS tools or npm packages are missing")
    args = parser.parse_args()
    tools = {name: executable(name) for name in ("g++", "cmake", "ctest", "ninja", "node", "npm", "python3", "google-chrome")}
    tools["clangd"] = executable("clangd", DEV / "tools/clangd_23.1.0/bin/clangd")
    for name in ("clang-tidy", "clang-format", "tsc", "ast-grep"):
        tools[name] = executable(name)
    sdk = Path(os.environ.get("EMSDK", str(Path.home() / "emsdk")))
    tools["emcc"] = executable("emcc", sdk / "upstream/emscripten/emcc")
    packages = {}
    manifest = json.loads((DEV / "package.json").read_text())
    for name, expected in manifest["devDependencies"].items():
        installed = DEV / "node_modules" / name / "package.json"
        actual = json.loads(installed.read_text())["version"] if installed.is_file() else None
        packages[name] = {"expected": expected, "installed": actual, "status": "VERIFIED" if expected == actual else "BLOCKED"}
    hashes = {}
    for relative in ("CMakeLists.txt", "wasm/CMakeLists.txt", "src/vio_engine.cpp", "src/backend/estimator.cpp", "web/js/app.js", "web/vio_engine.js", "web/vio_engine.wasm"):
        path = ROOT / relative
        if path.is_file():
            with path.open("rb") as source:
                digest = hashlib.file_digest(source, "sha256").hexdigest()
            hashes[relative] = {"sha256": digest, "bytes": path.stat().st_size}
    dependencies = {name: run(["pkg-config", "--modversion", name]) for name in ("eigen3", "opencv4", "yaml-cpp")}
    evaluation_modules = {}
    evaluation_pins = dict(line.strip().split("==", 1)
                           for line in (DEV / "requirements-evaluation.txt").read_text().splitlines()
                           if line.strip() and not line.strip().startswith("#"))
    for name, expected in evaluation_pins.items():
        try:
            installed = version(name)
            evaluation_modules[name] = {"status": "VERIFIED" if installed == expected else "BLOCKED",
                                        "version": installed, "expected": expected}
        except PackageNotFoundError:
            evaluation_modules[name] = {"status": "BLOCKED", "reason": "not installed in this Python environment"}
    report = {"recorded_at": datetime.now(timezone.utc).isoformat(), "root": str(ROOT),
              "git_head": run(["git", "rev-parse", "HEAD"]), "tools": tools, "packages": packages,
              "python_executable": sys.executable, "evaluation_modules": evaluation_modules,
              "native_dependencies": dependencies, "fingerprints": hashes,
              "datasets": dataset_inventory(),
              "compile_databases": [str(path.relative_to(ROOT)) for path in ROOT.glob("build/*/compile_commands.json")],
              "limits": ["Version probes do not verify algorithm correctness or device performance.",
                         "The native CMake project still requires Pangolin, Ceres and GoogleTest.",
                         "clang-tidy, clang-format, tsc and ast-grep are optional; sg is not ast-grep."]}
    encoded = json.dumps(report, indent=2) + "\n"
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(encoded)
    print(encoded, end="")
    required = ("g++", "cmake", "ctest", "ninja", "node", "npm", "python3", "google-chrome", "clangd", "emcc")
    if args.strict and (any(tools[name]["status"] != "VERIFIED" for name in required)
                        or any(item["status"] != "VERIFIED" for item in packages.values())
                        or any(item["status"] != "VERIFIED" for item in evaluation_modules.values())):
        raise SystemExit(1)


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""Isolated Vuforia reference environment. No download, license key or engine activation."""
import argparse
import datetime
import hashlib
import json
import os
import pathlib
import platform
import re
import shutil
import stat
import subprocess
import sys
import zipfile

SOURCE = pathlib.Path(__file__).resolve().parent
ROOT = SOURCE.parents[3]
DEFAULT_ASSETS = ROOT / "assets/references/Vuforia"


def sha(path):
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def run(argv, cwd, evidence, name, env=None):
    result = subprocess.run([str(x) for x in argv], cwd=cwd, env=env, text=True,
                            stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    (evidence / (name + ".stdout.log")).write_text(result.stdout)
    (evidence / (name + ".stderr.log")).write_text(result.stderr)
    return {"argv": [str(x) for x in argv], "cwd": str(cwd), "exit_code": result.returncode,
            "stdout": name + ".stdout.log", "stderr": name + ".stderr.log"}


def locate_sdk(root):
    headers = list(root.glob("**/VuforiaEngine/VuforiaEngine.h")) if root.exists() else []
    libraries = list(root.glob("**/arm64-v8a/libVuforia.so")) if root.exists() else []
    if len(headers) != 1 or len(libraries) != 1:
        raise ValueError("SDK_NOT_PROVISIONED: need exactly one VuforiaEngine/VuforiaEngine.h and arm64-v8a/libVuforia.so in official Android SDK")
    header, library = headers[0], libraries[0]
    text = header.read_text()
    version = [re.search(r"#\s*define\s+VU_VERSION_" + part + r"\s+(\d+)", text) for part in ("MAJOR", "MINOR", "PATCH")]
    if any(match is None for match in version) or tuple(int(match[1]) for match in version) != (11, 4, 4):
        raise ValueError("VERSION_MISMATCH: reference requires official Vuforia 11.4.4 headers")
    return {"version": "11.4.4", "root": str(root), "include": str(header.parents[1]),
            "library": str(library), "header_sha256": sha(header), "library_sha256": sha(library)}


def provision(archive, expected_sha, assets):
    if not archive or not archive.is_file():
        raise ValueError("SDK_ARCHIVE_REQUIRED: supply --sdk-archive from the official Engine Developer Portal and --sdk-sha256 from its checksum")
    if not expected_sha or not re.fullmatch(r"[0-9a-fA-F]{64}", expected_sha):
        raise ValueError("SDK_CHECKSUM_REQUIRED: supply a 64-character official SHA-256")
    if archive.stat().st_size > 512 * 1024 * 1024:
        raise ValueError("SDK_ARCHIVE_TOO_LARGE: bound is 512 MiB compressed")
    actual = sha(archive)
    if actual != expected_sha.lower():
        raise ValueError("SDK_CHECKSUM_MISMATCH: archive not extracted")
    target = assets / ("sdk-11.4.4-" + actual[:12])
    if not target.exists():
        with zipfile.ZipFile(archive) as package:
            entries = package.infolist()
            if sum(x.file_size for x in entries) > 1024 * 1024 * 1024:
                raise ValueError("SDK_UNPACK_LIMIT: bound is 1 GiB uncompressed")
            for entry in entries:
                path = pathlib.PurePosixPath(entry.filename)
                if path.is_absolute() or ".." in path.parts or "\\" in entry.filename or stat.S_ISLNK(entry.external_attr >> 16):
                    raise ValueError("SDK_UNSAFE_ARCHIVE: absolute paths, traversal and symlinks rejected")
            # Validate all paths before creating files. No packaged code is executed.
            target.mkdir()
            for entry in entries:
                output = target / entry.filename
                if entry.is_dir():
                    output.mkdir(parents=True, exist_ok=True)
                else:
                    output.parent.mkdir(parents=True, exist_ok=True)
                    with package.open(entry) as src, output.open("xb") as dst:
                        shutil.copyfileobj(src, dst)
    info = locate_sdk(target)
    info.update(archive=str(archive), archive_sha256=actual, checksum_source="operator-supplied official checksum; no authentication inferred")
    return info


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("command", choices=("status", "provision", "self-test", "build-native", "build-app"))
    parser.add_argument("--assets", type=pathlib.Path, default=DEFAULT_ASSETS)
    parser.add_argument("--evidence", type=pathlib.Path, default=ROOT / "build/research-20261012/vuforia/environment")
    parser.add_argument("--sdk-archive", type=pathlib.Path)
    parser.add_argument("--sdk-sha256")
    parser.add_argument("--sdk-root", type=pathlib.Path)
    parser.add_argument("--ndk", type=pathlib.Path)
    parser.add_argument("--android-sdk", type=pathlib.Path)
    parser.add_argument("--gradle", type=pathlib.Path)
    parser.add_argument("--java-home", type=pathlib.Path)
    args = parser.parse_args()
    assets, evidence = args.assets.resolve(), args.evidence.resolve()
    assets.mkdir(parents=True, exist_ok=True)
    evidence.mkdir(parents=True, exist_ok=True)
    manifest = {"schema": "vuforia-reference-environment-v1", "command": args.command,
                "queried_at_utc": datetime.datetime.now(datetime.timezone.utc).isoformat(),
                "host": platform.uname()._asdict(), "commands": [],
                "sdk_runtime_executed": False, "engine_activated": False,
                "physical_accuracy_verified": False, "jobs": 1,
                "source_sha256": {str(p.relative_to(SOURCE)): sha(p) for p in SOURCE.rglob("*")
                                  if p.is_file() and "__pycache__" not in p.parts and p.suffix != ".pyc"}}
    exit_code = 0
    try:
        if args.command == "self-test":
            build = assets / "host-contract-build"
            commands = [(["cmake", "-S", SOURCE, "-B", build, "-DCMAKE_BUILD_TYPE=Release"], "host-configure"),
                        (["cmake", "--build", build, "--parallel", "1"], "host-build"),
                        (["ctest", "--test-dir", build, "--output-on-failure"], "host-ctest"),
                        ([build / "vuforia_public_contracts"], "host-executable"),
                        ([sys.executable, SOURCE / "contract_probe.py"], "python-contract"),
                        (["node", SOURCE / "timestamp_probe.mjs"], "node-contract"),
                        ([sys.executable, SOURCE / "provision_safety_test.py"], "provision-safety")]
            env = dict(os.environ, VUFORIA_EVIDENCE_DIR=str(evidence))
            for argv, name in commands:
                record = run(argv, ROOT, evidence, name, env)
                manifest["commands"].append(record)
                if record["exit_code"]:
                    raise ValueError("SELF_TEST_FAILED: " + name)
            manifest.update(status="OFFLINE_CONTRACTS_PASS", evidence_scope="independent_arithmetic_executable; no SDK headers or runtime used")
        else:
            if args.command == "provision":
                sdk = provision(args.sdk_archive.resolve() if args.sdk_archive else None, args.sdk_sha256, assets)
                (assets / "provisioned-sdk.json").write_text(json.dumps(sdk, indent=2) + "\n")
            elif args.sdk_root:
                sdk = locate_sdk(args.sdk_root.resolve())
            elif (assets / "provisioned-sdk.json").exists():
                previous = json.loads((assets / "provisioned-sdk.json").read_text())
                sdk = locate_sdk(pathlib.Path(previous["root"]))
                if sdk["header_sha256"] != previous["header_sha256"] or sdk["library_sha256"] != previous["library_sha256"]:
                    raise ValueError("SDK_FINGERPRINT_CHANGED: re-provision from verified archive")
            else:
                raise ValueError("SDK_NOT_PROVISIONED: official Android SDK archive and checksum required")
            manifest["sdk"] = sdk
            manifest["status"] = "SDK_PROVISIONED_NOT_ACTIVATED"
            if args.command == "build-native":
                if not args.ndk or not (args.ndk / "build/cmake/android.toolchain.cmake").is_file():
                    raise ValueError("NDK_REQUIRED: supply --ndk pointing to Android NDK 26.1.10909125 or later")
                build = assets / "android-arm64-build"
                argv = ["cmake", "-S", SOURCE, "-B", build, "-DCMAKE_BUILD_TYPE=Release",
                        "-DCMAKE_TOOLCHAIN_FILE=" + str(args.ndk.resolve() / "build/cmake/android.toolchain.cmake"),
                        "-DANDROID_ABI=arm64-v8a", "-DANDROID_PLATFORM=android-29",
                        "-DVUFORIA_REFERENCE_WITH_SDK=ON", "-DVUFORIA_SDK_INCLUDE=" + sdk["include"],
                        "-DVUFORIA_SDK_LIBRARY=" + sdk["library"]]
                for command, name in ((argv, "android-configure"), (["cmake", "--build", build, "--parallel", "1"], "android-build")):
                    record = run(command, ROOT, evidence, name)
                    manifest["commands"].append(record)
                    if record["exit_code"]:
                        raise ValueError("ANDROID_NATIVE_BUILD_FAILED: " + name)
                manifest.update(status="ANDROID_NATIVE_LINKED_NOT_RUN", artifact_sha256=sha(build / "libvuforia_reference.so"))
            if args.command == "build-app":
                if not args.android_sdk or not args.gradle or not args.gradle.is_file():
                    raise ValueError("ANDROID_APP_TOOLCHAIN_REQUIRED: --android-sdk and --gradle executable, Java17, platform35/build-tools35, NDK26.1.10909125, CMake3.22.1")
                stage = assets / "version-app-source"
                # Work only on an owned reference copy; generated Gradle files never enter tracked sources.
                shutil.copytree(SOURCE, stage, dirs_exist_ok=True, ignore=shutil.ignore_patterns("__pycache__", "*.pyc"))
                env = dict(os.environ, ANDROID_HOME=str(args.android_sdk.resolve()),
                           GRADLE_USER_HOME=str(assets / "gradle-cache"),
                           VUFORIA_REFERENCE_INCLUDE=sdk["include"], VUFORIA_REFERENCE_LIBRARY=sdk["library"])
                if args.java_home:
                    env["JAVA_HOME"] = str(args.java_home.resolve())
                record = run([args.gradle.resolve(), "--no-daemon", "--max-workers=1", "-Dorg.gradle.parallel=false", "assembleDebug"], stage / "android-app", evidence, "android-apk-build", env)
                manifest["commands"].append(record)
                if record["exit_code"]:
                    raise ValueError("ANDROID_APP_BUILD_FAILED: read android-apk-build.stderr.log")
                apk = stage / "android-app/app/build/outputs/apk/debug/app-debug.apk"
                manifest.update(status="ANDROID_APK_BUILT_NOT_RUN", apk=str(apk), artifact_sha256=sha(apk))
    except (ValueError, OSError, zipfile.BadZipFile) as exc:
        manifest.update(status="BLOCKED", blocker=str(exc))
        exit_code = 2
    manifest["exit_code"] = exit_code
    (evidence / (args.command + ".json")).write_text(json.dumps(manifest, indent=2) + "\n")
    print(json.dumps({k: manifest[k] for k in ("status", "blocker", "exit_code", "evidence_scope") if k in manifest}, indent=2))
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())

#!/usr/bin/env python3
"""Rebuild and run the public ARCore sample without changing existing SDK/AVDs.

Only the fresh --output directory and a newly named Docker container are writable.
The SDK/reference mounts are read-only; no host ADB or physical device is used.
"""
import argparse
import hashlib
import json
import pathlib
import shutil
import subprocess
import time

ROOT = pathlib.Path(__file__).resolve().parents[4]
REFERENCE = ROOT / "assets/references/ARCore"
IMAGE = "mobile-slam-arcore-env:20261012"
PACKAGE = "com.google.ar.core.examples.java.helloar"
SERIAL = "emulator-5556"


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


class Run:
    def __init__(self, output, container, motion=False):
        self.out = output.resolve()
        self.container = container
        self.records = []
        self.owned = False
        self.motion = motion
        self.out.mkdir(parents=True, exist_ok=False)
        (self.out / "runner.py").write_bytes(pathlib.Path(__file__).read_bytes())

    def cmd(self, name, argv, *, check=True, input=None, timeout=120):
        path = self.out / (name + ".log")
        start = time.monotonic()
        timed_out = False
        with path.open("xb") as stream:
            try:
                result = subprocess.run(argv, input=input, stdout=stream,
                                        stderr=subprocess.STDOUT, timeout=timeout)
                code = result.returncode
            except subprocess.TimeoutExpired:
                timed_out, code = True, 124
        self.records.append({"name": name, "argv": argv, "exit": code,
                             "timedOut": timed_out,
                             "durationMs": (time.monotonic() - start) * 1000,
                             "logSha256": sha(path), "logBytes": path.stat().st_size})
        (self.out / "commands.json").write_text(json.dumps(self.records, indent=2) + "\n")
        print(name, code, flush=True)
        if check and code:
            raise RuntimeError(f"{name}: exit {code}; evidence {path}")
        return code

    def docker(self, name, args, **kwargs):
        return self.cmd(name, ["docker", "exec", *(["-i"] if kwargs.get("input") else []),
                               self.container, *args], **kwargs)

    def adb(self, name, args, **kwargs):
        return self.docker(name, ["adb", "-s", SERIAL, *args], **kwargs)

    def initialize(self):
        # Check absence before creating any container; never reuse an unrelated name.
        existing = subprocess.run(["docker", "inspect", self.container], capture_output=True)
        if existing.returncode == 0:
            raise RuntimeError("Container name already exists; choose a fresh name")
        source = REFERENCE / "source/samples/hello_ar_java"
        shutil.copytree(source, self.out / "sample", ignore=shutil.ignore_patterns("build", ".gradle", "local.properties"))
        observer = self.out / "sample/app/src/main/java/com/google/ar/core/examples/java/helloar/ReferenceAudit.java"
        # Preserve partial SDK output on timeout instead of losing PrintWriter buffers.
        text = observer.read_text()
        for kind in ("poses", "imu"):
            old = f'{kind} = new PrintWriter(new File(activity.getFilesDir(), "reference-" + mode + "-{kind}.jsonl"));'
            new = f'{kind} = new PrintWriter(new FileOutputStream(new File(activity.getFilesDir(), "reference-" + mode + "-{kind}.jsonl")), true);'
            if old not in text:
                raise RuntimeError("Pinned observer statement changed")
            text = text.replace(old, new)
        observer.write_text(text)
        motion = pathlib.Path(__file__).with_name("motion.py").read_text()
        (self.out / "motion.py").write_text(motion.replace("/work/toolchain/grpc-generated", "/reference/toolchain/grpc-generated"))
        manifest = {"upstreamCommit": subprocess.check_output(["git", "-C", str(REFERENCE / "source"), "rev-parse", "HEAD"], text=True).strip(),
                    "observerChange": "autoflush partial public SDK observations only; solver unchanged",
                    "files": [{"path": str(p.relative_to(self.out)), "sha256": sha(p)}
                              for p in sorted((self.out / "sample").rglob("*")) if p.is_file()],
                    "serviceApkSha256": sha(REFERENCE / "toolchain/downloads/Google_Play_Services_for_AR_1.56.0_x86_for_emulator.apk"),
                    "accuracy": None, "independentGT": None}
        (self.out / "source-manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
        self.cmd("container-create", ["docker", "run", "-d", "--name", self.container,
                 "--label", "mobile-slam-reference=arcore-isolated-20261013", "--device", "/dev/kvm",
                 "--cpus=4", "--memory=8g", "--shm-size=1g",
                 "-e", "ANDROID_AVD_HOME=/work/toolchain/android-user/avd",
                 "-v", str(self.out) + ":/work",
                 "-v", str(REFERENCE) + ":/reference:ro",
                 "-v", str(REFERENCE / "toolchain/sdk") + ":/work/toolchain/sdk:ro", IMAGE])
        self.owned = True
        self.cmd("container-inspect", ["docker", "inspect", self.container])
        self.docker("cache-copy", ["bash", "-c", "mkdir -p /work/toolchain/gradle; cp -a /reference/toolchain/gradle/caches /reference/toolchain/gradle/wrapper /work/toolchain/gradle/"], timeout=120)
        self.docker("sample-build", ["bash", "-c", "cd /work/sample && JAVA_TOOL_OPTIONS='-XX:ActiveProcessorCount=2 -Xmx2048m' ./gradlew --offline --no-daemon --max-workers=2 assembleDebug"], timeout=360)
        self.docker("java-version", ["java", "-version"])
        self.docker("emulator-version", ["emulator", "-no-window", "-version"])
        self.docker("adb-version", ["adb", "version"])
        self.docker("image-properties", ["cat", "/work/toolchain/sdk/system-images/android-30/google_apis/x86_64/source.properties"])

    def boot(self):
        self.docker("avd-create", ["avdmanager", "create", "avd", "--name", "isolated_arcore_api30", "--package", "system-images;android-30;google_apis;x86_64", "--device", "pixel_3a"], input=b"no\n")
        code = "from pathlib import Path; p=Path('/work/toolchain/android-user/avd/isolated_arcore_api30.avd/config.ini'); s=dict(x.split('=',1) for x in p.read_text().splitlines() if '=' in x); s.update({'hw.camera.back':'virtualscene','hw.camera.front':'none','hw.gpu.enabled':'yes','hw.gpu.mode':'swiftshader','hw.cpu.ncore':'4','hw.ramSize':'2048','hw.lcd.width':'480','hw.lcd.height':'800','hw.lcd.density':'240','disk.dataPartition.size':'3G','showDeviceFrame':'no'}); p.write_text(''.join(k+'='+v+'\\n' for k,v in s.items())); print(p.read_text())"
        self.docker("avd-config", ["python3", "-c", code])
        self.docker("accel-check", ["emulator", "-accel-check"])
        self.cmd("emulator-start", ["docker", "exec", "-d", self.container, "bash", "-c", "exec emulator -avd isolated_arcore_api30 -port 5556 -grpc 8554 -no-window -no-audio -gpu swiftshader -feature -Vulkan -accel on -cores 4 -memory 2048 -no-snapshot -no-boot-anim -camera-back virtualscene > /work/emulator.log 2>&1"])
        deadline = time.monotonic() + 180
        while time.monotonic() < deadline:
            result = subprocess.run(["docker", "exec", self.container, "adb", "-s", SERIAL, "shell", "getprop", "sys.boot_completed"], capture_output=True, timeout=8)
            if result.returncode == 0 and result.stdout.strip() == b"1":
                break
            time.sleep(3)
        else:
            raise RuntimeError("Isolated emulator boot timed out")
        self.adb("device-properties", ["shell", "getprop"])
        self.adb("arcore-install", ["install", "-r", "/reference/toolchain/downloads/Google_Play_Services_for_AR_1.56.0_x86_for_emulator.apk"])
        self.adb("sample-install", ["install", "-r", "-g", "/work/sample/app/build/outputs/apk/debug/app-debug.apk"])
        self.adb("arcore-package", ["shell", "dumpsys", "package", "com.google.ar.core"])
        self.adb("sample-package", ["shell", "dumpsys", "package", PACKAGE])
        self.adb("disable-rotation", ["shell", "settings", "put", "system", "accelerometer_rotation", "0"])
        self.adb("unlock", ["shell", "input", "keyevent", "82"])

    def experiment(self, mode, frames, timeout):
        self.adb(mode + "-stop-before", ["shell", "am", "force-stop", PACKAGE])
        self.adb(mode + "-clear-log", ["logcat", "-c"])
        self.adb(mode + "-start", ["shell", "am", "start", "-n", PACKAGE + "/.HelloArActivity",
                 "--es", "compare_mode", mode, "--ei", "compare_frames", str(frames),
                 "--ez", "compare_save_images", "true"])
        if self.motion and mode in ("live", "record"):
            self.docker(mode + "-motion", ["/reference/toolchain/grpc-python/bin/python", "/work/motion.py"], check=False, timeout=25)
        deadline = time.monotonic() + timeout
        summary = None
        while time.monotonic() < deadline:
            result = subprocess.run(["docker", "exec", self.container, "adb", "-s", SERIAL, "shell", "run-as", PACKAGE, "cat", "files/reference-" + mode + "-summary.json"], capture_output=True, timeout=8)
            if result.returncode == 0:
                try:
                    summary = json.loads(result.stdout)
                    break
                except json.JSONDecodeError:
                    pass
            time.sleep(3)
        for kind in ("poses", "imu"):
            self.adb(mode + "-" + kind, ["exec-out", "run-as", PACKAGE, "cat", "files/reference-" + mode + "-" + kind + ".jsonl"], check=False)
        self.adb(mode + "-image-files", ["shell", "run-as", PACKAGE, "ls", "-l", "files/reference-" + mode + "-images"], check=False)
        self.adb(mode + "-images.tar", ["exec-out", "run-as", PACKAGE, "tar", "-C", "files", "-cf", "-", "reference-" + mode + "-images"], check=False)
        self.adb(mode + "-logcat", ["logcat", "-d"])
        self.adb(mode + "-stop-after", ["shell", "am", "force-stop", PACKAGE])
        result = {"mode": mode, "completed": summary is not None, "sdkSummary": summary, "timeoutS": timeout,
                  "accuracy": None, "independentGT": None, "privateSolverTiming": None}
        (self.out / (mode + "-result.json")).write_text(json.dumps(result, indent=2) + "\n")
        print(json.dumps(result), flush=True)
        return summary

    def close(self):
        # Stop only this newly created container. Preserve all evidence and AVD bytes.
        if not self.owned:
            return
        self.adb("emulator-stop", ["emu", "kill"], check=False, timeout=15)
        self.cmd("container-stop", ["docker", "stop", "-t", "10", self.container], check=False, timeout=20)
        self.cmd("container-final", ["docker", "inspect", self.container], check=False)


def sequence_complete(results):
    """Execution completion only; tracking/accuracy are separate measured gates."""
    return (set(results) == {"live", "record", "playback1", "playback2"}
            and all(summary is not None and summary.get("setupError") is None
                    and summary.get("frames", 0) > 0 for summary in results.values()))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", required=True, type=pathlib.Path)
    parser.add_argument("--container", default="mobile-slam-arcore-20261013-isolated")
    parser.add_argument("--frames", type=int, default=60)
    parser.add_argument("--timeout", type=int, default=90)
    parser.add_argument("--motion", action="store_true", help="Drive only the new emulator public physical model")
    args = parser.parse_args()
    run = Run(args.output, args.container, args.motion)
    results = {}
    try:
        run.initialize()
        run.boot()
        results["live"] = run.experiment("live", args.frames, args.timeout)
        record = run.experiment("record", args.frames, args.timeout)
        results["record"] = record
        if record and record.get("recordingBytes", 0) > 0 and record.get("setupError") is None:
            run.adb("recording.mp4", ["exec-out", "run-as", PACKAGE, "cat", "files/reference-recording.mp4"])
            results["playback1"] = run.experiment("playback1", args.frames, args.timeout)
            results["playback2"] = run.experiment("playback2", args.frames, args.timeout)
    finally:
        run.close()
        (run.out / "execution-result.json").write_text(json.dumps({
            "sdkSequenceComplete": sequence_complete(results), "sdkSummaries": results,
            "trackingAccepted": False, "accuracy": None,
            "gateMeaning": "exit0 requires all four public SDK experiments to finish with nonzero frames and no setup error; not tracking/accuracy acceptance"
        }, indent=2) + "\n")
    if not sequence_complete(results):
        raise SystemExit(1)


if __name__ == "__main__":
    main()

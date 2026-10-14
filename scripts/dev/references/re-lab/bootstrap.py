#!/usr/bin/env python3
"""Install pinned analysis tools locally; no root or product dependency changes."""
import hashlib
import json
import os
import pathlib
import platform
import subprocess
import tarfile
import urllib.request

ROOT = pathlib.Path(__file__).resolve().parents[4]
TOOLS = ROOT / "scripts/dev/tools/re-lab"
PACKAGES = [
    ("radare2_6.2.4_amd64.deb",
     "https://github.com/radareorg/radare2/releases/download/6.2.4/radare2_6.2.4_amd64.deb",
     "7019eedc0e0e87d1f53d6b8f5fc62b898567679c5efdce7fbfc557c9e7655e90"),
    ("wabt-1.0.42-linux-x64.tar.gz",
     "https://github.com/WebAssembly/wabt/releases/download/1.0.42/wabt-1.0.42-linux-x64.tar.gz",
     "84895407a6bbb80e918f33b16b2fb2206021c150b6bc9ff6f761263a745ab131"),
]


def digest(path):
    with path.open("rb") as source:
        return hashlib.file_digest(source, "sha256").hexdigest()


def tool_environment():
    env = os.environ.copy()
    libraries = TOOLS / "radare2/usr/lib"
    env["LD_LIBRARY_PATH"] = str(libraries) + ":" + str(libraries / "x86_64-linux-gnu") + ":" + env.get("LD_LIBRARY_PATH", "")
    env["R2_NOPLUGINS"] = "1"
    return env


def main():
    if platform.system() != "Linux" or platform.machine() not in ("x86_64", "amd64"):
        raise SystemExit("Pinned packages require Linux x86_64")
    downloads = TOOLS / "downloads"
    downloads.mkdir(parents=True, exist_ok=True)
    records = []
    for name, url, expected in PACKAGES:
        archive = downloads / name
        if not archive.exists():
            partial = archive.with_suffix(archive.suffix + ".partial")
            with urllib.request.urlopen(url, timeout=60) as response, partial.open("xb") as output:
                total = 0
                while chunk := response.read(1024 * 1024):
                    total += len(chunk)
                    if total > 32 * 1024 * 1024:
                        raise RuntimeError("Package exceeds download bound")
                    output.write(chunk)
            if digest(partial) != expected:
                raise RuntimeError("Checksum mismatch; partial package preserved: " + str(partial))
            partial.rename(archive)
        if digest(archive) != expected:
            raise RuntimeError("Existing package differs; preserving it: " + str(archive))
        destination = TOOLS / ("radare2" if name.endswith(".deb") else "wabt")
        if not destination.exists():
            staging = destination.with_name(destination.name + ".staging")
            staging.mkdir()
            if name.endswith(".deb"):
                subprocess.run(["dpkg-deb", "-x", str(archive), str(staging)], check=True)
            else:
                with tarfile.open(archive) as package:
                    package.extractall(staging, filter="data")
            staging.rename(destination)
        records.append({"url": url, "path": str(archive.relative_to(ROOT)), "sha256": expected})
    versions = []
    for tool, flag in ((TOOLS / "radare2/usr/bin/radare2", "-v"), (TOOLS / "wabt/wabt-1.0.42/bin/wasm-objdump", "--version")):
        result = subprocess.run([str(tool), flag], env=tool_environment(), capture_output=True, text=True, check=True)
        versions.append({"path": str(tool.relative_to(ROOT)), "sha256": digest(tool), "version": result.stdout.strip()})
    manifest = {"schema": "mobile-slam-re-tools-v1", "packages": records, "tools": versions}
    (TOOLS / "installation.json").write_text(json.dumps(manifest, indent=2) + "\n")
    print(json.dumps(manifest, indent=2))


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""Build the pinned upstream VINS-Mono without editing the existing checkout."""
import argparse
import hashlib
import json
import shutil
import subprocess
from pathlib import Path

ROOT = Path(__file__).resolve().parents[4]
REVISION = "90dabb5ec79946ae42fd2e1e91d4e69aabe1e25d"
IMAGE = "mobile-slam-ref-vins-mono:90dabb5-20261012"


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def stage(output):
    source = ROOT / "assets/references/VINS-Mono"
    revision = subprocess.check_output(
        ["git", "-C", str(source), "rev-parse", "HEAD"], text=True
    ).strip()
    dirty = subprocess.check_output(
        ["git", "-C", str(source), "status", "--porcelain"], text=True
    )
    if revision != REVISION or dirty:
        raise SystemExit("Reference checkout is dirty or has a different revision; preserved unchanged.")
    context = output / "docker-context"
    if context.exists():
        raise SystemExit(f"Refusing to replace existing snapshot: {context}")
    context.mkdir(parents=True)
    shutil.copytree(source, context / "sources/VINS-Mono", ignore=shutil.ignore_patterns(".git"))
    shutil.copy2(Path(__file__).with_name("Dockerfile"), context / "Dockerfile")
    files = {str(p.relative_to(context)): sha(p) for p in sorted(context.rglob("*")) if p.is_file()}
    manifest = {"source": str(source), "revision": revision, "source_dirty": False,
                "image": IMAGE, "files": files, "snapshot_only": True}
    (output / "source-manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
    print(context)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("mode", choices=["stage", "build", "inspect"])
    parser.add_argument("--output", type=Path, default=ROOT / "build/reference-engines/vins-mono")
    args = parser.parse_args()
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    if args.mode == "stage":
        stage(output)
    elif args.mode == "build":
        context = output / "docker-context"
        manifest = json.loads((output / "source-manifest.json").read_text())
        for name, digest in manifest["files"].items():
            if sha(context / name) != digest:
                raise SystemExit(f"Snapshot hash mismatch: {name}")
        command = ["flock", str(ROOT / "build/reference-env-build.lock"),
                   "docker", "build", "--progress=plain", "--tag", IMAGE, str(context)]
        (output / "build-command.json").write_text(json.dumps(command, indent=2) + "\n")
        raise SystemExit(subprocess.call(command))
    else:
        command = ["docker", "image", "inspect", IMAGE]
        result = subprocess.run(command, capture_output=True, text=True)
        (output / "image-inspect.json").write_text(result.stdout or json.dumps({"error": result.stderr}))
        print(result.stdout or result.stderr)
        raise SystemExit(result.returncode)


if __name__ == "__main__":
    main()

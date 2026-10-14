#!/usr/bin/env python3
"""Inspect selected ELF functions without loading or executing the library."""
import argparse
import datetime
import json
import pathlib
import re
import shutil
import subprocess
import time
import zipfile

from bootstrap import TOOLS, digest, tool_environment


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=pathlib.Path, required=True)
    parser.add_argument("--apk-member", help="Exact .so member in a local APK")
    parser.add_argument("--output", type=pathlib.Path, required=True)
    parser.add_argument("--export", action="append", required=True, dest="exports")
    parser.add_argument("--objdump", default="objdump", help="GNU objdump executable, including a target cross-binutils build")
    args = parser.parse_args()
    if any(not re.fullmatch(r"[A-Za-z_][A-Za-z0-9_]*", name) for name in args.exports):
        raise SystemExit("Export must be a simple identifier")
    objdump_path = shutil.which(args.objdump)
    if objdump_path is None:
        parser.error("objdump executable not found: " + args.objdump)
    objdump = pathlib.Path(objdump_path).resolve()
    source, out = args.input.resolve(), args.output.resolve()
    if out.is_relative_to(pathlib.Path("/mnt/backup/SLAM")):
        raise SystemExit("Canonical dataset is read-only")
    out.mkdir(parents=True, exist_ok=False)
    sources = []
    for script in (pathlib.Path(__file__), pathlib.Path(__file__).with_name("bootstrap.py")):
        snapshot = out / script.name
        snapshot.write_bytes(script.read_bytes())
        sources.append({"path": str(script.resolve()), "snapshot": snapshot.name, "sha256": digest(snapshot)})
    if args.apk_member:
        with zipfile.ZipFile(source) as archive:
            if archive.getinfo(args.apk_member).file_size > 128 * 1024 * 1024:
                raise SystemExit("ELF member exceeds bound")
            data = archive.read(args.apk_member)
    else:
        data = source.read_bytes()
    if data[:4] != b"\x7fELF":
        raise SystemExit("ELF input required")
    target = out / "target.so"
    target.write_bytes(data)
    commands = []

    def run(name, argv):
        start = time.monotonic()
        stdout, stderr = out / (name + ".stdout"), out / (name + ".stderr")
        with stdout.open("wb") as so, stderr.open("wb") as se:
            try:
                code = subprocess.run([str(x) for x in argv], env=tool_environment(), stdout=so, stderr=se, timeout=120).returncode
            except subprocess.TimeoutExpired:
                code = None
        commands.append({"name": name, "argv": [str(x) for x in argv], "exit": code, "seconds": time.monotonic() - start,
                         "stdout_sha256": digest(stdout), "stderr_sha256": digest(stderr)})
        (out / "commands.json").write_text(json.dumps(commands, indent=2) + "\n")
        if code != 0 or not stdout.stat().st_size:
            raise SystemExit("Analysis failed or empty; logs preserved: " + name)
        return stdout.read_text()

    r2 = TOOLS / "radare2/usr/bin/radare2"
    symbols = json.loads(run("symbols", [r2, "-q", "-c", "isj", target]))
    run("elf-headers", ["readelf", "-h", "-d", target])
    selected = []
    for name in args.exports:
        symbol = next((s for s in symbols if s["name"] == name and not s.get("is_imported")), None)
        if symbol is None:
            raise SystemExit("Export absent: " + name)
        address = symbol["vaddr"]
        for operation in ("pdf", "agfj", "pdc"):
            run(name + "-" + operation, [r2, "-q", "-e", "scr.color=false", "-c", f"s {address};af;{operation}", target])
        run(name + "-objdump", [objdump, "-d", "--disassemble=" + name, target])
        graph = json.loads((out / (name + "-agfj.stdout")).read_text())
        selected.append({"symbol": symbol, "blocks": len(graph[0]["blocks"]),
                         "instructions": sum(len(b.get("ops", [])) for b in graph[0]["blocks"]),
                         "calls": [op for b in graph[0]["blocks"] for op in b.get("ops", []) if op.get("type") in ("call", "ucall", "icall")]})
    manifest = {"schema": "mobile-slam-elf-functions-v1", "observed_at_utc": datetime.datetime.now(datetime.timezone.utc).isoformat(),
                "input": {"path": str(source), "sha256": digest(source), "member": args.apk_member},
                "elf": {"sha256": digest(target), "bytes": len(data)}, "selected": selected,
                "analysis_sources": sources,
                "tools": [{"path": str(p), "sha256": digest(p)} for p in (r2, objdump, pathlib.Path("/usr/bin/readelf"))],
                "limits": ["Static public entrypoint analysis, not proprietary solver recovery", "No target ELF execution or trajectory/latency measurement"]}
    (out / "summary.json").write_text(json.dumps(manifest, indent=2) + "\n")
    print(json.dumps({"elf": manifest["elf"], "selected": [{"name": f["symbol"]["name"], "bytes": f["symbol"]["size"],
          "blocks": f["blocks"], "instructions": f["instructions"], "calls": len(f["calls"])} for f in selected]}, indent=2))


if __name__ == "__main__":
    main()

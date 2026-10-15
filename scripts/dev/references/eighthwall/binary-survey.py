#!/usr/bin/env python3
"""Read-only, cross-module survey of the official standalone embedded WASM."""
import argparse
import base64
import collections
import datetime
import hashlib
import importlib.util
import json
import pathlib
import re
import subprocess
import sys
import time

RELAB = pathlib.Path(__file__).resolve().parents[1] / "re-lab"
sys.path.insert(0, str(RELAB))
spec = importlib.util.spec_from_file_location("re_lab_analysis", RELAB / "analyze.py")
analysis = importlib.util.module_from_spec(spec)
spec.loader.exec_module(analysis)

MODULES = ("xr.js", "xr-slam.js", "xr-face.js", "resources/media-worker.js", "resources/semantics-worker.js")
SELECTED = {
    "xr": ("getReadyInPipeline", "markTextureFilledInPipeline", "markStageFinishedInPipeline", "releaseInPipeline", "resetOnPipelineStop"),
    "xr-slam": ("configureXr", "setEventQueue", "query", "recenter", "engineCleanup", "prepareYuvBuffer", "processStagedFrame"),
    "xr-face": ("loadFaceDetectModel", "processStagedFaceControllerFrame"),
    "semantics-worker": ("loadSemanticsModel", "processStagedSemanticsWorkerFrame"),
}


def write_json(path, value):
    path.write_text(json.dumps(value, indent=2) + "\n")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", required=True, type=pathlib.Path)
    parser.add_argument("--output", required=True, type=pathlib.Path)
    args = parser.parse_args()
    out = args.output.resolve()
    if out.is_relative_to(pathlib.Path("/mnt/backup/SLAM")):
        parser.error("Canonical dataset is read-only")
    out.mkdir(parents=True, exist_ok=False)
    sources = []
    for src in (pathlib.Path(__file__), RELAB / "analyze.py", RELAB / "bootstrap.py"):
        snapshot = out / src.name
        snapshot.write_bytes(src.read_bytes())
        sources.append({"path": str(src.resolve()), "sha256": analysis.digest(snapshot), "snapshot": snapshot.name})
    commands = []

    def run(directory, name, argv, timeout=120):
        stdout, stderr = directory / (name + ".stdout"), directory / (name + ".stderr")
        start = time.monotonic()
        with stdout.open("wb") as so, stderr.open("wb") as se:
            try:
                result = subprocess.run([str(x) for x in argv], env=analysis.tool_environment(), stdout=so, stderr=se, timeout=timeout)
                code = result.returncode
            except subprocess.TimeoutExpired:
                code = None
        commands.append({"name": str(stdout.relative_to(out)), "argv": list(map(str, argv)), "exit": code,
                         "seconds": time.monotonic() - start, "stdout_sha256": analysis.digest(stdout),
                         "stderr_sha256": analysis.digest(stderr), "stderr_bytes": stderr.stat().st_size})
        write_json(out / "commands.json", commands)
        return code

    for name, argv in (("wabt-version", [analysis.WABT / "wasm-objdump", "--version"]),
                       ("radare-version", [analysis.R2, "-v"])):
        if run(out, name, argv) != 0:
            raise SystemExit("Tool version check failed")
    records = []
    for module in MODULES:
        source = (args.input / module).resolve()
        data = source.read_bytes()
        blobs = [base64.b64decode(m[1], validate=True) for m in re.finditer(rb"data:application/(?:octet-stream|wasm);base64,([A-Za-z0-9+/=]+)", data)]
        blobs = [b for b in blobs if b[:8] == b"\0asm\1\0\0\0"]
        if len(blobs) != 1:
            raise SystemExit(f"Require one embedded module: {source}")
        directory = out / source.stem
        directory.mkdir()
        target = directory / "target.wasm"
        target.write_bytes(blobs[0])
        for name, argv in (("validate", [analysis.WABT / "wasm-validate", "--enable-all", target]),
                           ("sections", [analysis.WABT / "wasm-objdump", "-h", "-x", target]),
                           ("disassembly", [analysis.WABT / "wasm-objdump", "-d", target]),
                           ("symbols", [analysis.R2, "-q", "-c", "isj", target])):
            if run(directory, name, argv) != 0:
                raise SystemExit("Required analysis failed: " + name)
        sections = (directory / "sections.stdout").read_text()
        functions = analysis.parse_disassembly((directory / "disassembly.stdout").read_text())
        symbols = json.loads((directory / "symbols.stdout").read_text())
        types = {int(m[1]): m[2] for m in re.finditer(r" - type\[(\d+)\] (.+)", sections)}
        signatures = {int(m[1]): int(m[2]) for m in re.finditer(r" - func\[(\d+)\] sig=(\d+)", sections)}
        imports = [{"index": int(m[1]), "name": m[3], "signature": types[int(m[2])]} for m in
                   re.finditer(r" - func\[(\d+)\] sig=(\d+).* <- (\S+)", sections)]
        exports = [{"index": int(m[1]), "name": m[2], "signature": types[signatures[int(m[1])]]} for m in
                   re.finditer(r' - func\[(\d+)\].* -> "([^"]+)"', sections)]
        counter = collections.Counter()
        for function in functions.values():
            counter.update(function["opcodes"])
        edges = [{"caller": i, "callee": j, "sites": n} for i, f in functions.items() for j, n in f["calls"].items()]
        write_json(directory / "direct-callgraph.json", edges)
        write_json(directory / "opcodes.json", counter)
        write_json(directory / "imports.json", imports)
        write_json(directory / "exports.json", exports)
        strings = [{"offset": m.start(), "text": m[0].decode("ascii")} for m in re.finditer(rb"[ -~]{8,}", blobs[0])
                   if re.search(rb"camera|calib|gyro|accel|tracking|landmark|keyframe|ceres|jacobian|optim|triang|parallax|relocal|bundle|quaternion|feature|patch|tensorflow|tflite|xnnpack|h264|mp4", m[0], re.I)]
        write_json(directory / "selected-strings.json", strings)
        selected = []
        for suffix in SELECTED.get(source.stem, ()):
            name = "c8EmAsm_" + suffix
            f = next(f for f in functions.values() if f["name"] == name)
            reached = analysis.reachable(functions, f["index"])
            (directory / (suffix + ".wasm.txt")).write_text("\n".join(f["lines"]) + "\n")
            symbol = next(s for s in symbols if s.get("name") in (name, "sym." + name))
            address = symbol["vaddr"]
            for operation in ("agfj", "pdc"):
                run(directory, suffix + "-" + operation, [analysis.R2, "-q", "-e", "scr.color=false", "-c", f"s {address};af;{operation}", target])
            graph = json.loads((directory / (suffix + "-agfj.stdout")).read_text())
            if not graph or not graph[0].get("blocks"):
                raise SystemExit("Empty radare graph: " + name)
            selected.append({"name": name, "index": f["index"], "signature": types[signatures[f["index"]]],
                             "instructions": f["instructions"], "direct_callees": f["calls"],
                             "direct_reachable_count": len(reached), "reachable_defined_count": sum(i in functions for i in reached),
                             "reachable_indirect_sites": sum(functions[i]["indirect_sites"] for i in reached if i in functions),
                             "reachable_imports": [i for i in imports if i["index"] in reached],
                             "radare_blocks": len(graph[0]["blocks"]),
                             "radare_instructions": sum(len(b.get("ops", [])) for b in graph[0]["blocks"])})
        record = {"module": module, "source": str(source), "source_sha256": hashlib.sha256(data).hexdigest(),
                  "wasm_bytes": len(blobs[0]), "wasm_sha256": analysis.digest(target),
                  "section_headers": [s.strip() for s in sections.splitlines() if " start=0x" in s],
                  "memory_table": [s.strip() for s in sections.splitlines() if re.search(r" - (?:memory\[\d+\] pages|table\[\d+\] type)", s)],
                  "defined_functions": len(functions), "imported_functions": len(imports), "function_exports": len(exports),
                  "instructions": sum(counter.values()), "simd_instructions": sum(v for k, v in counter.items() if re.match(r"(?:v128|[if]\d+x\d+)", k)),
                  "atomic_instructions": sum(v for k, v in counter.items() if "atomic" in k),
                  "direct_edges": len(edges), "indirect_sites": sum(f["indirect_sites"] for f in functions.values()),
                  "selected_strings": len(strings), "selected": selected}
        write_json(directory / "summary.json", record)
        records.append(record)
        print(json.dumps({k: v for k, v in record.items() if k not in ("selected", "section_headers")}), flush=True)
    manifest = {"schema": "8thwall-cross-module-survey-v1", "observed_at_utc": datetime.datetime.now(datetime.timezone.utc).isoformat(),
                "sources": sources, "tools": json.loads((analysis.TOOLS / "installation.json").read_text()), "modules": records,
                "failed_commands": [c for c in commands if c["exit"] != 0],
                "limits": ["Direct call graph excludes unresolved indirect edges and JS-mediated cross-module calls",
                           "Strings/exports indicate interfaces or compiled material, not runtime use or algorithm identity",
                           "No camera input or target code execution in this static survey; no accuracy/performance result"]}
    write_json(out / "summary.json", manifest)
    if manifest["failed_commands"]:
        raise SystemExit(1)


if __name__ == "__main__":
    main()

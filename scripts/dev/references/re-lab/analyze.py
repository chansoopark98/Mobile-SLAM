#!/usr/bin/env python3
"""Read-only WASM function analysis with bounded native tools and provenance."""
import argparse
import base64
import collections
import datetime
import hashlib
import json
import pathlib
import re
import subprocess
import time

from bootstrap import ROOT, TOOLS, digest, tool_environment

WABT = TOOLS / "wabt/wabt-1.0.42/bin"
R2 = TOOLS / "radare2/usr/bin/radare2"


def parse_disassembly(text):
    functions = {}
    current = None
    for line in text.splitlines():
        match = re.match(r"\s*([0-9a-f]+) func\[(\d+)\](?: <([^>]+)>)?:", line)
        if match:
            current = int(match[2])
            functions[current] = {"index": current, "offset": int(match[1], 16), "name": match[3], "instructions": 0,
                                  "calls": collections.Counter(), "indirect_sites": 0, "opcodes": collections.Counter(), "lines": []}
        instruction = re.search(r"\|\s+([\w.]+)(?:\s+(.*))?$", line)
        if current is not None:
            functions[current]["lines"].append(line)
        if instruction and current is not None:
            function = functions[current]
            opcode, operands = instruction.groups()
            function["instructions"] += 1
            function["opcodes"][opcode] += 1
            if opcode in ("call", "return_call"):
                target = re.match(r"(\d+)", operands or "")
                if target:
                    function["calls"][int(target[1])] += 1
            elif opcode in ("call_indirect", "return_call_indirect"):
                function["indirect_sites"] += 1
    return functions


def reachable(functions, start):
    seen, pending = set(), [start]
    while pending:
        index = pending.pop()
        if index in seen:
            continue
        seen.add(index)
        pending.extend(functions.get(index, {}).get("calls", {}))
    return seen


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=pathlib.Path, required=True, help="Local WASM or JS with exactly one embedded WASM")
    parser.add_argument("--output", type=pathlib.Path, required=True, help="New directory; existing evidence is preserved")
    parser.add_argument("--export", action="append", required=True, dest="exports")
    parser.add_argument("--timeout", type=int, default=120)
    args = parser.parse_args()
    if any(not re.fullmatch(r"[A-Za-z_][A-Za-z0-9_]*", name) for name in args.exports):
        raise SystemExit("Export must be a simple identifier")
    source = args.input.resolve()
    data = source.read_bytes()
    if data[:8] != b"\0asm\1\0\0\0":
        modules = [base64.b64decode(match[1], validate=True) for match in re.finditer(rb"data:application/(?:octet-stream|wasm);base64,([A-Za-z0-9+/=]+)", data)]
        modules = [item for item in modules if item[:8] == b"\0asm\1\0\0\0"]
        if len(modules) != 1:
            raise SystemExit("Require exactly one embedded WASM; select the desired module explicitly")
        wasm = modules[0]
    else:
        wasm = data
    out = args.output.resolve()
    if out.is_relative_to(pathlib.Path("/mnt/backup/SLAM")):
        raise SystemExit("Canonical dataset is read-only")
    out.mkdir(parents=True, exist_ok=False)
    sources = []
    for script in (pathlib.Path(__file__), pathlib.Path(__file__).with_name("bootstrap.py")):
        snapshot = out / script.name
        snapshot.write_bytes(script.read_bytes())
        sources.append({"path": str(script.resolve()), "snapshot": snapshot.name, "sha256": digest(snapshot)})
    target = out / "target.wasm"
    target.write_bytes(wasm)
    commands = []

    def run(name, argv):
        start = time.monotonic()
        stdout, stderr = out / (name + ".stdout"), out / (name + ".stderr")
        with stdout.open("wb") as so, stderr.open("wb") as se:
            try:
                result = subprocess.run([str(x) for x in argv], env=tool_environment(), stdout=so, stderr=se, timeout=args.timeout)
                code, timed_out = result.returncode, False
            except subprocess.TimeoutExpired:
                code, timed_out = None, True
        commands.append({"name": name, "argv": [str(x) for x in argv], "exit": code, "timed_out": timed_out,
                         "seconds": time.monotonic() - start, "stdout_bytes": stdout.stat().st_size, "stderr_bytes": stderr.stat().st_size,
                         "stdout_sha256": digest(stdout), "stderr_sha256": digest(stderr)})
        (out / "commands.json").write_text(json.dumps(commands, indent=2) + "\n")
        return code

    required = [("validate", [WABT / "wasm-validate", "--enable-all", target]),
                ("sections", [WABT / "wasm-objdump", "-h", "-x", target]),
                ("disassembly", [WABT / "wasm-objdump", "-d", target]),
                ("r2-symbols", [R2, "-q", "-c", "isj", target])]
    for name, argv in required:
        if run(name, argv) != 0:
            raise SystemExit("Required analysis failed; logs preserved: " + name)
    functions = parse_disassembly((out / "disassembly.stdout").read_text())
    symbols = json.loads((out / "r2-symbols.stdout").read_text())
    selected = []
    missing = []
    for name in args.exports:
        function = next((f for f in functions.values() if f["name"] == name), None)
        if function is None:
            missing.append(name)
            continue
        index = function["index"]
        (out / (name + ".wasm-disassembly.txt")).write_text("\n".join(function["lines"]) + "\n")
        symbol = next((s for s in symbols if s.get("name") in (name, "sym." + name)), None)
        address = symbol.get("vaddr") if symbol else None
        if address is None:
            raise SystemExit("Radare2 symbol absent; logs preserved: " + name)
        if address is not None:
            # Do not run recursive/global analysis on the whole binary.
            for operation in ("pdf", "agfj", "pdc"):
                run(name + "-" + operation, [R2, "-q", "-e", "scr.color=false",
                    "-c", f"s {address};af;{operation}", target])
                if not (out / (name + "-" + operation + ".stdout")).stat().st_size:
                    raise SystemExit("Empty function analysis; logs preserved: " + name + "-" + operation)
        reached = reachable(functions, index)
        graph = json.loads((out / (name + "-agfj.stdout")).read_text()) if address is not None else []
        if not graph or not graph[0].get("blocks") or not any(b.get("ops") for b in graph[0]["blocks"]):
            raise SystemExit("Empty function graph; logs preserved: " + name)
        r2_ops = sum(len(b.get("ops", [])) for b in graph[0]["blocks"]) if graph else None
        selected.append({k: v for k, v in function.items() if k != "lines"} | {
            "radare_symbol": symbol, "reachable_direct_functions": len(reached),
            "reachable_defined_functions": sum(i in functions for i in reached),
            "reachable_indirect_sites": sum(functions[i]["indirect_sites"] for i in reached if i in functions),
            "radare_blocks": len(graph[0]["blocks"]) if graph else None, "radare_instructions": r2_ops,
            "radare_instruction_count_matches_wabt": r2_ops == function["instructions"]})
    lowered_ok = run("wasm2c", [WABT / "wasm2c", "-o", out / "lowered.c", target]) == 0
    if lowered_ok:
        lowered = (out / "lowered.c").read_text()
        for function in selected:
            name = function["name"]
            match = re.search(r"^void w2c_target_" + re.escape(name) + r"_0\(", lowered, re.M)
            if match:
                end = lowered.index("\n}\n", match.start()) + 3
                excerpt = out / (name + ".lowered.c")
                excerpt.write_text(lowered[match.start():end])
                function["lowered_excerpt"] = {"path": excerpt.name, "sha256": digest(excerpt)}
    edges = [{"caller": i, "callee": j, "sites": count} for i, f in functions.items() for j, count in f["calls"].items()]
    (out / "direct-callgraph.json").write_text(json.dumps(edges, indent=2) + "\n")
    summary = {"schema": "mobile-slam-wasm-functions-v1", "observed_at_utc": datetime.datetime.now(datetime.timezone.utc).isoformat(),
               "input": {"path": str(source), "bytes": len(data), "sha256": hashlib.sha256(data).hexdigest()},
               "wasm": {"bytes": len(wasm), "sha256": digest(target)},
               "tool_manifest": json.loads((TOOLS / "installation.json").read_text()), "analysis_sources": sources,
               "defined_functions": len(functions), "direct_edges": len(edges),
               "indirect_sites": sum(f["indirect_sites"] for f in functions.values()), "selected": selected, "missing_exports": missing,
               "failed_commands": [c["name"] for c in commands if c["exit"] != 0],
               "limits": ["Direct reachability is a lower bound; indirect calls are unresolved", "Radare2 may omit instructions/branches; compare counts and diagnostics with WABT",
                          "Radare2 pseudo-disassembly and wasm2c lowering are not original source recovery",
                          "No target code executed, private solver semantics or runtime performance established"]}
    (out / "summary.json").write_text(json.dumps(summary, indent=2) + "\n")
    print(json.dumps({k: v for k, v in summary.items() if k not in ("selected", "tool_manifest")}, indent=2))
    if missing or summary["failed_commands"]:
        raise SystemExit(1)


if __name__ == "__main__":
    main()

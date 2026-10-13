#!/usr/bin/env python3
"""Extract explicit embedded Wasm and run verified WABT without JS execution."""
import base64
import collections
import gzip
import hashlib
import json
import pathlib
import re
import subprocess

ROOT = pathlib.Path(__file__).resolve().parents[4]
ASSETS = ROOT / "assets/references/8thwall/analysis-runtime/standalone"
OUT = ROOT / "build/research-20261012/8thwall-binary"
WABT = ROOT / "build/research-20261012/commercial/wabt/wabt-1.0.42/bin"
OUT.mkdir(parents=True, exist_ok=True)
modules = []
for name in ("xr.js", "xr-slam.js", "xr-face.js", "resources/media-worker.js", "resources/semantics-worker.js"):
    path = ASSETS / name
    text = path.read_text()
    matches = list(re.finditer(r"data:application/(?:octet-stream|wasm);base64,([A-Za-z0-9+/=]+)", text))
    for i, match in enumerate(matches):
        data = base64.b64decode(match[1], validate=True)
        if data[:8] != b"\x00asm\x01\x00\x00\x00":
            continue
        prefix = pathlib.Path(name).stem + "-" + str(i)
        wasm = OUT / (prefix + ".wasm")
        wasm.write_bytes(data)
        commands = []
        for operation, args in (("validate", ["wasm-validate", "--enable-all", wasm]), ("sections", ["wasm-objdump", "-h", "-x", wasm])):
            argv = [str(WABT / args[0])] + [str(x) for x in args[1:]]
            result = subprocess.run(argv, text=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            (OUT / (prefix + "-" + operation + ".txt")).write_text(result.stdout + result.stderr)
            commands.append({"argv": argv, "exit_code": result.returncode})
            if result.returncode:
                raise RuntimeError("WABT failed: " + operation)
        ascii_strings = [m.group().decode("ascii") for m in re.finditer(rb"[ -~]{6,}", data)]
        wanted = [s for s in ascii_strings if re.search(r"camera|calib|sensor|gyro|accel|tracking|landmark|keyframe|ceres|jacobian|optim|triang|parallax|relocal|bundle|quaternion|feature|patch", s, re.I)]
        (OUT / (prefix + "-selected-strings.json")).write_text(json.dumps(wanted, indent=2) + "\n")
        info = {"parent_js": str(path.relative_to(ROOT)), "parent_sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
                "data_uri_start": match.start(), "base64_chars": len(match[1]), "wasm": str(wasm.relative_to(ROOT)),
                "bytes": len(data), "sha256": hashlib.sha256(data).hexdigest(), "commands": commands,
                "selected_string_count": len(wanted)}
        if name in ("xr-slam.js", "xr.js"):
            counter = collections.Counter()
            calls = collections.Counter()
            current = None
            process = subprocess.Popen([str(WABT / "wasm-objdump"), "-d", str(wasm)], stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
            with gzip.open(OUT / (prefix + "-disassembly.txt.gz"), "wt") as record:
                for line in process.stdout:
                    record.write(line)
                    function = re.match(r"\s*[0-9a-f]+\s+func\[(\d+)\].*:\s*$", line)
                    if function:
                        current = int(function[1])
                    instruction = re.search(r"\|\s+([a-zA-Z0-9_.]+)(?:\s+(.*))?$", line)
                    if instruction:
                        counter[instruction[1]] += 1
                        if instruction[1] == "call" and current is not None:
                            target = re.match(r"(\d+)", instruction[2] or "")
                            if target:
                                calls[(current, int(target[1]))] += 1
            error = process.stderr.read()
            process.wait()
            if process.returncode:
                raise RuntimeError(error)
            info.update(instruction_count=sum(counter.values()), simd_instructions=sum(v for k, v in counter.items() if re.match(r"(?:v128|[if]\d+x\d+)", k)),
                        atomic_instructions=sum(v for k, v in counter.items() if "atomic" in k), direct_call_edges=len(calls),
                        disassembly_exit=process.returncode)
            (OUT / (prefix + "-opcodes.json")).write_text(json.dumps(counter, indent=2) + "\n")
            (OUT / (prefix + "-direct-callgraph.json")).write_text(json.dumps([{"caller": a, "callee": b, "sites": n} for (a, b), n in calls.items()], indent=2) + "\n")
        modules.append(info)
summary = {"schema": "8thwall-embedded-wasm-inspection-v1", "modules": modules,
           "tool": {"version": "WABT1.0.42", "wasm_objdump_sha256": hashlib.sha256((WABT / "wasm-objdump").read_bytes()).hexdigest()},
           "limits": ["Static call edges are direct-call sites, not runtime execution", "Export/string names do not establish recovered optimizer mathematics", "No proprietary source reconstructed or product implementation changed"]}
(OUT / "wasm-inspection.json").write_text(json.dumps(summary, indent=2) + "\n")
print(json.dumps(summary, indent=2))

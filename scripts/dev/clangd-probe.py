#!/usr/bin/env python3
"""Probe genuine clangd LSP on one C++ source with its existing compile database."""
import argparse
import json
import os
from pathlib import Path
import selectors
import subprocess
import time

ROOT = Path(__file__).resolve().parents[2]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--compile-db", type=Path, default=ROOT / "build/dev-native")
    parser.add_argument("--file", type=Path, default=ROOT / "src/vio_engine.cpp")
    parser.add_argument("--output", type=Path, default=ROOT / "build/dev-tools/clangd-probe.json")
    args = parser.parse_args()
    database = args.compile_db.resolve()
    source = args.file.resolve()
    if not (database / "compile_commands.json").is_file():
        raise SystemExit(f"Missing compile database: {database}")
    clangd = os.environ.get("MOBILE_SLAM_CLANGD", str(ROOT / "scripts/dev/tools/clangd_23.1.0/bin/clangd"))
    args.output.parent.mkdir(parents=True, exist_ok=True)
    report = {"status": "BLOCKED", "source": str(source), "compile_database": str(database), "diagnostics": None}
    with args.output.with_suffix(".log").open("w") as log:
        process = subprocess.Popen([clangd, f"--compile-commands-dir={database}",
                                    "--query-driver=/usr/bin/c++,/usr/bin/g++", "--background-index=0", "--log=info"],
                                   stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=log, cwd=ROOT)
        selector = selectors.DefaultSelector()
        selector.register(process.stdout, selectors.EVENT_READ)
        buffered = bytearray()

        def send(payload):
            content = json.dumps(payload).encode()
            process.stdin.write(f"Content-Length: {len(content)}\r\n\r\n".encode() + content)
            process.stdin.flush()

        def receive(deadline):
            while time.monotonic() < deadline:
                boundary = buffered.find(b"\r\n\r\n")
                if boundary >= 0:
                    headers = buffered[:boundary].decode()
                    size = int(next(line.split(":", 1)[1] for line in headers.split("\r\n")
                                    if line.lower().startswith("content-length:")))
                    start = boundary + 4
                    if len(buffered) >= start + size:
                        message = json.loads(buffered[start:start + size])
                        del buffered[:start + size]
                        return message
                if selector.select(min(1, max(0, deadline - time.monotonic()))):
                    chunk = os.read(process.stdout.fileno(), 65536)
                    if not chunk:
                        raise RuntimeError("clangd exited before completing the probe")
                    buffered.extend(chunk)
            raise TimeoutError("clangd probe exceeded its deadline")

        def wait_for(request_id, deadline):
            while True:
                message = receive(deadline)
                if message.get("method") == "textDocument/publishDiagnostics" and message["params"]["uri"] == source.as_uri():
                    report["diagnostics"] = message["params"]["diagnostics"]
                if message.get("id") == request_id:
                    if "error" in message:
                        raise RuntimeError(str(message["error"]))
                    return message.get("result")

        try:
            deadline = time.monotonic() + 60
            send({"jsonrpc": "2.0", "id": 1, "method": "initialize", "params": {
                "processId": os.getpid(), "rootUri": ROOT.as_uri(),
                "capabilities": {"textDocument": {"documentSymbol": {"hierarchicalDocumentSymbolSupport": True}}},
            }})
            initialization = wait_for(1, deadline)
            report["server"] = initialization.get("serverInfo")
            send({"jsonrpc": "2.0", "method": "initialized", "params": {}})
            send({"jsonrpc": "2.0", "method": "textDocument/didOpen", "params": {"textDocument": {
                "uri": source.as_uri(), "languageId": "cpp", "version": 1, "text": source.read_text(),
            }}})
            send({"jsonrpc": "2.0", "id": 2, "method": "textDocument/documentSymbol",
                  "params": {"textDocument": {"uri": source.as_uri()}}})
            symbols = wait_for(2, deadline)
            report["symbol_count"] = len(symbols or [])
            report["symbols"] = [item["name"] for item in (symbols or [])][:20]
            while report["diagnostics"] is None:
                message = receive(deadline)
                if message.get("method") == "textDocument/publishDiagnostics" and message["params"]["uri"] == source.as_uri():
                    report["diagnostics"] = message["params"]["diagnostics"]
            report["error_count"] = sum(item.get("severity") == 1 for item in report["diagnostics"])
            if not report["symbol_count"] or report["error_count"]:
                raise RuntimeError("No C++ symbols or compiler diagnostics contain errors")
            report["status"] = "VERIFIED"
            send({"jsonrpc": "2.0", "id": 3, "method": "shutdown", "params": None})
            wait_for(3, deadline)
            send({"jsonrpc": "2.0", "method": "exit", "params": None})
            process.wait(timeout=5)
        except (OSError, RuntimeError, TimeoutError, subprocess.TimeoutExpired) as error:
            report["error"] = str(error)
        finally:
            selector.close()
            if process.poll() is None:
                process.terminate()
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait(timeout=5)
    args.output.write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps({key: report.get(key) for key in ("status", "server", "symbol_count", "error_count", "error")}, indent=2))
    if report["status"] != "VERIFIED":
        raise SystemExit(1)


if __name__ == "__main__":
    main()

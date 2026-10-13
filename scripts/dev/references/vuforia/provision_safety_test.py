#!/usr/bin/env python3
"""Fabricated ZIP inputs exercise refusal paths; never fabricate SDK headers/libs."""
import importlib.util
import json
import os
import pathlib
import tempfile
import zipfile

BASE = pathlib.Path(__file__).resolve().parent
EVIDENCE = pathlib.Path(os.environ.get("VUFORIA_EVIDENCE_DIR", BASE.parents[3] / "build/research-20261012/vuforia/environment"))
spec = importlib.util.spec_from_file_location("vuforia_reference", BASE / "reference.py")
reference = importlib.util.module_from_spec(spec)
spec.loader.exec_module(reference)
fixtures = EVIDENCE / "provision-fixtures"
fixtures.mkdir(parents=True, exist_ok=True)
results = []
with tempfile.TemporaryDirectory(prefix="provision-check-", dir=EVIDENCE) as directory:
    work = pathlib.Path(directory)
    assets = work / "assets"
    assets.mkdir()
    for name, entry, expected_reason in [
        ("wrong-checksum", "readme.txt", "SDK_CHECKSUM_MISMATCH"),
        ("path-traversal", "../escape.txt", "SDK_UNSAFE_ARCHIVE"),
        ("not-an-sdk", "readme.txt", "SDK_NOT_PROVISIONED"),
    ]:
        archive = fixtures / (name + ".zip")
        with zipfile.ZipFile(archive, "w") as package:
            package.writestr(zipfile.ZipInfo(entry, date_time=(1980, 1, 1, 0, 0, 0)), b"Independent refusal fixture; no SDK code\n")
        expected_hash = "0" * 64 if name == "wrong-checksum" else reference.sha(archive)
        try:
            reference.provision(archive, expected_hash, assets)
        except ValueError as exc:
            assert str(exc).startswith(expected_reason), str(exc)
            results.append({"name": name, "archive": str(archive), "sha256": reference.sha(archive), "reason": str(exc)})
        else:
            raise AssertionError("Unsafe/non-SDK input was accepted: " + name)
        assert not (work / "escape.txt").exists()
out = {"schema": "vuforia-provision-failure-fixtures-v1", "checks": len(results), "cases": results,
       "sdk_archives_used": False, "scope": "Fabricated ZIP refusal inputs; no SDK stubs or licenses accepted"}
(EVIDENCE / "provision-failures.json").write_text(json.dumps(out, indent=2) + "\n")
print(json.dumps(out, indent=2))

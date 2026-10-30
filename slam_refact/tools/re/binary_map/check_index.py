#!/usr/bin/env python3
"""Consistency checks for function-index.json (exit 1 on failure).

Usage: python3 -I check_index.py --index function-index.json --wasm xr-slam.wasm
"""

import argparse
import collections
import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import rules  # noqa: E402
import wasm_parse  # noqa: E402


def main():
  ap = argparse.ArgumentParser()
  ap.add_argument("--index", required=True)
  ap.add_argument("--wasm", required=True)
  args = ap.parse_args()
  with open(args.index) as fh:
    idx = json.load(fh)
  mod = wasm_parse.Module(args.wasm)
  fs = idx["functions"]
  errors = []
  if mod.sha256 != idx["summary"]["wasm_sha256"]:
    errors.append("sha256 mismatch")
  if [e["index"] for e in fs] != list(range(mod.num_funcs)):
    errors.append("index list is not 0..%d" % (mod.num_funcs - 1))
  n_def = sum(1 for e in fs if e["index"] >= mod.num_imported_funcs)
  if n_def != 5455:
    errors.append("defined functions %d != 5455" % n_def)
  sizes = {i: be - bs for i, bs, be, _ in mod.bodies}
  missing_c = 0
  prev_end = 0
  for e in fs:
    if e["class"] not in rules.CLASSES:
      errors.append("bad class %s at %d" % (e["class"], e["index"]))
    if e["label"] not in ("VERIFIED", "INFERRED", "UNVERIFIED"):
      errors.append("bad label at %d" % e["index"])
    if e["class"] != "unknown" and not e["classified_by"]:
      errors.append("classified without method at %d" % e["index"])
    if e["class"] == "unknown" and e["label"] != "UNVERIFIED":
      errors.append("unknown not UNVERIFIED at %d" % e["index"])
    if e["index"] >= mod.num_imported_funcs:
      if e["size"] != sizes[e["index"]]:
        errors.append("size mismatch at %d" % e["index"])
      dl = e.get("dcmp_lines")
      if not dl or dl[0] <= prev_end or dl[1] < dl[0]:
        errors.append("dcmp range not monotonic at %d" % e["index"])
      else:
        prev_end = dl[1]
    if e.get("ghidra_c") and not os.path.exists(e["ghidra_c"]):
      errors.append("ghidra_c missing file at %d" % e["index"])
    if e["index"] >= mod.num_imported_funcs and not e.get("ghidra_c"):
      missing_c += 1
  counts = collections.Counter(e["class"] for e in fs if e["index"] >= mod.num_imported_funcs)
  print(json.dumps({"defined": n_def, "classes": dict(counts), "defined_without_ghidra_c": missing_c,
                    "errors": errors[:20], "n_errors": len(errors)}, indent=1))
  sys.exit(1 if errors else 0)


if __name__ == "__main__":
  main()

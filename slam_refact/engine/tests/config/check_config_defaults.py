#!/usr/bin/env python3
"""Compare the compiled config defaults (sr_config_dump) with config-defaults.json.

Independent of the generator: reads the JSON directly and checks, for every unique key,
  * a typed key (some entry with getter 458/851/881/1131 and one typed literal) is dumped with
    the getter's type and exactly the literal value (float/double compared bit-exactly,
    i32 wrapped to int32, bool = literal != 0);
  * a key without such an entry is dumped as 'unrecovered';
  * the dump has no extra keys, and the unique key count is 211 (architecture.md §3.1 [M9]);
  * config_defaults.inc records the sha256 of this JSON.
Exit status 0 = PASS.
"""
import argparse
import hashlib
import json
import re
import struct
import subprocess
import sys

GETTER = {458: ("float", "f32"), 851: ("int", "i32"), 881: ("bool", "i32"), 1131: ("double", "f64")}


def expected_value(kind, lit):
    if kind == "float":
        return "%08x" % struct.unpack("<I", struct.pack("<f", lit))[0]
    if kind == "double":
        return "%016x" % struct.unpack("<Q", struct.pack("<d", lit))[0]
    if kind == "bool":
        return "1" if int(lit) != 0 else "0"
    v = int(lit)
    return str(v - 2**32 if v >= 2**31 else v)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--json", required=True)
    ap.add_argument("--inc", required=True)
    ap.add_argument("--dump", required=True)
    a = ap.parse_args()

    raw = open(a.json, "rb").read()
    sha = hashlib.sha256(raw).hexdigest()
    entries = json.loads(raw)["entries"]
    errors = []

    m = re.search(r"sha256 ([0-9a-f]{64})", open(a.inc).read())
    if not m or m.group(1) != sha:
        errors.append(f"config_defaults.inc input sha256 {m.group(1) if m else None} != json {sha}")

    expected = {}
    for e in entries:
        expected.setdefault(e["key"], set())
        if e.get("getter") in GETTER and len(e["args"]) == 1 and e["args"][0][0] in ("i32", "f32", "f64"):
            kind, want = GETTER[e["getter"]]
            if e["args"][0][0] != want:
                errors.append(f"{e['key']}: literal type {e['args'][0][0]} vs getter {e['getter']}")
                continue
            expected[e["key"]].add((kind, expected_value(kind, e["args"][0][1])))

    out = subprocess.run([a.dump], check=True, capture_output=True, text=True).stdout
    dumped = {}
    for line in out.splitlines():
        key, kind, value = line.split("\t")
        if key in dumped:
            errors.append(f"duplicate key in dump: {key}")
        dumped[key] = (kind, value)

    if len(expected) != 211:
        errors.append(f"json unique keys {len(expected)} != 211")
    for key in sorted(set(dumped) - set(expected)):
        errors.append(f"dumped key not in json: {key}")
    n_typed = n_unrec = 0
    for key in sorted(expected):
        got = dumped.get(key)
        if got is None:
            errors.append(f"missing key: {key}")
            continue
        want = expected[key]
        if not want:
            n_unrec += 1
            if got[0] != "unrecovered":
                errors.append(f"{key}: json has no typed default but dump has {got}")
            continue
        n_typed += 1
        if len(want) != 1:
            errors.append(f"{key}: conflicting json defaults {sorted(want)}")
        elif got != next(iter(want)):
            errors.append(f"{key}: dump {got} != json {next(iter(want))}")

    print(f"json sha256 {sha}; keys {len(expected)} (typed {n_typed}, unrecovered {n_unrec}); "
          f"dumped {len(dumped)}")
    for err in errors:
        print("FAIL:", err)
    print("PASS" if not errors else f"FAIL ({len(errors)} errors)")
    return 0 if not errors else 1


if __name__ == "__main__":
    sys.exit(main())

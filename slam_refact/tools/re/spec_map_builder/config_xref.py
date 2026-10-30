"""Cross-reference config globals (from annotate.py's config-globals.json) to reading sites.

For every key whose class prefix is selected, list the functions (other than the getter itself)
whose Ghidra C mentions the global, with line numbers, and whether the raw wasm code contains
the address as a constant (catches base+offset accesses the decompiler may hide).

Usage (cwd = repo root):
  python3 -I slam_refact/tools/re/spec_map_builder/config_xref.py \
      --wasm build/slam-refact-20261015/inputs/xr-slam.wasm \
      --cdir build/slam-refact-20261015/re/ghidra/export/c \
      --globals build/slam-refact-20261015/spec/map-builder/annotated/config-globals.json \
      --defaults build/rea-analysis-20261015/xr_reference/config-defaults.json \
      --prefix MapBuilder BundleAdjustment EpipolarMatcher WorldMap. \
      --out build/slam-refact-20261015/spec/map-builder/config-xref.json
"""

import argparse
import glob
import json
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "binary_map"))
import wasm_parse  # noqa: E402


def main():
  ap = argparse.ArgumentParser()
  ap.add_argument("--wasm", required=True)
  ap.add_argument("--cdir", required=True)
  ap.add_argument("--globals", required=True)
  ap.add_argument("--defaults", required=True)
  ap.add_argument("--prefix", nargs="+", required=True)
  ap.add_argument("--out", required=True)
  a = ap.parse_args()
  glb = json.load(open(a.globals))
  defaults = {}
  for e in json.load(open(a.defaults))["entries"]:
    defaults.setdefault(e["key"], e)
  mod = wasm_parse.Module(a.wasm)
  feats = wasm_parse.extract_features(mod)
  const_users = {}
  for idx, f in feats.items():
    for addr in set(f["addrs"]) | set(f["load_offsets"]):
      const_users.setdefault(addr, set()).add(idx)
  files = {}
  for path in glob.glob(os.path.join(a.cdir, "*.c")):
    m = re.search(r"_unnamed_function_(\d+)\.c$", path)
    if m:
      files[int(m.group(1))] = path
  sel = {addr: v for addr, v in glb.items() if any(v["key"].startswith(p) for p in a.prefix)}
  pats = {addr: re.compile(r"DAT_ram_%s\b" % addr) for addr in sel}
  reads = {addr: {} for addr in sel}
  for n, path in files.items():
    lines = open(path, errors="replace").read().split("\n")
    for addr, pat in pats.items():
      if n == sel[addr]["func"]:
        continue
      hits = [i + 1 for i, ln in enumerate(lines) if pat.search(ln)]
      if hits:
        reads[addr][n] = hits
  out = []
  covered = set()
  for addr, v in sorted(sel.items(), key=lambda kv: kv[0]):
    covered.add(v["key"])
    d = defaults.get(v["key"], {})
    out.append({
      "key": v["key"], "global": "0x" + addr, "type": v["type"], "getter_func": v["func"],
      "default": d.get("args", [[None, None]])[0][1],
      "ghidra_reads": reads[addr],
      "wasm_const_users": sorted(x for x in const_users.get(int(addr, 16), set()) if x != v["func"]),
    })
  missing = sorted(k for k in defaults if any(k.startswith(p) for p in a.prefix) and k not in covered)
  json.dump({"keys": out, "keys_without_global": missing}, open(a.out, "w"), indent=1)
  for r in out:
    rd = ", ".join("%d:%s" % (f, ",".join(map(str, l[:6]))) for f, l in r["ghidra_reads"].items()) or "-"
    print("%-62s %-6s %-12s getter=%s reads=[%s] const=%s" % (r["key"], r["type"], r["default"], r["getter_func"], rd, r["wasm_const_users"]))
  print("keys without global:", missing)


if __name__ == "__main__":
  main()

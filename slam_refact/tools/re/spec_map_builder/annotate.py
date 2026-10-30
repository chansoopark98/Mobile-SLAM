"""Annotate Ghidra C exports of xr-slam.wasm for the MapBuilder spec (lane S3).

Adds trailing comments to each line:
  cfg:<Key>      for DAT_ram_<addr> globals written by a config getter (458/851/881/1131)
  vt:<Type>      for integer constants equal to an RTTI vtable address point
  fN=<module>    for unnamed_function_N calls (class/module from function-index.json)

Usage (cwd = repo root):
  python3 -I slam_refact/tools/re/spec_map_builder/annotate.py \
      --wasm build/slam-refact-20261015/inputs/xr-slam.wasm \
      --index build/slam-refact-20261015/re/function-index.json \
      --cdir build/slam-refact-20261015/re/ghidra/export/c \
      --out build/slam-refact-20261015/spec/map-builder/annotated  936 1112 ...
Writes <out>/<N>.c per function and <out>/config-globals.json (all getter globals found).
"""

import argparse
import glob
import json
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "binary_map"))
import evidence  # noqa: E402

GETTERS = {"458": "f32", "851": "i32", "881": "bool", "1131": "f64"}
GET_RE = re.compile(r"(\w+) = (?:\([^)]*\))?unnamed_function_(458|851|881|1131)\(s_\w+?_ram_([0-9a-f]{8}),")
ASSIGN_RE = re.compile(r"(DAT_ram_[0-9a-f]{8}|uRam[0-9a-f]{8}) = (?:\([^)]*\))?(\w+);")
DAT_RE = re.compile(r"DAT_ram_([0-9a-f]{8})")
CALL_RE = re.compile(r"unnamed_function_(\d+)\(")
HEX_RE = re.compile(r"\b0x([0-9a-f]{3,8})\b")


def config_globals(cdir, ev):
  """Scan every Ghidra C file for getter calls and the globals they are stored into."""
  out = {}
  for path in glob.glob(os.path.join(cdir, "*.c")):
    src = open(path, errors="replace").read()
    if "unnamed_function_458(s_" not in src and "unnamed_function_851(s_" not in src and \
       "unnamed_function_881(s_" not in src and "unnamed_function_1131(s_" not in src:
      continue
    func = int(re.search(r"_unnamed_function_(\d+)\.c$", path).group(1)) if "_unnamed_function_" in path else None
    src = re.sub(r"=\s*\n\s*", "= ", src)  # join "X =\n   getter(...)" continuation lines
    events = [(m.start(), "get", m) for m in GET_RE.finditer(src)]
    events += [(m.start(), "set", m) for m in ASSIGN_RE.finditer(src)]
    var_key = {}
    for _, kind, m in sorted(events, key=lambda e: e[0]):
      if kind == "get":
        var, getter, addr = m.group(1), m.group(2), int(m.group(3), 16)
        key = ev.strings.get(addr, "?@%x" % addr)
        var_key[var] = (key, GETTERS[getter])
        if var.startswith("DAT_ram_"):
          out[var[8:]] = {"key": key, "type": GETTERS[getter], "func": func}
      else:
        dst, var = m.group(1), m.group(2)
        if var in var_key:
          a = dst[8:] if dst.startswith("DAT_ram_") else dst[4:]
          out[a] = {"key": var_key[var][0], "type": var_key[var][1], "func": func}
  return out


def main():
  ap = argparse.ArgumentParser()
  ap.add_argument("--wasm", required=True)
  ap.add_argument("--index", required=True)
  ap.add_argument("--cdir", required=True)
  ap.add_argument("--out", required=True)
  ap.add_argument("funcs", nargs="+", type=int)
  a = ap.parse_args()
  ev = evidence.Evidence(a.wasm)
  idx = json.load(open(a.index))
  recs = idx if isinstance(idx, list) else (idx.get("functions") or idx.get("records"))
  by = {r["index"]: r for r in recs}
  os.makedirs(a.out, exist_ok=True)
  cfg = config_globals(a.cdir, ev)
  json.dump(dict(sorted(cfg.items())), open(os.path.join(a.out, "config-globals.json"), "w"), indent=1)
  vt = {addr: ev.demangled.get(nm, nm) for addr, (nm, _, _) in ev.vtables.items()}
  for n in a.funcs:
    paths = glob.glob(os.path.join(a.cdir, "*_unnamed_function_%d.c" % n))
    if not paths:
      print("missing", n)
      continue
    lines = open(paths[0], errors="replace").read().split("\n")
    res = []
    for ln in lines:
      notes = []
      for m in DAT_RE.finditer(ln):
        c = cfg.get(m.group(1))
        if c:
          notes.append("cfg:%s" % c["key"])
      for m in HEX_RE.finditer(ln):
        v = int(m.group(1), 16)
        if v in vt:
          notes.append("vt:%s" % vt[v])
      for m in CALL_RE.finditer(ln):
        r = by.get(int(m.group(1)))
        if r and int(m.group(1)) not in (370, 381, 279, 261, 466, 468, 651):
          notes.append("f%s=%s/%s" % (m.group(1), r.get("class"), r.get("module")))
      res.append(ln + ("    // " + "; ".join(dict.fromkeys(notes)) if notes else ""))
    open(os.path.join(a.out, "%d.c" % n), "w").write("\n".join(res))
  print("config globals:", len(cfg), "vtables:", len(vt))


if __name__ == "__main__":
  main()

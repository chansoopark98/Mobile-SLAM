#!/usr/bin/env python3
"""Library matching by normalized opcode-sequence hash.

Parses a static archive of wasm object files (e.g. wasm/libs/ceres/lib/libceres.a built with
our emsdk), names each defined function from the `linking` symbol table, hashes its opcode
sequence (immediates dropped) and looks for identical hashes among xr-slam.wasm functions.

Usage:
  python3 -I libmatch.py --wasm xr-slam.wasm --archive libceres.a [--archive ...] --out libmatch.json
"""

import argparse
import collections
import json
import os
import subprocess
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import wasm_parse  # noqa: E402


def ar_members(path):
  with open(path, "rb") as fh:
    data = fh.read()
  assert data[:8] == b"!<arch>\n", "not an ar archive"
  p = 8
  longnames = b""
  while p + 60 <= len(data):
    hdr = data[p:p + 60]
    name = hdr[:16].decode("ascii", "replace").strip()
    size = int(hdr[48:58].decode().strip())
    body = data[p + 60:p + 60 + size]
    if name == "//":
      longnames = body
    elif name not in ("/", "/SYM64/"):
      if name.startswith("/") and name[1:].isdigit():
        off = int(name[1:])
        end = longnames.find(b"/\n", off)
        name = longnames[off:end].decode("ascii", "replace")
      yield name.rstrip("/"), body
    p += 60 + size + (size & 1)


def object_functions(blob):
  """Return list of (symbol name, seq_hash, n_instr) for defined functions in a wasm object."""
  tmp = wasm_parse.Module.__new__(wasm_parse.Module)
  tmp.raw = blob
  tmp.types, tmp.imports, tmp.func_types = [], [], []
  tmp.num_imported_funcs = 0
  tmp.exports, tmp.elem, tmp.data_segments, tmp.bodies = {}, {}, [], []
  tmp.custom_sections, tmp.globals = [], []
  tmp._parse()
  names = {}
  # linking section: find it again and parse SYMTAB (subsection 8).
  r = wasm_parse.Reader(blob, 8)
  while r.p < len(blob):
    sid = r.u8()
    size = r.uleb()
    end = r.p + size
    if sid == 0:
      sr = wasm_parse.Reader(blob, r.p)
      nm = sr.name()
      if nm == "linking":
        sr.uleb()  # version
        while sr.p < end:
          sub = sr.u8()
          ssize = sr.uleb()
          send = sr.p + ssize
          if sub == 8:
            for _ in range(sr.uleb()):
              kind = sr.u8()
              flags = sr.uleb()
              if kind in (0, 2, 4, 5):  # function, global, event/tag, table
                idx = sr.uleb()
                undefined = flags & 0x10
                if not undefined or flags & 0x40:
                  sym = sr.name()
                  if kind == 0 and not undefined:
                    names[idx] = sym
              elif kind == 1:  # data
                sym = sr.name()
                if not flags & 0x10:
                  sr.uleb()
                  sr.uleb()
                  sr.uleb()
              elif kind == 3:  # section
                sr.uleb()
          sr.p = send
    r.p = end
  out = []
  for idx, bstart, bend, cstart in tmp.bodies:
    try:
      f = wasm_parse.decode_function(blob, cstart, bend)
    except (ValueError, IndexError):
      continue
    out.append((names.get(idx, "func%d" % idx), f["seq_hash"], f["n_instr"]))
  return out


def demangle(names):
  if not names:
    return {}
  try:
    res = subprocess.run(["c++filt"], input="\n".join(names), capture_output=True, text=True, check=True)
    return dict(zip(names, res.stdout.splitlines()))
  except (OSError, subprocess.CalledProcessError):
    return {n: n for n in names}


def main():
  ap = argparse.ArgumentParser()
  ap.add_argument("--wasm", required=True)
  ap.add_argument("--archive", action="append", required=True)
  ap.add_argument("--min-instr", type=int, default=12, help="ignore tiny functions (ambiguous hashes)")
  ap.add_argument("--out", required=True)
  args = ap.parse_args()

  mod = wasm_parse.Module(args.wasm)
  target = collections.defaultdict(list)
  for idx, bstart, bend, cstart in mod.bodies:
    f = wasm_parse.decode_function(mod.raw, cstart, bend)
    if f["n_instr"] >= args.min_instr:
      target[f["seq_hash"]].append(idx)

  lib_hashes = collections.defaultdict(set)
  n_lib_funcs = 0
  for ar in args.archive:
    for member, blob in ar_members(ar):
      if blob[:4] != b"\0asm":
        continue
      for sym, h, n in object_functions(blob):
        n_lib_funcs += 1
        if n >= args.min_instr:
          lib_hashes[h].add(sym)
  dem = demangle(sorted({s for v in lib_hashes.values() for s in v}))
  matches = {}
  for h, idxs in target.items():
    if h in lib_hashes:
      syms = sorted(lib_hashes[h])
      d = dem.get(syms[0], syms[0])
      cls = "eigen" if d.startswith("Eigen::") or "Eigen::internal" in d.split("(")[0] else "ceres"
      if d.startswith(("std::", "void std::")):
        cls = "libc/libc++"
      for idx in idxs:
        matches[idx] = {"symbol": d, "n_symbols": len(syms), "class": cls, "ambiguous_targets": len(idxs)}
  res = {
      "archives": args.archive,
      "library_functions": n_lib_funcs,
      "target_functions_hashed": sum(len(v) for v in target.values()),
      "matched_target_functions": len(matches),
      "matches": {str(k): v for k, v in sorted(matches.items())},
  }
  with open(args.out, "w") as fh:
    json.dump(res, fh, indent=1)
  print(json.dumps({k: v for k, v in res.items() if k != "matches"}, indent=1))


if __name__ == "__main__":
  main()

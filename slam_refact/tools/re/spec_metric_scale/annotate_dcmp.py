#!/usr/bin/env python3
"""Annotate wasm-decompile output for the MetricScaleEstimator spec lane (S5).

For a set of function indices, extracts their dcmp text and appends inline comments:
  * f_xxx(...)            -> callee wasm index (+ module/class from function-index.json)
  * d_<seg>[off]          -> absolute address and C string there (if any)
  * f_ju/f_yq/... (N)     -> C string at address N when N points into a string
  * 0[N]:type             -> config key bound to that global (from getter bodies)
Usage:
  python3 -I annotate_dcmp.py --dcmp X.dcmp --wasm X.wasm --index function-index.json \
     --config config-defaults.json --funcs 944,1163-1244 [--objdump-dir DIR_WITH_funcN.txt] --out DIR
"""
import argparse
import bisect
import json
import os
import re
import struct
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "binary_map"))
import wasm_parse  # noqa: E402

HDR_RE = re.compile(r"^(?:export )?function (\S+?)\(.*// func(\d+)\s*$")
STR_RE = re.compile(rb"[\x09\x0a\x0d\x20-\x7e]{2,}\x00")


def parse_ranges(s):
  out = []
  for part in s.split(","):
    if "-" in part:
      a, b = part.split("-")
      out.extend(range(int(a), int(b) + 1))
    elif part:
      out.append(int(part))
  return out


def main():
  ap = argparse.ArgumentParser()
  ap.add_argument("--dcmp", required=True)
  ap.add_argument("--wasm", required=True)
  ap.add_argument("--index", required=True)
  ap.add_argument("--config", required=True, help="config-defaults.json")
  ap.add_argument("--funcs", required=True)
  ap.add_argument("--out", required=True)
  ap.add_argument("--objdump-dir", default=None,
                  help="dir with func<N>.txt (wasm-objdump -d); V128 consts are substituted in order")
  args = ap.parse_args()

  mod = wasm_parse.Module(args.wasm)
  base, img = mod.mem_image()
  raw = bytes(img)
  strings = {}
  for mt in STR_RE.finditer(raw):
    strings[base + mt.start()] = mt.group()[:-1].decode("ascii")
  saddrs = sorted(strings)

  def string_at(addr):
    i = bisect.bisect_right(saddrs, addr) - 1
    if i >= 0 and addr < saddrs[i] + len(strings[saddrs[i]]) + 1:
      s = saddrs[i]
      return strings[s][addr - s:]
    return None

  # data segment symbol base: wasm-decompile names each data segment d_<...>;
  # we resolve by segment order of appearance in the dcmp header.
  lines = open(args.dcmp, encoding="utf-8", errors="replace").read().split("\n")
  seg_names = []
  for ln in lines[:20000]:
    m = re.match(r"^data (d_\w+)\(offset: (\d+)\)", ln)
    if m:
      seg_names.append((m.group(1), int(m.group(2))))
  seg_base = dict(seg_names)

  name_to_idx = {}
  starts = {}
  for i, ln in enumerate(lines):
    m = HDR_RE.match(ln)
    if m:
      name_to_idx[m.group(1)] = int(m.group(2))
      starts[int(m.group(2))] = i
  idx = json.load(open(args.index))
  recs = idx if isinstance(idx, list) else idx.get("functions", idx)
  info = {r["index"]: r for r in recs}

  # config globals: parse getter bodies (key-string address -> local var -> 0[N] store)
  cfg = json.load(open(args.config))["entries"]
  key_by_addr = {e["addr"]: e for e in cfg}
  getters = sorted({e["func"] for e in cfg})
  glob_key = {}
  for gidx in getters:
    if gidx not in starts:
      continue
    i = starts[gidx]
    var_key = {}
    while True:
      ln = lines[i]
      m = re.search(r"(?:var )?(\w+)(?::\w+)? = f_\w+\((\d+), ", ln)
      if m and int(m.group(2)) in key_by_addr:
        var_key[m.group(1)] = key_by_addr[int(m.group(2))]
      m = re.search(r"0\[(\d+)\]:(\w+) = f_\w+\((\d+), ", ln)
      if m and int(m.group(3)) in key_by_addr:
        glob_key[(int(m.group(1)), m.group(2))] = key_by_addr[int(m.group(3))]["key"]
      m = re.search(r"0\[(\d+)\]:(\w+) = (\w+);", ln)
      if m and m.group(3) in var_key:
        glob_key[(int(m.group(1)), m.group(2))] = var_key[m.group(3)]["key"]
      if ln.startswith("}"):
        break
      i += 1

  def glob_name(n, t):
    t = {"ubyte": "byte"}.get(t, t)
    k = glob_key.get((n, t))
    if k:
      return k
    # dcmp scales the index by type size: byte/ubyte -> addr n, int/float -> 4n, double/long -> 8n
    return None

  os.makedirs(args.out, exist_ok=True)
  summary = {"config_globals": {"%d:%s" % k: v for k, v in sorted(glob_key.items())}}
  json.dump(summary, open(os.path.join(args.out, "config-globals.json"), "w"), indent=1)

  call_re = re.compile(r"\b(f_\w+)\(")
  d_re = re.compile(r"\b(d_\w+)\[(\d+)\]")
  g_re = re.compile(r"\b0\[(\d+)\]:(u?byte|int|float|double|long|short|ushort)")
  num_re = re.compile(r"\b(f_\w+)\(([^()]*?)\b(\d{4,6})\b")
  for fidx in parse_ranges(args.funcs):
    if fidx not in starts:
      continue
    i = starts[fidx]
    out = []
    callees = {}
    v128 = []
    if args.objdump_dir:
      op = os.path.join(args.objdump_dir, "func%d.txt" % fidx)
      if os.path.exists(op):
        for oln in open(op):
          mm = re.search(r"v128\.const ((?:0x[0-9a-f]{8} ?){4})", oln)
          if mm:
            lanes = []
            for w in mm.group(1).split():
              u = int(w, 16)
              f = struct.unpack("<f", struct.pack("<I", u))[0]
              lanes.append(repr(f) if (u == 0 or 1e-30 < abs(f) < 1e30) else w)
            v128.append("V128{" + ",".join(lanes) + "}")
    while True:
      ln = lines[i]
      if v128 is not None and "V128" in ln and v128:
        parts = ln.split("V128")
        rebuilt = parts[0]
        for part in parts[1:]:
          rebuilt += (v128.pop(0) if v128 else "V128?") + part
        ln = rebuilt
      notes = []
      for nm in call_re.findall(ln):
        ci = name_to_idx.get(nm)
        if ci is not None and ci != fidx:
          r = info.get(ci, {})
          tag = "%s=func%d[%s]" % (nm, ci, r.get("module") or r.get("class") or "?")
          notes.append(tag)
          callees[ci] = r.get("module") or r.get("class")
      for nm, off in d_re.findall(ln):
        b0 = seg_base.get(nm)
        if b0 is not None:
          a = b0 + int(off)
          s = string_at(a)
          notes.append("@%d%s" % (a, (' "%s"' % s[:80]) if s else ""))
      for n, t in g_re.findall(ln):
        k = glob_name(int(n), t)
        if k:
          notes.append("0[%s]=%s" % (n, k))
      for m in re.finditer(r"\((\d{4,6})\b|, (\d{4,6})\)", ln):
        a = int(m.group(1) or m.group(2))
        s = string_at(a) if a > 1024 else None
        if s and len(s) >= 3:
          notes.append('str@%d="%s"' % (a, s[:80]))
      seen = []
      for x in notes:
        if x not in seen:
          seen.append(x)
      out.append(ln + (("    /* " + "; ".join(seen) + " */") if seen else ""))
      if ln.startswith("}"):
        break
      i += 1
    with open(os.path.join(args.out, "func%d.ann.dcmp" % fidx), "w") as f:
      f.write("// dcmp lines %d-%d\n" % (starts[fidx] + 1, i + 1))
      f.write("// callees: %s\n" % json.dumps({str(k): v for k, v in sorted(callees.items())}))
      f.write("\n".join(out) + "\n")


if __name__ == "__main__":
  main()

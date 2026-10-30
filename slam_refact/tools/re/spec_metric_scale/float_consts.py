#!/usr/bin/env python3
"""List exact f32/f64 constants per function from wasm-objdump -d output files.

wasm-decompile prints small constants lossy (e.g. 1e-9 as 0.0); this table is the
authoritative value source for the S5 spec. Usage: float_consts.py DIR/func*.txt
"""
import collections
import re
import struct
import sys

RE = re.compile(r"\|\s*(f32|f64)\.const (\S+)")
for path in sys.argv[1:]:
  cnt = collections.Counter()
  for ln in open(path):
    m = RE.search(ln)
    if m:
      t, v = m.groups()
      x = float.fromhex(v) if "0x" in v else float(v)
      if t == "f32":
        x32 = struct.unpack("<f", struct.pack("<f", x))[0]
        cnt[(t, repr(x32), v)] += 1
      else:
        cnt[(t, repr(x), v)] += 1
  name = re.search(r"func(\d+)", path).group(1)
  items = ", ".join("%s %s (%s) x%d" % (t, d, h, n) for (t, d, h), n in sorted(cnt.items()))
  print("func%s: %s" % (name, items))

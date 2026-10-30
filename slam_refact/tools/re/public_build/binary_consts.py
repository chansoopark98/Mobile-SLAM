#!/usr/bin/env python3
"""Cheap public-vs-binary parity helpers over the xr-slam WASM and its WABT disassembly.

Subcommands
  func-consts <wasm> <disassembly> <out.json> FUNC [FUNC ...]
      Decode every f32/f64/v128 constant used by the given function indices (v128 is decoded as
      f64x2 and f32x4).
  string-xrefs <wasm> <disassembly> <out.json> STRING [STRING ...]
      Find functions with `i32.const <linear address of exact C string>` and record the 6
      instructions after each reference (constants/calls), to see which code consumes the string.

The disassembly is `wasm-objdump -d` output (lines "<addr> func[N]:" and "... | <instr>").
"""
import json
import re
import struct
import sys


def leb(b, p):
  r = s = 0
  while True:
    x = b[p]
    p += 1
    r |= (x & 0x7F) << s
    s += 7
    if x < 0x80:
      return r, p


def sleb(b, p):
  r = s = 0
  while True:
    x = b[p]
    p += 1
    r |= (x & 0x7F) << s
    s += 7
    if x < 0x80:
      if x & 0x40:
        r -= 1 << s
      return r, p


def data_segments(b):
  p = 8
  segs = []
  while p < len(b):
    sid = b[p]
    p += 1
    size, p = leb(b, p)
    end = p + size
    if sid == 11:
      q = p
      n, q = leb(b, q)
      for _ in range(n):
        flag, q = leb(b, q)
        assert flag == 0
        assert b[q] == 0x41
        q += 1
        off, q = sleb(b, q)
        assert b[q] == 0x0B
        q += 1
        sz, q = leb(b, q)
        segs.append((off, q, sz))
        q += sz
    p = end
  return segs


def string_addresses(b, segs, s):
  needle = s.encode() + b'\x00'
  out = []
  for off, fo, sz in segs:
    blob = b[fo:fo + sz]
    i = blob.find(needle)
    while i >= 0:
      # Exact C string: previous byte must be NUL (or segment start).
      if i == 0 or blob[i - 1] == 0:
        out.append(off + i)
      i = blob.find(needle, i + 1)
  return out


def iter_funcs(dis):
  func = None
  for line in open(dis, errors='replace'):
    m = re.match(r'^([0-9a-f]+) func\[(\d+)\]', line)
    if m:
      func = int(m.group(2))
      continue
    if '|' in line and func is not None:
      yield func, line.split('|', 1)[1].strip()


def decode_v128(words):
  raw = struct.pack('<4I', *words)
  return {'f64x2': list(struct.unpack('<2d', raw)), 'f32x4': list(struct.unpack('<4f', raw))}


def func_consts(dis, funcs):
  want = set(funcs)
  res = {f: {'f32': {}, 'f64': {}, 'v128': {}} for f in funcs}
  for f, ins in iter_funcs(dis):
    if f not in want:
      continue
    m = re.match(r'(f32|f64)\.const (\S+)', ins)
    if m:
      v = float.fromhex(m.group(2)) if 'x' in m.group(2) else float(m.group(2))
      key = repr(v)
      res[f][m.group(1)][key] = res[f][m.group(1)].get(key, 0) + 1
      continue
    m = re.match(r'v128\.const ((?:0x[0-9a-f]+\s*){4})', ins)
    if m:
      words = [int(w, 16) for w in m.group(1).split()]
      key = ' '.join('%08x' % w for w in words)
      if key not in res[f]['v128']:
        res[f]['v128'][key] = {'count': 0, **decode_v128(words)}
      res[f]['v128'][key]['count'] += 1
  return res


def string_xrefs(b, dis, strings):
  segs = data_segments(b)
  addrs = {s: string_addresses(b, segs, s) for s in strings}
  by_addr = {a: s for s, lst in addrs.items() for a in lst}
  hits = []
  window = []
  for f, ins in iter_funcs(dis):
    window.append((f, ins))
    if len(window) > 7:
      window.pop(0)
    if len(window) == 7:
      f0, ins0 = window[0]
      m = re.match(r'i32\.const (-?\d+)$', ins0)
      if m and int(m.group(1)) in by_addr:
        hits.append({'string': by_addr[int(m.group(1))], 'func': f0, 'addr': int(m.group(1)),
                     'next': [i for _, i in window[1:]]})
  return {'addresses': addrs, 'xrefs': hits}


def main():
  cmd, wasm, dis, out = sys.argv[1:5]
  args = sys.argv[5:]
  b = open(wasm, 'rb').read()
  if cmd == 'func-consts':
    res = func_consts(dis, [int(a) for a in args])
  elif cmd == 'string-xrefs':
    res = string_xrefs(b, dis, args)
  else:
    raise SystemExit('unknown subcommand ' + cmd)
  with open(out, 'w') as f:
    json.dump(res, f, indent=1)
  print(json.dumps(res, indent=1)[:4000])


if __name__ == '__main__':
  main()

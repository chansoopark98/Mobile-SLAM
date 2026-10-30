#!/usr/bin/env python3
"""Write a copy of a wasm module whose export section has extra function exports (US-015 RE oracle).

The function bodies, the data, the table and every other section are copied byte for byte; only the
export section (id 7) gets new entries appended, so the module computes exactly what the original
does. Used to call internal functions of xr-slam.wasm (e.g. func994, the closed image-world-tracker)
from Node with synthetic memory inputs (run_f994.mjs). Never writes next to the input.

  python3 -I export_func.py IN.wasm OUT.wasm f994=994 [name=index ...]
Stdlib only.
"""
import hashlib
import sys


def leb_u(buf, pos):
  result = shift = 0
  while True:
    b = buf[pos]
    pos += 1
    result |= (b & 0x7F) << shift
    shift += 7
    if not b & 0x80:
      return result, pos


def enc_u(v):
  out = bytearray()
  while True:
    b = v & 0x7F
    v >>= 7
    if v:
      out.append(b | 0x80)
    else:
      out.append(b)
      return bytes(out)


def main():
  src, dst = sys.argv[1], sys.argv[2]
  extra = []
  for spec in sys.argv[3:]:
    name, idx = spec.split('=')
    extra.append((name.encode(), int(idx)))
  buf = open(src, 'rb').read()
  assert buf[:8] == b'\0asm\x01\0\0\0'
  out = bytearray(buf[:8])
  pos = 8
  done = False
  while pos < len(buf):
    sid = buf[pos]
    size, body = leb_u(buf, pos + 1)
    end = body + size
    if sid == 7:
      n, p = leb_u(buf, body)
      names = set()
      q = p
      for _ in range(n):
        ln, q = leb_u(buf, q)
        names.add(bytes(buf[q:q + ln]))
        q += ln + 1
        _, q = leb_u(buf, q)
      assert q == end
      entries = bytes(buf[p:end])
      for name, idx in extra:
        assert name not in names, name
        entries += enc_u(len(name)) + name + b'\x00' + enc_u(idx)
      payload = enc_u(n + len(extra)) + entries
      out += bytes([7]) + enc_u(len(payload)) + payload
      done = True
    else:
      out += buf[pos:end]
    pos = end
  assert done, 'no export section'
  open(dst, 'wb').write(out)
  print('in ', hashlib.sha256(buf).hexdigest(), len(buf))
  print('out', hashlib.sha256(out).hexdigest(), len(out), 'exports +%d' % len(extra))


if __name__ == '__main__':
  main()

#!/usr/bin/env python3
"""Load active data segments of a wasm module and resolve addresses to C strings / floats.

Usage (python3 -I):
  wasm_data.py WASM str ADDR [ADDR ...]       # print NUL-terminated string at each address
  wasm_data.py WASM f32 ADDR [N]              # print N float32 values starting at ADDR
  wasm_data.py WASM f64 ADDR [N]
  wasm_data.py WASM i32 ADDR [N]
  wasm_data.py WASM find TEXT                 # print addresses where TEXT occurs
Stdlib only.
"""
import struct
import sys


def _leb(buf, pos, signed=False):
    result = 0
    shift = 0
    while True:
        b = buf[pos]
        pos += 1
        result |= (b & 0x7F) << shift
        shift += 7
        if not b & 0x80:
            break
    if signed and (b & 0x40):
        result -= 1 << shift
    return result, pos


def load_memory(path):
    buf = open(path, 'rb').read()
    assert buf[:4] == b'\0asm'
    pos = 8
    mem = {}
    while pos < len(buf):
        sid = buf[pos]
        pos += 1
        size, pos = _leb(buf, pos)
        end = pos + size
        if sid == 11:  # data section
            n, p = _leb(buf, pos)
            for _ in range(n):
                flag, p = _leb(buf, p)
                if flag == 0:
                    op = buf[p]
                    p += 1
                    assert op == 0x41, op  # i32.const
                    off, p = _leb(buf, p, signed=True)
                    assert buf[p] == 0x0B
                    p += 1
                elif flag == 2:
                    _, p = _leb(buf, p)
                    op = buf[p]
                    p += 1
                    off, p = _leb(buf, p, signed=True)
                    p += 1
                else:  # passive
                    off = None
                ln, p = _leb(buf, p)
                if off is not None:
                    mem[off] = buf[p:p + ln]
                p += ln
        pos = end
    flat = bytearray(max(o + len(b) for o, b in mem.items()))
    for o, b in mem.items():
        flat[o:o + len(b)] = b
    return bytes(flat)


def main():
    path, mode = sys.argv[1], sys.argv[2]
    m = load_memory(path)
    if mode == 'str':
        for a in sys.argv[3:]:
            a = int(a, 0)
            e = m.find(b'\0', a)
            print(a, repr(m[a:e][:200].decode('latin1')))
    elif mode in ('f32', 'f64', 'i32'):
        a = int(sys.argv[3], 0)
        n = int(sys.argv[4]) if len(sys.argv) > 4 else 1
        fmt, sz = {'f32': ('<f', 4), 'f64': ('<d', 8), 'i32': ('<i', 4)}[mode]
        print([struct.unpack_from(fmt, m, a + i * sz)[0] for i in range(n)])
    elif mode == 'find':
        t = sys.argv[3].encode()
        i = m.find(t)
        while i >= 0:
            print(i)
            i = m.find(t, i + 1)


if __name__ == '__main__':
    main()

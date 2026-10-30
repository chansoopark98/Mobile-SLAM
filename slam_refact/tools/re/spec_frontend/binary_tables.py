#!/usr/bin/env python3
"""Spec lane S1: check constant tables of the frontend inside xr-slam.wasm data segments.

1. FEATURE_LUT (reality/engine/features/gr8gl-data.h, 512x36 int8 pairs = 36864 bytes): exact byte search.
2. Intrinsics focal-length jump table (func509 doIosWorkaround / Intrinsics::getCameraIntrinsics) at
   address 296576 (111 floats, DeviceModel 2..112) vs c8/geometry/intrinsics.cc MODEL_TO_INTRINSIC cases.
3. Config key strings used by frontend getters (addresses taken from the decompile).

Usage: python3 -I binary_tables.py --out OUT.json
"""
import argparse
import hashlib
import json
import os
import re
import struct
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, '..', '..', '..', '..'))
WASM = os.path.join(ROOT, 'build/slam-refact-20261015/inputs/xr-slam.wasm')
SRC = os.path.join(ROOT, 'assets/references/xr_reference/official/xr_reference')


def leb(b, i):
  r = s = 0
  while True:
    x = b[i]
    i += 1
    r |= (x & 0x7f) << s
    s += 7
    if x < 0x80:
      return r, i


def sleb(b, i):
  r = s = 0
  while True:
    x = b[i]
    i += 1
    r |= (x & 0x7f) << s
    s += 7
    if x < 0x80:
      if x & 0x40:
        r -= 1 << s
      return r, i


def data_segments(data):
  i, segs = 8, []
  while i < len(data):
    sid = data[i]
    i += 1
    size, i = leb(data, i)
    start = i
    if sid == 11:
      n, j = leb(data, i)
      for _ in range(n):
        flag, j = leb(data, j)
        assert flag == 0 and data[j] == 0x41
        addr, j = sleb(data, j + 1)
        assert data[j] == 0x0b
        ln, j = leb(data, j + 1)
        segs.append((addr, data[j:j + ln]))
        j += ln
    i = start + size
  return segs


def mem(segs, addr, n):
  for a, b in segs:
    if a <= addr and addr + n <= a + len(b):
      return b[addr - a:addr - a + n]
  return None


def main():
  ap = argparse.ArgumentParser()
  ap.add_argument('--out', required=True)
  args = ap.parse_args()
  data = open(WASM, 'rb').read()
  segs = data_segments(data)
  out = {'wasm_sha256': hashlib.sha256(data).hexdigest()}
  # 1. FEATURE_LUT
  txt = open(os.path.join(SRC, 'reality/engine/features/gr8gl-data.h')).read()
  body = txt[txt.index('FEATURE_LUT[512][36]'):]
  pairs = re.findall(r'\{\s*(-?\d+)\s*,\s*(-?\d+)\s*\}', body)
  assert len(pairs) == 512 * 36, len(pairs)
  lut = b''.join(struct.pack('<bb', int(a), int(b)) for a, b in pairs)
  hits = []
  for a, b in segs:
    k = b.find(lut)
    while k >= 0:
      hits.append(a + k)
      k = b.find(lut, k + 1)
  out['feature_lut'] = {'bytes': len(lut), 'sha256': hashlib.sha256(lut).hexdigest(), 'addresses': hits}
  # 2. focal table
  h = open(os.path.join(SRC, 'c8/camera/device-infos.h')).read()
  enum = re.search(r'enum DeviceModel \{(.*?)\};', h, re.S).group(1)
  names = [re.match(r'\s*(\w+)', l).group(1) for l in enum.split('\n') if re.match(r'\s*[A-Z]\w*\s*,', l)]
  c = open(os.path.join(SRC, 'c8/geometry/intrinsics.cc')).read()
  cases = {a: float(b) for a, b in re.findall(r'MODEL_TO_INTRINSIC\((\w+),\s*([\d.]+)f\)', c)}
  tab = struct.unpack('<111f', mem(segs, 296576, 111 * 4))
  mism = []
  for idx, name in enumerate(names):
    if 2 <= idx <= 112:
      exp = cases.get(name, 506.4104)
      if abs(tab[idx - 2] - exp) > 1e-3:
        mism.append([idx, name, tab[idx - 2], exp])
  out['focal_table'] = {'address': 296576, 'entries': 111, 'enum_range': [2, 112], 'mismatches': mism,
                        'GOOGLE_PIXEL7': [names.index('GOOGLE_PIXEL7'), tab[names.index('GOOGLE_PIXEL7') - 2]],
                        'APPLE_IPHONE_X(default)': [names.index('APPLE_IPHONE_X'), tab[names.index('APPLE_IPHONE_X') - 2]],
                        'public_cases': len(cases)}
  # 2b. subpixelQ (gr8-feature-shader.cc createSubpixelParams, double math cast to float).
  p0, p1, p2 = 0.975316, 0.006133, 0.000039
  k0, k1, k2 = p0 * p1, p0 * p2, p1 * p2
  k3, k4, k5, k6 = k1 + 2.0 * k2, k0 + 2.0 * k1, k0 + 4.0 * (k1 + k2), 2.0 * p1 + 4.0 * p2
  q = struct.pack('<6f', 0.5 * k3 / k5, 0.5 * k4 / k5, p1 / k6, p2 / k6, k2 / k5, (k4 + 2.0 * k3 - 4.0 * k2) / k5)
  qhits = []
  for a, b in segs:
    k = b.find(q)
    while k >= 0:
      qhits.append(a + k)
      k = b.find(q, k + 1)
  out['subpixel_q'] = {'floats': list(struct.unpack('<6f', q)), 'addresses': qhits}
  # 3. config key strings
  keys = {}
  for addr in (31309, 20112, 20005, 33330, 24709, 21521):
    s = mem(segs, addr, 64)
    keys[addr] = s.split(b'\0')[0].decode() if s else None
  out['config_key_strings'] = keys
  os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
  json.dump(out, open(args.out, 'w'), indent=1)
  print(json.dumps({'subpixel_q_hits': qhits, 'feature_lut_hits': hits, 'focal_mismatches': len(mism), 'keys': keys}))
  return 0


if __name__ == '__main__':
  sys.exit(main())

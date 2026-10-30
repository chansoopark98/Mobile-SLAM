#!/usr/bin/env python3
"""Generate src/frontend/device_table.inc from the public reference runtime source (S1 §5.12, WP1).

Inputs (public xr_reference/xr_reference@4edfa484, SOURCE):
  c8/camera/device-infos.h   enum DeviceModel (value = position in the enum)
  c8/geometry/intrinsics.cc  MODEL_TO_INTRINSIC cases (99); every other model -> default 506.4104f
  c8/camera/device-infos.cc  deviceMap (manufacturer -> model string -> DeviceModel)

Usage (cwd = repo root):
  python3 -I slam_refact/engine/src/frontend/gen_device_table_inc.py \
      > slam_refact/engine/src/frontend/device_table.inc
"""
import os
import re
import sys


def main():
  src = sys.argv[1] if len(sys.argv) > 1 else os.path.abspath(os.path.join(
      os.path.dirname(__file__), '../../third_party/xr_source'))
  h = open(os.path.join(src, 'c8/camera/device-infos.h')).read()
  enum = re.search(r'enum DeviceModel \{(.*?)\};', h, re.S).group(1)
  names = []
  for line in enum.split('\n'):
    m = re.match(r'\s*([A-Z]\w*)\s*,?', line)
    if m:
      names.append(m.group(1))
  c = open(os.path.join(src, 'c8/geometry/intrinsics.cc')).read()
  cases = re.findall(r'MODEL_TO_INTRINSIC\((\w+),\s*([\d.]+f)\)', c)
  casemap = dict(cases)
  assert len(cases) == len(casemap) == 99, len(cases)
  cc = open(os.path.join(src, 'c8/camera/device-infos.cc')).read()
  body = cc[cc.index('deviceMap{'):cc.index('// Built from deviceMap')]
  out = [
    '// Generated from public xr_reference/xr_reference@4edfa484 (SOURCE), do not edit by hand:',
    '//   SR_DEVICE_MODEL(enum, focal): c8/camera/device-infos.h:34-148 enum order (value = position);',
    '//     focal = c8/geometry/intrinsics.cc:366-475 MODEL_TO_INTRINSIC case, or the default 506.4104f.',
    '//   SR_DEVICE_ENTRY(manufacturer, model, enum): c8/camera/device-infos.cc:91-267 deviceMap.',
    '// Every macro must be defined by the includer.',
  ]
  for n in names:
    out.append('SR_DEVICE_MODEL(%s, %s)' % (n, casemap.get(n, '506.4104f')))
  manufacturer = None
  count = 0
  for line in body.split('\n'):
    m = re.match(r'\s*\{"([^"]*)",\s*(\{|$)', line)
    if m and not re.match(r'\s*\{"[^"]*",\s*[A-Za-z_0-9]+\}', line):
      manufacturer = m.group(1)
    for mm in re.finditer(r'\{"([^"]*)",\s*([A-Z][A-Za-z_0-9]*)\}', line):
      out.append('SR_DEVICE_ENTRY("%s", "%s", %s)' % (manufacturer, mm.group(1), mm.group(2)))
      count += 1
  print('\n'.join(out))
  print('models %d, focal cases %d, map entries %d' % (len(names), len(cases), count), file=sys.stderr)
  return 0


if __name__ == '__main__':
  sys.exit(main())

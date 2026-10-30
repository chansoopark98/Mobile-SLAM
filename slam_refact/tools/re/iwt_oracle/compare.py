#!/usr/bin/env python3
"""Compare the original func994 outputs (run_f994.mjs JSON) with the engine's (sr_iwt_replay text).

Per case and field: element counts must be equal; integer fields (visible indices, ids) must be equal;
float fields report the number of bit-identical values and the max absolute difference. Prints one
JSON summary; exit 0 only when every count and integer field matches and the out/state matrices and
the scale agree within --tol (default 1e-5 absolute). The tolerance is a reporting threshold of this
development comparison, not a gate (the gate is preregistered separately; spec/11 section 9.6).

  python3 -I compare.py ORACLE.json ENGINE.txt [--tol 1e-5]
Stdlib only.
"""
import json
import math
import struct
import sys


def f32(x):
  if x is None:  # JSON.stringify writes NaN and +-Infinity as null
    return math.nan
  return struct.unpack('<f', struct.pack('<f', x))[0]


def parse_engine(path):
  cases = {}
  cur = None
  for line in open(path):
    tok = line.split()
    if not tok:
      continue
    tag = tok[0]
    if tag == 'case':
      cur = cases.setdefault(tok[1], {})
    elif tag == 'scale':
      cur['scale'] = float(tok[1])
    elif tag in ('out', 'state_pose'):
      v = [float(x) for x in tok[1:]]
      cur[tag] = {'m': v[:16], 'inv': v[16:32]}
    elif tag in ('rays',):
      n = int(tok[1])
      v = [float(x) for x in tok[2:]]
      cur[tag] = [v[3 * i:3 * i + 3] for i in range(n)]
    elif tag in ('search', 'world', 'target'):
      n = int(tok[1])
      v = [float(x) for x in tok[2:]]
      cur[tag] = [v[4 * i:4 * i + 4] for i in range(n)]
    elif tag == 'visible':
      cur[tag] = [int(x) for x in tok[2:]]
    elif tag == 'ids':
      v = [int(x) for x in tok[2:]]
      cur[tag] = [v[2 * i:2 * i + 2] for i in range(len(v) // 2)]
  return cases


def floats(a, b):
  """(count bit-identical, max abs diff, n) over two equally shaped nested float lists."""
  fa, fb = [], []

  def flat(x, out):
    if isinstance(x, list):
      for y in x:
        flat(y, out)
    else:
      out.append(x)
  flat(a, fa)
  flat(b, fb)
  if len(fa) != len(fb):
    return None
  same = 0
  mx = 0.0
  for x, y in zip(fa, fb):
    x, y = f32(x), f32(y)
    if math.isnan(x) and not math.isfinite(y):  # null in the oracle JSON: NaN or an infinity
      same += 1
    elif x == y:
      same += 1
    elif not (math.isfinite(x) and math.isfinite(y)):
      mx = math.inf  # v2 (review F-8): a finite/non-finite mismatch must fail, max() ignores NaN
    else:
      mx = max(mx, abs(x - y))
  return same, mx, len(fa)


def main():
  oracle = json.load(open(sys.argv[1]))
  engine = parse_engine(sys.argv[2])
  tol = float(sys.argv[sys.argv.index('--tol') + 1]) if '--tol' in sys.argv else 1e-5
  report = {}
  ok = True
  fields = [('rays', 'rays'), ('search_pts_in_camera', 'search'), ('world_pts', 'world'),
            ('target_pts', 'target')]
  for r in oracle['results']:
    name = r['name']
    e = engine.get(name)
    rec = {}
    if r['error'] or e is None:
      rec['error'] = r['error'] or 'engine case missing'
      ok = False
      report[name] = rec
      continue
    s = r['state']
    rec['scale'] = {'oracle': s['scale'], 'engine': e['scale'], 'abs_diff': abs(f32(s['scale']) - f32(e['scale']))}
    ok = ok and rec['scale']['abs_diff'] <= tol
    for key in ('out', 'state_pose'):
      o = r['out'] if key == 'out' else s['world_pose']
      res = floats([o['m'], o['inv']], [e[key]['m'], e[key]['inv']])
      rec[key] = {'bit_identical': '%d/%d' % (res[0], res[2]), 'max_abs_diff': res[1]}
      ok = ok and res[1] <= tol
    for ok_key, e_key in fields:
      a, b = s[ok_key], e[e_key]
      if len(a) != len(b):
        rec[ok_key] = {'count': [len(a), len(b)]}
        ok = False
        continue
      res = floats(a, b)
      rec[ok_key] = {'count': len(a), 'bit_identical': '%d/%d' % (res[0], res[2]), 'max_abs_diff': res[1]}
    rec['visible_equal'] = s['visible'] == e['visible']
    rec['ids_equal'] = s['ids'] == e['ids']
    ok = ok and rec['visible_equal'] and rec['ids_equal']
    report[name] = rec
  print(json.dumps({'all_within_tol': ok, 'tol': tol, 'cases': report}, indent=1))
  sys.exit(0 if ok else 1)


if __name__ == '__main__':
  main()

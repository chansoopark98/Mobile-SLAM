#!/usr/bin/env python3
"""Applies the ML regression gate v2 to measured error statistics (RM-1b).

Independent of the C++ test (tests/ml/regression_gate.h): the same rule written again, so the two can
be compared on the same numbers. Rule per row and statistic: observed <= baseline + 3 * resolution
(the table already holds resolution = max(null-run spread, 2^-23 * scale)).

Inputs: the table (tests/ml/data/model_regression_gate_v2.tsv) and measured statistics as
  --stats <tsv>   model, input, output, max_abs, rms, ref_max_abs, ref_rms (the statdump driver), or
  --bench <json>  the --json output of sr_ml_bench (native or WASM), or
  --report <dir>  the report-native-<model>.json files written by sr_tests_ml_models.
Exit status 0 = every row passes, 1 = at least one row fails or a row is missing.

  python3 -I eval_gate_v2.py --table <tsv> --stats <tsv> [--quiet] [--json out.json]
"""
import argparse
import glob
import json
import os
import sys

FACTOR = 3.0


def load_table(path):
  rows = {}
  with open(path) as f:
    for line in f:
      line = line.rstrip('\n')
      if not line or line.startswith('#'):
        continue
      p = line.split('\t')
      if len(p) != 9:
        sys.exit('%s: bad row %r' % (path, line))
      rows[(p[0], int(p[1]), p[2])] = [float(x) for x in p[3:]]
  return rows


def load_stats(args):
  out = {}
  if args.stats:
    with open(args.stats) as f:
      for line in f:
        p = line.rstrip('\n').split('\t')
        if len(p) == 7 and not line.startswith('#'):
          out[(p[0], int(p[1]), p[2])] = (float(p[3]), float(p[4]))
  if args.bench:
    d = json.load(open(args.bench))
    for m in d['models']:
      for o in m['outputs']:
        out[(m['name'], o['input'], o['name'])] = (o['max_abs'], o['rms'])
  if args.report:
    for path in sorted(glob.glob(os.path.join(args.report, 'report-native-*.json'))):
      d = json.load(open(path))
      for inp in d['inputs']:
        for o in inp['outputs']:
          out[(d['model'], inp['input'], o['name'])] = (o['max_abs'], o['rms'])
  return out


def main():
  ap = argparse.ArgumentParser()
  ap.add_argument('--table', required=True)
  ap.add_argument('--stats')
  ap.add_argument('--bench')
  ap.add_argument('--report')
  ap.add_argument('--quiet', action='store_true')
  ap.add_argument('--json')
  args = ap.parse_args()
  table = load_table(args.table)
  stats = load_stats(args)
  if not stats:
    sys.exit('no measured statistics given')
  results = []
  failures = 0
  worst = 0.0
  for key, (b_max, b_rms, _rm, _rr, r_max, r_rms) in table.items():
    if key not in stats:
      print('MISSING row %s' % (key,))
      failures += 1
      continue
    o_max, o_rms = stats[key]
    lim_max = b_max + FACTOR * r_max
    lim_rms = b_rms + FACTOR * r_rms
    ratio_max = o_max / lim_max if lim_max > 0 else (0.0 if o_max == 0 else float('inf'))
    ratio_rms = o_rms / lim_rms if lim_rms > 0 else (0.0 if o_rms == 0 else float('inf'))
    ok = not (o_max != o_max or o_rms != o_rms) and o_max <= lim_max and o_rms <= lim_rms
    worst = max(worst, ratio_max, ratio_rms)
    failures += 0 if ok else 1
    results.append({'row': '/'.join(map(str, key)), 'ok': ok, 'ratio_max_abs': ratio_max,
                    'ratio_rms': ratio_rms})
    if not args.quiet or not ok:
      print('%-5s %-44s max_abs %.3f rms %.3f of limit' % ('PASS' if ok else 'FAIL', '/'.join(map(str, key)),
                                                           ratio_max, ratio_rms))
  extra = sorted(set(stats) - set(table))
  if extra:
    print('note: %d measured rows have no table row: %s' % (len(extra), extra[:3]))
  print('%d rows, %d failing, worst observed/limit %.4f' % (len(table), failures, worst))
  if args.json:
    with open(args.json, 'w') as f:
      json.dump({'rows': results, 'failures': failures, 'worst_ratio': worst}, f, indent=1)
  sys.exit(1 if failures else 0)


if __name__ == '__main__':
  main()

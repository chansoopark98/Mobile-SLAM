#!/usr/bin/env python3
"""Applies the ML regression gate v3 to measured error statistics (RM-1b).

Independent of the C++ test (tests/ml/regression_gate.h): the same rules written again, so the two can
be compared on the same numbers.

Row rule per row and statistic: observed <= baseline + 3 * value (the table holds value =
max(2^-23 * scale, allowance)). Reference rule: the observed max|r| and RMS(r) equal the table's
within 1e-12 relative. Aggregate rule: the geometric mean of observed / baseline over the rows with
baseline_rms > 0, baseline_max_abs > 0 and ref_rms >= 1e-3 is <= the registered limit, for each
statistic.

Inputs: the tables and measured statistics as
  --stats <tsv>   model, input, output, max_abs, rms, ref_max_abs, ref_rms (the statdump driver), or
  --bench <json>  the --json output of sr_ml_bench (native or WASM; has no reference scales, so the
                  reference rule is skipped and said so), or
  --report <dir>  the report-native-<model>.json files written by sr_tests_ml_models.
Exit status 0 = every row and both aggregates pass, 1 = a failure or a missing row.

  python3 -I eval_gate_v3.py --table <tsv> --aggregate <tsv> --stats <tsv> [--quiet] [--json out.json]
"""
import argparse
import glob
import json
import math
import os
import sys

FACTOR = 3.0
REF_REL_TOLERANCE = 1e-12
AGGREGATE_MIN_REF_RMS = 1e-3


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
      key = (p[0], int(p[1]), p[2])
      if key in rows:
        sys.exit('%s: duplicate row %r' % (path, key))
      rows[key] = [float(x) for x in p[3:]]
  return rows


def load_aggregate(path):
  out = {}
  with open(path) as f:
    for line in f:
      line = line.rstrip('\n')
      if not line or line.startswith('#'):
        continue
      p = line.split('\t')
      if len(p) != 3:
        sys.exit('%s: bad row %r' % (path, line))
      out[p[0]] = (float(p[1]), int(p[2]))
  if set(out) != {'max_abs', 'rms'}:
    sys.exit('%s: needs one max_abs and one rms line' % path)
  return out


def load_stats(args):
  """key -> (max_abs, rms, ref_max_abs or None, ref_rms or None)"""
  out = {}

  def put(key, value):
    if key in out:
      sys.exit('duplicate measured row %r' % (key,))
    out[key] = value

  if args.stats:
    with open(args.stats) as f:
      for line in f:
        p = line.rstrip('\n').split('\t')
        if len(p) == 7 and not line.startswith('#'):
          put((p[0], int(p[1]), p[2]), (float(p[3]), float(p[4]), float(p[5]), float(p[6])))
  if args.bench:
    d = json.load(open(args.bench))
    for m in d['models']:
      for o in m['outputs']:
        put((m['name'], o['input'], o['name']), (o['max_abs'], o['rms'], o.get('ref_max_abs'),
                                                o.get('ref_rms')))
  if args.report:
    for path in sorted(glob.glob(os.path.join(args.report, 'report-native-*.json'))):
      if path.endswith('report-native-aggregate.json'):
        continue
      d = json.load(open(path))
      for inp in d['inputs']:
        for o in inp['outputs']:
          put((d['model'], inp['input'], o['name']), (o['max_abs'], o['rms'], o['ref_max_abs'],
                                                     o['ref_rms']))
  return out


def rel_equal(a, b):
  return abs(a - b) <= REF_REL_TOLERANCE * abs(b)


def main():
  ap = argparse.ArgumentParser()
  ap.add_argument('--table', required=True)
  ap.add_argument('--aggregate', required=True)
  ap.add_argument('--stats')
  ap.add_argument('--bench')
  ap.add_argument('--report')
  ap.add_argument('--quiet', action='store_true')
  ap.add_argument('--json')
  args = ap.parse_args()
  table = load_table(args.table)
  agg_limits = load_aggregate(args.aggregate)
  stats = load_stats(args)
  if not stats:
    sys.exit('no measured statistics given')
  results = []
  failures = 0
  worst = 0.0
  ref_checked = 0
  sum_log = {'max_abs': 0.0, 'rms': 0.0}
  agg_n = 0
  for key, (b_max, b_rms, ref_max, ref_rms, v_max, v_rms) in table.items():
    if key not in stats:
      print('MISSING row %s' % (key,))
      failures += 1
      continue
    o_max, o_rms, o_ref_max, o_ref_rms = stats[key]
    lim_max = b_max + FACTOR * v_max
    lim_rms = b_rms + FACTOR * v_rms
    ratio_max = o_max / lim_max if lim_max > 0 else (0.0 if o_max == 0 else float('inf'))
    ratio_rms = o_rms / lim_rms if lim_rms > 0 else (0.0 if o_rms == 0 else float('inf'))
    ref_ok = True
    if o_ref_max is not None and o_ref_rms is not None:
      ref_checked += 1
      ref_ok = rel_equal(o_ref_max, ref_max) and rel_equal(o_ref_rms, ref_rms)
    ok = (ref_ok and not (o_max != o_max or o_rms != o_rms) and o_max <= lim_max and o_rms <= lim_rms)
    worst = max(worst, ratio_max, ratio_rms)
    failures += 0 if ok else 1
    if b_rms > 0 and b_max > 0 and ref_rms >= AGGREGATE_MIN_REF_RMS:
      sum_log['max_abs'] += math.log(o_max / b_max)
      sum_log['rms'] += math.log(o_rms / b_rms)
      agg_n += 1
    results.append({'row': '/'.join(map(str, key)), 'ok': ok, 'reference_ok': ref_ok,
                    'ratio_max_abs': ratio_max, 'ratio_rms': ratio_rms})
    if not args.quiet or not ok:
      print('%-5s %-44s max_abs %.3f rms %.3f of limit%s' % (
          'PASS' if ok else 'FAIL', '/'.join(map(str, key)), ratio_max, ratio_rms,
          '' if ref_ok else '  (reference scales differ from the table)'))
  agg_result = {}
  for name in ('max_abs', 'rms'):
    limit, rows = agg_limits[name]
    g = math.exp(sum_log[name] / agg_n) if agg_n else float('nan')
    ok = agg_n == rows and g <= limit
    failures += 0 if ok else 1
    agg_result[name] = {'geomean_ratio': g, 'limit': limit, 'rows': agg_n, 'ok': ok}
    if not args.quiet or not ok:
      print('%-5s aggregate %-8s geomean observed/baseline %.4f of limit %.4f over %d rows' % (
          'PASS' if ok else 'FAIL', name, g, limit, agg_n))
  extra = sorted(set(stats) - set(table))
  if extra:
    print('note: %d measured rows have no table row: %s' % (len(extra), extra[:3]))
  if ref_checked == 0:
    print('note: no reference scales in the measured statistics; the reference rule was not applied')
  print('%d rows, %d failing (rows + aggregates), worst row observed/limit %.4f, aggregate max_abs %.4f / '
        'rms %.4f' % (len(table), failures, worst, agg_result['max_abs']['geomean_ratio'],
                      agg_result['rms']['geomean_ratio']))
  if args.json:
    with open(args.json, 'w') as f:
      json.dump({'rows': results, 'aggregate': agg_result, 'failures': failures,
                 'worst_ratio': worst}, f, indent=1)
  sys.exit(1 if failures else 0)


if __name__ == '__main__':
  main()

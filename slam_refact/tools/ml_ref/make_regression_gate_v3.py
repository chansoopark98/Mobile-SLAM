#!/usr/bin/env python3
"""Writes the relative regression gate v3 tables (RM-1b; docs/design/ml-runtime.md section 5.3).

Rules (registered in build/slam-refact-full-20261019/preregistration/rm1-regression-gate.v3.md).

Row rule, for each (model, input, output) row and each statistic s in {max_abs, rms}:

    baseline_s     the engine's own error statistic (--baseline)
    allowance_s    max over the null-run files (--null) of |s_V - baseline_s|: the change of the
                   statistic under implementations of the same accuracy class (a tolerance policy)
    resolution_s   2^-23 * scale   (scale = max|r| for max_abs, RMS(r) for rms): one float32 rounding
                   step of the output scale. The null-run noise of the engine itself (same binary on
                   P and E cores) is measured by the caller and must be 0.
    table value    max(resolution_s, allowance_s)
    limit_s        baseline_s + 3 * table value                (tolerance >= 3 x resolution, OR-3)

Aggregate rule: over the rows with baseline_rms > 0, baseline_max_abs > 0 and ref_rms >= 1e-3,
G_s(V) = geometric mean of s_V / baseline_s. allowance_G = max_V |G_s(V) - 1|, limit_G =
1 + 3 * max(2^-23, allowance_G).

The test (tests/ml/regression_gate.h) reads the tables and applies the same arithmetic. Input files
are the TSVs of the statdump driver: model, input, output, max_abs, rms, ref_max_abs, ref_rms (17
significant digits, the CompareOutput of the engine tests). Run once; refuses to overwrite.

  python3 -I make_regression_gate_v3.py --baseline <engine.tsv> --null <v1.tsv> [--null ...] \\
      --out slam_refact/engine/tests/ml/data/model_regression_gate_v3.tsv \\
      --aggregate-out slam_refact/engine/tests/ml/data/model_regression_gate_v3_aggregate.tsv \\
      [--analysis analysis.json]
"""
import argparse
import json
import math
import os
import sys

EPS32 = 2.0 ** -23
FACTOR = 3.0
ABS_TOL = 1e-3   # ml-runtime.md 5.2 (compare.h kModelMaxAbsTolerance)
RMS_TOL = 1e-4   # ml-runtime.md 5.2 (compare.h kModelRmsTolerance)
AGGREGATE_MIN_REF_RMS = 1e-3


def load(path):
  rows = {}
  order = []
  with open(path) as f:
    for line in f:
      line = line.rstrip('\n')
      if not line or line.startswith('#'):
        continue
      p = line.split('\t')
      if len(p) != 7:
        sys.exit('%s: bad row %r' % (path, line))
      key = (p[0], int(p[1]), p[2])
      if key in rows:
        sys.exit('%s: duplicate row %r' % (path, key))
      rows[key] = [float(x) for x in p[3:]]
      order.append(key)
  return rows, order


def main():
  ap = argparse.ArgumentParser()
  ap.add_argument('--baseline', required=True)
  ap.add_argument('--null', action='append', required=True)
  ap.add_argument('--out', required=True)
  ap.add_argument('--aggregate-out', required=True)
  ap.add_argument('--analysis')
  args = ap.parse_args()
  for p in (args.out, args.aggregate_out):
    if os.path.exists(p):
      sys.exit('%s exists; the tables are written once' % p)
  base, order = load(args.baseline)
  nulls = []
  for path in args.null:
    rows, _ = load(path)
    if set(rows) != set(base):
      sys.exit('%s: rows differ from the baseline' % path)
    for k in base:
      if rows[k][2] != base[k][2] or rows[k][3] != base[k][3]:
        sys.exit('%s: reference scale differs for %r' % (path, k))
    nulls.append(rows)
  out_rows = []
  analysis = []
  for key in order:
    b_max, b_rms, ref_max, ref_rms = base[key]
    table = []
    info = {'row': '/'.join(map(str, key))}
    for name, idx, base_v, scale, tol_abs in (('max_abs', 0, b_max, ref_max, ABS_TOL * max(1.0, ref_max)),
                                              ('rms', 1, b_rms, ref_rms, RMS_TOL * max(1.0, ref_rms))):
      allowance = max(abs(n[key][idx] - base_v) for n in nulls)
      resolution = EPS32 * scale
      value = max(resolution, allowance)
      limit = base_v + FACTOR * value
      table.append(value)
      info[name] = {
          'baseline': base_v, 'allowance': allowance, 'resolution': resolution, 'table_value': value,
          'limit': limit, 'resolution_bound': allowance <= resolution,
          'abs_tolerance_5_2': tol_abs,
          'no_added_value_over_5_2': limit >= tol_abs,
          'limit_over_baseline': (limit / base_v) if base_v > 0 else None,
          'tolerance_over_resolution': round((limit - base_v) / resolution, 9),
      }
    out_rows.append((key, base[key], table))
    analysis.append(info)
  # aggregate
  agg_rows = [k for k in order
              if base[k][1] > 0 and base[k][0] > 0 and base[k][3] >= AGGREGATE_MIN_REF_RMS]
  agg = {}
  for name, idx in (('max_abs', 0), ('rms', 1)):
    per_variant = []
    for n in nulls:
      g = math.exp(sum(math.log(n[k][idx] / base[k][idx]) for k in agg_rows) / len(agg_rows))
      per_variant.append(g)
    allowance = max(abs(g - 1.0) for g in per_variant)
    resolution = max(EPS32, allowance)
    agg[name] = {'variants': per_variant, 'allowance': allowance, 'limit': 1.0 + FACTOR * resolution,
                 'rows': len(agg_rows)}
  with open(args.out, 'x') as f:
    f.write('# slam_refact ML relative regression gate v3 (RM-1b). Written once by '
            'tools/ml_ref/make_regression_gate_v3.py.\n')
    f.write('# row rule: observed <= baseline + 3 * value, per statistic; value = max(2^-23 * scale, '
            'allowance of same-class implementations)\n')
    f.write('# model\tinput\toutput\tbaseline_max_abs\tbaseline_rms\tref_max_abs\tref_rms\t'
            'value_max_abs\tvalue_rms\n')
    for key, vals, table in out_rows:
      f.write('%s\t%d\t%s\t%s\n' % (key[0], key[1], key[2],
                                    '\t'.join(repr(v) for v in list(vals) + table)))
  with open(args.aggregate_out, 'x') as f:
    f.write('# slam_refact ML regression gate v3, aggregate rule. Written once by '
            'tools/ml_ref/make_regression_gate_v3.py.\n')
    f.write('# geometric mean of observed/baseline over the rows with baseline_rms > 0, baseline_max_abs > 0 '
            'and ref_rms >= 1e-3 must be <= limit = 1 + 3 * max(2^-23, allowance)\n')
    f.write('# statistic\tlimit\trows\n')
    for name in ('max_abs', 'rms'):
      f.write('%s\t%s\t%d\n' % (name, repr(agg[name]['limit']), agg[name]['rows']))
  if args.analysis:
    with open(args.analysis, 'x') as f:
      json.dump({'rows': analysis, 'aggregate': agg}, f, indent=1)
  n_res = sum(1 for a in analysis for s in ('max_abs', 'rms') if a[s]['resolution_bound'])
  n_nav = sum(1 for a in analysis for s in ('max_abs', 'rms') if a[s]['no_added_value_over_5_2'])
  print('%d rows, %d statistics: %d resolution-bound (allowance <= one rounding step), %d with a limit '
        'at or above the section 5.2 tolerance' % (len(analysis), 2 * len(analysis), n_res, n_nav))
  print('aggregate over %d rows: max_abs limit %.6f (allowance %.6f), rms limit %.6f (allowance %.6f)'
        % (len(agg_rows), agg['max_abs']['limit'], agg['max_abs']['allowance'], agg['rms']['limit'],
           agg['rms']['allowance']))


if __name__ == '__main__':
  main()

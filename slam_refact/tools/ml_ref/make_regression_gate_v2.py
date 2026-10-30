#!/usr/bin/env python3
"""Writes the relative regression gate table v2 (RM-1b; docs/design/ml-runtime.md section 5.3).

Rule (registered in build/slam-refact-full-20261019/preregistration/rm1-regression-gate.v2.md),
for each (model, input, output) row and each statistic s in {max_abs, rms}:

    baseline_s    the engine's own error statistic (--baseline)
    spread_s      max over the null-run files (--null) of |s_V - baseline_s|
    floor_s       2^-23 * scale   (scale = max|r| for max_abs, RMS(r) for rms): one float32 rounding
                  step of the output scale
    resolution_s  max(spread_s, floor_s)
    limit_s       baseline_s + 3 * resolution_s            (tolerance = 3 x resolution, OR-3)

The test (tests/ml/regression_gate.h) reads baseline and resolution from the table and applies
limit = baseline + 3 * resolution.

Input files are the TSVs of the statdump driver: model, input, output, max_abs, rms, ref_max_abs,
ref_rms (17 significant digits, the CompareOutput of the engine tests). Run once; refuses to
overwrite an existing table.

  python3 -I make_regression_gate_v2.py --baseline <engine.tsv> --null <v1.tsv> [--null ...] \
      --out slam_refact/engine/tests/ml/data/model_regression_gate_v2.tsv [--analysis analysis.json]
"""
import argparse
import json
import os
import sys

EPS32 = 2.0 ** -23
FACTOR = 3.0
ABS_TOL = 1e-3   # ml-runtime.md 5.2 (compare.h kModelMaxAbsTolerance)
RMS_TOL = 1e-4   # ml-runtime.md 5.2 (compare.h kModelRmsTolerance)


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
  ap.add_argument('--analysis')
  args = ap.parse_args()
  if os.path.exists(args.out):
    sys.exit('%s exists; the table is written once' % args.out)
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
    res = []
    info = {'row': '/'.join(map(str, key))}
    for name, idx, base_v, scale, tol_abs in (('max_abs', 0, b_max, ref_max, ABS_TOL * max(1.0, ref_max)),
                                              ('rms', 1, b_rms, ref_rms, RMS_TOL * max(1.0, ref_rms))):
      spread = max(abs(n[key][idx] - base_v) for n in nulls)
      floor = EPS32 * scale
      resolution = max(spread, floor)
      limit = base_v + FACTOR * resolution
      res.append(resolution)
      info[name] = {
          'baseline': base_v, 'spread': spread, 'floor': floor, 'resolution': resolution,
          'limit': limit, 'floor_bound': spread <= floor,
          'abs_tolerance_5_2': tol_abs,
          # a row whose limit reaches the absolute tolerance of section 5.2 adds nothing to it
          'no_added_value_over_5_2': limit >= tol_abs,
          'limit_over_baseline': (limit / base_v) if base_v > 0 else None,
          'tolerance_over_resolution': (limit - base_v) / resolution,
      }
    out_rows.append((key, base[key], res))
    analysis.append(info)
  with open(args.out, 'x') as f:
    f.write('# slam_refact ML relative regression gate v2 (RM-1b). Written once by '
            'tools/ml_ref/make_regression_gate_v2.py.\n')
    f.write('# rule: observed <= baseline + 3 * resolution, per statistic; resolution = max(null-run spread, '
            '2^-23 * scale)\n')
    f.write('# model\tinput\toutput\tbaseline_max_abs\tbaseline_rms\tref_max_abs\tref_rms\t'
            'resolution_max_abs\tresolution_rms\n')
    for key, vals, res in out_rows:
      f.write('%s\t%d\t%s\t%s\n' % (key[0], key[1], key[2],
                                    '\t'.join(repr(v) for v in list(vals) + res)))
  if args.analysis:
    with open(args.analysis, 'x') as f:
      json.dump(analysis, f, indent=1)
  n_floor = sum(1 for a in analysis for s in ('max_abs', 'rms') if a[s]['floor_bound'])
  n_nav = sum(1 for a in analysis for s in ('max_abs', 'rms') if a[s]['no_added_value_over_5_2'])
  print('%d rows, %d statistics: %d floor-bound, %d with a limit at or above the section 5.2 tolerance'
        % (len(analysis), 2 * len(analysis), n_floor, n_nav))


if __name__ == '__main__':
  main()

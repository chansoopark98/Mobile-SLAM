#!/usr/bin/env python3
"""Writes tests/ml/data/model_regression_gate.tsv from the report-native-*.json files that
sr_tests_ml_models wrote for a known-good build (RM-1, docs/design/ml-runtime.md section 5.3).

The table holds, for every (model, input, output), the error the engine had when the gate was
registered. The test then requires  error <= max(2 * baseline, float32 epsilon * reference scale).
Run it once, register the sha256 of the result (build/slam-refact-full-20261019/preregistration/),
and only then let a test read the table.

  python3 -I slam_refact/tools/ml_ref/make_regression_gate.py --reports <ml_ref dir> --out <tsv>
"""
import argparse
import glob
import json
import os
import sys


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--reports', required=True, help='directory with report-native-*.json')
    ap.add_argument('--out', required=True)
    args = ap.parse_args()
    if os.path.exists(args.out):
        sys.exit('refusing to overwrite ' + args.out)
    lines = ['# model\tinput\toutput\tbaseline_max_abs\tbaseline_rms\tref_max_abs\tref_rms']
    paths = sorted(glob.glob(os.path.join(args.reports, 'report-native-*.json')))
    if len(paths) != 7:
        sys.exit('expected 7 reports, found %d' % len(paths))
    for path in paths:
        doc = json.load(open(path))
        for inp in doc['inputs']:
            for out in inp['outputs']:
                lines.append('\t'.join([
                    doc['model'], str(inp['input']), out['name'],
                    '%.6e' % out['max_abs'], '%.6e' % out['rms'],
                    '%.6e' % out['ref_max_abs'], '%.6e' % out['ref_rms']]))
    with open(args.out, 'w') as f:
        f.write('\n'.join(lines) + '\n')
    print('wrote %d rows to %s' % (len(lines) - 1, args.out))


if __name__ == '__main__':
    main()

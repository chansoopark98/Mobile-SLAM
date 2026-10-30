#!/usr/bin/env python3
"""Reference baseline of the trace-level gates: the observables of trace_metrics.py and
frame_parity.py computed on the ORIGINAL headless reference runtime runs (r1, r2; profile1 for A-*).

The output is embedded verbatim in thresholds-v1.json ("reference_baseline") at registration time,
before any engine result exists, together with the sha256 of every trace it read. Re-running this
script must reproduce the embedded values exactly (tests/test_eval.py: self-consistency).

Usage (repo root): python3 -I slam_refact/tools/harness/eval/reference_baseline.py [--out FILE]
"""
import argparse
import json
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import accuracy  # noqa: E402
import frame_parity  # noqa: E402
import trace_metrics as tm  # noqa: E402
from common import (EVIDENCE_ROOT, REFERENCE_RUN_LABELS, SCENARIOS, load_trace, percentile,  # noqa: E402
                    reference_trace_path, sha256_file)

TUM_FIXTURES = {
    'tum-room1-full': EVIDENCE_ROOT / 'reference/fixtures/tum-room1-portrait-full',
    'tum-room1-lowtex': EVIDENCE_ROOT / 'reference/fixtures/tum-room1-portrait-lowtex17-20',
}
# A-* reference set: the design's numbers (§6.6.6) include the profile1 run of tum-room1-full.
ACCURACY_RUNS = {'tum-room1-full': ('r1', 'r2', 'profile1'), 'tum-room1-lowtex': ('r1', 'r2')}


def run_observables(scenario, label, rows):
    path = reference_trace_path(scenario, label)
    obs = {
        'rows': len(rows),
        'fresh': sum(1 for r in rows if r['fresh']),
        'first_translation_i': tm.first_translation(rows),
        'first_limited': tm.first_limited(rows),
        'limited_ratio': tm.limited_ratio(rows),
        'world_points_p50': tm.world_points_p50(rows),
        'first_world_points': tm.first_world_points(rows),
        'early_burst_share_5_6': tm.early_burst_share_5_6(rows)['share_5_6'],
        'zero_step_fractions': tm.zero_step_fractions(rows),
        'publish_lag_ms': tm.publish_lag_spread_ms(rows),
        'worldpoint_padding': tm.worldpoint_padding_fraction(path),
        'status_hysteresis_pass': tm.status_hysteresis(path)['pass'],
        'runtime_invariants': tm.runtime_invariants(path),
    }
    return obs


def accuracy_observables(scenario, label):
    fx = TUM_FIXTURES[scenario]
    gt = accuracy.load_gt(fx / 'groundtruth-body.csv')
    t_ic = json.loads((fx / 'manifest.json').read_text())['t_ic']
    acc = accuracy.accuracy(load_trace(reference_trace_path(scenario, label)), gt, t_ic, 0)
    return {'coverage': acc['coverage'],
            'ape_rot_rmse_deg': acc['apeRotationDegRotationOnlyAlignment']['rmse'],
            'rpe1s_rot_rmse_deg': acc['rpe1s']['rotationDeg']['rmse'],
            'sim3_ate_rmse': acc['sim3']['ateTranslation']['rmse'],
            'sim3_scale': acc['sim3']['scale'],
            'se3_ate_rmse': acc['se3']['ateTranslation']['rmse'],
            'se3_rpe1s_trans_rmse': acc['rpe1s']['translationSe3']['rmse']}


def build():
    out = {'schema': 'slam-refact-reference-baseline-v1', 'scenarios': {}, 'trace_sha256': {}}
    for sc in SCENARIOS:
        traces = {lb: load_trace(reference_trace_path(sc, lb)) for lb in REFERENCE_RUN_LABELS}
        for lb in REFERENCE_RUN_LABELS:
            out['trace_sha256'][f'{sc}/{lb}'] = sha256_file(reference_trace_path(sc, lb))
        entry = {'runs': {lb: run_observables(sc, lb, rows) for lb, rows in traces.items()}}
        r1, r2 = traces['r1'], traces['r2']
        _, _, dr = frame_parity.matched_diffs(r1, r2)
        entry['r1_r2'] = {
            'status_agreement': frame_parity.status_agreement(r1, r2)['agreement'],
            'full_rotation_diff_p95_deg_linear': percentile(dr, 95, 'linear'),
        }
        if sc in ACCURACY_RUNS:
            entry['accuracy'] = {}
            for lb in ACCURACY_RUNS[sc]:
                entry['accuracy'][lb] = accuracy_observables(sc, lb)
                out['trace_sha256'][f'{sc}/{lb}'] = sha256_file(reference_trace_path(sc, lb))
        out['scenarios'][sc] = entry
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', type=pathlib.Path)
    a = ap.parse_args()
    text = json.dumps(build(), indent=1, sort_keys=True)
    if a.out:
        a.out.write_text(text + '\n')
    print(text)


if __name__ == '__main__':
    main()

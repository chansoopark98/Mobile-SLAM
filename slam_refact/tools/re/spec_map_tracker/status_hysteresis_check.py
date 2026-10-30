#!/usr/bin/env python3
"""Trace-level check of the recovered reference runtime NORMAL/LIMITED status machine.

Spec: slam_refact/docs/spec/02-map-tracker.md section 6.3 (VioTracker func 784,
xr-slam.dcmp lines 85056-85130 and 85912-85930). The machine only changes the
public status when a 2 s window (frame time) has elapsed since the previous
transition, so every interior status run must last >= 2 s.

Tolerances were fixed before running this script (spec section 8):
  * MIN_RUN_S = 2.0 - FRAME_SLACK_S, FRAME_SLACK_S = 0.1 s (one frame at >= 10 Hz,
    covers the one-frame publish lag and dropped frames shifting run edges).
  * The first LIMITED frame must satisfy t >= t_first_frame + MIN_RUN_S.
A run violates the model if it is shorter than MIN_RUN_S and is neither the
first nor the last run of the trace.

Usage: python3 -I status_hysteresis_check.py --runs-root DIR --out FILE.json
"""
import argparse
import json
import os
import sys

FRAME_SLACK_S = 0.1
MIN_RUN_S = 2.0 - FRAME_SLACK_S


def load_fresh(trace_path):
    rows = []
    with open(trace_path) as f:
        for line in f:
            r = json.loads(line)
            if r.get('fresh'):
                rows.append(r)
    rows.sort(key=lambda r: r['t'])
    return rows


def runs_of(rows):
    runs = []
    for r in rows:
        s = r['status']
        if runs and runs[-1]['status'] == s:
            runs[-1]['n'] += 1
            runs[-1]['t_last'] = r['t']
        else:
            runs.append({'status': s, 't_start': r['t'], 't_last': r['t'], 'i_start': r['i'], 'n': 1})
    for k in range(len(runs) - 1):
        runs[k]['duration_s'] = runs[k + 1]['t_start'] - runs[k]['t_start']
    if runs:
        runs[-1]['duration_s'] = None
    return runs


def zero_step_stats(rows):
    out = {}
    for prev, cur in zip(rows, rows[1:]):
        key = cur['status']
        d = out.setdefault(key, {'steps': 0, 'zero_position_steps': 0, 'rotation_changed_in_zero_steps': 0})
        d['steps'] += 1
        if prev['p'] == cur['p']:
            d['zero_position_steps'] += 1
            if prev['q_wxyz'] != cur['q_wxyz']:
                d['rotation_changed_in_zero_steps'] += 1
    return out


def check_run(trace_path):
    rows = load_fresh(trace_path)
    runs = runs_of(rows)
    interior = runs[1:-1]
    violations = [r for r in interior if r['duration_s'] < MIN_RUN_S]
    first_limited = next((r for r in rows if r['status'] == 'LIMITED'), None)
    first_ok = True
    if first_limited is not None:
        first_ok = first_limited['t'] - rows[0]['t'] >= MIN_RUN_S
    statuses = {}
    for r in rows:
        statuses[r['status']] = statuses.get(r['status'], 0) + 1
    durations = [r['duration_s'] for r in interior]
    return {
        'trace': trace_path,
        'fresh_frames': len(rows),
        'status_counts': statuses,
        'n_runs': len(runs),
        'n_interior_runs': len(interior),
        'min_interior_run_s': min(durations) if durations else None,
        'violations': [{k: v for k, v in r.items()} for r in violations],
        'first_limited_t': first_limited['t'] if first_limited else None,
        'first_limited_i': first_limited['i'] if first_limited else None,
        'first_limited_ok': first_ok,
        'pass': not violations and first_ok,
        'zero_position_steps_by_status': zero_step_stats(rows),
        'runs': runs,
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--runs-root', required=True)
    ap.add_argument('--out', required=True)
    a = ap.parse_args()
    if os.path.exists(a.out):
        sys.exit('refusing to overwrite ' + a.out)
    results = []
    for scen in sorted(os.listdir(a.runs_root)):
        sdir = os.path.join(a.runs_root, scen)
        if not os.path.isdir(sdir):
            continue
        for run in sorted(os.listdir(sdir)):
            if run == 'wrap1':  # instrumented run, not a reference trace (03-runtime-reference.md 9.2)
                continue
            tp = os.path.join(sdir, run, 'trace.jsonl')
            if os.path.isfile(tp):
                results.append(check_run(tp))
    summary = {
        'min_run_s': MIN_RUN_S,
        'frame_slack_s': FRAME_SLACK_S,
        'n_traces': len(results),
        'n_pass': sum(r['pass'] for r in results),
        'results': results,
    }
    with open(a.out, 'w') as f:
        json.dump(summary, f, indent=1)
    for r in results:
        print('%-60s pass=%s runs=%d min_interior=%s first_L=%s counts=%s' % (
            os.path.relpath(r['trace'], a.runs_root), r['pass'], r['n_runs'],
            None if r['min_interior_run_s'] is None else round(r['min_interior_run_s'], 3),
            r['first_limited_t'], r['status_counts']))
    print('PASS %d/%d' % (summary['n_pass'], summary['n_traces']))


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""Accuracy matrix over engine runs (accuracy-mode development, report only, no gate verdicts).

For each RUN given as SCENARIO=RUN_DIR (trace.jsonl in the original reference schema):
  * TUM scenarios (fixture with groundtruth-body.csv): the committed evaluator eval/accuracy.py —
    SE(3) scale-1 ATE (primary; metres only for metric-scale runs), Sim(3) scale + ATE (diagnostic),
    rotation APE, RPE 1 s, coverage;
  * every scenario: NORMAL ratio over fresh rows, published path length, end-start distance,
    camera height (y) range and p05..p95 spread, largest per-frame step.
The fixture is taken from slam_refact/tools/re/runtime/scenarios.json (`dataset`).

Usage (repo root):
  python3 -I slam_refact/tools/harness/eval/accuracy_matrix.py --run tum-room1-full=<dir> ... [--out-json new.json]
"""
import argparse
import json
import math
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import accuracy as acc_mod  # noqa: E402
from common import load_trace  # noqa: E402

ROOT = pathlib.Path(__file__).resolve().parents[4]
SCENARIOS = ROOT / 'slam_refact/tools/re/runtime/scenarios.json'


def pct(xs, p):
    s = sorted(xs)
    return s[min(len(s) - 1, int(math.floor((len(s) - 1) * p)))] if s else None


def trajectory_stats(rows):
    fresh = [r for r in rows if r.get('fresh')]
    pts = [r['p'] for r in fresh if r.get('p')]
    out = {'rows': len(rows), 'fresh': len(fresh),
           'normalRatio': (sum(1 for r in fresh if r.get('status') == 'NORMAL') / len(fresh)) if fresh else None}
    if len(pts) > 1:
        steps = [math.dist(a, b) for a, b in zip(pts, pts[1:])]
        ys = [p[1] for p in pts]
        out.update({'pathLength': sum(steps), 'endStart': math.dist(pts[0], pts[-1]), 'maxStep': max(steps),
                    'yMin': min(ys), 'yMax': max(ys), 'yP05': pct(ys, 0.05), 'yP95': pct(ys, 0.95)})
    return out


def evaluate(scenario, run_dir, scenarios, normal_only=False, metric_only=False):
    rows = load_trace(pathlib.Path(run_dir) / 'trace.jsonl')
    if metric_only:
        # Accuracy-mode metric output: only rows whose scale has converged count as metric poses.
        rows = [r if r.get('srMetricInitialized') else {**r, 'p': None} for r in rows]
    if normal_only:
        # LIMITED / INITIALIZING rows are not counted as tracked: drop their pose (coverage shows it).
        rows = [r if r.get('status') == 'NORMAL' else {**r, 'p': None} for r in rows]
    out = {'scenario': scenario, 'run': str(run_dir), 'trajectory': trajectory_stats(rows)}
    base = scenario.replace('-absolute', '')
    spec = scenarios.get(scenario) or scenarios.get(base)
    fx = ROOT / spec['dataset'] if spec else None
    if fx is not None and (fx / 'groundtruth-body.csv').exists():
        gt = acc_mod.load_gt(fx / 'groundtruth-body.csv')
        t_ic = json.loads((fx / 'manifest.json').read_text())['t_ic']
        a = acc_mod.accuracy(rows, gt, t_ic, 0)
        # Scale drift: Sim(3) scale of each third of the associated trajectory (diagnostic).
        assoc = [r for r in rows if r.get('p') and acc_mod.gt_lookup(gt, r['t']) is not None]
        thirds = [assoc[k * len(assoc) // 3:(k + 1) * len(assoc) // 3] for k in range(3)]
        seg_scales = []
        for seg in thirds:
            sa = acc_mod.accuracy(seg, gt, t_ic, 0) if len(seg) >= 10 else {}
            if 'sim3' in sa:
                seg_scales.append(sa['sim3']['scale'])
        if 'se3' in a:
            out['accuracy'] = {
                'coverage': a['coverage'],
                'se3AteRmse': a['se3']['ateTranslation']['rmse'],
                'sim3Scale': a['sim3']['scale'],
                'sim3AteRmse': a['sim3']['ateTranslation']['rmse'],
                'apeRotDegRmse': a['apeRotationDegRotationOnlyAlignment']['rmse'],
                'rpe1sTransSe3Rmse': a['rpe1s']['translationSe3']['rmse'],
                'rpe1sTransSim3Rmse': a['rpe1s']['translationSim3Scaled']['rmse'],
                'rpe1sRotDegRmse': a['rpe1s']['rotationDeg']['rmse'],
                'segmentSim3Scales': seg_scales,
                'scaleDrift': (max(seg_scales) / min(seg_scales) - 1.0) if len(seg_scales) == 3 and min(seg_scales) > 0 else None,
            }
    return out


def fmt(v, nd=3):
    return '-' if v is None else ('%.*f' % (nd, v))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--run', action='append', required=True, help='SCENARIO=RUN_DIR')
    ap.add_argument('--out-json', type=pathlib.Path)
    ap.add_argument('--normal-only', action='store_true', help='evaluate NORMAL rows only')
    ap.add_argument('--metric-only', action='store_true', help='evaluate rows with a converged metric scale only')
    a = ap.parse_args()
    scenarios = json.loads(SCENARIOS.read_text())
    results = []
    for spec in a.run:
        scenario, run_dir = spec.split('=', 1)
        results.append(evaluate(scenario, run_dir, scenarios, a.normal_only, a.metric_only))
    print('| scenario | run | NORMAL | coverage | SE3 ATE | Sim3 scale | Sim3 ATE | rot APE° | RPE1s Sim3 | RPE1s rot° '
          '| scale drift | path | end-start | y range | max step |')
    print('|' + '---|' * 15)
    for r in results:
        t, acc = r['trajectory'], r.get('accuracy', {})
        print('| %s | %s | %s | %s | %s | %s | %s | %s | %s | %s | %s | %s | %s | %s..%s | %s |' % (
            r['scenario'], pathlib.Path(r['run']).name, fmt(t.get('normalRatio')), fmt(acc.get('coverage')),
            fmt(acc.get('se3AteRmse')), fmt(acc.get('sim3Scale'), 4), fmt(acc.get('sim3AteRmse')),
            fmt(acc.get('apeRotDegRmse'), 2), fmt(acc.get('rpe1sTransSim3Rmse')), fmt(acc.get('rpe1sRotDegRmse'), 2),
            fmt(acc.get('scaleDrift')), fmt(t.get('pathLength'), 2), fmt(t.get('endStart')), fmt(t.get('yMin'), 2), fmt(t.get('yMax'), 2),
            fmt(t.get('maxStep'))))
    if a.out_json:
        if a.out_json.exists():
            raise SystemExit('refusing to overwrite ' + str(a.out_json))
        a.out_json.write_text(json.dumps(results, indent=1) + '\n')


if __name__ == '__main__':
    main()

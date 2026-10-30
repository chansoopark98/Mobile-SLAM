#!/usr/bin/env python3
"""US-011 columns for engine runs and the original reference runs (architecture.md §6.4).

Per scenario, for the engine run and for every original run (r1, r2, profile1 where present):
  * TUM GT accuracy with the committed evaluator (eval/accuracy.py): SE(3) scale-1 ATE translation,
    rotation APE (rotation-only alignment), RPE 1 s translation / rotation, coverage; Sim(3) scale and
    ATE separately (diagnostic only; responsive units are not metres);
  * time to first pose: first finite pose, first translation (eval/trace_metrics.first_translation),
    first world points;
  * tracking ratio: NORMAL rows / fresh rows;
  * trajectory difference vs original r1 over all frames fresh in both (nearest-rank p50/p95/max of
    position distance and rotation angle).
No gate verdicts here; gates are in parity_report.py.

Usage (repo root):
  python3 -I slam_refact/tools/harness/eval/us011_table.py --run SCENARIO=RUN_DIR ... --out-json <new file>
"""
import argparse
import json
import math
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import accuracy as acc_mod  # noqa: E402
import trace_metrics as tm  # noqa: E402
from common import load_trace, quat_angle_deg, reference_trace_path  # noqa: E402
from gates import TUM_GT  # noqa: E402
from stage_timing import nearest_rank  # noqa: E402

REF_LABELS = ('r1', 'r2', 'profile1')


def first_index(rows, pred):
    return next((r['i'] for r in rows if r.get('fresh') and pred(r)), None)


def columns(rows, scenario, base_scenario):
    fresh = [r for r in rows if r.get('fresh')]
    out = {
        'rows': len(rows), 'fresh': len(fresh),
        'first_finite_pose_i': first_index(rows, lambda r: r.get('p') is not None),
        'first_translation': tm.first_translation(rows),
        'first_world_points_i': first_index(rows, lambda r: (r.get('worldPoints') or 0) > 0),
        'tracking_ratio_normal': sum(1 for r in fresh if r.get('status') == 'NORMAL') / len(fresh) if fresh else None,
    }
    if base_scenario in TUM_GT:
        fx = TUM_GT[base_scenario]
        gt = acc_mod.load_gt(fx / 'groundtruth-body.csv')
        t_ic = json.loads((fx / 'manifest.json').read_text())['t_ic']
        a = acc_mod.accuracy(rows, gt, t_ic, 0)
        if 'se3' in a:
            out['accuracy'] = {
                'coverage': a['coverage'], 'associatedPoses': a['associatedPoses'],
                'se3_ate_translation_rmse': a['se3']['ateTranslation']['rmse'],
                'se3_ate_translation_p95': a['se3']['ateTranslation']['p95'],
                'ape_rotation_deg_rmse': a['apeRotationDegRotationOnlyAlignment']['rmse'],
                'rpe1s_translation_se3_rmse': a['rpe1s']['translationSe3']['rmse'],
                'rpe1s_rotation_deg_rmse': a['rpe1s']['rotationDeg']['rmse'],
                'sim3_scale_diagnostic': a['sim3']['scale'],
                'sim3_ate_translation_rmse_diagnostic': a['sim3']['ateTranslation']['rmse'],
            }
        else:
            out['accuracy'] = {'coverage': a['coverage'], 'associatedPoses': a['associatedPoses']}
    return out


def trajectory_diff(rows, ref):
    by_i = {r['i']: r for r in ref}
    pos, rot = [], []
    for r in rows:
        o = by_i.get(r['i'])
        if not (r.get('fresh') and r.get('p') and o and o.get('fresh') and o.get('p')):
            continue
        pos.append(math.dist(r['p'], o['p']))
        rot.append(quat_angle_deg(r['q_wxyz'], o['q_wxyz']))
    return {'frames': len(pos), 'pos_p50': nearest_rank(pos, 50), 'pos_p95': nearest_rank(pos, 95),
            'pos_max': max(pos) if pos else None, 'rot_deg_p50': nearest_rank(rot, 50),
            'rot_deg_p95': nearest_rank(rot, 95), 'rot_deg_max': max(rot) if rot else None}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--run', action='append', required=True, help='SCENARIO=RUN_DIR (engine)')
    ap.add_argument('--out-json', type=pathlib.Path, required=True)
    a = ap.parse_args()
    if a.out_json.exists():
        raise SystemExit('refusing to overwrite ' + str(a.out_json))
    report = {'schema': 'slam-refact-us011-v1', 'scenarios': {}}
    for item in a.run:
        scenario, _, run_dir = item.partition('=')
        base = scenario[:-len('-absolute')] if scenario.endswith('-absolute') else scenario
        eng = load_trace(pathlib.Path(run_dir) / 'trace.jsonl')
        entry = {'engine_run': run_dir, 'engine': columns(eng, scenario, base), 'reference': {}}
        refs = {}
        for label in REF_LABELS:
            p = reference_trace_path(scenario, label)
            if pathlib.Path(p).exists():
                refs[label] = load_trace(p)
                entry['reference'][label] = columns(refs[label], scenario, base)
        if 'r1' in refs:
            entry['engine_vs_r1'] = trajectory_diff(eng, refs['r1'])
            if 'r2' in refs:
                entry['r2_vs_r1'] = trajectory_diff(refs['r2'], refs['r1'])
        report['scenarios'][scenario] = entry
    a.out_json.write_text(json.dumps(report, indent=1) + '\n')
    print(json.dumps({s: {'engine_ate': (e['engine'].get('accuracy') or {}).get('se3_ate_translation_rmse'),
                          'vs_r1': e.get('engine_vs_r1')} for s, e in report['scenarios'].items()}))


if __name__ == '__main__':
    main()

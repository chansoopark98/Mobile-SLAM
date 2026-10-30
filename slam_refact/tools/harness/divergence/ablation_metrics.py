#!/usr/bin/env python3
"""Metrics of the failing gates for ablation replays (diagnosis only; evaluator functions reused verbatim).

For each <scenario>/<run>/<variant> replay under OUT/replays, computes against the original run's own trace
(reference/runs/<scenario>/<run>) and against r1 (the gate reference):
  MT-P7-4 rot p95 (frame_parity.matched_diffs, normalized angle, linear percentile),
  V-PD window stats (thresholds-v1.json window of the scenario, nearest rank),
  MT-P7-2 LIMITED ratio, G-M2(b) early burst share 5/6, V-EV4 zero-step fractions per status,
  first differing frames vs the run's own trace (exact_input_report.compare) and vs variant E (compare_engines).
Also the same observables for original traces (r1, r2, the runs themselves) and the gate replays (c22-r2).
usage: ablation_metrics.py OUT scenario/run ... > OUT/metrics.json
"""
import json
import pathlib
import sys

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / 'eval'))
sys.path.insert(0, str(HERE))
import exact_input_report as eir  # noqa: E402
import frame_parity as fp  # noqa: E402
import trace_metrics as tm  # noqa: E402
from common import REFERENCE_RUNS, THRESHOLDS_PATH, load_trace, percentile  # noqa: E402

TH = json.loads(THRESHOLDS_PATH.read_text())
VPD = next(g for g in TH['gates'] if g['id'] == 'V-PD')['tolerance']
ENGINE_RUNS = REFERENCE_RUNS.parent.parent / 'engine-runs'


def observables(rows, ref_rows, sc):
    _, _, dr = fp.matched_diffs(rows, ref_rows, angle=fp.normalized_angle_deg)
    w = VPD['windows'][sc]['window']
    ws = fp.window_stats(rows, ref_rows, w[0], w[1], VPD['percentile'])
    return {'rot_p95_full': percentile(dr, 95, 'linear'), 'matched': len(dr),
            'vpd': {k: ws[k] for k in ('frames_compared', 'pos_p95', 'pos_max', 'rot_deg_p95', 'rot_deg_max')}}


def self_obs(rows):
    return {'limited_ratio': tm.limited_ratio(rows), 'share_5_6': tm.early_burst_share_5_6(rows)['share_5_6'],
            'zero_step': tm.zero_step_fractions(rows), 'first_limited': tm.first_limited(rows),
            'first_world_points': tm.first_world_points(rows)}


def main():
    out = pathlib.Path(sys.argv[1])
    res = {'thresholds': str(THRESHOLDS_PATH), 'runs': {}}
    cache = {}

    def ref(sc, lb):
        k = (sc, lb)
        if k not in cache:
            cache[k] = load_trace(REFERENCE_RUNS / sc / lb / 'trace.jsonl')
        return cache[k]

    for sr in sys.argv[2:]:
        sc, run = sr.split('/')
        own, r1 = ref(sc, run), ref(sc, 'r1')
        entry = {'original': {lb: dict(self_obs(ref(sc, lb)), vs_r1=observables(ref(sc, lb), r1, sc))
                              for lb in ('r1', 'r2', run)}}
        gate = ENGINE_RUNS / sc / 'c22-r2' / 'trace.jsonl'
        gate_rows = load_trace(gate) if gate.exists() else None
        if gate_rows:
            entry['gate_c22_r2'] = dict(self_obs(gate_rows), vs_r1=observables(gate_rows, r1, sc),
                                        vs_own=observables(gate_rows, own, sc))
        vdir = out / 'replays' / sc / run
        e_rows = load_trace(vdir / 'E' / 'trace.jsonl')
        variants = {}
        for vd in sorted(p for p in vdir.iterdir() if p.is_dir()):
            rows = load_trace(vd / 'trace.jsonl')
            c = eir.compare([x for x in rows if x.get('fresh') and x.get('p') is not None], own)
            v = dict(self_obs(rows), vs_own=observables(rows, own, sc), vs_r1=observables(rows, r1, sc),
                     first_vs_own=c['first'], bitwise_equal_frac_vs_own=c['fraction_within']['pose_bitwise_equal'],
                     vs_E=eir.compare_engines(rows, e_rows))
            if gate_rows and vd.name == 'TBSPD':
                v['vs_gate_c22_r2'] = eir.compare_engines(rows, gate_rows)
            variants[vd.name] = v
        entry['variants'] = variants
        res['runs'][sr] = entry
    json.dump(res, sys.stdout, indent=1, default=str)


if __name__ == '__main__':
    main()

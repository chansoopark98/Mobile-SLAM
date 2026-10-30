#!/usr/bin/env python3
"""MS-RT observables for absolute-scale ablation replays (diagnosis only; gates.ms_rt_verdict reused verbatim).

For each <scenario>/<run>/<variant> replay under OUT/replays: first scale latch (trace_metrics.scale_latches),
latch count, the MS-RT verdict against r1/r2 (as the gate) and the first differing frames vs the original run's own
trace. Also the originals (r1, r2, every recorded run given) evaluated as MS-RT candidates against r1/r2.
usage: ablation_metrics_absolute.py OUT scenario/run ... > OUT/metrics-absolute.json
"""
import json
import pathlib
import sys

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / 'eval'))
sys.path.insert(0, str(HERE))
import exact_input_report as eir  # noqa: E402
import gates  # noqa: E402
import trace_metrics as tm  # noqa: E402
from common import REFERENCE_RUNS, load_trace  # noqa: E402

ENGINE_RUNS = REFERENCE_RUNS.parent.parent / 'engine-runs'


def first_latch(rows):
    lat = tm.scale_latches(rows)
    return (lat[0] if lat else None), len(lat)


def main():
    out = pathlib.Path(sys.argv[1])
    res = {}
    for sr in sys.argv[2:]:
        sc, run = sr.split('/')
        r1, _ = first_latch(load_trace(REFERENCE_RUNS / sc / 'r1' / 'trace.jsonl'))
        r2, _ = first_latch(load_trace(REFERENCE_RUNS / sc / 'r2' / 'trace.jsonl'))
        own_rows = load_trace(REFERENCE_RUNS / sc / run / 'trace.jsonl')
        own, own_n = first_latch(own_rows)
        entry = {'r1_latch': r1, 'r2_latch': r2, 'own_latch': own, 'own_latches': own_n,
                 'own_as_candidate_vs_r1r2': gates.ms_rt_verdict(own, r1, r2)}
        g = ENGINE_RUNS / sc / 'c22-r2' / 'trace.jsonl'
        if g.exists():
            gl, gn = first_latch(load_trace(g))
            entry['gate_c22_r2'] = {'latch': gl, 'latches': gn, 'ms_rt_vs_r1r2': gates.ms_rt_verdict(gl, r1, r2)}
        variants = {}
        for vd in sorted(p for p in (out / 'replays' / sc / run).iterdir() if p.is_dir()):
            rows = load_trace(vd / 'trace.jsonl')
            lat, n = first_latch(rows)
            c = eir.compare([x for x in rows if x.get('fresh') and x.get('p') is not None], own_rows)
            variants[vd.name] = {'latch': lat, 'latches': n, 'ms_rt_vs_r1r2': gates.ms_rt_verdict(lat, r1, r2),
                                 'latch_i_equal_own': (lat or {}).get('i') == (own or {}).get('i'),
                                 'scale_rel_diff_vs_own': (abs(lat['scale_proxy'] - own['scale_proxy']) / own['scale_proxy']
                                                           if lat and own else None),
                                 'first_vs_own': c['first'], 'max_vs_own': c['max'],
                                 'bitwise_equal_frac_vs_own': c['fraction_within']['pose_bitwise_equal']}
        entry['variants'] = variants
        res[sr] = entry
    json.dump(res, sys.stdout, indent=1, default=str)


if __name__ == '__main__':
    main()

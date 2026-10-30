#!/usr/bin/env python3
"""it-scale-gate.v4 REPORT-only runs (never counted in the verdict, a failure here never makes the sequence INVALID).

  python3 -I v4_report_runs.py --pipe <e5_pipeline.py output dir (--arms 1)> --seq <scene dir> --engine <sr_e5_replay> --prefix-engine <sr_e5_replay_prefix> --out <NEW report.json>

  arm 0         the engine with the world tracker OFF (public extrapolation branch, shipped default) on the exact-input bundle of every original (o1..o5)
  prefix        the PRE-FIX engine (before US015-CC-1, git 19fd011) with the world tracker ON on the bundle of o1: a realistic defect, not a synthetic mutation
  cross pairs   the gate-arm engine run of original i against original j (j != i): the same engine output measured against a different realisation of the
                original (the null contains input-realisation variation that the exact-input own pair lacks)
Each entry carries the pair statistics like e5_pipeline.py (events, vs_gt_rho) or {"error": ...}.  Development/gate tool.
"""
import argparse, json, pathlib, subprocess, sys
from concurrent.futures import ThreadPoolExecutor

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE / '../../harness/e5'))
import e5_extract as ex, e5_compare as cmp  # noqa: E402

ap = argparse.ArgumentParser()
ap.add_argument('--pipe', required=True); ap.add_argument('--seq', required=True); ap.add_argument('--engine', required=True)
ap.add_argument('--prefix-engine', required=True); ap.add_argument('--out', required=True); ap.add_argument('--k', type=int, default=5)
a = ap.parse_args()
pipe, seq, out = pathlib.Path(a.pipe).resolve(), pathlib.Path(a.seq).resolve(), pathlib.Path(a.out).resolve()
if out.exists():
    sys.exit(f'refusing existing {out}')
name = 'e5-' + seq.name
gtf = json.load(open(seq / 'gt.json'))['frames']
keys = [f'o{k}' for k in range(1, a.k + 1)]
rep = {'schema': 'e5-report-v4', 'arm0': {}, 'prefix': None, 'cross_pairs': {}}
try:
    orig = {k: ex.original(str(pipe / 'runs' / name / k)) for k in keys}
except Exception as e:  # noqa: BLE001
    out.write_text(json.dumps({'schema': 'e5-report-v4', 'error': f'originals unreadable: {e}'}))
    sys.exit(0)


def stats(k, d):
    s = ex.engine(str(d))
    m = cmp.pair(orig[k], s, orig[k]['cam_q'], s['cam_q'])
    m['events'] = [list(e) for e in s['events']]; m['vs_gt_rho'] = cmp.vs_gt(s, s['cam_q'], gtf)
    return m


def run(job):
    label, eng, k, arm = job
    d = pipe / 'eng' / label
    try:
        r = subprocess.run([eng, '--bundle', str(pipe / 'bundles' / name / f'ex-{k}'), '--out', str(d), '--targets', str(seq / 'target'), '--world-tracker', str(arm)],
                           capture_output=True, text=True, timeout=1200)
        (pipe / 'logs').mkdir(exist_ok=True)
        (pipe / 'logs' / f'report-{label}.log').write_text(f'rc={r.returncode}\n{r.stdout}\n--- stderr\n{r.stderr}\n')
        if r.returncode:
            return label, {'error': f'engine rc={r.returncode}'}
        return label, stats(k, d)
    except Exception as e:  # noqa: BLE001
        return label, {'error': str(e)}


jobs = [(f'{k}-wt0', a.engine, k, 0) for k in keys] + [('o1-prefix-wt1', a.prefix_engine, 'o1', 1)]
with ThreadPoolExecutor(max_workers=2) as pool:
    for label, m in pool.map(run, jobs):
        if label == 'o1-prefix-wt1':
            rep['prefix'] = m
        else:
            rep['arm0'][label] = m
for i in keys:
    d = pipe / 'eng' / f'{i}-wt1'
    for j in keys:
        if i == j:
            continue
        try:
            s = ex.engine(str(d))
            m = cmp.pair(orig[j], s, orig[j]['cam_q'], s['cam_q'])
            m['events'] = [list(e) for e in s['events']]; m['vs_gt_rho'] = cmp.vs_gt(s, s['cam_q'], gtf)
            rep['cross_pairs'][f'e{i}-vs-{j}'] = m
        except Exception as e:  # noqa: BLE001
            rep['cross_pairs'][f'e{i}-vs-{j}'] = {'error': str(e)}
out.write_text(json.dumps(rep))
print('report written', out)

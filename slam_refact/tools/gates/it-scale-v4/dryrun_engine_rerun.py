#!/usr/bin/env python3
"""it-scale-gate.v4 dry run (stage 1b): replay an EXISTING dev e5_pipeline output (3 original runs, exact-input bundles) with a new engine binary and judge it
with the v4 rule (N = 3).  No original SDK run, no registered data.

  python3 -I dryrun_engine_rerun.py --dev-pipe <dev pipe dir> --seq <dev scene dir> --engine <sr_e5_replay> --prefix-engine <sr_e5_replay_prefix> --out <NEW dir>

Copies bundles/ and runs/ of the dev pipe into --out, runs the engine on every ex-oK bundle at the GATE arm (world tracker ON, 2 in parallel),
writes metrics.json like e5_pipeline.py step 4 (gate arm only), then v4_report_runs.py (REPORT: arm 0, pre-fix engine, cross pairs; --k 3),
v4_controls.py (arm 1) and e5_gate_rule_v4.py judge --n-orig 3 --report.  Development tool.
"""
import argparse, json, pathlib, shutil, subprocess, sys
from concurrent.futures import ThreadPoolExecutor

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE / '../../harness/e5'))
import e5_extract as ex, e5_compare as cmp  # noqa: E402

ap = argparse.ArgumentParser()
ap.add_argument('--dev-pipe', required=True); ap.add_argument('--seq', required=True); ap.add_argument('--engine', required=True); ap.add_argument('--prefix-engine', required=True); ap.add_argument('--out', required=True)
a = ap.parse_args()
dev, seq, out = pathlib.Path(a.dev_pipe).resolve(), pathlib.Path(a.seq).resolve(), pathlib.Path(a.out).resolve()
name = 'e5-' + seq.name
out.mkdir(parents=True)
for d in ('bundles', 'runs'):
    shutil.copytree(dev / d, out / d)
(out / 'eng').mkdir(); (out / 'logs').mkdir()
K = 3
jobs = [(f'o{k}-wt1', k, 1) for k in range(1, K + 1)]


def run(job):
    label, k, arm = job
    r = subprocess.run([a.engine, '--bundle', str(out / 'bundles' / name / f'ex-o{k}'), '--out', str(out / 'eng' / label), '--targets', str(seq / 'target'),
                        '--world-tracker', str(arm)], capture_output=True, text=True)
    (out / 'logs' / f'eng-{label}.log').write_text(f'rc={r.returncode}\n{r.stdout}\n{r.stderr}')
    return label, r.returncode


with ThreadPoolExecutor(max_workers=2) as pool:
    for label, rc in pool.map(run, jobs):
        if rc:
            sys.exit(f'engine {label} rc={rc}')
gtp = seq / 'gt.json'
gtf = json.load(open(gtp))['frames']
orig = {f'o{k}': ex.original(str(out / 'runs' / name / f'o{k}')) for k in range(1, K + 1)}
M = {'schema': 'e5-metrics-v1', 'seq': str(seq), 'k': K, 'arms': [1], 'mutations': [], 'gt_runs': cmp.gt_runs(cmp.gt_vis(gtp), 5, 15), 'original': {}, 'null': [], 'engine': {}}
for k, s in orig.items():
    M['original'][k] = {'events': [list(e) for e in s['events']], 'detected': sum(s['detected']), 'processed': sum(s['processed']), 'vs_gt_rho': cmp.vs_gt(s, s['cam_q'], gtf)}
keys = sorted(orig)
for i, x in enumerate(keys):
    for y in keys[i + 1:]:
        m = cmp.pair(orig[x], orig[y], orig[x]['cam_q'], orig[y]['cam_q']); m['pair'] = [x, y]; M['null'].append(m)
for label, k, arm in jobs:
    s = ex.engine(str(out / 'eng' / label))
    m = cmp.pair(orig[f'o{k}'], s, orig[f'o{k}']['cam_q'], s['cam_q'])
    m['events'] = [list(e) for e in s['events']]; m['detected'] = sum(s['detected']); m['vs_gt_rho'] = cmp.vs_gt(s, s['cam_q'], gtf)
    M['engine'][label] = m
(out / 'metrics.json').write_text(json.dumps(M, indent=1))
r = subprocess.run(['python3', '-I', str(HERE / 'v4_report_runs.py'), '--pipe', str(out), '--seq', str(seq), '--engine', a.engine, '--prefix-engine', a.prefix_engine, '--out', str(out / 'report.json'), '--k', str(K)], capture_output=True, text=True)
print(r.stdout, r.stderr)
r = subprocess.run(['python3', '-I', str(HERE / 'v4_controls.py'), '--pipe', str(out), '--seq', str(seq), '--engine', a.engine, '--out', str(out / 'controls.json')], capture_output=True, text=True)
print(r.stdout, r.stderr)
r = subprocess.run(['python3', '-I', str(HERE / 'e5_gate_rule_v4.py'), 'judge', '--metrics', str(out / 'metrics.json'), '--gt', str(gtp), '--controls', str(out / 'controls.json'),
                    '--out', str(out / 'verdict.json'), '--n-orig', '3', '--report', str(out / 'report.json')], capture_output=True, text=True)
print('judge:', r.stdout.strip(), r.stderr.strip())

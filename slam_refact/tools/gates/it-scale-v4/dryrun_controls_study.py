#!/usr/bin/env python3
"""it-scale-gate.v4 dry run (stage 1c): effect sizes of candidate positive controls against the KNOWN in-run-null tolerances of sequences that already exist.

  python3 -I dryrun_controls_study.py --out <NEW dir> --engine <sr_e5_replay (d7629f6)> --prefix-engine <sr_e5_replay (19fd011, pre-ROI-fix)> [--jobs 3]

Sequences (NOT v4 registration seeds; their originals exist): dev seed 202, dev seed 101, and the two v3 registered sequences seq-a, seq-b
(consumed by v3, `build/slam-refact-pkgh-20261022/it-scale-v3-run1`).  For each, on the exact-input bundle of o1, world tracker ON (the gate arm):
  focal-1.10, focal-1.25, drop-every:2, drop-every:3   (engine d7629f6, input-side mutations on a bundle copy / --mutation)
  prefix-engine                                        (engine 19fd011 = before US015-CC-1, no mutation: the realistic v3-style defect)
Each result is evaluated with e5_gate_rule_v4.evaluate_pair against the tolerances the v4 rule derives from that sequence's three existing original
runs (N = 3, 3 null pairs): failed checks and, per check, value / T (> 1 = detected, the margin is the ratio).  No registered data, no SDK run.
"""
import argparse, json, pathlib, shutil, subprocess, sys
from concurrent.futures import ThreadPoolExecutor

HERE = pathlib.Path(__file__).resolve().parent
ROOT = HERE.parents[3]
sys.path.insert(0, str(HERE)); sys.path.insert(0, str(ROOT / 'slam_refact/tools/harness/e5'))
import e5_gate_rule_v4 as r4, e5_extract as ex, e5_compare as cmp  # noqa: E402

FX = 506.4103889465332
V3 = ROOT / 'build/slam-refact-pkgh-20261022/it-scale-v3-run1/gate'
DEV = ROOT / 'build/slam-refact-pkgh-20261022/dev'
SEQS = {'dev-202': (DEV / 'pipe-202', DEV / 'seq-202'), 'dev-101': (DEV / 'pipe-101', DEV / 'seq-101'),
        'v3-a': (V3 / 'pipe-seq-a', V3 / 'seq-a'), 'v3-b': (V3 / 'pipe-seq-b', V3 / 'seq-b')}
ap = argparse.ArgumentParser()
ap.add_argument('--out', required=True); ap.add_argument('--engine', required=True); ap.add_argument('--prefix-engine', required=True); ap.add_argument('--jobs', type=int, default=3)
a = ap.parse_args()
out = pathlib.Path(a.out).resolve(); out.mkdir(parents=True)
jobs = []
for sq, (pipe, seq) in SEQS.items():
    name = 'e5-' + seq.name
    src = pipe / 'bundles' / name / 'ex-o1'
    (out / sq).mkdir()
    for f in (1.10, 1.25):
        b = out / sq / f'bundle-focal-{f}'
        shutil.copytree(src, b)
        subprocess.run(['chmod', '-R', 'u+w', str(b)], check=True)
        with open(b / 'engine-config.tsv', 'a') as fh:
            fh.write('intrinsics\t%r %r 240.0 320.0\n' % (FX * f, FX * f))
        jobs.append((sq, f'focal-{f:.2f}', a.engine, b, []))
    jobs.append((sq, 'drop-every:2', a.engine, src, ['--mutation', 'drop-every:2']))
    jobs.append((sq, 'drop-every:3', a.engine, src, ['--mutation', 'drop-every:3']))
    jobs.append((sq, 'prefix-engine', a.prefix_engine, src, []))


def run(j):
    sq, label, eng, bundle, extra = j
    pipe, seq = SEQS[sq]
    d = out / sq / ('eng-' + label.replace(':', '-'))
    r = subprocess.run([eng, '--bundle', str(bundle), '--out', str(d), '--targets', str(seq / 'target'), '--world-tracker', '1'] + extra, capture_output=True, text=True)
    return sq, label, d, r.returncode


res = {}
with ThreadPoolExecutor(max_workers=a.jobs) as pool:
    done = list(pool.map(run, jobs))
for sq, (pipe, seq) in SEQS.items():
    M = json.load(open(pipe / 'metrics.json')); vis = cmp.gt_vis(seq / 'gt.json'); gtf = json.load(open(seq / 'gt.json'))['frames']
    tol = r4.tolerances(M, vis, 3)
    o = ex.original(str(pipe / 'runs' / ('e5-' + seq.name) / 'o1'))
    res[sq] = {'tolerance': tol['T'], 'resolution': tol['R'], 'judged_anchors': tol['judged'], 'controls': {}}
    for s2, label, d, rc in done:
        if s2 != sq:
            continue
        if rc:
            res[sq]['controls'][label] = {'engine_rc': rc}
            continue
        e = ex.engine(str(d))
        m = cmp.pair(o, e, o['cam_q'], e['cam_q']); m['events'] = [list(x) for x in e['events']]; m['vs_gt_rho'] = cmp.vs_gt(e, e['cam_q'], gtf)
        r = r4.evaluate_pair(M['original']['o1']['events'], m['events'], m, vis, tol)
        over_T = {c: (None if v is None else round(v / 3.0, 2)) for c, v in r['ratio_to_resolution'].items()}
        res[sq]['controls'][label] = {'detected': not r['pass'], 'failed': sorted(c for c, ok in r['checks'].items() if not ok), 'value_over_T': over_T,
                                      'anchor_diffs': r['values']['anchor_diffs'], 'rho_p95': r['values']['rho_rel'], 'dir_p95': r['values']['dir_deg'],
                                      'relrot_p95': r['values']['relrot_deg'], 'vs_gt_p95': r['values']['vs_gt_rho_p95']}
(out / 'controls-study.json').write_text(json.dumps(res, indent=1))
for sq, r in res.items():
    print(sq, 'T anchors', r['tolerance']['anchor'], 'T cont', {k: round(v, 4) for k, v in r['tolerance']['cont'].items()}, 'T gt', round(r['tolerance']['vs_gt_rho_p95'], 4))
    for lab, c in r['controls'].items():
        if 'engine_rc' in c:
            print('  ', lab, 'ENGINE FAILED', c); continue
        best = {k: v for k, v in c['value_over_T'].items() if v is not None and v > 1}
        print('  ', lab, 'DETECTED' if c['detected'] else 'not detected', 'x T:', best or max((v, k) for k, v in c['value_over_T'].items() if v is not None))

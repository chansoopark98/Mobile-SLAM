#!/usr/bin/env python3
"""E5 pipeline for ONE synthetic sequence (scene.py output): K original-SDK runs, exact-input engine bundles, engine runs, raw metrics.

  python3 -I e5_pipeline.py --seq <scene dir> --out <NEW dir> --engine <sr_e5_replay> [--k 3] [--arms 0,1] [--engine-jobs 2]
                            [--mutation <m> ...] [--no-lock]

Stages (all in one call): (1) original SDK x K inside `flock build/c1-chromium.lock` (headless Chromium, ephemeral 127.0.0.1 server,
external requests aborted, run_original.mjs with --memprobe --record-inputs --memprobe-detail 12); (2) per original a base bundle
(prepare_bundle.py) and the exact-input bundle (divergence/make_variant.py --capture-binding --capture-wallclock --queue-times
--capture-id-seed); (3) the engine on every bundle for every world-tracker arm (--world-tracker 0|1, targets loaded); (4) metrics.json:
engine-vs-own-original pairs (exact input), original-vs-original null pairs, both against the GT, with the raw per-pair statistics
(no verdict: the verdict is the gate's frozen rule, e5_gate_rule.py).  Output only under the given NEW directory.
Development/gate tool (desktop, synthetic input); not product code.
"""
import argparse, json, os, pathlib, subprocess, sys, time
from concurrent.futures import ThreadPoolExecutor

HERE = pathlib.Path(__file__).resolve().parent
ROOT = HERE.parents[3]
sys.path.insert(0, str(HERE))
import e5_extract as ex, e5_compare as cmp  # noqa: E402


def sh(cmd, env=None, log=None):
    r = subprocess.run(cmd, env=env, capture_output=True, text=True)
    if log:
        pathlib.Path(log).write_text('$ ' + ' '.join(map(str, cmd)) + '\n' + r.stdout + '\n--- stderr\n' + r.stderr + f'\nrc={r.returncode}\n')
    return r


def main(argv):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('--seq', required=True)
    ap.add_argument('--out', required=True)
    ap.add_argument('--engine', required=True)
    ap.add_argument('--k', type=int, default=3)
    ap.add_argument('--arms', default='0,1')
    ap.add_argument('--engine-jobs', type=int, default=2)
    ap.add_argument('--mutation', action='append', default=[], help='sr_e5_replay --mutation value; extra arm runs "m<i>-wt0"')
    ap.add_argument('--no-lock', action='store_true')
    a = ap.parse_args(argv)
    seq, out = pathlib.Path(a.seq).resolve(), pathlib.Path(a.out).resolve()
    if out.exists():
        print('refusing existing ' + str(out), file=sys.stderr); return 2
    out.mkdir(parents=True)
    name = 'e5-' + seq.name
    scen = out / 'scenarios.json'
    scen.write_text(json.dumps({name: {'kind': 'tum', 'dataset': str(seq.relative_to(ROOT)), 'ua': 'reduced', 'scale': 'responsive',
                                       'note': 'E5 synthetic room (package H)'}}))
    runs, bundles, eng = out / 'runs', out / 'bundles', out / 'eng'
    for d in (runs, bundles, eng):
        d.mkdir()
    (out / 'logs').mkdir()
    env = dict(os.environ, E5_SCENARIOS=str(scen))
    lock = [] if a.no_lock else ['flock', '-w', '28800', str(ROOT / 'build/c1-chromium.lock')]
    arms = [int(v) for v in a.arms.split(',') if v != '']
    t0 = time.time()
    # (1) original runs
    for k in range(1, a.k + 1):
        r = sh(lock + ['timeout', '1800', 'node', str(HERE / 'run_original.mjs'), '--scenario', name, '--label', f'o{k}', '--out-root', str(runs),
                       '--image-targets', str(seq / 'target'), '--memprobe', '--record-inputs', '--memprobe-detail', '12'], env, out / 'logs' / f'orig-o{k}.log')
        if r.returncode != 0:
            print(f'original run o{k} failed rc={r.returncode} (see logs)', file=sys.stderr); return 3
    # (2) bundles
    sensors = runs / name / 'engine-input-sensors.jsonl.gz'
    for k in range(1, a.k + 1):
        base = ['python3', '-I', str(HERE / 'prepare_bundle.py'), '--scenarios-file', str(scen), '--runs-dir', str(runs), '--bundles-dir', str(bundles),
                '--scenario', name, '--label', f'base-o{k}', '--sensors', str(sensors), '--drop-mask-from', str(runs / name / f'o{k}' / 'trace.jsonl')]
        if sh(base, None, out / 'logs' / f'bundle-base-o{k}.log').returncode != 0:
            print('prepare_bundle failed', file=sys.stderr); return 4
        mv = ['python3', '-I', str(ROOT / 'slam_refact/tools/harness/divergence/make_variant.py'), '--base', str(bundles / name / f'base-o{k}'),
              '--out', str(bundles / name / f'ex-o{k}'), '--capture', str(runs / name / f'o{k}'), '--capture-binding', '--capture-wallclock',
              '--queue-times', '--capture-id-seed']
        if sh(mv, None, out / 'logs' / f'bundle-ex-o{k}.log').returncode != 0:
            print('make_variant failed', file=sys.stderr); return 4
    # (3) engine runs
    jobs = []
    for k in range(1, a.k + 1):
        for arm in arms:
            jobs.append((f'o{k}-wt{arm}', ['--world-tracker', str(arm)], k))
        for i, m in enumerate(a.mutation):
            jobs.append((f'o{k}-m{i}-wt0', ['--world-tracker', '0', '--mutation', m], k))

    def run_eng(job):
        label, extra, k = job
        cmd = [a.engine, '--bundle', str(bundles / name / f'ex-o{k}'), '--out', str(eng / label), '--targets', str(seq / 'target')] + extra
        return label, sh(cmd, None, out / 'logs' / f'eng-{label}.log').returncode
    with ThreadPoolExecutor(max_workers=a.engine_jobs) as pool:
        for label, rc in pool.map(run_eng, jobs):
            if rc != 0:
                print(f'engine run {label} failed rc={rc}', file=sys.stderr); return 5
    # (4) metrics
    orig = {k: ex.original(str(runs / name / f'o{k}')) for k in range(1, a.k + 1)}
    gtp = seq / 'gt.json'
    gtf = json.load(open(gtp))['frames']
    res = {'schema': 'e5-metrics-v1', 'seq': str(seq), 'k': a.k, 'arms': arms, 'mutations': a.mutation, 'wall_s': time.time() - t0,
           'gt_runs': cmp.gt_runs(cmp.gt_vis(gtp), 5, 15), 'original': {}, 'null': [], 'engine': {}}
    for k, s in orig.items():
        res['original'][f'o{k}'] = {'events': [list(e) for e in s['events']], 'detected': sum(s['detected']), 'processed': sum(s['processed']),
                                    'vs_gt_rho': cmp.vs_gt(s, s['cam_q'], gtf)}
    for k in range(1, a.k + 1):
        for l in range(k + 1, a.k + 1):
            m = cmp.pair(orig[k], orig[l], orig[k]['cam_q'], orig[l]['cam_q'])
            m['pair'] = [f'o{k}', f'o{l}']
            res['null'].append(m)
    for label, extra, k in jobs:
        s = ex.engine(str(eng / label))
        m = cmp.pair(orig[k], s, orig[k]['cam_q'], s['cam_q'])
        m['events'] = [list(e) for e in s['events']]; m['detected'] = sum(s['detected']); m['vs_gt_rho'] = cmp.vs_gt(s, s['cam_q'], gtf)
        res['engine'][label] = m
    (out / 'metrics.json').write_text(json.dumps(res, indent=1))
    print('done', out, f'{time.time() - t0:.0f}s')
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))

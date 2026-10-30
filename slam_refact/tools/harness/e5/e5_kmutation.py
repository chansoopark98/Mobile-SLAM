#!/usr/bin/env python3
"""E5 dev tool (mutation power): re-run the engine on the exact-input bundles of an existing e5_pipeline.py output with a scaled
focal length (engine-config.tsv `intrinsics` override = scale x 506.4103889 at the SDK's 480x640 K, principal point 240/320), and write the
same per-pair statistics as the pipeline into <pipe dir>/kmut-<factor>.json.  Input-side mutation of the engine arm only (bundle copy).

  python3 -I e5_kmutation.py --pipe <pipe dir> --engine <sr_e5_replay> --seq <scene dir> --factor 1.10 [--k 3]
"""
import argparse, json, pathlib, shutil, subprocess, sys

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import e5_extract as ex, e5_compare as cmp  # noqa: E402

ap = argparse.ArgumentParser()
ap.add_argument('--pipe', required=True); ap.add_argument('--engine', required=True); ap.add_argument('--seq', required=True)
ap.add_argument('--factor', type=float, default=1.10); ap.add_argument('--k', type=int, default=3)
a = ap.parse_args()
pipe, seq = pathlib.Path(a.pipe), pathlib.Path(a.seq)
name = 'e5-' + seq.name
fx = 506.4103889465332 * a.factor
res = {'factor': a.factor, 'pairs': {}}
for k in range(1, a.k + 1):
    src = pipe / 'bundles' / name / f'ex-o{k}'
    dst = pipe / 'bundles' / name / f'ex-o{k}-k{a.factor}'
    shutil.copytree(src, dst)
    with open(dst / 'engine-config.tsv', 'a') as f:
        f.write('intrinsics\t%r %r 240.0 320.0\n' % (fx, fx))
    outd = pipe / 'eng' / f'o{k}-kmut{a.factor}-wt0'
    r = subprocess.run([a.engine, '--bundle', str(dst), '--out', str(outd), '--targets', str(seq / 'target'), '--world-tracker', '0'], capture_output=True, text=True)
    if r.returncode:
        sys.exit(f'engine rc={r.returncode}: {r.stderr[-300:]}')
    o = ex.original(str(pipe / 'runs' / name / f'o{k}')); e = ex.engine(str(outd))
    m = cmp.pair(o, e, o['cam_q'], e['cam_q']); m['events'] = [list(x) for x in e['events']]
    m['vs_gt_rho'] = cmp.vs_gt(e, e['cam_q'], json.load(open(seq / 'gt.json'))['frames'])
    res['pairs'][f'o{k}'] = m
(pipe / f'kmut-{a.factor}.json').write_text(json.dumps(res, indent=1))
print(json.dumps({k: {x: v[x] for x in ('state_disagree', 'rho_rel', 'dir_deg', 'vs_gt_rho')} for k, v in res['pairs'].items()}))

#!/usr/bin/env python3
"""it-scale-gate.v4 positive controls: two known-bad engine runs on the exact-input bundle of original o1 (arm 1 = world tracker ON, the gate arm), measured like the engine pairs.

  python3 -I v4_controls.py --pipe <e5_pipeline.py output dir> --seq <scene dir> --engine <sr_e5_replay> --out <NEW controls.json>

  focal-1.25    the bundle's engine-config.tsv gets an `intrinsics` override with fx = fy = 1.25 x 506.4103889465332 (principal point 240 / 320)
                (same construction as e5_kmutation.py; scale-class defect)
  drop-every:2  sr_e5_replay --mutation drop-every:2 (every second input frame is not processed; frame-drop class: shifts event frames AND the continuous metrics)
  drop-every:3  sr_e5_replay --mutation drop-every:3 (the same class, weaker)

Effect sizes of these (and of focal-1.10) against the tolerances of four existing sequences: dryrun/stage1c/controls-study/controls-study.json.
The gate's rule (e5_gate_rule_v4.py CONTROL_GROUPS) requires the scale class (focal-1.25) AND at least one frame-drop control to be DETECTED against the tolerance
derived in the same run, otherwise the run is INSENSITIVE.
Source files, the original bundle and the original runs are not modified (the focal control copies the bundle).  Development/gate tool.
"""
import argparse, json, pathlib, shutil, subprocess, sys
from concurrent.futures import ThreadPoolExecutor

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE / '../../harness/e5'))
import e5_extract as ex, e5_compare as cmp  # noqa: E402

FX_BASE = 506.4103889465332
FOCAL = 1.25
ap = argparse.ArgumentParser()
ap.add_argument('--pipe', required=True); ap.add_argument('--seq', required=True); ap.add_argument('--engine', required=True)
ap.add_argument('--out', required=True)
a = ap.parse_args()
pipe, seq, out = pathlib.Path(a.pipe).resolve(), pathlib.Path(a.seq).resolve(), pathlib.Path(a.out).resolve()
if out.exists():
    sys.exit(f'refusing existing {out}')
name = 'e5-' + seq.name
src = pipe / 'bundles' / name / 'ex-o1'
gt = json.load(open(seq / 'gt.json'))['frames']
orig = ex.original(str(pipe / 'runs' / name / 'o1'))


def run(label, bundle, extra):
    outd = pipe / 'eng' / f'o1-ctl-{label}-wt1'
    r = subprocess.run([a.engine, '--bundle', str(bundle), '--out', str(outd), '--targets', str(seq / 'target'), '--world-tracker', '1'] + extra,
                       capture_output=True, text=True)
    (pipe / 'logs').mkdir(exist_ok=True)
    (pipe / 'logs' / f'ctl-{label}.log').write_text(f'rc={r.returncode}\n{r.stdout}\n--- stderr\n{r.stderr}\n')
    if r.returncode:
        sys.exit(f'engine control {label} rc={r.returncode}: {r.stderr[-300:]}')
    s = ex.engine(str(outd))
    m = cmp.pair(orig, s, orig['cam_q'], s['cam_q'])
    m['events'] = [list(e) for e in s['events']]
    m['detected'] = sum(s['detected'])
    m['vs_gt_rho'] = cmp.vs_gt(s, s['cam_q'], gt)
    return m


focal = pipe / 'bundles' / name / 'ex-o1-ctl-focal-1.25'
shutil.copytree(src, focal)
subprocess.run(['chmod', '-R', 'u+w', str(focal)], check=True)   # a source bundle made read-only after a run must not block the copy
fx = FX_BASE * FOCAL
with open(focal / 'engine-config.tsv', 'a') as f:
    f.write('intrinsics\t%r %r 240.0 320.0\n' % (fx, fx))
jobs = [('focal-1.25', 'focal125', focal, []), ('drop-every:2', 'drop2', src, ['--mutation', 'drop-every:2']), ('drop-every:3', 'drop3', src, ['--mutation', 'drop-every:3'])]
with ThreadPoolExecutor(max_workers=3) as pool:
    ms = list(pool.map(lambda j: (j[0], run(j[1], j[2], j[3])), jobs))
res = {'schema': 'e5-controls-v4', 'against': 'o1', 'arm': 1, 'pairs': dict(ms)}
out.write_text(json.dumps(res, indent=1))
print(json.dumps({k: {'rho_p95': v['rho_rel']['p95'], 'dir_p95': v['dir_deg']['p95'], 'vs_gt_p95': v['vs_gt_rho']['p95']} for k, v in res['pairs'].items()}))

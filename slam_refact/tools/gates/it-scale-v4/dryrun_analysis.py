#!/usr/bin/env python3
"""it-scale-gate.v4 dry-run analysis (offline, no SDK, no engine, no marker): applies the v4 rule to EXISTING evidence.

  python3 -I dryrun_analysis.py --out <NEW dir>

(a) v3-on-v4: the v4 rule (N = 3 originals, 3 null pairs, no controls) applied to the 12 engine pairs of the v3 first run
    (build/slam-refact-pkgh-20261022/it-scale-v3-run1).  POST-HOC and KNOWN (the v3 results were seen): shows what the new rule would have said,
    it is not evidence for the v4 gate and nothing in the rule was changed after looking at it.
(b) mutation power: dev sequences 101 and 202 (seen; used for development only): the dev mutants judged with the tolerance of the dev
    sequence's own original-vs-original null (3 originals, 3 null pairs).  Mutants: orient-beta:3, orient-drift:1, drop-every:2/3/5, focal x1.10.
(c) new engine on the v3 bundles (stage 1b): the d7629f6-engine exact-input replays of the 12 v3 pairs made by the diagnosis worker
    (build/slam-refact-roifix-20261023/verify/it/new/seq-*/oK-wtA; IN-SAMPLE: the fixes were diagnosed on these sequences) judged with the v4 rule (N = 3, no controls).
All write JSON files under --out (a new directory) and print a summary.
"""
import argparse, json, os, pathlib, sys

HERE = pathlib.Path(__file__).resolve().parent
ROOT = HERE.parents[3]
sys.path.insert(0, str(HERE)); sys.path.insert(0, str(ROOT / 'slam_refact/tools/harness/e5'))
import e5_gate_rule_v4 as r4, e5_extract as ex, e5_compare as cmp  # noqa: E402

V3 = ROOT / 'build/slam-refact-pkgh-20261022/it-scale-v3-run1/gate'
DEV = ROOT / 'build/slam-refact-pkgh-20261022/dev'
ap = argparse.ArgumentParser(); ap.add_argument('--out', required=True); a = ap.parse_args()
out = pathlib.Path(a.out); out.mkdir(parents=True)


def rows_of(M, vis, n):
    tol = r4.tolerances(M, vis, n)
    rows = []
    for label in sorted(M['engine']):
        k = label.split('-')[0]
        r = r4.evaluate_pair(M['original'][k]['events'], M['engine'][label]['events'], M['engine'][label], vis, tol)
        rows.append({'label': label, 'pass': r['pass'], 'failed': sorted(c for c, ok in r['checks'].items() if not ok),
                     'ratio': {c: (None if v is None else round(v, 2)) for c, v in r['ratio_to_resolution'].items()}})
    return tol, rows


res_a = {}
for s in ('a', 'b'):
    M = json.load(open(V3 / f'pipe-seq-{s}/metrics.json')); vis = cmp.gt_vis(V3 / f'seq-{s}/gt.json')
    tol, rows = rows_of(M, vis, 3)
    res_a[s] = {'judged_anchors': tol['judged'], 'undefined_anchors': tol['undefined_anchors'],
                'null_anchor_diffs': tol['null']['anchor'], 'null_cont_p95': tol['null']['cont'], 'resolution': tol['R'], 'tolerance': tol['T'], 'rows': rows,
                'arm_pass': {arm: all(r['pass'] for r in rows if r['label'].endswith('wt' + arm)) for arm in '01'}}
(out / 'v3-on-v4.json').write_text(json.dumps(res_a, indent=1))

res_b = {}
for seed in ('202', '101'):
    pipe = DEV / f'pipe-{seed}'
    M = json.load(open(pipe / 'metrics.json')); vis = cmp.gt_vis(DEV / f'seq-{seed}/gt.json'); gtf = json.load(open(DEV / f'seq-{seed}/gt.json'))['frames']
    name = f'e5-seq-{seed}'
    base = {'k': M['k'], 'null': M['null'], 'original': M['original'], 'engine': {l: m for l, m in M['engine'].items() if '-m' not in l}}
    tol = r4.tolerances(base, vis, 3)
    mutants = {}
    for label, m in M['engine'].items():
        if '-m' in label:
            k, mi = label.split('-')[0], int(label.split('-')[1][1:])
            mutants[f'{k} {M["mutations"][mi]}'] = (k, m)
    for tag in ('dropmut2', 'dropmut5'):
        d = pipe / 'eng' / f'o1-{tag}-wt0'
        if d.exists():
            o = ex.original(str(pipe / 'runs' / name / 'o1')); e = ex.engine(str(d))
            m = cmp.pair(o, e, o['cam_q'], e['cam_q']); m['events'] = [list(x) for x in e['events']]; m['vs_gt_rho'] = cmp.vs_gt(e, e['cam_q'], gtf)
            mutants[f'o1 drop-every:{tag[-1]}'] = ('o1', m)
    kp = pipe / 'kmut-1.1.json'
    if kp.exists():
        for k, m in json.load(open(kp))['pairs'].items():
            mutants[f'{k} focal x1.10'] = (k, m)
    det = {}
    for lab, (k, m) in sorted(mutants.items()):
        r = r4.evaluate_pair(M['original'][k]['events'], m['events'], m, vis, tol)
        det[lab] = {'detected': not r['pass'], 'failed': sorted(c for c, ok in r['checks'].items() if not ok)}
    nat = [r4.evaluate_pair(M['original'][l.split('-')[0]]['events'], m['events'], m, vis, tol) for l, m in base['engine'].items()]
    res_b[seed] = {'resolution': tol['R'], 'tolerance': tol['T'], 'judged_anchors': tol['judged'], 'mutants': det,
                   'unmutated_engine_pairs_pass': [bool(r['pass']) for r in nat]}
(out / 'mutation-power.json').write_text(json.dumps(res_b, indent=1))

NEWENG = ROOT / 'build/slam-refact-roifix-20261023/verify/it/new'
res_c = {}
if NEWENG.exists():
    for sq in ('a', 'b'):
        M = json.load(open(V3 / f'pipe-seq-{sq}/metrics.json')); vis = cmp.gt_vis(V3 / f'seq-{sq}/gt.json'); gtf = json.load(open(V3 / f'seq-{sq}/gt.json'))['frames']
        name = f'e5-seq-{sq}'
        orig = {k: ex.original(str(V3 / f'pipe-seq-{sq}/runs/{name}/{k}')) for k in M['original']}
        eng = {}
        for label in M['engine']:
            k = label.split('-')[0]
            s_ = ex.engine(str(NEWENG / f'seq-{sq}' / label))
            m = cmp.pair(orig[k], s_, orig[k]['cam_q'], s_['cam_q']); m['events'] = [list(x) for x in s_['events']]; m['vs_gt_rho'] = cmp.vs_gt(s_, s_['cam_q'], gtf)
            eng[label] = m
        base = {'k': M['k'], 'null': M['null'], 'original': M['original'], 'engine': eng}
        tol, rows = rows_of(base, vis, 3)
        res_c[sq] = {'rows': rows, 'arm_pass': {arm: all(r['pass'] for r in rows if r['label'].endswith('wt' + arm)) for arm in '01'},
                     'state_disagree_frames_total': {l: m['state_disagree'] for l, m in eng.items()}, 'rho_p95': {l: m['rho_rel']['p95'] for l, m in eng.items()}}
    (out / 'new-engine-on-v3.json').write_text(json.dumps(res_c, indent=1))

for s, r in res_a.items():
    print(f'v3-on-v4 seq-{s}: arm_pass={r["arm_pass"]} judged={r["judged_anchors"]}')
    for row in r['rows']:
        print('   ', row['label'], 'PASS' if row['pass'] else 'FAIL ' + ','.join(row['failed']))
for s, r in res_b.items():
    print(f'dev {s}: unmutated engine pairs pass={r["unmutated_engine_pairs_pass"]}')
    for lab, d in r['mutants'].items():
        print('   ', lab, 'DETECTED' if d['detected'] else 'not detected', ','.join(d['failed']))
for sq, r in res_c.items():
    print(f'new engine on v3 bundles seq-{sq}: arm_pass={r["arm_pass"]}  state_disagree={r["state_disagree_frames_total"]}')
    for row in r['rows']:
        if not row['pass']:
            print('   ', row['label'], 'FAIL', ','.join(row['failed']))

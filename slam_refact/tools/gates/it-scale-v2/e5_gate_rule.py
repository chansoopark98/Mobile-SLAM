#!/usr/bin/env python3
"""it-scale-gate.v3 rule (directory it-scale-v2; v2 was withdrawn before any run, see the v3 gate file section 0): tolerances from dev null-runs, and the verdict of one registered sequence.

  e5_gate_rule.py derive --dev <metrics.json>... --out <tolerances.json>     (dev null pairs -> tolerances; run BEFORE registration)
  e5_gate_rule.py judge --metrics <metrics.json> --tol <tolerances.json> --gt <gt.json> --out <verdict.json>

The rule is the gate file's section 4/5; the file is pinned by sha256 in the manifest. Nothing here reads a registered result
before the tolerances are frozen: `derive` only takes dev metrics (dev seeds are listed in the manifest `held.dev_seeds`).
Tolerance = 3 x max(resolution), resolution = max(dev original-vs-original null-run value, representation floor)
(preregistration-protocol.md sections 2-3); the representation floors are 1 frame (event frames), 1e-3 relative (rho), 0.05 deg.
"""
import argparse, json, math, os, sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '../../harness/e5'))
import e5_compare as cmp  # noqa: E402

SETTLE = 8          # frames after a GT run starts that are excluded from the 'clear' set (detection / loss lag)
MIN_OUT, MIN_IN = 5, 15
FLOOR = {'event_frames': 1.0, 'rho_rel': 1e-3, 'dir_deg': 0.05, 'relrot_deg': 0.05, 'state_clear': 1.0}


def clear_frames(vis):
    runs = cmp.gt_runs(vis, MIN_OUT, MIN_IN)
    return {i for _, a, b in runs for i in range(a + SETTLE, b + 1)}, runs


def anchors(events, runs):
    """Anchored events of one run: F0 (first found), and for every GT 'out' run: L = first lost in [first-5, last], F = first found in [last-5, last+60]."""
    out = {}
    f = [e[1] for e in events if e[0] == 'found']
    out['F0'] = f[0] if f else None
    for n, (kind, a, b) in enumerate([r for r in runs if r[0] == 'out']):
        ls = [e[1] for e in events if e[0] == 'lost' and a - 5 <= e[1] <= b]
        fs = [e[1] for e in events if e[0] == 'found' and b - 5 <= e[1] <= b + 60]
        out[f'L{n}'] = ls[0] if ls else None
        out[f'F{n + 1}'] = fs[0] if fs else None
    return out


def anchor_diffs(ev_a, ev_b, runs):
    a, b = anchors(ev_a, runs), anchors(ev_b, runs)
    return {k: (None if a[k] is None or b.get(k) is None else abs(b[k] - a[k])) for k in a}, a, b


def pair_values(orig_events, other_events, m, vis):
    """The judged scalar values of one pair (other = engine or another original) against its original."""
    clear, runs = clear_frames(vis)
    dis = [i for i in m['state_disagree_frames'] if i in clear]
    d, _, _ = anchor_diffs(orig_events, other_events, runs)
    missing = [k for k, v in d.items() if v is None]
    return {'state_clear': len(dis), 'anchor_diffs': d, 'anchor_missing': missing,
            'event_frames': max([v for v in d.values() if v is not None], default=0),
            'rho_rel': m['rho_rel']['p95'], 'dir_deg': m['dir_deg']['p95'], 'relrot_deg': m['relrot_deg']['p95']}


def derive(dev_paths, out_path):
    res = {k: 0.0 for k in FLOOR}
    detail = []
    for p in dev_paths:
        M = json.load(open(p))
        vis = cmp.gt_vis(os.path.join(M['seq'], 'gt.json'))
        for n in M['null']:
            o_ev = M['original'][n['pair'][0]]['events']; l_ev = M['original'][n['pair'][1]]['events']
            v = pair_values(o_ev, l_ev, n, vis)
            if v['anchor_missing']:
                detail.append({'seq': M['seq'], 'pair': n['pair'], 'anchor_missing': v['anchor_missing']})
            for k in FLOOR:
                if v[k] is not None:
                    res[k] = max(res[k], float(v[k]))
            detail.append({'seq': M['seq'], 'pair': n['pair'], **{k: v[k] for k in FLOOR}})
    tol = {k: 3.0 * max(res[k], FLOOR[k]) for k in FLOOR}
    g = 0.0
    for p in dev_paths:
        for o in json.load(open(p))['original'].values():
            if o['vs_gt_rho']['p95'] is not None:
                g = max(g, o['vs_gt_rho']['p95'])
    tol['vs_gt_rho_p95'] = 3.0 * max(g, 1e-2)
    json.dump({'schema': 'e5-tolerances-v1', 'resolution_from_dev_null': res, 'tolerance': tol, 'dev_metrics': [os.path.basename(os.path.dirname(p)) for p in dev_paths],
               'floors': FLOOR, 'rule': 'tolerance = 3 x max(dev null-run value, floor); vs_gt = 3 x max(dev original p95 error vs GT, 1e-2)', 'detail': detail},
              open(out_path, 'x'), indent=1)
    return tol


def judge(metrics_path, tol_path, gt_path, out_path):
    M = json.load(open(metrics_path)); T = json.load(open(tol_path))['tolerance']
    vis = cmp.gt_vis(gt_path)
    items, verdict_all = [], True
    for label, m in sorted(M['engine'].items()):
        if '-m' in label:
            continue                      # mutation runs are dev-only
        k = label.split('-')[0]
        v = pair_values(M['original'][k]['events'], m['events'], m, vis)
        row = {'label': label, **v, 'vs_gt_rho_p95': m['vs_gt_rho']['p95']}
        # S1 (clear-frame state disagreement) is REPORT only in v3: no mutation shows it (dev drop-every:2 -> 25, :5 -> 10 <= bound 51)
        row['report_S1_state_clear'] = {'value': v['state_clear'], 'bound': T['state_clear'], 'within_bound': v['state_clear'] <= T['state_clear']}
        row['checks'] = {
            'S2_anchor_frames': (not v['anchor_missing']) and v['event_frames'] <= T['event_frames'],
            'S3_rho': v['rho_rel'] is not None and v['rho_rel'] <= T['rho_rel'],
            'S3_dir': v['dir_deg'] is not None and v['dir_deg'] <= T['dir_deg'],
            'S3_relrot': v['relrot_deg'] is not None and v['relrot_deg'] <= T['relrot_deg'],
            'S4_vs_gt': m['vs_gt_rho']['p95'] is not None and m['vs_gt_rho']['p95'] <= T['vs_gt_rho_p95'],
        }
        items.append(row)
    per_arm = {}
    arms_seen = sorted({r['label'].rsplit('-wt', 1)[1] for r in items})
    if len(items) != 6 or arms_seen != ['0', '1'] or any(sum(1 for r in items if r['label'].endswith('-wt' + a)) != 3 for a in arms_seen):
        raise SystemExit(f'judge: expected 6 engine pairs (3 per arm, arms 0 and 1), got {[r["label"] for r in items]}')
    for r in items:
        arm = r['label'].rsplit('-wt', 1)[1]
        per_arm[arm] = per_arm.get(arm, True) and all(r['checks'].values())
        verdict_all = verdict_all and all(r['checks'].values())
    json.dump({'schema': 'e5-verdict-v1', 'metrics': os.path.basename(metrics_path), 'tolerance': T, 'items': items, 'all_pass': verdict_all,
               'all_pass_by_arm': per_arm, 'S1_note': 'REPORT only (not judged)', 'arm_note': 'arm 0 = world tracker off (public path, shipped default), arm 1 = on (994/938 opt-in)'},
              open(out_path, 'x'), indent=1)
    return verdict_all


if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    sub = ap.add_subparsers(dest='cmd', required=True)
    d = sub.add_parser('derive'); d.add_argument('--dev', nargs='+', required=True); d.add_argument('--out', required=True)
    j = sub.add_parser('judge'); j.add_argument('--metrics', required=True); j.add_argument('--tol', required=True); j.add_argument('--gt', required=True); j.add_argument('--out', required=True)
    a = ap.parse_args()
    if a.cmd == 'derive':
        print(json.dumps(derive(a.dev, a.out)))
    else:
        print('ALL_PASS' if judge(a.metrics, a.tol, a.gt, a.out) else 'NOT_ALL_PASS')

#!/usr/bin/env python3
"""it-scale-gate.v4 rule: the tolerance is derived from the original-vs-original NULL measured inside the registered run itself.

  e5_gate_rule_v4.py judge --metrics <metrics.json> --gt <gt.json> --controls <controls.json> --out <verdict.json> [--n-orig N] [--report <report.json>]
  e5_gate_rule_v4.py final --verdicts <verdict.json>... --out <result.json>

judge = the verdict of ONE registered sequence (K = N_ORIG original SDK runs, the engine on every run's exact-input bundle at the gate arm,
three positive-control engine runs, an optional REPORT file). final = the verdict of the whole run.  Nothing here reads a tolerance from a
file: the constants below and the null of the same sequence are the whole rule (gate file it-scale-gate.v4.md section 5).

Rule (per sequence s):
  null pairs  P = all N(N-1)/2 original-vs-original pairs (same input sequence, independent SDK runs).
  per metric m (anchor frames F0/L0/F1/L1/F2, rho/dir/relrot p95):   R_m = max( max_{p in P} v_m(p), FLOOR_m )     (resolution, protocol 3)
  T_m = FACTOR x R_m                                                                                              (tolerance, FACTOR = 3)
  S4:  T_gt = FACTOR x max( max_o vs_gt_rho_p95(o), FLOOR_gt )
  engine pair (engine on the exact-input bundle of original o, arm a): every judged check <= T  -> pair PASS; an arm passes a sequence when
  all its N pairs pass.  ONLY THE GATE ARM (GATE_ARM = world tracker ON = the configuration the original always runs) enters the verdict; the
  other arm (world tracker OFF, shipped default) is judged with the same tolerances and reported (`report.arm0`), never counted.
  Positive controls on the bundle of o1 at GATE_ARM (CONTROL_GROUPS: scale = focal x1.25; frame-drop = drop-every:2 or drop-every:3): every group needs a
  member that is DETECTED (some check > T), else the sequence is INSENSITIVE (no PASS can be claimed); so is a sequence with no judged L anchor.
  Arm 0, a pre-fix engine and cross pairs are REPORT only (verdict.report) and can never make a sequence FAIL or INVALID.  Status of a sequence: FAIL > INSENSITIVE > PASS.  Whole run: FAIL > INVALID > INSENSITIVE > PASS.
Exit codes of the CLI: 0 PASS, 1 FAIL, 4 INVALID, 5 INSENSITIVE.  Development/gate tool (desktop synthetic input).
"""
import argparse, json, os, sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '../../harness/e5'))
import e5_compare as cmp  # noqa: E402

FACTOR = 3.0                      # protocol 3: tolerance >= 3 x resolution; resolution = in-run null (max over pairs) or the representation floor
N_ORIG = 5                        # original SDK runs per registered sequence -> N(N-1)/2 = 10 null pairs
N_SEQ = 3
MIN_JUDGED_ANCHORS = 3            # per sequence, F0 included, else the sequence is INVALID
FLOOR = {'event_frames': 1.0, 'rho_rel': 1e-3, 'dir_deg': 0.05, 'relrot_deg': 0.05, 'vs_gt_rho_p95': 1e-2}   # representation floors (same as v3)
# positive controls (dev-chosen strengths, gate md 5.3; effect sizes in dryrun/stage1c/controls-study.json): a sequence is sensitive when EVERY group has a detected member
CONTROL_GROUPS = {'scale': ('focal-1.25',), 'frame-drop': ('drop-every:2', 'drop-every:3')}
CONTROLS = tuple(c for g in CONTROL_GROUPS.values() for c in g)
GATE_ARM = '1'                    # world tracker ON: the original always runs the closed world-tracker branch (gate md 1, 5.2); arm '0' is REPORT only
CONT = ('rho_rel', 'dir_deg', 'relrot_deg')
# identical to it-scale-v2/e5_gate_rule.py (v3): clear-frame definition, anchor definition (tested equal in test_rule.py)
SETTLE = 8
MIN_OUT, MIN_IN = 5, 15


class Invalid(Exception):
    """A premise or tool-integrity failure: no GATE verdict for this sequence."""


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


def _diff(a, b):
    return None if a is None or b is None else abs(a - b)


def tolerances(M, vis, n_orig):
    """The in-run null and the tolerances of one sequence from metrics.json (raises Invalid when the null cannot be formed)."""
    keys = sorted(M['original'])
    if len(keys) != n_orig or M.get('k') != n_orig:
        raise Invalid(f'expected {n_orig} original runs, metrics has {len(keys)} (k={M.get("k")})')
    if len(M['null']) != n_orig * (n_orig - 1) // 2:
        raise Invalid(f'expected {n_orig * (n_orig - 1) // 2} null pairs, metrics has {len(M["null"])}')
    clear, runs = clear_frames(vis)
    A = {k: anchors(M['original'][k]['events'], runs) for k in keys}
    names = list(A[keys[0]].keys())
    judged = [a for a in names if all(A[k][a] is not None for k in keys)]
    undefined = {a: [k for k in keys if A[k][a] is None] for a in names if a not in judged}
    if 'F0' not in judged or len(judged) < MIN_JUDGED_ANCHORS:
        raise Invalid(f'premise P5: judged anchors {judged} (need F0 and >= {MIN_JUDGED_ANCHORS}); undefined in originals: {undefined}')
    null_anchor = {a: [] for a in judged}
    null_cont = {m: [] for m in CONT}
    null_state = []
    for p in M['null']:
        i, j = p['pair']
        for a in judged:
            null_anchor[a].append(_diff(A[i][a], A[j][a]))
        for m in CONT:
            v = p[m]['p95']
            if v is None:
                raise Invalid(f'premise P5: null pair {p["pair"]} has no co-detected frame ({m} p95 is null)')
            null_cont[m].append(v)
        null_state.append(len([f for f in p['state_disagree_frames'] if f in clear]))
    gt_p95 = []
    for k in keys:
        v = M['original'][k]['vs_gt_rho']['p95']
        if v is None:
            raise Invalid(f'premise P5: original {k} has no frame against the GT (vs_gt_rho p95 is null)')
        gt_p95.append(v)
    R = {'anchor': {a: max(max(null_anchor[a]), FLOOR['event_frames']) for a in judged},
         'cont': {m: max(max(null_cont[m]), FLOOR[m]) for m in CONT},
         'vs_gt_rho_p95': max(max(gt_p95), FLOOR['vs_gt_rho_p95'])}
    T = {'anchor': {a: FACTOR * v for a, v in R['anchor'].items()}, 'cont': {m: FACTOR * v for m, v in R['cont'].items()},
         'vs_gt_rho_p95': FACTOR * R['vs_gt_rho_p95']}
    return {'keys': keys, 'runs': runs, 'clear': clear, 'A': A, 'judged': judged, 'undefined_anchors': undefined,
            'null': {'anchor': null_anchor, 'cont': null_cont, 'state_clear': null_state, 'vs_gt_rho_p95': gt_p95}, 'R': R, 'T': T}


def evaluate_pair(orig_events, eng_events, m, vis, tol):
    """The checks of one (original, other run) pair against the tolerances of its sequence."""
    clear, runs = tol['clear'], tol['runs']
    a_o, a_e = anchors(orig_events, runs), anchors(eng_events, runs)
    diffs = {a: _diff(a_o[a], a_e.get(a)) for a in a_o}
    checks = {}
    for a in tol['judged']:
        checks[f'S2_{a}'] = diffs[a] is not None and diffs[a] <= tol['T']['anchor'][a]
    values = {'anchor_diffs': diffs, 'rho_rel': m['rho_rel']['p95'], 'dir_deg': m['dir_deg']['p95'], 'relrot_deg': m['relrot_deg']['p95'],
              'vs_gt_rho_p95': m['vs_gt_rho']['p95']}
    for key, name in (('rho_rel', 'S3_rho'), ('dir_deg', 'S3_dir'), ('relrot_deg', 'S3_relrot')):
        checks[name] = values[key] is not None and values[key] <= tol['T']['cont'][key]
    checks['S4_vs_gt'] = values['vs_gt_rho_p95'] is not None and values['vs_gt_rho_p95'] <= tol['T']['vs_gt_rho_p95']
    ratio = {f'S2_{a}': (None if diffs[a] is None else diffs[a] / tol['R']['anchor'][a]) for a in tol['judged']}
    for key, name in (('rho_rel', 'S3_rho'), ('dir_deg', 'S3_dir'), ('relrot_deg', 'S3_relrot')):
        ratio[name] = None if values[key] is None else values[key] / tol['R']['cont'][key]
    ratio['S4_vs_gt'] = None if values['vs_gt_rho_p95'] is None else values['vs_gt_rho_p95'] / tol['R']['vs_gt_rho_p95']
    state = len([f for f in m['state_disagree_frames'] if f in clear])
    return {'values': values, 'checks': checks, 'pass': all(checks.values()), 'ratio_to_resolution': ratio,
            'report_S1_state_clear': state, 'anchors_unjudged': {a: diffs[a] for a in diffs if a not in tol['judged']}}


def _failed(r):
    return sorted(c for c, ok in r['checks'].items() if not ok)


def _report(report_path, M, keys, vis, tol):
    """REPORT-only evidence (arm 0, pre-fix engine control, cross pairs). Never raises: anything unusable is reported as unavailable, never as INVALID."""
    out = {'arm0': {'available': False}, 'prefix_engine_control': {'available': False}, 'cross_pairs': {'available': False}}
    try:
        R = json.load(open(report_path)) if report_path else None
    except Exception as e:  # noqa: BLE001
        R = None
        out['error'] = f'report unreadable: {e}'
    if not R or 'error' in R:
        out.setdefault('error', (R or {}).get('error', 'no report file'))
        return out
    try:
        pairs, errs = [], []
        for k in keys:
            m = R['arm0'].get(f'{k}-wt0')
            if not m or 'error' in m:
                errs.append(f'{k}-wt0: {(m or {}).get("error", "missing")}')
                continue
            r = evaluate_pair(M['original'][k]['events'], m['events'], m, vis, tol)
            pairs.append((f'{k}-wt0', r))
        out['arm0'] = {'available': not errs, 'errors': errs, 'pass': bool(pairs) and not errs and all(r['pass'] for _, r in pairs),
                       'failed_pairs': sorted(f"{l}: {','.join(_failed(r))}" for l, r in pairs if not r['pass'])}
        m = R.get('prefix')
        if m and 'error' not in m:
            r = evaluate_pair(M['original'][keys[0]]['events'], m['events'], m, vis, tol)
            out['prefix_engine_control'] = {'available': True, 'detected': not r['pass'], 'failed_checks': _failed(r),
                                            'value_over_T': {c: (None if v is None else v / FACTOR) for c, v in r['ratio_to_resolution'].items()}}
        else:
            out['prefix_engine_control'] = {'available': False, 'error': (m or {}).get('error', 'missing')}
        cp, bad = {}, []
        for lab, m in R.get('cross_pairs', {}).items():
            if 'error' in m:
                bad.append(lab)
                continue
            j = lab.split('-vs-')[1]
            r = evaluate_pair(M['original'][j]['events'], m['events'], m, vis, tol)
            cp[lab] = r
        mx = {}
        for r in cp.values():
            for c, v in r['ratio_to_resolution'].items():
                if v is not None:
                    mx[c] = max(mx.get(c, 0.0), v)
        out['cross_pairs'] = {'available': bool(cp), 'n': len(cp), 'errors': bad, 'n_over_tolerance': sum(1 for r in cp.values() if not r['pass']),
                              'failed': sorted(f"{l}: {','.join(_failed(r))}" for l, r in cp.items() if not r['pass']), 'max_ratio_to_resolution': mx}
    except Exception as e:  # noqa: BLE001
        out['error'] = f'report evaluation failed: {e}'
    return out


def judge(metrics_path, gt_path, controls_path, out_path, n_orig=N_ORIG, report_path=None):
    M = json.load(open(metrics_path))
    vis = cmp.gt_vis(gt_path)
    tol = tolerances(M, vis, n_orig)
    keys = tol['keys']
    want = sorted(f'{k}-wt{GATE_ARM}' for k in keys)
    labels = sorted(M['engine'])
    if labels != want:
        raise Invalid(f'expected gate-arm engine pairs {want}, metrics has {labels}')
    rows = []
    for label in labels:
        k, arm = label.split('-wt')
        r = evaluate_pair(M['original'][k]['events'], M['engine'][label]['events'], M['engine'][label], vis, tol)
        rows.append({'label': label, 'arm': arm, **r})
    gate_pass = all(r['pass'] for r in rows)
    C = json.load(open(controls_path))
    if C.get('against') != keys[0] or sorted(C.get('pairs', {})) != sorted(CONTROLS) or str(C.get('arm')) != GATE_ARM:
        raise Invalid(f'controls must be exactly {list(CONTROLS)} against {keys[0]} on arm {GATE_ARM}; got {C.get("against")} {sorted(C.get("pairs", {}))} arm {C.get("arm")}')
    controls = {}
    for name in CONTROLS:
        r = evaluate_pair(M['original'][keys[0]]['events'], C['pairs'][name]['events'], C['pairs'][name], vis, tol)
        controls[name] = {'detected': not r['pass'], 'failed_checks': _failed(r), 'row': r,
                          'value_over_T': {c: (None if v is None else v / FACTOR) for c, v in r['ratio_to_resolution'].items()}}
    insensitive = sorted(g for g, members in CONTROL_GROUPS.items() if not any(controls[m]['detected'] for m in members))
    if not any(a.startswith('L') for a in tol['judged']):
        insensitive.append('coverage:no-L-anchor-judged')        # the sequence cannot support a PASS about lost events (explicit, not INVALID)
    status = 'FAIL' if not gate_pass else ('INSENSITIVE' if insensitive else 'PASS')
    verdict = {'schema': 'e5-verdict-v4', 'metrics': os.path.basename(metrics_path), 'n_orig': n_orig, 'factor': FACTOR, 'floors': FLOOR,
               'status': status, 'gate_arm': GATE_ARM, 'gate_arm_pass': gate_pass, 'insensitive_controls': insensitive,
               'report': _report(report_path, M, keys, vis, tol),
               'judged_anchors': tol['judged'], 'undefined_anchors': tol['undefined_anchors'], 'anchors_of_originals': tol['A'],
               'null': {'anchor_diffs': tol['null']['anchor'], 'cont_p95': tol['null']['cont'], 'S1_state_clear': tol['null']['state_clear'],
                        'vs_gt_rho_p95_of_originals': tol['null']['vs_gt_rho_p95']},
               'resolution': tol['R'], 'tolerance': tol['T'], 'rows': rows, 'controls': controls,
               'S1_note': 'REPORT only (not judged)',
               'arm_note': 'GATE arm 1 = world tracker ON (the configuration the original always runs; counted). Arm 0 (world tracker OFF, shipped default), the pre-fix engine control and the cross pairs are REPORT only (verdict.report), never counted, and their failure never makes a sequence INVALID',
               'lenient_bias_note': 'the null contains input-realisation variation (frame sets, queue timing) that an engine run on the exact-input bundle of its own original does not have; report.cross_pairs measures the same engine runs against the other originals'}
    json.dump(verdict, open(out_path, 'x'), indent=1)
    return verdict


def final(verdict_paths, out_path):
    """Whole-run status from the per-sequence verdicts (a path whose content is {"invalid": why} stands for an INVALID sequence)."""
    seqs, gate_pass, statuses = [], True, []
    for p in verdict_paths:
        v = json.load(open(p))
        if 'invalid' in v:
            seqs.append({'name': os.path.basename(p), 'status': 'INVALID', 'why': v['invalid']})
            statuses.append('INVALID')
            continue
        seqs.append({'name': os.path.basename(p), 'status': v['status'], 'gate_arm_pass': v['gate_arm_pass'], 'insensitive_controls': v['insensitive_controls'],
                     'report_arm0': v.get('report', {}).get('arm0'), 'report_prefix_engine_control': v.get('report', {}).get('prefix_engine_control')})
        statuses.append(v['status'])
        gate_pass = gate_pass and v['gate_arm_pass']
    status = ('FAIL' if 'FAIL' in statuses else 'INVALID' if 'INVALID' in statuses else 'INSENSITIVE' if 'INSENSITIVE' in statuses else 'PASS')
    if len(statuses) != N_SEQ:
        status = 'FAIL' if 'FAIL' in statuses else 'INVALID'      # a missing sequence is never a PASS
    res = {'schema': 'e5-gate-result-v4', 'status': status, 'gate_arm': GATE_ARM, 'gate_arm_pass': gate_pass, 'sequences': seqs,
           'scope': 'desktop headless SwiftShader, synthetic E5 room, original reference runtime SDK vs engine (exact-input bundles), tolerance = 3 x in-run original-vs-original null (null-run resolution of this run, protocol rules 3-4; not a distribution claim); not device performance'}
    json.dump(res, open(out_path, 'x'), indent=1)
    return status


CODE = {'PASS': 0, 'FAIL': 1, 'INVALID': 4, 'INSENSITIVE': 5}

if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    sub = ap.add_subparsers(dest='cmd', required=True)
    j = sub.add_parser('judge')
    j.add_argument('--metrics', required=True); j.add_argument('--gt', required=True); j.add_argument('--controls', required=True)
    j.add_argument('--out', required=True); j.add_argument('--n-orig', type=int, default=N_ORIG); j.add_argument('--report')
    f = sub.add_parser('final')
    f.add_argument('--verdicts', nargs='+', required=True); f.add_argument('--out', required=True)
    a = ap.parse_args()
    try:
        s = judge(a.metrics, a.gt, a.controls, a.out, a.n_orig, a.report)['status'] if a.cmd == 'judge' else final(a.verdicts, a.out)
    except Invalid as e:
        print(f'INVALID: {e}')
        sys.exit(CODE['INVALID'])
    print(s)
    sys.exit(CODE[s])

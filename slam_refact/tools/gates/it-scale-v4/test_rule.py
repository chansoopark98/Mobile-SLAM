#!/usr/bin/env python3
"""Unit tests of e5_gate_rule_v4.py on synthetic series (no SDK, no engine).  python3 -I -m unittest test_rule   (from this directory)."""
import json, os, sys, tempfile, unittest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, '../../harness/e5'))
sys.path.insert(0, os.path.join(HERE, '../it-scale-v2'))
import e5_compare as cmp  # noqa: E402
import e5_gate_rule_v4 as r4  # noqa: E402
import e5_gate_rule as r3  # noqa: E402   (v3 rule: only to show that the shared definitions are identical)

N = 100
VIS = [1.0] * 30 + [0.0] * 10 + [1.0] * 30 + [0.0] * 20 + [1.0] * 10      # in 0-29, out 30-39, in 40-69, out 70-89, last 'in' too short for a run


def ser(l0=30, f1=40, l1=70, f2=90, scale=2.0, f0=2, rho_k=1.0):
    """A run that is detected in [f0,l0), [f1,l1), [f2,N); events consistent with it; rho_k scales the published scale (rho = d/scale/rho_k)."""
    s = {'n': N, 'processed': [True] * N, 'detected': [False] * N, 'pos': [None] * N, 'quat': [None] * N, 'scale': [None] * N,
         'pose': [[0.0, 0.0, 0.0]] * N, 'cam_q': [[1.0, 0.0, 0.0, 0.0]] * N, 'events': []}
    ev = []
    for a, b in ((f0, l0), (f1, l1), (f2, N)):
        if a is None:
            continue
        ev.append(('found', a))
        if b < N:
            ev.append(('lost', b))
        for i in range(a, b):
            s['detected'][i] = True; s['pos'][i] = [0.0, 0.0, 3.0 * scale]; s['quat'][i] = [1.0, 0.0, 0.0, 0.0]; s['scale'][i] = scale * rho_k
    s['events'] = ev
    return s


def stats0():
    return {'n': 100, 'p50': 0.0, 'p95': 0.0, 'max': 0.0}


def build(origs, engines, gt_p95=0.015):
    """metrics.json as e5_pipeline.py writes it: origs = {'o1': series}, engines = {'o1-wt0': series}."""
    keys = sorted(origs)
    M = {'schema': 'e5-metrics-v1', 'k': len(keys), 'gt_runs': cmp.gt_runs(VIS, 5, 15), 'original': {}, 'null': [], 'engine': {}}
    for k in keys:
        M['original'][k] = {'events': [list(e) for e in origs[k]['events']], 'detected': sum(origs[k]['detected']), 'processed': N, 'vs_gt_rho': {'p95': gt_p95}}
    for i, a in enumerate(keys):
        for b in keys[i + 1:]:
            m = cmp.pair(origs[a], origs[b], origs[a]['cam_q'], origs[b]['cam_q']); m['pair'] = [a, b]
            M['null'].append(m)
    for label, s in engines.items():
        k = label.split('-')[0]
        m = cmp.pair(origs[k], s, origs[k]['cam_q'], s['cam_q']); m['events'] = [list(e) for e in s['events']]; m['vs_gt_rho'] = {'p95': gt_p95}
        M['engine'][label] = m
    return M


def controls(origs, bad_f=True, bad_d=True, bad_d2=None):
    """Control pairs against o1 on the gate arm; 'detected' controls get a 10 % scale error / event shifts. bad_d2 (default = bad_d) is drop-every:2, bad_d drop-every:3."""
    o1 = origs['o1']
    bad_d2 = bad_d if bad_d2 is None else bad_d2
    out = {'schema': 'e5-controls-v4', 'against': 'o1', 'arm': 1, 'pairs': {}}
    for name, bad in (('focal-1.25', bad_f), ('drop-every:2', bad_d2), ('drop-every:3', bad_d)):
        s = ser(rho_k=1.1, l0=36, f1=47) if bad else ser()
        m = cmp.pair(o1, s, o1['cam_q'], s['cam_q']); m['events'] = [list(e) for e in s['events']]; m['vs_gt_rho'] = {'p95': 0.015}
        out['pairs'][name] = m
    return out


def report(origs, arm0_series, prefix=None, cross_bad=None):
    """report.json as v4_report_runs.py writes it: arm 0 = own original unless replaced, prefix control against o1, cross pairs e_i vs o_j."""
    def st(o, s):
        m = cmp.pair(o, s, o['cam_q'], s['cam_q']); m['events'] = [list(e) for e in s['events']]; m['vs_gt_rho'] = {'p95': 0.015}
        return m
    R = {'schema': 'e5-report-v4', 'arm0': {}, 'prefix': st(origs['o1'], prefix) if prefix is not None else None, 'cross_pairs': {}}
    for k, o in origs.items():
        R['arm0'][f'{k}-wt0'] = st(o, arm0_series.get(k, o))
    for i in origs:
        for j in origs:
            if i != j:
                R['cross_pairs'][f'e{i}-vs-{j}'] = st(origs[j], ser(f1=45) if cross_bad == i else origs[i])
    return R


def run_judge(M, C, n=5, R=None):
    with tempfile.TemporaryDirectory() as t:
        mp, gp, cp, op = (os.path.join(t, x) for x in ('m.json', 'g.json', 'c.json', 'o.json'))
        json.dump(M, open(mp, 'w')); json.dump(C, open(cp, 'w'))
        json.dump({'frames': [{'target_visible_fraction': v} for v in VIS]}, open(gp, 'w'))
        rp = None
        if R is not None:
            rp = os.path.join(t, 'r.json'); json.dump(R, open(rp, 'w'))
        return r4.judge(mp, gp, cp, op, n, rp)


def five(jitter=None):
    """5 originals; jitter = {'o3': {'f1': 45}} etc. shifts that run's anchor."""
    jitter = jitter or {}
    return {f'o{i}': ser(**jitter.get(f'o{i}', {})) for i in range(1, 6)}


def eng_same(origs, arms=('1',)):
    return {f'{k}-wt{a}': origs[k] for k in origs for a in arms}


class Rule(unittest.TestCase):
    def test_shared_definitions_equal_v3(self):
        for ev in ([('found', 2), ('lost', 30), ('found', 41), ('lost', 70), ('found', 90)], [('found', 5)], []):
            self.assertEqual(r4.anchors(ev, r4.clear_frames(VIS)[1]), r3.anchors(ev, r3.clear_frames(VIS)[1]))
        self.assertEqual(r4.clear_frames(VIS), r3.clear_frames(VIS))

    def test_identical_engine_passes_and_controls_detected(self):
        o = five(); v = run_judge(build(o, eng_same(o)), controls(o))
        self.assertEqual(v['status'], 'PASS'); self.assertTrue(v['gate_arm_pass'])
        self.assertEqual(sorted(v['judged_anchors']), ['F0', 'F1', 'F2', 'L0', 'L1'])
        # null is 0 everywhere: resolution = floors, tolerance = 3 x floors
        self.assertEqual(v['tolerance']['anchor']['L0'], 3.0); self.assertAlmostEqual(v['tolerance']['cont']['rho_rel'], 3e-3)

    def test_anchor_shift_beyond_floor_tolerance_fails_when_null_is_zero(self):
        o = five(); e = eng_same(o); e['o2-wt1'] = ser(l0=35)           # engine (gate arm) loses 5 frames later while the originals agree exactly
        v = run_judge(build(o, e), controls(o))
        self.assertEqual(v['status'], 'FAIL'); self.assertFalse(v['gate_arm_pass'])
        row = [r for r in v['rows'] if r['label'] == 'o2-wt1'][0]
        self.assertFalse(row['checks']['S2_L0']); self.assertTrue(row['checks']['S2_F0'])

    def test_arm0_is_reported_but_never_counted(self):
        o = five(); e = eng_same(o)
        R = report(o, {'o2': ser(l0=35), 'o4': ser(rho_k=1.2)})
        v = run_judge(build(o, e), controls(o), R=R)
        self.assertEqual(v['status'], 'PASS'); self.assertTrue(v['gate_arm_pass'])
        a0 = v['report']['arm0']
        self.assertTrue(a0['available']); self.assertFalse(a0['pass']); self.assertEqual(len(a0['failed_pairs']), 2)
        self.assertIn('o2-wt0: S2_L0', a0['failed_pairs'])

    def test_arm0_failure_or_missing_report_never_invalidates(self):
        o = five(); e = eng_same(o)
        R = report(o, {})
        R['arm0']['o3-wt0'] = {'error': 'engine rc=5'}
        v = run_judge(build(o, e), controls(o), R=R)
        self.assertEqual(v['status'], 'PASS'); self.assertFalse(v['report']['arm0']['available']); self.assertIn('o3-wt0', v['report']['arm0']['errors'][0])
        v = run_judge(build(o, e), controls(o), R=None)                    # no report file at all
        self.assertEqual(v['status'], 'PASS'); self.assertFalse(v['report']['arm0']['available'])
        v = run_judge(build(o, e), controls(o), R={'schema': 'e5-report-v4', 'error': 'originals unreadable'})
        self.assertEqual(v['status'], 'PASS'); self.assertFalse(v['report']['prefix_engine_control']['available'])

    def test_prefix_engine_control_and_cross_pairs_are_report_only(self):
        o = five(); e = eng_same(o)
        R = report(o, {}, prefix=ser(l0=36, f1=47), cross_bad='o2')
        v = run_judge(build(o, e), controls(o), R=R)
        self.assertEqual(v['status'], 'PASS')
        self.assertTrue(v['report']['prefix_engine_control']['detected'])
        self.assertEqual(v['report']['cross_pairs']['n'], 20); self.assertGreater(v['report']['cross_pairs']['n_over_tolerance'], 0)

    def test_sequence_without_a_judged_L_anchor_is_insensitive_not_invalid(self):
        o = five()
        for k in o:                                                       # no original ever loses the target in the out runs: L0, L1 undefined everywhere
            s = ser(); s['events'] = [x for x in s['events'] if x[0] != 'lost']; o[k] = s
        v = run_judge(build(o, eng_same(o)), controls(o))
        self.assertEqual(v['status'], 'INSENSITIVE'); self.assertIn('coverage:no-L-anchor-judged', v['insensitive_controls'])

    def test_scale_error_fails_s3_rho_and_s4_is_independent(self):
        o = five(); e = eng_same(o); e['o4-wt1'] = ser(rho_k=1.05)
        v = run_judge(build(o, e), controls(o))
        row = [r for r in v['rows'] if r['label'] == 'o4-wt1'][0]
        self.assertFalse(row['checks']['S3_rho']); self.assertTrue(row['checks']['S4_vs_gt'])
        self.assertEqual(v['status'], 'FAIL')

    def test_gt_error_of_the_engine_is_bounded_by_3x_the_worst_original(self):
        o = five(); M = build(o, eng_same(o), gt_p95=0.015)
        M['engine']['o1-wt1']['vs_gt_rho'] = {'p95': 0.044}               # < 3 x 0.015
        self.assertEqual(run_judge(M, controls(o))['status'], 'PASS')
        M['engine']['o1-wt1']['vs_gt_rho'] = {'p95': 0.046}               # > 3 x 0.015
        self.assertEqual(run_judge(M, controls(o))['status'], 'FAIL')

    def test_undetected_control_makes_the_run_insensitive_not_pass(self):
        o = five(); v = run_judge(build(o, eng_same(o)), controls(o, bad_f=False))
        self.assertEqual(v['status'], 'INSENSITIVE'); self.assertEqual(v['insensitive_controls'], ['scale'])

    def test_frame_drop_group_needs_only_one_member(self):
        o = five()
        v = run_judge(build(o, eng_same(o)), controls(o, bad_d=False, bad_d2=True)); self.assertEqual(v['status'], 'PASS')
        v = run_judge(build(o, eng_same(o)), controls(o, bad_d=True, bad_d2=False)); self.assertEqual(v['status'], 'PASS')
        v = run_judge(build(o, eng_same(o)), controls(o, bad_d=False, bad_d2=False))
        self.assertEqual(v['status'], 'INSENSITIVE'); self.assertEqual(v['insensitive_controls'], ['frame-drop'])

    def test_fail_takes_precedence_over_insensitive(self):
        o = five(); e = eng_same(o); e['o2-wt1'] = ser(l0=35)
        self.assertEqual(run_judge(build(o, e), controls(o, bad_f=False, bad_d=False))['status'], 'FAIL')

    def test_anchor_undefined_in_one_original_is_not_judged(self):
        o = five()                                                        # o5 never loses the target in the second out run -> no L1
        s = ser(); s['events'] = [e for e in s['events'] if e != ('lost', 70)]
        o['o5'] = s
        v = run_judge(build(o, eng_same(o)), controls(o))
        self.assertIn('L1', v['undefined_anchors']); self.assertNotIn('L1', v['judged_anchors'])

    def test_too_few_judged_anchors_is_invalid(self):
        o = five()
        for k in ('o1', 'o2'):
            s = ser(); s['events'] = [('found', 2)]; o[k] = s                # only F0 is defined
        with self.assertRaises(r4.Invalid):
            run_judge(build(o, eng_same(o)), controls(o))

    def test_missing_or_extra_engine_pair_is_invalid(self):
        o = five(); e = eng_same(o); del e['o5-wt1']
        with self.assertRaises(r4.Invalid):
            run_judge(build(o, e), controls(o))
        e = eng_same(o); e['o1-m0-wt1'] = ser()
        with self.assertRaises(r4.Invalid):
            run_judge(build(o, e), controls(o))

    def test_wrong_number_of_originals_is_invalid(self):
        o = {f'o{i}': ser() for i in range(1, 4)}
        with self.assertRaises(r4.Invalid):
            run_judge(build(o, eng_same(o)), controls(o))

    def test_controls_must_be_the_named_ones(self):
        o = five(); C = controls(o); del C['pairs']['drop-every:2']
        with self.assertRaises(r4.Invalid):
            run_judge(build(o, eng_same(o)), C)
        C = controls(o); C['against'] = 'o2'
        with self.assertRaises(r4.Invalid):
            run_judge(build(o, eng_same(o)), C)
        C = controls(o); C['arm'] = 0                                      # controls on the REPORT arm are not accepted
        with self.assertRaises(r4.Invalid):
            run_judge(build(o, eng_same(o)), C)

    def test_engine_without_a_judged_anchor_fails(self):
        o = five(); e = eng_same(o); s = ser(); s['events'] = [x for x in s['events'] if x != ('found', 90)]; e['o1-wt1'] = s
        v = run_judge(build(o, e), controls(o))
        row = [r for r in v['rows'] if r['label'] == 'o1-wt1'][0]
        self.assertFalse(row['checks']['S2_F2']); self.assertEqual(v['status'], 'FAIL')

    def test_final_precedence_and_missing_sequence(self):
        def v(t, status, arms=(True, True), ins=()):
            p = os.path.join(t, f'v{len(os.listdir(t))}.json')
            json.dump({'invalid': 'x'} if status == 'INVALID' else {'status': status, 'gate_arm_pass': arms[1], 'report': {}, 'insensitive_controls': list(ins)}, open(p, 'w'))
            return p
        with tempfile.TemporaryDirectory() as t:
            def fin(*ps):
                return r4.final(list(ps), os.path.join(t, f'r{len(os.listdir(t))}.json'))
            self.assertEqual(fin(v(t, 'PASS'), v(t, 'PASS'), v(t, 'PASS')), 'PASS')
            self.assertEqual(fin(v(t, 'PASS'), v(t, 'INSENSITIVE'), v(t, 'PASS')), 'INSENSITIVE')
            self.assertEqual(fin(v(t, 'PASS'), v(t, 'INVALID'), v(t, 'INSENSITIVE')), 'INVALID')
            self.assertEqual(fin(v(t, 'FAIL', (False, True)), v(t, 'INVALID'), v(t, 'PASS')), 'FAIL')     # FAIL-first
            self.assertEqual(fin(v(t, 'PASS'), v(t, 'PASS')), 'INVALID')                                   # a missing sequence is never a PASS
            self.assertEqual(fin(v(t, 'FAIL', (True, False)), v(t, 'PASS')), 'FAIL')


if __name__ == '__main__':
    unittest.main()

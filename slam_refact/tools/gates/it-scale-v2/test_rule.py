#!/usr/bin/env python3
"""Unit tests of e5_gate_rule.py / e5_compare.py on synthetic series (no SDK, no engine). python3 -I -m unittest test_rule (from this directory)."""
import json, math, os, sys, tempfile, unittest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, '../../harness/e5'))
import e5_compare as cmp  # noqa: E402
import e5_gate_rule as rule  # noqa: E402


def series(n, found_from, lost_from, scale=2.0):
    s = {'n': n, 'processed': [True] * n, 'detected': [False] * n, 'pos': [None] * n, 'quat': [None] * n, 'scale': [None] * n,
         'pose': [None] * n, 'cam_q': [None] * n, 'events': [('found', found_from), ('lost', lost_from)]}
    for i in range(n):
        s['pose'][i] = [0.0, 0.0, 0.0]; s['cam_q'][i] = [1.0, 0.0, 0.0, 0.0]
        if found_from <= i < lost_from:
            s['detected'][i] = True; s['pos'][i] = [0.0, 0.0, 3.0 * scale]; s['quat'][i] = [1.0, 0.0, 0.0, 0.0]; s['scale'][i] = scale
    return s


class Rule(unittest.TestCase):
    def test_gauge_free_ignores_world_scale(self):
        a, b = series(40, 5, 35, 2.0), series(40, 5, 35, 7.0)     # same scene, different world gauge (scale 2 vs 7, positions scale with it)
        m = cmp.pair(a, b, a['cam_q'], b['cam_q'])
        self.assertEqual(m['state_disagree'], 0)
        self.assertLess(m['rho_rel']['max'], 1e-12)

    def test_scale_error_is_seen(self):
        a, b = series(40, 5, 35, 2.0), series(40, 5, 35, 2.0)
        for i in range(40):
            if b['scale'][i]:
                b['scale'][i] *= 1.1
        m = cmp.pair(a, b, a['cam_q'], b['cam_q'])
        self.assertAlmostEqual(m['rho_rel']['p95'], 1 / 1.1 - 1 if False else abs(1 / 1.1 - 1), places=9)

    def test_anchor_missing_is_a_failure(self):
        vis = [1.0] * 10 + [0.0] * 10 + [1.0] * 20
        runs = rule.cmp.gt_runs(vis, 5, 15)
        d, _, _ = rule.anchor_diffs([('found', 1), ('lost', 10), ('found', 22)], [('found', 1)], runs)
        self.assertIsNone(d['L0'])

    def test_judge_requires_six_pairs(self):
        M = {'seq': '.', 'original': {}, 'engine': {}}
        with tempfile.TemporaryDirectory() as t:
            mp, tp, gp, op = [os.path.join(t, n) for n in ('m.json', 't.json', 'g.json', 'o.json')]
            json.dump(M, open(mp, 'w'))
            json.dump({'tolerance': {k: 1.0 for k in ('event_frames', 'rho_rel', 'dir_deg', 'relrot_deg', 'state_clear', 'vs_gt_rho_p95')}}, open(tp, 'w'))
            json.dump({'frames': [{'target_visible_fraction': 1.0}] * 20}, open(gp, 'w'))
            with self.assertRaises(SystemExit):
                rule.judge(mp, tp, gp, op)


if __name__ == '__main__':
    unittest.main()

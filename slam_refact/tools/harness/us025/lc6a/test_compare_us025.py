#!/usr/bin/env python3
"""Unit tests of compare_us025.py on synthetic attempt data (no replay, no Chromium): a consistent original/engine pair passes, and every compared quantity,
mutated alone, makes the verdict FAIL. Also the aggregate rules (INVALID never dropped, k < K_MIN is INSUFFICIENT) and the exit-code contract.
  python3 -B -I slam_refact/tools/harness/us025/lc6a/test_compare_us025.py"""
import copy
import importlib.util
import json
import pathlib
import struct
import sys
import tempfile
import unittest

HERE = pathlib.Path(__file__).resolve().parent
spec = importlib.util.spec_from_file_location('compare_us025', HERE.parent / 'compare_us025.py')
C = importlib.util.module_from_spec(spec)
spec.loader.exec_module(C)


def mat(seed):
    return struct.pack('<32f', *[seed + 0.001 * k for k in range(32)])


def make_pair(hooked=True):
    """Three attempts (frames 10, 20, 30); the third is the success F = 30 with a self candidate."""
    live = [11, 22, 33, 44, 55, 66]
    desc = [bytes([k] * 32) for k in range(6)]
    orig_att, eng_att = [], {}
    for n, (frame, slots, newest, pick, ok) in enumerate([(10, 6, [5, 66], 0, 0), (20, 6, [5, 66], 33, 0), (30, 6, [5, 66], 66, 1)]):
        ring = [newest[0], newest[1], pick, 0, 0, 1 if pick else 0]
        hooks = {}
        if hooked and pick:
            hooks = {'matches': 83 if ok else 10}
            if ok:
                hooks.update({'corr': 42, 'pnpRet': 1, 'poseOld': mat(1.0)})
        a = {'seq': n + 1, 'frame': frame, 'nSlots': slots, 'nLive': slots, 'free': 0, 'newest': newest, 'ringPre': [0] * 6, 'ringPost': ring, 'pick': pick,
             'liveIds': live, 'outLcOk': ok, 'desc': b''.join(desc), 'outLcMat': mat(2.0) if ok else b'', 'newestExtrinsic': mat(3.0) if ok else b'', 'hooks': hooks}
        orig_att.append(a)
        outcome = 1 if not pick else (3 if not ok else 6)
        eng_att[frame] = {'frame': frame, 'nSlots': slots, 'nLive': slots, 'newest': newest, 'slots': live, 'desc': [d.hex() for d in desc], 'ringPre': [0] * 6,
                          'ringPost': list(ring), 'outcome': outcome, 'applied': bool(ok), 'outLcOk': ok, 'cand': [0, pick], 'candDist': 0.0,
                          'nMatches': (83 if ok else 10) if pick else 0, 'nCorr': 42 if ok else 0, 'nUnlinked': 0,
                          'poseBefore': mat(1.0).hex(), 'poseAfter': mat(3.0).hex() if ok else '', 'C': mat(2.0).hex() if ok else ''}
    orig = {'attempts': orig_att, 'hooked': hooked, 'firstSuccess': orig_att[2]}
    return orig, {'attempts': eng_att}


def verdict(orig, eng):
    checks, _ = C.judge_attempts(orig, eng)
    return C.verdict_of(checks), {c['name']: c for c in checks}


class Judge(unittest.TestCase):
    def test_consistent_pair_passes(self):
        for hooked in (True, False):
            v, ch = verdict(*make_pair(hooked))
            self.assertEqual(v, 'PASS', ch)
            self.assertEqual(ch['successCall']['compared'], 1)
            self.assertEqual(ch['attemptFrameSet']['compared'], 3)

    def mutated(self, fn, hooked=True):
        orig, eng = make_pair(hooked)
        fn(orig, eng)
        return verdict(orig, eng)

    def test_each_quantity_detected(self):
        cases = {
            'candidate id (live id)': (lambda o, e: e['attempts'][20].update(ringPost=[5, 66, 44, 0, 0, 1]), 'candidateId'),
            'ring cursor': (lambda o, e: e['attempts'][20]['ringPost'].__setitem__(5, 2), 'ringState'),
            'slot count': (lambda o, e: e['attempts'][10].update(nSlots=7), 'slotLiveFree'),
            'live count': (lambda o, e: e['attempts'][10].update(nLive=5), 'slotLiveFree'),
            'newest handle': (lambda o, e: e['attempts'][10].update(newest=[4, 66]), 'newestHandle'),
            'dropped attempt': (lambda o, e: e['attempts'].pop(10), 'attemptFrameSet'),
            'extra attempt': (lambda o, e: e['attempts'].__setitem__(15, copy.deepcopy(e['attempts'][10]) | {'frame': 15}), 'attemptFrameSet'),
            'descriptor byte': (lambda o, e: e['attempts'][20]['desc'].__setitem__(2, 'ff' * 32), 'globalDescriptorLiveSlots'),
            'stage class': (lambda o, e: e['attempts'][20].update(outcome=2), 'thresholdStage'),
            'engine success earlier': (lambda o, e: e['attempts'][20].update(applied=True, outcome=6), 'triggerFrameSet'),
            'engine success missing': (lambda o, e: e['attempts'][30].update(applied=False, outcome=3), 'successCall'),
            'out+0x1c': (lambda o, e: e['attempts'][30].update(outLcOk=0), 'successCall'),
            'C one bit': (lambda o, e: e['attempts'][30].update(C=(bytes([mat(2.0)[0] ^ 1]) + mat(2.0)[1:]).hex()), 'successCall'),
            'match count': (lambda o, e: e['attempts'][20].update(nMatches=11), 'matchCount'),
            'correspondence count': (lambda o, e: e['attempts'][30].update(nCorr=41), 'correspondenceCount'),
            'P_old bit': (lambda o, e: e['attempts'][30].update(poseBefore=(mat(1.0)[:4] + bytes([mat(1.0)[4] ^ 1]) + mat(1.0)[5:]).hex()), 'successCall'),
        }
        for name, (fn, expect) in cases.items():
            v, ch = self.mutated(fn)
            self.assertEqual(v, 'FAIL', name)
            self.assertGreater(ch[expect]['mismatch'], 0, '%s not caught by %s' % (name, expect))

    def test_pnp_pose_tolerance_is_three_ulp(self):
        def bump(ulps_):
            def fn(o, e):
                b = bytearray(mat(3.0))
                w = struct.unpack('<I', b[0:4])[0]
                b[0:4] = struct.pack('<I', w + ulps_)
                e['attempts'][30]['poseAfter'] = bytes(b).hex()
            return fn
        v, _ = self.mutated(bump(3))
        self.assertEqual(v, 'PASS')
        v, ch = self.mutated(bump(4))
        self.assertEqual(v, 'FAIL')
        self.assertEqual(ch['successCall']['first']['item'], 'newestPoseAfter')

    def test_declared_u2_pick_is_not_a_match_or_a_mismatch(self):
        def fn(o, e):
            a = o['attempts'][1]
            a['pick'] = 987654           # not a live keyframe id: uninitialized list tail of the original
            a['ringPost'] = [5, 66, 987654, 0, 0, 1]
            a['hooks'] = {}
        v, ch = self.mutated(fn)
        self.assertEqual(v, 'PASS_DECLARED')
        self.assertEqual(ch['candidateId']['declaredU2'], 1)
        self.assertEqual(ch['candidateId']['mismatch'], 0)

    def test_live_original_pick_that_differs_is_a_mismatch_even_if_engine_is_none(self):
        def fn(o, e):
            e['attempts'][20].update(ringPost=[5, 66, 0, 0, 0, 0], outcome=1)
        v, ch = self.mutated(fn)
        self.assertEqual(v, 'FAIL')
        self.assertGreater(ch['candidateId']['mismatch'], 0)

    def test_plain_run_merges_not_applied_stages(self):
        v, ch = self.mutated(lambda o, e: e['attempts'][20].update(outcome=5), hooked=False)
        self.assertEqual(v, 'PASS')       # INVALID_HANDLE..PNP_FAILED are one class when the match count was not recorded
        self.assertEqual(ch['matchCount']['status'], 'UNOBSERVED')

    def test_no_success_is_blocked_before_judging(self):
        orig, eng = make_pair()
        orig['firstSuccess'] = None
        with self.assertRaises(C.Invalid):
            C.judge_attempts(orig, eng)

    def test_empty_comparison_is_never_a_pass(self):
        orig, eng = make_pair()
        eng['attempts'] = {}
        v, ch = verdict(orig, eng)
        self.assertEqual(v, 'FAIL')       # every original attempt is missing
        self.assertEqual(C.verdict_of([C.check('attemptFrameSet', compared=0), C.check('successCall', compared=1, match=1)]), 'INVALID')


class Matrix(unittest.TestCase):
    def test_bits_and_ulps(self):
        a = mat(1.0)
        self.assertTrue(C.matrix_diff(a, a)['bitIdentical'])
        b = bytearray(a)
        b[8:12] = struct.pack('<I', struct.unpack('<I', a[8:12])[0] + 2)
        d = C.matrix_diff(a, bytes(b))
        self.assertFalse(d['bitIdentical'])
        self.assertEqual(d['first']['component'], 2)
        self.assertEqual(d['maxUlps'], 2.0)
        self.assertFalse(C.matrix_diff(a, a[:100])['bitIdentical'])


class Aggregate(unittest.TestCase):
    def agg(self, **kw):
        res = [{'arm': arm, 'verdict': v} for arm, vs in kw.items() for v in vs]
        return C.aggregate(res)['verdict']

    def test_rules(self):
        self.assertEqual(self.agg(on=['PASS'] * 3, off=['PASS'] * 3, null=['PASS']), 'PASS')
        self.assertEqual(self.agg(on=['PASS', 'PASS_DECLARED', 'PASS']), 'PASS_DECLARED')
        self.assertEqual(self.agg(on=['PASS'] * 6, off=['PASS'], null=['FAIL']), 'FAIL')
        # one evaluated PASS next to four INVALID must not be a PASS (ba-e2e v2 M-2)
        self.assertEqual(self.agg(on=['PASS'] + ['INVALID'] * 4), 'INVALID')
        self.assertEqual(self.agg(on=['PASS', 'PASS']), 'INSUFFICIENT')
        self.assertEqual(self.agg(on=['BLOCKED'] * 3), 'BLOCKED')
        self.assertEqual(self.agg(on=['PASS'] * 3, null=['TOOL_ERROR']), 'TOOL_ERROR')
        self.assertEqual(self.agg(on=['FAIL', 'INVALID']), 'FAIL')


class ExitCodes(unittest.TestCase):
    def test_tool_crash_is_not_fail(self):
        with tempfile.TemporaryDirectory() as d:
            rc = C.main(['on', '--original', d + '/missing', '--engine', d + '/missing2', '--out', d + '/out.json'])
        self.assertEqual(rc, C.EXIT['TOOL_ERROR'])
        self.assertNotIn(rc, (0, 1, C.EXIT['FAIL']))

    def test_exit_codes_distinct(self):
        vals = [C.EXIT[k] for k in ('PASS', 'TOOL_ERROR', 'FAIL', 'INVALID', 'INSUFFICIENT', 'BLOCKED')]
        self.assertEqual(len(set(vals)), len(vals))
        self.assertNotIn(1, vals)

    def test_refuses_to_overwrite(self):
        with tempfile.TemporaryDirectory() as d:
            p = pathlib.Path(d) / 'x.json'
            p.write_text('{}')
            with self.assertRaises(SystemExit):
                C.write_new(p, {})


if __name__ == '__main__':
    unittest.main()

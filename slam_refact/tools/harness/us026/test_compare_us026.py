#!/usr/bin/env python3
"""Unit tests of compare_us026.py gate mode on SYNTHETIC runs (no original SDK, no replay): the comparator's own null-run
(self vs self = 0) and its mutation power (each defect class must change the decision in the pre-declared direction).

  python3 -I slam_refact/tools/harness/us026/test_compare_us026.py -v
"""
import copy
import gzip
import json
import math
import os
import pathlib
import struct
import sys
import tempfile
import unittest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import compare_us026 as C  # noqa: E402

N = 40          # frames
KF = (13, 18, 23, 28, 33)
LATCH = 30


def f32(x):
    return struct.unpack('<f', struct.pack('<f', x))[0]


def nudge(x, k):
    """x moved by k float32 ulps (upwards in magnitude)."""
    b = struct.unpack('<I', struct.pack('<f', abs(x)))[0] + k
    y = struct.unpack('<f', struct.pack('<I', b))[0]
    return math.copysign(y, x)


def pose(i):
    return [f32(0.01 * i), f32(1.0 + 0.002 * i), f32(-0.03 * i)], [f32(0.99), f32(0.01 * (i % 7)), f32(0.02), f32(0.03)]


def write_jsonl(p, rows, gz=False):
    text = ''.join(json.dumps(r) + '\n' for r in rows)
    if gz:
        with gzip.open(p, 'wt') as f:
            f.write(text)
    else:
        pathlib.Path(p).write_text(text)


class Synth:
    """One original run directory and one engine replay directory with equal content."""

    def __init__(self, base, name, keys, method=1, latch=LATCH, scale=0.4567):
        self.o = pathlib.Path(base) / ('orig-' + name)
        self.e = pathlib.Path(base) / ('eng-' + name)
        self.o.mkdir()
        self.e.mkdir()
        self.keys, self.method, self.latch, self.scale = keys, method, latch, f32(scale)
        self.trace, self.eng, self.probe, self.poke = [], [], [], []
        for i in range(N):
            p, q = pose(i)
            st = 'LIMITED' if i < 5 else 'NORMAL'
            rs = 'INITIALIZING' if i < 5 else 'UNSPECIFIED'
            wp = 0 if i < 13 else 20 + i
            self.trace.append({'i': i, 'fresh': i != 7, 'status': st, 'reason': rs, 'p': p, 'q_wxyz': q, 'worldPoints': wp, 'wpAdded': 1, 'wpRemoved': 0})
            kfl = sum(1 for k in KF if k <= i)
            pts = [[0, 0, 5 + i], [0, 0, 2], [0, 0, 1], [0, 0, 0], [0, 0, 0]]
            latched = latch is not None and i >= latch
            self.probe.append({'i': i if i != 7 else None, 'seq': i + 1, 'outMapReset': 1 if i == 2 else 0, 'outKeyframeCreated': 1 if i in KF else 0, 'kfLive': kfl,
                               **{'pts%d' % (k + 1): pts[k] for k in range(5)}})
            self.poke.append({'i': i if i != 7 else None, 'seq': i + 1, 'gap': [], 'post': [0, keys.get('ba', 0), 0, keys.get('mse', 1), 0], 'root': 1000,
                              'baLatch': 1 if keys.get('ba') == 1 and i >= 13 else 0, 'kfPoseChanged': 3 if keys.get('ba') == 1 and i in KF[1:] else 0,
                              'mseOk': True, 'mseSum': 700.0 if i >= 25 else 100.0, 'mseResidual': -1 if method in (3, 4) and i >= 25 else 0.05,
                              'vtLatch': 1 if latched else 0, 'vtLatchedScale': self.scale if latched else 0})
            self.eng.append({'i': i, 'fresh': i != 7, 'status': st, 'reason': rs, 'p': list(p), 'q_wxyz': list(q), 'worldPoints': wp, 'wpAdded': 1, 'wpRemoved': 0,
                             'srMapReset': i == 2, 'srKeyframeCreated': i in KF, 'srKeyframes': kfl, 'srPointsBySource': [x[2] for x in pts],
                             'srScaleLatched': latched, 'srLatchedScale': self.scale if latched else 0})
        self.wp = [{'frameIndex': 20, 'points': [[11, 0, 0.1, 0.2, 0.3], [12, 0, 1.0, 2.0, 3.0]]}]
        self.meta = {'status': 'COMPLETED', 'summary': {}, 'recordedInputs': {'wrapped': True, 'error': None, 'droppedCalls': 0},
                     'input': {'engineInputSensors': {'sha256Uncompressed': 'abc'}}, 'loadAvgAtStart': [1, 1, 1]}

    def write(self):
        write_jsonl(self.o / 'trace.jsonl', self.trace)
        write_jsonl(self.o / 'memprobe.jsonl.gz', [{'schema': 'x', 'wasmSha256': C.WASM_SHA256, 'error': None}] + self.probe, gz=True)
        write_jsonl(self.o / 'poke.jsonl.gz', [{'schema': 'y', 'keys': self.keys, 'wrapped': True, 'error': None, 'wasmSha256': C.WASM_SHA256}] + self.poke, gz=True)
        write_jsonl(self.o / 'worldpoints-sampled.jsonl', getattr(self, 'wp_orig', self.wp))
        (self.o / 'meta.json').write_text(json.dumps(self.meta))
        write_jsonl(self.e / 'trace.jsonl', self.eng)
        write_jsonl(self.e / 'worldpoints-sampled.jsonl', self.wp)
        return self


def run_entry(s):
    return {'dir': str(s.o), 'runnerExit': 0, 'prerun': {'ok': True}}


class Base(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.dir = self.tmp.name
        self.k = 0

    def tearDown(self):
        self.tmp.cleanup()

    def synth(self, keys, **kw):
        self.k += 1
        return Synth(self.dir, str(self.k), keys, **kw)


class BaScenario(Base):
    def make(self, mutate_on=None, mutate_off=None, mutate_null=None, mutate_poke=None):
        poke = self.synth({'ba': 1})
        null = self.synth({'ba': 0, 'mse': 1})
        off = copy.deepcopy(poke.eng)
        for r in off[20:]:          # the BA-off arm differs from the BA-on original (M-BA-1)
            r['p'][0] = nudge(r['p'][0], 1000)
        for m, rows in ((mutate_on, poke.eng), (mutate_null, null.eng)):
            if m:
                m(rows)
        if mutate_poke:
            mutate_poke(poke)
        if mutate_off:
            mutate_off(off)
        poke.write()
        null.write()
        offdir = pathlib.Path(self.dir) / ('off-%d' % self.k)
        offdir.mkdir()
        write_jsonl(offdir / 'trace.jsonl', off)
        write_jsonl(offdir / 'worldpoints-sampled.jsonl', poke.wp)
        spec = {'gate': 'ba', 'scenario': 'synthetic', 'kind': 'test', 'runs': {'poke': run_entry(poke), 'null': run_entry(null)},
                'replays': {'on': {'dir': str(poke.e), 'exit': 0}, 'off': {'dir': str(offdir), 'exit': 0}, 'null': {'dir': str(null.e), 'exit': 0}},
                'sensors': {'expectedSha256Uncompressed': 'abc'}}
        return C.ba_scenario(spec)

    def test_identical_pass(self):
        r = self.make()
        self.assertEqual((r['class'], r['verdict']), ('EVALUATED', 'PASS'))
        self.assertTrue(r['compare']['on']['bitIdentical'])

    def test_integer_plus_one_fails_D(self):          # sub-resolution mutation of the integer tier: one count + 1 on one frame
        def m(rows):
            rows[21]['srPointsBySource'][0] += 1
        r = self.make(mutate_on=m)
        self.assertEqual(r['tiers']['G-BA-D'], 'FAIL')
        self.assertEqual(r['verdict'], 'FAIL')

    def test_status_reason_fresh_wp_list(self):
        for field in ('status', 'reason', 'fresh', 'wplist', 'kf', 'reset'):
            def m(rows, field=field):
                if field == 'status':
                    rows[22]['status'] = 'LIMITED'
                elif field == 'reason':
                    rows[22]['reason'] = 'EXCESSIVE_MOTION'
                elif field == 'fresh':
                    rows[7]['fresh'] = True
                elif field == 'kf':
                    rows[18]['srKeyframeCreated'] = False
                elif field == 'reset':
                    rows[3]['srMapReset'] = True
            if field == 'wplist':
                r = self.make(mutate_poke=lambda s: setattr(s, 'wp_orig', [{'frameIndex': 20, 'points': [[11, 0, 0.1, 0.2, 0.3], [13, 0, 1.0, 2.0, 3.0]]}]))
            else:
                r = self.make(mutate_on=m)
            self.assertEqual(r.get('verdict'), 'FAIL', field)

    def test_pose_3ulp_pass_4ulp_fail(self):
        def m3(rows):
            rows[25]['p'][1] = nudge(rows[25]['p'][1], 3)
        def m4(rows):
            rows[25]['p'][1] = nudge(rows[25]['p'][1], 4)
        self.assertEqual(self.make(mutate_on=m3)['verdict'], 'PASS')
        r = self.make(mutate_on=m4)
        self.assertEqual((r['tiers']['G-BA-S'], r['verdict']), ('FAIL', 'FAIL'))

    def test_null_one_ulp_is_invalid(self):            # the null-poke control must be bit-identical, else the scenario is INVALID
        def m(rows):
            rows[30]['q_wxyz'][2] = nudge(rows[30]['q_wxyz'][2], 1)
        r = self.make(mutate_null=m)
        self.assertEqual(r['class'], 'INVALID')

    def test_off_arm_identical_blocked(self):           # E-BA-1: BA without effect (the off arm equals the original)
        poke = self.synth({'ba': 1}).write()
        null = self.synth({'ba': 0, 'mse': 1}).write()
        spec = {'gate': 'ba', 'scenario': 's', 'runs': {'poke': run_entry(poke), 'null': run_entry(null)},
                'replays': {'on': {'dir': str(poke.e), 'exit': 0}, 'off': {'dir': str(poke.e), 'exit': 0}, 'null': {'dir': str(null.e), 'exit': 0}},
                'sensors': {'expectedSha256Uncompressed': 'abc'}}
        self.assertEqual(C.ba_scenario(spec)['class'], 'BLOCKED')

    def test_no_effect_but_on_arm_fails_counts(self):   # E-BA-1 not met, yet the BA-on arm differs: FAIL
        def m(s):
            for r in s.poke:
                r['kfPoseChanged'] = 0
            s.eng[30]['p'][0] = nudge(s.eng[30]['p'][0], 9)
        self.assertEqual(self.make(mutate_poke=m).get('verdict'), 'FAIL')

    def test_no_kf_pose_change_blocked(self):
        def m(s):
            for r in s.poke:
                r['kfPoseChanged'] = 0
        self.assertEqual(self.make(mutate_poke=m)['class'], 'BLOCKED')

    def test_gap_on_keyframe_call_invalid(self):
        def m(s):
            s.poke[18]['gap'] = ['ba']
        self.assertEqual(self.make(mutate_poke=m)['class'], 'INVALID')

    def test_gap_on_non_keyframe_call_valid(self):
        def m(s):
            s.poke[19]['gap'] = ['ba']
        self.assertEqual(self.make(mutate_poke=m)['class'], 'EVALUATED')

    def test_no_ba_latch_invalid(self):
        def m(s):
            for r in s.poke:
                r['baLatch'] = 0
        self.assertEqual(self.make(mutate_poke=m)['class'], 'INVALID')

    def test_sensors_mismatch_invalid(self):
        def m(s):
            s.meta['input']['engineInputSensors']['sha256Uncompressed'] = 'other'
        self.assertEqual(self.make(mutate_poke=m)['class'], 'INVALID')

    def test_wrong_wasm_invalid(self):
        poke = self.synth({'ba': 1})
        poke.write()
        p = poke.o / 'poke.jsonl.gz'
        rows = [json.loads(x) for x in gzip.open(p, 'rt')]
        rows[0]['wasmSha256'] = '0' * 64
        write_jsonl(p, rows, gz=True)
        o = C.load_original(poke.o)
        self.assertFalse(C.run_validity(o, {'ba': 1}, 'ba')['valid'])

    def test_original_abort_is_blocked(self):           # design §3.9: the BA-poke original did not complete, the null one did
        def m(s):
            s.meta['status'] = 'FAILED'
        self.assertEqual(self.make(mutate_poke=m)['class'], 'BLOCKED')

    def test_wp_position_is_report_only(self):
        def m(s):
            s.wp = [{'frameIndex': 20, 'points': [[11, 0, 0.1, 0.2, 0.30001], [12, 0, 1.0, 2.0, 3.0]]}]
        poke = self.synth({'ba': 1})
        m(poke)
        poke.write()
        write_jsonl(poke.e / 'worldpoints-sampled.jsonl', [{'frameIndex': 20, 'points': [[11, 0, 0.1, 0.2, 0.3], [12, 0, 1.0, 2.0, 3.0]]}])
        c = C.compare(C.load_original(poke.o), C.load_engine(poke.e))
        self.assertEqual(c['S']['wpPositionPrintedMismatchFrames'], 1)
        self.assertTrue(c['bitIdentical'])

    def test_original_nondeterminism_before_ba_is_invalid(self):   # the ORIGINAL differs before BA could act; both engine arms agree
        def m(s):
            s.trace[5]['p'][0] = nudge(s.trace[5]['p'][0], 50)
        r = self.make(mutate_poke=m)
        self.assertEqual(r['class'], 'INVALID')
        self.assertEqual(r['nondeterminism']['firstDifferenceFrame'], 5)

    def test_engine_defect_before_ba_still_fails(self):            # an engine-only defect before the BA frame: on != off there, FAIL
        def m(rows):
            rows[5]['p'][0] = nudge(rows[5]['p'][0], 50)
        r = self.make(mutate_on=m)
        self.assertEqual((r['class'], r.get('verdict')), ('EVALUATED', 'FAIL'))

    def test_original_difference_after_ba_frame_fails(self):        # after the first BA frame the difference is judged
        def m(s):
            s.trace[16]['p'][0] = nudge(s.trace[16]['p'][0], 50)
        self.assertEqual(self.make(mutate_poke=m).get('verdict'), 'FAIL')


class MseScenario(Base):
    def make(self, mutate=None, methods=(2, 3, 4, 5), no_effect=()):
        null = self.synth({'ba': 0, 'mse': 1}).write()
        runs, reps = {'null': run_entry(null)}, {'null': {'dir': str(null.e), 'exit': 0}}
        for n in methods:
            s = self.synth({'mse': n}, method=n, scale=0.3 + 0.05 * n)
            m1 = copy.deepcopy(s.eng)
            for r in m1[LATCH:]:
                r['srLatchedScale'] = f32(0.11)
                r['p'][0] = nudge(r['p'][0], 5000)
            if mutate:
                mutate(n, s)
            s.write()
            d = pathlib.Path(self.dir) / ('m1-%d' % self.k)
            d.mkdir()
            write_jsonl(d / 'trace.jsonl', m1 if n != 5 and n not in no_effect else s.eng)   # method 5 outside the device table = method 1
            write_jsonl(d / 'worldpoints-sampled.jsonl', s.wp)
            runs['m%d' % n] = run_entry(s)
            reps['m%d-on' % n] = {'dir': str(s.e), 'exit': 0}
            reps['m%d-m1' % n] = {'dir': str(d), 'exit': 0}
        return C.mse_scenario({'gate': 'mse', 'scenario': 'synthetic', 'runs': runs, 'replays': reps, 'sensors': {'expectedSha256Uncompressed': 'abc'}})

    def test_identical_pass(self):
        r = self.make()
        self.assertEqual(r['class'], 'EVALUATED')
        self.assertEqual({k: v.get('verdict') for k, v in r['pairs'].items()}, {'m2': 'PASS', 'm3': 'PASS', 'm4': 'PASS', 'm5': None})
        self.assertEqual(r['pairs']['m5']['class'], 'BLOCKED')    # method 5 outside the device table = method 1: no effect (E-MSE-2)
        self.assertEqual(r['M-ID-5']['verdict'], 'PASS')
        self.assertFalse(r['pairs']['m2']['R-MSE-2_method1Arm']['bitIdentical'])

    def scale(self, method, k_ulps=None, rel=None):
        def m(n, s):
            if n != method:
                return
            for r in s.eng:
                if r['srScaleLatched']:
                    r['srLatchedScale'] = nudge(s.scale, k_ulps) if k_ulps is not None else f32(s.scale * (1 + rel))
        return self.make(mutate=m)['pairs']['m%d' % method]

    def test_ulp_methods_scale(self):
        self.assertEqual(self.scale(2, k_ulps=3)['tiers']['G-MSE-S'], 'PASS')
        self.assertEqual(self.scale(2, k_ulps=4)['tiers']['G-MSE-S'], 'FAIL')
        self.assertEqual(self.scale(5, k_ulps=4)['verdict'], 'FAIL')     # a FAIL counts even without effect (E-MSE-2)

    def test_rel_methods_scale(self):
        p = self.scale(3, rel=5e-7)
        self.assertEqual(p['tiers']['G-MSE-S'], 'PASS')
        self.assertEqual(p['tiers']['G-MSE-P_detail']['reportFromFrame'], LATCH)    # post-latch pose is REPORT once the 3/4 scales differ
        self.assertEqual(self.scale(4, rel=2e-6)['tiers']['G-MSE-S'], 'FAIL')

    def test_rel_method_pre_latch_pose_still_gated(self):
        def m(n, s):
            if n == 3:
                for r in s.eng:
                    if r['srScaleLatched']:
                        r['srLatchedScale'] = f32(s.scale * (1 + 5e-7))
                s.eng[LATCH - 3]['p'][2] = nudge(s.eng[LATCH - 3]['p'][2], 10)
        self.assertEqual(self.make(mutate=m)['pairs']['m3']['tiers']['G-MSE-P'], 'FAIL')

    def test_ulp_method_post_latch_pose(self):
        def m(n, s):
            if n == 2:
                s.eng[LATCH + 2]['p'][2] = nudge(s.eng[LATCH + 2]['p'][2], 4)
        self.assertEqual(self.make(mutate=m)['pairs']['m2']['tiers']['G-MSE-P'], 'FAIL')

    def test_latch_frame_shift_fails_D(self):
        def m(n, s):
            if n == 4:
                s.eng[LATCH]['srScaleLatched'] = False
        p = self.make(mutate=m)['pairs']['m4']
        self.assertEqual(p['tiers']['G-MSE-D'], 'FAIL')

    def test_e_mse_2_no_effect_blocked(self):
        def m(n, s):
            if n == 2:
                s.no_effect = True
        r = self.make(mutate=m, no_effect=(2,))
        self.assertEqual(r['pairs']['m2']['class'], 'BLOCKED')

    def test_e_mse_1(self):
        def low(n, s):
            if n == 2:
                for r in s.poke:
                    r['mseSum'] = 599.0
        self.assertEqual(self.make(mutate=low)['pairs']['m2']['class'], 'BLOCKED')

        def low_fail(n, s):                                  # path not executed but the engine differs: FAIL, not BLOCKED
            if n == 2:
                for r in s.poke:
                    r['mseSum'] = 599.0
                s.eng[LATCH + 1]['p'][0] = nudge(s.eng[LATCH + 1]['p'][0], 9)
        self.assertEqual(self.make(mutate=low_fail)['pairs']['m2'].get('verdict'), 'FAIL')

        def nores(n, s):
            if n == 3:
                for r in s.poke:
                    r['mseResidual'] = 0.1
        self.assertEqual(self.make(mutate=nores)['pairs']['m3']['class'], 'BLOCKED')

    def test_mid5_detects(self):
        def m(n, s):
            if n == 5:
                for r in s.poke:
                    if r['vtLatch']:
                        r['vtLatchedScale'] = nudge(s.scale, 10)
        r = self.make(mutate=m)
        self.assertEqual(r['M-ID-5']['verdict'], 'FAIL')

    def test_null_latch_difference_invalid(self):
        null = self.synth({'ba': 0, 'mse': 1})
        for r in null.eng:
            if r['srScaleLatched']:
                r['srLatchedScale'] = nudge(null.scale, 1)
        null.write()
        spec = {'gate': 'mse', 'scenario': 's', 'runs': {'null': run_entry(null)}, 'replays': {'null': {'dir': str(null.e), 'exit': 0}}, 'sensors': {}}
        self.assertEqual(C.mse_scenario(spec)['class'], 'INVALID')

    def test_mse_ok_false_invalid(self):
        def m(n, s):
            if n == 2:
                s.poke[10]['mseOk'] = False
        self.assertEqual(self.make(mutate=m)['pairs']['m2']['class'], 'INVALID')

    def test_original_nondeterminism_before_estimate_is_invalid(self):
        def m(n, s):
            if n == 4:
                s.trace[10]['q_wxyz'][1] = nudge(s.trace[10]['q_wxyz'][1], 20)
        r = self.make(mutate=m)
        self.assertEqual(r['pairs']['m4']['class'], 'INVALID')
        self.assertEqual(r['pairs']['m2']['verdict'], 'PASS')

    def test_engine_defect_before_estimate_still_fails(self):
        def m(n, s):
            if n == 4:
                s.eng[10]['q_wxyz'][1] = nudge(s.eng[10]['q_wxyz'][1], 20)
        self.assertEqual(self.make(mutate=m)['pairs']['m4']['verdict'], 'FAIL')

    def test_original_difference_after_estimate_frame_fails(self):
        def m(n, s):
            if n == 2:
                s.trace[26]['p'][1] = nudge(s.trace[26]['p'][1], 20)
        self.assertEqual(self.make(mutate=m)['pairs']['m2']['verdict'], 'FAIL')

    def test_mid5_original_nondeterminism_not_evaluated(self):
        def m(n, s):
            if n == 5:
                s.trace[10]['p'][1] = nudge(s.trace[10]['p'][1], 20)
        r = self.make(mutate=m)
        self.assertFalse(r['M-ID-5']['evaluated'])


class SelfNull(Base):
    def test_nr1_zero(self):
        s = self.synth({'ba': 1}).write()
        r = C.nr1(s.o)
        self.assertTrue(r['zero'])
        self.assertEqual(r['compare']['S']['poseMaxUlps'], 0.0)

    def test_ulp32(self):
        self.assertEqual(C.ulp32(1.0), 2 ** -23)
        self.assertEqual(C.ulps(1.0, nudge(1.0, 3)), 3.0)


if __name__ == '__main__':
    unittest.main()

"""Unit tests of the WP5 parity evaluators on the original reference traces (self-consistency).

Run (repo root): python3 -m unittest discover -s slam_refact/tools/harness/tests -v
Needs the evidence tree build/slam-refact-20261015 (reference runs, fixtures); tests that need it
fail with a clear message when it is absent (they are not skipped).
"""
import gzip
import json
import math
import pathlib
import subprocess
import sys
import tempfile
import unittest

HARNESS = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(HARNESS / 'eval'))
sys.path.insert(0, str(HARNESS))

import accuracy  # noqa: E402
import common  # noqa: E402
import derive_linear_accel as dla  # noqa: E402
import frame_parity  # noqa: E402
import gates  # noqa: E402
import parity_report  # noqa: E402
import reference_baseline  # noqa: E402
import trace_metrics as tm  # noqa: E402

RUNS = common.REFERENCE_RUNS
SECTION_6_6_IDS = [
    'F-H2', 'F-H3', 'F-H4', 'F-H5', 'F-H6', 'F-H7', 'F-H8', 'F-H9', 'F-LAY', 'F-FAST', 'F-K',
    'MT-P1', 'MT-P2u', 'MT-P2t', 'MT-P3', 'MT-P4', 'MT-P5a', 'MT-P5b', 'MT-P6', 'MT-P7-1', 'MT-P7-2', 'MT-P7-3', 'MT-P7-4',
    'MT-P8', 'MT-P9', 'MB-TRI', 'MB-UNIT', 'MB-EPI', 'G-M1', 'G-M2', 'G-M3', 'G-M4', 'G-M5', 'G-M6', 'G-M7',
    'V-MP', 'V-H1/H2', 'V-H3', 'V-ST', 'V-3DOF', 'V-LATCH', 'V-XR', 'V-P1-P4', 'V-P5/P6', 'V-P8', 'V-FFT-abc', 'V-FFT-d',
    'V-FFT-ISO', 'V-H7', 'V-H7b', 'V-EV1', 'V-EV2', 'V-EV3', 'V-EV4', 'V-EV5', 'V-PD',
    'MS-P1', 'MS-P2', 'MS-P3', 'MS-P4', 'MS-P5', 'MS-P6', 'MS-P7', 'MS-P8', 'MS-P9a-n', 'MS-RT', 'A-1', 'A-2', 'A-3',
    'B-1', 'B-2', 'B-3', 'B-4', 'B-5']


def require_evidence(test):
    if not RUNS.exists():
        test.fail('evidence tree missing: %s (BLOCKED, not skipped)' % RUNS)


class ThresholdsTest(unittest.TestCase):
    def test_registered_hash_and_gate_set(self):
        th, digest = parity_report.load_thresholds()
        self.assertEqual(digest, parity_report.REGISTERED_SHA256)
        ids = [g['id'] for g in th['gates']]
        self.assertEqual(ids, SECTION_6_6_IDS)
        for g in th['gates']:
            self.assertIn(g['class'], parity_report.CLASS_ORDER, g['id'])
            for k in ('owner', 'level', 'evaluator', 'metric', 'tolerance', 'source'):
                self.assertTrue(g.get(k), (g['id'], k))
        blocked = sorted(g['id'] for g in th['gates'] if g.get('predeclared_blocked_reason'))
        self.assertEqual(blocked, ['B-5', 'V-H7'])

    def test_baseline_reproduces(self):
        require_evidence(self)
        th, _ = parity_report.load_thresholds()
        fresh = json.loads(json.dumps(reference_baseline.build(), sort_keys=True))
        self.assertEqual(fresh, th['reference_baseline']['data'])


class SchemaTest(unittest.TestCase):
    def test_reference_traces_validate(self):
        require_evidence(self)
        for sc in common.SCENARIOS:
            for lb in common.REFERENCE_RUN_LABELS:
                rows = common.load_trace(common.reference_trace_path(sc, lb))
                self.assertEqual(common.validate_trace(rows, engine=False), [], (sc, lb))

    def test_engine_rules(self):
        row = {k: None for k in common.REFERENCE_FIELDS}
        row.update(i=0, t=0.0, fresh=True, status='NORMAL', reason='UNSPECIFIED', p=[0, 2, 0], q_wxyz=[1, 0, 0, 0], worldPoints=0)
        self.assertEqual(common.validate_row(dict(row, srRepeated=False), engine=True), [])
        self.assertTrue(common.validate_row(dict(row, extra=1), engine=True))
        self.assertTrue(common.validate_row(dict(row, publishMs=1.0), engine=True))
        self.assertTrue(common.validate_row(dict(row, srRepeated=False), engine=False))


class AccuracyTest(unittest.TestCase):
    def test_matches_lane_c_analysis(self):
        require_evidence(self)
        fx = reference_baseline.TUM_FIXTURES['tum-room1-full']
        gt = accuracy.load_gt(fx / 'groundtruth-body.csv')
        t_ic = json.loads((fx / 'manifest.json').read_text())['t_ic']
        lane_c = json.loads((RUNS / 'tum-room1-full/analysis.json').read_text())
        for lb in ('r1', 'r2', 'profile1'):
            rows = common.load_trace(common.reference_trace_path('tum-room1-full', lb))
            mine = accuracy.accuracy(rows, gt, t_ic, 0)
            ref = lane_c['runs'][lb]['accuracyLag0']
            for path in (('coverage',), ('se3', 'ateTranslation', 'rmse'), ('sim3', 'scale'), ('sim3', 'ateTranslation', 'rmse'),
                         ('apeRotationDegRotationOnlyAlignment', 'rmse'), ('rpe1s', 'rotationDeg', 'rmse'),
                         ('rpe1s', 'translationSe3', 'p95')):
                a, b = mine, ref
                for k in path:
                    a, b = a[k], b[k]
                self.assertAlmostEqual(a, b, delta=1e-9 * max(1.0, abs(b)), msg=(lb, path))
            self.assertEqual(accuracy.best_gt_frame_lag(rows, gt)[0], lane_c['runs'][lb]['conventionChecks']['bestGtFrameLag'])

    def test_rotation_angle(self):
        q = (math.cos(0.5), math.sin(0.5), 0.0, 0.0)  # 1 rad about x
        self.assertAlmostEqual(common.rotation_angle_deg(common.quat_wxyz_to_matrix(q)), math.degrees(1.0), places=10)
        self.assertAlmostEqual(common.quat_angle_deg((1, 0, 0, 0), q), math.degrees(1.0), places=6)


class TraceMetricTest(unittest.TestCase):
    def test_zero_steps_equal_runtime_checks(self):
        require_evidence(self)
        for sc in ('tum-room1-full', 'flip6-s3-lowtex'):
            path = common.reference_trace_path(sc, 'r1')
            lane = tm._runtime_tool.check_run(str(path))['P7_zero_translation_step_fraction']
            mine = tm.zero_step_fractions(common.load_trace(path))
            self.assertEqual(sorted(lane), sorted(mine))
            for k, v in lane.items():
                self.assertAlmostEqual(mine[k], v['frac'], places=4)

    def test_full_rotation_p95_matches_repeatability(self):
        require_evidence(self)
        for sc in ('tum-room1-full', 'flip6-s2', 'flip6-n1', 'flip6-s3-lowtex'):
            r1, r2 = (common.load_trace(common.reference_trace_path(sc, lb)) for lb in ('r1', 'r2'))
            _, _, dr = frame_parity.matched_diffs(r2, r1, angle=frame_parity.normalized_angle_deg)
            lane = json.loads((RUNS / sc / 'analysis.json').read_text())['repeatability']['r1~r2']['rotationDiffDeg']['p95']
            self.assertAlmostEqual(common.percentile(dr, 95, 'linear'), lane, delta=1e-9 * lane)
            # The frozen baseline field uses the raw-acos angle of reference_variability.py on float32-rounded,
            # not re-normalised quaternions (informational only): |q| - 1 ~ 1e-7 moves 2 acos(d) by up to
            # ~5e-4 deg at 3 deg, so it agrees with the normalised lane C value only to 1e-3 deg.
            th, _ = parity_report.load_thresholds()
            raw = th['reference_baseline']['data']['scenarios'][sc]['r1_r2']['full_rotation_diff_p95_deg_linear']
            self.assertAlmostEqual(raw, lane, delta=1e-3)

    def test_pre_divergence_matches_reference_variability(self):
        require_evidence(self)
        var = json.loads((common.EVIDENCE_ROOT / 'spec/vio-tracker/reference-variability.json').read_text())
        for sc, v in var.items():
            r1, r2 = (common.load_trace(common.reference_trace_path(sc, lb)) for lb in ('r1', 'r2'))
            last = v['first_divergent_index']
            idx, dp, dr = frame_parity.matched_diffs(r1, r2, 0, last - 1)
            self.assertEqual(common.percentile(dp, 95, 'nearest_rank'), v['pos_p95'], sc)
            self.assertAlmostEqual(common.percentile(dr, 95, 'nearest_rank'), v['rot_deg_p95'], places=9, msg=sc)

    def test_scale_latch_proxy(self):
        rows = [{'i': 0, 't': 0.0, 'fresh': True, 'reason': 'INITIALIZING', 'p': [0, 1.0, 0]},
                {'i': 1, 't': 0.1, 'fresh': True, 'reason': 'INITIALIZING', 'p': [0.1, 1.5, 0]},
                {'i': 2, 't': 0.2, 'fresh': True, 'reason': 'UNSPECIFIED', 'p': [0, 0.75, 0]},
                {'i': 3, 't': 0.3, 'fresh': True, 'reason': 'INITIALIZING', 'p': [0, 1.0, 0]}]
        lat = tm.scale_latches(rows)
        self.assertEqual(len(lat), 1)
        self.assertEqual((lat[0]['i'], lat[0]['scale_proxy']), (2, 0.5))


class MsRtRuleTest(unittest.TestCase):
    def test_rules(self):
        a = {'i': 100, 'scale_proxy': 1.0}
        b = {'i': 102, 'scale_proxy': 1.02}
        self.assertEqual(gates.ms_rt_verdict(None, None, None)[0], 'PASS')
        self.assertEqual(gates.ms_rt_verdict(a, None, None)[0], 'FAIL')
        self.assertEqual(gates.ms_rt_verdict({'i': 105, 'scale_proxy': 1.05}, a, b)[1]['rule'], 'primary')
        self.assertEqual(gates.ms_rt_verdict({'i': 107, 'scale_proxy': 1.0}, a, b)[0], 'FAIL')
        self.assertEqual(gates.ms_rt_verdict(None, a, b)[0], 'FAIL')
        far = {'i': 120, 'scale_proxy': 1.5}
        v, info = gates.ms_rt_verdict({'i': 125, 'scale_proxy': 1.4}, a, far)
        self.assertEqual((v, info['rule'], info['L_tol']), ('PASS', 'fallback', 40))
        self.assertEqual(gates.ms_rt_verdict(None, a, None)[0], 'PASS')
        self.assertEqual(gates.ms_rt_verdict({'i': 110, 'scale_proxy': 1.0}, a, None)[0], 'FAIL')


class SelfConsistencyTest(unittest.TestCase):
    """Reference vs reference: every evaluable trace gate must PASS for an original run as candidate."""

    def run_report(self, label):
        th, digest = parity_report.load_thresholds()
        runs = {sc: str(RUNS / sc / label) for sc in common.SCENARIOS}
        injected = {'flip6-s2': runs['flip6-s2'], 'tum-room1-full': runs['tum-room1-full']}
        absolute = {sc: str(RUNS / sc / label) for sc in ('tum-room1-full-absolute', 'flip6-s2-absolute') if (RUNS / sc / label).exists()}
        return parity_report.build_report(th, digest, runs, injected, absolute, {}, ['test'])

    def check(self, label):
        require_evidence(self)
        report = self.run_report(label)
        allowed = {'PASS', 'REPORTED'}
        for r in report['results']:
            if r['id'] not in gates.TRACE_EVALUATORS:
                self.assertEqual(r['verdict'], 'NOT-RUN', r['id'])
                continue
            if r['id'] in ('G-M3', 'G-M4', 'G-M5'):
                self.assertEqual(r['verdict'], 'BLOCKED', (r['id'], r['scenario']))
            elif r['id'] == 'MT-P7-4' and r['scenario'] == 'tum-room1-lowtex':
                self.assertEqual(r['verdict'], 'NOT-REGISTERED')
            elif r['id'] == 'MS-RT' and r['verdict'] == 'NOT-RUN':
                self.fail('absolute reference runs missing for ' + r['scenario'])
            else:
                self.assertIn(r['verdict'], allowed, (r['id'], r['scenario'], r.get('measured')))

    def test_r1_against_itself(self):
        self.check('r1')

    def test_r2_against_r1(self):
        self.check('r2')


class DerivedLinearAccelerationTest(unittest.TestCase):
    def test_formula(self):
        self.assertEqual(dla.linear_acceleration((0.1, 0.2, 9.80665), (1.0, 0.0, 0.0, 0.0)), (0.1, 0.2, 0.0))
        s = math.sqrt(0.5)  # 90 deg about x: device z = earth -y, gravity appears on device +y
        lin = dla.linear_acceleration((0.0, 9.80665, 0.0), (s, s, 0.0, 0.0))
        for v in lin:
            self.assertAlmostEqual(v, 0.0, places=12)

    def test_absolute_runs_dispatched_the_derived_values(self):
        require_evidence(self)
        for sc in ('tum-room1-full-absolute', 'flip6-s2-absolute'):
            for lb in ('r1', 'r2'):
                ev = RUNS / sc / lb / 'events.json'
                self.assertTrue(dla.verify_events(RUNS / sc, ev)['ok'], (sc, lb))


class PrepareInputsTest(unittest.TestCase):
    def test_bundle_matches_reference_inputs(self):
        require_evidence(self)
        with tempfile.TemporaryDirectory() as tmp:
            cmd = [sys.executable, '-I', str(HARNESS / 'prepare_inputs.py'), '--scenario', 'flip6-s3-lowtex', '--label', 't',
                   '--out-root', tmp, '--drop-mask-from', 'r1']
            subprocess.run(cmd, check=True, capture_output=True)
            out = pathlib.Path(tmp) / 'flip6-s3-lowtex' / 't'
            bundle = json.loads((out / 'bundle.json').read_text())
            self.assertTrue(all(bundle['checks'].values()))
            sensors = (out / 'sensors.tsv').read_text().splitlines()
            frames = (out / 'frames.tsv').read_text().splitlines()
            with gzip.open(RUNS / 'flip6-s3-lowtex/engine-input-sensors.jsonl.gz', 'rt') as f:
                src = [json.loads(line) for line in f]
            self.assertEqual(len(sensors), len(src) + 1)
            first_run = sensors[1].split('\t')
            self.assertEqual(first_run[0], 'run')
            self.assertEqual(float(first_run[2]), src[0]['accelIncludingGravity'][0])
            self.assertEqual(int(first_run[1]), round((src[0]['t'] + 1.0) * 1e9))
            self.assertEqual(sensors[0].split('\t')[2:], first_run[2:])
            self.assertEqual(len(frames), 382)
            self.assertTrue(frames[0].startswith('bootstrap\t-1\t'))


if __name__ == '__main__':
    unittest.main()

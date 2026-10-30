"""Unit tests of the OR-4 gate tool (or4_world_regression.py, gate v3): metrics, verdict, mutants, exit classes, held-run and
analysis guards.

Run (repo root): python3 -I -m unittest discover -s slam_refact/tools/image_target/tests -p 'test_or4*.py' -v
The held-run tests build synthetic manifests / designation files in a temporary directory; every bypass of
protocol 12.5-12.6 must be rejected for the expected reason.
"""
import copy
import json
import math
import os
import pathlib
import sys
import tempfile
import unittest

TOOL = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(TOOL))

import or4_world_regression as g  # noqa: E402


def make_trace(n=400, step=0.01, yaw_rate=0.0, bias=(0.0, 0.0, 0.0), norm_every=0, drop_every=0, reset_at=(), init_every=0):
    rows = []
    for i in range(n):
        yaw = math.radians(yaw_rate * i)
        q = [math.cos(yaw / 2), 0.0, math.sin(yaw / 2), 0.0]
        dropped = drop_every and i % drop_every == 0
        limited = norm_every and i % norm_every == 0
        init = init_every and i % init_every == 0
        rows.append({'i': i, 't': i / 30, 'fresh': not dropped,
                     'status': 'LIMITED' if (limited or init) else 'NORMAL', 'reason': 'INITIALIZING' if init else 'UNSPECIFIED',
                     # a helix-like path so that the trajectory has an extent in all three axes
                     'p': None if dropped else [step * i + bias[0], 0.3 * math.sin(i / 20) + bias[1], 0.2 * math.cos(i / 15) + bias[2]],
                     'q_wxyz': None if dropped else q, 'srMapReset': i in reset_at, 'srStageMs': {'frontend': 1.0 + i}})
    return rows


class MetricTests(unittest.TestCase):
    def test_identical_trace_has_zero_metrics(self):
        a = make_trace()
        pm = g.pair_metrics(a, copy.deepcopy(a))
        for m in ('pos', 'rot', 'cov', 'norm', 'reset'):
            self.assertEqual(pm[m], 0, m)
        self.assertLess(pm['scale'], 1e-9)      # null-run floor of the Sim(3) scale (numerical noise)

    def test_position_error_is_normalised_by_the_rms_radius_not_the_path_length(self):
        a = make_trace()
        b = make_trace(bias=(0.0, 0.1, 0.0))
        pm = g.pair_metrics(a, b)
        self.assertAlmostEqual(pm['pos'], 0.1 / g.rms_radius(a), places=9)
        self.assertGreater(g.path_length(a), 0)   # the path length is not the divisor

    def test_scale_metric_sees_a_scale_change_and_not_a_shift(self):
        a = make_trace()
        shifted = make_trace(bias=(5.0, -3.0, 2.0))
        self.assertLess(g.pair_metrics(a, shifted)['scale'], 1e-9)
        scaled = g.trace_mutant('TM-scale-1.2', a)
        self.assertAlmostEqual(g.pair_metrics(a, scaled)['scale'], math.log(1.2), places=6)
        shrunk = g.trace_mutant('TM-scale-0.9', a)
        self.assertAlmostEqual(g.pair_metrics(a, shrunk)['scale'], math.log(1 / 0.9), places=6)

    def test_position_and_scale_ignore_initializing_rows(self):
        a = make_trace(init_every=4)                      # every 4th row is INITIALIZING (not tracked)
        scaled_init = g.trace_mutant('TM-scale-1.5-init', a)   # only the INITIALIZING rows are scaled
        pm = g.pair_metrics(a, scaled_init)
        self.assertEqual(pm['pos'], 0.0)
        self.assertLess(pm['scale'], 1e-9)
        scaled = g.trace_mutant('TM-scale-1.1', a)         # the tracked rows are scaled: seen at full size
        self.assertAlmostEqual(g.pair_metrics(a, scaled)['scale'], math.log(1.1), places=6)
        # a trace that is mostly INITIALIZING: the dilution of v4 (all rows, scale 0.0028 instead of 0.0953) is gone
        mostly = make_trace(init_every=1)
        for r in mostly[::7]:
            r['reason'] = 'UNSPECIFIED'
            r['status'] = 'NORMAL'
        self.assertAlmostEqual(g.pair_metrics(mostly, g.trace_mutant('TM-scale-1.1', mostly))['scale'], math.log(1.1), places=6)

    def test_rotation_angle_is_stable_near_zero(self):
        q = [1.0, 0.0, 0.0, 0.0]
        tiny = [math.cos(2e-8), 0.0, math.sin(2e-8), 0.0]
        self.assertAlmostEqual(g.quat_angle_deg(q, tiny), math.degrees(4e-8), places=12)
        self.assertEqual(g.quat_angle_deg(q, q), 0.0)
        self.assertEqual(g.quat_angle_deg(q, [-1.0, 0.0, 0.0, 0.0]), 0.0)   # q and -q are the same rotation

    def test_coverage_counts_tracked_rows_not_published_poses(self):
        a = make_trace()
        lost = make_trace(init_every=20)           # poses are still published, tracking is INITIALIZING
        pm = g.pair_metrics(a, lost)
        self.assertEqual(pm['cov'], 20)            # 400 / 20 rows fell out of "tracked"
        self.assertEqual(pm['norm'], 20)
        dropped = make_trace(drop_every=10)        # a dropped frame has no pose and is not fresh
        self.assertEqual(g.pair_metrics(a, dropped)['cov'], 40)

    def test_reset_is_none_without_the_key(self):
        a = [{k: v for k, v in r.items() if k != 'srMapReset'} for r in make_trace()]
        self.assertIsNone(g.pair_metrics(a, a)['reset'])
        self.assertEqual(g.pair_metrics(make_trace(), make_trace(reset_at=(5, 9)))['reset'], 2)

    def test_identical_ignores_timing_but_sees_one_ulp(self):
        a = make_trace()
        b = copy.deepcopy(a)
        for r in b:
            r['srStageMs'] = {'frontend': 999.0}
        self.assertTrue(g.identical(a, b)[0])
        b[7]['p'][0] = math.nextafter(b[7]['p'][0], 1.0)
        same, first = g.identical(a, b)
        self.assertFalse(same)
        self.assertEqual(first, 7)
        self.assertFalse(g.identical(a, a[:-1])[0])                 # a deleted row
        self.assertFalse(g.identical(a, make_trace(step=0.02))[0])  # other input

    def test_seeds(self):
        s = g.seeds_of(2275038191)
        self.assertEqual(len(set(s)), 4)
        self.assertEqual(s[0], 2275038191)
        self.assertEqual(s[1], (2275038191 + 2654435761) % 2 ** 32)
        r = g.rng_seeds()
        self.assertEqual(len(set(r + [g.RNG_DEFAULT])), 4)
        self.assertTrue(all(0 <= v < 2 ** 32 for v in s + r))


class VerdictTests(unittest.TestCase):
    def noise(self, **kw):
        row = {'pos': 0.0005, 'rot': 0.1, 'scale': 0.001, 'cov': 0, 'norm': 1, 'reset': 0}
        row.update(kw)
        return row

    def test_tolerance_is_max_of_three_noise_and_floor(self):
        self.assertEqual(g.tolerance(self.noise(pos=0.5), 'pos'), 1.5)
        self.assertEqual(g.tolerance(self.noise(pos=0.0005), 'pos'), g.FLOOR['pos'])
        self.assertEqual(g.tolerance(self.noise(norm=5), 'norm'), 15)
        self.assertAlmostEqual(g.tolerance(self.noise(scale=0.1), 'scale'), 0.3)

    def test_band_verdict_passes_a_small_effect_and_fails_a_large_one(self):
        base = make_trace()
        small = make_trace(bias=(0.0, 0.001, 0.0))
        large = make_trace(bias=(0.0, 0.5, 0.0))
        noise = {'b': self.noise()}
        self.assertTrue(g.band_verdict(noise, g.effect_table({'b': [base]}, {'b': [small]}))['b']['pass'])
        bad = g.band_verdict(noise, g.effect_table({'b': [base]}, {'b': [large]}))['b']
        self.assertFalse(bad['pass'])
        self.assertFalse(bad['pos']['pass'])

    def test_undefined_metric_fails(self):
        a = [{k: v for k, v in r.items() if k != 'srMapReset'} for r in make_trace()]
        res = g.band_verdict({'b': self.noise()}, g.effect_table({'b': [a]}, {'b': [a]}))['b']
        self.assertFalse(res['reset']['pass'])
        self.assertFalse(res['pass'])

    def test_tracked_coverage_cap_can_fail_and_is_scaled_by_the_noise(self):
        base = make_trace()
        lossy = make_trace(init_every=50)          # 8 of 400 rows (2 %) leave "tracked"
        mx = {'b': g.metric_maxima(base)}
        quiet = {'b': self.noise(cov=0, norm=100)}
        res = g.band_verdict(quiet, g.effect_table({'b': [base]}, {'b': [lossy]}), mx)['b']
        self.assertEqual(res['cov_cap']['cap'], g.COV_LOSS_CAP)
        self.assertFalse(res['cov_cap']['pass'])
        self.assertFalse(res['pass'])
        noisy = {'b': self.noise(cov=40, norm=100)}      # cap 3 * 40 / 400 = 0.30
        res2 = g.band_verdict(noisy, g.effect_table({'b': [base]}, {'b': [lossy]}), mx)['b']
        self.assertAlmostEqual(res2['cov_cap']['cap'], 0.30)
        self.assertTrue(res2['cov_cap']['pass'])
        self.assertFalse(res2['cov_cap']['vacuous'])

    def test_cov_cap_is_vacuous_when_it_cannot_be_exceeded(self):
        base = make_trace()
        mx = {'b': g.metric_maxima(base)}
        # tracked rows: 400 -> the largest possible loss is 100 %; R_D[cov] = 150 gives a cap of 112 % -> vacuous
        res = g.band_verdict({'b': self.noise(cov=150)}, g.effect_table({'b': [base]}, {'b': [make_trace(drop_every=2)]}), mx)['b']
        self.assertTrue(res['cov_cap']['vacuous'])
        self.assertTrue(res['cov_cap']['pass'])

    def test_vacuous_maxima_are_row_counts(self):
        base = make_trace(n=200, init_every=2)
        mx = g.metric_maxima(base)
        self.assertEqual(mx['cov'], 200.0)
        self.assertEqual(mx['norm'], 200.0)
        self.assertEqual(mx['tracked'], 100.0)

    def test_exercised_positive_needs_a_loaded_target_and_requested_rois(self):
        self.assertTrue(g.exercised_positive({'targetsLoaded': 1, 'framesWithNextRois': 479}))
        self.assertFalse(g.exercised_positive({'targetsLoaded': 1, 'framesWithNextRois': 0}))   # 938 / ROI path off
        self.assertFalse(g.exercised_positive({'targetsLoaded': 0, 'framesWithNextRois': 0}))   # the N and D arms
        self.assertFalse(g.exercised_positive({'targetsLoaded': 0, 'framesWithNextRois': 7}))
        self.assertFalse(g.exercised_positive({'targetsLoaded': 1}))

    def test_observed_effect_is_a_report_only(self):
        a = make_trace()
        self.assertFalse(g.observed_effect([a], [copy.deepcopy(a)]))
        self.assertTrue(g.observed_effect([a], [make_trace(bias=(0, 1e-7, 0))]))

    def test_noise_table_uses_all_noise_run_pairs(self):
        traces = [make_trace(bias=(0, 0.0, 0.0)), make_trace(bias=(0, 0.01, 0.0)), make_trace(bias=(0, 0.03, 0.0)), make_trace()]
        t = g.noise_table({'b': traces})
        self.assertAlmostEqual(t['b']['pos'], 0.03 / g.rms_radius(traces[0]), places=9)


class TraceMutantTests(unittest.TestCase):
    """Every registered trace mutant moves its own metric by the analytic amount; detection follows the tolerance."""

    def setUp(self):
        self.base = make_trace(n=400)
        self.zero = {m: 0.0 for m in g.METRICS}

    def test_each_mutant_moves_its_metric_by_the_analytic_amount(self):
        for name, (metric, analytic) in g.TRACE_MUTANTS.items():
            pm = g.pair_metrics(self.base, g.trace_mutant(name, self.base))
            if analytic is not None:
                self.assertAlmostEqual(pm[metric], analytic, places=6, msg=name)
            self.assertGreater(pm[metric], 0, name)

    def test_required_mutants_detected_and_controls_not_with_zero_noise(self):
        for name, (metric, _) in g.TRACE_MUTANTS.items():
            det, fails, pm, cap = g.mutation_detected(self.zero, self.base, g.trace_mutant(name, self.base))
            expected, value, tol = g.mutant_expectation(self.zero, name, pm)
            self.assertEqual(metric in fails, expected, f'{name}: {pm}')
        # the named controls are below the floors, the others above
        for name in ('TM-pos-3pct', 'TM-scale-1.02', 'TM-rot-0.5deg', 'TM-norm-2', 'TM-cov-2', 'TM-reset-2'):
            det, fails, _, _ = g.mutation_detected(self.zero, self.base, g.trace_mutant(name, self.base))
            self.assertNotIn(g.TRACE_MUTANTS[name][0], fails, name)   # metric-specific: a scale change also moves positions
        for name in ('TM-pos-10pct', 'TM-scale-1.1', 'TM-scale-1.2', 'TM-scale-1.5', 'TM-scale-0.9', 'TM-rot-2deg',
                     'TM-norm-every10', 'TM-cov-every20', 'TM-reset-5'):
            self.assertTrue(g.mutation_detected(self.zero, self.base, g.trace_mutant(name, self.base))[0], name)

    def test_reset_mutant_adds_exactly_n_resets_even_when_rows_already_reset(self):
        base = make_trace(n=400, reset_at=(11, 12))          # the v3 defect: rows 10..14 already contained resets
        self.assertEqual(g.pair_metrics(base, g.trace_mutant('TM-reset-5', base))['reset'], 5)
        self.assertEqual(g.pair_metrics(base, g.trace_mutant('TM-reset-2', base))['reset'], 2)

    def test_mutant_is_a_copy(self):
        before = json.dumps(self.base)
        g.trace_mutant('TM-scale-1.5', self.base)
        g.trace_mutant('TM-cov-every20', self.base)
        self.assertEqual(json.dumps(self.base), before)

    def test_a_noisy_bundle_hides_a_small_scale_change_by_construction(self):
        noisy = dict(self.zero, scale=0.05)          # tolerance 0.15 > ln 1.1
        det, fails, pm, cap = g.mutation_detected(noisy, self.base, g.trace_mutant('TM-scale-1.1', self.base))
        self.assertNotIn('scale', fails)
        expected, _, _ = g.mutant_expectation(noisy, 'TM-scale-1.1', pm)
        self.assertFalse(expected)

    def test_old_blind_spot_a_scale_error_with_unchanged_rotation_and_counts_is_now_seen(self):
        # v2 B1: x1.10 / x1.20 passed the position metric normalised by the path length. Here the scale metric flags it.
        noise = dict(self.zero)
        det, fails, _, _ = g.mutation_detected(noise, self.base, g.trace_mutant('TM-scale-1.1', self.base))
        self.assertTrue(det)
        self.assertIn('scale', fails)
        self.assertNotIn('rot', fails)
        self.assertNotIn('norm', fails)


class DecisionTests(unittest.TestCase):
    """R-US014 (gate v6 section 0.2) as code: every row of the outcome table."""
    P = [('a', 'cpu', True), ('b', 'cpu', True), ('c', 'injected', True), ('d', 'injected', True)]

    def decide(self, **kw):
        args = dict(null_ok=True, run_ok=True, infra=[], det_t_ok=True, exercised_ok=True, powered=self.P)
        args.update(kw)
        return g.us014_decision(**args)

    def test_permit(self):
        self.assertEqual(self.decide()[:2], ('PERMIT', 'PERMIT'))

    def test_every_hold_reason(self):
        self.assertEqual(self.decide(null_ok=False)[1], 'PREMISE-NULL')
        self.assertEqual(self.decide(infra=[('b', 'INFRA', -9, 1)])[1], 'INFRA')
        self.assertEqual(self.decide(run_ok=False)[1], 'PREMISE-RUN')
        self.assertEqual(self.decide(det_t_ok=False)[1], 'PREMISE-DET-T')
        self.assertEqual(self.decide(exercised_ok=False)[1], 'PREMISE-EXERCISED')
        self.assertEqual(self.decide(powered=self.P[:3] + [('d', 'injected', False)])[1], 'BAND-FAIL')
        self.assertEqual(self.decide(powered=self.P[:2])[1], 'POWER-SHORT')                     # 2 powered PASS < 3
        self.assertEqual(self.decide(powered=[('c', 'injected', True), ('d', 'injected', True), ('e', 'injected', True)])[1], 'POWER-SHORT')   # no CPU
        self.assertEqual(self.decide(powered=[])[1], 'POWER-SHORT')

    def test_premises_come_before_the_band_and_infra_never_permits(self):
        self.assertEqual(self.decide(null_ok=False, infra=[1], powered=[('a', 'cpu', False)])[1], 'PREMISE-NULL')
        self.assertEqual(self.decide(infra=[1])[0], 'HOLD')

    def test_non_powered_bundles_are_not_an_input(self):
        # the function only receives powered bundles: a non-powered FAIL cannot hold, a non-powered PASS cannot count
        self.assertEqual(self.decide(powered=self.P + [('x', 'injected', True)] * 5)[0], 'PERMIT')


class LooTests(unittest.TestCase):
    def test_leave_one_out_flags_a_new_realisation_outside_the_others(self):
        runs = [make_trace(bias=(0, b, 0)) for b in (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)]
        mx = {'b': g.metric_maxima(runs[0])}
        quiet = g.loo_false_fail({'b': runs}, ['b'], mx)['b']
        self.assertEqual(quiet['false_fail_cases'], [])
        runs[3] = make_trace(bias=(0, 1.0, 0))                         # one run far from the other six
        res = g.loo_false_fail({'b': runs}, ['b'], mx)['b']
        self.assertEqual(res['false_fail_cases'], [3])
        self.assertAlmostEqual(res['rate'], 1 / 7)


class ExitClassTests(unittest.TestCase):
    def test_classes(self):
        self.assertEqual(g.classify_exit(0, ''), 'OK')
        self.assertEqual(g.classify_exit(-6, 'SR_CHECK failed at pyramid_source.cc:110'), 'CRASH')
        self.assertEqual(g.classify_exit(-11, ''), 'CRASH')
        self.assertEqual(g.classify_exit(2, 'sr_or4_replay: usage'), 'CRASH')
        self.assertEqual(g.classify_exit(1, 'write: No space left on device'), 'INFRA')
        self.assertEqual(g.classify_exit(0, '', timed_out=True), 'CRASH')

    def test_sigkill_is_infra_only_with_a_memory_baseline(self):
        self.assertEqual(g.classify_exit(-9, ''), 'CRASH')                                    # no evidence: a failure
        self.assertEqual(g.classify_exit(-9, '', maxrss_kb=900_000, baseline_maxrss_kb=800_000), 'INFRA')
        self.assertEqual(g.classify_exit(-9, '', maxrss_kb=2_000_000, baseline_maxrss_kb=800_000), 'CRASH')   # product used the memory
        self.assertEqual(g.classify_exit(-9, '', maxrss_kb=1_600_000, baseline_maxrss_kb=800_000), 'INFRA')   # exactly twice


class VacuousTests(unittest.TestCase):
    def test_vacuous_metric_is_excluded_and_marked(self):
        base = make_trace(n=200)
        ref = {'b': [base]}
        mx = {'b': g.metric_maxima(base)}
        noise = {'b': {'pos': 0.0, 'rot': 70.0, 'scale': 0.0, 'cov': 0, 'norm': 0, 'reset': 0}}   # rot tolerance 210 deg >= 180
        wild = make_trace(n=200, yaw_rate=3.0)                                                     # rotation drifts by 597 deg
        res = g.band_verdict(noise, g.effect_table(ref, {'b': [wild]}), mx)['b']
        self.assertTrue(res['rot']['vacuous'])
        self.assertTrue(res['rot']['pass'])
        self.assertNotIn('vacuous', [k for k, v in res.items() if isinstance(v, dict) and not v.get('vacuous', True)])
        noise_ok = {'b': dict(noise['b'], rot=0.1)}
        res2 = g.band_verdict(noise_ok, g.effect_table(ref, {'b': [wild]}), mx)['b']
        self.assertFalse(res2['rot']['vacuous'])
        self.assertFalse(res2['rot']['pass'])

    def test_power_entry_requires_non_vacuous_limits(self):
        base = make_trace(n=200)
        mx = g.metric_maxima(base)
        self.assertTrue(g.power_entry({'pos': 0.0, 'rot': 0.0, 'scale': 0.0, 'cov': 0, 'norm': 0, 'reset': 0}, mx)['powered'])
        self.assertFalse(g.power_entry({'pos': 0.0, 'rot': 4.0, 'scale': 0.0, 'cov': 0, 'norm': 0, 'reset': 0}, mx)['powered'])   # 12 deg > 10
        self.assertFalse(g.power_entry({'pos': 0.0, 'rot': 0.0, 'scale': 0.2, 'cov': 0, 'norm': 0, 'reset': 0}, mx)['powered'])   # ln 1.5 < 0.6


class FileTests(unittest.TestCase):
    def test_write_once_refuses_to_replace(self):
        d = pathlib.Path(tempfile.mkdtemp())
        g.write_once(d / 'r.json', {'a': 1})
        with self.assertRaises(FileExistsError):
            g.write_once(d / 'r.json', {'a': 2})
        self.assertEqual(json.loads((d / 'r.json').read_text()), {'a': 1})

    def test_trace_list_sha_follows_content_and_names(self):
        d = pathlib.Path(tempfile.mkdtemp())
        (d / 'a').mkdir()
        (d / 'a/trace.jsonl').write_text('1')
        (d / 'b').mkdir()
        (d / 'b/trace.jsonl').write_text('2')
        h1, n = g.trace_list_sha(d)
        self.assertEqual(n, 2)
        (d / 'b/trace.jsonl').write_text('3')
        self.assertNotEqual(h1, g.trace_list_sha(d)[0])

    def test_trace_list_sha_unions_roots_and_run_outputs_are_verified(self):
        a, b = pathlib.Path(tempfile.mkdtemp()) / 'D', pathlib.Path(tempfile.mkdtemp()) / 'D'
        for root, name in ((a, 'force'), (b, 's1')):
            (root / 'k' / name).mkdir(parents=True)
            (root / 'k' / name / 'trace.jsonl').write_text(name)
        h, n = g.trace_list_sha(a, b)
        self.assertEqual(n, 2)
        self.assertEqual(h, g.trace_list_sha(b, a)[0])           # order of roots does not matter
        run = pathlib.Path(tempfile.mkdtemp()) / 'T'
        run.mkdir()
        (run / 'trace.jsonl').write_text('rows')
        (run / 'meta.json').write_text('{}')
        status = {'cmd': ['bin', '--bundle', 'x', '--out', str(run)], 'trace_sha256': g.sha256_file(run / 'trace.jsonl'),
                  'meta_sha256': g.sha256_file(run / 'meta.json')}
        self.assertEqual(g.verify_run_outputs(run, [status]), [])
        (run / 'trace.jsonl').write_text('replaced')
        self.assertEqual(g.verify_run_outputs(run, [status]), ['T/trace.jsonl'])

    def test_manifest_core_ignores_review_and_first_run_only(self):
        m = {'a': 1, 'b': {'c': 2}, 'review': None, 'first_real_run_at': None}
        h = g.manifest_core_sha(m)
        self.assertEqual(h, g.manifest_core_sha(dict(m, review={'at': 'x'}, first_real_run_at='y')))
        self.assertNotEqual(h, g.manifest_core_sha(dict(m, a=2)))

    def test_bundle_and_tree_hashes_follow_content(self):
        d = pathlib.Path(tempfile.mkdtemp()) / 'tree'
        (d / 'sub').mkdir(parents=True)
        (d / 'a.txt').write_text('1')
        (d / 'sub' / 'b.txt').write_text('2')
        (d / '__pycache__').mkdir()
        (d / '__pycache__' / 'x.pyc').write_text('ignored')
        h1 = g.tree_sha(d)
        (d / '__pycache__' / 'x.pyc').write_text('still ignored')
        self.assertEqual(h1, g.tree_sha(d))
        (d / 'sub' / 'b.txt').write_text('3')
        self.assertNotEqual(h1, g.tree_sha(d))


class HeldTests(unittest.TestCase):
    """held_check against synthetic files: each bypass is rejected for its own reason (also as the analysis guard)."""

    def setUp(self):
        self.tmp = pathlib.Path(tempfile.mkdtemp())
        self.gate = self.tmp / 'gate.md'
        self.gate.write_text('gate text')
        self.pins = {'engine_binary': 'aa', 'node': 'bb'}
        self.inputs = {'bundle:x': 'cc', 'target:t.json': 'dd', 'noise_table': 'ee', 'n_arm_traces': 'ff', 'd_arm_traces': 'gg'}
        self.manifest = {'gate_id': 'g', 'created_at': '2026-10-20T05:00:00+09:00', 'finalized_at': '2026-10-20T05:30:00+0900', 'sha256': g.sha256_file(self.gate), 'first_real_run_at': None, 'pins': dict(self.pins),
                         'input_pins': dict(self.inputs), 'review': None}
        self.designation = self.tmp / 'designation.json'
        self.markers = self.tmp / 'markers'
        self.markers.mkdir()
        self.write_designation()

    def write_designation(self, **over):
        d = {'gate_id': 'g', 'first_real_run_permitted': True, 'gate_sha256': g.sha256_file(self.gate),
             'manifest_core_sha256': g.manifest_core_sha(self.manifest), 'noise_table_sha256': 'ee', 'n_arm_trace_list_sha256': 'ff', 'd_arm_trace_list_sha256': 'gg'}
        d.update(over)
        self.designation.write_text(json.dumps(d))
        self.manifest['review'] = {'at': '2026-10-20T06:00:00+09:00', 'designation_file_sha256': g.sha256_file(self.designation)}
        self.env = {'OR4_DESIGNATION_SHA256': g.sha256_file(self.designation)}

    def check(self, manifest=None, env=None, pins=None, inputs=None, kind='engine', **kw):
        mp = self.tmp / 'manifest.json'
        mp.write_text(json.dumps(manifest if manifest is not None else self.manifest))
        return g.held_check(mp, self.designation, env if env is not None else self.env, self.gate,
                            pins if pins is not None else self.pins, inputs if inputs is not None else self.inputs, kind, self.markers, **kw)

    def assertHeld(self, result, needle):
        ok, msgs = result
        self.assertFalse(ok)
        self.assertEqual(msgs[0], 'HELD')
        self.assertTrue(any(needle in m for m in msgs), msgs)
        self.assertEqual(list(self.markers.iterdir()), [], 'a rejected run must not leave a marker')

    def test_all_conditions_satisfied(self):
        ok, msgs = self.check()
        self.assertTrue(ok, msgs)
        self.assertEqual(len(list(self.markers.iterdir())), 1)

    def test_second_first_run_of_the_same_kind_is_held(self):
        self.assertTrue(self.check()[0])
        ok, msgs = self.check()
        self.assertFalse(ok)
        self.assertTrue(any('(g)' in m for m in msgs))
        self.assertTrue(self.check(kind='original')[0])

    def test_missing_or_wrong_environment_variable(self):
        self.assertHeld(self.check(env={}), '(a)')
        self.assertHeld(self.check(env={'OR4_DESIGNATION_SHA256': 'x' * 64}), '(a)')

    def test_designation_changed_after_registration(self):
        self.designation.write_text(json.dumps({'gate_id': 'g', 'first_real_run_permitted': True, 'gate_sha256': 'whatever'}))
        self.assertHeld(self.check(), '(a)')

    def test_not_permitted_or_wrong_gate(self):
        self.write_designation(gate_id='other', first_real_run_permitted=False)
        self.assertHeld(self.check(), '(c) designation file lacks')

    def test_review_not_recorded(self):
        m = copy.deepcopy(self.manifest)
        m['review']['at'] = None
        self.assertHeld(self.check(manifest=m), '(b)')

    def test_review_before_the_registration_or_in_the_future_is_held(self):
        m = copy.deepcopy(self.manifest)
        m['review']['at'] = '2000-01-01T00:00:00+09:00'
        self.assertHeld(self.check(manifest=m), 'review.at is not after created_at')
        m['review']['at'] = '2999-01-01T00:00:00+09:00'
        self.assertHeld(self.check(manifest=m), 'review.at is not after created_at')
        m['review']['at'] = '2026-10-20T06:00:00'          # no timezone
        self.assertHeld(self.check(manifest=m), 'timezone-aware')

    def test_review_before_the_evidence_was_finalized_is_held(self):
        m = copy.deepcopy(self.manifest)
        m['review']['at'] = '2026-10-20T05:10:00+09:00'      # after created_at, before finalized_at (v5 review L2)
        self.assertHeld(self.check(manifest=m), 'finalized_at')

    def test_gate_file_changed(self):
        self.gate.write_text('gate text, edited after registration')
        self.assertHeld(self.check(), '(d)')

    def test_manifest_edited_after_the_designation(self):
        m = copy.deepcopy(self.manifest)
        m['pins']['node'] = 'changed'
        self.assertHeld(self.check(manifest=m, pins={'engine_binary': 'aa', 'node': 'changed'}), 'manifest_core_sha256')

    def test_noise_table_or_n_arm_changed_after_the_designation(self):
        self.assertHeld(self.check(inputs=dict(self.inputs, noise_table='zz')), 'noise_table')
        self.assertHeld(self.check(inputs=dict(self.inputs, n_arm_traces='zz')), 'n_arm_trace_list_sha256')
        self.assertHeld(self.check(inputs=dict(self.inputs, d_arm_traces='zz')), 'd_arm_trace_list_sha256')

    def test_pin_mismatch_unavailable_and_unregistered(self):
        self.assertHeld(self.check(pins={'engine_binary': 'zz', 'node': 'bb'}), '(e) pin engine_binary')
        self.assertHeld(self.check(pins={'engine_binary': 'aa', 'node': 'bb', 'extra': 'ee'}), 'not registered')

    def test_input_mismatch_and_unregistered(self):
        self.assertHeld(self.check(inputs=dict(self.inputs, **{'bundle:x': 'changed'})), '(f) input bundle:x')
        self.assertHeld(self.check(inputs=dict(self.inputs, **{'bundle:new': 'ff'})), 'bundle:new')

    def test_first_real_run_already_recorded(self):
        m = copy.deepcopy(self.manifest)
        m['first_real_run_at'] = '2026-10-20T07:00:00+09:00'
        self.assertHeld(self.check(manifest=m), 'first_real_run_at')

    def test_analysis_guard_needs_the_first_run_marker_and_creates_none(self):
        self.assertHeld(self.check(kind='analysis', create_marker=False, require_marker_of=('engine',)), 'has not started')
        self.assertTrue(self.check(kind='engine')[0])                           # the held first run creates its marker
        ok, msgs = self.check(kind='analysis', create_marker=False, require_marker_of=('engine',))
        self.assertTrue(ok, msgs)
        self.assertEqual(len(list(self.markers.iterdir())), 1)                  # the guard added nothing
        m = copy.deepcopy(self.manifest)
        m['pins']['node'] = 'changed'
        ok, msgs = self.check(manifest=m, pins={'engine_binary': 'aa', 'node': 'changed'}, kind='analysis', create_marker=False,
                              require_marker_of=('engine',))
        self.assertFalse(ok)                                                    # the analysis guard re-checks pins / core


if __name__ == '__main__':
    unittest.main()

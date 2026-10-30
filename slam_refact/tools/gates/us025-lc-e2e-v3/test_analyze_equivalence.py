#!/usr/bin/env python3
"""Synthetic unit tests of analyze_equivalence.py: python3 -B test_analyze_equivalence.py -v.

Every rejection and adoption path of the preregistered rules (G0-G9, section 5) is driven with hand-made dev-summary data and sampler lines. No real run, no file of the experiment."""
import contextlib
import copy
import datetime
import io
import importlib.util
import itertools
import json
import math
import pathlib
import random
import shutil
import subprocess
import sys
import tempfile
import unittest
import unittest.mock

import numpy as np

sys.dont_write_bytecode = True
HERE = pathlib.Path(__file__).resolve().parent
_spec = importlib.util.spec_from_file_location('analyze_equivalence', HERE / 'analyze_equivalence.py')
A = importlib.util.module_from_spec(_spec)
sys.modules['analyze_equivalence'] = A
_spec.loader.exec_module(A)
_spec2 = importlib.util.spec_from_file_location('sample_load', HERE / 'sample_load.py')
SL = importlib.util.module_from_spec(_spec2)
sys.modules['sample_load'] = SL
_spec2.loader.exec_module(SL)

BLOCK_ARMS = ['S', 'P2', 'P4', 'S', 'P4', 'P2', 'P4', 'P2', 'S', 'P2', 'S', 'P4', 'P2', 'P4', 'S', 'P4', 'S', 'P2']   # preregistration section 2.3
S3, S2 = 'flip6-s3-lowtex', 'flip6-s2'
BASE_SECONDS = {S3: 33.0, S2: 63.5}
FRAMES = {S3: 381, S2: 1037}
SUCCESS_IDX = {0, 3, 6}                  # 3 of every 8 runs: 9 of 24 per arm and scenario
HANG_IDX = {S3: {5}, S2: {1, 2, 5}}      # s3 3 of 24, s2 9 of 24
T0 = datetime.datetime(2026, 10, 29, 0, 0, 0, tzinfo=datetime.timezone.utc)


def iso(seconds):
    return (T0 + datetime.timedelta(seconds=seconds)).strftime('%Y-%m-%dT%H:%M:%S.000Z')


def make_summary(shift=None, mutate=None, per_run=None):
    """shift: {(arm, scenario): relative runner-seconds shift}; per_run: callable(run_dict, index_in_arm_scenario) that edits a run; mutate: callable(summary) editing the whole summary."""
    shift = shift or {}
    runs = []
    counters = {}
    for block, arm in enumerate(BLOCK_ARMS, start=1):
        for scn in (S3, S2):
            for k in range(4):
                key = (arm, scn)
                i = counters.get(key, 0)
                counters[key] = i + 1
                rnd = random.Random(f'{arm}|{scn}|{i}')
                hang = i % 8 in HANG_IDX[scn]
                success = (i % 8 in SUCCESS_IDX) and not hang
                seconds = 600.2 if hang else BASE_SECONDS[scn] * (1 + shift.get(key, 0.0)) * (1 + rnd.gauss(0, 0.004))
                nf = max(0, round(rnd.gauss(2 if scn == S3 else 40, 1.5 if scn == S3 else 10)))
                start = (block - 1) * 1000 + (0 if scn == S3 else 500) + k * 100
                run = {'scenario': scn, 'run': f'b{block:02d}-{arm.lower()}-lc1-{k + 1:02d}', 'block': block, 'arm': arm, 'lc': 1, 'slot': 0, 'cpus': None,
                       'startedAt': iso(start), 'endedAt': iso(start + seconds), 'runnerSeconds': round(seconds, 2), 'runnerExit': 0 if not hang else None,
                       'status': 'COMPLETED_WITH_ERRORS' if success else 'COMPLETED', 'infra': 'watchdog: the run exceeded 600 s' if hang else None,
                       'loadAvgAtStart': [1.0, 1.0, 1.0],
                       'metrics': {'inputFrames': FRAMES[scn], 'freshCallbacks': FRAMES[scn] - nf, 'noFreshCallback': nf},
                       'originalSuccess': True if success else (False if not hang else None),
                       'verdicts': {'on': 'PASS' if success else 'BLOCKED', 'off': 'PASS', 'full': None if success else 'PASS_DECLARED', 'null': None}}
                if hang:
                    run['metrics'] = None
                    run['verdicts'] = {'on': None, 'off': None, 'full': None, 'null': None}
                if per_run:
                    per_run(run, i)
                runs.append(run)
    summary = {'schema': 'slam-refact-us025-lc-e2e-v3-dev-summary-v1', 'infraStop': False, 'runs': runs, 'blocks': []}
    if mutate:
        mutate(summary)
    return summary


def sampler_line(t, hot=(), baseline=False):
    """One line of sample_load.py: `others` are the processes outside the gate tree (the recorder has already excluded the gate pids), cores = CPU cores used over the interval."""
    return {'t': iso(t), 'dt': None if baseline else 10.0, 'baseline': baseline, 'loadavg': '1.0 1.0 1.0 1/100 1', 'sys_busy_cores': None if baseline else 1.0, 'gate_pids': [10, 11],
            'others': [{'pid': pid, 'ppid': 1, 'comm': 'python3', 'cores': cores, 'cpu_s': 100.0, 'args': 'python3 busy.py'} for pid, cores in hot]}


def write_sampler(path, hot_samples=None, until=19000, skip=()):
    """hot_samples: {sample_second: [(pid, cores)...]}; a quiet 10 s grid for the whole experiment; skip: [(from_s, to_s)) windows without samples (gaps)."""
    hot_samples = hot_samples or {}
    with open(path, 'w', encoding='utf8') as f:
        for t in range(0, until, 10):
            if any(a <= t < b for a, b in skip):
                continue
            f.write(json.dumps(sampler_line(t, hot_samples.get(t, ()), baseline=(t == 0))) + '\n')
        f.write(json.dumps({'t': iso(until), 'end': True}) + '\n')


class Base(unittest.TestCase):
    def setUp(self):
        self.tmp = pathlib.Path(tempfile.mkdtemp(prefix='eq-analysis-'))
        self.addCleanup(shutil.rmtree, self.tmp, True)
        self.sampler = self.tmp / 'sampler.jsonl'
        write_sampler(self.sampler)

    def run_analysis(self, summary, sampler='default', rerun=None):
        return A.analyze(summary, self.sampler if sampler == 'default' else sampler, rerun)


class TestAdoption(Base):
    def test_equal_arms_adopt_four(self):
        r = self.run_analysis(make_summary())
        self.assertEqual((r['adopted_N'], r['status']), (4, 'ADOPT'), r['reasons'])
        self.assertTrue(r['g0']['pass'])
        self.assertTrue(r['g9']['pass'] and r['g9']['evaluated'] and not r['g9']['flagged_runs'])
        for parm in ('P2', 'P4'):
            c = r['comparisons'][parm]
            self.assertTrue(c['ratios']['g4']['pass'] and c['ratios']['g5']['pass'] and c['g6']['pass'], parm)
            for scn in (S3, S2):
                t = c['timing'][scn]
                self.assertTrue(t['g1']['pass'] and t['g2']['pass'] and t['g3']['pass'], (parm, scn, t))
        self.assertTrue(r['g8']['pass'])
        self.assertEqual(r['notices'], [])

    def test_p4_slow_p2_fine_adopts_two(self):
        r = self.run_analysis(make_summary(shift={('P4', S3): 0.05, ('P4', S2): 0.05}))
        self.assertEqual((r['adopted_N'], r['status']), (2, 'ADOPT'), r['reasons'])
        self.assertFalse(r['comparisons']['P4']['timing'][S3]['g1']['pass'])
        self.assertTrue(r['comparisons']['P2']['timing'][S3]['g1']['pass'])

    def test_p2_slow_p4_fine_is_g7_ambiguous(self):
        r = self.run_analysis(make_summary(shift={('P2', S3): 0.05, ('P2', S2): 0.05}))
        self.assertFalse(r['comparisons']['P2']['timing'][S3]['g1']['pass'])
        self.assertTrue(r['passed_g1_g6']['P4'] and not r['passed_g1_g6']['P2'])
        self.assertTrue(r['g7']['triggered'])
        self.assertEqual((r['adopted_N'], r['status']), (1, 'AMBIGUOUS'))
        self.assertTrue(any('G7' in x for x in r['reasons']))

    def test_both_slow_no_pass(self):
        r = self.run_analysis(make_summary(shift={(a, s): 0.05 for a in ('P2', 'P4') for s in (S3, S2)}))
        self.assertEqual((r['adopted_N'], r['status']), (1, 'NO_PASS'))

    def test_a_shift_just_inside_the_band_is_a_pass(self):
        r = self.run_analysis(make_summary(shift={(a, s): 0.015 for a in ('P2', 'P4') for s in (S3, S2)}))
        self.assertEqual(r['adopted_N'], 4, r['reasons'])
        ci = r['comparisons']['P2']['timing'][S3]['g1']['ci90']
        self.assertTrue(0.97 <= ci[0] <= ci[1] <= 1.03, ci)


class TestTimingGates(Base):
    def test_g1_shift_five_percent_fails(self):
        r = self.run_analysis(make_summary(shift={('P2', S3): 0.05}))
        g1 = r['comparisons']['P2']['timing'][S3]['g1']
        self.assertFalse(g1['pass'])
        self.assertGreater(g1['ci90'][0], 1.03)
        self.assertTrue(r['comparisons']['P2']['timing'][S2]['g1']['pass'])   # per scenario, both must hold
        self.assertFalse(r['passed_g1_g6']['P2'])

    def test_g1_speedup_also_fails(self):
        r = self.run_analysis(make_summary(shift={('P2', S2): -0.05}))
        self.assertFalse(r['comparisons']['P2']['timing'][S2]['g1']['pass'])   # the interval must be inside [0.97, 1.03] on both sides

    def test_g2_one_slow_run_fails_the_tail_not_the_median(self):
        def edit(run, i):
            if run['arm'] == 'P2' and run['scenario'] == S3 and i == 11:
                run['runnerSeconds'] = round(33.0 * 1.2, 2)
        r = self.run_analysis(make_summary(per_run=edit))
        t = r['comparisons']['P2']['timing'][S3]
        self.assertTrue(t['g1']['pass'])
        self.assertFalse(t['g2']['pass'])
        self.assertGreater(t['g2']['max_P'], t['g2']['max_limit'])

    def test_g2_p90_limit(self):
        # the 3 slowest of ~21 runs 8 % slower: p90 above 1.05 x p90(S) while the median is untouched
        def edit(run, i):
            if run['arm'] == 'P2' and run['scenario'] == S2 and i % 8 in (4, 7):
                run['runnerSeconds'] = round(run['runnerSeconds'] * 1.08, 2)
        t = self.run_analysis(make_summary(per_run=edit))['comparisons']['P2']['timing'][S2]
        self.assertFalse(t['g2']['pass'])
        self.assertGreater(t['g2']['p90_P'], t['g2']['p90_limit'])

    def test_g3_frame_count_must_equal_the_s_value(self):
        def edit(run, i):
            if run['arm'] == 'P4' and run['scenario'] == S3 and i == 2:
                run['metrics']['inputFrames'] = 380
        t = self.run_analysis(make_summary(per_run=edit))['comparisons']['P4']['timing'][S3]['g3']
        self.assertFalse(t['pass'])
        self.assertEqual(len(t['P_runs_with_other_frames']), 1)

    def test_g3_no_fresh_margin_per_scenario(self):
        def edit(run, i):
            if run['arm'] == 'P2' and run['scenario'] == S3 and run['status'] == 'COMPLETED' and run['infra'] is None:
                run['metrics']['noFreshCallback'] += 6          # s3 margin is +4 frames
            if run['arm'] == 'P4' and run['scenario'] == S2 and run['infra'] is None:
                run['metrics']['noFreshCallback'] += 12         # s2 margin is +21 frames: a mean +12 stays inside
        r = self.run_analysis(make_summary(per_run=edit))
        self.assertFalse(r['comparisons']['P2']['timing'][S3]['g3']['pass'])
        self.assertTrue(r['comparisons']['P4']['timing'][S2]['g3']['pass'])
        self.assertEqual(r['comparisons']['P2']['timing'][S3]['g3']['margin_upper'], 4.0)
        self.assertEqual(r['comparisons']['P4']['timing'][S2]['g3']['margin_upper'], 21.0)

    def test_g3_no_fresh_s2_over_margin(self):
        def edit(run, i):
            if run['arm'] == 'P4' and run['scenario'] == S2 and run['infra'] is None:
                run['metrics']['noFreshCallback'] += 30
        self.assertFalse(self.run_analysis(make_summary(per_run=edit))['comparisons']['P4']['timing'][S2]['g3']['pass'])

    def test_s_frames_not_unique_is_ambiguous(self):
        def edit(run, i):
            if run['arm'] == 'S' and run['scenario'] == S2 and i == 4:
                run['metrics']['inputFrames'] = 1036
        r = self.run_analysis(make_summary(per_run=edit))
        self.assertEqual((r['adopted_N'], r['status']), (1, 'AMBIGUOUS'))
        self.assertTrue(any('inputFrames' in x for x in r['reasons']))

    def test_hung_runs_are_not_in_the_timing_set(self):
        r = self.run_analysis(make_summary())
        self.assertEqual(r['arms']['S'][S3]['timing_set'], 21)
        self.assertEqual(r['arms']['S'][S2]['timing_set'], 15)
        self.assertLess(r['arms']['S'][S2]['runner_seconds']['max'], 65)   # the 600 s watchdog runs are INFRA, not timing


class TestRatioGates(Base):
    def test_g4_exact_test_boundary(self):
        # S 18/48 vs P 12/48: Farrington-Manning exact p = 0.0948 <= 0.10 (reject H0, non-inferior), P 11/48: p = 0.1389 (cannot reject)
        s = self.success_drop_more(make_summary(), 'P2', S3, 3)
        s = self.success_drop_more(s, 'P2', S2, 3)
        g4 = self.run_analysis(s)['comparisons']['P2']['ratios']['g4']
        self.assertEqual((g4['success_S'], g4['success_P']), (18, 12))
        self.assertAlmostEqual(g4['p_exact'], 0.0948, places=3)
        self.assertTrue(g4['rejects_h0'])
        self.assertTrue(g4['pass'])
        s = self.success_drop_more(make_summary(), 'P2', S3, 4)
        s = self.success_drop_more(s, 'P2', S2, 3)
        g4 = self.run_analysis(s)['comparisons']['P2']['ratios']['g4']
        self.assertEqual(g4['success_P'], 11)
        self.assertAlmostEqual(g4['p_exact'], 0.1389, places=3)
        self.assertFalse(g4['rejects_h0'])
        self.assertFalse(g4['point_guard_ok'])      # -7/48 = -0.146 is also below the guard -0.125
        self.assertFalse(g4['pass'])

    def success_drop_more(self, summary, arm, scn, n):
        done = 0
        for r in summary['runs']:
            if r['arm'] == arm and r['scenario'] == scn and r['originalSuccess'] is True and done < n:
                r['originalSuccess'] = False
                r['verdicts'] = {'on': 'BLOCKED', 'off': 'PASS', 'full': 'PASS_DECLARED', 'null': None}
                done += 1
        return summary

    def test_g4_collapse_fails_the_exact_test(self):
        s = make_summary()
        s = self.success_drop_more(s, 'P2', S3, 9)
        s = self.success_drop_more(s, 'P2', S2, 9)
        g4 = self.run_analysis(s)['comparisons']['P2']['ratios']['g4']
        self.assertEqual(g4['success_P'], 0)
        self.assertFalse(g4['rejects_h0'])
        self.assertFalse(g4['pass'])
        self.assertGreater(g4['p_exact'], A.ALPHA)

    def test_g4_point_guard_boundary_is_inclusive(self):
        # P - S = exactly -6/48 = -0.125 pooled and -0.25 in one scenario: the guards pass (inclusive), the result then depends on the exact test only
        s = make_summary()
        s = self.success_drop_more(s, 'P2', S3, 3)
        s = self.success_drop_more(s, 'P2', S2, 3)
        g4 = self.run_analysis(s)['comparisons']['P2']['ratios']['g4']
        self.assertEqual(g4['point_P_minus_S'], -0.125)
        self.assertTrue(g4['point_guard_ok'])
        self.assertTrue(all(g4['per_scenario_guard_ok'].values()))
        s2 = self.success_drop_more(make_summary(), 'P2', S3, 5)    # -5/24 = -0.208 in s3 (guard -0.30 ok), pooled -5/48
        g4b = self.run_analysis(s2)['comparisons']['P2']['ratios']['g4']
        self.assertTrue(g4b['per_scenario_guard_ok'][S3])
        s3 = self.success_drop_more(make_summary(), 'P2', S3, 8)    # -8/24 = -0.333 < -0.30
        g4c = self.run_analysis(s3)['comparisons']['P2']['ratios']['g4']
        self.assertFalse(g4c['per_scenario_guard_ok'][S3])
        self.assertFalse(g4c['pass'])

    def test_g4_small_drop_passes(self):
        s = self.success_drop_more(make_summary(), 'P4', S3, 2)
        g4 = self.run_analysis(s)['comparisons']['P4']['ratios']['g4']
        self.assertTrue(g4['pass'], g4)

    def hang_more(self, summary, arm, scn, n):
        done = 0
        for r in summary['runs']:
            if r['arm'] == arm and r['scenario'] == scn and r['infra'] is None and r['originalSuccess'] is not True and done < n:
                r.update({'infra': 'watchdog: the run exceeded 600 s', 'runnerSeconds': 600.2, 'metrics': None, 'originalSuccess': None, 'verdicts': {'on': None, 'off': None, 'full': None, 'null': None}})
                done += 1
        return summary

    def test_g5_pooled_point_guard(self):
        s = self.hang_more(make_summary(), 'P2', S3, 6)      # +6/24 = +0.25 in s3 (scenario guard inclusive), pooled +6/48 = +0.125 > +0.10
        g5 = self.run_analysis(s)['comparisons']['P2']['ratios']['g5']
        self.assertEqual(g5['point_P_minus_S'], 0.125)
        self.assertFalse(g5['point_guard_ok'])
        self.assertTrue(g5['per_scenario_guard_ok'][S3])
        self.assertFalse(g5['pass'])

    def test_g5_scenario_guard(self):
        s = self.hang_more(make_summary(), 'P4', S3, 7)      # +7/24 = 0.2917 > 0.25 in s3, pooled +7/48 = 0.146
        g5 = self.run_analysis(s)['comparisons']['P4']['ratios']['g5']
        self.assertFalse(g5['per_scenario_guard_ok'][S3])
        self.assertFalse(g5['pass'])

    def test_g5_exact_test_rejects_a_big_hang_rate(self):
        s = self.hang_more(make_summary(), 'P2', S3, 10)
        s = self.hang_more(s, 'P2', S2, 8)
        g5 = self.run_analysis(s)['comparisons']['P2']['ratios']['g5']
        self.assertFalse(g5['rejects_h0'])
        self.assertFalse(g5['pass'])

    def test_g5_exact_test_boundary(self):
        # hang 15/48 vs 12/48: exact p = 0.0704 (non-inferior), 16/48 vs 12/48: p = 0.1289 (cannot reject) although the point guard (+0.10) still allows +4/48 = 0.083
        g5 = self.run_analysis(self.hang_more(make_summary(), 'P4', S3, 3))['comparisons']['P4']['ratios']['g5']
        self.assertEqual(g5['hang_P'] - g5['hang_S'], 3)
        self.assertAlmostEqual(g5['p_exact'], 0.0704, places=3)
        self.assertTrue(g5['pass'], g5)
        g5 = self.run_analysis(self.hang_more(make_summary(), 'P4', S3, 4))['comparisons']['P4']['ratios']['g5']
        self.assertAlmostEqual(g5['p_exact'], 0.1289, places=3)
        self.assertTrue(g5['point_guard_ok'])
        self.assertFalse(g5['rejects_h0'])
        self.assertFalse(g5['pass'])


class TestVerdictGate(Base):
    def edit_success_verdict(self, arm, key, value):
        done = []

        def edit(run, i):
            if run['arm'] == arm and run['originalSuccess'] is True and not done:
                run['verdicts'][key] = value
                done.append(run['run'])
        return make_summary(per_run=edit)

    def test_one_fail_in_a_success_run_rejects_that_n(self):
        r = self.run_analysis(self.edit_success_verdict('P4', 'on', 'FAIL'))
        g6 = r['comparisons']['P4']['g6']
        self.assertFalse(g6['pass'])
        self.assertEqual(len(g6['on_off_fail_runs']), 1)
        self.assertFalse(r['passed_g1_g6']['P4'])
        # P2 is clean (N = 2); without the one P4 block that holds the FAIL run P4 would pass (N = 4), but D-V3-9 ignores a LARGER N in the leave-one-block-out: the main N = 2 stays
        self.assertEqual((r['adopted_N'], r['status']), (2, 'ADOPT'), r['reasons'])
        self.assertTrue(any(c['adopted_N'] == 4 for c in r['r1_leave_one_block_out']['cases']))
        self.assertEqual(r['r1_leave_one_block_out']['ambiguous'], [])

    def test_a_missing_verdict_or_failed_replay_counts_as_fail(self):
        r = self.run_analysis(self.edit_success_verdict('P2', 'off', None))
        self.assertFalse(r['comparisons']['P2']['g6']['pass'])

    def test_off_fail_also_rejects(self):
        self.assertFalse(self.run_analysis(self.edit_success_verdict('P2', 'off', 'FAIL'))['comparisons']['P2']['g6']['pass'])

    def test_pass_declared_is_fine(self):
        self.assertTrue(self.run_analysis(self.edit_success_verdict('P2', 'on', 'PASS_DECLARED'))['comparisons']['P2']['g6']['pass'])

    def test_tool_error_residual_is_ambiguous_and_the_rerun_file_resolves_it(self):
        s = self.edit_success_verdict('P2', 'on', 'TOOL_ERROR')
        r = self.run_analysis(s)
        self.assertEqual((r['adopted_N'], r['status']), (1, 'AMBIGUOUS'))
        self.assertEqual(len(r['comparisons']['P2']['g6']['residual_other_values']), 1)
        key = r['comparisons']['P2']['g6']['residual_other_values'][0].split(':')[0]
        r2 = self.run_analysis(s, rerun={key: {'on': 'PASS'}})
        self.assertEqual((r2['adopted_N'], r2['status']), (4, 'ADOPT'), r2['reasons'])
        self.assertEqual(r2['rerun_applied'][0]['before'], 'TOOL_ERROR')
        self.assertEqual(r2['rerun_applied'][0]['after'], 'PASS')
        r3 = self.run_analysis(s, rerun={key: {'on': 'TOOL_ERROR'}})   # still there after the one re-run
        self.assertEqual(r3['status'], 'AMBIGUOUS')

    def test_s_arm_fail_is_a_notice_not_a_change_of_n(self):
        r = self.run_analysis(self.edit_success_verdict('S', 'on', 'FAIL'))
        self.assertEqual(r['adopted_N'], 4)
        self.assertEqual(len(r['notices']), 1)
        self.assertIn('S ARM VERDICT FAIL', r['notices'][0])

    def full_fails(self, arm, n):
        done = []

        def edit(run, i):
            if run['arm'] == arm and run['infra'] is None and run['originalSuccess'] is not True and len(done) < n:
                run['verdicts']['full'] = 'FAIL'
                done.append(run['run'])
        return make_summary(per_run=edit)

    def test_full_verdict_fail_rate_fisher(self):
        r = self.run_analysis(self.full_fails('P2', 9))
        g6 = r['comparisons']['P2']['g6']
        self.assertEqual((g6['full_fail_P'], g6['full_den_P'], g6['full_fail_S'], g6['full_den_S']), (9, 18, 0, 18))
        self.assertLessEqual(g6['fisher_p_greater'], 0.10)
        self.assertTrue(g6['full_rejects'])
        self.assertFalse(g6['pass'])
        one = self.run_analysis(self.full_fails('P2', 1))['comparisons']['P2']['g6']
        self.assertFalse(one['full_rejects'])      # one FAIL of 18 vs 0 of 18: p = 0.5
        self.assertTrue(one['pass'])
        # p <= 0.10 is the boundary: 4 vs 0 of 18 gives p = 0.0575 (reject), 3 vs 0 gives 0.1176 (no reject)
        self.assertTrue(self.run_analysis(self.full_fails('P2', 4))['comparisons']['P2']['g6']['full_rejects'])
        self.assertFalse(self.run_analysis(self.full_fails('P2', 3))['comparisons']['P2']['g6']['full_rejects'])

    def test_full_verdict_a_higher_s_rate_does_not_reject_p(self):
        r = self.run_analysis(self.full_fails('S', 9))
        self.assertTrue(r['comparisons']['P2']['g6']['pass'])      # one-sided: only "P higher than S" counts


class TestValidity(Base):
    def test_infra_stop_is_incomplete(self):
        r = self.run_analysis(make_summary(mutate=lambda s: s.update({'infraStop': True})))
        self.assertEqual((r['adopted_N'], r['status']), (1, 'INCOMPLETE'))
        self.assertTrue(any('infraStop' in x for x in r['reasons']))

    def test_too_few_started_runs_is_incomplete(self):
        def edit(run, i):
            if run['arm'] == 'P4' and run['scenario'] == S2 and i >= 21:
                run.update({'infra': 'not started: resources or lock not available', 'startedAt': None, 'endedAt': None, 'runnerSeconds': None, 'metrics': None, 'originalSuccess': None})
        r = self.run_analysis(make_summary(per_run=edit))
        self.assertEqual(r['status'], 'INCOMPLETE')
        self.assertEqual(r['g0']['started_per_arm_scenario'][f'P4/{S2}'], 21)
        self.assertTrue(r['g0']['not_run'])

    def test_exactly_22_started_runs_is_still_complete(self):
        def edit(run, i):
            if run['arm'] == 'P4' and run['scenario'] == S3 and i >= 22:
                run.update({'infra': 'resources or lock not available within 14400 s', 'startedAt': iso(0), 'endedAt': iso(1), 'runnerSeconds': None, 'metrics': None, 'originalSuccess': None})
        r = self.run_analysis(make_summary(per_run=edit))
        self.assertTrue(r['g0']['pass'], r['g0'])
        self.assertEqual(r['g0']['started_per_arm_scenario'][f'P4/{S3}'], 22)

    def test_missing_arm_is_incomplete(self):
        s = make_summary()
        s['runs'] = [r for r in s['runs'] if r['arm'] != 'P4']
        self.assertEqual(self.run_analysis(s)['status'], 'INCOMPLETE')

    def test_missing_summary_file_is_incomplete_n1(self):
        out = self.tmp / 'a.json'
        with contextlib.redirect_stdout(io.StringIO()):
            rc = A.main(['--summary', str(self.tmp / 'nope.json'), '--out', str(out)])
        self.assertEqual(rc, 0)
        res = json.loads(out.read_text())
        self.assertEqual((res['adopted_N'], res['status']), (1, 'INCOMPLETE'))
        self.assertIn('does not exist', res['reasons'][0])

    def test_g8_baseline_drift_is_ambiguous(self):
        def edit(run, i):
            if run['arm'] == 'S' and run['scenario'] == S3 and run['block'] in A.G8_LATE_BLOCKS:
                run['runnerSeconds'] = round(run['runnerSeconds'] * 1.04, 2)
        r = self.run_analysis(make_summary(per_run=edit))
        self.assertEqual((r['adopted_N'], r['status']), (1, 'AMBIGUOUS'))
        self.assertFalse(r['g8']['scenarios'][S3]['pass'])
        self.assertGreater(r['g8']['scenarios'][S3]['ratio'], 1.02)

    def test_g8_uses_the_registered_blocks(self):
        r = self.run_analysis(make_summary())
        self.assertEqual(r['g8']['early_blocks'], [1, 4, 9])
        self.assertEqual(r['g8']['late_blocks'], [11, 15, 17])
        self.assertEqual([BLOCK_ARMS[b - 1] for b in A.G8_EARLY_BLOCKS + A.G8_LATE_BLOCKS], ['S'] * 6)

    def test_g8_an_empty_group_is_ambiguous(self):
        s = make_summary()
        s['runs'] = [r for r in s['runs'] if not (r['arm'] == 'S' and r['scenario'] == S2 and r['block'] in A.G8_LATE_BLOCKS)]
        self.assertEqual(self.run_analysis(s)['status'], 'INCOMPLETE')    # fewer than 22 S/s2 runs also fails G0 first
        # an empty late group with enough runs is not possible with 4 runs per block: construct it through hangs
        s = make_summary()
        for r in s['runs']:
            if r['arm'] == 'S' and r['scenario'] == S2 and r['block'] in A.G8_LATE_BLOCKS:
                r.update({'infra': 'watchdog: the run exceeded 600 s', 'runnerSeconds': 600.2, 'metrics': None, 'originalSuccess': None})
        res = self.run_analysis(s)
        self.assertEqual(res['status'], 'AMBIGUOUS')
        self.assertIn('empty', ' '.join(res['reasons']))


class TestDisturbance(Base):
    def run_window(self, summary, name):
        r = next(x for x in summary['runs'] if x['run'] == name and x['scenario'] == S3)
        return (datetime.datetime.fromisoformat(r['startedAt'].replace('Z', '+00:00')) - T0).total_seconds()

    def test_three_consecutive_hot_samples_flag_a_run(self):
        s = make_summary()
        t = self.run_window(s, 'b01-s-lc1-01') + 5
        sampler = self.tmp / 'hot.jsonl'
        write_sampler(sampler, {int(t // 10) * 10 + k * 10: [(999, 1.5)] for k in range(3)})
        r = self.run_analysis(s, sampler)
        self.assertIn(f'{S3}/b01-s-lc1-01', r['g9']['flagged_runs'])
        self.assertEqual(r['g9']['per_arm']['S']['flagged'], 1)

    def test_two_samples_a_gap_and_a_cooler_process_do_not_flag(self):
        s = make_summary()
        t = int(self.run_window(s, 'b01-s-lc1-01') // 10) * 10 + 10
        hot = {t: [(999, 1.5)], t + 10: [(999, 1.5)], t + 30: [(999, 1.5)]}                # 2 consecutive, then a missing sample: the streak restarts
        hot[t + 70] = [(998, 0.99)]                                                         # under 1.0 core
        hot[t + 80] = [(998, 0.99)]
        hot[t + 90] = [(998, 0.99)]
        sampler = self.tmp / 'cool.jsonl'
        write_sampler(sampler, hot)
        r = self.run_analysis(s, sampler)
        self.assertEqual(r['g9']['flagged_runs'], [])
        self.assertEqual(r['adopted_N'], 4)

    def test_different_pids_do_not_make_a_streak(self):
        s = make_summary()
        t = int(self.run_window(s, 'b01-s-lc1-01') // 10) * 10 + 10
        sampler = self.tmp / 'moving.jsonl'
        write_sampler(sampler, {t: [(901, 2.0)], t + 10: [(902, 2.0)], t + 20: [(903, 2.0)], t + 30: [(904, 2.0)]})
        self.assertEqual(self.run_analysis(s, sampler)['g9']['flagged_runs'], [])

    def hot_for_arm(self, summary, arm, fraction_runs):
        runs = [r for r in summary['runs'] if r['arm'] == arm]
        hot = {}
        for r in runs[:fraction_runs]:
            t0 = int((datetime.datetime.fromisoformat(r['startedAt'].replace('Z', '+00:00')) - T0).total_seconds() // 10) * 10 + 10
            for k in range(3):
                hot.setdefault(t0 + 10 * k, []).append((777, 1.8))
        sampler = self.tmp / f'hot-{arm}-{fraction_runs}.jsonl'
        write_sampler(sampler, hot)
        return sampler

    def test_more_than_15_percent_marked_runs_in_an_arm_is_ambiguous(self):
        s = make_summary()
        # 48 runs per arm: 8 marked = 16.7 % > 15 %. Distinct runs sit in different blocks; the marks are placed per run start
        sampler = self.hot_for_arm(s, 'P2', 8)
        r = self.run_analysis(s, sampler)
        self.assertGreater(r['g9']['per_arm']['P2']['fraction'], 0.15)
        self.assertEqual((r['adopted_N'], r['status']), (1, 'AMBIGUOUS'))
        self.assertTrue(any('G9' in x for x in r['reasons']))

    def test_seven_marked_runs_of_48_are_under_the_limit(self):
        s = make_summary()
        sampler = self.hot_for_arm(s, 'P2', 7)
        r = self.run_analysis(s, sampler)
        self.assertLessEqual(r['g9']['per_arm']['P2']['fraction'], 0.15)
        self.assertEqual(r['adopted_N'], 4)       # the re-analysis without the marked runs adopts the same N

    def test_sensitivity_that_changes_n_is_ambiguous(self):
        # P4 looks slow only because of runs that were disturbed: without them P4 passes, with them it does not -> the main analysis says N = 2, the reduced one N = 4
        s = make_summary()
        marked = [r for r in s['runs'] if r['arm'] == 'P4' and r['scenario'] == S3 and r['infra'] is None][:6]
        for r in marked:
            r['runnerSeconds'] = round(r['runnerSeconds'] * 1.12, 2)
            r['endedAt'] = iso((datetime.datetime.fromisoformat(r['startedAt'].replace('Z', '+00:00')) - T0).total_seconds() + r['runnerSeconds'])
        hot = {}
        for r in marked:
            t0 = int((datetime.datetime.fromisoformat(r['startedAt'].replace('Z', '+00:00')) - T0).total_seconds() // 10) * 10 + 10
            for k in range(3):
                hot.setdefault(t0 + 10 * k, []).append((555, 2.0))
        sampler = self.tmp / 'sens.jsonl'
        write_sampler(sampler, hot)
        r = self.run_analysis(s, sampler)
        self.assertTrue({f'{S3}/{m["run"]}' for m in marked} <= set(r['g9']['flagged_runs']))
        self.assertLessEqual(r['g9']['per_arm']['P4']['fraction'], 0.15)    # a 600 s watchdog run overlaps its neighbours in time and may be marked as well: still under the 15 % limit
        self.assertEqual(r['g9']['sensitivity']['adopted_N'], 4)
        self.assertTrue(r['g9']['differs_from_main'])
        self.assertEqual((r['adopted_N'], r['status']), (1, 'AMBIGUOUS'))

    def start_s(self, run):
        return (datetime.datetime.fromisoformat(run['startedAt'].replace('Z', '+00:00')) - T0).total_seconds()

    def window_runs(self, summary, a):
        """The runs whose [start, end] overlaps the sample-free window (a - 15, a + 55) that skip=[(a - 5, a + 45)] leaves."""
        return [r for r in summary['runs'] if r['startedAt'] and self.start_s(r) < a + 55 and self.start_s(r) + (r['runnerSeconds'] or 0) > a - 15]

    def isolated_runs(self, summary, n):
        """n normal P2 s3 runs whose gap window touches no other run (a 600 s watchdog run of a neighbouring block overlaps others in time)."""
        out = [r for r in summary['runs'] if r['arm'] == 'P2' and r['scenario'] == S3 and r['infra'] is None and len(self.window_runs(summary, self.start_s(r))) == 1]
        self.assertGreaterEqual(len(out), n)
        return out[:n]

    def test_a_gap_longer_than_30_s_makes_the_overlapping_run_unobserved(self):
        s = make_summary()
        run = self.isolated_runs(s, 1)[0]
        a = self.start_s(run)
        sampler = self.tmp / 'gap.jsonl'
        write_sampler(sampler, skip=[(a - 5, a + 45)])
        r = self.run_analysis(s, sampler)
        self.assertEqual(r['g9']['unobserved_runs'], [f"{S3}/{run['run']}"])
        self.assertEqual(r['g9']['flagged_runs'], [])
        self.assertEqual(r['g9']['per_arm']['P2']['unobserved'], 1)
        self.assertEqual(len(r['g9']['gaps']), 1)
        self.assertEqual(r['adopted_N'], 4)       # 1 of 48 is under the limit

    def test_a_gap_of_exactly_30_s_is_not_a_gap(self):
        s = make_summary()
        run = self.isolated_runs(s, 1)[0]
        a = self.start_s(run)
        sampler = self.tmp / 'nogap.jsonl'
        write_sampler(sampler, skip=[(a + 1, a + 21)])         # samples at ...,a-0?,a+0, a+30: two missing samples = 30 s between neighbours
        r = self.run_analysis(s, sampler)
        self.assertEqual(r['g9']['unobserved_runs'], [])

    def test_more_than_15_percent_unobserved_runs_in_an_arm_is_ambiguous(self):
        s = make_summary()
        runs = [r for r in s['runs'] if r['arm'] == 'P2' and r['scenario'] == S3 and r['infra'] is None][:8]
        sampler = self.tmp / 'gaps8.jsonl'
        write_sampler(sampler, skip=[(self.start_s(r) - 5, self.start_s(r) + 45) for r in runs])
        r = self.run_analysis(s, sampler)
        self.assertGreaterEqual(r['g9']['per_arm']['P2']['unobserved'], 8)       # 600 s watchdog runs of neighbouring blocks overlap in time and may be unobserved as well
        self.assertGreater(r['g9']['per_arm']['P2']['fraction'], 0.15)
        self.assertEqual((r['adopted_N'], r['status']), (1, 'AMBIGUOUS'))
        self.assertTrue(any('no observation' in x for x in r['reasons']))

    def test_a_run_outside_the_recorded_time_is_unobserved(self):
        s = make_summary()
        sampler = self.tmp / 'late.jsonl'
        write_sampler(sampler, until=5000)                    # the recorder stopped after block 5: later runs are not seen
        r = self.run_analysis(s, sampler)
        self.assertGreater(len(r['g9']['unobserved_runs']), 48)
        self.assertEqual((r['adopted_N'], r['status']), (1, 'AMBIGUOUS'))

    def test_error_and_end_lines_are_not_samples(self):
        sampler = self.tmp / 'mixed.jsonl'
        write_sampler(sampler, until=100)
        with open(sampler, 'a', encoding='utf8') as f:
            f.write(json.dumps({'t': iso(105), 'error': 'OSError: x'}) + '\n')
        samples, end = A.load_sampler(sampler)
        self.assertEqual(len(samples), 10)
        self.assertIsNotNone(end)

    def test_no_sampler_is_ambiguous(self):
        r = self.run_analysis(make_summary(), sampler=None)
        self.assertEqual((r['adopted_N'], r['status']), (1, 'AMBIGUOUS'))
        self.assertFalse(r['g9']['evaluated'])


class TestLeaveOneBlockOut(Base):
    def p2_successes(self, keep):
        """P2 keeps the original success only for (scenario, index in arm) in `keep`; the other runs become ordinary non-success runs."""
        def edit(run, i):
            if run['arm'] == 'P2' and run['originalSuccess'] is True and (run['scenario'], i) not in keep:
                run.update({'originalSuccess': False, 'status': 'COMPLETED'})
                run['verdicts'] = {'on': 'BLOCKED', 'off': 'PASS', 'full': 'PASS_DECLARED', 'null': None}
        return make_summary(shift={('P4', S3): 0.05, ('P4', S2): 0.05}, per_run=edit)

    def test_equal_arms_every_block_can_go_and_n_stays_four(self):
        r = self.run_analysis(make_summary())
        lobo = r['r1_leave_one_block_out']
        self.assertTrue(lobo['evaluated'])
        self.assertEqual(len(lobo['cases']), 12)                       # 6 blocks of P2 and 6 blocks of P4
        self.assertEqual({c['left_out'].split('/')[0] for c in lobo['cases']}, {'P2', 'P4'})
        self.assertTrue(all(c['adopted_N'] == 4 and c['runs_removed'] == 8 for c in lobo['cases']), lobo['cases'])
        self.assertEqual((r['adopted_N'], r['status']), (4, 'ADOPT'))

    def test_a_pass_that_depends_on_one_block_is_ambiguous(self):
        # P2 has 12 of 48 successes (exact p 0.0948: passes G4), all of them in blocks that carry 4 each; without one of those blocks 8 of 40 (p 0.22) fails: the pass was not stable
        keep = {(scn, i) for scn in (S3, S2) for i in (0, 3, 8, 11, 16, 19)}
        s = self.p2_successes(keep)
        r = self.run_analysis(s)
        self.assertEqual(r['passed_g1_g6'], {'P2': True, 'P4': False})
        self.assertEqual(r['comparisons']['P2']['ratios']['g4']['success_P'], 12)
        self.assertTrue(r['r1_leave_one_block_out']['evaluated'])
        self.assertTrue(any(c['adopted_N'] != 2 for c in r['r1_leave_one_block_out']['cases']))
        self.assertEqual((r['adopted_N'], r['status']), (1, 'AMBIGUOUS'))
        self.assertTrue(any(x.startswith('R-1') for x in r['reasons']))

    def test_the_leave_out_does_not_change_a_stable_pass(self):
        r = self.run_analysis(self.p2_successes({(scn, i) for scn in (S3, S2) for i in (0, 3, 6, 8, 11, 14, 16, 19, 22)}))   # P2 keeps its 18 of 48: the same as S
        self.assertEqual((r['adopted_N'], r['status']), (2, 'ADOPT'), r['reasons'])
        self.assertTrue(all(c['adopted_N'] == 2 for c in r['r1_leave_one_block_out']['cases']))

    def test_n_one_needs_no_leave_out(self):
        r = self.run_analysis(make_summary(shift={(a, s): 0.05 for a in ('P2', 'P4') for s in (S3, S2)}))
        self.assertEqual(r['adopted_N'], 1)
        self.assertFalse(r['r1_leave_one_block_out']['evaluated'])

    def test_the_leave_out_loop_with_a_stubbed_evaluation(self):
        runs = [r for r in make_summary()['runs'] if r['lc'] == 1]
        main = {'adopted_N': 4}
        seen = []

        def fake(rs):
            seen.append(len(rs))
            has_p4_b3 = any(r['arm'] == 'P4' and r['block'] == 3 for r in rs)
            return {'adopted_N': 4 if has_p4_b3 else 2, 'status': 'ADOPT', 'passed': {}}
        with unittest.mock.patch.object(A, 'evaluate', fake):
            out = A.gate_lobo(runs, main)
        self.assertEqual(len(seen), 12)
        self.assertEqual(out['ambiguous'], ['R-1: without P4 block 3 the adopted N is 2 (ADOPT), smaller than 4'])
        with unittest.mock.patch.object(A, 'evaluate', lambda rs: {'adopted_N': 4, 'status': 'ADOPT', 'passed': {}}):
            self.assertEqual(A.gate_lobo(runs, {'adopted_N': 2})['ambiguous'], [])      # D-V3-9: a larger N in every leave-out is not ambiguous


class TestNullSuccess(Base):
    def null_run(self, summary_arm='P2'):
        done = []

        def edit(run, i):
            if run['arm'] == summary_arm and run['originalSuccess'] is True and not done:
                run['originalSuccess'] = None
                run['verdicts'] = {'on': 'TOOL_ERROR', 'off': 'PASS', 'full': None, 'null': None}
                done.append(run['run'])
        return make_summary(per_run=edit), done

    def test_a_run_without_original_success_but_with_an_on_verdict_is_ambiguous(self):
        s, done = self.null_run()
        r = self.run_analysis(s)
        self.assertEqual(len(r['r5_null_success']), 1)
        self.assertEqual((r['adopted_N'], r['status']), (1, 'AMBIGUOUS'))
        self.assertTrue(any(x.startswith('R-5') for x in r['reasons']))

    def test_the_rerun_of_the_on_comparison_resolves_it(self):
        s, done = self.null_run()
        key = f'{S3}/{done[0]}' if any(x['run'] == done[0] and x['scenario'] == S3 for x in s['runs']) else f'{S2}/{done[0]}'
        r = self.run_analysis(s, rerun={key: {'on': 'PASS', 'originalSuccess': True}})
        self.assertEqual(r['r5_null_success'], [])
        self.assertEqual((r['adopted_N'], r['status']), (4, 'ADOPT'), r['reasons'])
        self.assertEqual({x['arm'] for x in r['rerun_applied']}, {'on', 'originalSuccess'})

    def test_an_infra_run_and_a_run_without_on_verdict_are_not_r5(self):
        r = self.run_analysis(make_summary())
        self.assertEqual(r['r5_null_success'], [])          # the hung runs have originalSuccess null and no verdicts: not an R-5 case


class TestSampler(unittest.TestCase):
    """sample_load.py against a fake /proc."""

    def setUp(self):
        self.proc = pathlib.Path(tempfile.mkdtemp(prefix='fake-proc-'))
        self.addCleanup(shutil.rmtree, self.proc, True)
        self.ticks = {}

    def put_proc(self, pid, ppid, comm, args, ticks, start=1000):
        d = self.proc / str(pid)
        d.mkdir(exist_ok=True)
        rest = ['S', str(ppid)] + ['0'] * 9 + [str(ticks), '0'] + ['0'] * 6 + [str(start)] + ['0'] * 5      # field 3 = state, 4 = ppid, 14 = utime, 15 = stime, 22 = starttime
        (d / 'stat').write_text(f'{pid} ({comm}) ' + ' '.join(rest) + '\n')
        (d / 'cmdline').write_bytes(args.replace(' ', '\0').encode())

    def put_sys(self, busy=0, idle=1000):
        (self.proc / 'stat').write_text(f'cpu  {busy} 0 0 {idle} 0 0 0 0 0 0\ncpu0 {busy} 0 0 {idle} 0 0 0 0 0 0\n')
        (self.proc / 'loadavg').write_text('0.50 0.40 0.30 2/300 12345\n')

    def test_stat_with_spaces_and_parentheses_in_the_command_name(self):
        self.put_proc(77, 5, 'we (ird) name', 'x', 321)
        self.assertEqual(SL.read_stat(self.proc / '77' / 'stat'), (5, 'we (ird) name', 321, 1000))

    def test_the_gate_tree_is_the_dev_process_and_all_its_descendants_without_the_recorder(self):
        self.put_proc(100, 1, 'node', 'node /x/gate_lc.mjs dev --scenarios a', 0)
        self.put_proc(101, 100, 'taskset', 'taskset -c 0-7,16-23 node run_lcgold.mjs', 0)
        self.put_proc(102, 101, 'chrome', 'chrome --headless', 0)
        self.put_proc(200, 1, 'python3', 'python3 other.py', 0)
        self.put_proc(300, 1, 'python3', 'python3 sample_load.py --out gate_lc.mjs dev', 0)    # the recorder's own command line mentions the gate: never counted
        self.assertEqual(SL.gate_tree(SL.read_procs(self.proc), 300), {100, 101, 102})

    def test_cores_are_a_delta_after_the_exclusion_of_the_gate_tree_and_before_the_cut(self):
        self.put_sys(0, 0)
        self.put_proc(100, 1, 'node', 'node /x/gate_lc.mjs dev', 0)
        self.put_proc(101, 100, 'chrome', 'chrome', 0)
        for pid, comm in ((201, 'a'), (202, 'b'), (203, 'c')):
            self.put_proc(pid, 1, comm, comm, 0)
        now = [0.0]
        smp = SL.Sampler(str(self.proc), top=2, clock=lambda: now[0], own_pid=999)
        first = smp.sample()
        self.assertTrue(first['baseline'] and first['others'] == [] and first['gate_pids'] == [100, 101])
        now[0] = 10.0
        t = SL.TICKS
        self.put_sys(busy=int(5 * 10 * t), idle=int(27 * 10 * t))
        self.put_proc(101, 100, 'chrome', 'chrome', 8 * 10 * t)         # the gate's browser: 8 cores, the busiest of all, must not appear
        self.put_proc(201, 1, 'a', 'a', 3 * 10 * t)                      # 3 cores
        self.put_proc(202, 1, 'b', 'b', 1 * 10 * t)                      # 1 core
        self.put_proc(203, 1, 'c', 'c', 2 * 10 * t)                      # 2 cores
        second = smp.sample()
        self.assertEqual([(o['pid'], o['cores']) for o in second['others']], [(201, 3.0), (203, 2.0)])    # top 2 of the non-gate processes
        self.assertEqual(second['dt'], 10.0)
        self.assertFalse(second['baseline'])
        self.assertEqual(second['gate_pids'], [100, 101])
        self.assertEqual(second['loadavg'], '0.50 0.40 0.30 2/300 12345')
        self.assertIsNotNone(second['sys_busy_cores'])

    def test_a_process_with_a_new_start_time_is_a_new_process_not_a_delta_of_the_old_pid(self):
        self.put_sys()
        self.put_proc(201, 1, 'a', 'a', 5000, start=1000)
        now = [0.0]
        smp = SL.Sampler(str(self.proc), clock=lambda: now[0], own_pid=999)
        smp.sample()
        now[0] = 10.0
        self.put_proc(201, 1, 'a', 'a', 50, start=2000)                  # pid reused: 0.5 s of CPU over 10 s, not a negative delta
        out = smp.sample()
        self.assertEqual([(o['pid'], o['cores']) for o in out['others']], [(201, 0.05)])

    def test_the_command_writes_baseline_samples_and_an_end_line_that_the_analysis_reads(self):
        self.put_sys()
        self.put_proc(201, 1, 'a', 'a', 10)
        out = self.proc / 'out.jsonl'
        self.assertEqual(SL.main(['--out', str(out), '--interval', '0.1', '--max-seconds', '0.3', '--proc', str(self.proc)]), 0)
        lines = [json.loads(x) for x in out.read_text().splitlines()]
        self.assertTrue(lines[0]['baseline'])
        self.assertTrue(lines[-1].get('end'))
        self.assertGreaterEqual(len(lines), 4)
        samples, end = A.load_sampler(out)
        self.assertEqual(len(samples), len(lines) - 1)
        self.assertIsNotNone(end)

    def test_recorder_lines_feed_the_disturbance_rule(self):
        self.put_sys()
        self.put_proc(201, 1, 'busy', 'busy', 0)
        stamps = [iso(t) for t in (100, 110, 120, 130, 140, 150)]
        now = [0.0]
        smp = SL.Sampler(str(self.proc), clock=lambda: now[0], own_pid=999)
        lines = []
        with unittest.mock.patch.object(SL, 'iso_now', side_effect=stamps):
            for k in range(6):
                now[0] = 10.0 * k
                self.put_proc(201, 1, 'busy', 'busy', int(1.5 * 10 * k * SL.TICKS))      # 1.5 cores all the time
                lines.append(smp.sample())
        path = self.proc / 'rec.jsonl'
        path.write_text(''.join(json.dumps(x) + '\n' for x in lines))
        samples, _ = A.load_sampler(path)
        run = {'scenario': S3, 'run': 'r1', 'startedAt': iso(105), 'endedAt': iso(145)}
        self.assertEqual(A.disturbed_runs([run], samples), [f'{S3}/r1'])
        self.assertEqual(A.disturbed_runs([{**run, 'run': 'r2', 'endedAt': iso(125)}], samples), [])        # only 2 samples (110, 120) inside


class TestStatistics(unittest.TestCase):
    def test_closed_form_restricted_mle_agrees_with_the_numeric_maximum(self):
        for n, d0 in [(24, -0.25), (48, -0.25), (48, -0.20), (30, -0.20)]:
            closed = A.fm_z_table(n, n, d0)
            numeric = A.fm_z_table(n, n, d0, numeric_only=True)
            self.assertFalse(np.isnan(closed).any())
            self.assertLess(float(np.abs(closed - numeric).max()), 1e-5, (n, d0))

    def test_exact_p_matches_an_independent_enumeration(self):
        old = A.GRID_STEP
        A.GRID_STEP = 0.05
        try:
            nt, nr, delta = 6, 7, 0.25
            xt_obs, xr_obs = 4, 3
            p, _, z_obs = A.fm_exact_noninferiority(xt_obs, nt, xr_obs, nr, delta)
            d0 = -delta

            def z(xt, xr):
                pt, pr = A._fm_restricted_numeric(xt, nt, xr, nr, d0)
                return (xt / nt - xr / nr - d0) / math.sqrt(pt * (1 - pt) / nt + pr * (1 - pr) / nr)
            zobs = z(xt_obs, xr_obs)
            self.assertAlmostEqual(zobs, z_obs, places=5)
            best = 0.0
            for k in range(int(round((1 - delta) / 0.05)) + 1):
                pt = min(k * 0.05, 1 - delta)
                pr = pt + delta
                tot = 0.0
                for xt, xr in itertools.product(range(nt + 1), range(nr + 1)):
                    if z(xt, xr) >= zobs - 1e-7:
                        tot += math.comb(nt, xt) * pt ** xt * (1 - pt) ** (nt - xt) * math.comb(nr, xr) * pr ** xr * (1 - pr) ** (nr - xr)
                best = max(best, tot)
            self.assertAlmostEqual(p, best, places=6)
        finally:
            A.GRID_STEP = old

    def test_exact_p_values_are_ordered_and_sensible(self):
        p_equal = A.fm_exact_noninferiority(18, 48, 18, 48, 0.25)[0]
        p_worse = A.fm_exact_noninferiority(12, 48, 18, 48, 0.25)[0]
        p_much_worse = A.fm_exact_noninferiority(6, 48, 18, 48, 0.25)[0]
        p_better = A.fm_exact_noninferiority(24, 48, 18, 48, 0.25)[0]
        self.assertLess(p_better, p_equal)
        self.assertLess(p_equal, p_worse)
        self.assertLess(p_worse, p_much_worse)
        self.assertLessEqual(p_equal, A.ALPHA)           # equal arms pass at 18/48 each (preregistration section 6: about 0.88 of the time)
        self.assertGreater(p_much_worse, A.ALPHA)
        for p in (p_equal, p_worse, p_much_worse, p_better):
            self.assertTrue(0.0 <= p <= 1.0)

    def test_exact_test_edge_tables(self):
        self.assertEqual(A.fm_exact_noninferiority(0, 48, 0, 48, 0.25)[0] <= 1.0, True)
        p_all = A.fm_exact_noninferiority(48, 48, 48, 48, 0.20)[0]
        self.assertLessEqual(p_all, A.ALPHA)          # everyone fine in both arms: non-inferior
        self.assertGreater(A.fm_exact_noninferiority(0, 48, 12, 48, 0.25)[0], A.ALPHA)

    def test_seeded_bootstrap_is_reproducible(self):
        s_runs = [{'runnerSeconds': 33 + 0.1 * i, 'metrics': {'noFreshCallback': i % 3}} for i in range(20)]
        p_runs = [{'runnerSeconds': 33.2 + 0.1 * i, 'metrics': {'noFreshCallback': i % 4}} for i in range(20)]
        a = A.bootstrap_pair(s_runs, p_runs)
        b = A.bootstrap_pair(s_runs, p_runs)
        self.assertEqual(a, b)
        self.assertEqual(A.SEED, 20261028)
        self.assertEqual(A.BOOTSTRAP, 10000)
        self.assertLess(a[0][0], a[0][1])


class TestOutput(Base):
    def test_whole_analysis_is_deterministic(self):
        s = make_summary(shift={('P2', S3): 0.01})
        a = json.dumps(self.run_analysis(s), sort_keys=True, default=str)
        b = json.dumps(self.run_analysis(copy.deepcopy(s)), sort_keys=True, default=str)
        self.assertEqual(a, b)

    def test_ks_is_reported_but_never_decides(self):
        r = self.run_analysis(make_summary())
        self.assertIn('p_exact', r['ks_report_only']['P2'][S3])
        self.assertEqual(r['adopted_N'], 4)

    def test_cli_writes_the_file_once_with_isolated_python(self):
        s = self.tmp / 'dev-summary.json'
        s.write_text(json.dumps(make_summary()))
        out = self.tmp / 'equivalence-analysis.json'
        cmd = [sys.executable, '-I', '-B', str(HERE / 'analyze_equivalence.py'), '--summary', str(s), '--sampler', str(self.sampler), '--out', str(out)]
        r = subprocess.run(cmd, capture_output=True, text=True)
        self.assertEqual(r.returncode, 0, r.stderr)
        res = json.loads(out.read_text())
        self.assertEqual((res['adopted_N'], res['status']), (4, 'ADOPT'))
        self.assertEqual(res['schema'], A.SCHEMA)
        self.assertEqual(len(res['inputs']['summary_sha256']), 64)
        self.assertEqual(json.loads(r.stdout)['adopted_N'], 4)
        again = subprocess.run(cmd, capture_output=True, text=True)
        self.assertEqual(again.returncode, 2)
        self.assertIn('never overwritten', again.stderr)

    def test_the_summary_of_the_real_dev_command_shape_is_read(self):
        # a run record in the form written by `gate_lc.mjs dev` (copied from the smoke dev-summary of 2026-10-28, one scenario, one arm): the script runs and says INCOMPLETE
        run = {'scenario': 'flip6-n1', 'run': 'b01-p2-lc1-01', 'block': 1, 'arm': 'P2', 'lc': 1, 'slot': 0, 'cpus': '8-19', 'startedAt': '2026-10-28T05:41:56.213Z', 'endedAt': '2026-10-28T05:43:03.133Z',
               'runnerSeconds': 66.92, 'runnerExit': 0, 'status': 'COMPLETED_WITH_ERRORS', 'infra': None, 'watchdog': None, 'loadAvgAtStart': [6.47, 7.54, 6.64],
               'startCheck': {'ok': True, 'loadavg': [6.86, 7.63, 6.66]}, 'metrics': {'status': 'COMPLETED_WITH_ERRORS', 'wallSeconds': 64.862, 'inputFrames': 720, 'publishedFrames': 720, 'freshCallbacks': 681, 'noFreshCallback': 39},
               'originalSuccess': True, 'originalAttempts': 86, 'verdicts': {'on': 'PASS', 'off': 'PASS', 'full': None, 'null': None}, 'replaySeconds': {'off': 108.6, 'on': 111.92}}
        summary = {'schema': 'slam-refact-us025-lc-e2e-v3-dev-summary-v1', 'infraStop': False, 'scenarios': ['flip6-n1'], 'cpuRange': [8, 31], 'runs': [run]}
        res = A.analyze(summary, self.sampler)
        self.assertEqual((res['adopted_N'], res['status']), (1, 'INCOMPLETE'))
        self.assertEqual(res['arms']['P2'][S3]['lc1_runs'], 0)   # other scenario names are not part of the experiment


if __name__ == '__main__':
    unittest.main()

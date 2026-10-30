#!/usr/bin/env python3
"""Regression tests of the v4 fixes in the Python tools (FIXES.md maps every defect id to the test names): finding #2 (R-6), #12 (re-run targets), G-18 (original_run_info.py), G-20, G-25.

    python3 -B -m unittest test_v4_fixes -v                        (run from this directory)

The analysis and the static-manifest tool are loaded from US025_LC_TOOL_DIR (default: this directory). Pointing it at ../us025-lc-e2e-v3 runs the SAME tests against the v3 code: that is how
"fails before the fix" was observed (FIXES.md, "baseline"). No real run, no file of any experiment."""
import importlib.util
import json
import os
import pathlib
import shutil
import sys
import tempfile
import unittest

sys.dont_write_bytecode = True
HERE = pathlib.Path(__file__).resolve().parent
TOOL_DIR = (HERE / os.environ.get('US025_LC_TOOL_DIR', '.')).resolve()


def load(name, directory, file):
    spec = importlib.util.spec_from_file_location(name, directory / file)
    mod = importlib.util.module_from_spec(spec)
    sys.modules[name] = mod
    spec.loader.exec_module(mod)
    return mod


T = load('test_analyze_equivalence_fixture', HERE, 'test_analyze_equivalence.py')   # fixtures only (make_summary, write_sampler); its own analyzer is the v4 one
A = load('analyze_equivalence_under_test', TOOL_DIR, 'analyze_equivalence.py')
S3, S2 = T.S3, T.S2


class Base(unittest.TestCase):
    def setUp(self):
        self.tmp = pathlib.Path(tempfile.mkdtemp(prefix='v4-fixes-'))
        self.addCleanup(shutil.rmtree, self.tmp, True)
        self.sampler = self.tmp / 'sampler.jsonl'
        T.write_sampler(self.sampler)

    def analyze(self, summary, rerun=None):
        return A.analyze(summary, self.sampler, rerun)


class TestFinding2(Base):
    """#2: a non-INFRA run whose ON replay exited non-zero arrives without any ON verdict and without originalSuccess."""

    def edit_first(self, arm, make):
        done = []

        def edit(run, i):
            if run['arm'] == arm and run['originalSuccess'] is False and not done:
                make(run)
                done.append(f"{run['scenario']}/{run['run']}")
        return T.make_summary(per_run=edit), done

    def test_2_non_success_run_with_failed_on_replay_is_ambiguous_not_a_silent_non_fail(self):
        def crash(run):
            run['originalSuccess'] = None
            run['verdicts'] = {'on': None, 'off': None, 'full': None, 'null': None}
            run['replayExits'] = {'on': 2, 'off': 0}
        s, done = self.edit_first('P2', crash)
        r = self.analyze(s)
        self.assertEqual((r['adopted_N'], r['status']), (1, 'AMBIGUOUS'), r['reasons'])
        self.assertTrue(any('R-6' in x for x in r['reasons']), r['reasons'])
        self.assertEqual(len(r['r6_replay_without_verdict']), 1)
        self.assertIn('replay exits', r['r6_replay_without_verdict'][0])

    def test_2_an_old_summary_without_replay_exit_codes_is_also_ambiguous(self):
        def crash(run):
            run['originalSuccess'] = None
            run['verdicts'] = {'on': None, 'off': None, 'full': None, 'null': None}
        s, done = self.edit_first('P2', crash)
        r = self.analyze(s)
        self.assertEqual(r['status'], 'AMBIGUOUS')
        self.assertIn('no ON verdict', r['r6_replay_without_verdict'][0])

    def test_2_companion_a_success_run_with_failed_on_replay_is_the_registered_fail_of_g6(self):
        """With the G-18 fix the summary has originalSuccess true for such a run: the registered rule (replay exit != 0 = FAIL) applies and rejects the P arm."""
        done = []

        def edit(run, i):
            if run['arm'] == 'P2' and run['originalSuccess'] is True and not done:
                run['verdicts'] = {'on': None, 'off': 'PASS', 'full': None, 'null': None}
                run['replayExits'] = {'on': 2, 'off': 0}
                done.append(1)
        r = self.analyze(T.make_summary(per_run=edit))
        self.assertFalse(r['comparisons']['P2']['g6']['pass'])
        self.assertEqual(r['r6_replay_without_verdict'], [])


class TestFinding12(Base):
    """#12: the re-run file may only touch the targets of preregistration section 8.1."""

    def success_run(self, arm, verdicts):
        done = []

        def edit(run, i):
            if run['arm'] == arm and run['originalSuccess'] is True and not done:
                run['verdicts'] = dict(run['verdicts'], **verdicts)
                done.append(f"{run['scenario']}/{run['run']}")
        return T.make_summary(per_run=edit), done[0] if done else None

    def test_12_a_registered_fail_cannot_be_turned_into_pass_by_the_rerun_file(self):
        s, _ = self.success_run('P2', {'on': 'FAIL'})
        key = next(f"{x['scenario']}/{x['run']}" for x in s['runs'] if x['arm'] == 'P2' and x['verdicts']['on'] == 'FAIL')
        r = self.analyze(s, rerun={key: {'on': 'PASS'}})
        self.assertFalse(r['comparisons']['P2']['g6']['pass'], 'the FAIL stays')
        self.assertEqual(r['rerun_applied'], [])
        self.assertEqual(r['rerun_rejected'][0]['key'], 'on')
        self.assertTrue(any('RERUN ENTRY REJECTED' in n for n in r['notices']))

    def test_12_full_is_never_a_rerun_target_and_a_non_success_run_is_none(self):
        s = T.make_summary()
        non_success = next(f"{x['scenario']}/{x['run']}" for x in s['runs'] if x['arm'] == 'P2' and x['originalSuccess'] is False)
        success = next(f"{x['scenario']}/{x['run']}" for x in s['runs'] if x['arm'] == 'P2' and x['originalSuccess'] is True)
        r = self.analyze(s, rerun={success: {'full': 'PASS_DECLARED'}, non_success: {'on': 'PASS', 'originalSuccess': True}})
        self.assertEqual(r['rerun_applied'], [])
        self.assertEqual({(x['run'], x['key']) for x in r['rerun_rejected']}, {(success, 'full'), (non_success, 'on'), (non_success, 'originalSuccess')})

    def test_12_an_infra_run_and_an_unknown_run_are_no_targets_and_originalSuccess_is_not_overwritten(self):
        s = T.make_summary()
        infra = next(f"{x['scenario']}/{x['run']}" for x in s['runs'] if x['infra'] is not None)
        success = next(f"{x['scenario']}/{x['run']}" for x in s['runs'] if x['arm'] == 'P2' and x['originalSuccess'] is True)
        r = self.analyze(s, rerun={infra: {'originalSuccess': True}, 'nope/none': {'on': 'PASS'}, success: {'originalSuccess': False}})
        self.assertEqual(r['rerun_applied'], [])
        self.assertEqual(len(r['rerun_rejected']), 3)

    def test_12_companion_the_two_registered_rerun_cases_are_still_accepted(self):
        s, _ = self.success_run('P2', {'on': 'TOOL_ERROR'})
        key = next(f"{x['scenario']}/{x['run']}" for x in s['runs'] if x['arm'] == 'P2' and x['verdicts']['on'] == 'TOOL_ERROR')
        r = self.analyze(s, rerun={key: {'on': 'PASS'}})
        self.assertEqual((r['adopted_N'], r['status']), (4, 'ADOPT'), r['reasons'])
        self.assertEqual(r['rerun_rejected'], [])


class TestG20(unittest.TestCase):
    def test_g20_the_end_line_does_not_make_a_stretch_after_the_last_sample_observed(self):
        t0 = T.T0.timestamp()
        samples = [{'t': t0, 'hot': {}}, {'t': t0 + 10, 'hot': {}}]
        run = {'scenario': S3, 'run': 'r1', 'startedAt': T.iso(500), 'endedAt': T.iso(600)}
        end = t0 + 1000      # the recorder wrote only `error` lines after its last sample and an `end` line much later
        self.assertEqual(A.unobserved_runs([run], samples, end), [f'{S3}/r1'])

    def test_g20_companion_a_run_that_ends_within_the_gap_limit_after_the_last_sample_is_observed(self):
        t0 = T.T0.timestamp()
        samples = [{'t': t0, 'hot': {}}, {'t': t0 + 10, 'hot': {}}]
        run = {'scenario': S3, 'run': 'r1', 'startedAt': T.iso(1), 'endedAt': T.iso(30)}
        self.assertEqual(A.unobserved_runs([run], samples, t0 + 40), [])


class TestG25(unittest.TestCase):
    def setUp(self):
        self.mls = load('make_lc_static_under_test', TOOL_DIR, 'make_lc_static.py')
        self.tmp = pathlib.Path(tempfile.mkdtemp(prefix='v4-g25-'))
        self.addCleanup(shutil.rmtree, self.tmp, True)

    def test_g25_an_empty_full_null_directory_is_not_a_passed_measurement(self):
        nulls = [{'verdict': 'PASS'}]
        self.assertEqual(self.mls.null_run_result(nulls, self.tmp), 'FAIL')
        self.assertEqual(self.mls.null_run_result(nulls, self.tmp / 'missing'), 'FAIL')

    def test_g25_companion_a_directory_with_passing_files_passes_and_one_failing_file_fails(self):
        nulls = [{'verdict': 'PASS'}]
        (self.tmp / 'a.json').write_text(json.dumps({'verdict': 'PASS_DECLARED'}))
        self.assertEqual(self.mls.null_run_result(nulls, self.tmp), 'PASS')
        (self.tmp / 'b.json').write_text(json.dumps({'verdict': 'FAIL'}))
        self.assertEqual(self.mls.null_run_result(nulls, self.tmp), 'FAIL')
        self.assertEqual(self.mls.null_run_result([], self.tmp), 'FAIL')


class TestOriginalRunInfo(unittest.TestCase):
    """G-18: original_run_info.py (v4 only)."""

    def setUp(self):
        self.ori = load('original_run_info_under_test', HERE, 'original_run_info.py')
        self.tmp = pathlib.Path(tempfile.mkdtemp(prefix='v4-ori-'))
        self.addCleanup(shutil.rmtree, self.tmp, True)

    def test_g18_success_comes_from_the_original_run_data_alone(self):
        loader = lambda d, expect: {'firstSuccess': {'frame': 7}, 'attempts': [1, 2, 3], 'hooked': False, 'restaged': {}, 'meta': {'loadAvgAtStart': [1, 1, 1], 'status': 'COMPLETED'}}   # noqa: E731
        res = self.ori.original_info('x', loader=loader)
        self.assertEqual(res['originalRun']['success'], True)
        self.assertEqual((res['originalRun']['attempts'], res['originalRun']['firstSuccessFrame'], res['error']), (3, 7, None))
        loader2 = lambda d, expect: {'firstSuccess': None, 'attempts': [], 'hooked': False, 'meta': {}}   # noqa: E731
        self.assertEqual(self.ori.original_info('x', loader=loader2)['originalRun']['success'], False)

    def test_g18_an_unreadable_original_is_recorded_as_an_error_not_raised(self):
        def bad(d, expect):
            raise ValueError('poke keys {}')
        res = self.ori.original_info('x', loader=bad)
        self.assertIsNone(res['originalRun'])
        self.assertIn('ValueError', res['error'])

    def test_g18_the_cli_writes_a_new_file_and_never_overwrites(self):
        out = self.tmp / 'o.json'
        self.assertEqual(self.ori.main(['--original', str(self.tmp / 'no-such-run'), '--out', str(out)]), 0)
        self.assertIn('error', json.loads(out.read_text()))
        self.assertEqual(self.ori.main(['--original', str(self.tmp), '--out', str(out)]), 2)


if __name__ == '__main__':
    unittest.main()

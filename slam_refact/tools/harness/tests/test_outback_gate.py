"""Tests for the gate evaluation tools in ../outback: speed_eval, gate_xr, run_timing."""
import contextlib
import copy
import io
import json
import pathlib
import sys
import tempfile
import unittest

HARNESS = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(HARNESS))
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

from outback import gate_xr, palindrome_bundle, run_timing, speed_eval  # noqa: E402
from test_outback import OutbackBase  # noqa: E402


def write_run(root, name, rows, meta=None):
    d = pathlib.Path(root) / name
    d.mkdir(parents=True, exist_ok=True)
    (d / 'trace.jsonl').write_text('\n'.join(json.dumps(r) for r in rows) + '\n')
    if meta is not None:
        (d / 'meta.json').write_text(json.dumps(meta))
    return d


def rows_from_x(xs, **extra):
    return [{'i': i, 'p': None if x is None else [x, 0.0, 0.0], **extra} for i, x in enumerate(xs)]


class TmpCase(unittest.TestCase):
    def setUp(self):
        tmp = tempfile.TemporaryDirectory()
        self.addCleanup(tmp.cleanup)
        self.tmp = pathlib.Path(tmp.name)


class SpeedErrorTest(TmpCase):
    def run_pair(self, run_xs, ref_xs, k, first_row=20):
        run = write_run(self.tmp, 'run', rows_from_x(run_xs))
        ref = write_run(self.tmp, 'ref', rows_from_x(ref_xs))
        return speed_eval.speed_error(run, ref, k, first_row)

    def test_identical_trajectory_at_k2_is_zero(self):
        ref = [float(i) for i in range(100)]
        run = [ref[2 * i] for i in range(50)]
        m = self.run_pair(run, ref, 2)
        self.assertEqual(m['eSpeed'], 0.0)
        self.assertEqual(m['pairs'], 30)
        self.assertTrue(m['complete'])

    def test_constant_offset(self):
        ref = [float(i) for i in range(100)]
        run = [ref[2 * i] + 0.25 for i in range(50)]
        m = self.run_pair(run, ref, 2)
        self.assertAlmostEqual(m['eSpeed'], 0.25)
        self.assertAlmostEqual(m['eSpeedP90'], 0.25)

    def test_index_mapping_is_k_times_i(self):
        # The reference is a ramp; the run shows reference row k*i exactly. A mapping of i or k*i+1
        # would give a distance of (k-1)*i or 1 and not 0.
        ref = [float(i) for i in range(120)]
        for k in (2, 3):
            run = [ref[k * i] for i in range(40)]
            self.assertEqual(self.run_pair(run, ref, k)['eSpeed'], 0.0)
            shifted = [ref[k * i + 1] for i in range(40)]
            self.assertAlmostEqual(self.run_pair(shifted, ref, k)['eSpeed'], 1.0)

    def test_first_row_respected(self):
        ref = [float(i) for i in range(100)]
        run = [ref[2 * i] for i in range(50)]
        run[19] += 100.0  # wrong, but before row 20
        self.assertEqual(self.run_pair(run, ref, 2)['eSpeed'], 0.0)
        for i in range(20, 30):
            run[i] += 100.0
        self.assertEqual(self.run_pair(run, ref, 2, first_row=20)['eSpeedP90'], 100.0)
        # With first_row 30 the bad rows 20..29 do not count.
        m = self.run_pair(run, ref, 2, first_row=30)
        self.assertEqual(m['eSpeedP90'], 0.0)
        self.assertEqual(m['expectedPairs'], 20)

    def test_incomplete_pairs_flagged(self):
        ref = [float(i) for i in range(100)]
        run = [ref[2 * i] for i in range(50)]
        run[30] = None  # no pose
        m = self.run_pair(run, ref, 2)
        self.assertFalse(m['complete'])
        self.assertEqual((m['pairs'], m['expectedPairs']), (29, 30))
        # reference too short for the last rows
        m = self.run_pair([ref[2 * i] for i in range(50)], ref[:60], 2)
        self.assertFalse(m['complete'])

    def test_no_pairs_gives_none(self):
        m = self.run_pair([0.0] * 10, [0.0] * 10, 2)
        self.assertIsNone(m['eSpeed'])
        self.assertEqual(m['expectedPairs'], 0)

    def test_cli_table_and_json(self):
        ref = [float(i) for i in range(100)]
        self.run_pair([ref[2 * i] for i in range(50)], ref, 2)
        out = self.tmp / 'o.json'
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf):
            speed_eval.main([f'{self.tmp / "run"}={self.tmp / "ref"}:2', '--json', str(out)])
        self.assertEqual(json.loads(out.read_text())[0]['eSpeed'], 0.0)
        with self.assertRaises(SystemExit):
            with contextlib.redirect_stderr(io.StringIO()):
                speed_eval.main([f'{self.tmp / "run"}={self.tmp / "ref"}:2', '--json', str(out)])


class PercentileTest(unittest.TestCase):
    def test_linear_interpolation(self):
        self.assertAlmostEqual(gate_xr.percentile_linear(list(range(1, 21)), 0.95), 19.05)
        self.assertAlmostEqual(gate_xr.percentile_linear([3, 1, 2], 0.95), 2.9)
        self.assertEqual(gate_xr.percentile_linear([5.0], 0.95), 5.0)
        self.assertEqual(gate_xr.percentile_linear([1, 2, 3, 4, 5], 0.5), 3)
        self.assertIsNone(gate_xr.percentile_linear([], 0.95))


class IdentityTest(TmpCase):
    def make(self, b_rows=None, b_meta=None):
        rows = [{'i': i, 'p': [float(i), 0, 0], 'srStageMs': {'mapTracker': 1.0}} for i in range(5)]
        meta = {'bundle': 'x', 'contractVersion': 11, 'wallSeconds': 1.0, 'rows': 5}
        write_run(self.tmp / 'A', 'r1', rows, meta)
        b = copy.deepcopy(rows) if b_rows is None else b_rows
        for r in b:
            r['srStageMs'] = {'mapTracker': 99.0}  # timing may differ
        write_run(self.tmp / 'B', 'r1', b, {**meta, 'contractVersion': 12, 'wallSeconds': 7.0} if b_meta is None else b_meta)

    def run_identity(self):
        out = self.tmp / 'id.json'
        with contextlib.redirect_stdout(io.StringIO()):
            code = gate_xr.main(['identity', '--e0-runs', str(self.tmp / 'A'), '--runs', str(self.tmp / 'B'),
                                 '--out', str(out)])
        return code, json.loads(out.read_text())

    def test_only_stage_and_ignored_meta_differ(self):
        self.make()
        code, res = self.run_identity()
        self.assertEqual(code, 0)
        self.assertTrue(res['pass'])
        self.assertEqual(res['results']['r1']['differingRows'], 0)

    def test_changed_position_fails_and_reports_row(self):
        rows = [{'i': i, 'p': [float(i), 0, 0]} for i in range(5)]
        rows[3]['p'] = [3.0, 0, 0.001]
        self.make(rows)
        code, res = self.run_identity()
        self.assertEqual(code, 1)
        self.assertFalse(res['pass'])
        self.assertEqual(res['results']['r1']['firstDifferingRow'], 3)
        self.assertEqual(res['results']['r1']['differingRows'], 1)

    def test_other_meta_key_fails(self):
        self.make(b_meta={'bundle': 'x', 'contractVersion': 12, 'wallSeconds': 7.0, 'rows': 6})
        code, res = self.run_identity()
        self.assertEqual(code, 1)
        self.assertEqual(res['results']['r1']['metaDiffKeys'], ['rows'])

    def test_row_count_difference_fails(self):
        self.make([{'i': i, 'p': [float(i), 0, 0]} for i in range(4)])
        code, res = self.run_identity()
        self.assertEqual(code, 1)
        self.assertEqual(res['results']['r1']['firstDifferingRow'], 4)

    def test_missing_run_fails(self):
        self.make()
        write_run(self.tmp / 'A', 'r2', [{'i': 0, 'p': None}], {'bundle': 'x'})
        code, res = self.run_identity()
        self.assertEqual(code, 1)
        self.assertTrue(res['results']['r2']['missing'])

    def test_refuses_to_overwrite(self):
        self.make()
        self.run_identity()
        with self.assertRaises(SystemExit):
            self.run_identity()


# ---------------------------------------------------------------- verdict

ENVS = {'OFF': '', 'K1': 'enabled=1,k1=1', 'K2': 'k2=1', 'K1+K2': 'enabled=1,k1=1,k2=1'}
CAND_BIN, E0_BIN = 'cand-bin-sha', 'e0-bin-sha'


def make_prereg(tmp=None):
    """Synthetic prereg; with tmp, the bundle directories it points at are created there too."""
    prereg = {
        'e0': {'replayBinarySha256': E0_BIN},
        'configurations': {'OFF': ENVS['OFF'], 'K1': ENVS['K1']},
        'sequences': {
            'outback': ['ob-a'], 'primary': ['ob-a'],
            'forwardSpeed': {'fw-a': {'reference': 'fx-a', 'k': 2}, 'fw-b': {'reference': 'fx-a', 'k': 3}},
            'k2Benefit': ['fw-a', 'fw-b'],
            'fixtures': {'fx-a': 'scn'}, 'chromeForward': ['fx-c'],
            'timing': {'ob-a': {'envelope': 'fx-a'}, 'fx-a': {}},
        },
        'bundleOf': {'fx-a': 'scn/a', 'fx-c': 'scn/c'},
        'limits': {'ratio': 1.05, 'absFloor': 0.02, 'primaryRatio': 0.5, 'coverageLossMax': 0.01,
                   'timingRatio': 1.10, 'k2BenefitRatio': 0.75, 'k2BenefitMinSequences': 2,
                   'eStartPairs': 21, 'speedFirstRow': 20, 'timingFirstRow': 30, 'timingRepetitions': 3,
                   'mapLostFloorFrames': 5, 'pathMinRatio': 0.9, 'rotFloorDeg': 1.0,
                   'lostFrameP95EnvelopeMs': 100.0, 'maxConfigurations': 4},
    }
    if tmp is not None:
        prereg['bundles'] = str(tmp / 'bundles')
        prereg['sourceBundles'] = str(tmp / 'src-bundles')
        for d in (tmp / 'bundles' / 'ob-a', tmp / 'bundles' / 'fw-a', tmp / 'bundles' / 'fw-b',
                  tmp / 'src-bundles' / 'scn' / 'a', tmp / 'src-bundles' / 'scn' / 'c'):
            d.mkdir(parents=True)
            for f in ('sensors.tsv', 'frames.tsv', 'engine-config.tsv'):
                (d / f).write_text(f'{d.name} {f}\n')
    return prereg


def base_run(kind, **kw):
    r = {'kind': kind, 'missing': False, 'exit': 0, 'stall': 0, 'rows': 100, 'resets': 2, 'discards': 1,
         'normalRatio': 0.9, 'identicalToE0': False, 'mapLost': 10}
    r.update(kw)
    return r


FIX_ACC = {'sim3AteRmse': 0.2, 'apeRotDegRmse': 4.0, 'rpe1sTransSim3Rmse': 0.1, 'rpe1sRotDegRmse': 2.0,
           'coverage': 0.95}


def make_metrics(config='E0'):
    return {'schema': 'xr-commercial-metrics-v1', 'config': config, 'accuracyEnv': ENVS.get(config),
            'replaySha256': E0_BIN if config == 'E0' else CAND_BIN, 'runs': {
        'ob-a': base_run('outback', eStart=1.0, eAll=1.0, eStartComplete=True, path=10.0, rotStartDeg=5.0),
        'fw-a': base_run('forwardSpeed', eSpeed=1.0, complete=True),
        'fw-b': base_run('forwardSpeed', eSpeed=2.0, complete=True),
        'fx-a': base_run('fixtures', accuracy=dict(FIX_ACC), longestNonNormalRun=4),
        'fx-c': base_run('chromeForward', longestNonNormalRun=4),
    }}


def make_timing(ob_off=10.0, ob_on=10.0, env_off=20.0, fx_off=10.0, fx_on=10.0, lost_on=0.0):
    def arm(m, lost=0.0):
        return {'values': [m] * 3, 'median': m, 'lostP95Median': lost}
    return {'replaySha256': CAND_BIN, 'accuracyOff': '', 'accuracyOn': ENVS['K1'],
            'badRuns': [], 'unknownExit': [],
            'sequences': {'ob-a': {'off': arm(ob_off), 'on': arm(ob_on, lost_on)},
                          'fx-a': {'off': arm(fx_off), 'on': arm(fx_on, lost_on)}}}


G0_KEYS = ('identity', 'rs0Fast', 'refactNative', 'unitTests')


class VerdictBase(TmpCase):
    def setUp(self):
        super().setUp()
        self.n = 0
        self.prereg = make_prereg(self.tmp)
        self.e0 = make_metrics()
        self.cand = make_metrics('K1')
        # A candidate that passes everything except it must beat G1: eStart exactly half of E0.
        self.cand['runs']['ob-a']['eStart'] = 0.5
        self.evidence = {}
        for k in G0_KEYS:
            f = self.tmp / f'evidence-{k}.txt'
            f.write_text(f'evidence of {k}\n')
            self.evidence[k] = f
        self.g0 = {k: {'pass': True, 'evidence': str(self.evidence[k]),
                       'sha256': gate_xr.sha256_file(self.evidence[k])} for k in G0_KEYS}
        self.g0['identity']['replaySha256'] = CAND_BIN
        self.set_identity_evidence()
        self.timing = make_timing()
        self.timing_on_override = None  # (value,): timing.accuracyOn instead of the configuration's env
        self.compare = None
        self.addenda = [{'schema': 'x', 'configurations': {'K2': ENVS['K2'], 'K1+K2': ENVS['K1+K2']}}]
        self.attempts = ['OFF', 'K1', 'K2', 'K1+K2']
        self.env_override = None  # accuracyEnv written into the candidate metrics instead of the right one
        self.after_freeze = None  # callable run after freeze.json was written, before the verdict
        self.tool = self.tmp / 'tool.py'
        self.tool.write_text('# a frozen tool\n')

    def set_identity_evidence(self, **kw):
        """(Re)write the identity evidence file (JSON) and the sha256 g0 records for it."""
        body = {'pass': True, 'replaySha256': CAND_BIN, **kw}
        self.evidence['identity'].write_text(json.dumps(body))
        self.g0['identity']['sha256'] = gate_xr.sha256_file(self.evidence['identity'])

    def verdict(self, config='K1'):
        self.n += 1
        cand = copy.deepcopy(self.cand)
        cand['config'] = config
        cand['accuracyEnv'] = ENVS[config] if self.env_override is None else self.env_override
        timing = copy.deepcopy(self.timing)
        if timing is not None:
            timing['accuracyOn'] = ENVS[config] if self.timing_on_override is None else self.timing_on_override[0]
        paths = {}
        for key, obj in (('prereg', self.prereg), ('e0', self.e0), ('metrics', cand), ('g0', self.g0),
                         ('timing', timing), ('compare', self.compare)):
            if obj is not None:
                p = self.tmp / f'{key}{self.n}.json'
                p.write_text(json.dumps(obj))
                paths[key] = str(p)
        add_args = []
        for i, a in enumerate(self.addenda):
            ap = self.tmp / f'addendum{self.n}-{i}.json'
            ap.write_text(json.dumps(a))
            add_args += ['--addendum', str(ap)]
        att = self.tmp / f'attempts{self.n}.jsonl'
        att.write_text('\n'.join(json.dumps({'config': c}) for c in self.attempts) + '\n')
        freeze = self.tmp / f'freeze{self.n}.json'
        with contextlib.redirect_stdout(io.StringIO()):
            gate_xr.main(['freeze', '--prereg', paths['prereg'], '--e0', paths['e0'], '--tools', str(self.tool),
                          '--out', str(freeze), *add_args])
        if self.after_freeze:
            self.after_freeze(freeze)
        argv = ['verdict', '--prereg', paths['prereg'], '--e0', paths['e0'], '--metrics', paths['metrics'],
                '--config', config, '--g0', paths['g0'], '--freeze', str(freeze), '--attempts', str(att),
                '--out', str(self.tmp / f'v{self.n}.json')]
        for opt in ('timing', 'compare'):
            if opt in paths:
                argv += [f'--{opt}', paths[opt]]
        with contextlib.redirect_stdout(io.StringIO()):
            gate_xr.main(argv)
        return json.loads((self.tmp / f'v{self.n}.json').read_text())

    def status(self, gate, config='K1'):
        return self.verdict(config)['gates'][gate]['status']

    def check(self, gate, sequence, metric, config='K1'):
        """The recorded check dict of a gate for (sequence, metric)."""
        checks = self.verdict(config)['gates'][gate]['checks']
        found = [c for c in checks if c['sequence'] == sequence and c['metric'] == metric]
        self.assertEqual(len(found), 1, (sequence, metric, [c['metric'] for c in checks]))
        return found[0]


class VerdictOverallTest(VerdictBase):
    def test_all_pass_is_adopt(self):
        v = self.verdict()
        self.assertEqual({g: x['status'] for g, x in v['gates'].items()},
                         {'BINDING': 'PASS', 'G0': 'PASS', 'G1': 'PASS', 'G2': 'PASS', 'G3': 'PASS', 'G4': 'PASS',
                          'G5': 'PASS', 'G6': 'NOT_APPLICABLE'})
        self.assertEqual(v['overall'], 'ADOPT')
        self.assertEqual(v['schema'], 'xr-commercial-verdict-v1')
        for key in ('preregSha256', 'e0Sha256', 'metricsSha256', 'timingSha256', 'freezeSha256',
                    'attemptsSha256'):
            self.assertEqual(len(v[key]), 64)

    def test_fail_gives_reject(self):
        self.cand['runs']['ob-a']['eStart'] = 0.6
        self.assertEqual(self.verdict()['overall'], 'REJECT')

    def test_fail_plus_blocked_gate_is_reject(self):
        self.cand['runs']['ob-a']['eStart'] = 0.6  # G1 FAIL
        self.timing = None                         # G5 BLOCKED: not measured
        del self.g0['rs0Fast']                     # G0 BLOCKED too
        v = self.verdict()
        self.assertEqual(v['gates']['G1']['status'], 'FAIL')
        self.assertEqual(v['gates']['G5']['status'], 'BLOCKED')
        self.assertEqual(v['gates']['BINDING']['status'], 'PASS')
        self.assertEqual(v['overall'], 'REJECT')

    def test_binding_blocked_beats_fail(self):
        self.cand['runs']['ob-a']['eStart'] = 0.6  # G1 FAIL
        self.env_override = 'wrong'                # BINDING blocked
        v = self.verdict()
        self.assertEqual(v['gates']['G1']['status'], 'FAIL')
        self.assertEqual(v['gates']['BINDING']['status'], 'BLOCKED')
        self.assertEqual(v['overall'], 'BLOCKED')

    def test_blocked_without_fail_is_blocked(self):
        self.timing = None
        v = self.verdict()
        self.assertEqual(v['overall'], 'BLOCKED')

    def test_blocked_within_a_gate_beats_fail(self):
        self.cand['runs']['fw-a']['eSpeed'] = 99.0  # FAIL
        self.cand['runs']['fw-b']['eSpeed'] = None  # BLOCKED
        self.assertEqual(self.status('G3'), 'BLOCKED')

    def test_limits_come_from_the_prereg(self):
        self.assertEqual(self.status('G1'), 'PASS')  # eStart 0.5 == 0.5 * 1.0
        self.prereg['limits']['primaryRatio'] = 0.4
        self.assertEqual(self.status('G1'), 'FAIL')
        self.prereg['limits']['primaryRatio'] = 0.5
        self.cand['runs']['fw-a']['eSpeed'] = 1.04
        self.assertEqual(self.status('G3'), 'PASS')
        self.prereg['limits']['ratio'] = 1.01
        self.prereg['limits']['absFloor'] = 0.0
        self.assertEqual(self.status('G3'), 'FAIL')

    def test_cli_refuses_overwrite(self):
        self.verdict()
        self.n -= 1
        with self.assertRaises(SystemExit):
            self.verdict()


class VerdictG0Test(VerdictBase):
    def test_false_entry_fails(self):
        for key in G0_KEYS:
            self.setUp()
            self.g0[key]['pass'] = False
            self.assertEqual(self.status('G0'), 'FAIL', key)

    def test_missing_key_blocks(self):
        for key in G0_KEYS:
            self.setUp()
            del self.g0[key]
            self.assertEqual(self.status('G0'), 'BLOCKED', key)

    def test_evidence_must_exist_and_match_its_sha(self):
        for key in G0_KEYS:
            self.setUp()
            self.g0[key]['evidence'] = str(self.tmp / 'no-such-file')
            self.assertEqual(self.status('G0'), 'BLOCKED', f'{key}: missing evidence file')
            self.setUp()
            self.g0[key]['sha256'] = '0' * 64
            self.assertEqual(self.status('G0'), 'BLOCKED', f'{key}: wrong sha')
            self.setUp()
            del self.g0[key]['sha256']
            self.assertEqual(self.status('G0'), 'BLOCKED', f'{key}: no sha')
            self.setUp()
            del self.g0[key]['evidence']
            self.assertEqual(self.status('G0'), 'BLOCKED', f'{key}: no evidence')

    def test_identity_needs_replay_sha(self):
        del self.g0['identity']['replaySha256']
        self.assertEqual(self.status('G0'), 'BLOCKED')


class VerdictBindingTest(VerdictBase):
    def binding(self, config='K1'):
        v = self.verdict(config)
        return v['gates']['BINDING']['status'], v

    def edit_freeze(self, fn):
        def hook(path):
            d = json.loads(path.read_text())
            fn(d)
            path.write_text(json.dumps(d))
        self.after_freeze = hook

    def test_consistent_inputs_pass(self):
        self.assertEqual(self.binding()[0], 'PASS')

    def blocked(self, why):
        status, v = self.binding()
        self.assertEqual(status, 'BLOCKED', why)
        self.assertEqual(v['overall'], 'BLOCKED', why)
        return v

    def test_prereg_sha_mismatch(self):
        self.edit_freeze(lambda d: d['prereg'].update(sha256='0' * 64))
        self.blocked('prereg sha')

    def test_e0_sha_mismatch(self):
        self.edit_freeze(lambda d: d['e0'].update(sha256='0' * 64))
        self.blocked('e0 sha')

    def test_tool_sha_mismatch(self):
        self.edit_freeze(lambda d: d['tools'].update({str(self.tool): '0' * 64}))
        self.blocked('tool sha')

    def test_addendum_sha_mismatch_or_missing(self):
        self.edit_freeze(lambda d: d['addenda'][0].update(sha256='0' * 64))
        self.blocked('addendum sha')
        self.setUp()
        self.edit_freeze(lambda d: d['addenda'][0].update(path=str(self.tmp / 'gone.json')))
        self.blocked('addendum file missing')

    def test_bundle_hash_mismatch(self):
        self.edit_freeze(lambda d: d['bundles']['ob-a'].update({'frames.tsv': '0' * 64}))
        self.blocked('bundle gate sequence')
        self.setUp()
        self.edit_freeze(lambda d: d['bundles']['fx-a'].update({'sensors.tsv': '0' * 64}))
        self.blocked('bundle of a fixture')  # fixtures come from sourceBundles/bundleOf
        self.setUp()
        self.edit_freeze(lambda d: d['bundles'].pop('fw-b'))
        self.blocked('sequence without a frozen bundle')

    def test_e0_replay_binary(self):
        self.prereg['e0']['replayBinarySha256'] = 'other'
        self.blocked('e0 binary')
        self.setUp()
        self.e0['replaySha256'] = 'other'
        self.blocked('e0 metrics binary')

    def test_config_name_binding(self):
        self.cand['config'] = 'K1'
        status, v = self.binding('OFF')  # --config OFF but env of OFF vs prereg OFF is fine ...
        self.assertEqual(status, 'PASS')  # ... verdict() rewrites config consistently
        # A candidate metrics file collected under another name is caught: patch it after the fact.
        self.setUp()
        orig = gate_xr.load_json

        def patched(path):
            d = orig(path)
            if isinstance(d, dict) and d.get('schema') == 'xr-commercial-metrics-v1' and d['config'] == 'K1':
                d['config'] = 'K2'
            return d
        gate_xr.load_json = patched
        self.addCleanup(setattr, gate_xr, 'load_json', orig)
        self.blocked('metrics.config != --config')

    def test_accuracy_env_must_equal_the_configuration(self):
        self.env_override = 'enabled=1,k1=0'
        self.blocked('accuracyEnv')
        self.setUp()
        self.env_override = ''
        self.blocked('empty env for K1')

    def test_unknown_configuration_blocks(self):
        self.addenda = []  # K2 is only defined by the addendum
        status, _ = self.binding('K2')
        self.assertEqual(status, 'BLOCKED')

    def test_addendum_may_add_but_not_redefine_or_carry_other_keys(self):
        self.addenda = [{'schema': 'x', 'note': 'n', 'configurations': {'K2': 'k2=1'}}]
        self.assertEqual(self.binding('K2')[0], 'PASS')
        self.setUp()
        self.addenda = [{'configurations': {'K1': 'changed'}}]
        self.blocked('redefines K1')
        self.setUp()
        self.addenda = [{'configurations': {'K2': 'k2=1'}, 'limits': {'ratio': 9}}]
        self.blocked('other key in addendum')
        self.setUp()
        self.addenda = [{'configurations': {'K2': 'a'}}, {'configurations': {'K2': 'b'}}]
        self.blocked('two addenda define the same name')

    def test_one_candidate_binary(self):
        self.g0['identity']['replaySha256'] = 'other-bin'
        self.blocked('g0 identity binary')
        self.setUp()
        self.timing['replaySha256'] = 'other-bin'
        self.blocked('timing binary')
        self.setUp()
        self.timing = None  # without --timing the timing binding is not checked (G5 blocks instead)
        self.assertEqual(self.binding()[0], 'PASS')

    def test_timing_accuracy_binding(self):
        self.timing_on_override = ('enabled=1,k1=0',)
        self.blocked('timing accuracyOn differs from the configuration env')
        self.setUp()
        self.timing_on_override = ('',)
        self.blocked('timing accuracyOn empty')
        self.setUp()
        self.timing_on_override = (None,)
        self.blocked('timing accuracyOn unknown')
        self.setUp()
        self.timing['accuracyOff'] = 'enabled=1,k1=1'
        self.blocked('timing accuracyOff not empty')
        self.setUp()
        self.timing['accuracyOff'] = None
        self.blocked('timing accuracyOff unknown')
        self.setUp()
        self.timing_on_override = None
        self.assertEqual(self.binding()[0], 'PASS')

    def test_identity_evidence_carries_the_binary(self):
        self.set_identity_evidence(replaySha256='other-bin')
        self.blocked('evidence binary differs from g0 and metrics')
        self.setUp()
        self.g0['identity']['replaySha256'] = 'other-bin'  # g0 says another binary than the evidence
        self.blocked('g0 binary differs from the evidence')
        self.setUp()
        self.set_identity_evidence(replaySha256=None)
        self.blocked('evidence has no binary')

    def test_identity_evidence_pass_must_be_true(self):
        self.set_identity_evidence(**{'pass': False})
        self.blocked('evidence pass false while g0 says true')

    def test_identity_evidence_must_be_json(self):
        self.evidence['identity'].write_text('not json')
        self.g0['identity']['sha256'] = gate_xr.sha256_file(self.evidence['identity'])
        self.blocked('evidence not JSON')

    def test_attempts_limit_and_membership(self):
        self.attempts = ['OFF', 'K1', 'K2', 'K1+K2']  # 3 other than OFF <= 4
        self.assertEqual(self.binding()[0], 'PASS')
        self.setUp()
        self.attempts = ['OFF', 'K1', 'K2', 'K1+K2', 'K3', 'K4']  # 5 > 4
        self.blocked('too many configurations')
        self.setUp()
        self.attempts = ['OFF', 'K2']  # K1 not listed
        self.blocked('config not among the attempts')
        self.setUp()
        self.prereg['limits']['maxConfigurations'] = 2
        self.blocked('limit read from the prereg')

    def test_attempts_boundary_is_inclusive(self):
        self.prereg['limits']['maxConfigurations'] = 3
        self.assertEqual(self.binding()[0], 'PASS')


class FreezeCommandTest(VerdictBase):
    def run_freeze(self, out, *extra):
        (self.tmp / 'p.json').write_text(json.dumps(self.prereg))
        (self.tmp / 'e.json').write_text(json.dumps(self.e0))
        with contextlib.redirect_stdout(io.StringIO()):
            return gate_xr.main(['freeze', '--prereg', str(self.tmp / 'p.json'), '--e0', str(self.tmp / 'e.json'),
                                 '--tools', str(self.tool), '--out', str(out), *extra])

    def test_freeze_contents_and_no_overwrite(self):
        out = self.tmp / 'freeze.json'
        self.run_freeze(out)
        f = json.loads(out.read_text())
        self.assertEqual(f['prereg']['sha256'], gate_xr.sha256_file(self.tmp / 'p.json'))
        self.assertEqual(f['e0']['sha256'], gate_xr.sha256_file(self.tmp / 'e.json'))
        self.assertEqual(f['tools'][str(self.tool)], gate_xr.sha256_file(self.tool))
        self.assertEqual(set(f['bundles']), {'ob-a', 'fw-a', 'fw-b', 'fx-a', 'fx-c'})
        self.assertEqual(set(f['bundles']['fx-a']), {'sensors.tsv', 'frames.tsv', 'engine-config.tsv'})
        self.assertEqual(f['bundles']['ob-a']['frames.tsv'],
                         gate_xr.sha256_file(self.tmp / 'bundles' / 'ob-a' / 'frames.tsv'))
        self.assertTrue(f['createdAt'].endswith('+00:00'))
        with self.assertRaises(SystemExit):
            self.run_freeze(out)

    def test_freeze_add(self):
        out = self.tmp / 'freeze.json'
        self.run_freeze(out)
        add = self.tmp / 'add.json'
        add.write_text(json.dumps({'configurations': {'K2': 'k2=1'}}))
        out2 = self.tmp / 'freeze-2.json'
        with contextlib.redirect_stdout(io.StringIO()):
            gate_xr.main(['freeze-add', '--freeze', str(out), '--addendum', str(add), '--out', str(out2)])
        a, b = json.loads(out.read_text()), json.loads(out2.read_text())
        self.assertEqual(b['previousFreezeSha256'], gate_xr.sha256_file(out))
        self.assertEqual([x['path'] for x in b['addenda']], [str(add)])
        for key in ('prereg', 'e0', 'bundles', 'tools'):
            self.assertEqual(a[key], b[key])
        with self.assertRaises(SystemExit):
            with contextlib.redirect_stdout(io.StringIO()):
                gate_xr.main(['freeze-add', '--freeze', str(out), '--addendum', str(add), '--out', str(out2)])


class VerdictG1Test(VerdictBase):
    def test_boundary(self):
        self.cand['runs']['ob-a']['eStart'] = 0.5
        self.assertEqual(self.status('G1'), 'PASS')
        self.cand['runs']['ob-a']['eStart'] = 0.5000001
        self.assertEqual(self.status('G1'), 'FAIL')

    def test_missing_metric_or_run_blocks(self):
        self.cand['runs']['ob-a']['eStart'] = None
        self.assertEqual(self.status('G1'), 'BLOCKED')
        self.cand['runs']['ob-a'] = {'kind': 'outback', 'missing': True, 'exit': None}
        self.assertEqual(self.status('G1'), 'BLOCKED')
        del self.cand['runs']['ob-a']
        self.assertEqual(self.status('G1'), 'BLOCKED')

    def test_e0_missing_blocks(self):
        self.e0['runs']['ob-a']['eStart'] = None
        self.assertEqual(self.status('G1'), 'BLOCKED')


class VerdictG2Test(VerdictBase):
    def set(self, **kw):
        self.cand['runs']['ob-a'].update(kw)

    def test_estart_ratio_boundary_large_e0(self):
        self.e0['runs']['ob-a']['eStart'] = 10.0  # ratio limit 10.5 beats floor limit 10.02
        self.set(eStart=10.4)  # G1 is not looked at here, only G2
        self.assertEqual(self.status('G2'), 'PASS')  # would fail with the absFloor limit alone
        self.set(eStart=10.5)
        self.assertEqual(self.status('G2'), 'PASS')
        self.set(eStart=10.6)
        self.assertEqual(self.status('G2'), 'FAIL')

    def test_estart_floor_for_small_e0(self):
        self.e0['runs']['ob-a']['eStart'] = 0.1  # ratio 0.105, floor 0.12
        self.set(eStart=0.115)  # beats the ratio limit only through the floor
        self.assertEqual(self.status('G2'), 'PASS')
        self.set(eStart=0.13)
        self.assertEqual(self.status('G2'), 'FAIL')

    def test_eall_same_rule(self):
        self.e0['runs']['ob-a']['eAll'] = 0.1
        self.set(eAll=0.115)
        self.assertEqual(self.status('G2'), 'PASS')
        self.set(eAll=0.13)
        self.assertEqual(self.status('G2'), 'FAIL')

    def test_eall_ratio_boundary(self):
        self.set(eAll=1.05)
        self.assertEqual(self.status('G2'), 'PASS')
        self.set(eAll=1.0500001)
        self.assertEqual(self.status('G2'), 'FAIL')

    def test_discards_and_normal_ratio(self):
        self.set(discards=1)
        self.assertEqual(self.status('G2'), 'PASS')
        self.set(discards=2)
        self.assertEqual(self.status('G2'), 'FAIL')
        self.set(discards=0, normalRatio=0.9 - 0.01)  # 0.89 == E0 - coverageLossMax (0.9 - 0.01)
        self.assertEqual(self.status('G2'), 'PASS')
        self.set(normalRatio=0.889)
        self.assertEqual(self.status('G2'), 'FAIL')

    def test_incomplete_e_start_fails(self):
        self.set(eStartComplete=False)
        self.assertEqual(self.status('G2'), 'FAIL')

    def test_exit_stall_fail_and_missing_block(self):
        # A run that exists and exits non-zero or stalls is the candidate's fault: FAIL.
        self.set(exit=1)
        self.assertEqual(self.status('G2'), 'FAIL')
        self.set(exit=0, stall=1)
        self.assertEqual(self.status('G2'), 'FAIL')
        # Missing information is BLOCKED.
        self.set(stall=0, exit=None)
        self.assertEqual(self.status('G2'), 'BLOCKED')
        self.set(exit=0, eAll=None)
        self.assertEqual(self.status('G2'), 'BLOCKED')
        self.cand['runs']['ob-a'] = {'kind': 'outback', 'missing': True, 'exit': 0}
        self.assertEqual(self.status('G2'), 'BLOCKED')

    def test_map_lost_limit_uses_ratio_or_floor(self):
        # E0 mapLost 10: ratio 10.5, floor 15 -> floor applies.
        self.set(mapLost=15)
        self.assertEqual(self.status('G2'), 'PASS')
        self.assertEqual(self.check('G2', 'ob-a', 'mapLost')['bound'], 'floor')
        self.set(mapLost=16)
        self.assertEqual(self.status('G2'), 'FAIL')
        # E0 mapLost 1000: ratio 1050, floor 1005 -> ratio applies.
        self.e0['runs']['ob-a']['mapLost'] = 1000
        self.set(mapLost=1050)
        self.assertEqual(self.status('G2'), 'PASS')
        self.assertEqual(self.check('G2', 'ob-a', 'mapLost')['bound'], 'ratio')
        self.set(mapLost=1051)
        self.assertEqual(self.status('G2'), 'FAIL')
        self.set(mapLost=None)
        self.assertEqual(self.status('G2'), 'BLOCKED')

    def test_map_lost_floor_comes_from_the_prereg(self):
        self.set(mapLost=15)
        self.prereg['limits']['mapLostFloorFrames'] = 2
        self.assertEqual(self.status('G2'), 'FAIL')

    def test_rows_must_equal_e0(self):
        self.set(rows=100)
        self.assertEqual(self.status('G2'), 'PASS')
        self.set(rows=99)
        self.assertEqual(self.status('G2'), 'FAIL')
        self.set(rows=101)
        self.assertEqual(self.status('G2'), 'FAIL')

    def test_path_must_not_collapse(self):
        self.set(path=9.0)  # == pathMinRatio 0.9 * 10
        self.assertEqual(self.status('G2'), 'PASS')
        self.set(path=8.99)
        self.assertEqual(self.status('G2'), 'FAIL')
        self.prereg['limits']['pathMinRatio'] = 0.5
        self.assertEqual(self.status('G2'), 'PASS')
        self.set(path=None)
        self.assertEqual(self.status('G2'), 'BLOCKED')

    def test_rot_start_ratio_or_floor(self):
        # E0 5.0: ratio 5.25, floor 6.0 -> floor
        self.set(rotStartDeg=6.0)
        self.assertEqual(self.status('G2'), 'PASS')
        self.assertEqual(self.check('G2', 'ob-a', 'rotStartDeg')['bound'], 'floor')
        self.set(rotStartDeg=6.01)
        self.assertEqual(self.status('G2'), 'FAIL')
        self.e0['runs']['ob-a']['rotStartDeg'] = 100.0  # ratio 105 > floor 101
        self.set(rotStartDeg=105.0)
        self.assertEqual(self.status('G2'), 'PASS')
        self.assertEqual(self.check('G2', 'ob-a', 'rotStartDeg')['bound'], 'ratio')
        self.set(rotStartDeg=105.1)
        self.assertEqual(self.status('G2'), 'FAIL')

    def test_bound_is_recorded(self):
        self.e0['runs']['ob-a']['eAll'] = 10.0
        self.assertEqual(self.check('G2', 'ob-a', 'eAll')['bound'], 'ratio')
        self.e0['runs']['ob-a']['eAll'] = 0.1
        self.assertEqual(self.check('G2', 'ob-a', 'eAll')['bound'], 'floor')


class VerdictG3Test(VerdictBase):
    def test_ratio_and_floor(self):
        self.cand['runs']['fw-a']['eSpeed'] = 1.05
        self.assertEqual(self.status('G3'), 'PASS')
        self.cand['runs']['fw-a']['eSpeed'] = 1.0500001
        self.assertEqual(self.status('G3'), 'FAIL')
        self.e0['runs']['fw-a']['eSpeed'] = 0.1
        self.cand['runs']['fw-a']['eSpeed'] = 0.115
        self.assertEqual(self.status('G3'), 'PASS')
        self.cand['runs']['fw-a']['eSpeed'] = 0.13
        self.assertEqual(self.status('G3'), 'FAIL')

    def test_incomplete_fails_and_run_problems_block(self):
        self.cand['runs']['fw-b']['complete'] = False
        self.assertEqual(self.status('G3'), 'FAIL')
        self.cand['runs']['fw-b']['complete'] = True
        self.cand['runs']['fw-b']['stall'] = 3
        self.assertEqual(self.status('G3'), 'FAIL')
        self.cand['runs']['fw-b']['stall'] = 0
        self.cand['runs']['fw-b']['exit'] = 139
        self.assertEqual(self.status('G3'), 'FAIL')
        self.cand['runs']['fw-b']['exit'] = None
        self.assertEqual(self.status('G3'), 'BLOCKED')

    def test_rows_and_map_lost(self):
        self.cand['runs']['fw-a']['rows'] = 99
        self.assertEqual(self.status('G3'), 'FAIL')
        self.cand['runs']['fw-a']['rows'] = 100
        self.cand['runs']['fw-a']['mapLost'] = 15  # E0 10: floor 15
        self.assertEqual(self.status('G3'), 'PASS')
        self.cand['runs']['fw-a']['mapLost'] = 16
        self.assertEqual(self.status('G3'), 'FAIL')


class VerdictG4Test(VerdictBase):
    def acc(self, **kw):
        self.cand['runs']['fx-a']['accuracy'].update(kw)

    def test_each_accuracy_metric_boundary(self):
        for key in gate_xr.ACCURACY_KEYS:
            self.cand = make_metrics('K1')
            self.cand['runs']['ob-a']['eStart'] = 0.5
            self.acc(**{key: FIX_ACC[key] * 1.05})
            self.assertEqual(self.status('G4'), 'PASS', key)
            self.acc(**{key: FIX_ACC[key] * 1.0501})
            self.assertEqual(self.status('G4'), 'FAIL', key)

    def test_coverage_normal_ratio_resets(self):
        self.acc(coverage=0.95 - 0.01)
        self.assertEqual(self.status('G4'), 'PASS')
        self.acc(coverage=0.9)
        self.assertEqual(self.status('G4'), 'FAIL')
        self.acc(coverage=0.95)
        self.cand['runs']['fx-a']['normalRatio'] = 0.88
        self.assertEqual(self.status('G4'), 'FAIL')
        self.cand['runs']['fx-a']['normalRatio'] = 0.9
        self.cand['runs']['fx-a']['resets'] = 3
        self.assertEqual(self.status('G4'), 'FAIL')

    def test_fixture_gate_keys_limit_the_gated_metrics(self):
        """sequences.fixtureGateKeys (TUM: RPE only): other accuracy keys are reported, not gated."""
        self.prereg['sequences']['fixtureGateKeys'] = {'fx-a': ['rpe1sTransSim3Rmse', 'rpe1sRotDegRmse']}
        self.acc(sim3AteRmse=FIX_ACC['sim3AteRmse'] * 3, apeRotDegRmse=FIX_ACC['apeRotDegRmse'] * 3)
        self.assertEqual(self.status('G4'), 'PASS')
        self.acc(rpe1sRotDegRmse=FIX_ACC['rpe1sRotDegRmse'] * 1.0501)
        self.assertEqual(self.status('G4'), 'FAIL')

    def test_identity_shortcut(self):
        self.cand['runs']['fx-a']['accuracy'] = {k: 99.0 for k in (*gate_xr.ACCURACY_KEYS, 'coverage')}
        self.cand['runs']['fx-a']['resets'] = 9
        self.assertEqual(self.status('G4'), 'FAIL')
        self.cand['runs']['fx-a']['identicalToE0'] = True
        v = self.verdict()
        self.assertEqual(v['gates']['G4']['status'], 'PASS')
        self.assertTrue(any(c.get('identity') for c in v['gates']['G4']['checks']))

    def test_identity_does_not_excuse_stall_or_exit(self):
        self.cand['runs']['fx-a']['identicalToE0'] = True
        self.cand['runs']['fx-a']['stall'] = 1
        self.assertEqual(self.status('G4'), 'FAIL')

    def test_rows_and_map_lost_for_fixtures_and_chrome(self):
        for name in ('fx-a', 'fx-c'):
            self.setUp()
            run = self.cand['runs'][name]
            run['rows'] = 99
            self.assertEqual(self.status('G4'), 'FAIL', name)
            run['rows'] = 100
            run['mapLost'] = 15
            self.assertEqual(self.status('G4'), 'PASS', name)
            run['mapLost'] = 16
            self.assertEqual(self.status('G4'), 'FAIL', name)

    def test_longest_non_normal_run(self):
        self.prereg['sequences']['fixtureGateKeys'] = {'fx-a': ['rpe1sTransSim3Rmse']}
        # E0 4: limit 1.05 * 4 = 4.2 -> 4 passes, 5 fails
        self.cand['runs']['fx-a']['longestNonNormalRun'] = 4
        self.assertEqual(self.status('G4'), 'PASS')
        self.cand['runs']['fx-a']['longestNonNormalRun'] = 5
        self.assertEqual(self.status('G4'), 'FAIL')
        self.cand['runs']['fx-a']['longestNonNormalRun'] = 4
        self.cand['runs']['fx-c']['longestNonNormalRun'] = 5  # chromeForward is always gated
        self.assertEqual(self.status('G4'), 'FAIL')
        self.cand['runs']['fx-c']['longestNonNormalRun'] = None
        self.assertEqual(self.status('G4'), 'BLOCKED')

    def test_longest_non_normal_run_only_for_gate_key_fixtures(self):
        self.cand['runs']['fx-a']['longestNonNormalRun'] = 400  # fx-a is not listed in fixtureGateKeys
        self.assertEqual(self.status('G4'), 'PASS')
        self.prereg['sequences']['fixtureGateKeys'] = {'fx-a': ['rpe1sTransSim3Rmse']}
        self.assertEqual(self.status('G4'), 'FAIL')

    def test_longest_non_normal_run_zero_in_e0_gives_limit_zero(self):
        self.e0['runs']['fx-c']['longestNonNormalRun'] = 0
        self.cand['runs']['fx-c']['longestNonNormalRun'] = 0
        self.assertEqual(self.status('G4'), 'PASS')
        self.cand['runs']['fx-c']['longestNonNormalRun'] = 1
        self.assertEqual(self.status('G4'), 'FAIL')

    def test_identity_shortcut_skips_the_other_checks(self):
        self.prereg['sequences']['fixtureGateKeys'] = {'fx-a': ['rpe1sTransSim3Rmse']}
        run = self.cand['runs']['fx-a']
        run.update(identicalToE0=True, rows=1, mapLost=999, longestNonNormalRun=999)
        self.assertEqual(self.status('G4'), 'PASS')

    def test_chrome_forward(self):
        self.cand['runs']['fx-c']['normalRatio'] = 0.9 - 0.01
        self.assertEqual(self.status('G4'), 'PASS')
        self.cand['runs']['fx-c']['normalRatio'] = 0.85
        self.assertEqual(self.status('G4'), 'FAIL')
        self.cand['runs']['fx-c']['normalRatio'] = 0.9
        self.cand['runs']['fx-c']['resets'] = 3
        self.assertEqual(self.status('G4'), 'FAIL')
        self.cand['runs']['fx-c']['resets'] = 2
        self.cand['runs']['fx-c']['exit'] = 1
        self.assertEqual(self.status('G4'), 'FAIL')
        self.cand['runs']['fx-c']['exit'] = None
        self.assertEqual(self.status('G4'), 'BLOCKED')

    def test_missing_accuracy_blocks(self):
        self.cand['runs']['fx-a'].pop('accuracy')
        self.cand['runs']['fx-a']['accuracyMissing'] = True
        self.assertEqual(self.status('G4'), 'BLOCKED')


class VerdictG5Test(VerdictBase):
    def test_fixture_boundary(self):
        self.timing = make_timing(fx_off=10.0, fx_on=11.0)
        self.assertEqual(self.status('G5'), 'PASS')
        self.timing = make_timing(fx_off=10.0, fx_on=11.1)
        self.assertEqual(self.status('G5'), 'FAIL')

    def test_envelope_uses_max_of_off_and_envelope_off(self):
        # off 10, envelope off 20: limit 1.1 * 20 = 22 (a plain on <= 1.1*off would fail at 12).
        self.timing = make_timing(ob_off=10.0, ob_on=22.0, env_off=20.0, fx_off=20.0, fx_on=20.0)
        self.assertEqual(self.status('G5'), 'PASS')
        self.timing = make_timing(ob_off=10.0, ob_on=22.5, env_off=20.0, fx_off=20.0, fx_on=20.0)
        self.assertEqual(self.status('G5'), 'FAIL')
        # the sequence's own off larger than the envelope: that one is the base
        self.timing = make_timing(ob_off=30.0, ob_on=33.0, fx_off=20.0, fx_on=20.0)
        self.assertEqual(self.status('G5'), 'PASS')
        self.timing = make_timing(ob_off=30.0, ob_on=33.5, fx_off=20.0, fx_on=20.0)
        self.assertEqual(self.status('G5'), 'FAIL')

    def test_bad_timing_runs_fail(self):
        self.timing['badRuns'] = ['ob-a/2-on: exit 139']
        v = self.verdict()
        self.assertEqual(v['gates']['G5']['status'], 'FAIL')
        self.timing['badRuns'] = ['fx-a/1-off: stall 3']
        self.assertEqual(self.status('G5'), 'FAIL')
        self.timing['badRuns'] = []
        self.assertEqual(self.status('G5'), 'PASS')

    def test_unknown_exit_blocks(self):
        self.timing['unknownExit'] = ['fx-a/1-on']
        self.assertEqual(self.status('G5'), 'BLOCKED')
        self.timing['unknownExit'] = []
        del self.timing['badRuns']
        self.assertEqual(self.status('G5'), 'BLOCKED')

    def test_lost_frame_cost_bound(self):
        # limit = timingRatio 1.1 * lostFrameP95EnvelopeMs 100 = 110 (inclusive)
        self.timing = make_timing(lost_on=110.0)
        self.assertEqual(self.status('G5'), 'PASS')
        self.timing = make_timing(lost_on=110.1)
        self.assertEqual(self.status('G5'), 'FAIL')
        self.prereg['limits']['lostFrameP95EnvelopeMs'] = 50.0
        self.timing = make_timing(lost_on=60.0)
        self.assertEqual(self.status('G5'), 'FAIL')
        self.timing = make_timing(lost_on=55.0)
        self.assertEqual(self.status('G5'), 'PASS')

    def test_lost_frame_cost_missing_repetition_blocks(self):
        self.timing = make_timing()
        self.timing['sequences']['fx-a']['on']['lostP95Median'] = None
        self.assertEqual(self.status('G5'), 'BLOCKED')
        del self.timing['sequences']['fx-a']['on']['lostP95Median']
        self.assertEqual(self.status('G5'), 'BLOCKED')

    def test_missing_timing_blocks(self):
        self.timing = None
        self.assertEqual(self.status('G5'), 'BLOCKED')
        self.timing = make_timing()
        self.timing['sequences']['fx-a']['on']['median'] = None
        self.assertEqual(self.status('G5'), 'BLOCKED')
        self.timing = make_timing()
        del self.timing['sequences']['ob-a']
        self.assertEqual(self.status('G5'), 'BLOCKED')


class VerdictG6Test(VerdictBase):
    def setUp(self):
        super().setUp()
        self.compare = make_metrics('OFF')
        self.compare['runs']['fw-a']['eSpeed'] = 1.0
        self.compare['runs']['fw-b']['eSpeed'] = 2.0

    def test_not_applicable_without_k2(self):
        for config in ('K1', 'OFF'):
            self.assertEqual(self.status('G6', config), 'NOT_APPLICABLE')

    def test_count_of_sequences(self):
        self.cand['runs']['fw-a']['eSpeed'] = 0.75   # == 0.75 * 1.0
        self.cand['runs']['fw-b']['eSpeed'] = 1.5    # == 0.75 * 2.0
        for config in ('K2', 'K1+K2'):
            self.assertEqual(self.status('G6', config), 'PASS')
        self.cand['runs']['fw-b']['eSpeed'] = 1.51   # only one sequence beats the comparison
        self.assertEqual(self.status('G6', 'K2'), 'FAIL')
        self.cand['runs']['fw-a']['eSpeed'] = 0.76
        self.assertEqual(self.status('G6', 'K2'), 'FAIL')

    def test_overall_requires_g6_for_k2(self):
        self.cand['runs']['fw-a']['eSpeed'] = 1.0
        self.assertEqual(self.verdict('K2')['overall'], 'REJECT')
        self.assertEqual(self.verdict('K1')['overall'], 'ADOPT')

    def test_without_compare_or_value_blocks(self):
        self.compare = None
        self.assertEqual(self.status('G6', 'K2'), 'BLOCKED')
        self.compare = make_metrics('OFF')
        self.compare['runs']['fw-a']['eSpeed'] = None
        self.assertEqual(self.status('G6', 'K2'), 'BLOCKED')

    def test_min_sequences_from_prereg(self):
        self.cand['runs']['fw-a']['eSpeed'] = 0.5
        self.cand['runs']['fw-b']['eSpeed'] = 2.0
        self.assertEqual(self.status('G6', 'K2'), 'FAIL')
        self.prereg['limits']['k2BenefitMinSequences'] = 1
        self.assertEqual(self.status('G6', 'K2'), 'PASS')


# ---------------------------------------------------------------- collect

class CollectTest(OutbackBase):
    def setUp(self):
        super().setUp()
        self.bundles = self.tmp / 'bundles'
        self.bundles.mkdir()
        palindrome_bundle.palindrome_bundle(self.src, self.bundles / 'ob-a')  # 8 forward frames, 15 rows
        self.prereg = {
            'bundles': str(self.bundles),
            'sequences': {'outback': ['ob-a', 'ob-gone'], 'primary': ['ob-a'],
                          'forwardSpeed': {'fw-a': {'reference': 'fx-a', 'k': 2}},
                          'k2Benefit': ['fw-a'], 'fixtures': {}, 'chromeForward': ['fx-c'], 'timing': {}},
            'bundleOf': {'fx-c': 'synthetic-none/acc'},
            'limits': {'speedFirstRow': 2},
        }
        (self.tmp / 'prereg.json').write_text(json.dumps(self.prereg))
        self.e0, self.runs = self.tmp / 'e0', self.tmp / 'runs'

    def posed(self, xs, stage=None):
        rows = []
        for i, x in enumerate(xs):
            r = {'i': i, 't': i * 0.05, 'fresh': True, 'status': 'NORMAL', 'p': [x, 0.0, 0.0],
                 'q_wxyz': [1, 0, 0, 0], 'srMapReset': i == 0, 'srMapTrackerState': 1}
            if stage is not None:
                r['srStageMs'] = stage
            rows.append(r)
        return rows

    def test_collect(self):
        out_xs = [0.0, 1, 2, 3, 4, 5, 6, 7]
        pal = out_xs + out_xs[-2::-1]
        for root, stage in ((self.e0, {'mapTracker': 5.0, 'mapBuilder': 1.0}), (self.runs, {'mapTracker': 1.0})):
            write_run(root, 'ob-a', self.posed(pal, stage))
            write_run(root, 'fx-a', self.posed([float(i) for i in range(40)]))
            write_run(root, 'fw-a', self.posed([2.0 * i for i in range(20)], stage))
            write_run(root, 'fx-c', self.posed([0.0, 1, 2, 3]))
        status = [{'name': 'ob-a', 'status': 'done', 'exit': 0, 'replaySha256': 'abc', 'accuracy': 'k=v'},
                  {'name': 'fw-a', 'status': 'failed', 'exit': 2, 'replaySha256': 'abc', 'accuracy': 'k=v'},
                  {'name': 'fw-a', 'status': 'exists', 'replaySha256': 'abc', 'accuracy': 'k=v'}]
        (self.runs / 'status.jsonl').write_text('\n'.join(json.dumps(s) for s in status) + '\n')
        out = self.tmp / 'metrics.json'
        with contextlib.redirect_stdout(io.StringIO()):
            gate_xr.main(['collect', '--prereg', str(self.tmp / 'prereg.json'), '--runs', str(self.runs),
                          '--config', 'K1', '--e0-runs', str(self.e0), '--out', str(out)])
        m = json.loads(out.read_text())
        self.assertEqual((m['schema'], m['config'], m['accuracyEnv'], m['replaySha256']),
                         ('xr-commercial-metrics-v1', 'K1', 'k=v', 'abc'))
        ob = m['runs']['ob-a']
        self.assertFalse(ob['missing'])
        self.assertEqual((ob['kind'], ob['exit'], ob['status'], ob['rows'], ob['resets'], ob['discards']),
                         ('outback', 0, 'done', 15, 1, 0))
        self.assertEqual(ob['stall'], 0)
        self.assertEqual(ob['eStart'], None)  # frames 20..40 do not exist in the 8-frame bundle
        self.assertFalse(ob['eStartComplete'])
        self.assertEqual(ob['normalRatio'], 1.0)
        self.assertTrue(ob['identicalToE0'])  # only srStageMs differs from E0
        self.assertEqual(ob['forwardFrames'], 8)
        fw = m['runs']['fw-a']
        self.assertEqual(fw['exit'], 2)  # the 'exists' line did not hide the failure
        self.assertEqual(fw['eSpeed'], 0.0)  # run row i is reference row 2*i
        self.assertTrue(fw['complete'])
        gone = m['runs']['ob-gone']
        self.assertTrue(gone['missing'])
        self.assertIsNone(gone['exit'])
        ch = m['runs']['fx-c']
        self.assertIsNone(ch['exit'])  # no status line
        self.assertEqual(ch['normalRatio'], 1.0)
        self.assertEqual(ch['trajectory']['pathLength'], 3.0)
        self.assertEqual((ch['rows'], ch['mapLost'], ch['longestNonNormalRun']), (4, 0, 0))
        self.assertEqual((ob['mapLost'], fw['mapLost']), (0, 0))
        self.assertGreater(ob['path'], 0)
        self.assertIn('rotStartDeg', ob)
        self.assertNotIn('fx-a', m['runs'])  # fixtures {} lists none

    def test_truncated_trace_and_missing_reference_do_not_abort(self):
        d = write_run(self.runs, 'ob-a', self.posed([0.0] * 15))
        (d / 'trace.jsonl').write_text('{"i": 0, "p": [0,0,0]}\n{"i": 1, "p"')  # run still being written
        write_run(self.runs, 'fw-a', self.posed([0.0] * 10))  # its reference fx-a is absent
        out = self.tmp / 'm2.json'
        with contextlib.redirect_stdout(io.StringIO()):
            gate_xr.main(['collect', '--prereg', str(self.tmp / 'prereg.json'), '--runs', str(self.runs),
                          '--config', 'K1', '--e0-runs', str(self.e0), '--out', str(out)])
        m = json.loads(out.read_text())
        self.assertTrue(m['runs']['ob-a']['missing'])
        self.assertTrue(m['runs']['fw-a']['referenceMissing'])
        self.assertIsNone(m['runs']['fw-a']['eSpeed'])

    def test_disagreeing_accuracy_env_is_an_error(self):
        write_run(self.runs, 'ob-a', self.posed([0.0] * 15))
        status = [{'name': 'ob-a', 'status': 'done', 'exit': 0, 'replaySha256': 'abc', 'accuracy': 'a'},
                  {'name': 'fw-a', 'status': 'done', 'exit': 0, 'replaySha256': 'abc', 'accuracy': 'b'}]
        (self.runs / 'status.jsonl').write_text('\n'.join(json.dumps(s) for s in status) + '\n')
        with self.assertRaises(SystemExit):
            with contextlib.redirect_stdout(io.StringIO()):
                gate_xr.main(['collect', '--prereg', str(self.tmp / 'prereg.json'), '--runs', str(self.runs),
                              '--config', 'K1', '--e0-runs', str(self.e0), '--out', str(self.tmp / 'm.json')])


class CollectMetricsTest(TmpCase):
    def test_map_lost_and_longest_non_normal_run(self):
        rows = []
        for i in range(10):
            r = {'i': i, 'fresh': True, 'status': 'NORMAL', 'p': [float(i), 0, 0], 'srMapTrackerState': 1}
            rows.append(r)
        rows[0]['srMapTrackerState'] = 0          # the first row never counts as map-lost
        rows[2]['srMapTrackerState'] = 0          # counts
        rows[3]['srMapTrackerState'] = None       # counts (!= 1)
        for i in (4, 5, 6):
            rows[i]['status'] = 'LIMITED'         # run of 3
        rows[8]['p'] = None                       # a pose-less row counts as non-NORMAL too
        m = gate_xr.common_metrics(rows)
        self.assertEqual(m['mapLost'], 2)
        self.assertEqual(gate_xr.longest_non_normal_run(rows), 3)   # rows 4..6; row 8 alone is 1
        rows[7]['status'] = 'LIMITED'                               # joins rows 4..8 into one run
        self.assertEqual(gate_xr.longest_non_normal_run(rows), 5)
        self.assertEqual(gate_xr.longest_non_normal_run([]), 0)


class TimingCollectTest(TmpCase):
    def test_timing_p95_and_missing(self):
        prereg = make_prereg()
        prereg['sequences']['timing'] = {'fx-a': {}}
        (self.tmp / 'prereg.json').write_text(json.dumps(prereg))
        root = self.tmp / 'timing'
        for rep, arm, scale in ((1, 'off', 1), (2, 'off', 2), (3, 'off', 3), (1, 'on', 1), (2, 'on', 1)):
            rows = [{'i': i, 'srStageMs': {'mapTracker': scale * 1.0, 'mapBuilder': scale * float(i)}}
                    for i in range(50)]
            rows[0]['srStageMs'] = {'mapTracker': 1e6, 'mapBuilder': 0}  # before timingFirstRow: ignored
            write_run(root / 'fx-a', f'{rep}-{arm}', rows)
        (root / 'status.jsonl').write_text(json.dumps(
            {'sequence': 'fx-a', 'rep': 1, 'arm': 'off', 'load1Start': 1.5, 'load1End': 2.5}) + '\n')
        out = self.tmp / 'timing.json'
        with contextlib.redirect_stdout(io.StringIO()):
            gate_xr.main(['timing', '--prereg', str(self.tmp / 'prereg.json'), '--runs', str(root),
                          '--out', str(out)])
        t = json.loads(out.read_text())
        # rows 30..49: totals scale*(1+i); sorted 31..50 -> p95 = 49.05 for scale 1
        off = t['sequences']['fx-a']['off']
        for got, want in zip(off['values'], (49.05, 98.1, 147.15)):
            self.assertAlmostEqual(got, want, places=6)
        self.assertAlmostEqual(off['median'], 98.1, places=6)
        self.assertIsNone(t['sequences']['fx-a']['on']['median'])  # only 2 of 3 repetitions
        self.assertEqual(t['missing'], ['fx-a/3-on'])
        self.assertEqual(t['load1']['fx-a/1-off'], [1.5, 2.5])

    def test_tracked_rows_only_and_lost_frame_report(self):
        """rows == 'tracked': map-lost rows do not enter the gated p95; they are reported apart."""
        prereg = make_prereg()
        prereg['sequences']['timing'] = {'ob-a': {'envelope': 'fx-a', 'rows': 'tracked'}}
        (self.tmp / 'prereg.json').write_text(json.dumps(prereg))
        root = self.tmp / 'timing'
        for rep in (1, 2, 3):
            for arm in ('off', 'on'):
                rows = []
                for i in range(50):
                    lost = arm == 'on' and i >= 40  # 10 map-lost rows with a large cost in the ON arm
                    rows.append({'i': i, 'srMapTrackerState': 0 if lost else 1,
                                 'srPointsBySource': [100, 200, 0, 0, 0],
                                 'srStageMs': {'mapTracker': 500.0 if lost else 2.0, 'mapBuilder': 1.0}})
                write_run(root / 'ob-a', f'{rep}-{arm}', rows)
        out = self.tmp / 'timing.json'
        with contextlib.redirect_stdout(io.StringIO()):
            gate_xr.main(['timing', '--prereg', str(self.tmp / 'prereg.json'), '--runs', str(root),
                          '--out', str(out)])
        seq = json.loads(out.read_text())['sequences']['ob-a']
        self.assertEqual(seq['rows'], 'tracked')
        self.assertAlmostEqual(seq['on']['median'], 3.0, places=6)   # lost rows (501 ms) excluded
        self.assertAlmostEqual(seq['off']['median'], 3.0, places=6)
        lost = seq['on']['lostFrames'][0]
        self.assertEqual(lost['frames'], 10)
        self.assertAlmostEqual(lost['p50'], 501.0, places=6)
        self.assertAlmostEqual(lost['max'], 501.0, places=6)
        self.assertEqual(lost['mapPointsMax'], 300)
        self.assertEqual(seq['off']['lostFrames'][0]['frames'], 0)
        self.assertAlmostEqual(seq['on']['lostP95Median'], 501.0, places=6)
        self.assertEqual(seq['off']['lostP95Median'], 0.0)  # no lost frame costs 0 ms

    def test_lost_p95_median_is_none_with_a_missing_repetition(self):
        prereg = make_prereg()
        prereg['sequences']['timing'] = {'fx-a': {}}
        (self.tmp / 'prereg.json').write_text(json.dumps(prereg))
        root = self.tmp / 'timing'
        for rep, arm in ((1, 'off'), (2, 'off'), (3, 'off'), (1, 'on'), (2, 'on')):
            write_run(root / 'fx-a', f'{rep}-{arm}', [{'i': i, 'srMapTrackerState': 1,
                                                       'srStageMs': {'mapTracker': 1.0}} for i in range(50)])
        out = self.tmp / 'timing.json'
        with contextlib.redirect_stdout(io.StringIO()):
            gate_xr.main(['timing', '--prereg', str(self.tmp / 'prereg.json'), '--runs', str(root),
                          '--out', str(out)])
        seq = json.loads(out.read_text())['sequences']['fx-a']
        self.assertEqual(seq['off']['lostP95Median'], 0.0)
        self.assertIsNone(seq['on']['lostP95Median'])

    def test_replay_sha_is_copied_and_must_agree(self):
        prereg = make_prereg()
        prereg['sequences']['timing'] = {'fx-a': {}}
        (self.tmp / 'prereg.json').write_text(json.dumps(prereg))
        root = self.tmp / 'timing'
        write_run(root / 'fx-a', '1-off', [{'i': 0}])
        lines = [{'sequence': 'fx-a', 'rep': 1, 'arm': 'off', 'replaySha256': 'abc'},
                 {'sequence': 'fx-a', 'rep': 1, 'arm': 'on', 'replaySha256': 'abc'}]
        (root / 'status.jsonl').write_text('\n'.join(json.dumps(l) for l in lines) + '\n')
        out = self.tmp / 'timing.json'
        with contextlib.redirect_stdout(io.StringIO()):
            gate_xr.main(['timing', '--prereg', str(self.tmp / 'prereg.json'), '--runs', str(root),
                          '--out', str(out)])
        self.assertEqual(json.loads(out.read_text())['replaySha256'], 'abc')
        lines[1]['replaySha256'] = 'def'
        (root / 'status.jsonl').write_text('\n'.join(json.dumps(l) for l in lines) + '\n')
        with self.assertRaises(SystemExit):
            gate_xr.main(['timing', '--prereg', str(self.tmp / 'prereg.json'), '--runs', str(root),
                          '--out', str(self.tmp / 'timing2.json')])


# ---------------------------------------------------------------- run_timing

class TimingStatusTest(TmpCase):
    def setUp(self):
        super().setUp()
        prereg = make_prereg()
        prereg['sequences']['timing'] = {'fx-a': {}}
        (self.tmp / 'prereg.json').write_text(json.dumps(prereg))
        self.root = self.tmp / 'timing'
        self.lines = []
        for rep in (1, 2, 3):
            for arm in ('off', 'on'):
                rows = [{'i': i, 'srMapTrackerState': 1, 'srStageMs': {'mapTracker': 1.0}} for i in range(40)]
                write_run(self.root / 'fx-a', f'{rep}-{arm}', rows)
                self.lines.append({'sequence': 'fx-a', 'rep': rep, 'arm': arm, 'exit': 0, 'replaySha256': 'abc',
                                   'accuracy': '' if arm == 'off' else 'k=v'})
        self.n = 0

    def timing(self):
        self.n += 1
        (self.root / 'status.jsonl').write_text('\n'.join(json.dumps(l) for l in self.lines) + '\n')
        out = self.tmp / f'timing{self.n}.json'
        with contextlib.redirect_stdout(io.StringIO()):
            gate_xr.main(['timing', '--prereg', str(self.tmp / 'prereg.json'), '--runs', str(self.root),
                          '--out', str(out)])
        return json.loads(out.read_text())

    def test_clean_runs(self):
        t = self.timing()
        self.assertEqual((t['accuracyOff'], t['accuracyOn']), ('', 'k=v'))
        self.assertEqual((t['badRuns'], t['unknownExit']), ([], []))

    def test_bad_exit_and_stall_are_listed(self):
        self.lines[3]['exit'] = 139  # fx-a/2-on
        rows = [{'i': i, 'srStageMs': {'mapTracker': 1.0}} for i in range(40)]
        rows[0]['srStageMs'] = {'mapTracker': 1000.0}
        rows[1]['srStageMs'] = {'mapBuilder': 200.0}
        write_run(self.root / 'fx-a', '3-off', rows)
        t = self.timing()
        self.assertEqual(sorted(t['badRuns']), ['fx-a/2-on: exit 139', 'fx-a/3-off: stall 2'])

    def test_unknown_exit_when_status_line_missing(self):
        del self.lines[1]  # fx-a/1-on has no status line
        self.assertEqual(self.timing()['unknownExit'], ['fx-a/1-on'])

    def test_accuracy_none_without_status_and_error_on_disagreement(self):
        self.timing()
        (self.root / 'status.jsonl').unlink()
        out = self.tmp / 'nostatus.json'
        with contextlib.redirect_stdout(io.StringIO()):
            gate_xr.main(['timing', '--prereg', str(self.tmp / 'prereg.json'), '--runs', str(self.root),
                          '--out', str(out)])
        t = json.loads(out.read_text())
        self.assertIsNone(t['accuracyOn'])
        self.assertIsNone(t['accuracyOff'])
        self.assertEqual(len(t['unknownExit']), 6)
        self.lines[5]['accuracy'] = 'other'  # two ON lines with different env
        with self.assertRaises(SystemExit):
            self.timing()


class IdentityBinaryTest(TmpCase):
    def identity(self, status_lines):
        write_run(self.tmp / 'A', 'r', [{'i': 0, 'p': None}], {'bundle': 'x'})
        write_run(self.tmp / 'B', 'r', [{'i': 0, 'p': None}], {'bundle': 'x'})
        if status_lines is not None:
            (self.tmp / 'B' / 'status.jsonl').write_text('\n'.join(json.dumps(l) for l in status_lines) + '\n')
        out = self.tmp / 'id.json'
        with contextlib.redirect_stdout(io.StringIO()):
            gate_xr.main(['identity', '--e0-runs', str(self.tmp / 'A'), '--runs', str(self.tmp / 'B'),
                          '--out', str(out)])
        return json.loads(out.read_text())

    def test_binary_and_status_sha_recorded(self):
        res = self.identity([{'name': 'r', 'replaySha256': 'abc'}, {'name': 's', 'replaySha256': 'abc'}])
        self.assertEqual(res['replaySha256'], 'abc')
        self.assertEqual(res['statusSha256'], gate_xr.sha256_file(self.tmp / 'B' / 'status.jsonl'))

    def test_missing_status_gives_none(self):
        res = self.identity(None)
        self.assertIsNone(res['replaySha256'])
        self.assertIsNone(res['statusSha256'])

    def test_differing_binaries_are_an_error(self):
        with self.assertRaises(SystemExit):
            self.identity([{'name': 'r', 'replaySha256': 'abc'}, {'name': 's', 'replaySha256': 'def'}])


class RunTimingTest(TmpCase):
    def test_dry_run_plan(self):
        prereg = make_prereg()
        prereg['sequences']['timing'] = {'ob-a': {'envelope': 'fx-a'}, 'fx-a': {}}
        prereg['bundles'] = str(self.tmp / 'bundles')
        prereg['bundleOf'] = {'fx-a': 'scn/acc'}
        (self.tmp / 'prereg.json').write_text(json.dumps(prereg))
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf):
            code = run_timing.main(['--replay', '/bin/replay', '--prereg', str(self.tmp / 'prereg.json'),
                                    '--e0-bundles-root', '/src', '--accuracy', 'a=1', '--out',
                                    str(self.tmp / 'out'), '--cpus', '4,6,8', '--dry-run'])
        self.assertEqual(code, 0)
        lines = buf.getvalue().splitlines()
        self.assertEqual(len(lines), 12)
        ob = [l for l in lines if l.startswith('ob-a ')]
        fx = [l for l in lines if l.startswith('fx-a ')]
        self.assertEqual([l.split(':')[0].split()[2] for l in ob],
                         ['1-off', '1-on', '2-off', '2-on', '3-off', '3-on'])
        self.assertTrue(all(' cpu4 ' in l and 'taskset -c 4 -- /bin/replay' in l for l in ob))
        self.assertTrue(all(' cpu6 ' in l and 'taskset -c 6 -- /bin/replay' in l for l in fx))
        self.assertIn(f'--bundle {self.tmp / "bundles" / "ob-a"} --out {self.tmp / "out" / "ob-a" / "1-off"}', ob[0])
        self.assertIn('--bundle /src/scn/acc ', fx[0])
        for l in lines:
            arm = l.split(':')[0].split()[2].split('-')[1]
            self.assertEqual("SLAM_REFACT_ACCURACY='a=1'" in l, arm == 'on')
        self.assertFalse((self.tmp / 'out').exists())  # a dry run creates nothing

    def test_too_few_cpus_is_an_error(self):
        prereg = make_prereg()
        (self.tmp / 'prereg.json').write_text(json.dumps(prereg))
        with self.assertRaises(SystemExit):
            with contextlib.redirect_stderr(io.StringIO()):
                run_timing.main(['--replay', 'x', '--prereg', str(self.tmp / 'prereg.json'),
                                 '--e0-bundles-root', '/src', '--accuracy', 'a', '--out', str(self.tmp / 'o'),
                                 '--cpus', '4', '--dry-run'])

    def test_real_run_order_env_and_no_overwrite(self):
        prereg = make_prereg()
        prereg['sequences']['timing'] = {'fx-a': {}}
        prereg['bundleOf'] = {'fx-a': 'scn/acc'}
        (self.tmp / 'prereg.json').write_text(json.dumps(prereg))
        fake = self.tmp / 'fake_replay.py'
        fake.write_text("#!/usr/bin/env python3\nimport os, sys, pathlib\n"
                        "out = pathlib.Path(sys.argv[sys.argv.index('--out') + 1])\n"
                        "out.mkdir(parents=True)\n"
                        "(out / 'env.txt').write_text(os.environ.get('SLAM_REFACT_ACCURACY', 'unset'))\n")
        fake.chmod(0o755)
        shim = self.tmp / 'bin'
        shim.mkdir()
        (shim / 'taskset').write_text('#!/bin/sh\nshift 3\nexec "$@"\n')  # drop "-c CPU --"
        (shim / 'taskset').chmod(0o755)
        import os
        old = os.environ['PATH']
        os.environ['PATH'] = f'{shim}:{old}'
        self.addCleanup(os.environ.__setitem__, 'PATH', old)
        argv = ['--replay', str(fake), '--prereg', str(self.tmp / 'prereg.json'), '--e0-bundles-root', '/src',
                '--accuracy', 'a=1', '--out', str(self.tmp / 'out'), '--cpus', '3']
        with contextlib.redirect_stdout(io.StringIO()):
            self.assertEqual(run_timing.main(argv), 0)
            envs = [(self.tmp / 'out' / 'fx-a' / f'{r}-{a}' / 'env.txt').read_text()
                    for r in (1, 2, 3) for a in ('off', 'on')]
            self.assertEqual(envs, ['unset', 'a=1'] * 3)
            sha = run_timing.sha256_file(fake)
            acc = [json.loads(l)['accuracy'] for l in (self.tmp / 'out' / 'status.jsonl').read_text().splitlines()]
            self.assertEqual(acc, ['', 'a=1'] * 3)
            self.assertTrue(all(json.loads(l)['replaySha256'] == sha for l in
                                (self.tmp / 'out' / 'status.jsonl').read_text().splitlines()))
            lines = [json.loads(l) for l in (self.tmp / 'out' / 'status.jsonl').read_text().splitlines()]
            self.assertEqual([(l['rep'], l['arm'], l['exit'], l['cpu']) for l in lines],
                             [(r, a, 0, 3) for r in (1, 2, 3) for a in ('off', 'on')])
            self.assertEqual(run_timing.main(argv), 1)  # everything exists now: nothing overwritten, failure


class SummaryTest(VerdictBase):
    def test_summary_prints_the_new_tables(self):
        e0, cand = self.tmp / 'e0.json', self.tmp / 'k1.json'
        e0.write_text(json.dumps(self.e0))
        cand.write_text(json.dumps(self.cand))
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf):
            gate_xr.main(['summary', '--e0', str(e0), '--metrics', str(cand)])
        text = buf.getvalue()
        for head in ('outback / mapLost', 'forwardSpeed / mapLost', 'fixtures / mapLost',
                     'chromeForward / mapLost', 'fixtures / longestNonNormalRun',
                     'chromeForward / longestNonNormalRun', 'outback / path', 'outback / rotStartDeg'):
            self.assertIn(head, text)


if __name__ == '__main__':
    unittest.main()

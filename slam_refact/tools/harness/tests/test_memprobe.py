"""Unit tests of the original memory-probe evaluator (eval/memprobe.py) and its use in parity_report.py.

Synthetic probe runs and engine traces only (no evidence tree needed). Run (repo root):
  python3 -m unittest discover -s slam_refact/tools/harness/tests -v
"""
import gzip
import json
import pathlib
import sys
import tempfile
import unittest

HARNESS = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(HARNESS / 'eval'))

import gates  # noqa: E402
import memprobe as mp  # noqa: E402
import parity_report  # noqa: E402

REPO = HARNESS.parents[2]
sys.path.insert(0, str(REPO / 'scripts/dev'))
from reference_metadata import load_reference_metadata  # noqa: E402

FIELDS = ['seq', 'tMs', 'callMs', 'probeMs', 'root', 'rootScans', 'mbFrameCount', 'mbLastResetFrame', 'mbLastStateChangeFrame',
          'mbState', 'outMapReset', 'outKeyframeCreated', 'outNewPoints', 'trkState', 'trkFrameTrackerOk', 'trkConsecutiveOk',
          'trkCandidates', 'trkSky', 'frmWorldMatches', 'wm', 'kfSlots', 'kfFree', 'kfLive', 'wmKfSeq', 'wmLastKeyframeFrame',
          'newestKfIndex', 'newestKfCreatedFrame', 'pts1', 'pts2', 'pts3', 'pts4', 'pts5', 'newGround', 'newSky',
          'fftAll', 'fftGround', 'fftSky', 'fftLastOk', 'fftHeight', 'fftCalls']


def probe_record(seq, i, t, fc, kf_frames, reset_frames, live, sky_new=10, frm=0):
    """One synthetic record; keyframe/reset decided by frame_count fc; live = (type1, type2, type3)."""
    kf = fc in kf_frames
    reset = fc in reset_frames
    n_kf = sum(1 for f in kf_frames if f <= fc)
    r = dict.fromkeys(FIELDS)
    r.update(seq=seq, i=i, t=t, root=1000, mbFrameCount=fc, mbLastResetFrame=max([f for f in reset_frames if f <= fc], default=-1),
             outMapReset=int(reset), outKeyframeCreated=int(kf), outNewPoints=(5 + sky_new) if kf else 0, trkState=1 if n_kf else 0,
             frmWorldMatches=frm, kfSlots=n_kf, kfFree=0, kfLive=n_kf,
             pts1=[live[0], 0, live[0], 0], pts2=[live[1] + 1, 1, live[1], 0], pts3=[live[2], 0, live[2], 0], pts4=[0, 0, 0, 0],
             pts5=[0, 0, 0, 0], newGround=[5, 5] if kf else None, newSky=[sky_new, sky_new] if kf else None)
    return r


def write_probe_run(d, n, kf_frames, reset_frames, live, sky_new=10, frm=0, dt=0.05):
    d.mkdir(parents=True)
    header = {'schema': 'slam-refact-xr_reference-memprobe-v1', 'fields': FIELDS, 'scans': [], 'rootHistory': [], 'error': None}
    lines = [json.dumps(header)]
    rows = []
    for k in range(n):
        lines.append(json.dumps(probe_record(k + 1, k, k * dt, k + 1, kf_frames, reset_frames, live, sky_new, frm)))
        rows.append({'i': k, 't': k * dt, 'fresh': True, 'status': 'NORMAL', 'worldPoints': 0, 'wpAdded': 0})
    with gzip.open(d / 'memprobe.jsonl.gz', 'wt') as f:
        f.write('\n'.join(lines) + '\n')
    (d / 'trace.jsonl').write_text(''.join(json.dumps(r) + '\n' for r in rows))
    (d / 'meta.json').write_text('{}')


def write_engine_run(d, n, kf_frames, reset_frames, live, dt=0.05):
    d.mkdir(parents=True)
    rows = []
    for k in range(n):
        fc = k + 1
        rows.append({'i': k, 't': k * dt, 'fresh': True, 'srKeyframes': sum(1 for f in kf_frames if f <= fc),
                     'srKeyframeCreated': fc in kf_frames, 'srMapReset': fc in reset_frames,
                     'srPointsBySource': [live[0], live[1], live[2], 0, 0]})
    (d / 'trace.jsonl').write_text(''.join(json.dumps(r) + '\n' for r in rows))


class SchemaCompatibilityTest(unittest.TestCase):
    def test_current_and_original_headers_keep_records_unchanged(self):
        current = 'slam-refact-xr_reference-memprobe-v1'
        legacy = f"slam-refact-{load_reference_metadata(REPO)['main_repository_name']}-memprobe-v1"
        for schema in (current, legacy):
            with self.subTest(schema=schema), tempfile.TemporaryDirectory() as tmp:
                path = pathlib.Path(tmp)
                header = {'schema': schema, 'fields': ['seq']}
                with gzip.open(path / 'memprobe.jsonl.gz', 'wt') as f:
                    f.write(json.dumps(header) + '\n{"seq": 2}\n{"seq": 1}\n')
                actual_header, records = mp.load_probe(path)
                self.assertEqual(actual_header, header)
                self.assertEqual(records, [{'seq': 1}, {'seq': 2}])

    def test_other_versions_families_and_schema_types_are_rejected(self):
        current = 'slam-refact-xr_reference-memprobe-v1'
        legacy = f"slam-refact-{load_reference_metadata(REPO)['main_repository_name']}-memprobe-v1"
        invalid = [current[:-1] + '2', legacy[:-1] + '2', 'other-memprobe-v1', None, [], {}, 1]
        for schema in invalid:
            with self.subTest(schema=schema), tempfile.TemporaryDirectory() as tmp:
                path = pathlib.Path(tmp)
                with gzip.open(path / 'memprobe.jsonl.gz', 'wt') as f:
                    f.write(json.dumps({'schema': schema}) + '\n')
                with self.assertRaises(ValueError):
                    mp.load_probe(path)


class RuleTest(unittest.TestCase):
    def test_rel_or_diff(self):
        # centre 100, 25% -> tol 25 (run diff 2 -> 4)
        self.assertEqual(mp.rel_or_diff(125, 99, 101, 0.25)[0], 'PASS')
        self.assertEqual(mp.rel_or_diff(125.01, 99, 101, 0.25)[0], 'FAIL')
        self.assertEqual(mp.rel_or_diff(75, 99, 101, 0.25)[0], 'PASS')
        # run difference dominates: |p1-p2| = 40 -> tol 80 > 0.3*100
        v, info = mp.rel_or_diff(180, 80, 120, 0.30)
        self.assertEqual((v, info['tol']), ('PASS', 80.0))
        self.assertEqual(mp.rel_or_diff(181, 80, 120, 0.30)[0], 'FAIL')
        self.assertEqual(mp.rel_or_diff(None, 1, 1, 0.3)[0], 'NOT-APPLICABLE')
        self.assertTrue(mp.rel_or_diff(5, 18, 10, 0.25)[1]['non_discriminating_below'])
        self.assertFalse(mp.rel_or_diff(100, 99, 101, 0.25)[1]['non_discriminating_below'])
        # zero reference: tol 0 -> only exact
        self.assertEqual(mp.rel_or_diff(0, 0, 0, 0.3)[0], 'PASS')
        self.assertEqual(mp.rel_or_diff(1, 0, 0, 0.3)[0], 'FAIL')

    def test_reset_window(self):
        self.assertEqual(mp.reset_window(3, 1, 2)[0], 'PASS')
        self.assertEqual(mp.reset_window(0, 1, 2)[0], 'PASS')
        self.assertEqual(mp.reset_window(4, 1, 2)[0], 'FAIL')
        self.assertEqual(mp.reset_window(None, 1, 2)[0], 'NOT-APPLICABLE')


class MetricsTest(unittest.TestCase):
    def test_metrics_and_validation(self):
        with tempfile.TemporaryDirectory() as tmp:
            d = pathlib.Path(tmp) / 'p'
            # 30 s = frames with t <= 30 at dt 0.05 -> frame index <= 600 -> fc <= 601
            kfs = [10, 15, 20, 30, 600, 601, 700]
            write_probe_run(d, 800, kfs, [10, 601], (100, 50, 20), sky_new=12, frm=0)
            header, recs = mp.load_probe(d)
            rows = [json.loads(x) for x in (d / 'trace.jsonl').read_text().splitlines()]
            m = mp.run_metrics(recs, rows[0]['t'])
            self.assertEqual(m['map_resets'], 2)
            self.assertEqual(m['keyframes_created'], 7)
            self.assertEqual(m['keyframes_created_first_30s'], 6)   # fc 601 = i 600, t = 30.0 (<=)
            self.assertEqual(m['median_live_type_1_2_3'], [100, 50, 20])
            self.assertEqual(m['median_sky_created_per_keyframe'], 12)
            self.assertEqual(m['frm_world_matches_nonzero_frames'], 0)
            self.assertEqual(mp.keyframe_intervals(recs), [5, 5, 10, 570, 99])  # 600->601 ends on a reset: excluded
            v = mp.validate(recs, rows, header)
            c = v['checks']
            self.assertTrue(c['keyframe_interval_ge_5']['ok'])
            self.assertTrue(c['reset_implies_keyframe']['ok'])
            self.assertTrue(c['free_list_consistency']['ok'])
            self.assertTrue(c['frame_count_increments_per_call']['ok'])
            self.assertTrue(c['new_points_vs_out_vector']['ok'])
            self.assertFalse(c['first_reset_vs_first_world_points']['ok'])  # synthetic trace has no world points

    def test_interval_violation_detected(self):
        with tempfile.TemporaryDirectory() as tmp:
            d = pathlib.Path(tmp) / 'p'
            write_probe_run(d, 50, [5, 9, 20], [5], (1, 1, 1))
            _, recs = mp.load_probe(d)
            self.assertEqual(mp.keyframe_intervals(recs), [4, 11])
            v = mp.validate(recs, [{'i': 0, 't': 0, 'fresh': True, 'status': 'NORMAL'}], None)
            self.assertFalse(v['checks']['keyframe_interval_ge_5']['ok'])

    def test_time_attribution_of_unassociated_calls(self):
        recs = [{'t': None}, {'t': 1.0}, {'t': None}, {'t': 2.0}, {'t': None}]
        self.assertEqual(mp.assign_times(recs), [1.0, 1.0, 2.0, 2.0, None])

    def test_parse_args(self):
        self.assertEqual(sorted(mp.parse_memprobe_args(['a=x,y'])), ['a'])
        with self.assertRaises(SystemExit):
            mp.parse_memprobe_args(['a=x'])


class ReportTest(unittest.TestCase):
    """parity_report with and without --memprobe on synthetic engine/probe runs."""

    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        root = pathlib.Path(self.tmp.name)
        kfs = [10, 15, 20, 25, 30]
        write_probe_run(root / 'p1', 200, kfs, [10], (100, 50, 20))
        write_probe_run(root / 'p2', 200, kfs + [35], [10], (110, 50, 22))
        write_engine_run(root / 'eng', 200, kfs + [40, 45], [10, 45], (104, 80, 21))
        self.root = root
        th, self.digest = parity_report.load_thresholds()   # registered sha256 checked here
        # only the memory-probe gates: the synthetic engine trace carries no status/pose columns for the others
        self.th = dict(th, gates=[g for g in th['gates'] if g['id'] in ('G-M3', 'G-M4', 'G-M5', 'G-M7')])

    def tearDown(self):
        self.tmp.cleanup()

    def report(self, memprobe, supplied=None):
        runs = {'flip6-s2': str(self.root / 'eng')}
        return parity_report.build_report(self.th, self.digest, runs, {}, {}, supplied or {}, ['test'], memprobe)

    def by(self, report, gid, sc='flip6-s2'):
        return next(r for r in report['results'] if r['id'] == gid and r['scenario'] == sc)

    def test_without_memprobe_blocked_as_before(self):
        rep = self.report(None, {'G-M7': {'verdict': 'BLOCKED', 'reason': 'memory probe not available (registered blocked_if)'}})
        self.assertEqual(self.by(rep, 'G-M3')['verdict'], 'BLOCKED')
        self.assertEqual(self.by(rep, 'G-M3')['measured']['b']['verdict'], 'BLOCKED')
        self.assertEqual(self.by(rep, 'G-M4')['verdict'], 'BLOCKED')
        self.assertEqual(self.by(rep, 'G-M5')['verdict'], 'BLOCKED')
        g7 = [r for r in rep['results'] if r['id'] == 'G-M7']
        self.assertEqual(len(g7), 1)
        self.assertEqual((g7[0]['scenario'], g7[0]['verdict']), (None, 'BLOCKED'))  # from --module-results, unchanged
        self.assertNotIn('memprobe', rep['inputs'])

    def test_with_memprobe(self):
        probe = mp.parse_memprobe_args(['flip6-s2=%s,%s' % (self.root / 'p1', self.root / 'p2')])
        rep = self.report(probe, {'G-M7': {'verdict': 'BLOCKED', 'reason': 'x'}})
        g3 = self.by(rep, 'G-M3')
        # engine 7 vs reference 5, 6: centre 5.5, tol max(1.375, 2) = 2 -> |7 - 5.5| = 1.5 PASS
        self.assertEqual((g3['verdict'], g3['measured']['b']['engine'], g3['measured']['b']['tol']), ('PASS', 7, 2.0))
        g4 = self.by(rep, 'G-M4')
        # type 2: engine 80 vs 50, 50 -> tol 15 -> FAIL; types 1, 3 PASS
        self.assertEqual(g4['verdict'], 'FAIL')
        self.assertEqual([g4['measured']['type_%d' % k]['verdict'] for k in (1, 2, 3)], ['PASS', 'FAIL', 'PASS'])
        g5 = self.by(rep, 'G-M5')
        self.assertEqual((g5['verdict'], g5['measured']['engine'], g5['measured']['window']), ('PASS', 2, [0, 2]))
        g7 = [r for r in rep['results'] if r['id'] == 'G-M7']
        self.assertEqual(len(g7), 1)  # module result for G-M7 is superseded by the probe evaluation
        self.assertEqual((g7[0]['scenario'], g7[0]['verdict']), ('flip6-s2', 'PASS'))  # 21 vs 20, 22
        self.assertTrue(g7[0]['measured']['d7_zero_throughout'])
        self.assertEqual(g7[0]['measured']['original_sky_created_per_keyframe_median'], [10, 10])
        self.assertIn('flip6-s2', rep['inputs']['memprobe'])
        # other scenarios without a probe stay BLOCKED / NOT-RUN exactly as before
        self.assertEqual(self.by(rep, 'G-M5', 'tum-room1-full')['verdict'], 'NOT-RUN')


if __name__ == '__main__':
    unittest.main()

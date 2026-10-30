"""Unit tests of divergence/exact_input_report.py with synthetic traces, probe and bundles (no evidence tree).

Run (repo root):
  python3 -m unittest discover -s slam_refact/tools/harness/tests -v
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
sys.path.insert(0, str(HARNESS / 'divergence'))

import exact_input_report as eir  # noqa: E402


def row(i, p=(0.0, 2.0, 0.0), q=(1.0, 0.0, 0.0, 0.0), status='NORMAL', wp=0, by=None, fresh=True):
    r = {'i': i, 't': i * 0.033, 'fresh': fresh, 'status': status, 'p': list(p), 'q_wxyz': list(q), 'worldPoints': wp}
    if by is not None:
        r['srPointsBySource'] = list(by)
        r['srRepeated'] = False
    return r


def qz(deg):
    h = math.radians(deg) / 2
    return (math.cos(h), 0.0, 0.0, math.sin(h))


def probe_rec(i, live):
    r = {'i': i, 'seq': i + 2}
    for k in range(1, 6):
        r['pts%d' % k] = [live[k - 1] + 1, 1, live[k - 1], 0]
    return r


class CompareTest(unittest.TestCase):
    def test_identical(self):
        ref = [row(i) for i in range(5)]
        eng = [row(i, by=[0] * 5) for i in range(5)]
        r = eir.compare(eng, ref, {i: probe_rec(i, [0] * 5) for i in range(5)})
        self.assertEqual(r['frames_compared'], 5)
        self.assertEqual(set(r['first'].values()), {None})
        self.assertEqual(r['fraction_within']['pose_bitwise_equal'], 1.0)
        self.assertIsNone(r['probe_live_points']['first_mismatch_any_type'])

    def test_first_frames_per_threshold(self):
        ref = [row(i) for i in range(10)]
        eng = [row(i, by=[0] * 5) for i in range(10)]
        eng[2]['p'][0] += 1e-9            # bitwise only
        eng[4]['p'][0] += 2e-3            # pos
        eng[5]['q_wxyz'] = list(qz(0.1))  # rot
        eng[6]['status'] = 'LIMITED'
        eng[7]['worldPoints'] = 3
        eng[8]['p'][1] += 0.5e-3          # below pos threshold
        r = eir.compare(eng, ref)
        f = r['first']
        self.assertEqual(f['bitwise_pose_differs'], 2)
        self.assertEqual(f['pos_gt_1e-3'], 4)
        self.assertEqual(f['rot_gt_0.05deg'], 5)
        self.assertEqual(f['status_differs'], 6)
        self.assertEqual(f['world_points_differs'], 7)
        self.assertAlmostEqual(r['fraction_within']['pos_le_1e-3'], 0.9)
        self.assertAlmostEqual(r['max']['rot_deg'], 0.1, places=6)
        self.assertEqual(r['max']['dpos_i'], 4)
        self.assertNotIn('probe_live_points', r)

    def test_quaternion_sign_is_same_rotation(self):
        ref = [row(0, q=qz(10))]
        eng = [row(0, q=tuple(-x for x in qz(10)))]
        r = eir.compare(eng, ref)
        self.assertIsNone(r['first']['rot_gt_0.05deg'])
        self.assertEqual(r['first']['bitwise_pose_differs'], 0)

    def test_only_reference_fresh_frames_in_both(self):
        ref = [row(0), row(1, fresh=False, status='LIMITED'), row(2), row(3)]
        eng = [row(0), row(1), row(2)]
        r = eir.compare(eng, ref)
        self.assertEqual(r['frames_compared'], 2)
        self.assertEqual(r['ref_fresh_missing_in_engine'], 1)
        self.assertIsNone(r['first']['status_differs'])

    def test_probe_live_points_per_type(self):
        ref = [row(i) for i in range(6)]
        eng = [row(i, by=[0, 10, 0, 0, 0]) for i in range(6)]
        eng[3]['srPointsBySource'] = [0, 11, 0, 0, 0]
        eng[5]['srPointsBySource'] = [1, 10, 0, 0, 0]
        probe = {i: probe_rec(i, [0, 10, 0, 0, 0]) for i in range(6)}
        del probe[1]
        r = eir.compare(eng, ref, probe)['probe_live_points']
        self.assertEqual(r['frames_compared'], 5)
        self.assertEqual(r['per_type']['type2']['first_mismatch'], 3)
        self.assertEqual(r['per_type']['type1']['first_mismatch'], 5)
        self.assertEqual(r['first_mismatch_any_type'], 3)
        self.assertAlmostEqual(r['per_type']['type2']['fraction_equal'], 0.8)

    def test_compare_engines(self):
        a = [row(i, by=[0] * 5) for i in range(4)]
        b = [row(i, by=[0] * 5) for i in range(4)]
        self.assertTrue(eir.compare_engines(a, b)['identical'])
        b[2]['status'] = 'LIMITED'
        r = eir.compare_engines(a, b)
        self.assertFalse(r['identical'])
        self.assertEqual(r['first_any'], 2)
        self.assertEqual(r['first_by_key']['status'], 2)


class CliTest(unittest.TestCase):
    def test_end_to_end_without_replay(self):
        with tempfile.TemporaryDirectory() as td:
            td = pathlib.Path(td)
            ref = td / 'runs' / 'sc' / 'pmr3a'
            ref.mkdir(parents=True)
            (ref / 'trace.jsonl').write_text(''.join(json.dumps(row(i, wp=i)) + '\n' for i in range(4)))
            with gzip.open(ref / 'memprobe.jsonl.gz', 'wt') as f:
                f.write(json.dumps({'schema': 'slam-refact-xr_reference-memprobe-v1'}) + '\n')
                for i in range(4):
                    f.write(json.dumps(probe_rec(i, [0, i, 0, 0, 0])) + '\n')
            blist = []
            for name, wp_shift in (('pmr3a', 0), ('pmr3a-linacc0', 1)):
                b = td / 'bundles' / 'sc' / name
                b.mkdir(parents=True)
                (b / 'bundle.json').write_text(json.dumps({'diagnosticVariant': {'argv': ['x', '--capture', str(ref)]}}))
                (b / 'engine-config.tsv').write_text('pyramid_injection\tnone\npyramid_model\tswiftshader-approx\n')
                (b / 'frames.tsv').write_text('bootstrap\t-1\t1\t1\t0\t1\ta\t-\t0\n' +
                                              ''.join('frame\t%d\t%d\t%d\t0\t1\ta\t-\t%d\n' % (i, i + 2, i + 2, i + 3) for i in range(4)))
                rp = td / 'replays' / 'sc' / name
                rp.mkdir(parents=True)
                (rp / 'trace.jsonl').write_text(''.join(
                    json.dumps(row(i, wp=i + (wp_shift if i >= 2 else 0), by=[0, i, 0, 0, 0])) + '\n' for i in range(4)))
                blist.append(str(b))
            (td / 'list.txt').write_text('\n'.join(blist) + '\n')
            out = td / 'out'
            subprocess.check_call([sys.executable, '-I', str(HARNESS / 'divergence' / 'exact_input_report.py'), '--bundles',
                                   str(td / 'list.txt'), '--replays', str(td / 'replays'), '--out', str(out)],
                                  stdout=subprocess.DEVNULL)
            rep = json.loads((out / 'report.json').read_text())
            base, lin = rep['cases']
            self.assertTrue(base['frame_time_column'])
            self.assertIsNone(base['result']['first']['world_points_differs'])
            self.assertEqual(lin['result']['first']['world_points_differs'], 2)
            self.assertIsNone(base['result']['probe_live_points']['first_mismatch_any_type'])
            self.assertEqual(rep['linacc0_pairs'][0]['first_any'], 2)
            self.assertIn('REPORT only', (out / 'report.md').read_text())
            with self.assertRaises(subprocess.CalledProcessError):  # never overwrites a report
                subprocess.check_call([sys.executable, '-I', str(HARNESS / 'divergence' / 'exact_input_report.py'), '--bundles',
                                       str(td / 'list.txt'), '--replays', str(td / 'replays'), '--out', str(out)],
                                      stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


if __name__ == '__main__':
    unittest.main()

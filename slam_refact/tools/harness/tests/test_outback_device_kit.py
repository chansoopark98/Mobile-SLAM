"""Tests for slam_refact/tools/device/compare_keepmap_runs.py (device comparison kit analyzer): synthetic pose logs + a golden on the f1 device log."""
import contextlib
import io
import json
import os
import sys
import tempfile
import unittest

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..', '..'))
sys.path.insert(0, os.path.join(ROOT, 'slam_refact', 'tools', 'device'))
import compare_keepmap_runs as ck  # noqa: E402

F1 = os.path.join(ROOT, 'build', 'xr-tracker-diag-20261116', 'device', 'f1-metrics.json')
DT = 0.033


def quat_yaw(deg):
    import math
    h = math.radians(deg) / 2
    return {'x': 0, 'y': math.sin(h), 'z': 0, 'w': math.cos(h)}


def row(i, state=1, x=0.0, yaw=0.0, reset=False, kf=False, gyro=10.0, ev=1, lat=40.0, session=1, nkf=1):
    t_ms = 1000.0 + i * DT * 1e3
    return {'tPerfMs': t_ms, 'position': {'x': x, 'y': 0, 'z': 0}, 'rotation': quat_yaw(yaw),
            'source': {'engineReturnedAtMs': t_ms + lat, 'sourceStageTimeNanos': t_ms * 1e6},
            'tracker': {'session': session, 'sourceFrameIndex': i, 'mapTrackerState': state, 'mapReset': reset,
                        'keyframeCreated': kf, 'numKeyframes': nkf, 'motionEvents': ev, 'gyroPeakDegS': gyro if ev else None}}


def write(rows, **top):
    fd, path = tempfile.mkstemp(suffix='.json')
    with os.fdopen(fd, 'w') as fh:
        json.dump({'poseLog': {'schema': ck.SCHEMA, 'rows': rows}, **top}, fh)
    return path


class Base(unittest.TestCase):
    def run_one(self, rows, **top):
        p = write(rows, **top)
        self.addCleanup(os.remove, p)
        return ck.analyse(p)


class TestAnalyse(Base):
    def test_discards_exclude_first_map_and_gate(self):
        rows = [row(0, state=0), row(1, state=0, reset=True)]            # first map being built: reset, not counted
        rows += [row(i) for i in range(2, 6)]
        rows += [row(6, state=0, gyro=120.0), row(7, state=0, gyro=120.0), row(8, state=0, reset=True)]  # discard after fast rotation
        rows += [row(i) for i in range(9, 24)]
        rows += [row(24, state=0, gyro=20.0), row(25, state=0, reset=True)]  # discard, slow
        rows += [row(i) for i in range(26, 30)]
        r = self.run_one(rows)
        self.assertEqual(r['mapDiscards'], 2)
        self.assertEqual(r['discardsAfterGyroOver80'], 1)
        self.assertEqual(r['extensionKeyframes'], 0)

    def test_extension_keyframes_and_longest_run(self):
        rows = [row(i) for i in range(4)]
        rows += [row(4, state=0, kf=True), row(5, state=0, kf=True), row(6, state=0, kf=True), row(7, state=0), row(8, state=0, kf=True)]
        rows += [row(i) for i in range(9, 12)]
        rows += [row(12, state=0, kf=True, reset=True)]   # reset frame does not count as an extension
        rows += [row(13)]
        r = self.run_one(rows)
        self.assertEqual(r['extensionKeyframes'], 4)
        self.assertEqual(r['longestExtensionRun'], 3)

    def test_lost_episodes(self):
        rows = [row(0, state=0)] + [row(i) for i in range(1, 4)]
        rows += [row(i, state=0) for i in range(4, 6)]          # episode of 2
        rows += [row(i) for i in range(6, 8)]
        rows += [row(i, state=0) for i in range(8, 12)]         # episode of 4
        rows += [row(i) for i in range(12, 14)]
        r = self.run_one(rows)
        self.assertEqual(r['mapLostFrames'], 6)                  # frame 0 is before the first map
        self.assertEqual(r['lostEpisodes'], 2)
        self.assertEqual(r['episodeFrames'], {'median': 3.0, 'max': 4})
        self.assertAlmostEqual(r['episodeSeconds']['max'], 4 * DT, places=6)   # first lost frame to the recovery frame

    def test_latency_split_by_state(self):
        rows = [row(0), row(1, lat=30.0), row(2, lat=50.0), row(3, state=0, lat=200.0), row(4, state=0, lat=300.0), row(5, lat=40.0)]
        r = self.run_one(rows)
        self.assertEqual(r['latencyMs']['lost']['n'], 2)
        self.assertAlmostEqual(r['latencyMs']['lost']['p50'], 250.0, places=3)
        self.assertAlmostEqual(r['latencyMs']['lost']['max'], 300.0, places=3)
        self.assertEqual(r['latencyMs']['tracked']['n'], 4)
        self.assertAlmostEqual(r['latencyMs']['tracked']['max'], 50.0, places=3)

    def trajectory(self, lost_between):
        # reference view (yaw 0) for the first 2 s, look away (yaw 90), come back to yaw 2 deg with a known +0.7 offset
        rows, i = [], 0
        for _ in range(70):
            rows.append(row(i, x=0.0, yaw=0.0)); i += 1
        for k in range(40):
            rows.append(row(i, x=0.1, yaw=90.0, state=0 if (lost_between and 10 <= k < 14) else 1)); i += 1
        for _ in range(30):
            rows.append(row(i, x=0.7, yaw=2.0)); i += 1
        return rows

    def test_same_view_after_lost_episode(self):
        r = self.run_one(self.trajectory(True))
        sv = r['sameView']['all']
        self.assertEqual(sv['n'], 30)
        self.assertAlmostEqual(sv['median'], 0.7, places=6)
        self.assertAlmostEqual(sv['max'], 0.7, places=6)
        self.assertEqual(sum(t['n'] for t in r['sameView']['thirds']), 30)

    def test_same_view_without_lost_episode_not_counted(self):
        r = self.run_one(self.trajectory(False))
        self.assertEqual(r['sameView']['all']['n'], 0)
        self.assertIsNone(r['sameView']['all']['median'])

    def test_missing_tracker_rows_is_error(self):
        p = write([{'tPerfMs': 1.0, 'position': {'x': 0, 'y': 0, 'z': 0}, 'tracker': None}] * 5)
        self.addCleanup(os.remove, p)
        with self.assertRaises(ck.LogError):
            ck.analyse(p)
        err = io.StringIO()
        with contextlib.redirect_stderr(err), contextlib.redirect_stdout(io.StringIO()):
            self.assertNotEqual(ck.main([p]), 0)
        self.assertIn('ERROR', err.getvalue())

    def test_not_v4_is_error(self):
        fd, p = tempfile.mkstemp(suffix='.json')
        with os.fdopen(fd, 'w') as fh:
            json.dump({'poseLog': {'schema': 'old', 'rows': []}}, fh)
        self.addCleanup(os.remove, p)
        with self.assertRaises(ck.LogError):
            ck.analyse(p)

    def test_keepmap_flag_not_effective_warns(self):
        rows = [row(i) for i in range(6)]
        bad = {'available': True, 'k1KeepMapActive': False, 'allProductFlagsActive': False, 'ignoredKeys': ['accuracy.k1KeepMap'],
               'flags': {'enabled': {'sent': True, 'effective': True, 'ignored': False}, 'k1KeepMap': {'sent': True, 'effective': False, 'ignored': True}}}
        r = self.run_one(rows, engineMode='K1', engineReadback=bad)
        self.assertIn('k1KeepMap', r['warning'])
        out = '\n'.join(ck.render([r]))
        self.assertIn('WARNING', out)
        good = {'available': True, 'k1KeepMapActive': True, 'allProductFlagsActive': True, 'ignoredKeys': [],
                'flags': {'enabled': {'sent': True, 'effective': True, 'ignored': False}, 'k1KeepMap': {'sent': True, 'effective': True, 'ignored': False}}}
        r2 = self.run_one(rows, engineMode='K1', engineReadback=good)
        self.assertIsNone(r2['warning'])
        self.assertEqual(r2['activeFlags'], ['enabled', 'k1KeepMap'])
        r3 = self.run_one(rows, engineMode='K1F')   # keepmap requested, no read-back at all
        self.assertIsNotNone(r3['warning'])
        self.assertEqual(self.run_one(rows)['engineMode'], ck.NOT_RECORDED)


@unittest.skipUnless(os.path.exists(F1), 'f1-metrics.json (build/xr-tracker-diag-20261116) not available')
class TestGoldenF1(unittest.TestCase):
    def test_f1_numbers(self):
        r = ck.analyse(F1)
        self.assertEqual(r['mapDiscards'], 8)
        self.assertEqual(r['discardsAfterGyroOver80'], 8)
        self.assertEqual(r['extensionKeyframes'], 0)
        # The reference analyzer prints the engine counter "map tracker not ok 71 frames" (it also counts the 13 frames before the
        # first map was tracked); the lost frames after the first map are 58, as in f1-attribution.txt ("map-lost frames 58").
        self.assertEqual(r['mapLostFrames'], 58)
        with open(F1) as fh:
            fr, _ = ck.load_frames(json.load(fh))
        self.assertEqual(sum(1 for x in fr if x['mapTrackerState'] != 1), 71)


if __name__ == '__main__':
    unittest.main()

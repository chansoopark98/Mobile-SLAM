"""Contracts for the 2026-10-15 lean Flip6 capture tools (replay view, summary, replay-suite comparison)."""
import hashlib
import importlib.util
import json
import os
import subprocess
import sys
import tempfile
import unittest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DEV = os.path.join(ROOT, 'scripts/dev')
W, H = 4, 2


def write_capture(directory, frame_times, imu_times, results):
    os.makedirs(os.path.join(directory, 'mav0/cam0/data'))
    os.makedirs(os.path.join(directory, 'mav0/imu0'))
    frames = []
    for index, timestamp in enumerate(frame_times):
        gray = bytes([index] * (W * H))
        name = f'{index:05d}.pgm'
        with open(os.path.join(directory, 'mav0/cam0/data', name), 'wb') as handle:
            handle.write(f'P5\n{W} {H}\n255\n'.encode() + gray)
        frames.append({'index': index, 'timestamp': timestamp, 'requestId': index, 'clientEpoch': 2, 'file': name,
                       'sha256': hashlib.sha256(gray).hexdigest(), 'payloadStatus': 'saved',
                       'sourceTiming': {'pixelReadStartMs': 10.0, 'pixelReadEndMs': 12.5}})
    with open(os.path.join(directory, 'mav0/imu0/data.csv'), 'w') as handle:
        handle.write('#timestamp\n')
    recording = {'startedAtMs': 1000.0, 'stoppedAtMs': 2000.0, 'width': W, 'height': H, 'config': {'width': W, 'height': H},
                 'profile': 'mobile_default', 'frames': frames, 'acceptedFrameMetadata': [dict(f) for f in frames],
                 'imu': [[t, 0, 0, 9.8, 0, 0, 0] for t in imu_times], 'results': results,
                 'estimatorLifecycle': {'clientEpochAtStart': 2}, 'stopReason': 'duration',
                 'counters': {'acceptedFrames': len(frames), 'invalidPayload': 0, 'imuOverflow': 0, 'resultOverflow': 0},
                 'completion': {'partial': False, 'imuCoversImages': False}}
    with open(os.path.join(directory, 'record.json'), 'w') as handle:
        json.dump({'recorder': 'android-lean-record', 'completed': True, 'recording': recording}, handle)


def pose(x):
    return [1, 0, 0, x, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1]


class LeanToolsTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.capture = os.path.join(self.temp.name, 'capture')
        results = [{'clientEpoch': 1, 'initialized': True, 'statusCode': 2, 'arrivalMs': 1001, 'poseFresh': True, 'pose': pose(9)},
                   {'clientEpoch': 2, 'initialized': False, 'statusCode': 1, 'arrivalMs': 1100, 'poseFresh': False, 'pose': None,
                    'engineProcessingMs': 5, 'pipeline': {'totalMs': 5, 'frontendMs': 4, 'solver': {}}},
                   {'clientEpoch': 2, 'initialized': True, 'statusCode': 2, 'arrivalMs': 1500, 'poseFresh': True, 'pose': pose(0.0),
                    'engineProcessingMs': 20, 'pipeline': {'totalMs': 20, 'frontendMs': 8, 'backendMs': 12, 'solver': {'ceresMs': 9, 'currentVisualFactors': 30}}},
                   {'clientEpoch': 2, 'initialized': True, 'statusCode': 3, 'arrivalMs': 1600, 'poseFresh': False, 'pose': None},
                   {'clientEpoch': 2, 'initialized': True, 'statusCode': 2, 'arrivalMs': 1700, 'poseFresh': True, 'pose': pose(0.5),
                    'engineProcessingMs': 30}]
        # Frame 0 precedes the first IMU sample and frame 4 follows the last one.
        write_capture(self.capture, [0.10, 0.20, 0.30, 0.40, 0.50], [0.15, 0.25, 0.35, 0.45], results)

    def tearDown(self):
        self.temp.cleanup()

    def run_tool(self, *args):
        return subprocess.run([sys.executable, *args], capture_output=True, text=True, check=True).stdout

    def test_replay_view_keeps_contiguous_frames_inside_imu_span(self):
        view = os.path.join(self.temp.name, 'view')
        self.run_tool(os.path.join(DEV, 'lean-replay-view.py'), self.capture, view)
        with open(os.path.join(view, 'record.json')) as handle:
            recording = json.load(handle)['recording']
        self.assertEqual([f['timestamp'] for f in recording['frames']], [0.20, 0.30, 0.40])
        self.assertEqual([f['index'] for f in recording['frames']], [0, 1, 2])
        self.assertEqual([f['sourceIndex'] for f in recording['frames']], [1, 2, 3])
        self.assertEqual(recording['counters']['acceptedFrames'], 3)
        self.assertEqual(recording['replayView']['droppedLeading'], 1)
        self.assertEqual(recording['replayView']['droppedTrailing'], 1)
        # Same contract replay-android-input.mjs enforces: bracket and saved contiguous prefix.
        self.assertLessEqual(recording['imu'][0][0], recording['frames'][0]['timestamp'])
        self.assertGreaterEqual(recording['imu'][-1][0], recording['frames'][-1]['timestamp'])
        self.assertTrue(all(a['index'] == i and a['payloadStatus'] == 'saved' for i, a in enumerate(recording['acceptedFrameMetadata'])))
        source = os.stat(os.path.join(self.capture, 'mav0/cam0/data/00001.pgm'))
        linked = os.stat(os.path.join(view, 'mav0/cam0/data/00000.pgm'))
        self.assertEqual(source.st_ino, linked.st_ino)

    def test_replay_view_refuses_existing_output(self):
        view = os.path.join(self.temp.name, 'view')
        os.makedirs(view)
        with self.assertRaises(subprocess.CalledProcessError):
            self.run_tool(os.path.join(DEV, 'lean-replay-view.py'), self.capture, view)

    def test_summary_ignores_previous_epoch_and_reads_row_major_translation(self):
        summary = json.loads(self.run_tool(os.path.join(DEV, 'summarize-lean-record.py'), self.capture))
        self.assertEqual(summary['droppedPreviousEpochResults'], 1)
        self.assertEqual(summary['results'], 4)
        self.assertAlmostEqual(summary['firstInitializedS'], 0.5)
        self.assertEqual(summary['lostSegments'], [1])
        self.assertAlmostEqual(summary['freshPoseStepM']['max'], 0.5)
        self.assertAlmostEqual(summary['pixelReadMs']['p50'], 2.5)
        self.assertEqual(summary['pipeline']['beforeInit']['frames'], 1)
        self.assertEqual(summary['pipeline']['afterInit']['ceresMs']['max'], 9)


class SuiteCompareTest(unittest.TestCase):
    def setUp(self):
        spec = importlib.util.spec_from_file_location('suite', os.path.join(DEV, 'flip6-replay-suite.py'))
        self.suite = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(self.suite)
        self.base = {'freshPoseFrames': 100, 'maxStepM': 0.5, 'motionGuardResets': 0}

    def test_equal_run_has_no_flags(self):
        self.assertEqual(self.suite.compare(self.base, dict(self.base)), [])

    def test_regressions_are_flagged(self):
        flags = self.suite.compare(self.base, {'freshPoseFrames': 90, 'maxStepM': 1.0, 'motionGuardResets': 1})
        self.assertEqual(len(flags), 3)

    def test_small_step_noise_is_tolerated(self):
        self.assertEqual(self.suite.compare(self.base, {**self.base, 'maxStepM': 0.52}), [])


if __name__ == '__main__':
    unittest.main()

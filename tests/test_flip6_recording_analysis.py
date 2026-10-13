"""Independent arithmetic and data-boundary checks for historical recording analysis."""
import hashlib
import importlib.util
import json
import math
from pathlib import Path
import tempfile
import unittest


spec = importlib.util.spec_from_file_location('flip6_analysis',
    Path(__file__).resolve().parents[1] / 'scripts/dev/analyze-flip6-recordings.py')
analysis = importlib.util.module_from_spec(spec)
spec.loader.exec_module(analysis)


def row(timestamp, fresh=True, initialized=True, status=2, epoch=1, pose=None):
    if pose is None:
        pose = [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1]
    return dict(inputTimestamp=timestamp, poseTimestamp=timestamp, arrivalMs=timestamp * 1000 + 40,
                initialized=initialized, statusCode=status, poseFresh=fresh, poseValid=fresh,
                engineEpoch=epoch, clientEpoch=1, requestId=int(timestamp * 10), reason='test', pose=pose)


class Flip6RecordingAnalysisTests(unittest.TestCase):
    def test_linear_quantiles_and_missing_values(self):
        result = analysis.stats([0, 10, 20, 30, 40, None, math.nan, True])
        self.assertEqual(result['count'], 5)
        self.assertEqual(result['p50'], 20)
        self.assertEqual(result['p95'], 38)
        self.assertEqual(result['p99'], 39.6)
        self.assertIsNone(analysis.stats([])['p50'])

    def test_gyro_bound_clips_endpoints_and_rejects_missing_brackets(self):
        # Constant 90 deg/s produces exactly 45 degrees in half a second.
        imu = [[t, 0, 0, 0, 0, 0, math.pi / 2] for t in [0, 1, 2]]
        self.assertAlmostEqual(analysis.gyro_bound(imu, .25, .75), 45)
        self.assertAlmostEqual(analysis.gyro_bound(imu, 0, 2), 180)
        for a, b in [(-.1, 1), (0, 2.1), (1, 1), (1, 0)]:
            self.assertIsNone(analysis.gyro_bound(imu, a, b))

    def test_pose_pairs_require_fresh_and_same_client_and_engine(self):
        identity = [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1]
        moved = [0, -1, 0, 3, 1, 0, 0, 4, 0, 0, 1, 0, 0, 0, 0, 1]
        rows = [row(1, pose=identity), row(1.5, fresh=False, status=3, pose=moved), row(2, pose=moved),
                row(3, epoch=2, pose=identity)]
        result = analysis.pose_pairs(rows, [])
        self.assertEqual(result['pairs'], 1)
        self.assertEqual(result['adjacentPairs'], 0)
        result = analysis.pose_pairs([rows[0], rows[2]], [])['largestAdjacentTranslation'][0]
        self.assertEqual(result['translationM'], 5)
        self.assertAlmostEqual(result['rotationDeg'], 90)
        self.assertIsNone(result['recordedGyroAngleUpperBoundDeg'])
        rows[2]['clientEpoch'] = 2
        self.assertEqual(analysis.pose_pairs([rows[0], rows[2]], [])['pairs'], 0)

    def test_initialization_is_not_lost_and_client_epoch_is_not_engine_reset(self):
        rows = [row(1, fresh=False, initialized=False, status=1), row(2), row(3, fresh=False, status=3), row(4, epoch=2)]
        rows.append(dict(row(5, epoch=7), clientEpoch=2))
        result = analysis.summarize_rows(rows)
        self.assertEqual(result['nonFreshResults'], 2)
        self.assertEqual(result['lostResults'], 1)
        self.assertEqual(result['initializedCoverage'], .8)
        self.assertEqual(result['freshPoseCoverage'], .6)
        self.assertEqual(result['engineEpochChangeCount'], 1)
        self.assertEqual(result['clientEpochChangeCount'], 1)
        self.assertEqual(result['timingMs']['freshArrivalPoseAgeMs']['p99'], 40)
        self.assertEqual(result['timingMs']['workerQueueMs']['count'], 0)

    def test_true_flags_do_not_validate_missing_or_malformed_pose(self):
        valid = row(0)
        malformed = [dict(row(1), poseTimestamp=value) for value in [None, -1, math.nan, math.inf, True]]
        malformed.extend(dict(row(1), pose=value) for value in
                         [None, [], [0] * 15, [0] * 17, [0] * 15 + [math.nan], [0] * 15 + [True]])
        missing_pose, missing_timestamp = row(1), row(1)
        del missing_pose['pose']
        del missing_timestamp['poseTimestamp']
        malformed.extend([missing_pose, missing_timestamp])
        self.assertTrue(analysis.fresh(valid))
        for candidate in malformed:
            with self.subTest(candidate=candidate):
                self.assertFalse(analysis.fresh(candidate))
        result = analysis.summarize_rows([valid, *malformed])
        self.assertEqual(result['results'], 14)
        self.assertEqual(result['freshPoseResults'], 1)
        self.assertEqual(result['freshPoseCoverage'], 1 / 14)
        self.assertEqual(result['timingMs']['freshArrivalPoseAgeMs']['count'], 1)

    def make_record(self, directory, schema='mobile-slam-android-cdp-v1'):
        path = directory / 'record.json'
        record = dict(startedAtMs=1000, stoppedAtMs=3000,
                      results=[row(.9), row(1), row(2), row(3)],
                      imu=[[t, 0, 0, 9.81, 0, 0, 0] for t in [0, 1, 2, 3]])
        document = dict(schema=schema, completed=True, recording=record)
        path.write_text(json.dumps(document))
        return path, document

    def test_input_window_is_half_open_and_original_is_unchanged(self):
        with tempfile.TemporaryDirectory() as temp:
            path, _ = self.make_record(Path(temp))
            original = path.read_bytes()
            result = analysis.analyze(path)
            self.assertEqual(result['allResults']['results'], 4)
            self.assertEqual(result['inputTimeWindow']['results'], 2)
            self.assertEqual(result['inputTimeWindowResultsPerSecond'], 1)
            self.assertFalse(result['currentReplay'])
            self.assertEqual(path.read_bytes(), original)

    def test_completed_archive_still_reports_partial_pixels_and_verifies_payload(self):
        with tempfile.TemporaryDirectory() as temp:
            path, document = self.make_record(Path(temp), 'mobile-slam-android-input-v1')
            payload = b'\x01\x02'
            document['recording'].update(width=2, height=1,
                counters=dict(acceptedFrames=4, uncopiedFrames=3),
                frames=[dict(index=0, timestamp=1, clientEpoch=1, requestId=10, file='00000.pgm',
                             sha256=hashlib.sha256(payload).hexdigest())])
            path.write_text(json.dumps(document))
            image = path.parent / 'mav0/cam0/data/00000.pgm'
            image.parent.mkdir(parents=True)
            image.write_bytes(b'P5\n2 1\n255\n' + payload)
            report = analysis.analyze(path)
            self.assertTrue(report['completed'])
            self.assertTrue(report['images']['partialImageRecord'])
            self.assertEqual(report['images']['savedImageFractionOfAccepted'], .25)
            self.assertEqual(report['images']['payloadHashesVerified'], 1)
            self.assertEqual(report['images']['savedPrefixHistoricalResults']['results'], 1)
            image.write_bytes(b'P5\n2 1\n255\n\x00\x00')
            with self.assertRaisesRegex(ValueError, 'hash mismatch'):
                analysis.analyze(path)

    def test_invalid_first_imu_or_duplicate_timestamp_fails(self):
        with tempfile.TemporaryDirectory() as temp:
            path, document = self.make_record(Path(temp))
            document['recording']['imu'][0][1] = math.nan
            path.write_text(json.dumps(document))
            with self.assertRaisesRegex(ValueError, 'seven finite'):
                analysis.analyze(path)
            document['recording']['imu'][0][1] = 0
            document['recording']['imu'][1][0] = 0
            path.write_text(json.dumps(document))
            with self.assertRaisesRegex(ValueError, 'strictly increasing'):
                analysis.analyze(path)


if __name__ == '__main__':
    unittest.main()

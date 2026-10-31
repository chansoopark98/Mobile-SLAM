"""Independent analytic fixtures for P0 metrics; no datasets or SLAM engine needed."""
import copy
import json
import math
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import strict_accuracy as metrics  # noqa: E402


def zq(degrees):
    half = math.radians(degrees) / 2
    return [math.cos(half), 0, 0, math.sin(half)]


def fixture():
    points = [[0, 0, 0], [1, 0, 0], [1, 1, 0], [0, 1, 0]]
    rows = [{'source_id': i, 't': float(i), 'epoch': 0, 'fresh': True, 'status': 'NORMAL',
             'p': p, 'q_wxyz': [1, 0, 0, 0]} for i, p in enumerate(points)]
    gt = [[float(i), *p, 0, 0, 0, 1] for i, p in enumerate(points)]
    protocol = {
        'schema': 'slam-refact-xr-strict-protocol-v1',
        'clock_mapping_provenance': 'analytic shared seconds',
        'metric_scale_provenance': 'analytic metre coordinates',
        'ground_truth_provenance': 'independently specified square vertices',
        'ground_truth_kind': 'synthetic', 'physical_exposure_verified': False,
        'pose_convention': 'world_from_camera', 'pose_axes': 'right_handed', 'pose_units': 'metres',
        'trajectory_kind': 'published_online',
        'expected_frames': 4, 'source_schedule': [{'id': i, 't': float(i)} for i in range(4)],
        'association_max_dt_s': 0.006, 'source_timestamp_tolerance_s': 1e-9, 'rpe_tolerance_s': 0.05,
        'accepted_statuses': ['NORMAL'], 'T_body_camera': np.eye(4).tolist(),
        'alignment': {'type': 'fixed', 'T_gt_est': np.eye(4).tolist()},
        'timestamp_field': 'videoTimeS', 'timestamp_scale_to_gt': 1.0, 'timestamp_offset_to_gt_s': 0.0,
        'row_metadata': [{'source_id': i, 'fresh': True, 'epoch': 0} for i in range(4)],
    }
    return rows, gt, protocol


def recorder(rows):
    return {'schema': 'slam-refact-phone-metrics-v1', 'poseLog': {
        'schema': 'slam-refact-xr-poselog-v1', 'droppedRows': 0, 'nullUpdates': 0,
        'rows': [{'tPerfMs': 1000 * r['t'] + 80, 'videoTimeS': r['t'], 'frameTimeS': r['t'] + 7,
                  'status': r['status'], 'reason': 'UNSPECIFIED',
                  'position': dict(zip(('x', 'y', 'z'), r['p'])),
                  'rotation': dict(zip(('w', 'x', 'y', 'z'), r['q_wxyz'])), 'worldPoints': 42}
                 for r in rows]}}


def recorder_v2(rows):
    document = recorder(rows)
    log = document['poseLog']
    log.update(schema='slam-refact-xr-poselog-v2', meta={'position': 'responsive units; NOT metres'})
    for row, source in zip(log['rows'], rows):
        row.update(sourceFrameIndex=source['source_id'], repeatFrame=False)
        row['videoTimeS'] += 1 / 30  # Uploaded image is newer than this pose's source.
    return document


def recorder_v3(rows):
    document = recorder_v2(rows)
    document['poseLog']['schema'] = 'slam-refact-xr-poselog-v3'
    for row, original in zip(document['poseLog']['rows'], rows):
        t = original['t']
        row['frameTimeS'] += 1 / 30
        row['tPerfMs'] = (t + 7.08) * 1000
        row['source'] = {
            'kind': 'engine-response', 'hasPose': True, 'fresh': True,
            'engineFresh': True, 'newEngineResponse': True, 'repeated': False,
            'publicationRepeated': False, 'sourceMetadataFound': True,
            'sourceFrameIndex': original['source_id'], 'sessionEpoch': 0, 'currentSessionEpoch': 0,
            'sourceVideoTimeS': t, 'sourceFrameTimeS': t + 7,
            'sourceStageTimeNanos': (t + 7.01) * 1e9, 'sourceTimeNanos': int((t + 7) * 1e9),
            'callbackVideoTimeS': row['videoTimeS'], 'callbackFrameTimeS': row['frameTimeS'],
            'engineReturnedAtMs': (t + 7.06) * 1000,
        }
    return document


class StrictAccuracyTest(unittest.TestCase):
    def test_identical_poses(self):
        result = metrics.evaluate(*fixture())
        self.assertEqual(result['expected_frames'], 4)
        self.assertEqual(result['full_session_associated_coverage'], 1)
        self.assertEqual(result['ate_translation_m']['rmse'], 0)
        self.assertEqual(result['ate_rotation_deg']['rmse'], 0)
        self.assertEqual(result['rpe_1s']['pairs'], [[0, 1], [1, 2], [2, 3]])
        self.assertEqual(result['rpe_1s']['translation_m']['rmse'], 0)
        self.assertNotIn('passes', result)

    def test_single_se3_applied_to_position_and_rotation(self):
        rows, gt, protocol = fixture()
        for row in rows:
            x, y, z = row['p']
            row['p'] = [4 - y, 5 + x, 6 + z]
            row['q_wxyz'] = zq(90)
        protocol['alignment'] = {'type': 'fit_se3', 'source_ids': [0, 1, 2]}
        result = metrics.evaluate(rows, gt, protocol)
        self.assertLess(result['ate_translation_m']['rmse'], 1e-12)
        self.assertLess(result['ate_rotation_deg']['rmse'], 1e-10)
        self.assertEqual(result['alignment']['scale'], 1)

    def test_scale_error_cannot_be_fitted_away(self):
        rows, gt, protocol = fixture()
        for row in rows:
            row['p'] = [2 * x for x in row['p']]
        protocol['alignment'] = {'type': 'fit_se3', 'source_ids': [0, 1, 2, 3]}
        result = metrics.evaluate(rows, gt, protocol)
        self.assertAlmostEqual(result['ate_translation_m']['rmse'], math.sqrt(0.5), places=12)
        self.assertAlmostEqual(result['rpe_1s']['translation_m']['rmse'], 1, places=12)

    def test_attitude_error_not_removed_by_separate_rotation_fit(self):
        rows, gt, protocol = fixture()
        for row in rows:
            row['q_wxyz'] = zq(30)
        protocol['alignment'] = {'type': 'fit_se3', 'source_ids': [0, 1, 2, 3]}
        result = metrics.evaluate(rows, gt, protocol)
        self.assertAlmostEqual(result['ate_rotation_deg']['rmse'], 30, places=10)
        self.assertAlmostEqual(result['rpe_1s']['translation_m']['rmse'], 2 * math.sin(math.pi / 12), places=10)

    def test_rotation_ate_and_one_second_rpe(self):
        rows, gt, protocol = fixture()
        for i, row in enumerate(rows):
            row['q_wxyz'] = zq(30 * i)
        result = metrics.evaluate(rows, gt, protocol)
        self.assertAlmostEqual(result['ate_rotation_deg']['rmse'], math.sqrt(3150), places=10)
        self.assertAlmostEqual(result['rpe_1s']['rotation_deg']['rmse'], 30, places=10)

    def test_camera_body_rotation_and_moving_lever_arm(self):
        rows, gt, protocol = fixture()
        protocol['T_body_camera'] = [[0, -1, 0, .4], [1, 0, 0, .2], [0, 0, 1, .1], [0, 0, 0, 1]]
        for i, row in enumerate(rows):
            angle = i * math.pi / 2
            bx, by, bz = row['p']
            row['p'] = [bx + .4 * math.cos(angle) - .2 * math.sin(angle),
                        by + .4 * math.sin(angle) + .2 * math.cos(angle), bz + .1]
            row['q_wxyz'] = zq(90 * i + 90)
            q = zq(90 * i)
            gt[i][4:8] = [q[1], q[2], q[3], q[0]]
        result = metrics.evaluate(rows, gt, protocol)
        self.assertLess(result['ate_translation_m']['rmse'], 1e-12)
        self.assertLess(result['ate_rotation_deg']['rmse'], 1e-10)

    def test_gt_is_not_reused(self):
        rows, gt, protocol = fixture()
        duplicate_nearby = copy.deepcopy(rows[0])
        duplicate_nearby.update(source_id=10, t=.005)
        rows.insert(1, duplicate_nearby)
        protocol['source_schedule'].insert(1, {'id': 10, 't': .005})
        protocol['expected_frames'] = 5
        result = metrics.evaluate(rows, gt, protocol)
        self.assertEqual(result['associated_poses'], 4)
        self.assertEqual(result['unassociated_source_ids'], [10])
        self.assertEqual(result['full_session_associated_coverage'], .8)
        self.assertEqual(len({p['gt_index'] for p in result['associations']}), 4)

    def test_full_session_denominator_keeps_missing_lost_and_repeated(self):
        rows, gt, protocol = fixture()
        protocol['source_schedule'].extend([{'id': 4, 't': 4.0}, {'id': 5, 't': 5.0}])
        protocol['expected_frames'] = 6
        rows[1]['status'] = 'LIMITED'
        repeat = copy.deepcopy(rows[2]); repeat['fresh'] = False
        rows.insert(3, repeat)
        result = metrics.evaluate(rows, gt, protocol)
        self.assertEqual(result['full_session_pose_coverage'], .5)
        self.assertEqual(result['full_session_associated_coverage'], .5)
        self.assertEqual(result['missing_source_ids'], [4, 5])
        self.assertEqual([r['reason'] for r in result['excluded_rows']], ['status', 'repeated'])

    def test_epoch_jump_kept_in_global_alignment_and_rpe(self):
        rows, gt, protocol = fixture()
        for row in rows[2:]:
            row['epoch'] = 1
            row['p'][0] += 10
        result = metrics.evaluate(rows, gt, protocol)
        self.assertAlmostEqual(result['ate_translation_m']['rmse'], math.sqrt(50))
        self.assertAlmostEqual(result['rpe_1s']['translation_m']['rmse'], 10 / math.sqrt(3))

    def test_missing_rpe_is_explicitly_incomplete(self):
        rows, gt, protocol = fixture()
        for i, row in enumerate(rows):
            row['t'] *= .1; gt[i][0] *= .1; protocol['source_schedule'][i]['t'] *= .1
        result = metrics.evaluate(rows, gt, protocol)
        self.assertFalse(result['metric_complete'])
        self.assertIsNone(result['rpe_1s']['translation_m'])

    def test_source_duplicates_and_order_errors_rejected(self):
        for mode in ('duplicate', 'reverse', 'unknown', 'wrong_time', 'epoch_backwards'):
            rows, gt, protocol = fixture()
            if mode == 'duplicate': rows.insert(1, copy.deepcopy(rows[0]))
            if mode == 'reverse': rows.reverse()
            if mode == 'unknown': rows[0]['source_id'] = 99
            if mode == 'wrong_time': rows[1]['t'] += .1
            if mode == 'epoch_backwards': rows[0]['epoch'] = 1
            with self.subTest(mode=mode), self.assertRaises(ValueError):
                metrics.evaluate(rows, gt, protocol)

    def test_zero_pose_zero_association_and_bad_gt_rejected(self):
        for mode in ('no_rows', 'no_pose', 'no_association', 'duplicate_gt', 'reverse_gt'):
            rows, gt, protocol = fixture()
            if mode == 'no_rows': rows = []
            if mode == 'no_pose':
                for row in rows: row['p'] = row['q_wxyz'] = None
            if mode == 'no_association':
                for row in gt: row[0] += 20
            if mode == 'duplicate_gt': gt[1][0] = gt[0][0]
            if mode == 'reverse_gt': gt.reverse()
            with self.subTest(mode=mode), self.assertRaises(ValueError):
                metrics.evaluate(rows, gt, protocol)

    def test_invalid_calibration_and_provenance_rejected(self):
        for mode in ('count', 'units', 'clock', 'axes', 'scaled_extrinsic', 'invalid_q', 'bool_time'):
            rows, gt, protocol = fixture()
            if mode == 'count': protocol['expected_frames'] = 3
            if mode == 'units': protocol['pose_units'] = 'responsive'
            if mode == 'clock': protocol.pop('clock_mapping_provenance')
            if mode == 'axes': protocol['pose_axes'] = 'left_handed'
            if mode == 'scaled_extrinsic': protocol['T_body_camera'][0][0] = 2
            if mode == 'invalid_q': rows[0]['q_wxyz'] = [0, 0, 0, 0]
            if mode == 'bool_time': rows[0]['t'] = True
            with self.subTest(mode=mode), self.assertRaises(ValueError):
                metrics.evaluate(rows, gt, protocol)

    def test_preregistered_alignment_has_no_fallback(self):
        rows, gt, protocol = fixture()
        protocol['alignment'] = {'type': 'fit_se3', 'source_ids': [0, 1, 9]}
        with self.assertRaises(ValueError): metrics.evaluate(rows, gt, protocol)
        protocol['alignment']['source_ids'] = [0, 1, 2]
        for row in rows: row['p'] = [0, 0, 0]
        with self.assertRaisesRegex(ValueError, 'unobservable'): metrics.evaluate(rows, gt, protocol)

    def test_finite_inputs_that_overflow_are_rejected_without_dropping_residuals(self):
        for mode in ('ate_norm', 'body_transform', 'fit_covariance', 'summary_square'):
            rows, gt, protocol = fixture()
            if mode == 'ate_norm': rows[0]['p'] = [1e200, 0, 0]
            if mode == 'body_transform':
                rows[0]['p'] = [-1e308, 0, 0]
                protocol['T_body_camera'][0][3] = 1e308
            if mode == 'fit_covariance':
                protocol['alignment'] = {'type': 'fit_se3', 'source_ids': [0, 1, 2, 3]}
                for row, truth in zip(rows, gt):
                    row['p'] = [v * 1e200 for v in row['p']]
                    truth[1:4] = [v * 1e200 for v in truth[1:4]]
            if mode == 'summary_square':
                for row in rows: row['p'][0] += 1e154
            with self.subTest(mode=mode), self.assertRaisesRegex(ValueError, 'non-finite|overflow'):
                metrics.evaluate(rows, gt, protocol)

    def test_reader_uses_explicit_video_clock_not_callback_or_frame_clock(self):
        rows, gt, protocol = fixture()
        document = recorder(rows)
        document['poseLog']['droppedRows'] = 2
        parsed, diagnostic = metrics.phone_rows(document, protocol)
        self.assertEqual(parsed, rows)
        self.assertEqual(diagnostic['dropped_rows'], 2)
        self.assertEqual(metrics.evaluate(parsed, gt, protocol)['ate_translation_m']['rmse'], 0)
        protocol['timestamp_offset_to_gt_s'] = 4
        with self.assertRaisesRegex(ValueError, 'declared source frame'):
            metrics.evaluate(metrics.phone_rows(document, protocol)[0], gt, protocol)

    def test_reader_requires_explicit_row_identity_and_time_source(self):
        rows, _, protocol = fixture()
        document = recorder(rows)
        for mode in ('missing_metadata', 'metadata_length', 'callback', 'null_time', 'publication_order', 'cap'):
            p, d = copy.deepcopy(protocol), copy.deepcopy(document)
            if mode == 'missing_metadata': p.pop('row_metadata')
            if mode == 'metadata_length': p['row_metadata'].pop()
            if mode == 'callback': p['timestamp_field'] = 'tPerfMs'
            if mode == 'null_time': d['poseLog']['rows'][0]['videoTimeS'] = None
            if mode == 'publication_order': d['poseLog']['rows'][1]['tPerfMs'] = -1
            if mode == 'cap': d['poseLog']['cap'] = 3
            with self.subTest(mode=mode), self.assertRaises(ValueError): metrics.phone_rows(d, p)

    def test_cli_writes_receipt_and_rejects_unverified_log(self):
        rows, gt, protocol = fixture()
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            for name, value in [('recording', recorder(rows)), ('gt', gt), ('protocol', protocol)]:
                (root / f'{name}.json').write_text(json.dumps(value))
            cmd = [sys.executable, str(Path(metrics.__file__)), '--recording', str(root / 'recording.json'),
                   '--protocol', str(root / 'protocol.json'), '--ground-truth', str(root / 'gt.json')]
            completed = subprocess.run(cmd, text=True, capture_output=True)
            self.assertEqual(completed.returncode, 0, completed.stderr)
            result = json.loads(completed.stdout)
            self.assertEqual(result['assessment'], 'metrics_only_not_acceptance')
            self.assertEqual(len(result['inputs']['protocol']['sha256']), 64)
            truncated = recorder(rows)
            truncated['poseLog']['droppedRows'] = 1
            (root / 'recording.json').write_text(json.dumps(truncated))
            partial = subprocess.run(cmd, text=True, capture_output=True)
            self.assertEqual(partial.returncode, 2)
            self.assertFalse(json.loads(partial.stdout)['recording']['recording_complete'])
            protocol.pop('row_metadata')
            (root / 'protocol.json').write_text(json.dumps(protocol))
            rejected = subprocess.run(cmd, text=True, capture_output=True)
            self.assertEqual(rejected.returncode, 2)
            self.assertEqual(json.loads(rejected.stdout)['assessment'], 'UNVERIFIED')

    def test_cli_rejects_alignment_type_and_overflow_as_unverified(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            for mode in ('null_alignment', 'list_alignment', 'overflow'):
                rows, gt, protocol = fixture()
                if mode == 'null_alignment': protocol['alignment'] = None
                if mode == 'list_alignment': protocol['alignment'] = []
                if mode == 'overflow': rows[0]['p'] = [1e200, 0, 0]
                for name, value in [('recording', recorder(rows)), ('gt', gt), ('protocol', protocol)]:
                    (root / f'{name}.json').write_text(json.dumps(value))
                cmd = [sys.executable, str(Path(metrics.__file__)), '--recording', str(root / 'recording.json'),
                       '--protocol', str(root / 'protocol.json'), '--ground-truth', str(root / 'gt.json')]
                completed = subprocess.run(cmd, text=True, capture_output=True)
                with self.subTest(mode=mode):
                    self.assertEqual(completed.returncode, 2, completed.stderr)
                    self.assertEqual(json.loads(completed.stdout)['assessment'], 'UNVERIFIED')
                    self.assertNotIn('Traceback', completed.stderr)

    def test_v2_recognized_without_trusting_metric_claim_or_upload_timestamp(self):
        rows, _, protocol = fixture()
        for claimed_units in ('responsive units; NOT metres', 'metres'):
            document = recorder_v2(rows)
            document['poseLog']['meta']['position'] = claimed_units
            with self.subTest(claimed_units=claimed_units), self.assertRaisesRegex(ValueError, 'v2.*nonmetric.*pose-source'):
                metrics.phone_rows(document, protocol)
        with self.assertRaisesRegex(ValueError, 'row_metadata'):
            metrics.phone_rows(recorder_v2(rows), {})

    def test_v2_identity_epoch_and_schedule_contradictions_rejected(self):
        cases = {'mismatch': 'sourceFrameIndex', 'duplicate': 'unique', 'no_epoch': 'epoch',
                 'unknown_id': 'schedule', 'count': 'expected_frames', 'bad_repeat': 'repeatFrame'}
        for mode, error in cases.items():
            rows, _, protocol = fixture()
            document = recorder_v2(rows)
            raw = document['poseLog']['rows']
            if mode == 'mismatch': raw[0]['sourceFrameIndex'] = 1
            if mode == 'duplicate':
                raw[1]['sourceFrameIndex'] = 0
                protocol['row_metadata'][1]['source_id'] = 0
            if mode == 'no_epoch': protocol['row_metadata'][0].pop('epoch')
            if mode == 'unknown_id':
                raw[0]['sourceFrameIndex'] = 99
                protocol['row_metadata'][0]['source_id'] = 99
            if mode == 'count': protocol['expected_frames'] = 3
            if mode == 'bad_repeat': raw[0]['repeatFrame'] = 0
            with self.subTest(mode=mode), self.assertRaisesRegex(ValueError, error):
                metrics.phone_rows(document, protocol)

    def test_v2_cli_stays_unverified_with_unknown_source_and_repeat(self):
        rows, gt, protocol = fixture()
        document = recorder_v2(rows)
        document['poseLog']['rows'][0].update(sourceFrameIndex=None, repeatFrame=None)
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            for name, value in [('recording', document), ('gt', gt), ('protocol', protocol)]:
                (root / f'{name}.json').write_text(json.dumps(value))
            cmd = [sys.executable, str(Path(metrics.__file__)), '--recording', str(root / 'recording.json'),
                   '--protocol', str(root / 'protocol.json'), '--ground-truth', str(root / 'gt.json')]
            result = subprocess.run(cmd, text=True, capture_output=True)
            self.assertEqual(result.returncode, 2)
            answer = json.loads(result.stdout)
            self.assertEqual(answer['assessment'], 'UNVERIFIED')
            self.assertIn('pose-source', answer['error'])
            self.assertNotIn('ate_translation_m', answer)

    def test_v3_diagnostics_use_pose_source_clock_not_uploaded_clock(self):
        rows, _, protocol = fixture()
        document = recorder_v3(rows)
        protocol['timestamp_field'] = 'source.sourceVideoTimeS'
        with self.assertRaises(metrics.DiagnosticOnly) as caught:
            metrics.phone_rows(document, protocol)
        diag = caught.exception.diagnostics
        self.assertEqual(diag['fresh_source_count'], 4)
        self.assertEqual(diag['upload_source_time_different_rows'], 4)
        self.assertEqual(diag['source_timestamp_field'], 'source.sourceVideoTimeS')
        self.assertEqual(diag['position_units'], 'responsive_nonmetric')
        self.assertFalse(diag['physical_exposure_verified'])
        document['poseLog']['rows'][0]['source'].update(kind='callback-repeat', engineFresh=False, newEngineResponse=False)
        document['poseLog']['rows'][0]['repeatFrame'] = True
        with self.assertRaises(metrics.DiagnosticOnly) as caught:
            metrics.phone_rows(document, protocol)
        self.assertEqual(caught.exception.diagnostics['fresh_source_count'], 4)

    def test_v3_no_pose_unknown_and_old_epoch_are_not_fresh(self):
        rows, _, _ = fixture()
        document = recorder_v3(rows)
        raw = document['poseLog']['rows']
        raw[0]['source'] = None
        raw[1]['source'].update(kind='no-result', hasPose=False, fresh=False, repeated=None,
                                publicationRepeated=False, sourceMetadataFound=False,
                                sourceFrameIndex=None, sessionEpoch=None,
                                sourceVideoTimeS=None, sourceFrameTimeS=None,
                                sourceStageTimeNanos=None, sourceTimeNanos=None)
        raw[2]['source'].update(kind='inactive-held', fresh=False, currentSessionEpoch=1)
        raw[3]['source'].update(sessionEpoch=1, currentSessionEpoch=1)
        with self.assertRaises(metrics.DiagnosticOnly) as caught:
            metrics.phone_rows(document, {})
        self.assertEqual(caught.exception.diagnostics['fresh_source_count'], 1)
        for mode in ('no_pose', 'metadata_missing', 'old_epoch', 'repeated', 'publication_repeat'):
            document = recorder_v3(rows)
            source = document['poseLog']['rows'][0]['source']
            if mode == 'no_pose': source['hasPose'] = False
            if mode == 'metadata_missing': source['sourceMetadataFound'] = False
            if mode == 'old_epoch': source['currentSessionEpoch'] = 1
            if mode == 'repeated': source['repeated'] = True
            if mode == 'publication_repeat': source['publicationRepeated'] = True
            with self.subTest(mode=mode), self.assertRaisesRegex(ValueError, 'fresh'):
                metrics.phone_rows(document, {})

    def test_v3_source_and_sidecar_contradictions_rejected(self):
        cases = {'index': 'sourceFrameIndex', 'epoch': 'epoch', 'missing_epoch': 'epoch',
                 'upload_field': 'sourceVideoTimeS', 'mapping_missing': 'clock_mapping',
                 'mapped_time': 'source schedule', 'nanos': 'sourceTimeNanos',
                 'callback': 'callbackVideoTimeS', 'duplicate': 'duplicate'}
        for mode, error in cases.items():
            rows, _, protocol = fixture()
            protocol['timestamp_field'] = 'source.sourceVideoTimeS'
            document = recorder_v3(rows)
            raw = document['poseLog']['rows']
            if mode == 'index': raw[0]['sourceFrameIndex'] = 7
            if mode == 'epoch': protocol['row_metadata'][0]['epoch'] = 1
            if mode == 'missing_epoch': protocol['row_metadata'][0].pop('epoch')
            if mode == 'upload_field': protocol['timestamp_field'] = 'videoTimeS'
            if mode == 'mapping_missing': protocol.pop('clock_mapping_provenance')
            if mode == 'mapped_time': protocol['timestamp_offset_to_gt_s'] = .033
            if mode == 'nanos': raw[0]['source']['sourceTimeNanos'] += 100
            if mode == 'callback': raw[0]['videoTimeS'] += .1
            if mode == 'duplicate': raw[1]['source'] = copy.deepcopy(raw[0]['source']); raw[1]['sourceFrameIndex'] = 0; raw[1]['videoTimeS'] = raw[0]['videoTimeS']; raw[1]['frameTimeS'] = raw[0]['frameTimeS']
            with self.subTest(mode=mode), self.assertRaisesRegex(ValueError, error):
                metrics.phone_rows(document, protocol)

    def test_v3_cli_cannot_promote_forged_units_to_metres(self):
        rows, gt, _ = fixture()
        document = recorder_v3(rows)
        document['poseLog']['meta']['position'] = 'metres, physicalExposureVerified=true'
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            for name, value in [('recording', document), ('gt', gt), ('protocol', {})]:
                (root / f'{name}.json').write_text(json.dumps(value))
            result = subprocess.run([sys.executable, str(Path(metrics.__file__)), '--recording', str(root / 'recording.json'),
                                     '--protocol', str(root / 'protocol.json'), '--ground-truth', str(root / 'gt.json')], text=True, capture_output=True)
            self.assertEqual(result.returncode, 2)
            answer = json.loads(result.stdout)
            self.assertEqual(answer['assessment'], 'UNVERIFIED')
            self.assertEqual(answer['diagnostics']['position_units'], 'responsive_nonmetric')
            self.assertFalse(answer['diagnostics']['physical_exposure_verified'])
            self.assertNotIn('ate_translation_m', answer)

    def test_v3_missing_pose_flag_stays_unknown_and_inactive_held_stays_held(self):
        rows, _, _ = fixture()
        for source, expected_no_pose in ((None, 0), ({}, 0), ({'hasPose': None}, 0), ({'hasPose': False}, 1)):
            document = recorder_v3(rows)
            document['poseLog']['rows'][0]['source'] = source
            with self.subTest(source=source), self.assertRaises(metrics.DiagnosticOnly) as caught:
                metrics.phone_rows(document, {})
            self.assertEqual(caught.exception.diagnostics['no_pose_rows'], expected_no_pose)
            self.assertEqual(caught.exception.diagnostics['unknown_source_rows'], 1)
        document = recorder_v3(rows)
        document['poseLog']['rows'][0]['source'].update(kind='inactive-held', fresh=False, currentSessionEpoch=None)
        with self.assertRaises(metrics.DiagnosticOnly) as caught:
            metrics.phone_rows(document, {})
        self.assertEqual(caught.exception.diagnostics['old_epoch_rows'], 1)
        self.assertEqual(caught.exception.diagnostics['no_pose_rows'], 0)
        self.assertEqual(caught.exception.diagnostics['fresh_source_count'], 3)

    def test_v3_no_result_cannot_claim_pose_source_metadata_or_freshness(self):
        rows, _, _ = fixture()
        for mode in ('has_pose', 'metadata', 'fresh', 'source_tuple'):
            document = recorder_v3(rows)
            source = document['poseLog']['rows'][0]['source']
            source.update(kind='no-result', hasPose=False, fresh=False, sourceMetadataFound=False,
                          sourceFrameIndex=None, sessionEpoch=None, sourceVideoTimeS=None,
                          sourceFrameTimeS=None, sourceStageTimeNanos=None, sourceTimeNanos=None)
            if mode == 'has_pose': source['hasPose'] = True
            if mode == 'metadata': source['sourceMetadataFound'] = True
            if mode == 'fresh': source['fresh'] = True
            if mode == 'source_tuple': source['sourceFrameIndex'] = 0
            with self.subTest(mode=mode), self.assertRaisesRegex(ValueError, 'no-result'):
                metrics.phone_rows(document, {})


if __name__ == '__main__':
    unittest.main()

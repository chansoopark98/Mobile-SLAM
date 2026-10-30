#!/usr/bin/env python3
"""Analytic fixtures: 90-degree pose, known timing, corrupt/missing/empty captures."""
import copy
import json
import hashlib
from pathlib import Path
import tempfile
import unittest

import analyze_physical as analyzer


class PhysicalAnalyzerTests(unittest.TestCase):
    def fixture(self, root):
        run = root/'files'/'fixture'
        run.mkdir(parents=True)
        summary = {'event':'summary','runId':'fixture','mode':'playback','frames':3,'trackingFrames':2,
                   'savedImages':0,'imuEvents':0,'saveImages':False,'saveImu':False,'saveArcoreRecording':False,
                   'recordingBytes':0,'setupError':None,'cleanupError':None,'elapsedMonotonicNs':'2000000000'}
        frame = {'event':'frame','index':0,'timestampNs':'1000000000','arrivalElapsedNs':'5000000000',
                 'trackingState':'PAUSED','trackingFailureReason':'NONE','sessionUpdateDurationNs':'1000000',
                 'translationM':[1,2,3],'quaternionXyzw':[0,0,2**-.5,2**-.5],
                 'matrixColumnMajorT_W_C_OpenGL':[0,1,0,0,-1,0,0,0,0,0,1,0,1,2,3,1],
                 'pointCloudPoints':7}
        frames = []
        for i in range(3):
            row = copy.deepcopy(frame)
            row.update(index=i,timestampNs=str(1000000000+i*50000000),arrivalElapsedNs=str(5000000000+i*60000000),trackingState='TRACKING' if i else 'PAUSED')
            frames.append(row)
        (run/'poses.jsonl').write_text('\n'.join(json.dumps(r) for r in frames+[summary])+'\n')
        (run/'summary.json').write_text(json.dumps(summary))
        (run/'lifecycle.json').write_text(json.dumps({'sessionClosed':True,'sessionCleanupError':None,'elapsedMonotonicNs':'2100000000'}))
        self.manifest(root)
        return run

    def manifest(self, root):
        records = [{'path':str(p.relative_to(root)),'bytes':p.stat().st_size,'sha256':analyzer.sha(p)} for p in (root/'files').rglob('*') if p.is_file()]
        (root/'manifest.json').write_text(json.dumps({'exit':0,'runId':'fixture','fileCount':len(records),'files':records}))

    def test_known_pose_timing_tracking(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory);self.fixture(root)
            result,_,_=analyzer.read_capture(root)
            self.assertEqual(result['camera']['observedRateHz'],20)
            self.assertEqual(result['camera']['intervalMs']['p99'],50)
            self.assertAlmostEqual(result['arrival']['observedRateHz'],1000/60)
            self.assertEqual(result['trackingRatio'],2/3)
            self.assertEqual(result['firstTrackingSourceDelayS'],.05)
            self.assertEqual(result['lossEpisodesAfterTracking'],0)
            self.assertLess(result['publicPoseContract']['quaternionRotationMatrixMaxAbsDiff'],1e-14)

    def test_missing_file_and_corrupt_payload(self):
        for mode in ('missing','corrupt'):
            with self.subTest(mode=mode),tempfile.TemporaryDirectory() as directory:
                root=Path(directory);run=self.fixture(root)
                if mode=='missing':(run/'poses.jsonl').unlink()
                else:(run/'poses.jsonl').write_text('corrupt')
                with self.assertRaisesRegex(ValueError,'manifest'):
                    analyzer.read_capture(root)

    def test_zero_frames(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory);run=self.fixture(root)
            (run/'poses.jsonl').write_text('')
            self.manifest(root)
            with self.assertRaisesRegex(ValueError,'zero frames'):analyzer.read_capture(root)

    def test_invalid_quaternion_matrix(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory);run=self.fixture(root)
            rows=analyzer.rows_read(run/'poses.jsonl');rows[1]['quaternionXyzw']=[0,0,0,1]
            (run/'poses.jsonl').write_text('\n'.join(json.dumps(r) for r in rows))
            self.manifest(root)
            with self.assertRaisesRegex(ValueError,'pose contract'):analyzer.read_capture(root)

    def test_duplicate_timestamps(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory);run=self.fixture(root)
            rows=analyzer.rows_read(run/'poses.jsonl');rows[1]['timestampNs']=rows[0]['timestampNs']
            (run/'poses.jsonl').write_text('\n'.join(json.dumps(r) for r in rows));self.manifest(root)
            with self.assertRaisesRegex(ValueError,'timestamps'):analyzer.read_capture(root)

    def test_pgm_payload_and_summary_counts(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory);run=self.fixture(root)
            (run/'images').mkdir()
            rows=analyzer.rows_read(run/'poses.jsonl')
            payload=bytes([7,255])
            for row in rows[:3]:
                row.update(imageWidth=2,imageHeight=1,imageTimestampNs=row['timestampNs'],imagePayloadSha256=hashlib.sha256(payload).hexdigest())
                (run/'images'/f"{row['index']:06d}.pgm").write_bytes(b'P5\n2 1\n255\n'+payload)
            rows[-1].update(savedImages=3,saveImages=True)
            (run/'summary.json').write_text(json.dumps(rows[-1]))
            (run/'poses.jsonl').write_text('\n'.join(json.dumps(r) for r in rows));self.manifest(root)
            self.assertEqual(analyzer.read_capture(root)[0]['images']['verifiedPayloads'],3)
            (run/'images'/'000001.pgm').write_bytes(b'P5\n2 1\n255\n'+bytes([8,255]))
            self.manifest(root)
            with self.assertRaisesRegex(ValueError,'PGM payload'):analyzer.read_capture(root)

    def test_json_documents_and_rows_require_objects(self):
        for filename in ('manifest.json','summary.json','lifecycle.json','poses.jsonl','imu.jsonl'):
            for value in ([],7,None,'text'):
                with self.subTest(filename=filename,value=value),tempfile.TemporaryDirectory() as directory:
                    root=Path(directory);run=self.fixture(root)
                    target=root/filename if filename=='manifest.json' else run/filename
                    target.write_text(json.dumps(value)+'\n')
                    if filename!='manifest.json':self.manifest(root)
                    with self.assertRaisesRegex(ValueError,'JSON object required'):
                        analyzer.read_capture(root)

    def test_malformed_nonfinite(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory);run=self.fixture(root)
            (run/'poses.jsonl').write_text('{"event":NaN}')
            self.manifest(root)
            with self.assertRaisesRegex(ValueError,'nonfinite'):analyzer.read_capture(root)


if __name__=='__main__':unittest.main()

#!/usr/bin/env python3
"""Validate owned physical capture copies. Output is new local build evidence only."""
import argparse
import collections
import hashlib
import json
import math
import os
from pathlib import Path
import sys

from analyze import compare, stats

REPO = Path(__file__).resolve().parents[4]


def require(condition, message):
    if not condition:
        raise ValueError(message)


def sha(path):
    with path.open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


def json_read(path):
    value = json.loads(path.read_text(), parse_constant=lambda value: (_ for _ in ()).throw(ValueError('nonfinite JSON: ' + value)))
    require(isinstance(value, dict), 'JSON object required: '+str(path))
    return value


def rows_read(path):
    rows = [json.loads(line, parse_constant=lambda value: (_ for _ in ()).throw(ValueError('nonfinite JSON: ' + value)))
            for line in path.read_text().splitlines()]
    for index, row in enumerate(rows, 1):
        require(isinstance(row, dict), f'JSON object required: {path}:{index}')
    return rows


def timing(times):
    intervals = [(b-a)/1e6 for a, b in zip(times, times[1:])]
    duration = (times[-1]-times[0])/1e9 if len(times)>1 else 0
    mean = sum(intervals)/len(intervals) if intervals else 0
    return {'count': len(times), 'unique': len(set(times)), 'strictlyIncreasing': all(x>0 for x in intervals),
            'durationS': duration, 'observedRateHz': (len(times)-1)/duration if duration>0 else None,
            'intervalMs': stats(intervals), 'intervalStdDevMs': math.sqrt(sum((x-mean)**2 for x in intervals)/len(intervals)) if intervals else None}


def vector(row, key, size):
    values = row[key]
    require(len(values)==size and all(isinstance(v, (int,float)) and math.isfinite(v) for v in values), 'invalid finite vector: '+key)
    return values


def pose_checks(frames):
    errors = collections.defaultdict(list)
    for row in frames:
        p = vector(row, 'translationM', 3)
        q = vector(row, 'quaternionXyzw', 4)
        m = vector(row, 'matrixColumnMajorT_W_C_OpenGL', 16)
        norm = sum(x*x for x in q)
        require(norm>0, 'zero quaternion')
        x,y,z,w = [v/math.sqrt(norm) for v in q]
        rotation = [[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                    [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                    [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]]
        r = [[m[col*4+row] for col in range(3)] for row in range(3)]
        det = sum(r[0][i]*(r[1][(i+1)%3]*r[2][(i+2)%3]-r[1][(i+2)%3]*r[2][(i+1)%3]) for i in range(3))
        errors['translationMatrixMaxAbsDiffM'].append(max(abs(m[12+i]-p[i]) for i in range(3)))
        errors['quaternionNormSquaredMaxAbsError'].append(abs(norm-1))
        errors['quaternionRotationMatrixMaxAbsDiff'].append(max(abs(r[i][j]-rotation[i][j]) for i in range(3) for j in range(3)))
        errors['rotationOrthogonalityMaxAbsError'].append(max(abs(sum(r[k][i]*r[k][j] for k in range(3))-(i==j)) for i in range(3) for j in range(3)))
        errors['rotationDeterminantMaxAbsError'].append(abs(det-1))
        errors['homogeneousBottomRowMaxAbsError'].append(max(abs(m[i]-v) for i,v in zip((3,7,11,15),(0,0,0,1))))
    result = {key:max(values) for key,values in errors.items()}
    require(all(value<1e-4 for value in result.values()), 'public pose contract inconsistent: '+str(result))
    return result


def read_capture(capture):
    capture = capture.resolve()
    manifest = json_read(capture/'manifest.json')
    require(manifest['exit']==0, 'pull failed')
    run = capture/'files'/manifest['runId']
    require(run.resolve().is_relative_to(capture), 'run path outside capture')
    files = {str(p.relative_to(capture)):p for p in (capture/'files').rglob('*') if p.is_file()}
    declared = {r['path']:r for r in manifest['files']}
    require(len(declared)==len(manifest['files'])==manifest['fileCount'] and set(files)==set(declared), 'manifest file inventory mismatch')
    for name,path in files.items():
        require(path.resolve().is_relative_to(capture) and not path.is_symlink(), 'unsafe capture path')
        require(path.stat().st_size==declared[name]['bytes'] and sha(path)==declared[name]['sha256'], 'manifest bytes/hash mismatch: '+name)
    summary = json_read(run/'summary.json')
    lifecycle = json_read(run/'lifecycle.json')
    rows = rows_read(run/'poses.jsonl')
    frames = [row for row in rows if row.get('event')=='frame']
    require(frames, 'zero frames')
    require([r['index'] for r in frames]==list(range(len(frames))), 'frame index gap/duplicate')
    require([row for row in rows if row.get('event')=='summary']==[summary], 'embedded summary mismatch')
    imu = rows_read(run/'imu.jsonl') if (run/'imu.jsonl').exists() else []
    tracked = [r for r in frames if r['trackingState']=='TRACKING']
    images = [r for r in frames if 'imagePayloadSha256' in r]
    require((summary['frames'],summary['trackingFrames'],summary['savedImages'],summary['imuEvents'])==(len(frames),len(tracked),len(images),len(imu)), 'summary/row count mismatch')
    require(summary['setupError'] is None and summary['cleanupError'] is None, 'capture reports error')
    require(lifecycle['sessionClosed'] is True and lifecycle['sessionCleanupError'] is None, 'session cleanup failed')
    camera_times = [int(r['timestampNs']) for r in frames]
    arrival_times = [int(r['arrivalElapsedNs']) for r in frames]
    require(min(camera_times)>0 and timing(camera_times)['strictlyIncreasing'], 'camera timestamps not positive unique increasing')
    require(timing(arrival_times)['strictlyIncreasing'], 'arrival timestamps not increasing')
    require(all(int(r['sessionUpdateDurationNs'])>=0 for r in frames), 'negative API duration')
    require(int(lifecycle['elapsedMonotonicNs'])>=int(summary['elapsedMonotonicNs'])>0, 'lifecycle duration order invalid')
    image_files = {p.name for p in (run/'images').glob('*.pgm')}
    require(image_files=={f"{r['index']:06d}.pgm" for r in images}, 'image inventory mismatch')
    verified_images = {}
    for row in images:
        path = run/'images'/f"{row['index']:06d}.pgm"
        data = path.read_bytes()
        header = f"P5\n{row['imageWidth']} {row['imageHeight']}\n255\n".encode('ascii')
        require(data.startswith(header), 'invalid PGM header: '+path.name)
        payload = data[len(header):]
        require(len(payload)==row['imageWidth']*row['imageHeight'] and hashlib.sha256(payload).hexdigest()==row['imagePayloadSha256'], 'invalid PGM payload: '+path.name)
        require(int(row['imageTimestampNs']) not in verified_images, 'duplicate image timestamp')
        verified_images[int(row['imageTimestampNs'])] = sha(path)
    mp4 = run/'recording.mp4'
    require(summary['recordingBytes']==(mp4.stat().st_size if mp4.exists() else 0), 'MP4 size mismatch')
    require(summary['saveImages']==bool(images), 'saveImages flag/actual mismatch or zero images')
    require(summary['saveImu']==bool(imu), 'saveImu flag/actual mismatch or zero IMU')
    require(summary['saveArcoreRecording']==mp4.exists(), 'saveArcoreRecording flag/actual mismatch')
    require(not mp4.exists() or mp4.stat().st_size>0, 'zero MP4')
    sensors = collections.defaultdict(list)
    for row in imu:
        require(row['sensorType'] in (1,4), 'unexpected sensor type')
        vector(row, 'values', 3)
        sensors[row['sensorType']].append(row)
    sensor_results = {}
    for sensor,values in sensors.items():
        times = [int(r['timestampNs']) for r in values]
        require(min(times)>0 and timing(times)['strictlyIncreasing'], 'IMU timestamps not positive unique increasing')
        sensor_results[str(sensor)] = {'name': {1:'accelerometer_m_s2',4:'gyroscope_rad_s'}[sensor], **timing(times),
            'arrivalIntervals': timing([int(r['arrivalElapsedNs']) for r in values]),
            'arrivalMinusSensorTimestampMs': stats([(int(r['arrivalElapsedNs'])-int(r['timestampNs']))/1e6 for r in values])}
    if summary['saveImu']:
        require(set(sensors)=={1,4}, 'missing accelerometer or gyroscope')
    episodes = []
    for row in frames:
        if not episodes or episodes[-1]['state']!=row['trackingState']:
            episodes.append({'state':row['trackingState'],'firstFrame':row['index'],'lastFrame':row['index'],'firstTimestampNs':row['timestampNs'],'lastTimestampNs':row['timestampNs'],'frames':1})
        else:
            episodes[-1].update(lastFrame=row['index'],lastTimestampNs=row['timestampNs'],frames=episodes[-1]['frames']+1)
    for episode in episodes:
        episode['observedSpanS'] = (int(episode['lastTimestampNs'])-int(episode['firstTimestampNs']))/1e9
    first_tracking = next((i for i,e in enumerate(episodes) if e['state']=='TRACKING'),None)
    result = {'capture':str(capture),'validation':'PASS','manifestFileCountVerified':len(files),'summary':summary,'lifecycle':lifecycle,
        'lifecycleElapsedS':int(lifecycle['elapsedMonotonicNs'])/1e9,'finishElapsedS':int(summary['elapsedMonotonicNs'])/1e9,
        'trackingRatio':len(tracked)/len(frames),'firstTrackingFrame':tracked[0]['index'] if tracked else None,
        'firstTrackingSourceDelayS':(int(tracked[0]['timestampNs'])-camera_times[0])/1e9 if tracked else None,
        'lossEpisodesAfterTracking':sum(e['state']!='TRACKING' for e in episodes[first_tracking+1:]) if first_tracking is not None else None,
        'stateEpisodes':episodes,'trackingFailureReasons':dict(collections.Counter(r['trackingFailureReason'] for r in frames)),
        'camera':timing(camera_times),'arrival':timing(arrival_times),
        'sessionUpdateBlockingApiMs':stats([int(r['sessionUpdateDurationNs'])/1e6 for r in frames]),
        'arrivalMinusCameraTimestampMs':{'clockDomainAssumption':'same elapsed realtime timebase unproven; playback source timestamps are historical','values':stats([(a-t)/1e6 for a,t in zip(arrival_times,camera_times)])},
        'imageTimestampMinusFrameNs':stats([int(r['imageTimestampNs'])-int(r['timestampNs']) for r in images]),
        'imageTimestamps':timing([int(r['imageTimestampNs']) for r in images]),
        'images':{'verifiedPayloads':len(images),'unavailableReasons':dict(collections.Counter(r.get('imageUnavailable') for r in frames if 'imagePayloadSha256' not in r)),'dimensions':sorted({(r['imageWidth'],r['imageHeight']) for r in images})},
        'pointCloudCounts':stats([r['pointCloudPoints'] for r in frames]),'publicPoseContract':pose_checks(frames),'externalImu':sensor_results,
        'recordingSha256':sha(mp4) if mp4.exists() else None,'independentGroundTruth':'UNAVAILABLE','privateSolverTiming':'UNAVAILABLE',
        'externalImuProvenIdenticalToARCoreInternal':'UNVERIFIED'}
    return result,frames,verified_images


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('captures', nargs='+', type=Path)
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    output = args.output.resolve()
    canonical = Path(os.environ.get('MOBILE_SLAM_DATA_ROOT','/mnt/backup/SLAM')).resolve()
    if (not output.is_relative_to(REPO/'build') or output.is_relative_to(canonical)
            or any(output.is_relative_to(p.resolve()) for p in args.captures)):
        parser.error('output must be a new repository build path outside captures and canonical data')
    output.parent.mkdir(parents=True,exist_ok=True)
    # Reserve new destination before analysis: never overwrite successful or failed evidence.
    try:
        stream = output.open('x')
    except FileExistsError:
        parser.error('output already exists')
    try:
        data = [read_capture(p) for p in args.captures]
        result = {'validation':'PASS','analyzerSha256':sha(Path(__file__)),
                  'sharedAnalyzeSha256':sha(Path(__file__).with_name('analyze.py')),
                  'experiments':[d[0] for d in data],'comparisons':[]}
        for i in range(len(data)):
            for j in range(i+1,len(data)):
                comparison = compare(data[i][1],data[j][1])
                keys = data[i][2].keys() & data[j][2].keys()
                comparison.update(captureA=str(args.captures[i]),captureB=str(args.captures[j]),
                                  exactImageTimestampIntersection=len(keys),verifiedPgmByteHashEqual=sum(data[i][2][k]==data[j][2][k] for k in keys))
                image_rows_a = [dict(r, timestampNs=r['imageTimestampNs']) for r in data[i][1] if 'imageTimestampNs' in r]
                image_rows_b = [dict(r, timestampNs=r['imageTimestampNs']) for r in data[j][1] if 'imageTimestampNs' in r]
                comparison['imageTimestampAssociatedPoseAgreement'] = compare(image_rows_a, image_rows_b)
                comparison['imageTimestampAssociatedPoseAgreement']['association'] = 'Exact CPU image timestamps; paired public ARCore frame pose has measured image/frame timestamp offset, no interpolation'
                result['comparisons'].append(comparison)
        status = 0
    except (ValueError,KeyError,TypeError,OSError,ZeroDivisionError) as error:
        result = {'validation':'FAIL','error':str(error),'errorType':type(error).__name__}
        status = 1
    with stream:
        json.dump(result,stream,indent=2,allow_nan=False)
        stream.write('\n')
    print(json.dumps({'validation':result['validation'],'output':str(output),'error':result.get('error')}))
    return status


if __name__=='__main__':
    sys.exit(main())

#!/usr/bin/env python3
"""Read-only analysis of Android numeric observations and archived input prefixes.

Example: python3 scripts/dev/analyze-flip6-recordings.py RECORD.json ... --output NEW.json
The output describes historical observations, never current replay or GT accuracy.
Quantiles, PGM validation and gyro bounds follow the 2026-10-12 diagnosis scripts.
"""
import argparse
import bisect
import collections
import hashlib
import json
import math
from pathlib import Path


def finite(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def stats(values):
    values = sorted(v for v in values if finite(v))
    def quantile(p):
        if not values:
            return None
        i = (len(values) - 1) * p
        lo, hi = math.floor(i), math.ceil(i)
        return values[lo] + (i - lo) * (values[hi] - values[lo])
    return dict(count=len(values), min=values[0] if values else None,
                mean=sum(values) / len(values) if values else None,
                p50=quantile(.5), p95=quantile(.95), p99=quantile(.99),
                max=values[-1] if values else None)


def difference(a, b):
    return a - b if finite(a) and finite(b) else None


def nested(row, *keys):
    for key in keys:
        row = row.get(key) if isinstance(row, dict) else None
    return row


def fresh(row):
    pose = row.get('pose')
    timestamp = row.get('poseTimestamp')
    return bool(row.get('poseFresh') and row.get('poseValid')
                and finite(timestamp) and timestamp >= 0
                and isinstance(pose, list) and len(pose) == 16
                and all(finite(value) for value in pose))


def gyro_bound(imu, start, end):
    """Degrees: trapezoid upper bound on the piecewise-linear gyro norm integral."""
    if not imu or start < imu[0][0] or end > imu[-1][0] or end <= start:
        return None
    times = [sample[0] for sample in imu]
    def norm_at(t):
        index = bisect.bisect_right(times, t) - 1
        if index == len(imu) - 1:
            gyro = imu[index][4:7]
        else:
            a, b = imu[index:index + 2]
            weight = (t - a[0]) / (b[0] - a[0])
            gyro = [x * (1 - weight) + y * weight for x, y in zip(a[4:7], b[4:7])]
        return math.sqrt(sum(v * v for v in gyro))
    knots = [start, *times[bisect.bisect_right(times, start):bisect.bisect_left(times, end)], end]
    norms = [norm_at(t) for t in knots]
    return math.degrees(sum((b - a) * (x + y) / 2
                            for a, b, x, y in zip(knots, knots[1:], norms, norms[1:])))


def pose_pairs(rows, imu):
    valid = [(i, row) for i, row in enumerate(rows) if fresh(row)]
    pairs = []
    for (i, a), (j, b) in zip(valid, valid[1:]):
        if any(a.get(key) != b.get(key) for key in ['clientEpoch', 'engineEpoch']):
            continue
        start, end = a['poseTimestamp'], b['poseTimestamp']
        if not (finite(start) and finite(end) and end > start):
            continue
        pa, pb = a['pose'], b['pose']
        trace = sum(pa[4 * r + c] * pb[4 * r + c] for r in range(3) for c in range(3))
        pairs.append(dict(fromResult=i, toResult=j, dtMs=(end - start) * 1000,
            arrivalGapMs=difference(b.get('arrivalMs'), a.get('arrivalMs')),
            translationM=math.sqrt(sum((pa[k] - pb[k]) ** 2 for k in [3, 7, 11])),
            rotationDeg=math.degrees(math.acos(max(-1, min(1, (trace - 1) / 2)))),
            recordedGyroAngleUpperBoundDeg=gyro_bound(imu, start, end),
            nonFreshResultsBetween=j - i - 1, toReason=b.get('reason'),
            currentVisualFactors=nested(b, 'pipeline', 'solver', 'currentVisualFactors')))
    adjacent = [pair for pair in pairs if pair['nonFreshResultsBetween'] == 0]
    return dict(pairs=len(pairs), adjacentPairs=len(adjacent),
                maxFreshPoseTimestampGapMs=max((p['dtMs'] for p in pairs), default=None),
                maxFreshArrivalGapMs=max((p['arrivalGapMs'] for p in pairs
                                         if finite(p['arrivalGapMs'])), default=None),
                largestAdjacentTranslation=sorted(adjacent, key=lambda p: p['translationM'], reverse=True)[:5],
                largestAdjacentRotation=sorted(adjacent, key=lambda p: p['rotationDeg'], reverse=True)[:5],
                interpretation='Numerical self-consistency only; no verified exposure alignment or physical trajectory')


def episodes(rows, predicate, started_ms):
    groups = []
    for i, row in enumerate(rows):
        if predicate(row):
            if not groups or groups[-1][-1] != i - 1:
                groups.append([])
            groups[-1].append(i)
    return [dict(firstResult=g[0], lastResult=g[-1], results=len(g),
                 firstElapsedS=rows[g[0]]['inputTimestamp'] - started_ms / 1000,
                 lastElapsedS=rows[g[-1]]['inputTimestamp'] - started_ms / 1000,
                 reasons=dict(collections.Counter(rows[i].get('reason') for i in g))) for g in groups]


def summarize_rows(rows):
    n = len(rows)
    fresh_rows = [row for row in rows if fresh(row)]
    lost = [row for row in rows if row.get('statusCode') == 3]
    timing = {key: stats(row.get(key) for row in rows) for key in
              ['engineProcessingMs', 'workerProcessingMs', 'workerQueueMs', 'imuWaitMs', 'roundtripMs', 'frameCopyMs']}
    timing['freshArrivalPoseAgeMs'] = stats(difference(row.get('arrivalMs'), row['poseTimestamp'] * 1000)
        for row in fresh_rows if finite(row.get('poseTimestamp')))
    timing['freshUiPoseAgeMs'] = stats(difference(row.get('uiStateUpdatedAtMs'), row['poseTimestamp'] * 1000)
        for row in fresh_rows if finite(row.get('poseTimestamp')))
    timing['inputIntervalMs'] = stats((b['inputTimestamp'] - a['inputTimestamp']) * 1000
        for a, b in zip(rows, rows[1:]) if a.get('clientEpoch') == b.get('clientEpoch'))
    for key in ['imuMs', 'frontendMs', 'backendMs', 'totalMs']:
        timing[key] = stats(nested(row, 'pipeline', key) for row in rows)
    for key in ['ceresMs', 'marginalizationMs']:
        timing[key] = stats(nested(row, 'pipeline', 'solver', key) for row in rows)
    timing['pixelReadMs'] = stats(difference(nested(row, 'sourceTiming', 'pixelReadEndMs'),
        nested(row, 'sourceTiming', 'pixelReadStartMs')) for row in rows)
    timing['sourceToSubmitProxyMs'] = stats(difference(row.get('sentAtMs'), row['inputTimestamp'] * 1000) for row in rows)
    timing['arrivalToUiMs'] = stats(difference(row.get('uiStateUpdatedAtMs'), row.get('arrivalMs')) for row in rows)
    transitions = [dict(result=i, clientBefore=a.get('clientEpoch'), clientAfter=b.get('clientEpoch'),
                        engineBefore=a.get('engineEpoch'), engineAfter=b.get('engineEpoch'), reason=b.get('reason'))
                   for i, (a, b) in enumerate(zip(rows, rows[1:]), 1)
                   if any(a.get(key) != b.get(key) for key in ['clientEpoch', 'engineEpoch'])]
    return dict(results=n, initializedResults=sum(bool(row.get('initialized')) for row in rows),
                freshPoseResults=len(fresh_rows), initializedCoverage=sum(bool(row.get('initialized')) for row in rows) / n if n else None,
                freshPoseCoverage=len(fresh_rows) / n if n else None, lostResults=len(lost),
                nonFreshResults=n - len(fresh_rows), reasons=dict(collections.Counter(row.get('reason') for row in rows)),
                observedEpochTransitions=transitions,
                engineEpochChangeCount=sum(t['clientBefore'] == t['clientAfter'] and t['engineBefore'] != t['engineAfter'] for t in transitions),
                clientEpochChangeCount=sum(t['clientBefore'] != t['clientAfter'] for t in transitions),
                timingMs=timing, lostTrackerFeatures=stats(row.get('featureCount') for row in lost),
                lostSolver={key: stats(nested(row, 'pipeline', 'solver', key) for row in lost) for key in
                    ['currentVisualFactors', 'eligibleFeatures', 'immatureFeatures', 'imuFactors', 'rejectedImuFactors',
                     'invalidDepthFeatures', 'projectionRejects', 'ceresMs', 'marginalizationMs']},
                lostCurrentFactorCounts=dict(collections.Counter(str(nested(row, 'pipeline', 'solver', 'currentVisualFactors')) for row in lost)))


def image_integrity(path, recording):
    frames, imu = recording['frames'], recording['imu']
    header = f"P5\n{recording['width']} {recording['height']}\n255\n".encode()
    expected_size = recording['width'] * recording['height']
    previous = None
    for i, frame in enumerate(frames):
        if frame['index'] != i or not finite(frame['timestamp']) or (previous is not None and frame['timestamp'] <= previous):
            raise ValueError('Saved frame indices/timestamps must be contiguous and strictly increasing')
        previous = frame['timestamp']
        if Path(frame['file']).name != frame['file']:
            raise ValueError('Recorder image filename must be a basename')
        raw = (path.parent / 'mav0/cam0/data' / frame['file']).read_bytes()
        if not raw.startswith(header) or len(raw) != len(header) + expected_size:
            raise ValueError(f"Invalid PGM header/size: {frame['file']}")
        if hashlib.sha256(raw[len(header):]).hexdigest() != frame['sha256']:
            raise ValueError(f"PGM payload hash mismatch: {frame['file']}")
    bracketed = bool(frames and imu and imu[0][0] <= frames[0]['timestamp'] and imu[-1][0] >= frames[-1]['timestamp'])
    return dict(savedFrames=len(frames), payloadHashesVerified=len(frames), selectedPrefixHasPastFutureBrackets=bracketed,
                firstTimestampS=frames[0]['timestamp'] if frames else None,
                lastTimestampS=frames[-1]['timestamp'] if frames else None,
                savedDurationS=frames[-1]['timestamp'] - frames[0]['timestamp'] if frames else 0,
                sourceTimingCounts=dict(collections.Counter(nested(frame, 'sourceTiming', 'source') for frame in frames)),
                associationCounts=dict(collections.Counter(nested(frame, 'sourceTiming', 'association') for frame in frames)))


def analyze(path):
    path = Path(path).resolve()
    raw = path.read_bytes()
    document = json.loads(raw)
    if document.get('schema') not in ['mobile-slam-android-cdp-v1', 'mobile-slam-android-input-v1']:
        raise ValueError(f'Unsupported record schema: {path}')
    recording = document['recording']
    rows, imu = recording['results'], recording['imu']
    if any(len(sample) != 7 or not all(finite(v) for v in sample) for sample in imu):
        raise ValueError('IMU samples must contain seven finite numbers')
    if any(a[0] >= b[0] for a, b in zip(imu, imu[1:])):
        raise ValueError('IMU timestamps must be strictly increasing')
    if any(not finite(row.get('inputTimestamp')) for row in rows):
        raise ValueError('Result input timestamps must be finite')
    start, stop = recording['startedAtMs'], recording['stoppedAtMs']
    if not (finite(start) and finite(stop) and stop > start):
        raise ValueError('Recording duration must be finite and positive')
    duration = (stop - start) / 1000
    window = [row for row in rows if start <= row['inputTimestamp'] * 1000 < stop]
    report = dict(path=str(path), sha256=hashlib.sha256(raw).hexdigest(), schema=document['schema'],
                  evidence='historical_device_recording', currentReplay=False, completed=document.get('completed'),
                  durationS=duration, config=recording.get('config'), calibration=recording.get('calibration'),
                  allResults=summarize_rows(rows), inputTimeWindow=summarize_rows(window),
                  inputTimeWindowRule='startedAtMs <= inputTimestamp * 1000 < stoppedAtMs',
                  inputTimeWindowResultsPerSecond=len(window) / duration,
                  imu=dict(samples=len(imu), samplesPerRecordingSecond=len(imu) / duration,
                           sourceGapMs=stats((b[0] - a[0]) * 1000 for a, b in zip(imu, imu[1:])),
                           gyroNormDegS=stats(math.degrees(math.sqrt(sum(v * v for v in sample[4:7]))) for sample in imu)),
                  lostEpisodes=episodes(rows, lambda row: row.get('statusCode') == 3, start),
                  nonFreshEpisodes=episodes(rows, lambda row: not fresh(row), start),
                  poseSelfConsistency=pose_pairs(rows, imu),
                  recorderCounters=recording.get('counters', recording.get('dropped')),
                  observedTransportCounters={key: stats(nested(row, 'imuDrops', key) for row in rows) for key in
                      ['overflow', 'stale', 'capacity', 'invalid', 'order', 'reset', 'lossResets']},
                  limitations=['No independent ground truth or verified physical exposure/IMU alignment',
                               'Result coverage is not complete camera source-frame coverage',
                               'Epoch changes only describe observed results, not unrecorded lifecycle events',
                               'Stage quantiles cannot be added to derive total latency'])
    if document['schema'] == 'mobile-slam-android-input-v1':
        images = image_integrity(path, recording)
        accepted = recording['counters']['acceptedFrames']
        fresh_rows = [row for row in rows if fresh(row)]
        saved_keys = {(frame['clientEpoch'], frame['requestId']) for frame in recording['frames']}
        saved_rows = [row for row in rows if (row.get('clientEpoch'), row.get('requestId')) in saved_keys]
        images.update(acceptedFrames=accepted, uncopiedFrames=recording['counters']['uncopiedFrames'],
                      savedImageFractionOfAccepted=len(recording['frames']) / accepted if accepted else None,
                      acceptedMinusRecordedResults=accepted - len(rows),
                      acceptedMinusInputWindowResults=accepted - len(window),
                      resultCountCaveat='Counts alone cannot reconcile uncopied input identities or in-flight results',
                      partialImageRecord=len(recording['frames']) < accepted,
                      savedPrefixHistoricalResults=summarize_rows(saved_rows),
                      firstFreshElapsedS=fresh_rows[0]['inputTimestamp'] - start / 1000 if fresh_rows else None,
                      firstFreshWithinSavedImages=bool(fresh_rows and images['lastTimestampS'] is not None
                          and images['firstTimestampS'] <= fresh_rows[0]['inputTimestamp'] <= images['lastTimestampS']))
        report['images'] = images
        report['limitations'].extend(['Only saved accepted-frame prefix is replayable; later pixels are absent',
                                     'Fresh replay lifecycle cannot reproduce original estimator window or arrival/drop scheduling'])
    else:
        report['images'] = dict(savedFrames=0, partialImageRecord=True)
        report['limitations'].append('Numeric observation has no images or initial estimator window; sensor/image replay is impossible')
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('records', type=Path, nargs='+')
    parser.add_argument('--output', type=Path, required=True, help='New JSON output; existing evidence is never overwritten')
    args = parser.parse_args()
    if args.output.exists():
        parser.error('Output already exists; choose a new evidence path')
    report = dict(schema='mobile-slam-flip6-recording-analysis-v1', quantiles='linear interpolation at (n - 1) * p',
                  freshPoseValidation='poseFresh and poseValid; finite nonnegative poseTimestamp; 16 finite pose values',
                  records=[analyze(path) for path in args.records])
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open('x') as stream:
        json.dump(report, stream, indent=2, allow_nan=False)
        stream.write('\n')
    print(args.output.resolve())


if __name__ == '__main__':
    main()

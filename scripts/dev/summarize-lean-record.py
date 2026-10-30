#!/usr/bin/env python3
"""Summarize an android-lean-record.mjs capture (live phone behaviour) and, if given, its desktop replay report.

Writes <capture>/summary.json and prints it. Live numbers are phone measurements; replay numbers are desktop
re-execution of the same input and must not be reported as phone performance.
"""
import argparse
import json
import math
import os


def quantiles(values):
    values = sorted(v for v in values if isinstance(v, (int, float)) and math.isfinite(v))
    if not values:
        return None
    pick = lambda p: values[min(len(values) - 1, int(round((len(values) - 1) * p)))]
    return {'count': len(values), 'p50': pick(0.5), 'p95': pick(0.95), 'p99': pick(0.99), 'max': values[-1]}


def counts(rows, key):
    out = {}
    for row in rows:
        out[str(row.get(key))] = out.get(str(row.get(key)), 0) + 1
    return out


def lost_segments(rows):
    segments, start = [], None
    for index, row in enumerate(rows):
        lost = row.get('statusCode') == 3
        if lost and start is None:
            start = index
        if not lost and start is not None:
            segments.append(index - start); start = None
    if start is not None:
        segments.append(len(rows) - start)
    return segments


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('capture')
    parser.add_argument('--replay', help='replay-android-input.mjs report.json')
    args = parser.parse_args()
    report = json.load(open(os.path.join(args.capture, 'record.json')))
    recording = report['recording']
    imu, frames = recording['imu'], recording['frames']
    # Results still in flight from before the capture's reset belong to the previous client epoch.
    epoch = recording.get('estimatorLifecycle', {}).get('clientEpochAtStart')
    results = [row for row in recording['results'] if epoch is None or row.get('clientEpoch') == epoch]
    gaps = [(imu[i + 1][0] - imu[i][0]) * 1000 for i in range(len(imu) - 1)]
    span = frames[-1]['timestamp'] - frames[0]['timestamp']
    initialized = [row for row in results if row.get('initialized')]
    poses = [row['pose'] for row in results if row.get('poseFresh') and row.get('pose')]
    # Row-major 4x4: translation is elements 3, 7, 11.
    jumps = [math.dist(a[3:12:4], b[3:12:4]) for a, b in zip(poses, poses[1:]) if len(a) == 16 and len(b) == 16]
    pixel = [f['sourceTiming']['pixelReadEndMs'] - f['sourceTiming']['pixelReadStartMs'] for f in frames
             if f.get('sourceTiming') and isinstance(f['sourceTiming'].get('pixelReadStartMs'), (int, float))]
    summary = {
        'capture': os.path.abspath(args.capture), 'label': 'VERIFIED live phone (lean recorder)',
        'durationS': round((recording['stoppedAtMs'] - recording['startedAtMs']) / 1000, 2),
        'frames': len(frames), 'fps': round(len(frames) / span, 2) if span > 0 else None,
        'counters': recording['counters'], 'completion': recording['completion'],
        'imu': {'rows': len(imu), 'hz': round(len(imu) / (imu[-1][0] - imu[0][0]), 1) if len(imu) > 1 else None,
                'gapMs': quantiles(gaps), 'gapsOver100ms': sum(g > 100 for g in gaps), 'gapsOver300ms': sum(g > 300 for g in gaps),
                'source': recording.get('finalDiagnostics', {}).get('imu', {}).get('lastSample', {}).get('source'),
                'sourceError': recording.get('finalDiagnostics', {}).get('imu', {}).get('error')},
        'results': len(results), 'droppedPreviousEpochResults': len(recording['results']) - len(results), 'reasons': counts(results, 'reason'), 'statusCodes': counts(results, 'statusCode'),
        'freshPoseRatio': round(sum(bool(r.get('poseFresh')) for r in results) / len(results), 4) if results else None,
        'firstInitializedS': round((initialized[0]['arrivalMs'] - recording['startedAtMs']) / 1000, 2) if initialized else None,
        'lostSegments': lost_segments(results), 'engineEpochs': len({r.get('engineEpoch') for r in results}),
        'engineProcessingMs': quantiles([r.get('engineProcessingMs') for r in results]),
        'workerProcessingMs': quantiles([r.get('workerProcessingMs') for r in results]),
        'pixelReadMs': quantiles(pixel), 'freshPoseStepM': quantiles(jumps),
        'featureCount': quantiles([r.get('featureCount') for r in results]),
    }
    staged = [r for r in results if isinstance(r.get('pipeline'), dict)]
    if staged:
        def stages(rows):
            pick = lambda key, sub=None: quantiles([(r['pipeline'].get(sub) or {}).get(key) if sub else r['pipeline'].get(key) for r in rows])
            return {'frames': len(rows), **{k: pick(k) for k in ('totalMs', 'imuMs', 'frontendMs', 'backendMs')},
                    **{k: pick(k, 'solver') for k in ('ceresMs', 'marginalizationMs', 'currentVisualFactors', 'visualFactors',
                                                       'eligibleFeatures', 'immatureFeatures', 'invalidDepthFeatures')}}
        summary['pipeline'] = {'beforeInit': stages([r for r in staged if not r.get('initialized')]),
                               'afterInit': stages([r for r in staged if r.get('initialized')])}
    if args.replay:
        replay = json.load(open(args.replay))
        summary['replay'] = {'label': 'VERIFIED desktop replay (not phone performance)', 'report': os.path.abspath(args.replay),
                             'completed': replay.get('completed'), **{k: replay['summary'].get(k) for k in
                             ('completedFrames', 'freshPoseFrames', 'freshPoseCoverage', 'firstFreshPoseFrame', 'reasons',
                              'motionGuardResets', 'maxConsecutivePublishedTranslationM', 'engineProcessingMs')}}
    with open(os.path.join(args.capture, 'summary.json'), 'w') as handle:
        json.dump(summary, handle, indent=2)
        handle.write('\n')
    print(json.dumps(summary, indent=1))


if __name__ == '__main__':
    main()

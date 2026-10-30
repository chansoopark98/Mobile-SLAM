#!/usr/bin/env python3
"""Summarize controlled public-SDK experiments; no trajectory accuracy inference."""
import argparse
import hashlib
import json
import math
from pathlib import Path

ROOT = Path(__file__).resolve().parents[4]
parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--root', type=Path, default=ROOT / 'build/xr_reference-deep-20261014/runtime')
parser.add_argument('--output', type=Path)
args = parser.parse_args()
output = args.output or args.root / 'matrix-summary.json'
if output.exists():
    raise SystemExit(f'Refusing to overwrite evidence: {output}')


def stats(values):
    values = sorted(x for x in values if math.isfinite(x))
    if not values:
        return {'n': 0}
    return {'n': len(values), **{name: values[int((len(values) - 1) * q)]
            for name, q in [('min', 0), ('p50', .5), ('p95', .95), ('p99', .99), ('max', 1)]}}


def phase_metrics(rows):
    finite = [r for r in rows if r.get('position') and r.get('rotation')]
    positions = [[r['position'][axis] for axis in 'xyz'] for r in finite]
    result = {'freshCallbacks': len(rows), 'finitePoses': len(finite), 'tracking': {},
              'worldPoints': stats([r['worldPointCount'] for r in rows if r.get('worldPointCount') is not None])}
    for row in rows:
        state = f"{row.get('trackingStatus')}/{row.get('trackingReason')}"
        result['tracking'][state] = result['tracking'].get(state, 0) + 1
    if positions:
        result['firstPosition'] = positions[0]
        result['lastPosition'] = positions[-1]
        result['positionRange'] = [max(p[i] for p in positions) - min(p[i] for p in positions) for i in range(3)]
        result['positionStep'] = stats([math.dist(a, b) for a, b in zip(positions, positions[1:])])
    return result


matrix = {'schema': 'xr_reference-matrix-summary-v1', 'interpretation': [
    'Fresh means repeatFrame=false; public callback counts are not unique engine solves.',
    'Phase uses latest source publication, which can differ from the staged image underlying the pose.',
    'CPU previous staging and GPU current frame are offset; videoTime match is metadata provenance only.',
    'Position changes, finite poses, world points and NORMAL do not establish metric or physical accuracy.',
    'Draft moving-responsive-240 artifact is preserved but excluded because its frameTime statistic mixed seconds/milliseconds.',
], 'arms': []}
for result_path in sorted(args.root.glob('*-v2/result.json')):
    raw = result_path.read_bytes()
    result = json.loads(raw)
    observed = result.get('observed', {})
    fresh = [r for r in observed.get('rows', []) if r.get('repeatFrame') is False]
    summary = result.get('summary', {})
    arm = {'label': result['label'], 'status': result['status'],
           'path': str(result_path.relative_to(ROOT)), 'sha256': hashlib.sha256(raw).hexdigest(),
           'engineAssets': result.get('engineAssets'), 'inputSha256': result.get('input', {}).get('sha256'),
           'browserVersion': result.get('browserVersion'),
           'sourceFrames': summary.get('sourceFrames'), 'uniqueSourcePixels': summary.get('uniqueSourcePixels'),
           'submittedFrames':sum(1 for f in observed.get('frames',[]) if f.get('writerAcceptedWallMs') is not None and f.get('submitted') is not False) if observed else None,
           'skippedPublications':[{'index':f['index'],'reason':f.get('skippedReason')} for f in observed.get('frames',[]) if f.get('submitted') is False],
           'featureCallbacks':summary.get('featureCallbacks',0),
           'featureSample':observed.get('featureRows',[])[:1],
           'callbacks': summary.get('callbacks'), 'freshCallbacks': len(fresh) if observed else None,
           'uniqueVideoTimes': summary.get('uniqueVideoTimes'),
           'videoTimeMatchedPublications': summary.get('videoTimeMatchedPublications'),
           'fresh': phase_metrics(fresh), 'phases': {},
           'timings': {k: v for k, v in summary.items() if k.endswith('Ms')},
           'lifecycle': observed.get('events', []),
           'errors': {'harness': observed.get('errors'), 'page': result.get('pageErrors'),
                      'requestFailures': result.get('requestFailures'), 'waitError': result.get('waitError'),
                      'cleanup':result.get('cleanupError'), 'fatal': result.get('error')},
           'cleanup': {'tracks': summary.get('resourceCleanup'), 'browser': result.get('browserClosed'),
                       'server': result.get('serverClosed')},
           'network': {'externalBlocked': result.get('blockedExternal'),
                       'engine': [r for r in result.get('resources', {}).get('performanceEntries', []) if '/engine/' in r['name']],
                       'non200': [r for r in result.get('requests', []) if r['status'] != 200]},
           'heap': result.get('resources', {}).get('memory')}
    for phase in sorted({r.get('phase', 'unknown') for r in fresh}):
        arm['phases'][phase] = phase_metrics([r for r in fresh if r.get('phase', 'unknown') == phase])
    matrix['arms'].append(arm)
output.write_text(json.dumps(matrix, indent=2) + '\n')
print(json.dumps({'output': str(output), 'arms': len(matrix['arms'])}))

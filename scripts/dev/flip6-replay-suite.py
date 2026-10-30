#!/usr/bin/env python3
"""Replay every Flip6 fixture in an index with one engine loader and compare against the index baseline.

The index (JSON) lists replay-ready views made by lean-replay-view.py together with the live phone summary and a
baseline desktop replay. This is a debugging/A-B harness: it reports deltas per fixture and flags regressions
(fewer fresh poses, a larger published jump, new motion-guard resets). It does not decide project gates; desktop
replay is not phone performance, and live phone scheduling effects are only in the live summary.
"""
import argparse
import hashlib
import json
import os
import subprocess
import sys
from concurrent.futures import ThreadPoolExecutor

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))


def sha256(path):
    with open(path, 'rb') as handle:
        return hashlib.sha256(handle.read()).hexdigest()


def metrics(report):
    summary = report['summary']
    return {'completedFrames': summary.get('completedFrames'), 'freshPoseFrames': summary.get('freshPoseFrames'),
            'freshPoseCoverage': summary.get('freshPoseCoverage'), 'firstFreshPoseFrame': summary.get('firstFreshPoseFrame'),
            'motionGuardResets': summary.get('motionGuardResets'),
            'maxStepM': summary.get('maxConsecutivePublishedTranslationM'),
            'engineP95Ms': (summary.get('engineProcessingMs') or {}).get('p95'), 'reasons': summary.get('reasons')}


def run_one(fixture, args):
    view = os.path.join(ROOT, fixture['view'])
    record = os.path.join(view, 'record.json')
    if fixture.get('recordSha256') and sha256(record) != fixture['recordSha256']:
        return fixture['id'], {'error': 'fixture record.json hash mismatch'}
    output = os.path.join(args.output, fixture['id'])
    command = ['node', os.path.join(ROOT, 'scripts/dev/replay-android-input.mjs'), '--record', record,
               '--input-dir', os.path.join(view, 'mav0/cam0/data'), '--loader', args.loader,
               '--frames', '2000', '--output', output]
    with open(output + '.log', 'w') as log:
        code = subprocess.run(command, cwd=ROOT, stdout=log, stderr=subprocess.STDOUT, timeout=1800).returncode
    report_path = os.path.join(output, 'report.json')
    if code != 0 or not os.path.exists(report_path):
        return fixture['id'], {'error': f'replay exit {code}; see {output}.log'}
    return fixture['id'], metrics(json.load(open(report_path)))


def compare(baseline, current):
    flags = []
    if current['freshPoseFrames'] < baseline['freshPoseFrames']:
        flags.append(f"fresh {baseline['freshPoseFrames']}→{current['freshPoseFrames']}")
    if (current['maxStepM'] or 0) > (baseline['maxStepM'] or 0) * 1.05 + 0.01:
        flags.append(f"maxStep {baseline['maxStepM']:.3f}→{current['maxStepM']:.3f} m")
    if (current['motionGuardResets'] or 0) > (baseline['motionGuardResets'] or 0):
        flags.append(f"resets {baseline['motionGuardResets']}→{current['motionGuardResets']}")
    return flags


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--index', required=True)
    parser.add_argument('--loader', required=True, help='vio_engine.js to test (e.g. build/dev-wasm/vio_engine.js)')
    parser.add_argument('--output', required=True, help='new directory for this suite run')
    parser.add_argument('--only', nargs='*', help='fixture ids to run')
    parser.add_argument('--jobs', type=int, default=2, choices=range(1, 5))
    args = parser.parse_args()
    index = json.load(open(args.index))
    args.loader = os.path.abspath(args.loader)
    args.output = os.path.abspath(args.output)
    os.makedirs(args.output)
    fixtures = [f for f in index['fixtures'] if not args.only or f['id'] in args.only]
    with ThreadPoolExecutor(args.jobs) as pool:
        results = dict(pool.map(lambda f: run_one(f, args), fixtures))
    rows, regressions = [], 0
    for fixture in fixtures:
        current, baseline = results[fixture['id']], fixture.get('baselineReplay')
        flags = ['ERROR: ' + current['error']] if 'error' in current else (compare(baseline, current) if baseline else ['no baseline'])
        regressions += bool(flags and flags != ['no baseline'])
        rows.append({'id': fixture['id'], 'scenario': fixture['scenario'], 'live': fixture.get('live'),
                     'baseline': baseline, 'current': current, 'flags': flags})
    out = {'schema': 'mobile-slam-flip6-replay-suite-v1', 'index': os.path.abspath(args.index), 'indexSha256': sha256(args.index),
           'loader': args.loader, 'loaderSha256': sha256(args.loader), 'baselineLoaderSha256': index.get('baselineLoaderSha256'),
           'label': 'desktop replay of recorded phone input; not phone real-time performance', 'fixtures': rows}
    with open(os.path.join(args.output, 'suite.json'), 'w') as handle:
        json.dump(out, handle, indent=2)
    print(f"{'fixture':<14}{'live fresh':>11}{'base fresh':>11}{'cur fresh':>10}{'base step':>10}{'cur step':>9}  flags")
    for row in rows:
        b, c, l = row['baseline'] or {}, row['current'], row['live'] or {}
        print(f"{row['id']:<14}{l.get('freshPoseRatio', float('nan')):>11.3f}{b.get('freshPoseCoverage', float('nan')):>11.3f}"
              f"{c.get('freshPoseCoverage', float('nan')):>10.3f}{b.get('maxStepM') or 0:>10.3f}{c.get('maxStepM') or 0:>9.3f}  {'; '.join(row['flags'])}")
    print(f"suite: {os.path.join(args.output, 'suite.json')}; regressions flagged: {regressions}")
    sys.exit(1 if any('error' in r['current'] for r in rows) else 0)


if __name__ == '__main__':
    main()

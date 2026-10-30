#!/usr/bin/env python3
"""US-026 replay binary pin check (REPORT, dev bundles only).

Question: does the FREEZE-listed replay binary (or1pair native/harness/slam_refact_replay, sha256 24fdba2a...) replay
the US-026 lane A dev bundles exactly as the binary lane A used (b9bfb76b...) did? For every bundle in the list files it
runs the candidate binary into a NEW directory and compares the result with lane A's replay directory of the same bundle:

  trace.jsonl              every row equal after dropping the timing field `srStageMs` (wall-clock stage times)
  worldpoints-sampled.jsonl byte equal
  meta.json                `engineConfig`, `rows`, `processedFrames`, `droppedFrames`, `sensorRows`, `sensorRowsPushed`,
                           `injectedPyramidFrames`, `bootstrap` equal (wallSeconds is not compared)

Registration scenarios are refused (the list files must name dev bundles only: flip6-n1, flip6-arcore-s3-absolute).
Nothing existing is overwritten.

  python3 -I replay_pin_check.py --binary <slam_refact_replay> --lists <list>... --lane-a-replays <dir> --out <new dir> [--jobs 2]
"""
import argparse
import concurrent.futures as cf
import hashlib
import json
import pathlib
import subprocess
import sys
import time

DEV_SCENARIOS = {'flip6-n1', 'flip6-arcore-s3-absolute'}
META_KEYS = ('engineConfig', 'rows', 'processedFrames', 'droppedFrames', 'sensorRows', 'sensorRowsPushed',
             'injectedPyramidFrames', 'bootstrap')


def sha256(p):
    return hashlib.sha256(pathlib.Path(p).read_bytes()).hexdigest()


def rows(p):
    out = []
    for line in pathlib.Path(p).read_text().splitlines():
        if line.strip():
            r = json.loads(line)
            r.pop('srStageMs', None)
            out.append(r)
    return out


def replay(binary, bundle, out):
    out.parent.mkdir(parents=True, exist_ok=True)
    t0 = time.time()
    with open(str(out) + '.log', 'x') as log:
        rc = subprocess.call([str(binary), '--bundle', str(bundle), '--out', str(out)], stdout=log, stderr=subprocess.STDOUT)
    return rc, time.time() - t0


def compare(a_dir, b_dir):
    res = {}
    ra, rb = rows(a_dir / 'trace.jsonl'), rows(b_dir / 'trace.jsonl')
    first = next((k for k, (x, y) in enumerate(zip(ra, rb)) if x != y), None)
    res['traceRows'] = [len(ra), len(rb)]
    res['traceFirstDifferentRow'] = first
    res['traceEqual'] = len(ra) == len(rb) and first is None
    wa, wb = a_dir / 'worldpoints-sampled.jsonl', b_dir / 'worldpoints-sampled.jsonl'
    res['worldpointsEqual'] = (wa.exists() == wb.exists()) and (not wa.exists() or wa.read_bytes() == wb.read_bytes())
    ma, mb = json.loads((a_dir / 'meta.json').read_text()), json.loads((b_dir / 'meta.json').read_text())
    res['metaDifferentKeys'] = [k for k in META_KEYS if ma.get(k) != mb.get(k)]
    res['identical'] = res['traceEqual'] and res['worldpointsEqual'] and not res['metaDifferentKeys']
    return res


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--binary', type=pathlib.Path, required=True)
    ap.add_argument('--lists', type=pathlib.Path, nargs='+', required=True)
    ap.add_argument('--lane-a-replays', type=pathlib.Path, required=True)
    ap.add_argument('--out', type=pathlib.Path, required=True)
    ap.add_argument('--jobs', type=int, default=2)
    a = ap.parse_args()
    if not 1 <= a.jobs <= 4:
        raise SystemExit('--jobs 1..4')
    if a.out.exists():
        raise SystemExit('refusing to reuse ' + str(a.out))
    bundles = []
    for lst in a.lists:
        for line in lst.read_text().splitlines():
            if line.strip() and line.strip() not in bundles:
                bundles.append(line.strip())
    cases = []
    for b in bundles:
        bp = pathlib.Path(b)
        scen = bp.parent.name
        if scen not in DEV_SCENARIOS:
            raise SystemExit('refused: %s is not a dev bundle (scenario %s)' % (b, scen))
        cases.append({'bundle': bp, 'scenario': scen, 'name': bp.name, 'laneA': a.lane_a_replays / scen / bp.name,
                      'new': a.out / 'replays' / scen / bp.name})
    a.out.mkdir(parents=True)
    started = time.strftime('%Y-%m-%dT%H:%M:%S%z')
    with cf.ThreadPoolExecutor(a.jobs) as ex:
        futs = {ex.submit(replay, a.binary, c['bundle'], c['new']): c for c in cases}
        for f in cf.as_completed(futs):
            c = futs[f]
            c['rc'], c['seconds'] = f.result()
    results = []
    for c in cases:
        r = {'bundle': str(c['bundle']), 'laneAReplay': str(c['laneA']), 'newReplay': str(c['new']), 'rc': c['rc'],
             'seconds': round(c['seconds'], 1)}
        if c['rc'] == 0:
            r.update(compare(c['laneA'], c['new']))
        else:
            r['identical'] = False
        results.append(r)
    rep = {'schema': 'slam-refact-us026-replay-pin-check-v1', 'kind': 'REPORT (dev bundles only)', 'startedAt': started,
           'finishedAt': time.strftime('%Y-%m-%dT%H:%M:%S%z'), 'argv': sys.argv,
           'candidateBinary': {'path': str(a.binary), 'sha256': sha256(a.binary)},
           'comparison': 'trace.jsonl rows without srStageMs, worldpoints-sampled.jsonl bytes, meta.json keys %s' % (list(META_KEYS),),
           'bundles': len(results), 'identical': sum(1 for r in results if r['identical']),
           'allIdentical': all(r['identical'] for r in results), 'results': results}
    (a.out / 'pin-check.json').write_text(json.dumps(rep, indent=1) + '\n')
    print(json.dumps({k: rep[k] for k in ('candidateBinary', 'bundles', 'identical', 'allIdentical')}))
    return 0 if rep['allIdentical'] else 1


if __name__ == '__main__':
    sys.exit(main())

#!/usr/bin/env python3
"""US-026 dev REPORT (design §2.6 V1-V4): summarize poke.jsonl.gz + memprobe.jsonl.gz of ORIGINAL poke runs.

Per run: V1 poke kept (gap counts; validity rule §2.3-4: BA = no gap on calls with memprobe outKeyframeCreated = 1,
MSE = no gap on seq >= 2), V2 PV / cache-version history, V3 baLatch first seq vs first keyframe seq, V4 behaviour
(kfPoseChanged / pts1Changed sums, callMs on keyframe vs other calls, MSE residual == -1 count, mseSum > 600 calls,
VioTracker latch seq / scale). No pass/fail.

  python3 -I poke_summary.py --runs <run dir> [<run dir> ...] --out <new json>
"""
import argparse
import gzip
import json
import pathlib


def jsonl_gz(p):
    with gzip.open(p, 'rt') as f:
        return [json.loads(line) for line in f if line.strip()]


def q(xs, p):
    s = sorted(x for x in xs if x is not None)
    return s[min(len(s) - 1, int((len(s) - 1) * p))] if s else None


def summarize(d):
    pk = jsonl_gz(d / 'poke.jsonl.gz')
    head, recs = pk[0], pk[1:]
    mp = {r['seq']: r for r in jsonl_gz(d / 'memprobe.jsonl.gz')[1:]}
    meta = json.loads((d / 'meta.json').read_text())
    keys = head['keys']
    kf = [r for r in recs if (mp.get(r['seq']) or {}).get('outKeyframeCreated') == 1]
    other = [r for r in recs if (mp.get(r['seq']) or {}).get('outKeyframeCreated') == 0]
    gap_kf = [r['seq'] for r in kf if r['gap']]
    gap_ge2 = [r['seq'] for r in recs if r['seq'] >= 2 and r['gap']]
    hist = []
    for r in recs:
        st = (tuple(r['pre'] or []), tuple(r['post'] or []))
        if not hist or hist[-1][1] != st:
            hist.append((r['seq'], st))
    latch = next((r['seq'] for r in recs if r['baLatch'] == 1), None)
    vt = next((r for r in recs if r['vtLatch'] == 1), None)
    resid_m1 = [r['seq'] for r in recs if r['mseResidual'] == -1]
    sum600 = [r['seq'] for r in recs if (r['mseSum'] or 0) > 600]
    have = next((r['seq'] for r in recs if r['mseHave'] == 1), None)
    out = {
        'run': str(d), 'keys': keys, 'status': meta.get('status'), 'loadAvgAtStart': meta.get('loadAvgAtStart'),
        'wallSeconds': meta.get('wallSeconds'), 'inputFrames': meta['summary']['inputFrames'], 'fresh': meta['summary']['freshCallbacks'],
        'wrapped': head['wrapped'], 'error': head['error'], 'wasmSha256': head['wasmSha256'], 'records': len(recs), 'memprobeRecords': len(mp),
        'mseCheck': head['mseCheck'],
        'V1': {'gapCallsTotal': sum(1 for r in recs if r['gap']), 'gapOnKeyframeCalls': len(gap_kf), 'gapSeqGe2': len(gap_ge2),
               'keyframeCalls': len(kf),
               'ruleBA_ok': ('ba' not in keys) or not gap_kf, 'ruleMSE_ok': ('mse' not in keys) or not gap_ge2},
        'V2': {'prePostLayout': head['prePostLayout'], 'history': [[s, list(a), list(b)] for s, (a, b) in hist],
               'pvValues': sorted({r['post'][0] for r in recs if r['post']})},
        'V3': {'baLatchFirstSeq': latch, 'firstKeyframeSeq': kf[0]['seq'] if kf else None,
               'firstKeyframeFrame': kf[0]['i'] if kf else None},
        'V4': {'kfPoseChangedSum': sum(r['kfPoseChanged'] or 0 for r in recs),
               'callsWithKfPoseChanged': sum(1 for r in recs if r['kfPoseChanged']),
               'firstKfPoseChangedSeq': next((r['seq'] for r in recs if r['kfPoseChanged']), None),
               'firstKfPoseChangedFrame': next((r['i'] for r in recs if r['kfPoseChanged']), None),
               'kfPoseMaxDt': max((r['kfPoseMaxDt'] or 0 for r in recs), default=0),
               'kfPoseChangedOnNonKeyframeCalls': sum(r['kfPoseChanged'] or 0 for r in other),
               'pts1ChangedSum': sum(r['pts1Changed'] or 0 for r in recs),
               'pts1ChangedOnKeyframeCalls': sum(r['pts1Changed'] or 0 for r in kf),
               'callMsKeyframe': {'n': len(kf), 'p50': q([r['callMs'] for r in kf], .5), 'p95': q([r['callMs'] for r in kf], .95), 'max': q([r['callMs'] for r in kf], 1)},
               'callMsOther': {'n': len(other), 'p50': q([r['callMs'] for r in other], .5), 'p95': q([r['callMs'] for r in other], .95)},
               'mseResidualMinus1Calls': len(resid_m1), 'firstResidualMinus1Seq': resid_m1[0] if resid_m1 else None,
               'mseSumGt600Calls': len(sum600), 'firstMseSumGt600Seq': sum600[0] if sum600 else None,
               'firstMseHaveSeq': have,
               'vtLatchSeq': vt['seq'] if vt else None, 'vtLatchFrame': vt['i'] if vt else None,
               'vtLatchedScale': vt['vtLatchedScale'] if vt else None,
               'mseScaleLast': recs[-1]['mseScale'], 'mseResidualLast': recs[-1]['mseResidual'],
               'mseResidualDistinct': len({r['mseResidual'] for r in recs if r['mseResidual'] is not None})},
    }
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--runs', type=pathlib.Path, nargs='+', required=True)
    ap.add_argument('--out', type=pathlib.Path, required=True)
    a = ap.parse_args()
    if a.out.exists():
        raise SystemExit('refusing to overwrite ' + str(a.out))
    rep = {'schema': 'slam-refact-us026-poke-summary-v1', 'note': 'REPORT (dev), design §2.6 V1-V4', 'runs': [summarize(d) for d in a.runs]}
    a.out.parent.mkdir(parents=True, exist_ok=True)
    a.out.write_text(json.dumps(rep, indent=1) + '\n')
    for r in rep['runs']:
        print(json.dumps({'run': r['run'].split('/')[-1], 'keys': r['keys'], 'load': r['loadAvgAtStart'][0], 'V1': r['V1'], 'pv': r['V2']['pvValues'],
                          'V3': r['V3'], 'V4': {k: r['V4'][k] for k in ('kfPoseChangedSum', 'kfPoseChangedOnNonKeyframeCalls', 'firstKfPoseChangedFrame',
                                                                       'kfPoseMaxDt', 'mseResidualMinus1Calls', 'mseSumGt600Calls', 'vtLatchFrame', 'vtLatchedScale')},
                          'callMsKf': r['V4']['callMsKeyframe'], 'callMsOther': r['V4']['callMsOther']}))


if __name__ == '__main__':
    main()

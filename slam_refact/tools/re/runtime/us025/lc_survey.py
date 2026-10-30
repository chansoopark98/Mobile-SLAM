#!/usr/bin/env python3
"""US-025 LC-0 trigger survey summary: per scenario, one original-SDK run with poke {"lc":1} and one with {"lc":0}
(run-reference.mjs --memprobe --memprobe-lc --poke). Reports whether loop close fired (outLcOk), the frames, the
loop-close ring words (MapBuilder+0xa28..), the correction matrix, the keyframe descriptor vector and whether the
trace differs between the arms. The original is not run-to-run deterministic (docs/reports/parity.md 3.1), so a trace
difference is reported with the first differing frame next to the first lc-state frame; it is not by itself an effect.

  python3 -I lc_survey.py --runs <scenario>=<lc1 run dir>,<lc0 run dir> ... --out <new json>
"""
import argparse
import gzip
import json
import pathlib


def jsonl(p):
    with gzip.open(p, 'rt') as f:
        return [json.loads(line) for line in f if line.strip()]


def trace(p):
    return {r['i']: r for r in map(json.loads, p.read_text().splitlines()) if r.get('fresh')}


def arm(d):
    mp = jsonl(d / 'memprobe.jsonl.gz')
    pk = jsonl(d / 'poke.jsonl.gz')
    hdr, rec = mp[0], mp[1:]
    phdr, prec = pk[0], pk[1:]
    meta = json.loads((d / 'meta.json').read_text())
    ok = [r for r in rec if r.get('outLcOk')]
    mat = [r for r in rec if r.get('outLcMat') is not None]
    ring_states, last = [], None
    for r in rec:
        s = r.get('mbLcRing')
        if s != last:
            ring_states.append({'seq': r['seq'], 'i': r['i'], 'ring': s})
            last = s
    resets = [r for r in rec if r.get('outMapReset')]
    desc_vs_kf = [r for r in rec if r.get('wmKfDescCount') is not None]
    return {
        'dir': str(d), 'records': len(rec), 'memprobeError': hdr.get('error'), 'pokeError': phdr.get('error'),
        'poke': phdr.get('keys'), 'prePostLayout': phdr.get('prePostLayout'),
        'pokeGapCalls': sum(1 for r in prec if r.get('gap')),
        'lcPostValues': sorted({r['post'][5] for r in prec if r.get('post') and len(r['post']) > 5}),
        'outLcOkCalls': len(ok), 'outLcOkSeq': [r['seq'] for r in ok][:50], 'outLcOkFrames': [r['i'] for r in ok][:50],
        'outLcMatNonIdentityCalls': len(mat), 'ringStateChanges': len(ring_states) - 1, 'ringStates': ring_states[:12],
        'mapResets': len(resets), 'mapResetFrames': [r['i'] for r in resets],
        'kfLiveMax': max((r['kfLive'] or 0) for r in rec), 'lcKfSlotsMax': max((r.get('lcKfSlots') or 0) for r in rec),
        'wmKfDescCountMax': max((r.get('wmKfDescCount') or 0) for r in desc_vs_kf) if desc_vs_kf else None,
        'wmKfDescCountNonZeroCalls': sum(1 for r in desc_vs_kf if r['wmKfDescCount']),
        'framesFresh': len(trace(d / 'trace.jsonl')), 'metaStatus': meta.get('status'),
        'kfSlotsMax': max((r['kfSlots'] or 0) for r in rec),
        '_trace': d / 'trace.jsonl',
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--runs', nargs='+', required=True)
    ap.add_argument('--out', type=pathlib.Path, required=True)
    a = ap.parse_args()
    if a.out.exists():
        raise SystemExit('refusing to overwrite ' + str(a.out))
    rep = {}
    for spec in a.runs:
        name, dirs = spec.split('=')
        d1, d0 = (pathlib.Path(x) for x in dirs.split(','))
        a1, a0 = arm(d1), arm(d0)
        t1, t0 = trace(a1.pop('_trace')), trace(a0.pop('_trace'))
        both = sorted(set(t1) & set(t0))
        key = lambda r: (r['p'], r['q_wxyz'], r['status'], r['worldPoints'])
        diff = [i for i in both if key(t1[i]) != key(t0[i])]
        rep[name] = {'lc1': a1, 'lc0': a0,
                     'traceCompare': {'freshBoth': len(both), 'differingFrames': len(diff),
                                      'firstDifferingFrame': diff[0] if diff else None,
                                      'firstLcOkFrame': a1['outLcOkFrames'][0] if a1['outLcOkFrames'] else None,
                                      'bitIdentical': not diff and len(both) > 0}}
        print(name, json.dumps({'lc1_ok_calls': a1['outLcOkCalls'], 'lc0_ok_calls': a0['outLcOkCalls'],
                                'ring_changes_lc1': a1['ringStateChanges'], 'trace': rep[name]['traceCompare']}))
    a.out.write_text(json.dumps(rep, indent=1) + '\n')


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""US-025 LC-6b minimal comparison (desktop synthetic scene only; not the LC-6a compare tool): ORIGINAL lc=1 run (lcgold dump) vs the engine's loop close on
the same exact-input bundle, attempt by attempt, up to and including the original's first success (the original leaves that call through an exception,
us025-lcg-golden.md §8; nothing after it is comparable).

  lc6b_compare.py --run <original run dir> --engine-log <diag engine stderr> --out <new json>

One attempt = one processStagedFrame call that created a keyframe while loopCloseEnabled == 1 (events.jsonl, in order); the engine calls LoopClose exactly
once per keyframe creation, so attempts pair by order (the keyframe id of the NEWEST keyframe is checked for equality as the pairing sanity check).
original (plain run): newest id, observed pick (ring after the call: [newest.index, newest.id, cand, 0, 0, 1]), out+0x1c, out+0x20 matrix.
original (hook run, additionally): best distance + candidate (u2-result), 1128 match count, 1146 observation count / return, 660 C.
engine (diag copy, make_diag_engine.sh): outcome code, candidate id, distance, matches, correspondences, C.
"""
import argparse, json, pathlib, re, struct, sys

OUT = {0: 'TooFewSlots', 1: 'NoCandidate', 2: 'InvalidHandle', 3: 'TooFewMatches', 4: 'TooFewCorrespondences', 5: 'PoseEstimationFailed', 6: 'Applied'}


def f32(x):
    return struct.unpack('<I', struct.pack('<f', x))[0]


def load_original(run):
    lg = run / 'lcgold'
    blobs = (lg / 'blobs.bin').read_bytes()
    rd = lambda r: blobs[r['off']:r['off'] + r['len']]
    events = [json.loads(l) for l in (lg / 'events.jsonl').read_text().splitlines() if l]
    hooks = {}
    if (lg / 'hooks.jsonl').exists():
        for l in (lg / 'hooks.jsonl').read_text().splitlines():
            if l:
                h = json.loads(l); hooks.setdefault(h['seq'], {}).setdefault(h['id'], []).append(h)
    atts = []
    for e in events:
        if e.get('lcEnabled') != 1:
            continue
        rp = e['ringPost']
        newest = e['newest']
        pick = rp[2] if (rp[2] != 0) else None
        a = {'seq': e['seq'], 'i': e['i'], 'nSlots': e['nSlots'], 'nLive': e['nLive'], 'S1': e['nSlots'] >= 5, 'newestId': newest[1],
             'pickId': pick, 'pickIsSelf': pick == newest[1] if pick else None, 'success': bool(e.get('outLcOk')), 'mapReset': e.get('mapReset')}
        if a['success']:
            m = rd(e['outLcMat'])[:0x82]
            a['C'] = [f32w for f32w in struct.unpack('<16f', m[:64])]
        h = hooks.get(e['seq'])
        if h:
            if 'u2-result' in h:
                r = h['u2-result'][-1]; a['hookBest'] = r['best']; a['hookFound'] = r['found']; a['hookCandId'] = r['candId']
            if '1128-out' in h:
                a['matches'] = h['1128-out'][-1]['nMatches']
            if '1146-in' in h:
                a['corr'] = h['1146-in'][-1]['nObs']; a['pnpReturn'] = h['1146-out'][-1]['ret'] if '1146-out' in h else None
        atts.append(a)
    return atts, json.loads((lg / 'header.json').read_text())


def load_engine(path):
    rows = []
    rx = re.compile(r'LC6B call=(\d+) outcome=(\d+) applied=(\d) newest=(\d+)/(\d+) cand=(\d+)/(\d+) dist=(\S+) slots=(\d+) live=(\d+) matches=(\d+) corr=(\d+) unlinked=(\d+) C=(\S+)')
    for l in pathlib.Path(path).read_text(errors='replace').splitlines():
        m = rx.search(l)
        if m:
            g = m.groups()
            rows.append({'call': int(g[0]), 'outcome': OUT[int(g[1])], 'applied': g[2] == '1', 'newestId': int(g[4]), 'candId': int(g[6]) if int(g[6]) else None,
                         'dist': float(g[7]), 'nSlots': int(g[8]), 'nLive': int(g[9]), 'matches': int(g[10]), 'corr': int(g[11]), 'unlinked': int(g[12]),
                         'C': [float(x) for x in g[13].split(',')]})
    return rows


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--run', type=pathlib.Path, required=True)
    ap.add_argument('--engine-log', type=pathlib.Path, required=True)
    ap.add_argument('--out', type=pathlib.Path, required=True)
    a = ap.parse_args()
    if a.out.exists():
        sys.exit('refusing existing ' + str(a.out))
    orig, hdr = load_original(a.run)
    eng = load_engine(a.engine_log)
    first = next((k for k, x in enumerate(orig) if x['success']), None)
    upto = (first + 1) if first is not None else len(orig)   # compare through the first success only
    rows, agree = [], {'newest': 0, 'pick': 0, 'pickNonSelf': 0, 'matches': 0, 'matchesChecked': 0}
    for k in range(min(upto, len(eng))):
        o, e = orig[k], eng[k]
        r = {'k': k, 'seq': o['seq'], 'i': o['i'], 'nSlots': o['nSlots'], 'S1': o['S1'], 'newestOrig': o['newestId'], 'newestEng': e['newestId'],
             'pickOrig': o['pickId'], 'pickIsSelfOrig': o['pickIsSelf'], 'candEng': e['candId'], 'engOutcome': e['outcome'],
             'engMatches': e['matches'], 'engCorr': e['corr'], 'engDist': e['dist'], 'origSuccess': o['success'], 'engApplied': e['applied']}
        for key in ('hookBest', 'hookCandId', 'matches', 'corr', 'pnpReturn'):
            if key in o:
                r['orig_' + key] = o[key]
        if e['newestId'] == 0 or o['newestId'] == e['newestId']:   # the engine report has no ids when S1 (<5 slots) ends the attempt
            agree['newest'] += 1
        if o['S1']:
            if o['pickId'] == e['candId']:
                agree['pick'] += 1
                if o['pickId'] and not o['pickIsSelf']:
                    agree['pickNonSelf'] += 1
        if 'matches' in o and e['candId'] is not None and o['hookCandId'] == e['candId']:
            agree['matchesChecked'] += 1
            agree['matches'] += int(o['matches'] == e['matches'])
        if o['success'] and e['applied']:
            Co = o['C']
            # engine prints row-major (r,c) with %.9g (a float32 round-trips exactly); the original HMatrix is column-major: word 4*c + r
            Ce = [struct.unpack('<f', struct.pack('<f', e['C'][4 * r + c]))[0] for c in range(4) for r in range(4)]
            r['C_bit_equal'] = all(f32(x) == f32(y) for x, y in zip(Co, Ce))
            r['C_max_abs_diff'] = max(abs(x - y) for x, y in zip(Co, Ce))
            r['C_orig'], r['C_eng'] = Co, Ce
        rows.append(r)
    summary = {'originalAttempts': len(orig), 'engineLoopCloseCalls': len(eng), 'comparedThroughAttempt': upto - 1 if upto else None,
               'firstOriginalSuccessAttempt': first, 'agreement': agree,
               'originalS1Attempts': sum(1 for x in orig if x['S1']), 'originalPicks': sum(1 for x in orig if x['pickId']),
               'originalSelfPicks': sum(1 for x in orig if x['pickIsSelf']), 'originalSuccesses': sum(1 for x in orig if x['success']),
               'originalThrows': hdr.get('throws'), 'originalAborts': len(hdr.get('aborts') or []),
               'engineApplied': sum(1 for x in eng if x['applied']), 'engineOutcomes': {},
               'engineWholeRun': {'attemptsWithNonSelfCandidate': sum(1 for x in eng if x['candId'] and x['candId'] != x['newestId'] and x['nSlots'] >= 5),
                                  'maxMatchesNonSelfCandidate': max([x['matches'] for x in eng if x['candId'] and x['candId'] != x['newestId']], default=None),
                                  'nonSelfApplied': sum(1 for x in eng if x['applied'] and x['candId'] != x['newestId']),
                                  'note': 'engine-only, beyond the original first success the original state is undefined (not compared)'}}
    for x in eng:
        summary['engineOutcomes'][x['outcome']] = summary['engineOutcomes'].get(x['outcome'], 0) + 1
    a.out.write_text(json.dumps({'summary': summary, 'attempts': rows}, indent=1))
    print(json.dumps(summary, indent=1))


if __name__ == '__main__':
    main()

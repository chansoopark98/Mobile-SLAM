#!/usr/bin/env python3
"""US-025 LC-6b: where is the original's candidate keyframe relative to the true revisit? (desktop synthetic ground truth, observation only)

For every loop-close attempt of an original run (S1 passed, a non-self candidate was picked) it prints the GT camera distance between the newest keyframe and
the picked candidate, and the GT distance to the geometrically nearest live keyframe that is not a recent one (created > --recent-frames earlier), plus the
local-match count (hook runs). Keyframe creation frame = the stage frame of the call (events.jsonl i, else seq-2 as in lc-g correspondence).
usage: lc6b_geometry.py --run <original run dir> --scene <scene dir> --out <new json> [--recent-frames 45]"""
import argparse, json, pathlib, sys
import numpy as np


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--run', type=pathlib.Path, required=True)
    ap.add_argument('--scene', type=pathlib.Path, required=True)
    ap.add_argument('--out', type=pathlib.Path, required=True)
    ap.add_argument('--recent-frames', type=int, default=45)
    a = ap.parse_args()
    if a.out.exists():
        sys.exit('refusing existing ' + str(a.out))
    P = np.array([g['p'] for g in json.load(open(a.scene / 'gt.json'))['frames']])
    lg = a.run / 'lcgold'
    ev = [e for e in (json.loads(l) for l in open(lg / 'events.jsonl')) if 'newest' in e]
    frame = lambda e: e['i'] if e['i'] is not None else e['seq'] - 2
    born = {}
    for e in ev:
        born.setdefault(e['newest'][1], frame(e))
    hk = {}
    if (lg / 'hooks.jsonl').exists():
        for l in open(lg / 'hooks.jsonl'):
            h = json.loads(l); hk.setdefault(h['seq'], {}).setdefault(h['id'], []).append(h)
    d = lambda x, y: float(np.linalg.norm(P[x] - P[y]))
    rows = []
    for e in ev:
        if e.get('lcEnabled') != 1 or e['nSlots'] < 5:
            continue
        cand = e['ringPost'][2] or None
        nid = e['newest'][1]
        if not cand or cand == nid or cand not in born:
            continue
        ni = born[nid]
        live = [s['id'] for s in e['slots'] if s['id'] and s['id'] != nid and s['id'] in born]
        old = [(d(ni, born[k]), born[k], k) for k in live if ni - born[k] > a.recent_frames]
        near = min(old) if old else None
        m = hk.get(e['seq'], {}).get('1128-out', [{}])[-1].get('nMatches')
        rows.append({'seq': e['seq'], 'newestFrame': ni, 'candFrame': born[cand], 'gtDistToCand_m': round(d(ni, born[cand]), 3),
                     'candIsRecent': ni - born[cand] <= a.recent_frames, 'nearestOldFrame': near[1] if near else None,
                     'gtDistToNearestOld_m': round(near[0], 3) if near else None, 'candIsNearestOld': bool(near and near[2] == cand), 'matches': m})
    summ = {'attempts': len(rows), 'candRecent': sum(r['candIsRecent'] for r in rows), 'candIsNearestOld': sum(r['candIsNearestOld'] for r in rows),
            'withMatches': sum(r['matches'] is not None for r in rows), 'maxMatches': max([r['matches'] for r in rows if r['matches'] is not None], default=None),
            'candOldAndWithin0.3m': sum((not r['candIsRecent']) and r['gtDistToCand_m'] <= 0.3 for r in rows)}
    a.out.write_text(json.dumps({'summary': summ, 'rows': rows}, indent=1))
    print(json.dumps(summ))


if __name__ == '__main__':
    main()

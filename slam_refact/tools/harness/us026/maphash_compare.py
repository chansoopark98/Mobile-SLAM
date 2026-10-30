#!/usr/bin/env python3
"""US-026 root-cause diagnostic (REPORT tier): compare the engine's per-frame map hashes
(slam_refact_replay_maphash -> maphash.jsonl) with the original memprobe hashes (config-poke.js
kfPoseHash / ptsHash1 in poke.jsonl.gz). Read-only on both inputs.

  maphash_compare.py --poke <original run dir>/poke.jsonl.gz --engine <replay dir>/maphash.jsonl [--fd N]

Frames are joined on the input frame index i. Prints the first frame at which kfPoseHash / ptsHash1 /
the live counts differ, the number of differing frames, and (with --fd) the lead of the first map
difference over the first pose difference frame f_d.
"""
import argparse
import gzip
import json


def load(path, gz):
    opener = gzip.open if gz else open
    rows = {}
    with opener(path, 'rt') as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            r = json.loads(line)
            if 'i' in r:
                rows[r['i']] = r
    return rows


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--poke', required=True)
    ap.add_argument('--engine', required=True)
    ap.add_argument('--fd', type=int, default=None)
    a = ap.parse_args()
    o = load(a.poke, a.poke.endswith('.gz'))
    e = load(a.engine, False)
    common = sorted(set(o) & set(e))
    no_wm = [i for i in common if not e[i].get('wm')]
    res = {'framesCompared': 0, 'engineNoMapFrames': len(no_wm), 'onlyOriginal': len(set(o) - set(e)),
           'onlyEngine': len(set(e) - set(o))}
    first = {'kfPoseHash': None, 'ptsHash1': None, 'kfLive': None, 'pts1Live': None}
    count = {k: 0 for k in first}
    for i in common:
        if not e[i].get('wm'):
            continue
        res['framesCompared'] += 1
        for k in first:
            if o[i].get(k) != e[i].get(k):
                count[k] += 1
                if first[k] is None:
                    first[k] = i
    res['firstDifferenceFrame'] = first
    res['differingFrames'] = count
    anyfirst = [v for v in first.values() if v is not None]
    res['firstMapDifference'] = min(anyfirst) if anyfirst else None
    if a.fd is not None and res['firstMapDifference'] is not None:
        res['f_d'] = a.fd
        res['leadOfMapDifferenceOverFd'] = a.fd - res['firstMapDifference']
    print(json.dumps(res, indent=1))


if __name__ == '__main__':
    main()

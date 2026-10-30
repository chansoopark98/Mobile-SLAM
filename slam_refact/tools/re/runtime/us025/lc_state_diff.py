#!/usr/bin/env python3
"""US-025 LC-0: first memprobe-state divergence between two original-SDK runs of the same scenario (REPORT).
State tuple per call (seq-aligned): mbFrameCount, outMapReset, outKeyframeCreated, kfLive, pts1..pts3 [slot,free,live,tenured],
trkState. Used with a lc1/lc0 pair and with an lc0/lc0b pair (noise floor of the non-deterministic original).

  python3 -I lc_state_diff.py <run dir A> <run dir B>
"""
import gzip
import json
import pathlib
import sys

K = ['mbFrameCount', 'outMapReset', 'outKeyframeCreated', 'kfLive', 'pts1', 'pts2', 'pts3', 'trkState']


def rows(d):
    with gzip.open(pathlib.Path(d) / 'memprobe.jsonl.gz', 'rt') as f:
        return {r['seq']: r for r in map(json.loads, list(f)[1:])}


def main():
    a, b = rows(sys.argv[1]), rows(sys.argv[2])
    common = sorted(set(a) & set(b))
    first = next((s for s in common if any(a[s][k] != b[s][k] for k in K)), None)
    n = sum(1 for s in common if any(a[s][k] != b[s][k] for k in K))
    print(json.dumps({'calls': [len(a), len(b)], 'common': len(common), 'firstStateDivergenceSeq': first,
                      'firstDivergenceFrame': a[first]['i'] if first else None, 'divergentCalls': n,
                      'resetsA': [s for s in a if a[s]['outMapReset']], 'resetsB': [s for s in b if b[s]['outMapReset']],
                      'kfLiveMaxA': max(r['kfLive'] or 0 for r in a.values()), 'kfLiveMaxB': max(r['kfLive'] or 0 for r in b.values())}))


if __name__ == '__main__':
    main()

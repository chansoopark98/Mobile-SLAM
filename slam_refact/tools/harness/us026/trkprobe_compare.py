#!/usr/bin/env python3
"""US-026 root-cause A-3 (REPORT tier, read-only): original memprobe/poke tracker fields vs the engine's
trkprobe.jsonl (slam_refact_replay_maphash, trkprobe.cc), joined by input frame index.

Compared (same names on both sides): mbState trkState trkFrameTrackerOk trkConsecutiveOk trkCandidates trkSky
frmWorldMatches fftAll fftGround fftSky fftLastOk fftHeight(6 decimals) fftCalls mseScale mseResidual mseSum mseHave
mseActive mseKfHash mseImuHash (poke) vtLatch vtLatchedScale (poke, only when latched).
Usage: trkprobe_compare.py --memprobe M.gz --poke P.gz --trkprobe T.jsonl [--from 320 --to 330] [--stages]
Prints per-field {compared, differing, first, firstValues} and, for --from/--to, a per-frame table of the fields."""
import argparse, gzip, json, struct


def load(path):
    op = gzip.open if path.endswith('.gz') else open
    rows = {}
    with op(path, 'rt') as f:
        for line in f:
            line = line.strip()
            if line:
                r = json.loads(line)
                if r.get('i') is not None:
                    rows[r['i']] = r
    return rows


def f32(x):
    return struct.unpack('<f', struct.pack('<f', float(x)))[0]


MEM = ['mbState', 'trkState', 'trkFrameTrackerOk', 'trkConsecutiveOk', 'trkCandidates', 'trkSky', 'frmWorldMatches',
       'fftAll', 'fftGround', 'fftSky', 'fftLastOk', 'fftHeight', 'fftCalls']
POKE = ['mseScale', 'mseResidual', 'mseSum', 'mseHave', 'mseActive', 'mseKfHash', 'mseImuHash', 'vtLatch']


def norm(v):
    if isinstance(v, bool):
        return int(v)
    if isinstance(v, float):
        return f32(v)
    return v


def main():
    ap = argparse.ArgumentParser()
    for k in ('memprobe', 'poke', 'trkprobe'):
        ap.add_argument('--' + k, required=True)
    ap.add_argument('--from', dest='lo', type=int, default=None)
    ap.add_argument('--to', dest='hi', type=int, default=None)
    ap.add_argument('--stages', action='store_true')
    a = ap.parse_args()
    m, p, t = load(a.memprobe), load(a.poke), load(a.trkprobe)
    fields = {}
    table = {}

    def add(name, i, ov, ev):
        d = fields.setdefault(name, {'compared': 0, 'differing': 0, 'first': None, 'firstValues': None})
        d['compared'] += 1
        if ov != ev:
            d['differing'] += 1
            if d['first'] is None:
                d['first'] = i
                d['firstValues'] = [ov, ev]

    for i in sorted(set(m) & set(t) & set(p)):
        mr, pr, tr = m[i], p[i], t[i]
        if mr.get('root') in (0, None) or 'trkState' not in tr:
            continue
        row = {}
        for k in MEM:
            if mr.get(k) is None or tr.get(k) is None:
                continue
            ov, ev = mr[k], tr[k]
            if k == 'fftHeight':
                ov, ev = round(float(ov), 6), round(float(ev), 6)
            add(k, i, norm(ov), norm(ev))
            row[k] = (ov, ev)
        for k in POKE:
            if pr.get(k) is None or tr.get(k) is None:
                continue
            ov, ev = pr[k], tr[k]
            if k in ('mseScale', 'mseResidual', 'mseSum'):
                ov, ev = f32(ov), f32(ev)
            add(k, i, norm(ov), norm(ev))
            row[k] = (ov, ev)
        if pr.get('vtLatch'):
            add('vtLatchedScale', i, f32(pr['vtLatchedScale']), f32(tr['vtLatchedScale']))
        table[i] = row
    firsts = [d['first'] for d in fields.values() if d['first'] is not None]
    print(json.dumps({'fields': fields, 'earliestFirstDifference': min(firsts) if firsts else None}, indent=1))
    if a.lo is not None:
        hi = a.hi if a.hi is not None else a.lo
        for i in range(a.lo, hi + 1):
            row = table.get(i, {})
            diff = [k for k, (o, e) in row.items() if norm(o) != norm(e)]
            print(i, 'DIFF ' + ','.join(diff) if diff else 'all-equal', {k: (v[0] if v[0] == v[1] else v) for k, v in row.items() if k not in POKE[:7]})
            if a.stages and i in t:
                for s in t[i].get('stages', []):
                    print('   ', json.dumps(s))


if __name__ == '__main__':
    main()

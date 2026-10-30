#!/usr/bin/env python3
"""US-026 root-cause diagnostic (REPORT tier, A-1b): per-field first-difference frames between the original
memprobe/poke records and the engine (maphash.jsonl pool stats + baLatch, trace.jsonl tracker / scale scalars).
Read-only. Fields compared (engine side readable without engine/src changes):
  pool k=1..5 slots/free/live/tenured  (memprobe pts<k> vs maphash pools)
  baLatch                              (poke baLatch vs maphash baLatch)
  trkState, trkFrameTrackerOk          (memprobe vs trace srMapTrackerState / srFrameTrackerOk)
  vtLatch, vtLatchedScale              (poke vs trace srScaleLatched / srLatchedScale)
Usage: maphash_fields.py --memprobe M.gz --poke P.gz --maphash H.jsonl --trace T.jsonl [--fd N]
"""
import argparse, gzip, json, struct


def load(path):
    op = gzip.open if path.endswith('.gz') else open
    rows = {}
    with op(path, 'rt') as f:
        for line in f:
            line = line.strip()
            if line:
                r = json.loads(line)
                if 'i' in r:
                    rows[r['i']] = r
    return rows


def f32(x):
    return struct.unpack('<f', struct.pack('<f', float(x)))[0]


def main():
    ap = argparse.ArgumentParser()
    for k in ('memprobe', 'poke', 'maphash', 'trace'):
        ap.add_argument('--' + k, required=True)
    ap.add_argument('--fd', type=int, default=None)
    a = ap.parse_args()
    m, p, h, t = load(a.memprobe), load(a.poke), load(a.maphash), load(a.trace)
    names = ['slots', 'free', 'live', 'tenured']
    fields = {}
    def add(name, i, ov, ev):
        d = fields.setdefault(name, {'compared': 0, 'differing': 0, 'first': None, 'firstValues': None})
        d['compared'] += 1
        if ov != ev:
            d['differing'] += 1
            if d['first'] is None:
                d['first'] = i
                d['firstValues'] = [ov, ev]
    for i in sorted(set(m) & set(h) & set(t) & set(p)):
        if not h[i].get('wm'):
            continue
        for k in range(1, 6):
            for j, n in enumerate(names):
                add(f'pool{k}.{n}', i, m[i][f'pts{k}'][j], h[i]['pools'][k - 1][j])
        add('baLatch', i, p[i]['baLatch'], h[i]['baLatch'])
        te = t[i]
        if te.get('srMapTrackerState') is None:
            continue
        add('trkState', i, m[i]['trkState'], te['srMapTrackerState'])
        add('trkFrameTrackerOk', i, int(m[i]['trkFrameTrackerOk']), int(bool(te['srFrameTrackerOk'])))
        add('vtLatch', i, int(p[i]['vtLatch']), int(bool(te['srScaleLatched'])))
        if p[i]['vtLatch']:  # unlatched: original b+2844 holds 1.0 (default), engine latched_scale is 0 (definition, not a difference)
            add('vtLatchedScale', i, f32(p[i]['vtLatchedScale']), f32(te['srLatchedScale']))
    firsts = [d['first'] for d in fields.values() if d['first'] is not None]
    out = {'fields': fields, 'earliestFirstDifference': min(firsts) if firsts else None}
    if a.fd is not None:
        out['f_d'] = a.fd
        out['fieldsDifferingBeforeOrAtFd'] = sorted(n for n, d in fields.items() if d['first'] is not None and d['first'] <= a.fd)
    print(json.dumps(out, indent=1))


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""WP2-PRE item 5: check an original-engine pyramid capture (capture_pyramid.mjs output).

Checks (fixed before the c2-73 run was inspected):
  * status COMPLETED, no page errors, no blocked external requests, no input pixel hash mismatch;
  * capture k -> input frame k-1 mapping, and every frame in [first, last] has exactly one capture;
  * every cap-*.rgba.gz decompresses to rows*1024*4 bytes whose sha256 equals meta rawSha256;
  * H10-style determinism (spec 01 §8, H10): for every input frame present in a reference capture,
    the raw readback sha256 is byte-identical.
Writes a JSON report; exit 1 if any check fails. Stdlib only; run with python3 -I.
"""
import argparse
import gzip
import hashlib
import json
import os
import sys


def load(d):
    with open(os.path.join(d, 'meta.json')) as f:
        return json.load(f)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--capture', required=True)
    ap.add_argument('--frames', required=True, help='first:last inclusive, e.g. 0:71')
    ap.add_argument('--reference', action='append', default=[], help='other capture dir(s) for H10-style comparison')
    ap.add_argument('--out', required=True)
    a = ap.parse_args()
    if os.path.exists(a.out):
        sys.exit('output exists; preserving evidence')
    first, last = (int(x) for x in a.frames.split(':'))
    m = load(a.capture)
    rep = {'capture': a.capture, 'frames': [first, last], 'status': m.get('status'), 'checks': {}}
    c = rep['checks']
    c['completed'] = m.get('status') == 'COMPLETED' and not m.get('pageErrors') and not m.get('blockedExternal') \
        and not m.get('pixelHashMismatches') and m.get('wrapped') is True
    caps = m['captures']
    mapping_ok = all(cp['capture'] == k for k, cp in enumerate(caps)) and \
        all(cp['inputFrame'] == (k - 1 if k > 0 else None) for k, cp in enumerate(caps))
    by_frame = {}
    for cp in caps:
        if cp['inputFrame'] is not None:
            by_frame.setdefault(cp['inputFrame'], []).append(cp)
    want = list(range(first, last + 1))
    c['mapping_k_to_frame_k_minus_1'] = mapping_ok
    c['frames_missing'] = [f for f in want if f not in by_frame]
    c['frames_duplicated'] = [f for f, v in by_frame.items() if len(v) > 1]
    c['n_captures'] = len(caps)
    raw_bad = []
    rows = m['rows']
    for cp in caps:
        p = os.path.join(a.capture, 'cap-%03d.rgba.gz' % cp['capture'])
        with gzip.open(p, 'rb') as f:
            raw = f.read()
        if len(raw) != rows * 1024 * 4 or hashlib.sha256(raw).hexdigest() != cp['rawSha256']:
            raw_bad.append(cp['capture'])
    c['raw_size_and_sha_bad'] = raw_bad
    rep['h10'] = []
    for ref in a.reference:
        rm = load(ref)
        rb = {cp['inputFrame']: cp['rawSha256'] for cp in rm['captures'] if cp['inputFrame'] is not None}
        common = sorted(set(rb) & set(by_frame))
        diff = [f for f in common if rb[f] != by_frame[f][0]['rawSha256']]
        rep['h10'].append({'reference': ref, 'common_frames': len(common), 'first': common[0] if common else None,
                           'last': common[-1] if common else None, 'byte_identical': len(common) - len(diff), 'diff': diff})
    ok = c['completed'] and mapping_ok and not c['frames_missing'] and not c['frames_duplicated'] and not raw_bad \
        and all(h['common_frames'] > 0 and not h['diff'] for h in rep['h10'])
    rep['pass'] = ok
    with open(a.out, 'w') as f:
        json.dump(rep, f, indent=1)
    print(json.dumps(rep, indent=1))
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()

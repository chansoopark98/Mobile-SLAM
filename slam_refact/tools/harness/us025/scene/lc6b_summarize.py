#!/usr/bin/env python3
"""US-025 LC-6b: one row per original run of a synthetic scene (status, loop-close attempts, picks, self picks, successes, exceptions, ORB counts, load).
usage: lc6b_summarize.py --root <lc6b dir> --out <new json>      (walks <root>/runs/<scenario>/<label>/meta.json)"""
import argparse, json, pathlib, sys


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--root', type=pathlib.Path, required=True)
    ap.add_argument('--out', type=pathlib.Path, required=True)
    a = ap.parse_args()
    if a.out.exists():
        sys.exit('refusing existing ' + str(a.out))
    rows = []
    for meta in sorted((a.root / 'runs').glob('*/*/meta.json')):
        run = meta.parent
        m = json.loads(meta.read_text())
        row = {'scenario': run.parent.name, 'label': run.name, 'status': m.get('status'), 'poke': (m.get('poke') or {}).get('keys') if isinstance(m.get('poke'), dict) else None,
               'probeWasm': bool(m.get('probeWasm')), 'loadStart': (m.get('loadAvgAtStart') or [None])[0], 'loadEnd': (m.get('loadAvgAtEnd') or [None])[0]}
        lg = run / 'lcgold'
        if (lg / 'events.jsonl').exists():
            ev = [e for e in (json.loads(l) for l in (lg / 'events.jsonl').read_text().splitlines() if l) if 'newest' in e]
            att = [e for e in ev if e.get('lcEnabled') == 1]
            s1 = [e for e in att if e['nSlots'] >= 5]
            picks = [e for e in s1 if e['ringPost'][2]]
            live = lambda e: {s['id'] for s in e['slots'] if s['id']}
            row.update({'keyframeCalls': len(ev), 'attempts': len(att), 'S1': len(s1), 'picks': len(picks),
                        'selfPicks': sum(e['ringPost'][2] == e['newest'][1] for e in picks),
                        'nonSelfValidPicks': sum(e['ringPost'][2] != e['newest'][1] and e['ringPost'][2] in live(e) for e in picks),
                        'garbagePicks': sum(e['ringPost'][2] not in live(e) for e in picks),
                        'successes': sum(bool(e.get('outLcOk')) for e in att),
                        'orbPerKeyframeMinMax': [min(e['newestOrb']['n'] for e in ev if e.get('newestOrb')), max(e['newestOrb']['n'] for e in ev if e.get('newestOrb'))] if any(e.get('newestOrb') for e in ev) else None})
            h = json.loads((lg / 'header.json').read_text())
            row['exceptions'] = [t.get('message', '')[:40] for t in h.get('throws', [])]
        else:
            row['note'] = 'no lcgold dump (run FAILED: page spun at ~98% CPU, renderer killed; lc-g §9)' if m.get('status') == 'FAILED' else 'no lcgold dump'
        rows.append(row)
    a.out.write_text(json.dumps(rows, indent=1))
    for r in rows:
        print(r['scenario'], r['label'], r['status'], {k: r.get(k) for k in ('keyframeCalls', 'attempts', 'S1', 'picks', 'selfPicks', 'nonSelfValidPicks', 'garbagePicks', 'successes')}, r.get('exceptions'), r.get('orbPerKeyframeMinMax'))


if __name__ == '__main__':
    main()

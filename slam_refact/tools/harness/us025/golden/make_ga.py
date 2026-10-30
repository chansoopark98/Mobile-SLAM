#!/usr/bin/env python3
"""US-025 LC-G: G-A golden package from run_lcgold.mjs runs (OFF runs; no poke needed).

For every keyframe-created call of a run (events.jsonl of the run's lcgold/ directory) this writes the pair the loop-close descriptor port (LC-2b) needs:
the keyframe's ORB store (`kf+0x2c`, n x 32 B) and the agate 32 B the original wrote for it (`wm+0x184 + slot*0x20`).  Layout of the output directory:

  <out>/<scenario>-<label>/keyframes.jsonl   one JSON object per keyframe (fields below)
  <out>/<scenario>-<label>/orb.bin           concatenated ORB stores (8-byte aligned records; keyframes.jsonl gives offset/length/sha256)
  <out>/<scenario>-<label>/agate.bin         concatenated agate descriptors, 32 B each, record k at byte 32*k
  <out>/<scenario>-<label>/checks.json       self-consistency checks (all must be true; see below)
  <out>/<scenario>-<label>/SHA256SUMS

keyframes.jsonl fields: k (record index = agate.bin index), run, seq (processStagedFrame call), i/t (input frame/seconds, null for bootstrap calls), slot, kfId,
orb_n, orb_off, orb_len, orb_sha256, orb_fnv64, agate_hex, nSlots, nLive, mapReset.  The agate is the descriptor of `slot` in the whole-vector snapshot taken in the
same call (cross-checked against the separately read copy).

checks.json: agate_equals_vector (the 32 B read at newest slot == the same slot of the descriptor-vector snapshot), orb_fnv_matches (FNV of the dumped ORB bytes
== FNV computed in the page from the live keyframe), stable_across_events (every (slot, kfId) that survives to a later event keeps the same agate bytes and the same
ORB store FNV), vector_len_equals_slots (nDesc == nSlots == nHandles at every event), handle_ids_match (handle vector id == kf id for every live slot).
usage: make_ga.py --run <scenario>=<run dir> [--run ...] --out <new dir>"""
import argparse
import hashlib
import json
import pathlib
import sys


def sha(b):
    return hashlib.sha256(b).hexdigest()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--run', action='append', required=True, help='<name>=<run dir containing lcgold/>')
    ap.add_argument('--out', type=pathlib.Path, required=True)
    a = ap.parse_args()
    if a.out.exists():
        raise SystemExit('refusing to overwrite ' + str(a.out))
    a.out.mkdir(parents=True)
    summary = []
    for spec in a.run:
        name, d = spec.split('=', 1)
        d = pathlib.Path(d)
        lg = d / 'lcgold'
        blobs = (lg / 'blobs.bin').read_bytes()
        rd = lambda r: blobs[r['off']:r['off'] + r['len']]
        events = [json.loads(l) for l in (lg / 'events.jsonl').read_text().splitlines() if l]
        header = json.loads((lg / 'header.json').read_text())
        meta = json.loads((d / 'meta.json').read_text())
        od = a.out / name
        od.mkdir()
        orb_bin, agate_bin, lines = bytearray(), bytearray(), []
        checks = {'agate_equals_vector': True, 'orb_fnv_matches': True, 'stable_across_events': True, 'vector_len_equals_slots': True, 'handle_ids_match': True}
        seen = {}   # (slot, kfId) -> (agate, orbFnv)
        stable_n = 0
        for ev in events:
            if 'skipped' in ev or 'newestOrb' not in ev:
                continue
            vec = rd(ev['desc'])
            if not (ev['nDesc'] == ev['nSlots'] == ev['nHandles'] and len(vec) == 32 * ev['nDesc']):
                checks['vector_len_equals_slots'] = False
            for s in ev['slots']:
                h = ev['handles'][2 * s['s'] + 1] if 2 * s['s'] + 1 < len(ev['handles']) else None
                if s['id'] != 0 and h != s['id']:
                    checks['handle_ids_match'] = False
                if s['id'] != 0:
                    key = (s['s'], s['id'], s['kseq'])
                    cur = (vec[32 * s['s']:32 * s['s'] + 32], s['orbFnv'])
                    if key in seen:
                        stable_n += 1
                        if seen[key] != cur:
                            checks['stable_across_events'] = False
                    seen[key] = cur
            slot = ev['newest'][0]
            agate = rd(ev['newestAgate'])
            if agate != vec[32 * slot:32 * slot + 32]:
                checks['agate_equals_vector'] = False
            orb = rd(ev['newestOrb']['bytes'])
            if len(orb) != 32 * ev['newestOrb']['n']:
                checks['orb_fnv_matches'] = False
            if ev['newestOrb']['fnv'] != next(s['orbFnv'] for s in ev['slots'] if s['s'] == slot):
                checks['orb_fnv_matches'] = False
            k = len(lines)
            off = len(orb_bin)
            orb_bin += orb + bytes((8 - len(orb) % 8) % 8)
            agate_bin += agate
            lines.append({'k': k, 'run': name, 'seq': ev['seq'], 'i': ev.get('i'), 't': ev.get('t'), 'slot': slot, 'kfId': ev['newest'][1], 'orb_n': ev['newestOrb']['n'],
                          'orb_off': off, 'orb_len': len(orb), 'orb_sha256': sha(orb), 'orb_fnv64': ev['newestOrb']['fnv'], 'agate_hex': agate.hex(),
                          'nSlots': ev['nSlots'], 'nLive': ev['nLive'], 'mapReset': ev['mapReset']})
        (od / 'keyframes.jsonl').write_text('\n'.join(json.dumps(x) for x in lines) + '\n')
        (od / 'orb.bin').write_bytes(bytes(orb_bin))
        (od / 'agate.bin').write_bytes(bytes(agate_bin))
        checks['stable_pairs_compared'] = stable_n
        checks['keyframes'] = len(lines)
        checks['orb_nonempty'] = sum(1 for x in lines if x['orb_n'] > 0)
        checks['orb_empty'] = sum(1 for x in lines if x['orb_n'] == 0)
        checks['distinct_agate'] = len({x['agate_hex'] for x in lines})
        checks['source'] = {'run_dir': str(d), 'scenario': meta['scenario'], 'label': meta['label'], 'wasmSha256': header['wasmSha256'], 'hookMode': header['hookMode'],
                            'poke': header['poke'], 'lcEnabledValues': sorted({ev.get('lcEnabled') for ev in events if 'skipped' not in ev}),
                            'loadAvgAtStart': meta.get('loadAvgAtStart'), 'loadAvgAtEnd': meta.get('loadAvgAtEnd'), 'events_sha256': sha((lg / 'events.jsonl').read_bytes()),
                            'blobs_sha256': sha((lg / 'blobs.bin').read_bytes()), 'inputFrames': meta['summary']['inputFrames']}
        (od / 'checks.json').write_text(json.dumps(checks, indent=1) + '\n')
        sums = ''.join('%s  %s\n' % (sha((od / f).read_bytes()), f) for f in ('keyframes.jsonl', 'orb.bin', 'agate.bin', 'checks.json'))
        (od / 'SHA256SUMS').write_text(sums)
        summary.append({'name': name, **{k: checks[k] for k in checks if k != 'source'}})
    print(json.dumps(summary, indent=1))
    if not all(s['agate_equals_vector'] and s['orb_fnv_matches'] and s['stable_across_events'] and s['vector_len_equals_slots'] and s['handle_ids_match'] for s in summary):
        sys.exit(1)


if __name__ == '__main__':
    main()

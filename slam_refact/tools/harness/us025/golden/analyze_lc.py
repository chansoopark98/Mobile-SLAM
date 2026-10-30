#!/usr/bin/env python3
"""US-025 LC-G: per-attempt tables from run_lcgold.mjs lc=1 runs (G-C candidate selection, G-F correction matrix, G-G post state, G-H triggers).

Reads <run>/lcgold/{header.json,events.jsonl[,hooks.jsonl],blobs.bin} and writes <out>/attempts.jsonl + <out>/summary.json:

  one attempt = one processStagedFrame call that created a keyframe while MapBuilder.loopCloseEnabled == 1 (events with lcEnabled == 1).
  The loop close block runs in that call iff slot count >= 5 (G936:4196, free slots included); `S1_slots_ge5` records that.
  observed pick:   ringPost = [lastNewest.index, lastNewest.id, ring0, ring1, ring2, cursor] (mb+0xa28..+0xa3f, state AFTER the call).
                   The ring is cleared whenever the newest keyframe handle changed (every keyframe-creating call), so after an attempt with a pick
                   ringPost == [newest.index, newest.id, candId, 0, 0, 1]; without a pick ringPost == [newest.index, newest.id, 0, 0, 0, 0].
  model picks:     recomputed from the dumped descriptor vector / handle vector / covisibility maps with the spec 12 s3.1 selection loop under
                     ideal      : distances indexed by slot (what the code would do without the compaction defect)
                     compact0   : the original's compacted list read by slot index, tail entries (index >= live) read as id 0 (skipped)
                     compactHeap: compact list + the observed uninitialised tail (hook mode `u2-dist` record), when available
                   and compared with the observed pick (agree = candidate id equal, or both none).
Spec 12 s3.1 constants: threshold 80.0 (strict <), covisibility minimum shared points 10, ring empty after the reset.
usage: analyze_lc.py --run <name>=<run dir> [--run ...] --out <new dir>"""
import argparse
import json
import pathlib
import struct
import sys

THRESH = 80.0
COVIS_MIN = 10


def popcnt(a, b):
    return sum(bin(x ^ y).count('1') for x, y in zip(a, b))


def covisible(covis, slot, n_slots):
    """1143: handles (index, id) whose shared-point count >= 10; empty when fewer than 2 slots."""
    if n_slots < 2:
        return []
    return [(e[0], e[1]) for e in covis.get(slot, []) if e[2] >= COVIS_MIN]


def select(policy, ev, desc, covis, tail=None):
    n_slots, n_desc = ev['nSlots'], ev['nDesc']
    ni, nid = ev['newest']
    handles = [(ev['handles'][2 * s], ev['handles'][2 * s + 1]) for s in range(ev['nHandles'])]
    newest_desc = desc[ni]
    # S3 compact list over live handles in slot order
    live_slots = [s for s in range(n_desc) if handles[s][1] != 0]
    compact = [(float(popcnt(desc[s], newest_desc)), handles[s]) for s in live_slots]
    # S4 2-hop covisible exclusion set (indices only, S2)
    s1 = set()
    for h1 in covisible(covis, ni, n_slots):
        for h2 in covisible(covis, h1[0], n_slots):
            s1.add(h2)
        s1.add(h1)
    s1.discard((ni, nid))
    s2 = {h[0] for h in s1}
    best, cand = THRESH, None
    for m in range(max(n_desc, 1)):
        if m == ni:
            continue
        if policy == 'ideal':
            if m >= n_desc:
                continue
            d, h = float(popcnt(desc[m], newest_desc)), handles[m]
        else:
            if m < len(compact):
                d, h = compact[m]
            elif policy == 'compactHeap' and tail is not None:
                d, h = tail[m]
            else:
                continue   # id 0 tail
        if h[1] == 0 or not d < best or (s2 and m in s2):
            continue
        best, cand = d, h
    return cand, best


def parse_dist(rec, blobs):
    r = rec['dist']
    raw = blobs[r['off']:r['off'] + r['len']]
    out = []
    for k in range(rec['nDesc']):
        d = struct.unpack_from('<f', raw, 12 * k)[0]
        idx, ident = struct.unpack_from('<II', raw, 12 * k + 4)
        out.append((d, (idx, ident)))
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--run', action='append', required=True)
    ap.add_argument('--out', type=pathlib.Path, required=True)
    a = ap.parse_args()
    a.out.mkdir(parents=True, exist_ok=False)
    rows, summaries = [], []
    for spec in a.run:
        name, d = spec.split('=', 1)
        d = pathlib.Path(d)
        lg = d / 'lcgold'
        blobs = (lg / 'blobs.bin').read_bytes()
        rd = lambda r: blobs[r['off']:r['off'] + r['len']]
        events = [json.loads(l) for l in (lg / 'events.jsonl').read_text().splitlines() if l]
        hooks = [json.loads(l) for l in (lg / 'hooks.jsonl').read_text().splitlines() if l] if (lg / 'hooks.jsonl').exists() else []
        by_seq = {}
        for h in hooks:
            by_seq.setdefault(h['seq'], []).append(h)
        n_att = n_run = n_pick = n_ok = n_free = n_self = n_garbage_pick = 0
        agree = {'ideal': 0, 'compact0': 0, 'compactHeap': 0}
        n_heap = 0
        tail_kinds = {}
        prefix_checked = prefix_ok = 0
        for ev in events:
            if 'skipped' in ev or not ev.get('lcEnabled'):
                continue
            n_att += 1
            vec = rd(ev['desc'])
            desc = [vec[32 * s:32 * s + 32] for s in range(ev['nDesc'])]
            covis = {s: [tuple(x) for x in c] for s, c in ev['covis']}
            ring = ev['ringPost']
            picked = ring[2] if ring[5] != 0 or ring[2] != 0 else None
            row = {'run': name, 'seq': ev['seq'], 'i': ev.get('i'), 't': ev.get('t'), 'S1_slots_ge5': ev['nSlots'] >= 5, 'nSlots': ev['nSlots'], 'nLive': ev['nLive'],
                   'nFree': ev['nSlots'] - ev['nLive'], 'newest': ev['newest'], 'ringPre': ev['ringPre'], 'ringPost': ring, 'observedPickId': picked,
                   'outLcOk': ev['outLcOk'], 'mapReset': ev['mapReset']}
            if ev['nSlots'] >= 5:
                live_ids = {ev['handles'][2 * s + 1] for s in range(ev['nHandles']) if ev['handles'][2 * s + 1]}
                row['pickIsSelf'] = picked is not None and picked == ev['newest'][1]
                row['pickIsLiveKeyframe'] = picked is not None and picked in live_ids
                n_self += bool(row['pickIsSelf'])
                n_garbage_pick += bool(picked is not None and not row['pickIsLiveKeyframe'])
                n_run += 1
                n_pick += picked is not None
                n_ok += bool(ev['outLcOk'])
                n_free += ev['nSlots'] > ev['nLive']
                tail = None
                for h in by_seq.get(ev['seq'], []):
                    if h['id'] == 'u2-dist':
                        tail = parse_dist(h, blobs)
                        row['u2Dist'] = [[x[0], x[1][0], x[1][1]] for x in tail]
                        live_ids = {hd[1] for hd in (((ev['handles'][2 * s]), ev['handles'][2 * s + 1]) for s in range(ev['nHandles'])) if hd[1]}
                        kinds = []
                        for pos in range(ev['nLive'], len(tail)):
                            dd, hh = tail[pos]
                            kinds.append('idZero' if hh[1] == 0 else ('liveHandle' if hh[1] in live_ids else 'garbage'))
                        row['tailKinds'] = kinds
                        for k in kinds:
                            tail_kinds[k] = tail_kinds.get(k, 0) + 1
                        compact_expect = [[float(popcnt(desc[s], desc[ev['newest'][0]])), ev['handles'][2 * s], ev['handles'][2 * s + 1]] for s in range(ev['nDesc']) if ev['handles'][2 * s + 1]]
                        row['compactPrefixMatchesModel'] = all(abs(row['u2Dist'][k][0] - compact_expect[k][0]) == 0 and row['u2Dist'][k][1:] == compact_expect[k][1:] for k in range(len(compact_expect)))
                        prefix_checked += 1
                        prefix_ok += bool(row['compactPrefixMatchesModel'])
                    if h['id'] == 'u2-result':
                        row['u2Result'] = {k: h[k] for k in ('best', 'candIdx', 'candId', 'found')}
                for pol in ('ideal', 'compact0', 'compactHeap'):
                    if pol == 'compactHeap' and tail is None:
                        continue
                    cand, best = select(pol, ev, desc, covis, tail)
                    row[pol] = {'candId': cand[1] if cand else None, 'candIndex': cand[0] if cand else None, 'best': best if cand else None}
                    ok = (cand[1] if cand else None) == picked
                    row[pol]['agrees'] = ok
                    agree[pol] += ok
                n_heap += tail is not None
            if ev['outLcOk']:
                row['correctionC'] = ev['outLcMat']
                row['newestExtrinsicAfter'] = ev['newestExtrinsic']
                row['cand'] = ev.get('cand')
            rows.append(row)
        summaries.append({'run': name, 'dir': str(d), 'attempts_lc_on': n_att, 'lc_block_ran_S1': n_run, 'pick_made': n_pick, 'pick_is_self_keyframe': n_self, 'pick_is_not_a_live_keyframe(garbage)': n_garbage_pick, 'outLcOk': n_ok, 'with_free_slots': n_free,
                          'policy_agreement_with_observed_pick': agree, 'attempts_with_heap_tail_observed': n_heap, 'tail_entry_kinds': tail_kinds, 'compact_prefix_model_checked': prefix_checked, 'compact_prefix_model_ok': prefix_ok, 'hook_records': len(hooks)})
    (a.out / 'attempts.jsonl').write_text(''.join(json.dumps(r) + '\n' for r in rows))
    (a.out / 'summary.json').write_text(json.dumps(summaries, indent=1) + '\n')
    print(json.dumps(summaries, indent=1))


if __name__ == '__main__':
    main()

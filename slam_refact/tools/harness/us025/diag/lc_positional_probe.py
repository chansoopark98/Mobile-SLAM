#!/usr/bin/env python3
"""US-025 LC positional probe (dev diagnostic, NOT a gate and NOT PASS evidence; it never changes a verdict).

Reads, offline and without running anything, an original (reference runtime) lc=1 run directory (`lcgold/events.jsonl`, via the pinned
compare_us025.load_original_lc) and the engine ON replay directory (`lc6a-attempts.jsonl`), and lays the two sides' loop close candidate
selection side by side, attempt by attempt:

  python3 -I -B lc_positional_probe.py --scenario-dir <G>/tum-room2-full --out-dir <out> [--run gate-lc1-03 ...]
  python3 -I -B lc_positional_probe.py --original <run dir> --engine <replay dir> --label <name> --out-dir <out>

--scenario-dir layout: runs/<scenario>/<run>/ (original), replays/<run>-on/ (engine), compare/<run>-full.json or -on.json (recorded, optional).

Model (engine loop_close.cc S3-S5, spec 12 U2): the live keyframes form a compressed list in slot order; the selection loop walks positions m = 0..n_slots-1,
skips m == newest.index, ids of 0, distance >= threshold, the effective ring ids (ring cleared when ringPre's last newest != current newest) and every
position m that is a member of the covisible exclusion set S2 (slot indices tested against the compressed POSITION, U2), keeps the first strictly smaller
distance. Neither side records S2. What the records do prove is a LOWER BOUND: for a pick at position p, S2 must contain every eligible position m with
dist[m] < dist[p] or (dist[m] == dist[p] and m < p) ("required S2"), and must not contain p. A position that is required on one side and picked on the other
is a witness that the two S2 sets differ (valid only when slot layout, newest index and descriptors are equal on both sides, which the probe checks).

Labels: every number here is VERIFIED only in the sense "computed from the recorded files now"; S2 sizes are lower bounds (INFERRED), the exact S2 is UNOBSERVED.
"""
import argparse
import importlib.util
import json
import pathlib
import sys

sys.dont_write_bytecode = True
HERE = pathlib.Path(__file__).resolve().parent
PINNED = HERE.parent / 'compare_us025.py'
THRESHOLD = 80.0          # WorldMaps.loopClosingDistanceThreshold default (engine config_defaults.inc:226); passed in, not read from the runs
MIN_SLOTS = 5             # spec 12 S1
EXIT = {'OK': 0, 'USAGE': 2, 'CROSSCHECK_DISAGREE': 3, 'INVALID_INPUT': 4}


class ProbeError(Exception):
    pass


def load_pinned():
    spec = importlib.util.spec_from_file_location('compare_us025_pinned', PINNED)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def hamming(a, b):
    return (int.from_bytes(a, 'little') ^ int.from_bytes(b, 'little')).bit_count()


# ---- side records: plain dicts so the analysis is testable with synthetic data ---------------------------------------------------------
def side_from_original(a, ev):
    """a: attempt dict of compare_us025.load_original_lc; ev: its lcgold event (for the per-slot id list)."""
    n = a['nSlots']
    by_s = {s['s']: s['id'] for s in ev['slots']}
    if len(by_s) != n or any(i not in by_s for i in range(n)):
        raise ProbeError('original seq %d: slot list does not cover slots 0..%d' % (a['seq'], n - 1))
    if len(a['desc']) < 32 * n:
        raise ProbeError('original seq %d: descriptor blob %d bytes < %d' % (a['seq'], len(a['desc']), 32 * n))
    return {'frame': a['frame'], 'n_slots': n, 'ids': [by_s[i] for i in range(n)], 'desc': [a['desc'][32 * i:32 * i + 32] for i in range(n)],
            'newest': list(a['newest']), 'ring_pre': list(a['ringPre']), 'ring_post': list(a['ringPost']), 'seq': a['seq'],
            'meta': {x['s']: {k: x.get(k) for k in ('id', 'kseq', 'created', 'orbN')} for x in ev['slots']}}


def side_from_engine(r):
    n = r['nSlots']
    if len(r['slots']) != n or len(r['desc']) != n:
        raise ProbeError('engine frame %d: slots/desc length != nSlots' % r['frame'])
    return {'frame': r['frame'], 'n_slots': n, 'ids': list(r['slots']), 'desc': [bytes.fromhex(x) for x in r['desc']], 'newest': list(r['newest']),
            'ring_pre': list(r['ringPre']), 'ring_post': list(r['ringPost']), 'cand_slot': r['cand'][0], 'cand_id': r['cand'][1],
            'cand_dist': r['candDist'], 'outcome': r['outcome']}


def analyse_side(s, threshold=THRESHOLD):
    """The selection model applied to one side's recorded state."""
    ids = s['ids']
    live_slots = [i for i, x in enumerate(ids) if x != 0]            # compressed list: position m -> slot index live_slots[m]
    live_ids = [ids[i] for i in live_slots]
    n_idx, n_id = s['newest']
    dist = None
    if 0 <= n_idx < s['n_slots'] and ids[n_idx] == n_id:
        nd = s['desc'][n_idx]
        dist = [hamming(s['desc'][i], nd) for i in live_slots]
    pre = s['ring_pre']
    reset = [pre[0], pre[1]] != [n_idx, n_id]
    ring_eff = [0, 0, 0] if reset else list(pre[2:5])
    post = s['ring_post']
    pick_id = post[2:5][(post[5] + 2) % 3]
    out = {'live_slots': live_slots, 'live_ids': live_ids, 'n_live': len(live_ids), 'dist': dist, 'ring_reset': reset, 'ring_eff': ring_eff,
           'pick_id': pick_id, 'pick_in_live': pick_id in live_ids if pick_id else None,
           'ring_nonlive': sorted(x for x in pre[2:5] if x and x not in live_ids), 'eligible': None, 'nos2_pos': None, 'pick_pos': None,
           'pick_slot': None, 'required': None, 'pick_dist': None, 'anomaly': None}
    if pick_id and pick_id in live_ids:
        out['pick_pos'] = live_ids.index(pick_id)
        out['pick_slot'] = live_slots[out['pick_pos']]
    if dist is None:
        out['anomaly'] = 'newest handle not live at its slot'
        return out
    elig = [m for m in range(len(live_ids)) if m != n_idx and dist[m] < threshold and live_ids[m] not in ring_eff]
    out['eligible'] = elig
    best, nos2 = threshold, None
    for m in elig:
        if dist[m] < best:
            best, nos2 = dist[m], m
    out['nos2_pos'] = nos2
    p = out['pick_pos']
    if pick_id == 0:
        out['required'] = list(elig)                                     # no candidate: every eligible position must have been excluded
    elif p is not None:
        out['pick_dist'] = dist[p]
        if p in elig:
            out['required'] = [m for m in elig if dist[m] < dist[p] or (dist[m] == dist[p] and m < p)]
        else:
            out['anomaly'] = 'pick position %d is not eligible (newest index / ring / threshold)' % p
    return out


def _pre_diff(o, e, ao, ae):
    d = []
    if o['n_slots'] != e['n_slots']:
        d.append('n_slots')
    if ao['live_slots'] != ae['live_slots']:
        d.append('live_slot_pattern')
    if o['newest'][0] != e['newest'][0]:
        d.append('newest_index')
    if ao['live_slots'] == ae['live_slots'] and any(o['desc'][i] != e['desc'][i] for i in ao['live_slots']):
        d.append('live_descriptors')
    return d


def compare_attempt(o, e, threshold=THRESHOLD, after_f=False):
    """One attempt, original side `o` and engine side `e` (both side_from_* dicts of the same frame). Returns the table row."""
    ao, ae = analyse_side(o, threshold), analyse_side(e, threshold)
    row = {'frame': o['frame'], 'after_F': after_f, 'n_slots': o['n_slots'], 'n_live_o': ao['n_live'], 'n_live_e': ae['n_live'],
           'newest_idx_o': o['newest'][0], 'newest_idx_e': e['newest'][0], 'newest_id_o': o['newest'][1], 'newest_id_e': e['newest'][1],
           'ids_equal': o['ids'] == e['ids'], 'ids_diff_slots': [i for i in range(min(len(o['ids']), len(e['ids']))) if o['ids'][i] != e['ids'][i]],
           'newest_equal': o['newest'] == e['newest'], 'ring_reset_o': ao['ring_reset'], 'ring_reset_e': ae['ring_reset'],
           'ring_pre_o': o['ring_pre'], 'ring_pre_e': e['ring_pre'], 'ring_post_o': o['ring_post'], 'ring_post_e': e['ring_post'],
           'ring_nonlive_o': ao['ring_nonlive'], 'ring_nonlive_e': ae['ring_nonlive'],
           'nos2_pos_o': ao['nos2_pos'], 'nos2_pos_e': ae['nos2_pos'], 'pick_id_o': ao['pick_id'], 'pick_id_e': ae['pick_id'],
           'pick_live_o': ao['pick_in_live'], 'pick_live_e': ae['pick_in_live'], 'pick_pos_o': ao['pick_pos'], 'pick_pos_e': ae['pick_pos'],
           'pick_slot_o': ao['pick_slot'], 'pick_slot_e': ae['pick_slot'], 'pick_dist_o': ao['pick_dist'], 'pick_dist_e': ae['pick_dist'],
           'dist_max_o': max(ao['dist']) if ao['dist'] else None, 'dist_min_o': min(ao['dist']) if ao['dist'] else None,
           'n_unique_live_desc_o': len({o['desc'][i] for i in ao['live_slots']}), 'n_unique_live_desc_e': len({e['desc'][i] for i in ae['live_slots']}),
           'engine_cand_slot': e.get('cand_slot'), 'engine_cand_id': e.get('cand_id'), 'engine_cand_dist': e.get('cand_dist'),
           'required_s2_size_o': len(ao['required']) if ao['required'] is not None else None,
           'required_s2_size_e': len(ae['required']) if ae['required'] is not None else None,
           'required_s2_o': ao['required'], 'required_s2_e': ae['required'], 'skipped_o': None, 'witness_pos': None, 'witness_s2_member_of': None,
           'pre_diff': [], 'notes': []}
    if ao['anomaly']:
        row['notes'].append('orig: ' + ao['anomaly'])
    if ae['anomaly']:
        row['notes'].append('engine: ' + ae['anomaly'])
    if ao['pick_pos'] is not None and ao['nos2_pos'] is not None:
        row['skipped_o'] = ao['pick_pos'] - ao['nos2_pos']
    row['pre_diff'] = _pre_diff(o, e, ao, ae)
    if o.get('meta') and ao['pick_slot'] is not None:               # original keyframe creation info for the slots up to max(pick, newest)
        row['slot_meta_o'] = [dict(slot=i, **o['meta'][i]) for i in range(max(ao['pick_slot'], o['newest'][0]) + 1) if i in o['meta']]
    if after_f:
        row['class'] = 'AFTER_F'                                          # N2: the original's state after its first success is undefined
        return row
    if o['n_slots'] < MIN_SLOTS:
        row['class'] = 'TOO_FEW_SLOTS'
        return row
    if row['pre_diff']:
        row['class'] = 'PRECONDITION_DIFF'
        return row
    po, pe = ao['pick_id'], ae['pick_id']
    same_id = po == pe
    if po != 0 and not ao['pick_in_live'] and not same_id:
        row['class'] = 'DECLARED_U2'                                      # original pick is not a live keyframe: uninitialized list tail (U2)
    elif same_id and po == 0:
        row['class'] = 'MATCH_NO_PICK'
    elif same_id:
        row['class'] = 'MATCH_FIRST' if ao['pick_pos'] == ao['nos2_pos'] else 'MATCH_SKIPPED'
    elif ao['pick_pos'] is not None and ao['pick_pos'] == ae['pick_pos']:
        row['class'] = 'POS_EQUAL_ID_DIFF'                                # same keyframe position, different handle id
    else:
        row['class'] = 'MISMATCH_SKIPPED' if (ao['pick_pos'] is not None and ao['pick_pos'] != ao['nos2_pos']) else 'MISMATCH_FIRST' if ao['pick_pos'] is not None else 'MISMATCH_OTHER'
    if row['class'].startswith('MISMATCH') or row['class'] == 'POS_EQUAL_ID_DIFF':
        cands = []
        if ae['pick_pos'] is not None and ao['required'] is not None and ae['pick_pos'] in ao['required']:
            cands.append((ae['pick_pos'], 'original'))
        if ao['pick_pos'] is not None and ae['required'] is not None and ao['pick_pos'] in ae['required']:
            cands.append((ao['pick_pos'], 'engine'))
        if cands and row['class'] != 'POS_EQUAL_ID_DIFF':
            row['witness_pos'], row['witness_s2_member_of'] = min(cands)
    return row


def build_rows(orig_sides, eng_sides, first_success_frame=None, threshold=THRESHOLD):
    """orig_sides / eng_sides: {frame: side}. One row per frame present on either side, frame order."""
    rows = []
    for f in sorted(set(orig_sides) | set(eng_sides)):
        after_f = first_success_frame is not None and f > first_success_frame
        if f not in orig_sides or f not in eng_sides:
            rows.append({'frame': f, 'class': 'AFTER_F' if after_f else 'ATTEMPT_SET_DIFF', 'after_F': after_f,
                         'notes': ['attempt on %s only' % ('original' if f in orig_sides else 'engine')]})
            continue
        rows.append(compare_attempt(orig_sides[f], eng_sides[f], threshold, after_f=after_f))
    return rows


def _first(rows, pred, key='frame'):
    for r in rows:
        if pred(r):
            return r[key]
    return None


def summarize(rows):
    """Run-level facts. The first-mismatch definition is the pinned comparator's candidateId one (ids differ, original pick is not a declared U2)."""
    cmp_rows = [r for r in rows if r['class'] not in ('AFTER_F', 'TOO_FEW_SLOTS', 'ATTEMPT_SET_DIFF')]
    classes = {}
    for r in rows:
        classes[r['class']] = classes.get(r['class'], 0) + 1
    id_mm = [r for r in cmp_rows if r['class'] in ('MISMATCH_SKIPPED', 'MISMATCH_FIRST', 'MISMATCH_OTHER', 'POS_EQUAL_ID_DIFF')]
    pos_mm = [r for r in cmp_rows if r['class'].startswith('MISMATCH')]
    skipped = [r for r in cmp_rows if r['class'] in ('MATCH_SKIPPED', 'MISMATCH_SKIPPED')]
    out = {'attempts': len(rows), 'classCounts': dict(sorted(classes.items())),
           'comparedAttempts': len(cmp_rows),
           'firstPickIdMismatchFrame': id_mm[0]['frame'] if id_mm else None,
           'firstPickPositionMismatchFrame': pos_mm[0]['frame'] if pos_mm else None,
           'firstPreconditionDiffFrame': _first(rows, lambda r: r['class'] == 'PRECONDITION_DIFF'),
           'firstNewestIdDiffFrame': _first(rows, lambda r: r.get('newest_equal') is False),
           'firstRingPostDiffFrame': _first(rows, lambda r: r.get('ring_post_o') is not None and r['ring_post_o'] != r['ring_post_e'] and not r['after_F']),
           'originalSkippedLivePick': {'attempts': len(skipped), 'engineMatches': sum(1 for r in skipped if r['class'] == 'MATCH_SKIPPED'),
                                       'engineMismatches': sum(1 for r in skipped if r['class'] == 'MISMATCH_SKIPPED')},
           'originalFirstPosition': sum(1 for r in cmp_rows if r['class'] == 'MATCH_FIRST'),
           'maxLiveDistance': max([r['dist_max_o'] for r in rows if r.get('dist_max_o') is not None] or [None]),
           'maxUniqueLiveDescriptors': max([r['n_unique_live_desc_o'] for r in rows if 'n_unique_live_desc_o' in r] or [0]),
           'preconditionDiffAttempts': classes.get('PRECONDITION_DIFF', 0),
           'ringResetEveryComparedAttempt': all(r['ring_reset_o'] and r['ring_reset_e'] for r in cmp_rows) if cmp_rows else None,
           'ringPreNonLiveNonzeroAttempts': sum(1 for r in cmp_rows if r['ring_nonlive_o'] or r['ring_nonlive_e']),
           'declaredU2Attempts': classes.get('DECLARED_U2', 0), 'witnesses': [{'frame': r['frame'], 'position': r['witness_pos'], 's2ContainsIt': r['witness_s2_member_of']}
                                                                            for r in cmp_rows if r['witness_pos'] is not None]}
    if id_mm:
        r = id_mm[0]
        out['firstMismatch'] = {k: r.get(k) for k in ('frame', 'class', 'n_slots', 'n_live_o', 'newest_idx_o', 'pick_id_o', 'pick_id_e', 'pick_pos_o', 'pick_pos_e', 'pick_slot_o',
                                                      'pick_slot_e', 'nos2_pos_o', 'nos2_pos_e', 'skipped_o', 'required_s2_size_o', 'required_s2_size_e', 'required_s2_o',
                                                      'required_s2_e', 'witness_pos', 'witness_s2_member_of', 'ring_reset_o', 'ring_reset_e', 'pick_dist_o', 'pick_dist_e',
                                                      'engine_cand_slot', 'engine_cand_dist', 'ring_pre_o', 'ring_pre_e', 'ring_post_o', 'ring_post_e', 'slot_meta_o', 'notes')}
        out['firstMismatch']['pickBeyondLive'] = None if r.get('pick_pos_o') is None else r['pick_pos_o'] >= r['n_live_o']
    return out


# ---- loading ------------------------------------------------------------------------------------------------------------------------
def load_sides(pinned, original_dir, engine_dir):
    orig = pinned.load_original_lc(original_dir, 1)
    eng = pinned.load_engine_lc(engine_dir)
    events = {e['seq']: e for e in pinned.jsonl(pathlib.Path(original_dir) / 'lcgold' / 'events.jsonl') if 'skipped' not in e}
    osides = {a['frame']: side_from_original(a, events[a['seq']]) for a in orig['attempts']}
    esides = {f: side_from_engine(r) for f, r in eng['attempts'].items()}
    return orig, eng, osides, esides


def cross_check(pinned, orig, eng, summary, rows):
    """The probe's counts against the pinned comparator's checks on the same loaded data (the whole run when the original never succeeded)."""
    o2 = dict(orig)
    if orig['firstSuccess'] is None:
        o2['firstSuccess'] = {'frame': 2 ** 30, 'seq': -1, 'outLcOk': 1, 'pick': 0, 'newest': [0, 0], 'outLcMat': b'', 'newestExtrinsic': b'', 'hooks': {}}
    checks, _ = pinned.judge_attempts(o2, eng)
    by = {c['name']: c for c in checks}
    cand = by['candidateId']
    comparable = [r for r in rows if r['class'] not in ('AFTER_F', 'ATTEMPT_SET_DIFF', 'TOO_FEW_SLOTS', 'PRECONDITION_DIFF')]
    probe = {'compared': len(comparable) + summary['preconditionDiffAttempts'],
             'mismatch': sum(1 for r in comparable if r['class'] in ('MISMATCH_SKIPPED', 'MISMATCH_FIRST', 'MISMATCH_OTHER', 'POS_EQUAL_ID_DIFF')),
             'declaredU2': sum(1 for r in comparable if r['class'] == 'DECLARED_U2'),
             'firstMismatchFrame': summary['firstPickIdMismatchFrame']}
    pin = {'compared': cand['compared'], 'mismatch': cand['mismatch'], 'declaredU2': cand['declaredU2'], 'firstMismatchFrame': (cand['first'] or {}).get('frame')}
    items = [{'check': 'candidateId', 'pinned': pin, 'probe': probe, 'agree': pin == probe}]
    for name, key in (('newestHandle', 'firstNewestIdDiffFrame'),):
        c = by[name]
        items.append({'check': name, 'pinned': {'mismatch': c['mismatch'], 'firstMismatchFrame': (c['first'] or {}).get('frame')},
                      'probe': {'firstMismatchFrame': summary[key]}, 'agree': (c['first'] or {}).get('frame') == summary[key]})
    c = by['slotLiveFree']
    pre = sum(1 for r in rows if r['class'] == 'PRECONDITION_DIFF')
    items.append({'check': 'slotLiveFree', 'pinned': {'mismatch': c['mismatch']}, 'probe': {'preconditionDiffAttempts': pre}, 'agree': (c['mismatch'] == 0) == (pre == 0)})
    for name in ('globalDescriptorLiveSlots', 'ringState'):
        c = by[name]
        items.append({'check': name, 'pinned': {'compared': c['compared'], 'mismatch': c['mismatch'], 'declaredU2': c['declaredU2'], 'firstMismatchFrame': (c['first'] or {}).get('frame')},
                      'probe': None, 'agree': None, 'note': 'recorded for reference; the probe has no counterpart of this exact check'})
    return {'items': items, 'allAgree': all(i['agree'] for i in items if i['agree'] is not None), 'pinnedComparator': str(PINNED)}


TSV_COLS = ['frame', 'class', 'after_F', 'n_slots', 'n_live_o', 'n_live_e', 'newest_idx_o', 'newest_idx_e', 'newest_id_o', 'newest_id_e', 'ids_equal', 'ids_diff_slots', 'pre_diff',
            'n_unique_live_desc_o', 'dist_max_o', 'ring_reset_o', 'ring_reset_e', 'ring_pre_o', 'ring_pre_e', 'ring_nonlive_o', 'nos2_pos_o', 'nos2_pos_e',
            'pick_id_o', 'pick_id_e', 'pick_live_o', 'pick_pos_o', 'pick_pos_e', 'pick_slot_o', 'pick_slot_e', 'pick_dist_o', 'pick_dist_e', 'engine_cand_slot', 'engine_cand_dist',
            'skipped_o', 'required_s2_size_o', 'required_s2_size_e', 'required_s2_o', 'required_s2_e', 'witness_pos', 'witness_s2_member_of', 'notes']


def _cell(v):
    if v is None:
        return ''
    if isinstance(v, (list, tuple)):
        return ';'.join(str(x) for x in v)
    return str(v)


def write_tsv(path, rows):
    with open(path, 'w') as f:
        f.write('\t'.join(TSV_COLS) + '\n')
        for r in rows:
            f.write('\t'.join(_cell(r.get(c)) for c in TSV_COLS) + '\n')


def run_one(pinned, label, original_dir, engine_dir, out_dir, compare_json=None, threshold=THRESHOLD):
    orig, eng, osides, esides = load_sides(pinned, original_dir, engine_dir)
    f_succ = orig['firstSuccess']['frame'] if orig['firstSuccess'] else None
    rows = build_rows(osides, esides, f_succ, threshold)
    summ = summarize(rows)
    xc = cross_check(pinned, orig, eng, summ, rows)
    result = {'schema': 'slam-refact-us025-lc-positional-probe-v1', 'label': label, 'isGateVerdict': False,
              'note': 'dev diagnostic; does not change any gate verdict and is not PASS evidence', 'threshold': threshold,
              'inputs': {'originalRun': str(original_dir), 'engineReplay': str(engine_dir), 'originalFirstSuccessFrame': f_succ,
                         'originalHookMode': orig['hooked'], 'engineAttempts': len(eng['attempts']), 'originalAttempts': len(orig['attempts'])},
              'summary': summ, 'crossCheckPinnedComparator': xc, 'rows': rows}
    if compare_json and pathlib.Path(compare_json).exists():
        cj = json.loads(pathlib.Path(compare_json).read_text())
        tr = next((c for c in cj.get('checks', []) if c['name'].startswith('traceAndWorldPoints')), None)
        result['recordedTraceComparison'] = {'file': str(compare_json), 'verdict': cj.get('verdict'), 'traceCheck': {k: tr.get(k) for k in ('name', 'bitIdentical', 'mismatch', 'compared')} if tr else None,
                                             'mismatchFrames': (tr or {}).get('D', {}).get('mismatchFrames') if tr else None, 'label': 'RECORDED (pinned comparator output, not recomputed)'}
    out = pathlib.Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    write_tsv(out / ('%s.attempts.tsv' % label), rows)
    (out / ('%s.json' % label)).write_text(json.dumps(result, indent=1, default=str) + '\n')
    return result


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--scenario-dir')
    ap.add_argument('--run', action='append', help='run names inside --scenario-dir (default: every runs/<scenario>/*lc1-*)')
    ap.add_argument('--original')
    ap.add_argument('--engine')
    ap.add_argument('--label')
    ap.add_argument('--compare-json')
    ap.add_argument('--out-dir', required=True)
    ap.add_argument('--threshold', type=float, default=THRESHOLD)
    a = ap.parse_args(argv)
    jobs = []
    if a.scenario_dir:
        sd = pathlib.Path(a.scenario_dir)
        rd = sd / 'runs' / sd.name
        names = a.run or sorted(p.name for p in rd.iterdir() if p.is_dir() and 'lc1-' in p.name)
        for n in names:
            cj = next((p for p in (sd / 'compare' / ('%s-full.json' % n), sd / 'compare' / ('%s-on.json' % n)) if p.exists()), None)
            jobs.append((n, rd / n, sd / 'replays' / ('%s-on' % n), cj))
    elif a.original and a.engine and a.label:
        jobs.append((a.label, pathlib.Path(a.original), pathlib.Path(a.engine), a.compare_json))
    else:
        ap.error('give --scenario-dir, or --original --engine --label')
    pinned = load_pinned()
    rc = EXIT['OK']
    summary_rows = []
    for label, o, e, cj in jobs:
        try:
            r = run_one(pinned, label, o, e, a.out_dir, cj, a.threshold)
        except (ProbeError, pinned.Invalid, OSError, KeyError, ValueError) as ex:
            print('%s: INVALID_INPUT %s: %s' % (label, type(ex).__name__, ex), file=sys.stderr)
            rc = max(rc, EXIT['INVALID_INPUT'])
            continue
        s = r['summary']
        print('%s: attempts %d classes %s firstPickIdMismatchFrame %s firstPositionMismatchFrame %s crossCheck %s' %
              (label, s['attempts'], s['classCounts'], s['firstPickIdMismatchFrame'], s['firstPickPositionMismatchFrame'],
               'AGREE' if r['crossCheckPinnedComparator']['allAgree'] else 'DISAGREE'))
        summary_rows.append([label, s['attempts'], s['comparedAttempts'], s['firstPickIdMismatchFrame'], s['firstPickPositionMismatchFrame'], s['firstNewestIdDiffFrame'],
                             s['originalFirstPosition'], s['originalSkippedLivePick']['attempts'], s['originalSkippedLivePick']['engineMatches'],
                             s['classCounts'].get('MATCH_NO_PICK', 0), s['declaredU2Attempts'], s['preconditionDiffAttempts'], s['maxUniqueLiveDescriptors'], s['maxLiveDistance'],
                             'AGREE' if r['crossCheckPinnedComparator']['allAgree'] else 'DISAGREE'])
        if not r['crossCheckPinnedComparator']['allAgree']:
            rc = max(rc, EXIT['CROSSCHECK_DISAGREE'])
    if summary_rows:
        cols = ['run', 'attempts', 'comparedAttempts', 'firstPickIdMismatchFrame', 'firstPickPositionMismatchFrame', 'firstNewestIdDiffFrame', 'matchFirstPosition',
                'origSkippedLivePick', 'origSkippedEngineMatches', 'matchNoPick', 'declaredU2', 'preconditionDiff', 'maxUniqueLiveDesc', 'maxLiveDistance', 'crossCheckPinned']
        with open(pathlib.Path(a.out_dir) / 'summary.tsv', 'w') as f:
            f.write('\t'.join(cols) + '\n')
            for row in summary_rows:
                f.write('\t'.join(_cell(x) for x in row) + '\n')
    return rc


if __name__ == '__main__':
    sys.exit(main())

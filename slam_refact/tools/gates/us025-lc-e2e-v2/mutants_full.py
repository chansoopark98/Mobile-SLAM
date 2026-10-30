#!/usr/bin/env python3
"""us025-lc-e2e-gate.v2 detection power of the whole-run comparison (compare_full_us025.py, D-V2-2). DEV EVIDENCE tool: it reads existing runs read-only and writes only below --out.

  python3 -B -I mutants_full.py --out <new dir> <name>=<original lc=1 run dir WITHOUT a loop close success>,<engine ON replay dir> [...]

Per run: (a) the unmutated pair must PASS / PASS_DECLARED (the null: the engine ON replay of a real run against its own original over the whole run, floor 0);
(b) each mutant changes ONE quantity of a COPY of the engine replay directory and the whole-run comparison must be FAIL with at least one check firing; for every mutant
the F-limited comparator of v1 (compare_us025.arm_on) is run on the same copy and its verdict is recorded (it answers BLOCKED for a run without an original success:
the blindness of v1 that D-V2-2 closes); (c) controls: the engine OFF replay is not an ON replay (INVALID or FAIL, never PASS), another run's replay is not this run's.
Mutations follow compare_us025's own mutants (lc6a/mutants_lc6a.py, which is not imported): candidate id / ring word, ring cursor, slot count, live count, dropped attempt,
extra attempt, descriptor byte, stage class, trace pose 1 ulp, trace integer +1, and the new one this arm exists for: an engine loop close SUCCESS where the original had none.
Exit 0 when every expectation holds, 10 otherwise."""
import argparse
import importlib.util
import json
import pathlib
import shutil
import struct
import sys

sys.dont_write_bytecode = True
HERE = pathlib.Path(__file__).resolve().parent
_spec = importlib.util.spec_from_file_location('compare_full_us025', HERE / 'compare_full_us025.py')
F = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(F)
C = F.C


def next_f32(x, steps):
    b = struct.unpack('<I', struct.pack('<f', x))[0]
    return struct.unpack('<f', struct.pack('<I', b + steps))[0]


def flip_hex_nibble(h, pos=0):
    return h[:pos] + '%x' % (int(h[pos], 16) ^ 1) + h[pos + 1:]


def rows_of(d):
    return [json.loads(line) for line in (d / 'lc6a-attempts.jsonl').read_text().splitlines() if line.strip()]


def save_rows(d, rows):
    (d / 'lc6a-attempts.jsonl').write_text(''.join(json.dumps(r) + '\n' for r in rows))


def pick_row(rows, pred, last=False):
    hit = [r for r in rows if pred(r)]
    if not hit:
        raise StopIteration
    return hit[-1] if last else hit[0]


def mutate_attempts(kind, d, orig):
    rows = rows_of(d)
    if kind == 'ring-word':
        r = pick_row(rows, lambda r: r['nSlots'] >= C.MIN_SLOTS and r['ringPost'][2] != 0)
        r['ringPost'][2] ^= 1
    elif kind == 'ring-cursor':
        r = pick_row(rows, lambda r: r['nSlots'] >= C.MIN_SLOTS)
        r['ringPost'][5] = 2 if r['ringPost'][5] != 2 else 1
    elif kind == 'slot-count':
        pick_row(rows, lambda r: True)['nSlots'] += 1
    elif kind == 'live-count':
        pick_row(rows, lambda r: r['nLive'] > 0)['nLive'] -= 1
    elif kind == 'drop-first-attempt':
        rows.remove(pick_row(rows, lambda r: True))
    elif kind == 'drop-last-attempt':
        rows.remove(rows[-1])
    elif kind == 'extra-attempt':
        extra = dict(rows[-1])
        extra['frame'] = rows[-1]['frame'] + 1
        rows.append(extra)
    elif kind == 'descriptor-byte':
        r = pick_row(rows, lambda r: r['nLive'] > 0)
        r['desc'][0] = flip_hex_nibble(r['desc'][0], 10)
    elif kind == 'descriptor-byte-last-attempt':
        r = pick_row(rows, lambda r: r['nLive'] > 0, last=True)
        r['desc'][-1] = flip_hex_nibble(r['desc'][-1], 10)
    elif kind in ('carried-u2-word', 'cursor-at-u2-attempt'):
        # carried-u2-word: tamper with the engine ring word at the LAST attempt where the U2 pair is only carried (its pick is not a declared-U2 pick);
        # cursor-at-u2-attempt: change the engine cursor at the LAST attempt that differs in the ring at all (v1 declares every ring difference at a declared-U2 attempt)
        by = {x['frame']: x for x in rows}
        def declared(x, r):
            return x['nSlots'] >= C.MIN_SLOTS and x['pick'] != 0 and x['pick'] not in x['liveIds'] and r['ringPost'][2:5][(r['ringPost'][5] + 2) % 3] != x['pick']
        hit = [x for x in orig['attempts'] if x['frame'] in by and x['ringPost'] != by[x['frame']]['ringPost']]
        if kind == 'carried-u2-word':
            hit = [x for x in hit if not declared(x, by[x['frame']])]
        if not hit:
            raise StopIteration
        r = by[hit[-1]['frame']]
        if kind == 'cursor-at-u2-attempt':
            r['ringPost'][5] = (r['ringPost'][5] + 1) % 3
        else:
            k = next(k for k in (2, 3, 4) if hit[-1]['ringPost'][k] != r['ringPost'][k])
            r['ringPost'][k] ^= 1
    elif kind in ('non-pick-slot-at-u2-attempt', 'cursor-propagate'):
        by = {x['frame']: x for x in rows}
        def declared_attempt(x):
            r = by.get(x['frame'])
            return r is not None and x['nSlots'] >= C.MIN_SLOTS and x['pick'] != 0 and x['pick'] not in x['liveIds'] and r['ringPost'][2:5][(r['ringPost'][5] + 2) % 3] != x['pick']
        dec = [x for x in orig['attempts'] if declared_attempt(x)]
        if not dec:
            raise StopIteration
        if kind == 'non-pick-slot-at-u2-attempt':
            # a ring word that is NOT the pick word, changed at the last declared-U2 attempt (pre-review C2: must be a mismatch, the exemption is the pick word only)
            x = dec[-1]
            r = by[x['frame']]
            k = next(k for k in (2, 3, 4) if k != 2 + (x['ringPost'][5] + 2) % 3)
            r['ringPost'][k] ^= 1
        else:
            # a cursor defect that STARTS at the first declared-U2 attempt and persists in every later record (pre-review R1)
            f0 = dec[0]['frame']
            for r in rows:
                if r['frame'] >= f0:
                    r['ringPost'][5] = (r['ringPost'][5] + 1) % 3
    elif kind == 'stage-class':
        hit = [r for r in rows if r['nSlots'] >= C.MIN_SLOTS and r['outcome'] == 1]
        if not hit:
            raise StopIteration
        for r in hit:
            r['outcome'] = 3
    elif kind == 'engine-success-early':     # the engine reports a loop close success the original never made
        r = pick_row(rows, lambda r: r['nSlots'] >= C.MIN_SLOTS)
        r['applied'], r['outcome'], r['outLcOk'] = True, 6, 1
    elif kind == 'engine-success-last':
        r = pick_row(rows, lambda r: r['nSlots'] >= C.MIN_SLOTS, last=True)
        r['applied'], r['outcome'], r['outLcOk'] = True, 6, 1
    elif kind == 'engine-success-flag-only':  # only the success flag: every other quantity of the attempt untouched
        rows[-1]['applied'] = True
    else:
        raise KeyError(kind)
    save_rows(d, rows)


def mutate_trace(kind, d):
    tr = [json.loads(line) for line in (d / 'trace.jsonl').read_text().splitlines() if line.strip()]
    fresh = [x for x in tr if x.get('i') is not None and x.get('fresh') and x.get('p')]
    if not fresh:
        raise StopIteration
    if kind == 'trace-pose-1ulp-first':
        r = fresh[0]
        r['p'][0] = next_f32(r['p'][0], 1)
    elif kind == 'trace-pose-1ulp-last':
        r = fresh[-1]
        r['p'][0] = next_f32(r['p'][0], 1)
    elif kind == 'trace-integer-last':
        fresh[-1]['worldPoints'] += 1
    else:
        raise KeyError(kind)
    (d / 'trace.jsonl').write_text(''.join(json.dumps(x) + '\n' for x in tr))


ATTEMPT_KINDS = ['ring-word', 'ring-cursor', 'slot-count', 'live-count', 'drop-first-attempt', 'drop-last-attempt', 'extra-attempt', 'descriptor-byte',
                 'descriptor-byte-last-attempt', 'carried-u2-word', 'cursor-at-u2-attempt', 'non-pick-slot-at-u2-attempt', 'cursor-propagate', 'stage-class', 'engine-success-early', 'engine-success-last', 'engine-success-flag-only']
TRACE_KINDS = ['trace-pose-1ulp-first', 'trace-pose-1ulp-last', 'trace-integer-last']


def fired(res):
    return [c['name'] for c in res.get('checks', []) if c.get('mismatch')]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', type=pathlib.Path, required=True)
    ap.add_argument('--off', action='append', default=[], help='<name>=<engine OFF replay dir of that run> (control: an OFF replay is not an ON replay)')
    ap.add_argument('--other', action='append', default=[], help='<name>=<engine ON replay dir of ANOTHER run> (control: a wrong pairing)')
    ap.add_argument('--success-original', action='append', default=[], help='<name>=<original run dir WITH a success>,<engine ON replay dir> (control: not applicable, BLOCKED)')
    ap.add_argument('run', nargs='+', help='<name>=<original run dir>,<engine ON replay dir>')
    a = ap.parse_args()
    if a.out.exists():
        raise SystemExit('refusing to overwrite ' + str(a.out))
    a.out.mkdir(parents=True)
    table, ok_all = [], True
    runs = {}
    for spec_ in a.run:
        name, dirs = spec_.split('=', 1)
        orig_dir, eng_dir = (pathlib.Path(x) for x in dirs.split(','))
        runs[name] = (orig_dir, eng_dir)
        base = F.arm_full(orig_dir, eng_dir)
        ok = base['verdict'] in ('PASS', 'PASS_DECLARED')
        table.append({'run': name, 'mutant': '(none: null)', 'verdict': base['verdict'], 'v1ArmOn': C.arm_on(orig_dir, eng_dir)['verdict'], 'expected': 'PASS*', 'fired': fired(base), 'ok': ok,
                      'declaredU2': sum(c.get('declaredU2', 0) for c in base.get('checks', []))})
        ok_all &= ok
        orig = C.load_original_lc(orig_dir, 1)
        for kind in ATTEMPT_KINDS + TRACE_KINDS:
            m = a.out / name / kind
            m.mkdir(parents=True)
            for f in ('meta.json', 'trace.jsonl', 'worldpoints-sampled.jsonl', 'lc6a-attempts.jsonl'):
                shutil.copyfile(eng_dir / f, m / f)
            try:
                mutate_trace(kind, m) if kind in TRACE_KINDS else mutate_attempts(kind, m, orig)
            except StopIteration:
                table.append({'run': name, 'mutant': kind, 'verdict': 'NOT-APPLICABLE', 'expected': '-', 'fired': [], 'ok': True, 'note': 'no suitable attempt / frame in this run'})
                continue
            res = F.arm_full(orig_dir, m)
            v1 = C.arm_on(orig_dir, m)['verdict']
            if kind in ('cursor-at-u2-attempt', 'cursor-propagate'):   # RECORDED, not asserted: the cursor is exempt at a declared-U2 attempt; cursor-propagate shows whether a persisting cursor defect is caught afterwards (pre-review R1)
                hole = res['verdict'] != 'FAIL'
                table.append({'run': name, 'mutant': kind, 'verdict': res['verdict'], 'v1ArmOn': v1, 'expected': 'FAIL|HOLE', 'fired': fired(res), 'ok': True, 'undetected': hole})
                continue
            ok = res['verdict'] == 'FAIL' and bool(fired(res))
            ok_all &= ok
            table.append({'run': name, 'mutant': kind, 'verdict': res['verdict'], 'v1ArmOn': v1, 'expected': 'FAIL', 'fired': fired(res), 'ok': ok})
    for spec_ in a.off:
        name, d = spec_.split('=', 1)
        res = F.arm_full(runs[name][0], pathlib.Path(d))
        ok = res['verdict'] in ('INVALID', 'FAIL')
        ok_all &= ok
        table.append({'run': name, 'mutant': 'control: engine OFF replay as the ON replay', 'verdict': res['verdict'], 'expected': 'INVALID|FAIL', 'fired': fired(res), 'ok': ok,
                      'reason': res.get('reason')})
    for spec_ in a.other:
        name, d = spec_.split('=', 1)
        res = F.arm_full(runs[name][0], pathlib.Path(d))
        ok = res['verdict'] in ('INVALID', 'FAIL')
        ok_all &= ok
        table.append({'run': name, 'mutant': 'control: replay of another run', 'verdict': res['verdict'], 'expected': 'INVALID|FAIL', 'fired': fired(res), 'ok': ok, 'reason': res.get('reason')})
    for spec_ in a.success_original:
        name, dirs = spec_.split('=', 1)
        o, e = (pathlib.Path(x) for x in dirs.split(','))
        res = F.arm_full(o, e)
        ok = res['verdict'] == 'BLOCKED'
        ok_all &= ok
        table.append({'run': name, 'mutant': 'control: original WITH a success (not applicable)', 'verdict': res['verdict'], 'expected': 'BLOCKED', 'fired': [], 'ok': ok, 'reason': res.get('reason')})
    (a.out / 'mutants.json').write_text(json.dumps({'schema': 'slam-refact-us025-lc-e2e-v2-full-mutants-v1', 'allExpectationsHold': ok_all, 'table': table}, indent=1) + '\n')
    for t in table:
        print('%-28s %-52s %-14s v1:%-8s expected %-12s %s %s' % (t['run'], t['mutant'], t['verdict'], t.get('v1ArmOn', '-'), t['expected'], 'ok ' if t['ok'] else 'BAD', ','.join(t['fired'])))
    print('ALL EXPECTATIONS HOLD' if ok_all else 'EXPECTATION VIOLATED')
    return 0 if ok_all else 10


if __name__ == '__main__':
    sys.exit(main())

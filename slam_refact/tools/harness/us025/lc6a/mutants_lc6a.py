#!/usr/bin/env python3
"""US-025 LC-6a detection power: each mutation changes ONE quantity of a real engine replay directory (a mutated copy; the replay and the original run are
never touched) and the ON comparison must turn FAIL. The two PnP boundary cases check the declared 3 ulp tolerance (3 ulp stays PASS, 4 ulp FAILs).
  python3 -B -I mutants_lc6a.py --out <new dir> <name>=<original run dir>,<engine ON replay dir> [...]
Writes <out>/mutants.json (per run and mutant: verdict, the check(s) that fired) and prints a table. Exit 0 when every expectation holds, 10 otherwise."""
import argparse
import importlib.util
import json
import pathlib
import shutil
import struct
import sys

sys.dont_write_bytecode = True
HERE = pathlib.Path(__file__).resolve().parent
spec = importlib.util.spec_from_file_location('compare_us025', HERE.parent / 'compare_us025.py')
C = importlib.util.module_from_spec(spec)
spec.loader.exec_module(C)


def next_f32(x, steps):
    b = struct.unpack('<I', struct.pack('<f', x))[0]
    return struct.unpack('<f', struct.pack('<I', b + steps))[0]


def flip_hex_nibble(h, pos=0):
    return h[:pos] + '%x' % (int(h[pos], 16) ^ 1) + h[pos + 1:]


def load_attempts(d):
    return [json.loads(l) for l in (d / 'lc6a-attempts.jsonl').read_text().splitlines() if l.strip()]


def save_attempts(d, rows):
    (d / 'lc6a-attempts.jsonl').write_text(''.join(json.dumps(r) + '\n' for r in rows))


def first(rows, pred):
    return next(r for r in rows if pred(r))


def mutate(kind, d, F, hooked):
    rows = load_attempts(d)
    if kind == 'candidate-id':
        r = first(rows, lambda r: r['frame'] < F and r['nSlots'] >= 5 and r['ringPost'][2] != 0)
        r['ringPost'][2] ^= 1
    elif kind == 'ring-cursor':
        r = first(rows, lambda r: r['frame'] < F and r['nSlots'] >= 5)
        r['ringPost'][5] = (r['ringPost'][5] + 1) % 3
    elif kind == 'slot-count':
        first(rows, lambda r: r['frame'] < F)['nSlots'] += 1
    elif kind == 'live-count':
        first(rows, lambda r: r['frame'] < F and r['nLive'] > 0)['nLive'] -= 1
    elif kind == 'drop-attempt':
        rows.remove(first(rows, lambda r: r['frame'] < F))
    elif kind == 'descriptor-byte':
        r = first(rows, lambda r: r['frame'] < F and r['nLive'] > 0)
        r['desc'][0] = flip_hex_nibble(r['desc'][0], 10)
    elif kind == 'stage-class':
        hit = [r for r in rows if r['frame'] < F and r['nSlots'] >= 5 and r['outcome'] == 1]   # all of them: one may be a declared-U2 attempt
        if not hit:
            raise StopIteration
        for r in hit:
            r['outcome'] = 3
    elif kind == 'engine-success-missing':
        r = first(rows, lambda r: r['frame'] == F)
        r['applied'], r['outcome'] = False, 3
    elif kind == 'out-0x1c':
        first(rows, lambda r: r['frame'] == F)['outLcOk'] = 0
    elif kind == 'C-one-bit':
        r = first(rows, lambda r: r['frame'] == F)
        r['C'] = flip_hex_nibble(r['C'], 20)
    elif kind in ('pnp-3ulp', 'pnp-4ulp'):
        r = first(rows, lambda r: r['frame'] == F)
        b = bytes.fromhex(r['poseAfter'])
        x = struct.unpack('<f', b[0:4])[0]
        r['poseAfter'] = (struct.pack('<f', next_f32(x, 3 if kind == 'pnp-3ulp' else 4)) + b[4:]).hex()
    elif kind == 'match-count':
        r = first(rows, lambda r: r['frame'] <= F and r['outcome'] >= 3)
        r['nMatches'] += 1
    elif kind == 'corr-count':
        r = first(rows, lambda r: r['frame'] <= F and r['outcome'] >= 4)
        r['nCorr'] += 1
    elif kind == 'p-old':
        r = first(rows, lambda r: r['frame'] == F)
        r['poseBefore'] = flip_hex_nibble(r['poseBefore'], 20)
    else:
        raise KeyError(kind)
    save_attempts(d, rows)


def mutate_trace(kind, d, F):
    tr = [json.loads(l) for l in (d / 'trace.jsonl').read_text().splitlines() if l.strip()]
    r = [x for x in tr if x.get('i') is not None and x['i'] < F and x.get('fresh') and x.get('p')][-1]
    if kind == 'trace-pose-1ulp':
        r['p'][0] = next_f32(r['p'][0], 1)
    elif kind == 'trace-integer':
        r['worldPoints'] += 1
    else:
        raise KeyError(kind)
    (d / 'trace.jsonl').write_text(''.join(json.dumps(x) + '\n' for x in tr))


ATTEMPT_KINDS = ['candidate-id', 'ring-cursor', 'slot-count', 'live-count', 'drop-attempt', 'descriptor-byte', 'stage-class', 'engine-success-missing', 'out-0x1c',
                 'C-one-bit', 'pnp-3ulp', 'pnp-4ulp', 'match-count', 'corr-count', 'p-old']
TRACE_KINDS = ['trace-pose-1ulp', 'trace-integer']
EXPECT_PASS = {'pnp-3ulp'}      # inside the declared PnP tolerance
HOOK_ONLY = {'match-count', 'corr-count', 'p-old'}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', type=pathlib.Path, required=True)
    ap.add_argument('run', nargs='+', help='<name>=<original run dir>,<engine ON replay dir>')
    a = ap.parse_args()
    if a.out.exists():
        raise SystemExit('refusing to overwrite ' + str(a.out))
    a.out.mkdir(parents=True)
    table, ok_all = [], True
    for spec_ in a.run:
        name, dirs = spec_.split('=', 1)
        orig_dir, eng_dir = (pathlib.Path(x) for x in dirs.split(','))
        base = C.arm_on(orig_dir, eng_dir)
        orig = C.load_original_lc(orig_dir, 1)
        F, hooked = orig['firstSuccess']['frame'], orig['hooked']
        table.append({'run': name, 'mutant': '(none)', 'verdict': base['verdict'], 'expected': 'PASS*', 'fired': [], 'ok': base['verdict'] in ('PASS', 'PASS_DECLARED')})
        for kind in ATTEMPT_KINDS + TRACE_KINDS:
            if kind in HOOK_ONLY and not hooked:
                continue
            m = a.out / name / kind
            m.mkdir(parents=True)
            for f in ('meta.json', 'trace.jsonl', 'worldpoints-sampled.jsonl', 'lc6a-attempts.jsonl'):
                shutil.copyfile(eng_dir / f, m / f)
            try:
                (mutate_trace if kind in TRACE_KINDS else mutate)(*(kind, m, F) if kind in TRACE_KINDS else (kind, m, F, hooked))
            except StopIteration:
                table.append({'run': name, 'mutant': kind, 'verdict': 'NOT-APPLICABLE', 'expected': '-', 'fired': [], 'ok': True, 'note': 'no suitable attempt in this run'})
                continue
            res = C.arm_on(orig_dir, m)
            fired = [c['name'] for c in res.get('checks', []) if c.get('mismatch')]
            want_pass = kind in EXPECT_PASS
            ok = res['verdict'] in ('PASS', 'PASS_DECLARED') if want_pass else res['verdict'] == 'FAIL' and bool(fired)
            ok_all &= ok
            table.append({'run': name, 'mutant': kind, 'verdict': res['verdict'], 'expected': 'PASS' if want_pass else 'FAIL', 'fired': fired, 'ok': ok})
    ok_all &= all(t['ok'] for t in table)
    (a.out / 'mutants.json').write_text(json.dumps({'schema': 'slam-refact-us025-lc6a-mutants-v1', 'allExpectationsHold': ok_all, 'table': table}, indent=1) + '\n')
    for t in table:
        print('%-10s %-24s %-14s expected %-5s %s %s' % (t['run'], t['mutant'], t['verdict'], t['expected'], 'ok ' if t['ok'] else 'BAD', ','.join(t['fired'])))
    print('ALL EXPECTATIONS HOLD' if ok_all else 'EXPECTATION VIOLATED')
    return 0 if ok_all else 10


if __name__ == '__main__':
    sys.exit(main())

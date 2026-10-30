#!/usr/bin/env python3
"""G6 / RM-3b criterion 5: helpers for the original-vs-engine recenter probe (REPORT only, no verdict).

  python3 -I pipeline.py actions --probe <run>/recenter-probe.json --out <actions.tsv>
      Writes the engine call script from the calls the ORIGINAL run actually executed (frame = atFrameActual).
      Only the call shapes the engine harness supports are accepted: recenter, configure({}), update({origin[, updateRecenterPoint]}).
  python3 -I pipeline.py compare --orig <run dir> --engine <engine run dir> --out <compare.json> [--md <table.md>]
      Per input frame: original (trace.jsonl, first fresh callback of the frame) against the engine (rows.jsonl) position / rotation,
      float32 bit equality and the max abs difference; per executed call a window of frames before and after the call.
"""
import argparse, json, pathlib, struct, sys


def f32(x):
    return struct.unpack('<f', struct.pack('<f', x))[0]


def cmd_actions(a):
    d = json.loads(pathlib.Path(a.probe).read_text())
    lines = ['# frame\tfn\tox\toy\toz\tupdate_recenter_point']
    for act in d['actions']:
        for c in act['calls']:
            if not c['ok']:
                sys.exit('original call failed: %s %s' % (act['id'], c))
            if c['fn'] == 'recenter':
                lines.append('%d\trecenter\t0\t0\t0\t1' % act['atFrameActual'])
            elif c['fn'] == 'configure':
                if c['args'] != {}:
                    sys.exit('configure args other than {} are not supported: %r' % c['args'])
                lines.append('%d\tconfigure\t0\t0\t0\t1' % act['atFrameActual'])
            elif c['fn'] == 'update':
                args = dict(c['args'])
                o = args.pop('origin')
                rcp = args.pop('updateRecenterPoint', True)
                if args:
                    sys.exit('update args not supported: %r' % args)
                lines.append('%d\tupdate\t%r\t%r\t%r\t%d' % (act['atFrameActual'], float(o['x']), float(o['y']), float(o['z']), 1 if rcp else 0))
            else:
                sys.exit('unknown fn ' + c['fn'])
    pathlib.Path(a.out).write_text('\n'.join(lines) + '\n')
    print('wrote', a.out, len(lines) - 1, 'calls')


def load_orig(run):
    run = pathlib.Path(run)
    rows = {}
    for line in open(run / 'trace.jsonl'):
        r = json.loads(line)
        rows[r['i']] = r
    probe = json.loads((run / 'recenter-probe.json').read_text())
    return rows, probe


def load_engine(run):
    run = pathlib.Path(run)
    rows = {}
    for line in open(run / 'rows.jsonl'):
        r = json.loads(line)
        rows[r['i']] = r
    calls = [json.loads(l) for l in open(run / 'calls.jsonl')]
    return rows, calls


def cmd_compare(a):
    orows, probe = load_orig(a.orig)
    erows, ecalls = load_engine(a.engine)
    frames = sorted(set(orows) & set(erows))
    per = {}
    for i in frames:
        o, e = orows[i], erows[i]
        if not o['fresh'] or not e.get('processed') or o['p'] is None:
            per[i] = None
            continue
        dp = [abs(x - y) for x, y in zip(o['p'], e['p'])]
        dq = [abs(x - y) for x, y in zip(o['q_wxyz'], e['q_wxyz'])]
        bit = all(f32(x) == f32(y) for x, y in zip(o['p'] + o['q_wxyz'], e['p'] + e['q_wxyz']))
        per[i] = {'o_p': o['p'], 'e_p': e['p'], 'max_dp': max(dp), 'max_dq': max(dq), 'bit_equal': bit, 'dy': e['p'][1] - o['p'][1],
                  'o_status': o['status'], 'e_status': e['status'], 'e_repeated': e['repeated']}
    cmpd = [i for i in frames if per[i]]
    first_diff = next((i for i in cmpd if not per[i]['bit_equal']), None)
    res = {'schema': 'rm3b-recenter-compare-v1', 'frames_compared': len(cmpd), 'frames_bit_equal': sum(per[i]['bit_equal'] for i in cmpd),
           'first_non_bit_equal_frame': first_diff, 'max_dp_all': max(per[i]['max_dp'] for i in cmpd), 'max_dq_all': max(per[i]['max_dq'] for i in cmpd),
           'max_abs_dy_all': max(abs(per[i]['dy']) for i in cmpd),
           'engine_calls': ecalls, 'original_actions': probe['actions'], 'windows': []}
    for act in probe['actions']:
        F = act['atFrameActual']
        w = []
        for i in range(F - 1, F + 7):
            if per.get(i):
                p = per[i]
                w.append({'i': i, 'o_p': p['o_p'], 'e_p': p['e_p'], 'dy': p['dy'], 'max_dp': p['max_dp'], 'bit_equal': p['bit_equal'], 'e_repeated': p['e_repeated']})
        res['windows'].append({'id': act['id'], 'frame': F, 'calls': [c['fn'] for c in act['calls']], 'rows': w})
    pathlib.Path(a.out).write_text(json.dumps(res, indent=1) + '\n')
    print(json.dumps({k: res[k] for k in ('frames_compared', 'frames_bit_equal', 'first_non_bit_equal_frame', 'max_dp_all', 'max_dq_all', 'max_abs_dy_all')}))
    if a.md:
        L = ['| action | frame i | call | orig p (x, y, z) | engine p (x, y, z) | dy (engine - orig) | max abs dp | bit equal |', '|---|---|---|---|---|---|---|---|']
        for win in res['windows']:
            for r in win['rows']:
                mark = ' <- call after this frame' if r['i'] == win['frame'] else ''
                L.append('| %s | %d%s | %s | %s | %s | %.3g | %.3g | %s |' % (win['id'], r['i'], mark, '+'.join(win['calls']),
                         ', '.join('%.6f' % v for v in r['o_p']), ', '.join('%.6f' % v for v in r['e_p']), r['dy'], r['max_dp'], r['bit_equal']))
        pathlib.Path(a.md).write_text('\n'.join(L) + '\n')


if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    sub = ap.add_subparsers(dest='cmd', required=True)
    p = sub.add_parser('actions'); p.add_argument('--probe', required=True); p.add_argument('--out', required=True); p.set_defaults(fn=cmd_actions)
    p = sub.add_parser('compare'); p.add_argument('--orig', required=True); p.add_argument('--engine', required=True); p.add_argument('--out', required=True); p.add_argument('--md'); p.set_defaults(fn=cmd_compare)
    a = ap.parse_args()
    a.fn(a)

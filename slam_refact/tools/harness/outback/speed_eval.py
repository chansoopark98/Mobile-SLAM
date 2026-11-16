#!/usr/bin/env python3
"""Speed-consistency error of a replay run against its 1x reference run.

A speed bundle replays every k-th source frame at k times the speed, so run row i shows the
scene of reference row k*i. eSpeed is the median over run rows i >= first_row of
|p_run(i) - p_ref(k*i)|.

usage: speed_eval.py [--json FILE] [--first-row 20] <run dir>=<reference run dir>:<k> ...
"""
import argparse
import json
import math
import pathlib
import statistics as st
import sys

FIRST_ROW = 20


def _load_rows(run_dir):
    with open(pathlib.Path(run_dir) / 'trace.jsonl') as f:
        return {r['i']: r for r in (json.loads(line) for line in f if line.strip())}


def speed_error(run_dir, reference_run_dir, k, first_row=FIRST_ROW):
    """Return eSpeed, eSpeedP90 and the pair counts of one run (None values without any pair)."""
    run, ref = _load_rows(run_dir), _load_rows(reference_run_dir)
    expected = [i for i in run if i >= first_row]
    errs = []
    for i in expected:
        a, b = run[i].get('p'), (ref.get(k * i) or {}).get('p')
        if a and b:
            errs.append(math.dist(a, b))
    errs.sort()
    return {'eSpeed': st.median(errs) if errs else None,
            'eSpeedP90': errs[int(len(errs) * 0.9)] if errs else None,
            'pairs': len(errs), 'expectedPairs': len(expected),
            'complete': len(errs) == len(expected)}


def _fmt(v):
    return f'{v:8.3f}' if v is not None else '     n/a'


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    ap.add_argument('runs', nargs='+', metavar='RUN=REFERENCE:K')
    ap.add_argument('--first-row', type=int, default=FIRST_ROW)
    ap.add_argument('--json', metavar='FILE', help='write the metric dicts as a JSON list')
    args = ap.parse_args(argv)
    if args.json and pathlib.Path(args.json).exists():
        ap.error(f'{args.json} exists; refusing to overwrite')
    print(f'{"run":28s} {"k":>2s} {"eSpeed":>8s} {"p90":>8s} {"pairs":>11s}')
    results = []
    for spec in args.runs:
        run, sep, rest = spec.partition('=')
        ref, sep2, k = rest.rpartition(':')
        if not sep or not sep2 or not k.isdigit():
            ap.error(f'bad run {spec!r}: expected RUN=REFERENCE:K')
        m = speed_error(run, ref, int(k), args.first_row)
        print(f'{pathlib.Path(run).name:28s} {k:>2s} {_fmt(m["eSpeed"])} {_fmt(m["eSpeedP90"])} '
              f'{m["pairs"]:5d}/{m["expectedPairs"]:<5d}' + ('' if m['complete'] else ' [incomplete]'))
        results.append({'run': run, 'reference': ref, 'k': int(k), **m})
    if args.json:
        with open(args.json, 'x') as f:
            f.write(json.dumps(results, indent=1) + '\n')
    return 0


if __name__ == '__main__':
    sys.exit(main())

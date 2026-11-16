#!/usr/bin/env python3
"""Evaluate an out-and-back replay: the pose seen at frame j on the way back
versus the pose at frame j on the way out.

Frame j on the way out is trace row i = j; on the way back it is row
i = 2*(N-1) - j, where N is the forward frame count of the palindrome bundle.

usage: mirror_eval.py [--json FILE] [--detail] <run dir>[=<bundle dir>] ...
  run dir     directory holding trace.jsonl (and meta.json) of a replay run
  bundle dir  palindrome bundle of that run; default: meta.json 'bundle',
              resolved against the cwd, then against the run dir's ancestors
"""
import argparse
import json
import math
import pathlib
import statistics as st
import sys

try:
    from palindrome_bundle import validate_palindrome
except ImportError:  # imported as a package module
    from .palindrome_bundle import validate_palindrome

START_FRAMES = range(20, 41)  # forward frames used for E_start (21 of them)


def ang(a, b):
    """Angle in degrees between two unit quaternions (wxyz)."""
    d = abs(sum(x * y for x, y in zip(a, b)))
    return 2 * math.degrees(math.acos(min(1.0, d)))


def find_bundle(run_dir):
    """Locate the bundle directory of a run through its meta.json."""
    run_dir = pathlib.Path(run_dir)
    meta_path = run_dir / 'meta.json'
    if not meta_path.is_file():
        raise FileNotFoundError(f'{meta_path} not found; pass <run dir>=<bundle dir>')
    rel = json.loads(meta_path.read_text()).get('bundle')
    if not rel:
        raise FileNotFoundError(f"{meta_path} has no 'bundle' field; pass <run dir>=<bundle dir>")
    bases = [pathlib.Path.cwd(), *run_dir.resolve().parents]
    for base in bases:
        if (base / rel / 'bundle.json').is_file():
            return base / rel
    raise FileNotFoundError(
        f"bundle.json for meta bundle '{rel}' not found under the cwd or any "
        f'ancestor of {run_dir}; pass <run dir>=<bundle dir>')


def _load_rows(run_dir):
    with open(pathlib.Path(run_dir) / 'trace.jsonl') as f:
        return [json.loads(line) for line in f if line.strip()]


def _median(values):
    return st.median(values) if values else None


def evaluate_run(run_dir, bundle_dir):
    """Return the mirror-error metrics of one run as a dict."""
    run_dir, bundle_dir = pathlib.Path(run_dir), pathlib.Path(bundle_dir)
    validate_palindrome(bundle_dir)
    n = json.loads((bundle_dir / 'bundle.json').read_text())['palindrome']['forwardFrames']
    rows = _load_rows(run_dir)
    by = {r['i']: r for r in rows}
    last = 2 * (n - 1)
    pairs = [(j, by[j], by[last - j]) for j in range(n - 1)
             if j in by and last - j in by and by[j].get('p') and by[last - j].get('p')]
    err = {j: math.dist(a['p'], b['p']) for j, a, b in pairs}
    rot = {j: ang(a['q_wxyz'], b['q_wxyz']) for j, a, b in pairs}
    start = [err[j] for j in START_FRAMES if j in err]
    path = sum(math.dist(by[i]['p'], by[i + 1]['p'])
               for i in range(n - 1) if i in by and i + 1 in by)
    all_err = sorted(err.values())
    e_start = _median(start)

    def stage(r):
        return r.get('srStageMs') or {}

    return {
        'forwardFrames': n,
        'rows': len(rows),
        'resets': sum(1 for r in rows if r.get('srMapReset')),
        'mapLost': sum(1 for r in rows[1:] if r.get('srMapTrackerState') != 1),
        'normalRatio': (sum(1 for r in rows if r.get('status') == 'NORMAL') / len(rows)
                        if rows else None),
        'eStart': e_start,
        'eStartPairs': len(start),
        'eStartComplete': len(start) == len(START_FRAMES),
        'eAll': _median(all_err),
        'eAllP90': all_err[int(len(all_err) * .9)] if all_err else None,
        'rotStartDeg': _median([rot[j] for j in START_FRAMES if j in rot]),
        'path': path,
        'eStartOverPath': e_start / path if e_start is not None and path else None,
        'stall': sum(1 for r in rows if stage(r).get('mapTracker', 0) >= 1000
                     or stage(r).get('mapBuilder', 0) >= 200),
        'maxKeyframes': max(((r.get('srKeyframes') or 0) for r in rows), default=0),
        'resetFramesOut': [r['i'] for r in rows if r['i'] < n and r.get('srMapReset')],
        'resetFramesBackMirrored': [last - r['i'] for r in rows
                                    if n <= r['i'] <= 2 * n - 2 and r.get('srMapReset')],
    }


def detail_lines(run_dir, bundle_dir):
    """Per-segment detail lines (way out / way back, mirror error by segment)."""
    run_dir, bundle_dir = pathlib.Path(run_dir), pathlib.Path(bundle_dir)
    n = json.loads((bundle_dir / 'bundle.json').read_text())['palindrome']['forwardFrames']
    rows = {r['i']: r for r in _load_rows(run_dir)}
    last = 2 * (n - 1)
    fwd = [rows[i] for i in range(n) if i in rows]
    back = [rows[i] for i in range(n, 2 * n - 1) if i in rows]

    def summary(rs):
        return (sum(1 for r in rs if r.get('srMapReset')),
                sum(1 for r in rs if r.get('srMapTrackerState') != 1),
                max(((r.get('srKeyframes') or 0) for r in rs), default=0))

    lines = [f'{run_dir.parent.name}/{run_dir.name}: out {len(fwd)} frames '
             f'(resets, map-lost frames, max keyframes) {summary(fwd)} | '
             f'back {len(back)} frames {summary(back)}',
             f"  resets on the way out at frames {[r['i'] for r in fwd if r.get('srMapReset')]}"
             f" | on the way back at mirrored frames "
             f"{[last - r['i'] for r in back if r.get('srMapReset')]}"]
    seg = max(n // 6, 1)
    for a in range(0, n - 1, seg):
        js = [j for j in range(a, min(a + seg, n - 1))
              if j in rows and last - j in rows
              and rows[j].get('p') and rows[last - j].get('p')]
        if not js:
            continue
        e = [math.dist(rows[j]['p'], rows[last - j]['p']) for j in js]
        lost_back = sum(1 for j in js if rows[last - j].get('srMapTrackerState') != 1)
        lines.append(f'  forward frames {a:4d}-{a + seg - 1:4d}: mirror error p50 '
                     f'{st.median(e):.3f} max {max(e):.3f} | map-lost on the way back '
                     f'{lost_back}/{len(js)}')
    return lines


def _fmt(value, spec):
    return format(value, spec) if value is not None else format('n/a', spec.split('.')[0])


HEADER = (f'{"run":22s} {"fwd":>4s} {"resets":>6s} {"mtFail":>6s} {"NORMAL":>6s} '
          f'{"E_start":>8s} {"E_all":>7s} {"E_all_p90":>9s} {"rot_start":>9s} '
          f'{"E_start/path":>12s} {"stall":>5s}')


def table_row(run_dir, m):
    run_dir = pathlib.Path(run_dir)
    row = (f'{run_dir.parent.name + "/" + run_dir.name:22s} {m["forwardFrames"]:4d} '
           f'{m["resets"]:6d} {m["mapLost"]:6d} {_fmt(m["normalRatio"], "6.2f")} '
           f'{_fmt(m["eStart"], "8.3f")} {_fmt(m["eAll"], "7.3f")} '
           f'{_fmt(m["eAllP90"], "9.3f")} {_fmt(m["rotStartDeg"], "9.2f")} '
           f'{_fmt(m["eStartOverPath"], "12.4f")} {m["stall"]:5d}')
    if not m['eStartComplete']:
        row += f'  [E_start from {m["eStartPairs"]}/{len(START_FRAMES)} pairs only]'
    return row


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    ap.add_argument('runs', nargs='+', metavar='RUN[=BUNDLE]')
    ap.add_argument('--json', metavar='FILE', help='write the metric dicts as a JSON list')
    ap.add_argument('--detail', action='store_true', help='print per-segment detail lines')
    args = ap.parse_args(argv)
    print(HEADER)
    results = []
    for spec in args.runs:
        run, _, bundle = spec.partition('=')
        bundle_dir = pathlib.Path(bundle) if bundle else find_bundle(run)
        m = evaluate_run(run, bundle_dir)
        print(table_row(run, m))
        if args.detail:
            print('\n'.join(detail_lines(run, bundle_dir)))
        results.append({'run': str(run), 'bundle': str(bundle_dir), **m})
    if args.json:
        pathlib.Path(args.json).write_text(json.dumps(results, indent=1) + '\n')
    return 0


if __name__ == '__main__':
    sys.exit(main())

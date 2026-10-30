#!/usr/bin/env python3
"""US-025 LC-6a: writes the suite spec for compare_us025.py suite (every arm of every run; which runs exist is decided here, not by results).
  make_spec.py --out <new json> [--null <scenario>:<label> ...]
The six ON/OFF runs are the LC-G exact-input runs (us025-lcg-golden.md §10: original lc=1 runs that contain a loop close success; hook runs are the
instrumented wasm). Engine replay directories follow run_engine_arms.sh (engine/<run>-on, engine/<run>-off, engine/<label>-null)."""
import argparse
import json
import pathlib

GOLD = 'build/slam-refact-pkgh-20261022/us025/golden/runs'
LC6A = 'build/slam-refact-final-20261025/lc6a'
ON_OFF = [('lc1-2', 'flip6-s3-lowtex'), ('lc1-5', 'flip6-s3-lowtex'), ('lc1-n1r2', 'flip6-n1'), ('hkd-5', 'flip6-s3-lowtex'), ('hkd-7', 'flip6-s3-lowtex'),
          ('hkd-9', 'flip6-s3-lowtex')]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', required=True)
    ap.add_argument('--null', action='append', default=[], help='<scenario>:<label> of an original lc=0 run under %s/orig' % LC6A)
    a = ap.parse_args()
    runs = []
    for arm in ('on', 'off'):
        for name, scenario in ON_OFF:
            runs.append({'arm': arm, 'name': '%s-%s' % (name, arm), 'scenario': scenario, 'original': '%s/%s/%s' % (GOLD, scenario, name),
                         'engine': '%s/engine/%s-%s' % (LC6A, name, arm)})
    for sl in a.null:
        scenario, label = sl.split(':')
        runs.append({'arm': 'null', 'name': label, 'scenario': scenario, 'original': '%s/orig/%s/%s' % (LC6A, scenario, label),
                     'engine': '%s/engine/%s-null' % (LC6A, label)})
    p = pathlib.Path(a.out)
    if p.exists():
        raise SystemExit('refusing to overwrite ' + a.out)
    p.write_text(json.dumps({'schema': 'slam-refact-us025-lc6a-spec-v1', 'runs': runs}, indent=1) + '\n')
    print('wrote', a.out, len(runs), 'runs')


if __name__ == '__main__':
    main()

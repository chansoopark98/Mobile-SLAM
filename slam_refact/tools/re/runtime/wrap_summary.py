#!/usr/bin/env python3
"""Summarize the JS-boundary call log of an instrumented (--wrap) headless reference runtime run (lane C, US-004).

Input: <run>/wasm-wrap.json written by run-reference.mjs --wrap (wasm-wrap.js).  Output: <run>/wrap-summary.json
with export call counts/durations, c8EmAsm_* cadence, the per-frame call pattern, and per-processStagedFrame
import (JS host function) call counts keyed by import name and Wasm function index.
"""
import argparse
import collections
import json
import pathlib

import numpy as np


def st(xs):
    xs = np.asarray(xs, float)
    if not len(xs):
        return None
    return {'n': int(len(xs)), 'mean': float(xs.mean()), 'p50': float(np.median(xs)), 'p95': float(np.percentile(xs, 95)),
            'p99': float(np.percentile(xs, 99)), 'max': float(xs.max())}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--run', required=True, type=pathlib.Path)
    a = ap.parse_args()
    w = json.loads((a.run / 'wasm-wrap.json').read_text())
    calls = w['calls']
    by_name = collections.defaultdict(list)
    for name, start, elapsed, seq in calls:
        by_name[name].append((start, elapsed, seq))
    cadence = {}
    for name, rows in by_name.items():
        starts = [r[0] for r in rows]
        cadence[name] = {'calls': len(rows), 'durationMs': st([r[1] for r in rows]), 'totalMs': float(sum(r[1] for r in rows)),
                         'intervalMs': st(np.diff(starts)) if len(starts) > 1 else None}
    # Per-frame pattern: names between consecutive processStagedFrame calls.
    patterns = collections.Counter()
    seq_names = []
    for name, start, elapsed, seq in calls:
        if name == 'c8EmAsm_processStagedFrame' and seq_names:
            patterns[' > '.join(seq_names)] += 1
            seq_names = []
        seq_names.append(name.replace('c8EmAsm_', ''))
    target = w.get('target') or {}
    index_of = target.get('importFunctionIndex', {})
    per_frame = collections.defaultdict(list)
    frames = w.get('frameImports', [])
    for _, counts in frames:
        for key in index_of:
            per_frame[key].append(counts.get(key, 0))
    imports = []
    for key, total in sorted(w['importCounts'].items(), key=lambda kv: -kv[1]):
        if not total:
            continue
        pf = per_frame.get(key, [])
        imports.append({'import': key, 'funcIndex': index_of.get(key), 'totalCalls': total,
                        'perProcessStagedFrame': st(pf) if pf else None})
    exports = sorted(({'export': k, 'calls': v, 'timedTotalMs': w['exportMs'].get(k) if k.startswith('c8EmAsm_') else None}
                      for k, v in w['exportCounts'].items() if v), key=lambda e: -e['calls'])
    out = {'run': str(a.run), 'modules': w['modules'], 'error': w.get('error'), 'processStagedFrameCalls': w.get('frameSeq'),
           'callLogEntries': len(calls), 'c8EmAsmCadence': cadence, 'callPatternsBetweenProcessStagedFrame': patterns.most_common(12),
           'exportsCalled': exports, 'importsCalled': imports,
           'importFunctionCount': target.get('importFunctionCount'),
           'notes': ['funcIndex of an import = position among function imports = Wasm function index (imports first)',
                     'durations include the JS wrapper and any nested JS host calls; wrapper overhead is not subtracted']}
    (a.run / 'wrap-summary.json').write_text(json.dumps(out, indent=1) + '\n')
    print(json.dumps({k: out[k] for k in ('modules', 'processStagedFrameCalls', 'callLogEntries', 'callPatternsBetweenProcessStagedFrame')}, indent=1)[:3000])
    for name, c in sorted(cadence.items(), key=lambda kv: -kv[1]['totalMs']):
        print(name, c['calls'], round(c['totalMs'], 1), c['durationMs'] and round(c['durationMs']['p50'], 3), c['intervalMs'] and round(c['intervalMs']['p50'], 2))


if __name__ == '__main__':
    main()

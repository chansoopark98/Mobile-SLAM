#!/usr/bin/env python3
"""us025-lc-e2e-gate.v4 (review G-18): did the ORIGINAL run (lc=1) reach a loop close success? Read from the original run directory alone.

    python3 -B -I original_run_info.py --original <lc=1 run dir> --out <new json>

compare_us025.py's ON arm reports `originalRun` only when it could load the engine replay too, so a run whose ON replay exited non-zero lost the original's success record (report and dev
summary showed null). This tool calls the same loader (compare_us025.load_original_lc, imported as compare_full_us025.py does) on the original run data only. It never decides anything: the
gate uses it for the `originalRun` record of the report and of the dev summary, the judge does not read it. Exit 0 always (a loader error is written as {"error": ...}); 2 = usage / the output exists.
"""
import argparse
import importlib.util
import json
import pathlib
import sys

sys.dont_write_bytecode = True
REPO = pathlib.Path(__file__).resolve().parents[4]
SCHEMA = 'slam-refact-us025-lc-e2e-v4-original-run-info-v1'
COMPARE_US025 = REPO / 'slam_refact' / 'tools' / 'harness' / 'us025' / 'compare_us025.py'


def load_compare():
    spec = importlib.util.spec_from_file_location('compare_us025', COMPARE_US025)
    mod = importlib.util.module_from_spec(spec)
    sys.modules['compare_us025'] = mod
    spec.loader.exec_module(mod)
    return mod


def original_info(original, loader=None):
    """{'schema', 'source', 'originalRun': {success, attempts, ...} | None, 'error': str | None}."""
    out = {'schema': SCHEMA, 'source': 'original run data only (compare_us025.load_original_lc)', 'original': str(original), 'originalRun': None, 'error': None}
    try:
        mod = load_compare() if loader is None else None
        load = loader if loader is not None else mod.load_original_lc
        orig = load(original, 1)
        out['originalRun'] = {'success': orig['firstSuccess'] is not None, 'attempts': len(orig['attempts']), 'hookMode': orig['hooked'], 'restaged': orig.get('restaged'),
                              'loadAvgAtStart': orig['meta'].get('loadAvgAtStart'), 'status': orig['meta'].get('status'), 'firstSuccessFrame': (orig['firstSuccess'] or {}).get('frame')}
    except Exception as error:   # noqa: BLE001 - an unreadable original is recorded, never raised: the record stays null
        out['error'] = f'{type(error).__name__}: {error}'
    return out


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--original', required=True)
    ap.add_argument('--out', required=True)
    a = ap.parse_args(argv)
    if pathlib.Path(a.out).exists():
        print(f'{a.out} exists: never overwritten', file=sys.stderr)
        return 2
    res = original_info(a.original)
    with open(a.out, 'x', encoding='utf8') as f:
        json.dump(res, f, indent=1, sort_keys=True, default=str)
        f.write('\n')
    print(json.dumps({'success': (res['originalRun'] or {}).get('success'), 'error': res['error']}))
    return 0


if __name__ == '__main__':
    sys.exit(main())

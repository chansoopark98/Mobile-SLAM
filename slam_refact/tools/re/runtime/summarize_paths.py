#!/usr/bin/env python3
"""Summarize the unreachable-path memprobe fields (run-reference.mjs --memprobe --memprobe-paths).

  python3 -I slam_refact/tools/re/runtime/summarize_paths.py --run <scenario>=<run dir> [...] --out-json <file>

Per run: records read, records with a valid VioTracker root, and per field the value histogram over valid records.
Checks (fixed before the full runs, docs/reports/re-unreachable-paths.md):
  path A      mtRelocPending == 0 and outRelocKf == 0 on every valid record
  1000-point  vtIncreasePts == 0 on every valid record; frmPoints <= 500
  gates       vpsMode == 0, mbCfg[6] (relocToKfWithDepthPoints) == 0, holderVps == [0, 0, 0]
  controls    engineOk == 1; mbCfg == func651 defaults [1,1,1,0,1,1,0,1,0]; vtCfgPts == [1000, 500];
              mtKfCreated == outKeyframeCreated on every valid record (664 writes MapTracker+0x12b0 from out[1])
"""
import argparse
import collections
import gzip
import json

MB_CFG_DEFAULTS = [1, 1, 1, 0, 1, 1, 0, 1, 0]


def summarize(path):
    lines = gzip.open(path, 'rt').read().splitlines()
    header = json.loads(lines[0])
    recs = [json.loads(line) for line in lines[1:]]
    valid = [r for r in recs if r.get('root') and r.get('vpsMode') is not None]
    hist = {}
    for key in ('mtRelocPending', 'outRelocKf', 'vtIncreasePts', 'vpsMode', 'engineOk', 'holderVps', 'mbCfg',
                'vtCfgPts', 'mtKfCreated', 'mtRelocKf', 'outRelocKfId'):
        c = collections.Counter(json.dumps(r.get(key)) for r in valid)
        hist[key] = dict(sorted(c.items()))
    pts = [r['frmPoints'] for r in valid if r.get('frmPoints') is not None]
    checks = {
        'pathA_mtRelocPending_all_zero': all(r['mtRelocPending'] == 0 for r in valid),
        'pathA_outRelocKf_all_zero': all(r['outRelocKf'] == 0 for r in valid),
        'increasePts_all_zero': all(r['vtIncreasePts'] == 0 for r in valid),
        'frmPoints_max_le_500': bool(pts) and max(pts) <= 500,
        'gate_vpsMode_all_zero': all(r['vpsMode'] == 0 for r in valid),
        'gate_relocToKfWithDepthPoints_all_zero': all(r['mbCfg'][6] == 0 for r in valid),
        'gate_holderVps_all_zero': all(r['holderVps'] == [0, 0, 0] for r in valid),
        'control_engineOk': all(r['engineOk'] == 1 for r in valid),
        'control_mbCfg_defaults': all(r['mbCfg'] == MB_CFG_DEFAULTS for r in valid),
        'control_vtCfgPts': all(r['vtCfgPts'] == [1000, 500] for r in valid),
        'control_mtKfCreated_eq_outKeyframeCreated': all(r['mtKfCreated'] == r['outKeyframeCreated'] for r in valid),
    }
    return {
        'file': path, 'schema': header.get('schema'), 'wasmSha256': header.get('wasmSha256'),
        'probeError': header.get('error'), 'fields': header.get('fields'),
        'records': len(recs), 'validRecords': len(valid),
        'associatedFrames': sum(1 for r in valid if r.get('i') is not None),
        'keyframeRecords': sum(1 for r in valid if r.get('outKeyframeCreated')),
        'mapResetRecords': sum(1 for r in valid if r.get('outMapReset')),
        'frmPoints': {'n': len(pts), 'min': min(pts) if pts else None, 'max': max(pts) if pts else None,
                      'hist': dict(sorted(collections.Counter(pts).items())) if pts else {}},
        'hist': hist, 'checks': checks, 'allChecksPass': all(checks.values()),
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--run', action='append', required=True, help='<scenario>=<run dir>')
    ap.add_argument('--out-json', required=True)
    args = ap.parse_args()
    out = {'schema': 'slam-refact-re-paths-summary-v1', 'runs': {}}
    for spec in args.run:
        name, run_dir = spec.split('=', 1)
        out['runs'][name] = summarize(run_dir.rstrip('/') + '/memprobe.jsonl.gz')
    out['allChecksPass'] = all(r['allChecksPass'] for r in out['runs'].values())
    with open(args.out_json, 'w') as f:
        json.dump(out, f, indent=1)
        f.write('\n')
    for name, r in out['runs'].items():
        failed = [k for k, v in r['checks'].items() if not v]
        print(name, r['records'], r['validRecords'], 'PASS' if r['allChecksPass'] else 'FAIL ' + ','.join(failed))


if __name__ == '__main__':
    main()

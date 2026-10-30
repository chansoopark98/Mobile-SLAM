#!/usr/bin/env python3
"""Dev-scenario evidence of us026-ba-e2e-gate.v2 (protocol sections 3 and 4 again for the new version). DEV scenario flip6-n1 only,
OFFLINE: no original SDK, no Chromium, no marker, no registration data. It re-plays the three exact-input BUNDLES that the final v1 dev
end-to-end run (gate-dev/e2e3) already built from its two ORIGINAL runs with the pinned v2 replay binary, and re-applies the unchanged
comparator (compare_us026.py ba-scenario) against the SAME original runs. The result is a gate_e2e-shaped dev output tree
(<out-root>/us026-ba-e2e-gate/flip6-n1/{report.json,spec.json,compare.json,replays/}) that make_e2e_static.py and mutants_e2e.py read.

  python3 -I refresh_dev_evidence.py --src <gate-dev/e2e3/us026-ba-e2e-gate/flip6-n1> --binary <v2 replay> --out-root <new dir> [--jobs 2]

Nothing under --src is modified; --out-root must not exist.
"""
import argparse
import concurrent.futures as cf
import hashlib
import json
import pathlib
import subprocess
import sys
import time

ROOT = pathlib.Path(__file__).resolve().parents[4]
COMPARE = ROOT / 'slam_refact/tools/harness/us026/compare_us026.py'
GATE_ID = 'us026-ba-e2e-gate'
DEV = 'flip6-n1'


def sha(p):
    return hashlib.sha256(pathlib.Path(p).read_bytes()).hexdigest()


def rel(p):
    return str(pathlib.Path(p).resolve().relative_to(ROOT))


def replay(binary, bundle, out):
    t0 = time.time()
    with open(str(out) + '.log', 'x') as log:
        rc = subprocess.call([str(binary), '--bundle', str(bundle), '--out', str(out)], stdout=log, stderr=subprocess.STDOUT, cwd=ROOT)
    return rc, round(time.time() - t0, 1)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--src', type=pathlib.Path, required=True)
    ap.add_argument('--binary', type=pathlib.Path, required=True)
    ap.add_argument('--out-root', type=pathlib.Path, required=True)
    ap.add_argument('--jobs', type=int, default=2)
    a = ap.parse_args()
    if not 1 <= a.jobs <= 4:
        raise SystemExit('--jobs 1..4')
    rep = json.loads((a.src / 'report.json').read_text())
    spec = json.loads((a.src / 'spec.json').read_text())
    if rep.get('kind') != 'dev' or rep.get('scenario') != DEV or rep['gate']['id'] != GATE_ID:
        raise SystemExit('refused: %s is not a dev report of %s %s' % (a.src, GATE_ID, DEV))
    if a.out_root.exists():
        raise SystemExit('refusing to reuse ' + str(a.out_root))
    out = a.out_root / GATE_ID / DEV
    (out / 'replays').mkdir(parents=True)
    names = ['on', 'off', 'null']
    res = {}
    with cf.ThreadPoolExecutor(a.jobs) as ex:
        futs = {ex.submit(replay, a.binary.resolve(), ROOT / rep['bundles'][n]['dir'], (out / 'replays' / n).resolve()): n for n in names}
        for f in cf.as_completed(futs):
            res[futs[f]] = f.result()
    spec2 = json.loads(json.dumps(spec))
    rep2 = json.loads(json.dumps(rep))
    for n in names:
        entry = dict(spec['replays'][n], dir=rel(out / 'replays' / n), exit=res[n][0], seconds=res[n][1])
        spec2['replays'][n] = entry
        rep2['replays'][n] = dict(rep['replays'][n], dir=rel(out / 'replays' / n), exit=res[n][0], seconds=res[n][1])
    (out / 'spec.json').write_text(json.dumps(spec2, indent=1) + '\n')
    r = subprocess.run([sys.executable, '-I', str(COMPARE), 'ba-scenario', '--spec', str(out / 'spec.json'), '--out', str(out / 'compare.json')],
                       cwd=ROOT, capture_output=True, text=True)
    if r.returncode != 0:
        raise SystemExit('compare_us026.py failed: ' + r.stderr[-2000:])
    comp = json.loads((out / 'compare.json').read_text())
    rep2['result'] = {'class': comp['class'], 'verdict': comp.get('verdict'), 'classReason': comp.get('classReason')}
    rep2['refreshOf'] = {'sourceReport': rel(a.src / 'report.json'), 'sourceReportSha256': sha(a.src / 'report.json'),
                         'what': 'v2 dev refresh: the bundles and original runs of the source report, replays re-run with the v2 binary, comparator unchanged',
                         'binary': {'path': rel(a.binary), 'sha256': sha(a.binary)}, 'comparator': {'path': rel(COMPARE), 'sha256': sha(COMPARE)}}
    (out / 'report.json').write_text(json.dumps(rep2, indent=1) + '\n')
    print(json.dumps({'out': rel(out), 'replayExits': {n: res[n][0] for n in names}, 'replaySeconds': {n: res[n][1] for n in names},
                      'class': comp['class'], 'verdict': comp.get('verdict'), 'classReason': comp.get('classReason'),
                      'nullBitIdentical': comp.get('compare', {}).get('null', {}).get('bitIdentical'),
                      'tiers': comp.get('tiers'), 'E-BA-1': comp.get('E-BA-1')}))
    return 0


if __name__ == '__main__':
    sys.exit(main())

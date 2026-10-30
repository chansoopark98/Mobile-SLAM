#!/usr/bin/env python3
"""REPORT-only sanity check of the pinned v2 binary against a v1 gate run (offline, no SDK, no Chromium, no marker, nothing under --src is modified):
re-play ONE exact-input bundle of a v1 gate scenario output with the v2 binary and re-apply the unchanged comparator against that scenario's
ORIGINAL runs. It shows whether the pinned artifact (not only the source tree) contains the engine fix: v1 FAIL scenarios flip6-s2 and tum-room2-full
should now compare clean on the `on` arm. Not a gate run: the v1 results stay the v1 results.

  python3 -I sanity_replay_v1_original.py --src <gate-runs/us026-ba-e2e-gate/<scenario>> --bundle on --binary <v2 replay> --out <new dir>
"""
import argparse
import json
import pathlib
import subprocess
import sys
import time

ROOT = pathlib.Path(__file__).resolve().parents[4]
COMPARE = ROOT / 'slam_refact/tools/harness/us026/compare_us026.py'


def rel(p):
    return str(pathlib.Path(p).resolve().relative_to(ROOT))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--src', type=pathlib.Path, required=True)
    ap.add_argument('--bundle', choices=('on', 'off', 'null'), required=True)
    ap.add_argument('--binary', type=pathlib.Path, required=True)
    ap.add_argument('--out', type=pathlib.Path, required=True)
    a = ap.parse_args()
    if a.out.exists():
        raise SystemExit('refusing to reuse ' + str(a.out))
    rep = json.loads((a.src / 'report.json').read_text())
    spec = json.loads((a.src / 'spec.json').read_text())
    if rep.get('kind') != 'gate' or rep['gate']['id'] != 'us026-ba-e2e-gate' or rep['gate']['version'] != 1:
        raise SystemExit('refused: not a us026-ba-e2e-gate v1 gate report')
    a.out.mkdir(parents=True)
    t0 = time.time()
    with open(a.out / 'replay.log', 'x') as log:
        rc = subprocess.call([str(a.binary.resolve()), '--bundle', str(ROOT / rep['bundles'][a.bundle]['dir']), '--out', str((a.out / 'replay').resolve())],
                             stdout=log, stderr=subprocess.STDOUT, cwd=ROOT)
    sec = round(time.time() - t0, 1)
    spec['replays'][a.bundle] = dict(spec['replays'][a.bundle], dir=rel(a.out / 'replay'), exit=rc, seconds=sec)
    (a.out / 'spec.json').write_text(json.dumps(spec, indent=1) + '\n')
    r = subprocess.run([sys.executable, '-I', str(COMPARE), 'ba-scenario', '--spec', str(a.out / 'spec.json'), '--out', str(a.out / 'compare.json')],
                       cwd=ROOT, capture_output=True, text=True)
    if r.returncode != 0:
        raise SystemExit('compare failed: ' + r.stderr[-1500:])
    c = json.loads((a.out / 'compare.json').read_text())
    arm = c['compare'].get(a.bundle, {})
    print(json.dumps({'scenario': rep['scenario'], 'bundle': a.bundle, 'replayExit': rc, 'replaySeconds': sec, 'v1Result': rep['result'],
                      'framesCompared': arm.get('framesCompared'), 'bitIdentical': arm.get('bitIdentical'), 'poseMaxUlps': (arm.get('S') or {}).get('poseMaxUlps'),
                      'poseViolationFrames': (arm.get('S') or {}).get('poseViolationFrames'),
                      'integerMismatchFrames': {k: v for k, v in ((arm.get('D') or {}).get('mismatchFrames') or {}).items() if v},
                      'note': 'REPORT only; the comparator class of this mixed spec (v1 off/null replays, v2 on replay) is not a gate result'}))
    return 0


if __name__ == '__main__':
    sys.exit(main())

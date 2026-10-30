#!/usr/bin/env python3
"""Sequential full-length pyramid captures in stage windows (capture_window.mjs holds every readback in page memory,
~2.7 MB each, so a 2821-frame sequence is split into windows of --window captures with --overlap stages of overlap).
Same load gate as runtime/run_pmr_series.py (1-min loadavg < 12, no other Playwright browser, poll 15 s, give up 60 min).
Labels <prefix>-wNN are never overwritten (an existing dir is skipped).

  python3 -I slam_refact/tools/harness/divergence/capture_series.py --scenario tum-room1-full --frames 2821 \
      --prefix pyr --log build/slam-refact-20261016/exact-input/capture-series.log
"""
import argparse
import datetime
import pathlib
import subprocess
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[2] / 're/runtime'))
import run_pmr_series as gate  # noqa: E402

ROOT = gate.ROOT
CAP = ROOT / 'build/slam-refact-20261016/divergence/capture'


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--scenario', required=True)
    ap.add_argument('--frames', type=int, required=True, help='input frames of the scenario')
    ap.add_argument('--prefix', default='pyr')
    ap.add_argument('--window', type=int, default=300)
    ap.add_argument('--overlap', type=int, default=20)
    ap.add_argument('--starts', type=int, nargs='*', help='explicit from-stage values (fill passes)')
    ap.add_argument('--log', type=pathlib.Path, required=True)
    a = ap.parse_args()
    starts = a.starts or list(range(1, a.frames + 1, a.window - a.overlap))
    for n, st in enumerate(starts, 1):
        label = '%s-w%02d' % (a.prefix, n)
        if (CAP / a.scenario / label).exists():
            gate.log(a.log, f'skip  {a.scenario}/{label} exists')
            continue
        w = gate.wait_low_load(a.log, f'{a.scenario}/{label}')
        if w.get('blocked'):
            gate.log(a.log, f'BLOCKED {a.scenario}/{label} {gate.now()} load={w["loadAtGiveUp"]}')
            continue
        maxf = min(a.frames, st + a.window + 40)
        cmd = ['node', 'slam_refact/tools/re/spec_frontend/capture_window.mjs', '--scenario', a.scenario, '--label', label,
               '--max-frames', str(maxf), '--from-stage', str(st), '--max-captures', str(a.window), '--rows', '660']
        gate.log(a.log, f'start {a.scenario}/{label} {gate.now()} load={gate.load()} waited={w["waitedS"]}s cmd={" ".join(cmd)}')
        with open(a.log.with_suffix('.%s-%s.out' % (a.scenario, label)), 'w') as f:
            rc = subprocess.run(cmd, cwd=ROOT, stdout=f, stderr=subprocess.STDOUT).returncode
        gate.log(a.log, f'end   {a.scenario}/{label} {gate.now()} exit={rc} load={gate.load()}')
    gate.log(a.log, 'DONE %s %s' % (a.scenario, datetime.datetime.now().isoformat(timespec='seconds')))


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""Pinned OFF/ON timing runs for gate G5.

usage: run_timing.py --replay BIN --prereg P --e0-bundles-root SOURCE_BUNDLES --accuracy "<ON env>"
                     --out TIMING_ROOT --cpus 2,4,6,8,10,12 [--dry-run]

Every sequence of prereg sequences.timing gets one worker thread pinned to its own CPU. A worker runs
1-off, 1-on, 2-off, 2-on, 3-off, 3-on in this order (taskset -c CPU BIN --bundle B --out
TIMING_ROOT/<sequence>/<rep>-<arm>); SLAM_REFACT_ACCURACY is removed for off and set to --accuracy
for on. One line per run is appended to TIMING_ROOT/status.jsonl. An existing run directory is not
overwritten: it is reported and counted as a failure. Exit status 1 if any run failed.
"""
import argparse
import hashlib
import json
import os
import pathlib
import subprocess
import sys
import threading
import time

REPO = pathlib.Path(__file__).resolve().parents[4]
ARMS = ('off', 'on')


def repo_path(p):
    p = pathlib.Path(p)
    return p if p.is_absolute() else REPO / p


def bundle_of(prereg, name, source_root):
    """Bundle of a timing sequence: gate bundles for ob-/fw-, else the source bundle of the fixture."""
    if name.startswith(('ob-', 'fw-')):
        return repo_path(prereg['bundles']) / name
    return pathlib.Path(source_root) / prereg['bundleOf'][name]


def plan(prereg, source_root, out_root, cpus):
    """[(sequence, cpu, [(rep, arm, cmd, run_dir)...])...]; raises ValueError when CPUs are too few."""
    names = list(prereg['sequences']['timing'])
    if len(cpus) < len(names):
        raise ValueError(f'{len(names)} sequences need {len(names)} CPUs, --cpus has {len(cpus)}')
    reps = prereg['limits']['timingRepetitions']
    out = []
    for name, cpu in zip(names, cpus):
        bundle = bundle_of(prereg, name, source_root)
        runs = []
        for rep in range(1, reps + 1):
            for arm in ARMS:
                run_dir = out_root / name / f'{rep}-{arm}'
                cmd = ['taskset', '-c', str(cpu), '--', 'BIN', '--bundle', str(bundle), '--out', str(run_dir)]
                runs.append((rep, arm, cmd, run_dir))
        out.append((name, cpu, runs))
    return out


def sha256_file(path):
    h = hashlib.sha256()
    with open(path, 'rb') as f:
        for chunk in iter(lambda: f.read(1 << 20), b''):
            h.update(chunk)
    return h.hexdigest()


def worker(replay, replay_sha, accuracy, out_root, name, cpu, runs, lock, failures):
    for rep, arm, cmd, run_dir in runs:
        cmd = [str(replay) if c == 'BIN' else c for c in cmd]
        record = {'sequence': name, 'rep': rep, 'arm': arm, 'cpu': cpu, 'replaySha256': replay_sha,
                  'accuracy': accuracy if arm == 'on' else ''}
        if run_dir.exists():
            record.update({'exit': None, 'error': 'run directory exists; not overwritten'})
            failures.append(record)
        else:
            run_dir.parent.mkdir(parents=True, exist_ok=True)
            env = dict(os.environ)
            env.pop('SLAM_REFACT_ACCURACY', None)
            if arm == 'on':
                env['SLAM_REFACT_ACCURACY'] = accuracy
            record['load1Start'] = os.getloadavg()[0]
            start = time.time()
            stem = run_dir.parent / f'{rep}-{arm}'
            with open(f'{stem}.stdout.log', 'wb') as so, open(f'{stem}.stderr.log', 'wb') as se:
                proc = subprocess.run(cmd, env=env, stdout=so, stderr=se)
            record.update({'exit': proc.returncode, 'wallSeconds': round(time.time() - start, 1),
                           'load1End': os.getloadavg()[0]})
            if proc.returncode != 0:
                failures.append(record)
        with lock:
            with open(out_root / 'status.jsonl', 'a') as f:
                f.write(json.dumps(record) + '\n')
            print(json.dumps(record), flush=True)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    ap.add_argument('--replay', required=True)
    ap.add_argument('--prereg', required=True)
    ap.add_argument('--e0-bundles-root', required=True)
    ap.add_argument('--accuracy', required=True)
    ap.add_argument('--out', required=True)
    ap.add_argument('--cpus', required=True, help='comma-separated CPU numbers, one per sequence')
    ap.add_argument('--dry-run', action='store_true', help='print the planned commands only')
    args = ap.parse_args(argv)
    prereg = json.loads(pathlib.Path(args.prereg).read_text())
    cpus = [int(c) for c in args.cpus.split(',') if c != '']
    if len(set(cpus)) != len(cpus):
        ap.error('--cpus has duplicates')
    out_root = pathlib.Path(args.out)
    try:
        sequences = plan(prereg, args.e0_bundles_root, out_root, cpus)
    except ValueError as e:
        ap.error(str(e))
    if args.dry_run:
        for name, cpu, runs in sequences:
            for rep, arm, cmd, _ in runs:
                env = f'SLAM_REFACT_ACCURACY={args.accuracy!r} ' if arm == 'on' else '(no SLAM_REFACT_ACCURACY) '
                print(f'{name} cpu{cpu} {rep}-{arm}: {env}' + ' '.join(str(args.replay) if c == 'BIN' else c for c in cmd))
        return 0
    out_root.mkdir(parents=True, exist_ok=True)
    replay_sha = sha256_file(args.replay)
    lock, failures = threading.Lock(), []
    threads = [threading.Thread(target=worker, args=(args.replay, replay_sha, args.accuracy, out_root, n, c, r, lock, failures))
               for n, c, r in sequences]
    for t in threads:
        t.start()
    for t in threads:
        t.join()
    print(f'{len(failures)} failed runs')
    return 1 if failures else 0


if __name__ == '__main__':
    sys.exit(main())

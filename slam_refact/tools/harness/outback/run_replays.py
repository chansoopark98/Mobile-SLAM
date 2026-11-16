#!/usr/bin/env python3
"""Run several native replays with a fixed number of workers and record every exit status.

usage: run_replays.py --replay <slam_refact_replay> --out <root> [--jobs 4] [--accuracy "k=v,..."]
                      [--cpu-list 0,1,...] <name>=<bundle dir> ...

Each run writes <root>/<name>/ (trace.jsonl, meta.json, ...), <root>/<name>.stdout.log,
<root>/<name>.stderr.log and one line of <root>/status.jsonl. SLAM_REFACT_ACCURACY is set to
--accuracy for every run, or removed from the environment when --accuracy is not given (parity
engine). A run whose output directory already exists is skipped and reported as "exists": the
replay tool refuses to overwrite, and a finished run is never repeated silently.
A failing run does not stop the others (a plain `xargs -P` drops its queue when a child dies).
"""
import argparse
import concurrent.futures
import hashlib
import json
import os
import pathlib
import subprocess
import sys
import time


def sha256_file(path):
    h = hashlib.sha256()
    with open(path, 'rb') as f:
        for chunk in iter(lambda: f.read(1 << 20), b''):
            h.update(chunk)
    return h.hexdigest()


def run_one(replay, replay_sha, out_root, name, bundle, accuracy, cpu):
    out_dir = out_root / name
    record = {'name': name, 'bundle': str(bundle), 'accuracy': accuracy, 'replaySha256': replay_sha}
    if out_dir.exists():
        record['status'] = 'exists'
        return record
    env = dict(os.environ)
    env.pop('SLAM_REFACT_ACCURACY', None)
    if accuracy:
        env['SLAM_REFACT_ACCURACY'] = accuracy
    cmd = [str(replay), '--bundle', str(bundle), '--out', str(out_dir)]
    if cpu is not None:
        cmd = ['taskset', '-c', str(cpu)] + cmd
    start = time.time()
    with open(out_root / f'{name}.stdout.log', 'wb') as so, open(out_root / f'{name}.stderr.log', 'wb') as se:
        proc = subprocess.run(cmd, env=env, stdout=so, stderr=se)
    record.update({'status': 'done' if proc.returncode == 0 else 'failed', 'exit': proc.returncode,
                   'wallSeconds': round(time.time() - start, 1), 'load1': os.getloadavg()[0]})
    return record


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--replay', required=True)
    ap.add_argument('--out', required=True)
    ap.add_argument('--jobs', type=int, default=4)
    ap.add_argument('--accuracy', default='')
    ap.add_argument('--cpu-list', default='', help='pin run i to cpu-list[i %% len] with taskset')
    ap.add_argument('runs', nargs='+', metavar='NAME=BUNDLE')
    args = ap.parse_args(argv)
    if not 1 <= args.jobs <= 8:
        ap.error('--jobs must be 1..8')
    replay = pathlib.Path(args.replay)
    replay_sha = sha256_file(replay)
    out_root = pathlib.Path(args.out)
    out_root.mkdir(parents=True, exist_ok=True)
    cpus = [int(c) for c in args.cpu_list.split(',') if c != '']
    jobs = []
    for i, item in enumerate(args.runs):
        name, sep, bundle = item.partition('=')
        if not sep or not (pathlib.Path(bundle) / 'bundle.json').exists():
            ap.error(f'bad run {item!r}: expected NAME=<bundle dir with bundle.json>')
        jobs.append((name, pathlib.Path(bundle), cpus[i % len(cpus)] if cpus else None))
    failed = 0
    with concurrent.futures.ThreadPoolExecutor(max_workers=args.jobs) as pool:
        futures = [pool.submit(run_one, replay, replay_sha, out_root, n, b, args.accuracy, c) for n, b, c in jobs]
        for fut in concurrent.futures.as_completed(futures):
            rec = fut.result()
            failed += rec['status'] == 'failed'
            with open(out_root / 'status.jsonl', 'a') as f:
                f.write(json.dumps(rec) + '\n')
            print(json.dumps(rec), flush=True)
    return 1 if failed else 0


if __name__ == '__main__':
    sys.exit(main())

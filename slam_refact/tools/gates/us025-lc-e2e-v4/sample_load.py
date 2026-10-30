#!/usr/bin/env python3
"""External-disturbance recorder of the us025-lc-e2e-gate v3 parallel equivalence experiment (preregistration section 7.2; prereview C-2, decision D-V3-8).

    python3 -I -B sample_load.py --out <sampler.jsonl> [--interval 10] [--top 20] [--max-seconds N] [--proc /proc]

Every INTERVAL seconds one JSON line is appended to --out:
    {"t": "<UTC ISO, ms>", "dt": <seconds since the previous sample or null>, "baseline": <true for the first sample>, "loadavg": "<contents of /proc/loadavg>",
     "sys_busy_cores": <busy CPUs over the interval, from /proc/stat, or null>, "gate_pids": [<pid of the gate process tree>],
     "others": [{"pid", "ppid", "comm", "cores", "cpu_s", "args"}, ...]}
`others` are the processes that are NOT in the process tree of `gate_lc.mjs dev` and not this recorder, taken AFTER that exclusion and then cut to the TOP busiest of the interval. `cores` is the CPU time
the process used between the two samples divided by the real time between them (utime + stime from /proc/<pid>/stat: the per-interval delta, not the lifetime average of `ps` %CPU), `cpu_s` its
cumulative CPU seconds (for the audit). The first sample is a baseline (`others` empty): a delta needs two samples. The recorder starts BEFORE the experiment (a process tree appears later; until then
`gate_pids` is empty) and ends at SIGTERM / SIGINT / --max-seconds with a last line {"t": ..., "end": true}. A line {"t": ..., "error": "..."} stands for a sample that could not be taken.
analyze_equivalence.py reads these lines (G9); a gap between two samples longer than its GAP_S makes the runs that overlap it "unobserved".
Standard library only."""
import argparse
import datetime
import json
import os
import signal
import sys
import time

sys.dont_write_bytecode = True

TICKS = os.sysconf('SC_CLK_TCK')
ARGS_MAX = 160


def iso_now():
    return datetime.datetime.now(datetime.timezone.utc).strftime('%Y-%m-%dT%H:%M:%S.%f')[:-3] + 'Z'


def read_stat(path):
    """(ppid, comm, cpu_ticks, starttime) from one /proc/<pid>/stat. comm may contain spaces and parentheses: split at the LAST ')'."""
    with open(path, encoding='utf8', errors='replace') as f:
        text = f.read()
    head, _, rest = text.rpartition(')')
    comm = head.partition('(')[2]
    fields = rest.split()                      # fields[0] = state (field 3), fields[1] = ppid (4), fields[11] = utime (14), fields[12] = stime (15), fields[19] = starttime (22)
    return int(fields[1]), comm, int(fields[11]) + int(fields[12]), int(fields[19])


def read_procs(proc='/proc'):
    """{pid: {ppid, comm, ticks, start, args}} of every process that can be read (a process that vanishes during the scan is skipped)."""
    procs = {}
    for name in os.listdir(proc):
        if not name.isdigit():
            continue
        pid = int(name)
        try:
            ppid, comm, ticks, start = read_stat(os.path.join(proc, name, 'stat'))
            with open(os.path.join(proc, name, 'cmdline'), 'rb') as f:
                args = f.read().replace(b'\0', b' ').decode('utf8', 'replace').strip()
        except (OSError, ValueError, IndexError):
            continue
        procs[pid] = {'ppid': ppid, 'comm': comm, 'ticks': ticks, 'start': start, 'args': args or f'[{comm}]'}
    return procs


def gate_tree(procs, own_pid):
    """pids of every `gate_lc.mjs dev` process and all their descendants. The recorder itself is never part of it."""
    roots = {pid for pid, p in procs.items() if pid != own_pid and 'gate_lc.mjs' in p['args'] and ' dev' in p['args']}
    children = {}
    for pid, p in procs.items():
        children.setdefault(p['ppid'], []).append(pid)
    tree, todo = set(), list(roots)
    while todo:
        pid = todo.pop()
        if pid in tree:
            continue
        tree.add(pid)
        todo.extend(children.get(pid, []))
    tree.discard(own_pid)
    return tree


def read_busy_total(proc='/proc'):
    """(busy jiffies, total jiffies) of all CPUs from the first line of /proc/stat (busy = everything but idle and iowait)."""
    with open(os.path.join(proc, 'stat'), encoding='utf8') as f:
        v = [int(x) for x in f.readline().split()[1:9]]
    return v[0] + v[1] + v[2] + v[5] + v[6] + v[7], sum(v)


class Sampler:
    def __init__(self, proc='/proc', top=20, clock=time.monotonic, own_pid=None):
        self.proc, self.top, self.clock, self.own_pid = proc, top, clock, own_pid if own_pid is not None else os.getpid()
        self.prev = None          # (monotonic time, {(pid, start): ticks}, busy, total)

    def sample(self):
        now = self.clock()
        procs = read_procs(self.proc)
        busy, total = read_busy_total(self.proc)
        tree = gate_tree(procs, self.own_pid)
        with open(os.path.join(self.proc, 'loadavg'), encoding='utf8') as f:
            loadavg = f.read().strip()
        ticks = {(pid, p['start']): p['ticks'] for pid, p in procs.items()}
        out = {'t': iso_now(), 'dt': None, 'baseline': self.prev is None, 'loadavg': loadavg, 'sys_busy_cores': None, 'gate_pids': sorted(tree), 'others': []}
        if self.prev is not None:
            t0, ticks0, busy0, total0 = self.prev
            dt = max(now - t0, 1e-6)
            out['dt'] = round(dt, 3)
            if total > total0:
                out['sys_busy_cores'] = round((busy - busy0) / (total - total0) * (os.cpu_count() or 1), 3)
            rows = []
            for pid, p in procs.items():
                if pid in tree or pid == self.own_pid:
                    continue                                                  # exclusion BEFORE the cut
                used = (p['ticks'] - ticks0.get((pid, p['start']), 0)) / TICKS
                if used > 0:
                    rows.append({'pid': pid, 'ppid': p['ppid'], 'comm': p['comm'], 'cores': round(used / dt, 3), 'cpu_s': round(p['ticks'] / TICKS, 2), 'args': p['args'][:ARGS_MAX]})
            rows.sort(key=lambda r: (-r['cores'], r['pid']))
            out['others'] = rows[:self.top]
        self.prev = (now, ticks, busy, total)
        return out


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--out', required=True)
    ap.add_argument('--interval', type=float, default=10.0)
    ap.add_argument('--top', type=int, default=20)
    ap.add_argument('--max-seconds', type=float, default=None)
    ap.add_argument('--proc', default='/proc')
    a = ap.parse_args(argv)
    if a.interval <= 0 or a.top < 1:
        print('--interval > 0 and --top >= 1', file=sys.stderr)
        return 2
    stop = []
    signal.signal(signal.SIGTERM, lambda *_: stop.append(1))
    signal.signal(signal.SIGINT, lambda *_: stop.append(1))
    sampler = Sampler(a.proc, a.top)
    started = time.monotonic()
    nxt = started
    with open(a.out, 'a', encoding='utf8') as f:
        while not stop:
            try:
                line = sampler.sample()
            except Exception as error:   # noqa: BLE001 - a failed sample is recorded, the recorder goes on
                line = {'t': iso_now(), 'error': f'{type(error).__name__}: {error}'}
            f.write(json.dumps(line, separators=(',', ':')) + '\n')
            f.flush()
            nxt += a.interval
            if a.max_seconds is not None and nxt - started > a.max_seconds:
                break
            while not stop and time.monotonic() < nxt:
                time.sleep(min(0.2, max(0.0, nxt - time.monotonic())))
        f.write(json.dumps({'t': iso_now(), 'end': True}) + '\n')
    return 0


if __name__ == '__main__':
    sys.exit(main())

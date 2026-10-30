#!/usr/bin/env python3
"""US-025 LC-6b: kills the RENDERER of a hung original run (lc-g §9: after a loop-close success the page can spin at ~98% CPU forever and
page.evaluate never returns). Only renderer processes that are descendants of a `run_lcgold_scene.mjs` node process (this lane's runner) and have
burned > 90% CPU for longer than --max-run-s are signalled. The runner then records status FAILED ('Target crashed'), exactly as in lc-g.
usage: watchdog.py --max-run-s 330 --until-file <path containing the line given by --until-text>"""
import argparse, os, signal, subprocess, time


def procs():
    out = subprocess.run(['ps', '-eo', 'pid=,ppid=,etimes=,pcpu=,args='], capture_output=True, text=True).stdout.splitlines()
    d = {}
    for l in out:
        p = l.split(None, 4)
        if len(p) == 5:
            d[int(p[0])] = (int(p[1]), int(p[2]), float(p[3]), p[4])
    return d


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--max-run-s', type=int, default=330)
    ap.add_argument('--until-file', required=True)
    ap.add_argument('--until-text', required=True)
    a = ap.parse_args()
    while True:
        try:
            if a.until_text in open(a.until_file).read():
                return
        except OSError:
            pass
        P = procs()
        mine = {pid for pid, v in P.items() if 'run_lcgold_scene.mjs' in v[3] and v[3].startswith('node ')}
        for pid, (ppid, et, cpu, args) in P.items():
            if '--type=renderer' not in args or et < a.max_run_s or cpu < 90:
                continue
            q, hop = ppid, 0
            while q in P and hop < 12:
                if q in mine:
                    print(time.strftime('%F %T'), 'kill renderer', pid, 'etime', et, 'cpu', cpu, flush=True)
                    os.kill(pid, signal.SIGKILL)
                    break
                q = P[q][0]; hop += 1
        time.sleep(10)


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""US-025 LC-G: assemble the stand-alone golden packages next to G-A / G-B from the collected runs and write one MANIFEST.sha256 over everything under the
golden directory (except runs/, which keeps the raw run outputs, and the manifest itself).

  <golden>/G-CFGH/<run>/{lcgold/{events.jsonl,blobs.bin,header.json[,hooks.jsonl]},run-meta.json}   every lc=1 run (plain original wasm and hook runs)
  <golden>/G-CFGH/attempts.jsonl, summary.json                      analyze_lc.py output over those runs (candidate selection, picks, U2 policy comparison)
  <golden>/G-DE/<run>/{lcgold/...,run-meta.json}                     hook runs only: 1128/1146/660/655/658/1124 inputs and outputs, u2-dist, unlink-* (hooks.jsonl)
  <golden>/G-DE/verify_hooks.json                                   verify_hooks.py output (F1..F5, D1)

usage: package_golden.py --golden <dir> --plain <name>=<run dir> [...] --hook <name>=<run dir> [...]"""
import argparse
import hashlib
import pathlib
import shutil
import subprocess
import sys

HERE = pathlib.Path(__file__).resolve().parent


def sha(p):
    return hashlib.sha256(pathlib.Path(p).read_bytes()).hexdigest()


def copy_run(name, d, dst_root):
    dst = dst_root / name
    if dst.exists():
        return
    (dst / 'lcgold').mkdir(parents=True)
    for f in pathlib.Path(d, 'lcgold').iterdir():
        if f.name in ('events.jsonl', 'blobs.bin', 'header.json', 'hooks.jsonl'):
            shutil.copyfile(f, dst / 'lcgold' / f.name)
    shutil.copyfile(pathlib.Path(d) / 'meta.json', dst / 'run-meta.json')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--golden', type=pathlib.Path, required=True)
    ap.add_argument('--plain', action='append', default=[])
    ap.add_argument('--hook', action='append', default=[])
    a = ap.parse_args()
    g = a.golden
    for spec in a.plain + a.hook:
        name, d = spec.split('=', 1)
        copy_run(name, d, g / 'G-CFGH')
    for spec in a.hook:
        name, d = spec.split('=', 1)
        copy_run(name, d, g / 'G-DE')
    if a.plain + a.hook and not (g / 'G-CFGH' / 'attempts.jsonl').exists():
        tmp = g / 'G-CFGH' / '_an'
        cmd = [sys.executable, '-B', '-I', str(HERE / 'analyze_lc.py'), '--out', str(tmp)]
        for spec in a.plain + a.hook:
            name, _ = spec.split('=', 1)
            cmd += ['--run', f'{name}={g / "G-CFGH" / name}']
        subprocess.run(cmd, check=True, stdout=subprocess.DEVNULL)
        for f in ('attempts.jsonl', 'summary.json'):
            shutil.move(str(tmp / f), str(g / 'G-CFGH' / f))
        tmp.rmdir()
    if a.hook and not (g / 'G-DE' / 'verify_hooks.json').exists():
        cmd = [sys.executable, '-B', '-I', str(HERE / 'verify_hooks.py'), '--out', str(g / 'G-DE' / 'verify_hooks.json')]
        for spec in a.hook:
            name, _ = spec.split('=', 1)
            cmd += ['--run', f'{name}={g / "G-DE" / name}']
        subprocess.run(cmd, check=True, stdout=subprocess.DEVNULL)
    lines = []
    skip_dirs = ('runs', 'probe', 'bundles', 'replays')   # raw run outputs / instrumented wasm / bundle+replay originals (LC-6a keeps the copies it needs)
    skip_names = ('MANIFEST.sha256', 'READY', 'run-log.tsv', 'hang_hunt.log', 'hang_hunt2.log', 'G-A.make.log')
    for p in sorted(g.rglob('*')):
        rel = p.relative_to(g)
        if p.is_file() and rel.parts[0] not in skip_dirs and p.name not in skip_names and not p.name.endswith(('.out', '.pause.json', '.done', 'search.log')):
            lines.append('%s  %s' % (sha(p), rel))
    (g / 'MANIFEST.sha256').write_text('\n'.join(lines) + '\n')
    print('manifest entries', len(lines))


if __name__ == '__main__':
    main()

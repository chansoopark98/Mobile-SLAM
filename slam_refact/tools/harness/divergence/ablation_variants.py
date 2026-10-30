#!/usr/bin/env python3
"""Ablation variants of an exact-input bundle (diagnosis only; no gate or engine change).

Starting from an exact-input bundle E (make_variant --capture-binding --capture-wallclock --queue-times
--capture-pyramids [--id-seed]) and the gate-mode bundle G of the same scenario (prepare_inputs.py
--drop-mask-from r1), replace a chosen subset of exact inputs by their gate-mode approximation:

  T  frame time      column 9 (frame_time_ns, the 647 tracking time) := video time (column 4)
  B  binding         sensors.tsv := G's (recorded receive times) and column 3 (binding time) := video time
                     (bundle rule: push rows with receive_time <= t); column 9 kept unless T
  S  id seed         id_generator_seed := 5489 (mt19937 default seed, the gate replays' value)
  P  pyramid         column 8 := '-' (engine CPU swiftshader-approx pyramid) on every stage
  D  drop mask       column 6 (process) := G's (r1 mask) instead of the run's own processed frames
  L  linear acc.     sensors lin columns := G's (DERIVED-LINACC file) instead of the recorded queue's (absolute only)
  b  half of B: sensors.tsv := G's only (binding time column 3 stays the recorded wall clock)
  f  half of B: column 3 := video time only (sensor receive times stay the recorded queue times)

With all five the frames/sensors are G's (paths and times identical), which the caller checks by
replaying it against the gate replay.

usage: ablation_variants.py --exact E --gate G --out OUT --approx TBSPD
"""
import argparse
import hashlib
import json
import pathlib
import sys

SEED_GATE = 5489


def read_tsv(path):
    return [line.rstrip('\n').split('\t') for line in open(path) if line.strip()]


def write_tsv(path, rows):
    with open(path, 'w') as f:
        for r in rows:
            f.write('\t'.join(r) + '\n')


def sha(path):
    return hashlib.sha256(pathlib.Path(path).read_bytes()).hexdigest()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--exact', required=True)
    ap.add_argument('--gate', required=True)
    ap.add_argument('--out', required=True)
    ap.add_argument('--bind-lag-ms', type=float, default=0.0,
                    help='with B or f: binding time = video time + this lag (diagnostic: a lagged bundle rule)')
    ap.add_argument('--approx', default='', help='subset of TBSPD (or b / f, the two halves of B)')
    a = ap.parse_args()
    approx = set(a.approx)
    if approx - set('TBSPDLbf'):
        sys.exit('bad --approx')
    E, G, out = pathlib.Path(a.exact), pathlib.Path(a.gate), pathlib.Path(a.out)
    out.mkdir(parents=True, exist_ok=False)
    ef, gf = read_tsv(E / 'frames.tsv'), read_tsv(G / 'frames.tsv')
    if len(ef) != len(gf) or any(x[1] != y[1] or x[3] != y[3] or x[6] != y[6] for x, y in zip(ef, gf)):
        sys.exit('frames.tsv of E and G do not describe the same stages')
    es, gs = read_tsv(E / 'sensors.tsv'), read_tsv(G / 'sensors.tsv')
    # Columns: phase t_ns aIG(3) lin(3|-) rate(3) interval_ms orientation(3). Only receive times (and, for an
    # absolute bundle built with --queue-linacc, the lin columns) may differ between E and G.
    if len(es) != len(gs) or any(x[0] != y[0] or x[2:5] != y[2:5] or x[8:] != y[8:] for x, y in zip(es, gs)):
        sys.exit('sensors.tsv of E and G differ in values (only receive times / lin may differ)')
    changed = {k: 0 for k in 'TBPD'}
    frames = []
    for x, y in zip(ef, gf):
        r = list(x)
        while len(r) < 9:
            r.append('0')
        if 'T' in approx and r[0] == 'frame' and r[8] != r[3]:
            r[8] = r[3]
            changed['T'] += 1
        if 'B' in approx or 'f' in approx:
            bt = str(int(r[3]) + int(round(a.bind_lag_ms * 1e6)))
            if r[2] != bt:
                r[2] = bt
                changed['B'] += 1
        if 'P' in approx and r[7] != '-':
            r[7] = '-'
            changed['P'] += 1
        if 'D' in approx and r[5] != y[5]:
            r[5] = y[5]
            changed['D'] += 1
        frames.append(r)
    sensors = []
    for x, y in zip(es, gs):
        r = list(x)
        if 'B' in approx or 'b' in approx:
            r[1] = y[1]
        if 'L' in approx:
            r[5:8] = y[5:8]
        sensors.append(r)
    if 'B' in approx or 'b' in approx:
        changed['B_sensor_rows_retimed'] = sum(x[1] != y[1] for x, y in zip(es, gs))
    changed['L_rows_differing'] = sum(x[5:8] != y[5:8] for x, y in zip(es, gs))
    write_tsv(out / 'frames.tsv', frames)
    write_tsv(out / 'sensors.tsv', sensors)
    cfg = read_tsv(E / 'engine-config.tsv')
    new_cfg = []
    for r in cfg:
        if r[0] == 'id_generator_seed' and 'S' in approx:
            r = [r[0], str(SEED_GATE)]
            changed['S_from'] = next(c[1] for c in cfg if c[0] == 'id_generator_seed')
        if r[0] == 'pyramid_injection' and 'P' in approx:
            r = [r[0], 'none']
        if r[0] == 'drop_mode' and 'D' in approx:
            r = [r[0], 'reference-r1-mask(ablation)']
        new_cfg.append(r)
    if 'S' in approx and not any(r[0] == 'id_generator_seed' for r in cfg):
        new_cfg.append(['id_generator_seed', str(SEED_GATE)])
    write_tsv(out / 'engine-config.tsv', new_cfg)
    b = json.loads((E / 'bundle.json').read_text())
    b['files'] = {f: sha(out / f) for f in ('sensors.tsv', 'frames.tsv', 'engine-config.tsv')}
    b['ablation'] = {'exact': str(E), 'gate': str(G), 'approx': ''.join(c for c in 'TBSPDLbf' if c in approx),
                     'bindLagMs': a.bind_lag_ms, 'changed': changed, 'argv': sys.argv}
    (out / 'bundle.json').write_text(json.dumps(b, indent=1) + '\n')
    print(json.dumps(b['ablation']))


if __name__ == '__main__':
    main()

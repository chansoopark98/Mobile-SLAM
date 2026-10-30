#!/usr/bin/env python3
"""Scenario data of the image-target scale gate `it-scale-gate` (US-015; preregistration/it-scale-gate.v1.md section 1).

The gate drives the engine's VioTracker with virtual feature photos of a planar target taken from an analytic camera orbit,
and with a *scripted* VIO extrinsic (tam). Everything the C++ test needs is the scenario table written here
(scenarios.tsv + variant.txt); the photos and the ground truth are computed from it in C++ (image_target_scale_test.cc).

Two variants:
  dev         fixed numbers, the only variant any engine run uses before the independent review (null-run, mutation, reports).
  registered  a deterministic function of the coordinator seed (derive_registered); the seed file is a required argument, the
              tool refuses to run without it, and the executor never adjusts a derived value.

  python3 -I its_gate_data.py render --variant dev --out <dir> [--force]
  python3 -I its_gate_data.py render --variant registered --seed-file <coordinator-seed.txt> --out <dir> [--force]
  python3 -I its_gate_data.py render --variant dev-seed --seed <int> --out <dir> [--force]

`dev-seed` applies the registered derivation to a seed the EXECUTOR chose (range and robustness checks before the review) and
labels the table `dev`, so that the dev tests accept it and the held tests never do. Its seeds are listed in the gate file
(section 1) and the coordinator seed must not be one of them.

Only the standard library is used (development tool).
"""
import argparse
import hashlib
import os
import pathlib
import sys

COLUMNS = ['id', 'kind', 'k', 'target_seed', 'rho', 'span_deg', 'speed', 'n_frames', 'tilt_deg', 'tilt_az_deg',
           'height_ratio', 'world_yaw_deg', 'world_pitch_deg', 'world_roll_deg', 'wx', 'wy', 'wz',
           'loss_start', 'loss_len', 'noise_px', 'pair_of', 'pair_c']

# kinds (gate classification in it-scale-gate.v1.md section 2):
#   main        S1 scale convergence, S2 world pose, S3 world stationarity
#   zero        S0 camera moves, the scripted tam keeps its rotation but loses its translation (disableVio-like)
#   slow        S0 per-frame camera step below the 3e-2 baseline threshold of computeScale
#   scale_pair  S4 world scaled by pair_c (same photos, tam translation x pair_c) of the scenario pair_of
#   yaw_pair    S5 world rotated about the vertical axis by pair_c degrees (same photos) of the scenario pair_of
#   loss        S6 REPORT: the target disappears for loss_len frames from loss_start
#   noise       REPORT: pixel noise noise_px on the feature locations (one random realisation: a small sample, protocol rule 8)
#   null_*      dev only: null-run inputs (truth = prior; identity pairs)

DEV_ROWS = [
    # id, kind, k, target_seed, rho, span, speed, n, tilt, tilt_az, hr, wyaw, wpitch, wroll, wx, wy, wz, ls, ll, noise, pair_of, pair_c
    ['dA', 'main', 0.38, 4711, 1.30, 32.0, 0.045, 140, 12.0, 40.0, 0.10, 25.0, 3.0, -2.0, 0.10, -0.20, 0.30, 0, 0, 0.0, '', 0.0],
    ['dB', 'main', 0.72, 5821, 1.25, 30.0, 0.050, 140, 8.0, 200.0, 0.12, -70.0, -4.0, 5.0, -0.30, 0.10, 0.45, 0, 0, 0.0, '', 0.0],
    ['dC', 'main', 1.15, 6931, 1.35, 35.0, 0.055, 140, 15.0, 310.0, 0.08, 140.0, 2.0, 3.0, 0.40, 0.00, -0.20, 0, 0, 0.0, '', 0.0],
    ['dD', 'main', 1.90, 7043, 1.20, 28.0, 0.042, 140, 6.0, 100.0, 0.15, -120.0, 6.0, -4.0, -0.10, 0.30, 0.10, 0, 0, 0.0, '', 0.0],
    ['dZ0', 'zero', 1.00, 8101, 1.30, 30.0, 0.050, 100, 10.0, 70.0, 0.10, 50.0, 0.0, 0.0, 0.00, 0.00, 0.00, 0, 0, 0.0, '', 0.0],
    ['dZ1', 'slow', 1.10, 8203, 1.30, 30.0, 0.012, 100, 10.0, 250.0, 0.10, -30.0, 0.0, 0.0, 0.00, 0.00, 0.00, 0, 0, 0.0, '', 0.0],
    ['dE1', 'scale_pair', 0.72, 5821, 1.25, 30.0, 0.050, 140, 8.0, 200.0, 0.12, -70.0, -4.0, 5.0, -0.30, 0.10, 0.45, 0, 0, 0.0, 'dB', 1.60],
    ['dE2', 'yaw_pair', 1.15, 6931, 1.35, 35.0, 0.055, 140, 15.0, 310.0, 0.08, 140.0, 2.0, 3.0, 0.40, 0.00, -0.20, 0, 0, 0.0, 'dC', 75.0],
    ['dL0', 'loss', 0.72, 5821, 1.25, 30.0, 0.050, 140, 8.0, 200.0, 0.12, -70.0, -4.0, 5.0, -0.30, 0.10, 0.45, 70, 10, 0.0, 'dB', 0.0],
    ['dN1', 'noise', 1.15, 6931, 1.35, 35.0, 0.055, 140, 15.0, 310.0, 0.08, 140.0, 2.0, 3.0, 0.40, 0.00, -0.20, 0, 0, 0.5, 'dC', 0.0],
    ['dN2', 'noise', 1.15, 6931, 1.35, 35.0, 0.055, 140, 15.0, 310.0, 0.08, 140.0, 2.0, 3.0, 0.40, 0.00, -0.20, 0, 0, 1.0, 'dC', 0.0],
    # null-run rows (dev only, never derived for the registered variant): truth = the public prior 0.5, and identity pairs
    ['dN0', 'null_main', 0.50, 9311, 1.30, 30.0, 0.050, 140, 10.0, 120.0, 0.10, 15.0, 2.0, -3.0, 0.20, -0.10, 0.30, 0, 0, 0.0, '', 0.0],
    ['dE1n', 'null_scale_pair', 0.72, 5821, 1.25, 30.0, 0.050, 140, 8.0, 200.0, 0.12, -70.0, -4.0, 5.0, -0.30, 0.10, 0.45, 0, 0, 0.0, 'dB', 1.00],
    ['dE2n', 'null_yaw_pair', 1.15, 6931, 1.35, 35.0, 0.055, 140, 15.0, 310.0, 0.08, 140.0, 2.0, 3.0, 0.40, 0.00, -0.20, 0, 0, 0.0, 'dC', 0.0],
]


def _seed_u(seed, label):
    """Uniform [0, 1) from sha256('<coordinator seed>:<label>') (first 8 bytes, big endian)."""
    h = hashlib.sha256(f'{seed}:{label}'.encode()).digest()
    return int.from_bytes(h[:8], 'big') / 2.0 ** 64


def _range(seed, label, lo, hi, digits):
    return round(lo + (hi - lo) * _seed_u(seed, label), digits)


def _int(seed, label, lo, hi):
    return lo + int(_seed_u(seed, label) * (hi - lo + 1))


# k ranges of the four main scenarios: below the public initial guess 0.5, and three above it.
K_RANGES = {'A': (0.35, 0.45), 'B': (0.60, 0.85), 'C': (1.00, 1.40), 'D': (1.60, 2.20)}


def _main_row(seed, tag):
    s = lambda name: f'main.{tag}.{name}'
    lo, hi = K_RANGES[tag]
    return [
        f'r{tag}', 'main', _range(seed, s('k'), lo, hi, 3), _int(seed, s('target_seed'), 1000, 999999),
        _range(seed, s('rho'), 1.15, 1.45, 3), _range(seed, s('span'), 25.0, 40.0, 1),
        _range(seed, s('speed'), 0.040, 0.060, 4), 140,
        _range(seed, s('tilt'), 5.0, 20.0, 1), _range(seed, s('tilt_az'), 0.0, 360.0, 1),
        _range(seed, s('hr'), 0.05, 0.20, 3), _range(seed, s('wyaw'), -180.0, 180.0, 1),
        _range(seed, s('wpitch'), -10.0, 10.0, 1), _range(seed, s('wroll'), -10.0, 10.0, 1),
        _range(seed, s('wx'), -0.5, 0.5, 3), _range(seed, s('wy'), -0.5, 0.5, 3), _range(seed, s('wz'), -0.5, 0.5, 3),
        0, 0, 0.0, '', 0.0]


def derive_registered(seed):
    """The registered variant, a deterministic function of the coordinator seed (procedure in it-scale-gate.v1.md section 1)."""
    rows = [_main_row(seed, t) for t in 'ABCD']
    by = {r[0]: r for r in rows}
    z0 = list(by['rC'])
    z0[0:2] = ['rZ0', 'zero']
    z0[2] = _range(seed, 'zero.k', 0.80, 1.20, 3)
    z0[3] = _int(seed, 'zero.target_seed', 1000, 999999)
    z0[7] = 100
    z1 = list(by['rD'])
    z1[0:2] = ['rZ1', 'slow']
    z1[2] = _range(seed, 'slow.k', 0.80, 1.40, 3)
    z1[3] = _int(seed, 'slow.target_seed', 1000, 999999)
    z1[6] = _range(seed, 'slow.speed', 0.008, 0.015, 4)
    z1[7] = 100
    e1 = list(by['rB'])
    e1[0:2] = ['rE1', 'scale_pair']
    e1[20:22] = ['rB', _range(seed, 'pair.scale_c', 1.40, 2.00, 3)]
    e2 = list(by['rC'])
    e2[0:2] = ['rE2', 'yaw_pair']
    e2[20:22] = ['rC', _range(seed, 'pair.yaw_deg', 30.0, 150.0, 1)]
    l0 = list(by['rB'])
    l0[0:2] = ['rL0', 'loss']
    l0[17:19] = [_int(seed, 'loss.start', 60, 80), _int(seed, 'loss.len', 8, 12)]
    l0[20:22] = ['rB', 0.0]
    n1 = list(by['rC'])
    n1[0:2] = ['rN1', 'noise']
    n1[19] = 0.5
    n1[20:22] = ['rC', 0.0]
    n2 = list(n1)
    n2[0] = 'rN2'
    n2[19] = 1.0
    return rows + [z0, z1, e1, e2, l0, n1, n2]


def _fmt(v):
    return repr(v) if isinstance(v, float) else str(v)


def write_dataset(rows, variant, out, seed=None, force=False):
    out = pathlib.Path(out)
    if out.exists() and any(out.iterdir()):
        if not force:
            raise SystemExit(f'{out} is not empty (use --force)')
        for p in out.iterdir():
            if p.is_file():
                p.unlink()
    out.mkdir(parents=True, exist_ok=True)
    lines = ['\t'.join(COLUMNS)] + ['\t'.join(_fmt(v) for v in row) for row in rows]
    (out / 'scenarios.tsv').write_text('\n'.join(lines) + '\n')
    (out / 'variant.txt').write_text(variant + '\n')
    if seed is not None:
        (out / 'seed.txt').write_text(f'{seed}\n')


# Seeds the EXECUTOR may use for range and robustness checks before the review (variant dev-seed). The coordinator seed must be
# outside this range (its_finalize.py seed refuses it).
EXECUTOR_SEEDS = range(9100, 9200)
COORDINATOR_SEED_FILE = pathlib.Path(__file__).resolve().parents[3] / 'build/slam-refact-full-20261019/preregistration/it-scale-gate.v1.coordinator-seed.txt'


def read_seed(path):
    if not pathlib.Path(path).is_file():
        raise SystemExit(f'{path}: the coordinator seed file does not exist (BLOCKED: the coordinator writes it)')
    text = pathlib.Path(path).read_text().split()
    if not text or not text[0].lstrip('-').isdigit():
        raise SystemExit(f'{path}: the first token must be the integer coordinator seed')
    return int(text[0])


def main(argv):
    ap = argparse.ArgumentParser()
    sub = ap.add_subparsers(dest='cmd', required=True)
    r = sub.add_parser('render')
    r.add_argument('--variant', choices=['dev', 'registered', 'dev-seed'], default='dev')
    r.add_argument('--seed-file')
    r.add_argument('--seed', type=int)
    r.add_argument('--out', required=True)
    r.add_argument('--force', action='store_true')
    a = ap.parse_args(argv)
    if a.variant == 'dev':
        if a.seed_file or a.seed is not None:
            raise SystemExit('the dev variant takes no seed')
        write_dataset(DEV_ROWS, 'dev', a.out, force=a.force)
    elif a.variant == 'dev-seed':
        if a.seed is None or a.seed_file:
            raise SystemExit('dev-seed needs --seed <int> (and no --seed-file)')
        if a.seed not in EXECUTOR_SEEDS:
            raise SystemExit(f'dev-seed accepts only the executor seeds {EXECUTOR_SEEDS.start}..{EXECUTOR_SEEDS.stop - 1}')
        if COORDINATOR_SEED_FILE.exists() and read_seed(COORDINATOR_SEED_FILE) == a.seed:
            raise SystemExit('dev-seed refuses the coordinator seed')
        write_dataset(derive_registered(a.seed), 'dev', a.out, seed=a.seed, force=a.force)
    else:
        if a.seed is not None:
            raise SystemExit('the registered variant reads its seed from --seed-file only')
        if not a.seed_file:
            raise SystemExit('the registered variant needs --seed-file <coordinator seed file> (no default, no dev seed)')
        seed = read_seed(a.seed_file)
        write_dataset(derive_registered(seed), 'registered', a.out, seed=seed, force=a.force)
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))

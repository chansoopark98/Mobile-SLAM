#!/usr/bin/env python3
"""OR-4 image-target world regression gate: runner, metrics, verdict, held-run and analysis guards (development tool).

Gate: build/slam-refact-full-20261019/preregistration/or4-it-world-regression.v6.md (registered before any run of
the v6 arms; section numbers in the comments refer to it).

The main judgement (GATE) compares the run with the target loaded (T) with the isolation arm D (no target, allOrb /
allGorb forced as a loaded target forces them, same seeds); T against the plain run N is a REPORT (v5 section 0).

Commands (repo root, python3 -I):
  pin-binary                 (v3, done) copy the built binaries to or4/v3/bin/, record sha256 and the engine source state
  null-engine                (v3, done) N arm: 14 bundles x (4 id seeds, 3 RNG seeds, s0 repeat) + baseline binary run
  decomp-engine              D arm: 14 bundles x (id seeds s0..s3, RNG seeds r1..r3, s0 repeat), allOrb/allGorb forced, no target
  mutation-engine            registered engine-level mutations on the D arm of the 3 mutation bundles
  analyze-null               noise tables R_N and R_D, determinism / harness checks, mutation verdicts, power table (written once)
  original-null              (v3, done) null-run of the original-side metrics on the existing plain runs r1 / r2
  verify-held                evaluates the held conditions without running anything
  real-engine                HELD: T arm (target loaded) 14 bundles x 4 id seeds + the s0 repeat
  analyze-real               HELD (analysis guard): verdicts and REPORTs (written once, never replaced, ledger)
  real-original              HELD: original SDK, 5 scenarios x (3 runs without + 3 runs with the target)
  analyze-original           original within / cross pair metrics next to the engine tables (REPORT)
  make-manifest              writes the manifest at registration (gate sha256, pins, input pins, snapshot)

The engine side is deterministic (the null-runs prove it); the original side is not (REPORT only, n = 3).
Nothing here is part of the product runtime.
"""
import argparse
import concurrent.futures
import datetime
import hashlib
import json
import math
import os
import pathlib
import shutil
import subprocess
import sys
import time

import numpy as np

ROOT = pathlib.Path(__file__).resolve().parents[3]
PREREG = ROOT / 'build/slam-refact-full-20261019/preregistration'
OR4 = ROOT / 'build/slam-refact-full-20261019/or4'
GATE_ID = 'or4-it-world-regression'
VERSION = 6   # v1-v5 were superseded before any real run (see the gate files)
OUT_VERSION = 6
OR4V = OR4 / f'v{OUT_VERSION}'    # new evidence of v6
OR4V5 = OR4 / 'v5'                # reused evidence of v5: D arm runs s1..r3, force-repeat, D-arm mutation runs (same pinned binary)
OR4V3 = OR4 / 'v3'                # reused evidence of v3 (same pinned binary): N arm, binaries, D arm run 'force'
GATE_MD = PREREG / f'{GATE_ID}.v{VERSION}.md'
MANIFEST = PREREG / f'{GATE_ID}.v{VERSION}.manifest.json'
NOISE_TABLE = PREREG / f'{GATE_ID}.v{VERSION}.noise-table.json'
DESIGNATION = PREREG / f'{GATE_ID}.v{VERSION}.review-designation.json'
B_SEED = 'build/slam-refact-20261016/fix/round2/bundles-seed'
B_ABS = 'build/slam-refact-20261016/ablation/abs/bundles-seed'
ENGINE_BUILT = ROOT / 'build/slam-refact-full-it2/tests/image_target/sr_or4_replay'
BASELINE_BUILT = ROOT / 'build/slam-refact-full-it2/harness/slam_refact_replay'
ENGINE_PIN = OR4V3 / 'bin/sr_or4_replay'
BASELINE_PIN = OR4V3 / 'bin/slam_refact_replay'
SOURCE_STATE = OR4V3 / 'bin/engine-source-state.json'
TARGET_DIR = OR4 / 'target'
EVAL_DIR = ROOT / 'slam_refact/tools/harness/eval'
RUNNER_IT = ROOT / 'slam_refact/tools/re/runtime/run-reference-it.mjs'

# Gate section 2: the 14 exact-input bundles (regress/run_exact.sh). cls: "cpu" = CPU pyramid (real ROI draw),
# "injected" = recorded GPU atlas, zero-padded ROI rows when a target is loaded.
BUNDLES = [(f'{s}__{l}', f'{B_SEED}/{s}/{l}', 'cpu' if s == 'flip6-n1' else 'injected', s) for s, l in [
    ('flip6-n1', 'pmr3b'), ('flip6-n1', 'pmr4a'), ('flip6-s2', 'pmr3a'), ('flip6-s2', 'pmr4a'),
    ('flip6-s3-lowtex', 'pmr3a'), ('flip6-s3-lowtex', 'pmr4a'), ('tum-room1-full', 'pmr3a'),
    ('tum-room1-full', 'pmr4a'), ('tum-room1-lowtex', 'pmr3b'), ('tum-room1-lowtex', 'pmr4a')]] + [
    (f'{s}__{l}', f'{B_ABS}/{s}/{l}', 'injected', s) for s, l in [
        ('flip6-s2-absolute', 'pma1'), ('flip6-s2-absolute', 'pma2'),
        ('tum-room1-full-absolute', 'pma1'), ('tum-room1-full-absolute', 'pma2')]]
SEED_MULT = 2654435761      # Knuth multiplicative constant; gate section 2
RNG_DEFAULT = 5489          # compat::kEngineRngSeed
METRICS = ('pos', 'rot', 'scale', 'cov', 'norm', 'reset')
# Gate section 4 floors. pos: fraction of the RMS radius of the reference trajectory (tracked rows); rot: deg; scale: |ln s|
# of the Sim(3) scale (0.05 = 5 %; by analogy with CLAUDE.md section 3 "TUM <= 1.05 x", not a reuse of that rule); counts: 3 x one frame.
FLOOR = {'pos': 0.05, 'rot': 1.0, 'scale': 0.05, 'cov': 3, 'norm': 3, 'reset': 3}
NOISE_FACTOR = 3.0
COV_LOSS_CAP = 0.01         # CLAUDE.md section 3: coverage loss <= 1 percentage point (of rows)
# Reporting classes chosen AFTER the v3 noise table was seen (disclosed in the gate file): a bundle is "powered" when its
# tolerances are at most these; they only classify the evidence, they do not change a verdict.
POWER_LIMITS = {'pos': 0.5, 'rot': 10.0, 'scale': math.log(1.5)}
# Engine-level mutations (D arm, gate section 7). Required: detected on every mutation bundle.
ENGINE_MUTATIONS = ('drop-every:7', 'orient-beta:3.0', 'orient-drift:0.5')
N_MUTATION_BUNDLES = 3      # the first three bundles in BUNDLES order whose D-arm noise R_D is zero on every metric
ORIGINAL_SCENARIOS = ('flip6-s2', 'tum-room1-full', 'flip6-n1', 'flip6-s3-lowtex', 'tum-room1-lowtex')
ORIGINAL_RUNS = 3
TIMING_KEYS = ('srStageMs', 'wallSeconds')
MAX_WORKERS = 2
RUN_TIMEOUT_S = 3600
# T vs N bundles whose plain-run tolerance D vs N already exceeded (v5 null-report, tracked-row metrics; declared, gate section 0.3).
EXPECTED_TN_EXCEED = ('flip6-n1__pmr3b', 'flip6-n1__pmr4a', 'flip6-s2__pmr3a', 'flip6-s2__pmr4a', 'flip6-s3-lowtex__pmr3a',
                      'flip6-s3-lowtex__pmr4a', 'flip6-s2-absolute__pma1', 'flip6-s2-absolute__pma2', 'tum-room1-full-absolute__pma1',
                      'tum-room1-full-absolute__pma2', 'tum-room1-lowtex__pmr3b')   # observed in the v5 null-report (D vs N, v5 metrics)

NOISE_RUNS = ['s0', 's1', 's2', 's3', 'r1', 'r2', 'r3']
D_RUNS = ['force', 's1', 's2', 's3', 'r1', 'r2', 'r3']     # D arm run names; 'force' is the s0 run (v3 name, reused)
D_PAIR = {0: 'force', 1: 's1', 2: 's2', 3: 's3'}           # T(b, s_k) is compared with D(b, D_PAIR[k])

# Trace-level mutants (gate section 7): name -> (metric, analytic magnitude or None for counts).
TRACE_MUTANTS = {
    'TM-pos-10pct': ('pos', 0.10), 'TM-pos-3pct': ('pos', 0.03),
    'TM-scale-1.1': ('scale', math.log(1.1)), 'TM-scale-1.2': ('scale', math.log(1.2)),
    'TM-scale-1.5': ('scale', math.log(1.5)), 'TM-scale-0.9': ('scale', math.log(1 / 0.9)),
    'TM-scale-1.02': ('scale', math.log(1.02)), 'TM-scale-1.5-init': ('scale', 0.0),
    'TM-rot-2deg': ('rot', 2.0), 'TM-rot-0.5deg': ('rot', 0.5),
    'TM-norm-every10': ('norm', None), 'TM-norm-2': ('norm', 2),
    'TM-cov-every20': ('cov', None), 'TM-cov-2': ('cov', 2),
    'TM-reset-5': ('reset', 5), 'TM-reset-2': ('reset', 2)}


# ------------------------------------------------------------------------------------------------- small helpers
def sha256_file(path):
    h = hashlib.sha256()
    with open(path, 'rb') as f:
        for chunk in iter(lambda: f.read(1 << 20), b''):
            h.update(chunk)
    return h.hexdigest()


def sha256_bytes(b):
    return hashlib.sha256(b).hexdigest()


def tree_sha(path):
    """sha256 of the sorted (relative path, file sha256) list; __pycache__ and *.pyc excluded (protocol 12.6)."""
    p = pathlib.Path(path)
    entries = []
    for f in sorted(p.rglob('*')):
        if f.is_file() and '__pycache__' not in f.parts and f.suffix != '.pyc':
            entries.append(f'{f.relative_to(p)}\t{sha256_file(f)}')
    return sha256_bytes('\n'.join(entries).encode())


def trace_list_sha(*roots):
    """sha256 of the sorted (relative path, sha256) list of every trace.jsonl below the roots (the N arm, the D arm)."""
    entries = []
    for root in roots:
        root = pathlib.Path(root)
        if root.is_dir():
            entries += [f'{root.name}/{f.relative_to(root)}\t{sha256_file(f)}' for f in sorted(root.rglob('trace.jsonl'))]
    return sha256_bytes('\n'.join(sorted(entries)).encode()), len(entries)


def d_roots():
    return (OR4V3 / 'decomp/D', OR4V5 / 'decomp/D')


def d_dir(key, name):
    """Directory of the D arm run `name` of a bundle (the v3 run 'force' lives in the v3 evidence directory)."""
    for r in d_roots():
        if (r / key / name).is_dir():
            return r / key / name
    return OR4V5 / 'decomp/D' / key / name


def manifest_core_sha(m):
    """sha256 of the manifest without review and first_real_run_at (sorted keys, compact JSON): what the designation binds."""
    core = {k: v for k, v in m.items() if k not in ('review', 'first_real_run_at')}
    return sha256_bytes(json.dumps(core, sort_keys=True, separators=(',', ':')).encode())


def seeds_of(s0):
    return [(s0 + k * SEED_MULT) % (1 << 32) for k in range(4)]


def rng_seeds():
    return [(RNG_DEFAULT + k * SEED_MULT) % (1 << 32) for k in range(1, 4)]


def bundle_seed(bundle_dir):
    for line in (ROOT / bundle_dir / 'engine-config.tsv').read_text().splitlines():
        k, _, v = line.partition('\t')
        if k == 'id_generator_seed':
            return int(v)
    raise SystemExit(f'{bundle_dir}: no id_generator_seed in engine-config.tsv')


def bundle_sha(bundle_dir):
    d = ROOT / bundle_dir
    h = hashlib.sha256()
    for name in ('bundle.json', 'engine-config.tsv', 'frames.tsv', 'sensors.tsv'):
        h.update(name.encode() + b'\0' + (d / name).read_bytes())
    return h.hexdigest()


def load_trace(path):
    with open(path) as f:
        return [json.loads(l) for l in f if l.strip()]


def strip_timing(o):
    if isinstance(o, dict):
        return {k: strip_timing(v) for k, v in o.items() if k not in TIMING_KEYS and not k.startswith('srTiming')}
    if isinstance(o, list):
        return [strip_timing(v) for v in o]
    return o


def stripped_lines(rows):
    return [json.dumps(strip_timing(r), sort_keys=True) for r in rows]


def identical(rows_a, rows_b):
    """Bit identity of two traces (timing fields excluded); returns (bool, first differing index or None)."""
    a, b = stripped_lines(rows_a), stripped_lines(rows_b)
    if len(a) != len(b):
        return False, min(len(a), len(b))
    for i, (x, y) in enumerate(zip(a, b)):
        if x != y:
            return False, i
    return True, None


# ------------------------------------------------------------------------------------------------- metrics (gate section 3)
def nearest_rank(xs, p):
    s = sorted(xs)
    return s[max(1, math.ceil(p / 100.0 * len(s))) - 1] if s else None


def quat_angle_deg(qa, qb):
    """Angle of qa^-1 * qb as 2*atan2(|v|, |w|) (numerically stable near 0; the acos form of IT-P3 is not)."""
    wa, xa, ya, za = qa
    wb, xb, yb, zb = qb
    w = wa * wb + xa * xb + ya * yb + za * zb
    x = wa * xb - xa * wb - ya * zb + za * yb
    y = wa * yb + xa * zb - ya * wb - za * xb
    z = wa * zb - xa * yb + ya * xb - za * wb
    return math.degrees(2.0 * math.atan2(math.sqrt(x * x + y * y + z * z), abs(w)))


def tracked(r):
    """A row that counts for tracking: processed (fresh), finite pose, and not in INITIALIZING."""
    return bool(r.get('fresh') and r.get('p') and r.get('reason') != 'INITIALIZING')


def rms_radius(rows):
    """RMS distance of the finite positions of the tracked rows from their centroid: the trajectory scale the position
    error is measured against."""
    pts = np.array([r['p'] for r in rows if tracked(r)], float)
    if len(pts) < 2:
        return 0.0
    return float(math.sqrt(np.mean(np.sum((pts - pts.mean(0)) ** 2, axis=1))))


def path_length(rows):
    pts = [r['p'] for r in rows if r.get('p')]
    return sum(math.dist(a, b) for a, b in zip(pts, pts[1:]))


def sim3_log_scale(px, py):
    """|ln s| of the Umeyama Sim(3) scale s that maps the points py onto px (rotation and translation free)."""
    x, y = np.asarray(px, float), np.asarray(py, float)
    if len(x) < 3:
        return float('inf')
    x0, y0 = x - x.mean(0), y - y.mean(0)
    var_y = float(np.sum(y0 ** 2) / len(y))
    if var_y <= 0.0:
        return float('inf')
    u, d, vt = np.linalg.svd(x0.T @ y0 / len(x))
    s = np.eye(3)
    if np.linalg.det(u) * np.linalg.det(vt) < 0:
        s[2, 2] = -1
    scale = float(np.trace(np.diag(d) @ s) / var_y)
    return abs(math.log(scale)) if scale > 0 else float('inf')


def pair_metrics(rx, ry):
    """Metrics of trace Y against trace X of the same input frames (X is the reference arm).

    pos    p95 over the frames tracked in both of |p_X - p_Y| / S_X, S_X = RMS radius of X's tracked positions
    rot    p95 over the same frames of the angle between q_X and q_Y (degrees)
    scale  | ln s | of the Sim(3) scale between the positions of X and Y over the same frames
    cov    | #tracked X - #tracked Y |  (tracked = fresh, finite pose, reason != INITIALIZING; frames)
    norm   | #NORMAL X - #NORMAL Y | over fresh rows (frames)
    reset  | #srMapReset X - #srMapReset Y |  (engine traces; None when X has no srMapReset key)
    pos, rot and scale use the tracked rows only: INITIALIZING rows are not a map and would dilute a scale error (v4 F3).
    """
    by_y = {r['i']: r for r in ry}
    sx = rms_radius(rx)
    d, a, px, py = [], [], [], []
    for r in rx:
        o = by_y.get(r['i'])
        if o is None or not tracked(r) or not tracked(o):
            continue
        d.append(math.dist(r['p'], o['p']) / sx if sx > 0 else float('inf'))
        a.append(quat_angle_deg(r['q_wxyz'], o['q_wxyz']))
        px.append(r['p'])
        py.append(o['p'])
    cx = sum(1 for r in rx if tracked(r))
    cy = sum(1 for r in ry if tracked(r))
    nx = sum(1 for r in rx if r.get('fresh') and r.get('status') == 'NORMAL')
    ny = sum(1 for r in ry if r.get('fresh') and r.get('status') == 'NORMAL')
    has_reset = any('srMapReset' in r for r in rx)
    rsx = sum(1 for r in rx if r.get('srMapReset'))
    rsy = sum(1 for r in ry if r.get('srMapReset'))
    return {'pos': nearest_rank(d, 95), 'rot': nearest_rank(a, 95), 'scale': sim3_log_scale(px, py) if px else None,
            'cov': abs(cx - cy), 'norm': abs(nx - ny), 'reset': abs(rsx - rsy) if has_reset else None,
            'pairs': len(d), 'rows': len(rx), 'covX': cx, 'covY': cy, 'normalX': nx, 'normalY': ny}


def metric_maxima(rows):
    """Largest value each metric can take (two-sided absolute differences): rot 180 deg, the counts cov / norm / reset at most
    the number of rows (v5 used the reference trace's own count, which is not the maximum of |difference|). `tracked` is the
    reference trace's tracked-row count: the largest coverage LOSS (one-directional), used by the G-COV cap."""
    return {'rot': 180.0, 'cov': float(len(rows)), 'norm': float(len(rows)), 'reset': float(len(rows)),
            'tracked': float(sum(1 for r in rows if tracked(r)))}


def cov_cap_limit(noise_row, rows, maxima):
    """G-COV: the largest tolerated tracked-coverage loss (fraction of rows) = max(1 %, 3 R_D[cov] / rows). When it reaches the
    largest possible loss (all tracked rows) the cap can never fail and is vacuous (excluded from the verdict)."""
    limit = max(COV_LOSS_CAP, NOISE_FACTOR * (noise_row.get('cov') or 0.0) / rows) if rows else COV_LOSS_CAP
    vac = maxima is not None and rows > 0 and limit >= maxima['tracked'] / rows
    return limit, vac


# ------------------------------------------------------------------------------------------------- tables and verdict (gate section 4)
def noise_table(n_traces):
    """R[b][m] = max over unordered pairs of the noise runs of m(X, Y). n_traces: {bundle: [rows per noise run]}."""
    table = {}
    for b, traces in n_traces.items():
        vals = {m: [] for m in METRICS}
        for i in range(len(traces)):
            for j in range(i + 1, len(traces)):
                pm = pair_metrics(traces[i], traces[j])
                for m in METRICS:
                    if pm[m] is not None:
                        vals[m].append(pm[m])
        table[b] = {m: (max(v) if v else None) for m, v in vals.items()}
    return table


def tolerance(noise_row, m):
    r = noise_row.get(m)
    return max(NOISE_FACTOR * (r or 0.0), FLOOR[m])


def vacuous(noise_row, m, maxima):
    """The tolerance is at or above the largest value the metric can take: the metric cannot fail (excluded from the verdict)."""
    return maxima is not None and m in maxima and tolerance(noise_row, m) >= maxima[m]


def effect_table(ref_traces, t_traces):
    """E[b][m] = max over the paired runs of m(ref(b,s), T(b,s)); also the worst tracked-coverage loss (T relative to ref)."""
    table = {}
    for b, nts in ref_traces.items():
        tts = t_traces[b]
        vals = {m: [] for m in METRICS}
        loss = []
        for n, t in zip(nts, tts):
            pm = pair_metrics(n, t)
            for m in METRICS:
                if pm[m] is not None:
                    vals[m].append(pm[m])
            loss.append((pm['covX'] - pm['covY']) / pm['rows'] if pm['rows'] else 1.0)
        table[b] = {m: (max(v) if v else None) for m, v in vals.items()}
        table[b]['cov_loss_max'] = max(loss)
        table[b]['rows'] = len(nts[0])
    return table


def band_verdict(noise, effect, maxima=None):
    """Per bundle and metric: PASS iff E <= max(3 R, F), the tracked-coverage loss cap on top. A None effect (metric
    undefined) is a FAIL, not a skip. A metric whose tolerance is vacuous (>= its largest possible value) is excluded and marked."""
    out = {}
    for b, e in effect.items():
        row = {}
        for m in METRICS:
            tol = tolerance(noise[b], m)
            if vacuous(noise[b], m, (maxima or {}).get(b)):
                row[m] = {'effect': e[m], 'noise': noise[b].get(m), 'tolerance': tol, 'vacuous': True, 'pass': True}
                continue
            ok = e[m] is not None and e[m] <= tol
            row[m] = {'effect': e[m], 'noise': noise[b].get(m), 'tolerance': tol, 'vacuous': False, 'pass': ok}
        limit, vac = cov_cap_limit(noise[b], e['rows'], (maxima or {}).get(b))
        row['cov_cap'] = {'loss': e['cov_loss_max'], 'cap': limit, 'vacuous': vac, 'pass': vac or e['cov_loss_max'] <= limit}
        row['pass'] = all(row[m]['pass'] for m in METRICS) and row['cov_cap']['pass']   # G-COV is part of G-BAND
        out[b] = row
    return out


def exercised_positive(meta):
    """Positive evidence that the target path ran, from the meta.json of one run: a target is loaded and the engine requested ROI
    slots for at least one frame (`framesWithNextRois`, present only when a target is loaded). The no-target N and D arms fail it."""
    return meta.get('targetsLoaded') == 1 and (meta.get('framesWithNextRois') or 0) >= 1


def observed_effect(ref_traces, t_traces):
    """REPORT: whether T differs from the reference arm in any trace field (not a gate: D already differs from N)."""
    return any(not identical(n, t)[0] for n, t in zip(ref_traces, t_traces))


def us014_decision(null_ok, run_ok, infra, det_t_ok, exercised_ok, powered):
    """R-US014 (gate v6 section 0.2), coded. powered: list of (bundle, class, band_pass) for the powered bundles only.
    Returns (PERMIT | HOLD, code, reasons). Non-powered bundles never enter (REPORT)."""
    reasons = []
    if not null_ok:
        return 'HOLD', 'PREMISE-NULL', ['G-DET-N / G-H / G-DET-D not all passed']
    if infra:
        return 'HOLD', 'INFRA', [f'INFRA runs {infra}: no permit, coordinator decides, new version']
    if not run_ok:
        return 'HOLD', 'PREMISE-RUN', ['a T run ended abnormally (product failure)']
    if not det_t_ok:
        return 'HOLD', 'PREMISE-DET-T', ['G-DET-T failed']
    if not exercised_ok:
        return 'HOLD', 'PREMISE-EXERCISED', ['G-EXERCISED failed: the target path did not run on every bundle']
    failed = [b for b, _, ok in powered if not ok]
    if failed:
        return 'HOLD', 'BAND-FAIL', [f'G-BAND (T vs D) failed on powered bundles {failed}']
    passed = [(b, c) for b, c, ok in powered if ok]
    if len(passed) < 3 or not any(c == 'cpu' for _, c in passed):
        return 'HOLD', 'POWER-SHORT', [f'powered PASS {len(passed)} (< 3) or no CPU-class powered PASS']
    return 'PERMIT', 'PERMIT', [f'powered PASS {len(passed)}']


def mutation_detected(noise_row, ref_rows, mutant_rows):
    """A mutant (reference arm with a registered world regression) is detected when the band verdict FAILs for it: a metric
    above the bundle's tolerance, or a tracked-coverage loss above the bundle's G-COV cap (unless that cap is vacuous).
    Returns (detected, failing metrics, pair metrics, cov_cap_exceeded)."""
    pm = pair_metrics(ref_rows, mutant_rows)
    limit, vac = cov_cap_limit(noise_row, pm['rows'], metric_maxima(ref_rows))
    cap_exceeded = (not vac) and bool(pm['rows']) and (pm['covX'] - pm['covY']) / pm['rows'] > limit
    fails = [m for m in METRICS if pm[m] is None or pm[m] > tolerance(noise_row, m)]
    return bool(fails) or cap_exceeded, fails, pm, cap_exceeded


def mutant_expectation(noise_row, mutant, pm):
    """(expected_detected, observed_value, tolerance) of a trace mutant: the metric it moves exceeds the bundle's tolerance
    (a mutant smaller than a bundle's tolerance cannot be detected there by construction)."""
    metric, _ = TRACE_MUTANTS[mutant]
    tol = tolerance(noise_row, metric)
    value = pm[metric]
    return value is not None and value > tol, value, tol


def power_entry(noise_row, maxima):
    tols = {m: tolerance(noise_row, m) for m in METRICS}
    vac = sorted(m for m in METRICS if vacuous(noise_row, m, maxima))
    return {'powered': all(tols[m] <= POWER_LIMITS[m] for m in POWER_LIMITS) and not any(m in vac for m in POWER_LIMITS),
            'tolerances': tols, 'vacuous': vac}


def trace_mutant(name, rows):
    """Deterministic output-level world regressions of a trace (a copy). pos: constant x offset of the stated fraction of the
    tracked RMS radius; scale: tracked positions scaled about the first tracked pose (`-init`: only the INITIALIZING rows);
    rot: yaw rotation applied to every tracked quaternion; norm: NORMAL rows set to LIMITED; cov: tracked rows set to LIMITED /
    INITIALIZING (a tracking loss, the pose is still published); reset: srMapReset set on rows that are not resets.
    Negative controls are smaller than the registered floors."""
    if name not in TRACE_MUTANTS:
        raise ValueError(name)
    out = json.loads(json.dumps(rows))
    sx = rms_radius(rows)
    trk = [r for r in out if tracked(r)]
    if name.startswith('TM-pos-'):
        off = float(name.split('-')[2].rstrip('pct')) / 100.0 * sx
        for r in trk:
            r['p'][0] += off
    elif name.startswith('TM-scale-'):
        c = float(name.split('-')[2])
        targets = [r for r in out if r.get('p') and not tracked(r)] if name.endswith('-init') else trk
        p0 = list(trk[0]['p']) if trk else [0.0, 0.0, 0.0]
        for r in targets:
            r['p'] = [p0[k] + c * (r['p'][k] - p0[k]) for k in range(3)]
    elif name.startswith('TM-rot-'):
        deg = float(name.split('-')[2].rstrip('deg'))
        h = math.radians(deg) / 2.0
        qw, qy = math.cos(h), math.sin(h)
        for r in trk:
            w, x, y, z = r['q_wxyz']
            r['q_wxyz'] = [qw * w - qy * y, qw * x + qy * z, qw * y + qy * w, qw * z - qy * x]
    elif name.startswith('TM-norm-every'):
        n = int(name[len('TM-norm-every'):])
        for k, r in enumerate(x for x in out if x.get('fresh') and x.get('status') == 'NORMAL'):
            if k % n == 0:
                r['status'] = 'LIMITED'
    elif name.startswith('TM-norm-'):
        for r in [x for x in out if x.get('fresh') and x.get('status') == 'NORMAL'][:int(name.split('-')[2])]:
            r['status'] = 'LIMITED'
    elif name.startswith('TM-cov-every'):
        n = int(name[len('TM-cov-every'):])
        for k, r in enumerate(trk):
            if k % n == 0:
                r['status'], r['reason'] = 'LIMITED', 'INITIALIZING'
    elif name.startswith('TM-cov-'):
        for r in trk[:int(name.split('-')[2])]:
            r['status'], r['reason'] = 'LIMITED', 'INITIALIZING'
    elif name.startswith('TM-reset-'):
        for r in [x for x in out[10:] if not x.get('srMapReset')][:int(name.split('-')[2])]:
            r['srMapReset'] = True
    return out


def classify_exit(rc, log_text, maxrss_kb=None, baseline_maxrss_kb=None, timed_out=False):
    """Exit class of an engine run (gate section 10): OK; INFRA = a full disk, or SIGKILL whose peak resident memory is not
    above twice the largest peak of the same bundle's no-target runs (an external OOM-kill / cancel; the coordinator decides,
    never a re-run by this tool); CRASH = every other abnormal exit, a hang past the timeout, and a SIGKILL without a memory
    baseline or with a peak above twice the baseline (the product used the memory): the bundle FAILs."""
    if timed_out:
        return 'CRASH'
    if rc == 0:
        return 'OK'
    if 'No space left on device' in log_text:
        return 'INFRA'
    if rc == -9:
        if maxrss_kb is not None and baseline_maxrss_kb is not None and maxrss_kb <= 2 * baseline_maxrss_kb:
            return 'INFRA'
        return 'CRASH'
    return 'CRASH'


# ------------------------------------------------------------------------------------------------- held-run check (protocol 12.5, 12.6)
def live_pins():
    chrome = pathlib.Path(os.path.realpath('/usr/bin/google-chrome'))
    pins = {
        'engine_binary': sha256_file(ENGINE_PIN) if ENGINE_PIN.exists() else 'unavailable',
        'baseline_replay_binary': sha256_file(BASELINE_PIN) if BASELINE_PIN.exists() else 'unavailable',
        'engine_source_state': sha256_file(SOURCE_STATE) if SOURCE_STATE.exists() else 'unavailable',
        'node': sha256_file(pathlib.Path(os.path.realpath(shutil.which('node') or '/nonexistent'))) if shutil.which('node') else 'unavailable',
        'python': sha256_file(pathlib.Path(os.path.realpath(sys.executable))),
        'numpy_tree': tree_sha(pathlib.Path(np.__file__).parent),
        'chrome_wrapper': sha256_file(chrome) if chrome.exists() else 'unavailable',
        'chrome_binary': sha256_file('/opt/google/chrome/chrome') if os.path.exists('/opt/google/chrome/chrome') else 'unavailable',
        'playwright_tree': 'unavailable', 'playwright_core_tree': 'unavailable',
        'xr_js': sha256_file(ROOT / 'assets/references/xr_reference/analysis-runtime/standalone/xr.js'),
        'xr_slam_js': sha256_file(ROOT / 'assets/references/xr_reference/analysis-runtime/standalone/xr-slam.js'),
        'run_reference_it_mjs': sha256_file(RUNNER_IT),
        'page_runtime_js': sha256_file(ROOT / 'slam_refact/tools/re/runtime/page-runtime.js'),
        'image_target_hook_js': sha256_file(ROOT / 'slam_refact/tools/re/runtime/image-target-hook.js'),
        'scenarios_json': sha256_file(ROOT / 'slam_refact/tools/re/runtime/scenarios.json'),
        'tool_or4_world_regression_py': sha256_file(pathlib.Path(__file__).resolve()),
    }
    for key, sub in (('playwright_tree', 'playwright'), ('playwright_core_tree', 'playwright-core')):
        d = ROOT / 'scripts/dev/node_modules' / sub
        if d.is_dir():
            pins[key] = tree_sha(d)
    return pins


D_TRACES_EXPECTED = len(BUNDLES) * (len(D_RUNS) + 1)   # 7 noise runs + the s0 repeat per bundle


def live_inputs():
    ins = {f'bundle:{key}': bundle_sha(path) for key, path, _, _ in BUNDLES}
    for f in sorted(TARGET_DIR.glob('*')):
        ins[f'target:{f.name}'] = sha256_file(f)
    if NOISE_TABLE.exists():
        ins['noise_table'] = sha256_file(NOISE_TABLE)
    if (OR4V / 'null/null-report.json').exists():
        ins['null_report'] = sha256_file(OR4V / 'null/null-report.json')
    if (OR4V3 / 'null/N').is_dir():
        ins['n_arm_traces'] = trace_list_sha(OR4V3 / 'null/N')[0]
    dl, dn = trace_list_sha(*d_roots())
    if dn == D_TRACES_EXPECTED:
        ins['d_arm_traces'] = dl
    return ins


def parse_time(text):
    try:
        return datetime.datetime.fromisoformat(text)
    except Exception:  # noqa: BLE001
        return None


def held_check(manifest_path, designation_path, env, gate_md, pins, inputs, marker_kind, marker_dir, create_marker=True,
               require_marker_of=(), now=None):
    """Every condition is checked before any engine answer or browser exists; returns (ok, messages).
    An accident guard, not an authentication (protocol 12.6): someone who can write the designation file, the manifest
    and the environment can pass it. It stops: unregistered binaries / bundles / targets, files changed after the
    designation, a manifest edited after the designation (manifest_core_sha256), a review recorded before the registration,
    a second first run of the same kind. create_marker=False is the analysis guard: the same conditions, no new marker, and
    require_marker_of lists the first-run kinds that must already have started."""
    msgs = []
    try:
        m = json.loads(pathlib.Path(manifest_path).read_text())
    except Exception as exc:  # noqa: BLE001
        return False, [f'HELD (manifest unreadable: {exc})']
    review = m.get('review') or {}
    envv = env.get('OR4_DESIGNATION_SHA256')
    dp = pathlib.Path(designation_path)
    if not dp.exists():
        msgs.append('designation file missing')
    else:
        actual = sha256_file(dp)
        if not envv or envv != actual:
            msgs.append('(a) OR4_DESIGNATION_SHA256 is not the designation file sha256')
        if envv != review.get('designation_file_sha256') or actual != review.get('designation_file_sha256'):
            msgs.append('(a) manifest review.designation_file_sha256 does not match the designation file / env')
        try:
            d = json.loads(dp.read_text())
        except Exception:  # noqa: BLE001
            d = {}
            msgs.append('(c) designation file is not JSON')
        if d.get('gate_id') != m.get('gate_id') or d.get('first_real_run_permitted') is not True:
            msgs.append('(c) designation file lacks the gate id or first_real_run_permitted: true')
        if d.get('gate_sha256') != m.get('sha256'):
            msgs.append('(c) designation gate_sha256 differs from the manifest sha256')
        if d.get('manifest_core_sha256') != manifest_core_sha(m):
            msgs.append('(c) designation manifest_core_sha256 differs from the manifest (edited after the designation?)')
        for key, live in (('noise_table_sha256', inputs.get('noise_table')), ('n_arm_trace_list_sha256', inputs.get('n_arm_traces')),
                          ('d_arm_trace_list_sha256', inputs.get('d_arm_traces'))):
            if not live or d.get(key) != live:
                msgs.append(f'(c) designation {key} differs from the live file')
    if not review.get('at'):
        msgs.append('(b) manifest review.at is not recorded')
    else:
        at, created = parse_time(review['at']), parse_time(m.get('created_at') or '')
        current = now or datetime.datetime.now(datetime.timezone.utc)
        if at is None or created is None or at.tzinfo is None or created.tzinfo is None:
            msgs.append('(b) review.at / created_at are not timezone-aware ISO times')
        elif not created < at <= current:
            msgs.append('(b) review.at is not after created_at (or lies in the future)')
        else:
            fin = parse_time(m.get('finalized_at') or '')
            if fin is None or fin.tzinfo is None or not fin < at:
                msgs.append('(b) review.at is not after finalized_at (the evidence fields were filled after the review)')
    if m.get('first_real_run_at') is not None:
        msgs.append('manifest first_real_run_at is already set')
    if sha256_file(gate_md) != m.get('sha256'):
        msgs.append('(d) gate file sha256 differs from the manifest')
    for k, v in (m.get('pins') or {}).items():
        if v == 'unavailable' or pins.get(k) != v:
            msgs.append(f'(e) pin {k}: live {pins.get(k)} != registered {v}')
    for k in pins:
        if k not in (m.get('pins') or {}):
            msgs.append(f'(e) live pin {k} is not registered in the manifest')
    for k, v in (m.get('input_pins') or {}).items():
        if inputs.get(k) != v:
            msgs.append(f'(f) input {k}: live {inputs.get(k)} != registered {v}')
    for k in inputs:
        if k not in (m.get('input_pins') or {}):
            msgs.append(f'(f) live input {k} is not registered in the manifest')
    for kind in require_marker_of:
        if not (pathlib.Path(marker_dir) / f'{GATE_ID}.v{VERSION}.first-run.{kind}.marker').exists():
            msgs.append(f'(g) the {kind} first run has not started under the hold (no marker)')
    if not msgs and create_marker:
        marker = pathlib.Path(marker_dir) / f'{GATE_ID}.v{VERSION}.first-run.{marker_kind}.marker'
        try:
            fd = os.open(marker, os.O_CREAT | os.O_EXCL | os.O_WRONLY, 0o644)
            os.write(fd, json.dumps({'created': time.strftime('%Y-%m-%dT%H:%M:%S%z'), 'kind': marker_kind}).encode())
            os.close(fd)
        except FileExistsError:
            msgs.append(f'(g) first-run marker for {marker_kind} already exists (a second first run is HELD)')
    return not msgs, (['HELD'] + msgs) if msgs else ['held conditions satisfied (a)-(g)']


def write_once(path, obj):
    """Result files are written with O_EXCL: an existing file is never replaced (gate section 10)."""
    with open(path, 'x') as f:
        f.write(json.dumps(obj, indent=1, sort_keys=True) + '\n')


def ledger_path(kind):
    return PREREG / f'{GATE_ID}.v{VERSION}.{kind}-report.ledger.json'


# ------------------------------------------------------------------------------------------------- engine runs
def run_engine(binary, key, bundle_dir, out, seed=None, targets=None, mutation=None, rng_seed=None, force_orb=False,
               tolerate_failure=False):
    out = pathlib.Path(out)
    out.parent.mkdir(parents=True, exist_ok=True)
    cmd = [str(binary), '--bundle', str(ROOT / bundle_dir), '--out', str(out)]
    if seed is not None:
        cmd += ['--id-seed', str(seed)]
    if rng_seed is not None:
        cmd += ['--rng-seed', str(rng_seed)]
    if force_orb:
        cmd += ['--force-orb-gorb', '1']
    if targets:
        cmd += ['--targets', str(targets)]
    if mutation:
        cmd += ['--mutation', mutation]
    t0 = time.time()
    timed_out = False
    with open(str(out) + '.log', 'xb') as log:
        proc = subprocess.Popen(cmd, cwd=ROOT, stdout=log, stderr=subprocess.STDOUT)
        while True:     # wait4 gives this child's own peak resident memory
            pid, wstatus, ru = os.wait4(proc.pid, os.WNOHANG)
            if pid:
                break
            if time.time() - t0 > RUN_TIMEOUT_S and not timed_out:
                timed_out = True
                proc.kill()
            time.sleep(0.5)
    rc = os.waitstatus_to_exitcode(wstatus)
    text = pathlib.Path(str(out) + '.log').read_text(errors='replace')
    status = {'key': key, 'cmd': cmd, 'rc': rc, 'timed_out': timed_out, 'maxrss_kb': ru.ru_maxrss,
              'class': classify_exit(rc, text, timed_out=timed_out), 'seconds': round(time.time() - t0, 1), 'log_tail': text[-600:]}
    for fname in ('trace.jsonl', 'meta.json'):
        f = out / fname
        status[fname.split('.')[0] + '_sha256'] = sha256_file(f) if f.exists() else None
    with open(str(out) + '.status.json', 'x') as f:
        f.write(json.dumps(status, indent=1) + '\n')
    with open(str(out) + '.cmd.txt', 'x') as f:
        f.write(' '.join(cmd) + f'\nrc {rc}\nseconds {status["seconds"]}\nmaxrss_kb {status["maxrss_kb"]}\n')
    if rc != 0 and not tolerate_failure:
        raise SystemExit(f'{key}: {" ".join(cmd)} failed rc {rc}')
    return status


def run_many(jobs, workers=MAX_WORKERS):
    workers = max(1, min(workers, MAX_WORKERS))
    with concurrent.futures.ThreadPoolExecutor(max_workers=workers) as ex:
        futs = [ex.submit(run_engine, *j[0], **j[1]) for j in jobs]
        return [f.result() for f in concurrent.futures.as_completed(futs)]


def source_state_files():
    """Engine source tree the binaries are built from (src, include, third_party of the engine, the replay harness, the
    OR-4 main): relative path -> sha256."""
    files = {}
    roots = [ROOT / 'slam_refact/engine/src', ROOT / 'slam_refact/engine/include', ROOT / 'slam_refact/engine/third_party',
             ROOT / 'slam_refact/tools/harness/replay', ROOT / 'slam_refact/engine/tests/image_target/or4_replay_main.cc',
             ROOT / 'slam_refact/engine/CMakeLists.txt']
    for r in roots:
        for f in ([r] if r.is_file() else sorted(r.rglob('*'))):
            if f.is_file() and '__pycache__' not in f.parts:
                files[str(f.relative_to(ROOT))] = sha256_file(f)
    return files


def tree_divergence():
    """Files whose sha256 differs from the engine source state recorded at pin time (v3): changed, added, removed."""
    rec = json.loads(SOURCE_STATE.read_text())['sha256']
    live = source_state_files()
    return {'changed': sorted(k for k in rec if k in live and rec[k] != live[k]), 'added': sorted(k for k in live if k not in rec),
            'removed': sorted(k for k in rec if k not in live)}


def cmd_pin_binary(_):
    raise SystemExit('pin-binary was run for v3 (or4/v3/bin, same binaries); it is not repeated')


def cmd_null_engine(_):
    raise SystemExit('null-engine was run for v3 (or4/v3/null/N, same binary); it is not repeated')


def cmd_decomp_engine(a):
    """D arm: allOrb / allGorb forced, no target. Run names: 'force' (= id seed s0, the v3 run, reused), s1..s3 (id seeds),
    r1..r3 (id seed s0, RNG seeds), 'force-repeat' (s0 again, determinism)."""
    jobs = []
    for key, path, _, _ in BUNDLES:
        s = seeds_of(bundle_seed(path))
        for k in (1, 2, 3):
            if not d_dir(key, f's{k}').is_dir():
                jobs.append(((ENGINE_PIN, key, path, OR4V5 / 'decomp/D' / key / f's{k}'), {'seed': s[k], 'force_orb': True}))
        for k, rs in enumerate(rng_seeds(), start=1):
            if not d_dir(key, f'r{k}').is_dir():
                jobs.append(((ENGINE_PIN, key, path, OR4V5 / 'decomp/D' / key / f'r{k}'), {'seed': s[0], 'rng_seed': rs, 'force_orb': True}))
        if not d_dir(key, 'force-repeat').is_dir():
            jobs.append(((ENGINE_PIN, key, path, OR4V5 / 'decomp/D' / key / 'force-repeat'), {'seed': s[0], 'force_orb': True}))
    run_many(jobs, a.workers)


def read_arm(key_to_dir, names):
    return {key: [load_trace(key_to_dir(key, n) / 'trace.jsonl') for n in names] for key, _, _, _ in BUNDLES}


def read_n():
    return read_arm(lambda key, n: OR4V3 / 'null/N' / key / n, NOISE_RUNS)


def read_d():
    return read_arm(d_dir, D_RUNS)


def mutation_bundles(d_noise):
    """The first N_MUTATION_BUNDLES bundles in BUNDLES order whose D-arm noise is zero on every metric (registered rule)."""
    zero = [k for k, _, _, _ in BUNDLES if all((d_noise[k][m] or 0.0) <= 1e-9 for m in METRICS)]
    if len(zero) < N_MUTATION_BUNDLES:
        raise SystemExit(f'only {len(zero)} bundles with zero D-arm noise: the registered rule cannot be met (v6)')
    return zero[:N_MUTATION_BUNDLES]


def cmd_mutation_engine(a):
    d = read_d()
    mb = mutation_bundles(noise_table(d))
    jobs = []
    for key, path, _, _ in BUNDLES:
        if key not in mb:
            continue
        s0 = seeds_of(bundle_seed(path))[0]
        for mut in ENGINE_MUTATIONS:
            out = OR4V5 / 'mutation' / ('D_' + mut.replace(':', '_')) / key
            if not out.exists():
                jobs.append(((ENGINE_PIN, key, path, out), {'seed': s0, 'mutation': mut, 'force_orb': True}))
    run_many(jobs, a.workers)


def acc_of(scenario, run_dir):
    sys.path.insert(0, str(EVAL_DIR))
    import accuracy_matrix  # noqa: WPS433
    scenarios = json.loads((ROOT / 'slam_refact/tools/re/runtime/scenarios.json').read_text())
    r = accuracy_matrix.evaluate(scenario, str(run_dir), scenarios)
    acc = r.get('accuracy') or {}
    return {'se3AteRmse': acc.get('se3AteRmse'), 'apeRotDegRmse': acc.get('apeRotDegRmse'), 'coverage': acc.get('coverage'),
            'sim3AteRmse': acc.get('sim3AteRmse'), 'sim3Scale': acc.get('sim3Scale')}


def loo_false_fail(d_runs, bundles, maxima):
    """Estimate of the false-FAIL rate of the GATE under the null (target code only moves the RNG stream), for the given bundles:
    each of the D arm's runs in turn plays a new realisation: the tolerance is rebuilt from the other runs and the worst pair metric
    of the left-out run against every other run is judged with the same rules (vacuous excluded, G-COV cap). INFERRED, pessimistic
    (the real comparison is a same-seed pair)."""
    out = {}
    for b in bundles:
        runs = d_runs[b]
        failed = []
        for j in range(len(runs)):
            rest = [r for i, r in enumerate(runs) if i != j]
            noise = noise_table({b: rest})[b]
            worst = {m: 0.0 for m in METRICS}
            loss = 0.0
            for i, other in enumerate(runs):
                if i == j:
                    continue
                pm = pair_metrics(other, runs[j])
                for m in METRICS:
                    worst[m] = max(worst[m], pm[m] if pm[m] is not None else float('inf'))
                loss = max(loss, (pm['covX'] - pm['covY']) / pm['rows'])
            limit, vac = cov_cap_limit(noise, len(runs[0]), maxima[b])
            bad = [m for m in METRICS if not vacuous(noise, m, maxima[b]) and worst[m] > tolerance(noise, m)]
            if bad or ((not vac) and loss > limit):
                failed.append(j)
        out[b] = {'cases': len(runs), 'false_fail_cases': failed, 'rate': len(failed) / len(runs)}
    return out


def cmd_analyze_null(a):
    n, d = read_n(), read_d()
    root = OR4V3 / 'null'
    report = {'schema': 'or4-null-report-v6', 'created': time.strftime('%Y-%m-%dT%H:%M:%S%z'),
              'binary_sha256': sha256_file(ENGINE_PIN), 'baseline_binary_sha256': sha256_file(BASELINE_PIN),
              'tool_sha256': sha256_file(pathlib.Path(__file__).resolve())}
    det, harness, det_d = {}, {}, {}
    for key, _, _, _ in BUNDLES:
        det[key] = identical(n[key][0], load_trace(root / 'N' / key / 's0-repeat' / 'trace.jsonl'))
        harness[key] = identical(n[key][0], load_trace(root / 'H' / key / 's0' / 'trace.jsonl'))
        det_d[key] = identical(d[key][0], load_trace(d_dir(key, 'force-repeat') / 'trace.jsonl'))
    report['G-DET-N'] = {k: {'identical': v[0], 'first_diff': v[1]} for k, v in det.items()}
    report['G-H'] = {k: {'identical': v[0], 'first_diff': v[1]} for k, v in harness.items()}
    report['G-DET-D'] = {k: {'identical': v[0], 'first_diff': v[1]} for k, v in det_d.items()}
    nr4 = {}
    for key, _, _, _ in BUNDLES:   # NR-4: identical traces through the metrics (the measurement floor)
        for label, other in (('s0-repeat', load_trace(root / 'N' / key / 's0-repeat' / 'trace.jsonl')),
                             ('H', load_trace(root / 'H' / key / 's0' / 'trace.jsonl')),
                             ('D-force-repeat', load_trace(d_dir(key, 'force-repeat') / 'trace.jsonl'))):
            ref = d[key][0] if label.startswith('D-') else n[key][0]
            pm = pair_metrics(ref, other)
            nr4[f'{key}/{label}'] = {m: pm[m] for m in METRICS}
    report['NR-4_same_trace_metrics'] = nr4
    report['NR-4_max_scale_noise'] = max(v['scale'] for v in nr4.values())
    r_n, r_d = noise_table(n), noise_table(d)
    report['R_N'], report['R_D'] = r_n, r_d
    report['noise_by_source_D'] = {'id_seeds': {k: noise_table({k: v[:4]})[k] for k, v in d.items()},
                                   'engine_rng_seeds': {k: noise_table({k: [v[0]] + v[4:]})[k] for k, v in d.items()}}
    report['noise_runs_change_trace_D'] = {k: [not identical(v[0], v[j])[0] for j in range(1, 7)] for k, v in d.items()}
    maxima = {k: metric_maxima(d[k][0]) for k in d}
    report['metric_maxima'] = maxima
    report['band_floor_table'] = {b: {m: tolerance(r_d[b], m) for m in METRICS} for b in r_d}
    report['power_table'] = {b: power_entry(r_d[b], maxima[b]) for b in r_d}
    report['powered_bundles'] = sorted(b for b, v in report['power_table'].items() if v['powered'])
    report['cov_cap_table'] = {b: dict(zip(('limit', 'vacuous'), cov_cap_limit(r_d[b], len(d[b][0]), maxima[b]))) for b in r_d}
    loo = loo_false_fail(d, report['powered_bundles'], maxima)
    report['loo_false_fail_powered'] = loo
    report['loo_false_fail_rate_powered'] = sum(len(v['false_fail_cases']) for v in loo.values()) / max(1, sum(v['cases'] for v in loo.values()))
    report['loo_bundle_fail_probability_any_powered'] = 1.0 - math.prod(1.0 - v['rate'] for v in loo.values())
    # G-EXERCISED mutation: the positive indicator must FAIL where the target path is off (the N and D arms, real runs).
    neg = {}
    for key, _, _, _ in BUNDLES:
        for label, mp in (('N-s0', OR4V3 / 'null/N' / key / 's0' / 'meta.json'), ('D-force', d_dir(key, 'force') / 'meta.json')):
            meta = json.loads(mp.read_text())
            neg[f'{key}/{label}'] = {'targetsLoaded': meta.get('targetsLoaded'), 'framesWithNextRois': meta.get('framesWithNextRois'),
                                     'exercised_positive': exercised_positive(meta)}
    report['exercised_negative_control'] = neg
    report['exercised_negative_control_ok'] = not any(v['exercised_positive'] for v in neg.values())
    # Inherited effect of the settings change (REPORT): D(b, s_k) against N(b, s_k) and the first-run TN expectation check.
    inh = {}
    for key, _, _, _ in BUNDLES:
        eff = effect_table({key: [n[key][k] for k in range(4)]}, {key: [d[key][k] for k in range(4)]})[key]
        inh[key] = {'effect': {m: eff[m] for m in METRICS}, 'exceeds_N_tolerance': [m for m in METRICS if eff[m] is None or eff[m] > tolerance(r_n[key], m)]}
    report['D_vs_N_inherited'] = inh
    report['expected_TN_exceed_declared'] = list(EXPECTED_TN_EXCEED)
    report['expected_TN_exceed_consistent_with_D_vs_N'] = sorted(k for k, v in inh.items() if v['exceeds_N_tolerance']) == sorted(EXPECTED_TN_EXCEED)
    mb = mutation_bundles(r_d)
    report['mutation_bundles'] = mb
    mut = {}
    for key in mb:
        for m in ENGINE_MUTATIONS:
            rows = load_trace(OR4V5 / 'mutation' / ('D_' + m.replace(':', '_')) / key / 'trace.jsonl')
            det_ok, fails, pm, cap = mutation_detected(r_d[key], d[key][0], rows)
            mut[f'{m}/{key}'] = {'detected': det_ok, 'cov_cap_exceeded': cap, 'failing_metrics': fails, 'metrics': {k: pm[k] for k in METRICS}, 'required': True, 'ok': det_ok}
    for key, _, _, _ in BUNDLES:   # trace mutants on every bundle: detected iff the mutant exceeds the bundle's tolerance
        for name in TRACE_MUTANTS:
            det_ok, fails, pm, cap = mutation_detected(r_d[key], d[key][0], trace_mutant(name, d[key][0]))
            expected, value, tol = mutant_expectation(r_d[key], name, pm)
            limit, vac = cov_cap_limit(r_d[key], pm['rows'], maxima[key])
            cap_expected = (not vac) and pm['rows'] > 0 and pm['cov'] / pm['rows'] > limit and name.startswith('TM-cov')
            metric, analytic = TRACE_MUTANTS[name]
            exact = analytic is None or value is None or abs(value - analytic) < 1e-6
            mut[f'{name}/{key}'] = {'detected': det_ok, 'failing_metrics': fails, 'cov_cap_exceeded': cap,
                                    'cov_cap_expected': cap_expected, 'metrics': {k: pm[k] for k in METRICS},
                                    'required': True, 'expected_detected': expected or cap_expected, 'metric': metric, 'value': value,
                                    'tolerance': tol, 'analytic_value_matches': exact,
                                    'ok': ((metric in fails) == expected) and (cap == cap_expected) and exact}
    report['mutations'] = mut
    report['scale_blind_bundles'] = {name: [b for b in r_d if tolerance(r_d[b], 'scale') >= TRACE_MUTANTS[name][1]]
                                     for name in ('TM-scale-1.1', 'TM-scale-1.2', 'TM-scale-1.5', 'TM-scale-0.9')}
    acc = {}
    for key in ('tum-room1-full-absolute__pma1', 'tum-room1-full-absolute__pma2'):
        acc[key] = [acc_of(key.split('__')[0], OR4V3 / 'null/N' / key / nm) for nm in NOISE_RUNS]
    report['acc_noise_report'] = acc
    report['G-DET-N-pass'] = all(v[0] for v in det.values())
    report['G-H-pass'] = all(v[0] for v in harness.values())
    report['G-DET-D-pass'] = all(v[0] for v in det_d.values())
    report['mutations-required-ok'] = all(v['ok'] for v in mut.values() if v['required']) and report['exercised_negative_control_ok']
    out = OR4V / 'null/null-report.json'
    out.parent.mkdir(parents=True, exist_ok=True)
    write_once(out, report)
    print(json.dumps({k: report[k] for k in ('G-DET-N-pass', 'G-H-pass', 'G-DET-D-pass', 'mutations-required-ok')}), out)
    if not NOISE_TABLE.exists():
        write_once(NOISE_TABLE, {'schema': 'or4-noise-table-v6', 'gate': f'{GATE_ID}.v{VERSION}', 'source': str(out.relative_to(ROOT)),
                                 'binary_sha256': report['binary_sha256'], 'noise_runs': NOISE_RUNS, 'D_runs': D_RUNS,
                                 'R_N': r_n, 'R_D': r_d, 'metric_maxima': maxima, 'floor': FLOOR, 'noise_factor': NOISE_FACTOR})


def cmd_verify_held(_):
    (OR4 / 'dryrun-markers').mkdir(exist_ok=True)
    ok, msgs = held_check(MANIFEST, DESIGNATION, os.environ, GATE_MD, live_pins(), live_inputs(), 'dryrun',
                          OR4 / 'dryrun-markers', create_marker=False)
    print('\n'.join(msgs))
    return 0 if ok else 1


def require_held(kind):
    ok, msgs = held_check(MANIFEST, DESIGNATION, os.environ, GATE_MD, live_pins(), live_inputs(), kind, PREREG)
    print('\n'.join(msgs))
    if not ok:
        raise SystemExit(3)


def analysis_guard(kinds):
    """analyze-real / analyze-original: the same conditions as the first run (tool sha = pin, noise table, N and D arm traces,
    manifest core), without a new marker, and the first-run markers of kinds must exist."""
    ok, msgs = held_check(MANIFEST, DESIGNATION, os.environ, GATE_MD, live_pins(), live_inputs(), 'analysis', PREREG,
                          create_marker=False, require_marker_of=kinds)
    print('\n'.join(msgs))
    if not ok:
        raise SystemExit(3)
    return {'tool_sha256': sha256_file(pathlib.Path(__file__).resolve()), 'noise_table_sha256': sha256_file(NOISE_TABLE),
            'n_arm_trace_list_sha256': trace_list_sha(OR4V3 / 'null/N')[0], 'd_arm_trace_list_sha256': trace_list_sha(*d_roots())[0],
            'manifest_core_sha256': manifest_core_sha(json.loads(MANIFEST.read_text())),
            'designation_file_sha256': sha256_file(DESIGNATION)}


def cmd_real_engine(a):
    require_held('engine')
    root = OR4V / 'real'
    jobs = []
    for key, path, _, _ in BUNDLES:
        s = seeds_of(bundle_seed(path))
        for k, seed in enumerate(s):
            jobs.append(((ENGINE_PIN, key, path, root / 'T' / key / f's{k}'), {'seed': seed, 'targets': TARGET_DIR, 'tolerate_failure': True}))
        jobs.append(((ENGINE_PIN, key, path, root / 'T' / key / 's0-repeat'), {'seed': s[0], 'targets': TARGET_DIR, 'tolerate_failure': True}))
    statuses = run_many(jobs, a.workers)
    write_once(root / 'run-status.json', {'schema': 'or4-run-status-v6', 'runs': sorted(statuses, key=lambda s: s['cmd'][4])})
    write_once(ledger_path('real-engine'), {'run_status': str((root / 'run-status.json').relative_to(ROOT)), 'run_status_sha256': sha256_file(root / 'run-status.json'),
                                           'created': time.strftime('%Y-%m-%dT%H:%M:%S%z')})
    bad = [s for s in statuses if s['class'] != 'OK']
    print(f'{len(statuses)} runs, {len(bad)} not OK:', [(s['key'], s['class'], s['rc']) for s in bad])


def cmd_real_original(a):
    require_held('original')
    root = OR4V / 'original'
    order = []
    for k in range(1, ORIGINAL_RUNS + 1):          # interleaved: same machine state for both arms
        for scen in ORIGINAL_SCENARIOS:
            order.append((scen, f'or4n{k}', None))
            order.append((scen, f'or4t{k}', TARGET_DIR))
    for scen, label, targets in order:
        cmd = ['node', str(RUNNER_IT.relative_to(ROOT)), '--scenario', scen, '--label', label, '--out-root', str(root.relative_to(ROOT))]
        if targets:
            cmd += ['--image-targets', str(targets.relative_to(ROOT))]
        print(' '.join(cmd), flush=True)
        try:
            rc = subprocess.run(cmd, cwd=ROOT, timeout=RUN_TIMEOUT_S).returncode
        except subprocess.TimeoutExpired:
            rc = 'timeout'
        if rc != 0:
            print(f'RUN FAILED rc {rc}: {scen} {label} (recorded; the run is not repeated by this tool)', flush=True)


def acc_report(nroot, root):
    """REPORT: TUM ATE ratios of the metric bundles. With Sim(3) scale about 0.024 the SE(3) ATE is dominated by the size of the
    estimated trajectory; the ratio is a size ratio. None-safe."""
    out = {}
    for key in ('tum-room1-full-absolute__pma1', 'tum-room1-full-absolute__pma2'):
        scen = key.split('__')[0]
        row = {}
        for k in range(4):
            try:
                t, n = acc_of(scen, root / 'T' / key / f's{k}'), acc_of(scen, nroot / key / f's{k}')
                row[f's{k}'] = {m: (t[m] / n[m] if t[m] is not None and n[m] else None) for m in ('se3AteRmse', 'apeRotDegRmse')}
            except Exception as exc:  # noqa: BLE001
                row[f's{k}'] = {'error': str(exc)}
        out[key] = row
    return out


def verify_run_outputs(root, statuses):
    """Every T trace / meta file must still have the sha256 recorded by real-engine (nothing replaced between the run and the
    analysis). Returns the list of mismatches."""
    bad = []
    for s in statuses:
        out = pathlib.Path(s['cmd'][4])
        for fname, field in (('trace.jsonl', 'trace_sha256'), ('meta.json', 'meta_sha256')):
            if s.get(field) is not None and (not (out / fname).exists() or sha256_file(out / fname) != s[field]):
                bad.append(f'{out.name}/{fname}')
    return bad


def d_baseline_maxrss(key):
    peaks = []
    for n in D_RUNS + ['force-repeat']:
        f = pathlib.Path(str(d_dir(key, n)) + '.status.json')
        if f.exists():
            v = json.loads(f.read_text()).get('maxrss_kb')
            if v:
                peaks.append(v)
    return max(peaks) if peaks else None


def cmd_analyze_real(a):
    guard = analysis_guard(('engine',))
    root = OR4V / 'real'
    out, ledger = root / 'real-report.json', ledger_path('real')
    if out.exists() or ledger.exists():
        raise SystemExit(f'{out} or {ledger} exists: a result is never replaced or re-written')
    led = json.loads(ledger_path('real-engine').read_text())
    if led['run_status_sha256'] != sha256_file(root / 'run-status.json'):
        raise SystemExit('run-status.json differs from the real-engine ledger')
    tables = json.loads(NOISE_TABLE.read_text())
    r_n, r_d, maxima = tables['R_N'], tables['R_D'], tables['metric_maxima']
    nullrep = json.loads((OR4V / 'null/null-report.json').read_text())
    runs = json.loads((root / 'run-status.json').read_text())['runs']
    mismatch = verify_run_outputs(root, runs)
    if mismatch:
        raise SystemExit(f'T output files changed after real-engine: {mismatch}')
    status = {(s['key'], s['cmd'][4].rsplit('/', 1)[-1]): s for s in runs}
    n, d = read_n(), read_d()
    names = ['s0', 's1', 's2', 's3']
    report = {'schema': 'or4-real-report-v6', 'created': time.strftime('%Y-%m-%dT%H:%M:%S%z'), 'guard': guard,
              'band': {}, 'report_T_vs_N': {}, 'G-RUN': {}, 'G-DET-T': {}, 'G-EXERCISED': {}, 'power': {}, 'observed_effect_T_vs_D': {}}
    for key, _, cls, _ in BUNDLES:
        base = d_baseline_maxrss(key)
        classes = {}
        for nm in names + ['s0-repeat']:
            s = status.get((key, nm))
            classes[nm] = (s, classify_exit(s['rc'], s.get('log_tail', ''), s.get('maxrss_kb'), base, s.get('timed_out', False))) if s else (None, 'CRASH')
        bad = [(nm, c, (s or {}).get('rc'), (s or {}).get('maxrss_kb')) for nm, (s, c) in classes.items() if c != 'OK']
        report['G-RUN'][key] = {'ok': not bad, 'not_ok': bad, 'infra': [b for b in bad if b[1] == 'INFRA'], 'd_baseline_maxrss_kb': base,
                                'maxrss_kb': {nm: (s or {}).get('maxrss_kb') for nm, (s, _) in classes.items()}}
        report['power'][key] = dict(power_entry(r_d[key], maxima[key]), cls=cls)
        if bad:
            report['band'][key] = {'pass': False, 'reason': f'T run(s) not OK: {bad}'}
            report['G-DET-T'][key] = {'identical': False, 'first_diff': None}
            report['G-EXERCISED'][key] = {'exercised': False, 'class': cls}
            continue
        t = [load_trace(root / 'T' / key / nm / 'trace.jsonl') for nm in names]
        dd = {key: [d[key][i] for i in range(4)]}
        report['band'][key] = band_verdict({key: r_d[key]}, effect_table(dd, {key: t}), {key: maxima[key]})[key]
        nmax = {key: metric_maxima(n[key][0])}
        tn = band_verdict({key: r_n[key]}, effect_table({key: n[key][:4]}, {key: t}), nmax)[key]
        report['report_T_vs_N'][key] = {'verdict': tn, 'declared_expected_exceed': key in EXPECTED_TN_EXCEED}
        report['observed_effect_T_vs_D'][key] = observed_effect(dd[key], t)
        same, first = identical(t[0], load_trace(root / 'T' / key / 's0-repeat' / 'trace.jsonl'))
        report['G-DET-T'][key] = {'identical': same, 'first_diff': first}
        metas = [json.loads((root / 'T' / key / nm / 'meta.json').read_text()) for nm in names]
        report['G-EXERCISED'][key] = {'exercised': all(exercised_positive(m) for m in metas), 'class': cls,
                                      'targetsLoaded': [m['targetsLoaded'] for m in metas],
                                      'framesWithNextRois': [m['framesWithNextRois'] for m in metas],
                                      'paddedAtlases': metas[0]['paddedAtlases'], 'detectedFrames': [m['framesWithDetectedImages'] for m in metas]}
    report['REPORT_acc_ratio'] = acc_report(OR4V3 / 'null/N', root)
    powered = [(b, report['power'][b]['cls'], report['band'][b]['pass']) for b in report['power'] if report['power'][b]['powered']]
    null_ok = nullrep['G-DET-N-pass'] and nullrep['G-H-pass'] and nullrep['G-DET-D-pass']
    infra = [b for v in report['G-RUN'].values() for b in v['infra']]
    decision, code, reasons = us014_decision(null_ok, all(v['ok'] for v in report['G-RUN'].values()), infra,
                                             all(v['identical'] for v in report['G-DET-T'].values()),
                                             all(v['exercised'] for v in report['G-EXERCISED'].values()), powered)
    nonpowered_fail = sorted(b for b in report['band'] if not report['power'][b]['powered'] and not report['band'][b]['pass'])
    report['summary'] = {'R-US014': decision, 'code': code, 'reasons': reasons,
                         'powered_bundles': len(powered), 'powered_pass': len([1 for _, _, ok in powered if ok]), 'bundles': len(BUNDLES),
                         'nonpowered_band_fail_REPORT': nonpowered_fail, 'infra_runs': infra,
                         'G-RUN': all(v['ok'] for v in report['G-RUN'].values()),
                         'G-DET-T': all(v['identical'] for v in report['G-DET-T'].values()),
                         'G-EXERCISED': all(v['exercised'] for v in report['G-EXERCISED'].values()), 'null_premises': null_ok}
    write_once(out, report)
    write_once(ledger, {'report': str(out.relative_to(ROOT)), 'report_sha256': sha256_file(out), 'created': report['created'],
                        'run_status_sha256': sha256_file(root / 'run-status.json'), 'guard': guard})
    print(json.dumps(report['summary']), out)


def orig_pairs(root, scenario, labels_a, labels_b=None):
    """Pair metrics between original runs: within one label list (a < b) or across two lists (all combinations).
    A run without a trace (failed run) is recorded as such, not fatal."""
    traces = {}
    for l in set(labels_a) | set(labels_b or []):
        p = pathlib.Path(root) / scenario / l / 'trace.jsonl'
        traces[l] = load_trace(p) if p.exists() else None
    pairs = ([(x, y) for i, x in enumerate(labels_a) for y in labels_a[i + 1:]] if labels_b is None
             else [(x, y) for x in labels_a for y in labels_b])
    return {f'{x}~{y}': (pair_metrics(traces[x], traces[y]) if traces[x] and traces[y] else {'error': 'trace missing'}) for x, y in pairs}


def cmd_original_null(_):
    raise SystemExit('original-null was run for v3 (or4/v3/null/original-null.json, tracked-row metrics changed in v5: rerun into or4/v5 only if needed)')


def cmd_analyze_original(_):
    """REPORT (n = 3 per arm, protocol rule 8): within-arm and cross-arm pair metrics of the original runs next to the engine's
    tables: R_N, the inherited effect D vs N, and the T vs D effect."""
    analysis_guard(('engine', 'original'))
    root = OR4V / 'original'
    path, ledger = root / 'original-compare.json', ledger_path('original')
    if path.exists() or ledger.exists():
        raise SystemExit(f'{path} or {ledger} exists: a result is never replaced or re-written')
    tables = json.loads(NOISE_TABLE.read_text())
    if json.loads(ledger_path('real').read_text())['report_sha256'] != sha256_file(OR4V / 'real/real-report.json'):
        raise SystemExit('real-report.json differs from its ledger')
    rep = json.loads((OR4V / 'real/real-report.json').read_text())
    n_labels = [f'or4n{k}' for k in range(1, ORIGINAL_RUNS + 1)]
    t_labels = [f'or4t{k}' for k in range(1, ORIGINAL_RUNS + 1)]
    out = {'schema': 'or4-original-compare-v5', 'created': time.strftime('%Y-%m-%dT%H:%M:%S%z'), 'class': 'REPORT'}
    for scen in ORIGINAL_SCENARIOS:
        bundles = [k for k, _, _, s in BUNDLES if s == scen]
        out[scen] = {'orig_within_no_target': orig_pairs(root, scen, n_labels), 'orig_within_target': orig_pairs(root, scen, t_labels),
                     'orig_cross': orig_pairs(root, scen, n_labels, t_labels), 'engine_R_N': {b: tables['R_N'][b] for b in bundles},
                     'engine_R_D': {b: tables['R_D'][b] for b in bundles},
                     'engine_T_vs_D_effect': {b: {m: rep['band'][b][m]['effect'] for m in METRICS} if 'scale' in rep['band'][b] else rep['band'][b] for b in bundles},
                     'engine_T_vs_N_effect': {b: {m: rep['report_T_vs_N'][b]['verdict'][m]['effect'] for m in METRICS} if b in rep['report_T_vs_N'] else None for b in bundles}}
    write_once(path, out)
    write_once(ledger, {'report': str(path.relative_to(ROOT)), 'report_sha256': sha256_file(path), 'created': out['created']})
    print(path)


PIN_SOURCES = [   # files copied to the snapshot at registration (pre-registration state, protocol section 5 "medium")
    'slam_refact/engine/tests/image_target/or4_replay_main.cc', 'slam_refact/engine/tests/image_target/CMakeLists.txt',
    'slam_refact/engine/include/slam_refact/replay_diagnostics.h', 'slam_refact/engine/src/engine/engine.cc',
    'slam_refact/engine/src/vio_tracker/vio_tracker.cc',
    'slam_refact/tools/image_target/or4_world_regression.py', 'slam_refact/tools/image_target/make_or4_target.py',
    'slam_refact/tools/image_target/tests/test_or4_world_regression.py', 'slam_refact/tools/re/runtime/run-reference-it.mjs',
    'slam_refact/tools/re/runtime/image-target-hook.js', 'slam_refact/tools/re/runtime/page-runtime.js',
    'slam_refact/tools/harness/replay/replay_main.cc', 'slam_refact/tools/harness/replay/bundle_reader.cc',
    'slam_refact/tools/harness/replay/trace_writer.cc', 'slam_refact/tools/harness/eval/accuracy_matrix.py',
    'slam_refact/tools/harness/eval/accuracy.py']


def cmd_make_manifest(_):
    """Writes the manifest at registration: gate md sha256, created_at, pins, input pins, snapshot. Refuses to overwrite,
    refuses a pin that is unavailable. Evidence fields (null-run, mutation, noise table) are filled by finalize-manifest."""
    if MANIFEST.exists():
        raise SystemExit(f'{MANIFEST} exists (never overwritten; supersede with the next version)')
    pins, inputs = live_pins(), live_inputs()
    bad = [k for k, v in {**pins, **inputs}.items() if v == 'unavailable']
    if bad:
        raise SystemExit(f'unavailable pins refused: {bad}')
    snap = PREREG / f'or4-snapshot-pre-v{VERSION}'
    snap.mkdir()
    sums = []
    for rel in PIN_SOURCES + [str(GATE_MD.relative_to(ROOT))]:
        dst = snap / rel.replace('/', '__')
        shutil.copy2(ROOT / rel, dst)
        sums.append(f'{sha256_file(dst)}  {dst.name}')
    (snap / 'SHA256SUMS').write_text('\n'.join(sums) + '\n')
    stamp = time.strftime('%Y-%m-%dT%H:%M:%S%z')
    prev = lambda v: {'file': f'build/slam-refact-full-20261019/preregistration/{GATE_ID}.v{v}.md',
                      'sha256': sha256_file(PREREG / f'{GATE_ID}.v{v}.md'),
                      'manifest_sha256': sha256_file(PREREG / f'{GATE_ID}.v{v}.manifest.json')}
    m = {
        'gate_id': GATE_ID, 'version': VERSION, 'supersedes': prev(VERSION - 1),
        'supersedes_chain': [prev(v) for v in range(1, VERSION)],
        'supersedes_note': ('v5 -> v6 before any real run, on the independent delta pre-review of v5 (REJECT, first_real_run_permitted false; '
                            'or4-it-world-regression.v5.review.md) and the coordinator decisions: G-COV cap per bundle max(1 %, 3 R_D[cov] / rows) with vacuous '
                            'exclusion (B1); R-US014 decided by the powered bundles only plus the 14-bundle premises, non-powered G-BAND is REPORT, false-FAIL rate '
                            'estimated (B2); G-EXERCISED needs the positive indicator framesWithNextRois >= 1 with an N / D arm negative control (M1); the outcome '
                            'table in section 0.2 and the tool computes PERMIT / HOLD (M2); inheritance condition by bit-identical N s0, D force, T s0 traces (M3); LOW items. '
                            'Class: pre-registration defect (B1, B2, M1-M3, L1-L8). v1-v5, their manifests, reviews, noise tables and null-reports stay unedited.'),
        'file': str(GATE_MD.relative_to(ROOT)), 'sha256': sha256_file(GATE_MD),
        'created_at': stamp[:-2] + ':' + stamp[-2:],
        'registered_by': 'OR-4 executor (image target lane)',
        'git_head': subprocess.run(['git', 'rev-parse', 'HEAD'], cwd=ROOT, capture_output=True, text=True).stdout.strip(),
        'tree_sha': None,
        'snapshot': f'{snap.relative_to(ROOT)}/ (pre-registration copies of {len(PIN_SOURCES)} tool / harness files and the gate md + SHA256SUMS, '
                    f'sha256 of SHA256SUMS: {sha256_file(snap / "SHA256SUMS")}); no write-tree (A3 not granted) -> evidence grade medium',
        'snapshot_dir': str(snap.relative_to(ROOT)),
        'pins': pins, 'input_pins': inputs,
        'first_real_run_at': None, 'review': None,
    }
    MANIFEST.write_text(json.dumps(m, indent=1) + '\n')
    print(MANIFEST, m['created_at'])


def main(argv):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    sub = ap.add_subparsers(dest='cmd', required=True)
    fns = {'pin-binary': cmd_pin_binary, 'null-engine': cmd_null_engine, 'decomp-engine': cmd_decomp_engine,
           'mutation-engine': cmd_mutation_engine, 'analyze-null': cmd_analyze_null, 'real-engine': cmd_real_engine,
           'real-original': cmd_real_original, 'analyze-real': cmd_analyze_real, 'verify-held': cmd_verify_held,
           'original-null': cmd_original_null, 'analyze-original': cmd_analyze_original, 'make-manifest': cmd_make_manifest}
    for name in fns:
        p = sub.add_parser(name)
        p.add_argument('--workers', type=int, default=MAX_WORKERS)
    a = ap.parse_args(argv)
    return fns[a.cmd](a) or 0


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))

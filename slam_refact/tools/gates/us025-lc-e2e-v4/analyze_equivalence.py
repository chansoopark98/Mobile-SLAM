#!/usr/bin/env python3
"""Analysis of the us025-lc-e2e-gate v3 parallel equivalence experiment (arms S / P2 / P4).

The rules are the ones of slam_refact/docs/design/us025-lc-v3-parallel-equivalence-prereg.md (sections 3-5), coded as written. NOTHING here is a tuning knob: every number below is
a registered number. A bug found after the experiment is fixed in a NEW file (analyze_equivalence_v2.py) and both results are reported; the rules are not changed.

    python3 -I -B analyze_equivalence.py --summary <equivalence/dev-summary.json> --sampler <sampler.jsonl> --out <equivalence/equivalence-analysis.json> [--rerun <rerun-verdicts.json>]

Input  dev-summary.json of `gate_lc.mjs dev` (runs[]: scenario, run, block, arm, lc, startedAt, endedAt, runnerSeconds, status, infra, metrics{inputFrames, noFreshCallback}, originalSuccess,
       verdicts{on, off, full}, loadAvgAtStart), infraStop.
       sampler.jsonl: one JSON object per line, written every 10 s by sample_load.py (preregistration section 7.2, prereview C-2): {"t", "dt", "baseline", "loadavg", "sys_busy_cores", "gate_pids",
       "others": [{"pid", "comm", "cores", ...}]} where `others` are the processes OUTSIDE the gate process tree with the CPU cores they used between two samples (/proc delta), or
       {"t", "end": true} / {"t", "error": ...}.
       rerun-verdicts.json (optional): {"<scenario>/<run>": {"on": ..., "off": ..., "full": ..., "originalSuccess": ...}}: the result of the single re-run of the phase B comparison of a run whose verdict was
       outside PASS / PASS_DECLARED / FAIL (G6) or whose originalSuccess is null although an ON verdict exists (R-5). The re-run itself is an operation (section 8), this file is its result.
Output equivalence-analysis.json (created with mode 'x': never overwritten). Exit 0 = analysis written (the adopted N is in the file), 2 = usage or I/O error.

Interpretations that the preregistration leaves open (fixed here, before any real data):
  * Percentiles (p10, p50, p90) are numpy's linear percentiles of the observed values; the bootstrap interval is numpy's linear 5th / 95th percentile of the 10 000 resampled statistics.
  * The bootstrap draws are numpy.random.default_rng(20261028).integers, drawn fresh for every (P arm, scenario) in this order: indices of S (B x n_S), then of P (B x n_P). The ratio of medians
    (G1) and the NO_FRESH mean difference (G3) use the SAME draws.
  * Point-estimate guards are compared as exact fractions (a difference of exactly -0.125 passes the guard "P - S >= -0.125").
  * G4 / G5: Farrington-Manning score statistic, exact unconditional p-value = maximum over the nuisance parameter grid (step 0.001) of the probability of a table with Z >= the observed Z
    (ties within 1e-9 count as equal). G5 runs the same code on the NON-INFRA proportions.
  * The full-verdict test of G6 pools both scenarios (like G4 and G5); a run without a full verdict counts as "not FAIL" in the denominator (non-INFRA, non-success runs).
  * Any "ambiguous" outcome anywhere (G3 reference, G6 residual, G7, G8, G9, R-1 leave-one-block-out, R-5, an empty S timing set) gives N = 1, as section 5 rule 2 says; INCOMPLETE (G0) gives N = 1.
  * R-1 (D-V3-8): when the adopted N is > 1, every block of a P arm is left out in turn (P2: 6, P4: 6 re-analyses of G1-G8 with the same rules); if one of them gives a SMALLER adopted N the result is ambiguous
    (N = 1; D-V3-9: a larger one is ignored). The independent unit is the block (6 per arm), not the run.
  * R-5 (D-V3-8): a run that is not INFRA, has originalSuccess null and an ON verdict is ambiguous (it would silently count as "no success"), unless the allowed single re-run of its ON comparison resolves it.
  * R-6 (v4, finding #2): a non-INFRA run without an ON verdict and without an original success (the ON replay did not exit 0, so no comparator ran) cannot be classified from the summary: ambiguous (N = 1)
    unless the allowed single re-run gave it a verdict. A success run without an ON verdict is judged by the registered rule (replay exit != 0 = FAIL, G6).
  * Re-run file (v4, finding #12): only the targets of preregistration section 8.1 are accepted (see apply_rerun); every other entry is rejected, listed in `rerun_rejected` and in the notices.
  * G9 (D-V3-8 C-2): a run is marked when one process outside the gate tree used >= 1.0 core in >= 3 consecutive samples (each at most GAP_S after the previous) inside the run; a run that overlaps a gap
    between samples longer than GAP_S (or lies outside the recorded time) is unobserved. Marked and unobserved runs together count against the 15 % limit; the re-analysis without them removes the marked ones.
"""
import argparse
import bisect
import datetime
import hashlib
import json
import os
import sys
from collections import Counter
from fractions import Fraction

import numpy as np
from scipy import optimize, special, stats

sys.dont_write_bytecode = True

SCHEMA = 'slam-refact-us025-lc-v4-equivalence-analysis-v1'
SEED = 20261028
BOOTSTRAP = 10000
SCENARIOS = ('flip6-s3-lowtex', 'flip6-s2')
ARMS = ('S', 'P2', 'P4')
P_ARMS = ('P2', 'P4')
PLANNED_PER_ARM_SCENARIO = 24
MIN_STARTED_PER_ARM_SCENARIO = 22                       # G0
G1_BAND = (0.97, 1.03)                                  # 90 % interval of median(P) / median(S) must lie inside
G2_P90_FACTOR = 1.05
G2_MAX_FACTOR = 1.15
G3_NOFRESH_MARGIN = {'flip6-s3-lowtex': 4.0, 'flip6-s2': 21.0}   # upper limit of the 90 % interval of mean(NO_FRESH P) - mean(NO_FRESH S)
G4_DELTA = 0.25                                         # H0: success(S) - success(P) >= 0.25
G4_GUARD_POOLED = Fraction(-1, 8)                       # point estimate P - S >= -0.125
G4_GUARD_SCENARIO = Fraction(-3, 10)                    # each scenario >= -0.30
G5_DELTA = 0.20                                         # H0: hang(P) - hang(S) >= 0.20
G5_GUARD_POOLED = Fraction(1, 10)                       # point estimate hang P - S <= +0.10
G5_GUARD_SCENARIO = Fraction(1, 4)                      # each scenario <= +0.25
ALPHA = 0.10                                            # one-sided, G4, G5 and the Fisher test of G6
GRID_STEP = 0.001
Z_TIE = 1e-9
G8_BAND = (0.98, 1.02)
G8_EARLY_BLOCKS = (1, 4, 9)
G8_LATE_BLOCKS = (11, 15, 17)
G9_MAX_FLAGGED_FRACTION = 0.15
DISTURB_CORES = 1.0                                     # one process outside the gate tree using at least this many cores ...
DISTURB_SAMPLES = 3                                     # ... in this many consecutive samples (about 30 s)
GAP_S = 30.0                                            # samples further apart than this are a gap: a run that overlaps it is unobserved
OK_VERDICTS = ('PASS', 'PASS_DECLARED')
COMPLETED = ('COMPLETED', 'COMPLETED_WITH_ERRORS')


# ---- run classification (preregistration section 3) ----------------------------------------------------------------------------------------------
def is_infra(r):
    return r.get('infra') is not None


def not_started(r):
    infra = r.get('infra') or ''
    return infra.startswith('not started') or infra.startswith('resources or lock not available')


def started(r):
    return r.get('startedAt') is not None and not not_started(r)


def in_timing_set(r):
    return not is_infra(r) and r.get('status') in COMPLETED and isinstance(r.get('runnerSeconds'), (int, float))


def is_success(r):
    return not is_infra(r) and r.get('originalSuccess') is True


def verdict_class(v):
    if v in OK_VERDICTS:
        return 'ok'
    if v is None or v == 'FAIL':
        return 'fail'        # a replay with exit code != 0 or a missing verdict is a FAIL (judge US025-M1/M2)
    return 'other'           # TOOL_ERROR, INVALID, BLOCKED, ORIGINAL_UNUSABLE ...: re-run once, then "ambiguous"


def frac(n, d):
    return Fraction(n, d) if d else None


def fl(x):
    return None if x is None else float(x)


def secs(runs):
    return np.array([r['runnerSeconds'] for r in runs], dtype=float)


def nofresh(runs):
    return np.array([float((r.get('metrics') or {}).get('noFreshCallback') or 0) for r in runs], dtype=float)


def pct(a, q):
    return float(np.percentile(a, q)) if len(a) else None


# ---- Farrington-Manning score test, exact unconditional p-value ------------------------------------------------------------------------------------
def _fm_restricted_closed(xt, nt, xr, nr, d0):
    """Restricted MLE of (p_t, p_r) under p_t - p_r = d0 (Farrington and Manning 1990, closed form)."""
    pt = xt / nt
    pr = xr / nr
    theta = nr / nt
    a = 1 + theta
    b = -(1 + theta + pt + theta * pr + d0 * (theta + 2))
    c = d0 * d0 + d0 * (2 * pt + theta + 1) + pt + theta * pr
    d = -pt * d0 * (1 + d0)
    v = b ** 3 / (27 * a ** 3) - b * c / (6 * a ** 2) + d / (2 * a)
    u = np.sign(v) * np.sqrt(b ** 2 / (9 * a ** 2) - c / (3 * a))
    w = (np.pi + np.arccos(np.clip(v / u ** 3, -1.0, 1.0))) / 3
    p1 = 2 * u * np.cos(w) - b / (3 * a)
    return p1, p1 - d0


def _fm_restricted_numeric(xt, nt, xr, nr, d0):
    lo, hi = max(0.0, d0), min(1.0, 1.0 + d0)

    def negll(p):
        return -(special.xlogy(xt, p) + special.xlogy(nt - xt, 1 - p) + special.xlogy(xr, p - d0) + special.xlogy(nr - xr, 1 - p + d0))

    res = optimize.minimize_scalar(negll, bounds=(lo, hi), method='bounded', options={'xatol': 1e-13})
    return float(res.x), float(res.x) - d0


def fm_z_table(nt, nr, d0, numeric_only=False):
    """Z statistic of every table (x_t, x_r), shape (nt+1, nr+1). d0 = the difference p_t - p_r under H0 (negative for non-inferiority)."""
    xt = np.arange(nt + 1, dtype=float)[:, None]
    xr = np.arange(nr + 1, dtype=float)[None, :]
    xt, xr = np.broadcast_arrays(xt, xr)
    with np.errstate(all='ignore'):
        pt_t, pr_t = _fm_restricted_closed(xt, nt, xr, nr, d0)
        bad = ~np.isfinite(pt_t) | (pt_t < max(0.0, d0) - 1e-12) | (pt_t > min(1.0, 1.0 + d0) + 1e-12)
    if numeric_only:
        bad = np.ones_like(bad, dtype=bool)
    pt_t = np.where(bad, 0.0, pt_t)
    pr_t = np.where(bad, 0.0, pr_t)
    for i, j in zip(*np.nonzero(bad)):
        pt_t[i, j], pr_t[i, j] = _fm_restricted_numeric(xt[i, j], nt, xr[i, j], nr, d0)
    var = pt_t * (1 - pt_t) / nt + pr_t * (1 - pr_t) / nr
    with np.errstate(all='ignore'):
        z = (xt / nt - xr / nr - d0) / np.sqrt(var)
    return z


def fm_exact_noninferiority(x_t, n_t, x_r, n_r, delta, numeric_only=False):
    """One-sided exact unconditional Farrington-Manning test of H0: p_r - p_t >= delta (test arm t is worse than reference r by at least delta).
    Returns (p-value, nuisance p_t of the maximum, observed Z). Reject H0 (non-inferior) when the p-value <= ALPHA."""
    d0 = -float(delta)
    z = fm_z_table(n_t, n_r, d0, numeric_only)
    z_obs = z[x_t, x_r]
    mask = z >= z_obs - Z_TIE
    xt = np.arange(n_t + 1)
    xr = np.arange(n_r + 1)
    steps = int(round((1 - delta) / GRID_STEP))
    best, arg = -1.0, 0.0
    for k in range(steps + 1):
        pt = min(k * GRID_STEP, 1.0 - delta)
        pr = pt + delta
        w = np.outer(stats.binom.pmf(xt, n_t, pt), stats.binom.pmf(xr, n_r, pr))
        p = float(w[mask].sum())
        if p > best:
            best, arg = p, pt
    return min(best, 1.0), arg, float(z_obs)


# ---- bootstrap -----------------------------------------------------------------------------------------------------------------------------------
def bootstrap_pair(s_runs, p_runs):
    """90 % percentile intervals (5th-95th) of median(P)/median(S) of runner seconds and of mean(NO_FRESH P) - mean(NO_FRESH S), same resamples."""
    rng = np.random.default_rng(SEED)
    s_sec, p_sec, s_nf, p_nf = secs(s_runs), secs(p_runs), nofresh(s_runs), nofresh(p_runs)
    si = rng.integers(0, len(s_sec), size=(BOOTSTRAP, len(s_sec)))
    pi = rng.integers(0, len(p_sec), size=(BOOTSTRAP, len(p_sec)))
    ratio = np.median(p_sec[pi], axis=1) / np.median(s_sec[si], axis=1)
    diff = p_nf[pi].mean(axis=1) - s_nf[si].mean(axis=1)
    return [float(np.percentile(ratio, 5)), float(np.percentile(ratio, 95))], [float(np.percentile(diff, 5)), float(np.percentile(diff, 95))]


# ---- per arm / scenario selections -----------------------------------------------------------------------------------------------------------------
def select(runs, arm, scenario=None):
    return [r for r in runs if r.get('arm') == arm and (scenario is None or r.get('scenario') == scenario)]


def arm_summary(runs, arm, scenario):
    rs = select(runs, arm, scenario)
    ts = [r for r in rs if in_timing_set(r)]
    s = secs(ts)
    loads = [r['loadAvgAtStart'][0] for r in rs if r.get('loadAvgAtStart')]
    return {'lc1_runs': len(rs), 'started': sum(1 for r in rs if started(r)), 'infra': sum(1 for r in rs if is_infra(r)), 'success': sum(1 for r in rs if is_success(r)),
            'timing_set': len(ts), 'runner_seconds': {'p10': pct(s, 10), 'p50': pct(s, 50), 'p90': pct(s, 90), 'max': float(s.max()) if len(s) else None},
            'original_attempts': dict(sorted(Counter(str(r.get('originalAttempts')) for r in rs if not is_infra(r)).items())),
            'noFresh_mean': float(nofresh(ts).mean()) if ts else None, 'input_frames': sorted({(r.get('metrics') or {}).get('inputFrames') for r in ts}, key=lambda x: (x is None, x)),
            'start_load_1min': {'mean': float(np.mean(loads)) if loads else None, 'max': float(max(loads)) if loads else None}}


def block_medians(runs, arm, scenario):
    out = {}
    for r in select(runs, arm, scenario):
        if in_timing_set(r):
            out.setdefault(r['block'], []).append(r['runnerSeconds'])
    return {str(b): float(np.median(v)) for b, v in sorted(out.items())}


def cpus_report(runs):
    """Report only (prereview C-1): runner seconds per arm, scenario and CPU set of the slot, to see whether the slots of an arm differ."""
    out = {}
    for arm in ARMS:
        for scn in SCENARIOS:
            groups = {}
            for r in select(runs, arm, scn):
                if in_timing_set(r):
                    groups.setdefault(str(r.get('cpus')), []).append(r['runnerSeconds'])
            out[f'{arm}/{scn}'] = {k: {'n': len(v), 'p10': pct(np.array(v), 10), 'p50': pct(np.array(v), 50), 'p90': pct(np.array(v), 90), 'max': float(max(v))} for k, v in sorted(groups.items())}
    return out


# ---- the gates -------------------------------------------------------------------------------------------------------------------------------------
def gate_g0(summary, runs, summary_present=True):
    reasons = []
    if not summary_present:
        return {'pass': False, 'reasons': ['dev-summary.json does not exist']}
    if summary.get('infraStop'):
        reasons.append('infraStop is true')
    counts = {}
    for arm in ARMS:
        for scn in SCENARIOS:
            rs = select(runs, arm, scn)
            n = sum(1 for r in rs if started(r))
            counts[f'{arm}/{scn}'] = n
            if n < MIN_STARTED_PER_ARM_SCENARIO:
                reasons.append(f'{arm}/{scn}: {n} started lc=1 runs < {MIN_STARTED_PER_ARM_SCENARIO} (planned {PLANNED_PER_ARM_SCENARIO})')
    not_run = [f"{r.get('scenario')}/{r.get('run')}" for r in runs if (r.get('infra') or '').startswith('not started')]
    if not_run:
        reasons.append(f'{len(not_run)} runs were not started (notRun): {", ".join(not_run[:5])}')
    return {'pass': not reasons, 'started_per_arm_scenario': counts, 'not_run': not_run, 'reasons': reasons}


def gate_timing(runs, parm, scenario):
    """G1, G2, G3 for one P arm and one scenario."""
    s_runs = [r for r in select(runs, 'S', scenario) if in_timing_set(r)]
    p_runs = [r for r in select(runs, parm, scenario) if in_timing_set(r)]
    out = {'n_S': len(s_runs), 'n_P': len(p_runs), 'ambiguous': []}
    if not s_runs:
        out['ambiguous'].append('S timing set is empty')
        return {**out, 'g1': {'pass': False}, 'g2': {'pass': False}, 'g3': {'pass': False}}
    if not p_runs:
        fail = {'pass': False, 'reason': 'P timing set is empty'}
        return {**out, 'g1': fail, 'g2': fail, 'g3': fail}
    s_sec, p_sec = secs(s_runs), secs(p_runs)
    ci_ratio, ci_nf = bootstrap_pair(s_runs, p_runs)
    out['g1'] = {'median_S': float(np.median(s_sec)), 'median_P': float(np.median(p_sec)), 'ratio_point': float(np.median(p_sec) / np.median(s_sec)), 'ci90': ci_ratio, 'band': list(G1_BAND),
                 'pass': bool(ci_ratio[0] >= G1_BAND[0] and ci_ratio[1] <= G1_BAND[1])}
    p90_s, p90_p = pct(s_sec, 90), pct(p_sec, 90)
    out['g2'] = {'p90_S': p90_s, 'p90_P': p90_p, 'p90_limit': G2_P90_FACTOR * p90_s, 'max_P': float(p_sec.max()), 'max_limit': G2_MAX_FACTOR * float(np.median(s_sec)),
                 'pass': bool(p90_p <= G2_P90_FACTOR * p90_s and float(p_sec.max()) <= G2_MAX_FACTOR * float(np.median(s_sec)))}
    s_frames = sorted({(r.get('metrics') or {}).get('inputFrames') for r in s_runs}, key=lambda x: (x is None, x))
    p_bad = [r['run'] for r in p_runs if (r.get('metrics') or {}).get('inputFrames') != s_frames[0]]
    if len(s_frames) != 1 or s_frames[0] is None:
        out['ambiguous'].append(f'S inputFrames is not a single value: {s_frames}')
    margin = G3_NOFRESH_MARGIN[scenario]
    out['g3'] = {'S_inputFrames': s_frames, 'P_runs_with_other_frames': p_bad, 'noFresh_mean_S': float(nofresh(s_runs).mean()), 'noFresh_mean_P': float(nofresh(p_runs).mean()),
                 'diff_ci90': ci_nf, 'margin_upper': margin, 'pass': bool(not p_bad and ci_nf[1] <= margin and len(s_frames) == 1)}
    return out


def gate_ratios(runs, parm):
    """G4 and G5 (both scenarios pooled, plus the per-scenario point guards)."""
    s_all, p_all = select(runs, 'S'), select(runs, parm)
    nS, nP = len(s_all), len(p_all)
    out = {'n_S': nS, 'n_P': nP}
    if not nS or not nP:
        return {**out, 'g4': {'pass': False, 'reason': 'no runs'}, 'g5': {'pass': False, 'reason': 'no runs'}}
    sS, sP = sum(1 for r in s_all if is_success(r)), sum(1 for r in p_all if is_success(r))
    hS, hP = sum(1 for r in s_all if is_infra(r)), sum(1 for r in p_all if is_infra(r))
    p4, arg4, z4 = fm_exact_noninferiority(sP, nP, sS, nS, G4_DELTA)
    point4 = frac(sP, nP) - frac(sS, nS)
    per4 = {}
    for scn in SCENARIOS:
        a, b = select(runs, 'S', scn), select(runs, parm, scn)
        per4[scn] = (frac(sum(1 for r in b if is_success(r)), len(b)) - frac(sum(1 for r in a if is_success(r)), len(a))) if a and b else None
    guard4 = point4 >= G4_GUARD_POOLED
    scn_guard4 = {scn: (per4[scn] is not None and per4[scn] >= G4_GUARD_SCENARIO) for scn in SCENARIOS}
    out['g4'] = {'success_S': sS, 'success_P': sP, 'rate_S': sS / nS, 'rate_P': sP / nP, 'point_P_minus_S': fl(point4), 'point_guard': fl(G4_GUARD_POOLED), 'point_guard_ok': bool(guard4),
                 'per_scenario_P_minus_S': {k: fl(v) for k, v in per4.items()}, 'per_scenario_guard': fl(G4_GUARD_SCENARIO), 'per_scenario_guard_ok': scn_guard4, 'delta': G4_DELTA, 'p_exact': p4, 'nuisance_at_max': arg4, 'z_observed': z4,
                 'alpha': ALPHA, 'rejects_h0': bool(p4 <= ALPHA), 'pass': bool(p4 <= ALPHA and guard4 and all(scn_guard4.values()))}
    p5, arg5, z5 = fm_exact_noninferiority(nP - hP, nP, nS - hS, nS, G5_DELTA)
    point5 = frac(hP, nP) - frac(hS, nS)
    per5 = {}
    for scn in SCENARIOS:
        a, b = select(runs, 'S', scn), select(runs, parm, scn)
        per5[scn] = (frac(sum(1 for r in b if is_infra(r)), len(b)) - frac(sum(1 for r in a if is_infra(r)), len(a))) if a and b else None
    guard5 = point5 <= G5_GUARD_POOLED
    scn_guard5 = {scn: (per5[scn] is not None and per5[scn] <= G5_GUARD_SCENARIO) for scn in SCENARIOS}
    out['g5'] = {'hang_S': hS, 'hang_P': hP, 'rate_S': hS / nS, 'rate_P': hP / nP, 'point_P_minus_S': fl(point5), 'point_guard': fl(G5_GUARD_POOLED), 'point_guard_ok': bool(guard5),
                 'per_scenario_P_minus_S': {k: fl(v) for k, v in per5.items()}, 'per_scenario_guard': fl(G5_GUARD_SCENARIO), 'per_scenario_guard_ok': scn_guard5, 'delta': G5_DELTA, 'p_exact': p5, 'nuisance_at_max': arg5,
                 'z_observed': z5, 'alpha': ALPHA, 'rejects_h0': bool(p5 <= ALPHA), 'pass': bool(p5 <= ALPHA and guard5 and all(scn_guard5.values()))}
    return out


def gate_verdicts(runs, parm):
    """G6: any ON/OFF FAIL of a success run of P rejects; a value outside PASS/PASS_DECLARED/FAIL is a residual (ambiguous); the full-verdict FAIL rate of the non-success runs (Fisher)."""
    p_all, s_all = select(runs, parm), select(runs, 'S')
    fails, residual = [], []
    for r in p_all:
        if not is_success(r):
            continue
        for arm in ('on', 'off'):
            c = verdict_class((r.get('verdicts') or {}).get(arm))
            if c == 'fail':
                fails.append(f"{r['scenario']}/{r['run']}:{arm}={(r.get('verdicts') or {}).get(arm)}")
            elif c == 'other':
                residual.append(f"{r['scenario']}/{r['run']}:{arm}={(r.get('verdicts') or {}).get(arm)}")

    def full_counts(rs):
        den = [r for r in rs if not is_infra(r) and not is_success(r)]
        return sum(1 for r in den if (r.get('verdicts') or {}).get('full') == 'FAIL'), len(den)

    fP, dP = full_counts(p_all)
    fS, dS = full_counts(s_all)
    p_fisher = None
    if dP and dS:
        p_fisher = float(stats.fisher_exact([[fP, dP - fP], [fS, dS - fS]], alternative='greater').pvalue)
    full_reject = bool(p_fisher is not None and p_fisher <= ALPHA)
    return {'on_off_fail_runs': fails, 'residual_other_values': residual, 'full_fail_P': fP, 'full_den_P': dP, 'full_fail_S': fS, 'full_den_S': dS, 'fisher_p_greater': p_fisher, 'full_rejects': full_reject,
            'pass': bool(not fails and not full_reject), 'ambiguous': ['G6 residual verdicts after the allowed re-run: ' + ', '.join(residual)] if residual else []}


def gate_g8(runs):
    out = {'band': list(G8_BAND), 'early_blocks': list(G8_EARLY_BLOCKS), 'late_blocks': list(G8_LATE_BLOCKS), 'scenarios': {}, 'pass': True, 'ambiguous': []}
    for scn in SCENARIOS:
        ts = [r for r in select(runs, 'S', scn) if in_timing_set(r)]
        early = [r['runnerSeconds'] for r in ts if r.get('block') in G8_EARLY_BLOCKS]
        late = [r['runnerSeconds'] for r in ts if r.get('block') in G8_LATE_BLOCKS]
        if not early or not late:
            out['scenarios'][scn] = {'pass': False, 'reason': 'an empty early or late group'}
            out['pass'] = False
            out['ambiguous'].append(f'G8 {scn}: empty early/late group')
            continue
        ratio = float(np.median(late) / np.median(early))
        ok = bool(G8_BAND[0] <= ratio <= G8_BAND[1])
        out['scenarios'][scn] = {'median_early': float(np.median(early)), 'median_late': float(np.median(late)), 'ratio': ratio, 'n_early': len(early), 'n_late': len(late), 'pass': ok}
        if not ok:
            out['pass'] = False
            out['ambiguous'].append(f'G8 {scn}: S late/early median ratio {ratio:.4f} outside {list(G8_BAND)}')
    return out


def null_success_runs(runs):
    """R-5: runs that are not INFRA, have no originalSuccess (null: the ON comparator gave no originalRun) but have an ON verdict. They would count as "no success" and bypass G6 and its re-run path."""
    return [f"{r['scenario']}/{r['run']}:on={(r.get('verdicts') or {}).get('on')}" for r in runs
            if not is_infra(r) and r.get('originalSuccess') is None and (r.get('verdicts') or {}).get('on') is not None]


def replay_without_verdict(runs):
    """#2 (v4): a lc=1 run that is not INFRA and has NO ON verdict. In the gate that means the ON replay did not exit 0 (the ON comparator then does not run: no verdict, no originalRun in v3 summaries), which
    judge US025-M1/M2 counts as an engine FAIL. A SUCCESS run is judged by the registered rule above (verdict_class(None) is 'fail', G6). Any other run would stay in the G6 denominator as "not FAIL"
    and in the G1-G3 timing set: the analysis cannot classify it from the summary, so it is ambiguous (N = 1, fail-closed) unless the single re-run gave it a verdict."""
    out = []
    for r in runs:
        if is_infra(r) or is_success(r) or (r.get('verdicts') or {}).get('on') is not None:
            continue
        exits = {a: e for a, e in (r.get('replayExits') or {}).items() if e != 0}
        out.append(f"{r['scenario']}/{r['run']}:" + (f'replay exits {exits}' if exits else 'no ON verdict (the summary has no replay exit codes)'))
    return out


def evaluate(runs):
    """G1-G8, R-5 and the adoption rule on a set of lc=1 runs (no G0, no G9, no R-1). Returns the per-arm gate record and the outcome."""
    g8 = gate_g8(runs)
    arms = {}
    ambiguous = list(g8['ambiguous'])
    r5 = null_success_runs(runs)
    if r5:
        ambiguous.append('R-5: runs without originalSuccess but with an ON verdict (not counted as success, not checked by G6): ' + ', '.join(r5))
    r6 = replay_without_verdict(runs)
    if r6:
        ambiguous.append('R-6 (finding #2): non-INFRA runs without an ON verdict and without an original success (an ON replay that did not exit 0 cannot be classified from the summary): ' + ', '.join(r6))
    passed = {}
    for parm in P_ARMS:
        timing = {scn: gate_timing(runs, parm, scn) for scn in SCENARIOS}
        ratios = gate_ratios(runs, parm)
        verd = gate_verdicts(runs, parm)
        for scn in SCENARIOS:
            ambiguous += [f'{parm}/{scn}: {a}' for a in timing[scn]['ambiguous']]
        ambiguous += [f'{parm}: {a}' for a in verd['ambiguous']]
        timing_ok = all(timing[scn]['g1']['pass'] and timing[scn]['g2']['pass'] and timing[scn]['g3']['pass'] for scn in SCENARIOS)
        passed[parm] = bool(timing_ok and ratios['g4']['pass'] and ratios['g5']['pass'] and verd['pass'])
        arms[parm] = {'timing': timing, 'ratios': ratios, 'g6': verd, 'pass_g1_g6': passed[parm]}
    g7 = {'p4_pass': passed['P4'], 'p2_pass': passed['P2'], 'triggered': bool(passed['P4'] and not passed['P2'])}
    if g7['triggered']:
        ambiguous.append('G7: P4 passed but P2 did not (a slot with half the CPUs cannot be better than one with all of them)')
    if ambiguous:
        n, status = 1, 'AMBIGUOUS'
    elif passed['P4']:
        n, status = 4, 'ADOPT'
    elif passed['P2']:
        n, status = 2, 'ADOPT'
    else:
        n, status = 1, 'NO_PASS'
    return {'arms': arms, 'g7': g7, 'g8': g8, 'r5_null_success': r5, 'r6_replay_without_verdict': r6, 'ambiguous': ambiguous, 'passed': passed, 'adopted_N': n, 'status': status}


# ---- external disturbance marks (G9, preregistration section 7.2) -------------------------------------------------------------------------------------
def parse_time(t):
    return datetime.datetime.fromisoformat(t.replace('Z', '+00:00')).timestamp()


def load_sampler(path):
    """Samples of sample_load.py: [{'t': epoch seconds, 'hot': {pid: cores}}] sorted by time, plus the time of the `end` line (or None). Lines with `error` / `end` are not samples."""
    samples, end = [], None
    with open(path, encoding='utf8') as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            o = json.loads(line)
            if o.get('end'):
                end = parse_time(o['t'])
                continue
            if 'error' in o or 'others' not in o:
                continue
            hot = {int(p['pid']): float(p['cores']) for p in o['others'] if float(p['cores']) >= DISTURB_CORES}
            samples.append({'t': parse_time(o['t']), 'hot': hot})
    samples.sort(key=lambda x: x['t'])
    return samples, end


def sample_gaps(samples):
    """[(t_i, t_i+1)] of consecutive samples more than GAP_S apart."""
    return [(a['t'], b['t']) for a, b in zip(samples, samples[1:]) if b['t'] - a['t'] > GAP_S]


def disturbed_runs(runs, samples):
    """Runs during whose [startedAt, endedAt] one process outside the gate tree used >= DISTURB_CORES in >= DISTURB_SAMPLES consecutive samples (no gap longer than GAP_S in between)."""
    times = [x['t'] for x in samples]
    flagged = []
    for r in runs:
        if not r.get('startedAt') or not r.get('endedAt'):
            continue
        a, b = bisect.bisect_left(times, parse_time(r['startedAt'])), bisect.bisect_right(times, parse_time(r['endedAt']))
        streak = {}
        hit = False
        prev_t = None
        for smp in samples[a:b]:
            if prev_t is not None and smp['t'] - prev_t > GAP_S:
                streak = {}
            prev_t = smp['t']
            streak = {pid: streak.get(pid, 0) + 1 for pid in smp['hot']}
            if any(v >= DISTURB_SAMPLES for v in streak.values()):
                hit = True
                break
        if hit:
            flagged.append(f"{r['scenario']}/{r['run']}")
    return flagged


def unobserved_runs(runs, samples, end):
    """Runs the recorder did not see: the run overlaps a gap between samples longer than GAP_S, or lies (partly) before the first sample or after the last sample (+ GAP_S, or the `end` line)."""
    if not samples:
        return [f"{r['scenario']}/{r['run']}" for r in runs if r.get('startedAt') and r.get('endedAt')]
    gaps = sample_gaps(samples)
    # G-20 (v4): the `end` line does NOT extend the observed time: a recorder that wrote only `error` lines after its last sample observed nothing in that stretch
    first, last = samples[0]['t'], samples[-1]['t'] + GAP_S
    out = []
    for r in runs:
        if not r.get('startedAt') or not r.get('endedAt'):
            continue
        a, b = parse_time(r['startedAt']), parse_time(r['endedAt'])
        if a < first or b > last or any(g0 < b and g1 > a for g0, g1 in gaps):
            out.append(f"{r['scenario']}/{r['run']}")
    return out


def gate_g9(runs, main_eval, sampler_path):
    if sampler_path is None:
        return {'pass': False, 'evaluated': False, 'reason': 'no sampler.jsonl was given: the disturbance recorder is a precondition of the experiment (section 7.1 item 7)', 'ambiguous': ['G9: no sampler']}
    samples, end = load_sampler(sampler_path)
    flagged = disturbed_runs(runs, samples)
    unobserved = unobserved_runs(runs, samples, end)
    marked = set(flagged) | set(unobserved)
    per_arm = {}
    ambiguous = []
    for arm in ARMS:
        rs = select(runs, arm)
        keys = [f"{r['scenario']}/{r['run']}" for r in rs]
        fa = [k for k in keys if k in flagged]
        ua = [k for k in keys if k in unobserved]
        frac_marked = len([k for k in keys if k in marked]) / len(rs) if rs else 0.0
        per_arm[arm] = {'runs': len(rs), 'flagged': len(fa), 'unobserved': len(ua), 'fraction': frac_marked, 'over_limit': bool(frac_marked > G9_MAX_FLAGGED_FRACTION)}
        if per_arm[arm]['over_limit']:
            ambiguous.append(f'G9: {arm} has {len(set(fa) | set(ua))} of {len(rs)} runs with an external disturbance mark or no observation (> {G9_MAX_FLAGGED_FRACTION:.0%})')
    reduced = [r for r in runs if f"{r['scenario']}/{r['run']}" not in set(flagged)]
    sens = evaluate(reduced) if flagged else main_eval
    different = sens['adopted_N'] != main_eval['adopted_N']
    if different:
        ambiguous.append(f"G9: without the marked runs the adopted N is {sens['adopted_N']}, not {main_eval['adopted_N']}")
    return {'pass': not ambiguous, 'evaluated': True, 'samples': len(samples), 'gaps': [[g0, g1] for g0, g1 in sample_gaps(samples)], 'flagged_runs': flagged, 'unobserved_runs': unobserved, 'per_arm': per_arm,
            'sensitivity': {'adopted_N': sens['adopted_N'], 'status': sens['status'], 'passed': sens['passed'], 'ambiguous': sens['ambiguous']}, 'differs_from_main': different, 'ambiguous': ambiguous}


# ---- R-1: leave one block out (D-V3-8) -----------------------------------------------------------------------------------------------------------------
def gate_lobo(runs, main_eval):
    """The independent unit of an arm is the block (6 per arm), not the run. With an adopted N > 1, leave out each block of each P arm in turn and re-run G1-G8 (and R-5); a SMALLER adopted N is ambiguous (D-V3-9), a larger one is only reported."""
    if main_eval['adopted_N'] <= 1:
        return {'evaluated': False, 'reason': 'the adopted N is 1: nothing to confirm', 'cases': [], 'ambiguous': []}
    cases, ambiguous = [], []
    for parm in P_ARMS:
        for block in sorted({r.get('block') for r in select(runs, parm) if r.get('block') is not None}):
            reduced = [r for r in runs if not (r.get('arm') == parm and r.get('block') == block)]
            ev = evaluate(reduced)
            cases.append({'left_out': f'{parm}/block {block}', 'runs_removed': len(runs) - len(reduced), 'adopted_N': ev['adopted_N'], 'status': ev['status'], 'passed': ev['passed']})
            if ev['adopted_N'] < main_eval['adopted_N']:   # D-V3-9: only a SMALLER N is ambiguous; a larger one is ignored (the main N is already the conservative one)
                ambiguous.append(f"R-1: without {parm} block {block} the adopted N is {ev['adopted_N']} ({ev['status']}), smaller than {main_eval['adopted_N']}")
    return {'evaluated': True, 'cases': cases, 'ambiguous': ambiguous}


# ---- the whole analysis ---------------------------------------------------------------------------------------------------------------------------
def apply_rerun(runs, rerun):
    """The single re-run of preregistration section 8.1, fail-closed (v4, finding #12). A re-run result is accepted only for a target the rules allow:
      * 'on' / 'off': a SUCCESS run (G6), or an R-5 run (originalSuccess null with an ON verdict); the value before must be OUTSIDE PASS / PASS_DECLARED / FAIL (TOOL_ERROR, INVALID, BLOCKED ...)
      * 'originalSuccess': an R-5 run only (before-value null, an ON verdict exists), a boolean
      * 'full' is never a re-run target; an INFRA run is none; an unknown run key or key is none.
    Everything else is REJECTED (not applied) and returned in `rejected`; the unresolved value then stays and decides as before. Returns (applied, rejected)."""
    applied, rejected = [], []
    by_key = {f"{r.get('scenario')}/{r.get('run')}": r for r in runs}
    for key, entry in rerun.items():
        r = by_key.get(key)
        if r is None or not isinstance(entry, dict):
            rejected.append({'run': key, 'key': None, 'why': 'not an lc=1 run of the summary, or the entry is not an object'})
            continue
        r.setdefault('verdicts', {})
        is_succ = is_success(r)
        is_r5 = (not is_infra(r)) and r.get('originalSuccess') is None and r['verdicts'].get('on') is not None
        for k, v in entry.items():
            why = None
            if is_infra(r):
                why = 'an INFRA run is not a re-run target'
            elif k == 'full':
                why = "'full' is not a re-run target (preregistration 8.1)"
            elif k in ('on', 'off'):
                before = r['verdicts'].get(k)
                if not (is_succ or is_r5):
                    why = 'neither a success run (G6) nor an R-5 run'
                elif before is None or verdict_class(before) != 'other':
                    why = f'the value before ({before!r}) is PASS / PASS_DECLARED / FAIL or missing: only a value outside them may be re-run'
                elif not isinstance(v, str):
                    why = 'the new verdict is not a string'
            elif k == 'originalSuccess':
                if not is_r5 or r.get('originalSuccess') is not None:
                    why = 'not an R-5 run (needs originalSuccess null and an ON verdict)'
                elif not isinstance(v, bool):
                    why = 'originalSuccess must be true or false'
            else:
                why = 'unknown key'
            if why:
                rejected.append({'run': key, 'key': k, 'why': why})
                continue
            if k == 'originalSuccess':
                applied.append({'run': key, 'arm': k, 'before': r.get('originalSuccess'), 'after': v})
                r['originalSuccess'] = v
            else:
                applied.append({'run': key, 'arm': k, 'before': r['verdicts'].get(k), 'after': v})
                r['verdicts'][k] = v
    return applied, rejected


def sha256_file(path):
    h = hashlib.sha256()
    with open(path, 'rb') as f:
        h.update(f.read())
    return h.hexdigest()


def analyze(summary, sampler_path=None, rerun=None, summary_present=True):
    all_runs = [dict(r) for r in (summary.get('runs') or [])]
    runs = [r for r in all_runs if r.get('lc') == 1 and r.get('arm') in ARMS]
    for r in runs:
        r['verdicts'] = dict(r.get('verdicts') or {})
    rerun_applied, rerun_rejected = apply_rerun(runs, rerun or {})
    notices = []
    for x in rerun_rejected:
        notices.append(f"RERUN ENTRY REJECTED ({x['run']} {x['key']}): {x['why']}")
    # rule 5.6: an ON/OFF FAIL in the S arm points to the gate, not to the parallelism
    s_fail = [f"{r['scenario']}/{r['run']}:{a}={r['verdicts'].get(a)}" for r in select(runs, 'S') if is_success(r) for a in ('on', 'off') if verdict_class(r['verdicts'].get(a)) == 'fail']
    if s_fail:
        notices.append('S ARM VERDICT FAIL (rule 5.6: a gate-side problem, independent of N; report to the orchestrator; may block the v3 registration): ' + ', '.join(s_fail))
    g0 = gate_g0(summary, runs, summary_present)
    main = evaluate(runs) if summary_present and runs else {'arms': {}, 'g7': None, 'g8': None, 'ambiguous': [], 'passed': {'P2': False, 'P4': False}, 'adopted_N': 1, 'status': 'NO_PASS'}
    g9 = gate_g9(runs, main, sampler_path) if summary_present and runs else {'pass': False, 'evaluated': False, 'reason': 'no runs', 'ambiguous': []}
    lobo = gate_lobo(runs, main) if summary_present and runs and not main['ambiguous'] and not g9['ambiguous'] else {'evaluated': False, 'reason': 'not needed: the result is already INCOMPLETE, ambiguous or N = 1', 'cases': [], 'ambiguous': []}
    reasons = []
    if not g0['pass']:
        n, status = 1, 'INCOMPLETE'
        reasons += g0['reasons']
    elif main['ambiguous'] or g9['ambiguous'] or lobo['ambiguous']:
        n, status = 1, 'AMBIGUOUS'
        reasons += main['ambiguous'] + g9['ambiguous'] + lobo['ambiguous']
    else:
        n, status = main['adopted_N'], main['status']
        reasons.append({'ADOPT': f'P{n} passed G1-G6, survived every leave-one-block-out (R-1), and neither G0, G7, G8, G9 nor R-5 applies', 'NO_PASS': 'neither P4 nor P2 passed G1-G6'}[status])
    return {'schema': SCHEMA, 'adopted_N': n, 'status': status, 'reasons': reasons, 'notices': notices,
            'rules': {'seed': SEED, 'bootstrap': BOOTSTRAP, 'g1_band': list(G1_BAND), 'g2': [G2_P90_FACTOR, G2_MAX_FACTOR], 'g3_nofresh_margin': G3_NOFRESH_MARGIN, 'g4': [G4_DELTA, fl(G4_GUARD_POOLED), fl(G4_GUARD_SCENARIO)],
                      'g5': [G5_DELTA, fl(G5_GUARD_POOLED), fl(G5_GUARD_SCENARIO)], 'alpha': ALPHA, 'grid_step': GRID_STEP, 'g8': [list(G8_BAND), list(G8_EARLY_BLOCKS), list(G8_LATE_BLOCKS)],
                      'g9': [G9_MAX_FLAGGED_FRACTION, DISTURB_CORES, DISTURB_SAMPLES, GAP_S], 'min_started': MIN_STARTED_PER_ARM_SCENARIO},
            'versions': {'python': sys.version.split()[0], 'numpy': np.__version__, 'scipy': __import__('scipy').__version__},
            'g0': g0, 'g7': main['g7'], 'g8': main['g8'], 'r5_null_success': main.get('r5_null_success', []), 'r6_replay_without_verdict': main.get('r6_replay_without_verdict', []), 'g9': g9, 'r1_leave_one_block_out': lobo, 'comparisons': main['arms'], 'passed_g1_g6': main['passed'], 'rerun_applied': rerun_applied, 'rerun_rejected': rerun_rejected,
            'arms': {arm: {scn: arm_summary(runs, arm, scn) for scn in SCENARIOS} for arm in ARMS},
            'runner_seconds_by_cpus_report_only': cpus_report(runs),
            'block_medians_runner_seconds': {arm: {scn: block_medians(runs, arm, scn) for scn in SCENARIOS} for arm in ARMS},
            'ks_report_only': {parm: {scn: ks_report(runs, parm, scn) for scn in SCENARIOS} for parm in P_ARMS}}


def ks_report(runs, parm, scn):
    a = secs([r for r in select(runs, 'S', scn) if in_timing_set(r)])
    b = secs([r for r in select(runs, parm, scn) if in_timing_set(r)])
    if not len(a) or not len(b):
        return None
    try:
        res = stats.ks_2samp(a, b, method='exact')
        return {'statistic': float(res.statistic), 'p_exact': float(res.pvalue), 'n_S': len(a), 'n_P': len(b), 'note': 'report only, not a criterion (preregistration section 4)'}
    except Exception as error:   # noqa: BLE001 - report-only statistic
        return {'error': str(error)}


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--summary', required=True)
    ap.add_argument('--sampler')
    ap.add_argument('--rerun')
    ap.add_argument('--out', required=True)
    a = ap.parse_args(argv)
    if os.path.exists(a.out):
        print(f'{a.out} exists: the analysis file is never overwritten', file=sys.stderr)
        return 2
    present = os.path.exists(a.summary)
    try:
        summary = json.load(open(a.summary, encoding='utf8')) if present else {}
        rerun = json.load(open(a.rerun, encoding='utf8')) if a.rerun else None
        result = analyze(summary, a.sampler, rerun, present)
    except (OSError, ValueError, KeyError) as error:
        print(f'cannot analyse: {error}', file=sys.stderr)
        return 2
    result['inputs'] = {'summary': a.summary, 'summary_sha256': sha256_file(a.summary) if present else None, 'sampler': a.sampler, 'sampler_sha256': sha256_file(a.sampler) if a.sampler else None,
                        'rerun': a.rerun, 'rerun_sha256': sha256_file(a.rerun) if a.rerun else None}
    with open(a.out, 'x', encoding='utf8') as f:
        json.dump(result, f, indent=1, sort_keys=True, default=lambda o: o.item() if hasattr(o, 'item') else str(o))
        f.write('\n')
    print(json.dumps({'adopted_N': result['adopted_N'], 'status': result['status'], 'reasons': result['reasons'], 'notices': result['notices'], 'out': a.out}, indent=1))
    return 0


if __name__ == '__main__':
    sys.exit(main())

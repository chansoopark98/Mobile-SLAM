"""Verdicts of the trace-level gates of thresholds-v1.json for one candidate trace per scenario.

A candidate is a published trace in the reference schema: an engine run (tools/harness/replay) or,
for self-consistency tests, an original reference run. Reference values come from the frozen
`reference_baseline` block of thresholds-v1.json (r1/r2 observables at registration), except
MS-RT whose original absolute runs were produced after registration (their definitions are frozen).

Verdict vocabulary: PASS, FAIL, BLOCKED (reason), NOT-RUN (no candidate given), NOT-REGISTERED
(no value in §6.6 for that scenario), BETTER-THAN-REF (A-1 below 0.8 x reference min), REPORTED
(report-only metric). Gate classes are applied in parity_report.py.
"""
import json
import math
import pathlib

import accuracy as acc_mod
import frame_parity as fp
import memprobe as mp
import trace_metrics as tm
from common import EVIDENCE_ROOT, REFERENCE_RUNS, load_trace, percentile

TUM_GT = {
    'tum-room1-full': EVIDENCE_ROOT / 'reference/fixtures/tum-room1-portrait-full',
    'tum-room1-lowtex': EVIDENCE_ROOT / 'reference/fixtures/tum-room1-portrait-lowtex17-20',
}


def result(gate, scenario, verdict, measured=None, criterion=None, tags=None, note=None):
    r = {'id': gate['id'], 'class': gate['class'], 'scenario': scenario, 'verdict': verdict}
    if measured is not None:
        r['measured'] = measured
    if criterion is not None:
        r['criterion'] = criterion
    if tags:
        r['tags'] = tags
    if note:
        r['note'] = note
    return r


def in_range(v, lo, hi):
    return v is not None and lo <= v <= hi


def combine(parts):
    verdicts = [p['verdict'] for p in parts.values()]
    if 'FAIL' in verdicts:
        return 'FAIL'
    if 'BLOCKED' in verdicts:
        return 'BLOCKED'
    if all(v == 'NOT-APPLICABLE' for v in verdicts):
        return 'NOT-RUN'
    return 'PASS'


class Context:
    """Candidate traces and frozen reference observables."""

    def __init__(self, thresholds, runs, injected=None, absolute=None, memprobe=None):
        self.thresholds = thresholds
        # {scenario: memprobe.ProbeReference}: two original memory-probe runs that replace r1/r2 as the reference of
        # G-M3(b), G-M4, G-M5, G-M7 (the only way their registered blocked_if condition is lifted; rules unchanged).
        self.memprobe = memprobe or {}
        self.baseline = thresholds['reference_baseline']['data']['scenarios']
        self.runs = {k: pathlib.Path(v) for k, v in (runs or {}).items()}
        self.injected = {k: pathlib.Path(v) for k, v in (injected or {}).items()}
        self.absolute = {k: pathlib.Path(v) for k, v in (absolute or {}).items()}
        self._rows = {}

    def rows(self, run_dir):
        key = str(run_dir)
        if key not in self._rows:
            self._rows[key] = load_trace(pathlib.Path(run_dir) / 'trace.jsonl')
        return self._rows[key]

    def ref_rows(self, scenario, label='r1'):
        return self.rows(REFERENCE_RUNS / scenario / label)

    def ref_obs(self, scenario):
        return self.baseline[scenario]['runs']


def _ref_values(ctx, scenario, fn):
    return [fn(o) for o in ctx.ref_obs(scenario).values()]


# ------------------------------------------------------------------------------------- per gate
def eval_status_hysteresis(ctx, gate, sc, run):
    h = tm.status_hysteresis(run / 'trace.jsonl')
    return result(gate, sc, 'PASS' if h['pass'] else 'FAIL', h, gate['tolerance'])


def eval_limited_ratio(ctx, gate, sc, run):
    refs = _ref_values(ctx, sc, lambda o: o['limited_ratio'])
    lo, hi = min(refs) - gate['tolerance']['margin_abs'], max(refs) + gate['tolerance']['margin_abs']
    v = tm.limited_ratio(ctx.rows(run))
    return result(gate, sc, 'PASS' if in_range(v, lo, hi) else 'FAIL', {'limited_ratio': v, 'reference': refs}, [lo, hi])


def _tag_input_artifact(ctx, sc):
    lag = ctx.ref_obs(sc)['r1']['publish_lag_ms']
    if lag and lag['spread'] > lag['frame_interval_ms']:
        return ['INPUT-ARTIFACT-POSSIBLE']
    return None


def _event_window(ctx, gate, sc, run, ref_fn, cand_fn, margin):
    refs = _ref_values(ctx, sc, ref_fn)
    v = cand_fn(ctx.rows(run))
    if all(r is None for r in refs):
        return 'PASS' if v is None else 'FAIL', v, refs, 'none expected'
    if any(r is None for r in refs):
        known = [r for r in refs if r is not None]
        window = [min(known) - margin, max(known) + margin]
        return ('PASS' if v is None or in_range(v, *window) else 'FAIL'), v, refs, {'none_or': window}
    window = [min(refs) - margin, max(refs) + margin]
    return ('PASS' if in_range(v, *window) else 'FAIL'), v, refs, window


def eval_first_limited(ctx, gate, sc, run):
    margin = gate['tolerance']['frames']
    verdict, v, refs, crit = _event_window(ctx, gate, sc, run, lambda o: o['first_limited'] and o['first_limited']['i'],
                                           lambda rows: (tm.first_limited(rows) or {}).get('i'), margin)
    tags = _tag_input_artifact(ctx, sc) if verdict == 'FAIL' and gate['id'] == 'MT-P7-3' else None
    return result(gate, sc, verdict, {'first_limited_i': v, 'reference': refs}, crit, tags)


def eval_first_translation(ctx, gate, sc, run):
    verdict, v, refs, crit = _event_window(ctx, gate, sc, run, lambda o: o['first_translation_i'], tm.first_translation,
                                           gate['tolerance']['frames'])
    return result(gate, sc, verdict, {'first_translation_i': v, 'reference': refs}, crit)


def eval_rotation_p95_full(ctx, gate, sc, run):
    lim = gate['tolerance']['rot_p95_max_deg'].get(sc)
    if lim is None:
        return result(gate, sc, 'NOT-REGISTERED', note=gate.get('not_registered', {}).get(sc))
    _, _, dr = fp.matched_diffs(ctx.rows(run), ctx.ref_rows(sc), angle=fp.normalized_angle_deg)
    v = percentile(dr, 95, 'linear')
    return result(gate, sc, 'PASS' if v is not None and v <= lim else 'FAIL', {'rot_diff_p95_deg': v, 'frames': len(dr)}, {'max': lim})


def eval_first_world_points(ctx, gate, sc, run):
    rows = ctx.rows(run)
    fw = tm.first_world_points(rows)
    refs = _ref_values(ctx, sc, lambda o: o['first_world_points'])
    first_reset = next((r['i'] for r in rows if r.get('fresh') and r.get('srMapReset')), None)
    if gate['id'] == 'G-M1':
        m = gate['tolerance']['frames']
        window = [min(r['i'] for r in refs) - m, max(r['i'] for r in refs) + m]
        v = fw and fw['i']
        return result(gate, sc, 'PASS' if in_range(v, *window) else 'FAIL',
                      {'first_world_points_i': v, 'reference': [r['i'] for r in refs], 'engine_first_srMapReset_i': first_reset}, window)
    counts = [r['count'] for r in refs]
    window = [0.8 * min(counts), 1.2 * max(counts)]
    v = fw and fw['count']
    return result(gate, sc, 'PASS' if in_range(v, *window) else 'FAIL', {'count': v, 'reference': counts}, window)


def eval_keyframe_cadence(ctx, gate, sc, run):
    rows = ctx.rows(run)
    parts = {}
    if any('srKeyframeCreated' in r for r in rows):
        iv = tm.engine_keyframe_intervals(rows)
        bad = [v for v in iv if v < 5]
        parts['a'] = {'verdict': 'FAIL' if bad else 'PASS', 'intervals': len(iv), 'violations': bad[:20]}
    else:
        parts['a'] = {'verdict': 'NOT-APPLICABLE', 'note': 'trace has no srKeyframeCreated (not an engine trace)'}
    refs = _ref_values(ctx, sc, lambda o: o['early_burst_share_5_6'])
    v = tm.early_burst_share_5_6(rows)['share_5_6']
    known = [r for r in refs if r is not None]
    window = [min(known) - 0.15, max(known) + 0.15] if known else None
    parts['b'] = {'verdict': 'PASS' if window and in_range(v, *window) else 'FAIL', 'share_5_6': v, 'reference': refs, 'window': window}
    return result(gate, sc, combine(parts), parts, gate['tolerance'])


def eval_keyframe_count(ctx, gate, sc, run):
    rows = ctx.rows(run)
    parts = {}
    kfs = [r['srKeyframes'] for r in rows if r.get('fresh') and r.get('srKeyframes') is not None]
    if kfs:
        parts['a'] = {'verdict': 'PASS' if max(kfs) <= 64 else 'FAIL', 'max_live_keyframes': max(kfs)}
    else:
        parts['a'] = {'verdict': 'NOT-APPLICABLE', 'note': 'trace has no srKeyframes'}
    created_30s = sum(1 for r in rows if r.get('fresh') and r.get('srKeyframeCreated') and r['t'] - rows[0]['t'] <= 30.0)
    probe = ctx.memprobe.get(sc)
    if probe is None:
        parts['b'] = {'verdict': 'BLOCKED', 'reason': gate['blocked_if']['b'], 'engine_created_first_30s': created_30s if kfs else None}
    else:
        p1, p2 = (m['keyframes_created_first_30s'] for m in probe.metrics())
        verdict, info = mp.rel_or_diff(created_30s if kfs else None, p1, p2, 0.25)
        parts['b'] = dict(info, verdict=verdict, reference_runs=MEMPROBE_SUBSTITUTION, probe_runs=[str(d) for d in probe.dirs])
    return result(gate, sc, combine(parts), parts, gate['tolerance'])


MEMPROBE_SUBSTITUTION = ('two original memory-probe runs (run-reference.mjs --memprobe) replace r1/r2 as the reference '
                         '(original is run-to-run nondeterministic, parity.md §3.1); registered tolerance rule unchanged')


def _engine_point_medians(fresh):
    by = [r['srPointsBySource'] for r in fresh if r.get('srPointsBySource')]
    return [sorted(b[k] for b in by)[len(by) // 2] for k in range(3)] if by else None


def eval_blocked_with_engine_values(ctx, gate, sc, run):
    rows = ctx.rows(run)
    fresh = [r for r in rows if r.get('fresh')]
    if gate['id'] == 'G-M4':
        med = _engine_point_medians(fresh)
        val = {'median_type_1_2_3': med} if med else None
    else:
        resets = [r for r in fresh if r.get('srMapReset')]
        val = {'map_resets': len(resets)} if any('srMapReset' in r for r in fresh) else None
    probe = ctx.memprobe.get(sc)
    if probe is None:
        return result(gate, sc, 'BLOCKED', {'engine': val}, gate['tolerance'], note=gate['blocked_if'])
    m1, m2 = probe.metrics()
    if gate['id'] == 'G-M4':
        parts = {}
        for k in range(3):
            verdict, info = mp.rel_or_diff(med[k] if med else None, m1['median_live_type_1_2_3'][k], m2['median_live_type_1_2_3'][k], 0.30)
            parts['type_%d' % (k + 1)] = dict(info, verdict=verdict)
        verdict = combine(parts)
        measured = dict(parts)
    else:
        verdict, measured = mp.reset_window(val and val['map_resets'], m1['map_resets'], m2['map_resets'])
    measured['reference_runs'] = MEMPROBE_SUBSTITUTION
    measured['probe_runs'] = [str(d) for d in probe.dirs]
    return result(gate, sc, verdict, measured, gate['tolerance'])


def eval_sky_points(ctx, gate, sc, run):
    """G-M7 (memory-probe level, flip6-s2). Verdict: median live type-3 count, G-M4 rule. Recorded: original sky
    points created per keyframe and frm+0x98 == 0 throughout (D-7/D-22); the engine trace has no per-keyframe
    creation count, so the per-keyframe value is reported for the original only."""
    probe = ctx.memprobe.get(sc)
    if probe is None:
        return result(gate, sc, 'BLOCKED', note=gate['blocked_if'])
    fresh = [r for r in ctx.rows(run) if r.get('fresh')]
    med = _engine_point_medians(fresh)
    m1, m2 = probe.metrics()
    verdict, info = mp.rel_or_diff(med and med[2], m1['median_live_type_1_2_3'][2], m2['median_live_type_1_2_3'][2], 0.30)
    measured = {'live_type_3_median': dict(info, verdict=verdict),
                'original_sky_created_per_keyframe_median': [m1['median_sky_created_per_keyframe'], m2['median_sky_created_per_keyframe']],
                'engine_sky_created_per_keyframe': None,
                'engine_sky_created_per_keyframe_note': 'not observable in the engine trace (srPointsBySource is a live count); not part of the verdict',
                'original_frm_0x98_nonzero_frames': [m1['frm_world_matches_nonzero_frames'], m2['frm_world_matches_nonzero_frames']],
                'original_frm_0x98_probed_frames': [m1['frm_world_matches_probed_frames'], m2['frm_world_matches_probed_frames']],
                'd7_zero_throughout': m1['frm_world_matches_nonzero_frames'] == 0 and m2['frm_world_matches_nonzero_frames'] == 0,
                'reference_runs': MEMPROBE_SUBSTITUTION, 'probe_runs': [str(d) for d in probe.dirs]}
    return result(gate, sc, verdict, measured, gate['tolerance'])


# Evaluated here only when the memory probe is given (parity_report --memprobe); otherwise the gate keeps its
# registered evaluator ('manual': verdict from --module-results).
MEMPROBE_EVALUATORS = {'G-M7': eval_sky_points}


def eval_runtime_invariants(ctx, gate, sc, run):
    inv = tm.runtime_invariants(run / 'trace.jsonl')
    if gate['id'] == 'V-P1-P4':
        ok = not inv['P1_violations'] and not inv['P2_violations'] and inv['P3_ok'] and inv['P4_ok']
        m = {k: inv[k] for k in ('P1_violations', 'P2_violations', 'P3_ok', 'P3_first_limited', 'P4_ok', 'P4_reasons')}
        return result(gate, sc, 'PASS' if ok else 'FAIL', m, gate['tolerance'])
    heads = inv['P6_heading_deg']
    p6 = all(h is not None and abs(h) <= gate['tolerance']['P6_abs_deg'] for h in heads.values())
    return result(gate, sc, 'PASS' if inv['P5_ok'] and p6 else 'FAIL', {'P5_ok': inv['P5_ok'], 'P6_heading_deg': heads}, gate['tolerance'])


def eval_padding(ctx, gate, sc, run):
    pad = tm.worldpoint_padding_fraction(run / 'trace.jsonl')
    ok = pad is not None and pad['fraction'] is not None and pad['fraction'] > 0
    return result(gate, sc, 'PASS' if ok else 'FAIL', pad, '> 0')


def _window_verdict(gate, sc, cand_rows, ref_rows, spec, tags_fn):
    lo, hi = spec['window']
    st = fp.window_stats(cand_rows, ref_rows, lo, hi, spec.get('percentile', 'nearest_rank'))
    ok = (st['frames_compared'] > 0 and st['rot_deg_p95'] <= spec['rot_p95_deg'] and st['rot_deg_max'] <= spec['rot_max_deg']
          and st['pos_p95'] <= spec['pos_p95'] and st['pos_max'] <= spec['pos_max'])
    crit = {k: spec[k] for k in ('window', 'rot_p95_deg', 'rot_max_deg', 'pos_p95', 'pos_max')}
    return result(gate, sc, 'PASS' if ok else 'FAIL', st, crit, tags_fn() if not ok else None)


def eval_pre_divergence(ctx, gate, sc, run):
    spec = dict(gate['tolerance']['windows'][sc], percentile=gate['tolerance']['percentile'])
    return _window_verdict(gate, sc, ctx.rows(run), ctx.ref_rows(sc), spec, lambda: None)


def eval_injected(ctx, gate, sc, run):
    spec = gate['tolerance']
    tags = (lambda: _tag_input_artifact(ctx, sc)) if gate['id'] == 'V-H7' else (lambda: None)
    return _window_verdict(gate, sc, ctx.rows(run), ctx.ref_rows(sc), spec, tags)


def eval_status_agreement(ctx, gate, sc, run):
    a = fp.status_agreement(ctx.rows(run), ctx.ref_rows(sc))
    lim = gate['tolerance']['min_agreement'][sc]
    return result(gate, sc, 'PASS' if a['agreement'] is not None and a['agreement'] >= lim else 'FAIL', a, {'min': lim})


def eval_zero_steps(ctx, gate, sc, run):
    m = gate['tolerance']['margin_abs']
    cand = tm.zero_step_fractions(ctx.rows(run))
    per = {}
    for status in sorted({s for o in ctx.ref_obs(sc).values() for s in o['zero_step_fractions']}):
        refs = [o['zero_step_fractions'][status] for o in ctx.ref_obs(sc).values() if status in o['zero_step_fractions']]
        window = [min(refs) - m, max(refs) + m]
        v = cand.get(status)
        per[status] = {'verdict': 'PASS' if in_range(v, *window) else 'FAIL', 'value': v, 'window': window}
    extra = sorted(set(cand) - set(per))
    verdict = 'FAIL' if any(p['verdict'] == 'FAIL' for p in per.values()) else 'PASS'
    return result(gate, sc, verdict, {'per_status': per, 'engine_only_statuses': {s: cand[s] for s in extra}}, gate['tolerance']['rule'])


def eval_world_points_p50(ctx, gate, sc, run):
    refs = _ref_values(ctx, sc, lambda o: o['world_points_p50'])
    window = [0.8 * min(refs), 1.25 * max(refs)]
    v = tm.world_points_p50(ctx.rows(run))
    return result(gate, sc, 'PASS' if in_range(v, *window) else 'FAIL', {'p50': v, 'reference': refs}, window)


def ms_rt_verdict(eng, r1, r2):
    """MS-RT pre-registered rule (thresholds-v1.json MS-RT tolerance). Each argument: first latch
    dict {'i', 'scale_proxy'} or None."""
    def close(e, refs, l_tol, s_tol):
        lb = sum(r['i'] for r in refs) / len(refs)
        sb = sum(r['scale_proxy'] for r in refs) / len(refs)
        return abs(e['i'] - lb) <= l_tol and abs(e['scale_proxy'] - sb) / sb <= s_tol, {'L_mean': lb, 's_bar': sb}
    if r1 is None and r2 is None:
        return ('PASS' if eng is None else 'FAIL'), {'rule': 'no_latch'}
    if (r1 is None) != (r2 is None):
        latched = r1 or r2
        if eng is None:
            return 'PASS', {'rule': 'originals_disagree_on_latch'}
        ok, info = close(eng, [latched], 5, 0.05)
        return ('PASS' if ok else 'FAIL'), dict(info, rule='originals_disagree_on_latch')
    sb = (r1['scale_proxy'] + r2['scale_proxy']) / 2
    primary = abs(r1['i'] - r2['i']) <= 5 and abs(r1['scale_proxy'] - r2['scale_proxy']) / sb <= 0.05
    if primary:
        l_tol, s_tol = 5, 0.05
    else:
        l_tol = max(5, 2 * abs(r1['i'] - r2['i']))
        s_tol = max(0.05, 2 * abs(r1['scale_proxy'] - r2['scale_proxy']) / sb)
    if eng is None:
        return 'FAIL', {'rule': 'primary' if primary else 'fallback', 'L_tol': l_tol, 's_tol': s_tol, 'engine': 'no latch'}
    ok, info = close(eng, [r1, r2], l_tol, s_tol)
    return ('PASS' if ok else 'FAIL'), dict(info, rule='primary' if primary else 'fallback', L_tol=l_tol, s_tol=s_tol)


def eval_ms_rt(ctx, gate, sc, run):
    def first(rows):
        lat = tm.scale_latches(rows)
        return lat[0] if lat else None
    refs = {}
    for lb in ('r1', 'r2'):
        p = REFERENCE_RUNS / sc / lb / 'trace.jsonl'
        if not p.exists():
            return result(gate, sc, 'BLOCKED', note=gate['blocked_if'] + ' (missing ' + str(p) + ')')
        refs[lb] = load_trace(p)
    r1, r2 = first(refs['r1']), first(refs['r2'])
    eng_latches = tm.scale_latches(ctx.rows(run))
    eng = eng_latches[0] if eng_latches else None
    verdict, info = ms_rt_verdict(eng, r1, r2)
    return result(gate, sc, verdict, {'engine_first_latch': eng, 'engine_latches': len(eng_latches), 'r1_first_latch': r1,
                                      'r2_first_latch': r2, **info}, gate['tolerance'])


def _accuracy(ctx, sc, run):
    fx = TUM_GT[sc]
    gt = acc_mod.load_gt(fx / 'groundtruth-body.csv')
    t_ic = json.loads((fx / 'manifest.json').read_text())['t_ic']
    return acc_mod.accuracy(ctx.rows(run), gt, t_ic, 0)


def eval_accuracy(ctx, gate, sc, run):
    a = _accuracy(ctx, sc, run)
    if 'sim3' not in a:
        return result(gate, sc, 'FAIL', {'associatedPoses': a['associatedPoses'], 'coverage': a['coverage']},
                      note='fewer than 10 associated poses')
    if gate['id'] == 'A-1':
        vals = {'ape_rot_rmse_deg': a['apeRotationDegRotationOnlyAlignment']['rmse'], 'rpe1s_rot_rmse_deg': a['rpe1s']['rotationDeg']['rmse'],
                'sim3_ate_rmse': a['sim3']['ateTranslation']['rmse']}
        parts = {}
        for k, v in vals.items():
            lo, hi = gate['tolerance']['reference_ranges'][k]
            lo, hi = 0.8 * lo, 1.25 * hi
            parts[k] = {'value': v, 'window': [lo, hi], 'verdict': 'BETTER-THAN-REF' if v < lo else ('PASS' if v <= hi else 'FAIL')}
        verdicts = [p['verdict'] for p in parts.values()]
        verdict = 'FAIL' if 'FAIL' in verdicts else ('BETTER-THAN-REF' if 'BETTER-THAN-REF' in verdicts else 'PASS')
        return result(gate, sc, verdict, dict(parts, sim3_scale=a['sim3']['scale']), gate['tolerance']['rule'])
    if gate['id'] == 'A-2':
        lim = gate['tolerance']['min_coverage']
        return result(gate, sc, 'PASS' if a['coverage'] >= lim else 'FAIL', {'coverage': a['coverage']}, {'min': lim})
    return result(gate, sc, 'REPORTED', {'se3_ate_translation': a['se3']['ateTranslation'], 'rpe1s': a['rpe1s'],
                                         'coverage': a['coverage'], 'units': 'responsive units (not metres) unless scale absolute'})


TRACE_EVALUATORS = {
    'MT-P2t': eval_status_hysteresis, 'MT-P7-1': eval_status_hysteresis, 'MT-P7-2': eval_limited_ratio,
    'MT-P7-3': eval_first_limited, 'MT-P7-4': eval_rotation_p95_full,
    'G-M1': eval_first_world_points, 'G-M2': eval_keyframe_cadence, 'G-M3': eval_keyframe_count,
    'G-M4': eval_blocked_with_engine_values, 'G-M5': eval_blocked_with_engine_values, 'G-M6': eval_first_world_points,
    'V-P1-P4': eval_runtime_invariants, 'V-P5/P6': eval_runtime_invariants, 'V-P8': eval_padding,
    'V-EV1': eval_first_translation, 'V-EV2': eval_first_limited, 'V-EV3': eval_status_agreement,
    'V-EV4': eval_zero_steps, 'V-EV5': eval_world_points_p50, 'V-PD': eval_pre_divergence,
    'V-H7': eval_injected, 'V-H7b': eval_injected, 'MS-RT': eval_ms_rt,
    'A-1': eval_accuracy, 'A-2': eval_accuracy, 'A-3': eval_accuracy,
}


def evaluate_trace_gates(ctx):
    out = []
    for gate in ctx.thresholds['gates']:
        fn = TRACE_EVALUATORS.get(gate['id'])
        if fn is None:
            continue
        source = ctx.injected if gate['id'] in ('V-H7', 'V-H7b') else ctx.absolute if gate['id'] == 'MS-RT' else ctx.runs
        for sc in gate.get('scenarios', []):
            run = source.get(sc)
            if run is None:
                out.append(result(gate, sc, 'NOT-RUN', note='no candidate run given'))
                continue
            out.append(fn(ctx, gate, sc, run))
    return out


def evaluate_memprobe_gates(ctx):
    """Gates of MEMPROBE_EVALUATORS for scenarios that have a memprobe reference; [] without --memprobe."""
    out = []
    for gate in ctx.thresholds['gates']:
        fn = MEMPROBE_EVALUATORS.get(gate['id'])
        if fn is None or not any(sc in ctx.memprobe for sc in gate.get('scenarios', [])):
            continue
        for sc in gate.get('scenarios', []):
            run = ctx.runs.get(sc)
            if sc not in ctx.memprobe:
                out.append(result(gate, sc, 'BLOCKED', note=gate['blocked_if']))
            elif run is None:
                out.append(result(gate, sc, 'NOT-RUN', note='no candidate run given'))
            else:
                out.append(fn(ctx, gate, sc, run))
    return out


def json_safe(x):
    if isinstance(x, float) and not math.isfinite(x):
        return None
    if isinstance(x, dict):
        return {k: json_safe(v) for k, v in x.items()}
    if isinstance(x, (list, tuple)):
        return [json_safe(v) for v in x]
    return x

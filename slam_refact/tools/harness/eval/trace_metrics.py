"""Per-trace observables used by the trace-level gates (architecture.md §6.6) and by the reference
baseline embedded in thresholds-v1.json. Every function takes the rows of one trace (original or
engine; same schema) and returns plain JSON values. Definitions are written once here so the
reference baseline and the engine evaluation cannot drift apart.

The status-machine and composition checks are the reused lane tools, imported unmodified:
  spec_map_tracker/status_hysteresis_check.py (MT-P2t, MT-P7 ①)
  spec_vio_tracker/runtime_checks.py          (V-P1..V-P8, V-EV4 zero-step fractions)
"""
import importlib.util
import json
import pathlib
import sys

from common import (ORIGIN_RESPONSIVE, REPO_ROOT, fresh_rows, frame_interval_s, percentile, posed_rows)

_RE = REPO_ROOT / 'slam_refact/tools/re'


def _load_tool(name, rel):
    # Loaded read-only: no __pycache__ is written into the lane tool directories.
    spec = importlib.util.spec_from_file_location(name, _RE / rel)
    mod = importlib.util.module_from_spec(spec)
    previous, sys.dont_write_bytecode = sys.dont_write_bytecode, True
    try:
        spec.loader.exec_module(mod)
    finally:
        sys.dont_write_bytecode = previous
    return mod


_hysteresis_tool = _load_tool('status_hysteresis_check', 'spec_map_tracker/status_hysteresis_check.py')
_runtime_tool = _load_tool('runtime_checks', 'spec_vio_tracker/runtime_checks.py')


def first_index(rows, pred):
    r = next((r for r in rows if pred(r)), None)
    return None if r is None else r['i']


def first_translation(rows, origin=ORIGIN_RESPONSIVE):
    """V-EV1: first fresh posed frame whose position differs from the configured origin."""
    return first_index(posed_rows(rows), lambda r: r['p'] != origin)


def first_limited(rows):
    """V-EV2 / MT-P7 ③: first fresh frame with status LIMITED (index and fixture time)."""
    r = next((r for r in fresh_rows(rows) if r['status'] == 'LIMITED'), None)
    return None if r is None else {'i': r['i'], 't': r['t']}


def limited_ratio(rows):
    """MT-P7 ②: LIMITED rows / input frames (trace rows), D-1 denominator."""
    return sum(r.get('status') == 'LIMITED' for r in rows) / len(rows)


def world_points_p50(rows):
    """V-EV5: p50 of the public world point count over fresh frames (run-reference.mjs stats)."""
    return percentile([r['worldPoints'] for r in fresh_rows(rows)], 50, 'floor_index')


def first_world_points(rows):
    """G-M1 / G-M6 proxy for the first MAP_RESET frame (S3 §8.3-8.4): first fresh frame with world
    points, and its public count."""
    r = next((r for r in fresh_rows(rows) if (r.get('worldPoints') or 0) > 0), None)
    return None if r is None else {'i': r['i'], 'count': r['worldPoints']}


def early_burst_share_5_6(rows, burst=10, horizon=200):
    """G-M2 proxy (S3 §8.3, cadence.py definitions): bursts = fresh frames with wpAdded >= 10, in
    processed-frame units; intervals of bursts within the first `horizon` processed frames from the
    first world-points frame; share of intervals equal to 5 or 6."""
    fresh = fresh_rows(rows)
    k_wp = next((k for k, r in enumerate(fresh) if (r.get('worldPoints') or 0) > 0), None)
    if k_wp is None:
        return {'intervals': [], 'share_5_6': None}
    bursts = [k for k, r in enumerate(fresh) if (r.get('wpAdded') or 0) >= burst and k_wp <= k < k_wp + horizon]
    iv = [b - a for a, b in zip(bursts, bursts[1:])]
    return {'intervals': iv, 'share_5_6': (sum(v in (5, 6) for v in iv) / len(iv)) if iv else None}


def zero_step_fractions(rows):
    """V-EV4: runtime_checks P7 = fraction of consecutive fresh posed frames (i, i+1) with exactly
    equal position, keyed by the status of the later frame (same loop as runtime_checks.check_run,
    which only takes a path)."""
    fresh = posed_rows(rows)
    zero = {}
    for a, b in zip(fresh, fresh[1:]):
        if b['i'] != a['i'] + 1:
            continue
        z = zero.setdefault(b['status'], [0, 0])
        z[1] += 1
        if a['p'] == b['p']:
            z[0] += 1
    return {k: v[0] / v[1] for k, v in sorted(zero.items())}


def publish_lag_spread_ms(rows):
    """MT-P7 ③ / V-H7 tag rule (D-20, M8): p95 - p5 of publishMs - 1000*t (floor-index percentiles as
    run-reference.mjs), with the median frame interval in ms. Null for engine traces."""
    lags = [r['publishMs'] - 1000.0 * r['t'] for r in rows if r.get('publishMs') is not None]
    if not lags:
        return None
    p5, p95 = percentile(lags, 5, 'floor_index'), percentile(lags, 95, 'floor_index')
    return {'p5': p5, 'p95': p95, 'spread': p95 - p5, 'frame_interval_ms': 1000.0 * frame_interval_s(rows)}


def worldpoint_padding_fraction(trace_path):
    """V-P8: share of sampled world points equal to [id 0, conf 0, (0, 2, 0)] (runtime_checks P8)."""
    wp = pathlib.Path(trace_path).parent / 'worldpoints-sampled.jsonl'
    if not wp.exists():
        return None
    tot = pad = 0
    with open(wp) as f:
        for line in f:
            pts = json.loads(line)['points']
            tot += len(pts)
            pad += sum(1 for x in pts if x == [0, 0, 0, 2, 0])
    return {'points': tot, 'padding': pad, 'fraction': pad / tot if tot else None}


def status_hysteresis(trace_path):
    """MT-P2t / MT-P7 ①: the lane tool verbatim (interior runs >= 1.9 s, first LIMITED >= 1.9 s)."""
    r = _hysteresis_tool.check_run(str(trace_path))
    return {'pass': r['pass'], 'violations': r['violations'], 'first_limited_ok': r['first_limited_ok'],
            'min_interior_run_s': r['min_interior_run_s'], 'n_runs': r['n_runs']}


def runtime_invariants(trace_path):
    """V-P1..V-P6: the lane tool verbatim (TOL 0.15 s)."""
    r = _runtime_tool.check_run(str(trace_path))
    heads = {d['i']: d['heading_pitch'][0] for d in r['P6_heading_pitch_deg_first']}
    return {'P1_violations': r['P1_violations'], 'P2_violations': r['P2_violations'], 'P3_ok': r['P3_ok'],
            'P3_first_limited': r['P3_first_limited'], 'P4_ok': r['P4_ok'], 'P4_reasons': r['P4_reasons'],
            'P5_ok': r['P5_ok'], 'P6_heading_deg': {str(i): heads.get(i) for i in (0, 6)},
            'first_translation': r['first_translation']}


def scale_latches(rows):
    """MS-RT (absolute runs): latch events = fresh frames whose reason leaves INITIALIZING (S4 §3.9,
    §6.2: reason INITIALIZING until the MSE scale latches; map reset unlatches). For each latch the
    scale proxy s_hat = p_y(latch frame) / p_y(previous fresh frame). INFERRED from S4 §3.9: the latch
    frame output is y' = s*(y+1) - 1 in internal units and the absolute glue publishes y + 1 (origin.y
    = 1, scale = 1), so published y' = s * published y when the camera moves little between the two
    frames."""
    fresh = posed_rows(rows)
    out = []
    for a, b in zip(fresh, fresh[1:]):
        if a['reason'] == 'INITIALIZING' and b['reason'] != 'INITIALIZING':
            s_hat = b['p'][1] / a['p'][1] if a['p'][1] != 0 else None
            out.append({'i': b['i'], 't': b['t'], 'prev_i': a['i'], 'y_prev': a['p'][1], 'y': b['p'][1], 'scale_proxy': s_hat})
    return out


def engine_keyframe_intervals(rows):
    """G-M2 (a) engine-only: processed-frame intervals between consecutive srKeyframeCreated frames,
    excluding intervals that end on a srMapReset frame (reset => keyframe on the same frame, S3 §8.2)."""
    fresh = fresh_rows(rows)
    kf = [(k, bool(r.get('srMapReset'))) for k, r in enumerate(fresh) if r.get('srKeyframeCreated')]
    return [b - a for (a, _), (b, reset) in zip(kf, kf[1:]) if not reset]

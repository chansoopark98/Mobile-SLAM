"""Shared trace loading, schema and statistics for the slam_refact parity evaluators (WP5).

Trace schema = the original headless reference runtime reference trace written by
slam_refact/tools/re/runtime/run-reference.mjs (one JSON object per input frame). Engine traces
written by the replay runner (tools/harness/replay) carry the same keys in the same order with
the same meaning; browser-only timing keys are null; engine-only debug keys start with "sr"
(architecture.md §6.3).

Statistics: three percentile conventions exist in the reused tools and each gate names the one its
source used (thresholds-v1.json `percentile`):
  * "nearest_rank": sorted[ceil(p/100*n) - 1]       (spec_vio_tracker/reference_variability.py)
  * "floor_index":  sorted[floor((n-1)*p)]           (runtime/run-reference.mjs summary stats)
  * "linear":       numpy.percentile default         (runtime/analyze_reference.py)
"""
import hashlib
import json
import math
import pathlib

import numpy as np

REPO_ROOT = pathlib.Path(__file__).resolve().parents[4]
EVIDENCE_ROOT = REPO_ROOT / 'build/slam-refact-20261015'
REFERENCE_RUNS = EVIDENCE_ROOT / 'reference/runs'
THRESHOLDS_PATH = REPO_ROOT / 'slam_refact/tools/harness/thresholds-v1.json'

# Key order of run-reference.mjs trace rows (VERIFIED against reference/runs/*/r1/trace.jsonl).
REFERENCE_FIELDS = ('i', 't', 'videoTimeUs', 'publishMs', 'acceptedMs', 'fresh', 'callbackMs', 'latencyMs',
                    'status', 'reason', 'p', 'q_wxyz', 'worldPoints', 'wpAdded', 'wpRemoved', 'wpMeanConfidence',
                    'xrGpuStageMs', 'xrCpuStageMs')
# Browser timing keys: null in engine traces (no browser).
BROWSER_ONLY_FIELDS = ('publishMs', 'acceptedMs', 'callbackMs', 'latencyMs', 'xrGpuStageMs', 'xrCpuStageMs')
STATUS_VALUES = ('UNSPECIFIED', 'NOT_AVAILABLE', 'LIMITED', 'NORMAL')
REASON_VALUES = ('UNSPECIFIED', 'INITIALIZING', 'RELOCALIZING', 'TOO_MUCH_MOTION', 'NOT_ENOUGH_TEXTURE')
ORIGIN_RESPONSIVE = [0, 2, 0]

# Reference scenarios (scenarios.json) and the runs that count as original references
# (wrap1 is instrumented and excluded everywhere: 03-runtime-reference.md §9.2; profile1 is a
# plain run with a CPU profiler attached and is used only where the design cites it, A-*).
SCENARIOS = ('tum-room1-full', 'tum-room1-lowtex', 'flip6-s2', 'flip6-n1', 'flip6-s3-lowtex')
REFERENCE_RUN_LABELS = ('r1', 'r2')


def sha256_file(path):
    h = hashlib.sha256()
    with open(path, 'rb') as f:
        for chunk in iter(lambda: f.read(1 << 20), b''):
            h.update(chunk)
    return h.hexdigest()


def load_trace(path):
    with open(path) as f:
        return [json.loads(line) for line in f if line.strip()]


def reference_trace_path(scenario, label):
    return REFERENCE_RUNS / scenario / label / 'trace.jsonl'


def validate_row(row, engine):
    """Returns a list of schema problems of one trace row (empty = valid)."""
    problems = []
    keys = list(row.keys())
    base = keys[:len(REFERENCE_FIELDS)]
    if tuple(base) != REFERENCE_FIELDS:
        problems.append('reference keys/order differ: %s' % base)
    extra = keys[len(REFERENCE_FIELDS):]
    if not engine and extra:
        problems.append('unexpected extra keys in reference row: %s' % extra)
    if engine:
        bad = [k for k in extra if not k.startswith('sr')]
        if bad:
            problems.append('engine extra keys must start with "sr": %s' % bad)
        for k in BROWSER_ONLY_FIELDS:
            if row.get(k) is not None:
                problems.append('browser-only key %s must be null in engine traces' % k)
    if not isinstance(row.get('i'), int):
        problems.append('i must be int')
    if not isinstance(row.get('fresh'), bool):
        problems.append('fresh must be bool')
    if row.get('fresh'):
        if row.get('status') not in STATUS_VALUES:
            problems.append('status %r' % row.get('status'))
        if row.get('reason') not in REASON_VALUES:
            problems.append('reason %r' % row.get('reason'))
        p, q = row.get('p'), row.get('q_wxyz')
        if (p is None) != (q is None):
            problems.append('p and q_wxyz must both be null or both set')
        if p is not None and (len(p) != 3 or len(q) != 4):
            problems.append('p/q_wxyz length')
    else:
        for k in ('status', 'reason', 'p', 'q_wxyz', 'worldPoints'):
            if row.get(k) is not None:
                problems.append('non-fresh row has %s' % k)
    return problems


def validate_trace(rows, engine):
    problems = []
    for n, row in enumerate(rows):
        if row.get('i') != n:
            problems.append('row %d: i=%r (rows must be input frames 0..N-1 in order)' % (n, row.get('i')))
            break
        for msg in validate_row(row, engine):
            problems.append('row %d: %s' % (n, msg))
            if len(problems) > 20:
                return problems
    return problems


def fresh_rows(rows):
    return [r for r in rows if r.get('fresh')]


def posed_rows(rows):
    return [r for r in rows if r.get('fresh') and r.get('p') is not None]


# ---------------------------------------------------------------------------------- statistics
def percentile(values, p, method):
    xs = sorted(v for v in values if v is not None and math.isfinite(v))
    if not xs:
        return None
    n = len(xs)
    if method == 'nearest_rank':
        return xs[min(n - 1, max(0, int(math.ceil(p / 100.0 * n)) - 1))]
    if method == 'floor_index':
        return xs[min(n - 1, int(math.floor((n - 1) * p / 100.0)))]
    if method == 'linear':
        return float(np.percentile(np.asarray(xs, float), p))
    raise ValueError('unknown percentile method ' + method)


def summary(values, method='linear'):
    xs = [v for v in values if v is not None and math.isfinite(v)]
    if not xs:
        return None
    a = np.asarray(xs, float)
    return {'n': len(xs), 'mean': float(a.mean()), 'rmse': float(np.sqrt(np.mean(a ** 2))),
            'p50': percentile(xs, 50, method), 'p95': percentile(xs, 95, method), 'max': float(a.max())}


# ---------------------------------------------------------------------------------- rotations
def quat_wxyz_to_matrix(q):
    """Unit rotation matrix of a (w, x, y, z) quaternion (normalised first, like scipy)."""
    w, x, y, z = (float(v) for v in q)
    n = math.sqrt(w * w + x * x + y * y + z * z)
    w, x, y, z = w / n, x / n, y / n, z / n
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def rotation_angle_deg(r):
    """Geodesic angle of a rotation matrix (atan2 form: accurate near 0 and pi)."""
    s = 0.5 * math.sqrt((r[2, 1] - r[1, 2]) ** 2 + (r[0, 2] - r[2, 0]) ** 2 + (r[1, 0] - r[0, 1]) ** 2)
    c = 0.5 * (r[0, 0] + r[1, 1] + r[2, 2] - 1.0)
    return math.degrees(math.atan2(s, c))


def quat_angle_deg(q1, q2):
    """Angle between two (w, x, y, z) rotations; the formula of reference_variability.py."""
    d = abs(sum(a * b for a, b in zip(q1, q2)))
    return math.degrees(2 * math.acos(min(1.0, d)))


def frame_interval_s(rows):
    ts = [r['t'] for r in rows]
    dts = sorted(b - a for a, b in zip(ts, ts[1:]))
    return dts[len(dts) // 2] if dts else None

#!/usr/bin/env python3
"""Machine evaluation of the preregistered XR commercialization gate (G0..G6).

The preregistration (build/xr-commercial-<date>/preregistration.json) is the specification: run
names, metric definitions, limits and gate rules come from it, never from this file.

subcommands (run from the repository root; relative paths inside the prereg are repo-relative):
  collect   --prereg P --runs ROOT --config NAME --e0-runs ROOT --out metrics.json
  identity  --e0-runs A --runs B --out identity.json
  timing    --prereg P --runs TIMING_ROOT --out timing.json
  verdict   --prereg P --e0 E0.json --metrics M.json --config NAME --g0 g0.json
            --freeze freeze.json --attempts attempts.jsonl
            [--timing timing.json] [--compare other-metrics.json] --out verdict.json
  summary   --e0 E0.json --metrics a.json [b.json ...]
  freeze    --prereg P --e0 E0.json --tools FILE ... --out freeze.json [--addendum A ...]
  freeze-add --freeze freeze.json --addendum A --out freeze-2.json

Every output file is created exclusively: an existing file is never overwritten.
"""
import argparse
import copy
import datetime
import hashlib
import json
import math
import pathlib
import statistics as st
import sys

HERE = pathlib.Path(__file__).resolve().parent
HARNESS = HERE.parent
REPO = HARNESS.parents[2]
sys.path.insert(0, str(HARNESS))
sys.path.insert(0, str(HARNESS / 'eval'))

from outback import mirror_eval  # noqa: E402
from outback.speed_eval import speed_error  # noqa: E402

METRICS_SCHEMA = 'xr-commercial-metrics-v1'
VERDICT_SCHEMA = 'xr-commercial-verdict-v1'
ACCURACY_KEYS = ('sim3AteRmse', 'apeRotDegRmse', 'rpe1sTransSim3Rmse', 'rpe1sRotDegRmse')
META_IGNORED_KEYS = ('contractVersion', 'wallSeconds')


# ---------------------------------------------------------------- small helpers

def sha256_file(path):
    return hashlib.sha256(pathlib.Path(path).read_bytes()).hexdigest()


def repo_path(p):
    """A prereg path: absolute as given, relative ones against the repository root."""
    p = pathlib.Path(p)
    return p if p.is_absolute() else REPO / p


def write_json_exclusive(path, obj):
    with open(path, 'x') as f:
        f.write(json.dumps(obj, indent=1) + '\n')


def load_json(path):
    return json.loads(pathlib.Path(path).read_text())


def load_rows(run_dir):
    with open(pathlib.Path(run_dir) / 'trace.jsonl') as f:
        return [json.loads(line) for line in f if line.strip()]


def strip_stage(rows):
    """Rows without the timing key srStageMs (the only key allowed to differ between runs)."""
    return [{k: v for k, v in r.items() if k != 'srStageMs'} for r in rows]


def stage(row):
    return row.get('srStageMs') or {}


def percentile_linear(values, q):
    """numpy.percentile default (linear interpolation between closest ranks); q in 0..1."""
    s = sorted(values)
    if not s:
        return None
    h = (len(s) - 1) * q
    lo, hi = math.floor(h), math.ceil(h)
    return s[lo] + (h - lo) * (s[hi] - s[lo])


def add_parser_output(ap):
    ap.add_argument('--out', required=True)


# ---------------------------------------------------------------- collect

def read_status(runs_root):
    """name -> status record of <runs_root>/status.jsonl.

    A later line replaces an earlier one, except that an 'exists' line (the replay was skipped
    because its directory was already there) never hides a real done/failed record.
    """
    path = pathlib.Path(runs_root) / 'status.jsonl'
    out = {}
    if not path.is_file():
        return out
    for line in path.read_text().splitlines():
        if not line.strip():
            continue
        rec = json.loads(line)
        old = out.get(rec['name'])
        if old is not None and rec.get('status') == 'exists' and old.get('status') != 'exists':
            continue
        out[rec['name']] = rec
    return out


def run_names(prereg):
    """(name, kind) of every run of the prereg, in prereg order, each name once."""
    seq = prereg['sequences']
    kinds = [('outback', seq['outback']), ('forwardSpeed', list(seq['forwardSpeed'])),
             ('fixtures', list(seq['fixtures'])), ('chromeForward', seq['chromeForward'])]
    seen, out = set(), []
    for kind, names in kinds:
        for name in names:
            if name not in seen:
                seen.add(name)
                out.append((name, kind))
    return out


def common_metrics(rows):
    return {
        'rows': len(rows),
        'resets': sum(1 for r in rows if r.get('srMapReset')),
        'stall': sum(1 for r in rows if stage(r).get('mapTracker', 0) >= 1000
                     or stage(r).get('mapBuilder', 0) >= 200),
        'normalRatio': (sum(1 for r in rows if r.get('status') == 'NORMAL') / len(rows)
                        if rows else None),
        # Same definition as mirror_eval: rows after the first one without a tracked map.
        'mapLost': sum(1 for r in rows[1:] if r.get('srMapTrackerState') != 1),
    }


def longest_non_normal_run(rows):
    """Longest run of consecutive rows whose status is not NORMAL or that have no pose."""
    best = cur = 0
    for r in rows:
        cur = cur + 1 if (r.get('status') != 'NORMAL' or not r.get('p')) else 0
        best = max(best, cur)
    return best


def collect_run(prereg, name, kind, runs_root, e0_runs, status, scenarios):
    run = pathlib.Path(runs_root) / name
    rec = {'kind': kind, 'missing': not (run / 'trace.jsonl').is_file()}
    st_rec = status.get(name) or {}
    rec['exit'] = st_rec.get('exit')
    rec['status'] = st_rec.get('status')
    rec['replaySha256'] = st_rec.get('replaySha256')
    rec['accuracyEnv'] = st_rec.get('accuracy')  # the SLAM_REFACT_ACCURACY string of the run
    rec['accuracy'] = rec['accuracyEnv']  # replaced by the accuracy dict for fixtures below
    if rec['missing']:
        return rec
    try:
        rows = load_rows(run)
    except ValueError:  # a run still being written has a truncated last line
        rec.update({'missing': True, 'unreadable': True})
        return rec
    rec.update(common_metrics(rows))
    rec['discards'] = max(rec['resets'] - 1, 0)
    e0_trace = pathlib.Path(e0_runs) / name / 'trace.jsonl'
    rec['identicalToE0'] = (strip_stage(rows) == strip_stage(load_rows(e0_trace.parent))
                            if e0_trace.is_file() else None)
    limits = prereg['limits']
    seq = prereg['sequences']
    if kind == 'outback':
        rec.update(mirror_eval.evaluate_run(run, repo_path(prereg['bundles']) / name))
    elif kind == 'forwardSpeed':
        spec = seq['forwardSpeed'][name]
        try:
            rec.update(speed_error(run, pathlib.Path(e0_runs) / spec['reference'], spec['k'],
                                   limits['speedFirstRow']))
        except (OSError, ValueError):  # reference run absent or still being written: eSpeed unknown
            rec.update({'eSpeed': None, 'eSpeedP90': None, 'pairs': None, 'expectedPairs': None,
                        'complete': None, 'referenceMissing': True})
    else:
        import accuracy_matrix  # lazy: needs numpy
        if kind == 'fixtures':
            scenario = seq['fixtures'][name]
        else:  # chromeForward: the scenario is the first part of the bundleOf entry
            scenario = prereg['bundleOf'][name].split('/')[0]
        res = accuracy_matrix.evaluate(scenario, run, scenarios)
        rec['longestNonNormalRun'] = longest_non_normal_run(rows)
        rec['trajectory'] = res.get('trajectory')
        rec['normalRatio'] = (res.get('trajectory') or {}).get('normalRatio')
        if kind == 'fixtures':
            if 'accuracy' in res:
                rec['accuracy'] = res['accuracy']
            else:
                rec['accuracyMissing'] = True
    return rec


def unique_shared(values, what):
    found = {v for v in values if v is not None}
    if len(found) > 1:
        raise SystemExit(f'error: runs disagree on {what}: {sorted(found)}')
    return next(iter(found), None)


def cmd_collect(args):
    if pathlib.Path(args.out).exists():
        raise SystemExit(f'error: {args.out} exists; refusing to overwrite')
    prereg = load_json(args.prereg)
    scenarios = load_json(repo_path('slam_refact/tools/re/runtime/scenarios.json'))
    status = read_status(args.runs)
    runs = {name: collect_run(prereg, name, kind, args.runs, args.e0_runs, status, scenarios)
            for name, kind in run_names(prereg)}
    env = unique_shared([r['accuracyEnv'] for r in runs.values()], 'the accuracy env')
    sha = unique_shared([r['replaySha256'] for r in runs.values()], 'the replay binary')
    out = {'schema': METRICS_SCHEMA, 'config': args.config, 'accuracyEnv': env,
           'replaySha256': sha, 'runs': runs}
    write_json_exclusive(args.out, out)
    missing = [n for n, r in runs.items() if r['missing']]
    print(f'collected {len(runs) - len(missing)}/{len(runs)} runs; missing: {missing or "none"}')
    return 0


# ---------------------------------------------------------------- identity

def cmd_identity(args):
    if pathlib.Path(args.out).exists():
        raise SystemExit(f'error: {args.out} exists; refusing to overwrite')
    a_root, b_root = pathlib.Path(args.e0_runs), pathlib.Path(args.runs)
    results, ok = {}, True
    for a in sorted(p for p in a_root.iterdir() if (p / 'trace.jsonl').is_file()):
        b = b_root / a.name
        if not (b / 'trace.jsonl').is_file():
            results[a.name] = {'rows': None, 'differingRows': None, 'firstDifferingRow': None,
                               'metaDiffKeys': None, 'pass': False, 'missing': True}
            ok = False
            continue
        ra, rb = strip_stage(load_rows(a)), strip_stage(load_rows(b))
        diff = [i for i in range(min(len(ra), len(rb))) if ra[i] != rb[i]]
        diff_rows = len(diff) + abs(len(ra) - len(rb))
        first = diff[0] if diff else (min(len(ra), len(rb)) if len(ra) != len(rb) else None)
        ma, mb = load_json(a / 'meta.json'), load_json(b / 'meta.json')
        meta_diff = sorted(k for k in set(ma) | set(mb)
                           if k not in META_IGNORED_KEYS and ma.get(k) != mb.get(k))
        passed = diff_rows == 0 and not meta_diff
        results[a.name] = {'rows': [len(ra), len(rb)], 'differingRows': diff_rows,
                           'firstDifferingRow': first, 'metaDiffKeys': meta_diff, 'pass': passed}
        ok = ok and passed
    # The binary that produced B: the common replaySha256 of B's status lines (None without them).
    status_file = b_root / 'status.jsonl'
    shas = ([json.loads(l).get('replaySha256') for l in status_file.read_text().splitlines() if l.strip()]
            if status_file.is_file() else [])
    out = {'schema': 'xr-commercial-identity-v1', 'e0Runs': str(a_root), 'runs': str(b_root),
           'replaySha256': unique_shared(shas, 'the replay binary'),
           'statusSha256': sha256_file(status_file) if status_file.is_file() else None,
           'results': results, 'pass': ok and bool(results)}
    write_json_exclusive(args.out, out)
    bad = [n for n, r in results.items() if not r['pass']]
    print(f'identity: {len(results) - len(bad)}/{len(results)} runs identical; differing: {bad or "none"}')
    return 0 if out['pass'] else 1


# ---------------------------------------------------------------- timing

def run_p95(run_dir, first_row, tracked_only=False):
    """p95 over rows i >= first_row of mapTracker + mapBuilder stage time, or None.

    tracked_only (prereg sequences.timing.<seq>.rows == 'tracked'): only rows whose map tracker
    state is 1; the map-lost rows are reported separately by lost_frame_cost."""
    vals = [stage(r).get('mapTracker', 0) + stage(r).get('mapBuilder', 0)
            for r in load_rows(run_dir)
            if r['i'] >= first_row and (not tracked_only or r.get('srMapTrackerState') == 1)]
    return percentile_linear(vals, 0.95)


def lost_frame_cost(run_dir, first_row):
    """Report only: mapTracker + mapBuilder time and map size of the map-lost rows i >= first_row."""
    rows = [r for r in load_rows(run_dir) if r['i'] >= first_row and r.get('srMapTrackerState') != 1]
    vals = [stage(r).get('mapTracker', 0) + stage(r).get('mapBuilder', 0) for r in rows]
    points = [sum(r.get('srPointsBySource') or []) for r in rows]
    return {'frames': len(rows), 'p50': percentile_linear(vals, 0.5), 'p95': percentile_linear(vals, 0.95),
            'max': max(vals) if vals else None, 'mapPointsP50': percentile_linear(points, 0.5),
            'mapPointsMax': max(points) if points else None}


def cmd_timing(args):
    if pathlib.Path(args.out).exists():
        raise SystemExit(f'error: {args.out} exists; refusing to overwrite')
    prereg = load_json(args.prereg)
    lim, root = prereg['limits'], pathlib.Path(args.runs)
    reps = lim['timingRepetitions']
    status_path = root / 'status.jsonl'
    status_lines = ([json.loads(l) for l in status_path.read_text().splitlines() if l.strip()]
                    if status_path.is_file() else [])
    by_run = {f"{r['sequence']}/{r['rep']}-{r['arm']}": r for r in status_lines}
    sequences, missing, bad_runs, unknown_exit = {}, [], [], []
    for seq, spec in prereg['sequences']['timing'].items():
        tracked_only = spec.get('rows') == 'tracked'
        sequences[seq] = {'rows': 'tracked' if tracked_only else 'all'}
        for arm in ('off', 'on'):
            values, lost, lost_p95 = [], [], []
            for rep in range(1, reps + 1):
                run = root / seq / f'{rep}-{arm}'
                if (run / 'trace.jsonl').is_file():
                    values.append(run_p95(run, lim['timingFirstRow'], tracked_only))
                    lost.append(lost_frame_cost(run, lim['timingFirstRow']))
                    lost_p95.append(lost[-1]['p95'] or 0.0)  # no lost frame costs 0 ms
                    name = f'{seq}/{rep}-{arm}'
                    code = (by_run.get(name) or {}).get('exit')
                    if code is None:
                        unknown_exit.append(name)
                    elif code != 0:
                        bad_runs.append(f'{name}: exit {code}')
                    stall = sum(1 for r in load_rows(run) if stage(r).get('mapTracker', 0) >= 1000
                                or stage(r).get('mapBuilder', 0) >= 200)
                    if stall:
                        bad_runs.append(f'{name}: stall {stall}')
                else:
                    missing.append(f'{seq}/{rep}-{arm}')
            sequences[seq][arm] = {'values': values,
                                   'median': st.median(values) if len(values) == reps else None,
                                   'lostFrames': lost,
                                   'lostP95Median': st.median(lost_p95) if len(lost_p95) == reps else None}
    load1 = {f"{r['sequence']}/{r['rep']}-{r['arm']}": [r.get('load1Start'), r.get('load1End')]
             for r in status_lines}
    sha = unique_shared([r.get('replaySha256') for r in status_lines], 'the replay binary')
    acc = {arm: unique_shared([r.get('accuracy') for r in status_lines if r['arm'] == arm],
                              f'the accuracy env of the {arm} arm') for arm in ('off', 'on')}
    write_json_exclusive(args.out, {'schema': 'xr-commercial-timing-v1', 'replaySha256': sha,
                                    'accuracyOff': acc['off'], 'accuracyOn': acc['on'],
                                    'sequences': sequences, 'missing': missing, 'badRuns': bad_runs,
                                    'unknownExit': unknown_exit, 'load1': load1})
    print(f'timing: {len(sequences)} sequences, missing runs: {missing or "none"}')
    return 0


# ---------------------------------------------------------------- verdict

BLOCKED, FAIL, PASS, NA = 'BLOCKED', 'FAIL', 'PASS', 'NOT_APPLICABLE'


def need(lim, key):
    """A limit the prereg must define; a clear error instead of a KeyError traceback."""
    if key not in lim:
        raise SystemExit(f'error: prereg limits has no {key!r}')
    return lim[key]


def ratio_floor(e0, ratio, floor):
    """(limit, bound) of max(ratio * e0, e0 + floor); bound names the larger term."""
    if e0 is None:
        return None, None
    by_ratio, by_floor = ratio * e0, e0 + floor
    return (by_ratio, 'ratio') if by_ratio >= by_floor else (by_floor, 'floor')


def limit_ratio_or_floor(e0, lim):
    return ratio_floor(e0, lim['ratio'], lim['absFloor'])


class Gate:
    """Collects the checks of one gate; the status is BLOCKED > FAIL > PASS."""

    def __init__(self):
        self.checks = []

    def check(self, sequence, metric, value, e0, limit, op, bound=None):
        """One comparison; a missing value, E0 or limit makes it blocked (pass None).

        bound ('ratio' | 'floor') records which term of a max(ratio, floor) limit applied."""
        if value is None or limit is None:
            ok = None
        elif op == '<=':
            ok = value <= limit
        elif op == '>=':
            ok = value >= limit
        else:
            ok = value == limit
        rec = {'sequence': sequence, 'metric': metric, 'value': value, 'e0': e0,
               'limit': limit, 'op': op, 'pass': ok}
        if bound is not None:
            rec['bound'] = bound
        self.checks.append(rec)

    def check_floor(self, sequence, metric, value, e0, ratio, floor):
        """value <= max(ratio * e0, e0 + floor)."""
        limit, bound = ratio_floor(e0, ratio, floor)
        self.check(sequence, metric, value, e0, limit, '<=', bound)

    def check_rows_mapl(self, seq, c, b, lim):
        """rows == E0.rows (a truncated run must not pass) and the map-lost frame bound."""
        self.check(seq, 'rows', get(c, 'rows'), get(b, 'rows'), get(b, 'rows'), '==')
        self.check_floor(seq, 'mapLost', get(c, 'mapLost'), get(b, 'mapLost'), lim['ratio'],
                         need(lim, 'mapLostFloorFrames'))

    def block(self, sequence, metric, reason):
        self.checks.append({'sequence': sequence, 'metric': metric, 'value': None, 'e0': None,
                            'limit': None, 'op': None, 'pass': None, 'reason': reason})

    def run_ok(self, sequence, run):
        """exit == 0 and stall == 0 of a run.

        A missing run, trace, exit status or stall count is BLOCKED. A run that exists with
        exit != 0 or stall > 0 is a FAIL: E0 has exit 0 / stall 0 everywhere, so the candidate
        caused it (an infrastructure rerun goes into a new runs directory).
        Returns True when the metric checks of the run may follow."""
        if run is None or run.get('missing'):
            self.block(sequence, 'run', 'run missing')
            return False
        if run.get('exit') is None:
            self.block(sequence, 'exit', 'no exit status recorded')
            return False
        if run['exit'] != 0:
            self.check(sequence, 'exit', run['exit'], 0, 0, '==')
            return False
        stall = run.get('stall')
        if stall is None:
            self.block(sequence, 'stall', 'no stall count')
            return False
        if stall > 0:
            self.check(sequence, 'stall', stall, 0, 0, '==')
            return False
        self.checks.append({'sequence': sequence, 'metric': 'exit/stall', 'value': 0, 'e0': None,
                            'limit': 0, 'op': '==', 'pass': True})
        return True

    @property
    def status(self):
        # Informational checks (G6 per-sequence rows) never decide the status.
        results = [c['pass'] for c in self.checks if not c.get('informational')]
        if not results or any(r is None for r in results):
            return BLOCKED
        return PASS if all(results) else FAIL

    def as_dict(self):
        return {'status': self.status, 'checks': self.checks}


def get(run, key):
    return None if run is None else run.get(key)


def gate_g0(g0):
    """pass + evidence file that exists + its sha256 for each entry (identity also replaySha256)."""
    g = Gate()
    for key in ('identity', 'rs0Fast', 'refactNative', 'unitTests'):
        entry = (g0 or {}).get(key)
        if not isinstance(entry, dict) or 'pass' not in entry:
            g.block(key, 'pass', 'missing in g0.json')
            continue
        problem = None
        ev = entry.get('evidence')
        if not ev or not repo_path(ev).is_file():
            problem = 'evidence file missing'
        elif not entry.get('sha256') or sha256_file(repo_path(ev)) != entry['sha256']:
            problem = 'evidence sha256 missing or wrong'
        elif key == 'identity' and not entry.get('replaySha256'):
            problem = 'identity entry has no replaySha256'
        if problem:
            g.block(key, 'evidence', problem)
            continue
        g.checks.append({'sequence': key, 'metric': 'pass', 'value': entry['pass'], 'e0': None,
                         'limit': True, 'op': '==', 'pass': entry['pass'] is True,
                         'evidence': ev})
    return g


def gate_g1(prereg, e0, cand):
    g, lim = Gate(), prereg['limits']
    for seq in prereg['sequences']['primary']:
        c, b = cand['runs'].get(seq), e0['runs'].get(seq)
        if not g.run_ok(seq, c):
            continue
        base = get(b, 'eStart')
        g.check(seq, 'eStart', get(c, 'eStart'), base,
                None if base is None else lim['primaryRatio'] * base, '<=')
    return g


def gate_g2(prereg, e0, cand):
    g, lim = Gate(), prereg['limits']
    for seq in prereg['sequences']['outback']:
        c, b = cand['runs'].get(seq), e0['runs'].get(seq)
        if not g.run_ok(seq, c):
            continue
        g.check(seq, 'eStartComplete', get(c, 'eStartComplete'), None, True, '==')
        for key in ('eStart', 'eAll'):
            g.check_floor(seq, key, get(c, key), get(b, key), lim['ratio'], lim['absFloor'])
        g.check(seq, 'discards', get(c, 'discards'), get(b, 'discards'), get(b, 'discards'), '<=')
        base = get(b, 'normalRatio')
        g.check(seq, 'normalRatio', get(c, 'normalRatio'), base,
                None if base is None else base - lim['coverageLossMax'], '>=')
        g.check_rows_mapl(seq, c, b, lim)
        # Degenerate candidates: a frozen pose gives a small E_start but also a short path / no rotation.
        base = get(b, 'path')
        g.check(seq, 'path', get(c, 'path'), base,
                None if base is None else need(lim, 'pathMinRatio') * base, '>=')
        g.check_floor(seq, 'rotStartDeg', get(c, 'rotStartDeg'), get(b, 'rotStartDeg'), lim['ratio'],
                      need(lim, 'rotFloorDeg'))
    return g


def gate_g3(prereg, e0, cand):
    g, lim = Gate(), prereg['limits']
    for seq in prereg['sequences']['forwardSpeed']:
        c, b = cand['runs'].get(seq), e0['runs'].get(seq)
        if not g.run_ok(seq, c):
            continue
        g.check_floor(seq, 'eSpeed', get(c, 'eSpeed'), get(b, 'eSpeed'), lim['ratio'], lim['absFloor'])
        g.check(seq, 'complete', get(c, 'complete'), None, True, '==')
        g.check_rows_mapl(seq, c, b, lim)
    return g


def _normal_and_resets(g, seq, c, b, lim):
    base = get(b, 'normalRatio')
    g.check(seq, 'normalRatio', get(c, 'normalRatio'), base,
            None if base is None else base - lim['coverageLossMax'], '>=')
    g.check(seq, 'resets', get(c, 'resets'), get(b, 'resets'), get(b, 'resets'), '<=')


def gate_g4(prereg, e0, cand):
    g, lim = Gate(), prereg['limits']
    gate_keys = prereg['sequences'].get('fixtureGateKeys') or {}
    names = list(prereg['sequences']['fixtures']) + list(prereg['sequences']['chromeForward'])
    for seq in names:
        c, b = cand['runs'].get(seq), e0['runs'].get(seq)
        if not g.run_ok(seq, c):
            continue
        if c.get('identicalToE0') is True:
            g.checks.append({'sequence': seq, 'metric': 'identicalToE0', 'value': True, 'e0': None,
                             'limit': True, 'op': '==', 'pass': True, 'identity': True})
            continue
        if c['kind'] == 'fixtures':
            ca, ba = c.get('accuracy') or {}, (b or {}).get('accuracy') or {}
            # sequences.fixtureGateKeys names the gated accuracy keys of a fixture (TUM: RPE only,
            # like the earlier G6); fixtures not listed there are gated on every key.
            for key in gate_keys.get(seq, ACCURACY_KEYS):
                base = ba.get(key)
                g.check(seq, key, ca.get(key), base, None if base is None else lim['ratio'] * base, '<=')
            base = ba.get('coverage')
            g.check(seq, 'coverage', ca.get('coverage'), base,
                    None if base is None else base - lim['coverageLossMax'], '>=')
        _normal_and_resets(g, seq, c, b, lim)
        g.check_rows_mapl(seq, c, b, lim)
        # TUM fixtures (listed in fixtureGateKeys) and the chromeForward runs: no longer gap of
        # non-NORMAL / pose-less rows than E0 (an E0 value of 0 gives limit 0).
        if seq in gate_keys or c['kind'] == 'chromeForward':
            base = get(b, 'longestNonNormalRun')
            g.check(seq, 'longestNonNormalRun', get(c, 'longestNonNormalRun'), base,
                    None if base is None else lim['ratio'] * base, '<=')
    return g


def gate_g5(prereg, timing):
    g, lim = Gate(), prereg['limits']
    if timing is None:
        g.block('timing', 'timing.json', 'no timing file given')
        return g
    seqs = timing.get('sequences', {})
    # A repetition that exited non-zero or stalled is the candidate's fault (FAIL); one whose exit
    # status is unknown cannot be judged (BLOCKED).
    if timing.get('badRuns') is None or timing.get('unknownExit') is None:
        g.block('timing', 'badRuns', 'timing.json has no badRuns/unknownExit')
    else:
        for name in timing['unknownExit']:
            g.block(name, 'exit', 'no exit status in the timing status.jsonl')
        for text in timing['badRuns']:
            g.checks.append({'sequence': text, 'metric': 'badRun', 'value': text, 'e0': None,
                             'limit': None, 'op': None, 'pass': False})
        g.check('timing', 'badRuns', len(timing['badRuns']), None, 0, '==')

    def median(seq, arm):
        return ((seqs.get(seq) or {}).get(arm) or {}).get('median')

    for seq, spec in prereg['sequences']['timing'].items():
        on, off = median(seq, 'on'), median(seq, 'off')
        base, label = off, 'on<=timingRatio*off'
        if 'envelope' in spec:
            env = median(spec['envelope'], 'off')
            base = None if off is None or env is None else max(off, env)
            label = f"on<=timingRatio*max(off, off[{spec['envelope']}])"
        g.check(seq, label, on, base, None if base is None else lim['timingRatio'] * base, '<=')
        # Cost of the map-lost frames: median over the ON repetitions of their p95 (0 ms without
        # lost frames); None (a missing repetition) is BLOCKED.
        lost = ((seqs.get(seq) or {}).get('on') or {}).get('lostP95Median')
        g.check(seq, 'lostP95Median(on)<=timingRatio*lostFrameP95EnvelopeMs', lost,
                need(lim, 'lostFrameP95EnvelopeMs'),
                lim['timingRatio'] * need(lim, 'lostFrameP95EnvelopeMs'), '<=')
    return g


def gate_g6(prereg, cand, compare, config):
    if 'K2' not in config:
        return None
    g, lim = Gate(), prereg['limits']
    if compare is None:
        g.block('compare', 'metrics', 'no --compare metrics given')
        return g
    passed, blocked = 0, False
    for seq in prereg['sequences']['k2Benefit']:
        v = get(cand['runs'].get(seq), 'eSpeed')
        base = get(compare['runs'].get(seq), 'eSpeed')
        limit = None if base is None else lim['k2BenefitRatio'] * base
        if v is None or limit is None:
            blocked = True
        elif v <= limit:
            passed += 1
        g.check(seq, 'eSpeed', v, base, limit, '<=')
    # The per-sequence comparisons are informational; the gate is the count of passing sequences.
    for c in g.checks:
        c['informational'] = True
    if blocked:
        g.block('k2Benefit', 'count', 'an eSpeed needed for the comparison is missing')
    else:
        g.check('k2Benefit', 'sequencesBeatingCompare', passed, None, lim['k2BenefitMinSequences'], '>=')
    return g


# ---------------------------------------------------------------- freeze and binding

BUNDLE_FILES = ('sensors.tsv', 'frames.tsv', 'engine-config.tsv')
ADDENDUM_KEYS = ('schema', 'configurations', 'note')


def all_sequence_names(prereg):
    """Every run name of the prereg (including the timing sequences), in order, each once."""
    names = [n for n, _ in run_names(prereg)]
    return names + [n for n in prereg['sequences'].get('timing', {}) if n not in names]


def bundle_dir_of(prereg, name):
    """Bundle of a sequence: gate bundle for ob-/fw-, else the source bundle of the fixture."""
    if name.startswith(('ob-', 'fw-')):
        return repo_path(prereg['bundles']) / name
    return repo_path(prereg['sourceBundles']) / prereg['bundleOf'][name]


def bundle_hashes(prereg, name):
    d = bundle_dir_of(prereg, name)
    return {f: sha256_file(d / f) if (d / f).is_file() else None for f in BUNDLE_FILES}


def file_entry(path):
    return {'path': str(path), 'sha256': sha256_file(path)}


def utc_now():
    return datetime.datetime.now(datetime.timezone.utc).isoformat(timespec='seconds')


def cmd_freeze(args):
    if pathlib.Path(args.out).exists():
        raise SystemExit(f'error: {args.out} exists; refusing to overwrite')
    prereg = load_json(args.prereg)
    out = {'schema': 'xr-commercial-freeze-v1', 'createdAt': utc_now(),
           'prereg': file_entry(args.prereg), 'e0': file_entry(args.e0),
           'addenda': [file_entry(a) for a in args.addendum],
           'bundles': {n: bundle_hashes(prereg, n) for n in all_sequence_names(prereg)},
           'tools': {t: sha256_file(repo_path(t)) for t in args.tools}}
    missing = [n for n, h in out['bundles'].items() if None in h.values()]
    if missing:
        raise SystemExit(f'error: bundle files missing for {missing}')
    write_json_exclusive(args.out, out)
    print(f'froze {len(out["bundles"])} bundles, {len(out["tools"])} tools, {len(out["addenda"])} addenda')
    return 0


def cmd_freeze_add(args):
    if pathlib.Path(args.out).exists():
        raise SystemExit(f'error: {args.out} exists; refusing to overwrite')
    old = load_json(args.freeze)
    new = copy.deepcopy(old)
    new['addenda'] = list(old.get('addenda', [])) + [file_entry(args.addendum)]
    new['previousFreezeSha256'] = sha256_file(args.freeze)
    new['createdAt'] = utc_now()
    write_json_exclusive(args.out, new)
    return 0


def configurations_of(prereg, addenda_files):
    """(configurations, problems): prereg configurations plus those the addenda ADD.

    An addendum may only carry the keys schema/configurations/note and must not redefine a name."""
    configs, problems = dict(prereg.get('configurations') or {}), []
    for path in addenda_files:
        add = load_json(path)
        extra = sorted(set(add) - set(ADDENDUM_KEYS))
        if extra:
            problems.append(f'addendum {path} has other keys: {extra}')
            continue
        for name, env in (add.get('configurations') or {}).items():
            if name in configs:
                problems.append(f'addendum {path} redefines configuration {name}')
            else:
                configs[name] = env
    return configs, problems


def gate_binding(args, prereg, e0, cand, g0, timing, freeze):
    """Hashes, identities and attempt counts that bind the verdict to the frozen inputs."""
    g = Gate()

    def require(what, ok, reason):
        """Any binding violation is BLOCKED (never FAIL): the inputs are not the frozen ones."""
        if ok:
            g.checks.append({'sequence': what, 'metric': 'binding', 'value': True, 'e0': None,
                             'limit': True, 'op': '==', 'pass': True})
        else:
            g.block(what, 'binding', reason)

    def eq(what, got, want):
        require(what, got is not None and got == want, f'mismatch or missing (got {got!r}, want {want!r})')

    eq('prereg sha256', sha256_file(args.prereg), (freeze.get('prereg') or {}).get('sha256'))
    eq('e0 sha256', sha256_file(args.e0), (freeze.get('e0') or {}).get('sha256'))
    addenda = []
    for a in freeze.get('addenda') or []:
        path = repo_path(a['path'])
        if not path.is_file():
            g.block(f"addendum {a['path']}", 'binding', 'file missing')
        else:
            addenda.append(path)
            eq(f"addendum {a['path']}", sha256_file(path), a.get('sha256'))
    for tool, want in (freeze.get('tools') or {}).items():
        path = repo_path(tool)
        eq(f'tool {tool}', sha256_file(path) if path.is_file() else None, want)
    frozen_bundles = freeze.get('bundles') or {}
    for name in all_sequence_names(prereg):
        if name not in frozen_bundles:
            g.block(f'bundle {name}', 'binding', 'not in freeze.bundles')
            continue
        have = bundle_hashes(prereg, name)
        for f in BUNDLE_FILES:
            eq(f'bundle {name}/{f}', have[f], frozen_bundles[name].get(f))
    eq('e0.replaySha256 == prereg e0.replayBinarySha256', e0.get('replaySha256'),
       (prereg.get('e0') or {}).get('replayBinarySha256'))
    eq('metrics.config', cand.get('config'), args.config)
    configs, problems = configurations_of(prereg, addenda)
    for text in problems:
        g.block('configurations', 'binding', text)
    if args.config not in configs:
        g.block('configurations', 'binding', f'{args.config} is not defined by the prereg or an addendum')
    else:
        eq('metrics.accuracyEnv == configuration env', cand.get('accuracyEnv'), configs[args.config])
    eq('metrics.replaySha256 == g0.identity.replaySha256', cand.get('replaySha256'),
       ((g0 or {}).get('identity') or {}).get('replaySha256'))
    if timing is not None:
        eq('metrics.replaySha256 == timing.replaySha256', cand.get('replaySha256'), timing.get('replaySha256'))
        eq('timing.accuracyOff == ""', timing.get('accuracyOff'), '')
        if args.config in configs:
            eq('timing.accuracyOn == configuration env', timing.get('accuracyOn'), configs[args.config])
    ident = (g0 or {}).get('identity') or {}
    try:
        evidence = load_json(repo_path(ident['evidence']))
    except (KeyError, OSError, ValueError, TypeError):
        evidence = None
    require('g0.identity.evidence', isinstance(evidence, dict), 'identity evidence is not a readable JSON file')
    if isinstance(evidence, dict):
        eq('identity evidence replaySha256 == g0.identity.replaySha256', evidence.get('replaySha256'),
           ident.get('replaySha256'))
        eq('identity evidence replaySha256 == metrics.replaySha256', evidence.get('replaySha256'),
           cand.get('replaySha256'))
        require('identity evidence pass', ident.get('pass') is not True or evidence.get('pass') is True,
                'g0 says identity passed but its evidence file does not')
    names = set()
    for line in pathlib.Path(args.attempts).read_text().splitlines():
        if line.strip():
            names.add(json.loads(line).get('config'))
    names.discard(None)
    others = names - {'OFF'}
    limit = need(prereg['limits'], 'maxConfigurations')
    require('attempts', len(others) <= limit, f'{len(others)} configurations other than OFF, limit {limit}')
    require('attempts', args.config in names, f'{args.config} is not listed in the attempts file')
    return g


def cmd_verdict(args):
    if pathlib.Path(args.out).exists():
        raise SystemExit(f'error: {args.out} exists; refusing to overwrite')
    prereg, e0, cand = load_json(args.prereg), load_json(args.e0), load_json(args.metrics)
    g0 = load_json(args.g0)
    timing = load_json(args.timing) if args.timing else None
    compare = load_json(args.compare) if args.compare else None
    freeze = load_json(args.freeze)
    gates = {'BINDING': gate_binding(args, prereg, e0, cand, g0, timing, freeze), 'G0': gate_g0(g0), 'G1': gate_g1(prereg, e0, cand), 'G2': gate_g2(prereg, e0, cand),
             'G3': gate_g3(prereg, e0, cand), 'G4': gate_g4(prereg, e0, cand),
             'G5': gate_g5(prereg, timing), 'G6': gate_g6(prereg, cand, compare, args.config)}
    out_gates = {}
    for name, gate in gates.items():
        if gate is None:
            out_gates[name] = {'status': NA, 'checks': []}
        else:
            out_gates[name] = gate.as_dict()
    # A candidate that provably fails a gate is rejected even when other gates could not be measured;
    # only unbound inputs (BINDING not PASS) make every statement void.
    gate_statuses = [g['status'] for n, g in out_gates.items() if n != 'BINDING' and g['status'] != NA]
    if out_gates['BINDING']['status'] != PASS:
        overall = 'BLOCKED'
    elif FAIL in gate_statuses:
        overall = 'REJECT'
    elif BLOCKED in gate_statuses:
        overall = 'BLOCKED'
    else:
        overall = 'ADOPT'
    out = {'schema': VERDICT_SCHEMA, 'config': args.config,
           'preregSha256': sha256_file(args.prereg), 'e0Sha256': sha256_file(args.e0),
           'metricsSha256': sha256_file(args.metrics),
           'timingSha256': sha256_file(args.timing) if args.timing else None,
           'freezeSha256': sha256_file(args.freeze), 'attemptsSha256': sha256_file(args.attempts),
           'gates': out_gates, 'overall': overall}
    write_json_exclusive(args.out, out)
    print(f'{args.config}: {overall}')
    for name, g in out_gates.items():
        bad = [f"{c['sequence']}:{c['metric']}" + (f" ({c['reason']})" if c.get('reason') else '')
               for c in g['checks'] if c['pass'] is not True and not c.get('informational')]
        print(f'  {name} {g["status"]:15s} {"; ".join(bad[:6])}' + (' ...' if len(bad) > 6 else ''))
    return 0


# ---------------------------------------------------------------- summary

def _cell(v, spec='8.3f'):
    return format(v, spec) if isinstance(v, (int, float)) and not isinstance(v, bool) else format('n/a', spec.split('.')[0])


def _ratio(v, base):
    if v is None or not base:
        return '    n/a'
    return f'{v / base:7.2f}'


def cmd_summary(args):
    e0 = load_json(args.e0)
    cands = [load_json(p) for p in args.metrics]
    tables = [('outback', 'eStart', lambda r: r.get('eStart')),
              ('outback', 'eAll', lambda r: r.get('eAll')),
              ('outback', 'discards', lambda r: r.get('discards')),
              ('forwardSpeed', 'eSpeed', lambda r: r.get('eSpeed')),
              ('fixtures', 'sim3AteRmse', lambda r: (r.get('accuracy') or {}).get('sim3AteRmse')),
              ('fixtures', 'apeRotDegRmse', lambda r: (r.get('accuracy') or {}).get('apeRotDegRmse')),
              ('fixtures', 'normalRatio', lambda r: r.get('normalRatio')),
              ('chromeForward', 'normalRatio', lambda r: r.get('normalRatio')),
              ('outback', 'mapLost', lambda r: r.get('mapLost')),
              ('forwardSpeed', 'mapLost', lambda r: r.get('mapLost')),
              ('fixtures', 'mapLost', lambda r: r.get('mapLost')),
              ('chromeForward', 'mapLost', lambda r: r.get('mapLost')),
              ('fixtures', 'longestNonNormalRun', lambda r: r.get('longestNonNormalRun')),
              ('chromeForward', 'longestNonNormalRun', lambda r: r.get('longestNonNormalRun')),
              ('outback', 'path', lambda r: r.get('path')),
              ('outback', 'rotStartDeg', lambda r: r.get('rotStartDeg'))]
    for kind, metric, pick in tables:
        names = [n for n, r in e0['runs'].items() if r['kind'] == kind]
        print(f'\n{kind} / {metric}')
        print(f'{"run":14s} {"E0":>8s} ' + ' '.join(f'{c["config"]:>8s} {"ratio":>7s}' for c in cands))
        for n in names:
            base = None if e0['runs'][n].get('missing') else pick(e0['runs'][n])
            cells = []
            for c in cands:
                r = c['runs'].get(n)
                v = None if r is None or r.get('missing') else pick(r)
                cells.append(f'{_cell(v)} {_ratio(v, base)}')
            print(f'{n:14s} {_cell(base)} ' + ' '.join(cells))
    return 0


# ---------------------------------------------------------------- CLI

def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    sub = ap.add_subparsers(dest='cmd', required=True)
    p = sub.add_parser('collect')
    p.add_argument('--prereg', required=True)
    p.add_argument('--runs', required=True)
    p.add_argument('--config', required=True)
    p.add_argument('--e0-runs', required=True)
    add_parser_output(p)
    p.set_defaults(fn=cmd_collect)
    p = sub.add_parser('identity')
    p.add_argument('--e0-runs', required=True)
    p.add_argument('--runs', required=True)
    add_parser_output(p)
    p.set_defaults(fn=cmd_identity)
    p = sub.add_parser('timing')
    p.add_argument('--prereg', required=True)
    p.add_argument('--runs', required=True)
    add_parser_output(p)
    p.set_defaults(fn=cmd_timing)
    p = sub.add_parser('verdict')
    p.add_argument('--prereg', required=True)
    p.add_argument('--e0', required=True)
    p.add_argument('--metrics', required=True)
    p.add_argument('--config', required=True)
    p.add_argument('--g0', required=True)
    p.add_argument('--freeze', required=True)
    p.add_argument('--attempts', required=True)
    p.add_argument('--timing')
    p.add_argument('--compare')
    add_parser_output(p)
    p.set_defaults(fn=cmd_verdict)
    p = sub.add_parser('summary')
    p.add_argument('--e0', required=True)
    p.add_argument('--metrics', nargs='+', required=True)
    p.set_defaults(fn=cmd_summary)
    p = sub.add_parser('freeze')
    p.add_argument('--prereg', required=True)
    p.add_argument('--e0', required=True)
    p.add_argument('--tools', nargs='+', required=True)
    p.add_argument('--addendum', action='append', default=[])
    add_parser_output(p)
    p.set_defaults(fn=cmd_freeze)
    p = sub.add_parser('freeze-add')
    p.add_argument('--freeze', required=True)
    p.add_argument('--addendum', required=True)
    add_parser_output(p)
    p.set_defaults(fn=cmd_freeze_add)
    args = ap.parse_args(argv)
    return args.fn(args)


if __name__ == '__main__':
    sys.exit(main())

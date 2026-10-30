#!/usr/bin/env python3
"""Original reference runtime memory probe (spec 03 §8.4): loading, per-run metrics, validation, gate rules.

A probe run is a run-reference.mjs --memprobe directory: trace.jsonl (public trace, same schema as r1/r2) plus
memprobe.jsonl.gz (line 1 header with `fields`, then one object per c8EmAsm_processStagedFrame call; `i` = input
frame index of the fresh callback that reported the call, `t` its fixture time; see runtime/memprobe.js).

Gate rules (thresholds-v1.json G-M3(b), G-M4, G-M5, G-M7; registered text, mechanics made explicit here before
any probe result was seen):
  * reference = the two probe runs p1, p2 (the probe runs replace r1/r2 for these gates: the original is run-to-run
    nondeterministic, parity.md §3.1, and r1/r2 carry no memory observation);
  * "+-X% or +-2 x run difference, whichever larger": centre = mean(p1, p2), tol = max(X * centre, 2 * |p1 - p2|),
    PASS iff |engine - centre| <= tol;
  * G-M3(b): keyframes created in processed frames with t - t(first trace row) <= 30 s, X = 25%;
  * G-M4: per type 1/2/3, median (sorted[n // 2], the engine evaluator's convention) of the live point count
    (state != retired) over fresh frames, X = 30%; all three types must pass;
  * G-M5: MAP_RESET count over the run in [min(p1, p2) - 1, max(p1, p2) + 1];
  * G-M7 (flip6-s2): median live type-3 count, same rule as G-M4. Recorded alongside, not part of the verdict:
    original sky points created per keyframe (median; the engine trace has no per-keyframe creation count) and
    whether original frm+0x98 == 0 on every probed frame (D-7 / D-22).

CLI (repo root):
  python3 -I slam_refact/tools/harness/eval/memprobe.py --run tum-room1-full=<probe run dir> ... \
      --reference-runs build/slam-refact-20261015/reference/runs --out-json <new file>
writes per-run metrics + validation checks + representativeness vs r1/r2.
"""
import argparse
import gzip
import json
import pathlib
import sys

REPO = pathlib.Path(__file__).resolve().parents[4]
sys.path.insert(0, str(REPO / 'scripts/dev'))
from reference_metadata import matches_reference_schema  # noqa: E402

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import trace_metrics as tm  # noqa: E402
from common import load_trace  # noqa: E402

MAX_KEYFRAMES = 64
KEYFRAME_COOLDOWN = 5


def load_probe(run_dir):
    """Returns (header, records) with records in processStagedFrame order."""
    with gzip.open(pathlib.Path(run_dir) / 'memprobe.jsonl.gz', 'rt') as f:
        lines = [json.loads(line) for line in f if line.strip()]
    if not lines or not matches_reference_schema(lines[0].get('schema'), 'slam-refact-xr_reference-memprobe-v1', REPO):
        raise ValueError('not a memprobe file: ' + str(run_dir))
    return lines[0], sorted(lines[1:], key=lambda r: r['seq'])


def assign_times(records):
    """t for every record: its own (associated) t, else the t of the next associated record (a call whose callback
    was a repeat or missing is attributed to the next reported frame). Records after the last association keep None."""
    out, nxt = [None] * len(records), None
    for k in range(len(records) - 1, -1, -1):
        if records[k].get('t') is not None:
            nxt = records[k]['t']
        out[k] = nxt
    return out


def median_floor(values):
    s = sorted(values)
    return s[len(s) // 2] if s else None


def live(r, k):
    p = r.get('pts%d' % k)
    return None if p is None else p[2]


def run_metrics(records, t0):
    """Per-run observables used by the gates. t0 = t of the first trace row (engine evaluator convention)."""
    times = assign_times(records)
    rooted = [r for r in records if r.get('mbFrameCount') is not None]
    kf = [(r, t) for r, t in zip(records, times) if r.get('outKeyframeCreated')]
    fresh = [r for r in records if r.get('i') is not None and r.get('pts1') is not None]
    created_30s = sum(1 for _, t in kf if t is not None and t - t0 <= 30.0)
    sky_created = [r['newSky'][0] for r, _ in kf if r.get('newSky') is not None]
    frm = [r['frmWorldMatches'] for r in rooted if r.get('frmWorldMatches') is not None]
    return {
        'records': len(records), 'rooted_records': len(rooted), 'fresh_probed_frames': len(fresh),
        'unassociated_records': sum(1 for r in records if r.get('i') is None),
        'map_resets': sum(1 for r in records if r.get('outMapReset')),
        'first_map_reset_i': next((r['i'] for r in records if r.get('outMapReset')), None),
        'keyframes_created': len(kf), 'keyframes_created_first_30s': created_30s,
        'max_live_keyframes': max((r['kfLive'] for r in rooted if r.get('kfLive') is not None), default=None),
        'median_live_type_1_2_3': [median_floor([live(r, k) for r in fresh]) for k in (1, 2, 3)],
        'median_tenured_type_1_2_3': [median_floor([r['pts%d' % k][3] for r in fresh]) for k in (1, 2, 3)],
        'median_sky_created_per_keyframe': median_floor(sky_created),
        'sky_created_per_keyframe_mean': (sum(sky_created) / len(sky_created)) if sky_created else None,
        'frm_world_matches_nonzero_frames': sum(1 for v in frm if v != 0), 'frm_world_matches_probed_frames': len(frm),
    }


def keyframe_intervals(records):
    """Intervals in MapBuilder frame_count_ between consecutive keyframes of one engine instance, excluding
    intervals that end on a MAP_RESET (spec 03 §3: a reset always creates a keyframe)."""
    out, last_kf, last_fc = [], None, None
    for r in records:
        fc = r.get('mbFrameCount')
        if fc is None:
            continue
        if last_fc is not None and fc <= last_fc:  # engine re-created: frame_count_ restarted
            last_kf = None
        last_fc = fc
        if r.get('outKeyframeCreated'):
            if last_kf is not None and not r.get('outMapReset'):
                out.append(fc - last_kf)
            last_kf = fc
    return out


def validate(records, trace_rows, header=None):
    """Probe-internal invariants and cross-checks against the public trace of the SAME run."""
    rooted = [r for r in records if r.get('mbFrameCount') is not None]
    by_i = {r['i']: r for r in trace_rows}
    checks = {}
    fw = tm.first_world_points(trace_rows)
    first_reset = next((r for r in records if r.get('outMapReset')), None)
    checks['first_reset_vs_first_world_points'] = {
        'probe_first_reset_i': first_reset and first_reset['i'], 'trace_first_world_points_i': fw and fw['i'],
        'probe_new_points_at_reset': first_reset and first_reset['outNewPoints'], 'trace_world_points_at_that_frame': fw and fw['count'],
        'ok': bool(first_reset and fw and first_reset['i'] == fw['i'])}
    iv = keyframe_intervals(records)
    checks['keyframe_interval_ge_5'] = {'intervals': len(iv), 'violations': [v for v in iv if v < KEYFRAME_COOLDOWN][:20],
                                        'share_5_6': (sum(v in (5, 6) for v in iv) / len(iv)) if iv else None,
                                        'ok': all(v >= KEYFRAME_COOLDOWN for v in iv)}
    resets = [r for r in records if r.get('outMapReset')]
    checks['reset_implies_keyframe'] = {'resets': len(resets), 'ok': all(r.get('outKeyframeCreated') for r in resets)}
    checks['reset_sets_last_reset_frame'] = {'ok': all(r['mbLastResetFrame'] == r['mbFrameCount'] for r in resets)}
    mx = max((r['kfLive'] for r in rooted if r.get('kfLive') is not None), default=None)
    checks['live_keyframes_le_64'] = {'max': mx, 'ok': mx is not None and mx <= MAX_KEYFRAMES}
    bad_pool, bad_kf = 0, 0
    for r in rooted:
        if r.get('kfSlots') is None:
            continue
        if r['kfSlots'] - r['kfFree'] != r['kfLive']:
            bad_kf += 1
        for k in range(1, 5):
            p = r['pts%d' % k]
            if p[0] - p[1] != p[2]:
                bad_pool += 1
    checks['free_list_consistency'] = {'pool_mismatch_rows': bad_pool, 'keyframe_mismatch_rows': bad_kf, 'ok': bad_pool == 0 and bad_kf == 0}
    steps, root_gaps, restarts, other = 0, 0, 0, 0
    for a, b in zip(rooted, rooted[1:]):
        d = b['mbFrameCount'] - a['mbFrameCount']
        if b['seq'] != a['seq'] + 1:
            root_gaps += 1  # calls without a valid root in between (engine re-creation window)
        elif d == 1:
            steps += 1
        elif d <= 0:
            restarts += 1  # engine re-created (frame_count_ restarts)
        else:
            other += 1
    checks['frame_count_increments_per_call'] = {'consecutive_plus_1': steps, 'restarts': restarts, 'root_gaps': root_gaps,
                                                 'other': other, 'ok': other == 0}
    kfr = [r for r in records if r.get('outKeyframeCreated') and r.get('newSky') is not None]
    created_match = sum(1 for r in kfr if r['newGround'][0] + r['newSky'][0] == r['outNewPoints'])
    checks['new_points_vs_out_vector'] = {'keyframes': len(kfr), 'match': created_match, 'ok': created_match == len(kfr)}
    kf_frames = [r['i'] for r in records if r.get('outKeyframeCreated') and r.get('i') is not None]
    burst = sum(1 for i in kf_frames if (by_i.get(i, {}).get('wpAdded') or 0) >= 10)
    checks['keyframe_vs_public_burst'] = {'keyframe_frames': len(kf_frames), 'with_wpAdded_ge_10': burst, 'ok': None,
                                         'note': 'informational: public world points are visibility-filtered (S3 §8.3)'}
    agree = n = 0
    for r in records:
        row = by_i.get(r.get('i'))
        if row is None or not row.get('fresh') or r.get('trkState') is None:
            continue
        n += 1
        agree += (row['status'] == 'NORMAL') == (r['trkState'] == 1)
    checks['trk_state_vs_public_status'] = {'frames': n, 'normal_iff_state1_share': agree / n if n else None, 'ok': None,
                                           'note': 'informational: status also depends on the VIO loss state machine (spec 04 §6.1)'}
    out = {'checks': checks, 'all_hard_checks_ok': all(c['ok'] for c in checks.values() if c['ok'] is not None)}
    if header is not None:
        out['root'] = {'scans': len(header.get('scans', [])), 'history': header.get('rootHistory'), 'error': header.get('error'),
                       'wasm_sha256': header.get('wasmSha256')}
    return out


def representativeness(rows):
    """Trace-level observables of one run (same functions as reference_baseline.run_observables)."""
    return {'rows': len(rows), 'fresh': sum(1 for r in rows if r['fresh']), 'first_translation_i': tm.first_translation(rows),
            'first_limited': tm.first_limited(rows), 'limited_ratio': tm.limited_ratio(rows),
            'world_points_p50': tm.world_points_p50(rows), 'first_world_points': tm.first_world_points(rows),
            'early_burst_share_5_6': tm.early_burst_share_5_6(rows)['share_5_6']}


# --------------------------------------------------------------------------------------------- gate rules
def rel_or_diff(engine, p1, p2, rel):
    centre = (p1 + p2) / 2.0
    tol = max(rel * centre, 2.0 * abs(p1 - p2))
    info = {'engine': engine, 'reference': [p1, p2], 'centre': centre, 'tol': tol, 'window': [centre - tol, centre + tol],
            # Reporting only (rule unchanged): a window reaching below 0 accepts every count up to its upper bound.
            'non_discriminating_below': centre - tol < 0}
    if engine is None:
        return 'NOT-APPLICABLE', dict(info, note='engine trace lacks the value')
    return ('PASS' if abs(engine - centre) <= tol else 'FAIL'), info


def reset_window(engine, p1, p2):
    lo, hi = min(p1, p2) - 1, max(p1, p2) + 1
    if engine is None:
        return 'NOT-APPLICABLE', {'engine': None, 'reference': [p1, p2], 'window': [lo, hi], 'note': 'engine trace lacks the value'}
    return ('PASS' if lo <= engine <= hi else 'FAIL'), {'engine': engine, 'reference': [p1, p2], 'window': [lo, hi]}


class ProbeReference:
    """The two probe runs of one scenario (paths), loaded lazily."""

    def __init__(self, dirs):
        if len(dirs) != 2:
            raise ValueError('a memprobe reference needs exactly two runs, got %d' % len(dirs))
        self.dirs = [pathlib.Path(d) for d in dirs]
        self._m = None

    def metrics(self):
        if self._m is None:
            self._m = []
            for d in self.dirs:
                _, recs = load_probe(d)
                rows = load_trace(d / 'trace.jsonl')
                self._m.append(run_metrics(recs, rows[0]['t']))
        return self._m


def parse_memprobe_args(items):
    """--memprobe SCENARIO=DIR1,DIR2 -> {scenario: ProbeReference}"""
    out = {}
    for item in items or []:
        key, _, value = item.partition('=')
        dirs = [v for v in value.split(',') if v]
        if not key or len(dirs) != 2:
            raise SystemExit('expected --memprobe SCENARIO=DIR1,DIR2, got ' + item)
        out[key] = ProbeReference(dirs)
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--run', action='append', required=True, help='SCENARIO=probe run dir (repeatable)')
    ap.add_argument('--reference-runs', type=pathlib.Path, help='reference runs root: adds r1/r2 trace observables per scenario')
    ap.add_argument('--out-json', type=pathlib.Path, required=True)
    a = ap.parse_args()
    if a.out_json.exists():
        raise SystemExit('refusing to overwrite ' + str(a.out_json))
    out = {'schema': 'slam-refact-memprobe-summary-v1', 'argv': sys.argv, 'runs': [], 'reference_trace_observables': {}}
    for item in a.run:
        sc, _, d = item.partition('=')
        d = pathlib.Path(d)
        header, recs = load_probe(d)
        rows = load_trace(d / 'trace.jsonl')
        meta = json.loads((d / 'meta.json').read_text())
        out['runs'].append({'scenario': sc, 'dir': str(d), 'label': d.name, 'metrics': run_metrics(recs, rows[0]['t']),
                            'validation': validate(recs, rows, header), 'trace_observables': representativeness(rows),
                            'probe_overhead_ms': meta.get('memprobe', {}).get('probeMs'), 'call_ms': meta.get('memprobe', {}).get('callMs'),
                            'load_avg': {'start': meta.get('loadAvgAtStart'), 'end': meta.get('loadAvgAtEnd')},
                            'status': meta.get('status'), 'wall_seconds': meta.get('wallSeconds')})
        if a.reference_runs and sc not in out['reference_trace_observables']:
            out['reference_trace_observables'][sc] = {lb: representativeness(load_trace(a.reference_runs / sc / lb / 'trace.jsonl'))
                                                      for lb in ('r1', 'r2')}
    a.out_json.write_text(json.dumps(out, indent=1) + '\n')
    print(json.dumps([{'run': r['scenario'] + '/' + r['label'], 'ok': r['validation']['all_hard_checks_ok']} for r in out['runs']]))


if __name__ == '__main__':
    main()
